/**
 * @file LinuxSocketChannel.hpp
 * @brief Linux AF_PACKET cyclic channel — recvfrom + frame-bank RX,
 *        sendto/sendmsg TX.
 *
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#if defined(__linux__)

#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"
#include "raw/CyclicChannelSupport.hpp"
#include "raw/RawWireFormat.hpp"
#include "hal/IEthernet.hpp"
#include "logging/Logger.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>

#include <linux/filter.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

namespace EtherCAT {

static const char* TAG = "cyc_chan";

class LinuxSocketChannel : public ICyclicChannel {
public:
    static constexpr int kBankSize   = 4;
    // Jumbo-sized: the socket backend stages/copies frames into these
    // buffers, so size them to the compile-time ceiling rather than
    // CyclicChannelConfig::frame_size (which sizes the ring backend slots).
    static constexpr int kFrameBytes =
        static_cast<int>(kMaxJumboFrameSize);

    LinuxSocketChannel(int fd, int ifindex) : fd_(fd), ifindex_(ifindex) {}
    ~LinuxSocketChannel() override { if (fd_ >= 0) ::close(fd_); }

    LinuxSocketChannel(const LinuxSocketChannel&) = delete;
    LinuxSocketChannel& operator=(const LinuxSocketChannel&) = delete;

    // ---- TX ----
    uint8_t* txAcquire() override { return tx_buf_; }
    size_t   txCapacity() const override { return sizeof(tx_buf_); }

    bool txCommitFrame(uint32_t frame_len) override {
        struct sockaddr_ll sll{};
        sll.sll_family  = AF_PACKET;
        sll.sll_ifindex = ifindex_;
        sll.sll_halen   = ETH_ALEN;
        std::memcpy(sll.sll_addr, tx_buf_, ETH_ALEN);   // dst MAC from frame
        if (frame_len < 60) frame_len = 60;  // min Ethernet frame (no FCS)
        const ssize_t s = ::sendto(fd_, tx_buf_, frame_len, MSG_DONTWAIT,
                                   reinterpret_cast<struct sockaddr*>(&sll),
                                   sizeof(sll));
        return s == static_cast<ssize_t>(frame_len);
    }

    bool txSendParts(const CyclicTxParts& p) override {
        struct sockaddr_ll sll{};
        sll.sll_family  = AF_PACKET;
        sll.sll_ifindex = ifindex_;
        sll.sll_halen   = ETH_ALEN;
        std::memcpy(sll.sll_addr, p.header, ETH_ALEN);  // dst MAC from header

        const uint16_t wkc_le = Raw::host_to_le16(p.wkc);
        struct iovec iov[3];
        int n = 0;
        iov[n].iov_base = const_cast<uint8_t*>(p.header);
        iov[n].iov_len  = p.header_len; ++n;
        if (p.payload && p.payload_len) {
            iov[n].iov_base = const_cast<uint8_t*>(p.payload);
            iov[n].iov_len  = p.payload_len; ++n;
        }
        iov[n].iov_base = const_cast<uint16_t*>(&wkc_le);
        iov[n].iov_len  = sizeof(wkc_le); ++n;

        struct msghdr msg{};
        msg.msg_name    = &sll;
        msg.msg_namelen = sizeof(sll);
        msg.msg_iov     = iov;
        msg.msg_iovlen  = n;
        const ssize_t s = ::sendmsg(fd_, &msg, MSG_DONTWAIT);
        const ssize_t want = static_cast<ssize_t>(
            p.header_len + (p.payload ? p.payload_len : 0) + sizeof(wkc_le));
        return s == want;
    }

    // ---- RX ----
    int rxPoll(CyclicFrameView* views, int max_views,
               uint32_t timeout_ns) override {
        // Recycle banks emitted before the previous poll that nobody held —
        // the "views valid until next rxPoll unless held" contract.
        for (auto& b : bank_) {
            if (b.emitted &&
                b.holds.load(std::memory_order_acquire) == 0) {
                b.emitted = false;
            }
        }

        int n = drainSocket(views, max_views);
        if (n > 0 || timeout_ns == 0) return n;

        const int r = waitReadable(fd_, timeout_ns);
        if (r <= 0) return r < 0 ? -errno : 0;
        return drainSocket(views, max_views);
    }

    void rxHold(uint32_t cookie) override {
        if (cookie < kBankSize)
            bank_[cookie].holds.fetch_add(1, std::memory_order_relaxed);
    }
    void rxRelease(uint32_t cookie) override {
        // Releasing an unheld cookie is a contract violation — clamp at 0
        // so a double-release can't wedge the bank slot forever.
        if (cookie < kBankSize &&
            bank_[cookie].holds.load(std::memory_order_acquire) > 0)
            bank_[cookie].holds.fetch_sub(1, std::memory_order_acq_rel);
    }

    // ---- introspection ----
    int  fd() const override { return fd_; }
    bool zeroCopy() const override { return false; }
    const char* backendName() const override { return "socket"; }
    uint64_t droppedRx() const override { return dropped_rx_; }

    uint64_t kernelRxDrops() override {
        // PACKET_STATISTICS resets on read — accumulate the observed
        // deltas so callers get a monotonic share of the true total.
        struct tpacket_stats st{};
        socklen_t len = sizeof(st);
        if (::getsockopt(fd_, SOL_PACKET, PACKET_STATISTICS,
                         &st, &len) == 0)
            kernel_rx_drops_ += st.tp_drops;
        return kernel_rx_drops_;
    }

protected:
    struct Bank {
        uint8_t buf[kFrameBytes];
        std::atomic<int> holds{0};
        bool emitted = false;
    };

    Bank* freeBank() {
        for (auto& b : bank_) {
            if (!b.emitted &&
                b.holds.load(std::memory_order_acquire) == 0) return &b;
        }
        return nullptr;
    }

    /// recvmsg() drain: kernel RX timestamp via SCM_TIMESTAMPNS cmsg when
    /// the socket provides it (SO_TIMESTAMPNS/PACKET_TIMESTAMP), else a
    /// monotonic userspace stamp.
    int drainSocket(CyclicFrameView* views, int max_views) {
        int n = 0;
        while (n < max_views) {
            Bank* b = freeBank();
            if (!b) { ++dropped_rx_; break; }
            alignas(8) uint8_t cbuf[64];
            struct iovec   iov { b->buf, sizeof(b->buf) };
            struct msghdr  msg {};
            msg.msg_iov        = &iov;
            msg.msg_iovlen     = 1;
            msg.msg_control    = cbuf;
            msg.msg_controllen = sizeof(cbuf);
            const ssize_t len = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
            if (len <= 0) break;                      // drained
            uint64_t stamp = 0;
            for (struct cmsghdr* c = CMSG_FIRSTHDR(&msg); c;
                 c = CMSG_NXTHDR(&msg, c)) {
                if (c->cmsg_level == SOL_SOCKET &&
                    c->cmsg_type == SCM_TIMESTAMPNS) {
                    const auto* ts =
                        reinterpret_cast<const struct timespec*>(CMSG_DATA(c));
                    stamp = static_cast<uint64_t>(ts->tv_sec) *
                            1'000'000'000ULL +
                            static_cast<uint64_t>(ts->tv_nsec);
                    break;
                }
            }
            b->emitted = true;
            views[n].frame     = b->buf;
            views[n].frame_len = static_cast<uint32_t>(len);
            views[n].stamp_ns  = stamp ? rtStampToMonoNs(stamp)
                                      : monoNowNs();
            views[n].cookie    = static_cast<uint32_t>(b - bank_);
            ++n;
        }
        return n;
    }

    int fd_;
    int ifindex_;
    uint8_t tx_buf_[kFrameBytes]{};
    Bank bank_[kBankSize];
    uint64_t dropped_rx_ = 0;
    uint64_t kernel_rx_drops_ = 0;   // accumulated PACKET_STATISTICS deltas
};

} // namespace EtherCAT

#endif // __linux__

