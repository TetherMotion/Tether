/**
 * @file CyclicChannel_linux.cpp
 * @brief Linux cyclic channels: AF_PACKET socket (+BPF demux) and
 *        PACKET_MMAP TPACKET_V2 rings.
 *
 * Two sockets share the interface:
 *   - socket A (this channel): bound to the EtherCAT ethertype + BPF
 *     accepting only frames whose first datagram idx lies in the reserved
 *     fastpath range (PDO slices 0xE0..0xEF + cyclic slots 0xF8..0xFD).
 *   - socket B (async, owned by the app's poll thread): gets the mirror
 *     filter attached via CyclicChannelConfig::async_fd so it never wakes
 *     for cyclic traffic.
 *
 * The filter is a latency boundary only — correctness never depends on it:
 * any cyclic-idx datagram that still reaches the async path is deposited by
 * parseEtherCATFrame() as before.
 *
 * Ring choice: TPACKET_V2 for both rings.  V2 gives per-frame slots — a slot
 * is freed by clearing tp_status and queued for TX by setting
 * TP_STATUS_SEND_REQUEST — which maps exactly onto the channel's cookie
 * hold/release model.  V3's block packing saves memory for mixed-size
 * traffic but complicates per-frame holds for no benefit at our frame sizes.
 */

#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"

#if defined(__linux__)

#include "logging/Logger.hpp"
#include "raw/RawWireFormat.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/filter.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef SOL_PACKET
#define SOL_PACKET 263
#endif
#ifndef PACKET_IGNORE_OUTGOING
#define PACKET_IGNORE_OUTGOING 23
#endif

namespace EtherCAT {

static const char* TAG = "cyc_chan";

namespace {

// ============================================================================
// BPF demux
// ============================================================================
//
// Frame layout (untagged): [eth 14][ecat hdr 2][dg: cmd|idx|adp|ado|len|irq].
// The first datagram's idx sits at byte offset 17.  Cyclic frames carry only
// reserved idxes; async frames can never get them (allocIdx skips the range)
// — so first-idx demux is exact.  Frames shorter than 18 bytes make the idx
// load fault out-of-bounds — kernel cBPF semantics then reject the packet on
// BOTH sockets (malformed traffic is dropped entirely, which is desirable).
// VLAN-tagged frames show EtherType 0x8100: the program unwraps them
// (inner EtherType at [16:18], idx at 21) so tagged fastpath traffic
// reaches socket A too.

constexpr int  kIdxByteOffset     = 17;   // untagged: first dg idx
constexpr int  kVlanIdxByteOffset = 21;   // 802.1Q-tagged: +4 tag bytes
constexpr int  kVlanInnerEthOff   = 16;   // inner EtherType offset
constexpr uint16_t kEtherCatType  = 0x88A4;
constexpr uint16_t kVlanType      = 0x8100;

static_assert(sizeof(CyclicBpfInsn) == sizeof(struct sock_filter),
              "CyclicBpfInsn must be layout-compatible with sock_filter");

// Socket A / socket B demux program — see cyclicChannelBpfProgram().
// Q15: VLAN-aware — a 0x8100 tag shifts the frame by 4 bytes; the inner
// EtherType lives at [16:18] and the first idx at byte 21.  Non-ECAT and
// VLAN-non-ECAT traffic falls through to the B verdict so socket A can be
// bound to ETH_P_ALL and still see only cyclic EtherCAT.
size_t buildFilterProg(bool accept_cyclic, struct sock_filter* p) {
    // The accept range covers BOTH fastpath pools: user PDO slices
    // (0xE0..0xEF) and cyclic slots (0xF8..0xFD).  Async allocIdx()
    // never reaches 0xE0, so first-idx demux stays exact.
    const uint32_t lo = kSliceSlotBaseIdx;
    const uint32_t hi = kFastSlotEndIdx;
    const struct sock_filter prog[] = {
        /* 0 */ BPF_STMT(BPF_LD | BPF_H | BPF_ABS, 12),              // EtherType
        /* 1 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, kEtherCatType, 0, 2),
        /* 2 */ BPF_STMT(BPF_LD | BPF_B | BPF_ABS, kIdxByteOffset),  // idx
        /* 3 */ BPF_JUMP(BPF_JMP | BPF_JGE | BPF_K, 0, 5, 0),        // → #9
        /* 4 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, kVlanType, 0, 7),
        /* 5 */ BPF_STMT(BPF_LD | BPF_H | BPF_ABS, kVlanInnerEthOff),
        /* 6 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, kEtherCatType, 0, 5),
        /* 7 */ BPF_STMT(BPF_LD | BPF_B | BPF_ABS, kVlanIdxByteOffset),
        /* 8 */ BPF_JUMP(BPF_JMP | BPF_JGE | BPF_K, 0, 0, 0),        // → #9
        /* 9 */ BPF_JUMP(BPF_JMP | BPF_JGE | BPF_K, lo, 0, 2),       // <lo → #12
        /*10 */ BPF_JUMP(BPF_JMP | BPF_JGT | BPF_K, hi, 1, 0),       // >hi → #12
        /*11 */ BPF_STMT(BPF_RET | BPF_K,
                        accept_cyclic ? 0xFFFFFFFFu : 0u),           // A: accept
        /*12 */ BPF_STMT(BPF_RET | BPF_K,
                        accept_cyclic ? 0u : 0xFFFFFFFFu),           // B: accept
    };
    std::memcpy(p, prog, sizeof(prog));
    return sizeof(prog) / sizeof(prog[0]);
}

bool attachFilter(int fd, struct sock_filter* prog, size_t n) {
    struct sock_fprog fp;
    fp.len    = static_cast<unsigned short>(n);
    fp.filter = prog;
    if (setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &fp, sizeof(fp)) < 0)
        return false;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_LOCK_FILTER, &one, sizeof(one)); // best effort
    return true;
}

int openCyclicSocket(int ifindex,
                     const CBPFInsn* accept_prog = nullptr,
                     size_t accept_prog_len = 0) {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) return -1;

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int one = 1;
    // Q18: PACKET_IGNORE_OUTGOING drops own-TX echoes upstream of the BPF.
    // Probe the sockopt result — a kernel that lacks it leaves the async
    // socket seeing its own cyclic frames (correctness unaffected, wakeup
    // cost) — warn so the operator knows.
    if (setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING,
                   &one, sizeof(one)) < 0) {
        TETHER_LOGW(TAG, "PACKET_IGNORE_OUTGOING unsupported ({}) — own "
                    "transmits will echo on the RX path", strerror(errno));
    }
    setsockopt(fd, SOL_PACKET, PACKET_TIMESTAMP, &one, sizeof(one));
    // Kernel RX timestamps arrive via recvmsg SCM_TIMESTAMPNS cmsgs.
    setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one));

    // ETH_P_ALL + the extended demux program: VLAN-tagged EtherCAT frames
    // (0x8100 outer) reach the socket and are filtered by inner ethertype;
    // the program rejects everything non-cyclic kernel-side.
    struct sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll)) < 0) {
        ::close(fd);
        return -1;
    }

    if (accept_prog && accept_prog_len) {
        // Caller-composed program (encap ∧ idx∈fastpath) — replaces the
        // built-in demux so an encapsulation clause isn't dropped.
        if (!CBPFProgramFactory::attach(fd, accept_prog, accept_prog_len)) {
            TETHER_LOGW(TAG, "cyclic composed BPF attach failed: {}",
                        strerror(errno));
        }
    } else {
        struct sock_filter prog[kCyclicBpfInsnCount];
        const size_t n = buildFilterProg(true, prog);
        if (!attachFilter(fd, prog, n)) {
            // Soft failure — the channel still works; crosstalk determinism
            // is lost but correctness is not (parser keys on idx either way).
            TETHER_LOGW(TAG, "cyclic BPF attach failed: {}", strerror(errno));
        }
    }
    return fd;
}

uint64_t monoNowNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

// ppoll on a single fd for POLLIN, timeout in ns → >0 ready, 0 timeout, <0 err
int waitReadable(int fd, uint32_t timeout_ns) {
    struct pollfd pfd { fd, POLLIN, 0 };
    struct timespec ts {
        static_cast<time_t>(timeout_ns / 1'000'000'000u),
        static_cast<long>(timeout_ns % 1'000'000'000u)
    };
    int ret;
    do { ret = ppoll(&pfd, 1, &ts, nullptr); } while (ret < 0 && errno == EINTR);
    return ret;
}

// ============================================================================
// LinuxSocketChannel — recvfrom + frame-bank RX, sendto/sendmsg TX
// ============================================================================

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
            views[n].stamp_ns  = stamp ? stamp : monoNowNs();
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
};

// ============================================================================
// LinuxRingChannel — TPACKET_V2 RX+TX rings on the same socket
// ============================================================================

class LinuxRingChannel : public LinuxSocketChannel {
public:
    struct Config {
        uint32_t rx_blocks  = 128;
        uint32_t tx_blocks  = 16;
        uint32_t rx_spin_ns = 0;   ///< busy-poll window inside rxPoll()
        bool     rx_v3      = false;
        uint32_t rx_v3_retire_us = 10'000;   ///< tp_retire_blk_tov
        uint32_t frame_size = 1600;          ///< per-slot frame bytes
    };

    LinuxRingChannel(int fd, int ifindex, const Config& cfg)
        : LinuxSocketChannel(fd, ifindex), cfg_(cfg) {}

    ~LinuxRingChannel() override { teardownRings(); }

    bool init() {
        // PACKET_VERSION must be set once, before any ring exists — calling
        // it again after a ring is configured returns EBUSY and wedges the
        // subsequent ring setup.
        const int ver = cfg_.rx_v3 ? TPACKET_V3 : TPACKET_V2;
        if (setsockopt(fd_, SOL_PACKET, PACKET_VERSION, &ver, sizeof(ver)) < 0)
            return false;

        // The kernel allocates RX and TX rings as ONE contiguous pg_vec
        // ([RX blocks | TX blocks]); they must be mapped by a single mmap of
        // the combined size at offset 0 — a second mmap for TX is EINVAL.
        // V3 is RX-only: the TX ring keeps the V2 per-frame tpacket_req.
        struct tpacket_req rx_req{}, tx_req{};
        if (!configRxRing(&rx_req)) return false;
        const bool have_tx = configTxRing(&tx_req);   // optional — sendto works
        if (!have_tx)
            TETHER_LOGW(TAG, "TX ring unavailable — cyclic TX uses sendto");

        const size_t rx_len =
            static_cast<size_t>(rx_req.tp_block_size) * rx_req.tp_block_nr;
        const size_t tx_len = have_tx
            ? static_cast<size_t>(tx_req.tp_block_size) * tx_req.tp_block_nr
            : 0;
        void* base = mmap(nullptr, rx_len + tx_len, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd_, 0);
        if (base == MAP_FAILED) {
            struct tpacket_req clear{};
            setsockopt(fd_, SOL_PACKET, PACKET_RX_RING, &clear, sizeof(clear));
            if (have_tx)
                setsockopt(fd_, SOL_PACKET, PACKET_TX_RING, &clear, sizeof(clear));
            return false;
        }

        rx_ring_       = static_cast<uint8_t*>(base);
        rx_ring_len_   = rx_len + tx_len;         // combined mapping length
        // V3: rx_frames_ indexes BLOCKS (tp_frame_nr is 0 under req3).
        rx_frame_size_ = cfg_.rx_v3 ? rx_req.tp_block_size
                                    : rx_req.tp_frame_size;
        rx_frames_     = cfg_.rx_v3 ? rx_req.tp_block_nr
                                    : rx_req.tp_frame_nr;
        rx_holds_      = std::make_unique<std::atomic<int>[]>(rx_frames_);
        rx_consumed_   = std::make_unique<std::atomic<bool>[]>(rx_frames_);
        if (cfg_.rx_v3)
            v3_emitted_ = std::make_unique<std::atomic<uint32_t>[]>(
                              rx_frames_);
        if (have_tx) {
            tx_ring_       = rx_ring_ + rx_len;   // TX region follows RX
            tx_ring_len_   = tx_len;
            tx_frame_size_ = tx_req.tp_frame_size;
            tx_frames_     = tx_req.tp_frame_nr;
        }
        return true;
    }

    /// Test seam: adopt caller-provided ring buffers instead of kernel
    /// PACKET_MMAP rings.  Lets tests exercise the walk/hold/cursor
    /// mechanics on synthetic tpacket2_hdr layouts without CAP_NET_RAW.
    /// The memory is borrowed — teardownRings() does not unmap it.
    void adoptRingsForTest(uint8_t* rx, uint32_t rx_frame_size,
                           uint32_t rx_frames,
                           uint8_t* tx, uint32_t tx_frame_size,
                           uint32_t tx_frames) {
        rx_ring_ = rx;  rx_ring_len_ = 0;
        rx_frame_size_ = rx_frame_size;  rx_frames_ = rx_frames;
        rx_holds_    = std::make_unique<std::atomic<int>[]>(rx_frames_);
        rx_consumed_ = std::make_unique<std::atomic<bool>[]>(rx_frames_);
        tx_ring_ = tx;  tx_ring_len_ = 0;
        tx_frame_size_ = tx_frame_size;  tx_frames_ = tx_frames;
        rings_borrowed_ = true;
    }

    /// V3 test seam: adopt a caller-provided block-mode RX ring
    /// (tpacket_block_desc array).  `rx` holds `blocks` blocks of
    /// `block_size` bytes each; TX is disabled.
    void adoptV3RingForTest(uint8_t* rx, uint32_t block_size,
                            uint32_t blocks) {
        cfg_.rx_v3       = true;
        rx_ring_         = rx;  rx_ring_len_ = 0;
        rx_frame_size_   = block_size;   // V3: stride is the block
        rx_frames_       = blocks;       // V3: rx_frames_ counts blocks
        rx_holds_        = std::make_unique<std::atomic<int>[]>(blocks);
        rx_consumed_     = std::make_unique<std::atomic<bool>[]>(blocks);
        v3_emitted_      = std::make_unique<std::atomic<uint32_t>[]>(blocks);
        tx_ring_         = nullptr;  tx_ring_len_ = 0;
        tx_frame_size_   = 0;  tx_frames_ = 0;
        rings_borrowed_  = true;
    }

    // ---- TX (ring) ----
    uint8_t* txAcquire() override {
        if (!tx_ring_) return LinuxSocketChannel::txAcquire();
        for (uint32_t i = 0; i < tx_frames_; ++i) {
            const uint32_t idx = (tx_cursor_ + i) % tx_frames_;
            auto* hdr = txSlot(idx);
            if (__atomic_load_n(&hdr->tp_status, __ATOMIC_ACQUIRE) ==
                TP_STATUS_AVAILABLE) {
                tx_acquired_ = static_cast<int64_t>(idx);
                tx_cursor_   = (idx + 1) % tx_frames_;   // resume past it
                return txData(hdr);
            }
        }
        return nullptr;
    }

    size_t txCapacity() const override {
        return tx_ring_ ? tx_frame_size_ - kTxDataOff
                        : LinuxSocketChannel::txCapacity();
    }

    bool txCommitFrame(uint32_t frame_len) override {
        if (!tx_ring_) return LinuxSocketChannel::txCommitFrame(frame_len);
        if (tx_acquired_ < 0) return false;   // nothing acquired
        auto* hdr = txSlot(static_cast<uint32_t>(tx_acquired_));
        tx_acquired_ = -1;
        if (frame_len < 60) frame_len = 60;   // min Ethernet frame (no FCS)
        // Without PACKET_TX_HAS_OFF the kernel ignores tp_mac and reads the
        // frame at the fixed offset tp_hdrlen - sizeof(sockaddr_ll); we place
        // the data there (kTxDataOff) and still set tp_mac to match so the
        // layout stays correct if TX_HAS_OFF is ever enabled.
        hdr->tp_mac     = kTxDataOff;
        hdr->tp_net     = hdr->tp_mac + 14;
        hdr->tp_len     = frame_len;
        hdr->tp_snaplen = frame_len;
        // Release store: payload visible before status.
        __atomic_store_n(&hdr->tp_status, TP_STATUS_SEND_REQUEST,
                         __ATOMIC_RELEASE);
        // One kick flushes every queued SEND_REQUEST slot.
        const ssize_t s = ::sendto(fd_, nullptr, 0, MSG_DONTWAIT,
                                   nullptr, 0);
        if (s < 0 && errno == EAGAIN) {
            // Frame stays queued — kernel TX queue is full, it flushes on
            // the next kick.  That is a *late* cyclic frame: count it so
            // the deferral is visible in diagnostics.
            tx_deferred_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        return s >= 0;
    }

    bool txSendParts(const CyclicTxParts& p) override {
        if (!tx_ring_) return LinuxSocketChannel::txSendParts(p);
        uint8_t* dst = txAcquire();
        if (!dst) return false;
        const uint32_t total =
            p.header_len + (p.payload ? p.payload_len : 0) + sizeof(uint16_t);
        if (total > txCapacity()) return false;
        std::memcpy(dst, p.header, p.header_len);
        dst += p.header_len;
        if (p.payload && p.payload_len) {
            std::memcpy(dst, p.payload, p.payload_len);
            dst += p.payload_len;
        }
        const uint16_t wkc_le = Raw::host_to_le16(p.wkc);
        std::memcpy(dst, &wkc_le, sizeof(wkc_le));
        return txCommitFrame(total);
    }

    // ---- RX (ring) ----
    int rxPoll(CyclicFrameView* views, int max_views,
               uint32_t timeout_ns) override {
        int n = cfg_.rx_v3 ? walkRingV3(views, max_views)
                           : walkRing(views, max_views);
        if (n > 0 || timeout_ns == 0) return n;

        // Optional spin phase: poll slot memory directly — a DMA write is
        // visible with zero syscalls and zero scheduler latency.  This is
        // what makes the whole cycle boundary syscall-free on a dedicated
        // core (paired with a HybridSpin deadline timer).  The spin shares
        // the caller's timeout budget, never extends it.
        const uint64_t deadline = monoNowNs() + timeout_ns;
        if (cfg_.rx_spin_ns > 0) {
            const uint64_t stop =
                std::min(deadline, monoNowNs() + cfg_.rx_spin_ns);
            while (monoNowNs() < stop) {
                if (rxPending())
                    return cfg_.rx_v3 ? walkRingV3(views, max_views)
                                      : walkRing(views, max_views);
            }
        }
        const uint64_t now = monoNowNs();
        if (now >= deadline) return 0;

        const int r = waitReadable(fd_,
            static_cast<uint32_t>(deadline - now));
        if (r <= 0) return r < 0 ? -errno : 0;
        return cfg_.rx_v3 ? walkRingV3(views, max_views)
                          : walkRing(views, max_views);
    }

    /// Any surfaced-but-unconsumed slot?  Pure memory reads — spin-safe.
    bool rxPending() const override {
        if (!rx_ring_) return false;
        if (cfg_.rx_v3) {
            for (uint32_t i = 0; i < rx_frames_; ++i) {
                const auto* bd = v3Block(i);
                if ((__atomic_load_n(&bd->hdr.bh1.block_status,
                                     __ATOMIC_ACQUIRE) & TP_STATUS_USER) &&
                    !rx_consumed_[i].load(std::memory_order_acquire))
                    return true;
            }
            return false;
        }
        for (uint32_t i = 0; i < rx_frames_; ++i) {
            const auto* hdr = reinterpret_cast<const struct tpacket2_hdr*>(
                rx_ring_ + static_cast<size_t>(i) * rx_frame_size_);
            if ((__atomic_load_n(&hdr->tp_status, __ATOMIC_ACQUIRE) &
                 TP_STATUS_USER) &&
                !rx_consumed_[i].load(std::memory_order_acquire))
                return true;
        }
        return false;
    }

    void rxHold(uint32_t cookie) override {
        // V3 cookies are (block << 16 | pkt): the hold granularity is the
        // whole block — one held frame pins every frame sharing its block.
        const uint32_t idx = cfg_.rx_v3 ? (cookie >> 16) : cookie;
        if (idx < rx_frames_)
            rx_holds_[idx].fetch_add(1, std::memory_order_relaxed);
    }
    void rxRelease(uint32_t cookie) override {
        const uint32_t idx = cfg_.rx_v3 ? (cookie >> 16) : cookie;
        if (idx >= rx_frames_) return;
        // Clamp at 0 — a double-release must not drive the count negative
        // and pin the ring slot forever.
        if (rx_holds_[idx].load(std::memory_order_acquire) <= 0) return;
        if (rx_holds_[idx].fetch_sub(1, std::memory_order_acq_rel) == 1 &&
            rx_consumed_[idx]) {
            __sync_synchronize();
            if (cfg_.rx_v3)
                v3Retire(idx);
            else
                __atomic_store_n(&rxSlot(idx)->tp_status, TP_STATUS_KERNEL,
                                 __ATOMIC_RELEASE);
        }
    }

    int  fd() const override { return fd_; }
    bool zeroCopy() const override { return true; }
    const char* backendName() const override { return "ring"; }
    uint64_t txDeferred() const override {
        return tx_deferred_.load(std::memory_order_relaxed);
    }

private:
    // Configure PACKET_RX_RING; fills *req for the caller to size the mmap.
    // tp_block_size must be a whole number of pages — pack floor(page /
    // frame_size) frames into each page-sized block.
    bool configRxRing(struct tpacket_req* req) {
        const uint32_t page = (uint32_t)::sysconf(_SC_PAGESIZE);
        if (cfg_.rx_v3) {
            // TPACKET_V3: the ring is an array of tp_block_size blocks;
            // the kernel packs each received frame into the current block
            // and hands the block to userspace (TP_STATUS_USER on the
            // block header) when full or when tp_retire_blk_tov elapses.
            // No fixed frame slots — tp_frame_size/nr stay 0.
            struct tpacket_req3 r3{};
            r3.tp_block_size        = page;
            r3.tp_block_nr          = cfg_.rx_blocks;
            r3.tp_frame_size        = 0;
            r3.tp_frame_nr          = 0;
            r3.tp_retire_blk_tov    = cfg_.rx_v3_retire_us;
            r3.tp_sizeof_priv       = 0;
            r3.tp_feature_req_word  = 0;
            if (setsockopt(fd_, SOL_PACKET, PACKET_RX_RING, &r3,
                           sizeof(r3)) != 0)
                return false;
            // Mirror the sizing fields into *req so init()'s mmap math is
            // version-agnostic.
            req->tp_block_size = r3.tp_block_size;
            req->tp_block_nr   = r3.tp_block_nr;
            return true;
        }
        const uint32_t frame_size =
            TPACKET_ALIGN(TPACKET2_HDRLEN + cfg_.frame_size);
        const uint32_t fpb = frame_size <= page ? page / frame_size : 1;
        req->tp_block_size = fpb * frame_size;
        req->tp_block_size = (req->tp_block_size + page - 1) / page * page;
        req->tp_block_nr   = cfg_.rx_blocks;
        req->tp_frame_size = frame_size;
        req->tp_frame_nr   = fpb * cfg_.rx_blocks;
        if (req->tp_frame_nr == 0) return false;
        return setsockopt(fd_, SOL_PACKET, PACKET_RX_RING, req,
                          sizeof(*req)) == 0;
    }

    bool configTxRing(struct tpacket_req* req) {
        const uint32_t page = (uint32_t)::sysconf(_SC_PAGESIZE);
        const uint32_t frame_size =
            TPACKET_ALIGN(TPACKET2_HDRLEN + cfg_.frame_size);
        const uint32_t fpb = frame_size <= page ? page / frame_size : 1;
        req->tp_block_size = fpb * frame_size;
        req->tp_block_size = (req->tp_block_size + page - 1) / page * page;
        req->tp_block_nr   = cfg_.tx_blocks;
        req->tp_frame_size = frame_size;
        req->tp_frame_nr   = fpb * cfg_.tx_blocks;
        if (req->tp_frame_nr == 0) return false;
        return setsockopt(fd_, SOL_PACKET, PACKET_TX_RING, req,
                          sizeof(*req)) == 0;
    }

    void teardownRings() {
        if (rings_borrowed_) {   // test seam — caller owns the memory
            rx_ring_ = nullptr;
            tx_ring_ = nullptr;
            return;
        }
        struct tpacket_req req{};
        struct tpacket_req3 req3{};
        if (tx_ring_) {
            setsockopt(fd_, SOL_PACKET, PACKET_TX_RING, &req, sizeof(req));
            tx_ring_ = nullptr;   // TX region shares the single RX mapping
        }
        if (rx_ring_) {
            // The clear must use the same req layout the ring was created
            // with — packet_set_ring selects the V3 parser on optlen.
            if (cfg_.rx_v3)
                setsockopt(fd_, SOL_PACKET, PACKET_RX_RING,
                           &req3, sizeof(req3));
            else
                setsockopt(fd_, SOL_PACKET, PACKET_RX_RING,
                           &req, sizeof(req));
            munmap(rx_ring_, rx_ring_len_);   // rx_ring_len_ is the combined size
            rx_ring_ = nullptr;
        }
    }

    struct tpacket2_hdr* rxSlot(uint32_t i) {
        return reinterpret_cast<struct tpacket2_hdr*>(
            rx_ring_ + static_cast<size_t>(i) * rx_frame_size_);
    }
    struct tpacket2_hdr* txSlot(uint32_t i) {
        return reinterpret_cast<struct tpacket2_hdr*>(
            tx_ring_ + static_cast<size_t>(i) * tx_frame_size_);
    }
    // TX frame data lives at tp_hdrlen - sizeof(sockaddr_ll) from the slot
    // start (the kernel's fixed offset for SOCK_RAW when PACKET_TX_HAS_OFF
    // is not enabled).  Equals TPACKET2_HDRLEN - sizeof(sockaddr_ll) = 32.
    static constexpr uint32_t kTxDataOff =
        TPACKET2_HDRLEN - sizeof(struct sockaddr_ll);
    static uint8_t* txData(struct tpacket2_hdr* hdr) {
        return reinterpret_cast<uint8_t*>(hdr) + kTxDataOff;
    }

    int walkRing(CyclicFrameView* views, int max_views) {
        // Single wrap-around pass from the resume cursor: free
        // consumed-and-unheld slots AND emit pending ones in one sweep.
        // (The free check must cover every slot; the emit side resumes at
        // rx_cursor_ so a quiescent poll costs O(active) not O(frames).)
        int n = 0;
        uint32_t last_emitted = rx_cursor_;
        for (uint32_t k = 0; k < rx_frames_; ++k) {
            const uint32_t idx = (rx_cursor_ + k) % rx_frames_;
            auto* hdr = rxSlot(idx);
            if (!(__atomic_load_n(&hdr->tp_status, __ATOMIC_ACQUIRE) &
                  TP_STATUS_USER))
                continue;
            if (rx_consumed_[idx].load(std::memory_order_acquire)) {
                if (rx_holds_[idx].load(std::memory_order_acquire) == 0) {
                    rx_consumed_[idx].store(false, std::memory_order_release);
                    __sync_synchronize();
                    __atomic_store_n(&hdr->tp_status, TP_STATUS_KERNEL,
                                     __ATOMIC_RELEASE);
                }
                continue;
            }
            if (n >= max_views) break;

            views[n].frame     = reinterpret_cast<const uint8_t*>(hdr) +
                                 hdr->tp_mac;
            views[n].frame_len = hdr->tp_snaplen;
            views[n].stamp_ns  =
                static_cast<uint64_t>(hdr->tp_sec) * 1'000'000'000ULL +
                hdr->tp_nsec;
            views[n].cookie    = idx;
            ++n;
            last_emitted = idx;
            // Do NOT clear tp_status here — the consumer may rxHold() this
            // cookie after rxPoll returns.  Consumed-and-unheld slots are
            // recycled on the next walk (here or in rxRelease).
            rx_consumed_[idx].store(true, std::memory_order_release);
        }
        if (n > 0) rx_cursor_ = (last_emitted + 1) % rx_frames_;
        return n;
    }

    // ---- TPACKET_V3 (block-mode RX prototype, Q17) ----
    // Ring = array of tp_block_size blocks; each block begins with a
    // tpacket_block_desc whose bh1 header carries block_status/num_pkts/
    // offset_to_first_pkt.  Frames inside a block are tpacket3_hdr chained
    // by tp_next_offset.  Cookie = (block << 16) | pkt_index — holds are
    // block-granular; a block is retired (one write) only when every frame
    // has been emitted AND all its holds are released.

    struct tpacket_block_desc* v3Block(uint32_t b) const {
        return reinterpret_cast<struct tpacket_block_desc*>(
            rx_ring_ + static_cast<size_t>(b) * rx_frame_size_);
    }

    void v3Retire(uint32_t b) {
        auto* bd = v3Block(b);
        rx_consumed_[b].store(false, std::memory_order_release);
        v3_emitted_[b].store(0, std::memory_order_release);
        __sync_synchronize();
        __atomic_store_n(&bd->hdr.bh1.block_status, TP_STATUS_KERNEL,
                         __ATOMIC_RELEASE);
    }

    int walkRingV3(CyclicFrameView* views, int max_views) {
        int n = 0;
        uint32_t last_done = rx_cursor_;
        uint32_t cut_block = rx_cursor_;
        bool resume_cut = false;
        for (uint32_t k = 0; k < rx_frames_ && n < max_views; ++k) {
            const uint32_t b = (rx_cursor_ + k) % rx_frames_;
            auto* bd = v3Block(b);
            if (!(__atomic_load_n(&bd->hdr.bh1.block_status,
                                  __ATOMIC_ACQUIRE) & TP_STATUS_USER))
                continue;

            if (rx_consumed_[b].load(std::memory_order_acquire)) {
                if (rx_holds_[b].load(std::memory_order_acquire) == 0)
                    v3Retire(b);
                continue;
            }

            // Walk the pkt chain, resuming past frames already emitted.
            const uint32_t num_pkts = bd->hdr.bh1.num_pkts;
            uint32_t emitted = v3_emitted_[b].load(std::memory_order_acquire);
            auto* hdr = reinterpret_cast<struct tpacket3_hdr*>(
                reinterpret_cast<uint8_t*>(bd) +
                bd->hdr.bh1.offset_to_first_pkt);
            for (uint32_t skip = emitted; skip > 0 && hdr; --skip)
                hdr = hdr->tp_next_offset
                    ? reinterpret_cast<struct tpacket3_hdr*>(
                          reinterpret_cast<uint8_t*>(hdr) +
                          hdr->tp_next_offset)
                    : nullptr;
            for (; emitted < num_pkts && hdr && n < max_views; ++emitted) {
                views[n].frame     = reinterpret_cast<const uint8_t*>(hdr) +
                                     hdr->tp_mac;
                views[n].frame_len = hdr->tp_snaplen;
                views[n].stamp_ns  =
                    static_cast<uint64_t>(hdr->tp_sec) * 1'000'000'000ULL +
                    hdr->tp_nsec;
                views[n].cookie    = (b << 16) | emitted;
                ++n;
                hdr = hdr->tp_next_offset
                    ? reinterpret_cast<struct tpacket3_hdr*>(
                          reinterpret_cast<uint8_t*>(hdr) +
                          hdr->tp_next_offset)
                    : nullptr;
            }
            v3_emitted_[b].store(emitted, std::memory_order_release);
            if (emitted >= num_pkts) {
                // Whole block drained — consumed; the block itself retires
                // once every emitted frame's hold is released.  The resume
                // cursor advances past it AFTER the sweep (mutating it
                // mid-loop would corrupt the (cursor + k) iteration).
                rx_consumed_[b].store(true, std::memory_order_release);
                last_done = b;
            } else {
                // max_views cut mid-block — resume the same block next pass.
                resume_cut = true;
                cut_block  = b;
                break;
            }
        }
        if (n > 0)
            rx_cursor_ = resume_cut ? cut_block
                                    : (last_done + 1) % rx_frames_;
        return n;
    }

    Config cfg_;

    std::unique_ptr<std::atomic<uint32_t>[]> v3_emitted_;

    uint8_t* rx_ring_ = nullptr;
    size_t   rx_ring_len_ = 0;
    uint32_t rx_frame_size_ = 0;
    uint32_t rx_frames_ = 0;
    std::unique_ptr<std::atomic<int>[]>  rx_holds_;
    std::unique_ptr<std::atomic<bool>[]> rx_consumed_;

    uint8_t* tx_ring_ = nullptr;
    size_t   tx_ring_len_ = 0;
    uint32_t tx_frame_size_ = 0;
    uint32_t tx_frames_ = 0;
    int64_t  tx_acquired_ = -1;   // index of the last txAcquire() slot
    uint32_t tx_cursor_ = 0;
    uint32_t rx_cursor_ = 0;
    bool     rings_borrowed_ = false;   // adoptRingsForTest() seam
    std::atomic<uint64_t> tx_deferred_{0};
};

} // namespace

// ============================================================================
// Exported filter helpers (usable by tests and the async-socket attach)
// ============================================================================

bool cyclicChannelAttachCyclicFilter(int fd) {
    struct sock_filter prog[kCyclicBpfInsnCount];
    return attachFilter(fd, prog, buildFilterProg(true, prog));
}

bool cyclicChannelAttachAsyncFilter(int fd) {
    struct sock_filter prog[kCyclicBpfInsnCount];
    return attachFilter(fd, prog, buildFilterProg(false, prog));
}

size_t cyclicChannelBpfProgram(bool accept_cyclic,
                               CyclicBpfInsn* out, size_t cap) {
    if (!out || cap < kCyclicBpfInsnCount) return 0;
    struct sock_filter prog[kCyclicBpfInsnCount];
    const size_t n = buildFilterProg(accept_cyclic, prog);
    static_assert(sizeof(prog) == kCyclicBpfInsnCount * sizeof(CyclicBpfInsn));
    std::memcpy(out, prog, sizeof(prog));
    return n;
}

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<ICyclicChannel> createCyclicChannel(
    const CyclicChannelConfig& cfg)
{
    if (cfg.ifindex <= 0) {
        TETHER_LOGE(TAG, "createCyclicChannel: invalid ifindex {}", cfg.ifindex);
        return nullptr;
    }

    // Mirror filter on the async socket — optional and independent of the
    // backend we end up running.  A caller-composed program (encap ∧
    // idx∉fastpath) replaces the built-in mirror so the encapsulation
    // clause survives — SO_ATTACH_FILTER swaps the whole program.
    if (cfg.async_fd >= 0) {
        bool attached;
        if (cfg.async_prog && cfg.async_prog_len) {
            attached = CBPFProgramFactory::attach(
                cfg.async_fd, cfg.async_prog, cfg.async_prog_len);
        } else {
            attached = cyclicChannelAttachAsyncFilter(cfg.async_fd);
        }
        if (attached) {
            TETHER_LOGI(TAG, "async socket: cyclic-idx exclusion BPF attached");
        } else {
            TETHER_LOGW(TAG,
                "async socket: exclusion BPF attach failed ({}) — async "
                "traffic may still see cyclic frames (correctness unaffected)",
                strerror(errno));
        }
    }

    if (cfg.wire_mode != CyclicWireMode::SocketIO) {
        const int fd = openCyclicSocket(cfg.ifindex, cfg.accept_prog,
                                        cfg.accept_prog_len);
        if (fd >= 0) {
            LinuxRingChannel::Config rcfg{cfg.rx_ring_blocks,
                                          cfg.tx_ring_blocks,
                                          cfg.rx_spin_ns,
                                          cfg.rx_tpacket_v3,
                                          cfg.rx_v3_retire_us,
                                          cfg.frame_size};
            auto ring = std::make_unique<LinuxRingChannel>(fd, cfg.ifindex,
                                                           rcfg);
            if (ring->init()) {
                TETHER_LOGI(TAG,
                    "cyclic channel: PACKET_MMAP rings active "
                    "(rx x{} blocks{}, tx x{} blocks, ifindex {})",
                    cfg.rx_ring_blocks,
                    cfg.rx_tpacket_v3 ? " [TPACKET_V3]" : "",
                    cfg.tx_ring_blocks, cfg.ifindex);
                return ring;
            }
            // ring dtor closes fd
        }
        if (cfg.wire_mode == CyclicWireMode::PacketRing) {
            TETHER_LOGE(TAG, "cyclic channel: PACKET_RING requested but "
                        "setup failed ({})", strerror(errno));
            return nullptr;
        }
        TETHER_LOGW(TAG,
            "cyclic channel: ring setup failed ({}) — "
            "falling back to socket mode", strerror(errno));
    }

    const int fd = openCyclicSocket(cfg.ifindex, cfg.accept_prog,
                                    cfg.accept_prog_len);
    if (fd < 0) {
        TETHER_LOGW(TAG, "createCyclicChannel: cannot open cyclic socket "
                    "on ifindex {} ({})", cfg.ifindex, strerror(errno));
        return nullptr;
    }
    TETHER_LOGI(TAG, "cyclic channel: socket mode (ifindex {})", cfg.ifindex);
    return std::make_unique<LinuxSocketChannel>(fd, cfg.ifindex);
}

// ---- Test/embedding seams -------------------------------------------------

std::unique_ptr<ICyclicChannel> createCyclicSocketChannelForFd(
    int fd, int ifindex)
{
    if (fd < 0) return nullptr;
    return std::make_unique<LinuxSocketChannel>(fd, ifindex);
}

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForFd(
    int fd, int ifindex, uint32_t rx_ring_blocks, uint32_t tx_ring_blocks,
    uint32_t rx_spin_ns)
{
    if (fd < 0) return nullptr;
    LinuxRingChannel::Config rcfg{rx_ring_blocks, tx_ring_blocks, rx_spin_ns};
    auto ring = std::make_unique<LinuxRingChannel>(fd, ifindex, rcfg);
    if (!ring->init()) return nullptr;   // dtor closes fd + unmaps
    return ring;
}

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForMemory(
    int fd, int ifindex,
    void* rx_ring, uint32_t rx_frame_size, uint32_t rx_frames,
    void* tx_ring, uint32_t tx_frame_size, uint32_t tx_frames,
    uint32_t rx_spin_ns)
{
    if (fd < 0 || !rx_ring || rx_frame_size == 0 || rx_frames == 0)
        return nullptr;
    LinuxRingChannel::Config rcfg{0, 0, rx_spin_ns};
    auto ring = std::make_unique<LinuxRingChannel>(fd, ifindex, rcfg);
    ring->adoptRingsForTest(static_cast<uint8_t*>(rx_ring), rx_frame_size,
                            rx_frames,
                            static_cast<uint8_t*>(tx_ring), tx_frame_size,
                            tx_frames);
    return ring;
}

std::unique_ptr<ICyclicChannel> createCyclicRingChannelV3ForMemory(
    int fd, int ifindex,
    void* rx_ring, uint32_t block_size, uint32_t blocks)
{
    if (fd < 0 || !rx_ring || block_size == 0 || blocks == 0)
        return nullptr;
    LinuxRingChannel::Config rcfg{};
    rcfg.rx_v3 = true;
    auto ring = std::make_unique<LinuxRingChannel>(fd, ifindex, rcfg);
    ring->adoptV3RingForTest(static_cast<uint8_t*>(rx_ring), block_size,
                             blocks);
    return ring;
}

} // namespace EtherCAT

#else  // !__linux__

namespace EtherCAT {

bool cyclicChannelAttachCyclicFilter(int) { return false; }
bool cyclicChannelAttachAsyncFilter(int)  { return false; }

size_t cyclicChannelBpfProgram(bool, CyclicBpfInsn*, size_t) { return 0; }

std::unique_ptr<ICyclicChannel> createCyclicChannel(
    const CyclicChannelConfig& cfg)
{
    (void)cfg;
    return nullptr;   // platform channels (ESP32, Windows) are separate impls
}

// Unsupported platform: channels unavailable — fd ownership stays with the
// caller (nullptr is not a channel, nothing was taken).
std::unique_ptr<ICyclicChannel> createCyclicSocketChannelForFd(
    int, int) { return nullptr; }

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForFd(
    int, int, uint32_t, uint32_t, uint32_t) { return nullptr; }

std::unique_ptr<ICyclicChannel> createCyclicRingChannelV3ForMemory(
    int, int, void*, uint32_t, uint32_t) { return nullptr; }

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForMemory(
    int, int, void*, uint32_t, uint32_t, void*, uint32_t, uint32_t,
    uint32_t) { return nullptr; }

} // namespace EtherCAT

#endif // __linux__
