/**
 * @file LinuxRingChannel.hpp
 * @brief Linux PACKET_MMAP TPACKET_V2 RX+TX ring cyclic channel.
 *
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#if defined(__linux__)

#include "raw/LinuxSocketChannel.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <format>

#include <linux/filter.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

namespace EtherCAT {

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
        // TX ring is opt-in (TETHER_CYCLIC_TX_RING): it wedged on some
        // kernels — slots stayed TP_STATUS_SEND_REQUEST forever and cyclic
        // TX died.  Plain sendmsg/sendto on the same socket is unaffected
        // by the RX filter and is the default classical TX path.
        struct tpacket_req rx_req{}, tx_req{};
        if (!configRxRing(&rx_req)) return false;
#if TETHER_CYCLIC_TX_RING
        const bool have_tx = configTxRing(&tx_req);   // optional — sendto works
        if (!have_tx)
            TETHER_LOGW(TAG, "TX ring unavailable — cyclic TX uses sendto");
#else
        const bool have_tx = false;
        (void)tx_req;
        TETHER_LOGI(TAG, "TX ring disabled (TETHER_CYCLIC_TX_RING=0) — "
                         "cyclic TX uses classical sendmsg/sendto");
#endif

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
        rx_block_size_ = rx_req.tp_block_size;
        rx_fpb_        = cfg_.rx_v3
            ? 1u : (rx_req.tp_frame_nr / cfg_.rx_blocks);
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
            tx_block_size_ = tx_req.tp_block_size;
            tx_fpb_        = tx_req.tp_frame_nr / cfg_.tx_blocks;
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
                           uint32_t tx_frames,
                           uint32_t rx_block_size = 0, uint32_t rx_fpb = 0,
                           uint32_t tx_block_size = 0, uint32_t tx_fpb = 0) {
        rx_ring_ = rx;  rx_ring_len_ = 0;
        rx_frame_size_ = rx_frame_size;  rx_frames_ = rx_frames;
        // Test rings adopt a flat layout unless the caller asked for a
        // padded per-block stride (the real kernel geometry).
        rx_block_size_ = rx_block_size ? rx_block_size : rx_frame_size;
        rx_fpb_        = rx_fpb ? rx_fpb : 1;
        rx_holds_    = std::make_unique<std::atomic<int>[]>(rx_frames_);
        rx_consumed_ = std::make_unique<std::atomic<bool>[]>(rx_frames_);
        tx_ring_ = tx;  tx_ring_len_ = 0;
        tx_frame_size_ = tx_frame_size;  tx_frames_ = tx_frames;
        tx_block_size_ = tx_block_size ? tx_block_size : tx_frame_size;
        tx_fpb_        = tx_fpb ? tx_fpb : 1;
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
        // All TX slots in-flight: kernel never drained the ring — the last
        // sendto kick errored or TX is wedged.  Rate-limited diag.
        if ((++tx_exhausted_ & 0x3FF) == 1) {
            uint32_t st[kNumTxStates]{};
            for (uint32_t i = 0; i < tx_frames_; ++i) {
                const uint32_t s =
                    __atomic_load_n(&txSlot(i)->tp_status, __ATOMIC_ACQUIRE);
                st[s < kNumTxStates ? s : kNumTxStates - 1]++;
            }
            TETHER_LOGW(TAG,
                "TX ring exhausted ({} slots): status histogram "
                "avail={} send_req={} sending={} err={} other={}",
                tx_frames_, st[0], st[1], st[2], st[3], st[kNumTxStates - 1]);
        }
        return nullptr;
    }
    static constexpr uint32_t kNumTxStates = 8;

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
        if (s < 0 && errno != EAGAIN && (tx_kick_errs_++ & 0xFF) == 0) {
            TETHER_LOGE(TAG, "TX ring kick failed: errno={} ({})",
                        errno, strerror(errno));
        }
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
            const auto* hdr = rxSlot(i);
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
        if (idx < rx_frames_) {
            dbg_holds_.fetch_add(1, std::memory_order_relaxed);
            rx_holds_[idx].fetch_add(1, std::memory_order_relaxed);
        }
    }
    void rxRelease(uint32_t cookie) override {
        const uint32_t idx = cfg_.rx_v3 ? (cookie >> 16) : cookie;
        if (idx >= rx_frames_) return;
        // Clamp at 0 — a double-release must not drive the count negative
        // and pin the ring slot forever.
        if (rx_holds_[idx].load(std::memory_order_acquire) <= 0) return;
        dbg_releases_.fetch_add(1, std::memory_order_relaxed);
        if (rx_holds_[idx].fetch_sub(1, std::memory_order_acq_rel) == 1 &&
            rx_consumed_[idx]) {
            __sync_synchronize();
            dbg_freed_.fetch_add(1, std::memory_order_relaxed);
            if (cfg_.rx_v3) {
                v3Retire(idx);
            } else {
                // KERNEL first, THEN clear consumed — the reverse order
                // lets a racing walk re-emit the stale frame while the
                // slot still reads USER.  (v3Retire already clears it;
                // omitting this here wedged the ring: every slot was
                // emitted exactly once, then skipped forever.)
                __atomic_store_n(&rxSlot(idx)->tp_status, TP_STATUS_KERNEL,
                                 __ATOMIC_RELEASE);
                rx_consumed_[idx].store(false, std::memory_order_release);
            }
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

    // V2 ring layout is [block][block]... with tp_block_size rounded up to
    // a page multiple — when frames_per_block*frame_size < block_size the
    // tail of each block is padding, so frame i lives at
    // (i/fpb)*block_size + (i%fpb)*frame_size, NOT i*frame_size.  Flat
    // indexing reads padding for every frame past the first block and the
    // ring wedges full forever.
    struct tpacket2_hdr* rxSlot(uint32_t i) {
        return reinterpret_cast<struct tpacket2_hdr*>(
            rx_ring_ + static_cast<size_t>(i / rx_fpb_) * rx_block_size_
                     + static_cast<size_t>(i % rx_fpb_) * rx_frame_size_);
    }
    const struct tpacket2_hdr* rxSlot(uint32_t i) const {
        return reinterpret_cast<const struct tpacket2_hdr*>(
            rx_ring_ + static_cast<size_t>(i / rx_fpb_) * rx_block_size_
                     + static_cast<size_t>(i % rx_fpb_) * rx_frame_size_);
    }
    struct tpacket2_hdr* txSlot(uint32_t i) {
        return reinterpret_cast<struct tpacket2_hdr*>(
            tx_ring_ + static_cast<size_t>(i / tx_fpb_) * tx_block_size_
                     + static_cast<size_t>(i % tx_fpb_) * tx_frame_size_);
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
                    dbg_freed_.fetch_add(1, std::memory_order_relaxed);
                }
                continue;
            }
            if (n >= max_views) break;

            views[n].frame     = reinterpret_cast<const uint8_t*>(hdr) +
                                 hdr->tp_mac;
            views[n].frame_len = hdr->tp_snaplen;
            views[n].stamp_ns  = rtStampToMonoNs(
                static_cast<uint64_t>(hdr->tp_sec) * 1'000'000'000ULL +
                hdr->tp_nsec);
            views[n].cookie    = idx;
            ++n;
            dbg_emitted_.fetch_add(1, std::memory_order_relaxed);
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

    std::string rxRingDebug() const override {
        uint32_t held = 0, consumed_unfreed = 0;
        for (uint32_t i = 0; i < rx_frames_; ++i) {
            if (rx_holds_[i].load(std::memory_order_acquire) > 0) ++held;
            if (rx_consumed_[i].load(std::memory_order_acquire))
                ++consumed_unfreed;
        }
        return std::format(
            "emitted={} freed={} holds={} releases={} "
            "held_now={} consumed_unfreed={} frames={}",
            dbg_emitted_.load(std::memory_order_relaxed),
            dbg_freed_.load(std::memory_order_relaxed),
            dbg_holds_.load(std::memory_order_relaxed),
            dbg_releases_.load(std::memory_order_relaxed),
            held, consumed_unfreed, rx_frames_);
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
                if (rx_holds_[b].load(std::memory_order_acquire) == 0) {
                    v3Retire(b);
                    dbg_freed_.fetch_add(1, std::memory_order_relaxed);
                }
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
                views[n].stamp_ns  = rtStampToMonoNs(
                    static_cast<uint64_t>(hdr->tp_sec) * 1'000'000'000ULL +
                    hdr->tp_nsec);
                views[n].cookie    = (b << 16) | emitted;
                ++n;
                dbg_emitted_.fetch_add(1, std::memory_order_relaxed);
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
    uint32_t rx_block_size_ = 0;  ///< V2: page-rounded block stride
    uint32_t rx_fpb_        = 1;  ///< V2: frames per block
    std::unique_ptr<std::atomic<int>[]>  rx_holds_;
    std::unique_ptr<std::atomic<bool>[]> rx_consumed_;

    uint8_t* tx_ring_ = nullptr;
    size_t   tx_ring_len_ = 0;
    uint32_t tx_frame_size_ = 0;
    uint32_t tx_frames_ = 0;
    uint32_t tx_block_size_ = 0;
    uint32_t tx_fpb_        = 1;
    int64_t  tx_acquired_ = -1;   // index of the last txAcquire() slot
    uint32_t tx_cursor_ = 0;
    uint32_t rx_cursor_ = 0;
    bool     rings_borrowed_ = false;   // adoptRingsForTest() seam
    std::atomic<uint64_t> tx_deferred_{0};
    uint32_t tx_exhausted_  = 0;
    uint32_t tx_kick_errs_  = 0;
    // RX lifecycle counters surfaced by rxRingDebug() — emit/free/hold/
    // release totals catch pin leaks that static inspection can't.
    std::atomic<uint64_t> dbg_emitted_{0};
    std::atomic<uint64_t> dbg_freed_{0};
    std::atomic<uint64_t> dbg_holds_{0};
    std::atomic<uint64_t> dbg_releases_{0};
};

} // namespace EtherCAT

#endif // __linux__

