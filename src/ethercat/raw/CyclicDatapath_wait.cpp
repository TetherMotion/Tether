/**
 * @file CyclicDatapath_wait.cpp
 * @brief CyclicDatapath — slot/pool wait machinery (view, mask, pool, list).
 *
 * TU split out of CyclicDatapath.cpp.
 */

#include "raw/CyclicDatapath.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "raw/internal.hpp"
#include "raw/PacketDebugger.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"

#include <chrono>
#include <algorithm>
#include <cstring>
#include <inttypes.h>
#ifdef __linux__
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <unistd.h>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

bool CyclicDatapath::waitView(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out)
{
    if (slot > kCyclicDcPoolPos) return false;
    return waitViewImpl(cyclicFastIndex(slot), token, timeout_ns, out);
}

bool CyclicDatapath::waitSliceView(uint8_t slice, uint64_t token,
                                   uint32_t timeout_ns, CyclicSlotView& out)
{
    if (slice >= kNumSliceSlots) return false;
    return waitViewImpl(sliceFastIndex(slice), token, timeout_ns, out);
}

bool CyclicDatapath::waitViewImpl(uint8_t fast_idx, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out)
{
    auto& s = slots_[fast_idx];

    auto read_slot = [&]() -> bool {
        // Seqlock read: the publisher writes the fields then bumps seq
        // (release).  Re-check seq after the read — a second publish
        // landing mid-read is detected and retried.
        for (int tries = 0; tries < 8; ++tries) {
            const uint64_t s0 = s.seq.load(std::memory_order_acquire);
            out.cmd     = s.cmd;
            out.adp     = s.adp;
            out.ado     = s.ado;
            out.datalen = s.datalen;
            out.wkc     = s.wkc;
            out.gen     = s.gen;
            out.stamp_ns = s.stamp_ns;
            out.payload = s.payload ? s.payload : s.data;
            const int64_t cookie = s.cookie;
            out.cookie  = cookie >= 0 ? static_cast<uint32_t>(cookie) : 0;
            out.channel = cookie >= 0 ? channel_.get() : nullptr;
            if (s.seq.load(std::memory_order_acquire) == s0) return true;
        }
        return true;   // accept last read — publisher is single-writer
    };

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_ns =
        clock.getMicroseconds() * 1000 + static_cast<int64_t>(timeout_ns);

    wait_calls_.fetch_add(1, std::memory_order_relaxed);

    // Drain any ring-resident frames BEFORE the fast-path check: the
    // deposit only happens here, so a frame sitting in the ring would
    // otherwise be invisible (and the ring is never left full).
    drainChannel();

    // Fast path: response already deposited.  seq_cst pairs with the
    // deposit's publish bump (Q13).
    if (s.seq.load(std::memory_order_seq_cst) != token) {
        return read_slot();
    }

    // Register as a waiter BEFORE any blocking point so a deposit landing
    // between the fast-path check and the sleep still wakes us (deposit
    // paths write the eventfd only while waiters_ > 0).  The seq
    // re-check inside the loop covers deposits that raced ahead of the
    // registration.
    struct WaiterGuard {
        std::atomic<int>& c;
        ~WaiterGuard() { c.fetch_sub(1, std::memory_order_acq_rel); }
    } waiter_guard{waiters_};
    waiters_.fetch_add(1, std::memory_order_acq_rel);

    // Unified wait: one ppoll over every wake source — the deposit eventfd
    // (covers frames consumed by the poll thread / socket-B deposits when
    // the BPF is absent) and the wire fd (the channel's own socket, or the
    // iface socket on the no-channel path).
#ifdef __linux__
    struct pollfd fds[2];
    nfds_t nfds = 0;
    int efd_pos = -1, wire_pos = -1;
    if (notify_fd_ >= 0) {
        efd_pos = static_cast<int>(nfds);
        fds[nfds++] = { notify_fd_, POLLIN, 0 };
    }
    int wire_fd = -1;
    if (channel_) {
        wire_fd = channel_->fd();
    } else if (master_.iface_.receive) {
        wire_fd = static_cast<int>(
            reinterpret_cast<intptr_t>(master_.iface_.native_handle));
    }
    if (wire_fd >= 0) {
        wire_pos = static_cast<int>(nfds);
        fds[nfds++] = { wire_fd, POLLIN, 0 };
    }
#endif

    while (true) {
        if (s.seq.load(std::memory_order_seq_cst) != token) {
            return read_slot();
        }
        if (master_.cancel_requested_.load(std::memory_order_acquire)) {
            drainChannel();   // leave the ring empty for the next run
            return false;
        }
        const int64_t now_ns = clock.getMicroseconds() * 1000;
        int64_t remain = deadline_ns - now_ns;
        if (remain <= 0) {
            // Deadline expired — still drain: a reply that arrived just
            // past its deadline is deposited (the caller's gen guard
            // rejects it) and, critically, the ring slot is freed.
            drainChannel();
            if (s.seq.load(std::memory_order_seq_cst) != token)
                return read_slot();
            return false;
        }

        if (channel_) {
            // Spin phase: the ring backend's rxPending() is pure memory
            // reads — a NIC DMA write becomes visible before the kernel
            // could ever wake a ppoll() sleeper.  Shares the deadline
            // budget; skipped entirely when slot_spin_ns_ == 0.  (This is
            // the *slot-wait* spin — rx_spin_ns_ is the channel's own
            // rxPoll busy-poll, a separate knob, Q11.)
            const uint32_t spin_ns = slot_spin_ns_;
            if (spin_ns > 0) {
                const int64_t spin_end = now_ns +
                    std::min<int64_t>(remain, spin_ns);
                while (clock.getMicroseconds() * 1000 < spin_end) {
                    if (s.seq.load(std::memory_order_seq_cst) != token) {
                        return read_slot();
                    }
                    if (channel_->rxPending()) break;
                }
            }
            CyclicFrameView views[8];
            const int n = channel_->rxPoll(views, 8, 0);
            for (int i = 0; i < n; ++i) dispatchFrame(views[i]);
            if (n > 0) continue;      // re-check slots before sleeping
        }

#ifdef __linux__
        if (nfds > 0) {
            const int64_t remain2 = deadline_ns -
                                    clock.getMicroseconds() * 1000;
            if (remain2 <= 0) continue;   // deadline check at loop top
            struct timespec ts;
            ts.tv_sec  = remain2 / 1'000'000'000LL;
            ts.tv_nsec = remain2 % 1'000'000'000LL;
            const int ret = ppoll(fds, nfds, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            if (ret == 0) {
                // ppoll deadline — drain before declaring a miss (the
                // frame may have landed in the ring just past expiry).
                drainChannel();
                if (s.seq.load(std::memory_order_seq_cst) != token)
                    return read_slot();
                return false;
            }

            if (wire_pos >= 0 && (fds[wire_pos].revents & POLLIN)) {
                if (channel_) {
                    CyclicFrameView views[8];
                    const int n = channel_->rxPoll(views, 8, 0);
                    for (int i = 0; i < n; ++i)
                        dispatchFrame(views[i]);
                } else {
                    // Bounded drain: an async burst must not burn the
                    // whole RX budget — POLLIN stays asserted so the
                    // remaining frames are handled next iteration.
                    uint8_t buf[1600];
                    for (int i = 0; i < 8; ++i) {
                        size_t n = 0;
                        if (!master_.iface_.receive(buf, sizeof(buf), &n) || n == 0)
                            break;
                        master_.handleRxFrame(buf, n);
                    }
                }
            }
            if (efd_pos >= 0 && (fds[efd_pos].revents & POLLIN)) {
                uint64_t v;
                ssize_t r = ::read(notify_fd_, &v, sizeof(v));
                (void)r;
            }
            continue;
        }
#endif

        // No fd wake-up path (non-Linux / no eventfd / no wire fd): loop —
        // a bounded spin on the sequence counter plus channel drain, with
        // the seq/cancel/deadline checks at the top bounding it so a dead
        // link cannot wedge the cyclic thread.  Q22: the fallback policy
        // is user-configurable — Yield (default) drops the thread's time
        // slice each pass; Spin keeps the tightest re-check cadence at
        // full CPU burn.
        if (slot_wait_fallback_ ==
            Master::CyclicLoopConfig::SlotWaitFallback::Yield) {
            std::this_thread::yield();
        }
    }
}

uint32_t CyclicDatapath::waitMask(uint32_t slot_mask,
                                    const uint64_t* tokens,
                                    uint32_t timeout_ns,
                                    CyclicSlotView* views)
{
    // Mask bits are rotating-pool positions → resolve each set bit to
    // its slots_ index and defer to the list wait.  The 32-bit mask can
    // only express positions 0..31; waitPool() covers the full range.
    if (!tokens || !views) return 0;
    uint8_t idxs[32];
    uint8_t mask_slots[32];
    bool    arrived[32];
    CyclicSlotView vtmp[32];
    uint8_t count = 0;
    uint32_t m = slot_mask;
    while (m && count < 32) {
        const uint8_t bit = static_cast<uint8_t>(__builtin_ctz(m));
        m &= m - 1;
        mask_slots[count] = bit;
        idxs[count]       = cyclicFastIndex(bit);
        ++count;
    }
    if (!count) return 0;
    uint64_t ttmp[32];
    for (uint8_t i = 0; i < count; ++i) ttmp[i] = tokens[mask_slots[i]];
    waitListImpl(idxs, ttmp, count, timeout_ns, vtmp, arrived);
    uint32_t result = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (!arrived[i]) continue;
        views[mask_slots[i]] = vtmp[i];
        result |= 1u << mask_slots[i];
    }
    return result;
}

uint32_t CyclicDatapath::waitSliceMask(uint32_t slice_mask,
                                     const uint64_t* tokens,
                                     uint32_t timeout_ns,
                                     CyclicSlotView* views)
{
    if (!tokens || !views) return 0;
    uint8_t idxs[kNumSliceSlots];
    uint8_t mask_slots[kNumSliceSlots];
    bool    arrived[kNumSliceSlots];
    CyclicSlotView vtmp[kNumSliceSlots];
    uint64_t ttmp[kNumSliceSlots];
    uint8_t count = 0;
    uint32_t m = slice_mask;
    while (m && count < kNumSliceSlots) {
        const uint8_t bit = static_cast<uint8_t>(__builtin_ctz(m));
        m &= m - 1;
        if (bit >= kNumSliceSlots) break;
        mask_slots[count] = bit;
        idxs[count]       = sliceFastIndex(bit);
        ttmp[count]       = tokens[bit];
        ++count;
    }
    if (!count) return 0;
    waitListImpl(idxs, ttmp, count, timeout_ns, vtmp, arrived);
    uint32_t result = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (!arrived[i]) continue;
        views[mask_slots[i]] = vtmp[i];
        result |= 1u << mask_slots[i];
    }
    return result;
}

uint8_t CyclicDatapath::waitPool(const uint8_t* positions,
                                 const uint64_t* tokens,
                                 uint8_t count, uint32_t timeout_ns,
                                 CyclicSlotView* views, bool* arrived)
{
    if (!positions || !tokens || !views || !arrived || !count ||
        count > kCyclicDcPoolPos + 1) return 0;
    uint8_t idxs[kCyclicDcPoolPos + 1];
    for (uint8_t i = 0; i < count; ++i) {
        if (positions[i] > kCyclicDcPoolPos) return 0;
        idxs[i] = cyclicFastIndex(positions[i]);
    }
    return waitListImpl(idxs, tokens, count, timeout_ns, views, arrived);
}

uint8_t CyclicDatapath::waitListImpl(const uint8_t* fast_idxs,
                                     const uint64_t* tokens,
                                     uint8_t count,
                                     uint32_t timeout_ns,
                                     CyclicSlotView* views,
                                     bool* arrived)
{
    // Q3: one wake for the whole list — a pipelined collect pays a
    // single ppoll registration instead of one sleep per request.  Each
    // wake re-scans all outstanding entries; a deposit on ANY slot is
    // progress.
    if (!fast_idxs || !tokens || !views || !arrived || !count) return 0;
    std::memset(arrived, 0, count);
    bool filled[kCyclicDcPoolPos + 1];
    std::memset(filled, 0, count);

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_ns =
        clock.getMicroseconds() * 1000 + static_cast<int64_t>(timeout_ns);

    // Outstanding entries → arrived so far; fill views for arrived slots.
    auto scan = [&]() -> uint8_t {
        uint8_t n_out = 0;
        for (uint8_t s = 0; s < count; ++s) {
            if (arrived[s]) continue;
            if (slots_[fast_idxs[s]].seq.load(std::memory_order_seq_cst)
                == tokens[s]) {
                ++n_out;
            } else {
                arrived[s] = true;
            }
        }
        return n_out;
    };
    auto fill = [&]() {
        for (uint8_t s = 0; s < count; ++s) {
            if (!arrived[s] || filled[s]) continue;
            auto& slot = slots_[fast_idxs[s]];
            for (int tries = 0; tries < 8; ++tries) {
                const uint64_t s0 = slot.seq.load(std::memory_order_acquire);
                views[s].cmd     = slot.cmd;
                views[s].adp     = slot.adp;
                views[s].ado     = slot.ado;
                views[s].datalen = slot.datalen;
                views[s].wkc     = slot.wkc;
                views[s].gen     = slot.gen;
                views[s].stamp_ns = slot.stamp_ns;
                views[s].payload = slot.payload ? slot.payload : slot.data;
                const int64_t cookie = slot.cookie;
                views[s].cookie  = cookie >= 0
                    ? static_cast<uint32_t>(cookie) : 0;
                views[s].channel = cookie >= 0 ? channel_.get()
                                               : nullptr;
                if (slot.seq.load(std::memory_order_acquire) == s0) break;
            }
            filled[s] = true;
        }
    };
    auto arrived_count = [&]() -> uint8_t {
        uint8_t n = 0;
        for (uint8_t s = 0; s < count; ++s) n += arrived[s] ? 1 : 0;
        return n;
    };

    wait_calls_.fetch_add(1, std::memory_order_relaxed);

    // Deposit any ring-resident frames first — replies that landed past
    // an earlier deadline become visible here instead of wedging the ring.
    drainChannel();

    uint8_t outstanding = scan();
    if (!outstanding) { fill(); return count; }

    struct WaiterGuard {
        std::atomic<int>& c;
        ~WaiterGuard() { c.fetch_sub(1, std::memory_order_acq_rel); }
    } waiter_guard{waiters_};
    waiters_.fetch_add(1, std::memory_order_acq_rel);

#ifdef __linux__
    struct pollfd fds[2];
    nfds_t nfds = 0;
    int efd_pos = -1, wire_pos = -1;
    if (notify_fd_ >= 0) {
        efd_pos = static_cast<int>(nfds);
        fds[nfds++] = { notify_fd_, POLLIN, 0 };
    }
    int wire_fd = -1;
    if (channel_) {
        wire_fd = channel_->fd();
    } else if (master_.iface_.receive) {
        wire_fd = static_cast<int>(
            reinterpret_cast<intptr_t>(master_.iface_.native_handle));
    }
    if (wire_fd >= 0) {
        wire_pos = static_cast<int>(nfds);
        fds[nfds++] = { wire_fd, POLLIN, 0 };
    }
#endif

    while (true) {
        outstanding = scan();
        if (!outstanding) { fill(); return count; }
        if (master_.cancel_requested_.load(std::memory_order_acquire)) {
            drainChannel();   // leave the ring empty for the next run
            scan();
            fill();
            return arrived_count();
        }
        const int64_t now_ns = clock.getMicroseconds() * 1000;
        if (deadline_ns - now_ns <= 0) {
            // Expired — drain anyway: stragglers get deposited (the gen
            // guard classifies them stale) and the ring is freed.
            drainChannel();
            scan();
            fill();
            return arrived_count();
        }

        if (channel_) {
            const uint32_t spin_ns = slot_spin_ns_;
            if (spin_ns > 0) {
                const int64_t spin_end = now_ns +
                    std::min<int64_t>(deadline_ns - now_ns, spin_ns);
                while (clock.getMicroseconds() * 1000 < spin_end) {
                    bool any = false;
                    for (uint8_t s = 0; s < count; ++s) {
                        if (arrived[s]) continue;
                        if (slots_[fast_idxs[s]].seq.load(
                                std::memory_order_seq_cst) != tokens[s]) {
                            any = true; break;
                        }
                    }
                    if (any || channel_->rxPending()) break;
                }
            }
            CyclicFrameView fviews[8];
            const int n = channel_->rxPoll(fviews, 8, 0);
            for (int i = 0; i < n; ++i) dispatchFrame(fviews[i]);
            if (n > 0) continue;
        }

#ifdef __linux__
        if (nfds > 0) {
            const int64_t remain2 = deadline_ns -
                                    clock.getMicroseconds() * 1000;
            if (remain2 <= 0) continue;
            struct timespec ts;
            ts.tv_sec  = remain2 / 1'000'000'000LL;
            ts.tv_nsec = remain2 % 1'000'000'000LL;
            const int ret = ppoll(fds, nfds, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                scan();
                fill();
                return arrived_count();
            }
            if (ret == 0) break;   // deadline reached — final scan below

            if (wire_pos >= 0 && (fds[wire_pos].revents & POLLIN)) {
                if (channel_) {
                    CyclicFrameView fviews[8];
                    const int n = channel_->rxPoll(fviews, 8, 0);
                    for (int i = 0; i < n; ++i)
                        dispatchFrame(fviews[i]);
                } else {
                    uint8_t buf[1600];
                    for (int i = 0; i < 8; ++i) {
                        size_t n = 0;
                        if (!master_.iface_.receive(buf, sizeof(buf), &n) || n == 0)
                            break;
                        master_.handleRxFrame(buf, n);
                    }
                }
            }
            if (efd_pos >= 0 && (fds[efd_pos].revents & POLLIN)) {
                uint64_t v;
                ssize_t r = ::read(notify_fd_, &v, sizeof(v));
                (void)r;
            }
            continue;
        }
#endif

        if (slot_wait_fallback_ ==
            Master::CyclicLoopConfig::SlotWaitFallback::Yield) {
            std::this_thread::yield();
        }
    }
    // ppoll deadline expired — drain + one last scan, then report what
    // arrived.
    drainChannel();
    scan();
    fill();
    return arrived_count();
}

bool CyclicDatapath::wait(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns, RxDatagram& out)
{
    CyclicSlotView view{};
    if (!waitView(slot, token, timeout_ns, view)) return false;
    out.idx     = cyclicPoolWireIdx(slot);
    out.cmd     = static_cast<Command>(view.cmd);
    out.adp     = view.adp;
    out.ado     = view.ado;
    out.datalen = view.datalen;
    out.wkc     = view.wkc;
    if (view.datalen > 0 && view.payload) {
        std::memcpy(out.data, view.payload,
                    std::min<size_t>(view.datalen, sizeof(out.data)));
    }
    return true;
}


} // namespace EtherCAT
