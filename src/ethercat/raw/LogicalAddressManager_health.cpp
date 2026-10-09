/**
 * @file LogicalAddressManager_health.cpp
 * @brief LogicalAddressManager — stall self-heal and exchange-health
 *        bookkeeping for the polled LRW path.
 *
 * TU split out of LogicalAddressManager.cpp; the class and its state are
 * declared in LogicalAddressManager.hpp.
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/Types.hpp"
#include "raw/LogicalAddressManagerInternal.hpp"
#include "raw/TxFailureDiagnostics.hpp"

#include <format>

namespace EtherCAT {

static const char* TAG = "ec_logaddr";

// ---- Stall self-heal helpers (polled LRW path) -----------------------------
//
// Host stalls (thread preemption, logging bursts, non-RT scheduling) break
// the polled exchange in two ways: (1) queued responses outlive their
// request and can satisfy a NEW waiter on a reused idx — the 8-bit idx
// wraps after 156 allocations — feeding stale PDO data to the caller, and
// (2) frames lost at the kernel socket produce silent timeouts.  Neither
// heals by itself.  Recovery is deliberately NON-DESTRUCTIVE: the wire is
// drained so queued replies still reach their live waiters, and each
// exchange prunes only the one slot it transmits on (see
// claimExchangeWaiter) — async slots are never purged.

void LogicalAddressManager::stallCheck()
{
    const int64_t now = static_cast<int64_t>(monoNowNs());
    const int64_t gap = last_call_ns_ ? now - last_call_ns_ : 0;
    last_call_ns_ = now;
    if (stall_detect_us_ == 0 || gap <= 0 ||
        gap <= static_cast<int64_t>(stall_detect_us_) * 1000) {
        return;
    }
    ++stats_.stall_events;
    // Flush the kernel RX backlog through normal routing: frames whose
    // waiters are still pending are delivered (they ARE legitimate late
    // replies), frames for dead slots drop as unrouted strays.  No waiter
    // is killed — independence of unrelated async slots is preserved.
    stats_.drained_frames +=
        static_cast<uint32_t>(transport_.drainWire(512));
    TETHER_LOGW(TAG,
        "exchangeLRW: host stall ({:.1f} ms since last call) — drained "
        "wire backlog",
        static_cast<double>(gap) / 1e6);
}

size_t LogicalAddressManager::claimExchangeWaiter(uint8_t& idx_out,
                                                  RxDatagram& resp)
{
    // Prune the wire state for whatever idx we are about to send on —
    // and nothing else.  Draining BEFORE registering routes backlog
    // frames for other idx to their still-pending waiters (giving them
    // their chance to complete) while a stale echo for the picked idx
    // hits a dead slot and drops as an unrouted stray.
    stats_.drained_frames +=
        static_cast<uint32_t>(transport_.drainWire(64));

    for (unsigned tries = 0; tries < kFastSlotBaseIdx; ++tries) {
        const uint8_t idx = transport_.allocIdx();
        const size_t slot = transport_.preRegisterResponseWaiter(
            idx, resp.data, sizeof(resp.data));
        if (slot == IPDOTransport::kPreRegInvalid) {
            idx_out = idx;
            return slot;   // no pre-registration — caller falls back
        }
        if (slot != IPDOTransport::kPreRegBusy) {
            idx_out = idx;
            return slot;   // claimed — the claim itself pruned the slot
        }
        // Busy: a live waiter owns this idx — leave it alone, try next.
    }
    return IPDOTransport::kPreRegBusy;
}

void LogicalAddressManager::onExchangeTimeout(const char* what)
{
    ++stats_.consecutive_timeouts;
    // Rate-limit the timeout log to 4 Hz — a dead ring otherwise spams
    // one line per exchange (and the logging itself worsens the stall).
    const int64_t now = static_cast<int64_t>(monoNowNs());
    if (now - last_timeout_log_ns_ >= 250'000'000) {
        if (timeout_log_suppressed_ > 0) {
            TETHER_LOGE(TAG,
                "{}: response timeout ({} suppressed since last log)",
                what, timeout_log_suppressed_);
            timeout_log_suppressed_ = 0;
        } else {
            TETHER_LOGE(TAG, "{}: response timeout", what);
        }
        last_timeout_log_ns_ = now;
    } else {
        ++timeout_log_suppressed_;
    }
    // Pull whatever landed late off the wire — either it frees a stuck
    // kernel-queue backlog or it confirms the ring is actually silent.
    stats_.drained_frames +=
        static_cast<uint32_t>(transport_.drainWire(64));
    if (escalate_after_timeouts_ &&
        stats_.consecutive_timeouts % escalate_after_timeouts_ == 0) {
        // Ring probe: APRD of AL_STATUS (0x0130) on the first slave —
        // distinguishes "host can't see replies" from "ring is broken".
        uint16_t al_status = 0;
        const bool alive = transport_.readRegister(0, 0x0130,
                                                   &al_status, 2, 20);
        TETHER_LOGE(TAG,
            "exchangeLRW: {} consecutive timeouts — ring probe {}"
            " (AL_STATUS=0x{:04X}). Check NIC drops / slave link state.",
            stats_.consecutive_timeouts,
            alive ? "ALIVE" : "FAILED — ring likely broken",
            al_status);
        escalate_logged_ = true;
    }
}

void LogicalAddressManager::onExchangeSendFailure(const std::string& what)
{
    // Rate-limit to 4 Hz — a dead NIC or exhausted slot pool otherwise
    // spams one line per exchange (and the logging itself worsens the stall).
    const int64_t now = static_cast<int64_t>(monoNowNs());
    if (now - last_send_fail_log_ns_ >= 250'000'000) {
        if (send_fail_log_suppressed_ > 0) {
            TETHER_LOGE(TAG, "{} ({} suppressed since last log)",
                        what, send_fail_log_suppressed_);
            send_fail_log_suppressed_ = 0;
        } else {
            TETHER_LOGE(TAG, "{}", what);
        }
        last_send_fail_log_ns_ = now;
    } else {
        ++send_fail_log_suppressed_;
    }
}

void LogicalAddressManager::noteTxSendFailure()
{
    tx_diag_->noteSendFailure();
}

void LogicalAddressManager::onExchangeSuccess()
{
    stats_.success++;
    stats_.consecutive_timeouts = 0;
    escalate_logged_ = false;
}


} // namespace EtherCAT

