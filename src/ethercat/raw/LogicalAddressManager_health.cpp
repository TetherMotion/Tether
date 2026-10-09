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
    tx_diag_->noteStall(gap);   // logged rate-limited by the monitor
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
    tx_diag_->noteTimeout(what);   // logged rate-limited by the monitor
    // Pull whatever landed late off the wire — either it frees a stuck
    // kernel-queue backlog or it confirms the ring is actually silent.
    stats_.drained_frames +=
        static_cast<uint32_t>(transport_.drainWire(64));
}

void LogicalAddressManager::setEscalateAfterTimeouts(uint32_t n)
{
    tx_diag_->setEscalateAfterTimeouts(n);
}

void LogicalAddressManager::onExchangeSuccess()
{
    stats_.success++;
    stats_.consecutive_timeouts = 0;
    tx_diag_->noteSuccess();
}


} // namespace EtherCAT

