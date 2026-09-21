/**
 * @file TransactionRouter.cpp
 * @brief Implementation of TransactionRouter — race-free idx-based packet routing
 */

#include "TransactionRouter.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/AtomicWait.hpp"

#include <algorithm>

namespace EtherCAT {

static const char* TAG = "PacketRouter";

// ============================================================================
// Constructor / Destructor
// ============================================================================

TransactionRouter::TransactionRouter() = default;

TransactionRouter::~TransactionRouter() { shutdown(); }

// ============================================================================
// Lifecycle
// ============================================================================

bool TransactionRouter::init()
{
    if (initialized_.load(std::memory_order_acquire)) return true;

    for (auto& s : slots_) {
        std::lock_guard<std::mutex> lock(s.mtx);
        s.pending.store(false, std::memory_order_relaxed);
        s.completed.store(false, std::memory_order_relaxed);
        s.buffer    = nullptr;
        s.buffer_size = 0;
        s.response  = {};
    }

    shutdown_.store(false, std::memory_order_release);
    resetStats();
    initialized_.store(true, std::memory_order_release);

    TETHER_LOGI(TAG, "TransactionRouter initialized ({} slots)", kNumSlots);
    return true;
}

void TransactionRouter::shutdown()
{
    if (!initialized_.load(std::memory_order_acquire)) return;

    shutdown_.store(true, std::memory_order_release);

    // Wake every slot waiter — bump seq so atomicWait() returns and the
    // waiter observes shutdown_ on re-check.  Pending or not is harmless:
    // a bumped seq on an idle slot is just a stale token.
    for (auto& s : slots_) {
        s.seq.fetch_add(1, std::memory_order_release);
        Tether::Platform::atomicWakeAll(&s.seq);
    }

    // Wake any waitForAny() waiters
    any_completion_gen_.fetch_add(1, std::memory_order_release);
    Tether::Platform::atomicWakeAll(&any_completion_gen_);

    initialized_.store(false, std::memory_order_release);
}

void TransactionRouter::cancel()
{
    cancelled_.store(true, std::memory_order_release);

    // Wake all threads blocked in atomicWait()
    for (auto& s : slots_) {
        s.seq.fetch_add(1, std::memory_order_release);
        Tether::Platform::atomicWakeAll(&s.seq);
    }

    // Wake any waitForAny() waiters
    any_completion_gen_.fetch_add(1, std::memory_order_release);
    Tether::Platform::atomicWakeAll(&any_completion_gen_);
}

void TransactionRouter::clearCancel()
{
    cancelled_.store(false, std::memory_order_release);
}

// ============================================================================
// RX path
// ============================================================================

size_t TransactionRouter::routePacket(const RxDatagram& dgram)
{
    if (!initialized_.load(std::memory_order_acquire)) return 0;

    stats_packets_routed_.fetch_add(1, std::memory_order_relaxed);

    auto& slot = slots_[dgram.idx];

    std::lock_guard<std::mutex> lock(slot.mtx);

    if (!slot.pending.load(std::memory_order_relaxed)) {
        stats_packets_dropped_.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }

    // Copy data into the caller's buffer
    if (slot.buffer && slot.buffer_size > 0 && dgram.datalen > 0) {
        size_t copy_len = std::min<size_t>(dgram.datalen, slot.buffer_size);
        std::memcpy(slot.buffer, dgram.data, copy_len);
    }

    // Fill response
    slot.response = dgram;
    slot.completed.store(true, std::memory_order_relaxed);
    stats_packets_matched_.fetch_add(1, std::memory_order_relaxed);
    // seq is the waiters' wake word: bumped (release) after all payload
    // fields so a woken waiter sees a complete response.
    slot.seq.fetch_add(1, std::memory_order_release);
    Tether::Platform::atomicWakeOne(&slot.seq);

    // Notify any waitForAny() waiters that a slot completed.
    any_completion_gen_.fetch_add(1, std::memory_order_release);
    Tether::Platform::atomicWakeAll(&any_completion_gen_);

    return 1;
}

// ============================================================================
// Timed slot wait — atomicWait() on slot.seq
// ============================================================================
//
// The waiter loop below replaces the former cv.wait_for().  Wake order is
// identical: routePacket() writes the response, sets completed, bumps seq,
// then wakes; cancel()/shutdown() bump seq on every slot so all waiters
// re-check the flags.  The mutex is only held for the bookkeeping around
// the wait, never during it.

WaitResult TransactionRouter::waitForSlotCompletion(Slot& slot,
                                                    uint32_t timeout_ms)
{
    uint32_t expected = slot.seq.load(std::memory_order_acquire);
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);

    for (;;) {
        if (slot.completed.load(std::memory_order_acquire) ||
            cancelled_.load(std::memory_order_acquire) ||
            shutdown_.load(std::memory_order_acquire)) {
            break;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        const int64_t remaining_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                deadline - now).count();
        const uint32_t cur = slot.seq.load(std::memory_order_acquire);
        if (cur != expected) { expected = cur; continue; }
        Tether::Platform::atomicWait(&slot.seq, cur, remaining_ns);
    }

    WaitResult result = WaitResult::Timeout();
    std::lock_guard<std::mutex> lock(slot.mtx);
    if (slot.completed.load(std::memory_order_acquire) &&
        !shutdown_.load(std::memory_order_acquire) &&
        !cancelled_.load(std::memory_order_acquire)) {
        auto& r = slot.response;
        result = WaitResult::Success(
            r.wkc, r.datalen, r.cmd, r.adp, r.ado, r.idx);
    } else {
        stats_timeouts_.fetch_add(1, std::memory_order_relaxed);
    }

    slot.pending.store(false, std::memory_order_relaxed);
    slot.buffer     = nullptr;
    slot.buffer_size = 0;
    return result;
}

// ============================================================================
// sendAndWait — primary API
// ============================================================================

WaitResult TransactionRouter::sendAndWait(uint8_t idx,
                                          uint8_t* buffer, size_t buffer_size,
                                          SendFunc send_fn,
                                          uint32_t timeout_ms)
{
    if (!initialized_.load(std::memory_order_acquire))
        return WaitResult::Timeout();
    if (shutdown_.load(std::memory_order_acquire))
        return WaitResult::Timeout();
    if (cancelled_.load(std::memory_order_acquire))
        return WaitResult::Timeout();

    stats_registrations_.fetch_add(1, std::memory_order_relaxed);

    auto& slot = slots_[idx];

    // 1. Mark slot as pending (under lock)
    {
        std::lock_guard<std::mutex> lock(slot.mtx);
        slot.pending.store(true, std::memory_order_relaxed);
        slot.completed.store(false, std::memory_order_relaxed);
        slot.buffer     = buffer;
        slot.buffer_size = buffer_size;
        slot.response   = {};
    }

    // 2. Send the frame.  The slot is now listening, so even an
    //    instantaneous response will be caught by routePacket().
    if (!send_fn()) {
        std::lock_guard<std::mutex> lock(slot.mtx);
        slot.pending.store(false, std::memory_order_relaxed);
        stats_registration_failures_.fetch_add(1, std::memory_order_relaxed);
        return WaitResult::Timeout();
    }

    // 3. Wait for the response
    return waitForSlotCompletion(slot, timeout_ms);
}

// ============================================================================
// Legacy / compatibility APIs
// ============================================================================

WaitResult TransactionRouter::waitForPacket(const PacketFilter& filter,
                                            uint8_t* buffer, size_t buffer_size,
                                            uint32_t timeout_ms)
{
    if (cancelled_.load(std::memory_order_acquire))
        return WaitResult::Timeout();

    // The old API registered a waiter and waited in one call.
    // We can only use this when the filter matches by idx (which is the
    // common case).  If match_idx is false, fall back to a timed poll.
    if (filter.match_idx) {
        // Slot is already pending or we set it up now
        auto& slot = slots_[filter.idx];

        stats_registrations_.fetch_add(1, std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> lock(slot.mtx);
            // If not already pending, set it up
            if (!slot.pending.load(std::memory_order_relaxed)) {
                slot.pending.store(true, std::memory_order_relaxed);
                slot.completed.store(false, std::memory_order_relaxed);
                slot.buffer     = buffer;
                slot.buffer_size = buffer_size;
                slot.response   = {};
            }
        }

        // Wait
        return waitForSlotCompletion(slot, timeout_ms);
    }

    // Fallback: if the filter doesn't match by idx, we can't use the
    // slot-based approach.  This should be rare.
    TETHER_LOGW(TAG, "waitForPacket: filter without match_idx — not supported by TransactionRouter");
    return WaitResult::Timeout();
}

size_t TransactionRouter::preRegisterWaiter(const PacketFilter& filter,
                                            uint8_t* buffer, size_t buffer_size)
{
    if (!initialized_.load(std::memory_order_acquire)) return kNumSlots;
    if (!filter.match_idx) return kNumSlots;

    auto& slot = slots_[filter.idx];
    std::lock_guard<std::mutex> lock(slot.mtx);

    // Reject if slot is already in use (pending and not yet completed)
    if (slot.pending.load(std::memory_order_relaxed) && !slot.completed.load(std::memory_order_relaxed)) return kNumSlots;

    slot.pending.store(true, std::memory_order_relaxed);
    slot.completed.store(false, std::memory_order_relaxed);
    slot.buffer     = buffer;
    slot.buffer_size = buffer_size;
    slot.response   = {};

    stats_registrations_.fetch_add(1, std::memory_order_relaxed);

    return static_cast<size_t>(filter.idx);
}

WaitResult TransactionRouter::waitForPreRegistered(size_t slot_idx, uint32_t timeout_ms)
{
    if (slot_idx >= kNumSlots) return WaitResult::Timeout();
    if (cancelled_.load(std::memory_order_acquire))
        return WaitResult::Timeout();

    auto& slot = slots_[slot_idx];
    return waitForSlotCompletion(slot, timeout_ms);
}

void TransactionRouter::cancelPreRegistered(size_t slot_idx)
{
    if (slot_idx >= kNumSlots) return;

    auto& slot = slots_[slot_idx];
    std::lock_guard<std::mutex> lock(slot.mtx);
    slot.pending.store(false, std::memory_order_relaxed);
    slot.completed.store(false, std::memory_order_relaxed);
    slot.buffer     = nullptr;
    slot.buffer_size = 0;
}

// ============================================================================
// Multi-slot wait (reactor pattern)
// ============================================================================

TransactionRouter::AnyWaitResult
TransactionRouter::waitForAny(const size_t* slot_indices, size_t count,
                               uint32_t timeout_ms)
{
    AnyWaitResult result;
    if (count == 0 || slot_indices == nullptr) {
        return result;  // timed_out = true
    }
    if (!initialized_.load(std::memory_order_acquire)) return result;
    if (cancelled_.load(std::memory_order_acquire)) return result;

    // First, check if any slot has already completed (fast path — no wait).
    for (size_t i = 0; i < count; ++i) {
        size_t si = slot_indices[i];
        if (si >= kNumSlots) continue;
        if (slots_[si].completed.load(std::memory_order_acquire)) {
            // Found a completed slot — extract the result.
            auto& slot = slots_[si];
            std::lock_guard<std::mutex> lock(slot.mtx);
            if (slot.completed.load(std::memory_order_relaxed)) {
                auto& r = slot.response;
                result.slot_index = i;
                result.result = WaitResult::Success(
                    r.wkc, r.datalen, r.cmd, r.adp, r.ado, r.idx);
                result.timed_out = false;
                // Clean up the slot (single-consumer semantics).
                slot.pending.store(false, std::memory_order_relaxed);
                slot.completed.store(false, std::memory_order_relaxed);
                slot.buffer     = nullptr;
                slot.buffer_size = 0;
                return result;
            }
        }
    }

    // No slot has completed yet — block on the generation word.
    // atomicWait() only returns early if gen changed between our load and
    // the wait, so there is no missed wake; spurious wakes just re-loop.
    uint32_t gen = any_completion_gen_.load(std::memory_order_acquire);
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);

    for (;;) {
        if (cancelled_.load(std::memory_order_acquire) ||
            shutdown_.load(std::memory_order_acquire)) {
            return result;  // timed_out = true
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        const int64_t remaining_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                deadline - now).count();
        const uint32_t cur =
            any_completion_gen_.load(std::memory_order_acquire);
        if (cur != gen) break;  // something completed — rescan below
        Tether::Platform::atomicWait(&any_completion_gen_, cur, remaining_ns);
    }

    // Check which slots have completed.
    for (size_t i = 0; i < count; ++i) {
        size_t si = slot_indices[i];
        if (si >= kNumSlots) continue;
        if (slots_[si].completed.load(std::memory_order_acquire)) {
            auto& slot = slots_[si];
            std::lock_guard<std::mutex> slock(slot.mtx);
            if (slot.completed.load(std::memory_order_relaxed)) {
                auto& r = slot.response;
                result.slot_index = i;
                result.result = WaitResult::Success(
                    r.wkc, r.datalen, r.cmd, r.adp, r.ado, r.idx);
                result.timed_out = false;
                slot.pending.store(false, std::memory_order_relaxed);
                slot.completed.store(false, std::memory_order_relaxed);
                slot.buffer     = nullptr;
                slot.buffer_size = 0;
                return result;
            }
        }
    }

    return result;  // timed_out = true
}

// ============================================================================
// Diagnostics
// ============================================================================

bool TransactionRouter::hasWaiters() const
{
    for (const auto& s : slots_) {
        // Atomic relaxed load — safe for lock-free diagnostic reads.
        if (s.pending.load(std::memory_order_relaxed)) return true;
    }
    return false;
}

size_t TransactionRouter::waiterCount() const
{
    size_t count = 0;
    for (const auto& s : slots_) {
        if (s.pending.load(std::memory_order_relaxed)) count++;
    }
    return count;
}

} // namespace EtherCAT
