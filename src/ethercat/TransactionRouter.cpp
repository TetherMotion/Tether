/**
 * @file TransactionRouter.cpp
 * @brief Implementation of TransactionRouter — race-free idx-based packet routing
 */

#include "TransactionRouter.hpp"
#include "tether/platform/EspCompat.hpp"

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

    // Wake up every pending waiter so it can exit
    for (auto& s : slots_) {
        std::lock_guard<std::mutex> lock(s.mtx);
        if (s.pending.load(std::memory_order_relaxed)) {
            s.completed.store(true, std::memory_order_relaxed);  // let it see "completed" but result has success=false
            s.cv.notify_all();
        }
    }

    initialized_.store(false, std::memory_order_release);
}

void TransactionRouter::cancel()
{
    cancelled_.store(true, std::memory_order_release);

    // Wake all threads blocked on condition variables
    for (auto& s : slots_) {
        std::lock_guard<std::mutex> lock(s.mtx);
        s.cv.notify_all();
    }
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
    slot.cv.notify_one();

    return 1;
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
    WaitResult result = WaitResult::Timeout();
    {
        std::unique_lock<std::mutex> lock(slot.mtx);
        bool got_it = slot.cv.wait_for(
            lock,
            std::chrono::milliseconds(timeout_ms),
            [&] { return slot.completed.load(std::memory_order_relaxed) || cancelled_.load(std::memory_order_acquire); });

        if (got_it && !shutdown_.load(std::memory_order_acquire) &&
            !cancelled_.load(std::memory_order_acquire)) {
            auto& r = slot.response;
            result = WaitResult::Success(
                r.wkc,
                r.datalen,
                r.cmd,
                r.adp,
                r.ado,
                r.idx);
        } else {
            stats_timeouts_.fetch_add(1, std::memory_order_relaxed);
        }

        slot.pending.store(false, std::memory_order_relaxed);
        slot.buffer     = nullptr;
        slot.buffer_size = 0;
    }

    return result;
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
        WaitResult result = WaitResult::Timeout();
        {
            std::unique_lock<std::mutex> lock(slot.mtx);
            bool got_it = slot.cv.wait_for(
                lock,
                std::chrono::milliseconds(timeout_ms),
                [&] { return slot.completed.load(std::memory_order_relaxed) || cancelled_.load(std::memory_order_acquire); });

            if (got_it && !shutdown_.load(std::memory_order_acquire) &&
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
        }

        return result;
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

    WaitResult result = WaitResult::Timeout();
    {
        std::unique_lock<std::mutex> lock(slot.mtx);
        bool got_it = slot.cv.wait_for(
            lock,
            std::chrono::milliseconds(timeout_ms),
            [&] { return slot.completed.load(std::memory_order_relaxed) || cancelled_.load(std::memory_order_acquire); });

        if (got_it && !shutdown_.load(std::memory_order_acquire) &&
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
    }

    return result;
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
