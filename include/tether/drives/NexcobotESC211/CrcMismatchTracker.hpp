#pragma once

/**
 * @file CrcMismatchTracker.hpp
 * @brief Lock-free counter that trips ESC211 config recovery after too
 *        many FSoE CRC mismatches.
 *
 * Frame processors (e.g. a black-channel relay's active-bridge
 * processor) call `increment()` whenever an FSoE frame CRC verification
 * fails.  When the cumulative count reaches the configured threshold,
 * `shouldRecover()` returns true — a monitoring loop or orchestrator
 * can then trigger a config re-write + re-activation via
 * `ConfigRecoveryHandler` and call `reset()` afterwards.
 *
 * Rationale: repeated CRC mismatches on *every* channel usually mean
 * the ESC211's stored config (FNI/RSP/SDD) was corrupted or lost —
 * rewriting it recovers the system without operator intervention.
 *
 * The counter is atomic and lock-free, safe to call from realtime
 * processing threads.
 */

#include <atomic>
#include <cstdint>

namespace EtherCAT::Drives::ESC211 {

class CrcMismatchTracker {
public:
    /// Construct with a mismatch threshold (default
    /// `kDefaultThreshold`).  Threshold 0 disables recovery:
    /// `shouldRecover()` never returns true.
    explicit CrcMismatchTracker(uint32_t threshold = kDefaultThreshold)
        : threshold_(threshold) {}

    /// Atomically increment the mismatch counter by 1.
    /// Returns the new count.
    uint32_t increment() {
        return count_.fetch_add(1, std::memory_order_relaxed) + 1;
    }

    /// Returns true when the mismatch count has reached the threshold,
    /// indicating that config recovery (re-write + re-activate) should
    /// be performed.  Once true, it stays true until `reset()`.
    bool shouldRecover() const {
        return count_.load(std::memory_order_relaxed) >= threshold_;
    }

    /// Reset the mismatch counter to 0.  Called after recovery has been
    /// performed.
    void reset() {
        count_.store(0, std::memory_order_relaxed);
    }

    /// Current mismatch count.
    uint32_t count() const {
        return count_.load(std::memory_order_relaxed);
    }

    /// Configured threshold.
    uint32_t threshold() const { return threshold_; }

    /// Default mismatch threshold before recovery is triggered.
    static constexpr uint32_t kDefaultThreshold = 50;

private:
    const uint32_t threshold_;
    std::atomic<uint32_t> count_{0};
};

} // namespace EtherCAT::Drives::ESC211
