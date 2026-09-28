/**
 * @file DeadlineLatch.hpp
 * @brief Single-slot last-value latch with a freshness deadline.
 *
 * Producer threads (transport/session callbacks) offer sequenced values; a
 * control loop consumes the newest value and learns whether it arrived
 * within a deadline.  This is the consumer-side half of the InvokeEx
 * timeout contract: the initiator sends a request with `deadline_us`,
 * the callback offers responses here, and the consumer applies a fresh
 * value or falls back to its own default behavior — never blocking on
 * the peer.
 *
 * All timestamps use the same monotonic µs clock as Session::TimestampFn.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include <cstdint>
#include <mutex>
#include <utility>

namespace tether { namespace io {

template <typename T>
class DeadlineLatch {
public:
    struct Sample {
        bool     valid = false;  ///< a value was offered at least once
        bool     fresh = false;  ///< newest value is within the deadline
        uint64_t seq   = 0;      ///< sequence the initiator correlates with
        uint64_t ageUs = 0;      ///< now - receive time of the newest value
        T        value{};
    };

    /// Store `value`, replacing any previous offer.  Out-of-order arrivals
    /// (late responses) are discarded; only the newest seq wins.
    void offer(uint64_t seq, T value, uint64_t receivedUs) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_ && seq <= seq_) return;
        has_ = true;
        seq_ = seq;
        receivedUs_ = receivedUs;
        value_ = std::move(value);
    }

    /// Snapshot the newest value.  `fresh` is true iff a value exists and
    /// `nowUs - receivedUs <= deadlineUs`.  A non-fresh or invalid sample
    /// is the application's cue to apply its fallback policy.
    Sample consume(uint64_t nowUs, uint64_t deadlineUs) const {
        std::lock_guard<std::mutex> lock(mutex_);
        Sample s;
        if (!has_) return s;
        s.valid = true;
        s.seq = seq_;
        s.ageUs = (nowUs > receivedUs_) ? nowUs - receivedUs_ : 0;
        s.fresh = (deadlineUs == 0) || (s.ageUs <= deadlineUs);
        s.value = value_;
        return s;
    }

    /// Discard the stored value (e.g. on mode transitions or reconnect).
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        has_ = false;
        seq_ = 0;
        receivedUs_ = 0;
        value_ = T{};
    }

private:
    mutable std::mutex mutex_;
    bool     has_ = false;
    uint64_t seq_ = 0;
    uint64_t receivedUs_ = 0;
    T        value_{};
};

}} // namespace tether::io
