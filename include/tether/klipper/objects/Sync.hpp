/**
 * @file Sync.hpp
 * @brief Trigger synchronization (Trsync) peripheral and proxy.
 */

#pragma once

#include <cstdint>

namespace tether::klipper::objects {

/// @brief Endstop state.
enum class TrsyncState {
    Idle,
    Armed,
    Triggered,
    Sent,
};

// ============================================================================
// Trsync (trigger synchronization)
// ============================================================================

/// @brief Trsync peripheral for homing synchronization.
class Trsync {
public:
    explicit Trsync(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    /// @brief Arm the trsync.
    void arm(uint32_t timeoutClock) {
        state_ = TrsyncState::Armed;
        timeoutClock_ = timeoutClock;
        triggerClock_ = 0;
    }

    /// @brief Arm with a report clock and timeout.
    void arm(uint32_t reportClock, uint32_t timeoutClock) {
        state_ = TrsyncState::Armed;
        timeoutClock_ = timeoutClock;
        reportClock_ = reportClock;
        triggerClock_ = 0;
    }

    /// @brief Process a tick at a given clock.
    void tick(uint32_t clock) {
        if (state_ == TrsyncState::Armed && clock >= timeoutClock_) {
            expire();
        }
    }

    /// @brief Trigger the trsync.
    void trigger(uint32_t clock) {
        if (state_ == TrsyncState::Armed) {
            state_ = TrsyncState::Triggered;
            triggerClock_ = clock;
        }
    }

    /// @brief Expire the trsync (timeout).
    void expire() {
        if (state_ == TrsyncState::Armed) {
            state_ = TrsyncState::Triggered;
            triggerClock_ = timeoutClock_;
        }
    }

    /// @brief Mark as sent (notification dispatched).
    void markSent() {
        if (state_ == TrsyncState::Triggered) {
            state_ = TrsyncState::Sent;
        }
    }

    /// @brief Get the current state.
    TrsyncState state() const { return state_; }

    /// @brief Get the trigger clock.
    uint32_t triggerClock() const { return triggerClock_; }

    /// @brief Get the timeout clock.
    uint32_t timeoutClock() const { return timeoutClock_; }

    /// @brief Reset to idle.
    void reset() {
        state_ = TrsyncState::Idle;
        triggerClock_ = 0;
        timeoutClock_ = 0;
    }

private:
    uint8_t oid_;
    TrsyncState state_ = TrsyncState::Idle;
    uint32_t triggerClock_ = 0;
    uint32_t timeoutClock_ = 0;
    uint32_t reportClock_ = 0;
};

/// @brief Proxy for Trsync that tracks state changes.
class TrsyncProxy {
public:
    explicit TrsyncProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    void arm(uint32_t reportClock, uint32_t timeoutClock) {
        state_ = TrsyncState::Armed;
    }

    void trigger(uint32_t clock) {
        if (state_ == TrsyncState::Armed) {
            state_ = TrsyncState::Triggered;
        }
    }

    TrsyncState state() const { return state_; }

private:
    uint8_t oid_;
    TrsyncState state_ = TrsyncState::Idle;
};

} // namespace tether::klipper::objects
