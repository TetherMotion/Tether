// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file WatchdogManager.hpp
 * @brief Watchdog management extracted from SlaveCore
 *
 * @details
 * Encapsulates the watchdog sub-responsibility of SlaveCore:
 *  - Watchdog tick counting and timeout detection
 *  - PDI and SM watchdog reset
 *  - Callback notification on watchdog trigger
 */

#include <cstdint>
#include <functional>

#include "tether/slave/core/WatchdogStateSlave.hpp"
#include "tether/slave/core/SlaveTypes.hpp"  // SlaveState
#include "tether/slave/core/SlaveCallbacks.hpp"  // WatchdogCallback

namespace EtherCAT { namespace slave {

class WatchdogManager {
public:
    /// @param enabled  Whether watchdog monitoring is enabled.
    /// @param state    Watchdog state (owned by SlaveCore).
    WatchdogManager(bool enabled, WatchdogState* state)
        : enabled_(enabled), state_(state) {}

    /// @brief Advance the watchdog by deltaNs.
    /// @param currentState  Current AL state (watchdog only active in OP).
    /// @param deltaNs       Time delta in nanoseconds.
    void update(SlaveState currentState, uint64_t deltaNs);

    /// @brief Reset both PDI and SM watchdogs.
    void reset() {
        state_->resetPdiWatchdog();
        state_->resetSmWatchdog();
    }

    void setCallback(WatchdogCallback callback) {
        callback_ = std::move(callback);
    }

    WatchdogState& state() { return *state_; }
    const WatchdogState& state() const { return *state_; }

private:
    bool enabled_;
    WatchdogState* state_;
    WatchdogCallback callback_;
};

}} // namespace EtherCAT::slave
