// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file StateManager.hpp
 * @brief AL (Application Layer) state machine extracted from SlaveCore
 *
 * @details
 * Encapsulates the AL state machine sub-responsibility of SlaveCore:
 *  - State transition validation (canTransition)
 *  - State transition execution (doStateTransition, onEnterState, onExitState)
 *  - Error setting/clearing (setError, clearError)
 *  - AL status register updates
 *  - State change callback notification
 */

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>

#include "tether/slave/core/ALTypes.hpp"
#include "tether/slave/core/SlaveTypes.hpp"  // SlaveState
#include "tether/slave/core/SlaveCallbacks.hpp"  // StateChangeCallback

namespace EtherCAT { namespace slave {

class StateManager {
public:
    /// @param registers  Pointer to the ESC register bank (for AL status/code updates).
    explicit StateManager(std::array<uint8_t, 4096>* registers)
        : registers_(registers) {}

    // ---- Status access -----------------------------------------------------

    ALStatus status() const { return alStatus_; }
    ALStatusCode statusCode() const { return alStatusCode_; }

    // ---- State transitions -------------------------------------------------

    /// @brief Request a state change. Thread-safe.
    /// @return true if the transition is valid and was performed.
    bool requestStateChange(const ALControl& control);

    /// @brief Check if a state transition is valid per ETG.1000.
    static bool canTransition(SlaveState from, SlaveState to);

    /// @brief Set an error state. Thread-safe.
    void setError(ALStatusCode code);

    /// @brief Clear the error state. Thread-safe.
    void clearError();

    void setStateChangeCallback(StateChangeCallback callback) {
        stateChangeCallback_ = std::move(callback);
    }

    /// @brief Set the initial state (called once during initialization).
    void setInitialState(SlaveState state);

    /// @brief Register a callback invoked on entering INIT state (for watchdog reset).
    /// This breaks the dependency on WatchdogManager.
    void setOnInitCallback(std::function<void()> callback) {
        onInitCallback_ = std::move(callback);
    }

    // ---- Direct access (for SlaveCore compatibility) -----------------------

    ALStatus& statusRef() { return alStatus_; }
    std::mutex& mutex() { return stateMutex_; }

    // Locked variants (caller must hold stateMutex_)
    void setErrorLocked(ALStatusCode code);
    void clearErrorLocked();

private:
    void doStateTransition(SlaveState newState);
    void onEnterState(SlaveState state);
    void onExitState(SlaveState state);

    std::array<uint8_t, 4096>* registers_;
    ALStatus alStatus_{};
    ALStatusCode alStatusCode_ = ALStatusCode::NoError;
    StateChangeCallback stateChangeCallback_;
    std::function<void()> onInitCallback_;
    std::mutex stateMutex_;
};

}} // namespace EtherCAT::slave
