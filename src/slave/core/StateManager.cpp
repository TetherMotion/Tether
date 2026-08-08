// SPDX-License-Identifier: MIT

#include "tether/slave/core/StateManager.hpp"

#include "tether/slave/core/SlaveTypes.hpp"  // ESCReg alias

#include <cstring>

namespace EtherCAT { namespace slave {

// ============================================================================
// Initialization
// ============================================================================

void StateManager::setInitialState(SlaveState state) {
    alStatus_.state = state;
    alStatus_.error = false;

    // Write initial AL status to registers
    uint16_t alStatusReg = alStatus_.toRegister();
    (*registers_)[ESCReg::ALStatus] = alStatusReg & 0xFF;
    (*registers_)[ESCReg::ALStatus + 1] = (alStatusReg >> 8) & 0xFF;
}

// ============================================================================
// State transitions
// ============================================================================

bool StateManager::requestStateChange(const ALControl& control) {
    std::lock_guard<std::mutex> lock(stateMutex_);

    // Check if transition is valid
    if (!canTransition(alStatus_.state, control.requestedState)) {
        setErrorLocked(ALStatusCode::InvalidStateChange);
        return false;
    }

    // Handle error acknowledgement
    if (control.acknowledgeError && alStatus_.error) {
        clearErrorLocked();
    }

    // Perform state change
    doStateTransition(control.requestedState);
    return true;
}

bool StateManager::canTransition(SlaveState from, SlaveState to) {
    // Valid transitions per ETG.1000
    if (from == to) return true;

    switch (from) {
        case SlaveState::INIT:
            return to == SlaveState::PRE_OP || to == SlaveState::BOOT;
        case SlaveState::PRE_OP:
            return to == SlaveState::INIT || to == SlaveState::SAFE_OP;
        case SlaveState::BOOT:
            return to == SlaveState::INIT;
        case SlaveState::SAFE_OP:
            return to == SlaveState::INIT || to == SlaveState::PRE_OP || to == SlaveState::OP;
        case SlaveState::OP:
            return to == SlaveState::INIT || to == SlaveState::PRE_OP || to == SlaveState::SAFE_OP;
        default:
            return false;
    }
}

void StateManager::doStateTransition(SlaveState newState) {
    SlaveState oldState = alStatus_.state;

    onExitState(oldState);
    alStatus_.state = newState;
    onEnterState(newState);

    // Update AL status register
    uint16_t statusReg = alStatus_.toRegister();
    (*registers_)[ESCReg::ALStatus] = statusReg & 0xFF;
    (*registers_)[ESCReg::ALStatus + 1] = (statusReg >> 8) & 0xFF;

    // Notify callback
    if (stateChangeCallback_) {
        stateChangeCallback_(oldState, newState);
    }
}

void StateManager::onEnterState(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:
            // Reset watchdog via callback
            if (onInitCallback_) onInitCallback_();
            break;
        case SlaveState::PRE_OP:
            // Mailbox communication starts
            break;
        case SlaveState::SAFE_OP:
            // Inputs active, outputs safe
            break;
        case SlaveState::OP:
            // Full operation
            break;
        default:
            break;
    }
}

void StateManager::onExitState(SlaveState state) {
    (void)state;  // Currently no exit actions
}

// ============================================================================
// Error handling
// ============================================================================

void StateManager::setErrorLocked(ALStatusCode code) {
    alStatusCode_ = code;
    alStatus_.error = true;

    // Update registers
    (*registers_)[ESCReg::ALStatusCode] = static_cast<uint16_t>(code) & 0xFF;
    (*registers_)[ESCReg::ALStatusCode + 1] = (static_cast<uint16_t>(code) >> 8) & 0xFF;

    uint16_t statusReg = alStatus_.toRegister();
    (*registers_)[ESCReg::ALStatus] = statusReg & 0xFF;
    (*registers_)[ESCReg::ALStatus + 1] = (statusReg >> 8) & 0xFF;
}

void StateManager::setError(ALStatusCode code) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    setErrorLocked(code);
}

void StateManager::clearErrorLocked() {
    alStatusCode_ = ALStatusCode::NoError;
    alStatus_.error = false;

    (*registers_)[ESCReg::ALStatusCode] = 0;
    (*registers_)[ESCReg::ALStatusCode + 1] = 0;

    uint16_t statusReg = alStatus_.toRegister();
    (*registers_)[ESCReg::ALStatus] = statusReg & 0xFF;
    (*registers_)[ESCReg::ALStatus + 1] = (statusReg >> 8) & 0xFF;
}

void StateManager::clearError() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    clearErrorLocked();
}

}} // namespace EtherCAT::slave
