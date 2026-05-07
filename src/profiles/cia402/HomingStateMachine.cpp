/**
 * @file HomingStateMachine.cpp
 * @brief Core HomingStateMachine implementation with state handlers
 * 
 * Split from HomingModes.cpp for maintainability.
 */

#include "profiles/cia402/HomingModes.hpp"
#include <algorithm>
#include <sstream>

namespace CiA402 {
namespace Homing {

// ============================================================================
// HomingStateMachine Implementation - Core
// ============================================================================

HomingStateMachine::HomingStateMachine() {
    // Initialize timing
    stateStartTime_ = getTimeMs();
    homingStartTime_ = 0;
}

void HomingStateMachine::setConfig(const HomingConfig& config) {
    config_ = config;
}

void HomingStateMachine::setCallbacks(const HomingCallbacks& callbacks) {
    callbacks_ = callbacks;
}

void HomingStateMachine::setErrorInjection(const HomingErrorInjection& injection) {
    errorInjection_ = injection;
}

bool HomingStateMachine::start(HomingMethod method) {
    if (state_ != HomingState::Idle && state_ != HomingState::Error) {
        setError(HomingError::AlreadyHoming, "Homing already in progress");
        return false;
    }
    
    method_ = method;
    lastError_ = HomingError::None;
    lastErrorMessage_.clear();
    
    // Validate method
    if (method == HomingMethod::NoHoming) {
        setError(HomingError::InvalidMethod, "No homing method selected");
        return false;
    }
    
    // Validate configuration
    if (!validateConfiguration()) {
        return false;
    }
    
    // Validate endstop connectivity
    if (config_.validateEndstops && !validateEndstopConnectivity()) {
        return false;
    }
    
    // Start homing
    stats_.homingAttempts++;
    homingStartTime_ = getTimeMs();
    
    setState(HomingState::Starting);
    
    return true;
}

void HomingStateMachine::stop() {
    if (isHoming()) {
        if (callbacks_.stopMotion) {
            callbacks_.stopMotion();
        }
        setState(HomingState::Interrupted);
        
        if (callbacks_.onHomingInterrupted) {
            callbacks_.onHomingInterrupted();
        }
    }
}

void HomingStateMachine::reset() {
    if (callbacks_.stopMotion) {
        callbacks_.stopMotion();
    }
    
    state_ = HomingState::Idle;
    method_ = HomingMethod::NoHoming;
    lastError_ = HomingError::None;
    lastErrorMessage_.clear();
    subState_ = 0;
    indexSeen_ = false;
    indexCount_ = 0;
}

void HomingStateMachine::update() {
    // Update debounced switch states
    if (callbacks_.getSwitchStates) {
        rawStates_ = callbacks_.getSwitchStates();
        
        // Apply error injection
        if (errorInjection_.enabled) {
            if (errorInjection_.disconnectNegativeLimit) {
                rawStates_.negativeLimit = false;
            }
            if (errorInjection_.disconnectPositiveLimit) {
                rawStates_.positiveLimit = false;
            }
            if (errorInjection_.disconnectHomeSwitch) {
                rawStates_.homeSwitch = false;
            }
            if (errorInjection_.disconnectIndex) {
                rawStates_.indexPulse = false;
                rawStates_.indexDetected = false;
            }
        }
        
        // Simple debouncing - wait for stable state
        uint64_t now = getTimeMs();
        if (rawStates_.positiveLimit != debouncedStates_.positiveLimit ||
            rawStates_.negativeLimit != debouncedStates_.negativeLimit ||
            rawStates_.homeSwitch != debouncedStates_.homeSwitch) {
            
            if (!statesStable_) {
                lastStateChangeTime_ = now;
            }
            
            if (now - lastStateChangeTime_ >= config_.endstopDebounceMs) {
                debouncedStates_ = rawStates_;
                statesStable_ = true;
            } else {
                statesStable_ = false;
            }
        } else {
            statesStable_ = true;
        }
        
        // Index is latched, not debounced
        if (rawStates_.indexDetected) {
            debouncedStates_.indexDetected = true;
            debouncedStates_.lastIndexPosition = rawStates_.lastIndexPosition;
        }
    }
    
    // Run state machine
    switch (state_) {
        case HomingState::Idle:
            handleIdle();
            break;
        case HomingState::Starting:
            handleStarting();
            break;
        case HomingState::FindSwitch:
            handleFindSwitch();
            break;
        case HomingState::LeaveSwitch:
            handleLeaveSwitch();
            break;
        case HomingState::FindIndex:
            handleFindIndex();
            break;
        case HomingState::ZeroVelocity:
            handleZeroVelocity();
            break;
        case HomingState::Attained:
            handleAttained();
            break;
        case HomingState::Error:
            handleError();
            break;
        case HomingState::Interrupted:
            handleInterrupted();
            break;
    }
    
    // Check for drive faults
    checkMotionFaults();
}

bool HomingStateMachine::isHoming() const {
    return state_ != HomingState::Idle && 
           state_ != HomingState::Attained && 
           state_ != HomingState::Error &&
           state_ != HomingState::Interrupted;
}

bool HomingStateMachine::isComplete() const {
    return state_ == HomingState::Attained;
}

bool HomingStateMachine::hasError() const {
    return state_ == HomingState::Error;
}

// ============================================================================
// State Handlers
// ============================================================================

void HomingStateMachine::handleIdle() {
    // Nothing to do
}

void HomingStateMachine::handleStarting() {
    // Check drive is enabled
    if (callbacks_.hasDriveFault && callbacks_.hasDriveFault()) {
        setError(HomingError::DriveFault, "Drive has fault");
        return;
    }
    
    // Initialize based on method
    int8_t methodVal = static_cast<int8_t>(method_);
    
    if (methodVal >= 1 && methodVal <= 2) {
        // Limit switch + index
        if (methodVal == 1) {
            startNegativeLimitHoming(true);
        } else {
            startPositiveLimitHoming(true);
        }
    } else if (methodVal >= 3 && methodVal <= 14) {
        // Home switch + index
        startHomeSwitchHoming(method_);
    } else if (methodVal >= 17 && methodVal <= 18) {
        // Limit switch only
        if (methodVal == 17) {
            startNegativeLimitHoming(false);
        } else {
            startPositiveLimitHoming(false);
        }
    } else if (methodVal >= 19 && methodVal <= 30) {
        // Home switch only
        startHomeSwitchHoming(method_);
    } else if (methodVal == 33 || methodVal == 34) {
        // Index only
        startIndexOnlyHoming(methodVal == 33);
    } else if (methodVal == 35 || methodVal == 37) {
        // Current position
        startCurrentPositionHoming(methodVal == 35);
    } else {
        setError(HomingError::InvalidMethod, "Unknown homing method");
    }
}

void HomingStateMachine::handleFindSwitch() {
    checkTimeouts();
    if (state_ == HomingState::Error) return;
    
    // Check for error injection
    if (errorInjection_.enabled && errorInjection_.simulateTimeout) {
        return; // Never find the switch
    }
    
    bool switchFound = false;
    
    int8_t methodVal = static_cast<int8_t>(method_);
    
    // Check appropriate switch
    if (methodVal == 1 || methodVal == 17) {
        // Looking for negative limit
        if (debouncedStates_.negativeLimit) {
            switchFound = true;
        }
    } else if (methodVal == 2 || methodVal == 18) {
        // Looking for positive limit
        if (debouncedStates_.positiveLimit) {
            switchFound = true;
        }
    } else {
        // Looking for home switch
        if (lookingForRisingEdge_) {
            if (debouncedStates_.homeSwitch) {
                switchFound = true;
            }
        } else {
            if (!debouncedStates_.homeSwitch) {
                switchFound = true;
            }
        }
    }
    
    if (switchFound) {
        if (callbacks_.getPosition) {
            switchFoundPosition_ = callbacks_.getPosition();
        }
        
        // Determine if we need to leave switch or find index
        bool needsIndex = methodRequiresIndex(method_);
        bool needsLeave = (methodVal >= 1 && methodVal <= 14) ||
                          (methodVal >= 3 && methodVal <= 14);
        
        if (needsLeave) {
            // Reverse direction to leave switch
            searchDirection_ = -searchDirection_;
            if (callbacks_.setVelocity) {
                callbacks_.setVelocity(config_.zeroVelocity * searchDirection_);
            }
            setState(HomingState::LeaveSwitch);
        } else if (needsIndex) {
            // Move slowly to find index
            if (callbacks_.setVelocity) {
                callbacks_.setVelocity(config_.zeroVelocity * searchDirection_);
            }
            setState(HomingState::FindIndex);
        } else {
            // Done - stop and set home
            stopAndSetHome();
        }
    }
}

void HomingStateMachine::handleLeaveSwitch() {
    checkTimeouts();
    if (state_ == HomingState::Error) return;
    
    bool leftSwitch = false;
    
    int8_t methodVal = static_cast<int8_t>(method_);
    
    if (methodVal == 1 || methodVal == 17) {
        if (!debouncedStates_.negativeLimit) {
            leftSwitch = true;
        }
    } else if (methodVal == 2 || methodVal == 18) {
        if (!debouncedStates_.positiveLimit) {
            leftSwitch = true;
        }
    } else {
        // Home switch
        if (lookingForRisingEdge_) {
            if (!debouncedStates_.homeSwitch) {
                leftSwitch = true;
            }
        } else {
            if (debouncedStates_.homeSwitch) {
                leftSwitch = true;
            }
        }
    }
    
    if (leftSwitch) {
        bool needsIndex = methodRequiresIndex(method_);
        
        if (needsIndex) {
            // Clear index seen flag
            indexSeen_ = false;
            indexCount_ = 0;
            debouncedStates_.indexDetected = false;
            
            // Move slowly to find index
            if (callbacks_.setVelocity) {
                callbacks_.setVelocity(config_.zeroVelocity * searchDirection_);
            }
            stateStartTime_ = getTimeMs(); // Reset timeout for index search
            setState(HomingState::FindIndex);
        } else {
            // Done
            stopAndSetHome();
        }
    }
}

void HomingStateMachine::handleFindIndex() {
    // Check timeout
    uint64_t elapsed = getTimeMs() - stateStartTime_;
    if (elapsed > config_.indexTimeoutMs) {
        setError(HomingError::IndexTimeout, "Index pulse not found within timeout");
        return;
    }
    
    // Check for error injection
    if (errorInjection_.enabled && errorInjection_.simulateTimeout) {
        return;
    }
    
    if (debouncedStates_.indexDetected) {
        indexCount_++;
        
        // Check for multiple index pulses (error injection)
        if (errorInjection_.enabled && errorInjection_.simulateMultipleIndex) {
            indexCount_ = 5; // Simulate multiple pulses
        }
        
        // Validate single index per revolution if configured
        if (config_.validateIndex && indexCount_ > 1) {
            setError(HomingError::MultipleIndexPulses, 
                    "Multiple index pulses detected");
            return;
        }
        
        indexFoundPosition_ = debouncedStates_.lastIndexPosition;
        indexSeen_ = true;
        
        // Stop and set home
        stopAndSetHome();
    }
}

void HomingStateMachine::handleZeroVelocity() {
    // Wait for motion to stop
    bool motionStopped = true;
    
    if (callbacks_.getVelocity) {
        int32_t velocity = callbacks_.getVelocity();
        motionStopped = (std::abs(velocity) < 10); // Threshold
    }
    
    if (callbacks_.isMotionComplete) {
        motionStopped = callbacks_.isMotionComplete();
    }
    
    if (motionStopped) {
        // Set home position
        int32_t homePosition = config_.homeOffset;
        
        // If index was found, use that position
        if (indexSeen_ && methodRequiresIndex(method_)) {
            homePosition = indexFoundPosition_ + config_.homeOffset;
        }
        
        if (callbacks_.setHomePosition) {
            callbacks_.setHomePosition(homePosition);
        }
        
        stats_.lastHomePosition = homePosition;
        
        setState(HomingState::Attained);
    }
    
    // Check timeout
    uint64_t elapsed = getTimeMs() - stateStartTime_;
    if (elapsed > 5000) { // 5 second timeout for stopping
        setError(HomingError::MotionFault, "Failed to reach zero velocity");
    }
}

void HomingStateMachine::handleAttained() {
    // Homing complete
    uint64_t duration = getTimeMs() - homingStartTime_;
    stats_.lastHomingDurationMs = static_cast<uint32_t>(duration);
    stats_.totalHomingTimeMs += stats_.lastHomingDurationMs;
    stats_.successfulHomings++;
    
    if (callbacks_.onHomingComplete) {
        callbacks_.onHomingComplete(stats_.lastHomePosition);
    }
    
    // Return to idle
    setState(HomingState::Idle);
}

void HomingStateMachine::handleError() {
    // Stop motion
    if (config_.stopOnError && callbacks_.stopMotion) {
        callbacks_.stopMotion();
    }
    
    stats_.failedHomings++;
    
    // Categorize error for statistics
    int errorCode = static_cast<int>(lastError_);
    if (errorCode >= 0x8010 && errorCode <= 0x8016) {
        stats_.endstopErrors++;
    } else if (errorCode >= 0x8020 && errorCode <= 0x8023) {
        stats_.endstopErrors++;
    } else if (errorCode >= 0x8030 && errorCode <= 0x8034) {
        stats_.indexErrors++;
    } else if (errorCode >= 0x8040 && errorCode <= 0x8044) {
        stats_.motionErrors++;
    }
    
    if (errorCode == 0x8043) {
        stats_.timeoutErrors++;
    }
}

void HomingStateMachine::handleInterrupted() {
    // Homing was stopped externally
    setState(HomingState::Idle);
}

// ============================================================================
// Helper Methods
// ============================================================================

void HomingStateMachine::setState(HomingState newState) {
    if (state_ != newState) {
        state_ = newState;
        stateStartTime_ = getTimeMs();
        subState_ = 0;
        
        if (callbacks_.onStateChange) {
            callbacks_.onStateChange(newState);
        }
    }
}

uint64_t HomingStateMachine::getTimeMs() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()
    ).count();
}

void HomingStateMachine::setError(HomingError error, const std::string& message) {
    lastError_ = error;
    lastErrorMessage_ = message;
    setState(HomingState::Error);
    
    if (callbacks_.onError) {
        callbacks_.onError(error, message);
    }
}

void HomingStateMachine::checkTimeouts() {
    uint64_t elapsed = getTimeMs() - stateStartTime_;
    
    if (state_ == HomingState::FindSwitch || state_ == HomingState::LeaveSwitch) {
        if (elapsed > config_.searchTimeoutMs) {
            setError(HomingError::Timeout, "Homing search timeout");
        }
    }
}

void HomingStateMachine::checkMotionFaults() {
    if (callbacks_.hasDriveFault && callbacks_.hasDriveFault()) {
        setError(HomingError::DriveFault, "Drive fault during homing");
    }
    
    if (errorInjection_.enabled && errorInjection_.simulateFollowingError) {
        setError(HomingError::FollowingError, "Following error during homing");
    }
    
    if (errorInjection_.enabled && errorInjection_.simulateJam) {
        setError(HomingError::MotionFault, "Motor jam detected");
    }
}

void HomingStateMachine::stopAndSetHome() {
    if (callbacks_.stopMotion) {
        callbacks_.stopMotion();
    }
    setState(HomingState::ZeroVelocity);
}

} // namespace Homing
} // namespace CiA402
