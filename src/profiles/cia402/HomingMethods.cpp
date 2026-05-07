/**
 * @file HomingMethods.cpp
 * @brief Homing method implementations and validation
 * 
 * Split from HomingModes.cpp for maintainability.
 */

#include "profiles/cia402/HomingModes.hpp"
#include <algorithm>
#include <sstream>

namespace CiA402 {
namespace Homing {

// ============================================================================
// Validation Methods
// ============================================================================

bool HomingStateMachine::validateConfiguration() {
    // Check required switches are configured
    if (methodRequiresNegativeLimit(method_)) {
        // Check callback can report limit state
        if (!callbacks_.getSwitchStates) {
            setError(HomingError::NegativeLimitNotConfigured,
                    "Negative limit required but no switch state callback");
            return false;
        }
    }
    
    if (methodRequiresPositiveLimit(method_)) {
        if (!callbacks_.getSwitchStates) {
            setError(HomingError::PositiveLimitNotConfigured,
                    "Positive limit required but no switch state callback");
            return false;
        }
    }
    
    if (methodRequiresHomeSwitch(method_)) {
        if (!callbacks_.getSwitchStates) {
            setError(HomingError::HomeSwitchNotConfigured,
                    "Home switch required but no switch state callback");
            return false;
        }
    }
    
    if (methodRequiresIndex(method_)) {
        if (!callbacks_.getSwitchStates) {
            setError(HomingError::IndexNotConfigured,
                    "Index pulse required but no switch state callback");
            return false;
        }
    }
    
    // Check motion callbacks
    if (!callbacks_.setVelocity || !callbacks_.stopMotion) {
        setError(HomingError::NotInitialized,
                "Motion callbacks not configured");
        return false;
    }
    
    return true;
}

bool HomingStateMachine::validateEndstopConnectivity() {
    if (!callbacks_.getSwitchStates) {
        return true;
    }
    
    SwitchStates states = callbacks_.getSwitchStates();
    
    // For NC (normally closed) endstops, disconnection shows as triggered
    // For NO (normally open) endstops, disconnection shows as not triggered
    // This is a simple heuristic - proper detection needs hardware support
    
    // Check for both limits triggered simultaneously (likely wiring issue)
    if (states.positiveLimit && states.negativeLimit) {
        setError(HomingError::BothLimitsActive,
                "Both limit switches active - possible wiring fault");
        return false;
    }
    
    return true;
}

bool HomingStateMachine::checkEndstopDisconnection() {
    // This is called during homing to detect disconnections
    // A properly designed system should have hardware detection
    
    if (errorInjection_.enabled) {
        if (errorInjection_.disconnectNegativeLimit && 
            methodRequiresNegativeLimit(method_)) {
            setError(HomingError::NegativeLimitDisconnected,
                    "Negative limit switch disconnected");
            return true;
        }
        if (errorInjection_.disconnectPositiveLimit &&
            methodRequiresPositiveLimit(method_)) {
            setError(HomingError::PositiveLimitDisconnected,
                    "Positive limit switch disconnected");
            return true;
        }
        if (errorInjection_.disconnectHomeSwitch &&
            methodRequiresHomeSwitch(method_)) {
            setError(HomingError::HomeSwitchDisconnected,
                    "Home switch disconnected");
            return true;
        }
        if (errorInjection_.disconnectIndex &&
            methodRequiresIndex(method_)) {
            setError(HomingError::IndexDisconnected,
                    "Index pulse disconnected");
            return true;
        }
    }
    
    return false;
}

// ============================================================================
// Homing Method Implementations
// ============================================================================

void HomingStateMachine::startNegativeLimitHoming(bool withIndex) {
    // Check if already at negative limit
    if (debouncedStates_.negativeLimit) {
        // Need to back off first
        searchDirection_ = 1; // Move positive
        subState_ = 1; // Backing off
    } else {
        searchDirection_ = -1; // Move negative
        subState_ = 0; // Searching
    }
    
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(config_.searchVelocity * searchDirection_);
    }
    
    setState(HomingState::FindSwitch);
}

void HomingStateMachine::startPositiveLimitHoming(bool withIndex) {
    if (debouncedStates_.positiveLimit) {
        searchDirection_ = -1; // Move negative
        subState_ = 1;
    } else {
        searchDirection_ = 1; // Move positive
        subState_ = 0;
    }
    
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(config_.searchVelocity * searchDirection_);
    }
    
    setState(HomingState::FindSwitch);
}

void HomingStateMachine::startHomeSwitchHoming(HomingMethod method) {
    int8_t methodVal = static_cast<int8_t>(method);
    
    // Determine initial direction and edge type based on method
    if (methodVal == 3 || methodVal == 19) {
        searchDirection_ = 1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 4 || methodVal == 20) {
        searchDirection_ = -1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 5 || methodVal == 21) {
        searchDirection_ = -1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 6 || methodVal == 22) {
        searchDirection_ = 1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 7 || methodVal == 23) {
        searchDirection_ = 1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 8 || methodVal == 24) {
        searchDirection_ = 1;
        lookingForRisingEdge_ = false;
    } else if (methodVal == 9 || methodVal == 25) {
        searchDirection_ = -1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 10 || methodVal == 26) {
        searchDirection_ = -1;
        lookingForRisingEdge_ = false;
    } else if (methodVal == 11 || methodVal == 27) {
        searchDirection_ = 1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 12 || methodVal == 28) {
        searchDirection_ = -1;
        lookingForRisingEdge_ = true;
    } else if (methodVal == 13 || methodVal == 29) {
        searchDirection_ = 1;
        lookingForRisingEdge_ = true;
        reverseAfterSwitch_ = true;
    } else if (methodVal == 14 || methodVal == 30) {
        searchDirection_ = -1;
        lookingForRisingEdge_ = true;
        reverseAfterSwitch_ = true;
    }
    
    // Check if already on switch
    if (debouncedStates_.homeSwitch) {
        // Move off switch first
        setState(HomingState::LeaveSwitch);
    } else {
        setState(HomingState::FindSwitch);
    }
    
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(config_.searchVelocity * searchDirection_);
    }
}

void HomingStateMachine::startIndexOnlyHoming(bool negativeFirst) {
    searchDirection_ = negativeFirst ? -1 : 1;
    indexSeen_ = false;
    indexCount_ = 0;
    debouncedStates_.indexDetected = false;
    
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(config_.zeroVelocity * searchDirection_);
    }
    
    setState(HomingState::FindIndex);
}

void HomingStateMachine::startCurrentPositionHoming(bool withIndex) {
    if (withIndex) {
        // Method 35: Move to index
        searchDirection_ = 1;
        indexSeen_ = false;
        indexCount_ = 0;
        
        if (callbacks_.setVelocity) {
            callbacks_.setVelocity(config_.zeroVelocity * searchDirection_);
        }
        
        setState(HomingState::FindIndex);
    } else {
        // Method 37: Current position is home
        stopAndSetHome();
    }
}

// ============================================================================
// Motion Helpers
// ============================================================================

void HomingStateMachine::movePositive() {
    searchDirection_ = 1;
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(config_.searchVelocity);
    }
}

void HomingStateMachine::moveNegative() {
    searchDirection_ = -1;
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(-config_.searchVelocity);
    }
}

void HomingStateMachine::moveToIndex() {
    if (callbacks_.setVelocity) {
        callbacks_.setVelocity(config_.zeroVelocity * searchDirection_);
    }
}

// ============================================================================
// Static Method Helpers
// ============================================================================

std::string HomingStateMachine::getMethodName(HomingMethod method) {
    switch (method) {
        case HomingMethod::NoHoming: return "No Homing";
        case HomingMethod::MethodNegLimitIndex: return "Negative Limit + Index";
        case HomingMethod::MethodPosLimitIndex: return "Positive Limit + Index";
        case HomingMethod::MethodNegLimit: return "Negative Limit";
        case HomingMethod::MethodPosLimit: return "Positive Limit";
        case HomingMethod::MethodIndexNeg: return "Index (Negative Direction)";
        case HomingMethod::MethodIndexPos: return "Index (Positive Direction)";
        case HomingMethod::MethodCurrentPosition: return "Current Position";
        case HomingMethod::MethodCurrentPositionIndex: return "Current Position + Index";
        default: {
            int8_t val = static_cast<int8_t>(method);
            if (val >= 3 && val <= 14) return "Home Switch + Index (Method " + std::to_string(val) + ")";
            if (val >= 19 && val <= 30) return "Home Switch (Method " + std::to_string(val) + ")";
            return "Unknown Method";
        }
    }
}

bool HomingStateMachine::methodRequiresNegativeLimit(HomingMethod method) {
    int8_t val = static_cast<int8_t>(method);
    return val == 1 || val == 17;
}

bool HomingStateMachine::methodRequiresPositiveLimit(HomingMethod method) {
    int8_t val = static_cast<int8_t>(method);
    return val == 2 || val == 18;
}

bool HomingStateMachine::methodRequiresHomeSwitch(HomingMethod method) {
    int8_t val = static_cast<int8_t>(method);
    return (val >= 3 && val <= 14) || (val >= 19 && val <= 30);
}

bool HomingStateMachine::methodRequiresIndex(HomingMethod method) {
    int8_t val = static_cast<int8_t>(method);
    return (val >= 1 && val <= 14) || val == 33 || val == 34 || val == 35;
}

} // namespace Homing
} // namespace CiA402
