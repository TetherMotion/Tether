/**
 * @file HomingHandler.cpp
 * @brief CiA 402 Homing Mode Implementation
 */

#include "profiles/cia402/HomingHandler.hpp"
#include "tether/platform/EspCompat.hpp"

static const char* TAG = "HomingHandler";

namespace CiA402 {

HomingHandler::HomingHandler() {
    m_params.method = static_cast<int8_t>(HomingMethod::CurrentPosition);
    m_params.speedSwitch = static_cast<uint32_t>(CIA402_DEFAULT_HOMING_VELOCITY);
    m_params.speedZero = static_cast<uint32_t>(CIA402_DEFAULT_HOMING_VELOCITY_SLOW);
    m_params.acceleration = static_cast<uint32_t>(CIA402_DEFAULT_HOMING_ACCELERATION);
    m_params.offset = 0;
}

void HomingHandler::configure(const HomingParams& params) {
    m_params = params;
    m_method = static_cast<HomingMethod>(params.method);
}

void HomingHandler::setSwitchCallback(HomingStatusCallback callback) {
    m_switchCallback = callback;
}

void HomingHandler::setCompleteCallback(HomingCompleteCallback callback) {
    m_completeCallback = callback;
}

bool HomingHandler::start() {
    if (m_state != HomingState::Idle && m_state != HomingState::Attained) {
        TETHER_LOGW(TAG, "Homing already in progress");
        return false;
    }
    
    TETHER_LOGI(TAG, "Starting homing method %d", static_cast<int>(m_method));
    
    m_state = HomingState::SearchingSwitch;
    m_error = HomingError::None;
    m_switchFound = false;
    m_indexFound = false;
    m_startTime = esp_timer_get_time();
    
    // Get initial search direction
    m_searchDirection = getInitialDirection(m_method);
    
    // Handle immediate methods
    if (m_method == HomingMethod::CurrentPosition ||
        m_method == HomingMethod::CurrentPositionIndex) {
        // No motion required
        return true;
    }
    
    // Start motion
    int32_t velocity = m_params.speedSwitch * m_searchDirection;
    setVelocity(velocity);
    
    return true;
}

void HomingHandler::abort() {
    if (m_state == HomingState::Idle || m_state == HomingState::Attained) {
        return;
    }
    
    TETHER_LOGI(TAG, "Homing aborted");
    stop();
    complete(false, HomingError::Aborted);
}

bool HomingHandler::update(int32_t currentPosition) {
    if (m_state == HomingState::Idle || m_state == HomingState::Attained ||
        m_state == HomingState::Error) {
        return false;
    }
    
    // Check timeout
    uint64_t elapsed = (esp_timer_get_time() - m_startTime) / 1000;
    if (elapsed > m_timeoutMs) {
        TETHER_LOGE(TAG, "Homing timeout");
        stop();
        complete(false, HomingError::Timeout);
        return false;
    }
    
    // Get switch status
    HomingSwitchStatus status{};
    if (m_switchCallback) {
        status = m_switchCallback();
    }
    
    // Process based on method
    int methodNum = static_cast<int>(m_method);
    
    if (m_method == HomingMethod::CurrentPosition ||
        m_method == HomingMethod::CurrentPositionIndex) {
        processCurrentPosition(currentPosition);
    }
    else if (methodNum >= 1 && methodNum <= 2) {
        if (methodNum == 1) {
            processMethod1(currentPosition, status);
        } else {
            processMethod2(currentPosition, status);
        }
    }
    else if (methodNum >= 3 && methodNum <= 6) {
        processHomeSwitchWithIndex(currentPosition, status);
    }
    else if (methodNum >= 7 && methodNum <= 14) {
        processHomeSwitchOnly(currentPosition, status);
    }
    else if (methodNum >= 17 && methodNum <= 30) {
        processBlockHoming(currentPosition, status);
    }
    else if (methodNum >= 33 && methodNum <= 34) {
        processIndexOnly(currentPosition, status);
    }
    else {
        TETHER_LOGE(TAG, "Invalid homing method: %d", methodNum);
        complete(false, HomingError::InvalidMethod);
        return false;
    }
    
    return m_state == HomingState::SearchingSwitch ||
           m_state == HomingState::Reversing ||
           m_state == HomingState::SearchingIndex;
}

void HomingHandler::processMethod1(int32_t position, const HomingSwitchStatus& status) {
    // Method 1: Move negative until negative limit, then find index
    
    switch (m_state) {
        case HomingState::SearchingSwitch:
            if (status.negativeLimitActive) {
                TETHER_LOGD(TAG, "Negative limit found");
                m_switchFound = true;
                m_searchDirection = 1;  // Reverse
                setVelocity(m_params.speedZero);
                m_state = HomingState::SearchingIndex;
            }
            break;
            
        case HomingState::SearchingIndex:
            if (status.indexPulseDetected) {
                TETHER_LOGD(TAG, "Index found at %ld", (long)position);
                stop();
                m_homePosition = position + m_params.offset;
                complete(true);
            }
            break;
            
        default:
            break;
    }
}

void HomingHandler::processMethod2(int32_t position, const HomingSwitchStatus& status) {
    // Method 2: Move positive until positive limit, then find index
    
    switch (m_state) {
        case HomingState::SearchingSwitch:
            if (status.positiveLimitActive) {
                TETHER_LOGD(TAG, "Positive limit found");
                m_switchFound = true;
                m_searchDirection = -1;  // Reverse
                setVelocity(-static_cast<int32_t>(m_params.speedZero));
                m_state = HomingState::SearchingIndex;
            }
            break;
            
        case HomingState::SearchingIndex:
            if (status.indexPulseDetected) {
                TETHER_LOGD(TAG, "Index found at %ld", (long)position);
                stop();
                m_homePosition = position + m_params.offset;
                complete(true);
            }
            break;
            
        default:
            break;
    }
}

void HomingHandler::processHomeSwitchWithIndex(int32_t position, 
                                                const HomingSwitchStatus& status) {
    int methodNum = static_cast<int>(m_method);
    
    // Methods 3-6: Home switch with index
    bool positiveFirst = (methodNum == 3 || methodNum == 4);
    bool reverseOnSwitch = (methodNum == 4 || methodNum == 6);
    
    switch (m_state) {
        case HomingState::SearchingSwitch:
            if (status.homeSwitchActive) {
                TETHER_LOGD(TAG, "Home switch found");
                m_switchFound = true;
                
                if (reverseOnSwitch) {
                    m_searchDirection = -m_searchDirection;
                }
                
                setVelocity(m_params.speedZero * m_searchDirection);
                m_state = HomingState::SearchingIndex;
            }
            break;
            
        case HomingState::SearchingIndex:
            if (status.indexPulseDetected) {
                TETHER_LOGD(TAG, "Index found at %ld", (long)position);
                stop();
                m_homePosition = position + m_params.offset;
                complete(true);
            }
            break;
            
        default:
            break;
    }
}

void HomingHandler::processHomeSwitchOnly(int32_t position, 
                                          const HomingSwitchStatus& status) {
    int methodNum = static_cast<int>(m_method);
    
    // Methods 7-14: Home switch without index
    // Determine behavior based on method number
    bool searchPositive = (methodNum == 7 || methodNum == 8 || methodNum == 11 || methodNum == 12);
    bool stopOnRising = (methodNum == 7 || methodNum == 9 || methodNum == 11 || methodNum == 13);
    
    switch (m_state) {
        case HomingState::SearchingSwitch:
            if (status.homeSwitchActive) {
                if (stopOnRising) {
                    // Found rising edge - this is home
                    TETHER_LOGD(TAG, "Home switch rising edge at %ld", (long)position);
                    stop();
                    m_homePosition = position + m_params.offset;
                    complete(true);
                } else {
                    // Need falling edge - reverse
                    m_searchDirection = -m_searchDirection;
                    setVelocity(m_params.speedZero * m_searchDirection);
                    m_state = HomingState::Reversing;
                }
            }
            break;
            
        case HomingState::Reversing:
            if (!status.homeSwitchActive) {
                // Found falling edge - this is home
                TETHER_LOGD(TAG, "Home switch falling edge at %ld", (long)position);
                stop();
                m_homePosition = position + m_params.offset;
                complete(true);
            }
            break;
            
        default:
            break;
    }
}

void HomingHandler::processBlockHoming(int32_t position, 
                                       const HomingSwitchStatus& status) {
    // Methods 17-30: Block detection (mechanical stop)
    // Uses torque/current limiting to detect block
    
    // Simplified implementation - check for stall or limit
    if (status.negativeLimitActive || status.positiveLimitActive) {
        TETHER_LOGD(TAG, "Block/limit detected at %ld", (long)position);
        stop();
        m_homePosition = position + m_params.offset;
        complete(true);
    }
}

void HomingHandler::processIndexOnly(int32_t position, 
                                     const HomingSwitchStatus& status) {
    // Methods 33-34: Index pulse only
    
    if (status.indexPulseDetected) {
        TETHER_LOGD(TAG, "Index pulse at %ld", (long)position);
        stop();
        m_homePosition = position + m_params.offset;
        complete(true);
    }
}

void HomingHandler::processCurrentPosition(int32_t position) {
    // Methods 35/37: Set current position as home
    TETHER_LOGI(TAG, "Setting current position %ld as home", (long)position);
    m_homePosition = position + m_params.offset;
    complete(true);
}

void HomingHandler::setVelocity(int32_t velocity) {
    if (m_backend) {
        m_backend->setTargetVelocity(velocity);
    }
}

void HomingHandler::stop() {
    if (m_backend) {
        m_backend->setTargetVelocity(0);
    }
}

void HomingHandler::complete(bool success, HomingError error) {
    m_state = success ? HomingState::Attained : HomingState::Error;
    m_error = error;
    
    if (success) {
        TETHER_LOGI(TAG, "Homing complete, home position: %ld", (long)m_homePosition);
    } else {
        TETHER_LOGE(TAG, "Homing failed, error: %d", static_cast<int>(error));
    }
    
    if (m_completeCallback) {
        m_completeCallback(success, error);
    }
}

bool HomingHandler::methodRequiresLimit(HomingMethod method) {
    int m = static_cast<int>(method);
    return (m >= 1 && m <= 2) || (m >= 17 && m <= 30);
}

bool HomingHandler::methodRequiresHome(HomingMethod method) {
    int m = static_cast<int>(method);
    return (m >= 3 && m <= 14);
}

bool HomingHandler::methodRequiresIndex(HomingMethod method) {
    int m = static_cast<int>(method);
    return (m >= 1 && m <= 6) || (m >= 33 && m <= 34);
}

int HomingHandler::getInitialDirection(HomingMethod method) {
    int m = static_cast<int>(method);
    
    // Negative initial direction
    if (m == 1 || m == 5 || m == 6 || m == 9 || m == 10 ||
        m == 13 || m == 14 || m == 17 || m == 18 || m == 21 ||
        m == 22 || m == 25 || m == 26 || m == 33) {
        return -1;
    }
    
    // Positive initial direction
    return 1;
}

const char* HomingHandler::getMethodDescription(HomingMethod method) {
    switch (method) {
        case HomingMethod::NoHoming:
            return "No homing";
        case HomingMethod::NegLimitIndex:
            return "Negative limit + index";
        case HomingMethod::PosLimitIndex:
            return "Positive limit + index";
        case HomingMethod::HomeSwitchPosIndex:
            return "Positive home + index, positive direction";
        case HomingMethod::HomeSwitchPosIndex2:
            return "Positive home + index, negative direction";
        case HomingMethod::HomeSwitchNegIndex:
            return "Negative home + index, negative direction";
        case HomingMethod::HomeSwitchNegIndex2:
            return "Negative home + index, positive direction";
        case HomingMethod::HomeSwitchPosIndexPos:
            return "Positive home, positive direction";
        case HomingMethod::HomeSwitchPosIndexPos2:
            return "Positive home, negative direction";
        case HomingMethod::HomeSwitchPosIndexNeg:
            return "Negative home, positive direction";
        case HomingMethod::HomeSwitchPosIndexNeg2:
            return "Negative home, negative direction";
        case HomingMethod::HomeSwitchNegIndexPos:
            return "Positive home, falling edge";
        case HomingMethod::HomeSwitchNegIndexPos2:
            return "Positive home, falling edge, reverse";
        case HomingMethod::HomeSwitchNegIndexNeg:
            return "Negative home, falling edge";
        case HomingMethod::HomeSwitchNegIndexNeg2:
            return "Negative home, falling edge, reverse";
        case HomingMethod::NegLimitOnly:
            return "Negative limit only";
        case HomingMethod::PosLimitOnly:
            return "Positive limit only";
        case HomingMethod::NegDirIndexPulse:
            return "Negative index pulse";
        case HomingMethod::PosDirIndexPulse:
            return "Positive index pulse";
        case HomingMethod::CurrentPosition:
            return "Current position as home";
        case HomingMethod::CurrentPositionIndex:
            return "Current position as home (alt)";
        default:
            return "Unknown method";
    }
}

} // namespace CiA402
