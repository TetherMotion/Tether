/**
 * @file CiA402StateMachine.cpp
 * @brief CiA 402 state machine implementation
 */

#include "profiles/cia402/CiA402StateMachine.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"
#include "tether/platform/EspCompat.hpp"

static const char* TAG = "CiA402SM";

namespace CiA402 {

StateMachine::StateMachine()
    : m_readStatus(nullptr)
    , m_writeControl(nullptr)
    , m_modeCallback(nullptr)
    , m_currentState(State::NotReadyToSwitchOn)
    , m_previousState(State::NotReadyToSwitchOn)
    , m_targetState(State::NotReadyToSwitchOn)
    , m_operatingMode(OperatingMode::ProfilePosition)
    , m_displayedMode(OperatingMode::ProfilePosition)
    , m_statusWord(0)
    , m_controlWord(0)
    , m_haltActive(false)
    , m_transitionPending(false)
{
}

void StateMachine::setCallbacks(StatusWordCallback readStatus, 
                                ControlWordCallback writeControl) {
    m_readStatus = readStatus;
    m_writeControl = writeControl;
}

void StateMachine::setModeCallback(ModeCallback modeCallback) {
    m_modeCallback = modeCallback;
}

State StateMachine::update() {
    if (!m_readStatus) {
        return m_currentState;
    }
    
    m_statusWord = m_readStatus();
    State newState = decodeState(m_statusWord);
    
    if (newState != m_currentState) {
        m_previousState = m_currentState;
        m_currentState = newState;
        
        TETHER_LOGD(TAG, "State transition: %d -> %d", 
                 static_cast<int>(m_previousState), 
                 static_cast<int>(m_currentState));
        
        // Check if we reached target state
        if (m_transitionPending && m_currentState == m_targetState) {
            m_transitionPending = false;
        }
    }
    
    return m_currentState;
}

State StateMachine::decodeState(uint16_t statusWord) const {
    // State decoding according to CiA 402
    // Bits: Ready to Switch On (0), Switched On (1), Operation Enabled (2),
    //       Fault (3), Voltage Enabled (4), Quick Stop (5), Switch On Disabled (6)
    
    if (StatusWord::isFault(statusWord)) {
        return State::Fault;
    }
    if (StatusWord::isFaultReactionActive(statusWord)) {
        return State::FaultReactionActive;
    }
    if (StatusWord::isSwitchOnDisabled(statusWord)) {
        return State::SwitchOnDisabled;
    }
    if (StatusWord::isQuickStopActive(statusWord)) {
        return State::QuickStopActive;
    }
    if (StatusWord::isOperationEnabled(statusWord)) {
        return State::OperationEnabled;
    }
    if (StatusWord::isSwitchedOn(statusWord)) {
        return State::SwitchedOn;
    }
    if (StatusWord::isReadyToSwitchOn(statusWord)) {
        return State::ReadyToSwitchOn;
    }
    
    return State::NotReadyToSwitchOn;
}

TransitionResult StateMachine::requestState(State targetState, uint32_t timeoutMs) {
    if (!m_readStatus || !m_writeControl) {
        TETHER_LOGE(TAG, "Callbacks not configured");
        return TransitionResult::InvalidTransition;
    }
    
    // Update current state
    update();
    
    // Already at target
    if (m_currentState == targetState) {
        return TransitionResult::Success;
    }
    
    // Cannot transition out of fault without reset
    if (m_currentState == State::Fault && targetState != State::SwitchOnDisabled) {
        TETHER_LOGE(TAG, "Must reset fault first");
        return TransitionResult::FaultOccurred;
    }
    
    m_targetState = targetState;
    m_transitionPending = true;
    
    int64_t startTime = esp_timer_get_time();
    int64_t timeoutUs = static_cast<int64_t>(timeoutMs) * 1000;
    
    while (m_currentState != targetState) {
        // Check for timeout
        if (timeoutMs > 0 && (esp_timer_get_time() - startTime) > timeoutUs) {
            m_transitionPending = false;
            TETHER_LOGW(TAG, "Transition timeout");
            return TransitionResult::Timeout;
        }
        
        // Check for fault
        update();
        if (m_currentState == State::Fault || m_currentState == State::FaultReactionActive) {
            m_transitionPending = false;
            return TransitionResult::FaultOccurred;
        }
        
        // Get intermediate state if needed
        State nextState = getIntermediateState(m_currentState, targetState);
        
        // Calculate and send control word
        uint16_t newControlWord = calculateControlWord(m_currentState, nextState);
        
        // Preserve non-transition bits
        m_controlWord = (m_controlWord & ~ControlWord::TransitionMask()) | 
                        (newControlWord & ControlWord::TransitionMask());
        
        // Set halt bit if needed
        if (m_haltActive) {
            m_controlWord |= static_cast<uint16_t>(ControlWordBit::Halt);
        } else {
            m_controlWord &= ~static_cast<uint16_t>(ControlWordBit::Halt);
        }
        
        m_writeControl(m_controlWord);
        
        // Small delay before next check
        if (timeoutMs > 0) {
            Tether::Platform::Clock::instance().delayMilliseconds(1);
        } else {
            break;  // No wait mode - single attempt
        }
    }
    
    m_transitionPending = false;
    return (m_currentState == targetState) ? TransitionResult::Success : TransitionResult::Pending;
}

bool StateMachine::executeTransition(uint16_t controlWordMask) {
    if (!m_writeControl) {
        return false;
    }
    
    m_controlWord = (m_controlWord & ~ControlWord::TransitionMask()) | 
                    (controlWordMask & ControlWord::TransitionMask());
    m_writeControl(m_controlWord);
    return true;
}

State StateMachine::getIntermediateState(State current, State target) const {
    // Determine next step in multi-step transition
    
    switch (current) {
        case State::SwitchOnDisabled:
            if (target == State::ReadyToSwitchOn || 
                target == State::SwitchedOn || 
                target == State::OperationEnabled) {
                return State::ReadyToSwitchOn;
            }
            break;
            
        case State::ReadyToSwitchOn:
            if (target == State::SwitchOnDisabled) {
                return State::SwitchOnDisabled;
            }
            if (target == State::SwitchedOn || target == State::OperationEnabled) {
                return State::SwitchedOn;
            }
            break;
            
        case State::SwitchedOn:
            if (target == State::SwitchOnDisabled) {
                return State::SwitchOnDisabled;
            }
            if (target == State::ReadyToSwitchOn) {
                return State::ReadyToSwitchOn;
            }
            if (target == State::OperationEnabled) {
                return State::OperationEnabled;
            }
            break;
            
        case State::OperationEnabled:
            if (target == State::SwitchOnDisabled) {
                return State::SwitchOnDisabled;
            }
            if (target == State::ReadyToSwitchOn) {
                return State::ReadyToSwitchOn;
            }
            if (target == State::SwitchedOn) {
                return State::SwitchedOn;
            }
            if (target == State::QuickStopActive) {
                return State::QuickStopActive;
            }
            break;
            
        case State::QuickStopActive:
            if (target == State::SwitchOnDisabled) {
                return State::SwitchOnDisabled;
            }
            break;
            
        case State::Fault:
            // Must use resetFault() first
            return State::SwitchOnDisabled;
            
        default:
            break;
    }
    
    return target;
}

uint16_t StateMachine::calculateControlWord(State currentState, State targetState) const {
    // Control word for state transitions
    
    switch (currentState) {
        case State::SwitchOnDisabled:
            if (targetState == State::ReadyToSwitchOn) {
                return ControlWord::Shutdown();
            }
            break;
            
        case State::ReadyToSwitchOn:
            if (targetState == State::SwitchedOn) {
                return ControlWord::SwitchOn();
            }
            if (targetState == State::SwitchOnDisabled) {
                return ControlWord::DisableVoltage();
            }
            if (targetState == State::OperationEnabled) {
                return ControlWord::SwitchOnEnable();
            }
            break;
            
        case State::SwitchedOn:
            if (targetState == State::OperationEnabled) {
                return ControlWord::EnableOperation();
            }
            if (targetState == State::ReadyToSwitchOn) {
                return ControlWord::Shutdown();
            }
            if (targetState == State::SwitchOnDisabled) {
                return ControlWord::DisableVoltage();
            }
            break;
            
        case State::OperationEnabled:
            if (targetState == State::SwitchedOn) {
                return ControlWord::DisableOperation();
            }
            if (targetState == State::ReadyToSwitchOn) {
                return ControlWord::Shutdown();
            }
            if (targetState == State::SwitchOnDisabled) {
                return ControlWord::DisableVoltage();
            }
            if (targetState == State::QuickStopActive) {
                return ControlWord::QuickStop();
            }
            break;
            
        case State::QuickStopActive:
            if (targetState == State::SwitchOnDisabled) {
                return ControlWord::DisableVoltage();
            }
            break;
            
        case State::Fault:
            return ControlWord::FaultReset();
            
        default:
            break;
    }
    
    return m_controlWord;
}

bool StateMachine::quickStop() {
    if (!m_writeControl) {
        return false;
    }
    
    m_controlWord = (m_controlWord & ~ControlWord::TransitionMask()) | 
                    ControlWord::QuickStop();
    m_writeControl(m_controlWord);
    
    TETHER_LOGI(TAG, "Quick stop initiated");
    return true;
}

bool StateMachine::resetFault() {
    if (!m_writeControl) {
        return false;
    }
    
    update();
    
    if (m_currentState != State::Fault) {
        TETHER_LOGD(TAG, "Not in fault state");
        return false;
    }
    
    // Rising edge on fault reset bit
    m_controlWord &= ~static_cast<uint16_t>(ControlWordBit::FaultReset);
    m_writeControl(m_controlWord);
    Tether::Platform::Clock::instance().delayMilliseconds(1);
    
    m_controlWord |= static_cast<uint16_t>(ControlWordBit::FaultReset);
    m_writeControl(m_controlWord);
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    
    m_controlWord &= ~static_cast<uint16_t>(ControlWordBit::FaultReset);
    m_writeControl(m_controlWord);
    
    TETHER_LOGI(TAG, "Fault reset sent");
    return true;
}

void StateMachine::setHalt(bool halt) {
    m_haltActive = halt;
    
    if (m_writeControl) {
        if (halt) {
            m_controlWord |= static_cast<uint16_t>(ControlWordBit::Halt);
        } else {
            m_controlWord &= ~static_cast<uint16_t>(ControlWordBit::Halt);
        }
        m_writeControl(m_controlWord);
    }
}

bool StateMachine::setOperatingMode(OperatingMode mode) {
    // Mode changes should typically happen before enabling
    // The actual SDO write must be done by the backend
    m_operatingMode = mode;
    
    if (m_modeCallback) {
        m_modeCallback(mode);
    }
    
    TETHER_LOGI(TAG, "Operating mode set to %s (%d)",
             CiA402::getOperatingModeName(static_cast<int8_t>(mode)),
             static_cast<int>(mode));
    return true;
}

bool StateMachine::isFaulted() const {
    return m_currentState == State::Fault || 
           m_currentState == State::FaultReactionActive;
}

bool StateMachine::isEnabled() const {
    return m_currentState == State::OperationEnabled;
}

bool StateMachine::isTargetReached() const {
    return StatusWord::isTargetReached(m_statusWord);
}

bool StateMachine::isInMotion() const {
    return isEnabled() && !isTargetReached();
}

bool StateMachine::hasWarning() const {
    return StatusWord::hasWarning(m_statusWord);
}

bool StateMachine::isLimitActive() const {
    return StatusWord::isLimitActive(m_statusWord);
}

bool StateMachine::isHomingAttained() const {
    return StatusWord::isHomingAttained(m_statusWord);
}

bool StateMachine::hasHomingError() const {
    return StatusWord::hasHomingError(m_statusWord);
}

bool StateMachine::startHoming() {
    if (!m_writeControl || m_operatingMode != OperatingMode::Homing) {
        return false;
    }
    
    // Set Bit 4 (Homing operation start) with rising edge
    m_controlWord &= ~static_cast<uint16_t>(ControlWordBit::HomingStart);
    m_writeControl(m_controlWord);
    Tether::Platform::Clock::instance().delayMilliseconds(1);
    
    m_controlWord |= static_cast<uint16_t>(ControlWordBit::HomingStart);
    m_writeControl(m_controlWord);
    
    TETHER_LOGI(TAG, "Homing started");
    return true;
}

} // namespace CiA402
