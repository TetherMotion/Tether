/**
 * @file CiA402SlaveState.cpp
 * @brief CiA 402 Slave - State Machine Transitions
 */

#include "CiA402SlaveCommon.hpp"

namespace EtherCAT {
namespace slave {

// ============================================================================
// Drive State Machine
// ============================================================================

uint16_t CiA402Slave::getStatusWord() const {
    return statusWord_;
}

void CiA402Slave::processControlWord(uint16_t controlWord) {
    uint16_t oldControlWord = controlWord_;
    controlWord_ = controlWord;
    
    // Detect fault reset rising edge
    bool faultResetEdge = ((controlWord & ControlWordBits::FaultReset) != 0) &&
                          ((oldControlWord & ControlWordBits::FaultReset) == 0);
    
    // Update drive state based on control word
    updateDriveState(controlWord);
    
    // Handle fault reset
    if (faultResetEdge && driveState_ == CiA402State::Fault) {
        clearFault();
    }
}

void CiA402Slave::updateDriveState(uint16_t controlWord) {
    // Extract control word bits
    bool switchOn = (controlWord & ControlWordBits::SwitchOn) != 0;
    bool enableVoltage = (controlWord & ControlWordBits::EnableVoltage) != 0;
    bool quickStop = (controlWord & ControlWordBits::QuickStop) != 0;  // Active LOW
    bool enableOperation = (controlWord & ControlWordBits::EnableOperation) != 0;
    bool faultReset = (controlWord & ControlWordBits::FaultReset) != 0;
    
    // DS402 State Machine Transitions
    // See IEC 61800-7-201 / CiA 402
    
    CiA402State newState = driveState_;
    
    switch (driveState_) {
        case CiA402State::NotReadyToSwitchOn:
            // Automatic transition to SwitchOnDisabled after initialization
            // (handled in initObjectDictionary)
            break;
            
        case CiA402State::SwitchOnDisabled:
            // Transition 2: SwitchOnDisabled -> ReadyToSwitchOn
            // Command: Shutdown (0xxx 0110)
            if (!switchOn && enableVoltage && quickStop) {
                newState = CiA402State::ReadyToSwitchOn;
            }
            break;
            
        case CiA402State::ReadyToSwitchOn:
            // Transition 3: ReadyToSwitchOn -> SwitchedOn
            // Command: Switch On (0xxx 0111)
            if (switchOn && enableVoltage && quickStop && !enableOperation) {
                newState = CiA402State::SwitchedOn;
            }
            // Transition 7: ReadyToSwitchOn -> SwitchOnDisabled
            // Command: Disable Voltage (0xxx xx0x) or Quick Stop (0xxx x01x)
            else if (!enableVoltage || !quickStop) {
                newState = CiA402State::SwitchOnDisabled;
            }
            break;
            
        case CiA402State::SwitchedOn:
            // Transition 4: SwitchedOn -> OperationEnabled
            // Command: Enable Operation (0xxx 1111)
            if (switchOn && enableVoltage && quickStop && enableOperation) {
                newState = CiA402State::OperationEnabled;
            }
            // Transition 6: SwitchedOn -> ReadyToSwitchOn
            // Command: Shutdown (0xxx 0110)
            else if (!switchOn && enableVoltage && quickStop) {
                newState = CiA402State::ReadyToSwitchOn;
            }
            // Transition 10: SwitchedOn -> SwitchOnDisabled
            // Command: Disable Voltage (0xxx xx0x)
            else if (!enableVoltage) {
                newState = CiA402State::SwitchOnDisabled;
            }
            break;
            
        case CiA402State::OperationEnabled:
            // Transition 5: OperationEnabled -> SwitchedOn
            // Command: Disable Operation (0xxx 0111)
            if (switchOn && enableVoltage && quickStop && !enableOperation) {
                newState = CiA402State::SwitchedOn;
            }
            // Transition 8: OperationEnabled -> ReadyToSwitchOn
            // Command: Shutdown (0xxx 0110)
            else if (!switchOn && enableVoltage && quickStop) {
                newState = CiA402State::ReadyToSwitchOn;
            }
            // Transition 9: OperationEnabled -> SwitchOnDisabled
            // Command: Disable Voltage (0xxx xx0x)
            else if (!enableVoltage) {
                newState = CiA402State::SwitchOnDisabled;
            }
            // Transition 11: OperationEnabled -> QuickStopActive
            // Command: Quick Stop (0xxx x01x)
            else if (!quickStop) {
                newState = CiA402State::QuickStopActive;
            }
            break;
            
        case CiA402State::QuickStopActive:
            // Transition 12: QuickStopActive -> SwitchOnDisabled
            // Command: Disable Voltage (0xxx xx0x) or after quick stop completed
            if (!enableVoltage) {
                newState = CiA402State::SwitchOnDisabled;
            }
            // Transition 16: QuickStopActive -> OperationEnabled
            // Command: Enable Operation (0xxx 1111) - if quick stop option allows
            else if (switchOn && enableVoltage && quickStop && enableOperation) {
                // Only if quick stop is complete and option allows restart
                if (std::abs(actualVelocity_) < static_cast<int32_t>(driveConfig_.velocityThreshold)) {
                    newState = CiA402State::OperationEnabled;
                }
            }
            break;
            
        case CiA402State::FaultReactionActive:
            // Automatic transition to Fault after reaction
            newState = CiA402State::Fault;
            break;
            
        case CiA402State::Fault:
            // Transition 15: Fault -> SwitchOnDisabled
            // Command: Fault Reset (rising edge of bit 7)
            // Handled in processControlWord
            break;
    }
    
    // Apply state change if different
    if (newState != driveState_) {
        setDriveState(newState);
    }
}

void CiA402Slave::setDriveState(CiA402State newState) {
    if (newState == driveState_) {
        return;
    }
    
    CiA402State oldState = driveState_;
    driveState_ = newState;
    
    // Actions on state entry
    switch (newState) {
        case CiA402State::SwitchOnDisabled:
            // Power stage disabled
            torqueDemand_ = 0;
            break;
            
        case CiA402State::ReadyToSwitchOn:
            // Ready for power stage
            break;
            
        case CiA402State::SwitchedOn:
            // Power stage enabled, no motion
            torqueDemand_ = 0;
            break;
            
        case CiA402State::OperationEnabled:
            // Full operation - update mode display
            operatingModeDisplay_ = operatingMode_;
            // Initialize demands from actual
            positionDemand_ = actualPosition_;
            velocityDemand_ = 0;
            break;
            
        case CiA402State::QuickStopActive:
            // Quick stop deceleration - zero torque demand to let friction stop the motor
            torqueDemand_ = 0;
            break;
            
        case CiA402State::FaultReactionActive:
            // Emergency stop
            torqueDemand_ = 0;
            break;
            
        case CiA402State::Fault:
            // Fault state
            torqueDemand_ = 0;
            break;
            
        default:
            break;
    }
    
    // Invoke callback
    if (driveStateCallback_) {
        driveStateCallback_(oldState, newState);
    }
}

uint16_t CiA402Slave::computeStatusWord() const {
    uint16_t sw = 0;
    
    // State-dependent bits (bits 0-3, 5-6)
    switch (driveState_) {
        case CiA402State::NotReadyToSwitchOn:
            // xxxx xxxx x0xx 0000
            break;
            
        case CiA402State::SwitchOnDisabled:
            // xxxx xxxx x1xx 0000
            sw |= StatusWordBits::SwitchOnDisabled;
            break;
            
        case CiA402State::ReadyToSwitchOn:
            // xxxx xxxx x01x 0001
            sw |= StatusWordBits::ReadyToSwitchOn;
            sw |= StatusWordBits::QuickStop;
            break;
            
        case CiA402State::SwitchedOn:
            // xxxx xxxx x01x 0011
            sw |= StatusWordBits::ReadyToSwitchOn;
            sw |= StatusWordBits::SwitchedOn;
            sw |= StatusWordBits::QuickStop;
            break;
            
        case CiA402State::OperationEnabled:
            // xxxx xxxx x01x 0111
            sw |= StatusWordBits::ReadyToSwitchOn;
            sw |= StatusWordBits::SwitchedOn;
            sw |= StatusWordBits::OperationEnabled;
            sw |= StatusWordBits::QuickStop;
            break;
            
        case CiA402State::QuickStopActive:
            // xxxx xxxx x00x 0111
            sw |= StatusWordBits::ReadyToSwitchOn;
            sw |= StatusWordBits::SwitchedOn;
            sw |= StatusWordBits::OperationEnabled;
            break;
            
        case CiA402State::FaultReactionActive:
            // xxxx xxxx x0xx 1111
            sw |= StatusWordBits::ReadyToSwitchOn;
            sw |= StatusWordBits::SwitchedOn;
            sw |= StatusWordBits::OperationEnabled;
            sw |= StatusWordBits::Fault;
            break;
            
        case CiA402State::Fault:
            // xxxx xxxx x0xx 1000
            sw |= StatusWordBits::Fault;
            break;
    }
    
    // Bit 4: Voltage enabled (always true when not in fault/init)
    if (driveState_ != CiA402State::NotReadyToSwitchOn &&
        driveState_ != CiA402State::SwitchOnDisabled &&
        driveState_ != CiA402State::Fault) {
        sw |= StatusWordBits::VoltageEnabled;
    }
    
    // Bit 7: Warning
    // Set if following error is large but not critical
    uint32_t feWindow = driveConfig_.followingErrorWindow;
    if (feWindow > 0 && std::abs(getFollowingError()) > static_cast<int32_t>(feWindow / 2)) {
        sw |= StatusWordBits::Warning;
    }
    
    // Bit 9: Remote (always 1 for EtherCAT - controlled remotely)
    sw |= StatusWordBits::Remote;
    
    // Bit 10: Target reached
    if (targetReached_) {
        sw |= StatusWordBits::TargetReached;
    }
    
    // Bit 11: Internal limit active
    if (actualPosition_ <= driveConfig_.softwarePosLimitMin ||
        actualPosition_ >= driveConfig_.softwarePosLimitMax) {
        sw |= StatusWordBits::InternalLimitActive;
    }
    
    // Mode-specific bits (12-13)
    switch (operatingModeDisplay_) {
        case CiA402ModeValue::ProfilePosition:
        case CiA402ModeValue::CyclicSyncPosition:
            // Bit 12: Set-point acknowledge
            if (setPointAcknowledge_) {
                sw |= StatusWordBits::SetPointAck;
            }
            // Bit 13: Following error
            if (std::abs(getFollowingError()) > static_cast<int32_t>(driveConfig_.followingErrorWindow)) {
                sw |= StatusWordBits::FollowingError;
            }
            break;
            
        case CiA402ModeValue::HomingMode:
            // Bit 12: Homing attained
            if (homingComplete_) {
                sw |= StatusWordBits::HomingAttained;
            }
            // Bit 13: Homing error - not implemented in simulation
            break;
            
        default:
            break;
    }
    
    return sw;
}

void CiA402Slave::triggerFault(uint16_t errorCode) {
    errorCode_ = errorCode;
    
    // Transition to fault reaction, then fault
    if (driveState_ != CiA402State::Fault && 
        driveState_ != CiA402State::FaultReactionActive) {
        setDriveState(CiA402State::FaultReactionActive);
        // In a real implementation, this would trigger emergency stop
        // For simulation, transition directly to Fault
        setDriveState(CiA402State::Fault);
    }
}

void CiA402Slave::clearFault() {
    if (driveState_ == CiA402State::Fault) {
        errorCode_ = 0;
        setDriveState(CiA402State::SwitchOnDisabled);
    }
}

void CiA402Slave::setDriveStateCallback(DriveStateCallback callback) {
    driveStateCallback_ = std::move(callback);
}

}  // namespace slave
}  // namespace EtherCAT
