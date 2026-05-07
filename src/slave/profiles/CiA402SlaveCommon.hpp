/**
 * @file CiA402SlaveCommon.hpp
 * @brief Common definitions for CiA 402 Slave implementation files
 *
 * Internal header shared between CiA402Slave*.cpp files.
 * Contains bit definitions, indices, and mode values used across the implementation.
 */

#pragma once

#include "slave/profiles/CiA402Slave.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 402 Control Word Bit Definitions
// ============================================================================

namespace ControlWordBits {
    constexpr uint16_t SwitchOn         = (1 << 0);   // Bit 0: Switch On
    constexpr uint16_t EnableVoltage    = (1 << 1);   // Bit 1: Enable Voltage
    constexpr uint16_t QuickStop        = (1 << 2);   // Bit 2: Quick Stop (active low)
    constexpr uint16_t EnableOperation  = (1 << 3);   // Bit 3: Enable Operation
    constexpr uint16_t NewSetPoint      = (1 << 4);   // Bit 4: New set-point (PP mode)
    constexpr uint16_t ChangeSetImmediately = (1 << 5);  // Bit 5: Change set immediately (PP)
    constexpr uint16_t AbsRel           = (1 << 6);   // Bit 6: Absolute/Relative (PP mode)
    constexpr uint16_t FaultReset       = (1 << 7);   // Bit 7: Fault Reset
    constexpr uint16_t Halt             = (1 << 8);   // Bit 8: Halt
    constexpr uint16_t HomingStart      = (1 << 4);   // Bit 4: Homing operation start (HM mode)
    // Mode-specific bits: 4-6, 8 are mode-specific
}

// ============================================================================
// CiA 402 Status Word Bit Definitions
// ============================================================================

namespace StatusWordBits {
    constexpr uint16_t ReadyToSwitchOn  = (1 << 0);   // Bit 0: Ready to switch on
    constexpr uint16_t SwitchedOn       = (1 << 1);   // Bit 1: Switched on
    constexpr uint16_t OperationEnabled = (1 << 2);   // Bit 2: Operation enabled
    constexpr uint16_t Fault            = (1 << 3);   // Bit 3: Fault
    constexpr uint16_t VoltageEnabled   = (1 << 4);   // Bit 4: Voltage enabled
    constexpr uint16_t QuickStop        = (1 << 5);   // Bit 5: Quick stop (active low)
    constexpr uint16_t SwitchOnDisabled = (1 << 6);   // Bit 6: Switch on disabled
    constexpr uint16_t Warning          = (1 << 7);   // Bit 7: Warning
    constexpr uint16_t Remote           = (1 << 9);   // Bit 9: Remote (always 1 for EtherCAT)
    constexpr uint16_t TargetReached    = (1 << 10);  // Bit 10: Target reached
    constexpr uint16_t InternalLimitActive = (1 << 11);  // Bit 11: Internal limit active
    constexpr uint16_t SetPointAck      = (1 << 12);  // Bit 12: Set-point acknowledge (PP)
    constexpr uint16_t FollowingError   = (1 << 13);  // Bit 13: Following error (PP, CSP)
    // Mode-specific bits: 12, 13 are mode-specific
    constexpr uint16_t HomingAttained   = (1 << 12);  // Bit 12: Homing attained (HM mode)
    constexpr uint16_t HomingError      = (1 << 13);  // Bit 13: Homing error (HM mode)
}

// ============================================================================
// CiA 402 Object Dictionary Indices
// ============================================================================

namespace CiA402Index {
    // Control/Status
    constexpr uint16_t ControlWord          = 0x6040;
    constexpr uint16_t StatusWord           = 0x6041;
    
    // Error
    constexpr uint16_t ErrorCode            = 0x603F;
    
    // Operating Mode
    constexpr uint16_t ModesOfOperation     = 0x6060;
    constexpr uint16_t ModesOfOperationDisplay = 0x6061;
    
    // Position
    constexpr uint16_t PositionDemand       = 0x6062;
    constexpr uint16_t PositionActual       = 0x6064;
    constexpr uint16_t FollowingErrorWindow = 0x6065;
    constexpr uint16_t FollowingErrorTimeout = 0x6066;
    constexpr uint16_t PositionWindow       = 0x6067;
    constexpr uint16_t PositionWindowTime   = 0x6068;
    constexpr uint16_t TargetPosition       = 0x607A;
    constexpr uint16_t HomeOffset           = 0x607C;
    constexpr uint16_t SoftwarePosLimit     = 0x607D;
    constexpr uint16_t MaxProfileVelocity   = 0x607F;
    constexpr uint16_t MaxMotorVelocity     = 0x6080;
    constexpr uint16_t ProfileVelocity      = 0x6081;
    constexpr uint16_t EndVelocity          = 0x6082;
    constexpr uint16_t ProfileAcceleration  = 0x6083;
    constexpr uint16_t ProfileDeceleration  = 0x6084;
    constexpr uint16_t QuickStopDeceleration = 0x6085;
    constexpr uint16_t MotionProfileType    = 0x6086;
    
    // Velocity
    constexpr uint16_t VelocityDemand       = 0x606B;
    constexpr uint16_t VelocityActual       = 0x606C;
    constexpr uint16_t VelocityThreshold    = 0x606F;
    constexpr uint16_t VelocityThresholdTime = 0x6070;
    constexpr uint16_t TargetVelocity       = 0x60FF;
    
    // Torque
    constexpr uint16_t TargetTorque         = 0x6071;
    constexpr uint16_t MaxTorque            = 0x6072;
    constexpr uint16_t TorqueDemand         = 0x6074;
    constexpr uint16_t MotorRatedCurrent    = 0x6075;
    constexpr uint16_t MotorRatedTorque     = 0x6076;
    constexpr uint16_t TorqueActual         = 0x6077;
    
    // Homing
    constexpr uint16_t HomingMethod         = 0x6098;
    constexpr uint16_t HomingSpeeds         = 0x6099;
    constexpr uint16_t HomingAcceleration   = 0x609A;
    
    // Position factor
    constexpr uint16_t PositionEncoderResolution = 0x608F;
    constexpr uint16_t VelocityEncoderResolution = 0x6090;
    constexpr uint16_t GearRatio            = 0x6091;
    constexpr uint16_t FeedConstant         = 0x6092;
    constexpr uint16_t PositionFactor       = 0x6093;
    constexpr uint16_t VelocityEncoderFactor = 0x6094;
    constexpr uint16_t VelocityFactor1      = 0x6095;
    constexpr uint16_t VelocityFactor2      = 0x6096;
    constexpr uint16_t AccelerationFactor   = 0x6097;
    
    // CSP/CSV/CST offsets
    constexpr uint16_t PositionOffset       = 0x60B0;
    constexpr uint16_t VelocityOffset       = 0x60B1;
    constexpr uint16_t TorqueOffset         = 0x60B2;
    
    // Touch probe
    constexpr uint16_t TouchProbeFunction   = 0x60B8;
    constexpr uint16_t TouchProbeStatus     = 0x60B9;
    constexpr uint16_t TouchProbe1PosValue  = 0x60BA;
    constexpr uint16_t TouchProbe1NegValue  = 0x60BB;
    constexpr uint16_t TouchProbe2PosValue  = 0x60BC;
    constexpr uint16_t TouchProbe2NegValue  = 0x60BD;
    
    // Interpolation
    constexpr uint16_t InterpolationTimePeriod = 0x60C2;
    
    // Max acceleration
    constexpr uint16_t MaxAcceleration      = 0x60C5;
    constexpr uint16_t MaxDeceleration      = 0x60C6;
    
    // Following error actual
    constexpr uint16_t FollowingErrorActual = 0x60F4;
    
    // Digital I/O
    constexpr uint16_t DigitalInputs        = 0x60FD;
    constexpr uint16_t DigitalOutputs       = 0x60FE;
    
    // Supported drive modes
    constexpr uint16_t SupportedDriveModes  = 0x6502;
}

// ============================================================================
// CiA 402 Operating Mode Values
// ============================================================================

namespace CiA402ModeValue {
    constexpr int8_t NoMode              = 0;
    constexpr int8_t ProfilePosition     = 1;   // PP
    constexpr int8_t VelocityMode        = 2;   // VL
    constexpr int8_t ProfileVelocity     = 3;   // PV
    constexpr int8_t ProfileTorque       = 4;   // PT
    constexpr int8_t HomingMode          = 6;   // HM
    constexpr int8_t InterpolatedPosition = 7;  // IP
    constexpr int8_t CyclicSyncPosition  = 8;   // CSP
    constexpr int8_t CyclicSyncVelocity  = 9;   // CSV
    constexpr int8_t CyclicSyncTorque    = 10;  // CST
}

}  // namespace slave
}  // namespace EtherCAT
