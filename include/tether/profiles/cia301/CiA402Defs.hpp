/**
 * @file CiA402Defs.hpp
 * @brief CiA 402 Drive Profile Object Dictionary Definitions
 * 
 * @details
 * This file contains all CiA 402 (IEC 61800-7-201/204) device profile object
 * dictionary indexes and subindexes for servo drives and frequency converters.
 * 
 * ## Object Dictionary Areas for CiA 402
 * 
 * | Range         | Description                                    |
 * |---------------|------------------------------------------------|
 * | 0x6000-0x67FF | Drive-specific objects (common to all modes)   |
 * | 0x6000-0x603F | Common device control                          |
 * | 0x6040-0x607F | Device control                                 |
 * | 0x6080-0x60BF | Factor group                                   |
 * | 0x60C0-0x60FF | Position control function                      |
 * | 0x6100-0x613F | Velocity control function                      |
 * | 0x6140-0x617F | Torque control function                        |
 * | 0x6180-0x61BF | Position demand / position actual value        |
 * | 0x61C0-0x61FF | Velocity demand / velocity actual value        |
 * | 0x6200-0x623F | Torque demand / torque actual value            |
 * | 0x6300-0x63FF | Touch probe function                           |
 * | 0x6400-0x64FF | Homing                                         |
 * | 0x6500-0x65FF | Interpolated position mode                     |
 * | 0x6600-0x67FF | Profile position mode                          |
 * 
 * @see CiA 402 DS-402 Drive and motion control
 * @see IEC 61800-7-201/204
 */

#pragma once

#include <cstdint>

namespace CiA402 {

/**
 * @brief Convenience ControlWord values (CiA 402)
 */
enum class ControlWord : uint16_t {
    DISABLE_VOLTAGE = 0x0000,
    SHUTDOWN = 0x0006,
    SWITCH_ON = 0x0007,
    ENABLE_OPERATION = 0x000F,
    FAULT_RESET = 0x0080,
};

enum class Register : uint16_t {
    // ========================================================================
    // Device Control (0x6000 - 0x607F)
    // ========================================================================
    
    /**
     * @brief Abort connection option code (0x6007)
     */
    AbortConnectionOptionCode   = 0x6007,

    /**
     * @brief Error Code (0x603F)
     */
    ErrorCode                   = 0x603F,

    /**
     * @brief Controlword (0x6040)
     * 
     * Primary control register for the drive state machine.
     * See StatusWord bits for decoding state and transitions.
     */
    Controlword                 = 0x6040,

    /**
     * @brief Statusword (0x6041)
     * 
     * Primary status register showing current drive state and flags.
     */
    Statusword                  = 0x6041,

    /**
     * @brief Quick Stop Option Code (0x605A)
     */
    QuickStopOptionCode         = 0x605A,

    /**
     * @brief Shutdown Option Code (0x605B)
     */
    ShutdownOptionCode          = 0x605B,

    /**
     * @brief Disable Operation Option Code (0x605C)
     */
    DisableOperationOption      = 0x605C,

    /**
     * @brief Halt Option Code (0x605D)
     */
    HaltOptionCode              = 0x605D,

    /**
     * @brief Fault Reaction Option Code (0x605E)
     */
    FaultReactionOption         = 0x605E,

    /**
     * @brief Modes of Operation (0x6060)
     */
    ModesOfOperation            = 0x6060,

    /**
     * @brief Modes of Operation Display (0x6061)
     */
    ModesOfOperationDisplay     = 0x6061,

    // ========================================================================
    // Position Control (0x6062 - 0x60FF)
    // ========================================================================

    /**
     * @brief Position Demand Value (0x6062)
     */
    PositionDemandValue         = 0x6062,

    /**
     * @brief Position Actual Internal Value (0x6063)
     */
    PositionActualInternal      = 0x6063,

    /**
     * @brief Position Actual Value (0x6064)
     */
    PositionActualValue         = 0x6064,

    /**
     * @brief Following Error Window (0x6065)
     */
    FollowingErrorWindow        = 0x6065,

    /**
     * @brief Following Error Timeout (0x6066)
     */
    FollowingErrorTimeout       = 0x6066,

    /**
     * @brief Position Window (0x6067)
     */
    PositionWindow              = 0x6067,

    /**
     * @brief Position Window Time (0x6068)
     */
    PositionWindowTime          = 0x6068,

    /**
     * @brief Velocity Sensor Actual Value (0x6069)
     */
    VelocitySensorActual        = 0x6069,

    /**
     * @brief Sensor Selection Code (0x606A)
     */
    SensorSelectionCode         = 0x606A,

    /**
     * @brief Velocity Demand Value (0x606B)
     */
    VelocityDemandValue         = 0x606B,

    /**
     * @brief Velocity Actual Value (0x606C)
     */
    VelocityActualValue         = 0x606C,

    /**
     * @brief Velocity Window (0x606D)
     */
    VelocityWindow              = 0x606D,

    /**
     * @brief Velocity Window Time (0x606E)
     */
    VelocityWindowTime          = 0x606E,

    /**
     * @brief Velocity Threshold (0x606F)
     */
    VelocityThreshold           = 0x606F,

    /**
     * @brief Velocity Threshold Time (0x6070)
     */
    VelocityThresholdTime       = 0x6070,

    /**
     * @brief Target Torque (0x6071)
     */
    TargetTorque                = 0x6071,

    /**
     * @brief Max Torque (0x6072)
     */
    MaxTorque                   = 0x6072,

    /**
     * @brief Max Current (0x6073)
     */
    MaxCurrent                  = 0x6073,

    /**
     * @brief Torque Demand Value (0x6074)
     */
    TorqueDemandValue           = 0x6074,

    /**
     * @brief Motor Rated Current (0x6075)
     */
    MotorRatedCurrent           = 0x6075,

    /**
     * @brief Motor Rated Torque (0x6076)
     */
    MotorRatedTorque            = 0x6076,

    /**
     * @brief Torque Actual Value (0x6077)
     */
    TorqueActualValue           = 0x6077,

    /**
     * @brief Current Actual Value (0x6078)
     */
    CurrentActualValue          = 0x6078,

    /**
     * @brief DC Link Circuit Voltage (0x6079)
     */
    DCLinkVoltage               = 0x6079,

    /**
     * @brief Target Position (0x607A)
     */
    TargetPosition              = 0x607A,

    /**
     * @brief Position Range Limit (0x607B)
     */
    PositionRangeLimit          = 0x607B,

    /**
     * @brief Home Offset (0x607C)
     */
    HomeOffset                  = 0x607C,

    /**
     * @brief Software Position Limit (0x607D)
     */
    SoftwarePositionLimit       = 0x607D,

    /**
     * @brief Polarity (0x607E)
     */
    Polarity                    = 0x607E,

    /**
     * @brief Max Profile Velocity (0x607F)
     */
    MaxProfileVelocity          = 0x607F,

    // ========================================================================
    // Factor Group (0x6080 - 0x60BF)
    // ========================================================================

    /**
     * @brief Max Motor Speed (0x6080)
     */
    MaxMotorSpeed               = 0x6080,

    /**
     * @brief Profile Velocity (0x6081)
     */
    ProfileVelocity             = 0x6081,

    /**
     * @brief End Velocity (0x6082)
     */
    EndVelocity                 = 0x6082,

    /**
     * @brief Profile Acceleration (0x6083)
     */
    ProfileAcceleration         = 0x6083,

    /**
     * @brief Profile Deceleration (0x6084)
     */
    ProfileDeceleration         = 0x6084,

    /**
     * @brief Quick Stop Deceleration (0x6085)
     */
    QuickStopDeceleration       = 0x6085,

    /**
     * @brief Motion Profile Type (0x6086)
     */
    MotionProfileType           = 0x6086,

    /**
     * @brief Torque Slope (0x6087)
     */
    TorqueSlope                 = 0x6087,

    /**
     * @brief Torque Profile Type (0x6088)
     */
    TorqueProfileType           = 0x6088,

    /**
     * @brief Position Encoder Resolution (0x608F)
     */
    PositionEncoderResolution   = 0x608F,

    /**
     * @brief Velocity Encoder Resolution (0x6090)
     */
    VelocityEncoderResolution   = 0x6090,

    /**
     * @brief Gear Ratio (0x6091)
     */
    GearRatio                   = 0x6091,

    /**
     * @brief Feed Constant (0x6092)
     */
    FeedConstant                = 0x6092,

    /**
     * @brief Position Factor (0x6093)
     */
    PositionFactor              = 0x6093,

    /**
     * @brief Velocity Encoder Factor (0x6094)
     */
    VelocityEncoderFactor       = 0x6094,

    /**
     * @brief Velocity Factor 1 (0x6095)
     */
    VelocityFactor1             = 0x6095,

    /**
     * @brief Velocity Factor 2 (0x6096)
     */
    VelocityFactor2             = 0x6096,

    /**
     * @brief Acceleration Factor (0x6097)
     */
    AccelerationFactor          = 0x6097,

    // ========================================================================
    // Homing (0x6098 - 0x609F)
    // ========================================================================

    /**
     * @brief Homing Method (0x6098)
     */
    HomingMethod                = 0x6098,

    /**
     * @brief Homing Speeds (0x6099)
     */
    HomingSpeeds                = 0x6099,

    /**
     * @brief Homing Acceleration (0x609A)
     */
    HomingAcceleration          = 0x609A,

    // ========================================================================
    // Touch Probe (0x60B8 - 0x60BB)
    // ========================================================================

    /**
     * @brief Touch Probe Function (0x60B8)
     */
    TouchProbeFunction          = 0x60B8,

    /**
     * @brief Touch Probe Status (0x60B9)
     */
    TouchProbeStatus            = 0x60B9,

    /**
     * @brief Touch Probe 1 Positive Edge (0x60BA)
     */
    TouchProbe1PosEdge          = 0x60BA,

    /**
     * @brief Touch Probe 1 Negative Edge (0x60BB)
     */
    TouchProbe1NegEdge          = 0x60BB,

    /**
     * @brief Touch Probe 2 Positive Edge (0x60BC)
     */
    TouchProbe2PosEdge          = 0x60BC,

    /**
     * @brief Touch Probe 2 Negative Edge (0x60BD)
     */
    TouchProbe2NegEdge          = 0x60BD,

    // ========================================================================
    // Interpolated Position Mode (0x60C0 - 0x60C4)
    // ========================================================================

    /**
     * @brief Interpolation Sub Mode Select (0x60C0)
     */
    InterpolationSubMode        = 0x60C0,

    /**
     * @brief Interpolation Data Record (0x60C1)
     */
    InterpolationDataRecord     = 0x60C1,

    /**
     * @brief Interpolation Time Period (0x60C2)
     */
    InterpolationTimePeriod     = 0x60C2,

    /**
     * @brief Interpolation Sync Definition (0x60C3)
     */
    InterpolationSyncDef        = 0x60C3,

    /**
     * @brief Interpolation Data Configuration (0x60C4)
     */
    InterpolationDataConfig     = 0x60C4,

    /**
     * @brief Max Acceleration (0x60C5)
     */
    MaxAcceleration             = 0x60C5,

    /**
     * @brief Max Deceleration (0x60C6)
     */
    MaxDeceleration             = 0x60C6,

    // ========================================================================
    // Cyclic Synchronous Mode (0x60F4 - 0x60FF)
    // ========================================================================

    /**
     * @brief Positioning Option Code (0x60F2)
     */
    PositioningOptionCode       = 0x60F2,

    /**
     * @brief Following Error Actual Value (0x60F4)
     */
    FollowingErrorActual        = 0x60F4,

    /**
     * @brief Control Effort (0x60F5)
     */
    ControlEffort               = 0x60F5,

    /**
     * @brief Current Control Parameter Set (0x60F6)
     */
    CurrentControlParameterSet  = 0x60F6,

    /**
     * @brief Velocity Control Parameter Set (0x60F9)
     */
    VelocityControlParameterSet = 0x60F9,

    /**
     * @brief Position Control Parameter Set (0x60FB)
     */
    PositionControlParameterSet = 0x60FB,

    /**
     * @brief Position Demand Internal Value (0x60FC)
     */
    PositionDemandInternal      = 0x60FC,

    /**
     * @brief Digital Inputs (0x60FD)
     */
    DigitalInputs               = 0x60FD,

    /**
     * @brief Digital Outputs (0x60FE)
     */
    DigitalOutputs              = 0x60FE,

    /**
     * @brief Target Velocity (0x60FF)
     */
    TargetVelocity              = 0x60FF,

    // ========================================================================
    // Motor Data (0x6400 - 0x643F)
    // ========================================================================

    /**
     * @brief Motor Type (0x6402)
     */
    MotorType                   = 0x6402,

    /**
     * @brief Motor Catalog Number (0x6403)
     */
    MotorCatalogNumber          = 0x6403,

    /**
     * @brief Motor Manufacturer (0x6404)
     */
    MotorManufacturer           = 0x6404,

    /**
     * @brief Motor Data (0x6410)
     */
    MotorData                   = 0x6410,

    // ========================================================================
    // Drive Data (0x6500 - 0x653F)
    // ========================================================================

    /**
     * @brief Supported Drive Modes (0x6502)
     */
    SupportedDriveModes         = 0x6502,

    /**
     * @brief Drive Catalog Number (0x6503)
     */
    DriveCatalogNumber          = 0x6503,

    /**
     * @brief Drive Manufacturer (0x6504)
     */
    DriveManufacturer           = 0x6504,

    /**
     * @brief Drive Serial Number (0x6505)
     */
    DriveSerialNumber           = 0x6505,
};

// ============================================================================
// Operating Mode Values
// ============================================================================

namespace OperatingMode {
    constexpr int8_t ProfilePosition        = 1;   // PP
    constexpr int8_t ProfileVelocity        = 3;   // PV
    constexpr int8_t ProfileTorque          = 4;   // TQ
    constexpr int8_t Homing                 = 6;   // HM
    constexpr int8_t InterpolatedPosition   = 7;   // IP
    constexpr int8_t CyclicSyncPosition     = 8;   // CSP
    constexpr int8_t CyclicSyncVelocity     = 9;   // CSV
    constexpr int8_t CyclicSyncTorque       = 10;  // CST
}

// ============================================================================
// Subindexes
// ============================================================================

namespace PositionRangeLimitSub {
    constexpr uint8_t MinPositionRangeLimit = 0x01;
    constexpr uint8_t MaxPositionRangeLimit = 0x02;
}

namespace SoftwarePositionLimitSub {
    constexpr uint8_t MinPositionLimit      = 0x01;
    constexpr uint8_t MaxPositionLimit      = 0x02;
}

namespace HomingSpeedsSub {
    constexpr uint8_t SpeedDuringSearchForSwitch = 0x01;
    constexpr uint8_t SpeedDuringSearchForZero   = 0x02;
}

namespace InterpolationTimePeriodSub {
    constexpr uint8_t InterpolationTimeValue = 0x01;
    constexpr uint8_t InterpolationTimeIndex = 0x02;
}

namespace DigitalOutputsSub {
    constexpr uint8_t PhysicalOutputs       = 0x01;
    constexpr uint8_t BitMask               = 0x02;
}

// ============================================================================
// Statusword Bit Definitions
// ============================================================================

namespace StatuswordBits {
    constexpr uint16_t ReadyToSwitchOn      = (1 << 0);
    constexpr uint16_t SwitchedOn           = (1 << 1);
    constexpr uint16_t OperationEnabled     = (1 << 2);
    constexpr uint16_t Fault                = (1 << 3);
    constexpr uint16_t VoltageEnabled       = (1 << 4);
    constexpr uint16_t QuickStop            = (1 << 5);
    constexpr uint16_t SwitchOnDisabled     = (1 << 6);
    constexpr uint16_t Warning              = (1 << 7);
    constexpr uint16_t Remote               = (1 << 9);
    constexpr uint16_t TargetReached        = (1 << 10);
    constexpr uint16_t InternalLimitActive  = (1 << 11);
    // Bits 12-13: Operation mode specific
    // For PP/IP mode:
    constexpr uint16_t SetPointAcknowledge  = (1 << 12);  // PP mode
    constexpr uint16_t FollowingError       = (1 << 13);  // PP mode
    // For HM mode:
    constexpr uint16_t HomingAttained       = (1 << 12);
    constexpr uint16_t HomingError          = (1 << 13);
}

// ============================================================================
// Controlword Bit Definitions
// ============================================================================

namespace ControlwordBits {
    constexpr uint16_t SwitchOn             = (1 << 0);
    constexpr uint16_t EnableVoltage        = (1 << 1);
    constexpr uint16_t QuickStop            = (1 << 2);
    constexpr uint16_t EnableOperation      = (1 << 3);
    // Bits 4-6: Operation mode specific
    // For PP mode:
    constexpr uint16_t NewSetPoint          = (1 << 4);   // PP mode
    constexpr uint16_t ChangeSetImmediately = (1 << 5);   // PP mode
    constexpr uint16_t AbsoluteRelative     = (1 << 6);   // PP mode
    // For HM mode:
    constexpr uint16_t HomingOperationStart = (1 << 4);   // HM mode
    // For CSV/CSP/CST mode: no additional bits
    constexpr uint16_t FaultReset           = (1 << 7);
    constexpr uint16_t Halt                 = (1 << 8);
}

// ============================================================================
// Homing Method Values
// ============================================================================

namespace HomingMethodValue {
    // No homing
    constexpr int8_t NoHoming               = 0;
    
    // Limit switch methods (1-14)
    constexpr int8_t NegLimitIndex          = 1;
    constexpr int8_t PosLimitIndex          = 2;
    constexpr int8_t PosHomeIndexPositive   = 3;
    constexpr int8_t PosHomeIndexNegative   = 4;
    constexpr int8_t NegHomeIndexNegative   = 5;
    constexpr int8_t NegHomeIndexPositive   = 6;
    constexpr int8_t PosHomePositive        = 7;
    constexpr int8_t PosHomeNegative        = 8;
    constexpr int8_t NegHomeNegative        = 9;
    constexpr int8_t NegHomePositive        = 10;
    constexpr int8_t PosHomeIndexPositiveNeg = 11;
    constexpr int8_t PosHomeIndexNegativePos = 12;
    constexpr int8_t NegHomeIndexNegativePos = 13;
    constexpr int8_t NegHomeIndexPositiveNeg = 14;
    
    // Index pulse methods (33-34)
    constexpr int8_t NegIndexPulse          = 33;
    constexpr int8_t PosIndexPulse          = 34;
    
    // Current position methods (35-37)
    constexpr int8_t CurrentPosition        = 35;
    constexpr int8_t CurrentPositionAlt     = 37;
}

namespace SupportedDriveModeBits {
    constexpr uint32_t ProfilePosition      = (1 << 0);
    constexpr uint32_t VelocityMode         = (1 << 1);
    constexpr uint32_t ProfileVelocity      = (1 << 2);
    constexpr uint32_t ProfileTorque        = (1 << 3);
    constexpr uint32_t HomingMode           = (1 << 5);
    constexpr uint32_t InterpolatedPosition = (1 << 6);
    constexpr uint32_t CyclicSyncPosition   = (1 << 7);
    constexpr uint32_t CyclicSyncVelocity   = (1 << 8);
    constexpr uint32_t CyclicSyncTorque     = (1 << 9);
}

} // namespace CiA402
