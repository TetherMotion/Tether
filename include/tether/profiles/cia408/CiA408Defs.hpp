/**
 * @file CiA408Defs.hpp
 * @brief CiA 408 Fluid Power Technology Profile - Object Dictionary
 *
 * Defines the complete object dictionary for CiA 408 profile which covers
 * proportional valves, servo valves, and variable displacement pumps.
 *
 * CiA 408 Profile Features:
 * - Proportional valve control (directional, pressure, flow)
 * - Servo valve control with position feedback
 * - Pressure/flow regulation
 * - Valve diagnostics and monitoring
 * - Multiple control modes (open loop, closed loop)
 */

#pragma once

#include <cstdint>

namespace CiA408 {

// ============================================================================
// Profile Identification
// ============================================================================

constexpr uint16_t PROFILE_NUMBER = 408;

// ============================================================================
// Object Dictionary Indices - Control/Status (0x6000-0x60FF)
// ============================================================================

// Device Control
constexpr uint16_t Controlword             = 0x6000;
constexpr uint16_t Statusword              = 0x6001;
constexpr uint16_t OperatingMode           = 0x6002;
constexpr uint16_t ControlModeDisplay      = 0x6003;

// Setpoint/Actual Values
constexpr uint16_t SetpointValue           = 0x6010;
constexpr uint16_t ActualValue             = 0x6011;
constexpr uint16_t SetpointRamp            = 0x6012;
constexpr uint16_t SetpointFilter          = 0x6013;

// Secondary Setpoints (for multi-channel devices)
constexpr uint16_t SetpointValue2          = 0x6014;
constexpr uint16_t ActualValue2            = 0x6015;
constexpr uint16_t SetpointValue3          = 0x6016;
constexpr uint16_t ActualValue3            = 0x6017;

// Position Control (for servo valves)
constexpr uint16_t PositionSetpoint        = 0x6020;
constexpr uint16_t PositionActual          = 0x6021;
constexpr uint16_t PositionWindow          = 0x6022;
constexpr uint16_t PositionWindowTime      = 0x6023;

// Velocity/Flow Control
constexpr uint16_t VelocitySetpoint        = 0x6030;
constexpr uint16_t VelocityActual          = 0x6031;
constexpr uint16_t VelocityWindow          = 0x6032;
constexpr uint16_t VelocityWindowTime      = 0x6033;

// Pressure Control
constexpr uint16_t PressureSetpoint        = 0x6040;
constexpr uint16_t PressureActualA         = 0x6041;
constexpr uint16_t PressureActualB         = 0x6042;
constexpr uint16_t PressureActualP         = 0x6043;  // Supply pressure
constexpr uint16_t PressureActualT         = 0x6044;  // Tank pressure
constexpr uint16_t PressureWindow          = 0x6045;
constexpr uint16_t MaxPressure             = 0x6046;
constexpr uint16_t MinPressure             = 0x6047;

// Force Control
constexpr uint16_t ForceSetpoint           = 0x6050;
constexpr uint16_t ForceActual             = 0x6051;
constexpr uint16_t ForceWindow             = 0x6052;
constexpr uint16_t MaxForce                = 0x6053;

// ============================================================================
// Object Dictionary Indices - Parameters (0x6100-0x61FF)
// ============================================================================

// Valve Parameters
constexpr uint16_t ValveType               = 0x6100;
constexpr uint16_t NominalFlow             = 0x6101;
constexpr uint16_t NominalPressure         = 0x6102;
constexpr uint16_t NominalStroke           = 0x6103;
constexpr uint16_t ResponseTime            = 0x6104;
constexpr uint16_t Hysteresis              = 0x6105;
constexpr uint16_t Repeatability           = 0x6106;

// Spool Parameters
constexpr uint16_t SpoolPosition           = 0x6110;
constexpr uint16_t SpoolOverlap            = 0x6111;
constexpr uint16_t Deadband                = 0x6112;
constexpr uint16_t NullOffset              = 0x6113;

// Flow Parameters
constexpr uint16_t FlowGain                = 0x6120;
constexpr uint16_t FlowCharacteristic      = 0x6121;
constexpr uint16_t FlowLinearization       = 0x6122;
constexpr uint16_t LeakageFlow             = 0x6123;

// Pressure Compensation
constexpr uint16_t PressureCompensation    = 0x6130;
constexpr uint16_t PressureCompGain        = 0x6131;
constexpr uint16_t LoadSensing             = 0x6132;

// ============================================================================
// Object Dictionary Indices - Controller (0x6200-0x62FF)
// ============================================================================

// Position Controller
constexpr uint16_t PosController_Kp        = 0x6200;
constexpr uint16_t PosController_Ki        = 0x6201;
constexpr uint16_t PosController_Kd        = 0x6202;
constexpr uint16_t PosController_Kv        = 0x6203;  // Velocity feedforward
constexpr uint16_t PosController_Ka        = 0x6204;  // Acceleration feedforward
constexpr uint16_t PosController_Limit     = 0x6205;

// Velocity Controller
constexpr uint16_t VelController_Kp        = 0x6210;
constexpr uint16_t VelController_Ki        = 0x6211;
constexpr uint16_t VelController_Limit     = 0x6212;

// Pressure Controller
constexpr uint16_t PrsController_Kp        = 0x6220;
constexpr uint16_t PrsController_Ki        = 0x6221;
constexpr uint16_t PrsController_Limit     = 0x6222;

// Force Controller
constexpr uint16_t FrcController_Kp        = 0x6230;
constexpr uint16_t FrcController_Ki        = 0x6231;
constexpr uint16_t FrcController_Limit     = 0x6232;

// Dither Parameters
constexpr uint16_t DitherAmplitude         = 0x6240;
constexpr uint16_t DitherFrequency         = 0x6241;
constexpr uint16_t DitherEnable            = 0x6242;

// ============================================================================
// Object Dictionary Indices - Limits (0x6300-0x63FF)
// ============================================================================

// Position Limits
constexpr uint16_t MaxPositionLimit        = 0x6300;
constexpr uint16_t MinPositionLimit        = 0x6301;
constexpr uint16_t PositionLimitEnable     = 0x6302;

// Velocity Limits
constexpr uint16_t MaxVelocityLimit        = 0x6310;
constexpr uint16_t MinVelocityLimit        = 0x6311;
constexpr uint16_t AccelerationLimit       = 0x6312;
constexpr uint16_t DecelerationLimit       = 0x6313;

// Pressure Limits
constexpr uint16_t MaxPressureLimitA       = 0x6320;
constexpr uint16_t MaxPressureLimitB       = 0x6321;
constexpr uint16_t MinPressureLimitA       = 0x6322;
constexpr uint16_t MinPressureLimitB       = 0x6323;
constexpr uint16_t ReliefPressure          = 0x6324;

// Current Limits (for solenoid control)
constexpr uint16_t MaxCurrentA             = 0x6330;
constexpr uint16_t MaxCurrentB             = 0x6331;
constexpr uint16_t DitheringCurrent        = 0x6332;

// ============================================================================
// Object Dictionary Indices - Diagnostics (0x6400-0x64FF)
// ============================================================================

// Valve Status
constexpr uint16_t ValveStatus             = 0x6400;
constexpr uint16_t ValveFault              = 0x6401;
constexpr uint16_t ValveWarning            = 0x6402;
constexpr uint16_t OperatingHours          = 0x6403;
constexpr uint16_t CycleCount              = 0x6404;

// Electrical Diagnostics
constexpr uint16_t CoilCurrentA            = 0x6410;
constexpr uint16_t CoilCurrentB            = 0x6411;
constexpr uint16_t CoilResistanceA         = 0x6412;
constexpr uint16_t CoilResistanceB         = 0x6413;
constexpr uint16_t CoilTemperature         = 0x6414;
constexpr uint16_t SupplyVoltage           = 0x6415;

// Sensor Diagnostics
constexpr uint16_t LVDTSignal              = 0x6420;
constexpr uint16_t LVDTStatus              = 0x6421;
constexpr uint16_t PressureSensorStatusA   = 0x6422;
constexpr uint16_t PressureSensorStatusB   = 0x6423;

// Performance Diagnostics
constexpr uint16_t FollowingError          = 0x6430;
constexpr uint16_t MaxFollowingError       = 0x6431;
constexpr uint16_t ControllerOutput        = 0x6432;
constexpr uint16_t SystemDelay             = 0x6433;

// ============================================================================
// Object Dictionary Indices - Calibration (0x6500-0x65FF)
// ============================================================================

constexpr uint16_t CalibrationCommand      = 0x6500;
constexpr uint16_t CalibrationStatus       = 0x6501;
constexpr uint16_t NullPointCalibration    = 0x6502;
constexpr uint16_t GainCalibration         = 0x6503;
constexpr uint16_t LinearizationTable      = 0x6504;

// ============================================================================
// Controlword Bits
// ============================================================================

namespace ControlwordBits {
    constexpr uint16_t Enable              = 0x0001;
    constexpr uint16_t Reset               = 0x0002;
    constexpr uint16_t DirectionA          = 0x0004;
    constexpr uint16_t DirectionB          = 0x0008;
    constexpr uint16_t FastStop            = 0x0010;
    constexpr uint16_t PressureCompEnable  = 0x0020;
    constexpr uint16_t FlowCompEnable      = 0x0040;
    constexpr uint16_t ClosedLoopEnable    = 0x0080;
    constexpr uint16_t OverrideEnable      = 0x0100;
    constexpr uint16_t ManualMode          = 0x0200;

    // Backwards compatibility aliases
    constexpr uint16_t FaultReset          = Reset;
    constexpr uint16_t ClosedLoop          = ClosedLoopEnable;
    constexpr uint16_t PressureCompensation = PressureCompEnable;
}

// Backwards-compatible object alias namespace to allow older call sites like
// Object::ValveType etc. Most code now uses the top-level constants above
// but older code used Object::X naming. Provide convenient aliases.
namespace Object {
    constexpr uint16_t ValveType = ::CiA408::ValveType;
    constexpr uint16_t NominalFlow = ::CiA408::NominalFlow;
    constexpr uint16_t NominalPressure = ::CiA408::NominalPressure;
    constexpr uint16_t NominalStroke = ::CiA408::NominalStroke;
    constexpr uint16_t ResponseTime = ::CiA408::ResponseTime;
    constexpr uint16_t Hysteresis = ::CiA408::Hysteresis;
    constexpr uint16_t Repeatability = ::CiA408::Repeatability;

    constexpr uint16_t PositionActual = ::CiA408::PositionActual;
    constexpr uint16_t ActualValue = ::CiA408::ActualValue;

    constexpr uint16_t PressureA = ::CiA408::PressureActualA;
    constexpr uint16_t PressureB = ::CiA408::PressureActualB;
    constexpr uint16_t PressureP = ::CiA408::PressureActualP;

    constexpr uint16_t CurrentActualA = ::CiA408::CoilCurrentA;
    constexpr uint16_t CurrentActualB = ::CiA408::CoilCurrentB;

    constexpr uint16_t DitherAmplitude = ::CiA408::DitherAmplitude;
    constexpr uint16_t DitherFrequency = ::CiA408::DitherFrequency;
    constexpr uint16_t DitherEnable = ::CiA408::DitherEnable;

    constexpr uint16_t Controlword = ::CiA408::Controlword;
    constexpr uint16_t Statusword = ::CiA408::Statusword;

    // Common aliases
    constexpr uint16_t VelocityActual = ::CiA408::VelocityActual;
    constexpr uint16_t OperatingMode = ::CiA408::OperatingMode;
    constexpr uint16_t Setpoint = ::CiA408::SetpointValue;
    constexpr uint16_t SetpointRamp = ::CiA408::SetpointRamp;
    constexpr uint16_t PositionSetpoint = ::CiA408::PositionSetpoint;
    constexpr uint16_t VelocitySetpoint = ::CiA408::VelocitySetpoint;
    constexpr uint16_t PressureSetpoint = ::CiA408::PressureSetpoint;

    // Additional aliases for legacy names
    constexpr uint16_t SetpointRampUp = ::CiA408::SetpointRamp;
    constexpr uint16_t SetpointRampDown = ::CiA408::SetpointRamp;
    constexpr uint16_t PositionWindow = ::CiA408::PositionWindow;
    constexpr uint16_t PositionWindowTime = ::CiA408::PositionWindowTime;
    constexpr uint16_t PressureLimit = ::CiA408::MaxPressure;
    constexpr uint16_t PositionMin = ::CiA408::MinPositionLimit;
    constexpr uint16_t PositionMax = ::CiA408::MaxPositionLimit;
    constexpr uint16_t VelocityMax = ::CiA408::MaxVelocityLimit;
    constexpr uint16_t Acceleration = ::CiA408::AccelerationLimit;
    constexpr uint16_t CurrentLimitA = ::CiA408::MaxCurrentA;
    constexpr uint16_t CurrentLimitB = ::CiA408::MaxCurrentB;
    constexpr uint16_t SupplyVoltage = ::CiA408::SupplyVoltage;
    constexpr uint16_t OperatingHours = ::CiA408::OperatingHours;
    constexpr uint16_t CycleCount = ::CiA408::CycleCount;
    constexpr uint16_t CalibrationCommand = ::CiA408::CalibrationCommand;
    constexpr uint16_t CalibrationStatus = ::CiA408::CalibrationStatus;

    constexpr uint16_t Temperature = ::CiA408::CoilTemperature;
    constexpr uint16_t FaultCode = ::CiA408::ValveFault;
    constexpr uint16_t WarningCode = ::CiA408::ValveWarning;

    constexpr uint16_t PositionKp = ::CiA408::PosController_Kp;
    constexpr uint16_t PositionKi = ::CiA408::PosController_Ki;
    constexpr uint16_t PositionKd = ::CiA408::PosController_Kd;
    constexpr uint16_t PositionKv = ::CiA408::PosController_Kv;
    constexpr uint16_t PositionKa = ::CiA408::PosController_Ka;
    constexpr uint16_t PositionOutputLimit = ::CiA408::PosController_Limit;

    constexpr uint16_t VelocityKp = ::CiA408::VelController_Kp;
    constexpr uint16_t VelocityKi = ::CiA408::VelController_Ki;
    constexpr uint16_t VelocityOutputLimit = ::CiA408::VelController_Limit;

    constexpr uint16_t PressureKp = ::CiA408::PrsController_Kp;
    constexpr uint16_t PressureKi = ::CiA408::PrsController_Ki;
    constexpr uint16_t PressureOutputLimit = ::CiA408::PrsController_Limit;

    constexpr uint16_t NullOffset = ::CiA408::NullOffset;
    constexpr uint16_t Deadband = ::CiA408::Deadband;
}


// ============================================================================
// Statusword Bits
// ============================================================================

namespace StatuswordBits {
    constexpr uint16_t Ready               = 0x0001;
    constexpr uint16_t Enabled             = 0x0002;
    constexpr uint16_t Fault               = 0x0004;
    constexpr uint16_t Warning             = 0x0008;
    constexpr uint16_t TargetReached       = 0x0010;
    constexpr uint16_t InPosition          = 0x0020;
    constexpr uint16_t InVelocity          = 0x0040;
    constexpr uint16_t InPressure          = 0x0080;
    constexpr uint16_t PressureCompActive  = 0x0100;
    constexpr uint16_t ClosedLoopActive    = 0x0200;
    constexpr uint16_t LimitReached        = 0x0400;
    constexpr uint16_t Overload            = 0x0800;
}

// ============================================================================
// Operating Modes
// ============================================================================

namespace OperatingModes {
    constexpr uint8_t OpenLoop             = 0x00;
    constexpr uint8_t PositionControl      = 0x01;
    constexpr uint8_t VelocityControl      = 0x02;
    constexpr uint8_t PressureControl      = 0x03;
    constexpr uint8_t ForceControl         = 0x04;
    constexpr uint8_t FlowControl          = 0x05;
    constexpr uint8_t Synchronized         = 0x06;
    constexpr uint8_t Manual               = 0x07;
}

// ============================================================================
// Valve Types
// ============================================================================

namespace ValveTypes {
    constexpr uint8_t ProportionalDirectional = 0x01;
    constexpr uint8_t ServoValve              = 0x02;
    constexpr uint8_t ProportionalPressure    = 0x03;
    constexpr uint8_t ProportionalFlow        = 0x04;
    constexpr uint8_t VariablePump            = 0x05;
    constexpr uint8_t ProportionalThrottle    = 0x06;
    constexpr uint8_t OnOffValve              = 0x07;
}

// ============================================================================
// Flow Characteristics
// ============================================================================

namespace FlowCharacteristics {
    constexpr uint8_t Linear               = 0x00;
    constexpr uint8_t Progressive          = 0x01;
    constexpr uint8_t Degressive           = 0x02;
    constexpr uint8_t EqualPercentage      = 0x03;
    constexpr uint8_t Custom               = 0x04;
}

// ============================================================================
// Fault Codes
// ============================================================================

namespace FaultCodes {
    constexpr uint16_t None                = 0x0000;
    constexpr uint16_t Overvoltage         = 0x0001;
    constexpr uint16_t Undervoltage        = 0x0002;
    constexpr uint16_t Overcurrent         = 0x0004;
    constexpr uint16_t CoilOpenA           = 0x0008;
    constexpr uint16_t CoilOpenB           = 0x0010;
    constexpr uint16_t CoilShortA          = 0x0020;
    constexpr uint16_t CoilShortB          = 0x0040;
    constexpr uint16_t Overtemperature     = 0x0080;
    constexpr uint16_t LVDTFault           = 0x0100;
    constexpr uint16_t PressureSensorFault = 0x0200;
    constexpr uint16_t FollowingError      = 0x0400;
    constexpr uint16_t PressureOverload    = 0x0800;
    constexpr uint16_t InternalFault       = 0x1000;
    constexpr uint16_t CommunicationFault  = 0x2000;
}

// ============================================================================
// Warning Codes
// ============================================================================

namespace WarningCodes {
    constexpr uint16_t None                = 0x0000;
    constexpr uint16_t HighTemperature     = 0x0001;
    constexpr uint16_t HighCurrent         = 0x0002;
    constexpr uint16_t LowVoltage          = 0x0004;
    constexpr uint16_t LVDTDegraded        = 0x0008;
    constexpr uint16_t ServiceDue          = 0x0010;
    constexpr uint16_t FilterClogged       = 0x0020;
    constexpr uint16_t OilContamination    = 0x0040;
}

// ============================================================================
// Calibration Commands
// ============================================================================

namespace CalibrationCommands {
    constexpr uint8_t None                 = 0x00;
    constexpr uint8_t StartNullCalibration = 0x01;
    constexpr uint8_t StartGainCalibration = 0x02;
    constexpr uint8_t StartAutoTune        = 0x03;
    constexpr uint8_t StoreCalibration     = 0x10;
    constexpr uint8_t ResetToDefault       = 0x20;
}

// ============================================================================
// PDO Structures
// ============================================================================

#pragma pack(push, 1)

// Basic control PDO
struct RxPDO_Basic {
    uint16_t controlword;
    int16_t  setpoint;         // Percentage * 100 (-10000 to +10000)
};
static_assert(sizeof(RxPDO_Basic) == 4, "RxPDO_Basic size mismatch");

// Basic status PDO
struct TxPDO_Basic {
    uint16_t statusword;
    int16_t  actual_value;     // Percentage * 100
};
static_assert(sizeof(TxPDO_Basic) == 4, "TxPDO_Basic size mismatch");

// Extended control PDO
struct RxPDO_Extended {
    uint16_t controlword;
    int16_t  setpoint;
    int16_t  pressure_setpoint;
    uint8_t  operating_mode;
    uint8_t  reserved;
};
static_assert(sizeof(RxPDO_Extended) == 8, "RxPDO_Extended size mismatch");

// Extended status PDO
struct TxPDO_Extended {
    uint16_t statusword;
    int16_t  actual_value;
    int16_t  pressure_a;       // Pressure * 10 (0.1 bar resolution)
    int16_t  pressure_b;
};
static_assert(sizeof(TxPDO_Extended) == 8, "TxPDO_Extended size mismatch");

// Position control PDO
struct RxPDO_Position {
    uint16_t controlword;
    int32_t  position_setpoint; // Position in micrometers
    int16_t  velocity_limit;    // Velocity * 10 (mm/s)
};
static_assert(sizeof(RxPDO_Position) == 8, "RxPDO_Position size mismatch");

// Position status PDO
struct TxPDO_Position {
    uint16_t statusword;
    int16_t  actual_value;
    int32_t  position_actual;
};
static_assert(sizeof(TxPDO_Position) == 8, "TxPDO_Position size mismatch");

// Full control PDO
struct RxPDO_Full {
    uint16_t controlword;
    int16_t  setpoint;
    int16_t  setpoint2;
    uint8_t  operating_mode;
    uint8_t  override_value;   // 0-100%
};
static_assert(sizeof(RxPDO_Full) == 8, "RxPDO_Full size mismatch");

// Full status PDO
struct TxPDO_Full {
    uint16_t statusword;
    int16_t  actual_value;
    int16_t  pressure_a;
    int16_t  pressure_b;
    int16_t  current_a;        // Current * 10 (mA)
    int16_t  current_b;
    int16_t  temperature;      // Temperature * 10 (°C)
    uint16_t fault_code;
};
static_assert(sizeof(TxPDO_Full) == 16, "TxPDO_Full size mismatch");

#pragma pack(pop)

// ============================================================================
// Helper Functions
// ============================================================================

inline const char* getOperatingModeName(uint8_t mode) {
    switch (mode) {
        case OperatingModes::OpenLoop:         return "Open Loop";
        case OperatingModes::PositionControl:  return "Position Control";
        case OperatingModes::VelocityControl:  return "Velocity Control";
        case OperatingModes::PressureControl:  return "Pressure Control";
        case OperatingModes::ForceControl:     return "Force Control";
        case OperatingModes::FlowControl:      return "Flow Control";
        case OperatingModes::Synchronized:     return "Synchronized";
        case OperatingModes::Manual:           return "Manual";
        default:                               return "Unknown";
    }
}

inline const char* getValveTypeName(uint8_t type) {
    switch (type) {
        case ValveTypes::ProportionalDirectional: return "Proportional Directional";
        case ValveTypes::ServoValve:              return "Servo Valve";
        case ValveTypes::ProportionalPressure:    return "Proportional Pressure";
        case ValveTypes::ProportionalFlow:        return "Proportional Flow";
        case ValveTypes::VariablePump:            return "Variable Pump";
        case ValveTypes::ProportionalThrottle:    return "Proportional Throttle";
        case ValveTypes::OnOffValve:              return "On/Off Valve";
        default:                                  return "Unknown";
    }
}

inline const char* getFaultName(uint16_t fault) {
    if (fault & FaultCodes::Overvoltage) return "Overvoltage";
    if (fault & FaultCodes::Undervoltage) return "Undervoltage";
    if (fault & FaultCodes::Overcurrent) return "Overcurrent";
    if (fault & FaultCodes::CoilOpenA) return "Coil A Open";
    if (fault & FaultCodes::CoilOpenB) return "Coil B Open";
    if (fault & FaultCodes::CoilShortA) return "Coil A Short";
    if (fault & FaultCodes::CoilShortB) return "Coil B Short";
    if (fault & FaultCodes::Overtemperature) return "Overtemperature";
    if (fault & FaultCodes::LVDTFault) return "LVDT Fault";
    if (fault & FaultCodes::PressureSensorFault) return "Pressure Sensor Fault";
    if (fault & FaultCodes::FollowingError) return "Following Error";
    if (fault & FaultCodes::PressureOverload) return "Pressure Overload";
    if (fault & FaultCodes::InternalFault) return "Internal Fault";
    if (fault & FaultCodes::CommunicationFault) return "Communication Fault";
    return "None";
}

// Convert percentage value (±100.00%) to raw (-10000 to +10000)
inline int16_t percentToRaw(float percent) {
    return static_cast<int16_t>(percent * 100.0f);
}

// Convert raw (-10000 to +10000) to percentage (±100.00%)
inline float rawToPercent(int16_t raw) {
    return static_cast<float>(raw) / 100.0f;
}

// Convert bar to raw (0.1 bar resolution)
inline int16_t barToRaw(float bar) {
    return static_cast<int16_t>(bar * 10.0f);
}

// Convert raw to bar
inline float rawToBar(int16_t raw) {
    return static_cast<float>(raw) / 10.0f;
}

} // namespace CiA408
