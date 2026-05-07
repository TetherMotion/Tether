/**
 * @file CiA404Defs.hpp
 * @brief CiA 404 Device Profile Definitions for Measuring Devices and Closed Loop Controllers
 * 
 * @details
 * This file contains comprehensive definitions for CiA 404 compliant devices:
 * - Object dictionary indices for analog measurement devices
 * - Closed loop controller parameters
 * - Process data input/output definitions
 * - Calibration and linearization objects
 * - Alarm and limit monitoring
 * 
 * Reference: CiA 404 v1.1 - CANopen device profile for measuring devices 
 *            and closed loop controllers
 * 
 * @note CiA 404 extends CiA 401 with additional measurement-specific features
 */

#pragma once

#include <cstdint>

namespace CiA404 {

// ============================================================================
// Device Type and Profile Information
// ============================================================================

/** Device type for CiA 404 measuring devices (returned in object 0x1000) */
constexpr uint32_t DEVICE_TYPE = 0x00000194; // Profile 404

/** Profile number */
constexpr uint16_t PROFILE_NUMBER = 404;

// ============================================================================
// Process Data Input Objects (0x6000-0x60FF)
// ============================================================================

/** @name Process Data Input Objects */
/** @{ */

/** Process data input 1 - primary measurement value (0x6000) */
constexpr uint16_t ProcessDataInput1 = 0x6000;

/** Process data input 2 - secondary measurement value (0x6010) */
constexpr uint16_t ProcessDataInput2 = 0x6010;

/** Process data input 3 (0x6020) */
constexpr uint16_t ProcessDataInput3 = 0x6020;

/** Process data input 4 (0x6030) */
constexpr uint16_t ProcessDataInput4 = 0x6030;

/** Process data input 5 (0x6040) */
constexpr uint16_t ProcessDataInput5 = 0x6040;

/** Process data input 6 (0x6050) */
constexpr uint16_t ProcessDataInput6 = 0x6050;

/** Process data input 7 (0x6060) */
constexpr uint16_t ProcessDataInput7 = 0x6060;

/** Process data input 8 (0x6070) */
constexpr uint16_t ProcessDataInput8 = 0x6070;

/** @} */

// ============================================================================
// Process Data Input Subindices
// ============================================================================

namespace PDInputSub {
    /** Number of mapped objects in this PDI */
    constexpr uint8_t NumberOfMappedObjects = 0x00;
    
    /** Process input value (32-bit signed) */
    constexpr uint8_t ProcessInputValue = 0x01;
    
    /** Process input value (16-bit signed) */
    constexpr uint8_t ProcessInputValue16 = 0x02;
    
    /** Process input status */
    constexpr uint8_t ProcessInputStatus = 0x03;
    
    /** Process input vendor specific */
    constexpr uint8_t VendorSpecific = 0x80;
}

// ============================================================================
// Process Data Output Objects (0x6200-0x62FF)
// ============================================================================

/** @name Process Data Output Objects */
/** @{ */

/** Process data output 1 - primary control value (0x6200) */
constexpr uint16_t ProcessDataOutput1 = 0x6200;

/** Process data output 2 - secondary control value (0x6210) */
constexpr uint16_t ProcessDataOutput2 = 0x6210;

/** Process data output 3 (0x6220) */
constexpr uint16_t ProcessDataOutput3 = 0x6220;

/** Process data output 4 (0x6230) */
constexpr uint16_t ProcessDataOutput4 = 0x6230;

/** @} */

namespace PDOutputSub {
    /** Number of mapped objects in this PDO */
    constexpr uint8_t NumberOfMappedObjects = 0x00;
    
    /** Process output value (32-bit signed) */
    constexpr uint8_t ProcessOutputValue = 0x01;
    
    /** Process output value (16-bit signed) */
    constexpr uint8_t ProcessOutputValue16 = 0x02;
    
    /** Process output command */
    constexpr uint8_t ProcessOutputCommand = 0x03;
}

// ============================================================================
// Analog Input Configuration Objects (0x6100-0x61FF)
// ============================================================================

/** @name Analog Input Configuration Objects */
/** @{ */

/** Analog input scaling factor (0x6110) */
constexpr uint16_t AnalogInputScalingFactor = 0x6110;

/** Analog input offset (0x6111) */
constexpr uint16_t AnalogInputOffset = 0x6111;

/** Analog input engineering unit (0x6112) */
constexpr uint16_t AnalogInputUnit = 0x6112;

/** Analog input decimal places (0x6113) */
constexpr uint16_t AnalogInputDecimalPlaces = 0x6113;

/** Analog input range selection (0x6114) */
constexpr uint16_t AnalogInputRange = 0x6114;

/** Analog input sensor type (0x6115) */
constexpr uint16_t AnalogInputSensorType = 0x6115;

/** Analog input filter time constant (0x6116) */
constexpr uint16_t AnalogInputFilterTime = 0x6116;

/** Analog input calibration offset (0x6117) */
constexpr uint16_t AnalogInputCalibrationOffset = 0x6117;

/** Analog input calibration gain (0x6118) */
constexpr uint16_t AnalogInputCalibrationGain = 0x6118;

/** Analog input linearization table (0x6119) */
constexpr uint16_t AnalogInputLinearization = 0x6119;

/** @} */

// ============================================================================
// Analog Output Configuration Objects (0x6300-0x63FF)
// ============================================================================

/** @name Analog Output Configuration Objects */
/** @{ */

/** Analog output scaling factor (0x6310) */
constexpr uint16_t AnalogOutputScalingFactor = 0x6310;

/** Analog output offset (0x6311) */
constexpr uint16_t AnalogOutputOffset = 0x6311;

/** Analog output engineering unit (0x6312) */
constexpr uint16_t AnalogOutputUnit = 0x6312;

/** Analog output decimal places (0x6313) */
constexpr uint16_t AnalogOutputDecimalPlaces = 0x6313;

/** Analog output range selection (0x6314) */
constexpr uint16_t AnalogOutputRange = 0x6314;

/** Analog output error behavior (0x6315) */
constexpr uint16_t AnalogOutputErrorBehavior = 0x6315;

/** Analog output error value (0x6316) */
constexpr uint16_t AnalogOutputErrorValue = 0x6316;

/** @} */

// ============================================================================
// Closed Loop Controller Objects (0x6400-0x64FF)
// ============================================================================

/** @name Closed Loop Controller Objects */
/** @{ */

/** Controller setpoint (0x6400) */
constexpr uint16_t ControllerSetpoint = 0x6400;

/** Controller actual value (feedback) (0x6401) */
constexpr uint16_t ControllerActualValue = 0x6401;

/** Controller control deviation (error) (0x6402) */
constexpr uint16_t ControllerDeviation = 0x6402;

/** Controller output value (0x6403) */
constexpr uint16_t ControllerOutput = 0x6403;

/** Controller mode (0x6410) */
constexpr uint16_t ControllerMode = 0x6410;

/** Controller status (0x6411) */
constexpr uint16_t ControllerStatus = 0x6411;

/** PID proportional gain (Kp) (0x6420) */
constexpr uint16_t PID_Kp = 0x6420;

/** PID integral time (Ti) (0x6421) */
constexpr uint16_t PID_Ti = 0x6421;

/** PID derivative time (Td) (0x6422) */
constexpr uint16_t PID_Td = 0x6422;

/** PID sample time (0x6423) */
constexpr uint16_t PID_SampleTime = 0x6423;

/** PID output upper limit (0x6424) */
constexpr uint16_t PID_OutputUpperLimit = 0x6424;

/** PID output lower limit (0x6425) */
constexpr uint16_t PID_OutputLowerLimit = 0x6425;

/** PID anti-windup limit (0x6426) */
constexpr uint16_t PID_AntiWindup = 0x6426;

/** PID derivative filter coefficient (0x6427) */
constexpr uint16_t PID_DerivativeFilter = 0x6427;

/** Setpoint ramp rate (0x6430) */
constexpr uint16_t SetpointRampRate = 0x6430;

/** Feedforward gain (0x6431) */
constexpr uint16_t FeedforwardGain = 0x6431;

/** @} */

// ============================================================================
// Alarm and Limit Objects (0x6500-0x65FF)
// ============================================================================

/** @name Alarm and Limit Objects */
/** @{ */

/** Alarm status (0x6500) */
constexpr uint16_t AlarmStatus = 0x6500;

/** Warning status (0x6501) */
constexpr uint16_t WarningStatus = 0x6501;

/** Alarm high-high limit (0x6510) */
constexpr uint16_t AlarmHighHighLimit = 0x6510;

/** Alarm high limit (0x6511) */
constexpr uint16_t AlarmHighLimit = 0x6511;

/** Alarm low limit (0x6512) */
constexpr uint16_t AlarmLowLimit = 0x6512;

/** Alarm low-low limit (0x6513) */
constexpr uint16_t AlarmLowLowLimit = 0x6513;

/** Warning high limit (0x6514) */
constexpr uint16_t WarningHighLimit = 0x6514;

/** Warning low limit (0x6515) */
constexpr uint16_t WarningLowLimit = 0x6515;

/** Alarm hysteresis (0x6516) */
constexpr uint16_t AlarmHysteresis = 0x6516;

/** Alarm delay time (0x6517) */
constexpr uint16_t AlarmDelayTime = 0x6517;

/** Rate of change alarm limit (0x6518) */
constexpr uint16_t RateOfChangeLimit = 0x6518;

/** @} */

// ============================================================================
// Calibration Objects (0x6600-0x66FF)
// ============================================================================

/** @name Calibration Objects */
/** @{ */

/** Calibration command (0x6600) */
constexpr uint16_t CalibrationCommand = 0x6600;

/** Calibration status (0x6601) */
constexpr uint16_t CalibrationStatus = 0x6601;

/** Calibration point 1 raw (0x6610) */
constexpr uint16_t CalibrationPoint1Raw = 0x6610;

/** Calibration point 1 engineering (0x6611) */
constexpr uint16_t CalibrationPoint1Eng = 0x6611;

/** Calibration point 2 raw (0x6612) */
constexpr uint16_t CalibrationPoint2Raw = 0x6612;

/** Calibration point 2 engineering (0x6613) */
constexpr uint16_t CalibrationPoint2Eng = 0x6613;

/** Tare command (0x6620) */
constexpr uint16_t TareCommand = 0x6620;

/** Tare value (0x6621) */
constexpr uint16_t TareValue = 0x6621;

/** @} */

// ============================================================================
// Diagnostic Objects (0x6700-0x67FF)
// ============================================================================

/** @name Diagnostic Objects */
/** @{ */

/** Sensor status (0x6700) */
constexpr uint16_t SensorStatus = 0x6700;

/** Sensor supply voltage (0x6701) */
constexpr uint16_t SensorSupplyVoltage = 0x6701;

/** Sensor temperature (0x6702) */
constexpr uint16_t SensorTemperature = 0x6702;

/** Signal quality (0x6703) */
constexpr uint16_t SignalQuality = 0x6703;

/** Operating hours (0x6710) */
constexpr uint16_t OperatingHours = 0x6710;

/** @} */

// ============================================================================
// Controller Mode Values
// ============================================================================

namespace ControllerModes {
    /** Controller disabled/manual mode */
    constexpr uint8_t Manual = 0x00;
    
    /** PID automatic mode */
    constexpr uint8_t PID_Auto = 0x01;
    
    /** P-only control */
    constexpr uint8_t P_Only = 0x02;
    
    /** PI control */
    constexpr uint8_t PI_Control = 0x03;
    
    /** PD control */
    constexpr uint8_t PD_Control = 0x04;
    
    /** Cascade inner loop */
    constexpr uint8_t CascadeInner = 0x10;
    
    /** Cascade outer loop */
    constexpr uint8_t CascadeOuter = 0x11;
    
    /** Ratio control */
    constexpr uint8_t RatioControl = 0x20;
    
    /** Feedforward only */
    constexpr uint8_t FeedforwardOnly = 0x30;
}

// ============================================================================
// Controller Status Bits
// ============================================================================

namespace ControllerStatusBits {
    /** Controller is active */
    constexpr uint16_t Active = 0x0001;
    
    /** Output at upper limit */
    constexpr uint16_t OutputUpperLimit = 0x0002;
    
    /** Output at lower limit */
    constexpr uint16_t OutputLowerLimit = 0x0004;
    
    /** Integrator saturated (anti-windup active) */
    constexpr uint16_t IntegratorSaturated = 0x0008;
    
    /** Setpoint tracking active */
    constexpr uint16_t SetpointTracking = 0x0010;
    
    /** Bumpless transfer active */
    constexpr uint16_t BumplessTransfer = 0x0020;
    
    /** Process value out of range */
    constexpr uint16_t PVOutOfRange = 0x0040;
    
    /** Setpoint out of range */
    constexpr uint16_t SPOutOfRange = 0x0080;
    
    /** Controller fault */
    constexpr uint16_t Fault = 0x8000;
}

// ============================================================================
// Alarm Status Bits
// ============================================================================

namespace AlarmStatusBits {
    /** High-high alarm active */
    constexpr uint16_t HighHigh = 0x0001;
    
    /** High alarm active */
    constexpr uint16_t High = 0x0002;
    
    /** Low alarm active */
    constexpr uint16_t Low = 0x0004;
    
    /** Low-low alarm active */
    constexpr uint16_t LowLow = 0x0008;
    
    /** Rate of change alarm */
    constexpr uint16_t RateOfChange = 0x0010;
    
    /** Sensor error */
    constexpr uint16_t SensorError = 0x0020;
    
    /** Communication error */
    constexpr uint16_t CommError = 0x0040;
    
    /** Calibration required */
    constexpr uint16_t CalibrationRequired = 0x0080;
}

namespace WarningStatusBits {
    /** High warning active */
    constexpr uint16_t High = 0x0001;
    
    /** Low warning active */
    constexpr uint16_t Low = 0x0002;
    
    /** Approaching high limit */
    constexpr uint16_t ApproachingHigh = 0x0004;
    
    /** Approaching low limit */
    constexpr uint16_t ApproachingLow = 0x0008;
}

// ============================================================================
// Calibration Commands
// ============================================================================

namespace CalibrationCommands {
    /** No operation */
    constexpr uint8_t NoOp = 0x00;
    
    /** Start zero calibration */
    constexpr uint8_t CalibrateZero = 0x01;
    
    /** Start span calibration */
    constexpr uint8_t CalibrateSpan = 0x02;
    
    /** Start two-point calibration */
    constexpr uint8_t CalibrateTwoPoint = 0x03;
    
    /** Accept calibration */
    constexpr uint8_t AcceptCalibration = 0x10;
    
    /** Reject/cancel calibration */
    constexpr uint8_t RejectCalibration = 0x11;
    
    /** Restore factory calibration */
    constexpr uint8_t RestoreFactory = 0x20;
    
    /** Save calibration to NVS */
    constexpr uint8_t SaveCalibration = 0x30;
}

// ============================================================================
// Calibration Status Bits
// ============================================================================

namespace CalibrationStatusBits {
    /** Calibration in progress */
    constexpr uint8_t InProgress = 0x01;
    
    /** Waiting for stable reading */
    constexpr uint8_t WaitingStable = 0x02;
    
    /** Calibration successful */
    constexpr uint8_t Success = 0x04;
    
    /** Calibration failed */
    constexpr uint8_t Failed = 0x08;
    
    /** Factory calibration active */
    constexpr uint8_t FactoryActive = 0x10;
    
    /** User calibration active */
    constexpr uint8_t UserActive = 0x20;
}

// ============================================================================
// Sensor Status Bits
// ============================================================================

namespace SensorStatusBits {
    /** Sensor OK */
    constexpr uint8_t OK = 0x00;
    
    /** Sensor not connected */
    constexpr uint8_t NotConnected = 0x01;
    
    /** Short circuit detected */
    constexpr uint8_t ShortCircuit = 0x02;
    
    /** Open circuit detected */
    constexpr uint8_t OpenCircuit = 0x04;
    
    /** Over range */
    constexpr uint8_t OverRange = 0x08;
    
    /** Under range */
    constexpr uint8_t UnderRange = 0x10;
    
    /** Supply voltage fault */
    constexpr uint8_t SupplyFault = 0x20;
    
    /** Temperature fault */
    constexpr uint8_t TemperatureFault = 0x40;
}

// ============================================================================
// Input Range Types
// ============================================================================

namespace InputRange {
    /** 0-10V voltage input */
    constexpr uint8_t Voltage_0_10V = 0x01;
    
    /** -10V to +10V voltage input */
    constexpr uint8_t Voltage_PM10V = 0x02;
    
    /** 0-5V voltage input */
    constexpr uint8_t Voltage_0_5V = 0x03;
    
    /** 1-5V voltage input */
    constexpr uint8_t Voltage_1_5V = 0x04;
    
    /** 4-20mA current input */
    constexpr uint8_t Current_4_20mA = 0x10;
    
    /** 0-20mA current input */
    constexpr uint8_t Current_0_20mA = 0x11;
    
    /** PT100 RTD */
    constexpr uint8_t RTD_PT100 = 0x20;
    
    /** PT1000 RTD */
    constexpr uint8_t RTD_PT1000 = 0x21;
    
    /** Type J thermocouple */
    constexpr uint8_t TC_Type_J = 0x30;
    
    /** Type K thermocouple */
    constexpr uint8_t TC_Type_K = 0x31;
    
    /** Type T thermocouple */
    constexpr uint8_t TC_Type_T = 0x32;
    
    /** Strain gauge bridge */
    constexpr uint8_t StrainGauge = 0x40;
    
    /** LVDT/RVDT */
    constexpr uint8_t LVDT = 0x50;
}

// ============================================================================
// Output Range Types
// ============================================================================

namespace OutputRange {
    /** 0-10V voltage output */
    constexpr uint8_t Voltage_0_10V = 0x01;
    
    /** -10V to +10V voltage output */
    constexpr uint8_t Voltage_PM10V = 0x02;
    
    /** 0-5V voltage output */
    constexpr uint8_t Voltage_0_5V = 0x03;
    
    /** 4-20mA current output */
    constexpr uint8_t Current_4_20mA = 0x10;
    
    /** 0-20mA current output */
    constexpr uint8_t Current_0_20mA = 0x11;
}

// ============================================================================
// Engineering Units (SI Unit Codes)
// ============================================================================

namespace EngineeringUnit {
    /** Dimensionless / percent */
    constexpr uint16_t Percent = 0x0001;
    
    /** Degrees Celsius */
    constexpr uint16_t DegreesCelsius = 0x0002;
    
    /** Degrees Fahrenheit */
    constexpr uint16_t DegreesFahrenheit = 0x0003;
    
    /** Kelvin */
    constexpr uint16_t Kelvin = 0x0004;
    
    /** Pascals */
    constexpr uint16_t Pascals = 0x0010;
    
    /** Bar */
    constexpr uint16_t Bar = 0x0011;
    
    /** PSI */
    constexpr uint16_t PSI = 0x0012;
    
    /** Millimeters */
    constexpr uint16_t Millimeters = 0x0020;
    
    /** Meters */
    constexpr uint16_t Meters = 0x0021;
    
    /** Inches */
    constexpr uint16_t Inches = 0x0022;
    
    /** Liters per minute */
    constexpr uint16_t LitersPerMinute = 0x0030;
    
    /** Cubic meters per hour */
    constexpr uint16_t CubicMetersPerHour = 0x0031;
    
    /** Gallons per minute */
    constexpr uint16_t GallonsPerMinute = 0x0032;
    
    /** Kilograms */
    constexpr uint16_t Kilograms = 0x0040;
    
    /** Newtons */
    constexpr uint16_t Newtons = 0x0041;
    
    /** Volts */
    constexpr uint16_t Volts = 0x0050;
    
    /** Millivolts */
    constexpr uint16_t Millivolts = 0x0051;
    
    /** Amperes */
    constexpr uint16_t Amperes = 0x0060;
    
    /** Milliamperes */
    constexpr uint16_t Milliamperes = 0x0061;
    
    /** Hertz */
    constexpr uint16_t Hertz = 0x0070;
    
    /** RPM */
    constexpr uint16_t RPM = 0x0071;
    
    /** pH */
    constexpr uint16_t pH = 0x0080;
    
    /** Conductivity (µS/cm) */
    constexpr uint16_t Conductivity = 0x0081;
}

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief PID controller parameters
 */
struct PIDParameters {
    int32_t kp;             ///< Proportional gain (Q16 fixed point)
    int32_t ti;             ///< Integral time in ms (0 = disabled)
    int32_t td;             ///< Derivative time in ms (0 = disabled)
    uint16_t sample_time;   ///< Sample time in ms
    int32_t output_max;     ///< Output upper limit
    int32_t output_min;     ///< Output lower limit
    int32_t anti_windup;    ///< Anti-windup limit
    uint8_t derivative_filter; ///< Derivative filter coefficient (0-255)
    
    PIDParameters()
        : kp(0x10000)  // 1.0 in Q16
        , ti(0)
        , td(0)
        , sample_time(100) // 100ms
        , output_max(32767)
        , output_min(-32768)
        , anti_windup(32767)
        , derivative_filter(0)
    {}
};

/**
 * @brief Analog input channel configuration
 */
struct AnalogInputChannelConfig {
    int32_t scaling_factor;     ///< Scaling factor (Q16)
    int32_t offset;             ///< Offset value
    uint16_t engineering_unit;  ///< Engineering unit code
    uint8_t decimal_places;     ///< Number of decimal places
    uint8_t input_range;        ///< Input range type
    uint8_t sensor_type;        ///< Sensor type code
    uint16_t filter_time;       ///< Filter time constant in ms
    
    AnalogInputChannelConfig()
        : scaling_factor(0x10000)
        , offset(0)
        , engineering_unit(EngineeringUnit::Percent)
        , decimal_places(2)
        , input_range(InputRange::Current_4_20mA)
        , sensor_type(0)
        , filter_time(100)
    {}
};

/**
 * @brief Alarm configuration
 */
struct AlarmConfig {
    int32_t high_high_limit;    ///< High-high alarm limit
    int32_t high_limit;         ///< High alarm limit
    int32_t low_limit;          ///< Low alarm limit
    int32_t low_low_limit;      ///< Low-low alarm limit
    int32_t warning_high;       ///< High warning limit
    int32_t warning_low;        ///< Low warning limit
    int32_t hysteresis;         ///< Alarm hysteresis
    uint16_t delay_time;        ///< Alarm delay time in ms
    int32_t rate_of_change;     ///< Rate of change limit
    
    AlarmConfig()
        : high_high_limit(32767)
        , high_limit(30000)
        , low_limit(-30000)
        , low_low_limit(-32768)
        , warning_high(25000)
        , warning_low(-25000)
        , hysteresis(100)
        , delay_time(1000)
        , rate_of_change(0)
    {}
};

/**
 * @brief Calibration data
 */
struct CalibrationData {
    int32_t zero_raw;           ///< Raw value at zero
    int32_t zero_eng;           ///< Engineering value at zero
    int32_t span_raw;           ///< Raw value at span
    int32_t span_eng;           ///< Engineering value at span
    int32_t tare_value;         ///< Current tare offset
    uint8_t status;             ///< Calibration status
    
    CalibrationData()
        : zero_raw(0)
        , zero_eng(0)
        , span_raw(32767)
        , span_eng(10000)
        , tare_value(0)
        , status(0)
    {}
};

// ============================================================================
// PDO Structures for Real-Time Data
// ============================================================================

/**
 * @brief TxPDO for process data input with status
 */
struct __attribute__((packed)) TxPDO_ProcessInput {
    int32_t value;          ///< Process input value
    uint8_t status;         ///< Input status
};

/**
 * @brief TxPDO for multiple process inputs
 */
struct __attribute__((packed)) TxPDO_MultiInput {
    int16_t input1;         ///< Process input 1 (16-bit)
    int16_t input2;         ///< Process input 2
    int16_t input3;         ///< Process input 3
    int16_t input4;         ///< Process input 4
};

/**
 * @brief RxPDO for process data output
 */
struct __attribute__((packed)) RxPDO_ProcessOutput {
    int32_t value;          ///< Process output value
    uint8_t command;        ///< Output command
};

/**
 * @brief TxPDO for controller status
 */
struct __attribute__((packed)) TxPDO_ControllerStatus {
    int32_t actual_value;   ///< Process variable
    int32_t output;         ///< Controller output
    int32_t deviation;      ///< Control deviation
    uint16_t status;        ///< Controller status
};

/**
 * @brief RxPDO for controller setpoint
 */
struct __attribute__((packed)) RxPDO_ControllerSetpoint {
    int32_t setpoint;       ///< Controller setpoint
    uint8_t mode;           ///< Controller mode
};

/**
 * @brief TxPDO for alarm status
 */
struct __attribute__((packed)) TxPDO_AlarmStatus {
    int32_t value;          ///< Current value
    uint16_t alarm_status;  ///< Alarm status bits
    uint16_t warning_status;///< Warning status bits
};

} // namespace CiA404
