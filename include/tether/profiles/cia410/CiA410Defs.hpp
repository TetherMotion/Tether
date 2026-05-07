/**
 * @file CiA410Defs.hpp
 * @brief CiA 410 Inclinometer Device Profile Object Dictionary
 *
 * Defines object dictionary entries for CiA 410 compliant inclinometers
 * and tilt sensors used in industrial automation.
 *
 * Features:
 * - Single and dual axis measurement
 * - Acceleration-based and gyroscopic sensing
 * - Temperature compensation
 * - Configurable measurement ranges
 * - Alarm and threshold monitoring
 */

#pragma once

#include <cstdint>

namespace CiA410 {

// ============================================================================
// Profile Identification
// ============================================================================

constexpr uint16_t PROFILE_NUMBER = 410;
constexpr uint16_t PROFILE_VERSION = 0x0100;

// ============================================================================
// Object Dictionary Indices
// ============================================================================

namespace Object {

// Device Information (0x6000-0x600F)
constexpr uint16_t DeviceType         = 0x6000;
constexpr uint16_t MeasurementType    = 0x6001;
constexpr uint16_t SensorType         = 0x6002;
constexpr uint16_t NumberOfAxes       = 0x6003;
constexpr uint16_t MeasurementRange   = 0x6004;
constexpr uint16_t Resolution         = 0x6005;
constexpr uint16_t Accuracy           = 0x6006;
constexpr uint16_t ResponseTime       = 0x6007;

// Angle Measurements (0x6010-0x601F)
constexpr uint16_t AngleActualX       = 0x6010;
constexpr uint16_t AngleActualY       = 0x6011;
constexpr uint16_t AngleActualZ       = 0x6012;  // For 3-axis devices
constexpr uint16_t AngleVelocityX     = 0x6013;
constexpr uint16_t AngleVelocityY     = 0x6014;
constexpr uint16_t AngleVelocityZ     = 0x6015;
constexpr uint16_t TotalAngle         = 0x6016;  // Resultant tilt
constexpr uint16_t AzimuthAngle       = 0x6017;  // Direction of tilt

// Raw Acceleration (0x6020-0x602F)
constexpr uint16_t AccelerationX      = 0x6020;
constexpr uint16_t AccelerationY      = 0x6021;
constexpr uint16_t AccelerationZ      = 0x6022;
constexpr uint16_t AccelerationTotal  = 0x6023;

// Gyroscope Data (0x6030-0x603F) - For MEMS gyro devices
constexpr uint16_t GyroRateX          = 0x6030;
constexpr uint16_t GyroRateY          = 0x6031;
constexpr uint16_t GyroRateZ          = 0x6032;
constexpr uint16_t GyroBiasX          = 0x6033;
constexpr uint16_t GyroBiasY          = 0x6034;
constexpr uint16_t GyroBiasZ          = 0x6035;

// Status and Control (0x6040-0x604F)
constexpr uint16_t Statusword         = 0x6040;
constexpr uint16_t Controlword        = 0x6041;
constexpr uint16_t OperatingMode      = 0x6042;
constexpr uint16_t FilterSetting      = 0x6043;
constexpr uint16_t SampleRate         = 0x6044;
constexpr uint16_t AveragingCount     = 0x6045;

// Calibration (0x6050-0x605F)
constexpr uint16_t CalibrationCommand = 0x6050;
constexpr uint16_t CalibrationStatus  = 0x6051;
constexpr uint16_t ZeroOffsetX        = 0x6052;
constexpr uint16_t ZeroOffsetY        = 0x6053;
constexpr uint16_t ZeroOffsetZ        = 0x6054;
constexpr uint16_t ScaleFactorX       = 0x6055;
constexpr uint16_t ScaleFactorY       = 0x6056;
constexpr uint16_t ScaleFactorZ       = 0x6057;
constexpr uint16_t CrossAxisX         = 0x6058;  // Cross-axis compensation
constexpr uint16_t CrossAxisY         = 0x6059;
constexpr uint16_t CrossAxisZ         = 0x605A;

// Temperature (0x6060-0x606F)
constexpr uint16_t Temperature        = 0x6060;
constexpr uint16_t TempCoeffX         = 0x6061;
constexpr uint16_t TempCoeffY         = 0x6062;
constexpr uint16_t TempCompEnable     = 0x6063;
constexpr uint16_t TempReferencePoint = 0x6064;

// Alarms and Thresholds (0x6070-0x607F)
constexpr uint16_t AlarmStatus        = 0x6070;
constexpr uint16_t AlarmEnable        = 0x6071;
constexpr uint16_t AngleThresholdHigh = 0x6072;
constexpr uint16_t AngleThresholdLow  = 0x6073;
constexpr uint16_t AccelThresholdHigh = 0x6074;
constexpr uint16_t TempThresholdHigh  = 0x6075;
constexpr uint16_t TempThresholdLow   = 0x6076;
constexpr uint16_t VelocityThreshold  = 0x6077;
constexpr uint16_t Hysteresis         = 0x6078;

// Mounting Configuration (0x6080-0x608F)
constexpr uint16_t MountingOrientation = 0x6080;
constexpr uint16_t MountingRotationX   = 0x6081;
constexpr uint16_t MountingRotationY   = 0x6082;
constexpr uint16_t MountingRotationZ   = 0x6083;
constexpr uint16_t GravityReference    = 0x6084;

// Diagnostics (0x6090-0x609F)
constexpr uint16_t FaultCode          = 0x6090;
constexpr uint16_t WarningCode        = 0x6091;
constexpr uint16_t SensorHealth       = 0x6092;
constexpr uint16_t SignalQuality      = 0x6093;
constexpr uint16_t OperatingHours     = 0x6094;
constexpr uint16_t PowerCycles        = 0x6095;
constexpr uint16_t LastCalibration    = 0x6096;

} // namespace Object

// ============================================================================
// Statusword Bits
// ============================================================================

namespace StatuswordBits {
constexpr uint16_t Ready              = 0x0001;
constexpr uint16_t DataValid          = 0x0002;
constexpr uint16_t Calibrated         = 0x0004;
constexpr uint16_t MotionDetected     = 0x0008;
constexpr uint16_t AlarmActive        = 0x0010;
constexpr uint16_t Warning            = 0x0020;
constexpr uint16_t Fault              = 0x0040;
constexpr uint16_t TempCompActive     = 0x0080;
constexpr uint16_t Settling           = 0x0100;  // Filter settling
constexpr uint16_t OverRange          = 0x0200;
constexpr uint16_t SelfTestOK         = 0x0400;
constexpr uint16_t CalibrationActive  = 0x0800;
} // namespace StatuswordBits

// ============================================================================
// Controlword Bits
// ============================================================================

namespace ControlwordBits {
constexpr uint16_t Enable             = 0x0001;
constexpr uint16_t ResetFault         = 0x0002;
constexpr uint16_t StartCalibration   = 0x0004;
constexpr uint16_t SetZero            = 0x0008;  // Tare/zero
constexpr uint16_t EnableTempComp     = 0x0010;
constexpr uint16_t SelfTest           = 0x0020;
constexpr uint16_t StoreConfig        = 0x0040;
constexpr uint16_t ResetToDefault     = 0x0080;
} // namespace ControlwordBits

// ============================================================================
// Device Types
// ============================================================================

namespace DeviceType {
constexpr uint8_t SingleAxis          = 0x00;
constexpr uint8_t DualAxis            = 0x01;
constexpr uint8_t ThreeAxis           = 0x02;
constexpr uint8_t WithGyroscope       = 0x10;  // Flag
constexpr uint8_t HighPrecision       = 0x20;  // Flag
} // namespace DeviceType

// ============================================================================
// Sensor Types
// ============================================================================

namespace SensorType {
constexpr uint8_t MEMS_Capacitive     = 0x01;
constexpr uint8_t MEMS_Piezoresistive = 0x02;
constexpr uint8_t FluidLevel          = 0x03;
constexpr uint8_t Pendulum            = 0x04;
constexpr uint8_t FiberOptic          = 0x05;
constexpr uint8_t MEMS_Gyro           = 0x06;
} // namespace SensorType

// ============================================================================
// Operating Modes
// ============================================================================

namespace OperatingMode {
constexpr uint8_t Continuous          = 0x00;
constexpr uint8_t Triggered           = 0x01;
constexpr uint8_t OnDemand            = 0x02;
constexpr uint8_t LowPower            = 0x03;
} // namespace OperatingMode

// ============================================================================
// Filter Settings
// ============================================================================

namespace FilterSetting {
constexpr uint8_t NoFilter            = 0x00;
constexpr uint8_t LowPass_1Hz         = 0x01;
constexpr uint8_t LowPass_5Hz         = 0x02;
constexpr uint8_t LowPass_10Hz        = 0x03;
constexpr uint8_t LowPass_50Hz        = 0x04;
constexpr uint8_t LowPass_100Hz       = 0x05;
constexpr uint8_t Moving_Average_4    = 0x10;
constexpr uint8_t Moving_Average_8    = 0x11;
constexpr uint8_t Moving_Average_16   = 0x12;
constexpr uint8_t Moving_Average_32   = 0x13;
constexpr uint8_t Kalman              = 0x20;
constexpr uint8_t Complementary       = 0x21;
} // namespace FilterSetting

// ============================================================================
// Calibration Commands
// ============================================================================

namespace CalibrationCommand {
constexpr uint8_t None                = 0x00;
constexpr uint8_t AutoZero            = 0x01;
constexpr uint8_t OnePoint            = 0x02;
constexpr uint8_t TwoPoint            = 0x03;
constexpr uint8_t MultiPoint          = 0x04;
constexpr uint8_t GyroBiasCalib       = 0x05;
constexpr uint8_t TempCalibration     = 0x06;
constexpr uint8_t CrossAxisCalib      = 0x07;
constexpr uint8_t StoreCalibration    = 0x10;
constexpr uint8_t ResetCalibration    = 0x11;
} // namespace CalibrationCommand

// ============================================================================
// Alarm Bits
// ============================================================================

namespace AlarmBits {
constexpr uint16_t AngleHighX         = 0x0001;
constexpr uint16_t AngleLowX          = 0x0002;
constexpr uint16_t AngleHighY         = 0x0004;
constexpr uint16_t AngleLowY          = 0x0008;
constexpr uint16_t AccelHigh          = 0x0010;
constexpr uint16_t TempHigh           = 0x0020;
constexpr uint16_t TempLow            = 0x0040;
constexpr uint16_t MotionExceeded     = 0x0080;
constexpr uint16_t SensorFailure      = 0x0100;
constexpr uint16_t CalibrationLost    = 0x0200;
} // namespace AlarmBits

// ============================================================================
// Mounting Orientations
// ============================================================================

namespace MountingOrientation {
constexpr uint8_t Horizontal_ZUp      = 0x00;
constexpr uint8_t Horizontal_ZDown    = 0x01;
constexpr uint8_t Vertical_XUp        = 0x02;
constexpr uint8_t Vertical_XDown      = 0x03;
constexpr uint8_t Vertical_YUp        = 0x04;
constexpr uint8_t Vertical_YDown      = 0x05;
constexpr uint8_t Custom              = 0xFF;
} // namespace MountingOrientation

// ============================================================================
// Fault Codes
// ============================================================================

namespace FaultCode {
constexpr uint16_t NoFault            = 0x0000;
constexpr uint16_t SensorFailure      = 0x0001;
constexpr uint16_t ADCError           = 0x0002;
constexpr uint16_t CommunicationError = 0x0003;
constexpr uint16_t OverTemperature    = 0x0004;
constexpr uint16_t UnderTemperature   = 0x0005;
constexpr uint16_t CalibrationError   = 0x0006;
constexpr uint16_t OverRange          = 0x0007;
constexpr uint16_t MemoryError        = 0x0008;
constexpr uint16_t SelfTestFailed     = 0x0009;
} // namespace FaultCode

// ============================================================================
// Unit Conversions
// ============================================================================

// Angle in 0.001 degrees (millidegrees)
inline float millidegToDeg(int32_t md) { return md / 1000.0f; }
inline int32_t degToMillideg(float deg) { return static_cast<int32_t>(deg * 1000.0f); }

// Acceleration in 0.001 g (milli-g)
inline float milligToG(int32_t mg) { return mg / 1000.0f; }
inline int32_t gToMillig(float g) { return static_cast<int32_t>(g * 1000.0f); }

// Angular velocity in 0.01 deg/s
inline float rawToDegsPerSec(int16_t raw) { return raw / 100.0f; }
inline int16_t degsPerSecToRaw(float dps) { return static_cast<int16_t>(dps * 100.0f); }

// Temperature in 0.1°C
inline float rawToTempC(int16_t raw) { return raw / 10.0f; }
inline int16_t tempCToRaw(float c) { return static_cast<int16_t>(c * 10.0f); }

// ============================================================================
// PDO Structures
// ============================================================================

#pragma pack(push, 1)

// Basic single-axis TxPDO
struct TxPDO_SingleAxis {
    uint16_t statusword;
    int32_t  angle_x;      // millidegrees
};

// Dual-axis TxPDO
struct TxPDO_DualAxis {
    uint16_t statusword;
    int32_t  angle_x;      // millidegrees
    int32_t  angle_y;      // millidegrees
};

// Extended TxPDO with acceleration
struct TxPDO_Extended {
    uint16_t statusword;
    int32_t  angle_x;
    int32_t  angle_y;
    int32_t  accel_x;      // milli-g
    int32_t  accel_y;      // milli-g
    int16_t  temperature;  // 0.1°C
};

// Full TxPDO with gyro
struct TxPDO_Full {
    uint16_t statusword;
    int32_t  angle_x;
    int32_t  angle_y;
    int32_t  angle_z;
    int16_t  velocity_x;   // 0.01 deg/s
    int16_t  velocity_y;
    int16_t  velocity_z;
    int32_t  accel_x;
    int32_t  accel_y;
    int32_t  accel_z;
    int16_t  temperature;
    uint16_t alarm_status;
};

// Basic RxPDO
struct RxPDO_Basic {
    uint16_t controlword;
};

// Extended RxPDO
struct RxPDO_Extended {
    uint16_t controlword;
    uint8_t  filter_setting;
    uint8_t  sample_rate;
};

#pragma pack(pop)

} // namespace CiA410
