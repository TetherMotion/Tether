/**
 * @file CiA406Defs.hpp
 * @brief CiA 406 Encoder Device Profile Object Dictionary Definitions
 * 
 * @details
 * This file contains comprehensive definitions for CiA 406 encoder and
 * position sensor devices according to the CANopen device profile DS-406.
 * 
 * ## CiA 406 Overview
 * 
 * The CiA 406 profile defines standardized objects for:
 * - Absolute and incremental encoders
 * - Single-turn and multi-turn encoders
 * - Linear and rotary position sensors
 * - Various encoder interfaces (SSI, BiSS, EnDat, SinCos, etc.)
 * 
 * ## Object Dictionary Structure
 * 
 * | Range         | Description                                    |
 * |---------------|------------------------------------------------|
 * | 0x6000-0x600F | Operating parameters                           |
 * | 0x6010-0x601F | Pre-set value and operating status             |
 * | 0x6020-0x603F | Position value objects                         |
 * | 0x6040-0x605F | Velocity and acceleration                      |
 * | 0x6060-0x607F | Working area monitoring                        |
 * | 0x6080-0x609F | Alarms and warnings                            |
 * | 0x60A0-0x60BF | Supported alarms                               |
 * | 0x60C0-0x60FF | Diagnostics                                    |
 * | 0x6100-0x61FF | Configuration parameters                       |
 * | 0x6200-0x62FF | Class specific parameters                      |
 * | 0x6300-0x63FF | Manufacturer specific                          |
 * | 0x6400-0x64FF | Scaling function                               |
 * | 0x6500-0x65FF | Absolute encoder multi-turn                    |
 * 
 * ## Encoder Classes
 * 
 * | Class | Type                    | Subclasses                          |
 * |-------|-------------------------|-------------------------------------|
 * | C1    | Absolute single-turn    | Rotary, Linear                      |
 * | C2    | Absolute multi-turn     | Rotary, Linear                      |
 * | C3    | Incremental             | Rotary, Linear                      |
 * | C4    | Incremental with limit  | Rotary, Linear                      |
 * 
 * @see CiA 406 DS-406 CANopen device profile for encoders
 * @see ETG.5000.3 EtherCAT encoder profile (similar structure)
 */

#pragma once

#include <cstdint>

namespace CiA406 {

// ============================================================================
// Device Type and Identification
// ============================================================================

/**
 * @brief CiA 406 Device Type value
 * 
 * Object 0x1000 returns this value for CiA 406 compliant devices.
 * Lower 16 bits: Profile number (406)
 * Upper 16 bits: Implementation type
 */
constexpr uint32_t DeviceType = 0x00000196; // 406 in hex

/**
 * @brief Encoder class definitions
 */
enum class EncoderClass : uint8_t {
    AbsoluteSingleTurn = 1,  ///< C1: Absolute single-turn encoder
    AbsoluteMultiTurn  = 2,  ///< C2: Absolute multi-turn encoder
    Incremental        = 3,  ///< C3: Incremental encoder
    IncrementalLimit   = 4,  ///< C4: Incremental with limit switches
};

/**
 * @brief Encoder subclass/type definitions
 */
enum class EncoderType : uint8_t {
    Rotary = 0,   ///< Rotary encoder
    Linear = 1,   ///< Linear encoder/scale
};

// ============================================================================
// Operating Parameters (0x6000 - 0x600F)
// ============================================================================

/**
 * @brief Operating Parameters (0x6000)
 * 
 * Record containing general operating parameters:
 * - Sub 1: Coding sequence (CW/CCW direction)
 * - Sub 2: Scaling enabled
 * - Sub 3: Class selection
 */
constexpr uint16_t OperatingParameters          = 0x6000;

namespace OperatingParametersSub {
    constexpr uint8_t NumberOfEntries           = 0x00;
    constexpr uint8_t CodingSequence            = 0x01;  ///< 0=CW increasing, 1=CCW increasing
    constexpr uint8_t ScalingFunctionEnabled    = 0x02;  ///< 0=disabled, 1=enabled
    constexpr uint8_t EncoderClass              = 0x03;  ///< Encoder class C1-C4
}

/**
 * @brief Measuring Units per Revolution (0x6001)
 * 
 * Total increments/resolution per revolution for rotary encoders.
 * For linear encoders, this represents increments per measuring unit.
 */
constexpr uint16_t MeasuringUnitsPerRevolution  = 0x6001;

/**
 * @brief Total Measuring Range (0x6002)
 * 
 * Total measuring range in encoder units.
 * For absolute encoders: Maximum position value + 1
 * For incremental encoders: Typically 2^resolution
 */
constexpr uint16_t TotalMeasuringRange          = 0x6002;

/**
 * @brief Preset Value (0x6003)
 * 
 * Value to preset/initialize the encoder position counter.
 * Writing sets the current position to this value.
 */
constexpr uint16_t PresetValue                  = 0x6003;

/**
 * @brief Position Value (0x6004)
 * 
 * Current encoder position (read-only during normal operation).
 * This is the main position output.
 */
constexpr uint16_t PositionValue                = 0x6004;

/**
 * @brief Position Value 2 (0x6005)
 * 
 * Secondary position value for multi-axis or redundant position.
 * Not available on all encoders.
 */
constexpr uint16_t PositionValue2               = 0x6005;

// ============================================================================
// Operating Status (0x6010 - 0x601F)
// ============================================================================

/**
 * @brief Operating Status (0x6010)
 * 
 * Bitfield showing encoder operational status:
 * - Bit 0: Position valid
 * - Bit 1: Scaling function active
 * - Bit 2: Reference done (homing complete)
 * - Bit 3: Preset executed
 * - Bit 4-7: Reserved
 * - Bit 8-15: Manufacturer specific
 */
constexpr uint16_t OperatingStatusObject        = 0x6010;

namespace OperatingStatusBits {
    constexpr uint16_t PositionValid            = 0x0001;
    constexpr uint16_t ScalingActive            = 0x0002;
    constexpr uint16_t ReferenceDone            = 0x0004;
    constexpr uint16_t PresetExecuted           = 0x0008;
    constexpr uint16_t OverspeedWarning         = 0x0010;
    constexpr uint16_t CountingRangeExceeded    = 0x0020;
    constexpr uint16_t SupplyVoltageLow         = 0x0040;
    constexpr uint16_t SupplyVoltageHigh        = 0x0080;
}

/**
 * @brief Single-turn Resolution (0x6011)
 * 
 * Bits of resolution for single-turn position.
 * E.g., 13 means 2^13 = 8192 steps per revolution.
 */
constexpr uint16_t SingleTurnResolution         = 0x6011;

/**
 * @brief Number of Distinguishable Revolutions (0x6012)
 * 
 * For multi-turn encoders: Number of complete revolutions that can be
 * distinguished. Often expressed as 2^n where n is multi-turn bits.
 */
constexpr uint16_t DistinguishableRevolutions   = 0x6012;

// ============================================================================
// Velocity and Acceleration (0x6020 - 0x603F)
// ============================================================================

/**
 * @brief Velocity Actual Value (0x6020)
 * 
 * Current velocity calculated from position change.
 * Units depend on configuration (RPM, counts/s, etc.)
 */
constexpr uint16_t VelocityActualValue          = 0x6020;

/**
 * @brief Velocity Demand Value (0x6021)
 * 
 * Target velocity for velocity-controlled operation.
 */
constexpr uint16_t VelocityDemandValue          = 0x6021;

/**
 * @brief Velocity Sensor Actual Value (0x6022)
 * 
 * Raw velocity from velocity sensor if available.
 */
constexpr uint16_t VelocitySensorValue          = 0x6022;

/**
 * @brief Acceleration Actual Value (0x6030)
 * 
 * Current acceleration calculated from velocity change.
 */
constexpr uint16_t AccelerationActualValue      = 0x6030;

// ============================================================================
// Working Area Monitoring (0x6040 - 0x605F)
// ============================================================================

/**
 * @brief Working Area Low Limit 1 (0x6040)
 */
constexpr uint16_t WorkingAreaLowLimit1         = 0x6040;

/**
 * @brief Working Area High Limit 1 (0x6041)
 */
constexpr uint16_t WorkingAreaHighLimit1        = 0x6041;

/**
 * @brief Working Area Low Limit 2 (0x6042)
 */
constexpr uint16_t WorkingAreaLowLimit2         = 0x6042;

/**
 * @brief Working Area High Limit 2 (0x6043)
 */
constexpr uint16_t WorkingAreaHighLimit2        = 0x6043;

/**
 * @brief Working Area State (0x6044)
 * 
 * Current state relative to working area limits:
 * - 0: Within working area 1
 * - 1: Below low limit 1
 * - 2: Above high limit 1
 * - 3: Within working area 2
 * etc.
 */
constexpr uint16_t WorkingAreaStateObject             = 0x6044;

// ============================================================================
// Alarm and Warning Objects (0x6050 - 0x607F)
// ============================================================================

/**
 * @brief Alarm Object (0x6050)
 * 
 * Current active alarms bitfield:
 * - Bit 0: General hardware error
 * - Bit 1: Temperature exceeded
 * - Bit 2: Light source error (optical encoders)
 * - Bit 3: Battery/backup power error
 * - Bit 4: Position error
 * - Bit 5: Communication error (interface encoders)
 * - Bit 6: Multi-turn error
 * - Bit 7: Initialization error
 */
constexpr uint16_t AlarmObject                  = 0x6050;

namespace AlarmBits {
    constexpr uint16_t HardwareError            = 0x0001;
    constexpr uint16_t TemperatureExceeded      = 0x0002;
    constexpr uint16_t LightSourceError         = 0x0004;
    constexpr uint16_t BatteryError             = 0x0008;
    constexpr uint16_t PositionError            = 0x0010;
    constexpr uint16_t CommunicationError       = 0x0020;
    constexpr uint16_t MultiTurnError           = 0x0040;
    constexpr uint16_t InitializationError      = 0x0080;
    constexpr uint16_t SpeedExceeded            = 0x0100;
    constexpr uint16_t PositionLimitExceeded    = 0x0200;
}

/**
 * @brief Warning Object (0x6051)
 * 
 * Current active warnings bitfield (similar structure to alarms).
 */
constexpr uint16_t WarningObject                = 0x6051;

/**
 * @brief Supported Alarms (0x6052)
 * 
 * Bitmap indicating which alarms are supported by this encoder.
 */
constexpr uint16_t SupportedAlarms              = 0x6052;

/**
 * @brief Supported Warnings (0x6053)
 * 
 * Bitmap indicating which warnings are supported.
 */
constexpr uint16_t SupportedWarnings            = 0x6053;

/**
 * @brief Alarm History (0x6054)
 * 
 * Record containing alarm history:
 * - Sub 0: Number of entries
 * - Sub 1-n: Historical alarm codes with timestamps
 */
constexpr uint16_t AlarmHistory                 = 0x6054;

// ============================================================================
// Configuration Parameters (0x6100 - 0x61FF)
// ============================================================================

/**
 * @brief Counting Direction (0x6100)
 * 
 * 0: CW = increasing counts
 * 1: CCW = increasing counts
 */
constexpr uint16_t CountingDirection            = 0x6100;

/**
 * @brief Signal Period (0x6101)
 * 
 * For incremental encoders: Period of one signal in nm.
 * For sine/cosine encoders: Signal period.
 */
constexpr uint16_t SignalPeriod                 = 0x6101;

/**
 * @brief Code Sequence (0x6102)
 * 
 * For Gray code encoders: Code sequence information.
 */
constexpr uint16_t CodeSequence                 = 0x6102;

/**
 * @brief Offset Value (0x6103)
 * 
 * Offset added to all position readings.
 */
constexpr uint16_t OffsetValue                  = 0x6103;

/**
 * @brief Filter Settings (0x6104)
 * 
 * Position filter/averaging settings.
 */
constexpr uint16_t FilterSettings               = 0x6104;

/**
 * @brief Interpolation Factor (0x6105)
 * 
 * Interpolation factor for sin/cos encoders.
 * Multiplies the basic resolution.
 */
constexpr uint16_t InterpolationFactor          = 0x6105;

// ============================================================================
// Scaling Function (0x6400 - 0x64FF)
// ============================================================================

/**
 * @brief Scaling Function Numerator (0x6400)
 * 
 * Numerator for position scaling:
 * Scaled Position = (Raw Position × Numerator) / Denominator + Offset
 */
constexpr uint16_t ScalingNumerator             = 0x6400;

/**
 * @brief Scaling Function Denominator (0x6401)
 */
constexpr uint16_t ScalingDenominator           = 0x6401;

/**
 * @brief Scaling Function Offset (0x6402)
 */
constexpr uint16_t ScalingOffset                = 0x6402;

/**
 * @brief Position Scaling (0x6403) - Complete record
 * 
 * Complete scaling configuration record.
 */
constexpr uint16_t PositionScaling              = 0x6403;

// ============================================================================
// Multi-turn Specific (0x6500 - 0x65FF)
// ============================================================================

/**
 * @brief Multi-turn Value (0x6500)
 * 
 * Current revolution counter for multi-turn encoders.
 */
constexpr uint16_t MultiTurnValue               = 0x6500;

/**
 * @brief Multi-turn Resolution (0x6501)
 * 
 * Number of bits for multi-turn position.
 */
constexpr uint16_t MultiTurnResolution          = 0x6501;

/**
 * @brief Multi-turn Preset (0x6502)
 * 
 * Preset value for multi-turn counter.
 */
constexpr uint16_t MultiTurnPreset              = 0x6502;

/**
 * @brief Gearing Numerator (0x6503)
 * 
 * For geared multi-turn: Numerator of gear ratio.
 */
constexpr uint16_t GearingNumerator             = 0x6503;

/**
 * @brief Gearing Denominator (0x6504)
 */
constexpr uint16_t GearingDenominator           = 0x6504;

/**
 * @brief Single-turn Value (0x6505)
 * 
 * Position within current revolution.
 */
constexpr uint16_t SingleTurnValue              = 0x6505;

// ============================================================================
// Interface Specific Objects (0x6600 - 0x66FF)
// SSI, BiSS, EnDat interfaces
// ============================================================================

/**
 * @brief Interface Type (0x6600)
 * 
 * Type of encoder interface:
 * - 0: Parallel
 * - 1: SSI
 * - 2: BiSS-C
 * - 3: BiSS-B
 * - 4: EnDat 2.1
 * - 5: EnDat 2.2
 * - 6: SinCos 1Vpp
 * - 7: TTL/RS422
 * - 8: Hiperface
 */
constexpr uint16_t InterfaceTypeObject        = 0x6600; // formerly InterfaceType (object index)

enum class EncoderInterface : uint8_t {
    Parallel    = 0,
    SSI         = 1,
    BiSSC       = 2,
    BiSSB       = 3,
    EnDat21     = 4,
    EnDat22     = 5,
    SinCos1Vpp  = 6,
    TTL_RS422   = 7,
    Hiperface   = 8,
    DRIVE_CLiQ  = 9,
    Tamagawa    = 10,
};

/**
 * @brief SSI Configuration (0x6601)
 */
constexpr uint16_t SSIConfiguration             = 0x6601;

namespace SSIConfigSub {
    constexpr uint8_t NumberOfEntries           = 0x00;
    constexpr uint8_t ClockFrequency            = 0x01;  ///< kHz
    constexpr uint8_t DataBits                  = 0x02;  ///< Total data bits
    constexpr uint8_t BinaryGray                = 0x03;  ///< 0=Binary, 1=Gray
    constexpr uint8_t MSBFirst                  = 0x04;  ///< 0=LSB first, 1=MSB first
    constexpr uint8_t MonoFlop                  = 0x05;  ///< Monoflop time in µs
    constexpr uint8_t ErrorBit                  = 0x06;  ///< Error bit position
    constexpr uint8_t WarningBit                = 0x07;  ///< Warning bit position
}

/**
 * @brief BiSS Configuration (0x6602)
 */
constexpr uint16_t BiSSConfiguration            = 0x6602;

/**
 * @brief EnDat Configuration (0x6603)
 */
constexpr uint16_t EnDatConfiguration           = 0x6603;

// ============================================================================
// Incremental Encoder Specific (0x6700 - 0x67FF)
// ============================================================================

/**
 * @brief Zero Pulse Position (0x6700)
 * 
 * Position at which the reference/zero pulse occurs.
 */
constexpr uint16_t ZeroPulsePosition            = 0x6700;

/**
 * @brief Zero Pulse Count (0x6701)
 * 
 * Number of zero pulses seen since power-on or reset.
 */
constexpr uint16_t ZeroPulseCount               = 0x6701;

/**
 * @brief Reference Position (0x6702)
 * 
 * Position at last reference event.
 */
constexpr uint16_t ReferencePosition            = 0x6702;

/**
 * @brief Limit Switch Status (0x6703)
 * 
 * For class C4 encoders: Status of limit switches.
 */
constexpr uint16_t LimitSwitchStatus            = 0x6703;

/**
 * @brief Index Gating Window (0x6704)
 * 
 * Window around index pulse for validation.
 */
constexpr uint16_t IndexGatingWindow            = 0x6704;

// ============================================================================
// SinCos Encoder Specific (0x6800 - 0x68FF)
// ============================================================================

/**
 * @brief SinCos Period (0x6800)
 * 
 * Period of sine/cosine signals in nm.
 */
constexpr uint16_t SinCosPeriod                 = 0x6800;

/**
 * @brief SinCos Amplitude (0x6801)
 * 
 * Amplitude of sine/cosine signals.
 */
constexpr uint16_t SinCosAmplitude              = 0x6801;

/**
 * @brief SinCos Offset Correction (0x6802)
 * 
 * Offset correction for sine/cosine signals.
 */
constexpr uint16_t SinCosOffsetCorrection       = 0x6802;

/**
 * @brief Interpolation Resolution (0x6803)
 * 
 * Resolution after interpolation (bits).
 */
constexpr uint16_t InterpolationResolution      = 0x6803;

// ============================================================================
// Diagnostics (0x6900 - 0x69FF)
// ============================================================================

/**
 * @brief Temperature (0x6900)
 * 
 * Internal encoder temperature in 0.1°C.
 */
constexpr uint16_t Temperature                  = 0x6900;

/**
 * @brief Supply Voltage (0x6901)
 * 
 * Measured supply voltage in mV.
 */
constexpr uint16_t SupplyVoltage                = 0x6901;

/**
 * @brief Signal Quality (0x6902)
 * 
 * Quality indicator for encoder signals (0-100%).
 */
constexpr uint16_t SignalQuality                = 0x6902;

/**
 * @brief Operating Time (0x6903)
 * 
 * Total operating time in hours.
 */
constexpr uint16_t OperatingTime                = 0x6903;

/**
 * @brief Serial Number (0x6904)
 * 
 * Encoder serial number string.
 */
constexpr uint16_t SerialNumber                 = 0x6904;

/**
 * @brief Firmware Version (0x6905)
 */
constexpr uint16_t FirmwareVersion              = 0x6905;

/**
 * @brief Hardware Version (0x6906)
 */
constexpr uint16_t HardwareVersion              = 0x6906;

// ============================================================================
// PDO Mapping Convenience
// ============================================================================

namespace PDOMapping {
    /// Common TxPDO entry for position value (32-bit)
    constexpr uint32_t PositionValue32      = 0x60040020;  // 0x6004:00, 32 bits
    
    /// Common TxPDO entry for velocity actual (32-bit)
    constexpr uint32_t VelocityActual32     = 0x60200020;  // 0x6020:00, 32 bits
    
    /// Common TxPDO entry for operating status (16-bit)
    constexpr uint32_t OperatingStatus16    = 0x60100010;  // 0x6010:00, 16 bits
    
    /// Common TxPDO entry for alarm object (16-bit)
    constexpr uint32_t AlarmObject16        = 0x60500010;  // 0x6050:00, 16 bits
    
    /// Common RxPDO entry for preset value (32-bit)
    constexpr uint32_t PresetValue32        = 0x60030020;  // 0x6003:00, 32 bits
}

} // namespace CiA406
