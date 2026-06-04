/**
 * @file Axia80Registers.hpp
 * @brief ATI Axia80 Force/Torque Sensor Register Definitions
 *
 * Object Dictionary indices, control bit masks, and enumerations
 * extracted from the ATI Axia EtherCAT ESI and reference driver.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

// ObjectDictionaryEntry-based register definitions (AS715N-style)
#include "tether/sensors/Axia80/Registers/Common.hpp"
#include "tether/sensors/Axia80/Registers/Identity.hpp"
#include "tether/sensors/Axia80/Registers/ATIConfiguration.hpp"
#include "tether/sensors/Axia80/Registers/ATIDiagnostics.hpp"
#include "tether/sensors/Axia80/Registers/ProcessData.hpp"
#include "tether/sensors/Axia80/Registers/Control.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {

// ============================================================================
// Vendor / Product Identification
// ============================================================================

static constexpr uint32_t kVendorId = 0x00000732u;
static constexpr uint32_t kProductCode = 0x26483053u;
static constexpr uint32_t kDeviceType = 0x00000192u; ///< Default device type (402 decimal)

// ============================================================================
// Object Dictionary Indices
// ============================================================================

static constexpr uint16_t OD_DEVICE_TYPE = 0x1000;
static constexpr uint16_t OD_ERROR_REGISTER = 0x1001;
static constexpr uint16_t OD_DEVICE_NAME = 0x1008;
static constexpr uint16_t OD_HARDWARE_VERSION = 0x1009;
static constexpr uint16_t OD_SOFTWARE_VERSION = 0x100A;
static constexpr uint16_t OD_IDENTITY = 0x1018;
static constexpr uint16_t OD_ERROR_SETTINGS = 0x10F1;

static constexpr uint16_t OD_PRODUCT_DESCRIPTION = 0x2019;
static constexpr uint16_t OD_TOOL_TRANSFORM = 0x2020;
static constexpr uint16_t OD_CALIBRATION_MATRIX = 0x2021;
static constexpr uint16_t OD_MONITOR_CONDITION_1 = 0x2060;
static constexpr uint16_t OD_MONITOR_CONDITION_2 = 0x2061;
static constexpr uint16_t OD_STATUS_INFO = 0x2080;
static constexpr uint16_t OD_VERSION_INFO = 0x2090;

static constexpr uint16_t OD_READING_DATA = 0x6000;
static constexpr uint16_t OD_STATUS_CODE = 0x6010;
static constexpr uint16_t OD_SAMPLE_COUNTER = 0x6020;
static constexpr uint16_t OD_GPIO_INPUTS = 0x6001;
static constexpr uint16_t OD_RAW_GAUGE_DATA = 0x6030;

static constexpr uint16_t OD_CONTROL_CODES = 0x7010;

// ============================================================================
// Product Description Sub-indices (0x2019)
// ============================================================================

static constexpr uint8_t PROD_SUBIDX_VENDOR_ID = 1;
static constexpr uint8_t PROD_SUBIDX_PRODUCT_CODE = 2;
static constexpr uint8_t PROD_SUBIDX_PRODUCT_NAME = 3;
static constexpr uint8_t PROD_SUBIDX_PRODUCT_REVISION = 4;
static constexpr uint8_t PROD_SUBIDX_PRODUCT_SERIAL = 5;
static constexpr uint8_t PROD_SUBIDX_MANUFACTURER = 6;
static constexpr uint8_t PROD_SUBIDX_COMMIT = 7; ///< Write password to commit changes

// ============================================================================
// Tool Transform Sub-indices (0x2020)
// ============================================================================

static constexpr uint8_t TT_SUBIDX_DX = 1;
static constexpr uint8_t TT_SUBIDX_DY = 2;
static constexpr uint8_t TT_SUBIDX_DZ = 3;
static constexpr uint8_t TT_SUBIDX_RX = 4;
static constexpr uint8_t TT_SUBIDX_RY = 5;
static constexpr uint8_t TT_SUBIDX_RZ = 6;
static constexpr uint8_t TT_SUBIDX_DIST_UNITS = 7;
static constexpr uint8_t TT_SUBIDX_ANG_UNITS = 8;
static constexpr uint8_t TT_SUBIDX_COMMIT = 9; ///< Write 123 to commit and activate

static constexpr uint16_t OD_RXPDO_MAP = 0x1601;
static constexpr uint16_t OD_TXPDO_MAP = 0x1A00;
static constexpr uint16_t OD_SYNC_MANAGER_TYPE = 0x1C00;
static constexpr uint16_t OD_SM2_ASSIGNMENT = 0x1C12;
static constexpr uint16_t OD_SM3_ASSIGNMENT = 0x1C13;
static constexpr uint16_t OD_SM_OUTPUT_PARAM = 0x1C32;
static constexpr uint16_t OD_SM_INPUT_PARAM = 0x1C33;

// ============================================================================
// Calibration Matrix Sub-indices (0x2021)
// ============================================================================

static constexpr uint8_t CAL_SUBIDX_FT_SERIAL = 1;
static constexpr uint8_t CAL_SUBIDX_PART_NUMBER = 2;
static constexpr uint8_t CAL_SUBIDX_FAMILY = 3;
static constexpr uint8_t CAL_SUBIDX_TIME = 4;
static constexpr uint8_t CAL_SUBIDX_MATRIX_FX_G0 = 5;
static constexpr uint8_t CAL_SUBIDX_MATRIX_FX_G6 = 11;
static constexpr uint8_t CAL_SUBIDX_MATRIX_FY_G0 = 12;
static constexpr uint8_t CAL_SUBIDX_MATRIX_FY_G6 = 18;
static constexpr uint8_t CAL_SUBIDX_MATRIX_FZ_G0 = 19;
static constexpr uint8_t CAL_SUBIDX_MATRIX_FZ_G6 = 25;
static constexpr uint8_t CAL_SUBIDX_MATRIX_TX_G0 = 26;
static constexpr uint8_t CAL_SUBIDX_MATRIX_TX_G6 = 32;
static constexpr uint8_t CAL_SUBIDX_MATRIX_TY_G0 = 33;
static constexpr uint8_t CAL_SUBIDX_MATRIX_TY_G6 = 39;
static constexpr uint8_t CAL_SUBIDX_MATRIX_TZ_G0 = 40;
static constexpr uint8_t CAL_SUBIDX_MATRIX_TZ_G6 = 46;
static constexpr uint8_t CAL_SUBIDX_FORCE_UNITS = 47;
static constexpr uint8_t CAL_SUBIDX_TORQUE_UNITS = 48;
static constexpr uint8_t CAL_SUBIDX_MAX_FX_COUNTS = 49;
static constexpr uint8_t CAL_SUBIDX_MAX_FY_COUNTS = 50;
static constexpr uint8_t CAL_SUBIDX_MAX_FZ_COUNTS = 51;
static constexpr uint8_t CAL_SUBIDX_MAX_TX_COUNTS = 52;
static constexpr uint8_t CAL_SUBIDX_MAX_TY_COUNTS = 53;
static constexpr uint8_t CAL_SUBIDX_MAX_TZ_COUNTS = 54;
static constexpr uint8_t CAL_SUBIDX_COUNTS_PER_FORCE = 55;
static constexpr uint8_t CAL_SUBIDX_COUNTS_PER_TORQUE = 56;
static constexpr uint8_t CAL_SUBIDX_GAIN_G0 = 57;
static constexpr uint8_t CAL_SUBIDX_GAIN_G7 = 64;
static constexpr uint8_t CAL_SUBIDX_FX_BIAS_DEFAULT = 65;
static constexpr uint8_t CAL_SUBIDX_TZ_BIAS_DEFAULT = 70;
static constexpr uint8_t CAL_SUBIDX_HWOFFSET_G0 = 71;
static constexpr uint8_t CAL_SUBIDX_HWOFFSET_G6 = 77;

// ============================================================================
// Control Register 1 Bit Definitions (0x7010.1)
// ============================================================================

static constexpr uint32_t CTRL_BIAS_BIT = 0x00000001u;      ///< Bit 0: Set bias
static constexpr uint32_t CTRL_RESERVED_BIT1 = 0x00000002u; ///< Bit 1: Reserved
static constexpr uint32_t CTRL_CLEAR_BIAS_BIT = 0x00000004u;///< Bit 2: Clear bias
static constexpr uint32_t CTRL_RESERVED_BIT3 = 0x00000008u; ///< Bit 3: Reserved
static constexpr uint32_t CTRL_FILTER_MASK = 0x000000F0u;    ///< Bits 4-7: Filter type
static constexpr uint32_t CTRL_FILTER_SHIFT = 4;
static constexpr uint32_t CTRL_CALIBRATION_MASK = 0x00000F00u; ///< Bits 8-11: Calibration slot
static constexpr uint32_t CTRL_CALIBRATION_SHIFT = 8;
static constexpr uint32_t CTRL_SAMPLE_RATE_MASK = 0x0000F000u; ///< Bits 12-15: Sample rate
static constexpr uint32_t CTRL_SAMPLE_RATE_SHIFT = 12;

// ============================================================================
// Control Register 2 Bit Definitions (0x7010.2)
// ============================================================================

static constexpr uint32_t CTRL2_SIMULATED_ERROR_BIT = 0x80000000u; ///< Bit 31: Inject simulated error into 0x6010 bit 28

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief Low-pass filter types for signal conditioning
 */
enum class FilterType : uint8_t {
    NO_FILTER = 0, ///< No filtering
    FILTER_1 = 1,  ///< Light filtering
    FILTER_2 = 2,  ///< Light-medium filtering
    FILTER_3 = 3,  ///< Medium filtering
    FILTER_4 = 4,  ///< Medium-heavy filtering
    FILTER_5 = 5,  ///< Heavy filtering
    FILTER_6 = 6,  ///< Very heavy filtering
    FILTER_7 = 7,  ///< Extremely heavy filtering
    FILTER_8 = 8,  ///< Maximum filtering
};

/**
 * @brief Calibration slot selection
 */
enum class CalibrationSlot : uint8_t {
    SLOT_0 = 0, ///< Calibration stored in slot 0
    SLOT_1 = 1, ///< Calibration stored in slot 1
};

/**
 * @brief Sample rate options
 */
enum class SampleRate : uint8_t {
    RATE_488_HZ = 0,  ///< 488 Hz
    RATE_976_HZ = 1,  ///< 976 Hz
    RATE_1953_HZ = 2, ///< 1953 Hz
    RATE_3906_HZ = 3, ///< 3906 Hz
    RATE_7812_HZ = 4, ///< 7812 Hz
};

/**
 * @brief Force measurement units
 */
enum class ForceUnits : uint8_t {
    LBF = 0,   ///< Pound-force
    N = 1,     ///< Newtons
    KLBF = 2,  ///< Kilo pound-force
    KN = 3,    ///< Kilonewtons
    KG = 4,    ///< Kilograms-force
};

/**
 * @brief Torque measurement units
 */
enum class TorqueUnits : uint8_t {
    LBF_IN = 0, ///< Pound-force inches
    LBF_FT = 1, ///< Pound-force feet
    NM = 2,     ///< Newton-meters
    NMM = 3,    ///< Newton-millimeters
    KGF_CM = 4, ///< Kilogram-force centimeters
    KNM = 5,    ///< Kilonewton-meters
};

/**
 * @brief Distance units for tool transform
 */
enum class DistanceUnits : uint8_t {
    INCHES = 0,
    FEET = 1,
    MILLIMETERS = 2,
    CENTIMETERS = 3,
    METERS = 4,
};

/**
 * @brief Angle units for tool transform
 */
enum class AngleUnits : uint8_t {
    DEGREES = 0,
    RADIANS = 1,
};

// ============================================================================
// Status Code Bit Definitions (0x6010)
// ============================================================================

static constexpr uint32_t STATUS_NONE = 0x00000000u;
static constexpr uint32_t STATUS_ERROR = 0x80000000u;              ///< General error (bit 31)
static constexpr uint32_t STATUS_TEMP_OUT_OF_RANGE = 0x00000001u;  ///< Bit 0
static constexpr uint32_t STATUS_VOLTAGE_OUT_OF_RANGE = 0x00000002u; ///< Bit 1
static constexpr uint32_t STATUS_BROKEN_GAGE = 0x00000004u;      ///< Bit 2
static constexpr uint32_t STATUS_BUSY = 0x00000008u;               ///< Bit 3
static constexpr uint32_t STATUS_HW_STACK_ERROR = 0x00000020u;     ///< Bit 5
static constexpr uint32_t STATUS_GAGE_OUT_OF_RANGE_WARN = 0x04000000u; ///< Bit 26
static constexpr uint32_t STATUS_GAGE_OUT_OF_RANGE = 0x08000000u;  ///< Bit 27
static constexpr uint32_t STATUS_SIMULATED_ERROR = 0x10000000u;    ///< Bit 28
static constexpr uint32_t STATUS_CAL_CHECKSUM_ERROR = 0x20000000u; ///< Bit 29
static constexpr uint32_t STATUS_SENSING_RANGE_EXCEEDED = 0x40000000u; ///< Bit 30

// ============================================================================
// Status Message Error Priority Hierarchy (from manual section 2.4)
// ============================================================================

enum class StatusMessagePriority : uint8_t {
    SUPPLY_VOLTAGE_OUT_OF_RANGE = 1,   ///< Input voltage < 12V or > 30V
    GAGE_TEMPERATURE_OUT_OF_RANGE = 2, ///< Temperature outside -5°C to +70°C
    ERROR_UNSPECIFIED = 3,             ///< General unmapped system flags
    GAGES_DISCONNECTED = 4,              ///< Open-circuit on strain gages
    GAGES_OUT_OF_RANGE = 5,            ///< Operational saturation on gage elements
    FT_OUT_OF_RANGE = 6,               ///< Combined loading exceeds 105% threshold
    COMMON_ERROR = 7,                  ///< Internal system processing bus fault
    SIMULATED_ERROR_MSG = 8,           ///< Verification test sequence flag enabled
    SPARE = 9,                         ///< Unused priority level
    NO_ERRORS = 10,                    ///< Nominal healthy operations
};

inline const char* statusPriorityToString(StatusMessagePriority p) {
    switch (p) {
        case StatusMessagePriority::SUPPLY_VOLTAGE_OUT_OF_RANGE:   return "Supply voltage out of range";
        case StatusMessagePriority::GAGE_TEMPERATURE_OUT_OF_RANGE: return "Gage temperature out of range";
        case StatusMessagePriority::ERROR_UNSPECIFIED:             return "Error (unspecified)";
        case StatusMessagePriority::GAGES_DISCONNECTED:            return "Gage(s) disconnected";
        case StatusMessagePriority::GAGES_OUT_OF_RANGE:            return "Gage(s) out-of-range";
        case StatusMessagePriority::FT_OUT_OF_RANGE:               return "F/T out of range";
        case StatusMessagePriority::COMMON_ERROR:                  return "Common error";
        case StatusMessagePriority::SIMULATED_ERROR_MSG:           return "Simulated error";
        case StatusMessagePriority::SPARE:                         return "Spare";
        case StatusMessagePriority::NO_ERRORS:                     return "No status code errors";
        default:                                                   return "Unknown";
    }
}

// ============================================================================
// Unit Conversion Helpers
// ============================================================================

inline const char* forceUnitsToString(ForceUnits unit) {
    switch (unit) {
        case ForceUnits::LBF:   return "lbf";
        case ForceUnits::N:     return "N";
        case ForceUnits::KLBF:  return "klbf";
        case ForceUnits::KN:    return "kN";
        case ForceUnits::KG:    return "kg";
        default:                return "unknown";
    }
}

inline const char* torqueUnitsToString(TorqueUnits unit) {
    switch (unit) {
        case TorqueUnits::LBF_IN: return "lbf-in";
        case TorqueUnits::LBF_FT: return "lbf-ft";
        case TorqueUnits::NM:     return "Nm";
        case TorqueUnits::NMM:    return "Nmm";
        case TorqueUnits::KGF_CM: return "kgf-cm";
        case TorqueUnits::KNM:    return "kNm";
        default:                  return "unknown";
    }
}

inline const char* sampleRateToString(SampleRate rate) {
    switch (rate) {
        case SampleRate::RATE_488_HZ:  return "488 Hz";
        case SampleRate::RATE_976_HZ:  return "976 Hz";
        case SampleRate::RATE_1953_HZ: return "1953 Hz";
        case SampleRate::RATE_3906_HZ: return "3906 Hz";
        case SampleRate::RATE_7812_HZ: return "7812 Hz";
        default:                       return "unknown";
    }
}

// ============================================================================
// Sensing Range Exceeded Threshold Verification (manual section 4)
// ============================================================================

/**
 * @brief Check if combined loading exceeds the 105% operational envelope.
 *
 * Bit 30 in the Status Code (0x6010) asserts when loading conditions surpass
 * 105% of the calibrated range. This helper computes the two multi-axis
 * threshold expressions from the Axia80 manual.
 *
 * @param fx,fy,fz,tx,ty,tz  Current force/torque values in counts
 * @param max_counts          6-element array of max calibrated counts [Fx,Fy,Fz,Tx,Ty,Tz]
 * @return true if either combined-load expression exceeds 1.05 (105%)
 */
inline bool isSensingRangeExceeded(int32_t fx, int32_t fy, int32_t fz,
                                      int32_t tx, int32_t ty, int32_t tz,
                                      const int32_t max_counts[6])
{
    if (max_counts[0] == 0 || max_counts[1] == 0 || max_counts[2] == 0 ||
        max_counts[3] == 0 || max_counts[4] == 0 || max_counts[5] == 0) {
        return false; // avoid division by zero
    }

    // Combined XY Force & Z Torque limit
    double f_xy_range = static_cast<double>(
        (std::abs(max_counts[0]) + std::abs(max_counts[1])) / 2);
    double t_z_range  = static_cast<double>(max_counts[5]);
    double expr1 = 0.0;
    if (f_xy_range > 0.0) {
        expr1 = std::sqrt(static_cast<double>(fx) * fx + static_cast<double>(fy) * fy) / f_xy_range;
    }
    if (t_z_range > 0.0) {
        expr1 += static_cast<double>(std::abs(tz)) / t_z_range;
    }

    // Combined Z Force & XY Torque limit
    double f_z_range  = static_cast<double>(max_counts[2]);
    double t_xy_range = static_cast<double>(
        (std::abs(max_counts[3]) + std::abs(max_counts[4])) / 2);
    double expr2 = 0.0;
    if (f_z_range > 0.0) {
        expr2 = static_cast<double>(std::abs(fz)) / f_z_range;
    }
    if (t_xy_range > 0.0) {
        expr2 += std::sqrt(static_cast<double>(tx) * tx + static_cast<double>(ty) * ty) / t_xy_range;
    }

    return (expr1 > 1.05) || (expr2 > 1.05);
}

// ============================================================================
// Calibration Matrix Structure
// ============================================================================

/**
 * @brief Calibration data read from OD 0x2021
 */
struct CalibrationData {
    char ft_serial[9] = {};        ///< Serial number (8 chars + null)
    char part_number[31] = {};     ///< Part number (30 chars + null)
    char family[9] = {};           ///< Family (8 chars + null)
    char cal_time[31] = {};        ///< Calibration timestamp (30 chars + null)
    double matrix[6][7] = {};      ///< 6×7 calibration matrix (Fx..Tz × G0..G6)
    ForceUnits force_units = ForceUnits::N;
    TorqueUnits torque_units = TorqueUnits::NM;
    int32_t max_counts[6] = {};    ///< Max counts per axis
    uint32_t counts_per_force = 0;
    uint32_t counts_per_torque = 0;
    uint16_t gains[8] = {};        ///< G0..G7
    int32_t bias_defaults[6] = {}; ///< Default bias values
    int32_t hw_offsets[7] = {};    ///< HW offsets G0..G6
};

/**
 * @brief Status info read from OD 0x2080
 */
struct StatusInfo {
    uint16_t supply_voltage_x10 = 0; ///< Supply voltage × 10 (decivolts)
    int16_t temperature_x10 = 0;     ///< Temperature × 10 (decidegrees C)
    char status_message[41] = {};    ///< Status message (40 chars + null)
};

/**
 * @brief Version info read from OD 0x2090
 */
struct VersionInfo {
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t revision = 0;
    uint32_t bootloader_version = 0;
    uint16_t sensor_hw_version = 0;
    uint16_t sensor_instrument = 0;
};

/**
 * @brief Product description read from OD 0x2019
 */
struct ProductDescription {
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    char product_name[33] = {};       ///< 32 chars + null
    uint32_t product_revision = 0;
    uint32_t product_serial_number = 0;
    char manufacturer[33] = {};       ///< 32 chars + null
};

/**
 * @brief Tool transform read from OD 0x2020
 *
 * Dx/Dy/Dz/Rx/Ry/Rz are stored as text-form floating-point strings on the
 * device. Use std::strtod() when reading them via SDO.
 * Write 123 to subindex 9 (Commit) to activate changes.
 */
struct ToolTransform {
    char dx_str[17] = {};     ///< Displacement X as string (16 chars max)
    char dy_str[17] = {};     ///< Displacement Y as string (16 chars max)
    char dz_str[17] = {};     ///< Displacement Z as string (16 chars max)
    char rx_str[17] = {};     ///< Rotation X as string (16 chars max)
    char ry_str[17] = {};     ///< Rotation Y as string (16 chars max)
    char rz_str[17] = {};     ///< Rotation Z as string (16 chars max)
    DistanceUnits distance_units = DistanceUnits::MILLIMETERS;
    AngleUnits angle_units = AngleUnits::DEGREES;
};

// ============================================================================
// Aggregated Register Lists (AS715N-style)
// ============================================================================

using RegisterList = ::EtherCAT::Sensors::Axia80::Registers::RegisterList;
using RegisterListOfLists = ::EtherCAT::Sensors::Axia80::Registers::RegisterListOfLists;
using RegisterListPtr = ::EtherCAT::Sensors::Axia80::Registers::RegisterListPtr;

inline const RegisterListOfLists kAllRegisterLists = {
    &Registers::Identity::kRegisterList,
    &Registers::ATIConfiguration::kRegisterList,
    &Registers::ATIDiagnostics::kRegisterList,
    &Registers::ProcessData::kRegisterList,
    &Registers::Control::kRegisterList,
};

} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT
