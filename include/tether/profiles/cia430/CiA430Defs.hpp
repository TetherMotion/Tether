/**
 * @file CiA430Defs.hpp
 * @brief CiA 430 Energy Metering Device Profile Object Dictionary
 *
 * Defines object dictionary entries for CiA 430 compliant energy meters
 * including power, energy, and power quality measurements.
 *
 * Features:
 * - Active/reactive/apparent power measurement
 * - Energy accumulation (import/export)
 * - Voltage/current measurement per phase
 * - Power factor and frequency
 * - Harmonic analysis (THD)
 * - Demand metering
 * - Tariff support
 */

#pragma once

#include <cstdint>

namespace CiA430 {

// ============================================================================
// Profile Identification
// ============================================================================

constexpr uint16_t PROFILE_NUMBER = 430;
constexpr uint16_t PROFILE_VERSION = 0x0100;

// ============================================================================
// Object Dictionary Indices
// ============================================================================

namespace Object {

// Device Information (0x6000-0x600F)
constexpr uint16_t MeterType          = 0x6000;
constexpr uint16_t NumberOfPhases     = 0x6001;
constexpr uint16_t RatedVoltage       = 0x6002;
constexpr uint16_t RatedCurrent       = 0x6003;
constexpr uint16_t RatedFrequency     = 0x6004;
constexpr uint16_t AccuracyClass      = 0x6005;
constexpr uint16_t CTRatio            = 0x6006;
constexpr uint16_t VTRatio            = 0x6007;

// Voltage Measurements (0x6010-0x601F)
constexpr uint16_t VoltageL1N         = 0x6010;
constexpr uint16_t VoltageL2N         = 0x6011;
constexpr uint16_t VoltageL3N         = 0x6012;
constexpr uint16_t VoltageL1L2        = 0x6013;
constexpr uint16_t VoltageL2L3        = 0x6014;
constexpr uint16_t VoltageL3L1        = 0x6015;
constexpr uint16_t VoltageAvgLN       = 0x6016;
constexpr uint16_t VoltageAvgLL       = 0x6017;

// Current Measurements (0x6020-0x602F)
constexpr uint16_t CurrentL1          = 0x6020;
constexpr uint16_t CurrentL2          = 0x6021;
constexpr uint16_t CurrentL3          = 0x6022;
constexpr uint16_t CurrentN           = 0x6023;
constexpr uint16_t CurrentAvg         = 0x6024;
constexpr uint16_t CurrentTotal       = 0x6025;

// Power Measurements (0x6030-0x604F)
constexpr uint16_t ActivePowerL1      = 0x6030;
constexpr uint16_t ActivePowerL2      = 0x6031;
constexpr uint16_t ActivePowerL3      = 0x6032;
constexpr uint16_t ActivePowerTotal   = 0x6033;
constexpr uint16_t ReactivePowerL1    = 0x6034;
constexpr uint16_t ReactivePowerL2    = 0x6035;
constexpr uint16_t ReactivePowerL3    = 0x6036;
constexpr uint16_t ReactivePowerTotal = 0x6037;
constexpr uint16_t ApparentPowerL1    = 0x6038;
constexpr uint16_t ApparentPowerL2    = 0x6039;
constexpr uint16_t ApparentPowerL3    = 0x603A;
constexpr uint16_t ApparentPowerTotal = 0x603B;

// Power Factor (0x6050-0x605F)
constexpr uint16_t PowerFactorL1      = 0x6050;
constexpr uint16_t PowerFactorL2      = 0x6051;
constexpr uint16_t PowerFactorL3      = 0x6052;
constexpr uint16_t PowerFactorTotal   = 0x6053;
constexpr uint16_t DisplacementPFL1   = 0x6054;
constexpr uint16_t DisplacementPFL2   = 0x6055;
constexpr uint16_t DisplacementPFL3   = 0x6056;

// Frequency (0x6060-0x606F)
constexpr uint16_t Frequency          = 0x6060;
constexpr uint16_t FrequencyMin       = 0x6061;
constexpr uint16_t FrequencyMax       = 0x6062;

// Energy Import (0x6070-0x607F)
constexpr uint16_t ActiveEnergyImport    = 0x6070;
constexpr uint16_t ActiveEnergyImportL1  = 0x6071;
constexpr uint16_t ActiveEnergyImportL2  = 0x6072;
constexpr uint16_t ActiveEnergyImportL3  = 0x6073;
constexpr uint16_t ReactiveEnergyImport  = 0x6074;
constexpr uint16_t ApparentEnergyImport  = 0x6075;

// Energy Export (0x6080-0x608F)
constexpr uint16_t ActiveEnergyExport    = 0x6080;
constexpr uint16_t ActiveEnergyExportL1  = 0x6081;
constexpr uint16_t ActiveEnergyExportL2  = 0x6082;
constexpr uint16_t ActiveEnergyExportL3  = 0x6083;
constexpr uint16_t ReactiveEnergyExport  = 0x6084;
constexpr uint16_t ApparentEnergyExport  = 0x6085;

// Demand (0x6090-0x609F)
constexpr uint16_t DemandActivePower     = 0x6090;
constexpr uint16_t DemandReactivePower   = 0x6091;
constexpr uint16_t DemandApparentPower   = 0x6092;
constexpr uint16_t DemandCurrent         = 0x6093;
constexpr uint16_t MaxDemandActivePower  = 0x6094;
constexpr uint16_t MaxDemandReactivePower= 0x6095;
constexpr uint16_t MaxDemandCurrent      = 0x6096;
constexpr uint16_t DemandPeriod          = 0x6097;
constexpr uint16_t DemandSubperiods      = 0x6098;

// Harmonics (0x60A0-0x60BF)
constexpr uint16_t THDV_L1            = 0x60A0;
constexpr uint16_t THDV_L2            = 0x60A1;
constexpr uint16_t THDV_L3            = 0x60A2;
constexpr uint16_t THDI_L1            = 0x60A3;
constexpr uint16_t THDI_L2            = 0x60A4;
constexpr uint16_t THDI_L3            = 0x60A5;
constexpr uint16_t HarmonicsVoltageL1 = 0x60A6;  // Array 1-50
constexpr uint16_t HarmonicsVoltageL2 = 0x60A7;
constexpr uint16_t HarmonicsVoltageL3 = 0x60A8;
constexpr uint16_t HarmonicsCurrentL1 = 0x60A9;
constexpr uint16_t HarmonicsCurrentL2 = 0x60AA;
constexpr uint16_t HarmonicsCurrentL3 = 0x60AB;

// Power Quality (0x60C0-0x60CF)
constexpr uint16_t VoltageUnbalance   = 0x60C0;
constexpr uint16_t CurrentUnbalance   = 0x60C1;
constexpr uint16_t PhaseSequence      = 0x60C2;
constexpr uint16_t SagCount           = 0x60C3;
constexpr uint16_t SwellCount         = 0x60C4;
constexpr uint16_t InterruptionCount  = 0x60C5;

// Tariff (0x60D0-0x60DF)
constexpr uint16_t ActiveTariff       = 0x60D0;
constexpr uint16_t TariffSchedule     = 0x60D1;
constexpr uint16_t EnergyTariff1      = 0x60D2;
constexpr uint16_t EnergyTariff2      = 0x60D3;
constexpr uint16_t EnergyTariff3      = 0x60D4;
constexpr uint16_t EnergyTariff4      = 0x60D5;

// Control/Status (0x60E0-0x60EF)
constexpr uint16_t Statusword         = 0x60E0;
constexpr uint16_t Controlword        = 0x60E1;
constexpr uint16_t OperatingMode      = 0x60E2;
constexpr uint16_t AlarmStatus        = 0x60E3;
constexpr uint16_t AlarmEnable        = 0x60E4;

// Alarms/Thresholds (0x60F0-0x60FF)
constexpr uint16_t VoltageHighThreshold  = 0x60F0;
constexpr uint16_t VoltageLowThreshold   = 0x60F1;
constexpr uint16_t CurrentHighThreshold  = 0x60F2;
constexpr uint16_t PowerFactorThreshold  = 0x60F3;
constexpr uint16_t FrequencyHighThreshold= 0x60F4;
constexpr uint16_t FrequencyLowThreshold = 0x60F5;
constexpr uint16_t THDThreshold          = 0x60F6;

// Diagnostics (0x6100-0x610F)
constexpr uint16_t FaultCode          = 0x6100;
constexpr uint16_t WarningCode        = 0x6101;
constexpr uint16_t OperatingHours     = 0x6102;
constexpr uint16_t LastCalibration    = 0x6103;
constexpr uint16_t Temperature        = 0x6104;

} // namespace Object

// Backwards compatibility: legacy nested namespaces and type aliases
namespace MeterIdentification {
    constexpr uint16_t MeterType = ::CiA430::Object::MeterType;
    constexpr uint16_t RatedVoltage = ::CiA430::Object::RatedVoltage;
    constexpr uint16_t RatedCurrent = ::CiA430::Object::RatedCurrent;
    constexpr uint16_t RatedFrequency = ::CiA430::Object::RatedFrequency;
    constexpr uint16_t AccuracyClass = ::CiA430::Object::AccuracyClass;
    constexpr uint16_t CTRatio = ::CiA430::Object::CTRatio;
    constexpr uint16_t VTRatio = ::CiA430::Object::VTRatio;
}

namespace Harmonics {
    constexpr uint16_t THDVoltage = ::CiA430::Object::THDV_L1;     // Base index for THD Voltage
    constexpr uint16_t HarmonicVoltage = ::CiA430::Object::HarmonicsVoltageL1; // Base for harmonic arrays
    constexpr uint16_t THDCurrent = ::CiA430::Object::THDI_L1;     // Base index for THD Current
    constexpr uint16_t HarmonicCurrent = ::CiA430::Object::HarmonicsCurrentL1;
}

namespace Demand {
    constexpr uint16_t ActivePower = ::CiA430::Object::DemandActivePower;
    constexpr uint16_t ReactivePower = ::CiA430::Object::DemandReactivePower;
    constexpr uint16_t ApparentPower = ::CiA430::Object::DemandApparentPower;
    constexpr uint16_t Current = ::CiA430::Object::DemandCurrent;
    constexpr uint16_t MaxActivePower = ::CiA430::Object::MaxDemandActivePower;
    constexpr uint16_t MaxReactivePower = ::CiA430::Object::MaxDemandReactivePower;
    constexpr uint16_t MaxCurrent = ::CiA430::Object::MaxDemandCurrent;
    constexpr uint16_t Period = ::CiA430::Object::DemandPeriod;
    constexpr uint16_t Subperiods = ::CiA430::Object::DemandSubperiods;
    constexpr uint16_t Reset = ::CiA430::Object::DemandPeriod; // Fallback alias
}

namespace Tariff {
    constexpr uint16_t ActiveTariff = ::CiA430::Object::ActiveTariff;
}

namespace Alarms {
    constexpr uint16_t VoltageHighThreshold = ::CiA430::Object::VoltageHighThreshold;
    constexpr uint16_t VoltageLowThreshold = ::CiA430::Object::VoltageLowThreshold;
    constexpr uint16_t CurrentHighThreshold = ::CiA430::Object::CurrentHighThreshold;
    constexpr uint16_t PowerFactorLowThreshold = ::CiA430::Object::PowerFactorThreshold;
    constexpr uint16_t FrequencyHighThreshold = ::CiA430::Object::FrequencyHighThreshold;
    constexpr uint16_t FrequencyLowThreshold = ::CiA430::Object::FrequencyLowThreshold;
    constexpr uint16_t THDThreshold = ::CiA430::Object::THDThreshold;
    constexpr uint16_t Enable = ::CiA430::Object::AlarmEnable;
    constexpr uint16_t Reset = ::CiA430::Object::AlarmEnable; // approximate
}

// Forward declarations for PDO structs (defined below) to allow legacy typedefs
struct TxPDO_Basic;
struct TxPDO_Extended;
struct TxPDO_Full;

// Provide PDO mapping typedefs for legacy names
using BasicPDOMapping = TxPDO_Basic;
using ExtendedPDOMapping = TxPDO_Extended;
using FullPDOMapping = TxPDO_Full;


// ============================================================================
// Statusword Bits
// ============================================================================

namespace StatuswordBits {
constexpr uint16_t Ready              = 0x0001;
constexpr uint16_t DataValid          = 0x0002;
constexpr uint16_t EnergyDirection    = 0x0004;  // 0=import, 1=export
constexpr uint16_t Calibrated         = 0x0008;
constexpr uint16_t AlarmActive        = 0x0010;
constexpr uint16_t Warning            = 0x0020;
constexpr uint16_t Fault              = 0x0040;
constexpr uint16_t PhaseL1OK          = 0x0100;
constexpr uint16_t PhaseL2OK          = 0x0200;
constexpr uint16_t PhaseL3OK          = 0x0400;
constexpr uint16_t NeutralOK          = 0x0800;
} // namespace StatuswordBits

// ============================================================================
// Controlword Bits
// ============================================================================

namespace ControlwordBits {
constexpr uint16_t Enable             = 0x0001;
constexpr uint16_t ResetEnergy        = 0x0002;
constexpr uint16_t ResetDemand        = 0x0004;
constexpr uint16_t ResetMinMax        = 0x0008;
constexpr uint16_t ResetAlarms        = 0x0010;
constexpr uint16_t FreezeEnergy       = 0x0020;
constexpr uint16_t SwitchTariff       = 0x0040;
} // namespace ControlwordBits

// ============================================================================
// Alarm Bits
// ============================================================================

namespace AlarmBits {
constexpr uint32_t VoltageHighL1      = 0x00000001;
constexpr uint32_t VoltageHighL2      = 0x00000002;
constexpr uint32_t VoltageHighL3      = 0x00000004;
constexpr uint32_t VoltageLowL1       = 0x00000008;
constexpr uint32_t VoltageLowL2       = 0x00000010;
constexpr uint32_t VoltageLowL3       = 0x00000020;
constexpr uint32_t CurrentHighL1      = 0x00000040;
constexpr uint32_t CurrentHighL2      = 0x00000080;
constexpr uint32_t CurrentHighL3      = 0x00000100;
constexpr uint32_t FrequencyHigh      = 0x00000200;
constexpr uint32_t FrequencyLow       = 0x00000400;
constexpr uint32_t PowerFactorLow     = 0x00000800;
constexpr uint32_t THDHigh            = 0x00001000;
constexpr uint32_t PhaseSequenceError = 0x00002000;
constexpr uint32_t PhaseLoss          = 0x00004000;
constexpr uint32_t NeutralLoss        = 0x00008000;
} // namespace AlarmBits

// ============================================================================
// Meter Types
// ============================================================================

namespace MeterType {
constexpr uint8_t SinglePhase         = 0x01;
constexpr uint8_t ThreePhaseWye       = 0x02;
constexpr uint8_t ThreePhaseDelta     = 0x03;
constexpr uint8_t ThreePhase4Wire     = 0x04;
constexpr uint8_t ThreePhase3Wire     = 0x05;
} // namespace MeterType

// ============================================================================
// Accuracy Classes
// ============================================================================

namespace AccuracyClass {
constexpr uint8_t Class_0_2           = 0x01;
constexpr uint8_t Class_0_5           = 0x02;
constexpr uint8_t Class_1             = 0x03;
constexpr uint8_t Class_2             = 0x04;
} // namespace AccuracyClass

// ============================================================================
// Unit Conversions
// ============================================================================

// Voltage in 0.01V
inline float rawToVolts(int32_t raw) { return raw / 100.0f; }
inline int32_t voltsToRaw(float v) { return static_cast<int32_t>(v * 100.0f); }

// Current in 0.001A (mA)
inline float rawToAmps(int32_t raw) { return raw / 1000.0f; }
inline int32_t ampsToRaw(float a) { return static_cast<int32_t>(a * 1000.0f); }

// Power in 0.1W
inline float rawToWatts(int32_t raw) { return raw / 10.0f; }
inline int32_t wattsToRaw(float w) { return static_cast<int32_t>(w * 10.0f); }

// Energy in Wh
inline float rawToKWh(int64_t raw) { return raw / 1000.0f; }
inline int64_t kWhToRaw(float kwh) { return static_cast<int64_t>(kwh * 1000.0f); }

// Power factor in 0.001
inline float rawToPF(int16_t raw) { return raw / 1000.0f; }
inline int16_t pfToRaw(float pf) { return static_cast<int16_t>(pf * 1000.0f); }

// Frequency in 0.01Hz
inline float rawToHz(uint16_t raw) { return raw / 100.0f; }
inline uint16_t hzToRaw(float hz) { return static_cast<uint16_t>(hz * 100.0f); }

// THD in 0.1%
inline float rawToTHD(uint16_t raw) { return raw / 10.0f; }

// ============================================================================
// PDO Structures
// ============================================================================

#pragma pack(push, 1)

// Basic power TxPDO
struct TxPDO_Basic {
    uint16_t statusword;
    int32_t  active_power_total;    // 0.1W
    int32_t  reactive_power_total;  // 0.1VAr
    int64_t  energy_import;         // Wh
};

// Extended TxPDO with voltage/current
struct TxPDO_Extended {
    uint16_t statusword;
    int32_t  voltage_l1n;           // 0.01V
    int32_t  voltage_l2n;
    int32_t  voltage_l3n;
    int32_t  current_l1;            // 0.001A
    int32_t  current_l2;
    int32_t  current_l3;
    int32_t  active_power_total;
    int32_t  reactive_power_total;
    int16_t  power_factor;          // 0.001
    uint16_t frequency;             // 0.01Hz
};

// Full TxPDO with all phases and energy
struct TxPDO_Full {
    uint16_t statusword;
    int32_t  voltage_l1n;
    int32_t  voltage_l2n;
    int32_t  voltage_l3n;
    int32_t  current_l1;
    int32_t  current_l2;
    int32_t  current_l3;
    int32_t  active_power_l1;
    int32_t  active_power_l2;
    int32_t  active_power_l3;
    int32_t  active_power_total;
    int32_t  reactive_power_total;
    int32_t  apparent_power_total;
    int16_t  power_factor_total;
    uint16_t frequency;
    int64_t  energy_import;
    int64_t  energy_export;
    uint32_t alarm_status;
};

// Basic control RxPDO
struct RxPDO_Basic {
    uint16_t controlword;
};

// Extended control RxPDO
struct RxPDO_Extended {
    uint16_t controlword;
    uint8_t  tariff_select;
    uint16_t demand_period;
};

#pragma pack(pop)

} // namespace CiA430
