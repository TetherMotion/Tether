#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "tether/ethercat/ObjectDictionary.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {

using RegisterEntry = ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry;
using RegisterEntryPtr = const RegisterEntry*;
using RegisterList = std::vector<RegisterEntryPtr>;
using RegisterListPtr = const RegisterList*;
using RegisterListOfLists = std::vector<RegisterListPtr>;

using OptionsType = ::EtherCAT::ObjectDictionary::OptionsType;
using Unit = ::EtherCAT::ObjectDictionary::Unit;
using ModificationMode = ::EtherCAT::ObjectDictionary::ModificationMode;
using EffectiveTime = ::EtherCAT::ObjectDictionary::EffectiveTime;

static constexpr Unit Unit_None           = ::EtherCAT::ObjectDictionary::Unit_None;
static constexpr Unit Unit_Voltage_Volt   = ::EtherCAT::ObjectDictionary::Unit_Voltage_Volt;
static constexpr Unit Unit_Current_Ampere = ::EtherCAT::ObjectDictionary::Unit_Current_Ampere;
static constexpr Unit Unit_Temperature_C  = ::EtherCAT::ObjectDictionary::Unit_Temperature_C;
static constexpr Unit Unit_Resistance_Ohm = ::EtherCAT::ObjectDictionary::Unit_Resistance_Ohm;

// ---------------------------------------------------------------------------
// OD index base addresses (slot-dependent, increment = 0x10)
// ---------------------------------------------------------------------------
static constexpr uint16_t kInputDataBaseIndex   = 0x6000;
static constexpr uint16_t kOutputDataBaseIndex  = 0x7000;
static constexpr uint16_t kConfigBaseIndex      = 0x8000;
static constexpr uint16_t kDiagnosisBaseIndex   = 0xA000;
static constexpr uint16_t kSlotIndexIncrement   = 0x10;

// PDO index bases (slot-dependent, increment = 1)
static constexpr uint16_t kRxPDOBaseIndex       = 0x1600;
static constexpr uint16_t kTxPDOBaseIndex       = 0x1A00;
static constexpr uint16_t kSlotPDOIncrement     = 1;

// ---------------------------------------------------------------------------
// Enums for RP20 module configuration
// ---------------------------------------------------------------------------

// AI signal form (DT0802EN32 for AI)
enum class AISignalForm : uint8_t {
    Current_4_20mA  = 0,  // 4mA~20mA (4000~20000)
    Current_20mA    = 1,  // -20mA~20mA (-20000~20000) [AI] / 0mA~20mA (0~20000) [AO]
    Voltage_1_5V    = 2,  // 1V~5V (1000~5000)
    Voltage_10V     = 3,  // -10V~+10V (-10000~10000)
};

// RTD signal form (DT0802EN32 for RD)
enum class RTDSignalForm : uint8_t {
    PT100   = 0,
    PT1000  = 1,
    Cu50    = 4,
    Cu100   = 5,
};

// Thermocouple signal form (DT0802EN32 for TC)
enum class TCSignalForm : uint8_t {
    Type_J     = 0,
    Type_K     = 1,
    Type_E     = 2,
    Type_S     = 3,
    Type_T     = 4,
    Voltage_100mV = 5,
};

// Filtering mode (DT0802EN33 / DT0803EN33)
enum class FilteringMode : uint8_t {
    None      = 0,
    Average   = 1,
};

// Stop mode (DT0802EN34 for AO)
enum class StopMode : uint8_t {
    KeepCurrentValue = 0,
    RetainPreset     = 255,
};

// Cold junction compensation (DT0804EN34 for TC)
enum class ColdJunctionCompensation : uint8_t {
    Internal = 0,
    External = 1,
};

} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
