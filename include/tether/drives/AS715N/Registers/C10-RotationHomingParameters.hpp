#pragma once
#include <cstddef>
#include <cstdint>
#include "tether/slave/mailbox/IMailboxHandler.hpp"
#include "tether/profiles/cia401/CiA401Defs.hpp"
#include "tether/profiles/cia404/CiA404Defs.hpp"
#include "tether/drives/AS715N/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace AS715N {
namespace C10 {

static constexpr uint16_t C10ObjectIndex = 0x2010; // Group C10 (Rotation / Homing Parameters)

// --- Option enums ---------------------------------------------------------
enum class ReferenceRunningModeOptions : uint16_t {
    Nearest = 0,
    AlwaysForward = 1,
    AlwaysReverse = 2,
    AlwaysCurrentDirection = 3,
    NotSpecified = 4,
};

// --- Registers ------------------------------------------------------------
// C10.15 - Multi-turn overflow flag (read-only)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MultiTurnOverflowFlag = {
    .index = C10ObjectIndex,
    .subindex = 0x01,
    .name = "Multi-turn overflow flag",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.16 - Reference running mode in rotation mode
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ReferenceRunningModeInRotation = {
    .index = C10ObjectIndex,
    .subindex = 0x16,
    .name = "Reference running mode in rotation mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<ReferenceRunningModeOptions>{},
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.18 - Numerator of electronic gear ratio in rotation mode
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ElectronicGearRatioNumerator = {
    .index = C10ObjectIndex,
    .subindex = 0x18,
    .name = "Numerator of electronic gear ratio in rotation mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .min_value = 1,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.19 - Denominator of electronic gear ratio in rotation mode
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ElectronicGearRatioDenominator = {
    .index = C10ObjectIndex,
    .subindex = 0x19,
    .name = "Denominator of electronic gear ratio in rotation mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .min_value = 1,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.1A - Upper limit of mechanical absolute position in rotation mode (low 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalAbsPosUpperLimitLow = {
    .index = C10ObjectIndex,
    .subindex = 0x1A,
    .name = "Upper limit of mechanical absolute position in rotation mode (low 32 bits)",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_None, // "Unit in application"
    .min_value = 0,
    .max_value = static_cast<int64_t>(0xFFFFFFFFu),
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.1C - Upper limit of mechanical absolute position in rotation mode (high 32 bits)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalAbsPosUpperLimitHigh = {
    .index = C10ObjectIndex,
    .subindex = 0x1C,
    .name = "Upper limit of mechanical absolute position in rotation mode (high 32 bits)",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_None,
    .min_value = 0,
    .max_value = static_cast<int64_t>(0xFFFFFFFFu),
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.1E - Single-turn homing absolute value offset (I32)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SingleTurnHomingAbsValueOffset = {
    .index = C10ObjectIndex,
    .subindex = 0x1E,
    .name = "Single-turn homing absolute value offset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_None, // "Unit in application"
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

// C10.30 - Torque limit of homing upon hit-and-stop (0.1%)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HomingHitAndStopTorqueLimit = {
    .index = C10ObjectIndex,
    .subindex = 0x30,
    .name = "Torque limit of homing upon hit-and-stop",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent, // 0.1% units
    .min_value = 0,
    .max_value = 3000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.31 - Speed for homing upon hit-and-stop (rpm)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HomingHitAndStopSpeed = {
    .index = C10ObjectIndex,
    .subindex = 0x31,
    .name = "Speed for homing upon hit-and-stop",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .unit = ::EtherCAT::ObjectDictionary::Unit_RPM,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// C10.32 - Number of times for homing upon hit-and-stop
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HomingHitAndStopRepeatCount = {
    .index = C10ObjectIndex,
    .subindex = 0x32,
    .name = "Number of times for homing upon hit-and-stop",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 30,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &MultiTurnOverflowFlag,
    &ReferenceRunningModeInRotation,
    &ElectronicGearRatioNumerator,
    &ElectronicGearRatioDenominator,
    &MechanicalAbsPosUpperLimitLow,
    &MechanicalAbsPosUpperLimitHigh,
    &SingleTurnHomingAbsValueOffset,
    &HomingHitAndStopTorqueLimit,
    &HomingHitAndStopSpeed,
    &HomingHitAndStopRepeatCount,
};

} // namespace C10
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
