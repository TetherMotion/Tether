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
namespace R22 {

static constexpr uint16_t kIndex = 0x2022; // R22 Motor Gain Parameters

enum class CurrentLoopModeOptions : uint16_t {
    Standard = 0,
    Performance = 1,
};

// Register entries for R22
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CurrentLoopMode = {
    .index = kIndex,
    .subindex = 0x01,
    .name = "Current loop mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<CurrentLoopModeOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CurrentLoopResponseLevel = {
    .index = kIndex,
    .subindex = 0x02,
    .name = "Current loop response level",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MTPAFieldWeakeningSwitch = {
    .index = kIndex,
    .subindex = 0x21,
    .name = "MTPA field-weakening switch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 256,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FieldWeakeningDepth = {
    .index = kIndex,
    .subindex = 0x22,
    .name = "Field-weakening depth",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent, // 0.1%
    .min_value = 500,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FieldWeakeningProportionalGain = {
    .index = kIndex,
    .subindex = 0x23,
    .name = "Field-weakening proportional gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Frequency_Hertz,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FieldWeakeningIntegralGain = {
    .index = kIndex,
    .subindex = 0x24,
    .name = "Field-weakening integral gain",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent, // 0.1%
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CutoffFreqDAxisLPF = {
    .index = kIndex,
    .subindex = 0x25,
    .name = "Cutoff frequency of d axis current low-pass filter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Frequency_Hertz,
    .min_value = 0,
    .max_value = 16000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FieldWeakeningDAxisCurrentLimit = {
    .index = kIndex,
    .subindex = 0x26,
    .name = "Field-weakening d axis current limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1500,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent, // 0.1%
    .min_value = 0,
    .max_value = 3000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DeadZoneCompensation = {
    .index = kIndex,
    .subindex = 0x31,
    .name = "Dead zone compensation",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent, // 0.1%
    .min_value = 0,
    .max_value = 2000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &CurrentLoopMode,
    &CurrentLoopResponseLevel,
    &MTPAFieldWeakeningSwitch,
    &FieldWeakeningDepth,
    &FieldWeakeningProportionalGain,
    &FieldWeakeningIntegralGain,
    &CutoffFreqDAxisLPF,
    &FieldWeakeningDAxisCurrentLimit,
    &DeadZoneCompensation,
};

} // namespace R22
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
