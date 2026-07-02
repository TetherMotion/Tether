#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace DO {

// 0x7000 - Digital Output data (RxPDO mapped, slot-dependent)
static constexpr uint16_t kDataIndex = 0x7000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex, .subindex = 0x00,
    .name = "Digital Output Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 2,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of DO entries (1 for 8-bit, 2 for 16-bit)",
};

constexpr RegisterEntry OutputCH0 = {
    .index = kDataIndex, .subindex = 0x01,
    .name = "Digital Output CH0-8bit",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital output channel 0 (bits 0-7)",
};

constexpr RegisterEntry OutputCH1 = {
    .index = kDataIndex, .subindex = 0x02,
    .name = "Digital Output CH1-8bit",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital output channel 1 (bits 8-15)",
};

// 0x8000 - DO configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

constexpr RegisterEntry StopModeCH0 = {
    .index = kConfigIndex, .subindex = 0x01,
    .name = "DO CH0 Stopmode After EtherCAT Lost Link",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Stop mode for DO CH0 after EtherCAT link loss",
};

constexpr RegisterEntry StopValueCH0 = {
    .index = kConfigIndex, .subindex = 0x02,
    .name = "DO CH0 Output Value After EtherCAT Lost Link",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output value for DO CH0 after EtherCAT link loss",
};

constexpr RegisterEntry StopModeCH1 = {
    .index = kConfigIndex, .subindex = 0x03,
    .name = "DO CH1 Stopmode After EtherCAT Lost Link",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Stop mode for DO CH1 after EtherCAT link loss",
};

constexpr RegisterEntry StopValueCH1 = {
    .index = kConfigIndex, .subindex = 0x04,
    .name = "DO CH1 Output Value After EtherCAT Lost Link",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Output value for DO CH1 after EtherCAT link loss",
};

static const RegisterList kRegisterList = {
    &DataCount, &OutputCH0, &OutputCH1,
    &StopModeCH0, &StopValueCH0,
    &StopModeCH1, &StopValueCH1,
};

} // namespace DO
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
