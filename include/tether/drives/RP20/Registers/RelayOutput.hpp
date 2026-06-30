#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace DR {

// 0x7000 - Digital Relay Output data (RxPDO mapped, slot-dependent)
static constexpr uint16_t kDataIndex = 0x7000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex, .subindex = 0x00,
    .name = "Digital Relay Output Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of DR entries",
};

constexpr RegisterEntry OutputCH0 = {
    .index = kDataIndex, .subindex = 0x01,
    .name = "Digital Relay Output CH0-8bit",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital relay output channel 0 (bits 0-7, 8 relays)",
};

// 0x8000 - DR configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

constexpr RegisterEntry StopModeCH0 = {
    .index = kConfigIndex, .subindex = 0x01,
    .name = "DR CH0 Stopmode After EtherCAT Lost Link",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Stop mode for relay CH0 after EtherCAT link loss",
};

constexpr RegisterEntry StopValueCH0 = {
    .index = kConfigIndex, .subindex = 0x02,
    .name = "DR CH0 Stopvalue After EtherCAT Lost Link",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Stop value for relay CH0 after EtherCAT link loss",
};

static const RegisterList kRegisterList = {
    &DataCount, &OutputCH0,
    &StopModeCH0, &StopValueCH0,
};

} // namespace DR
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
