#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace DI {

// 0x6000 - Digital Input data (TxPDO mapped, slot-dependent)
static constexpr uint16_t kDataIndex = 0x6000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex, .subindex = 0x00,
    .name = "Digital Input Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 2,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of DI entries (1 for 8-bit, 2 for 16-bit)",
};

constexpr RegisterEntry InputCH0 = {
    .index = kDataIndex, .subindex = 0x01,
    .name = "Digital Input CH0-8bit",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital input channel 0 (bits 0-7)",
};

constexpr RegisterEntry InputCH1 = {
    .index = kDataIndex, .subindex = 0x02,
    .name = "Digital Input CH1-8bit",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital input channel 1 (bits 8-15)",
};

// 0x8000 - DI configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

constexpr RegisterEntry FilterTimeCH0_Low = {
    .index = kConfigIndex, .subindex = 0x01,
    .name = "DI Filter time CH0 bit0-3",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital input filter time for CH0 bits 0-3",
};

constexpr RegisterEntry FilterTimeCH0_High = {
    .index = kConfigIndex, .subindex = 0x02,
    .name = "DI Filter time CH0 bit4-7",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital input filter time for CH0 bits 4-7",
};

constexpr RegisterEntry FilterTimeCH1_Low = {
    .index = kConfigIndex, .subindex = 0x03,
    .name = "DI Filter time CH1 bit0-3",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital input filter time for CH1 bits 0-3",
};

constexpr RegisterEntry FilterTimeCH1_High = {
    .index = kConfigIndex, .subindex = 0x04,
    .name = "DI Filter time CH1 bit4-7",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Digital input filter time for CH1 bits 4-7",
};

static const RegisterList kRegisterList = {
    &DataCount, &InputCH0, &InputCH1,
    &FilterTimeCH0_Low, &FilterTimeCH0_High,
    &FilterTimeCH1_Low, &FilterTimeCH1_High,
};

} // namespace DI
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
