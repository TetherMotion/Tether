#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace RD {

// 0x7000 - RTD Input data (TxPDO mapped, slot-dependent)
static constexpr uint16_t kDataIndex = 0x7000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex, .subindex = 0x00,
    .name = "RTD Input Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 4, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 4,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of RTD entries",
};

#define RP20_RD_INPUT_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kDataIndex, .subindex = (CH), \
        .name = "RTD Input CH" #CH, \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16, \
        .default_value = 0, .unit = Unit_Temperature_C, .options_enum = nullptr, \
        .min_value = -32768, .max_value = 32767, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "RTD input channel " #CH, \
    }

RP20_RD_INPUT_REG(InputCH0, 0x01);
RP20_RD_INPUT_REG(InputCH1, 0x02);
RP20_RD_INPUT_REG(InputCH2, 0x03);
RP20_RD_INPUT_REG(InputCH3, 0x04);
#undef RP20_RD_INPUT_REG

// 0x8000 - RD configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

#define RP20_RD_SIGNAL_FORM_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "RD CH" #CH " Signal Form", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = std::type_identity<RTDSignalForm>{}, \
        .min_value = 0, .max_value = 5, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "RTD type for channel " #CH, \
    }

RP20_RD_SIGNAL_FORM_REG(SignalFormCH0, 0x01);
RP20_RD_SIGNAL_FORM_REG(SignalFormCH1, 0x02);
RP20_RD_SIGNAL_FORM_REG(SignalFormCH2, 0x03);
RP20_RD_SIGNAL_FORM_REG(SignalFormCH3, 0x04);
#undef RP20_RD_SIGNAL_FORM_REG

#define RP20_RD_FILTER_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "RD CH" #CH " Filtering Mode", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 1, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 1, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Filtering mode for RTD channel " #CH, \
    }

RP20_RD_FILTER_REG(FilteringModeCH0, 0x05);
RP20_RD_FILTER_REG(FilteringModeCH1, 0x06);
RP20_RD_FILTER_REG(FilteringModeCH2, 0x07);
RP20_RD_FILTER_REG(FilteringModeCH3, 0x08);
#undef RP20_RD_FILTER_REG

static const RegisterList kRegisterList = {
    &DataCount, &InputCH0, &InputCH1, &InputCH2, &InputCH3,
    &SignalFormCH0, &SignalFormCH1, &SignalFormCH2, &SignalFormCH3,
    &FilteringModeCH0, &FilteringModeCH1, &FilteringModeCH2, &FilteringModeCH3,
};

} // namespace RD
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
