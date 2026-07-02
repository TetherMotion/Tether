#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace AI {

// 0x7000 - Analog Input data (TxPDO mapped for AI-only modules, slot-dependent)
// Note: RP20 AI modules use 0x7000 for TxPDO data (vendor-specific)
// Mixed modules (0202IV) use 0x6000 for AI data instead.
static constexpr uint16_t kDataIndex_AIOnly  = 0x7000;
static constexpr uint16_t kDataIndex_Mixed   = 0x6000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex_AIOnly, .subindex = 0x00,
    .name = "Analog Input Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 4, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 4,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of AI entries",
};

#define RP20_AI_INPUT_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kDataIndex_AIOnly, .subindex = (CH), \
        .name = "Analog Input CH" #CH, \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = -32768, .max_value = 32767, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Analog input channel " #CH, \
    }

RP20_AI_INPUT_REG(InputCH0, 0x01);
RP20_AI_INPUT_REG(InputCH1, 0x02);
RP20_AI_INPUT_REG(InputCH2, 0x03);
RP20_AI_INPUT_REG(InputCH3, 0x04);
#undef RP20_AI_INPUT_REG

// 0x8000 - AI configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

#define RP20_AI_SIGNAL_FORM_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "AI CH" #CH " Signal Form", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 3, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Signal form for AI channel " #CH, \
    }

RP20_AI_SIGNAL_FORM_REG(SignalFormCH0, 0x01);
RP20_AI_SIGNAL_FORM_REG(SignalFormCH1, 0x02);
RP20_AI_SIGNAL_FORM_REG(SignalFormCH2, 0x03);
RP20_AI_SIGNAL_FORM_REG(SignalFormCH3, 0x04);
#undef RP20_AI_SIGNAL_FORM_REG

#define RP20_AI_FILTER_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "AI CH" #CH " Filtering Mode", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 1, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 1, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Filtering mode for AI channel " #CH, \
    }

RP20_AI_FILTER_REG(FilteringModeCH0, 0x05);
RP20_AI_FILTER_REG(FilteringModeCH1, 0x06);
RP20_AI_FILTER_REG(FilteringModeCH2, 0x07);
RP20_AI_FILTER_REG(FilteringModeCH3, 0x08);
#undef RP20_AI_FILTER_REG

static const RegisterList kRegisterList = {
    &DataCount, &InputCH0, &InputCH1, &InputCH2, &InputCH3,
    &SignalFormCH0, &SignalFormCH1, &SignalFormCH2, &SignalFormCH3,
    &FilteringModeCH0, &FilteringModeCH1, &FilteringModeCH2, &FilteringModeCH3,
};

} // namespace AI
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
