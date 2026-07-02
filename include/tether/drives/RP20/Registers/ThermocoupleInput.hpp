#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace TC {

// 0x7000 - Thermocouple Input data (TxPDO mapped, slot-dependent)
static constexpr uint16_t kDataIndex = 0x7000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex, .subindex = 0x00,
    .name = "Thermocouple Input Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 4, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 4,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of TC entries",
};

#define RP20_TC_INPUT_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kDataIndex, .subindex = (CH), \
        .name = "Thermocouple Input CH" #CH, \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16, \
        .default_value = 0, .unit = Unit_Temperature_C, .options_enum = nullptr, \
        .min_value = -32768, .max_value = 32767, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Thermocouple input channel " #CH, \
    }

RP20_TC_INPUT_REG(InputCH0, 0x01);
RP20_TC_INPUT_REG(InputCH1, 0x02);
RP20_TC_INPUT_REG(InputCH2, 0x03);
RP20_TC_INPUT_REG(InputCH3, 0x04);
#undef RP20_TC_INPUT_REG

// 0x8000 - TC configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

#define RP20_TC_SIGNAL_FORM_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "TC CH" #CH " Signal Form", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 5, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Thermocouple type for channel " #CH, \
    }

RP20_TC_SIGNAL_FORM_REG(SignalFormCH0, 0x01);
RP20_TC_SIGNAL_FORM_REG(SignalFormCH1, 0x02);
RP20_TC_SIGNAL_FORM_REG(SignalFormCH2, 0x03);
RP20_TC_SIGNAL_FORM_REG(SignalFormCH3, 0x04);
#undef RP20_TC_SIGNAL_FORM_REG

#define RP20_TC_FILTER_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "TC CH" #CH " Filtering Mode", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 1, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 1, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Filtering mode for TC channel " #CH, \
    }

RP20_TC_FILTER_REG(FilteringModeCH0, 0x05);
RP20_TC_FILTER_REG(FilteringModeCH1, 0x06);
RP20_TC_FILTER_REG(FilteringModeCH2, 0x07);
RP20_TC_FILTER_REG(FilteringModeCH3, 0x08);
#undef RP20_TC_FILTER_REG

#define RP20_TC_CJC_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "TC CH" #CH " Cold Junction Compensation Mode", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 1, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Cold junction compensation mode for TC channel " #CH, \
    }

RP20_TC_CJC_REG(ColdJunctionCH0, 0x09);
RP20_TC_CJC_REG(ColdJunctionCH1, 0x0A);
RP20_TC_CJC_REG(ColdJunctionCH2, 0x0B);
RP20_TC_CJC_REG(ColdJunctionCH3, 0x0C);
#undef RP20_TC_CJC_REG

static const RegisterList kRegisterList = {
    &DataCount, &InputCH0, &InputCH1, &InputCH2, &InputCH3,
    &SignalFormCH0, &SignalFormCH1, &SignalFormCH2, &SignalFormCH3,
    &FilteringModeCH0, &FilteringModeCH1, &FilteringModeCH2, &FilteringModeCH3,
    &ColdJunctionCH0, &ColdJunctionCH1, &ColdJunctionCH2, &ColdJunctionCH3,
};

} // namespace TC
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
