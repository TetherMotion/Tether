#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace AO {

// 0x7000 - Analog Output data (RxPDO mapped, slot-dependent)
static constexpr uint16_t kDataIndex = 0x7000;

constexpr RegisterEntry DataCount = {
    .index = kDataIndex, .subindex = 0x00,
    .name = "Analog Output Number of Entries",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 4, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 4,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of AO entries",
};

#define RP20_AO_OUTPUT_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kDataIndex, .subindex = (CH), \
        .name = "Analog Output CH" #CH, \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = -32768, .max_value = 32767, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Analog output channel " #CH, \
    }

RP20_AO_OUTPUT_REG(OutputCH0, 0x01);
RP20_AO_OUTPUT_REG(OutputCH1, 0x02);
RP20_AO_OUTPUT_REG(OutputCH2, 0x03);
RP20_AO_OUTPUT_REG(OutputCH3, 0x04);
#undef RP20_AO_OUTPUT_REG

// 0x8000 - AO configuration (slot-dependent)
static constexpr uint16_t kConfigIndex = 0x8000;

#define RP20_AO_SIGNAL_FORM_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "AO CH" #CH " Signal Form", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 3, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Signal form for AO channel " #CH, \
    }

RP20_AO_SIGNAL_FORM_REG(SignalFormCH0, 0x01);
RP20_AO_SIGNAL_FORM_REG(SignalFormCH1, 0x02);
RP20_AO_SIGNAL_FORM_REG(SignalFormCH2, 0x03);
RP20_AO_SIGNAL_FORM_REG(SignalFormCH3, 0x04);
#undef RP20_AO_SIGNAL_FORM_REG

#define RP20_AO_STOPMODE_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "AO CH" #CH " Stopmode After EtherCAT Lost Link", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 255, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Stop mode for AO channel " #CH, \
    }

RP20_AO_STOPMODE_REG(StopModeCH0, 0x05);
RP20_AO_STOPMODE_REG(StopModeCH1, 0x06);
RP20_AO_STOPMODE_REG(StopModeCH2, 0x07);
RP20_AO_STOPMODE_REG(StopModeCH3, 0x08);
#undef RP20_AO_STOPMODE_REG

#define RP20_AO_STOPVALUE_REG(NAME, CH) \
    constexpr RegisterEntry NAME = { \
        .index = kConfigIndex, .subindex = (CH), \
        .name = "AO CH" #CH " Stopvalue After EtherCAT Lost Link", \
        .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = -32768, .max_value = 32767, \
        .modification_mode = ModificationMode::AtStop, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Stop value for AO channel " #CH, \
    }

RP20_AO_STOPVALUE_REG(StopValueCH0, 0x09);
RP20_AO_STOPVALUE_REG(StopValueCH1, 0x0A);
RP20_AO_STOPVALUE_REG(StopValueCH2, 0x0B);
RP20_AO_STOPVALUE_REG(StopValueCH3, 0x0C);
#undef RP20_AO_STOPVALUE_REG

static const RegisterList kRegisterList = {
    &DataCount, &OutputCH0, &OutputCH1, &OutputCH2, &OutputCH3,
    &SignalFormCH0, &SignalFormCH1, &SignalFormCH2, &SignalFormCH3,
    &StopModeCH0, &StopModeCH1, &StopModeCH2, &StopModeCH3,
    &StopValueCH0, &StopValueCH1, &StopValueCH2, &StopValueCH3,
};

} // namespace AO
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
