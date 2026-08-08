/**
 * @file Encoders2110.hpp
 * @brief Synapticon SOMANET manufacturer objects 0x2110-0x2113
 *
 * Encoder 1 and 2 configuration and feedback.
 *
 * Object dictionary entries extracted from SOMANET_CiA_402_v5.1.9.xml ESI
 * with access rights supplemented from Synapticon documentation:
 *   0x2111/0x2113 feedback subindices: readonly (default)
 */

#pragma once

#include <cstdint>
#include "tether/drives/Synapticon/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace Synapticon {

using ODDataType = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;

// ============================================================================
// 0x2110 Encoder 1 configuration — DT2110, rw
// ============================================================================
namespace Obj2110 {

static constexpr uint16_t ObjectIndex = 0x2110;

constexpr RegisterEntry SamplingFrequency = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Sampling frequency",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry SourceType = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Source type",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry Resolution = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Resolution",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry MultiturnBits = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Multiturn bits",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 64,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry Polarity = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Polarity",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry SingleturnOffset = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "Singleturn offset",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry MultiturnOffset = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "Multiturn offset",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry BissClockFrequency = {
    .index = ObjectIndex,
    .subindex = 0x08,
    .name = "BiSS clock frequency",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

inline const RegisterList kRegisterList = {
    &SamplingFrequency,
    &SourceType,
    &Resolution,
    &MultiturnBits,
    &Polarity,
    &SingleturnOffset,
    &MultiturnOffset,
    &BissClockFrequency,
};

} // namespace Obj2110

// ============================================================================
// 0x2111 Encoder 1 feedback — DT2111
//   Access from web docs: all readonly (default)
// ============================================================================
namespace Obj2111 {

static constexpr uint16_t ObjectIndex = 0x2111;

constexpr RegisterEntry RawPosition = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Raw position",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AdjustedPosition = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Adjusted position",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Velocity = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Velocity",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &RawPosition,
    &AdjustedPosition,
    &Velocity,
};

} // namespace Obj2111

// ============================================================================
// 0x2112 Encoder 2 configuration — DT2112, rw
// ============================================================================
namespace Obj2112 {

static constexpr uint16_t ObjectIndex = 0x2112;

constexpr RegisterEntry SamplingFrequency = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Sampling frequency",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry SourceType = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Source type",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry Resolution = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Resolution",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry MultiturnBits = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Multiturn bits",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 64,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry Polarity = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Polarity",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry SingleturnOffset = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "Singleturn offset",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry MultiturnOffset = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "Multiturn offset",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry BissClockFrequency = {
    .index = ObjectIndex,
    .subindex = 0x08,
    .name = "BiSS clock frequency",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

inline const RegisterList kRegisterList = {
    &SamplingFrequency,
    &SourceType,
    &Resolution,
    &MultiturnBits,
    &Polarity,
    &SingleturnOffset,
    &MultiturnOffset,
    &BissClockFrequency,
};

} // namespace Obj2112

// ============================================================================
// 0x2113 Encoder 2 feedback — DT2113
//   Access from web docs: all readonly (default)
// ============================================================================
namespace Obj2113 {

static constexpr uint16_t ObjectIndex = 0x2113;

constexpr RegisterEntry RawPosition = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Raw position",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AdjustedPosition = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Adjusted position",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Velocity = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Velocity",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &RawPosition,
    &AdjustedPosition,
    &Velocity,
};

} // namespace Obj2113

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
