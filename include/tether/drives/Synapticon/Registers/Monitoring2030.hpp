/**
 * @file Monitoring2030.hpp
 * @brief Synapticon SOMANET manufacturer objects 0x2030-0x2040, 0x20E1-0x20F3
 *
 * Temperature monitoring, error report, input counter, high-resolution data,
 * timestamp, assigned name, and DC synchronization.
 *
 * Object dictionary entries extracted from SOMANET_CiA_402_v5.1.9.xml ESI
 * with access rights supplemented from Synapticon documentation.
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
// 0x2030 Core temperature — DT2030
//   Access from web docs: :1 readonly (default)
// ============================================================================
namespace Obj2030 {

static constexpr uint16_t ObjectIndex = 0x2030;

constexpr RegisterEntry MeasuredTemperature = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Measured temperature",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Temperature_C,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &MeasuredTemperature,
};

} // namespace Obj2030

// ============================================================================
// 0x2031 Drive temperature — DT2031
//   Access from web docs: :1 readonly (default)
// ============================================================================
namespace Obj2031 {

static constexpr uint16_t ObjectIndex = 0x2031;

constexpr RegisterEntry MeasuredTemperature = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Measured temperature",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Temperature_C,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &MeasuredTemperature,
};

} // namespace Obj2031

// ============================================================================
// 0x2038 External scaled measurement — DT2038
//   Access from web docs: :1 readonly (default), :2-13 readwrite
// ============================================================================
namespace Obj2038 {

static constexpr uint16_t ObjectIndex = 0x2038;

constexpr RegisterEntry ScaledMeasurementValue = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Scaled measurement value",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_Dimensionless,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

enum class AnalogInputOptions : uint8_t {
    Internal = 0,
    AnalogInput1 = 1,
    AnalogInput2 = 2,
    AnalogInput3 = 3,
    AnalogInput4 = 4,
};

constexpr RegisterEntry AnalogInput = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Analog input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<AnalogInputOptions>{},
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Resistance = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Resistance",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Resistance_Ohm,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ConstantA0 = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Constant a0",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ConstantA1 = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Constant a1",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ConstantA2 = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "Constant a2",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ConstantA3 = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "Constant a3",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ConstantA4 = {
    .index = ObjectIndex,
    .subindex = 0x08,
    .name = "Constant a4",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ConstantA5 = {
    .index = ObjectIndex,
    .subindex = 0x09,
    .name = "Constant a5",
    .data_type = ODDataType::Real32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &ScaledMeasurementValue,
    &AnalogInput,
    &Resistance,
    &ConstantA0,
    &ConstantA1,
    &ConstantA2,
    &ConstantA3,
    &ConstantA4,
    &ConstantA5,
};

} // namespace Obj2038

// ============================================================================
// 0x203F Error report — DT203F
//   Access from web docs: :1 readonly (default)
// ============================================================================
namespace Obj203F {

static constexpr uint16_t ObjectIndex = 0x203F;

constexpr RegisterEntry Description = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Description",
    .data_type = ODDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Description,
};

} // namespace Obj203F

// ============================================================================
// 0x2040 Input counter — UDINT, ro
// ============================================================================
namespace Obj2040 {

static constexpr uint16_t ObjectIndex = 0x2040;

constexpr RegisterEntry InputCounter = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Input counter",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &InputCounter,
};

} // namespace Obj2040

// ============================================================================
// 0x20E1 High resolution data — DT20E1, ro
// ============================================================================
namespace Obj20E1 {

static constexpr uint16_t ObjectIndex = 0x20E1;

constexpr RegisterEntry PositionActualValue = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Position actual value",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VelocityActualValue = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Velocity actual value",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TorqueActualValue = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Torque actual value",
    .data_type = ODDataType::Integer16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = -32768,
    .max_value = 32767,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Timestamp = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Timestamp",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &PositionActualValue,
    &VelocityActualValue,
    &TorqueActualValue,
    &Timestamp,
};

} // namespace Obj20E1

// ============================================================================
// 0x20F0 Timestamp — UDINT, ro
// ============================================================================
namespace Obj20F0 {

static constexpr uint16_t ObjectIndex = 0x20F0;

constexpr RegisterEntry Timestamp = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Timestamp",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Timestamp,
};

} // namespace Obj20F0

// ============================================================================
// 0x20F2 Assigned name — STRING(50), rw
// ============================================================================
namespace Obj20F2 {

static constexpr uint16_t ObjectIndex = 0x20F2;

constexpr RegisterEntry AssignedName = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Assigned name",
    .data_type = ODDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &AssignedName,
};

} // namespace Obj20F2

// ============================================================================
// 0x20F3 DC synchronization — DT20F3, rw
// ============================================================================
namespace Obj20F3 {

static constexpr uint16_t ObjectIndex = 0x20F3;

constexpr RegisterEntry DcPulseTime = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "DC pulse time",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &DcPulseTime,
};

} // namespace Obj20F3

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
