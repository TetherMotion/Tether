/**
 * @file Controllers2010.hpp
 * @brief Synapticon SOMANET manufacturer objects 0x2010-0x2027
 *
 * Torque, velocity, position controllers, gain scheduling, filters,
 * following error, and control input FIR filter.
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
// 0x2010 Torque controller — DT2010, rw
// ============================================================================
namespace Obj2010 {

static constexpr uint16_t ObjectIndex = 0x2010;

constexpr RegisterEntry TorqueControllerKp = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Torque controller Kp",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TorqueControllerKi = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Torque controller Ki",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TorqueControllerKd = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Torque controller Kd",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TorqueControllerIntegralLimit = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Torque controller integral limit",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &TorqueControllerKp,
    &TorqueControllerKi,
    &TorqueControllerKd,
    &TorqueControllerIntegralLimit,
};

} // namespace Obj2010

// ============================================================================
// 0x2011 Velocity controller — DT2011, rw
// ============================================================================
namespace Obj2011 {

static constexpr uint16_t ObjectIndex = 0x2011;

constexpr RegisterEntry VelocityControllerKp = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Velocity controller Kp",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VelocityControllerKi = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Velocity controller Ki",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VelocityControllerIntegralLimit = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Velocity controller integral limit",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &VelocityControllerKp,
    &VelocityControllerKi,
    &VelocityControllerIntegralLimit,
};

} // namespace Obj2011

// ============================================================================
// 0x2012 Position controller — DT2012, rw
// ============================================================================
namespace Obj2012 {

static constexpr uint16_t ObjectIndex = 0x2012;

constexpr RegisterEntry PositionControllerKp = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Position controller Kp",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PositionControllerKi = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Position controller Ki",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PositionControllerKd = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Position controller Kd",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PositionControllerIntegralLimit = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Position controller integral limit",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PositionControllerFeedForward = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Position controller feed forward",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &PositionControllerKp,
    &PositionControllerKi,
    &PositionControllerKd,
    &PositionControllerIntegralLimit,
    &PositionControllerFeedForward,
};

} // namespace Obj2012

// ============================================================================
// 0x2017 Following error option codes — DT2017
//   Access from web docs: :1 readwrite
// ============================================================================
namespace Obj2017 {

static constexpr uint16_t ObjectIndex = 0x2017;

enum class FollowingErrorOptionCodeOptions : uint8_t {
    RaiseFollowingErrorBitAndStayOperational = 0,
    RaiseFollowingErrorBitQuickStopAndFault = 1,
    RaiseFollowingErrorBitAndImmediatelyFault = 2,
};

constexpr RegisterEntry PositionFollowingErrorOptionCode = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Position following error option code",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<FollowingErrorOptionCodeOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &PositionFollowingErrorOptionCode,
};

} // namespace Obj2017

// ============================================================================
// 0x2021 Velocity feedback filter — DT2021, rw
// ============================================================================
namespace Obj2021 {

static constexpr uint16_t ObjectIndex = 0x2021;

constexpr RegisterEntry CutoffFrequency = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Cutoff frequency",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Enable = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Enable",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &CutoffFrequency,
    &Enable,
};

} // namespace Obj2021

// ============================================================================
// 0x2022 Position feedback filter — DT2022, rw
// ============================================================================
namespace Obj2022 {

static constexpr uint16_t ObjectIndex = 0x2022;

constexpr RegisterEntry CutoffFrequency = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Cutoff frequency",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Enable = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Enable",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &CutoffFrequency,
    &Enable,
};

} // namespace Obj2022

// ============================================================================
// 0x2023 Notch filter — DT2023, rw
// ============================================================================
namespace Obj2023 {

static constexpr uint16_t ObjectIndex = 0x2023;

constexpr RegisterEntry RejectionBand = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Rejection band",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Frequency_Hertz,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Enable = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Enable",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &RejectionBand,
    &Enable,
};

} // namespace Obj2023

// ============================================================================
// 0x2027 Control input FIR filter — DT2027, rw
// ============================================================================
namespace Obj2027 {

static constexpr uint16_t ObjectIndex = 0x2027;

constexpr RegisterEntry Order = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Order",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Order,
};

} // namespace Obj2027

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
