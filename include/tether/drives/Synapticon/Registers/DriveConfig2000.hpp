/**
 * @file DriveConfig2000.hpp
 * @brief Synapticon SOMANET manufacturer objects 0x2000-0x200B
 *
 * Object dictionary entries extracted from SOMANET_CiA_402_v5.1.9.xml ESI
 * with access rights supplemented from Synapticon documentation:
 *   https://doc.synapticon.com/circulo_safe_motion/sw5.1/objects_html/2xxx/
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
// 0x2000 Command object (disabled) — UINT, rw
// ============================================================================
namespace Obj2000 {

static constexpr uint16_t ObjectIndex = 0x2000;

constexpr RegisterEntry CommandObject = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Command object (disabled)",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &CommandObject,
};

} // namespace Obj2000

// ============================================================================
// 0x2001 Commutation angle offset — INT, rw
// ============================================================================
namespace Obj2001 {

static constexpr uint16_t ObjectIndex = 0x2001;

constexpr RegisterEntry CommutationAngleOffset = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Commutation angle offset",
    .data_type = ODDataType::Integer16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = -32768,
    .max_value = 32767,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &CommutationAngleOffset,
};

} // namespace Obj2001

// ============================================================================
// 0x2002 Position control strategy — UINT, rw
// ============================================================================
namespace Obj2002 {

static constexpr uint16_t ObjectIndex = 0x2002;

enum class PositionControlStrategyOptions : uint16_t {
    Standard = 0,
    Enhanced = 1,
};

constexpr RegisterEntry PositionControlStrategy = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Position control strategy",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<PositionControlStrategyOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &PositionControlStrategy,
};

} // namespace Obj2002

// ============================================================================
// 0x2003 Motor specific settings — DT2003, rw
// ============================================================================
namespace Obj2003 {

static constexpr uint16_t ObjectIndex = 0x2003;

constexpr RegisterEntry PolePairs = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Pole pairs",
    .data_type = ODDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry TorqueConstant = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Torque constant",
    .data_type = ODDataType::Integer32,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry PhaseResistance = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Phase resistance",
    .data_type = ODDataType::Integer32,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry PhaseInductance = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Phase inductance",
    .data_type = ODDataType::Integer32,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr RegisterEntry MotorPhasesInverted = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Motor phases inverted",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

inline const RegisterList kRegisterList = {
    &PolePairs,
    &TorqueConstant,
    &PhaseResistance,
    &PhaseInductance,
    &MotorPhasesInverted,
};

} // namespace Obj2003

// ============================================================================
// 0x2004 Brake options — DT2004, rw
// ============================================================================
namespace Obj2004 {

static constexpr uint16_t ObjectIndex = 0x2004;

constexpr RegisterEntry PullVoltage = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Pull voltage",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Voltage_Volt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry HoldVoltage = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Hold voltage",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Voltage_Volt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PullTime = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Pull time",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ReleaseStrategy = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Release strategy",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry ControllerDisableDelay = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Controller disable delay",
    .data_type = ODDataType::Unsigned16,
    .default_value = 100,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry BrakeStatus = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "Brake status",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry OutputVoltage = {
    .index = ObjectIndex,
    .subindex = 0x0A,
    .name = "Output voltage",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Voltage_Volt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SwitchingFrequency = {
    .index = ObjectIndex,
    .subindex = 0x0B,
    .name = "Switching frequency",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &PullVoltage,
    &HoldVoltage,
    &PullTime,
    &ReleaseStrategy,
    &ControllerDisableDelay,
    &BrakeStatus,
    &OutputVoltage,
    &SwitchingFrequency,
};

} // namespace Obj2004

// ============================================================================
// 0x2005 Homing Options — DT2005
//   Access from web docs: :1 readonly, :2-4 readwrite
// ============================================================================
namespace Obj2005 {

static constexpr uint16_t ObjectIndex = 0x2005;

constexpr RegisterEntry HomePosition = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Home Position",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

enum class RestoreHomePositionOptions : uint8_t {
    No = 0,
    Yes = 1,
};

constexpr RegisterEntry RestoreHomePosition = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Restore Home Position when loading configuration",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<RestoreHomePositionOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TorqueThresholdForHoming = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Torque Threshold for Homing on end stop",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry OffsetFromEndstop = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Offset From Endstop",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &HomePosition,
    &RestoreHomePosition,
    &TorqueThresholdForHoming,
    &OffsetFromEndstop,
};

} // namespace Obj2005

// ============================================================================
// 0x2006 Protection — DT2006, rw
// ============================================================================
namespace Obj2006 {

static constexpr uint16_t ObjectIndex = 0x2006;

constexpr RegisterEntry UndervoltageSetpoint = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Undervoltage setpoint",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Voltage_Volt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry OvervoltageSetpoint = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Overvoltage setpoint",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Voltage_Volt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry OvercurrentSetpoint = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Overcurrent setpoint",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Current_Ampere,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry MaxPower = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Max power",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Power_Watt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &UndervoltageSetpoint,
    &OvervoltageSetpoint,
    &OvercurrentSetpoint,
    &MaxPower,
};

} // namespace Obj2006

// ============================================================================
// 0x2008 Cogging torque compensation — DT2008, rw
// ============================================================================
namespace Obj2008 {

static constexpr uint16_t ObjectIndex = 0x2008;

constexpr RegisterEntry Enabled = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Enabled",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry CoggingTorqueCompensationValue = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Cogging torque compensation value",
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
    &Enabled,
    &CoggingTorqueCompensationValue,
};

} // namespace Obj2008

// ============================================================================
// 0x200A I2t and Stall protection — DT200A, rw
// ============================================================================
namespace Obj200A {

static constexpr uint16_t ObjectIndex = 0x200A;

constexpr RegisterEntry Enabled = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Enabled",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry I2tLimit = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "I2t limit",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Current_Ampere,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry StallTime = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Stall time",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry StallVelocity = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Stall velocity",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry MotorI2t = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Motor i2t",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_Percent,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Enabled,
    &I2tLimit,
    &StallTime,
    &StallVelocity,
    &MotorI2t,
};

} // namespace Obj200A

// ============================================================================
// 0x200B Max power — UDINT, rw
// ============================================================================
namespace Obj200B {

static constexpr uint16_t ObjectIndex = 0x200B;

constexpr RegisterEntry MaxPower = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Max power",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_Power_Watt,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &MaxPower,
};

} // namespace Obj200B

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
