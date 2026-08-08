/**
 * @file GPIO2210.hpp
 * @brief Synapticon SOMANET manufacturer objects 0x2210-0x2215
 *
 * GPIO pin configuration, output events, input actions, position trigger,
 * global options, and addressable LED control.
 *
 * Object dictionary entries extracted from SOMANET_CiA_402_v5.1.9.xml ESI
 * with access rights supplemented from Synapticon documentation:
 *   0x2211:1 readwrite, 0x2212:1 readonly (default),
 *   0x2213:1-3 readwrite, 0x2214:1 readwrite, 0x2215:1 readwrite
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
// 0x2210 GPIO pin configuration — DT2210, rw
// ============================================================================
namespace Obj2210 {

static constexpr uint16_t ObjectIndex = 0x2210;

constexpr RegisterEntry GPIO0 = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "GPIO 0",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO1 = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "GPIO 1",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO2 = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "GPIO 2",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO3 = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "GPIO 3",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO4 = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "GPIO 4",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO5 = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "GPIO 5",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO6 = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "GPIO 6",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry GPIO7 = {
    .index = ObjectIndex,
    .subindex = 0x08,
    .name = "GPIO 7",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &GPIO0, &GPIO1, &GPIO2, &GPIO3,
    &GPIO4, &GPIO5, &GPIO6, &GPIO7,
};

} // namespace Obj2210

// ============================================================================
// 0x2211 GPIO output events — DT2211
//   Access from web docs: :1 readwrite
// ============================================================================
namespace Obj2211 {

static constexpr uint16_t ObjectIndex = 0x2211;

constexpr RegisterEntry TimestampSetpoint = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Timestamp setpoint",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &TimestampSetpoint,
};

} // namespace Obj2211

// ============================================================================
// 0x2212 GPIO input actions — DT2212
//   Access from web docs: :1 readonly (default)
// ============================================================================
namespace Obj2212 {

static constexpr uint16_t ObjectIndex = 0x2212;

constexpr RegisterEntry TimestampOnRisingEdge = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Timestamp on rising edge",
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
    &TimestampOnRisingEdge,
};

} // namespace Obj2212

// ============================================================================
// 0x2213 GPIO position trigger — DT2213
//   Access from web docs: :1-3 readwrite
// ============================================================================
namespace Obj2213 {

static constexpr uint16_t ObjectIndex = 0x2213;

constexpr RegisterEntry PositionSetpoint = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Position setpoint",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

enum class TargetEncoderOptions : uint8_t {
    NoEncoderSelected = 0,
    Encoder1 = 1,
    Encoder2 = 2,
};

constexpr RegisterEntry TargetEncoder = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Target encoder",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<TargetEncoderOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PulseWidth = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Pulse width",
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
    &PositionSetpoint,
    &TargetEncoder,
    &PulseWidth,
};

} // namespace Obj2213

// ============================================================================
// 0x2214 GPIO global options — DT2214
//   Access from web docs: :1 readwrite
// ============================================================================
namespace Obj2214 {

static constexpr uint16_t ObjectIndex = 0x2214;

enum class VoltageLevelOptions : uint8_t {
    Volts3_3 = 1,
    Volts5_0 = 2,
};

constexpr RegisterEntry VoltageLevel = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Voltage level",
    .data_type = ODDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_Voltage_Volt,
    .options_enum = std::type_identity<VoltageLevelOptions>{},
    .min_value = 1,
    .max_value = 2,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &VoltageLevel,
};

} // namespace Obj2214

// ============================================================================
// 0x2215 GPIO addressable LED control — DT2215
//   Access from web docs: :1 readwrite
// ============================================================================
namespace Obj2215 {

static constexpr uint16_t ObjectIndex = 0x2215;

constexpr RegisterEntry LEDColor = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "LED color",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &LEDColor,
};

} // namespace Obj2215

// ============================================================================
// 0x2401 Analog input 1 — UINT, ro
// ============================================================================
namespace Obj2401 {

static constexpr uint16_t ObjectIndex = 0x2401;

constexpr RegisterEntry AnalogInput1 = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Analog input 1",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &AnalogInput1,
};

} // namespace Obj2401

// ============================================================================
// 0x2402 Analog input 2 — UINT, ro
// ============================================================================
namespace Obj2402 {

static constexpr uint16_t ObjectIndex = 0x2402;

constexpr RegisterEntry AnalogInput2 = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Analog input 2",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &AnalogInput2,
};

} // namespace Obj2402

// ============================================================================
// 0x2403 Analog input 3 — UINT, ro
// ============================================================================
namespace Obj2403 {

static constexpr uint16_t ObjectIndex = 0x2403;

constexpr RegisterEntry AnalogInput3 = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Analog input 3",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &AnalogInput3,
};

} // namespace Obj2403

// ============================================================================
// 0x2404 Analog input 4 — UINT, ro
// ============================================================================
namespace Obj2404 {

static constexpr uint16_t ObjectIndex = 0x2404;

constexpr RegisterEntry AnalogInput4 = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Analog input 4",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &AnalogInput4,
};

} // namespace Obj2404

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
