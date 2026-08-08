/**
 * @file Safety2600.hpp
 * @brief Synapticon SOMANET safety objects 0x2600-0x26F0
 *
 * Safe-motion status, safe inputs/outputs, safety module diagnostics,
 * manufacturing parameters, general safety config, safety digital IO,
 * safety analog input, encoder source/verification/selection, and
 * STO/SS1/SOS/SS2/SLS input configuration.
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
// 0x2600 Temperature warning — BOOL, ro
// ============================================================================
namespace Obj2600 {

static constexpr uint16_t ObjectIndex = 0x2600;

constexpr RegisterEntry TemperatureWarning = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Temperature warning",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &TemperatureWarning,
};

} // namespace Obj2600

// ============================================================================
// 0x2601 Safe position valid — BOOL, ro
// ============================================================================
namespace Obj2601 {

static constexpr uint16_t ObjectIndex = 0x2601;

constexpr RegisterEntry SafePositionValid = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Safe position valid",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SafePositionValid,
};

} // namespace Obj2601

// ============================================================================
// 0x2602 Safe speed valid — BOOL, ro
// ============================================================================
namespace Obj2602 {

static constexpr uint16_t ObjectIndex = 0x2602;

constexpr RegisterEntry SafeSpeedValid = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Safe speed valid",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SafeSpeedValid,
};

} // namespace Obj2602

// ============================================================================
// 0x2603 Safe input — DT2603, rw
// ============================================================================
namespace Obj2603 {

static constexpr uint16_t ObjectIndex = 0x2603;

constexpr RegisterEntry SafeInput1 = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Safe input 1",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeInput2 = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Safe input 2",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeInput3 = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Safe input 3",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeInput4 = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Safe input 4",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SafeInput1,
    &SafeInput2,
    &SafeInput3,
    &SafeInput4,
};

} // namespace Obj2603

// ============================================================================
// 0x2604 Safe output monitor — DT2604, rw
// ============================================================================
namespace Obj2604 {

static constexpr uint16_t ObjectIndex = 0x2604;

constexpr RegisterEntry SafeOutputMonitor1 = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Safe output monitor 1",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeOutputMonitor2 = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Safe output monitor 2",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SafeOutputMonitor1,
    &SafeOutputMonitor2,
};

} // namespace Obj2604

// ============================================================================
// 0x2605 Analog input — DT2605, rw
// ============================================================================
namespace Obj2605 {

static constexpr uint16_t ObjectIndex = 0x2605;

constexpr RegisterEntry SafeAnalogValueScaled = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Safe analog value (scaled)",
    .data_type = ODDataType::Integer16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = -32768,
    .max_value = 32767,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafePositionActualValueLow = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Safe position actual value low",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafePositionActualValueHigh = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Safe position actual value high",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeVelocityActualValueLow = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Safe velocity actual value low",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SafeAnalogValueScaled,
    &SafePositionActualValueLow,
    &SafePositionActualValueHigh,
    &SafeVelocityActualValueLow,
};

} // namespace Obj2605

// ============================================================================
// 0x2610 Manufacturing parameters — DT2610
//   Access from web docs: all readonly (default)
// ============================================================================
namespace Obj2610 {

static constexpr uint16_t ObjectIndex = 0x2610;

constexpr RegisterEntry VendorID = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Vendor ID",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry DeviceType = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Device Type",
    .data_type = ODDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry DeviceVersion = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Device Version",
    .data_type = ODDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SerialNumber = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Serial Number",
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
    &VendorID,
    &DeviceType,
    &DeviceVersion,
    &SerialNumber,
};

} // namespace Obj2610

// ============================================================================
// 0x2611 Safety Module input diagnostics — DT2611
//   Access from web docs: :1-2 readonly (default)
// ============================================================================
namespace Obj2611 {

static constexpr uint16_t ObjectIndex = 0x2611;

constexpr RegisterEntry Input1 = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Input 1",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Input2 = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Input 2",
    .data_type = ODDataType::Boolean,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Input1,
    &Input2,
};

} // namespace Obj2611

// ============================================================================
// 0x2620 General — DT2620
//   Access from web docs: :1-4 readwrite
// ============================================================================
namespace Obj2620 {

static constexpr uint16_t ObjectIndex = 0x2620;

constexpr RegisterEntry DriveSafetyName = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Drive safety name",
    .data_type = ODDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeFieldbus = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Safe fieldbus",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeAddress = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Safe address",
    .data_type = ODDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry FSoEDownload = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "FSoE Download",
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
    &DriveSafetyName,
    &SafeFieldbus,
    &SafeAddress,
    &FSoEDownload,
};

} // namespace Obj2620

// ============================================================================
// 0x2621 Safety digital IO — DT2621
//   Access from web docs: :1-8 readwrite
// ============================================================================
namespace Obj2621 {

static constexpr uint16_t ObjectIndex = 0x2621;

constexpr RegisterEntry AcknowledgeViaDrive = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Acknowledge via drive",
    .data_type = ODDataType::Unsigned8,
    .default_value = 255,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AcknowledgementInput = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Acknowledgement input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry InputTestPulseDetection = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Input test pulse detection",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry InputFilterTime = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Input filter time",
    .data_type = ODDataType::Unsigned8,
    .default_value = 4,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 2,
    .max_value = 20,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TestPulseMaxDistance = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Test pulse max. distance",
    .data_type = ODDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 100,
    .max_value = 60000,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry OutputTestPulse = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "Output test pulse",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Output1Function = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "Output1 function",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry Output2Function = {
    .index = ObjectIndex,
    .subindex = 0x08,
    .name = "Output2 function",
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
    &AcknowledgeViaDrive,
    &AcknowledgementInput,
    &InputTestPulseDetection,
    &InputFilterTime,
    &TestPulseMaxDistance,
    &OutputTestPulse,
    &Output1Function,
    &Output2Function,
};

} // namespace Obj2621

// ============================================================================
// 0x2625 Safety IO analog input — DT2625
//   Access from web docs: :1-5 readwrite
// ============================================================================
namespace Obj2625 {

static constexpr uint16_t ObjectIndex = 0x2625;

constexpr RegisterEntry AnalogInput1Gain = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Analog input1 Gain",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AnalogInput1Offset = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Analog input1 Offset",
    .data_type = ODDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -32767,
    .max_value = 32767,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AnalogInput2Gain = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Analog input2 Gain",
    .data_type = ODDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = -0x7FFFFFFF,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AnalogInput2Offset = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Analog input2 Offset",
    .data_type = ODDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = -32767,
    .max_value = 32767,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AnalogAllowedError = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Analog allowed error",
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
    &AnalogInput1Gain,
    &AnalogInput1Offset,
    &AnalogInput2Gain,
    &AnalogInput2Offset,
    &AnalogAllowedError,
};

} // namespace Obj2625

// ============================================================================
// 0x2630 Encoder source type — DT2630
//   Access from web docs: :1-6 readwrite
// ============================================================================
namespace Obj2630 {

static constexpr uint16_t ObjectIndex = 0x2630;

constexpr RegisterEntry EncoderSourceType = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Encoder source type",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry EncoderResolution = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Encoder resolution",
    .data_type = ODDataType::Unsigned32,
    .default_value = 524288,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 16,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry EncoderMultiturnBits = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Encoder multiturn bits",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 64,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry EncoderClockFrequency = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Encoder clock frequency",
    .data_type = ODDataType::Unsigned8,
    .default_value = 18,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 20,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry EncoderTimeout = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Encoder timeout",
    .data_type = ODDataType::Unsigned16,
    .default_value = 36,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 1,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry MultiturnCountingBySMM = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "Multiturn Counting by SMM",
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
    &EncoderSourceType,
    &EncoderResolution,
    &EncoderMultiturnBits,
    &EncoderClockFrequency,
    &EncoderTimeout,
    &MultiturnCountingBySMM,
};

} // namespace Obj2630

// ============================================================================
// 0x2631 Encoder verification — DT2631
//   Access from web docs: :1-3 readwrite
// ============================================================================
namespace Obj2631 {

static constexpr uint16_t ObjectIndex = 0x2631;

constexpr RegisterEntry VerificationSensorSourceType = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Verification sensor source type",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VerificationSensorResolution = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Verification sensor resolution",
    .data_type = ODDataType::Unsigned32,
    .default_value = 524288,
    .unit = Unit_Inc,
    .options_enum = nullptr,
    .min_value = 16,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VerificationSensorMultiturnBits = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Verification sensor multiturn bits",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_RPM,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 64,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &VerificationSensorSourceType,
    &VerificationSensorResolution,
    &VerificationSensorMultiturnBits,
};

} // namespace Obj2631

// ============================================================================
// 0x2635 Encoder selection — DT2635
//   Access from web docs: :1-9 readwrite
// ============================================================================
namespace Obj2635 {

static constexpr uint16_t ObjectIndex = 0x2635;

constexpr RegisterEntry SpeedWindow = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Speed window",
    .data_type = ODDataType::Unsigned8,
    .default_value = 4,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 1,
    .max_value = 100,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AbsolutePosition = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Absolute position",
    .data_type = ODDataType::Unsigned8,
    .default_value = 255,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry PositionResetInput = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "Position reset input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AbsolutePositionOnReset = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "Absolute position on reset",
    .data_type = ODDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AllowedPositionDiscrepancy = {
    .index = ObjectIndex,
    .subindex = 0x05,
    .name = "Allowed position discrepancy",
    .data_type = ODDataType::Unsigned16,
    .default_value = 100,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry AllowedSpeedDiscrepancy = {
    .index = ObjectIndex,
    .subindex = 0x06,
    .name = "Allowed speed discrepancy",
    .data_type = ODDataType::Unsigned16,
    .default_value = 100,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry DiscrepancyTimer = {
    .index = ObjectIndex,
    .subindex = 0x07,
    .name = "Discrepancy timer",
    .data_type = ODDataType::Unsigned16,
    .default_value = 200,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 10,
    .max_value = 1000,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VerificationScalingNumerator = {
    .index = ObjectIndex,
    .subindex = 0x08,
    .name = "Verification scaling numerator",
    .data_type = ODDataType::Integer16,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = -32767,
    .max_value = 32767,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry VerificationScalingDenominator = {
    .index = ObjectIndex,
    .subindex = 0x09,
    .name = "Verification scaling denominator",
    .data_type = ODDataType::Unsigned16,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 1,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SpeedWindow,
    &AbsolutePosition,
    &PositionResetInput,
    &AbsolutePositionOnReset,
    &AllowedPositionDiscrepancy,
    &AllowedSpeedDiscrepancy,
    &DiscrepancyTimer,
    &VerificationScalingNumerator,
    &VerificationScalingDenominator,
};

} // namespace Obj2635

// ============================================================================
// 0x2641 STO input — DT2641
//   Access from web docs: :1-2 readwrite
// ============================================================================
namespace Obj2641 {

static constexpr uint16_t ObjectIndex = 0x2641;

constexpr RegisterEntry STOInput = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "STO input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SBC = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "SBC",
    .data_type = ODDataType::Unsigned8,
    .default_value = 255,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &STOInput,
    &SBC,
};

} // namespace Obj2641

// ============================================================================
// 0x2650 SS1 input — DT2650
//   Access from web docs: :1-2 readwrite
// ============================================================================
namespace Obj2650 {

static constexpr uint16_t ObjectIndex = 0x2650;

constexpr RegisterEntry SS1Input = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "SS1 input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SS1DecelerationMonitoring = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "SS1: Deceleration monitoring",
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
    &SS1Input,
    &SS1DecelerationMonitoring,
};

} // namespace Obj2650

// ============================================================================
// 0x2668 SOS input — DT2668
//   Access from web docs: :1-2 readwrite
// ============================================================================
namespace Obj2668 {

static constexpr uint16_t ObjectIndex = 0x2668;

constexpr RegisterEntry SOSInput = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "SOS input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry TDSOS = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "t_D_SOS",
    .data_type = ODDataType::Unsigned16,
    .default_value = 200,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 60000,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SOSInput,
    &TDSOS,
};

} // namespace Obj2668

// ============================================================================
// 0x2670 SS2 input — DT2670
//   Access from web docs: :1 readwrite
// ============================================================================
namespace Obj2670 {

static constexpr uint16_t ObjectIndex = 0x2670;

constexpr RegisterEntry SS2Input = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "SS2 input",
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
    &SS2Input,
};

} // namespace Obj2670

// ============================================================================
// 0x2690 SLS input — DT2690
//   Access from web docs: :1-4 readwrite
// ============================================================================
namespace Obj2690 {

static constexpr uint16_t ObjectIndex = 0x2690;

constexpr RegisterEntry SLS1Input = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "SLS1 input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SLS2Input = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "SLS2 input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SLS3Input = {
    .index = ObjectIndex,
    .subindex = 0x03,
    .name = "SLS3 input",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SLS4Input = {
    .index = ObjectIndex,
    .subindex = 0x04,
    .name = "SLS4 input",
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
    &SLS1Input,
    &SLS2Input,
    &SLS3Input,
    &SLS4Input,
};

} // namespace Obj2690

// ============================================================================
// 0x26A0 Reset position — BOOL, rw
// ============================================================================
namespace Obj26A0 {

static constexpr uint16_t ObjectIndex = 0x26A0;

constexpr RegisterEntry ResetPosition = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "Reset position",
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
    &ResetPosition,
};

} // namespace Obj26A0

// ============================================================================
// 0x26F0 Safe output — DT26F0, rw
// ============================================================================
namespace Obj26F0 {

static constexpr uint16_t ObjectIndex = 0x26F0;

constexpr RegisterEntry SafeOutput1 = {
    .index = ObjectIndex,
    .subindex = 0x01,
    .name = "Safe output 1",
    .data_type = ODDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

constexpr RegisterEntry SafeOutput2 = {
    .index = ObjectIndex,
    .subindex = 0x02,
    .name = "Safe output 2",
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
    &SafeOutput1,
    &SafeOutput2,
};

} // namespace Obj26F0

} // namespace Synapticon
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
