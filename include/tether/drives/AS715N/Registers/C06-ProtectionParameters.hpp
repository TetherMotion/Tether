#pragma once
#include <cstddef>
#include <cstdint>
#include "tether/slave/mailbox/IMailboxHandler.hpp"
#include "tether/profiles/cia401/CiA401Defs.hpp"
#include "tether/profiles/cia404/CiA404Defs.hpp"
#include "tether/drives/AS715N/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace AS715N {
namespace C06 {

static constexpr uint16_t C06ObjectIndex = 0x2006; // Group C06 (Protection Parameters)

// Binary enable/disable options (common)
enum class BinaryEnableOptions : uint16_t {
    Inactive = 0,
    Enabled = 1,
};

// Some registers use vendor-specific boolean encodings (0=Enabled / 1=Disabled)
enum class InputPhaseLossDetectionOptions : uint16_t {
    Enabled = 0,
    Disabled = 1,
};

enum class RetentiveAtPowerFailureOptions : uint16_t {
    NonRetentive = 0,
    Retentive = 1,
};

enum class MechanicalLimitPositionOptions : uint16_t {
    Inactive = 0,
    Enabled = 1,
    EnabledAfterHoming = 2,
};

// C06.03 — Threshold of excessive speed
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ThresholdOfExcessiveSpeed = {
    .index = C06ObjectIndex,
    .subindex = 0x04,
    .name = "Threshold of excessive speed",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 9000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.04 — Input phase loss detection (vendor encoding: 0=Enabled, 1=Disabled)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InputPhaseLossDetection = {
    .index = C06ObjectIndex,
    .subindex = 0x05,
    .name = "Input phase loss detection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<InputPhaseLossDetectionOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.05 — Retentive at power failure
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RetentiveAtPowerFailure = {
    .index = C06ObjectIndex,
    .subindex = 0x06,
    .name = "Retentive at power failure",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<RetentiveAtPowerFailureOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.07 — Mechanical limit position (enable/behavior)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalLimitPosition = {
    .index = C06ObjectIndex,
    .subindex = 0x08,
    .name = "Mechanical limit position",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<MechanicalLimitPositionOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.08 — Mechanical PL (positive limit)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalPositiveLimit = {
    .index = C06ObjectIndex,
    .subindex = 0x09,
    .name = "Mechanical PL",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = static_cast<uint32_t>(INT32_MAX),
    .unit = Unit_None, // "Unit in application"
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.0A — Mechanical NL (negative limit)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalNegativeLimit = {
    .index = C06ObjectIndex,
    .subindex = 0x0B,
    .name = "Mechanical NL",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = static_cast<uint32_t>(INT32_MIN),
    .unit = Unit_None, // "Unit in application"
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.10 — Drive overload protection threshold (0.1%)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DriveOverloadProtectionThreshold = {
    .index = C06ObjectIndex,
    .subindex = 0x11,
    .name = "Drive overload protection threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1150,
    .unit = Unit_Percent, // value is in 0.1%
    .min_value = 0,
    .max_value = 3500,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C06.11 — Motor overload protection threshold (0.1%)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorOverloadProtectionThreshold = {
    .index = C06ObjectIndex,
    .subindex = 0x12,
    .name = "Motor overload protection threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1150,
    .unit = Unit_Percent, // value is in 0.1%
    .min_value = 0,
    .max_value = 3500,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// Existing entries (previously added)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorLockedRotorDetection = {
    .index = C06ObjectIndex,
    .subindex = 0x13,
    .name = "Motor locked-rotor detection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .options_enum = std::type_identity<BinaryEnableOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorLockedRotorDetectionTime = {
    .index = C06ObjectIndex,
    .subindex = 0x14,
    .name = "Motor locked-rotor detection time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 200,
    .min_value = 0,
    .max_value = 3000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorLockedRotorDetectionSpeed = {
    .index = C06ObjectIndex,
    .subindex = 0x15,
    .name = "Motor locked-rotor detection speed",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OutputPhaseLossDetection = {
    .index = C06ObjectIndex,
    .subindex = 0x16,
    .name = "Output phase loss detection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<BinaryEnableOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderCommFaultToleranceThreshold = {
    .index = C06ObjectIndex,
    .subindex = 0x1C,
    .name = "Encoder communication fault tolerance threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3,
    .min_value = 0,
    .max_value = 88,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ProtectionFromOutOfControl = {
    .index = C06ObjectIndex,
    .subindex = 0x20,
    .name = "Protection from out of control",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .options_enum = std::type_identity<BinaryEnableOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &ThresholdOfExcessiveSpeed,            // 0x04
    &InputPhaseLossDetection,              // 0x05
    &RetentiveAtPowerFailure,              // 0x06
    &MechanicalLimitPosition,              // 0x08
    &MechanicalPositiveLimit,              // 0x09 (PL)
    &MechanicalNegativeLimit,              // 0x0B (NL)
    &DriveOverloadProtectionThreshold,     // 0x11
    &MotorOverloadProtectionThreshold,     // 0x12
    &MotorLockedRotorDetection,            // 0x13
    &MotorLockedRotorDetectionTime,        // 0x14
    &MotorLockedRotorDetectionSpeed,       // 0x15
    &OutputPhaseLossDetection,             // 0x16
    &EncoderCommFaultToleranceThreshold,   // 0x1C
    &ProtectionFromOutOfControl,           // 0x20
};

} // namespace C06
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
