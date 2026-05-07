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
namespace F30 {

static constexpr uint16_t F30ObjectIndex = 0x2030; // Group F30 (Parameters of Control in Progress)

// Option enums for F30
enum class JogEnableOptions : uint16_t {
    Inactive = 0,
    Active = 1,
};

enum class InertiaAutoTuningOptions : uint16_t {
    Disabled = 0,
    Enabled = 1,
};

enum class InitialAngleAutoTuningOptions : uint16_t {
    Disabled = 0,
    Enabled = 1,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry JogEnableVelocity = {
    .index = F30ObjectIndex,
    .subindex = 0x01,
    .name = "JOG enabling in velocity mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<JogEnableOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry JogEnablePosition = {
    .index = F30ObjectIndex,
    .subindex = 0x02,
    .name = "JOG enabling in position mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<JogEnableOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry JogVelocityReference = {
    .index = F30ObjectIndex,
    .subindex = 0x03,
    .name = "JOG velocity reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry JogAccelDecelTime = {
    .index = F30ObjectIndex,
    .subindex = 0x04,
    .name = "JOG acceleration/deceleration time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 100,
    .min_value = 0,
    .max_value = 3600000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry JogDistancePosition = {
    .index = F30ObjectIndex,
    .subindex = 0x06,
    .name = "JOG distance in position mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 20000,
    .unit = Unit_Inc,
    .min_value = static_cast<int64_t>(INT32_MIN),
    .max_value = static_cast<int64_t>(INT32_MAX),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InertiaAutoTuning = {
    .index = F30ObjectIndex,
    .subindex = 0x11,
    .name = "Inertia auto-tuning selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<InertiaAutoTuningOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InitialAngleAutoTuning = {
    .index = F30ObjectIndex,
    .subindex = 0x12,
    .name = "Initial angle auto-tuning selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<InitialAngleAutoTuningOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &JogEnableVelocity,
    &JogEnablePosition,
    &JogVelocityReference,
    &JogAccelDecelTime,
    &JogDistancePosition,
    &InertiaAutoTuning,
    &InitialAngleAutoTuning,
};

} // namespace F30
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
