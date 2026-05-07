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
namespace C07 {

static constexpr uint16_t C07ObjectIndex = 0x2007; // Group C07 (Auto-tuning Parameters)

// Register entries for C07 (offline inertia auto-tuning)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OfflineInertiaAutoTuningMode = {
    .index = C07ObjectIndex,
    .subindex = 0x01,
    .name = "Offline inertia auto-tuning mode setting",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 769,
    .min_value = 0,
    .max_value = 785,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OfflineInertiaAutoTuningSpeedRef = {
    .index = C07ObjectIndex,
    .subindex = 0x02,
    .name = "Offline inertia auto-tuning speed reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 500,
    .unit = Unit_RPM,
    .min_value = 50,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OfflineInertiaAutoTuningAccelDecelTime = {
    .index = C07ObjectIndex,
    .subindex = 0x03,
    .name = "Acceleration/Deceleration time for offline inertia auto-tuning",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OfflineInertiaAutoTuningTargetTorque = {
    .index = C07ObjectIndex,
    .subindex = 0x04,
    .name = "Offline inertia auto-tuning target torque",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 150,
    .unit = Unit_Percent, // represented as 0.1% units
    .min_value = 1,
    .max_value = 1500,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OfflineInertiaAutoTuningRevolutions = {
    .index = C07ObjectIndex,
    .subindex = 0x05,
    .name = "Offline inertia auto-tuning revolutions",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 200,
    .min_value = 10,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &OfflineInertiaAutoTuningMode,
    &OfflineInertiaAutoTuningSpeedRef,
    &OfflineInertiaAutoTuningAccelDecelTime,
    &OfflineInertiaAutoTuningTargetTorque,
    &OfflineInertiaAutoTuningRevolutions,
};

} // namespace C07
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
