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
namespace C13 {

static constexpr uint16_t C13ObjectIndex = 0x2013; // Group C13 (EtherCAT Parameters)

enum class EnhancedLinkSelectionOptions : uint16_t {
    Inactive = 0,
    Enabled = 1,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSlaveName = {
    .index = C13ObjectIndex,
    .subindex = 0x01, // C13.00 -> subindex = 0x00 + 1
    .name = "EtherCAT slave name",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSlaveAlias = {
    .index = C13ObjectIndex,
    .subindex = 0x02, // C13.01
    .name = "EtherCAT slave alias",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSyncLossThreshold = {
    .index = C13ObjectIndex,
    .subindex = 0x03, // C13.02
    .name = "EtherCAT sync loss threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8,
    .min_value = 1,
    .max_value = 20,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSyncDetectionMode = {
    .index = C13ObjectIndex,
    .subindex = 0x04, // C13.03
    .name = "EtherCAT synchronization detection mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSyncLossCount = {
    .index = C13ObjectIndex,
    .subindex = 0x05, // C13.04
    .name = "EtherCAT sync loss count",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSyncModeSetting = {
    .index = C13ObjectIndex,
    .subindex = 0x06, // C13.05
    .name = "EtherCAT synchronization mode setting",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATSyncErrorThreshold = {
    .index = C13ObjectIndex,
    .subindex = 0x07, // C13.06
    .name = "EtherCAT synchronization error threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .min_value = 0,
    .max_value = 6000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OccurrenceCountExcessivePosRef = {
    .index = C13ObjectIndex,
    .subindex = 0x08, // C13.07
    .name = "Occurrence count of excessive position reference increment in sync position mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 5,
    .min_value = 1,
    .max_value = 30,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATEnhancedLinkSelection = {
    .index = C13ObjectIndex,
    .subindex = 0x09, // C13.08
    .name = "EtherCAT enhanced link selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<EnhancedLinkSelectionOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::UponRepowerOn,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &EtherCATSlaveName,
    &EtherCATSlaveAlias,
    &EtherCATSyncLossThreshold,
    &EtherCATSyncDetectionMode,
    &EtherCATSyncLossCount,
    &EtherCATSyncModeSetting,
    &EtherCATSyncErrorThreshold,
    &OccurrenceCountExcessivePosRef,
    &EtherCATEnhancedLinkSelection,
};

} // namespace C13
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
