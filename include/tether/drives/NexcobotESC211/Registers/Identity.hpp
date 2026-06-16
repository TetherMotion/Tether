#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace Identity {

// Standard CiA 301 identity objects (0x1000-0x1018) are defined generically in
// tether/profiles/cia301/CiA301Defs.hpp and should be used from there.
// Device-specific non-generic objects defined below.

static constexpr uint16_t ErrorSettingsIndex      = 0x10F1;
static constexpr uint16_t TimestampObjectIndex    = 0x10F8;

// ---------------------------------------------------------------------------
// 0x10F1: Error Settings
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ErrorSettings = {
    .index = ErrorSettingsIndex,
    .subindex = 0x00,
    .name = "Error settings",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for 0x10F1 (Local Error Reaction, Sync Error Counter Limit)",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LocalErrorReaction = {
    .index = ErrorSettingsIndex,
    .subindex = 0x01,
    .name = "Local Error Reaction",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Device-specific local error reaction code",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SyncErrorCounterLimit = {
    .index = ErrorSettingsIndex,
    .subindex = 0x02,
    .name = "Sync Error Counter Limit",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 4,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum allowed consecutive sync errors before error reaction",
};

// ---------------------------------------------------------------------------
// 0x10F8: Timestamp Object
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TimestampObject = {
    .index = TimestampObjectIndex,
    .subindex = 0x00,
    .name = "Timestamp object",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned64,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = static_cast<int64_t>(-1),
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "64-bit timestamp (ULINT) for distributed clock synchronisation",
};

// ---------------------------------------------------------------------------
// Device identification (from ESI/SII dump)
// ---------------------------------------------------------------------------

static constexpr uint32_t kVendorId              = 0x00000DCBu;
static constexpr uint32_t kProductCode           = 0x45534331u;

inline const RegisterList kRegisterList = {
    &ErrorSettings,
    &LocalErrorReaction,
    &SyncErrorCounterLimit,
    &TimestampObject,
};

} // namespace Identity
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
