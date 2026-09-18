#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace SafetyIO {

// ESI v0.9: the 0x4200-range no longer carries Safety Input/Output channel
// settings (the old 0x4200/0x4201/0x4202 records were removed).  The only
// remaining object is 0x4200 "RSAP version build" — a 16-byte string.
static constexpr uint16_t RSAPVersionBuildIndex = 0x4200;

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPVersionBuild = {
    .index = RSAPVersionBuildIndex, .subindex = 0x00,
    .name = "RSAP version build",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "RSAP version build (STRING(16))",
};

inline const RegisterList kRegisterList = {
    &RSAPVersionBuild,
};

} // namespace SafetyIO
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
