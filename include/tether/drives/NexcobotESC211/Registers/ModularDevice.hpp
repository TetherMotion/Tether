#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace ModularDevice {

static constexpr uint16_t ModularDeviceProfileIndex    = 0xF000;
static constexpr uint16_t ModuleProfileListIndex       = 0xF010;

// ---------------------------------------------------------------------------
// 0xF000: Modular Device Profile
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModularDeviceProfileCount = {
    .index = ModularDeviceProfileIndex,
    .subindex = 0x00,
    .name = "Modular Device Profile count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Modular Device Profile",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry IndexDistance = {
    .index = ModularDeviceProfileIndex,
    .subindex = 0x01,
    .name = "Index distance",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Index distance between modules",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaxNumberOfModules = {
    .index = ModularDeviceProfileIndex,
    .subindex = 0x02,
    .name = "Maximum number of modules",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum number of modules",
};

// ---------------------------------------------------------------------------
// 0xF010: Module Profile List
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModuleProfileListCount = {
    .index = ModuleProfileListIndex,
    .subindex = 0x00,
    .name = "Module Profile List count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Module Profile List",
};

#define NEXCOBOT_MODULE_PROFILE_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModuleProfile_##NUM = { \
        .index = ModuleProfileListIndex, \
        .subindex = (NUM), \
        .name = "Module Profile " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Module profile entry " #NUM, \
    }

NEXCOBOT_MODULE_PROFILE_REG(1);
NEXCOBOT_MODULE_PROFILE_REG(2);

#undef NEXCOBOT_MODULE_PROFILE_REG

inline const RegisterList kRegisterList = {
    &ModularDeviceProfileCount,
    &IndexDistance,
    &MaxNumberOfModules,
    &ModuleProfileListCount,
    &ModuleProfile_1,
    &ModuleProfile_2,
};

} // namespace ModularDevice
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
