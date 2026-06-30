#pragma once

#include <cstdint>
#include "tether/drives/RP20/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace RP20 {
namespace ModuleInfo {

// 0xA000 - Module diagnosis/info (slot-dependent)
static constexpr uint16_t kDiagnosisIndex = 0xA000;

constexpr RegisterEntry DiagnosisCount = {
    .index = kDiagnosisIndex, .subindex = 0x00,
    .name = "Number of Diagnosis",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 3, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 3,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of module diagnosis entries",
};

constexpr RegisterEntry ModuleID = {
    .index = kDiagnosisIndex, .subindex = 0x01,
    .name = "Module ID",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Module identifier",
};

constexpr RegisterEntry ModuleSWVersion = {
    .index = kDiagnosisIndex, .subindex = 0x02,
    .name = "Module SW Version",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Module software version",
};

constexpr RegisterEntry ModuleHWVersion = {
    .index = kDiagnosisIndex, .subindex = 0x03,
    .name = "Module HW Version",
    .data_type = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Module hardware version",
};

static const RegisterList kRegisterList = {
    &DiagnosisCount, &ModuleID, &ModuleSWVersion, &ModuleHWVersion,
};

} // namespace ModuleInfo
} // namespace RP20
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
