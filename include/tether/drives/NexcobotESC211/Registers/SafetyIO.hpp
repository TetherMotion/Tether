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

// ---------------------------------------------------------------------------
// 0xA008 / 0xA009: DI / DO Diagnosis Error (16-byte arrays)
// ---------------------------------------------------------------------------

static constexpr uint16_t DIDiagnosisErrorIndex = 0xA008;
static constexpr uint16_t DODiagnosisErrorIndex = 0xA009;

#define NEXCOBOT_DIAG_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = DESC " (ARRAY[0..15] OF BYTE)", \
    }

NEXCOBOT_DIAG_REG(DIDiagnosisError, DIDiagnosisErrorIndex, "DI Diagnosis Error");
NEXCOBOT_DIAG_REG(DODiagnosisError, DODiagnosisErrorIndex, "DO Diagnosis Error");

#undef NEXCOBOT_DIAG_REG

inline const RegisterList kRegisterList = {
    &RSAPVersionBuild,
    &DIDiagnosisError,
    &DODiagnosisError,
};

} // namespace SafetyIO
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
