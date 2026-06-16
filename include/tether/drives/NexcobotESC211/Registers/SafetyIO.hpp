#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace SafetyIO {

static constexpr uint16_t SafetyInputAIndex  = 0x4200;
static constexpr uint16_t SafetyInputBIndex  = 0x4201;
static constexpr uint16_t SafetyOutputIndex    = 0x4202;

// ---------------------------------------------------------------------------
// 0x4200: Safety Input A Setting (sub 1..10, SINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SafetyInputACount = {
    .index = SafetyInputAIndex,
    .subindex = 0x00,
    .name = "Safety Input A count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 10,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 10,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Safety Input A",
};

#define NEXCOBOT_DI_A_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI_##NUM##A = { \
        .index = SafetyInputAIndex, \
        .subindex = (NUM), \
        .name = "DI_" #NUM "A", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer8, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = -128, \
        .max_value = 127, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Safety digital input A channel " #NUM, \
    }

NEXCOBOT_DI_A_REG(1);
NEXCOBOT_DI_A_REG(2);
NEXCOBOT_DI_A_REG(3);
NEXCOBOT_DI_A_REG(4);
NEXCOBOT_DI_A_REG(5);
NEXCOBOT_DI_A_REG(6);
NEXCOBOT_DI_A_REG(7);
NEXCOBOT_DI_A_REG(8);
NEXCOBOT_DI_A_REG(9);
NEXCOBOT_DI_A_REG(10);

#undef NEXCOBOT_DI_A_REG

// ---------------------------------------------------------------------------
// 0x4201: Safety Input B Setting (sub 1..10, SINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SafetyInputBCount = {
    .index = SafetyInputBIndex,
    .subindex = 0x00,
    .name = "Safety Input B count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 10,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 10,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Safety Input B",
};

#define NEXCOBOT_DI_B_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI_##NUM##B = { \
        .index = SafetyInputBIndex, \
        .subindex = (NUM), \
        .name = "DI_" #NUM "B", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer8, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = -128, \
        .max_value = 127, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Safety digital input B channel " #NUM, \
    }

NEXCOBOT_DI_B_REG(1);
NEXCOBOT_DI_B_REG(2);
NEXCOBOT_DI_B_REG(3);
NEXCOBOT_DI_B_REG(4);
NEXCOBOT_DI_B_REG(5);
NEXCOBOT_DI_B_REG(6);
NEXCOBOT_DI_B_REG(7);
NEXCOBOT_DI_B_REG(8);
NEXCOBOT_DI_B_REG(9);
NEXCOBOT_DI_B_REG(10);

#undef NEXCOBOT_DI_B_REG

// ---------------------------------------------------------------------------
// 0x4202: Safety Output Setting (sub 1..16, SINT)
//   DO_1A..DO_8A (sub 1..8), DO_1B..DO_8B (sub 9..16)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SafetyOutputCount = {
    .index = SafetyOutputIndex,
    .subindex = 0x00,
    .name = "Safety Output count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 16,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 16,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Safety Output",
};

#define NEXCOBOT_DO_A_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO_##NUM##A = { \
        .index = SafetyOutputIndex, \
        .subindex = (NUM), \
        .name = "DO_" #NUM "A", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer8, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = -128, \
        .max_value = 127, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Safety digital output A channel " #NUM, \
    }

NEXCOBOT_DO_A_REG(1);
NEXCOBOT_DO_A_REG(2);
NEXCOBOT_DO_A_REG(3);
NEXCOBOT_DO_A_REG(4);
NEXCOBOT_DO_A_REG(5);
NEXCOBOT_DO_A_REG(6);
NEXCOBOT_DO_A_REG(7);
NEXCOBOT_DO_A_REG(8);

#undef NEXCOBOT_DO_A_REG

#define NEXCOBOT_DO_B_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO_##NUM##B = { \
        .index = SafetyOutputIndex, \
        .subindex = (NUM + 8), \
        .name = "DO_" #NUM "B", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer8, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = -128, \
        .max_value = 127, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Safety digital output B channel " #NUM, \
    }

NEXCOBOT_DO_B_REG(1);
NEXCOBOT_DO_B_REG(2);
NEXCOBOT_DO_B_REG(3);
NEXCOBOT_DO_B_REG(4);
NEXCOBOT_DO_B_REG(5);
NEXCOBOT_DO_B_REG(6);
NEXCOBOT_DO_B_REG(7);
NEXCOBOT_DO_B_REG(8);

#undef NEXCOBOT_DO_B_REG

inline const RegisterList kRegisterList = {
    &SafetyInputACount,
    &DI_1A, &DI_2A, &DI_3A, &DI_4A, &DI_5A,
    &DI_6A, &DI_7A, &DI_8A, &DI_9A, &DI_10A,
    &SafetyInputBCount,
    &DI_1B, &DI_2B, &DI_3B, &DI_4B, &DI_5B,
    &DI_6B, &DI_7B, &DI_8B, &DI_9B, &DI_10B,
    &SafetyOutputCount,
    &DO_1A, &DO_2A, &DO_3A, &DO_4A, &DO_5A,
    &DO_6A, &DO_7A, &DO_8A,
    &DO_1B, &DO_2B, &DO_3B, &DO_4B, &DO_5B,
    &DO_6B, &DO_7B, &DO_8B,
};

} // namespace SafetyIO
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
