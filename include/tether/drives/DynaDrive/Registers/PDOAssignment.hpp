#pragma once

#include <cstdint>
#include "tether/drives/DynaDrive/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace DynaDrive {

static constexpr uint16_t SM2PDOAssignmentObjectIndex = 0x1C12; // RxPDO assignment
static constexpr uint16_t SM3PDOAssignmentObjectIndex = 0x1C13; // TxPDO assignment

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2PDOCount = {
    .index = SM2PDOAssignmentObjectIndex,
    .subindex = 0x00,
    .name = "SM2 PDO assignment count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2PDO1 = {
    .index = SM2PDOAssignmentObjectIndex,
    .subindex = 0x01,
    .name = "SM2 PDO 1",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0x1603,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3PDOCount = {
    .index = SM3PDOAssignmentObjectIndex,
    .subindex = 0x00,
    .name = "SM3 PDO assignment count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3PDO1 = {
    .index = SM3PDOAssignmentObjectIndex,
    .subindex = 0x01,
    .name = "SM3 PDO 1",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0x1A04,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ModificationMode::AtStop,
    .effective_time = EffectiveTime::UponRepowerOn,
};

static constexpr uint16_t RxPDOModuleIdObjectIndex = 0xF030;

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOModuleId = {
    .index = RxPDOModuleIdObjectIndex,
    .subindex = 0x01,
    .name = "RxPDO Module ID",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x00519800,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &SM2PDOCount,
    &SM2PDO1,
    &SM3PDOCount,
    &SM3PDO1,
    &RxPDOModuleId,
};

} // namespace DynaDrive
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
