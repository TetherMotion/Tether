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
namespace U42 {

static constexpr uint16_t U42ObjectIndex = 0x2042; // Group U42 (Version Parameters)

// U42.00 - ARM version (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ARMVersion = {
    .index = U42ObjectIndex,
    .subindex = 0x01,
    .name = "ARM version",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.02 - Encoder version (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderVersion = {
    .index = U42ObjectIndex,
    .subindex = 0x03,
    .name = "Encoder version",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.03 - ARM-based machine (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ARMBasedMachine = {
    .index = U42ObjectIndex,
    .subindex = 0x04,
    .name = "ARM-based machine",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.05 - Internal software version (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InternalSoftwareVersion = {
    .index = U42ObjectIndex,
    .subindex = 0x06,
    .name = "Internal software version",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.0A - EtherCAT CoE version (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATCoEVersion = {
    .index = U42ObjectIndex,
    .subindex = 0x08,
    .name = "EtherCAT CoE version",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.0B - EtherCAT XML version (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EtherCATXMLVersion = {
    .index = U42ObjectIndex,
    .subindex = 0x0C,
    .name = "EtherCAT XML version",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.10 - Drive model (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DriveModel = {
    .index = U42ObjectIndex,
    .subindex = 0x11,
    .name = "Drive model",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.11 - Motor model (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorModel = {
    .index = U42ObjectIndex,
    .subindex = 0x12,
    .name = "Motor model",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.12 - Encoder model (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderModel = {
    .index = U42ObjectIndex,
    .subindex = 0x13,
    .name = "Encoder model",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.13 - Power supply unit model identification (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PowerSupplyUnitModelIdentification = {
    .index = U42ObjectIndex,
    .subindex = 0x14,
    .name = "Power supply unit model identification",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.14 - Inverter model identification 1 (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InverterModelIdentification1 = {
    .index = U42ObjectIndex,
    .subindex = 0x15,
    .name = "Inverter model identification 1",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.15 - Inverter model identification 2 (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InverterModelIdentification2 = {
    .index = U42ObjectIndex,
    .subindex = 0x16,
    .name = "Inverter model identification 2",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U42.16 - Servo version (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ServoVersion = {
    .index = U42ObjectIndex,
    .subindex = 0x17,
    .name = "Servo version",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &ARMVersion,
    &EncoderVersion,
    &ARMBasedMachine,
    &InternalSoftwareVersion,
    &EtherCATCoEVersion,
    &EtherCATXMLVersion,
    &DriveModel,
    &MotorModel,
    &EncoderModel,
    &PowerSupplyUnitModelIdentification,
    &InverterModelIdentification1,
    &InverterModelIdentification2,
    &ServoVersion,
};

} // namespace U42
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
