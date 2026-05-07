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
namespace R20 {

static constexpr uint16_t kIndex = 0x2020; // R20 Motor Parameters

// R20.00 - Motor model
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorModel = {
    .index = kIndex,
    .subindex = 0x01, // document index 01h => parameter R20.00
    .name = "Motor model",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 20000,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

// R20.22 - Encoder type (read-only)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderType = {
    .index = kIndex,
    .subindex = 0x23, // document index 23h => parameter R20.22
    .name = "Encoder type",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::UponRepowerOn,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &MotorModel,
    &EncoderType,
};

} // namespace R20
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
