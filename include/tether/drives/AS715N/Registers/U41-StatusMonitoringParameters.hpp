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
namespace U41 {

static constexpr uint16_t U41ObjectIndex = 0x2041; // Group U41 (Status Monitoring)

// U41.00 - MCU system status (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MCUSystemStatus = {
    .index = U41ObjectIndex,
    .subindex = 0x01,
    .name = "MCU system status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.01 - MCU fault state (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MCUFaultState = {
    .index = U41ObjectIndex,
    .subindex = 0x02,
    .name = "MCU fault state",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.04 - Encoder system status (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderSystemStatus = {
    .index = U41ObjectIndex,
    .subindex = 0x05,
    .name = "Encoder system status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.05 - Encoder fault state (read-only, U16)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderFaultState = {
    .index = U41ObjectIndex,
    .subindex = 0x06,
    .name = "Encoder fault state",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.06 - Group number of abnormal parameter (read-only, U16, 0-255)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry GroupNumberOfAbnormalParameter = {
    .index = U41ObjectIndex,
    .subindex = 0x07,
    .name = "Group number of abnormal parameter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.07 - Offset of the abnormal parameter within the parameter group (read-only, U16, 0-255)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OffsetOfAbnormalParameterWithinGroup = {
    .index = U41ObjectIndex,
    .subindex = 0x08,
    .name = "Offset of the abnormal parameter within the parameter group",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.0A - Servo Status (read-only, U16, 0-3)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ServoStatus = {
    .index = U41ObjectIndex,
    .subindex = 0x0B,
    .name = "Servo Status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.0B - Servo running mode (read-only, U16, 0-9)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ServoRunningMode = {
    .index = U41ObjectIndex,
    .subindex = 0x0C,
    .name = "Servo running mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .min_value = 0,
    .max_value = 9,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// U41.0C - Servo running time (read-only, U32, reported in 0.1s units)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ServoRunningTime = {
    .index = U41ObjectIndex,
    .subindex = 0x0D,
    .name = "Servo running time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None, // reported in 0.1s units
    .min_value = 0,
    .max_value = static_cast<int64_t>(0xFFFFFFFFu),
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &MCUSystemStatus,
    &MCUFaultState,
    &EncoderSystemStatus,
    &EncoderFaultState,
    &GroupNumberOfAbnormalParameter,
    &OffsetOfAbnormalParameterWithinGroup,
    &ServoStatus,
    &ServoRunningMode,
    &ServoRunningTime,
};

} // namespace U41
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
