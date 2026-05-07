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
namespace C03 {

static constexpr uint16_t C03ObjectIndex = 0x2003; // Group C03 (Instruction Parameters)

// Register entries for C03
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedReference = {
    .index = C03ObjectIndex,
    .subindex = 0x21,
    .name = "Speed reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 100,
    .unit = ::EtherCAT::ObjectDictionary::Unit_RPM,
    .min_value = -8000,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AccelerationRate = {
    .index = C03ObjectIndex,
    .subindex = 0x22,
    .name = "Acceleration rate",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 10,
    .min_value = 0,
    .max_value = 3600000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DecelerationRate = {
    .index = C03ObjectIndex,
    .subindex = 0x24,
    .name = "Deceleration rate",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 10,
    .min_value = 0,
    .max_value = 3600000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InternalPositiveSpeedLimit = {
    .index = C03ObjectIndex,
    .subindex = 0x27,
    .name = "Internal positive speed limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 6000,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InternalNegativeSpeedLimit = {
    .index = C03ObjectIndex,
    .subindex = 0x28,
    .name = "Internal negative speed limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 6000,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedReachThreshold = {
    .index = C03ObjectIndex,
    .subindex = 0x2B,
    .name = "Speed reach threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedSynchronizationThreshold = {
    .index = C03ObjectIndex,
    .subindex = 0x2C,
    .name = "Speed synchronization threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedRotationThreshold = {
    .index = C03ObjectIndex,
    .subindex = 0x2D,
    .name = "Speed rotation threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 20,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ZeroSpeedOutputThreshold = {
    .index = C03ObjectIndex,
    .subindex = 0x2E,
    .name = "Zero speed output threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueReference = {
    .index = C03ObjectIndex,
    .subindex = 0x41,
    .name = "Torque reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .unit = Unit_Percent, // 0.1% units
    .min_value = -4000,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InternalPositiveTorqueLimit = {
    .index = C03ObjectIndex,
    .subindex = 0x43,
    .name = "Internal positive torque limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .unit = Unit_Percent, // 0.1% units
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InternalNegativeTorqueLimit = {
    .index = C03ObjectIndex,
    .subindex = 0x44,
    .name = "Internal negative torque limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .unit = Unit_Percent, // 0.1% units
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositiveSpeedLimitInTorqueMode = {
    .index = C03ObjectIndex,
    .subindex = 0x47,
    .name = "Positive speed limit in torque mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NegativeSpeedLimitInTorqueMode = {
    .index = C03ObjectIndex,
    .subindex = 0x48,
    .name = "Negative speed limit in torque mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .unit = Unit_RPM,
    .min_value = 0,
    .max_value = 8000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ReferenceValueForTorqueReach = {
    .index = C03ObjectIndex,
    .subindex = 0x49,
    .name = "Reference value for torque reach",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ValidValueForTorqueReached = {
    .index = C03ObjectIndex,
    .subindex = 0x4A,
    .name = "Valid value for torque reached",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 200,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry InvalidValueForTorqueReached = {
    .index = C03ObjectIndex,
    .subindex = 0x4B,
    .name = "Invalid value for torque reached",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .unit = Unit_Percent,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &SpeedReference,
    &AccelerationRate,
    &DecelerationRate,
    &InternalPositiveSpeedLimit,
    &InternalNegativeSpeedLimit,
    &SpeedReachThreshold,
    &SpeedSynchronizationThreshold,
    &SpeedRotationThreshold,
    &ZeroSpeedOutputThreshold,
    &TorqueReference,
    &InternalPositiveTorqueLimit,
    &InternalNegativeTorqueLimit,
    &PositiveSpeedLimitInTorqueMode,
    &NegativeSpeedLimitInTorqueMode,
    &ReferenceValueForTorqueReach,
    &ValidValueForTorqueReached,
    &InvalidValueForTorqueReached,
};

} // namespace C03
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
