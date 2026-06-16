#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace RSAPMonitoring {

static constexpr uint16_t RSAPTCP1MonitoringVelocityIndex = 0x4100;
static constexpr uint16_t RSAPCalculateTCP1Index          = 0x4101;
static constexpr uint16_t RSAPCalculateTCPForceIndex    = 0x4108;

// ---------------------------------------------------------------------------
// 0x4100: RSAP Calculate TCP1 Monitoring Velocity
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1MonitoringVelocity = {
    .index = RSAPTCP1MonitoringVelocityIndex,
    .subindex = 0x00,
    .name = "RSAP Calculate TCP1 Monitoring Velocity",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "TCP1 monitoring velocity",
};

// ---------------------------------------------------------------------------
// 0x4101: RSAP Calculate TCP1 (sub 1..6, DINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1Count = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x00,
    .name = "RSAP Calculate TCP1 count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 6,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 6,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for RSAP Calculate TCP1",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1PositionX = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x01,
    .name = "TCP Position X",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "TCP position X coordinate",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1PositionY = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x02,
    .name = "TCP Position Y",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "TCP position Y coordinate",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1Axis3LeverArm = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x03,
    .name = "Axis3 Lever Arm Length",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Axis 3 lever arm length",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1Axis4LeverArm = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x04,
    .name = "Axis4 Lever Arm Length",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Axis 4 lever arm length",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1Axis5LeverArm = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x05,
    .name = "Axis5 Lever Arm Length",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Axis 5 lever arm length",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP1Axis6LeverArm = {
    .index = RSAPCalculateTCP1Index,
    .subindex = 0x06,
    .name = "Axis6 Lever Arm Length",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Axis 6 lever arm length",
};

// ---------------------------------------------------------------------------
// 0x4108: RSAP Calculate TCP Force (sub 1..10, DINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCPForceCount = {
    .index = RSAPCalculateTCPForceIndex,
    .subindex = 0x00,
    .name = "RSAP Calculate TCP Force count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 10,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 10,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for RSAP Calculate TCP Force",
};

#define NEXCOBOT_TCP_FORCE_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSAPTCP##NUM##Force = { \
        .index = RSAPCalculateTCPForceIndex, \
        .subindex = (NUM), \
        .name = "TCP" #NUM " Force", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = static_cast<int64_t>(-0x80000000LL), \
        .max_value = 0x7FFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "TCP " #NUM " force value", \
    }

NEXCOBOT_TCP_FORCE_REG(1);
NEXCOBOT_TCP_FORCE_REG(2);
NEXCOBOT_TCP_FORCE_REG(3);
NEXCOBOT_TCP_FORCE_REG(4);
NEXCOBOT_TCP_FORCE_REG(5);
NEXCOBOT_TCP_FORCE_REG(6);
NEXCOBOT_TCP_FORCE_REG(7);
NEXCOBOT_TCP_FORCE_REG(8);
NEXCOBOT_TCP_FORCE_REG(9);
NEXCOBOT_TCP_FORCE_REG(10);

#undef NEXCOBOT_TCP_FORCE_REG

inline const RegisterList kRegisterList = {
    &RSAPTCP1MonitoringVelocity,
    &RSAPTCP1Count,
    &RSAPTCP1PositionX,
    &RSAPTCP1PositionY,
    &RSAPTCP1Axis3LeverArm,
    &RSAPTCP1Axis4LeverArm,
    &RSAPTCP1Axis5LeverArm,
    &RSAPTCP1Axis6LeverArm,
    &RSAPTCPForceCount,
    &RSAPTCP1Force,
    &RSAPTCP2Force,
    &RSAPTCP3Force,
    &RSAPTCP4Force,
    &RSAPTCP5Force,
    &RSAPTCP6Force,
    &RSAPTCP7Force,
    &RSAPTCP8Force,
    &RSAPTCP9Force,
    &RSAPTCP10Force,
};

} // namespace RSAPMonitoring
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
