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
namespace C05 {

static constexpr uint16_t C05ObjectIndex = 0x2005; // Group C05 (Stop Mode)

enum class StopModeAtOvertravelOptions : uint16_t {
    CoastToStop_KeepDeenergized = 0,
    StopAtZeroSpeed_KeepPositionLock = 1,
    StopAtZeroSpeed_KeepDeenergized = 2,
    RampToStop6085_KeepDeenergized = 3,
    RampToStop6085_KeepPositionLock = 4,
    DynamicBrakingStop_KeepDeenergized = 5,
    DynamicBrakingStop_KeepDynamicBraking = 6,
    NotRespondingToOvertravel = 7,
};

enum class StopModeAtFaultOptions : uint16_t {
    CoastToStop_KeepDeenergized = 0,
    DynamicBrakingStop_KeepDeenergized = 1,
    DynamicBrakingStop_KeepDynamicBraking = 2,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry StopModeAtOvertravel = {
    .index = C05ObjectIndex,
    .subindex = 0x02,
    .name = "Stop mode at overtravel",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .options_enum = std::type_identity<StopModeAtOvertravelOptions>{},
    .min_value = 0,
    .max_value = 7,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry StopModeAtNo1Fault = {
    .index = C05ObjectIndex,
    .subindex = 0x03,
    .name = "Stop mode at No. 1 fault",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 2,
    .options_enum = std::type_identity<StopModeAtFaultOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LimitForStopAtEmergencyStopTorque = {
    .index = C05ObjectIndex,
    .subindex = 0x0C,
    .name = "Limit for stop at emergency-stop torque",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1000,
    .unit = Unit_Percent, // 0.1%
    .min_value = 0,
    .max_value = 3000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaximumDowntime = {
    .index = C05ObjectIndex,
    .subindex = 0x0D,
    .name = "Maximum downtime",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10000,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DelayFromBrakeCloseToMotorDeenergized = {
    .index = C05ObjectIndex,
    .subindex = 0x10,
    .name = "Delay from brake close to motor de-energized",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedThresholdAtBrakeClosing = {
    .index = C05ObjectIndex,
    .subindex = 0x11,
    .name = "Speed threshold at brake closing",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 30,
    .unit = Unit_RPM,
    .min_value = 10,
    .max_value = 3000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &StopModeAtOvertravel,
    &StopModeAtNo1Fault,
    &LimitForStopAtEmergencyStopTorque,
    &MaximumDowntime,
    &DelayFromBrakeCloseToMotorDeenergized,
    &SpeedThresholdAtBrakeClosing,
};

} // namespace C05
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
