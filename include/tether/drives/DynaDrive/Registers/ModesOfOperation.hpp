#pragma once

#include <cstdint>
#include "tether/drives/DynaDrive/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace DynaDrive {

static constexpr uint16_t ModesOfOperationObjectIndex = 0x6060;

enum class DynaDriveModeOptions : uint16_t {
    Freeze                  = 1,
    Disable                 = 2,
    Current                 = 3,
    MotorPosition           = 4,
    MotorVelocity           = 5,
    GearPosition            = 6,
    GearVelocity            = 7,
    JointPosition           = 8,
    JointVelocity           = 9,
    JointTorque             = 10,
    JointPositionVelocity   = 11,
    JointPositionVelocityTorque = 12,
    JointPositionVelocityTorquePidGains = 13,
    JointPositionTorque     = 16,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModesOfOperation = {
    .index = ModesOfOperationObjectIndex,
    .subindex = 0x00,
    .name = "Modes of Operation",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = static_cast<uint32_t>(DynaDriveModeOptions::Freeze),
    .unit = Unit_None,
    .options_enum = std::type_identity<DynaDriveModeOptions>{},
    .min_value = 1,
    .max_value = 16,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &ModesOfOperation,
};

} // namespace DynaDrive
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
