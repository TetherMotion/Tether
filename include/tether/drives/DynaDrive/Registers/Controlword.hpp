#pragma once

#include <cstdint>
#include "tether/drives/DynaDrive/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace DynaDrive {
namespace Controlword {

static constexpr uint16_t ObjectIndex = 0x6040;

enum class Options : uint16_t {
    WarmReset                = 0x01,
    ClearErrorsToMotorOp     = 0x02,
    StandbyToConfigure       = 0x03,
    ConfigureToStandby       = 0x04,
    CalibrateToConfigure     = 0x05,
    ConfigureToCalibrate     = 0x06,
    MotorOpToStandby         = 0x07,
    StandbyToMotorPreOp      = 0x08,
    ControlOpToMotorOp       = 0x09,
    MotorOpToControlOp       = 0x0A,
    ControlOpToStandby       = 0x0B,
    ClearErrorsToStandby     = 0x0C,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry Entry = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "DynaDrive Controlword",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<Options>{},
    .min_value = 0,
    .max_value = 0x0C,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Entry,
};

} // namespace Controlword
} // namespace DynaDrive
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
