#pragma once

#include <cstdint>
#include "tether/drives/DynaDrive/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace DynaDrive {
namespace Status {

static constexpr uint16_t ObjectIndex = 0x6041;

enum class StateOptions : uint8_t {
    NA            = 0,
    ColdStart     = 1,
    WarmStart     = 2,
    Configure     = 3,
    Calibrate     = 4,
    Standby       = 5,
    MotorOp       = 6,
    ControlOp     = 7,
    Error         = 8,
    Fatal         = 9,
    MotorPreOp    = 10,
    DeviceMissing = 11,
    Unknown       = 255,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry Entry = {
    .index = ObjectIndex,
    .subindex = 0x00,
    .name = "DynaDrive Statusword",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
};

inline const RegisterList kRegisterList = {
    &Entry,
};

// ============================================================================
// Runtime statusword decoder (not an ObjectDictionaryEntry — binary helper)
// ============================================================================

struct StatuswordDecoder {
    uint32_t raw = 0;

    explicit StatuswordDecoder(uint32_t data) : raw(data) {}

    StateOptions state() const {
        return static_cast<StateOptions>(raw & 0x0F);
    }

    uint8_t modeId() const { return static_cast<uint8_t>((raw >> 4) & 0x0F); }

    bool isError()  const { return state() == StateOptions::Error; }
    bool isFatal()  const { return state() == StateOptions::Fatal; }
    bool isControlOp() const { return state() == StateOptions::ControlOp; }
    bool isMotorOp() const { return state() == StateOptions::MotorOp; }
    bool isStandby() const { return state() == StateOptions::Standby; }

    bool hasWarningOvertemperatureBridge() const { return (raw >> 8)  & 1; }
    bool hasWarningOvertemperatureStator() const { return (raw >> 9)  & 1; }
    bool hasWarningOvertemperatureCpu()     const { return (raw >> 10) & 1; }
    bool hasErrorPdoTimeout()              const { return (raw >> 15) & 1; }
};

} // namespace Status
} // namespace DynaDrive
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
