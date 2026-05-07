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
namespace C00 {

static constexpr uint16_t C00ObjectIndex = 0x2000; // Group C00 (Parameters)

// Option enums for C00
enum class ControlModeOptions : uint16_t {
    // table shows 0-10 allowed; 10 = EtherCAT (default)
    Manual = 0,
    // ... other modes may be device-specific
    EtherCAT = 10,
};

enum class MotorRotatingDirectionOptions : uint16_t {
    CCW = 0,
    CW = 1,
};

enum class AutoTuningModeOptions : uint16_t {
    Manual = 0,
    Standard = 1,
    Positioning = 2,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ControlMode = {
    .index = C00ObjectIndex,
    .subindex = 0x00,
    .name = "Control mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 10,
    .options_enum = std::type_identity<ControlModeOptions>{},
    .min_value = 0,
    .max_value = 10,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorRotatingDirection = {
    .index = C00ObjectIndex,
    .subindex = 0x01,
    .name = "Motor rotating direction",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<MotorRotatingDirectionOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AutoTuningMode = {
    .index = C00ObjectIndex,
    .subindex = 0x04,
    .name = "Auto-tuning mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .options_enum = std::type_identity<AutoTuningModeOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry StiffnessLevel = {
    .index = C00ObjectIndex,
    .subindex = 0x05,
    .name = "Stiffness level",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 12,
    .min_value = 1,
    .max_value = 31,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LoadInertiaRatio = {
    .index = C00ObjectIndex,
    .subindex = 0x06,
    .name = "Load inertia ratio",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 100,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Percent,
    .min_value = 0,
    .max_value = 12000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// Additional C00 registers (from provided table)
enum class AbsoluteModeOptions : uint16_t {
    IncrementalPositionMode = 0,
    AbsolutePositionLinearMode = 1,
    AbsolutePositionLinearInfiniteMode = 2,
    AbsolutePositionSingleTurnMode = 3,
    AbsolutePositionRotationMode = 4,
    AbsoluteMechanicalSingleTurnMode = 5,
};

enum class BleederResistorSelectionOptions : uint16_t {
    InternalBleederResistor = 0,
    ExternalBleederResistor = 1,
    NoBleederResistor = 2,
    CapacitorBleederResistor = 3,
};

enum class PanelDisplayOptions : uint16_t {
    DefaultDisplay = 0,
    SpeedDisplay = 1,
    TorqueDisplay = 2,
    VoltageDisplay = 3,
    LoadRateDisplay = 4,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AbsoluteMode = {
    .index = C00ObjectIndex,
    .subindex = 0x07,
    .name = "Absolute mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<AbsoluteModeOptions>{},
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry BleederResistorSelection = {
    .index = C00ObjectIndex,
    .subindex = 0x10,
    .name = "Bleeder resistor selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<BleederResistorSelectionOptions>{},
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry BleederResistorPower = {
    .index = C00ObjectIndex,
    .subindex = 0x11,
    .name = "Bleeder resistor power",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 50,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Power_Watt,
    .min_value = 1,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry BleederResistorResistance = {
    .index = C00ObjectIndex,
    .subindex = 0x12,
    .name = "Bleeder resistor resistance",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 50,
    .unit = ::EtherCAT::ObjectDictionary::Unit_Resistance_Ohm,
    .min_value = 1,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry BleederResistorHeatDissipationCoefficient = {
    .index = C00ObjectIndex,
    .subindex = 0x13,
    .name = "Bleeder resistor heat dissipation coefficient",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 30,
    .min_value = 1,
    .max_value = 100,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry BrakeEnableSwitch = {
    .index = C00ObjectIndex,
    .subindex = 0x14,
    .name = "Brake enable switch",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PanelDisplay = {
    .index = C00ObjectIndex,
    .subindex = 0x16,
    .name = "Panel display",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<PanelDisplayOptions>{},
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SuperUser = {
    .index = C00ObjectIndex,
    .subindex = 0x31,
    .name = "Super user",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &ControlMode,
    &MotorRotatingDirection,
    &AutoTuningMode,
    &StiffnessLevel,
    &LoadInertiaRatio,
    &AbsoluteMode,
    &BleederResistorSelection,
    &BleederResistorPower,
    &BleederResistorResistance,
    &BleederResistorHeatDissipationCoefficient,
    &BrakeEnableSwitch,
    &PanelDisplay,
    &SuperUser,
};

} // namespace C00
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
