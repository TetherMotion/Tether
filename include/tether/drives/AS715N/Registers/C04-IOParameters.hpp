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
namespace C04 {

static constexpr uint16_t C04ObjectIndex = 0x2004; // Group C04 (I/O Parameters)

// DI function selection options (common for DI1/DI2)
enum class DigitalInputFunctionOptions : uint16_t {
    NoDefinition = 0,
    S_ON = 1,
    FaultReset = 2,
    EmergencyStop = 4,
    HomeSwitch = 5,
    ForwardOvertravel = 6,
    ReverseOvertravel = 7,
    Probe1 = 30,
    Probe2 = 31,
};

enum class DigitalInputLogicOptions : uint16_t {
    ActiveLow = 0,
    ActiveHigh = 1,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI1FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x00,
    .name = "DI1 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 6,
    .options_enum = std::type_identity<DigitalInputFunctionOptions>{},
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI1LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x01,
    .name = "DI1 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalInputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI1FilterTime = {
    .index = C04ObjectIndex,
    .subindex = 0x02,
    .name = "DI1 filter time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 150,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI2FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x04,
    .name = "DI2 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 7,
    .options_enum = std::type_identity<DigitalInputFunctionOptions>{},
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI2LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x05,
    .name = "DI2 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalInputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI2FilterTime = {
    .index = C04ObjectIndex,
    .subindex = 0x06,
    .name = "DI2 filter time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 150,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI3FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x08,
    .name = "DI3 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 5,
    .options_enum = std::type_identity<DigitalInputFunctionOptions>{},
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI3LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x09,
    .name = "DI3 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalInputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI3FilterTime = {
    .index = C04ObjectIndex,
    .subindex = 0x0A,
    .name = "DI3 filter time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 150,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI4FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x0C,
    .name = "DI4 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 31,
    .options_enum = std::type_identity<DigitalInputFunctionOptions>{},
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI4LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x0D,
    .name = "DI4 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalInputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI4FilterTime = {
    .index = C04ObjectIndex,
    .subindex = 0x0E,
    .name = "DI4 filter time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 150,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI5FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x10,
    .name = "DI5 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 30,
    .options_enum = std::type_identity<DigitalInputFunctionOptions>{},
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI5LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x11,
    .name = "DI5 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalInputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DI5FilterTime = {
    .index = C04ObjectIndex,
    .subindex = 0x12,
    .name = "DI5 filter time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 150,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// Digital Output function / logic enums
enum class DigitalOutputFunctionOptions : uint16_t {
    NoDefinition = 0,
    ServoReady = 1,
    MotorRotation = 2,
    BrakeOutput = 9,
    Alarm = 10,
    Fault = 11,
    EDM_SafetyState = 32,
};

enum class DigitalOutputLogicOptions : uint16_t {
    ActiveLow = 0,
    ActiveHigh = 1,
};

// DO registers
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO1FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x30,
    .name = "DO1 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .options_enum = std::type_identity<DigitalOutputFunctionOptions>{},
    .min_value = 0,
    .max_value = 20,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO1LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x31,
    .name = "DO1 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalOutputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO2FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x32,
    .name = "DO2 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 4,
    .options_enum = std::type_identity<DigitalOutputFunctionOptions>{},
    .min_value = 0,
    .max_value = 20,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO2LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x33,
    .name = "DO2 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalOutputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO3FunctionSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x34,
    .name = "DO3 function selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3,
    .options_enum = std::type_identity<DigitalOutputFunctionOptions>{},
    .min_value = 0,
    .max_value = 20,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DO3LogicSelection = {
    .index = C04ObjectIndex,
    .subindex = 0x35,
    .name = "DO3 logic selection",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DigitalOutputLogicOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &DI1FunctionSelection,
    &DI1LogicSelection,
    &DI1FilterTime,
    &DI2FunctionSelection,
    &DI2LogicSelection,
    &DI2FilterTime,
    &DI3FunctionSelection,
    &DI3LogicSelection,
    &DI3FilterTime,
    &DI4FunctionSelection,
    &DI4LogicSelection,
    &DI4FilterTime,
    &DI5FunctionSelection,
    &DI5LogicSelection,
    &DI5FilterTime,
    &DO1FunctionSelection,
    &DO1LogicSelection,
    &DO2FunctionSelection,
    &DO2LogicSelection,
    &DO3FunctionSelection,
    &DO3LogicSelection,
};

} // namespace C04
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
