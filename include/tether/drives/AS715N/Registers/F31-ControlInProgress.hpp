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
namespace F31 {

static constexpr uint16_t F31ObjectIndex = 0x2031; // Group F31 (Parameters of Control in Progress)

// Option enums for F31 subindexes
enum class FaultResetOptions : uint16_t {
    Inactive = 0,
    Reset = 1,
};

enum class SoftwareResetOptions : uint16_t {
    Inactive = 0,
    Reset = 1,
};

enum class ParameterInitializationOptions : uint16_t {
    Inactive = 0,
    RestoreParameters = 1,
    RestoreObjectDictionary = 2,
};

enum class DriveMotorParameterResetOptions : uint16_t {
    Inactive = 0,
    FactoryResetDriveParameters = 1,
    FactoryResetMotorParameters = 2,
};

enum class FaultRecordInitializationOptions : uint16_t {
    Inactive = 0,
    FaultRecordClearing = 1,
};

enum class EncoderDataResetOptions : uint16_t {
    Inactive = 0,
    ReadEncoder = 1,
    WriteEncoder = 2,
    ResetEncoderFault = 3,
    ResetEncoderFaultAndMultiTurnData = 4,
    OperationFailed = 16,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FaultReset = {
    .index = F31ObjectIndex,
    .subindex = 0x01,
    .name = "Fault reset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<FaultResetOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SoftwareReset = {
    .index = F31ObjectIndex,
    .subindex = 0x02,
    .name = "Software reset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<SoftwareResetOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ParameterInitialization = {
    .index = F31ObjectIndex,
    .subindex = 0x03,
    .name = "Parameter initialization",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<ParameterInitializationOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DriveMotorParameterReset = {
    .index = F31ObjectIndex,
    .subindex = 0x04,
    .name = "Drive motor parameter reset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<DriveMotorParameterResetOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FaultRecordInitialization = {
    .index = F31ObjectIndex,
    .subindex = 0x05,
    .name = "Fault record initialization",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<FaultRecordInitializationOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderDataReset = {
    .index = F31ObjectIndex,
    .subindex = 0x11,
    .name = "Encoder data reset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<EncoderDataResetOptions>{},
    .min_value = 0,
    .max_value = 31,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &FaultReset,
    &SoftwareReset,
    &ParameterInitialization,
    &DriveMotorParameterReset,
    &FaultRecordInitialization,
    &EncoderDataReset,
};

} // namespace F31
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
