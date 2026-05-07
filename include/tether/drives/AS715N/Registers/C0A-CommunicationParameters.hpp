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
namespace C0A {

static constexpr uint16_t C0AObjectIndex = 0x200A; // Group C0A (Communication Parameters)

// Baud-rate options for commissioning software
enum class CommBaudRateOptions : uint16_t {
    Baud_1200   = 0,
    Baud_2400   = 1,
    Baud_4800   = 2,
    Baud_9600   = 3,
    Baud_19200  = 4,
    Baud_38400  = 5,
    Baud_57600  = 6,
    Baud_115200 = 7,
};

// Communication format options (parity/stop bits)
enum class CommFormatOptions : uint16_t {
    NoParity_1Stop   = 0,
    OddParity_1Stop  = 1,
    EvenParity_1Stop = 2,
    NoParity_2Stop   = 3,
    OddParity_2Stop  = 4,
    EvenParity_2Stop = 5,
};

enum class StorageOptions : uint16_t {
    NoStorage = 0,
    Storage = 1,
};

// C0A.08 - Commissioning software communication station ID
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommStationID = {
    .index = C0AObjectIndex,
    .subindex = 0x08,
    .name = "Commissioning software communication station ID",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .min_value = 1,
    .max_value = 255,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C0A.09 - Commissioning software communication baud rate
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommBaudRate = {
    .index = C0AObjectIndex,
    .subindex = 0x09,
    .name = "Commissioning software communication baud rate",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 7,
    .options_enum = std::type_identity<CommBaudRateOptions>{},
    .min_value = 0,
    .max_value = 7,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::UponRepowerOn,
};

// C0A.0A - Commissioning software communication format
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommFormat = {
    .index = C0AObjectIndex,
    .subindex = 0x0A,
    .name = "Commissioning software communication format",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<CommFormatOptions>{},
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::UponRepowerOn,
};

// C0A.0B - Commissioning software communication response time (ms)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommResponseTime = {
    .index = C0AObjectIndex,
    .subindex = 0x0B,
    .name = "Commissioning software communication response time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .min_value = 1,
    .max_value = 1000,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C0A.0C - Commissioning software communication timeout (read-only)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommTimeout = {
    .index = C0AObjectIndex,
    .subindex = 0x0C,
    .name = "Commissioning software communication timeout",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 65535,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C0A.0D - Commissioning software communication storage
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommStorage = {
    .index = C0AObjectIndex,
    .subindex = 0x0D,
    .name = "Commissioning software communication storage",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 1,
    .options_enum = std::type_identity<StorageOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

// C0A.0E - Commissioning software data format
enum class CommDataFormatOptions : uint16_t {
    Low16BeforeHigh16 = 0,
    High16BeforeLow16 = 1,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommDataFormat = {
    .index = C0AObjectIndex,
    .subindex = 0x0E,
    .name = "Commissioning software data format",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<CommDataFormatOptions>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::Drives::Registers::ModificationMode::AtStop,
    .effective_time = ::EtherCAT::Drives::Registers::EffectiveTime::Immediately,
};

inline const ::EtherCAT::Drives::Registers::RegisterList kRegisterList = {
    &CommStationID,    // 0x08
    &CommBaudRate,      // 0x09
    &CommFormat,        // 0x0A
    &CommResponseTime,  // 0x0B
    &CommTimeout,       // 0x0C
    &CommStorage,       // 0x0D
    &CommDataFormat,    // 0x0E
};

} // namespace C0A
} // namespace AS715N
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
