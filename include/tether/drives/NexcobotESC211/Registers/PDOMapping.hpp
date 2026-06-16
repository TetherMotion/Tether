#pragma once

#include <cstdint>
#include <array>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace PDOMapping {

// ---------------------------------------------------------------------------
// RxPDO Mapping Objects
// ---------------------------------------------------------------------------

static constexpr uint16_t RxPDOMapFSOEIndex        = 0x1600;
static constexpr uint16_t RxPDOMapIndex            = 0x1601;
static constexpr uint16_t RxPDOMapFSoE0Index       = 0x1610;
static constexpr uint16_t RxPDOMapFSoE7Index       = 0x1617;

// 0x1600: RxPDO-Map_FSOE (16 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOMapFSOECount = {
    .index = RxPDOMapFSOEIndex,
    .subindex = 0x00,
    .name = "RxPDO-Map_FSOE number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 16,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 16,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in RxPDO-Map_FSOE (0x1600)",
};

// 0x1601: RxPDO-Map (8 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOMapCount = {
    .index = RxPDOMapIndex,
    .subindex = 0x00,
    .name = "RxPDO-Map number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 8,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 8,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in RxPDO-Map (0x1601)",
};

// 0x1610-0x1617: RxPDO-Map-FSoE0..FSoE7 (18 mapping entries each)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOMapFSoECount = {
    .index = RxPDOMapFSoE0Index,
    .subindex = 0x00,
    .name = "RxPDO-Map-FSoE number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 18,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 18,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in RxPDO-Map-FSoE (0x1610-0x1617)",
};

// Subindex helpers for RxPDO mapping entries (each entry is Unsigned32)
static constexpr uint8_t kRxPDOMapMaxEntriesFSOE   = 16;
static constexpr uint8_t kRxPDOMapMaxEntries       = 8;
static constexpr uint8_t kRxPDOMapMaxEntriesFSoE = 18;

// ---------------------------------------------------------------------------
// TxPDO Mapping Objects
// ---------------------------------------------------------------------------

static constexpr uint16_t TxPDOMapIndex            = 0x1A01;
static constexpr uint16_t TxPDORSAPInfoIndex       = 0x1A02;
static constexpr uint16_t TxPDOMapFSoE0Index       = 0x1A10;
static constexpr uint16_t TxPDOMapFSoE7Index       = 0x1A17;

// 0x1A01: TxPDO-Map (8 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOMapCount = {
    .index = TxPDOMapIndex,
    .subindex = 0x00,
    .name = "TxPDO-Map number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 8,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 8,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-Map (0x1A01)",
};

// 0x1A02: TxPDO-RSAP-Info (32 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDORSAPInfoCount = {
    .index = TxPDORSAPInfoIndex,
    .subindex = 0x00,
    .name = "TxPDO-RSAP-Info number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 32,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-RSAP-Info (0x1A02)",
};

// 0x1A10-0x1A17: TxPDO-Map-FSoE0..FSoE7 (18 mapping entries each)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOMapFSoECount = {
    .index = TxPDOMapFSoE0Index,
    .subindex = 0x00,
    .name = "TxPDO-Map-FSoE number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 18,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 18,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-Map-FSoE (0x1A10-0x1A17)",
};

// Subindex helpers for TxPDO mapping entries
static constexpr uint8_t kTxPDOMapMaxEntries       = 8;
static constexpr uint8_t kTxPDORSAPInfoMaxEntries    = 32;
static constexpr uint8_t kTxPDOMapMaxEntriesFSoE     = 18;

// All PDO mapping object indices in arrays for iteration
static constexpr std::array<uint16_t, 8> RxPDOMapFSoEIndices = {
    0x1610, 0x1611, 0x1612, 0x1613, 0x1614, 0x1615, 0x1616, 0x1617
};

static constexpr std::array<uint16_t, 8> TxPDOMapFSoEIndices = {
    0x1A10, 0x1A11, 0x1A12, 0x1A13, 0x1A14, 0x1A15, 0x1A16, 0x1A17
};

inline const RegisterList kRegisterList = {
    &RxPDOMapFSOECount,
    &RxPDOMapCount,
    &RxPDOMapFSoECount,
    &TxPDOMapCount,
    &TxPDORSAPInfoCount,
    &TxPDOMapFSoECount,
};

} // namespace PDOMapping
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
