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
static constexpr uint16_t RxPDOMapFSoE31Index      = 0x162F;

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

// 0x1601: RxPDO-Map (2 mapping entries: OutputCounter + SAFE_DO)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOMapCount = {
    .index = RxPDOMapIndex,
    .subindex = 0x00,
    .name = "RxPDO-Map number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in RxPDO-Map (0x1601)",
};

// 0x1610-0x162F: RxPDO-Map-FSoE0..FSoE31 (2 mapping entries each:
// 0x6000:1 (248b) + 0x6000:2 (96b) = 344b / 43B per ESI v0.9)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RxPDOMapFSoECount = {
    .index = RxPDOMapFSoE0Index,
    .subindex = 0x00,
    .name = "RxPDO-Map-FSoE number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in RxPDO-Map-FSoE (0x1610-0x162F)",
};

// Subindex helpers for RxPDO mapping entries (each entry is Unsigned32)
static constexpr uint8_t kRxPDOMapMaxEntriesFSOE   = 16;
static constexpr uint8_t kRxPDOMapMaxEntries       = 2;
static constexpr uint8_t kRxPDOMapMaxEntriesFSoE   = 2;

// ---------------------------------------------------------------------------
// TxPDO Mapping Objects
// ---------------------------------------------------------------------------

static constexpr uint16_t TxPDOMapFSOEIndex        = 0x1A00;
static constexpr uint16_t TxPDOMapIndex            = 0x1A01;
static constexpr uint16_t TxPDORSAPInfoIndex       = 0x1A02;
static constexpr uint16_t TxPDORSAPDebugIndex     = 0x1A03;
static constexpr uint16_t TxPDOMapFSoE0Index       = 0x1A10;
static constexpr uint16_t TxPDOMapFSoE31Index      = 0x1A2F;

// 0x1A00: TxPDO-Map-FSoE (16 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOMapFSOECount = {
    .index = TxPDOMapFSOEIndex,
    .subindex = 0x00,
    .name = "TxPDO-Map-FSoE number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 16,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 16,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-Map-FSoE (0x1A00)",
};

// 0x1A01: TxPDO-Map (7 mapping entries: InputCounter, SAFE_DI,
// Power_Status, DO_Monitor, DO_Value(Actual), DI_Value, DO_Command)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOMapCount = {
    .index = TxPDOMapIndex,
    .subindex = 0x00,
    .name = "TxPDO-Map number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 7,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 7,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-Map (0x1A01)",
};

// 0x1A02: TxPDO-RSAP-Info (46 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDORSAPInfoCount = {
    .index = TxPDORSAPInfoIndex,
    .subindex = 0x00,
    .name = "TxPDO-RSAP-Info number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 46,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 46,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-RSAP-Info (0x1A02)",
};

// 0x1A03: TxPDO-RSAP-Debug (104 mapping entries)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDORSAPDebugCount = {
    .index = TxPDORSAPDebugIndex,
    .subindex = 0x00,
    .name = "TxPDO-RSAP-Debug number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 104,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 104,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-RSAP-Debug (0x1A03)",
};

// 0x1A10-0x1A2F: TxPDO-Map-FSoE0..FSoE31 (2 mapping entries each:
// 0x7000:1 (248b) + 0x7000:2 (96b) = 344b / 43B per ESI v0.9)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TxPDOMapFSoECount = {
    .index = TxPDOMapFSoE0Index,
    .subindex = 0x00,
    .name = "TxPDO-Map-FSoE number of entries",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of mapped objects in TxPDO-Map-FSoE (0x1A10-0x1A2F)",
};

// Subindex helpers for TxPDO mapping entries
static constexpr uint8_t kTxPDOMapMaxEntriesFSOE     = 16;
static constexpr uint8_t kTxPDOMapMaxEntries         = 7;
static constexpr uint8_t kTxPDORSAPInfoMaxEntries    = 46;
static constexpr uint8_t kTxPDORSAPDebugMaxEntries   = 104;
static constexpr uint8_t kTxPDOMapMaxEntriesFSoE     = 2;

// All PDO mapping object indices in arrays for iteration
// (ESI v0.9 defines FSoE0..FSoE31 maps: 0x1610-0x162F / 0x1A10-0x1A2F)
static constexpr std::array<uint16_t, 32> RxPDOMapFSoEIndices = [] {
    std::array<uint16_t, 32> a{};
    for (uint16_t i = 0; i < 32; ++i) a[i] = static_cast<uint16_t>(0x1610 + i);
    return a;
}();

static constexpr std::array<uint16_t, 32> TxPDOMapFSoEIndices = [] {
    std::array<uint16_t, 32> a{};
    for (uint16_t i = 0; i < 32; ++i) a[i] = static_cast<uint16_t>(0x1A10 + i);
    return a;
}();

inline const RegisterList kRegisterList = {
    &RxPDOMapFSOECount,
    &RxPDOMapCount,
    &RxPDOMapFSoECount,
    &TxPDOMapFSOECount,
    &TxPDOMapCount,
    &TxPDORSAPInfoCount,
    &TxPDORSAPDebugCount,
    &TxPDOMapFSoECount,
};

} // namespace PDOMapping
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
