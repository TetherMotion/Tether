#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <magic_enum/magic_enum.hpp>
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/ethercat/SyncManager.hpp"

/**
 * @file 1Cxx-SyncManagerParameters.hpp
 * @brief CiA 301 / ETG.1000 SM Synchronization object-dictionary entries (0x1C32, 0x1C33).
 *
 * This header is a sibling of `60xx-Parameters.hpp` and follows the same register-
 * definition style.  It exposes:
 *
 *  - `SyncMode`            — strongly-typed enum for 0x1C32/0x1C33 sub-index 0x01.
 *  - `SupportedSyncTypes`  — bitmask enum for 0x1C32/0x1C33 sub-index 0x04.
 *  - `toString()` / `supportedSyncTypesToString()` — human-readable stringification.
 *  - `ObjectDictionaryEntry` constants for every sub-index consumed by diagnostics.
 *
 * The index and sub-index constants themselves live in `EtherCAT::SyncManager` /
 * `EtherCAT::SyncManager::SyncSub` (see `tether/ethercat/SyncManager.hpp`); this
 * header re-exports the relevant aliases so callers can use a single include.
 */

namespace CiA301 {
namespace Parameters1Cxx {

// ============================================================================
// Aliases — bring the SyncManager constants into this namespace
// ============================================================================

/// OD index for SM2 (RxPDO / process-data output) synchronization parameters.
constexpr uint16_t kIdxSM2Sync = EtherCAT::SyncManager::kSyncParamSM2; // 0x1C32

/// OD index for SM3 (TxPDO / process-data input) synchronization parameters.
constexpr uint16_t kIdxSM3Sync = EtherCAT::SyncManager::kSyncParamSM3; // 0x1C33

/// Sub-index: synchronization mode.
constexpr uint8_t kSubSyncMode         = EtherCAT::SyncManager::SyncSub::SyncMode;          // 0x01
/// Sub-index: cycle time in nanoseconds.
constexpr uint8_t kSubCycleTime        = EtherCAT::SyncManager::SyncSub::CycleTime;         // 0x02
/// Sub-index: output / input shift time.
constexpr uint8_t kSubShiftTime        = EtherCAT::SyncManager::SyncSub::ShiftTime;         // 0x03
/// Sub-index: supported synchronization types (bitmask).
constexpr uint8_t kSubSupportedTypes   = EtherCAT::SyncManager::SyncSub::SupportedSyncTypes; // 0x04
/// Sub-index: minimum cycle time.
constexpr uint8_t kSubMinCycleTime     = EtherCAT::SyncManager::SyncSub::MinimumCycleTime;  // 0x05
/// Sub-index: calculation and copy time.
constexpr uint8_t kSubCalcCopyTime     = EtherCAT::SyncManager::SyncSub::CalcAndCopyTime;   // 0x06
/// Sub-index: command register (latch cycle time).
constexpr uint8_t kSubCommand          = EtherCAT::SyncManager::SyncSub::CommandRegister;   // 0x08
/// Sub-index: SM-event-missed counter.
constexpr uint8_t kSubSMEventMissed    = EtherCAT::SyncManager::SyncSub::SMEventMissedCounter; // 0x0B
/// Sub-index: sync error flag.
constexpr uint8_t kSubSyncError        = EtherCAT::SyncManager::SyncSub::SyncError;         // 0x20

// ============================================================================
// SyncMode — synchronization mode (sub-index 0x01)
// ============================================================================

/**
 * @brief Synchronization mode values for 0x1C32:01 / 0x1C33:01.
 *
 * Defined by ETG.1000.6 section 5.5.2.
 */
enum class SyncMode : uint16_t {
    FreeRun  = 0, ///< Free-run — SM acts asynchronously to the master bus cycle.
    SMSync   = 1, ///< SM-synchronous — SM event triggers PDO update.
    DcSync0  = 2, ///< DC SYNC0 — PDO update locked to the SYNC0 signal.
    DcSync1  = 3, ///< DC SYNC1 — PDO update locked to the SYNC1 signal.
};

// ============================================================================
// SupportedSyncTypes — bitmask of supported modes (sub-index 0x04)
// ============================================================================

/**
 * @brief Bitmask flags for the Supported Synchronization Types field (sub-index 0x04).
 *
 * Each bit indicates that the corresponding SyncMode is available on the device.
 * Defined by ETG.1000.6 section 5.5.2.
 */
enum class SupportedSyncTypes : uint16_t {
    FreeRun         = (1u << 0), ///< Free-run mode supported.
    SMSync          = (1u << 1), ///< SM-synchronous mode supported.
    DcSync0         = (1u << 2), ///< DC SYNC0 mode supported.
    DcSync1         = (1u << 3), ///< DC SYNC1 mode supported.
    SubAppCycle     = (1u << 4), ///< Sub-application cycle supported (sub-cycle of DC SYNC0).
};

/**
 * @brief Format the Supported Synchronization Types bitmask as a comma-separated string.
 *
 * Example: `"FreeRun, DC-SYNC0"` for a value of 0x0005.
 * Returns `"none"` if no known bits are set.
 */
inline std::string supportedSyncTypesToString(uint16_t mask) {
    std::string result;
    auto append = [&](const char* label) {
        if (!result.empty()) result += ", ";
        result += label;
    };
    if (mask & static_cast<uint16_t>(SupportedSyncTypes::FreeRun))     append("FreeRun");
    if (mask & static_cast<uint16_t>(SupportedSyncTypes::SMSync))      append("SM-Synchronous");
    if (mask & static_cast<uint16_t>(SupportedSyncTypes::DcSync0))     append("DC-SYNC0");
    if (mask & static_cast<uint16_t>(SupportedSyncTypes::DcSync1))     append("DC-SYNC1");
    if (mask & static_cast<uint16_t>(SupportedSyncTypes::SubAppCycle)) append("SubAppCycle");
    // Report any unknown bits
    const uint16_t knownMask = 0x001Fu;
    const uint16_t unknown = mask & static_cast<uint16_t>(~knownMask);
    if (unknown) {
        char buf[16];
        if (!result.empty()) result += ", ";
        std::snprintf(buf, sizeof(buf), "0x%04X(?)", unknown);
        result += buf;
    }
    return result.empty() ? "none" : result;
}

// ============================================================================
// ObjectDictionaryEntry definitions — 0x1C32 (SM2) subindices
// ============================================================================

/// 0x1C32:01 — SM2 synchronization mode (output / RxPDO).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2SyncMode = {
    .index    = kIdxSM2Sync,
    .subindex = kSubSyncMode,
    .name     = "SM2 synchronization mode",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<SyncMode>{},
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
    .comment = "0=FreeRun, 1=SM-Synchronous, 2=DC-SYNC0, 3=DC-SYNC1",
};

/// 0x1C32:02 — SM2 cycle time in nanoseconds.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2CycleTime = {
    .index    = kIdxSM2Sync,
    .subindex = kSubCycleTime,
    .name     = "SM2 cycle time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit      = ::EtherCAT::ObjectDictionary::Unit_None,   // unit is nanoseconds; not in enum
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
    .comment = "Cycle time in nanoseconds written by master during PREOP→SAFEOP transition.",
};

/// 0x1C32:03 — SM2 output shift time in nanoseconds.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2ShiftTime = {
    .index    = kIdxSM2Sync,
    .subindex = kSubShiftTime,
    .name     = "SM2 shift time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
    .comment = "Output shift time relative to SYNC0 event, in nanoseconds.",
};

/// 0x1C32:04 — SM2 supported synchronization types (bitmask).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2SupportedSyncTypes = {
    .index    = kIdxSM2Sync,
    .subindex = kSubSupportedTypes,
    .name     = "SM2 supported synchronization types",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum  = std::type_identity<SupportedSyncTypes>{},
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
    .comment = "Bitmask: bit0=FreeRun, bit1=SM-Sync, bit2=DC-SYNC0, bit3=DC-SYNC1.",
};

/// 0x1C32:05 — SM2 minimum cycle time in nanoseconds (read-only, device capability).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2MinCycleTime = {
    .index    = kIdxSM2Sync,
    .subindex = kSubMinCycleTime,
    .name     = "SM2 minimum cycle time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C32:06 — SM2 calculation and copy time in nanoseconds (device-reported).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2CalcCopyTime = {
    .index    = kIdxSM2Sync,
    .subindex = kSubCalcCopyTime,
    .name     = "SM2 calculation and copy time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C32:0B — SM2 SM-event-missed counter (increments when a cycle is missed).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2SMEventMissedCounter = {
    .index    = kIdxSM2Sync,
    .subindex = kSubSMEventMissed,
    .name     = "SM2 SM-event missed counter",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT16_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C32:20 — SM2 sync error flag (set when sync is lost).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM2SyncError = {
    .index    = kIdxSM2Sync,
    .subindex = kSubSyncError,
    .name     = "SM2 sync error",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Boolean,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// ============================================================================
// ObjectDictionaryEntry definitions — 0x1C33 (SM3) subindices
// ============================================================================

/// 0x1C33:01 — SM3 synchronization mode (input / TxPDO).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3SyncMode = {
    .index    = kIdxSM3Sync,
    .subindex = kSubSyncMode,
    .name     = "SM3 synchronization mode",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum  = std::type_identity<SyncMode>{},
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
    .comment = "0=FreeRun, 1=SM-Synchronous, 2=DC-SYNC0, 3=DC-SYNC1",
};

/// 0x1C33:02 — SM3 cycle time in nanoseconds.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3CycleTime = {
    .index    = kIdxSM3Sync,
    .subindex = kSubCycleTime,
    .name     = "SM3 cycle time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C33:03 — SM3 input shift time in nanoseconds.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3ShiftTime = {
    .index    = kIdxSM3Sync,
    .subindex = kSubShiftTime,
    .name     = "SM3 shift time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::AtStop,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C33:04 — SM3 supported synchronization types (bitmask).
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3SupportedSyncTypes = {
    .index    = kIdxSM3Sync,
    .subindex = kSubSupportedTypes,
    .name     = "SM3 supported synchronization types",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum  = std::type_identity<SupportedSyncTypes>{},
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C33:05 — SM3 minimum cycle time in nanoseconds.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3MinCycleTime = {
    .index    = kIdxSM3Sync,
    .subindex = kSubMinCycleTime,
    .name     = "SM3 minimum cycle time",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C33:0B — SM3 SM-event-missed counter.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3SMEventMissedCounter = {
    .index    = kIdxSM3Sync,
    .subindex = kSubSMEventMissed,
    .name     = "SM3 SM-event missed counter",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT16_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

/// 0x1C33:20 — SM3 sync error flag.
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SM3SyncError = {
    .index    = kIdxSM3Sync,
    .subindex = kSubSyncError,
    .name     = "SM3 sync error",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Boolean,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time    = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// ============================================================================
// Register table
// ============================================================================

constexpr std::array<const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry*, 14> kRegisterList = {
    &SM2SyncMode,
    &SM2CycleTime,
    &SM2ShiftTime,
    &SM2SupportedSyncTypes,
    &SM2MinCycleTime,
    &SM2CalcCopyTime,
    &SM2SMEventMissedCounter,
    &SM2SyncError,
    &SM3SyncMode,
    &SM3CycleTime,
    &SM3ShiftTime,
    &SM3SupportedSyncTypes,
    &SM3SMEventMissedCounter,
    &SM3SyncError,
};

} // namespace Parameters1Cxx
} // namespace CiA301
