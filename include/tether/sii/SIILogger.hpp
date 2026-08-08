// SPDX-License-Identifier: MIT
/**
 * @file SIILogger.hpp
 * @brief SII EEPROM logging and diagnostic printing functions
 *
 * @details
 * Provides human-readable logging of parsed SII (Slave Information
 * Interface) EEPROM data, plus a detailed mailbox-derivation debug
 * helper.  These functions are stateless — they operate only on the
 * data structures passed to them.
 *
 * Extracted from SIIReader.cpp to improve modularity.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {
class Master;
namespace SII {

struct SIIData;
struct SIIIdentity;
struct SIIMailboxConfig;

// ============================================================================
// Utility Functions
// ============================================================================

/// Map an SII category type code to a human-readable string.
const char* getCategoryTypeName(uint16_t type);

/// Map a mailbox protocol bitfield to a human-readable string.
const char* getMailboxProtocolName(uint16_t protocol);

// ============================================================================
// Logging Functions
// ============================================================================

/// Log complete SII data (identity, mailbox, SMs, FMMUs, PDOs, DC).
void logSIIData(const SIIData& data, const char* tag);

/// Log SII identity (vendor ID, product code, revision, serial).
void logSIIIdentity(const SIIIdentity& identity, const char* tag);

/// Log SII mailbox configuration (standard + bootstrap, protocols).
void logSIIMailbox(const SIIMailboxConfig& mailbox, const char* tag);

/// Log SII Sync Manager configuration.
void logSIISyncManagers(const SIIData& data, const char* tag);

/// Log SII PDO configuration (TxPDO and RxPDO entries).
void logSIIPDOs(const SIIData& data, const char* tag);

/// Log a one-line SII summary for a slave.
void logSIISummary(const SIIData& data, uint16_t slave_index, const char* tag);

/// Debug SII mailbox derivation with step-by-step detail.
///
/// Reads the raw EEPROM words and logs every step of how the mailbox
/// configuration is derived, including byte breakdowns, field
/// assignments, protocol flag decoding, and validation checks.
///
/// @param master      Master instance for network I/O
/// @param slave_index Slave index
/// @param tag         Log tag
void debugSIIMailboxDerivation(Master& master, uint16_t slave_index, const char* tag);

} // namespace SII
} // namespace EtherCAT
