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
#include <string_view>

namespace EtherCAT {
class Master;
namespace SII {

struct SIIData;
struct SIIIdentity;
struct SIIMailboxConfig;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Map an SII category type code to a human-readable string.
 * @param type  Category type field from the SII category header.
 * @return static category name, or "Unknown" for unlisted codes.
 */
const char* getCategoryTypeName(uint16_t type);

/**
 * @brief Map a mailbox protocol bitfield to a human-readable string.
 * @param protocol  Protocol support bits from the SII mailbox config.
 * @return static protocol name, or "Unknown" for unlisted values.
 */
const char* getMailboxProtocolName(uint16_t protocol);

// ============================================================================
// Logging Functions
// ============================================================================

/**
 * @brief Log complete SII data (identity, mailbox, SMs, FMMUs, PDOs, DC).
 * @param data  Parsed SII data.
 * @param tag   Logger tag prefix.
 */
void logSIIData(const SIIData& data, const char* tag);

/**
 * @brief Log SII identity (vendor ID, product code, revision, serial).
 * @param identity  Parsed identity block.
 * @param tag       Logger tag prefix.
 */
void logSIIIdentity(const SIIIdentity& identity, const char* tag);

/**
 * @brief Log SII mailbox configuration (standard + bootstrap, protocols).
 * @param mailbox  Parsed mailbox config.
 * @param tag      Logger tag prefix.
 */
void logSIIMailbox(const SIIMailboxConfig& mailbox, const char* tag);

/**
 * @brief Log SII Sync Manager configuration.
 * @param data  Parsed SII data (uses its sync-manager list).
 * @param tag   Logger tag prefix.
 */
void logSIISyncManagers(const SIIData& data, const char* tag);

/**
 * @brief Log SII PDO configuration (TxPDO and RxPDO entries).
 * @param data  Parsed SII data (uses both PDO lists).
 * @param tag   Logger tag prefix.
 */
void logSIIPDOs(const SIIData& data, const char* tag);

/**
 * Log a one-line SII summary for a slave.
 * @param data        Parsed SII data
 * @param log_prefix  Log prefix string (e.g. "Slave My Drive (#0)" or "Slave 0")
 * @param tag         Log tag
 */
void logSIISummary(const SIIData& data, std::string_view log_prefix, const char* tag);

/**
 * Debug SII mailbox derivation with step-by-step detail.
 *
 * Reads the raw EEPROM words and logs every step of how the mailbox
 * configuration is derived, including byte breakdowns, field
 * assignments, protocol flag decoding, and validation checks.
 *
 * @param master      Master instance for network I/O
 * @param slave_index Slave index
 * @param tag         Log tag
 */
void debugSIIMailboxDerivation(Master& master, uint16_t slave_index, const char* tag);

} // namespace SII
} // namespace EtherCAT
