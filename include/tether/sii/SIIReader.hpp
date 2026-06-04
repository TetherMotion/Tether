/**
 * @file SIIReader.hpp
 * @brief EtherCAT SII EEPROM Reader
 * 
 * @details
 * This module provides low-level EEPROM reading functionality and high-level
 * parsing of the SII (Slave Information Interface) data.
 * 
 * ## Reading Process
 * 
 * 1. Read EEPROM configuration area (words 0x0000-0x0007)
 * 2. Read device identity (words 0x0008-0x000F)
 * 3. Read mailbox configuration (words 0x0014-0x001D)
 * 4. Parse category area starting at word 0x0040
 * 
 * ## Category Parsing
 * 
 * Categories are parsed in order:
 * - CAT_STRINGS (10): Device strings
 * - CAT_GENERAL (30): General device info
 * - CAT_FMMU (40): FMMU configuration
 * - CAT_SYNC_MANAGER (41): Sync Manager configuration
 * - CAT_TXPDO (50): TxPDO descriptions
 * - CAT_RXPDO (51): RxPDO descriptions
 * - CAT_DC (60): Distributed Clock configuration
 */

#pragma once

#include "sii/SIIParser.hpp"
#include <cstdint>
#include <cstddef>

namespace EtherCAT {

class EtherCATMaster;  // forward declaration

namespace SII {

// ============================================================================
// SII Reader Class
// ============================================================================

/**
 * @brief Low-level SII EEPROM reader
 * 
 * Handles direct EEPROM communication and reading of raw data.
 */
class SIIReader {
public:
    /**
     * @brief Constructor
     * @param master EtherCATMaster instance for network I/O
     */
    explicit SIIReader(EtherCATMaster& master);
    
    /**
     * @brief Read a single word (16-bit) from EEPROM
     * @param slave_index Slave index (0-based)
     * @param word_address EEPROM word address
     * @param out Output value
     * @return true on success
     */
    bool readWord(uint16_t slave_index, uint16_t word_address, uint16_t& out);
    
    /**
     * @brief Read a double-word (32-bit) from EEPROM
     * @param slave_index Slave index
     * @param word_address EEPROM word address (must be aligned)
     * @param out Output value
     * @return true on success
     */
    bool readDWord(uint16_t slave_index, uint16_t word_address, uint32_t& out);
    
    /**
     * @brief Read multiple words from EEPROM
     * @param slave_index Slave index
     * @param word_address Starting word address
     * @param buffer Output buffer (in words)
     * @param word_count Number of words to read
     * @return Number of words successfully read
     */
    size_t readWords(uint16_t slave_index, uint16_t word_address, 
                     uint16_t* buffer, size_t word_count);
    
    /**
     * @brief Read bytes from EEPROM
     * @param slave_index Slave index
     * @param byte_address Starting byte address
     * @param buffer Output buffer
     * @param byte_count Number of bytes to read
     * @return Number of bytes successfully read
     */
    size_t readBytes(uint16_t slave_index, uint16_t byte_address,
                     uint8_t* buffer, size_t byte_count);
    
    /**
     * @brief Read a string from EEPROM string category
     * @param slave_index Slave index
     * @param string_index String index (1-based)
     * @param buffer Output buffer
     * @param buffer_size Buffer size
     * @return true on success
     */
    bool readString(uint16_t slave_index, uint8_t string_index,
                    char* buffer, size_t buffer_size);
    
    /**
     * @brief Set read timeout
     */
    void setTimeout(uint32_t timeout_ms) { m_timeout_ms = timeout_ms; }
    
private:
    EtherCATMaster& m_master;
    uint32_t m_timeout_ms{500};
    
    // Internal helpers
    bool waitNotBusy(uint16_t slave_index, uint16_t* out_status);
    bool readRaw32(uint16_t slave_index, uint16_t word_address, uint32_t* out);
    uint16_t adpForSlave(uint16_t slave_index);
};

// ============================================================================
// SII Parser Class
// ============================================================================

/**
 * @brief High-level SII parser
 * 
 * Parses raw SII data into structured SIIData.
 */
class SIIParser {
public:
    /**
     * @brief Constructor with reader reference
     */
    explicit SIIParser(SIIReader& reader);
    
    /**
     * @brief Parse complete SII for a slave
     * @param slave_index Slave index
     * @param out_data Output parsed data
     * @return true on success
     */
    bool parse(uint16_t slave_index, SIIData& out_data);
    
    /**
     * @brief Parse only the configuration area (quick read)
     */
    bool parseConfigArea(uint16_t slave_index, SIIData& out_data);
    
    /**
     * @brief Parse only identity and mailbox (medium read)
     */
    bool parseIdentity(uint16_t slave_index, SIIData& out_data);
    
    /**
     * @brief Get last parse error message
     */
    const char* lastError() const { return m_last_error; }
    
private:
    SIIReader& m_reader;
    char m_last_error[128]{0};
    
    // Category parsers
    bool parseStrings(uint16_t slave_index, uint16_t byte_offset, 
                      uint16_t size_bytes, SIIData& data);
    bool parseGeneral(uint16_t slave_index, uint16_t byte_offset,
                      uint16_t size_bytes, SIIData& data);
    bool parseFMMU(uint16_t slave_index, uint16_t byte_offset,
                   uint16_t size_bytes, SIIData& data);
    bool parseSyncManager(uint16_t slave_index, uint16_t byte_offset,
                          uint16_t size_bytes, SIIData& data);
    bool parsePDO(uint16_t slave_index, uint16_t byte_offset,
                  uint16_t size_bytes, SIIData& data, bool is_tx);
    bool parseDC(uint16_t slave_index, uint16_t byte_offset,
                 uint16_t size_bytes, SIIData& data);
    
    void setError(const char* fmt, ...);
};

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Read complete SII data from a slave
 * 
 * This is the main entry point for SII reading. It creates temporary
 * reader/parser objects and returns the parsed data.
 * 
 * @param master EtherCATMaster instance for network I/O
 * @param slave_index Slave index
 * @param out_data Output parsed SII data
 * @return true on success
 */
bool readSII(EtherCATMaster& master, uint16_t slave_index, SIIData& out_data);

/**
 * @brief Read SII identity only (quick read)
 */
bool readSIIIdentity(EtherCATMaster& master, uint16_t slave_index, SIIIdentity& out_identity);

/**
 * @brief Read SII mailbox configuration
 */
bool readSIIMailbox(EtherCATMaster& master, uint16_t slave_index, SIIMailboxConfig& out_mailbox);

// ============================================================================
// Logging/Printing Functions
// ============================================================================

/**
 * @brief Log complete SII data
 * @param data Parsed SII data
 * @param tag ESP_LOG tag
 */
void logSIIData(const SIIData& data, const char* tag);

/**
 * @brief Log SII identity
 */
void logSIIIdentity(const SIIIdentity& identity, const char* tag);

/**
 * @brief Log SII mailbox configuration
 */
void logSIIMailbox(const SIIMailboxConfig& mailbox, const char* tag);

/**
 * @brief Log SII Sync Manager configuration
 */
void logSIISyncManagers(const SIIData& data, const char* tag);

/**
 * @brief Log SII PDO configuration
 */
void logSIIPDOs(const SIIData& data, const char* tag);

/**
 * @brief Log SII summary (one-line per slave)
 */
void logSIISummary(const SIIData& data, uint16_t slave_index, const char* tag);

/**
 * @brief Debug SII mailbox derivation with step-by-step detail
 * 
 * This function provides extremely detailed logging of how mailbox configuration
 * is derived from SII EEPROM, including:
 * - Raw EEPROM word reads with addresses and values
 * - Byte extraction from multi-byte reads
 * - Field assignments and their meanings
 * - Final SM0/SM1 configuration mapping
 * - Protocol flag decoding
 * 
 * @param master EtherCATMaster instance for network I/O
 * @param slave_index Slave index
 * @param tag ESP_LOG tag
 */
void debugSIIMailboxDerivation(EtherCATMaster& master, uint16_t slave_index, const char* tag);

} // namespace SII
} // namespace EtherCAT
