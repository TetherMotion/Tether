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
#include "tether/sii/SIILogger.hpp"
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <string>

namespace EtherCAT {

class Master;  // forward declaration

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
     * @param master Master instance for network I/O
     */
    explicit SIIReader(Master& master);
    
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
     * @brief Prefetch a contiguous block of EEPROM words into the master cache.
     *
     * Reads `count` words starting at `word_address` in one pass, storing each
     * word in the master-level per-slave cache. Subsequent reads of these
     * words will be cache hits with zero bus traffic.
     *
     * @param slave_index  Slave index
     * @param word_address Starting EEPROM word address
     * @param count        Number of words to prefetch
     * @return Number of words successfully prefetched
     */
    size_t prefetchWords(uint16_t slave_index, uint16_t word_address, uint16_t count);

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

    /// @brief Access the owning Master (for log prefix queries)
    Master& master() { return m_master; }
    const Master& master() const { return m_master; }

private:
    Master& m_master;
    uint32_t m_timeout_ms{500};

    // Internal helpers
    bool waitNotBusy(uint16_t slave_index, uint16_t* out_status, uint32_t* out_poll_iters = nullptr);
    bool readRaw32(uint16_t slave_index, uint16_t word_address, uint32_t* out);
    uint16_t adpForSlave(uint16_t slave_index);

    // --- FPWR fallback for ESCs that reject APWR to EEPCTL (0x0502) ---
    // Some ESCs (e.g. Synapticon SOMANET drive) return WKC=0 for APWR to
    // the EEPROM control register, while APRD to the same register works
    // fine.  As a fallback, we read the slave's configured station address
    // (ESC register 0x0010, via APRD) and retry the write with FPWR.
    // If the configured station address is 0x0000 (unassigned), a unique
    // address (slave_index + 1) is written via APWR to 0x0010 first.
    //
    // Each SIIReader is owned by exactly one SIIManager (which is owned by
    // one Slave), so this state is strictly per-slave — no slave_index-keyed
    // containers are needed. A mutex protects concurrent same-slave access.

    /// Cached configured station address (0 = not yet determined).
    /// Guarded by state_mutex_.
    uint16_t configured_addr_ = 0;
    bool configured_addr_set_ = false;

    /// True if APWR to EEPCTL already failed for this slave — use FPWR.
    /// Guarded by state_mutex_.
    bool use_fpwr_ = false;

    /// True if the EEPROM has already been forced to ECAT control.
    /// Guarded by state_mutex_.
    bool eeprom_forced_ = false;

    /// Mutex protecting configured_addr_, use_fpwr_, eeprom_forced_.
    mutable std::mutex state_mutex_;

    /// Read (and cache) the configured station address for this reader's slave.
    /// If the address is 0x0000, assigns a unique one via APWR to 0x0010.
    /// If force_assign is true, always writes a fresh address (used when
    /// the existing configured address is stale and FPWR to it fails).
    bool getConfiguredStationAddr(uint16_t slave_index, uint16_t& addr,
                                  bool force_assign = false);

    /// Write the EEPCTL register (0x0502).  Tries APWR first; on WKC=0
    /// falls back to FPWR using the configured station address.
    bool writeEEPCTL(uint16_t slave_index, uint16_t eepctl_val);

    /// Write an EEPROM register (any of 0x0500–0x050F) using the same
    /// APWR→FPWR fallback as writeEEPCTL.
    bool writeEepromReg(uint16_t slave_index, uint16_t reg_addr,
                        const void* data, uint16_t len);

    /// Force EEPROM interface from PDI to ECAT control.
    /// Writes 0x02 then 0x00 to EEPConfig (0x0500) via APWR.
    /// Called automatically before the first EEPROM access for each slave.
    /// Tracks whether the EEPROM has already been forced.
    bool forceEepromToEcat(uint16_t slave_index);

public:
    // --- Per-slave SII bus state introspection (used by SIIManager) ---

    /// @brief Cached configured station address, or 0 if not yet determined.
    uint16_t configuredStationAddr() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return configured_addr_;
    }

    /// @brief True if this slave should use FPWR for EEPROM control writes.
    bool useFpwr() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return use_fpwr_;
    }

    /// @brief True if the EEPROM has already been forced to ECAT control.
    bool eepromForcedToEcat() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return eeprom_forced_;
    }
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

    /// Build the log prefix for a slave (delegates to Master)
    std::string logPrefix(uint16_t slave_index) const;
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
 * @param master Master instance for network I/O
 * @param slave_index Slave index
 * @param out_data Output parsed SII data
 * @return true on success
 */
bool readSII(Master& master, uint16_t slave_index, SIIData& out_data);

/**
 * @brief Read SII identity only (quick read)
 */
bool readSIIIdentity(Master& master, uint16_t slave_index, SIIIdentity& out_identity);

/**
 * @brief Read SII mailbox configuration
 */
bool readSIIMailbox(Master& master, uint16_t slave_index, SIIMailboxConfig& out_mailbox);

} // namespace SII
} // namespace EtherCAT
