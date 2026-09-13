/**
 * @file SIIDemandParser.hpp
 * @brief Demand-driven SII parser operating on a word cache
 *
 * @details
 * SIIDemandParser implements the same SII parsing logic as SIIParser but
 * operates exclusively on a SIISlaveCache — it never performs bus I/O.
 * Instead of blocking when a needed EEPROM word is missing from the cache,
 * it returns a list of word-pair addresses that the caller must fetch.
 * The caller fetches those word-pairs (via SIIReader::readDWord or the
 * EEPROMReactor), stores them in the cache, and calls parse() again.
 * This continues until parsing is complete.
 *
 * ## Why demand-driven?
 *
 * The traditional SIIParser reads words on demand from the bus, blocking
 * for each read. For a single slave this is fine, but for concurrent
 * multi-slave discovery it serialises all EEPROM reads. The demand parser
 * decouples "what words are needed" from "how words are fetched":
 *
 *  - The **blocking SII reader** (SIIManager::parseCategories) calls
 *    parse() in a loop, fetching needed word-pairs via SIIReader::readDWord.
 *  - The **EEPROMReactor** calls parse() per slave, fetching needed
 *    word-pairs concurrently across all slaves via the packet router.
 *
 * Both paths use the same parsing code, ensuring consistency.
 *
 * ## Parsing phases
 *
 * The parser is a resumable state machine with the following phases:
 *
 * 1. **CONFIG_AREA** — reads words 0x0000-0x0007 (PDI config, alias, CRC).
 * 2. **IDENTITY** — reads words 0x0008-0x000F (vendor/product/revision/serial)
 *    and words 0x0014-0x001D (mailbox configuration).
 * 3. **SIZE_INFO** — reads word-pair 0x002E-0x002F (EEPROM size, version).
 * 4. **CATEGORY_SCAN** — scans categories from 0x0040. For each category:
 *    reads the 2-word header, then (if the category is requested) reads
 *    and parses the category data. Categories not in cat_mask are skipped
 *    (only their headers are read to advance the scan).
 *
 * Each phase returns all missing word-pairs at once, allowing the caller
 * to fetch them in a single batch. After the word-pairs are cached, the
 * next parse() call advances to the next phase (or the next category).
 *
 * ## Reuse of parsing logic
 *
 * The category data parsing (strings, general, FMMU, sync managers, PDOs,
 * DC) is extracted into shared free functions (parseStringsFromBuffer,
 * parseGeneralFromBuffer, etc.) that operate on a raw byte buffer. Both
 * SIIParser (blocking, bus-backed) and SIIDemandParser (cache-backed)
 * call these functions, ensuring the parsing logic is not duplicated.
 *
 * ## Example (blocking)
 *
 * @code
 * SIIDemandParser parser;
 * parser.init(SII::CAT_MASK_ALL);
 * while (true) {
 *     auto result = parser.parse(cache, out_data);
 *     if (result.status == SIIDemandResult::COMPLETE) break;
 *     if (result.status == SIIDemandResult::FAILED) return false;
 *     for (uint16_t addr : result.needed_word_pairs) {
 *         uint32_t dword;
 *         reader.readDWord(slave_index, addr, dword);  // caches the word-pair
 *     }
 * }
 * @endcode
 *
 * ## Example (reactor)
 *
 * The EEPROMReactor calls parse() per slave, queues the needed word-pairs,
 * and issues concurrent bus reads. See EEPROMReactor for details.
 *
 * @see SIIParser
 * @see SIISlaveCache
 * @see EEPROMReactor
 */

#pragma once

#include "tether/ethercat/TetherConfig.hpp"

#if TETHER_ENABLE_SII

#include "tether/sii/SIIParser.hpp"
#include "tether/sii/SIIReader.hpp"
#include <cstdint>
#include <vector>

namespace EtherCAT {
namespace SII {

class SIISlaveCache;

// ============================================================================
// Shared category-data parsing functions
// ============================================================================

/**
 * @brief Parse the strings category from a raw byte buffer.
 *
 * Shared between SIIParser (bus-backed) and SIIDemandParser (cache-backed).
 *
 * @param data        Raw category bytes (size_bytes long).
 * @param size_bytes  Number of bytes in the buffer.
 * @param out         Output SII data (strings table is populated).
 * @return true on success.
 */
bool parseStringsFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out);

/**
 * @brief Parse the general device info category from a raw byte buffer.
 */
bool parseGeneralFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out);

/**
 * @brief Parse the FMMU category from a raw byte buffer.
 */
bool parseFMMUFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out);

/**
 * @brief Parse the sync manager category from a raw byte buffer.
 */
bool parseSyncManagerFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out);

/**
 * @brief Parse a PDO category (TxPDO or RxPDO) from a raw byte buffer.
 * @param is_tx  true for TxPDO, false for RxPDO.
 */
bool parsePDOFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out, bool is_tx);

/**
 * @brief Parse the distributed clocks category from a raw byte buffer.
 */
bool parseDCFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out);

// ============================================================================
// SIIDemandResult
// ============================================================================

/**
 * @brief Result of a demand-driven SII parse attempt.
 */
struct SIIDemandResult {
    /// Parse status.
    enum Status : uint8_t {
        NEED_WORDS,  ///< More word-pairs must be fetched from the bus.
        COMPLETE,    ///< Parsing is complete; out_data is valid.
        FAILED,      ///< Parsing failed (permanent error).
    } status{NEED_WORDS};

    /// Even word addresses that must be fetched (32-bit reads).
    /// Populated only when status == NEED_WORDS.
    /// Each address starts a 2-word (32-bit) EEPROM read.
    std::vector<uint16_t> needed_word_pairs;

    /// True if parsing is complete.
    bool isComplete() const { return status == COMPLETE; }

    /// True if parsing failed.
    bool isFailed() const { return status == FAILED; }

    /// True if more words are needed.
    bool needsWords() const { return status == NEED_WORDS; }
};

// ============================================================================
// SIIDemandParser
// ============================================================================

/**
 * @brief Demand-driven SII parser that operates on a word cache.
 *
 * See the file-level documentation for a full description of the design
 * and usage patterns.
 */
class SIIDemandParser {
public:
    SIIDemandParser() = default;

    /**
     * @brief Initialize the parser for a new parse operation.
     *
     * Resets all internal state. Must be called before parse().
     *
     * @param cat_mask  Bitmask of SIICategoryMask values selecting which
     *                  categories to parse. CAT_MASK_ALL for everything.
     *                  CAT_STRINGS is auto-included if any category that
     *                  references string indices is requested.
     */
    void init(uint32_t cat_mask);

    /**
     * @brief Attempt to advance parsing using cached data.
     *
     * Advances through the parsing phases as far as possible using only
     * words already present in the cache. If a needed word is missing,
     * returns NEED_WORDS with the list of word-pair addresses to fetch.
     * The caller fetches those word-pairs, stores them in the cache, and
     * calls parse() again.
     *
     * @param cache    Per-slave SII word cache (read-only).
     * @param out_data Output parsed SII data. Valid when status == COMPLETE.
     *                 May be partially populated between calls.
     * @return Parse result.
     */
    SIIDemandResult parse(const SIISlaveCache& cache, SIIData& out_data);

    /// True if parsing is complete (all phases done).
    bool isComplete() const { return phase_ == Phase::COMPLETE; }

    /// True if parsing failed.
    bool isFailed() const { return phase_ == Phase::FAILED; }

    /// Current phase (for diagnostics).
    uint8_t phase() const { return static_cast<uint8_t>(phase_); }

    /// Last error message (if failed).
    const char* lastError() const { return last_error_; }

    // ----------------------------------------------------------------
    // Cache reading helpers (public for testing)
    // ----------------------------------------------------------------

    /**
     * @brief Read a 32-bit word-pair from the cache.
     * @param cache   Per-slave cache.
     * @param addr    Even word address.
     * @param out     32-bit value (lo word in bits 0..15).
     * @return true if both words were cached.
     */
    static bool readWordPair(const SIISlaveCache& cache, uint16_t addr, uint32_t& out);

    /**
     * @brief Read a 16-bit word from the cache.
     * @param cache   Per-slave cache.
     * @param addr    Word address (may be odd).
     * @param out     16-bit value.
     * @return true if the word was cached.
     */
    static bool readWord(const SIISlaveCache& cache, uint16_t addr, uint16_t& out);

    /**
     * @brief Read bytes from the cache into a buffer.
     *
     * Reads byte_count bytes starting at byte_addr. All needed word-pairs
     * must be cached; if any is missing, reading stops and the return
     * value is less than byte_count.
     *
     * @param cache       Per-slave cache.
     * @param byte_addr   Starting byte address.
     * @param buffer      Output buffer.
     * @param byte_count  Number of bytes to read.
     * @return Number of bytes actually read (== byte_count if all cached).
     */
    static size_t readBytes(const SIISlaveCache& cache, uint16_t byte_addr,
                            uint8_t* buffer, size_t byte_count);

    /**
     * @brief Find the first missing word-pair in a range.
     *
     * Scans word-pairs from start to start+word_count-1 (stepping by 2).
     * Returns the address of the first missing word-pair, or a value
     * >= start+word_count if all are cached.
     *
     * @param cache       Per-slave cache.
     * @param start       Starting word address (should be even).
     * @param word_count  Number of words to check.
     * @return Address of first missing word-pair, or 0xFFFF if all cached.
     */
    static uint16_t firstMissingWordPair(const SIISlaveCache& cache,
                                          uint16_t start, uint16_t word_count);

private:
    /// Parsing phases (sequential, resumable).
    enum class Phase : uint8_t {
        CONFIG_AREA,    ///< Words 0x0000-0x0007
        IDENTITY,       ///< Words 0x0008-0x000F, 0x0014-0x001D
        SIZE_INFO,      ///< Word-pair 0x002E-0x002F
        CATEGORY_SCAN,  ///< Categories from 0x0040
        COMPLETE,       ///< All phases done
        FAILED,         ///< Error
    };

    Phase    phase_{Phase::CONFIG_AREA};
    uint32_t cat_mask_{CAT_MASK_ALL};

    // Category scan state (used during CATEGORY_SCAN phase)
    uint16_t cat_scan_addr_{SII_CATEGORY_START};  ///< Next category header addr
    int      category_count_{0};                   ///< Categories processed

    char last_error_[128]{};

    /// Collect missing word-pairs from a list of even addresses.
    static void collectMissing(const SIISlaveCache& cache,
                               std::vector<uint16_t>& out,
                               std::initializer_list<uint16_t> pairs);

    /// Set error message and transition to FAILED.
    void fail(const std::string& msg);

    // Phase handlers — each returns NEED_WORDS, COMPLETE, or FAILED.
    SIIDemandResult doConfigArea(const SIISlaveCache& cache, SIIData& out);
    SIIDemandResult doIdentity(const SIISlaveCache& cache, SIIData& out);
    SIIDemandResult doSizeInfo(const SIISlaveCache& cache, SIIData& out);
    SIIDemandResult doCategoryScan(const SIISlaveCache& cache, SIIData& out);
};

} // namespace SII
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
