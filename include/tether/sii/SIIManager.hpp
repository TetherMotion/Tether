/**
 * @file SIIManager.hpp
 * @brief Per-slave SII (Slave Information Interface) manager
 *
 * @details
 * `SIIManager` is the public, per-slave API for reading and parsing a slave's
 * SII EEPROM. Each `Slave` owns one `SIIManager`, accessed as
 * `master.slave(5).sii()`. The manager is thread-safe for concurrent access to
 * different slaves; access to the same slave is serialized by a per-slave mutex.
 *
 * The design is built around two per-slave objects:
 *   - `SIISlaveCache`: a word-address -> word-value cache with its own mutex.
 *   - `SIIManager`: the cache plus the slave's SII bus state (configured
 *     station address, FPWR fallback, "EEPROM forced to ECAT" flag) and the
 *     high-level read / parse API.
 *
 * The low-level SII bus protocol is implemented in `SIIManager.cpp` using the
 * owning `Master` for register reads/writes. All of this code is guarded by
 * `#if TETHER_ENABLE_SII`; set the CMake option `TETHER_ENABLE_SII=OFF` to
 * compile the SII subsystem out.
 *
 * ## Thread-safety
 *
 * `SIIManager` is fully thread-safe:
 *   - Concurrent reads of **different** slaves are parallel (each slave has
 *     its own manager with its own mutexes).
 *   - Concurrent reads of **already-cached** words on the same slave are
 *     parallel (the word cache has its own mutex).
 *   - Concurrent bus operations (EEPROM reads) on the **same** slave are
 *     serialized by an internal `bus_mutex_`, so multiple threads may call
 *     `readWord()` / `parseFull()` / `prefetchWords()` on the same manager
 *     without corrupting the EEPROM read sequence.
 *   - The parsed-SII cache (`parseFull()`) is also guarded by `bus_mutex_`.
 *
 * ## Example
 * @code
 *   #if TETHER_ENABLE_SII
 *   auto& sii = master.slave(5).sii();
 *
 *   uint16_t vendor = 0;
 *   if (sii.readWord(0x0008, vendor)) {
 *       // second call is a cache hit if 0x0009 was part of the same 32-bit read
 *       uint16_t product = 0;
 *       sii.readWord(0x0009, product);
 *   }
 *
 *   SII::SIIData data;
 *   if (sii.parseFull(data)) {
 *       // use data.productCode, data.mailboxConfig, etc.
 *   }
 *   #endif
 * @endcode
 */

#pragma once

#include "tether/ethercat/TetherConfig.hpp"

#if TETHER_ENABLE_SII

#include "tether/sii/SIIParser.hpp"
#include "tether/sii/SIIReader.hpp"
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace EtherCAT {

class Master;  // forward declaration

namespace SII {

// ============================================================================
// SIISlaveCache
// ============================================================================

/**
 * @brief Thread-safe, per-slave cache of 16-bit SII EEPROM words.
 *
 * The cache is keyed by word address. It has its own mutex so that pure
 * cache reads/writes do not contend with the per-slave SII bus state mutex.
 */
class SIISlaveCache {
public:
    SIISlaveCache() = default;

    /**
     * @brief Look up a word in the cache.
     * @param word_addr  SII word address
     * @param[out] out   Cached value, if present
     * @return true if the word was in the cache
     */
    bool get(uint16_t word_addr, uint16_t& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = words_.find(word_addr);
        if (it == words_.end()) return false;
        out = it->second;
        return true;
    }

    /**
     * @brief Store a word in the cache.
     * @param word_addr  SII word address
     * @param value      16-bit word value
     */
    void set(uint16_t word_addr, uint16_t value) {
        std::lock_guard<std::mutex> lock(mutex_);
        words_[word_addr] = value;
    }

    /** @brief Remove all cached words. */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        words_.clear();
    }

    /** @brief Number of cached words. */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return words_.size();
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<uint16_t, uint16_t> words_;
};

// ============================================================================
// SIIManager
// ============================================================================

/**
 * @brief Per-slave SII access manager.
 *
 * This is the only user-facing object for reading a slave's EEPROM.
 * It combines a word-level cache with the per-slave SII bus state and the
 * higher-level read/parse API. Access to a single slave is serialized; access
 * to different slaves is parallel.
 */
class SIIManager {
public:
    /**
     * @brief Construct an uninitialised manager (must call `init()` before use).
     */
    SIIManager() = default;

    /**
     * @brief Construct and bind to a master and slave index.
     * @param master       Owning EtherCAT master
     * @param slave_index  Slave position on the bus
     */
    SIIManager(Master& master, uint16_t slave_index);

    /** @brief Destructor: clears the back-pointer to the owning master. */
    ~SIIManager();

    /**
     * @brief Deferred binding.
     * @param master       Owning EtherCAT master
     * @param slave_index  Slave position on the bus
     */
    void init(Master& master, uint16_t slave_index);

    /** @brief Slave index this manager is bound to. */
    uint16_t slaveIndex() const { return slave_index_; }

    /** @brief True if the manager has been bound to a master and slave index. */
    bool isInitialised() const { return master_ != nullptr; }

    /** @brief Alias for isInitialised(). */
    bool isInitialized() const { return isInitialised(); }

    /** @brief Access the word cache directly. */
    SIISlaveCache& cache() { return cache_; }
    const SIISlaveCache& cache() const { return cache_; }

    /** @brief Read a 16-bit word from SII (cached). */
    bool readWord(uint16_t word_addr, uint16_t& out);

    /** @brief Read a 32-bit double-word from SII (cached as two words). */
    bool readDWord(uint16_t word_addr, uint32_t& out);

    /**
     * @brief Read multiple contiguous words.
     * @return Number of words successfully read.
     */
    size_t readWords(uint16_t word_addr, uint16_t* buffer, size_t word_count);

    /**
     * @brief Read a contiguous block of bytes from SII.
     * @return Number of bytes successfully read.
     */
    size_t readBytes(uint16_t byte_addr, uint8_t* buffer, size_t byte_count);

    /**
     * @brief Prefetch a contiguous block of words into the cache.
     * @return Number of words successfully prefetched.
     */
    size_t prefetchWords(uint16_t word_addr, uint16_t word_count);

    /**
     * @brief Read a string from the SII strings category.
     * @param string_index 1-based string index
     * @param buffer       Output buffer
     * @param buffer_size  Buffer capacity
     * @return true on success
     */
    bool readString(uint8_t string_index, char* buffer, size_t buffer_size);

    /**
     * @brief Parse the full SII and cache the result.
     *
     * The first call performs the full parse; subsequent calls return the
     * cached result.
     *
     * @param[out] data Parsed SII data
     * @return true on success
     */
    bool parseFull(SIIData& data);

    /**
     * @brief Parse identity plus a selectable subset of SII categories.
     *
     * Only reads the EEPROM categories whose bits are set in `cat_mask`.
     * Categories not requested are skipped (only their 2-word headers are
     * read to advance the scan). This avoids unnecessary EEPROM bus
     * traffic when only a subset of SII data is needed.
     *
     * CAT_STRINGS is implicitly included if any of CAT_GENERAL, CAT_TXPDO,
     * or CAT_RXPDO is requested.
     *
     * The result is cached per category mask; a subsequent call with the
     * same mask returns the cached result. A call with a different mask
     * re-parses from scratch.
     *
     * @param[out] data   Parsed SII data (identity always populated)
     * @param      cat_mask Bitmask of SIICategoryMask values
     * @return true on success
     */
    bool parseCategories(SIIData& data, uint32_t cat_mask);

    /** @brief Invalidate the word cache and parsed SII cache. */
    void invalidateCache();

    /** @brief Set the timeout used for SII bus operations. */
    void setTimeoutMs(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

    /** @brief Current SII bus operation timeout, in milliseconds. */
    uint32_t timeoutMs() const { return timeout_ms_; }

    // -------------------------------------------------------------------------
    // Per-slave SII bus state (delegated to the owned SIIReader, which is
    // per-manager and therefore strictly per-slave).
    // -------------------------------------------------------------------------

    /** @brief Cached configured station address, or 0 if not yet determined. */
    uint16_t configuredStationAddr() const;

    /** @brief Set the cached configured station address. */
    void setConfiguredStationAddr(uint16_t addr);

    /** @brief True if this slave should use FPWR for EEPROM control writes. */
    bool useFpwr() const;

    /** @brief Mark this slave as needing FPWR for EEPROM control writes. */
    void setUseFpwr(bool use);

    /** @brief True if the EEPROM has already been forced to ECAT control. */
    bool eepromForcedToEcat() const;

    /** @brief Mark the EEPROM as forced to ECAT control. */
    void setEepromForcedToEcat(bool forced);

private:
    Master* master_ = nullptr;
    uint16_t slave_index_ = 0;
    uint32_t timeout_ms_ = 500;

    SIISlaveCache cache_;

    // Serializes bus operations (EEPROM reads) on this slave so concurrent
    // callers do not interleave register accesses. Cache reads (which only
    // touch cache_) do not need this mutex.
    std::mutex bus_mutex_;

    // Parsed SII cache (guarded by bus_mutex_).
    SIIData cached_data_{};
    bool full_parse_done_ = false;
    uint32_t cached_cat_mask_ = 0;  ///< Category mask for cached_data_ (0 = full parse)

    // Low-level SII bus reader owned by this manager. One reader per manager
    // means the SIIReader's configured-address / FPWR / force-to-ECAT state
    // is strictly per-slave and protected by the reader's own state_mutex_.
    std::unique_ptr<SIIReader> reader_;
};

} // namespace SII
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
