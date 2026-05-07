/**
 * @file CachedSIIReader.hpp
 * @brief SII EEPROM reader with per-slave word-level cache
 *
 * @details
 * CachedSIIReader wraps `SII::SIIReader` and maintains a per-slave cache of
 * previously read EEPROM words.  Repeated reads of the same word address
 * return the cached value without issuing a new bus transaction.
 *
 * The cache can be invalidated explicitly with `invalidate()`.
 *
 * ## Usage
 * @code
 *   SII::CachedSIIReader cached(reader, 0);
 *   uint16_t word;
 *   cached.readWord(0x0008, word);   // bus read
 *   cached.readWord(0x0008, word);   // cache hit — no bus traffic
 * @endcode
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <unordered_map>

#include "tether/sii/SIIReader.hpp"

namespace EtherCAT {
namespace SII {

/**
 * @brief SII EEPROM reader with word-level cache.
 *
 * Each CachedSIIReader is associated with a single slave index so that the
 * caller doesn't need to pass the slave index on every read.
 */
class CachedSIIReader {
public:
    /**
     * @brief Default-construct (deferred init).
     *
     * Must call `init()` before use.
     */
    CachedSIIReader() = default;

    /**
     * @brief Construct from an existing SIIReader and slave index.
     * @param reader       Underlying SII reader
     * @param slave_index  Slave index this cache is bound to
     */
    CachedSIIReader(SIIReader& reader, uint16_t slave_index)
        : reader_(&reader), slave_index_(slave_index) {}

    /**
     * @brief Deferred initialisation.
     * @param reader       Underlying SII reader
     * @param slave_index  Slave index
     */
    void init(SIIReader& reader, uint16_t slave_index) {
        reader_ = &reader;
        slave_index_ = slave_index;
    }

    /**
     * @brief Read a 16-bit word from the EEPROM (cached).
     *
     * @param word_addr  EEPROM word address
     * @param[out] out   Output value
     * @return true on success (cache hit or bus read succeeded)
     */
    bool readWord(uint16_t word_addr, uint16_t& out) {
        auto it = cache_.find(word_addr);
        if (it != cache_.end()) {
            out = it->second;
            return true;
        }
        if (!reader_) return false;
        if (!reader_->readWord(slave_index_, word_addr, out)) return false;
        cache_[word_addr] = out;
        return true;
    }

    /**
     * @brief Read a 32-bit double-word from the EEPROM (cached).
     *
     * Cached as two 16-bit words.
     *
     * @param word_addr  EEPROM word address (low half)
     * @param[out] out   Output value
     * @return true on success
     */
    bool readDWord(uint16_t word_addr, uint32_t& out) {
        uint16_t lo = 0, hi = 0;
        if (!readWord(word_addr, lo)) return false;
        if (!readWord(static_cast<uint16_t>(word_addr + 1), hi)) return false;
        out = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
        return true;
    }

    /**
     * @brief Read multiple words (each individually cached).
     *
     * @param word_addr   Start address
     * @param buffer      Output word array
     * @param word_count  Number of words to read
     * @return Number of words successfully read
     */
    size_t readWords(uint16_t word_addr, uint16_t* buffer, size_t word_count) {
        for (size_t i = 0; i < word_count; ++i) {
            if (!readWord(static_cast<uint16_t>(word_addr + i), buffer[i]))
                return i;
        }
        return word_count;
    }

    /**
     * @brief Read a string from SII (not cached — delegates directly).
     *
     * @param string_index  String index (1-based)
     * @param buf           Output buffer
     * @param buf_size      Buffer capacity
     * @return true on success
     */
    bool readString(uint8_t string_index, char* buf, size_t buf_size) {
        if (!reader_) return false;
        return reader_->readString(slave_index_, string_index, buf, buf_size);
    }

    /**
     * @brief Parse full SII using the cache.
     *
     * Uses a local `SIIParser` with the underlying reader.
     * The first call does a full parse; subsequent calls return the cached result.
     *
     * @param[out] data  Output parsed SII data
     * @return true on success
     */
    bool parseFull(SIIData& data) {
        if (full_parse_done_) {
            data = cached_sii_;
            return true;
        }
        if (!reader_) return false;
        SIIParser parser(*reader_);
        if (!parser.parse(slave_index_, data)) return false;
        cached_sii_ = data;
        full_parse_done_ = true;
        return true;
    }

    /** @brief Clear all cached data. */
    void invalidate() {
        cache_.clear();
        full_parse_done_ = false;
        cached_sii_ = SIIData{};
    }

    /** @brief Slave index bound to this cache. */
    uint16_t slaveIndex() const { return slave_index_; }

    /** @brief Number of cached words. */
    size_t cacheSize() const { return cache_.size(); }

    /** @brief Check if full SII has been parsed. */
    bool isFullParseDone() const { return full_parse_done_; }

    /** @brief Check if a reader has been assigned. */
    bool isInitialized() const { return reader_ != nullptr; }

private:
    SIIReader* reader_ = nullptr;
    uint16_t slave_index_ = 0;

    std::unordered_map<uint16_t, uint16_t> cache_;
    SIIData cached_sii_{};
    bool full_parse_done_ = false;
};

} // namespace SII
} // namespace EtherCAT
