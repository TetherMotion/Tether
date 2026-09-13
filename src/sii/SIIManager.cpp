/**
 * @file SIIManager.cpp
 * @brief Per-slave SII manager implementation
 */

#include "tether/sii/SIIManager.hpp"

#if TETHER_ENABLE_SII

#include "tether/ethercat/Master.hpp"
#include "tether/sii/SIIDemandParser.hpp"
#include "ethercat/raw/internal.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {
namespace SII {

static const char* TAG = "sii_manager";

// ============================================================================
// Construction / binding
// ============================================================================

SIIManager::SIIManager(Master& master, uint16_t slave_index)
    : master_(&master)
    , slave_index_(slave_index)
    , reader_(std::make_unique<SIIReader>(master))
{
    reader_->setTimeout(master.siiTimeoutMs());
}

SIIManager::~SIIManager()
{
    // Clear the back-pointer to the owning master and release the low-level
    // reader before this child object finishes destruction.
    master_ = nullptr;
    reader_.reset();
}

void SIIManager::init(Master& master, uint16_t slave_index)
{
    master_ = &master;
    slave_index_ = slave_index;
    reader_ = std::make_unique<SIIReader>(master);
    reader_->setTimeout(master.siiTimeoutMs());
    cache_.clear();
    cached_data_ = SIIData{};
    full_parse_done_ = false;
    cached_cat_mask_ = 0;
}

// ============================================================================
// Per-slave SII bus state (delegated to the owned SIIReader)
// ============================================================================

uint16_t SIIManager::configuredStationAddr() const
{
    return reader_ ? reader_->configuredStationAddr() : 0;
}

void SIIManager::setConfiguredStationAddr(uint16_t addr)
{
    // No-op: the configured station address is managed internally by the
    // SIIReader during EEPROM access. This setter is retained for API
    // compatibility but has no effect on the reader's authoritative state.
    (void)addr;
}

bool SIIManager::useFpwr() const
{
    return reader_ ? reader_->useFpwr() : false;
}

void SIIManager::setUseFpwr(bool use)
{
    (void)use;  // Managed internally by SIIReader
}

bool SIIManager::eepromForcedToEcat() const
{
    return reader_ ? reader_->eepromForcedToEcat() : false;
}

void SIIManager::setEepromForcedToEcat(bool forced)
{
    (void)forced;  // Managed internally by SIIReader
}

// ============================================================================
// Cached word / dword / block reads
// ============================================================================

bool SIIManager::readWord(uint16_t word_addr, uint16_t& out)
{
    if (!reader_ || !master_) return false;
    // Fast path: cache hit — no bus access, no bus mutex needed.
    if (cache_.get(word_addr, out)) return true;
    // Cache miss — serialize bus access.
    std::lock_guard<std::mutex> lock(bus_mutex_);
    // Re-check cache after acquiring the lock (another thread may have
    // populated it while we waited).
    if (cache_.get(word_addr, out)) return true;
    return reader_->readWord(slave_index_, word_addr, out);
}

bool SIIManager::readDWord(uint16_t word_addr, uint32_t& out)
{
    if (!reader_ || !master_) return false;
    // Fast path: both words cached.
    const uint16_t wa = static_cast<uint16_t>(word_addr & 0xFFFEu);
    uint16_t lo = 0, hi = 0;
    if (cache_.get(wa, lo) && cache_.get(static_cast<uint16_t>(wa + 1), hi)) {
        out = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
        return true;
    }
    std::lock_guard<std::mutex> lock(bus_mutex_);
    // Re-check cache.
    if (cache_.get(wa, lo) && cache_.get(static_cast<uint16_t>(wa + 1), hi)) {
        out = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
        return true;
    }
    return reader_->readDWord(slave_index_, word_addr, out);
}

size_t SIIManager::readWords(uint16_t word_addr, uint16_t* buffer, size_t word_count)
{
    if (!reader_ || !master_) return 0;
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return reader_->readWords(slave_index_, word_addr, buffer, word_count);
}

size_t SIIManager::readBytes(uint16_t byte_addr, uint8_t* buffer, size_t byte_count)
{
    if (!reader_ || !master_) return 0;
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return reader_->readBytes(slave_index_, byte_addr, buffer, byte_count);
}

size_t SIIManager::prefetchWords(uint16_t word_addr, uint16_t word_count)
{
    if (!reader_ || !master_) return 0;
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return reader_->prefetchWords(slave_index_, word_addr, word_count);
}

bool SIIManager::readString(uint8_t string_index, char* buffer, size_t buffer_size)
{
    if (!reader_ || !master_) return false;
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return reader_->readString(slave_index_, string_index, buffer, buffer_size);
}

// ============================================================================
// Full parse
// ============================================================================

bool SIIManager::parseFull(SIIData& data)
{
    return parseCategories(data, SII::CAT_MASK_ALL);
}

bool SIIManager::parseCategories(SIIData& data, uint32_t cat_mask)
{
    if (!reader_ || !master_) return false;

    std::lock_guard<std::mutex> lock(bus_mutex_);
    // Return cached result if the mask matches (or a full parse was done
    // and the requested mask is a subset of CAT_MASK_ALL).
    if (full_parse_done_) {
        data = cached_data_;
        return true;
    }
    if (cached_cat_mask_ == cat_mask && cached_cat_mask_ != 0) {
        data = cached_data_;
        return true;
    }

    // Use the demand-driven parser, fetching missing word-pairs via the
    // blocking SII reader. This shares the same parsing logic as the
    // EEPROMReactor (which uses SIIDemandParser directly against the
    // cache with concurrent bus reads).
    //
    // The loop iterates:
    //   1. Call the demand parser against the cache.
    //   2. If COMPLETE, we're done.
    //   3. If NEED_WORDS, fetch each missing word-pair via readDWord
    //      (which populates the cache), then loop.
    //   4. If FAILED, report error.
    SIIDemandParser parser;
    parser.init(cat_mask);

    constexpr int max_iterations = 4096;  // Safety valve
    for (int i = 0; i < max_iterations; ++i) {
        auto result = parser.parse(cache_, data);

        if (result.isComplete()) {
            cached_data_ = data;
            cached_cat_mask_ = cat_mask;
            if (cat_mask == SII::CAT_MASK_ALL) {
                full_parse_done_ = true;
            }
            return true;
        }

        if (result.isFailed()) {
            TETHER_LOGW(TAG, "SII demand parse failed for slave {}: {}",
                        slave_index_, parser.lastError());
            return false;
        }

        // NEED_WORDS — fetch each missing word-pair via the blocking reader.
        // readDWord() populates the cache, so the next parse() call will
        // find the words present and advance.
        for (uint16_t addr : result.needed_word_pairs) {
            uint32_t dword = 0;
            if (!reader_->readDWord(slave_index_, addr, dword)) {
                TETHER_LOGW(TAG, "SII readDWord failed at 0x{:04X} for slave {}",
                            addr, slave_index_);
                return false;
            }
        }
    }

    TETHER_LOGW(TAG, "SII demand parse exceeded {} iterations for slave {}",
                max_iterations, slave_index_);
    return false;
}

void SIIManager::invalidateCache()
{
    cache_.clear();
    std::lock_guard<std::mutex> lock(bus_mutex_);
    cached_data_ = SIIData{};
    full_parse_done_ = false;
    cached_cat_mask_ = 0;
}

} // namespace SII
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
