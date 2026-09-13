/**
 * @file SIIDemandParser.cpp
 * @brief Implementation of the demand-driven SII parser
 *
 * @details
 * This file implements SIIDemandParser and the shared category-data
 * parsing functions (parseStringsFromBuffer, parseGeneralFromBuffer, etc.)
 * that are reused by both SIIParser (bus-backed) and SIIDemandParser
 * (cache-backed).
 */

#include "tether/sii/SIIDemandParser.hpp"

#if TETHER_ENABLE_SII

#include "tether/sii/SIIManager.hpp"
#include "logging/Logger.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace EtherCAT {
namespace SII {

static const char* TAG = "sii_demand";

// ============================================================================
// Shared category-data parsing functions
// ============================================================================
//
// These functions parse category data from a raw byte buffer. They are
// extracted from the original SIIParser methods (parseStrings, parseGeneral,
// etc.) so that both the blocking SIIParser and the demand-driven
// SIIDemandParser can share the same parsing logic.
//
// The original SIIParser methods read bytes via m_reader.readBytes() and
// then parsed them. These free functions take the already-read bytes as
// a parameter, decoupling parsing from I/O.

bool parseStringsFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out) {
    if (size_bytes < 1) return true;  // Empty category is valid

    // First byte is number of strings
    uint8_t num_strings = data[0];
    size_t pos = 1;

    for (uint8_t i = 0; i < num_strings && pos < size_bytes; i++) {
        uint8_t str_len = data[pos++];

        if (str_len > 0) {
            size_t read_len = (str_len < 255) ? str_len : 255;
            if (pos + read_len > size_bytes) {
                read_len = size_bytes - pos;
            }
            char temp[256];
            std::memcpy(temp, data + pos, read_len);
            temp[read_len] = '\0';
            out.strings.addString(temp, read_len);
            pos += str_len;
        } else {
            out.strings.addString("", 0);
        }
    }

    return true;
}

bool parseGeneralFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out) {
    if (size_bytes < 16) return false;

    out.general.group_idx      = data[0];
    out.general.image_idx     = data[1];
    out.general.order_idx     = data[2];
    out.general.name_idx       = data[3];
    out.general.reserved       = data[4];
    out.general.coe_details    = data[5];
    out.general.foe_details    = data[6];
    out.general.eoe_details    = data[7];
    out.general.soe_channels   = data[8];
    out.general.ds402_channels = data[9];
    out.general.sys_man_class  = data[10];
    out.general.flags          = data[11];
    out.general.current_ebus   = static_cast<int16_t>(data[12] | (data[13] << 8));
    out.general.group_idx2     = data[14];
    out.general.reserved2      = data[15];

    if (size_bytes >= 18) {
        out.general.phys_port = static_cast<uint16_t>(data[16] | (data[17] << 8));
    }
    if (size_bytes >= 20) {
        out.general.phys_mem_addr = static_cast<uint16_t>(data[18] | (data[19] << 8));
    }

    out.has_general = true;
    return true;
}

bool parseFMMUFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out) {
    size_t num_fmmus = (size_bytes < 8) ? size_bytes : 8;

    out.fmmu_count = num_fmmus;
    for (size_t i = 0; i < num_fmmus; i++) {
        out.fmmus[i].fmmu_type = data[i];
    }
    return true;
}

bool parseSyncManagerFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out) {
    size_t num_sms = size_bytes / 8;
    if (num_sms > 8) num_sms = 8;

    for (size_t i = 0; i < num_sms; i++) {
        const uint8_t* sm = data + i * 8;
        out.sync_managers[i].phys_start_address =
            static_cast<uint16_t>(sm[0] | (sm[1] << 8));
        out.sync_managers[i].length =
            static_cast<uint16_t>(sm[2] | (sm[3] << 8));
        out.sync_managers[i].control_register =
            std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm[4]);
        out.sync_managers[i].status_register =
            std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(sm[5]);
        out.sync_managers[i].enable =
            std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(sm[6]);
        out.sync_managers[i].sm_type = sm[7];
        out.sm_count = i + 1;
    }
    return true;
}

bool parsePDOFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out, bool is_tx) {
    size_t pos = 0;

    while (pos + 8 <= size_bytes) {
        SIIPDO pdo;

        pdo.pdo_index   = static_cast<uint16_t>(data[pos] | (data[pos+1] << 8));
        pdo.n_entries   = data[pos+2];
        pdo.sync_manager = data[pos+3];
        pdo.dc_sync     = data[pos+4];
        pdo.name_idx    = data[pos+5];
        pdo.flags       = static_cast<uint16_t>(data[pos+6] | (data[pos+7] << 8));
        pos += 8;

        for (uint8_t i = 0; i < pdo.n_entries && pos + 8 <= size_bytes; i++) {
            SIIPDOEntry entry;
            entry.index      = static_cast<uint16_t>(data[pos] | (data[pos+1] << 8));
            entry.subindex   = data[pos+2];
            entry.name_idx    = data[pos+3];
            entry.data_type   = data[pos+4];
            entry.bit_length  = data[pos+5];
            entry.flags       = static_cast<uint16_t>(data[pos+6] | (data[pos+7] << 8));
            pos += 8;
            pdo.entries.push_back(entry);
        }

        if (is_tx) {
            out.tx_pdos.push_back(pdo);
        } else {
            out.rx_pdos.push_back(pdo);
        }
    }
    return true;
}

bool parseDCFromBuffer(const uint8_t* data, size_t size_bytes, SIIData& out) {
    size_t pos = 0;

    while (pos + 24 <= size_bytes) {
        SIIDCConfig dc;
        dc.cycle_time_0 = static_cast<uint32_t>(data[pos]) |
                          (static_cast<uint32_t>(data[pos+1]) << 8) |
                          (static_cast<uint32_t>(data[pos+2]) << 16) |
                          (static_cast<uint32_t>(data[pos+3]) << 24);
        dc.shift_time_0 = static_cast<uint32_t>(data[pos+4]) |
                          (static_cast<uint32_t>(data[pos+5]) << 8) |
                          (static_cast<uint32_t>(data[pos+6]) << 16) |
                          (static_cast<uint32_t>(data[pos+7]) << 24);
        dc.shift_time_1 = static_cast<uint32_t>(data[pos+8]) |
                          (static_cast<uint32_t>(data[pos+9]) << 8) |
                          (static_cast<uint32_t>(data[pos+10]) << 16) |
                          (static_cast<uint32_t>(data[pos+11]) << 24);
        dc.cycle_time_1_factor = static_cast<int16_t>(data[pos+12] | (data[pos+13] << 8));
        dc.assign_activate     = static_cast<uint16_t>(data[pos+14] | (data[pos+15] << 8));
        dc.cycle_time_0_factor = static_cast<int16_t>(data[pos+16] | (data[pos+17] << 8));
        dc.name_idx            = data[pos+18];
        dc.desc_idx            = data[pos+19];
        out.dc_configs.push_back(dc);
        pos += 24;
    }
    return true;
}

// ============================================================================
// SIIDemandParser — cache reading helpers
// ============================================================================

bool SIIDemandParser::readWordPair(const SIISlaveCache& cache, uint16_t addr, uint32_t& out) {
    return cache.getWordPair(addr, out);
}

bool SIIDemandParser::readWord(const SIISlaveCache& cache, uint16_t addr, uint16_t& out) {
    // Align to even and read the word-pair
    uint16_t aligned = addr & 0xFFFEu;
    uint32_t dword;
    if (!cache.getWordPair(aligned, dword)) return false;
    out = (addr & 1) ? static_cast<uint16_t>((dword >> 16) & 0xFFFF)
                     : static_cast<uint16_t>(dword & 0xFFFF);
    return true;
}

size_t SIIDemandParser::readBytes(const SIISlaveCache& cache, uint16_t byte_addr,
                                   uint8_t* buffer, size_t byte_count) {
    size_t bytes_read = 0;
    while (bytes_read < byte_count) {
        uint16_t word_addr = static_cast<uint16_t>((byte_addr + bytes_read) >> 1);
        uint16_t aligned   = static_cast<uint16_t>(word_addr & 0xFFFEu);

        uint32_t dword;
        if (!cache.getWordPair(aligned, dword)) break;

        uint8_t bytes[4] = {
            static_cast<uint8_t>(dword & 0xFF),
            static_cast<uint8_t>((dword >> 8) & 0xFF),
            static_cast<uint8_t>((dword >> 16) & 0xFF),
            static_cast<uint8_t>((dword >> 24) & 0xFF),
        };

        size_t offset = (byte_addr + bytes_read) % 4;
        for (size_t j = offset; j < 4 && bytes_read < byte_count; j++) {
            buffer[bytes_read++] = bytes[j];
        }
    }
    return bytes_read;
}

uint16_t SIIDemandParser::firstMissingWordPair(const SIISlaveCache& cache,
                                                uint16_t start, uint16_t word_count) {
    // Check each individual word in the range. The range may start at an
    // odd word address (when the previous category had an odd word count),
    // so we must check per-word, not per-word-pair. We return the aligned
    // word-pair address that contains the first missing word.
    for (uint16_t i = 0; i < word_count; i++) {
        uint16_t word_addr = static_cast<uint16_t>(start + i);
        uint16_t pair_addr  = static_cast<uint16_t>(word_addr & 0xFFFEu);
        uint32_t dummy;
        if (!cache.getWordPair(pair_addr, dummy)) {
            return pair_addr;
        }
    }
    return 0xFFFF;  // All cached
}

// ============================================================================
// SIIDemandParser — initialization
// ============================================================================

void SIIDemandParser::init(uint32_t cat_mask) {
    phase_            = Phase::CONFIG_AREA;
    cat_mask_         = cat_mask;
    cat_scan_addr_   = SII_CATEGORY_START;
    category_count_  = 0;
    last_error_[0]   = '\0';

    // Implicit dependency: categories that reference string indices
    // need the strings table.
    if ((cat_mask & (CAT_MASK_GENERAL | CAT_MASK_TXPDO | CAT_MASK_RXPDO)) != 0) {
        cat_mask_ |= CAT_MASK_STRINGS;
    }
}

// ============================================================================
// SIIDemandParser — error handling
// ============================================================================

void SIIDemandParser::fail(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(last_error_, sizeof(last_error_), fmt, args);
    va_end(args);
    phase_ = Phase::FAILED;
    TETHER_LOGE(TAG, "{}", last_error_);
}

void SIIDemandParser::collectMissing(const SIISlaveCache& cache,
                                      std::vector<uint16_t>& out,
                                      std::initializer_list<uint16_t> pairs) {
    for (uint16_t addr : pairs) {
        uint32_t dummy;
        if (!cache.getWordPair(addr, dummy)) {
            out.push_back(addr);
        }
    }
}

// ============================================================================
// SIIDemandParser — phase handlers
// ============================================================================

SIIDemandResult SIIDemandParser::doConfigArea(const SIISlaveCache& cache,
                                               SIIData& out) {
    // Need word-pairs 0x0000, 0x0002, 0x0004, 0x0006 (8 words = 4 pairs)
    std::vector<uint16_t> missing;
    collectMissing(cache, missing, {0x0000, 0x0002, 0x0004, 0x0006});

    if (!missing.empty()) {
        return {SIIDemandResult::NEED_WORDS, std::move(missing)};
    }

    // All 8 words are cached — read and parse
    uint16_t config[8];
    for (int i = 0; i < 8; ++i) {
        readWord(cache, static_cast<uint16_t>(i), config[i]);
    }

    out.pdi_control       = config[SII_PDI_CONTROL];
    out.pdi_config        = config[SII_PDI_CONFIG];
    out.sync_impulse_len  = config[SII_SYNC_IMPULSE_LEN];
    out.pdi_config2       = config[SII_PDI_CONFIG2];
    out.alias_address     = config[SII_ALIAS_ADDRESS];
    out.checksum          = config[SII_CHECKSUM];

    // CRC-8 validation (polynomial 0x07, init 0xFF, words 0..6)
    auto crc8_msb = [](uint8_t init, const uint8_t* data, size_t len) -> uint8_t {
        uint8_t crc = init;
        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (int b = 0; b < 8; ++b) {
                if (crc & 0x80) crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
                else            crc = static_cast<uint8_t>(crc << 1);
            }
        }
        return crc;
    };

    uint8_t conf_bytes[14];
    for (int i = 0; i < 7; ++i) {
        conf_bytes[i * 2]     = static_cast<uint8_t>(config[i] & 0xFF);
        conf_bytes[i * 2 + 1] = static_cast<uint8_t>((config[i] >> 8) & 0xFF);
    }

    uint8_t calc_crc  = crc8_msb(0xFF, conf_bytes, sizeof(conf_bytes));
    uint8_t stored_crc = static_cast<uint8_t>(config[SII_CHECKSUM] & 0xFF);
    out.checksum_ok   = (calc_crc == stored_crc);

    // Advance to IDENTITY
    phase_ = Phase::IDENTITY;
    return doIdentity(cache, out);
}

SIIDemandResult SIIDemandParser::doIdentity(const SIISlaveCache& cache, SIIData& out) {
    // Need word-pairs for:
    //   Vendor ID:       0x0008
    //   Product Code:    0x000A
    //   Revision:        0x000C
    //   Serial Number:   0x000E
    //   Mailbox config:  0x0014, 0x0016, 0x0018, 0x001A, 0x001C
    std::vector<uint16_t> missing;
    collectMissing(cache, missing,
        {0x0008, 0x000A, 0x000C, 0x000E,
         0x0014, 0x0016, 0x0018, 0x001A, 0x001C});

    if (!missing.empty()) {
        return {SIIDemandResult::NEED_WORDS, std::move(missing)};
    }

    // All identity words are cached — read and parse
    uint32_t vendor_id = 0, product_code = 0, revision = 0, serial = 0;
    readWordPair(cache, SII_VENDOR_ID, vendor_id);
    readWordPair(cache, SII_PRODUCT_CODE, product_code);
    readWordPair(cache, SII_REVISION, revision);
    readWordPair(cache, SII_SERIAL_NUMBER, serial);

    out.identity.vendor_id       = vendor_id;
    out.identity.product_code    = product_code;
    out.identity.revision_number = revision;
    out.identity.serial_number   = serial;

    // Mailbox configuration (10 words from 0x0014)
    uint16_t mbx[10];
    for (int i = 0; i < 10; ++i) {
        readWord(cache, static_cast<uint16_t>(SII_BOOTSTRAP_RX_MBX_OFFSET + i), mbx[i]);
    }

    out.mailbox.bootstrap_rx_offset = mbx[0];
    out.mailbox.bootstrap_rx_size   = mbx[1];
    out.mailbox.bootstrap_tx_offset = mbx[2];
    out.mailbox.bootstrap_tx_size   = mbx[3];
    out.mailbox.std_rx_offset       = mbx[4];
    out.mailbox.std_rx_size         = mbx[5];
    out.mailbox.std_tx_offset       = mbx[6];
    out.mailbox.std_tx_size         = mbx[7];
    out.mailbox.protocols           = mbx[8];

    out.valid = true;

    // Advance to SIZE_INFO
    phase_ = Phase::SIZE_INFO;
    return doSizeInfo(cache, out);
}

SIIDemandResult SIIDemandParser::doSizeInfo(const SIISlaveCache& cache, SIIData& out) {
    // Need word-pair 0x002E (size_info + version)
    std::vector<uint16_t> missing;
    collectMissing(cache, missing, {SII_SIZE_INFO});

    if (!missing.empty()) {
        return {SIIDemandResult::NEED_WORDS, std::move(missing)};
    }

    uint16_t size_info = 0, version = 0;
    readWord(cache, SII_SIZE_INFO, size_info);
    readWord(cache, SII_VERSION, version);

    out.eeprom_size_kbits = size_info & 0xFF;
    if (out.eeprom_size_kbits > 0) {
        out.eeprom_size_words = static_cast<uint16_t>(
            static_cast<uint32_t>(out.eeprom_size_kbits) * 1024 / 16);
    }
    out.version = version;

    // Advance to CATEGORY_SCAN
    phase_ = Phase::CATEGORY_SCAN;
    return doCategoryScan(cache, out);
}

SIIDemandResult SIIDemandParser::doCategoryScan(const SIISlaveCache& cache,
                                                 SIIData& out) {
    const int max_categories = 64;  // Safety limit

    while (category_count_ < max_categories) {
        // EEPROM size boundary check
        if (out.eeprom_size_words > 0 && cat_scan_addr_ >= out.eeprom_size_words) {
            break;
        }

        // --- Read category header (2 words: cat_type at cat_scan_addr_,
        //     cat_size at cat_scan_addr_+1) ---
        // Use readWord() instead of readWordPair() because cat_scan_addr_
        // may be odd (when the previous category had an odd word count).
        // readWord() correctly extracts the right half of a word-pair.
        uint16_t cat_type = 0, cat_size = 0;
        if (!readWord(cache, cat_scan_addr_, cat_type)) {
            uint16_t pair = static_cast<uint16_t>(cat_scan_addr_ & 0xFFFEu);
            return {SIIDemandResult::NEED_WORDS, {pair}};
        }
        if (!readWord(cache, static_cast<uint16_t>(cat_scan_addr_ + 1), cat_size)) {
            uint16_t pair = static_cast<uint16_t>((cat_scan_addr_ + 1) & 0xFFFEu);
            return {SIIDemandResult::NEED_WORDS, {pair}};
        }

        // CAT_END or blank EEPROM → done
        if (cat_type == CAT_END || (cat_type == 0 && cat_size == 0)) {
            phase_ = Phase::COMPLETE;
            out.parse_complete = true;
            return {SIIDemandResult::COMPLETE, {}};
        }

        // --- Category data starts at cat_scan_addr_ + 2 ---
        uint16_t data_start = static_cast<uint16_t>(cat_scan_addr_ + 2);

        // Check if we need to parse this category
        bool need_data = false;
        switch (cat_type) {
            case CAT_STRINGS:      need_data = (cat_mask_ & CAT_MASK_STRINGS)  != 0; break;
            case CAT_GENERAL:      need_data = (cat_mask_ & CAT_MASK_GENERAL)  != 0; break;
            case CAT_FMMU:         need_data = (cat_mask_ & CAT_MASK_FMMU)     != 0; break;
            case CAT_SYNC_MANAGER: need_data = (cat_mask_ & CAT_MASK_SYNC_MGR) != 0; break;
            case CAT_TXPDO:        need_data = (cat_mask_ & CAT_MASK_TXPDO)     != 0; break;
            case CAT_RXPDO:        need_data = (cat_mask_ & CAT_MASK_RXPDO)     != 0; break;
            case CAT_DC:           need_data = (cat_mask_ & CAT_MASK_DC)        != 0; break;
            default:               need_data = false; break;
        }

        if (need_data && cat_size > 0) {
            // Check if all data words are cached
            uint16_t missing_addr = firstMissingWordPair(cache, data_start, cat_size);
            if (missing_addr != 0xFFFF) {
                return {SIIDemandResult::NEED_WORDS, {missing_addr}};
            }

            // All data is cached — read bytes and parse
            uint16_t data_bytes = static_cast<uint16_t>(cat_size * 2);
            // Use a heap buffer for large categories to avoid stack overflow
            std::vector<uint8_t> buf(data_bytes);
            size_t n = readBytes(cache, static_cast<uint16_t>(data_start * 2),
                                 buf.data(), data_bytes);
            if (n != data_bytes) {
                fail("Failed to read category {} data ({} of {} bytes)",
                     cat_type, n, data_bytes);
                return {SIIDemandResult::FAILED, {}};
            }

            // Dispatch to the shared parsing function
            switch (cat_type) {
                case CAT_STRINGS:
                    parseStringsFromBuffer(buf.data(), data_bytes, out);
                    break;
                case CAT_GENERAL:
                    parseGeneralFromBuffer(buf.data(), data_bytes, out);
                    break;
                case CAT_FMMU:
                    parseFMMUFromBuffer(buf.data(), data_bytes, out);
                    break;
                case CAT_SYNC_MANAGER:
                    parseSyncManagerFromBuffer(buf.data(), data_bytes, out);
                    break;
                case CAT_TXPDO:
                    parsePDOFromBuffer(buf.data(), data_bytes, out, true);
                    break;
                case CAT_RXPDO:
                    parsePDOFromBuffer(buf.data(), data_bytes, out, false);
                    break;
                case CAT_DC:
                    parseDCFromBuffer(buf.data(), data_bytes, out);
                    break;
                default:
                    // Unknown category type — skip
                    break;
            }
        }

        // Advance to next category
        cat_scan_addr_ = static_cast<uint16_t>(cat_scan_addr_ + 2 + cat_size);
        category_count_++;
    }

    // Reached max categories or EEPROM boundary
    phase_ = Phase::COMPLETE;
    out.parse_complete = true;
    return {SIIDemandResult::COMPLETE, {}};
}

// ============================================================================
// SIIDemandParser — main parse entry point
// ============================================================================

SIIDemandResult SIIDemandParser::parse(const SIISlaveCache& cache, SIIData& out_data) {
    if (phase_ == Phase::COMPLETE) {
        return {SIIDemandResult::COMPLETE, {}};
    }
    if (phase_ == Phase::FAILED) {
        return {SIIDemandResult::FAILED, {}};
    }

    switch (phase_) {
        case Phase::CONFIG_AREA:    return doConfigArea(cache, out_data);
        case Phase::IDENTITY:       return doIdentity(cache, out_data);
        case Phase::SIZE_INFO:      return doSizeInfo(cache, out_data);
        case Phase::CATEGORY_SCAN:  return doCategoryScan(cache, out_data);
        default:
            fail("Invalid phase {}", static_cast<int>(phase_));
            return {SIIDemandResult::FAILED, {}};
    }
}

} // namespace SII
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
