/**
 * @file SIIReader.cpp
 * @brief EtherCAT SII EEPROM Reader Implementation
 */

#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/Platform.hpp"
#include "ethercat/raw/internal.hpp"

#include <bit>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace EtherCAT {
namespace SII {

static const char* TAG = "sii_reader";

// ============================================================================
// EEPROM Register Definitions
// ============================================================================

// EEPROM control/status register
static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;
static constexpr uint16_t EC_REG_EEPSTAT  = 0x0502;  // Same as EEPCTL
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;

// EEPROM commands
static constexpr uint16_t EC_ECMD_NOP  = 0x0000;
static constexpr uint16_t EC_ECMD_READ = 0x0100;

// EEPROM status flags
static constexpr uint16_t EC_ESTAT_BUSY  = 0x8000;
static constexpr uint16_t EC_ESTAT_EMASK = 0x7800;
static constexpr uint16_t EC_ESTAT_NACK  = 0x2000;

// ============================================================================
// SIIReader Implementation
// ============================================================================

SIIReader::SIIReader(Master& master)
    : m_master(master)
{
}

uint16_t SIIReader::adpForSlave(uint16_t slave_index) {
    // ADP for auto-increment addressing
    // Slave 0 = 0x0000, Slave 1 = 0xFFFF (-1), etc.
    return (slave_index == 0) ? 0x0000 : static_cast<uint16_t>(0 - slave_index);
}

bool SIIReader::waitNotBusy(uint16_t slave_index, uint16_t* out_status, uint32_t* out_poll_iters) {
    uint32_t iters = 0;
    const int64_t deadline_us = Tether::Platform::Clock::instance().getMicroseconds() + static_cast<int64_t>(m_timeout_ms) * 1000LL;

    while (true) {
        if (Tether::Platform::Clock::instance().getMicroseconds() >= deadline_us) {
            if (out_poll_iters) *out_poll_iters = iters;
            return false;
        }

        uint16_t estat_le = 0;
        if (m_master.readRegister(Master::slaveAddressFromADP(adpForSlave(slave_index)), EC_REG_EEPSTAT, estat_le, 100)) {
            uint16_t estat = Raw::le16_to_host(estat_le);
            if (out_status) {
                *out_status = estat;
            }
            if ((estat & EC_ESTAT_BUSY) == 0) {
                if (out_poll_iters) *out_poll_iters = iters;
                return true;
            }
        }
        iters++;
        Tether::Platform::Clock::instance().delayMicroseconds(200);
    }
}

bool SIIReader::readRaw32(uint16_t slave_index, uint16_t word_address, uint32_t* out) {
    if (out) *out = 0;

    // Check the master-level per-slave cache before touching the bus.
    uint16_t lo = 0, hi = 0;
    const uint16_t wa = static_cast<uint16_t>(word_address & 0xFFFEu);
    if (m_master.getSIICachedWord(slave_index, wa, lo) &&
        m_master.getSIICachedWord(slave_index, static_cast<uint16_t>(wa + 1), hi)) {
        if (out) *out = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
        if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
            TETHER_LOGD(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X cache hit", slave_index, word_address);
        }
        return true;
    }

    uint16_t estat = 0;
    uint32_t pre_iters = 0;
    if (!waitNotBusy(slave_index, &estat, &pre_iters)) {
        if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
            TETHER_LOGW(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X pre-wait timed out (%u iters)",
                        slave_index, word_address, pre_iters);
        }
        return false;
    }

    // Clear errors if present
    if ((estat & EC_ESTAT_EMASK) != 0) {
        uint16_t nop_le = Raw::host_to_le16(EC_ECMD_NOP);
        m_master.writeRegister(Master::slaveAddressFromADP(adpForSlave(slave_index)), EC_REG_EEPCTL, nop_le, 200);
    }

    int nack_count = 0;
    do {
        // EEPROM read command structure
        struct __attribute__((packed)) EepromCmd {
            uint16_t comm_le;
            uint16_t addr_le;
            uint16_t d2_le;
        } cmd{};

        cmd.comm_le = Raw::host_to_le16(EC_ECMD_READ);
        cmd.addr_le = Raw::host_to_le16(word_address);
        cmd.d2_le = 0;

        if (!m_master.writeRegister(Master::slaveAddressFromADP(adpForSlave(slave_index)), EC_REG_EEPCTL, cmd, 200)) {
            if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
                TETHER_LOGW(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X writeRegister(EEPCTL) failed (nack=%d)",
                            slave_index, word_address, nack_count);
            }
            return false;
        }

        Tether::Platform::Clock::instance().delayMicroseconds(200);

        estat = 0;
        uint32_t post_iters = 0;
        if (!waitNotBusy(slave_index, &estat, &post_iters)) {
            if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
                TETHER_LOGW(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X post-wait timed out (%u iters, nack=%d)",
                            slave_index, word_address, post_iters, nack_count);
            }
            return false;
        }

        if ((estat & EC_ESTAT_NACK) != 0) {
            nack_count++;
            if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
                TETHER_LOGD(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X NACK retry %d/3", slave_index, word_address, nack_count);
            }
            Tether::Platform::Clock::instance().delayMicroseconds(1000);
            continue;
        }

        uint32_t edat_le = 0;
        if (!m_master.readRegister(Master::slaveAddressFromADP(adpForSlave(slave_index)), EC_REG_EEPDAT, edat_le, 200)) {
            if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
                TETHER_LOGW(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X readRegister(EEPDAT) failed (nack=%d)",
                            slave_index, word_address, nack_count);
            }
            return false;
        }

        if (out) {
            *out = Raw::le32_to_host(edat_le);
        }
        // Populate the master-level cache so future readers hit it.
        m_master.setSIICachedWord(slave_index, wa, static_cast<uint16_t>(edat_le & 0xFFFF));
        m_master.setSIICachedWord(slave_index, static_cast<uint16_t>(wa + 1), static_cast<uint16_t>((edat_le >> 16) & 0xFFFF));
        if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
            TETHER_LOGD(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X success pre=%u post=%u nack=%d",
                        slave_index, word_address, pre_iters, post_iters, nack_count);
        }
        return true;

    } while (nack_count > 0 && nack_count < 3);

    if (m_master.debugFlags().siiEeprom && m_master.debugFlags().siiEepromFilt.allows(slave_index)) {
        TETHER_LOGW(TAG, "SII EEPROM [slave %u]: readRaw32 addr=0x%04X failed after %d NACK retries", slave_index, word_address, nack_count);
    }
    return false;
}

bool SIIReader::readWord(uint16_t slave_index, uint16_t word_address, uint16_t& out) {
    uint32_t dword = 0;
    uint16_t aligned_addr = word_address & ~1u;  // Align to even address

    if (!readRaw32(slave_index, aligned_addr, &dword)) {
        return false;
    }
    
    // Extract the correct word
    if (word_address & 1) {
        out = static_cast<uint16_t>((dword >> 16) & 0xFFFF);
    } else {
        out = static_cast<uint16_t>(dword & 0xFFFF);
    }
    return true;
}

bool SIIReader::readDWord(uint16_t slave_index, uint16_t word_address, uint32_t& out) {
    return readRaw32(slave_index, word_address, &out);
}

size_t SIIReader::prefetchWords(uint16_t slave_index, uint16_t word_address, uint16_t count) {
    size_t prefetched = 0;
    uint32_t dummy = 0;
    // readRaw32() populates the master-level cache on success.
    for (uint16_t i = 0; i < count; i += 2) {
        uint16_t addr = static_cast<uint16_t>(word_address + i);
        if (readRaw32(slave_index, addr, &dummy)) {
            prefetched += 2;
        } else {
            break;  // Stop on first failure (past end of EEPROM, etc.)
        }
    }
    return prefetched;
}

size_t SIIReader::readWords(uint16_t slave_index, uint16_t word_address,
                            uint16_t* buffer, size_t word_count) {
    size_t words_read = 0;

    // Read 2 words at a time (32-bit reads)
    for (size_t i = 0; i < word_count; i += 2) {
        uint32_t dword = 0;
        if (!readRaw32(slave_index, static_cast<uint16_t>(word_address + i), &dword)) {
            break;
        }
        
        buffer[i] = static_cast<uint16_t>(dword & 0xFFFF);
        words_read++;
        
        if (i + 1 < word_count) {
            buffer[i + 1] = static_cast<uint16_t>((dword >> 16) & 0xFFFF);
            words_read++;
        }
    }
    
    return words_read;
}

size_t SIIReader::readBytes(uint16_t slave_index, uint16_t byte_address,
                            uint8_t* buffer, size_t byte_count) {
    size_t bytes_read = 0;

    while (bytes_read < byte_count) {
        uint16_t word_addr = static_cast<uint16_t>((byte_address + bytes_read) >> 1);
        uint16_t aligned_word = word_addr & ~1u;

        uint32_t dword = 0;
        if (!readRaw32(slave_index, aligned_word, &dword)) {
            break;
        }
        
        // Extract bytes from the 32-bit read
        uint8_t bytes[4];
        bytes[0] = static_cast<uint8_t>(dword & 0xFF);
        bytes[1] = static_cast<uint8_t>((dword >> 8) & 0xFF);
        bytes[2] = static_cast<uint8_t>((dword >> 16) & 0xFF);
        bytes[3] = static_cast<uint8_t>((dword >> 24) & 0xFF);
        
        // Calculate offset within the 4-byte block
        size_t start_offset = (byte_address + bytes_read) % 4;
        
        for (size_t j = start_offset; j < 4 && bytes_read < byte_count; j++) {
            buffer[bytes_read++] = bytes[j];
        }
    }
    
    return bytes_read;
}

bool SIIReader::readString(uint16_t slave_index, uint8_t string_index,
                           char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0 || string_index == 0) {
        return false;
    }
    buffer[0] = '\0';
    
    // Find string category
    uint16_t byte_addr = SII_CATEGORY_START * 2;  // Convert word address to byte address

    // Scan for string category
    while (true) {
        uint16_t cat_type = 0;
        uint16_t cat_size = 0;

        // Read category header (2 words = 4 bytes)
        uint32_t header = 0;
        if (!readRaw32(slave_index, byte_addr / 2, &header)) {
            return false;
        }
        
        cat_type = static_cast<uint16_t>(header & 0xFFFF);
        cat_size = static_cast<uint16_t>((header >> 16) & 0xFFFF);
        
        if (cat_type == CAT_END) {
            return false;  // String category not found
        }
        
        if (cat_type == CAT_STRINGS) {
            // Found strings category
            uint16_t str_byte_addr = byte_addr + 4;  // Skip category header
            
            // First byte is number of strings
            uint8_t num_strings = 0;
            if (readBytes(slave_index, str_byte_addr, &num_strings, 1) != 1) {
                return false;
            }
            str_byte_addr++;
            
            if (string_index > num_strings) {
                return false;  // String index out of range
            }
            
            // Skip to desired string
            for (uint8_t i = 1; i <= string_index; i++) {
                uint8_t str_len = 0;
                if (readBytes(slave_index, str_byte_addr, &str_len, 1) != 1) {
                    return false;
                }
                str_byte_addr++;
                
                if (i == string_index) {
                    // Read this string
                    size_t copy_len = (str_len < buffer_size - 1) ? str_len : buffer_size - 1;
                    if (readBytes(slave_index, str_byte_addr, 
                                  reinterpret_cast<uint8_t*>(buffer), copy_len) != copy_len) {
                        return false;
                    }
                    buffer[copy_len] = '\0';
                    return true;
                }
                
                str_byte_addr += str_len;
            }
            return false;
        }
        
        // Skip to next category
        byte_addr += 4 + (cat_size * 2);  // Header + content
    }
}

// ============================================================================
// SIIParser Implementation
// ============================================================================

SIIParser::SIIParser(SIIReader& reader)
    : m_reader(reader)
{
}

void SIIParser::setError(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(m_last_error, sizeof(m_last_error), fmt, args);
    va_end(args);
    TETHER_LOGE(TAG, "%s", m_last_error);
}

bool SIIParser::parseConfigArea(uint16_t slave_index, SIIData& out_data) {
    // Read configuration area (words 0x0000-0x0007)
    uint16_t config[8];
    size_t words_read = m_reader.readWords(slave_index, 0, config, 8);
    if (words_read != 8) {
        setError("Slave %u: Failed to read SII config area (addr=0x0000, requested=8, got=%zu)",
                 slave_index, words_read);
        return false;
    }

    out_data.pdi_control = config[SII_PDI_CONTROL];
    out_data.pdi_config = config[SII_PDI_CONFIG];
    out_data.sync_impulse_len = config[SII_SYNC_IMPULSE_LEN];
    out_data.pdi_config2 = config[SII_PDI_CONFIG2];
    out_data.alias_address = config[SII_ALIAS_ADDRESS];
    out_data.checksum = config[SII_CHECKSUM];

    // --- CRC-8 validation (SII spec: low byte of word 0x0007)
    // Polynomial: x^8 + x^2 + x + 1 (0x07), initial value 0xFF
    auto crc8_msb = [](uint8_t init, const uint8_t* data, size_t len) -> uint8_t {
        uint8_t crc = init;
        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (int b = 0; b < 8; ++b) {
                if (crc & 0x80) crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
                else crc = static_cast<uint8_t>(crc << 1);
            }
        }
        return crc;
    };

    uint8_t conf_bytes[14]; // words 0..6 => 7 * 2 bytes
    for (int i = 0; i < 7; ++i) {
        conf_bytes[i * 2]     = static_cast<uint8_t>(config[i] & 0xFF);
        conf_bytes[i * 2 + 1] = static_cast<uint8_t>((config[i] >> 8) & 0xFF);
    }

    uint8_t calc_crc = crc8_msb(0xFF, conf_bytes, sizeof(conf_bytes));
    uint8_t stored_crc = static_cast<uint8_t>(config[SII_CHECKSUM] & 0xFF);

    out_data.checksum = config[SII_CHECKSUM];
    out_data.checksum_ok = (calc_crc == stored_crc);

    if (!out_data.checksum_ok) {
        // Log an error but continue parsing (backwards-compatible)
        setError("Slave %u: SII config CRC mismatch: expected=0x%02X calculated=0x%02X "
                 "(raw words: %04X %04X %04X %04X %04X %04X %04X %04X)",
                 slave_index,
                 static_cast<unsigned>(stored_crc), static_cast<unsigned>(calc_crc),
                 config[0], config[1], config[2], config[3],
                 config[4], config[5], config[6], config[7]);
    }

    return true;
}

bool SIIParser::parseIdentity(uint16_t slave_index, SIIData& out_data) {
    // Parse config area first
    if (!parseConfigArea(slave_index, out_data)) {
        return false;
    }
    
    // Read identity (words 0x0008-0x000F)
    uint32_t vendor_id = 0, product_code = 0, revision = 0, serial = 0;
    
    if (!m_reader.readDWord(slave_index, SII_VENDOR_ID, vendor_id)) {
        setError("Slave %u: Failed to read SII vendor ID (addr=0x%04X)", slave_index, SII_VENDOR_ID);
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_PRODUCT_CODE, product_code)) {
        setError("Slave %u: Failed to read SII product code (addr=0x%04X)", slave_index, SII_PRODUCT_CODE);
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_REVISION, revision)) {
        setError("Slave %u: Failed to read SII revision (addr=0x%04X)", slave_index, SII_REVISION);
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_SERIAL_NUMBER, serial)) {
        setError("Slave %u: Failed to read SII serial number (addr=0x%04X)", slave_index, SII_SERIAL_NUMBER);
        return false;
    }
    
    out_data.identity.vendor_id = vendor_id;
    out_data.identity.product_code = product_code;
    out_data.identity.revision_number = revision;
    out_data.identity.serial_number = serial;
    
    // Read mailbox configuration
    uint16_t mbx_data[10];
    size_t mbx_words = m_reader.readWords(slave_index, SII_BOOTSTRAP_RX_MBX_OFFSET, mbx_data, 10);
    if (mbx_words != 10) {
        setError("Slave %u: Failed to read SII mailbox config (addr=0x%04X, requested=10, got=%zu)",
                 slave_index, SII_BOOTSTRAP_RX_MBX_OFFSET, mbx_words);
        return false;
    }
    
    out_data.mailbox.bootstrap_rx_offset = mbx_data[0];
    out_data.mailbox.bootstrap_rx_size = mbx_data[1];
    out_data.mailbox.bootstrap_tx_offset = mbx_data[2];
    out_data.mailbox.bootstrap_tx_size = mbx_data[3];
    out_data.mailbox.std_rx_offset = mbx_data[4];
    out_data.mailbox.std_rx_size = mbx_data[5];
    out_data.mailbox.std_tx_offset = mbx_data[6];
    out_data.mailbox.std_tx_size = mbx_data[7];
    out_data.mailbox.protocols = mbx_data[8];
    
    out_data.valid = true;
    return true;
}

bool SIIParser::parseStrings(uint16_t slave_index, uint16_t byte_offset,
                             uint16_t size_bytes, SIIData& data) {
    if (size_bytes < 1) return true;  // Empty category is valid
    
    // First byte is number of strings
    uint8_t num_strings = 0;
    if (m_reader.readBytes(slave_index, byte_offset, &num_strings, 1) != 1) {
        return false;
    }
    
    uint16_t pos = byte_offset + 1;
    
    for (uint8_t i = 0; i < num_strings && pos < byte_offset + size_bytes; i++) {
        uint8_t str_len = 0;
        if (m_reader.readBytes(slave_index, pos, &str_len, 1) != 1) {
            break;
        }
        pos++;
        
        if (str_len > 0) {
            char temp[256];
            size_t read_len = (str_len < sizeof(temp) - 1) ? str_len : sizeof(temp) - 1;
            
            if (m_reader.readBytes(slave_index, pos, reinterpret_cast<uint8_t*>(temp), 
                                   read_len) == read_len) {
                temp[read_len] = '\0';
                data.strings.addString(temp, read_len);
            }
            pos += str_len;
        } else {
            data.strings.addString("", 0);
        }
    }
    
    return true;
}

bool SIIParser::parseGeneral(uint16_t slave_index, uint16_t byte_offset,
                             uint16_t size_bytes, SIIData& data) {
    if (size_bytes < 16) {
        setError("General category too small: %u bytes", size_bytes);
        return false;
    }
    
    uint8_t gen_data[32];
    size_t read_size = (size_bytes < sizeof(gen_data)) ? size_bytes : sizeof(gen_data);
    
    if (m_reader.readBytes(slave_index, byte_offset, gen_data, read_size) != read_size) {
        return false;
    }
    
    // Parse general info structure
    data.general.group_idx = gen_data[0];
    data.general.image_idx = gen_data[1];
    data.general.order_idx = gen_data[2];
    data.general.name_idx = gen_data[3];
    data.general.reserved = gen_data[4];
    data.general.coe_details = gen_data[5];
    data.general.foe_details = gen_data[6];
    data.general.eoe_details = gen_data[7];
    data.general.soe_channels = gen_data[8];
    data.general.ds402_channels = gen_data[9];
    data.general.sys_man_class = gen_data[10];
    data.general.flags = gen_data[11];
    data.general.current_ebus = static_cast<int16_t>(gen_data[12] | (gen_data[13] << 8));
    data.general.group_idx2 = gen_data[14];
    data.general.reserved2 = gen_data[15];
    
    if (read_size >= 18) {
        data.general.phys_port = static_cast<uint16_t>(gen_data[16] | (gen_data[17] << 8));
    }
    if (read_size >= 20) {
        data.general.phys_mem_addr = static_cast<uint16_t>(gen_data[18] | (gen_data[19] << 8));
    }
    
    data.has_general = true;
    return true;
}

bool SIIParser::parseFMMU(uint16_t slave_index, uint16_t byte_offset,
                          uint16_t size_bytes, SIIData& data) {
    // Each FMMU entry is 1 byte
    size_t num_fmmus = (size_bytes < 8) ? size_bytes : 8;
    
    uint8_t fmmu_data[8];
    if (m_reader.readBytes(slave_index, byte_offset, fmmu_data, num_fmmus) != num_fmmus) {
        return false;
    }
    
    data.fmmu_count = num_fmmus;
    for (size_t i = 0; i < num_fmmus; i++) {
        data.fmmus[i].fmmu_type = fmmu_data[i];
    }
    
    return true;
}

bool SIIParser::parseSyncManager(uint16_t slave_index, uint16_t byte_offset,
                                 uint16_t size_bytes, SIIData& data) {
    // Each SM entry is 8 bytes
    size_t num_sms = size_bytes / 8;
    if (num_sms > 8) num_sms = 8;
    
    for (size_t i = 0; i < num_sms; i++) {
        uint8_t sm_data[8];
        if (m_reader.readBytes(slave_index, static_cast<uint16_t>(byte_offset + i * 8), 
                               sm_data, 8) != 8) {
            break;
        }
        
        data.sync_managers[i].phys_start_address = static_cast<uint16_t>(sm_data[0] | (sm_data[1] << 8));
        data.sync_managers[i].length = static_cast<uint16_t>(sm_data[2] | (sm_data[3] << 8));
        data.sync_managers[i].control_register = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm_data[4]);
        data.sync_managers[i].status_register = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(sm_data[5]);
        data.sync_managers[i].enable = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(sm_data[6]);
        data.sync_managers[i].sm_type = sm_data[7];
        
        data.sm_count = i + 1;
    }
    
    return true;
}

bool SIIParser::parsePDO(uint16_t slave_index, uint16_t byte_offset,
                         uint16_t size_bytes, SIIData& data, bool is_tx) {
    uint16_t pos = byte_offset;
    uint16_t end = byte_offset + size_bytes;
    
    while (pos + 8 <= end) {
        SIIPDO pdo;
        
        // Read PDO header (8 bytes)
        uint8_t pdo_hdr[8];
        if (m_reader.readBytes(slave_index, pos, pdo_hdr, 8) != 8) {
            break;
        }
        pos += 8;
        
        pdo.pdo_index = static_cast<uint16_t>(pdo_hdr[0] | (pdo_hdr[1] << 8));
        pdo.n_entries = pdo_hdr[2];
        pdo.sync_manager = pdo_hdr[3];
        pdo.dc_sync = pdo_hdr[4];
        pdo.name_idx = pdo_hdr[5];
        pdo.flags = static_cast<uint16_t>(pdo_hdr[6] | (pdo_hdr[7] << 8));
        
        // Read PDO entries (8 bytes each)
        for (uint8_t i = 0; i < pdo.n_entries && pos + 8 <= end; i++) {
            uint8_t entry_data[8];
            if (m_reader.readBytes(slave_index, pos, entry_data, 8) != 8) {
                break;
            }
            pos += 8;
            
            SIIPDOEntry entry;
            entry.index = static_cast<uint16_t>(entry_data[0] | (entry_data[1] << 8));
            entry.subindex = entry_data[2];
            entry.name_idx = entry_data[3];
            entry.data_type = entry_data[4];
            entry.bit_length = entry_data[5];
            entry.flags = static_cast<uint16_t>(entry_data[6] | (entry_data[7] << 8));
            
            pdo.entries.push_back(entry);
        }
        
        if (is_tx) {
            data.tx_pdos.push_back(pdo);
        } else {
            data.rx_pdos.push_back(pdo);
        }
    }
    
    return true;
}

bool SIIParser::parseDC(uint16_t slave_index, uint16_t byte_offset,
                        uint16_t size_bytes, SIIData& data) {
    // Each DC entry is typically 24 bytes
    uint16_t pos = byte_offset;
    uint16_t end = byte_offset + size_bytes;
    
    while (pos + 24 <= end) {
        uint8_t dc_data[24];
        if (m_reader.readBytes(slave_index, pos, dc_data, 24) != 24) {
            break;
        }
        
        SIIDCConfig dc;
        dc.cycle_time_0 = static_cast<uint32_t>(dc_data[0] | (dc_data[1] << 8) | 
                                                 (dc_data[2] << 16) | (dc_data[3] << 24));
        dc.shift_time_0 = static_cast<uint32_t>(dc_data[4] | (dc_data[5] << 8) |
                                                 (dc_data[6] << 16) | (dc_data[7] << 24));
        dc.shift_time_1 = static_cast<uint32_t>(dc_data[8] | (dc_data[9] << 8) |
                                                 (dc_data[10] << 16) | (dc_data[11] << 24));
        dc.cycle_time_1_factor = static_cast<int16_t>(dc_data[12] | (dc_data[13] << 8));
        dc.assign_activate = static_cast<uint16_t>(dc_data[14] | (dc_data[15] << 8));
        dc.cycle_time_0_factor = static_cast<int16_t>(dc_data[16] | (dc_data[17] << 8));
        dc.name_idx = dc_data[18];
        dc.desc_idx = dc_data[19];
        
        data.dc_configs.push_back(dc);
        pos += 24;
    }
    
    return true;
}

bool SIIParser::parse(uint16_t slave_index, SIIData& out_data) {
    out_data.clear();
    
    // Parse identity first
    if (!parseIdentity(slave_index, out_data)) {
        return false;
    }
    
    // Read version info
    uint16_t version = 0;
    if (m_reader.readWord(slave_index, SII_VERSION, version)) {
        out_data.version = version;
    }
    
    // Parse category area
    uint16_t word_addr = SII_CATEGORY_START;
    int category_count = 0;
    const int max_categories = 64;  // Prevent infinite loops
    
    while (category_count < max_categories) {
        // Read category header (2 words)
        uint16_t cat_type = 0;
        uint16_t cat_size = 0;
        
        if (!m_reader.readWord(slave_index, word_addr, cat_type)) {
            setError("Failed to read category type at word 0x%04X", word_addr);
            break;
        }
        word_addr++;
        
        if (cat_type == CAT_END) {
            TETHER_LOGD(TAG, "End of categories at word 0x%04X", word_addr - 1);
            break;
        }
        
        if (!m_reader.readWord(slave_index, word_addr, cat_size)) {
            setError("Failed to read category size at word 0x%04X", word_addr);
            break;
        }
        word_addr++;
        
        // Category data starts here
        uint16_t data_byte_offset = word_addr * 2;
        uint16_t data_size_bytes = cat_size * 2;
        
        TETHER_LOGD(TAG, "Category %u: type=%u size=%u words at 0x%04X",
                 category_count, cat_type, cat_size, word_addr);
        
        // Parse category content
        switch (cat_type) {
            case CAT_STRINGS:
                parseStrings(slave_index, data_byte_offset, data_size_bytes, out_data);
                break;
            case CAT_GENERAL:
                parseGeneral(slave_index, data_byte_offset, data_size_bytes, out_data);
                break;
            case CAT_FMMU:
                parseFMMU(slave_index, data_byte_offset, data_size_bytes, out_data);
                break;
            case CAT_SYNC_MANAGER:
                parseSyncManager(slave_index, data_byte_offset, data_size_bytes, out_data);
                break;
            case CAT_TXPDO:
                parsePDO(slave_index, data_byte_offset, data_size_bytes, out_data, true);
                break;
            case CAT_RXPDO:
                parsePDO(slave_index, data_byte_offset, data_size_bytes, out_data, false);
                break;
            case CAT_DC:
                parseDC(slave_index, data_byte_offset, data_size_bytes, out_data);
                break;
            case CAT_NOP:
                // Skip NOP categories
                break;
            default:
                TETHER_LOGD(TAG, "Unknown category type %u, skipping", cat_type);
                break;
        }
        
        // Move to next category
        word_addr += cat_size;
        category_count++;
    }
    
    out_data.parse_complete = true;
    return true;
}

// ============================================================================
// Convenience Functions
// ============================================================================

bool readSII(Master& master, uint16_t slave_index, SIIData& out_data) {
    SIIReader reader(master);
    SIIParser parser(reader);
    return parser.parse(slave_index, out_data);
}

bool readSIIIdentity(Master& master, uint16_t slave_index, SIIIdentity& out_identity) {
    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;
    
    if (parser.parseIdentity(slave_index, data)) {
        out_identity = data.identity;
        return true;
    }
    return false;
}

bool readSIIMailbox(Master& master, uint16_t slave_index, SIIMailboxConfig& out_mailbox) {
    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;
    
    if (parser.parseIdentity(slave_index, data)) {
        out_mailbox = data.mailbox;
        return true;
    }
    return false;
}

// ============================================================================
// Utility Functions
// ============================================================================

const char* getCategoryTypeName(uint16_t type) {
    switch (type) {
        case CAT_NOP: return "NOP";
        case CAT_STRINGS: return "Strings";
        case CAT_DATA_TYPES: return "DataTypes";
        case CAT_GENERAL: return "General";
        case CAT_FMMU: return "FMMU";
        case CAT_SYNC_MANAGER: return "SyncManager";
        case CAT_FMMU_EX: return "FMMU_EX";
        case CAT_SYNC_UNIT: return "SyncUnit";
        case CAT_TXPDO: return "TxPDO";
        case CAT_RXPDO: return "RxPDO";
        case CAT_DC: return "DC";
        case CAT_END: return "End";
        default: return "Unknown";
    }
}

const char* getMailboxProtocolName(uint16_t protocol) {
    static char buf[64];
    buf[0] = '\0';
    size_t buf_len = 0;
    
    if (protocol & MBX_PROTO_AOE) {
        strncat(buf, "AoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_EOE) {
        strncat(buf, "EoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_COE) {
        strncat(buf, "CoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_FOE) {
        strncat(buf, "FoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_SOE) {
        strncat(buf, "SoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_VOE) {
        strncat(buf, "VoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    
    if (buf[0] == '\0') {
        return "None";
    }
    
    // Remove trailing space
    if (buf_len > 0 && buf[buf_len-1] == ' ') {
        buf[buf_len-1] = '\0';
    }
    
    return buf;
}

// ============================================================================
// Logging Functions
// ============================================================================

#include "tether/platform/Platform.hpp"

void logSIIIdentity(const SIIIdentity& identity, const char* tag) {
    TETHER_LOGI(tag, "SII Identity:\n  Vendor ID:    0x%08" PRIx32 "\n  Product Code: 0x%08" PRIx32 "\n  Revision:     %u.%u (0x%08" PRIx32 ")\n  Serial:       0x%08" PRIx32,
             identity.vendor_id, identity.product_code,
             identity.revisionMajor(), identity.revisionMinor(), identity.revision_number,
             identity.serial_number);
} 

void logSIIMailbox(const SIIMailboxConfig& mailbox, const char* tag) {
    TETHER_LOGI(tag, "SII Mailbox Configuration:");

    if (mailbox.hasMailbox()) {
        TETHER_LOGI(tag, "  Standard Mailbox:\n    RX (MbxIn — Master->Slave, SM0): addr=0x%04X size=%u\n    TX (MbxOut — Slave->Master, SM1): addr=0x%04X size=%u",
                 mailbox.std_rx_offset, mailbox.std_rx_size, mailbox.std_tx_offset, mailbox.std_tx_size);
    } else {
        TETHER_LOGI(tag, "  Standard Mailbox: Not configured");
    }

    if (mailbox.hasBootstrapMailbox()) {
        TETHER_LOGI(tag, "  Bootstrap Mailbox:\n    RX (MbxIn — Master->Slave, SM0): addr=0x%04X size=%u\n    TX (MbxOut — Slave->Master, SM1): addr=0x%04X size=%u",
                 mailbox.bootstrap_rx_offset, mailbox.bootstrap_rx_size, mailbox.bootstrap_tx_offset, mailbox.bootstrap_tx_size);
    } else {
        TETHER_LOGI(tag, "  Bootstrap Mailbox: Not configured");
    }

    TETHER_LOGI(tag, "  Protocols: 0x%04X (%s)",
             mailbox.protocols, getMailboxProtocolName(mailbox.protocols));
}

void logSIISyncManagers(const SIIData& data, const char* tag) {
    TETHER_LOGI(tag, "SII Sync Managers (%zu configured):", data.sm_count);
    
    for (size_t i = 0; i < data.sm_count; i++) {
        const auto& sm = data.sync_managers[i];
        TETHER_LOGI(tag, "  SM%zu: %s\n    Address: 0x%04X  Length: %u bytes", i, sm.getTypeName(), sm.phys_start_address, sm.length);
        TETHER_LOGI(tag, "    Control: 0x%02X  Enable: 0x%02X %s",
                 sm.control_register, sm.enable,
                 sm.isEnabled() ? "[ENABLED]" : "[DISABLED]");
    }
}

void logSIIPDOs(const SIIData& data, const char* tag) {
    if (!data.tx_pdos.empty()) {
        TETHER_LOGI(tag, "SII TxPDOs (Slave→Master):");
        for (const auto& pdo : data.tx_pdos) {
            const char* name = data.strings.getString(pdo.name_idx);
            TETHER_LOGI(tag, "  PDO 0x%04X '%s' SM%u (%zu entries, %zu bits)",
                     pdo.pdo_index, name, pdo.sync_manager,
                     pdo.entries.size(), pdo.totalBits());
            
            for (const auto& entry : pdo.entries) {
                const char* entry_name = data.strings.getString(entry.name_idx);
                TETHER_LOGI(tag, "    0x%04X:%02X '%s' %u bits",
                         entry.index, entry.subindex, entry_name, entry.bit_length);
            }
        }
    }
    
    if (!data.rx_pdos.empty()) {
        TETHER_LOGI(tag, "SII RxPDOs (Master→Slave):");
        for (const auto& pdo : data.rx_pdos) {
            const char* name = data.strings.getString(pdo.name_idx);
            TETHER_LOGI(tag, "  PDO 0x%04X '%s' SM%u (%zu entries, %zu bits)",
                     pdo.pdo_index, name, pdo.sync_manager,
                     pdo.entries.size(), pdo.totalBits());
            
            for (const auto& entry : pdo.entries) {
                const char* entry_name = data.strings.getString(entry.name_idx);
                TETHER_LOGI(tag, "    0x%04X:%02X '%s' %u bits",
                         entry.index, entry.subindex, entry_name, entry.bit_length);
            }
        }
    }
}

void logSIIData(const SIIData& data, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║           SII EEPROM Data                                    ║\n╚══════════════════════════════════════════════════════════════╝\n");
    
    // Device info
    if (data.has_general) {
        TETHER_LOGI(tag, "Device Information:\n  Name:  %s\n  Group: %s\n  Order: %s\n", data.deviceName(), data.groupName(), data.orderCode());
    }
    
    // Identity
    logSIIIdentity(data.identity, tag);
    TETHER_LOGI(tag, "");
    
    // Mailbox
    logSIIMailbox(data.mailbox, tag);
    TETHER_LOGI(tag, "");
    
    // Sync Managers
    logSIISyncManagers(data, tag);
    TETHER_LOGI(tag, "");
    
    // FMMUs
    if (data.fmmu_count > 0) {
        TETHER_LOGI(tag, "SII FMMUs (%zu configured):", data.fmmu_count);
        for (size_t i = 0; i < data.fmmu_count; i++) {
            TETHER_LOGI(tag, "  FMMU%zu: %s", i, data.fmmus[i].getTypeName());
        }
        TETHER_LOGI(tag, "");
    }
    
    // PDOs
    logSIIPDOs(data, tag);
    
    // DC
    if (!data.dc_configs.empty()) {
        TETHER_LOGI(tag, "SII Distributed Clocks (%zu configured):", data.dc_configs.size());
        for (size_t i = 0; i < data.dc_configs.size(); i++) {
            const auto& dc = data.dc_configs[i];
            TETHER_LOGI(tag, "  DC%zu: cycle0=%uns shift0=%uns sync0=%s sync1=%s",
                     i, dc.cycle_time_0, dc.shift_time_0,
                     dc.sync0Enabled() ? "on" : "off", dc.sync1Enabled() ? "on" : "off");
        }
    }
}

void debugSIIMailboxDerivation(Master& master, uint16_t slave_index, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  SII Mailbox Derivation Debug (Slave %u)                      ║\n╚══════════════════════════════════════════════════════════════╝\n", (unsigned)slave_index);
    
    SIIReader reader(master);
    
    // Helper to log a word read with byte-level detail
    auto log_word_read = [&](uint16_t word_addr, const char* field_name, uint16_t& out_value) -> bool {
        if (!reader.readWord(slave_index, word_addr, out_value)) {
            TETHER_LOGE(tag, "  ❌ Failed to read word 0x%04X (%s)", (unsigned)word_addr, field_name);
            return false;
        }
        
        // Log word read
        TETHER_LOGI(tag, "  📖 Read word 0x%04X (%s): 0x%04X (%u)",
                 (unsigned)word_addr, field_name, (unsigned)out_value, (unsigned)out_value);
        
        // Log byte breakdown
        uint8_t lo_byte = out_value & 0xFF;
        uint8_t hi_byte = (out_value >> 8) & 0xFF;
        TETHER_LOGI(tag, "     Byte breakdown: [0x%02X, 0x%02X] (little-endian: lo=0x%02X hi=0x%02X)",
                 (unsigned)lo_byte, (unsigned)hi_byte, (unsigned)lo_byte, (unsigned)hi_byte);
        
        return true;
    };
    
    TETHER_LOGI(tag, "\n📋 STEP 1: Reading Bootstrap Mailbox Configuration (words 0x0014-0x0017)");
    
    uint16_t bootstrap_rx_offset = 0, bootstrap_rx_size = 0;
    uint16_t bootstrap_tx_offset = 0, bootstrap_tx_size = 0;
    
    log_word_read(SII_BOOTSTRAP_RX_MBX_OFFSET, "Bootstrap RX Offset", bootstrap_rx_offset);
    log_word_read(SII_BOOTSTRAP_RX_MBX_SIZE, "Bootstrap RX Size", bootstrap_rx_size);
    log_word_read(SII_BOOTSTRAP_TX_MBX_OFFSET, "Bootstrap TX Offset", bootstrap_tx_offset);
    log_word_read(SII_BOOTSTRAP_TX_MBX_SIZE, "Bootstrap TX Size", bootstrap_tx_size);
    
    TETHER_LOGI(tag, "\n📋 STEP 2: Reading Standard Mailbox Configuration (words 0x0018-0x001C)");
    
    uint16_t std_rx_offset = 0, std_rx_size = 0;
    uint16_t std_tx_offset = 0, std_tx_size = 0;
    uint16_t protocols = 0;
    
    log_word_read(SII_STD_RX_MBX_OFFSET, "Standard RX Offset", std_rx_offset);
    log_word_read(SII_STD_RX_MBX_SIZE, "Standard RX Size", std_rx_size);
    log_word_read(SII_STD_TX_MBX_OFFSET, "Standard TX Offset", std_tx_offset);
    log_word_read(SII_STD_TX_MBX_SIZE, "Standard TX Size", std_tx_size);
    log_word_read(SII_MAILBOX_PROTOCOLS, "Mailbox Protocols", protocols);
    
    TETHER_LOGI(tag, "\n📋 STEP 3: Field Assignments");
    TETHER_LOGI(tag, "  Bootstrap Mailbox:");
    TETHER_LOGI(tag, "    bootstrap_rx_offset  = 0x%04X (word 0x0014) → RX mailbox address (MbxIn, Master→Slave, SM0)",
             (unsigned)bootstrap_rx_offset);
    TETHER_LOGI(tag, "    bootstrap_rx_size    = 0x%04X (word 0x0015) → RX mailbox size in bytes",
             (unsigned)bootstrap_rx_size);
    TETHER_LOGI(tag, "    bootstrap_tx_offset  = 0x%04X (word 0x0016) → TX mailbox address (MbxOut, Slave→Master, SM1)",
             (unsigned)bootstrap_tx_offset);
    TETHER_LOGI(tag, "    bootstrap_tx_size    = 0x%04X (word 0x0017) → TX mailbox size in bytes",
             (unsigned)bootstrap_tx_size);
    
    TETHER_LOGI(tag, "  Standard Mailbox:");
    TETHER_LOGI(tag, "    std_rx_offset        = 0x%04X (word 0x0018) → RX mailbox address (MbxIn, Master→Slave, SM0)",
             (unsigned)std_rx_offset);
    TETHER_LOGI(tag, "    std_rx_size          = 0x%04X (word 0x0019) → RX mailbox size in bytes",
             (unsigned)std_rx_size);
    TETHER_LOGI(tag, "    std_tx_offset        = 0x%04X (word 0x001A) → TX mailbox address (MbxOut, Slave→Master, SM1)",
             (unsigned)std_tx_offset);
    TETHER_LOGI(tag, "    std_tx_size          = 0x%04X (word 0x001B) → TX mailbox size in bytes",
             (unsigned)std_tx_size);
    TETHER_LOGI(tag, "    protocols            = 0x%04X (word 0x001C) → Supported mailbox protocols",
             (unsigned)protocols);
    
    TETHER_LOGI(tag, "\n📋 STEP 4: Protocol Flag Decoding");
    TETHER_LOGI(tag, "  Protocol value: 0x%04X (%s)", (unsigned)protocols, getMailboxProtocolName(protocols));
    TETHER_LOGI(tag, "  Bit breakdown:");
    TETHER_LOGI(tag, "    Bit 0 (0x0001): AoE (ADS over EtherCAT)  = %s", (protocols & MBX_PROTO_AOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 1 (0x0002): EoE (Ethernet over EtherCAT) = %s", (protocols & MBX_PROTO_EOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 2 (0x0004): CoE (CANopen over EtherCAT) = %s", (protocols & MBX_PROTO_COE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 3 (0x0008): FoE (File over EtherCAT)   = %s", (protocols & MBX_PROTO_FOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 4 (0x0010): SoE (Servo over EtherCAT)  = %s", (protocols & MBX_PROTO_SOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 5 (0x0020): VoE (Vendor over EtherCAT) = %s", (protocols & MBX_PROTO_VOE) ? "✓ Supported" : "✗ Not supported");
    
    TETHER_LOGI(tag, "\n📋 STEP 5: Sync Manager Mapping (EtherCAT Convention)");
    TETHER_LOGI(tag, "  Per EtherCAT spec ETG.2010:");
    TETHER_LOGI(tag, "    SM0 = Receive mailbox (MbxIn) = Master→Slave direction");
    TETHER_LOGI(tag, "    SM1 = Send mailbox (MbxOut) = Slave→Master direction");
    TETHER_LOGI(tag, "  SII terminology mapping:");
    TETHER_LOGI(tag, "    std_rx  (SII word 0x0018/0x0019) → SM0 (Receive/MbxIn/Master→Slave)");
    TETHER_LOGI(tag, "    std_tx  (SII word 0x001A/0x001B) → SM1 (Send/MbxOut/Slave→Master)");
    
    TETHER_LOGI(tag, "\n📋 STEP 6: Final Mailbox Configuration");
    TETHER_LOGI(tag, "  SM0 (Receive/MbxIn/Master→Slave):");
    TETHER_LOGI(tag, "    Address = 0x%04X (from std_rx_offset at word 0x0018)", (unsigned)std_rx_offset);
    TETHER_LOGI(tag, "    Size    = %u bytes (from std_rx_size at word 0x0019)", (unsigned)std_rx_size);
    
    TETHER_LOGI(tag, "  SM1 (Send/MbxOut/Slave→Master):");
    TETHER_LOGI(tag, "    Address = 0x%04X (from std_tx_offset at word 0x001A)", (unsigned)std_tx_offset);
    TETHER_LOGI(tag, "    Size    = %u bytes (from std_tx_size at word 0x001B)", (unsigned)std_tx_size);
    
    TETHER_LOGI(tag, "  Supported Protocols: 0x%04X (%s)", (unsigned)protocols, getMailboxProtocolName(protocols));
    
    TETHER_LOGI(tag, "\n📋 STEP 7: Validation Checks");
    bool has_mailbox = (std_rx_size > 0 && std_tx_size > 0);
    TETHER_LOGI(tag, "  Has valid mailbox: %s", has_mailbox ? "✓ Yes" : "✗ No");
    
    if (has_mailbox) {
        if (std_rx_size == 0 || std_tx_size == 0) {
            TETHER_LOGW(tag, "  ⚠ WARNING: Zero-size mailbox detected");
        }
        if (std_rx_size < 32 || std_tx_size < 32) {
            TETHER_LOGW(tag, "  ⚠ WARNING: Unusually small mailbox (< 32 bytes)");
        }
        if (std_rx_offset == std_tx_offset) {
            TETHER_LOGW(tag, "  ⚠ WARNING: RX and TX mailboxes have same address");
        }
        if (std_rx_offset >= std_tx_offset) {
            TETHER_LOGW(tag, "  ⚠ WARNING: Non-standard address ordering (SM1 >= SM0)");
        }
    }
    
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  End of SII Mailbox Derivation Debug                          ║\n╚══════════════════════════════════════════════════════════════╝\n");
}

void logSIISummary(const SIIData& data, uint16_t slave_index, const char* tag) {
    TETHER_LOGI(tag, "Slave %u: %s (Vendor=0x%08" PRIx32 " Product=0x%08" PRIx32 ") "
             "SM:%zu RxPDO:%zu/%zuB TxPDO:%zu/%zuB %s",
             slave_index,
             data.deviceName(),
             data.identity.vendor_id,
             data.identity.product_code,
             data.sm_count,
             data.rx_pdos.size(), data.totalRxPDOBytes(),
             data.tx_pdos.size(), data.totalTxPDOBytes(),
             getMailboxProtocolName(data.mailbox.protocols));
}

} // namespace SII
} // namespace EtherCAT
