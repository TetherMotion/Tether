/**
 * @file SIIReader.cpp
 * @brief EtherCAT SII EEPROM Reader Implementation
 */

#include "sii/SIIReader.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/platform/Platform.hpp"
#include "ethercat/raw/internal.hpp"

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

SIIReader::SIIReader(EtherCATMaster& master)
    : m_master(master)
{
}

uint16_t SIIReader::adpForSlave(uint16_t slave_index) {
    // ADP for auto-increment addressing
    // Slave 0 = 0x0000, Slave 1 = 0xFFFF (-1), etc.
    return (slave_index == 0) ? 0x0000 : static_cast<uint16_t>(0 - slave_index);
}

bool SIIReader::waitNotBusy(uint16_t adp, uint16_t* out_status) {
    const int64_t deadline_us = Tether::Platform::Clock::instance().getMicroseconds() + static_cast<int64_t>(m_timeout_ms) * 1000LL;

    while (true) {
        if (Tether::Platform::Clock::instance().getMicroseconds() >= deadline_us) {
            return false;
        }

        uint16_t estat_le = 0;
        if (m_master.readRegister(adp, EC_REG_EEPSTAT, estat_le, 100)) {
            uint16_t estat = Raw::le16_to_host(estat_le);
            if (out_status) {
                *out_status = estat;
            }
            if ((estat & EC_ESTAT_BUSY) == 0) {
                return true;
            }
        }
        Tether::Platform::Clock::instance().delayMicroseconds(200);
    }
}

bool SIIReader::readRaw32(uint16_t adp, uint16_t word_address, uint32_t* out) {
    if (out) *out = 0;
    
    uint16_t estat = 0;
    if (!waitNotBusy(adp, &estat)) {
        return false;
    }
    
    // Clear errors if present
    if ((estat & EC_ESTAT_EMASK) != 0) {
        uint16_t nop_le = Raw::host_to_le16(EC_ECMD_NOP);
        m_master.writeRegister(adp, EC_REG_EEPCTL, nop_le, 200);
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
        
        if (!m_master.writeRegister(adp, EC_REG_EEPCTL, cmd, 200)) {
            return false;
        }
        
        Tether::Platform::Clock::instance().delayMicroseconds(200);
        
        estat = 0;
        if (!waitNotBusy(adp, &estat)) {
            return false;
        }
        
        if ((estat & EC_ESTAT_NACK) != 0) {
            nack_count++;
            Tether::Platform::Clock::instance().delayMicroseconds(1000);
            continue;
        }
        
        uint32_t edat_le = 0;
        if (!m_master.readRegister(adp, EC_REG_EEPDAT, edat_le, 200)) {
            return false;
        }
        
        if (out) {
            *out = Raw::le32_to_host(edat_le);
        }
        return true;
        
    } while (nack_count > 0 && nack_count < 3);
    
    return false;
}

bool SIIReader::readWord(uint16_t slave_index, uint16_t word_address, uint16_t& out) {
    uint32_t dword = 0;
    uint16_t adp = adpForSlave(slave_index);
    uint16_t aligned_addr = word_address & ~1u;  // Align to even address
    
    if (!readRaw32(adp, aligned_addr, &dword)) {
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
    uint16_t adp = adpForSlave(slave_index);
    return readRaw32(adp, word_address, &out);
}

size_t SIIReader::readWords(uint16_t slave_index, uint16_t word_address,
                            uint16_t* buffer, size_t word_count) {
    size_t words_read = 0;
    uint16_t adp = adpForSlave(slave_index);
    
    // Read 2 words at a time (32-bit reads)
    for (size_t i = 0; i < word_count; i += 2) {
        uint32_t dword = 0;
        if (!readRaw32(adp, static_cast<uint16_t>(word_address + i), &dword)) {
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
    uint16_t adp = adpForSlave(slave_index);
    size_t bytes_read = 0;
    
    while (bytes_read < byte_count) {
        uint16_t word_addr = static_cast<uint16_t>((byte_address + bytes_read) >> 1);
        uint16_t aligned_word = word_addr & ~1u;
        
        uint32_t dword = 0;
        if (!readRaw32(adp, aligned_word, &dword)) {
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
    uint16_t adp = adpForSlave(slave_index);
    uint16_t byte_addr = SII_CATEGORY_START * 2;  // Convert word address to byte address
    
    // Scan for string category
    while (true) {
        uint16_t cat_type = 0;
        uint16_t cat_size = 0;
        
        // Read category header (2 words = 4 bytes)
        uint32_t header = 0;
        if (!readRaw32(adp, byte_addr / 2, &header)) {
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
    if (m_reader.readWords(slave_index, 0, config, 8) != 8) {
        setError("Failed to read SII config area");
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
        setError("SII config CRC mismatch: expected=0x%02X calculated=0x%02X",
                 static_cast<unsigned>(stored_crc), static_cast<unsigned>(calc_crc));
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
        setError("Failed to read vendor ID");
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_PRODUCT_CODE, product_code)) {
        setError("Failed to read product code");
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_REVISION, revision)) {
        setError("Failed to read revision");
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_SERIAL_NUMBER, serial)) {
        setError("Failed to read serial number");
        return false;
    }
    
    out_data.identity.vendor_id = vendor_id;
    out_data.identity.product_code = product_code;
    out_data.identity.revision_number = revision;
    out_data.identity.serial_number = serial;
    
    // Read mailbox configuration
    uint16_t mbx_data[10];
    if (m_reader.readWords(slave_index, SII_BOOTSTRAP_RX_MBX_OFFSET, mbx_data, 10) != 10) {
        setError("Failed to read mailbox config");
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
        data.sync_managers[i].control_register = sm_data[4];
        data.sync_managers[i].status_register = sm_data[5];
        data.sync_managers[i].enable = sm_data[6];
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

bool readSII(EtherCATMaster& master, uint16_t slave_index, SIIData& out_data) {
    SIIReader reader(master);
    SIIParser parser(reader);
    return parser.parse(slave_index, out_data);
}

bool readSIIIdentity(EtherCATMaster& master, uint16_t slave_index, SIIIdentity& out_identity) {
    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;
    
    if (parser.parseIdentity(slave_index, data)) {
        out_identity = data.identity;
        return true;
    }
    return false;
}

bool readSIIMailbox(EtherCATMaster& master, uint16_t slave_index, SIIMailboxConfig& out_mailbox) {
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
        TETHER_LOGI(tag, "\nSII DC Configurations:");
        for (size_t i = 0; i < data.dc_configs.size(); i++) {
            const auto& dc = data.dc_configs[i];
            TETHER_LOGI(tag, "  DC%zu: CycleTime0=%uns AssignActivate=0x%04X\n    SYNC0=%s SYNC1=%s",
                     i, dc.cycle_time_0, dc.assign_activate,
                     dc.sync0Enabled() ? "Enabled" : "Disabled",
                     dc.sync1Enabled() ? "Enabled" : "Disabled");
        }
    }
    
    // CoE details
    if (data.has_general) {
        TETHER_LOGI(tag, "\nCoE Details: 0x%02X", data.general.coe_details);
        if (data.general.coeEnableSdo()) TETHER_LOGI(tag, "  ✓ SDO");
        if (data.general.coeEnableSdoInfo()) TETHER_LOGI(tag, "  ✓ SDO Info");
        if (data.general.coeEnablePdoAssign()) TETHER_LOGI(tag, "  ✓ PDO Assignment");
        if (data.general.coeEnablePdoConfig()) TETHER_LOGI(tag, "  ✓ PDO Configuration");
        if (data.general.coeEnableUploadStartup()) TETHER_LOGI(tag, "  ✓ Upload at Startup");
        if (data.general.coeEnableSdoComplete()) TETHER_LOGI(tag, "  ✓ Complete Access");
        
        TETHER_LOGI(tag, "\nGeneral Flags: 0x%02X\n  LRW Support: %s", data.general.flags, 
                 data.general.enableNotLRW() ? "NOT SUPPORTED (separate LRD/LWR only)" :
                 data.general.enableLRW() ? "Explicitly Supported" : "Default (assumed supported)");
        if (data.general.enableSafeOp()) TETHER_LOGI(tag, "  ✓ SafeOp (no outputs in Safe-Op)");
    }
    
    // Strings
    if (data.strings.count() > 0) {
        TETHER_LOGI(tag, "\nSII Strings (%zu):", data.strings.count());
        for (size_t i = 1; i <= data.strings.count(); i++) {
            TETHER_LOGI(tag, "  [%zu] \"%s\"", i, data.strings.getString(static_cast<uint8_t>(i)));
        }
    }
    
    TETHER_LOGI(tag, "\n═══════════════════════════════════════════════════════════════");
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
