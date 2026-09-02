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
static constexpr uint16_t EC_REG_EEPCFG   = 0x0500;  // EEPROM configuration
static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;  // EEPROM control/status
static constexpr uint16_t EC_REG_EEPSTAT  = 0x0502;  // Same as EEPCTL
static constexpr uint16_t EC_REG_EEPADDR  = 0x0504;  // EEPROM address
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;  // EEPROM data

// EEPROM configuration bits (0x0500)
// Bit 0 (read-only): 1 = PDI has EEPROM control, 0 = ECAT has control
// Bit 1 (write):     1 = force PDI to release EEPROM to ECAT
static constexpr uint8_t  EC_EEPCFG_FORCE_RELEASE = 0x02;
static constexpr uint8_t  EC_EEPCFG_ECAT_MASTER   = 0x00;

// EEPROM commands
static constexpr uint16_t EC_ECMD_NOP  = 0x0000;
static constexpr uint16_t EC_ECMD_READ = 0x0100;

// EEPROM status flags (per ETG.1000.4 EEPSTAT register at 0x0502)
// Bit 15: EEBP     - EEPROM busy (ECAT-side command in progress)
// Bit 14: LOADEE   - EEPROM loading error
// Bit 13: NACK     - EEPROM not acknowledged
// Bit 12: WRERR    - EEPROM write error
// Bit 11: CRCERR   - EEPROM CRC error
// Bit  7: EEPSIZE  - EEPROM size indicator (always 1 if EEPROM present)
// Bits 6-0: EEADR  - Current EEPROM address
static constexpr uint16_t EC_ESTAT_BUSY      = 0x8000;  // Bit 15: EEPROM busy
static constexpr uint16_t EC_ESTAT_EMASK     = 0x7800;  // Bits 14-11: error flags
static constexpr uint16_t EC_ESTAT_CRC_ERR   = 0x0800;  // Bit 11: CRC error
static constexpr uint16_t EC_ESTAT_LOAD_ERR  = 0x1000;  // Bit 12: loading error
static constexpr uint16_t EC_ESTAT_NACK      = 0x2000;  // Bit 13: not acknowledged
static constexpr uint16_t EC_ESTAT_WRITE_ERR = 0x4000;  // Bit 14: write error

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

bool SIIReader::getConfiguredStationAddr(uint16_t slave_index, uint16_t& addr,
                                         bool force_assign) {
    // Check cache first (skip cache if force_assign is true)
    if (!force_assign) {
        auto it = configured_addr_cache_.find(slave_index);
        if (it != configured_addr_cache_.end()) {
            addr = it->second;
            return true;
        }
    }

    const auto ap_addr = Master::slaveAddressFromADP(adpForSlave(slave_index));

    // Read configured station address from ESC register 0x0010 via APRD.
    // APRD works even on ESCs that reject APWR to EEPCTL.
    uint16_t addr_le = 0;
    if (!m_master.readRegister(ap_addr, 0x0010, addr_le, 200)) {
        return false;
    }
    addr = Raw::le16_to_host(addr_le);

    const bool dbg = m_master.debugFlags().eeprom &&
                     m_master.debugFlags().eepromFilt.allows(slave_index);

    // If the configured station address is 0x0000 (unassigned) or force_assign
    // is set, write a unique address so FPWR targets only this slave.
    // APWR to register 0x0010 works on ESCs that reject APWR to EEPCTL
    // (the rejection is specific to the EEPROM control register, not APWR).
    if (addr == 0x0000 || force_assign) {
        uint16_t new_addr = static_cast<uint16_t>(0x0001u + slave_index);
        uint16_t new_addr_le = Raw::host_to_le16(new_addr);
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: writing configured station addr 0x{:04X} "
                        "to reg 0x0010 (force={}, old=0x{:04X})",
                        slave_index, new_addr, force_assign ? 1 : 0, addr);
        }
        if (!m_master.writeRegister(ap_addr, 0x0010,
                                    &new_addr_le, sizeof(new_addr_le), 200)) {
            return false;
        }
        addr = new_addr;
    }

    configured_addr_cache_[slave_index] = addr;
    return true;
}

bool SIIReader::writeEepromReg(uint16_t slave_index, uint16_t reg_addr,
                               const void* data, uint16_t len) {
    const auto ap_addr = Master::slaveAddressFromADP(adpForSlave(slave_index));

    const bool dbg = m_master.debugFlags().eeprom &&
                     m_master.debugFlags().eepromFilt.allows(slave_index);

    // If we already know APWR fails for this slave, use FPWR directly.
    if (use_fpwr_slaves_.count(slave_index)) {
        uint16_t cfg_addr = 0;
        if (!getConfiguredStationAddr(slave_index, cfg_addr)) {
            return false;
        }
        if (m_master.writeRegister(EtherCAT::LogicalAddress(cfg_addr),
                                   reg_addr, data, len, 200)) {
            return true;
        }
        // FPWR with cached address failed — try force-assigning a fresh address.
        if (dbg) {
            TETHER_LOGW(TAG, "EEPROM [slave {}]: FPWR to 0x{:04X} with addr 0x{:04X} failed, "
                        "force-assigning fresh address", slave_index, reg_addr, cfg_addr);
        }
        if (!getConfiguredStationAddr(slave_index, cfg_addr, /*force_assign=*/true)) {
            return false;
        }
        return m_master.writeRegister(EtherCAT::LogicalAddress(cfg_addr),
                                      reg_addr, data, len, 200);
    }

    // Try APWR first (standard path, works on most ESCs).
    if (m_master.writeRegister(ap_addr, reg_addr, data, len, 200)) {
        return true;
    }

    // APWR failed (likely WKC=0) — fall back to FPWR using the
    // configured station address.
    if (dbg) {
        TETHER_LOGI(TAG, "EEPROM [slave {}]: APWR to 0x{:04X} failed, trying FPWR fallback",
                    slave_index, reg_addr);
    }
    uint16_t cfg_addr = 0;
    if (!getConfiguredStationAddr(slave_index, cfg_addr)) {
        return false;
    }
    if (m_master.writeRegister(EtherCAT::LogicalAddress(cfg_addr),
                               reg_addr, data, len, 200)) {
        use_fpwr_slaves_.insert(slave_index);
        return true;
    }

    // FPWR with the existing configured address also failed.
    // Force-assign a fresh unique address via APWR to 0x0010, then retry FPWR.
    if (dbg) {
        TETHER_LOGW(TAG, "EEPROM [slave {}]: FPWR to 0x{:04X} with addr 0x{:04X} failed, "
                    "force-assigning fresh address", slave_index, reg_addr, cfg_addr);
    }
    if (!getConfiguredStationAddr(slave_index, cfg_addr, /*force_assign=*/true)) {
        return false;
    }
    use_fpwr_slaves_.insert(slave_index);
    return m_master.writeRegister(EtherCAT::LogicalAddress(cfg_addr),
                                  reg_addr, data, len, 200);
}

bool SIIReader::writeEEPCTL(uint16_t slave_index, uint16_t eepctl_val) {
    const uint16_t eepctl_le = Raw::host_to_le16(eepctl_val);
    return writeEepromReg(slave_index, EC_REG_EEPCTL, &eepctl_le, sizeof(eepctl_le));
}

bool SIIReader::forceEepromToEcat(uint16_t slave_index) {
    if (eeprom_forced_to_ecat_.count(slave_index)) {
        return true;  // Already forced
    }

    const bool dbg = m_master.debugFlags().eeprom &&
                     m_master.debugFlags().eepromFilt.allows(slave_index);

    // Read current EEPConfig to check if PDI has control
    uint8_t eep_cfg = 0;
    const auto ap_addr = Master::slaveAddressFromADP(adpForSlave(slave_index));
    if (m_master.readRegister(ap_addr, EC_REG_EEPCFG, eep_cfg, 200)) {
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: EEPConfig=0x{:02X} (bit0={}, PDI={})",
                        slave_index, eep_cfg, eep_cfg & 1,
                        (eep_cfg & 1) ? "control" : "no control");
        }
        if ((eep_cfg & 0x01) == 0) {
            // ECAT already has control — no need to force
            eeprom_forced_to_ecat_.insert(slave_index);
            return true;
        }
    }

    // Force PDI to release EEPROM to ECAT (write 0x02 to EEPConfig)
    if (dbg) {
        TETHER_LOGI(TAG, "EEPROM [slave {}]: forcing EEPROM from PDI to ECAT control",
                    slave_index);
    }
    uint8_t force_val = EC_EEPCFG_FORCE_RELEASE;
    if (!writeEepromReg(slave_index, EC_REG_EEPCFG, &force_val, sizeof(force_val))) {
        if (dbg) {
            TETHER_LOGW(TAG, "EEPROM [slave {}]: failed to write FORCE_RELEASE to EEPConfig",
                        slave_index);
        }
        return false;
    }

    // Small delay for control transfer
    Tether::Platform::Clock::instance().delayMicroseconds(10000);

    // Set ECAT as master (write 0x00 to EEPConfig)
    uint8_t master_val = EC_EEPCFG_ECAT_MASTER;
    if (!writeEepromReg(slave_index, EC_REG_EEPCFG, &master_val, sizeof(master_val))) {
        if (dbg) {
            TETHER_LOGW(TAG, "EEPROM [slave {}]: failed to write ECAT_MASTER to EEPConfig",
                        slave_index);
        }
        return false;
    }

    // Verify control transfer
    eep_cfg = 0xFF;
    if (m_master.readRegister(ap_addr, EC_REG_EEPCFG, eep_cfg, 200)) {
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: EEPConfig after force = 0x{:02X}",
                        slave_index, eep_cfg);
        }
        if ((eep_cfg & 0x01) != 0) {
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: PDI still has EEPROM control after force!",
                            slave_index);
            }
            // Don't return false — try anyway, some ESCs report stale values
        }
    }

    eeprom_forced_to_ecat_.insert(slave_index);
    return true;
}

bool SIIReader::waitNotBusy(uint16_t slave_index, uint16_t* out_status, uint32_t* out_poll_iters) {
    uint32_t iters = 0;
    const int64_t deadline_us = Tether::Platform::Clock::instance().getMicroseconds() + static_cast<int64_t>(m_timeout_ms) * 1000LL;

    const bool dbg = m_master.debugFlags().eeprom &&
                     m_master.debugFlags().eepromFilt.allows(slave_index);

    while (true) {
        if (Tether::Platform::Clock::instance().getMicroseconds() >= deadline_us) {
            if (out_poll_iters) *out_poll_iters = iters;
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: waitNotBusy TIMEOUT after {} iters",
                            slave_index, iters);
            }
            return false;
        }

        uint16_t estat_le = 0;
        bool rd_ok = m_master.readRegister(
            Master::slaveAddressFromADP(adpForSlave(slave_index)),
            EC_REG_EEPSTAT, estat_le, 100);
        if (iters == 0 && dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: waitNotBusy first readRegister(0x0502) ok={} estat=0x{:04X}",
                        slave_index, rd_ok, Raw::le16_to_host(estat_le));
        }
        if (rd_ok) {
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

    const bool dbg = m_master.debugFlags().eeprom &&
                     m_master.debugFlags().eepromFilt.allows(slave_index);

    // Check the master-level per-slave cache before touching the bus.
    uint16_t lo = 0, hi = 0;
    const uint16_t wa = static_cast<uint16_t>(word_address & 0xFFFEu);
    if (m_master.getSIICachedWord(slave_index, wa, lo) &&
        m_master.getSIICachedWord(slave_index, static_cast<uint16_t>(wa + 1), hi)) {
        if (out) *out = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
        if (dbg) {
            TETHER_LOGD(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} cache hit", slave_index, word_address);
        }
        return true;
    }

    if (dbg) {
        TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} cache MISS — reading from bus", slave_index, word_address);
    }

    // Ensure the EEPROM interface is under ECAT control (not PDI).
    // Some ESCs boot with PDI having EEPROM control; writes to EEPCTL
    // are silently ignored in that state.
    if (!forceEepromToEcat(slave_index)) {
        if (dbg) {
            TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} failed to force ECAT control",
                        slave_index, word_address);
        }
        return false;
    }

    uint16_t estat = 0;
    uint32_t pre_iters = 0;
    if (!waitNotBusy(slave_index, &estat, &pre_iters)) {
        if (dbg) {
            TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} pre-wait timed out ({} iters)",
                        slave_index, word_address, pre_iters);
        }
        return false;
    }

    if (dbg) {
        TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} pre-wait OK estat=0x{:04X} iters={}",
                    slave_index, word_address, estat, pre_iters);
    }

    // Clear errors if present
    if ((estat & EC_ESTAT_EMASK) != 0) {
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} clearing error flags estat=0x{:04X}",
                        slave_index, word_address, estat);
        }
        writeEEPCTL(slave_index, EC_ECMD_NOP);
    }

    int nack_count = 0;
    do {
        // Standard EEPROM read sequence (per ETG.1000.4 / SOEM reference):
        // 1. Write EEPROM word address to EEPADDR (0x0504)
        // 2. Write READ command to EEPCTL (0x0502)
        // 3. Wait for busy to clear
        // 4. Read data from EEPDAT (0x0508)
        //
        // The EEPROM address must be written to the separate EEPADDR
        // register (0x0504), NOT combined into the low byte of EEPCTL.
        // Some ESCs (e.g. ET1100) accept the address in EEPCTL's low byte,
        // but others (e.g. IP cores, ET1200) require the separate write.
        const uint16_t eepaddr_le = Raw::host_to_le16(word_address);
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} writing EEPADDR=0x{:04X}",
                        slave_index, word_address, word_address);
        }
        if (!writeEepromReg(slave_index, EC_REG_EEPADDR, &eepaddr_le, sizeof(eepaddr_le))) {
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} writeEepromReg(EEPADDR) FAILED",
                            slave_index, word_address);
            }
            return false;
        }

        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} writing EEPCTL=0x{:04X} (READ cmd)",
                        slave_index, word_address, EC_ECMD_READ);
        }
        if (!writeEEPCTL(slave_index, EC_ECMD_READ)) {
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} writeEEPCTL FAILED (nack={})",
                            slave_index, word_address, nack_count);
            }
            return false;
        }

        estat = 0;
        uint32_t post_iters = 0;
        if (!waitNotBusy(slave_index, &estat, &post_iters)) {
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} post-wait timed out ({} iters, nack={})",
                            slave_index, word_address, post_iters, nack_count);
            }
            return false;
        }

        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} post-wait OK estat=0x{:04X} iters={}",
                        slave_index, word_address, estat, post_iters);
        }

        if ((estat & EC_ESTAT_EMASK) != 0) {
            if ((estat & EC_ESTAT_NACK) != 0) {
                nack_count++;
                if (dbg) {
                    TETHER_LOGD(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} NACK retry {}/3 estat=0x{:04X}",
                                slave_index, word_address, nack_count, estat);
                }
                Tether::Platform::Clock::instance().delayMicroseconds(1000);
                continue;
            }
            // Non-NACK error (CRC, loading, write error): log and fail
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} error status=0x{:04X} (crc={} load={} write={})",
                            slave_index, word_address, estat,
                            (estat & EC_ESTAT_CRC_ERR) ? 1 : 0,
                            (estat & EC_ESTAT_LOAD_ERR) ? 1 : 0,
                            (estat & EC_ESTAT_WRITE_ERR) ? 1 : 0);
            }
            // Clear error by writing NOP
            writeEEPCTL(slave_index, EC_ECMD_NOP);
            return false;
        }

        uint32_t edat_le = 0;
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} reading EEPDAT", slave_index, word_address);
        }
        if (!m_master.readRegister(Master::slaveAddressFromADP(adpForSlave(slave_index)), EC_REG_EEPDAT, edat_le, 200)) {
            if (dbg) {
                TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} readRegister(EEPDAT) FAILED (nack={})",
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
        if (dbg) {
            TETHER_LOGI(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} SUCCESS edat=0x{:08X} pre={} post={} nack={}",
                        slave_index, word_address, Raw::le32_to_host(edat_le), pre_iters, post_iters, nack_count);
        }
        return true;

    } while (nack_count > 0 && nack_count < 3);

    if (dbg) {
        TETHER_LOGW(TAG, "EEPROM [slave {}]: readRaw32 addr=0x{:04X} failed after {} NACK retries",
                    slave_index, word_address, nack_count);
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
    TETHER_LOGE(TAG, "{}", m_last_error);
}

std::string SIIParser::logPrefix(uint16_t slave_index) const {
    return m_reader.master().slaveLogPrefix(slave_index);
}

bool SIIParser::parseConfigArea(uint16_t slave_index, SIIData& out_data) {
    // Read configuration area (words 0x0000-0x0007)
    uint16_t config[8];
    size_t words_read = m_reader.readWords(slave_index, 0, config, 8);
    if (words_read != 8) {
        setError("%s: Failed to read SII config area (addr=0x0000, requested=8, got=%zu)",
                 logPrefix(slave_index).c_str(), words_read);
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
        setError("%s: SII config CRC mismatch: expected=0x%02X calculated=0x%02X "
                 "(raw words: %04X %04X %04X %04X %04X %04X %04X %04X)",
                 logPrefix(slave_index).c_str(),
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
        setError("%s: Failed to read SII vendor ID (addr=0x%04X)", logPrefix(slave_index).c_str(), SII_VENDOR_ID);
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_PRODUCT_CODE, product_code)) {
        setError("%s: Failed to read SII product code (addr=0x%04X)", logPrefix(slave_index).c_str(), SII_PRODUCT_CODE);
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_REVISION, revision)) {
        setError("%s: Failed to read SII revision (addr=0x%04X)", logPrefix(slave_index).c_str(), SII_REVISION);
        return false;
    }
    if (!m_reader.readDWord(slave_index, SII_SERIAL_NUMBER, serial)) {
        setError("%s: Failed to read SII serial number (addr=0x%04X)", logPrefix(slave_index).c_str(), SII_SERIAL_NUMBER);
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
        setError("%s: Failed to read SII mailbox config (addr=0x%04X, requested=10, got=%zu)",
                 logPrefix(slave_index).c_str(), SII_BOOTSTRAP_RX_MBX_OFFSET, mbx_words);
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
    
    // Read EEPROM size info (word 0x002E)
    uint16_t size_info = 0;
    if (m_reader.readWord(slave_index, SII_SIZE_INFO, size_info)) {
        out_data.eeprom_size_kbits = size_info & 0xFF;
        if (out_data.eeprom_size_kbits > 0) {
            out_data.eeprom_size_words = static_cast<uint16_t>(
                static_cast<uint32_t>(out_data.eeprom_size_kbits) * 1024 / 16);
        }
    }
    
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
        // EEPROM size boundary check
        if (out_data.eeprom_size_words > 0 && word_addr >= out_data.eeprom_size_words) {
            TETHER_LOGW(TAG, "Category parse exceeded EEPROM size ({} words) at word 0x{:04X}",
                        out_data.eeprom_size_words, word_addr);
            break;
        }

        // Read category header (2 words)
        uint16_t cat_type = 0;
        uint16_t cat_size = 0;
        
        if (!m_reader.readWord(slave_index, word_addr, cat_type)) {
            setError("Failed to read category type at word 0x%04X", word_addr);
            break;
        }
        word_addr++;
        
        if (cat_type == CAT_END) {
            TETHER_LOGD(TAG, "End of categories at word 0x{:04X}", word_addr - 1);
            break;
        }

        if (!m_reader.readWord(slave_index, word_addr, cat_size)) {
            setError("Failed to read category size at word 0x%04X", word_addr);
            break;
        }
        word_addr++;

        // Uninitialized/blank EEPROM region: treat as implicit end
        if (cat_type == 0 && cat_size == 0) {
            TETHER_LOGW(TAG, "Zero category header at word 0x{:04X}, treating as end",
                        static_cast<uint16_t>(word_addr - 2));
            break;
        }
        
        // Category data starts here
        uint16_t data_byte_offset = word_addr * 2;
        uint16_t data_size_bytes = cat_size * 2;
        
        TETHER_LOGD(TAG, "Category {}: type={} size={} words at 0x{:04X}",
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
                TETHER_LOGD(TAG, "Unknown category type {}, skipping", cat_type);
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

} // namespace SII
} // namespace EtherCAT
