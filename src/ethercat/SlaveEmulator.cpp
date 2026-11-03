/**
 * @file SlaveEmulator.cpp
 * @brief Implementation of EtherCAT Slave Emulator
 */

#include "SlaveEmulator.hpp"
#include "tether/ethercat/DCClass.hpp"
#include "tether/platform/EspCompat.hpp"
#include "raw/RawConstants.hpp"
#include "raw/RawWireFormat.hpp"

#include <algorithm>
#include <bit>

#define LOG_TAG "EC_EMU"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) TETHER_LOGD(LOG_TAG, fmt, ##__VA_ARGS__)

namespace EtherCAT {
namespace Emulator {

namespace {
/**
 * Per-SM data buffer cap.  The SM length register is wire-writable and the
 * emulator resizes sm.buffer to match; real ESC DPRAM is only a few KB in
 * total, so bound the emulation buffer to stop a hostile master from
 * growing memory (up to 64KB per SM, repeated writes included).
 */
constexpr size_t kMaxSmBufferBytes = 8 * 1024;
}

// ============================================================================
// SlaveEmulator Implementation
// ============================================================================

SlaveEmulator::SlaveEmulator() {
    registers_.fill(0);
    al_status_.state = SlaveState::INIT;
    initObjectDictionary();
}

void SlaveEmulator::setSIIConfig(const SIIConfig& config) {
    sii_config_ = config;
    
    // Build SII EEPROM data
    sii_data_.clear();
    sii_data_.resize(256 * 2, 0);  // 256 words minimum
    
    // Word 0-7: PDI Control, etc. (typical values)
    sii_data_[0] = 0x00; sii_data_[1] = 0x00;  // PDI Control
    
    // Word 8: Vendor ID (low)
    sii_data_[16] = config.vendor_id & 0xFF;
    sii_data_[17] = (config.vendor_id >> 8) & 0xFF;
    
    // Word 9: Vendor ID (high) - typically 0
    sii_data_[18] = 0; sii_data_[19] = 0;
    
    // Word 10: Product Code (low)
    sii_data_[20] = config.product_code & 0xFF;
    sii_data_[21] = (config.product_code >> 8) & 0xFF;
    
    // Word 11: Product Code (high)
    sii_data_[22] = 0; sii_data_[23] = 0;
    
    // Word 12-13: Revision
    sii_data_[24] = config.revision & 0xFF;
    sii_data_[25] = (config.revision >> 8) & 0xFF;
    sii_data_[26] = 0; sii_data_[27] = 0;
    
    // Word 14-15: Serial
    sii_data_[28] = config.serial & 0xFF;
    sii_data_[29] = (config.serial >> 8) & 0xFF;
    sii_data_[30] = (config.serial >> 16) & 0xFF;
    sii_data_[31] = (config.serial >> 24) & 0xFF;
    
    // Configure sync managers from config
    for (size_t i = 0; i < config.sync_managers.size() && i < 8; i++) {
        sync_managers_[i].start_addr = config.sync_managers[i].start_addr;
        sync_managers_[i].length = config.sync_managers[i].length;
        sync_managers_[i].control = config.sync_managers[i].control;
        sync_managers_[i].enable = config.sync_managers[i].enable;
        sync_managers_[i].buffer.resize(
            std::min<size_t>(config.sync_managers[i].length, kMaxSmBufferBytes), 0);
    }
}

void SlaveEmulator::setPosition(uint16_t position) {
    position_ = position;
}

void SlaveEmulator::setConfiguredAddress(uint16_t addr) {
    configured_addr_ = addr;
    // Write to register 0x0010 (Configured Station Address)
    registers_[0x10] = addr & 0xFF;
    registers_[0x11] = (addr >> 8) & 0xFF;
}

void SlaveEmulator::requestState(SlaveState state) {
    if (canTransition(al_status_.state, state)) {
        doTransition(state);
    } else {
        LOGW("Invalid state transition: %s -> %s",
             slaveStateToString(al_status_.state), slaveStateToString(state));
        al_status_.error = true;
        al_status_code_ = 0x0011;  // Invalid state transition
    }
}

bool SlaveEmulator::canTransition(SlaveState from, SlaveState to) {
    // Check valid transitions per EtherCAT spec
    switch (from) {
        case SlaveState::INIT:
            return to == SlaveState::PRE_OP || to == SlaveState::BOOT;
        case SlaveState::PRE_OP:
            return to == SlaveState::INIT || to == SlaveState::SAFE_OP;
        case SlaveState::SAFE_OP:
            return to == SlaveState::INIT || to == SlaveState::PRE_OP || to == SlaveState::OP;
        case SlaveState::OP:
            return to == SlaveState::INIT || to == SlaveState::PRE_OP || to == SlaveState::SAFE_OP;
        case SlaveState::BOOT:
            return to == SlaveState::INIT;
        default:
            return false;
    }
}

void SlaveEmulator::doTransition(SlaveState new_state) {
    LOGI("Slave %u: %s -> %s", position_,
         slaveStateToString(al_status_.state), slaveStateToString(new_state));
    al_status_.state = new_state;
    al_status_.error = false;
    al_status_code_ = 0;
}

bool SlaveEmulator::processAPRD(uint16_t ado, uint8_t* data, uint16_t len) {
    return readRegister(ado, data, len);
}

bool SlaveEmulator::processAPWR(uint16_t ado, const uint8_t* data, uint16_t len) {
    return writeRegister(ado, data, len);
}

bool SlaveEmulator::processFPRD(uint16_t ado, uint8_t* data, uint16_t len) {
    return readRegister(ado, data, len);
}

bool SlaveEmulator::processFPWR(uint16_t ado, const uint8_t* data, uint16_t len) {
    return writeRegister(ado, data, len);
}

bool SlaveEmulator::readRegister(uint16_t addr, uint8_t* data, uint16_t len) {
    if (errors_.inject_timeout && (errors_.timeout_register == 0 || errors_.timeout_register == addr)) {
        return false;  // Simulate timeout
    }
    
    // Handle special registers
    switch (addr) {
        case 0x0130:  // AL Status
        {
            uint16_t val = al_status_.toRegister();
            if (len >= 2) {
                data[0] = val & 0xFF;
                data[1] = (val >> 8) & 0xFF;
            }
            return true;
        }
        
        case 0x0134:  // AL Status Code
            if (len >= 2) {
                data[0] = al_status_code_ & 0xFF;
                data[1] = (al_status_code_ >> 8) & 0xFF;
            }
            return true;
            
        case toUInt16(DCRegisters::DCSysTime):  // DC System Time
            if (len >= 8) {
                for (int i = 0; i < 8; i++) {
                    data[i] = (dc_state_.system_time >> (i * 8)) & 0xFF;
                }
            }
            return true;
            
        case toUInt16(DCRegisters::DCSysOffset):  // DC System Time Offset
            if (len >= 8) {
                for (int i = 0; i < 8; i++) {
                    data[i] = (dc_state_.system_time_offset >> (i * 8)) & 0xFF;
                }
            }
            return true;
            
        case toUInt16(DCRegisters::DCSysDiff):  // DC System Time Difference
            if (len >= 4) {
                data[0] = dc_state_.system_time_diff & 0xFF;
                data[1] = (dc_state_.system_time_diff >> 8) & 0xFF;
                data[2] = (dc_state_.system_time_diff >> 16) & 0xFF;
                data[3] = (dc_state_.system_time_diff >> 24) & 0xFF;
            }
            return true;
    }
    
    // Sync Manager registers (0x0800 - 0x087F): serve any subrange of the
    // 8-byte register block field-by-field — drivers commonly read/write
    // whole blocks (or several) in one datagram.
    if (addr >= 0x0800 && addr < 0x0880 && addr + len <= 0x0880) {
        for (uint16_t i = 0; i < len; ++i) {
            const uint16_t a = static_cast<uint16_t>(addr + i);
            const uint8_t sm_num = (a - 0x0800) / 8;
            const uint8_t off = (a - 0x0800) % 8;
            if (sm_num >= 8) { data[i] = 0; continue; }
            SyncManager& sm = sync_managers_[sm_num];
            switch (off) {
                case 0: data[i] = sm.start_addr & 0xFF; break;
                case 1: data[i] = sm.start_addr >> 8; break;
                case 2: data[i] = sm.length & 0xFF; break;
                case 3: data[i] = sm.length >> 8; break;
                case 4: data[i] = std::bit_cast<uint8_t>(sm.control); break;
                case 5: data[i] = std::bit_cast<uint8_t>(sm.status); break;
                case 6: data[i] = std::bit_cast<uint8_t>(sm.enable); break;
                default: data[i] = 0; break;
            }
        }
        return true;
    }
    
    // FMMU registers (0x0600 - 0x06FF)
    if (addr >= 0x0600 && addr < 0x0700) {
        uint8_t fmmu_num = (addr - 0x0600) / 16;
        uint8_t fmmu_offset = (addr - 0x0600) % 16;
        
        if (fmmu_num < 8) {
            FMMU& fmmu = fmmus_[fmmu_num];
            // Read FMMU configuration
            if (fmmu_offset == 0 && len >= 4) {
                data[0] = fmmu.logical_start & 0xFF;
                data[1] = (fmmu.logical_start >> 8) & 0xFF;
                data[2] = (fmmu.logical_start >> 16) & 0xFF;
                data[3] = (fmmu.logical_start >> 24) & 0xFF;
                return true;
            }
            if (fmmu_offset == 4 && len >= 2) {
                data[0] = fmmu.length & 0xFF;
                data[1] = (fmmu.length >> 8) & 0xFF;
                return true;
            }
        }
    }
    
    // SM-buffer data access by physical address (mailbox + PDO areas)
    if (smBufferRead(addr, data, len)) {
        return true;
    }

    // Generic register read
    if (addr + len <= registers_.size()) {
        std::memcpy(data, &registers_[addr], len);
        return true;
    }
    
    return false;
}

bool SlaveEmulator::writeRegister(uint16_t addr, const uint8_t* data, uint16_t len) {
    // Handle special registers
    switch (addr) {
        case 0x0120:  // AL Control (state request)
        {
            if (len >= 2) {
                uint16_t val = data[0] | (data[1] << 8);
                SlaveState requested = static_cast<SlaveState>(val & 0x0F);
                requestState(requested);
            }
            return true;
        }
        
        case 0x0010:  // Configured Station Address
            if (len >= 2) {
                configured_addr_ = data[0] | (data[1] << 8);
                registers_[0x10] = data[0];
                registers_[0x11] = data[1];
            }
            return true;
            
        case toUInt16(DCRegisters::DCSysOffset):  // DC System Time Offset
            if (len >= 8) {
                dc_state_.system_time_offset = 0;
                for (int i = 0; i < 8; i++) {
                    dc_state_.system_time_offset |= static_cast<int64_t>(data[i]) << (i * 8);
                }
            }
            return true;
            
        case toUInt16(DCRegisters::DCSyncAct):  // DC Activation
            dc_state_.dc_active = (data[0] & 0x01) != 0;
            dc_state_.sync0_enable = (data[0] & 0x02) != 0;
            dc_state_.sync1_enable = (data[0] & 0x04) != 0;
            return true;
            
        case toUInt16(DCRegisters::DCCycle0):  // DC SYNC0 Cycle Time
            if (len >= 4) {
                dc_state_.cycle_time_0 = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
            }
            return true;
    }
    
    // Sync Manager registers: per-byte field updates so multi-byte and
    // multi-block writes behave like a real ESC.
    if (addr >= 0x0800 && addr < 0x0880 && addr + len <= 0x0880) {
        for (uint16_t i = 0; i < len; ++i) {
            const uint16_t a = static_cast<uint16_t>(addr + i);
            const uint8_t sm_num = (a - 0x0800) / 8;
            const uint8_t off = (a - 0x0800) % 8;
            if (sm_num >= 8) continue;
            SyncManager& sm = sync_managers_[sm_num];
            switch (off) {
                case 0:
                    sm.start_addr = (sm.start_addr & 0xFF00) | data[i];
                    break;
                case 1:
                    sm.start_addr = (sm.start_addr & 0x00FF)
                                  | (static_cast<uint16_t>(data[i]) << 8);
                    break;
                case 2:
                    sm.length = (sm.length & 0xFF00) | data[i];
                    sm.buffer.resize(
                        std::min<size_t>(sm.length, kMaxSmBufferBytes), 0);
                    break;
                case 3:
                    sm.length = (sm.length & 0x00FF)
                              | (static_cast<uint16_t>(data[i]) << 8);
                    sm.buffer.resize(
                        std::min<size_t>(sm.length, kMaxSmBufferBytes), 0);
                    break;
                case 4:
                    sm.control = std::bit_cast<
                        EtherCAT::SyncManager::SMControlReg>(data[i]);
                    break;
                case 6:
                    sm.enable = std::bit_cast<
                        EtherCAT::SyncManager::SMActivateReg>(data[i]);
                    break;
                default:
                    break;  // status + reserved: read-only / ignored
            }
        }
        return true;
    }
    
    // FMMU registers
    if (addr >= 0x0600 && addr < 0x0700) {
        uint8_t fmmu_num = (addr - 0x0600) / 16;
        uint8_t fmmu_offset = (addr - 0x0600) % 16;
        
        if (fmmu_num < 8) {
            FMMU& fmmu = fmmus_[fmmu_num];
            if (fmmu_offset == 0 && len >= 16) {
                // Full FMMU write
                fmmu.logical_start = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                fmmu.length = data[4] | (data[5] << 8);
                fmmu.logical_start_bit = data[6];
                fmmu.logical_end_bit = data[7];
                fmmu.physical_start = data[8] | (data[9] << 8);
                fmmu.physical_start_bit = data[10];
                fmmu.read_enable = (data[11] & 0x01) != 0;
                fmmu.write_enable = (data[11] & 0x02) != 0;
                fmmu.enabled = (data[12] & 0x01) != 0;
                return true;
            }
        }
    }
    
    // SM-buffer data access by physical address (mailbox + PDO areas)
    if (smBufferWrite(addr, data, len)) {
        return true;
    }

    // Generic register write
    if (addr + len <= registers_.size()) {
        std::memcpy(&registers_[addr], data, len);
        return true;
    }
    
    return false;
}

bool SlaveEmulator::processLogicalRead(uint32_t logical_addr, uint8_t* data, uint16_t len) {
    // Each FMMU serves only the bytes inside its own logical window —
    // a multi-slave LRW image overlaps each slave's window partially.
    bool any = false;
    const uint64_t req_end = uint64_t(logical_addr) + len;
    for (auto& fmmu : fmmus_) {
        if (!fmmu.read_enable || !fmmu.enabled || fmmu.length == 0) continue;
        const uint64_t win_lo = std::max<uint64_t>(
            logical_addr, fmmu.logical_start);
        const uint64_t win_hi = std::min<uint64_t>(
            req_end, uint64_t(fmmu.logical_start) + fmmu.length);
        if (win_lo >= win_hi) continue;
        const uint16_t phys = static_cast<uint16_t>(
            fmmu.physical_start + (win_lo - fmmu.logical_start));
        const uint16_t wlen = static_cast<uint16_t>(win_hi - win_lo);
        for (auto& sm : sync_managers_) {
            if (sm.isEnabled() && phys >= sm.start_addr &&
                phys + wlen <= sm.start_addr + sm.length) {
                const uint16_t offset = phys - sm.start_addr;
                // buffer may be capped below sm.length (kMaxSmBufferBytes)
                if (static_cast<size_t>(offset) + wlen > sm.buffer.size())
                    break;
                std::memcpy(data + (win_lo - logical_addr),
                            &sm.buffer[offset], wlen);
                any = true;
                break;
            }
        }
    }
    return any;
}

bool SlaveEmulator::processLogicalWrite(uint32_t logical_addr, const uint8_t* data, uint16_t len) {
    bool any = false;
    const uint64_t req_end = uint64_t(logical_addr) + len;
    for (auto& fmmu : fmmus_) {
        if (!fmmu.write_enable || !fmmu.enabled || fmmu.length == 0) continue;
        const uint64_t win_lo = std::max<uint64_t>(
            logical_addr, fmmu.logical_start);
        const uint64_t win_hi = std::min<uint64_t>(
            req_end, uint64_t(fmmu.logical_start) + fmmu.length);
        if (win_lo >= win_hi) continue;
        const uint16_t phys = static_cast<uint16_t>(
            fmmu.physical_start + (win_lo - fmmu.logical_start));
        const uint16_t wlen = static_cast<uint16_t>(win_hi - win_lo);
        for (auto& sm : sync_managers_) {
            if (sm.isEnabled() && phys >= sm.start_addr &&
                phys + wlen <= sm.start_addr + sm.length) {
                const uint16_t offset = phys - sm.start_addr;
                if (static_cast<size_t>(offset) + wlen > sm.buffer.size())
                    break;
                std::memcpy(&sm.buffer[offset],
                            data + (win_lo - logical_addr), wlen);
                any = true;
                break;
            }
        }
    }
    return any;
}

bool SlaveEmulator::processSIIRead(uint32_t word_addr, uint16_t* data) {
    if (word_addr * 2 + 1 < sii_data_.size()) {
        *data = sii_data_[word_addr * 2] | (sii_data_[word_addr * 2 + 1] << 8);
        return true;
    }
    return false;
}

bool SlaveEmulator::processSIIWrite(uint32_t word_addr, uint16_t data) {
    if (word_addr * 2 + 1 < sii_data_.size()) {
        sii_data_[word_addr * 2] = data & 0xFF;
        sii_data_[word_addr * 2 + 1] = (data >> 8) & 0xFF;
        return true;
    }
    return false;
}

void SlaveEmulator::advanceDCTime(uint64_t delta_ns) {
    dc_state_.advanceTime(delta_ns);
    
    // Simulate drift if enabled
    if (errors_.inject_dc_drift) {
        int64_t drift = (delta_ns * errors_.dc_drift_ppb) / 1000000000LL;
        dc_state_.system_time += drift;
    }
}

void SlaveEmulator::enableCiA402(bool enable) {
    cia402_enabled_ = enable;
    if (enable) {
        // Add CiA 402 objects to dictionary
        // Control word 0x6040
        object_dictionary_.push_back({0x6040, 0, 0x0006, 16, 0x03, {0, 0}, "Controlword"});
        // Status word 0x6041
        object_dictionary_.push_back({0x6041, 0, 0x0006, 16, 0x01, {0, 0}, "Statusword"});
        // Modes of operation 0x6060
        object_dictionary_.push_back({0x6060, 0, 0x0002, 8, 0x03, {0}, "Modes of operation"});
        // Target position 0x607A
        object_dictionary_.push_back({0x607A, 0, 0x0004, 32, 0x03, {0, 0, 0, 0}, "Target position"});
        // Actual position 0x6064
        object_dictionary_.push_back({0x6064, 0, 0x0004, 32, 0x01, {0, 0, 0, 0}, "Actual position"});
        // Modes of operation display 0x6061
        object_dictionary_.push_back({0x6061, 0, 0x0002, 8, 0x01, {0}, "Modes of operation display"});
        // Actual velocity 0x606C
        object_dictionary_.push_back({0x606C, 0, 0x0004, 32, 0x01, {0, 0, 0, 0}, "Actual velocity"});
        // Error code 0x603F
        object_dictionary_.push_back({0x603F, 0, 0x0006, 16, 0x01, {0, 0}, "Error code"});
    }
}

void SlaveEmulator::simulate(uint32_t delta_us) {
    if (cia402_enabled_) {
        drive_state_.simulate(delta_us);
        
        // Update actual position in SM buffer (typical TxPDO mapping)
        // This is simplified - real implementation would use PDO mapping
    }
}

void SlaveEmulator::initObjectDictionary() {
    // Standard CoE objects
    object_dictionary_.push_back({0x1000, 0, 0x0007, 32, 0x01, {0, 0, 0, 0}, "Device Type"});
    object_dictionary_.push_back({0x1001, 0, 0x0005, 8, 0x01, {0}, "Error Register"});
    object_dictionary_.push_back({0x1018, 1, 0x0007, 32, 0x01, {0, 0, 0, 0}, "Vendor ID"});
    object_dictionary_.push_back({0x1018, 2, 0x0007, 32, 0x01, {0, 0, 0, 0}, "Product Code"});
}

ODEntry* SlaveEmulator::findODEntry(uint16_t index, uint8_t subindex) {
    for (auto& entry : object_dictionary_) {
        if (entry.index == index && entry.subindex == subindex) {
            return &entry;
        }
    }
    return nullptr;
}

// ============================================================================
// SM-buffer data path + CoE mailbox
// ============================================================================

bool SlaveEmulator::smBufferRead(uint16_t addr, uint8_t* data, uint16_t len) {
    for (size_t i = 0; i < sync_managers_.size(); ++i) {
        SyncManager& sm = sync_managers_[i];
        if (!sm.isEnabled() || sm.buffer.empty()) continue;
        if (addr >= sm.start_addr &&
            static_cast<size_t>(addr - sm.start_addr) + len <= sm.buffer.size()) {
            std::memcpy(data, &sm.buffer[addr - sm.start_addr], len);
            sm.status.read_event = 1;
            if (i == 1 && sii_config_.mailbox_protocol != 0) {
                // SM1 = slave -> master mailbox: reading drains the response.
                sm.status.mailbox_full = 0;
                sm.status.read_buf_full = 0;
            }
            return true;
        }
    }
    return false;
}

bool SlaveEmulator::smBufferWrite(uint16_t addr, const uint8_t* data, uint16_t len) {
    for (size_t i = 0; i < sync_managers_.size(); ++i) {
        SyncManager& sm = sync_managers_[i];
        if (!sm.isEnabled() || sm.buffer.empty()) continue;
        if (addr >= sm.start_addr &&
            static_cast<size_t>(addr - sm.start_addr) + len <= sm.buffer.size()) {
            std::memcpy(&sm.buffer[addr - sm.start_addr], data, len);
            sm.status.write_event = 1;
            if (i == 0 && sii_config_.mailbox_protocol != 0) {
                // SM0 = master -> slave mailbox: the PDI drains it
                // synchronously and posts the response into SM1.
                processMailboxRequest();
            } else if (cia402_enabled_ && i >= 2 &&
                       sm.buffer.size() >= 2) {
                // Process-data SM (RxPDO): bytes 0-1 carry the controlword
                // on CiA402 drives.
                const uint16_t cw = static_cast<uint16_t>(
                    sm.buffer[0] | (sm.buffer[1] << 8));
                drive_state_.processControlWord(cw);
            }
            return true;
        }
    }
    return false;
}

void SlaveEmulator::syncDriveFromOD(uint16_t index, uint8_t sub,
                                    const uint8_t* data, size_t len) {
    if (!cia402_enabled_ || sub != 0) return;
    auto u16 = [&] { return static_cast<uint16_t>(data[0] | (data[1] << 8)); };
    auto u32 = [&] {
        return static_cast<uint32_t>(data[0] | (data[1] << 8) |
                                   (data[2] << 16) | (data[3] << 24));
    };
    switch (index) {
        case 0x6040: if (len >= 2) drive_state_.processControlWord(u16()); break;
        case 0x6060: if (len >= 1) {
                drive_state_.target_mode = static_cast<int8_t>(data[0]);
                drive_state_.operation_mode = drive_state_.target_mode;
            }
            break;
        case 0x607A: if (len >= 4) drive_state_.target_position = static_cast<int32_t>(u32()); break;
        case 0x60FF: if (len >= 4) drive_state_.target_velocity = static_cast<int32_t>(u32()); break;
        case 0x6071: if (len >= 2) drive_state_.target_torque = static_cast<int16_t>(u16()); break;
        default: break;
    }
}

void SlaveEmulator::syncODFromDrive(ODEntry& entry) {
    if (!cia402_enabled_ || entry.subindex != 0) return;
    auto put16 = [&](uint16_t v) {
        entry.data.assign({static_cast<uint8_t>(v & 0xFF),
                           static_cast<uint8_t>((v >> 8) & 0xFF)});
    };
    auto put32 = [&](uint32_t v) {
        entry.data.assign({static_cast<uint8_t>(v & 0xFF),
                           static_cast<uint8_t>((v >> 8) & 0xFF),
                           static_cast<uint8_t>((v >> 16) & 0xFF),
                           static_cast<uint8_t>((v >> 24) & 0xFF)});
    };
    switch (entry.index) {
        case 0x603F: put16(drive_state_.error_code); break;
        case 0x6041: put16(drive_state_.getStatusWord()); break;
        case 0x6061: entry.data = {static_cast<uint8_t>(drive_state_.operation_mode)}; break;
        case 0x6064: put32(static_cast<uint32_t>(drive_state_.actual_position)); break;
        case 0x606C: put32(static_cast<uint32_t>(drive_state_.actual_velocity)); break;
        default: break;
    }
}

namespace {
constexpr uint32_t kSdoAbortNoObject      = 0x06020000;
constexpr uint32_t kSdoAbortNotReadable   = 0x06010001;
constexpr uint32_t kSdoAbortNotWritable   = 0x06010002;
constexpr uint32_t kSdoAbortGeneral       = 0x08000000;
}

void SlaveEmulator::coeSdoAbort(uint8_t index_hi, uint8_t index_lo,
                                uint8_t sub, uint32_t abort_code) {
    SyncManager& sm1 = sync_managers_[1];
    Raw::MbxHeader mbx{};
    mbx.length_le = Raw::host_to_le16(sizeof(Raw::CoeHeader) + 8);
    mbx.address_le = 0;
    mbx.priority = 0;
    mbx.mbxtype = Raw::mbx_type_with_cnt(Raw::EC_MBXT_COE, 0);
    Raw::CoeHeader coe{};
    coe.raw_le = Raw::host_to_le16(Raw::coe_make_raw(0, Raw::EC_COES_SDORES));
    const size_t total = sizeof(mbx) + sizeof(coe) + 8;
    if (total > sm1.buffer.size()) return;
    uint8_t* p = sm1.buffer.data();
    std::memcpy(p, &mbx, sizeof(mbx));
    p += sizeof(mbx);
    std::memcpy(p, &coe, sizeof(coe));
    p += sizeof(coe);
    p[0] = Raw::EC_SDO_ABORT;
    p[1] = index_lo;
    p[2] = index_hi;
    p[3] = sub;
    const uint32_t le_code = Raw::host_to_le32(abort_code);
    std::memcpy(p + 4, &le_code, 4);
    sm1.status.mailbox_full = 1;
    sm1.status.read_buf_full = 1;
}

void SlaveEmulator::processMailboxRequest() {
    SyncManager& sm0 = sync_managers_[0];
    SyncManager& sm1 = sync_managers_[1];
    if (sm0.buffer.size() < sizeof(Raw::MbxHeader) + sizeof(Raw::CoeHeader) + 8 ||
        sm1.buffer.empty()) {
        return;
    }

    const uint8_t* req = sm0.buffer.data();
    Raw::MbxHeader mbx{};
    std::memcpy(&mbx, req, sizeof(mbx));
    const uint8_t mbx_type = mbx.mbxtype & 0x0F;
    const uint8_t mbx_cnt = (mbx.mbxtype >> 4) & 0x07;

    // Only CoE SDO requests are emulated.
    if (mbx_type != Raw::EC_MBXT_COE) return;
    Raw::CoeHeader coe{};
    std::memcpy(&coe, req + sizeof(mbx), sizeof(coe));
    const uint8_t service = static_cast<uint8_t>(
        (Raw::le16_to_host(coe.raw_le) >> 12) & 0x0F);
    if (service != Raw::EC_COES_SDOREQ) return;

    const uint8_t* sdo = req + sizeof(mbx) + sizeof(coe);
    const size_t sdo_avail =
        sm0.buffer.size() - sizeof(Raw::MbxHeader) - sizeof(Raw::CoeHeader);
    const uint8_t cmd = sdo[0];
    const uint16_t index = static_cast<uint16_t>(sdo[1] | (sdo[2] << 8));
    const uint8_t sub = sdo[3];
    const uint8_t index_lo = sdo[1];
    const uint8_t index_hi = sdo[2];

    // Response scaffolding: same mailbox counter as the request.
    auto begin_response = [&](size_t sdo_rsp_len) -> uint8_t* {
        const size_t total = sizeof(Raw::MbxHeader) + sizeof(Raw::CoeHeader) + sdo_rsp_len;
        if (total > sm1.buffer.size()) return nullptr;
        Raw::MbxHeader r{};
        r.length_le = Raw::host_to_le16(
            static_cast<uint16_t>(sizeof(Raw::CoeHeader) + sdo_rsp_len));
        r.address_le = mbx.address_le;
        r.priority = mbx.priority;
        r.mbxtype = Raw::mbx_type_with_cnt(Raw::EC_MBXT_COE, mbx_cnt);
        Raw::CoeHeader c{};
        c.raw_le = Raw::host_to_le16(Raw::coe_make_raw(0, Raw::EC_COES_SDORES));
        uint8_t* p = sm1.buffer.data();
        std::memset(p, 0, total);
        std::memcpy(p, &r, sizeof(r));
        std::memcpy(p + sizeof(r), &c, sizeof(c));
        return p + sizeof(r) + sizeof(c);
    };
    auto finish_response = [&] {
        sm1.status.mailbox_full = 1;
        sm1.status.read_buf_full = 1;
    };

    const uint8_t req_class = cmd & 0xE0u;

    if (req_class == Raw::EC_SDO_DOWN_REQ) {
        // Initiate download: expedited when bit1 set; size-indicated bit0.
        const bool expedited = (cmd & 0x02u) != 0;
        const bool size_indicated = (cmd & 0x01u) != 0;
        const uint8_t unused = (cmd >> 2) & 0x03;
        size_t data_len = 0;
        const uint8_t* payload = sdo + 4;
        if (expedited) {
            data_len = size_indicated ? (4u - unused) : 4u;
        } else {
            // Normal initiate: first 4 bytes declare the total size.
            uint32_t declared = 0;
            std::memcpy(&declared, sdo + 4, 4);
            declared = Raw::le32_to_host(declared);
            payload = sdo + 8;
            data_len = (sdo_avail >= 8)
                     ? std::min<size_t>(declared, sdo_avail - 8) : 0;
        }

        ODEntry* entry = findODEntry(index, sub);
        if (!entry) {
            coeSdoAbort(index_hi, index_lo, sub, kSdoAbortNoObject);
            return;
        }
        if (!entry->isWritable()) {
            coeSdoAbort(index_hi, index_lo, sub, kSdoAbortNotWritable);
            return;
        }
        const size_t avail = expedited
            ? std::min<size_t>(data_len, sdo_avail - 4)
            : data_len;
        if (entry->bit_length && avail != (entry->bit_length + 7) / 8) {
            // Tolerate size mismatches: real drives store what fits.
            entry->data.assign(payload, payload + avail);
        } else {
            entry->data.assign(payload, payload + avail);
        }
        syncDriveFromOD(index, sub, entry->data.data(), entry->data.size());

        uint8_t* rsp = begin_response(8);
        if (!rsp) return;
        rsp[0] = 0x60;  // Download response
        rsp[1] = index_lo;
        rsp[2] = index_hi;
        rsp[3] = sub;
        finish_response();
        return;
    }

    if (req_class == Raw::EC_SDO_UP_REQ) {
        ODEntry* entry = findODEntry(index, sub);
        if (!entry) {
            coeSdoAbort(index_hi, index_lo, sub, kSdoAbortNoObject);
            return;
        }
        if (!entry->isReadable()) {
            coeSdoAbort(index_hi, index_lo, sub, kSdoAbortNotReadable);
            return;
        }
        syncODFromDrive(*entry);

        const size_t n = entry->data.size();
        if (n <= 4) {
            uint8_t* rsp = begin_response(8);
            if (!rsp) return;
            const uint8_t unused = static_cast<uint8_t>(4 - n);
            rsp[0] = static_cast<uint8_t>(0x40 | 0x02 | 0x01 | (unused << 2));
            rsp[1] = index_lo;
            rsp[2] = index_hi;
            rsp[3] = sub;
            std::memcpy(rsp + 4, entry->data.data(), n);
        } else {
            // Normal upload: report size; segments served below.
            uint8_t* rsp = begin_response(8);
            if (!rsp) return;
            rsp[0] = 0x41;
            rsp[1] = index_lo;
            rsp[2] = index_hi;
            rsp[3] = sub;
            const uint32_t le_n = Raw::host_to_le32(static_cast<uint32_t>(n));
            std::memcpy(rsp + 4, &le_n, 4);
            upload_index_ = index;
            upload_sub_ = sub;
            upload_offset_ = 0;
            upload_toggle_ = false;
        }
        finish_response();
        return;
    }

    if (req_class == Raw::EC_SDO_SEG_UP_REQ) {
        // Segmented upload: serve chunks of the OD entry declared earlier.
        ODEntry* entry = findODEntry(upload_index_, upload_sub_);
        const bool req_toggle = (cmd & 0x10u) != 0;
        if (!entry || req_toggle != upload_toggle_) {
            coeSdoAbort(index_hi, index_lo, sub, kSdoAbortGeneral);
            return;
        }
        const size_t max_seg = sm1.buffer.size()
            - sizeof(Raw::MbxHeader) - sizeof(Raw::CoeHeader) - 1;
        const size_t remaining = entry->data.size() - upload_offset_;
        const size_t seg = std::min(remaining, max_seg);
        const bool last = (upload_offset_ + seg >= entry->data.size());

        uint8_t* rsp = begin_response(1 + seg);
        if (!rsp) return;
        rsp[0] = static_cast<uint8_t>((upload_toggle_ ? 0x10 : 0x00) |
                                      (((7 - seg) & 0x07) << 1) |
                                      (last ? 0x01 : 0x00));
        std::memcpy(rsp + 1, entry->data.data() + upload_offset_, seg);
        upload_offset_ += seg;
        upload_toggle_ = !upload_toggle_;
        finish_response();
        return;
    }

    coeSdoAbort(index_hi, index_lo, sub, kSdoAbortGeneral);
}

void SlaveEmulator::dumpRegisters() const {
    LOGI("Slave %u registers:", position_);
    LOGI("  AL Status: 0x%04X (%s%s)", al_status_.toRegister(),
         slaveStateToString(al_status_.state), al_status_.error ? " ERROR" : "");
    LOGI("  AL Status Code: 0x%04X", al_status_code_);
    LOGI("  Configured Address: 0x%04X", configured_addr_);
}

void SlaveEmulator::dumpFMMUs() const {
    LOGI("Slave %u FMMUs:", position_);
    for (int i = 0; i < 8; i++) {
        if (fmmus_[i].enabled) {
            LOGI("  FMMU%d: log=0x%08X len=%u phys=0x%04X R=%d W=%d",
                 i, fmmus_[i].logical_start, fmmus_[i].length,
                 fmmus_[i].physical_start, fmmus_[i].read_enable, fmmus_[i].write_enable);
        }
    }
}

void SlaveEmulator::dumpSyncManagers() const {
    LOGI("Slave %u Sync Managers:", position_);
    for (int i = 0; i < 8; i++) {
        if (sync_managers_[i].isEnabled()) {
            LOGI("  SM%d: addr=0x%04X len=%u ctrl=0x%02X",
                 i, sync_managers_[i].start_addr, sync_managers_[i].length,
                 std::bit_cast<uint8_t>(sync_managers_[i].control));
        }
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<SlaveEmulator> createGenericIOSlave(
    uint16_t vendor_id, uint16_t product_code,
    uint8_t digital_inputs, uint8_t digital_outputs,
    uint8_t analog_inputs, uint8_t analog_outputs)
{
    auto slave = std::make_unique<SlaveEmulator>();
    
    SIIConfig config;
    config.vendor_id = vendor_id;
    config.product_code = product_code;
    config.device_name = "Generic I/O";
    
    // Calculate sizes
    uint16_t input_size = (digital_inputs + 7) / 8 + analog_inputs * 2;
    uint16_t output_size = (digital_outputs + 7) / 8 + analog_outputs * 2;
    
    // Configure sync managers
    // SM0: Mailbox In (master -> slave) at 0x1000
    config.sync_managers.push_back({0x1000, 128,
        std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x26)),
        std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    // SM1: Mailbox Out (slave -> master) at 0x1080
    config.sync_managers.push_back({0x1080, 128,
        std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x22)),
        std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    // SM2: Process Data Out (RxPDO) at 0x1100
    config.sync_managers.push_back({0x1100, output_size,
        std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x64)),
        std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    // SM3: Process Data In (TxPDO) at 0x1180
    config.sync_managers.push_back({0x1180, input_size,
        std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x20)),
        std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    
    slave->setSIIConfig(config);
    return slave;
}

std::unique_ptr<SlaveEmulator> createCiA402Drive(
    uint16_t vendor_id, uint16_t product_code,
    const std::string& name)
{
    auto slave = std::make_unique<SlaveEmulator>();
    
    SIIConfig config;
    config.vendor_id = vendor_id;
    config.product_code = product_code;
    config.device_name = name;
    config.supports_dc = true;
    config.cycle_time_0 = 1000000;  // 1ms
    
    // SM configuration for CiA 402
    config.sync_managers.push_back({0x1000, 128, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x26)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});  // Mailbox Out
    config.sync_managers.push_back({0x1080, 128, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x22)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});  // Mailbox In
    config.sync_managers.push_back({0x1100, 12, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x44)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});   // RxPDO (control, target pos, etc.)
    config.sync_managers.push_back({0x1180, 12, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x40)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});   // TxPDO (status, actual pos, etc.)
    
    slave->setSIIConfig(config);
    slave->enableCiA402(true);
    
    return slave;
}

std::unique_ptr<SlaveEmulator> createSimpleSlave(
    uint16_t vendor_id, uint16_t product_code,
    uint16_t input_bytes, uint16_t output_bytes)
{
    auto slave = std::make_unique<SlaveEmulator>();
    
    SIIConfig config;
    config.vendor_id = vendor_id;
    config.product_code = product_code;
    config.device_name = "Simple Slave";
    
    config.sync_managers.push_back({0x1000, 128, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x26)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    config.sync_managers.push_back({0x1080, 128, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x22)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    if (output_bytes > 0) {
        config.sync_managers.push_back({0x1100, output_bytes, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x64)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    }
    if (input_bytes > 0) {
        config.sync_managers.push_back({0x1180, input_bytes, std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(0x20)), std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(0x01))});
    }
    
    slave->setSIIConfig(config);
    return slave;
}

}  // namespace Emulator
}  // namespace EtherCAT
