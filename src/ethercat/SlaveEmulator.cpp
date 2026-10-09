/**
 * @file SlaveEmulator.cpp
 * @brief Implementation of EtherCAT Slave Emulator
 */

#include "SlaveEmulator.hpp"
#include "tether/ethercat/DCClass.hpp"
#include "tether/platform/EspCompat.hpp"

#include <bit>

#define LOG_TAG "EC_EMU"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) TETHER_LOGD(LOG_TAG, fmt, ##__VA_ARGS__)

namespace EtherCAT {
namespace Emulator {

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
        sync_managers_[i].buffer.resize(config.sync_managers[i].length, 0);
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
    
    // Sync Manager registers (0x0800 - 0x087F)
    if (addr >= 0x0800 && addr < 0x0880) {
        uint8_t sm_num = (addr - 0x0800) / 8;
        uint8_t sm_offset = (addr - 0x0800) % 8;
        
        if (sm_num < 8) {
            SyncManager& sm = sync_managers_[sm_num];
            switch (sm_offset) {
                case 0:  // Start Address (2 bytes)
                    if (len >= 2) {
                        data[0] = sm.start_addr & 0xFF;
                        data[1] = (sm.start_addr >> 8) & 0xFF;
                    }
                    return true;
                case 2:  // Length (2 bytes)
                    if (len >= 2) {
                        data[0] = sm.length & 0xFF;
                        data[1] = (sm.length >> 8) & 0xFF;
                    }
                    return true;
                case 4:  // Control
                    data[0] = std::bit_cast<uint8_t>(sm.control);
                    return true;
                case 5:  // Status
                    data[0] = std::bit_cast<uint8_t>(sm.status);
                    return true;
                case 6:  // Enable
                    data[0] = std::bit_cast<uint8_t>(sm.enable);
                    return true;
            }
        }
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
    
    // Sync Manager registers
    if (addr >= 0x0800 && addr < 0x0880) {
        uint8_t sm_num = (addr - 0x0800) / 8;
        uint8_t sm_offset = (addr - 0x0800) % 8;
        
        if (sm_num < 8) {
            SyncManager& sm = sync_managers_[sm_num];
            switch (sm_offset) {
                case 0:
                    if (len >= 2) {
                        sm.start_addr = data[0] | (data[1] << 8);
                    }
                    return true;
                case 2:
                    if (len >= 2) {
                        sm.length = data[0] | (data[1] << 8);
                        sm.buffer.resize(sm.length, 0);
                    }
                    return true;
                case 4:
                    sm.control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(data[0]);
                    return true;
                case 6:
                    sm.enable = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(data[0]);
                    return true;
            }
        }
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
    
    // Generic register write
    if (addr + len <= registers_.size()) {
        std::memcpy(&registers_[addr], data, len);
        return true;
    }
    
    return false;
}

bool SlaveEmulator::processLogicalRead(uint32_t logical_addr, uint8_t* data, uint16_t len) {
    // Find FMMU that handles this address
    for (auto& fmmu : fmmus_) {
        if (fmmu.containsLogicalAddress(logical_addr, len) && fmmu.read_enable) {
            uint16_t phys = fmmu.translateToPhysical(logical_addr);
            
            // Find which SM contains this physical address
            for (auto& sm : sync_managers_) {
                if (sm.isEnabled() && phys >= sm.start_addr && 
                    phys + len <= sm.start_addr + sm.length) {
                    uint16_t offset = phys - sm.start_addr;
                    std::memcpy(data, &sm.buffer[offset], len);
                    return true;
                }
            }
        }
    }
    return false;
}

bool SlaveEmulator::processLogicalWrite(uint32_t logical_addr, const uint8_t* data, uint16_t len) {
    // Find FMMU that handles this address
    for (auto& fmmu : fmmus_) {
        if (fmmu.containsLogicalAddress(logical_addr, len) && fmmu.write_enable) {
            uint16_t phys = fmmu.translateToPhysical(logical_addr);
            
            // Find which SM contains this physical address
            for (auto& sm : sync_managers_) {
                if (sm.isEnabled() && phys >= sm.start_addr && 
                    phys + len <= sm.start_addr + sm.length) {
                    uint16_t offset = phys - sm.start_addr;
                    std::memcpy(&sm.buffer[offset], data, len);
                    return true;
                }
            }
        }
    }
    return false;
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
