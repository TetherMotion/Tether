/**
 * @file EtherCATSlaveEmulator.cpp
 * @brief Implementation of EtherCAT Slave Emulator
 */

#include "EtherCATSlaveEmulator.hpp"
#include "tether/ethercat/EtherCATDCClass.hpp"
#include "tether/platform/EspCompat.hpp"

#define LOG_TAG "EC_EMU"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) TETHER_LOGD(LOG_TAG, fmt, ##__VA_ARGS__)

namespace EtherCAT {
namespace Emulator {

// ============================================================================
// CiA 402 Drive State Implementation
// ============================================================================

namespace CiA402 {

uint16_t DriveState::getStatusWord() const {
    uint16_t sw = 0;
    
    switch (state) {
        case State::NOT_READY_TO_SWITCH_ON:
            sw = 0x0000;
            break;
        case State::SWITCH_ON_DISABLED:
            sw = 0x0040;
            break;
        case State::READY_TO_SWITCH_ON:
            sw = 0x0021;
            break;
        case State::SWITCHED_ON:
            sw = 0x0023;
            break;
        case State::OPERATION_ENABLED:
            sw = 0x0027;
            break;
        case State::QUICK_STOP_ACTIVE:
            sw = 0x0007;
            break;
        case State::FAULT_REACTION_ACTIVE:
            sw = 0x000F;
            break;
        case State::FAULT:
            sw = 0x0008;
            break;
    }
    
    // Add mode-specific bits
    if (operation_mode == target_mode) {
        sw |= 0x0400;  // Target reached / mode-specific
    }
    
    if (homing_complete && operation_mode == 6) {
        sw |= 0x1000;  // Homing attained
    }
    
    return sw;
}

void DriveState::processControlWord(uint16_t cw) {
    // State machine transitions based on control word
    switch (state) {
        case State::NOT_READY_TO_SWITCH_ON:
            // Automatic transition to SWITCH_ON_DISABLED
            state = State::SWITCH_ON_DISABLED;
            break;
            
        case State::SWITCH_ON_DISABLED:
            // Shutdown command: bit 0=0, bit 1=1, bit 2=0
            if ((cw & 0x0007) == 0x0006) {
                state = State::READY_TO_SWITCH_ON;
            }
            break;
            
        case State::READY_TO_SWITCH_ON:
            // Switch on command: bit 0=1, bit 1=1, bit 2=0
            if ((cw & 0x0007) == 0x0007) {
                state = State::SWITCHED_ON;
            }
            // Disable voltage: bit 1=0
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::SWITCHED_ON:
            // Enable operation: bit 0=1, bit 1=1, bit 2=1, bit 3=1
            if ((cw & 0x000F) == 0x000F) {
                state = State::OPERATION_ENABLED;
            }
            // Shutdown
            else if ((cw & 0x0007) == 0x0006) {
                state = State::READY_TO_SWITCH_ON;
            }
            // Disable voltage
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::OPERATION_ENABLED:
            // Quick stop: bit 2=0
            if ((cw & 0x0004) == 0) {
                state = State::QUICK_STOP_ACTIVE;
            }
            // Disable operation: bit 3=0
            else if ((cw & 0x0008) == 0) {
                state = State::SWITCHED_ON;
            }
            // Shutdown
            else if ((cw & 0x0007) == 0x0006) {
                state = State::READY_TO_SWITCH_ON;
            }
            // Disable voltage
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::QUICK_STOP_ACTIVE:
            // Enable operation from quick stop
            if ((cw & 0x000F) == 0x000F) {
                state = State::OPERATION_ENABLED;
            }
            // Disable voltage
            else if ((cw & 0x0002) == 0) {
                state = State::SWITCH_ON_DISABLED;
            }
            break;
            
        case State::FAULT_REACTION_ACTIVE:
            // Automatic transition to FAULT
            state = State::FAULT;
            break;
            
        case State::FAULT:
            // Fault reset: bit 7 rising edge
            static bool prev_reset = false;
            bool reset = (cw & 0x0080) != 0;
            if (reset && !prev_reset) {
                state = State::SWITCH_ON_DISABLED;
                error_code = 0;
            }
            prev_reset = reset;
            break;
    }
}

void DriveState::simulate(uint32_t delta_us) {
    if (state != State::OPERATION_ENABLED) {
        actual_velocity = 0;
        return;
    }
    
    // Simple motion simulation based on mode
    switch (operation_mode) {
        case 1:  // Profile Position
        {
            int32_t error = target_position - actual_position;
            int32_t max_step = 1000 * delta_us / 1000;  // Scale with time
            if (error > max_step) error = max_step;
            else if (error < -max_step) error = -max_step;
            actual_position += error;
            actual_velocity = error * 1000000 / static_cast<int32_t>(delta_us);
            break;
        }
        case 3:  // Profile Velocity
            actual_velocity = target_velocity;  // Instant for simulation
            actual_position += actual_velocity * delta_us / 1000000;
            break;
        case 4:  // Profile Torque
            actual_torque = target_torque;
            break;
        case 8:  // Cyclic Sync Position
            actual_position = target_position;
            break;
        case 9:  // Cyclic Sync Velocity
            actual_velocity = target_velocity;
            actual_position += actual_velocity * delta_us / 1000000;
            break;
        case 10: // Cyclic Sync Torque
            actual_torque = target_torque;
            break;
    }
}

}  // namespace CiA402

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
                    data[0] = sm.control;
                    return true;
                case 5:  // Status
                    data[0] = sm.status;
                    return true;
                case 6:  // Enable
                    data[0] = sm.enable;
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
                    sm.control = data[0];
                    return true;
                case 6:
                    sm.enable = data[0];
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
                 sync_managers_[i].control);
        }
    }
}

// ============================================================================
// NetworkEmulator Implementation
// ============================================================================

NetworkEmulator::NetworkEmulator() {
}

void NetworkEmulator::addSlave(std::unique_ptr<SlaveEmulator> slave) {
    std::lock_guard<std::mutex> lock(mutex_);
    slave->setPosition(slaves_.size());
    slaves_.push_back(std::move(slave));
}

SlaveEmulator* NetworkEmulator::getSlave(size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < slaves_.size()) {
        return slaves_[index].get();
    }
    return nullptr;
}

void NetworkEmulator::clearSlaves() {
    std::lock_guard<std::mutex> lock(mutex_);
    slaves_.clear();
}

std::vector<uint8_t> NetworkEmulator::processFrame(const uint8_t* frame_data, size_t frame_len) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (frame_len < 14 + 2) {  // Ethernet header + EtherCAT header
        return {};
    }
    
    // Check EtherType (0x88A4 for EtherCAT)
    uint16_t ethertype = (frame_data[12] << 8) | frame_data[13];
    if (ethertype != 0x88A4) {
        return {};
    }
    
    stats_.frames_processed++;
    
    // Build response frame (copy header)
    std::vector<uint8_t> response(frame_data, frame_data + frame_len);
    
    // Swap source/dest MAC for response
    std::swap_ranges(response.begin(), response.begin() + 6, response.begin() + 6);
    
    // EtherCAT header at offset 14
    uint16_t ecat_len = (frame_data[14]) | ((frame_data[15] & 0x07) << 8);
    (void)ecat_len;  // Length of datagram area
    
    // Parse datagrams starting at offset 16
    size_t offset = 16;
    while (offset + 10 <= frame_len) {  // Minimum datagram header size
        uint8_t cmd = frame_data[offset];
        uint8_t idx = frame_data[offset + 1];
        uint16_t adp = frame_data[offset + 2] | (frame_data[offset + 3] << 8);
        uint16_t ado = frame_data[offset + 4] | (frame_data[offset + 5] << 8);
        uint16_t len_flags = frame_data[offset + 6] | (frame_data[offset + 7] << 8);
        uint16_t datalen = len_flags & 0x07FF;
        bool more = (len_flags & 0x8000) != 0;
        
        if (offset + 10 + datalen + 2 > frame_len) {
            break;  // Invalid frame
        }
        
        // Get pointer to data in response
        uint8_t* data_ptr = &response[offset + 10];
        
        // Process datagram and get WKC
        uint16_t wkc = processDatagram(
            static_cast<Command>(cmd), idx, adp, ado, data_ptr, datalen);
        
        // Write WKC to response (after data)
        response[offset + 10 + datalen] = wkc & 0xFF;
        response[offset + 10 + datalen + 1] = (wkc >> 8) & 0xFF;
        
        stats_.datagrams_processed++;
        
        if (!more) break;
        offset += 10 + datalen + 2;
    }
    
    return response;
}

uint16_t NetworkEmulator::processDatagram(Command cmd, uint8_t idx,
                                           uint16_t adp, uint16_t ado,
                                           uint8_t* data, uint16_t datalen)
{
    (void)idx;
    
    switch (cmd) {
        case Command::APRD:
        case Command::APWR:
        case Command::APRW:
            return processAutoIncrement(cmd, static_cast<int16_t>(adp), ado, data, datalen);
            
        case Command::FPRD:
        case Command::FPWR:
        case Command::FPRW:
            return processConfiguredAddr(cmd, adp, ado, data, datalen);
            
        case Command::BRD:
        case Command::BWR:
        case Command::BRW:
            return processBroadcast(cmd, ado, data, datalen);
            
        case Command::LRD:
        case Command::LWR:
        case Command::LRW:
        {
            uint32_t logical_addr = (static_cast<uint32_t>(adp) << 16) | ado;
            return processLogical(cmd, logical_addr, data, datalen);
        }
            
        default:
            stats_.unknown_commands++;
            return 0;
    }
}

uint16_t NetworkEmulator::processAutoIncrement(Command cmd, int16_t adp, uint16_t ado,
                                                uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    // ADP starts negative, increments as frame passes through each slave
    // Slave processes when ADP == 0
    int16_t current_adp = adp;
    
    for (auto& slave : slaves_) {
        if (current_adp == 0) {
            // This slave processes the datagram
            bool ok = false;
            switch (cmd) {
                case Command::APRD:
                    ok = slave->processAPRD(ado, data, len);
                    break;
                case Command::APWR:
                    ok = slave->processAPWR(ado, data, len);
                    break;
                case Command::APRW:
                    // Read first, then write (in practice, simultaneous)
                    ok = slave->processAPRD(ado, data, len);
                    break;
                default:
                    break;
            }
            if (ok) {
                wkc += (cmd == Command::APRW) ? 3 : 1;
            }
        }
        current_adp++;
    }
    
    return wkc;
}

uint16_t NetworkEmulator::processConfiguredAddr(Command cmd, uint16_t addr, uint16_t ado,
                                                 uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    for (auto& slave : slaves_) {
        // Check configured station address
        if (slave->getALStatus().state != SlaveState::INIT) {
            uint8_t addr_low, addr_high;
            slave->processFPRD(0x0010, &addr_low, 1);
            slave->processFPRD(0x0011, &addr_high, 1);
            uint16_t slave_addr = addr_low | (addr_high << 8);
            
            if (slave_addr == addr) {
                bool ok = false;
                switch (cmd) {
                    case Command::FPRD:
                        ok = slave->processFPRD(ado, data, len);
                        break;
                    case Command::FPWR:
                        ok = slave->processFPWR(ado, data, len);
                        break;
                    case Command::FPRW:
                        ok = slave->processFPRD(ado, data, len);
                        break;
                    default:
                        break;
                }
                if (ok) {
                    wkc += (cmd == Command::FPRW) ? 3 : 1;
                }
                break;  // Found the slave
            }
        }
    }
    
    return wkc;
}

uint16_t NetworkEmulator::processBroadcast(Command cmd, uint16_t ado,
                                            uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    for (auto& slave : slaves_) {
        bool ok = false;
        switch (cmd) {
            case Command::BRD:
                ok = slave->processAPRD(ado, data, len);
                // For broadcast read, data is OR'd from all slaves
                break;
            case Command::BWR:
                ok = slave->processAPWR(ado, data, len);
                break;
            case Command::BRW:
                ok = slave->processAPRD(ado, data, len);
                break;
            default:
                break;
        }
        if (ok) {
            wkc++;
        }
    }
    
    return wkc;
}

uint16_t NetworkEmulator::processLogical(Command cmd, uint32_t logical_addr,
                                          uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    for (auto& slave : slaves_) {
        bool ok = false;
        switch (cmd) {
            case Command::LRD:
                ok = slave->processLogicalRead(logical_addr, data, len);
                if (ok) wkc += 1;
                break;
            case Command::LWR:
                ok = slave->processLogicalWrite(logical_addr, data, len);
                if (ok) wkc += 1;
                break;
            case Command::LRW:
                // Read from TxPDO (inputs), write to RxPDO (outputs)
                if (slave->processLogicalRead(logical_addr, data, len)) {
                    wkc += 1;
                }
                if (slave->processLogicalWrite(logical_addr, data, len)) {
                    wkc += 2;
                }
                break;
            default:
                break;
        }
    }
    
    return wkc;
}

void NetworkEmulator::simulate(uint32_t delta_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slave : slaves_) {
        slave->simulate(delta_us);
        slave->advanceDCTime(delta_us * 1000);  // Convert to ns
    }
}

void NetworkEmulator::setGlobalErrorInjection(const ErrorInjection& errors) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slave : slaves_) {
        slave->setErrorInjection(errors);
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
    // SM0: Mailbox Out (slave -> master) at 0x1000
    config.sync_managers.push_back({0x1000, 128, 0x26, 0x01});
    // SM1: Mailbox In (master -> slave) at 0x1080
    config.sync_managers.push_back({0x1080, 128, 0x22, 0x01});
    // SM2: Process Data Out (RxPDO) at 0x1100
    config.sync_managers.push_back({0x1100, output_size, 0x64, 0x01});
    // SM3: Process Data In (TxPDO) at 0x1180
    config.sync_managers.push_back({0x1180, input_size, 0x20, 0x01});
    
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
    config.sync_managers.push_back({0x1000, 128, 0x26, 0x01});  // Mailbox Out
    config.sync_managers.push_back({0x1080, 128, 0x22, 0x01});  // Mailbox In
    config.sync_managers.push_back({0x1100, 12, 0x64, 0x01});   // RxPDO (control, target pos, etc.)
    config.sync_managers.push_back({0x1180, 12, 0x20, 0x01});   // TxPDO (status, actual pos, etc.)
    
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
    
    config.sync_managers.push_back({0x1000, 128, 0x26, 0x01});
    config.sync_managers.push_back({0x1080, 128, 0x22, 0x01});
    if (output_bytes > 0) {
        config.sync_managers.push_back({0x1100, output_bytes, 0x64, 0x01});
    }
    if (input_bytes > 0) {
        config.sync_managers.push_back({0x1180, input_bytes, 0x20, 0x01});
    }
    
    slave->setSIIConfig(config);
    return slave;
}

}  // namespace emulator
}  // namespace EtherCAT
