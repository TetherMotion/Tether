/**
 * @file SlaveEmulator.hpp
 * @brief Full EtherCAT Slave Emulator for Host-Mode Testing
 * 
 * Implements TODO #6: A complete EtherCAT slave emulator that can run in host mode
 * for automated testing without real hardware.
 * 
 * Features:
 * - Full state machine (INIT → PRE-OP → SAFE-OP → OP)
 * - SII (EEPROM) emulation with configurable device info
 * - FMMU emulation (address translation)
 * - Sync Manager emulation
 * - DC (Distributed Clock) emulation
 * - PDO mapping emulation
 * - CoE (CANopen over EtherCAT) SDO support
 * - CiA 401 (I/O) and CiA 402 (Drives) profile emulation
 * - Error injection for testing fault handling
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <mutex>

#include "Types.hpp"
#include "tether/ethercat/SMRegisters.hpp"

namespace EtherCAT {
namespace Emulator {

// ============================================================================
// Forward Declarations
// ============================================================================

class SlaveEmulator;
class NetworkEmulator;

// ============================================================================
// EtherCAT State Machine
// ============================================================================

// Use SlaveState from Types.hpp
using EtherCAT::SlaveState;
using EtherCAT::slaveStateToString;

/// AL Status register value (includes error flag)
struct ALStatus {
    SlaveState state = SlaveState::INIT;
    bool error = false;
    bool id_request = false;
    
    uint16_t toRegister() const {
        uint16_t val = static_cast<uint16_t>(state);
        if (error) val |= 0x0010;
        if (id_request) val |= 0x0020;
        return val;
    }
    
    static ALStatus fromRegister(uint16_t val) {
        ALStatus status;
        status.state = static_cast<SlaveState>(val & 0x0F);
        status.error = (val & 0x0010) != 0;
        status.id_request = (val & 0x0020) != 0;
        return status;
    }
};

// ============================================================================
// SII (Slave Information Interface) Data
// ============================================================================

/**
 * @brief SII EEPROM content configuration
 */
struct SIIConfig {
    // Identity
    uint16_t vendor_id = 0x1234;
    uint16_t product_code = 0x5678;
    uint16_t revision = 0x0001;
    uint32_t serial = 0x00000001;
    
    // Device name
    std::string device_name = "Emulated Slave";
    
    // Hardware/software version
    std::string hw_version = "1.0";
    std::string sw_version = "1.0";
    
    // Mailbox configuration
    uint16_t mailbox_out_size = 128;  // Slave → Master
    uint16_t mailbox_in_size = 128;   // Master → Slave
    uint16_t mailbox_protocol = 0x04; // CoE (bit 2)
    
    // SM configuration
    struct SMConfig {
        uint16_t start_addr = 0;
        uint16_t length = 0;
        EtherCAT::SyncManager::SMControlReg  control{};
        EtherCAT::SyncManager::SMActivateReg enable{};
    };
    std::vector<SMConfig> sync_managers;
    
    // PDO info
    struct PDOEntry {
        uint16_t index;
        uint8_t subindex;
        uint8_t bit_length;
    };
    struct PDOConfig {
        uint16_t index;
        uint8_t sm_num;
        bool is_txpdo;  // TxPDO = slave → master, RxPDO = master → slave
        std::vector<PDOEntry> entries;
    };
    std::vector<PDOConfig> pdos;
    
    // DC configuration
    bool supports_dc = false;
    uint32_t cycle_time_0 = 1000000;  // ns (1ms default)
    uint32_t shift_time = 0;
};

// ============================================================================
// Sync Manager Emulation
// ============================================================================

struct SyncManager {
    uint16_t start_addr = 0;
    uint16_t length = 0;
    EtherCAT::SyncManager::SMControlReg  control{};
    EtherCAT::SyncManager::SMActivateReg enable{};
    EtherCAT::SyncManager::SMStatusReg   status{};
    
    // Buffer for SM data
    std::vector<uint8_t> buffer;
    
    bool isEnabled() const { return enable.enable; }
    bool isMailbox() const { return !control.direction; }
    bool isOutput() const { return control.direction && control.mode == 0x02; }  // RxPDO
    bool isInput() const { return control.direction && control.mode != 0x02; }   // TxPDO
};

// ============================================================================
// FMMU Emulation
// ============================================================================

struct FMMU {
    uint32_t logical_start = 0;
    uint16_t length = 0;
    uint8_t logical_start_bit = 0;
    uint8_t logical_end_bit = 7;
    uint16_t physical_start = 0;
    uint8_t physical_start_bit = 0;
    bool read_enable = false;
    bool write_enable = false;
    bool enabled = false;
    
    /// Check if a logical address falls within this FMMU
    bool containsLogicalAddress(uint32_t addr, uint16_t len) const {
        if (!enabled) return false;
        return addr < (logical_start + length) && (addr + len) > logical_start;
    }
    
    /// Translate logical address to physical
    uint16_t translateToPhysical(uint32_t logical_addr) const {
        return physical_start + (logical_addr - logical_start);
    }
};

// ============================================================================
// Distributed Clock Emulation
// ============================================================================

struct DCState {
    // DC registers
    uint64_t system_time = 0;          // 0x0910
    uint64_t receive_time = 0;         // 0x0918 (port 0)
    int64_t system_time_offset = 0;    // 0x0920
    int32_t system_time_delay = 0;     // 0x0928
    uint32_t system_time_diff = 0;     // 0x092C
    uint16_t speed_counter = 0;        // 0x0930 (filter depth)
    uint16_t speed_counter_diff = 0;   // 0x0932
    
    // DC control
    bool dc_active = false;
    uint32_t cycle_time_0 = 0;         // 0x09A0 (SYNC0 cycle)
    uint32_t cycle_time_1 = 0;         // 0x09A4 (SYNC1 cycle)
    int32_t sync0_shift = 0;           // 0x09A8
    
    // SYNC signals
    bool sync0_enable = false;
    bool sync1_enable = false;
    
    /// Advance simulated time
    void advanceTime(uint64_t delta_ns) {
        system_time += delta_ns;
    }
};

// ============================================================================
// CoE (CANopen over EtherCAT) Object Dictionary
// ============================================================================

struct ODEntry {
    uint16_t index;
    uint8_t subindex;
    uint16_t data_type;
    uint16_t bit_length;
    uint8_t access;  // 0x01=read, 0x02=write
    std::vector<uint8_t> data;
    std::string name;
    
    bool isReadable() const { return (access & 0x01) != 0; }
    bool isWritable() const { return (access & 0x02) != 0; }
};

// ============================================================================
// CiA 402 (Drives) State Machine
// ============================================================================

namespace CiA402 {

enum class State : uint8_t {
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED = 1,
    READY_TO_SWITCH_ON = 2,
    SWITCHED_ON = 3,
    OPERATION_ENABLED = 4,
    QUICK_STOP_ACTIVE = 5,
    FAULT_REACTION_ACTIVE = 6,
    FAULT = 7
};

struct DriveState {
    State state = State::SWITCH_ON_DISABLED;
    
    // Status word (0x6041)
    uint16_t getStatusWord() const;
    
    // Process control word (0x6040)
    void processControlWord(uint16_t controlword);
    
    // Operation mode (0x6060/0x6061)
    int8_t operation_mode = 0;  // Current mode
    int8_t target_mode = 0;     // Requested mode
    
    // Position/velocity/torque
    int32_t target_position = 0;     // 0x607A
    int32_t actual_position = 0;     // 0x6064
    int32_t target_velocity = 0;     // 0x60FF
    int32_t actual_velocity = 0;     // 0x606C
    int16_t target_torque = 0;       // 0x6071
    int16_t actual_torque = 0;       // 0x6077
    
    // Homing
    bool homing_complete = false;

    // Fault-reset edge detection (per-instance, latched between cycles)
    bool prev_reset = false;

    // Fault code
    uint16_t error_code = 0;         // 0x603F
    
    // Simulate motion (call periodically)
    void simulate(uint32_t delta_us);
};

}  // namespace CiA402

// ============================================================================
// Error Injection
// ============================================================================

/**
 * @brief Configuration for injecting errors during testing
 */
struct ErrorInjection {
    bool inject_al_error = false;
    uint16_t al_error_code = 0;
    
    bool inject_wkc_error = false;  // Return wrong WKC
    
    bool inject_timeout = false;    // Don't respond
    uint16_t timeout_register = 0;  // Only for specific register
    
    bool inject_dc_drift = false;   // Simulate clock drift
    int32_t dc_drift_ppb = 0;       // Parts per billion
    
    bool inject_sync_error = false; // Err74.1 No Sync
    
    void clear() {
        inject_al_error = false;
        inject_wkc_error = false;
        inject_timeout = false;
        inject_dc_drift = false;
        inject_sync_error = false;
    }
};

// ============================================================================
// Slave Emulator Class
// ============================================================================

/**
 * @brief Emulates a single EtherCAT slave
 * 
 * Usage:
 * @code
 * SlaveEmulator slave;
 * slave.setSIIConfig(config);
 * slave.setPosition(0);  // First slave
 * 
 * // Process commands from master
 * slave.processAPRD(ado, buffer, len);
 * slave.processAPWR(ado, data, len);
 * @endcode
 */
class SlaveEmulator {
public:
    SlaveEmulator();
    ~SlaveEmulator() = default;

    // Configuration
    void setSIIConfig(const SIIConfig& config);
    void setPosition(uint16_t position);
    void setConfiguredAddress(uint16_t addr);

    // State management
    SlaveState getState() const { return al_status_.state; }
    ALStatus getALStatus() const { return al_status_; }
    void requestState(SlaveState state);

    // Register access (for auto-increment commands)
    bool processAPRD(uint16_t ado, uint8_t* data, uint16_t len);
    bool processAPWR(uint16_t ado, const uint8_t* data, uint16_t len);

    // Register access (for configured address commands)
    bool processFPRD(uint16_t ado, uint8_t* data, uint16_t len);
    bool processFPWR(uint16_t ado, const uint8_t* data, uint16_t len);

    // Logical access (via FMMU)
    bool processLogicalRead(uint32_t logical_addr, uint8_t* data, uint16_t len);
    bool processLogicalWrite(uint32_t logical_addr, const uint8_t* data, uint16_t len);

    // SII/EEPROM access
    bool processSIIRead(uint32_t word_addr, uint16_t* data);
    bool processSIIWrite(uint32_t word_addr, uint16_t data);

    // CoE mailbox
    bool hasMailboxData() const;
    std::vector<uint8_t> getMailboxResponse();
    void processMailboxRequest(const uint8_t* data, size_t len);

    // DC
    DCState& getDCState() { return dc_state_; }
    void advanceDCTime(uint64_t delta_ns);

    // CiA 402 drive
    CiA402::DriveState& getDriveState() { return drive_state_; }
    void enableCiA402(bool enable);
    bool isCiA402Enabled() const { return cia402_enabled_; }

    // Error injection
    void setErrorInjection(const ErrorInjection& errors) { errors_ = errors; }
    ErrorInjection& getErrorInjection() { return errors_; }

    // Simulation
    void simulate(uint32_t delta_us);

    // Debug
    void dumpRegisters() const;
    void dumpFMMUs() const;
    void dumpSyncManagers() const;

private:
    // Internal register read/write
    bool readRegister(uint16_t addr, uint8_t* data, uint16_t len);
    bool writeRegister(uint16_t addr, const uint8_t* data, uint16_t len);

    // State transition
    bool canTransition(SlaveState from, SlaveState to);
    void doTransition(SlaveState new_state);

    // Object dictionary
    void initObjectDictionary();
    ODEntry* findODEntry(uint16_t index, uint8_t subindex);

    // SDO processing
    std::vector<uint8_t> processSDORequest(const uint8_t* data, size_t len);

    // PDO mapping update
    void updatePDOMapping();

    // Position/address
    uint16_t position_ = 0;
    uint16_t configured_addr_ = 0;

    // State
    ALStatus al_status_;
    uint16_t al_status_code_ = 0;  // 0x0134

    // SII
    SIIConfig sii_config_;
    std::vector<uint8_t> sii_data_;  // EEPROM content

    // Sync Managers (8 max)
    std::array<SyncManager, 8> sync_managers_;

    // FMMUs (8 max in most slaves)
    std::array<FMMU, 8> fmmus_;

    // DC
    DCState dc_state_;

    // Object dictionary
    std::vector<ODEntry> object_dictionary_;

    // CiA 402
    bool cia402_enabled_ = false;
    CiA402::DriveState drive_state_;

    // Mailbox
    std::vector<uint8_t> mailbox_request_;
    std::vector<uint8_t> mailbox_response_;

    // Error injection
    ErrorInjection errors_;

    // Register memory (0x0000 - 0x0FFF typical)
    std::array<uint8_t, 4096> registers_;
};

// ============================================================================
// Network Emulator Class
// ============================================================================

/**
 * @brief Emulates an EtherCAT network with multiple slaves
 * 
 * Processes complete EtherCAT frames and returns responses as if
 * the frame traveled through all slaves in sequence.
 * 
 * Usage:
 * @code
 * NetworkEmulator network;
 * network.addSlave(std::make_unique<SlaveEmulator>());
 * network.addSlave(std::make_unique<SlaveEmulator>());
 * 
 * // Process a frame (returns response frame)
 * auto response = network.processFrame(frame_data, frame_len);
 * @endcode
 */
class NetworkEmulator {
public:
    NetworkEmulator();
    ~NetworkEmulator() = default;

    /// Add a slave to the network (takes ownership)
    void addSlave(std::unique_ptr<SlaveEmulator> slave);

    /// Get slave by position (0-based)
    SlaveEmulator* getSlave(size_t index);

    /// Get number of slaves
    size_t getSlaveCount() const { return slaves_.size(); }

    /// Clear all slaves
    void clearSlaves();

    /**
     * @brief Process an EtherCAT frame
     * 
     * Parses the frame, routes datagrams to appropriate slaves,
     * and builds a response frame.
     * 
     * @param frame_data Raw Ethernet frame data
     * @param frame_len Length of frame
     * @return Response frame data (or empty if no response needed)
     */
    std::vector<uint8_t> processFrame(const uint8_t* frame_data, size_t frame_len);

    /// Advance simulation time for all slaves
    void simulate(uint32_t delta_us);

    /// Set global error injection (affects all slaves)
    void setGlobalErrorInjection(const ErrorInjection& errors);

    /// Get statistics
    struct NetworkStats {
        uint64_t frames_processed = 0;
        uint64_t datagrams_processed = 0;
        uint64_t wkc_errors = 0;
        uint64_t unknown_commands = 0;
    };
    NetworkStats getStats() const { return stats_; }
    void resetStats() { stats_ = {}; }

private:
    // Process individual datagrams
    uint16_t processDatagram(Command cmd, uint8_t idx,
                             uint16_t adp, uint16_t ado,
                             uint8_t* data, uint16_t datalen);

    // Helpers for specific command types
    uint16_t processAutoIncrement(Command cmd, int16_t adp, uint16_t ado,
                                   uint8_t* data, uint16_t len);
    uint16_t processConfiguredAddr(Command cmd, uint16_t addr, uint16_t ado,
                                    uint8_t* data, uint16_t len);
    uint16_t processBroadcast(Command cmd, uint16_t ado,
                               uint8_t* data, uint16_t len);
    uint16_t processLogical(Command cmd, uint32_t logical_addr,
                             uint8_t* data, uint16_t len);

    std::vector<std::unique_ptr<SlaveEmulator>> slaves_;
    NetworkStats stats_;
    std::mutex mutex_;  // Thread safety
};

// ============================================================================
// Factory Functions for Common Slave Profiles
// ============================================================================

/// Create a generic I/O slave (CiA 401 style)
std::unique_ptr<SlaveEmulator> createGenericIOSlave(
    uint16_t vendor_id, uint16_t product_code,
    uint8_t digital_inputs, uint8_t digital_outputs,
    uint8_t analog_inputs = 0, uint8_t analog_outputs = 0);

/// Create a CiA 402 servo drive
std::unique_ptr<SlaveEmulator> createCiA402Drive(
    uint16_t vendor_id, uint16_t product_code,
    const std::string& name = "Emulated Drive");

/// Create a simple slave with configurable PDOs
std::unique_ptr<SlaveEmulator> createSimpleSlave(
    uint16_t vendor_id, uint16_t product_code,
    uint16_t input_bytes, uint16_t output_bytes);

}  // namespace emulator
}  // namespace EtherCAT
