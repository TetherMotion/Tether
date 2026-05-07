#pragma once

/**
 * @file EtherCATDCClass.hpp
 * @brief Class-based EtherCAT Distributed Clock (DC) synchronization
 *
 * This module provides an instance-based DC implementation allowing
 * multiple independent DC instances for different EtherCAT masters.
 */

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

#include "tether/platform/Platform.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/IDCTransport.hpp"

namespace EtherCAT {

// Forward declarations
class EtherCATRealtimeLoop;

// Maximum number of slaves for DC synchronization
constexpr size_t kMaxDCSlaves = 16;

/**
 * @brief DC synchronization state for a single slave
 */
struct SlaveTimeInfo {
    uint64_t system_time_ns;        ///< Slave's DC system time (64-bit)
    uint64_t receive_time_ns;       ///< Port 0 receive time for delay measurement
    int64_t  offset_to_master_ns;   ///< Calculated offset from master reference
    uint32_t propagation_delay_ns;  ///< Propagation delay to this slave
    uint64_t sync0_start_time_ns;   ///< SYNC0 start time (from slave if fixed)
    bool     dc_supported;          ///< Slave supports DC (has 64-bit local time)
    bool     dc_active;             ///< DC synchronization is actively running
};

/**
 * @brief DC loop statistics
 */
struct DCLoopStats {
    uint64_t cycle_count;           ///< Total cycles executed
    uint64_t sync_count;            ///< Number of synchronization frames sent
    uint64_t pdo_error_count;       ///< Number of PDO exchange errors
    uint32_t max_jitter_us;         ///< Maximum observed jitter in microseconds
    uint32_t avg_jitter_us;         ///< Average jitter in microseconds
    int64_t  last_drift_ns;         ///< Last measured clock drift in nanoseconds
    uint64_t last_master_time_ns;   ///< Last master time used for sync
};

/**
 * @brief DC synchronization configuration
 */
struct DCConfig {
    uint32_t cycle_period_us;       ///< Realtime loop period in microseconds (default 1000 = 1kHz)
    uint32_t sync_interval_cycles;  ///< Number of cycles between full DC sync (default 10)
    uint32_t sync0_cycle_time_ns;   ///< SYNC0 cycle time in nanoseconds (0 = disabled)
    uint32_t sync1_cycle_time_ns;   ///< SYNC1 cycle time in nanoseconds (0 = disabled)
    int32_t  sync0_shift_ns;        ///< SYNC0 shift time in nanoseconds
    bool     enable_sync0;          ///< Enable SYNC0 signal generation
    bool     enable_sync1;          ///< Enable SYNC1 signal generation

    static DCConfig defaults() {
        return DCConfig{
            .cycle_period_us = 1000,        // 1kHz loop
            .sync_interval_cycles = 10,     // Sync every 10ms
            .sync0_cycle_time_ns = 1000000, // 1ms SYNC0 cycle
            .sync1_cycle_time_ns = 0,       // SYNC1 disabled
            .sync0_shift_ns = 0,
            .enable_sync0 = true,
            .enable_sync1 = false
        };
    }
};

/**
 * @brief DC synchronization state machine states
 */
enum class DCState : uint8_t {
    Disabled = 0,       ///< DC not initialized
    Initializing,       ///< Reading slave DC capabilities
    PropagationCalc,    ///< Calculating propagation delays
    DriftCompensation,  ///< Initial drift compensation
    Running,            ///< Normal operation with periodic sync
    Error               ///< Error state, needs reset
};

/**
 * @brief Convert DCState to string for logging
 */
inline const char* dc_state_name(DCState state) {
    switch (state) {
        case DCState::Disabled:         return "Disabled";
        case DCState::Initializing:     return "Initializing";
        case DCState::PropagationCalc:  return "PropagationCalc";
        case DCState::DriftCompensation: return "DriftCompensation";
        case DCState::Running:          return "Running";
        case DCState::Error:            return "Error";
        default:                        return "Unknown";
    }
}

/**
 * @name DC register addresses
 * @brief Standard EtherCAT Distributed Clock (DC) register offsets.
 *
 * These values correspond to the commonly-used DC CoE/SDO register
 * addresses. They are provided here for use in diagnostics, tests and
 * for low-level register I/O. The sizes shown in the descriptions
 * reflect the common usage in this codebase (e.g. 64-bit times, 32-bit
 * cycle times).
 *\@{
 */

/**
 * @brief EtherCAT DC register offsets
 */
enum class DCRegisters : uint16_t {
    DCSysTime      = 0x0910, ///< 64-bit DC system time (nanoseconds)
    DCRecvTimes    = 0x0900, ///< Receive timestamp registers
    DCSysTxTime    = 0x0928, ///< 64-bit transmit time (TX timestamp)
    DCSysOffset    = 0x0920, ///< 64-bit system time offset
    DCSysDiff      = 0x092C, ///< System time difference
    DCStartOfFrame = 0x0918, ///< Start-of-frame time
    DCSpeedCnt     = 0x0930, ///< Speed/counter-related register
    DCTimeFilter   = 0x0934, ///< Time filtering / smoothing register
    DCCuc          = 0x0980, ///< DC control unit configuration
    DCSyncAct      = 0x0981, ///< SYNC activation register (8-bit)
    DCSyncLatch    = 0x098E, ///< SYNC latch status
    DCStart0       = 0x0990, ///< SYNC0 start time (64-bit)
    DCCycle0       = 0x09A0, ///< SYNC0 cycle time (32-bit)
    DCCycle1       = 0x09A4, ///< SYNC1 cycle time (32-bit)
};

/**
 * @brief Helper: convert DCRegisters to its uint16_t register address
 *
 * Use this to avoid repetitive explicit casts in call-sites.
 */
inline constexpr uint16_t toUInt16(DCRegisters reg) noexcept {
    return static_cast<uint16_t>(reg);
}

/**
 * @brief Bits for the DCSyncAct register
 *
 * These bits control enabling the SYNC unit and individual SYNC outputs
 * as well as automatic activation behaviour.
 */
enum DCSyncActBits : uint8_t {
    DC_SYNCACT_ENA        = 0x01, ///< Enable the SYNC activation unit
    DC_SYNCACT_SYNC0_ENA  = 0x02, ///< Enable SYNC0 output
    DC_SYNCACT_SYNC1_ENA  = 0x04, ///< Enable SYNC1 output
    DC_SYNCACT_AUTO_ACT   = 0x08, ///< Auto-activation (use start time register)
};

/*\@*/

/**
 * @brief EtherCAT Distributed Clock (DC) synchronization class
 * 
 * This class provides instance-based DC synchronization, allowing multiple
 * independent DC instances for different EtherCAT masters.
 * 
 * Key features:
 * - Platform-independent timer abstraction (ESP32 gptimer, host std::thread)
 * - 1kHz realtime loop for periodic synchronization
 * - SYNC0/SYNC1 signal generation
 * - Propagation delay calculation and compensation
 * - Master/slave clock offset management
 * - Optional PDO exchange in realtime loop
 * 
 * Thread-safety: Methods are thread-safe unless noted otherwise.
 */
class EtherCATDC {
public:
    /**
     * @brief Construct a DC instance
     * 
     * @param transport Transport interface for DC register I/O
     * @param slave_count Number of slaves to synchronize
     * @param config DC configuration (uses defaults if nullptr)
     */
    EtherCATDC(IDCTransport& transport,
               uint16_t slave_count,
               const DCConfig* config = nullptr);

    /**
     * @brief Destructor - stops DC and cleans up resources
     */
    virtual ~EtherCATDC();

    // Non-copyable, non-movable (transport reference prevents move)
    EtherCATDC(const EtherCATDC&) = delete;
    EtherCATDC& operator=(const EtherCATDC&) = delete;
    EtherCATDC(EtherCATDC&&) = delete;
    EtherCATDC& operator=(EtherCATDC&&) = delete;

    /**
     * @brief Start the DC realtime loop
     * 
     * Creates and starts a hardware timer that triggers at the configured
     * frequency (default 1kHz). The timer callback performs time-critical
     * DC operations and optional PDO exchange.
     * 
     * @return true on success, false if DC not initialized or already running
     */
    virtual bool start();

    /**
     * @brief Start with an explicit PDO exchange callback
     * 
     * @param pdo_exchange_fn Called every cycle when PDO is enabled
     * @return true on success
     */
    virtual bool start(std::function<bool()> pdo_exchange_fn);

    /**
     * @brief Explicit initialization step that reads slave capabilities.
     *
     * This is intentionally separate from the constructor so that callers
     * may control when network I/O and DC probing occurs.
     *
     * @return true on success (DC-capable slaves found and configured)
     */
    virtual bool init();

    /**
     * @brief Stop the DC realtime loop
     * 
     * Stops the hardware timer and disables DC synchronization.
     * Can be restarted by calling start() again.
     */
    virtual void stop();

    /**
     * @brief Get current DC state
     * 
     * @return Current state machine state
     */
    virtual DCState getState() const { return state_.load(std::memory_order_acquire); }

    /**
     * @brief Get DC loop statistics
     * 
     * Returns a snapshot of current statistics including cycle count,
     * jitter measurements, and clock drift.
     * 
     * @return Copy of current statistics
     */
    virtual DCLoopStats getStats() const;

    /**
     * @brief Force a DC synchronization cycle
     * 
     * Triggers an immediate synchronization frame to all slaves.
     * Typically not needed as the timer handles this automatically.
     * 
     * Thread-safe.
     */
    virtual void forceSync();

    /**
     * @brief Enable or disable PDO exchange in the realtime loop
     * 
     * When enabled, PDO data is exchanged with slaves every cycle via
     * the PDO module. Must configure PDO mappings before enabling.
     * 
     * @param enable true to enable PDO exchange
     */
    virtual void setPDOEnabled(bool enable);

    /**
     * @brief Check if PDO exchange is enabled
     * 
     * @return true if PDO exchange runs in the realtime loop
     */
    virtual bool isPDOEnabled() const;

    /**
     * @brief Check if a specific slave supports DC
     * 
     * @param slave_index Slave index (0-based)
     * @return true if slave has DC capability, false otherwise
     */
    virtual bool isSlaveSupported(uint16_t slave_index) const;

    /**
     * @brief Get slave time offset from master
     * 
     * @param slave_index Slave index (0-based)
     * @return Offset in nanoseconds (positive = slave ahead, negative = behind)
     */
    virtual int64_t getSlaveOffset(uint16_t slave_index) const;

    /**
     * @brief Read back and log DC configuration for diagnostics
     * 
     * Reads SYNC0/SYNC1 cycle times and activation registers from slave.
     * Useful for verifying DC configuration was correctly applied.
     * 
     * @param slave_index Slave index (0-based)
     */
    virtual void readSyncConfig(uint16_t slave_index);

    /**
     * @brief Reconfigure DC SYNC signals on a slave
     * 
     * Updates SYNC0/SYNC1 cycle times, start time, and activation registers.
     * Call this after SM configuration when the slave is in PRE_OP or SAFE_OP.
     * 
     * @param slave_index Slave index (0-based)
     * @return true if configuration succeeded
     */
    virtual bool reconfigureSync(uint16_t slave_index);

    /**
     * @brief Get master reference time in nanoseconds
     * 
     * Returns the current time from the master's perspective via the transport.
     * 
     * @return Current time in nanoseconds since boot (or epoch)
     */
    virtual uint64_t getMasterTimeNs();

    /**
     * @brief Read a DC register from a slave using the scoped DCRegisters enum
     *
     * Convenience wrapper around the transport's readRegister that accepts
     * a DCRegisters value and verifies the slave index is in range.
     *
     * @param slave_index Zero-based slave index
     * @param reg          DCRegisters enum value to read
     * @param data         Destination buffer
     * @param size         Number of bytes to read
     * @param timeout_ms   Optional timeout in milliseconds (default 200)
     * @return true on success, false on invalid index or transport failure
     */
    virtual bool readRegister(uint16_t slave_index, DCRegisters reg, void* data, uint16_t size,
                      unsigned int timeout_ms = 200);

    /**
     * @brief Write a DC register on a slave using the scoped DCRegisters enum
     *
     * Convenience wrapper around the transport's writeRegister that accepts
     * a DCRegisters value and verifies the slave index is in range.
     */
    virtual bool writeRegister(uint16_t slave_index, DCRegisters reg, const void* data, uint16_t size,
                       unsigned int timeout_ms = 200);

    /**
     * @brief Send DC synchronization frame
     * 
     * Sends a sync datagram to the reference slave with current master time.
     * Public so it can be used as a callback by EtherCATRealtimeLoop.
     * 
     * @return true on success
     */
    virtual bool sendSyncFrame();

private:
    // ========================================================================
    // Internal state
    // ========================================================================

    DCConfig config_;
    std::atomic<DCState> state_{DCState::Disabled};
    
    uint16_t slave_count_;
    SlaveTimeInfo slaves_[kMaxDCSlaves];

    // Timing
    uint64_t master_reference_time_ns_{0};
    uint64_t dc_start_time_ns_{0};
    uint64_t next_sync_time_ns_{0};

    // Loop control
    std::atomic<bool> initialized_{false}; // Set true after explicit init() call

    // Statistics (DC-specific; loop stats come from realtime_loop_)
    DCLoopStats stats_{};
    mutable std::mutex stats_mutex_;

    // Transport (injected, caller-owned)
    IDCTransport& transport_;

    // Realtime loop (created on start(), destroyed on stop())
    std::unique_ptr<EtherCATRealtimeLoop> realtime_loop_;
    std::function<bool()> pdo_exchange_fn_;  // Stored for restart

    // ========================================================================
    // Internal methods
    // ========================================================================

    // Internal implementation for init()
    bool initialize();

    /**
     * @brief Read DC capabilities from a slave
     * 
     * @param slave_index Slave index (0-based)
     * @return true if slave supports DC
     */
    bool readSlaveCapabilities(uint16_t slave_index);

    /**
     * @brief Calculate propagation delay to a slave
     * 
     * @param slave_index Slave index (0-based)
     * @return true on success
     */
    bool calcPropagationDelay(uint16_t slave_index);

    /**
     * @brief Write system time offset to a slave
     * 
     * @param slave_index Slave index (0-based)
     * @param offset Offset in nanoseconds
     * @return true on success
     */
    bool writeSystemTimeOffset(uint16_t slave_index, int64_t offset);

    /**
     * @brief Configure SYNC0/SYNC1 signals on a slave
     * 
     * @param slave_index Slave index (0-based)
     * @return true on success
     */
    bool configureSyncSignals(uint16_t slave_index);

    /**
     * @brief Update SYNC0 start time to ensure it's in the future
     * 
     * @return true on success
     */
    bool updateSyncStartTime();
};

// ============================================================================
// NoDistributedClockConfigured — sentinel returned when DC is not configured
// ============================================================================

/**
 * @brief Sentinel DC instance returned by DCManager::get() when the
 * distributed clock is not initialized. All methods log a CRITICAL error
 * explaining that the DC must be configured first and return safe defaults.
 */
class NoDistributedClockConfigured final : public EtherCATDC {
public:
    NoDistributedClockConfigured();

    bool start() override;
    bool start(std::function<bool()> pdo_exchange_fn) override;
    bool init() override;
    void stop() override;

    DCState getState() const override;
    DCLoopStats getStats() const override;

    void forceSync() override;
    void setPDOEnabled(bool enable) override;
    bool isPDOEnabled() const override;

    bool isSlaveSupported(uint16_t slave_index) const override;
    int64_t getSlaveOffset(uint16_t slave_index) const override;

    void readSyncConfig(uint16_t slave_index) override;
    bool reconfigureSync(uint16_t slave_index) override;

    uint64_t getMasterTimeNs() override;

    bool readRegister(uint16_t slave_index, DCRegisters reg, void* data, uint16_t size,
                      unsigned int timeout_ms = 200) override;
    bool writeRegister(uint16_t slave_index, DCRegisters reg, const void* data, uint16_t size,
                       unsigned int timeout_ms = 200) override;

    bool sendSyncFrame() override;

private:
    void logCritical(const char* method) const;
};

} // namespace EtherCAT
