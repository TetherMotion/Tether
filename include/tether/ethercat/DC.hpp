#pragma once

/**
 * @file DC.hpp
 * @brief EtherCAT Distributed Clock (DC) synchronization data structures and API
 *
 * This module provides:
 * - Data structures for DC time synchronization
 * - Hardware timer-based realtime loop at 1kHz
 * - Platform-abstracted master time source
 * - Periodic DC synchronization with configurable frequency
 */

#include <cstdint>
#include <cstddef>
#include <vector>
#include <atomic>
#include <memory>

#include "tether/platform/EspCompat.hpp"
#include "tether/platform/IPlatformTimer.hpp"
#include "tether/ethercat/TetherConfig.hpp"

namespace EtherCAT {

// Forward declarations
struct NetworkInterface;
class Master;

namespace DC {

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
    /// Slave indices that should NOT receive SYNC0/SYNC1 activation.
    /// All other DC-capable slaves get SYNC signals as normal.
    /// Empty by default (all slaves enabled).
    std::vector<uint16_t> sync_disabled_slaves;

    static DCConfig defaults() {
        return DCConfig{
            .cycle_period_us = 1000,        // 1kHz loop
            .sync_interval_cycles = 10,     // Sync every 10ms
            .sync0_cycle_time_ns = 1000000, // 1ms SYNC0 cycle
            .sync1_cycle_time_ns = 0,       // SYNC1 disabled
            .sync0_shift_ns = 0,
            .enable_sync0 = true,
            .enable_sync1 = false,
            .sync_disabled_slaves = {}
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

// Maximum number of slaves for DC synchronization
constexpr size_t kMaxDCSlaves = ECAT_DC_MAX_SLAVES;

// ============================================================================
// Forward declaration of the class-based DC implementation
// ============================================================================

} // namespace DC
} // namespace EtherCAT

// Include the class-based API header so that the DC:: free functions can
// reference EtherCATDC without a separate forward declaration.
#include "tether/ethercat/DCClass.hpp"

namespace EtherCAT {
namespace DC {

// ============================================================================
// Platform time-source functions (defined in dc_time_source.cpp, weak symbols)
// ============================================================================

/**
 * @brief Get current master time in nanoseconds
 *
 * Uses platform-specific high-resolution timer.
 * Can be overridden by linking a stronger symbol.
 *
 * @return Monotonic time in nanoseconds
 */
extern "C" uint64_t ecdc_get_master_time_ns();

/**
 * @brief Initialize platform-specific time source
 *
 * @return true on success, false on failure
 */
extern "C" bool ecdc_init_time_source();

/**
 * @brief Deinitialize platform-specific time source
 *
 * Called during DC shutdown to clean up time source resources.
 * Default implementation is a no-op.
 */
extern "C" void ecdc_deinit_time_source();

} // namespace DC
} // namespace EtherCAT

// Include DCManager implementation after DC:: types are defined
#include "tether/ethercat/DCManager.hpp"
