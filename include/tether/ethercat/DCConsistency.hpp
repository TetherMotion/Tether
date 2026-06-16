/**
 * @file DCConsistency.hpp
 * @brief DC Synchronization Consistency Checks and Diagnostics
 *
 * @details
 * This module provides comprehensive consistency checking for EtherCAT
 * Distributed Clock (DC) synchronization. It validates:
 *
 * - Internal DC configuration consistency
 * - Slave DC register values vs expectations
 * - Master time base validity
 * - Propagation delay calculations
 * - SYNC0/SYNC1 timing parameters
 *
 * ## Master Time Algorithm
 *
 * The master time is based on:
 *   2026-01-01 00:00:00 UTC + [time since ESP32 startup]
 *
 * This provides:
 * - A known reference epoch for debugging
 * - Monotonically increasing time
 * - Deterministic behavior across restarts
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace EtherCAT {
namespace DC {

// ============================================================================
// Master Time Base
// ============================================================================

/**
 * @brief Epoch for master time: 2026-01-01 00:00:00 UTC
 *
 * In nanoseconds since Unix epoch (1970-01-01 00:00:00 UTC)
 * = 1767225600 seconds * 1,000,000,000
 */
constexpr uint64_t kMasterEpochNs = 1767225600ULL * 1000000000ULL;

/**
 * @brief Get current master time in nanoseconds
 *
 * Returns time as: 2026-01-01 00:00:00 + ESP32 uptime
 *
 * @return Current master time in nanoseconds
 */
uint64_t dc_get_master_time_with_epoch();

/**
 * @brief Convert master time to human-readable string
 *
 * @param time_ns Time in nanoseconds (relative to kMasterEpochNs)
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return Number of characters written
 */
size_t dc_format_time(uint64_t time_ns, char* buffer, size_t buffer_size);

// ============================================================================
// DC Slave State Structure
// ============================================================================

/**
 * @brief Complete DC state read from a slave
 */
struct SlaveDCState {
    // System time registers
    uint64_t system_time;          ///< Current DC system time (0x0910)
    uint64_t system_time_offset;   ///< System time offset (0x0920)
    uint32_t system_time_diff;     ///< System time difference (0x092C)
    uint32_t speed_counter;        ///< Speed counter (0x0930)
    uint16_t time_loop_filter;     ///< Time loop filter (0x0934)

    // Receive times (for propagation delay)
    uint32_t receive_time_port0;   ///< Port 0 receive time (0x0900)
    uint32_t receive_time_port1;   ///< Port 1 receive time (0x0904)
    uint32_t receive_time_port2;   ///< Port 2 receive time (0x0908)
    uint32_t receive_time_port3;   ///< Port 3 receive time (0x090C)

    // SYNC configuration
    uint8_t cyclic_unit_control;   ///< Cyclic unit control (0x0980)
    uint8_t sync_activation;       ///< Sync activation (0x0981)
    uint64_t sync0_start_time;     ///< SYNC0 start time (0x0990)
    uint32_t sync0_cycle_time;     ///< SYNC0 cycle time (0x09A0)
    uint32_t sync1_cycle_time;     ///< SYNC1 cycle time (0x09A4)

    // Status
    bool dc_supported;             ///< Slave supports DC
    bool dc_active;                ///< DC is currently active
    bool sync0_active;             ///< SYNC0 is enabled
    bool sync1_active;             ///< SYNC1 is enabled

    // Calculated values
    int64_t offset_to_master;      ///< Offset relative to master
    uint32_t propagation_delay;    ///< Propagation delay from master
};

// ============================================================================
// Consistency Check Results
// ============================================================================

/**
 * @brief Result of a consistency check
 */
struct ConsistencyCheckResult {
    bool passed;                   ///< Check passed
    const char* check_name;        ///< Name of the check
    const char* description;       ///< Description of failure (if any)
    int64_t expected_value;        ///< Expected value (for comparisons)
    int64_t actual_value;          ///< Actual value found
};

/**
 * @brief Collection of all consistency check results
 */
struct DCConsistencyReport {
    static constexpr size_t kMaxChecks = 32;

    ConsistencyCheckResult checks[kMaxChecks];
    size_t check_count;
    size_t passed_count;
    size_t failed_count;

    bool all_passed() const { return failed_count == 0; }
};

// ============================================================================
// Consistency Check API
// ============================================================================

/**
 * @brief Read complete DC state from a slave
 *
 * @param slave_index Slave to read
 * @param state Output state structure
 * @return true if read succeeded
 */
bool dc_read_slave_state(uint16_t slave_index, SlaveDCState& state);

/**
 * @brief Run all consistency checks for a slave
 *
 * @param slave_index Slave to check
 * @param report Output report structure
 * @return true if all checks passed
 */
bool dc_run_consistency_checks(uint16_t slave_index, DCConsistencyReport& report);

/**
 * @brief Check if SYNC0 start time is valid (in the future)
 *
 * @param slave_index Slave to check
 * @return true if SYNC0 start time is valid
 */
bool dc_check_sync0_start_time(uint16_t slave_index);

/**
 * @brief Check if propagation delay is reasonable
 *
 * @param slave_index Slave to check
 * @param max_delay_ns Maximum acceptable delay (default 10ms)
 * @return true if delay is within bounds
 */
bool dc_check_propagation_delay(uint16_t slave_index, uint32_t max_delay_ns = 10000000);

/**
 * @brief Check if system time offset is reasonable
 *
 * @param slave_index Slave to check
 * @return true if offset is reasonable
 */
bool dc_check_system_time_offset(uint16_t slave_index);

/**
 * @brief Check if SYNC cycle times match configuration
 *
 * @param slave_index Slave to check
 * @return true if cycle times match
 */
bool dc_check_sync_cycle_times(uint16_t slave_index);

// ============================================================================
// Diagnostic Logging
// ============================================================================

/**
 * @brief Log complete DC state for a slave (interpreted, not just hex)
 *
 * @param slave_index Slave to diagnose
 */
void dc_log_slave_state(uint16_t slave_index);

/**
 * @brief Log DC consistency report
 *
 * @param report Report to log
 */
void dc_log_consistency_report(const DCConsistencyReport& report);

/**
 * @brief Log DC sync activation register interpretation
 *
 * @param sync_activation Value of sync activation register (0x0981)
 */
void dc_log_sync_activation(uint8_t sync_activation);

/**
 * @brief Log DC cyclic unit control register interpretation
 *
 * @param cuc Value of cyclic unit control register (0x0980)
 */
void dc_log_cyclic_unit_control(uint8_t cuc);

/**
 * @brief Log SYNC0 timing analysis
 *
 * @param slave_index Slave to analyze
 * @param current_master_time Current master time
 */
void dc_log_sync0_timing(uint16_t slave_index, uint64_t current_master_time);

/**
 * @brief Full DC diagnostic dump for debugging
 *
 * Logs everything: registers, calculated values, consistency checks
 *
 * @param slave_index Slave to diagnose (0xFFFF for all slaves)
 */
void dc_full_diagnostic(uint16_t slave_index);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert receive times to propagation delay
 *
 * @param rt_port0 Port 0 receive time
 * @param rt_port1 Port 1 receive time (0 if not connected)
 * @return Calculated propagation delay in nanoseconds
 */
uint32_t dc_calc_propagation_delay(uint32_t rt_port0, uint32_t rt_port1);

/**
 * @brief Check if a time value is in the future
 *
 * @param time_ns Time to check
 * @param reference_ns Reference time (current time)
 * @param margin_ns Allowed margin (default 0)
 * @return true if time_ns > reference_ns - margin_ns
 */
bool dc_is_time_in_future(uint64_t time_ns, uint64_t reference_ns, uint64_t margin_ns = 0);

/**
 * @brief Calculate time difference with proper wrapping
 *
 * @param time1 First time value
 * @param time2 Second time value
 * @return time1 - time2 (signed)
 */
int64_t dc_time_diff(uint64_t time1, uint64_t time2);

} // namespace DC
} // namespace EtherCAT
