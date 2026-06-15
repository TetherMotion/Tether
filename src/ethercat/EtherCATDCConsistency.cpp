/**
 * @file EtherCATDCConsistency.cpp
 * @brief Implementation of DC Consistency Checks
 */

#include "EtherCATDCConsistency.hpp"
#include "EtherCATDC.hpp"
#include "ethercat/raw/internal.hpp"
#include "tether/platform/EspCompat.hpp"

#ifdef ESP_PLATFORM
#include "esp_eth.h"
#include "ethernet_init.h"
#endif

#include <cstring>
#include <cinttypes>

namespace EtherCAT {
namespace DC {

static const char* TAG = "DCConsistency";

// ============================================================================
// Master Time Implementation
// ============================================================================

uint64_t dc_get_master_time_with_epoch() {
#ifdef ESP_PLATFORM
    // Get ESP32 uptime in microseconds, convert to nanoseconds
    uint64_t uptime_ns = static_cast<uint64_t>(esp_timer_get_time()) * 1000ULL;
    return kMasterEpochNs + uptime_ns;
#else
    // Linux host: use system time
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t uptime_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
                          static_cast<uint64_t>(ts.tv_nsec);
    return kMasterEpochNs + uptime_ns;
#endif
}

size_t dc_format_time(uint64_t time_ns, char* buffer, size_t buffer_size) {
    if (buffer == nullptr || buffer_size == 0) {
        return 0;
    }

    // Calculate seconds since epoch
    uint64_t total_seconds = time_ns / 1000000000ULL;
    uint32_t nanos = static_cast<uint32_t>(time_ns % 1000000000ULL);

    // For times relative to our 2026-01-01 epoch:
    // Calculate offset from epoch
    if (time_ns >= kMasterEpochNs) {
        uint64_t offset_ns = time_ns - kMasterEpochNs;
        uint64_t offset_seconds = offset_ns / 1000000000ULL;
        uint32_t offset_nanos = static_cast<uint32_t>(offset_ns % 1000000000ULL);

        uint64_t hours = offset_seconds / 3600;
        uint32_t mins = (offset_seconds % 3600) / 60;
        uint32_t secs = offset_seconds % 60;

        return snprintf(buffer, buffer_size,
                        "2026-01-01 + %" PRIu64 "h%02" PRIu32 "m%02" PRIu32 ".%09" PRIu32 "s",
                        hours, mins, secs, offset_nanos);
    } else {
        // Raw time display
        return snprintf(buffer, buffer_size,
                        "%" PRIu64 ".%09" PRIu32 " s",
                        total_seconds, nanos);
    }
}

// ============================================================================
// DC State Reading
// ============================================================================

#ifdef ESP_PLATFORM
bool dc_read_slave_state(uint16_t slave_index, SlaveDCState& state) {
    std::memset(&state, 0, sizeof(state));

    void* eth = EthernetGetHandle();
    if (eth == nullptr) {
        return false;
    }

    uint8_t mac[6];
    if (esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, mac) != ESP_OK) {
        return false;
    }

    const uint16_t adp = static_cast<uint16_t>(0u - slave_index);
    const unsigned int timeout_ms = 50;

    // Read DC System Time (0x0910, 8 bytes)
    uint8_t sys_time[8];
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCSysTime), sys_time, 8, timeout_ms)) {
        state.system_time = 0;
        for (int i = 7; i >= 0; i--) {
            state.system_time = (state.system_time << 8) | sys_time[i];
        }
        state.dc_supported = true;
    }

    // Read System Time Offset (0x0920, 8 bytes)
    uint8_t offset[8];
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCSysOffset), offset, 8, timeout_ms)) {
        state.system_time_offset = 0;
        for (int i = 7; i >= 0; i--) {
            state.system_time_offset = (state.system_time_offset << 8) | offset[i];
        }
    }

    // Read System Time Difference (0x092C, 4 bytes)
    uint32_t diff_le = 0;
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCSysDiff), &diff_le, 4, timeout_ms)) {
        state.system_time_diff = Raw::le32_to_host(diff_le);
    }

    // Read Speed Counter (0x0930, 2 bytes)
    uint16_t speed_le = 0;
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCSpeedCnt), &speed_le, 2, timeout_ms)) {
        state.speed_counter = Raw::le16_to_host(speed_le);
    }

    // Read Time Loop Filter (0x0934, 2 bytes)
    uint16_t filter_le = 0;
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCTimeFilter), &filter_le, 2, timeout_ms)) {
        state.time_loop_filter = Raw::le16_to_host(filter_le);
    }

    // Read Receive Times (0x0900-0x090F, 16 bytes)
    uint32_t rt[4] = {0};
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCRecvTimes), rt, 16, timeout_ms)) {
        state.receive_time_port0 = Raw::le32_to_host(rt[0]);
        state.receive_time_port1 = Raw::le32_to_host(rt[1]);
        state.receive_time_port2 = Raw::le32_to_host(rt[2]);
        state.receive_time_port3 = Raw::le32_to_host(rt[3]);
    }

    // Read Cyclic Unit Control (0x0980, 1 byte)
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCCuc), &state.cyclic_unit_control, 1, timeout)) {
        // Success
    }

    // Read Sync Activation (0x0981, 1 byte)
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCSyncAct), &state.sync_activation, 1, timeout)) {
        state.dc_active = (state.sync_activation & 0x01) != 0;
        state.sync0_active = (state.sync_activation & 0x02) != 0;
        state.sync1_active = (state.sync_activation & 0x04) != 0;
    }

    // Read SYNC0 Start Time (0x0990, 8 bytes)
    uint8_t sync0_start[8];
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCStart0), sync0_start, 8, timeout)) {
        state.sync0_start_time = 0;
        for (int i = 7; i >= 0; i--) {
            state.sync0_start_time = (state.sync0_start_time << 8) | sync0_start[i];
        }
    }

    // Read SYNC0 Cycle Time (0x09A0, 4 bytes)
    uint32_t cycle0_le = 0;
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCCycle0), &cycle0_le, 4, timeout)) {
        state.sync0_cycle_time = Raw::le32_to_host(cycle0_le);
    }

    // Read SYNC1 Cycle Time (0x09A4, 4 bytes)
    uint32_t cycle1_le = 0;
    if (Raw::ec_aprd(eth, mac, adp, toUInt16(DCRegisters::DCCycle1), &cycle1_le, 4, timeout)) {
        state.sync1_cycle_time = Raw::le32_to_host(cycle1_le);
    }

    // Calculate propagation delay
    state.propagation_delay = dc_calc_propagation_delay(
        state.receive_time_port0, state.receive_time_port1);

    return state.dc_supported;
}
#else
// Linux host stub — no direct EtherCAT hardware access available
bool dc_read_slave_state(uint16_t slave_index, SlaveDCState& state) {
    std::memset(&state, 0, sizeof(state));
    state.dc_supported = true;
    state.system_time = dc_get_master_time_with_epoch();
    state.sync0_cycle_time = 1000000;  // 1ms
    return true;
}
#endif

// ============================================================================
// Consistency Checks
// ============================================================================

bool dc_run_consistency_checks(uint16_t slave_index, DCConsistencyReport& report) {
    std::memset(&report, 0, sizeof(report));

    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        report.checks[0] = {false, "DC Read", "Failed to read DC state", 0, 0};
        report.check_count = 1;
        report.failed_count = 1;
        return false;
    }

    size_t idx = 0;
    (void)dc_get_master_time_with_epoch(); // Ensure master time is computed

    // Check 1: DC Supported
    report.checks[idx] = {
        state.dc_supported,
        "DC Supported",
        state.dc_supported ? "OK" : "Slave does not support DC",
        1, state.dc_supported ? 1 : 0
    };
    if (state.dc_supported) report.passed_count++; else report.failed_count++;
    idx++;

    // Check 2: DC Active
    report.checks[idx] = {
        state.dc_active,
        "DC Active",
        state.dc_active ? "OK" : "DC not activated (0x0981 bit 0)",
        1, state.dc_active ? 1 : 0
    };
    if (state.dc_active) report.passed_count++; else report.failed_count++;
    idx++;

    // Check 3: SYNC0 Active
    report.checks[idx] = {
        state.sync0_active,
        "SYNC0 Active",
        state.sync0_active ? "OK" : "SYNC0 not enabled (0x0981 bit 1)",
        1, state.sync0_active ? 1 : 0
    };
    if (state.sync0_active) report.passed_count++; else report.failed_count++;
    idx++;

    // Check 4: SYNC0 Start Time in Future
    bool sync0_valid = state.sync0_start_time > state.system_time;
    report.checks[idx] = {
        sync0_valid,
        "SYNC0 Start Time",
        sync0_valid ? "OK (in future)" : "SYNC0 start time is in the past!",
        static_cast<int64_t>(state.system_time),
        static_cast<int64_t>(state.sync0_start_time)
    };
    if (sync0_valid) report.passed_count++; else report.failed_count++;
    idx++;

    // Check 5: SYNC0 Cycle Time Non-Zero
    bool cycle_valid = state.sync0_cycle_time > 0;
    report.checks[idx] = {
        cycle_valid,
        "SYNC0 Cycle Time",
        cycle_valid ? "OK" : "SYNC0 cycle time is zero!",
        1000000, // Expected ~1ms
        static_cast<int64_t>(state.sync0_cycle_time)
    };
    if (cycle_valid) report.passed_count++; else report.failed_count++;
    idx++;

    // Check 6: Propagation Delay Reasonable (< 10ms)
    bool delay_valid = state.propagation_delay < 10000000;
    report.checks[idx] = {
        delay_valid,
        "Propagation Delay",
        delay_valid ? "OK" : "Propagation delay too high!",
        10000000,
        static_cast<int64_t>(state.propagation_delay)
    };
    if (delay_valid) report.passed_count++; else report.failed_count++;
    idx++;

    // Check 7: System Time Difference Small (< 1ms)
    bool diff_valid = state.system_time_diff < 1000000 ||
                       static_cast<int32_t>(state.system_time_diff) > -1000000;
    report.checks[idx] = {
        diff_valid,
        "System Time Diff",
        diff_valid ? "OK" : "Large time difference detected!",
        0,
        static_cast<int64_t>(static_cast<int32_t>(state.system_time_diff))
    };
    if (diff_valid) report.passed_count++; else report.failed_count++;
    idx++;

    report.check_count = idx;
    return report.all_passed();
}

bool dc_check_sync0_start_time(uint16_t slave_index) {
    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        return false;
    }
    return state.sync0_start_time > state.system_time;
}

bool dc_check_propagation_delay(uint16_t slave_index, uint32_t max_delay_ns) {
    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        return false;
    }
    return state.propagation_delay < max_delay_ns;
}

bool dc_check_system_time_offset(uint16_t slave_index) {
    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        return false;
    }
    // Offset should be non-zero after DC init
    return state.system_time_offset != 0;
}

bool dc_check_sync_cycle_times(uint16_t slave_index) {
    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        return false;
    }

    // DCContext was removed — cannot validate cycle times without config.
    TETHER_LOGW(TAG, "dc_check_sync_cycle_times: DCContext removed, skipping validation");
    return false;
}

// ============================================================================
// Diagnostic Logging
// ============================================================================

void dc_log_sync_activation(uint8_t sync_activation) {
    TETHER_LOGI(TAG, "SYNC Activation (0x0981): 0x%02X\n  Bit 0 - Cyclic Operation:    %s\n  Bit 1 - SYNC0 Generation:    %s\n  Bit 2 - SYNC1 Generation:    %s\n  Bit 3 - Auto Activation:     %s\n  Bit 4 - Ext SYNC0 to SYNC0:  %s\n  Bit 5 - Ext SYNC0 to LATCH0: %s\n  Bit 6 - Ext SYNC1 to SYNC1:  %s\n  Bit 7 - Ext SYNC1 to LATCH1: %s",
               sync_activation,
               (sync_activation & 0x01) ? "ENABLED" : "disabled",
               (sync_activation & 0x02) ? "ENABLED" : "disabled",
               (sync_activation & 0x04) ? "ENABLED" : "disabled",
               (sync_activation & 0x08) ? "yes" : "no",
               (sync_activation & 0x10) ? "yes" : "no",
               (sync_activation & 0x20) ? "yes" : "no",
               (sync_activation & 0x40) ? "yes" : "no",
               (sync_activation & 0x80) ? "yes" : "no");
} 

void dc_log_cyclic_unit_control(uint8_t cuc) {
    TETHER_LOGI(TAG, "Cyclic Unit Control (0x0980): 0x%02X\n  Bit 0 - SYNC Out Unit:       %s\n  Bit 1-3 - Reserved\n  Bit 4 - Latch In Unit:       %s",
               cuc,
               (cuc & 0x01) ? "ENABLED" : "disabled",
               (cuc & 0x10) ? "ENABLED" : "disabled");
} 

void dc_log_slave_state(uint16_t slave_index) {
    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        TETHER_LOGE(TAG, "Failed to read DC state for slave %u", slave_index);
        return;
    }

    char time_str[64];

    TETHER_LOGI(TAG, "╔═══════════════════════════════════════════════════════════════╗\n║         DC STATE - Slave %u                                    ║\n╚═══════════════════════════════════════════════════════════════╝\nDC Support: %s",
               slave_index, state.dc_supported ? "YES" : "NO");

    dc_format_time(state.system_time, time_str, sizeof(time_str));
    TETHER_LOGI(TAG, "System Time (0x0910):     %" PRIu64 " ns (%s)\nSystem Time Offset (0x0920): %" PRIu64 " ns\nSystem Time Diff (0x092C):   %d ns (signed: %d)\nSpeed Counter (0x0930):   %u\nTime Loop Filter (0x0934): %u\n\nReceive Times (for propagation delay calculation):\n  Port 0: %u ns\n  Port 1: %u ns\n  Port 2: %u ns\n  Port 3: %u ns\n  Calculated propagation delay: %u ns",
               state.system_time, time_str,
               state.system_time_offset,
               state.system_time_diff, static_cast<int32_t>(state.system_time_diff),
               state.speed_counter,
               state.time_loop_filter,
               state.receive_time_port0, state.receive_time_port1,
               state.receive_time_port2, state.receive_time_port3,
               state.propagation_delay);
    dc_log_cyclic_unit_control(state.cyclic_unit_control);
    dc_log_sync_activation(state.sync_activation);

    dc_format_time(state.sync0_start_time, time_str, sizeof(time_str));
    TETHER_LOGI(TAG, "\nSYNC0 Start Time (0x0990): %" PRIu64 " ns (%s)\nSYNC0 Cycle Time (0x09A0): %u ns (%.3f ms)\nSYNC1 Cycle Time (0x09A4): %u ns (%.3f ms)",
             state.sync0_start_time, time_str,
             state.sync0_cycle_time, state.sync0_cycle_time / 1000000.0f,
             state.sync1_cycle_time, state.sync1_cycle_time / 1000000.0f);

    // Check if SYNC0 start is in future
    if (state.sync0_start_time > state.system_time) {
        int64_t delta = static_cast<int64_t>(state.sync0_start_time - state.system_time);
        TETHER_LOGI(TAG, "SYNC0 Start: %" PRId64 " ns IN FUTURE (OK)", delta);
    } else {
        int64_t delta = static_cast<int64_t>(state.system_time - state.sync0_start_time);
        TETHER_LOGE(TAG, "SYNC0 Start: %" PRId64 " ns IN PAST (PROBLEM!)", delta);
    }

    TETHER_LOGI(TAG, "═══════════════════════════════════════════════════════════════");
}

void dc_log_consistency_report(const DCConsistencyReport& report) {
    TETHER_LOGI(TAG, "DC Consistency Report: %zu/%zu checks passed",
             report.passed_count, report.check_count);

    for (size_t i = 0; i < report.check_count; i++) {
        const auto& c = report.checks[i];
        if (c.passed) {
            TETHER_LOGI(TAG, "  ✓ %s: %s", c.check_name, c.description);
        } else {
            TETHER_LOGE(TAG, "  ✗ %s: %s (expected=%" PRId64 ", actual=%" PRId64 ")",
                     c.check_name, c.description, c.expected_value, c.actual_value);
        }
    }
}

void dc_log_sync0_timing(uint16_t slave_index, uint64_t current_master_time) {
    SlaveDCState state;
    if (!dc_read_slave_state(slave_index, state)) {
        return;
    }

    TETHER_LOGI(TAG, "SYNC0 Timing Analysis (Slave %u):\n  Master time:       %" PRIu64 " ns\n  Slave system time: %" PRIu64 " ns\n  SYNC0 start time:  %" PRIu64 " ns\n  SYNC0 cycle:       %u ns",
               slave_index, current_master_time, state.system_time, state.sync0_start_time, state.sync0_cycle_time);

    if (state.sync0_start_time > state.system_time) {
        uint64_t ns_until_start = state.sync0_start_time - state.system_time;
        TETHER_LOGI(TAG, "  Time until SYNC0:  %" PRIu64 " ns (%.3f ms)",
                 ns_until_start, ns_until_start / 1000000.0f);
    } else {
        uint64_t ns_since_start = state.system_time - state.sync0_start_time;
        uint64_t cycles_elapsed = ns_since_start / state.sync0_cycle_time;
        TETHER_LOGI(TAG, "  SYNC0 started:     %" PRIu64 " ns ago (%" PRIu64 " cycles)",
                 ns_since_start, cycles_elapsed);
    }
}

void dc_full_diagnostic(uint16_t slave_index) {
    if (slave_index == 0xFFFF) {
        // Cannot iterate all slaves without external slave count.
        TETHER_LOGW(TAG, "dc_full_diagnostic: cannot iterate all slaves. "
                         "Use dc_full_diagnostic(slave_index) for individual slaves.");
        return;
    }

    TETHER_LOGI(TAG, "\n╔═══════════════════════════════════════════════════════════════╗\n║           FULL DC DIAGNOSTIC - Slave %u                       ║\n╚═══════════════════════════════════════════════════════════════╝",
               slave_index);

    // Log current state
    dc_log_slave_state(slave_index);

    // Run and log consistency checks
    TETHER_LOGI(TAG, "");
    DCConsistencyReport report;
    dc_run_consistency_checks(slave_index, report);
    dc_log_consistency_report(report);

    // Timing analysis
    TETHER_LOGI(TAG, "");
    dc_log_sync0_timing(slave_index, dc_get_master_time_with_epoch());

    TETHER_LOGI(TAG, "═══════════════════════════════════════════════════════════════");
}

// ============================================================================
// Utility Functions
// ============================================================================

uint32_t dc_calc_propagation_delay(uint32_t rt_port0, uint32_t rt_port1) {
    // Simple calculation: delay from port 0 to port 1
    // More sophisticated calculations consider all ports
    if (rt_port1 > rt_port0) {
        return rt_port1 - rt_port0;
    }
    return rt_port0;  // Single port, use as-is
}

bool dc_is_time_in_future(uint64_t time_ns, uint64_t reference_ns, uint64_t margin_ns) {
    if (time_ns > reference_ns) {
        return true;
    }
    // Allow margin
    return (reference_ns - time_ns) < margin_ns;
}

int64_t dc_time_diff(uint64_t time1, uint64_t time2) {
    // Proper signed difference
    if (time1 >= time2) {
        uint64_t diff = time1 - time2;
        if (diff > static_cast<uint64_t>(INT64_MAX)) {
            return INT64_MAX;
        }
        return static_cast<int64_t>(diff);
    } else {
        uint64_t diff = time2 - time1;
        if (diff > static_cast<uint64_t>(INT64_MAX)) {
            return INT64_MIN;
        }
        return -static_cast<int64_t>(diff);
    }
}

} // namespace DC
} // namespace EtherCAT
