/**
 * @file DCTypes.hpp
 * @brief Distributed Clock (DC) type definitions
 *
 * Extracted from DCClass.hpp for modularity. Contains:
 * - SlaveTimeInfo, DCLoopStats, DCConfig structs
 * - DCState enum and dc_state_name()
 * - DCRegisters enum and DCSyncActBits
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace EtherCAT {

constexpr size_t kMaxDCSlaves = 16;

struct SlaveTimeInfo {
    uint64_t system_time_ns;
    uint64_t receive_time_ns;
    int64_t  offset_to_master_ns;
    uint32_t propagation_delay_ns;
    uint64_t sync0_start_time_ns;
    bool     dc_supported;
    bool     dc_active;
};

struct DCLoopStats {
    uint64_t cycle_count;
    uint64_t sync_count;
    uint64_t pdo_error_count;
    uint32_t max_jitter_us;
    uint32_t avg_jitter_us;
    int64_t  last_drift_ns;
    uint64_t last_master_time_ns;
};

struct DCConfig {
    uint32_t cycle_period_us;
    uint32_t sync_interval_cycles;
    uint32_t sync0_cycle_time_ns;
    uint32_t sync1_cycle_time_ns;
    int32_t  sync0_shift_ns;
    bool     enable_sync0;
    bool     enable_sync1;

    static DCConfig defaults() {
        return DCConfig{
            .cycle_period_us = 1000,
            .sync_interval_cycles = 10,
            .sync0_cycle_time_ns = 1000000,
            .sync1_cycle_time_ns = 0,
            .sync0_shift_ns = 0,
            .enable_sync0 = true,
            .enable_sync1 = false
        };
    }
};

enum class DCState : uint8_t {
    Disabled = 0,
    Initializing,
    PropagationCalc,
    DriftCompensation,
    Running,
    Error
};

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

enum class DCRegisters : uint16_t {
    DCSysTime      = 0x0910,
    DCRecvTimes    = 0x0900,
    DCSysTxTime    = 0x0928,
    DCSysOffset    = 0x0920,
    DCSysDiff      = 0x092C,
    DCStartOfFrame = 0x0918,
    DCSpeedCnt     = 0x0930,
    DCTimeFilter   = 0x0934,
    DCCuc          = 0x0980,
    DCSyncAct      = 0x0981,
    DCSyncLatch    = 0x098E,
    DCStart0       = 0x0990,
    DCCycle0       = 0x09A0,
    DCCycle1       = 0x09A4,
};

inline constexpr uint16_t toUInt16(DCRegisters reg) noexcept {
    return static_cast<uint16_t>(reg);
}

enum DCSyncActBits : uint8_t {
    DC_SYNCACT_ENA        = 0x01,
    DC_SYNCACT_SYNC0_ENA  = 0x02,
    DC_SYNCACT_SYNC1_ENA  = 0x04,
    DC_SYNCACT_AUTO_ACT   = 0x08,
};

} // namespace EtherCAT
