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
#include <vector>
#include "tether/ethercat/EtherCATConfig.hpp"

namespace EtherCAT {

constexpr size_t kMaxDCSlaves = ECAT_DC_MAX_SLAVES;

struct SlaveTimeInfo {
    uint64_t system_time_ns;
    uint64_t receive_time_ns;
    int64_t  offset_to_master_ns;
    uint32_t propagation_delay_ns;
    uint64_t sync0_start_time_ns;
    bool     dc_supported;
    bool     dc_active;
};

/**
 * @brief Latched per-port receive times of one slave (registers 0x0900-0x090F)
 *
 * Filled by a single broadcast latch frame so all values refer to the
 * same EtherCAT frame. The port with the smallest non-zero receive time
 * is the entry port (facing the master); every other port that saw the
 * frame is a downstream port leading to a child subtree.
 */
struct DCPortInfo {
    uint32_t recv_time_ns[4] = {};  ///< Latched receive time per port; 0 = port saw no frame
    uint8_t  active_port_mask = 0;  ///< Bit p set => port p latched a non-zero receive time
    int8_t   entry_port = -1;       ///< Upstream port (smallest recv time); -1 = unknown
};

/**
 * @brief Position of one slave in the measured DC topology
 *
 * The EtherCAT position index (auto-increment order) defines the order in
 * which frames reach slaves. The parent link and parent port describe
 * which earlier slave forwards frames to this slave and through which of
 * its ports — this is reconstructed from the latched receive times, so
 * branched (multi-port) topologies are handled, not just linear chains.
 */
struct DCLinkInfo {
    int16_t  parent = -1;          ///< Parent slave index; -1 = directly at the master
    int8_t   parent_port = -1;     ///< Downstream port on the parent leading to this slave
    uint32_t link_delay_ns = 0;    ///< Measured one-way delay parent -> this slave
    uint32_t subtree_time_ns = 0;  ///< Time the latch frame spent inside this slave's subtree
    uint32_t total_delay_ns = 0;   ///< Accumulated delay from the reference clock (written to 0x0928)
    bool     timing_valid = false; ///< Receive-time registers could be read and were non-zero
};

/**
 * @brief Measured DC topology of the whole segment
 *
 * Filled during initialize() by the propagation-delay measurement.
 * Exposed read-only via EtherCATDC::topology() for diagnostics and tests.
 */
struct DCTopology {
    DCPortInfo ports[kMaxDCSlaves];
    DCLinkInfo links[kMaxDCSlaves];
    int16_t    reference = -1;     ///< Index of the DC reference clock slave; -1 = none
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
    /// Slave indices that should NOT receive SYNC0/SYNC1 activation.
    /// All other DC-capable slaves get SYNC signals as normal.
    /// Empty by default (all slaves enabled).
    std::vector<uint16_t> sync_disabled_slaves;

    static DCConfig defaults() {
        return DCConfig{
            .cycle_period_us = 1000,
            .sync_interval_cycles = 10,
            .sync0_cycle_time_ns = 1000000,
            .sync1_cycle_time_ns = 0,
            .sync0_shift_ns = 0,
            .enable_sync0 = true,
            .enable_sync1 = false,
            .sync_disabled_slaves = {}
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
