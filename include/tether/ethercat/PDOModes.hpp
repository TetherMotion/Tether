/**
 * @file PDOModes.hpp
 * @brief PDO interaction mode types and configurations
 *
 * Defines the three user-facing PDO interaction modes:
 *  - Direct (split send/receive): user calls sendAll()/receiveAll()
 *  - Queue-based async: internal RT loop, user interacts via queues
 *  - Callback-based: per-entry callbacks fire during sendAll()/receiveAll()
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace EtherCAT {

// ============================================================================
// Mode enum
// ============================================================================

enum class PDOMode : uint8_t {
    Direct = 0,     ///< Default: user calls sendAll()/receiveAll() or exchangeAll()
    Callback = 1,   ///< Per-entry callbacks fire during send/receive
    Queue = 2,      ///< Internal RT loop with lock-free queues (audio driver model)
};

// ============================================================================
// Callback mode types (Mode 3)
// ============================================================================

/// Called after an RxPDO (master→slave) datagram has been sent
using PDOTxSentCallback = std::function<void(
    uint16_t slave_index,
    uint32_t cycle_count,
    uint64_t timestamp_ns
)>;

/// Called after a TxPDO (slave→master) response has been received and copied
using PDORxReceivedCallback = std::function<void(
    uint16_t slave_index,
    const uint8_t* data,
    size_t size,
    uint32_t cycle_count,
    uint64_t timestamp_ns
)>;

struct CallbackModeConfig {
    bool fire_on_tx_sent = true;
    bool fire_on_rx_received = true;
};

// ============================================================================
// Queue mode types (Mode 2)
// ============================================================================

/// A single PDO data frame for queue-based async mode
struct PDOFrame {
    std::vector<uint8_t> data;
    uint64_t timestamp_ns = 0;
    uint32_t cycle_count = 0;
};

/// Event notification for queue-based async mode
struct PDOEvent {
    enum class Type : uint8_t {
        TxSent,         ///< TX frame was sent to the NIC
        RxReceived,     ///< RX frame was received from the bus
        Underrun,       ///< TX queue was empty when the RT cycle needed data
        Error,          ///< Bus error (send failure, timeout, WKC=0)
    };

    Type type = Type::TxSent;
    uint16_t slave_index = 0;
    uint16_t pdo_entry_index = 0;
    uint64_t timestamp_ns = 0;
    uint32_t cycle_count = 0;
    std::shared_ptr<PDOFrame> frame;  ///< Associated frame data (may be null for errors)
};

/// TX underrun policy for queue mode
enum class UnderrunPolicy : uint8_t {
    RepeatLastFrame,    ///< Resend the most recent TX data
    SafeState,          ///< Send safe-state buffer (user-provided)
    SkipCycle,          ///< Don't send anything that cycle
    Custom,             ///< User callback decides
};

using UnderrunCallback = std::function<void(
    uint16_t slave_index,
    uint32_t cycle_count
)>;

struct QueueModeConfig {
    uint32_t cycle_period_us = 1000;
    uint32_t sync_interval_cycles = 10;

    UnderrunPolicy underrun_policy = UnderrunPolicy::RepeatLastFrame;
    std::vector<uint8_t> safe_state_buffer;

    uint32_t tx_queue_capacity = 64;
    uint32_t rx_queue_capacity = 64;
    uint32_t event_queue_capacity = 128;

    bool enable_tx_sent_events = true;
    bool enable_rx_received_events = true;

    /// If true, a consumer thread drains queues and invokes callbacks
    /// (for non-RT-safe user processing)
    bool use_consumer_thread = false;
};

} // namespace EtherCAT
