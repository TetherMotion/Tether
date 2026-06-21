# PDO Interaction Modes

This document describes the three PDO interaction modes provided by `PDOManager` and guides you in choosing the right one for your application.

## Overview

The `PDOManager` supports three mutually exclusive interaction modes for EtherCAT Process Data Object (PDO) exchange. Each mode targets a different use-case pattern, from hard real-time control loops to asynchronous, convenience-oriented APIs.

| Mode | Enum | Pattern | RT-Safe | When to Use |
|------|------|---------|---------|-------------|
| **Direct** | `PDOMode::Direct` | User calls `sendAll()` / `receiveAll()` directly from RT thread | Yes | Hard real-time control loops where you manage the thread yourself |
| **Queue** | `PDOMode::Queue` | Internal RT loop; user interacts via lock-free queues | Yes (RT loop) / No (user side) | Async processing, data logging, non-RT user code that feeds data to/from the bus |
| **Callback** | `PDOMode::Callback` | Per-entry callbacks fire during `sendAll()` / `receiveAll()` | Depends on callback | Event-driven notifications for specific PDO entries without polling |

### Decision Guide

```
Do you need a hard real-time loop?
├── Yes → Do you want to manage the RT thread yourself?
│         ├── Yes → Mode 1: Direct (sendAll / receiveAll)
│         └── No  → Mode 2: Queue (startQueueModeLoop)
└── No  → Do you need per-entry event notifications?
          ├── Yes → Mode 3: Callback
          └── No  → Mode 2: Queue (async, audio-driver model)
```

### Mode Exclusivity

Modes are **mutually exclusive per `PDOManager` instance**. Calling `configureCallbackMode()` or `configureQueueMode()` switches the mode. To revert to Direct mode, create a new `PDOManager` instance.

```cpp
// Check current mode
if (mgr.getMode() == PDOMode::Queue) { ... }
```

---

## Mode 1: Direct (Split Send / Receive)

### Description

The default mode. You call `sendAll()` and `receiveAll()` directly from your real-time thread. This allows overlapping computation with bus I/O: send RxPDO datagrams, do some work while the bus round-trips, then collect TxPDO responses.

For LRW (Logical Read-Write) mode, the exchange is atomic and cannot be split — `sendAll()` performs the full exchange and `receiveAll()` is a no-op.

### API

| Method | Description |
|--------|-------------|
| `sendAll()` | Sends all RxPDO datagrams (master → slave). Returns `true` if send phase succeeded. |
| `receiveAll()` | Waits for all TxPDO responses (slave → master) and copies data into app buffers. Returns `true` if all responses received successfully. |
| `exchangeAll()` | Convenience: calls `sendAll()` then `receiveAll()`. Backward-compatible with pre-refactor API. |

### Example: Split Send/Receive in a User-Managed RT Loop

```cpp
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;

void controlLoop(Master& master) {
    auto& pdo = master.pdo();

    // PDO mapping is set up earlier (add_rxpdo, add_txpdo, finalizeMapping, etc.)
    // Mode is Direct by default — no configure* call needed.

    // Option A: Use Master's built-in RT loop with a motion control callback
    master.setMotionControlCallback([&pdo](double dt) -> bool {
        // Send all RxPDOs (motor setpoints, outputs)
        if (!pdo.sendAll()) return false;

        // --- Overlap: do computation while bus round-trips ---
        // e.g. update controller state, compute next setpoint

        // Receive all TxPDOs (encoder feedback, inputs)
        if (!pdo.receiveAll()) return false;

        return true;  // continue loop
    });

    master.startRealtimeMotionControlLoop();
    // ... loop runs until stopMotionControlLoop() is called
}
```

### Example: Manual exchangeAll (Simplest, No Overlap)

```cpp
// If you don't need overlap, exchangeAll() is equivalent:
master.setMotionControlCallback([&pdo](double dt) -> bool {
    return pdo.exchangeAll();
});
master.startRealtimeMotionControlLoop();
```

### LRW Mode Caveat

When `LogicalAddressManager` is initialized (LRW mode), the entire PDO exchange happens in a single atomic LRW datagram. In this case:

- `sendAll()` performs the full exchange (send + receive)
- `receiveAll()` is a no-op that returns the result from `sendAll()`
- `exchangeAll()` still works correctly (calls both, but `receiveAll()` does nothing)

You cannot overlap computation with bus I/O in LRW mode.

---

## Mode 2: Queue-Based Async (Audio Driver Model)

### Description

Inspired by audio drivers: an internal real-time loop runs autonomously, exchanging PDOs every cycle. The user interacts through lock-free queues:

- **TX queues** (per PDO entry): user pushes data to send; RT loop pops and writes to the bus
- **RX queues** (per PDO entry): RT loop pushes received data; user pops at their own pace
- **Event queue** (global): notifications for TX sent, RX received, underruns, and errors

All queues use `atomic_queue` (lock-free, fixed-size MPMC). Queue operations are non-blocking (`try_push` / `try_pop`). If a queue is full, data is silently dropped — the RT thread is never blocked.

### Underrun Handling

When the RT loop needs TX data but the queue is empty (underrun), a configurable policy determines behavior:

| Policy | Behavior |
|--------|----------|
| `UnderrunPolicy::RepeatLastFrame` | Resend the most recently queued TX data (default) |
| `UnderrunPolicy::SafeState` | Send a user-provided safe-state buffer |
| `UnderrunPolicy::SkipCycle` | Skip sending for that entry this cycle |
| `UnderrunPolicy::Custom` | Invoke user callback; no automatic action |

An `Underrun` event is always pushed to the event queue regardless of policy.

### API

#### PDOManager Methods

| Method | Description |
|--------|-------------|
| `configureQueueMode(config)` | Switch to Queue mode and allocate queues |
| `enqueueTx(entry_index, frame)` | Push a `shared_ptr<PDOFrame>` to a TX queue (non-blocking) |
| `tryDequeueRx(entry_index, &frame)` | Pop a `shared_ptr<PDOFrame>` from an RX queue (non-blocking) |
| `tryPollEvent(&event)` | Pop a `shared_ptr<PDOEvent>` from the global event queue |
| `setUnderrunCallback(callback)` | Set a callback invoked on underrun (called from RT thread) |
| `queueCycle()` | Execute one RT cycle (called internally by the loop; can also be called manually) |

#### Master Methods

| Method | Description |
|--------|-------------|
| `startQueueModeLoop()` | Start the internal RT loop (uses `RealtimeLoop` infrastructure) |
| `startQueueModeLoop(config)` | Start with custom `RealtimeMotionLoopConfig` (cycle period, DC sync, etc.) |
| `stopQueueModeLoop()` | Stop the internal RT loop |
| `isQueueModeLoopRunning()` | Check if the loop is running |

### Configuration (`QueueModeConfig`)

| Field | Default | Description |
|-------|---------|-------------|
| `cycle_period_us` | `1000` | RT loop period in microseconds |
| `sync_interval_cycles` | `10` | DC sync frame interval (in cycles) |
| `underrun_policy` | `RepeatLastFrame` | Behavior when TX queue is empty |
| `safe_state_buffer` | empty | Buffer for `SafeState` policy |
| `tx_queue_capacity` | `64` | Per-entry TX queue size |
| `rx_queue_capacity` | `64` | Per-entry RX queue size |
| `event_queue_capacity` | `128` | Global event queue size |
| `enable_tx_sent_events` | `true` | Generate `TxSent` events |
| `enable_rx_received_events` | `true` | Generate `RxReceived` events |
| `use_consumer_thread` | `false` | (Future) auto-drain queues on a consumer thread |

### Example: Producer / Consumer Pattern

```cpp
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDOModes.hpp"
#include <memory>
#include <thread>
#include <atomic>

using namespace EtherCAT;

void queueModeExample(Master& master) {
    auto& pdo = master.pdo();

    // Configure queue mode
    QueueModeConfig config;
    config.cycle_period_us = 1000;       // 1 kHz
    config.underrun_policy = UnderrunPolicy::RepeatLastFrame;
    config.tx_queue_capacity = 32;
    config.rx_queue_capacity = 32;
    config.event_queue_capacity = 256;
    pdo.configureQueueMode(config);

    // Set up underrun callback (optional, called from RT thread)
    pdo.setUnderrunCallback([](uint16_t slave_index, uint32_t cycle) {
        // Called from RT thread — keep it short and allocation-free
        // e.g. increment a counter, set a flag
    });

    // Start the internal RT loop
    master.startQueueModeLoop();

    std::atomic<bool> running{true};

    // --- Producer thread: feeds TX data ---
    std::thread producer([&]() {
        while (running) {
            // Build a frame with motor setpoints
            auto frame = std::make_shared<PDOFrame>();
            frame->data.resize(sizeof(uint32_t));
            uint32_t setpoint = computeSetpoint();
            std::memcpy(frame->data.data(), &setpoint, sizeof(setpoint));

            // entry_index 0 is the first RxPDO (master→slave output)
            if (!pdo.enqueueTx(0, frame)) {
                // TX queue full — user is producing faster than bus can send
                // Data is dropped; consider slowing down or increase queue capacity
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // --- Consumer thread: drains RX data and events ---
    std::thread consumer([&]() {
        while (running) {
            // Drain RX data (entry_index 1 = first TxPDO, slave→master input)
            std::shared_ptr<PDOFrame> rx_frame;
            while (pdo.tryDequeueRx(1, rx_frame)) {
                // Process received data at your own pace
                uint32_t feedback;
                std::memcpy(&feedback, rx_frame->data.data(), sizeof(feedback));
                processFeedback(feedback, rx_frame->cycle_count);
            }

            // Drain events
            std::shared_ptr<PDOEvent> event;
            while (pdo.tryPollEvent(event)) {
                switch (event->type) {
                    case PDOEvent::Type::TxSent:
                        // Confirmed: TX data was sent to the NIC
                        break;
                    case PDOEvent::Type::RxReceived:
                        // Confirmed: RX data was received from the bus
                        break;
                    case PDOEvent::Type::Underrun:
                        // TX queue was empty — check your producer rate
                        logUnderrun(event->slave_index, event->cycle_count);
                        break;
                    case PDOEvent::Type::Error:
                        // Bus error (send failure, timeout, WKC=0)
                        logError(event->cycle_count);
                        break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // ... run for a while ...

    running = false;
    producer.join();
    consumer.join();
    master.stopQueueModeLoop();
}
```

### Memory Management

Data in queues is managed via `std::shared_ptr<PDOFrame>`. When enqueuing, the `shared_ptr` is moved into the queue (no copy). When the RT loop pops it, the `shared_ptr` is moved out. This ensures zero-copy semantics between user and RT threads, and automatic memory reclamation when the last reference is dropped.

**Important**: If the user thread is slow to consume RX data, frames accumulate in the RX queue until it fills up. Subsequent pushes are silently dropped. The RT thread is never blocked.

### Manual `queueCycle()` (Without `Master`)

If you are not using `Master` (e.g. in unit tests or custom setups), you can call `queueCycle()` directly from your own thread:

```cpp
pdo.configureQueueMode(config);
// ... enqueue TX data ...
pdo.queueCycle();  // One cycle: drain TX → send → receive → push RX + events
// ... dequeue RX data and events ...
```

---

## Mode 3: Callback

### Description

Per-entry callbacks fire inline during `sendAll()` and `receiveAll()` on the calling thread. This is useful when you want event-driven notifications for specific PDO entries without polling queues or checking buffers.

**RT-Safety**: Callbacks execute on the same thread that calls `sendAll()` / `receiveAll()`. If that thread is a real-time thread, your callbacks must be RT-safe (no heap allocation, no blocking, no I/O).

### API

| Method | Description |
|--------|-------------|
| `configureCallbackMode(config)` | Switch to Callback mode |
| `setTxSentCallback(entry_index, callback)` | Set callback fired after an RxPDO datagram is sent |
| `setRxReceivedCallback(entry_index, callback)` | Set callback fired after a TxPDO response is received and data is copied |

### Callback Signatures

```cpp
// Called after an RxPDO (master→slave) datagram has been sent
using PDOTxSentCallback = std::function<void(
    uint16_t slave_index,
    uint32_t cycle_count,
    uint64_t timestamp_ns
)>;

// Called after a TxPDO (slave→master) response has been received
using PDORxReceivedCallback = std::function<void(
    uint16_t slave_index,
    const uint8_t* data,
    size_t size,
    uint32_t cycle_count,
    uint64_t timestamp_ns
)>;
```

### Configuration (`CallbackModeConfig`)

| Field | Default | Description |
|-------|---------|-------------|
| `fire_on_tx_sent` | `true` | Enable/disable TxSent callbacks |
| `fire_on_rx_received` | `true` | Enable/disable RxReceived callbacks |

### Example: Callback Mode with RT Loop

```cpp
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDOModes.hpp"
#include <atomic>

using namespace EtherCAT;

void callbackModeExample(Master& master) {
    auto& pdo = master.pdo();

    // Enable callback mode
    pdo.configureCallbackMode();

    // Set up PDO mapping (indices 0 = RxPDO output, 1 = TxPDO input)
    uint32_t rx_buf = 0, tx_buf = 0;
    int rx_idx = pdo.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600,
                                          PDOAddressMode::Position);
    int tx_idx = pdo.mapping().add_txpdo(0, &tx_buf, sizeof(tx_buf), 0x1A00,
                                          PDOAddressMode::Position);

    // Callback: fires after motor setpoint is sent to the slave
    pdo.setTxSentCallback(static_cast<size_t>(rx_idx),
        [](uint16_t slave_index, uint32_t cycle, uint64_t ts) {
            // RT-safe: just update an atomic counter or set a flag
            // Do NOT allocate memory, log, or do I/O here
        });

    // Callback: fires after encoder feedback is received
    std::atomic<uint32_t> last_feedback{0};
    pdo.setRxReceivedCallback(static_cast<size_t>(tx_idx),
        [&last_feedback](uint16_t slave_index, const uint8_t* data,
                         size_t size, uint32_t cycle, uint64_t ts) {
            if (size >= sizeof(uint32_t)) {
                uint32_t val;
                std::memcpy(&val, data, sizeof(val));
                last_feedback.store(val, std::memory_order_relaxed);
            }
        });

    // Run with Master's RT loop (callbacks fire on the RT thread)
    master.setMotionControlCallback([&pdo](double dt) -> bool {
        return pdo.exchangeAll();
    });
    master.startRealtimeMotionControlLoop();

    // ... last_feedback is updated automatically each cycle ...

    master.stopMotionControlLoop();
}
```

### Example: Callback Mode with Selective Enable

```cpp
// Only fire RxReceived callbacks, skip TxSent
CallbackModeConfig config;
config.fire_on_tx_sent = false;
config.fire_on_rx_received = true;
pdo.configureCallbackMode(config);
```

---

## API Reference

### Types (`PDOModes.hpp`)

| Type | Description |
|------|-------------|
| `PDOMode` | Enum: `Direct`, `Callback`, `Queue` |
| `PDOFrame` | Data frame: `data` (vector<uint8_t>), `timestamp_ns`, `cycle_count` |
| `PDOEvent` | Event notification: `Type`, `slave_index`, `pdo_entry_index`, `timestamp_ns`, `cycle_count`, `frame` |
| `PDOEvent::Type` | Enum: `TxSent`, `RxReceived`, `Underrun`, `Error` |
| `UnderrunPolicy` | Enum: `RepeatLastFrame`, `SafeState`, `SkipCycle`, `Custom` |
| `CallbackModeConfig` | Config for callback mode: `fire_on_tx_sent`, `fire_on_rx_received` |
| `QueueModeConfig` | Config for queue mode (see table above) |
| `PDOTxSentCallback` | `std::function<void(uint16_t, uint32_t, uint64_t)>` |
| `PDORxReceivedCallback` | `std::function<void(uint16_t, const uint8_t*, size_t, uint32_t, uint64_t)>` |
| `UnderrunCallback` | `std::function<void(uint16_t, uint32_t)>` |

### PDOManager Public API (Mode-Related)

| Method | Mode | Description |
|--------|------|-------------|
| `sendAll()` | Direct / Callback | Send all RxPDO datagrams |
| `receiveAll()` | Direct / Callback | Receive all TxPDO responses |
| `exchangeAll()` | Direct / Callback | `sendAll()` + `receiveAll()` |
| `getMode()` | All | Returns current `PDOMode` |
| `configureCallbackMode(config)` | Callback | Switch to callback mode |
| `setTxSentCallback(idx, cb)` | Callback | Set per-entry TxSent callback |
| `setRxReceivedCallback(idx, cb)` | Callback | Set per-entry RxReceived callback |
| `configureQueueMode(config)` | Queue | Switch to queue mode, allocate queues |
| `enqueueTx(idx, frame)` | Queue | Push to TX queue (non-blocking) |
| `tryDequeueRx(idx, &frame)` | Queue | Pop from RX queue (non-blocking) |
| `tryPollEvent(&event)` | Queue | Pop from event queue (non-blocking) |
| `setUnderrunCallback(cb)` | Queue | Set underrun notification callback |
| `queueCycle()` | Queue | Execute one RT cycle (drain TX → bus → push RX + events) |

### Master Public API (Queue Mode)

| Method | Description |
|--------|-------------|
| `startQueueModeLoop()` | Start internal RT loop for queue mode |
| `startQueueModeLoop(config)` | Start with custom `RealtimeMotionLoopConfig` |
| `stopQueueModeLoop()` | Stop the queue mode RT loop |
| `isQueueModeLoopRunning()` | Check if running |

### Statistics

All modes update the same `TransferStats` structure, accessible via `pdo.getStats()`:

| Field | Description |
|-------|-------------|
| `total_cycles` | Total PDO exchange cycles executed |
| `rxpdo_frames_sent` | Total RxPDO datagrams successfully sent |
| `rxpdo_errors` | Total RxPDO send errors |
| `txpdo_frames_recv` | Total TxPDO datagrams successfully received |
| `txpdo_errors` | Total TxPDO receive errors |

---

## Comparison Summary

| Feature | Direct (Mode 1) | Queue (Mode 2) | Callback (Mode 3) |
|---------|-----------------|----------------|-------------------|
| Thread management | User | Internal (RT loop) | User |
| RT-safety | Full | RT loop is safe; user side is async | Depends on callback |
| Data flow | Direct buffer access | `shared_ptr<PDOFrame>` queues | Direct buffer + callback |
| Overlap send/receive | Yes (split calls) | No (internal) | Yes (split calls) |
| Underrun handling | N/A (user controls timing) | Configurable policies | N/A |
| Event feedback | None | `PDOEvent` queue | Callbacks |
| Memory model | User-owned buffers | `shared_ptr` (shared ownership) | User-owned buffers |
| Best for | Hard RT control loops | Async / logging / non-RT user code | Event-driven per-entry notifications |

---

## See Also

- [Tether API Documentation](api.md)
- [HAL Porting Guide](HAL_PORTING_GUIDE.md)
- [IO Protocol](IOProtocol.md)
- `include/tether/ethercat/PDOModes.hpp` — Type definitions
- `include/tether/ethercat/PDOManager.hpp` — PDOManager class
- `tests/ethercat/test_pdo_modes.cpp` — Comprehensive test examples
