# Klipper Module Architecture

This document describes the architecture of the `tether_klipper` component,
which implements a Klipper-compatible firmware emulation layer for Tether.

## Module Dependency Diagram

```
                          ┌──────────────────────────────────┐
                          │         KlippyInstance           │
                          │  (top-level orchestrator)        │
                          │                                  │
                          │  ┌──────────┐  ┌──────────────┐  │
                          │  │KlippyHost│  │KlippyUdsServer│  │
                          │  │(MCU comm)│  │  (JSON-RPC)  │  │
                          │  └────┬─────┘  └──────┬───────┘  │
                          │       │               │          │
                          │  ┌────▼─────┐  ┌──────▼───────┐  │
                          │  │  Motion  │  │   Printer    │  │
                          │  │Translator│  │ObjectRegistry│  │
                          │  └────┬─────┘  └──────────────┘  │
                          │       │                          │
                          │  ┌────▼─────────────────────┐     │
                          │  │   IGlipperDevice (iface)  │     │
                          │  └────────┬────────────────┘     │
                          └───────────┼──────────────────────┘
                                      │
                          ┌───────────▼──────────────────┐
                          │      KlipperDevice            │
                          │  (protocol + peripherals)     │
                          │                               │
                          │  ┌─────────┐  ┌────────────┐  │
                          │  │DataDict │  │CommandTable│  │
                          │  └─────────┘  └────────────┘  │
                          │  ┌─────────┐  ┌────────────┐  │
                          │  │McuClock │  │StepScheduler│ │
                          │  └─────────┘  └────────────┘  │
                          └───────────┬──────────────────┘
                                      │
                          ┌───────────▼──────────────────┐
                          │   IByteStreamTransport        │
                          │  ┌────────┐ ┌──────────────┐  │
                          │  │ Loopback│ │   Pipe       │  │
                          │  │Transport│ │  Transport   │  │
                          │  └────────┘ └──────────────┘  │
                          └───────────────────────────────┘
```

### Layer Dependencies (top to bottom)

1. **KlippyInstance** — Top-level orchestrator. Wires together all
   subsystems. Depends on `IKlipperDevice` (interface), not the concrete
   `KlipperDevice`.
2. **KlippyHost** — MCU communication client. Connects to a device,
   downloads the data dictionary, syncs the clock, and sends commands.
3. **KlippyUdsServer** — Unix domain socket JSON-RPC server. Exposes
   Moonraker-compatible endpoints for external clients.
4. **MotionTranslator** — Converts Tether `MotionPlan` into Klipper
   `queue_step` command sequences. Has no dependency on the klippy layer.
5. **KlipperDevice** — Device-side protocol handler. Serves the data
   dictionary, processes command blocks, and executes motion.
6. **Transport** — Byte-stream transport abstraction (loopback, pipe, TCP).

### Key Interface Boundaries

- `IKlipperDevice` — Abstract interface for the device's motion operations
  (`start`, `pump`, `advanceClock`, `registerStepper`, `enableStepperMotion`).
  Allows `KlippyInstance` to depend on an abstraction rather than the concrete
  `KlipperDevice` implementation.

- `IByteStreamTransport` — Abstract transport interface. Allows the host and
  device to communicate over any byte-stream medium (loopback for testing,
  pipe for subprocess, TCP for network).

- `MotionBlockSink` — Abstract sink for motion blocks. Allows the device to
  feed reconstructed motion data to any consumer.

## Threading Model

```
┌─────────────────────────────────────────────────────┐
│                   Main Thread                        │
│                                                      │
│  KlippyInstance::pump()  ◄── called periodically     │
│    ├── KlippyHost::pump()     (MCU communication)    │
│    ├── KlipperDevice::pump()  (device event loop)    │
│    └── MotionTranslator::translate()                 │
│                                                      │
├─────────────────────────────────────────────────────┤
│              UDS Event Thread                        │
│                                                      │
│  KlippyUdsServer::eventLoop()  ◄── runs continuously │
│    ├── accept() new client connections               │
│    ├── read() JSON-RPC requests                      │
│    ├── dispatch to endpoint handlers                 │
│    └── write() JSON-RPC responses                    │
│                                                      │
├─────────────────────────────────────────────────────┤
│         Step Scheduler Thread (optional)             │
│                                                      │
│  StepScheduler::tick()  ◄── timer-driven             │
│    └── fires step callbacks at scheduled clock times │
│                                                      │
└─────────────────────────────────────────────────────┘
```

### Thread Safety

- **KlippyUdsServer** uses a `std::recursive_mutex` to protect internal
  state. Endpoint handlers are called with the mutex held. The event loop
  acquires the mutex for each iteration.

- **KlippyInstance** is NOT thread-safe. All methods must be called from
  the same thread (typically the main thread). The UDS server runs on its
  own thread and communicates with the instance via the mutex-protected
  `PrinterObjectRegistry`.

- **KlipperDevice** is NOT thread-safe. All methods must be called from
  the same thread. The optional `StepScheduler` runs on the calling thread
  (not a separate thread) when `tickStepScheduler()` is called.

- **KlippyHost** is NOT thread-safe. All methods must be called from the
  same thread as the owning `KlippyInstance`.

### Concurrency Guidelines

1. Call `KlippyInstance::pump()` from the main thread periodically.
2. The UDS server manages its own thread; do not call UDS methods directly
   from the main thread.
3. If using the `StepScheduler`, call `tickStepScheduler()` from the main
   thread (it uses `std::chrono::steady_clock` for timing).
4. Use `KlippyInstance::waitStepScheduler()` to block until all scheduled
   steps complete (with optional timeout).

## Performance Characteristics

### Protocol Layer

- **MessageBlock encoding**: O(n) where n = content length. CRC-16 CCITT
  is computed over header + content. Typical block size: 5-64 bytes.
- **VLQ encoding**: O(1) per parameter. Average 2-3 bytes per parameter.
- **DataDictionary lookup**: O(log n) via `std::map` (n = number of
  commands, typically 20-50).
- **CommandTable dispatch**: O(log n) via `std::map` (n = number of
  registered handlers, typically 30-100).

### Transport Layer

- **LoopbackTransport**: In-memory ring buffer. O(1) read/write.
  No system calls. Used for testing.
- **PipeTransport**: OS pipe. O(n) read/write. One syscall per operation.
  Used for subprocess communication.
- **TcpTransport**: TCP socket. O(n) read/write. One syscall per operation.
  Used for network communication.

### Motion Layer

- **MotionTranslator**: O(s * a) where s = number of samples and
  a = number of axes. Each sample produces one `StepCommand` per axis.
  Typical: 1000 samples * 4 axes = 4000 commands per move.
- **StepScheduler**: O(n log n) for scheduling n steps. Uses a priority
  queue (min-heap) ordered by clock time. Real-time execution via
  `std::chrono::steady_clock` with microsecond precision.
- **MotionReconstructor**: O(n) for reconstructing n step commands into
  a trajectory. Used for analysis only (not real-time).

### UDS Server

- **JSON parsing**: O(n) where n = input length. Hand-written recursive
  descent parser. No external dependency.
- **JSON serialization**: O(n) where n = value tree size.
- **Endpoint dispatch**: O(log n) via `std::map` (n = number of endpoints,
  typically 30-80).
- **Client handling**: Single-threaded event loop using `poll()`.
  Supports up to ~100 concurrent clients. Each client has a 64KB
  read buffer.

### Memory Usage

- **KlippyInstance**: ~500KB base, plus ~1KB per registered printer object.
- **KlipperDevice**: ~100KB base, plus ~100 bytes per peripheral OID.
- **KlippyUdsServer**: ~50KB base, plus ~64KB per connected client.
- **MotionTranslator**: ~10KB base, plus ~100 bytes per pending command.
- **DataDictionary**: ~5KB for a typical 30-command dictionary.

### Latency

- **MCU command round-trip**: ~1ms (loopback), ~5ms (pipe), ~10ms (TCP).
  Dominated by transport latency.
- **UDS request handling**: ~0.1ms for simple endpoints, ~1ms for endpoints
  that trigger G-code execution.
- **Clock sync**: ~10ms for 5 samples (converges in 3-5 samples).
- **Step execution**: ~10us per step (StepScheduler, real-time).

## Key Design Decisions

### 1. Header-Only KlippyInstance

`KlippyInstance` is implemented entirely in a header file. This was chosen
to allow inlining of the hot-path `pump()` method, which is called
thousands of times per second. The trade-off is longer compile times.

### 2. Recursive Mutex in UDS Server

The UDS server uses `std::recursive_mutex` because endpoint handlers may
call back into server methods (e.g., to query printer state). This allows
handlers to be written without worrying about deadlock, at the cost of
slightly reduced performance compared to a non-recursive mutex.

### 3. Interface Extraction (IKlipperDevice)

`KlipperDevice` implements `IKlipperDevice` to break the dependency from
the klippy layer to the device layer. `KlippyInstance` holds a
`std::unique_ptr<IKlipperDevice>` and exposes `IKlipperDevice*` to callers.
This allows mocking the device in tests and swapping implementations.

### 4. Separate Config Header

`KlipperDeviceConfig` is extracted to a separate header
(`KlipperDeviceConfig.hpp`) so that clients can construct a config without
including the full `KlipperDevice.hpp` (which pulls in the protocol,
clock, and motion headers).

### 5. VLQ Encoding for Wire Protocol

The Klipper wire protocol uses variable-length quantity (VLQ) encoding
for message IDs and parameters. This minimizes wire overhead for small
values (common case) while supporting the full 10-bit message ID range
and 32-bit parameter values.

### 6. Single-Threaded Device Pump

The device event loop (`pump()`) is single-threaded and non-blocking.
It reads available data from the transport, parses complete blocks,
dispatches commands, and sends acks. This avoids the complexity of
multi-threaded protocol handling while maintaining responsiveness.
