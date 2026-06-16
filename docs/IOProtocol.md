# Tether IO Protocol

High-level guide for integrating and using the `tether_io_protocol` module.

## Overview

The Tether IO Protocol provides **real-time parameter and signal streaming** over TCP or serial transports using [SLIP framing](https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol). It is designed for embedded systems (ESP32, Linux host) and supports:

- **Parameters** — read/write values exposed by Tether modules (e.g. PID gains, feed overrides)
- **Signals** — read-only values (e.g. motor positions, sensor readings)
- **High-speed streaming** — periodic or change-based with configurable chunking and skip filtering
- **Datalogging** — binary recording of sampled entries to a configurable sink
- **Threshold filtering** — absolute, relative, or custom change detection per entry
- **Feature exchange** — client/server capability negotiation
- **Snapshots** — bulk read of params or signals in one atomic request
- **Binary struct descriptions** — composite value layouts with named fields
- **Catalog change notifications** — server pushes when new params/signals are registered

## Architecture

```
┌────────────────────────────────────────────────┐
│                 Application                     │
│  ┌─────────┐ ┌─────────┐ ┌──────────────────┐  │
│  │  PID    │ │ EtherCAT│ │  Motion Planner  │  │
│  │Controller│ │  Master │ │                  │  │
│  └────┬────┘ └────┬────┘ └────────┬─────────┘  │
│       │           │               │             │
│  ┌────▼───────────▼───────────────▼──────────┐  │
│  │              Registry                      │  │
│  │   (thread-safe param/signal catalog)       │  │
│  └────────────────┬───────────────────────────┘  │
│                   │                              │
│  ┌────────────────▼───────────────────────────┐  │
│  │              Server                         │  │
│  │   ┌──────────┐  ┌──────────┐               │  │
│  │   │ Session 1│  │ Session 2│  ...          │  │
│  │   │ (Thread) │  │ (Thread) │               │  │
│  │   └────┬─────┘  └────┬─────┘              │  │
│  └────────┼──────────────┼────────────────────┘  │
│           │              │                       │
│  ┌────────▼──────────────▼────────────────────┐  │
│  │        Transport (TCP / Serial)             │  │
│  └─────────────────────────────────────────────┘  │
└────────────────────────────────────────────────┘
```

### Key components

| Component | Header | Description |
|---|---|---|
| Protocol | `Protocol.hpp` | Wire format definitions, BufWriter/BufReader, varint, enums |
| Registry | `Registry.hpp` | Thread-safe param/signal catalog with change listeners |
| Session | `Session.hpp` | Per-client session: SLIP deframing, protocol dispatch, streaming |
| Server | `Server.hpp` | Multi-client accept loop, session lifecycle management |
| Transport | `Transport.hpp` | Abstract transport interfaces (ITransport, ITransportServer) |
| TcpTransport | `TcpTransport.hpp` | POSIX TCP transport implementation |
| SerialTransport | `SerialTransport.hpp` | Serial transport with abstract driver (POSIX impl included) |
| ThresholdFilter | `ThresholdFilter.hpp` | Change detection with absolute/relative/custom thresholds |
| FeatureExchange | `FeatureExchange.hpp` | Client/server feature negotiation |
| Datalogging | `Datalogging.hpp` | Binary datalogging subsystem |
| BinaryStruct | `BinaryStruct.hpp` | Composite struct field descriptions |
| ParameterExposer | `ParameterExposer.hpp` | Interface for modules to expose their params/signals |

## Namespace

All IO protocol types live in `tether::io`. Exposers live in `tether::io::exposers`.

## Quick start

### 1. Create a registry and expose module parameters

```cpp
#include "tether/io/Registry.hpp"
#include "tether/io/exposers/PIDExposer.hpp"

tether::io::Registry registry;

// Your PID controller (from tether_controls)
PIDController pidX;

// Expose it — adds all params/signals to the registry
tether::io::exposers::PIDExposer pidExposer(pidX, "x_axis");
pidExposer.expose(registry);
```

### 2. Start a TCP server

```cpp
#include "tether/io/Server.hpp"
#include "tether/io/TcpTransport.hpp"

auto tcpServer = std::make_unique<TcpTransportServer>(4000, 5);

ServerConfig cfg;
cfg.maxClients = 4;
cfg.timestampFn = []() -> uint64_t {
    return /* your microsecond timestamp */;
};

Server server(registry, std::move(tcpServer), cfg);
server.start();
// Server runs in background threads, one per client session
```

### 3. Connect and stream (client side)

The wire protocol is transport-agnostic. A client:
1. Connects via TCP to port 4000
2. Sends `FeatureExchangeReq` to negotiate capabilities
3. Sends `ListParamsReq` / `ListSignalsReq` to discover the catalog
4. Sends `ConfigureStreamReq` with desired entry IDs and trigger mode
5. Sends `StartStream` to begin receiving `StreamData` packets
6. Sends `StopStream` to stop

## Exposer pattern

Every Tether module exposes its parameters and signals through a header-only **Exposer** class that implements `IParameterExposer`:

```cpp
class IParameterExposer {
public:
    virtual ~IParameterExposer() = default;
    virtual void expose(Registry& registry) = 0;
    virtual const char* moduleName() const = 0;
};
```

### Available exposers

| Exposer | Module | Params | Signals |
|---|---|---|---|
| `PIDExposer` | PIDController | kp, ki, kd, derivative_filter, enabled, output_min, output_max | error, proportional, integral_term, derivative_term, output, saturated, cycle_count |
| `EtherCATExposer` | Master | enable_mailbox_fallback | discovered_slave_count, is_running, per-slave ec_state/has_fault |
| `CiA402Exposer` | CiA402Drive | controlword | statusword, drive_state, ec_state, is_enabled, is_faulted, target_reached, operating_mode |
| `GCodeExposer` | GCode Interpreter | dry_run, block_delete, optional_stop, mode | state, current_line, total_lines, position.x/y/z, machine_position.x/y/z, stats.* |
| `MotionPlannerExposer` | MotionPlanner | feed_override | total_duration, total_length, num_segments, is_paused, is_reverse |
| `SimulationExposer` | SimulationEngine | — | current_time, is_finished, state[0..N] |

### Writing a custom exposer

```cpp
#include "tether/io/ParameterExposer.hpp"

class MyModuleExposer : public tether::io::IParameterExposer {
    MyModule& module_;
public:
    MyModuleExposer(MyModule& m) : module_(m) {}

    const char* moduleName() const override { return "my_module"; }

    void expose(tether::io::Registry& reg) override {
        using namespace tether::io;

        // Expose a read/write parameter
        ParamEntry p;
        p.id = makeId(ModuleId::User, 0x01);
        p.name = "my_param";
        p.description = "My tunable parameter";
        p.group = "my_module";
        p.valueType = ValueType::F64;
        p.readFn = [this](void* d) {
            double v = module_.getMyParam();
            std::memcpy(d, &v, 8);
        };
        p.writeFn = [this](const void* s) {
            double v; std::memcpy(&v, s, 8);
            module_.setMyParam(v);
        };
        reg.addParam(std::move(p));

        // Expose a read-only signal
        SignalEntry s;
        s.id = makeId(ModuleId::User, 0x01);
        s.name = "my_signal";
        s.description = "Current sensor reading";
        s.group = "my_module";
        s.valueType = ValueType::F32;
        s.readFn = [this](void* d) {
            float v = module_.getSensorValue();
            std::memcpy(d, &v, 4);
        };
        reg.addSignal(std::move(s));
    }
};
```

### Module ID allocation

Well-known module IDs are defined in `ModuleId` namespace:

| ID | Module |
|---|---|
| 0x0001 | Master |
| 0x0002 | CiA402Drive |
| 0x0003 | PIDController |
| 0x0004 | MotionPlanner |
| 0x0005 | GCodeInterpreter |
| 0x0006 | SimulationEngine |
| 0x0007 | HAL |
| 0x0008 | FSoE |
| 0x0009 | Kinematics |
| 0x000A | Destabilizer |
| 0x000B | MotionControl |
| 0x000C | Export |
| 0x000D | Identification |
| 0x000E | SlaveEmulation |
| 0x000F | CiA301 |
| 0x0010 | CiA401 |
| 0x0011 | CiA406 |
| 0x1000 | User (start of user-defined range) |

Use `makeId(moduleId, localId)` to combine: `entryId = (moduleId << 16) | localId`.

## Dependency isolation

Core Tether modules (`tether_controls`, `tether_gcode`, etc.) **do not depend on** `tether_io_protocol`. The exposer headers include both the module header and the IO registry header — the dependency only exists at the application integration level.

This means:
- You can build `tether_controls` without `tether_io_protocol`
- The IO module is controlled by `TETHER_BUILD_IO_PROTOCOL` CMake option
- Exposer headers are in `tether/io/exposers/` and are only included by application code

## Streaming modes

### Time-based streaming

Samples all configured entries at a fixed interval:

```
ConfigureStreamReq {
    triggerMode: Time,
    intervalUs: 1000,    // 1 kHz
    chunkSize: 10,       // send every 10 rows
    skipCount: 0,
    entryIds: [1, 2, 3]
}
```

### Change-based streaming

Triggers only when any sampled entry changes value:

```
ConfigureStreamReq {
    triggerMode: OnChange,
    intervalUs: 100,     // minimum interval (µs)
    chunkSize: 1,
    skipCount: 0,
    entryIds: [1, 2, 3]
}
```

### Skip filtering

`skipCount` skips N samples between each transmitted sample (decimation):

```
skipCount: 9  // transmit every 10th sample (divide rate by 10)
```

### Chunking

`chunkSize` batches multiple rows into a single SLIP packet to reduce framing overhead:

```
chunkSize: 50  // send 50 rows per StreamData packet
```

## Threshold filtering

Configure per-entry change thresholds to suppress small or insignificant changes during streaming:

```
ConfigureThresholdReq {
    name: "default",
    isWhitelist: false,
    rules: [
        { entryId: 1, type: Absolute, threshold: 0.01 },
        { entryId: 2, type: Relative, threshold: 0.05 },
    ]
}
```

Threshold types:
- **Absolute** — send when `|new − old| > threshold`
- **Relative** — send when `|new − old| / |old| > threshold`
- **Custom** — user-defined evaluation function registered by name

## Datalogging

Binary datalogging writes sampled data to a configurable sink (file, UART, ring buffer):

```
ConfigureDatalogReq {
    logName: "capture1",
    sampleRateHz: 1000,
    enabled: true,
    entryIds: [1, 2, 3]
}
```

Each record is: `[8-byte timestamp] [field1 bytes] [field2 bytes] ...`

The `DatalogRecorder` class handles configuration, sampling, and writing. The sink callback receives raw binary record data.

## Feature exchange

Clients and servers exchange capability sets at connection start:

```
Client → FeatureExchangeReq { client_name: "MyTool", ... }
Server → FeatureExchangeResp { protocol_version: 1, max_stream_entries: 256, ... }
```

Standard feature names:
- `protocol_version` (U32) — always present
- `client_name` / `server_name` (String)
- `max_stream_entries` (U32)
- `supports_datalogging` (Bool)
- `supports_threshold` (Bool)
- `supports_binary_struct` (Bool)

## Serial transport

For serial connections, implement the `ISerialDriver` interface or use the built-in `PosixSerialDriver`:

```cpp
#include "tether/io/SerialTransport.hpp"

auto driver = std::make_unique<PosixSerialDriver>();
auto transport = std::make_unique<SerialTransport>(std::move(driver), "/dev/ttyUSB0", 115200);
```

On ESP-IDF, provide a custom `ISerialDriver` wrapping `uart_driver_install()` / `uart_write_bytes()` / `uart_read_bytes()`.

## CMake

Enable/disable with:

```cmake
option(TETHER_BUILD_IO_PROTOCOL "Build IO protocol (SLIP-based parameter streaming)" ON)
```

Link in your application:

```cmake
target_link_libraries(my_app PRIVATE tether_io_protocol)
```

The component automatically builds `libSLIPspeed` as a static library and links it.

## Wire format

See [IOProtocolWireFormat.md](IOProtocolWireFormat.md) for the complete byte-level specification.
