# AGENTS.md - Project Guide for AI Agents

## Project Overview

Tether is a C++23 EtherCAT master and motion control framework for 3D printers
and CNC machines. It includes a Klipper-compatible firmware emulation layer
(`tether_klipper`) that implements the Klipper wire protocol, G-code execution,
and Moonraker-compatible UDS API.

## Build Commands

```bash
# Configure (with Klipper support)
cmake -B build -DTETHER_ENABLE_KLIPPER=1

# Configure with pressure advance (compile-time default ON, runtime opt-in)
# To compile out PA entirely: -DTETHER_ENABLE_PRESSURE_ADVANCE=OFF
cmake -B build -DTETHER_ENABLE_KLIPPER=1 -DTETHER_ENABLE_PRESSURE_ADVANCE=ON

# Build all
cmake --build build -j$(nproc)

# Build only klipper tests
cmake --build build --target tether_klipper_tests -j$(nproc)

# Build klipper HTTP server (requires Drogon)
cmake --build build --target tether_klipper_http_shared -j$(nproc)

# Build klipper HTTP tests
cmake --build build --target tether_klipper_http_tests -j$(nproc)

# Run klipper tests (excluding slow thermal simulation)
./build/bin/tests/tether_klipper_tests --gtest_filter='-ThermalIntegrationTest.*'

# Run klipper HTTP tests
./build/bin/tests/tether_klipper_http_tests

# Run all klipper tests (including thermal, takes ~6 min)
./build/bin/tests/tether_klipper_tests
```

## Test Commands

```bash
# Run specific test suite
./build/bin/tests/tether_klipper_tests --gtest_filter='KlippyUdsTest.*'

# Run fuzz tests
./build/bin/tests/tether_klipper_tests --gtest_filter='*Fuzz*:*Property*'

# Run with verbose output
./build/bin/tests/tether_klipper_tests --gtest_print_time=0
```

## Architecture

See `docs/KlipperArchitecture.md` for the module dependency diagram,
threading model, and performance characteristics.

### Key Layers (top to bottom)

1. **KlippyInstance** — Top-level orchestrator (header-only)
2. **KlippyServer** — Business logic: endpoints, state, data stores (transport-agnostic)
3. **KlippyUdsServer** — Thin UDS transport (delegates to KlippyServer)
4. **KlippyHttpServer** — Thin HTTP/WebSocket transport for Mainsail/Fluidd (delegates to KlippyServer)
5. **KlippyHost** — MCU communication client
6. **MotionTranslator** — MotionPlan to queue_step translation
7. **KlipperDevice** — Device-side protocol handler (implements `IKlipperDevice`)
8. **Transport** — Byte-stream abstraction (loopback, pipe, TCP)

### Native HTTP/WebSocket Server (tether_klipper_http)

The `tether_klipper_http` component implements the full Moonraker HTTP + WebSocket
API directly in C++, eliminating the need for a separate Moonraker process between
Tether and Mainsail/Fluidd frontends.

**Key design:**
- Uses Drogon as the HTTP/WebSocket framework and Glaze for JSON serialization
- Delegates all 120+ endpoint handlers to the shared `KlippyServer` via `callEndpoint()`
- `GlazeAdapter` converts between `JsonValue` (KlippyServer) and `glz::generic` (Glaze JSON)
- `JsonRpcDispatcher` maps dotted JSON-RPC method names (e.g. `server.info`) to
  slash-style UDS method names (e.g. `server/info`)
- `WsSessionManager` tracks WebSocket client sessions and subscriptions
- `NotificationBridge` implements `NotificationSink` to fan out events to WS clients
- `KlippyWsController` is a Drogon WebSocketController for the `/websocket` endpoint

**Build:** Requires Drogon. Enabled automatically when Drogon is found, or
explicitly with `-DTETHER_ENABLE_KLIPPER_HTTP=ON`. If jsoncpp dev headers are
missing, the build system FetchContent's jsoncpp automatically. Stub CMake
find modules for Drogon's optional DB dependencies (pg, SQLite3, MySQL, etc.)
are provided in `cmake/drogon_compat/`.

### Interface Boundaries

- `IKlipperDevice` — Abstract device interface (breaks klippy->device coupling)
- `IByteStreamTransport` — Abstract transport interface
- `MotionBlockSink` — Abstract motion block consumer

### Transport-Agnostic Server Architecture

`KlippyServer` holds all business logic (endpoint handlers, state management,
data stores). Transport layers are thin wrappers:

- `KlippyUdsServer` — UDS transport (socket lifecycle, frame parsing, UDS subscriptions)
- `KlippyHttpServer` — HTTP/WebSocket transport (Drogon routes, JSON-RPC, WS sessions)

Both transports share a single `KlippyServer` instance. See
`examples/klipper_http_mainsail.cpp` for a complete example running both
transports, and `docs/MainsailDocker.md` for Docker deployment with Mainsail.

## Code Conventions

- C++23 (`-std=c++23`)
- Use `std::format` instead of string concatenation or `std::to_string`
- Use `std::ranges` algorithms where applicable
- Error handling: `std::expected<T, KlipperError>` for recoverable errors
- Logging: `KLIPPER_LOG_ERROR`, `KLIPPER_LOG_WARN`, `KLIPPER_LOG_INFO` macros
- Tests: Google Test (gtest), use `ASSERT_*` for setup, `EXPECT_*` for checks
- Header guards: `#pragma once`
- Namespaces: `tether::klipper::{layer}` (e.g., `device`, `klippy`, `motion`)

## Test File Organization

Tests are in `tests/klipper/` and use `file(GLOB)` to collect `*.cpp` files.
New test files are automatically picked up by CMake on re-configure.

### Test Categories

- `test_klipper_*.cpp` — Unit and integration tests
- `test_klipper_error_paths.cpp` — Error-path and failure injection tests
- `test_klipper_fuzz.cpp` — Fuzz/property-based tests (VLQ, MessageBlock, JSON)
- `test_helpers.hpp` — Shared test utilities (temp dirs, socket paths)
- `http/test_glaze_adapter.cpp` — HTTP server unit tests (GlazeAdapter, ResponseBuilder, JsonRpcDispatcher, WsSessionManager)

## Known Issues

- `LinuxCanHal.VcanLoopbackTest` is skipped when `vcan0` is not available
  (requires: `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan`)
- `ThermalIntegrationTest` tests take ~6 minutes total (real-time simulation)
- `test_klipper_tier_features.cpp` is the largest test file (1092 lines) and
  could be split for better parallelization

## Non-Newtonian Extrusion Compensation

Tether extends Klipper's pressure advance with non-Newtonian rheology models
(power-law, Cross-WLF) and flow-adaptive heater control. The flow-adaptive
heater controller uses a **three-state thermal model** (heater block → sensor →
melt zone) with a Luenberger observer that corrects the state estimate from
the real thermistor reading. See `docs/extrusion/` for full documentation.

### Build & test

```bash
# Build control-level extrusion tests
cmake --build build --target tether_control_extrusion_tests -j$(nproc)
./build/bin/tests/tether_control_extrusion_tests

# Build klipper-level extrusion compensation tests (requires TETHER_ENABLE_KLIPPER=1)
cmake --build build --target tether_klipper_tests -j$(nproc)
./build/bin/tests/tether_klipper_tests --gtest_filter='*ExtrusionCompensation*:*ExtrusionFlowTracker*:*ExtrusionInstance*'
```

### Key source files

- `include/tether/control/extrusion/` — Rheology models, PA models, thermal observer, flow-adaptive heater, deconvolution controllers
- `include/tether/klipper/motion/ExtrusionFlowTracker.hpp` — Shared flow tap
- `include/tether/klipper/motion/MotionTranslator.hpp` — PA offset application (3 models)
- `include/tether/klipper/objects/Thermal.hpp` — Heater flow-compensation hook

## LTI and LPV Deconvolution Controllers

Four deconvolution controllers for extrusion feedforward compensation:

- `LTIFrequencyDomainDeconvolver` — Baseline regularized spectral deconvolution (Tikhonov/Wiener)
- `OverlapAddLPVDeconvolver` — Gain-scheduled overlap-add for host-side planning
- `ARXLPVInverseFilter` — Time-domain IIR inverse for bare-metal MCU streaming
- `StateSpaceLPVInputEstimator` — State-space input estimation with Tikhonov-regularized matrix inversion

See `docs/extrusion/DeconvolutionControllers.md` and
`docs/extrusion/LPVDeconvolution.md` for full documentation and tuning guides.

### Klipper integration

The deconvolution controllers are fully integrated into the Klipper layer:

- **Config**: `[extruder]` keys `deconvolution_controller`, `deconvolution_enabled`,
  `deconvolution_lambda`, `lti_pad_to_power_of_two`, `overlap_add_block_size`,
  `overlap_add_overlap_ratio`, `arx_na`, `arx_nb`, `state_space_state_dim`,
  `state_space_input_dim`, `state_space_output_dim`
- **G-code**: `SET_DECONVOLUTION_CONTROLLER ENABLE=1 CONTROLLER=lti_freq LAMBDA=0.001`
- **Status object**: `deconvolution` exposes `controller`, `enabled`, `lambda`
- **Accessors**: `KlippyInstance::ltiDeconvolver()`, `overlapAddDeconvolver()`,
  `arxInverseFilter()`, `stateSpaceEstimator()`, `applyDeconvolutionSettings()`

## EtherCAT Slave Supervision and Automatic Recovery

The `SlaveSupervisor` monitors EtherCAT slaves for critical conditions and
automatically attempts recovery by forcing the slave to `INIT` and
re-initializing it from scratch.

### Key components

- `SlaveSupervisor` (`include/tether/ethercat/SlaveSupervisor.hpp`) —
  Detects critical conditions, orchestrates recovery with retry limiting,
  dispatches events to listeners, and manages per-slave suspension state.
- `ISlaveRecoveryHandler` — Interface for re-initializing a slave after
  forced INIT.  The application provides a handler that re-configures PDOs,
  mailbox, and transitions the slave back to OP.
- `DS402Master::DS402RecoveryHandler` — Built-in handler for CiA 402 drives
  that replays `configureDrive()` + `enableDrive()`.
- `RecoveryConfig` — Configures trigger sources, critical AL status codes,
  retry limits, delays, and whether to suspend all slaves during recovery.
- `IRecoveryEventListener` — Callback interface for recovery events
  (CriticalDetected, RecoveryStarted, RecoverySucceeded, RecoveryFailed,
  RecoveryGaveUp, SlaveSuspended, SlaveResumed).

### Trigger sources

- `CriticalTrigger::ALStatusCodes` — Error flag set with a critical AL
  status code (default set includes `SlaveNeedsInit`, `FatalSyncError`,
  `NoSyncError`, `SynchronizationError`, etc.)
- `CriticalTrigger::TransitionFailures` — State drop from SAFE_OP/OP to
  INIT/PRE_OP, or explicit `handleTransitionFailure()` calls
- `CriticalTrigger::AppInjected` — Application calls `markCritical()`
- `CriticalTrigger::All` — All of the above (default)

### Recovery flow

1. Critical condition detected → `CriticalDetected` event
2. Slave(s) suspended (PDO data not passed to motion controllers)
3. `ALResetController` forces slave to `INIT` (two-step AL reset with ack)
4. `ISlaveRecoveryHandler::reinitializeSlave()` called
5. On success: slave resumed, `RecoverySucceeded` event, attempt count reset
6. On failure: retry (up to `max_attempts`), then `RecoveryGaveUp` + `Failed`

### Usage with DS402Master

```cpp
// Enable automatic recovery for DS402 drives
RecoveryConfig cfg;
cfg.max_attempts = 3;
cfg.retry_delay_ms = 500;
cfg.stop_loop_during_recovery = false;
ds402.enableSlaveRecovery(cfg, drive_configs);

// Or use the supervisor directly
auto& sup = master.slaveSupervisor();
sup.configure(cfg);
sup.setRecoveryHandler(std::make_unique<MyHandler>());
sup.start();

// Mark a slave as critical from application code
sup.markCritical(slave_index, "Custom critical condition");

// Check suspension state in the motion loop (realtime-safe)
if (sup.isSlaveSuspended(slave_index)) {
    // Skip PDO processing for this slave
}
```

### Build & test

```bash
# Build the supervisor test
cmake --build build --target tether_ethercat_supervisor_tests -j$(nproc)

# Run tests
./build/bin/tests/tether_ethercat_supervisor_tests
```

### Key source files

- `include/tether/ethercat/SlaveSupervisor.hpp` — Public API
- `src/ethercat/SlaveSupervisor.cpp` — Implementation
- `src/profiles/cia402/DS402Master.cpp` — DS402 integration + recovery handler
- `tests/ethercat/test_slave_supervisor.cpp` — Unit tests (32 tests)

