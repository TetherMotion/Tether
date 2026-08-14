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
