# Tether

**Tether** is a modular C++23 framework for industrial motion control. It combines
a full EtherCAT master stack, CiA device profiles, Fail-Safe over EtherCAT (FSoE),
real-time motion planning, G-code interpretation, and a complete Klipper firmware
emulation layer into a set of composable libraries — targeting 3D printers, CNC
machines, and general multi-axis automation on Linux, embedded targets (ESP-IDF),
and simulation environments.

## Why Tether?

- **High-level, intent-based API** — describe *what* you want (move an axis,
  read a terminal, bring a drive to OP) and Tether handles the register bits,
  AL state transitions, SyncManager/FMMU setup, and mailbox protocols for you.
  No manual ESC register twiddling required.
- **Robust and proven** — built on a rigorously tested EtherCAT master stack
  with an extensive test suite (protocol, PDO, timing, supervisor, FSoE,
  profiles) plus fuzz/property tests, fault-injection tooling
  (`tether_destabilizer`), and slave emulation for hardware-free verification.
- **Embedded-systems capable** — the proven EtherCAT master stack runs from
  bare Linux userspace (raw sockets, no kernel modules) down to ESP-IDF
  targets (ESP32, S2, S3, C3, C6) via a first-class component manifest and HAL
  abstraction.
- **Liberally licensed** — dual Apache-2.0 / MPL-2.0. Use it in closed-source
  commercial products without a commercial license, and stay compatible with
  both GPLv2 and GPLv3 ecosystems.
- **Safety-aware by design** — developed by a certified machinery safety
  expert to facilitate safe and secure operation: Fail-Safe over EtherCAT
  (FSoE), safety terminals, watchdog handling, and automatic slave recovery
  are integral parts of the stack.
- **Integration testing built in** — from automatic CI-based memory-leak
  testing to hardware-emulation integration test frameworks: run masters
  against emulated slaves, emulated Klipper MCUs, and simulated plants so
  the full stack is verifiable without physical hardware.
- **Full-featured motion stack** — covers the entire pipeline: EtherCAT
  fieldbus, CiA drive profiles, FSoE safety, G-code parsing, time-optimal and
  jerk-constrained path planning (TOPPRA, S-curves, ReNURBS), kinematics,
  advanced extrusion compensation, and a complete Klipper-compatible firmware
  layer so stock frontends (Mainsail, Fluidd, Moonraker) work out of the box.
- **Composable and testable** — every component is an independent library with
  a HAL in between; run the full planning/control pipeline against simulated
  plants in CI, then deploy the same code to real hardware.

---

## Table of Contents

- [Why Tether?](#why-tether)
- [Feature Highlights](#feature-highlights)
- [Component Libraries](#component-libraries)
- [Repository Layout](#repository-layout)
- [Requirements](#requirements)
- [Building](#building)
- [Running Tests](#running-tests)
- [Examples](#examples)
- [Python Bindings](#python-bindings)
- [ESP-IDF Component](#esp-idf-component)
- [Toolchain Compatibility (`<format>`, `<print>`, `<expected>`)](#toolchain-compatibility-format-print-expected)
- [Documentation](#documentation)
- [License](#license)

---

## Feature Highlights

### EtherCAT Master

- **Complete master stack** — frame protocol (RAW/UDP encapsulation), mailbox
  protocols (CoE, EoE, FoE, VoE), state machine (INIT → PRE-OP → SAFE-OP → OP),
  SyncManager/FMMU configuration, and wire-format register access.
- **Distributed Clocks (DC)** — full DC support including drift compensation,
  sync-signal configuration, consistency checking, and reference-clock
  management (`DCManager`, `DCTypes`, `DCConfigurationValidation`).
- **Flexible PDO exchange** — fixed mappings, custom PDO mapping
  (`CustomPDOMapping`), logical addressing via `LogicalAddressManager`, and
  **partial LRW exchange** for process images larger than one Ethernet frame
  (`exchangeLRWSlice`).
- **SII EEPROM** — parsing of ESC SII contents (`ESIParser`, `CachedSIIReader`,
  `SIIManager`), with an `extract_esi` tool for XML ESI files.
- **Automatic slave recovery** — `SlaveSupervisor` detects critical conditions
  (AL status codes, transition failures, app-injected faults), forces the slave
  to `INIT`, and re-initializes it through a pluggable
  `ISlaveRecoveryHandler` with retry limiting, event listeners, and per-slave
  suspension so the motion loop stays realtime-safe.
- **Deterministic realtime loop** — `RealtimeLoop`, `CyclicTaskScheduler`,
  `RealtimeJitterMonitor`, and lock-free (`atomic_queue`) PDO queues.
- **EtherCAT-over-UDP encapsulation** — optional, for virtualized or routed
  networks (`TETHER_ENABLE_UDP_ENCAPSULATION`).

### CiA / ETG Device Profiles

First-class support for the CANopen-over-EtherCAT profile family:

| Profile | Description |
|---------|-------------|
| CiA 301 | Common communication profile |
| CiA 401 | Generic I/O modules |
| CiA 402 | Drive profile (state machine, modes of operation, homing) |
| CiA 404 | Weighing / torque measurement |
| CiA 405 | IEC 61131-3 programmable devices |
| CiA 406 | Encoders |
| CiA 408 | Fluid power / valves |
| CiA 410 | Inclinometers |
| CiA 417 | Lift control |
| CiA 430 | Energy metering |
| ETG 5000 | Modular device profile |

`DS402Master` provides high-level CiA 402 drive management — `configureDrive()`,
`enableDrive()`, homing, CSP/CSV/CST operation — and ships a built-in
`DS402RecoveryHandler` for automatic drive recovery.

### Fail-Safe over EtherCAT (FSoE)

Implementation of the ETG 5100 black-channel safety protocol:

- Master and slave endpoints (`FSoEMaster`, `FSoESlave`, `FSoEMasterConnection`)
- CRC handling (`FSoECRC`), including opt-in **native CRC resynchronization**
  for slaves joining mid-stream (see `docs/FSoECrcResync.md`)
- Slave emulator and rewriter for testing safety configurations
- Synapticon-specific helpers under `include/tether/fsoe/Synapticon`

### Vendor Device Drivers

Ready-made helpers for real hardware:

- **Servo drives** — AS715N, DynaDrive (ANYdrive), PBLR81FGF, Synapticon SOMANET,
  Nexcobot ESC211
- **Beckhoff terminals** — digital in/out (EL1xxx/EL200x), analog in/out,
  oversampling, PWM, pulse-train, stepper, DC motor, position, power-meter,
  safety, and serial-bridge terminals (`include/tether/Beckhoff/`)
- **Kinco RP20** modular I/O

### EtherCAT Slave Emulation

Build software EtherCAT slaves (`tether_ethercat_slave`) for testing masters,
CI pipelines, or virtual machines — including ETG 5000 modular slaves and
profile-aware emulation.

### Motion Planning & Replanning

- **G64-style continuous-path blending** with tangent/curvature verification,
  configurable deviation limits, and multiple transition strategies
- **Time-optimal path parameterization** — `BasicTOPPRA`,
  `JerkConstrainedTOPPRA`, and an analytical TOPPRA implementation
  (see `docs/motion/ToppraDerivation.md`, `AnalyticalTOPPRA.md`)
- **Velocity profilers** — trapezoidal, S-curve (`SCurveVelocityProfiler`),
  and snap-space profiling (`SnapSpaceVelocityProfiler`)
- **ReNURBS** curve geometry (`profile_renurbs`, `ReNURBS.md`)
- **Motion replanner** — online re-planning and machine identification
  (`docs/MotionReplanner.md`)
- **Kinematics** — Cartesian, delta, and rotary-delta printer kinematics,
  forward/inverse transforms

### G-code

A full lexer/parser/interpreter (`tether_gcode`) supporting:

- Standard motion commands, arcs (G2/G3), and blended continuous-path motion
- Marlin-style M-code handling (`MarlinMCodeHandler`)
- O-code subroutines (`GCodeOCodes`), variables, expressions
- Planning-segment building that feeds the motion planner

### Klipper Firmware Emulation (`tether_klipper`)

A complete, host-side reimplementation of Klipper so that stock frontends
(Mainsail, Fluidd, Moonraker clients) work against Tether:

- **Klipper wire protocol** — MCU message encoding, VLQ, command dispatch,
  multi-MCU management (`MultiMcuManager`), clock synchronization
- **Printer objects** — the full klippy object model: heaters, fans, extruders,
  bed mesh/leveling, probes, TMC drivers, LEDs, input shaper, firmware
  retraction, virtual sdcard, g-code macros, and more
- **G-code execution** — `GCodeExecutor`, `GCodeParser`, macro expansion,
  extended commands
- **Transport-agnostic server** — `KlippyServer` holds all business logic;
  `KlippyUdsServer` (Unix domain sockets, Moonraker-compatible) and
  `KlippyHttpServer` (Drogon-based HTTP/WebSocket for Mainsail/Fluidd) are thin
  transports over it
- **120+ Moonraker API endpoints** — JSON-RPC dispatch, WebSocket sessions,
  notification fan-out (see `docs/KlipperMoonrakerApi.md`)
- **Pressure advance & extrusion compensation** — classic PA plus
  non-Newtonian rheology models (power-law, Cross-WLF), flow-adaptive heater
  control with a three-state thermal model and Luenberger observer, and four
  LTI/LPV deconvolution feedforward controllers
  (see `docs/extrusion/`)

### Control & Identification

- **Controllers** — PID (classical + fractional), LQR/LQG, state-space,
  robust/QFT, learning, and composite controllers; Kalman and extended Kalman
  filters; parameter ramping
- **Autotuning** — classical relay/rule-based methods plus model-based tuning
- **System identification** — step response, frequency response, friction
  (incl. advanced nonlinear models), rigid-body dynamics, subspace methods,
  least squares, adaptive observers
- **Simulation** — a dynamical-system simulation engine with integrators and
  sensor/actuator models, enabling full plant-in-the-loop testing

### IO Protocol

SLIP-based parameter streaming protocol (`tether_io`) with binary struct
serialization, parameter exposers, registries, sessions, and serial/TCP/WebSocket
transports — used to expose live parameters to UIs (see
`examples/tether_ui_backend.cpp`, `docs/IOProtocol.md`).

---

## Component Libraries

CMake builds Tether as a set of component libraries. Each can be enabled or
disabled independently (`TETHER_BUILD_*` options):

| Library | Contents |
|---------|----------|
| `tether_common` | Shared utilities, types, configuration |
| `tether_hal` | Hardware abstraction (Ethernet, CAN, timers, threading, logging) |
| `tether_controls` | PID, LQR/LQG, Kalman, robust, fractional, learning controllers |
| `tether_autotuning` | Controller autotuning (classical + model-based) |
| `tether_gcode` | G-code lexer/parser/interpreter and planning segments |
| `tether_export` | CSV/SVG export utilities |
| `tether_motion_planner` | Blending, TOPPRA, velocity profiles, ReNURBS |
| `tether_motion_control` | Real-time motion control, CiA 402 motion logic |
| `tether_identification` | System identification suite |
| `tether_ethercat_common` | Common EtherCAT types |
| `tether_ethercat_master` | EtherCAT master (protocol, mailbox, DC, FMMU, SII, PDO) |
| `tether_cia_profiles` | CiA 301–430 profiles + ETG 5000 |
| `tether_device_drivers` | Vendor drivers (AS715N, DynaDrive, ESC211, RP20, Synapticon, Beckhoff) |
| `tether_fsoe` | FSoE safety protocol (ETG 5100) |
| `tether_ethercat_slave` | EtherCAT slave emulation |
| `tether_simulation` | Dynamical systems + simulation engine |
| `tether_destabilizer` | Adversarial/fault-injection testing helpers |
| `tether_io` | SLIP-based IO protocol |
| `tether_klipper` | Klipper protocol, klippy objects, UDS/HTTP servers |
| `tether_terminal_ui` | ncurses terminal UI for examples |

---

## Repository Layout

```
include/tether/     Public headers, organized by module
src/                Component implementations
tests/              Google Test suites (one binary per component)
examples/           70+ example programs (drives, terminals, Klipper, G-code)
docs/               Sphinx docs + standalone Markdown design documents
benchmarks/         Micro-benchmarks
python_bindings/    pybind11 bindings + `tether` Python package
tools/extract_esi/  ESI XML extraction utility
dependencies/       Vendored header-only deps (git submodules)
cmake/              CMake helpers (incl. Drogon compat find-modules)
web/                Web dashboard assets
scripts/            Helper scripts
```

---

## Requirements

- **CMake ≥ 3.16**
- **A C++23 compiler** — GCC ≥ 11, recent Clang, or MSVC.
  `std::format` / `std::print` / `std::expected` are auto-shimmed on older
  toolchains via vendored {fmt} 11.1.4 and tl::expected v1.3.1
  (see [Toolchain Compatibility](#toolchain-compatibility-format-print-expected)).
- **Git submodules** (required): `magic_enum`, `argparse`, `atomic_queue`,
  `glaze`, `eigen`, `fmt`, `expected`, `libSLIPspeed`

  ```bash
  git submodule update --init --recursive
  ```

- **Optional system dependencies:**
  - `libdrogon-dev` + `libjsoncpp-dev` — native Klipper HTTP/WebSocket server
    (`tether_klipper_http`)
  - `ncurses` — terminal-UI examples
  - SocketCAN — Klipper CAN transport (`TETHER_ENABLE_KLIPPER_CAN`)
  - pybind11 / Python 3 dev headers — Python bindings
  - `vcan0` (SocketCAN virtual interface) — optional for one CAN loopback test

Hardware examples that talk to real EtherCAT devices need a NIC and typically
root or `CAP_NET_RAW` (raw sockets). Everything else — simulation, Klipper
emulation, planning, control — runs without special privileges.

---

## Building

> **Parallelism:** use `-j8` max. Higher parallelism can OOM the build machine.

```bash
# Fetch dependencies
git submodule update --init --recursive

# Configure (default: all components, tests, examples, Python bindings)
cmake -B build

# Or with the preset
cmake --preset default

# Build everything
cmake --build build -j8

# Install
cmake --install build --prefix /opt/tether
```

### Common configuration

```bash
# Klipper support is on by default (TETHER_BUILD_KLIPPER=ON);
# HTTP/WebSocket server additionally needs Drogon + jsoncpp:
cmake -B build -DTETHER_ENABLE_KLIPPER_HTTP=ON

# Minimal EtherCAT-only build:
cmake -B build \
  -DTETHER_BUILD_KLIPPER=OFF -DTETHER_BUILD_GCODE=OFF \
  -DTETHER_BUILD_MOTION_PLANNER=OFF -DTETHER_BUILD_EXAMPLES=OFF \
  -DTETHER_BUILD_TESTS=OFF -DTETHER_BUILD_PYTHON_BINDINGS=OFF

# Static instead of shared libraries:
cmake -B build -DTETHER_BUILD_SHARED_LIBS=OFF -DTETHER_BUILD_STATIC_LIBS=ON

# Coverage instrumentation:
cmake -B build -DTETHER_ENABLE_COVERAGE=ON
```

### Selected CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `TETHER_BUILD_TESTS` | ON | Google Test suites |
| `TETHER_BUILD_EXAMPLES` | ON | Example programs |
| `TETHER_BUILD_BENCHMARKS` | OFF | Micro-benchmarks |
| `TETHER_BUILD_PYTHON_BINDINGS` | ON | pybind11 `tether` package |
| `TETHER_BUILD_KLIPPER` | ON | Klipper emulation layer |
| `TETHER_ENABLE_KLIPPER_HTTP` | auto | Drogon HTTP/WS server (needs Drogon + jsoncpp) |
| `TETHER_ENABLE_KLIPPER_CAN` | OFF | Klipper SocketCAN transport |
| `TETHER_ENABLE_KLIPPER_JSON` | ON | Glaze JSON I/O for klipper |
| `TETHER_ENABLE_PRESSURE_ADVANCE` | ON | Compile in extrusion pressure advance |
| `TETHER_BUILD_FSOE` | ON | FSoE safety protocol |
| `TETHER_BUILD_ETHERCAT_SLAVE` | ON | Slave emulation |
| `TETHER_BUILD_SIMULATION` | ON | Simulation engine |
| `TETHER_BUILD_DESTABILIZER` | ON | Fault-injection test helpers |
| `TETHER_ENABLE_SII` | ON | SII EEPROM support |
| `TETHER_ENABLE_UDP_ENCAPSULATION` | OFF | EtherCAT-over-UDP |
| `TETHER_ENABLE_CIA3xx/4xx` | ON | Per-profile toggles (301…430 + ETG5000) |
| `TETHER_DRIVE_*` / `TETHER_DRIVER_BECKHOFF` | ON | Per-driver toggles |
| `TETHER_BUILD_SHARED_LIBS` / `STATIC_LIBS` | ON / OFF | Library flavor |

See `CMakeLists.txt` for the full list (~50 options).

---

## Running Tests

Tests are Google Test binaries under `build/bin/tests/`, one per component:

```bash
# Everything via CTest
ctest --test-dir build -j8

# A specific suite
./build/bin/tests/tether_ethercat_master_tests
./build/bin/tests/tether_motion_planner_tests
./build/bin/tests/tether_klipper_tests --gtest_filter='KlippyUdsTest.*'

# Fuzz / property tests
./build/bin/tests/tether_klipper_tests --gtest_filter='*Fuzz*:*Property*'
```

Notable suites: `tether_ethercat_{master,protocol,pdo,timing,supervisor}_tests`,
`tether_profiles_cia402_*_tests`, `tether_fsoe{,_integration}_tests`,
`tether_gcode_*_tests`, `tether_motion_planner_*_tests`,
`tether_control_{core,robust,qft,extrusion,autotuning}_tests`,
`tether_simulation_*_tests`, `tether_klipper{,_http}_tests`.

Known caveats:

- `ThermalIntegrationTest` runs a real-time simulation (~6 min) — exclude with
  `--gtest_filter='-ThermalIntegrationTest.*'`
- `LinuxCanHal.VcanLoopbackTest` is skipped unless `vcan0` exists:
  `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan`

---

## Examples

`examples/` contains 70+ runnable programs. A few entry points:

| Example | What it shows |
|---------|---------------|
| `list_slaves.cpp` | Scanning an EtherCAT network |
| `real_cia402_example.cpp` | CiA 402 drive bring-up |
| `cia402_drive.cpp`, `synapticon_cst*.cpp` | Vendor drive control |
| `beckhoff_*` | Beckhoff terminal I/O, PWM, oversampling, safety |
| `ethercat_dump_sii.cpp` | SII EEPROM dump |
| `klipper_http.cpp` | Full Klipper emulation + Mainsail/Fluidd HTTP/WS server |
| `klipper_loopback_demo.cpp` | Klipper over a loopback transport |
| `gcode_generation.cpp`, `g64_demo_export.cpp` | G-code → blended toolpaths |
| `machine_tester_*.cpp` | Stepwise machine commissioning |
| `slave_emulator.cpp` | Software EtherCAT slave |
| `web_dashboard_example.cpp` | Live-parameter web dashboard via `tether_io` |

Build with `cmake --build build --target <example_name>` (targets match the
source basename).

---

## Python Bindings

`python_bindings/` builds a `tether` Python package (pybind11) covering
EtherCAT master/common/slave, CiA 401/402, FSoE, controls, G-code, motion
control, motion planning, Klipper, and export.

```bash
cmake -B build -DTETHER_BUILD_PYTHON_BINDINGS=ON
cmake --build build -j8

cd python_bindings
pip install -e .        # development mode
# or: pip wheel . --no-deps
```

---

## ESP-IDF Component

Tether ships an `idf_component.yml` manifest and a `Kconfig`, so it can be used
as an ESP-IDF component (IDF ≥ 5.0; targets: ESP32, S2, S3, C3, C6). When
`ESP_PLATFORM` is set, the top-level CMake defers to
`CMakeLists_component.txt`, which builds a reduced component set suitable for
embedded targets (ESP32 HAL pieces live under `include/tether/hal/`). See
`docs/CrossCompiling.md` for cross-compilation notes.

---

## Toolchain Compatibility (`<format>`, `<print>`, `<expected>`)

Tether uses `std::format`, `std::print`, and `std::expected` throughout. CMake
detects each header with `check_include_file_cxx` and, where the native header
is missing, activates a thin shim that re-exports {fmt} / tl::expected into
`namespace std`:

| Header | Fallback | Shim | Native since |
|--------|----------|------|--------------|
| `<format>` | {fmt} 11.1.4 | `include/tether/fmt_shim/format_shim/format` | GCC 13 |
| `<print>` | {fmt} 11.1.4 | `include/tether/fmt_shim/print_shim/print` | GCC 15 |
| `<expected>` | tl::expected 1.3.1 | `include/tether/expected_shim/expected` | GCC 12 |

The `<format>` and `<print>` shims are activated **independently** — on GCC
13/14 (native `<format>`, no `<print>`) only the `<print>` shim is added, so
libstdc++ internals that `#include <format>` keep resolving natively. No source
changes are needed on any toolchain.

---

## Documentation

Design and reference docs live in `docs/` (Sphinx-ready via `docs/index.rst`,
plus a Doxygen `Doxyfile`):

- **EtherCAT** — `docs/PDOModes.md`, `docs/EXTRACT_ESI.md`, `docs/FSoECrcResync.md`
- **Motion** — `docs/motion/` (architecture, blending, TOPPRA derivation,
  profiler selection, ReNURBS), `KINEMATIC_MODELS.md`, `docs/SnapSpaceVelocityProfiler.md`
- **Klipper** — `docs/KlipperArchitecture.md`, `KlipperProtocol.md`,
  `KlipperGcodeCommands.md`, `KlipperMoonrakerApi.md`, `KlipperPrinterObjects.md`,
  `KlipperTerminology.md`
- **Extrusion compensation** — `docs/extrusion/` (non-Newtonian pressure
  advance, rheology models, flow-adaptive temperature control, deconvolution
  controllers, LPV deconvolution)
- **Platform** — `docs/HAL_PORTING_GUIDE.md`, `docs/CrossCompiling.md`,
  `docs/IOProtocol.md`, `docs/IOProtocolWireFormat.md`, `docs/ModelIdentification.md`,
  `docs/MotionReplanner.md`

`AGENTS.md` contains a project guide with build/test commands and architecture
notes for AI agents and contributors.

---

## License

Tether is **dual-licensed** under the **Apache License 2.0** and the
**Mozilla Public License 2.0** — you may choose either. The combination was
selected for compatibility with both GPLv2 and GPLv3. See `LICENSE.md` for the
full text and rationale.
