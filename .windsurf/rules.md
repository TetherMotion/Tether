# Tether Build Agent Instructions

## Core Principle: Incremental Builds Only

**Never do a clean build unless every other option has been exhausted.** A full clean + reconfigure takes an extremely long time. Always prefer selective target builds.

## Build Escalation Ladder (use in this order)

1. **Selective target build** — build only the component library + its test that changed
2. **Selective downstream builds** — if a shared header changed, build all affected components selectively
3. **Full build** (`cmake --build build`) — only after selective builds and tests pass successfully
4. **Clean build** (`rm -rf build/ && reconfigure`) — **LAST RESORT ONLY** after 3 failed attempts with linker/cache errors, CMake stale-cache errors, or generated header mismatches

## Source-to-Target Mapping

| Source Directory | Component Library | Test Target |
|-----------------|-------------------|-------------|
| `src/platform/`, `src/logging/` | `tether_common` | `tether_platform_tests` |
| `src/hal/` | `tether_hal` | `tether_hal_tests` |
| `src/packetloggers/pcap/` | `tether_pcap` | `tether_pcap_tests` |
| `src/control/` | `tether_controls` | `tether_control_tests` |
| `src/gcode/` | `tether_gcode` | `tether_gcode_tests` |
| `src/export/` | `tether_export` | `tether_export_tests` |
| `src/replanner/` | `tether_motion_planner` | `tether_motion_planner_tests`, `tether_motion_replanner_core_tests`, `tether_motion_replanner_machine_tests`, `tether_motion_replanner_identification_tests` |
| `src/motion/`, `src/profiles/cia402/`, `src/identification/` | `tether_motion_control` | `tether_motion_tests`, `tether_model_identification_tests`, `tether_simulation_identification_tests` |
| `src/sii/`, `src/ethercat/ESIParser.cpp` | `tether_ethercat_common` | `tether_sii_tests`, `tether_ethercat_common_tests` |
| `src/ethercat/*.cpp`, `src/fmmu/`, `src/reset/`, `src/fsoe/`, `src/etg5000/`, `src/profiles/cia*/` (drive), `src/drives/` | `tether_ethercat_master` | `tether_profiles_tests`, `tether_fsoe_tests`, `tether_mailbox_tests` |
| `src/slave/` | `tether_ethercat_slave` | `tether_slave_tests`, `tether_slave_emulation_tests` |
| `src/simulation/` | `tether_simulation` | `tether_simulation_tests` |
| `src/destabilizer/` | `tether_destabilizer` | `tether_destabilizer_tests` |
| `src/io/` | `tether_io_protocol` | `tether_io_tests` |
| `tests/mocks/` | — | `tether_mocks_tests` |

### Special Test Targets (not tied to a single library)

| Target | Description |
|--------|-------------|
| `tether_dc_tests` | DC clock/distribution tests |
| `tether_motion_replanner_core_tests` | Motion replanner core tests |
| `tether_motion_replanner_machine_tests` | Motion replanner machine tester tests |
| `tether_motion_replanner_identification_tests` | Motion replanner system identification tests |
| `tether_header_tests` | Header compilation tests |
| `tether_fsoe_integration_tests` | FSoE integration tests |

## Selective Build Commands

### Build a single component library

```bash
cmake --build build --target tether_controls
```

### Build and run tests for a single module

```bash
# Example: control module
cmake --build build --target tether_control_tests
ctest --test-dir build --output-on-failure -R "^tether_control_tests$"
```

### Build and run tests for multiple related modules

```bash
cmake --build build --target tether_motion_planner_tests --target tether_motion_tests
ctest --test-dir build --output-on-failure -R "^tether_motion_(planner|)_tests$"
```

## Cross-Component Header Changes

If you modify a **shared header** in `include/tether/` (especially `include/tether/Tether.hpp`, `include/tether/platform/`, or `include/tether/hal/`), you must identify and build all downstream components that include it:

```bash
# Common header changed — build common + everything that depends on it
cmake --build build --target tether_common --target tether_platform_tests
# Then build downstream consumers selectively
cmake --build build --target tether_hal --target tether_controls --target tether_gcode --target tether_ethercat_common
```

## Full Build Rules

A full build (`cmake --build build`) is allowed **only** when:

- All selective builds for changed modules have succeeded
- All selective tests for changed modules have passed
- You need to verify nothing else broke before declaring the task done

## Clean Build Rules (LAST RESORT)

A clean build (`rm -rf build/ && cmake -S . -B build ...`) is allowed **only** when:

1. CMake explicitly reports a stale cache error
2. Generated headers (`TetherConfig.hpp`) are out of sync and `cmake --build` won't fix it
3. A selective rebuild fails 3 times with linker errors suggesting corrupted object files
4. You changed CMake options (e.g., `-DTETHER_BUILD_*`) and reconfiguration is required

**Before a clean build, try these first:**
- `cmake --build build --target clean` (clean only object files, keep config)
- Delete just the target's object files in `build/CMakeFiles/`
- Re-run `cmake -S . -B build` (reconfigure without deleting)

## Test Running Rules

- **Always run tests selectively first.** Use `ctest -R "^tether_<module>_tests$"` to run only the relevant module.
- Only run the full test suite (`ctest --test-dir build --output-on-failure`) after selective tests pass.
- The CI runs `ctest --test-dir build --output-on-failure -R "integration|ethercat|simulation"` for integration tests; you should too if you touched those areas.

## Build Script Note

The repo contains `./build.sh` but it **always builds all targets**. For selective builds, use `cmake --build build --target <target>` directly instead of `build.sh`.

## Common Component Dependencies

```
tether_common (root)
  └─ tether_hal
      └─ tether_ethercat_common
          ├─ tether_ethercat_master
          │   └─ depends on: tether_common, tether_hal, tether_ethercat_common, tether_controls, tether_motion_control
          └─ tether_ethercat_slave
              └─ depends on: tether_common, tether_hal, tether_ethercat_common
  └─ tether_pcap
  └─ tether_controls
      └─ tether_motion_control
          └─ depends on: tether_common, tether_controls
      └─ tether_destabilizer
          └─ depends on: tether_common, tether_simulation
  └─ tether_gcode
      └─ tether_export
          └─ depends on: tether_common, tether_gcode
      └─ tether_motion_planner
          └─ depends on: tether_common, tether_gcode, tether_export
```

When a lower-level component changes, build it first, then build its direct dependents selectively.
