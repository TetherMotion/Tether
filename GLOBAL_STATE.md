# GLOBAL STATE — `g_` variables inventory (Tether)

This document inventories global variables that use the `g_` prefix across the Tether component, explains where global state is concentrated, the risk/impact, and recommended next steps to reduce global coupling.

**Note:** Most global state has been refactored to instance-based implementations. This document now only tracks remaining platform-specific globals.

---

## Executive summary ✅
- **Global state refactoring completed:** Transport, PDO, DC, FoE, Fault detection, Packet router, and Write-verify have all been migrated to instance-based implementations.
- **Remaining globals:** Only platform-specific globals remain (ESP32 filesystem state, ESP32 spinlock, and host build stubs).
- **Risk level:** Low — remaining globals are isolated to specific platform implementations and are acceptable.

---

## Platform-specific globals (low risk)

### ESP32 Platform
- `g_fs_initialized` — tracks whether the filesystem (LittleFS/SPIFFS) has been mounted
  - File: `src/ethercat/platform_esp32.cpp`
  - Scope: file-static, managed exclusively through `fs_init()`/`fs_deinit()`
  - Why it's acceptable: Platform-specific singleton with proper lifecycle control, isolated to ESP32 builds only

- `g_spinlock` — FreeRTOS spinlock for critical section entry/exit
  - File: `src/ethercat/platform_esp32.cpp`
  - Scope: file-static, compile-time constant initializer (`portMUX_INITIALIZER_UNLOCKED`)
  - Why it's acceptable: Effectively a constant initializer, no mutable global state requiring lifecycle management

### Host Build Stubs
- `g_eth_handle` — opaque handle pointer for network interface (stub only)
  - Files: `src/ethercat/host_stubs.cpp`, `tests/mocks/ethercat_platform_stubs.cpp`
  - Scope: namespace-scoped in VoE, FoE, EoE namespaces
  - Why it's acceptable: Build-time stubs only, used when EtherCAT types are not available on host builds

- `g_src_mac[6]` — source MAC address array (stub only)
  - Files: `src/ethercat/host_stubs.cpp`, `tests/mocks/ethercat_platform_stubs.cpp`
  - Scope: namespace-scoped in VoE, FoE, EoE namespaces
  - Why it's acceptable: Build-time stubs only, zero-initialized placeholders for host builds

---

## Historical refactoring (completed)

The following global state has been successfully refactored to instance-based implementations:

- **Transport / Raw API:** `g_network_iface`, `g_src_mac`, `g_aprd_cb`, `g_apwr_cb`, `g_aprd_responses` → moved to `EtherCATMaster` instance
- **PDO / Sync manager:** `g_slave_configs`, `g_pdo_mapping`, `g_pdo_stats`, `g_pdo_initialized`, `g_slave_count` → moved to `PDOManager` instance
- **Distributed Clock (DC):** `g_singleton_dc`, `g_dc_ctx`, `g_dc_timer`, `g_dc_mutex`, `g_dc_sync_pending` → moved to `DCManager`/`EtherCATDC` instance
- **FoE:** `g_initialized`, `g_foe_thread`, `g_running`, `g_stats`, `g_request_queue` → moved to `FoEManager` instance
- **Fault detection:** `g_slave_faults`, `g_slave_count`, `g_initialized`, `g_fault_callback` → moved to `FaultDetector` instance
- **Packet router:** `g_packet_router` → replaced by `TransactionRouter` instance
- **Write-verify:** `g_config`, `g_enabled`, `g_stats` → moved to `WriteVerifier` instance

---

## Where to inspect code (quick links)
- ESP32 platform: `src/ethercat/platform_esp32.cpp`
- Host stubs: `src/ethercat/host_stubs.cpp`, `tests/mocks/ethercat_platform_stubs.cpp`

---

## Notes & references
- Historical global state inventory is documented in `docs/ETHERCAT_INVENTORY.md` (may contain outdated references).
- Refactoring was completed as part of the migration to instance-based architectures for better testability and reduced coupling.