# GLOBAL STATE — `g_` variables inventory (Tether)

This document inventories global variables that use the `g_` prefix across the Tether component, explains where global state is concentrated, the risk/impact, and recommended next steps to reduce global coupling.

**Note:** All global state has been removed from the codebase. The Tether component now uses exclusively instance-based implementations.

---

## Executive summary ✅
- **Global state refactoring completed:** All global state has been removed from the codebase.
- **Current state:** Zero global variables using the `g_` prefix remain in the codebase.
- **Architecture:** All subsystems now use instance-based implementations (EtherCATMaster, PDOManager, SDOManager, DCManager, FoEManager, FaultDetector, WriteVerifier, etc.).
- **Platform-specific code:** ESP32 platform code uses runtime checks (stat()) for filesystem state and FreeRTOS macros for critical sections, eliminating the need for global state.

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
- **Platform-specific (ESP32):** `g_fs_initialized` → replaced with runtime stat() check; `g_spinlock` → replaced with FreeRTOS taskENTER_CRITICAL/taskEXIT_CRITICAL macros
- **Host build stubs:** `g_eth_handle`, `g_src_mac` → removed (unused stubs)

---

## Notes & references
- Historical global state inventory is documented in `docs/ETHERCAT_INVENTORY.md` (may contain outdated references).
- Refactoring was completed as part of the migration to instance-based architectures for better testability and reduced coupling.