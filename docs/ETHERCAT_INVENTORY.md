# EtherCAT Subsystem — Comprehensive Inventory

> Generated for refactoring planning. Covers all free functions, classes, global/static state, source files, test files, and examples in the EtherCAT subsystem under `Tether/`.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [All Source Files](#2-all-source-files)
3. [All Free Functions](#3-all-free-functions)
4. [All Class Definitions](#4-all-class-definitions)
5. [All Global / Static Variables](#5-all-global--static-variables)
6. [All Test Files](#6-all-test-files)
7. [All Example Files](#7-all-example-files)
8. [Refactoring Notes](#8-refactoring-notes)

---

## 1. Architecture Overview

The EtherCAT subsystem now has a **layered runtime architecture**:

| Layer | Pattern | Responsibility |
|-------|---------|----------------|
| **EtherCAT transport/runtime** | `EtherCATMaster` + sub-manager classes (`PDOManager`, `SDOManager`, `DCManager`, `FaultDetector`, `FoEManager`, `VoEManager`, `EoEManager`) | Bus transport, discovery, AL state transitions, mailbox, PDO/SDO/DC runtime |
| **DS402 orchestration** | `DS402Master` + `CiA402Drive` | Drive configuration, DS402 startup/shutdown, operating mode setup, per-drive state machine |
| **Example/application layer** | Example-specific motion policy and typed PDO loops | Application behavior only; no raw EtherCAT setup or SOEM transport |

**Key implication for refactoring**: `EtherCATMaster` is the owning EtherCAT API, `DS402Master` sits above it, and examples should stay at the application layer. Legacy free-function paths may still exist internally, but the public direction is instance-owned transport and descriptive APIs.

**Platform duality**: Code uses `#ifndef UNIT_TEST_HOST` / `#ifdef UNIT_TEST_HOST` guards to provide ESP32-specific (FreeRTOS, gptimer, esp_eth) and host-native (std::thread, std::mutex) code paths.

---

## 2. All Source Files

### 2.1 Headers — `include/tether/ethercat/` (18 files)

| File | Lines | Primary contents |
|------|-------|-----------------|
| `EtherCATMaster.hpp` | 322 | `EtherCATMaster` class, `Config` struct |
| `EtherCATRaw.hpp` | ~400 | Top-level legacy API (`HandleRxFrame`, `StartMasterTask`, etc.), includes all sub-headers |
| `EtherCATPDO.hpp` | 753 | `PDO` namespace: `PDOMapping` class, `PDOEntry`, `SlaveConfig`, `SyncManagerConfig`, all PDO free functions |
| `EtherCATSDO.hpp` | 639 | `sdo` namespace: SDO request/response types, async queue API, sync helpers, type-safe read/write, `SDOManager` |
| `EtherCATDC.hpp` | 318 | `DC` namespace: `DCConfig`, `DCContext`, `DCState`, DC free functions, `DCManager` |
| `EtherCATFoE.hpp` | 531 | `FoE` namespace: file transfer protocol types and free functions (feature-gated) |
| `EtherCATVoE.hpp` | 406 | `VoE` namespace: vendor-specific protocol types and free functions (feature-gated) |
| `EtherCATEoE.hpp` | 628 | `EoE` namespace: Ethernet-over-EtherCAT types and free functions (feature-gated) |
| `EtherCATFaultDetection.hpp` | 369 | `ALStatusCode`, `CiA402ErrorCode` enums, `FaultDetector` class, fault free functions |
| `EtherCATConfig.hpp` | 777 | Feature enable macros, buffer/timing config, compile-time knobs |
| `EtherCATTypes.hpp` | 534 | Core types: `NetworkInterface`, `Command` enum, `RxDatagram`, wire-format packed structs |
| `EtherCATReset.hpp` | 770 | `ResetLevel` enum, `ALState` enum, `ALControl` namespace constants |
| `EtherCATWriteVerify.hpp` | 245 | `Verify` namespace: write-and-verify functions |
| `EtherCATRetry.hpp` | 391 | `Raw` namespace: `RetryPolicy`, `StoredDatagram`, `RetryExecutor` class, datagram builders |
| `EtherCATDCConsistency.hpp` | 271 | `DC` namespace: consistency check types and functions |
| `ConditionalPacketRouter.hpp` | ~400 | `ConditionalPacketRouter` class, `PacketFilter`, `WaitResult`, `WaiterEntry` |
| `EtherCATSlaveEmulator.hpp` | ~500 | `SlaveEmulator`, `NetworkEmulator` classes, CiA402 `DriveState` |
| `EtherCATPlatform.hpp` | ~250 | `PlatformFile` abstract class, filesystem free functions |

### 2.2 Internal header — `src/ethercat/raw/internal.hpp` (802 lines)

Contains wire-format structures, register address constants, byte-order helpers, transport function declarations, mailbox structures, SII/EEPROM helpers, CoE SDO internal functions. This is the **implementation-private** header shared across raw/*.cpp files.

### 2.3 Source — `src/ethercat/raw/` (20 files)

| File | Lines | Purpose |
|------|-------|---------|
| `internal.hpp` | 802 | Internal header (see above) |
| `transport.cpp` | 433 | Transport layer: `send_raw_frame`, `send_single_datagram`, `ec_apwr`, `ec_aprd`, response wait |
| `runtime.cpp` | ~300 | Queue management: `alloc_idx`, `flush_rx_queue`, `log_dedup`, `adp_for_slave_index` |
| `master.cpp` | ~400 | Master task: `discover_slaves`, `set_preop_and_confirm`, `master_task`, `start_master_task` |
| `EtherCATMaster.cpp` | 847 | `EtherCATMaster` class implementation |
| `coe_sdo.cpp` | 535 | CoE SDO upload/download implementation |
| `dc_init.cpp` | 502 | DC initialization (ESP32 gptimer, mutexes) |
| `dc_sync.cpp` | 688 | DC sync protocol: propagation delay, system time offset, sync signals |
| `dc_realtime.cpp` | 233 | DC realtime task with timer ISR |
| `pdo_api.cpp` | ~200 | PDO public API dispatch: `pdo_send_rxpdo`, `pdo_receive_txpdo`, `pdo_exchange_all` |
| `pdo_logical.cpp` | 447 | FMMU-based LRW/LRD/LWR exchange: `pdo_exchange_lrw`, `pdo_exchange_separate` |
| `pdo_transfer.cpp` | 233 | Position/configured/broadcast PDO transfer |
| `sync_manager.cpp` | 601 | Sync Manager configuration, `PDOMapping` methods, `pdo_init`/`pdo_deinit` |
| `sdo_async.cpp` | 462 | Async SDO worker thread: `sdo_init`, `sdo_queue_request`, `sdo_processing_task` |
| `eeprom_sii.cpp` | ~200 | SII/EEPROM read: `ec_eeprom_read_u32_ap`, `sii_read_string`, `configure_mailbox_from_sii` |
| `FoE.cpp` | 222 | FoE protocol (mostly stubbed) |
| `VoE.cpp` | ~100 | VoE protocol (all stubs) |
| `EoE.cpp` | ~100 | EoE protocol (all stubs) |
| `SubManagers.cpp` | ~150 | Out-of-line sub-manager methods (PDOManager, SDOManager, DCManager) |
| `host_shims.cpp` | ~50 | Host stub for legacy master-task startup |

### 2.4 Source — `src/ethercat/` (non-raw, 8 files)

| File | Lines | Purpose |
|------|-------|---------|
| `ConditionalPacketRouter.cpp` | 499 | `ConditionalPacketRouter` implementation + global instance |
| `EtherCATRetry.cpp` | 390 | `RetryExecutor`, datagram builder functions (`buildAPRD`, `buildAPWR`, `buildFPRD`, `buildFPWR`, `buildBRD`, `buildBWR`, `buildLRW`, `buildLRD`, `buildLWR`) |
| `EtherCATSlaveEmulator.cpp` | 1033 | `SlaveEmulator` + `NetworkEmulator` + CiA402 drive simulation |
| `EtherCATWriteVerify.cpp` | 201 | Write-and-verify implementation |
| `EtherCATFaultDetection.cpp` | 480 | Fault detection implementation |
| `EtherCATDCConsistency.cpp` | 535 | DC consistency checks |
| `host_stubs.cpp` | ~200 | Host-build stubs (when `!TETHER_ENABLE_ETHERCAT && !TETHER_COMPILE_MASTER`) |
| `dc_time_source.cpp` | ~50 | DC time source default implementation |
| `platform_esp32.cpp` | ~250 | ESP32 platform: LittleFS/SPIFFS filesystem, `PlatformFile` impl |

### 2.5 CiA402 Profile — `include/tether/profiles/cia402/` (14 headers)

| File | Key classes / contents |
|------|----------------------|
| `CiA402Drive.hpp` | `CiA402Drive`, `DriveManager`, `DriveState` enum |
| `CiA402StateMachine.hpp` | CiA402 state machine logic |
| `CiA402Config.hpp` | Drive configuration types |
| `MotionController.hpp` | `CiA402Axis`, `MotionController` |
| `MotionProfile.hpp` | Motion profile (trapezoid, S-curve) |
| `PIDController.hpp` | `PIDController`, `CascadedPIDController` |
| `MotorModel.hpp` | `MotorModel` |
| `AdvancedMotorModel.hpp` | `AdvancedMotorModel` |
| `DriveBackend.hpp` | `DriveBackend` abstract class |
| `EtherCATBackend.hpp` | `EtherCATBackend : DriveBackend`, `EtherCATBackendFactory` |
| `HomingHandler.hpp` | `HomingHandler` |
| `HomingModes.hpp` | Homing mode definitions |
| `ElectronicGearing.hpp` | Electronic gearing |
| `MultiAxisPath.hpp` | Multi-axis coordinated path |

### 2.6 CiA402 Profile — `src/profiles/cia402/` (20 source files)

| File | Purpose |
|------|---------|
| `CiA402Drive.cpp` | Drive logic |
| `CiA402DrivePDO.cpp` | Drive PDO integration |
| `CiA402DriveStateMachine.cpp` | Drive state machine |
| `CiA402StateMachine.cpp` | Generic CiA402 state machine |
| `MotionController.cpp` | Motion controller |
| `MotionControllerAxis.cpp` | Axis control |
| `MotionControllerPath.cpp` | Path control |
| `MotionProfile.cpp` | Profile generation |
| `PIDController.cpp` | PID controller |
| `MotorModel.cpp` | Motor model |
| `AdvancedMotorModelCore.cpp` | Advanced motor model core |
| `AdvancedMotorModelFactory.cpp` | Factory methods |
| `AdvancedMotorModelPhysics.cpp` | Physics simulation |
| `EtherCATBackend.cpp` | EtherCAT drive backend |
| `HomingHandler.cpp` | Homing handler |
| `HomingFactory.cpp` | Homing mode factory |
| `HomingMethods.cpp` | Individual homing methods |
| `HomingStateMachine.cpp` | Homing state machine |
| `ElectronicGearing.cpp` | Gearing |
| `MultiAxisPath.cpp` | Multi-axis path |

---

## 3. All Free Functions

### 3.1 `EtherCAT::Raw::` namespace (transport & master)

**Note:** Most of these functions have been refactored to instance-based implementations in `EtherCATMaster`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `set_network_interface(NetworkInterface)` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `network_interface() → NetworkInterface&` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `send_raw_frame(iface, data, len)` | transport.cpp | **Deprecated** — use `EtherCATMaster::sendRawFrame()` |
| `set_aprd_cb(fn)` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `set_apwr_cb(fn)` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `push_aprd_response(RxDatagram)` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `clear_aprd_responses()` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `set_src_mac(mac[6])` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `get_src_mac(out[6])` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `send_single_datagram(mac, cmd, idx, adp, ado, data, len, wkc)` | transport.cpp | **Deprecated** — use `EtherCATMaster::sendSingleDatagram()` |
| `wait_for_response_idx(idx, timeout, out)` | transport.cpp | **Deprecated** — use `EtherCATMaster::waitForResponseIdx()` |
| `wait_for_response_ado(ado, timeout, out)` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `ec_apwr(mac, adp, ado, data, len, timeout)` | transport.cpp | **Deprecated** — use `EtherCATMaster::writeRegister()` |
| `ec_apwr_u16(mac, adp, ado, value, timeout)` | transport.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `ec_aprd(mac, adp, ado, data, len, timeout)` | transport.cpp | **Deprecated** — use `EtherCATMaster::readRegister()` |
| `alloc_idx() → uint8_t` | runtime.cpp | **Deprecated** — use `EtherCATMaster::allocIdx()` |
| `reset_idx()` | runtime.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `rx_queue() → MessageQueue*` | runtime.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `txpdo_rx_queue() → MessageQueue*` | runtime.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `ensure_rx_queue()` | runtime.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `flush_rx_queue()` | runtime.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `log_dedup_key(tag, msg) → string` | runtime.cpp | Logging helper (stateless) |
| `log_dedup(key, tag, msg)` | runtime.cpp | Logging helper (stateless) |
| `adp_for_slave_index(idx) → uint16_t` | runtime.cpp | Pure computation (stateless) |
| `get_discovered_slave_count() → uint16_t` | master.cpp | **Deprecated** — use `EtherCATMaster::getDiscoveredSlaveCount()` |
| `build_scan_frame(mac, buf) → size_t` | master.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `discover_slaves(mac) → uint16_t` | master.cpp | **Deprecated** — use `EtherCATMaster::discoverSlaves()` |
| `set_preop_and_confirm(mac, nslaves)` | master.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `master_task(arg)` | master.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `start_master_task(iface, mac)` | master.cpp | **Deprecated** — use `EtherCATMaster::start()` |
| `coe_sdo_upload(mac, adp, index, subindex, ...)` | coe_sdo.cpp | **Deprecated** — use `EtherCATMaster::coeSdoUpload()` |
| `coe_sdo_download(mac, adp, index, subindex, ...)` | coe_sdo.cpp | **Deprecated** — use `EtherCATMaster::coeSdoDownload()` |
| `host_to_le16(v) → uint16_t` | internal.hpp | Byte-order helper (inline, stateless) |
| `host_to_le32(v) → uint32_t` | internal.hpp | Byte-order helper (inline, stateless) |
| `le16_to_host(v) → uint16_t` | internal.hpp | Byte-order helper (inline, stateless) |
| `le32_to_host(v) → uint32_t` | internal.hpp | Byte-order helper (inline, stateless) |
| `buildAPRD(idx, slave_pos, ado, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildAPWR(idx, slave_pos, ado, data, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildFPRD(idx, cfg_addr, ado, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildFPWR(idx, cfg_addr, ado, data, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildBRD(idx, ado, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildBWR(idx, ado, data, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildLRW(idx, logical_addr, data, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildLRD(idx, logical_addr, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `buildLWR(idx, logical_addr, data, len) → StoredDatagram` | EtherCATRetry.cpp | Pure builder (stateless) |
| `sii_read_string(mac, adp, string_num, out, cap) → bool` | eeprom_sii.cpp | **Deprecated** — use `EtherCATMaster::siiReadString()` |
| `configure_mailbox_from_sii(mac, adp, ...)` | eeprom_sii.cpp | **Deprecated** — use `EtherCATMaster` instance |

### 3.2 `EtherCAT::PDO::` namespace

**Note:** These functions have been refactored to instance-based implementations in `PDOManager`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `pdo_init()` | sync_manager.cpp | **Deprecated** — use `PDOManager::init()` |
| `pdo_deinit()` | sync_manager.cpp | **Deprecated** — use `PDOManager::deinit()` |
| `pdo_get_mapping() → PDOMapping*` | sync_manager.cpp | **Deprecated** — use `PDOManager::getMapping()` |
| `pdo_get_slave_configs() → SlaveConfig*` | sync_manager.cpp | **Deprecated** — use `PDOManager::getSlaveConfigs()` |
| `pdo_configure_slave_sms(mac, slave_index)` | sync_manager.cpp | **Deprecated** — use `PDOManager::configureSlaveSync()` |
| `pdo_configure_all_slave_sms(mac, count)` | sync_manager.cpp | **Deprecated** — use `PDOManager::configureAllSlaveSync()` |
| `pdo_finalize_mapping(slave_index)` | sync_manager.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_get_stats() → PDOStats` | sync_manager.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_reset_stats()` | sync_manager.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_send_rxpdo(mac, mapping, entry_index)` | pdo_api.cpp | **Deprecated** — use `PDOManager::sendRxPDO()` |
| `pdo_receive_txpdo(mac, mapping, entry_index)` | pdo_api.cpp | **Deprecated** — use `PDOManager::receiveTxPDO()` |
| `pdo_exchange_all(mac, mapping)` | pdo_api.cpp | **Deprecated** — use `PDOManager::exchangeAll()` |
| `pdo_exchange_lrw(mac, mapping)` | pdo_logical.cpp | **Deprecated** — use `PDOManager::exchangeLRW()` |
| `pdo_get_lrw_stats() → LRWStats` | pdo_logical.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_set_separate_mode(bool)` | pdo_logical.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_get_separate_mode() → bool` | pdo_logical.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_exchange_separate(mac, mapping)` | pdo_logical.cpp | **Deprecated** — use `PDOManager::exchangeSeparate()` |
| `pdo_get_separate_stats()` | pdo_logical.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_set_physical_mode(bool)` | pdo_logical.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_get_physical_mode() → bool` | pdo_logical.cpp | **Deprecated** — use `PDOManager` instance |
| `pdo_exchange_physical(mac, mapping)` | pdo_transfer.cpp | **Deprecated** — use `PDOManager::exchangePhysical()` |
| `pdo_get_physical_stats()` | pdo_transfer.cpp | **Deprecated** — use `PDOManager` instance |
| `send_rxpdo_position(mac, entry)` | pdo_transfer.cpp | **Deprecated** — internal helper |
| `recv_txpdo_position(mac, entry)` | pdo_transfer.cpp | **Deprecated** — internal helper |
| `send_rxpdo_configured(mac, entry)` | pdo_transfer.cpp | **Deprecated** — internal helper |
| `recv_txpdo_configured(mac, entry)` | pdo_transfer.cpp | **Deprecated** — internal helper |
| `send_rxpdo_broadcast(mac, entry)` | pdo_transfer.cpp | **Deprecated** — internal helper |
| `recv_txpdo_broadcast(mac, entry)` | pdo_transfer.cpp | **Deprecated** — internal helper |

### 3.3 `EtherCAT::sdo::` namespace

**Note:** These functions have been refactored to instance-based implementations in `SDOManager`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `sdo_init()` | sdo_async.cpp | **Deprecated** — use `SDOManager::init()` |
| `sdo_deinit()` | sdo_async.cpp | **Deprecated** — use `SDOManager::deinit()` |
| `sdo_configure_slave_mailbox(slave, wr_addr, wr_len, rd_addr, rd_len)` | sdo_async.cpp | **Deprecated** — use `SDOManager::configureSlaveMailbox()` |
| `sdo_configure_network(slave_count)` | sdo_async.cpp | **Deprecated** — use `SDOManager::configureNetwork()` |
| `sdo_configure_network(mac, slave_count)` | sdo_async.cpp | **Deprecated** — use `SDOManager::configureNetwork()` |
| `sdo_queue_request(req) → uint32_t` | sdo_async.cpp | **Deprecated** — use `SDOManager::queueRequest()` |
| `sdo_is_complete(id) → bool` | sdo_async.cpp | **Deprecated** — use `SDOManager::isComplete()` |
| `sdo_get_response(id) → SDOResponse` | sdo_async.cpp | **Deprecated** — use `SDOManager::getResponse()` |
| `sdo_cancel_request(id)` | sdo_async.cpp | **Deprecated** — use `SDOManager` instance |
| `sdo_pending_count() → size_t` | sdo_async.cpp | **Deprecated** — use `SDOManager::pendingCount()` |
| `sdo_read_sync(mac, slave, idx, sub, buf, len, timeout) → SDOResponse` | EtherCATSDO.hpp | **Deprecated** — use `SDOManager::readSync()` |
| `sdo_write_sync(mac, slave, idx, sub, data, len, timeout) → SDOResponse` | EtherCATSDO.hpp | **Deprecated** — use `SDOManager::writeSync()` |
| `sdo_read_u8/u16/u32/i32(...)` | EtherCATSDO.hpp | **Deprecated** — use `SDOManager` instance |
| `sdo_write_u8/u16/u32/i32(...)` | EtherCATSDO.hpp | **Deprecated** — use `SDOManager` instance |
| `sdo_set_emergency_callback(cb)` | EtherCATSDO.hpp | **Deprecated** — use `SDOManager` instance |
| `sdo_get_last_emergency()` | EtherCATSDO.hpp | **Deprecated** — use `SDOManager` instance |
| `sdo_abort_code_str(code) → const char*` | sdo_async.cpp | Pure lookup (stateless) |
| `sdo_processing_task()` | sdo_async.cpp | **Deprecated** — internal worker loop |
| `execute_sdo_request(req) → SDOResponse` | sdo_async.cpp | **Deprecated** — internal |

### 3.4 `EtherCAT::DC::` namespace

**Note:** These functions have been refactored to instance-based implementations in `DCManager`/`EtherCATDC`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `dc_init(config)` | dc_init.cpp | **Deprecated** — use `DCManager::init()` |
| `dc_start()` | dc_init.cpp | **Deprecated** — use `DCManager::start()` |
| `dc_stop()` | dc_init.cpp | **Deprecated** — use `DCManager::stop()` |
| `dc_get_state() → DCState` | dc_init.cpp | **Deprecated** — use `DCManager::getState()` |
| `dc_get_stats() → DCLoopStats` | dc_init.cpp | **Deprecated** — use `DCManager::getStats()` |
| `dc_get_context() → DCContext*` | dc_init.cpp | **Deprecated** — use `DCManager::getContext()` |
| `dc_force_sync()` | dc_init.cpp | **Deprecated** — use `DCManager::forceSync()` |
| `dc_set_pdo_enabled(bool)` | dc_init.cpp | **Deprecated** — use `DCManager` instance |
| `dc_is_pdo_enabled() → bool` | dc_init.cpp | **Deprecated** — use `DCManager` instance |
| `dc_slave_supported(mac, adp) → bool` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_get_slave_offset(mac, adp) → int64_t` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_read_sync_config(mac, adp, ...)` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_reconfigure_sync(mac, adp, ...)` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_read_slave_capabilities(mac, adp)` | dc_sync.cpp | **Deprecated** — internal |
| `dc_calc_propagation_delay(mac, adp)` | dc_sync.cpp | **Deprecated** — internal |
| `dc_write_system_time_offset(mac, adp, offset)` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_update_sync_start_time(mac, adp)` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_configure_sync_signals(mac, adp, ...)` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_send_sync_frame(mac)` | dc_sync.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_timer_callback(...)` | dc_realtime.cpp | **Deprecated** — internal ISR callback |
| `dc_realtime_task(arg)` | dc_realtime.cpp | **Deprecated** — internal FreeRTOS task |
| `dc_get_master_time_with_epoch()` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_format_time(ns) → string` | EtherCATDCConsistency.cpp | Pure formatting (stateless) |
| `dc_read_slave_state(mac, adp) → SlaveDCState` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_run_consistency_checks(mac, count) → DCConsistencyReport` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_check_sync0_start_time(...)` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_check_propagation_delay(...)` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_check_system_time_offset(...)` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_check_sync_cycle_times(...)` | EtherCATDCConsistency.cpp | **Deprecated** — use `EtherCATDC` instance |
| `dc_log_slave_state(state)` | EtherCATDCConsistency.cpp | Logging only (stateless) |
| **Weak C symbols** | dc_init.cpp | Platform time source overrides |
| `ecdc_get_master_time_ns() → uint64_t` | dc_init.cpp | Platform time source (can be overridden) |
| `ecdc_init_time_source()` | dc_init.cpp / dc_time_source.cpp | Platform time source init |
| `ecdc_deinit_time_source()` | dc_init.cpp / dc_time_source.cpp | Platform time source deinit |

### 3.5 `EtherCAT::FoE::` namespace

**Note:** These functions have been refactored to instance-based implementations in `FoEManager`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `foe_init()` | FoE.cpp | **Deprecated** — use `FoEManager::init()` |
| `foe_deinit()` | FoE.cpp | **Deprecated** — use `FoEManager::deinit()` |
| `foe_upload_file(...)` | FoE.cpp | **Deprecated** — use `FoEManager::uploadFile()` |
| `foe_download_file(...)` | FoE.cpp | **Deprecated** — use `FoEManager::downloadFile()` |
| `foe_upload_memory(...)` | FoE.cpp | **Deprecated** — use `FoEManager::uploadMemory()` |
| `foe_download_memory(...)` | FoE.cpp | **Deprecated** — use `FoEManager::downloadMemory()` |
| `foe_get_status(handle)` | FoE.cpp | **Deprecated** — use `FoEManager` instance |
| `foe_get_stats()` | FoE.cpp | **Deprecated** — use `FoEManager::getStats()` |

### 3.6 `EtherCAT::VoE::` namespace

**Note:** These functions are stubs and have not been fully implemented.

| Function | File | Notes |
|----------|------|-------|
| `voe_init()` | VoE.cpp | Stub (not implemented) |
| `voe_deinit()` | VoE.cpp | Stub (not implemented) |
| `voe_transact(...)` | VoE.cpp | Stub (not implemented) |
| `voe_send(...)` | VoE.cpp | Stub (not implemented) |
| `voe_queue_request(...)` | VoE.cpp | Stub (not implemented) |
| `voe_register_handler(...)` | VoE.cpp | Stub (not implemented) |
| `voe_get_stats()` | VoE.cpp | Stub (not implemented) |

### 3.7 `EtherCAT::EoE::` namespace

**Note:** These functions are stubs and have not been fully implemented.

| Function | File | Notes |
|----------|------|-------|
| `eoe_init()` | EoE.cpp | Stub (not implemented) |
| `eoe_deinit()` | EoE.cpp | Stub (not implemented) |
| `eoe_configure_slave(...)` | EoE.cpp | Stub (not implemented) |
| `eoe_set_ip(...)` | EoE.cpp | Stub (not implemented) |
| `eoe_send_frame(...)` | EoE.cpp | Stub (not implemented) |
| `eoe_is_link_up(...)` | EoE.cpp | Stub (not implemented) |
| `eoe_register_frame_callback(...)` | EoE.cpp | Stub (not implemented) |
| `eoe_get_stats()` | EoE.cpp | Stub (not implemented) |

### 3.8 `EtherCAT::Verify::` namespace

**Note:** These functions have been refactored to instance-based implementations in `WriteVerifier`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `set_config(config)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::setConfig()` |
| `get_config() → WriteVerifyConfig&` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::config()` |
| `set_enabled(bool)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::setEnabled()` |
| `is_enabled() → bool` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::isEnabled()` |
| `get_stats() → WriteVerifyStats&` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::stats()` |
| `reset_stats()` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::resetStats()` |
| `log_stats()` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::logStats()` |
| `ec_apwr_verify(mac, adp, ado, data, len, timeout)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::apwrVerify()` |
| `ec_apwr_verify_u16(...)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::apwrVerifyU16()` |
| `ec_apwr_verify_u32(...)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::apwrVerifyU32()` |
| `ec_apwr_verify_u64(...)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::apwrVerifyU64()` |
| `ec_bwr_verify(...)` | EtherCATWriteVerify.cpp | **Deprecated** — use `WriteVerifier::bwrVerify()` |

### 3.9 Fault Detection free functions

**Note:** These functions have been refactored to instance-based implementations in `FaultDetector`. The following table documents legacy free functions that may still exist for backward compatibility.

| Function | File | Notes |
|----------|------|-------|
| `fault_init(count)` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::init()` |
| `fault_shutdown()` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::shutdown()` |
| `fault_poll(mac, slave_idx)` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::poll()` |
| `fault_poll_all(mac, count)` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::pollAll()` |
| `fault_get_state(slave_idx) → SlaveFaultState` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::getState()` |
| `fault_any_active() → bool` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::anyActive()` |
| `fault_clear(slave_idx)` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::clear()` |
| `fault_set_callback(cb)` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::setCallback()` |
| `fault_diagnose(mac, slave_idx) → string` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::diagnose()` |
| `fault_diagnose_no_sync(slave_idx) → string` | EtherCATFaultDetection.cpp | **Deprecated** — use `FaultDetector::diagnoseNoSync()` |
| `getALStatusCodeName(code) → const char*` | EtherCATFaultDetection.cpp | Pure lookup (stateless) |
| `getCiA402ErrorCodeName(code) → const char*` | EtherCATFaultDetection.cpp | Pure lookup (stateless) |
| `al_status_get_state_name(state) → const char*` | EtherCATFaultDetection.cpp | Pure lookup (stateless) |

### 3.10 Top-level `EtherCAT::` namespace free functions

| Function | File | Notes |
|----------|------|-------|
| `HandleRxFrame(data, len)` | EtherCATRaw.hpp / master.cpp | **Deprecated** — use `EtherCATMaster::handleRxFrame()` |
| `StartMasterTask(...)` | master.cpp / host_shims.cpp | **Deprecated** — use `EtherCATMaster::start()` |
| `StartMasterTask(NetworkInterface, mac)` | master.cpp | **Deprecated** — use `EtherCATMaster::start()` |
| `requestSlaveApplicationLayerState(slave_index, state_code)` | host_stubs.cpp | Host-only minimal stub |
| `transitionSlaveToPreOperational(slave_index)` | host_stubs.cpp | Host-only minimal stub |
| `readSlaveApplicationLayerState(slave_index, state_code)` | host_stubs.cpp | Host-only minimal stub |
| `ConfigureWatchdogs(mac, adp, ...)` | host_stubs.cpp | Host-only minimal stub |
| `DisableWatchdogs(mac, adp)` | host_stubs.cpp | Host-only minimal stub |
| `ReadWatchdogStatus(mac, adp)` | host_stubs.cpp | Host-only minimal stub |

### 3.11 Packet Router free functions

**Note:** These functions have been refactored to instance-based implementations. The global packet router has been replaced by `TransactionRouter`.

| Function | File | Notes |
|----------|------|-------|
| `getPacketRouter() → ConditionalPacketRouter&` | ConditionalPacketRouter.cpp | **Deprecated** — use `EtherCATMaster::getPacketRouter()` |
| `initPacketRouter() → bool` | ConditionalPacketRouter.cpp | **Deprecated** — use `EtherCATMaster` instance |
| `shutdownPacketRouter()` | ConditionalPacketRouter.cpp | **Deprecated** — use `EtherCATMaster` instance |

### 3.12 Platform free functions (`EtherCAT::Platform::`)

| Function | File | Notes |
|----------|------|-------|
| `file_exists(path) → bool` | platform_esp32.cpp | Platform filesystem API (stateless) |
| `file_stat(path, info) → bool` | platform_esp32.cpp | Platform filesystem API (stateless) |
| `file_delete(path) → bool` | platform_esp32.cpp | Platform filesystem API (stateless) |
| `file_rename(old, new) → bool` | platform_esp32.cpp | Platform filesystem API (stateless) |
| `dir_create(path) → bool` | platform_esp32.cpp | Platform filesystem API (stateless) |
| `fs_init() → bool` | platform_esp32.cpp | Uses `g_fs_initialized` (platform-specific) |

---

## 4. All Class Definitions

### 4.1 Core EtherCAT Classes

#### `EtherCATMaster` — [EtherCATMaster.hpp](include/tether/ethercat/EtherCATMaster.hpp)

**Real implementation** in `raw/EtherCATMaster.cpp` (847 lines). This is the most complex class.

| Category | Members |
|----------|---------|
| **Config** | `Config` nested struct (interface, src_mac, auto_discover, ...) |
| **Lifecycle** | `start(Config)`, `stop()`, `isRunning()` |
| **Transport** | `sendRawFrame()`, `sendSingleDatagram()`, `writeRegister()`, `readRegister()` — **real implementations** using instance-owned `iface_`, `src_mac_`, `packet_router_` |
| **Wait helpers** | `waitForResponseIdx()`, `waitForResponseAdo()` — **real**, use `packet_router_` |
| **Frame handling** | `handleRxFrame()`, `parseEtherCATFrame()` — **real** |
| **Index alloc** | `allocIdx()` — **real**, instance-level counter |
| **Discovery** | `discoverSlaves()`, `getDiscoveredSlaveCount()` — **real** |
| **AL State** | `requestSlaveApplicationLayerState()`, `readSlaveApplicationLayerState()`, `transitionSlaveToPreOperational()` — **real instance APIs** |
| **Watchdog** | `configureWatchdogs()`, `disableWatchdogs()` — **delegate** |
| **SII/EEPROM** | `siiReadString()` — **delegate** to `Raw::sii_read_string()` |
| **CoE/SDO** | `coeSdoUpload()`, `coeSdoDownload()` — **delegate** to `Raw::coe_sdo_upload/download()` |
| **Sub-managers** | `pdo()`, `sdo()`, `dc()`, `foe()`, `voe()`, `eoe()`, `faults()` — return references to `unique_ptr` sub-managers |
| **Getters** | `getSrcMac()`, `getNetworkInterface()`, `getPacketRouter()` |
| **Private state** | `iface_`, `src_mac_[6]`, `running_`, `next_idx_`, `slave_count_`, `packet_router_` (instance-owned), `master_thread_`, unique_ptrs to all sub-managers |

**Wrapper vs Real**:
- Transport/wait/frame/discovery/index = **real** (instance state)
- AL state/watchdog/SII/CoE = **delegate** to global free functions
- Sub-manager accessors = **accessor** only

#### `PDOManager` — [EtherCATPDO.hpp](include/tether/ethercat/EtherCATPDO.hpp)

**Thin wrapper**. All methods in `SubManagers.cpp` delegate to `PDO::` free functions.

| Method | Delegates to |
|--------|-------------|
| `init()` | `PDO::pdo_init()` |
| `deinit()` | `PDO::pdo_deinit()` |
| `getMapping()` | `PDO::pdo_get_mapping()` |
| `getSlaveConfigs()` | `PDO::pdo_get_slave_configs()` |
| `configureSlaveSync(idx)` | `PDO::pdo_configure_slave_sms(master_.getSrcMac(), idx)` |
| `configureAllSlaveSync(count)` | `PDO::pdo_configure_all_slave_sms(...)` |
| `exchangeAll()` | `PDO::pdo_exchange_all(master_.getSrcMac(), ...)` |
| `exchangeLRW()` | `PDO::pdo_exchange_lrw(...)` |
| `exchangeSeparate()` | `PDO::pdo_exchange_separate(...)` |
| `exchangePhysical()` | `PDO::pdo_exchange_physical(...)` |
| `sendRxPDO(entry_index)` | `PDO::pdo_send_rxpdo(...)` |
| `receiveTxPDO(entry_index)` | `PDO::pdo_receive_txpdo(...)` |

#### `SDOManager` — [EtherCATSDO.hpp](include/tether/ethercat/EtherCATSDO.hpp)

**Thin wrapper**. Delegates to `sdo::` free functions.

| Method | Delegates to |
|--------|-------------|
| `init()` | `sdo::sdo_init()` |
| `deinit()` | `sdo::sdo_deinit()` |
| `configureNetwork(count)` | `sdo::sdo_configure_network(master_.getSrcMac(), count)` |
| `queueRequest(req)` | `sdo::sdo_queue_request(req)` |
| `isComplete(id)` | `sdo::sdo_is_complete(id)` |
| `getResponse(id)` | `sdo::sdo_get_response(id)` |
| `readSync(...)` | `sdo::sdo_read_sync(...)` |
| `writeSync(...)` | `sdo::sdo_write_sync(...)` |

#### `DCManager` — [EtherCATDC.hpp](include/tether/ethercat/EtherCATDC.hpp)

**Thin wrapper**. Delegates to `DC::` free functions.

| Method | Delegates to |
|--------|-------------|
| `init(config)` | `DC::dc_init(config)` |
| `start()` | `DC::dc_start()` |
| `stop()` | `DC::dc_stop()` |
| `getState()` | `DC::dc_get_state()` |
| `getStats()` | `DC::dc_get_stats()` |
| `getContext()` | `DC::dc_get_context()` |
| `forceSync()` | `DC::dc_force_sync()` |

#### `FoEManager` / `VoEManager` / `EoEManager`

Declared in respective headers. **Thin wrappers** delegating to `FoE::`/`VoE::`/`EoE::` free functions.

#### `FaultDetector` — [EtherCATFaultDetection.hpp](include/tether/ethercat/EtherCATFaultDetection.hpp)

**Thin wrapper** delegating to fault detection free functions.

| Method | Delegates to |
|--------|-------------|
| `init(count)` | `fault_init(count)` |
| `shutdown()` | `fault_shutdown()` |
| `poll(mac, idx)` | `fault_poll(mac, idx)` |
| `pollAll(mac, count)` | `fault_poll_all(mac, count)` |
| `getState(idx)` | `fault_get_state(idx)` |
| `anyActive()` | `fault_any_active()` |
| `clear(idx)` | `fault_clear(idx)` |
| `setCallback(cb)` | `fault_set_callback(cb)` |

#### `ConditionalPacketRouter` — [ConditionalPacketRouter.hpp](include/tether/ethercat/ConditionalPacketRouter.hpp)

**Real implementation** (499 lines). This class owns its state.

| Method | Real/Wrapper |
|--------|-------------|
| `init()` | **Real** — initializes mutex, waiter slots |
| `shutdown()` | **Real** — wakes all waiters, cleans up |
| `routePacket(RxDatagram)` | **Real** — matches incoming datagrams to waiting threads |
| `waitForPacket(filter, buf, size, timeout)` | **Real** — blocks thread until match or timeout |
| `preRegisterWaiter(filter, buf, size)` | **Real** — pre-register without blocking |
| `waitForPreRegistered(slot, timeout)` | **Real** — wait on pre-registered slot |
| `hasWaiters() / waiterCount()` | **Real** |
| `getStats()` | **Real** |
| Private: `matchesFilter()`, `findFreeSlot()`, `activateSlot()`, `releaseSlot()` | **Real** |
| **Instance state**: `m_waiters[kMaxWaiters]`, `m_initialized`, `m_shutdown`, `m_stats`, `m_global_mutex` | |

#### `RetryExecutor` — [EtherCATRetry.hpp](include/tether/ethercat/EtherCATRetry.hpp)

**Real implementation**. Owns reference to router and send function.

| Method | Type |
|--------|------|
| `execute(request, filter, buf, size, policy)` | **Real** — retry loop with backoff |
| Private: `router_`, `send_func_`, `stats_` | Instance state |

#### `PDOMapping` — [EtherCATPDO.hpp](include/tether/ethercat/EtherCATPDO.hpp)

**Real implementation** (methods in sync_manager.cpp).

| Method | Notes |
|--------|-------|
| `add_rxpdo(slave, buf, size, pdo_idx, mode)` | |
| `add_txpdo(slave, buf, size, pdo_idx, mode)` | |
| `add_broadcast_rxpdo(buf, size, offset)` | |
| `add_broadcast_txpdo(buf, size, offset)` | |
| `set_slave_configured_address(slave, addr)` | |
| `get_entry(idx) / get_entry_mut(idx)` | |
| `clear()` | |
| `entry_count() / total_rxpdo_bytes() / total_txpdo_bytes()` | |
| Private: `m_entries[kMaxPDOEntries]`, `m_entry_count`, `m_slave_configured_addrs[]` | |

### 4.2 Slave Emulator Classes

#### `SlaveEmulator` — [EtherCATSlaveEmulator.hpp](include/tether/ethercat/EtherCATSlaveEmulator.hpp)

**Real implementation** (1033 lines). Emulates an EtherCAT slave for testing.

Contains nested types: `ALStatus`, `SIIConfig`, `SMConfig`, `PDOEntry`, `PDOConfig`, `SyncManager`, `FMMU`, `DCState`, `ODEntry`, `ErrorInjection`.

#### `NetworkEmulator` — same header

**Real implementation**. Manages multiple `SlaveEmulator` instances.

Contains `NetworkStats` struct.

#### `CiA402::DriveState` — inside `EtherCATSlaveEmulator.hpp`

CiA402 drive simulation (status word, control word state machine, motion simulation).

### 4.3 CiA402 Profile Classes

| Class | Header | Notes |
|-------|--------|-------|
| `CiA402Drive` | CiA402Drive.hpp | Full drive implementation with state machine, PDO, SDO |
| `DriveManager` | CiA402Drive.hpp | Manages multiple drives |
| `CiA402Axis` | MotionController.hpp | Single axis control |
| `MotionController` | MotionController.hpp | Multi-axis motion controller |
| `PIDController` | PIDController.hpp | PID control loop |
| `CascadedPIDController` | PIDController.hpp | Cascaded (position→velocity→torque) PID |
| `MotorModel` | MotorModel.hpp | Basic motor simulation |
| `AdvancedMotorModel` | AdvancedMotorModel.hpp | Physics-based motor model |
| `DriveBackend` | DriveBackend.hpp | Abstract interface for drive communication |
| `EtherCATBackend` | EtherCATBackend.hpp | `DriveBackend` implementation using EtherCAT |
| `EtherCATBackendFactory` | EtherCATBackend.hpp | Factory for EtherCATBackend |
| `HomingHandler` | HomingHandler.hpp | Homing procedure handler |

### 4.4 Support Classes

| Class | Header | Notes |
|-------|--------|-------|
| `PlatformFile` | EtherCATPlatform.hpp | Abstract file interface |
| `SDO` (compat) | EtherCATSDO.hpp | Backward-compatible SDO class (wraps free functions) |
| `SDO` (sdo:: namespace) | EtherCATSDO.hpp | Another compat shim |

### 4.5 Structs (data-only, non-class)

| Struct | Header |
|--------|--------|
| `NetworkInterface` | EtherCATTypes.hpp |
| `RxDatagram` | EtherCATTypes.hpp |
| `EthernetHeader` (packed) | EtherCATTypes.hpp |
| `FrameHeader` (packed) | EtherCATTypes.hpp |
| `DatagramHeader` (packed) | EtherCATTypes.hpp |
| `PDOEntry` | EtherCATPDO.hpp |
| `SlaveConfig` | EtherCATPDO.hpp |
| `SyncManagerConfig` | EtherCATPDO.hpp |
| `PDOStats` | EtherCATPDO.hpp |
| `SDORequest` | EtherCATSDO.hpp |
| `SDOResponse` | EtherCATSDO.hpp |
| `DCConfig` | EtherCATDC.hpp |
| `DCContext` | EtherCATDC.hpp |
| `DCLoopStats` | EtherCATDC.hpp |
| `SlaveTimeInfo` | EtherCATDC.hpp |
| `SlaveDCState` | EtherCATDCConsistency.hpp |
| `ConsistencyCheckResult` | EtherCATDCConsistency.hpp |
| `DCConsistencyReport` | EtherCATDCConsistency.hpp |
| `RetryPolicy` | EtherCATRetry.hpp |
| `StoredDatagram` | EtherCATRetry.hpp |
| `RetryResult` | EtherCATRetry.hpp |
| `RetryStats` | EtherCATRetry.hpp |
| `WriteVerifyConfig` | EtherCATWriteVerify.hpp |
| `WriteVerifyResult` | EtherCATWriteVerify.hpp |
| `WriteVerifyStats` | EtherCATWriteVerify.hpp |
| `ManufacturerFault` | EtherCATFaultDetection.hpp |
| `SlaveFaultState` | EtherCATFaultDetection.hpp |
| `PacketFilter` | ConditionalPacketRouter.hpp |
| `WaitResult` | ConditionalPacketRouter.hpp |
| `WaiterSync` | ConditionalPacketRouter.hpp |
| `WaiterEntry` | ConditionalPacketRouter.hpp |
| `FileInfo` | EtherCATPlatform.hpp |

---

## 5. All Global / Static Variables

**Note:** All global state has been removed from the codebase. The Tether component now uses exclusively instance-based implementations.

---

## Historical Global State (Removed)

The following global state has been successfully refactored to instance-based implementations and no longer exists:

- **Transport layer:** `g_network_iface`, `g_aprd_cb`, `g_apwr_cb`, `g_aprd_responses`, `g_src_mac` → moved to `EtherCATMaster` instance
- **Runtime queue:** `s_rx_dg_queue`, `s_txpdo_rx_queue`, `s_next_idx`, etc. → moved to instance state
- **Master task:** `s_discovered_slave_count`, `s_master_thread`, `s_master_running` → moved to `EtherCATMaster` instance
- **PDO:** `g_slave_configs`, `g_pdo_mapping`, `g_pdo_stats`, `g_pdo_initialized`, `g_slave_count` → moved to `PDOManager` instance
- **PDO exchange:** `s_lrw_stats`, `s_separate_stats`, `s_use_separate_commands`, etc. → moved to instance state
- **SDO:** `s_queue`, `s_worker_thread`, `s_shutdown_requested`, `s_slave_mbx[]`, etc. → moved to `SDOManager` instance
- **DC:** `g_dc_ctx`, `g_dc_timer`, `g_dc_mutex`, `g_dc_event`, `g_dc_sync_pending` → moved to `DCManager`/`EtherCATDC` instance
- **FoE:** `g_initialized`, `g_foe_thread`, `g_running`, `g_stats`, `g_transfers[]`, `g_request_queue` → moved to `FoEManager` instance
- **Fault detection:** `g_slave_faults`, `g_slave_count`, `g_initialized`, `g_fault_callback` → moved to `FaultDetector` instance
- **Write-verify:** `g_config`, `g_enabled`, `g_stats` → moved to `WriteVerifier` instance
- **Packet router:** `g_packet_router` → replaced by `TransactionRouter` instance
- **Platform-specific (ESP32):** `g_fs_initialized` → replaced with runtime stat() check; `g_spinlock` → replaced with FreeRTOS taskENTER_CRITICAL/taskEXIT_CRITICAL macros
- **Host build stubs:** `g_eth_handle`, `g_src_mac` → removed (unused stubs)

---

## 6. All Test Files

### 6.1 `tests/sii/` — Main EtherCAT tests

| File | What it tests |
|------|--------------|
| `aprd_sequence_tests.cpp` | APRD sequence logic |
| `raw_transport_tests.cpp` | Transport layer |
| `raw_eeprom_tests.cpp` | EEPROM / SII read |
| `edge_raw_tests.cpp` | Edge cases in raw API |
| `more_raw_tests.cpp` | Additional raw API tests |
| `test_ethercat_packets.cpp` | Packet construction / parsing |
| `test_EtherCATReset.cpp` | Reset / AL state |
| `test_EtherCATRetry.cpp` | Retry logic |
| `test_ConditionalPacketRouter.cpp` | Packet router |
| `test_EtherCATFaultDetection.cpp` | Fault detection |
| `fault_detection_tests.cpp` | More fault detection tests |
| `write_verify_tests.cpp` | Write-and-verify |
| `slave_emulator_tests.cpp` | Slave emulator |
| `host_stubs_and_dc_tests.cpp` | Host stubs + DC |
| `dc_time_source_tests.cpp` | DC time source |
| `dc_time_source_default_tests.cpp` | Default DC time source |
| `test_sii_parser.cpp` | SII parser |
| `SIIParserCategoryTests.cpp` | SII category parsing |
| `SIIReaderTests.cpp` | SII reader |
| `zero_coverage_header_tests.cpp` | Coverage for untested headers |
| `zero_coverage_headers_more.cpp` | More coverage tests |
| `zero_coverage_small_tests2.cpp` | Additional coverage |
| `foe_tests.cpp` | FoE protocol |

### 6.2 `tests/mailbox/`

| File | What it tests |
|------|--------------|
| `coe_sdo_tests.cpp` | CoE SDO upload/download |
| `sdo_async_tests.cpp` | Async SDO queue/worker |

### 6.3 `tests/profiles/cia402/`

| File | What it tests |
|------|--------------|
| `test_cia402_protocol.cpp` | CiA402 drive protocol |

### 6.4 `tests/slave_emulation/`

| File | What it tests |
|------|--------------|
| `test_ETG5000ModularDevice.cpp` | ETG5000 modular device profile |

### 6.5 `tests/control/`

Motion/control tests (not fully enumerated in directory listing).

### 6.6 `tests/hal/`

HAL abstraction tests.

### 6.7 `tests/mocks/`

| File | Purpose |
|------|---------|
| `MockHAL.cpp` | Mock HAL implementation |
| `MockHAL.hpp` | Mock HAL header |
| `MockSDOHelper.hpp` | Mock SDO helper |
| `ethercat_platform_stubs.cpp` | Platform stubs for tests |
| `ethercat_raw_stubs.cpp` | Raw API stubs for tests |

### 6.8 `tests/fsoe/`

| File | What it tests |
|------|--------------|
| `test_FSoEConnection.cpp` | FSoE (Functional Safety over EtherCAT) |

### 6.9 `tests/motion/`

3 test files covering motion/trajectory tests.

### 6.10 `tests/export/`

Empty directory.

---

## 7. All Example Files

### 7.1 EtherCAT-specific examples (`examples/`)

| File | Description |
|------|-------------|
| `as715n_sine_motion.cpp` | Sine motion on AS715N drive (ESP32) |
| `as715n_sine_motion_native.cpp` | Same, native/host build |
| `pblr81fgf_sine_motion_native.cpp` | Copy of AS715N example for PBLR81FGF drive |
| `cia402_drive.cpp` | CiA402 drive example |
| `real_cia402_example.cpp` | Real hardware CiA402 example |
| `virtual_cia402_example.cpp` | Virtual/simulated CiA402 |
| `dual_instance_linux.cpp` | Dual EtherCATMaster instances on Linux |
| `simple_slave.cpp` | Simple slave emulator example |
| `LinuxHostMain.cpp` | Linux host main entry point |

### 7.2 Other examples

| File | Description |
|------|-------------|
| `fifo_communication.cpp` | FIFO communication |
| `gcode_*.cpp` | G-code processing examples (multiple files) |
| `machine_tester_*.cpp` | Machine tester utilities |
| `trajectory_analysis.cpp` | Trajectory analysis |
| `heatmap_generation.cpp` | Heatmap visualization |

---

## 8. Refactoring Notes

### 8.1 Global State Census Summary

| Module | # Global/Static vars | Severity |
|--------|---------------------|----------|
| Transport | 7 | **Critical** — `g_network_iface`, `g_src_mac` used by everything |
| Runtime/Queues | 5 | **High** — `s_next_idx`, queues |
| Master Task | 3 | **High** — `s_master_thread`, `s_discovered_slave_count` |
| PDO (sync_manager) | 5 | **High** — `g_slave_configs`, `g_pdo_mapping`, `g_pdo_stats` |
| PDO (logical/transfer) | 6 | Medium — stats and mode flags |
| SDO (async) | 8 | **High** — worker thread, queues, mailbox config |
| CoE SDO | 1 | Low |
| DC | 5 | **High** — context, timer, task |
| FoE | 6 | Medium (mostly stubs) |
| Fault Detection | 4 | Medium |
| Write-Verify | 3 | Low |
| Packet Router | 1 | Low (but it's the global instance) |

**Total: ~54 global/static variables** across the EtherCAT subsystem.

### 8.2 Functions That Are Pure (No Global State)

These can be moved to instance methods or utility functions without any state migration:

- All `build*()` datagram builders in EtherCATRetry.cpp
- `adp_for_slave_index()`, `log_dedup_key()`
- `sdo_abort_code_str()`
- `dc_format_time()`, `dc_log_slave_state()`
- `getALStatusCodeName()`, `getCiA402ErrorCodeName()`, `al_status_get_state_name()`
- Byte-order helpers (`host_to_le16/32`, `le16/32_to_host`)
- Platform filesystem functions (except `fs_init`)

### 8.3 Recommended Refactoring Strategy

1. **Phase 1 — Keep transport ownership in `EtherCATMaster`**: New code should use the master's instance transport (`sendSingleDatagram`, `readRegister`, `writeRegister`) rather than legacy free-function helpers.
2. **Phase 2 — Move PDO state into `PDOManager`**: Replace `g_slave_configs`, `g_pdo_mapping`, `g_pdo_stats` with instance members.
3. **Phase 3 — Move SDO state into `SDOManager`**: Replace `s_queue`, `s_worker_thread`, `s_slave_mbx[]` with instance members.
4. **Phase 4 — Move DC state into `DCManager`**: Replace `g_dc_ctx` and related globals.
5. **Phase 5 — Move fault state into `FaultDetector`**: Replace `g_slave_faults` etc.
6. **Phase 6 — Eliminate remaining global router paths**: `EtherCATMaster` already owns a packet router; remaining global accessors should be retired as call sites are migrated.
7. **Phase 7 — Clean up stubs**: VoE/EoE/FoE can remain stubbed but should reference master instance.

### 8.4 Key Risk: Legacy Internal Paths Still Exist

The public architecture is now `DS402Master` on top of `EtherCATMaster`, with `CiA402Drive` handling per-drive DS402 behavior and examples reduced to motion policy. The remaining risk is any internal code path that still bypasses this stack through legacy free-function helpers or stale documentation.

Report the inventory as of the current codebase state. Written to `Tether/docs/ETHERCAT_INVENTORY.md`.
