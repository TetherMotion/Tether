#pragma once

// Shared ESC211 SM2/SM3 process-data assignment per ESI v0.9 (RSAPTest):
//
//   SM2 (host -> ESC211):
//     0x1600    flat FSoE SafetyPDU map, 16 x 31 B = 496 B  [flat_fsoe_maps]
//     0x1601    OutputCounter + SAFE_DO (8 B)               ESI mandatory
//     0x1610+k  per-channel FSoE-k PDO                      k < fsoe_channels
//
//   SM3 (ESC211 -> host):
//     0x1A00    flat FSoE SafetyPDU map, 496 B              [flat_fsoe_maps]
//     0x1A01    status: InputCounter/SAFE_DI/PowerStatus/DO_Monitor/
//               DO_Value/DI_Value/DO_Command (7 x UDINT)    ESI mandatory
//     0x1A02    RSAP-Info                                   ESI SM3 default
//     0x1A03    RSAP-Debug                                  ESI SM3 default
//     0x1A10+k  per-channel FSoE-k PDO                      k < fsoe_channels
//
// The flat maps are rewritten explicitly — the ESC211 accepts mapping
// writes for 0x1600/0x1A00 and the content matches the ESI (all 16
// 0x6000/0x7000 subitems of 31 bytes each).
//
// Every other PDO is vendor-fixed: the device's own mapping is read back
// via SDO (registerExisting*PDO) and only the SM assignment (0x1C12 /
// 0x1C13) is written — never the PDO's contents.  This is robust across
// firmware/ESI revisions (e.g. the RSAP PDOs grew between ESI releases);
// an optional PDO the device does not declare is skipped with a warning.
//
// Call in PRE-OP, then applyCustomPDOs() + configurePDOSyncManagers().

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/Mailbox.hpp"
#include "tether/drives/NexcobotESC211/Registers/SafetyStatus.hpp"
#include "tether/drives/NexcobotESC211/Registers/RSAPMonitoring.hpp"
#include "tether/drives/NexcobotESC211/Registers/FSOERx.hpp"
#include "tether/drives/NexcobotESC211/Registers/FSOETx.hpp"
#include "logging/Logger.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace Reg = EtherCAT::Drives::Registers::NexcobotESC211;

/// Assign the ESC211 SM2/SM3 PDOs per the ESI.  @p flat_fsoe_maps adds
/// the raw 496-byte FSoE SafetyPDU blocks (0x1600/0x1A00) — needed by
/// applications that relay the raw FSoE frames.  @p fsoe_channels is the
/// number of per-channel PDOs (0x1610+k / 0x1A10+k) to assign.
///
/// Returns false when a mandatory part fails; optional PDOs (RSAP,
/// per-channel) that the device does not declare are skipped with a
/// warning.
inline bool configureEsc211PdoAssignment(EtherCAT::Slave& s,
                                         const char* tag,
                                         bool flat_fsoe_maps,
                                         size_t fsoe_channels) {
    // ---- SM2 (RxPDO, host -> ESC211) --------------------------------------

    if (flat_fsoe_maps) {
        // 0x1600: 16 x 31-byte FSoE SafetyPDU sections (0x6000:01..10).
        auto err = s.configureCustomRxPDO(0x1600, {
            {&Reg::FSOERx::FSOERxPDU_1,  31}, {&Reg::FSOERx::FSOERxPDU_2,  31},
            {&Reg::FSOERx::FSOERxPDU_3,  31}, {&Reg::FSOERx::FSOERxPDU_4,  31},
            {&Reg::FSOERx::FSOERxPDU_5,  31}, {&Reg::FSOERx::FSOERxPDU_6,  31},
            {&Reg::FSOERx::FSOERxPDU_7,  31}, {&Reg::FSOERx::FSOERxPDU_8,  31},
            {&Reg::FSOERx::FSOERxPDU_9,  31}, {&Reg::FSOERx::FSOERxPDU_10, 31},
            {&Reg::FSOERx::FSOERxPDU_11, 31}, {&Reg::FSOERx::FSOERxPDU_12, 31},
            {&Reg::FSOERx::FSOERxPDU_13, 31}, {&Reg::FSOERx::FSOERxPDU_14, 31},
            {&Reg::FSOERx::FSOERxPDU_15, 31}, {&Reg::FSOERx::FSOERxPDU_16, 31},
        });
        if (err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "Custom RxPDO 0x1600 config failed: {}",
                        EtherCAT::slaveErrorToString(err));
            return false;
        }
    }

    // 0x1601: mandatory general output PDO (OutputCounter + SAFE_DO).
    if (auto err = s.registerExistingRxPDO(0x1601); err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(tag, "Register RxPDO 0x1601 failed: {}",
                    EtherCAT::slaveErrorToString(err));
        return false;
    }

    for (size_t k = 0; k < fsoe_channels; ++k) {
        const auto idx = static_cast<uint16_t>(0x1610 + k);
        if (auto err = s.registerExistingRxPDO(idx); err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGW(tag, "RxPDO 0x{:04X} (FSoE{}) not available: {} — skipped",
                        idx, k, EtherCAT::slaveErrorToString(err));
        }
    }

    // ---- SM3 (TxPDO, ESC211 -> host) --------------------------------------

    if (flat_fsoe_maps) {
        // 0x1A00: 16 x 31-byte FSoE SafetyPDU sections (0x7000:01..10).
        auto err = s.configureCustomTxPDO(0x1A00, {
            {&Reg::FSOETx::FSOETxPDU_1,  31}, {&Reg::FSOETx::FSOETxPDU_2,  31},
            {&Reg::FSOETx::FSOETxPDU_3,  31}, {&Reg::FSOETx::FSOETxPDU_4,  31},
            {&Reg::FSOETx::FSOETxPDU_5,  31}, {&Reg::FSOETx::FSOETxPDU_6,  31},
            {&Reg::FSOETx::FSOETxPDU_7,  31}, {&Reg::FSOETx::FSOETxPDU_8,  31},
            {&Reg::FSOETx::FSOETxPDU_9,  31}, {&Reg::FSOETx::FSOETxPDU_10, 31},
            {&Reg::FSOETx::FSOETxPDU_11, 31}, {&Reg::FSOETx::FSOETxPDU_12, 31},
            {&Reg::FSOETx::FSOETxPDU_13, 31}, {&Reg::FSOETx::FSOETxPDU_14, 31},
            {&Reg::FSOETx::FSOETxPDU_15, 31}, {&Reg::FSOETx::FSOETxPDU_16, 31},
        });
        if (err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "Custom TxPDO 0x1A00 config failed: {}",
                        EtherCAT::slaveErrorToString(err));
            return false;
        }
    }

    // 0x1A01: mandatory status PDO (InputCounter, SAFE_DI, PowerStatus,
    //         DO_Monitor, DO_Value, DI_Value, DO_Command).
    if (auto err = s.registerExistingTxPDO(0x1A01); err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(tag, "Register TxPDO 0x1A01 failed: {}",
                    EtherCAT::slaveErrorToString(err));
        return false;
    }

    // 0x1A02 RSAP-Info, 0x1A03 RSAP-Debug — ESI SM3 defaults, optional.
    for (uint16_t idx : {0x1A02, 0x1A03}) {
        if (auto err = s.registerExistingTxPDO(idx); err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGW(tag, "TxPDO 0x{:04X} (RSAP) not available: {} — skipped",
                        idx, EtherCAT::slaveErrorToString(err));
        }
    }

    for (size_t k = 0; k < fsoe_channels; ++k) {
        const auto idx = static_cast<uint16_t>(0x1A10 + k);
        if (auto err = s.registerExistingTxPDO(idx); err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGW(tag, "TxPDO 0x{:04X} (FSoE{}) not available: {} — skipped",
                        idx, k, EtherCAT::slaveErrorToString(err));
        }
    }

    return true;
}

/// Bump the ESC211 OutputCounter — the first UDINT of the mandatory RxPDO
/// 0x1601 (host→ESC211 heartbeat per the ESI).  Call once per PDO cycle
/// before exchangeAll().  No-op when 0x1601 is not assigned.
inline void bumpOutputCounter(EtherCAT::PDOManager& pdo,
                              uint16_t slave_index) {
    auto& mapping = pdo.mapping();
    for (size_t i = 0; i < mapping.entry_count(); ++i) {
        auto* e = mapping.get_entry_mut(i);
        if (e && e->slave_index == slave_index &&
            e->direction == EtherCAT::PDO::PDODirection::RxPDO &&
            e->pdo_index == 0x1601 && e->app_buffer && e->data_size >= 4) {
            auto* p = static_cast<uint8_t*>(e->app_buffer);
            const uint32_t n = (static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24)) + 1;
            p[0] = static_cast<uint8_t>(n);
            p[1] = static_cast<uint8_t>(n >> 8);
            p[2] = static_cast<uint8_t>(n >> 16);
            p[3] = static_cast<uint8_t>(n >> 24);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Shared INIT -> SAFE-OP -> OP ladder
//
// Both the session tools (examples/esc211_slave_session.cpp) and the
// blackchannel orchestrator (Esc211InitSequence) run the same sequence:
// force INIT -> explicit mailbox config -> PRE-OP -> drain mailbox ->
// optional config hook -> ESI PDO assignment -> applyCustomPDOs ->
// sync-manager config -> SAFE-OP -> PDO priming -> OP.  The helpers
// below parameterise the differences (PDO map variant, SDO mutex,
// priming policy) instead of duplicating the ladder.
// ---------------------------------------------------------------------------

/// Configuration for initEsc211ToSafeOp().
struct Esc211InitConfig {
    EtherCAT::MailboxSyncManagerConfig mailbox_out;
    EtherCAT::MailboxSyncManagerConfig mailbox_in;
    uint16_t mailbox_protocols = 0x0004;

    /// PDO assignment variant (see configureEsc211PdoAssignment).
    bool flat_fsoe_maps = false;
    size_t fsoe_channels = 0;

    /// Drain stale SM1 mailbox data after the PRE-OP transition.
    bool drain_mailbox = false;

    /// Stop after the PRE-OP transition (before PDO assignment) —
    /// for tools that only need CoE/SDO access.
    bool stop_after_preop = false;

    /// PDO priming after SAFE-OP: prime_cycles x prime_delay exchanges.
    int prime_cycles = 5;
    std::chrono::milliseconds prime_delay{0};
    /// Increment the 0x1601 OutputCounter during priming.
    bool bump_counters = false;

    /// Optional mutex serialising every bus step (blackchannel sdo_mutex).
    std::mutex* sdo_mutex = nullptr;

    /// Optional hook run in PRE-OP after the mailbox drain (e.g. the
    /// blackchannel --program config write).  Return false to abort.
    std::function<bool(EtherCAT::Slave&)> preop_hook;
};

/// Force INIT -> mailbox -> PRE-OP -> [drain] -> [preop_hook] ->
/// ESI PDO assignment -> applyCustomPDOs -> sync-manager config ->
/// SAFE-OP -> PDO priming.  Returns true in SAFE-OP (or PRE-OP when
/// cfg.stop_after_preop).
inline bool initEsc211ToSafeOp(EtherCAT::Master& master,
                               EtherCAT::PDOManager& pdo,
                               uint16_t slave_index,
                               const Esc211InitConfig& cfg,
                               const char* tag) {
    auto lock = [&cfg]() {
        return cfg.sdo_mutex ? std::unique_lock<std::mutex>(*cfg.sdo_mutex)
                             : std::unique_lock<std::mutex>();
    };
    auto& s = master.slave(slave_index);

    // Clear any stale PDO mapping entries from a previous (failed) attempt.
    pdo.mapping().remove_entries_for_slave(slave_index);

    // 0. Force slave to INIT before any configuration.  Ensures a clean
    //    starting state regardless of what the slave firmware was doing
    //    before (stale mailbox data, error state after a failed init).
    TETHER_LOGI(tag, "Forcing slave {} to INIT before configuration...", slave_index);
    {
        auto g = lock();
        EtherCAT::ALResetController reset_ctrl(master);
        auto reset_result = reset_ctrl.resetSlave(slave_index, 0x01, 50, 50);
        if (!reset_result.success) {
            TETHER_LOGW(tag, "AL reset to INIT failed (AL_STATUS=0x{:04X}, code=0x{:04X}) — continuing",
                        reset_result.final_al_status, reset_result.final_al_status_code);
        }
    }

    // 1. Mailbox configuration
    TETHER_LOGI(tag, "Configuring mailbox for slave {}...", slave_index);
    {
        auto g = lock();
        auto mb_err = s.configureMailbox(cfg.mailbox_out, cfg.mailbox_in,
                                         cfg.mailbox_protocols);
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "Mailbox config failed: {}",
                        EtherCAT::slaveErrorToString(mb_err));
            return false;
        }
    }

    // 2. PRE-OP transition
    {
        auto g = lock();
        auto pre_err = s.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "PRE-OP transition failed: {}",
                        EtherCAT::slaveErrorToString(pre_err));
            return false;
        }
    }
    TETHER_LOGI(tag, "Slave {} in PRE-OP", slave_index);

    // 3. Drain stale mailbox data (blackchannel does this; session tools
    //    configure a fresh mailbox so it is optional).
    if (cfg.drain_mailbox) {
        auto g = lock();
        if (!s.drainMailbox()) {
            TETHER_LOGW(tag, "Mailbox drain did not complete — stale responses may occur");
        }
    }

    // 4. Enable post-SDO AL status read-back for diagnostics.
    {
        auto g = lock();
        master.sdoManager(slave_index)
            .setBehaviourOptions({.request_al_status_after_coe_requests = true});
    }

    if (cfg.stop_after_preop) {
        return true;
    }

    // 4b. Optional application hook in PRE-OP (e.g. --program config
    //     write — the ESC211 only accepts config data writes in PRE-OP).
    if (cfg.preop_hook) {
        auto g = lock();
        if (!cfg.preop_hook(s)) {
            return false;
        }
    }

    // 5./6. PDO assignment per ESI v0.9.
    TETHER_LOGI(tag, "Slave {}: Configuring ESI PDO assignment "
                "(flat_fsoe_maps={}, {} channel PDOs)...",
                slave_index, cfg.flat_fsoe_maps, cfg.fsoe_channels);
    {
        auto g = lock();
        if (!configureEsc211PdoAssignment(s, tag, cfg.flat_fsoe_maps,
                                        cfg.fsoe_channels)) {
            TETHER_LOGE(tag, "ESI PDO assignment failed");
            return false;
        }
    }

    // 7. Register PDO buffers and configure sync managers.
    {
        auto g = lock();
        auto apply_err = s.applyCustomPDOs();
        if (apply_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "Apply custom PDOs failed: {}",
                        EtherCAT::slaveErrorToString(apply_err));
            return false;
        }

        auto pdo_err = s.configurePDOSyncManagers();
        if (pdo_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "PDO sync-manager config failed: {}",
                        EtherCAT::slaveErrorToString(pdo_err));
            return false;
        }
    }

    // 8. SAFE-OP transition
    {
        auto g = lock();
        auto safe_err = s.transitionToSafeOp();
        if (safe_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "SAFE-OP transition failed: {}",
                        EtherCAT::slaveErrorToString(safe_err));
            return false;
        }
    }
    TETHER_LOGI(tag, "Slave {} in SAFE-OP", slave_index);

    // 9. Prime PDO exchange (raises the PDO request/reply counters the
    //    OP transition check requires).
    for (int i = 0; i < cfg.prime_cycles; ++i) {
        if (cfg.bump_counters) bumpOutputCounter(pdo, slave_index);
        pdo.exchangeAll();
        if (cfg.prime_delay.count() > 0) {
            std::this_thread::sleep_for(cfg.prime_delay);
        }
    }
    return true;
}

/// Configuration for esc211TransitionToOp().
struct Esc211OpTransitionConfig {
    /// PDO priming before OP.  When exchange_thread_priming is false,
    /// skips the manual priming and waits prime_delay * prime_cycles
    /// for the cyclic exchange thread to raise the counters.
    int prime_cycles = 5;
    std::chrono::milliseconds prime_delay{0};
    bool bump_counters = false;
    bool exchange_thread_priming = true;
    std::mutex* sdo_mutex = nullptr;
};

/// Prime PDO exchange (or wait for the exchange thread to prime), then
/// request the OP transition.  Call after initEsc211ToSafeOp().
inline bool esc211TransitionToOp(EtherCAT::Slave& s,
                                 EtherCAT::PDOManager& pdo,
                                 uint16_t slave_index,
                                 const Esc211OpTransitionConfig& cfg,
                                 const char* tag) {
    auto lock = [&cfg]() {
        return cfg.sdo_mutex ? std::unique_lock<std::mutex>(*cfg.sdo_mutex)
                             : std::unique_lock<std::mutex>();
    };

    if (cfg.exchange_thread_priming) {
        TETHER_LOGI(tag, "Slave {}: priming PDO exchange before OP transition...", slave_index);
        for (int i = 0; i < cfg.prime_cycles; ++i) {
            if (cfg.bump_counters) bumpOutputCounter(pdo, slave_index);
            pdo.exchangeAll();
            if (cfg.prime_delay.count() > 0) {
                std::this_thread::sleep_for(cfg.prime_delay);
            }
        }
    } else {
        TETHER_LOGI(tag, "Slave {}: waiting for exchange thread to prime PDO counters...", slave_index);
        std::this_thread::sleep_for(cfg.prime_delay * cfg.prime_cycles);
    }

    {
        auto g = lock();
        auto op_err = s.transitionToOp();
        if (op_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag, "OP transition failed: {}",
                        EtherCAT::slaveErrorToString(op_err));
            return false;
        }
    }
    TETHER_LOGI(tag, "Slave {} in OP", slave_index);
    return true;
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
