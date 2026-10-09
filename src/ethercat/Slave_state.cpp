/**
 * @file Slave_state.cpp
 * @brief Slave — AL state transitions and state queries.
 *
 * TU split out of Slave.cpp.
 */

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "SlaveESIHelpers.hpp"
#include "raw/RawConstants.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"

#include <cstring>
#include <bit>
#include <format>

namespace EtherCAT {

static const char* TAG = "Slave";

// -- State transitions -------------------------------------------------------

namespace {

/**
 * @brief Verify that the slave's SM hardware registers match the expected configs.
 *
 * Logs mismatches as errors but never blocks the caller.
 * Detailed per-SM dumps are emitted when the corresponding debug flag is set.
 *
 * @param slave        The slave to verify
 * @param sm_start     First SM index to check (inclusive)
 * @param sm_end       Last SM index to check (inclusive)
 * @param debug_flag   If true, dump detailed SM register state
 * @param tag          Logger tag for diagnostic output
 */
void verifySyncManagers(EtherCAT::Slave& slave,
                        uint8_t sm_start,
                        uint8_t sm_end,
                        bool debug_flag,
                        const char* tag)
{
    using EtherCAT::PDO::kMaxPDOSlaves;
    const uint16_t idx = slave.index();
    auto& pdo = slave.master().pdo();
    auto* cfgs = pdo.slaveConfigs();
    if (idx >= kMaxPDOSlaves) {
        TETHER_LOGE(tag, "{}: Cannot verify SMs — index out of range", idx);
        return;
    }
    const auto& expected = cfgs[idx].sm;

    if (debug_flag) {
        TETHER_LOGI(tag,
            "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(tag,
            "║  SM Verification: {}  SM{}–SM{}                           ║",
            slave.logPrefix().c_str(), static_cast<unsigned>(sm_start), static_cast<unsigned>(sm_end));
        TETHER_LOGI(tag,
            "╚══════════════════════════════════════════════════════════════╝");
    }

    bool any_mismatch = false;
    for (uint8_t i = sm_start; i <= sm_end; ++i) {
        if (!expected[i].enable) {
            if (debug_flag) {
                TETHER_LOGI(tag, "SM{}: expected disabled — skipped", static_cast<unsigned>(i));
            }
            continue;
        }
        auto result = slave.sm(i).validate(expected[i]);
        if (!result.valid) {
            TETHER_LOGE(tag, "{}: SM{} verification FAILED — {}",
                        idx, static_cast<unsigned>(i), result.message.c_str());
            any_mismatch = true;
        } else if (debug_flag) {
            TETHER_LOGI(tag, "{}: SM{} verification PASSED", idx, static_cast<unsigned>(i));
        }
        if (debug_flag) {
            slave.sm(i).dump(tag);
        }
    }

    if (debug_flag) {
        TETHER_LOGI(tag,
            "{}: SM verification summary — {}",
            idx, any_mismatch ? "MISMATCHES DETECTED (see errors above)" : "ALL OK");
    }
}

} // anonymous namespace

SlaveError Slave::transitionTo(SlaveState target) {
    switch (target) {
        case SlaveState::INIT:    return transitionToInit();
        case SlaveState::PRE_OP:  return transitionToPreOp();
        case SlaveState::SAFE_OP: return transitionToSafeOp();
        case SlaveState::OP:      return transitionToOp();
        case SlaveState::BOOT:    return transitionToBoot();
        default:
            TETHER_LOGE( TAG,
                "{}: Unknown target state 0x{:02X}", logPrefix().c_str(), static_cast<uint8_t>(target));
            return SlaveError::InvalidStateTransition;
    }
}

SlaveError Slave::transitionToInit() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: {}                          ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: {} => INIT", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Requested by user/application");
        TETHER_LOGI(TAG, "║  Requirements: None (INIT is the base state)");
        TETHER_LOGI(TAG, "║  Status:     Fulfilled - proceeding with transition");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!master_->requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::INIT))) {
        TETHER_LOGE( TAG,
            "{}: Failed to transition to INIT", logPrefix().c_str());
        return SlaveError::TransportError;
    }
    // Reset configuration flags when going back to INIT
    mailbox_configured_ = false;
    pdo_configured_ = false;
    
    if (slave_debug_flags_.stateMachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: {} => INIT SUCCESS                  ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "║  Configuration flags reset: mailbox=false, pdo=false          ║");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

SlaveError Slave::transitionToPreOp() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: {}                          ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: {} => PRE_OP", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Mailbox operations (SDO, FoE, etc.) require PRE_OP");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - Mailbox (SM0/SM1) must be configured: {}", 
                    mailbox_configured_ ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        TETHER_LOGI(TAG, "║  Status:     {}", 
                    mailbox_configured_ ? "Fulfilled - proceeding with transition" : "NOT Fulfilled - transition blocked");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!mailbox_configured_) {
        TETHER_LOGE( TAG,
            "{}: Cannot transition to PRE_OP — mailbox (SM0/SM1) "
            "not configured. Call configureMailbox(), "
            "assumeMailboxAlreadyConfigured(), or markNoMailbox() "
            "for mailbox-less slaves first.", logPrefix().c_str());
        return SlaveError::MailboxNotConfigured;
    }
    verifySyncManagers(*this, 0, 1, slave_debug_flags_.verifyPreOp, TAG);
    if (!master_->transitionSlaveToPreOperational(index_)) {
        TETHER_LOGE( TAG,
            "{}: Failed to transition to PRE_OP", logPrefix().c_str());
        return SlaveError::TransportError;
    }

    // Drain any stale mailbox data that the slave firmware may have written
    // into SM1 during or after the PRE-OP transition.  configureMailbox()
    // drains once before the transition, but the firmware can emit AL status
    // notifications or initialization messages as it enters PRE-OP, leaving
    // stale data whose mailbox counter doesn't match the master's first SDO
    // request — this causes "Stale mailbox response" errors and SDO failures
    // on the first SDO exchange after entering PRE-OP.
    (void)drainMailbox();

    if (slave_debug_flags_.stateMachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: {} => PRE_OP SUCCESS                ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }

    return SlaveError::Ok;
}

SlaveError Slave::transitionToSafeOp() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: {}                          ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: {} => SAFE_OP", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Process data exchange requires SAFE_OP");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - PDO sync-managers (SM2/SM3) must be configured: {}", 
                    pdo_configured_ ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        TETHER_LOGI(TAG, "║  Status:     {}", 
                    pdo_configured_ ? "Fulfilled - proceeding with transition" : "NOT Fulfilled - transition blocked");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!pdo_configured_) {
        TETHER_LOGE( TAG,
            "{}: Cannot transition to SAFE_OP — PDO sync-managers "
            "(SM2/SM3) not configured. Call configurePDOSyncManagers() or "
            "assumePDOAlreadyConfigured() first.", logPrefix().c_str());
        return SlaveError::PDONotConfigured;
    }
    verifySyncManagers(*this, 0, 3, slave_debug_flags_.verifySafeOp, TAG);
    if (!master_->requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::SAFE_OP))) {
        TETHER_LOGE( TAG,
            "{}: Failed to transition to SAFE_OP (transport error)", logPrefix().c_str());
        return SlaveError::TransportError;
    }

    // Confirm SAFE_OP (up to 2 s).  Some slaves need time to validate SM2/SM3.
    for (int attempt = 0; attempt < 200; attempt++) {
        if (master_->isCancelRequested()) {
            TETHER_LOGI(TAG, "{}: SAFE_OP confirmation cancelled", logPrefix().c_str());
            return SlaveError::Cancelled;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(10);
        uint8_t state = 0;
        if (master_->readSlaveApplicationLayerState(index_, state)) {
            if (state == static_cast<uint8_t>(SlaveState::SAFE_OP)) {
                if (slave_debug_flags_.stateMachine) {
                    TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
                    TETHER_LOGI(TAG, "║  Transition Result: {} => SAFE_OP SUCCESS               ║", logPrefix().c_str());
                    TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
                }
                // Debug gate checkpoint: SAFE_OP confirmed
                master_->debugGate().notifyCheckpoint("state:safe-op", index_);
                return SlaveError::Ok;
            }
        }
    }

    uint16_t al_code = 0;
    readALStatusCode(al_code);
    TETHER_LOGE(TAG, "{}: SAFE_OP not confirmed after 2s (AL status code: {} (0x{:04X}))", logPrefix().c_str(), getALStatusCodeName(al_code), al_code);
    return SlaveError::TransportError;
}

SlaveError Slave::transitionToOp() {
    // --- Evaluate requirements before printing the debug banner ---
    bool pdo_req_ok = false;
    bool pdo_reply_ok = false;
    bool has_pdo_entries = false;
    {
        auto& pdo_mgr = master_->pdoForSlave(index_);
        has_pdo_entries = pdo_mgr.hasSlavePDOEntries(index_);
        if (has_pdo_entries) {
            for (int wait_ms = 0; wait_ms < 100; wait_ms++) {
                if (master_->isCancelRequested()) {
                    TETHER_LOGI(TAG, "{}: OP transition cancelled during PDO counter wait", logPrefix().c_str());
                    return SlaveError::Cancelled;
                }
                const uint32_t req   = pdo_mgr.getSlavePDORequestCount(index_);
                const uint32_t reply = pdo_mgr.getSlavePDOReplyCount(index_);
                pdo_req_ok   = (req > 0);
                pdo_reply_ok = (reply > 0);
                if (pdo_req_ok && pdo_reply_ok) {
                    break;
                }
                Tether::Platform::Clock::instance().delayMilliseconds(1);
            }
        }
    }
    bool fmmu_ok = fmmu_mgr_.verifyFromSlave();

    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: {}                          ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: {} => OP", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Full operational mode for process data exchange");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - PDO sync-managers (SM2/SM3) should be configured: {}",
                    pdo_configured_ ? "✓ FULFILLED" : "⚠ NOT FULFILLED (warning only)");
        if (has_pdo_entries) {
            TETHER_LOGI(TAG, "║    - PDO request counter  > 0: {}",
                        pdo_req_ok ? "✓ FULFILLED" : "✗ NOT FULFILLED");
            TETHER_LOGI(TAG, "║    - PDO reply counter    > 0: {}",
                        pdo_reply_ok ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        } else {
            TETHER_LOGI(TAG, "║    - PDO exchange check:     N/A (no PDO entries for this slave)");
        }
        TETHER_LOGI(TAG, "║    - FMMU configuration matches slave hardware: {}",
                    fmmu_ok ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        TETHER_LOGI(TAG, "║  Status:     {}", (pdo_req_ok && pdo_reply_ok && fmmu_ok)
                    ? "Proceeding with transition"
                    : "HALTED — requirements not met");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }

    if (!pdo_configured_) {
        TETHER_LOGW( TAG,
            "{}: Transitioning to OP without PDO sync-managers configured. "
            "This may cause issues with process data exchange.", logPrefix().c_str());
    }

    if (has_pdo_entries && (!pdo_req_ok || !pdo_reply_ok)) {
        auto& pdo_mgr = master_->pdoForSlave(index_);
        const uint32_t req   = pdo_mgr.getSlavePDORequestCount(index_);
        const uint32_t reply = pdo_mgr.getSlavePDOReplyCount(index_);
        TETHER_LOGE(TAG,
            "{}: OP transition rejected — no PDO exchange after 100 ms "
            "(req={} reply={}). Start the motion loop or call exchangeAll() "
            "before requesting OP.",
            logPrefix().c_str(), req, reply);
        return SlaveError::TransportError;
    }

    if (!fmmu_ok) {
        TETHER_LOGE(TAG,
            "{}: OP transition rejected — FMMU configuration mismatch "
            "(read from slave hardware does not match expected values)",
            logPrefix().c_str());
        return SlaveError::TransportError;
    }

    // Request OP with Error Acknowledge bit (0x08 | 0x10 = 0x18)
    // Some slaves require the ACK bit to clear internal error latches.
    if (!master_->requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::OP) | 0x10)) {
        TETHER_LOGE( TAG,
            "{}: Failed to transition to OP (transport error)", logPrefix().c_str());
        return SlaveError::TransportError;
    }

    // Diagnostic: read back AL_CONTROL to prove the OP request landed, plus
    // watchdog/DL/SM2 status so we can tell "slave declined" from "write lost".
    {
        uint16_t al_ctrl = 0, wd = 0, dl = 0;
        const bool c_ok = master_->readRegister(index_, Raw::EC_REG_AL_CONTROL, al_ctrl, 200);
        const bool w_ok = master_->readRegister(index_, Raw::EC_REG_WD_STATUS, wd, 200);
        const bool d_ok = master_->readRegister(index_, 0x0110, dl, 200);
        uint8_t sm2[8] = {};
        const bool s_ok = master_->readRegister(index_, 0x0810, sm2, sizeof(sm2), 200);
        TETHER_LOGI(TAG, "{}: OP request readback: AL_CTRL={}0x{:04X} "
                         "WD={}0x{:04X} DL={}0x{:04X} "
                         "SM2[{}] addr=0x{:02X}{:02X} len={} ctrl=0x{:02X} "
                         "stat=0x{:02X} act=0x{:02X} pdi=0x{:02X}",
                    logPrefix().c_str(),
                    c_ok ? "" : "? ", al_ctrl,
                    w_ok ? "" : "? ", wd,
                    d_ok ? "" : "? ", dl,
                    s_ok ? "ok" : "rd-fail",
                    sm2[1], sm2[0],
                    sm2[2] | (sm2[3] << 8), sm2[4], sm2[5], sm2[6], sm2[7]);
    }

    // Confirm OP (up to 5 s).  The slave may need continuous process data.
    uint8_t last_state = 0;
    int last_read_ok = 0;
    for (int attempt = 0; attempt < 500; attempt++) {
        if (master_->isCancelRequested()) {
            TETHER_LOGI(TAG, "{}: OP confirmation cancelled", logPrefix().c_str());
            return SlaveError::Cancelled;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(10);
        uint8_t state = 0;
        const bool ok = master_->readSlaveApplicationLayerState(index_, state);
        const bool changed = ok && (state != last_state);
        last_read_ok = ok;
        if (ok)
            last_state = state;
        if ((attempt % 100) == 99 || changed)
            TETHER_LOGI(TAG, "{}: OP confirm poll: read={} state=0x{:02X} "
                             "(attempt {})",
                        logPrefix().c_str(), ok, last_state, attempt + 1);
        // Re-issue the OP request every ~500 ms.  If the output SM watchdog
        // was expired when the request first arrived, the slave declines OP
        // and never re-evaluates the request on its own — a fresh write lets
        // it transition as soon as the watchdog recovers.
        if (ok && state != static_cast<uint8_t>(SlaveState::OP) &&
            (attempt % 50) == 49) {
            TETHER_LOGI(TAG, "{}: Re-issuing OP request (still 0x{:02X})",
                        logPrefix().c_str(), state);

            // A latched SM watchdog trigger (status bit5) blocks the
            // SAFE_OP->OP transition with AL code 0 — it may have expired
            // in the window between SM activation and the first cyclic
            // frame.  Toggling the SM activate bit re-arms the channel
            // and clears the trigger so the slave accepts OP.
            bool wd_latched = false;
            for (uint8_t sm = 2; sm < 4; ++sm) {
                const uint16_t base =
                    static_cast<uint16_t>(0x0800 + sm * 8);
                uint8_t stat = 0;
                if (master_->readRegister(index_, base + 5, &stat, 1, 200) &&
                    (stat & 0x20u)) {
                    wd_latched = true;
                    TETHER_LOGW(TAG, "{}: SM{} watchdog trigger latched "
                                     "(stat=0x{:02X}) — resetting channel",
                                logPrefix().c_str(), sm, stat);
                    const uint8_t off = 0x00, on = 0x01;
                    master_->writeRegister(SlaveAddress(index_),
                                           static_cast<uint16_t>(base + 6),
                                           &off, 1, 200);
                    master_->writeRegister(SlaveAddress(index_),
                                           static_cast<uint16_t>(base + 6),
                                           &on, 1, 200);
                }
            }

            // Do NOT disarm the watchdogs here.  Firmware-driven ESCs
            // (e.g. AS715N) require the process-data watchdog armed to
            // validate the cyclic window during SAFE_OP->OP — zeroing the
            // timers makes the ESM refuse the transition outright.  If the
            // latch re-appears, the watchdog genuinely fired on a late or
            // missing frame; the correct fix is a clean cyclic window, not
            // masking the detector.
            (void)wd_latched;

            master_->requestSlaveApplicationLayerState(
                index_, static_cast<uint8_t>(SlaveState::OP) | 0x10);

            dumpOpDiagnostics();
        }
        if (ok) {
            if (state == static_cast<uint8_t>(SlaveState::OP)) {
                if (slave_debug_flags_.stateMachine) {
                    TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
                    TETHER_LOGI(TAG, "║  Transition Result: {} => OP SUCCESS                    ║", logPrefix().c_str());
                    TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
                }
                // Debug gate checkpoint: OP confirmed
                master_->debugGate().notifyCheckpoint("state:op", index_);
                return SlaveError::Ok;
            }
            // If state dropped to INIT or PRE_OP, something went wrong
            if (state == static_cast<uint8_t>(SlaveState::INIT) ||
                state == static_cast<uint8_t>(SlaveState::PRE_OP)) {
                uint16_t al_code = 0;
                readALStatusCode(al_code);
                TETHER_LOGE(TAG, "{}: Unexpected state 0x{:02X} during OP transition (AL status code: {} (0x{:04X}))",
                         logPrefix().c_str(), state, getALStatusCodeName(al_code), al_code);
                return SlaveError::TransportError;
            }
        }
    }

    uint16_t al_code = 0;
    readALStatusCode(al_code);
    TETHER_LOGE(TAG, "{}: OP not confirmed after 5s (last AL state=0x{:02X} "
                     "read_ok={} AL status code: {} (0x{:04X}))",
                logPrefix().c_str(), last_state, last_read_ok,
                getALStatusCodeName(al_code), al_code);
    // Final snapshot before giving up — same register set the retry
    // diagnostics dump, so the failure verdict includes which SM
    // watchdog fired and whether it ever recovered.
    dumpOpDiagnostics();
    return SlaveError::TransportError;
}

bool Slave::mailboxAvailable() {
    if (mailbox_serviced_in_safeop_) return true;
    SlaveState state;
    if (readState(state) != SlaveError::Ok) return true;  // unknown — let it try
    return state != SlaveState::SAFE_OP;
}

void Slave::dumpOpDiagnostics() {
    // One frame: FMMU block (0x0600, 32B) + SM2 process-data buffer
    // (0x1800, 8B) + watchdog status/counters (0x0440, 4B: status u16,
    // SM-WD count u8, PDI-WD count u8) + SM2/SM3 register blocks
    // (0x0810/0x0818, 8B each: addr, len, ctrl, stat, act, pdi).
    const SlaveAddress addrs[5] = {
        SlaveAddress(index_), SlaveAddress(index_), SlaveAddress(index_),
        SlaveAddress(index_), SlaveAddress(index_)};
    const uint16_t regs[5] = {0x0600, 0x1800, Raw::EC_REG_WD_STATUS,
                              0x0810, 0x0818};
    const uint16_t lens[5] = {32, 8, 4, 8, 8};
    auto batch = master_->readRegistersBatch(addrs, regs, lens, 5);
    std::vector<BatchReadResult> res;
    batch.waitAll(200, res);

    // FMMU readback proves the logical->physical map directly instead of
    // hiding behind WKC totals.
    if (res.size() > 0 && res[0].success &&
        res[0].data && res[0].datalen >= 32) {
        const uint8_t* fmmu_raw = res[0].data;
        for (int f = 0; f < 2; ++f) {
            const uint8_t* r = fmmu_raw + f * 16;
            const uint32_t log =
                static_cast<uint32_t>(r[0]) |
                (static_cast<uint32_t>(r[1]) << 8) |
                (static_cast<uint32_t>(r[2]) << 16) |
                (static_cast<uint32_t>(r[3]) << 24);
            const uint16_t len =
                static_cast<uint16_t>(r[4] | (r[5] << 8));
            const uint16_t phys =
                static_cast<uint16_t>(r[8] | (r[9] << 8));
            TETHER_LOGI(TAG, "{}: FMMU{}: log=0x{:08X} len={} "
                             "phys=0x{:04X} type=0x{:02X} act=0x{:02X}",
                        logPrefix().c_str(), f, log, len, phys,
                        r[11], r[12]);
        }
    } else {
        TETHER_LOGW(TAG, "{}: FMMU register readback failed",
                    logPrefix().c_str());
    }

    // SM2's process-data buffer proves cyclic output data is actually
    // reaching the slave — all-zero would mean the LRW/FMMU write side
    // is broken.
    uint8_t sm2_data[8] = {};
    const bool b_ok = res.size() > 1 && res[1].success &&
        res[1].data && res[1].datalen >= 8;
    if (b_ok) std::memcpy(sm2_data, res[1].data, 8);

    // Watchdog status + per-watchdog trigger counters.  The counters
    // increment on each trigger event — rising counts across retries
    // mean the watchdog keeps firing on new (late/missing) frames, not
    // just a stale latch.
    uint16_t wd_status = 0;
    uint8_t wd_sm_count = 0, wd_pdi_count = 0;
    if (res.size() > 2 && res[2].success &&
        res[2].data && res[2].datalen >= 4) {
        std::memcpy(&wd_status, res[2].data, 2);
        wd_sm_count  = res[2].data[2];
        wd_pdi_count = res[2].data[3];
    }
    TETHER_LOGI(TAG, "{}: SM2 buf[{}] {:02X} {:02X} {:02X} {:02X} "
                     "{:02X} {:02X} {:02X} {:02X} WD=0x{:04X} "
                     "wd_sm_cnt={} wd_pdi_cnt={}",
                logPrefix().c_str(), b_ok ? "ok" : "rd-fail",
                sm2_data[0], sm2_data[1], sm2_data[2], sm2_data[3],
                sm2_data[4], sm2_data[5], sm2_data[6], sm2_data[7],
                wd_status, wd_sm_count, wd_pdi_count);

    // SM2/SM3 register blocks: addr, len, ctrl, stat (bit5 = watchdog
    // trigger latched, bit0-4 status), act, pdi.
    for (int i = 0; i < 2; ++i) {
        const auto& r = res[3 + i];
        if (res.size() > static_cast<size_t>(3 + i) && r.success &&
            r.data && r.datalen >= 8) {
            TETHER_LOGI(TAG, "{}: SM{} regs[{}]: addr=0x{:02X}{:02X} "
                             "len={} ctrl=0x{:02X} stat=0x{:02X} "
                             "act=0x{:02X} pdi=0x{:02X}{}",
                        logPrefix().c_str(), 2 + i,
                        r.success ? "ok" : "rd-fail",
                        r.data[1], r.data[0],
                        r.data[2] | (r.data[3] << 8), r.data[4],
                        r.data[5], r.data[6], r.data[7],
                        (r.data[5] & 0x20u) ? " WD-TRIGGERED" : "");
        }
    }

    // Liveness probe: issue one SDO upload (0x1018:1 vendor ID).  The
    // mailbox is serviced by the slave's application CPU — if the SDO
    // answers, the firmware is alive and OP is gated by the ESM; if it
    // times out, the firmware is wedged and only a slave
    // reset/power-cycle is likely to recover it.  Firmware known to
    // drop mailbox service in SAFE_OP (SOMANET >= 5.6.x) is skipped:
    // a guaranteed timeout is noise, not diagnosis.
    if (!mailboxAvailable()) {
        TETHER_LOGW(TAG, "{}: SDO liveness probe skipped — mailbox not "
                         "serviced in SAFE_OP on this firmware",
                    logPrefix().c_str());
        return;
    }
    {
        CoE::CoETransactionOptions sdo_opts{};
        sdo_opts.timeout_ms = 300;
        auto sdo_res = master_->sdoManager(index_)
                           .template readSync<uint32_t>(0x1018, 1, sdo_opts);
        TETHER_LOGW(TAG, "{}: SDO liveness probe (0x1018:1): {}",
                    logPrefix().c_str(),
                    sdo_res.has_value()
                        ? "firmware ALIVE — ESM gating OP"
                        : "NO RESPONSE — slave firmware wedged");
    }
}

SlaveError Slave::transitionToBoot() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: {}                          ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: {} => BOOT", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Firmware update or bootstrap mode");
        TETHER_LOGI(TAG, "║  Requirements: None (BOOT is a special state)");
        TETHER_LOGI(TAG, "║  Status:     Fulfilled - proceeding with transition");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!master_->requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::BOOT))) {
        TETHER_LOGE( TAG,
            "{}: Failed to transition to BOOT (transport error)", logPrefix().c_str());
        return SlaveError::TransportError;
    }
    
    if (slave_debug_flags_.stateMachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: {} => BOOT SUCCESS                  ║", logPrefix().c_str());
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

// -- State query -------------------------------------------------------------

SlaveError Slave::readState(SlaveState& state) {
    uint8_t raw = 0;
    if (!master_->readSlaveApplicationLayerState(index_, raw)) {
        return SlaveError::TransportError;
    }
    state = static_cast<SlaveState>(raw & 0x0F);
    return SlaveError::Ok;
}

SlaveError Slave::readALStatusCode(uint16_t& code) {
    uint16_t status = 0;
    if (!master_->readRegister(SlaveAddress(index_), reg::AL_STATUS_CODE, status)) {
        return SlaveError::TransportError;
    }
    code = status;
    return SlaveError::Ok;
}

std::optional<SlaveState> Slave::ALState() {
    SlaveState st{};
    if (readState(st) != SlaveError::Ok) return std::nullopt;
    return st;
}

std::optional<uint16_t> Slave::ALCode() {
    uint16_t code = 0;
    if (readALStatusCode(code) != SlaveError::Ok) return std::nullopt;
    return code;
}

} // namespace EtherCAT

