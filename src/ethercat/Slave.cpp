/**
 * @file Slave.cpp
 * @brief Slave and NonExistingSlave implementation
 */

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>
#include <bit>

namespace EtherCAT {

static const char* TAG = "Slave";

// ============================================================================
// Slave
// ============================================================================

Slave::Slave(Master& master, uint16_t index)
    : master_(master), index_(index)
{
}

uint16_t Slave::adp() const {
    return Master::adpForSlaveIndex(index_);
}

bool Slave::apwr(uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms) {
    return master_.writeRegister(SlaveAddress(index_), ado, data, len, timeout_ms);
}

bool Slave::aprd(uint16_t ado, void* out, uint16_t len, unsigned int timeout_ms) {
    return master_.readRegister(SlaveAddress(index_), ado, out, len, timeout_ms);
}

// -- Mailbox configuration ---------------------------------------------------

SlaveError Slave::configureMailbox(Tether::Platform::LogLevel log_level) {
    if (!master_.autoConfigureMailbox(index_, log_level)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to auto-configure mailbox from SII", index_);
        return SlaveError::MailboxConfigFailed;
    }
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Mailbox configured from SII", index_);
    // Debug gate checkpoint: mailbox configured
    master_.debugGate().notifyCheckpoint("mailbox-configured", index_);
    return SlaveError::Ok;
}

SlaveError Slave::configureMailbox(
    const MailboxSyncManagerConfig& mbox_out,
    const MailboxSyncManagerConfig& mbox_in,
    uint16_t protocols)
{
    master_.setMailboxOverride(index_,
                               mbox_in.address, mbox_in.length,
                               mbox_out.address, mbox_out.length,
                               protocols);
    // Configure SDO manager with these mailbox params
    master_.sdoManager(index_).configureMailbox(
        mbox_in.address, mbox_in.length,
        mbox_out.address, mbox_out.length);

    // Write mailbox SM registers to slave ESC (same as autoConfigureMailbox)
    auto& pdo = master_.pdo();
    auto* slave_configs = pdo.slaveConfigs();
    if (index_ < PDO::kMaxPDOSlaves) {
        slave_configs[index_].sm[0] = PDO::SyncManagerConfig::mailbox_write(
            mbox_in.address, mbox_in.length);
        slave_configs[index_].sm[1] = PDO::SyncManagerConfig::mailbox_read(
            mbox_out.address, mbox_out.length);
        if (!pdo.configureSlavesSMs(index_)) {
            TETHER_LOGE(TAG, "Slave %u: Failed to write mailbox SM registers", index_);
            return SlaveError::MailboxConfigFailed;
        }

        // SM1 may contain stale/junk data left over from slave firmware boot.
        // Drain it now so the slave has a free outbound mailbox before the
        // first SDO exchange.
        (void)master_.drainSlaveMailbox(index_);
    }

    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Mailbox configured (wr=0x%04X/%u, rd=0x%04X/%u, proto=0x%04X)",
        index_, mbox_in.address, mbox_in.length,
        mbox_out.address, mbox_out.length, protocols);
    // Debug gate checkpoint: mailbox configured
    master_.debugGate().notifyCheckpoint("mailbox-configured", index_);
    return SlaveError::Ok;
}

void Slave::assumeMailboxAlreadyConfigured() {
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Assuming mailbox already configured", index_);
    // Debug gate checkpoint: mailbox configured
    master_.debugGate().notifyCheckpoint("mailbox-configured", index_);
}

// -- PDO SM configuration ----------------------------------------------------

SlaveError Slave::configurePDOSyncManagers() {
    if (!master_.configureProcessDataSyncManagersFromSii(index_)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to configure PDO sync-managers from SII", index_);
        return SlaveError::PDOConfigFailed;
    }
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: PDO sync-managers configured from SII", index_);

    if (slave_debug_flags_.pdoSm) {
        EtherCAT::debugPDOSyncManagerConfiguration(master_, index_, TAG);
    }

    return SlaveError::Ok;
}

SlaveError Slave::configurePDOSyncManagers(
    uint16_t sm2_addr, uint16_t sm2_len, uint8_t sm2_ctrl,
    uint16_t sm3_addr, uint16_t sm3_len, uint8_t sm3_ctrl)
{
    auto& pdo = master_.pdo();
    auto* cfgs = pdo.slaveConfigs();
    if (index_ >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE( TAG,
            "Slave %u: Index exceeds max PDO slaves (%zu)", index_, PDO::kMaxPDOSlaves);
        return SlaveError::PDOConfigFailed;
    }
    cfgs[index_].sm[2].phys_start_addr = sm2_addr;
    cfgs[index_].sm[2].length = sm2_len;
    cfgs[index_].sm[2].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm2_ctrl);
    cfgs[index_].sm[2].enable = 1;
    cfgs[index_].sm[2].type = PDO::SyncManagerType::ProcessOutput;

    cfgs[index_].sm[3].phys_start_addr = sm3_addr;
    cfgs[index_].sm[3].length = sm3_len;
    cfgs[index_].sm[3].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm3_ctrl);
    cfgs[index_].sm[3].enable = 1;
    cfgs[index_].sm[3].type = PDO::SyncManagerType::ProcessInput;

    if (!pdo.configureSlavesSMs(index_)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to write PDO SM registers", index_);
        return SlaveError::PDOConfigFailed;
    }

    pdo_configured_ = true;
    return SlaveError::Ok;
}

void Slave::assumePDOAlreadyConfigured() {
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Assuming PDO sync-managers already configured", index_);
}

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
        TETHER_LOGE(tag, "Slave %u: Cannot verify SMs — index out of range", idx);
        return;
    }
    const auto& expected = cfgs[idx].sm;

    if (debug_flag) {
        TETHER_LOGI(tag,
            "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(tag,
            "║  SM Verification: Slave %u  SM%u–SM%u                           ║",
            idx, static_cast<unsigned>(sm_start), static_cast<unsigned>(sm_end));
        TETHER_LOGI(tag,
            "╚══════════════════════════════════════════════════════════════╝");
    }

    bool any_mismatch = false;
    for (uint8_t i = sm_start; i <= sm_end; ++i) {
        if (!expected[i].enable) {
            if (debug_flag) {
                TETHER_LOGI(tag, "SM%u: expected disabled — skipped", static_cast<unsigned>(i));
            }
            continue;
        }
        auto result = slave.sm(i).validate(expected[i]);
        if (!result.valid) {
            TETHER_LOGE(tag, "Slave %u: SM%u verification FAILED — %s",
                        idx, static_cast<unsigned>(i), result.message.c_str());
            any_mismatch = true;
        } else if (debug_flag) {
            TETHER_LOGI(tag, "Slave %u: SM%u verification PASSED", idx, static_cast<unsigned>(i));
        }
        if (debug_flag) {
            slave.sm(i).dump(tag);
        }
    }

    if (debug_flag) {
        TETHER_LOGI(tag,
            "Slave %u: SM verification summary — %s",
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
                "Slave %u: Unknown target state 0x%02X", index_, static_cast<uint8_t>(target));
            return SlaveError::InvalidStateTransition;
    }
}

SlaveError Slave::transitionToInit() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u                          ║", index_);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: %s => INIT", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Requested by user/application");
        TETHER_LOGI(TAG, "║  Requirements: None (INIT is the base state)");
        TETHER_LOGI(TAG, "║  Status:     Fulfilled - proceeding with transition");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::INIT))) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to INIT", index_);
        return SlaveError::TransportError;
    }
    // Reset configuration flags when going back to INIT
    mailbox_configured_ = false;
    pdo_configured_ = false;
    
    if (slave_debug_flags_.stateMachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => INIT SUCCESS                  ║", index_);
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
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u                          ║", index_);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: %s => PRE_OP", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Mailbox operations (SDO, FoE, etc.) require PRE_OP");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - Mailbox (SM0/SM1) must be configured: %s", 
                    mailbox_configured_ ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        TETHER_LOGI(TAG, "║  Status:     %s", 
                    mailbox_configured_ ? "Fulfilled - proceeding with transition" : "NOT Fulfilled - transition blocked");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!mailbox_configured_) {
        TETHER_LOGE( TAG,
            "Slave %u: Cannot transition to PRE_OP — mailbox (SM0/SM1) "
            "not configured. Call configureMailbox() or "
            "assumeMailboxAlreadyConfigured() first.", index_);
        return SlaveError::MailboxNotConfigured;
    }
    verifySyncManagers(*this, 0, 1, slave_debug_flags_.verifyPreOp, TAG);
    if (!master_.transitionSlaveToPreOperational(index_)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to PRE_OP", index_);
        return SlaveError::TransportError;
    }
    
    if (slave_debug_flags_.stateMachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => PRE_OP SUCCESS                ║", index_);
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

SlaveError Slave::transitionToSafeOp() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u                          ║", index_);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: %s => SAFE_OP", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Process data exchange requires SAFE_OP");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - PDO sync-managers (SM2/SM3) must be configured: %s", 
                    pdo_configured_ ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        TETHER_LOGI(TAG, "║  Status:     %s", 
                    pdo_configured_ ? "Fulfilled - proceeding with transition" : "NOT Fulfilled - transition blocked");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!pdo_configured_) {
        TETHER_LOGE( TAG,
            "Slave %u: Cannot transition to SAFE_OP — PDO sync-managers "
            "(SM2/SM3) not configured. Call configurePDOSyncManagers() or "
            "assumePDOAlreadyConfigured() first.", index_);
        return SlaveError::PDONotConfigured;
    }
    verifySyncManagers(*this, 0, 3, slave_debug_flags_.verifySafeOp, TAG);
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::SAFE_OP))) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to SAFE_OP (transport error)", index_);
        return SlaveError::TransportError;
    }

    // Confirm SAFE_OP (up to 2 s).  Some slaves need time to validate SM2/SM3.
    for (int attempt = 0; attempt < 200; attempt++) {
        Tether::Platform::Clock::instance().delayMilliseconds(10);
        uint8_t state = 0;
        if (master_.readSlaveApplicationLayerState(index_, state)) {
            if (state == static_cast<uint8_t>(SlaveState::SAFE_OP)) {
                if (slave_debug_flags_.stateMachine) {
                    TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
                    TETHER_LOGI(TAG, "║  Transition Result: Slave %u => SAFE_OP SUCCESS               ║", index_);
                    TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
                }
                // Debug gate checkpoint: SAFE_OP confirmed
                master_.debugGate().notifyCheckpoint("state:safe-op", index_);
                return SlaveError::Ok;
            }
        }
    }

    uint16_t al_code = 0;
    readALStatusCode(al_code);
    TETHER_LOGE(TAG, "Slave %u: SAFE_OP not confirmed after 2s (AL status code: %s (0x%04X))", index_, getALStatusCodeName(al_code), al_code);
    return SlaveError::TransportError;
}

SlaveError Slave::transitionToOp() {
    // --- Evaluate requirements before printing the debug banner ---
    bool pdo_req_ok = false;
    bool pdo_reply_ok = false;
    bool has_pdo_entries = false;
    {
        auto& pdo_mgr = master_.pdo();
        has_pdo_entries = pdo_mgr.hasSlavePDOEntries(index_);
        if (has_pdo_entries) {
            for (int wait_ms = 0; wait_ms < 100; wait_ms++) {
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
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u                          ║", index_);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: %s => OP", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Full operational mode for process data exchange");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - PDO sync-managers (SM2/SM3) should be configured: %s",
                    pdo_configured_ ? "✓ FULFILLED" : "⚠ NOT FULFILLED (warning only)");
        if (has_pdo_entries) {
            TETHER_LOGI(TAG, "║    - PDO request counter  > 0: %s",
                        pdo_req_ok ? "✓ FULFILLED" : "✗ NOT FULFILLED");
            TETHER_LOGI(TAG, "║    - PDO reply counter    > 0: %s",
                        pdo_reply_ok ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        } else {
            TETHER_LOGI(TAG, "║    - PDO exchange check:     N/A (no PDO entries for this slave)");
        }
        TETHER_LOGI(TAG, "║    - FMMU configuration matches slave hardware: %s",
                    fmmu_ok ? "✓ FULFILLED" : "✗ NOT FULFILLED");
        TETHER_LOGI(TAG, "║  Status:     %s", (pdo_req_ok && pdo_reply_ok && fmmu_ok)
                    ? "Proceeding with transition"
                    : "HALTED — requirements not met");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }

    if (!pdo_configured_) {
        TETHER_LOGW( TAG,
            "Slave %u: Transitioning to OP without PDO sync-managers configured. "
            "This may cause issues with process data exchange.", index_);
    }

    if (has_pdo_entries && (!pdo_req_ok || !pdo_reply_ok)) {
        auto& pdo_mgr = master_.pdo();
        const uint32_t req   = pdo_mgr.getSlavePDORequestCount(index_);
        const uint32_t reply = pdo_mgr.getSlavePDOReplyCount(index_);
        TETHER_LOGE(TAG,
            "Slave %u: OP transition rejected — no PDO exchange after 100 ms "
            "(req=%u reply=%u). Start the motion loop or call exchangeAll() "
            "before requesting OP.",
            index_, req, reply);
        return SlaveError::TransportError;
    }

    if (!fmmu_ok) {
        TETHER_LOGE(TAG,
            "Slave %u: OP transition rejected — FMMU configuration mismatch "
            "(read from slave hardware does not match expected values)",
            index_);
        return SlaveError::TransportError;
    }

    // Request OP with Error Acknowledge bit (0x08 | 0x10 = 0x18)
    // Some slaves require the ACK bit to clear internal error latches.
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::OP) | 0x10)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to OP (transport error)", index_);
        return SlaveError::TransportError;
    }

    // Confirm OP (up to 5 s).  The slave may need continuous process data.
    for (int attempt = 0; attempt < 500; attempt++) {
        Tether::Platform::Clock::instance().delayMilliseconds(10);
        uint8_t state = 0;
        if (master_.readSlaveApplicationLayerState(index_, state)) {
            if (state == static_cast<uint8_t>(SlaveState::OP)) {
                if (slave_debug_flags_.stateMachine) {
                    TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
                    TETHER_LOGI(TAG, "║  Transition Result: Slave %u => OP SUCCESS                    ║", index_);
                    TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
                }
                // Debug gate checkpoint: OP confirmed
                master_.debugGate().notifyCheckpoint("state:op", index_);
                return SlaveError::Ok;
            }
            // If state dropped to INIT or PRE_OP, something went wrong
            if (state == static_cast<uint8_t>(SlaveState::INIT) ||
                state == static_cast<uint8_t>(SlaveState::PRE_OP)) {
                uint16_t al_code = 0;
                readALStatusCode(al_code);
                TETHER_LOGE(TAG, "Slave %u: Unexpected state 0x%02X during OP transition (AL status code: %s (0x%04X))",
                         index_, state, getALStatusCodeName(al_code), al_code);
                return SlaveError::TransportError;
            }
        }
    }

    uint16_t al_code = 0;
    readALStatusCode(al_code);
    TETHER_LOGE(TAG, "Slave %u: OP not confirmed after 5s (AL status code: %s (0x%04X))", index_, getALStatusCodeName(al_code), al_code);
    return SlaveError::TransportError;
}

SlaveError Slave::transitionToBoot() {
    if (slave_debug_flags_.stateMachine) {
        SlaveState current_state;
        readState(current_state);
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u                          ║", index_);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: %s => BOOT", slaveStateToString(current_state));
        TETHER_LOGI(TAG, "║  Reason:    Firmware update or bootstrap mode");
        TETHER_LOGI(TAG, "║  Requirements: None (BOOT is a special state)");
        TETHER_LOGI(TAG, "║  Status:     Fulfilled - proceeding with transition");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::BOOT))) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to BOOT (transport error)", index_);
        return SlaveError::TransportError;
    }
    
    if (slave_debug_flags_.stateMachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => BOOT SUCCESS                  ║", index_);
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

// -- State query -------------------------------------------------------------

SlaveError Slave::readState(SlaveState& state) {
    uint8_t raw = 0;
    if (!master_.readSlaveApplicationLayerState(index_, raw)) {
        return SlaveError::TransportError;
    }
    state = static_cast<SlaveState>(raw & 0x0F);
    return SlaveError::Ok;
}

SlaveError Slave::readALStatusCode(uint16_t& code) {
    uint16_t status = 0;
    if (!master_.readRegister(SlaveAddress(index_), reg::AL_STATUS_CODE, status)) {
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

// -- Watchdog ----------------------------------------------------------------

SlaveError Slave::configureWatchdogs(uint16_t pdi_timeout_100us,
                                              uint16_t pdata_timeout_100us) {
    if (!master_.configureWatchdogs(index_, pdi_timeout_100us, pdata_timeout_100us)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::disableWatchdogs() {
    if (!master_.disableWatchdogs(index_)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::readWatchdogStatus(uint8_t& wd_status,
                                              uint8_t& pdi_cnt,
                                              uint8_t& pdata_cnt) {
    if (!master_.readWatchdogStatus(index_, wd_status, pdi_cnt, pdata_cnt)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

// -- SDO convenience ---------------------------------------------------------

SlaveError Slave::sdoRead(uint16_t index, uint8_t subindex,
                                   void* data, size_t& size) {
    auto& sdo = master_.sdoManager(index_);
    size_t actual = 0;
    if (!sdo.readSync(index, subindex,
                      data, size, SDO::kDefaultSDOTimeoutMs, &actual)) {
        return SlaveError::SDOError;
    }
    size = actual;
    return SlaveError::Ok;
}

SlaveError Slave::sdoWrite(uint16_t index, uint8_t subindex,
                                    const void* data, size_t size) {
    auto& sdo = master_.sdoManager(index_);
    if (!sdo.writeSync(index, subindex,
                       data, size, {.timeout_ms = SDO::kDefaultSDOTimeoutMs})) {
        return SlaveError::SDOError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU8(uint16_t index, uint8_t sub, uint8_t& out) {
    auto& sdo = master_.sdoManager(index_);
    auto result = sdo.readU8(index, sub);
    if (!result.has_value()) return SlaveError::SDOError;
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU16(uint16_t index, uint8_t sub, uint16_t& out) {
    auto& sdo = master_.sdoManager(index_);
    auto result = sdo.readU16(index, sub);
    if (!result.has_value()) return SlaveError::SDOError;
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU32(uint16_t index, uint8_t sub, uint32_t& out) {
    auto& sdo = master_.sdoManager(index_);
    auto result = sdo.readU32(index, sub);
    if (!result.has_value()) return SlaveError::SDOError;
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val) {
    auto& sdo = master_.sdoManager(index_);
    auto result = sdo.writeU8(index, sub, val);
    if (!result.has_value()) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val) {
    auto& sdo = master_.sdoManager(index_);
    auto result = sdo.writeU16(index, sub, val);
    if (!result.has_value()) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val) {
    auto& sdo = master_.sdoManager(index_);
    auto result = sdo.writeU32(index, sub, val);
    if (!result.has_value()) return SlaveError::SDOError;
    return SlaveError::Ok;
}

// -- SII convenience ---------------------------------------------------------

SlaveError Slave::readSII(SII::SIIData& data) {
    if (!sii_cache_.isInitialized()) {
        // Lazy-init the SII cache through the master's SII reader
        // The SIIReader is created on-demand
        TETHER_LOGW( TAG,
            "Slave %u: SII cache not initialized — using direct read", index_);
        if (!SII::readSII(master_, index_, data)) {
            return SlaveError::SIIReadError;
        }
        return SlaveError::Ok;
    }
    if (!sii_cache_.parseFull(data)) {
        return SlaveError::SIIReadError;
    }
    return SlaveError::Ok;
}

void Slave::logSIISummary(const char* tag) {
    SII::SIIData data;
    if (readSII(data) == SlaveError::Ok) {
        SII::logSIISummary(data, index_, tag);
    } else {
        TETHER_LOGW( tag,
            "Slave %u: Failed to read SII for summary", index_);
    }
}

// ============================================================================
// PDO auto-configuration from SII
// ============================================================================

SlaveError Slave::registerPDOsFromSII(SIIPDOConfig& out_config) {
    SII::SIIData sii;
    if (readSII(sii) != SlaveError::Ok) {
        TETHER_LOGE(TAG, "Slave %u: Failed to read SII for PDO auto-config", index_);
        return SlaveError::SIIReadError;
    }

    // Find best-matching PDOs
    const SII::SIIPDO* rxpdo = nullptr;
    const SII::SIIPDO* txpdo = nullptr;

    for (const auto& pdo : sii.rx_pdos) {
        if (pdo.sync_manager == 2 || pdo.isDefault()) {
            rxpdo = &pdo;
            break;
        }
    }
    for (const auto& pdo : sii.tx_pdos) {
        if (pdo.sync_manager == 3 || pdo.isDefault()) {
            txpdo = &pdo;
            break;
        }
    }

    if (!rxpdo && !txpdo) {
        TETHER_LOGE(TAG, "Slave %u: No PDO data found in SII", index_);
        return SlaveError::PDOConfigFailed;
    }

    // Remove any existing entries for this slave to avoid duplicates
    PDO::PDOMapping& mapping = master_.pdo().mapping();
    mapping.remove_entries_for_slave(index_);

    // Allocate buffers and register entries
    out_config = SIIPDOConfig{};  // clear

    if (rxpdo) {
        uint16_t size = static_cast<uint16_t>(rxpdo->totalBytes());
        pdo_rx_buffer_.assign(size, 0);
        int idx = mapping.add_rxpdo(index_, pdo_rx_buffer_.data(), size,
                                    rxpdo->pdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register RxPDO mapping entry", index_);
            return SlaveError::PDOMappingFailed;
        }
        out_config.rxpdo_index = rxpdo->pdo_index;
        out_config.rxpdo_size  = size;
        out_config.has_rxpdo   = true;
        TETHER_LOGI(TAG, "Slave %u: Registered RxPDO 0x%04X (%u bytes) from SII", index_,
                    rxpdo->pdo_index, size);
    }

    if (txpdo) {
        uint16_t size = static_cast<uint16_t>(txpdo->totalBytes());
        pdo_tx_buffer_.assign(size, 0);
        int idx = mapping.add_txpdo(index_, pdo_tx_buffer_.data(), size,
                                    txpdo->pdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register TxPDO mapping entry", index_);
            return SlaveError::PDOMappingFailed;
        }
        out_config.txpdo_index = txpdo->pdo_index;
        out_config.txpdo_size  = size;
        out_config.has_txpdo   = true;
        TETHER_LOGI(TAG, "Slave %u: Registered TxPDO 0x%04X (%u bytes) from SII", index_,
                    txpdo->pdo_index, size);
    }

    // Finalize so SlaveConfig rxpdo_size / txpdo_size are updated
    master_.pdo().finalizeMapping(index_);

    return SlaveError::Ok;
}

SlaveError Slave::assignPDOs(const SIIPDOConfig& config) {
    if (!config.has_rxpdo && !config.has_txpdo) {
        TETHER_LOGE(TAG, "Slave %u: assignPDOs called with empty config (zero PDOs available), skipping", index_);
        return SlaveError::Ok;
    }

    auto& sdo = master_.sdoManager(index_);
    uint8_t zero = 0;
    uint8_t one  = 1;
    bool sdo_ok = true;

    if (config.has_rxpdo) {
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, zero).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to clear SM2 PDO count (may be fixed)", index_);
        }
        if (!sdo.writeU16(CiA301::SyncManager2PDOAssign, 1, config.rxpdo_index).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to assign RxPDO 0x%04X to SM2", index_, config.rxpdo_index);
            sdo_ok = false;
        }
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, one).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to set SM2 PDO count", index_);
        }
        TETHER_LOGI(TAG, "Slave %u: Assigned RxPDO 0x%04X to SM2 (0x1C12)", index_, config.rxpdo_index);
    }

    if (config.has_txpdo) {
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, zero).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to clear SM3 PDO count (may be fixed)", index_);
        }
        if (!sdo.writeU16(CiA301::SyncManager3PDOAssign, 1, config.txpdo_index).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to assign TxPDO 0x%04X to SM3", index_, config.txpdo_index);
            sdo_ok = false;
        }
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, one).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to set SM3 PDO count", index_);
        }
        TETHER_LOGI(TAG, "Slave %u: Assigned TxPDO 0x%04X to SM3 (0x1C13)", index_, config.txpdo_index);
    }

    if (!sdo_ok) {
        TETHER_LOGW(TAG, "Slave %u: PDO assignment had SDO failures; continuing anyway", index_);
    }

    return SlaveError::Ok;
}

SlaveError Slave::registerFixedPDOs(const SIIPDOConfig& config) {
    if (!config.has_rxpdo && !config.has_txpdo) {
        TETHER_LOGW(TAG, "Slave %u: registerFixedPDOs called with empty config, skipping", index_);
        return SlaveError::Ok;
    }

    // Remove any existing entries for this slave to avoid duplicates
    PDO::PDOMapping& mapping = master_.pdo().mapping();
    mapping.remove_entries_for_slave(index_);

    if (config.has_rxpdo) {
        pdo_rx_buffer_.assign(config.rxpdo_size, 0);
        int idx = mapping.add_rxpdo(index_, pdo_rx_buffer_.data(), config.rxpdo_size,
                                    config.rxpdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register fixed RxPDO 0x%04X mapping entry", index_, config.rxpdo_index);
            return SlaveError::PDOMappingFailed;
        }
        TETHER_LOGI(TAG, "Slave %u: Registered fixed RxPDO 0x%04X (%u bytes)", index_,
                    config.rxpdo_index, config.rxpdo_size);
    }

    if (config.has_txpdo) {
        pdo_tx_buffer_.assign(config.txpdo_size, 0);
        int idx = mapping.add_txpdo(index_, pdo_tx_buffer_.data(), config.txpdo_size,
                                    config.txpdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register fixed TxPDO 0x%04X mapping entry", index_, config.txpdo_index);
            return SlaveError::PDOMappingFailed;
        }
        TETHER_LOGI(TAG, "Slave %u: Registered fixed TxPDO 0x%04X (%u bytes)", index_,
                    config.txpdo_index, config.txpdo_size);
    }

    master_.pdo().finalizeMapping(index_);
    return SlaveError::Ok;
}

// ============================================================================
// Custom PDO mapping
// ============================================================================

SlaveError Slave::configureCustomRxPDO(
    uint16_t pdo_index,
    std::initializer_list<CustomPDOMappingEntry> entries)
{
    return configureCustomTxPDO(pdo_index, entries, PDO::PDODirection::RxPDO);
}

SlaveError Slave::configureCustomTxPDO(
    uint16_t pdo_index,
    std::initializer_list<CustomPDOMappingEntry> entries)
{
    return configureCustomTxPDO(pdo_index, entries, PDO::PDODirection::TxPDO);
}

SlaveError Slave::configureCustomTxPDO(
    uint16_t pdo_index,
    std::initializer_list<CustomPDOMappingEntry> entries,
    PDO::PDODirection direction)
{
    if (entries.size() == 0) {
        TETHER_LOGE(TAG, "configureCustomPDO: empty entry list for PDO 0x%04X", pdo_index);
        return SlaveError::PDOMappingFailed;
    }
    if (entries.size() > 255) {
        TETHER_LOGE(TAG, "configureCustomPDO: too many entries (%zu) for PDO 0x%04X",
                    entries.size(), pdo_index);
        return SlaveError::PDOMappingFailed;
    }

    // Build field layout and compute total size
    std::vector<CustomPDOFieldLayout> fields;
    fields.reserve(entries.size());
    uint16_t offset = 0;
    bool any_unresolved = false;

    for (const auto& e : entries) {
        uint8_t sz = e.resolvedSize();
        if (sz == 0) {
            TETHER_LOGE(TAG, "configureCustomPDO: cannot infer size for entry 0x%04X:0x%02X (%s) — specify explicit byte size",
                        e.entry->index, e.entry->subindex, e.entry->name ? e.entry->name : "?");
            any_unresolved = true;
        }
        fields.push_back({e.entry, offset, sz});
        offset += sz;
    }
    if (any_unresolved) {
        return SlaveError::PDOMappingFailed;
    }

    uint16_t total_size = offset;
    const char* dir_str = (direction == PDO::PDODirection::RxPDO) ? "RxPDO" : "TxPDO";
    TETHER_LOGI(TAG, "Slave %u: Configuring custom %s 0x%04X: %zu entries, %u bytes",
                index_, dir_str, pdo_index, entries.size(), total_size);

    // Write SDO mapping to slave
    auto err = sdoWriteU8(pdo_index, 0x00, 0);  // clear count
    if (err != SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to clear PDO mapping count for 0x%04X", pdo_index);
        return err;
    }

    uint8_t sub = 1;
    for (const auto& e : entries) {
        uint8_t sz = e.resolvedSize();
        uint32_t val = encodePDOMappingValue(e.entry, sz);
        err = sdoWriteU32(pdo_index, sub, val);
        if (err != SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to write PDO mapping entry %u for 0x%04X", sub, pdo_index);
            return err;
        }
        TETHER_LOGI(TAG, "  %s 0x%04X sub %u: 0x%08X (idx=0x%04X sub=0x%02X %u bytes)",
                    dir_str, pdo_index, sub, val, e.entry->index, e.entry->subindex, sz);
        ++sub;
    }

    err = sdoWriteU8(pdo_index, 0x00, static_cast<uint8_t>(entries.size()));
    if (err != SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to set PDO mapping count for 0x%04X", pdo_index);
        return err;
    }

    storeCustomPDOInfo(pdo_index, direction, total_size, std::move(fields));

    return SlaveError::Ok;
}

void Slave::storeCustomPDOInfo(
    uint16_t pdo_index,
    PDO::PDODirection direction,
    uint16_t total_size,
    std::vector<CustomPDOFieldLayout>&& fields)
{
    // Remove any existing entry with the same pdo_index
    for (auto it = custom_pdo_infos_.begin(); it != custom_pdo_infos_.end(); ++it) {
        if (it->pdo_index == pdo_index) {
            custom_pdo_infos_.erase(it);
            break;
        }
    }
    CustomPDOInfo info;
    info.pdo_index = pdo_index;
    info.direction = direction;
    info.total_size = total_size;
    info.fields = std::move(fields);
    info.mapping_entry_index = -1;
    custom_pdo_infos_.push_back(std::move(info));
}

SlaveError Slave::applyCustomPDOs() {
    if (custom_pdo_infos_.empty()) {
        TETHER_LOGW(TAG, "Slave %u: applyCustomPDOs called with no custom PDOs configured", index_);
        return SlaveError::Ok;
    }

    // Remove existing PDO mapping entries for this slave
    PDO::PDOMapping& mapping = master_.pdo().mapping();
    mapping.remove_entries_for_slave(index_);

    // Register each custom PDO
    std::vector<uint16_t> rx_indices, tx_indices;
    for (auto& info : custom_pdo_infos_) {
        info.buffer.assign(info.total_size, 0);
        int idx;
        if (info.direction == PDO::PDODirection::RxPDO) {
            idx = mapping.add_rxpdo(index_, info.buffer.data(), info.total_size,
                                    info.pdo_index, PDO::PDOAddressMode::Position);
            rx_indices.push_back(info.pdo_index);
        } else {
            idx = mapping.add_txpdo(index_, info.buffer.data(), info.total_size,
                                    info.pdo_index, PDO::PDOAddressMode::Position);
            tx_indices.push_back(info.pdo_index);
        }
        if (idx < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register custom PDO 0x%04X", index_, info.pdo_index);
            return SlaveError::PDOMappingFailed;
        }
        info.mapping_entry_index = idx;
        TETHER_LOGI(TAG, "Slave %u: Registered custom PDO 0x%04X (%u bytes, entry %d)",
                    index_, info.pdo_index, info.total_size, idx);
    }

    // Write PDO assignment SDOs (0x1C12 for Rx, 0x1C13 for Tx)
    auto& sdo = master_.sdoManager(index_);
    bool sdo_ok = true;

    if (!rx_indices.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to clear SM2 PDO count", index_);
        }
        for (size_t i = 0; i < rx_indices.size(); i++) {
            if (!sdo.writeU16(CiA301::SyncManager2PDOAssign, static_cast<uint8_t>(i + 1),
                              rx_indices[i]).has_value()) {
                TETHER_LOGE(TAG, "Slave %u: Failed to assign RxPDO 0x%04X to SM2", index_, rx_indices[i]);
                sdo_ok = false;
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, static_cast<uint8_t>(rx_indices.size())).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to set SM2 PDO count", index_);
        }
        TETHER_LOGI(TAG, "Slave %u: Assigned %zu RxPDO(s) to SM2 (0x1C12)", index_, rx_indices.size());
    }

    if (!tx_indices.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to clear SM3 PDO count", index_);
        }
        for (size_t i = 0; i < tx_indices.size(); i++) {
            if (!sdo.writeU16(CiA301::SyncManager3PDOAssign, static_cast<uint8_t>(i + 1),
                              tx_indices[i]).has_value()) {
                TETHER_LOGE(TAG, "Slave %u: Failed to assign TxPDO 0x%04X to SM3", index_, tx_indices[i]);
                sdo_ok = false;
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, static_cast<uint8_t>(tx_indices.size())).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: Failed to set SM3 PDO count", index_);
        }
        TETHER_LOGI(TAG, "Slave %u: Assigned %zu TxPDO(s) to SM3 (0x1C13)", index_, tx_indices.size());
    }

    if (!sdo_ok) {
        TETHER_LOGW(TAG, "Slave %u: Custom PDO assignment had SDO failures; continuing anyway", index_);
    }

    // Finalize mapping to update SM lengths
    master_.pdo().finalizeMapping(index_);

    return SlaveError::Ok;
}

const uint8_t* Slave::customPDOData(uint16_t pdo_index) const {
    for (const auto& info : custom_pdo_infos_) {
        if (info.pdo_index == pdo_index) {
            return info.buffer.data();
        }
    }
    return nullptr;
}

const uint8_t* Slave::customPDOFieldRaw(uint16_t pdo_index, size_t field_index) const {
    for (const auto& info : custom_pdo_infos_) {
        if (info.pdo_index == pdo_index) {
            if (field_index >= info.fields.size()) return nullptr;
            return info.buffer.data() + info.fields[field_index].offset;
        }
    }
    return nullptr;
}

const uint8_t* Slave::customPDOField(
    uint16_t pdo_index,
    const ObjectDictionary::ObjectDictionaryEntry* entry) const {
    for (const auto& info : custom_pdo_infos_) {
        if (info.pdo_index == pdo_index) {
            for (const auto& f : info.fields) {
                if (f.entry == entry) {
                    return info.buffer.data() + f.offset;
                }
            }
        }
    }
    return nullptr;
}

// ============================================================================
// NonExistingSlave
// ============================================================================

NonExistingSlave::NonExistingSlave(Master& master, uint16_t index)
    : Slave(master, index)
{
}

void NonExistingSlave::logCritical(const char* method) const {
    TETHER_LOGE( TAG,
        "CRITICAL: %s() called on non-existing slave %u. "
        "Check getDiscoveredSlaveCount() before accessing slaves. "
        "Valid range: 0 to %u.",
        method, index_, master_.getDiscoveredSlaveCount() > 0
            ? static_cast<unsigned>(master_.getDiscoveredSlaveCount() - 1) : 0u);
}

SlaveError NonExistingSlave::configureMailbox(Tether::Platform::LogLevel) {
    logCritical("configureMailbox"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configureMailbox(const MailboxSyncManagerConfig&,
                                               const MailboxSyncManagerConfig&, uint16_t) {
    logCritical("configureMailbox"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::assumeMailboxAlreadyConfigured() {
    logCritical("assumeMailboxAlreadyConfigured");
}
SlaveError NonExistingSlave::configurePDOSyncManagers() {
    logCritical("configurePDOSyncManagers"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configurePDOSyncManagers(uint16_t, uint16_t, uint8_t,
                                                       uint16_t, uint16_t, uint8_t) {
    logCritical("configurePDOSyncManagers"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::assumePDOAlreadyConfigured() {
    logCritical("assumePDOAlreadyConfigured");
}
SlaveError NonExistingSlave::registerPDOsFromSII(SIIPDOConfig&) {
    logCritical("registerPDOsFromSII"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::assignPDOs(const SIIPDOConfig&) {
    logCritical("assignPDOs"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::registerFixedPDOs(const SIIPDOConfig&) {
    logCritical("registerFixedPDOs"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configureCustomRxPDO(uint16_t, std::initializer_list<CustomPDOMappingEntry>) {
    logCritical("configureCustomRxPDO"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configureCustomTxPDO(uint16_t, std::initializer_list<CustomPDOMappingEntry>) {
    logCritical("configureCustomTxPDO"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::applyCustomPDOs() {
    logCritical("applyCustomPDOs"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::transitionTo(SlaveState) {
    logCritical("transitionTo"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::transitionToInit() {
    logCritical("transitionToInit"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::transitionToPreOp() {
    logCritical("transitionToPreOp"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::transitionToSafeOp() {
    logCritical("transitionToSafeOp"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::transitionToOp() {
    logCritical("transitionToOp"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::transitionToBoot() {
    logCritical("transitionToBoot"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::readState(SlaveState&) {
    logCritical("readState"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::readALStatusCode(uint16_t&) {
    logCritical("readALStatusCode"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configureWatchdogs(uint16_t, uint16_t) {
    logCritical("configureWatchdogs"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::disableWatchdogs() {
    logCritical("disableWatchdogs"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::readWatchdogStatus(uint8_t&, uint8_t&, uint8_t&) {
    logCritical("readWatchdogStatus"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoRead(uint16_t, uint8_t, void*, size_t&) {
    logCritical("sdoRead"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoWrite(uint16_t, uint8_t, const void*, size_t) {
    logCritical("sdoWrite"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoReadU8(uint16_t, uint8_t, uint8_t&) {
    logCritical("sdoReadU8"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoReadU16(uint16_t, uint8_t, uint16_t&) {
    logCritical("sdoReadU16"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoReadU32(uint16_t, uint8_t, uint32_t&) {
    logCritical("sdoReadU32"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoWriteU8(uint16_t, uint8_t, uint8_t) {
    logCritical("sdoWriteU8"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoWriteU16(uint16_t, uint8_t, uint16_t) {
    logCritical("sdoWriteU16"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::sdoWriteU32(uint16_t, uint8_t, uint32_t) {
    logCritical("sdoWriteU32"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::readSII(SII::SIIData&) {
    logCritical("readSII"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::logSIISummary(const char*) {
    logCritical("logSIISummary");
}

SyncManagerAccessor NonExistingSlave::sm(uint8_t smIndex) {
    logCritical("sm");
    // Still returns an accessor — all its read/SDO methods will fail gracefully
    return SyncManagerAccessor(*this, smIndex);
}

// ============================================================================
// Slave::sm()
// ============================================================================

SyncManagerAccessor Slave::sm(uint8_t smIndex) {
    return SyncManagerAccessor(*this, smIndex);
}

} // namespace EtherCAT
