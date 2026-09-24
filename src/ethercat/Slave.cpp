/**
 * @file Slave.cpp
 * @brief Slave and NonExistingSlave implementation
 */

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/ESITypes.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "SlaveESIHelpers.hpp"
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
    : master_(&master), index_(index)
{
}

Slave::~Slave()
{
    // Clear the back-pointer to the owning master so nothing can accidentally
    // use it while this child object is being torn down.
    master_ = nullptr;
}

uint16_t Slave::adp() const {
    return Master::adpForSlaveIndex(index_);
}

void Slave::setName(std::string name) {
    master_->setSlaveName(index_, std::move(name));
}

std::string_view Slave::name() const {
    return master_->slaveName(index_);
}

std::string Slave::logPrefix() const {
    return master_->slaveLogPrefix(index_);
}

bool Slave::apwr(uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms) {
    return master_->writeRegister(SlaveAddress(index_), ado, data, len, timeout_ms);
}

bool Slave::aprd(uint16_t ado, void* out, uint16_t len, unsigned int timeout_ms) {
    return master_->readRegister(SlaveAddress(index_), ado, out, len, timeout_ms);
}

// -- Mailbox configuration ---------------------------------------------------

SlaveError Slave::configureMailbox(Tether::Platform::LogLevel log_level) {
    if (no_mailbox_) {
        TETHER_LOGE( TAG,
            "{}: slave was declared mailbox-less via markNoMailbox() — "
            "refusing configureMailbox()", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }
    if (!master_->autoConfigureMailbox(index_, log_level)) {
        TETHER_LOGE( TAG,
            "{}: Failed to auto-configure mailbox from SII", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Mailbox configured from SII", logPrefix().c_str());
    // Debug gate checkpoint: mailbox configured
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
    return SlaveError::Ok;
}

SlaveError Slave::configureMailbox(
    const MailboxSyncManagerConfig& mbox_out,
    const MailboxSyncManagerConfig& mbox_in,
    uint16_t protocols)
{
    if (no_mailbox_) {
        TETHER_LOGE( TAG,
            "{}: slave was declared mailbox-less via markNoMailbox() — "
            "refusing configureMailbox()", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }
    master_->setMailboxOverride(index_,
                               mbox_in.address, mbox_in.length,
                               mbox_out.address, mbox_out.length,
                               protocols);
    // Configure SDO manager with these mailbox params
    master_->sdoManager(index_).configureMailbox(
        mbox_in.address, mbox_in.length,
        mbox_out.address, mbox_out.length);

    // Write mailbox SM registers to slave ESC (same as autoConfigureMailbox)
    auto& pdo = master_->pdoForSlave(index_);
    auto* slave_configs = pdo.slaveConfigs();
    if (index_ < PDO::kMaxPDOSlaves) {
        slave_configs[index_].sm[0] = PDO::SyncManagerConfig::mailbox_write(
            mbox_in.address, mbox_in.length);
        slave_configs[index_].sm[1] = PDO::SyncManagerConfig::mailbox_read(
            mbox_out.address, mbox_out.length);
        if (!pdo.configureSlavesSMs(index_)) {
            TETHER_LOGE(TAG, "{}: Failed to write mailbox SM registers", logPrefix().c_str());
            return SlaveError::MailboxConfigFailed;
        }

        // SM1 may contain stale/junk data left over from slave firmware boot.
        // Drain it now so the slave has a free outbound mailbox before the
        // first SDO exchange.
        (void)master_->drainSlaveMailbox(index_);
    }

    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Mailbox configured (wr=0x{:04X}/{}, rd=0x{:04X}/{}, proto=0x{:04X})",
        logPrefix().c_str(), mbox_in.address, mbox_in.length,
        mbox_out.address, mbox_out.length, protocols);
    // Debug gate checkpoint: mailbox configured
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
    return SlaveError::Ok;
}


SlaveError Slave::configureMailbox(
    const ESIFile& esi,
    Tether::Platform::LogLevel log_level)
{
    if (esi.empty()) {
        TETHER_LOGE(TAG, "{}: ESI file is empty — cannot configure mailbox from ESI", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }

    // Match device by SII identity
    auto id = readIdentityForESIMatch(*master_, index_);
    const ESI::DeviceInfo* dev = esi.findDevice(id.vendorId, id.productCode);
    if (!dev) {
        TETHER_LOGE(TAG, "{}: ESI file has no devices", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }

    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(TAG, "{}: ESI device '{}' matched (vendor=0x{:08X} product=0x{:08X})",
                    logPrefix().c_str(), dev->name.c_str(), dev->vendorId, dev->productCode);
    }

    // Find MBoxOut (master→slave write, SM0) and MBoxIn (slave→master read, SM1)
    const ESI::SyncManagerEntry* mbxOut = findSmByName(*dev, "MBoxOut");
    const ESI::SyncManagerEntry* mbxIn  = findSmByName(*dev, "MBoxIn");

    MailboxSyncManagerConfig mbox_out{};
    MailboxSyncManagerConfig mbox_in{};

    if (mbxOut) {
        mbox_out.address = mbxOut->startAddress;
        mbox_out.length  = mbxOut->defaultSize;
    } else {
        TETHER_LOGW(TAG, "{}: ESI has no MBoxOut sync manager — using defaults", logPrefix().c_str());
        mbox_out.address = 0x1000;
        mbox_out.length  = 256;
    }

    if (mbxIn) {
        mbox_in.address = mbxIn->startAddress;
        mbox_in.length  = mbxIn->defaultSize;
    } else {
        TETHER_LOGW(TAG, "{}: ESI has no MBoxIn sync manager — using defaults", logPrefix().c_str());
        mbox_in.address = 0x1200;
        mbox_in.length  = 256;
    }

    // Protocol flags
    uint16_t protocols = 0x0004; // default: CoE
    if (dev->mailbox.protocols.has_value()) {
        protocols = *dev->mailbox.protocols;
    }

    TETHER_LOGI(TAG, "{}: Configuring mailbox from ESI (out=0x{:04X}/{}, in=0x{:04X}/{}, proto=0x{:04X})",
                logPrefix().c_str(), mbox_out.address, mbox_out.length,
                mbox_in.address, mbox_in.length, protocols);

    return configureMailbox(mbox_out, mbox_in, protocols);
}

void Slave::assumeMailboxAlreadyConfigured() {
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Assuming mailbox already configured", logPrefix().c_str());

    // Best-effort drain: the slave firmware may have left stale data in SM1
    // from boot or a previous session.  If the CoE subsystem doesn't have
    // mailbox address info (common when the firmware truly pre-configured
    // everything), the drain simply logs a warning and continues.
    if (!drainMailbox()) {
        TETHER_LOGW(TAG,
            "{}: Mailbox drain after assumeMailboxAlreadyConfigured() "
            "did not complete — stale responses may occur on first SDO exchange",
            logPrefix().c_str());
    }

    // Debug gate checkpoint: mailbox configured
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
}

void Slave::markNoMailbox() {
    no_mailbox_ = true;
    // The PRE_OP prerequisite is vacuously satisfied — there is no
    // mailbox to configure or drain.
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Declared mailbox-less — skipping all mailbox handling",
        logPrefix().c_str());
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
}

bool Slave::drainMailbox(unsigned int max_drain) {
    if (no_mailbox_) return true;   // nothing to drain
    return master_->drainSlaveMailbox(index_, max_drain);
}

// -- PDO SM configuration ----------------------------------------------------

SlaveError Slave::configurePDOSyncManagers() {
    if (!master_->configureProcessDataSyncManagersFromSii(index_)) {
        TETHER_LOGE( TAG,
            "{}: Failed to configure PDO sync-managers from SII", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: PDO sync-managers configured from SII", logPrefix().c_str());

    if (slave_debug_flags_.pdoSm) {
        EtherCAT::debugPDOSyncManagerConfiguration(*master_, index_, TAG);
    }

    return SlaveError::Ok;
}

SlaveError Slave::configurePDOSyncManagers(
    uint16_t sm2_addr, uint16_t sm2_len, uint8_t sm2_ctrl,
    uint16_t sm3_addr, uint16_t sm3_len, uint8_t sm3_ctrl)
{
    auto& pdo = master_->pdoForSlave(index_);
    auto* cfgs = pdo.slaveConfigs();
    if (index_ >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE( TAG,
            "{}: Index exceeds Tether internal max PDO slaves ({}). "
            "This is a Tether limit, not a slave limit. "
            "Increase ECAT_PDO_MAX_SLAVES in EtherCATConfig.hpp.",
            logPrefix().c_str(), PDO::kMaxPDOSlaves);
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
            "{}: Failed to write PDO SM registers", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    pdo_configured_ = true;
    return SlaveError::Ok;
}

SlaveError Slave::configurePDOSyncManagers(const ESIFile& esi) {
    if (esi.empty()) {
        TETHER_LOGE(TAG, "{}: ESI file is empty — cannot configure PDO SMs from ESI", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    auto id = readIdentityForESIMatch(*master_, index_);
    const ESI::DeviceInfo* dev = esi.findDevice(id.vendorId, id.productCode);
    if (!dev) {
        TETHER_LOGE(TAG, "{}: ESI file has no devices", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    // Find Outputs (SM2) and Inputs (SM3) sync managers
    const ESI::SyncManagerEntry* smOut = findSmByName(*dev, "Outputs");
    const ESI::SyncManagerEntry* smIn  = findSmByName(*dev, "Inputs");

    if (!smOut || !smIn) {
        TETHER_LOGE(TAG, "{}: ESI missing Outputs/Inputs sync managers", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    uint8_t sm2_ctrl = std::bit_cast<uint8_t>(smOut->control);
    uint8_t sm3_ctrl = std::bit_cast<uint8_t>(smIn->control);

    TETHER_LOGI(TAG, "{}: Configuring PDO SMs from ESI (SM2=0x{:04X}/{} ctrl=0x{:02X}, SM3=0x{:04X}/{} ctrl=0x{:02X})",
                logPrefix().c_str(), smOut->startAddress, smOut->defaultSize, sm2_ctrl,
                smIn->startAddress, smIn->defaultSize, sm3_ctrl);

    return configurePDOSyncManagers(
        smOut->startAddress, smOut->defaultSize, sm2_ctrl,
        smIn->startAddress, smIn->defaultSize, sm3_ctrl);
}

void Slave::assumePDOAlreadyConfigured() {
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Assuming PDO sync-managers already configured", logPrefix().c_str());
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

    // Confirm OP (up to 5 s).  The slave may need continuous process data.
    for (int attempt = 0; attempt < 500; attempt++) {
        if (master_->isCancelRequested()) {
            TETHER_LOGI(TAG, "{}: OP confirmation cancelled", logPrefix().c_str());
            return SlaveError::Cancelled;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(10);
        uint8_t state = 0;
        if (master_->readSlaveApplicationLayerState(index_, state)) {
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
    TETHER_LOGE(TAG, "{}: OP not confirmed after 5s (AL status code: {} (0x{:04X}))", logPrefix().c_str(), getALStatusCodeName(al_code), al_code);
    return SlaveError::TransportError;
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

// -- Watchdog ----------------------------------------------------------------

SlaveError Slave::configureWatchdogs(uint16_t pdi_timeout_100us,
                                              uint16_t pdata_timeout_100us) {
    if (!master_->configureWatchdogs(index_, pdi_timeout_100us, pdata_timeout_100us)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::disableWatchdogs() {
    if (!master_->disableWatchdogs(index_)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::readWatchdogStatus(uint8_t& wd_status,
                                              uint8_t& pdi_cnt,
                                              uint8_t& pdata_cnt) {
    if (!master_->readWatchdogStatus(index_, wd_status, pdi_cnt, pdata_cnt)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

// -- SDO convenience ---------------------------------------------------------

SlaveError Slave::sdoRead(uint16_t index, uint8_t subindex,
                                   void* data, size_t& size) {
    auto& sdo = master_->sdoManager(index_);
    size_t actual = 0;
    if (!sdo.readSync(index, subindex,
                      data, size, SDO::kDefaultSDOTimeoutMs, &actual)) {
        return (sdo.lastSdoAbortCode() != 0) ? SlaveError::SDOAborted
                                              : SlaveError::SDOError;
    }
    size = actual;
    return SlaveError::Ok;
}

SlaveError Slave::sdoWrite(uint16_t index, uint8_t subindex,
                                    const void* data, size_t size) {
    auto& sdo = master_->sdoManager(index_);
    if (!sdo.writeSync(index, subindex,
                       data, size, {.timeout_ms = SDO::kDefaultSDOTimeoutMs})) {
        return (sdo.lastSdoAbortCode() != 0) ? SlaveError::SDOAborted
                                              : SlaveError::SDOError;
    }
    return SlaveError::Ok;
}

// Map a failed CoE typed read/write to a SlaveError.  ShuttingDown (master
// cancellation via requestCancel()) maps to Cancelled so callers can tell
// an expected shutdown failure apart from a real SDO problem.
static SlaveError sdoCoeErrorToSlaveError(CoE::CoEManager& sdo, CoE::CoEError err) {
    if (err.code == CoE::CoEErrorCode::ShuttingDown) return SlaveError::Cancelled;
    return (sdo.lastSdoAbortCode() != 0) ? SlaveError::SDOAborted
                                        : SlaveError::SDOError;
}

SlaveError Slave::sdoReadU8(uint16_t index, uint8_t sub, uint8_t& out) {
    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.readU8(index, sub);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU16(uint16_t index, uint8_t sub, uint16_t& out) {
    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.readU16(index, sub);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU32(uint16_t index, uint8_t sub, uint32_t& out) {
    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.readU32(index, sub);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val) {
    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.writeU8(index, sub, val);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val) {
    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.writeU16(index, sub, val);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val) {
    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.writeU32(index, sub, val);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    return SlaveError::Ok;
}

uint32_t Slave::lastSdoAbortCode() const {
    return master_->sdoManager(index_).lastSdoAbortCode();
}

bool Slave::lastSdoWasDownload() const {
    return master_->sdoManager(index_).lastSdoWasDownload();
}

size_t Slave::lastSdoAttemptedLength() const {
    return master_->sdoManager(index_).lastSdoAttemptedLength();
}

// -- SII convenience ---------------------------------------------------------

SlaveError Slave::readSII(SII::SIIData& data) {
#if TETHER_ENABLE_SII
    if (!sii().isInitialised()) {
        TETHER_LOGW( TAG,
            "{}: SII manager not initialised — using direct read", logPrefix().c_str());
        if (!SII::readSII(*master_, index_, data)) {
            return SlaveError::SIIReadError;
        }
        return SlaveError::Ok;
    }
    if (!sii().parseFull(data)) {
        return SlaveError::SIIReadError;
    }
    return SlaveError::Ok;
#else
    TETHER_LOGE(TAG, "{}: SII support is disabled", logPrefix().c_str());
    return SlaveError::SIIReadError;
#endif
}

void Slave::logSIISummary(const char* tag) {
#if TETHER_ENABLE_SII
    SII::SIIData data;
    if (readSII(data) == SlaveError::Ok) {
        SII::logSIISummary(data, logPrefix(), tag);
    } else {
        TETHER_LOGW( tag,
            "{}: Failed to read SII for summary", logPrefix().c_str());
    }
#else
    (void)tag;
    TETHER_LOGE(TAG, "{}: SII support is disabled", logPrefix().c_str());
#endif
}

// ============================================================================

SyncManagerAccessor Slave::sm(uint8_t smIndex) {
    return SyncManagerAccessor(*this, smIndex);
}

} // namespace EtherCAT
