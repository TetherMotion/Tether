/**
 * @file EtherCATSlave.cpp
 * @brief EtherCATSlave and NonExistingSlave implementation
 */

#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>

namespace EtherCAT {

static const char* TAG = "EtherCATSlave";

// Global debug flag for ethercat-statemachine (shared with EtherCATMaster)
bool g_debug_statemachine = false;

void enableStateMachineDebug(bool enable) {
    g_debug_statemachine = enable;
}

// ============================================================================
// EtherCATSlave
// ============================================================================

EtherCATSlave::EtherCATSlave(EtherCATMaster& master, uint16_t index)
    : master_(master), index_(index)
{
}

uint16_t EtherCATSlave::adp() const {
    return EtherCATMaster::adpForSlaveIndex(index_);
}

// -- Mailbox configuration ---------------------------------------------------

SlaveError EtherCATSlave::configureMailbox(Tether::Platform::LogLevel log_level) {
    if (!master_.autoConfigureMailbox(index_, log_level)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to auto-configure mailbox from SII", index_);
        return SlaveError::MailboxConfigFailed;
    }
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Mailbox configured from SII", index_);
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::configureMailbox(
    uint16_t wr_addr, uint16_t wr_len,
    uint16_t rd_addr, uint16_t rd_len,
    uint16_t protocols)
{
    master_.setMailboxOverride(index_, wr_addr, wr_len, rd_addr, rd_len, protocols);
    // Configure SDO manager with these mailbox params
    master_.sdoManager().configureSlaveMailbox(index_, wr_addr, wr_len, rd_addr, rd_len);
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Mailbox configured (wr=0x%04X/%u, rd=0x%04X/%u, proto=0x%04X)",
        index_, wr_addr, wr_len, rd_addr, rd_len, protocols);
    return SlaveError::Ok;
}

void EtherCATSlave::assumeMailboxAlreadyConfigured() {
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Assuming mailbox already configured", index_);
}

// -- PDO SM configuration ----------------------------------------------------

SlaveError EtherCATSlave::configurePDOSyncManagers() {
    if (!master_.configureProcessDataSyncManagersFromSii(index_)) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to configure PDO sync-managers from SII", index_);
        return SlaveError::PDOConfigFailed;
    }
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: PDO sync-managers configured from SII", index_);
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::configurePDOSyncManagers(
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
    cfgs[index_].sm[2].control = sm2_ctrl;
    cfgs[index_].sm[2].enable = 1;
    cfgs[index_].sm[2].type = PDO::SyncManagerType::ProcessOutput;

    cfgs[index_].sm[3].phys_start_addr = sm3_addr;
    cfgs[index_].sm[3].length = sm3_len;
    cfgs[index_].sm[3].control = sm3_ctrl;
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

void EtherCATSlave::assumePDOAlreadyConfigured() {
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "Slave %u: Assuming PDO sync-managers already configured", index_);
}

// -- State transitions -------------------------------------------------------

SlaveError EtherCATSlave::transitionTo(SlaveState target) {
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

SlaveError EtherCATSlave::transitionToInit() {
    if (g_debug_statemachine) {
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
    
    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => INIT SUCCESS                  ║", index_);
        TETHER_LOGI(TAG, "║  Configuration flags reset: mailbox=false, pdo=false          ║");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::transitionToPreOp() {
    if (g_debug_statemachine) {
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
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::PRE_OP))) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to PRE_OP (transport error)", index_);
        return SlaveError::TransportError;
    }
    
    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => PRE_OP SUCCESS                ║", index_);
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::transitionToSafeOp() {
    if (g_debug_statemachine) {
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
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::SAFE_OP))) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to SAFE_OP (transport error)", index_);
        return SlaveError::TransportError;
    }
    
    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => SAFE_OP SUCCESS               ║", index_);
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::transitionToOp() {
    if (g_debug_statemachine) {
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
        TETHER_LOGI(TAG, "║  Status:     Proceeding with transition (PDO config is optional for OP)");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    if (!pdo_configured_) {
        TETHER_LOGW( TAG,
            "Slave %u: Transitioning to OP without PDO sync-managers configured. "
            "This may cause issues with process data exchange.", index_);
    }
    if (!master_.requestSlaveApplicationLayerState(index_, static_cast<uint8_t>(SlaveState::OP))) {
        TETHER_LOGE( TAG,
            "Slave %u: Failed to transition to OP (transport error)", index_);
        return SlaveError::TransportError;
    }
    
    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => OP SUCCESS                    ║", index_);
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::transitionToBoot() {
    if (g_debug_statemachine) {
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
    
    if (g_debug_statemachine) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => BOOT SUCCESS                  ║", index_);
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return SlaveError::Ok;
}

// -- State query -------------------------------------------------------------

SlaveError EtherCATSlave::readState(SlaveState& state) {
    uint8_t raw = 0;
    if (!master_.readSlaveApplicationLayerState(index_, raw)) {
        return SlaveError::TransportError;
    }
    state = static_cast<SlaveState>(raw & 0x0F);
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::readALStatusCode(uint16_t& code) {
    uint16_t status = 0;
    if (!master_.readRegister(SlaveAddress(index_), reg::AL_STATUS_CODE, status)) {
        return SlaveError::TransportError;
    }
    code = status;
    return SlaveError::Ok;
}

std::optional<SlaveState> EtherCATSlave::ALState() {
    SlaveState st{};
    if (readState(st) != SlaveError::Ok) return std::nullopt;
    return st;
}

std::optional<uint16_t> EtherCATSlave::ALCode() {
    uint16_t code = 0;
    if (readALStatusCode(code) != SlaveError::Ok) return std::nullopt;
    return code;
}

// -- Watchdog ----------------------------------------------------------------

SlaveError EtherCATSlave::configureWatchdogs(uint16_t pdi_timeout_100us,
                                              uint16_t pdata_timeout_100us) {
    if (!master_.configureWatchdogs(index_, pdi_timeout_100us, pdata_timeout_100us)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::disableWatchdogs() {
    if (!master_.disableWatchdogs(index_)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::readWatchdogStatus(uint8_t& wd_status,
                                              uint8_t& pdi_cnt,
                                              uint8_t& pdata_cnt) {
    if (!master_.readWatchdogStatus(index_, wd_status, pdi_cnt, pdata_cnt)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

// -- SDO convenience ---------------------------------------------------------

SlaveError EtherCATSlave::sdoRead(uint16_t index, uint8_t subindex,
                                   void* data, size_t& size) {
    auto& sdo = master_.sdoManager();
    size_t actual = 0;
    if (!sdo.readSync(index_, index, subindex,
                      data, size, SDO::kDefaultSDOTimeoutMs, &actual)) {
        return SlaveError::SDOError;
    }
    size = actual;
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoWrite(uint16_t index, uint8_t subindex,
                                    const void* data, size_t size) {
    auto& sdo = master_.sdoManager();
    if (!sdo.writeSync(index_, index, subindex,
                       data, size, SDO::kDefaultSDOTimeoutMs)) {
        return SlaveError::SDOError;
    }
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoReadU8(uint16_t index, uint8_t sub, uint8_t& out) {
    auto& sdo = master_.sdoManager();
    if (!sdo.readU8(index_, index, sub, out)) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoReadU16(uint16_t index, uint8_t sub, uint16_t& out) {
    auto& sdo = master_.sdoManager();
    if (!sdo.readU16(index_, index, sub, out)) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoReadU32(uint16_t index, uint8_t sub, uint32_t& out) {
    auto& sdo = master_.sdoManager();
    if (!sdo.readU32(index_, index, sub, out)) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val) {
    auto& sdo = master_.sdoManager();
    if (!sdo.writeU8(index_, index, sub, val)) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val) {
    auto& sdo = master_.sdoManager();
    if (!sdo.writeU16(index_, index, sub, val)) return SlaveError::SDOError;
    return SlaveError::Ok;
}

SlaveError EtherCATSlave::sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val) {
    auto& sdo = master_.sdoManager();
    if (!sdo.writeU32(index_, index, sub, val)) return SlaveError::SDOError;
    return SlaveError::Ok;
}

// -- SII convenience ---------------------------------------------------------

SlaveError EtherCATSlave::readSII(SII::SIIData& data) {
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

void EtherCATSlave::logSIISummary(const char* tag) {
    SII::SIIData data;
    if (readSII(data) == SlaveError::Ok) {
        SII::logSIISummary(data, index_, tag);
    } else {
        TETHER_LOGW( tag,
            "Slave %u: Failed to read SII for summary", index_);
    }
}

// ============================================================================
// NonExistingSlave
// ============================================================================

NonExistingSlave::NonExistingSlave(EtherCATMaster& master, uint16_t index)
    : EtherCATSlave(master, index)
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
SlaveError NonExistingSlave::configureMailbox(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t) {
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
// EtherCATSlave::sm()
// ============================================================================

SyncManagerAccessor EtherCATSlave::sm(uint8_t smIndex) {
    return SyncManagerAccessor(*this, smIndex);
}

} // namespace EtherCAT
