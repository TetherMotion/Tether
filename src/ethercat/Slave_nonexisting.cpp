/**
 * @file Slave_nonexisting.cpp
 * @brief NonExistingSlave — sentinel returned for out-of-range slave indices
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
#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>
#include <bit>

namespace EtherCAT {

static const char* TAG = "Slave";

// ============================================================================
// NonExistingSlave
// ============================================================================

NonExistingSlave::NonExistingSlave(Master& master, uint16_t index)
    : Slave(master, index)
{
}

void NonExistingSlave::logCritical(const char* method) const {
    TETHER_LOGE( TAG,
        "CRITICAL: {}() called on non-existing {}. "
        "Check getDiscoveredSlaveCount() before accessing slaves. "
        "Valid range: 0 to {}.",
        method, logPrefix().c_str(), master_->getDiscoveredSlaveCount() > 0
            ? static_cast<unsigned>(master_->getDiscoveredSlaveCount() - 1) : 0u);
}

SlaveError NonExistingSlave::configureMailbox(Tether::Platform::LogLevel) {
    logCritical("configureMailbox"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configureMailbox(const MailboxSyncManagerConfig&,
                                               const MailboxSyncManagerConfig&, uint16_t) {
    logCritical("configureMailbox"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configureMailbox(const ESIFile&,
                                               Tether::Platform::LogLevel) {
    logCritical("configureMailbox"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::assumeMailboxAlreadyConfigured() {
    logCritical("assumeMailboxAlreadyConfigured");
}
void NonExistingSlave::markNoMailbox() {
    logCritical("markNoMailbox");
}

bool NonExistingSlave::drainMailbox(unsigned int) {
    logCritical("drainMailbox"); return false;
}
SlaveError NonExistingSlave::configurePDOSyncManagers() {
    logCritical("configurePDOSyncManagers"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configurePDOSyncManagers(uint16_t, uint16_t, uint8_t,
                                                       uint16_t, uint16_t, uint8_t) {
    logCritical("configurePDOSyncManagers"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::configurePDOSyncManagers(const ESIFile&) {
    logCritical("configurePDOSyncManagers"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::assumePDOAlreadyConfigured() {
    logCritical("assumePDOAlreadyConfigured");
}
SlaveError NonExistingSlave::registerPDOsFromSII(SIIPDOConfig&) {
    logCritical("registerPDOsFromSII"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::registerPDOsFromESI(const ESIFile&, SIIPDOConfig&) {
    logCritical("registerPDOsFromESI"); return SlaveError::SlaveNotFound;
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
SlaveError NonExistingSlave::configureCustomTxPDO(
    uint16_t, std::span<const CustomPDOMappingEntry>, PDO::PDODirection) {
    logCritical("configureCustomTxPDO"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::registerExistingRxPDO(uint16_t) {
    logCritical("registerExistingRxPDO"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::registerExistingTxPDO(uint16_t) {
    logCritical("registerExistingTxPDO"); return SlaveError::SlaveNotFound;
}
SlaveError NonExistingSlave::applyCustomPDOs() {
    logCritical("applyCustomPDOs"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::clearCustomPDOs() {
    logCritical("clearCustomPDOs");
}
SlaveError NonExistingSlave::configureMultiPDOs(const MultiPDOAssignment&) {
    logCritical("configureMultiPDOs"); return SlaveError::SlaveNotFound;
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
#if TETHER_ENABLE_SII
SlaveError NonExistingSlave::readSII(SII::SIIData&) {
    logCritical("readSII"); return SlaveError::SlaveNotFound;
}
void NonExistingSlave::logSIISummary(const char*) {
    logCritical("logSIISummary");
}
#endif

SyncManagerAccessor NonExistingSlave::sm(uint8_t smIndex) {
    logCritical("sm");
    // Still returns an accessor — all its read/SDO methods will fail gracefully
    return SyncManagerAccessor(*this, smIndex);
}

// ============================================================================
// Slave::sm()
} // namespace EtherCAT
