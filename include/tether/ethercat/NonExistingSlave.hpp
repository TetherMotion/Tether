/**
 * @file NonExistingSlave.hpp
 * @brief NonExistingSlave — sentinel slave for invalid indices.
 *
 * Split out of Slave.hpp.
 */

#pragma once

#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {

class NonExistingSlave final : public Slave {
public:
    /**
     * @brief Construct a NonExistingSlave.
     * @param master   Reference back to the master
     * @param index    The invalid index that was requested
     */
    NonExistingSlave(Master& master, uint16_t index);

    SlaveError configureMailbox(Tether::Platform::LogLevel) override;
    SlaveError configureMailbox(const MailboxSyncManagerConfig&,
                                 const MailboxSyncManagerConfig&, uint16_t) override;
    SlaveError configureMailbox(const ESIFile&,
                                 Tether::Platform::LogLevel) override;
    void assumeMailboxAlreadyConfigured() override;
    void markNoMailbox() override;
    bool drainMailbox(unsigned int) override;

    SlaveError configurePDOSyncManagers() override;
    SlaveError configurePDOSyncManagers(uint16_t, uint16_t, uint8_t,
                                         uint16_t, uint16_t, uint8_t) override;
    SlaveError configurePDOSyncManagers(const ESIFile&) override;
    void assumePDOAlreadyConfigured() override;

    SlaveError registerPDOsFromSII(SIIPDOConfig&) override;
    SlaveError registerPDOsFromESI(const ESIFile&, SIIPDOConfig&) override;
    SlaveError assignPDOs(const SIIPDOConfig&) override;
    SlaveError registerFixedPDOs(const SIIPDOConfig&) override;

    SlaveError configureCustomRxPDO(uint16_t, std::initializer_list<CustomPDOMappingEntry>) override;
    SlaveError configureCustomTxPDO(uint16_t, std::initializer_list<CustomPDOMappingEntry>) override;
    SlaveError configureCustomTxPDO(uint16_t, std::span<const CustomPDOMappingEntry>,
                                    PDO::PDODirection) override;
    SlaveError applyCustomPDOs() override;
    void clearCustomPDOs() override;
    SlaveError registerExistingRxPDO(uint16_t) override;
    SlaveError registerExistingTxPDO(uint16_t) override;
    SlaveError configureMultiPDOs(const MultiPDOAssignment&) override;

    SlaveError transitionTo(SlaveState) override;
    SlaveError transitionToInit() override;
    SlaveError transitionToPreOp() override;
    SlaveError transitionToSafeOp() override;
    SlaveError transitionToOp() override;
    SlaveError transitionToBoot() override;

    SlaveError readState(SlaveState&) override;
    SlaveError readALStatusCode(uint16_t&) override;

    SlaveError configureWatchdogs(uint16_t, uint16_t) override;
    SlaveError disableWatchdogs() override;
    SlaveError readWatchdogStatus(uint8_t&, uint8_t&, uint8_t&) override;

    SlaveError sdoRead(uint16_t, uint8_t, void*, size_t&) override;
    SlaveError sdoWrite(uint16_t, uint8_t, const void*, size_t) override;
    SlaveError sdoReadU8(uint16_t, uint8_t, uint8_t&) override;
    SlaveError sdoReadU16(uint16_t, uint8_t, uint16_t&) override;
    SlaveError sdoReadU32(uint16_t, uint8_t, uint32_t&) override;
    SlaveError sdoWriteU8(uint16_t, uint8_t, uint8_t) override;
    SlaveError sdoWriteU16(uint16_t, uint8_t, uint16_t) override;
    SlaveError sdoWriteU32(uint16_t, uint8_t, uint32_t) override;

#if TETHER_ENABLE_SII
    SlaveError readSII(SII::SIIData&) override;
    void logSIISummary(const char*) override;
#endif

    SyncManagerAccessor sm(uint8_t smIndex) override;

private:
    void logCritical(const char* method) const;
};

} // namespace EtherCAT
