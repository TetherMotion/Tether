#pragma once

/**
 * @file EtherCatSupervisionAdapter.hpp
 * @brief IMachinePdoSource / IMachineSupervisorAccess over a running
 *        EtherCAT::Master.
 *
 * Plugging the optional machine.pdo.map and machine.supervisor.* surfaces
 * into a real master is:
 *
 * @code
 *   EtherCatPdoAccess pdo(master);
 *   EtherCatSupervisorAccess supervisorAccess(master);
 *   MachineSupervisorService supervisor(supervisorAccess, &eventJournal);
 *   machineService.setPdoSource(&pdo);
 *   machineService.setSupervisor(&supervisor);         // before install()
 * @endcode
 *
 * Recovery events are mirrored into the EventJournal as `recovery.*` records
 * so the web UI's event history shows the supervisor timeline.
 *
 * Dependency direction: tether/io depends on tether/ethercat here, never the
 * reverse. A controlled retry asks the supervisor to re-attempt recovery of
 * a failed/critical slave — it never bypasses hardware safety interlocks.
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveSupervisor.hpp"
#include "tether/io/EventJournal.hpp"
#include "tether/io/MachineService.hpp"

namespace tether::io::machine {

/// PDO map surface over LogicalAddressManager::describeEntries + the master's
/// active PDOMapping.
class EtherCatPdoAccess final : public IMachinePdoSource {
public:
    explicit EtherCatPdoAccess(EtherCAT::Master& master) : master_(master) {}

    std::vector<PdoEntryV1> entries() override {
        std::vector<PdoEntryV1> out;
        const auto slices =
            master_.logicalAddressManager().describeEntries(master_.pdo().mapping());
        for (const auto& slice : slices) {
            out.push_back({
                .slaveIndex = slice.slave_index,
                .pdoIndex = slice.pdo_index,
                .direction =
                    slice.direction == EtherCAT::PDO::PDODirection::RxPDO ? uint8_t{0}
                                                                        : uint8_t{1},
                .offset = slice.offset,
                .length = slice.length,
                .entryIndex = static_cast<uint16_t>(slice.entry_index),
            });
        }
        return out;
    }

private:
    EtherCAT::Master& master_;
};

/// Supervisor surface over Master::slaveSupervisor(). When an EventJournal is
/// supplied, an IRecoveryEventListener mirrors recovery events into the audit
/// trail as `recovery.*` records.
class EtherCatSupervisorAccess final : public IMachineSupervisorAccess {
public:
    EtherCatSupervisorAccess(EtherCAT::Master& master,
                             EventJournal* auditJournal = nullptr)
        : master_(master) {
        if (auditJournal) {
            listener_ = std::make_unique<JournalListener>(*auditJournal);
            master_.slaveSupervisor().addEventListener(listener_.get());
        }
    }

    std::vector<SupervisorEntryV1> slaves() override {
        auto& supervisor = master_.slaveSupervisor();
        std::vector<SupervisorEntryV1> out;
        for (uint16_t i = 0; i < supervisor.slaveCount(); ++i) {
            out.push_back({
                .slaveIndex = i,
                .state = static_cast<uint8_t>(supervisor.slaveState(i)),
                .suspended = supervisor.isSlaveSuspended(i),
                .recovering = supervisor.isSlaveRecovering(i),
                .attemptCount =
                    static_cast<uint16_t>(std::clamp(supervisor.slaveAttemptCount(i), 0, 0xFFFF)),
            });
        }
        return out;
    }

    bool retry(uint16_t slave, std::string& error) override {
        auto& supervisor = master_.slaveSupervisor();
        if (slave >= supervisor.slaveCount()) {
            error = "slave " + std::to_string(slave) + " is not supervised";
            return false;
        }
        if (supervisor.isSlaveRecovering(slave)) {
            error = "recovery already in progress for slave " + std::to_string(slave);
            return false;
        }
        // Clear a terminal Failed/Critical state so the supervisor will accept
        // a new recovery pass, then inject the trigger.
        const auto state = supervisor.slaveState(slave);
        if (state == EtherCAT::SlaveRecoveryState::Failed ||
            state == EtherCAT::SlaveRecoveryState::Critical) {
            supervisor.resetSlaveState(slave);
        }
        supervisor.markCritical(slave, "operator-initiated recovery retry");
        return true;
    }

private:
    /// Mirrors supervisor recovery events into the audit journal.
    class JournalListener final : public EtherCAT::IRecoveryEventListener {
    public:
        explicit JournalListener(EventJournal& journal) : journal_(journal) {}

        void onRecoveryEvent(const EtherCAT::RecoveryEvent& event) override {
            EventRecordV1 record;
            record.timestampUs = detail::nowUs();
            record.eventType =
                std::string("recovery.") + EtherCAT::recoveryEventTypeName(event.type);
            record.sourceId = std::to_string(event.slave_index);
            record.severity = severityFor(event.type);
            record.description.assign(event.detail);
            journal_.append(std::move(record));
        }

    private:
        static EventSeverity severityFor(EtherCAT::RecoveryEventType type) {
            switch (type) {
                case EtherCAT::RecoveryEventType::RecoverySucceeded:
                case EtherCAT::RecoveryEventType::SlaveResumed:
                    return EventSeverity::Info;
                case EtherCAT::RecoveryEventType::RecoveryFailed:
                case EtherCAT::RecoveryEventType::RecoveryGaveUp:
                    return EventSeverity::Fault;
                default:
                    return EventSeverity::Warning;
            }
        }

        EventJournal& journal_;
    };

    EtherCAT::Master& master_;
    std::unique_ptr<JournalListener> listener_;
};

} // namespace tether::io::machine
