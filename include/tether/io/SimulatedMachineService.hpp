#pragma once

/**
 * @file SimulatedMachineService.hpp
 * @brief Simulated machine.cia402.v1 service stack over SimulatedCiA402Fleet.
 *
 * Wires the generic MachineService surface (authority, commands, operations,
 * alarms) to the deterministic fleet: state validation mirrors realistic
 * CiA 402 transitions and the dispatcher applies them to simulated drive
 * snapshots. It is a demo/test fixture — a real machine substitutes
 * IMachineStateSource and IMachineDispatcher backed by EtherCAT/DS402.
 */

#include "tether/io/ApplicationProfile.hpp"
#include "tether/io/MachineService.hpp"
#include "tether/io/SdoService.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"

#include <algorithm>
#include <cstring>
#include <optional>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

namespace tether::io::machine {

/// DS402 state values used by the simulated fleet (subset of CiA 402).
namespace sim_states {
inline constexpr uint8_t kSwitchOnDisabled = 1;
inline constexpr uint8_t kReadyToSwitchOn = 2;
inline constexpr uint8_t kSwitchedOn = 3;
inline constexpr uint8_t kOperationEnabled = 4;
inline constexpr uint8_t kQuickStop = 5;
inline constexpr uint8_t kFault = 7;
}

/// Deterministic action-parameter encoding for the simulated dispatcher:
/// little-endian scalars packed into CommandRequest::parameters.
struct SimulatedCommandParameters {
    std::optional<int8_t> mode;        ///< SetMode
    std::optional<int32_t> position;   ///< MoveToPosition
    std::optional<int32_t> delta;      ///< StepMove
    std::optional<int32_t> velocity;   ///< JogStart (drive units/s)
    std::optional<uint32_t> durationMs;///< JogStart / JogRenew
    std::optional<uint64_t> alarmId;   ///< AcknowledgeAlarm
};

inline std::optional<SimulatedCommandParameters> decodeSimulatedParameters(
    const std::vector<uint8_t>& bytes) {
    SimulatedCommandParameters out;
    size_t position = 0;
    const auto take = [&bytes, &position](size_t count, uint64_t& value) {
        if (position + count > bytes.size()) return false;
        value = 0;
        for (size_t index = 0; index < count; ++index)
            value |= static_cast<uint64_t>(bytes[position + index]) << (index * 8U);
        position += count;
        return true;
    };
    while (position < bytes.size()) {
        const uint8_t tag = bytes[position++];
        uint64_t value = 0;
        switch (tag) {
            case 1: if (!take(1, value)) return std::nullopt;
                    out.mode = static_cast<int8_t>(value); break;
            case 2: if (!take(4, value)) return std::nullopt;
                    out.position = static_cast<int32_t>(value); break;
            case 3: if (!take(4, value)) return std::nullopt;
                    out.delta = static_cast<int32_t>(value); break;
            case 4: if (!take(4, value)) return std::nullopt;
                    out.velocity = static_cast<int32_t>(value); break;
            case 5: if (!take(4, value)) return std::nullopt;
                    out.durationMs = static_cast<uint32_t>(value); break;
            case 6: if (!take(8, value)) return std::nullopt;
                    out.alarmId = value; break;
            default: return std::nullopt;
        }
    }
    return out;
}

/// TLV helper matching decodeSimulatedParameters.
inline void putParameter(std::vector<uint8_t>& bytes, uint8_t tag, uint64_t value,
                         size_t width) {
    bytes.push_back(tag);
    for (size_t index = 0; index < width; ++index)
        bytes.push_back(static_cast<uint8_t>(value >> (index * 8U)));
}

/// Simulated state source: the fleet is the source of truth.
class SimulatedStateSource final : public IMachineStateSource {
public:
    explicit SimulatedStateSource(cia402::SimulatedCiA402Fleet& fleet)
        : fleet_(fleet) {}

    const cia402::MachineDescriptorV1& descriptor() const {
        return fleet_.descriptor();
    }

    std::vector<std::string> axisIds() const override {
        std::vector<std::string> ids;
        ids.reserve(fleet_.axisCount());
        for (const auto& axis : fleet_.descriptor().axes) ids.push_back(axis.stableId);
        return ids;
    }

    cia402::MachineSnapshotV1 machineSnapshot() const {
        // Re-aggregate from the same logic the signal encoder uses.
        cia402::MachineSnapshotV1 snapshot;
        snapshot.simulated = true;
        snapshot.alState = 8;
        snapshot.linkUp = true;
        snapshot.dcLocked = true;
        snapshot.axisCount = static_cast<uint16_t>(fleet_.axisCount());
        for (const auto& axis : fleet_.descriptor().axes) {
            const auto drive = fleet_.driveSnapshot(axis.stableId);
            if (!drive) continue;
            snapshot.timestampUs = std::max(snapshot.timestampUs, drive->timestampUs);
            snapshot.stateGeneration = std::max(snapshot.stateGeneration, drive->stateGeneration);
            if (drive->ds402State == sim_states::kOperationEnabled) ++snapshot.enabledCount;
            if (drive->faultCode != 0 || drive->ds402State == sim_states::kFault)
                ++snapshot.faultCount;
            if ((drive->statusWord & (1U << 7)) != 0) ++snapshot.warningCount;
            if ((drive->qualityFlags &
                 static_cast<uint32_t>(cia402::DriveQuality::Stale)) != 0) ++snapshot.staleCount;
            snapshot.alState = std::min(snapshot.alState, drive->alState);
            snapshot.linkUp &= drive->alState != 0;
        }
        snapshot.expectedWkc = snapshot.actualWkc =
            static_cast<uint32_t>(fleet_.axisCount() * 2);
        return snapshot;
    }

    std::optional<cia402::DriveSnapshotV1> driveSnapshot(
        std::string_view stableId) const override {
        return fleet_.driveSnapshot(stableId);
    }

    std::optional<CommandEnvironment> commandEnvironment(
        const CommandRequest&) override {
        CommandEnvironment environment;
        environment.configurationRevision = configurationRevision();
        const auto machine = machineSnapshot();
        if (!machine.linkUp)
            environment.machineBlockers.push_back("EtherCAT link is down");
        if (!machine.dcLocked)
            environment.machineBlockers.push_back("Distributed clocks are not locked");
        for (const auto& axis : fleet_.descriptor().axes) {
            const auto drive = fleet_.driveSnapshot(axis.stableId);
            if (!drive) continue;
            ResourceCommandState state;
            state.generation = drive->stateGeneration;
            const bool stale = (drive->qualityFlags &
                static_cast<uint32_t>(cia402::DriveQuality::Stale)) != 0;
            if (stale) state.blockers.push_back("Drive snapshot is stale");
            if (drive->alState != 8) state.blockers.push_back("Slave is not in AL OP");
            supportedFor(*drive, state.supportedActions);
            if (stale || drive->alState != 8)
                state.supportedActions.clear();
            environment.resources[axis.stableId] = std::move(state);
        }
        return environment;
    }

    uint64_t configurationRevision() const override { return configurationRevision_; }
    void bumpConfigurationRevision() { ++configurationRevision_; }

    static void supportedFor(const cia402::DriveSnapshotV1& drive,
                             std::unordered_set<Action>& actions) {
        using namespace sim_states;
        switch (drive.ds402State) {
            case kFault:
                actions.insert({Action::FaultReset, Action::Recover});
                break;
            case kSwitchOnDisabled:
            case kReadyToSwitchOn:
            case kSwitchedOn:
                actions.insert({Action::Enable, Action::Disable, Action::QuickStop,
                                Action::SetMode, Action::HomePrepare,
                                Action::ConfigurationStage, Action::ConfigurationValidate,
                                Action::ConfigurationCommit, Action::ConfigurationRollback});
                if (drive.ds402State == kSwitchedOn) actions.insert(Action::HomeStart);
                break;
            case kOperationEnabled:
                actions.insert({Action::Disable, Action::QuickStop, Action::SetMode,
                                Action::JogStart, Action::JogRenew, Action::JogStop,
                                Action::StepMove, Action::MoveToPosition,
                                Action::HomePrepare, Action::HomeStart,
                                Action::HomeCancel, Action::CancelOperation});
                break;
            case kQuickStop:
                actions.insert({Action::Disable, Action::FaultReset});
                break;
            default:
                break;
        }
        actions.insert(Action::AcknowledgeAlarm);
    }

private:
    cia402::SimulatedCiA402Fleet& fleet_;
    uint64_t configurationRevision_ = 1;
};

/// Simulated dispatcher: applies command effects to the fleet and drives the
/// operation record to a terminal state. Continuous actions (jog, homing)
/// stay Running until a stop/cancel command or session end.
class SimulatedDispatcher final : public IMachineDispatcher {
public:
    SimulatedDispatcher(cia402::SimulatedCiA402Fleet& fleet,
                        SimulatedStateSource& stateSource,
                        AlarmService& alarms)
        : fleet_(fleet), stateSource_(stateSource), alarms_(alarms) {}

    DispatchResult dispatch(const CommandRequest& request,
                            const SessionIdentity& identity,
                            OperationTracker& operations) override {
        DispatchResult result;
        const auto params = decodeSimulatedParameters(request.parameters);
        if (!request.parameters.empty() && !params) {
            result.message = "Command parameters are malformed";
            return result;
        }
        const std::string operationId = operations.begin(
            request.requestUuid, identity.sessionId, request.action,
            request.targets.empty() ? request.scope : request.targets.front());
        result.operationId = operationId;

        switch (request.action) {
            case Action::Enable: return apply(request, operations, operationId, 4, 0x1637,
                0x000F, "Drive enabled");
            case Action::Disable: return apply(request, operations, operationId, 2, 0x1621,
                0x0006, "Drive disabled");
            case Action::QuickStop: return apply(request, operations, operationId, 5, 0x1607,
                0x0002, "Quick stop active");
            case Action::FaultReset: return clearFault(request, operations, operationId);
            case Action::Recover:
                return apply(request, operations, operationId, sim_states::kReadyToSwitchOn,
                    0x1621, 0x0006, "Drive recovered to ready");
            case Action::SetMode:
                if (!params || !params->mode) {
                    operations.finish(operationId, OperationState::Failed,
                                      "SetMode requires parameter tag 1 (i8 mode)");
                    result.accepted = true;
                    result.message = "Missing mode parameter";
                    return result;
                }
                return setMode(request, operations, operationId, *params->mode);
            case Action::JogStart:
                if (!params || !params->velocity) {
                    operations.finish(operationId, OperationState::Failed,
                                      "JogStart requires parameter tag 4 (i32 velocity)");
                    result.accepted = true;
                    result.message = "Missing jog velocity";
                    return result;
                }
                return jogStart(request, operations, operationId, *params->velocity);
            case Action::JogStop: return jogStop(request, operations, operationId);
            case Action::JogRenew:
                operations.update(operationId, 50, "Jog lease renewed");
                result.accepted = true;
                result.message = "Jog lease renewed";
                return result;
            case Action::StepMove:
                if (!params || !params->delta) {
                    operations.finish(operationId, OperationState::Failed,
                                      "StepMove requires parameter tag 3 (i32 delta)");
                    result.accepted = true;
                    return result;
                }
                return stepMove(request, operations, operationId, *params->delta);
            case Action::MoveToPosition:
                if (!params || !params->position) {
                    operations.finish(operationId, OperationState::Failed,
                                      "MoveToPosition requires parameter tag 2 (i32 position)");
                    result.accepted = true;
                    return result;
                }
                return moveTo(request, operations, operationId, *params->position);
            case Action::HomePrepare:
                operations.finish(operationId, OperationState::Completed,
                                  "Homing prepared");
                result.accepted = true;
                result.message = "Homing prepared";
                return result;
            case Action::HomeStart:
                return homeStart(request, operations, operationId);
            case Action::HomeCancel:
                operations.finish(operationId, OperationState::Cancelled,
                                  "Homing cancelled by operator");
                forEachTarget(request, [](cia402::DriveSnapshotV1& drive) {
                    drive.homingState = 0;
                });
                result.accepted = true;
                result.message = "Homing cancelled";
                return result;
            case Action::AcknowledgeAlarm:
                if (!params || !params->alarmId) {
                    operations.finish(operationId, OperationState::Failed,
                                      "AcknowledgeAlarm requires parameter tag 6 (u64 alarm id)");
                    result.accepted = true;
                    return result;
                }
                operations.finish(operationId,
                    alarms_.acknowledge(*params->alarmId, identity) == 0
                        ? OperationState::Completed : OperationState::Failed,
                    "Alarm acknowledgement");
                result.accepted = true;
                result.message = "Alarm acknowledgement processed";
                return result;
            case Action::ConfigurationStage:
            case Action::ConfigurationValidate:
                operations.finish(operationId, OperationState::Completed,
                                  "Configuration change staged/validated (simulated)");
                result.accepted = true;
                result.message = "Configuration staged";
                return result;
            case Action::ConfigurationCommit:
                stateSource_.bumpConfigurationRevision();
                operations.finish(operationId, OperationState::Completed,
                                  "Configuration committed (simulated)");
                result.accepted = true;
                result.message = "Configuration committed";
                return result;
            case Action::ConfigurationRollback:
                operations.finish(operationId, OperationState::Completed,
                                  "Configuration rolled back (simulated)");
                result.accepted = true;
                result.message = "Configuration rolled back";
                return result;
            case Action::GroupMove:
            case Action::CancelOperation:
                operations.finish(operationId, OperationState::Failed,
                                  "Action is not implemented by the simulated dispatcher");
                result.accepted = true;
                result.message = "Unsupported simulated action";
                return result;
        }
        operations.finish(operationId, OperationState::Failed, "Unknown action");
        result.message = "Unknown action";
        return result;
    }

    bool cancelOperation(std::string_view) override { return true; }

private:
    template <typename Mutator>
    void forEachTarget(const CommandRequest& request, Mutator mutator) {
        for (const auto& target : request.targets) {
            auto drive = fleet_.driveSnapshot(target);
            if (!drive) continue;
            mutator(*drive);
            fleet_.updateDriveSnapshot(target, *drive);
            if (drive->faultCode != 0 || drive->ds402State == sim_states::kFault)
                alarms_.raise(EventSeverity::Fault, drive->faultCode, target,
                              "Drive fault " + std::to_string(drive->faultCode));
        }
    }

    DispatchResult apply(const CommandRequest& request, OperationTracker& operations,
                         const std::string& operationId, uint8_t ds402State,
                         uint16_t statusWord, uint16_t controlWord,
                         const std::string& message) {
        forEachTarget(request, [&](cia402::DriveSnapshotV1& drive) {
            drive.ds402State = ds402State;
            drive.statusWord = statusWord;
            drive.controlWord = controlWord;
        });
        operations.finish(operationId, OperationState::Completed, message);
        return {true, operationId, message};
    }

    DispatchResult clearFault(const CommandRequest& request, OperationTracker& operations,
                              const std::string& operationId) {
        forEachTarget(request, [](cia402::DriveSnapshotV1& drive) {
            drive.faultCode = 0;
            drive.ds402State = sim_states::kSwitchOnDisabled;
            drive.statusWord = 0x1650;
            drive.controlWord = 0x0080;
        });
        operations.finish(operationId, OperationState::Completed, "Fault reset");
        return {true, operationId, "Fault reset"};
    }

    DispatchResult setMode(const CommandRequest& request, OperationTracker& operations,
                           const std::string& operationId, int8_t mode) {
        forEachTarget(request, [mode](cia402::DriveSnapshotV1& drive) {
            drive.targetMode = mode;
            drive.displayMode = mode;
        });
        operations.finish(operationId, OperationState::Completed,
                          "Mode set to " + std::to_string(mode));
        return {true, operationId, "Mode updated"};
    }

    DispatchResult jogStart(const CommandRequest& request, OperationTracker& operations,
                            const std::string& operationId, int32_t velocity) {
        forEachTarget(request, [velocity](cia402::DriveSnapshotV1& drive) {
            drive.targetVelocity = velocity;
            drive.actualVelocity = velocity;
        });
        operations.update(operationId, 0, "Continuous jog active");
        return {true, operationId, "Continuous jog active"};
    }

    DispatchResult jogStop(const CommandRequest& request, OperationTracker& operations,
                           const std::string& operationId) {
        forEachTarget(request, [](cia402::DriveSnapshotV1& drive) {
            drive.targetVelocity = 0;
            drive.actualVelocity = 0;
        });
        operations.finish(operationId, OperationState::Completed, "Jog stopped");
        return {true, operationId, "Jog stopped"};
    }

    DispatchResult stepMove(const CommandRequest& request, OperationTracker& operations,
                            const std::string& operationId, int32_t delta) {
        forEachTarget(request, [delta](cia402::DriveSnapshotV1& drive) {
            drive.targetPosition += delta;
            drive.demandPosition = drive.targetPosition;
            drive.actualPosition = drive.targetPosition;
            drive.followingError = 0;
        });
        operations.finish(operationId, OperationState::Completed, "Step move complete");
        return {true, operationId, "Step move complete"};
    }

    DispatchResult moveTo(const CommandRequest& request, OperationTracker& operations,
                          const std::string& operationId, int32_t position) {
        forEachTarget(request, [position](cia402::DriveSnapshotV1& drive) {
            drive.targetPosition = position;
            drive.demandPosition = position;
            drive.actualPosition = position;
            drive.followingError = 0;
        });
        operations.finish(operationId, OperationState::Completed, "Move complete");
        return {true, operationId, "Move complete"};
    }

    DispatchResult homeStart(const CommandRequest& request, OperationTracker& operations,
                             const std::string& operationId) {
        forEachTarget(request, [](cia402::DriveSnapshotV1& drive) {
            drive.homingState = 3;  // simulated: attained
        });
        operations.finish(operationId, OperationState::Completed,
                          "Homing attained (simulated)");
        return {true, operationId, "Homing attained"};
    }

    cia402::SimulatedCiA402Fleet& fleet_;
    SimulatedStateSource& stateSource_;
    AlarmService& alarms_;
};

/// IMachineDiagnosticsSource fed from the simulated fleet's own snapshot —
/// honest counters (no fabricated jitter), matching what a real adapter
/// reads from the master's loop stats.
class SimulatedDiagnosticsSource final : public IMachineDiagnosticsSource {
public:
    explicit SimulatedDiagnosticsSource(cia402::SimulatedCiA402Fleet& fleet)
        : fleet_(fleet) {}

    MachineDiagnosticsV1 read() override {
        MachineDiagnosticsV1 d;
        d.timestampUs = detail::nowUs();
        const auto axes = static_cast<uint16_t>(fleet_.axisCount());
        d.slavesDetected = axes;
        d.lastWkc = static_cast<uint32_t>(axes * 2);
        uint16_t operational = 0;
        for (const auto& axis : fleet_.descriptor().axes) {
            const auto drive = fleet_.driveSnapshot(axis.stableId);
            if (drive) {
                d.cycleCount = std::max(d.cycleCount, drive->stateGeneration);
                if (drive->alState == 8) ++operational;
            }
        }
        d.slavesOperational = operational;
        return d;
    }

private:
    cia402::SimulatedCiA402Fleet& fleet_;
};

/// IMachinePdoSource fabricated from the fleet's axis layout — each axis maps
/// a canonical CiA 402 RxPDO (controlword/target position/velocity) and TxPDO
/// (statusword/actual position/velocity), laid out consecutively per slave.
class SimulatedPdoSource final : public IMachinePdoSource {
public:
    explicit SimulatedPdoSource(cia402::SimulatedCiA402Fleet& fleet) : fleet_(fleet) {}

    std::vector<PdoEntryV1> entries() override {
        std::vector<PdoEntryV1> out;
        // Per-axis: RxPDO 2+4+4=10 B, TxPDO 2+4+4=10 B.
        uint32_t offset = 0;
        const auto axes = fleet_.descriptor().axes;
        for (size_t i = 0; i < axes.size(); ++i) {
            const uint16_t slave = static_cast<uint16_t>(i);
            for (uint16_t pdo = 0; pdo < 3; ++pdo) {
                out.push_back({slave, pdo, 0, offset, uint16_t(pdo == 0 ? 2 : 4),
                               uint16_t(i * 6 + pdo)});
                offset += pdo == 0 ? 2 : 4;
            }
            for (uint16_t pdo = 0; pdo < 3; ++pdo) {
                out.push_back({slave, pdo, 1, offset, uint16_t(pdo == 0 ? 2 : 4),
                               uint16_t(i * 6 + 3 + pdo)});
                offset += pdo == 0 ? 2 : 4;
            }
        }
        return out;
    }

private:
    cia402::SimulatedCiA402Fleet& fleet_;
};

/// IMachineSupervisorAccess mirroring the fleet's simulated recovery states —
/// recovering drives map to Recovering, faulted drives to Critical; retry()
/// records the request and flips a faulted drive back to normal.
class SimulatedSupervisorAccess final : public IMachineSupervisorAccess {
public:
    explicit SimulatedSupervisorAccess(cia402::SimulatedCiA402Fleet& fleet)
        : fleet_(fleet) {}

    std::vector<SupervisorEntryV1> slaves() override {
        std::vector<SupervisorEntryV1> out;
        const auto axes = fleet_.descriptor().axes;
        for (size_t i = 0; i < axes.size(); ++i) {
            const auto drive = fleet_.driveSnapshot(axes[i].stableId);
            SupervisorEntryV1 entry;
            entry.slaveIndex = static_cast<uint16_t>(i);
            if (drive) {
                const bool recovering =
                    (drive->qualityFlags &
                     static_cast<uint32_t>(cia402::DriveQuality::EthercatDegraded)) != 0;
                const bool faulted = drive->faultCode != 0 || drive->ds402State == 7;
                entry.state = recovering ? 2 : faulted ? 1 : retries_.count(entry.slaveIndex) ? 3 : 0;
                entry.suspended = recovering;
                entry.recovering = recovering;
                entry.attemptCount = retries_[entry.slaveIndex];
            }
            out.push_back(entry);
        }
        return out;
    }

    bool retry(uint16_t slave, std::string& error) override {
        if (slave >= fleet_.axisCount()) {
            error = "slave " + std::to_string(slave) + " is not supervised";
            return false;
        }
        ++retries_[slave];
        const auto axes = fleet_.descriptor().axes;
        if (slave < axes.size()) {
            auto drive = fleet_.driveSnapshot(axes[slave].stableId);
            if (drive && (drive->faultCode != 0 || drive->ds402State == 7)) {
                drive->faultCode = 0;
                if (drive->ds402State == 7) drive->ds402State = 1;
                fleet_.updateDriveSnapshot(axes[slave].stableId, *drive);
            }
        }
        return true;
    }

private:
    cia402::SimulatedCiA402Fleet& fleet_;
    std::map<uint16_t, uint16_t> retries_;
};

/// IMachineChecklistSource evaluating commissioning requirements against the
/// live simulated fleet: all slaves OP, no faults, authority established,
/// staged config committed (nonzero revision), capture configured.
class SimulatedChecklistSource final : public IMachineChecklistSource {
public:
    SimulatedChecklistSource(cia402::SimulatedCiA402Fleet& fleet,
                             const StagedConfigService& config,
                             const MachineCaptureService& capture)
        : fleet_(fleet), config_(config), capture_(capture) {}

    std::vector<ChecklistItem> evaluate() override {
        std::vector<ChecklistItem> items;
        const auto axes = fleet_.descriptor().axes;
        uint32_t faulted = 0, notOp = 0;
        for (const auto& axis : axes) {
            const auto drive = fleet_.driveSnapshot(axis.stableId);
            if (!drive) continue;
            if (drive->alState != 8) ++notOp;
            if (drive->faultCode != 0 || drive->ds402State == 7) ++faulted;
        }
        items.push_back({1, "All slaves reached OP state", true, notOp == 0,
                         notOp == 0 ? "all drives AL state OP"
                                    : std::to_string(notOp) + " drive(s) not OP"});
        items.push_back({2, "No drive faults active", true, faulted == 0,
                         faulted == 0 ? "no fault codes"
                                      : std::to_string(faulted) + " faulted drive(s)"});
        const bool configured = config_.status().revision > 1;  // baseline is 1
        items.push_back({3, "Configuration committed at least once", true, configured,
                         configured ? "revision " +
                                 std::to_string(config_.status().revision)
                                    : "no committed transaction yet"});
        const auto cap = capture_.status();
        const bool capturing = cap.state != DatalogState::Idle;
        items.push_back({4, "Service capture template configured", false, capturing,
                         capturing ? "log '" + cap.metadata.logName + "' present"
                                   : "capture not configured"});
        return items;
    }

private:
    cia402::SimulatedCiA402Fleet& fleet_;
    const StagedConfigService& config_;
    const MachineCaptureService& capture_;
};

/// Convenience aggregate: owns every piece of the simulated machine service.
struct SimulatedMachineStack {
    cia402::SimulatedCiA402Fleet fleet;
    ControlAuthority authority;
    AlarmService alarms;
    OperationTracker operations;
    SimulatedStateSource stateSource;
    SimulatedDispatcher dispatcher;
    MachineCommandGate gate;
    DatalogRecorder captureRecorder;
    std::optional<MachineCaptureService> capture;
    std::optional<StagedConfigService> config;
    std::optional<SimulatedSdoAccess> sdoAccess;
    std::optional<MachineSdoService> sdo;
    std::optional<SimulatedDiagnosticsSource> diagnostics;
    std::optional<SimulatedPdoSource> pdo;
    std::optional<SimulatedSupervisorAccess> supervisorAccess;
    std::optional<MachineSupervisorService> supervisor;
    std::optional<SimulatedChecklistSource> checklist;
    std::optional<StaticAppProfileSource> appProfile;
    std::optional<InMemoryRecipeStore> recipes;
    MachineService service;

    SimulatedMachineStack()
        : authority(std::chrono::seconds(60),
                    [this](const AuthorityEvent& event) {
                        fleet.appendSimulationEvent(
                            "audit.authority." + event.action, event.scope,
                            EventSeverity::Info, 0,
                            event.actor + " " + event.action + " " + event.scope +
                                (event.reason.empty() ? "" : " (" + event.reason + ")"));
                    }),
          stateSource(fleet),
          dispatcher(fleet, stateSource, alarms),
          gate(authority,
               [this](const CommandRequest& request) {
                   return stateSource.commandEnvironment(request);
               },
               [this](const CommandRequest& request, const SessionIdentity& identity) {
                   return dispatcher.dispatch(request, identity, operations);
               },
               [this](const AuditRecord& record) {
                   const auto id = fleet.appendSimulationEvent(
                       "audit.command", record.scope, EventSeverity::Info,
                       static_cast<uint8_t>(record.action),
                       record.actor + " action=" + record.message +
                           " result=" + std::to_string(static_cast<uint8_t>(record.result)));
                   return "audit-" + std::to_string(id);
               }),
          service(stateSource, dispatcher, authority, gate, alarms, operations,
                  &fleet.eventJournal()) {}

    /// Create the registry-dependent services and install every machine
    /// function into `registry`. Call once, before sessions are accepted.
    /// `enableSdo` attaches the in-memory SDO inspector backend — surfaces
    /// are opt-in; pass false (or leave a real stack without setSdoService)
    /// to keep machine.sdo.* off the catalog entirely. `enableExtension`
    /// attaches the application-profile + recipe surfaces (Phase 7).
    bool install(Registry& registry, const SchemaCatalog& catalog,
                 bool enableSdo = true, bool enableDiagnostics = true,
                 bool enableExtension = true) {
        capture.emplace(registry, captureRecorder);
        config.emplace(registry);
        config->setRevisionHooks(
            [this] { return stateSource.configurationRevision(); },
            [this] { stateSource.bumpConfigurationRevision(); });
        service.setAuxiliaryServices(&*capture, &*config);
        if (enableSdo) {
            sdoAccess.emplace(simulatedObjectTable());
            sdo.emplace(*sdoAccess, &fleet.eventJournal());
            sdoAccess->seed(0, 0x6041, 0, {0x50, 0x02});  // statusword sample
            service.setSdoService(&*sdo);
        }
        if (enableDiagnostics) {
            diagnostics.emplace(fleet);
            service.setDiagnosticsSource(&*diagnostics);
            pdo.emplace(fleet);
            service.setPdoSource(&*pdo);
            supervisorAccess.emplace(fleet);
            supervisor.emplace(*supervisorAccess, &fleet.eventJournal());
            service.setSupervisor(&*supervisor);
            checklist.emplace(fleet, *config, *capture);
            service.setChecklistSource(&*checklist);
        }
        if (enableExtension) {
            // Demo profile: declarative widget panels bound to signal names.
            // The demo key is NOT a secret — production deployments sign the
            // document offline and ship only the verifying key.
            ProfileKey key{};
            for (size_t i = 0; i < key.size(); ++i) key[i] = static_cast<uint8_t>(0xA5 + i);
            const std::string doc = R"PROFILE({
                "format": "tether.app.profile.v1",
                "panels": [{
                    "id": "demo",
                    "title": "Demonstration signals",
                    "widgets": [
                        {"kind": "bar", "label": "Sine", "entry": "sine_wave", "unit": "V", "min": -2, "max": 2},
                        {"kind": "gauge", "label": "Cosine", "entry": "cosine_wave", "unit": "V", "min": -2, "max": 2},
                        {"kind": "sparkline", "label": "Sine trace", "entry": "sine_wave", "unit": "V", "min": -2, "max": 2},
                        {"kind": "value", "label": "Elapsed", "entry": "elapsed_time", "unit": "s"},
                        {"kind": "lamp", "label": "Warmup", "entry": "elapsed_time", "warnAbove": 30, "critAbove": 120}
                    ]
                }, {
                    "id": "axes",
                    "title": "Axis control (simulated fleet)",
                    "widgets": [
                        {"kind": "dro", "label": "X position", "entry": "drive.sim-axis-x.snapshot", "field": "actual_position", "unit": "cts", "decimals": 0},
                        {"kind": "dro", "label": "Y position", "entry": "drive.sim-axis-y.snapshot", "field": "actual_position", "unit": "cts", "decimals": 0},
                        {"kind": "state", "label": "X DS402 state", "entry": "drive.sim-axis-x.snapshot", "field": "ds402_state",
                         "states": {"0": "not ready", "1": "switch on disabled", "2": "ready", "3": "switched on", "4": "enabled", "5": "quick stop", "6": "fault reaction", "7": "fault"}},
                        {"kind": "jog", "label": "Jog X", "axis": "sim-axis-x", "entry": "drive.sim-axis-x.snapshot", "field": "actual_velocity", "unit": "cts/s"},
                        {"kind": "jog", "label": "Jog Y", "axis": "sim-axis-y", "entry": "drive.sim-axis-y.snapshot", "field": "actual_velocity", "unit": "cts/s"},
                        {"kind": "command", "label": "Enable X", "axis": "sim-axis-x", "action": 0},
                        {"kind": "command", "label": "Fault reset X", "axis": "sim-axis-x", "action": 3},
                        {"kind": "button", "label": "Acceptance report", "fn": "machine.checklist.report"}
                    ]
                }, {
                    "id": "gantry",
                    "title": "Machine view (read-only)",
                    "widgets": [
                        {"kind": "scene", "label": "XY position", "axes": [
                            {"name": "x", "entry": "drive.sim-axis-x.snapshot", "field": "actual_position", "targetField": "target_position", "min": -10000, "max": 10000},
                            {"name": "y", "entry": "drive.sim-axis-y.snapshot", "field": "actual_position", "targetField": "target_position", "min": -10000, "max": 10000}
                        ]}
                    ]
                }]
            })PROFILE";
            appProfile.emplace(key);
            const auto mac = signProfileDocument(doc.data(), doc.size(), key);
            if (appProfile->install("tether.demo", "1.0",
                                    {doc.begin(), doc.end()}, mac))
                service.setAppProfileSource(&*appProfile);
            recipes.emplace();
            service.setRecipeStore(&*recipes);
        }
        return service.install(registry, catalog);
    }

    /// Common CiA 402 object table shown by machine.sdo.list in simulation.
    static std::vector<SdoObjectInfoV1> simulatedObjectTable() {
        return {
            {0x1000, 0, 0x0007, 1, "Device type"},
            {0x1008, 0, 0x0009, 1, "Device name"},
            {0x6040, 0, 0x0006, 3, "Controlword"},
            {0x6041, 0, 0x0006, 1, "Statusword"},
            {0x6060, 0, 0x0002, 3, "Modes of operation"},
            {0x6061, 0, 0x0002, 1, "Modes of operation display"},
            {0x6064, 0, 0x0004, 1, "Position actual value"},
            {0x606C, 0, 0x0004, 1, "Velocity actual value"},
            {0x607A, 0, 0x0004, 3, "Target position", "Commanded profile position",
             -2147483648.0, 2147483647.0},
            {0x60FF, 0, 0x0004, 3, "Target velocity"},
            {0x6502, 0, 0x0007, 1, "Supported drive modes",
             "Bitmask of supported modes of operation"},
        };
    }
};

} // namespace tether::io::machine
