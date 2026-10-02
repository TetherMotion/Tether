#pragma once

/**
 * @file DS402MachineAdapter.hpp
 * @brief Real-machine IMachineStateSource/IMachineDispatcher over DS402Master.
 *
 * Bridges the generic machine.cia402.v1 IO surface to a live EtherCAT master:
 * drive snapshots are assembled from AL state/status code, the CiA 402
 * statusword/state machine, and application-supplied PDO extractors (PDO
 * layouts are drive- and mapping-specific, so positions/velocities/torques
 * are filled by per-axis callbacks rather than guessed).
 *
 * Threading: snapshot reads touch TxPDO/RxPDO scratch buffers and perform
 * best-effort register reads; dispatch() performs blocking SDO/ESM work and
 * must run on the MachineService command executor — never on the cyclic
 * real-time thread. The adapter never writes the RxPDO buffer from the IO
 * path except through CiA402Drive's own helpers.
 */

#include "tether/io/MachineService.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"
#include "tether/ethercat/ALRegisters.hpp"

#include <array>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace tether::io::machine {

/// Application-owned description of one DS402-managed axis.
struct DS402AxisConfig {
    std::string stableId;      ///< Position-independent resource id (required)
    std::string name;          ///< Human-readable axis name
    std::string groupId;       ///< Motion-group resource id, may be empty
    uint16_t slaveIndex = 0;   ///< EtherCAT slave position
    uint16_t displayOrder = 0; ///< Preferred fleet display order
    std::string positionUnit;
    std::string velocityUnit;
    double positionScale = 1.0; ///< Application units per drive-native count
    double velocityScale = 1.0; ///< Application units per drive-native count/s
    bool supportsHoming = false;

    /// Optional per-axis extractor for cyclic PDO data (positions,
    /// velocities, torques, following error, modes). Called while holding
    /// the adapter's snapshot mutex; must be non-blocking and only read the
    /// drive's TxPDO buffer / cached values.
    std::function<void(const EtherCAT::CiA402Drive&, cia402::DriveSnapshotV1&)>
        fillProcessData;
};

struct DS402MachineConfig {
    std::string machineId = "machine";
    std::string displayName = "CiA 402 machine";
    std::string timezone = "UTC";
    std::string unitSystem = "si";
    std::vector<DS402AxisConfig> axes;
};

class DS402MachineAdapter final : public IMachineStateSource,
                                  public IMachineDispatcher {
public:
    DS402MachineAdapter(EtherCAT::DS402Master& master, DS402MachineConfig config)
        : master_(master), axes_(std::move(config.axes)) {
        descriptor_.machineId = std::move(config.machineId);
        descriptor_.displayName = std::move(config.displayName);
        descriptor_.timezone = std::move(config.timezone);
        descriptor_.unitSystem = std::move(config.unitSystem);
        descriptor_.axes.reserve(axes_.size());
        for (const auto& axis : axes_) {
            cia402::AxisDescriptorV1 entry;
            entry.stableId = axis.stableId;
            entry.name = axis.name;
            entry.slaveIndex = axis.slaveIndex;
            entry.displayOrder = axis.displayOrder;
            entry.groupId = axis.groupId;
            entry.positionUnit = axis.positionUnit;
            entry.positionScale = axis.positionScale;
            entry.velocityUnit = axis.velocityUnit;
            entry.velocityScale = axis.velocityScale;
            entry.supportsHoming = axis.supportsHoming;
            descriptor_.axes.push_back(std::move(entry));
        }
    }

    const cia402::MachineDescriptorV1& descriptor() const { return descriptor_; }

    // ---- IMachineStateSource ------------------------------------------------

    std::vector<std::string> axisIds() const override {
        std::vector<std::string> ids;
        ids.reserve(axes_.size());
        for (const auto& axis : axes_) ids.push_back(axis.stableId);
        return ids;
    }

    std::optional<cia402::DriveSnapshotV1> driveSnapshot(
        std::string_view stableId) const override {
        const auto* axis = findAxis(stableId);
        if (!axis) return std::nullopt;
        std::lock_guard lock(mutex_);
        // Register reads go over the wire — inherently non-const.
        return snapshotFor(*axis, const_cast<EtherCAT::DS402Master&>(master_));
    }

    /// Coherent aggregate; EtherCAT-wide health fields are best-effort.
    cia402::MachineSnapshotV1 machineSnapshot() {
        std::lock_guard lock(mutex_);
        cia402::MachineSnapshotV1 snapshot;
        snapshot.simulated = false;
        snapshot.axisCount = static_cast<uint16_t>(axes_.size());
        uint8_t aggregateAl = 8;
        for (const auto& axis : axes_) {
            const auto drive = snapshotFor(axis, master_);
            snapshot.timestampUs = std::max(snapshot.timestampUs, drive.timestampUs);
            snapshot.stateGeneration = std::max(snapshot.stateGeneration,
                                                drive.stateGeneration);
            if (drive.ds402State == static_cast<uint8_t>(EtherCAT::DriveState::OperationEnabled))
                ++snapshot.enabledCount;
            if (drive.faultCode != 0 ||
                drive.ds402State == static_cast<uint8_t>(EtherCAT::DriveState::Fault))
                ++snapshot.faultCount;
            if ((drive.statusWord & (1U << 7)) != 0) ++snapshot.warningCount;
            if ((drive.qualityFlags &
                 static_cast<uint32_t>(cia402::DriveQuality::Stale)) != 0)
                ++snapshot.staleCount;
            aggregateAl = std::min(aggregateAl, drive.alState);
        }
        snapshot.alState = axes_.empty() ? 0 : aggregateAl;
        snapshot.linkUp = master_.ethercatMaster().hasAnySlaves();
        snapshot.actualWkc = master_.ethercatMaster().lastWkc();
        // Expected WKC is application/mapping-specific; the aggregate only
        // reports the observed counter.
        snapshot.expectedWkc = snapshot.actualWkc;
        snapshot.dcLocked = master_.ethercatMaster().dc().isInitialized();
        return snapshot;
    }

    std::optional<CommandEnvironment> commandEnvironment(
        const CommandRequest&) override {
        CommandEnvironment environment;
        environment.configurationRevision = configurationRevision();
        const auto machine = machineSnapshot();
        if (!machine.linkUp)
            environment.machineBlockers.push_back("EtherCAT link reports no slaves");
        if (!machine.dcLocked)
            environment.machineBlockers.push_back("Distributed clocks are not initialized");
        for (const auto& axis : axes_) {
            const auto drive = snapshotFor(axis, master_);
            ResourceCommandState state;
            state.generation = drive.stateGeneration;
            const bool stale = (drive.qualityFlags &
                static_cast<uint32_t>(cia402::DriveQuality::Stale)) != 0;
            const bool degraded = (drive.qualityFlags &
                static_cast<uint32_t>(cia402::DriveQuality::EthercatDegraded)) != 0;
            if (stale) state.blockers.push_back("Drive snapshot is stale");
            if (degraded) state.blockers.push_back("EtherCAT read failed for this slave");
            if (drive.alState != static_cast<uint8_t>(EtherCAT::AL::SlaveState::OP))
                state.blockers.push_back("Slave is not in AL OP");
            supportedFor(drive, axis.supportsHoming, state.supportedActions);
            if (stale || degraded ||
                drive.alState != static_cast<uint8_t>(EtherCAT::AL::SlaveState::OP))
                state.supportedActions.clear();
            environment.resources[axis.stableId] = std::move(state);
        }
        return environment;
    }

    uint64_t configurationRevision() const override {
        return configurationRevision_.load(std::memory_order_acquire);
    }
    void bumpConfigurationRevision() {
        configurationRevision_.fetch_add(1, std::memory_order_acq_rel);
    }

    // ---- IMachineDispatcher -------------------------------------------------
    //
    // All operations are synchronous SDO/PDO writes; they run on the command
    // executor thread, never the cyclic loop. Continuous motion (jog/CSP/CSV)
    // is deliberately not dispatched here — cyclic setpoint streaming must go
    // through the native planner, not a browser command.

    DispatchResult dispatch(const CommandRequest& request,
                            const SessionIdentity& identity,
                            OperationTracker& operations) override {
        DispatchResult result;
        const std::string operationId = operations.begin(
            request.requestUuid, identity.sessionId, request.action,
            request.targets.empty() ? request.scope : request.targets.front());
        result.operationId = operationId;

        const auto finish = [&](OperationState state, std::string message,
                                bool accepted) {
            operations.finish(operationId, state, message);
            result.accepted = accepted;
            result.message = std::move(message);
            return result;
        };

        switch (request.action) {
            case Action::Enable:
                return eachTarget(request, operationId, operations, "Drive enabled",
                    [](EtherCAT::CiA402Drive& drive) { return drive.enable(); });
            case Action::Disable:
                return eachTarget(request, operationId, operations, "Drive disabled",
                    [](EtherCAT::CiA402Drive& drive) { return drive.disable(); });
            case Action::QuickStop:
                return eachTarget(request, operationId, operations, "Quick stop active",
                    [](EtherCAT::CiA402Drive& drive) { return drive.quickStop(); });
            case Action::FaultReset:
                return eachTarget(request, operationId, operations, "Fault reset",
                    [](EtherCAT::CiA402Drive& drive) { return drive.resetFault(); });
            case Action::Recover:
                // Guarded recovery: reset the fault, then run the standard
                // enable sequence so the drive returns to Operation Enabled.
                return eachTarget(request, operationId, operations, "Drive recovered",
                    [](EtherCAT::CiA402Drive& drive) {
                        return drive.resetFault() && drive.enable();
                    });
            case Action::SetMode: {
                const auto mode = parameterI8(request.parameters, 1);
                if (!mode)
                    return finish(OperationState::Failed,
                                  "SetMode requires parameter tag 1 (i8 mode)", false);
                return eachTarget(request, operationId, operations, "Mode updated",
                    [&](EtherCAT::CiA402Drive& drive) {
                        return drive.setOperatingMode(*mode);
                    });
            }
            case Action::HomeStart:
                return eachTarget(request, operationId, operations, "Homing complete",
                    [](EtherCAT::CiA402Drive& drive) {
                        return drive.executeHoming();
                    });
            case Action::HomeCancel:
            case Action::CancelOperation:
                // CiA 402 halt: quick stop is the safe interrupt for an
                // SDO-driven homing sequence.
                return eachTarget(request, operationId, operations, "Operation halted",
                    [](EtherCAT::CiA402Drive& drive) { return drive.quickStop(); });
            case Action::ConfigurationCommit:
                bumpConfigurationRevision();
                return finish(OperationState::Completed,
                              "Configuration committed; revision incremented", true);
            case Action::JogStart:
            case Action::JogRenew:
            case Action::JogStop:
            case Action::StepMove:
            case Action::MoveToPosition:
            case Action::GroupMove:
            case Action::HomePrepare:
            case Action::AcknowledgeAlarm:
            case Action::ConfigurationStage:
            case Action::ConfigurationValidate:
            case Action::ConfigurationRollback:
                return finish(OperationState::Failed,
                              "Action requires the application's native planner "
                              "or configuration service and is not dispatched by "
                              "the generic DS402 adapter", false);
        }
        return finish(OperationState::Failed, "Unknown action", false);
    }

    bool cancelOperation(std::string_view) override {
        // The dispatcher is synchronous; by the time cancel is meaningful the
        // call has already returned. Abort semantics belong to the native
        // planner / hold-to-run path, not this adapter.
        return false;
    }

private:
    const DS402AxisConfig* findAxis(std::string_view stableId) const {
        for (const auto& axis : axes_)
            if (axis.stableId == stableId) return &axis;
        return nullptr;
    }

    /// Assemble a snapshot from AL state/status code + CiA 402 statusword.
    /// Register read failures degrade the snapshot rather than throwing.
    cia402::DriveSnapshotV1 snapshotFor(const DS402AxisConfig& axis,
                                      EtherCAT::DS402Master& master) const {
        cia402::DriveSnapshotV1 snapshot;
        snapshot.slaveIndex = axis.slaveIndex;
        snapshot.timestampUs = detail::nowUs();
        snapshot.stateGeneration = generation_.load(std::memory_order_acquire);

        auto& slave = master.ethercatMaster().slave(axis.slaveIndex);
        if (const auto state = slave.ALState()) {
            snapshot.alState = static_cast<uint8_t>(*state);
        } else {
            snapshot.qualityFlags |=
                static_cast<uint32_t>(cia402::DriveQuality::EthercatDegraded);
        }
        snapshot.alStatusCode = slave.ALCode().value_or(0);

        auto* drive = master.driveBySlaveIndex(axis.slaveIndex);
        if (!drive) {
            snapshot.qualityFlags |=
                static_cast<uint32_t>(cia402::DriveQuality::EthercatDegraded);
            return snapshot;
        }
        snapshot.statusWord = drive->getStatusword();
        snapshot.controlWord = drive->getControlword();
        snapshot.ds402State =
            static_cast<uint8_t>(EtherCAT::decodeDriveState(snapshot.statusWord));
        if (drive->isFaulted()) {
            // 0x603F error code requires a drive-specific SDO read; surfaced
            // as nonzero fault indicator for now.
            snapshot.faultCode = 1;
        }
        if (axis.fillProcessData) axis.fillProcessData(*drive, snapshot);
        return snapshot;
    }

    /// Bump the machine state generation — call after any adapter-observed
    /// state change (the cyclic loop or supervisor should invoke this).
    void bumpGeneration() {
        generation_.fetch_add(1, std::memory_order_acq_rel);
    }

    static void supportedFor(const cia402::DriveSnapshotV1& drive,
                             bool supportsHoming,
                             std::unordered_set<Action>& actions) {
        using State = EtherCAT::DriveState;
        switch (static_cast<State>(drive.ds402State)) {
            case State::Fault:
            case State::FaultReactionActive:
                actions.insert({Action::FaultReset, Action::Recover});
                break;
            case State::SwitchOnDisabled:
            case State::ReadyToSwitchOn:
            case State::SwitchedOn:
                actions.insert({Action::Enable, Action::Disable, Action::QuickStop,
                                Action::SetMode});
                if (supportsHoming) actions.insert(Action::HomeStart);
                break;
            case State::OperationEnabled:
                actions.insert({Action::Disable, Action::QuickStop, Action::SetMode,
                                Action::HomeCancel, Action::CancelOperation});
                if (supportsHoming) actions.insert(Action::HomeStart);
                break;
            case State::QuickStopActive:
                actions.insert({Action::Disable, Action::FaultReset});
                break;
            default:
                break;
        }
        actions.insert(Action::AcknowledgeAlarm);
    }

    /// Extract a tagged little-endian parameter (tag, width, value) matching
    /// SimulatedCommandParameters' wire shape.
    static std::optional<int8_t> parameterI8(const std::vector<uint8_t>& bytes,
                                           uint8_t wantedTag) {
        size_t position = 0;
        while (position < bytes.size()) {
            const uint8_t tag = bytes[position++];
            static constexpr std::array<size_t, 7> widths{0, 1, 4, 4, 4, 4, 8};
            const size_t width = tag < widths.size() ? widths[tag] : 0;
            if (width == 0 || position + width > bytes.size()) return std::nullopt;
            uint64_t value = 0;
            for (size_t index = 0; index < width; ++index)
                value |= static_cast<uint64_t>(bytes[position + index]) << (index * 8U);
            position += width;
            if (tag == wantedTag) return static_cast<int8_t>(value);
        }
        return std::nullopt;
    }

    template <typename Fn>
    DispatchResult eachTarget(const CommandRequest& request,
                              const std::string& operationId,
                              OperationTracker& operations,
                              const char* successMessage, Fn&& fn) {
        for (const auto& target : request.targets) {
            const auto* axis = findAxis(target);
            auto* drive = axis ? master_.driveBySlaveIndex(axis->slaveIndex) : nullptr;
            if (!drive) {
                operations.finish(operationId, OperationState::Failed,
                                  "Target is not a managed DS402 drive: " + target);
                return {false, operationId,
                        "Target is not a managed DS402 drive: " + target};
            }
            if (!fn(*drive)) {
                operations.finish(operationId, OperationState::Failed,
                                  std::string("Drive rejected the action: ") + target);
                return {false, operationId,
                        std::string("Drive rejected the action: ") + target};
            }
        }
        bumpGeneration();
        operations.finish(operationId, OperationState::Completed, successMessage);
        return {true, operationId, successMessage};
    }

    EtherCAT::DS402Master& master_;
    std::vector<DS402AxisConfig> axes_;
    cia402::MachineDescriptorV1 descriptor_;
    mutable std::mutex mutex_;
    std::atomic<uint64_t> generation_{1};
    std::atomic<uint64_t> configurationRevision_{1};
};

} // namespace tether::io::machine
