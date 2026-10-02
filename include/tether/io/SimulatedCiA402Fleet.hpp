#pragma once

/**
 * @file SimulatedCiA402Fleet.hpp
 * @brief Read-only schema-backed simulated fleet for machine.cia402.v1 demos/tests.
 *
 * This fixture is deliberately disconnected from EtherCAT and motion-control
 * dispatch. It publishes observational signals and a bounded read-only event
 * history function; it must not be used as a substitute for a production
 * machine service or a safety function.
 */

#include "tether/io/CiA402MachineProfile.hpp"
#include "tether/io/EventJournal.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaValueWire.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace tether::io::cia402 {

struct AxisDescriptorV1 {
    std::string stableId;
    std::string name;
    uint16_t slaveIndex = 0;
    uint16_t displayOrder = 0;
    std::string groupId;
    std::string positionUnit = "mm";
    double positionScale = 1.0;
    std::string velocityUnit = "mm/s";
    double velocityScale = 1.0;
    bool supportsHoming = false;
};

struct MachineDescriptorV1 {
    uint32_t profileVersion = kProfileVersion;
    std::string machineId = "simulated-cia402-machine";
    std::string displayName = "Simulated CiA 402 Machine";
    std::string timezone = "UTC";
    std::string unitSystem = "metric";
    std::vector<AxisDescriptorV1> axes;
};

struct MachineSnapshotV1 {
    uint64_t timestampUs = 0;
    uint64_t stateGeneration = 0;
    bool simulated = true;
    uint16_t axisCount = 0;
    uint16_t enabledCount = 0;
    uint16_t faultCount = 0;
    uint16_t warningCount = 0;
    uint16_t staleCount = 0;
    uint8_t alState = 8;
    uint32_t expectedWkc = 0;
    uint32_t actualWkc = 0;
    bool linkUp = true;
    bool dcLocked = true;
};

namespace simulated_fleet_detail {

inline void appendVarint(std::vector<uint8_t>& bytes, uint64_t value) {
    while (value >= 0x80) {
        bytes.push_back(static_cast<uint8_t>(value) | 0x80U);
        value >>= 7;
    }
    bytes.push_back(static_cast<uint8_t>(value));
}

inline void appendString(std::vector<uint8_t>& bytes, std::string_view value) {
    bytes.insert(bytes.end(), value.begin(), value.end());
    bytes.push_back(0);
}

template <typename T>
inline std::vector<uint8_t> scalar(T value) {
    using Unsigned = std::make_unsigned_t<T>;
    const Unsigned bits = static_cast<Unsigned>(value);
    std::vector<uint8_t> bytes(sizeof(T));
    for (size_t index = 0; index < sizeof(T); ++index) {
        bytes[index] = static_cast<uint8_t>(bits >> (index * 8U));
    }
    return bytes;
}

inline std::vector<uint8_t> scalar(bool value) {
    return {static_cast<uint8_t>(value ? 1 : 0)};
}

inline std::vector<uint8_t> scalar(double value) {
    const uint64_t bits = std::bit_cast<uint64_t>(value);
    return scalar(bits);
}

inline void appendTagged(std::vector<uint8_t>& bytes,
                         const std::vector<std::pair<uint32_t, std::vector<uint8_t>>>& fields) {
    appendVarint(bytes, fields.size());
    for (const auto& [key, value] : fields) {
        appendVarint(bytes, key);
        appendVarint(bytes, value.size());
        bytes.insert(bytes.end(), value.begin(), value.end());
    }
}

inline std::vector<uint8_t> encodeAxis(const AxisDescriptorV1& axis) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> fields;
    fields.reserve(10);
    fields.emplace_back(1, std::vector<uint8_t>{}); appendString(fields.back().second, axis.stableId);
    fields.emplace_back(2, std::vector<uint8_t>{}); appendString(fields.back().second, axis.name);
    fields.emplace_back(3, scalar(axis.slaveIndex));
    fields.emplace_back(4, scalar(axis.displayOrder));
    fields.emplace_back(5, std::vector<uint8_t>{}); appendString(fields.back().second, axis.groupId);
    fields.emplace_back(6, std::vector<uint8_t>{}); appendString(fields.back().second, axis.positionUnit);
    fields.emplace_back(7, scalar(axis.positionScale));
    fields.emplace_back(8, std::vector<uint8_t>{}); appendString(fields.back().second, axis.velocityUnit);
    fields.emplace_back(9, scalar(axis.velocityScale));
    fields.emplace_back(10, scalar(axis.supportsHoming));
    std::vector<uint8_t> encoded;
    appendTagged(encoded, fields);
    return encoded;
}

inline std::vector<uint8_t> encodeDescriptor(const MachineDescriptorV1& descriptor) {
    std::vector<uint8_t> axes;
    appendVarint(axes, descriptor.axes.size());
    for (const auto& axis : descriptor.axes) {
        auto encoded = encodeAxis(axis);
        appendVarint(axes, encoded.size());
        axes.insert(axes.end(), encoded.begin(), encoded.end());
    }
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> fields;
    fields.reserve(6);
    fields.emplace_back(1, scalar(descriptor.profileVersion));
    fields.emplace_back(2, std::vector<uint8_t>{}); appendString(fields.back().second, descriptor.machineId);
    fields.emplace_back(3, std::vector<uint8_t>{}); appendString(fields.back().second, descriptor.displayName);
    fields.emplace_back(4, std::vector<uint8_t>{}); appendString(fields.back().second, descriptor.timezone);
    fields.emplace_back(5, std::vector<uint8_t>{}); appendString(fields.back().second, descriptor.unitSystem);
    fields.emplace_back(6, std::move(axes));
    std::vector<uint8_t> encoded;
    appendTagged(encoded, fields);
    return encoded;
}

inline std::vector<uint8_t> encodeMachineSnapshot(const MachineSnapshotV1& snapshot) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> fields;
    fields.reserve(13);
    fields.emplace_back(1, scalar(snapshot.timestampUs));
    fields.emplace_back(2, scalar(snapshot.stateGeneration));
    fields.emplace_back(3, scalar(snapshot.simulated));
    fields.emplace_back(4, scalar(snapshot.axisCount));
    fields.emplace_back(5, scalar(snapshot.enabledCount));
    fields.emplace_back(6, scalar(snapshot.faultCount));
    fields.emplace_back(7, scalar(snapshot.warningCount));
    fields.emplace_back(8, scalar(snapshot.staleCount));
    fields.emplace_back(9, scalar(snapshot.alState));
    fields.emplace_back(10, scalar(snapshot.expectedWkc));
    fields.emplace_back(11, scalar(snapshot.actualWkc));
    fields.emplace_back(12, scalar(snapshot.linkUp));
    fields.emplace_back(13, scalar(snapshot.dcLocked));
    std::vector<uint8_t> encoded;
    appendTagged(encoded, fields);
    return encoded;
}

inline std::vector<uint8_t> encodeEvent(const EventRecordV1& event) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> fields;
    fields.reserve(8);
    fields.emplace_back(1, scalar(event.eventId));
    fields.emplace_back(2, scalar(event.timestampUs));
    fields.emplace_back(3, scalar(event.stateGeneration));
    fields.emplace_back(4, std::vector<uint8_t>{}); appendString(fields.back().second, event.eventType);
    fields.emplace_back(5, std::vector<uint8_t>{}); appendString(fields.back().second, event.sourceId);
    fields.emplace_back(6, scalar(static_cast<uint8_t>(event.severity)));
    fields.emplace_back(7, scalar(event.code));
    fields.emplace_back(8, std::vector<uint8_t>{}); appendString(fields.back().second, event.description);
    std::vector<uint8_t> encoded;
    appendTagged(encoded, fields);
    return encoded;
}

inline std::vector<uint8_t> encodeEventPage(const EventPageV1& page) {
    std::vector<uint8_t> events;
    appendVarint(events, page.events.size());
    for (const auto& event : page.events) {
        auto encoded = encodeEvent(event);
        appendVarint(events, encoded.size());
        events.insert(events.end(), encoded.begin(), encoded.end());
    }
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> fields;
    fields.reserve(5);
    fields.emplace_back(1, scalar(page.oldestCursor));
    fields.emplace_back(2, scalar(page.latestCursor));
    fields.emplace_back(3, scalar(page.nextCursor));
    fields.emplace_back(4, scalar(page.gap));
    fields.emplace_back(5, std::move(events));
    std::vector<uint8_t> encoded;
    appendTagged(encoded, fields);
    return encoded;
}

template <typename T>
inline bool decodeLittleEndian(const std::vector<uint8_t>& bytes, T& value) {
    if (bytes.size() != sizeof(T)) return false;
    using Unsigned = std::make_unsigned_t<T>;
    Unsigned bits = 0;
    for (size_t index = 0; index < sizeof(T); ++index)
        bits |= static_cast<Unsigned>(bytes[index]) << (index * 8U);
    value = static_cast<T>(bits);
    return true;
}

inline uint64_t monotonicTimestampUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

} // namespace simulated_fleet_detail

/**
 * A deterministic read-only fleet publisher. The owner must outlive the
 * Registry callbacks installed by registerSignals(). Mutation is intended for
 * simulation/test setup, never for browser-driven motion commands.
 */
class SimulatedCiA402Fleet {
public:
    static constexpr uint64_t kMachineDescriptorSignalId = 0x4349410000000001ULL;
    static constexpr uint64_t kMachineSnapshotSignalId = 0x4349410000000002ULL;
    static constexpr uint64_t kDriveSignalIdBase = 0x4349410100000000ULL;
    static constexpr uint64_t kEventCursorSignalId = 0x4349410200000001ULL;
    static constexpr uint64_t kEventReadFunctionId = 0x4349410200000002ULL;
    static constexpr uint16_t kMaximumEncodedValue = 8192;

    explicit SimulatedCiA402Fleet(MachineDescriptorV1 descriptor = fourAxisDemoDescriptor())
        : descriptor_(std::move(descriptor)), snapshots_(descriptor_.axes.size()) {
        validateDescriptor();
        initializeSnapshots();
        journal_.append({0, simulated_fleet_detail::monotonicTimestampUs(), stateGeneration_,
                 "SimulatorStarted", descriptor_.machineId, EventSeverity::Info, 0,
                 "Read-only simulated CiA 402 fleet initialized"});
    }

    static MachineDescriptorV1 fourAxisDemoDescriptor() {
        MachineDescriptorV1 descriptor;
        descriptor.axes = {
            {"sim-axis-x", "X", 0, 0, "gantry", "mm", 0.001, "mm/s", 0.001, true},
            {"sim-axis-y", "Y", 1, 1, "gantry", "mm", 0.001, "mm/s", 0.001, true},
            {"sim-axis-z", "Z", 2, 2, "gantry", "mm", 0.001, "mm/s", 0.001, true},
            {"sim-axis-a", "A", 3, 3, "extruder", "mm", 0.001, "mm/s", 0.001, false},
        };
        return descriptor;
    }

    static bool installSchemas(const SchemaGraph& graph, SchemaCatalog& catalog) {
        const auto manifest = machineProfileManifest(graph);
        return manifest.size() == 31 && catalog.install(graph, manifest);
    }

    bool registerSignals(Registry& registry, const SchemaCatalog& catalog) {
        const auto* graph = catalog.graph();
        if (!graph) return false;
        const auto makeRef = [graph](const SchemaKey& key) -> SchemaRef {
            const auto* node = graph->find(key);
            return node ? SchemaRef{node->key, computeSchemaDigest(*node)} : SchemaRef{};
        };
        const auto descriptorRef = makeRef(schemaKey(MachineSchemaId::MachineDescriptor));
        const auto snapshotRef = makeRef(schemaKey(MachineSchemaId::MachineSnapshot));
        const auto driveRef = makeRef(driveSnapshotSchemaKey(8));
        const auto eventPageRef = makeRef(schemaKey(MachineSchemaId::EventPage));
        const auto descriptorSlot = catalog.slotFor(descriptorRef);
        const auto snapshotSlot = catalog.slotFor(snapshotRef);
        const auto driveSlot = catalog.slotFor(driveRef);
        const auto eventPageSlot = catalog.slotFor(eventPageRef);
        if (!descriptorSlot || !snapshotSlot || !driveSlot || !eventPageSlot) return false;

        if (!addStructSignal(registry, kMachineDescriptorSignalId, "machine.descriptor",
                             "Read-only simulated machine identity and axis topology",
                             descriptorRef, *descriptorSlot, [this] { return encodeDescriptor(); },
                             {{"resource.stable_id", descriptor_.machineId}})) return false;
        if (!addStructSignal(registry, kMachineSnapshotSignalId, "machine.snapshot",
                             "Coherent read-only simulated machine state",
                             snapshotRef, *snapshotSlot, [this] { return encodeMachineSnapshot(); },
                             {{"resource.stable_id", descriptor_.machineId}})) return false;
        for (size_t index = 0; index < descriptor_.axes.size(); ++index) {
            const auto& axis = descriptor_.axes[index];
            const uint64_t id = kDriveSignalIdBase + index + 1;
            const std::string name = "drive." + axis.stableId + ".snapshot";
            if (!addStructSignal(registry, id, name, "Read-only simulated CiA 402 drive snapshot",
                                 driveRef, *driveSlot, [this, index] { return encodeDriveSnapshot(index); },
                                 {{"resource.stable_id", axis.stableId}, {"access.role", "observer"},
                                  {"quality.source", "simulation"}})) return false;
        }

        SignalEntry cursorSignal{};
        cursorSignal.id = kEventCursorSignalId;
        cursorSignal.name = "machine.events.cursor";
        cursorSignal.description = "Latest retained simulated event cursor";
        cursorSignal.group = "machine.cia402";
        cursorSignal.valueType = ValueType::U64;
        cursorSignal.metadata = {{"access.role", "observer"}, {"quality.source", "simulation"}};
        cursorSignal.readFn = [this](void* destination) {
            const uint64_t cursor = journal_.latestCursor();
            std::memcpy(destination, &cursor, sizeof(cursor));
        };
        if (!registry.addSignal(std::move(cursorSignal))) return false;

        FunctionEntry readEvents;
        readEvents.id = kEventReadFunctionId;
        readEvents.name = "machine.events.read";
        readEvents.description = "Read a bounded page from the retained machine event journal";
        readEvents.group = "machine.cia402";
        readEvents.parameters = {
            FunctionParameter{"after_cursor", "Exclusive event cursor", ValueType::U64},
            FunctionParameter{"limit", "Maximum events to return (1–100)", ValueType::U32},
        };
        readEvents.parameters[0].key = 1;
        readEvents.parameters[1].key = 2;
        readEvents.parameters[0].metadata["range.min"] = "0";
        readEvents.parameters[1].metadata["range.min"] = "1";
        readEvents.parameters[1].metadata["range.max"] = "100";
        readEvents.returnValue.present = true;
        readEvents.returnValue.name = "page";
        readEvents.returnValue.description = "Cursor page and retained records";
        readEvents.returnValue.type = ValueType::Struct;
        readEvents.returnValue.schema = eventPageRef;
        readEvents.returnValue.schemaSlot = *eventPageSlot;
        readEvents.returnValue.maxValueSize = kMaximumEncodedValue;
        readEvents.metadata = {{"access.role", "observer"}, {"quality.source", "simulation"}};
        readEvents.callback = [this](const std::vector<FunctionArgument>& arguments) {
            FunctionCallResult result;
            if (arguments.size() != 2) {
                result.errorMessage = "machine.events.read requires cursor and limit";
                return result;
            }
            uint64_t cursor = 0;
            uint32_t limit = 0;
            if (!simulated_fleet_detail::decodeLittleEndian(arguments[0].value, cursor) ||
                !simulated_fleet_detail::decodeLittleEndian(arguments[1].value, limit) ||
                limit == 0 || limit > 100) {
                result.errorMessage = "machine.events.read arguments are outside bounds";
                return result;
            }
            result.returnValue = simulated_fleet_detail::encodeEventPage(
                journal_.readAfter(cursor, limit));
            result.success = true;
            result.error = ErrorCode::None;
            return result;
        };
        if (!registry.addFunction(std::move(readEvents))) return false;
        return true;
    }

    size_t axisCount() const { return descriptor_.axes.size(); }
    const MachineDescriptorV1& descriptor() const { return descriptor_; }

    /// Current coherent snapshot for one axis, or nullopt for unknown ids.
    std::optional<DriveSnapshotV1> driveSnapshot(std::string_view stableId) const {
        std::lock_guard lock(mutex_);
        const auto axis = std::find_if(descriptor_.axes.begin(), descriptor_.axes.end(),
            [stableId](const AxisDescriptorV1& candidate) { return candidate.stableId == stableId; });
        if (axis == descriptor_.axes.end()) return std::nullopt;
        return snapshots_[static_cast<size_t>(axis - descriptor_.axes.begin())];
    }

    bool updateDriveSnapshot(std::string_view stableId, DriveSnapshotV1 snapshot) {
        DriveSnapshotV1 previous;
        size_t index = 0;
        {
            std::lock_guard lock(mutex_);
            const auto axis = std::find_if(descriptor_.axes.begin(), descriptor_.axes.end(),
                [stableId](const AxisDescriptorV1& candidate) { return candidate.stableId == stableId; });
            if (axis == descriptor_.axes.end()) return false;
            index = static_cast<size_t>(axis - descriptor_.axes.begin());
            previous = snapshots_[index];
            snapshot.slaveIndex = axis->slaveIndex;
            snapshot.timestampUs = simulated_fleet_detail::monotonicTimestampUs();
            snapshot.stateGeneration = ++stateGeneration_;
            snapshot.qualityFlags |= static_cast<uint32_t>(DriveQuality::Simulated);
            snapshots_[index] = snapshot;
        }
        appendDriveEvents(descriptor_.axes[index], previous, snapshot);
        return true;
    }

    uint64_t appendSimulationEvent(std::string eventType, std::string sourceId,
                                   EventSeverity severity, uint32_t code,
                                   std::string description) {
        uint64_t generation = 0;
        {
            std::lock_guard lock(mutex_);
            generation = stateGeneration_;
        }
        return journal_.append({0, simulated_fleet_detail::monotonicTimestampUs(), generation,
                                std::move(eventType), std::move(sourceId), severity, code,
                                std::move(description)});
    }

    const EventJournal& eventJournal() const noexcept { return journal_; }
    EventJournal& eventJournal() noexcept { return journal_; }

private:
    void appendDriveEvents(const AxisDescriptorV1& axis,
                           const DriveSnapshotV1& previous,
                           const DriveSnapshotV1& current) {
        const auto append = [this, &axis, &current](std::string type, EventSeverity severity,
                                                     uint32_t code, std::string description) {
            journal_.append({0, current.timestampUs, current.stateGeneration, std::move(type),
                             axis.stableId, severity, code, std::move(description)});
        };
        const bool previousFault = previous.faultCode != 0 || previous.ds402State == 7;
        const bool currentFault = current.faultCode != 0 || current.ds402State == 7;
        if (currentFault != previousFault) {
            append(currentFault ? "FaultSet" : "FaultCleared",
                   currentFault ? EventSeverity::Fault : EventSeverity::Info,
                   current.faultCode,
                   currentFault ? "Drive entered a fault state" : "Drive fault state cleared");
        }
        const bool previousWarning = (previous.statusWord & (1U << 7)) != 0;
        const bool currentWarning = (current.statusWord & (1U << 7)) != 0;
        if (currentWarning != previousWarning) {
            append(currentWarning ? "WarningSet" : "WarningCleared",
                   currentWarning ? EventSeverity::Warning : EventSeverity::Info,
                   current.statusWord,
                   currentWarning ? "Drive warning bit set" : "Drive warning bit cleared");
        }
        if (current.alState != previous.alState || current.ds402State != previous.ds402State) {
            append("DriveStateChanged", EventSeverity::Info, current.ds402State,
                   "Drive EtherCAT or CiA 402 state changed");
        }
    }

    void validateDescriptor() const {
        if (descriptor_.axes.empty() || descriptor_.axes.size() > kMaximumProfileAxes ||
            !validText(descriptor_.machineId) || !validText(descriptor_.displayName) ||
            !validText(descriptor_.timezone) || !validText(descriptor_.unitSystem)) {
            throw std::invalid_argument("simulated machine descriptor is outside profile bounds");
        }
        for (const auto& axis : descriptor_.axes) {
            if (!validText(axis.stableId) || !validText(axis.name) || !validText(axis.groupId) ||
                !validText(axis.positionUnit) || !validText(axis.velocityUnit) ||
                !std::isfinite(axis.positionScale) || !std::isfinite(axis.velocityScale) ||
                axis.positionScale == 0.0 || axis.velocityScale == 0.0) {
                throw std::invalid_argument("simulated axis descriptor is invalid");
            }
        }
    }

    static bool validText(std::string_view value) {
        return !value.empty() && value.size() <= 127 && value.find('\0') == std::string_view::npos;
    }

    void initializeSnapshots() {
        const uint64_t now = simulated_fleet_detail::monotonicTimestampUs();
        for (size_t index = 0; index < snapshots_.size(); ++index) {
            auto& snapshot = snapshots_[index];
            snapshot.timestampUs = now;
            snapshot.stateGeneration = stateGeneration_;
            snapshot.slaveIndex = descriptor_.axes[index].slaveIndex;
            snapshot.alState = 8;
            snapshot.ds402State = 4;
            snapshot.statusWord = 0x1637;
            snapshot.controlWord = 0x000F;
            snapshot.targetMode = 8;
            snapshot.displayMode = 8;
            snapshot.targetPosition = static_cast<int32_t>(index * 10000);
            snapshot.demandPosition = snapshot.targetPosition - 2;
            snapshot.actualPosition = snapshot.targetPosition - 3;
            snapshot.followingError = 1;
            snapshot.qualityFlags = static_cast<uint32_t>(DriveQuality::Simulated);
        }
        // Demonstrate a non-fault warning while keeping the default fleet enabled.
        snapshots_[2].statusWord |= 1U << 7;
    }

    template <typename Encoder>
    bool addStructSignal(Registry& registry, uint64_t id, const std::string& name,
                         const std::string& description, const SchemaRef& schema,
                         SchemaSlot slot, Encoder encoder,
                         std::map<std::string, std::string> metadata) {
        SignalEntry signal{};
        signal.id = id;
        signal.name = name;
        signal.description = description;
        signal.group = "machine.cia402";
        signal.valueType = ValueType::Struct;
        signal.schema = schema;
        signal.schemaSlot = slot;
        signal.maxValueSize = kMaximumEncodedValue;
        signal.metadata = std::move(metadata);
        signal.metadata["schema.name"] = name == "machine.descriptor"
            ? std::string{kMachineDescriptorSchemaName}
            : name == "machine.snapshot" ? std::string{kMachineSnapshotSchemaName}
                                          : std::string{kDriveSnapshotSchemaName};
        signal.varReadFn = [encoder = std::move(encoder)](void* destination, size_t capacity) {
            const std::vector<uint8_t> bytes = encoder();
            if (bytes.size() > capacity) return size_t{0};
            std::memcpy(destination, bytes.data(), bytes.size());
            return bytes.size();
        };
        return registry.addSignal(std::move(signal));
    }

    std::vector<uint8_t> encodeDescriptor() const {
        return simulated_fleet_detail::encodeDescriptor(descriptor_);
    }

    std::vector<uint8_t> encodeDriveSnapshot(size_t index) const {
        DriveSnapshotV1 snapshot;
        {
            std::lock_guard lock(mutex_);
            snapshot = snapshots_[index];
        }
        std::array<uint8_t, DriveSnapshotV1::kEncodedSize> bytes{};
        BufWriter writer(bytes.data(), bytes.size());
        snapshot.encode(writer);
        return writer.ok() ? std::vector<uint8_t>(bytes.begin(), bytes.end()) : std::vector<uint8_t>{};
    }

    std::vector<uint8_t> encodeMachineSnapshot() const {
        MachineSnapshotV1 aggregate;
        aggregate.timestampUs = simulated_fleet_detail::monotonicTimestampUs();
        aggregate.axisCount = static_cast<uint16_t>(snapshots_.size());
        aggregate.expectedWkc = static_cast<uint32_t>(snapshots_.size() * 2);
        aggregate.actualWkc = aggregate.expectedWkc;
        {
            std::lock_guard lock(mutex_);
            aggregate.stateGeneration = stateGeneration_;
            for (const auto& snapshot : snapshots_) {
                aggregate.timestampUs = std::max(aggregate.timestampUs, snapshot.timestampUs);
                if (snapshot.ds402State == 4) ++aggregate.enabledCount;
                if (snapshot.faultCode != 0 || snapshot.ds402State == 7) ++aggregate.faultCount;
                if ((snapshot.statusWord & (1U << 7)) != 0) ++aggregate.warningCount;
                if ((snapshot.qualityFlags & static_cast<uint32_t>(DriveQuality::Stale)) != 0)
                    ++aggregate.staleCount;
                if (snapshot.alState != 8) aggregate.alState = std::min(aggregate.alState, snapshot.alState);
            }
        }
        return simulated_fleet_detail::encodeMachineSnapshot(aggregate);
    }

    MachineDescriptorV1 descriptor_;
    mutable std::mutex mutex_;
    std::vector<DriveSnapshotV1> snapshots_;
    uint64_t stateGeneration_ = 1;
    EventJournal journal_;
};

} // namespace tether::io::cia402
