#pragma once

/**
 * @file CiA402MachineProfile.hpp
 * @brief Versioned machine-level schemas for the machine.cia402.v1 profile.
 */

#include "tether/io/CiA402Profile.hpp"
#include "tether/io/SchemaCatalog.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace tether::io::cia402 {

inline constexpr std::string_view kMachineDescriptorSchemaName =
    "tether.machine.cia402.MachineDescriptorV1";
inline constexpr std::string_view kMachineSnapshotSchemaName =
    "tether.machine.cia402.MachineSnapshotV1";
inline constexpr size_t kMaximumProfileAxes = 16;

inline SchemaKey machineProfileSchemaKey(uint8_t id) {
    SchemaKey key{};
    key[15] = id;
    return key;
}

inline SchemaKey machineDescriptorSchemaKey() { return machineProfileSchemaKey(14); }
inline SchemaKey machineSnapshotSchemaKey() { return machineProfileSchemaKey(15); }
inline SchemaKey eventRecordSchemaKey() { return machineProfileSchemaKey(16); }
inline SchemaKey eventRecordArraySchemaKey() { return machineProfileSchemaKey(17); }
inline SchemaKey eventPageSchemaKey() { return machineProfileSchemaKey(18); }

inline SchemaGraph machineProfileSchemaGraph() {
    SchemaGraph graph = driveSnapshotSchemaGraph();
    const auto makeScalar = [](uint8_t id, ValueType type, std::string name) {
        SchemaNode node;
        node.key = machineProfileSchemaKey(id);
        node.kind = SchemaKind::Scalar;
        node.scalarType = type;
        node.name = std::move(name);
        return node;
    };
    SchemaNode stringNode;
    stringNode.key = machineProfileSchemaKey(9);
    stringNode.kind = SchemaKind::String;
    stringNode.maxBytes = 128;
    stringNode.name = "tether.machine.cia402.String128";
    SchemaNode eventDescriptionNode;
    eventDescriptionNode.key = machineProfileSchemaKey(19);
    eventDescriptionNode.kind = SchemaKind::String;
    eventDescriptionNode.maxBytes = 512;
    eventDescriptionNode.name = "tether.machine.cia402.EventDescription512";
    auto booleanNode = makeScalar(10, ValueType::Bool, "tether.machine.cia402.Bool");
    auto f64Node = makeScalar(11, ValueType::F64, "tether.machine.cia402.F64");

    const auto ref = [](const SchemaNode& node) {
        return SchemaRef{node.key, computeSchemaDigest(node)};
    };
    const auto findRef = [&graph](uint8_t key) {
        const auto* node = graph.find(machineProfileSchemaKey(key));
        return SchemaRef{node->key, computeSchemaDigest(*node)};
    };
    const auto stringRef = ref(stringNode);
    const auto eventDescriptionRef = ref(eventDescriptionNode);
    const auto boolRef = ref(booleanNode);
    const auto f64Ref = ref(f64Node);
    const auto u8Ref = findRef(1);
    const auto u16Ref = findRef(2);
    const auto u32Ref = findRef(3);
    const auto u64Ref = findRef(4);

    SchemaNode axis;
    axis.key = machineProfileSchemaKey(12);
    axis.kind = SchemaKind::Struct;
    axis.structEncoding = StructEncoding::Tagged;
    axis.name = "tether.machine.cia402.AxisDescriptorV1";
    axis.description = "Stable, read-only identity and engineering units for one simulated axis";
    axis.fields = {
        {1, 0, stringRef, "stable_id", "Stable application resource identifier"},
        {2, 0, stringRef, "name", "Human-readable axis name"},
        {3, 0, u16Ref, "slave_index", "EtherCAT slave position"},
        {4, 0, u16Ref, "display_order", "Preferred fleet display order"},
        {5, 0, stringRef, "group_id", "Motion-group resource identifier"},
        {6, 0, stringRef, "position_unit", "Application position unit"},
        {7, 0, f64Ref, "position_scale", "Application units per drive-native count"},
        {8, 0, stringRef, "velocity_unit", "Application velocity unit"},
        {9, 0, f64Ref, "velocity_scale", "Application units per drive-native count per second"},
        {10, 0, boolRef, "supports_homing", "Homing is configured by the application"},
    };

    SchemaNode axes;
    axes.key = machineProfileSchemaKey(13);
    axes.kind = SchemaKind::DynamicArray;
    axes.name = "tether.machine.cia402.AxisDescriptorArrayV1";
    axes.element = ref(axis);
    axes.minCount = 1;
    axes.maxCount = static_cast<uint32_t>(kMaximumProfileAxes);

    SchemaNode descriptor;
    descriptor.key = machineDescriptorSchemaKey();
    descriptor.kind = SchemaKind::Struct;
    descriptor.structEncoding = StructEncoding::Tagged;
    descriptor.name = std::string{kMachineDescriptorSchemaName};
    descriptor.description = "Application-owned machine identity and axis topology";
    descriptor.fields = {
        {1, 0, u32Ref, "profile_version", "Machine profile version"},
        {2, 0, stringRef, "machine_id", "Stable machine resource identifier"},
        {3, 0, stringRef, "display_name", "Machine display name"},
        {4, 0, stringRef, "timezone", "IANA timezone identifier"},
        {5, 0, stringRef, "unit_system", "Declared engineering unit system"},
        {6, 0, ref(axes), "axes", "Stable axis identities and unit conversions"},
    };

    SchemaNode snapshot;
    snapshot.key = machineSnapshotSchemaKey();
    snapshot.kind = SchemaKind::Struct;
    snapshot.structEncoding = StructEncoding::Tagged;
    snapshot.name = std::string{kMachineSnapshotSchemaName};
    snapshot.description = "Coherent, read-only aggregate machine status";
    snapshot.fields = {
        {1, 0, u64Ref, "timestamp_us", "Snapshot source timestamp in microseconds"},
        {2, 0, u64Ref, "state_generation", "Monotonic safety-relevant state generation"},
        {3, 0, boolRef, "simulated", "Snapshot originates from a simulator"},
        {4, 0, u16Ref, "axis_count", "Number of configured axes"},
        {5, 0, u16Ref, "enabled_count", "Number of operation-enabled axes"},
        {6, 0, u16Ref, "fault_count", "Number of faulted axes"},
        {7, 0, u16Ref, "warning_count", "Number of axes reporting a warning"},
        {8, 0, u16Ref, "stale_count", "Number of stale axis snapshots"},
        {9, 0, u8Ref, "al_state", "Aggregate EtherCAT AL state"},
        {10, 0, u32Ref, "expected_wkc", "Expected working counter in the simulation"},
        {11, 0, u32Ref, "actual_wkc", "Observed working counter in the simulation"},
        {12, 0, boolRef, "link_up", "Simulated EtherCAT link state"},
        {13, 0, boolRef, "dc_locked", "Simulated distributed-clock lock state"},
    };

    SchemaNode eventRecord;
    eventRecord.key = eventRecordSchemaKey();
    eventRecord.kind = SchemaKind::Struct;
    eventRecord.structEncoding = StructEncoding::Tagged;
    eventRecord.name = "tether.machine.cia402.EventRecordV1";
    eventRecord.description = "Immutable event journal record with a monotonically increasing cursor";
    eventRecord.fields = {
        {1, 0, u64Ref, "event_id", "Monotonic event cursor"},
        {2, 0, u64Ref, "timestamp_us", "Source timestamp in microseconds"},
        {3, 0, u64Ref, "state_generation", "Related machine state generation"},
        {4, 0, stringRef, "event_type", "Stable event type identifier"},
        {5, 0, stringRef, "source_id", "Stable machine or axis resource identifier"},
        {6, 0, u8Ref, "severity", "0 info, 1 warning, 2 fault, 3 critical"},
        {7, 0, u32Ref, "code", "Profile-defined event or fault code"},
        {8, 0, eventDescriptionRef, "description", "Human-readable event description"},
    };

    SchemaNode eventArray;
    eventArray.key = eventRecordArraySchemaKey();
    eventArray.kind = SchemaKind::DynamicArray;
    eventArray.name = "tether.machine.cia402.EventRecordArrayV1";
    eventArray.element = ref(eventRecord);
    eventArray.minCount = 0;
    eventArray.maxCount = 100;

    SchemaNode eventPage;
    eventPage.key = eventPageSchemaKey();
    eventPage.kind = SchemaKind::Struct;
    eventPage.structEncoding = StructEncoding::Tagged;
    eventPage.name = "tether.machine.cia402.EventPageV1";
    eventPage.description = "Bounded event history page with explicit cursor-gap reporting";
    eventPage.fields = {
        {1, 0, u64Ref, "oldest_cursor", "Oldest event retained by the server, or zero"},
        {2, 0, u64Ref, "latest_cursor", "Latest event cursor currently retained"},
        {3, 0, u64Ref, "next_cursor", "Cursor to use for the next page"},
        {4, 0, boolRef, "gap", "Requested cursor precedes retained history"},
        {5, 0, ref(eventArray), "events", "Ordered events after the requested cursor"},
    };

    graph.nodes.push_back(std::move(stringNode));
    graph.nodes.push_back(std::move(eventDescriptionNode));
    graph.nodes.push_back(std::move(booleanNode));
    graph.nodes.push_back(std::move(f64Node));
    graph.nodes.push_back(std::move(axis));
    graph.nodes.push_back(std::move(axes));
    graph.nodes.push_back(std::move(descriptor));
    graph.nodes.push_back(std::move(snapshot));
    graph.nodes.push_back(std::move(eventRecord));
    graph.nodes.push_back(std::move(eventArray));
    graph.nodes.push_back(std::move(eventPage));
    return graph;
}

inline std::vector<SchemaManifestEntry> machineProfileManifest(const SchemaGraph& graph) {
    std::vector<SchemaManifestEntry> manifest;
    for (const auto& key : {driveSnapshotSchemaKey(8), machineDescriptorSchemaKey(),
                            machineSnapshotSchemaKey(), eventPageSchemaKey()}) {
        const auto* node = graph.find(key);
        if (!node) return {};
        manifest.push_back({SchemaRef{node->key, computeSchemaDigest(*node)}, node->revision});
    }
    return manifest;
}

} // namespace tether::io::cia402
