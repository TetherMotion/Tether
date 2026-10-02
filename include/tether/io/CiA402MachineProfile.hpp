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

/// Slot identifiers for every node in the machine-profile schema graph.
/// Ids 1–8 belong to the packed DriveSnapshotV1 subgraph and 9–13/19 are
/// shared primitives; the enum names document every remaining assignment.
enum class MachineSchemaId : uint8_t {
    String128 = 9,
    Bool = 10,
    F64 = 11,
    AxisDescriptor = 12,
    AxisDescriptorArray = 13,
    MachineDescriptor = 14,
    MachineSnapshot = 15,
    EventRecord = 16,
    EventRecordArray = 17,
    EventPage = 18,
    EventDescription512 = 19,
    CommandParameters = 20,
    TargetArray = 21,
    GenerationArray = 22,
    AuthorityLease = 23,
    AuthorityLeaseArray = 24,
    AuthoritySnapshot = 25,
    AuthorityResult = 26,
    CommandRequest = 27,
    CommandReceipt = 28,
    OperationSnapshot = 29,
    AlarmRecord = 30,
    AlarmRecordArray = 31,
    AlarmPage = 32,
    BlockerArray = 33,
    OperationArray = 34,
    CaptureStatus = 35,
    ConfigEntry = 36,
    ConfigEntryArray = 37,
    ConfigStatus = 38,
    ConfigValueBytes = 39,
    SdoDataBytes = 40,
    SdoEntry = 41,
    SdoEntryArray = 42,
    SdoRequest = 43,
    SdoWriteRequest = 44,
    SdoResult = 45,
    CapturePayload = 46,
    CaptureExport = 47,
    CaptureField = 48,
    CaptureFieldArray = 49,
    MachineDiagnostics = 50,
    PdoEntry = 51,
    PdoEntryArray = 52,
    SupervisorEntry = 53,
    SupervisorEntryArray = 54,
    SupervisorResult = 55,
    ConfigExport = 56,
    ConfigDiffEntry = 57,
    ConfigDiffEntryArray = 58,
    ConfigDiff = 59,
    ChecklistItem = 60,
    ChecklistItemArray = 61,
    ChecklistReport = 62,
    AppProfile = 63,
    RecipeInfo = 64,
    RecipeInfoArray = 65,
    Metrics = 66,
};

constexpr SchemaKey schemaKey(MachineSchemaId id) {
    SchemaKey key{};
    key[15] = static_cast<uint8_t>(id);
    return key;
}

inline SchemaGraph machineProfileSchemaGraph() {
    SchemaGraph graph = driveSnapshotSchemaGraph();
    const auto makeScalar = [](MachineSchemaId id, ValueType type, std::string name) {
        SchemaNode node;
        node.key = schemaKey(id);
        node.kind = SchemaKind::Scalar;
        node.scalarType = type;
        node.name = std::move(name);
        return node;
    };
    SchemaNode stringNode;
    stringNode.key = schemaKey(MachineSchemaId::String128);
    stringNode.kind = SchemaKind::String;
    stringNode.maxBytes = 128;
    stringNode.name = "tether.machine.cia402.String128";
    SchemaNode eventDescriptionNode;
    eventDescriptionNode.key = schemaKey(MachineSchemaId::EventDescription512);
    eventDescriptionNode.kind = SchemaKind::String;
    eventDescriptionNode.maxBytes = 512;
    eventDescriptionNode.name = "tether.machine.cia402.EventDescription512";
    auto booleanNode = makeScalar(MachineSchemaId::Bool, ValueType::Bool, "tether.machine.cia402.Bool");
    auto f64Node = makeScalar(MachineSchemaId::F64, ValueType::F64, "tether.machine.cia402.F64");

    const auto ref = [](const SchemaNode& node) {
        return SchemaRef{node.key, computeSchemaDigest(node)};
    };
    const auto findRef = [&graph](uint8_t id) {
        const auto* node = graph.find(driveSnapshotSchemaKey(id));
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
    axis.key = schemaKey(MachineSchemaId::AxisDescriptor);
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
    axes.key = schemaKey(MachineSchemaId::AxisDescriptorArray);
    axes.kind = SchemaKind::DynamicArray;
    axes.name = "tether.machine.cia402.AxisDescriptorArrayV1";
    axes.element = ref(axis);
    axes.minCount = 1;
    axes.maxCount = static_cast<uint32_t>(kMaximumProfileAxes);

    SchemaNode descriptor;
    descriptor.key = schemaKey(MachineSchemaId::MachineDescriptor);
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
    snapshot.key = schemaKey(MachineSchemaId::MachineSnapshot);
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
    eventRecord.key = schemaKey(MachineSchemaId::EventRecord);
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
    eventArray.key = schemaKey(MachineSchemaId::EventRecordArray);
    eventArray.kind = SchemaKind::DynamicArray;
    eventArray.name = "tether.machine.cia402.EventRecordArrayV1";
    eventArray.element = ref(eventRecord);
    eventArray.minCount = 0;
    eventArray.maxCount = 100;

    SchemaNode eventPage;
    eventPage.key = schemaKey(MachineSchemaId::EventPage);
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

    SchemaNode commandParameters;
    commandParameters.key = schemaKey(MachineSchemaId::CommandParameters);
    commandParameters.kind = SchemaKind::Bytes;
    commandParameters.maxBytes = 1024;
    commandParameters.name = "tether.machine.cia402.CommandParameters";
    commandParameters.description = "Opaque action-specific parameters, validated server-side";

    SchemaNode targetArray;
    targetArray.key = schemaKey(MachineSchemaId::TargetArray);
    targetArray.kind = SchemaKind::DynamicArray;
    targetArray.name = "tether.machine.cia402.TargetArrayV1";
    targetArray.element = stringRef;
    targetArray.minCount = 1;
    targetArray.maxCount = 128;

    SchemaNode generationArray;
    generationArray.key = schemaKey(MachineSchemaId::GenerationArray);
    generationArray.kind = SchemaKind::DynamicArray;
    generationArray.name = "tether.machine.cia402.GenerationArrayV1";
    generationArray.element = u64Ref;
    generationArray.minCount = 0;
    generationArray.maxCount = 128;

    SchemaNode blockerArray;
    blockerArray.key = schemaKey(MachineSchemaId::BlockerArray);
    blockerArray.kind = SchemaKind::DynamicArray;
    blockerArray.name = "tether.machine.cia402.BlockerArrayV1";
    blockerArray.element = eventDescriptionRef;
    blockerArray.minCount = 0;
    blockerArray.maxCount = 64;

    SchemaNode authorityLease;
    authorityLease.key = schemaKey(MachineSchemaId::AuthorityLease);
    authorityLease.kind = SchemaKind::Struct;
    authorityLease.structEncoding = StructEncoding::Tagged;
    authorityLease.name = "tether.machine.cia402.AuthorityLeaseV1";
    authorityLease.description = "One server-owned control-authority lease";
    authorityLease.fields = {
        {1, 0, stringRef, "scope", "Machine or motion-group scope"},
        {2, 0, stringRef, "owner_actor", "Authenticated actor holding the lease"},
        {3, 0, u64Ref, "token", "Server-issued lease token"},
        {4, 0, u32Ref, "remaining_ms", "Milliseconds until server-side expiry"},
    };

    SchemaNode authorityLeaseArray;
    authorityLeaseArray.key = schemaKey(MachineSchemaId::AuthorityLeaseArray);
    authorityLeaseArray.kind = SchemaKind::DynamicArray;
    authorityLeaseArray.name = "tether.machine.cia402.AuthorityLeaseArrayV1";
    authorityLeaseArray.element = ref(authorityLease);
    authorityLeaseArray.minCount = 0;
    authorityLeaseArray.maxCount = 16;

    SchemaNode authoritySnapshot;
    authoritySnapshot.key = schemaKey(MachineSchemaId::AuthoritySnapshot);
    authoritySnapshot.kind = SchemaKind::Struct;
    authoritySnapshot.structEncoding = StructEncoding::Tagged;
    authoritySnapshot.name = "tether.machine.cia402.AuthoritySnapshotV1";
    authoritySnapshot.description = "Read-only view of every active control-authority lease";
    authoritySnapshot.fields = {
        {1, 0, u64Ref, "timestamp_us", "Snapshot source timestamp in microseconds"},
        {2, 0, ref(authorityLeaseArray), "leases", "Active leases, ordered by scope"},
    };

    SchemaNode authorityResult;
    authorityResult.key = schemaKey(MachineSchemaId::AuthorityResult);
    authorityResult.kind = SchemaKind::Struct;
    authorityResult.structEncoding = StructEncoding::Tagged;
    authorityResult.name = "tether.machine.cia402.AuthorityResultV1";
    authorityResult.description = "Result of an authority acquire/renew/release request";
    authorityResult.fields = {
        {1, 0, boolRef, "granted", "Whether the request was granted"},
        {2, 0, stringRef, "scope", "Scope the result applies to"},
        {3, 0, u64Ref, "token", "Lease token, or zero when rejected"},
        {4, 0, u32Ref, "remaining_ms", "Milliseconds until lease expiry"},
        {5, 0, stringRef, "owner_actor", "Current owner when the request conflicts"},
        {6, 0, eventDescriptionRef, "blocker", "Structured refusal reason, empty on success"},
    };

    SchemaNode commandRequest;
    commandRequest.key = schemaKey(MachineSchemaId::CommandRequest);
    commandRequest.kind = SchemaKind::Struct;
    commandRequest.structEncoding = StructEncoding::Tagged;
    commandRequest.name = "tether.machine.cia402.CommandRequestV1";
    commandRequest.description = "Idempotent, generation-checked machine command";
    commandRequest.fields = {
        {1, 0, stringRef, "request_uuid", "Client-generated idempotency key"},
        {2, 0, stringRef, "scope", "Authority scope covering all targets"},
        {3, 0, u64Ref, "authority_token", "Lease token from machine.authority.acquire"},
        {4, 0, u8Ref, "action", "Profile action identifier"},
        {5, 0, ref(targetArray), "targets", "Stable resource identifiers"},
        {6, 0, ref(generationArray), "expected_generations",
            "Per-target expected state_generation, ordered as targets"},
        {7, 0, u64Ref, "deadline_us",
            "Validity budget in microseconds, relative to server receipt"},
        {8, 0, u64Ref, "configuration_revision",
            "Expected configuration revision, or zero when not checked"},
        {9, 0, ref(commandParameters), "parameters", "Action-specific parameter payload"},
    };

    SchemaNode commandReceipt;
    commandReceipt.key = schemaKey(MachineSchemaId::CommandReceipt);
    commandReceipt.kind = SchemaKind::Struct;
    commandReceipt.structEncoding = StructEncoding::Tagged;
    commandReceipt.name = "tether.machine.cia402.CommandReceiptV1";
    commandReceipt.description = "Validated command outcome with structured blockers";
    commandReceipt.fields = {
        {1, 0, stringRef, "request_uuid", "Echoed idempotency key"},
        {2, 0, u8Ref, "state", "0 in-progress, 1 accepted, 2 rejected, 3 failed"},
        {3, 0, eventDescriptionRef, "message", "Primary human-readable outcome"},
        {4, 0, stringRef, "operation_id", "Server operation identifier, empty when rejected"},
        {5, 0, stringRef, "audit_id", "Durable audit record identifier"},
        {6, 0, ref(blockerArray), "blockers", "Every precondition that failed"},
    };

    SchemaNode operationSnapshot;
    operationSnapshot.key = schemaKey(MachineSchemaId::OperationSnapshot);
    operationSnapshot.kind = SchemaKind::Struct;
    operationSnapshot.structEncoding = StructEncoding::Tagged;
    operationSnapshot.name = "tether.machine.cia402.OperationSnapshotV1";
    operationSnapshot.description = "Progress and terminal state of one dispatched command";
    operationSnapshot.fields = {
        {1, 0, stringRef, "operation_id", "Server operation identifier"},
        {2, 0, stringRef, "request_uuid", "Originating idempotency key"},
        {3, 0, u8Ref, "state", "0 queued, 1 running, 2 completed, 3 failed, 4 cancelled"},
        {4, 0, u8Ref, "progress", "Completion percentage, 0-100"},
        {5, 0, u8Ref, "action", "Command action identifier"},
        {6, 0, stringRef, "target", "Primary affected resource"},
        {7, 0, eventDescriptionRef, "message", "Latest operation status"},
        {8, 0, u64Ref, "started_us", "Dispatch timestamp in microseconds"},
        {9, 0, u64Ref, "finished_us", "Terminal timestamp, or zero while running"},
        {10, 0, u32Ref, "result_code", "Application-defined result code"},
    };

    SchemaNode alarmRecord;
    alarmRecord.key = schemaKey(MachineSchemaId::AlarmRecord);
    alarmRecord.kind = SchemaKind::Struct;
    alarmRecord.structEncoding = StructEncoding::Tagged;
    alarmRecord.name = "tether.machine.cia402.AlarmRecordV1";
    alarmRecord.description = "Alarm lifecycle record with acknowledgement state";
    alarmRecord.fields = {
        {1, 0, u64Ref, "alarm_id", "Monotonic alarm cursor"},
        {2, 0, u64Ref, "raised_us", "Raise timestamp in microseconds"},
        {3, 0, u64Ref, "updated_us", "Last state-change timestamp"},
        {4, 0, u8Ref, "state", "0 active, 1 acknowledged, 2 cleared"},
        {5, 0, u8Ref, "severity", "0 info, 1 warning, 2 fault, 3 critical"},
        {6, 0, u32Ref, "code", "Profile-defined alarm code"},
        {7, 0, stringRef, "source_id", "Stable machine or axis resource identifier"},
        {8, 0, eventDescriptionRef, "description", "Human-readable alarm text"},
        {9, 0, stringRef, "actor", "Actor who acknowledged or cleared the alarm"},
    };

    SchemaNode alarmArray;
    alarmArray.key = schemaKey(MachineSchemaId::AlarmRecordArray);
    alarmArray.kind = SchemaKind::DynamicArray;
    alarmArray.name = "tether.machine.cia402.AlarmRecordArrayV1";
    alarmArray.element = ref(alarmRecord);
    alarmArray.minCount = 0;
    alarmArray.maxCount = 100;

    SchemaNode alarmPage;
    alarmPage.key = schemaKey(MachineSchemaId::AlarmPage);
    alarmPage.kind = SchemaKind::Struct;
    alarmPage.structEncoding = StructEncoding::Tagged;
    alarmPage.name = "tether.machine.cia402.AlarmPageV1";
    alarmPage.description = "Bounded alarm history page with cursor-gap reporting";
    alarmPage.fields = {
        {1, 0, u64Ref, "oldest_cursor", "Oldest alarm retained by the server, or zero"},
        {2, 0, u64Ref, "latest_cursor", "Latest alarm cursor currently retained"},
        {3, 0, u64Ref, "next_cursor", "Cursor to use for the next page"},
        {4, 0, boolRef, "gap", "Requested cursor precedes retained history"},
        {5, 0, ref(alarmArray), "alarms", "Ordered alarms after the requested cursor"},
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
    graph.nodes.push_back(std::move(commandParameters));
    graph.nodes.push_back(std::move(targetArray));
    graph.nodes.push_back(std::move(generationArray));
    graph.nodes.push_back(std::move(blockerArray));
    graph.nodes.push_back(std::move(authorityLease));
    graph.nodes.push_back(std::move(authorityLeaseArray));
    graph.nodes.push_back(std::move(authoritySnapshot));
    graph.nodes.push_back(std::move(authorityResult));
    graph.nodes.push_back(std::move(commandRequest));
    graph.nodes.push_back(std::move(commandReceipt));
    graph.nodes.push_back(std::move(operationSnapshot));
    SchemaNode operationArray;
    operationArray.key = schemaKey(MachineSchemaId::OperationArray);
    operationArray.kind = SchemaKind::DynamicArray;
    operationArray.name = "tether.machine.cia402.OperationArrayV1";
    operationArray.element = ref(operationSnapshot);
    operationArray.minCount = 0;
    operationArray.maxCount = 64;

    SchemaNode captureStatus;
    captureStatus.key = schemaKey(MachineSchemaId::CaptureStatus);
    captureStatus.kind = SchemaKind::Struct;
    captureStatus.structEncoding = StructEncoding::Tagged;
    captureStatus.name = "tether.machine.cia402.CaptureStatusV1";
    captureStatus.description = "Server-side recording/capture status";
    captureStatus.fields = {
        {1, 0, u64Ref, "timestamp_us", "Status timestamp in microseconds"},
        {2, 0, u8Ref, "state", "0 idle, 1 recording, 2 stopped, 3 error"},
        {3, 0, u64Ref, "records_written", "Records persisted so far"},
        {4, 0, u64Ref, "bytes_written", "Bytes persisted so far"},
        {5, 0, stringRef, "log_name", "Capture/log identifier"},
        {6, 0, u32Ref, "sample_rate_hz", "Nominal sample rate"},
        {7, 0, u32Ref, "field_count", "Fields per record"},
        {8, 0, boolRef, "enabled", "Recording requested"},
        {9, 0, u8Ref, "trigger_state",
            "0 none, 1 armed (pre-trigger ring), 2 capturing post-trigger, 3 complete"},
        {10, 0, u64Ref, "records_available", "Records retained and exportable"},
        {11, 0, u64Ref, "records_dropped", "Records evicted by retention limits"},
        {12, 0, SchemaRef{}, "record_fields", "Layout of each exported record"},
    };

    SchemaNode captureField;
    captureField.key = schemaKey(MachineSchemaId::CaptureField);
    captureField.kind = SchemaKind::Struct;
    captureField.structEncoding = StructEncoding::Tagged;
    captureField.name = "tether.machine.cia402.CaptureFieldV1";
    captureField.description = "One field inside an exported capture record";
    captureField.fields = {
        {1, 0, u64Ref, "entry_id", "Catalog entry this field was read from"},
        {2, 0, u32Ref, "offset", "Byte offset inside the record"},
        {3, 0, u32Ref, "size", "Byte size of this field"},
    };

    SchemaNode captureFieldArray;
    captureFieldArray.key = schemaKey(MachineSchemaId::CaptureFieldArray);
    captureFieldArray.kind = SchemaKind::DynamicArray;
    captureFieldArray.name = "tether.machine.cia402.CaptureFieldArrayV1";
    captureFieldArray.element = ref(captureField);
    captureFieldArray.minCount = 0;
    captureFieldArray.maxCount = 512;
    // Resolve the back-reference now that the array node exists.
    captureStatus.fields[11].schema = ref(captureFieldArray);

    SchemaNode capturePayload;
    capturePayload.key = schemaKey(MachineSchemaId::CapturePayload);
    capturePayload.kind = SchemaKind::Bytes;
    capturePayload.maxBytes = 65536;
    capturePayload.name = "tether.machine.cia402.CapturePayloadBytes";
    capturePayload.description = "One export chunk of concatenated capture records";

    SchemaNode captureExport;
    captureExport.key = schemaKey(MachineSchemaId::CaptureExport);
    captureExport.kind = SchemaKind::Struct;
    captureExport.structEncoding = StructEncoding::Tagged;
    captureExport.name = "tether.machine.cia402.CaptureExportV1";
    captureExport.description =
        "Bounded capture export chunk — paginate via offset until offset + "
        "records reaches total_records";
    captureExport.fields = {
        {1, 0, u64Ref, "offset", "Record offset this chunk starts at"},
        {2, 0, u64Ref, "total_records", "Records retained on the server"},
        {3, 0, u64Ref, "dropped_records", "Records evicted by retention limits"},
        {4, 0, u32Ref, "record_size", "Bytes per record (0 when unknown)"},
        {5, 0, ref(capturePayload), "payload", "Concatenated record bytes"},
    };

    SchemaNode configValue;
    configValue.key = schemaKey(MachineSchemaId::ConfigValueBytes);
    configValue.kind = SchemaKind::Bytes;
    configValue.maxBytes = 512;
    configValue.name = "tether.machine.cia402.ConfigValueBytes";
    configValue.description = "Raw encoded parameter value for a staged configuration write";

    SchemaNode configEntry;
    configEntry.key = schemaKey(MachineSchemaId::ConfigEntry);
    configEntry.kind = SchemaKind::Struct;
    configEntry.structEncoding = StructEncoding::Tagged;
    configEntry.name = "tether.machine.cia402.ConfigEntryV1";
    configEntry.description = "One staged parameter write";
    configEntry.fields = {
        {1, 0, u64Ref, "entry_id", "Parameter id to write on commit"},
        {2, 0, ref(configValue), "value", "Encoded parameter value bytes"},
    };

    SchemaNode configEntryArray;
    configEntryArray.key = schemaKey(MachineSchemaId::ConfigEntryArray);
    configEntryArray.kind = SchemaKind::DynamicArray;
    configEntryArray.name = "tether.machine.cia402.ConfigEntryArrayV1";
    configEntryArray.element = ref(configEntry);
    configEntryArray.minCount = 0;
    configEntryArray.maxCount = 256;

    SchemaNode configStatus;
    configStatus.key = schemaKey(MachineSchemaId::ConfigStatus);
    configStatus.kind = SchemaKind::Struct;
    configStatus.structEncoding = StructEncoding::Tagged;
    configStatus.name = "tether.machine.cia402.ConfigStatusV1";
    configStatus.description = "Staged configuration transaction status";
    configStatus.fields = {
        {1, 0, u64Ref, "transaction_id", "Monotonic transaction identifier"},
        {2, 0, u8Ref, "state",
            "0 idle, 1 staged, 2 validated, 3 committed, 4 failed, 5 rolled back"},
        {3, 0, u64Ref, "revision", "Active configuration revision"},
        {4, 0, u32Ref, "staged_count", "Entries in the staged transaction"},
        {5, 0, eventDescriptionRef, "message", "Last transaction outcome"},
        {6, 0, ref(configEntryArray), "staged", "Entries staged for the next commit"},
    };

    SchemaNode machineDiagnostics;
    machineDiagnostics.key = schemaKey(MachineSchemaId::MachineDiagnostics);
    machineDiagnostics.kind = SchemaKind::Struct;
    machineDiagnostics.structEncoding = StructEncoding::Tagged;
    machineDiagnostics.name = "tether.machine.cia402.MachineDiagnosticsV1";
    machineDiagnostics.description =
        "EtherCAT cyclic-loop, DC, mailbox, and link counters (optional "
        "diagnostics source; diagnostic only, not protective)";
    machineDiagnostics.fields = {
        {1, 0, u64Ref, "timestamp_us", "Sample timestamp"},
        {2, 0, u64Ref, "cycle_count", "Completed cyclic exchanges"},
        {3, 0, u64Ref, "missed_deadlines", "Skipped/overrun cyclic deadlines"},
        {4, 0, u32Ref, "max_cycle_work_us", "Longest in-cycle work burst"},
        {5, 0, u32Ref, "jitter_max_us", "Worst cyclic wake jitter"},
        {6, 0, u32Ref, "jitter_avg_us", "EWMA cyclic wake jitter"},
        {7, 0, u64Ref, "dc_sync_count", "DC sync frames emitted"},
        {8, 0, u64Ref, "dc_sync_errors", "DC sync failures"},
        {9, 0, u32Ref, "dc_jitter_max_us", "Worst DC-thread jitter"},
        {10, 0, u64Ref, "mbx_sends", "Mailbox/async datagram sends"},
        {11, 0, u64Ref, "mbx_send_errors", "Failed mailbox/async sends"},
        {12, 0, u64Ref, "mbx_collects", "Mailbox/async collects"},
        {13, 0, u64Ref, "mbx_collect_errors", "Failed mailbox/async collects"},
        {14, 0, u32Ref, "max_send_latency_ns", "Worst trigger→wire latency"},
        {15, 0, u32Ref, "tx_retries", "Link transmit retries"},
        {16, 0, u32Ref, "tx_failures", "Link transmit failures"},
        {17, 0, u32Ref, "rx_frames", "Link frames received"},
        {18, 0, u16Ref, "slaves_detected", "Slaves answering on the segment"},
        {19, 0, u16Ref, "slaves_operational", "Slaves currently in OP"},
        {20, 0, u32Ref, "last_wkc", "Working counter of the last exchange"},
    };

    // -- SDO inspection surface (optional; registered only when a pluggable
    //    IMachineSdoAccess backend is attached) ------------------------------

    SchemaNode sdoData;
    sdoData.key = schemaKey(MachineSchemaId::SdoDataBytes);
    sdoData.kind = SchemaKind::Bytes;
    sdoData.maxBytes = 512;
    sdoData.name = "tether.machine.cia402.SdoDataBytes";
    sdoData.description = "Raw SDO transfer payload";

    SchemaNode sdoEntry;
    sdoEntry.key = schemaKey(MachineSchemaId::SdoEntry);
    sdoEntry.kind = SchemaKind::Struct;
    sdoEntry.structEncoding = StructEncoding::Tagged;
    sdoEntry.name = "tether.machine.cia402.SdoEntryV1";
    sdoEntry.description = "One known object-dictionary entry";
    sdoEntry.fields = {
        {1, 0, u16Ref, "index", "Object dictionary index"},
        {2, 0, u8Ref, "subindex", "Object subindex"},
        {3, 0, u16Ref, "data_type", "CiA 301 data type code"},
        {4, 0, u8Ref, "access", "1 read-only, 3 read/write"},
        {5, 0, stringRef, "name", "Object name from the device profile"},
        {6, 0, eventDescriptionRef, "description", "Object description (ESI/device profile)"},
        {7, 0, f64Ref, "min_value", "Lower bound from the object table (NaN = unbounded)"},
        {8, 0, f64Ref, "max_value", "Upper bound from the object table (NaN = unbounded)"},
    };

    SchemaNode sdoEntryArray;
    sdoEntryArray.key = schemaKey(MachineSchemaId::SdoEntryArray);
    sdoEntryArray.kind = SchemaKind::DynamicArray;
    sdoEntryArray.name = "tether.machine.cia402.SdoEntryArrayV1";
    sdoEntryArray.element = ref(sdoEntry);
    sdoEntryArray.minCount = 0;
    sdoEntryArray.maxCount = 4096;

    SchemaNode sdoRequest;
    sdoRequest.key = schemaKey(MachineSchemaId::SdoRequest);
    sdoRequest.kind = SchemaKind::Struct;
    sdoRequest.structEncoding = StructEncoding::Tagged;
    sdoRequest.name = "tether.machine.cia402.SdoRequestV1";
    sdoRequest.description = "SDO upload request by explicit index";
    sdoRequest.fields = {
        {1, 0, u16Ref, "slave", "EtherCAT slave position"},
        {2, 0, u16Ref, "index", "Object dictionary index"},
        {3, 0, u8Ref, "subindex", "Object subindex"},
        {4, 0, u16Ref, "max_bytes", "Maximum bytes to return (1-512)"},
    };

    SchemaNode sdoWriteRequest;
    sdoWriteRequest.key = schemaKey(MachineSchemaId::SdoWriteRequest);
    sdoWriteRequest.kind = SchemaKind::Struct;
    sdoWriteRequest.structEncoding = StructEncoding::Tagged;
    sdoWriteRequest.name = "tether.machine.cia402.SdoWriteRequestV1";
    sdoWriteRequest.description = "SDO download request; requires technician role";
    sdoWriteRequest.fields = {
        {1, 0, u16Ref, "slave", "EtherCAT slave position"},
        {2, 0, u16Ref, "index", "Object dictionary index"},
        {3, 0, u8Ref, "subindex", "Object subindex"},
        {4, 0, ref(sdoData), "data", "Download payload, 1-512 bytes"},
        {5, 0, boolRef, "verify_readback", "Post-write verification read (default on)"},
    };

    SchemaNode sdoResult;
    sdoResult.key = schemaKey(MachineSchemaId::SdoResult);
    sdoResult.kind = SchemaKind::Struct;
    sdoResult.structEncoding = StructEncoding::Tagged;
    sdoResult.name = "tether.machine.cia402.SdoResultV1";
    sdoResult.description = "SDO transfer outcome with the slave's CoE abort code";
    sdoResult.fields = {
        {1, 0, boolRef, "ok", "Transfer completed"},
        {2, 0, u32Ref, "abort_code", "CoE SDO abort code, or zero"},
        {3, 0, ref(sdoData), "data", "Upload payload (reads only)"},
        {4, 0, eventDescriptionRef, "error", "Transport or adapter failure text"},
        {5, 0, eventDescriptionRef, "abort_name", "Decoded CoE abort-code name (writes/reads)"},
        {6, 0, ref(sdoData), "readback", "Post-write verification payload (writes only)"},
    };

    graph.nodes.push_back(std::move(alarmRecord));
    graph.nodes.push_back(std::move(alarmArray));
    graph.nodes.push_back(std::move(alarmPage));
    graph.nodes.push_back(std::move(operationArray));
    graph.nodes.push_back(std::move(captureStatus));
    graph.nodes.push_back(std::move(captureField));
    graph.nodes.push_back(std::move(captureFieldArray));
    graph.nodes.push_back(std::move(capturePayload));
    graph.nodes.push_back(std::move(captureExport));
    graph.nodes.push_back(std::move(configValue));
    graph.nodes.push_back(std::move(configEntry));
    graph.nodes.push_back(std::move(configEntryArray));
    graph.nodes.push_back(std::move(configStatus));
    graph.nodes.push_back(std::move(sdoData));
    graph.nodes.push_back(std::move(sdoEntry));
    graph.nodes.push_back(std::move(sdoEntryArray));
    graph.nodes.push_back(std::move(sdoRequest));
    graph.nodes.push_back(std::move(sdoWriteRequest));
    // -- PDO map + supervisor surfaces (optional; registered only when the
    //    corresponding backends are attached) ---------------------------------

    SchemaNode pdoEntry;
    pdoEntry.key = schemaKey(MachineSchemaId::PdoEntry);
    pdoEntry.kind = SchemaKind::Struct;
    pdoEntry.structEncoding = StructEncoding::Tagged;
    pdoEntry.name = "tether.machine.cia402.PdoEntryV1";
    pdoEntry.description = "Logical placement of one enabled PDO mapping entry";
    pdoEntry.fields = {
        {1, 0, u16Ref, "slave_index", "EtherCAT slave position"},
        {2, 0, u16Ref, "pdo_index", "PDO mapping index"},
        {3, 0, u8Ref, "direction", "0 = RxPDO (outputs), 1 = TxPDO (inputs)"},
        {4, 0, u32Ref, "offset", "Byte offset from the base logical address"},
        {5, 0, u16Ref, "length", "Entry length in bytes"},
        {6, 0, u16Ref, "entry_index", "Index into the PDO mapping"},
    };

    SchemaNode pdoEntryArray;
    pdoEntryArray.key = schemaKey(MachineSchemaId::PdoEntryArray);
    pdoEntryArray.kind = SchemaKind::DynamicArray;
    pdoEntryArray.name = "tether.machine.cia402.PdoEntryArrayV1";
    pdoEntryArray.element = ref(pdoEntry);
    pdoEntryArray.minCount = 0;
    pdoEntryArray.maxCount = 4096;

    SchemaNode supervisorEntry;
    supervisorEntry.key = schemaKey(MachineSchemaId::SupervisorEntry);
    supervisorEntry.kind = SchemaKind::Struct;
    supervisorEntry.structEncoding = StructEncoding::Tagged;
    supervisorEntry.name = "tether.machine.cia402.SupervisorEntryV1";
    supervisorEntry.description =
        "SlaveSupervisor recovery state for one slave (diagnostic only — not a "
        "protective channel)";
    supervisorEntry.fields = {
        {1, 0, u16Ref, "slave_index", "EtherCAT slave position"},
        {2, 0, u8Ref, "state", "0 normal, 1 critical, 2 recovering, 3 recovered, 4 failed"},
        {3, 0, boolRef, "suspended", "PDO data is suspended for motion"},
        {4, 0, boolRef, "recovering", "Recovery pass in progress"},
        {5, 0, u16Ref, "attempt_count", "Recovery attempts for the current incident"},
    };

    SchemaNode supervisorEntryArray;
    supervisorEntryArray.key = schemaKey(MachineSchemaId::SupervisorEntryArray);
    supervisorEntryArray.kind = SchemaKind::DynamicArray;
    supervisorEntryArray.name = "tether.machine.cia402.SupervisorEntryArrayV1";
    supervisorEntryArray.element = ref(supervisorEntry);
    supervisorEntryArray.minCount = 0;
    supervisorEntryArray.maxCount = 1024;

    SchemaNode supervisorResult;
    supervisorResult.key = schemaKey(MachineSchemaId::SupervisorResult);
    supervisorResult.kind = SchemaKind::Struct;
    supervisorResult.structEncoding = StructEncoding::Tagged;
    supervisorResult.name = "tether.machine.cia402.SupervisorResultV1";
    supervisorResult.description = "Controlled recovery-retry outcome";
    supervisorResult.fields = {
        {1, 0, boolRef, "ok", "Retry was accepted"},
        {2, 0, u8Ref, "state", "Resulting slave recovery state"},
        {3, 0, eventDescriptionRef, "error", "Rejection reason"},
    };

    graph.nodes.push_back(std::move(sdoResult));
    graph.nodes.push_back(std::move(machineDiagnostics));
    graph.nodes.push_back(std::move(pdoEntry));
    graph.nodes.push_back(std::move(pdoEntryArray));
    graph.nodes.push_back(std::move(supervisorEntry));
    graph.nodes.push_back(std::move(supervisorEntryArray));
    graph.nodes.push_back(std::move(supervisorResult));

    // -- Configuration baselines/diff + commissioning checklist ---------------

    SchemaNode configExport;
    configExport.key = schemaKey(MachineSchemaId::ConfigExport);
    configExport.kind = SchemaKind::Struct;
    configExport.structEncoding = StructEncoding::Tagged;
    configExport.name = "tether.machine.cia402.ConfigExportV1";
    configExport.description =
        "Baseline export: live values of every writable parameter";
    configExport.fields = {
        {1, 0, u64Ref, "revision", "Configuration revision at export time"},
        {2, 0, ref(configEntryArray), "entries", "Parameter id/value pairs"},
    };

    SchemaNode configDiffEntry;
    configDiffEntry.key = schemaKey(MachineSchemaId::ConfigDiffEntry);
    configDiffEntry.kind = SchemaKind::Struct;
    configDiffEntry.structEncoding = StructEncoding::Tagged;
    configDiffEntry.name = "tether.machine.cia402.ConfigDiffEntryV1";
    configDiffEntry.description = "One baseline/current value mismatch";
    configDiffEntry.fields = {
        {1, 0, u64Ref, "entry_id", "Parameter id that differs"},
        {2, 0, ref(configValue), "expected", "Value in the submitted baseline"},
        {3, 0, ref(configValue), "actual", "Value currently live"},
    };

    SchemaNode configDiffEntryArray;
    configDiffEntryArray.key = schemaKey(MachineSchemaId::ConfigDiffEntryArray);
    configDiffEntryArray.kind = SchemaKind::DynamicArray;
    configDiffEntryArray.name = "tether.machine.cia402.ConfigDiffEntryArrayV1";
    configDiffEntryArray.element = ref(configDiffEntry);
    configDiffEntryArray.minCount = 0;
    configDiffEntryArray.maxCount = 256;

    SchemaNode configDiff;
    configDiff.key = schemaKey(MachineSchemaId::ConfigDiff);
    configDiff.kind = SchemaKind::Struct;
    configDiff.structEncoding = StructEncoding::Tagged;
    configDiff.name = "tether.machine.cia402.ConfigDiffV1";
    configDiff.description = "Baseline-vs-live comparison result";
    configDiff.fields = {
        {1, 0, u64Ref, "revision", "Live configuration revision"},
        {2, 0, u32Ref, "diff_count", "Entries that differ"},
        {3, 0, ref(configDiffEntryArray), "entries", "Mismatched parameters"},
    };

    SchemaNode checklistItem;
    checklistItem.key = schemaKey(MachineSchemaId::ChecklistItem);
    checklistItem.kind = SchemaKind::Struct;
    checklistItem.structEncoding = StructEncoding::Tagged;
    checklistItem.name = "tether.machine.cia402.ChecklistItemV1";
    checklistItem.description = "One commissioning checklist item with live verdict";
    checklistItem.fields = {
        {1, 0, u32Ref, "item_id", "Stable checklist item identifier"},
        {2, 0, stringRef, "label", "Human-readable requirement"},
        {3, 0, boolRef, "required", "Must pass for acceptance"},
        {4, 0, boolRef, "passed", "Current evaluated verdict"},
        {5, 0, eventDescriptionRef, "evidence", "Why the item passed/failed"},
    };

    SchemaNode checklistItemArray;
    checklistItemArray.key = schemaKey(MachineSchemaId::ChecklistItemArray);
    checklistItemArray.kind = SchemaKind::DynamicArray;
    checklistItemArray.name = "tether.machine.cia402.ChecklistItemArrayV1";
    checklistItemArray.element = ref(checklistItem);
    checklistItemArray.minCount = 0;
    checklistItemArray.maxCount = 256;

    SchemaNode checklistReport;
    checklistReport.key = schemaKey(MachineSchemaId::ChecklistReport);
    checklistReport.kind = SchemaKind::Struct;
    checklistReport.structEncoding = StructEncoding::Tagged;
    checklistReport.name = "tether.machine.cia402.ChecklistReportV1";
    checklistReport.description =
        "Commissioning acceptance report; generated only when every required "
        "item currently passes";
    checklistReport.fields = {
        {1, 0, u64Ref, "generated_us", "Report generation timestamp"},
        {2, 0, u64Ref, "revision", "Configuration revision at report time"},
        {3, 0, boolRef, "all_required_passed", "Every required item passed"},
        {4, 0, ref(checklistItemArray), "items", "Evaluated checklist items"},
    };

    graph.nodes.push_back(std::move(configExport));
    graph.nodes.push_back(std::move(configDiffEntry));
    graph.nodes.push_back(std::move(configDiffEntryArray));
    graph.nodes.push_back(std::move(configDiff));
    graph.nodes.push_back(std::move(checklistItem));
    graph.nodes.push_back(std::move(checklistItemArray));
    graph.nodes.push_back(std::move(checklistReport));

    // -- Application profile + recipes (extension layer) ---------------------

    SchemaNode appProfile;
    appProfile.key = schemaKey(MachineSchemaId::AppProfile);
    appProfile.kind = SchemaKind::Struct;
    appProfile.structEncoding = StructEncoding::Tagged;
    appProfile.name = "tether.machine.cia402.AppProfileV1";
    appProfile.description =
        "Application profile document: signed declarative UI/profile payload "
        "served verbatim; integrity is enforced at attach time";
    appProfile.fields = {
        {1, 0, stringRef, "name", "Profile identifier"},
        {2, 0, stringRef, "version", "Profile version string"},
        {3, 0, ref(sdoData), "document", "Opaque profile document bytes"},
        {4, 0, ref(sdoData), "signature", "Keyed BLAKE3 MAC of the document"},
    };

    SchemaNode recipeInfo;
    recipeInfo.key = schemaKey(MachineSchemaId::RecipeInfo);
    recipeInfo.kind = SchemaKind::Struct;
    recipeInfo.structEncoding = StructEncoding::Tagged;
    recipeInfo.name = "tether.machine.cia402.RecipeInfoV1";
    recipeInfo.description = "Named parameter set offered by machine.recipe.*";
    recipeInfo.fields = {
        {1, 0, stringRef, "name", "Stable recipe identifier"},
        {2, 0, stringRef, "description", "Human-readable purpose"},
        {3, 0, u32Ref, "entry_count", "Parameters staged when applied"},
    };

    SchemaNode recipeInfoArray;
    recipeInfoArray.key = schemaKey(MachineSchemaId::RecipeInfoArray);
    recipeInfoArray.kind = SchemaKind::DynamicArray;
    recipeInfoArray.name = "tether.machine.cia402.RecipeInfoArrayV1";
    recipeInfoArray.element = ref(recipeInfo);
    recipeInfoArray.minCount = 0;
    recipeInfoArray.maxCount = 256;

    graph.nodes.push_back(std::move(appProfile));
    graph.nodes.push_back(std::move(recipeInfo));
    graph.nodes.push_back(std::move(recipeInfoArray));

    // -- Operational metrics -------------------------------------------------

    SchemaNode metrics;
    metrics.key = schemaKey(MachineSchemaId::Metrics);
    metrics.kind = SchemaKind::Struct;
    metrics.structEncoding = StructEncoding::Tagged;
    metrics.name = "tether.machine.cia402.MetricsV1";
    metrics.description =
        "Process-local operational counters for monitoring; informational "
        "only, reset on restart, never safety-relevant";
    metrics.fields = {
        {1, 0, u64Ref, "commands_received", "CommandRequestV1 decode+validate attempts"},
        {2, 0, u64Ref, "commands_denied", "Commands rejected by the gate"},
        {3, 0, u64Ref, "commands_dispatched", "Commands accepted and dispatched"},
        {4, 0, u64Ref, "audits_written", "Audit events journaled"},
        {5, 0, u64Ref, "config_commits", "Configuration transactions committed"},
        {6, 0, u64Ref, "recipes_applied", "Recipes staged successfully"},
    };
    graph.nodes.push_back(std::move(metrics));
    resolveSchemaDigests(graph);
    return graph;
}

/// Manifest roots every machine.cia402.v1 endpoint must commit.
inline std::vector<SchemaManifestEntry> machineProfileManifest(const SchemaGraph& graph) {
    std::vector<SchemaManifestEntry> manifest;
    for (const auto& key : {driveSnapshotSchemaKey(8), schemaKey(MachineSchemaId::MachineDescriptor),
                            schemaKey(MachineSchemaId::MachineSnapshot), schemaKey(MachineSchemaId::EventPage),
                            schemaKey(MachineSchemaId::AuthoritySnapshot), schemaKey(MachineSchemaId::AuthorityResult),
                            schemaKey(MachineSchemaId::CommandRequest), schemaKey(MachineSchemaId::CommandReceipt),
                            schemaKey(MachineSchemaId::OperationSnapshot), schemaKey(MachineSchemaId::OperationArray),
                            schemaKey(MachineSchemaId::AlarmPage), schemaKey(MachineSchemaId::CaptureStatus),
                            schemaKey(MachineSchemaId::GenerationArray),
                            schemaKey(MachineSchemaId::ConfigEntryArray),
                            schemaKey(MachineSchemaId::ConfigStatus),
                            schemaKey(MachineSchemaId::SdoEntryArray),
                            schemaKey(MachineSchemaId::SdoRequest),
                            schemaKey(MachineSchemaId::SdoWriteRequest),
                            schemaKey(MachineSchemaId::SdoResult),
                            schemaKey(MachineSchemaId::CaptureExport),
                            schemaKey(MachineSchemaId::MachineDiagnostics),
                            schemaKey(MachineSchemaId::PdoEntryArray),
                            schemaKey(MachineSchemaId::SupervisorEntryArray),
                            schemaKey(MachineSchemaId::SupervisorResult),
                            schemaKey(MachineSchemaId::ConfigExport),
                            schemaKey(MachineSchemaId::ConfigDiff),
                            schemaKey(MachineSchemaId::ChecklistItemArray),
                            schemaKey(MachineSchemaId::ChecklistReport),
                            schemaKey(MachineSchemaId::AppProfile),
                            schemaKey(MachineSchemaId::RecipeInfoArray),
                            schemaKey(MachineSchemaId::Metrics)}) {
        const auto* node = graph.find(key);
        if (!node) return {};
        manifest.push_back({SchemaRef{node->key, computeSchemaDigest(*node)}, node->revision});
    }
    return manifest;
}

} // namespace tether::io::cia402
