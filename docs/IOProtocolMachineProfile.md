# `machine.cia402.v1` IO Profile

This document defines the first typed application profile for a Tether IO
session that controls or observes a fleet of CiA 402 / DS402 drives.

The profile deliberately uses existing Tether IO primitives. It does not add a
new message type for every drive operation. A server exposes V6 schemas through
the negotiated catalog, publishes typed values through the parameter and signal
catalogs, streams telemetry with `ConfigureStream`, and invokes commands
through `InvokeEx`.

This profile is an application contract. The generic IO protocol remains
usable by applications that do not implement it.

## Negotiation

A client completes the V6 `ClientHello` / `ServerHello` /
`SchemaRequest` / `SchemaDefinition` / `SchemaCommit` bootstrap before
discovering this profile. Profile values and function arguments identify their
types by schema slot in the committed catalog epoch. A client must resolve that
slot to a manifest entry, validate its key, digest, revision, and schema shape,
and reject an unknown or stale epoch rather than guessing field meaning.

The profile name is `machine.cia402.v1`. Its availability is determined by the
presence of its advertised schemas and catalog entries, not by an application
message discriminator or a feature-exchange flag. Optional application
surfaces, such as motion, authority, and capture, are represented by the
catalog entries the server publishes; absent entries make the related view
unavailable or read-only. The simulator advertises read-only event history,
but not an active-alarm lifecycle or acknowledgement service.

## Catalog conventions

Profile entries use stable names and IDs. IDs may change between connections;
the stable resource ID in the descriptor is the identity used by persisted UI
state and commands.

| Entry | Kind | Value | Purpose |
| --- | --- | --- | --- |
| `machine.descriptor` | Signal | Negotiated V6 schema value | Machine, topology, axes, groups, units, capabilities, and limits. |
| `machine.snapshot` | Signal | Negotiated V6 schema value | Coherent machine-wide state. |
| `drive.<stable-id>.snapshot` | Signal | `DriveSnapshotV1` packed V6 schema | Coherent state for one drive. |
| `machine.events.cursor` | Signal | `U64` | Highest event ID retained for this session/profile. |
| `machine.events.read` | Function | Typed args/return | Read events after a cursor with bounded pagination. |
| `machine.command` | Function | `CommandRequestV1` / `CommandReceiptV1` | Validate and dispatch a command. |
| `machine.operation.<id>.snapshot` | Signal | `OperationSnapshotV1` | Progress and result for a long-running command. |
| `machine.authority.snapshot` | Signal | `AuthoritySnapshotV1` | Current control owner and lease. |
| `machine.authority.acquire` | Function | Typed args/return | Acquire a scoped control lease. |
| `machine.authority.renew` | Function | Typed args/return | Renew a lease before expiry. |
| `machine.authority.release` | Function | Typed args/return | Release a lease. |

The server may expose additional profile entries. Their metadata must identify
the schema, stable resource, unit, and access policy. The schema reference is
the key/digest selected through the catalog slot, not a raw layout claim.

## Packed `DriveSnapshotV1` schema

The C++ schema graph and codec are in `include/tether/io/CiA402Profile.hpp`.
`tether.machine.cia402.DriveSnapshotV1` is the root node of a dependency-closed
V6 graph: seven reusable scalar nodes (`U8`, `U16`, `U32`, `U64`, `I8`, `I16`,
and `I32`) followed by a 22-field `Struct` node with `Packed` encoding. The
server advertises the root's key, digest, and revision in its manifest and a
catalog entry refers to its epoch-bound slot.

Once the root schema is negotiated, its packed payload is exactly 64 bytes,
with fields concatenated in ascending field-key order as below. Fixed-width
fields are little-endian. C++ object alignment is not part of the value
encoding, and the name is schema metadata, not an application discriminator.

| Offset | Size | Field | Type | Meaning |
| ---: | ---: | --- | --- | --- |
| 0 | 8 | `timestamp_us` | U64 | Source timestamp in microseconds. |
| 8 | 8 | `state_generation` | U64 | Increments for safety-relevant state changes. |
| 16 | 2 | `slave_index` | U16 | EtherCAT slave position for diagnostics. |
| 18 | 2 | `al_status_code` | U16 | EtherCAT AL status code, zero when clear. |
| 20 | 2 | `status_word` | U16 | CiA 402 statusword (0x6041). |
| 22 | 2 | `control_word` | U16 | Controlword actually applied (0x6040). |
| 24 | 2 | `fault_code` | U16 | CiA 402 error code (0x603F). |
| 26 | 4 | `quality_flags` | U32 | `DriveQuality` bitmask. |
| 30 | 1 | `al_state` | U8 | EtherCAT AL state enum value. |
| 31 | 1 | `ds402_state` | U8 | CiA 402 state enum value. |
| 32 | 1 | `target_mode` | I8 | Requested mode of operation (0x6060). |
| 33 | 1 | `display_mode` | I8 | Displayed mode of operation (0x6061). |
| 34 | 4 | `target_position` | I32 | Target in drive-native units. |
| 38 | 4 | `demand_position` | I32 | Position demand in drive-native units. |
| 42 | 4 | `actual_position` | I32 | Actual position in drive-native units. |
| 46 | 4 | `following_error` | I32 | Demand minus actual position. |
| 50 | 4 | `target_velocity` | I32 | Target in drive-native units per second. |
| 54 | 4 | `actual_velocity` | I32 | Actual in drive-native units per second. |
| 58 | 2 | `target_torque` | I16 | Target in drive-native units. |
| 60 | 2 | `actual_torque` | I16 | Actual in drive-native units. |
| 62 | 1 | `homing_state` | U8 | Profile-defined homing state enum. |
| 63 | 1 | `reserved` | U8 | Must be zero in version 1. |

A client must resolve the schema slot in the payload's epoch and verify the
manifest digest plus the packed root shape, field keys, scalar dependencies,
and 64-byte fixed size before decoding it. It must reject a stale slot, an
unknown schema, or a payload that does not exactly satisfy that schema.

## Machine descriptor and aggregate snapshot

The V1 machine roots are `MachineDescriptorV1` and `MachineSnapshotV1`; their
schema graphs are built by `machineProfileSchemaGraph()` in
`include/tether/io/CiA402MachineProfile.hpp`. Both use tagged structs so new
optional fields can be introduced in a future compatible revision without
changing the packed drive payload.

`MachineDescriptorV1` contains `profile_version`, stable `machine_id`,
`display_name`, `timezone`, `unit_system`, and a bounded `axes` array (1–16
entries). Each axis describes its stable ID, display name, slave index, display
order, group ID, position/velocity units and scale factors, and whether
homing is configured. Position and velocity conversions are
`application_value = drive_native_value * scale`; a zero or non-finite scale
is invalid. The descriptor is static application configuration, not a live
read of EtherCAT state.

`MachineSnapshotV1` is a coherent aggregate containing source timestamp,
state generation, an explicit simulated flag, axis/enabled/fault/warning/stale
counts, aggregate AL state, expected/actual WKC, link state, and DC lock state.
It reports telemetry observations only; it does not declare a safety state or
machine readiness. The example `SimulatedCiA402Fleet` registers descriptor,
aggregate, per-drive, and event-cursor signals plus a typed event-read
function. It has no parameters or motion-control functions and is not
connected to an EtherCAT master or motion dispatcher.

## Bounded event history

`EventRecordV1` and `EventPageV1` are tagged roots in the same negotiated
machine-profile graph. The server exposes `machine.events.cursor` as the
latest retained `U64` cursor and `machine.events.read` as a read-only typed
function with `(after_cursor: U64, limit: U32)` arguments. The limit is bounded
to 1–100 events. Event IDs are strictly increasing within a journal lifetime;
records include source timestamp, state generation, event type, stable source
ID, severity (`0` info, `1` warning, `2` fault, `3` critical), code, and a
bounded description. Unknown event types remain ordinary strings and must be
preserved by clients.

The page reports `oldest_cursor`, `latest_cursor`, `next_cursor`, an explicit
`gap` flag, and ordered records. Reads are exclusive of `after_cursor`. If the
requested cursor predates retained history, `gap` is true and the page starts
with the oldest retained record; clients should show the gap rather than
silently presenting the result as complete. The bounded simulator journal is
read-only and produces events for simulated startup and drive state/fault/
warning transitions. It does not implement alarm acknowledgement, clearing,
or production retention policy.

### Quality flags

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `Stale` | The source value is older than the advertised freshness bound. |
| 1 | `Simulated` | The value comes from a simulation/emulator. |
| 2 | `Estimated` | The value is estimated rather than directly measured. |
| 3 | `EthercatDegraded` | EtherCAT health is degraded for this value. |
| 4 | `UnitConversionFailed` | Application-unit conversion was unavailable. |

Unknown quality bits must be preserved and displayed as an unknown quality
condition; they must not be treated as healthy.

## Command and lifecycle rules

The first implementation may expose the following as typed function arguments
and return values. The profile uses `InvokeEx` request IDs for transport
correlation and puts a separate command UUID in `CommandRequestV1` for
application idempotency.

- The server validates role, authority lease, target IDs, expected
  `state_generation`, interlocks, limits, and deadline before dispatch.
- A rejected command returns a successful function transport with a typed
  rejected `CommandReceiptV1`; rejection is not a malformed-packet error.
- A long-running command returns an operation ID. Progress and terminal result
  are read from its operation snapshot.
- Repeating a command UUID returns the original receipt and must not duplicate
  motion within the server's idempotency retention period.
- A deadline bounds the initiator's wait. `InvokeEx` does not currently cancel
  server work when that deadline expires. The operation snapshot remains the
  source of truth for eventual completion.
- Control authority is server-owned. A browser disconnect, lease expiry, or
  policy revocation must prevent new commands and must invoke the server's
  configured motion-stop behavior for active lease-based actions.

## Compatibility

`DriveSnapshotV1` is a specific packed schema revision. A future incompatible
layout requires a new negotiated schema identity and digest; clients must not
infer new meanings from the `reserved` byte. New optional profile entries are
backward compatible when their schemas and slots are independently negotiated.
Unknown catalog entries and unknown event codes must be ignored safely while
preserving their IDs for diagnostics.

Profile values are bounded by the generic IO limits. Servers must reject
oversized values, invalid enum values, invalid target counts, and malformed
function arguments before touching motion state.

## Required tests

A profile implementation must test:

- Exact little-endian round trips for every fixed field.
- Truncated, oversized, wrong-schema, and invalid-version values.
- Unknown quality bits and reserved-byte handling.
- Stale `state_generation` command rejection.
- Duplicate command UUID idempotency.
- Authority acquisition, renewal, expiry, conflict, and release.
- Event cursor pagination, retention gaps, and reconnect resynchronization.
- `InvokeEx` deadline expiry without claiming that server work was cancelled.
