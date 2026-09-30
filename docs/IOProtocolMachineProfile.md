# `machine.cia402.v1` IO Profile

This document defines the first typed application profile for a Tether IO
session that controls or observes a fleet of CiA 402 / DS402 drives.

The profile deliberately uses existing Tether IO primitives. It does not add a
new message type for every drive operation. A server advertises the profile
through `FeatureExchange`, exposes fixed typed values through the parameter and
signal catalogs, streams telemetry with `ConfigureStream`, and invokes
commands through `InvokeEx`.

This profile is an application contract. The generic IO protocol remains
usable by applications that do not implement it.

## Negotiation

A server implementing the profile advertises these features in its
`FeatureExchangeResp`:

| Name | Type | Meaning |
| --- | --- | --- |
| `profile.machine.cia402.v1` | `Bool` | The required profile surface is available. |
| `profile.machine.cia402.version` | `U32` | Highest compatible profile schema version. |
| `profile.machine.cia402.motion` | `Bool` | Motion commands are available. |
| `profile.machine.cia402.authority` | `Bool` | Control-authority leases are available. |
| `profile.machine.cia402.events` | `Bool` | Event cursor/query support is available. |
| `profile.machine.cia402.capture` | `Bool` | Server-side capture support is available. |

A client must check the feature set before enabling controls. A missing
optional feature means that the related view is hidden or read-only. A client
must reject a profile version it cannot decode rather than guessing field
meaning.

## Catalog conventions

Profile entries use stable names and IDs. IDs may change between connections;
the stable resource ID in the descriptor is the identity used by persisted UI
state and commands.

| Entry | Kind | Value | Purpose |
| --- | --- | --- | --- |
| `machine.descriptor` | Signal | `Struct` or `Binary` | Machine, topology, axes, groups, units, capabilities, and limits. |
| `machine.snapshot` | Signal | `Struct` or `Binary` | Coherent machine-wide state. |
| `drive.<stable-id>.snapshot` | Signal | `Struct` or `Binary` | Coherent state for one drive. |
| `machine.events.cursor` | Signal | `U64` | Highest event ID retained for this session/profile. |
| `machine.events.read` | Function | Typed args/return | Read events after a cursor with bounded pagination. |
| `machine.command` | Function | `CommandRequestV1` / `CommandReceiptV1` | Validate and dispatch a command. |
| `machine.operation.<id>.snapshot` | Signal | `OperationSnapshotV1` | Progress and result for a long-running command. |
| `machine.authority.snapshot` | Signal | `AuthoritySnapshotV1` | Current control owner and lease. |
| `machine.authority.acquire` | Function | Typed args/return | Acquire a scoped control lease. |
| `machine.authority.renew` | Function | Typed args/return | Renew a lease before expiry. |
| `machine.authority.release` | Function | Typed args/return | Release a lease. |

The server may expose additional profile entries. Their metadata must identify
the schema, version, stable resource, unit, and access policy.

## Fixed `DriveSnapshotV1` value

The C++ definition and codec are in
`include/tether/io/CiA402Profile.hpp`. The value is exactly 64 bytes and is
encoded little-endian. C++ object alignment is not part of the wire format.

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

The `StructDescriptor` returned for this value is named
`tether.machine.cia402.DriveSnapshotV1`. A client must validate the schema name,
version, total size, and field bounds before decoding it.

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

Version 1 uses append-free fixed layouts. A future incompatible layout gets a
new schema name/version; clients must not infer new meanings from reserved
bytes. New optional profile entries and feature flags are backward compatible.
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
