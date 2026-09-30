# Tether Web Interface Plan

## Purpose

Turn the current Tether IO dashboard into a useful browser-based operator,
commissioning, service, and application interface for EtherCAT machines,
especially a coordinated series of CiA 402 / DS402 servos.

The target is not a generic signal viewer with nicer tabs. It is a
safety-aware machine console that makes the current state, allowed actions,
motion intent, faults, and evidence for a decision visible in one place.
Generic signal and parameter exploration remains valuable, but it becomes an
expert tool within the larger product rather than the primary interaction
model.

## Evidence And Current Position

### Tether today

The existing dashboard at `web/tether-io-dashboard/` provides a good technical
starting point:

- A Vite/TypeScript web-component client over the binary Tether IO WebSocket
  protocol.
- Discovery of parameters, signals, and functions through a registry.
- Selection of signals for a WebGPU oscilloscope, currently optimized for a
  short live window and up to 16 channels.
- Direct scalar parameter editing, read-once cards, basic filtering, and a
  dynamically discovered jog panel.
- The jog panel already uses an important safety pattern: continuous motion is
  lease based and stops server-side when the lease expires.
- `tether::io::Protocol` already defines much more than the browser consumes:
  metadata, logs, snapshots, feature exchange, catalog updates, datalogging,
  thresholds, structured values, streams, and correlated invocation.
- Tether already has useful backend building blocks: EtherCAT slave state and
  AL-status access in `tether::UI::MasterEntityProvider`, SDO access through
  `SdoRegisterReader`, `SlaveSupervisor` recovery, CiA 402 drive state and
  operating-mode support, homing, and multi-axis motion facilities.

The dashboard is still a generic IO cockpit. Its function list is descriptive,
not a safe command surface; it has no concept of a machine, servo fleet,
motion group, control authority, alarm lifecycle, command result timeline, or
commissioning session. Metadata is decoded only to be skipped, so units,
ranges, enum labels, danger level, and display hints do not reach the UI.

### ASX lessons

The ASX source review shows useful product capabilities beyond its
signal-and-parameter tab composition model:

- Persisted dashboards with drag/drop composition, grouping, editable
  presentation settings, and undo/redo.
- Multi-pane live plots with trace search, persisted scope state, measurements,
  FFT, math traces, thresholds, retention control, and configurable layouts.
- Save/load/autosave of connections, tabs, layouts, and log presets.
- Connection-aware plugins for dashboard, plots, IO devices, robots, scripts,
  gRPC, Tether IO, and replay.
- Robot replay with record/play controls, frame navigation, signal lists,
  history, export locations, snapshots, and reconnection hints.

Adopt the persistence, composition, analysis, replay, and plugin ideas.
Do not make a free-form tab canvas the main operating mode. A machine operator
should arrive at an immediately useful fleet overview, not an empty layout
that must be assembled before a fault or enabled drive can be understood.

### Critical design conclusion

There are two separate missing layers:

1. The dashboard needs to finish the browser support for existing generic IO
   protocol capabilities.
2. Generic parameter/signal/function discovery cannot be the authoritative
   contract for safe, coordinated servo control. Tether needs a versioned
   machine and drive domain contract with explicit capability, permission,
   state-generation, command, event, and audit semantics.

## Product Scope

### Primary users and applications

| Application | Primary user | What must be fast and obvious |
| --- | --- | --- |
| Production operation | Operator | Machine readiness, active mode/program, cycle state, motion ownership, stop path, active alarms, concise drive status |
| Multi-axis setup | Commissioning engineer | Topology, drive identity, AL/DS402 state, PDO mapping, units, limits, mode support, enable sequence, home sequence |
| Diagnostics and service | Technician | Fault chain, error history, state transitions, DC/WKC health, SDO reads, trace capture, evidence export, controlled recovery |
| Motion tuning | Controls engineer | Target/actual/error trends, gains and limits with units/ranges, controlled write/apply/revert, snapshots and comparison |
| Machine-specific application | OEM/application developer | A stable API to declare machine views, commands, recipes, visualizations, and rules without modifying the core dashboard |
| Remote observation | Supervisor/support | Read-only fleet view, alarm acknowledgement policy, shareable captures, audit trail, no accidental authority |

### Non-goals

- The browser is not a functional-safety controller and must never replace a
  hardware E-stop, STO, safety PLC, safety fieldbus, guards, or risk analysis.
- Browser timing must not be used for cyclic CSP/CSV/CST control. The native
  Tether cyclic loop owns real-time setpoints; browser commands request bounded
  server-side actions.
- A raw object-dictionary editor is not the normal operator UI.
- Tether should not clone ASX's desktop/plugin architecture wholesale. The
  browser needs a clear operator flow, a stable API, and a constrained extension
  model.

## Design Principles

1. **Model the machine, not just its scalar leaves.** Drives, axes, groups,
   safety zones, programs, and alarms are first-class resources.
2. **Server authority.** The server validates every command against live state,
   configuration, permissions, interlocks, ownership, and deadline. The UI
   merely explains whether and why it is allowed.
3. **Readability before configurability.** The first screen must show useful
   fleet state with no layout work. Personal and shared layouts are optional
   enhancements.
4. **Use progressive disclosure.** An operator sees state and safe commands;
   a technician can investigate; an authenticated expert can access guarded
   diagnostics.
5. **Preserve causal evidence.** A fault view must correlate AL state, DS402
   state, fault code, command history, samples, and relevant logs by time.
6. **No untyped magic names as the principal API.** Registry entries remain a
   compatibility and extension plane. Stable application features use typed,
   versioned descriptors and identifiers.
7. **Connection loss is visible and conservative.** Distinguish stale data,
   disconnected UI, lost command authority, stopped lease motion, and a stopped
   EtherCAT cyclic loop.
8. **Every value has context.** Present engineering unit, scale, source,
   quality, timestamp, update cadence, valid range, and access policy where it
   changes a decision.

## Target Information Architecture

Use persistent application navigation rather than a single catalog page:

| View | Default audience | Core content |
| --- | --- | --- |
| Overview | Operator | Machine state, safety/interlock summary, active alarm banner, fleet health, active job/recipe, motion owner, quick stop/acknowledge paths |
| Drives | Operator, technician | Scannable drive fleet table and per-drive state drawer |
| Motion | Operator | Axis/group jog, incremental move, homing, coordinated move/program controls, position view, ownership indicator |
| Trends | Technician, controls engineer | Live/persisted multitrace plots, trigger/capture, cursors, measurements, FFT/math as appropriate |
| Alarms and events | All roles | Active/history alarms, acknowledgements, command/state timeline, filtering and evidence export |
| Diagnostics | Technician | EtherCAT topology, AL status, DC/WKC/network health, PDO/SDO diagnostics, supervisor recovery state, logs |
| Commissioning | Engineer | Discover/identify, map/configure, mode/limits, homing setup, tuning, controlled write/apply/revert, configuration diff |
| Recipes and programs | Operator, engineer | Versioned named actions/programs, validation, dry-run, execution progress, result history |
| Explore | Expert | Existing generic catalog, raw stream, function discovery, structured value inspector, user-defined dashboard |
| Settings | Administrator | Users/roles, connection profiles, time/source settings, shared layouts, retention, audit/export policy |

The global chrome should always show connection state, machine name, active
role, data freshness, safety/interlock summary, current control owner, and an
alarm count. It must never hide a critical alarm behind the current page.

## CiA 402 Fleet Model

### Machine descriptor

Define a versioned `MachineDescriptor` supplied by the Tether application:

- Machine identity, software/build/configuration revision, timezone, unit
  system, and declared application profile.
- EtherCAT master information: interface, cycle time, DC status, expected and
  actual working counter, link state, cyclic-loop health.
- Servo inventory with stable UUID, EtherCAT position, vendor/product/revision,
  ESI name, axis name, display order, physical location, motion-group
  membership, and capability flags.
- Coordinate frames and engineering-unit conversions. Explicitly separate drive
  counts, motor units, and application units.
- Motion groups, kinematic model name/version, soft limits, workspace zones,
  synchronization constraints, and declared jog/homing/move capabilities.
- Declared safety and interlock status as observable server facts, not browser
  booleans. Include source, severity, timestamp, quality, and human-readable
  reason.

### Per-drive snapshot

Publish a coherent, timestamped `DriveSnapshot` at an advertised rate:

- Connectivity and EtherCAT AL state, AL status code/error flag, mailbox/SDO
  availability, slave-supervisor suspension and recovery attempt status.
- DS402 state, decoded statusword, controlword actually applied, fault code
  (0x603F), warning bits, error-history summary, quick-stop state, voltage and
  enable state.
- Requested and displayed mode of operation; supported modes; mismatch reason.
- Target, demand, and actual position/velocity/torque; following error;
  current/temperature/voltage signals when supplied by the drive.
- Homing method/state/progress/result, offsets, active reference validity, and
  limit/home switch state where available.
- Quality flags: stale, simulated, unavailable, estimated, out of range,
  degraded network, and unit-conversion failure.
- A `state_generation` monotonically incremented for safety-relevant changes.

### Drive fleet UI

Build the Drives view around a dense, sortable fleet table rather than cards.
Rows must surface at least: axis name, AL state, DS402 state, mode, enable,
fault/warning, target/actual position, following error, motion-group ownership,
data age, and recovery status. Color supplements text and icons; it is never
the only signal. A row opens a detail drawer with:

- DS402 state machine visualization and last transitions.
- Statusword/controlword decoded bit table with values and timestamps.
- Target/demand/actual/error strip charts.
- Mode-specific controls only when server capability/preconditions allow them.
- Fault code meaning, history, likely source, recommended checks, and capture
  link.
- Current PDO map summary and guarded link to an object-dictionary inspector.

### Supported control actions

Expose actions by application-declared capability, not by hard-coded drive
brand assumptions:

- Enable, disable, quick-stop request, fault-reset request, and recovery
  request.
- Requested mode change for PP, PV, PT, HM, IP, CSP, CSV, and CST only when
  configured and supported.
- Lease-based continuous jog, bounded incremental move, move-to-position, and
  group moves through the native trajectory layer.
- Homing preparation, start, cancel, progress, result review, and post-home
  validity confirmation.
- Controlled tuning/configuration changes with preview, validation, write,
  readback, commit, rollback, and audit event.
- Raw controlword and SDO writes only in a separately enabled expert session,
  with a clear risk banner, range/schema validation, rate limits, audit, and
  an application policy that may disable them entirely.

## Safety, Authorization, And Command Semantics

### Safety rules

1. Keep hardware safety independent of this interface.
2. The server evaluates all interlocks immediately before action dispatch and
   while a lease action is active.
3. A command always names its target resource IDs, request UUID, client
   deadline, expected `state_generation`, optional configuration revision, and
   intended action parameters.
4. Reject stale UI intent with a structured reason and current state; never
   silently retarget the command after a state change.
5. Group actions validate all members first, then either reject with per-member
   blockers or dispatch through one server-side coordinated transaction.
6. Continuous motion has a server-enforced lease and bounded rate/acceleration.
   Releasing the control, losing authority, expiring the lease, an interlock,
   or client disconnect stops the command according to the server policy.
7. Parameter edits that affect motion use a staged transaction and explicit
   readback. The UI must show that a value is pending, applied, rejected, or
   reverted.
8. Commands are idempotent by request UUID for a defined retention window;
   retries must return the original result rather than duplicate motion.
9. Every write, fault reset, recovery, ownership transfer, and recipe run is
   audited with actor, role, source IP/session, old/new values, result, and
   correlated capture/event IDs.

### Roles and ownership

Define application-configurable roles at minimum:

| Role | Typical rights |
| --- | --- |
| Observer | Read all permitted state, trends, alarms, and exports |
| Operator | Acquire motion ownership; approved enable/jog/home/recipe actions; acknowledge operator alarms |
| Technician | Operator rights plus diagnostics, capture, guarded recovery, and non-motion maintenance controls |
| Controls engineer | Technician rights plus staged tuning and commissioning writes |
| Administrator | User/role/policy/configuration management; no implicit right to bypass application safety policy |

Use an explicit, visible control-ownership lease scoped to a machine or motion
group. It needs acquire, renew, release, transfer, timeout, and forced-release
events. Read-only clients must see the owner and remaining lease time. The
server must decide whether a role can preempt an owner and record the reason.

### Human factors for hazardous actions

- Render unavailable actions disabled with the server-provided blocker reason.
- Require a hold-to-run interaction for continuous jog and make it keyboard
  accessible only while focus and ownership remain valid.
- Require a concise confirmation sheet for enable, fault reset, home,
  recovery, group motion, and configuration commit. Show affected axes,
  precondition summary, limits, and action consequence.
- Do not add a browser "E-stop" that implies safety certification. Label any
  software stop according to its real server-side action and scope.
- Never rely on red/green alone; include words, icons, and current state.

## Backend And Protocol Plan

### Protocol-first decision

Do **not** begin by adding a large set of `Machine*` message types. The
existing Tether IO primitives are sufficient for a first, useful CiA 402 web
profile:

- `FeatureExchange` advertises support and profile/schema versions.
- The catalog, metadata, `Struct`/`Array` values, and `DescribeStruct` describe
  known resources and their fixed semantic data types.
- `Get`, snapshots, and configured streams carry descriptor/state/telemetry
  values.
- `InvokeEx` gives a correlated, deadline-bounded request/response path for
  commands. A command UUID included in its typed argument supplies
  application-level idempotency across reconnects.
- Functions return typed receipts; a long-running action returns an operation
  ID, whose progress is exposed as a normal signal or snapshot field.
- `SubscribeLog`, datalogging, and thresholds support generic diagnostics and
  capture immediately.

Accordingly, the first deliverable is a **documented IO profile** named
`machine.cia402.v1`, plus C++ and TypeScript codecs/adapters for its fixed
types. It uses well-known catalog entries and functions, but it remains fully
negotiated through the existing `FeatureExchange` message. For example, a
server advertises:

| Feature | Type | Meaning |
| --- | --- | --- |
| `profile.machine.cia402.v1` | `Bool` | This session supplies the required profile surface |
| `profile.machine.cia402.version` | `U32` | Highest compatible profile schema version |
| `profile.machine.cia402.motion` | `Bool` | Motion command surface is present |
| `profile.machine.cia402.authority` | `Bool` | Control-authority lease surface is present |
| `profile.machine.cia402.events` | `Bool` | Event journal/query surface is present |
| `profile.machine.cia402.capture` | `Bool` | Server-side capture surface is present |

The profile defines stable names, metadata keys, binary layout/schema names,
units, lifecycle rules, and UI semantics. Those definitions are the contract;
the catalog is the delivery mechanism. A server that only supplies a subset
advertises only the corresponding features, and the UI omits or makes those
capabilities read-only.

#### Initial profile surface using existing primitives

| Profile member | Existing primitive | Fixed meaning and UI primitive |
| --- | --- | --- |
| `machine.descriptor` | Readable variable `Struct` or `Binary` signal | Versioned machine/topology/axis/group descriptor; initializes navigation, labels, units, and fleet ordering |
| `machine.snapshot` | Readable `Struct` signal plus stream/snapshot | Coherent aggregate machine state: source timestamp, topology revision, snapshot sequence, EtherCAT health, safety/interlock summary, and per-drive snapshots; drives Overview and fleet table |
| `drive.<stable-id>.snapshot` | Readable/streamable `Struct` signal | Coherent drive state: AL/DS402 state, mode, status/fault bits, target/demand/actual values, following error, quality, and `state_generation`; drives a row and detail drawer |
| `machine.events.cursor` and `machine.events.read` | Scalar signal plus typed function | Monotonic event cursor plus paged event journal; drives alarms/timeline with polling or on-change refresh initially |
| `machine.command` | `InvokeEx` function with typed request/receipt | Validates a named action against targets, authority, expected generation, bounds, and interlocks; drives command sheet/result timeline |
| `machine.operation.<id>.snapshot` | Signal or aggregate snapshot field | Long-running homing/move/recovery progress, outcome, timestamps, blockers, and audit ID; drives progress UI |
| `machine.authority.acquire`, `.renew`, `.release`, `.snapshot` | Typed functions plus signal | Server-owned control lease; drives the ownership banner and command enablement |
| `machine.config.*` | Typed functions and parameters | Stage/validate/diff/apply/readback/rollback of configuration; drives commissioning workflow |
| `machine.capture.*` | Typed functions plus current datalog messages | Capture profile/configuration/status/artifact metadata; drives trends and support bundles |

The descriptor and snapshots should be application-owned coherent copies, not
UI-side joins of many independently read scalar entries. The cyclic task
publishes snapshots through a bounded handoff; the IO session reads that copy.
High-rate plotting remains on individually streamed scalar signals so an
operator view does not carry unnecessary fleet payloads.

#### Fixed types to specify first

Define these in a shared C++ header and TypeScript codec module before adding
screens. They can be registered as `Struct` values with a schema name/version
in metadata, or as an explicitly versioned compact `Binary` codec where a
variable-length nested collection is impractical. Do not use ad-hoc JSON in
the real-time state path.

1. `MachineDescriptorV1`: machine identity, descriptor revision, axes, drive
  stable IDs, EtherCAT locations, group/kinematic membership, units,
  capabilities, limits, and profile feature flags.
2. `MachineSnapshotV1`: source time, `snapshot_sequence`, descriptor revision,
  cyclic/DC/WKC health, safety/interlock summary, authority summary, alarm
  summary, and references to per-drive state.
3. `DriveSnapshotV1`: AL state/code, DS402 state, decoded status flags,
  requested/displayed mode, target/demand/actual position/velocity/torque,
  following error, fault/warning data, homing state, quality flags, and
  `state_generation`.
4. `CommandRequestV1`: command UUID, command kind, target stable IDs,
  authority lease ID/epoch, expected state generation(s), deadline, bounded
  motion/configuration arguments, and optional confirmation context.
5. `CommandReceiptV1`: accepted/rejected/completed state, structured blocker
  list, operation ID, accepted time, outcome, current generations, audit ID,
  and a human-readable summary. A rejected business command is a valid typed
  receipt, not a transport failure.
6. `OperationSnapshotV1`: operation ID/kind, targets, lifecycle state,
  progress, cancellability, stop reason, result, and audit correlation.
7. `EventRecordV1`: monotonic event ID, source timestamp, severity, lifecycle,
  resource IDs, typed cause/code, actor, command/operation/audit references,
  and display text.
8. `AuthoritySnapshotV1`: scope, owner identity/role display name, lease ID,
  epoch, expiry, policy, and transfer/preemption state.
9. `ConfigurationChangeSetV1`: base revision, individual typed edits,
  validation results, apply/readback/rollback status, and provenance.

#### Metadata vocabulary to specify first

Document canonical string metadata keys so generic UI components are useful
before the profile-specific ones exist: `schema.name`, `schema.version`,
`unit`, `scale`, `offset`, `display.precision`, `display.category`,
`display.order`, `range.min`, `range.max`, `range.step`, `enum.labels`,
`access.role`, `access.danger`, `stream.max_hz`, `quality.source`, and
`resource.stable_id`. Define parsing, validation, fallback behavior, and
whether each key is advisory or server-enforced.

#### Protocol changes that are *not* needed for profile v1

- A new message ID for each drive operation, alarm, or configuration action.
- REST endpoints or JSON-RPC parallel to the Tether IO WebSocket for control.
- A browser-facing cyclic setpoint message.
- A new authentication message when the WebSocket upgrade already establishes
  an authenticated server session through TLS/mTLS, reverse-proxy identity, or
  a secure session cookie. Bind that identity to `Session` and enforce roles
  server-side.
- A generic cancellation message: define `machine.command.cancel` first and
  add a protocol primitive only if multiple profiles need universal cancellation.

#### Narrow protocol additions to evaluate after the vertical slice

Only add wire changes when the profile cannot provide the required behavior
without polling, ambiguity, or unbounded overhead:

1. **Stream sequence/overrun fields.** `StreamData` has timestamps and a stream
  spec but no explicit monotonically increasing sample sequence or producer
  drop counter. Add a negotiated v2 stream payload only if the UI must
  distinguish transport reconnect, producer overrun, and a large but valid
  sample-time gap.
2. **Typed push event journal.** The v1 cursor-plus-query contract works over
  existing functions/signals. Add subscribe/unsubscribe/event-data messages
  only when polling cannot meet alarm/event latency or retention requirements.
  The addition must include resume cursor, explicit overflow/gap record,
  filtering, bounded queues, and replay semantics.
3. **Generic operation cancellation.** Add correlated cancel request/response
  messages only after several profiles prove that a function-level cancel
  contract causes duplicated lifecycle semantics. Include idempotency and a
  clear distinction between cancellation accepted, cancellation completed,
  and an operation that was already terminal.
4. **Authenticated raw IO sessions.** Add a protocol authentication handshake
  only if Tether IO must be safely exposed over raw TCP/serial without an
  authenticated WebSocket/transport boundary. Keep credentials out of generic
  feature metadata and document replay protection and session binding.

### Required implementation and documentation before UI expansion

1. Add `docs/IOProtocolMachineProfile.md` describing the `machine.cia402.v1`
  catalog contract, feature keys, fixed type schemas, metadata vocabulary,
  state/command/event lifecycle, compatibility, and error/blocker rules.
2. Update `docs/IOProtocolWireFormat.md` with current feature exchange,
  metadata, struct, snapshot, stream, log, datalog, and `InvokeEx` behavior;
  explicitly state that an `InvokeEx` deadline is presently an initiator-side
  bound, not a server-side cancellation guarantee.
3. Add shared C++ codecs/types for the fixed profile values and matching strict
  TypeScript codecs, including size/version validation and unknown-field
  compatibility behavior.
4. Add an application adapter that publishes a simulated multi-axis CiA 402
  descriptor/snapshot/event/command surface through the existing `Registry`.
5. Extend `TetherIOClient` to negotiate features, preserve metadata, decode
  structs, send `InvokeEx`, and surface logs/snapshots/catalog changes.
6. Test the profile in C++ and TypeScript for malformed data, schema mismatch,
  stale generation rejection, duplicate command UUID, lease expiry, event
  cursor gap, and reconnect/resynchronization.

### Contract strategy

Retain the binary Tether IO WebSocket transport. Complete support for its
existing generic messages, then deliver `machine.cia402.v1` as the negotiated
profile defined above. Do not create a second, competing REST or ad-hoc JSON
control plane for live operation.

The browser may use JSON for local layout/capture files, but all live machine
state and commands use the authenticated Tether IO session.

Use a small typed domain profile instead of encoding every action as a
stringly named registry function. The first implementation maps these
contracts onto catalog entries, structs, streams, and `InvokeEx`; it does not
require new message IDs:

| Capability | Required contract |
| --- | --- |
| Discovery | Machine descriptor, resource IDs, API version, feature flags, role and policy summary |
| Snapshot | Coherent machine/fleet/group/drive snapshots with monotonic sequence, source timestamp, data quality, and state generation |
| Subscription | Rate-limited delta/event subscription with snapshot-on-subscribe, sequence gap detection, resubscribe, and backpressure policy |
| Commands | Correlated command request/result/progress/cancel, deadline, idempotency key, target IDs, expected generation, structured blockers, audit ID |
| Authority | Acquire/renew/release/transfer ownership, lease state stream, conflict response |
| Alarms/events | Typed active/history/event records, acknowledgement and clear lifecycle, severity, cause links, retention/cursor pagination |
| Capture | Server-side trace/datalog configuration, trigger state, status, artifact metadata, download authorization |
| Configuration | Versioned staged change set, validate, diff, apply, readback, rollback, configuration revision and provenance |
| Diagnostics | Controlled SDO/object dictionary read/write descriptor, topology/PDO/DC/WKC state, recovery status, log subscription |

### New backend modules

Create an application-facing service boundary, for example
`tether::web::MachineService`, that is implemented by an application adapter.
It must depend on stable Tether interfaces rather than forcing the dashboard to
reach into a particular DS402 master implementation. Suggested components:

- `MachineDescriptorProvider`: static/configured topology and capabilities.
- `MachineStateProvider`: coherent read-only snapshots from EtherCAT, DS402,
  motion, safety, and application state.
- `MachineCommandService`: validated action dispatch, command progress, and
  cancellation to native Tether controllers.
- `ControlAuthorityService`: session/role ownership leases.
- `AlarmEventService`: typed alarm lifecycle, correlated logs, and audit.
- `CaptureService`: stream selection, trigger/capture, durable artifact store,
  replay index, and export policy.
- `ConfigurationService`: staged configuration and tuning transactions.
- `MachinePolicy`: role authorization, interlock policy, expert-mode policy,
  command limits, retention, and redaction.

Adapt existing `MasterEntityProvider`, `SdoRegisterReader`, `SlaveSupervisor`,
DS402 state/mode APIs, jog controller, homing facilities, and motion planner
behind these interfaces. Avoid putting browser-specific state in the cyclic
thread. Snapshot data into lock-free or bounded handoff structures; queue
commands to the appropriate non-real-time executor with explicit deadlines.

### Generic IO completion

The following is valuable even before `machine.fleet.v1` is deployed:

1. Decode and retain catalog metadata, value descriptors, enum/struct
   descriptors, units, ranges, scale, precision, access policy, display hints,
   and danger classification.
2. Implement `GetMetadata`, snapshot requests, `FeatureExchange`, and
   `CatalogChanged` handling in `TetherIOClient`.
3. Implement log subscribe/unsubscribe with severity/component/text filters and
   sequence/timestamp handling.
4. Implement datalog configure/status and threshold configuration with server
   validation feedback.
5. Add structured/array/enum/binary inspectors and editors where descriptors
   make the edit safe; remove the current F64-only parameter limitation.
6. Provide function invocation only from a typed form generated from parameter
   descriptors, with return-value display, timeout, error, confirmation, and
   role metadata. Never expose an opaque universal "run" button for hazardous
   functions.
7. Add request correlation, cancellation/deadline display, reconnect policy,
   sequence gap detection, and visible data freshness.

## Frontend Architecture Plan

### Foundation

Keep TypeScript and web components unless a later implementation spike proves
that component composition, state synchronization, or testing materially needs
a framework. First remove the single `main.ts` orchestration bottleneck:

- `transport/`: binary protocol, authentication/session, reconnect,
  subscriptions, correlation, feature negotiation.
- `domain/`: typed DTOs, units, value formatting, machine/drive command
  clients, permission and freshness helpers.
- `stores/`: connection/session, machine snapshot, authority, alarms, command
  history, layout, notification state. Every update preserves sequence and
  timestamp metadata.
- `views/`: overview, drives, motion, trends, alarms, diagnostics,
  commissioning, recipes, explore, settings.
- `components/`: dense table, status badge, command sheet, value editor,
  plot, event timeline, state-machine view, SDO inspector, capture picker.
- `extensions/`: constrained application-contributed manifests and widgets.

Do not let arbitrary extension JavaScript run with full dashboard authority.
Use a signed/trusted manifest, typed declarative cards and actions first. If
third-party executable widgets later become necessary, isolate them in a
sandboxed iframe/worker with a scoped capability token and explicit message
bridge.

### View behavior

#### Overview

- Render a zero-configuration machine summary immediately after descriptor and
  snapshot arrival.
- Show fleet counts by state, blocked/disabled/faulted drives, network health,
  active alarm list, motion owner, current activity, and last command result.
- Provide drill-through to the precise drive/group/alarm rather than a generic
  catalog search.

#### Motion

- Separate observation from authority acquisition.
- Show a coordinate frame and units at every move control.
- Support continuous lease jog, bounded step jog, and named safe positions.
- Display max rate/acceleration, soft limits, current position, command
  progress, and stop reason.
- Support group motion only through application-defined groups and trajectories;
  never assemble a group move by issuing browser-timed individual axis moves.
- Provide homing as a guided state machine with prerequisites, progress,
  cancellation, result, and post-home validation.

#### Trends and capture

- Support multiple synchronized plot panes with channel search, units and
  per-axis scale, cursor measurements, delta/derivative, trigger, freeze,
  decimation, retention, pause, and export.
- Add derived traces and FFT only after timestamp integrity and sample-rate
  metadata are correct. Mark derived results invalid when source data is stale,
  gapped, or non-uniform.
- Configure server-side recording for faults and expensive captures; browser
  ring buffers are for interaction, not the sole forensic record.
- Persist named trend layouts and capture profiles. Ship useful defaults for
  following error, target/actual position, velocity, torque, and EtherCAT/DC
  health.

#### Diagnostics and commissioning

- Provide topology view with slave order, identity mismatch, AL state/code,
  link/DC/WKC health, PDO bytes, and supervisor state.
- Provide a searchable object-dictionary/SDO inspector with typed decoding,
  abort-code interpretation, read rate guard, readonly default, and a
  policy-gated write flow.
- Present configuration as a versioned diff. An engineer can stage changes,
  validate against drive capabilities and interlocks, apply atomically where
  possible, read back, and roll back.
- Guide a commissioning checklist: discover, identify, map PDO, verify units,
  verify limits, configure modes, test enable, verify direction, home, test
  bounded motion, tune, save baseline, accept.

#### Alarms, events, and audit

- Use a persistent alarm banner and dedicated timeline, not browser toasts as
  the durable record.
- Show active, acknowledged, cleared, shelved (if allowed), and historical
  states with actor, timestamps, severity, source, causal links, and matching
  drive/group.
- Correlate command attempts, DS402/AL transitions, logs, captures, recovery,
  and configuration changes by IDs and time window.
- Support filtered export of a support bundle with configuration revision,
  machine descriptor, audit slice, events, logs, and selected captures. Apply
  policy-based redaction before export.

### Responsiveness and accessibility

- Optimize desktop first for service laptops and control-room screens, then
  provide a coherent tablet layout for observation and selected operator tasks.
- On narrow screens, preserve global status/alarm visibility and replace dense
  tables with drill-down rows; do not cram unsafe command controls into a
  mobile layout.
- Use keyboard navigation, focus restoration after dialogs, semantic labels,
  live regions for alarms/command results, contrast-compliant states, and
  reduced-motion support.
- Establish a browser support policy and graceful fallback when WebGPU is
  unavailable. A Canvas/WebGL/DOM plot fallback must retain diagnostic value.

## Persisted Workspaces And Application Profiles

Implement two levels of persistence:

1. **Application defaults:** versioned, source-controlled profile supplied by
   the machine application. It declares the default overview, fleet columns,
   motion groups, trend presets, diagnostics panels, recipes, and policy.
2. **User workspace overlays:** per-user layouts, filters, preferred units,
   saved trends, and non-safety visual preferences. An overlay cannot grant a
   capability or alter a server policy.

Each persisted item records schema version, profile/configuration revision,
creator, update time, access scope, and migration information. On descriptor
mismatch, retain the saved layout but mark unresolved resources and offer a
safe repair flow. Do not silently bind an old `axis_x` setting to a different
physical drive.

## Delivery Roadmap

The following work packages are ordered so every release leaves a usable,
testable product increment. A release gate is a behavior and safety decision,
not merely a collection of screens.

### Phase 0 - Product and safety contract

1. Name the first target machine class and its operating environment.
2. Write user stories for operator, technician, controls engineer, and remote
   observer during normal operation, drive fault, communication loss, homing,
   and commissioning.
3. Inventory the existing application-level controllers that can truthfully
   report machine, safety, and motion state.
4. Identify which state is safety related, safety relevant, diagnostic, or
   merely informational.
5. Define the server-side command authority boundary and all browser-invoked
   actions.
6. Document safe-state behavior for WebSocket loss, server restart, EtherCAT
   loss, control-owner expiry, and UI crash.
7. Decide the authentication identity source for deployed systems.
8. Define role permissions and approval/preemption policy.
9. Define audit retention, export, privacy, and redaction requirements.
10. Perform a formal safety review; obtain sign-off that UI features do not
    claim a safety function they do not implement.
11. Set data-rate, latency, capture-retention, and concurrent-viewer budgets.
12. Define acceptance criteria for the initial fleet: supported drives, axes,
    modes, homing, alarms, and diagnostic depth.

**Exit gate:** an approved command/safety matrix exists for every intended web
action, including the server-side preconditions and failure response.

### Phase 1 - Protocol and domain foundation

13. Add existing protocol capability negotiation to the TypeScript client.
14. Decode and expose all catalog metadata and value descriptors.
15. Add typed TypeScript representations for units, ranges, enum values,
    quality, timestamps, descriptors, and error envelopes.
16. Implement generic metadata, snapshot, log, datalog, threshold, and
    catalog-change protocol paths with unit tests.
17. Add correlation IDs, deadline propagation, request cancellation, and
    idempotent retry semantics to the browser client.
18. Implement reconnect with explicit stale/fresh transitions rather than
    silently reconnecting in the background.
19. Define and document the `machine.cia402.v1` profile mapping on existing
  entries, structs, streams, functions, and feature keys.
20. Add profile-schema, protocol-version, and feature-version compatibility
  tests.
21. Define stable resource identifiers independent of EtherCAT position.
22. Define descriptor, snapshot, delta, event, command, command-progress,
    authority, capture, and configuration DTOs.
23. Define sequence, timestamp, state-generation, event-cursor gap, and
  resynchronization rules using existing streams/functions.
24. Establish bounded message sizes, polling/subscription rates, per-session
  queues, and backpressure behavior; defer new message IDs until the
  vertical-slice measurements prove them necessary.

**Exit gate:** a test client can discover a simulated machine, subscribe to a
fleet snapshot stream, detect a gap, and resynchronize without raw registry
name knowledge.

### Phase 2 - Backend services and simulated fleet

25. Create the application-facing `MachineService` interfaces.
26. Implement a DS402 adapter that produces `DriveSnapshot` from the existing
    CiA 402 and EtherCAT state.
27. Adapt master/slave AL-state, AL-code, supervisor, DC, WKC, and link-health
    information into the machine snapshot.
28. Adapt application motion groups and coordinate transforms into the
    descriptor.
29. Adapt existing jog controller state and limits without duplicating its
    lease logic.
30. Add a server-side control-authority lease manager.
31. Add a command dispatcher that validates role, authority, state generation,
    mode, interlock, bounds, and deadline before queueing work.
32. Add command idempotency storage and structured results.
33. Emit command lifecycle events and immutable audit records.
34. Add typed alarm/event aggregation with DS402 and AL state transitions.
35. Add capture-session management backed by existing stream/datalog support.
36. Build a deterministic simulated multi-drive fixture with normal, stale,
    warning, fault, disabled, mode-mismatch, homing, recovery, and network-gap
    scenarios.

**Exit gate:** integration tests drive a simulated four-axis fleet through
discovery, authority, enable, bounded jog, stop, fault, reset/recovery, and
audit verification without a browser.

### Phase 3 - Read-only operator console

37. Split the frontend into transport, domain, state, view, and reusable
component modules.
38. Add an application shell with global connection, data-freshness, role,
authority, and alarm status.
39. Implement zero-configuration Overview using the machine descriptor.
40. Implement the dense sortable Drives fleet table.
41. Implement the per-drive drawer with DS402 state machine, decoded words,
    target/demand/actual/error values, units, timestamps, and quality.
42. Implement motion-group summary and ownership presentation.
43. Implement topology/network-health summary and deep link to a drive.
44. Implement alarm/event list with active and historical states.
45. Implement command/event timeline for a drive or group.
46. Implement explicit stale/gap/offline banners and recovery instructions.
47. Keep Explore as the generic catalog/stream view and add structured value
    inspection there.
48. Add basic per-user table-column, filter, and trend-preset persistence.

**Exit gate:** a technician can determine from one browser session which drive
is blocking production, whether the cause is AL/DS402/application state, when
it changed, and which evidence to inspect next.

### Phase 4 - Authority and controlled motion

49. Add authenticated session and role display to the live session.
50. Implement acquire, renew, release, expiry, conflict, transfer, and forced
    release flows for control ownership.
51. Add server-provided precondition/blocker display for every command.
52. Implement a command sheet showing targets, state, limits, interlocks,
    deadline, confirmation text, and outcome.
53. Connect the existing lease-based jog implementation to authority and the
    new command/audit pipeline.
54. Add bounded incremental move and move-to-named-position for one axis.
55. Add coordinated group move through the native planner, including an atomic
    preflight response identifying blocked members.
56. Add software stop actions with accurate scope/semantics and a clear
    distinction from hardware safety functions.
57. Add enable/disable, mode request, fault reset request, and guarded recovery
    commands where the application policy permits them.
58. Add guided homing with prerequisite checks, progress, timeout, cancel,
    result, and post-home validity state.
59. Implement client disconnect and ownership-expiry test cases for all motion
    actions.
60. Add keyboard, pointer, focus-loss, visibility-change, and repeat-rate tests
    for hold-to-run jog behavior.

**Exit gate:** no browser action can cause a motion command after its authority,
lease, state generation, or interlock validity has expired; the simulated
fleet tests prove this.

### Phase 5 - Trends, capture, and evidence

61. Upgrade the single scope into a reusable multi-pane trend workspace.
62. Add channel search by machine resource and semantic signal category.
63. Attach units, quality, timebase, decimation, and scale policy to every
    trace.
64. Add crosshair, cursor delta, statistics, freeze, zoom, and synchronized
    panes.
65. Add reusable following-error, position, velocity, torque, and network-health
    presets.
66. Add trigger rules and pre/post-trigger server capture profiles.
67. Implement capture status, cancellation, retention, download, and access
    control.
68. Add annotation of commands, alarms, state transitions, and configuration
    changes onto trends.
69. Add derived trace and FFT support after validating sampling assumptions.
70. Add support-bundle generation and policy-driven export/redaction.
71. Add capture playback/replay for recorded data, clearly labeled as replay.
72. Persist named trend/capture layouts with resource-ID migration checks.

**Exit gate:** a faulted axis can produce a single support artifact containing
its trend evidence, event timeline, logs, descriptor/configuration revision,
and audit slice.

### Phase 6 - Diagnostics and commissioning

73. Implement the detailed EtherCAT topology page.
74. Add DC synchronization, WKC, link, cyclic-loop, and mailbox diagnostics.
75. Add a typed SDO/object-dictionary reader with ESI/schema metadata and
    decoded abort errors.
76. Add a policy-gated expert SDO write workflow with preview, confirm,
    readback, rate limit, and audit.
77. Display PDO mapping and logical-address placement, including read/write
    direction and expected payload size.
78. Add slave-supervisor state, recovery history, and controlled retry action.
79. Implement versioned configuration baselines and comparison views.
80. Implement staged parameter/tuning changes with validation, apply, readback,
    commit, and rollback.
81. Add drive capability/mode/limit validation before commissioning writes.
82. Build the guided commissioning checklist and a signed/recorded acceptance
    report.
83. Add a service diagnostic capture template for common DS402 faults.
84. Add configuration import/export with schema validation, diff preview,
    authorization, and rollback plan.

**Exit gate:** an authorized technician can diagnose an AL or DS402 fault,
capture evidence, execute an approved recovery, and leave an auditable record
without terminal-only tools.

### Phase 7 - Application profile and extension layer

85. Define a declarative application-profile schema for resource naming,
    navigation, overview cards, allowed commands, trend presets, recipes,
    commissioning checklist, and custom diagnostic hints.
86. Version and validate profiles server-side at startup.
87. Let application code contribute typed adapters and declarative widgets.
88. Add named recipes/programs with input validation, dry-run capability,
    server-side execution, progress, stop policy, and result history.
89. Add optional machine visualization through a profile-provided kinematic
    scene, with actual/target overlays and no control implication by default.
90. Add external notification hooks for alarms with deduplication, escalation,
    acknowledgement policy, and audit.
91. Add a constrained custom-panel contract only after core views cover the
    identified application needs.
92. Define signing/trust/loading rules for application profiles and extensions.

**Exit gate:** a second machine application can deliver a useful operator
experience by providing an adapter and profile, without forking the dashboard
or relying on convention-only registry names.

### Phase 8 - Hardening, deployment, and release

93. Add C++ unit tests for every state-to-snapshot and command-precondition
    mapping.
94. Add protocol fuzz/property tests for malformed, oversized, reordered,
    duplicated, and gap-containing messages.
95. Add frontend unit tests for stores, unit conversion, command blockers,
    freshness, and descriptor rendering.
96. Add browser end-to-end tests against the simulated fleet for read-only,
    authority, motion, fault, capture, role, reconnect, and replay workflows.
97. Run hardware-in-the-loop tests with actual representative DS402 drives in a
    guarded test cell before enabling new write paths.
98. Load-test multiple observers, one controller, high-rate streams, long
    event history, and capture export without cyclic-loop regression.
99. Perform security testing for auth/session fixation, WebSocket origin/CSRF,
    command replay, authorization bypass, download path traversal, and audit
    tampering.
100. Perform accessibility and operational usability testing with target roles.
101. Add metrics for command latency/outcome, stream gaps, stale time, queue
    depth, browser render rate, capture failures, and UI error rate.
102. Package static assets with version/build metadata and cache-safe upgrades.
103. Document deployment for local-only, LAN, and reverse-proxy/TLS setups;
    forbid exposing an unauthenticated control endpoint to untrusted networks.
104. Provide migration, rollback, and feature-flag strategy per capability.
105. Publish an operator guide, commissioning guide, diagnostic guide, and
    security/safety boundary statement.

**Exit gate:** release readiness requires passed simulation, HIL, security,
performance, accessibility, and rollback criteria, plus safety-owner approval
for any newly enabled command.

## Prioritization

### First usable release

Deliver Phases 0 through 3, then the authority and lease portions of Phase 4.
This yields a real fleet console: immediate read-only observability, causal
diagnosis, controlled ownership, and safe bounded jog. It is more useful than
a large configurable dashboard because it answers the operational questions
that arise first.

### Next highest value

Deliver Phase 5 and Phase 6 next. Server-side captures and commissioning/
diagnostics turn the interface from a live display into a practical support
and engineering tool.

### Later expansion

Phase 7 is the scaling mechanism for varied motion applications. Do it only
after the core machine schema has survived two real applications; otherwise the
extension API will freeze the wrong abstractions.

## Measures Of Success

Track outcomes instead of screen count:

- Time for an operator to identify why a machine is not ready.
- Time for a technician to identify the faulting axis and relevant state
  transition.
- Percentage of commands rejected with an actionable structured blocker rather
  than an opaque transport/server error.
- Mean time from alarm to evidence bundle.
- Successful recovery/commissioning workflow completion without a terminal.
- Browser-induced command or stream failures, stale-data duration, and missed
  event rate.
- Cyclic-loop jitter and latency impact under expected dashboard load.
- Number of machine applications using the same typed fleet adapter/profile
  without dashboard forks.

## Immediate Next Actions

1. Select the first representative multi-axis CiA 402 machine and write the
   Phase 0 command/safety matrix with its controls owner.
2. Implement a small `machine.fleet.v1` protocol and simulated four-axis
   backend spike, including read-only snapshots and a deliberately rejected
   command with structured blockers.
3. Build only the Overview, fleet table, and drive detail drawer against that
   spike. Validate the data model and workflow with operators before styling or
   adding a layout builder.
4. In parallel, finish metadata/feature/log handling in the generic Tether IO
   browser client so current applications gain value immediately and the new
   domain UI has a robust transport foundation.