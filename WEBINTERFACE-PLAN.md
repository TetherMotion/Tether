# Tether Web Interface Plan

## Implementation Status

This document remains the target roadmap; it is **not complete**. Large parts
of the protocol foundation, backend services, and console UI are implemented;
the remaining work is concentrated in Phase 0 (safety contract), the
extension/profile trust layer, and Phase 8 hardening.

### Implemented

**Protocol and client foundation**

- Full V6 bootstrap in the TypeScript client: `ClientHello`/`ServerHello`,
  manifest-cache fast path (`schema-v6/cache`, keyed by connection URL,
  credentials stripped), schema-graph fetch-to-closure, epoch commit, digest
  verification, slot resolution, and stale-epoch rejection.
- Generic recursive schema-value decoding with field-restriction checks,
  retained catalog metadata, snapshots, datalog configuration/status,
  thresholds, correlated `InvokeEx`, and reconnect with explicit
  `stale`/`resynchronized` transitions.

**Backend (`tether::io::machine`, `MachineService.hpp`)**

- Versioned `machine.cia402.v1` schemas: `MachineDescriptorV1`,
  `MachineSnapshotV1`, `DriveSnapshotV1`, `CommandRequestV1`,
  `CommandReceiptV1`, `OperationSnapshotV1`, `EventRecordV1`,
  `AuthoritySnapshotV1`, `ConfigEntryV1` graphs.
- Fail-closed `MachineCommandGate`: role, authority token, state-generation,
  deadline, interlock, and idempotent request-UUID handling; audit IDs on
  receipts.
- Typed catalog functions: `machine.command`, `machine.command.cancel`,
  `machine.authority.{acquire,renew,release,takeover}` +
  `authority.snapshot`, `machine.alarms.{read,acknowledge,clear}`,
  `machine.capture.{configure,status,cancel,export}`,
  `machine.config.{export,diff,import,stage,validate,commit,rollback}`,
  `machine.sdo.{list,read,write}` (policy-gated, rate-limited, audited),
  `machine.supervisor.{status,retry}`, `machine.pdo.map`,
  `machine.recipe.{list,apply}`, `machine.checklist.{list,report}`,
  `machine.metrics`, and the bounded cursor-paginated event journal.
- Command actions include enable/disable, quick-stop, fault reset, mode
  change, jog start/renew/stop, step move, move-to-position, group move,
  recovery, configuration transactions, and home prepare/start/cancel.
- `EventJournal` supports a durable sink callback for audit persistence; the
  in-memory journal may evict while the sink remains the durable record.
- `DS402MachineAdapter` adapts real CiA 402/EtherCAT state alongside the
  deterministic `SimulatedMachineService` four-axis fixture.
- Authentication boundary: `StaticTokenAuthProvider` (constant-time bearer
  tokens from a `token<TAB>actor<TAB>role` file); the Drogon WebSocket
  controller maps `Authorization: Bearer` on the upgrade to
  `Session::setIdentity`. Unauthenticated sessions fail closed to observer.

**Frontend (`web/tether-io-dashboard`)**

- Modular layout: `transport/`/`protocol`, `domain/`, `stores/`, `views/`,
  `components/`, schema-v6, and scope modules replacing the `main.ts`
  monolith.
- Machine shell with connection state, access-token login, role, authority
  owner, data freshness, alarm count, and persistent safety notice.
- Views: zero-config Overview, dense Drives table + detail drawer, Motion
  (authority lease panel, hold-to-jog buttons, step/position moves, and a
  guided homing panel with per-axis Prepare → Start → verify/cancel over the
  `Home*` command actions), Trends (two synchronized panes, presets, freeze,
  derived channels, FFT, annotations, capture replay), Diagnostics
  (topology, PDO map, supervisor), Commissioning (capture config, SDO
  inspector, staged config), Recipes, event history, and Explore.
- Domain clients: `machine-control.ts` (authority + command + operation
  decoding), `machine-profile.ts`, `app-profile.ts`, `event-history.ts`,
  `analysis.ts` (capture decode, derived channels, FFT, redacted support
  bundle).
- Tests: vitest unit tests across client/domain/views plus an env-gated live
  e2e test (`TETHER_E2E_URL`) against `web_dashboard_example`.

### Not yet implemented / verified

- **Phase 0**: the command/safety matrix is drafted
  (`docs/CommandSafetyMatrix.md`); user stories, state safety
  classification, audit-retention requirements, budgets, and the formal
  safety review/sign-off remain open process work.
- **Transport hardening under load**: bounded frame sizes, per-session
  inbound queues, and rate clamps are now enforced; behavior under real load
  (load testing) remains a Phase 8 activity.
- **Audit-sink rollout**: `FileEventJournalSink` (append-only TSV, flushed per
  record) exists and `web_dashboard_example --audit-log` wires it to the
  event journal. Rotation/retention policy and a deployment default are not
  defined.
- **Extension trust layer**: complete — signed profiles with server-side
  MAC verification and document-contract validation
  (`docs/AppProfileTrust.md`), external alarm notification hooks
  (`IAlarmNotifier`), and the kinematic `scene` widget.
- **Phase 8 hardening**: HIL runs on real drives, security testing, load
  testing, formal accessibility/usability testing, and build-metadata
  packaging remain. Deployment and operator/commissioning/diagnostic guides
  are written (`docs/WebInterfaceDeployment.md`,
  `docs/WebInterfaceOperatorGuide.md`).

The browser control surface is not a functional-safety feature and must not
imply or replace E-stop or STO. Deployments must keep the authenticated
endpoint off untrusted networks until Phase 8 security review is complete.

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
- The V6 protocol now supplies a mandatory schema bootstrap, catalog epochs and
  slots, recursive typed values, streams, snapshots, logs, datalogging,
  thresholds, and correlated function invocation.
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

Do **not** add a large family of `Machine*` messages or a second JSON control
plane. V6 already provides the missing generic contract: every application
value is bound to a schema key, digest, and epoch-local catalog slot.

The browser must first complete `ClientHello`, `ServerHello`,
`SchemaRequest`, `SchemaDefinition`, and `SchemaCommit`. It then validates the
schema graph and uses catalog slots to decode parameters, signals, stream
rows, snapshots, and function arguments/returns. A stale epoch, unknown slot,
digest mismatch, or invalid payload is a visible data-integrity failure, never
a cue to guess a layout.

`machine.cia402.v1` is now documented in `docs/IOProtocolMachineProfile.md`.
Its availability is discovered from the negotiated catalog: required profile
entries and schema roots must be present. Optional surfaces are available only
when their typed catalog entries are present. This eliminates V5 feature flags,
`DescribeStruct`, raw `Binary` layouts, and per-type TypeScript wire codecs.

#### Initial profile surface

| Profile member | V6 representation | UI purpose |
| --- | --- | --- |
| `machine.descriptor` | Negotiated schema signal | Machine, topology, axes, groups, units, capabilities, and limits. |
| `machine.snapshot` | Negotiated schema signal/stream | Coherent machine state for Overview and fleet health. |
| `drive.<stable-id>.snapshot` | `DriveSnapshotV1` packed schema signal/stream | Drive row and detail drawer. |
| `machine.events.cursor`, `.read` | `U64` signal and typed function | Bounded event-history pagination. |
| `machine.command` | Typed function request/receipt | Validated, correlated command dispatch. |
| `machine.operation.<id>.snapshot` | Negotiated schema signal | Long-running command progress and outcome. |
| `machine.authority.*` | Typed functions and schema signal | Server-owned control lease. |
| `machine.config.*`, `machine.capture.*` | Typed functions, parameters, and signals | Commissioning and evidence workflows. |

The descriptor and snapshots are application-owned coherent copies, not
browser-side joins of scalar values. The cyclic task publishes them through a
bounded handoff. High-rate plots use individually streamed schema-backed
signals so an operator view does not carry unnecessary fleet payloads.

#### Profile schemas

Use V6 schema graphs for `MachineDescriptorV1`, `MachineSnapshotV1`,
`DriveSnapshotV1`, `CommandRequestV1`, `CommandReceiptV1`,
`OperationSnapshotV1`, `EventRecordV1`, `AuthoritySnapshotV1`, and
`ConfigurationChangeSetV1`. Their keys, digests, revisions, field keys,
restrictions, and optionality are the wire contract; names and metadata are
annotations only.

`DriveSnapshotV1` is already implemented as a negotiated 64-byte packed
schema in `CiA402Profile.hpp`. The TypeScript client should use one generic V6
schema-value decoder plus a profile adapter that maps validated field keys to
domain view models. Add a hand-written decoder only when a measured hot path
needs it, and keep its schema-root/digest verification in front of that path.

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
  authenticated WebSocket/transport boundary. Keep credentials out of catalog
  metadata and document replay protection and session binding.

### Required implementation before UI expansion

1. Keep `docs/IOProtocolMachineProfile.md` aligned with the published profile
   schemas and catalog entries.
2. Complete the TypeScript V6 bootstrap, manifest cache, schema-graph
   validation, slot resolution, and generic recursive value decoder.
3. Add a profile adapter that validates required `machine.cia402.v1` schemas
   and maps their stable field keys into UI domain values.
4. Add an application adapter that publishes a simulated multi-axis CiA 402
   descriptor/snapshot/event/command surface through the `Registry` and its
   installed `SchemaCatalog`.
5. Extend the browser client with metadata retention, functions, snapshots,
   logs, and catalog/schema-update resynchronization.
6. Test malformed bootstrap/schema/value data, stale epochs, digest mismatch,
   stale generation rejection, duplicate command UUID, lease expiry, event
   cursor gaps, and reconnect resynchronization.

### Contract strategy

Retain the binary Tether IO WebSocket transport. Complete V6 schema negotiation
and generic decoding, then deliver `machine.cia402.v1` as the schema-negotiated
profile defined above. Do not create a second, competing REST or ad-hoc JSON
control plane for live operation.

The browser may use JSON for local layout/capture files, but all live machine
state and commands use the authenticated Tether IO session.

Use a small typed domain profile instead of encoding every action as a
stringly named registry function. The first implementation maps these
contracts onto schema-backed catalog entries, streams, and typed functions; it
does not require new message IDs:

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

The browser foundation is now smaller and more reusable:

1. Cache V6 manifests and validated schema graphs by key/digest; invalidate
  epoch-bound slots on a schema update or reconnect.
2. Retain catalog metadata alongside the resolved schema reference, units,
  ranges, precision, access policy, display hints, and danger classification.
3. Generate generic viewers/editors from schema kinds and field restrictions;
  use `AllowedValues`, `LengthRange`, `NumericRange`, and enum/oneof keys
  rather than local layout assumptions.
4. Invoke functions from their schema-backed parameters and returns, with
  confirmation, timeout, error, and role metadata. Never expose an opaque
  universal "run" button for hazardous functions.
5. Implement logs, snapshots, datalogging, thresholds, correlation, visible
  freshness, reconnect, and catalog/schema-update resynchronization.

## Frontend Architecture Plan

### Foundation

Keep TypeScript and web components unless a later implementation spike proves
that component composition, state synchronization, or testing materially needs
a framework. First remove the single `main.ts` orchestration bottleneck:

- `transport/`: binary protocol, authentication/session, V6 bootstrap,
  manifest/schema cache, reconnect, subscriptions, and correlation.
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

> Numbering preserves the original plan — items already verified as
> implemented have been removed (see *Implementation Status* above), so the
> remaining item numbers are not contiguous. Tests and docs may still cite
> the original numbers.

### Phase 0 - Product and safety contract

Done: the first target machine class is the representative multi-axis CiA 402
fleet (currently `SimulatedCiA402Fleet`); the command authority boundary,
browser-invoked actions, role permissions, and safe-state behavior are drafted
in `docs/CommandSafetyMatrix.md` from the implemented gate; the deployed
identity source is the bearer-token file (`--auth-file`); initial transport
budgets are the bounds shipped under item 24. Remaining:

2. Write user stories for operator, technician, controls engineer, and remote
   observer during normal operation, drive fault, communication loss, homing,
   and commissioning.
3. Inventory the existing application-level controllers that can truthfully
   report machine, safety, and motion state.
4. Identify which state is safety related, safety relevant, diagnostic, or
   merely informational.
9. Define audit retention, export, privacy, and redaction requirements.
10. Perform a formal safety review; obtain sign-off that UI features do not
    claim a safety function they do not implement, and approve the
    `docs/CommandSafetyMatrix.md` draft.
11. Set data-rate, latency, capture-retention, and concurrent-viewer budgets.
12. Define acceptance criteria for the initial fleet: supported drives, axes,
    modes, homing, alarms, and diagnostic depth.

**Exit gate:** an approved command/safety matrix exists for every intended web
action, including the server-side preconditions and failure response.
A code-derived draft now exists in `docs/CommandSafetyMatrix.md` (per-action
roles, DS402-state preconditions, parameters, and OPEN decisions); it still
needs controls-owner review and the real-drive limits it flags.

### Phase 1 - Protocol and domain foundation — complete

Items 13–24 are implemented: V6 bootstrap, manifest cache, typed schema
representations, metadata/snapshot/datalog/threshold paths, correlation and
deadline propagation, `machine.command.cancel`, stale/resynchronized
reconnect, the `machine.cia402.v1` profile adapter, compatibility tests,
stable resource IDs, DTOs, cursor-gap/resync rules, exhaustive
restriction/encoding validation tests (`value-codec.test.ts` covers all seven
restriction kinds plus both struct encodings and varint/depth/canonicity
edge cases), shared cross-language digest vectors
(`test-fixtures/schema-digest-vectors.json` verified independently by the
TypeScript client and `test_io_schema.cpp`), and bounded transport behavior:
1 MiB frame limits on both sides, per-session inbound queue bounds with
fail-closed disconnect (256 messages / 4 MiB), bounded pending-request and
stream-entry counts, and clamped stream interval/chunk parameters.

**Exit gate:** a test client completes V6 negotiation, discovers a simulated
machine from the profile catalog, subscribes to schema-validated fleet state,
detects a gap, and resynchronizes without raw registry-layout knowledge.
(Met.)

### Phase 2 - Backend services and simulated fleet — complete

Items 25–36 are implemented: `MachineService`, `DS402MachineAdapter`,
machine/drive snapshots, the authority lease manager, the validating command
gate, idempotency storage, audit records, alarm aggregation, capture-session
management, and the deterministic `SimulatedMachineService` fixture.
Validation of `DS402MachineAdapter` against real drives is tracked under
Phase 8 item 97.

**Exit gate:** integration tests drive a simulated four-axis fleet through
discovery, authority, enable, bounded jog, stop, fault, reset/recovery, and
audit verification without a browser. (Met.)

### Phase 3 - Read-only operator console — complete

Items 37–48 are implemented: the modular frontend, machine shell with global
status, zero-config Overview, Drives fleet table and detail drawer,
motion-group/ownership display, topology summary, alarm/event views,
command/event timeline, stale/offline banners, Explore, and per-user
preference persistence.

**Exit gate:** a technician can determine from one browser session which drive
is blocking production, whether the cause is AL/DS402/application state, when
it changed, and which evidence to inspect next. (Met.)

### Phase 4 - Authority and controlled motion — complete

Items 49–60 are implemented: authenticated sessions with role display, the
full authority lease lifecycle (acquire/renew/release/takeover), server
blocker/precondition display, command dispatch through the audited gate,
lease-based hold-to-run jog, bounded step and move-to-position, group move,
scoped software stop, enable/disable/mode/fault-reset/recovery commands, the
guided homing UI (Prepare → Start → verify, per-axis cancel), and
disconnect/expiry plus hold-to-run test coverage (`SessionEnded*` /
`ExpiredLease*` / `CommandsWithReleasedToken*` C++ tests and the jsdom
`bindHoldToRunJog` suite covering pointer, keyboard-repeat, blur, and
visibility-loss paths).

**Exit gate:** no browser action can cause a motion command after its authority,
lease, state generation, or interlock validity has expired; the simulated
fleet tests prove this.

### Phase 5 - Trends, capture, and evidence — complete

Items 61–72 are implemented: the multi-pane trend workspace, channel search,
trace metadata, presets, server capture configure/status/cancel/export,
trend annotations, derived traces and FFT, the redacted support bundle,
capture replay, per-channel statistics, zoom/pan/reset, the hover crosshair
with per-channel readout, pinned A/B measurement cursors with Δt/Δv, and
versioned named-layout persistence that reports resource IDs that no longer
exist instead of silently dropping them.

**Exit gate:** a faulted axis can produce a single support artifact containing
its trend evidence, event timeline, logs, descriptor/configuration revision,
and audit slice.

### Phase 6 - Diagnostics and commissioning — complete against the simulator

Items 73–84 are implemented: topology view, supervisor status/retry, typed
SDO list/read and policy-gated write, PDO map display, configuration
export/diff/import plus stage/validate/commit/rollback, commissioning
checklist and acceptance report, and capture templates. Remaining work is
validation against real hardware (item 97) rather than new code.

**Exit gate:** an authorized technician can diagnose an AL or DS402 fault,
capture evidence, execute an approved recovery, and leave an auditable record
without terminal-only tools. (Met on the simulator; pending HIL.)

### Phase 7 - Application profile and extension layer — complete

Items 85–92 are implemented: `app-profile.ts` typed adapters,
`machine.recipe.{list,apply}`, the constrained widget/panel contract,
keyed-BLAKE3 signing with server-side MAC verification, document-contract
validation at attach time (`validateAppProfileDocument` in
`ApplicationProfile.hpp`, fail-closed without Glaze), the
signing/trust/loading rules in `docs/AppProfileTrust.md`, external alarm
notification hooks (`IAlarmNotifier` on `AlarmService` — inherent dedup,
severity floor, bounded escalation, journalled `alarm.notify.*` audit), and
the profile-provided kinematic `scene` widget — a read-only top-down view of
actual (filled) vs target (hollow) positions over 1–3 axis bindings, with no
control implication.

**Exit gate:** a second machine application can deliver a useful operator
experience by providing an adapter and profile, without forking the dashboard
or relying on convention-only registry names. (Met: profiles drive panels,
scenes, recipes, and control widgets through the signed-document surface.)

### Phase 8 - Hardening, deployment, and release

Items 93–96, 101, and 103–105 are implemented: C++ unit/fuzz coverage,
frontend unit tests, the env-gated live e2e test (`TETHER_E2E_URL`),
`machine.metrics`, deployment guidance for local/LAN/reverse-proxy-TLS with
secure defaults (`docs/WebInterfaceDeployment.md`), per-capability
schema-versioning/rollback/feature-flag strategy (same document §6), and the
operator/commissioning/diagnostic guide including the safety boundary
statement (`docs/WebInterfaceOperatorGuide.md`). Remaining:

97. Run hardware-in-the-loop tests with actual representative DS402 drives in a
    guarded test cell before enabling new write paths.
98. Load-test multiple observers, one controller, high-rate streams, long
    event history, and capture export without cyclic-loop regression.
99. Perform security testing for auth/session fixation, WebSocket origin/CSRF,
    command replay, authorization bypass, download path traversal, and audit
    tampering.
100. Perform accessibility and operational usability testing with target roles.
    (Smoke coverage exists — `views/a11y.test.ts` checks labelling and live
    regions under jsdom; a real assisted-technology pass remains.)

Item 102 is implemented: `vite.config.ts` stamps `version.json`
(`{version, gitSha, buildTime}`) into `dist/`, the same values are baked
into the bundle and shown on the Settings page (`build-info.ts`), and
`web_dashboard_example` serves hashed `assets/` as immutable with
`index.html`/`version.json` as `no-cache`.

**Exit gate:** release readiness requires passed simulation, HIL, security,
performance, accessibility, and rollback criteria, plus safety-owner approval
for any newly enabled command.

## Prioritization

### First usable release — scope already built

Phases 1–4 are implemented, yielding the real fleet console: read-only
observability, causal diagnosis, controlled ownership, safe bounded jog, and
guided homing. What stands between this and a deployable release is not more
features but Phase 0's safety contract and Phase 8's hardening gates (HIL,
security, load).

### Next highest value

Phases 1–6 are implemented, including the Phase 1 residual verification
gaps (14, 24). Complete Phase 0 so the command/safety matrix is signed off
before the write paths ship.

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
   Phase 0 command/safety matrix with its controls owner — this gates every
   remaining write path.
2. Phase 7 is complete — the kinematic `scene` widget (89) closed the last
   gap alongside profile validation (86), trust/loading rules (92), and
   alarm notification hooks (90).
3. Plan the Phase 8 hardening program (HIL cell, security review, load
   testing) against the now-working simulated fleet.