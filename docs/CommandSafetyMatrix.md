# Command and Safety Matrix — Web Console (Draft)

**Status: DRAFT — requires controls-owner sign-off (WEBINTERFACE-PLAN Phase 0
exit gate).** This matrix is generated from the implemented enforcement
points in `MachineCommandGate`, `SimulatedStateSource`,
`SimulatedDispatcher`, and `MachineService`. It describes what the code
*enforces today* against the simulated fleet; items marked **OPEN** are
decisions the controls owner must make before this is a safety contract.

## 1. Scope and safety boundary

- Reference machine: `SimulatedCiA402Fleet` — a multi-axis CiA 402 fleet
  standing in for the first target machine class (a multi-axis CNC/printer
  gantry). Matrix semantics carry over to `DS402MachineAdapter`, but
  real-drive limits are **OPEN**.
- The web console is **not** a functional-safety feature. E-stop, STO, and
  hardware interlocks are independent of this surface. Fail-closed behavior
  (§5) is defense-in-depth, not a safety function.

## 2. Roles

| Role                | Level | Can acquire control authority |
|---------------------|-------|-------------------------------|
| Observer            | 0     | no                            |
| Operator            | 1     | yes                           |
| Technician          | 2     | yes                           |
| ControlsEngineer    | 3     | yes                           |
| Administrator       | 4     | **no** (authority-eligible set excludes it) |

## 3. Universal preconditions (`MachineCommandGate::submit`)

Every `machine.command` request must satisfy **all** of the following or it
is `Rejected` with explicit blockers and a journaled `AuditRecord`:

- `requestUuid` non-empty; replay within the retention window returns the
  cached receipt (idempotent) and is rejected if resubmitted by another
  session.
- Authenticated session (`Authorization: Bearer` at WS upgrade).
- Role authorized for the action (§4), unless `CommandPolicy::actionRoles`
  overrides.
- Audit sink available — dispatch is **blocked** if journaling fails.
- 1–128 targets, no duplicates, each with an `expectedGenerations` entry
  matching the live `stateGeneration`.
- `deadline` in the future.
- Valid authority lease token for the requested scope.
- Machine environment: EtherCAT link up and DC locked (else machine-level
  blockers).
- Per-target: resource exists, action is advertised for the current DS402
  state, and no resource blockers (stale snapshot, slave not in AL OP).
- Optional `configurationRevision` must match the live revision.

## 4. Per-action matrix

Minimum roles below are the `MachineCommandGate` defaults.

| Action                  | Min role          | Allowed DS402 state(s)          | Parameters (tag)              | Effect / bounds                                   |
|-------------------------|-------------------|---------------------------------|-------------------------------|---------------------------------------------------|
| Enable                  | Operator          | SwitchOnDisabled→SwitchedOn     | —                             | drive → OperationEnabled                          |
| Disable                 | Operator          | SwitchOnDisabled…OperationEnabled, QuickStop | —              | drive → SwitchOnDisabled                          |
| QuickStop               | Operator          | SwitchOnDisabled…OperationEnabled | —                             | drive → QuickStop                                 |
| FaultReset              | Operator          | Fault, QuickStop                | —                             | clears fault → SwitchOnDisabled                   |
| Recover                 | Technician        | Fault                           | —                             | drive → ReadyToSwitchOn                           |
| SetMode                 | Operator          | SwitchOnDisabled…OperationEnabled | mode (i8, tag 1)            | sets target/display mode                          |
| JogStart                | Operator          | OperationEnabled                | velocity (i32, tag 4)         | continuous jog; stays Running until JogStop/cancel/session end. **OPEN: velocity bounds — simulator accepts raw i32; real axes need per-axis velocity limit** |
| JogRenew                | Operator          | OperationEnabled                | —                             | renews jog lease marker                           |
| JogStop                 | Operator          | OperationEnabled                | —                             | velocity → 0, completes                           |
| StepMove                | Operator          | OperationEnabled                | delta (i32, tag 3)            | relative move. **OPEN: travel-limit validation**  |
| MoveToPosition          | Operator          | OperationEnabled                | position (i32, tag 2)         | absolute move. **OPEN: travel-limit validation**  |
| GroupMove               | Operator          | OperationEnabled                | —                             | **not implemented** in simulated dispatcher — returns Failed |
| HomePrepare             | Operator          | SwitchOnDisabled…OperationEnabled | —                             | homing prerequisite step                          |
| HomeStart               | Operator          | SwitchedOn, OperationEnabled    | —                             | runs homing; `homingState` → attained             |
| HomeCancel              | Operator          | OperationEnabled                | —                             | aborts homing, resets `homingState`               |
| CancelOperation         | Operator          | OperationEnabled                | —                             | **not implemented** in simulated dispatcher       |
| AcknowledgeAlarm        | Operator          | any                             | alarm id (u64, tag 6)         | ack via `AlarmService`                            |
| ConfigurationStage      | ControlsEngineer  | SwitchOnDisabled…SwitchedOn     | via `machine.config.*`        | stages txn                                        |
| ConfigurationValidate   | ControlsEngineer  | SwitchOnDisabled…SwitchedOn     | —                             | validates staged txn                              |
| ConfigurationCommit     | ControlsEngineer  | SwitchOnDisabled…SwitchedOn     | —                             | applies txn, bumps config revision                |
| ConfigurationRollback   | ControlsEngineer  | SwitchOnDisabled…SwitchedOn     | —                             | discards staged txn                               |

## 5. Write functions outside `machine.command`

These are role-gated at the function layer, journaled, and rate/bounds
limited, but do **not** require an authority lease:

| Function                        | Min role   | Constraints                                                        |
|---------------------------------|------------|--------------------------------------------------------------------|
| `machine.alarms.acknowledge`    | Operator   | u64 alarm id                                                       |
| `machine.alarms.clear`          | Operator   | u64 alarm id                                                       |
| `machine.sdo.write`             | Technician | 1–512 B payload; 1 write / 50 ms / server; journaled               |
| `machine.config.import`         | Technician | decodes entry array → staged txn; journaled                        |
| `machine.config.stage`          | Technician | ConfigEntryArray → staged txn                                      |
| `machine.config.commit`         | Technician | requires a *validated* txn; rechecks writability at apply          |
| `machine.recipe.apply`          | Technician | named recipe → staged txn (same commit path)                       |
| `machine.capture.configure`     | Technician | name ≤ bounded, rate Hz, entry ids                                 |
| `machine.capture.cancel`        | Technician | discards retained records                                          |
| `machine.capture.export`        | Technician | paginated bounded download                                         |
| `machine.supervisor.retry`      | Technician | 1 retry / s; triggers slave recovery                               |
| `machine.checklist.*`           | Technician | guided checklist functions                                         |
| `machine.command.cancel`        | (gate)     | cancel own operation, or Technician+ can cancel others             |

## 6. Fail-closed behavior

- **Disconnect**: session end releases every lease it held and cancels all
  its non-terminal operations (`cancelAllForSession`); in-flight jogs stop.
- **Lease expiry**: commands, renewals, and takeovers with stale tokens are
  rejected; the scope becomes reacquirable.
- **Stale state**: a stale drive snapshot or non-OP slave clears the whole
  supported-action set — no command can dispatch against it.
- **Audit failure**: missing or failing audit sink blocks dispatch entirely.
- **Transport**: oversized frames dropped, per-session queue bounds close
  the connection on overflow.
- **Generation mismatch**: any state change between read and dispatch
  rejects the command.

## 7. OPEN items for the controls owner

1. Per-axis velocity/position bounds — the simulated dispatcher accepts raw
   i32 values; real axes need rated limits before `JogStart`/`StepMove`/
   `MoveToPosition` ship to hardware.
2. `GroupMove` and `CancelOperation` are advertised only where the sim
   dispatcher returns Failed — implement or remove from the matrix.
3. Lease duration policy (current max 30 s) and takeover rules
   (Technician+/administrator override) — confirm per environment.
4. Whether `AcknowledgeAlarm` may also drive `AcknowledgeAlarm` via the
   gate vs. the function path — both exist today.
5. Audit-retention/redaction requirements (Phase 0 item 6) and the role
   policy sign-off (Phase 0 item 3).
