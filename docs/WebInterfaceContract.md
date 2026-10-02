# Web Interface Product & Safety Contract (Phase 0)

This is the Phase 0 exit-gate document for `WEBINTERFACE-PLAN.md`. It names the
reference machine, fixes the command/safety matrix, records the identity and
authorization model, and sets the metrics baseline. Changes to hazard
classification require controls-owner sign-off.

## 1. Reference machine

| Property | Value |
|---|---|
| Machine class | Coordinated multi-axis CiA 402 / DS402 servo machine |
| Reference target | `sim-gantry` — the simulated four-axis gantry served by `examples/web_dashboard_example.cpp` (`SimulatedMachineStack`) |
| Control plane | `machine.cia402.v1` typed surface over TetherIO V6 |
| Bus | EtherCAT; per-axis DS402 slave. Functional safety (E-stop, STO, safety PLC) is **independent hardware** and is never represented or implied by this interface. |

All browser behavior is validated against `sim-gantry` first. A second real
machine is the Phase 7 exit-gate test of the adapter/profile model.

## 2. Users

| Role | Persona | Typical tasks |
|---|---|---|
| Observer | shift supervisor, remote viewer | status, trends, alarms, logs — no writes |
| Operator | machine operator | ack/clear alarms, enable/disable, fault reset, bounded jog and moves **with authority lease** |
| Technician | commissioning/service | staged config transactions, capture config, SDO writes |
| Controls engineer | controls development | tuning, recipes, all technician rights + control policy |
| Administrator | platform owner | auth/audit configuration, token management |

Roles map to `tether::io::Role` (`Observer` … `Administrator`). The server is
the sole authority: the browser mirrors state, it never decides it.

## 3. Command/safety matrix

Every mutation goes through `machine.command` (or the typed service functions)
and is re-checked by `MachineCommandGate` immediately before dispatch:
authority token, role eligibility, per-target state generations, configuration
revision, deadline, interlocks, bounds. Rejections return structured blockers.

| Action | Min role | Authority lease | Interlocks (server-checked) | Notes |
|---|---|---|---|---|
| `AcknowledgeAlarm`, `ClearAlarm` | Operator | no | none | lifecycle only; does not imply safe state |
| `Enable` | Operator | yes (scope covers target) | AL state OP, not faulted, fresh snapshot | DS402 transition |
| `Disable` | Operator | yes | none | always allowed — stop-class action |
| `QuickStop` | Operator | yes | none | always allowed — stop-class action |
| `FaultReset`, `Recover` | Operator | yes | faulted state | bounded retries |
| `SetMode` | Operator | yes | Switched-On, mode supported by drive | capability check (Phase 6 item 81) |
| `JogStart/JogRenew/JogStop` | Operator | yes | enabled, not stale | jog is a renewable operation; expiry or disconnect stops it |
| `StepMove`, `MoveToPosition`, `GroupMove` | Operator | yes | enabled, bounds, generation match | bounded move with deadline |
| `HomePrepare`, `HomeStart`, `HomeCancel` | Operator | yes | Switched-On, homing configured | `supports_homing` in AxisDescriptor |
| `CancelOperation` | Operator | partial | own/visible operations only | |
| `ConfigurationStage/Validate` | Operator | no | writable entries, type/size checks | nothing applied |
| `ConfigurationCommit` | Technician | no | validated txn only | bumps configuration revision |
| `ConfigurationRollback` | Operator | no | staged txn exists | |
| `machine.capture.configure` | Technician | no | fixed-size entries, rate bounds | records stay server-side |
| `machine.sdo.read/list` | Observer (authenticated) | no | none | mailbox op on session thread |
| `machine.sdo.write` | Technician | no | role + payload bounds; journaled | preview/readback UX (Phase 6 item 76) |

**Stop-class actions** (Disable, QuickStop, JogStop, HomeCancel, CancelOperation)
are never blocked by stale telemetry for execution — the gate validates the
lease and dispatches; blockers apply to start-class actions.

**Not a safety function**: the interface must never claim to prove STO, safe
position, or E-stop state, and must not infer safety readiness from ordinary
telemetry. Diagnostics views state this explicitly.

## 4. Identity and authorization decision

- **Session identity** is assigned at WebSocket upgrade by an
  `IMachineAuthProvider`. The shipped `StaticTokenAuthProvider` maps a token
  file (`token⇥actor⇥role`) and is an example-grade provider.
- **Transport**: `Authorization: Bearer` where possible; browsers use the
  `?token=` query parameter because the WebSocket API cannot set headers.
  Tokens are never logged and never persisted client-side (schema-cache keys
  strip the parameter).
- **Unauthenticated sessions are read-only observers.** Every mutation path
  fails closed.
- **Control authority** is a server-owned lease (`ControlAuthority`) with
  acquire/renew/release, scope conflict reporting, expiry, and forced release
  on disconnect. The UI cannot fabricate authority.
- **Audit**: every command produces an `AuditRecord`; `EventJournal` keeps a
  bounded in-memory history and `FileEventJournalSink` appends a durable TSV
  record. SDO writes are journaled the same way.
- **Open decision**: production identity (OIDC/mTLS/OS accounts) is deferred
  to Phase 7/8 hardening; the provider interface is the seam.

## 5. Metrics baseline (Measures of Success instrumented)

Tracked outcomes (plan §Measures of Success):

| Metric | Baseline mechanism |
|---|---|
| Command outcome quality | `CommandReceipt.blockers[]` — % rejections with structured blockers vs. transport errors |
| Alarm → evidence latency | event/alarm cursors + capture timestamps |
| Stale-data exposure | `stale`/`reconnecting`/`resynchronized` events; per-snapshot `data_age` |
| Cyclic-loop impact | SDO/mailbox ops run off the RT thread; load test in Phase 8 |
| Browser failures | stream gaps (cursor gaps), rejected pending requests, UI error toasts |

## 6. Release posture

- Local-only loopback by default; `--auth-file` required before LAN exposure.
- An unauthenticated control endpoint must not be exposed to untrusted
  networks (deployment doc, Phase 8 item 103).

## 7. Safety review record (Phase 8)

Reviewed against the command/safety matrix (§3) at feature freeze:

- [x] Browser controls cannot replace functional safety — the persistent
      shell banner states it; supervisor retry carries an explicit note that
      it does not act on safety interlocks; no surface claims safety rating.
- [x] Every mutating function enforces its role inside the callback
      (verified by tests: observer rejected on `config.import`,
      `recipe.apply`, `supervisor.retry`, `sdo.write`, command dispatch).
- [x] Bounded commands require a live authority lease + generation match;
      lease expiry/disconnect force-releases server-side.
- [x] Writes are journaled (`audit.<surface>` events) with actor identity;
      durable copy via `FileEventJournalSink` / `--audit-log`.
- [x] Recipe apply and config import *stage* transactions — validate/commit
      are separate audited steps; abandon leaves live config untouched.
- [x] Profile documents are integrity-protected (keyed BLAKE3 MAC verified
      at attach); the dashboard renders only the declared widget contract —
      no arbitrary code path.
- [x] Rate limits: SDO writes (50 ms), supervisor retry (1 s), capture
      export chunks (≤ 64 KiB paginated).
- [x] Malformed-input hardening: protocol fuzz (IOMalformed +
      `test_io_machine_fuzz`) and TS decoder fuzz; decoders fail closed.

Residual (documented, accepted): production identity is bearer-token file
or `?role=` dev mode only; OIDC/mTLS integration is a deployment seam via
`IMachineAuthProvider`. App-profile verifying key provisioning is the
operator's responsibility (`WebInterfaceDeployment.md` §4).
