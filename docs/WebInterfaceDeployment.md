# Web Interface Deployment & Rollback

Deployment, hardening, and rollback guidance for the TetherIO machine
web interface (`web_dashboard_example` / any host embedding
`MachineService` behind `TetherIOWebSocketController`).

> **Safety**: the browser interface is not an E-stop, STO, safety PLC,
> or safety-rated control. Functional safety remains on independent
> hardware (see `docs/WebInterfaceContract.md`).

## 1. Build

```bash
cmake -B build
cmake --build build --target web_dashboard_example -j8
cd web/tether-io-dashboard && npm ci && npm run build   # produces dist/
```

Serve `web/tether-io-dashboard/dist` as the static root. Any HTTP+WS
server embedding `TetherIOWebSocketController` works; the example
binary is the reference.

The build stamps each `dist/` with `version.json`
(`{version, gitSha, buildTime}`) and bakes the same values into the
bundle (Settings page shows the running build). Hashed `assets/` files
are served `immutable` while `index.html`/`version.json` are `no-cache`,
so upgrades are cache-safe; `web_dashboard_example` sets these headers
via a post-handling advice.

## 2. Run

```bash
web_dashboard_example \
    --port 8080 \
    --web-root web/tether-io-dashboard/dist \
    --auth-file /etc/tether/web-tokens.tsv \
    --audit-log /var/log/tether/machine-journal.log
```

| Flag | Purpose |
|------|---------|
| `--port`, `-p` | HTTP/WebSocket port (default 8080) |
| `--web-root`, `-w` | Dashboard static files |
| `--auth-file` | Bearer-token file; when set, `?role=` dev login is disabled and a real token is required on the WS upgrade |
| `--audit-log` | Append-only copy of the machine event/audit journal |
| `--verbose` | Per-frame protocol logging (debug builds / bring-up only) |

### Token file format

One record per line: `<token><TAB><actor><TAB><role>` where role is
`observer`, `operator`, `technician`, or `controls_engineer`. Tokens
are 16–256 chars and compared in constant time; they are never logged.
Generate them from a CSPRNG, e.g. `openssl rand -hex 32`.

Clients authenticate with `Authorization: Bearer <token>` semantics on
the WebSocket handshake (the dev `?role=` query parameter works only
when no `--auth-file` is configured).

## 3. Role model recap

| Role | Surface |
|------|---------|
| observer | All read signals; SDO list/read; config export/diff; checklist list; recipe list; capture status/export |
| operator | + authority acquire/renew/release; bounded command dispatch |
| technician | + SDO write; config stage/validate/commit/import; capture configure/cancel; supervisor retry; recipe apply; checklist report |
| controls_engineer | + authority takeover; parameter catalogue management |

Mutating functions re-check the role inside the callback; the
`access.role` field in the schema is descriptive, not the enforcement
point. Every mutation journals an `audit.<surface>` event.

## 4. Application profile signing

`machine.app.profile` carries a keyed-BLAKE3 MAC over the profile
document. The demo source (`StaticAppProfileSource`) generates a key at
startup — **never ship that**. For production:

- Provision a 32-byte verifying key out-of-band (HSM, sealed config,
  or OS keyring) and construct the profile source with that key.
- Re-sign the profile document on every change and bump `version`;
  the browser treats signature verification failure as "surface absent".

## 5. TLS / network placement

`web_dashboard_example` speaks plain HTTP+WS. Terminate TLS on a
reverse proxy (nginx, caddy) and forward `/tether-io` upgrades. Bind
the bare port to `127.0.0.1` or a machine VLAN; do not expose the
interface to untrusted networks — it is an engineering tool, not a
public endpoint.

## 6. Schema versioning & rollback

- The server advertises a V6 schema manifest (31 roots). Clients
  negotiate the full dependency graph once per schema epoch and cache
  it per-URL (`tether.schema.*` localStorage keys). A graph change
  bumps `schemaEpoch` and forces re-negotiation — there is no partial
  upgrade path; roll the whole profile forward or back together.
- **Configuration rollback**: Commissioning → Baseline → *Export*
  before any change window; to revert, *Import & stage* the baseline
  and run validate→commit. `machine.config.diff` shows the exact
  drift between baseline and live values.
- **Recipe apply** stages a transaction only; validate and commit are
  explicit steps, so a bad recipe can be discarded by abandoning the
  transaction.
- **Binary rollback**: keep the previous dashboard `dist/` and server
  binary; protocol V6 is additive-tolerant — older clients simply don't
  discover new optional surfaces.

## 7. Observability

`machine.metrics` exposes protocol/message/session/audit/config/recipe
counters (`IOServiceMetricsV1`). Poll it on a slow cadence or wire it
to your monitoring; the audit journal (`--audit-log`) is the forensic
record for every mutating call.

## 8. HIL / simulation testing

- Simulated fleet: `web_dashboard_example` runs the full simulated
  CiA 402 stack — exercises every surface without hardware.
- Slave-level HIL: use `slave_emulator` (EtherCAT slave emulator) behind
  a real `Master` + `EtherCatDiagnosticsAdapter` /
  `EtherCatSupervisionAdapter` to test recovery/diagnostics paths
  against real AL state transitions without physical drives.
- Fuzz coverage: `tests/io/test_io_machine_fuzz.cpp` and
  `src/domain/fuzz.test.ts` run per-CI.
- Real-wire e2e: `TETHER_E2E_URL=ws://host:port/tether-io npx vitest
  run src/e2e.test.ts`.
