# Web Interface Operator Guide

Operator-facing guide to the TetherIO machine dashboard.

> The dashboard is **not** an E-stop, STO, safety PLC, or safety-rated
> control. Use the machine's independent hardware safety system for all
> functional-safety actions.

## Pages

| Page | Contents |
|------|----------|
| Overview | Machine snapshot, drive/operation states, alarm feed |
| Motion | Axis/operation detail, hold-to-run jog pad, command dispatch |
| Trends | Live scope panes, channel search/presets, stats/freeze, FFT, derived traces, annotations, capture replay |
| Diagnostics | EtherCAT loop/mailbox depth, slave topology chain, PDO map, supervisor state + controlled retry |
| Commissioning | SDO inspector (preview→confirm→readback), config transaction, capture with triggers, baseline diff/import, checklist + acceptance report |
| Recipes | Named configuration recipes — apply stages a transaction |
| Panels | Declarative widgets from the signed application profile |
| Events | Machine event/audit timeline |
| Settings | Authority lease, layouts, support-bundle export |

## Common workflows

### Jog (hold-to-run)

Motion → jog pad. Motion continues only while the button is held;
releasing (or focus loss) issues stop. Requires an operator+ authority
lease.

### Configuration change

1. Commissioning → Baseline → **Export** (saves `tether-baseline.json`).
2. Stage writes in the Configuration card (or apply a recipe).
3. **Validate** — fix any range/metadata failures.
4. **Commit** — takes effect and bumps the configuration revision.
5. To revert: Baseline → choose the exported file → **Import & stage**
   → validate → commit.

### SDO write (technician)

Commissioning → SDO inspector → select object → **Preview write**
shows current value, the new bytes, and the object's declared range →
**Confirm write**. The server rate-limits writes and does a readback
verification; the result shows the decoded CiA 301 abort name on
failure.

### Commissioning acceptance

Commissioning → Checklist → **Re-evaluate** for live verdicts, then
**Generate acceptance report**. The report is refused while any
required item fails — re-check the failing evidence first.

### Slave recovery (diagnostics)

Diagnostics → Supervision shows per-slave state/suspended/attempts.
**Retry** on a faulted slave triggers a controlled, audited recovery
attempt (rate-limited; server refuses duplicates in flight).

### Capture with trigger

Commissioning → Capture → enable the service preset or pick signals,
optionally arm a trigger (signal, above/below/|x| level, pre/post
record counts). **Export records** downloads a `.bin`; Trends →
**Replay capture** decodes it using the server-published record layout.

### Support bundle

Settings → **Download support bundle** — JSON snapshot for bug
reports. Credentials, tokens, IP addresses, and MACs are redacted
client-side.

## Notifications

Fault/critical journal events raise toast notifications. The event
timeline (Events page) is the authoritative history.
