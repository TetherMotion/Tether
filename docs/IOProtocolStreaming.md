# Tether IO Protocol — Streaming and Data Flow

How values move: polled reads, output streams (polled and
ring-driven), input streams, snapshots, log subscriptions, datalog,
and threshold filtering. Message layouts are in
[IOProtocolMessageCatalog.md](IOProtocolMessageCatalog.md); the
producer-side helpers are in
[IOProtocolServer.md](IOProtocolServer.md#producer-helpers).

## The four data paths

```
producer ──► Registry entry ──► GetSignal/GetParam/Snapshot ──► client
                        │
                        ├──► output stream ──► StreamData ──► client
                        │        (poll interval or ring drain)
                        │
                        ├──► DatalogRecorder ──► sink (app-owned)
                        │
client ──► input stream ──► inputStreamDataFn (app-owned)
```

Signals are sampled, never pushed by the producer — `varReadFn` /
`readFn` run on session threads. `LatestSignal` gives the producer a
lock-free-ish way to publish a snapshot + row history without touching
the socket.

## Output streams

### Lifecycle

```
configureStream(entryIds, intervalMs, chunkSize, skipCount,
                triggerMode, triggerEntryId)
        → ConfigureAck{specId, layout[(id, schemaSlot, valueSize)]}
startStream()      → StreamData packets until stopStream()
stopStream()
```

One configured stream per session — `configureStream` replaces the
previous spec. `startStream`/`stopStream` are fire-and-forget (no
response frame); errors surface as `Error` messages.

### Trigger modes

| Mode | Meaning |
|---|---|
| `Time` (0) | Sample every `intervalMs` ms regardless of change |
| `OnChange` (1) | Emit when a sampled entry differs from its last sent value; `triggerEntryId` restricts the trigger to one entry (0 = any) |

`intervalMs` doubles as the minimum spacing for OnChange. `skipCount`
decimates — N samples dropped per emitted sample. `chunkSize` batches
rows per `StreamData` packet (1 = one row per packet).

Rows carry the sampler's `timestamp_us` and one field per configured
entry, in `ConfigureAck` order. Fixed entries are `valueSize` bytes;
`valueSize == 0` marks a varint-length-prefixed variable field.

### Ring-bound vs polled sampling

When every configured entry is covered by a `ServerConfig::ringSources`
source's schema, the session **drains the producer's ring** instead of
polling read functions:

| | Polled | Ring-bound |
|---|---|---|
| Data origin | `readFn`/`varReadFn` at interval | producer `produce()` rows |
| Row timestamp | sample time | producer timestamp |
| Every row? | No — samples alias | Yes — up to ring capacity; drops counted |
| Entry types | any | fixed-size scalars only |
| Field order | ConfigureAck order | source `setSchema` order → ack slots |

Binding silently falls back to polling when the source doesn't cover
the configured set or any entry is variable-length — verify binding
behavior by row rate/gap analysis, not by assuming the source was
picked up. `SpscRingStreamSource` is SPSC: exactly one producer
thread.

## Input streams

Client→server batch channel for high-rate setpoint/command feeds
(`examples/tether_io_input_stream.cpp`):

```cpp
auto st = client.createInputStream(schemaSlot, maxValueSize, maxBatch);
// st->streamId, st->success
client.inputStreamData(streamId, {v1, v2, …});   // schema-encoded values
client.closeInputStream(streamId);
```

- The `schemaSlot` must resolve in the committed epoch; the server's
  `inputStreamCreateFn` accepts or refuses it — refuse is a clean
  nonzero status, not a protocol error.
- Every value must fit `maxValueSize` and validate against the
  declared schema before `inputStreamDataFn` sees the batch — wire
  validation runs first, always.
- `maxBatch` caps values per `InputStreamData` message.
- Status bytes on both response messages follow the `0 = success`
  convention.

Ownership: `inputStreamDataFn` is an application callback — the
session validates and delivers; it does not store or dispatch values
itself.

## Snapshots

Atomic multi-entry reads — one `timestamp_us` covers every returned
value:

```cpp
auto [ts, params]  = *client.snapshotParams({id1, id2});   // {} = all
auto [ts2, sigs]   = *client.snapshotSignals({id3});
```

Use for coherent cross-entry reads (e.g. all axes' positions at one
instant) where a stream's per-row pacing or a per-entry get would
smear the picture. Values decode per entry via
`decodeValue(schemaSlot, …)`.

## Log subscription

```cpp
auto sub = client.subscribeLog(minSeverity, "component-filter",
                               "message-filter", "location-filter");
client.setLogDataCallback([](const ClientLogRecord& r) {
    // r.severity / r.component / r.message / r.location
});
client.unsubscribeLog(sub->subscriptionId);
```

- `min_severity` — record passes at `record.severity >= min`.
- Each filter is a substring `contains` match; empty disables it.
- Server side: `Server::publishLog` / `Session::publishLog` create
  records; `logFn` in `ServerConfig` taps internal session logs.
- `subscriptionId` is scoped to the session; subscriptions die with
  the connection.

## Datalogging

`DatalogRecorder` writes fixed-layout binary records —
`[u64 timestamp][field0][field1]…` — at a configured rate over a
chosen entry set:

```cpp
// Remote arm/disarm — config is wire-accessible:
client.configureDatalog("run-42", /*sampleRateHz*/ 1000,
                        /*enabled*/ true, /*entryIds*/ {});  // {} = all

// Application side — the session attaches a null sink, so output
// requires reconfiguring with an application-owned recorder/sink:
DatalogRecorder rec;
DatalogSinkFn sink = [](const uint8_t* rec, size_t n) { write(fd_, rec, n); };
rec.configure(cfg, readEntryFn, tsFn, sink);
rec.sampleOnce();          // drive at cfg.sampleRateHz
```

Key semantic split: **wire controls the recorder config, the
application owns the sink.** A client `configureDatalog` arms the
session-shared recorder whose sink is null — useful as a uniform
remote enable. To receive bytes, the embedding application attaches
its own `DatalogSinkFn` and drives `sampleOnce()` (or wraps the
recorder in `MachineCaptureService`). `DatalogState`:
`Idle/Recording/Stopped/Error`; counters reset on each `configure()`.

## Threshold filtering

A named rule set attached to the session's stream filter
(`ConfigureThresholdReq`):

```cpp
ThresholdConfig cfg;
cfg.name = "slowband";
cfg.isWhitelist = false;   // rules select frozen entries
cfg.rules.push_back({entryId, ThresholdType::Absolute, 0.5, "", {}});
cfg.rules.push_back({0,        ThresholdType::None,     0.0, "", {}}); // default
```

| `ThresholdType` | Semantics |
|---|---|
| `None` (0) | Always pass |
| `Absolute` (1) | Pass when `|new − last_sent| > threshold` |
| `Relative` (2) | Pass when `|new − last| / |last| > threshold` |
| `Custom` (3) | Named evaluator + config primitives — registered session-side, **not reachable through the public client/server API** |

Semantics:

- **Blacklist** (`isWhitelist=false`): a value that fails its rule's
  test repeats the last-sent field — dead-band compression; a rule
  `entryId==0` is the default for unlisted entries.
- **Whitelist** (`isWhitelist=true`): unmatched entries are frozen
  entirely.
- Rules are evaluated per field, per sample — a suppressed field
  keeps its previous wire value, so decoders see a constant, not a
  gap.

## Choosing a path

| Need | Use |
|---|---|
| Occasional read | `getSignal`/`getParam` |
| Coherent multi-entry read | `snapshotSignals`/`snapshotParams` |
| ≤ sample-rate telemetry, every row | ring-bound stream |
| Low-rate telemetry, changes only | polled stream + OnChange/threshold |
| High-rate setpoint feed in | input stream |
| Operator-visible events | log subscription |
| Binary capture for offline analysis | datalog |
