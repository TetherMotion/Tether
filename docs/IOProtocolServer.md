# Tether IO Protocol — Server API

Server-side guide: the `Registry` catalog, `Server`/`Session`
lifecycle, producer helpers, and the configurable hooks. Client-side
usage is in [IOProtocolClient.md](IOProtocolClient.md); the wire
contract is in
[IOProtocolMessageCatalog.md](IOProtocolMessageCatalog.md) and
[IOProtocolWireFormat.md](IOProtocolWireFormat.md).

## Registry — the catalog

`Registry` is the single thread-safe catalog behind every session. It
holds three entry families:

```cpp
ParamEntry    // readable + optionally writable values
SignalEntry   // readable, read-only by convention
FunctionEntry // invocable entries (see IOProtocolFunctions.md)
```

### Entry fields

```cpp
ParamEntry p;
p.id            = entryId("pid.x.kp");  // stable, nonzero, unique
p.name          = "pid.x.kp";           // dotted, unique
p.description   = "Proportional gain";
p.group         = "pid.x";              // logical grouping
p.valueType     = ValueType::F64;       // flat wire type
p.readFn        = [](void* d){ /* fixed-size read */ };
p.writeFn       = nullptr;              // null → read-only
p.varReadFn     = nullptr;              // String/Binary/Struct reads
p.varWriteFn    = nullptr;
p.maxValueSize  = 0;                    // var-length cap (bytes)
p.metadata      = {{"units", "mm"}, {"range.min", "0"}};
p.extraFlags    = 0;                    // e.g. EntryFlags::NoStream
p.schema        = schemaRef;            // V6 schema ref (optional)
p.schemaSlot    = slot;                 // epoch-bound slot
```

Signals use the same shape minus `writeFn`/`varWriteFn`. Functions add
`parameters`, `returnValue`, and `callback`/`contextualCallback` —
see [IOProtocolFunctions.md](IOProtocolFunctions.md).

Callback shapes:

| Field | Signature | Use |
|---|---|---|
| `readFn` | `void(void* dest)` | fixed-size reads (`valueTypeSize` bytes) |
| `writeFn` | `void(const void* src)` | fixed-size writes |
| `varReadFn` | `size_t(void* dest, size_t maxLen)` | variable-length reads; return bytes written |
| `varWriteFn` | `void(const void* src, size_t len)` | variable-length writes |

`isVariableLength()` decides which pair the session uses: it is true
for `String/Binary/Enum/UVarint/IVarint/Struct/Array/Stream` — and
always for `ValueType::Struct`. Fixed-size entries must supply
`readFn`; variable-length entries must supply `varReadFn`.

### Ids and flags

`entryId(name)` gives the canonical id derivation — FNV-1a 64 over the
dotted name — so producers and clients agree on the id↔name mapping by
construction. Ids are still negotiated through the listing; the
convention keeps catalog code free of magic numbers.

`flags()` is computed, not stored: `Readable` always; `Writable` when
a write callback exists; `VariableLen` for variable entries; plus
`extraFlags` (`NoStream` excludes an entry from polled collect plans —
e.g. reads that cost a mailbox round-trip).

### Mutation and listeners

```cpp
registry.addParam(p);            // bool — false on duplicate/invalid
registry.addSignal(s);
registry.addFunction(f);
size_t h = registry.addChangeListener([] { /* catalog mutated */ });
registry.removeChangeListener(h);
```

Adds are mutex-protected and callable from any thread — entries may
be registered after `server.start()`; each active session pushes a
`CatalogChanged` message so clients re-list. Reads after publication
take a shared lock and stay cheap.

`paramCount()/signalCount()/functionCount()/entryCount()` report the
catalog size; `findParam/findSignal/find` give `EntryView` handles.

## Server and sessions

```cpp
Registry registry;
ServerConfig cfg;
cfg.timestampFn   = steadyClockUs;        // required µs clock
cfg.maxClients    = 8;
cfg.framing       = Framing::Slip;        // or Framing::None
cfg.schemaCatalog = &catalog;             // installed SchemaCatalog
cfg.ringSources   = { &ringSourceA };     // producer-driven streams
cfg.logFn         = [](const LogRecord& r){ /* server log sink */ };
cfg.inputStreamCreateFn = /* … */;
cfg.inputStreamDataFn   = /* … */;
cfg.encodedBufferFactory = /* custom RX buffers (advanced) */;
cfg.decodedBufferFactory = /* … */;

Server server(registry,
              std::make_unique<TcpTransportServer>(42780, "127.0.0.1"),
              cfg);
server.start();    // accept loop thread
// ... serve ...
server.stop();     // ends all sessions
```

One `Session` per accepted connection; each owns its negotiated
schema epoch, its configured stream, its log subscriptions, and its
peer-function catalog.

### ServerConfig knobs

| Field | Meaning |
|---|---|
| `maxClients` | Concurrent session cap (accept rejects beyond) |
| `timestampFn` | Required µs clock feeding every timestamp the wire carries |
| `logFn` | Receives server-internal `LogRecord`s |
| `framing` | `Slip` (byte transports) or `None` (message transports) |
| `ringSources` | `IRingStreamSource*` list for producer-driven streams — must outlive the Server |
| `schemaCatalog` | The installed catalog sessions negotiate against |
| `inputStreamCreateFn` | Decide whether a client's `CreateInputStreamReq` is accepted (schema slot + sizes in) |
| `inputStreamDataFn` | Receive each validated input-stream batch |
| `encoded/decodedBufferFactory` | Override per-session RX buffers (embedded sizing) |

### Server surface

```cpp
server.activeSessionCount();     // live sessions
server.peerFunctions();          // (name,id) across live sessions —
                                 // dead sessions excluded
server.callPeerFunction(name, args, deadlineUs, cb); // route to the
                                                     // hosting session
server.publishLog(severity, component, message, location);
```

`callPeerFunction` picks the healthiest hosting session (fewest
consecutive invoke timeouts; dead sessions skipped) — see
[IOProtocolFunctions.md](IOProtocolFunctions.md). `publishLog` fans a
record out to every session whose subscriptions match.

## Session internals (when you bypass Server)

A `Session` wraps one `ITransport` + `Registry` directly — used for
point-to-point links without an accept loop (serial, in-process pipes):

```cpp
Session session(std::move(transport), registry, steadyClockUs,
                /*logFn*/ nullptr, /*datalogRecorder*/ nullptr,
                /*inputStreamCreateFn*/ nullptr,
                /*inputStreamDataFn*/ nullptr,
                /*encBufFactory*/ nullptr, /*decBufFactory*/ nullptr,
                Framing::Slip, /*ringSources*/ nullptr,
                /*schemaCatalog*/ nullptr);
session.markRunning();                       // before a worker enters run()
std::thread t([&]{ session.run(); });        // blocking event loop
session.requestStop();
t.join();
```

`run()` owns the socket: deframing, dispatch, stream sampling, invoke
sweep. Public cross-thread surface: `callPeer(functionId, args,
deadlineUs, cb)` (reverse invoke),
`peerFunctions()` (`vector<FunctionDescriptor>` — the session's copy of
the client's catalog), `findPeerFunctionId(name)` (name→id lookup),
`publishLog(severity, component, message, location)`, `isRunning()`,
`requestStop()`.

## Producer helpers

Real-time code should never be called from session threads — the
helpers below decouple the producer from the socket.

### LatestSignal — pollable latest + optional ring

```cpp
LatestSignal<sizeof(Frame), 256> sig(/*initialLatestSize*/ sizeof(Frame));

// producer thread, at its own rate:
sig.produce(nowUs(), frameBytes, sizeof frameBytes);

// registry wiring:
SignalEntry e;
e.id = entryId("imu.frame");
e.name = "imu.frame"; e.valueType = ValueType::Struct;
e.maxValueSize = sizeof(Frame);
e.varReadFn = sig.varReadFn();            // polled path reads the
                                          // latest snapshot
registry.addSignal(e);

// ring path (optional):
sig.setSchema({e.id}, {sizeof(Frame)});   // row-field map
config.ringSources.push_back(&sig.source());
```

`produce()` updates the latest snapshot unconditionally and pushes a
`{tsUs, payload}` row to the ring when there's room; `dropped()`
counts ring overflow. Polled `getSignal`/stream sampling read the
snapshot — never the producer.

### SpscRingStreamSource — producer-driven stream rows

For rates where interval polling would alias (e.g. 1 kHz rows vs a
100 ms poll), a configured stream can *drain a ring* instead of
calling read functions:

```cpp
struct Row { uint64_t tsUs; uint64_t seq; double value; };
SpscRingStreamSource<Row, 1024> ring;
ring.setSchema({seqId, valId}, {8, 8});   // entryIds + field sizes
config.ringSources.push_back(&ring);

// producer:
ring.produce([](Row& r){ r.tsUs = now; r.seq = ++s; r.value = v; });
```

Binding rules (checked by `Session::bindRingSource`):

- Every configured stream entry must be covered by the source schema;
- each covered entry must be **fixed-size** and its `valueTypeSize`
  must equal the declared field size;
- a stream containing any variable-length entry never binds — it
  silently falls back to interval polling.

Wire rows are `[u64 producer_ts][field0][field1]…` — the timestamp
comes from the ring row, not the sampler. Plan for the quiet-fallback
rule: a misconfigured source looks like polling, only slower-looking
data.

## Read-path validation

When `schemaCatalog` is installed, the session validates catalog
values at the boundary: `GetParam`/`GetSignal`/stream samples run
`validateCatalogValue` — a value that doesn't satisfy its schema is
rejected rather than shipped. Struct/var-length entries therefore
must emit schema-encoded bytes from `varReadFn` (a
`nv::`-builder/`SchemaValueBuilder` encoding, not a raw struct
memcpy — unless the packed layout provably matches; see
[IOProtocolSchemaAuthoring.md](IOProtocolSchemaAuthoring.md)).

## Dynamic catalogs

`addParam`/`addSignal`/`addFunction` after start is supported: new
entries appear in the next `list*` page and every session receives
`CatalogChanged`. Removal changes ids/slots — bump the schema epoch
by reinstalling the `SchemaCatalog` so committed clients see the
update via `SchemaUpdate` rather than silently decoding against a
stale slot.

## Exposers

The `tether/io/exposers/` headers adapt existing modules (PID,
EtherCAT master, CiA402 drives, G-code, motion planner, simulation)
into registry entries — the table and authoring pattern are in
[IOProtocol.md](IOProtocol.md#exposer-pattern).
