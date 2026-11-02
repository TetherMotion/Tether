# Tether IO Protocol — Client API

Complete guide to `TetherIOClient`
(`include/tether/io/TetherIOClient.hpp`), the reference V6 client. For
wire details see
[IOProtocolMessageCatalog.md](IOProtocolMessageCatalog.md) and
[IOProtocolWireFormat.md](IOProtocolWireFormat.md); for the matching
server side see [IOProtocolServer.md](IOProtocolServer.md).

Working end-to-end programs live in `examples/tether_io_*.cpp` — each
is a self-contained client+server pair.

## Construction and the V6 handshake

```cpp
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

auto transport = tether::io::tcpConnect("127.0.0.1", 42780, 2000);
tether::io::TetherIOClient client(
    std::make_unique<tether::io::SlipFramedTransport>(std::move(transport)),
    2000 /* default timeout ms */);
```

The constructor performs the whole V6 bootstrap before returning:
`ClientHello` → `ServerHello` (limits, epoch, manifest) →
`SchemaRequest`/`SchemaDefinition` for every uncached definition →
validation → `SchemaCommit`. On success the negotiated state is
immediately available:

```cpp
client.schemaEpoch();      // committed epoch (U64)
client.schemaManifest();   // advertised manifest entries
client.schemaGraph();      // resolved schema graph
client.resolveSchemaSlot(slot);        // SchemaNode* for a slot
client.decodeValue(slot, data, len);   // SchemaValue decode
```

On failure the constructor throws/returns an unusable client — check
`isConnected()` and treat constructor failure as fatal for the
session. A stale cached manifest is refreshed transparently through
`SchemaRequest`.

Any `ITransport` works — TCP needs the `SlipFramedTransport` wrapper
(byte stream → message framing), message-oriented transports
(WebSocket, in-process pipes) pass through directly. See
[IOProtocolTransports.md](IOProtocolTransports.md).

## Error model and timeouts

Every request/response method returns
`std::expected<T, ClientError>`:

```cpp
auto v = client.getParam(id);
if (!v) {
    // v.error().code — ErrorCode (Timeout, InvalidId, NotWritable, ...)
    // v.error().message — server-supplied or local description
}
```

`ClientError{code, message}` carries the server's `Error` frame code
when one was sent, or a locally generated code (`Timeout`,
`InvalidMessage`) for transport/decode failures. `defaultTimeoutMs`
bounds each request's wait; pass a per-call `timeoutMs` where the API
offers it.

## The receive pump

The client processes inbound traffic only while someone calls
`receiveMessage()` (or a synchronous `request()`, which pumps
reentrantly). This is the single most important runtime rule:

- **Asynchronous messages** — `StreamData`, `LogData`,
  `CatalogChanged`, `SchemaUpdate`, inbound `InvokeExReq` — are
  dispatched inside `receiveMessage()`.
- **Peer functions** — a client that hosts functions via
  `registerFunctions()` answers invokes only while pumped.
- **Synchronous calls** — `getParam`, `callFunction`, … pump internally
  until their response arrives, so async traffic can also surface
  *during* a blocking call.

A hosting client typically runs a dedicated pump thread:

```cpp
std::atomic<bool> on{true};
std::thread pump([&] {
    while (on) (void)client.receiveMessage(50);
});
// ... at shutdown:
on = false;
client.close();
pump.join();
```

`receiveMessage(timeoutMs)` waits up to the timeout for one message;
it returns a `std::expected<MessageView, ClientError>` — the usual
pattern ignores the result and relies on the registered callbacks.

## Callbacks

| Setter | Fires for | Context |
|---|---|---|
| `setStreamDataCallback(cb)` | each decoded `ClientStreamRow` | inside `receiveMessage`/`request` |
| `setLogDataCallback(cb)` | each `ClientLogRecord` | same |
| `setCatalogChangedCallback(cb)` | `CatalogChanged` push | same |
| `setSchemaUpdateCallback(cb)` | `SchemaUpdate` (new epoch) | same |
| `setAsyncMessageCallback(cb)` | catch-all for unhandled types | same |

Callbacks run on the pumping thread — keep them short or hand work
off. Re-issuing `request()` from inside a callback is legal
(recursive mutex) but beware head-of-line blocking: a callback's
round-trip blocks the pump for other async traffic.

## Catalogs

```cpp
auto params   = client.listParams(offset, maxCount);   // vector<ClientCatalogEntry>
auto signals  = client.listSignals(offset, maxCount);
auto funcs    = client.listFunctions(offset, maxCount);
```

`ClientCatalogEntry` carries `id`, `schemaEpoch`, `schemaSlot`,
`flags`, `name`, `description`, `group`. The epoch is checked against
the committed one — a mismatch rejects the whole page (a stale
catalog means the client must re-handshake or apply the pushed
`SchemaUpdate`). Page through large catalogs by incrementing
`offset` until `offset + page >= total`.

`ClientFunctionEntry` adds `parameters` (key, flags, defaults, schema
slot) and `returnSchemaSlot` for decoding return values.

`getMetadata(id)` returns the entry's metadata map (`{key, value}`
pairs) — the channel for units, ranges, access policy, stable ids.

## Parameters and signals

```cpp
auto value = client.getParam(id);          // vector<uint8_t>
client.setParam(id, &v, sizeof v);         // fixed-size write
client.setParamVar(id, data, len);         // variable-length write
auto sig = client.getSignal(id);           // latest signal value
```

Returned bytes are schema-encoded for schema-backed entries (decode
via `decodeValue(entry.schemaSlot, ...)`) or the flat `ValueType`
encoding otherwise (fixed scalars little-endian). Reads of
variable-length entries return their full payload.

Writes are validated server-side: size, schema, writability
(`NotWritable` on read-only params).

## Snapshots

```cpp
auto [tsUs, values] = *client.snapshotSignals({id1, id2});
auto all = client.snapshotParams();        // empty list = all params
```

One `timestamp_us` applies to every value in the response — the
server samples all requested entries under one timestamp, giving a
coherent cross-entry read. `ClientSnapshotValue` carries
`id/schemaEpoch/schemaSlot/valueSize/value` for decode-by-slot.

## Functions

```cpp
TetherIOClient::FunctionArg a;
a.position = 0;                 // positional (key = position + 1)
a.key = 0;                      // explicit key wins when nonzero
a.value = encodedArg;
auto res = client.callFunction(functionId, {a});
auto res2 = client.invokeEx(functionId, {a}, deadlineUs, timeoutMs);
```

- `callFunction` — serialized classic call.
- `invokeEx` — correlated call with a `requestId` and relative
  `deadlineUs`; `InvokeExResp` carries `success`, `errorCode`,
  `errorMessage`, `hasReturnValue`, `returnValue`. Deadline expiry is
  enforced client-side and does not cancel server work.
- `registerFunctions(entries)` — publish/replace the client's whole
  peer catalog (ids must be nonzero, signatures valid). The client
  must pump to answer invokes.
- `setInvokeHandler(cb)` — catch-all invoked for `InvokeExReq` ids not
  in the registered catalog:
  `ClientCallResult(uint64_t functionId, const vector<FunctionArg>&)`.

Full semantics, including peer dispatch, deadlines, and the monitored
invocation helper, are in
[IOProtocolFunctions.md](IOProtocolFunctions.md).

## Output streams

```cpp
auto cfg = client.configureStream(entryIds,
                                  /*intervalMs*/ 1,
                                  /*chunkSize*/ 1,
                                  /*skipCount*/ 0,
                                  /*triggerMode*/ 0 /*Time*/,
                                  /*triggerEntryId*/ 0);
// cfg->specId, cfg->layout (per-entry id/schemaSlot/valueSize)
client.setStreamDataCallback([](const ClientStreamRow& row) { /* … */ });
client.startStream();
// ... StreamData rows arrive while receiveMessage() is pumped
client.stopStream();
```

`configureStream` returns `{specId, layout}`; the layout is bound to
the response's schema epoch — reconfigure after epoch changes. See
[IOProtocolStreaming.md](IOProtocolStreaming.md) for ring-vs-poll
semantics, chunking, skip, and thresholds.

## Input streams

```cpp
auto st = client.createInputStream(schemaSlot, maxValueSize, maxBatch);
client.inputStreamData(st->streamId, {encoded1, encoded2, ...});
client.closeInputStream(st->streamId);
```

Client→server value batches; each value must schema-validate. The
server's `inputStreamCreateFn` may refuse the schema (`status != 0` on
the response — status byte 0 means success on this family).

## Log subscription

```cpp
auto sub = client.subscribeLog(minSeverity, componentFilter,
                               messageFilter, locationFilter);
client.setLogDataCallback([](const ClientLogRecord& r) { /* … */ });
client.unsubscribeLog(sub->subscriptionId);
```

Filters are substring matches (empty = off); severity is
`record >= minSeverity`. `sub->failed` plus `sub->errorMessage` report
server-side rejection.

## Datalog and thresholds

```cpp
client.configureDatalog("run-42", /*sampleRateHz*/ 1000,
                        /*enabled*/ true, /*entryIds*/ {});

// Threshold rules travel pre-encoded — build the byte block with
// ThresholdRule::encode (see the threshold example).
std::vector<uint8_t> rules(256);
BufWriter w(rules.data(), rules.size());
ThresholdRule{entryId, ThresholdType::Absolute, 0.5, "", {}}.encode(w);
rules.resize(w.pos);
client.configureThreshold("slowband", /*isWhitelist*/ false, rules);
```

`configureDatalog` arms/disarms the shared recorder remotely — the
record sink is application-owned and the status message
(`DatalogStatusReq/Resp` on the wire) is available through `request()`
for applications that need it; see
[IOProtocolStreaming.md](IOProtocolStreaming.md#datalogging). The
threshold config is a named rule set attached to the session's stream
filter; `Custom` evaluators are registered server-side and are not
reachable through the public client path.

## Raw access

- `sendRaw(data, len)` — send an arbitrary frame (advanced/debug).
- `request(payload, expectedRespType)` — synchronous request with
  response-type matching; used by every typed wrapper above.
- `receiveMessage(timeoutMs)` — pump/dispatch one inbound frame.

## Threading rules

- All public methods are serialized by an internal recursive mutex —
  calls from multiple threads are safe.
- `receiveMessage()` must not run concurrently with itself from two
  threads; use one pump thread or synchronous calls.
- Callbacks execute under the pump's stack — copy data out; don't
  hold the callback across slow work.
- `close()` is safe from any thread and unblocks pending waits.
- The object is movable only through `unique_ptr` — the transport is
  owned outright.
