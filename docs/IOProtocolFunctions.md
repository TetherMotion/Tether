# Tether IO Protocol — Function Invocation

Typed request/response calls in both directions: client→server
(`CallFunction`, `InvokeEx`) and server→client (peer functions,
`InvokeEx` only). Message layouts:
[IOProtocolMessageCatalog.md](IOProtocolMessageCatalog.md#function-messages).

## Signatures

A `FunctionEntry` declares a positional signature:

```cpp
FunctionEntry f;
f.id = 0x1234;                       // nonzero, unique
f.name = "filter.reset";
f.group = "filter";
f.description = "Reset filter state";
f.parameters = {
    {"alpha", "Blend factor", ValueType::F64},                  // required
    {"precision", "Digits", ValueType::U8, /*optional*/ true,
     /*hasDefault*/ true, /*defaultValue*/ {2}},                // optional
};
f.parameters[1].key = 2;             // optional explicit wire key
f.returnValue.present = true;
f.returnValue.name = "status";
f.returnValue.type = ValueType::String;
f.returnValue.schemaSlot = statusSlot;   // schema-backed return
f.callback = [](const std::vector<FunctionArgument>& args)
    -> FunctionCallResult { /* … */ };
registry.addFunction(f);
```

`validSignature()` rules:

- Nonzero `id`; every parameter has a name.
- **Positional calls**: once an `optional` parameter appears, every
  later parameter must also be optional.
- Optional parameters **must** carry a default (`hasDefault` +
  `defaultValue`) — omitted positions are materialized from the
  default locally by the serving side.
- `valueDescriptor`, when present, must match `type` and validate the
  declared default.

`contextualCallback` replaces `callback` when the implementation
needs the caller's context: `FunctionInvokeContext{identity,
requestId, deadlineUs}` — identity is the transport-authenticated
`SessionIdentity` (unauthenticated observer otherwise), and
`deadlineUs` is the caller's relative budget for this call.

## Argument encoding

Arguments travel as keyed values
`[key varint][length varint][value]`:

- `key = position + 1` by default; a descriptor's nonzero `key` wins.
- `value` is schema-encoded when the parameter is schema-backed,
  else its flat `ValueType` encoding.
- Server-side `invokeFunctionChecked` validates: duplicate/unknown
  keys, missing required positions, schema-invalid payloads — all
  rejected before user code runs. Omitted optionals receive their
  defaults.

Return values follow the same rules: a declared `returnValue` with a
schema must be returned schema-encoded.

## CallFunction vs InvokeEx

| | `CallFunctionReq/Resp` `0x37/0x38` | `InvokeExReq/Resp` `0x3E/0x3F` |
|---|---|---|
| Direction | client → server only | **both** directions |
| Correlation | serialized, one outstanding | `request_id` — pipelines freely |
| Deadline | none | `deadline_us` relative budget |
| Timeout behavior | caller waits for the response | initiator-side expiry, locally synthesized result |
| Server sees | callback args | callback + `FunctionInvokeContext` |

Use `CallFunction` for simple config-time calls; use `InvokeEx` for
anything latency-bounded, concurrent, or reverse-direction.

```cpp
// client → server
auto r = client.invokeEx(fnId, args, /*deadlineUs*/ 50'000,
                         /*timeoutMs*/ 0);
```

`deadlineUs == 0` disables the timeout. Expiry produces
`InvokeResult{timedOut=true, error=Timeout}` — **the callee is not
cancelled**; a late response is dropped. The callee still sees the
budget in `FunctionInvokeContext::deadlineUs` and may honor it.

## Peer functions (server → client)

A client publishes its own catalog once per connection:

```cpp
client.registerFunctions({workFn});   // replaces the whole catalog
```

Requirements: nonzero ids, `validSignature()` per entry. The catalog
is session-scoped — it dies with the connection.

Dispatch on the client happens inside `receiveMessage()` (or a
pumping `request()`): the registered entry's callback runs, else the
optional `setInvokeHandler` catch-all, else `Function not found` —
a hosting client must pump continuously.

```cpp
// Server side — by name, routed to the hosting session:
server.callPeerFunction("work", args, deadlineUs,
                        [](const Session::InvokeResult& r){ /* … */ });

// Or session-direct, by function id:
session.callPeer(functionId, args, deadlineUs, callback);
```

`callPeer`/`callPeerFunction` contract:

- Up to `MAX_PENDING_INVOKES` (256) outstanding per session; reverse
  calls pipeline and interleave with streams/log traffic.
- The callback fires **exactly once** — wire response (session
  worker thread), deadline expiry (sweep thread), or session end.
- `callPeer` can invoke ids never advertised (dynamic namespaces) —
  they reach the client's `setInvokeHandler` catch-all.
- `callPeerFunction(name, …)` picks the provider with the fewest
  consecutive invoke timeouts; dead sessions are skipped. A hung
  peer yields to a healthy duplicate; a disconnected provider's calls
  fail fast rather than waiting out the deadline.

`InvokeResult`: `requestId, functionId, success, timedOut, error,
errorMessage, hasReturnValue, returnType, returnValue, rttUs`.

## Monitoring an invocation — MonitoredRemoteFunctionInvocation

For control loops consuming a remote stage (the vesoma open-loop
remote-stage pattern), `MonitoredRemoteFunctionInvocation<Payload>`
(`MonitoredRemoteFunctionInvocation.hpp`) wraps an invoker in
single-flight bookkeeping:

```cpp
using Mon = MonitoredRemoteFunctionInvocation<Payload>;
Mon mon(
    /*invoke*/  [&](std::vector<uint8_t> args, uint64_t timeoutUs,
                    Session::InvokeResultFn cb) {
        return server.callPeerFunction("impedance", {}, timeoutUs,
                                       std::move(cb));
    },
    /*decode*/  [](const uint8_t* data, size_t len, Payload& p) {
        return decodeFrame(data, len, p);
    },
    /*maxAgeTicks*/ 20,
    /*timeoutUs*/    2'000'000);

// per loop tick:
mon.call(tickCount, argBytes);           // one call in flight max
if (mon.fresh(tickCount)) apply(mon.result().payload);
else                        applyFallback();
```

It enforces: at most one call in flight (`call()` returns `false`
while busy or when the invoker refuses), caller-stamped ticks carried
on each result, an inclusive `maxAgeTicks` freshness window
(`fresh(tick)` ⟺ `age <= maxAge`), decode-time error counting, and
counters — `issued()/completed()/timeouts()/errors()/inFlight()`,
plus `age()`/`producerTick()`. `maxAgeTicks == 0` means same-tick
freshness only — an async call can never satisfy it; pass the window
at construction and retune via `setMaxAgeTicks()`/`timeoutUs` setters
at runtime.

`DeadlineLatch<T>` (`DeadlineLatch.hpp`) is the lighter consumer half:
the invoke callback `offer(seq, value, receivedUs)`s responses; the
loop `consume(nowUs, deadlineUs)`s the newest sample plus a freshness
verdict against its own µs deadline — apply on fresh, else run the
local fallback.

## Failure and failover semantics

| Situation | Result |
|---|---|
| Unknown name/id | callback `success=false`, `InvalidId`, or `callPeer` returns false |
| Pending table full / send fails | `callPeer` returns `false`, callback never fires |
| Deadline expires | `timedOut=true`, `error=Timeout`, late response dropped |
| Session ends in flight | callbacks fire once with failure — never hung |
| Duplicate providers | fewest consecutive timeouts wins; manual `callPeer` can pin a session |
| Peer never registered | `callPeerFunction` returns `false` (not queued) |

Common pitfalls:

- **Unpumped client** — registered functions sit unread; the server
  sees consecutive timeouts and dispatches elsewhere.
- **`maxAgeTicks=0`** on a monitor — freshness requires same-tick
  results; always configure it.
- **Raw bytes for schema-typed args/returns** — encode through the
  schema (`nv::` builders / `SchemaValueBuilder`); the validator
  rejects unencodable payloads before user code runs.
- **Blocking the pump** — peer callbacks and the catch-all run inside
  `receiveMessage()`; delegate heavy work or every in-flight request
  stalls behind it.
