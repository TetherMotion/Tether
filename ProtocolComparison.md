# Tether IO Protocol and ParameterStreamProtocol comparison

**Status:** design/comparison document

## Implemented merge baseline

The direct Tether merge uses ParameterStreamProtocol v4's canonical message
IDs `0x01`–`0x13` for catalog, streaming, metadata, writes, ping/pong, and
log subscriptions. Tether's unused v1 wire numbering is not retained as a
compatibility ABI. The protocol version is `5`.

Where the original ParameterStream format used 16-bit stream/catalog counts,
Tether uses 32-bit counts for entry counts, filter counts, chunk sizes, row
counts, and metadata counts. This preserves Tether's larger-capacity behavior
and is an intentional wire-format choice for the merged protocol.

Tether-only operations remain available as extension messages `0x20`–`0x34`,
including the separate signal catalog, individual reads, snapshots, feature
exchange, catalog changes, datalogging, threshold filtering, and struct
descriptions. ParameterStream value IDs `14`–`19` are now reserved for IPv4,
IPv6, MAC, enum, unsigned varint, and signed varint; Tether structs use value
ID `20`.

Function RPC operations use extension messages `0x35`–`0x38`:
`ListFunctionsReq`, `ListFunctionsResp`, `CallFunctionReq`, and
`CallFunctionResp`. Function parameters are named and fully annotated in the
catalog, but invocation is positional. Calls use bounded TLV tuples, and
optional parameters must form a suffix with defaults. `ListParams` remains
parameter-only; functions are not mixed into that catalog.

The portable Tether session retains transport abstraction and SLIP overflow
recovery. It also implements ping/pong and filtered log subscriptions with
bounded per-session subscription counts and asynchronous server broadcasting.

**Compared implementations:**

- Tether IO: `include/tether/io/`, `src/io/`, `tests/io/`,
  `docs/IOProtocol.md`, `docs/IOProtocolWireFormat.md`, and
  `cmake/components/io_protocol.cmake`.
- ParameterStreamProtocol: `/home/uli/dev/Metexon/Granulatzufuehrung-Sensor-Firmware/PTX1-Firmware-ESPIDF/components/ParameterStreamProtocol/`, including `PROTOCOL.md`, all public headers, all sources, `Kconfig`, and the C++/Python/Rust examples.

The source code is treated as authoritative where it disagrees with a protocol
document or an example. This matters for both implementations: several
described features are present only as data structures or documentation, and
some documented field layouts no longer match the current code.

## 1. Executive summary

The two protocols share a wire ancestry and a useful common core:

- SLIP framing with the same `END`, `ESC`, `ESC_END`, and `ESC_ESC` bytes.
- One binary message per SLIP packet.
- A one-byte message discriminator.
- Little-endian fixed-width integers and IEEE-754 floating-point values.
- The same protobuf-style, unsigned 32-bit varint codec.
- A catalog of callback-backed values, paginated discovery, metadata, and
  timestamped binary stream rows.
- Per-connection stream state and one worker thread per client in the normal
  TCP server.

They are **not wire-compatible** beyond framing and a small prefix of the
value-type IDs. The most important incompatibilities are:

1. The message ID spaces diverge immediately at `0x03`.
2. Tether has separate parameters and signals; ParameterStreamProtocol has one
   unified parameter catalog.
3. The stream configuration structures use different widths and fields
   (`interval_us`/32-bit counts versus `interval_ms`/16-bit counts plus a
   trigger parameter and filters).
4. Tether value type `14` is `Struct`; ParameterStreamProtocol value type `14`
   is `IPv4`. Tether `Enum` is fixed four-byte data; ParameterStreamProtocol
   `Enum` is a variable unsigned varint.
5. Tether adds snapshots, feature exchange, catalog-change notifications,
   threshold filtering, datalogging, and struct descriptions. ParameterStream-
   Protocol adds ping/pong, a documented log-subscription protocol, declarative
   stream-filter schemas, static registries, and an ESP-IDF polling bridge.
6. Tether deliberately separates protocol logic from transports. The other
   implementation uses direct POSIX TCP in the normal path and callback-based
   framed I/O as an extension point.

The recommended merge is therefore **not** to make either existing version
pretend to be the other. Since Tether's wire format is not deployed, retaining
it has no inherent compatibility benefit. The preferred direction is to use
ParameterStreamProtocol v4 as the baseline wire format wherever it already
provides the needed operation, then add explicitly negotiated extensions for
the features that cannot be represented by v4. Tether's useful advantages are
primarily in its internal registry, transport abstraction, implementation
coverage, and extra control-plane features—not in its existing byte layout.

## 2. Layer model

The implementations can be compared as the following layers:

| Layer | Tether | ParameterStreamProtocol | Merge consequence |
|---|---|---|---|
| Application content | Parameters plus read-only signals, module exposers | One parameter catalog, optional writes, direct application registration | Keep a unified internal entry model with an explicit kind and write capability |
| Schema/catalog | Mutable thread-safe registry, revision and listeners | Immutable-after-start registry, heap and ROM/static entries | Add static storage support to Tether; retain Tether's kind and revision model |
| Value model | Scalar types, strings/binary, struct, enum | Scalar types, strings/binary, IPv4/IPv6/MAC, varints, enum | Use a collision-free type namespace in a new wire version |
| Serialization | Header-only bounded reader/writer and varint | Similar header-only bounded reader/writer and varint | Consolidate, but make endianness and malformed-input rules explicit |
| Framing | SLIP supplied by libSLIPspeed | SLIP supplied by SLIPStream | Share one tested SLIP codec or preserve a codec boundary |
| Session protocol | 33 message IDs, request/response plus async notifications | 19 documented message IDs, including ping/logs | Allocate a new message namespace/version; do not reuse overlapping IDs |
| Streaming | Time or any-entry-on-change, microsecond interval, 32-bit counts, threshold filter | Time or one-trigger-parameter-on-change, millisecond interval, 16-bit counts, filter properties | Define one stream model and an explicit compatibility mapping |
| Transport | `ITransport`/`ITransportServer`, TCP, serial, test pipes | Direct TCP, framed callback mode, FreeRTOS polling bridge | Make transport injection the common boundary; keep embedded adapters separate |
| Runtime | `std::thread`, configurable client limit, shared datalog recorder | `pthread`, configurable stack, custom session factory | Provide a platform policy layer rather than forcing one threading API |
| Integration | CMake component, C++23 project, module exposers | ESP-IDF/Kconfig, small-footprint options, static registry | Split portable protocol core from host and ESP-IDF integrations |

## 3. Content and catalog model

### 3.1 Tether's parameter/signal model

Tether has two entry structures:

- `ParamEntry`: an ID, name, description, group, value type, read callback,
  optional fixed-size and variable-size write callbacks, optional metadata,
  maximum variable value size, and optional `StructDescriptor`.
- `SignalEntry`: the corresponding read-only value. It has no write callback.

Both use the same `uint64_t` ID space and duplicate IDs are rejected across
both categories. `EntryView` provides a common view and exposes the entry kind,
readability, writability, metadata, and value operations.

The wire protocol reflects this model directly:

- `ListParamsReq`/`ListParamsResp` enumerate parameters.
- `ListSignalsReq`/`ListSignalsResp` enumerate signals.
- `GetParam*` and `SetParam*` address parameters.
- `GetSignal*` addresses signals.
- Streams can contain either kind because stream IDs are resolved through the
  unified registry lookup.

Tether also has `IParameterExposer` and well-known module ID bases. This keeps
core modules independent of the IO library while allowing PID, EtherCAT,
CiA 402, G-code, motion-planner, and simulation integrations to populate the
registry in a consistent way.

### 3.2 ParameterStreamProtocol's unified model

ParameterStreamProtocol has one `ParamEntry` type. A parameter may be read-only
by leaving its write callback null; there is no separate signal kind on the
wire or in the registry. `EntryView` can refer either to a heap-owned
`ParamEntry` or a `StaticParamEntry`.

The registry preserves insertion order through an `order_` vector and indexes
IDs with `idIndex_`. Static entries are pointers into caller-owned storage,
which is useful for ROM-friendly ESP-IDF catalogs but imposes a lifetime
requirement on the application. The documented thread-safety contract is that
registration happens before the server starts; reads then need no mutex.

### 3.3 Catalog comparison

| Capability | Tether | ParameterStreamProtocol |
|---|---|---|
| Entry kinds | Separate parameter and signal types | One parameter type |
| Write capability | Explicitly represented by callbacks and flags | Optional callbacks on every parameter |
| ID width | `uint64_t` | `uint64_t` |
| Ordering | Separate insertion-order parameter and signal vectors | One insertion-order vector across heap/static entries |
| Pagination | Separate parameter and signal pages; `uint32_t` offset/count | One page; `size_t` API and `uint32_t` wire offset/max count |
| Group field | Yes, included in Tether catalog responses | No group field |
| Metadata | String map | String map for heap entries; fixed metadata array for static entries |
| Dynamic registration | Supported; revision and change listeners | Not supported after startup by contract |
| Static/ROM entries | No dedicated static-entry API | `StaticParamEntry` and bulk `addStatic()` |
| Exposer abstraction | Yes, including module ID allocation | No; applications call `Registry::add()` directly |

### 3.4 Merge recommendation for content

Use a superset internal catalog resembling Tether's `EntryView`, but add a
storage policy:

```text
Entry {
    uint64 id
    EntryKind { parameter, signal }
    name, description, group
    ValueDescriptor
    read/write callbacks or static function pointers
    metadata
    optional struct/enum descriptor
}
```

The `EntryKind` must remain an internal and negotiated protocol concept. A
ParameterStreamProtocol compatibility view can expose signals as read-only
parameters, but this is a deliberate loss of information and must not be used
for a round-trip conversion back to Tether.

Tether should adopt static entries if ESP-IDF footprint is a goal. Static
entries should be stored in a separate stable collection or represented by
non-owning descriptors so adding a dynamic entry cannot invalidate pointers
held by an active stream. The current Tether registry stores entries in vectors;
dynamic vector growth can invalidate `EntryView` pointers already captured by a
collection plan, so catalog mutation during an active stream needs a lifetime
policy as part of the merge.

## 4. Value types and content encoding

### 4.1 Value type IDs

| ID | Tether v1 | Size | ParameterStreamProtocol v4 | Size | Compatibility |
|---:|---|---:|---|---:|---|
| 1 | `U8` | 1 | `U8` | 1 | Same |
| 2 | `U16` | 2 | `U16` | 2 | Same |
| 3 | `U32` | 4 | `U32` | 4 | Same |
| 4 | `U64` | 8 | `U64` | 8 | Same |
| 5 | `I8` | 1 | `I8` | 1 | Same |
| 6 | `I16` | 2 | `I16` | 2 | Same |
| 7 | `I32` | 4 | `I32` | 4 | Same |
| 8 | `I64` | 8 | `I64` | 8 | Same |
| 9 | `F32` | 4 | `F32` | 4 | Same |
| 10 | `F64` | 8 | `F64` | 8 | Same |
| 11 | `Bool` | 1 | `Bool` | 1 | Same basic representation |
| 12 | `String` | Variable | `String` | Variable | Same length convention |
| 13 | `Binary` | Variable | `Binary` | Variable | Same length convention |
| 14 | `Struct` | Variable | `IPv4` | 4 | **Collision** |
| 15 | `Enum` | 4 | `IPv6` | 16 | **Collision** |
| 16+ | Not defined through 19 | — | `MAC` 16, `Enum` 17, `UVarint` 18, `IVarint` 19 | Variable/fixed | Tether has no compatible IDs |

Tether's `valueTypeSize()` reports four bytes for `Enum` and zero for
`Struct`, while its `isVariableLength()` helper reports only `String` and
`Binary`. `ParamEntry::isVariableLength()` additionally treats `Struct` as
variable because a struct has a descriptor and a configured maximum size.
The result is that a struct value is serialized with a varint length in get,
set, snapshot, and variable-entry stream paths.

ParameterStreamProtocol reports `Enum`, `UVarint`, and `IVarint` as variable
length. Its enum is an unsigned varint whose labels are expected to be
described through metadata. The two enum encodings cannot be decoded by the
other implementation even before considering their conflicting IDs.

### 4.2 Common scalar encoding

Both implementations intend the following encoding:

- All multi-byte integer fields are little-endian.
- `U8` through `I64` are raw fixed-width values.
- `F32` and `F64` are raw IEEE-754 bytes.
- `Bool` is one byte, with zero false and nonzero true.
- `String` and `Binary` values are `[uint32 varint length][payload]`.
- Descriptor strings, names, descriptions, groups, and metadata keys/values
  use `[uint16 little-endian length][bytes]`.

The C++ implementations use `memcpy` for integer and floating-point fields.
That is correct on the stated little-endian targets, but it is not a portable
implementation of the wire-endian promise on a big-endian host. A merged core
should encode/decode each integer explicitly or enforce little-endian targets.
Neither implementation validates UTF-8.

### 4.3 Varints

The core varint codec is effectively the same in both implementations:

- 7 payload bits per byte.
- Least-significant group first.
- High bit means continuation.
- Maximum five bytes for `uint32_t`.
- Bounds/capacity failure is represented by zero bytes or a sticky reader error.

ParameterStreamProtocol additionally provides 32- and 64-bit zigzag helpers
for signed varints. Tether has no corresponding signed-varint value type.

### 4.4 Flags and descriptors

Tether's catalog entry flags are:

| Bit | Tether |
|---:|---|
| `0x01` | Readable |
| `0x02` | Writable |
| `0x04` | Variable length |
| `0x08` | Has struct descriptor |
| `0x10` | Has enum information |

ParameterStreamProtocol has only the first three flags (`ParamReadable`,
`ParamWritable`, `ParamVariableLen`). The Tether `HasEnum` bit is defined, but
the current `ParamEntry` and `SignalEntry` do not carry enum label data or set
that bit automatically. This is an unfinished part of the Tether superset.

Tether's `StructDescriptor` contains an entry ID, type name, total size, and
fields with name, type, byte offset, byte size, and unit. It is requested by
`DescribeStructReq` and returned by `DescribeStructResp`.

ParameterStreamProtocol has no struct descriptor. IPv4, IPv6, MAC, and enum
semantics are represented as value types and/or metadata rather than a
separate descriptor message.

## 5. Framing and binary message envelope

Both protocols use standard SLIP:

| Byte sequence | Meaning |
|---|---|
| `0xC0` | End/delimiter |
| `0xDB 0xDC` | Literal `0xC0` in payload |
| `0xDB 0xDD` | Literal `0xDB` in payload |

The normal sender emits a payload followed by `END`; each payload contains
exactly one protocol message, beginning with `MessageType`. Multiple SLIP
packets may arrive in one TCP read.

Tether delegates encoding/decoding to `libSLIPspeed` and keeps an 8192-byte
receive buffer and 8192-byte decode buffer per session. On overflow it enters a
discard-until-next-END state, which prevents a suffix of an oversized frame
from being interpreted as a new message.

ParameterStreamProtocol uses `SLIPStream`. Its direct TCP session has a
`MAX_MESSAGE_SIZE` of 2048 and fixed receive/decode arrays, but the current
`feedSlipData()` resets the receive position on overflow without an explicit
discard-until-END state. A malformed oversized frame can therefore be
resynchronized differently from Tether. Its separate framed-session
constructor does not perform SLIP itself; the injected receive callback is
expected to return one complete message.

Neither protocol authenticates, encrypts, or checksums a SLIP payload. SLIP
provides framing only. TCP deployments bind to `INADDR_ANY` and have no TLS or
application authentication in these modules.

## 6. Message protocol comparison

### 6.1 Message ID map

| Code | Tether v1 | Direction | ParameterStreamProtocol v4 | Direction |
|---:|---|---|---|---|
| `0x01` | ListParamsReq | C→S | ListParamsReq | C→S |
| `0x02` | ListParamsResp | S→C | ListParamsResp | S→C |
| `0x03` | ListSignalsReq | C→S | ConfigureStream | C→S |
| `0x04` | ListSignalsResp | S→C | ConfigureAck | S→C |
| `0x05` | GetParamReq | C→S | StartStream | C→S |
| `0x06` | GetParamResp | S→C | StopStream | C→S |
| `0x07` | SetParamReq | C→S | StreamData | S→C |
| `0x08` | SetParamResp | S→C | Error | S→C |
| `0x09` | GetSignalReq | C→S | GetMetadataReq | C→S |
| `0x0A` | GetSignalResp | S→C | GetMetadataResp | S→C |
| `0x0B` | ConfigureStreamReq | C→S | SetParameterReq | C→S |
| `0x0C` | ConfigureStreamAck | S→C | SetParameterResp | S→C |
| `0x0D` | StartStream | C→S | PingReq | C→S |
| `0x0E` | StopStream | C→S | PongResp | S→C |
| `0x0F` | StreamData | S→C | SubscribeLogReq (documented) | C→S |
| `0x10` | Error | S→C | SubscribeLogResp (documented) | S→C |
| `0x11` | GetMetadataReq | C→S | UnsubscribeLogReq (documented) | C→S |
| `0x12` | GetMetadataResp | S→C | UnsubscribeLogResp (documented) | S→C |
| `0x13` | SnapshotParamsReq | C→S | LogData (documented) | S→C |
| `0x14` | SnapshotParamsResp | S→C | — | — |
| `0x15` | SnapshotSignalsReq | C→S | — | — |
| `0x16` | SnapshotSignalsResp | S→C | — | — |
| `0x17` | FeatureExchangeReq | C→S | — | — |
| `0x18` | FeatureExchangeResp | S→C | — | — |
| `0x19` | CatalogChanged | S→C | — | — |
| `0x1A` | ConfigureDatalogReq | C→S | — | — |
| `0x1B` | ConfigureDatalogResp | S→C | — | — |
| `0x1C` | DatalogStatusReq | C→S | — | — |
| `0x1D` | DatalogStatusResp | S→C | — | — |
| `0x1E` | ConfigureThresholdReq | C→S | — | — |
| `0x1F` | ConfigureThresholdResp | S→C | — | — |
| `0x20` | DescribeStructReq | C→S | — | — |
| `0x21` | DescribeStructResp | S→C | — | — |

This table alone rules out direct negotiation by simply changing the protocol
version byte. A ParameterStreamProtocol client sending `ConfigureStream` at
`0x03` to Tether asks for a signal catalog, and a Tether client sending
`ListSignalsReq` at `0x03` asks ParameterStreamProtocol to configure a stream.

### 6.2 Common catalog request

Both `ListParamsReq` messages are nine bytes including the type:

```text
[type: u8 = 0x01][offset: u32][max_count: u32]
```

Tether's response header is:

```text
[0x02][total_count: u32][offset: u32][count: u32]
```

Each Tether entry is:

```text
[id: u64][value_type: u8][value_size: u8][flags: u8]
[name: str16][description: str16][group: str16]
```

ParameterStreamProtocol's response header and entry are:

```text
[0x02][total_count: u32][offset: u32][count: u16]
[id: u64][value_type: u8][value_size: u8][flags: u8]
[name: str16][description: str16]
```

Thus the request is reusable, but the response count width and entry tail are
different. A Tether-compatible adapter can convert a Tether parameter page to
the older form, clamping or paging at `uint16_t` count limits and dropping the
group. It cannot represent signals without presenting them as read-only
parameters.

### 6.3 Single-value access

Tether has separate get operations. A fixed-size response is:

```text
[response_type][entry_id: u64][value_size: u8][value bytes]
```

For a variable entry, Tether writes the value-size byte as zero (the
`valueSize()` result) and then writes `[length: varint][value bytes]`.

ParameterStreamProtocol has no `GetParamReq`/`GetParamResp`; a value is
normally observed by starting a stream. It does have `SetParameterReq` and
`SetParameterResp`, at IDs `0x0B` and `0x0C`, with the same ID followed by a
fixed value or a varint-length value. Tether's set operation is at `0x07` and
`0x08` and is named `SetParam`.

For a compatibility adapter:

- Tether get → ParameterStreamProtocol requires a one-row temporary stream or
  an adapter extension; it is not a wire-level mapping.
- ParameterStreamProtocol set → Tether can map to `SetParamReq` only after
  resolving the target as a parameter and checking writability.
- A Tether signal must be rejected for set, while the unified old model only
  exposes that fact through its writable flag.

### 6.4 Metadata

Both use an ID request and string key/value pairs. The differences are:

| Field | Tether | ParameterStreamProtocol |
|---|---|---|
| Request ID | `0x11`, any param or signal | `0x09`, parameter catalog entry |
| Response type | `0x12` | `0x0A` |
| Pair count | `u32` | `u16` |
| Pair format | `str16 key`, `str16 value` | Same |

Tether can return metadata for signals and can additionally return a struct
descriptor through a separate request. ParameterStreamProtocol metadata is
also where the documented enum labels and application filter semantics are
expected to live.

### 6.5 Stream configuration

Tether's request body after the type is:

```text
[trigger_mode: u8]
[interval_us: u32]
[chunk_size: u32]
[skip_count: u32]
[entry_count: u32]
[entry_id: u64] * entry_count
```

The current Tether handler requires at least the first 17 body bytes. It
accepts both parameters and signals, ignores IDs that cannot be resolved when
building the plan, and returns an acknowledgement containing the resolved
plan. It does not enforce the advertised `TooManyEntries` error in the current
handler.

ParameterStreamProtocol's request body is:

```text
[trigger_mode: u8]
[interval_ms: u32]
[chunk_size: u16]
[skip_count: u16]
[trigger_param_id: u64]
[param_count: u16]
[param_id: u64] * param_count
[filter_count: u16]
[filter entry] * filter_count
```

The filter entry is `[name_len: u8][name][value_type: u8][value]`, where a
fixed type has its fixed bytes and a variable type has a varint length followed
by data. The protocol document says filters are schema validated and invalid
filters produce a rejection. The current `Session::handleConfigureStream()`
instead logs that filters are not implemented, parses and skips them, and
continues configuration. `StreamFilterSchema` is implemented as a standalone
validator but is not connected to that session handler.

The acknowledgement layouts are also different:

| Field | Tether current implementation | ParameterStreamProtocol |
|---|---|---|
| Type | `0x0C` | `0x04` |
| Spec ID | `u32` | `u32` |
| Resolved count | `u32` | `u16` |
| Row size | `u32` | `u16` |
| Per resolved item | `u64 id`, `u8 type`, `u8 size` | Same |

The Tether wire-format document currently omits the resolved-count and row-size
widths and does not describe the per-entry portion correctly. The source and
tests use the four-byte widths shown above. The ParameterStreamProtocol
document and source agree on the acknowledgement layout, although the
current C++/Python/Rust examples are not all consistent with the documented
filter-count tail.

### 6.6 Stream control and data

Both use one-byte start and stop requests and flush a partial chunk on stop.
The `StreamData` header is:

| Field | Tether | ParameterStreamProtocol |
|---|---:|---:|
| Type | `0x0F` | `0x07` |
| Spec ID | `u32` | `u32` |
| Row count | `u32` | `u16` |

Each row starts with an eight-byte microsecond timestamp followed by values in
the resolved configuration order. Fixed values have no per-value type tags or
lengths. Variable values are varint-length-prefixed.

Tether precomputes a collection plan and applies threshold filtering for
fixed-size entries. ParameterStreamProtocol precomputes a similar plan but has
no threshold sampling filter. For a fixed row layout, the older protocol's
`u16 row_size` and `u16 row_count` limits are materially smaller than Tether's
advertised 32-bit fields.

### 6.7 Error responses

Both error payloads have the same shape:

```text
[error type][error_code: u32][message: str16]
```

The error type is `0x10` in Tether and `0x08` in ParameterStreamProtocol.

The shared codes 0 through 8 have essentially the same meaning, although the
names differ (`InvalidId` versus `InvalidParameterId`, and `TooManyEntries`
versus `TooManyParameters`). Tether adds:

| Tether code | Meaning |
|---:|---|
| 9 | NotWritable |
| 10 | FeatureNotSupported |
| 11 | DatalogError |
| 12 | ThresholdError |

ParameterStreamProtocol declares `NotWritable` at 9. Its
`FilterPropertyErrorType` values are internal schema-validation results, not
the on-wire error code set.

### 6.8 Tether-only control planes

Tether's additional operations are substantial protocol layers rather than
minor extensions:

- **Snapshots (`0x13`–`0x16`):** request selected IDs or all parameters/signals;
  response contains one timestamp, a count, and ID/value records. The handler
  reads all values while constructing one response, but this is not a
  transaction or synchronization barrier across callbacks.
- **Feature exchange (`0x17`/`0x18`):** a count followed by name, `ValueType`,
  four-byte value length, and raw typed value. The server always adds a
  `protocol_version` feature if one was not supplied.
- **CatalogChanged (`0x19`):** asynchronous revision notification generated by
  registry change listeners.
- **Datalogging (`0x1A`–`0x1D`):** configuration, state, counters, record layout,
  and a raw record sink.
- **Thresholds (`0x1E`/`0x1F`):** named whitelist/blacklist rule sets with
  absolute, relative, or custom evaluation.
- **Struct description (`0x20`/`0x21`):** explicit field layout and units.

The current datalogging path is only partial: `DatalogRecorder::configure()`
creates placeholder fields with zero sizes unless the caller fills metadata,
and the session supplies a null sink. The server owns one recorder shared by
all sessions without a documented synchronization policy. These issues should
be resolved before advertising datalogging as a production merged feature.

### 6.9 ParameterStreamProtocol-only control planes

ParameterStreamProtocol declares:

- **Ping/pong (`0x0D`/`0x0E`):** a four-byte nonce echo for connectivity checks.
- **Log subscriptions (`0x0F`–`0x13`):** severity, component, message, and
  location substring filters, subscription IDs, and log records.

The current `Session.cpp` dispatches list, stream, metadata, set, and ping
handlers, but it contains no subscribe/unsubscribe/log-data handlers. The
public `Session.hpp` shown in the compared tree likewise has no log handler
methods. These messages are therefore **documented protocol surface, not a
complete server implementation** in the inspected revision.

The protocol document also says that an invalid stream filter produces
`ConfigureStreamReject`, but no such message type is defined in `Protocol.hpp`
and the current session skips filters. A merge should either implement these
features end-to-end or remove them from the advertised capability set.

## 7. Streaming semantics and polling

### 7.1 Trigger semantics

The trigger enum values are nominally shared (`Time = 0`, `OnChange = 1`) but
the meaning differs:

- Tether's `OnChange` compares every configured entry and triggers when any
  sampled value changes. There is no `trigger_param_id` field in its request.
- ParameterStreamProtocol compares only the configured `trigger_param_id`.
  The trigger parameter need not be one of the streamed parameters.

Both initialize the first observed value as a trigger, apply `skip_count` to
triggered samples, and accumulate rows until `chunk_size` is reached. Both
stop the current stream when a new configuration is accepted. Tether's
interval is in microseconds; ParameterStreamProtocol's is in milliseconds.

### 7.2 Thresholds versus stream filters

These are different concepts and should not be merged under one name:

| Concept | Tether | ParameterStreamProtocol |
|---|---|---|
| Purpose | Suppress insignificant changes from stream output | Validate/configure named stream properties |
| Configuration | Separate `ConfigureThresholdReq` | Filter properties inside stream configure |
| Selection | Per-entry rule plus default; whitelist/blacklist | Named schema properties |
| Built-ins | None, absolute, relative | Type/range/implemented checks |
| Extension | `CustomThresholdFn` callback | No connected runtime evaluator |
| Runtime | Applied to fixed entries during collection | No filtering runtime in current session |

Tether's current threshold evaluator is byte-size based: four-byte values are
interpreted as `float`, eight-byte values as `double`, and other sizes are
compared bytewise. Consequently an absolute threshold on a `U32` or `I32` is
not numerically interpreted as that integer type. Variable-length collection
also follows a separate path that does not apply the fixed-entry threshold
logic. These behaviors should be specified or corrected during a merge.

### 7.3 ParameterStreamProtocol `PollingBridge`

`PollingBridge` is an additional ESP-IDF application utility, not another SLIP
layer. It:

- Runs a FreeRTOS task at a configurable interval.
- Reads selected fixed-size registry entries.
- Pushes `PolledValueSample` records into a FreeRTOS queue.
- Drops the oldest sample when the queue is full.
- Exposes JSON list and control functions using cJSON.
- Provides start/stop, interval, selected IDs, queue depth, and page-size
  controls.

`PolledValueSample` has a 16-byte value array, so values with size zero
(including variable types) or larger than 16 bytes are not sampled. The bridge
has its own interval clamping and does not emit SLIP messages. Tether has no
equivalent, but its transport-independent session and registry would make a
FreeRTOS bridge easier to add without changing the wire protocol.

## 8. Transport and framing architecture

### 8.1 Tether transports

Tether defines two explicit interfaces:

```text
ITransport
  send(data, length)
  receive(buffer, max_length, timeout_ms)
  close()
  isConnected()

ITransportServer
  start()
  stop()
  accept() -> unique_ptr<ITransport>
  isListening()
```

`Session` owns one `ITransport`, performs SLIP deframing, dispatches messages,
and owns all per-client stream state. `Server` owns an `ITransportServer` and
creates one session thread per accepted transport.

Implemented transports:

- `TcpTransport`/`TcpTransportServer`: POSIX TCP, full-send loops, `select()`
  timeout receive, `SO_REUSEADDR`, and `TCP_NODELAY` on accepted connections.
- `SerialTransport`: wraps an injected `ISerialDriver`; the POSIX driver uses
  raw termios 8N1 and configurable baud rates. There is no serial server
  acceptor in the module, so serial is a client-side/point-to-point transport
  unless the application supplies one.
- `PipeTransport` in tests: in-process test transport, not a production wire
  transport.

`SpiDriver` is a separate full-duplex SPI peripheral driver for devices such as
accelerometers and thermocouples. It is in the IO source set but is not a
protocol transport and should not be confused with a serial or TCP endpoint.

### 8.2 ParameterStreamProtocol transports

The normal server opens an AF_INET TCP listener directly in `Server.cpp` and
passes the accepted file descriptor to `Session`. The session performs
`select()` and `recv()` itself and sends SLIP-encoded bytes directly.

The second session constructor provides a useful alternative:

- `ReceiveFn` returns a complete message and a `ReceiveStatus`.
- `SendFn` sends one complete message.
- `StopTransportFn` interrupts a blocking receive during shutdown.
- `TransportMode::Framed` bypasses the session's SLIP/TCP code.

This mode can host BLE, IPC, or an already-framed link, but it is a callback
boundary rather than a reusable transport abstraction. There is no serial
driver or serial transport in the component itself. The `PollingBridge` is an
ESP-IDF polling/JSON utility, not a byte transport.

### 8.3 Recommended transport merge

Use Tether's `ITransport` boundary as the portable interface, then supply:

1. Tether POSIX TCP and serial implementations.
2. An ESP-IDF socket transport.
3. An ESP-IDF UART/USB-CDC transport.
4. A framed adapter whose `receive()` returns already-decoded payloads, if BLE
   or another bearer cannot expose a byte stream.

The session should not contain POSIX headers, FreeRTOS headers, or bearer-specific
logic. A transport may either expose raw bytes and let the session run SLIP, or
explicitly implement a `IMessageTransport` mode; mixing both implicitly makes
double-framing bugs likely.

## 9. Session, server, and concurrency model

### Tether

- `ServerConfig`: maximum clients, timestamp callback, log callback, and server
  feature set.
- `Server`: background accept thread and one `std::thread` per session.
- `Session`: one owned transport, fixed receive/decode buffers, dynamically
  sized transmit and stream buffers, atomic stop/running flags.
- Registry add operations are mutex-protected and notify listeners outside the
  registry lock. Read operations also lock.
- A session registers a catalog listener and asynchronously sends a revision
  notification.
- `requestStop()` closes the transport to interrupt a blocked receive.

### ParameterStreamProtocol

- `ServerConfig`: port, maximum clients, session stack size, timestamp/log
  callbacks, and an optional `SessionFactory`.
- `Server`: direct listener fd, pthread accept thread, pthread session threads,
  configurable stack sizing.
- `Session`: either owns a TCP fd or uses callback-based framed I/O.
- The registry is expected to be immutable after startup and has no change
  notifications.
- `requestStop()` shuts down the TCP fd or invokes the injected stop callback.

### Important runtime differences

1. Tether can serve TCP, serial-adapted, and test transports through the same
   session. ParameterStreamProtocol's custom session factory is less invasive
   for a custom bearer but leaves transport lifecycle inside the session.
2. ParameterStreamProtocol exposes stack sizing, which is important on an
   ESP32. Tether should add a platform-specific thread configuration rather
   than hard-coding an 8192-byte POSIX-style default into the portable layer.
3. Tether's `DatalogRecorder` is held by `Server` and passed to every session.
   It is therefore shared mutable state despite the otherwise independent
   session model. It needs a mutex, per-session recorder, or an explicit single
   owner before concurrent clients can configure it safely.
4. Tether's mutable registry and listener support are more capable, but entry
   pointer stability must be fixed before live additions are allowed while
   collection plans exist.

## 10. Error handling and robustness

### Shared strengths

- Both use bounded readers/writers with sticky error state.
- Both reject truncated fixed fields and truncated varint payloads in normal
  handlers.
- Both cap varints at five bytes.
- Both reject unknown IDs and invalid stream lifecycle operations.
- Both cap client concurrency at the server level.

### Tether-specific considerations

- The session has an explicit discard-until-END state after an oversized SLIP
  receive frame.
- There is no built-in keepalive or idle timeout.
- Several handler counts and requested chunk sizes are used to size vectors and
  buffers without a protocol-level maximum. The `TooManyEntries` error exists
  but is not currently enforced in `handleConfigureStreamReq()`.
- Feature and descriptor decoders resize vectors from wire counts before an
  application-defined maximum is checked.
- `BufWriter::putStr16()` casts a `size_t` to `uint16_t` without rejecting a
  string longer than 65535 bytes; it can encode a wrapped length.
- The session uses a 1024-byte temporary buffer for variable stream values,
  even though an entry may advertise a larger maximum.

### ParameterStreamProtocol-specific considerations

- The direct session declares a 2048-byte maximum, but its overflow recovery
  should discard through END rather than merely reset the position.
- `chunk_size`, `skip_count`, and response counts are deliberately 16-bit on
  the wire, which bounds resource use but needs explicit rejection or clamping
  at the API boundary.
- The current handler does not enforce a published maximum parameter count.
- Static registry pointers require the caller to keep all descriptors and
  metadata alive.
- Filter values are parsed/skipped but not schema validated in the current
  session.
- The server has no TLS, authentication, idle timeout, or keepalive policy;
  ping/pong is useful only if a client and a complete handler use it.

### Merged security baseline

Before exposing either protocol on an untrusted network, add:

- A hard maximum decoded payload size before allocation.
- Hard maxima for all wire counts, strings, metadata pairs, fields, stream
  entries, rows, and variable values.
- Strict exact-consumption checks where a message has no extension tail.
- Explicit rejection of invalid enum/type IDs and malformed fifth varint bytes.
- Rate limiting and per-client memory budgets.
- A keepalive/idle policy.
- Authentication and TLS at the transport or deployment boundary.

## 11. Build, dependency, and platform comparison

### Tether

The CMake component `cmake/components/io_protocol.cmake` builds:

- `libSLIPspeed` static/shared variants from the Tether dependency tree.
- `tether_io_protocol` static/shared variants.
- Registry, threshold, datalog, session, server, TCP, and (on non-ESP
  targets) serial/SPI sources.

It links `tether_common`, SLIP, and `Threads::Threads`. The component CMake
sets C++20 on its targets, while the project conventions and surrounding Tether
code use C++23; this should be normalized during the merge. POSIX TCP is in the
common source list, while host serial and SPI implementations are omitted for
`ESP_PLATFORM` and are expected to be supplied through embedded drivers.

### ParameterStreamProtocol

The component is organized for ESP-IDF and uses Kconfig options:

- `PARAMETER_STREAM_PROTOCOL_ENABLE_VARIABLE_LENGTH`: string/binary support,
  default disabled.
- `PARAMETER_STREAM_PROTOCOL_MAX_STRING_LENGTH`: configured variable payload
  limit, default 128 and range 1–4096.
- `PARAMETER_STREAM_PROTOCOL_ENABLE_BLE_POLLING_BRIDGE`: FreeRTOS/cJSON
  polling utility, default enabled.

The protocol headers use C++ standard library containers and callbacks. The
normal server/session includes POSIX socket and pthread APIs; the polling
bridge additionally includes FreeRTOS queue, semaphore, and task APIs and cJSON.
The inspected component directory contains no independent test suite, but it
does contain C++, Python, and Rust client examples.

### Example quality and compatibility implications

The examples reveal protocol drift that a merge should eliminate:

- The C++ example uses `SLIPParameterStream/Protocol.hpp`, while the public
  component headers use `ParameterStreamProtocol/Protocol.hpp`.
- The C++ client constructs a configure message without the documented trailing
  `filter_count`; the current server requires and reads that field.
- The Python client implements only scalar value types 1–11 and the older
  `u16` response count/layout.
- The Rust example has a `ConfigureStreamResponse` enum value of `0x14` even
  though the protocol table and server use `0x04`.
- The Python and Rust examples do not model variable-length rows.

The merged project should provide one canonical test client or codec fixture
shared by C++, Python, and Rust examples, generated from the same wire schema.

## 12. Compatibility assessment

### Direct connection

Direct connection between the current servers and clients is not viable. SLIP
will successfully delimit packets, but message IDs and field layouts will be
misinterpreted. A client must know which protocol version and namespace it is
speaking before sending the first request.

### Lossless mappings

The following can be mapped without changing value content if the adapter has
already negotiated a compatible message dialect:

- Common scalar values 1–13, excluding any use of the conflicting type IDs and
  assuming fixed/variable semantics are known from the source protocol.
- SLIP packets and the common request for a parameter page, after converting
  the response header and dropping/adding group information as appropriate.
- Metadata key/value strings, with count-width conversion.
- Set operations for writable parameter IDs.
- Fixed-size stream rows after converting the stream header, interval unit,
  count widths, and resolved-entry list.

### Lossy mappings

- Tether signals → old unified parameters: read-only status can be retained as
  a writable=false flag, but signal catalog identity and `GetSignal` semantics
  are lost.
- Tether groups → old catalog: group must be dropped or copied into metadata.
- Any-entry-on-change → one trigger parameter: choose a trigger ID or emulate
  the behavior in the adapter; neither is exact on the old wire.
- Tether snapshots → old protocol: use one-row streams, losing one-response
  snapshot semantics and signal/parameter separation.
- Tether thresholds → old filters: no equivalent runtime operation.
- Tether datalog records → log subscriptions: these are different products;
  binary field records cannot be represented as text log messages.
- ParameterStreamProtocol filters → Tether: a schema validation property is
  not necessarily a Tether threshold rule.
- IPv4/IPv6/MAC/varints → Tether v1: require binary/string fallback and an
  out-of-band type descriptor, or a new value-type namespace.

### Adapter modes

There are two practical compatibility adapters:

1. **ParameterStream façade over Tether (recommended).** Present Tether's
  parameters and signals as ParameterStreamProtocol's unified catalog, use the
  ParameterStream message IDs and layouts, and support scalar values and
  fixed-size streams first. This gives new Tether users the already-defined
  ParameterStream wire format without preserving an unused Tether wire ABI.
2. **Tether-wire façade over ParameterStreamProtocol (not recommended).** This
  would preserve the unused Tether layout and require a signal-aware façade,
  snapshots, and negotiated feature handlers around the other registry. It
  provides no deployment benefit in the stated situation.

A gateway that terminates both protocols and has two independent sessions is
  preferable to attempting to parse one protocol with the other session
  implementation.

## 13. Recommended merge design

### Phase 1: Consolidate the portable core

Create a protocol-core library with no sockets, threads, CMake platform logic,
FreeRTOS, or application modules. It should provide:

- Explicit little-endian integer codecs.
- Strict bounded reader/writer and varint/zigzag utilities.
- A shared SLIP encoder/decoder with a documented maximum frame size and
  discard-until-END recovery.
- A versioned `ValueDescriptor` supporting scalar, string, binary, IP, MAC,
  varint, enum, and struct forms without numeric collisions.
- Common catalog entry/metadata descriptors.
- Golden byte fixtures for every message and malformed-input cases.

Make ParameterStreamProtocol v4 the canonical wire profile for the common
subset. Preserve Tether v1 codec definitions only as an optional source/API
compatibility layer or test fixture; there is no reason to expose it as a
second server wire mode if no clients use it. Do not silently change the
meaning of any ParameterStreamProtocol v4 type or message ID.

### Phase 2: Unify the internal content model

Adopt a registry with:

- Parameter and signal kinds.
- Optional write support independent of kind.
- Stable dynamic-entry references.
- Heap and static/ROM storage policies.
- Groups, metadata, struct fields, and enum labels.
- Catalog revision and listener support.

Provide two public registration façades:

- Tether's exposer API for Tether modules and stable module IDs.
- A compatibility `add(ParamEntry)` and `addStatic()` API for the ESP-IDF
  component.

### Phase 3: Define extensions to the ParameterStream wire

A merged protocol should retain ParameterStreamProtocol v4 messages and
explicitly define a negotiated extension profile for features outside v4:

- A capability exchange before using extension messages, or a new versioned
  connection preamble that cannot be confused with a v4 message.
- Extension message IDs outside the existing v4 range, leaving the documented
  log-subscription IDs reserved until those messages are implemented.
- 32-bit or 64-bit count policy with hard maxima.
- A single stream configuration model with an interval unit explicitly named.
- Separate trigger configuration from output filtering.
- Signal/parameter kind in stream resolution.
- A collision-free type registry that keeps v4 IDs 1–19 unchanged and assigns
  new IDs to types such as Tether structs.
- Feature advertisement for snapshots, thresholds, structs, datalogging,
  ping/pong, log subscriptions, static catalogs, and variable types.

The extension profile can carry the union of Tether's and
ParameterStreamProtocol's features, but it should not promise a feature until
its server handler, transport path, and client fixture are complete.

### Phase 4: Add platform adapters

- Keep `ITransport`/`ITransportServer` as the common transport boundary.
- Implement POSIX TCP and serial in Tether.
- Implement ESP-IDF TCP, UART/USB, and FreeRTOS task integration in a separate
  platform component.
- Adapt `PollingBridge` to the unified stable registry and make its JSON API
  explicitly separate from the binary wire protocol.
- Add a session factory only where a platform needs custom task creation or a
  pre-framed bearer.

### Phase 5: Deprecate only after interoperability tests

For each legacy version, add a compatibility test matrix covering:

- Catalog pages with zero, one, and maximum entries.
- Every scalar and variable value type.
- Set/get and read-only behavior.
- Stream configure/ack/data/stop/reconfigure.
- Malformed lengths, counts, varints, SLIP escapes, and oversized frames.
- Multiple clients and shutdown while blocked.
- Static and dynamic registry entries.
- Feature negotiation and unsupported-feature behavior.

Only after these tests pass should old names and headers be deprecated or
renamed. Existing ESP32 deployments should remain on a clearly identified
legacy wire version until a new client is available.

## 14. Low-hanging-fruit merge plan

The following sequence provides useful interoperability quickly without
committing to the unused Tether wire format:

1. **Freeze the wire decision:** declare ParameterStreamProtocol v4 the base
  profile for new clients. Keep the Tether v1 constants only in a compatibility
  namespace if source-level users need them.
2. **Reuse the common codec:** consolidate the bounded reader/writer, varint,
  string, and SLIP tests. Keep the ParameterStream framing behavior but fix
  oversized-frame recovery to discard until `END`.
3. **Add a ParameterStream session profile to Tether:** implement list,
  configure, configure-ack, start, stop, stream-data, metadata, set, error,
  and ping using ParameterStream's existing IDs and field widths. Reuse
  Tether's transport-independent session machinery rather than copying the
  POSIX socket code.
4. **Adapt the registry:** expose Tether parameters and signals as one ordered
  read-only/read-write ParameterStream catalog. Preserve the original kind
  internally; omit `group` from v4 or expose it as metadata such as
  `tether.group`.
5. **Normalize stream semantics at the boundary:** convert wire milliseconds
  to Tether's internal microseconds, reject values that exceed v4's `u16`
  chunk/skip/count limits, and implement ParameterStream's single
  `trigger_param_id` semantics explicitly. Do not silently claim that this is
  equivalent to Tether's any-entry-on-change behavior.
6. **Use the shared scalar type IDs:** types 1–13 can be supported first. Add
  ParameterStream's IPv4, IPv6, MAC, enum, and varint types to the internal
  value model without reusing Tether's conflicting IDs. Represent structs as
  binary plus metadata initially, or add a negotiated extension type later.
7. **Add static registry support:** import the static/ROM entry concept with
  stable lifetimes. This is useful independently of the wire protocol and is
  likely the most valuable ESP-IDF-specific merge.
8. **Add small extensions next:** individual get operations, snapshots,
  catalog revision/notifications, and feature exchange are straightforward
  extension candidates. Use new IDs above the v4 range and require capability
  negotiation before emitting them.
9. **Defer complex or incomplete features:** finish and test datalogging,
  threshold evaluation, struct descriptors, and log subscriptions before
  assigning them production status. Do not reserve the documented
  ParameterStream log IDs for unrelated Tether operations.
10. **Harden both paths while touching them:** enforce all count and string
   limits, fix `putStr16()` overflow, size variable-value buffers from the
   negotiated maximum, and require exact message consumption.

The first interoperability milestone should therefore be: a Tether server and
ParameterStream client can list a catalog, set a writable scalar, configure a
time-based fixed-size stream, receive rows, stop it, and ping the server. That
milestone avoids every Tether-only wire feature and gives the merge a concrete
compatibility test.

## 15. Decisions still required

The merge needs explicit decisions on the following points:

1. Is the new protocol expected to preserve byte compatibility with either
   deployed version, or is a gateway acceptable?
2. Are signals first-class on the new wire, or are all values intentionally
   unified as optional-write parameters?
3. Is `Enum` fixed-width or varint, and where are labels carried?
4. Are structs a first-class type, a descriptor attached to binary, or both?
5. Are stream filters selection/schema properties, runtime threshold rules, or
   two separate features?
6. Is datalogging a server-side file/sink facility, a streamed protocol, or an
   application API outside the wire protocol?
7. Which log-subscription messages are required, and who owns log fan-out?
8. What are the maximum frame, catalog, stream, metadata, and client limits on
   Linux and ESP-IDF?
9. Is catalog mutation during active sessions supported?
10. Is security provided by the transport deployment, or must the protocol
    include authentication and encryption negotiation?

## 16. Bottom line

ParameterStreamProtocol contributes valuable embedded-oriented features:
static registries, configurable task stacks, a framed callback mode, compact
16-bit stream counts, ping/pong intent, and a useful FreeRTOS polling bridge.

Tether provides the stronger long-term protocol architecture: explicit
parameter/signal semantics, transport separation, dynamic catalog events,
feature exchange, bulk snapshots, structured values, thresholds, and a broader
test/build integration.

The appropriate merge is to use ParameterStreamProtocol v4 as the canonical
wire format, retain Tether's stronger internal architecture, and import the
other implementation's static-storage and embedded-adapter ideas. Add Tether's
snapshots, feature exchange, catalog events, thresholds, datalogging, and
struct descriptions as negotiated extensions rather than preserving a second
unused wire format. A direct header merge is still unsafe: message `0x03` and
value type IDs `14` and `15` have incompatible meanings, so the common wire
must be adopted deliberately through an adapter and conformance tests.