# Tether IO Protocol — Message Catalog

Complete reference for every `MessageType` in protocol version 6. For
the bootstrap handshake, schema wire forms, and value encoding see
[IOProtocolWireFormat.md](IOProtocolWireFormat.md); for API usage see
[IOProtocolClient.md](IOProtocolClient.md) and
[IOProtocolServer.md](IOProtocolServer.md).

Conventions below:

- `U8/U16/U32/U64` — little-endian fixed-width integers.
- `varuint` — canonical base-128 LSB-first varint (≤ 10 bytes).
- `str16` — `[byte_length U16][bytes]` (catalog strings; distinct from
  schema `String` values).
- `descriptor_string` — `[byte_length U32][bytes]` (schema metadata).
- `value` — schema-encoded payload when a schema slot applies, else the
  flat `ValueType` encoding (fixed-width scalars little-endian;
  variable types length-prefixed).
- Every message starts with its one-byte `MessageType` discriminator,
  omitted from the layouts below.

## V4 catalog and parameter messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x01` | `ListParamsReq` | C→S | Paged catalog read |
| `0x02` | `ListParamsResp` | S→C | Parameter descriptors |
| `0x0B` | `SetParameterReq` | C→S | Write a parameter |
| `0x0C` | `SetParameterResp` | S→C | Write acknowledgement |
| `0x0D` | `PingReq` | C→S | Liveness check |
| `0x0E` | `PongResp` | S→C | Liveness reply |
| `0x09` | `GetMetadataReq` | C→S | Read entry metadata |
| `0x0A` | `GetMetadataResp` | S→C | Metadata key/value pairs |

```
ListParamsReq   := offset U32 max_count U32
ListParamsResp  := total U32 offset U32 count U32 schema_epoch U64
                   * param_descriptor
param_descriptor := id U64
                    entry_type
                    flags U32
                    name str16 description str16 group str16

// entry_type — depends on negotiated ENCODING_FEATURE_TYPED_DESCRIPTORS:
//   negotiated:   type_tag U8        // 0 = Schema, 1 = Flat
//                 schema_slot U32    // type_tag=0: authoritative; else 0
//                 value_type U8      // type_tag=1: ValueType; else 0
//                 element_type U8    // type_tag=1 & Array: elem ValueType; else 0
//                 value_size U8      // type_tag=1: canonical bytes; else 0
//                 max_value_size U16 // type_tag=1: var-len cap; else 0
//   legacy V6:    schema_slot U32    // (flat entries are indistinguishable
//                                    //  from manifest slot 0)

GetParamReq     := id U64
GetParamResp    := id U64 value_length varuint value[value_length]

SetParameterReq := id U64 value_length varuint value[value_length]
SetParameterResp:= id U64                // no status — errors use Error
```

`type_tag` is a discriminated union over how the entry's type is
described: `Schema`(0) — `schema_slot` resolves in the negotiated
manifest/graph; `Flat`(1) — the entry carries a plain `ValueType` and
`value_type`/`element_type`/`value_size`/`max_value_size` describe the
encoding. Flat values use the flat encodings — fixed-width
little-endian scalars, `varuint`-length-prefixed variable values —
regardless of whether the session negotiated a schema manifest.
The inapplicable half of the union is always 0 on the wire, so
`schema_slot == 0` unambiguously means "no schema".

The full `ValueType` set usable in flat descriptors:

| Tag | Name | Encoding |
|----:|---|---|
| 1–4 | `U8`/`U16`/`U32`/`U64` | fixed-width LE uint |
| 5–8 | `I8`/`I16`/`I32`/`I64` | fixed-width LE int |
| 9–10 | `F32`/`F64` | fixed-width LE IEEE-754 |
| 11 | `Bool` | 1 byte, 0/1 |
| 12 | `String` | `varuint` len + UTF-8 bytes |
| 13 | `Binary` | `varuint` len + raw bytes |
| 14 | `IPv4` | 4 bytes |
| 15 | `IPv6` | 16 bytes |
| 16 | `MAC` | 6 bytes |
| 17 | `Enum` | `varuint` ordinal (max_value_size = max ordinal bytes) |
| 18 | `UVarint` | protobuf-style varuint |
| 19 | `IVarint` | zigzag varint |
| 20 | `Struct` | `varuint` len + opaque bytes |
| 21 | `Array` | `varuint` count + element_type-encoded values |
| 22 | `FixedString` | `varuint` len + bytes (cap in max_value_size) |
| 23 | `Stream` | `varuint` len + chunked bytes |

`value_size` is the canonical byte size for fixed-width types (1–11,
14–16); it is 0 for the variable-length types (12, 13, 17–23), whose
per-value byte length travels with the value itself. `max_value_size`
carries the byte cap for variable-length entries. `element_type` is
only meaningful for `Array` (element ValueType, including nested
`Array`); it is 0 (`None`) otherwise.

`setParam` and `setParamVar` emit the same message; the server
validates `value_length` against the entry (exact `valueSize` for
fixed-size entries, `<= maxValueSize` for variable-length) and
validates the payload against the catalog schema when one is
installed.

```
PingReq         := nonce varint
PongResp        := nonce varint

GetMetadataReq  := id U64
GetMetadataResp := id U64 count U32 * (key str16 value str16)
```

Metadata applies to params and signals alike. Unknown ids produce
`Error(InvalidId)`.

## Streaming messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x03` | `ConfigureStream` | C→S | Install a stream spec |
| `0x04` | `ConfigureAck` | S→C | Resolved layout (epoch + slots) |
| `0x05` | `StartStream` | C→S | Begin delivery |
| `0x06` | `StopStream` | C→S | End delivery |
| `0x07` | `StreamData` | S→C | Batched sample rows |
| `0x31` | `ConfigureThresholdReq` | C→S | Install threshold filter |
| `0x32` | `ConfigureThresholdResp` | S→C | Filter acknowledgement |

```
ConfigureStream := trigger_mode U8 interval_ms U32 chunk_size U32
                   skip_count U32 trigger_entry_id U64
                   entry_count U32 * (entry_id U64)
                   filter_count U32 * stream_filter
stream_filter   := name_length U8 name[name_length]
                   schema_epoch U32 schema_slot U32
                   value_length varint32 value[value_length]

ConfigureAck    := spec_id U32 resolved_count U32 row_size U32
                   schema_epoch U64
                   * stream_layout_entry

// stream_layout_entry — negotiated (16 bytes):
//     entry_id U64 type_tag U8 schema_slot U32
//     value_type U8 element_type U8 value_size U8
//   legacy V6 (13 bytes):
//     entry_id U64 schema_slot U32 value_size U8

StartStream/StopStream := (empty body)     // one stream per session

StreamData      := spec_id U32 row_count U32
                   * (timestamp_us U64 * row_fields)
```

Each `row_fields` element follows `ConfigureAck` order. When the ack's
`value_size` is nonzero the field is exactly that many bytes; a zero
`value_size` marks a variable-length field encoded as
`[length varuint][value]`. `trigger_mode` selects `Time` or `OnChange`;
`trigger_entry_id` restricts OnChange to one entry (0 = any).
`skip_count` decimates: N skipped samples per transmitted sample.
`chunk_size` caps rows per `StreamData` packet.

Threshold filter wire form (see
[IOProtocolStreaming.md](IOProtocolStreaming.md)):

```
ConfigureThresholdReq := name str16 is_whitelist U8 rule_count U32
                         * threshold_rule
threshold_rule := entry_id U64 threshold_type U8 threshold F64
                  custom_name str16 config_count U32
                  * (name str16 type U8 value_length U32 value)
ConfigureThresholdResp := status U8   // 0 = success
```

`threshold_type` is `None`(0), `Absolute`(1), `Relative`(2),
`Custom`(3). `entry_id == 0` is the default rule.

## Signal, snapshot, and extension catalog messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x20` | `ListSignalsReq` | C→S | Paged signal catalog read |
| `0x21` | `ListSignalsResp` | S→C | Signal descriptors |
| `0x24` | `GetSignalReq` | C→S | Latest signal value |
| `0x25` | `GetSignalResp` | S→C | Signal value |
| `0x22` | `GetParamReq` | C→S | Latest param value |
| `0x23` | `GetParamResp` | S→C | Param value |
| `0x26` | `SnapshotParamsReq` | C→S | Atomic multi-param read |
| `0x27` | `SnapshotParamsResp` | S→C | Timestamped param values |
| `0x28` | `SnapshotSignalsReq` | C→S | Atomic multi-signal read |
| `0x29` | `SnapshotSignalsResp` | S→C | Timestamped signal values |
| `0x2C` | `CatalogChanged` | S→C | Push: catalog mutated |
| `0x2D` | `ConfigureDatalogReq` | C→S | Arm the recorder |
| `0x2E` | `ConfigureDatalogResp` | S→C | Recorder acknowledgement |
| `0x2F` | `DatalogStatusReq` | C→S | Read recorder state |
| `0x30` | `DatalogStatusResp` | S→C | Recorder state |

```
ListSignalsReq   := offset U32 max_count U32
ListSignalsResp  := total U32 offset U32 count U32 schema_epoch U64
                    * signal_descriptor   // same shape as param_descriptor

GetSignalReq     := id U64
GetSignalResp    := id U64 value_length varuint value[value_length]

SnapshotParamsReq  := count U32 * (id U64)     // count 0 = all params
SnapshotParamsResp := timestamp_us U64 count U32 schema_epoch U64
                      * snapshot_value
// snapshot_value — negotiated:
//     id U64 type_tag U8 schema_slot U32
//     value_type U8 element_type U8 value_size U8 value
//   legacy V6:
//     id U64 schema_slot U32 value_size U8 value
// value_size 0 = variable: value is [length varuint][bytes]
SnapshotSignalsReq/Resp — same layout over signals.

CatalogChanged   := (empty body)         // re-list to observe changes
```

`CatalogChanged` fires on each registered session whenever the
registry's catalog mutates. Clients should re-run `listParams` /
`listSignals` / `listFunctions`; entry ids and schema epochs may have
changed.

```
ConfigureDatalogReq  := log_name str16 sample_rate_hz U32 enabled U8
                        entry_count U32 * (entry_id U64)  // 0 = all
ConfigureDatalogResp := status U8                          // 0 = success

DatalogStatusReq     := (empty body)
DatalogStatusResp    := state U8 records_written U64 bytes_written U64
                        metadata
```

`state` is `Idle`(0), `Recording`(1), `Stopped`(2), `Error`(3). The
request arms or disarms the shared `DatalogRecorder`; record delivery
itself is application-side (the sink is configured by the embedding
application, not over the wire — see
[IOProtocolStreaming.md](IOProtocolStreaming.md#datalogging)).

## Log subscription messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x0F` | `SubscribeLogReq` | C→S | Open filtered log feed |
| `0x10` | `SubscribeLogResp` | S→C | Subscription id or failure |
| `0x11` | `UnsubscribeLogReq` | C→S | Close a feed |
| `0x12` | `UnsubscribeLogResp` | S→C | Close acknowledgement |
| `0x13` | `LogData` | S→C | One log record |

```
SubscribeLogReq   := min_severity U8 component_filter str16
                     message_filter str16 location_filter str16
SubscribeLogResp  := subscription_id varuint failed U8 [error str16]
UnsubscribeLogReq := subscription_id varuint
UnsubscribeLogResp:= subscription_id varuint not_found U8

LogData           := timestamp_us U64 severity U8 component str16
                     message str16 location str16
```

Filters are substring matches; an empty string disables the
dimension. Severity passes when `record >= min_severity`. Server-side
records come from `Server::publishLog` / the session log callback.

## Function messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x35` | `ListFunctionsReq` | C→S | Paged function catalog read |
| `0x36` | `ListFunctionsResp` | S→C | Function descriptors |
| `0x37` | `CallFunctionReq` | C→S | Classic serialized call |
| `0x38` | `CallFunctionResp` | S→C | Call result |
| `0x3E` | `InvokeExReq` | both | Correlated call + deadline |
| `0x3F` | `InvokeExResp` | both | Correlated result |
| `0x40` | `RegisterFunctionsReq` | C→S | Publish peer catalog |
| `0x41` | `RegisterFunctionsResp` | S→C | Registration ack |

```
ListFunctionsReq  := offset U32 max_count U32
ListFunctionsResp := total U32 offset U32 count U32 schema_epoch U64
                     * function_descriptor
function_descriptor := id U64 name str16 description str16 group str16
    param_count U32
    * (name str16 description str16 key U32 schema_slot U32 flags U32
       type U8 [default_length U32 default_value] metadata)
    has_return U8 [return_name str16 return_description str16
                   return_schema_slot U32 return_flags U32 metadata]
    metadata
metadata := count U32 * (key str16 value str16)
```

Arguments and returns are `keyed_value` sequences. A call encodes its
arguments as `[count U32]` followed by
`[key varint32][length varint32][value]`; required positions use
`key = position + 1` unless the descriptor assigns an explicit nonzero
`key`. Omitted optional arguments are materialized from their declared
defaults server-side.

```
CallFunctionReq  := function_id U64 arg_count U32 * keyed_value
CallFunctionResp := function_id U64 status U8
                    [error_code U32 error_message str16]   // on status != 0
                    reserved U32 reserved U16               // on success
                    has_return U8 [value_length U32 value]
```

`status == 0` is success (matching `InvokeExResp`).

```
InvokeExReq  := request_id U64 function_id U64 deadline_us U64
                arg_count U32 * keyed_value
InvokeExResp := request_id U64 function_id U64 status U8
                [error_code U32 error_message str16]
                has_return U8 [return_type U8 value_length U32 value]
```

`InvokeExReq` is the only message legal in both directions: a client
invokes server functions; a server invokes functions a client
published via `RegisterFunctionsReq`. `deadline_us` is a relative
budget enforced by the initiator's sweep machinery; expiry yields a
locally synthesized `InvokeResult{timedOut=true}` — the callee is not
cancelled. `request_id` correlates the pair, so calls pipeline freely.

```
RegisterFunctionsReq  := count U32 * function_descriptor
RegisterFunctionsResp := count U32 status U8       // 0 = accepted
```

The descriptor shape matches `ListFunctionsResp`. Registration
replaces the session's whole peer catalog; the catalog dies with the
connection.

## Input-stream messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x39` | `CreateInputStreamReq` | C→S | Open client→server stream |
| `0x3A` | `CreateInputStreamResp` | S→C | Stream id or refusal |
| `0x3B` | `InputStreamData` | C→S | Batched inbound values |
| `0x3C` | `CloseInputStreamReq` | C→S | Close the stream |
| `0x3D` | `CloseInputStreamResp` | S→C | Close acknowledgement |

```
CreateInputStreamReq  := schema_epoch U32 schema_slot U32
                         max_value_size U32 max_batch U32
CreateInputStreamResp := stream_id U32 status U8        // 0 = accepted

InputStreamData       := stream_id U32 count U32
                         * (value_length varuint value)
CloseInputStreamReq   := stream_id U32
CloseInputStreamResp  := stream_id U32 status U8        // 0 = closed
```

The schema slot must resolve in the committed epoch; the server-side
`inputStreamCreateFn` may refuse (nonzero status). Every delivered
value must fit `max_value_size` and validate against the stream schema
before it reaches `inputStreamDataFn`.

## V6 bootstrap messages

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x50` | `ClientHello` | C→S | Version range, limits, cached manifest |
| `0x51` | `ServerHello` | S→C | Selected version, epoch + manifest |
| `0x52` | `SchemaRequest` | C→S | Ask for missing definitions |
| `0x53` | `SchemaDefinition` | S→C | One schema node |
| `0x54` | `SchemaCommit` | C→S | Commit the advertised epoch |
| `0x55` | `SchemaReject` | S→C | Negotiation failure |
| `0x56` | `SchemaUpdate` | S→C | Push a new epoch + manifest |

Payloads are specified in
[IOProtocolWireFormat.md](IOProtocolWireFormat.md#mandatory-bootstrap).
`SchemaUpdate` mirrors `ServerHello`'s epoch+manifest tail; it is sent
to committed sessions when the server installs a new catalog epoch.

## Error message

| ID | Message | Direction | Purpose |
|---:|---|---|---|
| `0x08` | `Error` | S→C | Request-level or protocol failure |

```
Error := code U32 message str16
```

`ErrorCode` values (`Protocol.hpp`): `InvalidMessage`(1),
`UnknownMessageType`(2), `InvalidId`(3), `StreamNotConfigured`(4),
`AlreadyStreaming`(5), `NotStreaming`(6), `TooManyEntries`(7),
`InternalError`(8), `NotWritable`(9), `FeatureNotSupported`(10),
`DatalogError`(11), `ThresholdError`(12),
`FunctionInvocationError`(13), `ResourceBusy`(14), `Timeout`(15 —
locally synthesized on InvokeEx deadline expiry; never on the wire).

Status-byte convention is uniform: **every response status byte uses
`0 = success`** — `InvokeExResp`, `CallFunctionResp`,
`InputStreamCreateResp`, `InputStreamCloseResp`,
`ConfigureDatalogResp`, `ConfigureThresholdResp`. Failure detail for
these arrives as an `Error` message or a nonzero status; the
`failed`/`not_found`/`has_return_value` flag bytes elsewhere are
booleans, not statuses (`0` = good/false throughout).
