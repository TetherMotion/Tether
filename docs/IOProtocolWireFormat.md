# Tether IO Protocol — Wire Contract

**Protocol Version:** 5

Version 5 is the direct merged Tether/ParameterStream contract. It adopts
ParameterStream message IDs `0x01`–`0x13`, retains Tether extensions
`0x20`–`0x34`, and intentionally widens protocol counts to `uint32_t`.
Compatibility with the unused Tether v1 wire ABI is not required.

## Transport & Framing

All messages are framed using [SLIP (RFC 1055)](https://tools.ietf.org/html/rfc1055):

| Byte | Meaning |
|---|---|
| `0xC0` | END — packet delimiter |
| `0xDB 0xDC` | Escaped END byte (data byte 0xC0) |
| `0xDB 0xDD` | Escaped ESC byte (data byte 0xDB) |

Each SLIP packet contains exactly **one** protocol message. The first byte of the decoded payload is the `MessageType` discriminator.

Malformed escape sequences and packets larger than the implementation maximum
are discarded. Literal `0xC0` and `0xDB` are escaped as shown above.

### Receive-buffer ownership and limits

`Session` uses separate bounded buffers for the encoded SLIP accumulator and
the decoded message. Applications may provide a fresh `IReceiveBuffer` from
`ServerConfig::encodedBufferFactory` and `decodedBufferFactory` for every
accepted connection. `StaticReceiveBuffer<N>` never allocates; users should
choose `N` large enough for the largest encoded frame. `DynamicReceiveBuffer`
starts at an initial capacity and grows geometrically, but never beyond its
configured maximum. The default dynamic limits are an 8 KiB initial capacity,
`MAX_ENCODED_MESSAGE_SIZE` for encoded data, and `MAX_MESSAGE_SIZE` for decoded
data. Overflow discards the current frame until the next SLIP END delimiter.

### Drogon binary WebSocket example

When Drogon is available, the `tether_io_drogon_websocket` example exposes
`ws://127.0.0.1:8080/tether-io`. WebSocket binary messages are treated as
arbitrary transport chunks, queued into an `ITransport`, and processed by the
same `Session` used by TCP and serial transports. Responses are sent as binary
WebSocket messages containing SLIP-framed protocol packets. Build with
`cmake -S . -B build` followed by `cmake --build build --target
tether_io_drogon_websocket`, then run `build/bin/tether_io_drogon_websocket`.

## Byte order

All multi-byte integers are **little-endian**.

## Value types

| ID | Name | Size (bytes) | Description |
|---|---|---|---|
| 1 | U8 | 1 | Unsigned 8-bit integer |
| 2 | U16 | 2 | Unsigned 16-bit integer |
| 3 | U32 | 4 | Unsigned 32-bit integer |
| 4 | U64 | 8 | Unsigned 64-bit integer |
| 5 | I8 | 1 | Signed 8-bit integer |
| 6 | I16 | 2 | Signed 16-bit integer |
| 7 | I32 | 4 | Signed 32-bit integer |
| 8 | I64 | 8 | Signed 64-bit integer |
| 9 | F32 | 4 | IEEE 754 single-precision float |
| 10 | F64 | 8 | IEEE 754 double-precision float |
| 11 | Bool | 1 | 0 = false, nonzero = true |
| 12 | String | variable | Length-prefixed UTF-8 string (varint length + bytes) |
| 13 | Binary | variable | Length-prefixed raw bytes (varint length + bytes) |
| 14 | IPv4 | 4 | IPv4 address bytes |
| 15 | IPv6 | 16 | IPv6 address bytes |
| 16 | MAC | 6 | MAC address bytes |
| 17 | Enum | variable | Named enum value, varint encoded |
| 18 | UVarint | variable | Unsigned varint |
| 19 | IVarint | variable | Zigzag signed varint |
| 20 | Struct | variable | Composite binary struct |
| 21 | Array | variable | U32 count followed by U32-length-delimited element payloads |
| 22 | Stream | 4 | U32 input-stream handle |

### Variable-length encoding

Variable-length values (String, Binary) are encoded as:
```
[varint length] [length bytes of data]
```

## Varint encoding

Protobuf-style variable-length unsigned 32-bit integer:
- 7 bits per byte, MSB set if more bytes follow
- LSB first (little-endian byte order)
- Maximum 5 bytes (for values up to 2^32 - 1)
- A fifth byte may contain only payload bit 0x0f; other encodings are rejected.

The implementation limits messages and variable values to 1 MiB and catalog,
stream, metadata, and filter counts to 1,000,000.

```
Value 0-127:    [0xxxxxxx]
Value 128-16383: [1xxxxxxx] [0xxxxxxx]
...etc
```

## String encoding

Strings in message fields (names, descriptions, etc.) are encoded as:
```
[U16 length] [length bytes of UTF-8 data]
```

Note: this is distinct from Value encoding which uses varint-prefixed strings.

## Entry flags

Bitfield (U8):

| Bit | Name | Description |
|---|---|---|
| 0 | Readable | Entry can be read |
| 1 | Writable | Entry can be written (parameters only) |
| 2 | VariableLen | Value is variable-length |
| 3 | HasStruct | Entry has a struct descriptor |
| 4 | HasEnum | Entry has enum label metadata |

---

## Messages

### 0x01 — ListParamsReq

Client requests a page of the parameter catalog.

```
Offset  Size  Field
0       1     MessageType = 0x01
1       4     offset (U32) — starting index in catalog
5       4     maxCount (U32) — max entries to return
```

### 0x02 — ListParamsResp

Server responds with a page of parameter descriptors.

```
Offset  Size  Field
0       1     MessageType = 0x02
1       4     totalCount (U32) — total params in catalog
5       4     offset (U32) — starting index of this page
9       4     count (U32) — number of entries in this page
13      ...   entry[0..count-1]
```

Each entry:
```
Offset  Size  Field
0       8     id (U64)
8       1     valueType (ValueType enum)
9       1     flags (EntryFlags bitfield)
10      2     nameLen (U16)
12      N     name (UTF-8 bytes, N = nameLen)
12+N    2     descLen (U16)
14+N    M     description (UTF-8 bytes, M = descLen)
14+N+M  2     groupLen (U16)
16+N+M  G     group (UTF-8 bytes, G = groupLen)
```

### 0x03 — ListSignalsReq

Same format as ListParamsReq with MessageType = 0x03.

### 0x03 — ConfigureStream

`0x03` is the ParameterStream ConfigureStream request. Tether's stream layout
uses `[trigger U8][interval_ms U32][chunk U32][skip U32][trigger_id U64]`
`[entry_count U32][entry IDs U64...][filter_count U32]`, followed by filter
properties. Each filter property is `[name length U8][name bytes][value type U8]`
`[value]`. Filters are schema-validated and applied to rows; unknown,
unsupported, wrong-type, malformed, out-of-range, or trailing properties are
rejected.

### 0x04 — ConfigureAck

The response is `[type][spec_id U32][resolved_count U32][row_size U32]`,
followed by resolved `[entry_id U64][value type U8][value size U8]` descriptors.

### 0x05 — StartStream

### 0x06 — StopStream

### 0x07 — StreamData

The response is `[type][spec_id U32][row_count U32]`, followed by timestamped
rows. Timestamps are U64 microseconds and variable values are length-prefixed.

### 0x09 — GetMetadataReq

### 0x0A — GetMetadataResp

### 0x0B — SetParameterReq

### 0x0C — SetParameterResp

### 0x0D — PingReq / 0x0E — PongResp

### 0x0F — SubscribeLogReq / 0x10 — SubscribeLogResp

### 0x11 — UnsubscribeLogReq / 0x12 — UnsubscribeLogResp

### 0x13 — LogData

Log subscriptions use minimum severity plus component, message, and location
U16 string filters. LogData contains timestamp, severity, and those same three
strings. Platform logger levels map Error/Warn/Info/Debug/Verbose to
Error/Warn/Info/Debug/Trace. Delivery is asynchronous and does not replace the
application's primary logger handler.

### 0x20–0x34 — Tether extensions

Tether extensions provide separate signal catalog operations, direct parameter
and signal reads, snapshots, feature exchange, catalog notifications,
datalogging, threshold configuration, and struct descriptions. `ListParams`
always contains parameters only; signals are never merged into that response.

Function RPC extensions are `0x35` ListFunctionsReq, `0x36` ListFunctionsResp,
`0x37` CallFunctionReq, and `0x38` CallFunctionResp.

### Recursive function value descriptors

Function parameters and returns may carry a recursive descriptor. A descriptor
is encoded as a type byte. An `Array` descriptor is followed by one element
descriptor. A `Struct` descriptor is followed by a U32 field count and ordered
fields, each encoded as `[name U16][name bytes][descriptor]`. Descriptor names
are annotations; struct payloads use the corresponding field positions.

Aggregate payloads are encoded as follows:

* `Array`: `[count U32]`, then `count` repetitions of `[length U32][payload]`.
* `Struct`: `[field_count U32]`, then positional TLVs of
	`[position U32][value_type U8][length U32][payload]`. Each declared field
	occurs exactly once, and positions may be sent in any order.

Nested descriptors and payloads are bounded by the implementation's aggregate
depth, field, element, message, and value-size limits. Scalar fixed-size values
must have their exact size; varints must be canonical bounded varints.
The descriptor is advertised in `ListFunctionsResp` as a presence byte,
descriptor length U32, and descriptor bytes after each value's maximum-size
field. Existing scalar clients can ignore the descriptor when its presence byte
is zero.

## Function RPC

`ListFunctionsReq` is `[type][offset U32][max_count U32]`. Its response is
`[type][total U32][offset U32][count U32]`, followed by function descriptors.
Each descriptor contains an ID, name, description, group, ordered parameter
annotations, an optional return annotation, and function metadata. Each
parameter annotation contains its name, description, type, flags, optional enum
and struct references, maximum value size, optional default value, and metadata.

`CallFunctionReq` is `[type][function_id U64][argument_count U32]` followed by
bounded TLV tuples. Every tuple is `[position U32][value_type U8][length U32]`
`[value bytes]`. Names are descriptive; calls use positions. The server validates
position, type, length, uniqueness, and maximum size before invoking a callback.
Missing optional arguments are filled with their annotated defaults. Optional
arguments must be a suffix of the parameter list.

`CallFunctionResp` contains the function ID, status, error code and error string;
successful responses additionally carry a return-value TLV tuple at position 0.

### Input streams

`CreateInputStreamReq` (`0x39`) is encoded as
`[max_value_size U32][max_batch_size U32][value descriptor]`. The server returns
`CreateInputStreamResp` (`0x3A`) as `[stream_id U32][status U8]`, where status
zero means success. The stream ID is a handle and may be passed as a `Stream`
function argument or returned as a `Stream` function value; streamed values do
not use function TLVs.

`InputStreamData` (`0x3B`) is
`[stream_id U32][count U32]` followed by `count` repetitions of
`[value_length U32][value payload]`. Every value is validated against the
stream's descriptor and configured size/count limits before delivery to the
application callback. `CloseInputStreamReq` (`0x3C`) is `[stream_id U32]` and
its response (`0x3D`) has the same `[stream_id U32][status U8]` body. Unknown
or closed stream IDs, malformed batches, invalid values, and trailing bytes
are rejected with an Error message.
