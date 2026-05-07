# Tether IO Protocol — Wire Format Specification

**Protocol Version:** 1

## Transport & Framing

All messages are framed using [SLIP (RFC 1055)](https://tools.ietf.org/html/rfc1055):

| Byte | Meaning |
|---|---|
| `0xC0` | END — packet delimiter |
| `0xDB 0xDC` | Escaped END byte (data byte 0xC0) |
| `0xDB 0xDD` | Escaped ESC byte (data byte 0xDB) |

Each SLIP packet contains exactly **one** protocol message. The first byte of the decoded payload is the `MessageType` discriminator.

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
| 14 | Struct | variable | Composite binary struct (layout from DescribeStructResp) |
| 15 | Enum | 4 | Integer with named labels |

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

### 0x04 — ListSignalsResp

Same format as ListParamsResp with MessageType = 0x04.

### 0x05 — GetParamReq

Client reads a single parameter value.

```
Offset  Size  Field
0       1     MessageType = 0x05
1       8     paramId (U64)
```

### 0x06 — GetParamResp

Server responds with the parameter value.

For fixed-size types:
```
Offset  Size  Field
0       1     MessageType = 0x06
1       8     paramId (U64)
9       1     valueSize (U8) — byte size of the value
10      N     value (N = valueSize bytes, little-endian)
```

For variable-length types:
```
Offset  Size  Field
0       1     MessageType = 0x06
1       8     paramId (U64)
9       V     varintSize — varint-encoded byte count
9+V     N     value (N bytes)
```

### 0x07 — SetParamReq

Client writes a parameter value.

```
Offset  Size  Field
0       1     MessageType = 0x07
1       8     paramId (U64)
9       N     value (N = valueTypeSize for fixed types, or varint-prefixed for variable)
```

### 0x08 — SetParamResp

Server acknowledges the write.

```
Offset  Size  Field
0       1     MessageType = 0x08
1       8     paramId (U64)
```

### 0x09 — GetSignalReq

Same format as GetParamReq with MessageType = 0x09.

### 0x0A — GetSignalResp

Same format as GetParamResp with MessageType = 0x0A.

### 0x0B — ConfigureStreamReq

Client configures a stream specification.

```
Offset  Size  Field
0       1     MessageType = 0x0B
1       1     triggerMode (TriggerMode enum)
2       4     intervalUs (U32) — sample interval in microseconds
6       4     chunkSize (U32) — rows per StreamData packet
10      4     skipCount (U32) — rows to skip between transmissions
14      4     entryCount (U32) — number of entry IDs following
18      8×N   entryIds[0..N-1] (U64 each)
```

### 0x0C — ConfigureStreamAck

Server acknowledges stream configuration.

```
Offset  Size  Field
0       1     MessageType = 0x0C
1       4     specId (U32) — unique identifier for this stream spec
5       4     rowSize (U32) — bytes per data row (excluding timestamp)
```

### 0x0D — StartStream

Client starts the configured stream.

```
Offset  Size  Field
0       1     MessageType = 0x0D
```

### 0x0E — StopStream

Client stops the active stream.

```
Offset  Size  Field
0       1     MessageType = 0x0E
```

### 0x0F — StreamData

Server sends a batch of timestamped data rows.

```
Offset  Size  Field
0       1     MessageType = 0x0F
1       4     specId (U32) — stream spec identifier
5       4     rowCount (U32) — number of rows in this packet
9       ...   row[0..rowCount-1]
```

Each row:
```
Offset  Size  Field
0       8     timestamp (U64) — microseconds
8       N     values — concatenated entry values in spec order (N = rowSize)
```

For entries with variable-length values, each value is varint-prefixed within the row.

### 0x10 — Error

Server reports an error.

```
Offset  Size  Field
0       1     MessageType = 0x10
1       4     errorCode (ErrorCode enum, U32)
5       2     msgLen (U16)
7       N     message (UTF-8 string, N = msgLen)
```

Error codes:

| Code | Name | Description |
|---|---|---|
| 0 | None | No error |
| 1 | InvalidMessage | Malformed message |
| 2 | UnknownMessageType | Unrecognized message type byte |
| 3 | InvalidId | Parameter/signal ID not found |
| 4 | StreamNotConfigured | Start/stop without prior configure |
| 5 | AlreadyStreaming | StartStream while already streaming |
| 6 | NotStreaming | StopStream while not streaming |
| 7 | TooManyEntries | Entry count exceeds limit |
| 8 | InternalError | Server internal error |
| 9 | NotWritable | Attempted write to read-only entry |
| 10 | FeatureNotSupported | Requested feature not available |
| 11 | DatalogError | Datalogging configuration error |
| 12 | ThresholdError | Threshold configuration error |

### 0x11 — GetMetadataReq

Client requests metadata for a parameter or signal.

```
Offset  Size  Field
0       1     MessageType = 0x11
1       8     entryId (U64)
```

### 0x12 — GetMetadataResp

Server responds with key/value metadata pairs.

```
Offset  Size  Field
0       1     MessageType = 0x12
1       8     entryId (U64)
9       4     pairCount (U32)
13      ...   pair[0..pairCount-1]
```

Each pair:
```
Offset  Size  Field
0       2     keyLen (U16)
2       K     key (UTF-8)
2+K     2     valueLen (U16)
4+K     V     value (UTF-8)
```

### 0x13 — SnapshotParamsReq

Client requests a bulk snapshot of parameter values.

```
Offset  Size  Field
0       1     MessageType = 0x13
1       4     count (U32) — number of IDs (0 = all params)
5       8×N   paramIds[0..N-1] (U64 each, absent if count = 0)
```

### 0x14 — SnapshotParamsResp

Server responds with all requested parameter values.

```
Offset  Size  Field
0       1     MessageType = 0x14
1       4     count (U32)
5       ...   entry[0..count-1]
```

Each entry:
```
Offset  Size  Field
0       8     paramId (U64)
8       1     valueSize (U8 for fixed; or varint for variable)
9       N     value bytes
```

### 0x15 — SnapshotSignalsReq

Same format as SnapshotParamsReq with MessageType = 0x15.

### 0x16 — SnapshotSignalsResp

Same format as SnapshotParamsResp with MessageType = 0x16.

### 0x17 — FeatureExchangeReq

Client sends its feature set.

```
Offset  Size  Field
0       1     MessageType = 0x17
1       ...   FeatureSet (see below)
```

### 0x18 — FeatureExchangeResp

Server responds with its feature set. Always includes `protocol_version`.

```
Offset  Size  Field
0       1     MessageType = 0x18
1       ...   FeatureSet (see below)
```

#### FeatureSet encoding

```
Offset  Size  Field
0       4     featureCount (U32)
4       ...   feature[0..featureCount-1]
```

Each feature:
```
Offset  Size  Field
0       2     nameLen (U16)
2       N     name (UTF-8)
2+N     1     type (0 = Bool, 1 = U32, 2 = String)
3+N     ...   value (depends on type)
```

Value encoding by type:
- **Bool (0):** 1 byte, 0x00 or 0x01
- **U32 (1):** 4 bytes, little-endian
- **String (2):** [U16 len] [len bytes of UTF-8]

### 0x19 — CatalogChanged

Server pushes this when the parameter/signal catalog has been modified (new entries added, entries removed).

```
Offset  Size  Field
0       1     MessageType = 0x19
1       4     revision (U32) — new catalog revision number
```

### 0x1A — ConfigureDatalogReq

Client configures the datalogging subsystem.

```
Offset  Size  Field
0       1     MessageType = 0x1A
1       ...   DatalogConfig (see below)
```

#### DatalogConfig encoding

```
Offset  Size  Field
0       2     nameLen (U16)
2       N     logName (UTF-8)
2+N     4     sampleRateHz (U32)
6+N     1     enabled (Bool)
7+N     4     entryCount (U32)
11+N    8×C   entryIds[0..C-1] (U64 each)
```

### 0x1B — ConfigureDatalogResp

Server acknowledges datalog configuration.

```
Offset  Size  Field
0       1     MessageType = 0x1B
1       1     success (Bool)
```

### 0x1C — DatalogStatusReq

```
Offset  Size  Field
0       1     MessageType = 0x1C
```

### 0x1D — DatalogStatusResp

```
Offset  Size  Field
0       1     MessageType = 0x1D
1       ...   DatalogStatus (see below)
```

#### DatalogStatus encoding

```
Offset  Size  Field
0       1     state (DatalogState enum)
1       8     recordsWritten (U64)
9       8     bytesWritten (U64)
17      ...   DatalogMetadata
```

#### DatalogMetadata encoding

```
Offset  Size  Field
0       2     nameLen (U16)
2       N     logName (UTF-8)
2+N     4     recordSize (U32)
6+N     4     sampleRateHz (U32)
10+N    4     fieldCount (U32)
14+N    ...   field[0..fieldCount-1]
```

Each field:
```
Offset  Size  Field
0       8     entryId (U64)
8       2     nameLen (U16)
10      N     name (UTF-8)
10+N    1     valueType (ValueType enum)
11+N    4     offset (U32) — byte offset within record
15+N    4     size (U32) — byte size of field
19+N    1     kind (EntryKind enum)
```

### 0x1E — ConfigureThresholdReq

Client configures the threshold filter for streaming.

```
Offset  Size  Field
0       1     MessageType = 0x1E
1       ...   ThresholdConfig (see below)
```

#### ThresholdConfig encoding

```
Offset  Size  Field
0       2     nameLen (U16)
2       N     name (UTF-8)
2+N     1     isWhitelist (Bool)
3+N     4     ruleCount (U32)
7+N     ...   rule[0..ruleCount-1]
```

#### ThresholdRule encoding

```
Offset  Size  Field
0       8     entryId (U64)
8       1     type (ThresholdType enum)
9       8     threshold (F64)
17      2     customNameLen (U16)
19      C     customName (UTF-8, C = customNameLen)
19+C    4     customConfigCount (U32)
23+C    ...   configPrimitive[0..count-1]
```

#### ConfigPrimitive encoding

```
Offset  Size  Field
0       1     tag (0 = Float, 1 = String)
```

If tag = 0 (Float):
```
1       8     value (F64)
```

If tag = 1 (String):
```
1       2     len (U16)
3       N     value (UTF-8, N = len)
```

### 0x1F — ConfigureThresholdResp

```
Offset  Size  Field
0       1     MessageType = 0x1F
1       1     success (Bool)
```

### 0x20 — DescribeStructReq

Client requests the field layout of a struct-typed entry.

```
Offset  Size  Field
0       1     MessageType = 0x20
1       8     entryId (U64)
```

### 0x21 — DescribeStructResp

Server responds with the struct descriptor.

```
Offset  Size  Field
0       1     MessageType = 0x21
1       ...   StructDescriptor (see below)
```

#### StructDescriptor encoding

```
Offset  Size  Field
0       8     entryId (U64)
8       2     nameLen (U16)
10      N     name (UTF-8)
10+N    4     totalSize (U32)
14+N    4     fieldCount (U32)
18+N    ...   field[0..fieldCount-1]
```

Each struct field:
```
Offset  Size  Field
0       2     nameLen (U16)
2       N     name (UTF-8)
2+N     1     valueType (ValueType enum)
3+N     4     offset (U32) — byte offset within struct
7+N     4     size (U32) — byte size
11+N    2     unitLen (U16)
13+N    U     unit (UTF-8, U = unitLen)
```

---

## Datalog binary record format

Each binary record written by the datalogging subsystem:

```
Offset  Size  Field
0       8     timestamp (U64) — microseconds
8       ...   field values in configured order (fixed-size, no length prefixes)
```

Total record size = 8 + sum of field sizes (from DatalogMetadata).
