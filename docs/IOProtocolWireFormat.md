# Tether IO Protocol - Wire Contract

The typed CiA 402 application profile built on these primitives is documented
in [IOProtocolMachineProfile.md](IOProtocolMachineProfile.md).

**Protocol Version:** 6

V6 is schema negotiated. A value is interpreted only through a committed
schema catalog, not through a raw `ValueType`, a C++ object layout, or an
application-defined discriminator.

## Transport and framing

All fixed-width multi-byte integers are little-endian. Every message begins
with its one-byte `MessageType` discriminator.

Byte-stream transports use [SLIP (RFC 1055)](https://tools.ietf.org/html/rfc1055):

| Bytes | Meaning |
| --- | --- |
| `0xC0` | Packet delimiter (`END`) |
| `0xDB 0xDC` | Escaped `0xC0` |
| `0xDB 0xDD` | Escaped `0xDB` |

Each decoded SLIP packet contains exactly one message. A malformed escape or
buffer overflow discards the current frame through its next `END` delimiter.
Message-oriented transports may carry the same unframed message bytes.

`Session` owns bounded encoded and decoded receive buffers. Dynamic buffers
start at 8 KiB and do not grow beyond `MAX_ENCODED_MESSAGE_SIZE` and
`MAX_MESSAGE_SIZE`, respectively.

## Mandatory bootstrap

Before sending application traffic, a client must:

1. Send `ClientHello` (`0x50`).
2. Receive `ServerHello` (`0x51`) with the selected version, limits, schema
   epoch, and manifest.
3. Request any missing definitions using `SchemaRequest` (`0x52`) and receive
   one `SchemaDefinition` (`0x53`) for each requested schema.
4. Validate the definitions and send `SchemaCommit` (`0x54`) for that epoch.

Before commit, bootstrap traffic is limited to `ClientHello`, `ServerHello`,
`SchemaRequest`, `SchemaDefinition`, `SchemaCommit`, and `SchemaReject` in
their defined directions. Other inbound application messages are rejected with
`Error(InvalidMessage, "Schema negotiation is not committed")`.

```
ClientHello := min_version U8 max_version U8 encoding_features U32 limits
               cached_manifest
ServerHello := selected_version U8 encoding_features U32 limits epoch U32 manifest

limits := max_message_bytes U32 max_descriptor_bytes U32 max_definitions U32
          max_value_bytes U32 max_value_depth U32 max_collection_entries U32
manifest := count U32 * (schema_key[16] schema_digest[32] revision U32)

SchemaRequest    := epoch U32 count U32 * schema_ref
SchemaDefinition := epoch U32 schema_node
SchemaCommit     := epoch U32
SchemaReject     := code U8 message str16
SchemaUpdate     := epoch U32 manifest
schema_ref       := schema_key[16] schema_digest[32]
```

The current V6 implementation uses encoding features zero and defaults to a
1 MiB message/value limit, 64 KiB descriptor limit, 1,024 definitions, value
depth 32, and 1,000,000 collection entries. A server supports V6 only. If V6
is not in the client range, it sends `SchemaReject(UnsupportedVersion)`.

`SchemaReject` codes are `UnsupportedVersion` (1), `LimitMismatch` (2),
`InvalidManifest` (3), `InvalidDefinition` (4), `MissingDependency` (5), and
`DigestMismatch` (6). Invalid bootstrap payloads and stale bootstrap epochs
produce `Error(InvalidMessage)`.

## Catalog epochs and slots

A `SchemaCatalog` installs a validated graph and manifest as a single epoch.
Every manifest reference must identify a graph node with the matching revision
and descriptor digest; missing nodes, duplicate entries, and digest mismatches
are rejected. Each successful installation increments the nonzero epoch.

Slots are assigned by ascending schema key, starting at zero. A slot is valid
only in its catalog epoch; resolving a stale epoch or an out-of-range slot
fails. The server advertises the sorted manifest in `ServerHello` and may
provide requested definitions from its graph before the client commits.

A `SchemaRef` carries a 16-byte key and a 32-byte digest. A slot is merely its
compact, epoch-bound catalog reference; it is never a globally stable type ID.

## Schema definition wire form

Schema metadata strings use `[length U32][bytes]`; this is distinct from a
schema `String` value.

```
schema_node := key[16] revision U32 kind U8 flags U32
               name descriptor_string description descriptor_string
               annotation_count U32 * (descriptor_string descriptor_string)
               scalar_type U8 max_bytes U32 fixed_count U32
               min_count U32 max_count U32 struct_encoding U8
               has_element U8 [schema_ref]
               has_map_key U8 [schema_ref]
               has_map_value U8 [schema_ref]
               has_target U8 [schema_ref]
               field_count U32 * field
               oneof_count U32 * (member_key U32 schema_ref)

field := key U32 flags U32 presence U8 schema_ref
         name descriptor_string description descriptor_string
         restriction_count U32 * (kind U8 payload_length U32 payload)
         default_length U32 default_value
```

Presence markers are exactly zero or one. Fields and `OneOf` members have
strictly increasing nonzero keys. A graph must be dependency-complete,
acyclic, and within the configured limits. Packed structs allow only required
fields without defaults. Map keys may be strings, bytes, enums, or
non-floating scalar schemas.

Kinds are `Scalar`, `String`, `Bytes`, `Enum`, `Struct`, `FixedArray`,
`DynamicArray`, `Optional`, `Map`, `OneOf`, and `Alias`. An alias encodes its
target exactly. Scalars and enums use their `ValueType` representation.

## Schema value encoding

V6 schema lengths and counts use canonical 64-bit LSB-first base-128
`varuint`s. They use at most ten bytes, byte ten may contain only bit zero,
and nonminimal encodings are invalid.

* `String`: UTF-8 bytes followed by a zero terminator. Embedded zeroes are
  invalid; there is no length prefix.
* `Bytes`: `[byte_length varuint][bytes]`.
* Fixed scalars: their fixed-width little-endian representation. `UVarint`,
  `IVarint`, and `Enum` use `varuint`.
* Packed struct: concatenated required field values in declared key order.
* Tagged struct: `[field_count varuint]`, then sorted unique
  `[field_key varuint][payload_length varuint][payload]` values. Each required
  field occurs once; optional fields may be omitted.
* Fixed array: exactly `fixed_count` consecutive element values.
* Dynamic array: `[count varuint]`, then
  `[element_length varuint][element_payload]` for each element. The count
  satisfies the schema bounds.
* Map: `[count varuint]`, then
  `[key_length varuint][key_payload][value_length varuint][value_payload]`.
  Keys are valid map keys and strictly increasing in canonical key order.
* Optional: `[present U8]`, where zero has no payload and one is followed by
  the element value.
* OneOf: `[member_key varuint][payload_length varuint][payload]`; the nonzero
  member key must be declared by the schema.

Lengths are bounded by the value-byte limit and remaining message bytes.
Validation also enforces recursion depth, collection bounds, and exact
consumption of each nested payload.

## Schema-slot application behavior

The retained parameter, signal, stream, function, and input-stream message
IDs are usable only after V6 commit. Application values must validate against
their catalog schema.

`ConfigureAck` identifies its layout with an epoch and slots:

```
[type][spec_id U32][resolved_count U32][row_size U32][schema_epoch U64]
  * [entry_id U64][schema_slot U32][value_size U8]
```

Each configured stream filter is:

```
[name_length U8][name bytes][schema_epoch U32][schema_slot U32]
[value_length varint32][value bytes]
```

Its slot must resolve in the supplied epoch, its value must validate against
the referenced schema, and the registry must accept the named property.
Unknown, unsupported, malformed, incorrectly typed, out-of-range,
invalid-schema, or trailing filter data is rejected.

`ListFunctionsResp` carries its schema epoch and gives every parameter and
return value a schema slot. Function calls carry
`[field_key varint32][value_length varint32][value]`; the signature maps each
nonzero key to its schema-backed parameter. Duplicate, unknown,
missing-required, malformed, or schema-invalid fields are rejected. Omitted
optional fields may use their declared defaults.

`CreateInputStreamReq` carries a schema epoch/slot and maximum value and batch
sizes. The slot must resolve in the committed epoch, and the creation callback
may refuse it. Input batches contain a stream ID, bounded count, and
length-delimited values. Every value must be within the configured size and
validate against the stream schema before delivery. Unknown or closed stream
IDs, malformed batches, invalid values, and trailing bytes are rejected.

## Staleness and rejection

Clients must discard a definition whose computed descriptor digest differs
from its advertised `SchemaRef`. They must not commit an epoch until required
definitions validate, and must not reuse a slot from a prior epoch.

Servers reject ordinary traffic before commit, requests or commits with the
wrong epoch, unknown slots, values that do not consume their payload, and
catalog values that do not match their installed schema. A client rejects a
`ConfigureAck` whose schema epoch is stale; the resulting stream layout is
valid only for the epoch it carries.

## Removed V5 facilities

V6 does not use `FeatureExchange`, `DescribeStruct`, `StructDescriptor`, or
raw V5 layout contracts for schema discovery. Retained message IDs do not
replace the mandatory V6 bootstrap and catalog.
