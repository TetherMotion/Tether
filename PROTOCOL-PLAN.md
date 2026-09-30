# Tether IO Protocol V6 Plan: Recursive Schema Negotiation

## Status

This document defines an incompatible Tether IO protocol v6. It replaces the
v5 scalar `FeatureExchange`, flat `StructDescriptor`, `ValueType::Struct`, and
`DescribeStruct` model. No v5 wire compatibility is required or desired.

The implementation must first make schemas a negotiated protocol capability.
It must not add another application-specific binary layout or a new manually
maintained TypeScript parser for every C++ struct.

## Problem

V5 `FeatureExchange` transfers only independent `name` / scalar-type / bytes
triples. `StructDescriptor` can describe only one fixed-size, flat structure
using byte offsets. Neither can describe nested structs, optional fields,
fixed arrays, variable arrays, or arbitrary finite nesting.

Applications therefore need an out-of-band layout agreement, untyped `Binary`
values, or a bespoke parser. A generic client cannot reliably discover and
render an unknown application value.

## Goals

1. A peer automatically learns every schema required by values the other peer
   advertises or sends.
2. Schemas represent scalars, enums, fixed arrays, variable arrays, maps,
   structs, optionals, numbered oneofs, aliases, and arbitrary finite nesting.
3. Semantic types and fields use stable machine keys, not labels or positions.
4. Peers exchange manifests and canonical digests, transferring only missing
   or changed schema definitions.
5. All received descriptors and values are validated before reaching an
   application or UI.
6. The wire format is deterministic, little-endian, bounded, and portable
   across C++, TypeScript, and embedded targets.
7. Catalogs, feature values, functions, input streams, parameters, signals,
   and returns use the same schema-reference model.
8. High-rate fixed telemetry remains compact; general values stay extensible.

## Non-Goals

- Automatic serialization of arbitrary C++ object graphs or native layouts.
- Serializing pointers, compiler padding, bitfields, endianness-dependent
  values, or object lifetime.
- Unbounded parser recursion, allocation, schema cache, or value size.
- Using a schema name or UI metadata as authorization.

## Core Decisions

### Clean V6 break

Set `PROTOCOL_VERSION` to `6`. V6 uses a new message map, mandatory handshake,
schema-aware catalog/function formats, and schema-directed values. Delete the
v5 dual-parser idea rather than carrying a compatibility branch.

`FeatureExchange` becomes the mandatory capability-and-schema negotiation
phase. Catalogs, function registration, stream setup, and application values
are unavailable until that phase commits.

### Minimal bootstrap

Only these fixed, statically decoded values are permitted before negotiation:

- Protocol range, encoding features, resource limits, and selected limits.
- Schema keys, digests, session slots, manifests, chunks, and errors.

All application data, including capability feature values, uses a negotiated
schema. The bootstrap must not grow application-specific value definitions.

### Stable schema identity

| Item | Wire representation | Meaning |
| --- | --- | --- |
| `SchemaKey` | 16 bytes | Stable semantic identity, allocated once and never reused |
| `SchemaDigest` | BLAKE3-256, 32 bytes | Exact canonical schema definition identity |
| `SchemaSlot` | U32, session-local | Compact reference after schema negotiation |
| `SchemaRevision` | U32 | Human/debugging compatibility revision; digest is authoritative |
| `SchemaName` | UTF-8 annotation | Display/debug label only, never identity |

`SchemaKey` is the pre-negotiated type key. An application can compile a known
key for `tether.cia402.drive-snapshot`; a generic client learns its descriptor
automatically. Peers match by `(SchemaKey, SchemaDigest)`, never by a name or
session-local slot. Every nested `Struct` is itself a separately keyed schema
definition, so it may have its own human-readable `SchemaName` and description.
Those annotations are optional and may be empty for an anonymous structural
type; the key and digest remain authoritative.

Keys are allocated from an approved UUID/128-bit registry and checked for
duplicates in CI. They must not be hashes of C++ type names.

### Stable field and oneof member keys

Every struct field has a non-zero U32 `FieldKey`. Every enum value and `OneOf`
member has a non-zero U32 key. Keys are unique within their immediate owner and
never renumbered. Labels, units, descriptions, order, and UI hints are
annotations that may change without changing a key.

### Finite schema graph

Schemas form a directed acyclic graph. A descriptor references another schema
by `(SchemaKey, SchemaDigest)`; definitions can arrive in any order. Arbitrary
finite nesting is allowed up to the negotiated depth. V6 forbids direct and
indirect recursive schema definitions.

This keeps descriptor validation, fixed-size calculation, and embedded support
predictable. A later protocol may introduce explicitly bounded recursion only
if a real application requires it.

## Schema Model

### Canonical descriptor header

Every schema definition encodes these canonical fields:

```
schema_key          [16 bytes]
schema_revision     [u32]
kind                [u8]
flags               [u32]
name                [string]
description         [string]
annotations         [typed annotation map]
kind_payload        [canonical kind-specific bytes]
```

The peer recomputes `BLAKE3-256` over the complete canonical descriptor,
including sorted annotations, before installing it. A claimed digest is never
trusted without local recomputation. Strings are valid UTF-8; annotation keys
are ASCII dot-separated identifiers. Unless a grammar explicitly says
otherwise, every wire `string`, including descriptor labels and annotations,
uses the zero-terminated `String` encoding defined below.

### Schema kinds

| Kind | Descriptor data | Payload behavior |
| --- | --- | --- |
| `Scalar` | Wire scalar | Exact scalar encoding |
| `String` | UTF-8 text | Arbitrary-length, zero-terminated UTF-8 |
| `Bytes` | No schema-local size limit | Arbitrary-length, length-delimited bytes |
| `Enum` | Unsigned scalar and keyed labels | Exact underlying scalar |
| `Struct` | Encoding mode and keyed fields | Packed or tagged fields |
| `FixedArray` | Element `SchemaRef` and exact count | Exactly declared count |
| `DynamicArray` | Element `SchemaRef` and optional count restrictions | Counted elements |
| `Map` | Uniform key/value `SchemaRef`s and optional entry-count restrictions | Ordered unique key/value entries |
| `Optional` | Element `SchemaRef` | Presence byte and optional payload |
| `OneOf` | Predefined numbered members and member `SchemaRef`s | Exactly one selected typed member |
| `Alias` | Target `SchemaRef` | Target payload with semantic annotations |

`Scalar` covers existing numeric, boolean, IPv4, IPv6, MAC, and canonical
signed/unsigned varint forms. It never means native enum, C++ struct, pointer,
or native object representation.

`String` values contain valid UTF-8 bytes followed by exactly one `0x00`
terminator. A string contains no interior `0x00`; its terminator is part of the
value encoding but not the logical value. `Bytes` values use a canonical U64
varuint byte count followed by that many arbitrary bytes. Neither type has a
schema-local maximum: their actual size is limited only by negotiated and local
resource budgets, or by an explicit field restriction.

Fixed and dynamic arrays can contain any schema kind, including arrays, maps,
and structs. A map has one predeclared key schema and one predeclared value
schema, analogous to `std::map<Key, Value>`: every entry has those exact types.
Map keys must be map-key eligible (`String`, `Bytes`, non-floating scalar,
`Enum`, or an `Alias` of one of those). Their order is the schema-defined
logical order: numeric values compare numerically, booleans use `false < true`,
network/address values and bytes compare unsigned lexicographically, and
strings compare their UTF-8 bytes lexicographically. Floating-point and
composite schemas cannot be map keys. No array or map special cases are
allowed in an application profile.

### Struct fields and nested names

A nested struct is referenced by `SchemaRef` exactly like an array element,
map key/value, or oneof member. It is not an inline anonymous layout hidden
inside its parent.
Consequently, a recursive decoder can expose both the parent struct name and
the name of each nested struct when those optional annotations are present.
The field that points to the nested struct separately has its own stable
`FieldKey` and optional display name.

Each descriptor field is:

```
field_key           [u32, non-zero]
presence             [u8: required or optional]
flags               [u32: read-only, write-only, deprecated, secret, no-stream, ...]
schema_ref          [schema key + digest]
name                [string]
description         [string]
annotations         [typed annotation map]
restrictions        [canonical restriction list]
default_value       [optional schema-directed payload]
```

Fields are sorted by `FieldKey`. A required field is mandatory in tagged values
and has no default. An optional field may be absent and may declare a default.
Field optionality controls whether the field itself is present; a present field
whose schema is `Optional` may independently carry a null value. `read-only`,
`write-only`, `deprecated`, `secret`, and `no-stream` are independent flags.
The `secret` flag is only a display/export hint; the server still enforces
access.

### Field restrictions

Every field may carry zero or more machine-validated restrictions. Restrictions
are scoped to that field, so one reusable schema can be unrestricted in one
field and constrained in another. Each restriction has a `RestrictionKind`,
canonical payload, and no display-only semantics. The list is sorted by kind
and duplicate kinds are rejected. Unknown or inapplicable restrictions are
protocol errors, not hints a receiver may ignore.

The canonical field descriptor encodes `restriction_count [u64 varuint]`, then
each restriction as `[restriction_kind u8][restriction_payload_length u64
varuint][restriction_payload]`. `AllowedValues` are sorted and deduplicated by
their complete canonical encoded value bytes. All restriction payloads are
fully consumed and are included in the enclosing schema digest.

The initial restriction kinds are:

| Restriction | Applies to | Canonical data |
| --- | --- | --- |
| `NumericRange` | Numeric scalar, enum, or alias | Optional inclusive lower and upper values encoded with the field schema |
| `MultipleOf` | Integer scalar, enum, or alias | Non-zero scalar divisor encoded with the field schema |
| `Finite` | `F32`, `F64`, or alias | Requires the value not to be NaN or infinity |
| `LengthRange` | `String`, `Bytes`, arrays, or map | Optional inclusive minimum and maximum logical byte/element/entry count |
| `AllowedValues` | Any schema | Sorted, duplicate-free schema-directed values allowed for this field |
| `AllowedMembers` | `Enum` or `OneOf` | Sorted, non-zero allowed member keys |
| `UniqueElements` | `DynamicArray` | Requires pairwise distinct canonical element values |

`LengthRange` measures UTF-8 bytes excluding a string terminator, raw bytes for
`Bytes`, element count for arrays, and entry count for maps. Restrictions are
checked after decoding a complete field value and before application code sees
it; defaults must also satisfy them. A restriction never changes the encoding
or fixed-size eligibility of the referenced schema.

### Maps and oneof members

A `Map` descriptor contains its key and value `SchemaRef`, followed by any
schema-wide entry count policy. Its values are encoded as:

```
entry_count          [u64 varuint]
repeat entry_count times, keys strictly increasing:
   key_payload_length [u64 varuint]
   key_payload        [key_payload_length bytes]
   value_payload_length [u64 varuint]
   value_payload      [value_payload_length bytes]
```

Receivers decode every key/value using the declared schemas, require the keys
to be strictly increasing in the map-key order, and reject duplicate keys,
invalid lengths, or incomplete consumption. Maps are semantically variable
length even if a field restricts them to a fixed number of entries. A map's
key/value schemas, entry policy, and canonical ordering are all part of its
descriptor digest.

`OneOf` is the V6 discriminated-union type. Its descriptor contains a sorted
set of predefined members, each with a non-zero `member_key`, `SchemaRef`, and
optional annotation/description. Exactly one numbered member is selected in a
value; there are no implicit, anonymous, or receiver-defined members.

### Fixed-size eligibility

A schema is fixed-size only when it is a fixed scalar, fixed alias,
`FixedArray` of a fixed-size element, or packed struct of only required,
fixed-size fields. It must contain no string, bytes, dynamic array, map,
optional, oneof, tagged struct, or variable scalar. The descriptor contains its
computed fixed size; every receiver recomputes it and rejects a mismatch.

## Value Encoding

### General rules

- Fixed-width values are little-endian.
- Counts and delimited-payload lengths are canonical U64 unsigned varints.
- Decoding is bounded by negotiated value bytes, element count, field count,
  graph depth, value depth, and aggregate allocation budgets.
- C/C++ padding, alignment, ABI, and native object representations never occur
  on the wire.
- A payload is decoded only using the schema attached to its catalog item,
  function signature, stream layout, or typed envelope.

### Packed structs

Packed structs are allowed only for fixed-size schemas. Their fields are
concatenated by strictly ascending `FieldKey`, without IDs, lengths, padding,
or alignment. Nested fixed schemas use their packed representation.

Changing a packed struct changes its digest. A sender must never emit the new
layout under an old digest. Packed values are the intended high-rate path for
fixed telemetry such as CiA 402 drive status samples.

### Tagged structs

Tagged structs support optional, variable, and evolvable fields:

```
field_count          [varuint]
repeat field_count times, field_key strictly increasing:
  field_key          [varuint]
  value_length       [varuint]
  field_payload      [value_length bytes]
```

Required fields appear exactly once. Optional fields appear at most once.
Unknown fields are permitted only when the descriptor permits unknown optional
fields. `value_length` lets a newer client skip a permitted unknown field
without guessing its nested type.

### Arrays and maps

`FixedArray` has no count. It contains exactly the declared number of elements.
Fixed-size elements are concatenated; variable-size elements are each
`[element_length varuint][element_payload]`.

`DynamicArray` is:

```
count                [varuint]
repeat count times:
  element_length     [varuint]
  element_payload    [element_length bytes]
```

The decoder validates count range, element length, exact payload consumption,
and aggregate byte/depth budgets. Elements can recursively be arrays or
maps or structs at any negotiated finite depth. Map encoding and validation are
defined in [Maps and oneof members](#maps-and-oneof-members).

### Optional, oneof, strings, and typed envelopes

`Optional` is `[present u8]`, where `0` has no payload and `1` is followed by
the element payload. Other values are invalid.

`OneOf` is `[member_key u64-varuint][payload_length u64-varuint][payload]`.
The member key must occur in the descriptor and must satisfy any
`AllowedMembers` restriction on its containing field. Unknown members are
always rejected.

`String` is its UTF-8 content followed by `0x00`, with no length prefix.
Containers whose grammar needs a boundary use their existing payload length;
otherwise the terminator supplies the boundary. `Bytes` is
`[byte_length u64-varuint][bytes]`. Both forms reject truncated, oversized,
or non-canonical values before allocation.

Catalog entries and function fields already have a schema slot, so their values
carry no redundant type identity. An otherwise untyped attachment uses:

```
schema_slot           [u32]
   payload_length        [u64 varuint]
payload               [payload_length bytes]
```

A missing or stale slot is a protocol error, never a best-effort binary decode.

## Mandatory Feature And Schema Negotiation

### Limits

Each peer offers and the server selects conservative limits for message bytes,
schema descriptor bytes, definitions per epoch, graph nodes/depth, value depth,
collection entries, individual string/bytes payload bytes, aggregate allocation,
and cache entries/bytes. The selected limit cannot exceed either offer. Local
hard limits always apply. The arbitrary-length `String` and `Bytes` types have
no schema-local cap; these negotiated limits remain mandatory for safe parsing.

### Bootstrap messages

V6 reserves these handshake messages. Exact numeric values are assigned when
`Protocol.hpp` is rewritten; names and order are normative:

| Message | Direction | Purpose |
| --- | --- | --- |
| `ClientHello` | client to server | Version range, encodings, limits, capabilities, cached schema manifest |
| `ServerHello` | server to client | Selected V6 limits, capabilities, schema epoch, server manifest |
| `SchemaRequest` | either | Requests `(SchemaKey, SchemaDigest)` definitions |
| `SchemaDefinition` | either | Supplies one canonical descriptor, optionally chunked |
| `SchemaCommit` | either | Acknowledges a validated dependency-closed schema set |
| `SchemaReject` | either | Reports descriptor/reference validation failure |
| `SchemaUpdate` | server to client | Announces a pending schema/catalog epoch update |

`FeatureExchange` is the name for this full phase. The v5 scalar feature triple
is removed. Capabilities themselves are keyed typed values and may be nested.

### Automatic manifest flow

1. Client sends `ClientHello` with its bounded cached `(SchemaKey,
   SchemaDigest)` manifest and bootstrap capabilities.
2. Server selects V6 and limits, allocates a `schema_epoch`, and returns
   `ServerHello` with every root needed by server catalog, functions, features,
   and stream layouts.
3. Each receiver compares roots and dependencies with its cache and sends
   `SchemaRequest` only for missing exact definitions.
4. The sender emits canonical `SchemaDefinition` messages. Definitions may be
   out of order; no definition enters the active table before its complete graph
   validates and its digest is recomputed.
5. Each side sends `SchemaCommit(schema_epoch)` only when it has every required
   schema. No typed catalog data or function invocation is permitted earlier.
6. Server sends the catalog snapshot using committed `SchemaSlot`s. Client-hosted
   functions use the same request/definition/commit path before registration.

Schema transfer is automatic from references. An application and its UI never
list schema dependencies manually or maintain a parallel type registry.

### Cache, slots, and runtime updates

The persistent cache key is `(SchemaKey, SchemaDigest)`. A cache hit avoids
transfer but not validation. `SchemaSlot` is assigned by the declaring peer,
valid only within one schema epoch, and never reused in that epoch.

An application changing a descriptor advertises a new digest in a new epoch.
The server sends `SchemaUpdate`, repeats manifest/request/definition/commit,
then sends one atomic catalog snapshot for the new epoch. Streams on an old
layout stop and must be configured again. A stream row never mixes epochs.

## Catalog And Function Integration

### Catalog entries

Each parameter/signal descriptor contains:

```
resource_key          [16 bytes, stable application identity]
schema_slot           [u32]
flags                 [u32]
name                  [string annotation]
description           [string annotation]
group                 [string annotation]
annotations           [typed metadata map]
```

V5 `valueType`, `valueSize`, `HasStruct`, `HasEnum`, raw metadata map, and
`DescribeStruct` disappear. Fixed/variable behavior comes from the schema.

### Functions, streams, and input streams

Function parameters and returns reference `SchemaSlot`s. Arguments use stable
parameter keys and tagged-struct field encoding, so arbitrary nesting needs no
special function descriptor.

Stream layouts carry ordered `(resource_key, schema_slot)` pairs. A layout of
only fixed-size values is compact concatenation in layout order; any variable
layout uses length-delimited value payloads. Layouts are immutable and bound to
one schema epoch.

Input streams declare a schema slot and validate every batch value against it.

## Validation And Security

Reject a descriptor containing duplicate/invalid keys, digest mismatch,
non-canonical order, missing/mismatched/cyclic dependency, limit violation,
invalid map-key schema, malformed/duplicate oneof member, variable or optional
packed field, invalid default payload, invalid field restriction, invalid
UTF-8 or string termination, or duplicate annotation key.

Reject a value with unknown/stale slot, incomplete consumption, bound violation,
invalid tagged field order/duplication/requirement, invalid restriction,
invalid map ordering/duplicate key, unknown oneof member, or aggregate budget
breach. Never truncate/coerce a value to make it decode.

Schema/value input is untrusted. Parse lengths before allocation, enforce
cumulative budgets, cache only validated definitions, LRU-evict under hard
limits, verify every digest, keep schema work off the EtherCAT cyclic path,
HTML-escape display annotations, and enforce authorization independently of
metadata.

## Documentation Deliverables

1. Replace `docs/IOProtocolWireFormat.md` with V6 bootstrap, schemas,
   negotiation, catalog, function, stream, and value encoding rules.
2. Add `docs/IOSchemaAuthoring.md` for key allocation, annotations,
   compatibility, code-generation guidance, and authoring examples.
3. Add `docs/IOSchemaExamples.md` with nested fixed/variable arrays, maps,
   numbered oneofs, unrestricted string/bytes fields, field restrictions,
   CiA 402 status, motion command, event record, and input-stream schemas.
4. Update public registry documentation to use `SchemaRef`/schema roots rather
   than `ValueType` and flat descriptors.
5. Document the resource-limit policy and mandatory fuzz corpus.

## Implementation Plan

### Step 1: Freeze V6

1. Approve key, digest, graph, encoding, and handshake decisions in this plan.
2. Select a BLAKE3-256 implementation suitable for all supported targets, or
   approve an alternative before code is written.
3. Assign bootstrap message IDs in one authoritative V6 table.
4. Set host/embedded hard limits and default negotiated limits.
5. Establish the schema-key allocation registry and duplicate-key CI check.

### Step 2: C++ schema core

6. Replace `BinaryStruct.hpp` with `Schema.hpp` types for keys, refs, nodes,
   descriptors, manifests, slots, and validation errors.
7. Implement canonical descriptor encoding/decoding and digest computation,
   including maps, numbered oneofs, field presence, and restrictions.
8. Implement graph assembly, acyclicity, fixed-size computation, canonical
   order validation, map-key eligibility, and restriction validation.
9. Implement bounded persistent schema cache and session schema table.
10. Implement schema-directed value encode/decode for every V6 schema kind,
   including zero-terminated strings, arbitrary bytes, maps, and oneofs.
11. Add unit, property, and fuzz tests for descriptors, restrictions, and
   nested values.

### Step 3: Session and registry migration

12. Rewrite `FeatureExchange` as the mandatory V6 schema-sync state machine.
13. Add epoch, manifest, request, chunk, commit, reject, and update handling
    to `Session`.
14. Rewrite registry and function descriptors to reference schemas.
15. Replace `DescribeStruct` and all `ValueType` layout decisions across get,
    set, snapshot, stream, and input-stream paths.
16. Couple atomic catalog updates to schema epochs.
17. Prove no schema parsing/sync/allocation executes in the cyclic task.

### Step 4: TypeScript and UI

18. Replace the V5 TypeScript protocol decoder with V6 bootstrap/schema sync.
19. Implement strict bounded recursive TypeScript value codec and cache,
    including zero-terminated strings, arbitrary bytes, maps, oneofs, and
    field restrictions.
20. Build generic scalar, enum, struct, fixed/dynamic array, map, optional,
   and numbered-oneof UI primitives from schemas and annotations.
21. Ensure browser rendering never exceeds negotiated limits.
22. Migrate dashboard catalog, scope, jog, and function UIs to schema slots.

### Step 5: Adoption and hardening

23. Express CiA 402 entirely through negotiated schemas and retire its
    hand-written codec when generic decoding is ready.
24. Add a simulated server exporting deeply nested fixed/dynamic values.
25. Add C++ <-> TypeScript golden vectors for every schema kind/nesting form,
   map ordering, oneof members, zero-terminated strings, arbitrary bytes, and
   restrictions.
26. Fuzz malformed manifests, chunks, cycles, digests, depth, stale slots,
   maps, oneofs, restrictions, and schema-update races.
27. Load-test cache handshakes, high-rate packed streams, catalog updates, many
    clients, and bounded memory.
28. Remove all V5 code/tests/docs only after V6 migration coverage is complete.

## Acceptance Criteria

- A generic client discovers, validates, caches, and renders a server schema
   such as `Map<String, Array<Struct<Optional<OneOf>>>>` without custom code.
- Maps reject out-of-order or duplicate keys; oneofs reject undefined member
   keys; zero-terminated strings and arbitrary bytes round-trip without a
   schema-local size cap.
- Field presence, defaults, and every applicable restriction are validated
   before values reach application code.
- A reconnect reuses cached definitions only after key/digest validation.
- Invalid/cyclic/oversized schema or value input fails deterministically with
  bounded recursion/allocation and no cyclic-thread impact.
- No catalog or stream uses unknown, stale, or uncommitted schema slots.
- C++ and TypeScript exactly match nested golden vectors.
- Fixed packed telemetry streams at the intended rate without per-field dynamic
  allocation or schema lookup in the hot row path.