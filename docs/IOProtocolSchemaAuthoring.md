# Tether IO Protocol — Schema Authoring

How to build a `SchemaGraph`, install it into a `SchemaCatalog`, and
produce/consume schema-encoded values. The negotiation handshake and
byte-level encodings are in
[IOProtocolWireFormat.md](IOProtocolWireFormat.md); this is the
author-facing half.

## Model in one paragraph

A **schema node** describes a type (kind + constraints + children).
Nodes reference each other by `SchemaRef{key, digest}` inside a
dependency-closed **graph**. A `SchemaCatalog` installs a graph plus a
manifest subset as one **epoch**; each manifest entry gets a compact
**slot** usable for that epoch only. Values are encoded against the
schema — the wire never carries raw struct layouts or raw
`ValueType` guesses for schema-backed entries.

## SchemaNode fields

```cpp
struct SchemaNode {
    SchemaKey key;              // 16-byte identity (name-keyed hash or UUID)
    uint32_t revision;          // bump on incompatible edits
    SchemaKind kind;
    std::string name, description;
    std::map<std::string,std::string> annotations;

    ValueType scalarType;       // for Scalar/Enum kinds
    uint32_t maxBytes;          // String/Bytes cap (0 = unlimited)
    uint32_t fixedCount;        // FixedArray/FixedString exact count
    uint32_t minCount, maxCount;// DynamicArray/Map bounds; maxCount==0 = unbounded
    StructEncoding structEncoding;   // Tagged or Packed
    std::optional<SchemaRef> element;      // array/optional element node
    std::optional<SchemaRef> mapKey, mapValue;   // map children
    std::optional<SchemaRef> target;             // alias/oneof target
    std::vector<SchemaField> fields;             // struct fields
    std::vector<std::pair<uint32_t,SchemaRef>> oneOfMembers;
};
```

`SchemaField{key, flags, schema, name, description}` +
`presence` (`Required`/`Optional`) + `restrictions` + `defaultValue`.

## Kinds

| Kind | Purpose | Constraints |
|---|---|---|
| `Scalar` | `ValueType` scalars | `scalarType` |
| `String` | UTF-8 + NUL terminator | `maxBytes` |
| `Bytes` | opaque bytes | `maxBytes` |
| `FixedString` | fixed-capacity, NUL-padded | `fixedCount` |
| `Enum` | `scalarType`-backed named values | `scalarType`, restrictions |
| `Struct` | named fields | `fields`, `structEncoding` |
| `FixedArray` | exact-count elements | `fixedCount`, `element` |
| `DynamicArray` | counted elements | `minCount/maxCount`, `element` |
| `Map` | ordered key→value | `mapKey/mapValue`, count bounds |
| `Optional` | presence-marked value | `element` |
| `OneOf` | tagged union | `oneOfMembers` |
| `Alias` | rename/annotate another node | `target` |

## Two field-level optionality models — pick the right one

- `FieldPresence::Optional` on a struct field → the field may be
  **omitted** from a tagged value (sparse encoding, patch semantics).
- `SchemaKind::Optional` node → the value carries an explicit
  `present` byte; `nv::none()` builds an absent value, a wrapped
  element value a present one.

A `FieldPresence::Optional` field whose schema is a plain `String`
node cannot take `nv::none()` — absent means the field is *not sent*.
To carry an explicit "empty vs set" distinction, wrap the element in
an `Optional` node.

## Packed vs tagged structs

| | `StructEncoding::Packed` | `StructEncoding::Tagged` |
|---|---|---|
| Wire | concatenated field values, key order | `[count][key][len][payload]…` sorted keys |
| Fields allowed | required only, no defaults | required + optional + defaults |
| Evolution | none — layout is the contract | add/omit optional fields freely |
| Fixed layout | `computeSchemaLayout` → `fixedLayout`, often `memcpyImage` | no |
| Use for | hot-path frames (drive snapshots, control frames) | descriptor/config/events |

`computeSchemaLayout(node)` reports `fixedLayout` (all offsets
compile-time known) and `memcpyImage` (the wire bytes equal a C struct
image — verify with a `static_assert`/`memcmp` before relying on it;
`FixedString` is how fixed-width text stays packed).

## Building and encoding values

`nv::` helpers in `SchemaValueBuilder.hpp` produce lazily-typed
`NativeValue`s; `encodeNative(graph, node, native)` validates + encodes:

```cpp
const SchemaNode& frame = *graph.find(key);
auto bytes = encodeNative(graph, frame, nv::fields(
    nv::field(1, nv::u64(seq)),
    nv::field(2, nv::str("run-42")),
    nv::field(3, nv::items(nv::f64(0.5), nv::f64(1.5))),
    nv::field(4, nv::map({{nv::str("kp"), nv::f64(1.0)}})),
    nv::field(5, nv::none()),                    // Optional node absent
    nv::field(6, nv::oneOf(2, nv::i64(-7))),     // OneOf member 2
    nv::field(7, nv::u64v(std::span(vals)))));
// bytes → varReadFn / FunctionCallResult::returnValue / input stream
```

Builders: `flag` (bool), `u/u8/u16/u32/u64` (unsigned, width-checked),
`i/i32/i64` (signed), `f/f64` (float), `str` (rejects embedded NUL),
`bytes`, `items` (array), `fields`/`field` (struct), `map` (ordered
entries), `none` (Optional absent), `oneOf(key, v)`, and the `*v`
vector convenience forms (`u64v/i64v/f64v/strv`).

Common errors → build/validation failures: wrong scalar width,
`nv::none()` on a non-Optional node, missing required fields, count
out of `minCount/maxCount`, non-canonical map key order (keys must be
strictly increasing in canonical order — `nv::map` sorts), trailing
bytes.

## Digests, manifest, epochs, slots

- `computeSchemaDigest(node)` — 32-byte digest over the node's wire
  descriptor (kind, constraints, children). Two nodes with the same
  key but different descriptors are different schemas.
- `SchemaManifestEntry{ref, revision}` — what the server advertises;
  `catalog.install(graph, manifest)` validates every entry against
  the graph and assigns slots **sorted by key** (not manifest order).
- `catalog.slotForKey(key)` — registration-time slot lookup for
  catalog entries.
- Epoch increments per install; a committed client holds exactly one
  epoch. A server-side catalog mutation that changes the schema set
  should reinstall the catalog (new epoch) so clients get
  `SchemaUpdate` rather than decoding against a stale slot.
- The catalog **borrows the graph pointer** — the `SchemaGraph` must
  outlive the catalog (pass a named object, not a temporary).

## Graph rules `validateSchemaGraph` enforces

- Dependency-complete, acyclic, within configured `SchemaLimits`.
- Field/member keys strictly increasing and nonzero; presence markers
  0/1.
- Packed structs: required fields only, no defaults.
- Map keys: string, bytes, enum, or non-floating scalar nodes.
- `maxCount == 0` means **unbounded** (graph validation treats it as
  no cap — don't write `maxCount = 0` meaning "empty allowed"; that's
  `minCount = 0`).

## Client-side decode

```cpp
auto raw  = client.getSignal(id);
auto node = client.resolveSchemaSlot(entry.schemaSlot);  // epoch-checked
auto val  = client.decodeValue(entry.schemaSlot, raw->data(), raw->size());
// val → SchemaValue{node, payload}; walk std::get<Struct>(payload).fields
```

`SchemaValue` preserves the schema pointer — field names, not byte
offsets, drive the decode. See
`examples/tether_io_schema_negotiation.cpp` and
`tether_io_schema_packed.cpp`/`tether_io_schema_tagged.cpp` for the
full loops.

## Checklist for a new schema

1. Declare scalar/element/struct nodes with 16-byte keys (`schemaKey`
  helpers) and revision 1.
2. Set constraints honestly: `maxBytes`, `fixedCount`,
   `minCount/maxCount` (0 = unbounded), packed vs tagged.
3. `SchemaCatalog::install(graph, manifest)` — fail hard on `false`.
4. `slotForKey` → `ParamEntry`/`SignalEntry`/`FunctionEntry`
   `schemaSlot` + `schema` fields.
5. Producer encodes via `encodeNative` (or a verified `memcpyImage`
   packed layout); consumer decodes via `decodeValue`.
6. Incompatible change later → bump `revision`, new digest, reinstall
   (new epoch).
