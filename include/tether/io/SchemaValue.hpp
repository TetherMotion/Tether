#pragma once

#include "tether/io/Schema.hpp"
#include "tether/io/SchemaValueWire.hpp"
#include "tether/io/SchemaValueCodec.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <expected>
#include <format>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace tether::io {

// ===========================================================================
// SchemaValue — decoded value tree for a V6 schema value
//
// decodeSchemaValue() turns wire bytes into a SchemaValue tree; every node
// pointer is non-owning and refers to nodes in the caller's SchemaGraph,
// which must outlive the values.
//
// encodeSchemaValue() turns a SchemaValue tree back into canonical wire
// bytes (sorted tagged-struct keys, canonical map-key order). Round-trips
// through validateSchemaValue().
// ===========================================================================

struct SchemaValue {
    /// The resolved node this value was decoded/encoded against.
    /// Aliases are chased during decode, so this is never an Alias node.
    const SchemaNode* node = nullptr;

    struct Scalar {
        /// Raw payload bits, matching SchemaValueCodec's NumericValue
        /// convention: unsigned/enum/varint values stored directly, signed
        /// integers sign-extended to 64 bits, F32/F64 stored as the raw
        /// IEEE bit pattern in the low bits.
        uint64_t bits = 0;
    };
    struct Str   { std::string value; };
    struct Bytes { std::vector<uint8_t> value; };
    struct Array { std::vector<SchemaValue> elements; };
    struct Struct {
        struct Field {
            uint32_t key = 0;
            const SchemaField* schemaField = nullptr;   ///< nullptr = unknown key
            std::unique_ptr<SchemaValue> value;
        };
        std::vector<Field> fields;                      ///< wire order (sorted keys)
    };
    struct Map {
        struct Entry {
            std::unique_ptr<SchemaValue> key;
            std::unique_ptr<SchemaValue> value;
        };
        std::vector<Entry> entries;                     ///< wire order
    };
    /// std::nullopt payload = present and value == nullptr; use `present`.
    struct Optional { bool present = false; std::unique_ptr<SchemaValue> value; };
    struct OneOf { uint32_t memberKey = 0; std::unique_ptr<SchemaValue> value; };

    std::variant<std::monostate, Scalar, Str, Bytes, Array, Struct, Map,
                 Optional, OneOf> payload;
};

// ===========================================================================
// Decode
// ===========================================================================

namespace schema_value_detail {

inline std::unexpected<std::string> svfail(const char* msg) {
    return std::unexpected(std::string(msg));
}

inline const SchemaNode* resolveAliasNode(const SchemaGraph& graph,
                                          const SchemaNode* node) {
    while (node && node->kind == SchemaKind::Alias) {
        if (!node->target) return nullptr;
        node = graph.find(node->target->key);
    }
    return node;
}

inline const SchemaNode* resolveAliasRef(const SchemaGraph& graph,
                                         const SchemaRef& ref) {
    return resolveAliasNode(graph, graph.find(ref.key));
}

inline std::expected<SchemaValue, std::string>
decodeNode(const SchemaGraph& graph, const SchemaNode& node, BufReader& reader,
           const SchemaValueLimits& limits, uint32_t depth);

inline std::expected<SchemaValue, std::string>
decodeRef(const SchemaGraph& graph, const SchemaRef& ref, BufReader& reader,
          const SchemaValueLimits& limits, uint32_t depth) {
    const SchemaNode* node = resolveAliasRef(graph, ref);
    if (!node) return svfail("unresolvable schema ref");
    return decodeNode(graph, *node, reader, limits, depth + 1);
}

/// Decode a payload of `len` bytes as the given ref (element/map children
/// are length-delimited on the wire).
inline std::expected<SchemaValue, std::string>
decodeChildPayload(const SchemaGraph& graph, const SchemaRef& ref,
                   BufReader& reader, const SchemaValueLimits& limits,
                   uint32_t depth) {
    uint64_t length = 0;
    if (!schema_value_wire_detail::getU64Varint(reader, length) ||
        length > limits.maxBytes || length > reader.remaining()) {
        return svfail("invalid child length");
    }
    const auto* payload = reader.getBytes(static_cast<size_t>(length));
    if (!reader.ok()) return svfail("truncated child payload");
    BufReader childReader(payload, static_cast<size_t>(length));
    auto child = decodeRef(graph, ref, childReader, limits, depth);
    if (!child) return child;
    if (childReader.remaining() != 0) return svfail("trailing child bytes");
    return child;
}

inline std::expected<SchemaValue, std::string>
decodeScalar(const SchemaNode& node, BufReader& reader) {
    SchemaValue::Scalar scalar;
    switch (node.scalarType) {
        case ValueType::U8: case ValueType::Bool:
            scalar.bits = reader.getU8(); break;
        case ValueType::U16:
            scalar.bits = reader.getU16(); break;
        case ValueType::U32: case ValueType::Stream:
            scalar.bits = reader.getU32(); break;
        case ValueType::U64:
            scalar.bits = reader.getU64(); break;
        case ValueType::I8:
            scalar.bits = static_cast<uint64_t>(static_cast<int64_t>(
                static_cast<int8_t>(reader.getU8()))); break;
        case ValueType::I16:
            scalar.bits = static_cast<uint64_t>(static_cast<int64_t>(
                static_cast<int16_t>(reader.getU16()))); break;
        case ValueType::I32:
            scalar.bits = static_cast<uint64_t>(static_cast<int64_t>(reader.getI32())); break;
        case ValueType::I64:
            scalar.bits = static_cast<uint64_t>(static_cast<int64_t>(reader.getU64())); break;
        case ValueType::F32:
            scalar.bits = reader.getU32(); break;
        case ValueType::F64:
            scalar.bits = reader.getU64(); break;
        case ValueType::UVarint: case ValueType::IVarint: case ValueType::Enum:
            if (!schema_value_wire_detail::getU64Varint(reader, scalar.bits)) {
                return svfail("invalid varint scalar");
            }
            break;
        case ValueType::IPv4: {
            const auto* p = reader.getBytes(4);
            if (!p) return svfail("truncated IPv4");
            SchemaValue v;
            v.node = &node;
            v.payload = SchemaValue::Bytes{std::vector<uint8_t>(p, p + 4)};
            return v;
        }
        case ValueType::IPv6: {
            const auto* p = reader.getBytes(16);
            if (!p) return svfail("truncated IPv6");
            SchemaValue v;
            v.node = &node;
            v.payload = SchemaValue::Bytes{std::vector<uint8_t>(p, p + 16)};
            return v;
        }
        case ValueType::MAC: {
            const auto* p = reader.getBytes(6);
            if (!p) return svfail("truncated MAC");
            SchemaValue v;
            v.node = &node;
            v.payload = SchemaValue::Bytes{std::vector<uint8_t>(p, p + 6)};
            return v;
        }
        default:
            return svfail("unsupported scalar type");
    }
    if (!reader.ok()) return svfail("truncated scalar");
    SchemaValue v;
    v.node = &node;
    v.payload = scalar;
    return v;
}

inline std::expected<SchemaValue, std::string>
decodeNode(const SchemaGraph& graph, const SchemaNode& node, BufReader& reader,
           const SchemaValueLimits& limits, uint32_t depth) {
    if (depth > limits.maxDepth) return svfail("schema value depth limit exceeded");
    SchemaValue v;
    v.node = &node;
    switch (node.kind) {
        case SchemaKind::Scalar:
        case SchemaKind::Enum:
            return decodeScalar(node, reader);
        case SchemaKind::String: {
            SchemaValue::Str s;
            if (!decodeSchemaString(reader, s.value, limits.maxBytes)) {
                return svfail("invalid string value");
            }
            v.payload = std::move(s);
            return v;
        }
        case SchemaKind::Bytes: {
            SchemaValue::Bytes b;
            if (!decodeSchemaBytes(reader, b.value, limits.maxBytes)) {
                return svfail("invalid bytes value");
            }
            v.payload = std::move(b);
            return v;
        }
        case SchemaKind::Struct: {
            SchemaValue::Struct st;
            if (node.structEncoding == StructEncoding::Packed) {
                for (const auto& field : node.fields) {
                    if (field.presence != FieldPresence::Required) {
                        return svfail("packed struct with optional field");
                    }
                    auto fv = decodeRef(graph, field.schema, reader, limits, depth);
                    if (!fv) return fv;
                    SchemaValue::Struct::Field f;
                    f.key = field.key;
                    f.schemaField = &field;
                    f.value = std::make_unique<SchemaValue>(std::move(*fv));
                    st.fields.push_back(std::move(f));
                }
            } else {
                uint64_t count = 0;
                if (!schema_value_wire_detail::getU64Varint(reader, count) ||
                    count > limits.maxElements) {
                    return svfail("invalid struct field count");
                }
                uint32_t previousKey = 0;
                for (uint64_t i = 0; i < count; ++i) {
                    uint64_t key = 0;
                    if (!schema_value_wire_detail::getU64Varint(reader, key) ||
                        key == 0 || key > UINT32_MAX || key <= previousKey) {
                        return svfail("invalid struct field key");
                    }
                    previousKey = static_cast<uint32_t>(key);
                    const auto fieldIt = std::lower_bound(
                        node.fields.begin(), node.fields.end(), previousKey,
                        [](const SchemaField& f, uint32_t k) { return f.key < k; });
                    if (fieldIt == node.fields.end() || fieldIt->key != previousKey) {
                        return svfail("unknown struct field key");
                    }
                    auto fv = decodeChildPayload(graph, fieldIt->schema, reader,
                                                 limits, depth);
                    if (!fv) return fv;
                    SchemaValue::Struct::Field f;
                    f.key = previousKey;
                    f.schemaField = &*fieldIt;
                    f.value = std::make_unique<SchemaValue>(std::move(*fv));
                    st.fields.push_back(std::move(f));
                }
                for (const auto& field : node.fields) {
                    if (field.presence == FieldPresence::Required) {
                        const bool present = std::any_of(
                            st.fields.begin(), st.fields.end(),
                            [&](const auto& f) { return f.key == field.key; });
                        if (!present) return svfail("missing required struct field");
                    }
                }
            }
            v.payload = std::move(st);
            return v;
        }
        case SchemaKind::FixedArray: {
            if (!node.element) return svfail("fixed array without element");
            SchemaValue::Array a;
            a.elements.reserve(node.fixedCount);
            for (uint32_t i = 0; i < node.fixedCount; ++i) {
                auto el = decodeRef(graph, *node.element, reader, limits, depth);
                if (!el) return el;
                a.elements.push_back(std::move(*el));
            }
            v.payload = std::move(a);
            return v;
        }
        case SchemaKind::DynamicArray: {
            if (!node.element) return svfail("array without element");
            uint64_t count = 0;
            if (!schema_value_wire_detail::getU64Varint(reader, count) ||
                count > limits.maxElements) {
                return svfail("invalid array count");
            }
            SchemaValue::Array a;
            a.elements.reserve(static_cast<size_t>(count));
            for (uint64_t i = 0; i < count; ++i) {
                auto el = decodeChildPayload(graph, *node.element, reader,
                                             limits, depth);
                if (!el) return el;
                a.elements.push_back(std::move(*el));
            }
            v.payload = std::move(a);
            return v;
        }
        case SchemaKind::Map: {
            if (!node.mapKey || !node.mapValue) return svfail("map without key/value");
            uint64_t count = 0;
            if (!schema_value_wire_detail::getU64Varint(reader, count) ||
                count > limits.maxElements) {
                return svfail("invalid map count");
            }
            SchemaValue::Map m;
            m.entries.reserve(static_cast<size_t>(count));
            for (uint64_t i = 0; i < count; ++i) {
                auto key = decodeChildPayload(graph, *node.mapKey, reader,
                                              limits, depth);
                if (!key) return key;
                auto val = decodeChildPayload(graph, *node.mapValue, reader,
                                              limits, depth);
                if (!val) return val;
                SchemaValue::Map::Entry e;
                e.key = std::make_unique<SchemaValue>(std::move(*key));
                e.value = std::make_unique<SchemaValue>(std::move(*val));
                m.entries.push_back(std::move(e));
            }
            v.payload = std::move(m);
            return v;
        }
        case SchemaKind::Optional: {
            if (!node.element) return svfail("optional without element");
            const uint8_t present = reader.getU8();
            if (!reader.ok() || present > 1) return svfail("invalid optional tag");
            SchemaValue::Optional o;
            o.present = present != 0;
            if (o.present) {
                auto el = decodeRef(graph, *node.element, reader, limits, depth);
                if (!el) return el;
                o.value = std::make_unique<SchemaValue>(std::move(*el));
            }
            v.payload = std::move(o);
            return v;
        }
        case SchemaKind::OneOf: {
            uint64_t member = 0;
            if (!schema_value_wire_detail::getU64Varint(reader, member) ||
                member == 0 || member > UINT32_MAX) {
                return svfail("invalid oneof member key");
            }
            const auto it = std::lower_bound(
                node.oneOfMembers.begin(), node.oneOfMembers.end(), member,
                [](const auto& item, uint64_t k) { return item.first < k; });
            if (it == node.oneOfMembers.end() || it->first != member) {
                return svfail("unknown oneof member");
            }
            auto payload = decodeChildPayload(graph, it->second, reader,
                                              limits, depth);
            if (!payload) return payload;
            SchemaValue::OneOf o;
            o.memberKey = static_cast<uint32_t>(member);
            o.value = std::make_unique<SchemaValue>(std::move(*payload));
            v.payload = std::move(o);
            return v;
        }
        case SchemaKind::Alias:
            // Resolved before dispatch — unreachable.
            break;
    }
    return svfail("unhandled schema kind");
}

} // namespace schema_value_detail

/// Decode one schema value from a BufReader. `node` must be the resolved
/// node (aliases are chased internally). Bytes consumed are read from
/// `reader` — check `reader.remaining()` for exact-consumption rules.
inline std::expected<SchemaValue, std::string>
decodeSchemaValue(const SchemaGraph& graph, const SchemaNode& node,
                  BufReader& reader, const SchemaValueLimits& limits = {}) {
    const SchemaNode* resolved = schema_value_detail::resolveAliasNode(graph, &node);
    if (!resolved) return std::unexpected(std::string("unresolvable schema node"));
    return schema_value_detail::decodeNode(graph, *resolved, reader, limits, 0);
}

/// Convenience full-buffer decode — requires exact consumption.
inline std::expected<SchemaValue, std::string>
decodeSchemaValue(const SchemaGraph& graph, const SchemaNode& node,
                  const uint8_t* data, size_t len,
                  const SchemaValueLimits& limits = {}) {
    BufReader reader(data, len);
    auto v = decodeSchemaValue(graph, node, reader, limits);
    if (!v) return v;
    if (reader.remaining() != 0) {
        return std::unexpected(std::string("trailing bytes after schema value"));
    }
    return v;
}

inline std::expected<SchemaValue, std::string>
decodeSchemaValue(const SchemaGraph& graph, const SchemaRef& ref,
                  const uint8_t* data, size_t len,
                  const SchemaValueLimits& limits = {}) {
    const SchemaNode* node = schema_value_detail::resolveAliasRef(graph, ref);
    if (!node) return std::unexpected(std::string("unresolvable schema ref"));
    return decodeSchemaValue(graph, *node, data, len, limits);
}

// ===========================================================================
// Encode (SchemaValue tree -> canonical wire bytes)
// ===========================================================================

namespace schema_value_detail {

/// Growable byte sink with the subset of the BufWriter API the encoder uses.
struct VecWriter {
    std::vector<uint8_t> buf;

    void putU8(uint8_t v) { buf.push_back(v); }
    void putU16(uint16_t v) { buf.push_back(v & 0xFF); buf.push_back(v >> 8); }
    void putU32(uint32_t v) {
        for (unsigned i = 0; i < 4; ++i) buf.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }
    void putU64(uint64_t v) {
        for (unsigned i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }
    void putBytes(const void* d, size_t n) {
        const auto* p = static_cast<const uint8_t*>(d);
        buf.insert(buf.end(), p, p + n);
    }
    void putVarint(uint32_t v) {
        while (v >= 0x80) { putU8((v & 0x7F) | 0x80); v >>= 7; }
        putU8(v);
    }
    void putU64Varint(uint64_t v) {
        while (v >= 0x80) { putU8((v & 0x7F) | 0x80); v >>= 7; }
        putU8(static_cast<uint8_t>(v));
    }
};

inline std::expected<void, std::string>
encodeNode(const SchemaGraph& graph, const SchemaNode& node,
           const SchemaValue& value, VecWriter& w, uint32_t depth);

/// Encode a single child value into a scratch buffer (for length-prefixed
/// wire forms).
inline std::expected<VecWriter, std::string>
encodeChild(const SchemaGraph& graph, const SchemaValue& value, uint32_t depth) {
    const SchemaNode* node = resolveAliasNode(graph, value.node);
    if (!node) return svfail("unresolvable child schema node");
    VecWriter w;
    if (auto r = encodeNode(graph, *node, value, w, depth); !r) {
        return std::unexpected(r.error());
    }
    return w;
}

inline std::expected<void, std::string>
encodeNode(const SchemaGraph& graph, const SchemaNode& node,
           const SchemaValue& value, VecWriter& w, uint32_t depth) {
    if (depth > SchemaValueLimits{}.maxDepth) {
        return svfail("schema value depth limit exceeded");
    }
    switch (node.kind) {
        case SchemaKind::Scalar:
        case SchemaKind::Enum: {
            if (const auto* s = std::get_if<SchemaValue::Scalar>(&value.payload)) {
                switch (node.scalarType) {
                    case ValueType::U8: case ValueType::Bool:
                        w.putU8(static_cast<uint8_t>(s->bits)); return {};
                    case ValueType::U16:
                        w.putU16(static_cast<uint16_t>(s->bits)); return {};
                    case ValueType::U32: case ValueType::Stream:
                        w.putU32(static_cast<uint32_t>(s->bits)); return {};
                    case ValueType::U64:
                        w.putU64(s->bits); return {};
                    case ValueType::I8:
                        w.putU8(static_cast<uint8_t>(s->bits)); return {};
                    case ValueType::I16:
                        w.putU16(static_cast<uint16_t>(s->bits)); return {};
                    case ValueType::I32:
                        w.putU32(static_cast<uint32_t>(s->bits)); return {};
                    case ValueType::I64:
                        w.putU64(s->bits); return {};
                    case ValueType::F32:
                        w.putU32(static_cast<uint32_t>(s->bits)); return {};
                    case ValueType::F64:
                        w.putU64(s->bits); return {};
                    case ValueType::UVarint: case ValueType::IVarint:
                    case ValueType::Enum:
                        w.putU64Varint(s->bits); return {};
                    default:
                        return svfail("unsupported scalar type");
                }
            }
            // Byte-typed scalars accept Bytes payloads too.
            if (const auto* b = std::get_if<SchemaValue::Bytes>(&value.payload)) {
                const size_t need = (node.scalarType == ValueType::IPv4) ? 4
                    : (node.scalarType == ValueType::IPv6) ? 16
                    : (node.scalarType == ValueType::MAC) ? 6 : 0;
                if (need == 0 || b->value.size() != need) {
                    return svfail("bad byte-scalar length");
                }
                w.putBytes(b->value.data(), b->value.size());
                return {};
            }
            return svfail("payload kind does not match scalar schema");
        }
        case SchemaKind::String: {
            const auto* s = std::get_if<SchemaValue::Str>(&value.payload);
            if (!s || s->value.find('\0') != std::string::npos) {
                return svfail("payload kind does not match string schema");
            }
            w.putBytes(s->value.data(), s->value.size());
            w.putU8(0);
            return {};
        }
        case SchemaKind::Bytes: {
            const auto* b = std::get_if<SchemaValue::Bytes>(&value.payload);
            if (!b) return svfail("payload kind does not match bytes schema");
            w.putU64Varint(b->value.size());
            w.putBytes(b->value.data(), b->value.size());
            return {};
        }
        case SchemaKind::Struct: {
            const auto* st = std::get_if<SchemaValue::Struct>(&value.payload);
            if (!st) return svfail("payload kind does not match struct schema");
            if (node.structEncoding == StructEncoding::Packed) {
                if (st->fields.size() != node.fields.size()) {
                    return svfail("packed struct field count mismatch");
                }
                for (const auto& field : node.fields) {
                    const auto it = std::find_if(
                        st->fields.begin(), st->fields.end(),
                        [&](const auto& f) { return f.key == field.key; });
                    if (it == st->fields.end() || !it->value) {
                        return svfail("missing packed struct field");
                    }
                    const SchemaNode* childNode =
                        resolveAliasRef(graph, field.schema);
                    if (!childNode) return svfail("unresolvable field schema");
                    if (auto r = encodeNode(graph, *childNode, *it->value, w, depth + 1);
                        !r) return r;
                }
                return {};
            }
            // Tagged: emit fields sorted by key.
            std::vector<const SchemaValue::Struct::Field*> sorted;
            sorted.reserve(st->fields.size());
            for (const auto& f : st->fields) sorted.push_back(&f);
            std::sort(sorted.begin(), sorted.end(),
                      [](const auto* a, const auto* b) { return a->key < b->key; });
            for (size_t i = 1; i < sorted.size(); ++i) {
                if (sorted[i]->key == sorted[i - 1]->key) {
                    return svfail("duplicate struct field key");
                }
            }
            for (const auto& field : node.fields) {
                if (field.presence != FieldPresence::Required) continue;
                const bool present = std::any_of(
                    sorted.begin(), sorted.end(),
                    [&](const auto* f) { return f->key == field.key; });
                if (!present) return svfail("missing required struct field");
            }
            w.putU64Varint(sorted.size());
            for (const auto* f : sorted) {
                const SchemaField* sf = f->schemaField;
                if (!sf) {
                    const auto it = std::lower_bound(
                        node.fields.begin(), node.fields.end(), f->key,
                        [](const SchemaField& x, uint32_t k) { return x.key < k; });
                    if (it == node.fields.end() || it->key != f->key) {
                        return svfail("unknown struct field key");
                    }
                    sf = &*it;
                }
                if (!f->value) return svfail("null struct field value");
                auto child = encodeChild(graph, *f->value, depth + 1);
                if (!child) return std::unexpected(child.error());
                w.putU64Varint(f->key);
                w.putU64Varint(child->buf.size());
                w.putBytes(child->buf.data(), child->buf.size());
            }
            return {};
        }
        case SchemaKind::FixedArray: {
            const auto* a = std::get_if<SchemaValue::Array>(&value.payload);
            if (!a || !node.element || a->elements.size() != node.fixedCount) {
                return svfail("payload kind does not match fixed array schema");
            }
            const SchemaNode* elementNode = resolveAliasRef(graph, *node.element);
            if (!elementNode) return svfail("unresolvable element schema");
            for (const auto& el : a->elements) {
                if (auto r = encodeNode(graph, *elementNode, el, w, depth + 1); !r) {
                    return r;
                }
            }
            return {};
        }
        case SchemaKind::DynamicArray: {
            const auto* a = std::get_if<SchemaValue::Array>(&value.payload);
            if (!a || !node.element) {
                return svfail("payload kind does not match array schema");
            }
            w.putU64Varint(a->elements.size());
            for (const auto& el : a->elements) {
                auto child = encodeChild(graph, el, depth + 1);
                if (!child) return std::unexpected(child.error());
                w.putU64Varint(child->buf.size());
                w.putBytes(child->buf.data(), child->buf.size());
            }
            return {};
        }
        case SchemaKind::Map: {
            const auto* m = std::get_if<SchemaValue::Map>(&value.payload);
            if (!m || !node.mapKey || !node.mapValue) {
                return svfail("payload kind does not match map schema");
            }
            struct EncodedEntry { VecWriter key; VecWriter value; };
            std::vector<EncodedEntry> encoded;
            encoded.reserve(m->entries.size());
            for (const auto& e : m->entries) {
                if (!e.key || !e.value) return svfail("null map entry");
                auto k = encodeChild(graph, *e.key, depth + 1);
                if (!k) return std::unexpected(k.error());
                auto v = encodeChild(graph, *e.value, depth + 1);
                if (!v) return std::unexpected(v.error());
                encoded.push_back({std::move(*k), std::move(*v)});
            }
            // Canonical key order.
            std::sort(encoded.begin(), encoded.end(),
                      [&](const EncodedEntry& a, const EncodedEntry& b) {
                          int cmp = 0;
                          if (!schema_value_codec_detail::compareMapKeys(
                                  graph, *node.mapKey, a.key.buf, b.key.buf, cmp)) {
                              return false;
                          }
                          return cmp < 0;
                      });
            for (size_t i = 1; i < encoded.size(); ++i) {
                int cmp = 0;
                if (!schema_value_codec_detail::compareMapKeys(
                        graph, *node.mapKey, encoded[i - 1].key.buf,
                        encoded[i].key.buf, cmp) || cmp >= 0) {
                    return svfail("map keys not unique or unordered");
                }
            }
            w.putU64Varint(encoded.size());
            for (const auto& e : encoded) {
                w.putU64Varint(e.key.buf.size());
                w.putBytes(e.key.buf.data(), e.key.buf.size());
                w.putU64Varint(e.value.buf.size());
                w.putBytes(e.value.buf.data(), e.value.buf.size());
            }
            return {};
        }
        case SchemaKind::Optional: {
            const auto* o = std::get_if<SchemaValue::Optional>(&value.payload);
            if (!o || !node.element) {
                return svfail("payload kind does not match optional schema");
            }
            w.putU8(o->present ? 1 : 0);
            if (o->present) {
                if (!o->value) return svfail("present optional without value");
                const SchemaNode* elementNode =
                    resolveAliasRef(graph, *node.element);
                if (!elementNode) return svfail("unresolvable element schema");
                if (auto r = encodeNode(graph, *elementNode, *o->value, w, depth + 1);
                    !r) return r;
            }
            return {};
        }
        case SchemaKind::OneOf: {
            const auto* o = std::get_if<SchemaValue::OneOf>(&value.payload);
            if (!o || o->memberKey == 0 || !o->value) {
                return svfail("payload kind does not match oneof schema");
            }
            const auto it = std::lower_bound(
                node.oneOfMembers.begin(), node.oneOfMembers.end(), o->memberKey,
                [](const auto& item, uint32_t k) { return item.first < k; });
            if (it == node.oneOfMembers.end() || it->first != o->memberKey) {
                return svfail("unknown oneof member");
            }
            const SchemaNode* memberNode = resolveAliasRef(graph, it->second);
            if (!memberNode) return svfail("unresolvable oneof member schema");
            VecWriter payload;
            if (auto r = encodeNode(graph, *memberNode, *o->value, payload, depth + 1);
                !r) return r;
            w.putU64Varint(o->memberKey);
            w.putU64Varint(payload.buf.size());
            w.putBytes(payload.buf.data(), payload.buf.size());
            return {};
        }
        case SchemaKind::Alias:
            break;
    }
    return svfail("unhandled schema kind");
}

} // namespace schema_value_detail

/// Encode a SchemaValue tree into canonical wire bytes.
/// `value.node` must point into `graph` (aliases are chased).
inline std::expected<std::vector<uint8_t>, std::string>
encodeSchemaValue(const SchemaGraph& graph, const SchemaValue& value) {
    const SchemaNode* node = schema_value_detail::resolveAliasNode(graph, value.node);
    if (!node) return std::unexpected(std::string("unresolvable schema node"));
    schema_value_detail::VecWriter w;
    if (auto r = schema_value_detail::encodeNode(graph, *node, value, w, 0); !r) {
        return std::unexpected(r.error());
    }
    return std::move(w.buf);
}

// ===========================================================================
// Formatting — human text and JSON
// ===========================================================================

namespace schema_value_detail {

inline std::string hexBytes(const uint8_t* data, size_t len) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        out.push_back(kHex[data[i] >> 4]);
        out.push_back(kHex[data[i] & 0x0F]);
    }
    return out;
}

inline std::string escapeText(std::string_view s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out += std::format("\\x{:02x}", static_cast<unsigned char>(c));
                } else {
                    out.push_back(c);
                }
        }
    }
    return out;
}

inline std::string escapeJson(std::string_view s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out += std::format("\\u{:04x}", static_cast<unsigned char>(c));
                } else {
                    out.push_back(c);
                }
        }
    }
    return out;
}

inline int64_t scalarAsSigned(uint64_t bits) {
    return static_cast<int64_t>(bits);
}

inline double scalarAsDouble(const SchemaNode& node, uint64_t bits) {
    if (node.scalarType == ValueType::F32) {
        const uint32_t raw = static_cast<uint32_t>(bits);
        float f;
        std::memcpy(&f, &raw, 4);
        return static_cast<double>(f);
    }
    double d;
    std::memcpy(&d, &bits, 8);
    return d;
}

inline bool isSignedScalar(ValueType t) {
    return t == ValueType::I8 || t == ValueType::I16 || t == ValueType::I32 ||
           t == ValueType::I64 || t == ValueType::IVarint;
}

inline bool isFloatScalar(ValueType t) {
    return t == ValueType::F32 || t == ValueType::F64;
}

inline std::string formatScalarText(const SchemaNode& node, uint64_t bits) {
    switch (node.scalarType) {
        case ValueType::Bool: return bits ? "true" : "false";
        case ValueType::F32: case ValueType::F64:
            return std::format("{}", scalarAsDouble(node, bits));
        case ValueType::IVarint: case ValueType::I8: case ValueType::I16:
        case ValueType::I32: case ValueType::I64:
            return std::format("{}", scalarAsSigned(bits));
        default:
            return std::format("{}", bits);
    }
}

/// Format a byte-typed scalar (IPv4/IPv6/MAC) or bytes payload.
inline std::string formatBytesText(const SchemaNode* node,
                                   const std::vector<uint8_t>& b) {
    if (node && node->scalarType == ValueType::IPv4 && b.size() == 4) {
        return std::format("{}.{}.{}.{}", b[0], b[1], b[2], b[3]);
    }
    if (node && node->scalarType == ValueType::MAC && b.size() == 6) {
        return std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                           b[0], b[1], b[2], b[3], b[4], b[5]);
    }
    if (node && node->scalarType == ValueType::IPv6 && b.size() == 16) {
        std::string out;
        for (size_t i = 0; i < 16; i += 2) {
            if (i) out.push_back(':');
            out += std::format("{:02x}{:02x}", b[i], b[i + 1]);
        }
        return out;
    }
    return "hex:" + hexBytes(b.data(), b.size());
}

inline void formatText(const SchemaValue& v, std::string& out,
                       size_t maxElements, size_t maxString, uint32_t depth) {
    const SchemaNode* node = v.node;
    if (depth > 64) { out += "…"; return; }
    if (const auto* s = std::get_if<SchemaValue::Scalar>(&v.payload)) {
        out += node ? formatScalarText(*node, s->bits) : std::format("{}", s->bits);
        return;
    }
    if (const auto* s = std::get_if<SchemaValue::Str>(&v.payload)) {
        const std::string& str = s->value;
        const bool truncated = maxString && str.size() > maxString;
        const std::string_view shown(str.data(),
            truncated ? maxString : str.size());
        out += '"';
        out += escapeText(shown);
        out += '"';
        if (truncated) out += "…";
        return;
    }
    if (const auto* b = std::get_if<SchemaValue::Bytes>(&v.payload)) {
        out += formatBytesText(node, b->value);
        return;
    }
    if (const auto* a = std::get_if<SchemaValue::Array>(&v.payload)) {
        out += '[';
        size_t shown = a->elements.size();
        if (maxElements && shown > maxElements) shown = maxElements;
        for (size_t i = 0; i < shown; ++i) {
            if (i) out += ", ";
            formatText(a->elements[i], out, maxElements, maxString, depth + 1);
        }
        if (shown < a->elements.size()) {
            out += std::format(", … +{}", a->elements.size() - shown);
        }
        out += ']';
        return;
    }
    if (const auto* st = std::get_if<SchemaValue::Struct>(&v.payload)) {
        out += '{';
        size_t shown = st->fields.size();
        if (maxElements && shown > maxElements) shown = maxElements;
        for (size_t i = 0; i < shown; ++i) {
            const auto& f = st->fields[i];
            if (i) out += ", ";
            if (f.schemaField && !f.schemaField->name.empty()) {
                out += f.schemaField->name;
            } else {
                out += std::format("#{}", f.key);
            }
            out += ": ";
            if (f.value) formatText(*f.value, out, maxElements, maxString, depth + 1);
            else out += "null";
        }
        if (shown < st->fields.size()) {
            out += std::format(", … +{}", st->fields.size() - shown);
        }
        out += '}';
        return;
    }
    if (const auto* m = std::get_if<SchemaValue::Map>(&v.payload)) {
        out += '{';
        size_t shown = m->entries.size();
        if (maxElements && shown > maxElements) shown = maxElements;
        for (size_t i = 0; i < shown; ++i) {
            const auto& e = m->entries[i];
            if (i) out += ", ";
            if (e.key) formatText(*e.key, out, maxElements, maxString, depth + 1);
            else out += "?";
            out += ": ";
            if (e.value) formatText(*e.value, out, maxElements, maxString, depth + 1);
            else out += "null";
        }
        if (shown < m->entries.size()) {
            out += std::format(", … +{}", m->entries.size() - shown);
        }
        out += '}';
        return;
    }
    if (const auto* o = std::get_if<SchemaValue::Optional>(&v.payload)) {
        if (!o->present || !o->value) out += "none";
        else formatText(*o->value, out, maxElements, maxString, depth + 1);
        return;
    }
    if (const auto* o = std::get_if<SchemaValue::OneOf>(&v.payload)) {
        // The member node's own name (carried on the decoded child) is the
        // display name; fall back to the numeric member key.
        const char* memberName = nullptr;
        if (o->value && o->value->node && !o->value->node->name.empty()) {
            memberName = o->value->node->name.c_str();
        }
        if (memberName) out += memberName;
        else out += std::format("#{}", o->memberKey);
        out += '{';
        if (o->value) formatText(*o->value, out, maxElements, maxString, depth + 1);
        out += '}';
        return;
    }
    out += "<empty>";
}

/// Look up a oneof member's display name via its member node (graph needed).
inline std::string oneOfMemberName(const SchemaGraph* graph,
                                   const SchemaNode& node, uint32_t memberKey) {
    const auto it = std::lower_bound(
        node.oneOfMembers.begin(), node.oneOfMembers.end(), memberKey,
        [](const auto& item, uint32_t k) { return item.first < k; });
    if (it != node.oneOfMembers.end() && it->first == memberKey && graph) {
        if (const SchemaNode* member = graph->find(it->second.key)) {
            if (!member->name.empty()) return member->name;
        }
    }
    return std::format("#{}", memberKey);
}

inline void formatJson(const SchemaGraph* graph, const SchemaValue& v,
                       std::string& out, uint32_t depth) {
    const SchemaNode* node = v.node;
    if (depth > 64) { out += "null"; return; }
    if (const auto* s = std::get_if<SchemaValue::Scalar>(&v.payload)) {
        const ValueType t = node ? node->scalarType : ValueType::U64;
        if (t == ValueType::Bool) {
            out += s->bits ? "true" : "false";
        } else if (t == ValueType::F32 || t == ValueType::F64) {
            const double d = scalarAsDouble(*node, s->bits);
            if (std::isfinite(d)) out += std::format("{}", d);
            else out += "null";
        } else if (isSignedScalar(t)) {
            const int64_t x = static_cast<int64_t>(s->bits);
            // Keep integers exactly representable in JS as numbers.
            if (x > -(1LL << 53) && x < (1LL << 53)) out += std::format("{}", x);
            else out += "\"" + std::format("{}", x) + "\"";
        } else {
            if (s->bits < (1ULL << 53)) out += std::format("{}", s->bits);
            else out += "\"" + std::format("{}", s->bits) + "\"";
        }
        return;
    }
    if (const auto* s = std::get_if<SchemaValue::Str>(&v.payload)) {
        out += '"';
        out += escapeJson(s->value);
        out += '"';
        return;
    }
    if (const auto* b = std::get_if<SchemaValue::Bytes>(&v.payload)) {
        out += '"';
        out += formatBytesText(node, b->value);
        out += '"';
        return;
    }
    if (const auto* a = std::get_if<SchemaValue::Array>(&v.payload)) {
        out += '[';
        for (size_t i = 0; i < a->elements.size(); ++i) {
            if (i) out += ',';
            formatJson(graph, a->elements[i], out, depth + 1);
        }
        out += ']';
        return;
    }
    if (const auto* st = std::get_if<SchemaValue::Struct>(&v.payload)) {
        out += '{';
        for (size_t i = 0; i < st->fields.size(); ++i) {
            const auto& f = st->fields[i];
            if (i) out += ',';
            out += '"';
            if (f.schemaField && !f.schemaField->name.empty()) {
                out += escapeJson(f.schemaField->name);
            } else {
                out += std::format("#{}", f.key);
            }
            out += "\":";
            if (f.value) formatJson(graph, *f.value, out, depth + 1);
            else out += "null";
        }
        out += '}';
        return;
    }
    if (const auto* m = std::get_if<SchemaValue::Map>(&v.payload)) {
        // Non-string keys can't be JSON object keys — emit [k, v] pairs.
        out += '[';
        for (size_t i = 0; i < m->entries.size(); ++i) {
            const auto& e = m->entries[i];
            if (i) out += ',';
            out += '[';
            if (e.key) formatJson(graph, *e.key, out, depth + 1);
            else out += "null";
            out += ',';
            if (e.value) formatJson(graph, *e.value, out, depth + 1);
            else out += "null";
            out += ']';
        }
        out += ']';
        return;
    }
    if (const auto* o = std::get_if<SchemaValue::Optional>(&v.payload)) {
        if (!o->present || !o->value) out += "null";
        else formatJson(graph, *o->value, out, depth + 1);
        return;
    }
    if (const auto* o = std::get_if<SchemaValue::OneOf>(&v.payload)) {
        out += '{';
        out += '"';
        out += escapeJson(node ? oneOfMemberName(graph, *node, o->memberKey)
                               : std::format("#{}", o->memberKey));
        out += "\":";
        if (o->value) formatJson(graph, *o->value, out, depth + 1);
        else out += "null";
        out += '}';
        return;
    }
    out += "null";
}

} // namespace schema_value_detail

/// Human-readable single-line rendering: `42`, `"str"`, `{a: 1, b: [1, 2]}`,
/// `hex:aa55`, `none`, `#3{...}`.
/// `maxElements`/`maxString` truncate for compact display (0 = unlimited).
inline std::string formatSchemaValueText(const SchemaValue& value,
                                         size_t maxElements = 0,
                                         size_t maxString = 0) {
    std::string out;
    schema_value_detail::formatText(value, out, maxElements, maxString, 0);
    return out;
}

/// JSON rendering: structs/objects for structs, arrays, `[k, v]` pair lists
/// for maps (JSON object keys must be strings), `{"<member>": v}` for oneof,
/// null for absent optionals, `"hex:…"`-style strings for byte payloads,
/// integers above 2^53 emitted as strings.
/// `graph` is optional — pass it for named oneof members.
inline std::string schemaValueToJson(const SchemaValue& value,
                                     const SchemaGraph* graph = nullptr) {
    std::string out;
    schema_value_detail::formatJson(graph, value, out, 0);
    return out;
}

// ===========================================================================
// Typed accessors (for CLI/TUI use)
// ===========================================================================

inline const SchemaValue::Scalar* schemaScalar(const SchemaValue& v) {
    return std::get_if<SchemaValue::Scalar>(&v.payload);
}
inline const SchemaValue::Str* schemaStr(const SchemaValue& v) {
    return std::get_if<SchemaValue::Str>(&v.payload);
}
inline const SchemaValue::Bytes* schemaBytes(const SchemaValue& v) {
    return std::get_if<SchemaValue::Bytes>(&v.payload);
}
inline const SchemaValue::Array* schemaArray(const SchemaValue& v) {
    return std::get_if<SchemaValue::Array>(&v.payload);
}
inline const SchemaValue::Struct* schemaStruct(const SchemaValue& v) {
    return std::get_if<SchemaValue::Struct>(&v.payload);
}

/// Signed/unsigned/double projections of a Scalar payload.
inline int64_t schemaValueAsInt(const SchemaValue& v) {
    if (const auto* s = schemaScalar(v)) return static_cast<int64_t>(s->bits);
    return 0;
}
inline uint64_t schemaValueAsUint(const SchemaValue& v) {
    if (const auto* s = schemaScalar(v)) return s->bits;
    return 0;
}
inline double schemaValueAsDouble(const SchemaValue& v) {
    if (const auto* s = schemaScalar(v)) {
        if (v.node && (v.node->scalarType == ValueType::F32 ||
                       v.node->scalarType == ValueType::F64)) {
            return schema_value_detail::scalarAsDouble(*v.node, s->bits);
        }
        return static_cast<double>(static_cast<int64_t>(s->bits));
    }
    return 0.0;
}
inline bool schemaValueAsBool(const SchemaValue& v) {
    if (const auto* s = schemaScalar(v)) return s->bits != 0;
    return false;
}

// ===========================================================================
// Tree builders (for set/call encoders and tests)
// ===========================================================================

inline SchemaValue makeScalarValue(const SchemaNode* node, uint64_t bits) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Scalar{bits};
    return v;
}
inline SchemaValue makeStringValue(const SchemaNode* node, std::string s) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Str{std::move(s)};
    return v;
}
inline SchemaValue makeBytesValue(const SchemaNode* node, std::vector<uint8_t> b) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Bytes{std::move(b)};
    return v;
}
inline SchemaValue makeArrayValue(const SchemaNode* node, std::vector<SchemaValue> elems) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Array{std::move(elems)};
    return v;
}
inline SchemaValue makeAbsentValue(const SchemaNode* node) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Optional{false, nullptr};
    return v;
}
inline SchemaValue makeOptionalValue(const SchemaNode* node, SchemaValue inner) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Optional{
        true, std::make_unique<SchemaValue>(std::move(inner))};
    return v;
}
inline SchemaValue makeOneOfValue(const SchemaNode* node, uint32_t memberKey,
                                  SchemaValue inner) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::OneOf{
        memberKey, std::make_unique<SchemaValue>(std::move(inner))};
    return v;
}
inline SchemaValue::Struct::Field makeStructField(uint32_t key,
                                                  const SchemaField* schemaField,
                                                  SchemaValue value) {
    SchemaValue::Struct::Field f;
    f.key = key;
    f.schemaField = schemaField;
    f.value = std::make_unique<SchemaValue>(std::move(value));
    return f;
}
inline SchemaValue makeStructValue(const SchemaNode* node,
                                   std::vector<SchemaValue::Struct::Field> fields) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Struct{std::move(fields)};
    return v;
}
inline SchemaValue::Map::Entry makeMapEntry(SchemaValue key, SchemaValue value) {
    SchemaValue::Map::Entry e;
    e.key = std::make_unique<SchemaValue>(std::move(key));
    e.value = std::make_unique<SchemaValue>(std::move(value));
    return e;
}
inline SchemaValue makeMapValue(const SchemaNode* node,
                                std::vector<SchemaValue::Map::Entry> entries) {
    SchemaValue v;
    v.node = node;
    v.payload = SchemaValue::Map{std::move(entries)};
    return v;
}

} // namespace tether::io
