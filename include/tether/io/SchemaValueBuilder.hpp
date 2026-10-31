/**
 * @file SchemaValueBuilder.hpp
 * @brief Native → SchemaValue builders and SchemaValue → native accessors
 *
 * decodeSchemaValue()/encodeSchemaValue() move between wire bytes and
 * SchemaValue trees; this header bridges the last mile: building a
 * SchemaValue from ordinary C++ values (ints, doubles, spans, nested
 * field lists) with every leaf checked against the declared schema, and
 * reading decoded values back into native types.
 *
 * ## Build side
 *
 * `nv::*` functions produce a lazily-typed NativeValue;
 * buildSchemaValue(graph, node, native) walks the schema and turns it
 * into a fully-typed SchemaValue tree — range-checking integer widths,
 * array element types, struct field names, and packed field counts
 * against `node`, so a wrong leaf fails with a message rather than
 * emitting malformed wire bytes.
 *
 * @code
 *   auto v = buildSchemaValue(graph, frameNode, nv::fields(
 *       nv::field("seq",      nv::u64(fr.seq)),
 *       nv::field("sim_time", nv::f64(fr.sim_time)),
 *       nv::field("name",     nv::str(fr.name)),
 *       nv::field("q_des",    nv::f64v(fr.q_des))));
 *   if (!v) fail(v.error());
 *   auto bytes = encodeSchemaValue(graph, *v);
 * @endcode
 *
 * ## Access side
 *
 * `asX(value)` getters extract native values from a decoded tree —
 * e.g. `decodeSchemaValue(...)` then `asF64Array(*v)` replaces a hand
 * rolled array decoder. `fieldValue(structVal, "name")` navigates named
 * struct fields.
 *
 * ## Contracts checked at the boundary
 *
 *   - Integer scalars: the variant arm must match the declared signedness
 *     (nv::u for unsigned/enum/varint, nv::i for signed) and the value
 *     must fit the declared width — a narrowing mismatch is an error,
 *     never a silent truncation.
 *   - FixedArray: element count must equal the declared fixedCount.
 *   - Packed struct: every declared field must be provided exactly once;
 *     tagged struct: names must resolve (required-field presence is
 *     enforced by encodeSchemaValue).
 *   - Strings: embedded NUL rejected (the wire form is NUL-terminated).
 *   - Non-owning inputs (string_view, spans) must outlive the build call;
 *     the produced SchemaValue owns copies.
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <expected>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "tether/io/SchemaValue.hpp"

namespace tether::io {

// ===========================================================================
// NativeValue — lazily-typed input for buildSchemaValue()
// ===========================================================================

struct NativeValue;

/// Struct member: field name → value.
struct NativeField {
    std::string_view name;
    std::unique_ptr<NativeValue> value;
    /// Defined below (needs a complete NativeValue) so
    /// `{"name", nv::x}` braced lists build fields directly.
    NativeField(std::string_view n, NativeValue v);
    NativeField(NativeField&&) = default;
    NativeField& operator=(NativeField&&) = default;
    NativeField(const NativeField&) = delete;
    NativeField& operator=(const NativeField&) = delete;
};
/// Map entry: key → value (each independently typed).
struct NativeMapEntry {
    std::unique_ptr<NativeValue> key;
    std::unique_ptr<NativeValue> value;
    NativeMapEntry(NativeValue k, NativeValue v);
    NativeMapEntry(NativeMapEntry&&) = default;
    NativeMapEntry& operator=(NativeMapEntry&&) = default;
    NativeMapEntry(const NativeMapEntry&) = delete;
    NativeMapEntry& operator=(const NativeMapEntry&) = delete;
};

struct NativeValue {
    /// Absent value — an empty Optional.
    struct None {};
    /// Tagged union member.
    struct OneOf {
        uint32_t key = 0;
        std::unique_ptr<NativeValue> value;
    };
    /// Named fields for a struct node.
    using Fields = std::vector<NativeField>;
    /// Entries for a map node.
    using MapEntries = std::vector<NativeMapEntry>;
    /// Heterogeneous/nested array contents (e.g. array of struct).
    using Items = std::vector<NativeValue>;

    std::variant<None, bool, int64_t, uint64_t, double,
                 std::string_view,               // String
                 std::span<const uint8_t>,       // Bytes / byte scalars
                 std::span<const double>,        // f64 array
                 std::span<const int64_t>,       // signed int array
                 std::span<const uint64_t>,      // unsigned int array
                 std::span<const std::string>,   // string array
                 Items, Fields, MapEntries, OneOf>
        payload;
};

inline NativeField::NativeField(std::string_view n, NativeValue v)
    : name(n), value(std::make_unique<NativeValue>(std::move(v))) {}
inline NativeMapEntry::NativeMapEntry(NativeValue k, NativeValue v)
    : key(std::make_unique<NativeValue>(std::move(k))),
      value(std::make_unique<NativeValue>(std::move(v))) {}

namespace nv {

/// Boolean scalar (Bool schema only).
inline NativeValue flag(bool v) { return NativeValue{{v}}; }
/// Unsigned scalar: U8/U16/U32/U64/UVarint/Enum/Stream, width-checked.
inline NativeValue u(uint64_t v) { return NativeValue{{v}}; }
inline NativeValue u8(uint8_t v) { return u(v); }
inline NativeValue u16(uint16_t v) { return u(v); }
inline NativeValue u32(uint32_t v) { return u(v); }
inline NativeValue u64(uint64_t v) { return u(v); }
/// Signed scalar: I8/I16/I32/I64/IVarint, width-checked.
inline NativeValue i(int64_t v) { return NativeValue{{v}}; }
inline NativeValue i32(int32_t v) { return i(v); }
inline NativeValue i64(int64_t v) { return i(v); }
/// Float scalar: F32 (rounds to f32) or F64.
inline NativeValue f(double v) { return NativeValue{{v}}; }
inline NativeValue f64(double v) { return f(v); }
/// String value (embedded NUL rejected at build).
inline NativeValue str(std::string_view v) { return NativeValue{{v}}; }
/// Raw bytes.
inline NativeValue bytes(std::span<const uint8_t> v) {
    return NativeValue{{v}};
}
/// f64 vector for FixedArray<F64>/DynamicArray<F64>.
inline NativeValue f64v(std::span<const double> v) {
    return NativeValue{{v}};
}
/// Signed-int vector for arrays of signed scalars.
inline NativeValue i64v(std::span<const int64_t> v) {
    return NativeValue{{v}};
}
/// Unsigned-int vector for arrays of unsigned scalars.
inline NativeValue u64v(std::span<const uint64_t> v) {
    return NativeValue{{v}};
}
/// String vector for DynamicArray<String>.
inline NativeValue strv(std::span<const std::string> v) {
    return NativeValue{{v}};
}
/// Generic array contents (nested/heterogeneous element types).
inline NativeValue items(std::vector<NativeValue> v) {
    return NativeValue{{std::move(v)}};
}
/// Variadic array items: items(nv::x, nv::y, ...).
template <typename... Vs>
inline NativeValue items(Vs&&... vs) {
    std::vector<NativeValue> v;
    v.reserve(sizeof...(vs));
    (v.push_back(std::forward<Vs>(vs)), ...);
    return items(std::move(v));
}

/// One struct field (name → value).
inline NativeField field(std::string_view name, NativeValue v) {
    return NativeField{name, std::move(v)};
}
/// Struct fields by name (a runtime-built list).
inline NativeValue fields(std::vector<NativeField> v) {
    return NativeValue{{std::move(v)}};
}
/// Variadic struct fields: fields(field("a", nv::x), field("b", nv::y)).
/// NOTE: initializer_list cannot hold move-only NativeField — use this
/// overload or push_back into a vector.
template <typename... Fs>
inline NativeValue fields(Fs&&... fs) {
    std::vector<NativeField> v;
    v.reserve(sizeof...(fs));
    (v.push_back(NativeField{std::forward<Fs>(fs)}), ...);
    return fields(std::move(v));
}

/// One map entry.
inline NativeMapEntry mapEntry(NativeValue k, NativeValue v) {
    return NativeMapEntry{std::move(k), std::move(v)};
}
/// Map entries (a runtime-built list).
inline NativeValue map(std::vector<NativeMapEntry> v) {
    return NativeValue{{std::move(v)}};
}
/// Variadic map entries: map(mapEntry(k1, v1), mapEntry(k2, v2)).
template <typename... Es>
inline NativeValue map(Es&&... es) {
    std::vector<NativeMapEntry> v;
    v.reserve(sizeof...(es));
    (v.push_back(NativeMapEntry{std::forward<Es>(es)}), ...);
    return map(std::move(v));
}
/// Absent Optional value.
inline NativeValue none() { return NativeValue{{NativeValue::None{}}}; }
/// OneOf member.
inline NativeValue oneOf(uint32_t key, NativeValue v) {
    return NativeValue{
        {NativeValue::OneOf{key, std::make_unique<NativeValue>(std::move(v))}}};
}

}  // namespace nv

// ===========================================================================
// buildSchemaValue — NativeValue → SchemaValue, checked against the schema
// ===========================================================================

namespace schema_value_build_detail {

inline auto bfail(const std::string& msg) {
    return std::unexpected(msg);
}

inline const SchemaNode* resolve(const SchemaGraph& g, const SchemaNode* n) {
    while (n && n->kind == SchemaKind::Alias) {
        if (!n->target) return nullptr;
        n = g.find(n->target->key);
    }
    return n;
}
inline const SchemaNode* resolveRef(const SchemaGraph& g, const SchemaRef& r) {
    return resolve(g, g.find(r.key));
}

template <typename UInt>
inline bool fitsUnsigned(uint64_t v) {
    return v <= static_cast<uint64_t>(std::numeric_limits<UInt>::max());
}
template <typename SInt>
inline bool fitsSigned(int64_t v) {
    return v >= static_cast<int64_t>(std::numeric_limits<SInt>::min()) &&
           v <= static_cast<int64_t>(std::numeric_limits<SInt>::max());
}

inline std::expected<SchemaValue, std::string>
build(const SchemaGraph& g, const SchemaNode& node, const NativeValue& nv,
      uint32_t depth);

inline std::expected<SchemaValue, std::string>
buildRef(const SchemaGraph& g, const SchemaRef& ref, const NativeValue& nv,
         uint32_t depth) {
    const SchemaNode* node = resolveRef(g, ref);
    if (!node) return bfail("unresolvable schema ref");
    return build(g, *node, nv, depth + 1);
}

inline std::expected<SchemaValue, std::string>
buildScalar(const SchemaNode& node, const NativeValue& nv) {
    SchemaValue v;
    v.node = &node;
    SchemaValue::Scalar s;
    const auto t = node.scalarType;
    if (const auto* b = std::get_if<bool>(&nv.payload)) {
        if (t != ValueType::Bool) return bfail("bool into non-Bool scalar");
        s.bits = *b ? 1 : 0;
    } else if (const auto* u = std::get_if<uint64_t>(&nv.payload)) {
        switch (t) {
            case ValueType::Bool:
                if (*u > 1) return bfail("bool value out of range");
                s.bits = *u;
                break;
            case ValueType::U8:
                if (!fitsUnsigned<uint8_t>(*u)) return bfail("U8 overflow");
                s.bits = *u;
                break;
            case ValueType::U16:
                if (!fitsUnsigned<uint16_t>(*u)) return bfail("U16 overflow");
                s.bits = *u;
                break;
            case ValueType::U32: case ValueType::Stream:
                if (!fitsUnsigned<uint32_t>(*u)) return bfail("U32 overflow");
                s.bits = *u;
                break;
            case ValueType::U64: case ValueType::UVarint:
            case ValueType::Enum:
                s.bits = *u;
                break;
            default:
                return bfail("unsigned value into non-unsigned scalar");
        }
    } else if (const auto* i = std::get_if<int64_t>(&nv.payload)) {
        switch (t) {
            case ValueType::I8:
                if (!fitsSigned<int8_t>(*i)) return bfail("I8 overflow");
                s.bits = static_cast<uint64_t>(*i);
                break;
            case ValueType::I16:
                if (!fitsSigned<int16_t>(*i)) return bfail("I16 overflow");
                s.bits = static_cast<uint64_t>(*i);
                break;
            case ValueType::I32:
                if (!fitsSigned<int32_t>(*i)) return bfail("I32 overflow");
                s.bits = static_cast<uint64_t>(*i);
                break;
            case ValueType::I64: case ValueType::IVarint:
                s.bits = static_cast<uint64_t>(*i);
                break;
            default:
                return bfail("signed value into non-signed scalar");
        }
    } else if (const auto* d = std::get_if<double>(&nv.payload)) {
        switch (t) {
            case ValueType::F32: {
                const float f = static_cast<float>(*d);
                uint32_t bits = 0;
                std::memcpy(&bits, &f, 4);
                s.bits = bits;
                break;
            }
            case ValueType::F64:
                std::memcpy(&s.bits, d, 8);
                break;
            default:
                return bfail("double into non-float scalar");
        }
    } else if (const auto* raw = std::get_if<std::span<const uint8_t>>(
                   &nv.payload)) {
        const size_t need = t == ValueType::IPv4  ? 4
                            : t == ValueType::IPv6 ? 16
                            : t == ValueType::MAC  ? 6
                                                   : 0;
        if (need == 0 || raw->size() != need)
            return bfail("byte scalar length mismatch");
        v.payload = SchemaValue::Bytes{
            std::vector<uint8_t>(raw->begin(), raw->end())};
        return v;
    } else {
        return bfail("payload kind does not match scalar schema");
    }
    v.payload = s;
    return v;
}

template <typename T, typename BuildOne>
inline std::expected<SchemaValue, std::string>
buildArrayFromSpan(const SchemaGraph& g, const SchemaNode& node,
                   std::span<const T> values, BuildOne&& one, uint32_t depth) {
    if (node.kind == SchemaKind::FixedArray &&
        values.size() != node.fixedCount) {
        return bfail("fixed array count mismatch");
    }
    SchemaValue v;
    v.node = &node;
    SchemaValue::Array a;
    a.elements.reserve(values.size());
    for (const auto& x : values) {
        auto el = one(x);
        if (!el) return el;
        a.elements.push_back(std::move(*el));
    }
    v.payload = std::move(a);
    return v;
}

inline std::expected<SchemaValue, std::string>
build(const SchemaGraph& g, const SchemaNode& node, const NativeValue& nv,
      uint32_t depth) {
    if (depth > SchemaValueLimits{}.maxDepth) {
        return bfail("schema value depth limit exceeded");
    }
    switch (node.kind) {
        case SchemaKind::Scalar:
        case SchemaKind::Enum:
            return buildScalar(node, nv);

        case SchemaKind::String: {
            const auto* s = std::get_if<std::string_view>(&nv.payload);
            if (!s) return bfail("payload kind does not match string schema");
            if (s->find('\0') != std::string_view::npos)
                return bfail("string contains NUL");
            SchemaValue v;
            v.node = &node;
            v.payload = SchemaValue::Str{std::string(*s)};
            return v;
        }

        case SchemaKind::Bytes: {
            const auto* b = std::get_if<std::span<const uint8_t>>(&nv.payload);
            if (!b) return bfail("payload kind does not match bytes schema");
            SchemaValue v;
            v.node = &node;
            v.payload =
                SchemaValue::Bytes{std::vector<uint8_t>(b->begin(), b->end())};
            return v;
        }

        case SchemaKind::Struct: {
            const auto* fs = std::get_if<NativeValue::Fields>(&nv.payload);
            if (!fs) return bfail("payload kind does not match struct schema");
            if (node.structEncoding == StructEncoding::Packed &&
                fs->size() != node.fields.size())
                return bfail("packed struct field count mismatch");
            SchemaValue v;
            v.node = &node;
            SchemaValue::Struct st;
            st.fields.reserve(fs->size());
            for (const auto& nf : *fs) {
                if (!nf.value) return bfail("null native field value");
                const auto it = std::find_if(
                    node.fields.begin(), node.fields.end(),
                    [&](const SchemaField& f) { return f.name == nf.name; });
                if (it == node.fields.end())
                    return bfail("unknown struct field name: " +
                                 std::string(nf.name));
                auto child = buildRef(g, it->schema, *nf.value, depth);
                if (!child) return std::unexpected(child.error());
                SchemaValue::Struct::Field f;
                f.key = it->key;
                f.schemaField = &*it;
                f.value = std::make_unique<SchemaValue>(std::move(*child));
                st.fields.push_back(std::move(f));
            }
            v.payload = std::move(st);
            return v;
        }

        case SchemaKind::FixedArray:
        case SchemaKind::DynamicArray: {
            if (!node.element) return bfail("array without element schema");
            const SchemaNode* el = resolveRef(g, *node.element);
            if (!el) return bfail("unresolvable array element schema");
            // Typed fast paths by element kind.
            if (const auto* d =
                    std::get_if<std::span<const double>>(&nv.payload)) {
                return buildArrayFromSpan<double>(
                    g, node, *d,
                    [&](double x) { return build(g, *el, nv::f64(x), depth); },
                    depth);
            }
            if (const auto* i =
                    std::get_if<std::span<const int64_t>>(&nv.payload)) {
                return buildArrayFromSpan<int64_t>(
                    g, node, *i,
                    [&](int64_t x) { return build(g, *el, nv::i(x), depth); },
                    depth);
            }
            if (const auto* u =
                    std::get_if<std::span<const uint64_t>>(&nv.payload)) {
                return buildArrayFromSpan<uint64_t>(
                    g, node, *u,
                    [&](uint64_t x) { return build(g, *el, nv::u(x), depth); },
                    depth);
            }
            if (const auto* ss =
                    std::get_if<std::span<const std::string>>(&nv.payload)) {
                return buildArrayFromSpan<std::string>(
                    g, node, *ss,
                    [&](const std::string& x) {
                        return build(g, *el, nv::str(x), depth);
                    },
                    depth);
            }
            // Generic items: each element is its own NativeValue.
            if (const auto* items =
                    std::get_if<NativeValue::Items>(&nv.payload)) {
                if (node.kind == SchemaKind::FixedArray &&
                    items->size() != node.fixedCount)
                    return bfail("fixed array count mismatch");
                SchemaValue v;
                v.node = &node;
                SchemaValue::Array a;
                a.elements.reserve(items->size());
                for (const auto& item : *items) {
                    auto elv = build(g, *el, item, depth + 1);
                    if (!elv) return elv;
                    a.elements.push_back(std::move(*elv));
                }
                v.payload = std::move(a);
                return v;
            }
            return bfail("payload kind does not match array schema");
        }

        case SchemaKind::Map: {
            const auto* es =
                std::get_if<NativeValue::MapEntries>(&nv.payload);
            if (!es) return bfail("payload kind does not match map schema");
            if (!node.mapKey || !node.mapValue)
                return bfail("map without key/value schema");
            SchemaValue v;
            v.node = &node;
            SchemaValue::Map m;
            m.entries.reserve(es->size());
            for (const auto& e : *es) {
                if (!e.key || !e.value) return bfail("null map entry");
                auto k = buildRef(g, *node.mapKey, *e.key, depth);
                if (!k) return std::unexpected(k.error());
                auto val = buildRef(g, *node.mapValue, *e.value, depth);
                if (!val) return std::unexpected(val.error());
                SchemaValue::Map::Entry entry;
                entry.key = std::make_unique<SchemaValue>(std::move(*k));
                entry.value = std::make_unique<SchemaValue>(std::move(*val));
                m.entries.push_back(std::move(entry));
            }
            v.payload = std::move(m);
            return v;
        }

        case SchemaKind::Optional: {
            if (!node.element) return bfail("optional without element schema");
            SchemaValue v;
            v.node = &node;
            SchemaValue::Optional o;
            if (std::holds_alternative<NativeValue::None>(nv.payload)) {
                o.present = false;
            } else {
                o.present = true;
                auto inner = buildRef(g, *node.element, nv, depth);
                if (!inner) return std::unexpected(inner.error());
                o.value = std::make_unique<SchemaValue>(std::move(*inner));
            }
            v.payload = std::move(o);
            return v;
        }

        case SchemaKind::OneOf: {
            const auto* o = std::get_if<NativeValue::OneOf>(&nv.payload);
            if (!o || !o->value || o->key == 0)
                return bfail("payload kind does not match oneof schema");
            const auto it = std::lower_bound(
                node.oneOfMembers.begin(), node.oneOfMembers.end(), o->key,
                [](const auto& item, uint32_t k) { return item.first < k; });
            if (it == node.oneOfMembers.end() || it->first != o->key)
                return bfail("unknown oneof member key");
            SchemaValue v;
            v.node = &node;
            SchemaValue::OneOf out;
            out.memberKey = o->key;
            auto inner = buildRef(g, it->second, *o->value, depth);
            if (!inner) return std::unexpected(inner.error());
            out.value = std::make_unique<SchemaValue>(std::move(*inner));
            v.payload = std::move(out);
            return v;
        }

        case SchemaKind::Alias:
            break;  // resolved before dispatch — unreachable
    }
    return bfail("unhandled schema kind");
}

}  // namespace schema_value_build_detail

/**
 * @brief Build a SchemaValue tree from a NativeValue, checked against
 *        `node` (aliases chased).
 * @param graph  schema graph the node lives in
 * @param node   resolved schema node the value must satisfy
 * @param native lazily-typed input (nv::* builders)
 * @return the SchemaValue, or an error describing the first contract
 *         violation
 */
inline std::expected<SchemaValue, std::string>
buildSchemaValue(const SchemaGraph& graph, const SchemaNode& node,
                 const NativeValue& native) {
    const SchemaNode* resolved =
        schema_value_build_detail::resolve(graph, &node);
    if (!resolved) return std::unexpected(std::string("unresolvable schema node"));
    return schema_value_build_detail::build(graph, *resolved, native, 0);
}

/// Convenience: build then encode in one call.
inline std::expected<std::vector<uint8_t>, std::string>
encodeNative(const SchemaGraph& graph, const SchemaNode& node,
             const NativeValue& native) {
    auto v = buildSchemaValue(graph, node, native);
    if (!v) return std::unexpected(v.error());
    return encodeSchemaValue(graph, *v);
}

// ===========================================================================
// Accessors — SchemaValue → native
// ===========================================================================

/// Any unsigned-width scalar / enum / varint as u64.
inline std::expected<uint64_t, std::string> asU64(const SchemaValue& v) {
    const auto* s = std::get_if<SchemaValue::Scalar>(&v.payload);
    if (!s || !v.node) return std::unexpected(std::string("not a scalar"));
    return s->bits;
}

/// Any signed scalar as i64 (sign-restored from stored bits).
inline std::expected<int64_t, std::string> asI64(const SchemaValue& v) {
    const auto* s = std::get_if<SchemaValue::Scalar>(&v.payload);
    if (!s || !v.node) return std::unexpected(std::string("not a scalar"));
    return static_cast<int64_t>(s->bits);
}

/// F32/F64 scalar as double.
inline std::expected<double, std::string> asF64(const SchemaValue& v) {
    const auto* s = std::get_if<SchemaValue::Scalar>(&v.payload);
    if (!s || !v.node) return std::unexpected(std::string("not a scalar"));
    if (v.node->scalarType == ValueType::F32) {
        const uint32_t raw = static_cast<uint32_t>(s->bits);
        float f;
        std::memcpy(&f, &raw, 4);
        return static_cast<double>(f);
    }
    if (v.node->scalarType == ValueType::F64) {
        double d;
        std::memcpy(&d, &s->bits, 8);
        return d;
    }
    return std::unexpected(std::string("scalar is not a float"));
}

/// Bool scalar.
inline std::expected<bool, std::string> asBool(const SchemaValue& v) {
    const auto* s = std::get_if<SchemaValue::Scalar>(&v.payload);
    if (!s || !v.node || v.node->scalarType != ValueType::Bool)
        return std::unexpected(std::string("not a bool"));
    return s->bits != 0;
}

/// String value.
inline std::expected<std::string, std::string> asString(const SchemaValue& v) {
    const auto* s = std::get_if<SchemaValue::Str>(&v.payload);
    if (!s) return std::unexpected(std::string("not a string"));
    return s->value;
}

/// Array of F64 scalars (fixed or dynamic) → vector<double>.
inline std::expected<std::vector<double>, std::string>
asF64Array(const SchemaValue& v) {
    const auto* a = std::get_if<SchemaValue::Array>(&v.payload);
    if (!a) return std::unexpected(std::string("not an array"));
    std::vector<double> out;
    out.reserve(a->elements.size());
    for (const auto& el : a->elements) {
        auto d = asF64(el);
        if (!d) return std::unexpected(d.error());
        out.push_back(*d);
    }
    return out;
}

/// Array of strings → vector<string>.
inline std::expected<std::vector<std::string>, std::string>
asStringArray(const SchemaValue& v) {
    const auto* a = std::get_if<SchemaValue::Array>(&v.payload);
    if (!a) return std::unexpected(std::string("not an array"));
    std::vector<std::string> out;
    out.reserve(a->elements.size());
    for (const auto& el : a->elements) {
        auto s = asString(el);
        if (!s) return std::unexpected(s.error());
        out.push_back(std::move(*s));
    }
    return out;
}

/// Named-field lookup inside a decoded struct (by SchemaField::name,
/// then key). Returns nullptr when `v` is not a struct or the field is
/// absent (tagged structs may omit optional fields).
inline const SchemaValue* fieldValue(const SchemaValue& v,
                                     std::string_view name) {
    const auto* st = std::get_if<SchemaValue::Struct>(&v.payload);
    if (!st) return nullptr;
    for (const auto& f : st->fields) {
        if (f.schemaField && f.schemaField->name == name)
            return f.value.get();
    }
    return nullptr;
}

}  // namespace tether::io
