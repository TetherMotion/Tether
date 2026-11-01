/**
 * @file SchemaLayout.hpp
 * @brief Static wire-layout computation for schema nodes — the "fast
 *        path" contract for Tether IO values.
 *
 * ## Model
 *
 * Schema validation happens once — at manifest/negotiation time. After
 * that, a node whose subtree is entirely fixed-length encodes to the
 * same wire size every time, and every field sits at a fixed byte
 * offset. computeSchemaLayout() derives that layout once; encoding and
 * decoding values against it is then memcpy-per-field — or a single
 * memcpy when `memcpyImage` holds (the wire image is bit-identical to a
 * `#pragma pack(1)` C struct on a little-endian machine, which is the
 * reduced-implementation target for MCU peers).
 *
 * Fixed-layout kinds (transitively):
 *   - Scalar/Enum with a non-varint scalarType (FixedString = fixedCount)
 *   - FixedArray<fixed>
 *   - Optional<fixed>  (wire: u8 tag + element slot, zero-filled absent)
 *   - Packed struct with all fields fixed
 *
 * Variable ("bailout") kinds: varint scalars, String, Bytes,
 * DynamicArray, Map, OneOf, Tagged structs.
 *
 * A packed struct may still carry a variable tail: graph validation
 * requires all fixed-layout fields to be declared before all
 * variable-layout fields (see validatePackedFieldOrdering in
 * Schema.hpp), so `head` always covers a fixed-offset prefix and `tail`
 * decodes sequentially from `wireSize`... i.e. from `headBytes`.
 *
 * DynamicArray elements are length-prefixed but individually
 * fixed-layout when the element schema is static — each element
 * payload is bailout-free; staticElementStride() gives the uniform
 * stride for slicing.
 *
 * Threading: SchemaLayout is read-only after construction; the
 * encode/read helpers are reentrant. Node pointers must outlive the
 * layout (same lifetime rule as SchemaValue::node).
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "tether/io/Schema.hpp"

namespace tether::io {

/**One fixed-offset field inside a packed-struct row. */
struct LayoutField {
    const SchemaField* field = nullptr; /**< name, key, schema ref */
    size_t offset = 0; /**< byte offset in the row */
    size_t size = 0; /**< wire bytes at that offset */
};

/**Precomputed wire layout for one schema node (or subtree). */
struct SchemaLayout {
    const SchemaNode* node = nullptr;
    /**Whole subtree is fixed-length: wire image size is constant. */
    bool fixedLayout = false;
    /**
     * fixedLayout && no varint-typed scalars inside: the wire image is
     * bit-identical to a packed C struct (LE), so a reduced peer can
     * encode/decode by memcpy of a native struct.
     */
    bool memcpyImage = false;
    /**
     * Total wire bytes when fixedLayout; the fixed head's size for a
     * mixed packed struct (dynamic tail follows at this offset).
     */
    size_t wireSize = 0;
    /**Packed structs: fixed-offset fields in declared order. */
    std::vector<LayoutField> head;
    /**
     * Packed structs: variable-layout fields in declared order
     * (each decodes sequentially after the head).
     */
    std::vector<const SchemaField*> tail;

    /**
     * Fixed byte offset of `key` inside the row, if it is a head field.
     * @param key SchemaField::key to look up.
     * @return offset in bytes, or nullopt for tail/absent fields.
     */
    std::optional<size_t> fieldOffset(uint32_t key) const {
        for (const auto& f : head)
            if (f.field->key == key) return f.offset;
        return std::nullopt;
    }
    /**
     * Wire size of `key` as a head field.
     * @param key SchemaField::key to look up.
     * @return wire bytes at that offset, or nullopt for tail/absent fields.
     */
    std::optional<size_t> fieldSize(uint32_t key) const {
        for (const auto& f : head)
            if (f.field->key == key) return f.size;
        return std::nullopt;
    }
};

namespace schema_layout_detail {

struct Probe {
    bool fixed = false;
    bool memcpyImage = false;
    size_t size = 0;
};

inline bool isVarintScalar(ValueType t) {
    return t == ValueType::UVarint || t == ValueType::IVarint ||
           t == ValueType::Enum;
}

/**Recursive fixed-size/memcpy-image probe; memoized over `seen`. */
inline Probe probe(const SchemaGraph& g, const SchemaNode& node,
                   std::unordered_map<const SchemaNode*, Probe>& seen) {
    if (const auto it = seen.find(&node); it != seen.end()) return it->second;
    Probe p;
    switch (node.kind) {
        case SchemaKind::Scalar:
        case SchemaKind::Enum:
            if (isVarintScalar(node.scalarType)) break;
            if (node.scalarType == ValueType::FixedString) {
                if (node.fixedCount > 0) {
                    p = {true, true, node.fixedCount};
                }
            } else if (const auto sz = valueTypeSize(node.scalarType)) {
                p = {true, true, sz};
            }
            break;
        case SchemaKind::FixedArray:
            if (node.element) {
                if (const auto* el = g.find(node.element->key)) {
                    const auto ep = probe(g, *el, seen);
                    if (ep.fixed && ep.size > 0 &&
                        node.fixedCount <= SIZE_MAX / ep.size)
                        p = {true, ep.memcpyImage, ep.size * node.fixedCount};
                }
            }
            break;
        case SchemaKind::Optional:
            if (node.element) {
                if (const auto* el = g.find(node.element->key)) {
                    const auto ep = probe(g, *el, seen);
                    if (ep.fixed && ep.size < SIZE_MAX)
                        p = {true, ep.memcpyImage, 1 + ep.size};
                }
            }
            break;
        case SchemaKind::Struct:
            if (node.structEncoding == StructEncoding::Packed) {
                size_t total = 0;
                bool allFixed = true, allMemcpy = true;
                for (const auto& f : node.fields) {
                    const auto* fn = g.find(f.schema.key);
                    const auto fp =
                        fn ? probe(g, *fn, seen) : Probe{};
                    allFixed = allFixed && fp.fixed;
                    allMemcpy = allMemcpy && fp.memcpyImage;
                    if (fp.fixed) total += fp.size;
                }
                if (allFixed) p = {true, allMemcpy, total};
                else p = {false, false, total};  // fixed head prefix
            }
            break;
        case SchemaKind::Alias:
            if (node.target) {
                if (const auto* tn = g.find(node.target->key))
                    p = probe(g, *tn, seen);
            }
            break;
        default:
            break;  // String/Bytes/DynamicArray/Map/OneOf/Tagged: dynamic
    }
    seen.emplace(&node, p);
    return p;
}

}  // namespace schema_layout_detail

/**
 * Computes the static layout for `node` inside `graph`.
 * Fails when the graph does not validate (unresolvable refs surface as
 * dynamic fields, so a failed ordering rule produces an empty head).
 * @param graph schema graph the node belongs to.
 * @param node node to lay out (aliases are chased first).
 * @return the computed layout, or an error for dangling refs.
 */
inline std::expected<SchemaLayout, std::string>
computeSchemaLayout(const SchemaGraph& graph, const SchemaNode& node) {
    const SchemaNode* resolved = &node;
    while (resolved->kind == SchemaKind::Alias) {
        if (!resolved->target)
            return std::unexpected(std::string("alias without target"));
        resolved = graph.find(resolved->target->key);
        if (!resolved)
            return std::unexpected(std::string("unresolvable alias"));
    }
    std::unordered_map<const SchemaNode*, schema_layout_detail::Probe> seen;
    const auto p = schema_layout_detail::probe(graph, *resolved, seen);

    SchemaLayout layout;
    layout.node = resolved;
    layout.fixedLayout = p.fixed;
    layout.memcpyImage = p.memcpyImage;
    layout.wireSize = p.size;

    if (resolved->kind == SchemaKind::Struct &&
        resolved->structEncoding == StructEncoding::Packed) {
        size_t off = 0;
        bool dynamicSeen = false;
        for (const auto& f : resolved->fields) {
            const auto* fn = graph.find(f.schema.key);
            if (!fn)
                return std::unexpected(std::string("unresolvable field schema"));
            const auto fp = schema_layout_detail::probe(graph, *fn, seen);
            if (fp.fixed && !dynamicSeen) {
                layout.head.push_back({&f, off, fp.size});
                off += fp.size;
            } else {
                dynamicSeen = true;
                layout.tail.push_back(&f);
            }
        }
        layout.wireSize = off;  // head size; equals total when all fixed
    }
    return layout;
}

/**Source bytes for one head field, in schema field order. */
struct RowField {
    const void* data = nullptr;
    size_t size = 0; /**< must equal the field's wire size */
};

/**
 * Encodes the fixed head of `layout` into `out`: `fields[i]` (schema
 * order) is memcpy'd to head[i].offset; each field's declared wire size
 * is checked — the only per-row cost beyond the copies.
 * @param layout computed layout of the packed struct node.
 * @param fields source bytes per head field, in schema field order.
 * @param out destination row, must be at least layout.wireSize bytes.
 * @return head bytes written (== layout.wireSize).
 */
inline std::expected<size_t, std::string>
encodeStaticRow(const SchemaLayout& layout, std::span<const RowField> fields,
                std::span<uint8_t> out) {
    if (fields.size() != layout.head.size())
        return std::unexpected(std::string("head field count mismatch"));
    if (out.size() < layout.wireSize)
        return std::unexpected(std::string("output too small for static row"));
    for (size_t i = 0; i < fields.size(); ++i) {
        const auto& lf = layout.head[i];
        if (fields[i].size != lf.size)
            return std::unexpected(
                std::string("field '") + lf.field->name + "' size mismatch");
        std::memcpy(out.data() + lf.offset, fields[i].data, lf.size);
    }
    return layout.wireSize;
}

/**
 * Wire slice of head field `key` inside a received `row` — the decode
 * side counterpart (a reduced peer would instead cast the row to its
 * native packed struct).
 * @param layout computed layout of the packed struct node.
 * @param key SchemaField::key to look up.
 * @param row received row bytes.
 * @return the field's wire slice, or nullopt for tail/absent fields or
 *         a truncated row.
 */
inline std::optional<std::span<const uint8_t>>
staticFieldBytes(const SchemaLayout& layout, uint32_t key,
                 std::span<const uint8_t> row) {
    for (const auto& f : layout.head) {
        if (f.field->key != key) continue;
        if (row.size() < f.offset + f.size) return std::nullopt;
        return row.subspan(f.offset, f.size);
    }
    return std::nullopt;
}

/**
 * Byte length of a base-128 varint.
 * @param v value to encode.
 * @return encoded length in bytes (1–10).
 */
inline constexpr size_t varintLength(uint64_t v) {
    size_t n = 1;
    while (v >= 0x80) { v >>= 7; ++n; }
    return n;
}
/**
 * Uniform wire stride of a DynamicArray<static> element payload:
 * varint(elemWireSize) + elemWireSize (the child length prefix is
 * constant for fixed-layout elements).
 * @param elemWireSize fixed wire size of the element schema.
 * @return bytes per element including its length prefix.
 */
inline size_t staticElementStride(size_t elemWireSize) {
    return varintLength(elemWireSize) + elemWireSize;
}

}  // namespace tether::io
