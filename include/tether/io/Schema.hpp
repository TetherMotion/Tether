#pragma once

#include "tether/io/Protocol.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace tether::io {

using SchemaKey = std::array<uint8_t, 16>;
using SchemaDigest = std::array<uint8_t, 32>;

struct SchemaKeyHash {
    size_t operator()(const SchemaKey& key) const noexcept {
        size_t value = 0;
        for (uint8_t byte : key) value = (value * 131U) ^ byte;
        return value;
    }
};

struct SchemaRef {
    SchemaKey key{};
    SchemaDigest digest{};

    friend bool operator==(const SchemaRef&, const SchemaRef&) = default;
};

enum class SchemaKind : uint8_t {
    Scalar = 1,
    String = 2,
    Bytes = 3,
    Enum = 4,
    Struct = 5,
    FixedArray = 6,
    DynamicArray = 7,
    Optional = 8,
    Variant = 9,
    Alias = 10,
};

enum class StructEncoding : uint8_t {
    Packed = 1,
    Tagged = 2,
};

namespace SchemaFieldFlags {
inline constexpr uint32_t Required = 1U << 0;
inline constexpr uint32_t ReadOnly = 1U << 1;
inline constexpr uint32_t Deprecated = 1U << 2;
inline constexpr uint32_t Secret = 1U << 3;
}

struct SchemaLimits {
    uint32_t maxGraphNodes = 1024;
    uint32_t maxGraphDepth = 32;
    uint32_t maxFields = 1024;
    uint32_t maxArrayElements = 1'000'000;
    uint32_t maxStringBytes = 1'000'000;
    uint32_t maxFixedSize = 16 * 1024 * 1024;
};

struct SchemaField {
    uint32_t key = 0;
    uint32_t flags = 0;
    SchemaRef schema;
    std::string name;
    std::string description;
};

struct SchemaNode {
    SchemaKey key{};
    uint32_t revision = 1;
    SchemaKind kind = SchemaKind::Scalar;
    uint32_t flags = 0;
    std::string name;
    std::string description;
    std::map<std::string, std::string> annotations;

    ValueType scalarType = ValueType::Binary;
    uint32_t maxBytes = 0;
    uint32_t fixedCount = 0;
    uint32_t minCount = 0;
    uint32_t maxCount = 0;
    StructEncoding structEncoding = StructEncoding::Tagged;
    std::optional<SchemaRef> element;
    std::optional<SchemaRef> target;
    std::vector<SchemaField> fields;
    std::vector<std::pair<uint32_t, SchemaRef>> variants;

    SchemaRef ref() const { return SchemaRef{key, {}}; }
};

struct SchemaValidationResult {
    bool ok = false;
    std::string message;
    std::vector<uint32_t> path;

    explicit operator bool() const { return ok; }
    static SchemaValidationResult success() { return {true, {}, {}}; }
    static SchemaValidationResult failure(std::string reason, std::vector<uint32_t> value = {}) {
        return {false, std::move(reason), std::move(value)};
    }
};

struct SchemaGraph {
    std::vector<SchemaNode> nodes;

    const SchemaNode* find(const SchemaKey& key) const {
        for (const auto& node : nodes) {
            if (node.key == key) return &node;
        }
        return nullptr;
    }
};

namespace detail {
inline void appendU8(std::vector<uint8_t>& out, uint8_t value) { out.push_back(value); }
inline void appendU32(std::vector<uint8_t>& out, uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) out.push_back(static_cast<uint8_t>(value >> shift));
}
inline void appendBytes(std::vector<uint8_t>& out, const uint8_t* data, size_t size) {
    out.insert(out.end(), data, data + size);
}
inline void appendString(std::vector<uint8_t>& out, const std::string& value) {
    appendU32(out, static_cast<uint32_t>(value.size()));
    appendBytes(out, reinterpret_cast<const uint8_t*>(value.data()), value.size());
}
inline void appendKey(std::vector<uint8_t>& out, const SchemaKey& key) {
    appendBytes(out, key.data(), key.size());
}
inline void appendRef(std::vector<uint8_t>& out, const SchemaRef& ref) {
    appendKey(out, ref.key);
    appendBytes(out, ref.digest.data(), ref.digest.size());
}
}

/// Canonical descriptor bytes used as the input to the protocol digest.
inline std::vector<uint8_t> canonicalDescriptor(const SchemaNode& node) {
    std::vector<uint8_t> out;
    detail::appendKey(out, node.key);
    detail::appendU32(out, node.revision);
    detail::appendU8(out, static_cast<uint8_t>(node.kind));
    detail::appendU32(out, node.flags);
    detail::appendString(out, node.name);
    detail::appendString(out, node.description);
    detail::appendU32(out, static_cast<uint32_t>(node.annotations.size()));
    for (const auto& [name, value] : node.annotations) {
        detail::appendString(out, name);
        detail::appendString(out, value);
    }

    detail::appendU8(out, static_cast<uint8_t>(node.scalarType));
    detail::appendU32(out, node.maxBytes);
    detail::appendU32(out, node.fixedCount);
    detail::appendU32(out, node.minCount);
    detail::appendU32(out, node.maxCount);
    detail::appendU8(out, static_cast<uint8_t>(node.structEncoding));
    detail::appendU8(out, node.element.has_value());
    if (node.element) detail::appendRef(out, *node.element);
    detail::appendU8(out, node.target.has_value());
    if (node.target) detail::appendRef(out, *node.target);

    detail::appendU32(out, static_cast<uint32_t>(node.fields.size()));
    for (const auto& field : node.fields) {
        detail::appendU32(out, field.key);
        detail::appendU32(out, field.flags);
        detail::appendRef(out, field.schema);
        detail::appendString(out, field.name);
        detail::appendString(out, field.description);
    }

    detail::appendU32(out, static_cast<uint32_t>(node.variants.size()));
    for (const auto& [key, ref] : node.variants) {
        detail::appendU32(out, key);
        detail::appendRef(out, ref);
    }
    return out;
}

namespace detail {
inline bool isFixedScalar(ValueType type) {
    return valueTypeSize(type) != 0;
}

inline SchemaValidationResult validateNodeShape(const SchemaNode& node, const SchemaLimits& limits) {
    if (node.key == SchemaKey{}) return SchemaValidationResult::failure("schema key must not be zero");
    if (node.kind == SchemaKind::Struct && node.fields.size() > limits.maxFields)
        return SchemaValidationResult::failure("too many struct fields");
    if ((node.kind == SchemaKind::String || node.kind == SchemaKind::Bytes) && node.maxBytes > limits.maxStringBytes)
        return SchemaValidationResult::failure("variable value exceeds byte limit");
    if (node.kind == SchemaKind::FixedArray && node.fixedCount > limits.maxArrayElements)
        return SchemaValidationResult::failure("fixed array exceeds element limit");
    if (node.kind == SchemaKind::DynamicArray &&
        (node.minCount > node.maxCount || node.maxCount > limits.maxArrayElements))
        return SchemaValidationResult::failure("invalid dynamic array bounds");
    if (node.kind == SchemaKind::Struct) {
        uint32_t previous = 0;
        for (const auto& field : node.fields) {
            if (field.key == 0 || field.key <= previous)
                return SchemaValidationResult::failure("struct fields must have increasing non-zero keys");
            previous = field.key;
            if ((field.flags & SchemaFieldFlags::Required) && field.schema.key == SchemaKey{})
                return SchemaValidationResult::failure("required field has no schema");
        }
        if (node.structEncoding == StructEncoding::Packed) {
            for (const auto& field : node.fields) {
                if (!(field.flags & SchemaFieldFlags::Required))
                    return SchemaValidationResult::failure("packed struct fields must be required");
            }
        }
    }
    uint32_t previousVariant = 0;
    for (const auto& [key, ref] : node.variants) {
        if (key == 0 || key <= previousVariant || ref.key == SchemaKey{})
            return SchemaValidationResult::failure("variant arms must have increasing non-zero keys");
        previousVariant = key;
    }
    return SchemaValidationResult::success();
}

inline SchemaValidationResult validateReferences(const SchemaGraph& graph, const SchemaNode& node) {
    const auto require = [&graph](const SchemaRef& ref) {
        return graph.find(ref.key) != nullptr;
    };
    if (node.element && !require(*node.element)) return SchemaValidationResult::failure("missing element schema");
    if (node.target && !require(*node.target)) return SchemaValidationResult::failure("missing target schema");
    for (const auto& field : node.fields) {
        if (!require(field.schema)) return SchemaValidationResult::failure("missing field schema", {field.key});
    }
    for (const auto& [key, ref] : node.variants) {
        if (!require(ref)) return SchemaValidationResult::failure("missing variant schema", {key});
    }
    return SchemaValidationResult::success();
}
}

inline SchemaValidationResult validateSchemaGraph(const SchemaGraph& graph,
                                                   const SchemaLimits& limits = {}) {
    if (graph.nodes.empty()) return SchemaValidationResult::failure("schema graph is empty");
    if (graph.nodes.size() > limits.maxGraphNodes) return SchemaValidationResult::failure("too many schema nodes");

    std::unordered_map<SchemaKey, const SchemaNode*, SchemaKeyHash> nodes;
    for (const auto& node : graph.nodes) {
        if (!detail::validateNodeShape(node, limits)) return detail::validateNodeShape(node, limits);
        if (!nodes.emplace(node.key, &node).second)
            return SchemaValidationResult::failure("duplicate schema key");
    }
    for (const auto& node : graph.nodes) {
        auto result = detail::validateReferences(graph, node);
        if (!result) return result;
    }

    std::unordered_map<SchemaKey, uint8_t, SchemaKeyHash> marks;
    std::function<SchemaValidationResult(const SchemaNode&, uint32_t)> visit =
        [&](const SchemaNode& node, uint32_t depth) -> SchemaValidationResult {
        if (depth > limits.maxGraphDepth) return SchemaValidationResult::failure("schema graph is too deep");
        auto& mark = marks[node.key];
        if (mark == 1) return SchemaValidationResult::failure("schema graph contains a cycle");
        if (mark == 2) return SchemaValidationResult::success();
        mark = 1;
        const auto visitRef = [&](const SchemaRef& ref) {
            return visit(*graph.find(ref.key), depth + 1);
        };
        if (node.element) {
            auto result = visitRef(*node.element);
            if (!result) return result;
        }
        if (node.target) {
            auto result = visitRef(*node.target);
            if (!result) return result;
        }
        for (const auto& field : node.fields) {
            auto result = visitRef(field.schema);
            if (!result) return result;
        }
        for (const auto& [key, ref] : node.variants) {
            (void)key;
            auto result = visitRef(ref);
            if (!result) return result;
        }
        mark = 2;
        return SchemaValidationResult::success();
    };
    for (const auto& node : graph.nodes) {
        auto result = visit(node, 1);
        if (!result) return result;
    }
    return SchemaValidationResult::success();
}

inline std::optional<size_t> fixedSchemaSize(const SchemaGraph& graph,
                                             const SchemaKey& root,
                                             const SchemaLimits& limits = {}) {
    if (!validateSchemaGraph(graph, limits)) return std::nullopt;
    std::unordered_map<SchemaKey, std::optional<size_t>, SchemaKeyHash> sizes;
    std::function<std::optional<size_t>(const SchemaKey&)> sizeOf = [&](const SchemaKey& key) {
        if (const auto cached = sizes.find(key); cached != sizes.end()) return cached->second;
        const auto* node = graph.find(key);
        if (!node) return std::optional<size_t>{};
        std::optional<size_t> result;
        switch (node->kind) {
            case SchemaKind::Scalar:
                if (detail::isFixedScalar(node->scalarType)) result = valueTypeSize(node->scalarType);
                break;
            case SchemaKind::Enum:
                if (detail::isFixedScalar(node->scalarType)) result = valueTypeSize(node->scalarType);
                break;
            case SchemaKind::Alias:
                result = node->target ? sizeOf(node->target->key) : std::nullopt;
                break;
            case SchemaKind::FixedArray:
                if (node->element) {
                    const auto elementSize = sizeOf(node->element->key);
                    if (elementSize && node->fixedCount <= limits.maxFixedSize / *elementSize)
                        result = *elementSize * node->fixedCount;
                }
                break;
            case SchemaKind::Struct:
                if (node->structEncoding == StructEncoding::Packed) {
                    size_t total = 0;
                    result = total;
                    for (const auto& field : node->fields) {
                        if (!(field.flags & SchemaFieldFlags::Required)) {
                            result = std::nullopt;
                            break;
                        }
                        const auto fieldSize = sizeOf(field.schema.key);
                        if (!fieldSize || total > limits.maxFixedSize - *fieldSize) {
                            result = std::nullopt;
                            break;
                        }
                        total += *fieldSize;
                        result = total;
                    }
                }
                break;
            default:
                break;
        }
        sizes.emplace(key, result);
        return result;
    };
    return sizeOf(root);
}

} // namespace tether::io
