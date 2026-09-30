#pragma once

#include "tether/io/Schema.hpp"
#include "tether/io/SchemaValueWire.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace tether::io {

struct SchemaValueLimits {
    uint64_t maxBytes = 1U << 20;
    uint64_t maxElements = 1'000'000;
    uint32_t maxDepth = 32;
};

namespace schema_value_codec_detail {

inline bool fail(BufReader& reader) {
    reader.error = true;
    return false;
}

inline const SchemaNode* findNode(const SchemaGraph& graph, const SchemaRef& ref) {
    return graph.find(ref.key);
}

inline bool readLength(BufReader& reader, uint64_t& length,
                       const SchemaValueLimits& limits) {
    return schema_value_wire_detail::getU64Varint(reader, length) &&
           length <= limits.maxBytes && length <= reader.remaining();
}

inline bool validateNode(const SchemaGraph& graph, const SchemaNode& node,
                         BufReader& reader, const SchemaValueLimits& limits,
                         uint32_t depth);

inline bool validateRef(const SchemaGraph& graph, const SchemaRef& ref,
                        BufReader& reader, const SchemaValueLimits& limits,
                        uint32_t depth) {
    const auto* node = findNode(graph, ref);
    return node != nullptr && validateNode(graph, *node, reader, limits, depth + 1);
}

inline bool validateScalar(ValueType type, BufReader& reader) {
    const auto size = valueTypeSize(type);
    if (size != 0) return reader.getBytes(size) != nullptr && reader.ok();
    if (type == ValueType::UVarint || type == ValueType::IVarint || type == ValueType::Enum) {
        uint64_t value = 0;
        return schema_value_wire_detail::getU64Varint(reader, value);
    }
    return false;
}

inline bool validateStruct(const SchemaGraph& graph, const SchemaNode& node,
                           BufReader& reader, const SchemaValueLimits& limits,
                           uint32_t depth) {
    if (node.structEncoding == StructEncoding::Packed) {
        for (const auto& field : node.fields) {
            if (field.presence != FieldPresence::Required) return fail(reader);
            if (!validateRef(graph, field.schema, reader, limits, depth)) return false;
        }
        return reader.ok();
    }

    uint64_t count = 0;
    if (!schema_value_wire_detail::getU64Varint(reader, count) ||
        count > limits.maxElements) return fail(reader);
    uint32_t previousKey = 0;
    std::vector<bool> present(node.fields.size(), false);
    for (uint64_t index = 0; index < count; ++index) {
        uint64_t key = 0;
        uint64_t length = 0;
        if (!schema_value_wire_detail::getU64Varint(reader, key) || key == 0 ||
            key > UINT32_MAX || key <= previousKey || !readLength(reader, length, limits)) {
            return fail(reader);
        }
        previousKey = static_cast<uint32_t>(key);
        const auto fieldIt = std::lower_bound(
            node.fields.begin(), node.fields.end(), previousKey,
            [](const SchemaField& field, uint32_t fieldKey) { return field.key < fieldKey; });
        if (fieldIt == node.fields.end() || fieldIt->key != previousKey) return fail(reader);
        const size_t fieldIndex = static_cast<size_t>(fieldIt - node.fields.begin());
        if (present[fieldIndex]) return fail(reader);
        const auto* payload = reader.getBytes(static_cast<size_t>(length));
        if (!reader.ok()) return false;
        BufReader fieldReader(payload, static_cast<size_t>(length));
        if (!validateRef(graph, fieldIt->schema, fieldReader, limits, depth) ||
            fieldReader.remaining() != 0) return fail(reader);
        present[fieldIndex] = true;
    }
    for (size_t index = 0; index < node.fields.size(); ++index) {
        if (node.fields[index].presence == FieldPresence::Required && !present[index]) {
            return fail(reader);
        }
    }
    return reader.ok();
}

inline bool validateNode(const SchemaGraph& graph, const SchemaNode& node,
                         BufReader& reader, const SchemaValueLimits& limits,
                         uint32_t depth) {
    if (depth > limits.maxDepth) return fail(reader);
    switch (node.kind) {
        case SchemaKind::Scalar:
        case SchemaKind::Enum:
            return validateScalar(node.scalarType, reader) || fail(reader);
        case SchemaKind::String: {
            std::string value;
            return decodeSchemaString(reader, value, limits.maxBytes);
        }
        case SchemaKind::Bytes: {
            std::vector<uint8_t> value;
            return decodeSchemaBytes(reader, value, limits.maxBytes);
        }
        case SchemaKind::Struct:
            return validateStruct(graph, node, reader, limits, depth);
        case SchemaKind::FixedArray:
            if (!node.element) return fail(reader);
            for (uint32_t index = 0; index < node.fixedCount; ++index) {
                if (!validateRef(graph, *node.element, reader, limits, depth)) return false;
            }
            return true;
        case SchemaKind::DynamicArray: {
            if (!node.element) return fail(reader);
            uint64_t count = 0;
            if (!schema_value_wire_detail::getU64Varint(reader, count) ||
                count < node.minCount || count > node.maxCount || count > limits.maxElements) {
                return fail(reader);
            }
            for (uint64_t index = 0; index < count; ++index) {
                uint64_t length = 0;
                if (!readLength(reader, length, limits)) return fail(reader);
                const auto* payload = reader.getBytes(static_cast<size_t>(length));
                if (!reader.ok()) return false;
                BufReader elementReader(payload, static_cast<size_t>(length));
                if (!validateRef(graph, *node.element, elementReader, limits, depth) ||
                    elementReader.remaining() != 0) return fail(reader);
            }
            return true;
        }
        case SchemaKind::Map: {
            if (!node.mapKey || !node.mapValue) return fail(reader);
            uint64_t count = 0;
            if (!schema_value_wire_detail::getU64Varint(reader, count) ||
                count < node.minCount || count > node.maxCount || count > limits.maxElements) {
                return fail(reader);
            }
            std::vector<uint8_t> previousKey;
            for (uint64_t index = 0; index < count; ++index) {
                uint64_t keyLength = 0;
                if (!readLength(reader, keyLength, limits)) return fail(reader);
                const auto* keyPayload = reader.getBytes(static_cast<size_t>(keyLength));
                if (!reader.ok()) return false;
                std::vector<uint8_t> keyBytes(keyPayload, keyPayload + keyLength);
                if (!previousKey.empty() && !(previousKey < keyBytes)) return fail(reader);
                BufReader keyReader(keyBytes.data(), keyBytes.size());
                if (!validateRef(graph, *node.mapKey, keyReader, limits, depth) ||
                    keyReader.remaining() != 0) return fail(reader);
                previousKey = std::move(keyBytes);
                uint64_t valueLength = 0;
                if (!readLength(reader, valueLength, limits)) return fail(reader);
                const auto* valuePayload = reader.getBytes(static_cast<size_t>(valueLength));
                if (!reader.ok()) return false;
                BufReader valueReader(valuePayload, static_cast<size_t>(valueLength));
                if (!validateRef(graph, *node.mapValue, valueReader, limits, depth) ||
                    valueReader.remaining() != 0) return fail(reader);
            }
            return true;
        }
        case SchemaKind::Optional: {
            if (!node.element) return fail(reader);
            const uint8_t present = reader.getU8();
            if (!reader.ok() || present > 1) return fail(reader);
            return present == 0 || validateRef(graph, *node.element, reader, limits, depth);
        }
        case SchemaKind::OneOf: {
            uint64_t member = 0;
            uint64_t length = 0;
            if (!schema_value_wire_detail::getU64Varint(reader, member) || member == 0 ||
                !readLength(reader, length, limits)) return fail(reader);
            const auto it = std::lower_bound(
                node.oneOfMembers.begin(), node.oneOfMembers.end(), member,
                [](const auto& item, uint64_t value) { return item.first < value; });
            if (it == node.oneOfMembers.end() || it->first != member) return fail(reader);
            const auto* payload = reader.getBytes(static_cast<size_t>(length));
            if (!reader.ok()) return false;
            BufReader memberReader(payload, static_cast<size_t>(length));
            if (!validateRef(graph, it->second, memberReader, limits, depth) ||
                memberReader.remaining() != 0) return fail(reader);
            return true;
        }
        case SchemaKind::Alias:
            return node.target && validateRef(graph, *node.target, reader, limits, depth);
    }
    return fail(reader);
}

} // namespace schema_value_codec_detail

inline bool validateSchemaValue(const SchemaGraph& graph, const SchemaKey& key,
                                BufReader& reader,
                                const SchemaValueLimits& limits = {}) {
    if (!validateSchemaGraph(graph)) return false;
    const auto* node = graph.find(key);
    return node != nullptr && schema_value_codec_detail::validateNode(
        graph, *node, reader, limits, 0);
}

} // namespace tether::io
