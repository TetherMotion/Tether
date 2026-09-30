#pragma once

#include "tether/io/Schema.hpp"
#include "tether/io/SchemaValueWire.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
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

inline const SchemaNode* resolveAlias(const SchemaGraph& graph, const SchemaRef& ref) {
    const SchemaNode* node = graph.find(ref.key);
    while (node && node->kind == SchemaKind::Alias) {
        if (!node->target) return nullptr;
        node = graph.find(node->target->key);
    }
    return node;
}

template <typename T>
inline T readFixedMapKey(BufReader& reader) {
    if constexpr (sizeof(T) == 1) return static_cast<T>(reader.getU8());
    if constexpr (sizeof(T) == 2) return static_cast<T>(reader.getU16());
    if constexpr (sizeof(T) == 4) return static_cast<T>(reader.getU32());
    return static_cast<T>(reader.getU64());
}

template <typename T>
inline bool compareFixedMapKey(BufReader& left, BufReader& right, int& result) {
    const T leftValue = readFixedMapKey<T>(left);
    const T rightValue = readFixedMapKey<T>(right);
    if (!left.ok() || !right.ok() || left.remaining() != 0 || right.remaining() != 0) return false;
    result = leftValue < rightValue ? -1 : leftValue > rightValue ? 1 : 0;
    return true;
}

inline bool compareByteMapKey(BufReader& left, BufReader& right, size_t size, int& result) {
    const auto* leftBytes = left.getBytes(size);
    const auto* rightBytes = right.getBytes(size);
    if (!left.ok() || !right.ok() || left.remaining() != 0 || right.remaining() != 0) return false;
    result = std::memcmp(leftBytes, rightBytes, size);
    return true;
}

inline bool compareMapKeys(const SchemaGraph& graph, const SchemaRef& schema,
                           const std::vector<uint8_t>& leftBytes,
                           const std::vector<uint8_t>& rightBytes, int& result) {
    const SchemaNode* node = resolveAlias(graph, schema);
    if (!node) return false;
    BufReader left(leftBytes.data(), leftBytes.size());
    BufReader right(rightBytes.data(), rightBytes.size());
    if (node->kind == SchemaKind::String) {
        std::string leftValue;
        std::string rightValue;
        if (!decodeSchemaString(left, leftValue, leftBytes.size()) ||
            !decodeSchemaString(right, rightValue, rightBytes.size()) ||
            left.remaining() != 0 || right.remaining() != 0) return false;
        result = leftValue < rightValue ? -1 : leftValue > rightValue ? 1 : 0;
        return true;
    }
    if (node->kind == SchemaKind::Bytes) {
        std::vector<uint8_t> leftValue;
        std::vector<uint8_t> rightValue;
        if (!decodeSchemaBytes(left, leftValue, leftBytes.size()) ||
            !decodeSchemaBytes(right, rightValue, rightBytes.size()) ||
            left.remaining() != 0 || right.remaining() != 0) return false;
        result = std::lexicographical_compare(leftValue.begin(), leftValue.end(),
                                               rightValue.begin(), rightValue.end()) ? -1 :
                 std::lexicographical_compare(rightValue.begin(), rightValue.end(),
                                               leftValue.begin(), leftValue.end()) ? 1 : 0;
        return true;
    }
    if (node->kind != SchemaKind::Scalar && node->kind != SchemaKind::Enum) return false;
    switch (node->scalarType) {
        case ValueType::U8: case ValueType::Bool: return compareFixedMapKey<uint8_t>(left, right, result);
        case ValueType::U16: return compareFixedMapKey<uint16_t>(left, right, result);
        case ValueType::U32: return compareFixedMapKey<uint32_t>(left, right, result);
        case ValueType::U64: return compareFixedMapKey<uint64_t>(left, right, result);
        case ValueType::I8: return compareFixedMapKey<int8_t>(left, right, result);
        case ValueType::I16: return compareFixedMapKey<int16_t>(left, right, result);
        case ValueType::I32: return compareFixedMapKey<int32_t>(left, right, result);
        case ValueType::I64: return compareFixedMapKey<int64_t>(left, right, result);
        case ValueType::UVarint: case ValueType::IVarint: case ValueType::Enum: {
            uint64_t leftValue = 0;
            uint64_t rightValue = 0;
            if (!schema_value_wire_detail::getU64Varint(left, leftValue) ||
                !schema_value_wire_detail::getU64Varint(right, rightValue) ||
                left.remaining() != 0 || right.remaining() != 0) return false;
            result = leftValue < rightValue ? -1 : leftValue > rightValue ? 1 : 0;
            return true;
        }
        case ValueType::IPv4: return compareByteMapKey(left, right, 4, result);
        case ValueType::IPv6: return compareByteMapKey(left, right, 16, result);
        case ValueType::MAC: return compareByteMapKey(left, right, 6, result);
        default: return false;
    }
}

struct NumericValue {
    ValueType type = ValueType::Binary;
    uint64_t bits = 0;
};

inline bool readNumericValue(const SchemaNode& node, BufReader& reader,
                             NumericValue& value) {
    if (node.kind != SchemaKind::Scalar && node.kind != SchemaKind::Enum) return false;
    value.type = node.scalarType;
    switch (node.scalarType) {
        case ValueType::U8: case ValueType::Bool:
            value.bits = reader.getU8(); break;
        case ValueType::U16:
            value.bits = reader.getU16(); break;
        case ValueType::U32:
            value.bits = reader.getU32(); break;
        case ValueType::U64:
            value.bits = reader.getU64(); break;
        case ValueType::I8:
            value.bits = static_cast<uint64_t>(static_cast<int64_t>(static_cast<int8_t>(reader.getU8()))); break;
        case ValueType::I16:
            value.bits = static_cast<uint64_t>(static_cast<int64_t>(static_cast<int16_t>(reader.getU16()))); break;
        case ValueType::I32:
            value.bits = static_cast<uint64_t>(static_cast<int64_t>(reader.getI32())); break;
        case ValueType::I64:
            value.bits = static_cast<uint64_t>(static_cast<int64_t>(reader.getU64())); break;
        case ValueType::F32:
            value.bits = reader.getU32(); break;
        case ValueType::F64:
            value.bits = reader.getU64(); break;
        case ValueType::UVarint: case ValueType::IVarint: case ValueType::Enum:
            if (!schema_value_wire_detail::getU64Varint(reader, value.bits)) return false;
            break;
        default:
            return false;
    }
    return reader.ok();
}

inline int compareNumericValues(const NumericValue& left, const NumericValue& right) {
    if (left.type != right.type) return 2;
    switch (left.type) {
        case ValueType::I8: case ValueType::I16: case ValueType::I32: case ValueType::I64: {
            const auto leftValue = static_cast<int64_t>(left.bits);
            const auto rightValue = static_cast<int64_t>(right.bits);
            return leftValue < rightValue ? -1 : leftValue > rightValue ? 1 : 0;
        }
        case ValueType::F32: {
            const auto decode = [](uint64_t bits) {
                uint32_t raw = static_cast<uint32_t>(bits);
                float value;
                std::memcpy(&value, &raw, sizeof(value));
                return value;
            };
            const float leftValue = decode(left.bits);
            const float rightValue = decode(right.bits);
            return leftValue < rightValue ? -1 : leftValue > rightValue ? 1 : 0;
        }
        case ValueType::F64: {
            const auto decode = [](uint64_t bits) {
                double value;
                std::memcpy(&value, &bits, sizeof(value));
                return value;
            };
            const double leftValue = decode(left.bits);
            const double rightValue = decode(right.bits);
            return leftValue < rightValue ? -1 : leftValue > rightValue ? 1 : 0;
        }
        default:
            return left.bits < right.bits ? -1 : left.bits > right.bits ? 1 : 0;
    }
}

inline bool logicalLength(const SchemaGraph& graph, const SchemaRef& schema,
                          const uint8_t* payload, size_t length,
                          uint64_t& result) {
    const SchemaNode* node = resolveAlias(graph, schema);
    if (!node) return false;
    BufReader reader(payload, length);
    switch (node->kind) {
        case SchemaKind::String: {
            std::string value;
            if (!decodeSchemaString(reader, value, length) || reader.remaining() != 0) return false;
            result = value.size();
            return true;
        }
        case SchemaKind::Bytes: {
            std::vector<uint8_t> value;
            if (!decodeSchemaBytes(reader, value, length) || reader.remaining() != 0) return false;
            result = value.size();
            return true;
        }
        case SchemaKind::FixedArray:
            result = node->fixedCount;
            return true;
        case SchemaKind::DynamicArray:
        case SchemaKind::Map:
            if (!schema_value_wire_detail::getU64Varint(reader, result)) return false;
            return true;
        default:
            return false;
    }
}

inline bool validateFieldRestrictions(const SchemaGraph& graph, const SchemaField& field,
                                      const uint8_t* payload, size_t length,
                                      const SchemaValueLimits& limits, uint32_t depth) {
    const SchemaNode* node = resolveAlias(graph, field.schema);
    if (!node) return false;
    for (const auto& restriction : field.restrictions) {
        BufReader restrictionReader(restriction.payload.data(), restriction.payload.size());
        switch (restriction.kind) {
            case RestrictionKind::NumericRange: {
                const uint8_t hasLower = restrictionReader.getU8();
                if (!restrictionReader.ok() || hasLower > 1) return false;
                NumericValue value;
                NumericValue lower;
                NumericValue upper;
                BufReader valueReader(payload, length);
                if (!readNumericValue(*node, valueReader, value) || valueReader.remaining() != 0) return false;
                if (hasLower && !readNumericValue(*node, restrictionReader, lower)) return false;
                const uint8_t hasUpper = restrictionReader.getU8();
                if (!restrictionReader.ok() || hasUpper > 1) return false;
                if (hasUpper && !readNumericValue(*node, restrictionReader, upper)) return false;
                if (restrictionReader.remaining() != 0 ||
                    (hasLower && compareNumericValues(value, lower) < 0) ||
                    (hasUpper && compareNumericValues(value, upper) > 0)) return false;
                break;
            }
            case RestrictionKind::MultipleOf: {
                NumericValue value;
                NumericValue divisor;
                BufReader valueReader(payload, length);
                if (!readNumericValue(*node, valueReader, value) || valueReader.remaining() != 0 ||
                    !readNumericValue(*node, restrictionReader, divisor) ||
                    restrictionReader.remaining() != 0) return false;
                switch (value.type) {
                    case ValueType::I8: case ValueType::I16: case ValueType::I32: case ValueType::I64: {
                        const int64_t divisorValue = static_cast<int64_t>(divisor.bits);
                        const int64_t valueValue = static_cast<int64_t>(value.bits);
                        if (divisorValue == 0 || (divisorValue != -1 && valueValue % divisorValue != 0)) return false;
                        break;
                    }
                    case ValueType::U8: case ValueType::U16: case ValueType::U32: case ValueType::U64:
                    case ValueType::UVarint: case ValueType::IVarint: case ValueType::Enum:
                        if (divisor.bits == 0 || value.bits % divisor.bits != 0) return false;
                        break;
                    default:
                        return false;
                }
                break;
            }
            case RestrictionKind::Finite: {
                if (restrictionReader.remaining() != 0) return false;
                NumericValue value;
                BufReader valueReader(payload, length);
                if (!readNumericValue(*node, valueReader, value) || valueReader.remaining() != 0) return false;
                if (value.type == ValueType::F32) {
                    uint32_t bits = static_cast<uint32_t>(value.bits);
                    float decoded;
                    std::memcpy(&decoded, &bits, sizeof(decoded));
                    if (!std::isfinite(decoded)) return false;
                } else if (value.type == ValueType::F64) {
                    double decoded;
                    std::memcpy(&decoded, &value.bits, sizeof(decoded));
                    if (!std::isfinite(decoded)) return false;
                } else {
                    return false;
                }
                break;
            }
            case RestrictionKind::LengthRange: {
                uint64_t minimum = 0;
                uint64_t maximum = 0;
                uint64_t valueLength = 0;
                if (!schema_value_wire_detail::getU64Varint(restrictionReader, minimum) ||
                    !schema_value_wire_detail::getU64Varint(restrictionReader, maximum) ||
                    restrictionReader.remaining() != 0 || minimum > maximum ||
                    !logicalLength(graph, field.schema, payload, length, valueLength) ||
                    valueLength < minimum || valueLength > maximum) return false;
                break;
            }
            case RestrictionKind::AllowedValues: {
                uint64_t count = 0;
                if (!schema_value_wire_detail::getU64Varint(restrictionReader, count) ||
                    count == 0 || count > limits.maxElements) return false;
                std::vector<uint8_t> previous;
                bool allowed = false;
                for (uint64_t index = 0; index < count; ++index) {
                    uint64_t allowedLength = 0;
                    if (!readLength(restrictionReader, allowedLength, limits)) return false;
                    const auto* allowedPayload = restrictionReader.getBytes(static_cast<size_t>(allowedLength));
                    if (!restrictionReader.ok()) return false;
                    BufReader allowedReader(allowedPayload, static_cast<size_t>(allowedLength));
                    if (!validateRef(graph, field.schema, allowedReader, limits, depth) ||
                        allowedReader.remaining() != 0) return false;
                    std::vector<uint8_t> value(allowedPayload, allowedPayload + allowedLength);
                    if (!previous.empty() && !std::lexicographical_compare(
                            previous.begin(), previous.end(), value.begin(), value.end())) return false;
                    allowed = allowed || value.size() == length &&
                              std::equal(value.begin(), value.end(), payload);
                    previous = std::move(value);
                }
                if (restrictionReader.remaining() != 0 || !allowed) return false;
                break;
            }
            case RestrictionKind::AllowedMembers: {
                if (node->kind != SchemaKind::OneOf) return false;
                BufReader valueReader(payload, length);
                uint64_t member = 0;
                uint64_t ignoredLength = 0;
                if (!schema_value_wire_detail::getU64Varint(valueReader, member) ||
                    !readLength(valueReader, ignoredLength, limits)) return false;
                uint64_t previous = 0;
                bool allowed = false;
                while (restrictionReader.remaining() != 0) {
                    uint64_t candidate = 0;
                    if (!schema_value_wire_detail::getU64Varint(restrictionReader, candidate) ||
                        candidate == 0 || candidate <= previous) return false;
                    previous = candidate;
                    allowed = allowed || candidate == member;
                }
                if (!allowed) return false;
                break;
            }
            case RestrictionKind::UniqueElements: {
                if (node->kind != SchemaKind::DynamicArray || !node->element ||
                    restrictionReader.remaining() != 0) return false;
                BufReader valueReader(payload, length);
                uint64_t count = 0;
                if (!schema_value_wire_detail::getU64Varint(valueReader, count)) return false;
                std::set<std::vector<uint8_t>> values;
                for (uint64_t index = 0; index < count; ++index) {
                    uint64_t elementLength = 0;
                    if (!readLength(valueReader, elementLength, limits)) return false;
                    const auto* element = valueReader.getBytes(static_cast<size_t>(elementLength));
                    if (!valueReader.ok() || !values.emplace(element, element + elementLength).second) return false;
                }
                if (valueReader.remaining() != 0) return false;
                break;
            }
        }
    }
    return true;
}

inline bool validateStruct(const SchemaGraph& graph, const SchemaNode& node,
                           BufReader& reader, const SchemaValueLimits& limits,
                           uint32_t depth) {
    if (node.structEncoding == StructEncoding::Packed) {
        for (const auto& field : node.fields) {
            if (field.presence != FieldPresence::Required) return fail(reader);
            const size_t start = reader.pos;
            if (!validateRef(graph, field.schema, reader, limits, depth) ||
                !validateFieldRestrictions(graph, field, reader.buf + start, reader.pos - start,
                                           limits, depth)) return false;
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
            fieldReader.remaining() != 0 ||
            !validateFieldRestrictions(graph, *fieldIt, payload, static_cast<size_t>(length),
                                       limits, depth)) return fail(reader);
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
                BufReader keyReader(keyBytes.data(), keyBytes.size());
                if (!validateRef(graph, *node.mapKey, keyReader, limits, depth) ||
                    keyReader.remaining() != 0) return fail(reader);
                if (!previousKey.empty()) {
                    int comparison = 0;
                    if (!compareMapKeys(graph, *node.mapKey, previousKey, keyBytes, comparison) ||
                        comparison >= 0) return fail(reader);
                }
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
