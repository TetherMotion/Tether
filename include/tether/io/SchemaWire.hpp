#pragma once

#include "tether/io/Schema.hpp"

#include <algorithm>
#include <limits>

namespace tether::io {
namespace schema_wire_detail {

inline void putString(BufWriter& writer, const std::string& value) {
    if (value.size() > std::numeric_limits<uint32_t>::max()) {
        writer.overflow = true;
        return;
    }
    writer.putU32(static_cast<uint32_t>(value.size()));
    writer.putBytes(value.data(), value.size());
}

inline bool getString(BufReader& reader, std::string& value, uint32_t maxBytes) {
    const uint32_t size = reader.getU32();
    if (!reader.ok() || size > maxBytes || size > reader.remaining()) return false;
    const auto* bytes = reader.getBytes(size);
    if (!reader.ok()) return false;
    value.assign(reinterpret_cast<const char*>(bytes), size);
    return true;
}

inline void putKey(BufWriter& writer, const SchemaKey& key) { writer.putBytes(key.data(), key.size()); }
inline bool getKey(BufReader& reader, SchemaKey& key) {
    const auto* bytes = reader.getBytes(key.size());
    if (!reader.ok()) return false;
    std::copy(bytes, bytes + key.size(), key.begin());
    return true;
}
inline void putDigest(BufWriter& writer, const SchemaDigest& digest) { writer.putBytes(digest.data(), digest.size()); }
inline bool getDigest(BufReader& reader, SchemaDigest& digest) {
    const auto* bytes = reader.getBytes(digest.size());
    if (!reader.ok()) return false;
    std::copy(bytes, bytes + digest.size(), digest.begin());
    return true;
}
inline void putRef(BufWriter& writer, const SchemaRef& ref) {
    putKey(writer, ref.key);
    putDigest(writer, ref.digest);
}
inline bool getRef(BufReader& reader, SchemaRef& ref) {
    return getKey(reader, ref.key) && getDigest(reader, ref.digest);
}

} // namespace schema_wire_detail

inline void encodeSchemaDefinition(BufWriter& writer, const SchemaNode& node) {
    using namespace schema_wire_detail;
    putKey(writer, node.key);
    writer.putU32(node.revision);
    writer.putU8(static_cast<uint8_t>(node.kind));
    writer.putU32(node.flags);
    putString(writer, node.name);
    putString(writer, node.description);
    writer.putU32(static_cast<uint32_t>(node.annotations.size()));
    for (const auto& [key, value] : node.annotations) {
        putString(writer, key);
        putString(writer, value);
    }
    writer.putU8(static_cast<uint8_t>(node.scalarType));
    writer.putU32(node.maxBytes);
    writer.putU32(node.fixedCount);
    writer.putU32(node.minCount);
    writer.putU32(node.maxCount);
    writer.putU8(static_cast<uint8_t>(node.structEncoding));
    writer.putU8(node.element.has_value() ? 1 : 0);
    if (node.element) putRef(writer, *node.element);
    writer.putU8(node.mapKey.has_value() ? 1 : 0);
    if (node.mapKey) putRef(writer, *node.mapKey);
    writer.putU8(node.mapValue.has_value() ? 1 : 0);
    if (node.mapValue) putRef(writer, *node.mapValue);
    writer.putU8(node.target.has_value() ? 1 : 0);
    if (node.target) putRef(writer, *node.target);
    writer.putU32(static_cast<uint32_t>(node.fields.size()));
    for (const auto& field : node.fields) {
        writer.putU32(field.key);
        writer.putU32(field.flags);
        writer.putU8(static_cast<uint8_t>(field.presence));
        putRef(writer, field.schema);
        putString(writer, field.name);
        putString(writer, field.description);
        writer.putU32(static_cast<uint32_t>(field.restrictions.size()));
        for (const auto& restriction : field.restrictions) {
            writer.putU8(static_cast<uint8_t>(restriction.kind));
            writer.putU32(static_cast<uint32_t>(restriction.payload.size()));
            writer.putBytes(restriction.payload.data(), restriction.payload.size());
        }
        writer.putU32(static_cast<uint32_t>(field.defaultValue.size()));
        writer.putBytes(field.defaultValue.data(), field.defaultValue.size());
    }
    writer.putU32(static_cast<uint32_t>(node.oneOfMembers.size()));
    for (const auto& [key, ref] : node.oneOfMembers) {
        writer.putU32(key);
        putRef(writer, ref);
    }
}

inline bool decodeSchemaDefinition(BufReader& reader, SchemaNode& node,
                                   const SchemaLimits& limits = {}) {
    using namespace schema_wire_detail;
    SchemaNode decoded;
    if (!getKey(reader, decoded.key)) return false;
    decoded.revision = reader.getU32();
    decoded.kind = static_cast<SchemaKind>(reader.getU8());
    decoded.flags = reader.getU32();
    if (!getString(reader, decoded.name, limits.maxStringBytes) ||
        !getString(reader, decoded.description, limits.maxStringBytes)) return false;
    const uint32_t annotationCount = reader.getU32();
    if (!reader.ok() || annotationCount > limits.maxFields) return false;
    for (uint32_t index = 0; index < annotationCount; ++index) {
        std::string key;
        std::string value;
        if (!getString(reader, key, limits.maxStringBytes) ||
            !getString(reader, value, limits.maxStringBytes)) return false;
        if (!decoded.annotations.emplace(std::move(key), std::move(value)).second) return false;
    }
    decoded.scalarType = static_cast<ValueType>(reader.getU8());
    decoded.maxBytes = reader.getU32();
    decoded.fixedCount = reader.getU32();
    decoded.minCount = reader.getU32();
    decoded.maxCount = reader.getU32();
    decoded.structEncoding = static_cast<StructEncoding>(reader.getU8());
    const uint8_t hasElement = reader.getU8();
    if (hasElement > 1) return false;
    if (hasElement) {
        SchemaRef value;
        if (!getRef(reader, value)) return false;
        decoded.element = value;
    }
    const uint8_t hasMapKey = reader.getU8();
    if (hasMapKey > 1) return false;
    if (hasMapKey) {
        SchemaRef value;
        if (!getRef(reader, value)) return false;
        decoded.mapKey = value;
    }
    const uint8_t hasMapValue = reader.getU8();
    if (hasMapValue > 1) return false;
    if (hasMapValue) {
        SchemaRef value;
        if (!getRef(reader, value)) return false;
        decoded.mapValue = value;
    }
    const uint8_t hasTarget = reader.getU8();
    if (hasTarget > 1) return false;
    if (hasTarget) {
        SchemaRef value;
        if (!getRef(reader, value)) return false;
        decoded.target = value;
    }
    const uint32_t fieldCount = reader.getU32();
    if (!reader.ok() || fieldCount > limits.maxFields) return false;
    decoded.fields.reserve(fieldCount);
    for (uint32_t index = 0; index < fieldCount; ++index) {
        SchemaField field;
        field.key = reader.getU32();
        field.flags = reader.getU32();
        field.presence = static_cast<FieldPresence>(reader.getU8());
        if (!getRef(reader, field.schema) ||
            !getString(reader, field.name, limits.maxStringBytes) ||
            !getString(reader, field.description, limits.maxStringBytes)) return false;
        const uint32_t restrictionCount = reader.getU32();
        if (!reader.ok() || restrictionCount > limits.maxFields) return false;
        field.restrictions.reserve(restrictionCount);
        for (uint32_t restrictionIndex = 0; restrictionIndex < restrictionCount; ++restrictionIndex) {
            SchemaRestriction restriction;
            restriction.kind = static_cast<RestrictionKind>(reader.getU8());
            const uint32_t payloadSize = reader.getU32();
            if (!reader.ok() || payloadSize > reader.remaining()) return false;
            const auto* payload = reader.getBytes(payloadSize);
            restriction.payload.assign(payload, payload + payloadSize);
            field.restrictions.push_back(std::move(restriction));
        }
        const uint32_t defaultSize = reader.getU32();
        if (!reader.ok() || defaultSize > reader.remaining()) return false;
        const auto* defaultValue = reader.getBytes(defaultSize);
        field.defaultValue.assign(defaultValue, defaultValue + defaultSize);
        decoded.fields.push_back(std::move(field));
    }
    const uint32_t variantCount = reader.getU32();
    if (!reader.ok() || variantCount > limits.maxFields) return false;
    decoded.oneOfMembers.reserve(variantCount);
    for (uint32_t index = 0; index < variantCount; ++index) {
        const uint32_t key = reader.getU32();
        SchemaRef value;
        if (!getRef(reader, value)) return false;
        decoded.oneOfMembers.emplace_back(key, value);
    }
    if (!reader.ok()) return false;
    node = std::move(decoded);
    return true;
}

} // namespace tether::io
