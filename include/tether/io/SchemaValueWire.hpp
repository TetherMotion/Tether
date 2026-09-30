#pragma once

#include "tether/io/Schema.hpp"

#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace tether::io {
namespace schema_value_wire_detail {

inline void putU64Varint(BufWriter& writer, uint64_t value) {
    while (value >= 0x80) {
        writer.putU8(static_cast<uint8_t>(value) | 0x80U);
        value >>= 7;
    }
    writer.putU8(static_cast<uint8_t>(value));
}

inline bool getU64Varint(BufReader& reader, uint64_t& value) {
    value = 0;
    for (unsigned index = 0; index < 10; ++index) {
        const uint8_t byte = reader.getU8();
        if (!reader.ok()) return false;
        if (index == 9 && byte > 1) return false;
        value |= static_cast<uint64_t>(byte & 0x7FU) << (index * 7U);
        if ((byte & 0x80U) == 0) {
            if (index > 0 && byte == 0) return false;
            return true;
        }
    }
    return false;
}

} // namespace schema_value_wire_detail

inline bool encodeSchemaString(BufWriter& writer, std::string_view value) {
    if (value.find('\0') != std::string_view::npos) {
        writer.overflow = true;
        return false;
    }
    writer.putBytes(value.data(), value.size());
    writer.putU8(0);
    return writer.ok();
}

inline bool decodeSchemaString(BufReader& reader, std::string& value,
                               uint64_t maxBytes) {
    value.clear();
    while (reader.remaining() != 0 && value.size() <= maxBytes) {
        const uint8_t byte = reader.getU8();
        if (!reader.ok()) return false;
        if (byte == 0) return true;
        value.push_back(static_cast<char>(byte));
    }
    return false;
}

inline bool encodeSchemaBytes(BufWriter& writer, const uint8_t* data, uint64_t size) {
    if (size > std::numeric_limits<size_t>::max()) {
        writer.overflow = true;
        return false;
    }
    schema_value_wire_detail::putU64Varint(writer, size);
    writer.putBytes(data, static_cast<size_t>(size));
    return writer.ok();
}

inline bool decodeSchemaBytes(BufReader& reader, std::vector<uint8_t>& value,
                              uint64_t maxBytes) {
    uint64_t size = 0;
    if (!schema_value_wire_detail::getU64Varint(reader, size) || size > maxBytes ||
        size > reader.remaining()) return false;
    const auto* data = reader.getBytes(static_cast<size_t>(size));
    if (!reader.ok()) return false;
    value.assign(data, data + size);
    return true;
}

inline bool encodeSchemaMapEntryCount(BufWriter& writer, uint64_t count) {
    schema_value_wire_detail::putU64Varint(writer, count);
    return writer.ok();
}

inline bool decodeSchemaMapEntryCount(BufReader& reader, uint64_t& count,
                                      uint64_t maxEntries) {
    return schema_value_wire_detail::getU64Varint(reader, count) && count <= maxEntries;
}

inline bool encodeSchemaOneOf(BufWriter& writer, uint32_t memberKey,
                              const uint8_t* payload, uint64_t payloadSize) {
    if (memberKey == 0 || payloadSize > std::numeric_limits<size_t>::max()) {
        writer.overflow = true;
        return false;
    }
    schema_value_wire_detail::putU64Varint(writer, memberKey);
    schema_value_wire_detail::putU64Varint(writer, payloadSize);
    writer.putBytes(payload, static_cast<size_t>(payloadSize));
    return writer.ok();
}

inline bool decodeSchemaOneOf(BufReader& reader, uint32_t& memberKey,
                              std::vector<uint8_t>& payload, uint64_t maxPayload) {
    uint64_t key = 0;
    uint64_t size = 0;
    if (!schema_value_wire_detail::getU64Varint(reader, key) || key == 0 || key > UINT32_MAX ||
        !schema_value_wire_detail::getU64Varint(reader, size) || size > maxPayload ||
        size > reader.remaining()) return false;
    const auto* data = reader.getBytes(static_cast<size_t>(size));
    if (!reader.ok()) return false;
    memberKey = static_cast<uint32_t>(key);
    payload.assign(data, data + size);
    return true;
}

} // namespace tether::io
