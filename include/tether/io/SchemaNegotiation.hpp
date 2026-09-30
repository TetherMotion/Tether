#pragma once

#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaWire.hpp"

#include <string>
#include <vector>

namespace tether::io {

inline constexpr uint8_t SCHEMA_PROTOCOL_VERSION = 6;

enum class SchemaRejectCode : uint8_t {
    UnsupportedVersion = 1,
    LimitMismatch = 2,
    InvalidManifest = 3,
    InvalidDefinition = 4,
    MissingDependency = 5,
    DigestMismatch = 6,
};

struct SchemaNegotiationLimits {
    uint32_t maxMessageBytes = 1 << 20;
    uint32_t maxDescriptorBytes = 1 << 16;
    uint32_t maxDefinitions = 1024;
    uint32_t maxValueBytes = 1 << 20;
    uint32_t maxValueDepth = 32;
    uint32_t maxCollectionEntries = 1'000'000;
};

struct ClientHelloV6 {
    uint8_t minVersion = SCHEMA_PROTOCOL_VERSION;
    uint8_t maxVersion = SCHEMA_PROTOCOL_VERSION;
    uint32_t encodingFeatures = 0;
    SchemaNegotiationLimits limits;
    std::vector<SchemaManifestEntry> cachedSchemas;
};

struct ServerHelloV6 {
    uint8_t selectedVersion = SCHEMA_PROTOCOL_VERSION;
    uint32_t encodingFeatures = 0;
    SchemaNegotiationLimits limits;
    SchemaEpoch epoch = 0;
    std::vector<SchemaManifestEntry> schemas;
};

struct SchemaRequestV6 {
    SchemaEpoch epoch = 0;
    std::vector<SchemaRef> definitions;
};

struct SchemaCommitV6 {
    SchemaEpoch epoch = 0;
};

struct SchemaRejectV6 {
    SchemaRejectCode code = SchemaRejectCode::InvalidDefinition;
    std::string message;
};

namespace schema_negotiation_detail {
inline void putLimits(BufWriter& writer, const SchemaNegotiationLimits& limits) {
    writer.putU32(limits.maxMessageBytes);
    writer.putU32(limits.maxDescriptorBytes);
    writer.putU32(limits.maxDefinitions);
    writer.putU32(limits.maxValueBytes);
    writer.putU32(limits.maxValueDepth);
    writer.putU32(limits.maxCollectionEntries);
}

inline SchemaNegotiationLimits getLimits(BufReader& reader) {
    return {
        reader.getU32(), reader.getU32(), reader.getU32(),
        reader.getU32(), reader.getU32(), reader.getU32()
    };
}

inline void putManifestEntry(BufWriter& writer, const SchemaManifestEntry& entry) {
    schema_wire_detail::putKey(writer, entry.ref.key);
    schema_wire_detail::putDigest(writer, entry.ref.digest);
    writer.putU32(entry.revision);
}

inline bool getManifestEntry(BufReader& reader, SchemaManifestEntry& entry) {
    return schema_wire_detail::getKey(reader, entry.ref.key) &&
           schema_wire_detail::getDigest(reader, entry.ref.digest) &&
           (entry.revision = reader.getU32(), reader.ok());
}

inline void putManifest(BufWriter& writer, const std::vector<SchemaManifestEntry>& manifest) {
    writer.putU32(static_cast<uint32_t>(manifest.size()));
    for (const auto& entry : manifest) putManifestEntry(writer, entry);
}

inline bool getManifest(BufReader& reader, std::vector<SchemaManifestEntry>& manifest,
                        uint32_t maxEntries) {
    const uint32_t count = reader.getU32();
    if (!reader.ok() || count > maxEntries) return false;
    manifest.resize(count);
    for (auto& entry : manifest) {
        if (!getManifestEntry(reader, entry)) return false;
    }
    return true;
}
} // namespace schema_negotiation_detail

inline void encodeClientHelloV6(BufWriter& writer, const ClientHelloV6& hello) {
    using namespace schema_negotiation_detail;
    writer.putU8(hello.minVersion);
    writer.putU8(hello.maxVersion);
    writer.putU32(hello.encodingFeatures);
    putLimits(writer, hello.limits);
    putManifest(writer, hello.cachedSchemas);
}

inline bool decodeClientHelloV6(BufReader& reader, ClientHelloV6& hello,
                                uint32_t maxManifestEntries = 1024) {
    using namespace schema_negotiation_detail;
    ClientHelloV6 decoded;
    decoded.minVersion = reader.getU8();
    decoded.maxVersion = reader.getU8();
    decoded.encodingFeatures = reader.getU32();
    decoded.limits = getLimits(reader);
    if (!getManifest(reader, decoded.cachedSchemas, maxManifestEntries)) return false;
    hello = std::move(decoded);
    return reader.ok();
}

inline void encodeServerHelloV6(BufWriter& writer, const ServerHelloV6& hello) {
    using namespace schema_negotiation_detail;
    writer.putU8(hello.selectedVersion);
    writer.putU32(hello.encodingFeatures);
    putLimits(writer, hello.limits);
    writer.putU32(hello.epoch);
    putManifest(writer, hello.schemas);
}

inline bool decodeServerHelloV6(BufReader& reader, ServerHelloV6& hello,
                                uint32_t maxManifestEntries = 1024) {
    using namespace schema_negotiation_detail;
    ServerHelloV6 decoded;
    decoded.selectedVersion = reader.getU8();
    decoded.encodingFeatures = reader.getU32();
    decoded.limits = getLimits(reader);
    decoded.epoch = reader.getU32();
    if (!getManifest(reader, decoded.schemas, maxManifestEntries)) return false;
    hello = std::move(decoded);
    return reader.ok();
}

inline void encodeSchemaRequestV6(BufWriter& writer, const SchemaRequestV6& request) {
    writer.putU32(request.epoch);
    writer.putU32(static_cast<uint32_t>(request.definitions.size()));
    for (const auto& ref : request.definitions) schema_wire_detail::putRef(writer, ref);
}

inline bool decodeSchemaRequestV6(BufReader& reader, SchemaRequestV6& request,
                                  uint32_t maxDefinitions = 1024) {
    SchemaRequestV6 decoded;
    decoded.epoch = reader.getU32();
    const uint32_t count = reader.getU32();
    if (!reader.ok() || count > maxDefinitions) return false;
    decoded.definitions.resize(count);
    for (auto& ref : decoded.definitions) {
        if (!schema_wire_detail::getRef(reader, ref)) return false;
    }
    request = std::move(decoded);
    return reader.ok();
}

inline void encodeSchemaCommitV6(BufWriter& writer, const SchemaCommitV6& commit) {
    writer.putU32(commit.epoch);
}

inline bool decodeSchemaCommitV6(BufReader& reader, SchemaCommitV6& commit) {
    SchemaCommitV6 decoded{reader.getU32()};
    if (!reader.ok()) return false;
    commit = decoded;
    return true;
}

inline void encodeSchemaRejectV6(BufWriter& writer, const SchemaRejectV6& reject) {
    writer.putU8(static_cast<uint8_t>(reject.code));
    schema_wire_detail::putString(writer, reject.message);
}

inline bool decodeSchemaRejectV6(BufReader& reader, SchemaRejectV6& reject,
                                 uint32_t maxMessageBytes = 4096) {
    SchemaRejectV6 decoded;
    decoded.code = static_cast<SchemaRejectCode>(reader.getU8());
    if (!schema_wire_detail::getString(reader, decoded.message, maxMessageBytes)) return false;
    reject = std::move(decoded);
    return reader.ok();
}

} // namespace tether::io
