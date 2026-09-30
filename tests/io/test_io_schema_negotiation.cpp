#include "tether/io/SchemaNegotiation.hpp"

#include <gtest/gtest.h>

#include <array>

namespace tether::io {
namespace {

SchemaRef negotiationRef(uint8_t value) {
    SchemaRef ref;
    ref.key[0] = value;
    ref.digest[0] = static_cast<uint8_t>(value + 10);
    return ref;
}

TEST(IOSchemaNegotiation, ClientAndServerHelloRoundTrip) {
    ClientHelloV6 source;
    source.encodingFeatures = 0xA5;
    source.limits.maxValueBytes = 8192;
    source.cachedSchemas = {{negotiationRef(1), 3}, {negotiationRef(2), 4}};

    std::array<uint8_t, 512> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    encodeClientHelloV6(writer, source);
    ASSERT_TRUE(writer.ok());

    BufReader reader(bytes.data(), writer.pos);
    ClientHelloV6 decoded;
    ASSERT_TRUE(decodeClientHelloV6(reader, decoded));
    EXPECT_EQ(decoded.encodingFeatures, source.encodingFeatures);
    EXPECT_EQ(decoded.limits.maxValueBytes, 8192U);
    EXPECT_EQ(decoded.cachedSchemas, source.cachedSchemas);

    ServerHelloV6 server;
    server.encodingFeatures = 7;
    server.epoch = 42;
    server.schemas = source.cachedSchemas;
    BufWriter serverWriter(bytes.data(), bytes.size());
    encodeServerHelloV6(serverWriter, server);
    BufReader serverReader(bytes.data(), serverWriter.pos);
    ServerHelloV6 serverDecoded;
    ASSERT_TRUE(decodeServerHelloV6(serverReader, serverDecoded));
    EXPECT_EQ(serverDecoded.epoch, 42U);
    EXPECT_EQ(serverDecoded.schemas, server.schemas);
}

TEST(IOSchemaNegotiation, RequestCommitAndRejectRoundTrip) {
    SchemaRequestV6 request;
    request.epoch = 9;
    request.definitions = {negotiationRef(3), negotiationRef(4)};
    std::array<uint8_t, 512> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    encodeSchemaRequestV6(writer, request);
    BufReader reader(bytes.data(), writer.pos);
    SchemaRequestV6 decodedRequest;
    ASSERT_TRUE(decodeSchemaRequestV6(reader, decodedRequest));
    EXPECT_EQ(decodedRequest.epoch, request.epoch);
    EXPECT_EQ(decodedRequest.definitions, request.definitions);

    BufWriter commitWriter(bytes.data(), bytes.size());
    encodeSchemaCommitV6(commitWriter, {9});
    BufReader commitReader(bytes.data(), commitWriter.pos);
    SchemaCommitV6 commit;
    ASSERT_TRUE(decodeSchemaCommitV6(commitReader, commit));
    EXPECT_EQ(commit.epoch, 9U);

    SchemaRejectV6 reject{SchemaRejectCode::DigestMismatch, "digest mismatch"};
    BufWriter rejectWriter(bytes.data(), bytes.size());
    encodeSchemaRejectV6(rejectWriter, reject);
    BufReader rejectReader(bytes.data(), rejectWriter.pos);
    SchemaRejectV6 decodedReject;
    ASSERT_TRUE(decodeSchemaRejectV6(rejectReader, decodedReject));
    EXPECT_EQ(decodedReject.code, reject.code);
    EXPECT_EQ(decodedReject.message, reject.message);
}

TEST(IOSchemaNegotiation, ManifestLimitIsEnforced) {
    ClientHelloV6 source;
    source.cachedSchemas = {{negotiationRef(1), 1}, {negotiationRef(2), 1}};
    std::array<uint8_t, 512> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    encodeClientHelloV6(writer, source);
    BufReader reader(bytes.data(), writer.pos);
    ClientHelloV6 decoded;
    EXPECT_FALSE(decodeClientHelloV6(reader, decoded, 1));
}

} // namespace
} // namespace tether::io
