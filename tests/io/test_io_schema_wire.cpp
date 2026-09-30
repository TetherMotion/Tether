#include "tether/io/SchemaWire.hpp"

#include <gtest/gtest.h>

namespace tether::io {
namespace {

SchemaKey wireKey(uint8_t value) {
    SchemaKey result{};
    result[0] = value;
    return result;
}

SchemaRef wireRef(uint8_t value) { return SchemaRef{wireKey(value), {}}; }

TEST(IOSchemaWire, DefinitionRoundTrips) {
    SchemaNode source;
    source.key = wireKey(1);
    source.revision = 7;
    source.kind = SchemaKind::Struct;
    source.flags = 3;
    source.name = "drive.snapshot";
    source.description = "A nested drive value";
    source.annotations["ui.unit"] = "mm";
    source.fields.push_back({11, SchemaFieldFlags::Required, wireRef(2), "position", "Actual position"});
    source.fields.push_back({12, 0, wireRef(3), "label", "Optional label"});
    source.oneOfMembers.emplace_back(21, wireRef(4));

    std::array<uint8_t, 2048> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    encodeSchemaDefinition(writer, source);
    ASSERT_TRUE(writer.ok());

    BufReader reader(bytes.data(), writer.pos);
    SchemaNode decoded;
    ASSERT_TRUE(decodeSchemaDefinition(reader, decoded));
    EXPECT_EQ(reader.remaining(), 0U);
    EXPECT_EQ(decoded.key, source.key);
    EXPECT_EQ(decoded.revision, source.revision);
    EXPECT_EQ(decoded.name, source.name);
    EXPECT_EQ(decoded.annotations, source.annotations);
    EXPECT_EQ(decoded.fields.size(), source.fields.size());
    EXPECT_EQ(decoded.fields[0].schema.key, source.fields[0].schema.key);
    EXPECT_EQ(decoded.oneOfMembers, source.oneOfMembers);
}

TEST(IOSchemaWire, RejectsTruncatedAndOversizedDefinitions) {
    SchemaNode source;
    source.key = wireKey(1);
    source.kind = SchemaKind::String;
    source.maxBytes = 16;
    source.name = "string";

    std::array<uint8_t, 256> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    encodeSchemaDefinition(writer, source);
    ASSERT_TRUE(writer.ok());

    BufReader truncated(bytes.data(), writer.pos - 1);
    SchemaNode decoded;
    EXPECT_FALSE(decodeSchemaDefinition(truncated, decoded));

    BufReader oversized(bytes.data(), writer.pos);
    SchemaLimits limits;
    limits.maxStringBytes = 0;
    EXPECT_FALSE(decodeSchemaDefinition(oversized, decoded, limits));
}

} // namespace
} // namespace tether::io
