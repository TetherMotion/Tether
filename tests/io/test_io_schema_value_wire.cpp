#include "tether/io/SchemaValueWire.hpp"
#include "tether/io/SchemaValueCodec.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>

namespace tether::io {
namespace {

TEST(IOSchemaValueWire, ZeroTerminatedStringsRoundTripAndRejectInteriorNull) {
    std::array<uint8_t, 64> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    ASSERT_TRUE(encodeSchemaString(writer, "axis-position"));

    BufReader reader(bytes.data(), writer.pos);
    std::string decoded;
    ASSERT_TRUE(decodeSchemaString(reader, decoded, 64));
    EXPECT_EQ(decoded, "axis-position");
    EXPECT_EQ(reader.remaining(), 0U);

    BufWriter invalid(bytes.data(), bytes.size());
    EXPECT_FALSE(encodeSchemaString(invalid, std::string_view("bad\0value", 9)));
}

TEST(IOSchemaValueWire, ArbitraryBytesUseCanonicalU64Length) {
    const std::array<uint8_t, 4> source{0, 0xFF, 0x01, 0x00};
    std::array<uint8_t, 64> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    ASSERT_TRUE(encodeSchemaBytes(writer, source.data(), source.size()));

    BufReader reader(bytes.data(), writer.pos);
    std::vector<uint8_t> decoded;
    ASSERT_TRUE(decodeSchemaBytes(reader, decoded, 64));
    EXPECT_EQ(decoded, std::vector<uint8_t>(source.begin(), source.end()));

    const std::array<uint8_t, 2> nonCanonical{0x80, 0x00};
    BufReader malformed(nonCanonical.data(), nonCanonical.size());
    EXPECT_FALSE(decodeSchemaBytes(malformed, decoded, 64));
}

TEST(IOSchemaValueWire, MapAndOneOfFramingIsBounded) {
    std::array<uint8_t, 128> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    ASSERT_TRUE(encodeSchemaMapEntryCount(writer, 300));
    const std::array<uint8_t, 3> payload{1, 2, 3};
    ASSERT_TRUE(encodeSchemaOneOf(writer, 42, payload.data(), payload.size()));

    BufReader reader(bytes.data(), writer.pos);
    uint64_t count = 0;
    ASSERT_TRUE(decodeSchemaMapEntryCount(reader, count, 300));
    EXPECT_EQ(count, 300U);
    uint32_t member = 0;
    std::vector<uint8_t> decoded;
    ASSERT_TRUE(decodeSchemaOneOf(reader, member, decoded, 8));
    EXPECT_EQ(member, 42U);
    EXPECT_EQ(decoded, std::vector<uint8_t>(payload.begin(), payload.end()));
}

TEST(IOSchemaValueWire, ValidatesNestedTaggedStructOptionalAndOneOf) {
    SchemaNode scalar;
    scalar.key[0] = 1;
    scalar.kind = SchemaKind::Scalar;
    scalar.scalarType = ValueType::U32;

    SchemaNode text;
    text.key[0] = 2;
    text.kind = SchemaKind::String;

    SchemaNode choice;
    choice.key[0] = 3;
    choice.kind = SchemaKind::OneOf;
    choice.oneOfMembers = {{10, SchemaRef{scalar.key, {}}}, {20, SchemaRef{text.key, {}}}};

    SchemaNode record;
    record.key[0] = 4;
    record.kind = SchemaKind::Struct;
    record.structEncoding = StructEncoding::Tagged;
    record.fields = {
        {1, 0, SchemaRef{scalar.key, {}}, "required", ""},
        {2, 0, SchemaRef{choice.key, {}}, "choice", ""},
    };
    record.fields[1].presence = FieldPresence::Optional;
    const SchemaGraph graph{{scalar, text, choice, record}};

    std::array<uint8_t, 128> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU8(2); // field count
    writer.putU8(1); // required field key
    writer.putU8(4); // payload length
    writer.putU32(42);
    writer.putU8(2); // optional field key
    writer.putU8(6); // oneof payload length
    writer.putU8(10); // member key
    writer.putU8(4); // member payload length
    writer.putU32(7);
    ASSERT_TRUE(writer.ok());

    BufReader reader(bytes.data(), writer.pos);
    ASSERT_TRUE(validateSchemaValue(graph, record.key, reader));
    EXPECT_EQ(reader.remaining(), 0U);

    BufReader missing(bytes.data() + 1, writer.pos - 1);
    EXPECT_FALSE(validateSchemaValue(graph, record.key, missing));
}

TEST(IOSchemaValueWire, RejectsDuplicateOrOutOfOrderMapKeys) {
    SchemaNode key;
    key.key[0] = 1;
    key.kind = SchemaKind::String;
    SchemaNode value;
    value.key[0] = 2;
    value.kind = SchemaKind::Scalar;
    value.scalarType = ValueType::U8;
    SchemaNode map;
    map.key[0] = 3;
    map.kind = SchemaKind::Map;
    map.mapKey = SchemaRef{key.key, {}};
    map.mapValue = SchemaRef{value.key, {}};
    map.maxCount = 4;
    const SchemaGraph graph{{key, value, map}};

    std::array<uint8_t, 64> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU8(2); // entry count
    writer.putU8(2); writer.putBytes("b\0", 2); writer.putU8(1); writer.putU8(2);
    writer.putU8(2); writer.putBytes("b\0", 2); writer.putU8(1); writer.putU8(3);
    BufReader reader(bytes.data(), writer.pos);
    EXPECT_FALSE(validateSchemaValue(graph, map.key, reader));
}

} // namespace
} // namespace tether::io
