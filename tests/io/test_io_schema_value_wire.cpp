#include "tether/io/SchemaValueWire.hpp"

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

} // namespace
} // namespace tether::io
