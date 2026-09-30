#include <gtest/gtest.h>
#include "tether/io/Protocol.hpp"
#include "tether/io/Function.hpp"

using namespace tether::io;

TEST(IOWireContract, LittleEndianVectors) {
    uint8_t bytes[14] = {};
    BufWriter writer(bytes, sizeof(bytes));
    writer.putU16(0x1234);
    writer.putU32(0x78563412);
    writer.putU64(0xEFCDAB9078563412ULL);
    ASSERT_TRUE(writer.ok());
    const uint8_t expected[] = {
        0x34, 0x12, 0x12, 0x34, 0x56, 0x78,
        0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF};
    EXPECT_EQ(std::vector<uint8_t>(bytes, bytes + sizeof(bytes)),
              std::vector<uint8_t>(expected, expected + sizeof(expected)));
}

TEST(IOWireContract, VarintRejectsOverflow) {
    const uint8_t malformed[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x10};
    uint32_t value = 0;
    EXPECT_EQ(decodeVarint(malformed, sizeof(malformed), value), 0u);
    BufReader reader(malformed, sizeof(malformed));
    reader.getVarint();
    EXPECT_FALSE(reader.ok());
}

TEST(IOWireContract, ZigzagVectors) {
    EXPECT_EQ(zigzagEncode32(-1), 1u);
    EXPECT_EQ(zigzagEncode32(1), 2u);
    EXPECT_EQ(zigzagDecode32(1), -1);
    EXPECT_EQ(zigzagDecode32(2), 1);
    EXPECT_EQ(zigzagEncode64(-1), 1u);
    EXPECT_EQ(zigzagDecode64(1), -1);
}

TEST(IOWireContract, FilterSchemaValidatesSchemaKey) {
    StreamFilterSchema schema;
    std::array<uint8_t, 16> expected{};
    expected[0] = 7;
    schema.defineProperty({"threshold", expected, true});

    FilterProperty valid;
    valid.name = "threshold";
    valid.value.schemaKey = expected;
    valid.value.data = {42, 0, 0, 0};
    EXPECT_TRUE(schema.validate(valid).ok);

    valid.value.schemaKey[0] = 8;
    EXPECT_EQ(schema.validate(valid).errorType, FilterPropertyErrorType::WrongDataType);
}

TEST(IOWireContract, FunctionValueVarintVector) {
    uint8_t bytes[10] = {};
    const uint8_t value[] = {0xAA, 0xBB, 0xCC, 0xDD};
    BufWriter writer(bytes, sizeof(bytes));
    ASSERT_TRUE(encodeFunctionValue(writer, 0x12, value, sizeof(value)));
    const uint8_t expected[] = {
        0x12, 0x04, 0xAA, 0xBB, 0xCC, 0xDD};
    EXPECT_EQ(std::vector<uint8_t>(bytes, bytes + writer.pos),
              std::vector<uint8_t>(expected, expected + sizeof(expected)));

    BufReader reader(bytes, writer.pos);
    FunctionArgument argument;
    ASSERT_TRUE(decodeFunctionValue(reader, argument));
    EXPECT_EQ(argument.key, 0x12u);
    EXPECT_EQ(argument.value, std::vector<uint8_t>(value, value + sizeof(value)));
    EXPECT_EQ(reader.remaining(), 0u);
}

TEST(IOWireContract, FunctionValueRejectsTruncatedAndOversizedValues) {
    const uint8_t truncated[] = {1, 4, 1, 2};
    BufReader truncatedReader(truncated, sizeof(truncated));
    FunctionArgument argument;
    EXPECT_FALSE(decodeFunctionValue(truncatedReader, argument));
    EXPECT_FALSE(truncatedReader.ok());

    uint8_t oversized[FUNCTION_VALUE_HEADER_MAX_SIZE] = {};
    BufWriter oversizedWriter(oversized, sizeof(oversized));
    oversizedWriter.putVarint(1);
    oversizedWriter.putVarint(static_cast<uint32_t>(MAX_VARIABLE_VALUE_SIZE + 1));
    ASSERT_TRUE(oversizedWriter.ok());
    BufReader oversizedReader(oversized, oversizedWriter.pos);
    EXPECT_FALSE(decodeFunctionValue(oversizedReader, argument));
    EXPECT_FALSE(oversizedReader.ok());
}