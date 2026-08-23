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

TEST(IOWireContract, FilterSchemaValidatesTypeAndRange) {
    StreamFilterSchema schema;
    schema.defineProperty({"threshold", ValueType::F32, true, true, 0.0, 100.0});

    FilterProperty valid;
    valid.name = "threshold";
    valid.value.type = ValueType::F32;
    uint8_t encoded[4];
    BufWriter writer(encoded, sizeof(encoded));
    writer.putF32(42.0f);
    valid.value.data.assign(encoded, encoded + sizeof(encoded));
    EXPECT_TRUE(schema.validate(valid).ok);

    valid.value.type = ValueType::U32;
    EXPECT_EQ(schema.validate(valid).errorType, FilterPropertyErrorType::WrongDataType);
}

TEST(IOWireContract, FunctionTlvLittleEndianVector) {
    uint8_t bytes[13] = {};
    const uint8_t value[] = {0xAA, 0xBB, 0xCC, 0xDD};
    BufWriter writer(bytes, sizeof(bytes));
    ASSERT_TRUE(encodeFunctionTlv(writer, 0x78563412, ValueType::U32,
                                  value, sizeof(value)));
    const uint8_t expected[] = {
        0x12, 0x34, 0x56, 0x78, 0x03, 0x04, 0x00, 0x00, 0x00,
        0xAA, 0xBB, 0xCC, 0xDD};
    EXPECT_EQ(std::vector<uint8_t>(bytes, bytes + sizeof(bytes)),
              std::vector<uint8_t>(expected, expected + sizeof(expected)));

    BufReader reader(bytes, sizeof(bytes));
    FunctionArgument argument;
    ASSERT_TRUE(decodeFunctionTlv(reader, argument));
    EXPECT_EQ(argument.position, 0x78563412u);
    EXPECT_EQ(argument.type, ValueType::U32);
    EXPECT_EQ(argument.value, std::vector<uint8_t>(value, value + sizeof(value)));
    EXPECT_EQ(reader.remaining(), 0u);
}

TEST(IOWireContract, FunctionTlvRejectsTruncatedAndOversizedValues) {
    const uint8_t truncated[] = {0, 0, 0, 0, static_cast<uint8_t>(ValueType::Binary),
                                 4, 0, 0, 0, 1, 2};
    BufReader truncatedReader(truncated, sizeof(truncated));
    FunctionArgument argument;
    EXPECT_FALSE(decodeFunctionTlv(truncatedReader, argument));
    EXPECT_FALSE(truncatedReader.ok());

    uint8_t oversized[FUNCTION_TLV_HEADER_SIZE] = {};
    BufWriter oversizedWriter(oversized, sizeof(oversized));
    oversizedWriter.putU32(0);
    oversizedWriter.putU8(static_cast<uint8_t>(ValueType::Binary));
    oversizedWriter.putU32(static_cast<uint32_t>(MAX_VARIABLE_VALUE_SIZE + 1));
    ASSERT_TRUE(oversizedWriter.ok());
    BufReader oversizedReader(oversized, sizeof(oversized));
    EXPECT_FALSE(decodeFunctionTlv(oversizedReader, argument));
    EXPECT_FALSE(oversizedReader.ok());
}