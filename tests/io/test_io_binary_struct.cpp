/**
 * @file test_io_binary_struct.cpp
 * @brief Unit tests for BinaryStruct — StructField, StructDescriptor encode/decode.
 */
#include <gtest/gtest.h>
#include "tether/io/BinaryStruct.hpp"

using namespace tether::io;

TEST(IOBinaryStruct, EncodeDecodeRoundtrip) {
    StructDescriptor desc;
    desc.entryId = 0xABCD;
    desc.name = "TestStruct";
    desc.totalSize = 16;
    desc.fields = {
        {"x_pos", ValueType::F32, 0, 4, "mm"},
        {"y_pos", ValueType::F32, 4, 4, "mm"},
        {"z_pos", ValueType::F32, 8, 4, "mm"},
        {"flags", ValueType::U32, 12, 4, ""},
    };

    uint8_t buf[512];
    BufWriter w(buf, sizeof(buf));
    desc.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    StructDescriptor decoded;
    EXPECT_TRUE(StructDescriptor::decode(r, decoded));

    EXPECT_EQ(decoded.entryId, 0xABCDu);
    EXPECT_EQ(decoded.name, "TestStruct");
    EXPECT_EQ(decoded.totalSize, 16u);
    ASSERT_EQ(decoded.fields.size(), 4u);

    EXPECT_EQ(decoded.fields[0].name, "x_pos");
    EXPECT_EQ(decoded.fields[0].type, ValueType::F32);
    EXPECT_EQ(decoded.fields[0].offset, 0u);
    EXPECT_EQ(decoded.fields[0].size, 4u);
    EXPECT_EQ(decoded.fields[0].unit, "mm");

    EXPECT_EQ(decoded.fields[3].name, "flags");
    EXPECT_EQ(decoded.fields[3].type, ValueType::U32);
    EXPECT_EQ(decoded.fields[3].unit, "");
}

TEST(IOBinaryStruct, EmptyStruct) {
    StructDescriptor desc;
    desc.entryId = 1;
    desc.name = "Empty";
    desc.totalSize = 0;

    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    desc.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    StructDescriptor decoded;
    EXPECT_TRUE(StructDescriptor::decode(r, decoded));
    EXPECT_EQ(decoded.name, "Empty");
    EXPECT_EQ(decoded.fields.size(), 0u);
}

TEST(IOBinaryStruct, DecodeFromTruncatedBuffer) {
    uint8_t buf[4] = {0x01, 0x02, 0x03, 0x04};
    BufReader r(buf, sizeof(buf));
    StructDescriptor decoded;
    EXPECT_FALSE(StructDescriptor::decode(r, decoded));
}
