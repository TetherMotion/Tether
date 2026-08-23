/**
 * @file test_io_protocol.cpp
 * @brief Unit tests for Protocol.hpp — BufWriter, BufReader, varint, ValueType utilities.
 */
#include <gtest/gtest.h>
#include <magic_enum/magic_enum.hpp>
#include "tether/io/Protocol.hpp"

using namespace tether::io;

// ===========================================================================
// ValueType utilities
// ===========================================================================

TEST(IOProtocol, ValueTypeSizes) {
    EXPECT_EQ(valueTypeSize(ValueType::U8),   1);
    EXPECT_EQ(valueTypeSize(ValueType::U16),  2);
    EXPECT_EQ(valueTypeSize(ValueType::U32),  4);
    EXPECT_EQ(valueTypeSize(ValueType::U64),  8);
    EXPECT_EQ(valueTypeSize(ValueType::I8),   1);
    EXPECT_EQ(valueTypeSize(ValueType::I16),  2);
    EXPECT_EQ(valueTypeSize(ValueType::I32),  4);
    EXPECT_EQ(valueTypeSize(ValueType::I64),  8);
    EXPECT_EQ(valueTypeSize(ValueType::F32),  4);
    EXPECT_EQ(valueTypeSize(ValueType::F64),  8);
    EXPECT_EQ(valueTypeSize(ValueType::Bool), 1);
    EXPECT_EQ(valueTypeSize(ValueType::IPv4), 4);
    EXPECT_EQ(valueTypeSize(ValueType::IPv6), 16);
    EXPECT_EQ(valueTypeSize(ValueType::MAC), 6);
    EXPECT_EQ(valueTypeSize(ValueType::Enum), 0);
    EXPECT_EQ(valueTypeSize(ValueType::String), 0);
    EXPECT_EQ(valueTypeSize(ValueType::Binary), 0);
    EXPECT_EQ(valueTypeSize(ValueType::Struct), 0);
}

TEST(IOProtocol, IsVariableLength) {
    EXPECT_FALSE(isVariableLength(ValueType::U32));
    EXPECT_FALSE(isVariableLength(ValueType::F64));
    EXPECT_TRUE(isVariableLength(ValueType::String));
    EXPECT_TRUE(isVariableLength(ValueType::Binary));
    EXPECT_TRUE(isVariableLength(ValueType::Enum));
    EXPECT_TRUE(isVariableLength(ValueType::UVarint));
    EXPECT_TRUE(isVariableLength(ValueType::IVarint));
    EXPECT_TRUE(isVariableLength(ValueType::Struct));
}

TEST(IOProtocol, ValueTypeName) {
    EXPECT_STREQ(magic_enum::enum_name(ValueType::U8).data(), "U8");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::F64).data(), "F64");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::Bool).data(), "Bool");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::String).data(), "String");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::Enum).data(), "Enum");
}

// ===========================================================================
// BufWriter / BufReader roundtrips
// ===========================================================================

TEST(IOProtocol, BufWriterReaderU8) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putU8(0x42);
    w.putU8(0xFF);
    EXPECT_TRUE(w.ok());
    EXPECT_EQ(w.pos, 2u);

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getU8(), 0x42);
    EXPECT_EQ(r.getU8(), 0xFF);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderU16) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putU16(0x1234);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getU16(), 0x1234);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderU32) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putU32(0xDEADBEEF);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getU32(), 0xDEADBEEF);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderU64) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putU64(0x0123456789ABCDEFULL);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getU64(), 0x0123456789ABCDEFULL);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderI32) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putI32(-12345);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getI32(), -12345);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderF32) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putF32(3.14f);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_FLOAT_EQ(r.getF32(), 3.14f);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderF64) {
    uint8_t buf[16];
    BufWriter w(buf, sizeof(buf));
    w.putF64(2.718281828);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_DOUBLE_EQ(r.getF64(), 2.718281828);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterReaderStr16) {
    uint8_t buf[128];
    BufWriter w(buf, sizeof(buf));
    std::string str = "hello world";
    w.putStr16(str.c_str(), str.size());
    EXPECT_TRUE(w.ok());
    EXPECT_EQ(w.pos, 2u + str.size());

    BufReader r(buf, w.pos);
    uint16_t len = r.getU16();
    EXPECT_EQ(len, str.size());
    const uint8_t* data = r.getBytes(len);
    ASSERT_NE(data, nullptr);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(data), len), "hello world");
}

TEST(IOProtocol, BufWriterReaderBytes) {
    uint8_t buf[32];
    BufWriter w(buf, sizeof(buf));
    uint8_t payload[] = {1, 2, 3, 4, 5};
    w.putBytes(payload, 5);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    const uint8_t* p = r.getBytes(5);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p[0], 1);
    EXPECT_EQ(p[4], 5);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocol, BufWriterOverflow) {
    uint8_t buf[2];
    BufWriter w(buf, sizeof(buf));
    w.putU32(0xDEADBEEF);  // needs 4 bytes, only 2 available
    EXPECT_FALSE(w.ok());
}

TEST(IOProtocol, BufReaderOverflow) {
    uint8_t buf[2] = {0x01, 0x02};
    BufReader r(buf, sizeof(buf));
    r.getU32();  // needs 4 bytes, only 2 available
    EXPECT_FALSE(r.ok());
}

TEST(IOProtocol, BufReaderSkip) {
    uint8_t buf[8] = {};
    BufReader r(buf, sizeof(buf));
    EXPECT_TRUE(r.skip(4));
    EXPECT_EQ(r.remaining(), 4u);
    EXPECT_FALSE(r.skip(5));
    EXPECT_FALSE(r.ok());
}

// ===========================================================================
// Varint encoding/decoding
// ===========================================================================

TEST(IOProtocol, VarintSmallValue) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 42);
    EXPECT_EQ(n, 1u);
    EXPECT_EQ(buf[0], 42);

    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 1u);
    EXPECT_EQ(val, 42u);
}

TEST(IOProtocol, VarintMultiByteValue) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 300);
    EXPECT_EQ(n, 2u);

    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 2u);
    EXPECT_EQ(val, 300u);
}

TEST(IOProtocol, VarintMaxValue) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 0xFFFFFFFF);
    EXPECT_EQ(n, 5u);

    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 5u);
    EXPECT_EQ(val, 0xFFFFFFFF);
}

TEST(IOProtocol, VarintZero) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 0);
    EXPECT_EQ(n, 1u);
    EXPECT_EQ(buf[0], 0);

    uint32_t val = 99;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 1u);
    EXPECT_EQ(val, 0u);
}

TEST(IOProtocol, VarintBufferOverflow) {
    uint8_t buf[1];
    size_t n = encodeVarint(buf, sizeof(buf), 300);
    EXPECT_EQ(n, 0u);  // 300 needs 2 bytes, only 1 available
}

TEST(IOProtocol, BufWriterReaderVarint) {
    uint8_t buf[32];
    BufWriter w(buf, sizeof(buf));
    w.putVarint(0);
    w.putVarint(127);
    w.putVarint(128);
    w.putVarint(65535);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getVarint(), 0u);
    EXPECT_EQ(r.getVarint(), 127u);
    EXPECT_EQ(r.getVarint(), 128u);
    EXPECT_EQ(r.getVarint(), 65535u);
    EXPECT_TRUE(r.ok());
}

// ===========================================================================
// EntryFlags
// ===========================================================================

TEST(IOProtocol, EntryFlagBits) {
    EXPECT_EQ(EntryFlags::Readable, 0x01);
    EXPECT_EQ(EntryFlags::Writable, 0x02);
    EXPECT_EQ(EntryFlags::VariableLen, 0x04);
    EXPECT_EQ(EntryFlags::HasStruct, 0x08);
    EXPECT_EQ(EntryFlags::HasEnum, 0x10);
}

// ===========================================================================
// Mixed write/read roundtrip
// ===========================================================================

TEST(IOProtocol, MixedRoundtrip) {
    uint8_t buf[256];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::StreamData));
    w.putU32(42);      // specId
    w.putU32(2);       // rowCount
    w.putF64(1.0);     // row 1 value
    w.putF64(2.0);     // row 2 value
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getU8(), static_cast<uint8_t>(MessageType::StreamData));
    EXPECT_EQ(r.getU32(), 42u);
    EXPECT_EQ(r.getU32(), 2u);
    EXPECT_DOUBLE_EQ(r.getF64(), 1.0);
    EXPECT_DOUBLE_EQ(r.getF64(), 2.0);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.remaining(), 0u);
}
