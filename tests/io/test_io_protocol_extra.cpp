/**
 * @file test_io_protocol_extra.cpp
 * @brief Additional coverage tests for Protocol.hpp edge cases.
 */
#include <gtest/gtest.h>
#include <magic_enum/magic_enum.hpp>
#include "tether/io/Protocol.hpp"
#include "tether/io/ParameterExposer.hpp"

using namespace tether::io;

// ===========================================================================
// BufWriter edge cases
// ===========================================================================

TEST(IOProtocolExtra, BufWriterExactCapacity) {
    uint8_t buf[4];
    BufWriter w(buf, sizeof(buf));
    w.putU32(0x12345678);
    EXPECT_TRUE(w.ok());
    EXPECT_EQ(w.pos, 4u);

    // One more byte should overflow
    w.putU8(1);
    EXPECT_FALSE(w.ok());
}

TEST(IOProtocolExtra, BufWriterMultipleOverflows) {
    uint8_t buf[2];
    BufWriter w(buf, sizeof(buf));
    w.putU8(1);
    w.putU8(2);
    EXPECT_TRUE(w.ok());

    // These all overflow, but should not crash
    w.putU16(0);
    w.putU32(0);
    w.putU64(0);
    w.putI32(0);
    w.putF32(0);
    w.putF64(0);
    w.putBytes("abc", 3);
    w.putStr16("x", 1);
    w.putVarint(1);
    EXPECT_FALSE(w.ok());
}

TEST(IOProtocolExtra, BufWriterPutI32Positive) {
    uint8_t buf[4];
    BufWriter w(buf, sizeof(buf));
    w.putI32(12345);
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getI32(), 12345);
}

// ===========================================================================
// BufReader edge cases
// ===========================================================================

TEST(IOProtocolExtra, BufReaderMultipleErrors) {
    uint8_t buf[2] = {1, 2};
    BufReader r(buf, sizeof(buf));
    r.getU32();  // error (not enough data)
    EXPECT_FALSE(r.ok());

    // Error flag is sticky but methods don't early-return on it.
    // Position is not advanced by the failed getU32, so getU8 still succeeds
    // in reading from the buffer. The error flag remains set however.
    EXPECT_EQ(r.getU8(), 1);
    EXPECT_FALSE(r.ok());  // still in error state

    // Further overflows also fail
    EXPECT_EQ(r.getU64(), 0u);
    EXPECT_EQ(r.getI32(), 0);
    EXPECT_FLOAT_EQ(r.getF32(), 0.0f);
    EXPECT_DOUBLE_EQ(r.getF64(), 0.0);
    EXPECT_FALSE(r.skip(100));
    EXPECT_FALSE(r.ok());
}

TEST(IOProtocolExtra, BufReaderRemainingAtEnd) {
    uint8_t buf[4] = {1, 2, 3, 4};
    BufReader r(buf, sizeof(buf));
    r.skip(4);
    EXPECT_EQ(r.remaining(), 0u);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocolExtra, BufReaderGetBytesZero) {
    uint8_t buf[4] = {1, 2, 3, 4};
    BufReader r(buf, sizeof(buf));
    const uint8_t* p = r.getBytes(0);
    EXPECT_NE(p, nullptr);
    EXPECT_EQ(r.remaining(), 4u);
}

TEST(IOProtocolExtra, BufReaderVarintMultiByte) {
    uint8_t buf[8];
    BufWriter w(buf, sizeof(buf));
    w.putVarint(0x0FFFFFFF);  // large value
    EXPECT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    EXPECT_EQ(r.getVarint(), 0x0FFFFFFFu);
    EXPECT_TRUE(r.ok());
}

// ===========================================================================
// Varint boundary values
// ===========================================================================

TEST(IOProtocolExtra, Varint128) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 128);
    EXPECT_EQ(n, 2u);

    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 2u);
    EXPECT_EQ(val, 128u);
}

TEST(IOProtocolExtra, Varint16383) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 16383);
    EXPECT_EQ(n, 2u);

    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 2u);
    EXPECT_EQ(val, 16383u);
}

TEST(IOProtocolExtra, Varint16384) {
    uint8_t buf[5];
    size_t n = encodeVarint(buf, sizeof(buf), 16384);
    EXPECT_EQ(n, 3u);

    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, n, val);
    EXPECT_EQ(consumed, 3u);
    EXPECT_EQ(val, 16384u);
}

TEST(IOProtocolExtra, VarintDecodeIncomplete) {
    // A continuation byte with no terminator
    uint8_t buf[1] = {0x80};  // continuation bit set, no final byte
    uint32_t val = 0;
    size_t consumed = decodeVarint(buf, 1, val);
    EXPECT_EQ(consumed, 0u);
}

TEST(IOProtocolExtra, VarintEncodeCapZero) {
    uint8_t buf[1];
    size_t n = encodeVarint(buf, 0, 42);
    EXPECT_EQ(n, 0u);
}

// ===========================================================================
// ValueType utilities coverage
// ===========================================================================

TEST(IOProtocolExtra, ValueTypeSizeI8I16I64) {
    EXPECT_EQ(valueTypeSize(ValueType::I8), 1u);
    EXPECT_EQ(valueTypeSize(ValueType::I16), 2u);
    EXPECT_EQ(valueTypeSize(ValueType::I64), 8u);
}

TEST(IOProtocolExtra, ValueTypeSizeUnknown) {
    EXPECT_EQ(valueTypeSize(static_cast<ValueType>(200)), 0u);
}

TEST(IOProtocolExtra, ValueTypeNameAllTypes) {
    EXPECT_STREQ(magic_enum::enum_name(ValueType::U16).data(), "U16");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::U64).data(), "U64");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::I8).data(), "I8");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::I16).data(), "I16");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::I32).data(), "I32");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::I64).data(), "I64");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::F32).data(), "F32");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::Binary).data(), "Binary");
    EXPECT_STREQ(magic_enum::enum_name(ValueType::Struct).data(), "Struct");
}

TEST(IOProtocolExtra, IsVariableLengthCoverage) {
    EXPECT_FALSE(isVariableLength(ValueType::Bool));
    EXPECT_FALSE(isVariableLength(ValueType::Enum));
    EXPECT_FALSE(isVariableLength(ValueType::I32));
    EXPECT_TRUE(isVariableLength(ValueType::String));
    EXPECT_TRUE(isVariableLength(ValueType::Binary));
}

// ===========================================================================
// EntryFlags
// ===========================================================================

TEST(IOProtocolExtra, EntryFlagCombinations) {
    uint8_t f = EntryFlags::Readable | EntryFlags::Writable | EntryFlags::HasEnum;
    EXPECT_TRUE(f & EntryFlags::Readable);
    EXPECT_TRUE(f & EntryFlags::Writable);
    EXPECT_TRUE(f & EntryFlags::HasEnum);
    EXPECT_FALSE(f & EntryFlags::VariableLen);
}

// ===========================================================================
// MessageType casting
// ===========================================================================

TEST(IOProtocolExtra, MessageTypeValues) {
    EXPECT_EQ(static_cast<uint8_t>(MessageType::ListParamsReq), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(MessageType::DescribeStructResp), 0x21);
    EXPECT_EQ(static_cast<uint8_t>(MessageType::CatalogChanged), 0x19);
    EXPECT_EQ(static_cast<uint8_t>(MessageType::Error), 0x10);
}

// ===========================================================================
// ParameterExposer makeId
// ===========================================================================

TEST(IOProtocolExtra, MakeIdHelper) {
    uint64_t id = makeId(ModuleId::PIDController, 42);
    EXPECT_EQ(id, (0x0003ULL << 32) | 42);
}

TEST(IOProtocolExtra, ModuleIdConstants) {
    EXPECT_EQ(ModuleId::EtherCATMaster, 0x0001u);
    EXPECT_EQ(ModuleId::CiA402Drive, 0x0002u);
    EXPECT_EQ(ModuleId::PIDController, 0x0003u);
    EXPECT_EQ(ModuleId::MotionPlanner, 0x0004u);
    EXPECT_EQ(ModuleId::GCodeInterpreter, 0x0005u);
    EXPECT_EQ(ModuleId::Simulation, 0x0006u);
    EXPECT_EQ(ModuleId::User, 0x1000u);
}

// ===========================================================================
// BufWriter/BufReader Str16 edge cases
// ===========================================================================

TEST(IOProtocolExtra, Str16EmptyString) {
    uint8_t buf[32];
    BufWriter w(buf, sizeof(buf));
    w.putStr16("", 0);
    EXPECT_TRUE(w.ok());
    EXPECT_EQ(w.pos, 2u);  // just the length prefix

    BufReader r(buf, w.pos);
    uint16_t len = r.getU16();
    EXPECT_EQ(len, 0u);
    EXPECT_TRUE(r.ok());
}

TEST(IOProtocolExtra, Str16LongString) {
    std::string longStr(500, 'X');
    std::vector<uint8_t> buf(600);
    BufWriter w(buf.data(), buf.size());
    w.putStr16(longStr.c_str(), longStr.size());
    EXPECT_TRUE(w.ok());

    BufReader r(buf.data(), w.pos);
    uint16_t len = r.getU16();
    EXPECT_EQ(len, 500u);
    const uint8_t* data = r.getBytes(len);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(data), len), longStr);
}

// ===========================================================================
// Constants
// ===========================================================================

TEST(IOProtocolExtra, ProtocolVersionAndPort) {
    EXPECT_EQ(PROTOCOL_VERSION, 1);
    EXPECT_EQ(DEFAULT_PORT, 4000);
}
