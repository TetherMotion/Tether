/**
 * @file test_klipper_protocol_ext.cpp
 * @brief Extended wire protocol tests: CRC edge cases, VLQ all ranges, block edge cases,
 *        constants, parameter format parse/encode/decode.
 */

#include <gtest/gtest.h>
#include "tether/klipper/protocol/Crc16.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/protocol/ParameterFormat.hpp"

#include <vector>
#include <cstring>
#include <random>

using namespace tether::klipper::protocol;

// ============================================================================
// CRC-16 edge cases
// ============================================================================

TEST(KlipperCrc16Ext, EmptyBuffer) {
    std::vector<uint8_t> empty;
    EXPECT_EQ(crc16Ccitt(empty), 0xFFFF);
}

TEST(KlipperCrc16Ext, SingleByteZero) {
    std::vector<uint8_t> data = {0x00};
    auto crc = crc16Ccitt(data);
    EXPECT_NE(crc, 0xFFFF); // Should change
}

TEST(KlipperCrc16Ext, SingleByteFF) {
    std::vector<uint8_t> data = {0xFF};
    auto crc = crc16Ccitt(data);
    EXPECT_NE(crc, 0xFFFF);
}

TEST(KlipperCrc16Ext, MaxContentLength) {
    std::vector<uint8_t> data(kMaxContentLength, 0xAA);
    auto crc = crc16Ccitt(data);
    EXPECT_NE(crc, 0);
}

TEST(KlipperCrc16Ext, PointerVsSpan) {
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    auto crc1 = crc16Ccitt(data.data(), data.size());
    auto crc2 = crc16Ccitt(data);
    EXPECT_EQ(crc1, crc2);
}

TEST(KlipperCrc16Ext, OrderMatters) {
    std::vector<uint8_t> d1 = {0x01, 0x02, 0x03};
    std::vector<uint8_t> d2 = {0x03, 0x02, 0x01};
    EXPECT_NE(crc16Ccitt(d1), crc16Ccitt(d2));
}

TEST(KlipperCrc16Ext, Deterministic) {
    std::vector<uint8_t> data(32);
    for (size_t i = 0; i < 32; ++i) data[i] = static_cast<uint8_t>(i);
    auto crc1 = crc16Ccitt(data);
    auto crc2 = crc16Ccitt(data);
    EXPECT_EQ(crc1, crc2);
}

TEST(KlipperCrc16Ext, RandomDataStable) {
    std::mt19937 rng(42);
    std::vector<uint8_t> data(100);
    for (auto& b : data) b = static_cast<uint8_t>(rng());
    auto crc1 = crc16Ccitt(data);
    auto crc2 = crc16Ccitt(data);
    EXPECT_EQ(crc1, crc2);
}

// ============================================================================
// VLQ encoding: all ranges
// ============================================================================

TEST(KlipperVlqExt, ParamOneByteRange) {
    // 1 byte: -32 .. 95
    for (int32_t v = -32; v <= 95; ++v) {
        uint8_t buf[8];
        size_t len = encodeParam(v, buf);
        ASSERT_EQ(len, 1u) << "value " << v << " should encode in 1 byte";
        const uint8_t* p = buf;
        auto decoded = decodeParam(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, ParamTwoByteRange) {
    // 2 bytes: -4096 .. 12287 (excluding 1-byte range)
    for (int32_t v : {-4096, -100, -33, 96, 1000, 12287}) {
        uint8_t buf[8];
        size_t len = encodeParam(v, buf);
        ASSERT_EQ(len, 2u) << "value " << v << " should encode in 2 bytes";
        const uint8_t* p = buf;
        auto decoded = decodeParam(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, ParamThreeByteRange) {
    for (int32_t v : {-524288, -100000, 12288, 100000, 1572863}) {
        uint8_t buf[8];
        size_t len = encodeParam(v, buf);
        ASSERT_EQ(len, 3u) << "value " << v << " should encode in 3 bytes";
        const uint8_t* p = buf;
        auto decoded = decodeParam(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, ParamFourByteRange) {
    for (int32_t v : {-67108864, -10000000, 1572864, 10000000, 201326591}) {
        uint8_t buf[8];
        size_t len = encodeParam(v, buf);
        ASSERT_EQ(len, 4u) << "value " << v << " should encode in 4 bytes";
        const uint8_t* p = buf;
        auto decoded = decodeParam(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, ParamFiveByteRange) {
    for (int32_t v : {-2000000000, -67108865, 201326592, 2000000000}) {
        uint8_t buf[8];
        size_t len = encodeParam(v, buf);
        ASSERT_EQ(len, 5u) << "value " << v << " should encode in 5 bytes";
        const uint8_t* p = buf;
        auto decoded = decodeParam(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, MsgIdOneByteRange) {
    for (uint16_t v = 0; v < 128; ++v) {
        uint8_t buf[4];
        size_t len = encodeMsgId(v, buf);
        ASSERT_EQ(len, 1u) << "msgid " << v << " should encode in 1 byte";
        const uint8_t* p = buf;
        auto decoded = decodeMsgId(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, MsgIdTwoByteRange) {
    for (uint16_t v : {128u, 255u, 511u, 1000u, 16383u}) {
        uint8_t buf[4];
        size_t len = encodeMsgId(v, buf);
        ASSERT_EQ(len, 2u) << "msgid " << v << " should encode in 2 bytes";
        const uint8_t* p = buf;
        auto decoded = decodeMsgId(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, DecodeParamTruncated) {
    uint8_t buf[1] = {0x80}; // Start of multi-byte, but no continuation
    const uint8_t* p = buf;
    auto decoded = decodeParam(p, buf + 1);
    EXPECT_FALSE(decoded.has_value());
}

TEST(KlipperVlqExt, DecodeMsgIdTruncated) {
    uint8_t buf[1] = {0x80}; // Start of 2-byte msgid, but no continuation
    const uint8_t* p = buf;
    auto decoded = decodeMsgId(p, buf + 1);
    EXPECT_FALSE(decoded.has_value());
}

TEST(KlipperVlqExt, DecodeParamUnsigned) {
    for (uint32_t v : {0u, 1u, 100u, 10000u, 1000000u}) {
        uint8_t buf[8];
        size_t len = encodeParam(static_cast<int32_t>(v), buf);
        const uint8_t* p = buf;
        auto decoded = decodeParamUnsigned(p, buf + len);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlqExt, MultipleParamsInBuffer) {
    // Encode multiple params sequentially
    std::vector<uint8_t> buf;
    uint8_t tmp[8];
    size_t len;

    len = encodeParam(42, tmp); buf.insert(buf.end(), tmp, tmp + len);
    len = encodeParam(-100, tmp); buf.insert(buf.end(), tmp, tmp + len);
    len = encodeParam(100000, tmp); buf.insert(buf.end(), tmp, tmp + len);

    const uint8_t* p = buf.data();
    const uint8_t* end = p + buf.size();

    auto d1 = decodeParam(p, end);
    ASSERT_TRUE(d1.has_value());
    EXPECT_EQ(*d1, 42);

    auto d2 = decodeParam(p, end);
    ASSERT_TRUE(d2.has_value());
    EXPECT_EQ(*d2, -100);

    auto d3 = decodeParam(p, end);
    ASSERT_TRUE(d3.has_value());
    EXPECT_EQ(*d3, 100000);

    EXPECT_EQ(p, end); // All bytes consumed
}

// ============================================================================
// Message block build/parse edge cases
// ============================================================================

TEST(KlipperMessageBlockExt, EmptyContent) {
    auto blk = buildBlockVec(0, {});
    ASSERT_FALSE(blk.empty());
    EXPECT_EQ(blk.size(), kMinBlockLength);
    auto pb = parseBlock(blk);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 0u);
    EXPECT_TRUE(pb.block.content.empty());
}

TEST(KlipperMessageBlockExt, MaxContentLength) {
    std::vector<uint8_t> content(kMaxContentLength, 0x42);
    auto blk = buildBlockVec(5, content);
    ASSERT_FALSE(blk.empty());
    EXPECT_EQ(blk.size(), kMaxBlockLength);
    auto pb = parseBlock(blk);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 5u);
    EXPECT_EQ(pb.block.content, content);
}

TEST(KlipperMessageBlockExt, ContentTooLarge) {
    std::vector<uint8_t> content(kMaxContentLength + 1, 0x42);
    uint8_t out[128];
    size_t n = buildBlock(0, content, out);
    EXPECT_EQ(n, 0u); // Should fail
}

TEST(KlipperMessageBlockExt, InvalidSequence) {
    uint8_t out[128];
    std::vector<uint8_t> content = {0x01};
    size_t n = buildBlock(16, content, out); // seq > 15
    EXPECT_EQ(n, 0u);
}

TEST(KlipperMessageBlockExt, AllSequenceValues) {
    for (uint8_t seq = 0; seq <= 15; ++seq) {
        std::vector<uint8_t> content = {static_cast<uint8_t>(seq)};
        auto blk = buildBlockVec(seq, content);
        auto pb = parseBlock(blk);
        EXPECT_EQ(pb.status, BlockParseStatus::Ok);
        EXPECT_EQ(pb.block.sequence, seq);
    }
}

TEST(KlipperMessageBlockExt, BadSyncByte) {
    std::vector<uint8_t> content = {0x01, 0x02};
    auto blk = buildBlockVec(0, content);
    // Corrupt the sync byte (last byte)
    blk[blk.size() - 1] = 0x00;
    auto pb = parseBlock(blk);
    // The parser will skip the bad block looking for sync
    EXPECT_NE(pb.status, BlockParseStatus::Ok);
}

TEST(KlipperMessageBlockExt, BadLength) {
    // Create a block with a valid-looking but out-of-range length
    std::vector<uint8_t> bad = {0x04, 0x10, 0x00, 0x00, 0x7E}; // len=4 < min
    auto pb = parseBlock(bad);
    // Should skip or need more data
    EXPECT_NE(pb.status, BlockParseStatus::Ok);
}

TEST(KlipperMessageBlockExt, MultipleBlocksInBuffer) {
    auto blk1 = buildBlockVec(1, std::vector<uint8_t>{0x01});
    auto blk2 = buildBlockVec(2, std::vector<uint8_t>{0x02, 0x03});

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), blk1.begin(), blk1.end());
    combined.insert(combined.end(), blk2.begin(), blk2.end());

    auto pb = parseBlock(combined);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 1u);
    EXPECT_EQ(pb.block.content, std::vector<uint8_t>({0x01}));

    // Parse second block
    std::span<const uint8_t> remaining(combined.data() + pb.consumedBytes,
                                        combined.size() - pb.consumedBytes);
    auto pb2 = parseBlock(remaining);
    EXPECT_EQ(pb2.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb2.block.sequence, 2u);
    EXPECT_EQ(pb2.block.content, std::vector<uint8_t>({0x02, 0x03}));
}

TEST(KlipperMessageBlockExt, GarbageBeforeBlock) {
    auto blk = buildBlockVec(3, std::vector<uint8_t>{0xAA, 0xBB});
    std::vector<uint8_t> combined = {0xFF, 0xFE, 0xFD}; // garbage
    combined.insert(combined.end(), blk.begin(), blk.end());

    auto pb = parseBlock(combined);
    // Parser should skip garbage and find the block
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 3u);
    EXPECT_EQ(pb.block.content, std::vector<uint8_t>({0xAA, 0xBB}));
}

TEST(KlipperMessageBlockExt, WireLength) {
    MessageBlock mb;
    mb.sequence = 0;
    mb.content = {0x01, 0x02, 0x03};
    EXPECT_EQ(mb.wireLength(), kHeaderSize + 3 + kTrailerSize);
}

TEST(KlipperMessageBlockExt, AckBlockMinSize) {
    auto ack = buildAckBlock(0);
    EXPECT_EQ(ack.size(), kMinBlockLength);
}

// ============================================================================
// Constants
// ============================================================================

TEST(KlipperConstantsExt, BlockSizes) {
    EXPECT_EQ(kMinBlockLength, 5);
    EXPECT_EQ(kMaxBlockLength, 64);
    EXPECT_EQ(kHeaderSize, 2);
    EXPECT_EQ(kTrailerSize, 3);
    EXPECT_EQ(kMaxContentLength, 59);
}

TEST(KlipperConstantsExt, SyncByte) {
    EXPECT_EQ(kSyncByte, 0x7E);
}

TEST(KlipperConstantsExt, SequenceMask) {
    EXPECT_EQ(kSequenceMask, 0x0F);
    EXPECT_EQ(kSequenceDestMarker, 0x10);
}

TEST(KlipperConstantsExt, MsgIds) {
    EXPECT_EQ(kMsgIdIdentifyResponse, 0);
    EXPECT_EQ(kMsgIdIdentify, 1);
    EXPECT_EQ(kFirstDynamicMsgId, 2);
}

TEST(KlipperConstantsExt, RtoDefaults) {
    EXPECT_GT(kMinRtoSeconds, 0);
    EXPECT_LT(kMinRtoSeconds, kMaxRtoSeconds);
}

// ============================================================================
// Parameter format: parse, encode, decode
// ============================================================================

TEST(KlipperParamFormatExt, ParseSimpleFormat) {
    auto fmt = parseFormatString("get_clock");
    ASSERT_TRUE(fmt.has_value());
    EXPECT_EQ(fmt->params.size(), 0u);
}

TEST(KlipperParamFormatExt, ParseWithUintParam) {
    auto fmt = parseFormatString("set_digital_out oid=%c value=%c");
    ASSERT_TRUE(fmt.has_value());
    EXPECT_EQ(fmt->params.size(), 2u);
    EXPECT_EQ(fmt->params[0].name, "oid");
    EXPECT_EQ(fmt->params[0].type, ParamType::Byte);
    EXPECT_EQ(fmt->params[1].name, "value");
    EXPECT_EQ(fmt->params[1].type, ParamType::Byte);
}

TEST(KlipperParamFormatExt, ParseWithUint32Param) {
    auto fmt = parseFormatString("queue_step oid=%c interval=%u count=%hu add=%hi");
    ASSERT_TRUE(fmt.has_value());
    EXPECT_EQ(fmt->params.size(), 4u);
    EXPECT_EQ(fmt->params[0].type, ParamType::Byte);
    EXPECT_EQ(fmt->params[1].type, ParamType::Uint32);
    EXPECT_EQ(fmt->params[2].type, ParamType::Uint16);
    EXPECT_EQ(fmt->params[3].type, ParamType::Int16);
}

TEST(KlipperParamFormatExt, ParseWithStringParam) {
    auto fmt = parseFormatString("config_stepper oid=%c step_pin=%s");
    ASSERT_TRUE(fmt.has_value());
    EXPECT_EQ(fmt->params.size(), 2u);
    EXPECT_EQ(fmt->params[1].type, ParamType::String);
}

TEST(KlipperParamFormatExt, ParseInvalidFormat) {
    auto fmt = parseFormatString("");
    EXPECT_FALSE(fmt.has_value());
}

TEST(KlipperParamFormatExt, Arity) {
    auto fmt = parseFormatString("test a=%u b=%i c=%c");
    ASSERT_TRUE(fmt.has_value());
    EXPECT_EQ(fmt->arity(), 3u);
}

TEST(KlipperParamFormatExt, IsIntegerType) {
    EXPECT_TRUE(isIntegerType(ParamType::Uint32));
    EXPECT_TRUE(isIntegerType(ParamType::Int32));
    EXPECT_TRUE(isIntegerType(ParamType::Uint16));
    EXPECT_TRUE(isIntegerType(ParamType::Int16));
    EXPECT_TRUE(isIntegerType(ParamType::Byte));
    EXPECT_FALSE(isIntegerType(ParamType::String));
    EXPECT_FALSE(isIntegerType(ParamType::Buffer));
}

TEST(KlipperParamFormatExt, EncodeDecodeIntegerParam) {
    uint8_t out[8];
    size_t len = encodeParamValue(ParamType::Uint32, 12345, {}, out);
    ASSERT_GT(len, 0u);

    const uint8_t* p = out;
    int32_t intVal;
    std::vector<uint8_t> str;
    bool ok = decodeParamValue(ParamType::Uint32, p, out + len, intVal, str);
    EXPECT_TRUE(ok);
    EXPECT_EQ(intVal, 12345);
}

TEST(KlipperParamFormatExt, EncodeDecodeStringParam) {
    uint8_t out[64];
    std::vector<uint8_t> strData = {'H', 'e', 'l', 'l', 'o'};
    size_t len = encodeParamValue(ParamType::String, 0, strData, out);
    ASSERT_GT(len, 0u);

    const uint8_t* p = out;
    int32_t intVal;
    std::vector<uint8_t> decodedStr;
    bool ok = decodeParamValue(ParamType::String, p, out + len, intVal, decodedStr);
    EXPECT_TRUE(ok);
    EXPECT_EQ(decodedStr, strData);
}

TEST(KlipperParamFormatExt, FormatSpecToString) {
    auto fmt = parseFormatString("test val=%u");
    ASSERT_TRUE(fmt.has_value());
    auto str = fmt->toString();
    EXPECT_FALSE(str.empty());
}
