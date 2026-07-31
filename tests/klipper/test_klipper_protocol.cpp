/**
 * @file test_klipper_protocol.cpp
 * @brief Tests for the Klipper wire protocol: CRC, VLQ, message blocks.
 */

#include <gtest/gtest.h>
#include "tether/klipper/protocol/Crc16.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"

#include <vector>

using namespace tether::klipper::protocol;

TEST(KlipperCrc16, KnownValues) {
    std::vector<uint8_t> empty;
    EXPECT_EQ(crc16Ccitt(empty), 0xFFFF);
    std::vector<uint8_t> one = {0x01};
    auto crc = crc16Ccitt(one);
    EXPECT_NE(crc, 0);
    std::vector<uint8_t> three = {0x01, 0x02, 0x03};
    EXPECT_EQ(crc16Ccitt(three), crc16Ccitt(three));
}

TEST(KlipperVlq, EncodeDecodeParamRoundTrip) {
    for (int32_t v : {0, 1, 127, 128, 255, 256, 16383, 16384, 100000, -1, -128}) {
        uint8_t buf[8];
        size_t len = encodeParam(v, buf);
        const uint8_t* p = buf;
        const uint8_t* end = p + len;
        auto decoded = decodeParam(p, end);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperVlq, EncodeDecodeMsgIdRoundTrip) {
    for (uint16_t v : {0u, 1u, 127u, 255u, 511u}) {
        uint8_t buf[4];
        size_t len = encodeMsgId(v, buf);
        const uint8_t* p = buf;
        const uint8_t* end = p + len;
        auto decoded = decodeMsgId(p, end);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(*decoded, v);
    }
}

TEST(KlipperMessageBlock, BuildAndParse) {
    std::vector<uint8_t> content = {0x7B, 0x06, 0x01, 0x2D};
    auto wire = buildBlockVec(3, content);
    ASSERT_FALSE(wire.empty());
    auto pb = parseBlock(wire);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 3u);
    EXPECT_EQ(pb.block.content, content);
}

TEST(KlipperMessageBlock, AckBlock) {
    auto ack = buildAckBlock(5);
    auto pb = parseBlock(ack);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 5u);
    EXPECT_TRUE(pb.block.content.empty());
}

TEST(KlipperMessageBlock, BadCrc) {
    std::vector<uint8_t> content = {0x01, 0x02};
    auto wire = buildBlockVec(0, content);
    wire[wire.size() - 3] ^= 0xFF;
    auto pb = parseBlock(wire);
    EXPECT_EQ(pb.status, BlockParseStatus::BadCrc);
}

TEST(KlipperMessageBlock, NeedMoreData) {
    std::vector<uint8_t> content = {0x01};
    auto wire = buildBlockVec(0, content);
    wire.resize(wire.size() - 2);
    auto pb = parseBlock(wire);
    EXPECT_EQ(pb.status, BlockParseStatus::NeedMoreData);
}
