// GoogleTest-based unit tests for SLIPStream buffer API
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include "SLIPStream/Buffer.hpp"

using namespace SLIPStream;

TEST(SLIPBufferEncodeDecode, SimpleEncode) {
    const uint8_t in1[] = {0x01, 0x02, 0x03};
    size_t elen1 = encoded_length(in1, sizeof(in1));
    EXPECT_EQ(elen1, sizeof(in1) + 1);
    std::vector<uint8_t> out1(elen1);
    size_t r1 = encode_packet(in1, sizeof(in1), out1.data(), out1.size());
    EXPECT_EQ(r1, elen1);
    EXPECT_EQ(out1[0], 0x01);
    EXPECT_EQ(out1[1], 0x02);
    EXPECT_EQ(out1[2], 0x03);
    EXPECT_EQ(out1[3], END);
}

TEST(SLIPBufferEncodeDecode, EncodeWithEscapes) {
    const uint8_t in2[] = {END, ESC, 0x55};
    size_t elen2 = encoded_length(in2, sizeof(in2));
    EXPECT_EQ(elen2, 6u);
    std::vector<uint8_t> out2(elen2);
    size_t r2 = encode_packet(in2, sizeof(in2), out2.data(), out2.size());
    EXPECT_EQ(r2, elen2);
    const uint8_t expected2[] = {ESC, ESCEND, ESC, ESCESC, 0x55, END};
    EXPECT_EQ(0, std::memcmp(out2.data(), expected2, elen2));

    size_t dlen = decoded_length(out2.data(), out2.size());
    EXPECT_EQ(dlen, sizeof(in2));
    std::vector<uint8_t> dec(dlen);
    size_t dwritten = decode_packet(out2.data(), out2.size(), dec.data(), dec.size());
    EXPECT_EQ(dwritten, dlen);
    EXPECT_EQ(0, std::memcmp(dec.data(), in2, dlen));
}

TEST(SLIPBufferDecode, NoEndError) {
    const uint8_t noend[] = {0x01, 0x02, 0x03};
    size_t badlen = decoded_length(noend, sizeof(noend));
    EXPECT_EQ(badlen, DECODE_ERROR);
}

TEST(SLIPBufferDecode, MalformedEscape) {
    const uint8_t malformed[] = {ESC, 0x00, END};
    size_t mal = decoded_length(malformed, sizeof(malformed));
    EXPECT_EQ(mal, DECODE_ERROR);
}

TEST(SLIPBufferDecode, OutputTooSmall) {
    const uint8_t in2[] = {END, ESC, 0x55};
    size_t elen2 = encoded_length(in2, sizeof(in2));
    std::vector<uint8_t> out2(elen2);
    encode_packet(in2, sizeof(in2), out2.data(), out2.size());
    size_t dlen = decoded_length(out2.data(), out2.size());
    ASSERT_NE(dlen, DECODE_ERROR);
    std::vector<uint8_t> small_out(dlen - 1);
    size_t too_small = decode_packet(out2.data(), out2.size(), small_out.data(), small_out.size());
    EXPECT_EQ(too_small, DECODE_ERROR);
}

// main moved to test_all_main.cpp
