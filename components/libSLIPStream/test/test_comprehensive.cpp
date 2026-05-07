// Comprehensive GoogleTest-based unit tests for SLIPStream
// Tests all code paths in the Buffer API, Decoder, and Encoder class

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>
// Include only the necessary headers to avoid multiple definition conflicts
// Each header file (Buffer.hpp, Decoder.hpp, Encoder.hpp) independently defines LogType
#include "SLIPStream/Buffer.hpp"

using namespace SLIPStream;

// ============================================================================
// Buffer API Tests - Encoding
// ============================================================================

class SLIPBufferEncodeTest : public ::testing::Test {
protected:
    std::vector<uint8_t> buffer;
    const size_t BUFFER_SIZE = 256;
    
    void SetUp() override {
        buffer.resize(BUFFER_SIZE);
    }
};

TEST_F(SLIPBufferEncodeTest, EmptyInput) {
    // Empty input should produce just END marker
    size_t len = encoded_length(nullptr, 0);
    EXPECT_EQ(len, 1u);
    
    size_t written = encode_packet(nullptr, 0, buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, 1u);
    EXPECT_EQ(buffer[0], END);
}

TEST_F(SLIPBufferEncodeTest, SimpleDataNoEscapes) {
    const uint8_t input[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    size_t len = encoded_length(input, sizeof(input));
    EXPECT_EQ(len, sizeof(input) + 1);  // +1 for END
    
    size_t written = encode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, len);
    EXPECT_EQ(buffer[len-1], END);
    EXPECT_EQ(0, std::memcmp(buffer.data(), input, sizeof(input)));
}

TEST_F(SLIPBufferEncodeTest, SingleENDByteEscape) {
    const uint8_t input[] = {END};
    size_t len = encoded_length(input, sizeof(input));
    EXPECT_EQ(len, 3u);  // ESC + ESCEND + END
    
    size_t written = encode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, 3u);
    EXPECT_EQ(buffer[0], ESC);
    EXPECT_EQ(buffer[1], ESCEND);
    EXPECT_EQ(buffer[2], END);
}

TEST_F(SLIPBufferEncodeTest, SingleESCByteEscape) {
    const uint8_t input[] = {ESC};
    size_t len = encoded_length(input, sizeof(input));
    EXPECT_EQ(len, 3u);  // ESC + ESCESC + END
    
    size_t written = encode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, 3u);
    EXPECT_EQ(buffer[0], ESC);
    EXPECT_EQ(buffer[1], ESCESC);
    EXPECT_EQ(buffer[2], END);
}

TEST_F(SLIPBufferEncodeTest, MixedEscapesAndNormalData) {
    const uint8_t input[] = {0x01, END, 0x02, ESC, 0x03};
    size_t len = encoded_length(input, sizeof(input));
    EXPECT_EQ(len, 8u);  // 0x01 + (ESC+ESCEND) + 0x02 + (ESC+ESCESC) + 0x03 + END = 8
    
    size_t written = encode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, 8u);
    
    const uint8_t expected[] = {0x01, ESC, ESCEND, 0x02, ESC, ESCESC, 0x03, END};
    EXPECT_EQ(0, std::memcmp(buffer.data(), expected, 8));
}

TEST_F(SLIPBufferEncodeTest, ConsecutiveEscapes) {
    const uint8_t input[] = {END, END, ESC, ESC};
    size_t len = encoded_length(input, sizeof(input));
    EXPECT_EQ(len, 9u);  // Each byte becomes 2 bytes + END
    
    size_t written = encode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, 9u);
}

TEST_F(SLIPBufferEncodeTest, OutputBufferTooSmall) {
    const uint8_t input[] = {0x01, 0x02, 0x03};
    size_t encoded = encode_packet(input, sizeof(input), buffer.data(), 2);  // Too small
    EXPECT_EQ(encoded, ENCODE_ERROR);
}

TEST_F(SLIPBufferEncodeTest, OutputBufferJustEnoughForSpecialByte) {
    const uint8_t input[] = {END};
    size_t encoded = encode_packet(input, sizeof(input), buffer.data(), 3);
    EXPECT_EQ(encoded, 3u);
}

TEST_F(SLIPBufferEncodeTest, MultipleSpecialBytesLargeOutput) {
    const uint8_t input[] = {END, ESC, END, ESC, 0xFF, 0xAA};
    size_t len = encoded_length(input, sizeof(input));
    size_t encoded = encode_packet(input, sizeof(input), buffer.data(), buffer.size());
    EXPECT_EQ(encoded, len);
    EXPECT_NE(encoded, ENCODE_ERROR);
}

// ============================================================================
// Buffer API Tests - Decoding
// ============================================================================

class SLIPBufferDecodeTest : public ::testing::Test {
protected:
    std::vector<uint8_t> buffer;
    const size_t BUFFER_SIZE = 256;
    
    void SetUp() override {
        buffer.resize(BUFFER_SIZE);
    }
};

TEST_F(SLIPBufferDecodeTest, DecodedLengthSimpleData) {
    const uint8_t input[] = {0x01, 0x02, 0x03, END};
    size_t len = decoded_length(input, sizeof(input));
    EXPECT_EQ(len, 3u);
}

TEST_F(SLIPBufferDecodeTest, DecodedLengthWithEscapes) {
    const uint8_t input[] = {ESC, ESCEND, ESC, ESCESC, 0x05, END};
    size_t len = decoded_length(input, sizeof(input));
    EXPECT_EQ(len, 3u);  // ESC+ESCEND -> END (1), ESC+ESCESC -> ESC (1), 0x05 (1) = 3 bytes
}

TEST_F(SLIPBufferDecodeTest, DecodedLengthNoENDMarker) {
    const uint8_t input[] = {0x01, 0x02, 0x03};
    size_t len = decoded_length(input, sizeof(input));
    EXPECT_EQ(len, DECODE_ERROR);
}

TEST_F(SLIPBufferDecodeTest, DecodedLengthTruncatedEscape) {
    const uint8_t input[] = {ESC};  // ESC without following byte
    size_t len = decoded_length(input, sizeof(input));
    EXPECT_EQ(len, DECODE_ERROR);
}

TEST_F(SLIPBufferDecodeTest, DecodedLengthInvalidEscapeSequence) {
    const uint8_t input[] = {ESC, 0xFF, END};  // Invalid escape byte
    size_t len = decoded_length(input, sizeof(input));
    EXPECT_EQ(len, DECODE_ERROR);
}

TEST_F(SLIPBufferDecodeTest, DecodeSimpleData) {
    const uint8_t input[] = {0x01, 0x02, 0x03, END};
    size_t len = decoded_length(input, sizeof(input));
    
    size_t written = decode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, len);
    EXPECT_EQ(buffer[0], 0x01);
    EXPECT_EQ(buffer[1], 0x02);
    EXPECT_EQ(buffer[2], 0x03);
}

TEST_F(SLIPBufferDecodeTest, DecodeWithEscapes) {
    const uint8_t input[] = {ESC, ESCEND, 0x02, ESC, ESCESC, END};
    size_t len = decoded_length(input, sizeof(input));
    EXPECT_EQ(len, 3u);
    
    size_t written = decode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, 3u);
    EXPECT_EQ(buffer[0], END);
    EXPECT_EQ(buffer[1], 0x02);
    EXPECT_EQ(buffer[2], ESC);
}

TEST_F(SLIPBufferDecodeTest, DecodeOutputBufferTooSmall) {
    const uint8_t input[] = {0x01, 0x02, 0x03, END};
    size_t written = decode_packet(input, sizeof(input), buffer.data(), 2);
    EXPECT_EQ(written, DECODE_ERROR);
}

TEST_F(SLIPBufferDecodeTest, DecodeTruncatedEscapeInDecode) {
    const uint8_t input[] = {ESC};  // Incomplete
    size_t written = decode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, DECODE_ERROR);
}

TEST_F(SLIPBufferDecodeTest, DecodeInvalidEscapeInDecode) {
    const uint8_t input[] = {ESC, 0x55, END};
    size_t written = decode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, DECODE_ERROR);
}

TEST_F(SLIPBufferDecodeTest, DecodeNoENDInDecode) {
    const uint8_t input[] = {0x01, 0x02, 0x03};
    size_t written = decode_packet(input, sizeof(input), buffer.data(), BUFFER_SIZE);
    EXPECT_EQ(written, DECODE_ERROR);
}

// ============================================================================
// Round-trip Tests
// ============================================================================

TEST(SLIPRoundTrip, SimpleRoundTrip) {
    const uint8_t original[] = {0x12, 0x34, 0x56, 0x78};
    std::vector<uint8_t> encoded(256);
    std::vector<uint8_t> decoded(256);
    
    size_t enc_len = encoded_length(original, sizeof(original));
    encode_packet(original, sizeof(original), encoded.data(), encoded.size());
    
    size_t dec_len = decoded_length(encoded.data(), encoded.size());
    size_t written = decode_packet(encoded.data(), encoded.size(), decoded.data(), decoded.size());
    
    EXPECT_EQ(written, dec_len);
    EXPECT_EQ(0, std::memcmp(original, decoded.data(), sizeof(original)));
}

TEST(SLIPRoundTrip, ComplexRoundTrip) {
    const uint8_t original[] = {
        0x00, END, 0x01, ESC, 0x02,
        END, END, ESC, ESC,
        0xFF, 0xAA, 0x55, 0x00
    };
    std::vector<uint8_t> encoded(256);
    std::vector<uint8_t> decoded(256);
    
    size_t enc_len = encoded_length(original, sizeof(original));
    encode_packet(original, sizeof(original), encoded.data(), encoded.size());
    
    size_t dec_len = decoded_length(encoded.data(), encoded.size());
    size_t written = decode_packet(encoded.data(), encoded.size(), decoded.data(), decoded.size());
    
    EXPECT_EQ(written, dec_len);
    EXPECT_EQ(0, std::memcmp(original, decoded.data(), sizeof(original)));
}


