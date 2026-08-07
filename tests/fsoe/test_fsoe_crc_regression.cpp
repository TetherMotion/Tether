/**
 * @file test_fsoe_crc_regression.cpp
 * @brief Regression tests for CRC-16 and frame format fixes (commits
 *        1b481e4, 94e4370, 11210d2).
 *
 * Covers:
 * - CRC-16 table uses correct non-reflected polynomial 0x39B7 (1b481e4)
 * - Odd-length safety data frame format per ETG.5100 (94e4370)
 * - parseFSoEFrame rejects payloads exceeding MAX_PARSE_DATA_SIZE (11210d2)
 * - Frame size calculations for even, odd, and zero data lengths
 * - Round-trip build/parse integrity for all payload sizes 0..18
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include "fsoe/FSoECRC.hpp"
#include "fsoe/FSoEDefs.hpp"

using namespace FSoE;

// ============================================================================
// CRC-16 Polynomial Correctness (commit 1b481e4)
// ============================================================================
//
// The old table used a reflected polynomial (0x22CF entries) which produced
// incorrect CRCs. The fix uses the non-reflected polynomial 0x39B7 per
// ETG.5100 §8.1.3.2 (refIn=false, refOut=false, init=0x0000).
//

TEST(FSoECRCRegression, PolynomialConstant) {
    EXPECT_EQ(CRC::kPolynomial, 0x39B7);
    EXPECT_EQ(CRC::kInitValue, 0x0000);
}

TEST(FSoECRCRegression, TableEntryOne) {
    // crcTable[1] should be the polynomial itself (CRC of single 0x01 byte)
    // For non-reflected CRC with poly 0x39B7:
    //   init=0x0000, byte=0x01: (0x00 << 8) ^ table[(0x00 >> 8) ^ 0x01] = table[1]
    //   table[1] = 0x39B7
    EXPECT_EQ(CRC::crcTable[1], 0x39B7);
}

TEST(FSoECRCRegression, TableEntryTwo) {
    // crcTable[2] = (0x39B7 << 8) ^ table[(0x39B7 >> 8) ^ 0x02]
    //             = 0xB700 ^ table[0x39 ^ 0x02] = 0xB700 ^ table[0x3B]
    // table[0x3B] = 0x6556 → 0xB700 ^ 0x6556 = 0xD256
    // But let's just verify it's NOT the old reflected value (0x36F1)
    EXPECT_NE(CRC::crcTable[2], 0x36F1);
    // The correct non-reflected value:
    EXPECT_EQ(CRC::crcTable[2], 0x736E);
}

TEST(FSoECRCRegression, CalculateSingleByte) {
    // CRC of [0x00] should be 0x0000 (init XOR table[0])
    uint8_t zero = 0x00;
    EXPECT_EQ(CRC::calculate(&zero, 1), 0x0000);

    // CRC of [0x01] should be 0x39B7 (the polynomial)
    uint8_t one = 0x01;
    EXPECT_EQ(CRC::calculate(&one, 1), 0x39B7);
}

TEST(FSoECRCRegression, CalculateDeterministic) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint16_t crc1 = CRC::calculate(data, sizeof(data));
    uint16_t crc2 = CRC::calculate(data, sizeof(data));
    EXPECT_EQ(crc1, crc2);
}

TEST(FSoECRCRegression, CalculateNonZeroForNonZeroData) {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint16_t crc = CRC::calculate(data, sizeof(data));
    EXPECT_NE(crc, 0x0000);
}

TEST(FSoECRCRegression, CalculateEmpty) {
    EXPECT_EQ(CRC::calculate(nullptr, 0), CRC::kInitValue);
    EXPECT_EQ(CRC::calculate(nullptr, 0, 0xABCD), 0xABCD);
}

TEST(FSoECRCRegression, CalculateChainedCRC) {
    // CRC of [A B C D] should equal CRC of CRC([A B]) + [C D]
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t full = CRC::calculate(data, 4);
    uint16_t partial = CRC::calculate(data, 2);
    uint16_t chained = CRC::calculate(data + 2, 2, partial);
    EXPECT_EQ(full, chained);
}

TEST(FSoECRCRegression, OldReflectedTableDiffers) {
    // The first few entries of the old reflected table were:
    //   0x0000, 0x22CF, 0x36F1, 0x143E, ...
    // The new non-reflected table should differ:
    EXPECT_NE(CRC::crcTable[1], 0x22CF);
    EXPECT_NE(CRC::crcTable[3], 0x143E);
    EXPECT_NE(CRC::crcTable[4], 0x1E8D);
}

// ============================================================================
// Frame Size Calculations (commit 94e4370)
// ============================================================================
//
// ETG.5100 frame layout:
//   [CMD(1)] [Data0(2B)][CRC0(2B)] ... [DataN(1-2B)][CRCN(2B)] [ConnID(2B)]
// Even data: N full chunks of 4 bytes each
// Odd data:  (N-1) full chunks + 1 odd chunk of 3 bytes (1 data + 2 CRC)
//

TEST(FSoEFrameSizeRegression, ZeroData) {
    EXPECT_EQ(CRC::fsoeFrameSize(0), 3u);  // CMD + ConnID
}

TEST(FSoEFrameSizeRegression, EvenData) {
    EXPECT_EQ(CRC::fsoeFrameSize(2), 7u);   // CMD + 1 chunk(4) + ConnID
    EXPECT_EQ(CRC::fsoeFrameSize(4), 11u);  // CMD + 2 chunks(8) + ConnID
    EXPECT_EQ(CRC::fsoeFrameSize(16), 35u); // CMD + 8 chunks(32) + ConnID
}

TEST(FSoEFrameSizeRegression, OddData) {
    // Odd data: last chunk is 1 data byte + 2 CRC = 3 bytes (not 4)
    EXPECT_EQ(CRC::fsoeFrameSize(1), 6u);   // CMD + 1 odd chunk(3) + ConnID
    EXPECT_EQ(CRC::fsoeFrameSize(3), 10u);  // CMD + 1 full(4) + 1 odd(3) + ConnID
    EXPECT_EQ(CRC::fsoeFrameSize(5), 14u);  // CMD + 2 full(8) + 1 odd(3) + ConnID
    EXPECT_EQ(CRC::fsoeFrameSize(15), 34u); // CMD + 7 full(28) + 1 odd(3) + ConnID
}

TEST(FSoEFrameSizeRegression, DataLenFromFrameSize) {
    // Round-trip: fsoeDataLen(fsoeFrameSize(n)) == n
    for (size_t n = 0; n <= 18; ++n) {
        size_t frame_size = CRC::fsoeFrameSize(n);
        size_t data_len = CRC::fsoeDataLen(frame_size);
        EXPECT_EQ(data_len, n) << "Failed for data_len=" << n;
    }
}

TEST(FSoEFrameSizeRegression, InvalidFrameSizes) {
    // Frame sizes with remainder 1 or 2 are invalid
    EXPECT_EQ(CRC::fsoeDataLen(4), 0u);  // remaining=1, invalid
    EXPECT_EQ(CRC::fsoeDataLen(5), 0u);  // remaining=2, invalid
    EXPECT_EQ(CRC::fsoeDataLen(2), 0u);  // < MIN_FSOE_FRAME_SIZE
}

// ============================================================================
// Build/Parse Round-Trip (commits 94e4370, 1b481e4)
// ============================================================================

class FSoEFrameRoundTripTest : public ::testing::TestWithParam<size_t> {};

TEST_P(FSoEFrameRoundTripTest, RoundTripAllSizes) {
    size_t data_len = GetParam();

    // Create test data with distinct values
    std::vector<uint8_t> data(data_len);
    for (size_t i = 0; i < data_len; ++i) {
        data[i] = static_cast<uint8_t>(0x30 + i);
    }

    uint8_t frame[64];
    uint16_t conn_id = 0x1234;
    uint8_t cmd = Command::ProcessData;

    size_t frame_size = CRC::buildFSoEFrame(frame, cmd, data.data(), data_len, conn_id);
    ASSERT_GT(frame_size, 0u);
    EXPECT_EQ(frame_size, CRC::fsoeFrameSize(data_len));
    EXPECT_EQ(frame[0], cmd);

    // Parse it back
    uint8_t out_cmd = 0;
    uint8_t out_data[18] = {0};
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    ASSERT_TRUE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, out_data, out_data_len, out_conn_id));
    EXPECT_EQ(out_cmd, cmd);
    EXPECT_EQ(out_data_len, data_len);
    EXPECT_EQ(out_conn_id, conn_id);

    // Verify data content
    for (size_t i = 0; i < data_len; ++i) {
        EXPECT_EQ(out_data[i], data[i]) << "Mismatch at byte " << i;
    }
}

INSTANTIATE_TEST_SUITE_P(Sizes, FSoEFrameRoundTripTest,
    ::testing::Values(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 15, 16, 17, 18));

TEST_P(FSoEFrameRoundTripTest, CRCVerificationFailsOnCorruption) {
    size_t data_len = GetParam();
    if (data_len == 0) return;  // No data to corrupt

    std::vector<uint8_t> data(data_len, 0xAA);

    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                             data.data(), data_len, 0x5678);
    ASSERT_GT(frame_size, 0u);

    // Corrupt one data byte
    frame[1] ^= 0xFF;

    uint8_t out_cmd = 0;
    uint8_t out_data[18] = {0};
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    EXPECT_FALSE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, out_data,
                                      out_data_len, out_conn_id));
}

INSTANTIATE_TEST_SUITE_P(Corruption, FSoEFrameRoundTripTest,
    ::testing::Values(1, 2, 3, 4, 8, 16));

// ============================================================================
// MAX_PARSE_DATA_SIZE Safety Check (commit 11210d2)
// ============================================================================

TEST(FSoEFrameMaxSizeRegression, MaxParseDataSizeConstant) {
    EXPECT_EQ(CRC::MAX_PARSE_DATA_SIZE, 18u);
    EXPECT_EQ(CRC::MAX_PARSE_DATA_SIZE, MAX_SAFE_DATA_SIZE + 2);
}

TEST(FSoEFrameMaxSizeRegression, ParseRejectsOversizedPayload) {
    // Build a frame with 20 bytes of data (exceeds MAX_PARSE_DATA_SIZE=18)
    // This requires a raw frame of fsoeFrameSize(20) = 1 + 10*4 + 2 = 43 bytes
    std::vector<uint8_t> data(20, 0x42);
    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                             data.data(), 20, 0x1234);
    ASSERT_GT(frame_size, 0u);

    uint8_t out_cmd = 0;
    uint8_t out_data[20] = {0};
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    // Should reject because out_data_len would be 20 > MAX_PARSE_DATA_SIZE
    EXPECT_FALSE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, out_data,
                                      out_data_len, out_conn_id));
}

TEST(FSoEFrameMaxSizeRegression, ParseAcceptsMaxPayload) {
    // 18 bytes is exactly MAX_PARSE_DATA_SIZE — should be accepted
    std::vector<uint8_t> data(18, 0x55);
    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::FailSafeData,
                                             data.data(), 18, 0x1234);
    ASSERT_GT(frame_size, 0u);

    uint8_t out_cmd = 0;
    uint8_t out_data[18] = {0};
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    ASSERT_TRUE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, out_data,
                                     out_data_len, out_conn_id));
    EXPECT_EQ(out_data_len, 18u);
}

TEST(FSoEFrameMaxSizeRegression, ParseNullOutDataDoesntCrash) {
    // parseFSoEFrame with out_data=nullptr should not crash
    uint8_t data[] = {0x01, 0x02};
    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::Session,
                                             data, 2, 0x1234);

    uint8_t out_cmd = 0;
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    EXPECT_TRUE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, nullptr,
                                     out_data_len, out_conn_id));
    EXPECT_EQ(out_data_len, 2u);
}

// ============================================================================
// Odd-Length Frame Format (commit 94e4370)
// ============================================================================
//
// Old format padded odd bytes to 2 bytes before CRC.
// New format: 1 data byte + 2 CRC bytes (no padding), per ETG.5100.
//

TEST(FSoEOddLengthFrameRegression, OddByteChunkIs3BytesNot4) {
    // 1-byte payload: frame = CMD(1) + 1 odd chunk(3) + ConnID(2) = 6 bytes
    uint8_t data = 0xAB;
    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                             &data, 1, 0x1234);
    EXPECT_EQ(frame_size, 6u);

    // Verify the odd chunk layout: [data_byte] [crc_lo] [crc_hi]
    // No padding byte — the byte after data is CRC, not a padding zero
    EXPECT_EQ(frame[1], 0xAB);  // Data byte
    // frame[2] and frame[3] are CRC bytes
    uint16_t crc = CRC::calculate(&data, 1);
    EXPECT_EQ(frame[2], crc & 0xFF);
    EXPECT_EQ(frame[3], (crc >> 8) & 0xFF);
    // frame[4] and frame[5] are ConnID
    EXPECT_EQ(frame[4], 0x34);
    EXPECT_EQ(frame[5], 0x12);
}

TEST(FSoEOddLengthFrameRegression, OddLengthParsesCorrectly) {
    // 3-byte payload: 1 full chunk + 1 odd chunk
    uint8_t data[] = {0x11, 0x22, 0x33};
    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::Parameter,
                                             data, 3, 0xABCD);
    EXPECT_EQ(frame_size, 10u);  // 1 + 4 + 3 + 2

    uint8_t out_cmd = 0;
    uint8_t out_data[18] = {0};
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    ASSERT_TRUE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, out_data,
                                     out_data_len, out_conn_id));
    EXPECT_EQ(out_data_len, 3u);
    EXPECT_EQ(out_data[0], 0x11);
    EXPECT_EQ(out_data[1], 0x22);
    EXPECT_EQ(out_data[2], 0x33);
}

TEST(FSoEOddLengthFrameRegression, OddCRCVerified) {
    // Corrupt the odd byte's CRC — should fail
    uint8_t data[] = {0x01, 0x02, 0x03};
    uint8_t frame[64];
    size_t frame_size = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                             data, 3, 0x1234);

    // Corrupt the odd chunk's CRC (byte at offset 1+4+1 = 6)
    frame[6] ^= 0xFF;

    uint8_t out_cmd = 0;
    uint8_t out_data[18] = {0};
    size_t out_data_len = 0;
    uint16_t out_conn_id = 0;

    EXPECT_FALSE(CRC::parseFSoEFrame(frame, frame_size, out_cmd, out_data,
                                      out_data_len, out_conn_id));
}
