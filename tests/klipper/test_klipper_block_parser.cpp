/**
 * @file test_klipper_block_parser.cpp
 * @brief Tests for the stateful BlockParser and BlockReader classes.
 *
 * Exercises:
 *   - BlockParser: statistics tracking, recovery mode, error callback, reset.
 *   - BlockReader: streaming readNext(), reset(), recovery mode, statistics.
 *   - ParsedBlock: new wireLength and skippedBytes metadata fields.
 *
 * These mirror the pcapng reader's recovery-mode and streaming-API tests,
 * adapted to the Klipper message-block framing.
 */

#include <gtest/gtest.h>
#include "tether/klipper/protocol/BlockParser.hpp"
#include "tether/klipper/protocol/BlockReader.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"

#include <vector>
#include <string>
#include <atomic>

using namespace tether::klipper::protocol;
using namespace tether::klipper::transport;

// ============================================================================
// Helper: build a valid block with known content
// ============================================================================
static std::vector<uint8_t> makeBlock(uint8_t seq, std::vector<uint8_t> content) {
    return buildBlockVec(seq, content);
}

/// Corrupt the CRC of a block in-place (flip CRC_HI byte).
static void corruptCrc(std::vector<uint8_t>& block) {
    ASSERT_GE(block.size(), kTrailerSize);
    block[block.size() - 3] ^= 0xFF;
}

/// Corrupt the sync byte of a block in-place.
static void corruptSync(std::vector<uint8_t>& block) {
    ASSERT_GE(block.size(), 1u);
    block[block.size() - 1] = 0x00;
}

// ============================================================================
// ParsedBlock metadata: wireLength and skippedBytes
// ============================================================================

TEST(KlipperParsedBlockMetadata, WireLengthSetOnOk) {
    std::vector<uint8_t> content = {0x01, 0x02, 0x03};
    auto wire = makeBlock(5, content);
    auto pb = parseBlock(wire);
    ASSERT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.wireLength, wire.size());
    EXPECT_EQ(pb.skippedBytes, 0u);
}

TEST(KlipperParsedBlockMetadata, SkippedBytesWithGarbage) {
    std::vector<uint8_t> content = {0xAA, 0xBB};
    auto wire = makeBlock(3, content);
    std::vector<uint8_t> garbage = {0xFF, 0xFE, 0xFD};
    std::vector<uint8_t> combined;
    combined.insert(combined.end(), garbage.begin(), garbage.end());
    combined.insert(combined.end(), wire.begin(), wire.end());

    auto pb = parseBlock(combined);
    ASSERT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.skippedBytes, 3u);
    EXPECT_EQ(pb.wireLength, wire.size());
    EXPECT_EQ(pb.block.content, content);
}

TEST(KlipperParsedBlockMetadata, WireLengthSetOnBadCrc) {
    std::vector<uint8_t> content = {0x01, 0x02};
    auto wire = makeBlock(0, content);
    corruptCrc(wire);
    auto pb = parseBlock(wire);
    EXPECT_EQ(pb.status, BlockParseStatus::BadCrc);
    EXPECT_EQ(pb.wireLength, wire.size());
}

TEST(KlipperParsedBlockMetadata, SkippedBytesOnNeedMoreData) {
    std::vector<uint8_t> content = {0x01};
    auto wire = makeBlock(0, content);
    std::vector<uint8_t> garbage = {0xFF, 0xFE};
    std::vector<uint8_t> combined;
    combined.insert(combined.end(), garbage.begin(), garbage.end());
    combined.insert(combined.end(), wire.begin(), wire.end());
    // Truncate the last 2 bytes so the block is incomplete.
    combined.resize(combined.size() - 2);

    auto pb = parseBlock(combined);
    EXPECT_EQ(pb.status, BlockParseStatus::NeedMoreData);
    EXPECT_EQ(pb.skippedBytes, 2u); // garbage bytes were skipped
}

// ============================================================================
// BlockParser: statistics tracking (no recovery mode)
// ============================================================================

TEST(KlipperBlockParser, StatsSingleBlock) {
    BlockParser parser;
    auto wire = makeBlock(1, {0x01});
    auto pb = parser.parse(wire);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(parser.stats().blocksParsed, 1u);
    EXPECT_EQ(parser.stats().totalErrors(), 0u);
}

TEST(KlipperBlockParser, StatsMultipleBlocks) {
    BlockParser parser;
    for (uint8_t i = 0; i < 5; ++i) {
        auto wire = makeBlock(i, {static_cast<uint8_t>(i)});
        auto pb = parser.parse(wire);
        EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    }
    EXPECT_EQ(parser.stats().blocksParsed, 5u);
    EXPECT_EQ(parser.stats().totalErrors(), 0u);
}

TEST(KlipperBlockParser, StatsBadCrc) {
    BlockParser parser;
    auto wire = makeBlock(0, {0x01, 0x02});
    corruptCrc(wire);
    auto pb = parser.parse(wire);
    EXPECT_EQ(pb.status, BlockParseStatus::BadCrc);
    EXPECT_EQ(parser.stats().badCrcCount, 1u);
    EXPECT_EQ(parser.stats().blocksParsed, 0u);
}

TEST(KlipperBlockParser, StatsBytesSkipped) {
    BlockParser parser;
    auto wire = makeBlock(2, {0xAA});
    std::vector<uint8_t> garbage = {0xFF, 0xFE, 0xFD, 0xFC};
    std::vector<uint8_t> combined;
    combined.insert(combined.end(), garbage.begin(), garbage.end());
    combined.insert(combined.end(), wire.begin(), wire.end());

    auto pb = parser.parse(combined);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(parser.stats().bytesSkipped, 4u);
}

TEST(KlipperBlockParser, StatsReset) {
    BlockParser parser;
    auto wire = makeBlock(0, {0x01});
    parser.parse(wire);
    parser.parse(wire);
    EXPECT_EQ(parser.stats().blocksParsed, 2u);

    parser.resetStats();
    EXPECT_EQ(parser.stats().blocksParsed, 0u);
    EXPECT_EQ(parser.stats().totalErrors(), 0u);
}

TEST(KlipperBlockParser, TotalBlocksAndErrors) {
    BlockParser parser;
    auto good = makeBlock(0, {0x01});
    auto bad = makeBlock(0, {0x02});
    corruptCrc(bad);

    parser.parse(good);
    parser.parse(bad);
    parser.parse(good);

    EXPECT_EQ(parser.stats().blocksParsed, 2u);
    EXPECT_EQ(parser.stats().badCrcCount, 1u);
    EXPECT_EQ(parser.stats().totalBlocks(), 3u);
    EXPECT_EQ(parser.stats().totalErrors(), 1u);
}

// ============================================================================
// BlockParser: recovery mode
// ============================================================================

TEST(KlipperBlockParserRecovery, SkipsBadCrcBlock) {
    BlockParser parser;
    parser.setRecoveryMode(true);

    auto good = makeBlock(1, {0x01});
    auto bad = makeBlock(0, {0x02});
    corruptCrc(bad);

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), bad.begin(), bad.end());
    combined.insert(combined.end(), good.begin(), good.end());

    auto pb = parser.parse(combined);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 1u);
    EXPECT_EQ(pb.block.content, std::vector<uint8_t>({0x01}));
    EXPECT_EQ(parser.stats().badCrcCount, 1u);
    EXPECT_EQ(parser.stats().blocksParsed, 1u);
    EXPECT_EQ(parser.skippedBlockCount(), 1u);
}

TEST(KlipperBlockParserRecovery, InvokesErrorCallback) {
    BlockParser parser;
    std::atomic<int> callbackCount{0};
    std::atomic<BlockParseStatus> lastStatus{BlockParseStatus::Ok};

    parser.setRecoveryMode(true, [&](size_t, BlockParseStatus s, std::string_view) {
        ++callbackCount;
        lastStatus = s;
    });

    auto good = makeBlock(1, {0x01});
    auto bad = makeBlock(0, {0x02});
    corruptCrc(bad);

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), bad.begin(), bad.end());
    combined.insert(combined.end(), good.begin(), good.end());

    auto pb = parser.parse(combined);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(callbackCount.load(), 1);
    EXPECT_EQ(lastStatus.load(), BlockParseStatus::BadCrc);
}

TEST(KlipperBlockParserRecovery, CallbackReceivesOffsetAndMessage) {
    BlockParser parser;
    size_t capturedOffset = 999;
    std::string capturedMsg;

    parser.setRecoveryMode(true, [&](size_t off, BlockParseStatus, std::string_view msg) {
        capturedOffset = off;
        capturedMsg = std::string(msg);
    });

    auto good = makeBlock(1, {0x01});
    auto bad = makeBlock(0, {0x02});
    corruptCrc(bad);

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), bad.begin(), bad.end());
    combined.insert(combined.end(), good.begin(), good.end());

    parser.parse(combined);
    // The bad block starts at offset 0 (no garbage before it).
    EXPECT_EQ(capturedOffset, 0u);
    EXPECT_FALSE(capturedMsg.empty());
}

TEST(KlipperBlockParserRecovery, SkipsMultipleBadBlocks) {
    BlockParser parser;
    parser.setRecoveryMode(true);

    auto good = makeBlock(2, {0x03});
    auto bad1 = makeBlock(0, {0x01});
    corruptCrc(bad1);
    auto bad2 = makeBlock(1, {0x02});
    corruptCrc(bad2);

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), bad1.begin(), bad1.end());
    combined.insert(combined.end(), bad2.begin(), bad2.end());
    combined.insert(combined.end(), good.begin(), good.end());

    auto pb = parser.parse(combined);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 2u);
    EXPECT_EQ(parser.stats().badCrcCount, 2u);
    EXPECT_EQ(parser.stats().blocksParsed, 1u);
    EXPECT_EQ(parser.skippedBlockCount(), 2u);
}

TEST(KlipperBlockParserRecovery, NoRecoveryReturnsError) {
    BlockParser parser;
    // Recovery mode disabled (default).
    auto bad = makeBlock(0, {0x01});
    corruptCrc(bad);

    auto pb = parser.parse(bad);
    EXPECT_EQ(pb.status, BlockParseStatus::BadCrc);
    EXPECT_EQ(parser.skippedBlockCount(), 0u);
}

TEST(KlipperBlockParserRecovery, ResetClearsRecoveryMode) {
    BlockParser parser;
    parser.setRecoveryMode(true);
    EXPECT_TRUE(parser.recoveryMode());

    parser.reset();
    EXPECT_FALSE(parser.recoveryMode());
    EXPECT_EQ(parser.skippedBlockCount(), 0u);
    EXPECT_EQ(parser.stats().blocksParsed, 0u);
}

TEST(KlipperBlockParserRecovery, OnlyBadBlocksReturnsNeedMoreData) {
    BlockParser parser;
    parser.setRecoveryMode(true);

    auto bad = makeBlock(0, {0x01});
    corruptCrc(bad);

    auto pb = parser.parse(bad);
    // All bytes consumed by skipping; no valid block found.
    EXPECT_EQ(pb.status, BlockParseStatus::NeedMoreData);
    EXPECT_EQ(parser.skippedBlockCount(), 1u);
    EXPECT_EQ(parser.stats().badCrcCount, 1u);
}

// ============================================================================
// BlockReader: streaming readNext() API
// ============================================================================

TEST(KlipperBlockReader, ReadSingleBlock) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto wire = makeBlock(3, {0xAA, 0xBB});
    host.write(wire);

    BlockReader reader(dev);
    MessageBlock block;
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 3u);
    EXPECT_EQ(block.content, std::vector<uint8_t>({0xAA, 0xBB}));
    EXPECT_EQ(reader.stats().blocksParsed, 1u);
}

TEST(KlipperBlockReader, ReadMultipleBlocks) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto w1 = makeBlock(1, {0x01});
    auto w2 = makeBlock(2, {0x02, 0x03});
    auto w3 = makeBlock(3, {0x04, 0x05, 0x06});
    host.write(w1);
    host.write(w2);
    host.write(w3);

    BlockReader reader(dev);
    MessageBlock block;

    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 1u);
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 2u);
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 3u);
    EXPECT_FALSE(reader.readNext(block)); // no more data
    EXPECT_EQ(reader.stats().blocksParsed, 3u);
}

TEST(KlipperBlockReader, ReadEmptyReturnsFalse) {
    LoopbackTransportPair pair;
    auto& dev = pair.deviceEnd();

    BlockReader reader(dev);
    MessageBlock block;
    EXPECT_FALSE(reader.readNext(block));
    EXPECT_EQ(reader.stats().blocksParsed, 0u);
}

TEST(KlipperBlockReader, PartialBlockThenComplete) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto wire = makeBlock(5, {0x01, 0x02, 0x03, 0x04});
    // Write first half, try to read (should get false/NeedMoreData).
    host.write(std::vector<uint8_t>(wire.begin(), wire.begin() + 3));

    BlockReader reader(dev);
    MessageBlock block;
    EXPECT_FALSE(reader.readNext(block)); // partial, no complete block

    // Write the rest.
    host.write(std::vector<uint8_t>(wire.begin() + 3, wire.end()));
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 5u);
    EXPECT_EQ(block.content, std::vector<uint8_t>({0x01, 0x02, 0x03, 0x04}));
}

TEST(KlipperBlockReader, RecoveryModeSkipsBadBlock) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto good = makeBlock(7, {0xCC});
    auto bad = makeBlock(0, {0xDD});
    corruptCrc(bad);

    host.write(bad);
    host.write(good);

    BlockReader reader(dev);
    reader.setRecoveryMode(true);
    MessageBlock block;

    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 7u);
    EXPECT_EQ(block.content, std::vector<uint8_t>({0xCC}));
    EXPECT_EQ(reader.skippedBlockCount(), 1u);
    EXPECT_EQ(reader.stats().badCrcCount, 1u);
    EXPECT_EQ(reader.stats().blocksParsed, 1u);
}

TEST(KlipperBlockReader, RecoveryModeCallbackInvoked) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto good = makeBlock(1, {0x01});
    auto bad = makeBlock(0, {0x02});
    corruptCrc(bad);

    host.write(bad);
    host.write(good);

    std::atomic<int> callbackCount{0};
    BlockReader reader(dev);
    reader.setRecoveryMode(true, [&](size_t, BlockParseStatus, std::string_view) {
        ++callbackCount;
    });

    MessageBlock block;
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(callbackCount.load(), 1);
}

TEST(KlipperBlockReader, NoRecoveryModeReturnsFalseOnBadBlock) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto bad = makeBlock(0, {0x01});
    corruptCrc(bad);
    host.write(bad);

    BlockReader reader(dev);
    // Recovery mode disabled (default).
    MessageBlock block;
    EXPECT_FALSE(reader.readNext(block));
    EXPECT_EQ(reader.stats().badCrcCount, 1u);
    EXPECT_EQ(reader.stats().blocksParsed, 0u);
}

TEST(KlipperBlockReader, ResetClearsBufferAndStats) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto wire = makeBlock(1, {0x01});
    host.write(wire);

    BlockReader reader(dev);
    MessageBlock block;
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(reader.stats().blocksParsed, 1u);

    reader.reset();
    EXPECT_EQ(reader.stats().blocksParsed, 0u);
    EXPECT_EQ(reader.bufferedBytes(), 0u);
}

TEST(KlipperBlockReader, BufferedBytesAfterPartialRead) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto wire = makeBlock(1, {0x01, 0x02, 0x03});
    host.write(std::vector<uint8_t>(wire.begin(), wire.begin() + 3));

    BlockReader reader(dev);
    MessageBlock block;
    EXPECT_FALSE(reader.readNext(block));
    // Some bytes should be buffered waiting for the rest.
    EXPECT_GT(reader.bufferedBytes(), 0u);
    EXPECT_LE(reader.bufferedBytes(), 3u);
}

TEST(KlipperBlockReader, ClearBufferWithoutResettingStats) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto wire = makeBlock(1, {0x01});
    host.write(wire);

    BlockReader reader(dev);
    MessageBlock block;
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(reader.stats().blocksParsed, 1u);

    // Write a partial block, then clear the buffer.
    host.write(std::vector<uint8_t>{0x08, 0x10});
    reader.readNext(block); // attempt to read (will buffer partial data)
    reader.clearBuffer();
    EXPECT_EQ(reader.bufferedBytes(), 0u);
    // Stats should be preserved.
    EXPECT_EQ(reader.stats().blocksParsed, 1u);
}

TEST(KlipperBlockReader, GarbageBeforeBlock) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto wire = makeBlock(4, {0xAA, 0xBB});
    std::vector<uint8_t> garbage = {0xFF, 0xFE, 0xFD};
    std::vector<uint8_t> combined;
    combined.insert(combined.end(), garbage.begin(), garbage.end());
    combined.insert(combined.end(), wire.begin(), wire.end());
    host.write(combined);

    BlockReader reader(dev);
    MessageBlock block;
    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 4u);
    EXPECT_EQ(block.content, std::vector<uint8_t>({0xAA, 0xBB}));
    EXPECT_EQ(reader.stats().blocksParsed, 1u);
    EXPECT_GT(reader.stats().bytesSkipped, 0u);
}

// ============================================================================
// BlockReader: recovery mode with multiple bad blocks
// ============================================================================

TEST(KlipperBlockReaderRecovery, MultipleBadBlocksThenGood) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto good = makeBlock(3, {0x99});
    auto bad1 = makeBlock(0, {0x01});
    corruptCrc(bad1);
    auto bad2 = makeBlock(1, {0x02});
    corruptCrc(bad2);

    host.write(bad1);
    host.write(bad2);
    host.write(good);

    BlockReader reader(dev);
    reader.setRecoveryMode(true);
    MessageBlock block;

    ASSERT_TRUE(reader.readNext(block));
    EXPECT_EQ(block.sequence, 3u);
    EXPECT_EQ(block.content, std::vector<uint8_t>({0x99}));
    EXPECT_EQ(reader.skippedBlockCount(), 2u);
    EXPECT_EQ(reader.stats().badCrcCount, 2u);
    EXPECT_EQ(reader.stats().blocksParsed, 1u);
}

TEST(KlipperBlockReaderRecovery, AllBadBlocksReturnsFalse) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();

    auto bad = makeBlock(0, {0x01});
    corruptCrc(bad);
    host.write(bad);

    BlockReader reader(dev);
    reader.setRecoveryMode(true);
    MessageBlock block;
    // All data is a bad block; after skipping, no data remains.
    EXPECT_FALSE(reader.readNext(block));
    EXPECT_EQ(reader.skippedBlockCount(), 1u);
}
