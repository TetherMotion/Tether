/**
 * @file test_pcap.cpp
 * @brief Unit tests for PacketLogger and PCAP adapter
 */

#include <gtest/gtest.h>
#include "tether/packetloggers/PacketLogger.hpp"
#include "mocks/MockHAL.hpp"
#include <fstream>
#include <cstdio>

using namespace Tether::PacketLoggers;
using namespace EtherCAT::HAL::mock;

// ============================================================================
// FakePacketLogger Tests
// ============================================================================

class FakePacketLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger = std::make_unique<FakePacketLogger>();
    }

    std::unique_ptr<FakePacketLogger> logger;
};

TEST_F(FakePacketLoggerTest, InitClose) {
    PCAP::PCAPLoggerConfig config;
    config.filename = "test.pcapng";

    EXPECT_EQ(logger->init(config), Error::OK);
    EXPECT_TRUE(logger->isOpen());

    logger->close();
    EXPECT_FALSE(logger->isOpen());
}

TEST_F(FakePacketLoggerTest, LogFrame) {
    PCAP::PCAPLoggerConfig config;
    config.filename = "test.pcapng";
    logger->init(config);

    uint8_t frame[64];
    for (int i = 0; i < 64; i++) frame[i] = i;

    EXPECT_EQ(logger->logFrame(frame, 64, FrameDirection::Tx, 1000000), Error::OK);
    EXPECT_EQ(logger->logFrame(frame, 64, FrameDirection::Rx, 2000000), Error::OK);

    EXPECT_EQ(logger->getFrameCount(), 2u);
}

TEST_F(FakePacketLoggerTest, GetFrames) {
    PCAP::PCAPLoggerConfig config;
    logger->init(config);

    uint8_t frame1[32], frame2[48];
    std::memset(frame1, 0xAA, 32);
    std::memset(frame2, 0xBB, 48);

    logger->logFrame(frame1, 32, FrameDirection::Tx, 1000);
    logger->logFrame(frame2, 48, FrameDirection::Rx, 2000);

    const auto& frames = logger->getFrames();
    ASSERT_EQ(frames.size(), 2u);

    EXPECT_EQ(frames[0].data.size(), 32u);
    EXPECT_EQ(frames[0].direction, FrameDirection::Tx);
    EXPECT_EQ(frames[0].timestamp, 1000u);

    EXPECT_EQ(frames[1].data.size(), 48u);
    EXPECT_EQ(frames[1].direction, FrameDirection::Rx);
    EXPECT_EQ(frames[1].timestamp, 2000u);
}

TEST_F(FakePacketLoggerTest, ClearFrames) {
    PCAP::PCAPLoggerConfig config;
    logger->init(config);

    uint8_t frame[32] = {0};
    logger->logFrame(frame, 32, FrameDirection::Tx, 1000);
    EXPECT_EQ(logger->getFrameCount(), 1u);

    logger->clearFrames();
    EXPECT_EQ(logger->getFrameCount(), 0u);
}

TEST_F(FakePacketLoggerTest, Stats) {
    PCAP::PCAPLoggerConfig config;
    logger->init(config);

    uint8_t frame[64] = {0};
    logger->logFrame(frame, 64, FrameDirection::Tx, 1000);
    logger->logFrame(frame, 32, FrameDirection::Tx, 2000);
    logger->logFrame(frame, 48, FrameDirection::Rx, 3000);

    auto stats = logger->getStats();
    EXPECT_EQ(stats.txFrames, 2u);
    EXPECT_EQ(stats.rxFrames, 1u);
    EXPECT_EQ(stats.totalBytes, 64u + 32u + 48u);
}

TEST_F(FakePacketLoggerTest, NotInitialized) {
    // Don't call init
    uint8_t frame[32] = {0};
    EXPECT_EQ(logger->logFrame(frame, 32, FrameDirection::Tx, 1000), Error::NotInitialized);
}

// ============================================================================
// MockPacketLogger Tests
// ============================================================================

TEST(MockPacketLoggerTest, ExpectCalls) {
    MockPacketLogger mockLogger;

    PCAP::PCAPLoggerConfig config;
    EXPECT_CALL(mockLogger, init(testing::_))
        .WillOnce(testing::Return(Error::OK));
    EXPECT_CALL(mockLogger, logFrame(testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Return(Error::OK));
    EXPECT_CALL(mockLogger, close()).Times(1);

    mockLogger.init(config);

    uint8_t frame[32] = {0};
    mockLogger.logFrame(frame, 32, FrameDirection::Tx, 1000);

    mockLogger.close();
}

// ============================================================================
// PCAPLoggerConfig Tests
// ============================================================================

TEST(PCAPLoggerConfigTest, Defaults) {
    PCAP::PCAPLoggerConfig config;

    EXPECT_TRUE(config.logTx);
    EXPECT_TRUE(config.logRx);
    EXPECT_FALSE(config.appendMode);
    EXPECT_EQ(config.maxFileSize, 0u);  // Unlimited
}

// ============================================================================
// FrameDirection Tests
// ============================================================================

TEST(FrameDirectionTest, Values) {
    // Verify enum values are distinct
    EXPECT_NE(FrameDirection::Rx, FrameDirection::Tx);
}
