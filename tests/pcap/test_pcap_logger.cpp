/**
 * @file test_pcap_logger.cpp
 * @brief PCAPLogger coverage: init/close/isOpen, frame logging with
 *        direction filtering, counters/stats, file rotation limits,
 *        and error paths — against real temp files.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "tether/packetloggers/PacketLogger.hpp"
#include "tether/packetloggers/pcap/PCAPLogger.hpp"
#include "tether/packetloggers/pcap/PCAPLoggerConfig.hpp"

namespace {

using namespace Tether::PacketLoggers;
using namespace Tether::PacketLoggers::PCAP;

class PCAPLoggerTest : public ::testing::Test {
protected:
    std::string path_ = "/tmp/tether_pcap_test_" +
                        std::to_string(getpid()) + ".pcap";
    void TearDown() override { std::filesystem::remove(path_); }

    PCAPLoggerConfig cfg() {
        PCAPLoggerConfig c;
        c.filename = path_;
        return c;
    }

    static const uint8_t* frame() {
        static const uint8_t f[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0x02, 0x00, 0x00, 0x00, 0x00, 0x01,
                                    0xA4, 0x88, 0x00, 0x00, 0x00, 0x00};
        return f;
    }
    static constexpr size_t kFrameLen = 18;
};

TEST_F(PCAPLoggerTest, InitOpenCloseLifecycle) {
    PCAPLogger l;
    EXPECT_FALSE(l.isOpen());
    EXPECT_EQ(l.init(cfg()), Error::OK);
    EXPECT_TRUE(l.isOpen());
    l.close();
    EXPECT_FALSE(l.isOpen());
    EXPECT_TRUE(std::filesystem::exists(path_));
}

TEST_F(PCAPLoggerTest, LogFrameWritesAndCounts) {
    PCAPLogger l;
    ASSERT_EQ(l.init(cfg()), Error::OK);
    EXPECT_EQ(l.logFrame(frame(), kFrameLen, FrameDirection::Tx),
              Error::OK);
    EXPECT_EQ(l.logFrame(frame(), kFrameLen, FrameDirection::Rx, 12345),
              Error::OK);
    EXPECT_EQ(l.getFrameCount(), 2u);
    // getFileSize() is a documented stub (writer doesn't expose size);
    // stats carry the byte total instead.
    EXPECT_EQ(l.getFileSize(), 0u);
    EXPECT_GE(l.getStats().totalBytes, 2 * kFrameLen);
    l.flush();
    l.close();
}

TEST_F(PCAPLoggerTest, PcapGlobalHeaderWritten) {
    {
        PCAPLogger l;
        ASSERT_EQ(l.init(cfg()), Error::OK);
        l.close();
    }
    std::ifstream in(path_, std::ios::binary);
    uint32_t magic = 0;
    in.read(reinterpret_cast<char*>(&magic), 4);
    EXPECT_EQ(magic, 0x0A0D0D0Au);  // pcapng section-header magic
}

TEST_F(PCAPLoggerTest, DirectionFilteringDropsFrames) {
    PCAPLogger l;
    auto c = cfg();
    c.logRx = false;  // drop RX
    ASSERT_EQ(l.init(c), Error::OK);
    EXPECT_EQ(l.logFrame(frame(), kFrameLen, FrameDirection::Tx),
              Error::OK);
    EXPECT_EQ(l.logFrame(frame(), kFrameLen, FrameDirection::Rx),
              Error::OK);  // filtered, not an error
    EXPECT_EQ(l.getFrameCount(), 1u);
    l.close();
}

TEST_F(PCAPLoggerTest, StatsTrackCounts) {
    PCAPLogger l;
    ASSERT_EQ(l.init(cfg()), Error::OK);
    l.logFrame(frame(), kFrameLen, FrameDirection::Tx);
    l.logFrame(frame(), kFrameLen, FrameDirection::Tx);
    auto s = l.getStats();
    // The adapter tracks totals only — per-direction split isn't kept.
    EXPECT_EQ(s.totalBytes, 2 * kFrameLen);
    EXPECT_EQ(s.droppedFrames, 0u);
    EXPECT_EQ(l.getFrameCount(), 2u);
    l.close();
}

TEST_F(PCAPLoggerTest, LogFrameBeforeInitFails) {
    PCAPLogger l;
    EXPECT_NE(l.logFrame(frame(), kFrameLen, FrameDirection::Tx),
              Error::OK);
}

TEST_F(PCAPLoggerTest, InitBadPathFails) {
    PCAPLogger l;
    PCAPLoggerConfig bad;
    bad.filename = "/nonexistent_dir_xyz/deep/f.pcap";
    EXPECT_NE(l.init(bad), Error::OK);
    EXPECT_FALSE(l.isOpen());
}

TEST_F(PCAPLoggerTest, MaxFileSizeFieldAccepted) {
    // The logger accepts maxFileSize/rotateFiles in config but does not
    // wire them into the writer — frames all log normally.  Documents
    // current behavior (rotation lives at PCAPWriter level).
    PCAPLogger l;
    auto c = cfg();
    c.maxFileSize = 64;
    c.rotateFiles = true;
    ASSERT_EQ(l.init(c), Error::OK);
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(l.logFrame(frame(), kFrameLen, FrameDirection::Tx),
                  Error::OK);
    EXPECT_EQ(l.getFrameCount(), 3u);
    l.close();
}

TEST_F(PCAPLoggerTest, FactoryFunctions) {
    auto l1 = createPCAPLogger();
    EXPECT_NE(l1, nullptr);
    EXPECT_FALSE(l1->isOpen());
    auto l2 = createPCAPLogger(cfg());
    ASSERT_NE(l2, nullptr);
    EXPECT_TRUE(l2->isOpen());
    l2->close();
}

}  // namespace
