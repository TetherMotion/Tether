/**
 * @file test_klipper_transport.cpp
 * @brief Tests for Klipper transports: loopback, pipe.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/transport/PipeTransport.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"

#include <unistd.h>
#include <vector>

using namespace tether::klipper::transport;
using namespace tether::klipper::protocol;

TEST(KlipperLoopbackTransport, RoundTrip) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();
    std::vector<uint8_t> data = {1, 2, 3, 4, 5};
    EXPECT_EQ(host.write(data), 5u);
    EXPECT_EQ(dev.available(), 5u);
    uint8_t buf[16];
    size_t n = dev.read(buf, 16);
    EXPECT_EQ(n, 5u);
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + n), data);
}

TEST(KlipperLoopbackTransport, Bidirectional) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();
    std::vector<uint8_t> d1 = {0xAA, 0xBB};
    std::vector<uint8_t> d2 = {0xCC, 0xDD, 0xEE};
    host.write(d1);
    dev.write(d2);
    EXPECT_EQ(dev.available(), 2u);
    EXPECT_EQ(host.available(), 3u);
    uint8_t buf[16];
    dev.read(buf, 16);
    EXPECT_EQ(buf[0], 0xAA);
    EXPECT_EQ(buf[1], 0xBB);
    host.read(buf, 16);
    EXPECT_EQ(buf[0], 0xCC);
    EXPECT_EQ(buf[1], 0xDD);
    EXPECT_EQ(buf[2], 0xEE);
}

TEST(KlipperLoopbackTransport, MessageBlockRoundTrip) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();
    std::vector<uint8_t> content = {0x7B, 0x06, 0x01, 0x2D};
    auto blk = buildBlockVec(7, content);
    host.write(blk);
    auto rd = dev.readAll();
    auto pb = parseBlock(rd);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 7u);
    EXPECT_EQ(pb.block.content, content);
}

TEST(KlipperPipeTransport, RoundTrip) {
    int pfd[2];
    ASSERT_EQ(pipe(pfd), 0);
    PipeTransportConfig cfg;
    cfg.readFd = pfd[0];
    cfg.writeFd = pfd[1];
    cfg.ownsFds = true;
    PipeTransport t(cfg);
    ASSERT_TRUE(t.open());
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    EXPECT_EQ(t.write(data), 3u);
    EXPECT_EQ(t.available(), 3u);
    uint8_t buf[16];
    size_t n = t.read(buf, 16);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + n), data);
}
