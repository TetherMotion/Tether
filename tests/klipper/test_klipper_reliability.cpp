/**
 * @file test_klipper_reliability.cpp
 * @brief Tests for the reliability layer: SerialQueue, RtoEstimator.
 */

#include <gtest/gtest.h>
#include "tether/klipper/reliability/SerialQueue.hpp"
#include "tether/klipper/reliability/RtoEstimator.hpp"
#include "tether/klipper/reliability/SequenceCounter.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"

#include <chrono>
#include <thread>

using namespace tether::klipper::reliability;
using namespace tether::klipper::transport;
using namespace tether::klipper::protocol;

TEST(KlipperSequenceCounter, Wraps16) {
    SequenceCounter sc(15);
    EXPECT_EQ(sc.value(), 15u);
    sc.advance();
    EXPECT_EQ(sc.value(), 0u);
    sc.advance();
    EXPECT_EQ(sc.value(), 1u);
}

TEST(KlipperSequenceCounter, Pending) {
    EXPECT_EQ(SequenceCounter::pending(5, 3), 2u);
    EXPECT_EQ(SequenceCounter::pending(0, 15), 1u);
    EXPECT_EQ(SequenceCounter::pending(3, 3), 0u);
}

TEST(KlipperRtoEstimator, FirstSample) {
    RtoEstimator est;
    EXPECT_GT(est.rto(), 0);
    est.update(0.1);
    EXPECT_GT(est.srtt(), 0);
    EXPECT_GE(est.rto(), 0.025);
}

TEST(KlipperRtoEstimator, Convergence) {
    RtoEstimator est;
    for (int i = 0; i < 10; ++i) est.update(0.05);
    EXPECT_NEAR(est.srtt(), 0.05, 0.01);
}

TEST(KlipperSerialQueue, SendAndAck) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    auto& dev = pair.deviceEnd();
    SerialQueue sq(host);
    std::vector<uint8_t> content = {0x01, 0x02};
    auto seq = sq.send(content);
    ASSERT_TRUE(seq.has_value());
    EXPECT_EQ(*seq, 0u);
    EXPECT_EQ(sq.pendingCount(), 1u);
    // Device reads and sends ack.
    auto rd = dev.readAll();
    auto pb = parseBlock(rd);
    ASSERT_EQ(pb.status, BlockParseStatus::Ok);
    auto ack = buildAckBlock(pb.block.sequence);
    dev.write(ack);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto ackRd = host.readAll();
    auto ackPb = parseBlock(ackRd);
    sq.processAck(ackPb.block);
    EXPECT_EQ(sq.pendingCount(), 0u);
}

TEST(KlipperSerialQueue, WindowFull) {
    LoopbackTransportPair pair;
    auto& host = pair.hostEnd();
    SerialQueue sq(host, 2); // window of 2
    std::vector<uint8_t> content = {0x01};
    EXPECT_TRUE(sq.send(content).has_value());
    EXPECT_TRUE(sq.send(content).has_value());
    EXPECT_FALSE(sq.send(content).has_value()); // window full
}
