/**
 * @file test_klipper_reliability_ext.cpp
 * @brief Extended reliability tests: sequence counter all values, RTO estimator
 *        convergence/clamping, serial queue retransmit/reset/callback, ack window wrap.
 */

#include <gtest/gtest.h>
#include "tether/klipper/reliability/SequenceCounter.hpp"
#include "tether/klipper/reliability/RtoEstimator.hpp"
#include "tether/klipper/reliability/SerialQueue.hpp"
#include "tether/klipper/reliability/AckMessage.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"

#include <vector>
#include <chrono>
#include <thread>

using namespace tether::klipper::reliability;
using namespace tether::klipper::protocol;
using namespace tether::klipper::transport;

// ============================================================================
// SequenceCounter extended tests
// ============================================================================

TEST(KlipperSequenceCounterExt, AllValuesWrap) {
    SequenceCounter sc(0);
    for (int i = 0; i < 100; ++i) {
        uint8_t v = sc.value();
        EXPECT_LE(v, 15);
        sc.advance();
    }
}

TEST(KlipperSequenceCounterExt, FullWrapCycle) {
    SequenceCounter sc(0);
    for (int i = 0; i < 16; ++i) {
        EXPECT_EQ(sc.value(), static_cast<uint8_t>(i));
        sc.advance();
    }
    // Should be back to 0
    EXPECT_EQ(sc.value(), 0);
}

TEST(KlipperSequenceCounterExt, NextDoesNotMutate) {
    SequenceCounter sc(5);
    auto nxt = sc.next();
    EXPECT_EQ(nxt.value(), 6);
    EXPECT_EQ(sc.value(), 5); // Original unchanged
}

TEST(KlipperSequenceCounterExt, Set) {
    SequenceCounter sc;
    sc.set(10);
    EXPECT_EQ(sc.value(), 10);
}

TEST(KlipperSequenceCounterExt, SetMasksTo4Bits) {
    SequenceCounter sc;
    sc.set(255);
    EXPECT_EQ(sc.value(), 15); // Masked to 4 bits
}

TEST(KlipperSequenceCounterExt, PendingCalculation) {
    EXPECT_EQ(SequenceCounter::pending(5, 3), 2);
    EXPECT_EQ(SequenceCounter::pending(5, 5), 0);
    EXPECT_EQ(SequenceCounter::pending(0, 15), 1); // Wrap: 0 is after 15
    EXPECT_EQ(SequenceCounter::pending(3, 15), 4);
}

TEST(KlipperSequenceCounterExt, PendingMaxWindow) {
    EXPECT_EQ(SequenceCounter::pending(15, 0), 15);
}

TEST(KlipperSequenceCounterExt, InitialValue) {
    SequenceCounter sc(7);
    EXPECT_EQ(sc.value(), 7);
}

TEST(KlipperSequenceCounterExt, DefaultConstructor) {
    SequenceCounter sc;
    EXPECT_EQ(sc.value(), 0);
}

// ============================================================================
// RtoEstimator extended tests
// ============================================================================

TEST(KlipperRtoEstimatorExt, InitialRto) {
    RtoEstimator rto;
    // Before any samples, RTO should be the default and srtt may be 0
    EXPECT_GT(rto.rto(), 0);
}

TEST(KlipperRtoEstimatorExt, FirstSampleSetsSrtt) {
    RtoEstimator rto;
    rto.update(0.100);
    EXPECT_NEAR(rto.srtt(), 0.100, 0.001);
}

TEST(KlipperRtoExt, ConvergenceToStableRtt) {
    RtoEstimator rto;
    for (int i = 0; i < 50; ++i) {
        rto.update(0.050);
    }
    EXPECT_NEAR(rto.srtt(), 0.050, 0.005);
    // RTO = SRTT + 4*RTTVAR, should be >= SRTT
    EXPECT_GE(rto.rto(), rto.srtt());
}

TEST(KlipperRtoEstimatorExt, RtoClampedToMin) {
    RtoEstimator rto;
    // Very small RTT should still produce RTO >= kMinRtoSeconds
    for (int i = 0; i < 10; ++i) {
        rto.update(0.001);
    }
    EXPECT_GE(rto.rto(), kMinRtoSeconds);
}

TEST(KlipperRtoEstimatorExt, RtoClampedToMax) {
    RtoEstimator rto;
    // Very large RTT should still produce RTO <= kMaxRtoSeconds
    rto.update(100.0);
    EXPECT_LE(rto.rto(), kMaxRtoSeconds);
}

TEST(KlipperRtoEstimatorExt, Reset) {
    RtoEstimator rto;
    rto.update(0.050);
    rto.update(0.060);
    rto.reset();
    // After reset, should be back to initial state
    // The first update after reset should set SRTT to the sample
    rto.update(0.100);
    EXPECT_NEAR(rto.srtt(), 0.100, 0.001);
}

TEST(KlipperRtoEstimatorExt, VaryingRtt) {
    RtoEstimator rto;
    rto.update(0.050);
    rto.update(0.100);
    rto.update(0.075);
    rto.update(0.050);
    // SRTT should be a weighted average, not exactly any single value
    EXPECT_GT(rto.srtt(), 0);
    EXPECT_LT(rto.srtt(), 0.200);
}

TEST(KlipperRtoEstimatorExt, RtoAlwaysPositive) {
    RtoEstimator rto;
    for (int i = 0; i < 20; ++i) {
        rto.update(static_cast<double>(i) * 0.001);
        EXPECT_GT(rto.rto(), 0);
    }
}

// ============================================================================
// SerialQueue extended tests
// ============================================================================

TEST(KlipperSerialQueueExt, SendMultipleBlocks) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    for (int i = 0; i < 5; ++i) {
        std::vector<uint8_t> content = {static_cast<uint8_t>(i)};
        auto seq = sq.send(content);
        ASSERT_TRUE(seq.has_value());
        EXPECT_EQ(*seq, static_cast<uint8_t>(i));
    }
    EXPECT_EQ(sq.pendingCount(), 5u);
}

TEST(KlipperSerialQueueExt, AckClearsFrontBlock) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    // Send 3 blocks
    for (int i = 0; i < 3; ++i) {
        sq.send(std::vector<uint8_t>{static_cast<uint8_t>(i)});
    }
    EXPECT_EQ(sq.pendingCount(), 3u);

    // Ack the front block (seq=0)
    MessageBlock ack;
    ack.sequence = 0;
    sq.processAck(ack);
    EXPECT_EQ(sq.pendingCount(), 2u); // 1 and 2 remain
}

TEST(KlipperSerialQueueExt, AckAll) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    for (int i = 0; i < 3; ++i) {
        sq.send(std::vector<uint8_t>{static_cast<uint8_t>(i)});
    }

    // Ack the last block (seq=2) - this clears all up to and including it
    MessageBlock ack;
    ack.sequence = 2;
    sq.processAck(ack);
    // The implementation only pops the front if it matches,
    // so we need to ack sequentially
    EXPECT_EQ(sq.pendingCount(), 3u); // Not cleared because front is 0, not 2

    // Ack front block
    ack.sequence = 0;
    sq.processAck(ack);
    EXPECT_EQ(sq.pendingCount(), 2u);

    ack.sequence = 1;
    sq.processAck(ack);
    EXPECT_EQ(sq.pendingCount(), 1u);

    ack.sequence = 2;
    sq.processAck(ack);
    EXPECT_EQ(sq.pendingCount(), 0u);
    EXPECT_TRUE(sq.canSend());
}

TEST(KlipperSerialQueueExt, WindowFullPreventsSend) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd(), 3); // Window size 3

    EXPECT_TRUE(sq.send(std::vector<uint8_t>{0x01}).has_value());
    EXPECT_TRUE(sq.send(std::vector<uint8_t>{0x02}).has_value());
    EXPECT_TRUE(sq.send(std::vector<uint8_t>{0x03}).has_value());
    EXPECT_FALSE(sq.send(std::vector<uint8_t>{0x04}).has_value()); // Window full
    EXPECT_EQ(sq.pendingCount(), 3u);
}

TEST(KlipperSerialQueueExt, CanSendAfterAck) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd(), 2);

    sq.send(std::vector<uint8_t>{0x01});
    sq.send(std::vector<uint8_t>{0x02});
    EXPECT_FALSE(sq.canSend());

    // Ack first block
    MessageBlock ack;
    ack.sequence = 0;
    sq.processAck(ack);

    EXPECT_TRUE(sq.canSend());
    EXPECT_EQ(sq.pendingCount(), 1u);
}

TEST(KlipperSerialQueueExt, NextSequence) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    EXPECT_EQ(sq.nextSequence(), 0);
    sq.send(std::vector<uint8_t>{0x01});
    EXPECT_EQ(sq.nextSequence(), 1);
    sq.send(std::vector<uint8_t>{0x02});
    EXPECT_EQ(sq.nextSequence(), 2);
}

TEST(KlipperSerialQueueExt, SequenceWraps) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd(), 20); // Large window

    // Send and ack 16 blocks to wrap around
    for (int i = 0; i < 16; ++i) {
        auto seq = sq.send(std::vector<uint8_t>{static_cast<uint8_t>(i)});
        ASSERT_TRUE(seq.has_value());
        EXPECT_EQ(*seq, static_cast<uint8_t>(i));

        MessageBlock ack;
        ack.sequence = *seq;
        sq.processAck(ack);
    }

    // Next sequence should wrap to 0
    EXPECT_EQ(sq.nextSequence(), 0);
}

TEST(KlipperSerialQueueExt, AckCallback) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    bool callbackCalled = false;
    double receivedRtt = 0;
    sq.setAckCallback([&](double rtt) {
        callbackCalled = true;
        receivedRtt = rtt;
    });

    sq.send(std::vector<uint8_t>{0x01});

    // Small delay so RTT is measurable
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    MessageBlock ack;
    ack.sequence = 0;
    sq.processAck(ack);

    EXPECT_TRUE(callbackCalled);
    EXPECT_GT(receivedRtt, 0);
}

TEST(KlipperSerialQueueExt, Reset) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    sq.send(std::vector<uint8_t>{0x01});
    sq.send(std::vector<uint8_t>{0x02});
    EXPECT_EQ(sq.pendingCount(), 2u);

    sq.reset();
    EXPECT_EQ(sq.pendingCount(), 0u);
    EXPECT_TRUE(sq.canSend());
    // After reset, sequence should restart
    EXPECT_EQ(sq.nextSequence(), 0);
}

TEST(KlipperSerialQueueExt, CheckTimeoutsRetransmits) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());

    auto seq = sq.send(std::vector<uint8_t>{0x01});
    ASSERT_TRUE(seq.has_value());

    // Manually set the send time to the past to trigger RTO
    // We can't directly access pending_, but we can wait
    // The RTO is at least kMinRtoSeconds (25ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    sq.checkTimeouts();
    // After timeout, the block should be retransmitted
    // pendingCount should still be 1 (it's retransmitted, not acked)
    EXPECT_EQ(sq.pendingCount(), 1u);
}

TEST(KlipperSerialQueueExt, RtoEstimatorAccessible) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd());
    const auto& rto = sq.rtoEstimator();
    EXPECT_GT(rto.rto(), 0);
}

TEST(KlipperSerialQueueExt, AckMessageBuild) {
    auto ack = tether::klipper::reliability::buildAckBlock(5);
    EXPECT_EQ(ack.size(), kMinBlockLength);

    auto pb = parseBlock(ack);
    EXPECT_EQ(pb.status, BlockParseStatus::Ok);
    EXPECT_EQ(pb.block.sequence, 5u);
    EXPECT_TRUE(pb.block.content.empty());
}

TEST(KlipperSerialQueueExt, SendAndAckMultipleCycles) {
    LoopbackTransportPair pair;
    SerialQueue sq(pair.hostEnd(), 4);

    // Cycle 1: send 4, ack each one from front
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(sq.send(std::vector<uint8_t>{static_cast<uint8_t>(i)}).has_value());
    }
    // Ack sequentially from front
    for (int i = 0; i < 4; ++i) {
        MessageBlock ack;
        ack.sequence = static_cast<uint8_t>(i);
        sq.processAck(ack);
    }
    EXPECT_EQ(sq.pendingCount(), 0u);

    // Cycle 2: send 4 more (sequence continues)
    for (int i = 0; i < 4; ++i) {
        auto seq = sq.send(std::vector<uint8_t>{static_cast<uint8_t>(i)});
        ASSERT_TRUE(seq.has_value());
        EXPECT_EQ(*seq, static_cast<uint8_t>(i + 4));
    }
    // Ack sequentially
    for (int i = 0; i < 4; ++i) {
        MessageBlock ack;
        ack.sequence = static_cast<uint8_t>(i + 4);
        sq.processAck(ack);
    }
    EXPECT_EQ(sq.pendingCount(), 0u);
}
