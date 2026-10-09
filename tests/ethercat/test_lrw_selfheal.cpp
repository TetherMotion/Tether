/**
 * @file test_lrw_selfheal.cpp
 * @brief Fault-injection tests for LogicalAddressManager self-heal:
 *        host-stall detection, wire drain, per-send slot prune, timeout
 *        streaks, ring probe escalation, and configurable response wait.
 *
 * These model the Raspberry Pi failure mode: a burst of host latency
 * (non-RT scheduling) makes LRW responses arrive late or get dropped in
 * the kernel queue, after which every subsequent exchange times out.
 * The self-heal path must detect the stall and drain the wire backlog —
 * delivering queued replies to their live waiters — while the per-send
 * prune touches only the slot the exchange transmits on.  Async slots
 * are never purged.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <cstring>
#include <functional>
#include <thread>

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Sequence;
using namespace std::chrono_literals;

// ============================================================================
// SelfHealMockTransport — same surface as the mock in
// test_logical_address_manager.cpp plus the self-heal hooks
// (drainWire / purgePendingResponses).  NOTE: distinct name required —
// both files link into the same binary and an identical class name would
// be an ODR violation (the linker merges the vtables and the new
// overrides silently resolve to the base no-ops).
// ============================================================================

class SelfHealMockTransport : public IPDOTransport {
public:
    MOCK_METHOD(bool, writeRegister,
                (uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(bool, readRegister,
                (uint16_t adp, uint16_t ado, void* data, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(bool, sendSingleDatagram,
                (Command cmd, uint8_t idx, uint16_t adp, uint16_t ado,
                 const void* data, uint16_t datalen, bool roundtrip),
                (override));
    MOCK_METHOD(size_t, sendMultiDatagram,
                (const MultiDatagramSpec* specs, size_t count),
                (override));
    MOCK_METHOD(bool, waitForResponseIdx,
                (uint8_t idx, unsigned int timeout_ms, RxDatagram& out),
                (override));
    MOCK_METHOD(size_t, preRegisterResponseWaiter,
                (uint8_t idx, uint8_t* buffer, size_t buffer_size),
                (override));
    MOCK_METHOD(bool, waitForPreRegistered,
                (size_t slot, unsigned int timeout_ms, RxDatagram& out),
                (override));
    MOCK_METHOD(uint8_t, allocIdx, (), (override));
    MOCK_METHOD(uint16_t, adpForSlaveIndex, (uint16_t slave_index), (override));
    MOCK_METHOD(int, drainWire, (int max_frames), (override));
    MOCK_METHOD(void, purgePendingResponses, (), (override));
};

// ============================================================================
// Fixture: one slave (4 B RxPDO + 8 B TxPDO), responder lambda lets each
// test script reply/timeout per call.
// ============================================================================

class LRWSelfHealTest : public ::testing::Test {
protected:
    NiceMock<SelfHealMockTransport> transport;
    LogicalAddressManager mgr{transport};
    PDOMapping mapping;

    /// Scripted response generator — tests assign to inject faults.
    std::function<bool(RxDatagram&)> responder{
        [](RxDatagram& out) { out.wkc = 1; out.datalen = 12; return true; }};
    uint8_t next_idx = 1;

    void SetUp() override {
        mgr.init();

        ON_CALL(transport, allocIdx())
            .WillByDefault(Invoke([this] { return next_idx++; }));
        ON_CALL(transport, preRegisterResponseWaiter(_, _, _))
            .WillByDefault(Return(0));
        ON_CALL(transport, waitForPreRegistered(_, _, _))
            .WillByDefault(Invoke(
                [this](size_t, unsigned int, RxDatagram& out) {
                    return responder(out);
                }));
        ON_CALL(transport, waitForResponseIdx(_, _, _))
            .WillByDefault(Invoke(
                [this](uint8_t, unsigned int, RxDatagram& out) {
                    return responder(out);
                }));
        ON_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
            .WillByDefault(Return(true));
        ON_CALL(transport, drainWire(_))
            .WillByDefault(Return(0));
        ON_CALL(transport, readRegister(_, _, _, _, _))
            .WillByDefault(Invoke([](uint16_t, uint16_t, void* data,
                                     uint16_t, unsigned int) {
                std::memset(data, 0x08, 2);  // AL_STATUS = SAFE_OP-ish
                return true;
            }));

        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        mgr.buildAddressMap(configs, 1);

        int rxi = mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        int txi = mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        (void)rxi; (void)txi;
    }

    /// Run a scripted exchange: responder() decides success/timeout.
    bool exchange() { return mgr.exchangeAllLRW(mapping); }
};

// ============================================================================
// Baseline
// ============================================================================

TEST_F(LRWSelfHealTest, CleanExchange_NoSelfHealActivity) {
    // No stall → no backlog flush, and async waiters are NEVER purged.
    // The per-send drainWire(64) is the slot prune: it runs on every
    // exchange (cheap — empty queue is one syscall).
    EXPECT_CALL(transport, purgePendingResponses()).Times(0);
    EXPECT_CALL(transport, drainWire(512)).Times(0);
    EXPECT_CALL(transport, drainWire(64)).Times(2).WillRepeatedly(Return(0));

    ASSERT_TRUE(exchange());
    ASSERT_TRUE(exchange());   // back-to-back — no stall

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.success, 2u);
    EXPECT_EQ(stats.stall_events, 0u);
    EXPECT_EQ(stats.drained_frames, 0u);
    EXPECT_EQ(stats.consecutive_timeouts, 0u);
    EXPECT_EQ(stats.timeout_errors, 0u);
}

// ============================================================================
// Host-stall detection (latency spike)
// ============================================================================

TEST_F(LRWSelfHealTest, LatencySpike_DrainsBeforeNextSend_NoPurge) {
    mgr.setStallDetectionGapUs(1000);  // 1 ms — a 5 ms sleep trips it

    ASSERT_TRUE(exchange());           // seeds last_call_ns_

    std::this_thread::sleep_for(5ms);  // the latency spike

    // Order matters: the stall flush (512) and the per-send prune drain
    // (64) must happen BEFORE the send, so a stale echo is flushed while
    // no waiter for the new idx is pending.  No waiter is ever purged —
    // drained frames for live waiters are DELIVERED, not dropped.
    Sequence seq;
    EXPECT_CALL(transport, drainWire(512)).InSequence(seq)
        .WillOnce(Return(7));
    EXPECT_CALL(transport, drainWire(64)).InSequence(seq);
    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
        .InSequence(seq)
        .WillOnce(Return(true));
    EXPECT_CALL(transport, purgePendingResponses()).Times(0);

    ASSERT_TRUE(exchange());

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.stall_events, 1u);
    EXPECT_EQ(stats.drained_frames, 7u);
    EXPECT_EQ(stats.success, 2u);
    EXPECT_EQ(stats.consecutive_timeouts, 0u);
}

TEST_F(LRWSelfHealTest, GapBelowThreshold_NoStallAction) {
    mgr.setStallDetectionGapUs(60'000'000);  // 60 s — never trips in a test

    ASSERT_TRUE(exchange());
    std::this_thread::sleep_for(5ms);

    EXPECT_CALL(transport, purgePendingResponses()).Times(0);
    EXPECT_CALL(transport, drainWire(512)).Times(0);  // stall flush only
    EXPECT_CALL(transport, drainWire(64)).WillRepeatedly(Return(0));
    ASSERT_TRUE(exchange());

    EXPECT_EQ(mgr.getStats().stall_events, 0u);
}

TEST_F(LRWSelfHealTest, StallDetectionDisabled_NoAction) {
    mgr.setStallDetectionGapUs(0);  // disabled

    ASSERT_TRUE(exchange());
    std::this_thread::sleep_for(5ms);

    EXPECT_CALL(transport, purgePendingResponses()).Times(0);
    EXPECT_CALL(transport, drainWire(512)).Times(0);  // no stall flush
    EXPECT_CALL(transport, drainWire(64)).WillRepeatedly(Return(0));
    ASSERT_TRUE(exchange());
    EXPECT_EQ(mgr.getStats().stall_events, 0u);
}

// ============================================================================
// Timeout path
// ============================================================================

TEST_F(LRWSelfHealTest, Timeout_DrainsWireAndCountsStreak) {
    responder = [](RxDatagram&) { return false; };

    // drainWire(64) runs twice: once as the pre-claim slot prune and once
    // as the post-timeout backlog flush.
    EXPECT_CALL(transport, drainWire(64))
        .Times(2)
        .WillOnce(Return(3))
        .WillOnce(Return(0));

    EXPECT_FALSE(exchange());

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.timeout_errors, 1u);
    EXPECT_EQ(stats.consecutive_timeouts, 1u);
    EXPECT_EQ(stats.drained_frames, 3u);
}

TEST_F(LRWSelfHealTest, TimeoutStreak_RingProbeAtThreshold) {
    mgr.setEscalateAfterTimeouts(3);
    responder = [](RxDatagram&) { return false; };

    // Ring probe = APRD of AL_STATUS (0x0130) on slave 0.
    EXPECT_CALL(transport, readRegister(0, 0x0130, _, 2, _)).Times(1);

    EXPECT_FALSE(exchange());  // 1
    EXPECT_FALSE(exchange());  // 2
    EXPECT_FALSE(exchange());  // 3 → probe fires

    EXPECT_EQ(mgr.getStats().consecutive_timeouts, 3u);
}

TEST_F(LRWSelfHealTest, SuccessResetsTimeoutStreak) {
    int call = 0;
    responder = [&call](RxDatagram& out) {
        ++call;
        if (call <= 2) return false;   // two timeouts, then recover
        out.wkc = 1; out.datalen = 12; return true;
    };

    EXPECT_FALSE(exchange());
    EXPECT_FALSE(exchange());
    EXPECT_EQ(mgr.getStats().consecutive_timeouts, 2u);

    EXPECT_TRUE(exchange());
    EXPECT_EQ(mgr.getStats().consecutive_timeouts, 0u);
    EXPECT_EQ(mgr.getStats().timeout_errors, 2u);
    EXPECT_EQ(mgr.getStats().success, 1u);
}

TEST_F(LRWSelfHealTest, EscalationDisabled_NoRingProbe) {
    mgr.setEscalateAfterTimeouts(0);
    responder = [](RxDatagram&) { return false; };

    EXPECT_CALL(transport, readRegister(_, _, _, _, _)).Times(0);
    for (int i = 0; i < 5; ++i) EXPECT_FALSE(exchange());
    EXPECT_EQ(mgr.getStats().consecutive_timeouts, 5u);
}

// ============================================================================
// End-to-end: latency spike mid-run, then recovery
// ============================================================================

TEST_F(LRWSelfHealTest, LatencySpikeThenTimeoutThenRecovery) {
    mgr.setStallDetectionGapUs(1000);
    mgr.setEscalateAfterTimeouts(64);

    int phase = 0;
    responder = [&phase](RxDatagram& out) {
        ++phase;
        if (phase == 2) return false;  // response lost during the stall
        out.wkc = 1; out.datalen = 12; return true;
    };

    ASSERT_TRUE(exchange());                       // good cycle

    std::this_thread::sleep_for(5ms);              // latency spike

    // Next exchange: stall self-heal drains the backlog (512), then the
    // (still failing) wire produces a timeout — which drains again and
    // counts the streak.  No waiter is ever purged.  drainWire(64) calls:
    // 2 for the failing exchange (pre-claim prune + timeout flush) and 1
    // for the recovering exchange's prune.
    EXPECT_CALL(transport, purgePendingResponses()).Times(0);
    EXPECT_CALL(transport, drainWire(512)).Times(1).WillOnce(Return(4));
    EXPECT_CALL(transport, drainWire(64)).Times(3);
    EXPECT_FALSE(exchange());

    // Wire healed — normal exchange resumes.
    EXPECT_TRUE(exchange());

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.stall_events, 1u);
    EXPECT_EQ(stats.timeout_errors, 1u);
    EXPECT_EQ(stats.consecutive_timeouts, 0u);
    EXPECT_EQ(stats.success, 2u);
    EXPECT_EQ(stats.drained_frames, 4u);
}

// ============================================================================
// Configuration plumbing
// ============================================================================

TEST_F(LRWSelfHealTest, ConfiguredResponseTimeout_UsedInWait) {
    mgr.setLrwResponseTimeoutMs(77);

    EXPECT_CALL(transport, waitForPreRegistered(0, 77, _))
        .WillOnce(Invoke([](size_t, unsigned int, RxDatagram& out) {
            out.wkc = 1; out.datalen = 12; return true;
        }));

    EXPECT_TRUE(exchange());
}

TEST_F(LRWSelfHealTest, ZeroTimeout_ClampedToOne) {
    mgr.setLrwResponseTimeoutMs(0);
    EXPECT_EQ(mgr.lrwResponseTimeoutMs(), 1u);
}

// ============================================================================
// Selective slot prune — busy slots belong to live waiters
// ============================================================================

TEST_F(LRWSelfHealTest, BusySlot_SkipsToNextIdx) {
    // idx 1's slot is owned by a live (async) waiter — the exchange must
    // not steal it; it claims the next free idx instead.
    EXPECT_CALL(transport, preRegisterResponseWaiter(1, _, _))
        .WillOnce(Return(IPDOTransport::kPreRegBusy));
    EXPECT_CALL(transport, preRegisterResponseWaiter(2, _, _))
        .WillOnce(Return(0));

    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 2, _, _, _, _, _))
        .WillOnce(Return(true));

    ASSERT_TRUE(exchange());
    EXPECT_EQ(mgr.getStats().success, 1u);
}

TEST_F(LRWSelfHealTest, AllSlotsBusy_FailsWithoutSending) {
    // Every candidate idx is owned by a live waiter — the exchange must
    // fail WITHOUT sending rather than steal another request's slot.
    ON_CALL(transport, preRegisterResponseWaiter(_, _, _))
        .WillByDefault(Return(IPDOTransport::kPreRegBusy));

    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _)).Times(0);
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _)).Times(0);

    EXPECT_FALSE(exchange());
    EXPECT_EQ(mgr.getStats().send_errors, 1u);
}

TEST_F(LRWSelfHealTest, PreRegUnsupported_FallsBackToWaitForResponseIdx) {
    ON_CALL(transport, preRegisterResponseWaiter(_, _, _))
        .WillByDefault(Return(IPDOTransport::kPreRegInvalid));

    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 1, _, _, _, _, _))
        .WillOnce(Return(true));
    // Fallback waits by idx — the unsupported transport's own mechanism.
    EXPECT_CALL(transport, waitForResponseIdx(1, _, _))
        .WillOnce(Invoke([](uint8_t, unsigned int, RxDatagram& out) {
            out.wkc = 1; out.datalen = 12; return true;
        }));

    EXPECT_TRUE(exchange());
}
