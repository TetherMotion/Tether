/**
 * @file test_hal_common_extra.cpp
 * @brief Coverage for EthernetHAL_Common.cpp (buildEthernetHeader,
 *        getEtherType, createMockHAL through IEthernetHAL), the real
 *        StateMachineLoggerImpl via createStateMachineLogger(), and
 *        InterfaceHelpers (resolveInterface passthrough / list helpers).
 */

#include <gtest/gtest.h>

#include <unistd.h>

#include <array>
#include <atomic>
#include <cstring>
#include <memory>
#include <vector>

#include "tether/hal/EthernetHAL.hpp"
#include "tether/hal/HALTypes.hpp"
#include "tether/hal/IClock.hpp"
#include "tether/hal/ILogger.hpp"
#include "tether/hal/InterfaceHelpers.hpp"
#include "tether/hal/StateMachineLogger.hpp"

namespace {

using namespace EtherCAT::HAL;

// ---------------------------------------------------------------------------
// buildEthernetHeader / getEtherType / isEtherCATFrame
// ---------------------------------------------------------------------------

TEST(EthernetHeaderTest, BuildRoundTrips) {
    const uint8_t dst[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t src[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint8_t buf[14]{};

    buildEthernetHeader(buf, dst, src, kEtherTypeEtherCAT);
    EXPECT_EQ(std::memcmp(buf, dst, 6), 0);
    EXPECT_EQ(std::memcmp(buf + 6, src, 6), 0);
    EXPECT_EQ(getEtherType(buf), kEtherTypeEtherCAT);
    EXPECT_TRUE(isEtherCATFrame(buf));
}

TEST(EthernetHeaderTest, NonEtherCATFrameDetected) {
    const uint8_t dst[6] = {0, 0, 0, 0, 0, 0};
    const uint8_t src[6] = {0, 0, 0, 0, 0, 0};
    uint8_t buf[14]{};
    buildEthernetHeader(buf, dst, src, 0x0800);  // IPv4
    EXPECT_EQ(getEtherType(buf), 0x0800);
    EXPECT_FALSE(isEtherCATFrame(buf));
}

// ---------------------------------------------------------------------------
// createMockHAL → IEthernetHAL interface
// ---------------------------------------------------------------------------

class MockHalInterfaceTest : public ::testing::Test {
protected:
    std::unique_ptr<IEthernetHAL> hal_ = createMockHAL();
};

TEST_F(MockHalInterfaceTest, InitShutdownLifecycle) {
    EXPECT_FALSE(hal_->isInitialized());
    EXPECT_TRUE(hal_->init("any"));
    EXPECT_TRUE(hal_->isInitialized());
    hal_->shutdown();
    EXPECT_FALSE(hal_->isInitialized());
}

TEST_F(MockHalInterfaceTest, DefaultMacAddress) {
    ASSERT_TRUE(hal_->init("x"));
    uint8_t mac[6]{};
    EXPECT_TRUE(hal_->getMacAddress(mac));
    EXPECT_EQ(mac[0], 0x02);  // locally administered
    EXPECT_EQ(mac[5], 0x01);
    hal_->shutdown();
    EXPECT_FALSE(hal_->getMacAddress(mac));
}

TEST_F(MockHalInterfaceTest, SetMacAddress) {
    ASSERT_TRUE(hal_->init("x"));
    const uint8_t custom[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    EXPECT_TRUE(hal_->setMacAddress(custom));
    uint8_t mac[6]{};
    EXPECT_TRUE(hal_->getMacAddress(mac));
    EXPECT_EQ(std::memcmp(mac, custom, 6), 0);
}

TEST_F(MockHalInterfaceTest, TransmitTracksStats) {
    ASSERT_TRUE(hal_->init("x"));
    uint8_t frame[64]{};
    EXPECT_TRUE(hal_->transmit(frame, sizeof(frame)));
    EXPECT_TRUE(hal_->transmit(frame, 100));
    auto s = hal_->getStats();
    EXPECT_EQ(s.tx_frames, 2u);
    EXPECT_EQ(s.tx_bytes, 64u + 100u);

    hal_->resetStats();
    s = hal_->getStats();
    EXPECT_EQ(s.tx_frames, 0u);
    EXPECT_EQ(s.tx_bytes, 0u);
}

TEST_F(MockHalInterfaceTest, TransmitRejectsBadSizes) {
    ASSERT_TRUE(hal_->init("x"));
    uint8_t frame[64]{};
    EXPECT_FALSE(hal_->transmit(frame, 10));   // below min frame size
    std::vector<uint8_t> big(2000, 0);
    EXPECT_FALSE(hal_->transmit(big.data(), big.size()));  // over max
    EXPECT_EQ(hal_->getStats().tx_frames, 0u);
}

TEST_F(MockHalInterfaceTest, TransmitBeforeInitFails) {
    uint8_t frame[64]{};
    EXPECT_FALSE(hal_->transmit(frame, sizeof(frame)));
}

TEST_F(MockHalInterfaceTest, RxCallbackNotFiredWithoutLoopback) {
    ASSERT_TRUE(hal_->init("x"));
    int rxCount = 0;
    hal_->setRxCallback(
        [](const uint8_t*, size_t, void* ud) {
            ++*static_cast<int*>(ud);
        },
        &rxCount);
    uint8_t frame[64]{};
    EXPECT_TRUE(hal_->transmit(frame, sizeof(frame)));
    EXPECT_EQ(rxCount, 0);  // loopback off → no RX delivery
}

TEST_F(MockHalInterfaceTest, EtherTypeFilterAndPromiscuous) {
    ASSERT_TRUE(hal_->init("x"));
    hal_->setEtherTypeFilter(kEtherTypeEtherCAT);  // accepted silently
    EXPECT_TRUE(hal_->setPromiscuous(true));
    EXPECT_TRUE(hal_->setPromiscuous(false));
}

TEST_F(MockHalInterfaceTest, PollReturnsImmediately) {
    ASSERT_TRUE(hal_->init("x"));
    EXPECT_EQ(hal_->poll(0), 0);
    EXPECT_EQ(hal_->poll(100), 0);
}

// ---------------------------------------------------------------------------
// StateMachineLoggerImpl via createStateMachineLogger()
// ---------------------------------------------------------------------------

class StateMachineLoggerImplTest : public ::testing::Test {
protected:
    std::unique_ptr<IStateMachineLogger> logger_ = createStateMachineLogger();
    void SetUp() override {
        ASSERT_EQ(logger_->init(StateMachineLoggerConfig{}), Error::OK);
    }
};

TEST_F(StateMachineLoggerImplTest, RecordAndQueryHistory) {
    logger_->recordTransition(0, ALState::Init, ALState::PreOp, 0, true);
    logger_->recordTransition(0, ALState::PreOp, ALState::SafeOp, 0, true);
    logger_->recordTransition(1, ALState::Init, ALState::PreOp, 0, true);

    auto h0 = logger_->getHistory(0);
    ASSERT_EQ(h0.size(), 2u);
    EXPECT_EQ(h0[0].previousState, ALState::Init);
    EXPECT_EQ(h0[0].newState, ALState::PreOp);
    EXPECT_TRUE(h0[0].requested);
    EXPECT_EQ(h0[1].newState, ALState::SafeOp);

    auto all = logger_->getAllHistory();
    EXPECT_EQ(all.size(), 3u);
    EXPECT_EQ(logger_->getSlaveCount(), 2u);
    EXPECT_EQ(logger_->getCurrentState(0), ALState::SafeOp);
    EXPECT_EQ(logger_->getCurrentState(1), ALState::PreOp);
    EXPECT_EQ(logger_->getCurrentState(99), ALState::Unknown);
}

TEST_F(StateMachineLoggerImplTest, StatsClassifyTransitions) {
    // error transition (next state has error bit)
    logger_->recordTransition(0, ALState::Op, ALState::SafeOpError,
                              0x001B, false);
    // recovery (error → non-error)
    logger_->recordTransition(0, ALState::SafeOpError, ALState::SafeOp, 0,
                              true);
    // downgrade (Op → Init)
    logger_->recordTransition(0, ALState::Op, ALState::Init, 0, false);

    auto s = logger_->getStats();
    EXPECT_EQ(s.totalTransitions, 3u);
    EXPECT_GE(s.errorTransitions, 1u);
    EXPECT_GE(s.recoveryTransitions, 1u);
    EXPECT_GE(s.downgradeTransitions, 1u);
}

TEST_F(StateMachineLoggerImplTest, CallbackReceivesTransitions) {
    std::vector<ALState> seen;
    logger_->setCallback([&seen](const StateTransition& t) {
        seen.push_back(t.newState);
    });
    logger_->recordTransition(2, ALState::Init, ALState::PreOp, 0, true);
    ASSERT_EQ(seen.size(), 1u);
    EXPECT_EQ(seen[0], ALState::PreOp);
    logger_->setCallback(nullptr);
    logger_->recordTransition(2, ALState::PreOp, ALState::Op, 0, true);
    EXPECT_EQ(seen.size(), 1u);  // cleared → no more calls
}

TEST_F(StateMachineLoggerImplTest, ClearHistoryClearsOnlyHistory) {
    // clearHistory drops the recorded transition log but keeps current
    // state tracking and cumulative stats — matches impl semantics.
    logger_->recordTransition(0, ALState::Init, ALState::PreOp, 0, true);
    logger_->clearHistory();
    EXPECT_TRUE(logger_->getAllHistory().empty());
    EXPECT_TRUE(logger_->getHistory(0).empty());
    EXPECT_EQ(logger_->getCurrentState(0), ALState::PreOp);  // retained
    EXPECT_EQ(logger_->getSlaveCount(), 1u);                 // retained
    EXPECT_EQ(logger_->getStats().totalTransitions, 1u);     // retained
}

TEST_F(StateMachineLoggerImplTest, HistorySizeIsBounded) {
    StateMachineLoggerConfig cfg;
    cfg.historySize = 4;
    ASSERT_EQ(logger_->init(cfg), Error::OK);
    for (int i = 0; i < 10; ++i)
        logger_->recordTransition(0, ALState::Init, ALState::PreOp, 0, true);
    EXPECT_LE(logger_->getHistory(0).size(), 4u);
}

TEST_F(StateMachineLoggerImplTest, SetLogLevelAccepted) {
    logger_->setLogLevel(LogLevel::Error);
    logger_->setLogLevel(LogLevel::Debug);
    SUCCEED();
}

TEST_F(StateMachineLoggerImplTest, StructTransitionDirectRecord) {
    StateTransition t;
    t.slaveIndex = 5;
    t.previousState = ALState::PreOp;
    t.newState = ALState::SafeOp;
    t.alStatusCode = 0x0000;
    t.requested = true;
    logger_->recordTransition(t);
    EXPECT_EQ(logger_->getCurrentState(5), ALState::SafeOp);
    ASSERT_EQ(logger_->getHistory(5).size(), 1u);
    EXPECT_EQ(logger_->getHistory(5)[0].slaveIndex, 5);
}

// ---------------------------------------------------------------------------
// InterfaceHelpers — safe subset (failure paths call exit(1))
// ---------------------------------------------------------------------------

TEST(InterfaceHelpersTest, ResolvePassthroughReturnsRequested) {
    EXPECT_EQ(resolveInterface("eth9", "test"), "eth9");
    EXPECT_EQ(resolveInterface("lo", "test"), "lo");
}

TEST(InterfaceHelpersTest, ListPhysicalInterfacesDoesNotCrash) {
    listPhysicalInterfaces("test-hal");
    SUCCEED();
}

// ---------------------------------------------------------------------------
// LinuxClock via getSystemClock() (real CLOCK_MONOTONIC/CLOCK_REALTIME)
// ---------------------------------------------------------------------------

class LinuxClockTest : public ::testing::Test {
protected:
    IClock& clock_ = getSystemClock();
};

TEST_F(LinuxClockTest, MonotonicTimeAdvances) {
    const Timestamp a = clock_.nowMicros();
    const Timestamp b = clock_.nowMicros();
    EXPECT_GE(b, a);  // CLOCK_MONOTONIC never goes backward
    EXPECT_GT(a, 0u);
}

TEST_F(LinuxClockTest, NowNanosConsistentWithMicros) {
    const Timestamp us = clock_.nowMicros();
    const Timestamp ns = clock_.nowNanos();
    // ns/1000 should be >= us (ns sampled later or equal).
    EXPECT_GE(ns / 1000, us);
    // And within a generous bound (clock calls are fast).
    EXPECT_LT(ns / 1000 - us, 100000u);  // <100ms skew
}

TEST_F(LinuxClockTest, SystemTimeMillisIsWallClock) {
    const Timestamp t = clock_.systemTimeMillis();
    // Wall clock well past epoch (>2020).
    EXPECT_GT(t, 1577836800000u);
}

TEST_F(LinuxClockTest, ResolutionIsPositive) {
    EXPECT_GT(clock_.resolution(), 0u);
    EXPECT_LT(clock_.resolution(), 1000000u);  // <1ms resolution
}

TEST_F(LinuxClockTest, DelayMicrosActuallyWaits) {
    const Timestamp before = clock_.nowMicros();
    clock_.delayMicros(500);   // nanosleep path
    const Timestamp after = clock_.nowMicros();
    EXPECT_GE(after - before, 500u);
}

TEST_F(LinuxClockTest, DelayMicrosBusyWaitPath) {
    const Timestamp before = clock_.nowMicros();
    clock_.delayMicros(50);    // <100us → busy-wait path
    const Timestamp after = clock_.nowMicros();
    EXPECT_GE(after - before, 50u);
}

TEST_F(LinuxClockTest, DelayMicrosZeroIsNoop) {
    const Timestamp before = clock_.nowMicros();
    clock_.delayMicros(0);
    clock_.delayMillis(0);
    const Timestamp after = clock_.nowMicros();
    EXPECT_LT(after - before, 50000u);  // returns immediately
}

TEST_F(LinuxClockTest, DelayMillisWaits) {
    const Timestamp before = clock_.nowMicros();
    clock_.delayMillis(2);
    const Timestamp after = clock_.nowMicros();
    EXPECT_GE(after - before, 2000u);
}

// ---------------------------------------------------------------------------
// Clock factory lifecycle
// ---------------------------------------------------------------------------

TEST(ClockFactoryTest, GetClockFactoryReturnsSameInstance) {
    IClockFactory& a = getClockFactory();
    IClockFactory& b = getClockFactory();
    EXPECT_EQ(&a, &b);
}

TEST(ClockFactoryTest, GetSystemClockIsSingleton) {
    IClock& a = getSystemClock();
    IClock& b = getSystemClock();
    EXPECT_EQ(&a, &b);
}

TEST(ClockFactoryTest, ResetRecreatesFactory) {
    // Address comparison is unreliable — the allocator may hand back the
    // same block. Verify reset leaves a working factory instead.
    getClockFactory();
    resetClockFactory();
    IClockFactory& f = getClockFactory();
    EXPECT_NE(f.createPeriodicTimer(), nullptr);
    EXPECT_NE(f.createOneShotTimer(), nullptr);
    EXPECT_GT(f.getSystemClock().nowMicros(), 0u);
}

// ---------------------------------------------------------------------------
// LinuxPeriodicTimer (timerfd) and LinuxOneShotTimer (thread-based)
// ---------------------------------------------------------------------------

TEST(LinuxPeriodicTimerTest, InitStartStopCycles) {
    auto timer = getClockFactory().createPeriodicTimer();
    ASSERT_NE(timer, nullptr);
    EXPECT_TRUE(timer->init(1000));  // 1kHz → 1ms period
    EXPECT_FALSE(timer->isRunning());
    timer->start();
    EXPECT_TRUE(timer->isRunning());
    timer->waitForCycle();
    timer->waitForCycle();
    auto stats = timer->getStats();
    EXPECT_GE(stats.tickCount, 1u);
    timer->stop();
    EXPECT_FALSE(timer->isRunning());
}

TEST(LinuxPeriodicTimerTest, CallbackFiresOnCycles) {
    auto timer = getClockFactory().createPeriodicTimer();
    ASSERT_TRUE(timer->init(1000));
    std::atomic<int> ticks{0};
    timer->setCallback([&ticks] { ++ticks; });
    timer->start();
    for (int i = 0; i < 3; ++i) timer->waitForCycle();
    timer->stop();
    EXPECT_GE(ticks.load(), 1);
}

TEST(LinuxPeriodicTimerTest, GetPeriodAndResetStats) {
    auto timer = getClockFactory().createPeriodicTimer();
    ASSERT_TRUE(timer->init(2000));  // 2kHz → 500us
    EXPECT_EQ(timer->getPeriodMicros(), 500);
    timer->start();
    timer->waitForCycle();
    timer->waitForCycle();
    EXPECT_GT(timer->getStats().tickCount, 0u);
    timer->resetStats();
    EXPECT_EQ(timer->getStats().tickCount, 0u);
    timer->stop();
}

TEST(LinuxPeriodicTimerTest, InvalidFrequencyRejected) {
    auto timer = getClockFactory().createPeriodicTimer();
    EXPECT_FALSE(timer->init(0));
}

TEST(LinuxOneShotTimerTest, FiresCallbackAfterDelay) {
    auto timer = getClockFactory().createOneShotTimer();
    ASSERT_NE(timer, nullptr);
    std::atomic<bool> fired{false};
    EXPECT_TRUE(timer->start(2000, [&fired] { fired = true; }));
    EXPECT_TRUE(timer->isPending());
    // Wait for expiry.
    for (int i = 0; i < 100 && !fired.load(); ++i) usleep(1000);
    EXPECT_TRUE(fired.load());
    EXPECT_FALSE(timer->isPending());
}

TEST(LinuxOneShotTimerTest, CancelPreventsCallback) {
    auto timer = getClockFactory().createOneShotTimer();
    std::atomic<bool> fired{false};
    EXPECT_TRUE(timer->start(50000, [&fired] { fired = true; }));
    EXPECT_TRUE(timer->isPending());
    EXPECT_TRUE(timer->cancel());
    EXPECT_FALSE(timer->isPending());
    usleep(60000);
    EXPECT_FALSE(fired.load());
}

TEST(LinuxOneShotTimerTest, RemainingCountsDown) {
    auto timer = getClockFactory().createOneShotTimer();
    timer->start(50000, [] {});
    const Microseconds r = timer->remaining();
    EXPECT_GT(r, 0);
    EXPECT_LE(r, 50000);
    timer->cancel();
    EXPECT_EQ(timer->remaining(), 0);
}

TEST(LinuxOneShotTimerTest, CancelWhenNotPendingReturnsFalse) {
    auto timer = getClockFactory().createOneShotTimer();
    EXPECT_FALSE(timer->isPending());
    EXPECT_FALSE(timer->cancel());
}

}  // namespace
