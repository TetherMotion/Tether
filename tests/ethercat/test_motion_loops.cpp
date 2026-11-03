/**
 * @file test_motion_loops.cpp
 * @brief Coverage for the motion-control loop factories — polling,
 *        realtime and queue wrappers over the real loop classes.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "mocks/EmulatedBus.hpp"
#include "ethercat/raw/MotionLoops.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class MotionLoopsTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "esc211");
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    static void waitCycles(int n) {
        std::this_thread::sleep_for(std::chrono::milliseconds(n));
    }

    EmulatedBus bus_;
    Master master_;
};

}  // namespace

// ============================================================================
// Polling loop
// ============================================================================

TEST_F(MotionLoopsTest, PollingLoopRunsCallbackUntilStopped) {
    std::atomic<int> ticks{0};
    PollingMotionLoopConfig cfg;
    cfg.cycle_period_us = 1000;
    cfg.request_realtime_priority = false;

    auto loop = makePollingMotionControlLoop(
        [&](double dt) {
            EXPECT_GT(dt, 0.0);
            ++ticks;
            return true;
        },
        cfg, nullptr);

    ASSERT_TRUE(loop->start());
    EXPECT_FALSE(loop->start());  // already running
    waitCycles(60);
    EXPECT_TRUE(loop->isRunning());
    loop->stop();
    EXPECT_FALSE(loop->isRunning());
    EXPECT_GT(ticks.load(), 0);
}

TEST_F(MotionLoopsTest, PollingLoopStopsWhenCallbackReturnsFalse) {
    std::atomic<int> ticks{0};
    PollingMotionLoopConfig cfg;
    cfg.cycle_period_us = 1000;
    cfg.request_realtime_priority = false;

    auto loop = makePollingMotionControlLoop(
        [&](double) {
            ++ticks;
            return ticks.load() < 3;
        },
        cfg, nullptr);

    ASSERT_TRUE(loop->start());
    waitCycles(100);
    EXPECT_FALSE(loop->isRunning());
    EXPECT_EQ(ticks.load(), 3);
    loop->stop();
}

TEST_F(MotionLoopsTest, PollingLoopSetShutdownDebugIsSafe) {
    PollingMotionLoopConfig cfg;
    cfg.request_realtime_priority = false;
    auto loop = makePollingMotionControlLoop([](double) { return true; },
                                             cfg, nullptr);
    loop->setShutdownDebug(true);
    loop->setShutdownDebug(false);
    loop->stop();  // not started → no-op
}

// ============================================================================
// Realtime loop
// ============================================================================

TEST_F(MotionLoopsTest, RealtimeLoopStartsAndStops) {
    std::atomic<int> ticks{0};
    RealtimeMotionLoopConfig cfg;
    cfg.cycle_period_us = 1000;

    auto loop = makeRealtimeMotionControlLoop(
        [&](double) {
            ++ticks;
            return true;
        },
        cfg, nullptr);

    if (!loop->start()) {
        GTEST_SKIP() << "realtime loop unavailable in this environment";
    }
    waitCycles(60);
    loop->stop();
    EXPECT_FALSE(loop->isRunning());
    EXPECT_GT(ticks.load(), 0);
}

TEST_F(MotionLoopsTest, RealtimeLoopStopsOnCallbackFalse) {
    std::atomic<int> ticks{0};
    RealtimeMotionLoopConfig cfg;
    cfg.cycle_period_us = 1000;

    auto loop = makeRealtimeMotionControlLoop(
        [&](double) {
            ++ticks;
            return ticks.load() < 2;
        },
        cfg, nullptr);

    if (!loop->start()) {
        GTEST_SKIP() << "realtime loop unavailable in this environment";
    }
    waitCycles(100);
    loop->stop();
    EXPECT_FALSE(loop->isRunning());
}

TEST_F(MotionLoopsTest, RealtimeLoopSetShutdownDebugIsSafe) {
    RealtimeMotionLoopConfig cfg;
    auto loop = makeRealtimeMotionControlLoop([](double) { return true; },
                                              cfg, nullptr);
    loop->setShutdownDebug(true);
    loop->setShutdownDebug(false);
    loop->stop();
}

// ============================================================================
// Queue loop
// ============================================================================

TEST_F(MotionLoopsTest, QueueLoopRunsQueueCycles) {
    RealtimeMotionLoopConfig cfg;
    cfg.cycle_period_us = 1000;

    auto loop = makeQueueMotionControlLoop(&master_.pdoForSlave(0), cfg,
                                           nullptr);
    if (!loop->start()) {
        GTEST_SKIP() << "realtime loop unavailable in this environment";
    }
    waitCycles(60);
    loop->stop();
    EXPECT_FALSE(loop->isRunning());
    loop->setShutdownDebug(false);
}

TEST_F(MotionLoopsTest, QueueLoopWithoutPdoManagerIsInert) {
    RealtimeMotionLoopConfig cfg;
    cfg.cycle_period_us = 1000;

    auto loop = makeQueueMotionControlLoop(nullptr, cfg, nullptr);
    // Without a PDOManager every cycle reports failure; the loop must not
    // crash whether or not the realtime thread starts.
    loop->start();
    waitCycles(30);
    loop->stop();
    EXPECT_FALSE(loop->isRunning());
}
