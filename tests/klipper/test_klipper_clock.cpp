/**
 * @file test_klipper_clock.cpp
 * @brief Tests for the clock sync layer: McuClock, ClockSync.
 */

#include <gtest/gtest.h>
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/clock/ClockSync.hpp"

#include <chrono>

using namespace tether::klipper::clock;

TEST(KlipperMcuClock, ForwardAdvance) {
    McuClock mc(1000);
    mc.advanceTo(100);
    EXPECT_EQ(mc.ticks64(), 100ull);
    EXPECT_EQ(mc.ticks32(), 100u);
    mc.advanceTo(200);
    EXPECT_EQ(mc.ticks64(), 200ull);
    EXPECT_EQ(mc.seconds(), 0.2);
}

TEST(KlipperMcuClock, Wraparound) {
    McuClock mc(1000);
    mc.advanceTo(0xFFFFFFF0u);
    EXPECT_EQ(mc.ticks32(), 0xFFFFFFF0u);
    mc.advanceTo(0x10); // wraps
    // 0x10 - 0xFFFFFFF0 = 0x20 (unsigned), so ticks64 = 0xFFFFFFF0 + 0x20 = 0x100000010
    EXPECT_EQ(mc.ticks64(), 0x100000010ull);
    EXPECT_EQ(mc.ticks32(), 0x10u);
}

TEST(KlipperMcuClock, ToTicks64) {
    McuClock mc(1000);
    mc.advanceTo(1000);
    EXPECT_EQ(mc.toTicks64(1050), 1050ull);
    EXPECT_EQ(mc.toTicks64(990), 990ull);
}

TEST(KlipperClockSync, BasicFit) {
    ClockSync cs(0.1);
    auto t0 = HostClock::now();
    for (int i = 0; i < 5; ++i) {
        auto send = t0 + std::chrono::milliseconds(i * 100);
        auto recv = t0 + std::chrono::milliseconds(i * 100 + 1);
        cs.addSample(send, recv, i * 1000);
    }
    EXPECT_TRUE(cs.isSynchronised());
    EXPECT_NEAR(cs.slope(), 10000.0, 100.0); // 10000 ticks/sec
}

TEST(KlipperClockSync, NotSyncedWithOneSample) {
    ClockSync cs(0.1);
    auto t0 = HostClock::now();
    cs.addSample(t0, t0, 0);
    EXPECT_FALSE(cs.isSynchronised());
}
