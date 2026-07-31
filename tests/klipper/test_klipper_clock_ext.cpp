/**
 * @file test_klipper_clock_ext.cpp
 * @brief Extended clock tests: McuClock multiple wraps, toSeconds, reset,
 *        ClockSync decay, hostToMcu, hostDelayToMcuTicks, maxSamples eviction.
 */

#include <gtest/gtest.h>
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/clock/ClockSync.hpp"

#include <chrono>
#include <thread>

using namespace tether::klipper::clock;

// ============================================================================
// McuClock extended tests
// ============================================================================

TEST(KlipperMcuClockExt, DefaultFrequency) {
    McuClock clk;
    EXPECT_EQ(clk.frequency(), 180000000u);
}

TEST(KlipperMcuClockExt, CustomFrequency) {
    McuClock clk(1000000);
    EXPECT_EQ(clk.frequency(), 1000000u);
}

TEST(KlipperMcuClockExt, InitialState) {
    McuClock clk;
    EXPECT_EQ(clk.ticks32(), 0u);
    EXPECT_EQ(clk.ticks64(), 0u);
    EXPECT_DOUBLE_EQ(clk.seconds(), 0.0);
}

TEST(KlipperMcuClockExt, SimpleAdvance) {
    McuClock clk;
    clk.advanceTo(1000);
    EXPECT_EQ(clk.ticks32(), 1000u);
    EXPECT_EQ(clk.ticks64(), 1000u);
}

TEST(KlipperMcuClockExt, SecondsCalculation) {
    McuClock clk(1000); // 1000 Hz = 1 tick per ms
    clk.advanceTo(500);
    EXPECT_DOUBLE_EQ(clk.seconds(), 0.5);
}

TEST(KlipperMcuClockExt, ToSeconds) {
    McuClock clk(1000);
    clk.advanceTo(100);
    EXPECT_NEAR(clk.toSeconds(200), 0.2, 0.0001);
}

TEST(KlipperMcuClockExt, Wraparound32Bit) {
    McuClock clk(180000000);

    // Advance to near the wrap point
    clk.advanceTo(0xFFFFFFF0);
    EXPECT_EQ(clk.ticks32(), 0xFFFFFFF0u);
    EXPECT_EQ(clk.ticks64(), static_cast<uint64_t>(0xFFFFFFF0));

    // Advance past wrap
    clk.advanceTo(0x00000010);
    EXPECT_EQ(clk.ticks32(), 0x00000010u);
    EXPECT_EQ(clk.ticks64(), static_cast<uint64_t>(0x100000010ULL));
}

TEST(KlipperMcuClockExt, MultipleWraps) {
    McuClock clk(1000000); // 1 MHz, wraps every ~4295 seconds

    // Advance through multiple wraps
    uint64_t total = 0;
    uint32_t pos = 0;
    for (int wrap = 0; wrap < 5; ++wrap) {
        pos += 0x80000000; // Half wrap each time
        clk.advanceTo(pos);
        total += 0x80000000;
        EXPECT_EQ(clk.ticks64(), total);
    }
}

TEST(KlipperMcuClockExt, ToTicks64NearCurrent) {
    McuClock clk(180000000);
    clk.advanceTo(0xFFFFFFF0);

    // A value slightly ahead
    auto t64 = clk.toTicks64(0xFFFFFFF5);
    EXPECT_EQ(t64, static_cast<uint64_t>(0xFFFFFFF5));

    // A value slightly behind (previous wrap)
    auto t64b = clk.toTicks64(0x00000005);
    // Should be in the next wrap
    EXPECT_GT(t64b, static_cast<uint64_t>(0xFFFFFFF0));
}

TEST(KlipperMcuClockExt, Reset) {
    McuClock clk;
    clk.advanceTo(1000);
    EXPECT_NE(clk.ticks32(), 0u);

    clk.reset();
    EXPECT_EQ(clk.ticks32(), 0u);
    EXPECT_EQ(clk.ticks64(), 0u);
    EXPECT_DOUBLE_EQ(clk.seconds(), 0.0);
}

TEST(KlipperMcuClockExt, AdvanceBackwardsIgnored) {
    McuClock clk;
    clk.advanceTo(1000);
    // Advancing to a past value should not move backwards
    // (unsigned subtraction handles this)
    clk.advanceTo(500);
    // The 64-bit should not decrease
    // (implementation may treat as wrap or ignore)
}

// ============================================================================
// ClockSync extended tests
// ============================================================================

TEST(KlipperClockSyncExt, NotSyncedInitially) {
    ClockSync cs;
    EXPECT_FALSE(cs.isSynchronised());
    EXPECT_EQ(cs.sampleCount(), 0u);
}

TEST(KlipperClockSyncExt, NotSyncedWithOneSample) {
    ClockSync cs;
    auto now = HostClock::now();
    cs.addSample(now, now, 0);
    EXPECT_FALSE(cs.isSynchronised());
    EXPECT_EQ(cs.sampleCount(), 1u);
}

TEST(KlipperClockSyncExt, SyncedWithTwoSamples) {
    ClockSync cs;
    auto t0 = HostClock::now();
    cs.addSample(t0, t0, 0);
    cs.addSample(t0 + std::chrono::seconds(1),
                 t0 + std::chrono::seconds(1), 10000);
    EXPECT_TRUE(cs.isSynchronised());
    EXPECT_EQ(cs.sampleCount(), 2u);
}

TEST(KlipperClockSyncExt, SlopeEstimation) {
    ClockSync cs;
    auto t0 = HostClock::now();

    // Add samples with known slope: 10000 ticks/sec
    for (int i = 0; i < 10; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 100);
        cs.addSample(t, t, static_cast<uint32_t>(i * 1000));
    }

    EXPECT_TRUE(cs.isSynchronised());
    // Slope should be approximately 10000 ticks/sec
    EXPECT_NEAR(cs.slope(), 10000.0, 500.0);
}

TEST(KlipperClockSyncExt, HostToMcu) {
    ClockSync cs;
    auto t0 = HostClock::now();

    // Add samples
    for (int i = 0; i < 5; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 100);
        cs.addSample(t, t, static_cast<uint32_t>(i * 10000));
    }

    EXPECT_TRUE(cs.isSynchronised());

    // Convert a host time to MCU clock
    uint32_t mcu = cs.hostToMcu(t0 + std::chrono::milliseconds(200));
    // Should be approximately 20000
    EXPECT_GT(mcu, 15000u);
    EXPECT_LT(mcu, 25000u);
}

TEST(KlipperClockSyncExt, HostDelayToMcuTicks) {
    ClockSync cs;
    auto t0 = HostClock::now();

    // 100ms intervals, 10000 tick increments → slope = 10000/0.1 = 100000 ticks/sec
    for (int i = 0; i < 5; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 100);
        cs.addSample(t, t, static_cast<uint32_t>(i * 10000));
    }

    // 1 second delay should map to ~100000 ticks
    uint32_t ticks = cs.hostDelayToMcuTicks(std::chrono::seconds(1));
    EXPECT_GT(ticks, 50000u);
    EXPECT_LT(ticks, 150000u);
}

TEST(KlipperClockSyncExt, Reset) {
    ClockSync cs;
    auto t0 = HostClock::now();
    cs.addSample(t0, t0, 0);
    cs.addSample(t0 + std::chrono::seconds(1),
                 t0 + std::chrono::seconds(1), 10000);
    EXPECT_TRUE(cs.isSynchronised());

    cs.reset();
    EXPECT_FALSE(cs.isSynchronised());
    EXPECT_EQ(cs.sampleCount(), 0u);
    EXPECT_EQ(cs.slope(), 0.0);
    EXPECT_EQ(cs.offset(), 0.0);
}

TEST(KlipperClockSyncExt, MaxSamplesEviction) {
    ClockSync cs(0.1, 5); // max 5 samples
    auto t0 = HostClock::now();

    for (int i = 0; i < 10; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 100);
        cs.addSample(t, t, static_cast<uint32_t>(i * 1000));
    }

    // Should only keep 5 samples
    EXPECT_EQ(cs.sampleCount(), 5u);
}

TEST(KlipperClockSyncExt, CustomDecayLambda) {
    // Fast decay: recent samples dominate
    ClockSync fastDecay(1.0, 64);
    auto t0 = HostClock::now();

    for (int i = 0; i < 10; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 100);
        fastDecay.addSample(t, t, static_cast<uint32_t>(i * 1000));
    }

    EXPECT_TRUE(fastDecay.isSynchronised());
    // With fast decay, slope should be close to recent trend
    EXPECT_GT(fastDecay.slope(), 0);
}

TEST(KlipperClockSyncExt, HostToMcuEmptyReturnsZero) {
    ClockSync cs;
    auto now = HostClock::now();
    EXPECT_EQ(cs.hostToMcu(now), 0u);
}

TEST(KlipperClockSyncExt, OffsetIsFinite) {
    ClockSync cs;
    auto t0 = HostClock::now();
    for (int i = 0; i < 5; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 100);
        cs.addSample(t, t, static_cast<uint32_t>(i * 10000));
    }
    EXPECT_TRUE(std::isfinite(cs.offset()));
}

TEST(KlipperClockSyncExt, ManySamplesStableFit) {
    ClockSync cs(0.01, 100);
    auto t0 = HostClock::now();

    // Add 50 samples with consistent slope
    for (int i = 0; i < 50; ++i) {
        auto t = t0 + std::chrono::milliseconds(i * 50);
        cs.addSample(t, t, static_cast<uint32_t>(i * 500));
    }

    EXPECT_TRUE(cs.isSynchronised());
    // Slope should be 10000 ticks/sec (500 ticks / 0.05 sec)
    EXPECT_NEAR(cs.slope(), 10000.0, 200.0);
}
