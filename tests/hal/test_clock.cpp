/**
 * @file test_clock.cpp
 * @brief Unit tests for Clock HAL
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "tether/hal/IClock.hpp"
#include "MockHAL.hpp"

using namespace EtherCAT::HAL;
using namespace EtherCAT::HAL::mock;

// ============================================================================
// FakeClock Tests
// ============================================================================

TEST(FakeClockTest, InitialTime) {
    FakeClock clock;
    
    // Initially time is 0
    EXPECT_EQ(clock.nowMicros(), 0u);
}

TEST(FakeClockTest, SetTime) {
    FakeClock clock;
    
    clock.setTime(1000000);
    EXPECT_EQ(clock.nowMicros(), 1000000u);
}

TEST(FakeClockTest, Advance) {
    FakeClock clock;
    
    clock.setTime(1000);
    clock.advance(500);
    EXPECT_EQ(clock.nowMicros(), 1500u);
}

TEST(FakeClockTest, NowNanos) {
    FakeClock clock;
    
    clock.setTime(1000);  // 1000 microseconds
    EXPECT_EQ(clock.nowNanos(), 1000000u);  // 1000000 nanoseconds
}

TEST(FakeClockTest, DelayMicros) {
    FakeClock clock;
    
    clock.setTime(0);
    clock.delayMicros(500);
    EXPECT_EQ(clock.nowMicros(), 500u);
}

TEST(FakeClockTest, DelayMillis) {
    FakeClock clock;
    
    clock.setTime(0);
    clock.delayMillis(10);
    EXPECT_EQ(clock.nowMicros(), 10000u);
}

TEST(FakeClockTest, Resolution) {
    FakeClock clock;
    
    // FakeClock has 1us resolution
    EXPECT_EQ(clock.resolution(), 1000u);  // 1000ns = 1us
}

TEST(FakeClockTest, SystemTime) {
    FakeClock clock;
    
    Timestamp sysTime = clock.systemTimeMillis();
    EXPECT_GT(sysTime, 0u);  // Should be non-zero
    
    clock.setSystemTime(2000000000000);  // Set to some future time
    EXPECT_EQ(clock.systemTimeMillis(), 2000000000000u);
}

// ============================================================================
// MockClock Tests  
// ============================================================================

TEST(MockClockTest, ExpectCalls) {
    MockClock mockClock;
    
    EXPECT_CALL(mockClock, nowMicros())
        .WillOnce(testing::Return(1000))
        .WillOnce(testing::Return(2000));
    
    EXPECT_EQ(mockClock.nowMicros(), 1000u);
    EXPECT_EQ(mockClock.nowMicros(), 2000u);
}

// ============================================================================
// Stopwatch Tests
// ============================================================================

TEST(StopwatchTest, BasicTiming) {
    FakeClock clock;
    Stopwatch sw(clock);
    
    clock.setTime(1000);
    sw.start();
    
    clock.setTime(2000);
    EXPECT_EQ(sw.elapsedMicros(), 1000);
    
    clock.setTime(3500);
    sw.stop();
    EXPECT_EQ(sw.elapsedMicros(), 2500);
    
    // After stop, elapsed doesn't change
    clock.setTime(5000);
    EXPECT_EQ(sw.elapsedMicros(), 2500);
}

TEST(StopwatchTest, Reset) {
    FakeClock clock;
    Stopwatch sw(clock);
    
    clock.setTime(1000);
    sw.start();
    clock.setTime(2000);
    sw.stop();
    
    EXPECT_EQ(sw.elapsedMicros(), 1000);
    
    sw.reset();
    EXPECT_EQ(sw.elapsedMicros(), 0);
    EXPECT_FALSE(sw.isRunning());
}

TEST(StopwatchTest, ElapsedMillis) {
    FakeClock clock;
    Stopwatch sw(clock);
    
    clock.setTime(0);
    sw.start();
    clock.setTime(5500);  // 5.5 ms
    
    EXPECT_EQ(sw.elapsedMillis(), 5);  // Truncated to 5ms
}

// ============================================================================
// FakePeriodicTimer Tests
// ============================================================================

TEST(FakePeriodicTimerTest, Init) {
    FakePeriodicTimer timer;
    
    EXPECT_TRUE(timer.init(1000));  // 1kHz
    EXPECT_EQ(timer.getPeriodMicros(), 1000);  // 1ms period
}

TEST(FakePeriodicTimerTest, InitZeroFreq) {
    FakePeriodicTimer timer;
    
    EXPECT_FALSE(timer.init(0));
}

TEST(FakePeriodicTimerTest, StartStop) {
    FakePeriodicTimer timer;
    timer.init(1000);
    
    EXPECT_FALSE(timer.isRunning());
    
    timer.start();
    EXPECT_TRUE(timer.isRunning());
    
    timer.stop();
    EXPECT_FALSE(timer.isRunning());
}

TEST(FakePeriodicTimerTest, Callback) {
    FakePeriodicTimer timer;
    timer.init(1000);
    timer.start();
    
    int callCount = 0;
    timer.setCallback([&callCount]() {
        callCount++;
    });
    
    timer.tick();
    timer.tick();
    timer.tick();
    
    EXPECT_EQ(callCount, 3);
}

TEST(FakePeriodicTimerTest, Stats) {
    FakePeriodicTimer timer;
    timer.init(1000);
    timer.start();
    
    timer.waitForCycle();
    timer.waitForCycle();
    timer.waitForCycle();
    
    auto stats = timer.getStats();
    EXPECT_EQ(stats.tickCount, 3u);
    
    timer.resetStats();
    stats = timer.getStats();
    EXPECT_EQ(stats.tickCount, 0u);
}
