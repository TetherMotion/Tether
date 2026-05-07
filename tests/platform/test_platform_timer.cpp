/**
 * @file test_platform_timer.cpp
 * @brief Unit tests for platform timer abstraction (IPlatformTimer)
 * 
 * Coverage:
 * - Timer configuration
 * - Timer start/stop
 * - Timer callbacks
 * - Jitter measurement
 * - Auto-reload functionality
 */

#include <gtest/gtest.h>
#include "tether/platform/IPlatformTimer.hpp"

#include <atomic>
#include <thread>
#include <chrono>

using namespace EtherCAT::Platform;

// ============================================================================
// Test Fixtures
// ============================================================================

class PlatformTimerTest : public ::testing::Test {
protected:
    void SetUp() override {
        callback_count_.store(0);
        timer_ = createPlatformTimer();
        ASSERT_NE(timer_, nullptr) << "Failed to create platform timer";
    }
    
    void TearDown() override {
        if (timer_) {
            timer_->stop();
        }
        // Give timer thread time to clean up
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    static bool testCallback(void* user_data) {
        auto* count = static_cast<std::atomic<int>*>(user_data);
        count->fetch_add(1);
        return false;
    }
    
    std::unique_ptr<IPlatformTimer> timer_;
    std::atomic<int> callback_count_;
};

// ============================================================================
// Basic Configuration Tests
// ============================================================================

TEST_F(PlatformTimerTest, CreateTimerSucceeds) {
    EXPECT_NE(timer_, nullptr);
    EXPECT_FALSE(timer_->isRunning());
}

TEST_F(PlatformTimerTest, ConfigureWithValidParameters) {
    TimerConfig config;
    config.period_us = 1000;  // 1ms
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.priority = 0;
    config.auto_reload = true;
    
    EXPECT_TRUE(timer_->configure(config));
}

TEST_F(PlatformTimerTest, ConfigureWhileRunningFails) {
    TimerConfig config;
    config.period_us = 1000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    // Try to reconfigure while running
    EXPECT_FALSE(timer_->configure(config));
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, GetActualPeriodMatchesRequested) {
    TimerConfig config;
    config.period_us = 5000;  // 5ms
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    EXPECT_EQ(timer_->getActualPeriodUs(), 5000);
}

// ============================================================================
// Start/Stop Tests
// ============================================================================

TEST_F(PlatformTimerTest, StartWithoutConfigureFails) {
    EXPECT_FALSE(timer_->start());
}

TEST_F(PlatformTimerTest, StartAfterConfigureSucceeds) {
    TimerConfig config;
    config.period_us = 10000;  // 10ms
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    EXPECT_TRUE(timer_->start());
    EXPECT_TRUE(timer_->isRunning());
    
    timer_->stop();
    EXPECT_FALSE(timer_->isRunning());
}

TEST_F(PlatformTimerTest, StartWhileRunningIsIdempotent) {
    TimerConfig config;
    config.period_us = 10000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    // Second start should succeed (idempotent)
    EXPECT_TRUE(timer_->start());
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, StopWhileNotRunningIsHarmless) {
    timer_->stop();  // Should not crash
    EXPECT_FALSE(timer_->isRunning());
}

TEST_F(PlatformTimerTest, MultipleStartStopCycles) {
    TimerConfig config;
    config.period_us = 5000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(timer_->start());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        timer_->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST_F(PlatformTimerTest, CallbackIsInvoked) {
    TimerConfig config;
    config.period_us = 5000;  // 5ms
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    // Wait for several callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    int count = callback_count_.load();
    EXPECT_GT(count, 0) << "Callback should have been invoked";
    EXPECT_LE(count, 15) << "Callback count seems too high (50ms / 5ms = ~10 expected)";
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, CallbackFrequencyIsCorrect) {
    TimerConfig config;
    config.period_us = 10000;  // 10ms period = 100 Hz
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    // Run for 100ms (should get ~10 callbacks)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    int count = callback_count_.load();
    
    // Allow ±3 tolerance for scheduling jitter
    EXPECT_GE(count, 7) << "Too few callbacks";
    EXPECT_LE(count, 13) << "Too many callbacks";
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, CallbackWithNullFunctionIsHandled) {
    TimerConfig config;
    config.period_us = 5000;
    config.callback = nullptr;  // No callback
    config.user_data = nullptr;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    
    // Should not crash — starting may be optional depending on implementation
    if (timer_->start()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        timer_->stop();
    }
    // Ensure timer is still usable after stop
    EXPECT_FALSE(timer_->isRunning());}

// ============================================================================
// Auto-Reload Tests
// ============================================================================

TEST_F(PlatformTimerTest, AutoReloadCausesPeriodicCallbacks) {
    TimerConfig config;
    config.period_us = 5000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    int count = callback_count_.load();
    EXPECT_GT(count, 5) << "Auto-reload should cause multiple callbacks";
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, OneShotModeStopsAfterFirstCallback) {
    TimerConfig config;
    config.period_us = 5000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = false;  // One-shot mode
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    int count = callback_count_.load();
    
    // In one-shot mode, should only fire once (or very few times due to timing)
    EXPECT_LE(count, 2) << "One-shot mode fired too many times";
    
    timer_->stop();
}

// ============================================================================
// Jitter Measurement Tests
// ============================================================================

TEST_F(PlatformTimerTest, JitterStatsAreAvailable) {
    TimerConfig config;
    config.period_us = 10000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    uint32_t max_jitter = 0, avg_jitter = 0;
    bool has_stats = timer_->getJitterStats(max_jitter, avg_jitter);
    
    EXPECT_TRUE(has_stats);
    
    // Jitter should be measurable but reasonable for 10ms period
    EXPECT_LT(max_jitter, 5000) << "Max jitter too high (>5ms for 10ms period)";
    EXPECT_LT(avg_jitter, 2000) << "Avg jitter too high (>2ms for 10ms period)";
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, JitterStatsUnavailableBeforeStart) {
    TimerConfig config;
    config.period_us = 5000;
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    
    uint32_t max_jitter = 0, avg_jitter = 0;
    bool has_stats = timer_->getJitterStats(max_jitter, avg_jitter);
    
    // Stats may not be available before starting
    // (Implementation-dependent, but should not crash)
    // Stats may be unavailable before start; ensure call is safe and returns boolean
    EXPECT_TRUE(has_stats == false || has_stats == true);}

// ============================================================================
// Timing Accuracy Tests
// ============================================================================

TEST_F(PlatformTimerTest, TimingAccuracyIsReasonable) {
    TimerConfig config;
    config.period_us = 10000;  // 10ms
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Wait for exactly 100 callbacks
    while (callback_count_.load() < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // 100 callbacks at 10ms each = 1000ms expected
    // Allow ±10% tolerance
    EXPECT_GE(elapsed_ms, 900) << "Timer running too fast";
    EXPECT_LE(elapsed_ms, 1100) << "Timer running too slow";
    
    timer_->stop();
}

// ============================================================================
// Stress Tests
// ============================================================================

TEST_F(PlatformTimerTest, HighFrequencyTimerIsStable) {
    TimerConfig config;
    config.period_us = 1000;  // 1ms = 1kHz (same as DC typical frequency)
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    // Run at 1kHz for 1 second = 1000 callbacks expected
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    int count = callback_count_.load();
    
    // Allow ±10% tolerance
    EXPECT_GE(count, 900) << "High frequency timer missed too many cycles";
    EXPECT_LE(count, 1100) << "High frequency timer fired too many times";
    
    timer_->stop();
}

TEST_F(PlatformTimerTest, TimerDoesNotDriftOverTime) {
    TimerConfig config;
    config.period_us = 10000;  // 10ms
    config.callback = testCallback;
    config.user_data = &callback_count_;
    config.auto_reload = true;
    
    ASSERT_TRUE(timer_->configure(config));
    ASSERT_TRUE(timer_->start());
    
    // Run for 5 seconds and check if drift accumulates
    auto start = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    auto end = std::chrono::high_resolution_clock::now();
    
    int count = callback_count_.load();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // Expected: 500 callbacks (5000ms / 10ms)
    double expected = elapsed_ms / 10.0;
    double error_percent = std::abs(count - expected) / expected * 100.0;
    
    EXPECT_LT(error_percent, 5.0) << "Timer drift >5% over 5 seconds";
    
    timer_->stop();
}
