/**
 * @file test_hal_lifecycle.cpp
 * @brief Tests for HAL singleton lifecycle management (init/shutdown/reset)
 *
 * Verifies that HAL platform singletons (Clock, Threading, HAL) have proper
 * lifecycle control and can be reset cleanly between test cases.
 */

#include <gtest/gtest.h>

#include "hal/IClock.hpp"
#include "hal/IThreading.hpp"
#include "hal/HAL.hpp"

namespace EtherCAT {
namespace HAL {
namespace test {

// ============================================================================
// Clock Factory Lifecycle Tests
// ============================================================================

TEST(ClockFactoryLifecycle, GetClockFactoryReturnsValid) {
    auto& factory = getClockFactory();
    auto& clock = factory.getSystemClock();
    // System clock should return non-zero timestamps
    auto t1 = clock.nowMicros();
    auto t2 = clock.nowMicros();
    EXPECT_GE(t2, t1);
}

TEST(ClockFactoryLifecycle, ResetAndReacquire) {
    // Get initial factory
    auto& factory1 = getClockFactory();
    auto t1 = factory1.getSystemClock().nowMicros();
    EXPECT_GT(t1, 0u);

    // Reset factory
    resetClockFactory();

    // Re-acquire — should create a new factory transparently
    auto& factory2 = getClockFactory();
    auto t2 = factory2.getSystemClock().nowMicros();
    EXPECT_GE(t2, t1);
}

TEST(ClockFactoryLifecycle, SetCustomFactory) {
    // Set a custom clock factory, then reset to default
    class DummyClock : public IClock {
    public:
        Timestamp nowMicros() override { return 42; }
        Timestamp systemTimeMillis() override { return 0; }
        Nanoseconds resolution() override { return 1000; }
        void delayMicros(Microseconds) override {}
        void delayMillis(Milliseconds) override {}
    };

    class DummyClockFactory : public IClockFactory {
    public:
        IClock& getSystemClock() override {
            static DummyClock clock;
            return clock;
        }
        std::unique_ptr<IPeriodicTimer> createPeriodicTimer() override { return nullptr; }
        std::unique_ptr<IOneShotTimer> createOneShotTimer() override { return nullptr; }
    };

    setClockFactory(std::make_unique<DummyClockFactory>());
    EXPECT_EQ(getClockFactory().getSystemClock().nowMicros(), 42u);

    // Reset back to platform default
    resetClockFactory();
    EXPECT_NE(getClockFactory().getSystemClock().nowMicros(), 42u);
}

// ============================================================================
// Threading Factory Lifecycle Tests
// ============================================================================

TEST(ThreadingFactoryLifecycle, GetThreadingFactoryReturnsValid) {
    auto& factory = getThreadingFactory();
    auto mutex = factory.createMutex();
    ASSERT_NE(mutex, nullptr);
    EXPECT_EQ(mutex->lock(), Error::OK);
    EXPECT_EQ(mutex->unlock(), Error::OK);
}

TEST(ThreadingFactoryLifecycle, ResetAndReacquire) {
    // Get initial factory
    auto& factory1 = getThreadingFactory();
    auto mutex1 = factory1.createMutex();
    ASSERT_NE(mutex1, nullptr);

    // Reset factory
    resetThreadingFactory();

    // Re-acquire — should create a new factory transparently
    auto& factory2 = getThreadingFactory();
    auto mutex2 = factory2.createMutex();
    ASSERT_NE(mutex2, nullptr);
    EXPECT_EQ(mutex2->lock(), Error::OK);
    EXPECT_EQ(mutex2->unlock(), Error::OK);
}

// ============================================================================
// HAL Instance Lifecycle Tests
// ============================================================================

TEST(HALLifecycle, ResetHALAllowsReinitialization) {
    // Reset to start clean
    resetHAL();

    // Get HAL (creates uninitialized instance)
    auto& hal = getHAL();
    EXPECT_FALSE(hal.isInitialized());

    // Reset again
    resetHAL();

    // Should be able to get a fresh instance
    auto& hal2 = getHAL();
    EXPECT_FALSE(hal2.isInitialized());

    // Clean up
    resetHAL();
}

}  // namespace test
}  // namespace HAL
}  // namespace EtherCAT
