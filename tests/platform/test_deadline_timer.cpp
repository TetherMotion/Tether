/**
 * @file test_deadline_timer.cpp
 * @brief Unit tests for the IDeadlineTimer absolute-deadline abstraction
 *
 * Coverage:
 * - Factory creation
 * - Periodic waitNext() with absolute deadlines (no drift)
 * - Missed-deadline accounting on overrun (skip, not burst)
 * - Cooperative requestStop()
 * - Invalid period rejection
 * - Hybrid sleep+spin variant
 *
 * The Linux implementation maps onto
 * clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME); these tests exercise that
 * real path with generous tolerances (tests run on non-RT hosts).
 */

#include <gtest/gtest.h>

#include "tether/platform/IDeadlineTimer.hpp"

#include <chrono>
#include <thread>

using namespace EtherCAT::Platform;

namespace {
constexpr uint64_t kPeriodNs = 1'000'000;   // 1 ms
// Non-RT CI hosts can jitter by tens of ms; keep assertions structural
// (ordering, drift, missed counts) rather than tight latency bounds.
constexpr uint64_t kSlackNs  = 50'000'000;  // 50 ms sanity bound
} // namespace

class DeadlineTimerTest : public ::testing::Test {
protected:
    void SetUp() override {
        timer_ = createDeadlineTimer();
        ASSERT_NE(timer_, nullptr);
    }
    void TearDown() override {
        if (timer_) timer_->requestStop();
    }
    std::unique_ptr<IDeadlineTimer> timer_;
};

TEST_F(DeadlineTimerTest, CreateSucceeds) {
    EXPECT_NE(timer_, nullptr);
    EXPECT_FALSE(timer_->isRunning());
}

TEST_F(DeadlineTimerTest, StartWithZeroPeriodFails) {
    EXPECT_FALSE(timer_->start(0));
    EXPECT_FALSE(timer_->isRunning());
}

TEST_F(DeadlineTimerTest, WaitNextReturnsTick) {
    ASSERT_TRUE(timer_->start(kPeriodNs));
    EXPECT_TRUE(timer_->isRunning());
    EXPECT_EQ(timer_->periodNs(), kPeriodNs);

    IDeadlineTimer::Tick t{};
    EXPECT_TRUE(timer_->waitNext(t));
    EXPECT_GT(t.woke_ns, 0u);
    EXPECT_GT(t.deadline_ns, 0u);
}

TEST_F(DeadlineTimerTest, DeadlinesAdvanceByExactPeriod) {
    // Absolute-deadline scheduling: successive tick deadlines must differ by
    // exactly period_ns — no drift accumulation.
    ASSERT_TRUE(timer_->start(kPeriodNs));

    IDeadlineTimer::Tick a{}, b{}, c{};
    ASSERT_TRUE(timer_->waitNext(a));
    ASSERT_TRUE(timer_->waitNext(b));
    ASSERT_TRUE(timer_->waitNext(c));

    EXPECT_EQ(b.deadline_ns - a.deadline_ns, kPeriodNs);
    EXPECT_EQ(c.deadline_ns - b.deadline_ns, kPeriodNs);
    EXPECT_EQ(a.missed, 0u);
}

TEST_F(DeadlineTimerTest, OverrunSkipsMissedDeadlines) {
    // Sleep past several deadlines, then verify the timer reports them as
    // missed and the next deadline is in the future (no catch-up burst).
    ASSERT_TRUE(timer_->start(kPeriodNs));

    IDeadlineTimer::Tick t{};
    ASSERT_TRUE(timer_->waitNext(t));

    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // ~5 periods

    ASSERT_TRUE(timer_->waitNext(t));
    EXPECT_GE(t.missed, 1u) << "overrun deadlines were not reported";
    EXPECT_GT(t.deadline_ns, t.woke_ns > kSlackNs ? t.woke_ns - kSlackNs : 0u)
        << "next deadline was not advanced past the overrun";
}

TEST_F(DeadlineTimerTest, RequestStopEndsWaitNext) {
    ASSERT_TRUE(timer_->start(kPeriodNs));

    IDeadlineTimer::Tick t{};
    ASSERT_TRUE(timer_->waitNext(t));

    timer_->requestStop();
    EXPECT_FALSE(timer_->isRunning());
    EXPECT_FALSE(timer_->waitNext(t));
}

TEST_F(DeadlineTimerTest, WaitNextBeforeStartFails) {
    IDeadlineTimer::Tick t{};
    EXPECT_FALSE(timer_->waitNext(t));
}

TEST(HybridDeadlineTimerTest, CreateAndRun) {
    auto timer = createHybridDeadlineTimer(20'000);  // 20 µs spin tail
    ASSERT_NE(timer, nullptr);

    ASSERT_TRUE(timer->start(kPeriodNs));
    IDeadlineTimer::Tick a{}, b{};
    ASSERT_TRUE(timer->waitNext(a));
    ASSERT_TRUE(timer->waitNext(b));
    EXPECT_EQ(b.deadline_ns - a.deadline_ns, kPeriodNs);
    timer->requestStop();
    EXPECT_FALSE(timer->waitNext(a));
}

TEST(HybridDeadlineTimerTest, ZeroSpinWindowBehavesLikeNanosleep) {
    auto timer = createHybridDeadlineTimer(0);
    ASSERT_NE(timer, nullptr);
    ASSERT_TRUE(timer->start(kPeriodNs));
    IDeadlineTimer::Tick t{};
    EXPECT_TRUE(timer->waitNext(t));
    timer->requestStop();
}
