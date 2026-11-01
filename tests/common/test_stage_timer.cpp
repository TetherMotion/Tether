// StageTimer: checkpoint deltas, elapsed span, reset semantics, fake clock.
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "tether/common/StageTimer.hpp"

using tether::common::StageTimer;

namespace {

using TimePoint = StageTimer::TimePoint;

// Manual time base: the NowFn lambda reads `t`, the test advances it.
struct FakeClock {
    TimePoint t{};

    StageTimer::NowFn fn() { return [this] { return t; }; }

    void advanceUs(double us) {
        t += std::chrono::duration_cast<TimePoint::duration>(
            std::chrono::duration<double, std::micro>(us));
    }
};

TEST(StageTimer, FirstCheckpointMeasuresSinceConstruction) {
    FakeClock clk;
    clk.advanceUs(100);
    StageTimer st(clk.fn());

    clk.advanceUs(40);
    EXPECT_DOUBLE_EQ(st.checkpoint(), 40.0);
}

TEST(StageTimer, CheckpointsReturnDeltasNotCumulative) {
    FakeClock clk;
    StageTimer st(clk.fn());

    clk.advanceUs(10);
    EXPECT_DOUBLE_EQ(st.checkpoint(), 10.0);
    clk.advanceUs(25);
    EXPECT_DOUBLE_EQ(st.checkpoint(), 25.0);   // not 35
    clk.advanceUs(5);
    EXPECT_DOUBLE_EQ(st.checkpoint(), 5.0);
}

TEST(StageTimer, CheckpointDeltasSumToElapsed) {
    // One clock read per boundary: consecutive deltas tile the whole span.
    FakeClock clk;
    StageTimer st(clk.fn());

    double sum = 0;
    for (double stage : {12.0, 7.5, 30.0, 3.0}) {
        clk.advanceUs(stage);
        sum += st.checkpoint();
    }
    EXPECT_DOUBLE_EQ(sum, 52.5);
    EXPECT_DOUBLE_EQ(st.elapsed(), 52.5);
}

TEST(StageTimer, ElapsedIncludesUncheckpointedTail) {
    FakeClock clk;
    StageTimer st(clk.fn());

    clk.advanceUs(10);
    (void)st.checkpoint();
    clk.advanceUs(8);   // tail with no checkpoint
    EXPECT_DOUBLE_EQ(st.elapsed(), 18.0);
}

TEST(StageTimer, ResetRebasesBothMarks) {
    FakeClock clk;
    StageTimer st(clk.fn());

    clk.advanceUs(100);
    st.reset();
    EXPECT_DOUBLE_EQ(st.elapsed(), 0.0);

    clk.advanceUs(15);
    EXPECT_DOUBLE_EQ(st.checkpoint(), 15.0);
    EXPECT_DOUBLE_EQ(st.elapsed(), 15.0);
}

TEST(StageTimer, ZeroDeltaWhenNoTimePasses) {
    FakeClock clk;
    StageTimer st(clk.fn());
    EXPECT_DOUBLE_EQ(st.checkpoint(), 0.0);
    EXPECT_DOUBLE_EQ(st.checkpoint(), 0.0);
    EXPECT_DOUBLE_EQ(st.elapsed(), 0.0);
}

TEST(StageTimer, PositionalFieldMapping) {
    // The intended use: checkpoint order maps to caller fields by assignment.
    FakeClock clk;
    struct Tick {
        double obs_us{}, exec_us{}, tick_us{};
    } tick;

    StageTimer st(clk.fn());
    clk.advanceUs(20);                       // observe()
    tick.obs_us = st.checkpoint();
    clk.advanceUs(45);                       // execute()
    tick.exec_us = st.checkpoint();
    tick.tick_us = st.elapsed();

    EXPECT_DOUBLE_EQ(tick.obs_us, 20.0);
    EXPECT_DOUBLE_EQ(tick.exec_us, 45.0);
    EXPECT_DOUBLE_EQ(tick.tick_us, 65.0);
}

TEST(StageTimer, DefaultClockIsSteadyClock) {
    StageTimer st;   // real clock — just verify sane behaviour
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    const double d = st.checkpoint();
    const double e = st.elapsed();
    EXPECT_GE(d, 40.0);   // slack below the 50us sleep
    EXPECT_GE(e, d);
}

}  // namespace
