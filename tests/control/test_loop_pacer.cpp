// LoopPacer: pacing policies (free/fixed-rate/snap-grid), catch-up and
// resync accounting, overrun/late-start stats — all on an injected clock.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>

#include "tether/control/LoopPacer.hpp"

using namespace tether::control;
using Clock = std::chrono::steady_clock;
using TP = Clock::time_point;

namespace {

// Deterministic clock: now() returns t_; sleep jumps to the target
// (overshoot configurable to simulate wake lateness).
struct FakeClock {
    TP t = TP(std::chrono::microseconds(1'000'000));
    double overshootUs = 0.0;

    LoopPacer<64>::NowFn nowFn() {
        return [this] { return t; };
    }
    LoopPacer<64>::SleepFn sleepFn() {
        return [this](TP target) {
            if (target > t) t = target;
            t += std::chrono::microseconds(static_cast<long>(overshootUs));
        };
    }
    void advance(double us) { t += std::chrono::microseconds(static_cast<long>(us)); }
};

TEST(LoopPacer, FreeRunFiresImmediately) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<FreeRunPolicy>(), 0.5, 0.0,
                        clk.nowFn(), clk.sleepFn());
    const double late = pacer.endTick(300.0);
    EXPECT_DOUBLE_EQ(late, 0.0);          // no target → no lateness
    EXPECT_EQ(pacer.periodUs(), 0.0);
    const auto s = pacer.stats();
    EXPECT_EQ(s.ticks, 1u);
    EXPECT_DOUBLE_EQ(s.emaTickUs, 300.0); // first sample seeds the EMA
    EXPECT_EQ(s.overruns, 0u);            // period 0 → never an overrun
}

TEST(LoopPacer, FixedRateSleepsTheRemainder) {
    FakeClock clk;
    const TP start = clk.t;
    LoopPacer<64> pacer(std::make_unique<FixedRatePolicy>(1000.0), 1.0, 0.0,
                        clk.nowFn(), clk.sleepFn());
    EXPECT_DOUBLE_EQ(pacer.endTick(400.0), 0.0);
    // First schedule anchors the grid at t0+1000; fake sleep jumped there.
    EXPECT_EQ(clk.t, start + std::chrono::microseconds(1000));
    pacer.endTick(400.0);
    EXPECT_EQ(clk.t, start + std::chrono::microseconds(2000));  // strict grid
    EXPECT_EQ(pacer.resyncs(), 0u);
    EXPECT_EQ(pacer.catchUpTicks(), 0u);
}

TEST(LoopPacer, FixedRateOverrunCatchesUpImmediately) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<FixedRatePolicy>(1000.0), 1.0, 0.0,
                        clk.nowFn(), clk.sleepFn());
    pacer.endTick(400.0);   // on-time tick
    clk.advance(1300.0);    // work overran the 1000µs slot by 300µs
    pacer.endTick(0.0);     // behind schedule → fires immediately, no sleep
    EXPECT_EQ(pacer.catchUpTicks(), 1u);
    const auto s = pacer.stats();
    EXPECT_EQ(s.overruns, 0u);  // workUs=0 vs period — no overrun recorded
}

TEST(LoopPacer, FixedRateDeepLagResyncs) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<FixedRatePolicy>(1000.0, 5000.0),
                        1.0, 0.0, clk.nowFn(), clk.sleepFn());
    pacer.endTick(0.0);     // anchor
    clk.advance(20'000.0);  // 20ms stall ≫ 5ms resync threshold
    pacer.endTick(0.0);
    EXPECT_EQ(pacer.resyncs(), 1u);       // grid rebased to now
    EXPECT_EQ(pacer.catchUpTicks(), 1u);  // the deep-lag fire counted first
    clk.advance(400.0);                   // next tick back on a fresh grid
    pacer.endTick(0.0);
    EXPECT_EQ(pacer.resyncs(), 1u);       // no further resync
}

TEST(LoopPacer, FixedRateNoResyncKeepsCatchingUp) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<FixedRatePolicy>(1000.0, 0.0), 1.0,
                        0.0, clk.nowFn(), clk.sleepFn());
    pacer.endTick(0.0);
    clk.advance(10'000.0);  // 10 slots behind, resync disabled
    pacer.endTick(0.0);
    pacer.endTick(0.0);     // still behind → immediate fires keep coming
    EXPECT_EQ(pacer.resyncs(), 0u);
    EXPECT_EQ(pacer.catchUpTicks(), 2u);
}

TEST(LoopPacer, SnapToGridWaitsForNextSlot) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<SnapToGridPolicy>(1000.0, 50.0),
                        1.0, 0.0, clk.nowFn(), clk.sleepFn());
    const TP start = clk.t;
    pacer.endTick(0.0);                    // anchors origin at start
    clk.advance(3700.0);                   // 3.7 slots in — outside tolerance
    pacer.endTick(0.0);
    // Next start = slot 4, i.e. 4000µs after the anchor — it waited.
    EXPECT_EQ(clk.t, start + std::chrono::microseconds(4000));
    EXPECT_EQ(pacer.catchUpTicks(), 0u);   // waited, not a catch-up
}

TEST(LoopPacer, SnapToGridToleranceFiresInsideGrace) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<SnapToGridPolicy>(1000.0, 100.0),
                        1.0, 0.0, clk.nowFn(), clk.sleepFn());
    pacer.endTick(0.0);          // anchor at t0
    clk.advance(1030.0);         // 30µs past slot 1 → inside 100µs grace
    pacer.endTick(0.0);          // fires now, counted as catch-up
    EXPECT_EQ(pacer.catchUpTicks(), 1u);
}

TEST(LoopPacer, OverrunAndLateStartAccounting) {
    FakeClock clk;
    clk.overshootUs = 250.0;  // every wake lands 250µs late
    LoopPacer<64> pacer(std::make_unique<FixedRatePolicy>(1000.0), 1.0, 200.0,
                        clk.nowFn(), clk.sleepFn());
    pacer.endTick(1500.0);   // work exceeds the 1000µs period → overrun
    const auto s = pacer.stats();
    EXPECT_EQ(s.overruns, 1u);
    EXPECT_DOUBLE_EQ(s.maxOverrunUs, 500.0);
    EXPECT_EQ(s.lateStarts, 1u);         // 250µs > 200µs threshold
    EXPECT_DOUBLE_EQ(s.lastLateStartUs, 250.0);
    EXPECT_DOUBLE_EQ(s.maxLateStartUs, 250.0);
}

TEST(LoopPacer, StatsAccumulateAndWindowFills) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<FreeRunPolicy>(), 0.5, 0.0,
                        clk.nowFn(), clk.sleepFn());
    pacer.endTick(100.0);
    pacer.endTick(300.0);
    const auto s = pacer.stats();
    EXPECT_EQ(s.ticks, 2u);
    EXPECT_DOUBLE_EQ(s.emaTickUs, 200.0);  // 0.5 blend: (100+300)/2
    EXPECT_DOUBLE_EQ(s.maxTickUs, 300.0);
    EXPECT_EQ(pacer.tickWindow().size(), 2u);
    EXPECT_DOUBLE_EQ(s.effectiveHz(), 5000.0);          // 1e6/200µs
    EXPECT_DOUBLE_EQ(s.effectiveRtf(1000.0), 5.0);      // 1ms sim / 200µs wall
}

TEST(LoopPacer, ResetClearsStatsAndReanchors) {
    FakeClock clk;
    LoopPacer<64> pacer(std::make_unique<FixedRatePolicy>(1000.0), 1.0, 0.0,
                        clk.nowFn(), clk.sleepFn());
    pacer.endTick(500.0);
    pacer.endTick(500.0);
    pacer.reset();
    const auto s = pacer.stats();
    EXPECT_EQ(s.ticks, 0u);
    EXPECT_EQ(pacer.tickWindow().size(), 0u);
    pacer.endTick(500.0);
    EXPECT_EQ(pacer.stats().ticks, 1u);
    EXPECT_EQ(pacer.catchUpTicks(), 0u);  // re-anchored, not behind
}

TEST(LoopPacer, BoundaryValidation) {
    FakeClock clk;
    EXPECT_THROW(LoopPacer<64>(nullptr, 0.5, 0.0, clk.nowFn(), clk.sleepFn()),
                 std::invalid_argument);
    EXPECT_THROW(LoopPacer<64>(std::make_unique<FreeRunPolicy>(), 0.0, 0.0,
                               clk.nowFn(), clk.sleepFn()),
                 std::invalid_argument);
    EXPECT_THROW(LoopPacer<64>(std::make_unique<FreeRunPolicy>(), 1.1, 0.0,
                               clk.nowFn(), clk.sleepFn()),
                 std::invalid_argument);
    EXPECT_THROW(FixedRatePolicy(0.0), std::invalid_argument);
    EXPECT_THROW(FixedRatePolicy(1000.0, -1.0), std::invalid_argument);
    EXPECT_THROW(SnapToGridPolicy(-5.0), std::invalid_argument);
}

// A user-defined policy: pace at a fixed wall-rate but count every tick.
class CountingPolicy final : public IPacingPolicy {
public:
    TimePoint schedule(TimePoint now) override {
        ++calls_;
        return now;
    }
    double periodUs() const override { return 0.0; }
    void reset(TimePoint) override { calls_ = 0; }
    uint64_t calls_ = 0;
};

TEST(LoopPacer, CustomPolicyViaVirtualInterface) {
    FakeClock clk;
    auto policy = std::make_unique<CountingPolicy>();
    auto* raw = policy.get();
    LoopPacer<64> pacer(std::move(policy), 1.0, 0.0, clk.nowFn(), clk.sleepFn());
    pacer.endTick(10.0);
    pacer.endTick(10.0);
    EXPECT_EQ(raw->calls_, 2u);
    EXPECT_EQ(pacer.stats().ticks, 2u);
}

}  // namespace
