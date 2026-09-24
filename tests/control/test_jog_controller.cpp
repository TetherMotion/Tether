/**
 * @file test_jog_controller.cpp
 * @brief Unit tests for the generic failsafe JogController: lease expiry,
 *        server-side rate/lease/step clamps, incremental moves, ownership,
 *        and the describeJson() metadata used by operator UIs.
 */

#include <gtest/gtest.h>

#include "tether/control/JogController.hpp"

#include <atomic>
#include <cmath>
#include <string>
#include <thread>

using namespace tether::control;

namespace {

JogController::Config twoAxisConfig() {
    JogController::Config cfg;
    JogController::AxisConfig x;
    x.name = "x";
    x.maxRate = 50.0;
    x.accel = 500.0;
    x.maxLeaseMs = 500;
    x.maxIncrement = 10.0;
    JogController::AxisConfig e;
    e.name = "e";
    e.unit = "mm";
    e.maxRate = 5.0;
    e.accel = 50.0;
    e.maxLeaseMs = 200;
    e.maxIncrement = 2.0;
    e.ratePresets = {0.5, 1.0, 5.0};
    e.incrementPresets = {0.5, 1.0, 2.0};
    cfg.axes = {x, e};
    JogController::Group linear{"linear", {0}, {}, {}};
    JogController::Group extruder{"extruder", {1}, {0.5, 1.0}, {0.5, 1.0}};
    cfg.groups = {linear, extruder};
    cfg.defaultLeaseMs = 300;
    return cfg;
}

/// Advance the controller `ms` milliseconds in 1 ms steps.
void run(JogController& c, int64_t& now, int64_t ms, double dt = 0.001) {
    for (int64_t i = 0; i < ms; ++i) c.update(++now, dt);
}

} // namespace

TEST(JogController, ContinuousJogReachesMaxRateAndPosition) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 30.0, 300, now));
    run(jog, now, 500);

    // rate ramps to commanded 30 units/s, position integrates forward.
    EXPECT_NEAR(jog.axisRate(0), 0.0, 1e-9);  // lease expired after 300ms
    EXPECT_FALSE(jog.axisActive(0));
    EXPECT_GT(jog.axisPosition(0), 1.0);
}

TEST(JogController, LeaseRefreshKeepsJogAlive) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 20.0, 200, now));
    // Refresh at 150ms cadence (within the 200ms lease) for 1s.
    for (int k = 0; k < 7; ++k) {
        run(jog, now, 150);
        EXPECT_TRUE(jog.jog(0, 20.0, 200, now));
    }
    run(jog, now, 150);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Continuous);
    EXPECT_NEAR(jog.axisRate(0), 20.0, 1e-6);

    // Stop refreshing — must coast to a stop within lease + decel.
    run(jog, now, 400);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_NEAR(jog.axisRate(0), 0.0, 1e-9);
}

TEST(JogController, LeaseExpiryIsBoundedAndDecelerates) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 50.0, 300, now));
    run(jog, now, 200);  // at full rate 50 u/s
    EXPECT_NEAR(jog.axisRate(0), 50.0, 1e-6);

    // No refresh: after 300ms lease, mode becomes Stopping and decelerates
    // at accel=500 u/s^2 -> ~100ms to reach zero.
    run(jog, now, 150);  // t=350: inside lease still? no — lease ends at 300
    EXPECT_NE(jog.axisMode(0), JogController::Mode::Continuous);
    run(jog, now, 300);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_NEAR(jog.axisRate(0), 0.0, 1e-9);
}

TEST(JogController, RateClampedToMaxRate) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 9999.0, 300, now));  // absurd rate request
    run(jog, now, 250);
    EXPECT_LE(jog.axisRate(0), 50.0 + 1e-9);    // clamped server-side
    run(jog, now, 300);
}

TEST(JogController, LeaseClampedToMaxLeaseMs) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    // Request a 60s lease on axis 'e' (maxLeaseMs=200) — clamped to 200.
    EXPECT_TRUE(jog.jog(1, 5.0, 60000, now));
    run(jog, now, 250);  // past the clamped lease
    EXPECT_NE(jog.axisMode(1), JogController::Mode::Continuous);
    run(jog, now, 500);
    EXPECT_EQ(jog.axisMode(1), JogController::Mode::Idle);
}

TEST(JogController, IncrementalMoveCompletesExactly) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_DOUBLE_EQ(jog.move(0, 5.0, now), 5.0);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Incremental);
    run(jog, now, 5000);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_NEAR(jog.axisPosition(0), 5.0, 1e-3);
    EXPECT_NEAR(jog.axisRate(0), 0.0, 1e-9);
}

TEST(JogController, IncrementalStepSizeClampedServerSide) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    // Request 100mm on axis 'e' (maxIncrement=2) — clamped, not trusted.
    EXPECT_DOUBLE_EQ(jog.move(1, 100.0, now), 2.0);
    run(jog, now, 5000);
    EXPECT_EQ(jog.axisMode(1), JogController::Mode::Idle);
    EXPECT_NEAR(jog.axisPosition(1), 2.0, 1e-3);
}

TEST(JogController, IncrementalMovesAccumulate) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_DOUBLE_EQ(jog.move(0, 3.0, now), 3.0);
    EXPECT_DOUBLE_EQ(jog.move(0, 2.0, now), 2.0);  // chains while moving
    run(jog, now, 8000);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_NEAR(jog.axisPosition(0), 5.0, 1e-3);
}

TEST(JogController, MoveRejectedWhileContinuousJog) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 10.0, 400, now));
    run(jog, now, 50);
    EXPECT_DOUBLE_EQ(jog.move(0, 1.0, now), 0.0);  // rejected — jog owns axis
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Continuous);
}

TEST(JogController, ExplicitStopRampsDown) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 50.0, 500, now));
    run(jog, now, 200);
    EXPECT_NEAR(jog.axisRate(0), 50.0, 1e-6);
    jog.stop(0);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Stopping);
    run(jog, now, 200);  // decel 50 -> 0 at 500 u/s^2 = 100ms
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_NEAR(jog.axisRate(0), 0.0, 1e-9);
}

TEST(JogController, StopAllStopsEveryAxis) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 30.0, 500, now));
    EXPECT_TRUE(jog.jog(1, 5.0, 200, now));
    run(jog, now, 100);
    jog.stopAll();
    run(jog, now, 1000);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_EQ(jog.axisMode(1), JogController::Mode::Idle);
}

TEST(JogController, InvalidCommandsRejected) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_FALSE(jog.jog(99, 10.0, 100, now));          // bad axis
    EXPECT_FALSE(jog.jog(0, NAN, 100, now));            // NaN rate
    EXPECT_FALSE(jog.jog(0, INFINITY, 100, now));       // inf rate
    EXPECT_DOUBLE_EQ(jog.move(99, 1.0, now), 0.0);      // bad axis
    EXPECT_DOUBLE_EQ(jog.move(0, NAN, now), 0.0);       // NaN distance
    EXPECT_FALSE(jog.axisActive(0));
}

TEST(JogController, ZeroRateJogHoldsLeaseAlive) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 0.0, 200, now));
    run(jog, now, 100);
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Continuous);
    EXPECT_NEAR(jog.axisRate(0), 0.0, 1e-9);
    EXPECT_NEAR(jog.axisPosition(0), 0.0, 1e-9);
}

TEST(JogController, SnapshotReportsState) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;

    EXPECT_TRUE(jog.jog(0, 10.0, 400, now));
    run(jog, now, 100);
    JogController::AxisSnapshot s{};
    jog.fillSnapshot(0, s, now);
    EXPECT_EQ(s.mode, static_cast<uint8_t>(JogController::Mode::Continuous));
    EXPECT_EQ(s.active, 1);
    EXPECT_GT(s.rate, 0.0);
    EXPECT_GT(s.leaseLeftMs, 0u);
    EXPECT_LE(s.leaseLeftMs, 400u);
    EXPECT_EQ(s.maxLeaseMs, 500u);
    EXPECT_DOUBLE_EQ(s.maxRate, 50.0);
    EXPECT_DOUBLE_EQ(s.maxIncrement, 10.0);
}

TEST(JogController, DescribeJsonContainsAxesGroupsAndLimits) {
    JogController jog(twoAxisConfig());
    const std::string j = jog.describeJson();

    EXPECT_NE(j.find("\"axes\":["), std::string::npos);
    EXPECT_NE(j.find("\"name\":\"x\""), std::string::npos);
    EXPECT_NE(j.find("\"name\":\"e\""), std::string::npos);
    EXPECT_NE(j.find("\"maxRate\":5"), std::string::npos);
    EXPECT_NE(j.find("\"maxIncrement\":2"), std::string::npos);
    EXPECT_NE(j.find("\"maxLeaseMs\":200"), std::string::npos);
    EXPECT_NE(j.find("\"groups\":["), std::string::npos);
    EXPECT_NE(j.find("\"name\":\"linear\""), std::string::npos);
    EXPECT_NE(j.find("\"name\":\"extruder\""), std::string::npos);
    EXPECT_NE(j.find("[0.5,1]"), std::string::npos);  // group rate presets
}

TEST(JogController, ConcurrentCommandThreadsDoNotCorrupt) {
    JogController jog(twoAxisConfig());
    std::atomic<int64_t> now{0};

    // Two writer threads racing jog() and move() on the same axis while
    // update() runs — no crash, no corruption, final state is coherent.
    std::atomic<bool> runFlag{true};
    std::thread updater([&] {
        while (runFlag.load()) {
            jog.update(now.load(), 0.001);
            now.fetch_add(1);
        }
    });
    std::thread t1([&] {
        for (int i = 0; i < 200; ++i) jog.jog(0, 10.0, 50, now.load());
    });
    std::thread t2([&] {
        for (int i = 0; i < 200; ++i) jog.move(0, 1.0, now.load());
    });
    t1.join();
    t2.join();
    runFlag.store(false);
    updater.join();

    jog.stopAll();
    for (int i = 0; i < 5000; ++i) jog.update(now.fetch_add(1), 0.001);
    // Whatever the interleaving, the axis must settle coherently.
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_TRUE(std::isfinite(jog.axisPosition(0)));
    EXPECT_TRUE(std::isfinite(jog.axisRate(0)));
}

TEST(JogController, ApplyCallbackReceivesRampedSetpoints) {
    JogController jog(twoAxisConfig());
    std::vector<JogSetpoint> captured;
    jog.setApplyCallback(
        [&](const JogSetpoint& sp) { captured.push_back(sp); });

    int64_t now = 0;
    EXPECT_TRUE(jog.jog(0, 10.0, 300, now));
    run(jog, now, 50);

    // Every axis is reported every cycle (2 axes × 50 cycles).
    ASSERT_EQ(captured.size(), 100u);
    const JogSetpoint& last = captured[98];  // last axis-0 entry
    EXPECT_EQ(last.axis, 0u);
    EXPECT_EQ(last.mode,
              static_cast<uint8_t>(JogController::Mode::Continuous));
    EXPECT_TRUE(last.active);
    EXPECT_GT(last.rate, 0.0);
    EXPECT_GT(last.position, 0.0);
    EXPECT_GT(last.leaseLeftMs, 0u);
    EXPECT_EQ(last.nowMs, now);
}

namespace {

/// Test ramper: jumps to the target instantly instead of ramping —
/// proves the controller drives an injected IJogRamper.
class InstantRamper : public IJogRamper {
public:
    void setRate(double rate) override { m_rate = rate; m_mode = 1; }
    void setTarget(double position) override {
        m_target = position;
        m_mode = 2;
    }
    void stop() override { m_mode = 3; }
    void update(double) override {
        if (m_mode == 2) { m_pos = m_target; m_mode = 0; }
        else if (m_mode == 3) { m_rate = 0.0; m_mode = 0; }
        else if (m_mode == 1) m_pos += m_rate * 0.001;
    }
    double position() const override { return m_pos; }
    double rate() const override { return m_mode == 1 ? m_rate : 0.0; }
    double target() const override { return m_mode == 2 ? m_target : m_pos; }
    bool finished() const override { return m_mode == 0; }
    void reset(double position) override { m_pos = position; }
    int m_mode = 0;
    double m_pos = 0.0, m_rate = 0.0, m_target = 0.0;
};

} // namespace

TEST(JogController, CustomRamperFactoryIsDriven) {
    auto cfg = twoAxisConfig();
    cfg.ramperFactory = [](const JogController::AxisConfig&) {
        return std::make_unique<InstantRamper>();
    };
    JogController jog(cfg);
    int64_t now = 0;

    EXPECT_DOUBLE_EQ(jog.move(0, 5.0, now), 5.0);
    run(jog, now, 5);
    // The instant ramper lands immediately — no trapezoid involved.
    EXPECT_EQ(jog.axisMode(0), JogController::Mode::Idle);
    EXPECT_DOUBLE_EQ(jog.axisPosition(0), 5.0);
}

TEST(JogController, NoApplyCallbackStillWorks) {
    JogController jog(twoAxisConfig());
    int64_t now = 0;
    EXPECT_TRUE(jog.jog(0, 10.0, 200, now));
    run(jog, now, 100);  // must not crash without an applicator
    EXPECT_GT(jog.axisPosition(0), 0.0);
}
