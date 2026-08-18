#include <gtest/gtest.h>

#include "tether/control/StepMotionController.hpp"
using namespace tether::control;

// Basic sequence test: two steps with hold periods.  Verify final
// position and that each hold interval occurred.
TEST(StepMotionController, SequenceProgression)
{
    StepMotionController::Config cfg;
    // first step: move +1.0 rad, hold 0.1s
    StepMotionController::Step s1;
    s1.displacement = 1.0;
    s1.holdTime = 0.1;
    s1.profile.maxVelocity = 10.0;   // fast so move completes quickly
    s1.profile.maxAcceleration = 20.0;
    s1.profile.maxJerk = 100.0;

    // second step: move +2.0 rad, hold 0.1s
    StepMotionController::Step s2;
    s2.displacement = 2.0;
    s2.holdTime = 0.1;
    s2.profile.maxVelocity = 20.0;
    s2.profile.maxAcceleration = 40.0;
    s2.profile.maxJerk = 200.0;

    cfg.steps = {s1, s2};
    cfg.initialPosition = 0.0;
    cfg.loop = false;

    StepMotionController ctrl(cfg);
    ctrl.start();

    double t = 0.0;
    double dt = 0.001;
    bool sawFirstHold = false;
    bool sawSecondHold = false;

    while (ctrl.isRunning() && t < 5.0) {
        double pos_before = ctrl.getPosition();
        double pos = ctrl.update(dt);
        t += dt;

        // detect hold by checking that position stops changing
        if (!sawFirstHold && t > 0.01 && fabs(pos - 1.0) < 1e-3) {
            sawFirstHold = true;
        }
        if (sawFirstHold && !sawSecondHold && t > 0.5 && fabs(pos - 3.0) < 1e-3) {
            sawSecondHold = true;
        }

        // ensure we never overshoot beyond expected max
        EXPECT_LE(pos, 3.01);
    }

    EXPECT_TRUE(sawFirstHold);
    EXPECT_TRUE(sawSecondHold);
    EXPECT_FALSE(ctrl.isRunning());
    EXPECT_NEAR(ctrl.getPosition(), 3.0, 1e-6);
}

// Verify loop mode continues past a single iteration
TEST(StepMotionController, Looping)
{
    StepMotionController::Config cfg;
    StepMotionController::Step s;
    s.displacement = 0.5;
    s.holdTime = 0.01;
    s.profile.maxVelocity = 50.0;
    s.profile.maxAcceleration = 500.0;
    s.profile.maxJerk = 5000.0;
    cfg.steps = {s};
    cfg.loop = true;
    cfg.initialPosition = 0.0;

    StepMotionController ctrl(cfg);
    ctrl.start();

    double t = 0.0;
    double dt = 0.001;

    // run for 0.5s — each loop adds 0.5 rad, should complete several loops
    while (t < 0.5) {
        ctrl.update(dt);
        t += dt;
    }

    // position should exceed single-step displacement (looped multiple times)
    EXPECT_GT(ctrl.getPosition(), 0.5);
    EXPECT_TRUE(ctrl.isRunning());
}

// Verify that a sequence containing two speed sets produces the expected
// approximate timings: first three steps about 0.5s each, second three steps
// about 0.25s each.  We don't need perfect accuracy, just order-of-magnitude.
TEST(StepMotionController, TwoSpeedProfile)
{
    using Step = StepMotionController::Step;
    StepMotionController::Config cfg;
    // helper same as example
    auto makeStep = [&](double disp, double totalTime, double accelTime) {
        Step s;
        s.displacement = disp;
        s.holdTime = 0.01; // small hold so timings don't dominate
        double vmax = disp / (totalTime - accelTime);
        double a = vmax / accelTime;
        s.profile.maxVelocity = vmax;
        s.profile.maxAcceleration = a;
        s.profile.maxJerk = a * 4.0;
        return s;
    };

    double oneEighth = 2.0 * M_PI / 8.0;
    double oneQuarter = 2.0 * M_PI / 4.0;
    double halfTurn = M_PI;

    cfg.steps.push_back(makeStep(oneEighth, 0.5, 0.05));
    cfg.steps.push_back(makeStep(oneQuarter, 0.5, 0.05));
    cfg.steps.push_back(makeStep(halfTurn, 0.5, 0.05));
    cfg.steps.push_back(makeStep(oneEighth, 0.25, 0.025));
    cfg.steps.push_back(makeStep(oneQuarter, 0.25, 0.025));
    cfg.steps.push_back(makeStep(halfTurn, 0.25, 0.025));
    cfg.initialPosition = 0.0;
    cfg.loop = false;

    StepMotionController ctrl(cfg);
    ctrl.start();

    double t = 0.0;
    double dt = 0.001;
    std::array<double,6> times{};
    size_t stepIndex = 0;

    while ((stepIndex < 6) && t < 5.0) {
        double pos = ctrl.update(dt);
        t += dt;
        double targetPos = 0.0;
        for (size_t i = 0; i <= stepIndex; ++i) {
            targetPos += cfg.steps[i].displacement;
        }
        if (stepIndex < 6 && fabs(pos - targetPos) < 1e-3) {
            times[stepIndex] = t;
            stepIndex++;
        }
    }

    // compute durations
    std::array<double,6> durs;
    durs[0] = times[0];
    for (int i = 1; i < 6; ++i) {
        durs[i] = times[i] - times[i-1];
    }

    // first three should be about 0.5s (±0.1), next three ~0.25s (±0.1)
    EXPECT_NEAR(durs[0], 0.5, 0.1);
    EXPECT_NEAR(durs[1], 0.5, 0.1);
    EXPECT_NEAR(durs[2], 0.5, 0.1);
    EXPECT_NEAR(durs[3], 0.25, 0.1);
    EXPECT_NEAR(durs[4], 0.25, 0.1);
    EXPECT_NEAR(durs[5], 0.25, 0.1);

    // finish any remaining hold period
    while (ctrl.isRunning() && t < 10.0) {
        ctrl.update(dt);
        t += dt;
    }
    EXPECT_FALSE(ctrl.isRunning());
}
