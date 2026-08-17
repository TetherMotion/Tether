#include <gtest/gtest.h>

#include "tether/control/SineMotionController.hpp"

using namespace tether::control;

TEST(SineMotionController, BasicPositionVelocity)
{
    SineMotionController::Config cfg;
    cfg.amplitude = 1.0;              // 1 radian peak
    cfg.frequency = 1.0;              // 1 Hz
    cfg.offset = 0.0;
    cfg.phaseOffset = 0.0;
    SineMotionController ctrl(cfg);

    ctrl.start();
    EXPECT_TRUE(ctrl.getState().running);
    EXPECT_DOUBLE_EQ(ctrl.getPosition(), 0.0); // at t=0, offset

    // advance 0.25 s: quarter period -> sin(pi/2)=1 -> position=amplitude
    double pos = ctrl.update(0.25);
    EXPECT_NEAR(pos, 1.0, 1e-6);
    EXPECT_NEAR(ctrl.getVelocity(), 0.0, 1e-3);

    // scaled helpers should match manual multiplication
    EXPECT_EQ(ctrl.getPositionScaled(100), static_cast<int32_t>(pos * 100));
    EXPECT_EQ(ctrl.getVelocityScaled(100), static_cast<int32_t>(ctrl.getVelocity() * 100));
}

TEST(SineMotionController, OffsetAndPhase)
{
    SineMotionController ctrl;
    ctrl.setAmplitude(2.0, true);
    ctrl.setFrequency(0.5, true);   // 2s period
    ctrl.setOffset(1.0, true);
    ctrl.setPhaseOffset(M_PI/2, true);

    ctrl.start();
    // call update(0) to initialize state
    ctrl.update(0.0);
    EXPECT_NEAR(ctrl.getPosition(), 1.0 + 2.0 * std::sin(M_PI/2), 1e-6);

    // after one quarter of period (0.5s) phase increases by pi/2 again
    ctrl.update(0.5);
    double pos2 = ctrl.getPosition();
    double expected = 1.0 + 2.0 * std::sin(M_PI);
    EXPECT_NEAR(pos2, expected, 1e-6);
}

TEST(SineMotionController, Ramping)
{
    SineMotionController ctrl;
    ctrl.setAmplitude(1.0, true);
    ctrl.setFrequency(1.0, true);
    ctrl.setRampTime(1.0); // one second ramp
    ctrl.start();

    // change amplitude and frequency with finite ramp time
    ctrl.setAmplitude(2.0, false);
    ctrl.setFrequency(2.0, false);

    // update for 0.5 seconds - parameters should be halfway
    ctrl.update(0.5);
    EXPECT_NEAR(ctrl.getAmplitude(), 1.5, 0.01);
    EXPECT_NEAR(ctrl.getFrequency(), 1.5, 0.01);
}
