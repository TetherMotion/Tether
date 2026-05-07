#include <gtest/gtest.h>

#include "tether/motion/MotionGenerator.hpp"

using namespace Motion;

TEST(MotionControl, SineMotionGenerator_BasicRun)
{
    SineMotionGenerator sine;
    sine.configure(/*amplitude=*/1000, /*frequency=*/1.0f, /*offset=*/500);

    sine.start();
    EXPECT_TRUE(sine.isRunning());
    EXPECT_EQ(sine.getPosition(), 500); // at t=0 position == offset

    // advance 250 ms -> quarter period for 1 Hz => sin(pi/2)=1
    sine.update(250.0f);
    EXPECT_NEAR(sine.getPositionFloat(), 1500.0f, 2.0f); // allow small truncation error
    EXPECT_NEAR(sine.getVelocityFloat(), 0.0f, 200.0f); // velocity near zero at peak
}

TEST(MotionControl, SineMotionGenerator_PauseResumeAndCycles)
{
    SineMotionGenerator sine;
    sine.setAmplitude(200);
    sine.setFrequency(2.0f); // 0.5s period
    sine.setOffset(10);
    sine.setCycles(1);

    sine.start();
    sine.update(100.0f);
    float pos1 = sine.getPositionFloat();

    sine.pause();
    EXPECT_FALSE(sine.isRunning());
    sine.update(200.0f); // should have no effect while paused
    EXPECT_NEAR(sine.getPositionFloat(), pos1, 1e-3);

    sine.resume();
    EXPECT_TRUE(sine.isRunning());

    // run until cycle complete (period = 500 ms)
    for (int i = 0; i < 10 && !sine.isComplete(); ++i) {
        sine.update(100.0f);
    }
    EXPECT_TRUE(sine.isComplete());
    EXPECT_GE(sine.getCompletedCycles(), 1u);
}

TEST(MotionControl, TrapezoidalProfileGenerator_BasicProfile)
{
    TrapezoidalProfileGenerator prof;
    prof.configure(/*start=*/0, /*target=*/1000, /*max_vel=*/300, /*accel=*/1000);

    prof.start();
    EXPECT_TRUE(prof.isRunning());
    EXPECT_EQ(prof.getPosition(), 0);

    // step the generator until complete (guarded loop)
    int iterations = 0;
    while (!prof.isComplete() && iterations++ < 2000) {
        prof.update(5.0f); // 5 ms step
    }

    EXPECT_TRUE(prof.isComplete());
    EXPECT_EQ(prof.getPosition(), 1000);
    EXPECT_EQ(prof.getPhase(), TrapezoidalProfileGenerator::Phase::Complete);
}
