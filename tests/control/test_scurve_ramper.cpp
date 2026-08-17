#include <gtest/gtest.h>

#include "tether/control/ParameterRamping.hpp"
using namespace tether::control;

// Verify that the default configuration for the S-curve ramper matches the
// newly-specified 10x‑faster values and that a ramper built with the default
// profile behaves sensibly (reaches its target without exceeding limits).

TEST(SCurveRamper, DefaultConfig)
{
    auto cfg = SCurveRamper::Config::getDefault();

    EXPECT_DOUBLE_EQ(cfg.maxVelocity, 10.0);
    EXPECT_DOUBLE_EQ(cfg.maxAcceleration, 20.0);
    EXPECT_DOUBLE_EQ(cfg.maxJerk, 100.0);
    EXPECT_DOUBLE_EQ(cfg.tolerance, 1e-9);
}

TEST(SCurveRamper, ReachTargetWithVelocityLimit)
{
    auto cfg = SCurveRamper::Config::getDefault();
    SCurveRamper ramper(cfg);

    ramper.reset(0.0);
    ramper.setTarget(5.0); // arbitrary distance

    double dt = 0.001;
    double time = 0.0;
    while (!ramper.isComplete() && time < 5.0) {
        ramper.update(dt);
        // velocity should never exceed the configured max (allow small epsilon)
        EXPECT_LE(std::abs(ramper.getVelocity()), cfg.maxVelocity + 1e-6);
        time += dt;
    }

    EXPECT_TRUE(ramper.isComplete());
    EXPECT_NEAR(ramper.getValue(), 5.0, 1e-5);
}
