/**
 * @file VelocityProfileTest.cpp
 * @brief Unit tests for the non-template VelocityProfile abstraction and
 *        SampledVelocityProfile.
 */

#include <tether/motion_planner/VelocityProfile.hpp>
#include <tether/motion_planner/analytical/AnalyticalSSRVelocityProfile.hpp>

#include <gtest/gtest.h>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;

TEST(VelocityProfileBase, CanReferToConcreteProfile) {
    SampledVelocityProfile sampled;
    sampled.addPoint({0.0, 0.0, 1.0, 0.0, 0.0});
    sampled.addPoint({1.0, 1.0, 1.0, 0.0, 1.0});

    // The abstract base can be used polymorphically.
    const VelocityProfile& base = sampled;
    EXPECT_EQ(base.points().size(), 2u);
    EXPECT_EQ(base.totalTime(), 1.0);
    EXPECT_EQ(base.totalLength(), 1.0);
}

TEST(SampledVelocityProfile, EmptyProfileIsZero) {
    SampledVelocityProfile profile;
    EXPECT_TRUE(profile.points().empty());
    EXPECT_EQ(profile.totalTime(), 0.0);
    EXPECT_EQ(profile.totalLength(), 0.0);
}

TEST(SampledVelocityProfile, AddPointsAndTotals) {
    SampledVelocityProfile profile;
    profile.addPoint({0.0, 0.0, 1.0, 0.0, 0.0});
    profile.addPoint({5.0, 1.0, 1.0, 0.0, 2.0});
    profile.addPoint({10.0, 0.0, -1.0, 0.0, 4.0});

    ASSERT_EQ(profile.points().size(), 3u);
    EXPECT_EQ(profile.totalLength(), 10.0);
    EXPECT_EQ(profile.totalTime(), 4.0);
}

TEST(SampledVelocityProfile, InterpolationAtExactSamples) {
    SampledVelocityProfile profile;
    profile.addPoint({0.0, 0.0, 1.0, 0.0, 0.0});
    profile.addPoint({10.0, 10.0, 2.0, 1.0, 5.0});

    EXPECT_EQ(profile.velocityAt(0.0), 0.0);
    EXPECT_EQ(profile.velocityAt(10.0), 10.0);
    EXPECT_EQ(profile.accelerationAt(0.0), 1.0);
    EXPECT_EQ(profile.accelerationAt(10.0), 2.0);
    EXPECT_EQ(profile.jerkAt(0.0), 0.0);
    EXPECT_EQ(profile.jerkAt(10.0), 1.0);
    EXPECT_EQ(profile.timeAt(0.0), 0.0);
    EXPECT_EQ(profile.timeAt(10.0), 5.0);
    EXPECT_EQ(profile.arcLengthAt(0.0), 0.0);
    EXPECT_EQ(profile.arcLengthAt(5.0), 10.0);
}

TEST(SampledVelocityProfile, InterpolationBetweenSamples) {
    SampledVelocityProfile profile;
    profile.addPoint({0.0, 0.0, 0.0, 0.0, 0.0});
    profile.addPoint({10.0, 20.0, 2.0, 0.5, 4.0});

    EXPECT_NEAR(profile.velocityAt(5.0), 10.0, 1e-9);
    EXPECT_NEAR(profile.accelerationAt(5.0), 1.0, 1e-9);
    EXPECT_NEAR(profile.jerkAt(5.0), 0.25, 1e-9);
    EXPECT_NEAR(profile.timeAt(5.0), 2.0, 1e-9);
    EXPECT_NEAR(profile.arcLengthAt(2.0), 5.0, 1e-9);
}

TEST(SampledVelocityProfile, OutOfRangeClamping) {
    SampledVelocityProfile profile;
    profile.addPoint({0.0, 0.0, 0.0, 0.0, 0.0});
    profile.addPoint({10.0, 10.0, 2.0, 0.0, 5.0});

    EXPECT_EQ(profile.velocityAt(-1.0), 0.0);
    EXPECT_EQ(profile.velocityAt(20.0), 10.0);
    EXPECT_EQ(profile.accelerationAt(-1.0), 0.0);
    EXPECT_EQ(profile.accelerationAt(20.0), 2.0);
    EXPECT_EQ(profile.timeAt(-1.0), 0.0);
    EXPECT_EQ(profile.timeAt(20.0), 5.0);
    EXPECT_EQ(profile.arcLengthAt(-1.0), 0.0);
    EXPECT_EQ(profile.arcLengthAt(10.0), 10.0);
}

TEST(SampledVelocityProfile, SinglePointIsConstant) {
    SampledVelocityProfile profile;
    profile.addPoint({2.0, 3.0, 4.0, 5.0, 6.0});

    EXPECT_EQ(profile.points().size(), 1u);
    EXPECT_EQ(profile.totalLength(), 2.0);
    EXPECT_EQ(profile.totalTime(), 6.0);

    // Before, at, and after the single point all return the same values.
    EXPECT_EQ(profile.velocityAt(-1.0), 3.0);
    EXPECT_EQ(profile.velocityAt(2.0), 3.0);
    EXPECT_EQ(profile.velocityAt(10.0), 3.0);
    EXPECT_EQ(profile.accelerationAt(0.0), 4.0);
    EXPECT_EQ(profile.jerkAt(0.0), 5.0);
    EXPECT_EQ(profile.timeAt(0.0), 6.0);
    EXPECT_EQ(profile.arcLengthAt(0.0), 2.0);
    EXPECT_EQ(profile.arcLengthAt(100.0), 2.0);
}

TEST(SampledVelocityProfile, RepeatedArcLengthDoesNotDivideByZero) {
    // Two points with the same arc length are invalid but should not crash.
    SampledVelocityProfile profile;
    profile.addPoint({5.0, 1.0, 0.0, 0.0, 0.0});
    profile.addPoint({5.0, 2.0, 0.0, 0.0, 1.0});

    // The query at that arc length just returns one of the stored values.
    EXPECT_TRUE(std::isfinite(profile.velocityAt(5.0)));
    EXPECT_EQ(profile.totalTime(), 1.0);
    EXPECT_EQ(profile.totalLength(), 5.0);
}

TEST(SampledVelocityProfile, NonMonotonicTimeRecovery) {
    // arcLengthAt uses time interpolation; non-monotonic time is still
    // invertible as long as the piece is monotonic locally.
    SampledVelocityProfile profile;
    profile.addPoint({0.0, 0.0, 0.0, 0.0, 0.0});
    profile.addPoint({10.0, 0.0, 0.0, 0.0, 5.0});
    profile.addPoint({20.0, 0.0, 0.0, 0.0, 3.0});

    // Times before and after the valid range clamp to the ends.
    EXPECT_EQ(profile.arcLengthAt(-1.0), 0.0);
    EXPECT_EQ(profile.arcLengthAt(10.0), 20.0);
}

TEST(SampledVelocityProfile, NonConstPointsMutable) {
    SampledVelocityProfile profile;
    profile.addPoint({0.0, 0.0, 0.0, 0.0, 0.0});
    profile.points().front().velocity = 5.0;
    EXPECT_EQ(profile.points().front().velocity, 5.0);
}

TEST(SampledVelocityProfile, TotalTimeAndLengthMonotonic) {
    SampledVelocityProfile profile;
    for (int i = 0; i <= 5; ++i) {
        profile.addPoint({
            static_cast<double>(i) * 2.0,
            static_cast<double>(i),
            0.0,
            0.0,
            static_cast<double>(i)});
    }
    EXPECT_EQ(profile.totalLength(), 10.0);
    EXPECT_EQ(profile.totalTime(), 5.0);
}

TEST(VelocityProfilePoint, LimitTypeValues) {
    VelocityProfilePoint pt;
    pt.limitedBy = VelocityProfilePoint::LimitType::AxisAcceleration;
    EXPECT_EQ(static_cast<uint8_t>(pt.limitedBy),
              static_cast<uint8_t>(VelocityProfilePoint::LimitType::AxisAcceleration));
    EXPECT_EQ(static_cast<uint8_t>(VelocityProfilePoint::LimitType::None), 0u);
}

TEST(AnalyticalSSRVelocityProfile, NullSourceReturnsZero) {
    // An analytical profile with no backing source must behave safely.
    AnalyticalSSRVelocityProfile<2> analytical(nullptr);
    EXPECT_EQ(analytical.totalTime(), 0.0);
    EXPECT_EQ(analytical.totalLength(), 0.0);
    EXPECT_EQ(analytical.velocityAt(1.0), 0.0);
    EXPECT_EQ(analytical.accelerationAt(1.0), 0.0);
    EXPECT_EQ(analytical.jerkAt(1.0), 0.0);
    EXPECT_EQ(analytical.timeAt(1.0), 0.0);
    EXPECT_EQ(analytical.arcLengthAt(1.0), 0.0);
    EXPECT_TRUE(analytical.points().empty());
}
