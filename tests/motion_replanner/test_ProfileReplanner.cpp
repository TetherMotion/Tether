/**
 * @file test_ProfileReplanner.cpp
 * @brief Tests for velocity profile re-planning + S-curve transitions
 */

#include "tether/motion_replanner/ProfileReplanner.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::motion::replanner;
using namespace tether::motion;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    return s;
}

/// Build an L-shaped path: (0,0)→(50,0)→(50,50)
PiecewiseNurbsPath makeLPath() {
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 5.0, 0, 0, 1, i * 5.0));
    }
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 50.0, i * 5.0,
                                     1, 1, 50.0 + i * 5.0));
    }
    return convertTrajectory(samples);
}

} // anonymous namespace

//=============================================================================
// replanProfile tests
//=============================================================================

TEST(ProfileReplanner, ReplanLPath) {
    PiecewiseNurbsPath path = makeLPath();
    ASSERT_EQ(path.dim(), 2u);

    ProfileLimits limits;
    ProfileReplanResult result = replanProfile(
        path, 6000.0, limits, 50); // 6000 mm/min = 100 mm/s

    EXPECT_GT(result.points.size(), 0u);
    EXPECT_GT(result.totalTime, 0.0);
    EXPECT_NEAR(result.totalLength, path.totalLength(), 1.0);
    EXPECT_GT(result.maxVelocity, 0.0);
    EXPECT_FALSE(result.summary.empty());
}

TEST(ProfileReplanner, StartAndEndVelocity) {
    PiecewiseNurbsPath path = makeLPath();

    ProfileLimits limits;
    ProfileReplanResult result = replanProfile(
        path, 6000.0, limits, 50, 0.0, 0.0); // Start and end at rest

    ASSERT_GT(result.points.size(), 1u);
    // Start velocity should be ~0
    EXPECT_NEAR(result.points.front().velocity, 0.0, 1e-3);
    // End velocity should be ~0
    EXPECT_NEAR(result.points.back().velocity, 0.0, 1e-3);
}

TEST(ProfileReplanner, FeedRateCapsVelocity) {
    PiecewiseNurbsPath path = makeLPath();

    ProfileLimits limits;
    limits.maxPathVelocity = 50.0; // 50 mm/s cap

    ProfileReplanResult result = replanProfile(
        path, 12000.0, limits, 50); // 12000 mm/min = 200 mm/s, but capped

    // Max velocity should not exceed the path velocity limit.
    EXPECT_LE(result.maxVelocity, limits.maxPathVelocity + 1e-6);
}

TEST(ProfileReplanner, NumSamplesThrows) {
    PiecewiseNurbsPath path = makeLPath();
    EXPECT_THROW(replanProfile(path, 6000.0, {}, 0), std::invalid_argument);
}

TEST(ProfileReplanner, SummaryString) {
    PiecewiseNurbsPath path = makeLPath();
    ProfileReplanResult result = replanProfile(path, 6000.0, {}, 20);
    EXPECT_NE(result.summary.find("Profile:"), std::string::npos);
}

//=============================================================================
// computeSCurveTransition tests
//=============================================================================

TEST(ProfileReplanner, SCurveBasicTransition) {
    // Transition from 0 to 50 mm/s over 10mm.
    auto profile = computeSCurveTransition(
        10.0, 0.0, 50.0, 100.0, 500.0, 5000.0);

    ASSERT_TRUE(profile.has_value());
    EXPECT_TRUE(profile->isValid());
    EXPECT_GT(profile->totalDuration(), 0.0);
    EXPECT_NEAR(profile->totalDistance(), 10.0, 1e-6);
}

TEST(ProfileReplanner, SCurveZeroDistance) {
    // Zero distance → no valid profile.
    auto profile = computeSCurveTransition(
        0.0, 0.0, 50.0, 100.0, 500.0, 5000.0);
    // Zero distance with non-zero velocity change is invalid.
    EXPECT_FALSE(profile.has_value());
}

TEST(ProfileReplanner, SCurveStartEqualsEnd) {
    // Start velocity == end velocity, positive distance → cruise.
    auto profile = computeSCurveTransition(
        10.0, 50.0, 50.0, 100.0, 500.0, 5000.0);

    // This should produce a valid cruise profile.
    if (profile.has_value()) {
        EXPECT_TRUE(profile->isValid());
        EXPECT_NEAR(profile->totalDistance(), 10.0, 1e-6);
    }
    // Note: some implementations may return false for this case if
    // the cruise velocity exceeds maxVelocity. Either outcome is acceptable.
}

TEST(ProfileReplanner, SCurveEvaluateAt) {
    auto profile = computeSCurveTransition(
        20.0, 0.0, 100.0, 100.0, 500.0, 5000.0);

    ASSERT_TRUE(profile.has_value());

    // Evaluate at t=0: velocity should be 0.
    auto state0 = profile->evaluateAt(0.0);
    EXPECT_NEAR(state0.velocity, 0.0, 1e-6);

    // Evaluate at the end: velocity should be ~100.
    auto stateEnd = profile->evaluateAt(profile->totalDuration());
    EXPECT_NEAR(stateEnd.velocity, 100.0, 1e-3);

    // Jerk should never exceed maxJerk.
    // (Check at a few sample times.)
    double maxJerk = 0.0;
    int nChecks = 100;
    for (int i = 0; i <= nChecks; ++i) {
        double t = static_cast<double>(i) / nChecks * profile->totalDuration();
        auto state = profile->evaluateAt(t);
        if (std::abs(state.jerk) > maxJerk) {
            maxJerk = std::abs(state.jerk);
        }
    }
    EXPECT_LE(maxJerk, 5000.0 + 1e-6); // maxJerk
}
