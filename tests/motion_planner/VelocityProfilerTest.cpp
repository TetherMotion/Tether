#include <gtest/gtest.h>
#include <tether/motion_planner/VelocityProfile.hpp>
#include <tether/motion_planner/BezierCurve.hpp>
#include <tether/motion_planner/PiecewiseNURBSPath.hpp>
#include <tether/motion_planner/NURBSCurve.hpp>
#include <tether/motion_planner/MathTypes.hpp>

using namespace MotionPlanner;

// Helper to create simple path using NURBS
using Path2D = PiecewiseNURBSPath<2, double>;
using Curve2D = NURBSCurve<2, double>;
using Vec2 = Vec<2, double>;

Path2D createLinearPath(double length) {
    Path2D path;
    // Linear segment from (0,0) to ({length}, 0)
    auto line = Curve2D::makeLine(Vec2{0.0, 0.0}, Vec2{length, 0.0});
    path.appendSegment(line);
    path.buildArcLengthTables();
    return path;
}

TEST(VelocityProfilerTest, InitialAcceleration) {
    VelocityProfiler2D profiler;
    auto path = createLinearPath(100.0);
    double feedRate = 50.0;
    
    // Test with non-zero initial acceleration
    double startAccel = 10.0;
    auto profile = profiler.computeProfile(
        path, 
        feedRate, 
        0.0, // startVel
        0.0, // endVel
        100, // numSamples
        startAccel // startAccel
    );
    
    ASSERT_GT(profile.points().size(), 0);
    // Use floating point comparison or exact?
    // startAccel is passed directly, so exact match expected
    EXPECT_EQ(profile.points()[0].acceleration, startAccel);
    
    // Check that standard call still defaults to 0
    auto profileDefault = profiler.computeProfile(path, feedRate);
    ASSERT_GT(profileDefault.points().size(), 0);
    EXPECT_EQ(profileDefault.points()[0].acceleration, 0.0);
}

TEST(VelocityProfilerTest, InitialJerkArgument) {
    // Just verify the argument is accepted and doesn't crash
    VelocityProfiler2D profiler;
    auto path = createLinearPath(100.0);
    
    auto profile = profiler.computeProfile(
        path, 
        50.0, 
        0.0, 
        0.0, 
        100, 
        10.0, // startAccel
        5.0   // startJerk
    );
    
    ASSERT_GT(profile.points().size(), 0);
    EXPECT_EQ(profile.points()[0].acceleration, 10.0);
}
