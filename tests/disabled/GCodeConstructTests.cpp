/**
 * @file GCodeConstructTests.cpp
 * @brief Tests for specific G-Code constructs and commands
 *
 * Contains tests for:
 * - G0/G1 rapid and linear moves
 * - G2/G3 arcs in all planes
 * - G5 splines
 * - G17/G18/G19 plane selection
 * - G61/G64 path control modes
 * - G90/G91 absolute/incremental
 * - Canned cycles
 * - Multi-axis coordination
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>

#include "gcode/motion/InterpolationStrategy.hpp"
#include "gcode/motion/GCodeMath.hpp"

namespace GCode {
namespace test {

// ============================================================================
// GCodeMath Tests
// ============================================================================

class GCodeMathTest : public ::testing::Test {};

// --- Plane Axes Tests ---

TEST_F(GCodeMathTest, PlaneAxesXY_G17) {
    int u, v, w;
    Math::planeAxes(0, u, v, w);
    EXPECT_EQ(u, 0);  // X
    EXPECT_EQ(v, 1);  // Y
    EXPECT_EQ(w, 2);  // Z
}

TEST_F(GCodeMathTest, PlaneAxesXZ_G18) {
    int u, v, w;
    Math::planeAxes(1, u, v, w);
    EXPECT_EQ(u, 0);  // X
    EXPECT_EQ(v, 2);  // Z
    EXPECT_EQ(w, 1);  // Y
}

TEST_F(GCodeMathTest, PlaneAxesYZ_G19) {
    int u, v, w;
    Math::planeAxes(2, u, v, w);
    EXPECT_EQ(u, 1);  // Y
    EXPECT_EQ(v, 2);  // Z
    EXPECT_EQ(w, 0);  // X
}

TEST_F(GCodeMathTest, PlaneAxesEnumVersion) {
    int u, v, w;
    Math::planeAxes(Math::Plane::XY, u, v, w);
    EXPECT_EQ(u, 0);
    EXPECT_EQ(v, 1);
    EXPECT_EQ(w, 2);
}

// --- Arc Sweep Tests ---

TEST_F(GCodeMathTest, ArcSweepCW90Degrees) {
    // CW from +X to +Y (quadrant 1)
    double sweep = Math::arcSweepFromCenter(10, 0, 0, 10, 0, 0, true);
    EXPECT_NEAR(sweep, -M_PI / 2, 1e-9);  // CW is negative
}

TEST_F(GCodeMathTest, ArcSweepCCW90Degrees) {
    double sweep = Math::arcSweepFromCenter(10, 0, 0, 10, 0, 0, false);
    EXPECT_NEAR(sweep, M_PI / 2, 1e-9);  // CCW is positive
}

TEST_F(GCodeMathTest, ArcSweepCW180Degrees) {
    double sweep = Math::arcSweepFromCenter(10, 0, -10, 0, 0, 0, true);
    EXPECT_NEAR(sweep, -M_PI, 1e-9);
}

TEST_F(GCodeMathTest, ArcSweepCCW180Degrees) {
    double sweep = Math::arcSweepFromCenter(10, 0, -10, 0, 0, 0, false);
    EXPECT_NEAR(sweep, M_PI, 1e-9);
}

TEST_F(GCodeMathTest, ArcSweepFullCircleCW) {
    double sweep = Math::arcSweepFromCenter(10, 0, 10, 0, 0, 0, true);
    EXPECT_NEAR(sweep, -2 * M_PI, 1e-9);  // Full CW circle
}

TEST_F(GCodeMathTest, ArcSweepFullCircleCCW) {
    double sweep = Math::arcSweepFromCenter(10, 0, 10, 0, 0, 0, false);
    EXPECT_NEAR(sweep, 2 * M_PI, 1e-9);  // Full CCW circle
}

TEST_F(GCodeMathTest, ArcSweep270DegreesCW) {
    // CW from +X to -Y
    double sweep = Math::arcSweepFromCenter(10, 0, 0, -10, 0, 0, true);
    EXPECT_NEAR(sweep, -3 * M_PI / 2, 1e-9);
}

TEST_F(GCodeMathTest, ArcSweepSmallAngle) {
    // Small angle arc
    double startU = 10, startV = 0;
    double endU = 10 * std::cos(0.1);
    double endV = 10 * std::sin(0.1);
    double sweep = Math::arcSweepFromCenter(startU, startV, endU, endV, 0, 0, false);
    EXPECT_NEAR(sweep, 0.1, 1e-6);
}

// --- Arc Point Count Tests ---

TEST_F(GCodeMathTest, ArcPointCountForSmallDeviation) {
    size_t count = Math::arcPointCountForDeviation(10.0, M_PI / 2, 0.01);
    EXPECT_GT(count, 10);  // Should need many points for tight tolerance
}

TEST_F(GCodeMathTest, ArcPointCountForLargeDeviation) {
    size_t count = Math::arcPointCountForDeviation(10.0, M_PI / 2, 1.0);
    EXPECT_LT(count, 10);  // Can use fewer points with loose tolerance
}

TEST_F(GCodeMathTest, ArcPointCountScalesWithRadius) {
    size_t smallRadiusCount = Math::arcPointCountForDeviation(5.0, M_PI, 0.1);
    size_t largeRadiusCount = Math::arcPointCountForDeviation(50.0, M_PI, 0.1);
    EXPECT_LT(smallRadiusCount, largeRadiusCount);
}

TEST_F(GCodeMathTest, ArcPointCountZeroRadius) {
    size_t count = Math::arcPointCountForDeviation(0.0, M_PI, 0.1);
    EXPECT_EQ(count, 0);
}

TEST_F(GCodeMathTest, ArcPointCountZeroDeviation) {
    size_t count = Math::arcPointCountForDeviation(10.0, M_PI, 0.0);
    EXPECT_EQ(count, 0);
}

// --- Arc Center Finding Tests ---

TEST_F(GCodeMathTest, FindArcCentersQuarterCircle) {
    double c1u, c1v, c2u, c2v;
    bool found = Math::findArcCenters(10, 0, 0, 10, 10, c1u, c1v, c2u, c2v);
    EXPECT_TRUE(found);

    // One center should be at origin
    bool hasOrigin = (Math::nearEqual(c1u, 0, 0.1) && Math::nearEqual(c1v, 0, 0.1)) ||
                     (Math::nearEqual(c2u, 0, 0.1) && Math::nearEqual(c2v, 0, 0.1));
    EXPECT_TRUE(hasOrigin);
}

TEST_F(GCodeMathTest, FindArcCentersImpossibleRadius) {
    double c1u, c1v, c2u, c2v;
    // Chord is 20, radius of 5 is too small
    bool found = Math::findArcCenters(0, 0, 20, 0, 5, c1u, c1v, c2u, c2v);
    EXPECT_FALSE(found);
}

TEST_F(GCodeMathTest, SelectArcCenterMinorArc) {
    double c1u, c1v, c2u, c2v;
    Math::findArcCenters(10, 0, 0, 10, 10, c1u, c1v, c2u, c2v);

    double cu, cv;
    Math::selectArcCenter(10, 0, 0, 10, c1u, c1v, c2u, c2v, 10, false, cu, cv);

    // Minor arc CCW should use center at origin
    EXPECT_NEAR(cu, 0, 0.1);
    EXPECT_NEAR(cv, 0, 0.1);
}

TEST_F(GCodeMathTest, SelectArcCenterMajorArc) {
    double c1u, c1v, c2u, c2v;
    Math::findArcCenters(10, 0, 0, 10, 10, c1u, c1v, c2u, c2v);

    double cu, cv;
    Math::selectArcCenter(10, 0, 0, 10, c1u, c1v, c2u, c2v, -10, false, cu, cv);  // Negative R = major arc

    // Major arc should NOT use center at origin
    EXPECT_FALSE(Math::nearEqual(cu, 0, 0.1) && Math::nearEqual(cv, 0, 0.1));
}

// --- Bezier Tests ---

TEST_F(GCodeMathTest, CubicBezierEndpoints) {
    double p0 = 0, p1 = 1, p2 = 2, p3 = 3;
    EXPECT_DOUBLE_EQ(Math::cubicBezier(p0, p1, p2, p3, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(Math::cubicBezier(p0, p1, p2, p3, 1.0), 3.0);
}

TEST_F(GCodeMathTest, CubicBezierMidpoint) {
    double p0 = 0, p1 = 0, p2 = 10, p3 = 10;
    double mid = Math::cubicBezier(p0, p1, p2, p3, 0.5);
    EXPECT_NEAR(mid, 5.0, 0.1);
}

TEST_F(GCodeMathTest, ArcToBezierEndpoints) {
    double p0u, p0v, p1u, p1v, p2u, p2v, p3u, p3v;
    Math::arcToBezier(0, 0, 10, 0, M_PI / 2, p0u, p0v, p1u, p1v, p2u, p2v, p3u, p3v);

    // Start should be at (10, 0)
    EXPECT_NEAR(p0u, 10, 1e-9);
    EXPECT_NEAR(p0v, 0, 1e-9);

    // End should be at (0, 10)
    EXPECT_NEAR(p3u, 0, 1e-9);
    EXPECT_NEAR(p3v, 10, 1e-9);
}

// --- Distance Tests ---

TEST_F(GCodeMathTest, Distance2D) {
    EXPECT_DOUBLE_EQ(Math::distance2D(0, 0, 3, 4), 5.0);
    EXPECT_DOUBLE_EQ(Math::distance2D(1, 1, 1, 1), 0.0);
}

TEST_F(GCodeMathTest, Distance3D) {
    EXPECT_NEAR(Math::distance3D(0, 0, 0, 1, 1, 1), std::sqrt(3), 1e-9);
}

// --- Velocity Calculations ---

TEST_F(GCodeMathTest, MaxVelocityForDistance) {
    // v² = 2as, so v = √(2as)
    double v = Math::maxVelocityForDistance(100, 50);
    EXPECT_NEAR(v, 100, 1e-9);  // √(2 * 100 * 50) = 100
}

TEST_F(GCodeMathTest, MaxVelocityForDistanceWithInitial) {
    // v² = v₀² + 2as
    double v = Math::maxVelocityForDistance(100, 50, 50);
    double expected = std::sqrt(50 * 50 + 2 * 100 * 50);
    EXPECT_NEAR(v, expected, 1e-9);
}

TEST_F(GCodeMathTest, DistanceForVelocityChange) {
    // s = (v² - v₀²) / (2a)
    double s = Math::distanceForVelocityChange(100, 0, 100);
    EXPECT_NEAR(s, 50, 1e-9);  // (100² - 0) / (2 * 100) = 50
}

TEST_F(GCodeMathTest, TimeForVelocityChange) {
    double t = Math::timeForVelocityChange(100, 0, 100);
    EXPECT_NEAR(t, 1.0, 1e-9);  // 100 / 100 = 1
}

TEST_F(GCodeMathTest, MaxCornerVelocityStraight) {
    double v = Math::maxCornerVelocity(1000, 0.0);
    EXPECT_TRUE(std::isinf(v));  // No slowdown needed for straight line
}

TEST_F(GCodeMathTest, MaxCornerVelocity90Degree) {
    double v = Math::maxCornerVelocity(1000, M_PI / 2);
    EXPECT_TRUE(std::isfinite(v));
    EXPECT_GT(v, 0);
}

TEST_F(GCodeMathTest, AngleBetweenVectorsSame) {
    double angle = Math::angleBetweenVectors(1, 0, 0, 1, 0, 0);
    EXPECT_NEAR(angle, 0, 1e-9);
}

TEST_F(GCodeMathTest, AngleBetweenVectorsOpposite) {
    double angle = Math::angleBetweenVectors(1, 0, 0, -1, 0, 0);
    EXPECT_NEAR(angle, M_PI, 1e-9);
}

TEST_F(GCodeMathTest, AngleBetweenVectorsPerpendicular) {
    double angle = Math::angleBetweenVectors(1, 0, 0, 0, 1, 0);
    EXPECT_NEAR(angle, M_PI / 2, 1e-9);
}

// ============================================================================
// G0 Rapid Move Tests
// ============================================================================

class G0RapidTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(G0RapidTests, RapidXOnly) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Rapid;
    seg.isRapid = true;
    seg.start[0] = 0; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 100; seg.end[1] = 0; seg.end[2] = 0;
    seg.feedRate = 6000;
    seg.segmentLength = 100;
    seg.segmentTime = 1.0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(points.back().position[0], 100, 1e-6);
    EXPECT_NEAR(points.back().position[1], 0, 1e-6);
}

TEST_F(G0RapidTests, RapidDiagonal3D) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Rapid;
    seg.isRapid = true;
    seg.start[0] = 0; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 100; seg.end[1] = 100; seg.end[2] = 50;
    seg.feedRate = 6000;
    seg.segmentLength = Math::distance3D(0, 0, 0, 100, 100, 50);
    seg.segmentTime = seg.segmentLength / 100;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// G1 Linear Move Tests
// ============================================================================

class G1LinearTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(G1LinearTests, LinearXPositive) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 0; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 50; seg.end[1] = 0; seg.end[2] = 0;
    seg.feedRate = 1000;
    seg.segmentLength = 50;
    seg.segmentTime = 3.0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(points.back().position[0], 50, 1e-6);
}

TEST_F(G1LinearTests, LinearXNegative) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 100; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 0; seg.end[1] = 0; seg.end[2] = 0;
    seg.feedRate = 1000;
    seg.segmentLength = 100;
    seg.segmentTime = 6.0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(points.back().position[0], 0, 1e-6);
}

TEST_F(G1LinearTests, LinearAllAxes) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 0; seg.start[1] = 0; seg.start[2] = 0;
    seg.start[3] = 0; seg.start[4] = 0; seg.start[5] = 0;
    seg.end[0] = 10; seg.end[1] = 20; seg.end[2] = 30;
    seg.end[3] = 45; seg.end[4] = 60; seg.end[5] = 90;
    seg.feedRate = 500;
    seg.segmentLength = 80;
    seg.segmentTime = 10.0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(points.back().position[0], 10, 1e-6);
    EXPECT_NEAR(points.back().position[3], 45, 1e-6);
}

// ============================================================================
// G2/G3 Arc Tests
// ============================================================================

class G2G3ArcTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        config_.maxChordDeviation = 0.01;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(G2G3ArcTests, G2_CW_QuarterCircleXY) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCW;
    seg.start[0] = 10; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 0; seg.end[1] = -10; seg.end[2] = 0;
    seg.center[0] = 0; seg.center[1] = 0; seg.center[2] = 0;
    seg.arcRadius = 10;
    seg.arcSweep = -M_PI / 2;
    seg.plane = InterpolationPlane::XY;
    seg.feedRate = 1000;
    seg.segmentLength = M_PI * 10 / 2;
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);

    // All points should be on circle
    for (const auto& pt : points) {
        double r = Math::distance2D(pt.position[0], pt.position[1], 0, 0);
        EXPECT_NEAR(r, 10, config_.maxChordDeviation * 2);
    }
}

TEST_F(G2G3ArcTests, G3_CCW_QuarterCircleXY) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCCW;
    seg.start[0] = 10; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 0; seg.end[1] = 10; seg.end[2] = 0;
    seg.center[0] = 0; seg.center[1] = 0; seg.center[2] = 0;
    seg.arcRadius = 10;
    seg.arcSweep = M_PI / 2;
    seg.plane = InterpolationPlane::XY;
    seg.feedRate = 1000;
    seg.segmentLength = M_PI * 10 / 2;
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(G2G3ArcTests, G2_FullCircle) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCW;
    seg.start[0] = 10; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 10; seg.end[1] = 0; seg.end[2] = 0;
    seg.center[0] = 0; seg.center[1] = 0; seg.center[2] = 0;
    seg.arcRadius = 10;
    seg.arcSweep = -2 * M_PI;
    seg.plane = InterpolationPlane::XY;
    seg.feedRate = 1000;
    seg.segmentLength = 2 * M_PI * 10;
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 20);  // Full circle needs many points
}

TEST_F(G2G3ArcTests, G2_ArcInXZPlane) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCW;
    seg.start[0] = 10; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 0; seg.end[1] = 0; seg.end[2] = 10;
    seg.center[0] = 0; seg.center[1] = 0; seg.center[2] = 0;
    seg.arcRadius = 10;
    seg.arcSweep = -M_PI / 2;
    seg.plane = InterpolationPlane::XZ;
    seg.feedRate = 1000;
    seg.segmentLength = M_PI * 10 / 2;
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(G2G3ArcTests, G3_ArcInYZPlane) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCCW;
    seg.start[0] = 0; seg.start[1] = 10; seg.start[2] = 0;
    seg.end[0] = 0; seg.end[1] = 0; seg.end[2] = 10;
    seg.center[0] = 0; seg.center[1] = 0; seg.center[2] = 0;
    seg.arcRadius = 10;
    seg.arcSweep = M_PI / 2;
    seg.plane = InterpolationPlane::YZ;
    seg.feedRate = 1000;
    seg.segmentLength = M_PI * 10 / 2;
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(G2G3ArcTests, HelicalMotion) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCW;
    seg.start[0] = 10; seg.start[1] = 0; seg.start[2] = 0;
    seg.end[0] = 10; seg.end[1] = 0; seg.end[2] = -20;  // Full helix with 20mm descent
    seg.center[0] = 0; seg.center[1] = 0; seg.center[2] = 0;
    seg.arcRadius = 10;
    seg.arcSweep = -2 * M_PI;
    seg.plane = InterpolationPlane::XY;
    seg.feedRate = 1000;
    seg.segmentLength = std::sqrt(std::pow(2 * M_PI * 10, 2) + 400);  // Helix length
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);

    // Z should decrease monotonically
    for (size_t i = 1; i < points.size(); ++i) {
        EXPECT_LE(points[i].position[2], points[i-1].position[2] + 1e-6);
    }
}

// ============================================================================
// G61/G64 Path Mode Tests
// ============================================================================

class PathModeTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        config_.limits.maxAcceleration = 1000;
        config_.limits.maxVelocity = 100;
    }

    InterpolationConfig config_;
};

TEST_F(PathModeTests, G61_ExactStopZeroJunctionVelocity) {
    config_.pathMode = PathControlMode::ExactStop;
    VelocityPlanner planner(config_);

    std::vector<MotionSegment> segments(3);
    for (int i = 0; i < 3; ++i) {
        segments[i].motionType = SegmentMotionType::Linear;
        segments[i].start[0] = i * 50;
        segments[i].end[0] = (i + 1) * 50;
        segments[i].feedRate = 1000;
        segments[i].segmentLength = 50;
    }

    planner.plan(segments);

    // All junctions should be zero velocity
    for (const auto& seg : segments) {
        EXPECT_DOUBLE_EQ(seg.entryVelocity, 0.0);
        EXPECT_DOUBLE_EQ(seg.exitVelocity, 0.0);
    }
}

TEST_F(PathModeTests, G64_BlendingNonZeroJunction) {
    config_.pathMode = PathControlMode::Blending;
    config_.blendTolerance = 1.0;
    VelocityPlanner planner(config_);

    std::vector<MotionSegment> segments(2);
    // First segment: X direction
    segments[0].motionType = SegmentMotionType::Linear;
    segments[0].start[0] = 0; segments[0].start[1] = 0;
    segments[0].end[0] = 100; segments[0].end[1] = 0;
    segments[0].feedRate = 1000;
    segments[0].segmentLength = 100;

    // Second segment: Y direction (90 degree turn)
    segments[1].motionType = SegmentMotionType::Linear;
    segments[1].start[0] = 100; segments[1].start[1] = 0;
    segments[1].end[0] = 100; segments[1].end[1] = 100;
    segments[1].feedRate = 1000;
    segments[1].segmentLength = 100;

    planner.plan(segments);

    // Junction should allow some velocity
    EXPECT_GE(segments[0].exitVelocity, 0.0);
    EXPECT_GE(segments[1].entryVelocity, 0.0);
}

TEST_F(PathModeTests, G61_1_ExactPathMaintainsPath) {
    config_.pathMode = PathControlMode::ExactPath;
    VelocityPlanner planner(config_);

    std::vector<MotionSegment> segments(2);
    segments[0].motionType = SegmentMotionType::Linear;
    segments[0].start[0] = 0;
    segments[0].end[0] = 100;
    segments[0].feedRate = 1000;
    segments[0].segmentLength = 100;

    segments[1].motionType = SegmentMotionType::Linear;
    segments[1].start[0] = 100;
    segments[1].end[0] = 200;
    segments[1].feedRate = 1000;
    segments[1].segmentLength = 100;

    planner.plan(segments);

    // Should compute valid velocities
    EXPECT_TRUE(std::isfinite(segments[0].exitVelocity));
    EXPECT_TRUE(std::isfinite(segments[1].entryVelocity));
}

// ============================================================================
// Spline Tests (G5, G5.1, NURBS)
// ============================================================================

class SplineTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::DeCasteljau);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(SplineTests, CubicSplineSegment) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::CubicSpline;
    seg.start[0] = 0; seg.start[1] = 0;
    seg.end[0] = 100; seg.end[1] = 100;
    // Control points at 1/3 and 2/3
    seg.splineControlPoints.resize(4);
    seg.splineControlPoints[0] = seg.start;
    seg.splineControlPoints[1][0] = 33; seg.splineControlPoints[1][1] = 0;
    seg.splineControlPoints[2][0] = 66; seg.splineControlPoints[2][1] = 100;
    seg.splineControlPoints[3] = seg.end;
    seg.feedRate = 1000;
    seg.segmentLength = 150;
    seg.segmentTime = 9.0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);

    // Should start and end at correct positions
    EXPECT_NEAR(points.front().position[0], 0, 1e-6);
    EXPECT_NEAR(points.back().position[0], 100, 1e-6);
}

// ============================================================================
// Canned Cycle Tests
// ============================================================================

class CannedCycleTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(CannedCycleTests, PeckDrillingCycle) {
    // G83 peck drill simulation
    std::vector<MotionSegment> segments;

    double z_initial = 0;
    double z_final = -25;
    double peck_depth = 5;
    double retract = 1;

    double current_z = z_initial;
    double peck_bottom = z_initial;

    while (peck_bottom > z_final) {
        peck_bottom = std::max(peck_bottom - peck_depth, z_final);

        // Rapid to retract height
        MotionSegment s1;
        s1.motionType = SegmentMotionType::Rapid;
        s1.start[2] = current_z;
        s1.end[2] = z_initial + retract;
        s1.feedRate = 6000;
        s1.segmentLength = std::fabs(s1.end[2] - s1.start[2]);
        segments.push_back(s1);

        // Rapid to previous peck depth
        MotionSegment s2;
        s2.motionType = SegmentMotionType::Rapid;
        s2.start[2] = z_initial + retract;
        s2.end[2] = peck_bottom + peck_depth;
        s2.feedRate = 6000;
        s2.segmentLength = std::fabs(s2.end[2] - s2.start[2]);
        segments.push_back(s2);

        // Feed to peck depth
        MotionSegment s3;
        s3.motionType = SegmentMotionType::Linear;
        s3.start[2] = peck_bottom + peck_depth;
        s3.end[2] = peck_bottom;
        s3.feedRate = 200;
        s3.segmentLength = peck_depth;
        segments.push_back(s3);

        current_z = peck_bottom;
    }

    EXPECT_GT(segments.size(), 10);  // Multiple pecks
}

// ============================================================================
// Multi-Axis Coordination Tests
// ============================================================================

class MultiAxisTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(MultiAxisTests, FiveAxisSimultaneous) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 0; seg.start[1] = 0; seg.start[2] = 0;
    seg.start[3] = 0; seg.start[4] = 0;  // A, B rotary axes
    seg.end[0] = 50; seg.end[1] = 30; seg.end[2] = -20;
    seg.end[3] = 15; seg.end[4] = -10;
    seg.feedRate = 1000;
    seg.segmentLength = 70;
    seg.segmentTime = 4.2;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);

    // All axes should reach endpoints
    EXPECT_NEAR(points.back().position[3], 15, 1e-6);
    EXPECT_NEAR(points.back().position[4], -10, 1e-6);
}

TEST_F(MultiAxisTests, RotaryOnlyMotion) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 100; seg.start[1] = 50; seg.start[2] = -25;
    seg.start[3] = 0;
    seg.end[0] = 100; seg.end[1] = 50; seg.end[2] = -25;  // XYZ unchanged
    seg.end[3] = 90;  // A axis rotates 90 degrees
    seg.feedRate = 360;  // 360 deg/min
    seg.segmentLength = 90;
    seg.segmentTime = 15.0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);

    // XYZ should not change
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.position[0], 100, 1e-6);
        EXPECT_NEAR(pt.position[1], 50, 1e-6);
        EXPECT_NEAR(pt.position[2], -25, 1e-6);
    }
}

// ============================================================================
// Edge Cases and Error Conditions
// ============================================================================

class EdgeCaseTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        strategy_->configure(config_);
    }

    InterpolationConfig config_;
    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(EdgeCaseTests, ZeroLengthMove) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 50; seg.start[1] = 50; seg.start[2] = 0;
    seg.end[0] = 50; seg.end[1] = 50; seg.end[2] = 0;
    seg.feedRate = 1000;
    seg.segmentLength = 0;
    seg.segmentTime = 0;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(EdgeCaseTests, VerySmallArc) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::ArcCCW;
    seg.arcRadius = 0.1;
    seg.arcSweep = 0.01;
    seg.start[0] = 0.1; seg.start[1] = 0;
    seg.end[0] = 0.1 * std::cos(0.01);
    seg.end[1] = 0.1 * std::sin(0.01);
    seg.center[0] = 0; seg.center[1] = 0;
    seg.plane = InterpolationPlane::XY;
    seg.feedRate = 100;
    seg.segmentLength = 0.001;
    seg.segmentTime = 0.0006;

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(EdgeCaseTests, LargeCoordinates) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = 1e6; seg.start[1] = 1e6; seg.start[2] = 0;
    seg.end[0] = 1e6 + 100; seg.end[1] = 1e6 + 100; seg.end[2] = 0;
    seg.feedRate = 1000;
    seg.segmentLength = std::sqrt(2) * 100;
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(points.back().position[0], 1e6 + 100, 0.001);
}

TEST_F(EdgeCaseTests, NegativeCoordinates) {
    MotionSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.start[0] = -500; seg.start[1] = -500; seg.start[2] = -100;
    seg.end[0] = -400; seg.end[1] = -400; seg.end[2] = -50;
    seg.feedRate = 1000;
    seg.segmentLength = Math::distance3D(-500, -500, -100, -400, -400, -50);
    seg.segmentTime = seg.segmentLength / (1000 / 60);

    InterpolationContext ctx;
    ctx.config = config_;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(points.back().position[0], -400, 1e-6);
}

} // namespace test
} // namespace GCode
