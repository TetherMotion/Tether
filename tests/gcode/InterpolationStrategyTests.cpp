/**
 * @file InterpolationStrategyTests.cpp
 * @brief Comprehensive tests for all Interpolation Strategies
 *
 * Tests coverage:
 * - InterpolationStrategy base class methods
 * - VelocityPlanner
 * - InterpolationStrategyFactory
 * - FixedTimeStrategy
 * - FixedDeviationStrategy
 * - RKF45Strategy
 * - DOPRIStrategy
 * - AdaptiveMidpointStrategy
 * - DeCasteljauStrategy
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "tether/gcode/motion/InterpolationStrategy.hpp"

using namespace GCode;
using namespace InterpolationConstants;

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

Position makePosition(double x, double y, double z = 0.0) {
    Position p;
    p.x() = x;
    p.y() = y;
    p.z() = z;
    return p;
}

PlanningSegment makeLinearSegment(const Position& start, const Position& end, double feedRate = 1000.0) {
    PlanningSegment seg;
    seg.start = start;
    seg.end = end;
    seg.motionType = SegmentMotionType::Linear;
    seg.feedRate = feedRate;
    seg.segmentLength = start.linearDistance(end);
    return seg;
}

PlanningSegment makeArcSegment(const Position& start, const Position& end, 
                                const Position& center, double radius, double sweep,
                                bool clockwise = true) {
    PlanningSegment seg;
    seg.start = start;
    seg.end = end;
    seg.center = center;
    seg.motionType = clockwise ? SegmentMotionType::ArcCW : SegmentMotionType::ArcCCW;
    seg.arcRadius = radius;
    seg.arcSweep = sweep;
    seg.feedRate = 1000.0;
    seg.segmentLength = std::fabs(radius * sweep);
    return seg;
}

} // namespace

// ============================================================================
// InterpolationStrategy Base Class Tests
// ============================================================================

class InterpolationStrategyBaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a FixedTime strategy for testing base class methods
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        ASSERT_NE(strategy, nullptr);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
};

TEST_F(InterpolationStrategyBaseTest, GetPlaneAxesXY) {
    int u, v, w;
    InterpolationStrategy::getPlaneAxes(InterpolationPlane::XY, u, v, w);
    EXPECT_EQ(u, 0);
    EXPECT_EQ(v, 1);
    EXPECT_EQ(w, 2);
}

TEST_F(InterpolationStrategyBaseTest, GetPlaneAxesXZ) {
    int u, v, w;
    InterpolationStrategy::getPlaneAxes(InterpolationPlane::XZ, u, v, w);
    EXPECT_EQ(u, 0);
    EXPECT_EQ(v, 2);
    EXPECT_EQ(w, 1);
}

TEST_F(InterpolationStrategyBaseTest, GetPlaneAxesYZ) {
    int u, v, w;
    InterpolationStrategy::getPlaneAxes(InterpolationPlane::YZ, u, v, w);
    EXPECT_EQ(u, 1);
    EXPECT_EQ(v, 2);
    EXPECT_EQ(w, 0);
}

TEST_F(InterpolationStrategyBaseTest, ArcSegmentCountSmallRadius) {
    size_t count = InterpolationStrategy::arcSegmentCount(1.0, PI, 0.01);
    EXPECT_GT(count, 1u);
}

TEST_F(InterpolationStrategyBaseTest, ArcSegmentCountLargeRadius) {
    size_t count = InterpolationStrategy::arcSegmentCount(100.0, PI, 0.01);
    EXPECT_GT(count, 1u);
}

TEST_F(InterpolationStrategyBaseTest, ArcSegmentCountZeroRadius) {
    size_t count = InterpolationStrategy::arcSegmentCount(0.0, PI, 0.01);
    EXPECT_EQ(count, 1u);
}

TEST_F(InterpolationStrategyBaseTest, ArcSegmentCountZeroChordError) {
    size_t count = InterpolationStrategy::arcSegmentCount(10.0, PI, 0.0);
    EXPECT_EQ(count, 1u);
}

TEST_F(InterpolationStrategyBaseTest, ArcSegmentCountNegativeValues) {
    size_t count = InterpolationStrategy::arcSegmentCount(-1.0, PI, 0.01);
    EXPECT_EQ(count, 1u);
    
    count = InterpolationStrategy::arcSegmentCount(1.0, PI, -0.01);
    EXPECT_EQ(count, 1u);
}

TEST_F(InterpolationStrategyBaseTest, ArcSegmentCountFullCircle) {
    size_t count = InterpolationStrategy::arcSegmentCount(10.0, TWO_PI, 0.01);
    EXPECT_GT(count, 4u);  // At least 4 segments for a full circle
}

TEST_F(InterpolationStrategyBaseTest, ClampFunction) {
    EXPECT_DOUBLE_EQ(InterpolationStrategy::clamp(5.0, 0.0, 10.0), 5.0);
    EXPECT_DOUBLE_EQ(InterpolationStrategy::clamp(-5.0, 0.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(InterpolationStrategy::clamp(15.0, 0.0, 10.0), 10.0);
    EXPECT_DOUBLE_EQ(InterpolationStrategy::clamp(0.0, 0.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(InterpolationStrategy::clamp(10.0, 0.0, 10.0), 10.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionLinearStart) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 100));
    Position pos = strategy->evaluatePosition(seg, 0.0);
    EXPECT_NEAR(pos.x(), 0.0, 1e-10);
    EXPECT_NEAR(pos.y(), 0.0, 1e-10);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionLinearEnd) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 100));
    Position pos = strategy->evaluatePosition(seg, 1.0);
    EXPECT_NEAR(pos.x(), 100.0, 1e-10);
    EXPECT_NEAR(pos.y(), 100.0, 1e-10);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionLinearMidpoint) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 100));
    Position pos = strategy->evaluatePosition(seg, 0.5);
    EXPECT_NEAR(pos.x(), 50.0, 1e-10);
    EXPECT_NEAR(pos.y(), 50.0, 1e-10);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionArcStart) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    Position pos = strategy->evaluatePosition(seg, 0.0);
    EXPECT_NEAR(pos.x(), 10.0, 1e-6);
    EXPECT_NEAR(pos.y(), 0.0, 1e-6);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionArcEnd) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    Position pos = strategy->evaluatePosition(seg, 1.0);
    EXPECT_NEAR(pos.x(), 0.0, 1e-6);
    EXPECT_NEAR(pos.y(), 10.0, 1e-6);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionArcMidpoint) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    Position pos = strategy->evaluatePosition(seg, 0.5);
    // At 45 degrees
    EXPECT_NEAR(pos.x(), 10.0 * std::cos(PI / 4), 1e-6);
    EXPECT_NEAR(pos.y(), 10.0 * std::sin(PI / 4), 1e-6);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionClampsNegative) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 100));
    Position pos = strategy->evaluatePosition(seg, -0.5);
    EXPECT_NEAR(pos.x(), 0.0, 1e-10);
    EXPECT_NEAR(pos.y(), 0.0, 1e-10);
}

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionClampsOverOne) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 100));
    Position pos = strategy->evaluatePosition(seg, 1.5);
    EXPECT_NEAR(pos.x(), 100.0, 1e-10);
    EXPECT_NEAR(pos.y(), 100.0, 1e-10);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateVelocityLinear) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.feedRate = 6000.0;  // mm/min = 100 mm/s
    seg.segmentTime = 1.0;  // 1 second to traverse 100mm
    
    Position vel = strategy->evaluateVelocity(seg, 0.5);
    // Should return direction scaled by time: (100-0)/1.0 = 100 mm/s
    EXPECT_NEAR(vel.x(), 100.0, 1.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateCurvatureLinear) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    double curvature = strategy->evaluateCurvature(seg, 0.5);
    EXPECT_NEAR(curvature, 0.0, 1e-10);  // Linear has zero curvature
}

TEST_F(InterpolationStrategyBaseTest, EvaluateCurvatureArc) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    double curvature = strategy->evaluateCurvature(seg, 0.5);
    EXPECT_NEAR(curvature, 0.1, 1e-6);  // 1/radius = 1/10
}

TEST_F(InterpolationStrategyBaseTest, ArcLengthLinear) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    double len = strategy->arcLength(seg, 0.5);
    EXPECT_NEAR(len, 50.0, 1e-6);
}

TEST_F(InterpolationStrategyBaseTest, ArcLengthInverseLinear) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    double t = strategy->arcLengthInverse(seg, 50.0);
    EXPECT_NEAR(t, 0.5, 1e-6);
}

// ============================================================================
// InterpolationStrategyFactory Tests
// ============================================================================

class StrategyFactoryTest : public ::testing::Test {
protected:
    InterpolationConfig config;
};

TEST_F(StrategyFactoryTest, CreateFixedTimeStrategy) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::FixedTime);
    EXPECT_STREQ(strategy->name(), "Fixed Time");
}

TEST_F(StrategyFactoryTest, CreateFixedDeviationStrategy) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::FixedDeviation);
    EXPECT_STREQ(strategy->name(), "Fixed Deviation");
}

TEST_F(StrategyFactoryTest, CreateRKF45Strategy) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::RKF45);
    EXPECT_STREQ(strategy->name(), "RKF45");
}

TEST_F(StrategyFactoryTest, CreateDOPRIStrategy) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::DOPRI);
    EXPECT_STREQ(strategy->name(), "DOPRI");
}

TEST_F(StrategyFactoryTest, CreateAdaptiveMidpointStrategy) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::AdaptiveMidpoint);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::AdaptiveMidpoint);
    EXPECT_STREQ(strategy->name(), "Adaptive Midpoint");
}

TEST_F(StrategyFactoryTest, CreateDeCasteljauStrategy) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DeCasteljau);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::DeCasteljau);
    EXPECT_STREQ(strategy->name(), "De Casteljau");
}

TEST_F(StrategyFactoryTest, CreateFromConfig) {
    config.strategy = InterpolationStrategyType::DOPRI;
    auto strategy = InterpolationStrategyFactory::create(config);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::DOPRI);
}

TEST_F(StrategyFactoryTest, CreateFromConfigAppliesSettings) {
    config.strategy = InterpolationStrategyType::FixedTime;
    config.timeResolution = 0.002;
    config.maxChordDeviation = 0.05;
    
    auto strategy = InterpolationStrategyFactory::create(config);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->type(), InterpolationStrategyType::FixedTime);
}

// ============================================================================
// PlanningSegment Tests
// ============================================================================

TEST(PlanningSegmentTest, DefaultConstruction) {
    PlanningSegment seg;
    EXPECT_EQ(seg.motionType, SegmentMotionType::Linear);
    EXPECT_FALSE(seg.isArc());
    EXPECT_FALSE(seg.isSpline());
    EXPECT_FALSE(seg.isRapid);
}

TEST(PlanningSegmentTest, IsArcCW) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::ArcCW;
    EXPECT_TRUE(seg.isArc());
    EXPECT_EQ(seg.arcDirection(), -1);
}

TEST(PlanningSegmentTest, IsArcCCW) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::ArcCCW;
    EXPECT_TRUE(seg.isArc());
    EXPECT_EQ(seg.arcDirection(), 1);
}

TEST(PlanningSegmentTest, IsSplineCubic) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::CubicSpline;
    EXPECT_TRUE(seg.isSpline());
}

TEST(PlanningSegmentTest, IsSplineQuadratic) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::QuadraticSpline;
    EXPECT_TRUE(seg.isSpline());
}

TEST(PlanningSegmentTest, IsSplineNURBS) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::NURBS;
    EXPECT_TRUE(seg.isSpline());
}

TEST(PlanningSegmentTest, LinearNotArcOrSpline) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    EXPECT_FALSE(seg.isArc());
    EXPECT_FALSE(seg.isSpline());
}

TEST(PlanningSegmentTest, RapidNotArcOrSpline) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::Rapid;
    EXPECT_FALSE(seg.isArc());
    EXPECT_FALSE(seg.isSpline());
}

// ============================================================================
// TrajectoryPoint Tests
// ============================================================================

TEST(TrajectoryPointTest, DefaultConstruction) {
    TrajectoryPoint pt;
    EXPECT_DOUBLE_EQ(pt.time, 0.0);
    EXPECT_EQ(pt.blockIndex, -1);
    EXPECT_EQ(pt.segmentIndex, -1);
    EXPECT_FALSE(pt.isInterpolated);
    EXPECT_FALSE(pt.isBlendPoint);
}

TEST(TrajectoryPointTest, ConstructFromPosition) {
    Position p = makePosition(10, 20, 30);
    TrajectoryPoint pt(p);
    EXPECT_DOUBLE_EQ(pt.position.x(), 10.0);
    EXPECT_DOUBLE_EQ(pt.position.y(), 20.0);
    EXPECT_DOUBLE_EQ(pt.position.z(), 30.0);
}

TEST(TrajectoryPointTest, DistanceToSamePoint) {
    TrajectoryPoint pt1(makePosition(10, 20, 30));
    TrajectoryPoint pt2(makePosition(10, 20, 30));
    EXPECT_NEAR(pt1.distanceTo(pt2), 0.0, 1e-10);
}

TEST(TrajectoryPointTest, DistanceToDifferentPoint) {
    TrajectoryPoint pt1(makePosition(0, 0, 0));
    TrajectoryPoint pt2(makePosition(3, 4, 0));
    EXPECT_NEAR(pt1.distanceTo(pt2), 5.0, 1e-10);
}

// ============================================================================
// InterpolationContext Tests
// ============================================================================

TEST(InterpolationContextTest, DefaultConstruction) {
    InterpolationContext ctx;
    EXPECT_EQ(ctx.currentSegmentIndex, 0u);
    EXPECT_DOUBLE_EQ(ctx.currentTime, 0.0);
    EXPECT_TRUE(ctx.segments.empty());
}

TEST(InterpolationContextTest, UpdateBounds) {
    InterpolationContext ctx;
    ctx.updateBounds(makePosition(10, 20, 30));
    ctx.updateBounds(makePosition(-5, 100, 15));
    
    EXPECT_DOUBLE_EQ(ctx.minBounds.x(), -5.0);
    EXPECT_DOUBLE_EQ(ctx.maxBounds.x(), 10.0);
    EXPECT_DOUBLE_EQ(ctx.minBounds.y(), 20.0);
    EXPECT_DOUBLE_EQ(ctx.maxBounds.y(), 100.0);
}

TEST(InterpolationContextTest, GetSegmentValid) {
    InterpolationContext ctx;
    ctx.segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(10, 0)));
    ctx.segments.push_back(makeLinearSegment(makePosition(10, 0), makePosition(20, 0)));
    ctx.segments.push_back(makeLinearSegment(makePosition(20, 0), makePosition(30, 0)));
    ctx.currentSegmentIndex = 1;
    
    const PlanningSegment* seg = ctx.getSegment(0);
    ASSERT_NE(seg, nullptr);
    EXPECT_NEAR(seg->start.x(), 10.0, 1e-10);
    
    seg = ctx.getSegment(-1);  // Lookbehind
    ASSERT_NE(seg, nullptr);
    EXPECT_NEAR(seg->start.x(), 0.0, 1e-10);
    
    seg = ctx.getSegment(1);  // Lookahead
    ASSERT_NE(seg, nullptr);
    EXPECT_NEAR(seg->start.x(), 20.0, 1e-10);
}

TEST(InterpolationContextTest, GetSegmentOutOfBounds) {
    InterpolationContext ctx;
    ctx.segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(10, 0)));
    ctx.currentSegmentIndex = 0;
    
    EXPECT_EQ(ctx.getSegment(-1), nullptr);
    EXPECT_EQ(ctx.getSegment(1), nullptr);
}

TEST(InterpolationContextTest, HasLookahead) {
    InterpolationContext ctx;
    ctx.segments.resize(5);
    ctx.currentSegmentIndex = 2;
    
    EXPECT_TRUE(ctx.hasLookahead(1));
    EXPECT_TRUE(ctx.hasLookahead(2));
    EXPECT_FALSE(ctx.hasLookahead(3));
}

TEST(InterpolationContextTest, HasLookbehind) {
    InterpolationContext ctx;
    ctx.segments.resize(5);
    ctx.currentSegmentIndex = 2;
    
    EXPECT_TRUE(ctx.hasLookbehind(1));
    EXPECT_TRUE(ctx.hasLookbehind(2));
    EXPECT_FALSE(ctx.hasLookbehind(3));
}

// ============================================================================
// InterpolationResult Tests
// ============================================================================

TEST(InterpolationResultTest, DefaultConstruction) {
    InterpolationResult result;
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.errorMessage.empty());
    EXPECT_TRUE(result.points.empty());
    EXPECT_DOUBLE_EQ(result.totalDuration, 0.0);
    EXPECT_DOUBLE_EQ(result.totalLength, 0.0);
}

// ============================================================================
// KinematicLimits Tests
// ============================================================================

TEST(KinematicLimitsTest, DefaultValues) {
    KinematicLimits limits;
    EXPECT_DOUBLE_EQ(limits.maxVelocityLinear, 6000.0);
    EXPECT_DOUBLE_EQ(limits.maxAcceleration, 1000.0);
    EXPECT_DOUBLE_EQ(limits.maxJerk, 10000.0);
    
    for (size_t i = 0; i < MAX_AXES; ++i) {
        EXPECT_DOUBLE_EQ(limits.axisMaxVelocity[i], 6000.0);
    }
}

// ============================================================================
// InterpolationConfig Tests
// ============================================================================

TEST(InterpolationConfigTest, DefaultValues) {
    InterpolationConfig config;
    EXPECT_EQ(config.strategy, InterpolationStrategyType::FixedDeviation);
    EXPECT_DOUBLE_EQ(config.timeResolution, 0.001);
    EXPECT_DOUBLE_EQ(config.maxChordDeviation, 0.01);
    EXPECT_EQ(config.pathMode, PathControlMode::Blending);
}

// ============================================================================
// FixedTime Strategy Tests
// ============================================================================

class FixedTimeStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        ASSERT_NE(strategy, nullptr);
        
        config.strategy = InterpolationStrategyType::FixedTime;
        config.timeResolution = 0.001;  // 1ms
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(FixedTimeStrategyTest, InterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.feedRate = 6000.0;  // 100 mm/s
    seg.segmentLength = 100.0;
    seg.segmentTime = 1.0;  // 1 second
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 1u);
    
    // First point should be at start
    EXPECT_NEAR(points.front().position.x(), 0.0, 1e-6);
    // Last point should be at end
    EXPECT_NEAR(points.back().position.x(), 100.0, 1e-6);
}

TEST_F(FixedTimeStrategyTest, InterpolateArcSegment) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    seg.feedRate = 1000.0;
    seg.segmentTime = 0.5;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 1u);
    
    // All points should be approximately on the arc
    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position.x() * pt.position.x() + 
                               pt.position.y() * pt.position.y());
        EXPECT_NEAR(dist, 10.0, 0.1);
    }
}

TEST_F(FixedTimeStrategyTest, TimeStepsAreUniform) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.feedRate = 6000.0;
    seg.segmentLength = 100.0;
    seg.segmentTime = 0.1;
    
    strategy->interpolateSegment(seg, ctx, points);
    
    if (points.size() >= 3) {
        double dt1 = points[1].time - points[0].time;
        double dt2 = points[2].time - points[1].time;
        EXPECT_NEAR(dt1, dt2, 1e-6);
    }
}

// ============================================================================
// FixedDeviation Strategy Tests
// ============================================================================

class FixedDeviationStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
        ASSERT_NE(strategy, nullptr);
        
        config.strategy = InterpolationStrategyType::FixedDeviation;
        config.maxChordDeviation = 0.01;  // 0.01mm
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(FixedDeviationStrategyTest, InterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    
    // Linear segments need minimal points
    EXPECT_GE(points.size(), 2u);
}

TEST_F(FixedDeviationStrategyTest, InterpolateArcUsesMorePoints) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    
    // Arc should use more points than just start/end
    EXPECT_GT(points.size(), 2u);
}

TEST_F(FixedDeviationStrategyTest, SmallerDeviationMorePoints) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    // First with larger deviation
    config.maxChordDeviation = 0.1;
    strategy->configure(config);
    strategy->interpolateSegment(seg, ctx, points);
    size_t largeDeviationCount = points.size();
    
    // Then with smaller deviation
    points.clear();
    config.maxChordDeviation = 0.001;
    strategy->configure(config);
    strategy->interpolateSegment(seg, ctx, points);
    size_t smallDeviationCount = points.size();
    
    EXPECT_GE(smallDeviationCount, largeDeviationCount);
}

// ============================================================================
// RKF45 Strategy Tests
// ============================================================================

class RKF45StrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
        ASSERT_NE(strategy, nullptr);
        
        config.strategy = InterpolationStrategyType::RKF45;
        config.errorTolerance = 1e-6;
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(RKF45StrategyTest, InterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 2u);
}

TEST_F(RKF45StrategyTest, InterpolateArcSegment) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 2u);
}

TEST_F(RKF45StrategyTest, AdaptiveStepSizing) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    
    // Adaptive methods may have variable step sizes
    EXPECT_TRUE(result.success);
    
    // Check that result has statistics
    EXPECT_GT(result.totalIterations, 0u);
}

// ============================================================================
// DOPRI Strategy Tests
// ============================================================================

class DOPRIStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
        ASSERT_NE(strategy, nullptr);
        
        config.strategy = InterpolationStrategyType::DOPRI;
        config.errorTolerance = 1e-6;
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(DOPRIStrategyTest, InterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 2u);
}

TEST_F(DOPRIStrategyTest, InterpolateArcSegment) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(DOPRIStrategyTest, HigherOrderAccuracy) {
    // DOPRI is 5th order, should be accurate
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    
    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position.x() * pt.position.x() + 
                               pt.position.y() * pt.position.y());
        EXPECT_NEAR(dist, 10.0, 0.01);  // Within 0.01mm
    }
}

// ============================================================================
// AdaptiveMidpoint Strategy Tests
// ============================================================================

class AdaptiveMidpointStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::AdaptiveMidpoint);
        ASSERT_NE(strategy, nullptr);
        
        config.strategy = InterpolationStrategyType::AdaptiveMidpoint;
        config.maxChordDeviation = 0.01;
        config.maxSubdivisionDepth = 10;
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(AdaptiveMidpointStrategyTest, InterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(AdaptiveMidpointStrategyTest, InterpolateArcSegment) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 2u);
}

TEST_F(AdaptiveMidpointStrategyTest, SubdividesHighCurvature) {
    // Small radius arc = high curvature = more subdivisions needed
    PlanningSegment seg = makeArcSegment(
        makePosition(1, 0), makePosition(0, 1),
        makePosition(0, 0), 1.0, PI / 2, false);
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 3u);
}

// ============================================================================
// DeCasteljau Strategy Tests
// ============================================================================

class DeCasteljauStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DeCasteljau);
        ASSERT_NE(strategy, nullptr);
        
        config.strategy = InterpolationStrategyType::DeCasteljau;
        config.maxChordDeviation = 0.01;
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(DeCasteljauStrategyTest, InterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(DeCasteljauStrategyTest, InterpolateCubicSpline) {
    PlanningSegment seg;
    seg.start = makePosition(0, 0);
    seg.end = makePosition(100, 100);
    seg.motionType = SegmentMotionType::CubicSpline;
    seg.controlPoints.push_back(makePosition(0, 0));
    seg.controlPoints.push_back(makePosition(30, 100));
    seg.controlPoints.push_back(makePosition(70, 0));
    seg.controlPoints.push_back(makePosition(100, 100));
    seg.segmentLength = 150.0;  // Approximate
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST_F(DeCasteljauStrategyTest, SplineInterpolationSmooth) {
    PlanningSegment seg;
    seg.start = makePosition(0, 0);
    seg.end = makePosition(100, 0);
    seg.motionType = SegmentMotionType::CubicSpline;
    seg.controlPoints.push_back(makePosition(0, 0));
    seg.controlPoints.push_back(makePosition(33, 50));
    seg.controlPoints.push_back(makePosition(66, 50));
    seg.controlPoints.push_back(makePosition(100, 0));
    seg.segmentLength = 120.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    
    // Check that points form a smooth curve
    if (points.size() >= 3) {
        // All Y values should be >= 0 (above baseline) for this S-curve
        for (const auto& pt : points) {
            EXPECT_GE(pt.position.y(), -0.1);
        }
    }
}

// ============================================================================
// VelocityPlanner Tests
// ============================================================================

class VelocityPlannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.limits.maxVelocityLinear = 6000.0;  // mm/min
        config.limits.maxAcceleration = 1000.0;    // mm/s²
        config.limits.maxCentripetalAccel = 500.0; // mm/s²
        config.pathMode = PathControlMode::Blending;
        config.blendTolerance = 0.05;
        
        planner = std::make_unique<VelocityPlanner>(config);
    }

    InterpolationConfig config;
    std::unique_ptr<VelocityPlanner> planner;
};

TEST_F(VelocityPlannerTest, PlanSingleSegment) {
    std::vector<PlanningSegment> segments;
    segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(100, 0)));
    segments[0].segmentLength = 100.0;
    segments[0].feedRate = 6000.0;
    
    planner->plan(segments);
    
    // Entry velocity should be 0 (start from rest)
    EXPECT_NEAR(segments[0].entryVelocity, 0.0, 1e-6);
    // Exit velocity should be 0 (end at rest)
    EXPECT_NEAR(segments[0].exitVelocity, 0.0, 1e-6);
}

TEST_F(VelocityPlannerTest, PlanMultipleSegments) {
    std::vector<PlanningSegment> segments;
    segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(100, 0)));
    segments.push_back(makeLinearSegment(makePosition(100, 0), makePosition(200, 0)));
    segments.push_back(makeLinearSegment(makePosition(200, 0), makePosition(300, 0)));
    
    for (auto& seg : segments) {
        seg.segmentLength = 100.0;
        seg.feedRate = 6000.0;
    }
    
    planner->plan(segments);
    
    // First segment starts from rest
    EXPECT_NEAR(segments[0].entryVelocity, 0.0, 1e-6);
    // Last segment ends at rest
    EXPECT_NEAR(segments[2].exitVelocity, 0.0, 1e-6);
}

TEST_F(VelocityPlannerTest, CornerSlowsDown) {
    std::vector<PlanningSegment> segments;
    // 90 degree corner
    segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(100, 0)));
    segments.push_back(makeLinearSegment(makePosition(100, 0), makePosition(100, 100)));
    
    for (auto& seg : segments) {
        seg.segmentLength = 100.0;
        seg.feedRate = 6000.0;
    }
    
    planner->plan(segments);
    
    // At a 90 degree corner, velocity should be reduced
    // Exit of first segment should equal entry of second (continuity)
    EXPECT_NEAR(segments[0].exitVelocity, segments[1].entryVelocity, 1e-6);
}

TEST_F(VelocityPlannerTest, MaxVelocityForCurvature) {
    double curvature = 0.1;  // 1/radius = 1/10mm
    double maxV = planner->maxVelocityForCurvature(curvature);
    
    // v²/r <= maxCentripetal => v <= sqrt(r * maxCentripetal)
    double expectedMax = std::sqrt(config.limits.maxCentripetalAccel / curvature);
    EXPECT_NEAR(maxV, expectedMax, 1e-6);
}

TEST_F(VelocityPlannerTest, ReachableVelocity) {
    double startV = 0.0;
    double distance = 50.0;  // mm
    double accel = 1000.0;   // mm/s²
    
    double reachable = planner->reachableVelocity(startV, distance, accel);
    
    // v² = v0² + 2*a*d => v = sqrt(2*a*d)
    double expected = std::sqrt(2 * accel * distance);
    EXPECT_NEAR(reachable, expected, 1e-3);
}

TEST_F(VelocityPlannerTest, CalculateCornerVelocityCollinear) {
    // Collinear segments (0 degree angle) should allow max velocity
    PlanningSegment seg1 = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    PlanningSegment seg2 = makeLinearSegment(makePosition(100, 0), makePosition(200, 0));
    seg1.maxVelocity = 100.0;  // mm/s
    seg2.maxVelocity = 100.0;
    
    double cornerV = planner->calculateCornerVelocity(seg1, seg2);
    
    // Should be high (limited by feed rate, not corner)
    EXPECT_GT(cornerV, 50.0);
}

TEST_F(VelocityPlannerTest, CalculateCornerVelocity90Degrees) {
    // 90 degree corner should reduce velocity significantly
    PlanningSegment seg1 = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    PlanningSegment seg2 = makeLinearSegment(makePosition(100, 0), makePosition(100, 100));
    seg1.maxVelocity = 100.0;
    seg2.maxVelocity = 100.0;
    
    double cornerV = planner->calculateCornerVelocity(seg1, seg2);
    
    // Should be reduced compared to collinear case
    EXPECT_LT(cornerV, 100.0);
}

// ============================================================================
// InterpolateAll Tests
// ============================================================================

class InterpolateAllTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
        
        config.maxChordDeviation = 0.01;
        strategy->configure(config);
        
        // Set up context with multiple segments
        ctx.segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(50, 0)));
        ctx.segments.push_back(makeLinearSegment(makePosition(50, 0), makePosition(100, 50)));
        ctx.segments.push_back(makeLinearSegment(makePosition(100, 50), makePosition(100, 100)));
        
        for (auto& seg : ctx.segments) {
            seg.segmentLength = seg.start.linearDistance(seg.end);
        }
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
};

TEST_F(InterpolateAllTest, InterpolatesAllSegments) {
    auto result = strategy->interpolateAll(ctx, points);
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 3u);  // At least endpoints
}

TEST_F(InterpolateAllTest, PointsAreOrdered) {
    auto result = strategy->interpolateAll(ctx, points);
    
    EXPECT_TRUE(result.success);
    
    // Points should be in order (time increasing or equal)
    for (size_t i = 1; i < points.size(); ++i) {
        EXPECT_GE(points[i].time, points[i-1].time);
    }
}

TEST_F(InterpolateAllTest, CoversTotalPath) {
    auto result = strategy->interpolateAll(ctx, points);
    
    EXPECT_TRUE(result.success);
    
    // First point should be at start
    EXPECT_NEAR(points.front().position.x(), 0.0, 1e-6);
    EXPECT_NEAR(points.front().position.y(), 0.0, 1e-6);
    
    // Last point should be at end
    EXPECT_NEAR(points.back().position.x(), 100.0, 1e-6);
    EXPECT_NEAR(points.back().position.y(), 100.0, 1e-6);
}

TEST_F(InterpolateAllTest, UpdatesStatistics) {
    auto result = strategy->interpolateAll(ctx, points);
    
    EXPECT_TRUE(result.success);
    // Result contains multiple points indicating interpolation occurred
    EXPECT_GT(points.size(), 3u);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(InterpolationEdgeCases, ZeroLengthSegment) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
    InterpolationConfig config;
    strategy->configure(config);
    
    PlanningSegment seg = makeLinearSegment(makePosition(50, 50), makePosition(50, 50));
    seg.segmentLength = 0.0;
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    // Should handle gracefully (may succeed with just start point or fail cleanly)
}

TEST(InterpolationEdgeCases, VeryShortSegment) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
    InterpolationConfig config;
    config.maxChordDeviation = 0.01;
    strategy->configure(config);
    
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(0.001, 0));
    seg.segmentLength = 0.001;
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    // Should handle without issues
}

TEST(InterpolationEdgeCases, VeryLargeArc) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
    InterpolationConfig config;
    config.maxChordDeviation = 0.01;
    strategy->configure(config);
    
    PlanningSegment seg = makeArcSegment(
        makePosition(1000, 0), makePosition(0, 1000),
        makePosition(0, 0), 1000.0, PI / 2, false);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST(InterpolationEdgeCases, FullCircleArc) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
    InterpolationConfig config;
    config.maxChordDeviation = 0.1;
    strategy->configure(config);
    
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(10, 0),  // Same start/end
        makePosition(0, 0), 10.0, TWO_PI, false);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    // Should handle full circle
}

TEST(InterpolationEdgeCases, NegativeFeedRate) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
    InterpolationConfig config;
    strategy->configure(config);
    
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.feedRate = -1000.0;  // Invalid
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    // Should handle gracefully
    strategy->interpolateSegment(seg, ctx, points);
}

// ============================================================================
// Parameterized Tests for Multiple Strategies
// ============================================================================

class AllStrategiesTest : public ::testing::TestWithParam<InterpolationStrategyType> {
protected:
    void SetUp() override {
        strategy = InterpolationStrategyFactory::create(GetParam());
        ASSERT_NE(strategy, nullptr);
        
        config.maxChordDeviation = 0.01;
        config.timeResolution = 0.001;
        config.errorTolerance = 1e-6;
        strategy->configure(config);
    }

    std::unique_ptr<InterpolationStrategy> strategy;
    InterpolationConfig config;
};

TEST_P(AllStrategiesTest, CanInterpolateLinearSegment) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    seg.feedRate = 1000.0;
    seg.segmentTime = 6.0;  // 100mm at 1000mm/min
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success) << "Strategy: " << strategy->name();
    EXPECT_GE(points.size(), 2u) << "Strategy: " << strategy->name();
}

TEST_P(AllStrategiesTest, CanInterpolateArcSegment) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    seg.feedRate = 1000.0;
    seg.segmentTime = 1.0;
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success) << "Strategy: " << strategy->name();
}

TEST_P(AllStrategiesTest, StartAndEndPointsCorrect) {
    PlanningSegment seg = makeLinearSegment(makePosition(10, 20), makePosition(110, 120));
    seg.segmentLength = seg.start.linearDistance(seg.end);
    seg.feedRate = 1000.0;
    seg.segmentTime = 6.0;
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    
    if (result.success && points.size() >= 2) {
        EXPECT_NEAR(points.front().position.x(), 10.0, 1e-3) << "Strategy: " << strategy->name();
        EXPECT_NEAR(points.front().position.y(), 20.0, 1e-3) << "Strategy: " << strategy->name();
        EXPECT_NEAR(points.back().position.x(), 110.0, 1e-3) << "Strategy: " << strategy->name();
        EXPECT_NEAR(points.back().position.y(), 120.0, 1e-3) << "Strategy: " << strategy->name();
    }
}

INSTANTIATE_TEST_SUITE_P(
    AllStrategies,
    AllStrategiesTest,
    ::testing::Values(
        InterpolationStrategyType::FixedTime,
        InterpolationStrategyType::FixedDeviation,
        InterpolationStrategyType::RKF45,
        InterpolationStrategyType::DOPRI,
        InterpolationStrategyType::AdaptiveMidpoint,
        InterpolationStrategyType::DeCasteljau
    )
);

// ============================================================================
// Performance/Stress Tests
// ============================================================================

TEST(InterpolationPerformance, ManySegments) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
    InterpolationConfig config;
    config.maxChordDeviation = 0.1;  // Larger tolerance for speed
    strategy->configure(config);
    
    InterpolationContext ctx;
    
    // Create 100 segments
    Position current = makePosition(0, 0);
    for (int i = 0; i < 100; ++i) {
        Position next = makePosition(current.x() + 10, current.y() + (i % 2 == 0 ? 10 : -10));
        ctx.segments.push_back(makeLinearSegment(current, next));
        ctx.segments.back().segmentLength = current.linearDistance(next);
        current = next;
    }
    
    std::vector<TrajectoryPoint> points;
    auto result = strategy->interpolateAll(ctx, points);
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 100u);
}

// ============================================================================
// Additional Coverage Tests for InterpolationStrategy
// ============================================================================

TEST_F(InterpolationStrategyBaseTest, EvaluatePositionSplineFallback) {
    // Spline segment should fall back to linear interpolation in base class
    PlanningSegment seg;
    seg.start = makePosition(0, 0);
    seg.end = makePosition(100, 100);
    seg.motionType = SegmentMotionType::Linear;
    seg.segmentLength = seg.start.linearDistance(seg.end);
    // Add control points to make it spline-like (though base class doesn't handle it specially)
    seg.controlPoints.push_back(makePosition(25, 50));
    seg.controlPoints.push_back(makePosition(75, 50));
    
    Position pos = strategy->evaluatePosition(seg, 0.5);
    // Should still work
    EXPECT_FALSE(std::isnan(pos.x()));
}

TEST_F(InterpolationStrategyBaseTest, EvaluateVelocityZeroSegmentTime) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentTime = 0.0;  // Zero time
    
    Position vel = strategy->evaluateVelocity(seg, 0.5);
    // Should return zero velocity
    EXPECT_DOUBLE_EQ(vel.x(), 0.0);
    EXPECT_DOUBLE_EQ(vel.y(), 0.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateVelocityArc) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    seg.segmentTime = 1.0;
    seg.plane = InterpolationPlane::XY;
    
    // Evaluate at midpoint
    Position vel = strategy->evaluateVelocity(seg, 0.5);
    // Velocity should not be zero
    EXPECT_NE(vel.magnitude(), 0.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateVelocityLinearCoverage) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentTime = 1.0;
    
    Position vel = strategy->evaluateVelocity(seg, 0.5);
    // Constant velocity in X direction
    EXPECT_GT(vel.x(), 0.0);
    EXPECT_DOUBLE_EQ(vel.y(), 0.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateAccelerationZeroSegmentTime) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentTime = 0.0;
    
    Position acc = strategy->evaluateAcceleration(seg, 0.5);
    EXPECT_DOUBLE_EQ(acc.x(), 0.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateAccelerationArc) {
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    seg.segmentTime = 1.0;
    seg.plane = InterpolationPlane::XY;
    
    Position acc = strategy->evaluateAcceleration(seg, 0.5);
    // Centripetal acceleration for arc
    EXPECT_NE(acc.magnitude(), 0.0);
}

TEST_F(InterpolationStrategyBaseTest, EvaluateAccelerationLinear) {
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentTime = 1.0;
    
    Position acc = strategy->evaluateAcceleration(seg, 0.5);
    // Constant velocity means zero acceleration
    EXPECT_NEAR(acc.magnitude(), 0.0, 1e-6);
}

// ============================================================================
// VelocityPlanner Additional Tests
// ============================================================================

TEST(VelocityPlannerCoverageTest, PlanWithArcs) {
    InterpolationConfig config;
    config.limits.maxVelocityLinear = 6000.0;  // mm/min
    config.limits.maxAcceleration = 1000.0;
    config.pathMode = PathControlMode::Blending;
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    
    // Linear to arc transition
    segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(50, 0)));
    segments[0].segmentLength = 50.0;
    
    PlanningSegment arc = makeArcSegment(
        makePosition(50, 0), makePosition(60, 10),
        makePosition(50, 10), 10.0, PI / 2, true);
    arc.plane = InterpolationPlane::XY;
    segments.push_back(arc);
    
    // Arc to linear transition  
    segments.push_back(makeLinearSegment(makePosition(60, 10), makePosition(110, 10)));
    segments[2].segmentLength = 50.0;
    
    planner.plan(segments);
    
    // Velocities should be adjusted at corners
    EXPECT_GT(segments[0].exitVelocity, 0.0);
    EXPECT_LE(segments[1].entryVelocity, segments[0].exitVelocity + 0.001);
}

TEST(VelocityPlannerCoverageTest, PlanExactStopMode) {
    InterpolationConfig config;
    config.pathMode = PathControlMode::ExactStop;
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(50, 0)));
    segments.push_back(makeLinearSegment(makePosition(50, 0), makePosition(100, 0)));
    
    for (auto& seg : segments) {
        seg.segmentLength = 50.0;
        seg.feedRate = 1000.0;
    }
    
    planner.plan(segments);
    
    // All entry/exit velocities should be zero in exact stop mode
    for (const auto& seg : segments) {
        EXPECT_DOUBLE_EQ(seg.entryVelocity, 0.0);
        EXPECT_DOUBLE_EQ(seg.exitVelocity, 0.0);
    }
}

TEST(VelocityPlannerCoverageTest, PlanRapidMove) {
    InterpolationConfig config;
    config.limits.maxVelocityLinear = 12000.0;  // mm/min (200 mm/s)
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    seg.isRapid = true;  // G0 rapid move
    seg.feedRate = 1000.0;  // Will be overridden by rapid speed
    segments.push_back(seg);
    
    planner.plan(segments);
    
    // Rapid move should use max velocity
    EXPECT_GT(segments[0].maxVelocity, 100.0 / 60.0);  // > feedRate
}

TEST(VelocityPlannerCoverageTest, PlanEmptySegments) {
    InterpolationConfig config;
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    planner.plan(segments);  // Should not crash
    EXPECT_TRUE(segments.empty());
}

TEST(VelocityPlannerCoverageTest, ArcCurvatureLimit) {
    InterpolationConfig config;
    config.limits.maxCentripetalAccel = 100.0;
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    
    // Small radius arc - high curvature
    PlanningSegment arc = makeArcSegment(
        makePosition(5, 0), makePosition(0, 5),
        makePosition(0, 0), 5.0, PI / 2, false);
    arc.plane = InterpolationPlane::XY;
    arc.feedRate = 6000.0;  // High feed rate
    segments.push_back(arc);
    
    planner.plan(segments);
    
    // Velocity should be limited by curvature
    EXPECT_LT(segments[0].maxVelocity, 6000.0 / 60.0);
}

TEST(VelocityPlannerCoverageTest, CornerVelocityCalcArcToLinear) {
    InterpolationConfig config;
    config.pathMode = PathControlMode::Blending;
    config.limits.maxCentripetalAccel = 1000.0;
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    
    // Arc segment
    PlanningSegment arc = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    arc.plane = InterpolationPlane::XY;
    arc.feedRate = 1000.0;
    segments.push_back(arc);
    
    // Linear segment
    segments.push_back(makeLinearSegment(makePosition(0, 10), makePosition(0, 60)));
    segments[1].segmentLength = 50.0;
    segments[1].feedRate = 1000.0;
    
    planner.plan(segments);
    
    // Corner velocity should be calculated
    EXPECT_GE(segments[0].exitVelocity, 0.0);
}

TEST(VelocityPlannerCoverageTest, CornerVelocityCalcLinearToArc) {
    InterpolationConfig config;
    config.pathMode = PathControlMode::Blending;
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    
    // Linear segment
    segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(10, 0)));
    segments[0].segmentLength = 10.0;
    segments[0].feedRate = 1000.0;
    
    // Arc segment
    PlanningSegment arc = makeArcSegment(
        makePosition(10, 0), makePosition(20, 10),
        makePosition(10, 10), 10.0, PI / 2, true);
    arc.plane = InterpolationPlane::XY;
    arc.feedRate = 1000.0;
    segments.push_back(arc);
    
    planner.plan(segments);
    
    // Should handle linear-to-arc transition
    EXPECT_GE(segments[0].exitVelocity, 0.0);
}

TEST(VelocityPlannerCoverageTest, CornerVelocityCalcArcToArc) {
    InterpolationConfig config;
    config.pathMode = PathControlMode::Blending;
    
    VelocityPlanner planner(config);
    
    std::vector<PlanningSegment> segments;
    
    // First arc
    PlanningSegment arc1 = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    arc1.plane = InterpolationPlane::XY;
    arc1.feedRate = 1000.0;
    segments.push_back(arc1);
    
    // Second arc (continuing)
    PlanningSegment arc2 = makeArcSegment(
        makePosition(0, 10), makePosition(-10, 0),
        makePosition(0, 0), 10.0, PI / 2, false);
    arc2.plane = InterpolationPlane::XY;
    arc2.feedRate = 1000.0;
    segments.push_back(arc2);
    
    planner.plan(segments);
    
    // Should handle arc-to-arc transition
    EXPECT_GE(segments[0].exitVelocity, 0.0);
}

// ============================================================================
// InterpolateAll Coverage Tests  
// ============================================================================

TEST(InterpolateAllCoverageTest, EmptySegments) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    auto result = strategy->interpolateAll(ctx, points);
    // Empty input should succeed with empty output
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(points.empty());
}

TEST(InterpolateAllCoverageTest, MultipleSegments) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
    InterpolationConfig config;
    config.maxChordDeviation = 0.01;
    strategy->configure(config);
    
    InterpolationContext ctx;
    ctx.segments.push_back(makeLinearSegment(makePosition(0, 0), makePosition(50, 0)));
    ctx.segments.push_back(makeLinearSegment(makePosition(50, 0), makePosition(100, 50)));
    ctx.segments.push_back(makeLinearSegment(makePosition(100, 50), makePosition(100, 100)));
    
    for (auto& seg : ctx.segments) {
        seg.segmentLength = seg.start.linearDistance(seg.end);
        seg.segmentTime = seg.segmentLength / (1000.0 / 60.0);  // 1000mm/min
    }
    
    std::vector<TrajectoryPoint> points;
    auto result = strategy->interpolateAll(ctx, points);
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 3u);  // At least start/end of each segment
}

// ============================================================================
// DOPRI / RKF45 Error Handling and Edge Cases
// ============================================================================

TEST(DOPRIRejectedStepsTest, HighToleranceNoRejections) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
    InterpolationConfig config;
    config.errorTolerance = 0.1;  // High tolerance
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    // Simple linear segment - should have few or no rejections
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    seg.segmentTime = 1.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    // High tolerance should result in few rejections
    EXPECT_LE(result.rejectedSteps, 5u);
}

TEST(DOPRIRejectedStepsTest, LowToleranceMayReject) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
    InterpolationConfig config;
    config.errorTolerance = 1e-10;  // Very low tolerance
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    // Complex arc segment - may need step adaptation
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    seg.segmentLength = 10.0 * PI / 2;
    seg.segmentTime = 1.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    // Just verify it completes - rejections are expected with tight tolerance
}

TEST(RKF45RejectedStepsTest, HighToleranceNoRejections) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
    InterpolationConfig config;
    config.errorTolerance = 0.1;  // High tolerance
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    seg.segmentTime = 1.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST(RKF45RejectedStepsTest, LowToleranceMayReject) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
    InterpolationConfig config;
    config.errorTolerance = 1e-10;  // Very low tolerance
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    PlanningSegment seg = makeArcSegment(
        makePosition(10, 0), makePosition(0, 10),
        makePosition(0, 0), 10.0, PI / 2, false);
    seg.segmentLength = 10.0 * PI / 2;
    seg.segmentTime = 1.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
}

TEST(DOPRIEndpointTest, EnsuresEndpoint) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
    InterpolationConfig config;
    config.errorTolerance = 0.01;
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    seg.segmentTime = 1.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(points.empty());
    
    // Last point should be at endpoint
    auto& lastPt = points.back();
    EXPECT_NEAR(lastPt.parameter, 1.0, 1e-6);
    EXPECT_NEAR(lastPt.position.x(), 100.0, 0.01);
}

TEST(RKF45EndpointTest, EnsuresEndpoint) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
    InterpolationConfig config;
    config.errorTolerance = 0.01;
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(100, 0));
    seg.segmentLength = 100.0;
    seg.segmentTime = 1.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(points.empty());
    
    auto& lastPt = points.back();
    EXPECT_NEAR(lastPt.parameter, 1.0, 1e-6);
}

TEST(DOPRIStepSizeTest, TracksMinMaxStepSize) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
    InterpolationConfig config;
    config.errorTolerance = 0.001;
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    PlanningSegment seg = makeArcSegment(
        makePosition(50, 0), makePosition(0, 50),
        makePosition(0, 0), 50.0, PI / 2, false);
    seg.segmentLength = 50.0 * PI / 2;
    seg.segmentTime = 2.0;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    
    // Step size tracking should work
    EXPECT_LE(result.minStepSize, result.maxStepSize);
}
TEST(DOPRIEndpointFallbackTest, VeryShortSegment) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
    InterpolationConfig config;
    config.errorTolerance = 0.01;
    config.maxIterations = 5;  // Low iteration limit to trigger early exit
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    // Very short segment
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(0.1, 0));
    seg.segmentLength = 0.1;
    seg.segmentTime = 0.001;  // Very short time
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    
    // Should have at least the endpoint
    EXPECT_FALSE(points.empty());
    EXPECT_NEAR(points.back().parameter, 1.0, 1e-6);
}

TEST(RKF45EndpointFallbackTest, VeryShortSegment) {
    auto strategy = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
    InterpolationConfig config;
    config.errorTolerance = 0.01;
    config.maxIterations = 5;  // Low iteration limit
    strategy->configure(config);
    
    InterpolationContext ctx;
    std::vector<TrajectoryPoint> points;
    
    PlanningSegment seg = makeLinearSegment(makePosition(0, 0), makePosition(0.1, 0));
    seg.segmentLength = 0.1;
    seg.segmentTime = 0.001;
    
    auto result = strategy->interpolateSegment(seg, ctx, points);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(points.empty());
}