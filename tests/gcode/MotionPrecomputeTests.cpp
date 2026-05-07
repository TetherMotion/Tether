/**
 * @file MotionPrecomputeTests.cpp
 * @brief Comprehensive tests for MotionPrecompute module
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "tether/gcode/motion/MotionPrecompute.hpp"

using namespace GCode;
using namespace GCode::Motion;
using namespace GCode::Math;

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

Vec3 makeVec3(double x, double y, double z = 0.0) {
    return Vec3(x, y, z);
}

MotionSegment makeLinearSegment(const Vec3& start, const Vec3& end, double feedRate = 1000.0) {
    MotionSegment seg;
    seg.start = start;
    seg.end = end;
    seg.feedRate = feedRate;
    seg.isArc = false;
    seg.isRapid = false;
    return seg;
}

MotionSegment makeArcSegment(const Vec3& start, const Vec3& end, const Vec3& center,
                              double radius, double sweep, int plane = 0, double feedRate = 1000.0) {
    MotionSegment seg;
    seg.start = start;
    seg.end = end;
    seg.center = center;
    seg.radius = radius;
    seg.sweep = sweep;
    seg.plane = plane;
    seg.feedRate = feedRate;
    seg.isArc = true;
    seg.isRapid = false;
    return seg;
}

} // namespace

// ============================================================================
// PrecomputeStats Tests
// ============================================================================

TEST(PrecomputeStatsTest, DefaultConstruction) {
    PrecomputeStats stats;
    EXPECT_EQ(stats.totalTime.count(), 0);
    EXPECT_EQ(stats.parseTime.count(), 0);
    EXPECT_EQ(stats.interpolationTime.count(), 0);
    EXPECT_EQ(stats.postProcessTime.count(), 0);
    EXPECT_EQ(stats.inputSegments, 0u);
    EXPECT_EQ(stats.outputPoints, 0u);
    EXPECT_EQ(stats.linearSegments, 0u);
    EXPECT_EQ(stats.arcSegments, 0u);
}

TEST(PrecomputeStatsTest, PointsPerSecondZeroTime) {
    PrecomputeStats stats;
    stats.totalTime = std::chrono::microseconds(0);
    stats.outputPoints = 100;
    EXPECT_DOUBLE_EQ(stats.pointsPerSecond(), 0.0);
}

TEST(PrecomputeStatsTest, PointsPerSecondCalculation) {
    PrecomputeStats stats;
    stats.totalTime = std::chrono::microseconds(1000);  // 1ms
    stats.outputPoints = 100;
    EXPECT_DOUBLE_EQ(stats.pointsPerSecond(), 100000.0);  // 100 points in 1ms = 100k/s
}

TEST(PrecomputeStatsTest, SummaryNotEmpty) {
    PrecomputeStats stats;
    stats.totalTime = std::chrono::microseconds(1000);
    stats.outputPoints = 100;
    stats.inputSegments = 5;
    
    std::string summary = stats.summary();
    EXPECT_FALSE(summary.empty());
    EXPECT_NE(summary.find("Statistics"), std::string::npos);
}

// ============================================================================
// MotionSegment Tests
// ============================================================================

TEST(MotionSegmentPrecomputeTest, DefaultConstruction) {
    MotionSegment seg;
    EXPECT_DOUBLE_EQ(seg.radius, 0.0);
    EXPECT_DOUBLE_EQ(seg.sweep, 0.0);
    EXPECT_DOUBLE_EQ(seg.feedRate, 1000.0);
    EXPECT_EQ(seg.plane, 0);
    EXPECT_EQ(seg.blockIndex, -1);
    EXPECT_FALSE(seg.isRapid);
    EXPECT_FALSE(seg.isArc);
}

TEST(MotionSegmentPrecomputeTest, LinearSegmentCreation) {
    MotionSegment seg = makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 2000.0);
    EXPECT_DOUBLE_EQ(seg.start.x, 0.0);
    EXPECT_DOUBLE_EQ(seg.end.x, 100.0);
    EXPECT_DOUBLE_EQ(seg.feedRate, 2000.0);
    EXPECT_FALSE(seg.isArc);
}

TEST(MotionSegmentPrecomputeTest, ArcSegmentCreation) {
    double sweep = M_PI / 2;
    MotionSegment seg = makeArcSegment(
        makeVec3(10, 0), makeVec3(0, 10),
        makeVec3(0, 0), 10.0, sweep, 0, 1500.0);
    
    EXPECT_DOUBLE_EQ(seg.radius, 10.0);
    EXPECT_DOUBLE_EQ(seg.sweep, sweep);
    EXPECT_TRUE(seg.isArc);
}

// ============================================================================
// FixedTimeStrategy Tests
// ============================================================================

class MotionFixedTimeStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = std::make_unique<FixedTimeStrategy>(0.01);  // 10ms time step
    }

    std::unique_ptr<FixedTimeStrategy> strategy;
    std::vector<TrajectoryPoint> points;
};

TEST_F(MotionFixedTimeStrategyTest, Name) {
    EXPECT_EQ(strategy->name(), "FixedTime");
}

TEST_F(MotionFixedTimeStrategyTest, TimeStepGetSet) {
    strategy->setTimeStep(0.02);
    EXPECT_DOUBLE_EQ(strategy->timeStep(), 0.02);
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateLinearBasic) {
    Vec3 start = makeVec3(0, 0);
    Vec3 end = makeVec3(100, 0);
    double feedRate = 6000.0;  // 100mm/s
    
    strategy->interpolateLinear(start, end, feedRate, points);
    
    EXPECT_GT(points.size(), 1u);
    EXPECT_NEAR(points.front().position.x, 0.0, 1e-6);
    EXPECT_NEAR(points.back().position.x, 100.0, 1e-6);
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateLinearZeroDistance) {
    Vec3 start = makeVec3(50, 50);
    Vec3 end = makeVec3(50, 50);  // Same point
    
    strategy->interpolateLinear(start, end, 1000.0, points);
    
    // Should still produce at least one point
    EXPECT_GE(points.size(), 1u);
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateLinearVertical) {
    Vec3 start = makeVec3(0, 0);
    Vec3 end = makeVec3(0, 100);
    
    strategy->interpolateLinear(start, end, 6000.0, points);
    
    EXPECT_GT(points.size(), 1u);
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.position.x, 0.0, 1e-6);
    }
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateLinearDiagonal) {
    Vec3 start = makeVec3(0, 0);
    Vec3 end = makeVec3(100, 100);
    
    strategy->interpolateLinear(start, end, 6000.0, points);
    
    EXPECT_GT(points.size(), 1u);
    // All points should be on the diagonal
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.position.x, pt.position.y, 1e-6);
    }
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateArcBasic) {
    Vec3 start = makeVec3(10, 0);
    Vec3 end = makeVec3(0, 10);
    Vec3 center = makeVec3(0, 0);
    double radius = 10.0;
    double sweep = M_PI / 2;  // 90 degrees
    
    strategy->interpolateArc(start, end, center, radius, sweep, 0, 6000.0, points);
    
    EXPECT_GT(points.size(), 1u);
    
    // All points should be approximately on the arc
    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position.x * pt.position.x + pt.position.y * pt.position.y);
        EXPECT_NEAR(dist, radius, 0.1);
    }
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateArcXZPlane) {
    Vec3 start = makeVec3(10, 0, 0);
    Vec3 end = makeVec3(0, 0, 10);
    Vec3 center = makeVec3(0, 0, 0);
    double radius = 10.0;
    double sweep = M_PI / 2;
    
    strategy->interpolateArc(start, end, center, radius, sweep, 1, 6000.0, points);  // XZ plane
    
    EXPECT_GT(points.size(), 1u);
    
    // Y should remain constant
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.position.y, 0.0, 0.1);
    }
}

TEST_F(MotionFixedTimeStrategyTest, InterpolateArcYZPlane) {
    Vec3 start = makeVec3(0, 10, 0);
    Vec3 end = makeVec3(0, 0, 10);
    Vec3 center = makeVec3(0, 0, 0);
    double radius = 10.0;
    double sweep = M_PI / 2;
    
    strategy->interpolateArc(start, end, center, radius, sweep, 2, 6000.0, points);  // YZ plane
    
    EXPECT_GT(points.size(), 1u);
    
    // X should remain constant
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.position.x, 0.0, 0.1);
    }
}

TEST_F(MotionFixedTimeStrategyTest, SmallerTimeStepMorePoints) {
    Vec3 start = makeVec3(0, 0);
    Vec3 end = makeVec3(100, 0);
    double feedRate = 6000.0;
    
    // First with larger time step
    strategy->setTimeStep(0.05);
    strategy->interpolateLinear(start, end, feedRate, points);
    size_t largeStepCount = points.size();
    
    // Then with smaller time step
    points.clear();
    strategy->setTimeStep(0.01);
    strategy->interpolateLinear(start, end, feedRate, points);
    size_t smallStepCount = points.size();
    
    EXPECT_GE(smallStepCount, largeStepCount);
}

// ============================================================================
// FixedDeviationStrategy Tests
// ============================================================================

class MotionFixedDeviationStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = std::make_unique<FixedDeviationStrategy>(0.01);  // 0.01mm deviation
    }

    std::unique_ptr<FixedDeviationStrategy> strategy;
    std::vector<TrajectoryPoint> points;
};

TEST_F(MotionFixedDeviationStrategyTest, Name) {
    EXPECT_EQ(strategy->name(), "FixedDeviation");
}

TEST_F(MotionFixedDeviationStrategyTest, MaxDeviationGetSet) {
    strategy->setMaxDeviation(0.05);
    EXPECT_DOUBLE_EQ(strategy->maxDeviation(), 0.05);
}

TEST_F(MotionFixedDeviationStrategyTest, InterpolateLinearBasic) {
    Vec3 start = makeVec3(0, 0);
    Vec3 end = makeVec3(100, 0);
    
    strategy->interpolateLinear(start, end, 6000.0, points);
    
    EXPECT_GE(points.size(), 2u);  // At least start and end
}

TEST_F(MotionFixedDeviationStrategyTest, InterpolateArcBasic) {
    Vec3 start = makeVec3(10, 0);
    Vec3 end = makeVec3(0, 10);
    Vec3 center = makeVec3(0, 0);
    double radius = 10.0;
    double sweep = M_PI / 2;
    
    strategy->interpolateArc(start, end, center, radius, sweep, 0, 6000.0, points);
    
    EXPECT_GT(points.size(), 2u);  // Arc needs more than just endpoints
}

TEST_F(MotionFixedDeviationStrategyTest, SmallerDeviationMoreArcPoints) {
    Vec3 start = makeVec3(10, 0);
    Vec3 end = makeVec3(0, 10);
    Vec3 center = makeVec3(0, 0);
    double radius = 10.0;
    double sweep = M_PI / 2;
    
    // Larger deviation
    strategy->setMaxDeviation(0.5);
    strategy->interpolateArc(start, end, center, radius, sweep, 0, 6000.0, points);
    size_t largeDeviationCount = points.size();
    
    // Smaller deviation
    points.clear();
    strategy->setMaxDeviation(0.01);
    strategy->interpolateArc(start, end, center, radius, sweep, 0, 6000.0, points);
    size_t smallDeviationCount = points.size();
    
    EXPECT_GE(smallDeviationCount, largeDeviationCount);
}

// ============================================================================
// AdaptiveStrategy Tests
// ============================================================================

class MotionAdaptiveStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        strategy = std::make_unique<AdaptiveStrategy>(0.001, 0.1, 0.01);
    }

    std::unique_ptr<AdaptiveStrategy> strategy;
    std::vector<TrajectoryPoint> points;
};

TEST_F(MotionAdaptiveStrategyTest, Name) {
    EXPECT_EQ(strategy->name(), "Adaptive");
}

TEST_F(MotionAdaptiveStrategyTest, InterpolateLinearBasic) {
    Vec3 start = makeVec3(0, 0);
    Vec3 end = makeVec3(100, 0);
    
    strategy->interpolateLinear(start, end, 6000.0, points);
    
    EXPECT_GE(points.size(), 2u);
}

TEST_F(MotionAdaptiveStrategyTest, InterpolateArcBasic) {
    Vec3 start = makeVec3(10, 0);
    Vec3 end = makeVec3(0, 10);
    Vec3 center = makeVec3(0, 0);
    double radius = 10.0;
    double sweep = M_PI / 2;
    
    strategy->interpolateArc(start, end, center, radius, sweep, 0, 6000.0, points);
    
    EXPECT_GT(points.size(), 2u);
}

// ============================================================================
// MotionPrecompute Strategy Factory Tests
// ============================================================================

TEST(MotionPrecomputeFactoryTest, CreateFixedTimeStrategy) {
    auto strategy = createStrategy("FixedTime", 0.01);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "FixedTime");
}

TEST(MotionPrecomputeFactoryTest, CreateFixedDeviationStrategy) {
    auto strategy = createStrategy("FixedDeviation", 0.05);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "FixedDeviation");
}

TEST(MotionPrecomputeFactoryTest, CreateAdaptiveStrategy) {
    auto strategy = createStrategy("Adaptive", 0.01);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "Adaptive");
}

TEST(MotionPrecomputeFactoryTest, UnknownStrategyReturnsDefault) {
    // Factory returns default (FixedTime) for unknown strategy names
    auto strategy = createStrategy("Unknown", 0.01);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "FixedTime");
}

TEST(MotionPrecomputeFactoryTest, CaseInsensitiveLowercase) {
    // Factory supports lowercase names
    auto strategy = createStrategy("fixedtime", 0.01);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "FixedTime");
}

TEST(MotionPrecomputeFactoryTest, CaseInsensitiveDeviation) {
    auto strategy = createStrategy("fixeddeviation", 0.05);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "FixedDeviation");
}

TEST(MotionPrecomputeFactoryTest, CaseInsensitiveAdaptive) {
    auto strategy = createStrategy("adaptive", 0.01);
    ASSERT_NE(strategy, nullptr);
    EXPECT_EQ(strategy->name(), "Adaptive");
}

// ============================================================================
// MotionPrecomputer Tests
// ============================================================================

class MotionPrecomputerTest : public ::testing::Test {
protected:
    void SetUp() override {
        precomputer = std::make_unique<MotionPrecomputer>();
    }

    std::unique_ptr<MotionPrecomputer> precomputer;
};

TEST_F(MotionPrecomputerTest, DefaultConstruction) {
    // Should have default strategy
    std::string name = precomputer->strategyName();
    EXPECT_FALSE(name.empty());
}

TEST_F(MotionPrecomputerTest, SetStrategy) {
    precomputer->setStrategy(std::make_unique<FixedTimeStrategy>(0.02));
    EXPECT_EQ(precomputer->strategyName(), "FixedTime");
}

TEST_F(MotionPrecomputerTest, SetRapidFeedRate) {
    precomputer->setRapidFeedRate(12000.0);
    // Should not crash
}

TEST_F(MotionPrecomputerTest, SetCalculateVelocity) {
    precomputer->setCalculateVelocity(true);
    precomputer->setCalculateVelocity(false);
    // Should not crash
}

TEST_F(MotionPrecomputerTest, SetCalculateAcceleration) {
    precomputer->setCalculateAcceleration(true);
    precomputer->setCalculateAcceleration(false);
    // Should not crash
}

TEST_F(MotionPrecomputerTest, PrecomputeEmptySegments) {
    std::vector<MotionSegment> segments;
    auto points = precomputer->precompute(segments);
    
    EXPECT_TRUE(points.empty());
    EXPECT_EQ(precomputer->stats().inputSegments, 0u);
}

TEST_F(MotionPrecomputerTest, PrecomputeSingleLinearSegment) {
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 6000.0));
    
    auto points = precomputer->precompute(segments);
    
    EXPECT_GT(points.size(), 1u);
    EXPECT_EQ(precomputer->stats().inputSegments, 1u);
    EXPECT_EQ(precomputer->stats().linearSegments, 1u);
}

TEST_F(MotionPrecomputerTest, PrecomputeSingleArcSegment) {
    std::vector<MotionSegment> segments;
    segments.push_back(makeArcSegment(
        makeVec3(10, 0), makeVec3(0, 10),
        makeVec3(0, 0), 10.0, M_PI / 2, 0, 6000.0));
    
    auto points = precomputer->precompute(segments);
    
    EXPECT_GT(points.size(), 1u);
    EXPECT_EQ(precomputer->stats().inputSegments, 1u);
    EXPECT_EQ(precomputer->stats().arcSegments, 1u);
}

TEST_F(MotionPrecomputerTest, PrecomputeMultipleSegments) {
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(50, 0), 6000.0));
    segments.push_back(makeLinearSegment(makeVec3(50, 0), makeVec3(100, 50), 6000.0));
    segments.push_back(makeLinearSegment(makeVec3(100, 50), makeVec3(100, 100), 6000.0));
    
    auto points = precomputer->precompute(segments);
    
    EXPECT_GT(points.size(), 3u);
    EXPECT_EQ(precomputer->stats().inputSegments, 3u);
    EXPECT_EQ(precomputer->stats().linearSegments, 3u);
}

TEST_F(MotionPrecomputerTest, PrecomputeMixedSegments) {
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(10, 0), 6000.0));
    segments.push_back(makeArcSegment(
        makeVec3(10, 0), makeVec3(20, 10),
        makeVec3(10, 10), 10.0, M_PI / 2, 0, 6000.0));
    segments.push_back(makeLinearSegment(makeVec3(20, 10), makeVec3(20, 20), 6000.0));
    
    auto points = precomputer->precompute(segments);
    
    EXPECT_GT(points.size(), 3u);
    EXPECT_EQ(precomputer->stats().linearSegments, 2u);
    EXPECT_EQ(precomputer->stats().arcSegments, 1u);
}

TEST_F(MotionPrecomputerTest, PrecomputeRapidSegment) {
    precomputer->setRapidFeedRate(12000.0);
    
    std::vector<MotionSegment> segments;
    MotionSegment rapid;
    rapid.start = makeVec3(0, 0);
    rapid.end = makeVec3(100, 100);
    rapid.isRapid = true;
    rapid.feedRate = 0;  // Will use rapid feed rate
    segments.push_back(rapid);
    
    auto points = precomputer->precompute(segments);
    
    EXPECT_GT(points.size(), 1u);
    for (const auto& pt : points) {
        EXPECT_TRUE(pt.isRapid);
    }
}

TEST_F(MotionPrecomputerTest, StatsUpdatedAfterPrecompute) {
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 6000.0));
    
    precomputer->precompute(segments);
    
    const auto& stats = precomputer->stats();
    EXPECT_GT(stats.outputPoints, 0u);
    EXPECT_GE(stats.totalTime.count(), 0);
}

TEST_F(MotionPrecomputerTest, PrecomputeWithDifferentStrategies) {
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 6000.0));
    
    // Test with FixedTime
    precomputer->setStrategy(std::make_unique<FixedTimeStrategy>(0.01));
    auto pts1 = precomputer->precompute(segments);
    
    // Test with FixedDeviation
    precomputer->setStrategy(std::make_unique<FixedDeviationStrategy>(0.01));
    auto pts2 = precomputer->precompute(segments);
    
    // Test with Adaptive
    precomputer->setStrategy(std::make_unique<AdaptiveStrategy>());
    auto pts3 = precomputer->precompute(segments);
    
    // All should produce points
    EXPECT_GT(pts1.size(), 0u);
    EXPECT_GT(pts2.size(), 0u);
    EXPECT_GT(pts3.size(), 0u);
}

// ============================================================================
// MotionPrecompute TrajectoryPoint Tests
// ============================================================================

TEST(MotionPrecomputeTrajectoryPointTest, DefaultConstruction) {
    TrajectoryPoint pt;
    EXPECT_DOUBLE_EQ(pt.feedRate, 0.0);
    EXPECT_DOUBLE_EQ(pt.distanceFromStart, 0.0);
    EXPECT_FALSE(pt.isRapid);
    EXPECT_FALSE(pt.isArc);
}

TEST(MotionPrecomputeTrajectoryPointTest, PositionStorage) {
    TrajectoryPoint pt;
    pt.position = makeVec3(10, 20, 30);
    EXPECT_DOUBLE_EQ(pt.position.x, 10.0);
    EXPECT_DOUBLE_EQ(pt.position.y, 20.0);
    EXPECT_DOUBLE_EQ(pt.position.z, 30.0);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(MotionPrecomputeIntegration, FullToolpath) {
    MotionPrecomputer precomputer;
    precomputer.setStrategy(std::make_unique<FixedTimeStrategy>(0.01));
    
    // Create a square toolpath
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 3000.0));
    segments.push_back(makeLinearSegment(makeVec3(100, 0), makeVec3(100, 100), 3000.0));
    segments.push_back(makeLinearSegment(makeVec3(100, 100), makeVec3(0, 100), 3000.0));
    segments.push_back(makeLinearSegment(makeVec3(0, 100), makeVec3(0, 0), 3000.0));
    
    auto points = precomputer.precompute(segments);
    
    EXPECT_GT(points.size(), 10u);
    
    // First point should be at origin
    EXPECT_NEAR(points.front().position.x, 0.0, 1e-6);
    EXPECT_NEAR(points.front().position.y, 0.0, 1e-6);
    
    // Last point should be back at origin
    EXPECT_NEAR(points.back().position.x, 0.0, 1e-6);
    EXPECT_NEAR(points.back().position.y, 0.0, 1e-6);
}

TEST(MotionPrecomputeIntegration, CircularPath) {
    MotionPrecomputer precomputer;
    precomputer.setStrategy(std::make_unique<FixedDeviationStrategy>(0.01));
    
    // Create a full circle
    std::vector<MotionSegment> segments;
    segments.push_back(makeArcSegment(
        makeVec3(10, 0), makeVec3(10, 0),  // Full circle
        makeVec3(0, 0), 10.0, 2 * M_PI, 0, 3000.0));
    
    auto points = precomputer.precompute(segments);
    
    EXPECT_GT(points.size(), 10u);
    
    // All points should be approximately at radius 10
    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position.x * pt.position.x + pt.position.y * pt.position.y);
        EXPECT_NEAR(dist, 10.0, 0.1);
    }
}

TEST(MotionPrecomputeIntegration, Helical3DPath) {
    MotionPrecomputer precomputer;
    precomputer.setStrategy(std::make_unique<FixedTimeStrategy>(0.01));
    
    // Helical path: arc in XY while moving in Z
    std::vector<MotionSegment> segments;
    MotionSegment helix;
    helix.start = makeVec3(10, 0, 0);
    helix.end = makeVec3(10, 0, 10);  // One turn going up 10mm
    helix.center = makeVec3(0, 0, 0);
    helix.radius = 10.0;
    helix.sweep = 2 * M_PI;
    helix.plane = 0;  // XY
    helix.feedRate = 3000.0;
    helix.isArc = true;
    segments.push_back(helix);
    
    auto points = precomputer.precompute(segments);
    
    EXPECT_GT(points.size(), 10u);
    
    // Z should increase from 0 to 10
    EXPECT_NEAR(points.front().position.z, 0.0, 0.1);
    EXPECT_NEAR(points.back().position.z, 10.0, 0.1);
}

// ============================================================================
// Edge Cases Tests
// ============================================================================

TEST(MotionPrecomputeEdgeCases, VeryShortSegment) {
    MotionPrecomputer precomputer;
    
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(0.001, 0), 6000.0));
    
    auto points = precomputer.precompute(segments);
    
    // Should handle without crashing
    EXPECT_GE(points.size(), 1u);
}

TEST(MotionPrecomputeEdgeCases, VerySmallArc) {
    MotionPrecomputer precomputer;
    
    std::vector<MotionSegment> segments;
    segments.push_back(makeArcSegment(
        makeVec3(0.01, 0), makeVec3(0, 0.01),
        makeVec3(0, 0), 0.01, M_PI / 2, 0, 6000.0));
    
    auto points = precomputer.precompute(segments);
    
    // Should handle without crashing
    EXPECT_GE(points.size(), 1u);
}

TEST(MotionPrecomputeEdgeCases, VeryHighFeedRate) {
    MotionPrecomputer precomputer;
    
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 100000.0));  // Very fast
    
    auto points = precomputer.precompute(segments);
    
    // Should handle without crashing
    EXPECT_GE(points.size(), 1u);
}

TEST(MotionPrecomputeEdgeCases, ZeroFeedRate) {
    MotionPrecomputer precomputer;
    
    std::vector<MotionSegment> segments;
    segments.push_back(makeLinearSegment(makeVec3(0, 0), makeVec3(100, 0), 0.0));  // Zero feed
    
    auto points = precomputer.precompute(segments);
    
    // Should handle gracefully (may use default or produce minimal points)
    // Just check it doesn't crash
}

TEST(MotionPrecomputeEdgeCases, NegativeArcRadius) {
    MotionPrecomputer precomputer;
    
    std::vector<MotionSegment> segments;
    segments.push_back(makeArcSegment(
        makeVec3(10, 0), makeVec3(0, 10),
        makeVec3(0, 0), -10.0, M_PI / 2, 0, 6000.0));  // Negative radius
    
    // Should handle gracefully
    auto points = precomputer.precompute(segments);
}

TEST(MotionPrecomputeEdgeCases, VeryLongPath) {
    MotionPrecomputer precomputer;
    precomputer.setStrategy(std::make_unique<FixedTimeStrategy>(0.1));  // Larger step to avoid huge point count
    
    std::vector<MotionSegment> segments;
    // Create 100 segments
    for (int i = 0; i < 100; ++i) {
        segments.push_back(makeLinearSegment(
            makeVec3(i * 10.0, 0), makeVec3((i + 1) * 10.0, 0), 6000.0));
    }
    
    auto points = precomputer.precompute(segments);
    
    EXPECT_GT(points.size(), 100u);
    EXPECT_EQ(precomputer.stats().inputSegments, 100u);
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST(MotionPrecomputePerformance, TimingReported) {
    MotionPrecomputer precomputer;
    
    std::vector<MotionSegment> segments;
    for (int i = 0; i < 10; ++i) {
        segments.push_back(makeLinearSegment(
            makeVec3(i * 10.0, 0), makeVec3((i + 1) * 10.0, 0), 6000.0));
    }
    
    precomputer.precompute(segments);
    
    const auto& stats = precomputer.stats();
    
    // Timing should be recorded
    EXPECT_GE(stats.totalTime.count(), 0);
}

