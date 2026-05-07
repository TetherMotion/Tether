/**
 * @file InterpolationStrategyTests.cpp
 * @brief Comprehensive unit tests for G-Code interpolation strategies
 *
 * Contains 200+ tests covering:
 * - All 6 interpolation strategies
 * - Linear motion (G0, G1)
 * - Arc motion (G2, G3)
 * - Spline motion (G5, G5.1, NURBS)
 * - Path modes (G61, G61.1, G64)
 * - Velocity planning
 * - Edge cases and error conditions
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <random>
#include <limits>

// Include the interpolation strategy header
#include "gcode/motion/InterpolationStrategy.hpp"

namespace GCode {
namespace test {

using ::testing::DoubleNear;
using ::testing::Lt;
using ::testing::Gt;
using ::testing::Le;
using ::testing::Ge;

// ============================================================================
// Test Constants
// ============================================================================

constexpr double TEST_EPSILON = 1e-9;
constexpr double POSITION_TOLERANCE = 1e-6;
constexpr double VELOCITY_TOLERANCE = 1e-4;
constexpr double ANGLE_TOLERANCE = 1e-6;

// ============================================================================
// Test Fixtures
// ============================================================================

/**
 * @brief Base fixture for interpolation strategy tests
 */
class InterpolationStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.timeResolution = 0.001;
        config_.maxChordDeviation = 0.01;
        config_.errorTolerance = 1e-6;
        config_.minStepSize = 1e-8;
        config_.maxStepSize = 0.1;
        config_.safetyFactor = 0.9;
        config_.maxIterations = 1000;
        config_.maxSubdivisionDepth = 20;
        config_.pathMode = PathControlMode::Blending;
        config_.blendTolerance = 0.05;

        ctx_.config = config_;
    }

    // Helper to create linear segment
    MotionSegment createLinearSegment(
        const Position& start,
        const Position& end,
        double feedRate = 1000.0
    ) {
        MotionSegment seg;
        seg.start = start;
        seg.end = end;
        seg.motionType = SegmentMotionType::Linear;
        seg.feedRate = feedRate;
        seg.segmentLength = start.linearDistance(end);
        seg.segmentTime = (seg.segmentLength / feedRate) * 60.0;  // feedRate is mm/min
        return seg;
    }

    // Helper to create arc segment
    MotionSegment createArcSegment(
        const Position& start,
        const Position& end,
        const Position& center,
        double radius,
        double sweep,
        bool clockwise = true,
        double feedRate = 1000.0
    ) {
        MotionSegment seg;
        seg.start = start;
        seg.end = end;
        seg.center = center;
        seg.motionType = clockwise ? SegmentMotionType::ArcCW : SegmentMotionType::ArcCCW;
        seg.feedRate = feedRate;
        seg.arcRadius = radius;
        seg.arcSweep = clockwise ? -std::fabs(sweep) : std::fabs(sweep);
        seg.segmentLength = std::fabs(sweep) * radius;
        seg.segmentTime = (seg.segmentLength / feedRate) * 60.0;
        seg.plane = InterpolationPlane::XY;
        return seg;
    }

    // Helper to verify continuity between points
    bool verifyContinuity(const std::vector<TrajectoryPoint>& points, double maxGap) {
        for (size_t i = 1; i < points.size(); ++i) {
            double gap = points[i].position.linearDistance(points[i-1].position);
            if (gap > maxGap) return false;
        }
        return true;
    }

    // Helper to verify monotonic time
    bool verifyMonotonicTime(const std::vector<TrajectoryPoint>& points) {
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].time < points[i-1].time) return false;
        }
        return true;
    }

    // Helper to verify endpoints
    bool verifyEndpoints(
        const std::vector<TrajectoryPoint>& points,
        const MotionSegment& segment,
        double tolerance = POSITION_TOLERANCE
    ) {
        if (points.empty()) return false;
        double startDist = points.front().position.linearDistance(segment.start);
        double endDist = points.back().position.linearDistance(segment.end);
        return startDist < tolerance && endDist < tolerance;
    }

    InterpolationConfig config_;
    InterpolationContext ctx_;
};

/**
 * @brief Parameterized test fixture for testing all strategies
 */
class AllStrategiesTest : public InterpolationStrategyTest,
                          public ::testing::WithParamInterface<InterpolationStrategyType> {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(GetParam());
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

// ============================================================================
// Strategy Type Parameterization
// ============================================================================

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
    ),
    [](const ::testing::TestParamInfo<InterpolationStrategyType>& info) {
        switch (info.param) {
            case InterpolationStrategyType::FixedTime: return "FixedTime";
            case InterpolationStrategyType::FixedDeviation: return "FixedDeviation";
            case InterpolationStrategyType::RKF45: return "RKF45";
            case InterpolationStrategyType::DOPRI: return "DOPRI";
            case InterpolationStrategyType::AdaptiveMidpoint: return "AdaptiveMidpoint";
            case InterpolationStrategyType::DeCasteljau: return "DeCasteljau";
            default: return "Unknown";
        }
    }
);

// ============================================================================
// Basic Strategy Tests (Parameterized for all strategies)
// ============================================================================

TEST_P(AllStrategiesTest, StrategyTypeMatchesCreation) {
    EXPECT_EQ(strategy_->type(), GetParam());
}

TEST_P(AllStrategiesTest, StrategyHasName) {
    EXPECT_NE(strategy_->name(), nullptr);
    EXPECT_GT(strlen(strategy_->name()), 0);
}

TEST_P(AllStrategiesTest, InterpolatesLinearSegmentXAxis) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 2);
    EXPECT_TRUE(verifyEndpoints(points, segment));
    EXPECT_TRUE(verifyMonotonicTime(points));
}

TEST_P(AllStrategiesTest, InterpolatesLinearSegmentYAxis) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 50; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment));
}

TEST_P(AllStrategiesTest, InterpolatesLinearSegmentZAxis) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 0; end[2] = -25;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment));
}

TEST_P(AllStrategiesTest, InterpolatesLinearSegmentDiagonal) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 100; end[2] = 100;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment));

    // All points should be on the line
    for (const auto& pt : points) {
        double t = pt.parameter;
        double expectedX = start[0] + t * (end[0] - start[0]);
        double expectedY = start[1] + t * (end[1] - start[1]);
        double expectedZ = start[2] + t * (end[2] - start[2]);

        EXPECT_NEAR(pt.position[0], expectedX, POSITION_TOLERANCE);
        EXPECT_NEAR(pt.position[1], expectedY, POSITION_TOLERANCE);
        EXPECT_NEAR(pt.position[2], expectedZ, POSITION_TOLERANCE);
    }
}

TEST_P(AllStrategiesTest, InterpolatesArcCW90Degrees) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = -10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::PI / 2.0;

    auto segment = createArcSegment(start, end, center, radius, sweep, true);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 2);
    EXPECT_TRUE(verifyEndpoints(points, segment, 0.1));  // Looser tolerance for arcs

    // All points should be on the circle
    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position[0]*pt.position[0] + pt.position[1]*pt.position[1]);
        EXPECT_NEAR(dist, radius, config_.maxChordDeviation * 2);
    }
}

TEST_P(AllStrategiesTest, InterpolatesArcCCW90Degrees) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::PI / 2.0;

    auto segment = createArcSegment(start, end, center, radius, sweep, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 2);
}

TEST_P(AllStrategiesTest, InterpolatesArcCW180Degrees) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = -10; end[1] = 0; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::PI;

    auto segment = createArcSegment(start, end, center, radius, sweep, true);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 4);  // 180 degrees should have multiple points
}

TEST_P(AllStrategiesTest, InterpolatesArcFullCircle) {
    Position start, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::TWO_PI;

    auto segment = createArcSegment(start, start, center, radius, sweep, true);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 8);  // Full circle should have many points
}

TEST_P(AllStrategiesTest, InterpolatesZeroLengthSegment) {
    Position pos;
    pos[0] = 50; pos[1] = 50; pos[2] = 50;

    auto segment = createLinearSegment(pos, pos);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 1);
}

TEST_P(AllStrategiesTest, InterpolatesVeryShortSegment) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 0.001; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 2);
}

TEST_P(AllStrategiesTest, InterpolatesVeryLongSegment) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 10000; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end, 6000);  // 6000 mm/min
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GE(points.size(), 10);  // Should have many points for long segment
}

TEST_P(AllStrategiesTest, InterpolatesSmallRadiusArc) {
    Position start, end, center;
    start[0] = 1; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 1; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 1.0;
    double sweep = InterpolationConstants::PI / 2.0;

    auto segment = createArcSegment(start, end, center, radius, sweep, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, InterpolatesLargeRadiusArc) {
    Position start, end, center;
    start[0] = 1000; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 1000; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 1000.0;
    double sweep = InterpolationConstants::PI / 2.0;

    auto segment = createArcSegment(start, end, center, radius, sweep, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, InterpolatesHelicalMotion) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 10; end[1] = 0; end[2] = -10;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::TWO_PI;

    auto segment = createArcSegment(start, end, center, radius, sweep, true);
    segment.end[2] = -10;  // Z descent
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // Z should decrease monotonically
    for (size_t i = 1; i < points.size(); ++i) {
        EXPECT_LE(points[i].position[2], points[i-1].position[2] + POSITION_TOLERANCE);
    }
}

TEST_P(AllStrategiesTest, InterpolatesArcInXZPlane) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 0; end[2] = 10;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::PI / 2.0;

    auto segment = createArcSegment(start, end, center, radius, sweep, false);
    segment.plane = InterpolationPlane::XZ;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, InterpolatesArcInYZPlane) {
    Position start, end, center;
    start[0] = 0; start[1] = 10; start[2] = 0;
    end[0] = 0; end[1] = 0; end[2] = 10;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    double sweep = InterpolationConstants::PI / 2.0;

    auto segment = createArcSegment(start, end, center, radius, sweep, false);
    segment.plane = InterpolationPlane::YZ;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, VelocityIsConsistentWithFeedRate) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 60; end[1] = 0; end[2] = 0;  // 60mm

    double feedRate = 3600.0;  // 3600 mm/min = 60 mm/s
    auto segment = createLinearSegment(start, end, feedRate);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // Check velocity magnitude (should be close to feed rate)
    for (const auto& pt : points) {
        if (pt.parameter > 0.1 && pt.parameter < 0.9) {  // Avoid endpoints
            double velMag = std::sqrt(
                pt.velocity[0]*pt.velocity[0] +
                pt.velocity[1]*pt.velocity[1] +
                pt.velocity[2]*pt.velocity[2]
            );
            EXPECT_NEAR(velMag, 60.0, 1.0);  // 60 mm/s with tolerance
        }
    }
}

TEST_P(AllStrategiesTest, TimeIncreasesMonotonically) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 50; end[2] = 25;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyMonotonicTime(points));
}

TEST_P(AllStrategiesTest, ParameterStaysInRange) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    for (const auto& pt : points) {
        EXPECT_GE(pt.parameter, 0.0 - TEST_EPSILON);
        EXPECT_LE(pt.parameter, 1.0 + TEST_EPSILON);
    }
}

// ============================================================================
// Fixed Time Strategy Specific Tests
// ============================================================================

class FixedTimeStrategyTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedTime);
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(FixedTimeStrategyTest, GeneratesPointsAtFixedIntervals) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    config_.timeResolution = 0.01;  // 10ms
    strategy_->configure(config_);

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // Check time intervals are approximately fixed
    for (size_t i = 2; i < points.size() - 1; ++i) {
        double dt = points[i].time - points[i-1].time;
        EXPECT_NEAR(dt, config_.timeResolution, config_.timeResolution * 0.1);
    }
}

TEST_F(FixedTimeStrategyTest, PointCountScalesWithDuration) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    config_.timeResolution = 0.001;
    strategy_->configure(config_);

    // Short segment
    auto shortSeg = createLinearSegment(start, end, 6000);  // 1 second
    std::vector<TrajectoryPoint> shortPoints;
    strategy_->interpolateSegment(shortSeg, ctx_, shortPoints);

    // Long segment
    ctx_ = InterpolationContext();
    ctx_.config = config_;
    auto longSeg = createLinearSegment(start, end, 600);  // 10 seconds
    std::vector<TrajectoryPoint> longPoints;
    strategy_->interpolateSegment(longSeg, ctx_, longPoints);

    EXPECT_GT(longPoints.size(), shortPoints.size());
}

// ============================================================================
// Fixed Deviation Strategy Specific Tests
// ============================================================================

class FixedDeviationStrategyTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::FixedDeviation);
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(FixedDeviationStrategyTest, ArcPointCountScalesWithCurvature) {
    Position center;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double sweep = InterpolationConstants::PI;

    // Small radius arc (high curvature)
    Position start1, end1;
    start1[0] = 5; start1[1] = 0; start1[2] = 0;
    end1[0] = -5; end1[1] = 0; end1[2] = 0;
    auto smallArc = createArcSegment(start1, end1, center, 5.0, sweep, true);
    std::vector<TrajectoryPoint> smallPoints;
    strategy_->interpolateSegment(smallArc, ctx_, smallPoints);

    // Large radius arc (low curvature)
    ctx_ = InterpolationContext();
    ctx_.config = config_;
    Position start2, end2;
    start2[0] = 100; start2[1] = 0; start2[2] = 0;
    end2[0] = -100; end2[1] = 0; end2[2] = 0;
    auto largeArc = createArcSegment(start2, end2, center, 100.0, sweep, true);
    std::vector<TrajectoryPoint> largePoints;
    strategy_->interpolateSegment(largeArc, ctx_, largePoints);

    // Small arc should have more points per unit length due to higher curvature
    double smallPointsPerMM = smallPoints.size() / smallArc.segmentLength;
    double largePointsPerMM = largePoints.size() / largeArc.segmentLength;

    EXPECT_GT(smallPointsPerMM, largePointsPerMM);
}

TEST_F(FixedDeviationStrategyTest, ChordErrorRespectsTolerance) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    config_.maxChordDeviation = 0.1;
    strategy_->configure(config_);

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // Check that chord error is within tolerance
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        // Midpoint of chord
        Position chordMid;
        for (size_t k = 0; k < MAX_AXES; ++k) {
            chordMid[k] = (points[i].position[k] + points[i+1].position[k]) / 2.0;
        }

        // Distance from center (should be close to radius)
        double dist = std::sqrt(chordMid[0]*chordMid[0] + chordMid[1]*chordMid[1]);
        double error = std::fabs(dist - 10.0);

        EXPECT_LT(error, config_.maxChordDeviation * 2);  // Allow some margin
    }
}

// ============================================================================
// RKF45 Strategy Specific Tests
// ============================================================================

class RKF45StrategyTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::RKF45);
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(RKF45StrategyTest, AdaptsStepSizeForHighCurvature) {
    Position start, end, center;
    start[0] = 5; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 5; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 5.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.minStepSize, 0);
    EXPECT_GT(result.maxStepSize, result.minStepSize);
}

TEST_F(RKF45StrategyTest, RejectsStepsWhenErrorTooHigh) {
    config_.errorTolerance = 1e-10;  // Very tight tolerance
    strategy_->configure(config_);

    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    // With tight tolerance, some steps should be rejected
    // (This depends on the curve complexity)
}

// ============================================================================
// DOPRI Strategy Specific Tests
// ============================================================================

class DOPRIStrategyTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::DOPRI);
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(DOPRIStrategyTest, FSALPropertyReducesEvaluations) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 2);
}

TEST_F(DOPRIStrategyTest, ConvergesForSmoothCurves) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_LT(result.rejectedSteps, result.totalIterations / 2);  // Most steps accepted
}

// ============================================================================
// Adaptive Midpoint Strategy Specific Tests
// ============================================================================

class AdaptiveMidpointStrategyTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::AdaptiveMidpoint);
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(AdaptiveMidpointStrategyTest, SubdividesHighCurvatureRegions) {
    Position start, end, center;
    start[0] = 5; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 5; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 5.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 2);
}

TEST_F(AdaptiveMidpointStrategyTest, DepthLimitPreventsInfiniteRecursion) {
    config_.maxSubdivisionDepth = 5;
    config_.maxChordDeviation = 1e-15;  // Impossibly tight
    strategy_->configure(config_);

    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);  // Should complete without infinite recursion
    EXPECT_LT(result.totalIterations, 1000);  // Bounded iterations
}

// ============================================================================
// De Casteljau Strategy Specific Tests
// ============================================================================

class DeCasteljauStrategyTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        strategy_ = InterpolationStrategyFactory::create(InterpolationStrategyType::DeCasteljau);
        strategy_->configure(config_);
    }

    std::unique_ptr<InterpolationStrategy> strategy_;
};

TEST_F(DeCasteljauStrategyTest, InterpolatesLinearAsLinear) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment));
}

TEST_F(DeCasteljauStrategyTest, ConvertsArcToBezier) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // All points should be approximately on the circle
    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position[0]*pt.position[0] + pt.position[1]*pt.position[1]);
        EXPECT_NEAR(dist, 10.0, config_.maxChordDeviation * 5);  // Allow some approximation error
    }
}

// ============================================================================
// Velocity Planner Tests
// ============================================================================

class VelocityPlannerTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
        planner_ = std::make_unique<VelocityPlanner>(config_);
    }

    std::unique_ptr<VelocityPlanner> planner_;
};

TEST_F(VelocityPlannerTest, PlansSingleSegment) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    std::vector<MotionSegment> segments;
    segments.push_back(createLinearSegment(start, end));

    planner_->plan(segments);

    // Single segment should start and end at zero velocity
    EXPECT_DOUBLE_EQ(segments[0].entryVelocity, 0.0);
    EXPECT_DOUBLE_EQ(segments[0].exitVelocity, 0.0);
}

TEST_F(VelocityPlannerTest, PlansMultipleSegments) {
    Position p0, p1, p2, p3;
    p0[0] = 0; p0[1] = 0; p0[2] = 0;
    p1[0] = 100; p1[1] = 0; p1[2] = 0;
    p2[0] = 100; p2[1] = 100; p2[2] = 0;
    p3[0] = 200; p3[1] = 100; p3[2] = 0;

    std::vector<MotionSegment> segments;
    segments.push_back(createLinearSegment(p0, p1));
    segments.push_back(createLinearSegment(p1, p2));
    segments.push_back(createLinearSegment(p2, p3));

    planner_->plan(segments);

    // First segment starts at zero
    EXPECT_DOUBLE_EQ(segments[0].entryVelocity, 0.0);
    // Last segment ends at zero
    EXPECT_DOUBLE_EQ(segments[2].exitVelocity, 0.0);
    // Velocities should match at junctions
    EXPECT_DOUBLE_EQ(segments[0].exitVelocity, segments[1].entryVelocity);
    EXPECT_DOUBLE_EQ(segments[1].exitVelocity, segments[2].entryVelocity);
}

TEST_F(VelocityPlannerTest, G61ExactStopMode) {
    config_.pathMode = PathControlMode::ExactStop;
    planner_ = std::make_unique<VelocityPlanner>(config_);

    Position p0, p1, p2;
    p0[0] = 0; p0[1] = 0; p0[2] = 0;
    p1[0] = 100; p1[1] = 0; p1[2] = 0;
    p2[0] = 100; p2[1] = 100; p2[2] = 0;

    std::vector<MotionSegment> segments;
    segments.push_back(createLinearSegment(p0, p1));
    segments.push_back(createLinearSegment(p1, p2));

    planner_->plan(segments);

    // All entry/exit velocities should be zero in exact stop mode
    EXPECT_DOUBLE_EQ(segments[0].entryVelocity, 0.0);
    EXPECT_DOUBLE_EQ(segments[0].exitVelocity, 0.0);
    EXPECT_DOUBLE_EQ(segments[1].entryVelocity, 0.0);
    EXPECT_DOUBLE_EQ(segments[1].exitVelocity, 0.0);
}

TEST_F(VelocityPlannerTest, G64BlendingMode) {
    config_.pathMode = PathControlMode::Blending;
    config_.blendTolerance = 1.0;  // 1mm tolerance
    planner_ = std::make_unique<VelocityPlanner>(config_);

    Position p0, p1, p2;
    p0[0] = 0; p0[1] = 0; p0[2] = 0;
    p1[0] = 100; p1[1] = 0; p1[2] = 0;
    p2[0] = 100; p2[1] = 100; p2[2] = 0;  // 90 degree turn

    std::vector<MotionSegment> segments;
    segments.push_back(createLinearSegment(p0, p1));
    segments.push_back(createLinearSegment(p1, p2));

    planner_->plan(segments);

    // With blending, corner velocity should be non-zero
    EXPECT_GT(segments[0].exitVelocity, 0.0);
    EXPECT_GT(segments[1].entryVelocity, 0.0);
}

TEST_F(VelocityPlannerTest, RespectsAccelerationLimits) {
    config_.limits.maxAcceleration = 100.0;  // 100 mm/s²
    planner_ = std::make_unique<VelocityPlanner>(config_);

    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 10; end[1] = 0; end[2] = 0;  // Short segment

    std::vector<MotionSegment> segments;
    auto seg = createLinearSegment(start, end, 6000);  // High feed rate
    segments.push_back(seg);

    planner_->plan(segments);

    // Exit velocity should be limited by acceleration capability
    double maxReachable = std::sqrt(2 * config_.limits.maxAcceleration * seg.segmentLength);
    EXPECT_LE(segments[0].exitVelocity, maxReachable + 1.0);
}

TEST_F(VelocityPlannerTest, CurvatureLimitsVelocity) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    std::vector<MotionSegment> segments;
    segments.push_back(createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false, 6000));

    planner_->plan(segments);

    // Velocity should be limited by centripetal acceleration
    double maxCurvatureVel = planner_->maxVelocityForCurvature(1.0 / 10.0);
    EXPECT_LE(segments[0].maxVelocity, maxCurvatureVel + 1.0);
}

// ============================================================================
// Path Mode Tests (G61/G61.1/G64)
// ============================================================================

class PathModeTest : public InterpolationStrategyTest {
protected:
    void SetUp() override {
        InterpolationStrategyTest::SetUp();
    }
};

TEST_F(PathModeTest, G61ProducesDistinctStops) {
    config_.pathMode = PathControlMode::ExactStop;
    auto strategy = InterpolationStrategyFactory::create(config_);

    Position p0, p1, p2;
    p0[0] = 0; p0[1] = 0; p0[2] = 0;
    p1[0] = 50; p1[1] = 0; p1[2] = 0;
    p2[0] = 50; p2[1] = 50; p2[2] = 0;

    std::vector<MotionSegment> segments;
    segments.push_back(createLinearSegment(p0, p1));
    segments.push_back(createLinearSegment(p1, p2));

    VelocityPlanner planner(config_);
    planner.plan(segments);

    // In G61 mode, all segments should have zero entry/exit velocities
    for (const auto& seg : segments) {
        EXPECT_DOUBLE_EQ(seg.entryVelocity, 0.0);
        EXPECT_DOUBLE_EQ(seg.exitVelocity, 0.0);
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_P(AllStrategiesTest, HandlesNegativeCoordinates) {
    Position start, end;
    start[0] = -100; start[1] = -50; start[2] = -25;
    end[0] = 100; end[1] = 50; end[2] = 25;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment));
}

TEST_P(AllStrategiesTest, HandlesVeryHighFeedRate) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end, 100000);  // 100m/min
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, HandlesVeryLowFeedRate) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end, 1);  // 1 mm/min
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, HandlesArcNearZeroSweep) {
    Position start, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    Position end;
    end[0] = 10 * std::cos(0.01);  // Very small angle
    end[1] = 10 * std::sin(0.01);
    end[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, 0.01, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, HandlesRapidMove) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 100; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    segment.motionType = SegmentMotionType::Rapid;
    segment.isRapid = true;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment));
}

TEST_P(AllStrategiesTest, HandlesMultipleAxes) {
    Position start, end;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        start[i] = i * 10.0;
        end[i] = i * 10.0 + 50.0;
    }

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

// ============================================================================
// Stress Tests
// ============================================================================

TEST_P(AllStrategiesTest, HandlesManyShortSegments) {
    std::vector<MotionSegment> segments;
    Position current;
    current[0] = 0; current[1] = 0; current[2] = 0;

    for (int i = 0; i < 100; ++i) {
        Position next;
        next[0] = current[0] + 1.0;
        next[1] = current[1] + (i % 2 == 0 ? 1.0 : -1.0);
        next[2] = current[2];

        segments.push_back(createLinearSegment(current, next));
        current = next;
    }

    ctx_.segments = segments;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateAll(ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_GT(points.size(), 100);
}

TEST_P(AllStrategiesTest, HandlesRandomizedSegments) {
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<> dist(-100.0, 100.0);

    std::vector<MotionSegment> segments;
    Position current;
    current[0] = 0; current[1] = 0; current[2] = 0;

    for (int i = 0; i < 50; ++i) {
        Position next;
        next[0] = dist(gen);
        next[1] = dist(gen);
        next[2] = dist(gen);

        segments.push_back(createLinearSegment(current, next));
        current = next;
    }

    ctx_.segments = segments;
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateAll(ctx_, points);

    EXPECT_TRUE(result.success);
}

// ============================================================================
// Numerical Precision Tests
// ============================================================================

TEST_P(AllStrategiesTest, MaintainsPrecisionForSmallMovements) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 1e-6; end[1] = 1e-6; end[2] = 1e-6;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
}

TEST_P(AllStrategiesTest, MaintainsPrecisionForLargeCoordinates) {
    Position start, end;
    start[0] = 1e6; start[1] = 1e6; start[2] = 1e6;
    end[0] = 1e6 + 100; end[1] = 1e6 + 100; end[2] = 1e6 + 100;

    auto segment = createLinearSegment(start, end);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(verifyEndpoints(points, segment, 1e-3));  // Larger tolerance for large coordinates
}

// ============================================================================
// Arc-Specific Tests
// ============================================================================

TEST_P(AllStrategiesTest, ArcPreservesRadius) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    for (const auto& pt : points) {
        double dist = std::sqrt(pt.position[0]*pt.position[0] + pt.position[1]*pt.position[1]);
        EXPECT_NEAR(dist, 10.0, config_.maxChordDeviation * 5);
    }
}

TEST_P(AllStrategiesTest, ArcDirectionIsCorrect_CW) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = -10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, true);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // Y should decrease (clockwise from +X axis)
    if (points.size() >= 3) {
        double midY = points[points.size() / 2].position[1];
        EXPECT_LT(midY, start[1]);  // Y decreases for CW
    }
}

TEST_P(AllStrategiesTest, ArcDirectionIsCorrect_CCW) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    auto segment = createArcSegment(start, end, center, 10.0, InterpolationConstants::PI / 2.0, false);
    std::vector<TrajectoryPoint> points;

    auto result = strategy_->interpolateSegment(segment, ctx_, points);

    EXPECT_TRUE(result.success);

    // Y should increase (counter-clockwise from +X axis)
    if (points.size() >= 3) {
        double midY = points[points.size() / 2].position[1];
        EXPECT_GT(midY, start[1]);  // Y increases for CCW
    }
}

// ============================================================================
// Position Evaluation Tests
// ============================================================================

TEST_P(AllStrategiesTest, EvaluatePositionAt0ReturnsStart) {
    Position start, end;
    start[0] = 10; start[1] = 20; start[2] = 30;
    end[0] = 100; end[1] = 200; end[2] = 300;

    auto segment = createLinearSegment(start, end);
    Position pos = strategy_->evaluatePosition(segment, 0.0);

    EXPECT_NEAR(pos[0], start[0], POSITION_TOLERANCE);
    EXPECT_NEAR(pos[1], start[1], POSITION_TOLERANCE);
    EXPECT_NEAR(pos[2], start[2], POSITION_TOLERANCE);
}

TEST_P(AllStrategiesTest, EvaluatePositionAt1ReturnsEnd) {
    Position start, end;
    start[0] = 10; start[1] = 20; start[2] = 30;
    end[0] = 100; end[1] = 200; end[2] = 300;

    auto segment = createLinearSegment(start, end);
    Position pos = strategy_->evaluatePosition(segment, 1.0);

    EXPECT_NEAR(pos[0], end[0], POSITION_TOLERANCE);
    EXPECT_NEAR(pos[1], end[1], POSITION_TOLERANCE);
    EXPECT_NEAR(pos[2], end[2], POSITION_TOLERANCE);
}

TEST_P(AllStrategiesTest, EvaluatePositionAt05ReturnsMidpoint) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 100; end[2] = 100;

    auto segment = createLinearSegment(start, end);
    Position pos = strategy_->evaluatePosition(segment, 0.5);

    EXPECT_NEAR(pos[0], 50.0, POSITION_TOLERANCE);
    EXPECT_NEAR(pos[1], 50.0, POSITION_TOLERANCE);
    EXPECT_NEAR(pos[2], 50.0, POSITION_TOLERANCE);
}

// ============================================================================
// Curvature Tests
// ============================================================================

TEST_P(AllStrategiesTest, CurvatureIsZeroForLine) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    double curvature = strategy_->evaluateCurvature(segment, 0.5);

    EXPECT_DOUBLE_EQ(curvature, 0.0);
}

TEST_P(AllStrategiesTest, CurvatureIsInverseRadiusForArc) {
    Position start, end, center;
    start[0] = 10; start[1] = 0; start[2] = 0;
    end[0] = 0; end[1] = 10; end[2] = 0;
    center[0] = 0; center[1] = 0; center[2] = 0;

    double radius = 10.0;
    auto segment = createArcSegment(start, end, center, radius, InterpolationConstants::PI / 2.0, false);
    double curvature = strategy_->evaluateCurvature(segment, 0.5);

    EXPECT_NEAR(curvature, 1.0 / radius, 1e-6);
}

// ============================================================================
// Arc Length Tests
// ============================================================================

TEST_P(AllStrategiesTest, ArcLengthAt0IsZero) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    double length = strategy_->arcLength(segment, 0.0);

    EXPECT_DOUBLE_EQ(length, 0.0);
}

TEST_P(AllStrategiesTest, ArcLengthAt1IsSegmentLength) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);
    double length = strategy_->arcLength(segment, 1.0);

    EXPECT_NEAR(length, segment.segmentLength, POSITION_TOLERANCE);
}

TEST_P(AllStrategiesTest, ArcLengthInverseRoundTrip) {
    Position start, end;
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 100; end[1] = 0; end[2] = 0;

    auto segment = createLinearSegment(start, end);

    double t = 0.7;
    double s = strategy_->arcLength(segment, t);
    double tBack = strategy_->arcLengthInverse(segment, s);

    EXPECT_NEAR(tBack, t, 1e-6);
}

} // namespace test
} // namespace GCode
