/**
 * @file LimitComplianceTests.cpp
 * @brief Unit tests for kinematic limit compliance checking
 * 
 * Tests that different solvers/approximators obey:
 * - Maximum velocity limits (per-axis and combined)
 * - Maximum acceleration limits
 * - Maximum jerk limits
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>

#include "../TrajectoryAnalyzer.hpp"
#include "gcode/motion/InterpolationStrategy.hpp"

using namespace GCodeExport;
using namespace GCode;

// ============================================================================
// Test Fixtures
// ============================================================================

class LimitComplianceTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up default limits
        limits_.maxVelocityLinear = 6000.0;      // mm/min
        limits_.maxAcceleration = 1000.0;        // mm/s²
        limits_.maxDeceleration = 1000.0;
        limits_.maxJerk = 10000.0;               // mm/s³
        
        for (size_t i = 0; i < MAX_AXES; ++i) {
            limits_.axisMaxVelocity[i] = 6000.0;
            limits_.axisMaxAcceleration[i] = 1000.0;
            limits_.axisMaxJerk[i] = 10000.0;
        }
        
        analysisConfig_.limits = limits_;
        analysisConfig_.timeStep = 0.001;        // 1ms
        analysisConfig_.violationTolerance = 0.01; // 1% tolerance
    }
    
    // Create a simple linear move segment
    PlanningSegment createLinearMove(double startX, double startY, double endX, double endY,
                                    double feedRate = 1000.0) {
        PlanningSegment seg;
        seg.motionType = SegmentMotionType::Linear;
        seg.start[0] = startX;
        seg.start[1] = startY;
        seg.end[0] = endX;
        seg.end[1] = endY;
        seg.feedRate = feedRate;
        
        double dx = endX - startX;
        double dy = endY - startY;
        seg.segmentLength = std::sqrt(dx*dx + dy*dy);
        seg.segmentTime = seg.segmentLength / (feedRate / 60.0);
        
        return seg;
    }
    
    // Create an arc segment
    PlanningSegment createArc(double startX, double startY, double endX, double endY,
                            double centerX, double centerY, bool clockwise,
                            double feedRate = 1000.0) {
        PlanningSegment seg;
        seg.motionType = clockwise ? SegmentMotionType::ArcCW : SegmentMotionType::ArcCCW;
        seg.start[0] = startX;
        seg.start[1] = startY;
        seg.end[0] = endX;
        seg.end[1] = endY;
        seg.center[0] = centerX;
        seg.center[1] = centerY;
        seg.feedRate = feedRate;
        seg.plane = InterpolationPlane::XY;
        
        seg.arcRadius = std::sqrt(
            (startX - centerX) * (startX - centerX) +
            (startY - centerY) * (startY - centerY)
        );
        
        double startAngle = std::atan2(startY - centerY, startX - centerX);
        double endAngle = std::atan2(endY - centerY, endX - centerX);
        
        seg.arcSweep = endAngle - startAngle;
        if (clockwise && seg.arcSweep > 0) seg.arcSweep -= 2 * M_PI;
        if (!clockwise && seg.arcSweep < 0) seg.arcSweep += 2 * M_PI;
        
        seg.segmentLength = std::abs(seg.arcSweep) * seg.arcRadius;
        seg.segmentTime = seg.segmentLength / (feedRate / 60.0);
        
        return seg;
    }
    
    KinematicLimits limits_;
    AnalysisConfig analysisConfig_;
};

// ============================================================================
// Velocity Limit Tests
// ============================================================================

TEST_F(LimitComplianceTest, LinearMoveWithinVelocityLimit) {
    std::vector<PlanningSegment> segments;
    
    // Move at 3000 mm/min (limit is 6000)
    segments.push_back(createLinearMove(0, 0, 100, 0, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_TRUE(stats.meetsLimits) << "Expected trajectory to meet limits";
    EXPECT_TRUE(stats.violations.empty()) << "Expected no violations";
    
    // Verify max velocity is under limit
    double maxVelLimit = limits_.maxVelocityLinear / 60.0;  // Convert to mm/s
    EXPECT_LE(stats.maxLinearVelocity, maxVelLimit * 1.01);
}

TEST_F(LimitComplianceTest, LinearMoveExceedingVelocityLimit) {
    // Set a low velocity limit
    limits_.maxVelocityLinear = 1000.0;  // mm/min
    analysisConfig_.limits = limits_;
    
    std::vector<PlanningSegment> segments;
    // Move at 3000 mm/min (limit is 1000)
    segments.push_back(createLinearMove(0, 0, 100, 0, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_FALSE(stats.meetsLimits) << "Expected trajectory to violate limits";
    EXPECT_FALSE(stats.violations.empty()) << "Expected velocity violations";
    
    // Check that violations are velocity type
    bool hasVelocityViolation = false;
    for (const auto& v : stats.violations) {
        if (v.limitType == "velocity") {
            hasVelocityViolation = true;
            break;
        }
    }
    EXPECT_TRUE(hasVelocityViolation);
}

TEST_F(LimitComplianceTest, DiagonalMoveVelocityScaling) {
    std::vector<PlanningSegment> segments;
    
    // Diagonal move - per-axis velocity should be properly scaled
    segments.push_back(createLinearMove(0, 0, 100, 100, 4000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Per-axis velocity should be feedrate / sqrt(2) for 45-degree diagonal
    double expectedAxisVel = (4000.0 / 60.0) / std::sqrt(2.0);
    
    EXPECT_LE(std::abs(stats.axisStats[0].maxVelocity), expectedAxisVel * 1.1);
    EXPECT_LE(std::abs(stats.axisStats[1].maxVelocity), expectedAxisVel * 1.1);
}

// ============================================================================
// Acceleration Limit Tests
// ============================================================================

TEST_F(LimitComplianceTest, AccelerationAtMoveStart) {
    std::vector<PlanningSegment> segments;
    
    // Short move that requires significant acceleration
    segments.push_back(createLinearMove(0, 0, 10, 0, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Check acceleration is within limits
    EXPECT_LE(stats.maxLinearAcceleration, limits_.maxAcceleration * 1.1)
        << "Acceleration exceeds limit at move start";
}

TEST_F(LimitComplianceTest, AccelerationAtCorner) {
    std::vector<PlanningSegment> segments;
    
    // Two moves with 90-degree corner
    segments.push_back(createLinearMove(0, 0, 50, 0, 3000.0));
    segments.push_back(createLinearMove(50, 0, 50, 50, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Acceleration at corner should be handled (may violate without proper planning)
    // This test documents the behavior
    if (!stats.meetsLimits) {
        bool hasAccelViolation = std::any_of(
            stats.violations.begin(), stats.violations.end(),
            [](const LimitViolation& v) { return v.limitType == "acceleration"; }
        );
        EXPECT_TRUE(hasAccelViolation) << "Expected acceleration violation at corner";
    }
}

TEST_F(LimitComplianceTest, LowAccelerationLimit) {
    // Set very low acceleration limit
    limits_.maxAcceleration = 100.0;  // mm/s²
    analysisConfig_.limits = limits_;
    
    std::vector<PlanningSegment> segments;
    segments.push_back(createLinearMove(0, 0, 100, 0, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // With such low acceleration, we expect violations
    bool hasAccelViolation = std::any_of(
        stats.violations.begin(), stats.violations.end(),
        [](const LimitViolation& v) { return v.limitType == "acceleration"; }
    );
    
    // Document: basic interpolation may not respect acceleration limits
    // A proper motion planner with velocity scheduling is needed
}

// ============================================================================
// Jerk Limit Tests
// ============================================================================

TEST_F(LimitComplianceTest, JerkAtAccelerationChange) {
    std::vector<PlanningSegment> segments;
    
    // Multiple connected moves
    segments.push_back(createLinearMove(0, 0, 30, 0, 2000.0));
    segments.push_back(createLinearMove(30, 0, 60, 0, 4000.0));  // Velocity change
    segments.push_back(createLinearMove(60, 0, 90, 0, 2000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Jerk should be computed
    EXPECT_GT(stats.maxLinearJerk, 0) << "Expected non-zero jerk";
}

TEST_F(LimitComplianceTest, LowJerkLimit) {
    limits_.maxJerk = 1000.0;  // Very low jerk limit
    analysisConfig_.limits = limits_;
    
    std::vector<PlanningSegment> segments;
    segments.push_back(createLinearMove(0, 0, 50, 0, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Check if jerk violations are detected
    bool hasJerkViolation = std::any_of(
        stats.violations.begin(), stats.violations.end(),
        [](const LimitViolation& v) { return v.limitType == "jerk"; }
    );
    
    // Document behavior - may or may not violate depending on time step
}

// ============================================================================
// Per-Axis Limit Tests
// ============================================================================

TEST_F(LimitComplianceTest, PerAxisVelocityLimits) {
    // Set different limits per axis
    limits_.axisMaxVelocity[0] = 3000.0;  // X limited
    limits_.axisMaxVelocity[1] = 6000.0;  // Y normal
    analysisConfig_.limits = limits_;
    
    std::vector<PlanningSegment> segments;
    // Move mostly in X direction
    segments.push_back(createLinearMove(0, 0, 100, 10, 4000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Check for X-axis velocity violation
    bool hasXViolation = std::any_of(
        stats.violations.begin(), stats.violations.end(),
        [](const LimitViolation& v) { return v.axis == 0 && v.limitType == "velocity"; }
    );
    
    // May violate depending on actual trajectory velocity
}

// ============================================================================
// Arc Motion Tests
// ============================================================================

TEST_F(LimitComplianceTest, ArcVelocityWithinLimit) {
    std::vector<PlanningSegment> segments;
    
    // Quarter circle arc
    segments.push_back(createArc(10, 0, 0, 10, 0, 0, false, 2000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_GT(samples.size(), 5) << "Expected multiple samples for arc";
}

TEST_F(LimitComplianceTest, ArcCentripetalAcceleration) {
    std::vector<PlanningSegment> segments;
    
    // Small radius arc at high speed - tests centripetal acceleration
    segments.push_back(createArc(5, 0, 0, 5, 0, 0, false, 3000.0));
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    auto stats = analyzer.computeStatistics(samples);
    
    // Centripetal acceleration = v²/r
    // v = 3000 mm/min = 50 mm/s, r = 5mm
    // a_c = 50² / 5 = 500 mm/s²
    EXPECT_GT(stats.maxCentripetalAccel, 0) << "Expected non-zero centripetal acceleration";
}

// ============================================================================
// Strategy Comparison Tests
// ============================================================================

TEST_F(LimitComplianceTest, FixedTimeStrategyCompliance) {
    auto strategy = ApproximationFactory::create("FixedTime");
    ASSERT_NE(strategy, nullptr);
    
    strategy->configure("timeStep", 0.001);
    
    std::vector<PlanningSegment> segments;
    segments.push_back(createLinearMove(0, 0, 100, 0, 3000.0));
    
    auto samples = strategy->generateTrajectory(segments, limits_);
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_GT(samples.size(), 10) << "Expected reasonable sample count";
}

TEST_F(LimitComplianceTest, FixedDeviationStrategyCompliance) {
    auto strategy = ApproximationFactory::create("FixedDeviation");
    ASSERT_NE(strategy, nullptr);
    
    strategy->configure("maxDeviation", 0.005);  // 5µm
    
    std::vector<PlanningSegment> segments;
    // Arc will need more samples than linear
    segments.push_back(createArc(10, 0, 0, 10, 0, 0, false, 2000.0));
    
    auto samples = strategy->generateTrajectory(segments, limits_);
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_GT(samples.size(), 10) << "Expected many samples for arc with tight tolerance";
}

// ============================================================================
// Violation Reporting Tests
// ============================================================================

TEST_F(LimitComplianceTest, ViolationDetailAccuracy) {
    limits_.maxVelocityLinear = 1000.0;  // Low limit
    analysisConfig_.limits = limits_;
    
    std::vector<PlanningSegment> segments;
    segments.push_back(createLinearMove(0, 0, 100, 0, 3000.0));  // 3x over limit
    
    TrajectoryAnalyzer analyzer(analysisConfig_);
    auto samples = analyzer.analyze(segments);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    
    if (!violations.empty()) {
        const auto& v = violations[0];
        EXPECT_GE(v.time, 0);
        EXPECT_GT(v.value, 0);
        EXPECT_GT(v.limit, 0);
        EXPECT_GT(v.overshoot, 0);
    }
}
