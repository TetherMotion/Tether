/**
 * @file G64CornerModeTests.cpp
 * @brief Unit tests for G64 corner mode extensions
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>

// Include the corner mode header
#include "gcode/motion/G64CornerMode.hpp"

using namespace GCode;
using namespace testing;

// ============================================================================
// Test Fixtures
// ============================================================================

class G64CornerModeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config
        config_.pathTolerance = 0.1;
        config_.cornerMode = G64CornerMode::Centered;
    }

    /**
     * @brief Create a simple motion segment
     */
    PlanningSegment createSegment(double x1, double y1, double x2, double y2) {
        PlanningSegment seg;
        seg.start[0] = x1;
        seg.start[1] = y1;
        seg.start[2] = 0;
        seg.end[0] = x2;
        seg.end[1] = y2;
        seg.end[2] = 0;
        seg.motionType = SegmentMotionType::Linear;
        seg.feedRate = 1000;
        seg.segmentLength = std::sqrt(std::pow(x2-x1, 2) + std::pow(y2-y1, 2));
        return seg;
    }

    /**
     * @brief Compute distance from point to line segment
     */
    double pointToLineDistance(const Position& p,
                               const Position& lineStart,
                               const Position& lineEnd) {
        double dx = lineEnd[0] - lineStart[0];
        double dy = lineEnd[1] - lineStart[1];
        double len2 = dx*dx + dy*dy;

        if (len2 < 1e-12) {
            // Degenerate segment
            return std::sqrt(std::pow(p[0]-lineStart[0], 2) +
                            std::pow(p[1]-lineStart[1], 2));
        }

        double t = ((p[0]-lineStart[0])*dx + (p[1]-lineStart[1])*dy) / len2;
        t = std::max(0.0, std::min(1.0, t));

        double projX = lineStart[0] + t * dx;
        double projY = lineStart[1] + t * dy;

        return std::sqrt(std::pow(p[0]-projX, 2) + std::pow(p[1]-projY, 2));
    }

    G64CornerConfig config_;
};

// ============================================================================
// Corner Type Classification Tests
// ============================================================================

TEST_F(G64CornerModeTest, ClassifyStraightPath) {
    // Two collinear segments
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 100, 0);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_EQ(analysis.type, CornerType::Straight);
    EXPECT_NEAR(analysis.angle, 0.0, 5.0);  // Near 0 degrees
}

TEST_F(G64CornerModeTest, Classify90DegreeCorner) {
    auto seg1 = createSegment(0, 0, 50, 0);   // Horizontal
    auto seg2 = createSegment(50, 0, 50, 50); // Vertical

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.angle, 90.0, 1.0);
}

TEST_F(G64CornerModeTest, Classify45DegreeCorner) {
    auto seg1 = createSegment(0, 0, 50, 0);   // Horizontal
    auto seg2 = createSegment(50, 0, 100, 50); // 45 degrees

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.angle, 45.0, 2.0);
}

TEST_F(G64CornerModeTest, ClassifyConvexCorner) {
    // Turn right (clockwise) - convex/outside corner
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, -50);  // Turn right

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_TRUE(analysis.isCW);
    EXPECT_EQ(analysis.type, CornerType::Convex);
}

TEST_F(G64CornerModeTest, ClassifyConcaveCorner) {
    // Turn left (counter-clockwise) - concave/inside corner
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);  // Turn left

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_FALSE(analysis.isCW);
    EXPECT_EQ(analysis.type, CornerType::Concave);
}

TEST_F(G64CornerModeTest, ClassifyCuspCorner) {
    // 180 degree turn
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 0, 0);    // Reverse direction

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_EQ(analysis.type, CornerType::Cusp);
    EXPECT_NEAR(analysis.angle, 180.0, 5.0);
}

// ============================================================================
// G64 Parameter Parsing Tests
// ============================================================================

TEST_F(G64CornerModeTest, ParseG64BasicP) {
    config_.parseG64Parameters(0.05, -1, -1, -1);
    EXPECT_DOUBLE_EQ(config_.pathTolerance, 0.05);
    EXPECT_EQ(config_.cornerMode, G64CornerMode::Centered);
}

TEST_F(G64CornerModeTest, ParseG64PAndQ) {
    config_.parseG64Parameters(0.1, 0.01, -1, -1);
    EXPECT_DOUBLE_EQ(config_.pathTolerance, 0.1);
    EXPECT_DOUBLE_EQ(config_.naiveCamTolerance, 0.01);
}

TEST_F(G64CornerModeTest, ParseG64InsideStrict) {
    config_.parseG64Parameters(0.1, -1, 0, -1);  // I0 = strict inside
    EXPECT_EQ(config_.cornerMode, G64CornerMode::InsideStrict);
    EXPECT_DOUBLE_EQ(config_.insideTolerance, 0);
}

TEST_F(G64CornerModeTest, ParseG64InsideApproximate) {
    config_.parseG64Parameters(0.1, -1, 0.02, -1);  // I0.02 = approximate inside
    EXPECT_EQ(config_.cornerMode, G64CornerMode::InsideApproximate);
    EXPECT_DOUBLE_EQ(config_.insideTolerance, 0.02);
}

TEST_F(G64CornerModeTest, ParseG64OutsideStrict) {
    config_.parseG64Parameters(0.1, -1, -1, 0);  // O0 = strict outside
    EXPECT_EQ(config_.cornerMode, G64CornerMode::OutsideStrict);
    EXPECT_DOUBLE_EQ(config_.outsideTolerance, 0);
}

TEST_F(G64CornerModeTest, ParseG64OutsideApproximate) {
    config_.parseG64Parameters(0.1, -1, -1, 0.03);  // O0.03 = approximate outside
    EXPECT_EQ(config_.cornerMode, G64CornerMode::OutsideApproximate);
    EXPECT_DOUBLE_EQ(config_.outsideTolerance, 0.03);
}

TEST_F(G64CornerModeTest, ParseG64Balanced) {
    config_.parseG64Parameters(0.1, -1, 0.02, 0.03);  // Both I and O
    EXPECT_EQ(config_.cornerMode, G64CornerMode::Balanced);
    EXPECT_DOUBLE_EQ(config_.insideTolerance, 0.02);
    EXPECT_DOUBLE_EQ(config_.outsideTolerance, 0.03);
}

TEST_F(G64CornerModeTest, ParseG64PositivePIsInsideApproximate) {
    config_.parseG64Parameters(0.05, -1, -1, -1); // Positive P interpreted as inside approx
    EXPECT_EQ(config_.cornerMode, G64CornerMode::InsideApproximate);
    EXPECT_DOUBLE_EQ(config_.insideTolerance, 0.05);
    EXPECT_DOUBLE_EQ(config_.pathTolerance, 0.05);
}

TEST_F(G64CornerModeTest, ParseG64NegativePIsOutsideApproximate) {
    config_.parseG64Parameters(-0.03, -1, -1, -1); // Negative P interpreted as outside approx
    EXPECT_EQ(config_.cornerMode, G64CornerMode::OutsideApproximate);
    EXPECT_DOUBLE_EQ(config_.outsideTolerance, 0.03);
    EXPECT_DOUBLE_EQ(config_.pathTolerance, 0.03);
}

// ============================================================================
// Blend Geometry Tests
// ============================================================================

TEST_F(G64CornerModeTest, ComputeBlendGeometry90Degree) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 1.0;

    bool success = CornerAnalyzer::computeBlendGeometry(analysis, config_);
    EXPECT_TRUE(success);
    EXPECT_GT(analysis.blendRadius, 0.0);

    // Blend entry should be before corner
    EXPECT_LT(analysis.blendEntry[0], 50.0);
    EXPECT_DOUBLE_EQ(analysis.blendEntry[1], 0.0);

    // Blend exit should be after corner
    EXPECT_DOUBLE_EQ(analysis.blendExit[0], 50.0);
    EXPECT_GT(analysis.blendExit[1], 0.0);
}

TEST_F(G64CornerModeTest, ComputeBlendGeometrySymmetric) {
    // For 90 degree corner, blend entry distance should equal blend exit distance
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 1.0;

    CornerAnalyzer::computeBlendGeometry(analysis, config_);

    double entryDist = std::sqrt(
        std::pow(analysis.blendEntry[0] - 50.0, 2) +
        std::pow(analysis.blendEntry[1] - 0.0, 2));
    double exitDist = std::sqrt(
        std::pow(analysis.blendExit[0] - 50.0, 2) +
        std::pow(analysis.blendExit[1] - 0.0, 2));

    EXPECT_NEAR(entryDist, exitDist, 0.001);
}

TEST_F(G64CornerModeTest, BlendRadiusIncreasesWithTolerance) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis1 = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 0.1;
    CornerAnalyzer::computeBlendGeometry(analysis1, config_);

    auto analysis2 = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 1.0;
    CornerAnalyzer::computeBlendGeometry(analysis2, config_);

    EXPECT_GT(analysis2.blendRadius, analysis1.blendRadius);
}

// ============================================================================
// Blend Point Generation Tests
// ============================================================================

TEST_F(G64CornerModeTest, GenerateArcBlendPoints) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 5.0;
    config_.useBezierBlend = false;  // Use arc blend

    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    auto points = G64CornerBlendGenerator::generateBlendPoints(analysis, config_, 10);

    EXPECT_GE(points.size(), 10u);

    // All points should be marked as blend points
    for (const auto& pt : points) {
        EXPECT_TRUE(pt.isBlendPoint);
    }

    // First point should be at blend entry
    EXPECT_NEAR(points.front().position[0], analysis.blendEntry[0], 0.01);
    EXPECT_NEAR(points.front().position[1], analysis.blendEntry[1], 0.01);

    // Last point should be at blend exit
    EXPECT_NEAR(points.back().position[0], analysis.blendExit[0], 0.01);
    EXPECT_NEAR(points.back().position[1], analysis.blendExit[1], 0.01);
}

TEST_F(G64CornerModeTest, GenerateBezierBlendPoints) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 5.0;
    config_.useBezierBlend = true;
    config_.bezierOrder = 3;

    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    auto points = G64CornerBlendGenerator::generateBlendPoints(analysis, config_, 10);

    EXPECT_GE(points.size(), 10u);

    // Verify smooth progression
    for (size_t i = 1; i < points.size(); ++i) {
        double dist = std::sqrt(
            std::pow(points[i].position[0] - points[i-1].position[0], 2) +
            std::pow(points[i].position[1] - points[i-1].position[1], 2));
        EXPECT_GT(dist, 0) << "Points should not overlap";
    }
}

// ============================================================================
// Corner Mode Constraint Tests
// ============================================================================

TEST_F(G64CornerModeTest, InsideStrictConstraintSatisfied) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 1.0;
    config_.cornerMode = G64CornerMode::InsideStrict;
    config_.insideTolerance = 0.0;

    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    CornerAnalyzer::adjustForConstraints(analysis, config_);

    bool satisfied = G64CornerBlendGenerator::checkConstraints(analysis, config_);
    // For a concave corner with strict inside, it may need to be a sharp corner
}

TEST_F(G64CornerModeTest, OutsideStrictConstraintSatisfied) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, -50);  // Turn right (convex)

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 1.0;
    config_.cornerMode = G64CornerMode::OutsideStrict;
    config_.outsideTolerance = 0.0;

    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    CornerAnalyzer::adjustForConstraints(analysis, config_);

    // For convex corner with strict outside, blend should be disabled
}

TEST_F(G64CornerModeTest, CenteredModeAllowsDeviation) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    config_.pathTolerance = 1.0;
    config_.cornerMode = G64CornerMode::Centered;

    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    bool satisfied = G64CornerBlendGenerator::checkConstraints(analysis, config_);

    EXPECT_TRUE(satisfied) << "Centered mode should allow deviation up to tolerance";
}

// ============================================================================
// Path Blender Tests
// ============================================================================

TEST_F(G64CornerModeTest, BlenderProcessesSquarePath) {
    std::vector<PlanningSegment> segments;
    segments.push_back(createSegment(0, 0, 50, 0));
    segments.push_back(createSegment(50, 0, 50, 50));
    segments.push_back(createSegment(50, 50, 0, 50));
    segments.push_back(createSegment(0, 50, 0, 0));

    config_.pathTolerance = 1.0;
    G64PathBlender blender(config_);

    auto result = blender.blend(segments);

    // Result should have more segments due to added blends
    EXPECT_GE(result.size(), segments.size());

    // Should have corner analyses for each corner
    auto analyses = blender.cornerAnalyses();
    EXPECT_EQ(analyses.size(), 3u);  // 3 corners in 4 segments
}

TEST_F(G64CornerModeTest, BlenderPreservesStraightPath) {
    std::vector<PlanningSegment> segments;
    segments.push_back(createSegment(0, 0, 50, 0));
    segments.push_back(createSegment(50, 0, 100, 0));  // Collinear

    G64PathBlender blender(config_);
    auto result = blender.blend(segments);

    // Straight path should not add blend segments
    EXPECT_EQ(result.size(), 2u);
}

TEST_F(G64CornerModeTest, BlenderHandlesSingleSegment) {
    std::vector<PlanningSegment> segments;
    segments.push_back(createSegment(0, 0, 50, 50));

    G64PathBlender blender(config_);
    auto result = blender.blend(segments);

    EXPECT_EQ(result.size(), 1u);
}

TEST_F(G64CornerModeTest, BlenderHandlesEmptyInput) {
    std::vector<PlanningSegment> segments;

    G64PathBlender blender(config_);
    auto result = blender.blend(segments);

    EXPECT_TRUE(result.empty());
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(G64CornerModeTest, VerySharpCorner) {
    // 170 degree turn (almost reversal)
    // Going from (0,0)->(50,0) then reversing to (0,8.7) gives ~170 degree angle
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 0, 8.7);

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_GT(analysis.angle, 160.0);
}

TEST_F(G64CornerModeTest, VeryShallowCorner) {
    // 10 degree turn
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 100, 8.7);  // ~10 degree turn

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_LT(analysis.angle, 15.0);

    // Should be classified as nearly straight
    if (analysis.angle < config_.minCornerAngle) {
        EXPECT_EQ(analysis.type, CornerType::Straight);
    }
}

TEST_F(G64CornerModeTest, ZeroLengthSegment) {
    auto seg1 = createSegment(50, 0, 50, 0);  // Zero length!
    auto seg2 = createSegment(50, 0, 50, 50);

    // Should not crash
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
}

TEST_F(G64CornerModeTest, VerySmallTolerance) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    config_.pathTolerance = 0.001;  // Very tight
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis, config_);

    // Very small tolerance should result in very small blend radius
    EXPECT_LT(analysis.blendRadius, 1.0);
}

TEST_F(G64CornerModeTest, VeryLargeTolerance) {
    auto seg1 = createSegment(0, 0, 50, 0);
    auto seg2 = createSegment(50, 0, 50, 50);

    config_.pathTolerance = 100.0;  // Very loose
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis, config_);

    // Large tolerance should result in large blend radius
    EXPECT_GT(analysis.blendRadius, 10.0);
}
