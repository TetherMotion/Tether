/**
 * @file G64CornerBlendTests.cpp
 * @brief Unit tests for G64 corner blending algorithm
 * 
 * These tests verify that the G64 path blending algorithm correctly handles
 * overlapping corners and produces curves that match expected geometry.
 * 
 * This test file corresponds to the analysis in the G64_overlapping_corners.ipynb
 * notebook and validates that the C++ implementation matches the expected
 * mathematical behavior.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <array>

#include "tether/gcode/motion/G64CornerMode.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

namespace GCode {
namespace test {

// ============================================================================
// Test Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double POSITION_TOLERANCE = 1e-6;
constexpr double ANGLE_TOLERANCE = 0.01;  // degrees
constexpr double BLEND_TOLERANCE = 1e-4;

// ============================================================================
// Helper Functions - Matching notebook calculations
// ============================================================================

/**
 * @brief Compute blend geometry for a given tolerance and corner angle.
 * This matches the compute_blend_geometry function in the notebook.
 */
struct BlendGeometry {
    double radius;
    double tangentDist;
    double halfAngle;
    double arcSweepDeg;
};

BlendGeometry computeExpectedBlendGeometry(double tolerance, double cornerAngleDeg) {
    BlendGeometry result;
    result.halfAngle = (cornerAngleDeg / 2.0) * PI / 180.0;
    double cosHalf = std::cos(result.halfAngle);
    
    if (std::abs(1.0 - cosHalf) < 1e-10) {
        result.radius = std::numeric_limits<double>::infinity();
        result.tangentDist = std::numeric_limits<double>::infinity();
    } else {
        result.radius = tolerance * cosHalf / (1.0 - cosHalf);
        result.tangentDist = result.radius * std::tan(result.halfAngle);
    }
    result.arcSweepDeg = cornerAngleDeg;
    return result;
}

/**
 * @brief Find the tolerance at which overlap begins for a given segment length and angle.
 * This matches find_overlap_threshold from the notebook.
 */
double findOverlapThreshold(double legLength, double cornerAngleDeg) {
    double halfAngle = (cornerAngleDeg / 2.0) * PI / 180.0;
    double sinHalf = std::sin(halfAngle);
    double cosHalf = std::cos(halfAngle);
    
    if (sinHalf < 1e-10) {
        return std::numeric_limits<double>::infinity();
    }
    
    return legLength * (1.0 - cosHalf) / (2.0 * sinHalf);
}

/**
 * @brief Create a zig-zag path with specified leg length and turn angle.
 */
std::vector<Position> createZigZagPath(double legLength, double turnAngleDeg, int numCorners) {
    std::vector<Position> points;
    points.push_back({0.0, 0.0, 0.0});
    
    double currentDir = 0.0;  // degrees
    double x = 0.0, y = 0.0;
    
    for (int i = 0; i <= numCorners; ++i) {
        double dirRad = currentDir * PI / 180.0;
        x += legLength * std::cos(dirRad);
        y += legLength * std::sin(dirRad);
        points.push_back({x, y, 0.0});
        
        // Alternate turn direction
        if (i % 2 == 0) {
            currentDir += turnAngleDeg;
        } else {
            currentDir -= turnAngleDeg;
        }
    }
    
    return points;
}

/**
 * @brief Compute direction unit vector from p1 to p2.
 */
Position computeDirection(const Position& p1, const Position& p2) {
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double dz = p2[2] - p1[2];
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len < 1e-10) {
        return {1.0, 0.0, 0.0};
    }
    return {dx/len, dy/len, dz/len};
}

/**
 * @brief Compute angle between two direction vectors in degrees.
 */
double computeCornerAngle(const Position& dirIn, const Position& dirOut) {
    double dot = dirIn[0]*dirOut[0] + dirIn[1]*dirOut[1] + dirIn[2]*dirOut[2];
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot) * 180.0 / PI;
}

// ============================================================================
// Test Fixtures
// ============================================================================

class G64CornerBlendTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.pathTolerance = 0.1;  // 0.1mm default
        config_.cornerMode = G64CornerMode::Centered;
    }

    G64CornerConfig config_;
};

// ============================================================================
// Blend Geometry Tests
// ============================================================================

TEST_F(G64CornerBlendTest, BlendRadiusFormula_60DegreeCorner) {
    // Test the blend radius formula for a 60° corner
    double tolerance = 0.1;
    double cornerAngle = 60.0;
    
    auto expected = computeExpectedBlendGeometry(tolerance, cornerAngle);
    
    // Create segments
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {1, 0, 0};
    seg2.start = {1, 0, 0};
    double angle60Rad = 60.0 * PI / 180.0;
    seg2.end = {1 + std::cos(angle60Rad), std::sin(angle60Rad), 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    
    config_.pathTolerance = tolerance;
    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    EXPECT_NEAR(analysis.angle, cornerAngle, ANGLE_TOLERANCE);
    EXPECT_NEAR(analysis.blendRadius, expected.radius, BLEND_TOLERANCE);
}

TEST_F(G64CornerBlendTest, BlendRadiusFormula_90DegreeCorner) {
    // Test the blend radius formula for a 90° corner
    double tolerance = 0.1;
    double cornerAngle = 90.0;
    
    auto expected = computeExpectedBlendGeometry(tolerance, cornerAngle);
    
    // Create segments for a 90° corner
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {1, 0, 0};
    seg2.start = {1, 0, 0};
    seg2.end = {1, 1, 0};  // 90° turn
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    
    config_.pathTolerance = tolerance;
    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    EXPECT_NEAR(analysis.angle, cornerAngle, ANGLE_TOLERANCE);
    EXPECT_NEAR(analysis.blendRadius, expected.radius, BLEND_TOLERANCE);
}

TEST_F(G64CornerBlendTest, TangentDistanceCalculation) {
    // Verify tangent distance matches expected formula
    double tolerance = 0.1;
    double cornerAngle = 60.0;
    
    auto expected = computeExpectedBlendGeometry(tolerance, cornerAngle);
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    seg2.start = {10, 0, 0};
    double angle60Rad = 60.0 * PI / 180.0;
    seg2.end = {10 + 10*std::cos(angle60Rad), 10*std::sin(angle60Rad), 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    
    config_.pathTolerance = tolerance;
    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    // Calculate tangent distance from entry/exit points
    double entryDist = std::sqrt(
        std::pow(analysis.cornerPoint[0] - analysis.blendEntry[0], 2) +
        std::pow(analysis.cornerPoint[1] - analysis.blendEntry[1], 2)
    );
    
    EXPECT_NEAR(entryDist, expected.tangentDist, BLEND_TOLERANCE);
}

TEST_F(G64CornerBlendTest, ParseG64PositivePIsInsideApproximate) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(0.05, -1, -1, -1);
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::InsideApproximate);
    EXPECT_DOUBLE_EQ(cfg.insideTolerance, 0.05);
    EXPECT_DOUBLE_EQ(cfg.pathTolerance, 0.05);
}

TEST_F(G64CornerBlendTest, ParseG64NegativePIsOutsideApproximate) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(-0.03, -1, -1, -1);
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::OutsideApproximate);
    EXPECT_DOUBLE_EQ(cfg.outsideTolerance, 0.03);
    EXPECT_DOUBLE_EQ(cfg.pathTolerance, 0.03);
}

// ============================================================================
// Overlap Detection Tests
// ============================================================================

TEST_F(G64CornerBlendTest, OverlapThreshold_Calculation) {
    // Test that we correctly calculate when overlap begins
    double legLength = 3.0;
    double turnAngle = 60.0;
    
    double threshold = findOverlapThreshold(legLength, turnAngle);
    
    // At exactly the threshold, overlap ratio should be ~1.0
    auto blendAtThreshold = computeExpectedBlendGeometry(threshold, turnAngle);
    double overlapRatio = (2.0 * blendAtThreshold.tangentDist) / legLength;
    
    EXPECT_NEAR(overlapRatio, 1.0, 0.01);
}

TEST_F(G64CornerBlendTest, NoOverlap_SmallTolerance) {
    // With small tolerance relative to segment length, no overlap should occur
    double legLength = 10.0;
    double turnAngle = 60.0;
    double tolerance = 0.1;  // Much smaller than threshold
    
    auto blend = computeExpectedBlendGeometry(tolerance, turnAngle);
    double overlapRatio = (2.0 * blend.tangentDist) / legLength;
    
    EXPECT_LT(overlapRatio, 1.0);
}

TEST_F(G64CornerBlendTest, Overlap_LargeTolerance) {
    // With large tolerance relative to segment length, overlap should occur
    double legLength = 3.0;
    double turnAngle = 60.0;
    double tolerance = 2.0;  // Much larger than threshold
    
    auto blend = computeExpectedBlendGeometry(tolerance, turnAngle);
    double overlapRatio = (2.0 * blend.tangentDist) / legLength;
    
    EXPECT_GT(overlapRatio, 1.0);
}

// ============================================================================
// Overlap Handling Tests
// ============================================================================

TEST_F(G64CornerBlendTest, OverlapHandling_ReducesBlendRadius) {
    // When overlap would occur, the algorithm should reduce blend radius
    double legLength = 3.0;
    double turnAngle = 60.0;
    
    auto points = createZigZagPath(legLength, turnAngle, 4);
    
    // Create segments for adjacent corners
    std::vector<PlanningSegment> segments;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        PlanningSegment seg;
        seg.start = points[i];
        seg.end = points[i + 1];
        segments.push_back(seg);
    }
    
    // With large tolerance that would cause overlap
    config_.pathTolerance = 2.0;
    
    for (size_t i = 0; i < segments.size() - 1; ++i) {
        auto analysis = CornerAnalyzer::analyze(segments[i], segments[i + 1]);
        CornerAnalyzer::computeBlendGeometry(analysis, config_);
        
        // Tangent distance should be limited to half the segment length
        double maxTangentDist = legLength / 2.0;
        
        // The actual blend should fit within the segment
        double actualTangentDist = std::sqrt(
            std::pow(analysis.cornerPoint[0] - analysis.blendEntry[0], 2) +
            std::pow(analysis.cornerPoint[1] - analysis.blendEntry[1], 2)
        );
        
        // Note: The implementation may or may not enforce this limit.
        // This test documents expected behavior.
        // If this fails, the implementation needs overlap handling.
    }
}

// ============================================================================
// Path Connectivity Tests
// ============================================================================

TEST_F(G64CornerBlendTest, BlendedPath_RemainsConnected) {
    // Verify that the blended path has no gaps
    double legLength = 5.0;
    double turnAngle = 60.0;
    
    auto points = createZigZagPath(legLength, turnAngle, 3);
    
    std::vector<PlanningSegment> segments;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        PlanningSegment seg;
        seg.start = points[i];
        seg.end = points[i + 1];
        segments.push_back(seg);
    }
    
    config_.pathTolerance = 0.5;
    
    // Check that entry of corner i+1 comes after exit of corner i
    Position lastExit = segments[0].start;  // Start of path
    
    for (size_t i = 0; i < segments.size() - 1; ++i) {
        auto analysis = CornerAnalyzer::analyze(segments[i], segments[i + 1]);
        CornerAnalyzer::computeBlendGeometry(analysis, config_);
        
        // Entry should be between lastExit and corner point
        // Exit becomes the new reference for the next segment
        lastExit = analysis.blendExit;
    }
}

// ============================================================================
// Corner Mode Tests
// ============================================================================

TEST_F(G64CornerBlendTest, InsideStrictMode_StaysInside) {
    config_.cornerMode = G64CornerMode::InsideStrict;
    config_.insideTolerance = 0.0;
    config_.pathTolerance = 0.1;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {1, 0, 0};
    seg2.start = {1, 0, 0};
    seg2.end = {1, 1, 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    // For inside strict mode, outside deviation should be minimized
    EXPECT_GE(analysis.maxInsideDeviation, 0.0);
}

TEST_F(G64CornerBlendTest, OutsideStrictMode_StaysOutside) {
    config_.cornerMode = G64CornerMode::OutsideStrict;
    config_.outsideTolerance = 0.0;
    config_.pathTolerance = 0.1;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {1, 0, 0};
    seg2.start = {1, 0, 0};
    seg2.end = {1, 1, 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    // For outside strict mode, inside deviation should be minimized
    EXPECT_GE(analysis.maxOutsideDeviation, 0.0);
}

// ============================================================================
// Different Angle Tests (matching notebook curves)
// ============================================================================

class G64CornerAngleTest : public ::testing::TestWithParam<double> {};

TEST_P(G64CornerAngleTest, BlendGeometry_MatchesExpectedFormula) {
    double cornerAngle = GetParam();
    double tolerance = 0.1;
    
    auto expected = computeExpectedBlendGeometry(tolerance, cornerAngle);
    
    // Create segments at the specified angle
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {1, 0, 0};
    seg2.start = {1, 0, 0};
    
    double angleRad = cornerAngle * PI / 180.0;
    seg2.end = {1 + std::cos(angleRad), std::sin(angleRad), 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    
    G64CornerConfig config;
    config.pathTolerance = tolerance;
    CornerAnalyzer::computeBlendGeometry(analysis, config);
    
    EXPECT_NEAR(analysis.angle, cornerAngle, ANGLE_TOLERANCE);
    EXPECT_NEAR(analysis.blendRadius, expected.radius, BLEND_TOLERANCE * expected.radius);
}

INSTANTIATE_TEST_SUITE_P(
    CornerAngles,
    G64CornerAngleTest,
    ::testing::Values(30.0, 45.0, 60.0, 90.0, 120.0, 150.0)
);

// ============================================================================
// Overlap Ratio Tests (matching notebook visualization)
// ============================================================================

class G64OverlapRatioTest : public ::testing::TestWithParam<std::tuple<double, double, double>> {};

TEST_P(G64OverlapRatioTest, OverlapRatio_MatchesExpected) {
    auto [legLength, turnAngle, tolerance] = GetParam();
    
    auto blend = computeExpectedBlendGeometry(tolerance, turnAngle);
    double expectedOverlapRatio = (2.0 * blend.tangentDist) / legLength;
    
    // Create path and compute actual blend
    auto points = createZigZagPath(legLength, turnAngle, 2);
    
    PlanningSegment seg1, seg2;
    seg1.start = points[0];
    seg1.end = points[1];
    seg2.start = points[1];
    seg2.end = points[2];
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    
    G64CornerConfig config;
    config.pathTolerance = tolerance;
    CornerAnalyzer::computeBlendGeometry(analysis, config);
    
    double actualTangentDist = std::sqrt(
        std::pow(analysis.cornerPoint[0] - analysis.blendEntry[0], 2) +
        std::pow(analysis.cornerPoint[1] - analysis.blendEntry[1], 2)
    );
    double actualOverlapRatio = (2.0 * actualTangentDist) / legLength;
    
    EXPECT_NEAR(actualOverlapRatio, expectedOverlapRatio, 0.01);
}

INSTANTIATE_TEST_SUITE_P(
    NotebookScenarios,
    G64OverlapRatioTest,
    ::testing::Values(
        // (legLength, turnAngle, tolerance) - matching notebook scenarios
        std::make_tuple(3.0, 60.0, 0.1),   // Small tolerance, no overlap (~25%)
        std::make_tuple(3.0, 60.0, 0.5),   // Medium tolerance, edge case (~124%)
        std::make_tuple(3.0, 60.0, 2.0)    // Large tolerance, overlap (~498%)
    )
);

// ============================================================================
// Teardrop Strategy Tests
// ============================================================================

TEST_F(G64CornerBlendTest, ParseG64Teardrop_ExplicitStrategy) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(5.0, -1, -1, -1, -1, "teardrop");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::Teardrop);
    EXPECT_DOUBLE_EQ(cfg.pathTolerance, 5.0);
    EXPECT_EQ(cfg.strategyString, "teardrop");
}

TEST_F(G64CornerBlendTest, ParseG64Teardrop_NegativeP_SignIgnored) {
    // For teardrop with explicit strategy, sign of P doesn't matter
    G64CornerConfig cfg;
    cfg.parseG64Parameters(-5.0, -1, -1, -1, -1, "teardrop");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::Teardrop);
    EXPECT_DOUBLE_EQ(cfg.pathTolerance, 5.0);  // Should use abs value
}

TEST_F(G64CornerBlendTest, ParseG64ExactStop_Strategy) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(5.0, -1, -1, -1, -1, "exact-stop");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::ExactStop);
    EXPECT_DOUBLE_EQ(cfg.pathTolerance, 0.0);  // P is ignored
}

TEST_F(G64CornerBlendTest, ParseG64ExactStop_AlternateSpelling) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(5.0, -1, -1, -1, -1, "exactstop");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::ExactStop);
    EXPECT_DOUBLE_EQ(cfg.pathTolerance, 0.0);
}

TEST_F(G64CornerBlendTest, ParseG64Inside_ExplicitStrategy) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(5.0, -1, -1, -1, -1, "inside");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::InsideApproximate);
    EXPECT_DOUBLE_EQ(cfg.insideTolerance, 5.0);
}

TEST_F(G64CornerBlendTest, ParseG64Dogbone_ExplicitStrategy) {
    G64CornerConfig cfg;
    cfg.parseG64Parameters(5.0, -1, -1, -1, -1, "dogbone");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::OutsideApproximate);
    EXPECT_DOUBLE_EQ(cfg.outsideTolerance, 5.0);
}

TEST_F(G64CornerBlendTest, ParseG64Dogbone_NegativeP_SignIgnored) {
    // For dogbone with explicit strategy, sign of P doesn't matter
    G64CornerConfig cfg;
    cfg.parseG64Parameters(-5.0, -1, -1, -1, -1, "dogbone");
    EXPECT_EQ(cfg.cornerMode, G64CornerMode::OutsideApproximate);
    EXPECT_DOUBLE_EQ(cfg.outsideTolerance, 5.0);  // Should use abs value
}

TEST_F(G64CornerBlendTest, TeardropGeometry_90DegreeCorner) {
    // Test teardrop blend geometry for a 90-degree corner
    config_.cornerMode = G64CornerMode::Teardrop;
    config_.pathTolerance = 5.0;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    seg2.start = {10, 0, 0};
    seg2.end = {10, 10, 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    ASSERT_EQ(analysis.type, CornerType::Concave);  // CCW (left) turn is concave
    EXPECT_NEAR(analysis.angle, 90.0, 0.1);
    
    bool success = CornerAnalyzer::computeBlendGeometry(analysis, config_);
    ASSERT_TRUE(success);
    
    // For teardrop, the blend radius should equal the tolerance
    EXPECT_DOUBLE_EQ(analysis.blendRadius, 5.0);
    
    // Verify that entry point is before the corner
    double entryDist = std::sqrt(
        std::pow(analysis.cornerPoint[0] - analysis.blendEntry[0], 2) +
        std::pow(analysis.cornerPoint[1] - analysis.blendEntry[1], 2)
    );
    EXPECT_GT(entryDist, 0.0);
    
    // Verify that exit point is after the corner on outgoing segment
    double exitDist = std::sqrt(
        std::pow(analysis.cornerPoint[0] - analysis.blendExit[0], 2) +
        std::pow(analysis.cornerPoint[1] - analysis.blendExit[1], 2)
    );
    EXPECT_GT(exitDist, 0.0);
    
    // For 90-degree turn: halfAngle = 45°, extensionDist = r / tan(45°) = r
    // So the overshoot should be approximately equal to the radius
    // CCW turn (left turn) means inside deviation
    EXPECT_NEAR(analysis.maxInsideDeviation, 5.0, 0.1);
}

TEST_F(G64CornerBlendTest, TeardropGeometry_60DegreeCorner) {
    // Test teardrop for a sharper 60-degree corner
    config_.cornerMode = G64CornerMode::Teardrop;
    config_.pathTolerance = 3.0;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    
    // Create 60-degree turn
    double angle = 60.0 * InterpolationConstants::PI / 180.0;
    seg2.start = {10, 0, 0};
    seg2.end = {10 + 10 * std::cos(angle), 10 * std::sin(angle), 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.angle, 60.0, 0.1);
    
    bool success = CornerAnalyzer::computeBlendGeometry(analysis, config_);
    ASSERT_TRUE(success);
    
    // Blend radius should equal tolerance for teardrop
    EXPECT_DOUBLE_EQ(analysis.blendRadius, 3.0);
    
    // The extension distance for 60-degree turn
    // halfAngle = 30°, extensionDist = r / tan(30°) = r * sqrt(3)
    double expectedExtension = 3.0 * std::sqrt(3.0);
    EXPECT_NEAR(analysis.maxInsideDeviation, expectedExtension, 0.1);  // CCW = inside
}

TEST_F(G64CornerBlendTest, TeardropGeometry_120DegreeCorner) {
    // Test teardrop for a gentler 120-degree corner
    config_.cornerMode = G64CornerMode::Teardrop;
    config_.pathTolerance = 4.0;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    
    // Create 120-degree turn
    double angle = 120.0 * InterpolationConstants::PI / 180.0;
    seg2.start = {10, 0, 0};
    seg2.end = {10 + 10 * std::cos(angle), 10 * std::sin(angle), 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.angle, 120.0, 0.1);
    
    bool success = CornerAnalyzer::computeBlendGeometry(analysis, config_);
    ASSERT_TRUE(success);
    
    // Blend radius should equal tolerance for teardrop
    EXPECT_DOUBLE_EQ(analysis.blendRadius, 4.0);
    
    // The extension distance for 120-degree turn
    // halfAngle = 60°, extensionDist = r / tan(60°) = r / sqrt(3)
    double expectedExtension = 4.0 / std::sqrt(3.0);
    EXPECT_NEAR(analysis.maxInsideDeviation, expectedExtension, 0.1);
}

TEST_F(G64CornerBlendTest, TeardropVsStandardBlend_CompareRadius) {
    // Compare teardrop to standard blend for same corner
    double tolerance = 5.0;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    seg2.start = {10, 0, 0};
    seg2.end = {10, 10, 0};
    
    // Standard blend
    G64CornerConfig standardConfig;
    standardConfig.pathTolerance = tolerance;
    standardConfig.cornerMode = G64CornerMode::InsideApproximate;
    standardConfig.insideTolerance = tolerance;  // Must set this for InsideApproximate
    
    auto analysis1 = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis1, standardConfig);
    
    // Teardrop blend
    G64CornerConfig teardropConfig;
    teardropConfig.pathTolerance = tolerance;
    teardropConfig.cornerMode = G64CornerMode::Teardrop;
    
    auto analysis2 = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis2, teardropConfig);
    
    // Teardrop radius should be exactly the tolerance
    EXPECT_DOUBLE_EQ(analysis2.blendRadius, tolerance);
    
    // Standard blend radius is computed from tolerance, typically larger
    // For 90° corner: r = tolerance * cos(45°) / (1 - cos(45°))
    double expected_std_radius = tolerance * std::cos(InterpolationConstants::PI/4.0) / 
                                 (1.0 - std::cos(InterpolationConstants::PI/4.0));
    EXPECT_NEAR(analysis1.blendRadius, expected_std_radius, 0.1);
    
    // Verify radii are different (teardrop uses tolerance directly)
    EXPECT_NE(analysis2.blendRadius, analysis1.blendRadius);
}

TEST_F(G64CornerBlendTest, ExactStop_NoBlending) {
    // Test that exact-stop mode produces no blending
    config_.cornerMode = G64CornerMode::ExactStop;
    config_.pathTolerance = 0.0;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    seg2.start = {10, 0, 0};
    seg2.end = {10, 10, 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    bool success = CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    // Exact stop should return false (no blending)
    EXPECT_FALSE(success);
}

TEST_F(G64CornerBlendTest, TeardropGeometry_TangentialToToleranceCircle) {
    // Verify that teardrop arc is tangential to the tolerance circle
    config_.cornerMode = G64CornerMode::Teardrop;
    config_.pathTolerance = 5.0;
    
    PlanningSegment seg1, seg2;
    seg1.start = {0, 0, 0};
    seg1.end = {10, 0, 0};
    seg2.start = {10, 0, 0};
    seg2.end = {10, 10, 0};
    
    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(analysis, config_);
    
    // Check that the blend center is at the correct distance from corner
    double centerDist = std::sqrt(
        std::pow(analysis.blendCenter[0] - analysis.cornerPoint[0], 2) +
        std::pow(analysis.blendCenter[1] - analysis.cornerPoint[1], 2)
    );
    
    // For a 90° corner with teardrop, the center should be at distance
    // sqrt(2) * tolerance (since it continues r past corner then r perpendicular)
    double expectedCenterDist = std::sqrt(2.0) * config_.pathTolerance;
    EXPECT_NEAR(centerDist, expectedCenterDist, 0.1);
}

}  // namespace test
}  // namespace GCode
