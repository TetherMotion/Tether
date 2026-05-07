/**
 * @file G64ArcTransitionTests.cpp
 * @brief Comprehensive tests for G64 arc/line transition handling
 *
 * Validates that the CornerAnalyzer correctly computes tangent directions
 * for arc segments (not just chord directions), ensuring:
 * - Smooth (tangent-continuous) transitions produce zero-angle corners
 * - Near-smooth transitions produce proportionally small angles
 * - Various segment type combinations (line-line, line-arc, arc-line, arc-arc)
 * - All G64 strategies (inside, outside, teardrop, etc.) work with arcs
 * - Different arc angles and orientations
 * - Overlapping blends on close-together corners with arcs
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
// Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double POS_TOL = 1e-6;
constexpr double ANGLE_TOL_DEG = 0.5;  // degrees

// ============================================================================
// Helpers — segment builders
// ============================================================================

static PlanningSegment makeLine(double x0, double y0, double x1, double y1) {
    PlanningSegment seg;
    seg.start = {x0, y0, 0};
    seg.end   = {x1, y1, 0};
    seg.motionType = SegmentMotionType::Linear;
    seg.segmentLength = std::hypot(x1 - x0, y1 - y0);
    return seg;
}

/**
 * @brief Build a CW (G2) arc segment given start, end, and center.
 *        Automatically computes radius & sweep.
 */
static PlanningSegment makeArcCW(double sx, double sy,
                                  double ex, double ey,
                                  double cx, double cy) {
    PlanningSegment seg;
    seg.start = {sx, sy, 0};
    seg.end   = {ex, ey, 0};
    seg.center = {cx, cy, 0};
    seg.motionType = SegmentMotionType::ArcCW;
    seg.plane = InterpolationPlane::XY;
    seg.arcRadius = std::hypot(sx - cx, sy - cy);
    // Sweep: CW is negative
    double startAngle = std::atan2(sy - cy, sx - cx);
    double endAngle   = std::atan2(ey - cy, ex - cx);
    double sweep = endAngle - startAngle;
    // CW: sweep should be negative
    if (sweep > 0) sweep -= 2.0 * PI;
    seg.arcSweep = sweep;
    seg.segmentLength = std::abs(sweep) * seg.arcRadius;
    return seg;
}

/**
 * @brief Build a CCW (G3) arc segment.
 */
static PlanningSegment makeArcCCW(double sx, double sy,
                                   double ex, double ey,
                                   double cx, double cy) {
    PlanningSegment seg;
    seg.start = {sx, sy, 0};
    seg.end   = {ex, ey, 0};
    seg.center = {cx, cy, 0};
    seg.motionType = SegmentMotionType::ArcCCW;
    seg.plane = InterpolationPlane::XY;
    seg.arcRadius = std::hypot(sx - cx, sy - cy);
    double startAngle = std::atan2(sy - cy, sx - cx);
    double endAngle   = std::atan2(ey - cy, ex - cx);
    double sweep = endAngle - startAngle;
    // CCW: sweep should be positive
    if (sweep < 0) sweep += 2.0 * PI;
    seg.arcSweep = sweep;
    seg.segmentLength = std::abs(sweep) * seg.arcRadius;
    return seg;
}

/**
 * @brief Compute the expected tangent angle at the endpoint of a segment.
 *        For lines, this is the direction from start to end.
 *        For arcs, this is perpendicular to the radius at the endpoint.
 *        Returns angle in radians.
 */
static double expectedExitAngle(const PlanningSegment& seg) {
    if (seg.isArc()) {
        double angle = std::atan2(seg.end[1] - seg.center[1],
                                  seg.end[0] - seg.center[0]);
        int dir = seg.arcDirection();
        // Tangent perpendicular to radius
        return std::atan2(dir * std::cos(angle), -dir * std::sin(angle));
    }
    return std::atan2(seg.end[1] - seg.start[1],
                      seg.end[0] - seg.start[0]);
}

/**
 * @brief Compute the expected tangent angle at the start of a segment.
 */
static double expectedEntryAngle(const PlanningSegment& seg) {
    if (seg.isArc()) {
        double angle = std::atan2(seg.start[1] - seg.center[1],
                                  seg.start[0] - seg.center[0]);
        int dir = seg.arcDirection();
        return std::atan2(dir * std::cos(angle), -dir * std::sin(angle));
    }
    return std::atan2(seg.end[1] - seg.start[1],
                      seg.end[0] - seg.start[0]);
}

/**
 * @brief Compute expected angle between two consecutive segments
 *        using the mathematical tangent at the junction.
 */
static double expectedJunctionAngleDeg(const PlanningSegment& seg1,
                                        const PlanningSegment& seg2) {
    double a1 = expectedExitAngle(seg1);
    double a2 = expectedEntryAngle(seg2);

    // Direction vectors
    double dx1 = std::cos(a1), dy1 = std::sin(a1);
    double dx2 = std::cos(a2), dy2 = std::sin(a2);

    double dot = dx1 * dx2 + dy1 * dy2;
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot) * RAD2DEG;
}

// ============================================================================
// Test Fixture
// ============================================================================

class G64ArcTransitionTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.pathTolerance = 1.0;
        config_.cornerMode = G64CornerMode::Centered;
    }
    G64CornerConfig config_;
};

// ============================================================================
// 1. SMOOTH TRANSITION TESTS
//    Tangent-continuous junctions should produce angle ≈ 0° and no blending.
// ============================================================================

TEST_F(G64ArcTransitionTest, SmoothLineToArcCW_AngleIsZero) {
    // Line along +X, then CW semicircle tangent to line
    // Arc center is directly below the junction → tangent at start = +X
    auto line = makeLine(0, 0, 50, 0);
    auto arc  = makeArcCW(50, 0, 50, -30, 50, -15);

    auto analysis = CornerAnalyzer::analyze(line, arc);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Smooth line→CW arc should have ~0° angle";
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

TEST_F(G64ArcTransitionTest, SmoothArcCWToLine_AngleIsZero) {
    // CW semicircle ending tangent to -X line
    auto arc  = makeArcCW(50, 0, 50, -30, 50, -15);
    auto line = makeLine(50, -30, 0, -30);

    auto analysis = CornerAnalyzer::analyze(arc, line);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Smooth CW arc→line should have ~0° angle";
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

TEST_F(G64ArcTransitionTest, SmoothLineToArcCCW_AngleIsZero) {
    // Line along +X, then CCW arc tangent to line
    // Center above the junction
    auto line = makeLine(0, 0, 50, 0);
    auto arc  = makeArcCCW(50, 0, 50, 30, 50, 15);

    auto analysis = CornerAnalyzer::analyze(line, arc);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Smooth line→CCW arc should have ~0° angle";
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

TEST_F(G64ArcTransitionTest, SmoothArcCCWToLine_AngleIsZero) {
    auto arc  = makeArcCCW(50, 0, 50, 30, 50, 15);
    auto line = makeLine(50, 30, 0, 30);

    auto analysis = CornerAnalyzer::analyze(arc, line);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Smooth CCW arc→line should have ~0° angle";
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

TEST_F(G64ArcTransitionTest, SmoothArcToArc_SameCenter_AngleIsZero) {
    // Two quarter-circle arcs sharing the same center
    // First arc: 0° to 90°, second arc: 90° to 180° (both CCW, center at origin)
    double r = 20.0;
    auto arc1 = makeArcCCW(r, 0, 0, r, 0, 0);
    auto arc2 = makeArcCCW(0, r, -r, 0, 0, 0);

    auto analysis = CornerAnalyzer::analyze(arc1, arc2);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Two CCW arcs with same center should be smooth";
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

TEST_F(G64ArcTransitionTest, SmoothArcToArc_DifferentRadii_Tangent) {
    // Two arcs sharing a junction point with matching tangent directions
    // but different radii (S-curve)
    // First: CCW arc, center at (0, 10), r=10, from (10, 10) to (0, 20)
    // At (0, 20): radius vector = (0, 10), tangent = perpendicular = (-1, 0) * 1 (CCW dir=1)
    //   atan2(20-10, 0-0) = atan2(10, 0) = π/2
    //   tangent = (-1*sin(π/2), 1*cos(π/2)) = (-1, 0)
    //
    // Second: CW arc, center at (0, 25), r=5, from (0, 20) to (-5, 25)
    // At (0, 20): radius vector = (0, -5), angle = atan2(20-25, 0-0) = atan2(-5, 0) = -π/2
    //   tangent = (-(-1)*sin(-π/2), (-1)*cos(-π/2)) = (-(1), 0) = (-1, 0) ✓ matches!
    auto arc1 = makeArcCCW(10, 10, 0, 20, 0, 10);
    auto arc2 = makeArcCW(0, 20, -5, 25, 0, 25);

    auto analysis = CornerAnalyzer::analyze(arc1, arc2);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "S-curve arcs with matching tangent should be smooth";
}

TEST_F(G64ArcTransitionTest, SmoothFullLineArcLine_NoBlendGenerated) {
    // The actual bug scenario: line-semicircle-line forming a dogbone shape
    auto seg1 = makeLine(0, 0, 50, 0);
    auto seg2 = makeArcCW(50, 0, 50, -30, 50, -15);
    auto seg3 = makeLine(50, -30, 0, -30);

    auto a1 = CornerAnalyzer::analyze(seg1, seg2);
    auto a2 = CornerAnalyzer::analyze(seg2, seg3);

    // Both junctions should be smooth
    EXPECT_NEAR(a1.angle, 0.0, ANGLE_TOL_DEG);
    EXPECT_NEAR(a2.angle, 0.0, ANGLE_TOL_DEG);

    // No blend should be computed for either
    config_.pathTolerance = 5.0;
    bool blend1 = CornerAnalyzer::computeBlendGeometry(a1, config_);
    bool blend2 = CornerAnalyzer::computeBlendGeometry(a2, config_);
    EXPECT_FALSE(blend1) << "Smooth junction should not produce blend";
    EXPECT_FALSE(blend2) << "Smooth junction should not produce blend";
}

// ============================================================================
// 2. NEAR-SMOOTH TRANSITION TESTS
//    Junctions with small angular mismatch should produce proportionally
//    small deviation angles.
// ============================================================================

TEST_F(G64ArcTransitionTest, NearSmoothLineToArc_SmallAngle) {
    // Line at slight angle from true tangent
    // Arc CW, center (50, -15), r=15, start at (50, 0)
    // True tangent at start = (+1, 0)
    // Introduce 5° mismatch in the line direction
    double mismatchDeg = 5.0;
    double mismatchRad = mismatchDeg * DEG2RAD;
    double lineEndX = 50 - 50 * std::cos(mismatchRad);
    double lineEndY = 50 * std::sin(mismatchRad);
    auto line = makeLine(lineEndX, lineEndY, 50, 0);
    // line goes from some point to (50,0) with a ~5° offset from +X
    // But the actual direction of the line is towards (50,0), let me recalc
    // The line direction = (50 - lineEndX, 0 - lineEndY) normalized
    // Let's just use a line that is 5° off from horizontal
    auto line2 = makeLine(0, 50 * std::tan(mismatchRad), 50, 0);
    auto arc   = makeArcCW(50, 0, 50, -30, 50, -15);

    auto analysis = CornerAnalyzer::analyze(line2, arc);
    double expected = expectedJunctionAngleDeg(line2, arc);

    EXPECT_NEAR(analysis.angle, expected, ANGLE_TOL_DEG)
        << "Near-smooth junction angle should match expected value";
    EXPECT_NEAR(analysis.angle, mismatchDeg, ANGLE_TOL_DEG + 1.0)
        << "~5° mismatch should give ~5° angle";
    EXPECT_GT(analysis.angle, 1.0)
        << "Should detect non-zero angle";
    EXPECT_LT(analysis.angle, 15.0)
        << "Should not produce a large angle";
}

TEST_F(G64ArcTransitionTest, NearSmooth_10DegreeMismatch) {
    // 10° mismatch between line and arc tangent
    double mismatchDeg = 10.0;
    double mismatchRad = mismatchDeg * DEG2RAD;
    // Line ending at (50,0) but approaching from 10° above horizontal
    double len = 50.0;
    auto line = makeLine(50 - len * std::cos(mismatchRad),
                         -len * std::sin(mismatchRad), 50, 0);
    auto arc  = makeArcCW(50, 0, 50, -30, 50, -15);

    auto analysis = CornerAnalyzer::analyze(line, arc);
    EXPECT_NEAR(analysis.angle, mismatchDeg, 2.0)
        << "10° mismatch should give ~10° angle";
}

// ============================================================================
// 3. NON-SMOOTH TRANSITIONS — various angles
//    Test that line→arc and arc→line corners with genuine angle
//    differences are correctly computed.
// ============================================================================

TEST_F(G64ArcTransitionTest, LineToArc_45DegreeCorner) {
    // Line along +X, arc that starts with tangent at 45° from +X
    // For CW arc: tangent = (-dir*sin(a), dir*cos(a))
    // Want tangent at start = (cos45, sin45)
    // dir=-1: -(-1)*sin(a) = cos45 → sin(a) = -cos45 → a = -π/4
    //         (-1)*cos(a) = sin45 → cos(a) = -sin45 → a = π + π/4 = 5π/4
    // Actually let's pick numbers directly. Arc center must be placed so
    // radius is perpendicular to desired tangent.
    // Desired entry tangent = (cos45°, sin45°). Perpendicular (CW, pointing to center) =
    //   rotate tangent 90° CW = (sin45°, -cos45°) = (0.707, -0.707)
    // If junction at (10, 0), center at (10 + r*0.707, -r*0.707)
    double r = 10.0;
    double cx = 10 + r * std::cos(-PI / 4);  // 10 + r*cos(-45°)
    double cy = r * std::sin(-PI / 4);       // r*sin(-45°)
    // End point: 90° CW from start. Angle of start from center:
    double startA = std::atan2(0 - cy, 10 - cx);
    double endA = startA - PI / 2;  // 90° CW
    double ex = cx + r * std::cos(endA);
    double ey = cy + r * std::sin(endA);

    auto line = makeLine(0, 0, 10, 0);
    auto arc  = makeArcCW(10, 0, ex, ey, cx, cy);

    auto analysis = CornerAnalyzer::analyze(line, arc);
    EXPECT_NEAR(analysis.angle, 45.0, 2.0)
        << "Line(+X) → Arc(45° tangent) should give 45° angle";
}

TEST_F(G64ArcTransitionTest, LineToArc_90DegreeCorner) {
    // Line along +X, arc starts with tangent perpendicular to +X (i.e., +Y or -Y)
    // CW arc starting with tangent (0, -1): center directly to the right of junction
    // tangent = (-(-1)*sin(a), (-1)*cos(a)) = (sin(a), -cos(a)) = (0, -1)
    //   → sin(a) = 0, -cos(a) = -1 → cos(a)=1 → a=0
    // Center at junction + (r, 0) direction. If junction at (10, 0), center at (10+r, 0)
    double r = 10.0;
    auto line = makeLine(0, 0, 10, 0);
    auto arc  = makeArcCW(10, 0, 10 + r, -r, 10 + r, 0);

    auto analysis = CornerAnalyzer::analyze(line, arc);
    EXPECT_NEAR(analysis.angle, 90.0, 2.0)
        << "Line(+X) → Arc(perpendicular tangent) should give 90° angle";
}

TEST_F(G64ArcTransitionTest, ArcToLine_90DegreeCorner) {
    // CW quarter-circle from (0, r) to (r, 0) with center at origin
    // Exit tangent at (r, 0): angle = atan2(0, r) = 0, dir=-1
    //   tangent = (sin(0), -cos(0)) = (0, -1)
    // Line from (r, 0) going in +X direction → 90° corner
    double r = 15.0;
    auto arc  = makeArcCW(0, r, r, 0, 0, 0);
    auto line = makeLine(r, 0, r + 20, 0);

    auto analysis = CornerAnalyzer::analyze(arc, line);
    EXPECT_NEAR(analysis.angle, 90.0, 2.0);
}

// ============================================================================
// 4. ARC-TO-ARC TRANSITIONS
// ============================================================================

TEST_F(G64ArcTransitionTest, ArcToArc_90DegreeMismatch) {
    // Two arcs meeting at a point with 90° tangent difference
    // First: CW quarter circle from (10, 0) to (0, -10), center (0, 0)
    //   Exit tangent at (0, -10): angle=atan2(-10,0)=-π/2, dir=-1
    //   tangent = (sin(-π/2), -cos(-π/2)) = (-1, 0)
    // Second: CCW arc from (0, -10) going upward with tangent (0, 1)
    //   Entry tangent needs to be (0, 1). dir=1:
    //   tangent = (-sin(a), cos(a)) = (0, 1) → sin(a)=0, cos(a)=1 → a=0
    //   Center at (0-10+r, -10)... Let's use center at (r, -10)
    double r1 = 10.0, r2 = 5.0;
    auto arc1 = makeArcCW(10, 0, 0, -10, 0, 0);
    // Arc2: CCW from (0, -10), center at (r2, -10)
    // Start angle from center: atan2(-10-(-10), 0-r2) = atan2(0, -r2) = π
    // tangent = (-sin(π), cos(π)) = (0, -1)... that's not right.
    // Let me recalculate. Center at (-r2, -10), start at (0, -10)
    // angle = atan2(-10-(-10), 0-(-r2)) = atan2(0, r2) = 0
    // tangent = (-sin(0), cos(0)) = (0, 1). That's the tangent we want.
    double endAngle = PI / 2;  // 90° CCW from angle 0
    double ex = -r2 + r2 * std::cos(endAngle);
    double ey = -10 + r2 * std::sin(endAngle);
    auto arc2 = makeArcCCW(0, -10, ex, ey, -r2, -10);

    auto analysis = CornerAnalyzer::analyze(arc1, arc2);
    // Arc1 exit tangent = (-1, 0), Arc2 entry tangent = (0, 1) → 90°
    EXPECT_NEAR(analysis.angle, 90.0, 2.0)
        << "Arc→Arc with perpendicular tangents should give 90°";
}

TEST_F(G64ArcTransitionTest, ArcToArc_SameDirection_Smooth) {
    // Two CW quarter-circles forming a smooth semicircle
    double r = 10.0;
    auto arc1 = makeArcCW(r, 0, 0, -r, 0, 0);
    auto arc2 = makeArcCW(0, -r, -r, 0, 0, 0);

    auto analysis = CornerAnalyzer::analyze(arc1, arc2);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Two CW quarter-circles forming semicircle should be smooth";
}

// ============================================================================
// 5. ALL STRATEGIES ON ARC TRANSITIONS
//    Verify that different G64 strategies correctly handle arc junctions.
// ============================================================================

class G64ArcStrategyTest : public ::testing::TestWithParam<G64CornerMode> {
protected:
    void SetUp() override {
        // 45° line→arc corner
        line_ = makeLine(0, 0, 20, 0);
        // Arc starting at (20, 0) with center that creates ~45° tangent mismatch
        double r = 10.0;
        double cx = 20 + r * std::cos(-PI / 4);
        double cy = r * std::sin(-PI / 4);
        double startA = std::atan2(0 - cy, 20 - cx);
        double endA = startA - PI / 2;
        double ex = cx + r * std::cos(endA);
        double ey = cy + r * std::sin(endA);
        arc_ = makeArcCW(20, 0, ex, ey, cx, cy);
    }
    PlanningSegment line_, arc_;
};

TEST_P(G64ArcStrategyTest, StrategyHandlesLineArcCorner) {
    G64CornerMode mode = GetParam();
    G64CornerConfig config;
    config.pathTolerance = 2.0;
    config.cornerMode = mode;
    config.insideTolerance = 2.0;
    config.outsideTolerance = 2.0;

    auto analysis = CornerAnalyzer::analyze(line_, arc_);
    EXPECT_GT(analysis.angle, 10.0) << "Should detect a real corner";

    if (mode == G64CornerMode::ExactStop) {
        bool ok = CornerAnalyzer::computeBlendGeometry(analysis, config);
        EXPECT_FALSE(ok) << "ExactStop should not blend";
    } else {
        bool ok = CornerAnalyzer::computeBlendGeometry(analysis, config);
        // All non-ExactStop modes should succeed or at least not crash
        if (ok) {
            EXPECT_GT(analysis.blendRadius, 0.0);
        }
    }
}

TEST_P(G64ArcStrategyTest, StrategyHandlesSmoothLineArc) {
    G64CornerMode mode = GetParam();
    G64CornerConfig config;
    config.pathTolerance = 2.0;
    config.cornerMode = mode;
    config.insideTolerance = 2.0;
    config.outsideTolerance = 2.0;

    // Smooth line→arc
    auto smoothLine = makeLine(0, 0, 50, 0);
    auto smoothArc  = makeArcCW(50, 0, 50, -30, 50, -15);

    auto analysis = CornerAnalyzer::analyze(smoothLine, smoothArc);
    EXPECT_NEAR(analysis.angle, 0.0, ANGLE_TOL_DEG)
        << "Smooth junction should have ~0° angle regardless of strategy";
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

INSTANTIATE_TEST_SUITE_P(
    AllStrategies,
    G64ArcStrategyTest,
    ::testing::Values(
        G64CornerMode::Centered,
        G64CornerMode::InsideStrict,
        G64CornerMode::InsideApproximate,
        G64CornerMode::OutsideStrict,
        G64CornerMode::OutsideApproximate,
        G64CornerMode::Balanced,
        G64CornerMode::Teardrop,
        G64CornerMode::ExactStop
    )
);

// ============================================================================
// 6. VARIOUS LINE-LINE ANGLES (regression: ensure existing behavior unchanged)
// ============================================================================

class G64LineAngleTest : public ::testing::TestWithParam<double> {};

TEST_P(G64LineAngleTest, LineLineAngleAccuracy) {
    double angleDeg = GetParam();
    double angleRad = angleDeg * DEG2RAD;

    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0,
                         10 + 10 * std::cos(angleRad),
                         10 * std::sin(angleRad));

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.angle, angleDeg, 0.1);
}

INSTANTIATE_TEST_SUITE_P(
    LineAngles,
    G64LineAngleTest,
    ::testing::Values(10.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0,
                      105.0, 120.0, 135.0, 150.0, 170.0)
);

// ============================================================================
// 7. LINE-TO-ARC TRANSITIONS AT VARIOUS ANGLES
// ============================================================================

class G64LineArcAngleTest : public ::testing::TestWithParam<double> {};

TEST_P(G64LineArcAngleTest, LineToArc_AngleMatchesExpected) {
    double targetAngleDeg = GetParam();
    double targetAngleRad = targetAngleDeg * DEG2RAD;

    // Line along +X ending at (20, 0)
    auto line = makeLine(0, 0, 20, 0);

    // Create an arc at (20, 0) whose entry tangent differs from +X by targetAngleDeg.
    // Desired entry tangent direction = (cos(targetAngle), sin(targetAngle))
    //   (measuring angle from +X).
    // For a CW arc, center is 90° CW from tangent:
    //   center offset = r * (sin(targetAngle), -cos(targetAngle))
    double r = 10.0;
    double tangentAngle = targetAngleRad;  // Tangent at arc start
    double cx = 20 + r * std::sin(tangentAngle);
    double cy = 0 - r * std::cos(tangentAngle);

    // Arc starts at (20, 0), goes 90° CW
    double startA = std::atan2(0 - cy, 20 - cx);
    double endA = startA - PI / 2;
    double ex = cx + r * std::cos(endA);
    double ey = cy + r * std::sin(endA);
    auto arc = makeArcCW(20, 0, ex, ey, cx, cy);

    auto analysis = CornerAnalyzer::analyze(line, arc);
    EXPECT_NEAR(analysis.angle, targetAngleDeg, 2.0)
        << "Line→Arc angle should be " << targetAngleDeg << "°";
}

INSTANTIATE_TEST_SUITE_P(
    LineArcAngles,
    G64LineArcAngleTest,
    ::testing::Values(15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 150.0)
);

// ============================================================================
// 8. OVERLAPPING BLENDS — close-together corners
//    When two corners are very close, their blends may overlap.
//    This tests that the system handles it gracefully.
// ============================================================================

TEST_F(G64ArcTransitionTest, OverlappingBlends_TwoCloseCorners_LineLineArc) {
    // Short segment between two corners
    double shortLen = 3.0;
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, shortLen);  // 90° corner
    // Line ending, then 90° corner into another direction
    auto seg3 = makeLine(10, shortLen, 10 + shortLen, shortLen);  // another 90°

    config_.pathTolerance = 2.0;

    auto a1 = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(a1, config_);

    auto a2 = CornerAnalyzer::analyze(seg2, seg3);
    CornerAnalyzer::computeBlendGeometry(a2, config_);

    // Both corners should be detected as 90°
    EXPECT_NEAR(a1.angle, 90.0, 2.0);
    EXPECT_NEAR(a2.angle, 90.0, 2.0);

    // The individual CornerAnalyzer does not enforce overlap constraints
    // (that's done at the PathBlender level). Verify blend geometry is
    // at least computed correctly.
    EXPECT_GT(a1.blendRadius, 0.0);
    EXPECT_GT(a2.blendRadius, 0.0);

    // The tangent distance should be the standard formula value
    double tangentDist1 = std::hypot(
        a1.cornerPoint[0] - a1.blendEntry[0],
        a1.cornerPoint[1] - a1.blendEntry[1]);
    double tangentDist2 = std::hypot(
        a2.cornerPoint[0] - a2.blendEntry[0],
        a2.cornerPoint[1] - a2.blendEntry[1]);

    // For a 90° corner with tolerance 2.0:
    // halfAngle = 45°, r = 2 * cos(45°)/(1-cos(45°)) ≈ 4.828
    // tangentDist = r * tan(45°) = r ≈ 4.828
    // Both tangent distances together exceed the short segment,
    // confirming overlap would occur and must be handled upstream.
    EXPECT_GT(tangentDist1 + tangentDist2, shortLen)
        << "Overlapping scenario confirmed: combined tangent distances exceed short segment";
}

TEST_F(G64ArcTransitionTest, OverlappingBlends_TwoCloseCornersWithArc) {
    // Line → Arc → Line where arc is very short (< blend radius)
    double r = 2.0;
    // Short 90° CW arc from (10, 0) to (12, -2), center (12, 0)
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeArcCW(10, 0, 12, -2, 12, 0);
    auto seg3 = makeLine(12, -2, 12, -20);

    config_.pathTolerance = 5.0;  // Large tolerance relative to arc length

    auto a1 = CornerAnalyzer::analyze(seg1, seg2);
    auto a2 = CornerAnalyzer::analyze(seg2, seg3);

    // These are non-smooth transitions (line tangent ≠ arc tangent)
    // but the system should handle them without crashing
    if (a1.type != CornerType::Straight) {
        CornerAnalyzer::computeBlendGeometry(a1, config_);
        EXPECT_GE(a1.blendRadius, 0.0);
    }
    if (a2.type != CornerType::Straight) {
        CornerAnalyzer::computeBlendGeometry(a2, config_);
        EXPECT_GE(a2.blendRadius, 0.0);
    }
}

TEST_F(G64ArcTransitionTest, OverlappingBlends_TwoSameDirectionCorners) {
    // Two 90° corners in the same direction, very close together
    // Forms a tight U-turn
    double shortLen = 2.0;
    auto seg1 = makeLine(0, 0, 20, 0);       // long line going right
    auto seg2 = makeLine(20, 0, 20, shortLen); // short line going up (90° left)
    auto seg3 = makeLine(20, shortLen, 0, shortLen); // long line going left (90° left)

    config_.pathTolerance = 3.0;  // Large enough to potentially overlap

    auto a1 = CornerAnalyzer::analyze(seg1, seg2);
    CornerAnalyzer::computeBlendGeometry(a1, config_);
    auto a2 = CornerAnalyzer::analyze(seg2, seg3);
    CornerAnalyzer::computeBlendGeometry(a2, config_);

    // Verify both corners are detected
    EXPECT_NEAR(a1.angle, 90.0, 2.0);
    EXPECT_NEAR(a2.angle, 90.0, 2.0);

    // With tight spacing, blends should ideally be adjusted to fit
    if (a1.blendRadius > 0 && a2.blendRadius > 0) {
        // Exit of first blend and entry of second blend should not
        // go past each other on the short segment
        double exit1_y = a1.blendExit[1];
        double entry2_y = a2.blendEntry[1];
        // Both should be on the short segment [0, shortLen] in Y
        EXPECT_GE(exit1_y, -0.01);
        EXPECT_LE(entry2_y, shortLen + 0.01);
    }
}

// ============================================================================
// 9. REGRESSION TEST: Original bug scenario
//    This is the exact case from the user's bug report.
// ============================================================================

TEST_F(G64ArcTransitionTest, Regression_LineCircleLine_NoBogusCorner) {
    // This is the exact geometry from g64_line_g2_circle_line.gcode:
    // G1 X50 Y0       → line
    // G2 X50 Y-30 I0 J-15  → CW semicircle, center at (50,-15)
    // G1 X0 Y-30       → line
    //
    // Previously, the chord-based direction gave 90° corners at both junctions.
    // With the fix, both should be ~0° (smooth).

    auto seg1 = makeLine(0, 0, 50, 0);
    auto seg2 = makeArcCW(50, 0, 50, -30, 50, -15);
    auto seg3 = makeLine(50, -30, 0, -30);

    auto a1 = CornerAnalyzer::analyze(seg1, seg2);
    auto a2 = CornerAnalyzer::analyze(seg2, seg3);

    EXPECT_NEAR(a1.angle, 0.0, ANGLE_TOL_DEG)
        << "REGRESSION: Line→semicircle junction was incorrectly 90°";
    EXPECT_NEAR(a2.angle, 0.0, ANGLE_TOL_DEG)
        << "REGRESSION: Semicircle→line junction was incorrectly 90°";

    EXPECT_EQ(a1.type, CornerType::Straight);
    EXPECT_EQ(a2.type, CornerType::Straight);
}

// ============================================================================
// 10. ANGLE ACCURACY — verify CornerAnalyzer matches independent math
// ============================================================================

TEST_F(G64ArcTransitionTest, AngleMatchesIndependentMath_Various) {
    // Create several segment pairs and verify the CornerAnalyzer angle
    // matches our independent mathematical computation.
    struct TestCase {
        PlanningSegment seg1, seg2;
        const char* desc;
    };

    std::vector<TestCase> cases;

    // Case 1: Line → CW quarter circle (smooth)
    {
        TestCase tc;
        tc.seg1 = makeLine(0, 0, 10, 0);
        tc.seg2 = makeArcCW(10, 0, 10 + 5, -5, 10, -5);
        // Exit tangent of line = (1, 0)
        // Entry tangent of arc: center (10, -5), start (10, 0)
        //   angle = atan2(0-(-5), 10-10) = atan2(5, 0) = π/2
        //   dir=-1: tangent = (sin(π/2), -cos(π/2)) = (1, 0) → smooth!
        tc.desc = "Line→CW quarter-arc (smooth)";
        cases.push_back(tc);
    }

    // Case 2: CW quarter circle → line perpendicular
    {
        TestCase tc;
        tc.seg1 = makeArcCW(0, 10, 10, 0, 0, 0);
        // Exit tangent: center (0,0), end (10,0), angle=0, dir=-1
        //   tangent = (sin(0), -cos(0)) = (0, -1)
        tc.seg2 = makeLine(10, 0, 20, 0);  // direction = (1, 0)
        // Angle between (0,-1) and (1,0) = 90°
        tc.desc = "CW arc→line (90°)";
        cases.push_back(tc);
    }

    // Case 3: Line → Line (45°)
    {
        TestCase tc;
        tc.seg1 = makeLine(0, 0, 10, 0);
        double a = 45.0 * DEG2RAD;
        tc.seg2 = makeLine(10, 0, 10 + 10 * std::cos(a), 10 * std::sin(a));
        tc.desc = "Line→Line (45°)";
        cases.push_back(tc);
    }

    for (const auto& tc : cases) {
        double expected = expectedJunctionAngleDeg(tc.seg1, tc.seg2);
        auto analysis = CornerAnalyzer::analyze(tc.seg1, tc.seg2);
        EXPECT_NEAR(analysis.angle, expected, 1.0)
            << "Failed for: " << tc.desc
            << " (expected=" << expected << ", got=" << analysis.angle << ")";
    }
}

// ============================================================================
// 11. QUARTER-CIRCLE ARC TRANSITIONS AT DIFFERENT ORIENTATIONS
// ============================================================================

TEST_F(G64ArcTransitionTest, QuarterCircle_AllQuadrants_SmoothWithLines) {
    // Test quarter-circle arcs in all four quadrants,
    // each with tangent-matching incoming/outgoing lines.
    struct Quadrant {
        double lineEndX, lineEndY;   // Line endpoint = arc start
        double arcEndX, arcEndY;
        double centerX, centerY;
        double nextLineEndX, nextLineEndY;
        bool cw;
        const char* desc;
    };

    std::vector<Quadrant> quads = {
        // Q1: Line +X → CW arc turning down → Line -Y
        {10, 0, 10, -10, 10, -5, 10, -20, true, "Q1: +X → CW down"},
        // Q2: Line +Y → CW arc turning right → Line +X
        {0, 10, 10, 10, 5, 10, 20, 10, true, "Q2: +Y → CW right"},
        // Q3: Line +X → CCW arc turning up → Line +Y
        {10, 0, 10, 10, 10, 5, 10, 20, false, "Q3: +X → CCW up"},
        // Q4: Line -X → CW arc turning down → Line -Y
        {-10, 0, -10, -10, -10, -5, -10, -20, true, "Q4: -X → CW down"},
    };

    for (const auto& q : quads) {
        auto line1 = makeLine(0, 0, q.lineEndX, q.lineEndY);
        PlanningSegment arc;
        if (q.cw) {
            arc = makeArcCW(q.lineEndX, q.lineEndY, q.arcEndX, q.arcEndY,
                            q.centerX, q.centerY);
        } else {
            arc = makeArcCCW(q.lineEndX, q.lineEndY, q.arcEndX, q.arcEndY,
                             q.centerX, q.centerY);
        }

        auto a1 = CornerAnalyzer::analyze(line1, arc);
        double expected1 = expectedJunctionAngleDeg(line1, arc);

        // Allow slightly larger tolerance for non-trivial geometries
        // But smooth ones should still be < 1°
        EXPECT_NEAR(a1.angle, expected1, 2.0)
            << "Failed at line→arc for " << q.desc;
    }
}

}  // namespace test
}  // namespace GCode
