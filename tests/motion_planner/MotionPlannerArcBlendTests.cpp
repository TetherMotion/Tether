/**
 * @file MotionPlannerArcBlendTests.cpp
 * @brief Tests for MotionPlanner::CornerAnalyzer arc tangent handling
 *
 * Validates that the MotionPlanner-level CornerAnalyzer also computes proper
 * tangent directions for arc segments, not just chord directions.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

#include "tether/motion_planner/CornerBlending.hpp"
#include "tether/motion_planner/MotionSegment.hpp"

namespace MotionPlanner {
namespace test {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double ANGLE_TOL_RAD = 0.02;  // ~1 degree in radians

using Arr = std::array<double, MAX_MOTION_AXES>;

static Arr makePos(double x, double y, double z = 0) {
    Arr a{};
    a[0] = x; a[1] = y; a[2] = z;
    return a;
}

static MotionSegment makeLine(double x0, double y0, double x1, double y1) {
    return MotionSegment::linear(makePos(x0, y0), makePos(x1, y1), 1000.0);
}

static MotionSegment makeArcCW(double sx, double sy,
                                double ex, double ey,
                                double cx, double cy) {
    return MotionSegment::arcCW(makePos(sx, sy), makePos(ex, ey),
                                makePos(cx, cy), 1000.0, ArcPlane::XY);
}

static MotionSegment makeArcCCW(double sx, double sy,
                                 double ex, double ey,
                                 double cx, double cy) {
    return MotionSegment::arcCCW(makePos(sx, sy), makePos(ex, ey),
                                 makePos(cx, cy), 1000.0, ArcPlane::XY);
}

// ============================================================================
// Smooth Transition Tests — MotionPlanner CornerAnalyzer
// ============================================================================

TEST(MotionPlannerArcBlend, SmoothLineToArcCW) {
    auto line = makeLine(0, 0, 50, 0);
    auto arc  = makeArcCW(50, 0, 50, -30, 50, -15);

    auto analysis = CornerAnalyzer2D::analyze(line, arc);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_LT(angleDeg, 2.0)
        << "Smooth line→CW arc should have near-zero angle, got " << angleDeg;
}

TEST(MotionPlannerArcBlend, SmoothArcCWToLine) {
    auto arc  = makeArcCW(50, 0, 50, -30, 50, -15);
    auto line = makeLine(50, -30, 0, -30);

    auto analysis = CornerAnalyzer2D::analyze(arc, line);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_LT(angleDeg, 2.0)
        << "Smooth CW arc→line should have near-zero angle, got " << angleDeg;
}

TEST(MotionPlannerArcBlend, SmoothLineToArcCCW) {
    auto line = makeLine(0, 0, 50, 0);
    auto arc  = makeArcCCW(50, 0, 50, 30, 50, 15);

    auto analysis = CornerAnalyzer2D::analyze(line, arc);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_LT(angleDeg, 2.0);
}

TEST(MotionPlannerArcBlend, SmoothArcToArc_SameCenter) {
    double r = 20.0;
    auto arc1 = makeArcCCW(r, 0, 0, r, 0, 0);
    auto arc2 = makeArcCCW(0, r, -r, 0, 0, 0);

    auto analysis = CornerAnalyzer2D::analyze(arc1, arc2);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_LT(angleDeg, 2.0)
        << "Two CCW arcs with same center should be smooth";
}

TEST(MotionPlannerArcBlend, Regression_LineCircleLine_NoBogusCorner) {
    auto seg1 = makeLine(0, 0, 50, 0);
    auto seg2 = makeArcCW(50, 0, 50, -30, 50, -15);
    auto seg3 = makeLine(50, -30, 0, -30);

    auto a1 = CornerAnalyzer2D::analyze(seg1, seg2);
    auto a2 = CornerAnalyzer2D::analyze(seg2, seg3);

    double angle1Deg = a1.angle * 180.0 / PI;
    double angle2Deg = a2.angle * 180.0 / PI;

    EXPECT_LT(angle1Deg, 2.0)
        << "REGRESSION: Line→semicircle was incorrectly 90°, got " << angle1Deg;
    EXPECT_LT(angle2Deg, 2.0)
        << "REGRESSION: Semicircle→line was incorrectly 90°, got " << angle2Deg;
}

// ============================================================================
// Non-smooth Transition Tests
// ============================================================================

TEST(MotionPlannerArcBlend, LineToArc_90Degree) {
    // Line along +X, arc with tangent going downward at junction
    // CW arc with center at (10 + r, 0), r = 10
    // At start (10, 0): radius angle = atan2(0, 10-20) = atan2(0, -10) = π
    // tangent = (-(-1)*sin(π), (-1)*cos(π)) = (-sin(π), cos(π)) = (0, -1)
    // line tangent = (1, 0). Angle = 90°
    auto line = makeLine(0, 0, 10, 0);
    auto arc  = makeArcCW(10, 0, 20, -10, 20, 0);

    auto analysis = CornerAnalyzer2D::analyze(line, arc);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_NEAR(angleDeg, 90.0, 3.0);
}

TEST(MotionPlannerArcBlend, ArcToLine_90Degree) {
    // CW quarter-circle from (0, 10) to (10, 0), center (0, 0)
    // Exit tangent at (10, 0): angle=0, dir=-1 → (sin(0), -cos(0)) = (0, -1)
    // Line from (10, 0) to (20, 0): direction = (1, 0)
    // Angle between (0,-1) and (1,0) = 90°
    auto arc  = makeArcCW(0, 10, 10, 0, 0, 0);
    auto line = makeLine(10, 0, 20, 0);

    auto analysis = CornerAnalyzer2D::analyze(arc, line);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_NEAR(angleDeg, 90.0, 3.0);
}

TEST(MotionPlannerArcBlend, LineLineFallback_NotBroken) {
    // Verify line-line analysis still works correctly
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    auto analysis = CornerAnalyzer2D::analyze(seg1, seg2);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_NEAR(angleDeg, 90.0, 1.0);
}

// ============================================================================
// 3D variants
// ============================================================================

TEST(MotionPlannerArcBlend, SmoothLineToArcCW_3D) {
    auto line = MotionSegment::linear(makePos(0, 0, 0), makePos(50, 0, 0), 1000.0);
    auto arc  = MotionSegment::arcCW(makePos(50, 0, 0), makePos(50, -30, 0),
                                      makePos(50, -15, 0), 1000.0, ArcPlane::XY);

    auto analysis = CornerAnalyzer3D::analyze(line, arc);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_LT(angleDeg, 2.0)
        << "3D: Smooth line→CW arc should have near-zero angle";
}

TEST(MotionPlannerArcBlend, SmoothArcCWToLine_3D) {
    auto arc  = MotionSegment::arcCW(makePos(50, 0, 0), makePos(50, -30, 0),
                                      makePos(50, -15, 0), 1000.0, ArcPlane::XY);
    auto line = MotionSegment::linear(makePos(50, -30, 0), makePos(0, -30, 0), 1000.0);

    auto analysis = CornerAnalyzer3D::analyze(arc, line);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_LT(angleDeg, 2.0)
        << "3D: Smooth CW arc→line should have near-zero angle";
}

// ============================================================================
// Various angles — MotionPlanner
// ============================================================================

class MotionPlannerArcAngleTest : public ::testing::TestWithParam<double> {};

TEST_P(MotionPlannerArcAngleTest, LineToArc_DetectsCorrectAngle) {
    double targetDeg = GetParam();
    double targetRad = targetDeg * DEG2RAD;

    // Line along +X ending at (20, 0)
    auto line = makeLine(0, 0, 20, 0);

    // Create CW arc at (20, 0) whose entry tangent differs from +X by targetDeg
    double r = 10.0;
    double cx = 20 + r * std::sin(targetRad);
    double cy = 0 - r * std::cos(targetRad);

    double startA = std::atan2(0 - cy, 20 - cx);
    double endA = startA - PI / 2;
    double ex = cx + r * std::cos(endA);
    double ey = cy + r * std::sin(endA);
    auto arc = makeArcCW(20, 0, ex, ey, cx, cy);

    auto analysis = CornerAnalyzer2D::analyze(line, arc);
    double angleDeg = analysis.angle * 180.0 / PI;
    EXPECT_NEAR(angleDeg, targetDeg, 3.0)
        << "Expected " << targetDeg << "°, got " << angleDeg << "°";
}

INSTANTIATE_TEST_SUITE_P(
    MPArcAngles,
    MotionPlannerArcAngleTest,
    ::testing::Values(15.0, 30.0, 45.0, 60.0, 90.0, 120.0)
);

}  // namespace test
}  // namespace MotionPlanner
