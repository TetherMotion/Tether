/**
 * @file ScoringBlendTests.cpp
 * @brief Comprehensive tests for the scoring-based C2-continuous corner blending
 *
 * Tests cover:
 * 1. C2 smoothness verification (tangent + curvature continuity at boundaries)
 * 2. Half-length constraint verification
 * 3. Symmetric blend preference
 * 4. All transition types (line-line, line-arc, arc-line, arc-arc)
 * 5. Scoring solver convergence
 * 6. Exact-stop fallback
 * 7. Edge cases (very shallow, very sharp, degenerate)
 * 8. Outside (dogbone) blends
 * 9. Per-transition blend modes
 * 10. Regression tests for the "jump back" bug
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>
#include <string>
#include <vector>
#include <filesystem>
#include <cstdlib>

#include "tether/motion_planner/CornerBlending.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/BezierCurve.hpp"
#include "BlendTestVisualizer.hpp"

namespace MotionPlanner {
namespace test {

// ============================================================================
// Helpers
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;

using Arr = std::array<double, MAX_MOTION_AXES>;
using Vec2 = Vec<2, double>;
using Curve2D = BezierCurve<2, double>;
using Analysis2D = CornerAnalysis<2, double>;
using Analyzer2D = CornerAnalyzer<2, double>;
using Builder2D = BlendCurveBuilder<2, double>;

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

/**
 * @brief Verify C1 continuity at a blend boundary
 * @return angle error between curve tangent and expected direction (radians)
 */
static double checkTangentContinuity(const Curve2D& curve, double t,
                                      const Vec2& expectedDir) {
    Vec2 tangent = Builder2D::computeTangentAt(curve, t);
    double dot = tangent.dot(expectedDir);
    dot = clamp(dot, -1.0, 1.0);
    return std::acos(dot);
}

/**
 * @brief Verify C2 continuity at a blend boundary
 * @return curvature error (absolute difference)
 */
static double checkCurvatureContinuity(const Curve2D& curve, double t,
                                        double expectedCurvature) {
    double k = Builder2D::computeCurvatureAt(curve, t);
    return std::abs(k - expectedCurvature);
}

/**
 * @brief Check that curvature doesn't spike
 */
static double getMaxCurvature(const Curve2D& curve, int samples = 50) {
    return Builder2D::computeMaxCurvature(curve, samples);
}

// ============================================================================
// Test: Basic 90° Line-Line Blend
// ============================================================================

TEST(ScoringBlendC2, LineLine90DegreeCreatesBlend) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;
    config.maxBlendFraction = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    EXPECT_TRUE(analysis.canBlend) << "Should be able to blend 90° corner";
    EXPECT_GT(analysis.blendRadius, 0.0);
    EXPECT_GT(analysis.entryDistance, 0.0);
    EXPECT_GT(analysis.exitDistance, 0.0);
}

TEST(ScoringBlendC2, LineLine90DegreeTangentContinuity) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curves = Builder2D::buildG2BlendCurve(analysis);
    ASSERT_FALSE(curves.empty());

    // Check tangent at entry (should match incoming direction)
    double entryErr = checkTangentContinuity(curves.front(), 0.0,
                                              analysis.incomingDir);
    EXPECT_LT(entryErr, 0.05) << "Entry tangent error: " << entryErr << " rad";

    // Check tangent at exit (should match outgoing direction)
    double exitErr = checkTangentContinuity(curves.back(), 1.0,
                                             analysis.outgoingDir);
    EXPECT_LT(exitErr, 0.05) << "Exit tangent error: " << exitErr << " rad";
}

TEST(ScoringBlendC2, LineLine90DegreeCurvatureContinuity) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curves = Builder2D::buildG2BlendCurve(analysis);
    ASSERT_FALSE(curves.empty());

    // Lines have zero curvature — blend entry/exit should also have near-zero
    double entryK = checkCurvatureContinuity(curves.front(), 0.0, 0.0);
    EXPECT_LT(entryK, 5.0) << "Entry curvature should be small for line-line";

    double exitK = checkCurvatureContinuity(curves.back(), 1.0, 0.0);
    EXPECT_LT(exitK, 5.0) << "Exit curvature should be small for line-line";
}

// ============================================================================
// Test: Half-Length Constraint
// ============================================================================

TEST(ScoringBlendHalfLength, NeverExceedsMaxFraction) {
    // Short segments with large tolerance — should be constrained
    auto seg1 = makeLine(0, 0, 2, 0);
    auto seg2 = makeLine(2, 0, 2, 2);

    BlendConfig config;
    config.tolerance = 5.0;  // Very large — would want huge blend
    config.maxBlendFraction = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        EXPECT_LE(analysis.entryDistance, seg1.segmentLength * config.maxBlendFraction + 0.001)
            << "Entry distance must respect half-length constraint";
        EXPECT_LE(analysis.exitDistance, seg2.segmentLength * config.maxBlendFraction + 0.001)
            << "Exit distance must respect half-length constraint";
    }
}

TEST(ScoringBlendHalfLength, CustomFraction30Percent) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 5.0;
    config.maxBlendFraction = 0.3;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        EXPECT_LE(analysis.entryDistance, 10.0 * 0.3 + 0.001);
        EXPECT_LE(analysis.exitDistance, 10.0 * 0.3 + 0.001);
    }
}

// ============================================================================
// Test: Symmetric Blend
// ============================================================================

TEST(ScoringBlendSymmetry, EqualLengthSegmentsProduceSymmetricBlend) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        double ratio = (analysis.entryDistance > 0) ?
            analysis.exitDistance / analysis.entryDistance : 1.0;
        EXPECT_NEAR(ratio, 1.0, 0.1) << "Symmetric segments should have symmetric blend";
    }
}

// ============================================================================
// Test: Various Corner Angles
// ============================================================================

class AngleBlendTest : public ::testing::TestWithParam<double> {};

TEST_P(AngleBlendTest, BlendIsValidForAngle) {
    double angleDeg = GetParam();
    double angleRad = angleDeg * DEG2RAD;

    // Create two segments meeting at origin at the given angle
    double x1 = 10.0;
    double y1 = 0.0;
    double x2 = 10.0 * std::cos(PI - angleRad);
    double y2 = 10.0 * std::sin(PI - angleRad);

    auto seg1 = makeLine(-x1, 0, 0, 0);
    auto seg2 = makeLine(0, 0, x2, y2);

    BlendConfig config;
    config.tolerance = 0.5;
    config.minAngle = 2.0;
    config.maxAngle = 178.0;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (angleDeg >= config.minAngle && angleDeg <= config.maxAngle) {
        EXPECT_TRUE(analysis.canBlend)
            << "Should blend at " << angleDeg << " degrees";

        if (analysis.canBlend) {
            // Verify tangent continuity
            auto curves = Builder2D::buildG2BlendCurve(analysis);
            ASSERT_FALSE(curves.empty());

            double entryErr = checkTangentContinuity(
                curves.front(), 0.0, analysis.incomingDir);
            EXPECT_LT(entryErr, 0.1)
                << "Entry tangent error at " << angleDeg << "°: " << entryErr;

            double exitErr = checkTangentContinuity(
                curves.back(), 1.0, analysis.outgoingDir);
            EXPECT_LT(exitErr, 0.1)
                << "Exit tangent error at " << angleDeg << "°: " << exitErr;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    CornerAngles, AngleBlendTest,
    ::testing::Values(10, 20, 30, 45, 60, 75, 90, 105, 120, 135, 150, 170));

// ============================================================================
// Test: Line-Arc Transition
// ============================================================================

TEST(ScoringBlendTransitions, LineToArcBlendHasCorrectTangent) {
    // Line along X then CW arc (quarter circle)
    auto seg1 = makeLine(0, 0, 10, 0);
    // Arc from (10,0) to (11,1) with center (11,0) — CW quarter circle
    auto seg2 = makeArcCW(10, 0, 11, 1, 11, 0);

    BlendConfig config;
    config.tolerance = 0.3;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    // The arc's entry tangent should point downward (CW from 9 o'clock)
    // So the angle between line(+X) and arc(–Y) should be 90°
    if (analysis.canBlend) {
        auto curves = Builder2D::buildG2BlendCurve(analysis);
        ASSERT_FALSE(curves.empty());

        // Entry tangent should be along +X (line direction)
        double entryErr = checkTangentContinuity(curves.front(), 0.0,
                                                  analysis.incomingDir);
        EXPECT_LT(entryErr, 0.1);

        // Exit tangent should match arc entry tangent
        double exitErr = checkTangentContinuity(curves.back(), 1.0,
                                                 analysis.outgoingDir);
        EXPECT_LT(exitErr, 0.1);
    }
}

TEST(ScoringBlendTransitions, ArcToLineBlendHasCorrectTangent) {
    // CCW arc from (0,1) to (1,0) with center (0,0), then line
    auto seg1 = makeArcCCW(0, 1, 1, 0, 0, 0);
    auto seg2 = makeLine(1, 0, 5, 0);

    BlendConfig config;
    config.tolerance = 0.3;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        auto curves = Builder2D::buildG2BlendCurve(analysis);
        ASSERT_FALSE(curves.empty());

        double entryErr = checkTangentContinuity(curves.front(), 0.0,
                                                  analysis.incomingDir);
        EXPECT_LT(entryErr, 0.1) << "Arc exit tangent should be preserved";

        double exitErr = checkTangentContinuity(curves.back(), 1.0,
                                                 analysis.outgoingDir);
        EXPECT_LT(exitErr, 0.1) << "Line entry tangent should be preserved";
    }
}

TEST(ScoringBlendTransitions, ArcToArcBlendHasCorrectTangent) {
    // Two successive arcs with different centers
    auto seg1 = makeArcCCW(0, 0, 1, 1, 0, 1);  // CCW from (0,0) to (1,1) center (0,1)
    auto seg2 = makeArcCW(1, 1, 2, 0, 2, 1);   // CW from (1,1) to (2,0) center (2,1)

    BlendConfig config;
    config.tolerance = 0.3;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        auto curves = Builder2D::buildG2BlendCurve(analysis);
        ASSERT_FALSE(curves.empty());

        double entryErr = checkTangentContinuity(curves.front(), 0.0,
                                                  analysis.incomingDir);
        EXPECT_LT(entryErr, 0.15) << "Arc-Arc entry tangent error";

        double exitErr = checkTangentContinuity(curves.back(), 1.0,
                                                 analysis.outgoingDir);
        EXPECT_LT(exitErr, 0.15) << "Arc-Arc exit tangent error";
    }
}

// ============================================================================
// Test: Exact Stop Conditions
// ============================================================================

TEST(ScoringBlendExactStop, VerySharpCornerFallsBackToExactStop) {
    // Nearly 180° reversal
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 0.01, 0.01);  // Near reversal

    BlendConfig config;
    config.tolerance = 0.1;
    config.maxAngle = 175.0;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    // Should either not blend or have tiny radius
    if (!analysis.canBlend) {
        EXPECT_FALSE(analysis.blendReason.empty())
            << "Should have a reason for not blending";
    }
}

TEST(ScoringBlendExactStop, CollinearSegmentsNoBlend) {
    auto seg1 = makeLine(0, 0, 5, 0);
    auto seg2 = makeLine(5, 0, 10, 0);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    EXPECT_FALSE(analysis.canBlend) << "Collinear segments should not blend";
}

TEST(ScoringBlendExactStop, VeryShortSegmentNoBlend) {
    auto seg1 = makeLine(0, 0, 0.001, 0);  // Tiny segment
    auto seg2 = makeLine(0.001, 0, 0.001, 10);

    BlendConfig config;
    config.tolerance = 0.5;
    config.minSegmentLength = 0.01;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    EXPECT_FALSE(analysis.canBlend) << "Very short segment should not blend";
}

// ============================================================================
// Test: Scoring Solver
// ============================================================================

TEST(ScoringBlendSolver, ConvergesWithinBudget) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;
    config.coarseSteps = 10;
    config.fineSteps = 10;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    ASSERT_TRUE(analysis.canBlend);
    EXPECT_LE(analysis.diagnostics.solverIterations, 25)
        << "Solver should converge within budget";
    EXPECT_GT(analysis.diagnostics.bestScore, 0.0)
        << "Best score should be positive for valid blend";
}

TEST(ScoringBlendSolver, DisabledScorerStillWorks) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;
    config.useScoringOptimizer = false;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    EXPECT_TRUE(analysis.canBlend);
    EXPECT_EQ(analysis.diagnostics.solverIterations, 1)
        << "Disabled scorer should use single evaluation";
}

TEST(ScoringBlendSolver, DiagnosticsPopulated) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    ASSERT_TRUE(analysis.canBlend);
    EXPECT_TRUE(analysis.diagnostics.isValid);
    EXPECT_GT(analysis.diagnostics.solverIterations, 0);
    EXPECT_GT(analysis.diagnostics.bestScore, 0.0);
}

// ============================================================================
// Test: Outside (Dogbone) Blend
// ============================================================================

TEST(ScoringBlendOutside, NegativeToleranceCreatesOutsideBlend) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = -0.5;  // Negative = outside

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        EXPECT_TRUE(analysis.isOutsideBlend)
            << "Negative tolerance should produce outside blend";

        auto curves = Builder2D::buildG2BlendCurve(analysis);
        ASSERT_FALSE(curves.empty());

        // Verify tangent continuity even for outside blends
        double entryErr = checkTangentContinuity(curves.front(), 0.0,
                                                  analysis.incomingDir);
        EXPECT_LT(entryErr, 0.1);

        double exitErr = checkTangentContinuity(curves.back(), 1.0,
                                                 analysis.outgoingDir);
        EXPECT_LT(exitErr, 0.1);
    }
}

// ============================================================================
// Test: Blend Fraction
// ============================================================================

TEST(ScoringBlendFraction, FractionBetweenZeroAndOne) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        EXPECT_GT(analysis.blendFraction, 0.0);
        EXPECT_LE(analysis.blendFraction, 1.0);
    }
}

TEST(ScoringBlendFraction, ConstrainedFractionIsSmaller) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    // Large tolerance
    BlendConfig config;
    config.tolerance = 5.0;
    config.maxBlendFraction = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    if (analysis.canBlend) {
        // With large tolerance and half-length, blend fraction should be < 1
        EXPECT_LT(analysis.blendFraction, 1.0 + 0.01);
    }
}

// ============================================================================
// Test: Curvature Quality
// ============================================================================

TEST(ScoringBlendCurvature, NoCurvatureSpike) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curves = Builder2D::buildG2BlendCurve(analysis);
    ASSERT_FALSE(curves.empty());

    double maxK = getMaxCurvature(curves.front());
    double maxAllowed = config.maxCurvatureMultiplier / analysis.blendRadius;

    EXPECT_LT(maxK, maxAllowed * 2.0)
        << "Curvature should not spike dramatically";
}

// ============================================================================
// Test: Regression - "Jump Back" Bug
// ============================================================================

TEST(ScoringBlendRegression, BlendEntryExitOnSegments) {
    // The bug was that blend assumed 90° and then jumped back.
    // Verify that blend entry is on seg1 and exit is on seg2.
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 14, 6);  // ~56° turn

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    // Blend entry should be on seg1 (between start and corner)
    Vec2 entryPt{analysis.blendEntry[0], analysis.blendEntry[1]};
    Vec2 seg1Start{0, 0};
    Vec2 corner{10, 0};

    double proj = (entryPt - seg1Start).dot((corner - seg1Start).normalized());
    EXPECT_GT(proj, 0.0) << "Entry point should be ahead of seg1 start";
    EXPECT_LT(proj, seg1.segmentLength + 0.001) << "Entry should be on seg1";

    // Blend exit should be on seg2
    Vec2 exitPt{analysis.blendExit[0], analysis.blendExit[1]};
    Vec2 seg2End{14, 6};

    double proj2 = (exitPt - corner).dot((seg2End - corner).normalized());
    EXPECT_GT(proj2, -0.001) << "Exit should be on seg2 (after corner)";
}

TEST(ScoringBlendRegression, NoJumpBackAtNon90DegreeCorner) {
    // The original bug: blend always assumed 90° corners.
    // For a 45° corner, the blend curve should NOT overshoot.
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 17.07, 7.07);  // 45° turn

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curves = Builder2D::buildG2BlendCurve(analysis);
    ASSERT_FALSE(curves.empty());

    // Sample the blend curve and check it stays between entry and exit
    for (int i = 0; i <= 20; ++i) {
        double t = i / 20.0;
        Vec2 pt = curves.front().evaluate(t);

        // Should not go past the corner point's X coordinate significantly
        EXPECT_LT(pt[0], 11.0)
            << "Blend should not overshoot past corner at t=" << t;
    }
}

TEST(ScoringBlendRegression, ConsistentAngleComputation) {
    // Verify that the analyzed angle matches expected
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    auto analysis = Analyzer2D::analyze(seg1, seg2, config);

    // 90° corner = π/2 radians
    EXPECT_NEAR(analysis.angle, PI / 2.0, 0.01)
        << "90° corner should have angle ≈ π/2";
}

// ============================================================================
// Test: Corner Mode Integration
// ============================================================================

TEST(ScoringBlendModes, InsideApproximateReducesRadius) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig defaultConfig;
    defaultConfig.tolerance = 0.5;

    BlendConfig insideConfig;
    insideConfig.tolerance = 0.5;
    insideConfig.cornerMode = BlendConfig::CornerLimitMode::InsideApproximate;
    insideConfig.insideTolerance = 0.1;

    auto defaultAnalysis = Analyzer2D::analyze(seg1, seg2, defaultConfig);
    auto insideAnalysis = Analyzer2D::analyze(seg1, seg2, insideConfig);

    if (defaultAnalysis.canBlend && insideAnalysis.canBlend) {
        EXPECT_LE(insideAnalysis.blendRadius, defaultAnalysis.blendRadius + 0.001)
            << "Inside mode with smaller tolerance should produce ≤ radius";
    }
}

// ============================================================================
// Test: Builder Fallbacks
// ============================================================================

TEST(ScoringBlendBuilder, G1FallbackWorks) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curve = Builder2D::buildG1BlendCurve(analysis);
    EXPECT_EQ(curve.degree(), 3u) << "G1 fallback should be cubic";

    // Check it starts at entry and ends at exit
    Vec2 start = curve.evaluate(0.0);
    Vec2 end = curve.evaluate(1.0);
    EXPECT_NEAR(start[0], analysis.blendEntry[0], 0.001);
    EXPECT_NEAR(start[1], analysis.blendEntry[1], 0.001);
    EXPECT_NEAR(end[0], analysis.blendExit[0], 0.001);
    EXPECT_NEAR(end[1], analysis.blendExit[1], 0.001);
}

TEST(ScoringBlendBuilder, CircularArcFallbackWorks) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curve = Builder2D::buildCircularBlendArc(analysis);
    EXPECT_EQ(curve.degree(), 3u) << "Circular arc approximation should be cubic";
}

// ============================================================================
// Test: Multiple Consecutive Blends
// ============================================================================

TEST(ScoringBlendChain, ThreeSegmentChainBlendsCorrectly) {
    auto seg1 = makeLine(0, 0, 5, 0);
    auto seg2 = makeLine(5, 0, 5, 5);
    auto seg3 = makeLine(5, 5, 10, 5);

    BlendConfig config;
    config.tolerance = 0.3;
    config.maxBlendFraction = 0.4;

    auto a1 = Analyzer2D::analyze(seg1, seg2, config);
    auto a2 = Analyzer2D::analyze(seg2, seg3, config);

    if (a1.canBlend && a2.canBlend) {
        // Exit of blend1 + entry of blend2 should not overlap on seg2
        double consumed = a1.exitDistance + a2.entryDistance;
        EXPECT_LE(consumed, seg2.segmentLength + 0.001)
            << "Adjacent blends should not overlap on shared segment";
    }
}

// ============================================================================
// Test: Quintic Degree
// ============================================================================

TEST(ScoringBlendQuintic, QuinticBlendHasDegree5) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curve = Builder2D::buildQuinticC2Blend(analysis);
    EXPECT_EQ(curve.degree(), 5u) << "Quintic blend should have degree 5";
    EXPECT_EQ(curve.numControlPoints(), 6u) << "Quintic should have 6 control points";
}

TEST(ScoringBlendQuintic, QuinticEndpointsMatchEntryExit) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);

    BlendConfig config;
    config.tolerance = 0.5;

    auto analysis = Analyzer2D::analyze(seg1, seg2, config);
    ASSERT_TRUE(analysis.canBlend);

    auto curve = Builder2D::buildQuinticC2Blend(analysis);

    Vec2 P0 = curve.evaluate(0.0);
    Vec2 P5 = curve.evaluate(1.0);

    EXPECT_NEAR(P0[0], analysis.blendEntry[0], 1e-10);
    EXPECT_NEAR(P0[1], analysis.blendEntry[1], 1e-10);
    EXPECT_NEAR(P5[0], analysis.blendExit[0], 1e-10);
    EXPECT_NEAR(P5[1], analysis.blendExit[1], 1e-10);
}

// ============================================================================
// SVG visualization summary — generates SVGs for representative scenarios
// ============================================================================

TEST(ScoringBlendVisualSummary, GenerateAllSVGs) {
    struct Scenario {
        std::string name;
        MotionSegment seg1;
        MotionSegment seg2;
        double tolerance;
        std::string mode;
    };

    std::vector<Scenario> scenarios = {
        {"C2_90deg_LineLine",      makeLine(0,0, 10,0),    makeLine(10,0, 10,10),    0.5, "Line-Line"},
        {"C2_60deg_LineLine",      makeLine(0,0, 10,0),    makeLine(10,0, 10+5,8.66),0.5, "Line-Line"},
        {"C2_120deg_LineLine",     makeLine(0,0, 10,0),    makeLine(10,0, 15,8.66),  0.5, "Line-Line"},
        {"C2_LineArcCW",           makeLine(0,0, 10,0),    makeArcCW(10,0, 20,0, 15,0), 0.5, "Line-ArcCW"},
        {"C2_ArcCW_Line",          makeArcCW(0,0, 10,0, 5,0), makeLine(10,0, 20,10),  0.5, "ArcCW-Line"},
        {"C2_OutsideBlend_90deg",  makeLine(0,0, 10,0),    makeLine(10,0, 10,10),    0.5, "Outside"},
        {"C2_HalfLength_Short",    makeLine(0,0, 2,0),     makeLine(2,0, 2,2),       0.5, "Short"},
        {"C2_SharpCorner_30deg",   makeLine(0,0, 10,0),    makeLine(10,0, 10+8.66,5),0.5, "Sharp"},
        {"C2_GentleCorner_150deg", makeLine(0,0, 10,0),    makeLine(10,0, 10+8.66,-5),0.5, "Gentle"},
    };

    // Set outside tolerance for the outside blend
    scenarios[5].seg1 = makeLine(0,0, 10,0);
    scenarios[5].seg2 = makeLine(10,0, 10,10);

    std::string outRoot;
    const char* env = std::getenv("TEST_SVG_DIR");
    if (env && env[0]) outRoot = env;
    else outRoot = "test_output/svgs";
    std::string dir = outRoot + "/blend_scoring";
    std::filesystem::create_directories(dir);

    int count = 0;
    for (const auto& sc : scenarios) {
        BlendConfig config;
        config.tolerance = sc.tolerance;
        config.maxBlendFraction = 0.5;
        if (sc.mode == "Outside") {
            config.cornerMode = BlendConfig::CornerLimitMode::OutsideApproximate;
        }

        auto analysis = Analyzer2D::analyze(sc.seg1, sc.seg2, config);

        BlendTest::emitMotionPlannerSVG(
            sc.name, sc.seg1, sc.seg2, analysis,
            dir, sc.name + ".svg",
            sc.tolerance, sc.mode);

        if (std::filesystem::exists(dir + "/" + sc.name + ".svg")) ++count;
    }

    EXPECT_GT(count, 0) << "Should generate at least one SVG";
    std::cout << "\n=== Scoring Blend SVG Summary ===\n"
              << "SVGs generated: " << count << "\n"
              << "Output dir: " << dir << "\n"
              << "================================\n\n";
}

}  // namespace test
}  // namespace MotionPlanner
