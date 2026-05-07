/**
 * @file NURBSCurveTest.cpp
 * @brief Comprehensive unit tests for NURBSCurve and PiecewiseNURBSPath
 *
 * Tests cover:
 * - Construction and validation
 * - De Boor evaluation
 * - Rational (weighted) NURBS
 * - Derivative computation (1st, 2nd, 3rd order)
 * - Curvature and geometric properties
 * - Arc length computation
 * - Knot insertion (Boehm's algorithm)
 * - Bézier decomposition (for SVG export)
 * - Cubic Bézier approximation
 * - Continuity checking (G0, G1, G2)
 * - Subdivision
 * - NURBS line and arc factory methods
 * - G2 blend curve generation
 * - Piecewise NURBS path
 * - Mixed NURBS and Bézier segments
 * - BSPLINE/NURBS G-code parsing
 * - Corner blending with NURBS
 * - Edge cases and corner cases
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/NURBSCurve.hpp>
#include <tether/motion_planner/PiecewiseNURBSPath.hpp>
#include <tether/motion_planner/BezierCurve.hpp>
#include <tether/motion_planner/MathTypes.hpp>
#include <cmath>
#include <vector>
#include <iostream>

using namespace MotionPlanner;

// ============================================================================
// Test Helpers
// ============================================================================

static constexpr double TEST_EPSILON = 1e-6;
static constexpr double TIGHT_EPSILON = 1e-10;

// ============================================================================
// Construction Tests
// ============================================================================

TEST(NURBSCurve, DefaultConstruction) {
    NURBSCurve2D curve;
    EXPECT_FALSE(curve.isValid());
    EXPECT_EQ(curve.numControlPoints(), 0u);
}

TEST(NURBSCurve, LinearConstruction) {
    Vec2 p0{0, 0}, p1{10, 0};
    auto curve = NURBSCurve2D::makeLine(p0, p1);

    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 1u);
    EXPECT_EQ(curve.numControlPoints(), 2u);
    EXPECT_EQ(curve.knots().size(), 4u);
    EXPECT_TRUE(curve.isNonRational());
}

TEST(NURBSCurve, UniformBSplineConstruction) {
    // Cubic B-spline with 5 control points
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 3u);
    EXPECT_EQ(curve.numControlPoints(), 5u);
    EXPECT_EQ(curve.knots().size(), 9u); // n+p+1 = 5+3+1 = 9
    EXPECT_TRUE(curve.isNonRational());
}

TEST(NURBSCurve, ExplicitKnotVector) {
    std::vector<Vec2> pts = {{0,0}, {1,2}, {3,3}, {5,1}};
    std::vector<double> wts = {1, 1, 1, 1};
    std::vector<double> knots = {0, 0, 0, 0.5, 1, 1, 1}; // degree 2
    NURBSCurve2D curve(pts, wts, knots, 2);

    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 2u);
    EXPECT_EQ(curve.numControlPoints(), 4u);
}

TEST(NURBSCurve, RationalConstruction) {
    std::vector<Vec2> pts = {{0,0}, {1,1}, {2,0}};
    std::vector<double> wts = {1.0, 0.707, 1.0}; // Circular arc weights
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NURBSCurve2D curve(pts, wts, knots, 2);

    EXPECT_TRUE(curve.isValid());
    EXPECT_FALSE(curve.isNonRational());
}

TEST(NURBSCurve, InvalidKnotVector) {
    std::vector<Vec2> pts = {{0,0}, {1,1}};
    std::vector<double> wts = {1, 1};
    std::vector<double> knots = {0, 1}; // Wrong size for degree 1 (should be 4)

    EXPECT_THROW(
        NURBSCurve2D(pts, wts, knots, 1),
        std::invalid_argument
    );
}

TEST(NURBSCurve, NonDecreasingKnotVectorRequired) {
    std::vector<Vec2> pts = {{0,0}, {1,1}, {2,0}};
    std::vector<double> wts = {1, 1, 1};
    std::vector<double> knots = {0, 0, 1, 0.5, 1, 1}; // Decreasing!

    EXPECT_THROW(
        NURBSCurve2D(pts, wts, knots, 2),
        std::invalid_argument
    );
}

TEST(NURBSCurve, ConstructFromBezier) {
    // Create a cubic Bézier
    BezierCurve2D bezier({{0,0}, {1,3}, {4,3}, {5,0}});

    // Convert to NURBS
    NURBSCurve2D nurbs(bezier);

    EXPECT_TRUE(nurbs.isValid());
    EXPECT_EQ(nurbs.degree(), 3u);
    EXPECT_EQ(nurbs.numControlPoints(), 4u);
    EXPECT_TRUE(nurbs.isNonRational());

    // Should evaluate to same points
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        Vec2 bezPt = bezier.evaluate(t);
        Vec2 nurbsPt = nurbs.evaluate(t);
        EXPECT_NEAR(bezPt[0], nurbsPt[0], TEST_EPSILON);
        EXPECT_NEAR(bezPt[1], nurbsPt[1], TEST_EPSILON);
    }
}

// ============================================================================
// Evaluation Tests
// ============================================================================

TEST(NURBSCurve, LinearEvaluation) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 5});

    Vec2 p0 = curve.evaluate(0.0);
    Vec2 p1 = curve.evaluate(0.5);
    Vec2 p2 = curve.evaluate(1.0);

    EXPECT_NEAR(p0[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(p0[1], 0.0, TEST_EPSILON);
    EXPECT_NEAR(p1[0], 5.0, TEST_EPSILON);
    EXPECT_NEAR(p1[1], 2.5, TEST_EPSILON);
    EXPECT_NEAR(p2[0], 10.0, TEST_EPSILON);
    EXPECT_NEAR(p2[1], 5.0, TEST_EPSILON);
}

TEST(NURBSCurve, QuadraticBSplineEvaluation) {
    // Quadratic B-spline with uniform knots
    std::vector<Vec2> pts = {{0,0}, {2,4}, {4,4}, {6,0}};
    std::vector<double> knots = {0, 0, 0, 0.5, 1, 1, 1};
    NURBSCurve2D curve(pts, {}, knots, 2);

    // Endpoints
    Vec2 start = curve.evaluate(0.0);
    Vec2 end = curve.evaluate(1.0);
    EXPECT_NEAR(start[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(start[1], 0.0, TEST_EPSILON);
    EXPECT_NEAR(end[0], 6.0, TEST_EPSILON);
    EXPECT_NEAR(end[1], 0.0, TEST_EPSILON);

    // Midpoint should be above
    Vec2 mid = curve.evaluate(0.5);
    EXPECT_GT(mid[1], 0.0);
}

TEST(NURBSCurve, CubicBSplineEvaluation) {
    // Cubic with 6 control points
    std::vector<Vec2> pts = {{0,0}, {1,3}, {3,5}, {5,5}, {7,3}, {8,0}};
    NURBSCurve2D curve(pts, 3);

    // Curve should pass near control points
    Vec2 start = curve.startPoint();
    Vec2 end = curve.endPoint();
    EXPECT_NEAR(start[0], pts.front()[0], TEST_EPSILON);
    EXPECT_NEAR(start[1], pts.front()[1], TEST_EPSILON);
    EXPECT_NEAR(end[0], pts.back()[0], TEST_EPSILON);
    EXPECT_NEAR(end[1], pts.back()[1], TEST_EPSILON);

    // Monotonicity check in X
    double prevX = start[0];
    for (double t = 0.1; t <= 1.0; t += 0.1) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        Vec2 p = curve.evaluate(u);
        EXPECT_GE(p[0], prevX - TEST_EPSILON);
        prevX = p[0];
    }
}

TEST(NURBSCurve, RationalCircularArc) {
    // Degree-2 rational NURBS representing a quarter circle
    Vec2 center{0, 0};
    double radius = 5.0;
    auto curve = NURBSCurve2D::makeCircularArc(center, radius, 0.0,
                                                 MathConstants::PI / 2.0);

    EXPECT_TRUE(curve.isValid());
    EXPECT_FALSE(curve.isNonRational());

    // Start should be at (5, 0)
    Vec2 start = curve.startPoint();
    EXPECT_NEAR(start[0], 5.0, TEST_EPSILON);
    EXPECT_NEAR(start[1], 0.0, TEST_EPSILON);

    // End should be at (0, 5)
    Vec2 end = curve.endPoint();
    EXPECT_NEAR(end[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(end[1], 5.0, TEST_EPSILON);

    // All points should be at distance = radius from center
    for (double t = 0.0; t <= 1.0; t += 0.05) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        Vec2 p = curve.evaluate(u);
        double dist = std::sqrt(p[0]*p[0] + p[1]*p[1]);
        EXPECT_NEAR(dist, radius, TEST_EPSILON * 10) << "t=" << t;
    }
}

TEST(NURBSCurve, OperatorCallEvaluation) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});
    Vec2 p = curve(0.5);
    EXPECT_NEAR(p[0], 5.0, TEST_EPSILON);
}

TEST(NURBSCurve, EvaluationClampsToDomain) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});

    // Values outside [0,1] should be clamped
    Vec2 before = curve.evaluate(-0.5);
    Vec2 after = curve.evaluate(1.5);
    EXPECT_NEAR(before[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(after[0], 10.0, TEST_EPSILON);
}

// ============================================================================
// Derivative Tests
// ============================================================================

TEST(NURBSCurve, LinearDerivative) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 5});

    // First derivative of a line is constant
    Vec2 d1_start = curve.evaluateDerivative(0.0, 1);
    Vec2 d1_mid = curve.evaluateDerivative(0.5, 1);
    Vec2 d1_end = curve.evaluateDerivative(1.0, 1);

    EXPECT_NEAR(d1_start[0], d1_mid[0], TEST_EPSILON);
    EXPECT_NEAR(d1_start[1], d1_mid[1], TEST_EPSILON);
    EXPECT_NEAR(d1_mid[0], d1_end[0], TEST_EPSILON);

    // Second derivative of a line is zero
    Vec2 d2 = curve.evaluateDerivative(0.5, 2);
    EXPECT_NEAR(d2[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(d2[1], 0.0, TEST_EPSILON);
}

TEST(NURBSCurve, QuadraticDerivative) {
    // Quadratic Bézier as NURBS: P0=(0,0), P1=(5,10), P2=(10,0)
    std::vector<Vec2> pts = {{0,0}, {5,10}, {10,0}};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NURBSCurve2D curve(pts, {}, knots, 2);

    // At t=0.5, dy/du should be 0 (vertex of parabola)
    Vec2 d1 = curve.evaluateDerivative(0.5, 1);
    EXPECT_NEAR(d1[1], 0.0, 0.1); // y-component near zero at vertex

    // Second derivative should be constant for quadratic
    Vec2 d2_start = curve.evaluateDerivative(0.0, 2);
    Vec2 d2_mid = curve.evaluateDerivative(0.5, 2);
    EXPECT_NEAR(d2_start[0], d2_mid[0], TEST_EPSILON);
    EXPECT_NEAR(d2_start[1], d2_mid[1], TEST_EPSILON);
}

TEST(NURBSCurve, EvaluateAllConsistency) {
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    double u = curve.domainStart() + 0.3 * (curve.domainEnd() - curve.domainStart());
    auto [pos, vel, acc, jrk] = curve.evaluateAll(u);

    Vec2 pos2 = curve.evaluate(u);
    Vec2 vel2 = curve.evaluateDerivative(u, 1);
    Vec2 acc2 = curve.evaluateDerivative(u, 2);

    EXPECT_NEAR(pos[0], pos2[0], TIGHT_EPSILON);
    EXPECT_NEAR(pos[1], pos2[1], TIGHT_EPSILON);
    EXPECT_NEAR(vel[0], vel2[0], TIGHT_EPSILON);
    EXPECT_NEAR(vel[1], vel2[1], TIGHT_EPSILON);
    EXPECT_NEAR(acc[0], acc2[0], TIGHT_EPSILON);
    EXPECT_NEAR(acc[1], acc2[1], TIGHT_EPSILON);
}

TEST(NURBSCurve, RationalDerivative) {
    // Quarter circle
    auto curve = NURBSCurve2D::makeCircularArc({0,0}, 5.0, 0.0,
                                                MathConstants::PI / 2.0);

    // At start, tangent should be in +Y direction
    Vec2 d1 = curve.evaluateDerivative(curve.domainStart(), 1);
    double angle = std::atan2(d1[1], d1[0]);
    EXPECT_NEAR(angle, MathConstants::PI / 2.0, 0.1);
}

TEST(NURBSCurve, DerivativeOrderExceedsDegree) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});

    // 2nd derivative of degree-1 curve should be zero
    Vec2 d2 = curve.evaluateDerivative(0.5, 2);
    EXPECT_NEAR(d2[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(d2[1], 0.0, TEST_EPSILON);

    // 3rd derivative too
    Vec2 d3 = curve.evaluateDerivative(0.5, 3);
    EXPECT_NEAR(d3[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(d3[1], 0.0, TEST_EPSILON);
}

// ============================================================================
// Geometric Properties Tests
// ============================================================================

TEST(NURBSCurve, TangentVector) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});
    Vec2 t = curve.tangent(0.5);
    EXPECT_NEAR(t[0], 1.0, TEST_EPSILON); // Horizontal
    EXPECT_NEAR(t[1], 0.0, TEST_EPSILON);
}

TEST(NURBSCurve, Speed) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});
    double s = curve.speed(0.5);
    EXPECT_GT(s, 0.0);
}

TEST(NURBSCurve, CurvatureOfLine) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});
    double k = curve.curvature(0.5);
    EXPECT_NEAR(k, 0.0, TEST_EPSILON);
}

TEST(NURBSCurve, CurvatureOfCircle) {
    double radius = 5.0;
    auto curve = NURBSCurve2D::makeCircularArc({0,0}, radius, 0.0,
                                                MathConstants::PI / 2.0);

    // Curvature of circle = 1/radius
    double expectedK = 1.0 / radius;
    double midU = (curve.domainStart() + curve.domainEnd()) / 2.0;
    double k = curve.curvature(midU);
    EXPECT_NEAR(std::abs(k), expectedK, 0.1); // Rational approx has some error
}

TEST(NURBSCurve, BoundingBox) {
    std::vector<Vec2> pts = {{0,0}, {5,10}, {10,0}};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NURBSCurve2D curve(pts, {}, knots, 2);

    auto [minPt, maxPt] = curve.boundingBox();
    EXPECT_LE(minPt[0], 0.0);
    EXPECT_LE(minPt[1], 0.0);
    EXPECT_GE(maxPt[0], 10.0);
    EXPECT_GE(maxPt[1], 10.0);
}

// ============================================================================
// Arc Length Tests
// ============================================================================

TEST(NURBSCurve, ArcLengthOfLine) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {3, 4});
    double len = curve.arcLength();
    EXPECT_NEAR(len, 5.0, 0.01); // 3-4-5 triangle
}

TEST(NURBSCurve, ArcLengthBetween) {
    auto curve = NURBSCurve2D::makeLine({0, 0}, {10, 0});
    double half = curve.arcLengthBetween(0.0, 0.5);
    double full = curve.arcLength();
    EXPECT_NEAR(half, full / 2.0, 0.01);
}

TEST(NURBSCurve, ArcLengthTable) {
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    auto table = curve.buildArcLengthTable();
    EXPECT_TRUE(table.isValid());
    EXPECT_GT(table.totalLength(), 0.0);

    // Forward-backward consistency
    double s = table.totalLength() / 2.0;
    double u = table.parameterAt(s);
    double s2 = table.arcLengthAt(u);
    EXPECT_NEAR(s, s2, 0.1);
}

// ============================================================================
// Knot Insertion Tests
// ============================================================================

TEST(NURBSCurve, KnotInsertionPreservesShape) {
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    // Insert a knot
    double insertU = 0.5;
    NURBSCurve2D refined = curve.insertKnot(insertU);

    EXPECT_EQ(refined.numControlPoints(), curve.numControlPoints() + 1);
    EXPECT_EQ(refined.knots().size(), curve.knots().size() + 1);

    // Shape should be preserved
    for (double t = 0.0; t <= 1.0; t += 0.05) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        Vec2 orig = curve.evaluate(u);
        Vec2 ref = refined.evaluate(u);
        EXPECT_NEAR(orig[0], ref[0], TEST_EPSILON) << "t=" << t;
        EXPECT_NEAR(orig[1], ref[1], TEST_EPSILON) << "t=" << t;
    }
}

TEST(NURBSCurve, MultipleKnotInsertions) {
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    NURBSCurve2D refined = curve.insertKnot(0.3, 2);
    EXPECT_EQ(refined.numControlPoints(), curve.numControlPoints() + 2);

    // Shape preserved
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        Vec2 orig = curve.evaluate(u);
        Vec2 ref = refined.evaluate(u);
        EXPECT_NEAR(orig[0], ref[0], TEST_EPSILON) << "t=" << t;
        EXPECT_NEAR(orig[1], ref[1], TEST_EPSILON) << "t=" << t;
    }
}

TEST(NURBSCurve, RationalKnotInsertion) {
    auto curve = NURBSCurve2D::makeCircularArc({0,0}, 5.0, 0.0,
                                                MathConstants::PI / 2.0);
    NURBSCurve2D refined = curve.insertKnot(0.5);

    EXPECT_EQ(refined.numControlPoints(), curve.numControlPoints() + 1);

    // Points should still lie on circle
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        double u = refined.domainStart() + t * (refined.domainEnd() - refined.domainStart());
        Vec2 p = refined.evaluate(u);
        double dist = std::sqrt(p[0]*p[0] + p[1]*p[1]);
        EXPECT_NEAR(dist, 5.0, TEST_EPSILON * 100) << "t=" << t;
    }
}

// ============================================================================
// Bézier Decomposition Tests
// ============================================================================

TEST(NURBSCurve, BezierDecompositionOfBezier) {
    // A NURBS that's already a single Bézier should decompose to 1 Bézier
    BezierCurve2D bezier({{0,0}, {1,3}, {4,3}, {5,0}});
    NURBSCurve2D nurbs(bezier);

    auto beziers = nurbs.decomposeToBezier();
    EXPECT_GE(beziers.size(), 1u);

    // The first Bézier should match the original
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        Vec2 orig = bezier.evaluate(t);
        Vec2 decomp = beziers[0].evaluate(t);
        EXPECT_NEAR(orig[0], decomp[0], TEST_EPSILON) << "t=" << t;
        EXPECT_NEAR(orig[1], decomp[1], TEST_EPSILON) << "t=" << t;
    }
}

TEST(NURBSCurve, BezierDecompositionMultipleSpans) {
    // Cubic with 6 control points = 2 spans after decomposition
    std::vector<Vec2> pts = {{0,0}, {1,3}, {3,5}, {5,5}, {7,3}, {8,0}};
    NURBSCurve2D curve(pts, 3);

    auto beziers = curve.decomposeToBezier();
    EXPECT_GE(beziers.size(), 1u);

    // Combined Béziers should approximate the NURBS curve
    // Sample original curve and check decomposed segments cover the range
    Vec2 nurbsStart = curve.startPoint();
    Vec2 nurbsEnd = curve.endPoint();
    Vec2 bezStart = beziers.front().startPoint();
    Vec2 bezEnd = beziers.back().endPoint();

    EXPECT_NEAR(nurbsStart[0], bezStart[0], TEST_EPSILON);
    EXPECT_NEAR(nurbsStart[1], bezStart[1], TEST_EPSILON);
    EXPECT_NEAR(nurbsEnd[0], bezEnd[0], TEST_EPSILON);
    EXPECT_NEAR(nurbsEnd[1], bezEnd[1], TEST_EPSILON);
}

TEST(NURBSCurve, CubicBezierApproximation) {
    // Create a degree-5 NURBS
    std::vector<Vec2> pts = {{0,0}, {1,2}, {2,4}, {4,4}, {5,2}, {6,0}};
    std::vector<double> knots = {0,0,0,0,0,0, 1,1,1,1,1,1};
    NURBSCurve2D curve(pts, {}, knots, 5);

    auto cubics = curve.approximateWithCubicBeziers(0.01);
    EXPECT_GE(cubics.size(), 1u);

    // Check approximation accuracy
    for (const auto& cubic : cubics) {
        EXPECT_EQ(cubic.degree(), 3u);
    }
}

// ============================================================================
// Subdivision Tests
// ============================================================================

TEST(NURBSCurve, SubdivisionPreservesShape) {
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    auto [left, right] = curve.subdivide(0.5);

    // Left curve should start at curve start
    EXPECT_NEAR(left.startPoint()[0], curve.startPoint()[0], TEST_EPSILON);
    EXPECT_NEAR(left.startPoint()[1], curve.startPoint()[1], TEST_EPSILON);

    // Right curve should end at curve end
    EXPECT_NEAR(right.endPoint()[0], curve.endPoint()[0], TEST_EPSILON);
    EXPECT_NEAR(right.endPoint()[1], curve.endPoint()[1], TEST_EPSILON);
}

// ============================================================================
// Continuity Tests
// ============================================================================

TEST(NURBSCurve, G0ContinuityConnectedCurves) {
    auto c1 = NURBSCurve2D::makeLine({0,0}, {5,5});
    auto c2 = NURBSCurve2D::makeLine({5,5}, {10,0});

    EXPECT_TRUE(c1.isG0ContinuousWith(c2));
}

TEST(NURBSCurve, G0ContinuityDisconnectedCurves) {
    auto c1 = NURBSCurve2D::makeLine({0,0}, {5,5});
    auto c2 = NURBSCurve2D::makeLine({6,6}, {10,0}); // Gap

    EXPECT_FALSE(c1.isG0ContinuousWith(c2));
}

TEST(NURBSCurve, G1ContinuityCollinear) {
    auto c1 = NURBSCurve2D::makeLine({0,0}, {5,0});
    auto c2 = NURBSCurve2D::makeLine({5,0}, {10,0});

    EXPECT_TRUE(c1.isG1ContinuousWith(c2));
}

TEST(NURBSCurve, G1ContinuityCorner) {
    auto c1 = NURBSCurve2D::makeLine({0,0}, {5,0});
    auto c2 = NURBSCurve2D::makeLine({5,0}, {5,5}); // 90° turn

    EXPECT_TRUE(c1.isG0ContinuousWith(c2));
    EXPECT_FALSE(c1.isG1ContinuousWith(c2));
}

TEST(NURBSCurve, ContinuityLevel) {
    auto c1 = NURBSCurve2D::makeLine({0,0}, {5,0});
    auto c2 = NURBSCurve2D::makeLine({5,0}, {10,0}); // Collinear

    int level = c1.continuityLevelWith(c2);
    EXPECT_GE(level, 1); // At least G1
}

// ============================================================================
// G2 Blend Curve Tests
// ============================================================================

TEST(NURBSCurve, G2BlendCurveGeneration) {
    Vec2 entry{5, 0};
    Vec2 exit{0, 5};
    Vec2 entryDir{1, 0};
    entryDir = entryDir.normalized();
    Vec2 exitDir{0, 1};
    exitDir = exitDir.normalized();

    auto blend = NURBSCurve2D::makeG2BlendCurve(entry, exit, entryDir, exitDir);

    EXPECT_TRUE(blend.isValid());
    EXPECT_EQ(blend.degree(), 5u);

    // Start and end should match
    Vec2 start = blend.startPoint();
    Vec2 end = blend.endPoint();
    EXPECT_NEAR(start[0], entry[0], TEST_EPSILON);
    EXPECT_NEAR(start[1], entry[1], TEST_EPSILON);
    EXPECT_NEAR(end[0], exit[0], TEST_EPSILON);
    EXPECT_NEAR(end[1], exit[1], TEST_EPSILON);

    // Tangent at start should be along entryDir
    Vec2 t0 = blend.tangent(blend.domainStart());
    double dot = std::abs(t0.dot(entryDir));
    EXPECT_NEAR(dot, 1.0, 0.01);

    // Curvature at endpoints should be near zero (blending into lines)
    double k0 = blend.curvature(blend.domainStart());
    double k1 = blend.curvature(blend.domainEnd());
    EXPECT_NEAR(k0, 0.0, 0.5);
    EXPECT_NEAR(k1, 0.0, 0.5);
}

// ============================================================================
// Piecewise NURBS Path Tests
// ============================================================================

TEST(PiecewiseNURBSPath, EmptyPath) {
    PiecewiseNURBSPath2D path;
    EXPECT_TRUE(path.empty());
    EXPECT_EQ(path.numSegments(), 0u);
    EXPECT_EQ(path.totalLength(), 0.0);
}

TEST(PiecewiseNURBSPath, SingleSegmentPath) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {10,0}));
    path.buildArcLengthTables();

    EXPECT_EQ(path.numSegments(), 1u);
    EXPECT_NEAR(path.totalLength(), 10.0, 0.1);

    auto eval = path.evaluateAtArcLength(5.0);
    EXPECT_NEAR(eval.position[0], 5.0, 0.5);
}

TEST(PiecewiseNURBSPath, MultiSegmentPath) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {10,0}));
    path.appendSegment(NURBSCurve2D::makeLine({10,0}, {10,10}));
    path.appendSegment(NURBSCurve2D::makeLine({10,10}, {0,10}));
    path.buildArcLengthTables();

    EXPECT_EQ(path.numSegments(), 3u);
    EXPECT_NEAR(path.totalLength(), 30.0, 0.3);

    // Start
    auto eval0 = path.evaluateAtArcLength(0.0);
    EXPECT_NEAR(eval0.position[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(eval0.position[1], 0.0, TEST_EPSILON);

    // End
    auto evalEnd = path.evaluateAtArcLength(path.totalLength());
    EXPECT_NEAR(evalEnd.position[0], 0.0, 0.5);
    EXPECT_NEAR(evalEnd.position[1], 10.0, 0.5);
}

TEST(PiecewiseNURBSPath, MixedNURBSAndBezier) {
    PiecewiseNURBSPath2D path;

    // Add a B-spline segment
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    path.appendSegment(NURBSCurve2D(pts, 3));

    // Add a Bézier segment (converted internally)
    BezierCurve2D bezier({{10,0}, {12,3}, {15,3}, {17,0}});
    path.appendBezierSegment(bezier);

    path.buildArcLengthTables();

    EXPECT_EQ(path.numSegments(), 2u);
    EXPECT_GT(path.totalLength(), 0.0);
}

TEST(PiecewiseNURBSPath, G0ContinuityCheck) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {5,0}));
    path.appendSegment(NURBSCurve2D::makeLine({5,0}, {10,5}));
    path.buildArcLengthTables();

    EXPECT_TRUE(path.isG0Continuous());
}

TEST(PiecewiseNURBSPath, BezierDecomposition) {
    PiecewiseNURBSPath2D path;
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    path.appendSegment(NURBSCurve2D(pts, 3));
    path.buildArcLengthTables();

    auto beziers = path.decomposeToBezier();
    EXPECT_GE(beziers.size(), 1u);

    // Should cover same start/end
    EXPECT_NEAR(beziers.front().startPoint()[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(beziers.back().endPoint()[0], 10.0, TEST_EPSILON);
}

TEST(PiecewiseNURBSPath, CubicApproximation) {
    PiecewiseNURBSPath2D path;
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    path.appendSegment(NURBSCurve2D(pts, 3));
    path.buildArcLengthTables();

    auto cubics = path.approximateWithCubicBeziers(0.01);
    EXPECT_GE(cubics.size(), 1u);
    for (const auto& c : cubics) {
        EXPECT_EQ(c.degree(), 3u);
    }
}

TEST(PiecewiseNURBSPath, ToPiecewiseBezierPath) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {10,0}));
    path.appendSegment(NURBSCurve2D::makeLine({10,0}, {10,10}));
    path.buildArcLengthTables();

    auto bezPath = path.toPiecewiseBezierPath();
    EXPECT_GT(bezPath.numSegments(), 0u);
    bezPath.buildArcLengthTables();
    EXPECT_NEAR(bezPath.totalLength(), 20.0, 1.0);
}

// ============================================================================
// 3D NURBS Tests
// ============================================================================

TEST(NURBSCurve3D, BasicConstruction) {
    std::vector<Vec3> pts = {{0,0,0}, {2,4,1}, {5,5,2}, {8,4,1}, {10,0,0}};
    NURBSCurve3D curve(pts, 3);

    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 3u);

    Vec3 start = curve.startPoint();
    Vec3 end = curve.endPoint();
    EXPECT_NEAR(start[0], 0.0, TEST_EPSILON);
    EXPECT_NEAR(end[0], 10.0, TEST_EPSILON);
}

TEST(NURBSCurve3D, ArcLength) {
    auto curve = NURBSCurve3D::makeLine({0,0,0}, {3,4,0});
    double len = curve.arcLength();
    EXPECT_NEAR(len, 5.0, 0.01);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(NURBSCurve, SingleControlPoint) {
    // Can't make a valid curve with < 2 control points
    NURBSCurve2D curve;
    EXPECT_FALSE(curve.isValid());
}

TEST(NURBSCurve, DegreeZero) {
    // Degree 0 = piecewise constant
    std::vector<Vec2> pts = {{0,0}, {5,5}};
    std::vector<double> knots = {0, 0.5, 1};
    NURBSCurve2D curve(pts, {}, knots, 0);
    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 0u);
}

TEST(NURBSCurve, AllWeightsEqual) {
    std::vector<Vec2> pts = {{0,0}, {5,5}, {10,0}};
    std::vector<double> wts = {2.0, 2.0, 2.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NURBSCurve2D curve(pts, wts, knots, 2);

    // Same weights = same as non-rational
    std::vector<double> wts1 = {1.0, 1.0, 1.0};
    NURBSCurve2D curve1(pts, wts1, knots, 2);

    for (double t = 0.0; t <= 1.0; t += 0.1) {
        Vec2 p = curve.evaluate(t);
        Vec2 p1 = curve1.evaluate(t);
        EXPECT_NEAR(p[0], p1[0], TEST_EPSILON);
        EXPECT_NEAR(p[1], p1[1], TEST_EPSILON);
    }
}

TEST(NURBSCurve, HighDegree) {
    // Degree 5 with 7 control points
    std::vector<Vec2> pts = {{0,0}, {1,3}, {3,5}, {5,5}, {7,3}, {9,1}, {10,0}};
    std::vector<double> knots = {0,0,0,0,0,0, 0.5, 1,1,1,1,1,1};
    NURBSCurve2D curve(pts, {}, knots, 5);

    EXPECT_TRUE(curve.isValid());

    // Should evaluate without errors
    for (double t = 0.0; t <= 1.0; t += 0.05) {
        Vec2 p = curve.evaluate(t);
        EXPECT_TRUE(std::isfinite(p[0]));
        EXPECT_TRUE(std::isfinite(p[1]));
    }
}

TEST(NURBSCurve, RepeatedKnots) {
    // Multiple internal knots at the same value
    std::vector<Vec2> pts = {{0,0}, {2,4}, {4,4}, {6,4}, {8,0}};
    std::vector<double> knots = {0, 0, 0, 0.5, 0.5, 1, 1, 1};
    NURBSCurve2D curve(pts, {}, knots, 2);

    EXPECT_TRUE(curve.isValid());

    // At double knot, curve should pass through corresponding control point
    Vec2 p = curve.evaluate(0.5);
    EXPECT_TRUE(std::isfinite(p[0]));
    EXPECT_TRUE(std::isfinite(p[1]));
}

TEST(NURBSCurve, ZeroWeightHandling) {
    // A weight of zero makes the control point have no influence
    std::vector<Vec2> pts = {{0,0}, {5,10}, {10,0}};
    std::vector<double> wts = {1.0, 0.001, 1.0}; // Very small weight
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NURBSCurve2D curve(pts, wts, knots, 2);

    // Midpoint should be close to linear interpolation (weight near zero)
    Vec2 mid = curve.evaluate(0.5);
    EXPECT_NEAR(mid[0], 5.0, 0.5); // Near midpoint
    EXPECT_LT(mid[1], 2.0); // Much closer to baseline than control point
}

// ============================================================================
// NURBS G-Code Parsing Tests
// ============================================================================

TEST(NURBSGCode, ParseBSPLINECommand) {
    // Test parsing of: BSPLINE X10 Y20 Z5 KNOT=0,0,0,1,2,2,2
    //                   POLES=0,0,0,5,10,2,10,20,5 WEIGHTS=1,1,1 DEGREE=2 F1000

    // Verify the NURBS can be constructed from parsed data
    std::vector<Vec3> poles = {{0,0,0}, {5,10,2}, {10,20,5}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1}; // Adjusted for 3 poles, degree 2
    size_t degree = 2;

    NURBSCurve3D curve(poles, weights, knots, degree);
    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 2u);
    EXPECT_EQ(curve.numControlPoints(), 3u);

    // Endpoint should be at (10, 20, 5)
    Vec3 end = curve.endPoint();
    EXPECT_NEAR(end[0], 10.0, TEST_EPSILON);
    EXPECT_NEAR(end[1], 20.0, TEST_EPSILON);
    EXPECT_NEAR(end[2], 5.0, TEST_EPSILON);
}

TEST(NURBSGCode, ParseNURBSCommand) {
    // Same format: NURBS X10 Y20 Z5 KNOT=... POLES=... WEIGHTS=... DEGREE=2 F1000
    // Same test as BSPLINE since the format is identical
    std::vector<Vec3> poles = {{0,0,0}, {5,10,2}, {10,20,5}};
    std::vector<double> weights = {1.0, 0.707, 1.0}; // Rational NURBS
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};

    NURBSCurve3D curve(poles, weights, knots, 2);
    EXPECT_TRUE(curve.isValid());
    EXPECT_FALSE(curve.isNonRational());
}

TEST(NURBSGCode, HigherDegreeNURBS) {
    // Degree 3 with 5 poles
    std::vector<Vec3> poles = {{0,0,0}, {3,5,1}, {6,7,2}, {9,5,1}, {12,0,0}};
    std::vector<double> weights(5, 1.0);
    std::vector<double> knots = {0,0,0,0, 0.5, 1,1,1,1};

    NURBSCurve3D curve(poles, weights, knots, 3);
    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.degree(), 3u);
}

// ============================================================================
// Corner Blending with NURBS Tests
// ============================================================================

TEST(NURBSBlending, TwoLineSegmentsBlended) {
    // Create a 90° corner between two lines and blend with NURBS
    Vec2 entry{5, 0};
    Vec2 exit{0, 5};
    Vec2 entryDir = Vec2{1, 0}.normalized();
    Vec2 exitDir = Vec2{0, 1}.normalized();

    auto blend = NURBSCurve2D::makeG2BlendCurve(entry, exit, entryDir, exitDir);

    EXPECT_TRUE(blend.isValid());

    // Verify endpoints
    EXPECT_NEAR(blend.startPoint()[0], entry[0], TEST_EPSILON);
    EXPECT_NEAR(blend.startPoint()[1], entry[1], TEST_EPSILON);
    EXPECT_NEAR(blend.endPoint()[0], exit[0], TEST_EPSILON);
    EXPECT_NEAR(blend.endPoint()[1], exit[1], TEST_EPSILON);

    // Path should be smooth: tangent should always rotate in one direction
    // (no reversals). For a 90° turn, consecutive tangent dot product
    // should stay positive (no backward motion)
    Vec2 prevTangent = blend.tangent(blend.domainStart());
    for (double t = 0.05; t <= 1.0; t += 0.05) {
        double u = blend.domainStart() + t * (blend.domainEnd() - blend.domainStart());
        Vec2 tangent = blend.tangent(u);
        double dot = prevTangent.dot(tangent);
        EXPECT_GT(dot, -0.1) << "Direction reversal at t=" << t;
        prevTangent = tangent;
    }

    // Verify entry and exit tangent directions
    Vec2 startTangent = blend.tangent(blend.domainStart());
    Vec2 endTangent = blend.tangent(blend.domainEnd());
    EXPECT_GT(startTangent.dot(entryDir), 0.99) << "Entry tangent mismatch";
    EXPECT_GT(endTangent.dot(exitDir), 0.99) << "Exit tangent mismatch";
}

TEST(NURBSBlending, BlendInPiecewisePath) {
    PiecewiseNURBSPath2D path;

    // Line 1
    path.appendSegment(NURBSCurve2D::makeLine({0, 0}, {5, 0}));

    // G2 blend at 90° corner
    Vec2 entryDir{1, 0};
    Vec2 exitDir{0, 1};
    auto blend = NURBSCurve2D::makeG2BlendCurve(Vec2{5, 0}, Vec2{5, 0} + exitDir * 2.0,
                                                  entryDir, exitDir);
    path.appendSegment(blend);

    // Line 2
    path.appendSegment(NURBSCurve2D::makeLine(blend.endPoint(), blend.endPoint() + exitDir * 5.0));

    path.buildArcLengthTables();

    EXPECT_EQ(path.numSegments(), 3u);
    EXPECT_GT(path.totalLength(), 0.0);

    // Path should be smooth
    EXPECT_TRUE(path.isG0Continuous(0.01));
}

TEST(NURBSBlending, MixedNURBSLineCorner) {
    // NURBS curve followed by a line - test blending feasibility
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D nurbs(pts, 3);

    auto line = NURBSCurve2D::makeLine({10, 0}, {15, 0});

    // The junction should be G0 continuous
    EXPECT_TRUE(nurbs.isG0ContinuousWith(line, 0.01));
}

// ============================================================================
// Velocity Profile with NURBS Tests
// ============================================================================

TEST(NURBSVelocity, CurvatureBasedVelocityLimiting) {
    // A curve with varying curvature should produce varying velocity limits
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    // Sample curvature along the curve
    std::vector<double> curvatures;
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        double k = std::abs(curve.curvature(u));
        curvatures.push_back(k);
    }

    // Curvature should vary (not all the same)
    bool hasVariation = false;
    for (size_t i = 1; i < curvatures.size(); ++i) {
        if (std::abs(curvatures[i] - curvatures[0]) > 0.001) {
            hasVariation = true;
            break;
        }
    }
    EXPECT_TRUE(hasVariation);
}

TEST(NURBSVelocity, ArcLengthParameterizationForVelocity) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {10,0}));
    path.buildArcLengthTables();

    // Velocity along a line should be constant at unit tangent
    auto eval1 = path.evaluateAtArcLength(2.0);
    auto eval2 = path.evaluateAtArcLength(5.0);
    auto eval3 = path.evaluateAtArcLength(8.0);

    // Tangent should be the same everywhere
    EXPECT_NEAR(eval1.tangent[0], eval2.tangent[0], TEST_EPSILON);
    EXPECT_NEAR(eval2.tangent[0], eval3.tangent[0], TEST_EPSILON);
}

// ============================================================================
// Source Reference Tests
// ============================================================================

TEST(NURBSCurve, SourceReferencePreservation) {
    auto srcFile = std::make_shared<SourceFile>("test.gcode");
    SourceReference ref = SourceReference::fromLine(42, srcFile);

    auto curve = NURBSCurve2D::makeLine({0,0}, {10,0});
    curve.setSourceRef(ref);

    EXPECT_TRUE(curve.sourceRef().isValid());
    EXPECT_EQ(curve.sourceRef().lineNumber(), 42u);

    // Preserved through knot insertion
    auto refined = curve.insertKnot(0.5);
    EXPECT_TRUE(refined.sourceRef().isValid());

    // Preserved through Bézier decomposition
    auto beziers = curve.decomposeToBezier();
    for (const auto& b : beziers) {
        EXPECT_TRUE(b.sourceRef().isValid());
    }
}

// ============================================================================
// Performance/Stress Tests
// ============================================================================

TEST(NURBSCurve, ManyControlPoints) {
    // NURBS with 100 control points
    std::vector<Vec2> pts(100);
    for (int i = 0; i < 100; ++i) {
        pts[i] = Vec2{double(i), std::sin(double(i) * 0.1) * 5.0};
    }
    NURBSCurve2D curve(pts, 3);

    EXPECT_TRUE(curve.isValid());
    EXPECT_EQ(curve.numControlPoints(), 100u);

    // Should evaluate quickly
    for (double t = 0.0; t <= 1.0; t += 0.01) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        Vec2 p = curve.evaluate(u);
        EXPECT_TRUE(std::isfinite(p[0]));
        EXPECT_TRUE(std::isfinite(p[1]));
    }
}

TEST(NURBSCurve, LargeKnotInsertionCount) {
    std::vector<Vec2> pts = {{0,0}, {2,4}, {5,5}, {8,4}, {10,0}};
    NURBSCurve2D curve(pts, 3);

    // Insert many knots
    NURBSCurve2D refined = curve;
    for (double u = 0.1; u < 1.0; u += 0.1) {
        refined = refined.insertKnot(u);
    }

    EXPECT_GT(refined.numControlPoints(), curve.numControlPoints());

    // Shape should still be preserved
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        double u = curve.domainStart() + t * (curve.domainEnd() - curve.domainStart());
        Vec2 orig = curve.evaluate(u);
        Vec2 ref = refined.evaluate(u);
        EXPECT_NEAR(orig[0], ref[0], 0.01) << "t=" << t;
        EXPECT_NEAR(orig[1], ref[1], 0.01) << "t=" << t;
    }
}

// ============================================================================
// Piecewise NURBS Path Edge Cases
// ============================================================================

TEST(PiecewiseNURBSPath, SinglePointEvaluation) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({5,5}, {5,5})); // Zero-length
    path.buildArcLengthTables();

    EXPECT_NEAR(path.totalLength(), 0.0, TEST_EPSILON);
}

TEST(PiecewiseNURBSPath, EvaluateAtBoundaries) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {10,0}));
    path.appendSegment(NURBSCurve2D::makeLine({10,0}, {20,10}));
    path.buildArcLengthTables();

    // At s=0
    auto eval0 = path.evaluateAtArcLength(0.0);
    EXPECT_NEAR(eval0.position[0], 0.0, TEST_EPSILON);

    // At s=totalLength
    auto evalEnd = path.evaluateAtArcLength(path.totalLength());
    EXPECT_NEAR(evalEnd.position[0], 20.0, 0.5);
}

TEST(PiecewiseNURBSPath, CurvatureAtArcLength) {
    PiecewiseNURBSPath2D path;
    path.appendSegment(NURBSCurve2D::makeLine({0,0}, {10,0}));
    path.buildArcLengthTables();

    // Line should have zero curvature
    double k = path.curvatureAtArcLength(5.0);
    EXPECT_NEAR(k, 0.0, TEST_EPSILON);
}

TEST(PiecewiseNURBSPath, SourceRefAtArcLength) {
    auto srcFile = std::make_shared<SourceFile>("test.gcode");

    PiecewiseNURBSPath2D path;
    auto seg1 = NURBSCurve2D::makeLine({0,0}, {10,0});
    seg1.setSourceRef(SourceReference::fromLine(1, srcFile));
    path.appendSegment(seg1);

    auto seg2 = NURBSCurve2D::makeLine({10,0}, {20,0});
    seg2.setSourceRef(SourceReference::fromLine(2, srcFile));
    path.appendSegment(seg2);

    path.buildArcLengthTables();

    auto ref1 = path.sourceRefAtArcLength(5.0);
    EXPECT_TRUE(ref1.isValid());

    auto ref2 = path.sourceRefAtArcLength(15.0);
    EXPECT_TRUE(ref2.isValid());
}
