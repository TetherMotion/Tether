/**
 * @file BezierCurveTest.cpp
 * @brief Unit tests for BezierCurve.hpp
 *
 * Tests Bézier curve implementation including de Casteljau algorithm,
 * derivatives, arc length, and subdivision.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/BezierCurve.hpp>
#include <cmath>

using namespace MotionPlanner;

// ============================================================================
// Linear Bézier Tests
// ============================================================================

class LinearBezierTest : public ::testing::Test {
protected:
    void SetUp() override {
        start = Vec2{0.0, 0.0};
        end = Vec2{10.0, 0.0};
        curve = createLinearBezier(start, end);
    }
    
    Vec2 start, end;
    BezierCurve2D curve;
};

TEST_F(LinearBezierTest, Degree) {
    EXPECT_EQ(curve.degree(), 1);
}

TEST_F(LinearBezierTest, Endpoints) {
    EXPECT_NEAR(curve.evaluate(0.0)[0], 0.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(0.0)[1], 0.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(1.0)[0], 10.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(1.0)[1], 0.0, 1e-10);
}

TEST_F(LinearBezierTest, Midpoint) {
    auto mid = curve.evaluate(0.5);
    EXPECT_NEAR(mid[0], 5.0, 1e-10);
    EXPECT_NEAR(mid[1], 0.0, 1e-10);
}

TEST_F(LinearBezierTest, Derivative) {
    auto deriv = curve.evaluateDerivative(0.5, 1);
    EXPECT_NEAR(deriv[0], 10.0, 1e-10);  // Constant velocity
    EXPECT_NEAR(deriv[1], 0.0, 1e-10);
}

TEST_F(LinearBezierTest, ArcLength) {
    EXPECT_NEAR(curve.arcLength(), 10.0, 1e-6);
}

TEST_F(LinearBezierTest, Curvature) {
    EXPECT_NEAR(curve.curvature(0.5), 0.0, 1e-10);  // Straight line
}

// ============================================================================
// Quadratic Bézier Tests
// ============================================================================

class QuadraticBezierTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Right angle turn
        p0 = Vec2{0.0, 0.0};
        p1 = Vec2{5.0, 0.0};
        p2 = Vec2{5.0, 5.0};
        curve = createQuadraticBezier(p0, p1, p2);
    }
    
    Vec2 p0, p1, p2;
    BezierCurve2D curve;
};

TEST_F(QuadraticBezierTest, Degree) {
    EXPECT_EQ(curve.degree(), 2);
}

TEST_F(QuadraticBezierTest, Endpoints) {
    EXPECT_NEAR(curve.evaluate(0.0)[0], 0.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(0.0)[1], 0.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(1.0)[0], 5.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(1.0)[1], 5.0, 1e-10);
}

TEST_F(QuadraticBezierTest, Tangents) {
    auto t0 = curve.tangent(0.0);
    EXPECT_NEAR(t0[0], 1.0, 1e-10);  // Horizontal start
    EXPECT_NEAR(t0[1], 0.0, 1e-10);
    
    auto t1 = curve.tangent(1.0);
    EXPECT_NEAR(t1[0], 0.0, 1e-10);  // Vertical end
    EXPECT_NEAR(t1[1], 1.0, 1e-10);
}

TEST_F(QuadraticBezierTest, NonZeroCurvature) {
    // Quadratic Bézier with a bend should have non-zero curvature
    double kappa = curve.curvature(0.5);
    EXPECT_GT(std::abs(kappa), 0.01);
}

// ============================================================================
// Cubic Bézier Tests
// ============================================================================

class CubicBezierTest : public ::testing::Test {
protected:
    void SetUp() override {
        // S-curve
        p0 = Vec2{0.0, 0.0};
        p1 = Vec2{1.0, 1.0};
        p2 = Vec2{2.0, 1.0};
        p3 = Vec2{3.0, 0.0};
        curve = createCubicBezier(p0, p1, p2, p3);
    }
    
    Vec2 p0, p1, p2, p3;
    BezierCurve2D curve;
};

TEST_F(CubicBezierTest, Degree) {
    EXPECT_EQ(curve.degree(), 3);
}

TEST_F(CubicBezierTest, Endpoints) {
    auto start = curve.evaluate(0.0);
    auto end = curve.evaluate(1.0);
    
    EXPECT_NEAR(start[0], p0[0], 1e-10);
    EXPECT_NEAR(start[1], p0[1], 1e-10);
    EXPECT_NEAR(end[0], p3[0], 1e-10);
    EXPECT_NEAR(end[1], p3[1], 1e-10);
}

TEST_F(CubicBezierTest, DerivativeCurve) {
    auto deriv = curve.derivativeCurve();
    EXPECT_EQ(deriv.degree(), 2);  // Derivative of cubic is quadratic
}

TEST_F(CubicBezierTest, Subdivision) {
    auto [left, right] = curve.subdivide(0.5);
    
    // Both halves should have same degree
    EXPECT_EQ(left.degree(), 3);
    EXPECT_EQ(right.degree(), 3);
    
    // Left should end where right starts
    auto leftEnd = left.evaluate(1.0);
    auto rightStart = right.evaluate(0.0);
    EXPECT_NEAR(leftEnd[0], rightStart[0], 1e-10);
    EXPECT_NEAR(leftEnd[1], rightStart[1], 1e-10);
    
    // Both should match original at endpoints
    auto originalStart = curve.evaluate(0.0);
    auto originalMid = curve.evaluate(0.5);
    auto originalEnd = curve.evaluate(1.0);
    
    EXPECT_NEAR(left.evaluate(0.0)[0], originalStart[0], 1e-10);
    EXPECT_NEAR(left.evaluate(1.0)[0], originalMid[0], 1e-10);
    EXPECT_NEAR(right.evaluate(0.0)[0], originalMid[0], 1e-10);
    EXPECT_NEAR(right.evaluate(1.0)[0], originalEnd[0], 1e-10);
}

// ============================================================================
// Quintic Bézier Tests
// ============================================================================

class QuinticBezierTest : public ::testing::Test {
protected:
    void SetUp() override {
        // G2-continuous curve
        std::vector<Vec2> pts = {
            {0.0, 0.0},
            {1.0, 0.0},
            {2.0, 0.5},
            {3.0, 1.5},
            {4.0, 2.0},
            {5.0, 2.0}
        };
        curve = createQuinticBezier(pts[0], pts[1], pts[2], pts[3], pts[4], pts[5]);
    }
    
    BezierCurve2D curve;
};

TEST_F(QuinticBezierTest, Degree) {
    EXPECT_EQ(curve.degree(), 5);
}

TEST_F(QuinticBezierTest, SmoothDerivatives) {
    // Quintic should have smooth derivatives up to second order
    for (double u = 0.1; u < 0.9; u += 0.1) {
        auto d1 = curve.evaluateDerivative(u, 1);
        auto d2 = curve.evaluateDerivative(u, 2);
        
        // Derivatives should be finite
        EXPECT_TRUE(std::isfinite(d1[0]));
        EXPECT_TRUE(std::isfinite(d1[1]));
        EXPECT_TRUE(std::isfinite(d2[0]));
        EXPECT_TRUE(std::isfinite(d2[1]));
    }
}

TEST_F(QuinticBezierTest, ContinuousCurvature) {
    double prevKappa = curve.curvature(0.0);
    for (double u = 0.1; u <= 1.0; u += 0.1) {
        double kappa = curve.curvature(u);
        // Curvature should not have discontinuities
        EXPECT_LT(std::abs(kappa - prevKappa), 10.0);  // Reasonable bound
        prevKappa = kappa;
    }
}

// ============================================================================
// Arc Length Tests
// ============================================================================

class ArcLengthTest : public ::testing::Test {};

TEST_F(ArcLengthTest, StraightLine) {
    auto curve = createLinearBezier(Vec2{0, 0}, Vec2{10, 0});
    EXPECT_NEAR(curve.arcLength(), 10.0, 1e-6);
}

TEST_F(ArcLengthTest, DiagonalLine) {
    auto curve = createLinearBezier(Vec2{0, 0}, Vec2{3, 4});
    EXPECT_NEAR(curve.arcLength(), 5.0, 1e-6);  // 3-4-5 triangle
}

TEST_F(ArcLengthTest, QuarterCircleApprox) {
    // Approximate quarter circle with cubic Bézier
    // Radius = 1, arc length should be ~π/2 ≈ 1.5708
    double k = 4.0/3.0 * (std::sqrt(2.0) - 1.0);  // Magic number for circle
    
    BezierCurve2D curve({
        Vec2{1.0, 0.0},
        Vec2{1.0, k},
        Vec2{k, 1.0},
        Vec2{0.0, 1.0}
    });
    
    double expectedLength = MathConstants::PI / 2.0;
    EXPECT_NEAR(curve.arcLength(), expectedLength, 0.001);  // ~0.1% error
}

TEST_F(ArcLengthTest, ArcLengthTable) {
    auto curve = createCubicBezier(
        Vec2{0, 0}, Vec2{1, 1}, Vec2{2, 1}, Vec2{3, 0});
    
    ArcLengthTable2D table(curve, 100);
    
    // Start and end should be exact
    EXPECT_NEAR(table.arcLengthAt(0.0), 0.0, 1e-10);
    EXPECT_NEAR(table.arcLengthAt(1.0), curve.arcLength(), 1e-6);
    
    // Monotonically increasing
    double prev = 0.0;
    for (double u = 0.1; u <= 1.0; u += 0.1) {
        double s = table.arcLengthAt(u);
        EXPECT_GT(s, prev);
        prev = s;
    }
}

TEST_F(ArcLengthTest, InverseArcLength) {
    auto curve = createCubicBezier(
        Vec2{0, 0}, Vec2{1, 1}, Vec2{2, 1}, Vec2{3, 0});
    
    ArcLengthTable2D table(curve, 100);
    double totalLength = curve.arcLength();
    
    // Inverse should be consistent with forward
    for (double u = 0.0; u <= 1.0; u += 0.1) {
        double s = table.arcLengthAt(u);
        double uBack = table.parameterAt(s);
        EXPECT_NEAR(uBack, u, 0.01);  // Within 1%
    }
}

// ============================================================================
// Continuity Tests
// ============================================================================

class ContinuityTest : public ::testing::Test {};

TEST_F(ContinuityTest, G0Continuity) {
    auto curve1 = createLinearBezier(Vec2{0, 0}, Vec2{1, 1});
    auto curve2 = createLinearBezier(Vec2{1, 1}, Vec2{2, 0});
    
    EXPECT_TRUE(checkG0Continuity(curve1, curve2));
    
    auto curve3 = createLinearBezier(Vec2{1.1, 1.0}, Vec2{2, 0});
    EXPECT_FALSE(checkG0Continuity(curve1, curve3));
}

TEST_F(ContinuityTest, G1Continuity) {
    // Two curves with same tangent at junction
    BezierCurve2D curve1({Vec2{0, 0}, Vec2{0.5, 0.5}, Vec2{1, 1}});
    BezierCurve2D curve2({Vec2{1, 1}, Vec2{1.5, 1.5}, Vec2{2, 2}});
    
    EXPECT_TRUE(checkG1Continuity(curve1, curve2, 0.01));
}

// ============================================================================
// 3D Curve Tests
// ============================================================================

class Bezier3DTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Helix-like curve
        curve = BezierCurve3D({
            Vec3{0, 0, 0},
            Vec3{1, 1, 1},
            Vec3{2, 0, 2},
            Vec3{3, 1, 3}
        });
    }
    
    BezierCurve3D curve;
};

TEST_F(Bezier3DTest, Evaluation) {
    auto start = curve.evaluate(0.0);
    auto end = curve.evaluate(1.0);
    
    EXPECT_NEAR(start[0], 0.0, 1e-10);
    EXPECT_NEAR(start[1], 0.0, 1e-10);
    EXPECT_NEAR(start[2], 0.0, 1e-10);
    
    EXPECT_NEAR(end[0], 3.0, 1e-10);
    EXPECT_NEAR(end[1], 1.0, 1e-10);
    EXPECT_NEAR(end[2], 3.0, 1e-10);
}

TEST_F(Bezier3DTest, Curvature3D) {
    // 3D curvature should be non-negative
    for (double u = 0.0; u <= 1.0; u += 0.1) {
        double kappa = curve.curvature(u);
        EXPECT_GE(kappa, 0.0);
        EXPECT_TRUE(std::isfinite(kappa));
    }
}

TEST_F(Bezier3DTest, ArcLength3D) {
    double length = curve.arcLength();
    EXPECT_GT(length, 0.0);
    
    // Should be at least as long as straight-line distance
    double directDist = Vec3{3, 1, 3}.length();
    EXPECT_GE(length, directDist);
}

// ============================================================================
// De Casteljau Algorithm Tests
// ============================================================================

class DeCasteljauTest : public ::testing::Test {};

TEST_F(DeCasteljauTest, LinearCase) {
    BezierCurve2D curve({Vec2{0, 0}, Vec2{2, 2}});
    
    // de Casteljau at u=0.5 should give midpoint
    auto pt = curve.evaluate(0.5);
    EXPECT_NEAR(pt[0], 1.0, 1e-10);
    EXPECT_NEAR(pt[1], 1.0, 1e-10);
}

TEST_F(DeCasteljauTest, QuadraticCase) {
    // Classic parabola: P0=(0,0), P1=(1,2), P2=(2,0)
    BezierCurve2D curve({Vec2{0, 0}, Vec2{1, 2}, Vec2{2, 0}});
    
    // At u=0.5: B(0.5) = 0.25*P0 + 0.5*P1 + 0.25*P2 = (1, 1)
    auto pt = curve.evaluate(0.5);
    EXPECT_NEAR(pt[0], 1.0, 1e-10);
    EXPECT_NEAR(pt[1], 1.0, 1e-10);
}

TEST_F(DeCasteljauTest, NumericalStability) {
    // Test with large coordinate values
    BezierCurve2D curve({
        Vec2{1e6, 1e6},
        Vec2{1e6 + 1, 1e6 + 1},
        Vec2{1e6 + 2, 1e6}
    });
    
    auto pt = curve.evaluate(0.5);
    EXPECT_NEAR(pt[0], 1e6 + 1, 1e-4);  // Some loss of precision expected
    EXPECT_NEAR(pt[1], 1e6 + 0.5, 1e-4);
}

