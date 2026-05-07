/**
 * @file MathTypesTest.cpp
 * @brief Unit tests for MathTypes.hpp
 *
 * Tests Vec<N,T> and Polynomial<MaxDegree,T> implementations.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MathTypes.hpp>
#include <cmath>

using namespace MotionPlanner;

// ============================================================================
// Vec Tests
// ============================================================================

class VecTest : public ::testing::Test {};

TEST_F(VecTest, DefaultConstruction) {
    Vec<3, double> v;
    EXPECT_DOUBLE_EQ(v[0], 0.0);
    EXPECT_DOUBLE_EQ(v[1], 0.0);
    EXPECT_DOUBLE_EQ(v[2], 0.0);
}

TEST_F(VecTest, InitializerListConstruction) {
    Vec<3, double> v{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);
    EXPECT_DOUBLE_EQ(v[2], 3.0);
}

TEST_F(VecTest, FillConstruction) {
    Vec<4, double> v(5.0);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(v[i], 5.0);
    }
}

TEST_F(VecTest, NamedAccessors2D) {
    Vec<2, double> v{3.0, 4.0};
    EXPECT_DOUBLE_EQ(v.x(), 3.0);
    EXPECT_DOUBLE_EQ(v.y(), 4.0);
}

TEST_F(VecTest, NamedAccessors3D) {
    Vec<3, double> v{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
}

TEST_F(VecTest, Addition) {
    Vec<3, double> a{1.0, 2.0, 3.0};
    Vec<3, double> b{4.0, 5.0, 6.0};
    Vec<3, double> c = a + b;
    EXPECT_DOUBLE_EQ(c[0], 5.0);
    EXPECT_DOUBLE_EQ(c[1], 7.0);
    EXPECT_DOUBLE_EQ(c[2], 9.0);
}

TEST_F(VecTest, Subtraction) {
    Vec<3, double> a{5.0, 7.0, 9.0};
    Vec<3, double> b{1.0, 2.0, 3.0};
    Vec<3, double> c = a - b;
    EXPECT_DOUBLE_EQ(c[0], 4.0);
    EXPECT_DOUBLE_EQ(c[1], 5.0);
    EXPECT_DOUBLE_EQ(c[2], 6.0);
}

TEST_F(VecTest, ScalarMultiplication) {
    Vec<3, double> v{1.0, 2.0, 3.0};
    Vec<3, double> scaled = v * 2.0;
    EXPECT_DOUBLE_EQ(scaled[0], 2.0);
    EXPECT_DOUBLE_EQ(scaled[1], 4.0);
    EXPECT_DOUBLE_EQ(scaled[2], 6.0);
}

TEST_F(VecTest, ScalarDivision) {
    Vec<3, double> v{2.0, 4.0, 6.0};
    Vec<3, double> div = v / 2.0;
    EXPECT_DOUBLE_EQ(div[0], 1.0);
    EXPECT_DOUBLE_EQ(div[1], 2.0);
    EXPECT_DOUBLE_EQ(div[2], 3.0);
}

TEST_F(VecTest, Negation) {
    Vec<3, double> v{1.0, -2.0, 3.0};
    Vec<3, double> neg = -v;
    EXPECT_DOUBLE_EQ(neg[0], -1.0);
    EXPECT_DOUBLE_EQ(neg[1], 2.0);
    EXPECT_DOUBLE_EQ(neg[2], -3.0);
}

TEST_F(VecTest, DotProduct) {
    Vec<3, double> a{1.0, 2.0, 3.0};
    Vec<3, double> b{4.0, 5.0, 6.0};
    double dot = a.dot(b);
    EXPECT_DOUBLE_EQ(dot, 32.0);  // 1*4 + 2*5 + 3*6 = 4 + 10 + 18
}

TEST_F(VecTest, CrossProduct2D) {
    Vec<2, double> a{1.0, 0.0};
    Vec<2, double> b{0.0, 1.0};
    double cross = a.cross(b);
    EXPECT_DOUBLE_EQ(cross, 1.0);  // Perpendicular, CCW
}

TEST_F(VecTest, CrossProduct3D) {
    Vec<3, double> a{1.0, 0.0, 0.0};
    Vec<3, double> b{0.0, 1.0, 0.0};
    Vec<3, double> c = a.cross(b);
    EXPECT_DOUBLE_EQ(c[0], 0.0);
    EXPECT_DOUBLE_EQ(c[1], 0.0);
    EXPECT_DOUBLE_EQ(c[2], 1.0);  // Z axis
}

TEST_F(VecTest, Length) {
    Vec<3, double> v{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(v.length(), 5.0);
}

TEST_F(VecTest, LengthSquared) {
    Vec<3, double> v{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(v.lengthSquared(), 25.0);
}

TEST_F(VecTest, Normalize) {
    Vec<3, double> v{3.0, 4.0, 0.0};
    Vec<3, double> n = v.normalized();
    EXPECT_NEAR(n[0], 0.6, 1e-10);
    EXPECT_NEAR(n[1], 0.8, 1e-10);
    EXPECT_NEAR(n[2], 0.0, 1e-10);
    EXPECT_NEAR(n.length(), 1.0, 1e-10);
}

TEST_F(VecTest, NormalizeZeroVector) {
    Vec<3, double> v{0.0, 0.0, 0.0};
    Vec<3, double> n = v.normalized();
    EXPECT_DOUBLE_EQ(n[0], 0.0);
    EXPECT_DOUBLE_EQ(n[1], 0.0);
    EXPECT_DOUBLE_EQ(n[2], 0.0);
}

TEST_F(VecTest, DistanceTo) {
    Vec<3, double> a{0.0, 0.0, 0.0};
    Vec<3, double> b{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(a.distanceTo(b), 5.0);
}

TEST_F(VecTest, Lerp) {
    Vec<3, double> a{0.0, 0.0, 0.0};
    Vec<3, double> b{10.0, 20.0, 30.0};
    
    Vec<3, double> mid = a.lerp(b, 0.5);
    EXPECT_DOUBLE_EQ(mid[0], 5.0);
    EXPECT_DOUBLE_EQ(mid[1], 10.0);
    EXPECT_DOUBLE_EQ(mid[2], 15.0);
    
    Vec<3, double> quarter = a.lerp(b, 0.25);
    EXPECT_DOUBLE_EQ(quarter[0], 2.5);
    EXPECT_DOUBLE_EQ(quarter[1], 5.0);
    EXPECT_DOUBLE_EQ(quarter[2], 7.5);
}

TEST_F(VecTest, IsZero) {
    Vec<3, double> zero{0.0, 0.0, 0.0};
    Vec<3, double> tiny{1e-15, 0.0, 0.0};
    Vec<3, double> nonzero{1.0, 0.0, 0.0};
    
    EXPECT_TRUE(zero.isZero());
    EXPECT_TRUE(tiny.isZero());  // Below epsilon
    EXPECT_FALSE(nonzero.isZero());
}

TEST_F(VecTest, Equality) {
    Vec<3, double> a{1.0, 2.0, 3.0};
    Vec<3, double> b{1.0, 2.0, 3.0};
    Vec<3, double> c{1.0, 2.0, 3.1};
    
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
    EXPECT_TRUE(a != c);
}

// ============================================================================
// Polynomial Tests
// ============================================================================

class PolynomialTest : public ::testing::Test {};

TEST_F(PolynomialTest, DefaultConstruction) {
    Polynomial<5, double> p;
    EXPECT_EQ(p.degree(), 0);
    EXPECT_DOUBLE_EQ(p.evaluate(1.0), 0.0);
}

TEST_F(PolynomialTest, CoefficientConstruction) {
    // p(x) = 1 + 2x + 3x²
    Polynomial<5, double> p({1.0, 2.0, 3.0});
    EXPECT_EQ(p.degree(), 2);
    EXPECT_DOUBLE_EQ(p[0], 1.0);
    EXPECT_DOUBLE_EQ(p[1], 2.0);
    EXPECT_DOUBLE_EQ(p[2], 3.0);
}

TEST_F(PolynomialTest, HornerEvaluation) {
    // p(x) = 1 + 2x + 3x² evaluated at x = 2
    // = 1 + 4 + 12 = 17
    Polynomial<5, double> p({1.0, 2.0, 3.0});
    EXPECT_DOUBLE_EQ(p.evaluate(2.0), 17.0);
}

TEST_F(PolynomialTest, Derivative) {
    // p(x) = 1 + 2x + 3x²
    // p'(x) = 2 + 6x
    Polynomial<5, double> p({1.0, 2.0, 3.0});
    auto dp = p.derivative();
    
    EXPECT_EQ(dp.degree(), 1);
    EXPECT_DOUBLE_EQ(dp[0], 2.0);
    EXPECT_DOUBLE_EQ(dp[1], 6.0);
}

TEST_F(PolynomialTest, SecondDerivative) {
    // p(x) = 1 + 2x + 3x² + 4x³
    // p'(x) = 2 + 6x + 12x²
    // p''(x) = 6 + 24x
    Polynomial<5, double> p({1.0, 2.0, 3.0, 4.0});
    auto d2p = p.derivative().derivative();
    
    EXPECT_EQ(d2p.degree(), 1);
    EXPECT_DOUBLE_EQ(d2p[0], 6.0);
    EXPECT_DOUBLE_EQ(d2p[1], 24.0);
}

TEST_F(PolynomialTest, Integral) {
    // p(x) = 2 + 6x
    // ∫p(x)dx = 2x + 3x²
    Polynomial<5, double> p({2.0, 6.0});
    auto ip = p.integral();
    
    EXPECT_EQ(ip.degree(), 2);
    EXPECT_DOUBLE_EQ(ip[0], 0.0);  // Constant of integration
    EXPECT_DOUBLE_EQ(ip[1], 2.0);
    EXPECT_DOUBLE_EQ(ip[2], 3.0);
}

TEST_F(PolynomialTest, Addition) {
    Polynomial<5, double> p({1.0, 2.0});       // 1 + 2x
    Polynomial<5, double> q({3.0, 4.0, 5.0});  // 3 + 4x + 5x²
    auto r = p + q;
    
    EXPECT_EQ(r.degree(), 2);
    EXPECT_DOUBLE_EQ(r[0], 4.0);
    EXPECT_DOUBLE_EQ(r[1], 6.0);
    EXPECT_DOUBLE_EQ(r[2], 5.0);
}

TEST_F(PolynomialTest, ScalarMultiplication) {
    Polynomial<5, double> p({1.0, 2.0, 3.0});
    auto r = p * 2.0;
    
    EXPECT_DOUBLE_EQ(r[0], 2.0);
    EXPECT_DOUBLE_EQ(r[1], 4.0);
    EXPECT_DOUBLE_EQ(r[2], 6.0);
}

TEST_F(PolynomialTest, QuadraticRoots_TwoRealRoots) {
    // p(x) = (x - 1)(x - 3) = x² - 4x + 3
    Polynomial<5, double> p({3.0, -4.0, 1.0});
    auto roots = p.quadraticRoots();
    
    EXPECT_TRUE(roots.has_value());
    auto [r1, r2] = *roots;
    EXPECT_NEAR(std::min(r1, r2), 1.0, 1e-10);
    EXPECT_NEAR(std::max(r1, r2), 3.0, 1e-10);
}

TEST_F(PolynomialTest, QuadraticRoots_OneRoot) {
    // p(x) = (x - 2)² = x² - 4x + 4
    Polynomial<5, double> p({4.0, -4.0, 1.0});
    auto roots = p.quadraticRoots();
    
    EXPECT_TRUE(roots.has_value());
    auto [r1, r2] = *roots;
    EXPECT_NEAR(r1, 2.0, 1e-10);
    EXPECT_NEAR(r2, 2.0, 1e-10);
}

TEST_F(PolynomialTest, QuadraticRoots_NoRealRoots) {
    // p(x) = x² + 1 (complex roots)
    Polynomial<5, double> p({1.0, 0.0, 1.0});
    auto roots = p.quadraticRoots();
    
    EXPECT_FALSE(roots.has_value());
}

TEST_F(PolynomialTest, CubicRoots_ThreeRealRoots) {
    // p(x) = (x - 1)(x - 2)(x - 3) = x³ - 6x² + 11x - 6
    Polynomial<5, double> p({-6.0, 11.0, -6.0, 1.0});
    auto roots = p.cubicRoots();
    
    EXPECT_EQ(roots.size(), 3);
    
    std::vector<double> sortedRoots(roots.begin(), roots.end());
    std::sort(sortedRoots.begin(), sortedRoots.end());
    
    EXPECT_NEAR(sortedRoots[0], 1.0, 1e-9);
    EXPECT_NEAR(sortedRoots[1], 2.0, 1e-9);
    EXPECT_NEAR(sortedRoots[2], 3.0, 1e-9);
}

// ============================================================================
// Helper Function Tests
// ============================================================================

TEST(MathHelperTest, Clamp) {
    EXPECT_DOUBLE_EQ(clamp(5.0, 0.0, 10.0), 5.0);
    EXPECT_DOUBLE_EQ(clamp(-5.0, 0.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(clamp(15.0, 0.0, 10.0), 10.0);
}

TEST(MathHelperTest, Constants) {
    EXPECT_NEAR(MathConstants::PI, 3.14159265358979323846, 1e-15);
    EXPECT_NEAR(MathConstants::TWO_PI, 2.0 * MathConstants::PI, 1e-15);
    EXPECT_NEAR(MathConstants::HALF_PI, MathConstants::PI / 2.0, 1e-15);
    EXPECT_NEAR(MathConstants::DEG_TO_RAD, MathConstants::PI / 180.0, 1e-15);
}

TEST(MathHelperTest, Binomial) {
    EXPECT_EQ(binomial(5, 0), 1);
    EXPECT_EQ(binomial(5, 1), 5);
    EXPECT_EQ(binomial(5, 2), 10);
    EXPECT_EQ(binomial(5, 3), 10);
    EXPECT_EQ(binomial(5, 4), 5);
    EXPECT_EQ(binomial(5, 5), 1);
}

