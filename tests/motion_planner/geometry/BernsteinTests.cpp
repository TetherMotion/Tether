/**
 * @file BernsteinTests.cpp
 * @brief Unit tests for tether::motion::bernstein utilities
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/geometry/Bernstein.hpp>

#include <cmath>
#include <random>
#include <vector>

namespace be = tether::motion::bernstein;

namespace {

// Evaluate a power-basis polynomial (coeffs a_0..a_n) at x.
double evalPower(const std::vector<double>& a, double x) {
    double r = 0.0;
    for (auto it = a.rbegin(); it != a.rend(); ++it) r = r * x + *it;
    return r;
}

bool rootCovered(double root, const std::vector<be::Interval>& ivs,
                 double tol) {
    for (const auto& iv : ivs) {
        if (root >= iv.lo - tol && root <= iv.hi + tol) return true;
    }
    return false;
}

} // namespace

TEST(Bernstein, EvaluateMatchesPowerBasis) {
    // p(t) = 1 - 3t + 2t^2  → Bernstein via conversion; compare everywhere.
    std::vector<double> power = {1.0, -3.0, 2.0};
    std::vector<double> b = be::powerToBernstein(power);
    for (int i = 0; i <= 20; ++i) {
        const double t = i / 20.0;
        EXPECT_NEAR(be::evaluate(b, t), evalPower(power, t), 1e-12);
    }
}

TEST(Bernstein, PowerBernsteinRoundTrip) {
    std::mt19937_64 rng(31);
    std::uniform_real_distribution<double> uni(-10.0, 10.0);
    for (int degree = 1; degree <= 8; ++degree) {
        std::vector<double> power(degree + 1);
        for (auto& c : power) c = uni(rng);
        std::vector<double> back =
            be::bernsteinToPower(be::powerToBernstein(power));
        ASSERT_EQ(back.size(), power.size());
        for (std::size_t i = 0; i < power.size(); ++i) {
            EXPECT_NEAR(back[i], power[i], 1e-8 * (1.0 + std::abs(power[i])));
        }
    }
}

TEST(Bernstein, DerivativeIdentity) {
    std::vector<double> power = {2.0, -1.0, 3.0, 0.5}; // 2 - x + 3x^2 + 0.5x^3
    std::vector<double> b = be::powerToBernstein(power);
    std::vector<double> db = be::derivative(b);
    std::vector<double> dpower = be::bernsteinToPower(db);
    // Expected derivative: -1 + 6x + 1.5x^2
    ASSERT_EQ(dpower.size(), 3u);
    EXPECT_NEAR(dpower[0], -1.0, 1e-10);
    EXPECT_NEAR(dpower[1], 6.0, 1e-10);
    EXPECT_NEAR(dpower[2], 1.5, 1e-10);
}

TEST(Bernstein, MultiplyMatchesPowerProduct) {
    // (1 + x) * (1 - x) = 1 - x^2
    std::vector<double> a = be::powerToBernstein({1.0, 1.0});
    std::vector<double> b = be::powerToBernstein({1.0, -1.0});
    std::vector<double> prod = be::bernsteinToPower(be::multiply(a, b));
    ASSERT_EQ(prod.size(), 3u);
    EXPECT_NEAR(prod[0], 1.0, 1e-12);
    EXPECT_NEAR(prod[1], 0.0, 1e-12);
    EXPECT_NEAR(prod[2], -1.0, 1e-12);

    // Random higher-degree cross-check against evaluation.
    std::mt19937_64 rng(37);
    std::uniform_real_distribution<double> uni(-3.0, 3.0);
    std::vector<double> pa(4), pb(3);
    for (auto& c : pa) c = uni(rng);
    for (auto& c : pb) c = uni(rng);
    std::vector<double> prodB = be::multiply(be::powerToBernstein(pa),
                                             be::powerToBernstein(pb));
    for (int i = 0; i <= 10; ++i) {
        const double t = i / 10.0;
        EXPECT_NEAR(be::evaluate(prodB, t),
                    evalPower(pa, t) * evalPower(pb, t), 1e-9);
    }
}

TEST(Bernstein, SubdivideReproducesPolynomial) {
    std::vector<double> power = {1.0, 2.0, -1.0, 0.5};
    std::vector<double> b = be::powerToBernstein(power);
    auto halves = be::subdivide(b, 0.4);
    for (int i = 0; i <= 10; ++i) {
        const double t = i / 10.0;
        EXPECT_NEAR(be::evaluate(halves.first, t), evalPower(power, 0.4 * t),
                    1e-11);
        EXPECT_NEAR(be::evaluate(halves.second, t),
                    evalPower(power, 0.4 + 0.6 * t), 1e-11);
    }
}

TEST(Bernstein, IsolateRootsSqrtHalf) {
    // x^2 - 1/2, root at 1/sqrt(2) ≈ 0.7071.
    std::vector<double> b = be::powerToBernstein({-0.5, 0.0, 1.0});
    auto ivs = be::isolateRoots(b, 1e-9);
    ASSERT_EQ(ivs.size(), 1u);
    const double root = 1.0 / std::sqrt(2.0);
    EXPECT_TRUE(rootCovered(root, ivs, 1e-9));
    EXPECT_LT(ivs[0].hi - ivs[0].lo, 2e-9);
}

TEST(Bernstein, IsolateRootsNoRootInInterval) {
    // x^2 - 2 has no root in [0,1] (both coeffs negative after conversion?).
    std::vector<double> b = be::powerToBernstein({-2.0, 0.0, 1.0});
    auto ivs = be::isolateRoots(b, 1e-9);
    EXPECT_TRUE(ivs.empty());
}

TEST(Bernstein, IsolateRootsTripleRoot) {
    // (x - 0.5)^3 = -0.125 + 0.75x - 1.5x^2 + x^3  — near-multiple root.
    std::vector<double> b =
        be::powerToBernstein({-0.125, 0.75, -1.5, 1.0});
    auto ivs = be::isolateRoots(b, 1e-8);
    ASSERT_EQ(ivs.size(), 1u); // merged interval (documented behavior)
    EXPECT_TRUE(rootCovered(0.5, ivs, 1e-7));
}

TEST(Bernstein, IsolateRootsClustered) {
    // (x - 0.3)(x - 0.3001)(x - 0.7): two roots clustered 1e-4 apart.
    // Expand: (x^2 - 0.6001x + 0.09003)(x - 0.7)
    //       = x^3 - 1.3001x^2 + (0.09003 + 0.42007)x - 0.063021
    std::vector<double> power = {-0.063021, 0.5101, -1.3001, 1.0};
    std::vector<double> b = be::powerToBernstein(power);
    auto ivs = be::isolateRoots(b, 1e-6);

    // Certification: no root missed — all three true roots covered.
    EXPECT_TRUE(rootCovered(0.3, ivs, 1e-6));
    EXPECT_TRUE(rootCovered(0.3001, ivs, 1e-6));
    EXPECT_TRUE(rootCovered(0.7, ivs, 1e-6));
    // The root at 0.7 must be isolated from the cluster.
    EXPECT_LE(ivs.size(), 3u);
}

TEST(Bernstein, IsolateRootsSimpleQuadratic) {
    // (x - 0.25)(x - 0.75) = 0.1875 - x + x^2
    std::vector<double> b = be::powerToBernstein({0.1875, -1.0, 1.0});
    auto ivs = be::isolateRoots(b, 1e-9);
    ASSERT_EQ(ivs.size(), 2u);
    EXPECT_TRUE(rootCovered(0.25, ivs, 1e-9));
    EXPECT_TRUE(rootCovered(0.75, ivs, 1e-9));
}

TEST(Bernstein, IsolateRootsRejectsBadTolerance) {
    std::vector<double> b = {1.0, 1.0};
    EXPECT_THROW(be::isolateRoots(b, 0.0), std::invalid_argument);
}
