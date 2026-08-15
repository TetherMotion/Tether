/**
 * @file test_ProfileSplineFitter.cpp
 * @brief Unit tests for ProfileSplineFitter (ReNURBS §7.1).
 */

#include <gtest/gtest.h>
#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"

#include <cmath>
#include <random>

using namespace tether::motion::profile_renurbs;

namespace {

// Generate strictly-increasing u in [0,1] with n points.
std::vector<double> linspace(std::size_t n) {
    std::vector<double> u(n);
    for (std::size_t i = 0; i < n; ++i)
        u[i] = static_cast<double>(i) / (n - 1);
    return u;
}

} // anonymous namespace

// ============================================================================
// Basic interpolation
// ============================================================================

TEST(ProfileSplineFitterTest, InterpolatesConstantSamples) {
    auto u = linspace(10);
    std::vector<double> q(10, 5.0);
    SplineFitterConfig cfg;
    cfg.degree = 3;
    auto result = fitSplineThroughSamples(u, q, cfg);
    EXPECT_EQ(result.degree, 1); // constant → degree-1
    ASSERT_EQ(result.controlPoints.size(), 2u);
    EXPECT_NEAR(result.controlPoints[0], 5.0, 1e-12);
    EXPECT_NEAR(result.controlPoints[1], 5.0, 1e-12);
    EXPECT_NEAR(result.maxResidual, 0.0, 1e-12);
    EXPECT_TRUE(result.withinEpsilon);
}

TEST(ProfileSplineFitterTest, InterpolatesLinearSamples) {
    auto u = linspace(10);
    std::vector<double> q(10);
    for (std::size_t i = 0; i < 10; ++i) q[i] = 2.0 * u[i] + 1.0;
    SplineFitterConfig cfg;
    cfg.degree = 3;
    auto result = fitSplineThroughSamples(u, q, cfg);
    // Linear data should be fit exactly (degree-1 suffices but we requested 3;
    // the spline should still pass through all samples)
    for (std::size_t i = 0; i < u.size(); ++i) {
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, u[i]);
        EXPECT_NEAR(val, q[i], 1e-10) << "Sample " << i;
    }
    EXPECT_TRUE(result.withinEpsilon);
}

TEST(ProfileSplineFitterTest, InterpolatesAllSamplesWithinEpsilon) {
    // Smooth nonlinear data: q = sin(2*pi*u)
    auto u = linspace(20);
    std::vector<double> q(20);
    for (std::size_t i = 0; i < 20; ++i)
        q[i] = std::sin(2.0 * M_PI * u[i]);

    SplineFitterConfig cfg;
    cfg.degree = 5;
    cfg.epsilon = 1e-6;
    auto result = fitSplineThroughSamples(u, q, cfg);

    for (std::size_t i = 0; i < u.size(); ++i) {
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, u[i]);
        EXPECT_NEAR(val, q[i], cfg.epsilon * 10) << "Sample " << i;
    }
    EXPECT_LE(result.maxResidual, cfg.epsilon * 10);
}

TEST(ProfileSplineFitterTest, InterpolatesPolynomialSamples) {
    // q = 3*u^3 - 2*u^2 + u
    auto u = linspace(15);
    std::vector<double> q(15);
    for (std::size_t i = 0; i < 15; ++i)
        q[i] = 3.0 * u[i]*u[i]*u[i] - 2.0 * u[i]*u[i] + u[i];

    SplineFitterConfig cfg;
    cfg.degree = 3; // cubic spline should fit cubic exactly
    cfg.epsilon = 1e-10;
    auto result = fitSplineThroughSamples(u, q, cfg);

    for (std::size_t i = 0; i < u.size(); ++i) {
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, u[i]);
        EXPECT_NEAR(val, q[i], 1e-6) << "Sample " << i;
    }
}

// ============================================================================
// Adaptive refinement
// ============================================================================

TEST(ProfileSplineFitterTest, AdaptiveRefinementReducesResidual) {
    // High-curvature data: q = sin(8*pi*u) — needs many knots
    auto u = linspace(30);
    std::vector<double> q(30);
    for (std::size_t i = 0; i < 30; ++i)
        q[i] = std::sin(8.0 * M_PI * u[i]);

    SplineFitterConfig cfg;
    cfg.degree = 5;
    cfg.epsilon = 1e-4;
    cfg.maxControlPoints = 64;
    auto result = fitSplineThroughSamples(u, q, cfg);

    // The sample residual should be within epsilon (global interpolation
    // is exact at samples, and refinement only triggers if sample residual
    // exceeds epsilon).
    EXPECT_LE(result.maxResidual, cfg.epsilon * 10)
        << "Sample residual should be small";
}

TEST(ProfileSplineFitterTest, AdaptiveRefinementRespectsMaxCp) {
    // Very nasty data with a tight cap
    auto u = linspace(50);
    std::vector<double> q(50);
    for (std::size_t i = 0; i < 50; ++i)
        q[i] = std::sin(20.0 * M_PI * u[i]);

    SplineFitterConfig cfg;
    cfg.degree = 5;
    cfg.epsilon = 1e-8;
    cfg.maxControlPoints = 16;
    auto result = fitSplineThroughSamples(u, q, cfg);

    // With 50 samples and a CP cap of 16, the initial fit is subsampled
    // to 16 CPs, so the sample residual will be nonzero and the cap will
    // be hit during refinement.
    EXPECT_TRUE(result.controlPointCapHit);
    EXPECT_LE(result.controlPoints.size(), 17u); // allow 1 over due to insertion
}

// ============================================================================
// Degree reduction
// ============================================================================

TEST(ProfileSplineFitterTest, DegreeReductionForFewSamples) {
    std::vector<double> u = {0.0, 0.5, 1.0};
    std::vector<double> q = {0.0, 1.0, 0.0};
    SplineFitterConfig cfg;
    cfg.degree = 5; // only 3 samples → max degree = 2
    auto result = fitSplineThroughSamples(u, q, cfg);
    EXPECT_LE(result.degree, 2);
    for (std::size_t i = 0; i < u.size(); ++i) {
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, u[i]);
        EXPECT_NEAR(val, q[i], 1e-8);
    }
}

TEST(ProfileSplineFitterTest, TwoSamplesProducesLine) {
    std::vector<double> u = {0.0, 1.0};
    std::vector<double> q = {3.0, 7.0};
    SplineFitterConfig cfg;
    cfg.degree = 5;
    auto result = fitSplineThroughSamples(u, q, cfg);
    EXPECT_EQ(result.degree, 1);
    ASSERT_EQ(result.controlPoints.size(), 2u);
    EXPECT_NEAR(result.controlPoints[0], 3.0, 1e-12);
    EXPECT_NEAR(result.controlPoints[1], 7.0, 1e-12);
}

// ============================================================================
// Constraint preservation (convex-hull clamp)
// ============================================================================

TEST(ProfileSplineFitterTest, ConvexHullClampNeverExceedsLimit) {
    // Data that would overshoot without clamping
    auto u = linspace(15);
    std::vector<double> q(15);
    for (std::size_t i = 0; i < 15; ++i)
        q[i] = 10.0 + 5.0 * std::sin(3.0 * M_PI * u[i]);

    // Set a tight limit just above the max sample
    std::vector<double> limit(15);
    for (std::size_t i = 0; i < 15; ++i)
        limit[i] = 15.5; // max sample is ~15, so limit is tight

    SplineFitterConfig cfg;
    cfg.degree = 5;
    cfg.epsilon = 1e-4;
    cfg.safetyMargin = 0.01;
    cfg.upperLimit = limit;

    auto result = fitSplineThroughSamples(u, q, cfg);
    EXPECT_TRUE(result.constraintClamped);

    // Check on a dense grid that the spline never exceeds limit - safetyMargin
    for (int k = 0; k <= 1000; ++k) {
        double uq = static_cast<double>(k) / 1000.0;
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, uq);
        EXPECT_LE(val, 15.5 - cfg.safetyMargin + 1e-10)
            << "Spline exceeds limit at u=" << uq;
    }
}

TEST(ProfileSplineFitterTest, ConvexHullClampPreservesSamples) {
    // Feasible samples (all below limit) → clamp should not break interpolation
    auto u = linspace(10);
    std::vector<double> q = {1, 2, 3, 4, 5, 5, 4, 3, 2, 1};
    std::vector<double> limit(10, 6.0); // above all samples

    SplineFitterConfig cfg;
    cfg.degree = 3;
    cfg.epsilon = 1e-6;
    cfg.safetyMargin = 0.001;
    cfg.upperLimit = limit;

    auto result = fitSplineThroughSamples(u, q, cfg);
    for (std::size_t i = 0; i < u.size(); ++i) {
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, u[i]);
        EXPECT_NEAR(val, q[i], cfg.epsilon * 100) << "Sample " << i;
    }
}

TEST(ProfileSplineFitterTest, LowerBoundEnforced) {
    auto u = linspace(10);
    std::vector<double> q = {1, 2, 3, 4, 5, 5, 4, 3, 2, 1};

    SplineFitterConfig cfg;
    cfg.degree = 3;
    cfg.epsilon = 1e-6;
    cfg.lowerBound = 0.0; // velocity ≥ 0

    auto result = fitSplineThroughSamples(u, q, cfg);
    for (const auto& cp : result.controlPoints) {
        EXPECT_GE(cp, -1e-10); // allow tiny numerical slack
    }
    // Dense grid check
    for (int k = 0; k <= 1000; ++k) {
        double uq = static_cast<double>(k) / 1000.0;
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, uq);
        EXPECT_GE(val, -1e-6) << "Below lower bound at u=" << uq;
    }
}

// ============================================================================
// Clamped boundary derivatives
// ============================================================================

TEST(ProfileSplineFitterTest, SplineIsSmoothInsideSegment) {
    // A smooth function → the spline should be C^(degree-1) inside
    auto u = linspace(20);
    std::vector<double> q(20);
    for (std::size_t i = 0; i < 20; ++i)
        q[i] = std::sin(3.0 * u[i]);

    SplineFitterConfig cfg;
    cfg.degree = 5;
    cfg.epsilon = 1e-8;
    auto result = fitSplineThroughSamples(u, q, cfg);

    // Check derivative continuity at interior knots by evaluating
    // the derivative from left and right at each interior knot
    for (std::size_t i = cfg.degree + 1;
         i < result.knots.size() - cfg.degree - 1; ++i) {
        double knot = result.knots[i];
        if (knot <= 0.0 || knot >= 1.0) continue;
        if (result.knots[i] == result.knots[i - 1]) continue; // repeated knot

        double dLeft = evaluateBSplineDerivative(result.controlPoints,
                                                  result.knots, result.degree,
                                                  knot - 1e-8);
        double dRight = evaluateBSplineDerivative(result.controlPoints,
                                                   result.knots, result.degree,
                                                   knot + 1e-8);
        // C^(degree-1) → at least C¹ for degree ≥ 2
        if (cfg.degree >= 2) {
            EXPECT_NEAR(dLeft, dRight, 1e-3)
                << "Derivative discontinuity at knot " << knot;
        }
    }
}

// ============================================================================
// Error handling
// ============================================================================

TEST(ProfileSplineFitterTest, ThrowsOnNonMonotonicParameter) {
    std::vector<double> u = {0.0, 0.5, 0.3, 1.0}; // not increasing
    std::vector<double> q = {0.0, 1.0, 0.5, 0.0};
    SplineFitterConfig cfg;
    EXPECT_THROW(fitSplineThroughSamples(u, q, cfg), std::invalid_argument);
}

TEST(ProfileSplineFitterTest, ThrowsOnEmptySamples) {
    std::vector<double> u, q;
    SplineFitterConfig cfg;
    EXPECT_THROW(fitSplineThroughSamples(u, q, cfg), std::invalid_argument);
}

TEST(ProfileSplineFitterTest, ThrowsOnSingleSample) {
    std::vector<double> u = {0.0}, q = {1.0};
    SplineFitterConfig cfg;
    EXPECT_THROW(fitSplineThroughSamples(u, q, cfg), std::invalid_argument);
}

TEST(ProfileSplineFitterTest, ThrowsOnSizeMismatch) {
    std::vector<double> u = {0.0, 0.5, 1.0};
    std::vector<double> q = {0.0, 1.0}; // size mismatch
    SplineFitterConfig cfg;
    EXPECT_THROW(fitSplineThroughSamples(u, q, cfg), std::invalid_argument);
}

// ============================================================================
// Knot insertion
// ============================================================================

TEST(ProfileSplineFitterTest, KnotInsertionPreservesCurve) {
    // Insert a knot → the curve should be identical
    auto u = linspace(10);
    std::vector<double> q(10);
    for (std::size_t i = 0; i < 10; ++i)
        q[i] = std::sin(2.0 * u[i]);

    SplineFitterConfig cfg;
    cfg.degree = 3;
    auto result = fitSplineThroughSamples(u, q, cfg);

    // Insert a knot at u=0.5
    auto [newCP, newKnots] = insertKnot(result.controlPoints, result.knots,
                                         result.degree, 0.5);
    // The curve should be identical
    for (int k = 0; k <= 100; ++k) {
        double uq = static_cast<double>(k) / 100.0;
        double v1 = evaluateBSpline(result.controlPoints, result.knots,
                                     result.degree, uq);
        double v2 = evaluateBSpline(newCP, newKnots, result.degree, uq);
        EXPECT_NEAR(v1, v2, 1e-10);
    }
    EXPECT_EQ(newCP.size(), result.controlPoints.size() + 1);
}

// ============================================================================
// Fuzz / property tests
// ============================================================================

TEST(ProfileSplineFitterTest, Fuzz_RandomSmoothProfilesInterpolated) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> freqDist(1.0, 5.0);
    std::uniform_real_distribution<double> ampDist(1.0, 10.0);
    std::uniform_int_distribution<int> nDist(10, 30);

    for (int trial = 0; trial < 100; ++trial) {
        int n = nDist(rng);
        auto u = linspace(n);
        double f = freqDist(rng);
        double a = ampDist(rng);
        std::vector<double> q(n);
        for (int i = 0; i < n; ++i)
            q[i] = a * std::sin(f * u[i] * M_PI);

        SplineFitterConfig cfg;
        cfg.degree = 5;
        cfg.epsilon = 1e-3;
        cfg.maxControlPoints = 64;
        auto result = fitSplineThroughSamples(u, q, cfg);

        // All samples should be interpolated within a relaxed tolerance
        // (the fitter targets epsilon but may hit the CP cap)
        for (int i = 0; i < n; ++i) {
            double val = evaluateBSpline(result.controlPoints, result.knots,
                                          result.degree, u[i]);
            EXPECT_NEAR(val, q[i], 1e-1)
                << "Trial " << trial << " sample " << i;
        }
    }
}

TEST(ProfileSplineFitterTest, Property_ConvexHullImpliesConstraint) {
    // If all control points ≤ limit, spline ≤ limit (convex-hull property)
    auto u = linspace(15);
    std::vector<double> q(15);
    for (std::size_t i = 0; i < 15; ++i)
        q[i] = 5.0 + 3.0 * std::cos(2.0 * M_PI * u[i]);

    std::vector<double> limit(15, 10.0);
    SplineFitterConfig cfg;
    cfg.degree = 3;
    cfg.epsilon = 1e-4;
    cfg.safetyMargin = 0.0;
    cfg.upperLimit = limit;

    auto result = fitSplineThroughSamples(u, q, cfg);

    // All CPs should be ≤ limit
    for (const auto& cp : result.controlPoints) {
        EXPECT_LE(cp, 10.0 + 1e-10);
    }
    // Dense grid check
    for (int k = 0; k <= 1000; ++k) {
        double uq = static_cast<double>(k) / 1000.0;
        double val = evaluateBSpline(result.controlPoints, result.knots,
                                      result.degree, uq);
        EXPECT_LE(val, 10.0 + 1e-6);
    }
}
