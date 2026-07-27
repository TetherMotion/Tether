/**
 * @file test_CertifiedSuggestionSolver.cpp
 * @brief Tests for certified limit suggestions via M15-pattern bisection
 */

#include "tether/motion_replanner/CertifiedSuggestionSolver.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <random>

using namespace tether::motion::replanner;

//=============================================================================
// Basic solver tests
//=============================================================================

TEST(CertifiedSuggestionSolver, ZeroErrorSuggestsMax) {
    SuggestionSolverConfig config;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 0.0, config);
    EXPECT_TRUE(s.accepted);
    EXPECT_NEAR(s.suggestedFeedRate, config.maxFeedRate * config.safetyFactor, 1e-6);
    EXPECT_NEAR(s.predictedError, 0.0, 1e-6);
}

TEST(CertifiedSuggestionSolver, ErrorWithinThresholdScalesUp) {
    // measuredError = 0.005mm, threshold = 0.01mm → can go faster.
    // v_max = 6000 * sqrt(0.01/0.005) = 6000 * sqrt(2) ≈ 8485.3
    SuggestionSolverConfig config;
    config.maxFeedRate = 20000.0; // Don't cap
    config.safetyFactor = 1.0;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 0.005, config);
    EXPECT_TRUE(s.accepted);
    double expected = 6000.0 * std::sqrt(0.01 / 0.005);
    EXPECT_NEAR(s.suggestedFeedRate, expected, 1e-3);
    EXPECT_LE(s.predictedError, config.contourErrorThreshold + 1e-9);
}

TEST(CertifiedSuggestionSolver, ErrorExceedsThresholdScalesDown) {
    // measuredError = 0.02mm, threshold = 0.01mm → must slow down.
    // v_max = 6000 * sqrt(0.01/0.02) = 6000 / sqrt(2) ≈ 4242.6
    SuggestionSolverConfig config;
    config.safetyFactor = 1.0;
    config.feedTolerance = 0.1;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 0.02, config);
    EXPECT_TRUE(s.accepted);
    double expected = 6000.0 * std::sqrt(0.01 / 0.02);
    EXPECT_NEAR(s.suggestedFeedRate, expected, 0.5);
    EXPECT_LE(s.predictedError, config.contourErrorThreshold + 1e-9);
}

TEST(CertifiedSuggestionSolver, AllFeedsRejected) {
    // Huge error, tiny threshold → even min feed is rejected.
    SuggestionSolverConfig config;
    config.contourErrorThreshold = 0.0001; // 0.1 µm
    config.minFeedRate = 100.0;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 1.0, config);
    EXPECT_FALSE(s.accepted);
    EXPECT_NEAR(s.suggestedFeedRate, config.minFeedRate, 1e-6);
}

TEST(CertifiedSuggestionSolver, MaxFeedAccepted) {
    // Very small error → even max feed is accepted.
    SuggestionSolverConfig config;
    config.contourErrorThreshold = 100.0; // Huge threshold
    config.safetyFactor = 1.0;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 0.001, config);
    EXPECT_TRUE(s.accepted);
    EXPECT_NEAR(s.suggestedFeedRate, config.maxFeedRate, 1e-6);
}

TEST(CertifiedSuggestionSolver, SafetyFactorApplied) {
    // The suggested feed should be safetyFactor × the max accepted feed.
    SuggestionSolverConfig config;
    config.safetyFactor = 0.9;
    config.maxFeedRate = 20000.0;
    config.feedTolerance = 0.1;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 0.02, config);
    EXPECT_TRUE(s.accepted);

    // Compute the raw max accepted feed (without safety factor).
    double rawMax = 6000.0 * std::sqrt(config.contourErrorThreshold / 0.02);
    rawMax = std::min(rawMax, config.maxFeedRate);
    EXPECT_NEAR(s.suggestedFeedRate, rawMax * 0.9, 0.5);
}

TEST(CertifiedSuggestionSolver, BisectionConverges) {
    // Verify the bisection iterations are reasonable.
    SuggestionSolverConfig config;
    config.feedTolerance = 0.1; // 0.1 mm/min
    config.maxFeedRate = 60000.0;
    CertifiedSuggestion s = solveCertifiedFeedRate(6000.0, 0.05, config);
    EXPECT_TRUE(s.accepted);
    // log2(60000 / 0.1) ≈ 19 iterations max
    EXPECT_LE(s.iterations, 25);
    EXPECT_GT(s.iterations, 0);
}

//=============================================================================
// Curvature-aware capping
//=============================================================================

TEST(CertifiedSuggestionSolver, CurvatureCapApplied) {
    // Error-based suggestion would be high, but curvature cap is lower.
    SuggestionSolverConfig config;
    config.safetyFactor = 1.0;
    config.maxFeedRate = 20000.0;
    double curvatureCap = 3000.0; // mm/min

    CertifiedSuggestion s = solveCertifiedFeedRateWithCurvature(
        6000.0, 0.005, curvatureCap, config);

    // Without curvature cap: v_max = 6000 * sqrt(0.01/0.005) ≈ 8485
    // With curvature cap: min(8485, 3000) = 3000
    EXPECT_TRUE(s.accepted);
    EXPECT_NEAR(s.suggestedFeedRate, 3000.0, 1e-3);
}

TEST(CertifiedSuggestionSolver, CurvatureCapNotAppliedWhenHigher) {
    // Curvature cap is higher than error-based suggestion → no cap.
    SuggestionSolverConfig config;
    config.safetyFactor = 1.0;
    config.maxFeedRate = 20000.0;
    config.feedTolerance = 0.1; // Tighter for this test
    double curvatureCap = 20000.0; // High cap

    CertifiedSuggestion s = solveCertifiedFeedRateWithCurvature(
        6000.0, 0.02, curvatureCap, config);

    // Error-based: v_max = 6000 * sqrt(0.01/0.02) ≈ 4242.6
    // Curvature cap: 20000 → not applied.
    double expected = 6000.0 * std::sqrt(0.01 / 0.02);
    EXPECT_NEAR(s.suggestedFeedRate, expected, 0.5); // Within bisection tol
}

//=============================================================================
// Fuzz test — T3 analog: no suggested feed violates the threshold
//=============================================================================

TEST(CertifiedSuggestionSolver, FuzzNoViolation) {
    // Mirror ToleranceFuzzTests.cpp: random measured feeds and errors,
    // verify that the suggested feed never violates the threshold.
    std::mt19937_64 rng(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<double> feedDist(100.0, 12000.0);
    std::uniform_real_distribution<double> errorDist(0.0001, 0.5);
    std::uniform_real_distribution<double> thresholdDist(0.001, 0.1);

    SuggestionSolverConfig config;
    config.safetyFactor = 1.0; // No safety margin — test the raw guarantee
    config.feedTolerance = 0.01;
    config.maxFeedRate = 20000.0;

    int violations = 0;
    int trials = 200;
    for (int i = 0; i < trials; ++i) {
        double measuredFeed = feedDist(rng);
        double measuredError = errorDist(rng);
        config.contourErrorThreshold = thresholdDist(rng);

        CertifiedSuggestion s = solveCertifiedFeedRate(
            measuredFeed, measuredError, config);

        if (s.accepted) {
            // T3 analog: the predicted error at the suggested feed
            // must be ≤ threshold.
            if (s.predictedError > config.contourErrorThreshold + 1e-9) {
                ++violations;
            }
        }
    }

    EXPECT_EQ(violations, 0) << "T3 violation: " << violations
                             << " out of " << trials
                             << " suggested feeds exceeded the threshold";
}
