/**
 * @file test_CurvatureAwareLimiter.cpp
 * @brief Tests for curvature-aware proactive feed limiting
 */

#include "tether/motion_replanner/CurvatureAwareLimiter.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::motion::replanner;
using namespace tether::motion;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos, double curvature) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    s.curvature = curvature;
    return s;
}

} // anonymous namespace

//=============================================================================
// computeFeedRateLimit — single-point formula
//=============================================================================

TEST(CurvatureAwareLimiter, StraightSegmentMaxFeed) {
    CurvatureLimiterConfig config;
    double feed = computeFeedRateLimit(0.0, config);
    // kappa=0 → straight → maxFeed * safety = 6000 * 0.9 = 5400
    EXPECT_NEAR(feed, 5400.0, 1e-6);
}

TEST(CurvatureAwareLimiter, CircleFeedLimit) {
    // On a circle of radius R, kappa = 1/R.
    // v = sqrt(a_cent / kappa) = sqrt(a_cent * R)
    // feed = v * 60 * safety
    double R = 50.0; // mm
    double kappa = 1.0 / R;
    CurvatureLimiterConfig config;
    config.maxCentripetalAcceleration = 500.0;
    config.safetyFactor = 1.0; // No safety margin for exact check
    config.maxFeedRate = 20000.0; // High enough to not cap

    double feed = computeFeedRateLimit(kappa, config);
    double expectedVelocity = std::sqrt(500.0 * R); // sqrt(25000) = 158.11 mm/s
    double expectedFeed = expectedVelocity * 60.0;
    EXPECT_NEAR(feed, expectedFeed, 1e-3);
}

TEST(CurvatureAwareLimiter, HighCurvatureCapsFeed) {
    // Very high curvature → very low feed
    CurvatureLimiterConfig config;
    double feed = computeFeedRateLimit(1.0, config); // kappa=1 → R=1mm
    // v = sqrt(500/1) = 22.36 mm/s, feed = 1341.6 mm/min, * 0.9 = 1207.5
    EXPECT_NEAR(feed, std::sqrt(500.0) * 60.0 * 0.9, 1e-3);
    // Should be well below maxFeedRate
    EXPECT_LT(feed, config.maxFeedRate);
}

TEST(CurvatureAwareLimiter, CappedAtMaxFeedRate) {
    CurvatureLimiterConfig config;
    config.maxFeedRate = 1000.0; // Low cap
    double feed = computeFeedRateLimit(1e-12, config); // Nearly straight
    // Should be capped at 1000 * 0.9 = 900
    EXPECT_NEAR(feed, 900.0, 1e-6);
}

//=============================================================================
// computeFeedLimitsFromSamples — immediate level
//=============================================================================

TEST(CurvatureAwareLimiter, FromSamplesStraightPath) {
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample(i * 0.1, i * 10.0, 0, 0, 1,
                                     i * 10.0, 0.0)); // curvature=0
    }

    CurvatureAwareFeedLimits limits = computeFeedLimitsFromSamples(samples);
    EXPECT_EQ(limits.points.size(), 11u);
    // All points should be at max feed (straight)
    EXPECT_NEAR(limits.minFeedRate, 5400.0, 1e-3);
    EXPECT_NEAR(limits.maxFeedRate, 5400.0, 1e-3);
}

TEST(CurvatureAwareLimiter, FromSamplesArcPath) {
    // Simulate a quarter arc with known curvature
    double R = 50.0;
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        double alpha = static_cast<double>(i) / 10.0;
        double angle = alpha * M_PI / 2;
        double x = R * std::cos(angle);
        double y = R * std::sin(angle);
        double pathPos = R * alpha * M_PI / 2;
        samples.push_back(makeSample(i * 0.1, x, y, 0, 3, pathPos, 1.0 / R));
    }

    CurvatureLimiterConfig config;
    config.safetyFactor = 1.0;
    config.maxFeedRate = 20000.0; // High enough to not cap
    CurvatureAwareFeedLimits limits = computeFeedLimitsFromSamples(samples, config);

    EXPECT_EQ(limits.points.size(), 11u);
    // All points have the same curvature → same feed limit
    double expectedVelocity = std::sqrt(500.0 * R); // sqrt(25000) = 158.11
    double expectedFeed = expectedVelocity * 60.0;   // 9486.8
    EXPECT_NEAR(limits.minFeedRate, expectedFeed, 1e-1);
    EXPECT_NEAR(limits.maxFeedRate, expectedFeed, 1e-1);
}

//=============================================================================
// computeCertifiedFeedLimits — certified level
//=============================================================================

TEST(CurvatureAwareLimiter, CertifiedLinePath) {
    // L-shaped path (straight segments only)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0, 0.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0, 0.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);

    CurvatureAwareFeedLimits limits = computeCertifiedFeedLimits(path, {}, 20);
    EXPECT_EQ(limits.points.size(), 20u);
    // Lines have zero curvature → max feed
    EXPECT_NEAR(limits.minFeedRate, 5400.0, 1e-3);
    EXPECT_NEAR(limits.maxFeedRate, 5400.0, 1e-3);
}

TEST(CurvatureAwareLimiter, CertifiedArcPath) {
    // CCW quarter arc, radius 50
    std::vector<TrajectorySample> samples;
    int n = 20;
    for (int i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        double angle = alpha * M_PI / 2;
        double x = 50.0 * std::cos(angle);
        double y = 50.0 * std::sin(angle);
        double pathPos = 50.0 * alpha * M_PI / 2;
        samples.push_back(makeSample(i * 0.01, x, y, 0, 3, pathPos, 1.0 / 50.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);

    CurvatureLimiterConfig config;
    config.safetyFactor = 1.0;
    CurvatureAwareFeedLimits limits = computeCertifiedFeedLimits(path, config, 20);

    EXPECT_EQ(limits.points.size(), 20u);
    // On a circle of R=50, v = sqrt(500*50) = sqrt(25000) ≈ 158.11 mm/s
    // feed = 158.11 * 60 = 9486.8 mm/min, but capped at maxFeedRate=6000
    // So all points should be at 6000 (capped).
    // Wait — sqrt(500*50) = 158.11 mm/s → feed = 9486.8 > 6000, so capped.
    EXPECT_NEAR(limits.maxFeedRate, 6000.0, 1e-3);
    // Min should also be at 6000 (constant curvature, capped)
    EXPECT_NEAR(limits.minFeedRate, 6000.0, 1e-3);
}

TEST(CurvatureAwareLimiter, CertifiedTightArc) {
    // Tight arc: radius 5mm → kappa = 0.2
    // v = sqrt(500/0.2) = sqrt(2500) = 50 mm/s → feed = 3000 mm/min
    std::vector<TrajectorySample> samples;
    int n = 20;
    double R = 5.0;
    for (int i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        double angle = alpha * M_PI / 2;
        double x = R * std::cos(angle) + 100.0; // Offset so X and Y vary
        double y = R * std::sin(angle);
        double pathPos = R * alpha * M_PI / 2;
        samples.push_back(makeSample(i * 0.01, x, y, 0, 3, pathPos, 1.0 / R));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);

    CurvatureLimiterConfig config;
    config.safetyFactor = 1.0;
    config.maxFeedRate = 6000.0;
    CurvatureAwareFeedLimits limits = computeCertifiedFeedLimits(path, config, 20);

    // v = sqrt(500 * 5) = 50 mm/s → feed = 3000 mm/min (< 6000 cap)
    // The certified curvature may be slightly higher than 1/R due to
    // the Lipschitz bound, so the feed may be slightly lower.
    EXPECT_LE(limits.minFeedRate, 3000.0 + 100.0); // Allow some tolerance
    EXPECT_GT(limits.minFeedRate, 0.0);
}

//=============================================================================
// certifiedFeedRateAt — single-point query
//=============================================================================

TEST(CurvatureAwareLimiter, FeedRateAtMidpoint) {
    // L-shaped path
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0, 0.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0, 0.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);

    double feed = certifiedFeedRateAt(path, 25.0); // Midpoint of first segment
    // Straight line → max feed
    EXPECT_NEAR(feed, 5400.0, 1e-3);
}

TEST(CurvatureAwareLimiter, EmptySamplesThrows) {
    std::vector<TrajectorySample> empty;
    // computeFeedLimitsFromSamples with empty samples returns empty result
    // (doesn't throw — just returns empty points)
    CurvatureAwareFeedLimits limits = computeFeedLimitsFromSamples(empty);
    EXPECT_TRUE(limits.points.empty());
    EXPECT_EQ(limits.minFeedRate, 0.0);
}

TEST(CurvatureAwareLimiter, ZeroNumSamplesThrows) {
    std::vector<TrajectorySample> samples;
    samples.push_back(makeSample(0, 0, 0, 0, 1, 0, 0));
    samples.push_back(makeSample(1, 100, 50, 0, 1, 100, 0));
    PiecewiseNurbsPath path = convertTrajectory(samples);

    EXPECT_THROW(computeCertifiedFeedLimits(path, {}, 0), std::invalid_argument);
}
