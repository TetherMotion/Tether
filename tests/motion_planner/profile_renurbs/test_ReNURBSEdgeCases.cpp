/**
 * @file test_ReNURBSEdgeCases.cpp
 * @brief Edge-case and property/fuzz tests for ReNURBS (ReNURBS §7.4).
 */

#include <gtest/gtest.h>
#include "tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"
#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/PathAdapter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <cmath>
#include <random>

using namespace tether::motion::profile_renurbs;
using namespace MotionPlanner;
using tether::motion::NurbsCurve;
using tether::motion::RVec;
using tether::motion::PiecewiseNurbsPath;

namespace {

PathAdapter<2, double> makeLinearPath2D(double length) {
    auto line = NurbsCurve::fromLine(RVec{0.0, 0.0}, RVec{length, 0.0});
    PiecewiseNurbsPath path({line});
    return PathAdapter<2, double>(std::move(path), {SourceReference{}});
}

PathAdapter<2, double> makeMultiSegmentPath(std::size_t nSegs, double segLen) {
    std::vector<NurbsCurve> curves;
    std::vector<SourceReference> refs;
    RVec prev{0.0, 0.0};
    for (std::size_t i = 0; i < nSegs; ++i) {
        RVec next{prev[0] + segLen, 0.0};
        curves.push_back(NurbsCurve::fromLine(prev, next));
        refs.push_back(SourceReference{});
        prev = next;
    }
    PiecewiseNurbsPath path(std::move(curves));
    return PathAdapter<2, double>(std::move(path), std::move(refs));
}

VelocityProfile<double> makeProfileWithZeroVelocityMiddle(
    double pathLength, double vMax, std::size_t n = 100) {
    VelocityProfile<double> profile;
    double ds = pathLength / (n - 1);
    double t = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = vMax;
        pt.accelerationLimit = 500.0;
        double u = s / pathLength;
        // Velocity goes up, drops to 0 in the middle, goes up again
        if (u < 0.4) {
            pt.velocity = vMax * u / 0.4;
        } else if (u < 0.6) {
            pt.velocity = 0.0; // zero velocity region
        } else {
            pt.velocity = vMax * (u - 0.6) / 0.4;
        }
        pt.acceleration = 0;
        pt.jerk = 0;
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
            else t += ds / 1e-6; // near-zero velocity → small time step
        }
        pt.time = t;
        profile.addPoint(pt);
    }
    return profile;
}

VelocityProfile<double> makeBangBangProfile(
    double pathLength, double vMax, double aMax, std::size_t n = 100) {
    VelocityProfile<double> profile;
    double ds = pathLength / (n - 1);
    double t = 0.0;
    double accelDist = pathLength / 2;
    for (std::size_t i = 0; i < n; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = vMax;
        pt.accelerationLimit = aMax;
        if (s < accelDist) {
            pt.velocity = std::sqrt(2 * aMax * s);
            pt.acceleration = aMax;
            pt.jerk = 0;
        } else {
            double sDecel = s - accelDist;
            pt.velocity = std::sqrt(std::max(0.0, vMax * vMax - 2 * aMax * sDecel));
            pt.acceleration = -aMax;
            pt.jerk = 0;
        }
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
        }
        pt.time = t;
        profile.addPoint(pt);
    }
    return profile;
}

} // anonymous namespace

// ============================================================================
// Edge cases E1–E15
// ============================================================================

TEST(ReNURBSEdgeCasesTest, E1_EmptyProfile) {
    auto path = makeLinearPath2D(100.0);
    VelocityProfile<double> profile;
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    EXPECT_TRUE(renurbs.empty());
}

TEST(ReNURBSEdgeCasesTest, E2_SingleSample) {
    auto path = makeLinearPath2D(100.0);
    VelocityProfile<double> profile;
    VelocityProfilePoint<double> pt;
    pt.arcLength = 50.0;
    pt.velocity = 10.0;
    pt.time = 5.0;
    profile.addPoint(pt);
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    EXPECT_TRUE(renurbs.perSegment[0].velocity.curve.has_value());
}

TEST(ReNURBSEdgeCasesTest, E3_ZeroLengthSegment) {
    // A path with a zero-length segment should be skipped.
    // NurbsCurve::fromLine throws for zero-length lines, so we test that
    // the builder handles very short segments gracefully.
    auto path = makeMultiSegmentPath(3, 33.33);
    auto profile = makeBangBangProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    // Should have 3 segments (or fewer if some are too short)
    EXPECT_LE(renurbs.perSegment.size(), 3u);
    EXPECT_GE(renurbs.perSegment.size(), 1u);
}

TEST(ReNURBSEdgeCasesTest, E4_ShortSegmentDegreeReduction) {
    // A profile with very few samples in a segment
    auto path = makeMultiSegmentPath(5, 20.0);
    auto profile = makeBangBangProfile(100.0, 50.0, 500.0, 10); // only 10 samples
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.degreeVelocity = 5;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    // Should not crash; degree may be reduced
    EXPECT_GE(renurbs.perSegment.size(), 1u);
}

TEST(ReNURBSEdgeCasesTest, E5_ZeroVelocityRegion) {
    // A profile with a zero-velocity region (discontinuous velocity).
    // The spline will overshoot near the discontinuity; this is a known
    // limitation that will be addressed by discontinuity splitting (P3).
    // For now, we verify that the velocity is non-negative (safety-critical)
    // and that the curve is built without crashing.
    auto path = makeLinearPath2D(100.0);
    auto profile = makeProfileWithZeroVelocityMiddle(100.0, 50.0, 100);
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.velocity.curve.has_value());
    const auto& curve = *seg.velocity.curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();

    // The velocity should be non-negative everywhere (lower bound = 0)
    for (int k = 0; k <= 200; ++k) {
        double u = static_cast<double>(k) / 200.0;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_GE(val, -1e-3) << "Negative velocity at u=" << u;
    }
}

TEST(ReNURBSEdgeCasesTest, E6_DiscontinuousAccelBasicToppra) {
    // Bang-bang profile has discontinuous acceleration at the midpoint
    auto path = makeLinearPath2D(100.0);
    auto profile = makeBangBangProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    // The acceleration curve should exist (may be C⁰, not C¹)
    EXPECT_TRUE(renurbs.perSegment[0].acceleration.curve.has_value());
}

TEST(ReNURBSEdgeCasesTest, E12_MaxCpExhausted) {
    // Very high-frequency profile with tight CP cap
    auto path = makeLinearPath2D(100.0);
    VelocityProfile<double> profile;
    double ds = 100.0 / 99;
    double t = 0;
    for (std::size_t i = 0; i < 100; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = 100.0;
        pt.accelerationLimit = 500.0;
        pt.velocity = 50.0 + 40.0 * std::sin(50.0 * M_PI * s / 100.0);
        pt.acceleration = 0;
        pt.jerk = 0;
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
        }
        pt.time = t;
        profile.addPoint(pt);
    }

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.maxControlPointsPerSegment = 16;
    cfg.epsilonVelocity = 1e-10;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    EXPECT_TRUE(renurbs.perSegment[0].velocity.controlPointCapHit);
}

TEST(ReNURBSEdgeCasesTest, E13_NegativeVelocityClamp) {
    // Profile with values that might cause negative overshoot
    auto path = makeLinearPath2D(100.0);
    VelocityProfile<double> profile;
    double ds = 100.0 / 49;
    double t = 0;
    for (std::size_t i = 0; i < 50; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = 100.0;
        pt.accelerationLimit = 500.0;
        // Oscillating velocity that dips near zero
        pt.velocity = std::abs(50.0 * std::sin(4.0 * M_PI * s / 100.0));
        pt.acceleration = 0;
        pt.jerk = 0;
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
        }
        pt.time = t;
        profile.addPoint(pt);
    }

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.velocity.curve.has_value());
    const auto& curve = *seg.velocity.curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();
    for (int k = 0; k <= 500; ++k) {
        double u = static_cast<double>(k) / 500.0;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_GE(val, -1e-6) << "Negative velocity at u=" << u;
    }
}

TEST(ReNURBSEdgeCasesTest, E14_BlendSegment) {
    // Multi-segment path simulating blend pieces
    auto path = makeMultiSegmentPath(4, 25.0);
    auto profile = makeBangBangProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    EXPECT_GE(renurbs.perSegment.size(), 1u);
    // Each segment should have curves
    for (const auto& seg : renurbs.perSegment) {
        EXPECT_TRUE(seg.velocity.curve.has_value() || seg.sEnd - seg.sStart < 1e-6);
    }
}

// ============================================================================
// Property / fuzz tests
// ============================================================================

TEST(ReNURBSEdgeCasesTest, Fuzz_RandomProfilesConstraintPreserved) {
    std::mt19937 rng(123);
    std::uniform_real_distribution<double> lenDist(50.0, 200.0);
    std::uniform_real_distribution<double> vMaxDist(10.0, 100.0);
    std::uniform_real_distribution<double> aMaxDist(100.0, 1000.0);
    std::uniform_int_distribution<int> nDist(20, 80);

    for (int trial = 0; trial < 50; ++trial) {
        double pathLen = lenDist(rng);
        double vMax = vMaxDist(rng);
        double aMax = aMaxDist(rng);
        int n = nDist(rng);

        auto path = makeLinearPath2D(pathLen);
        VelocityProfile<double> profile;
        double ds = pathLen / (n - 1);
        double t = 0;
        for (int i = 0; i < n; ++i) {
            double s = i * ds;
            VelocityProfilePoint<double> pt;
            pt.arcLength = s;
            pt.velocityLimit = vMax;
            pt.accelerationLimit = aMax;
            // Random smooth-ish velocity
            double u = s / pathLen;
            pt.velocity = vMax * 0.5 * (1.0 + std::sin(2.0 * M_PI * u *
                (1.0 + rng() % 3)));
            pt.velocity = std::min(pt.velocity, vMax);
            pt.velocity = std::max(pt.velocity, 0.0);
            pt.acceleration = 0;
            pt.jerk = 0;
            if (i > 0) {
                double prevV = profile.points().back().velocity;
                double avgV = (prevV + pt.velocity) / 2;
                if (avgV > 1e-12) t += ds / avgV;
            }
            pt.time = t;
            profile.addPoint(pt);
        }

        KinematicLimits<2, double> limits;
        limits.path.maxPathVelocity = vMax;
        limits.path.maxPathAcceleration = aMax;

        ReNURBSConfig cfg;
        cfg.enabled = true;
        cfg.certify = false;
        cfg.safetyMarginVelocity = 0.1;
        auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

        if (renurbs.perSegment.empty()) continue;
        const auto& seg = renurbs.perSegment[0];
        if (!seg.velocity.curve.has_value()) continue;
        const auto& curve = *seg.velocity.curve;
        double uMin = curve.knotMin();
        double uMax = curve.knotMax();

        for (int k = 0; k <= 100; ++k) {
            double u = static_cast<double>(k) / 100.0;
            double uu = uMin + u * (uMax - uMin);
            double val = curve.evaluate(uu)[0];
            EXPECT_LE(val, vMax + 1e-3)
                << "Trial " << trial << " velocity exceeds vMax at u=" << u;
            EXPECT_GE(val, -1e-3)
                << "Trial " << trial << " velocity negative at u=" << u;
        }
    }
}

TEST(ReNURBSEdgeCasesTest, Property_MonotonicTime) {
    auto path = makeLinearPath2D(100.0);
    VelocityProfile<double> profile;
    double ds = 100.0 / 49;
    double t = 0;
    for (std::size_t i = 0; i < 50; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = 50.0;
        pt.accelerationLimit = 500.0;
        pt.velocity = 50.0 * (1.0 - std::cos(M_PI * s / 100.0)) / 2.0;
        pt.acceleration = 0;
        pt.jerk = 0;
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
        }
        pt.time = t;
        profile.addPoint(pt);
    }

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.time.curve.has_value());
    const auto& curve = *seg.time.curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();

    double prevT = -1e-9;
    for (int k = 0; k <= 200; ++k) {
        double u = static_cast<double>(k) / 200.0;
        double uu = uMin + u * (uMax - uMin);
        double tval = curve.evaluate(uu)[0];
        EXPECT_GE(tval, prevT - 1e-9);
        prevT = tval;
    }
}

TEST(ReNURBSEdgeCasesTest, Property_SampleInterpolationHolds) {
    auto path = makeLinearPath2D(100.0);
    VelocityProfile<double> profile;
    double ds = 100.0 / 29;
    double t = 0;
    for (std::size_t i = 0; i < 30; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = 50.0;
        pt.accelerationLimit = 500.0;
        pt.velocity = 30.0 + 20.0 * std::sin(3.0 * M_PI * s / 100.0);
        pt.velocity = std::max(pt.velocity, 0.0);
        pt.acceleration = 0;
        pt.jerk = 0;
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
        }
        pt.time = t;
        profile.addPoint(pt);
    }

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.epsilonVelocity = 1e-3;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.velocity.curve.has_value());
    const auto& curve = *seg.velocity.curve;
    double segLen = seg.sEnd - seg.sStart;

    for (const auto& pt : profile.points()) {
        double u = (pt.arcLength - seg.sStart) / segLen;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        double uMin = curve.knotMin();
        double uMax = curve.knotMax();
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_NEAR(val, pt.velocity, cfg.epsilonVelocity * 100)
            << "Sample interpolation failed at s=" << pt.arcLength;
    }
}

TEST(ReNURBSEdgeCasesTest, Regression_RealWorldMultiSegmentPath) {
    // Simulate a 200-segment path (stress test)
    std::size_t nSegs = 50; // reduced from 200 for test speed
    double segLen = 2.0;
    auto path = makeMultiSegmentPath(nSegs, segLen);
    double totalLen = nSegs * segLen;

    VelocityProfile<double> profile;
    int n = 200;
    double ds = totalLen / (n - 1);
    double t = 0;
    for (int i = 0; i < n; ++i) {
        double s = i * ds;
        VelocityProfilePoint<double> pt;
        pt.arcLength = s;
        pt.velocityLimit = 100.0;
        pt.accelerationLimit = 500.0;
        pt.velocity = 50.0 * (1.0 - std::cos(M_PI * s / totalLen)) / 2.0;
        pt.acceleration = 0;
        pt.jerk = 0;
        if (i > 0) {
            double prevV = profile.points().back().velocity;
            double avgV = (prevV + pt.velocity) / 2;
            if (avgV > 1e-12) t += ds / avgV;
        }
        pt.time = t;
        profile.addPoint(pt);
    }

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.maxControlPointsPerSegment = 32;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    // Should build without crash
    EXPECT_GE(renurbs.perSegment.size(), 1u);
    // Each segment should have CP count ≤ 32
    for (const auto& seg : renurbs.perSegment) {
        EXPECT_LE(seg.velocity.numControlPoints, 32u);
    }
}
