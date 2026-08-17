/**
 * @file test_ReNURBSProfileBuilder.cpp
 * @brief End-to-end tests for ReNURBSProfileBuilder (ReNURBS §7.2).
 */

#include <gtest/gtest.h>
#include "tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/ReNURBSProfile.hpp"
#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/PathAdapter.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <cmath>
#include <random>

using namespace tether::motion::profile_renurbs;
using namespace MotionPlanner;
using tether::motion::NurbsCurve;
using tether::motion::RVec;
using tether::motion::PiecewiseNurbsPath;

namespace {

/// Build a simple single-segment linear path adapter.
PathAdapter<2, double> makeLinearPath2D(double length) {
    RVec a{0.0, 0.0};
    RVec b{length, 0.0};
    auto line = NurbsCurve::fromLine(a, b);
    PiecewiseNurbsPath path({line});
    return PathAdapter<2, double>(std::move(path), {SourceReference{}});
}

/// Build a multi-segment path (two lines).
PathAdapter<2, double> makeTwoSegmentPath2D(double len1, double len2) {
    RVec a{0.0, 0.0};
    RVec b{len1, 0.0};
    RVec c{len1, len2};
    auto line1 = NurbsCurve::fromLine(a, b);
    auto line2 = NurbsCurve::fromLine(b, c);
    PiecewiseNurbsPath path({line1, line2});
    return PathAdapter<2, double>(std::move(path),
        {SourceReference{}, SourceReference{}});
}

/// Build a trapezoidal velocity profile (accel, cruise, decel) over a path.
SampledVelocityProfile makeTrapezoidalProfile(
    double pathLength, double vMax, double aMax, std::size_t n = 100) {
    SampledVelocityProfile profile;
    double ds = pathLength / (n - 1);
    double t = 0.0;
    double v = 0.0;
    // Accel distance: vMax^2 / (2*aMax)
    double accelDist = (vMax * vMax) / (2.0 * aMax);
    double decelDist = accelDist;
    double cruiseDist = pathLength - 2 * accelDist;
    if (cruiseDist < 0) {
        // Triangle profile
        accelDist = pathLength / 2;
        decelDist = pathLength / 2;
        cruiseDist = 0;
    }
    for (std::size_t i = 0; i < n; ++i) {
        double s = i * ds;
        VelocityProfilePoint pt;
        pt.arcLength = s;
        pt.velocityLimit = vMax;
        pt.accelerationLimit = aMax;
        if (s < accelDist) {
            pt.velocity = std::sqrt(2 * aMax * s);
            pt.acceleration = aMax;
            pt.jerk = 0;
        } else if (s < accelDist + cruiseDist) {
            pt.velocity = vMax;
            pt.acceleration = 0;
            pt.jerk = 0;
        } else {
            double sDecel = s - accelDist - cruiseDist;
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

/// Build an S-curve-like profile (smooth jerk transitions).
SampledVelocityProfile makeSCurveProfile(
    double pathLength, double vMax, double aMax, double jMax,
    std::size_t n = 100) {
    SampledVelocityProfile profile;
    double ds = pathLength / (n - 1);
    double t = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double s = i * ds;
        double u = s / pathLength;
        VelocityProfilePoint pt;
        pt.arcLength = s;
        pt.velocityLimit = vMax;
        pt.accelerationLimit = aMax;
        // Smooth bell-shaped velocity profile
        pt.velocity = vMax * (1.0 - std::cos(M_PI * u)) / 2.0;
        pt.acceleration = vMax * M_PI * M_PI / (2 * pathLength) *
                          std::cos(M_PI * u);
        pt.jerk = -vMax * M_PI * M_PI * M_PI / (2 * pathLength * pathLength) *
                  std::sin(M_PI * u);
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
// Basic construction
// ============================================================================

TEST(ReNURBSProfileBuilderTest, BuildsFromTrapezoidalProfile) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 50.0;
    limits.path.maxPathAcceleration = 500.0;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false; // disable for basic test
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    const auto& seg = renurbs.perSegment[0];
    EXPECT_TRUE(seg.velocity.curve.has_value());
    EXPECT_TRUE(seg.acceleration.curve.has_value());
    EXPECT_TRUE(seg.time.curve.has_value());
}

TEST(ReNURBSProfileBuilderTest, BuildsFromSCurveProfile) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSCurveProfile(100.0, 50.0, 500.0, 5000.0, 100);
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 50.0;
    limits.path.maxPathAcceleration = 500.0;
    limits.path.maxPathJerk = 5000.0;
    limits.path.jerkLimitEnabled = true;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    const auto& seg = renurbs.perSegment[0];
    EXPECT_TRUE(seg.velocity.curve.has_value());
    EXPECT_TRUE(seg.jerk.curve.has_value());
}

TEST(ReNURBSProfileBuilderTest, PerSegmentCountMatchesPathPieces) {
    auto path = makeTwoSegmentPath2D(50.0, 50.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    EXPECT_EQ(renurbs.perSegment.size(), 2u);
}

// ============================================================================
// Interpolation
// ============================================================================

TEST(ReNURBSProfileBuilderTest, VelocityInterpolatesSamples) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSCurveProfile(100.0, 50.0, 500.0, 5000.0, 50);
    KinematicLimits<2, double> limits;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.epsilonVelocity = 1e-3;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.velocity.curve.has_value());
    const auto& curve = *seg.velocity.curve;
    double sStart = seg.sStart;
    double sEnd = seg.sEnd;
    double segLen = sEnd - sStart;

    for (const auto& pt : profile.points()) {
        double u = (pt.arcLength - sStart) / segLen;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        double uMin = curve.knotMin();
        double uMax = curve.knotMax();
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_NEAR(val, pt.velocity, cfg.epsilonVelocity * 100)
            << "At s=" << pt.arcLength;
    }
}

TEST(ReNURBSProfileBuilderTest, TimeInterpolatesSamples) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSCurveProfile(100.0, 50.0, 500.0, 5000.0, 50);
    KinematicLimits<2, double> limits;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.epsilonTime = 1e-4;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.time.curve.has_value());
    const auto& curve = *seg.time.curve;
    double segLen = seg.sEnd - seg.sStart;

    for (const auto& pt : profile.points()) {
        double u = (pt.arcLength - seg.sStart) / segLen;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        double uMin = curve.knotMin();
        double uMax = curve.knotMax();
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_NEAR(val, pt.time, cfg.epsilonTime * 100)
            << "At s=" << pt.arcLength;
    }
}

// ============================================================================
// Constraint preservation
// ============================================================================

TEST(ReNURBSProfileBuilderTest, VelocityNeverExceedsLimit) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 50.0;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    cfg.safetyMarginVelocity = 0.01;
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
        EXPECT_LE(val, 50.0 - cfg.safetyMarginVelocity + 1e-6)
            << "Velocity exceeds limit at u=" << u;
    }
}

TEST(ReNURBSProfileBuilderTest, VelocityNonNegative) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
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
        EXPECT_GE(val, -1e-6) << "Velocity negative at u=" << u;
    }
}

// ============================================================================
// Edge cases
// ============================================================================

TEST(ReNURBSProfileBuilderTest, EmptyProfileReturnsEmpty) {
    auto path = makeLinearPath2D(100.0);
    SampledVelocityProfile profile; // empty
    KinematicLimits<2, double> limits;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    EXPECT_TRUE(renurbs.empty());
}

TEST(ReNURBSProfileBuilderTest, SingleSampleProfileReturnsConstant) {
    auto path = makeLinearPath2D(100.0);
    SampledVelocityProfile profile;
    VelocityProfilePoint pt;
    pt.arcLength = 50.0;
    pt.velocity = 10.0;
    pt.acceleration = 0.0;
    pt.jerk = 0.0;
    pt.time = 5.0;
    profile.addPoint(pt);

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    ASSERT_EQ(renurbs.perSegment.size(), 1u);
    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.velocity.curve.has_value());
    double val = seg.velocity.curve->evaluate(
        seg.velocity.curve->knotMin())[0];
    EXPECT_NEAR(val, 10.0, 1e-10);
}

TEST(ReNURBSProfileBuilderTest, ZeroVelocityStartEnd) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    // Ensure start and end are at rest
    profile.points().front().velocity = 0.0;
    profile.points().back().velocity = 0.0;

    KinematicLimits<2, double> limits;
    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    const auto& seg = renurbs.perSegment[0];
    ASSERT_TRUE(seg.velocity.curve.has_value());
    const auto& curve = *seg.velocity.curve;
    double vStart = curve.evaluate(curve.knotMin())[0];
    double vEnd = curve.evaluate(curve.knotMax())[0];
    EXPECT_NEAR(vStart, 0.0, 1e-6);
    EXPECT_NEAR(vEnd, 0.0, 1e-6);
}

TEST(ReNURBSProfileBuilderTest, SourceRefPropagated) {
    auto path = makeTwoSegmentPath2D(50.0, 50.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);
    ASSERT_EQ(renurbs.perSegment.size(), 2u);
    // Source refs should match the path segments
    for (std::size_t i = 0; i < renurbs.perSegment.size(); ++i) {
        EXPECT_EQ(renurbs.perSegment[i].segmentIndex, i);
    }
}

TEST(ReNURBSProfileBuilderTest, TimeCurveMonotonic) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSCurveProfile(100.0, 50.0, 500.0, 5000.0, 50);
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
        double t = curve.evaluate(uu)[0];
        EXPECT_GE(t, prevT - 1e-9) << "Time not monotonic at k=" << k;
        prevT = t;
    }
}

// ============================================================================
// Certification integration
// ============================================================================

TEST(ReNURBSProfileBuilderTest, CertificationPassesForCompliantProfile) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 50.0;
    limits.path.maxPathAcceleration = 500.0;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = true;
    cfg.certifyThrowOnFailure = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    ASSERT_TRUE(renurbs.certificate.has_value());
    // The trapezoidal profile should be compliant (all samples within limits)
    EXPECT_TRUE(renurbs.certificate->compliant ||
                renurbs.certificate->violations.empty());
}

TEST(ReNURBSProfileBuilderTest, CertificationThrowsOnFailure) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeTrapezoidalProfile(100.0, 50.0, 500.0, 100);
    // Set an impossibly tight limit to force violations
    for (auto& pt : profile.points()) {
        pt.velocityLimit = 1.0; // much lower than the profile velocities
    }

    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 1.0;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = true;
    cfg.certifyThrowOnFailure = true;
    EXPECT_THROW(buildReNURBSProfile(profile, path, limits, cfg),
                 ReNURBSCertificationError);
}
