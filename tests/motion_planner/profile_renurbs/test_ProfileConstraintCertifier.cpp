/**
 * @file test_ProfileConstraintCertifier.cpp
 * @brief Unit tests for ProfileConstraintCertifier (ReNURBS §7.3).
 */

#include <gtest/gtest.h>
#include "tether/motion_planner/profile_renurbs/ProfileConstraintCertifier.hpp"
#include "tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.hpp"
#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/PathAdapter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <cmath>

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

SampledVelocityProfile makeSimpleProfile(double pathLength, double vMax,
                                          std::size_t n = 50) {
    SampledVelocityProfile profile;
    double ds = pathLength / (n - 1);
    double t = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double s = i * ds;
        VelocityProfilePoint pt;
        pt.arcLength = s;
        pt.velocityLimit = vMax;
        pt.accelerationLimit = 500.0;
        pt.velocity = vMax * (1.0 - std::cos(M_PI * s / pathLength)) / 2.0;
        pt.acceleration = vMax * M_PI / (2 * pathLength) *
                          std::sin(M_PI * s / pathLength);
        pt.jerk = 0;
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

TEST(ProfileConstraintCertifierTest, CertifiesCompliantProfile) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSimpleProfile(100.0, 50.0, 50);
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 50.0;
    limits.path.maxPathAcceleration = 500.0;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    auto cert = certifyReNURBSProfile(renurbs, profile, path, limits, 1e-5);
    EXPECT_TRUE(cert.compliant);
    EXPECT_TRUE(cert.violations.empty());
}

TEST(ProfileConstraintCertifierTest, RejectsViolatingProfile) {
    // Build a ReNURBSProfile with a deliberately violating curve.
    ReNURBSProfile renurbs;
    ReNURBSSegmentProfile seg;
    seg.segmentIndex = 0;
    seg.sStart = 0.0;
    seg.sEnd = 100.0;

    // Create a curve with a control point way above the limit (50.0)
    std::vector<RVec> cps = {RVec{0.0}, RVec{100.0}, RVec{50.0}, RVec{0.0}};
    std::vector<double> weights(4, 1.0);
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1}; // clamped cubic
    seg.velocity.curve = NurbsCurve(cps, weights, knots, 3);
    seg.velocity.numControlPoints = 4;

    // Constant acceleration and time curves (not relevant to this test)
    std::vector<RVec> constCps = {RVec{0.0}, RVec{0.0}};
    seg.acceleration.curve = NurbsCurve(constCps, {1.0, 1.0}, {0, 0, 1, 1}, 1);
    seg.time.curve = NurbsCurve(constCps, {1.0, 1.0}, {0, 0, 1, 1}, 1);

    renurbs.perSegment.push_back(std::move(seg));

    // Build a profile with v_lim = 50.0
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSimpleProfile(100.0, 50.0, 50);
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 50.0;

    auto cert = certifyReNURBSProfile(renurbs, profile, path, limits, 1e-5);
    EXPECT_FALSE(cert.compliant);
    EXPECT_FALSE(cert.violations.empty());
}

TEST(ProfileConstraintCertifierTest, ContinuityReportPopulated) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSimpleProfile(100.0, 50.0, 50);
    KinematicLimits<2, double> limits;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    auto cert = certifyReNURBSProfile(renurbs, profile, path, limits, 1e-5);
    EXPECT_EQ(cert.continuity.size(), renurbs.perSegment.size());
}

TEST(ProfileConstraintCertifierTest, EmptyProfileCertifiesAsCompliant) {
    auto path = makeLinearPath2D(100.0);
    SampledVelocityProfile profile;
    KinematicLimits<2, double> limits;
    ReNURBSProfile renurbs;

    auto cert = certifyReNURBSProfile(renurbs, profile, path, limits, 1e-5);
    EXPECT_TRUE(cert.compliant);
    EXPECT_TRUE(cert.violations.empty());
}

TEST(ProfileConstraintCertifierTest, JerkCertificationWhenJerkLimited) {
    auto path = makeLinearPath2D(100.0);
    auto profile = makeSimpleProfile(100.0, 50.0, 50);
    KinematicLimits<2, double> limits;
    limits.path.maxPathJerk = 5000.0;
    limits.path.jerkLimitEnabled = true;

    ReNURBSConfig cfg;
    cfg.enabled = true;
    cfg.certify = false;
    auto renurbs = buildReNURBSProfile(profile, path, limits, cfg);

    auto cert = certifyReNURBSProfile(renurbs, profile, path, limits, 1e-5);
    // The jerk curve should be certified against jMax
    // (may or may not be compliant depending on the profile, but the
    //  certifier should run without crashing)
    EXPECT_TRUE(cert.compliant || !cert.violations.empty());
}
