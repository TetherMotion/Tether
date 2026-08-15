/**
 * @file ProfileConstraintCertifier.inl
 * @brief Template implementation of ProfileConstraintCertifier.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/ProfileConstraintCertifier.hpp"
#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::profile_renurbs {

namespace detail {

/// Evaluate a 1-D NurbsCurve at parameter u, returning the scalar value.
inline double evalScalarCurve(const std::optional<NurbsCurve>& curve, double u) {
    if (!curve) return 0.0;
    const double uMin = curve->knotMin();
    const double uMax = curve->knotMax();
    double uu = u;
    if (uu < uMin) uu = uMin;
    if (uu > uMax) uu = uMax;
    return curve->evaluate(uu)[0];
}

/// Check a quantity curve against its limit on a dense grid.
inline std::vector<SegmentViolation> checkConstraintOnGrid(
    const ReNurbsSegmentProfile& seg,
    const std::optional<NurbsCurve>& curve,
    SegmentViolation::Quantity quantity,
    const std::vector<double>& sampleU,
    const std::vector<double>& sampleLimit,
    double safetyMargin,
    std::size_t gridPoints = 200) {

    std::vector<SegmentViolation> violations;
    if (!curve) return violations;
    if (sampleLimit.empty()) return violations;

    double sStart = seg.sStart;
    double sEnd = seg.sEnd;
    double segLen = sEnd - sStart;

    for (std::size_t k = 0; k <= gridPoints; ++k) {
        double u = static_cast<double>(k) / gridPoints;
        double val = evalScalarCurve(curve, u);

        // Interpolate the limit at this u
        double lim = 0.0;
        if (u <= sampleU.front()) lim = sampleLimit.front();
        else if (u >= sampleU.back()) lim = sampleLimit.back();
        else {
            auto it = std::lower_bound(sampleU.begin(), sampleU.end(), u);
            int idx = static_cast<int>(it - sampleU.begin());
            if (idx == 0) { lim = sampleLimit[0]; }
            else {
                double alpha = (u - sampleU[idx-1]) /
                               (sampleU[idx] - sampleU[idx-1]);
                lim = sampleLimit[idx-1] * (1.0 - alpha) +
                      sampleLimit[idx] * alpha;
            }
        }

        double effectiveLimit = lim - safetyMargin;
        if (val > effectiveLimit + 1e-10) {
            SegmentViolation v;
            v.segmentIndex = seg.segmentIndex;
            v.quantity = quantity;
            v.arcLength = sStart + u * segLen;
            v.value = val;
            v.limit = lim;
            v.overshoot = val - effectiveLimit;
            violations.push_back(v);
        }
    }
    return violations;
}

/// Collect samples for a segment from the profile (inline, used by certifier).
template<typename T>
struct CertSegmentSamples {
    std::vector<double> u;
    std::vector<double> vLim;
    std::vector<double> aLim;
};

template<typename T>
CertSegmentSamples<T> collectCertSamples(
    const MotionPlanner::VelocityProfile<T>& profile,
    double sStart, double sEnd) {

    CertSegmentSamples<T> seg;
    const auto& pts = profile.points();
    if (pts.empty()) return seg;

    double segLen = sEnd - sStart;
    if (segLen <= 0.0) return seg;

    for (std::size_t i = 0; i < pts.size(); ++i) {
        double s = pts[i].arcLength;
        if (s < sStart - 1e-12) continue;
        if (s > sEnd + 1e-12) break;
        double u = (s - sStart) / segLen;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        seg.u.push_back(u);
        seg.vLim.push_back(pts[i].velocityLimit);
        seg.aLim.push_back(pts[i].accelerationLimit);
    }
    return seg;
}

} // namespace detail

template<std::size_t Dim, typename T>
ProfileConstraintCertificate certifyReNurbsProfile(
    const ReNurbsProfile& renurbs,
    const MotionPlanner::VelocityProfile<T>& profile,
    const MotionPlanner::PathAdapter<Dim, T>& /*path*/,
    const MotionPlanner::KinematicLimits<Dim, T>& limits,
    double /*epsilon*/) {

    ProfileConstraintCertificate cert;
    cert.compliant = true;

    double jMax = limits.path.jerkLimitEnabled
        ? limits.path.maxPathJerk
        : std::numeric_limits<double>::infinity();

    for (const auto& seg : renurbs.perSegment) {
        auto segSamples = detail::collectCertSamples<T>(profile, seg.sStart, seg.sEnd);
        if (segSamples.u.empty()) continue;

        // Check velocity constraint
        auto vViol = detail::checkConstraintOnGrid(
            seg, seg.velocity.curve, SegmentViolation::Quantity::Velocity,
            segSamples.u, segSamples.vLim, 1e-4);
        cert.violations.insert(cert.violations.end(), vViol.begin(), vViol.end());

        // Check acceleration constraint
        auto aViol = detail::checkConstraintOnGrid(
            seg, seg.acceleration.curve, SegmentViolation::Quantity::Acceleration,
            segSamples.u, segSamples.aLim, 1e-3);
        cert.violations.insert(cert.violations.end(), aViol.begin(), aViol.end());

        // Check jerk constraint (if jerk-limited)
        if (std::isfinite(jMax)) {
            std::vector<double> jLim(segSamples.u.size(), jMax);
            auto jViol = detail::checkConstraintOnGrid(
                seg, seg.jerk.curve, SegmentViolation::Quantity::Jerk,
                segSamples.u, jLim, 1e-2);
            cert.violations.insert(cert.violations.end(), jViol.begin(), jViol.end());
        }

        // Check control-point-cap exhaustion
        if (seg.velocity.controlPointCapHit ||
            seg.acceleration.controlPointCapHit ||
            seg.jerk.controlPointCapHit ||
            seg.time.controlPointCapHit) {
            cert.residualBudgetExhausted = true;
        }

        // Continuity report
        ContinuityReport cr;
        cr.segmentIndex = seg.segmentIndex;
        cr.velocity = seg.boundaryContinuityVelocity;
        cr.acceleration = seg.boundaryContinuityAcceleration;
        cr.jerk = seg.boundaryContinuityJerk;
        cr.time = seg.boundaryContinuityTime;
        cert.continuity.push_back(cr);
    }

    cert.compliant = cert.violations.empty() && !cert.residualBudgetExhausted;
    return cert;
}

} // namespace tether::motion::profile_renurbs
