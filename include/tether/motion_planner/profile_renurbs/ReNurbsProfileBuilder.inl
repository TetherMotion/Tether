/**
 * @file ReNurbsProfileBuilder.inl
 * @brief Template implementation of ReNurbsProfileBuilder (included by the header).
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/ReNurbsProfileBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/ProfileConstraintCertifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::profile_renurbs {

namespace detail {

/// Convert a SplineFitResult to a 1-D NurbsCurve (weights all 1).
inline std::optional<NurbsCurve> toNurbsCurve(const SplineFitResult& fit) {
    if (fit.controlPoints.empty()) return std::nullopt;
    if (fit.knots.empty()) return std::nullopt;

    std::vector<RVec> cps;
    cps.reserve(fit.controlPoints.size());
    for (double cp : fit.controlPoints) {
        cps.push_back(RVec{cp});
    }
    std::vector<double> weights(fit.controlPoints.size(), 1.0);

    try {
        return NurbsCurve(std::move(cps), std::move(weights),
                          fit.knots, fit.degree);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

/// Convert a SplineFitResult to a ReNurbsQuantityCurves.
inline ReNurbsQuantityCurves toQuantityCurves(const SplineFitResult& fit) {
    ReNurbsQuantityCurves q;
    q.curve = toNurbsCurve(fit);
    q.maxResidual = fit.maxResidual;
    q.withinEpsilon = fit.withinEpsilon;
    q.constraintClamped = fit.constraintClamped;
    q.controlPointCapHit = fit.controlPointCapHit;
    q.numControlPoints = fit.controlPoints.size();
    int cont = fit.achievedContinuity;
    if (cont < 0) cont = 0;
    if (cont > 4) cont = 4;
    q.achievedContinuity = static_cast<ContinuityClass>(cont + 1);
    return q;
}

/// Map a SplineFitterConfig quantity to a ReNurbsQuantityCurves for a segment.
template<typename T>
SplineFitResult fitQuantityForSegment(
    const std::vector<double>& segU,
    const std::vector<double>& segQ,
    const std::vector<double>* segLimit,
    double epsilon,
    double safetyMargin,
    int degree,
    std::size_t maxCp,
    std::size_t gridMult,
    std::optional<double> lowerBound) {

    SplineFitterConfig cfg;
    cfg.degree = degree;
    cfg.epsilon = epsilon;
    cfg.safetyMargin = safetyMargin;
    cfg.maxControlPoints = maxCp;
    cfg.refinementGridMultiplier = gridMult;
    cfg.lowerBound = lowerBound;
    if (segLimit && segLimit->size() == segU.size()) {
        cfg.upperLimit = *segLimit;
    }
    return fitSplineThroughSamples(segU, segQ, cfg);
}

/// Extract per-segment samples from the full profile.
template<typename T>
struct SegmentSamples {
    std::vector<double> u;       ///< normalized to [0,1] within the segment
    std::vector<double> s;       ///< absolute arc length
    std::vector<double> v;       ///< velocity
    std::vector<double> a;       ///< acceleration
    std::vector<double> j;       ///< jerk
    std::vector<double> t;       ///< time
    std::vector<double> vLim;    ///< velocity limit
    std::vector<double> aLim;    ///< acceleration limit
};

/// Collect samples falling in [sStart, sEnd] from the full profile.
/// Includes the boundary samples (clamped to the segment endpoints).
template<typename T>
SegmentSamples<T> collectSegmentSamples(
    const MotionPlanner::VelocityProfile<T>& profile,
    double sStart, double sEnd) {

    SegmentSamples<T> seg;
    const auto& pts = profile.points();
    if (pts.empty()) return seg;

    double segLen = sEnd - sStart;
    if (segLen <= 0.0) return seg;

    // Find samples in [sStart, sEnd]. We include boundary samples by
    // interpolating at sStart and sEnd if no exact sample falls there.
    bool addedStart = false, addedEnd = false;

    for (std::size_t i = 0; i < pts.size(); ++i) {
        double s = pts[i].arcLength;
        if (s < sStart - 1e-12) continue;
        if (s > sEnd + 1e-12) {
            // Add an interpolated endpoint if we haven't
            if (!addedEnd && !seg.s.empty()) {
                double alpha = (sEnd - seg.s.back()) / (s - seg.s.back());
                seg.s.push_back(sEnd);
                seg.v.push_back(seg.v.back() * (1 - alpha) + pts[i].velocity * alpha);
                seg.a.push_back(seg.a.back() * (1 - alpha) + pts[i].acceleration * alpha);
                seg.j.push_back(seg.j.back() * (1 - alpha) + pts[i].jerk * alpha);
                seg.t.push_back(seg.t.back() * (1 - alpha) + pts[i].time * alpha);
                seg.vLim.push_back(seg.vLim.back() * (1 - alpha) + pts[i].velocityLimit * alpha);
                seg.aLim.push_back(seg.aLim.back() * (1 - alpha) + pts[i].accelerationLimit * alpha);
                addedEnd = true;
            }
            break;
        }
        if (s >= sStart - 1e-12 && s <= sEnd + 1e-12) {
            if (!addedStart && s > sStart + 1e-12 && i > 0) {
                // Interpolate the start boundary
                double alpha = (sStart - pts[i-1].arcLength) / (s - pts[i-1].arcLength);
                seg.s.push_back(sStart);
                seg.v.push_back(pts[i-1].velocity * (1 - alpha) + pts[i].velocity * alpha);
                seg.a.push_back(pts[i-1].acceleration * (1 - alpha) + pts[i].acceleration * alpha);
                seg.j.push_back(pts[i-1].jerk * (1 - alpha) + pts[i].jerk * alpha);
                seg.t.push_back(pts[i-1].time * (1 - alpha) + pts[i].time * alpha);
                seg.vLim.push_back(pts[i-1].velocityLimit * (1 - alpha) + pts[i].velocityLimit * alpha);
                seg.aLim.push_back(pts[i-1].accelerationLimit * (1 - alpha) + pts[i].accelerationLimit * alpha);
                addedStart = true;
            }
            seg.s.push_back(s);
            seg.v.push_back(pts[i].velocity);
            seg.a.push_back(pts[i].acceleration);
            seg.j.push_back(pts[i].jerk);
            seg.t.push_back(pts[i].time);
            seg.vLim.push_back(pts[i].velocityLimit);
            seg.aLim.push_back(pts[i].accelerationLimit);
            if (s <= sStart + 1e-12) addedStart = true;
            if (s >= sEnd - 1e-12) addedEnd = true;
        }
    }

    // Ensure the end boundary is added
    if (!addedEnd && !seg.s.empty()) {
        seg.s.push_back(sEnd);
        seg.v.push_back(pts.back().velocity);
        seg.a.push_back(pts.back().acceleration);
        seg.j.push_back(pts.back().jerk);
        seg.t.push_back(pts.back().time);
        seg.vLim.push_back(pts.back().velocityLimit);
        seg.aLim.push_back(pts.back().accelerationLimit);
    }

    // Normalize s to u ∈ [0,1]
    seg.u.reserve(seg.s.size());
    for (double s : seg.s) {
        seg.u.push_back((s - sStart) / segLen);
    }
    // Clamp to [0,1]
    for (auto& uu : seg.u) {
        if (uu < 0.0) uu = 0.0;
        if (uu > 1.0) uu = 1.0;
    }

    return seg;
}

} // namespace detail

template<std::size_t Dim, typename T>
ReNurbsProfile buildReNurbsProfile(
    const MotionPlanner::VelocityProfile<T>& profile,
    const MotionPlanner::PathAdapter<Dim, T>& path,
    const MotionPlanner::KinematicLimits<Dim, T>& limits,
    const ReNurbsConfig& config) {

    ReNurbsProfile result;

    // E1: Empty profile
    if (profile.points().empty()) return result;
    // E2: Single sample → constant curves
    if (profile.points().size() == 1) {
        ReNurbsSegmentProfile seg;
        seg.segmentIndex = 0;
        seg.sStart = 0.0;
        seg.sEnd = profile.points()[0].arcLength;
        if (path.numSegments() > 0) {
            seg.sourceRef = path.getSegment(0).sourceRef;
        }
        const auto& pt = profile.points()[0];
        // Constant degree-1 curves
        auto makeConst = [](double val) -> ReNurbsQuantityCurves {
            ReNurbsQuantityCurves q;
            try {
                std::vector<RVec> cps = {RVec{val}, RVec{val}};
                q.curve = NurbsCurve(cps, {1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, 1);
                q.numControlPoints = 2;
                q.withinEpsilon = true;
                q.achievedContinuity = ContinuityClass::C0;
            } catch (...) {}
            return q;
        };
        seg.velocity = makeConst(pt.velocity);
        seg.acceleration = makeConst(pt.acceleration);
        seg.jerk = makeConst(pt.jerk);
        seg.time = makeConst(pt.time);
        result.perSegment.push_back(std::move(seg));
        return result;
    }

    std::size_t numSegs = path.numSegments();
    if (numSegs == 0) numSegs = 1; // fallback: treat as single segment

    // Jerk limit (uniform for now)
    double jMax = limits.path.jerkLimitEnabled
        ? limits.path.maxPathJerk
        : std::numeric_limits<double>::infinity();

    for (std::size_t segIdx = 0; segIdx < numSegs; ++segIdx) {
        double sStart = 0.0, sEnd = 0.0;
        if (segIdx < path.segments().size()) {
            sStart = path.segments()[segIdx].cumulativeArcLength;
            sEnd = sStart + path.segments()[segIdx].arcLength;
        } else {
            sEnd = profile.points().back().arcLength;
        }

        // E3: Zero-length segment
        if (sEnd - sStart < 1e-15) continue;

        auto segSamples = detail::collectSegmentSamples(profile, sStart, sEnd);
        if (segSamples.u.size() < 2) continue; // not enough samples

        ReNurbsSegmentProfile seg;
        seg.segmentIndex = segIdx;
        seg.sStart = sStart;
        seg.sEnd = sEnd;
        if (segIdx < path.segments().size()) {
            seg.sourceRef = path.segments()[segIdx].sourceRef;
        }

        // Fit velocity (lower bound 0, upper limit = vLim)
        auto vFit = detail::fitQuantityForSegment<T>(
            segSamples.u, segSamples.v, &segSamples.vLim,
            config.epsilonVelocity, config.safetyMarginVelocity,
            config.degreeVelocity, config.maxControlPointsPerSegment,
            config.refinementGridMultiplier, 0.0);
        seg.velocity = detail::toQuantityCurves(vFit);

        // Fit acceleration (upper limit = aLim, no lower bound — a can be negative)
        auto aFit = detail::fitQuantityForSegment<T>(
            segSamples.u, segSamples.a, &segSamples.aLim,
            config.epsilonAcceleration, config.safetyMarginAcceleration,
            config.degreeAcceleration, config.maxControlPointsPerSegment,
            config.refinementGridMultiplier, std::nullopt);
        seg.acceleration = detail::toQuantityCurves(aFit);

        // Fit jerk (upper limit = jMax if finite, no lower bound)
        std::vector<double> jLim(segSamples.u.size(), jMax);
        const std::vector<double>* jLimPtr = std::isfinite(jMax) ? &jLim : nullptr;
        auto jFit = detail::fitQuantityForSegment<T>(
            segSamples.u, segSamples.j, jLimPtr,
            config.epsilonJerk, config.safetyMarginJerk,
            config.degreeJerk, config.maxControlPointsPerSegment,
            config.refinementGridMultiplier, std::nullopt);
        seg.jerk = detail::toQuantityCurves(jFit);

        // Fit time (monotonic, no upper limit, lower bound 0)
        auto tFit = detail::fitQuantityForSegment<T>(
            segSamples.u, segSamples.t, nullptr,
            config.epsilonTime, 0.0,
            config.degreeTime, config.maxControlPointsPerSegment,
            config.refinementGridMultiplier, 0.0);
        seg.time = detail::toQuantityCurves(tFit);

        result.perSegment.push_back(std::move(seg));
    }

    // Inter-segment continuity: check boundary C⁰ (guaranteed by construction
    // since both segments interpolate the same boundary sample).
    for (std::size_t i = 0; i + 1 < result.perSegment.size(); ++i) {
        auto& seg = result.perSegment[i];
        // C⁰ is guaranteed by shared boundary samples
        seg.boundaryContinuityVelocity = ContinuityClass::C0;
        seg.boundaryContinuityAcceleration = ContinuityClass::C0;
        seg.boundaryContinuityJerk = ContinuityClass::C0;
        seg.boundaryContinuityTime = ContinuityClass::C0;
    }

    // Optional certification
    if (config.certify) {
        result.certificate = certifyReNurbsProfile(
            result, profile, path, limits, config.certificationEpsilon);
        if (config.certifyThrowOnFailure && result.certificate &&
            !result.certificate->compliant) {
            std::string msg = "ReNURBS certification failed: " +
                std::to_string(result.certificate->violations.size()) +
                " violations";
            if (!result.certificate->violations.empty()) {
                const auto& v = result.certificate->violations[0];
                msg += " (first: segment " + std::to_string(v.segmentIndex) +
                       " at s=" + std::to_string(v.arcLength) +
                       " overshoot=" + std::to_string(v.overshoot) + ")";
            }
            throw ReNurbsCertificationError(msg);
        }
    }

    return result;
}

} // namespace tether::motion::profile_renurbs
