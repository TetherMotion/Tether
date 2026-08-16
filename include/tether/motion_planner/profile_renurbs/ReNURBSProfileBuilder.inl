/**
 * @file ReNURBSProfileBuilder.inl
 * @brief Template implementation of ReNURBSProfileBuilder.
 *
 * @details
 * This is now a thin adapter that converts the velocity-specific inputs
 * (VelocityProfile, PathAdapter, KinematicLimits) into the generic
 * SampledCurve format and delegates to buildGenericReNURBSProfile.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNURBSBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNURBSCertifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::profile_renurbs {

namespace detail {

/// Convert a ReNURBSConfig to a GenericReNURBSConfig with 4 quantities
/// (velocity, acceleration, jerk, time).
template<std::size_t Dim, typename T>
GenericReNURBSConfig toGenericConfig(
    const ReNURBSConfig& cfg,
    const MotionPlanner::KinematicLimits<Dim, T>& limits) {

    GenericReNURBSConfig gc;
    gc.enabled = cfg.enabled;
    gc.maxControlPointsPerSegment = cfg.maxControlPointsPerSegment;
    gc.refinementGridMultiplier = cfg.refinementGridMultiplier;
    gc.certify = cfg.certify;
    gc.certificationEpsilon = cfg.certificationEpsilon;
    gc.certifyThrowOnFailure = cfg.certifyThrowOnFailure;

    // Jerk limit (uniform)
    double jMax = limits.path.jerkLimitEnabled
        ? limits.path.maxPathJerk
        : std::numeric_limits<double>::infinity();

    // Quantity 0: velocity
    {
        QuantitySpec qs;
        qs.name = "velocity";
        qs.epsilon = cfg.epsilonVelocity;
        qs.safetyMargin = cfg.safetyMarginVelocity;
        qs.degree = cfg.degreeVelocity;
        qs.lowerBound = 0.0;
        qs.limitType = LimitType::UpperPerSample;
        gc.quantities.push_back(qs);
    }
    // Quantity 1: acceleration
    {
        QuantitySpec qs;
        qs.name = "acceleration";
        qs.epsilon = cfg.epsilonAcceleration;
        qs.safetyMargin = cfg.safetyMarginAcceleration;
        qs.degree = cfg.degreeAcceleration;
        qs.lowerBound = std::nullopt; // can be negative
        qs.limitType = LimitType::UpperPerSample;
        gc.quantities.push_back(qs);
    }
    // Quantity 2: jerk
    {
        QuantitySpec qs;
        qs.name = "jerk";
        qs.epsilon = cfg.epsilonJerk;
        qs.safetyMargin = cfg.safetyMarginJerk;
        qs.degree = cfg.degreeJerk;
        qs.lowerBound = std::nullopt;
        if (std::isfinite(jMax)) {
            qs.limitType = LimitType::SymmetricUniform;
            qs.uniformLimit = jMax;
        } else {
            qs.limitType = LimitType::None;
        }
        gc.quantities.push_back(qs);
    }
    // Quantity 3: time
    {
        QuantitySpec qs;
        qs.name = "time";
        qs.epsilon = cfg.epsilonTime;
        qs.safetyMargin = 0.0;
        qs.degree = cfg.degreeTime;
        qs.lowerBound = 0.0;
        qs.limitType = LimitType::None;
        gc.quantities.push_back(qs);
    }
    return gc;
}

/// Convert a VelocityProfile + PathAdapter into generic samples + segments.
template<std::size_t Dim, typename T>
void toGenericSamples(
    const MotionPlanner::VelocityProfile<T>& profile,
    const MotionPlanner::PathAdapter<Dim, T>& path,
    std::vector<GenericSample>& samples,
    std::vector<SegmentInfo>& segments) {

    const auto& pts = profile.points();
    samples.reserve(pts.size());
    for (const auto& pt : pts) {
        GenericSample s;
        s.parameter = pt.arcLength;
        s.quantities = {pt.velocity, pt.acceleration, pt.jerk, pt.time};
        s.limits = {pt.velocityLimit, pt.accelerationLimit,
                    std::numeric_limits<double>::infinity(),  // jerk limit is uniform
                    std::numeric_limits<double>::infinity()}; // time has no limit
        samples.push_back(s);
    }

    // Build segments from PathAdapter
    std::size_t numSegs = path.numSegments();
    for (std::size_t i = 0; i < numSegs; ++i) {
        SegmentInfo si;
        si.paramStart = path.segments()[i].cumulativeArcLength;
        si.paramEnd = si.paramStart + path.segments()[i].arcLength;
        si.sourceRef = path.segments()[i].sourceRef;
        segments.push_back(si);
    }
}

} // namespace detail

template<std::size_t Dim, typename T>
ReNURBSProfile buildReNURBSProfile(
    const MotionPlanner::VelocityProfile<T>& profile,
    const MotionPlanner::PathAdapter<Dim, T>& path,
    const MotionPlanner::KinematicLimits<Dim, T>& limits,
    const ReNURBSConfig& config) {

    // Convert to generic format
    auto genericConfig = detail::toGenericConfig<Dim, T>(config, limits);

    std::vector<GenericSample> samples;
    std::vector<SegmentInfo> segments;
    detail::toGenericSamples(profile, path, samples, segments);

    // Build using the generic builder
    auto genericProfile = buildGenericReNURBSProfile(samples, segments, genericConfig);

    // Convert back to velocity-specific format
    return toVelocityProfile(genericProfile);
}

} // namespace tether::motion::profile_renurbs
