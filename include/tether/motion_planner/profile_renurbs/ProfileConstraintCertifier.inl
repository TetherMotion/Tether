/**
 * @file ProfileConstraintCertifier.inl
 * @brief Template implementation of ProfileConstraintCertifier.
 *
 * @details
 * This is now a thin adapter that delegates to the generic certifier.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/ProfileConstraintCertifier.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNurbsCertifier.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNurbsBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/ReNurbsProfileBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::profile_renurbs {

template<std::size_t Dim, typename T>
ProfileConstraintCertificate certifyReNurbsProfile(
    const ReNurbsProfile& renurbs,
    const MotionPlanner::VelocityProfile<T>& profile,
    const MotionPlanner::PathAdapter<Dim, T>& path,
    const MotionPlanner::KinematicLimits<Dim, T>& limits,
    double epsilon) {

    // Convert to generic format
    ReNurbsConfig defaultCfg; // use defaults for quantity specs
    auto genericConfig = detail::toGenericConfig<Dim, T>(defaultCfg, limits);
    genericConfig.certificationEpsilon = epsilon;

    std::vector<GenericSample> samples;
    std::vector<SegmentInfo> segments;
    detail::toGenericSamples(profile, path, samples, segments);

    // Convert the ReNurbsProfile to a GenericReNurbsProfile
    auto genericProfile = fromVelocityProfile(renurbs);

    // Certify using the generic certifier
    auto genericCert = certifyGenericReNurbsProfile(
        genericProfile, samples, segments, genericConfig, epsilon);

    // Convert back to velocity-specific format
    return toVelocityCertificate(genericCert);
}

} // namespace tether::motion::profile_renurbs
