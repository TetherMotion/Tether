/**
 * @file ReNURBSProfileBuilder.hpp
 * @brief Builds a ReNURBSProfile from a sampled VelocityProfile + PathAdapter.
 *
 * @details
 * See ReNURBS.md §3–§5 for the full design. The builder:
 *
 * 1. Partitions the sampled VelocityProfile by PathAdapter segment.
 * 2. For each segment and each quantity (v, a, j, t), fits an adaptive
 *    B-spline through the samples using ProfileSplineFitter.
 * 3. Applies convex-hull constraint clamping so the NURBS never violates
 *    the limits stored in VelocityProfilePoint (velocityLimit, accelerationLimit).
 * 4. Enforces inter-segment continuity (C⁰ minimum; C¹+ where the source
 *    profile supports it).
 * 5. Optionally runs ProfileConstraintCertifier to certify the result.
 *
 * The builder is API-optional: if ReNURBSConfig::enabled is false (default),
 * MotionPlanBuilder skips it entirely.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/ReNURBSProfile.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNURBSProfile.hpp"
#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"
#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/PathAdapter.hpp"

#include <optional>

namespace tether::motion::profile_renurbs {

// ReNURBSCertificationError is now defined in GenericReNURBSProfile.hpp
// and re-exported here via the include above.

/**
 * @brief Build a ReNURBSProfile from a sampled velocity profile and path.
 *
 * @tparam Dim Spatial dimension of the path (2 or 3).
 * @tparam T   Numeric type (default: double).
 * @param profile The velocity profile (sampled or analytical) to fit.
 * @param path The path adapter (provides segment boundaries + source refs).
 * @param limits Kinematic limits (for jerk limit, if jerk-constrained).
 * @param config ReNURBS configuration.
 * @return The ReNURBS profile with per-segment NURBS curves + optional certificate.
 */
template<std::size_t Dim, typename T = double>
ReNURBSProfile buildReNURBSProfile(
    const MotionPlanner::VelocityProfile& profile,
    const MotionPlanner::PathAdapter<Dim, T>& path,
    const MotionPlanner::KinematicLimits<Dim, T>& limits,
    const ReNURBSConfig& config = {});

} // namespace tether::motion::profile_renurbs

// Include the template implementation.
#include "tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.inl"
