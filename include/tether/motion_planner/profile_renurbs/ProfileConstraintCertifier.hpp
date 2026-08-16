/**
 * @file ProfileConstraintCertifier.hpp
 * @brief Lipschitz-based certification of ReNURBS profile constraint
 *        preservation and interpolation accuracy.
 *
 * @details
 * See ReNURBS.md §9. This certifier mirrors the existing Lipschitz-
 * certification pattern (DeviationCertifier, CertifiedCurvatureSampler):
 *
 * 1. **Interpolation**: checks |q_NURBS(s_i) − q_sample(s_i)| ≤ ε at every
 *    sample (pointwise, exact).
 * 2. **Constraint preservation**: checks q_NURBS(s) ≤ q_lim(s) for *all* s
 *    in each segment, using a dense grid + a Lipschitz bound on the NURBS
 *    curve to certify between grid points.
 * 3. **Continuity**: reports the achieved continuity class per segment and
 *    per boundary.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/ReNURBSProfile.hpp"
#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/PathAdapter.hpp"

namespace tether::motion::profile_renurbs {

/**
 * @brief Certify a ReNURBSProfile against the original sampled profile.
 *
 * @tparam Dim Spatial dimension.
 * @tparam T   Numeric type.
 * @param renurbs The ReNURBS profile to certify.
 * @param profile The original sampled velocity profile (ground truth).
 * @param path The path adapter (for segment boundaries).
 * @param limits Kinematic limits (for jerk limit).
 * @param epsilon Lipschitz certificate width goal.
 * @return The certificate (compliant flag, violations, continuity reports).
 */
template<std::size_t Dim, typename T = double>
ProfileConstraintCertificate certifyReNURBSProfile(
    const ReNURBSProfile& renurbs,
    const MotionPlanner::VelocityProfile<T>& profile,
    const MotionPlanner::PathAdapter<Dim, T>& path,
    const MotionPlanner::KinematicLimits<Dim, T>& limits,
    double epsilon = 1e-5);

} // namespace tether::motion::profile_renurbs

// Include the template implementation.
#include "tether/motion_planner/profile_renurbs/ProfileConstraintCertifier.inl"
