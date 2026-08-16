/**
 * @file GenericReNURBSCertifier.hpp
 * @brief Generic Lipschitz-based certifier for any ReNURBS profile.
 *
 * @details
 * This certifier works with the generic GenericReNURBSProfile, checking:
 *
 * 1. **Interpolation**: |q_NURBS(p_i) − q_sample(p_i)| ≤ ε at every sample.
 * 2. **Constraint preservation**: q_NURBS(p) ≤ q_lim(p) for all p in each
 *    segment, using a dense grid.
 * 3. **Continuity**: reports achieved continuity class per segment boundary.
 *
 * It is domain-agnostic — it doesn't know what the quantities represent
 * physically. The constraint checking logic is driven by the LimitType
 * in each QuantitySpec.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/GenericReNURBSProfile.hpp"

#include <vector>

namespace tether::motion::profile_renurbs {

/**
 * @brief Certify a generic ReNURBS profile against the original samples.
 *
 * @param profile The generic ReNURBS profile to certify.
 * @param samples The original sampled data (ground truth).
 * @param segments Segment boundaries (for parameter mapping).
 * @param config Configuration (contains QuantitySpec with LimitType info).
 * @param epsilon Lipschitz certificate width goal.
 * @return The certificate (compliant flag, violations, continuity reports).
 */
GenericCertificate certifyGenericReNURBSProfile(
    const GenericReNURBSProfile& profile,
    const std::vector<GenericSample>& samples,
    const std::vector<SegmentInfo>& segments,
    const GenericReNURBSConfig& config,
    double epsilon = 1e-5);

} // namespace tether::motion::profile_renurbs
