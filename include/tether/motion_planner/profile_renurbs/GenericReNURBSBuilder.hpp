/**
 * @file GenericReNURBSBuilder.hpp
 * @brief Generic ReNURBS builder — works with any sampled curve.
 *
 * @details
 * This builder converts a generic SampledCurve (a vector of GenericSample
 * + segment boundaries + per-quantity configuration) into a
 * GenericReNURBSProfile with per-segment NURBS curves.
 *
 * It is domain-agnostic: it doesn't know about velocity, acceleration,
 * pressure advance, or any specific physical quantity. The caller
 * provides the samples and the quantity specifications, and the builder
 * handles the B-spline fitting, adaptive refinement, constraint
 * clamping, and optional certification.
 *
 * ## Usage
 *
 * ```cpp
 * using namespace tether::motion::profile_renurbs;
 *
 * // 1. Prepare samples (e.g., from a pressure advance algorithm)
 * std::vector<GenericSample> samples;
 * for (int i = 0; i < n; ++i) {
 *     GenericSample s;
 *     s.parameter = i * dt;  // time
 *     s.quantities = {offset[i]};  // PA offset
 *     s.limits = {maxCompensation};  // ±maxCompensation
 *     samples.push_back(s);
 * }
 *
 * // 2. Configure quantities
 * GenericReNURBSConfig config;
 * config.enabled = true;
 * QuantitySpec qs;
 * qs.name = "pressure_offset";
 * qs.epsilon = 1e-6;
 * qs.degree = 5;
 * qs.limitType = LimitType::SymmetricUniform;
 * qs.uniformLimit = maxCompensation;
 * qs.safetyMargin = 1e-6;
 * config.quantities = {qs};
 *
 * // 3. Single segment covering the full range
 * std::vector<SegmentInfo> segments = {
 *     {0.0, (n-1) * dt, {}}
 * };
 *
 * // 4. Build
 * auto profile = buildGenericReNURBSProfile(samples, segments, config);
 * ```
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/GenericReNURBSProfile.hpp"
#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"

#include <vector>
#include <stdexcept>

namespace tether::motion::profile_renurbs {

/// Exception thrown when generic certification fails and
/// certifyThrowOnFailure is true.
/// Inherits from ReNURBSCertificationError so existing code catching
/// the velocity-specific exception type still works.
class GenericReNURBSCertificationError : public ReNURBSCertificationError {
public:
    explicit GenericReNURBSCertificationError(const std::string& msg)
        : ReNURBSCertificationError(msg) {}
};

/**
 * @brief Build a generic ReNURBS profile from any sampled curve.
 *
 * @param samples The sampled data (parameter + quantities + optional limits).
 * @param segments Segment boundaries (if empty, a single segment covering
 *                 the full parameter range is used).
 * @param config Configuration (quantity specs, refinement caps, certification).
 * @return The generic ReNURBS profile with per-segment NURBS curves.
 *
 * @throws std::invalid_argument if samples is empty or config.quantities
 *         is empty.
 * @throws GenericReNURBSCertificationError if certification is enabled,
 *         fails, and certifyThrowOnFailure is true.
 */
GenericReNURBSProfile buildGenericReNURBSProfile(
    const std::vector<GenericSample>& samples,
    const std::vector<SegmentInfo>& segments,
    const GenericReNURBSConfig& config);

} // namespace tether::motion::profile_renurbs
