/**
 * @file PressureAdvanceReNurbsAdapter.hpp
 * @brief Adapter to convert pressure advance outputs into ReNURBS profiles.
 *
 * @details
 * Pressure advance (PA) algorithms produce time-parameterized offset
 * arrays — simple `std::vector<double>` values representing the extruder
 * position compensation at each time sample. This adapter wraps that
 * output into the generic SampledCurve format so it can be converted
 * into an analytical NURBS curve using the generic ReNURBS builder.
 *
 * ## Supported PA models
 *
 * - **Linear** (classic Klipper): δe = PA · v_e
 * - **PowerLaw** (non-Newtonian): δe = K_base · (v_e · A_f)^n
 * - **CrossWLF** (temperature-dependent): δe = (βV_m/A_f) · P_LUT(Q, T)
 *
 * All three produce the same output format: a time-parameterized offset
 * array with a ±maxCompensation safety clamp.
 *
 * ## Usage
 *
 * ```cpp
 * using namespace tether::motion::profile_renurbs;
 * using namespace tether::control::extrusion;
 *
 * // 1. Compute PA offsets (using any PA model)
 * PowerLawPressureAdvance pa(params, filament);
 * auto offsets = pa.offsetSeries(velocities, sampleInterval);
 *
 * // 2. Build a ReNURBS profile from the offsets
 * auto profile = buildPressureAdvanceReNurbs(
 *     offsets, sampleInterval, pa.params().maxCompensation);
 *
 * // 3. Use the profile (e.g., for SVG visualization)
 * exporter.exportGenericReNurbsProfile("pa_profile.svg", profile);
 * ```
 *
 * The resulting GenericReNurbsProfile has a single quantity named
 * "pressure_offset" with a SymmetricUniform constraint at ±maxCompensation.
 */

#pragma once

#include "tether/motion_planner/profile_renurbs/GenericReNurbsProfile.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNurbsBuilder.hpp"

#include <vector>
#include <string>

namespace tether::motion::profile_renurbs {

/// Configuration for pressure-advance ReNURBS construction.
struct PressureAdvanceReNurbsConfig {
    /// Interpolation tolerance for the offset curve [mm].
    double epsilon = 1e-6;

    /// Safety margin below the ±maxCompensation limit [mm].
    double safetyMargin = 1e-6;

    /// B-spline degree (continuity = degree − 1).
    int degree = 5;

    /// Max control points per segment.
    std::size_t maxControlPointsPerSegment = 64;

    /// Refinement grid multiplier.
    std::size_t refinementGridMultiplier = 10;

    /// Run the certifier after building.
    bool certify = true;

    /// Lipschitz certificate width goal.
    double certificationEpsilon = 1e-5;

    /// If true, throw on certification failure.
    bool certifyThrowOnFailure = false;

    /// Quantity name (default: "pressure_offset").
    std::string quantityName = "pressure_offset";
};

/**
 * @brief Build a generic ReNURBS profile from a pressure advance offset
 *        series.
 *
 * @param offsets The PA offset values [mm], one per time sample.
 * @param sampleInterval Time between samples [s].
 * @param maxCompensation The ±maxCompensation safety clamp [mm].
 * @param config Configuration (tolerances, degree, etc.).
 * @return A GenericReNurbsProfile with a single "pressure_offset" quantity,
 *         constrained to ±maxCompensation.
 *
 * The resulting profile has a single segment covering the full time range
 * [0, (n-1)·sampleInterval]. The offset is parameterized by time and
 * constrained to ±maxCompensation via a SymmetricUniform limit.
 */
inline GenericReNurbsProfile buildPressureAdvanceReNurbs(
    const std::vector<double>& offsets,
    double sampleInterval,
    double maxCompensation,
    const PressureAdvanceReNurbsConfig& config = {}) {

    // Build generic samples
    std::vector<GenericSample> samples;
    samples.reserve(offsets.size());
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        GenericSample s;
        s.parameter = static_cast<double>(i) * sampleInterval;
        s.quantities = {offsets[i]};
        s.limits = {maxCompensation};
        samples.push_back(s);
    }

    // Build segment info (single segment covering the full range)
    std::vector<SegmentInfo> segments;
    if (!offsets.empty()) {
        SegmentInfo si;
        si.paramStart = 0.0;
        si.paramEnd = static_cast<double>(offsets.size() - 1) * sampleInterval;
        segments.push_back(si);
    }

    // Build generic config
    GenericReNurbsConfig gcfg;
    gcfg.enabled = true;
    gcfg.maxControlPointsPerSegment = config.maxControlPointsPerSegment;
    gcfg.refinementGridMultiplier = config.refinementGridMultiplier;
    gcfg.certify = config.certify;
    gcfg.certificationEpsilon = config.certificationEpsilon;
    gcfg.certifyThrowOnFailure = config.certifyThrowOnFailure;

    QuantitySpec qs;
    qs.name = config.quantityName;
    qs.epsilon = config.epsilon;
    qs.safetyMargin = config.safetyMargin;
    qs.degree = config.degree;
    qs.lowerBound = std::nullopt; // SymmetricUniform handles the lower bound
    qs.limitType = LimitType::SymmetricUniform;
    qs.uniformLimit = maxCompensation;
    gcfg.quantities = {qs};

    return buildGenericReNurbsProfile(samples, segments, gcfg);
}

/**
 * @brief Build a generic ReNURBS profile from a PA offset series and
 *        corresponding extruder velocity series.
 *
 * This overload produces a 2-quantity profile:
 * - "pressure_offset": the PA offset (SymmetricUniform ±maxCompensation)
 * - "extruder_velocity": the extruder velocity (no constraint)
 *
 * Both are parameterized by time.
 */
inline GenericReNurbsProfile buildPressureAdvanceReNurbs(
    const std::vector<double>& offsets,
    const std::vector<double>& velocities,
    double sampleInterval,
    double maxCompensation,
    const PressureAdvanceReNurbsConfig& config = {}) {

    std::size_t n = std::min(offsets.size(), velocities.size());

    // Build generic samples
    std::vector<GenericSample> samples;
    samples.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        GenericSample s;
        s.parameter = static_cast<double>(i) * sampleInterval;
        s.quantities = {offsets[i], velocities[i]};
        s.limits = {maxCompensation,
                    std::numeric_limits<double>::infinity()};
        samples.push_back(s);
    }

    // Build segment info
    std::vector<SegmentInfo> segments;
    if (n > 0) {
        SegmentInfo si;
        si.paramStart = 0.0;
        si.paramEnd = static_cast<double>(n - 1) * sampleInterval;
        segments.push_back(si);
    }

    // Build generic config
    GenericReNurbsConfig gcfg;
    gcfg.enabled = true;
    gcfg.maxControlPointsPerSegment = config.maxControlPointsPerSegment;
    gcfg.refinementGridMultiplier = config.refinementGridMultiplier;
    gcfg.certify = config.certify;
    gcfg.certificationEpsilon = config.certificationEpsilon;
    gcfg.certifyThrowOnFailure = config.certifyThrowOnFailure;

    // Quantity 0: pressure_offset
    QuantitySpec qsOffset;
    qsOffset.name = config.quantityName;
    qsOffset.epsilon = config.epsilon;
    qsOffset.safetyMargin = config.safetyMargin;
    qsOffset.degree = config.degree;
    qsOffset.limitType = LimitType::SymmetricUniform;
    qsOffset.uniformLimit = maxCompensation;
    gcfg.quantities.push_back(qsOffset);

    // Quantity 1: extruder_velocity
    QuantitySpec qsVel;
    qsVel.name = "extruder_velocity";
    qsVel.epsilon = config.epsilon * 10; // looser for velocity
    qsVel.safetyMargin = 0.0;
    qsVel.degree = config.degree;
    qsVel.lowerBound = 0.0; // velocity is non-negative
    qsVel.limitType = LimitType::None;
    gcfg.quantities.push_back(qsVel);

    return buildGenericReNurbsProfile(samples, segments, gcfg);
}

} // namespace tether::motion::profile_renurbs
