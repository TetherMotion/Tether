/**
 * @file GenericReNurbsProfile.hpp
 * @brief Generic data structures for ReNURBS — applicable to any sampled curve.
 *
 * @details
 * The original ReNURBS implementation was tightly coupled to velocity
 * profiles (v(s), a(s), j(s), t(s)) with KinematicLimits. This header
 * defines a *generic* layer that works with any sampled curve:
 *
 * - Velocity profiles (parameterized by arc length, constrained by v_lim/a_lim)
 * - Pressure advance offsets (parameterized by time, constrained by ±maxCompensation)
 * - Deconvolution controller outputs (parameterized by time, optionally constrained)
 * - Any other scalar sampled quantity with optional per-sample limits
 *
 * The generic layer sits between the mathematical core (ProfileSplineFitter,
 * which is already fully generic) and domain-specific adapters
 * (VelocityProfileAdapter, PressureAdvanceAdapter).
 *
 * ## Architecture
 *
 * ```
 *   Domain data                    Generic layer
 *   ────────────                   ─────────────
 *   VelocityProfile  ──▶ VelocityProfileAdapter ──┐
 *                                                   ├──▶ SampledCurve
 *   PA offset array ──▶ PressureAdvanceAdapter ──┘       │
 *                                                         ▼
 *                                                  GenericReNurbsBuilder
 *                                                         │
 *                                                         ▼
 *                                                  GenericReNurbsProfile
 *                                                         │
 *                                                         ▼
 *                                                  GenericReNurbsCertifier
 *                                                         │
 *                                                         ▼
 *                                                  ProfileConstraintCertificate
 * ```
 */

#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/SourceReference.hpp"
#include "tether/motion_planner/profile_renurbs/ReNurbsProfile.hpp"

#include <vector>
#include <optional>
#include <string>
#include <stdexcept>
#include <cstddef>
#include <limits>

namespace tether::motion::profile_renurbs {

/// Base exception for ReNURBS certification failures.
/// Defined here (in the base data header) so both the velocity-specific
/// and generic builders can reference it without circular dependencies.
class ReNurbsCertificationError : public std::runtime_error {
public:
    explicit ReNurbsCertificationError(const std::string& msg)
        : std::runtime_error(msg) {}
};

// ===========================================================================
// Generic Sample Representation
// ===========================================================================

/// A single sample of one or more quantities at a parameter value.
///
/// The parameter can be anything monotonic: arc length, time, or any
/// other independent variable. Each sample carries one or more quantity
/// values and optionally per-quantity upper limits (for constraint
/// preservation).
///
struct GenericSample {
    /// The independent variable (arc length, time, etc.).
    double parameter = 0.0;

    /// Quantity values at this sample point. Index corresponds to the
    /// quantity index in the QuantitySpec vector.
    std::vector<double> quantities;

    /// Per-quantity upper limit at this sample point. Use +inf for
    /// "no limit". If empty, no per-sample limits are provided (the
    /// builder will use uniform limits from QuantitySpec, or none).
    std::vector<double> limits;
};

// ===========================================================================
// Generic Configuration
// ===========================================================================

/// How limits are interpreted for a quantity.
enum class LimitType : uint8_t {
    /// No constraint — pure interpolation, no clamping.
    None = 0,

    /// Upper limit only: q(u) ≤ limit(u) − safetyMargin.
    /// The limit comes from per-sample data (GenericSample::limits).
    UpperPerSample = 1,

    /// Upper limit only: q(u) ≤ uniformLimit − safetyMargin.
    /// The limit is a single constant for the entire curve.
    UpperUniform = 2,

    /// Symmetric limit: |q(u)| ≤ uniformLimit − safetyMargin.
    /// Used for pressure advance (±maxCompensation) and similar
    /// bounded-offset quantities.
    SymmetricUniform = 3,
};

/// Configuration for a single quantity in a generic ReNURBS profile.
struct QuantitySpec {
    /// Human-readable name (e.g., "velocity", "pressure_offset").
    std::string name;

    /// Interpolation tolerance: |B(u_i) − q_i| ≤ epsilon.
    double epsilon = 1e-4;

    /// Safety margin below the limit envelope.
    double safetyMargin = 1e-4;

    /// B-spline degree (continuity = degree − 1 inside a segment).
    int degree = 5;

    /// Optional lower bound (e.g., 0.0 for velocity, or nullopt for
    /// quantities that can go negative).
    std::optional<double> lowerBound;

    /// How limits are interpreted for this quantity.
    LimitType limitType = LimitType::None;

    /// Uniform limit value (used when limitType is UpperUniform or
    /// SymmetricUniform). Ignored for None and UpperPerSample.
    double uniformLimit = std::numeric_limits<double>::infinity();
};

/// Generic ReNURBS configuration.
struct GenericReNurbsConfig {
    /// Master switch.
    bool enabled = false;

    /// Per-quantity configuration. The order defines the quantity index
    /// in GenericSample::quantities and GenericSegmentProfile::quantities.
    std::vector<QuantitySpec> quantities;

    /// Adaptive refinement cap: max control points per segment per quantity.
    std::size_t maxControlPointsPerSegment = 64;

    /// Refinement grid multiplier (test grid = multiplier × samples).
    std::size_t refinementGridMultiplier = 10;

    /// Run the certifier after building.
    bool certify = true;

    /// Lipschitz certificate width goal.
    double certificationEpsilon = 1e-5;

    /// If true, throw on certification failure; if false, return the
    /// profile with the certificate attached (for debugging).
    bool certifyThrowOnFailure = true;
};

// ===========================================================================
// Generic Segment / Profile Representation
// ===========================================================================

/// Segment boundary info (decoupled from PathAdapter).
struct SegmentInfo {
    /// Parameter range [paramStart, paramEnd] covered by this segment.
    double paramStart = 0.0;
    double paramEnd = 0.0;

    /// Source reference (G-code traceability, or empty).
    MotionPlanner::SourceReference sourceRef;
};

/// NURBS curve result for one quantity in one segment (reuses the
/// existing ReNurbsQuantityCurves struct since it's already generic
/// in its fields — only the naming is velocity-specific, but the
/// data is just "a NURBS curve + fit metadata").
using GenericQuantityCurve = ReNurbsQuantityCurves;

/// All quantity curves for one segment.
struct GenericSegmentProfile {
    /// Segment index (matches SegmentInfo order).
    std::size_t segmentIndex = 0;

    /// Parameter range [paramStart, paramEnd].
    double paramStart = 0.0;
    double paramEnd = 0.0;

    /// Source reference.
    MotionPlanner::SourceReference sourceRef;

    /// Per-quantity NURBS curves. Index matches QuantitySpec order.
    std::vector<GenericQuantityCurve> quantities;

    /// Per-quantity boundary continuity (between this segment and the
    /// next). Index matches QuantitySpec order.
    std::vector<ContinuityClass> boundaryContinuity;
};

/// A generic constraint violation (reuses SegmentViolation but with
/// a quantity index instead of a fixed enum).
struct GenericViolation {
    std::size_t segmentIndex = 0;
    std::size_t quantityIndex = 0;  ///< Index into QuantitySpec vector
    std::string quantityName;       ///< Human-readable name
    double parameter = 0.0;         ///< Where the violation was detected
    double value = 0.0;             ///< The offending NURBS value
    double limit = 0.0;             ///< The limit at that parameter
    double overshoot = 0.0;         ///< value − limit
};

/// Generic continuity report for one segment boundary.
struct GenericContinuityReport {
    std::size_t segmentIndex = 0;
    std::vector<ContinuityClass> perQuantity;  ///< Index matches QuantitySpec
};

/// Certificate produced by the generic certifier.
struct GenericCertificate {
    bool compliant = true;
    std::vector<GenericViolation> violations;
    double lipschitzWidth = 0.0;
    bool residualBudgetExhausted = false;
    std::vector<GenericContinuityReport> continuity;
};

/// The complete generic ReNURBS profile.
struct GenericReNurbsProfile {
    /// Per-segment curves.
    std::vector<GenericSegmentProfile> perSegment;

    /// Optional certificate.
    std::optional<GenericCertificate> certificate;

    /// Quantity names (mirrors QuantitySpec::name for convenience).
    std::vector<std::string> quantityNames;

    /// True if the profile is empty.
    bool empty() const noexcept { return perSegment.empty(); }

    /// Number of segments.
    std::size_t numSegments() const noexcept { return perSegment.size(); }

    /// Number of quantities.
    std::size_t numQuantities() const noexcept { return quantityNames.size(); }
};

// ===========================================================================
// Convenience: convert between generic and velocity-specific structures
// ===========================================================================

/// Convert a GenericReNurbsProfile to a velocity-specific ReNurbsProfile.
///
/// This is used by the velocity adapter to maintain backward compatibility.
/// The generic profile must have exactly 4 quantities named (in order)
/// "velocity", "acceleration", "jerk", "time".
///
ReNurbsProfile toVelocityProfile(const GenericReNurbsProfile& generic);

/// Convert a velocity-specific ReNurbsProfile to a generic one.
GenericReNurbsProfile fromVelocityProfile(const ReNurbsProfile& velocity);

/// Convert a GenericCertificate to a velocity-specific
/// ProfileConstraintCertificate.
ProfileConstraintCertificate toVelocityCertificate(
    const GenericCertificate& generic);

} // namespace tether::motion::profile_renurbs
