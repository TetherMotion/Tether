/**
 * @file ReNurbsProfile.hpp
 * @brief Data structures for the ReNURBS profile representation.
 *
 * @details
 * See ReNURBS.md for the full design. This file defines the output
 * structures of ReNurbsProfileBuilder: per-segment NURBS curves for
 * v(s), a(s), j(s), t(s), plus an optional constraint certificate.
 *
 * The curves are stored as tether::motion::NurbsCurve (1-D, weights all 1)
 * so they reuse the existing NURBS infrastructure (evaluation, derivatives,
 * Bézier decomposition for SVG rendering).
 */

#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/SourceReference.hpp"

#include <vector>
#include <optional>
#include <cstddef>

namespace tether::motion::profile_renurbs {

/// Continuity class achieved for a quantity (v, a, j, t) on a segment.
enum class ContinuityClass : uint8_t {
    Discontinuous = 0, ///< C⁻¹ (not even C⁰)
    C0 = 1,            ///< C⁰ / G⁰
    C1 = 2,            ///< C¹ / G¹
    C2 = 3,            ///< C² / G²
    C3 = 4,            ///< C³ / G³
    C4 = 5,            ///< C⁴ / G⁴ (quintic inside-segment)
};

/// NURBS curves for one profile quantity on one segment.
struct ReNurbsQuantityCurves {
    /// The NURBS curve (1-D, parameterized by u ∈ [0,1] within the segment).
    /// May be std::nullopt if the segment is degenerate (zero-length, etc.).
    std::optional<NurbsCurve> curve;

    /// Maximum residual |B(u_i) − q_i| over all samples in this segment.
    double maxResidual = 0.0;

    /// True if the fit achieved maxResidual ≤ epsilon.
    bool withinEpsilon = false;

    /// True if the constraint clamp was applied to this quantity.
    bool constraintClamped = false;

    /// True if the max control point cap was hit (graceful degradation).
    bool controlPointCapHit = false;

    /// Achieved continuity inside the segment.
    ContinuityClass achievedContinuity = ContinuityClass::C0;

    /// Number of control points (0 if no curve).
    std::size_t numControlPoints = 0;
};

/// All four NURBS curves for one motion segment.
struct ReNurbsSegmentProfile {
    /// Segment index in the PathAdapter.
    std::size_t segmentIndex = 0;

    /// Arc length range [sStart, sEnd] covered by this segment.
    double sStart = 0.0;
    double sEnd = 0.0;

    /// Source reference (G-code traceability).
    MotionPlanner::SourceReference sourceRef;

    /// NURBS curves for velocity v(s), acceleration a(s), jerk j(s), time t(s).
    ReNurbsQuantityCurves velocity;
    ReNurbsQuantityCurves acceleration;
    ReNurbsQuantityCurves jerk;
    ReNurbsQuantityCurves time;

    /// Continuity achieved at the *boundary* between this segment and the
    /// next (Cᵏ means both position and k derivatives match). For the last
    /// segment, this is N/A.
    ContinuityClass boundaryContinuityVelocity = ContinuityClass::C0;
    ContinuityClass boundaryContinuityAcceleration = ContinuityClass::C0;
    ContinuityClass boundaryContinuityJerk = ContinuityClass::C0;
    ContinuityClass boundaryContinuityTime = ContinuityClass::C0;
};

/// A constraint violation found by the certifier.
struct SegmentViolation {
    std::size_t segmentIndex = 0;
    enum class Quantity : uint8_t { Velocity, Acceleration, Jerk, Time } quantity;
    double arcLength = 0.0;  ///< Where the violation was detected
    double value = 0.0;      ///< The offending NURBS value
    double limit = 0.0;      ///< The limit at that s
    double overshoot = 0.0;  ///< value − limit
};

/// Continuity report for one segment boundary.
struct ContinuityReport {
    std::size_t segmentIndex = 0;
    ContinuityClass velocity = ContinuityClass::C0;
    ContinuityClass acceleration = ContinuityClass::C0;
    ContinuityClass jerk = ContinuityClass::C0;
    ContinuityClass time = ContinuityClass::C0;
};

/// Certificate produced by ProfileConstraintCertifier.
struct ProfileConstraintCertificate {
    bool compliant = true;
    std::vector<SegmentViolation> violations;
    double lipschitzWidth = 0.0;
    bool residualBudgetExhausted = false;
    std::vector<ContinuityReport> continuity;
};

/// The complete ReNURBS profile: per-segment NURBS curves + optional certificate.
struct ReNurbsProfile {
    /// Per-segment curves (one entry per PathAdapter segment).
    std::vector<ReNurbsSegmentProfile> perSegment;

    /// Optional constraint certificate (populated if config.certify == true).
    std::optional<ProfileConstraintCertificate> certificate;

    /// True if the profile is empty (no segments / no samples).
    bool empty() const noexcept { return perSegment.empty(); }

    /// Number of segments.
    std::size_t numSegments() const noexcept { return perSegment.size(); }
};

/// Configuration for ReNURBS profile construction.
struct ReNurbsConfig {
    bool enabled = false;

    // Interpolation tolerances
    double epsilonVelocity     = 1e-4;
    double epsilonAcceleration = 1e-3;
    double epsilonJerk         = 1e-2;
    double epsilonTime         = 1e-6;

    // Safety margins below the limit envelope
    double safetyMarginVelocity     = 1e-4;
    double safetyMarginAcceleration = 1e-3;
    double safetyMarginJerk         = 1e-2;

    // B-spline degrees (continuity = degree − 1 inside a segment)
    int degreeVelocity     = 5;  // C⁴ inside
    int degreeAcceleration = 4;  // C³ inside
    int degreeJerk         = 3;  // C² inside
    int degreeTime         = 5;  // C⁴ inside

    // Adaptive refinement caps
    std::size_t maxControlPointsPerSegment = 64;
    std::size_t refinementGridMultiplier = 10;

    // Discontinuity handling
    bool splitAtDiscontinuities = true;
    bool smoothAccelBasicToppra = false;

    // Optional features
    bool allowRationalExactFit = false;
    bool certify = true;
    double certificationEpsilon = 1e-5;
    bool certifyThrowOnFailure = true;
};

} // namespace tether::motion::profile_renurbs
