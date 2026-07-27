/**
 * @file BlendSpec.hpp
 * @brief Blend specification and curve-type enum (plan §2.4)
 *
 * @details
 * `BlendSpec` is the per-corner input to the blend solver: tolerance
 * (signed — negative means "outside"/ear), continuity order (G² or G³),
 * curve type (exact Bézier or opt-in PH quintic fast path), and various
 * numerical knobs. See docs/motion/BlendingAlgorithm.md for the meaning
 * of each field and the acceptance test (M15)/(M20).
 */
#pragma once

#include "tether/motion_planner/geometry/Vector.hpp"

#include <array>
#include <cstdint>

namespace tether::motion {

/// Geometric continuity order of the blend boundary.
enum class Continuity { G2, G3 };

/// Path mode at a corner.
enum class PathMode {
    ExactStop, ///< Decelerate to zero velocity at the corner (δ = 0).
    ExactPath, ///< Follow the exact path (no blend, δ = 0).
    Blend,     ///< Cut the corner within the tolerance.
};

/// Blend curve construction strategy.
enum class BlendCurveType {
    BezierGk,   ///< Exact G² (quintic) or G³ (septic) Bézier — the default.
    PHQuintic,  ///< Opt-in PH quintic fast path (G¹ boundaries, D6).
};

/// Per-corner blend specification.
struct BlendSpec {
    PathMode mode = PathMode::Blend;
    /// Signed tolerance. Positive = inside cut (standard G64 P).
    /// Negative = outside bulge ("ear"/dogbone, M20).
    double tolerance = 0.0;
    Continuity continuity = Continuity::G2;
    BlendCurveType curveType = BlendCurveType::BezierGk;

    /// Maximum fraction of each adjacent piece's free length that a trim
    /// may consume (default 0.5 = up to half of each neighbor).
    double maxBlendFraction = 0.5;

    /// Minimum segment length below which a piece is too short to trim.
    double minSegmentLength = 0.01;

    /// Corner classification thresholds (radians).
    double minAngleRad = 0.017453292519943295; // 1°
    double maxAngleRad = 3.054326190990077;    // 175°

    /// Certificate width for the deviation certifier (M10).
    /// 0 means auto: 1e-3·|tol|, with a hard floor of 1e-9.
    double certEpsilon = 0.0;

    /// Diagonal axis metric for weighted axis space (5-axis). All zeros
    /// (the default) means identity — Euclidean distance in all axes.
    /// When non-zero, distances are computed as √(Σ metric[i]·Δ[i]²).
    std::array<double, RVec::kMaxDim> metric{};

    /// Validate the spec. Throws std::invalid_argument if:
    /// - mode == Blend and tolerance == 0 (Blend requires a non-zero tol),
    /// - any metric entry is negative,
    /// - maxBlendFraction is outside (0, 1],
    /// - minSegmentLength <= 0,
    /// - angle thresholds are out of (0, π) or inverted.
    void validate() const;
};

} // namespace tether::motion
