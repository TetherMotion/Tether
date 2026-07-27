/**
 * @file CornerAnalysis.hpp
 * @brief Per-corner analysis in the N-D tangent plane (plan §2.4, math M13)
 *
 * @details
 * Given two NURBS pieces meeting at a junction, `CornerAnalyzer::analyze`
 * extracts the endpoint tangents, curvature vectors, and (optionally)
 * jounce vectors from the neighbors' exact arc-length derivatives
 * (GeometryFoundations.md (G.18)–(G.21)), classifies the corner, and
 * builds an orthonormal basis of the tangent plane
 * span{t_in, t_out} ⊂ ℝᴺ by modified Gram–Schmidt (M13).
 *
 * Classification (using the angle θ = acos(clamp(t_in·t_out, −1, 1))):
 * - Straight: θ < minAngleRad — the two pieces are nearly collinear;
 *   no blend is needed.
 * - Corner: minAngleRad ≤ θ ≤ maxAngleRad — a blend can be constructed.
 * - Cusp: θ > maxAngleRad — the path reverses; blending is unsafe.
 *
 * The plane basis (e₁, e₂) is the foundation of all blend geometry: every
 * blend is constructed as a planar curve in this basis and lifted back to
 * ℝᴺ, giving one code path for 2, 3, and 5 axes (design decision D2).
 *
 * Math reference: docs/motion/BlendingAlgorithm.md, equation (M13).
 */
#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <cstdint>

namespace tether::motion {

/// Classification of a junction between two path pieces.
enum class CornerKind {
    Straight, ///< θ < minAngleRad: nearly collinear, no blend needed.
    Corner,   ///< minAngleRad ≤ θ ≤ maxAngleRad: blendable.
    Cusp,     ///< θ > maxAngleRad: path reverses, blending unsafe.
};

/// Result of analyzing one corner (the junction between two NURBS pieces).
struct CornerAnalysis {
    CornerKind kind = CornerKind::Corner;
    double angleRad = 0.0; ///< θ ∈ (0, π), the turning angle.

    RVec vertex;       ///< The shared junction point (end of `in`, start of `out`).
    RVec tangentIn;    ///< Unit tangent at the end of the incoming piece.
    RVec tangentOut;   ///< Unit tangent at the start of the outgoing piece.

    /// Orthonormal basis of span{t_in, t_out} (M13):
    /// e₁ = t_in,  e₂ = normalize(t_out − (t_out·e₁) e₁).
    RVec planeE1;
    RVec planeE2;

    RVec curvatureIn;  ///< κ⃗ = d²p/ds² at the end of the incoming piece.
    RVec curvatureOut; ///< κ⃗ at the start of the outgoing piece.
    RVec jounceIn;     ///< j⃗ = d³p/ds³ at the end of `in` (valid iff hasJounce).
    RVec jounceOut;    ///< j⃗ at the start of `out` (valid iff hasJounce).
    bool hasJounce = false;
};

/// Analyzes corners between consecutive NURBS pieces.
class CornerAnalyzer {
public:
    /**
     * @brief Construct with classification thresholds.
     * @param minAngleRad corners with θ < this are "Straight" (default ~1°).
     * @param maxAngleRad corners with θ > this are "Cusp" (default ~175°).
     */
    CornerAnalyzer(double minAngleRad, double maxAngleRad);

    /**
     * @brief Analyze the junction at the end of `in` / start of `out`.
     *
     * Extracts tangents, curvatures, and jounce from the neighbors' exact
     * NURBS derivatives, computes θ, classifies the corner, and builds the
     * (e₁, e₂) plane basis (M13). For `Corner` and `Cusp` kinds the basis
     * is always valid (θ is bounded away from 0 and π by construction).
     * For `Straight` the basis may be degenerate (t_out ≈ t_in) and is
     * left as zero vectors — callers must not use it.
     *
     * @throws std::invalid_argument if `in` and `out` have different
     *         dimensions, or if the junction points do not coincide
     *         (end of `in` ≠ start of `out`).
     */
    CornerAnalysis analyze(const NurbsCurve& in,
                           const NurbsCurve& out) const;

    double minAngleRad() const noexcept { return minAngleRad_; }
    double maxAngleRad() const noexcept { return maxAngleRad_; }

private:
    double minAngleRad_;
    double maxAngleRad_;
};

} // namespace tether::motion
