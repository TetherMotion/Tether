/**
 * @file BlendCurveBuilder.hpp
 * @brief Exact G² (quintic) and G³ (septic) Bézier blend construction
 *
 * @details
 * This is the *only* place where the control-point formulas (M11)/(M12)
 * live. Given boundary conditions (position, unit tangent, curvature
 * vector, and optionally jounce) at the two trim points, and free scalar
 * "speed" parameters α₁ (entry) and β₁ (exit), the builder solves the
 * Bernstein endpoint derivative identities (G.4) to produce a single-span
 * Bézier NURBS (weights 1, clamped knots) that matches the boundary data.
 *
 * The result is true Gᵏ continuity with the neighbors (Theorem T1):
 * because ‖B'(0)‖ = α₁, the parametric derivative equals the arc-length
 * derivative at the boundary, and the imposed B″(0) = α₁² κ⃗_A converts
 * exactly to d²p/ds² = κ⃗_A.
 *
 * Outside (negative-tolerance) blends use the *same* formulas with
 * augmented curvature vectors (M20); see `BlendSolver` for the
 * augmentation and `BlendingAlgorithm.md` for the construction.
 *
 * Math reference: docs/motion/BlendingAlgorithm.md, equations (M11)/(M12).
 */
#pragma once

#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

namespace tether::motion {

/// Static, pure builder for G²/G³ Bézier blends. No state, no side effects.
class BlendCurveBuilder {
public:
    /**
     * @brief Build a degree-5 (quintic) Bézier matching G² boundary data.
     *
     * Imposes (M11):
     *   B(0) = p_A,  B'(0) = α₁ T_A,  B''(0) = α₁² κ⃗_A
     *   B(1) = p_B,  B'(1) = β₁ T_B,  B''(1) = β₁² κ⃗_B
     *
     * Control points (closed form, n=5, n(n−1)=20):
     *   P0 = p_A
     *   P1 = P0 + (α₁/5) T_A
     *   P2 = 2P1 − P0 + (α₁²/20) κ⃗_A
     *   P5 = p_B
     *   P4 = P5 − (β₁/5) T_B
     *   P3 = 2P4 − P5 + (β₁²/20) κ⃗_B
     *
     * @param entry  Boundary conditions at the trim point on the incoming piece.
     * @param exit   Boundary conditions at the trim point on the outgoing piece.
     * @param alpha1 Entry speed α₁ > 0 (‖B'(0)‖).
     * @param beta1  Exit speed β₁ > 0 (‖B'(1)‖).
     * @throws std::invalid_argument if α₁ or β₁ ≤ 0, or if entry/exit
     *         dimensions differ.
     */
    static NurbsCurve buildQuintic(const BoundaryConditions& entry,
                                   const BoundaryConditions& exit,
                                   double alpha1, double beta1);

    /**
     * @brief Build a degree-7 (septic) Bézier matching G³ boundary data.
     *
     * Imposes (M12) — additionally matches jounce j⃗ at both ends.
     * n=7, n(n−1)=42, n(n−1)(n−2)=210:
     *   P0 = p_A
     *   P1 = P0 + (α₁/7) T_A
     *   P2 = 2P1 − P0 + (α₁²/42) κ⃗_A
     *   P3 = 3P2 − 3P1 + P0 + (α₁³/210) j⃗_A
     *   P7 = p_B
     *   P6 = P7 − (β₁/7) T_B
     *   P5 = 2P6 − P7 + (β₁²/42) κ⃗_B
     *   P4 = 3P5 − 3P6 + P7 − (β₁³/210) j⃗_B   ← note the MINUS sign
     *
     * @throws std::invalid_argument if α₁ or β₁ ≤ 0, dimensions differ,
     *         or either boundary lacks jounce (`hasJounce == false`).
     */
    static NurbsCurve buildSeptic(const BoundaryConditions& entry,
                                  const BoundaryConditions& exit,
                                  double alpha1, double beta1);
};

} // namespace tether::motion
