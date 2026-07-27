/**
 * @file BoundaryConditions.hpp
 * @brief Boundary conditions (p, T, κ⃗, j⃗) at a trim point (plan §2.4)
 *
 * @details
 * A `BoundaryConditions` struct captures the arc-length derivatives of a
 * NURBS piece at one parameter value: position, unit tangent, curvature
 * vector, and (optionally) jounce. These are the inputs to the blend
 * control-point formulas (M11)/(M12).
 *
 * The free function `boundaryAt` extracts them from a NURBS curve at a
 * given arc length, using `NurbsCurve::arcDerivatives` (G.18)–(G.21).
 */
#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

namespace tether::motion {

/// (p, T, κ⃗, j⃗) at one trim point — the boundary data for a blend end.
struct BoundaryConditions {
    RVec position;  ///< Point on the neighbor curve at the trim location.
    RVec tangent;   ///< Unit tangent T = dp/ds.
    RVec curvature; ///< Curvature vector κ⃗ = d²p/ds² (⊥ T by construction).
    RVec jounce;    ///< Jounce vector j⃗ = d³p/ds³ (valid iff hasJounce).
    bool hasJounce = false;
};

/**
 * @brief Extract boundary conditions from `curve` at arc length `s`.
 *
 * `s = 0` gives the start of the curve; `s = curve.length()` gives the
 * end. The tangent is normalized; curvature and jounce come from
 * `arcDerivatives` (G.19)–(G.20). `hasJounce` is true unless the curve
 * lacks a usable 3rd derivative (which the caller may treat as j⃗ = 0
 * for lines — `boundaryAt` sets it explicitly in that case).
 *
 * @param atEnd If true, `s` is measured from the start but the tangent
 *        is flipped to point *out of* the curve (toward the junction);
 *        this is the convention for the incoming piece's end. If false,
 *        the tangent points *into* the curve (away from the junction),
 *        the convention for the outgoing piece's start.
 *
 * @throws std::domain_error if the curve has a degenerate parameterization
 *         at the trim point (|C'| ≈ 0).
 */
BoundaryConditions boundaryAt(const NurbsCurve& curve, double s, bool atEnd);

} // namespace tether::motion
