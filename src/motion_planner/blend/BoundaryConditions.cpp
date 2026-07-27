/**
 * @file BoundaryConditions.cpp
 * @brief Implementation of tether::motion::boundaryAt.
 */

#include "tether/motion_planner/blend/BoundaryConditions.hpp"

#include <cmath>
#include <stdexcept>

namespace tether::motion {

BoundaryConditions boundaryAt(const NurbsCurve& curve, double s, bool atEnd) {
    const double L = curve.length();
    if (!(s >= 0.0 && s <= L)) {
        throw std::invalid_argument(
            "boundaryAt: arc length s out of [0, length]");
    }
    // Invert s → u on the curve.
    const double u = (s <= 0.0) ? curve.knotMin()
                      : (s >= L) ? curve.knotMax()
                                 : curve.invertLength(s);

    // arcDerivatives at order 3 gives p, T, κ⃗, j⃗ (G.18)–(G.21).
    const ArcDerivatives ad = curve.arcDerivatives(u, 3);

    BoundaryConditions bc;
    bc.position = ad.position;
    bc.curvature = ad.curvature;
    bc.jounce = ad.jounce;
    bc.hasJounce = true; // j⃗ = 0 for lines is a usable value (M12).

    // Tangent convention:
    // - atEnd = true  (incoming piece's end): tangent points OUT of the
    //   piece, toward the junction/vertex. NurbsCurve's T = dp/ds points
    //   forward (toward increasing s = toward the end). At the end of the
    //   piece, forward = toward the vertex. So T as-is is correct.
    // - atEnd = false (outgoing piece's start): tangent points INTO the
    //   piece, away from the vertex. NurbsCurve's T at the start points
    //   forward = away from the vertex. So T as-is is correct.
    // In both cases the arc-length tangent already points "forward along
    // the path", which is the convention the blend formulas (M11)/(M12)
    // expect: T_A points from the entry trim point toward the vertex,
    // T_B points from the vertex toward the exit trim point.
    bc.tangent = ad.tangent;

    // For atEnd, the boundary is at the END of the incoming piece. The
    // blend's entry trim point is at arc length (L - trim) on the
    // incoming piece, and T_A should point toward the vertex (forward).
    // The caller passes s = L - trim; at that point T points forward
    // (toward the end = toward the vertex). Correct.
    //
    // For !atEnd, the boundary is at the START region of the outgoing
    // piece. The blend's exit trim point is at arc length trim on the
    // outgoing piece, and T_B should point away from the vertex (forward
    // into the outgoing piece). The caller passes s = trim; at that
    // point T points forward. Correct.
    (void)atEnd; // convention is already handled by the forward direction.

    return bc;
}

} // namespace tether::motion
