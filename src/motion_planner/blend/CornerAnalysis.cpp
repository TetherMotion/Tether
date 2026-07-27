/**
 * @file CornerAnalysis.cpp
 * @brief Implementation of tether::motion::CornerAnalyzer (math M13).
 *
 * Equation numbers (M.x) refer to docs/motion/BlendingAlgorithm.md.
 */

#include "tether/motion_planner/blend/CornerAnalysis.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tether::motion {

CornerAnalyzer::CornerAnalyzer(double minAngleRad, double maxAngleRad)
    : minAngleRad_(minAngleRad), maxAngleRad_(maxAngleRad) {
    if (!(minAngleRad > 0.0 && minAngleRad < maxAngleRad &&
          maxAngleRad < M_PI)) {
        throw std::invalid_argument(
            "CornerAnalyzer: need 0 < minAngle < maxAngle < pi");
    }
}

CornerAnalysis CornerAnalyzer::analyze(const NurbsCurve& in,
                                       const NurbsCurve& out) const {
    if (in.dim() != out.dim()) {
        throw std::invalid_argument(
            "CornerAnalyzer: incoming and outgoing pieces have different "
            "dimensions");
    }
    const std::size_t dim = in.dim();

    // Junction point: end of `in` must equal start of `out` (G0 input).
    const RVec junctionIn = in.endPoint();
    const RVec junctionOut = out.startPoint();
    // Use a loose tolerance for the connectivity check — exact bitwise
    // equality is the caller's responsibility (plan §4.6); here we just
    // need the tangents to be meaningful.
    const double scale = std::max(in.length(), out.length());
    const double connTol = 1e-9 * std::max(scale, 1.0);
    if (junctionIn.distanceTo(junctionOut) > connTol) {
        throw std::invalid_argument(
            "CornerAnalyzer: incoming piece end does not coincide with "
            "outgoing piece start");
    }

    // Endpoint arc-length derivatives (G.18)–(G.21).
    // Incoming: evaluate at the END (u = knotMax), tangent points INTO
    // the piece (backward along the path). We want t_in = the tangent
    // at the junction as seen by the incoming piece, pointing toward the
    // vertex. NurbsCurve gives T = dp/ds which points forward (toward
    // increasing s, i.e. toward the end). At the end of `in`, T points
    // toward the vertex — that is t_in.
    const ArcDerivatives adIn = in.arcDerivatives(in.knotMax(), 3);
    // Outgoing: at the START (u = knotMin), T = dp/ds points forward —
    // away from the vertex, into the outgoing piece. That is t_out.
    const ArcDerivatives adOut = out.arcDerivatives(out.knotMin(), 3);

    CornerAnalysis result;
    result.vertex = junctionIn; // bitwise copy of the incoming endpoint
    result.tangentIn = adIn.tangent;
    result.tangentOut = adOut.tangent;
    result.curvatureIn = adIn.curvature;
    result.curvatureOut = adOut.curvature;
    result.jounceIn = adIn.jounce;
    result.jounceOut = adOut.jounce;
    result.hasJounce = true; // arcDerivatives(order=3) always fills jounce
                             // (zero for lines, which is a usable value).

    // θ from the clamped dot product (pitfall: always clamp before acos).
    const double dot = std::max(-1.0, std::min(1.0,
                                  result.tangentIn.dot(result.tangentOut)));
    result.angleRad = std::acos(dot);

    // Classify.
    if (result.angleRad < minAngleRad_) {
        result.kind = CornerKind::Straight;
        // Basis is degenerate for Straight; leave as zero.
        result.planeE1 = RVec::zero(dim);
        result.planeE2 = RVec::zero(dim);
        return result;
    }
    if (result.angleRad > maxAngleRad_) {
        result.kind = CornerKind::Cusp;
    } else {
        result.kind = CornerKind::Corner;
    }

    // (M13) corner-plane basis by modified Gram–Schmidt.
    //   e₁ = t_in   (already unit)
    //   e₂ = normalize(t_out − (t_out·e₁) e₁)
    // ‖t_out − (t_out·e₁)e₁‖ = sin θ > sin(minAngle) > 0, so stable.
    result.planeE1 = result.tangentIn; // unit by construction
    const double proj = result.tangentOut.dot(result.planeE1);
    RVec e2 = result.tangentOut - result.planeE1 * proj;
    // For a Cusp (θ near π) sin θ is small but still > sin(maxAngle) > 0
    // because maxAngle < π by the constructor invariant.
    result.planeE2 = e2.normalized();

    return result;
}

} // namespace tether::motion
