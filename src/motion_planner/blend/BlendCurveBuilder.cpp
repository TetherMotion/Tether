/**
 * @file BlendCurveBuilder.cpp
 * @brief Implementation of BlendCurveBuilder — (M11)/(M12) verbatim.
 *
 * Equation numbers (M.x) refer to docs/motion/BlendingAlgorithm.md.
 * The control-point formulas are derived there from the Bernstein
 * endpoint derivative identities (G.4).
 */

#include "tether/motion_planner/blend/BlendCurveBuilder.hpp"

#include <stdexcept>
#include <vector>

namespace tether::motion {

namespace {

/// Build a single-span Bézier NURBS from control points (weights 1,
/// clamped knots {0×(n+1), 1×(n+1)}).
NurbsCurve makeBezier(std::vector<RVec> cps) {
    const int n = static_cast<int>(cps.size()) - 1;
    std::vector<double> w(cps.size(), 1.0);
    std::vector<double> knots(static_cast<std::size_t>(n + 1) + n + 1, 0.0);
    for (std::size_t i = knots.size() / 2; i < knots.size(); ++i)
        knots[i] = 1.0;
    return NurbsCurve(std::move(cps), std::move(w), std::move(knots), n);
}

} // namespace

NurbsCurve BlendCurveBuilder::buildQuintic(const BoundaryConditions& entry,
                                           const BoundaryConditions& exit,
                                           double alpha1, double beta1) {
    if (!(alpha1 > 0.0) || !(beta1 > 0.0)) {
        throw std::invalid_argument(
            "BlendCurveBuilder::buildQuintic: alpha1 and beta1 must be > 0");
    }
    if (entry.position.dim() != exit.position.dim()) {
        throw std::invalid_argument(
            "BlendCurveBuilder::buildQuintic: entry/exit dimension mismatch");
    }

    // (M11) — degree 5, n(n−1) = 20.
    // Solve the (G.4) endpoint identities sequentially; each step is a
    // vector equation in one unknown control point (fully closed form).
    const RVec& pA = entry.position;
    const RVec& TA = entry.tangent;
    const RVec& kA = entry.curvature;
    const RVec& pB = exit.position;
    const RVec& TB = exit.tangent;
    const RVec& kB = exit.curvature;

    const RVec P0 = pA;                                  // B(0) = p_A
    const RVec P1 = P0 + TA * (alpha1 / 5.0);            // B'(0) = α₁ T_A
    const RVec P2 = P1 * 2.0 - P0 + kA * (alpha1 * alpha1 / 20.0); // B''(0)
    const RVec P5 = pB;                                  // B(1) = p_B
    const RVec P4 = P5 - TB * (beta1 / 5.0);             // B'(1) = β₁ T_B
    const RVec P3 = P4 * 2.0 - P5 + kB * (beta1 * beta1 / 20.0); // B''(1)

    return makeBezier({P0, P1, P2, P3, P4, P5});
}

NurbsCurve BlendCurveBuilder::buildSeptic(const BoundaryConditions& entry,
                                          const BoundaryConditions& exit,
                                          double alpha1, double beta1) {
    if (!(alpha1 > 0.0) || !(beta1 > 0.0)) {
        throw std::invalid_argument(
            "BlendCurveBuilder::buildSeptic: alpha1 and beta1 must be > 0");
    }
    if (entry.position.dim() != exit.position.dim()) {
        throw std::invalid_argument(
            "BlendCurveBuilder::buildSeptic: entry/exit dimension mismatch");
    }
    if (!entry.hasJounce || !exit.hasJounce) {
        throw std::invalid_argument(
            "BlendCurveBuilder::buildSeptic: both boundaries need jounce");
    }

    // (M12) — degree 7, n(n−1) = 42, n(n−1)(n−2) = 210.
    // NOTE the MINUS sign on the exit jounce term: B‴(1) = 210(P7 − 3P6 +
    // 3P5 − P4), so P4 = 3P5 − 3P6 + P7 − (β₁³/210) j⃗_B. Getting this
    // wrong passes position/tangent/curvature checks and fails only G³.
    const RVec& pA = entry.position;
    const RVec& TA = entry.tangent;
    const RVec& kA = entry.curvature;
    const RVec& jA = entry.jounce;
    const RVec& pB = exit.position;
    const RVec& TB = exit.tangent;
    const RVec& kB = exit.curvature;
    const RVec& jB = exit.jounce;

    const RVec P0 = pA;                                     // B(0)
    const RVec P1 = P0 + TA * (alpha1 / 7.0);               // B'(0)
    const RVec P2 = P1 * 2.0 - P0 + kA * (alpha1 * alpha1 / 42.0); // B''(0)
    const RVec P3 = P2 * 3.0 - P1 * 3.0 + P0 +
                    jA * (alpha1 * alpha1 * alpha1 / 210.0); // B'''(0)
    const RVec P7 = pB;                                     // B(1)
    const RVec P6 = P7 - TB * (beta1 / 7.0);                // B'(1)
    const RVec P5 = P6 * 2.0 - P7 +
                    kB * (beta1 * beta1 / 42.0);            // B''(1)
    const RVec P4 = P5 * 3.0 - P6 * 3.0 + P7 -
                    jB * (beta1 * beta1 * beta1 / 210.0); // B'''(1) ← MINUS

    return makeBezier({P0, P1, P2, P3, P4, P5, P6, P7});
}

} // namespace tether::motion
