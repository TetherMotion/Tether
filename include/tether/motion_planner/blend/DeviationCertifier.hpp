/**
 * @file DeviationCertifier.hpp
 * @brief Certified Hausdorff deviation of a blend vs the original path (M14, M10)
 *
 * @details
 * The certifier computes a *certified interval* [lower, upper] that
 * provably contains the true max deviation δ of the blend curve from the
 * two trimmed original pieces (M14). The width upper − lower is ≤ the
 * requested `epsilon` (M10 Lipschitz certificate).
 *
 * For negative-tolerance (outside/ear) blends, an optional `cutDirection`
 * enables the signed inside/outside split (M20): the certificate is
 * decomposed into (insideLo, insideHi) — the interior-cut component that
 * must be ≈ 0 — and (outsideLo, outsideHi) — the exterior ear-height
 * component that must be ≤ |tol|.
 *
 * Math reference: docs/motion/BlendingAlgorithm.md, (M10), (M14), (M20).
 */
#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

namespace tether::motion {

/// Certified deviation interval: the true δ lies in [lower, upper].
struct DeviationCertificate {
    /// Two-sided Hausdorff deviation (M14): true δ ∈ [lower, upper].
    double lower = 0.0;
    double upper = 0.0;

    /// Signed split (M20): interior-cut component (the forbidden side for
    /// ears). Valid (non-zero) only when a cut direction was supplied.
    double insideLo = 0.0;
    double insideHi = 0.0;

    /// Signed split (M20): exterior (ear-height) component.
    double outsideLo = 0.0;
    double outsideHi = 0.0;
};

/// Computes certified deviation intervals for blend curves.
class DeviationCertifier {
public:
    /// @param epsilon Certificate width goal (upper − lower ≤ epsilon).
    explicit DeviationCertifier(double epsilon);

    /**
     * @brief Certify the Hausdorff deviation of `blend` vs the trimmed
     *        original pieces `trimmedIn` and `trimmedOut` (M14).
     *
     * δ = max( max_t dist(B(t), Ω),  max_{q∈Ω} dist(q, B) )
     * where Ω = trimmedIn ∪ trimmedOut.
     *
     * Both terms use the (M10) Lipschitz certificate: sample on a grid
     * fine enough that the Lipschitz bound (‖B'‖ or 1) times half the
     * grid spacing is ≤ epsilon, evaluate the distance exactly at each
     * sample via `pointCurveDistance` (G.29), and report
     *   [max_sample, max_sample + L·h/2].
     *
     * @param cutDirection If non-null, additionally computes the signed
     *        inside/outside split (M20). Samples within ε_cert of side 0
     *        (on the cut-direction line) are attributed to both sides
     *        conservatively. The cut direction is
     *        c = normalize(T_B − T_A) — the wedge bisector pointing into
     *        the corner interior — NOT the tangent bisector b (which is
     *        perpendicular to c for symmetric corners and classifies
     *        nothing).
     */
    DeviationCertificate certify(const NurbsCurve& blend,
                                 const NurbsCurve& trimmedIn,
                                 const NurbsCurve& trimmedOut,
                                 const RVec* cutDirection = nullptr) const;

    double epsilon() const noexcept { return epsilon_; }

private:
    double epsilon_;
};

} // namespace tether::motion
