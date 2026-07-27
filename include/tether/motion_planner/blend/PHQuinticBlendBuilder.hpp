/**
 * @file PHQuinticBlendBuilder.hpp
 * @brief Opt-in Pythagorean-hodograph quintic blend fast path (D6, M16–M19)
 *
 * @details
 * A Pythagorean-hodograph (PH) quintic is a planar polynomial Bézier whose
 * hodograph is the square of a complex polynomial, making its parametric
 * speed σ(ξ) = ‖r'(ξ)‖ and therefore its arc length *polynomial* (M16).
 * This eliminates adaptive quadrature for arc length, inversion, and
 * curvature — the reason PH curves are attractive for real-time CNC
 * interpolation (M19).
 *
 * **Trade-off (documented, opt-in only):** a PH quintic constrained by
 * endpoint positions and tangents (Hermite data) has *no remaining DOF*
 * to match boundary curvature, so PH blends are only G¹-continuous with
 * their neighbors (a centripetal-acceleration step v²·Δκ appears at the
 * blend boundaries). This is the price of the fast path; the geometric
 * tolerance guarantee is NOT traded away — PH candidates pass through
 * the same DeviationCertifier acceptance loop (T2/T3) as exact Béziers.
 *
 * **Four-candidate construction (M17):** the Hermite ω construction has
 * independent ± choices giving four distinct interpolants. This builder
 * constructs *all four*, and the caller (BlendSolver) certifies each and
 * keeps the one with the smallest certified deviation — never select by
 * sign convention.
 *
 * **Planar by nature:** the complex-number form (M16) exists only in 2D.
 * All construction happens in the corner plane (M13) via the `PHData`
 * basis and is lifted back to ℝᴺ.
 *
 * Math reference: docs/motion/BlendingAlgorithm.md, (M16)–(M19).
 * References: [R1] Wang et al. 2010, [R2] Farouki & Shah 1996,
 *             [R3] Moon, Farouki & Choi 2001.
 */
#pragma once

#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <complex>
#include <vector>

namespace tether::motion {

/// Sidecar for a PH quintic blend: the ω coefficients and corner-plane
/// basis that enable closed-form arc length / inversion / curvature (M16).
struct PHData {
    /// Complex ω coefficients of the quadratic ω(ξ) = ω₀(1−ξ)² +
    /// 2ω₁(1−ξ)ξ + ω₂ξ² whose square is the hodograph (M16).
    std::complex<double> w0, w1, w2;

    /// Corner-plane basis (M13): the complex coordinate r = x + iy refers
    /// to the plane through `origin` spanned by `planeE1`, `planeE2`.
    RVec planeE1, planeE2, origin;
};

/// Opt-in PH quintic blend builder (D6 fast path).
class PHQuinticBlendBuilder {
public:
    /// A candidate PH blend: the ordinary quintic NURBS (weights 1) plus
    /// the PHData sidecar for closed-form operations.
    struct Result {
        NurbsCurve curve;
        PHData ph;
        bool degenerate = false; ///< true if σ ≡ 0 anywhere (discard).
    };

    /**
     * @brief Build ALL FOUR Hermite candidates (M17).
     *
     * The caller (BlendSolver) certifies each and keeps the best — never
     * select by sign convention. Degenerate candidates (σ touching 0,
     * i.e. u = v = 0 simultaneously) are marked `degenerate = true` and
     * must be discarded by the caller.
     *
     * Construction happens in the corner plane (e₁, e₂) from (M13) and
     * is lifted back to ℝᴺ. Endpoint tangent magnitudes are c·‖unit‖
     * with c = 2θ/π as the [R1] default initial value.
     *
     * @param entry  Boundary conditions at the entry trim point.
     * @param exit   Boundary conditions at the exit trim point.
     * @param planeE1, planeE2  Orthonormal basis of the corner plane (M13).
     */
    static std::vector<Result> buildCandidates(const BoundaryConditions& entry,
                                               const BoundaryConditions& exit,
                                               const RVec& planeE1,
                                               const RVec& planeE2);

    /// @name Closed-form operations on the PH representation (T4, no quadrature).
    ///@{

    /// Arc length s(ξ) = ∫₀^ξ σ(τ) dτ — a polynomial (M16).
    static double arcLength(const PHData& ph, double xi);

    /// Invert s → ξ by Newton–Raphson on the polynomial s(ξ) (M19).
    /// Bracketed by bisection on [0,1] for safety (σ > 0 ⇒ monotone).
    static double invertArcLength(const PHData& ph, double s);

    /// Curvature κ(ξ) = 2(u v' − u' v) / σ²(ξ) — closed form (M16).
    static double curvature(const PHData& ph, double xi);

    ///@}
};

} // namespace tether::motion
