/**
 * @file BlendSolver.hpp
 * @brief Per-corner blend solver (plan §2.4, math M15, M20)
 *
 * @details
 * `BlendSolver::solve` takes a `CornerAnalysis` plus a `BlendSpec` and
 * produces a `BlendGeometry` by:
 *
 * 1. Selecting the construction (exact Bézier G²/G³ or opt-in PH quintic).
 * 2. Running the (M15) bisection on the speed parameters α₁, β₁ to hit
 *    the certified deviation ≤ |tol| (T2).
 * 3. For negative tolerance (M20), augmenting the curvature vectors and
 *    using the signed-split certifier.
 * 4. Falling back to `ExactStop` if no acceptable blend is found (e.g.
 *    a neighbor is too short to trim, or the corner is a cusp).
 *
 * The solver is stateless and pure; all state lives in the per-corner
 * `BlendGeometry` output. `PathBlender` (Phase 3) calls `solve` per
 * corner and resolves overlaps between adjacent blends.
 *
 * Math reference: docs/motion/BlendingAlgorithm.md, (M15), (M20).
 */
#pragma once

#include "tether/motion_planner/blend/BlendGeometry.hpp"
#include "tether/motion_planner/blend/BlendSpec.hpp"
#include "tether/motion_planner/blend/CornerAnalysis.hpp"

namespace tether::motion {

class BlendSolver {
public:
    /**
     * @brief Construct with the two neighbor pieces (kept by reference
     *        only for the duration of `solve`). The pieces must be the
     *        ones analyzed by `CornerAnalyzer::analyze`.
     */
    BlendSolver(const NurbsCurve& in, const NurbsCurve& out,
                const CornerAnalysis& corner);

    /**
     * @brief Solve for the blend given a spec.
     *
     * Steps:
     * 1. If `spec.mode == ExactPath` or `spec.mode == ExactStop`, return
     *    immediately with the corresponding outcome.
     * 2. If `corner.kind == Straight`, return `NoBlendNeeded`.
     * 3. If `corner.kind == Cusp`, return `ExactStop` with a reason.
     * 4. Check that both neighbors are long enough to trim
     *    (`length ≥ 2·minSegmentLength`); if not, `ExactStop`.
     * 5. Run the (M15) bisection on α₁ = β₁ (symmetric initial search),
     *    building blends and certifying deviation, until
     *    `certificate.upper ≤ |tol|` or the speed range is exhausted.
     * 6. For negative tolerance (M20), augment curvature and use the
     *    signed-split certifier; require `insideHi ≤ ε` and
     *    `outsideHi ≤ |tol|`.
     * 7. For PH opt-in, build all 8 candidates, certify each, keep the
     *    best.
     *
     * @param spec Validated blend spec (`spec.validate()` is called).
     */
    BlendGeometry solve(const BlendSpec& spec) const;

private:
    const NurbsCurve& in_;
    const NurbsCurve& out_;
    const CornerAnalysis& corner_;

    // Internal helpers.
    BlendGeometry solveBezier(const BlendSpec& spec) const;
    BlendGeometry solvePH(const BlendSpec& spec) const;
};

} // namespace tether::motion
