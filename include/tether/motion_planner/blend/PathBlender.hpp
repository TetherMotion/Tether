/**
 * @file PathBlender.hpp
 * @brief Whole-path blend orchestration (plan §2.4, lemmas L1/L2)
 *
 * @details
 * `PathBlender::blend` takes a `PiecewiseNurbsPath` and a `BlendSpec`
 * template, analyzes every junction, solves each corner with
 * `BlendSolver`, resolves overlaps between adjacent blends (L1/L2), and
 * returns a `BlendedPath` — the new piece sequence plus an audit trail
 * of every decision made.
 *
 * Overlap resolution (L1): if two adjacent blends B_k and B_{k+1} both
 * want to trim the shared piece P_{k+1}, and their trims overlap
 * (trimOut_k + trimIn_{k+1} > length(P_{k+1})), the trims are reduced
 * proportionally until they fit. If even the minimum trims don't fit,
 * the smaller-deviation blend is kept and the other falls back to
 * ExactStop (L2).
 *
 * The audit trail records, per corner: the spec used, the solver
 * outcome, the certified deviation, the number of iterations, and any
 * overlap adjustments. This is the "no silent fallback" guarantee —
 * every decision is visible to the caller.
 *
 * Math reference: (M15) solver and acceptance test, (M18) initial trim
 * guess from bisector geometry. See docs/motion/BlendingAlgorithm.md.
 */
#pragma once

#include "tether/motion_planner/blend/BlendGeometry.hpp"
#include "tether/motion_planner/blend/BlendSpec.hpp"
#include "tether/motion_planner/blend/CornerAnalysis.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <optional>
#include <string>
#include <vector>

namespace tether::motion {

/// One entry in the blend audit trail.
struct BlendAuditEntry {
    std::size_t cornerIndex;       ///< Index of the junction (0..N-2).
    CornerKind cornerKind;         ///< Classification of the corner.
    double angleRad;               ///< Turning angle.
    BlendSpec spec;                ///< Spec used (may differ from template
                                   ///< after overlap adjustment).
    BlendGeometry geometry;        ///< Solver result.
    double originalTrimIn = 0.0;   ///< Trim requested before overlap adj.
    double originalTrimOut = 0.0;
    double overlapAdjustment = 0.0; ///< Reduction due to overlap (≥ 0).
    std::string note;              ///< Human-readable diagnostic.
};

/// Result of blending a whole path.
struct BlendedPath {
    /// The new piece sequence: original pieces trimmed and interleaved
    /// with blend curves. Empty if no blends were applied.
    std::vector<NurbsCurve> pieces;

    /// PHData sidecar per piece: phData[i] is populated iff pieces[i] is
    /// a PH quintic blend curve (BlendCurveType::PHQuintic). Absent for
    /// original (non-blend) pieces and non-PH blend curves.
    /// When present, enables closed-form arc length / curvature (M16–M19).
    std::vector<std::optional<PHData>> phData;

    /// Audit trail, one entry per junction.
    std::vector<BlendAuditEntry> audit;

    /// Number of corners that were successfully blended.
    int blendedCount = 0;
    /// Number that fell back to ExactStop.
    int exactStopCount = 0;
    /// Number that were straight (no blend needed).
    int straightCount = 0;
};

/// Whole-path blend orchestrator.
class PathBlender {
public:
    /**
     * @brief Blend all junctions in a path.
     *
     * @param path The input piecewise path (G0-connected recommended but
     *        not required; disconnected junctions are skipped with a
     *        note in the audit).
     * @param specTemplate The default spec applied to every corner. The
     *        solver may adjust trims per-corner for overlap; the spec is
     *        copied into each audit entry.
     * @return A `BlendedPath` with the new piece sequence and audit trail.
     */
    BlendedPath blend(const PiecewiseNurbsPath& path,
                      const BlendSpec& specTemplate) const;
};

} // namespace tether::motion
