/**
 * @file BlendGeometry.hpp
 * @brief Result of solving one corner (plan §2.4)
 *
 * @details
 * `BlendGeometry` is the output of `BlendSolver::solve`: either a blend
 * curve with its trim distances and certified deviation, or an exact-stop
 * fallback with a reason string. The `BlendOutcome` enum distinguishes
 * the three cases.
 */
#pragma once

#include "tether/motion_planner/blend/DeviationCertifier.hpp"
#include "tether/motion_planner/blend/PHQuinticBlendBuilder.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <optional>
#include <string>

namespace tether::motion {

/// Outcome of a corner blend attempt.
enum class BlendOutcome {
    Blended,     ///< A blend curve was built and certified ≤ |tol|.
    ExactStop,   ///< No acceptable blend; fall back to exact stop (δ = 0).
    NoBlendNeeded, ///< The corner is straight (θ < min); nothing to do.
};

/// Result of solving one corner.
struct BlendGeometry {
    BlendOutcome outcome = BlendOutcome::ExactStop;
    /// Human-readable diagnostic; filled iff outcome != Blended.
    std::string reason;

    /// The blend curve; valid iff outcome == Blended.
    /// (std::optional because NurbsCurve has no default constructor.)
    std::optional<NurbsCurve> blendCurve;

    /// Arc-length trims from the vertex: d₁ on the incoming piece,
    /// d₂ on the outgoing piece (M15). Zero for non-Blended outcomes.
    double trimIn = 0.0;
    double trimOut = 0.0;

    /// Certified deviation of the blend vs the trimmed original path (M14).
    /// Valid iff outcome == Blended.
    DeviationCertificate deviation{};

    /// PHData sidecar for closed-form operations (M16–M19).
    /// Filled iff outcome == Blended and spec.curveType == PHQuintic.
    /// When present, enables polynomial arc length, Newton inversion,
    /// and closed-form curvature on the blend curve (Phase 5.4 fast path).
    std::optional<PHData> phData;

    /// Number of bisection iterations the solver used (diagnostic).
    int solverIterations = 0;
};

} // namespace tether::motion
