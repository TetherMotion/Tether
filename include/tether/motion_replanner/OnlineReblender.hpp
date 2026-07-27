/**
 * @file OnlineReblender.hpp
 * @brief Online re-blending via PathBlender — true geometric replanning
 *
 * @details
 * When a junction persistently exceeds the contour error threshold, the
 * replanner can re-invoke tether::motion::PathBlender::blend on the path
 * with a tighter BlendSpec — producing a new BlendedPath with certified
 * deviation bounds and a full audit trail. This is true geometric
 * replanning (changing the path geometry), not just scalar feed/accel
 * adjustments.
 *
 * ## Why this is better
 *
 * The old replanner could only adjust scalar limits (feed rate, accel,
 * jerk) — it could not change the path geometry. On a sharp corner that
 * consistently causes tracking errors, the only option was to slow down,
 * which reduces throughput without addressing the root cause.
 *
 * The online reblender can:
 * - Tighten the blend tolerance at a problematic junction.
 * - Bump continuity from G² to G³ for smoother transitions.
 * - Switch from Bézier to PH quintic for the fast path.
 * - Fall back to ExactStop if no acceptable blend exists.
 *
 * Every decision is recorded in the BlendAuditEntry trail, providing the
 * "no silent fallback" guarantee from the kernel.
 *
 * ## Usage
 *
 * 1. Build a PiecewiseNurbsPath from the trajectory (Phase 1).
 * 2. Detect corners (Phase 3) and identify problematic junctions.
 * 3. Call reblend() with a tighter spec for those junctions.
 * 4. The result is a new BlendedPath that can be fed to the velocity
 *    profiler (Phase 7) for a complete re-plan.
 *
 * @see PathBlender.hpp for the kernel's blend orchestration.
 * @see BlendSpec.hpp for the per-corner specification.
 */

#pragma once

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/blend/PathBlender.hpp"
#include "tether/motion_planner/blend/BlendSpec.hpp"
#include "tether/motion_planner/blend/BlendGeometry.hpp"

#include <optional>
#include <vector>
#include <cstddef>
#include <string>

namespace tether::motion::replanner {

/// Configuration for the online reblender.
struct ReblenderConfig {
    /// Default blend tolerance (mm). Positive = inside cut.
    double defaultTolerance = 0.1; // 100 µm

    /// Tighter tolerance for problematic junctions (mm).
    double tightTolerance = 0.02; // 20 µm

    /// Default continuity order.
    Continuity defaultContinuity = Continuity::G2;

    /// Higher continuity for problematic junctions.
    Continuity tightContinuity = Continuity::G3;

    /// Default curve type.
    BlendCurveType defaultCurveType = BlendCurveType::BezierGk;

    /// Maximum blend fraction per neighbor.
    double maxBlendFraction = 0.5;

    /// Minimum segment length for trimming.
    double minSegmentLength = 0.01;
};

/// Result of a reblend operation.
struct ReblendResult {
    /// The new blended path (may be the same as input if no blends applied).
    BlendedPath blendedPath;

    /// Whether any junctions were re-blended with a tighter spec.
    bool reblended = false;

    /// Number of junctions that were successfully blended.
    int blendedCount = 0;

    /// Number that fell back to ExactStop.
    int exactStopCount = 0;

    /// Number that were straight (no blend needed).
    int straightCount = 0;

    /// Human-readable summary.
    std::string summary;
};

/**
 * @brief Re-blend the entire path with the default spec.
 *
 * This is the simplest form — applies the same BlendSpec to every
 * junction. Useful for initial path smoothing.
 *
 * @param path The input path.
 * @param config Reblender configuration.
 * @return The reblend result with the new BlendedPath and audit trail.
 */
ReblendResult reblend(
    const PiecewiseNurbsPath& path,
    const ReblenderConfig& config = {});

/**
 * @brief Re-blend the entire path with a custom BlendSpec.
 *
 * @param path The input path.
 * @param spec The blend spec to apply to every junction.
 * @return The reblend result.
 */
ReblendResult reblendWithSpec(
    const PiecewiseNurbsPath& path,
    const BlendSpec& spec);

/**
 * @brief Re-blend specific junctions with a tighter spec.
 *
 * This is the key online replanning operation: identify problematic
 * junctions (e.g. from Phase 3's corner detection + Phase 2's contour
 * error) and re-blend them with a tighter tolerance or higher continuity.
 *
 * The implementation blends the whole path with the tight spec for the
 * specified junctions and the default spec for the rest. (PathBlender
 * currently applies one spec template to all junctions; per-junction
 * specs require multiple blend passes or a future PathBlender extension.)
 *
 * @param path The input path.
 * @param problematicJunctions Indices of the problematic junctions
 *        (0..numPieces-2). These get the tight spec.
 * @param config Reblender configuration.
 * @return The reblend result.
 */
ReblendResult reblendJunctions(
    const PiecewiseNurbsPath& path,
    const std::vector<std::size_t>& problematicJunctions,
    const ReblenderConfig& config = {});

/**
 * @brief Extract the new PiecewiseNurbsPath from a BlendedPath.
 *
 * The BlendedPath contains the new piece sequence (trimmed originals +
 * blend curves). This wraps it back into a PiecewiseNurbsPath for
 * downstream use (e.g. feeding to the velocity profiler).
 *
 * @param blended The blended path from a reblend operation.
 * @return The new PiecewiseNurbsPath, or nullopt if the blended path
 *         is empty (no pieces).
 */
std::optional<PiecewiseNurbsPath> extractPath(
    const BlendedPath& blended);

} // namespace tether::motion::replanner
