/**
 * @file PlanningSegmentConverter.hpp
 * @brief Convert GCode::PlanningSegment[] → PiecewiseNurbsPath
 *
 * @details
 * This utility bridges the G-code parsing layer (tether_gcode) and the
 * geometry core (tether_motion_geometry). It converts a vector of
 * GCode::PlanningSegment into a tether::motion::PiecewiseNurbsPath,
 * using NurbsCurve::fromLine for linear/rapid segments and
 * NurbsCurve::fromArc for arc segments.
 *
 * Per-piece attributes (corner deviation %, extruder speed) are returned
 * alongside the path so that callers (e.g. the WebGCodeViewer) can
 * color-map the toolpath without recomputing them.
 *
 * Behavior:
 * - Zero-length segments (XYZ displacement < 1e-6 mm) are skipped.
 * - Arc segments that fail NurbsCurve::fromArc construction fall back
 *   to a line (NurbsCurve::fromLine) to avoid losing the segment.
 * - Plane selection for arcs: XY → axis1=X, axis2=Y; XZ → axis1=X,
 *   axis2=Z; YZ → axis1=Y, axis2=Z.
 *
 * Thread-safety: NOT thread-safe (PiecewiseNurbsPath has lazy caches).
 */

#pragma once

#include "tether/gcode/motion/InterpolationStrategy.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <utility>
#include <vector>

namespace tether::motion {

/// @brief Result of converting PlanningSegments to a NURBS path.
struct PlanningSegmentNurbsResult {
    PiecewiseNurbsPath path;                ///< The assembled NURBS path
    std::vector<float> deviations;          ///< Per-piece corner deviation % (0–100)
    std::vector<float> extruderSpeeds;      ///< Per-piece extruder speed (mm/s)
};

/// @brief Build a PiecewiseNurbsPath from a vector of PlanningSegments.
///
/// @param segments Planning segments (from PlanningSegmentBuilder or similar)
/// @return Result containing the NURBS path and per-piece attributes.
///         The deviations and extruderSpeeds vectors have one entry per
///         piece in the path (zero-length segments are skipped).
/// @throws std::invalid_argument if all segments are zero-length (empty path).
PlanningSegmentNurbsResult piecewiseNurbsFromSegments(
    const std::vector<GCode::PlanningSegment>& segments);

} // namespace tether::motion
