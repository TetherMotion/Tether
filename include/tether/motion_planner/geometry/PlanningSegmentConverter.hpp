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

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace tether::motion {

/**@brief A dwell point: a position on the path where motion pauses. */
struct DwellPoint {
    double arcLength = 0.0; /**< Arc length along the path (mm) */
    double duration = 0.0; /**< Dwell duration (seconds) */
};

/**
 * Statistics from geometric recompression (populated only when
 * `SegmentRecompressOptions::enabled` and recompression actually ran).
 */
struct RecompressionStats {
    std::size_t sourcePieces = 0;        /**< pieces before recompression */
    std::size_t runsFitted = 0;          /**< runs replaced by fitted pieces */
    std::size_t runsSplit = 0;           /**< runs recursively split */
    std::size_t piecesKeptVerbatim = 0;  /**< source pieces copied unchanged */
    double maxCertifiedDeviation = 0.0;  /**< certified Hausdorff bound [mm] */
};

/**@brief Result of converting PlanningSegments to a NURBS path. */
struct PlanningSegmentNurbsResult {
    PiecewiseNurbsPath path; /**< The assembled NURBS path */
    std::vector<float> deviations; /**< Per-piece corner deviation % (0–100) */
    std::vector<float> extruderSpeeds; /**< Per-piece extruder speed (mm/s) */
    std::vector<double> feedRates; /**< Per-piece feed rate (mm/s, converted from mm/min) */
    std::vector<DwellPoint> dwellPoints; /**< Dwell points (G4 commands), in arc-length order */
    std::optional<RecompressionStats> recompression; /**< Set when recompression ran */
};

/**
 * @brief Build a PiecewiseNurbsPath from a vector of PlanningSegments.
 *
 * @param segments Planning segments (from PlanningSegmentBuilder or similar)
 * @return Result containing the NURBS path and per-piece attributes.
 *         The deviations and extruderSpeeds vectors have one entry per
 *         piece in the path (zero-length segments are skipped).
 * @throws std::invalid_argument if all segments are zero-length (empty path).
 */
PlanningSegmentNurbsResult piecewiseNurbsFromSegments(
    const std::vector<GCode::PlanningSegment>& segments);

/**
 * Options for error-bounded geometric recompression of the converted
 * path (see PathRecompressor.hpp). Disabled by default — the converter
 * then behaves exactly as before (one piece per segment). When enabled,
 * runs of tiny degree-1 pieces (slicer-tessellated curves) are merged
 * into fitted B-spline pieces certified within `tolerance` mm of the
 * original polyline; corners sharper than `cornerAngleDeg`, non-line
 * pieces (exact arcs/splines), dwell boundaries, and G0 breaks are
 * preserved exactly.
 */
struct SegmentRecompressOptions {
    bool enabled = false;              ///< opt-in — off keeps 1:1 pieces
    double tolerance = 0.02;           ///< certified deviation bound [mm]
    int degree = 3;                    ///< fitted piece degree
    double cornerAngleDeg = 10.0;      ///< junction break angle
    std::size_t maxControlPoints = 48; ///< per-piece CP cap before split
};

/**
 * Same as above, with optional geometric recompression applied after
 * conversion (not applied to outside-circle-blended paths — blending
 * already restructures the geometry).
 *
 * When recompression merges pieces, per-piece attributes are merged
 * conservatively over each output piece's source range: feed rates
 * take the minimum positive value (a fitted piece never gets a looser
 * cap than its tightest source segment), deviations and extruder speeds
 * take the maximum (conservative for colour-map/planner inspection),
 * and dwell points keep their piece-boundary positions remapped onto
 * the output path's arc-length coordinate.
 */
PlanningSegmentNurbsResult piecewiseNurbsFromSegments(
    const std::vector<GCode::PlanningSegment>& segments,
    const SegmentRecompressOptions& opts);

} // namespace tether::motion
