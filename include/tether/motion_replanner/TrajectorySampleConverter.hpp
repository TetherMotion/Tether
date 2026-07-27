/**
 * @file TrajectorySampleConverter.hpp
 * @brief Bridge from GCodeExport::TrajectorySample → tether::motion::PiecewiseNurbsPath
 *
 * @details
 * This is the integration bridge between the export module's dense sampled
 * trajectory representation (`GCodeExport::TrajectorySample`, a 9-axis sample
 * list) and the motion kernel's geometric path type
 * (`tether::motion::PiecewiseNurbsPath`, a sequence of `NurbsCurve` pieces).
 *
 * ## Why this exists
 *
 * The motion replanner historically consumed `vector<TrajectorySample>` and
 * performed all geometry with tangent-projection heuristics. To use the
 * kernel's certified primitives (`pointCurveDistance`, `CornerAnalyzer`,
 * `CertifiedCurvatureSampler`, `PathBlender`), the replanner needs a real
 * `PiecewiseNurbsPath`. This converter builds one from the sample list.
 *
 * ## Conversion rules
 *
 * Samples are grouped by `segmentIndex`. For each group:
 *
 * | motionType | Output |
 * |---|---|
 * | 0 (rapid), 1 (linear) | `NurbsCurve::fromLine(start, end)` (degree 1) |
 * | 2 (arcCW), 3 (arcCCW) | `NurbsCurve::fromArc` via 3-point circumcenter fit; polyline fallback if the fit residual exceeds `arcFitTolerance` |
 * | other | Polyline through all sample positions (degree 1, multi-span) |
 *
 * ## Active-axis extraction
 *
 * `TrajectorySample` stores all 9 axes, but only the axes that actually vary
 * across the trajectory are included in the resulting `RVec` dimension. This
 * keeps the blend math in the correct tangent subspace (matching
 * `SegmentConverter`'s convention).
 *
 * ## Limitations
 *
 * - Arc reconstruction from samples is approximate (3-point circumcenter).
 *   For exact arc geometry, use `SegmentConverter` with `MotionSegment` inputs.
 * - The converter does not propagate `SourceReference` (the sample list does
 *   not carry G-code line references).
 *
 * @see SegmentConverter for the MotionSegment → NurbsCurve bridge.
 * @see PiecewiseNurbsPath for the path type.
 */

#pragma once

#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <cstddef>
#include <vector>
#include <utility>
#include <optional>

namespace tether::motion::replanner {

/// Mapping from segment index to piece index in the resulting path.
struct SegmentToPieceMap {
    /// (segmentIndex, pieceIndex) pairs, sorted by segmentIndex.
    std::vector<std::pair<int32_t, std::size_t>> entries;

    /// Look up the piece index for a given segment index, or nullopt.
    std::optional<std::size_t> pieceForSegment(int32_t segmentIndex) const;
};

/// Configuration for the converter.
struct ConverterConfig {
    /// Maximum residual (mm) for accepting an arc fit.
    /// If the circumcenter fit residual exceeds this, a polyline is used.
    double arcFitTolerance = 0.001; // 1 µm

    /// Minimum number of samples in a segment to attempt arc fitting.
    /// Segments with fewer samples use a line or polyline.
    std::size_t minSamplesForArc = 3;

    /// Whether to collapse collinear line segments into a single line.
    /// If true, consecutive linear segments with the same direction are
    /// merged into one NurbsCurve. If false, each segment gets its own piece.
    bool mergeCollinearLines = false;
};

/**
 * @brief Convert a dense sampled trajectory to a piecewise NURBS path.
 *
 * @param samples The trajectory samples, ordered by time.
 * @return A PiecewiseNurbsPath with one piece per segment.
 * @throws std::invalid_argument if samples is empty or no segment has
 *         a valid geometry (e.g. all segments are zero-length).
 */
PiecewiseNurbsPath convertTrajectory(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    const ConverterConfig& config = {});

/**
 * @brief Convert a dense sampled trajectory to a piecewise NURBS path,
 *        also returning the segment-to-piece mapping.
 */
PiecewiseNurbsPath convertTrajectory(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    SegmentToPieceMap& map,
    const ConverterConfig& config = {});

} // namespace tether::motion::replanner
