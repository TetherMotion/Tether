/**
 * @file SegmentConverter.hpp
 * @brief Convert legacy `MotionPlanner::MotionSegment` → `tether::motion::NurbsCurve`.
 *
 * @details
 * This is the bridge between the parsed G-code representation
 * (`MotionSegment`, a 9-axis DTO in the `MotionPlanner` namespace) and the
 * new geometry core (`tether::motion::NurbsCurve`, a dynamic-dimension
 * NURBS in the `tether::motion` namespace).
 *
 * ## Conversion rules (plan §4.1)
 *
 * | `MotionSegmentType` | Output |
 * |---|---|
 * | `Linear`, `Rapid` | `NurbsCurve::fromLine(start, end)` (degree 1) |
 * | `ArcCW`, `ArcCCW` | `NurbsCurve::fromArc` (M6) with the plane from `ArcPlane` |
 * | `CubicSpline`, `QuadSpline` | NURBS from stored control points |
 * | `NURBS` | pass-through of control points / weights / knots / degree |
 * | `Dwell` | no geometry — `convert` returns `std::nullopt` |
 *
 * ## Active-axis extraction
 *
 * `MotionSegment` stores all 9 axes in `startPosition`/`endPosition`, but
 * only `numActiveAxes` are actually moving. The converter extracts only
 * the active axes (preserving their order) so the resulting `NurbsCurve`
 * has the minimal dimension. This keeps the blend math in the correct
 * tangent subspace.
 *
 * ## Source reference propagation
 *
 * The `SourceReference` from the segment is attached to the curve via
 * `NurbsCurve::setSourceRef` (if available) so traceability is preserved
 * through the whole pipeline.
 */
#pragma once

#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <optional>
#include <vector>

namespace tether::motion {

class SegmentConverter {
public:
    /**
     * @brief Convert a single `MotionSegment` to a `NurbsCurve`.
     *
     * @param seg The segment to convert.
     * @return The NURBS curve, or `std::nullopt` for `Dwell` segments
     *         (no geometry).
     * @throws std::invalid_argument if the segment is degenerate
     *         (zero-length line, zero-radius arc, empty NURBS).
     */
    static std::optional<NurbsCurve> convert(const MotionPlanner::MotionSegment& seg);

    /**
     * @brief Convert a list of segments to a list of NURBS curves,
     *        skipping `Dwell` segments.
     *
     * @param segments The input segments.
     * @return The NURBS curves in order.
     * @throws std::invalid_argument if any non-Dwell segment is degenerate.
     */
    static std::vector<NurbsCurve> convertAll(
        const std::vector<MotionPlanner::MotionSegment>& segments);

private:
    /// Extract the active axes from a 9-axis position array as an `RVec`.
    static RVec extractActive(const std::array<double, MotionPlanner::MAX_MOTION_AXES>& pos,
                              const std::array<bool, MotionPlanner::MAX_MOTION_AXES>& activeAxes,
                              std::size_t numActive);

    /// Build a line NURBS from start/end positions.
    static NurbsCurve convertLinear(const MotionPlanner::MotionSegment& seg);

    /// Build an arc NURBS (M6) honoring the ArcPlane.
    static NurbsCurve convertArc(const MotionPlanner::MotionSegment& seg);

    /// Build a NURBS from stored control points / weights / knots / degree.
    static NurbsCurve convertNurbs(const MotionPlanner::MotionSegment& seg);
};

} // namespace tether::motion
