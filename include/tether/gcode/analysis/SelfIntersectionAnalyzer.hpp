/**
 * @file SelfIntersectionAnalyzer.hpp
 * @brief Detect self-intersections and crossing toolpaths.
 */

#pragma once

#include "tether/gcode/SegmentSpeed.hpp"
#include "tether/gcode/analysis/AnalysisTypes.hpp"

#include <string>
#include <vector>

namespace GCode {
struct PlanningSegment;
}

namespace tether::gcode::analysis {

/**
 * @brief Analyze self-intersections and crossing toolpaths from aligned
 *        planning data.
 * @param planningSegments Planned motion segments to inspect.
 * @param segmentSpeeds    Per-segment speed results aligned with
 *        `planningSegments`.
 * @param gcodeLines       Raw G-code lines for reporting context.
 * @param options          Analyzer options; default-constructed when omitted.
 * @return Analysis section describing detected self-intersections.
 */
Section analyzeSelfIntersections(const std::vector<GCode::PlanningSegment>& planningSegments,
                                 const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                                 const std::vector<std::string>& gcodeLines,
                                 const Options& options = {});

} // namespace tether::gcode::analysis
