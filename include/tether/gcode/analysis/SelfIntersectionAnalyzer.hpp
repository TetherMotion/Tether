/// @file SelfIntersectionAnalyzer.hpp
/// @brief Detect self-intersections and crossing toolpaths.

#pragma once

#include "tether/gcode/SegmentSpeed.hpp"
#include "tether/gcode/analysis/AnalysisTypes.hpp"

#include <string>
#include <vector>

namespace GCode {
struct PlanningSegment;
}

namespace tether::gcode::analysis {

/// @brief Analyze self-intersections and crossing toolpaths from aligned planning data.
Section analyzeSelfIntersections(const std::vector<GCode::PlanningSegment>& planningSegments,
                                 const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                                 const std::vector<std::string>& gcodeLines,
                                 const Options& options = {});

} // namespace tether::gcode::analysis
