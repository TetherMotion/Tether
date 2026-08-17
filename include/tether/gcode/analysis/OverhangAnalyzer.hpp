/// @file OverhangAnalyzer.hpp
/// @brief Detect overhangs, bridges and support usage from G-code planning segments.

#pragma once

#include "tether/gcode/SegmentSpeed.hpp"
#include "tether/gcode/analysis/AnalysisTypes.hpp"

#include <string>
#include <vector>

namespace GCode {
struct PlanningSegment;
}

namespace tether::gcode::analysis {

/// @brief Analyze overhangs, bridges and support usage from aligned planning data.
Section analyzeOverhangs(const std::vector<GCode::PlanningSegment>& planningSegments,
                         const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                         const std::vector<std::string>& gcodeLines,
                         const Options& options = {});

} // namespace tether::gcode::analysis
