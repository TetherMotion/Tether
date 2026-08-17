/// @file ZSeamAnalyzer.hpp
/// @brief Detect Z-seam positions and consistency across layers.

#pragma once

#include "tether/gcode/SegmentSpeed.hpp"
#include "tether/gcode/analysis/AnalysisTypes.hpp"

#include <string>
#include <vector>

namespace GCode {
struct PlanningSegment;
}

namespace tether::gcode::analysis {

/// @brief Analyze Z-seam positions and consistency from aligned planning data.
Section analyzeZSeam(const std::vector<GCode::PlanningSegment>& planningSegments,
                     const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                     const std::vector<std::string>& gcodeLines,
                     const Options& options = {});

} // namespace tether::gcode::analysis
