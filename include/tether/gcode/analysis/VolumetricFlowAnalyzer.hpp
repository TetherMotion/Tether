/// @file VolumetricFlowAnalyzer.hpp
/// @brief Volumetric flow rate and extrusion consistency analysis.

#pragma once

#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"
#include "tether/gcode/SegmentSpeed.hpp"

#include <string>
#include <vector>

namespace tether::gcode::analysis {

/// @brief Analyze volumetric flow consistency across extrusion moves.
///
/// @param planningSegments Parsed motion segments (aligned with segmentSpeeds).
/// @param segmentSpeeds Per-segment timing and speed data.
/// @param gcodeLines Original G-code text split into lines.
/// @param options Detail-level options.
/// @return A populated analysis section, or a default Section{} if no data.
Section analyzeVolumetricFlow(const std::vector<GCode::PlanningSegment>& planningSegments,
                              const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                              const std::vector<std::string>& gcodeLines,
                              const Options& options = {});

} // namespace tether::gcode::analysis
