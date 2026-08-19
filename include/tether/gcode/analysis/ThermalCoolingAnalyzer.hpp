/// @file ThermalCoolingAnalyzer.hpp
/// @brief Per-layer thermal and cooling time analysis.

#pragma once

#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/SegmentSpeed.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <string>
#include <vector>

namespace tether::gcode::analysis {

/// @brief Analyze per-layer thermal and cooling behavior.
///
/// @param planningSegments Motion planning segments from a successful process.
/// @param segmentSpeeds Per-segment timing and speed data (aligned with planningSegments).
/// @param gcodeLines Original G-code lines.
/// @param options Analysis detail options.
/// @return Analysis section for thermal and cooling.
Section analyzeThermalCooling(const std::vector<GCode::PlanningSegment>& planningSegments,
                              const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                              const std::vector<std::string>& gcodeLines,
                              const Options& options = {});

} // namespace tether::gcode::analysis
