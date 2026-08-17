/// @file PatternAnalyzer.hpp
/// @brief Detect recurring toolpath patterns (spirals, concentric contours,
///        zigzags, arcs and linear moves) from parsed G-code segments.

#pragma once

#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"
#include "tether/gcode/SegmentSpeed.hpp"

#include <string>
#include <vector>

namespace tether::gcode::analysis {

/// @brief Analyze toolpath patterns from aligned planning segments and speeds.
///
/// @param planningSegments Parsed motion segments (aligned with segmentSpeeds).
/// @param segmentSpeeds Per-segment timing and speed data.
/// @param gcodeLines Original G-code text split into lines (unused but kept for API parity).
/// @param options Detail-level options.
/// @return A populated analysis section, or a default Section{} if no data.
Section analyzePatterns(const std::vector<GCode::PlanningSegment>& planningSegments,
                        const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                        const std::vector<std::string>& /*gcodeLines*/,
                        const Options& options = {});

} // namespace tether::gcode::analysis
