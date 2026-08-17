/// @file StockCollisionAnalyzer.hpp
/// @brief Stock, fixture and build-plate collision/clearance analysis.

#pragma once

#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/SegmentSpeed.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <string>
#include <vector>

namespace tether::gcode::analysis {

/// @brief Analyze stock, fixture, and build-plate collisions / clearances.
///
/// @param planningSegments Motion planning segments from a successful process.
/// @param segmentSpeeds Per-segment timing and speed data (aligned with planningSegments).
/// @param gcodeLines Original G-code lines.
/// @param options Analysis detail options.
/// @return Analysis section for stock and fixture clearance.
Section analyzeStockCollision(const std::vector<GCode::PlanningSegment>& planningSegments,
                              const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                              const std::vector<std::string>& gcodeLines,
                              const Options& options = {});

} // namespace tether::gcode::analysis
