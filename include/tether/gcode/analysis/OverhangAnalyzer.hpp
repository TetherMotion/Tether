/**
 * @file OverhangAnalyzer.hpp
 * @brief Detect overhangs, bridges and support usage from G-code planning segments.
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
 * @brief Analyze overhangs, bridges and support usage from aligned planning data.
 * @param planningSegments Planned motion segments (geometry + Z heights).
 * @param segmentSpeeds    Per-segment speed results aligned with
 *        `planningSegments`.
 * @param gcodeLines       Raw G-code lines for reporting/snippet context.
 * @param options          Analyzer thresholds/options; default-constructed
 *        when omitted.
 * @return Analysis section containing the detected overhang/bridge/support
 *         statistics.
 */
Section analyzeOverhangs(const std::vector<GCode::PlanningSegment>& planningSegments,
                         const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                         const std::vector<std::string>& gcodeLines,
                         const Options& options = {});

} // namespace tether::gcode::analysis
