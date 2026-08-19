/// @file FirstLayerAnalyzer.cpp
/// @brief First-layer quality analysis.

#include "tether/gcode/analysis/FirstLayerAnalyzer.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

using GCode::InterpolationPlane;

#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <vector>

namespace tether::gcode::analysis {

namespace {

struct Issue {
    std::string type;
    std::string description;
    Severity severity;
};

double parseTemperature(const std::string& raw, double code) {
    std::string line = stripGcodeComments(raw);
    line = toUpper(line);
    // Search for S word following the M code.
    std::size_t pos = line.find(std::format("M{:.0f}", code));
    if (pos == std::string::npos) return -1.0;
    if (auto s = findWordValue(line.substr(pos), 'S')) return *s;
    return -1.0;
}

} // namespace

Section analyzeFirstLayer(const std::vector<GCode::PlanningSegment>& planningSegments,
                          const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                          const std::vector<std::string>& gcodeLines,
                          const Options& options) {
    Section section;

    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return section;
    }

    const std::size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    const auto edeltas = computeEdeltas(gcodeLines);

    // Identify the lowest extruding layer.
    double firstZ = std::numeric_limits<double>::infinity();
    int firstLayerEndLine = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        if (seg.isRapid) continue;
        const auto& ss = segmentSpeeds[i];
        const std::size_t lineIndex = (ss.lineNumber >= 1)
                                          ? static_cast<std::size_t>(ss.lineNumber - 1)
                                          : std::numeric_limits<std::size_t>::max();
        const double eDelta = (lineIndex < gcodeLines.size()) ? edeltas[lineIndex] : 0.0;
        if (eDelta <= 1e-9) continue;

        if (seg.start.z() < firstZ) firstZ = seg.start.z();
        if (ss.lineNumber > firstLayerEndLine) firstLayerEndLine = ss.lineNumber;
    }

    if (!std::isfinite(firstZ)) {
        return section;
    }

    const double zTol = 0.05;
    double totalExtrusion = 0.0;
    double extrudingPath = 0.0;
    double totalPath = 0.0;
    double travelPath = 0.0;
    double firstLayerTime = 0.0;
    std::uint32_t moveCount = 0;
    std::uint32_t extrudingMoves = 0;
    double feedRateSum = 0.0;
    double maxFeed = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        const auto& ss = segmentSpeeds[i];
        if (std::abs(seg.start.z() - firstZ) > zTol) continue;

        totalPath += seg.segmentLength;
        firstLayerTime += ss.duration;
        ++moveCount;
        feedRateSum += seg.feedRate;
        maxFeed = std::max(maxFeed, seg.feedRate);

        const std::size_t lineIndex = (ss.lineNumber >= 1)
                                          ? static_cast<std::size_t>(ss.lineNumber - 1)
                                          : std::numeric_limits<std::size_t>::max();
        const double eDelta = (lineIndex < gcodeLines.size()) ? edeltas[lineIndex] : 0.0;
        if (eDelta > 1e-9) {
            totalExtrusion += eDelta;
            extrudingPath += seg.segmentLength;
            ++extrudingMoves;
        } else if (seg.isRapid) {
            travelPath += seg.segmentLength;
        }
    }

    // Parse first-layer temperatures and fan from gcode comments/commands.
    int bedTemp = 0;
    int hotendTemp = 0;
    int fanSpeed = 0;
    for (int i = 0; i < firstLayerEndLine && i < static_cast<int>(gcodeLines.size()); ++i) {
        const std::string line = stripGcodeComments(gcodeLines[i]);

        if (auto v = findWordValue(line, 'M')) {
            if ((*v == 140.0 || *v == 190.0)) {
                if (auto s = findWordValue(line, 'S')) bedTemp = static_cast<int>(std::round(*s));
            } else if (*v == 104.0 || *v == 109.0) {
                if (auto s = findWordValue(line, 'S')) hotendTemp = static_cast<int>(std::round(*s));
            } else if (*v == 106.0) {
                if (auto s = findWordValue(line, 'S')) fanSpeed = static_cast<int>(std::round(*s));
            } else if (*v == 107.0) {
                fanSpeed = 0;
            }
        }
    }

    std::vector<Issue> issues;

    if (bedTemp > 0 && bedTemp < 50) {
        issues.push_back({"bed_temp",
                          std::format("Bed temperature {}°C is low for reliable first-layer adhesion", bedTemp),
                          Severity::Low});
    }
    if (hotendTemp > 0 && hotendTemp < 180) {
        issues.push_back({"hotend_temp",
                          std::format("Hotend temperature {}°C is low for proper filament flow", hotendTemp),
                          Severity::Low});
    }
    if (fanSpeed > 50) {
        issues.push_back({"fan_speed",
                          std::format("First-layer fan speed {}/255 may reduce adhesion", fanSpeed),
                          Severity::Low});
    }
    if (maxFeed > 3000.0) {
        issues.push_back({"feed_rate",
                          std::format("First-layer feed rate {:.0f} mm/min is high for adhesion", maxFeed),
                          Severity::Low});
    }
    if (extrudingMoves == 0) {
        issues.push_back({"no_extrusion", "No extrusion detected in the first layer",
                          Severity::High});
    }

    section.name = "first_layer";
    section.displayName = "First Layer Quality";

    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const std::size_t topLimit = fullEvents
                                     ? std::numeric_limits<std::size_t>::max()
                                     : (options.topEventLimit > 0 ? options.topEventLimit : 64);

    section.metrics.push_back(makeMetric("first_layer_z", firstZ));
    section.metrics.push_back(makeMetric("bed_temp_c", static_cast<int64_t>(bedTemp)));
    section.metrics.push_back(makeMetric("hotend_temp_c", static_cast<int64_t>(hotendTemp)));
    section.metrics.push_back(makeMetric("fan_speed", static_cast<int64_t>(fanSpeed)));
    section.metrics.push_back(makeMetric("total_extrusion_mm", totalExtrusion));
    section.metrics.push_back(makeMetric("extruding_path_mm", extrudingPath));
    section.metrics.push_back(makeMetric("travel_path_mm", travelPath));
    section.metrics.push_back(makeMetric("total_path_mm", totalPath));
    section.metrics.push_back(makeMetric("first_layer_time_s", firstLayerTime));
    section.metrics.push_back(makeMetric("move_count", static_cast<int64_t>(moveCount)));
    section.metrics.push_back(makeMetric("extruding_move_count", static_cast<int64_t>(extrudingMoves)));
    section.metrics.push_back(makeMetric("max_feed_rate_mm_min", maxFeed));
    section.metrics.push_back(makeMetric("avg_feed_rate_mm_min", moveCount > 0 ? feedRateSum / static_cast<double>(moveCount) : 0.0));

    double score = 100.0;
    for (const auto& issue : issues) {
        if (issue.severity == Severity::High) score -= 30.0;
        else score -= 12.0;
    }
    section.score = std::clamp(score, 0.0, 100.0);
    section.totalEventCount = issues.size();
    section.hasMoreEvents = issues.size() > std::min(topLimit, issues.size());

    if (summaryOnly) {
        section.hasMoreEvents = !issues.empty();
        return section;
    }

    const std::size_t eventCount = std::min(topLimit, issues.size());
    section.hasMoreEvents = issues.size() > eventCount;

    for (std::size_t i = 0; i < eventCount; ++i) {
        const auto& issue = issues[i];

        Event e;
        e.id = std::format("first_layer:{}", issue.type);
        e.type = issue.type;
        e.severity = issue.severity;
        e.message = issue.description;
        e.metricValue = 0.0;
        e.detailsJson = std::format(
            R"!({{"type":"{}","first_layer_z":{:.2f},"bed_temp":{},"hotend_temp":{},"fan_speed":{},"max_feed_rate":{:.0f}}})!",
            issue.type, firstZ, bedTemp, hotendTemp, fanSpeed, maxFeed);

        section.events.push_back(std::move(e));
    }

    return section;
}

} // namespace tether::gcode::analysis
