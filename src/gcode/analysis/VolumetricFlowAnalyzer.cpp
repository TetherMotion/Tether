/// @file VolumetricFlowAnalyzer.cpp
/// @brief Volumetric flow rate and extrusion consistency analysis.

#include "tether/gcode/analysis/VolumetricFlowAnalyzer.hpp"
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

struct FlowSample {
    int lineNumber = 0;
    double flowRate = 0.0; // mm³/s
    double duration = 0.0;
    double pathLength = 0.0;
    double extrusion = 0.0;
    double linearSpeed = 0.0;
    double deviation = 0.0;
};

} // namespace

Section analyzeVolumetricFlow(const std::vector<GCode::PlanningSegment>& planningSegments,
                              const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                              const std::vector<std::string>& gcodeLines,
                              const Options& options) {
    Section section;

    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return section;
    }

    const std::size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    const auto edeltas = computeEdeltas(gcodeLines);

    const double radius = kFilamentDiameterMm / 2.0;
    const double crossArea = std::numbers::pi * radius * radius;

    std::vector<FlowSample> samples;
    samples.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        if (seg.isRapid) continue;

        const auto& ss = segmentSpeeds[i];
        const std::size_t lineIndex = (ss.lineNumber >= 1)
                                          ? static_cast<std::size_t>(ss.lineNumber - 1)
                                          : std::numeric_limits<std::size_t>::max();
        const double eDelta = (lineIndex < gcodeLines.size()) ? edeltas[lineIndex] : 0.0;
        if (eDelta <= 1e-9) continue;

        const double duration = ss.duration;
        if (duration < 1e-9) continue;

        const double flowRate = (eDelta / duration) * crossArea; // mm³/s
        const double pathLength = seg.segmentLength;
        const double linearSpeed = pathLength > 1e-9 ? pathLength / duration : 0.0;

        samples.push_back({ss.lineNumber, flowRate, duration, pathLength, eDelta,
                           linearSpeed, 0.0});
    }

    if (samples.empty()) {
        return section;
    }

    double minFlow = std::numeric_limits<double>::infinity();
    double maxFlow = -std::numeric_limits<double>::infinity();
    double sumFlow = 0.0;
    for (const auto& s : samples) {
        minFlow = std::min(minFlow, s.flowRate);
        maxFlow = std::max(maxFlow, s.flowRate);
        sumFlow += s.flowRate;
    }
    const double meanFlow = sumFlow / static_cast<double>(samples.size());

    double sumSq = 0.0;
    for (auto& s : samples) {
        s.deviation = s.flowRate - meanFlow;
        sumSq += s.deviation * s.deviation;
    }
    const double variance = sumSq / static_cast<double>(samples.size());
    const double stdDev = std::sqrt(variance);

    section.name = "volumetric_flow";
    section.displayName = "Volumetric Flow";

    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const std::size_t topLimit = fullEvents
                                     ? std::numeric_limits<std::size_t>::max()
                                     : (options.topEventLimit > 0 ? options.topEventLimit : 64);

    section.metrics.push_back(makeMetric("sample_count", static_cast<int64_t>(samples.size())));
    section.metrics.push_back(makeMetric("min_flow_mm3_s", minFlow));
    section.metrics.push_back(makeMetric("max_flow_mm3_s", maxFlow));
    section.metrics.push_back(makeMetric("mean_flow_mm3_s", meanFlow));
    section.metrics.push_back(makeMetric("stddev_flow_mm3_s", stdDev));
    section.metrics.push_back(makeMetric("coefficient_of_variation", meanFlow > 1e-9 ? stdDev / meanFlow : 0.0));

    // Score: lower consistency -> lower score.
    double score = 100.0;
    if (meanFlow > 1e-9) {
        const double cv = stdDev / meanFlow;
        if (cv > 0.5) score -= 25.0;
        else if (cv > 0.3) score -= 15.0;
        else if (cv > 0.15) score -= 5.0;
    }
    if (maxFlow > meanFlow * 3.0) score -= 5.0;
    section.score = std::clamp(score, 0.0, 100.0);
    section.totalEventCount = samples.size();

    if (summaryOnly) {
        section.hasMoreEvents = !samples.empty();
        return section;
    }

    std::vector<FlowSample> sorted = samples;
    std::sort(sorted.begin(), sorted.end(),
              [](const FlowSample& a, const FlowSample& b) {
                  return std::abs(a.deviation) > std::abs(b.deviation);
              });

    const std::size_t eventCount = std::min(topLimit, sorted.size());
    section.hasMoreEvents = sorted.size() > eventCount;

    for (std::size_t i = 0; i < eventCount; ++i) {
        const auto& s = sorted[i];

        Severity sev = Severity::Info;
        if (std::abs(s.deviation) > 2.0 * stdDev) sev = Severity::High;
        else if (std::abs(s.deviation) > stdDev) sev = Severity::Low;

        Event e;
        e.id = std::format("flow:line{}", s.lineNumber);
        e.type = "flow_sample";
        e.severity = sev;
        e.message = std::format(
            "Flow {:.2f} mm³/s at line {} ({}{:.2f} from mean)",
            s.flowRate, s.lineNumber,
            s.deviation > 0.0 ? "+" : "", s.deviation);
        e.metricValue = s.flowRate;
        e.detailsJson = std::format(
            R"!({{"line":{},"flow_rate_mm3_s":{:.3f},"mean_flow_mm3_s":{:.3f},"deviation_mm3_s":{:.3f},"duration_s":{:.3f},"extrusion_mm":{:.3f},"path_length_mm":{:.3f},"linear_speed_mm_s":{:.3f}}})!",
            s.lineNumber, s.flowRate, meanFlow, s.deviation, s.duration, s.extrusion,
            s.pathLength, s.linearSpeed);

        section.events.push_back(std::move(e));
    }

    return section;
}

} // namespace tether::gcode::analysis
