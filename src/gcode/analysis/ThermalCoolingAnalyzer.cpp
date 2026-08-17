/// @file ThermalCoolingAnalyzer.cpp
/// @brief Per-layer thermal and cooling time analysis.

#include "tether/gcode/analysis/ThermalCoolingAnalyzer.hpp"
#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <vector>

namespace tether::gcode::analysis {

namespace {

struct LayerCooling {
    double z = 0.0;
    double time = 0.0;
    double extrusion = 0.0;
    double path = 0.0;
    int firstLine = std::numeric_limits<int>::max();
    int lastLine = 0;
    double fanTime = 0.0;
    double fanSum = 0.0;
    double maxFan = 0.0;
};

} // namespace

Section analyzeThermalCooling(const std::vector<GCode::PlanningSegment>& planningSegments,
                              const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                              const std::vector<std::string>& gcodeLines,
                              const Options& options) {
    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return {};
    }

    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    const auto edeltas = computeEdeltas(gcodeLines);

    // Current fan speed for each gcode line.
    std::vector<int> fanByLine(gcodeLines.size(), 0);
    int currentFan = 0;
    for (size_t i = 0; i < gcodeLines.size(); ++i) {
        const std::string line = stripGcodeComments(gcodeLines[i]);
        if (auto m = findWordValue(line, 'M')) {
            if (*m == 106.0) {
                if (auto s = findWordValue(line, 'S')) {
                    currentFan = static_cast<int>(std::round(std::clamp(*s, 0.0, 255.0)));
                }
            } else if (*m == 107.0) {
                currentFan = 0;
            }
        }
        fanByLine[i] = currentFan;
    }

    // Aggregate per layer.
    std::map<int64_t, LayerCooling> layers;
    for (size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        const auto& ss = segmentSpeeds[i];
        if (seg.isRapid) continue;

        const size_t lineIndex = (ss.lineNumber >= 1)
                                     ? static_cast<size_t>(ss.lineNumber - 1)
                                     : std::numeric_limits<size_t>::max();
        const double eDelta = (lineIndex < gcodeLines.size()) ? edeltas[lineIndex] : 0.0;
        if (eDelta <= 1e-9) continue;

        const int64_t zKey = std::llround(seg.start.z() / kLayerZSnap);
        auto& layer = layers[zKey];
        layer.z = zKey * kLayerZSnap;
        layer.time += ss.duration;
        layer.extrusion += eDelta;
        layer.path += seg.segmentLength;
        layer.firstLine = std::min(layer.firstLine, ss.lineNumber);
        layer.lastLine = std::max(layer.lastLine, ss.lineNumber);

        const int fan = (lineIndex < fanByLine.size()) ? fanByLine[lineIndex] : 0;
        if (ss.duration > 0) {
            layer.fanTime += ss.duration;
            layer.fanSum += fan * ss.duration;
            layer.maxFan = std::max(layer.maxFan, static_cast<double>(fan));
        }
    }

    if (layers.empty()) return {};

    constexpr double kFastLayerTime = 5.0;  // seconds
    constexpr double kSlowLayerTime = 60.0; // seconds
    constexpr double kAdequateFan = 50.0;   // /255

    // Build events for layers with concerning thermal/cooling behavior.
    std::vector<std::pair<const int64_t, LayerCooling>*> sortedLayers;
    for (auto& it : layers) sortedLayers.push_back(&it);

    double totalLayerTime = 0.0;
    double totalExtrusion = 0.0;
    size_t fastLayers = 0;
    size_t slowLayers = 0;
    size_t underCooled = 0;
    double maxFan = 0.0;
    double fanTimeWeighted = 0.0;

    for (const auto& it : sortedLayers) {
        const auto& l = it->second;
        totalLayerTime += l.time;
        totalExtrusion += l.extrusion;
        if (l.time < kFastLayerTime) ++fastLayers;
        if (l.time > kSlowLayerTime) ++slowLayers;
        const double avgFan = l.fanTime > 0.0 ? l.fanSum / l.fanTime : 0.0;
        if (l.time < kFastLayerTime && avgFan < kAdequateFan) ++underCooled;
        maxFan = std::max(maxFan, l.maxFan);
        fanTimeWeighted += l.fanSum;
    }

    Section section;
    section.name = "thermal_cooling";
    section.displayName = "Thermal & Cooling";

    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const size_t topLimit = (options.topEventLimit > 0)
                                ? options.topEventLimit
                                : (fullEvents ? std::numeric_limits<size_t>::max() : 64);

    // Min/max/avg layer time.
    std::vector<double> times;
    times.reserve(sortedLayers.size());
    for (const auto& it : sortedLayers) times.push_back(it->second.time);
    std::ranges::sort(times);

    section.metrics.push_back(makeMetric("min_layer_time_s", times.front()));
    section.metrics.push_back(makeMetric("max_layer_time_s", times.back()));
    section.metrics.push_back(makeMetric("avg_layer_time_s", totalLayerTime / sortedLayers.size()));
    section.metrics.push_back(makeMetric("total_layer_time_s", totalLayerTime));
    section.metrics.push_back(makeMetric("total_extrusion_mm", totalExtrusion));
    section.metrics.push_back(makeMetric("layer_count", static_cast<int64_t>(sortedLayers.size())));
    section.metrics.push_back(makeMetric("fast_layers_count", static_cast<int64_t>(fastLayers)));
    section.metrics.push_back(makeMetric("slow_layers_count", static_cast<int64_t>(slowLayers)));
    section.metrics.push_back(makeMetric("undercooled_layers", static_cast<int64_t>(underCooled)));
    section.metrics.push_back(makeMetric("max_fan_speed", maxFan));
    section.metrics.push_back(makeMetric("avg_fan_speed", totalLayerTime > 0.0 ? fanTimeWeighted / totalLayerTime : 0.0));

    double score = 100.0;
    if (underCooled > 0) score -= std::min(30.0, static_cast<double>(underCooled) * 5.0);
    if (fastLayers > 0) score -= std::min(20.0, static_cast<double>(fastLayers) * 3.0);
    if (slowLayers > 0) score -= std::min(10.0, static_cast<double>(slowLayers) * 2.0);
    section.score = std::clamp(score, 0.0, 100.0);

    // Sort layers by time ascending for top events (shortest = most concerning).
    std::ranges::sort(sortedLayers,
                      [](const auto* a, const auto* b) {
                          if (a->second.time != b->second.time) return a->second.time < b->second.time;
                          return a->first < b->first;
                      });

    section.totalEventCount = sortedLayers.size();
    section.hasMoreEvents = sortedLayers.size() > topLimit;

    if (summaryOnly) return section;

    const size_t eventCount = std::min(topLimit, sortedLayers.size());
    section.events.reserve(eventCount);
    for (size_t i = 0; i < eventCount; ++i) {
        const auto& l = sortedLayers[i]->second;
        const double avgFan = l.fanTime > 0.0 ? l.fanSum / l.fanTime : 0.0;
        const bool under = l.time < kFastLayerTime && avgFan < kAdequateFan;

        Severity sev = Severity::Info;
        if (under) sev = Severity::High;
        else if (l.time < kFastLayerTime) sev = Severity::Low;

        section.events.push_back(Event{
            .id = std::format("thermal:z={:.2f}:line{}", l.z, l.firstLine),
            .type = "layer_cooling",
            .severity = sev,
            .message = std::format(
                "Layer Z={:.2f} mm: time {:.1f} s, fan {:.0f}/255{}{}",
                l.z, l.time, avgFan,
                under ? " — under-cooled" : "",
                l.time > kSlowLayerTime ? " — slow layer" : ""),
            .metricValue = l.time,
            .detailsJson = std::format(
                R"({{"z":{:.2f},"time_s":{:.3f},"extrusion_mm":{:.3f},"path_mm":{:.3f},"avg_fan":{:.0f},"max_fan":{:.0f},"first_line":{},"undercooled":{}}})" ,
                l.z, l.time, l.extrusion, l.path, avgFan, l.maxFan, l.firstLine, under)});
    }

    return section;
}

} // namespace tether::gcode::analysis
