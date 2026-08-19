/// @file ThermalSimulationAnalyzer.cpp
/// @brief Basic layer-wise thermal and warping simulation.

#include "tether/gcode/analysis/ThermalSimulationAnalyzer.hpp"
#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <numbers>
#include <vector>

namespace tether::gcode::analysis {

namespace {

struct LayerThermal {
    double z = 0.0;
    double time = 0.0;
    double startTime = 0.0;
    double endTime = 0.0;
    double area = 0.0;
    double temp = 0.0;
    int lineNumber = 0;
};

} // namespace

Section analyzeThermalSimulation(const std::vector<GCode::PlanningSegment>& planningSegments,
                                 const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                                 const std::vector<std::string>& gcodeLines,
                                 const Options& options) {
    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return {};
    }

    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    const auto edeltas = computeEdeltas(gcodeLines);

    // Per-line environment state.
    double hotendTemp = 210.0;
    double bedTemp = 60.0;
    double fanSpeed = 0.0;
    double ambientTemp = 25.0;

    // Look for explicit ambient setting in comments; otherwise default.
    for (const auto& line : gcodeLines) {
        if (auto v = extractFeatureTag(line, "AMBIENT_TEMP")) {
            try { ambientTemp = std::stod(*v); } catch (...) {}
        }
    }

    // Build per-line state arrays for hotend, bed, fan.
    std::vector<double> hotendByLine(gcodeLines.size(), hotendTemp);
    std::vector<double> bedByLine(gcodeLines.size(), bedTemp);
    std::vector<double> fanByLine(gcodeLines.size(), fanSpeed);
    for (size_t i = 0; i < gcodeLines.size(); ++i) {
        const std::string line = stripGcodeComments(gcodeLines[i]);

        if (auto m = findWordValue(line, 'M')) {
            if (*m == 104.0 || *m == 109.0) {
                if (auto s = findWordValue(line, 'S')) {
                    hotendTemp = *s;
                }
            } else if (*m == 140.0 || *m == 190.0) {
                if (auto s = findWordValue(line, 'S')) {
                    bedTemp = *s;
                }
            } else if (*m == 106.0) {
                if (auto s = findWordValue(line, 'S')) {
                    fanSpeed = std::clamp(*s, 0.0, 255.0);
                }
            } else if (*m == 107.0) {
                fanSpeed = 0.0;
            }
        }

        hotendByLine[i] = hotendTemp;
        bedByLine[i] = bedTemp;
        fanByLine[i] = fanSpeed;
    }

    // Aggregate per-layer: collect start/end times and XY area (AABB).
    std::map<int64_t, LayerThermal> layers;
    double cumulativeTime = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        const auto& ss = segmentSpeeds[i];

        const size_t lineIndex = (ss.lineNumber >= 1)
                                     ? static_cast<size_t>(ss.lineNumber - 1)
                                     : std::numeric_limits<size_t>::max();
        const double eDelta = (lineIndex < gcodeLines.size()) ? edeltas[lineIndex] : 0.0;

        const double startT = cumulativeTime;
        cumulativeTime += ss.duration;
        const double endT = cumulativeTime;

        const int64_t zKey = std::llround(seg.start.z() / kLayerZSnap);
        auto& layer = layers[zKey];
        if (layer.startTime > startT) layer.startTime = startT;
        if (layer.endTime < endT) layer.endTime = endT;
        layer.z = zKey * kLayerZSnap;
        layer.time += ss.duration;
        if (eDelta > 1e-9) {
            layer.area += (seg.end.x() - seg.start.x()) * (seg.end.y() - seg.start.y());
            if (layer.lineNumber == 0) layer.lineNumber = ss.lineNumber;
        }
    }

    if (layers.size() < 2) return {};

    std::vector<LayerThermal> ordered;
    ordered.reserve(layers.size());
    for (auto& [_, l] : layers) ordered.push_back(std::move(l));
    std::ranges::sort(ordered,
                      [](const LayerThermal& a, const LayerThermal& b) { return a.startTime < b.startTime; });

    double lastExtrusionTime = 0.0;
    double totalArea = 0.0;
    for (auto& l : ordered) {
        totalArea += std::abs(l.area);
        const double timeSinceExtrusion = l.startTime - lastExtrusionTime;
        lastExtrusionTime = l.endTime;

        // Representative line for environment state (first extruding line).
        const size_t lineIndex = (l.lineNumber >= 1) ? static_cast<size_t>(l.lineNumber - 1) : 0;
        const double hotend = (lineIndex < hotendByLine.size()) ? hotendByLine[lineIndex] : hotendByLine.back();
        const double bed = (lineIndex < bedByLine.size()) ? bedByLine[lineIndex] : bedByLine.back();
        const double fan = (lineIndex < fanByLine.size()) ? fanByLine[lineIndex] : fanByLine.back();

        const double fanCooling = (fan / 255.0) * 30.0;
        const double bedWarming = std::max(0.0, (bed - ambientTemp) * std::exp(-l.z * 0.1));
        const double coolingFactor = std::exp(-timeSinceExtrusion * 0.05);
        const double temp = ambientTemp + (hotend - ambientTemp) * coolingFactor - fanCooling + bedWarming;
        l.temp = temp;
    }

    // Warping estimate: large thermal gradients and large flat areas increase risk.
    double maxGradient = 0.0;
    double maxArea = 0.0;
    for (size_t i = 1; i < ordered.size(); ++i) {
        maxGradient = std::max(maxGradient, std::abs(ordered[i].temp - ordered[i - 1].temp));
    }
    for (const auto& l : ordered) {
        maxArea = std::max(maxArea, std::abs(l.area));
    }
    const double avgArea = totalArea / ordered.size();
    const auto largeFlatAreaCount = std::count_if(
        ordered.begin(), ordered.end(),
        [&](const LayerThermal& l) { return std::abs(l.area) > avgArea * 1.5; });

    Section section;
    section.name = "thermal_simulation";
    section.displayName = "Thermal Simulation";

    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const size_t topLimit = (options.topEventLimit > 0)
                                ? options.topEventLimit
                                : (fullEvents ? std::numeric_limits<size_t>::max() : 64);

    const double minTemp = std::min_element(ordered.begin(), ordered.end(),
                                            [](const auto& a, const auto& b) { return a.temp < b.temp; })->temp;
    const double maxTemp = std::max_element(ordered.begin(), ordered.end(),
                                            [](const auto& a, const auto& b) { return a.temp < b.temp; })->temp;
    double avgTemp = 0.0;
    for (const auto& l : ordered) avgTemp += l.temp;
    avgTemp /= ordered.size();

    // Hot zones: layers whose temperature exceeds the average by 10%.
    std::vector<LayerThermal> hotZones;
    const double hotThreshold = avgTemp * 1.1;
    for (const auto& l : ordered) {
        if (l.temp > hotThreshold) hotZones.push_back(l);
    }
    std::ranges::sort(hotZones,
                      [](const auto& a, const auto& b) { return a.temp > b.temp; });

    section.metrics.push_back(makeMetric("min_temp_c", minTemp));
    section.metrics.push_back(makeMetric("max_temp_c", maxTemp));
    section.metrics.push_back(makeMetric("avg_temp_c", avgTemp));
    section.metrics.push_back(makeMetric("max_thermal_gradient_c", maxGradient));
    section.metrics.push_back(makeMetric("hot_zone_count", static_cast<int64_t>(hotZones.size())));
    section.metrics.push_back(makeMetric("large_flat_area_count", static_cast<int64_t>(largeFlatAreaCount)));
    section.metrics.push_back(makeMetric("max_flat_area_mm2", maxArea));

    // Warp risk score 0..100.
    double warpRisk = 0.0;
    if (maxGradient > 20.0) warpRisk += 30.0;
    else if (maxGradient > 10.0) warpRisk += 15.0;
    if (largeFlatAreaCount > static_cast<long>(ordered.size()) * 0.3) warpRisk += 20.0;
    if (maxTemp > avgTemp * 1.15) warpRisk += 15.0;
    section.metrics.push_back(makeMetric("warp_risk_score", std::clamp(warpRisk, 0.0, 100.0)));

    double score = 100.0 - warpRisk;
    if (hotZones.size() > ordered.size() * 0.2) score -= 10.0;
    section.score = std::clamp(score, 0.0, 100.0);

    section.totalEventCount = hotZones.size();
    section.hasMoreEvents = hotZones.size() > topLimit;

    if (summaryOnly) return section;

    const size_t eventCount = std::min(topLimit, hotZones.size());
    section.events.reserve(eventCount);
    for (size_t i = 0; i < eventCount; ++i) {
        const auto& l = hotZones[i];

        Severity sev = Severity::Info;
        if (l.temp > avgTemp * 1.25) sev = Severity::High;
        else if (l.temp > avgTemp * 1.15) sev = Severity::Medium;
        else sev = Severity::Low;

        section.events.push_back(Event{
            .id = std::format("thermal:z={:.2f}:line{}", l.z, l.lineNumber),
            .type = "hot_zone",
            .severity = sev,
            .message = std::format(
                "Hot zone at Z={:.2f} mm: {:.1f}°C (avg {:.1f}°C)",
                l.z, l.temp, avgTemp),
            .metricValue = l.temp,
            .detailsJson = std::format(
                R"({{"z":{:.2f},"temp_c":{:.2f},"avg_temp_c":{:.2f},"time_s":{:.3f},"area_mm2":{:.2f},"line":{}}})" ,
                l.z, l.temp, avgTemp, l.time, std::abs(l.area), l.lineNumber)});
    }

    return section;
}

} // namespace tether::gcode::analysis
