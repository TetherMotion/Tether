/// @file OverhangAnalyzer.cpp
/// @brief Overhang, bridge and support detection using layer-by-layer geometry.

#include "tether/gcode/analysis/OverhangAnalyzer.hpp"
#include "AnalysisUtil.hpp"
#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <numbers>
#include <vector>

namespace tether::gcode::analysis {

namespace {

constexpr double kOverhangAngleRad = 45.0 * std::numbers::pi / 180.0;
constexpr double kBridgeFactor = 1.5;
constexpr double kMinOverhangMm = 0.1;

struct LayerBox {
    double z = 0.0;
    double minX = 0.0;
    double maxX = 0.0;
    double minY = 0.0;
    double maxY = 0.0;
    std::vector<std::array<double, 4>> extrudingSegs; // x1,y1,x2,y2
};

struct OverhangEvent {
    bool bridge = false;
    double z = 0.0;
    double layerHeight = 0.0;
    double maxDist = 0.0;
    double angle = 0.0;
    int lineNumber = 0;
    double length = 0.0;
    double time = 0.0;
};

double pointToSegmentDistance2D(double px, double py,
                                double x1, double y1,
                                double x2, double y2) {
    const double vx = x2 - x1;
    const double vy = y2 - y1;
    const double wx = px - x1;
    const double wy = py - y1;
    const double c1 = vx * wx + vy * wy;
    if (c1 <= 0.0) {
        const double dx = px - x1;
        const double dy = py - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
    const double c2 = vx * vx + vy * vy;
    if (c2 <= c1) {
        const double dx = px - x2;
        const double dy = py - y2;
        return std::sqrt(dx * dx + dy * dy);
    }
    const double b = c1 / c2;
    const double projX = x1 + b * vx;
    const double projY = y1 + b * vy;
    const double dx = px - projX;
    const double dy = py - projY;
    return std::sqrt(dx * dx + dy * dy);
}

std::pair<double, double> arcMidpoint(const GCode::PlanningSegment& seg) {
    const double startAngle = std::atan2(seg.start.y() - seg.center.y(),
                                         seg.start.x() - seg.center.x());
    const double halfSweep = 0.5 * seg.arcSweep * seg.arcDirection();
    const double midAngle = startAngle + halfSweep;
    return {
        seg.center.x() + seg.arcRadius * std::cos(midAngle),
        seg.center.y() + seg.arcRadius * std::sin(midAngle),
    };
}

} // namespace

Section analyzeOverhangs(const std::vector<GCode::PlanningSegment>& planningSegments,
                         const std::vector<SegmentSpeed>& segmentSpeeds,
                         const std::vector<std::string>& gcodeLines,
                         const Options& options) {
    if (planningSegments.empty()) {
        return Section{};
    }

    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    const std::vector<double> edeltas = computeEdeltas(gcodeLines);
    const std::vector<std::string> features = computeFeatures(gcodeLines);

    // Group extruding non-rapid segments per layer.
    std::map<int64_t, LayerBox> layerMap;
    std::map<int64_t, std::vector<size_t>> layerSegIndexMap;

    double supportExtrusion = 0.0;
    double supportTime = 0.0;
    double supportPath = 0.0;
    uint32_t supportSegments = 0;

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

        auto& layer = layerMap[zKey];
        layer.z = zKey * kLayerZSnap;
        if (layer.extrudingSegs.empty()) {
            layer.minX = layer.maxX = seg.start.x();
            layer.minY = layer.maxY = seg.start.y();
        }
        layer.minX = std::min(layer.minX, std::min(seg.start.x(), seg.end.x()));
        layer.maxX = std::max(layer.maxX, std::max(seg.start.x(), seg.end.x()));
        layer.minY = std::min(layer.minY, std::min(seg.start.y(), seg.end.y()));
        layer.maxY = std::max(layer.maxY, std::max(seg.start.y(), seg.end.y()));
        layer.extrudingSegs.push_back({seg.start.x(), seg.start.y(),
                                       seg.end.x(), seg.end.y()});

        layerSegIndexMap[zKey].push_back(i);

        if (lineIndex < features.size() && features[lineIndex].find("SUPPORT") != std::string::npos) {
            supportExtrusion += std::max(0.0, eDelta);
            supportTime += ss.duration;
            supportPath += seg.segmentLength;
            ++supportSegments;
        }
    }

    if (layerMap.size() < 2) {
        return Section{};
    }

    // Sorted layers.
    std::vector<LayerBox> layers;
    layers.reserve(layerMap.size());
    for (auto& [_, box] : layerMap) layers.push_back(std::move(box));
    std::sort(layers.begin(), layers.end(),
              [](const LayerBox& a, const LayerBox& b) { return a.z < b.z; });

    std::vector<OverhangEvent> events;

    for (size_t li = 1; li < layers.size(); ++li) {
        const auto& prevLayer = layers[li - 1];
        const auto& currLayer = layers[li];
        const double layerHeight = currLayer.z - prevLayer.z;
        if (layerHeight < 1e-9) continue;

        const double allowedOverhang = layerHeight / std::tan(kOverhangAngleRad) + 0.1;
        const auto itIdx = layerSegIndexMap.find(std::llround(currLayer.z / kLayerZSnap));
        if (itIdx == layerSegIndexMap.end()) continue;

        for (size_t idx : itIdx->second) {
            const auto& seg = planningSegments[idx];
            const auto& ss = segmentSpeeds[idx];

            const double x1 = seg.start.x(), y1 = seg.start.y();
            const double x2 = seg.end.x(), y2 = seg.end.y();

            auto testPoint = [&](double px, double py) {
                double best = std::numeric_limits<double>::infinity();
                for (const auto& s : prevLayer.extrudingSegs) {
                    double d = pointToSegmentDistance2D(px, py, s[0], s[1], s[2], s[3]);
                    if (d < best) best = d;
                }
                return best;
            };

            const double startDist = testPoint(x1, y1);
            const double endDist = testPoint(x2, y2);

            double midX, midY;
            if (seg.isArc()) {
                auto [mx, my] = arcMidpoint(seg);
                midX = mx;
                midY = my;
            } else {
                midX = 0.5 * (x1 + x2);
                midY = 0.5 * (y1 + y2);
            }
            const double midDist = testPoint(midX, midY);

            const double maxDist = std::max({startDist, endDist, midDist});
            if (maxDist <= std::max(allowedOverhang, kMinOverhangMm)) continue;

            const double angle = std::atan2(maxDist, layerHeight) * 180.0 / std::numbers::pi;

            const bool isBridge = (startDist <= allowedOverhang && endDist <= allowedOverhang &&
                                   midDist > kBridgeFactor * allowedOverhang &&
                                   seg.motionType == GCode::SegmentMotionType::Linear);

            events.push_back({isBridge, currLayer.z, layerHeight, maxDist, angle,
                              ss.lineNumber, seg.segmentLength, ss.duration});
        }
    }

    Section section;
    section.name = "overhang_bridge_support";
    section.displayName = "Overhangs, Bridges & Supports";

    size_t overhangCount = 0;
    size_t bridgeCount = 0;
    double overhangLength = 0.0;
    double bridgeLength = 0.0;
    double maxAngle = 0.0;
    double angleSum = 0.0;

    for (const auto& ev : events) {
        if (ev.bridge) {
            ++bridgeCount;
            bridgeLength += ev.length;
        } else {
            ++overhangCount;
            overhangLength += ev.length;
        }
        maxAngle = std::max(maxAngle, ev.angle);
        angleSum += ev.angle;
    }

    section.metrics.push_back(makeMetric("overhang_count", static_cast<int64_t>(overhangCount)));
    section.metrics.push_back(makeMetric("bridge_count", static_cast<int64_t>(bridgeCount)));
    section.metrics.push_back(makeMetric("overhang_length_mm", overhangLength));
    section.metrics.push_back(makeMetric("bridge_length_mm", bridgeLength));
    section.metrics.push_back(makeMetric("max_overhang_angle", maxAngle));
    section.metrics.push_back(makeMetric("avg_overhang_angle", !events.empty() ? angleSum / events.size() : 0.0));
    section.metrics.push_back(makeMetric("support_extrusion_mm", supportExtrusion));
    section.metrics.push_back(makeMetric("support_time_s", supportTime));
    section.metrics.push_back(makeMetric("support_path_length_mm", supportPath));
    section.metrics.push_back(makeMetric("support_segment_count", static_cast<int64_t>(supportSegments)));

    double score = 100.0;
    if (maxAngle > 60.0) score -= 25.0;
    else if (maxAngle > 45.0) score -= 15.0;
    score -= static_cast<double>(overhangCount) * 2.0;
    score -= static_cast<double>(bridgeCount) * 5.0;
    section.score = std::clamp(score, 0.0, 100.0);

    section.totalEventCount = events.size();

    const std::string& detail = options.detailLevel;
    const bool summaryOnly = (detail == "summary");
    const bool fullEvents = (detail == "full");

    size_t topLimit = 0;
    if (fullEvents) {
        topLimit = std::numeric_limits<size_t>::max();
    } else if (!summaryOnly) {
        topLimit = (options.topEventLimit > 0) ? options.topEventLimit : 64;
    }

    section.hasMoreEvents = !events.empty() && (summaryOnly || events.size() > topLimit);

    if (summaryOnly) {
        return section;
    }

    std::sort(events.begin(), events.end(),
              [](const OverhangEvent& a, const OverhangEvent& b) { return a.angle > b.angle; });

    const size_t eventCount = std::min(topLimit, events.size());
    for (size_t i = 0; i < eventCount; ++i) {
        const auto& ev = events[i];

        std::string severityLabel = "minor";
        Severity sev = Severity::Low;
        if (ev.angle >= 60.0) {
            severityLabel = "severe";
            sev = Severity::High;
        } else if (ev.angle >= 45.0) {
            severityLabel = "moderate";
            sev = Severity::Medium;
        }

        Event e;
        e.id = std::format("{}:z={:.2f}:line{}", ev.bridge ? "bridge" : "overhang", ev.z, ev.lineNumber);
        e.type = ev.bridge ? "bridge" : "overhang";
        e.severity = sev;
        e.message = std::format("{} at Z={:.2f} mm: {:.1f}° overhang (line {})",
                                ev.bridge ? "Bridge" : "Overhang",
                                ev.z, ev.angle, ev.lineNumber);
        e.metricValue = ev.angle;
        e.detailsJson = std::format(
            R"({{"z":{:.2f},"layer_height":{:.3f},"overhang_distance":{:.3f},"angle":{:.1f},"length":{:.3f},"time_s":{:.3f},"line":{},"severity":"{}"}})",
            ev.z, ev.layerHeight, ev.maxDist, ev.angle, ev.length, ev.time, ev.lineNumber, severityLabel);

        section.events.push_back(std::move(e));
    }

    return section;
}

} // namespace tether::gcode::analysis
