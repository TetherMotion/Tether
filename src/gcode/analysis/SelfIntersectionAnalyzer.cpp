/// @file SelfIntersectionAnalyzer.cpp
/// @brief Detect self-intersections and crossing toolpaths.

#include "tether/gcode/analysis/SelfIntersectionAnalyzer.hpp"
#include "AnalysisUtil.hpp"
#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <optional>
#include <vector>

namespace tether::gcode::analysis {

namespace {

struct Segment2D {
    double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
    double z = 0.0;
    bool isRapid = false;
    int lineNumber = 0;
    size_t index = 0;
};

struct IntersectionEvent {
    double x = 0.0, y = 0.0, z = 0.0;
    int line1 = 0;
    int line2 = 0;
    bool isRapid = false;
    double distanceToStart = 0.0;
};

std::optional<std::pair<double, double>> lineIntersection(
    const Segment2D& a, const Segment2D& b) {
    const double x1 = a.x1, y1 = a.y1, x2 = a.x2, y2 = a.y2;
    const double x3 = b.x1, y3 = b.y1, x4 = b.x2, y4 = b.y2;

    const double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 1e-10) return std::nullopt; // parallel or collinear

    const double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    const double u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom;

    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        const double px = x1 + t * (x2 - x1);
        const double py = y1 + t * (y2 - y1);
        return std::make_optional(std::make_pair(px, py));
    }
    return std::nullopt;
}

} // namespace

Section analyzeSelfIntersections(const std::vector<GCode::PlanningSegment>& planningSegments,
                                 const std::vector<SegmentSpeed>& segmentSpeeds,
                                 const std::vector<std::string>& gcodeLines,
                                 const Options& options) {
    if (planningSegments.empty()) {
        return Section{};
    }

    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    [[maybe_unused]] const std::vector<double> edeltas = computeEdeltas(gcodeLines);

    [[maybe_unused]] constexpr double kZThreshold = 0.5; // mm
    constexpr size_t kMaxPerLayer = 500;

    // Group 2D segments by layer Z key.
    std::map<int64_t, std::vector<Segment2D>> layerSegs;
    for (size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        const auto& ss = segmentSpeeds[i];

        [[maybe_unused]] const double eDelta = (ss.lineNumber >= 1 && static_cast<size_t>(ss.lineNumber) <= gcodeLines.size())
                                                   ? edeltas[ss.lineNumber - 1]
                                                   : 0.0;
        // Include all non-trivial motion. For 3DP include both extruding and
        // non-extruding moves; for CNC include rapids and cuts.
        const double path = seg.segmentLength;
        if (path < 1e-1) continue;

        const int64_t zKey = std::llround(seg.start.z() / kLayerZSnap);
        Segment2D s;
        s.x1 = seg.start.x();
        s.y1 = seg.start.y();
        s.x2 = seg.end.x();
        s.y2 = seg.end.y();
        s.z = seg.start.z();
        s.isRapid = seg.isRapid;
        s.lineNumber = ss.lineNumber;
        s.index = i;
        layerSegs[zKey].push_back(s);
    }

    std::vector<IntersectionEvent> intersections;

    for (auto& [_, segs] : layerSegs) {
        // Limit per layer to keep runtime bounded (O(n²)).
        if (segs.size() > kMaxPerLayer) {
            segs.resize(kMaxPerLayer);
        }

        for (size_t i = 0; i < segs.size(); ++i) {
            for (size_t j = i + 2; j < segs.size(); ++j) {
                // Arcs are approximated by their chord for the intersection test.
                if (auto point = lineIntersection(segs[i], segs[j])) {
                    const double dist = std::sqrt(
                        (point->first - segs[i].x1) * (point->first - segs[i].x1) +
                        (point->second - segs[i].y1) * (point->second - segs[i].y1));
                    intersections.push_back({point->first, point->second, segs[i].z,
                                             segs[i].lineNumber, segs[j].lineNumber,
                                             segs[i].isRapid || segs[j].isRapid, dist});
                }
            }
        }
    }

    Section section;
    section.name = "path_intersections";
    section.displayName = "Path Intersections";

    size_t cuttingCount = 0;
    size_t rapidCount = 0;
    for (const auto& ev : intersections) {
        if (ev.isRapid) ++rapidCount;
        else ++cuttingCount;
    }

    section.metrics.push_back(makeMetric("intersection_count", static_cast<int64_t>(intersections.size())));
    section.metrics.push_back(makeMetric("cutting_intersections", static_cast<int64_t>(cuttingCount)));
    section.metrics.push_back(makeMetric("rapid_intersections", static_cast<int64_t>(rapidCount)));
    section.metrics.push_back(makeMetric("layers_checked", static_cast<int64_t>(layerSegs.size())));

    double score = 100.0;
    score -= static_cast<double>(cuttingCount) * 10.0;
    score -= static_cast<double>(rapidCount) * 2.0;
    section.score = std::clamp(score, 0.0, 100.0);

    section.totalEventCount = intersections.size();

    const std::string& detail = options.detailLevel;
    const bool summaryOnly = (detail == "summary");
    const bool fullEvents = (detail == "full");

    size_t topLimit = 0;
    if (fullEvents) {
        topLimit = std::numeric_limits<size_t>::max();
    } else if (!summaryOnly) {
        topLimit = (options.topEventLimit > 0) ? options.topEventLimit : 64;
    }

    section.hasMoreEvents = !intersections.empty() && (summaryOnly || intersections.size() > topLimit);

    if (summaryOnly) {
        return section;
    }

    std::sort(intersections.begin(), intersections.end(),
              [](const IntersectionEvent& a, const IntersectionEvent& b) {
                  if (a.isRapid != b.isRapid) return !a.isRapid; // cutting first
                  return a.distanceToStart > b.distanceToStart;
              });

    const size_t eventCount = std::min(topLimit, intersections.size());
    for (size_t i = 0; i < eventCount; ++i) {
        const auto& ev = intersections[i];

        Event e;
        e.id = std::format("intersection:z={:.2f}:l{}:l{}", ev.z, ev.line1, ev.line2);
        e.type = "intersection";
        e.severity = ev.isRapid ? Severity::Medium : Severity::High;
        e.message = std::format(
            "{} intersection at Z={:.2f} mm between lines {} and {} ({:.1f}, {:.1f})",
            ev.isRapid ? "Rapid" : "Cutting",
            ev.z, ev.line1, ev.line2, ev.x, ev.y);
        e.metricValue = ev.distanceToStart;
        e.detailsJson = std::format(
            R"({{"x":{:.3f},"y":{:.3f},"z":{:.2f},"line1":{},"line2":{},"is_rapid":{},"type":"{}"}})",
            ev.x, ev.y, ev.z, ev.line1, ev.line2, ev.isRapid,
            ev.isRapid ? "rapid" : "cutting");

        section.events.push_back(std::move(e));
    }

    return section;
}

} // namespace tether::gcode::analysis
