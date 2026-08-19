/// @file ZSeamAnalyzer.cpp
/// @brief Z-seam detection and consistency scoring.

#include "tether/gcode/analysis/ZSeamAnalyzer.hpp"
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

struct SeamPoint {
    double z = 0.0;
    double x = 0.0;
    double y = 0.0;
    int lineNumber = 0;
    bool aligned = false;
    double distanceToPrevious = 0.0;
};

} // namespace

Section analyzeZSeam(const std::vector<GCode::PlanningSegment>& planningSegments,
                     const std::vector<SegmentSpeed>& segmentSpeeds,
                     const std::vector<std::string>& gcodeLines,
                     const Options& options) {
    if (planningSegments.empty()) {
        return Section{};
    }

    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());
    const std::vector<double> edeltas = computeEdeltas(gcodeLines);

    // Find the first extruding segment per layer.
    std::map<int64_t, SeamPoint> seamMap;
    for (size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        if (seg.isRapid) continue;

        const auto& ss = segmentSpeeds[i];
        const size_t lineIndex = (ss.lineNumber >= 1)
                                     ? static_cast<size_t>(ss.lineNumber - 1)
                                     : std::numeric_limits<size_t>::max();
        const double eDelta = (lineIndex < gcodeLines.size()) ? edeltas[lineIndex] : 0.0;
        if (eDelta <= 1e-9) continue;

        const int64_t zKey = std::llround(seg.start.z() / kLayerZSnap);
        if (seamMap.find(zKey) != seamMap.end()) continue; // first seam wins

        seamMap[zKey] = {seg.start.z(), seg.start.x(), seg.start.y(), ss.lineNumber, false, 0.0};
    }

    if (seamMap.size() < 2) {
        return Section{};
    }

    std::vector<SeamPoint> seams;
    seams.reserve(seamMap.size());
    for (auto& [_, p] : seamMap) seams.push_back(std::move(p));
    std::sort(seams.begin(), seams.end(),
              [](const SeamPoint& a, const SeamPoint& b) { return a.z < b.z; });

    constexpr double kAlignedToleranceMm = 2.0;

    [[maybe_unused]] double totalDist = 0.0;
    double maxDist = 0.0;
    double distSum = 0.0;
    double distSumSq = 0.0;
    size_t alignedCount = 0;

    for (size_t i = 1; i < seams.size(); ++i) {
        const double dx = seams[i].x - seams[i - 1].x;
        const double dy = seams[i].y - seams[i - 1].y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        seams[i].distanceToPrevious = dist;
        seams[i].aligned = dist < kAlignedToleranceMm;
        if (seams[i].aligned) ++alignedCount;
        totalDist += dist;
        maxDist = std::max(maxDist, dist);
        distSum += dist;
        distSumSq += dist * dist;
    }

    const size_t pairCount = seams.size() - 1;
    const double avgDist = pairCount > 0 ? distSum / pairCount : 0.0;
    const double variance = pairCount > 0 ? (distSumSq / pairCount) - (avgDist * avgDist) : 0.0;

    // Centroid and max dispersion.
    double cx = 0.0, cy = 0.0;
    for (const auto& s : seams) { cx += s.x; cy += s.y; }
    cx /= seams.size();
    cy /= seams.size();
    double maxDispersion = 0.0;
    for (const auto& s : seams) {
        const double d = std::sqrt((s.x - cx) * (s.x - cx) + (s.y - cy) * (s.y - cy));
        maxDispersion = std::max(maxDispersion, d);
    }

    Section section;
    section.name = "z_seam";
    section.displayName = "Z-Seam Analysis";

    section.metrics.push_back(makeMetric("seam_count", static_cast<int64_t>(seams.size())));
    section.metrics.push_back(makeMetric("aligned_count", static_cast<int64_t>(alignedCount)));
    section.metrics.push_back(makeMetric("alignment_score", pairCount > 0 ? static_cast<double>(alignedCount) / pairCount : 1.0));
    section.metrics.push_back(makeMetric("average_seam_distance_mm", avgDist));
    section.metrics.push_back(makeMetric("max_seam_distance_mm", maxDist));
    section.metrics.push_back(makeMetric("seam_variance_mm", std::max(0.0, variance)));
    section.metrics.push_back(makeMetric("seam_dispersion_mm", maxDispersion));

    const double alignmentScore = pairCount > 0 ? static_cast<double>(alignedCount) / pairCount : 1.0;
    double score = 100.0 * alignmentScore;
    if (maxDist > 10.0) score -= 10.0;
    if (maxDispersion > 10.0) score -= 10.0;
    section.score = std::clamp(score, 0.0, 100.0);

    section.totalEventCount = seams.size();

    const std::string& detail = options.detailLevel;
    const bool summaryOnly = (detail == "summary");
    const bool fullEvents = (detail == "full");

    size_t topLimit = 0;
    if (fullEvents) {
        topLimit = std::numeric_limits<size_t>::max();
    } else if (!summaryOnly) {
        topLimit = (options.topEventLimit > 0) ? options.topEventLimit : 64;
    }

    section.hasMoreEvents = !seams.empty() && (summaryOnly || seams.size() > topLimit);

    if (summaryOnly) {
        return section;
    }

    std::vector<SeamPoint> sorted = seams;
    std::sort(sorted.begin(), sorted.end(),
              [](const SeamPoint& a, const SeamPoint& b) {
                  return a.distanceToPrevious > b.distanceToPrevious;
              });

    const size_t eventCount = std::min(topLimit, sorted.size());
    for (size_t i = 0; i < eventCount; ++i) {
        const auto& s = sorted[i];

        Event e;
        e.id = std::format("z_seam:z={:.2f}:line{}", s.z, s.lineNumber);
        e.type = "z_seam";
        e.severity = Severity::Info;
        e.message = std::format(
            "Layer Z={:.2f} mm: seam at ({:.1f}, {:.1f}){}{}",
            s.z, s.x, s.y,
            s.aligned ? ", aligned" : ", misaligned",
            s.distanceToPrevious > 1e-9
                ? std::format(" (distance to previous: {:.2f} mm)", s.distanceToPrevious)
                : "");
        e.metricValue = s.distanceToPrevious;
        e.detailsJson = std::format(
            R"({{"x":{:.3f},"y":{:.3f},"z":{:.2f},"line":{},"aligned":{},"distance_to_previous_mm":{:.3f}}})" ,
            s.x, s.y, s.z, s.lineNumber, s.aligned, s.distanceToPrevious);

        section.events.push_back(std::move(e));
    }

    return section;
}

} // namespace tether::gcode::analysis
