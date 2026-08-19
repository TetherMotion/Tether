/// @file CncToolpathAnalyzer.cpp
/// @brief Basic CNC toolpath, chip-load and MRR analysis.

#include "tether/gcode/analysis/CncToolpathAnalyzer.hpp"
#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace tether::gcode::analysis {

namespace {

struct ChipRange {
    double min = 0.0;
    double max = 0.0;
};

const std::map<std::string, ChipRange>& materialChipRanges() {
    static const std::map<std::string, ChipRange> ranges = {
        {"aluminum", {0.05, 0.15}},
        {"steel", {0.03, 0.08}},
        {"stainless", {0.02, 0.06}},
        {"wood", {0.10, 0.30}},
        {"plastic", {0.10, 0.25}},
        {"brass", {0.05, 0.12}},
    };
    return ranges;
}

ChipRange getRange(const std::string& material) {
    auto it = materialChipRanges().find(material);
    if (it != materialChipRanges().end()) return it->second;
    return materialChipRanges().at("aluminum");
}

} // namespace

Section analyzeCncToolpath(const std::vector<GCode::PlanningSegment>& planningSegments,
                           const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                           const std::vector<std::string>& gcodeLines,
                           const Options& options) {
    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return {};
    }

    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());

    // Defaults; may be overridden by comments.
    int toolNumber = 1;
    double toolDiameter = 6.0;
    int flutes = 2;
    double axialDepth = 2.0;
    double radialDepth = toolDiameter * 0.5;
    std::string material = "aluminum";

    // Per-line state.
    std::vector<int> rpmByLine(gcodeLines.size(), 0);
    int currentRpm = 0;
    bool anySpindle = false;
    bool anyTool = false;

    for (size_t i = 0; i < gcodeLines.size(); ++i) {
        const std::string line = stripGcodeComments(gcodeLines[i]);
        if (auto v = findWordValue(line, 'M')) {
            if (*v == 3.0 || *v == 4.0) {
                if (auto s = findWordValue(line, 'S')) {
                    currentRpm = static_cast<int>(std::round(std::max(0.0, *s)));
                    anySpindle = true;
                }
            } else if (*v == 5.0) {
                currentRpm = 0;
            }
        }
        if (auto t = findWordValue(line, 'T')) {
            if (*t > 0) {
                toolNumber = static_cast<int>(*t);
                anyTool = true;
            }
        }

        // Try to parse slicer/CAM comments for tooling.
        if (auto f = extractFeatureTag(gcodeLines[i], "MATERIAL")) material = toUpper(*f);
        if (auto f = extractFeatureTag(gcodeLines[i], "TOOL_DIA")) {
            try { toolDiameter = std::stod(*f); } catch (...) {}
        }
        if (auto f = extractFeatureTag(gcodeLines[i], "FLUTES")) {
            try { flutes = std::stoi(*f); } catch (...) {}
            if (flutes < 1) flutes = 2;
        }
        if (auto f = extractFeatureTag(gcodeLines[i], "AXIAL_DEPTH")) {
            try { axialDepth = std::stod(*f); } catch (...) {}
        }
        if (auto f = extractFeatureTag(gcodeLines[i], "RADIAL_DEPTH")) {
            try { radialDepth = std::stod(*f); } catch (...) {}
        }

        rpmByLine[i] = currentRpm;
    }

    const ChipRange range = getRange(material);

    double cuttingTime = 0.0;
    double rapidTime = 0.0;
    double cuttingPath = 0.0;
    double rapidPath = 0.0;
    double feedRateSum = 0.0;
    double rpmSum = 0.0;
    size_t cuttingMoves = 0;
    size_t rapidMoves = 0;
    size_t inRange = 0;
    size_t outOfRange = 0;
    double chipLoadSum = 0.0;
    double maxChipLoad = 0.0;
    double maxMrr = 0.0;
    double mrrSum = 0.0;
    size_t mrrCount = 0;

    // Events: tool/spindle changes and chip-load warnings.
    std::vector<int> toolChanges;
    std::vector<std::pair<int, int>> rpmChanges; // line, rpm
    struct ChipEvent {
        int line;
        double chipLoad;
        bool inRange;
        double feed;
        int rpm;
        double mrr;
    };
    std::vector<ChipEvent> chipEvents;

    int prevRpm = -1;
    int prevTool = -1;

    for (size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        const auto& ss = segmentSpeeds[i];
        const size_t lineIndex = (ss.lineNumber >= 1)
                                     ? static_cast<size_t>(ss.lineNumber - 1)
                                     : std::numeric_limits<size_t>::max();
        const int rpm = (lineIndex < rpmByLine.size()) ? rpmByLine[lineIndex] : 0;

        if (seg.isRapid) {
            rapidTime += ss.duration;
            rapidPath += seg.segmentLength;
            ++rapidMoves;
        } else {
            cuttingTime += ss.duration;
            cuttingPath += seg.segmentLength;
            ++cuttingMoves;
            feedRateSum += seg.feedRate;
            if (rpm > 0) rpmSum += rpm;

            // Track tool/spindle changes based on the active values at this segment.
            if (anyTool && toolNumber != prevTool) {
                toolChanges.push_back(ss.lineNumber);
                prevTool = toolNumber;
            }
            if (anySpindle && rpm != prevRpm) {
                rpmChanges.push_back({ss.lineNumber, rpm});
                prevRpm = rpm;
            }

            if (rpm > 0 && flutes > 0) {
                const double chipLoad = seg.feedRate / (static_cast<double>(rpm) * flutes);
                const bool ok = chipLoad >= range.min && chipLoad <= range.max;
                if (ok) ++inRange;
                else ++outOfRange;
                chipLoadSum += chipLoad;
                maxChipLoad = std::max(maxChipLoad, chipLoad);
                chipEvents.push_back({ss.lineNumber, chipLoad, ok, seg.feedRate, rpm, 0.0});

                // Approximate MRR (mm³/min) = feedRate * axialDepth * radialDepth.
                const double mrr = seg.feedRate * axialDepth * radialDepth;
                mrrSum += mrr;
                maxMrr = std::max(maxMrr, mrr);
                ++mrrCount;
                chipEvents.back().mrr = mrr;
            }
        }
    }

    // Only skip the whole section if there is no cutting data and no spindle/tool state.
    if (cuttingMoves == 0 && !anySpindle && !anyTool) return {};

    Section section;
    section.name = "cnc_toolpath";
    section.displayName = "CNC Toolpath";

    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const size_t topLimit = (options.topEventLimit > 0)
                                ? options.topEventLimit
                                : (fullEvents ? std::numeric_limits<size_t>::max() : 64);

    section.metrics.push_back(makeMetric("tool_number", static_cast<int64_t>(toolNumber)));
    section.metrics.push_back(makeMetric("tool_diameter_mm", toolDiameter));
    section.metrics.push_back(makeMetric("flutes", static_cast<int64_t>(flutes)));
    section.metrics.push_back(makeMetric("axial_depth_mm", axialDepth));
    section.metrics.push_back(makeMetric("radial_depth_mm", radialDepth));
    section.metrics.push_back(makeMetric("cutting_time_s", cuttingTime));
    section.metrics.push_back(makeMetric("rapid_time_s", rapidTime));
    section.metrics.push_back(makeMetric("cutting_path_mm", cuttingPath));
    section.metrics.push_back(makeMetric("rapid_path_mm", rapidPath));
    section.metrics.push_back(makeMetric("avg_feed_rate_mm_min", cuttingMoves > 0 ? feedRateSum / cuttingMoves : 0.0));
    section.metrics.push_back(makeMetric("avg_spindle_rpm", cuttingMoves > 0 ? rpmSum / cuttingMoves : 0.0));
    section.metrics.push_back(makeMetric("max_chip_load", maxChipLoad));
    section.metrics.push_back(makeMetric("avg_chip_load", chipEvents.empty() ? 0.0 : chipLoadSum / chipEvents.size()));
    section.metrics.push_back(makeMetric("max_mrr_mm3_min", maxMrr));
    section.metrics.push_back(makeMetric("avg_mrr_mm3_min", mrrCount > 0 ? mrrSum / mrrCount : 0.0));
    section.metrics.push_back(makeMetric("tool_changes", static_cast<int64_t>(toolChanges.size())));
    section.metrics.push_back(makeMetric("spindle_changes", static_cast<int64_t>(rpmChanges.size())));
    section.metrics.push_back(makeMetric("chip_load_samples", static_cast<int64_t>(chipEvents.size())));
    section.metrics.push_back(makeMetric("in_range_samples", static_cast<int64_t>(inRange)));
    section.metrics.push_back(makeMetric("out_of_range_samples", static_cast<int64_t>(outOfRange)));
    section.metrics.push_back(makeMetric("in_range_percentage", chipEvents.empty() ? 0.0
                                                                                 : 100.0 * inRange / chipEvents.size()));

    double score = 100.0;
    if (chipEvents.empty()) {
        if (cuttingMoves > 0 && rapidTime > cuttingTime) score -= 20.0;
    } else {
        const double pct = 100.0 * inRange / chipEvents.size();
        score = pct;
        if (maxChipLoad > range.max * 1.5) score -= 10.0;
        if (rapidTime > cuttingTime) score -= 10.0;
    }
    section.score = std::clamp(score, 0.0, 100.0);

    // Collect all candidate events. Tool and spindle changes come first;
    // chip-load events are sorted by load descending.
    std::vector<Event> allEvents;
    allEvents.reserve(toolChanges.size() + rpmChanges.size() + chipEvents.size());

    for (int line : toolChanges) {
        allEvents.push_back(Event{
            .id = std::format("cnc:tool_change:line{}", line),
            .type = "tool_change",
            .severity = Severity::Info,
            .message = std::format("Tool change at line {}", line),
            .metricValue = static_cast<double>(toolNumber),
            .detailsJson = std::format(R"({{"line":{},"tool_number":{}}})" , line, toolNumber)});
    }

    for (const auto& [line, rpm] : rpmChanges) {
        allEvents.push_back(Event{
            .id = std::format("cnc:spindle:line{}:rpm{}", line, rpm),
            .type = "spindle_change",
            .severity = Severity::Info,
            .message = std::format("Spindle set to {} RPM at line {}", rpm, line),
            .metricValue = static_cast<double>(rpm),
            .detailsJson = std::format(R"({{"line":{},"rpm":{}}})" , line, rpm)});
    }

    std::ranges::sort(chipEvents,
                      [](const ChipEvent& a, const ChipEvent& b) {
                          return a.chipLoad > b.chipLoad;
                      });

    for (const auto& ev : chipEvents) {
        const bool tooHigh = ev.chipLoad > range.max;
        const bool tooLow = ev.chipLoad < range.min;
        Severity sev = Severity::Info;
        std::string note;
        if (tooHigh) {
            sev = Severity::High;
            note = std::format(
                "Chip load {:.3f} mm/tooth exceeds recommended max {:.3f} for {}",
                ev.chipLoad, range.max, material);
        } else if (tooLow) {
            sev = Severity::Low;
            note = std::format(
                "Chip load {:.3f} mm/tooth below recommended min {:.3f} for {}",
                ev.chipLoad, range.min, material);
        } else {
            note = std::format("Chip load {:.3f} mm/tooth in range for {}",
                               ev.chipLoad, material);
        }

        allEvents.push_back(Event{
            .id = std::format("cnc:chip:line{}:{:.4f}", ev.line, ev.chipLoad),
            .type = "chip_load",
            .severity = sev,
            .message = std::format("Line {}: {}", ev.line, note),
            .metricValue = ev.chipLoad,
            .detailsJson = std::format(
                R"({{"line":{},"chip_load_mm_tooth":{:.4f},"feed_rate_mm_min":{:.1f},"rpm":{},"flutes":{},"mrr_mm3_min":{:.1f},"material":"{}","in_range":{}}})" ,
                ev.line, ev.chipLoad, ev.feed, ev.rpm, flutes, ev.mrr, material, ev.inRange)});
    }

    // Deduplicate by event id, preserving order.
    std::vector<std::string> seenIds;
    std::vector<Event> uniqueEvents;
    uniqueEvents.reserve(allEvents.size());
    for (auto& e : allEvents) {
        if (std::ranges::find(seenIds, e.id) == seenIds.end()) {
            seenIds.push_back(e.id);
            uniqueEvents.push_back(std::move(e));
        }
    }

    section.totalEventCount = uniqueEvents.size();
    section.hasMoreEvents = uniqueEvents.size() > topLimit;

    if (!summaryOnly) {
        const size_t limit = std::min(topLimit, uniqueEvents.size());
        section.events.reserve(limit);
        for (size_t i = 0; i < limit; ++i) {
            section.events.push_back(uniqueEvents[i]);
        }
    }

    return section;
}

} // namespace tether::gcode::analysis
