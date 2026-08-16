/**
 * @file ToolpathEfficiencyAnalyzer.cpp
 * @brief Analyze toolpath length, efficiency, air cutting, and rapid travel.
 */
#include "tether/analysis/ToolpathEfficiencyAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace tether::analysis {

ToolpathEfficiencyResult ToolpathEfficiencyAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    ToolpathEfficiencyResult result;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;
    double feedRate = 0;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);
        if (!isMotion(m)) {
            prev.feedRate = m.feedRate;
            continue;
        }

        double dist = moveDistance(prev, m);
        if (dist <= 0) { prev = m; continue; }

        bool isRapid = (m.mode == MotionMode::Rapid);
        bool isArc = (m.mode == MotionMode::ArcCW || m.mode == MotionMode::ArcCCW);
        bool isExtruding = (m.hasE && m.e > prev.e);
        bool isCutting = isExtruding || (!isRapid && !isArc && m.z < params_.cuttingZThreshold);

        double time = (m.feedRate > 0) ? dist / (m.feedRate / 60.0) : 0;

        if (isArc) {
            result.arcDistance += dist;
            if (time > 0) result.cuttingTime += time;
        } else if (isRapid) {
            result.travelDistance += dist;
            if (time > 0) result.travelTime += time;
            result.rapidDistance += dist;
            result.rapidCount++;
            result.longestRapid = std::max(result.longestRapid, dist);
        } else if (isCutting) {
            result.cuttingDistance += dist;
            if (time > 0) result.cuttingTime += time;
        } else {
            // Non-cutting feed move (air cutting).
            result.travelDistance += dist;
            if (time > 0) result.travelTime += time;
            result.airCuttingTime += time;
            result.airCuttingDistance += dist;
            result.airCuttingCount++;
            result.longestAirCut = std::max(result.longestAirCut, dist);
        }

        prev = m;
    }

    // Compute derived metrics.
    result.totalDistance = result.cuttingDistance + result.travelDistance + result.arcDistance;
    result.cuttingPercentage = (result.totalDistance > 0)
        ? (result.cuttingDistance / result.totalDistance) * 100.0 : 0;
    result.travelPercentage = (result.totalDistance > 0)
        ? (result.travelDistance / result.totalDistance) * 100.0 : 0;
    result.efficiencyRatio = (result.travelDistance > 0)
        ? result.cuttingDistance / result.travelDistance : 0;

    result.totalTime = result.cuttingTime + result.travelTime;
    double totalDist2 = result.cuttingDistance + result.travelDistance;
    result.cuttingRatio = (totalDist2 > 0) ? result.cuttingDistance / totalDist2 : 0;
    result.engagementRatio = (result.totalTime > 0) ? result.cuttingTime / result.totalTime : 0;
    result.efficiencyScore = std::round(result.engagementRatio * 100);
    result.wastedTravelPercentage = (1.0 - result.cuttingRatio) * 100.0;
    result.optimizationPotential = std::max(0.0, result.wastedTravelPercentage - 20.0);

    double totalTime2 = result.airCuttingTime + result.cuttingTime;
    result.airCuttingPercentage = (totalTime2 > 0)
        ? (result.airCuttingTime / totalTime2) * 100.0 : 0;
    result.airCuttingEfficiencyScore = std::max(0.0, 100.0 - result.airCuttingPercentage);

    result.rapidPercentage = (result.totalDistance > 0)
        ? (result.rapidDistance / result.totalDistance) * 100.0 : 0;
    result.avgRapidLength = (result.rapidCount > 0)
        ? result.rapidDistance / result.rapidCount : 0;
    result.rapidEfficiencyScore = std::max(0.0, 100.0 - result.rapidPercentage * 0.8);

    // Recommendations.
    if (result.travelPercentage > 50) {
        result.recommendations.push_back(
            std::format("Travel is {:.1f}% of total — optimize travel paths", result.travelPercentage));
    }
    if (result.airCuttingPercentage > 30) {
        result.recommendations.push_back(
            std::format("High air cutting ({:.0f}%) — optimize toolpath", result.airCuttingPercentage));
    }
    if (result.rapidPercentage > 40) {
        result.recommendations.push_back(
            std::format("High rapid travel ({:.0f}%) — reorder operations", result.rapidPercentage));
    }
    if (result.efficiencyScore > 80) {
        result.recommendations.push_back("High efficiency — minimal travel waste");
    }
    if (result.recommendations.empty()) {
        result.recommendations.push_back(
            std::format("Cutting: {:.0f}mm, Travel: {:.0f}mm, Efficiency: {:.0f}%",
                        result.cuttingDistance, result.travelDistance, result.efficiencyScore));
    }

    return result;
}

} // namespace tether::analysis
