/**
 * @file PathContinuityChecker.cpp
 * @brief Check toolpath continuity: gaps, jumps, retracts, plunges, per-layer.
 */
#include "tether/analysis/PathContinuityChecker.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <format>

namespace tether::analysis {

PathContinuityResult PathContinuityChecker::analyze(
    const std::vector<std::string>& gcodeLines) const {

    PathContinuityResult result;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;

    // Per-layer tracking.
    double currentZ = 0;
    int layerNum = 0;
    int gapCount = 0;
    double maxGap = 0;
    double totalGapDistance = 0;
    bool hasPrevDir = false;
    double prevDirX = 0, prevDirY = 0;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);
        if (!isMotion(m)) {
            prev.feedRate = m.feedRate;
            continue;
        }

        double dist = moveDistance(prev, m);
        bool isRapid = (m.mode == MotionMode::Rapid);

        // === Overall continuity ===
        // Gap (rapid move > threshold).
        if (isRapid && dist > params_.gapThresholdMm) {
            ContinuityIssue issue;
            issue.line = i;
            issue.type = ContinuityIssue::Type::Gap;
            issue.gapDistance = dist;
            issue.description = std::format("Rapid gap of {:.2f}mm", dist);
            issue.severity = (dist > 10) ? ContinuityIssue::Severity::High
                : (dist > 5) ? ContinuityIssue::Severity::Medium
                : ContinuityIssue::Severity::Low;
            result.issues.push_back(issue);
        }

        // Z jump (non-rapid).
        if (std::abs(m.z - prev.z) > 5 && !isRapid) {
            ContinuityIssue issue;
            issue.line = i;
            issue.type = ContinuityIssue::Type::Jump;
            issue.gapDistance = std::abs(m.z - prev.z);
            issue.description = std::format("Z jump of {:.2f}mm", std::abs(m.z - prev.z));
            issue.severity = ContinuityIssue::Severity::Medium;
            result.issues.push_back(issue);
        }

        // Retraction.
        if (m.hasE && m.e < prev.e) {
            ContinuityIssue issue;
            issue.line = i;
            issue.type = ContinuityIssue::Type::Retract;
            issue.gapDistance = prev.e - m.e;
            issue.description = std::format("Retraction of {:.2f}mm", prev.e - m.e);
            issue.severity = ContinuityIssue::Severity::Low;
            result.issues.push_back(issue);
        }

        // Plunge (Z decrease while extruding).
        if (m.z < prev.z && m.hasE && m.e > prev.e && std::abs(m.z - prev.z) > 0.5) {
            ContinuityIssue issue;
            issue.line = i;
            issue.type = ContinuityIssue::Type::Plunge;
            issue.gapDistance = prev.z - m.z;
            issue.description = std::format("Plunge of {:.2f}mm", prev.z - m.z);
            issue.severity = ContinuityIssue::Severity::Medium;
            result.issues.push_back(issue);
        }

        // Direction change.
        if (dist > 0.1 && !isRapid) {
            double dx = m.x - prev.x, dy = m.y - prev.y;
            if (hasPrevDir) {
                double prevLen = std::sqrt(prevDirX * prevDirX + prevDirY * prevDirY);
                double curLen = std::sqrt(dx * dx + dy * dy);
                if (prevLen > 0 && curLen > 0) {
                    double dot = (dx * prevDirX + dy * prevDirY) / (curLen * prevLen);
                    if (dot < 0) {
                        ContinuityIssue issue;
                        issue.line = i;
                        issue.type = ContinuityIssue::Type::DirectionChange;
                        issue.gapDistance = 0;
                        issue.description = "Direction reversal";
                        issue.severity = ContinuityIssue::Severity::Low;
                        result.issues.push_back(issue);
                    }
                }
            }
            prevDirX = dx; prevDirY = dy;
            hasPrevDir = true;
        }

        // === Per-layer continuity ===
        if (m.z > currentZ + 0.01 && layerNum > 0) {
            // Layer change.
            result.layers.push_back({layerNum, currentZ, gapCount == 0,
                gapCount, maxGap, totalGapDistance});
            layerNum++;
            currentZ = m.z;
            gapCount = 0; maxGap = 0; totalGapDistance = 0;
        } else if (layerNum == 0) {
            currentZ = m.z;
            layerNum = 1;
        }

        if (layerNum > 0 && m.mode == MotionMode::Feed) {
            double xyDist = moveDistanceXY(prev, m);
            if (xyDist > params_.layerGapThresholdMm) {
                gapCount++;
                maxGap = std::max(maxGap, xyDist);
                totalGapDistance += xyDist;
            }
        }

        prev = m;
    }

    // Save last layer.
    if (layerNum > 0) {
        result.layers.push_back({layerNum, currentZ, gapCount == 0,
            gapCount, maxGap, totalGapDistance});
    }

    // === Overall statistics ===
    result.issueCount = static_cast<int>(result.issues.size());
    result.highSeverityCount = static_cast<int>(std::count_if(
        result.issues.begin(), result.issues.end(),
        [](const ContinuityIssue& i) { return i.severity == ContinuityIssue::Severity::High; }));
    result.mediumSeverityCount = static_cast<int>(std::count_if(
        result.issues.begin(), result.issues.end(),
        [](const ContinuityIssue& i) { return i.severity == ContinuityIssue::Severity::Medium; }));
    result.lowSeverityCount = static_cast<int>(std::count_if(
        result.issues.begin(), result.issues.end(),
        [](const ContinuityIssue& i) { return i.severity == ContinuityIssue::Severity::Low; }));

    for (const auto& issue : result.issues) {
        if (issue.type == ContinuityIssue::Type::Gap)
            result.totalGapDistance += issue.gapDistance;
    }

    result.continuityScore = std::max(0.0,
        100.0 - result.highSeverityCount * 10.0 -
        result.mediumSeverityCount * 5.0 - result.lowSeverityCount * 1.0);
    result.isContinuous = (result.highSeverityCount == 0);

    // === Per-layer statistics ===
    result.layerCount = static_cast<int>(result.layers.size());
    result.continuousLayerCount = static_cast<int>(std::count_if(
        result.layers.begin(), result.layers.end(),
        [](const LayerContinuity& l) { return l.isContinuous; }));
    result.totalGaps = std::accumulate(result.layers.begin(), result.layers.end(), 0,
        [](int s, const LayerContinuity& l) { return s + l.gapCount; });
    for (const auto& l : result.layers)
        result.maxGap = std::max(result.maxGap, l.maxGap);

    // Recommendations.
    if (result.highSeverityCount > 0) {
        result.recommendations.push_back(
            std::format("{} high-severity discontinuities", result.highSeverityCount));
    }
    if (result.totalGapDistance > 100) {
        result.recommendations.push_back(
            std::format("Total gap: {:.0f}mm — optimize travel", result.totalGapDistance));
    }
    if (result.totalGaps > 0) {
        result.recommendations.push_back(
            std::format("{} gaps across {} layers", result.totalGaps, result.layerCount));
    }
    if (result.isContinuous && result.totalGaps == 0) {
        result.recommendations.push_back("Toolpath is continuous — good quality");
    }

    return result;
}

} // namespace tether::analysis
