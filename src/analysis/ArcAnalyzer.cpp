/**
 * @file ArcAnalyzer.cpp
 * @brief Analyze arc interpolation quality, detect arc-fitting candidates, compute arc length.
 */
#include "tether/analysis/ArcAnalyzer.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <format>
#include <optional>

namespace tether::analysis {

std::optional<ArcAnalyzer::CircleFit> ArcAnalyzer::fitCircle(
    double x1, double y1, double x2, double y2,
    double x3, double y3) {

    double d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    if (std::abs(d) < 1e-10) return std::nullopt;

    double ux = ((x1*x1 + y1*y1) * (y2 - y3) +
                 (x2*x2 + y2*y2) * (y3 - y1) +
                 (x3*x3 + y3*y3) * (y1 - y2)) / d;
    double uy = ((x1*x1 + y1*y1) * (x3 - x2) +
                 (x2*x2 + y2*y2) * (x1 - x3) +
                 (x3*x3 + y3*y3) * (x2 - x1)) / d;
    double r = std::sqrt((x1 - ux) * (x1 - ux) + (y1 - uy) * (y1 - uy));
    return CircleFit{ux, uy, r};
}

ArcAnalysisResult ArcAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    ArcAnalysisResult result;

    // Collect G1 XY points for arc fitting.
    struct Pt { double x, y; int line; };
    std::vector<Pt> g1Points;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;
    double totalLinearDistance = 0;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);

        if (m.mode == MotionMode::Feed) {
            double dist = moveDistanceXY(prev, m);
            if (dist > 0.01) {
                g1Points.push_back({m.x, m.y, i});
                totalLinearDistance += dist;
            }
        }

        // Process existing G2/G3 arcs.
        if (m.mode == MotionMode::ArcCW || m.mode == MotionMode::ArcCCW) {
            bool isCW = (m.mode == MotionMode::ArcCW);
            double endX = m.x, endY = m.y;
            double radius = 0, centerX = 0, centerY = 0;

            if (m.hasR) {
                radius = std::abs(m.r);
            } else if (m.hasI || m.hasJ) {
                centerX = prev.x + m.i;
                centerY = prev.y + m.j;
                radius = std::sqrt(m.i * m.i + m.j * m.j);
            }

            double angle = 0, length = 0;
            if (radius > 0) {
                double startAngle = std::atan2(prev.y - centerY, prev.x - centerX);
                double endAngle = std::atan2(endY - centerY, endX - centerX);
                double angleDiff = std::abs(endAngle - startAngle);
                if (isCW) angleDiff = 2 * M_PI - angleDiff;
                angle = angleDiff * 180.0 / M_PI;
                length = radius * angleDiff;
            }

            bool hasIssue = false;
            std::string issue;
            if (radius < 0.1) { hasIssue = true; issue = "very_small_radius"; }
            else if (radius > 500) { hasIssue = true; issue = "very_large_radius"; }
            else if (angle > 350) { hasIssue = true; issue = "near_full_circle"; }

            result.arcs.push_back({i, radius, angle, length, isCW, hasIssue, issue});

            // Full arc data.
            ArcData ad;
            ad.line = i;
            ad.isCW = isCW;
            ad.startX = prev.x; ad.startY = prev.y;
            ad.endX = endX; ad.endY = endY;
            ad.centerX = centerX; ad.centerY = centerY;
            ad.radius = radius;
            ad.arcLength = length;
            ad.sweepAngle = angle;
            result.arcData.push_back(ad);

            if (isCW) result.cwCount++; else result.ccwCount++;
            result.totalArcLength += length;
        }

        prev = m;
    }

    // === Arc fitting candidates ===
    std::size_t idx = 0;
    while (idx + static_cast<std::size_t>(params_.minSegments) < g1Points.size()) {
        int bestEnd = -1;
        double bestError = 1e18;
        double bestCx = 0, bestCy = 0, bestRadius = 0;
        bool bestCW = false;

        std::size_t maxJ = std::min(g1Points.size(), idx + 100);
        for (std::size_t j = idx + params_.minSegments; j < maxJ; ++j) {
            const auto& start = g1Points[idx];
            const auto& mid = g1Points[(idx + j) / 2];
            const auto& end = g1Points[j];

            auto fit = fitCircle(start.x, start.y, mid.x, mid.y, end.x, end.y);
            if (!fit) continue;

            double maxErr = 0;
            for (std::size_t k = idx; k <= j; ++k) {
                double d = std::sqrt(
                    (g1Points[k].x - fit->cx) * (g1Points[k].x - fit->cx) +
                    (g1Points[k].y - fit->cy) * (g1Points[k].y - fit->cy));
                maxErr = std::max(maxErr, std::abs(d - fit->radius));
            }

            if (maxErr < params_.fitToleranceMm && maxErr < bestError) {
                bestEnd = static_cast<int>(j);
                bestCx = fit->cx;
                bestCy = fit->cy;
                bestRadius = fit->radius;
                bestError = maxErr;
                double cross = (end.x - start.x) * (mid.y - start.y) -
                               (end.y - start.y) * (mid.x - start.x);
                bestCW = (cross <= 0);
            }
        }

        if (bestEnd >= 0) {
            int segCount = bestEnd - static_cast<int>(idx);
            result.candidates.push_back({
                g1Points[idx].line, g1Points[bestEnd].line,
                bestCx, bestCy, bestRadius, bestCW,
                segCount, static_cast<double>(segCount * 20 - 30), bestError
            });
            result.totalFittableSegments += segCount;
            result.totalEstimatedSavings += segCount * 20.0 - 30.0;
            idx = static_cast<std::size_t>(bestEnd) + 1;
        } else {
            ++idx;
        }
    }

    // === Arc quality statistics ===
    result.arcCount = static_cast<int>(result.arcs.size());
    if (!result.arcs.empty()) {
        std::vector<double> radii;
        for (const auto& a : result.arcs)
            if (a.radius > 0) radii.push_back(a.radius);
        if (!radii.empty()) {
            result.avgRadius = std::accumulate(radii.begin(), radii.end(), 0.0) / radii.size();
            result.minRadius = *std::min_element(radii.begin(), radii.end());
        }
        result.issueCount = static_cast<int>(
            std::count_if(result.arcs.begin(), result.arcs.end(),
                          [](const ArcQualityInfo& a) { return a.hasIssue; }));
        result.qualityScore = std::max(0.0, 100.0 - result.issueCount * 10.0);
    }

    // === Arc length statistics ===
    if (!result.arcData.empty()) {
        result.avgArcLength = result.totalArcLength / result.arcData.size();
        result.maxArcLength = 0;
        for (const auto& a : result.arcData)
            result.maxArcLength = std::max(result.maxArcLength, a.arcLength);
    }

    double totalPath = result.totalArcLength + totalLinearDistance;
    result.totalLinearDistance = totalLinearDistance;
    result.arcPercentage = (totalPath > 0) ? (result.totalArcLength / totalPath) * 100.0 : 0.0;

    // Recommendations.
    if (result.arcCount == 0 && result.candidates.empty()) {
        result.recommendations.push_back("No arc commands (G2/G3) — use arcs for smoother finishes");
    }
    if (!result.candidates.empty()) {
        result.recommendations.push_back(
            std::format("{} arc fitting candidates, ~{:.0f} bytes savings",
                        result.candidates.size(), result.totalEstimatedSavings));
    }
    if (result.issueCount > 0) {
        result.recommendations.push_back(
            std::format("{} arc quality issues", result.issueCount));
    }
    if (result.qualityScore > 85 && result.arcCount > 0) {
        result.recommendations.push_back("Good arc quality");
    }

    return result;
}

} // namespace tether::analysis
