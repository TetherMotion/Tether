/**
 * @file CurvatureAnalyzer.cpp
 * @brief Analyze toolpath curvature, generate heatmap, and compute cornering speed.
 */
#include "tether/analysis/CurvatureAnalyzer.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <format>

namespace tether::analysis {

CurvatureResult CurvatureAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    CurvatureResult result;

    // Collect XY points from G0/G1 moves.
    struct Point { double x, y; int line; };
    std::vector<Point> points;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;
    double feedRate = 0;
    bool addedStart = false;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);
        if (!isMotion(m)) {
            prev.feedRate = m.feedRate;
            continue;
        }

        double dist = moveDistanceXY(prev, m);
        if (dist > 0.01) {
            // Add the starting point on the first real move so we capture
            // the turn at the very first vertex.
            if (!addedStart) {
                points.push_back({prev.x, prev.y, i});
                addedStart = true;
            }
            points.push_back({m.x, m.y, i});
            feedRate = m.feedRate;
        }

        prev = m;
    }

    // Compute curvature at each interior point using 3-point method.
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const auto& p0 = points[i - 1];
        const auto& p1 = points[i];
        const auto& p2 = points[i + 1];

        double v1x = p1.x - p0.x, v1y = p1.y - p0.y;
        double v2x = p2.x - p1.x, v2y = p2.y - p1.y;
        double len1 = std::sqrt(v1x * v1x + v1y * v1y);
        double len2 = std::sqrt(v2x * v2x + v2y * v2y);

        if (len1 < 0.01 || len2 < 0.01) continue;

        double dot = (v1x * v2x + v1y * v2y) / (len1 * len2);
        dot = std::max(-1.0, std::min(1.0, dot));
        double angleChange = std::acos(dot) * (180.0 / M_PI);

        double cross = v1x * v2y - v1y * v2x;
        double curvature = std::abs(cross) / (len1 * len2 * (len1 + len2) / 2.0);
        double radius = curvature > 0 ? 1.0 / curvature : 1e18;

        CurvatureSegment seg;
        seg.lineNumber = p1.line;
        seg.curvature = curvature;
        seg.radius = radius;
        seg.angleChange = angleChange;
        seg.severity = (angleChange > params_.sharpThresholdDeg) ? CurvatureSegment::Severity::Sharp
            : (angleChange > params_.sharpThresholdDeg / 2.0) ? CurvatureSegment::Severity::Moderate
            : CurvatureSegment::Severity::Smooth;
        result.segments.push_back(seg);

        // Heatmap point.
        CurvatureHeatmapPoint hp;
        hp.x = p1.x;
        hp.y = p1.y;
        hp.curvature = curvature;
        hp.category = (curvature < 0.001) ? CurvatureHeatmapPoint::Category::Straight
            : (curvature < 0.01) ? CurvatureHeatmapPoint::Category::Gentle
            : (curvature < 0.05) ? CurvatureHeatmapPoint::Category::Moderate
            : (curvature < 0.1) ? CurvatureHeatmapPoint::Category::Sharp
            : CurvatureHeatmapPoint::Category::VerySharp;
        result.heatmap.push_back(hp);

        // Cornering speed.
        if (angleChange > 5.0) {
            double speedFactor = std::max(0.1, 1.0 - (angleChange / 180.0) * 0.8);
            double recSpeed = std::round(params_.maxFeedRateMmPerMin * speedFactor);
            double curSpeed = feedRate;
            double speedReduction = (curSpeed > 0)
                ? ((curSpeed - recSpeed) / curSpeed) * 100.0 : 0.0;

            CorneringSpeedPoint cp;
            cp.line = p1.line;
            cp.cornerAngle = angleChange;
            cp.recommendedSpeed = recSpeed;
            cp.currentSpeed = curSpeed;
            cp.speedReduction = speedReduction;
            cp.severity = (angleChange < 30) ? CorneringSpeedPoint::Severity::Gentle
                : (angleChange < 60) ? CorneringSpeedPoint::Severity::Moderate
                : (angleChange < 120) ? CorneringSpeedPoint::Severity::Sharp
                : CorneringSpeedPoint::Severity::VerySharp;
            result.corneringPoints.push_back(cp);
        }
    }

    // Statistics.
    std::vector<double> curvatures;
    for (const auto& s : result.segments)
        if (s.curvature > 0) curvatures.push_back(s.curvature);

    if (!curvatures.empty()) {
        result.maxCurvature = *std::max_element(curvatures.begin(), curvatures.end());
        result.avgCurvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
    }
    result.minRadius = 1e18;
    for (const auto& s : result.segments) {
        if (s.radius < result.minRadius && s.radius < 1e17)
            result.minRadius = s.radius;
    }
    if (result.minRadius >= 1e17) result.minRadius = 0;

    result.sharpTurnCount = static_cast<int>(
        std::count_if(result.segments.begin(), result.segments.end(),
                       [](const CurvatureSegment& s) {
                           return s.severity == CurvatureSegment::Severity::Sharp;
                       }));
    result.smoothnessScore = result.segments.empty() ? 100.0
        : std::max(0.0, 100.0 - (static_cast<double>(result.sharpTurnCount) / result.segments.size()) * 200.0);

    result.cornerCount = static_cast<int>(result.corneringPoints.size());
    if (result.cornerCount > 0) {
        result.avgSpeedReduction = std::accumulate(
            result.corneringPoints.begin(), result.corneringPoints.end(), 0.0,
            [](double s, const CorneringSpeedPoint& p) { return s + p.speedReduction; })
            / result.cornerCount;
        result.overspeedCount = static_cast<int>(
            std::count_if(result.corneringPoints.begin(), result.corneringPoints.end(),
                          [](const CorneringSpeedPoint& p) { return p.currentSpeed > p.recommendedSpeed; }));
    }
    result.corneringEfficiencyScore = std::max(0.0, 100.0 - result.overspeedCount * 5.0 - std::abs(result.avgSpeedReduction) * 0.5);

    // Recommendations.
    if (result.sharpTurnCount > 20) {
        result.recommendations.push_back(
            std::format("{} sharp turns — consider arcs (G2/G3)", result.sharpTurnCount));
    }
    if (result.minRadius < 1.0 && result.minRadius > 0) {
        result.recommendations.push_back(
            std::format("Min radius {:.2f}mm — tight turns", result.minRadius));
    }
    if (result.smoothnessScore < 50) {
        result.recommendations.push_back("Low smoothness — consider spline interpolation");
    }
    if (result.recommendations.empty()) {
        result.recommendations.push_back("Toolpath curvature is smooth");
    }

    return result;
}

} // namespace tether::analysis
