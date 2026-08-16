/**
 * @file PathTopologyDetector.cpp
 * @brief Detect toolpath loops, self-intersections, overlaps, and symmetry.
 */
#include "tether/analysis/PathTopologyDetector.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <format>

namespace tether::analysis {

bool PathTopologyDetector::segmentIntersection(
    double x1, double y1, double x2, double y2,
    double x3, double y3, double x4, double y4,
    double& ix, double& iy) {

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 1e-10) return false;

    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        ix = x1 + t * (x2 - x1);
        iy = y1 + t * (y2 - y1);
        return true;
    }
    return false;
}

void PathTopologyDetector::detectLoops(
    const std::vector<ParsedMove>& moves, PathTopologyResult& result) const {

    struct Pt { double x, y; int line; };
    std::vector<Pt> positions;
    for (const auto& m : moves) {
        if (m.mode == MotionMode::Feed) {
            positions.push_back({m.x, m.y, m.lineNumber});
        }
    }

    std::size_t i = 0;
    while (i + 4 < positions.size()) {
        for (std::size_t j = i + 4; j < positions.size(); ++j) {
            double dx = positions[j].x - positions[i].x;
            double dy = positions[j].y - positions[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < params_.loopToleranceMm) {
                auto loopLen = j - i;
                int iterations = 1;
                std::size_t lastEnd = j;

                for (std::size_t k = j + loopLen; k < positions.size(); k += loopLen) {
                    double dx2 = positions[k].x - positions[i].x;
                    double dy2 = positions[k].y - positions[i].y;
                    if (std::sqrt(dx2 * dx2 + dy2 * dy2) < params_.loopToleranceMm) {
                        iterations++;
                        lastEnd = k;
                    } else break;
                }

                if (iterations > 1) {
                    double loopDist = 0;
                    for (std::size_t k = i; k + 1 < j; ++k) {
                        double ddx = positions[k+1].x - positions[k].x;
                        double ddy = positions[k+1].y - positions[k].y;
                        loopDist += std::sqrt(ddx * ddx + ddy * ddy);
                    }

                    LoopInfo loop;
                    loop.startLine = positions[i].line;
                    loop.endLine = positions[lastEnd].line;
                    loop.iterations = iterations;
                    loop.loopDistance = loopDist;

                    // Classify loop type.
                    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
                    for (std::size_t k = i; k <= j && k < positions.size(); ++k) {
                        minX = std::min(minX, positions[k].x);
                        maxX = std::max(maxX, positions[k].x);
                        minY = std::min(minY, positions[k].y);
                        maxY = std::max(maxY, positions[k].y);
                    }
                    double xRange = maxX - minX, yRange = maxY - minY;
                    if (std::abs(xRange - yRange) < 5) loop.type = LoopInfo::Type::Circular;
                    else if (xRange > 0 && yRange > 0) loop.type = LoopInfo::Type::Rectangular;
                    else loop.type = LoopInfo::Type::Spiral;

                    result.loops.push_back(loop);
                    result.totalLoopDistance += loopDist * iterations;
                    i = lastEnd;
                    goto nextOuter;
                }
            }
        }
        ++i;
        nextOuter:;
    }

    result.loopCount = static_cast<int>(result.loops.size());
    for (const auto& l : result.loops)
        result.maxIterations = std::max(result.maxIterations, l.iterations);
    result.loopEfficiencyScore = std::max(0.0, 100.0 - result.loopCount * 10.0);
}

void PathTopologyDetector::detectSelfIntersections(
    const std::vector<ParsedMove>& moves, PathTopologyResult& result) const {

    struct Seg { double x1, y1, z1, x2, y2, z2; int line; bool isRapid; };
    std::vector<Seg> segments;
    ParsedMove prev;
    prev.x = prev.y = prev.z = 0;

    for (const auto& m : moves) {
        if (!isMotion(m)) continue;
        double dist = moveDistanceXY(prev, m);
        if (dist > 0.1) {
            segments.push_back({prev.x, prev.y, prev.z, m.x, m.y, m.z,
                                m.lineNumber, m.mode == MotionMode::Rapid});
        }
        prev = m;
    }

    int maxSegs = std::min(static_cast<int>(segments.size()),
                           params_.maxSegmentsForIntersection);
    for (int i = 0; i < maxSegs; ++i) {
        for (int j = i + 2; j < maxSegs; ++j) {
            const auto& s1 = segments[i];
            const auto& s2 = segments[j];

            if (std::abs(s1.z1 - s2.z1) > params_.intersectionZThresholdMm &&
                std::abs(s1.z2 - s2.z2) > params_.intersectionZThresholdMm)
                continue;

            double ix, iy;
            if (segmentIntersection(s1.x1, s1.y1, s1.x2, s1.y2,
                                    s2.x1, s2.y1, s2.x2, s2.y2, ix, iy)) {
                bool isRapid = s1.isRapid || s2.isRapid;
                result.intersections.push_back({
                    s1.line, s2.line, ix, iy, isRapid,
                    isRapid ? SelfIntersection::Severity::Warning
                            : SelfIntersection::Severity::Error
                });
            }
        }
    }

    result.intersectionCount = static_cast<int>(result.intersections.size());
    result.hasCuttingIntersections = std::any_of(
        result.intersections.begin(), result.intersections.end(),
        [](const SelfIntersection& si) { return !si.isRapid; });
    result.hasRapidIntersections = std::any_of(
        result.intersections.begin(), result.intersections.end(),
        [](const SelfIntersection& si) { return si.isRapid; });
}

void PathTopologyDetector::detectOverlaps(
    const std::vector<ParsedMove>& moves, PathTopologyResult& result) const {

    struct Seg { double x1, y1, x2, y2; int line; };
    std::vector<Seg> segments;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;

    for (const auto& m : moves) {
        if (m.mode != MotionMode::Feed) continue;
        if (!m.hasE || m.e <= prev.e) { prev = m; continue; }

        double dist = moveDistanceXY(prev, m);
        if (dist > 0.1) {
            segments.push_back({prev.x, prev.y, m.x, m.y, m.lineNumber});
        }
        prev = m;
    }

    int maxSegs = std::min(static_cast<int>(segments.size()),
                           params_.maxSegmentsForOverlap);
    for (int i = 0; i < maxSegs; ++i) {
        for (int j = i + 1; j < maxSegs; ++j) {
            const auto& s1 = segments[i];
            const auto& s2 = segments[j];

            double dx1 = s1.x2 - s1.x1, dy1 = s1.y2 - s1.y1;
            double dx2 = s2.x2 - s2.x1, dy2 = s2.y2 - s2.y1;
            double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (len1 < 0.1 || len2 < 0.1) continue;

            double dot = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
            if (std::abs(std::abs(dot) - 1.0) > 0.1) continue;

            double midX1 = (s1.x1 + s1.x2) / 2, midY1 = (s1.y1 + s1.y2) / 2;
            double midX2 = (s2.x1 + s2.x2) / 2, midY2 = (s2.y1 + s2.y2) / 2;
            double dist = std::sqrt((midX1 - midX2) * (midX1 - midX2) +
                                    (midY1 - midY2) * (midY1 - midY2));

            if (dist < params_.overlapExtrusionWidthMm * (1.0 - params_.overlapTolerancePercent / 100.0)) {
                double area = std::min(len1, len2) * (params_.overlapExtrusionWidthMm - dist);
                double pct = (1.0 - dist / params_.overlapExtrusionWidthMm) * 100.0;
                result.overlaps.push_back({s1.line, s2.line, area, pct});
            }
        }
    }

    result.overlapCount = static_cast<int>(result.overlaps.size());
    result.totalOverlapArea = std::accumulate(
        result.overlaps.begin(), result.overlaps.end(), 0.0,
        [](double s, const OverlapRegion& o) { return s + o.area; });
    result.hasSignificantOverlaps = std::any_of(
        result.overlaps.begin(), result.overlaps.end(),
        [](const OverlapRegion& o) { return o.percentage > 50; });
}

void PathTopologyDetector::detectSymmetry(
    const std::vector<ParsedMove>& moves, PathTopologyResult& result) const {

    struct Pt { double x, y; };
    std::vector<Pt> positions;
    for (const auto& m : moves) {
        if (m.mode == MotionMode::Feed)
            positions.push_back({m.x, m.y});
    }

    if (positions.size() < 20) return;

    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
    for (const auto& p : positions) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    double centerX = (minX + maxX) / 2, centerY = (minY + maxY) / 2;

    int xMirrorMatches = 0;
    for (const auto& p : positions) {
        double mx = 2 * centerX - p.x;
        for (const auto& p2 : positions) {
            if (std::abs(p2.x - mx) < params_.symmetryToleranceMm &&
                std::abs(p2.y - p.y) < params_.symmetryToleranceMm) {
                xMirrorMatches++;
                break;
            }
        }
    }
    double xMirrorPct = (static_cast<double>(xMirrorMatches) / positions.size()) * 100.0;

    int yMirrorMatches = 0;
    for (const auto& p : positions) {
        double my = 2 * centerY - p.y;
        for (const auto& p2 : positions) {
            if (std::abs(p2.x - p.x) < params_.symmetryToleranceMm &&
                std::abs(p2.y - my) < params_.symmetryToleranceMm) {
                yMirrorMatches++;
                break;
            }
        }
    }
    double yMirrorPct = (static_cast<double>(yMirrorMatches) / positions.size()) * 100.0;

    if (xMirrorPct > 70) {
        result.symmetryType = PathTopologyResult::SymmetryType::MirrorX;
        result.matchedPercentage = xMirrorPct;
        result.symmetryAxisValue = centerX;
        result.symmetryAxisIsX = true;
    } else if (yMirrorPct > 70) {
        result.symmetryType = PathTopologyResult::SymmetryType::MirrorY;
        result.matchedPercentage = yMirrorPct;
        result.symmetryAxisValue = centerY;
        result.symmetryAxisIsX = false;
    } else if (xMirrorPct > 50 && yMirrorPct > 50) {
        result.symmetryType = PathTopologyResult::SymmetryType::Bilateral;
        result.matchedPercentage = (xMirrorPct + yMirrorPct) / 2.0;
    }

    result.isSymmetric = result.matchedPercentage > 70;
    result.symmetryScore = std::round(result.matchedPercentage);
}

PathTopologyResult PathTopologyDetector::analyze(
    const std::vector<std::string>& gcodeLines) const {

    PathTopologyResult result;

    // Parse all moves.
    std::vector<ParsedMove> moves;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);
        if (isMotion(m) || m.mode != MotionMode::None) {
            moves.push_back(m);
        }
        prev = m;
    }

    detectLoops(moves, result);
    detectSelfIntersections(moves, result);
    detectOverlaps(moves, result);
    detectSymmetry(moves, result);

    // Recommendations.
    if (result.loopCount > 0) {
        result.recommendations.push_back(
            std::format("{} loops, max {} iterations", result.loopCount, result.maxIterations));
    }
    if (result.hasCuttingIntersections) {
        int cuttingX = static_cast<int>(std::count_if(
            result.intersections.begin(), result.intersections.end(),
            [](const SelfIntersection& si) { return !si.isRapid; }));
        result.recommendations.push_back(
            std::format("{} cutting self-intersections — check for gouging", cuttingX));
    }
    if (result.overlapCount > 0) {
        result.recommendations.push_back(
            std::format("{} overlapping paths — may cause over-extrusion", result.overlapCount));
    }
    if (result.isSymmetric) {
        result.recommendations.push_back(
            std::format("Symmetric toolpath ({:.0f}% match)", result.matchedPercentage));
    }
    if (result.recommendations.empty()) {
        result.recommendations.push_back("No topology issues detected");
    }

    return result;
}

} // namespace tether::analysis
