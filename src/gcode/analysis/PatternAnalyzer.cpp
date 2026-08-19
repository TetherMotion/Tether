/// @file PatternAnalyzer.cpp
/// @brief Detect toolpath patterns (spirals, concentric contours, zigzags) from
///        already parsed G-code segments.

#include "tether/gcode/analysis/PatternAnalyzer.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

using GCode::InterpolationPlane;

#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

namespace tether::gcode::analysis {

namespace {

using GCode::PlanningSegment;
using GCode::SegmentMotionType;
using GCode::InterpolationPlane;

constexpr double kCloseTol = 0.5;            // mm: loop closure tolerance
constexpr double kCenterTol = 0.5;           // mm: shared-center tolerance
constexpr double kConcentricCenterTol = 0.6; // mm: concentric centroid tolerance
constexpr double kMinLoopLength = 1.0;       // mm: minimum closed-loop length
constexpr double kMinZigzagStrokeLength = 0.5; // mm: minimum zigzag pass length
constexpr double kRadiusRelTol = 0.05;       // relative radius tolerance
constexpr double kSpacingRelTol = 0.5;       // relative zigzag spacing tolerance
constexpr std::size_t kMaxLoopWindow = 200;  // segments: max loop search window
constexpr std::size_t kMinSpiralArcs = 2;    // arcs required for a spiral
constexpr std::size_t kMinZigzagStrokes = 3; // strokes required for a zigzag

/// Two-dimensional projection of a PlanningSegment in its active plane.
struct Seg2D {
    const PlanningSegment* seg = nullptr;
    const tether::gcode::SegmentSpeed* speed = nullptr;
    std::size_t index = 0;

    bool isArc = false;
    bool isLinear = false;

    InterpolationPlane plane = InterpolationPlane::XY;
    int u = 0; // primary plane axis
    int v = 1; // secondary plane axis
    int w = 2; // perpendicular axis

    double startX = 0.0;
    double startY = 0.0;
    double endX = 0.0;
    double endY = 0.0;
    double centerX = 0.0;
    double centerY = 0.0;
    double dirX = 0.0;
    double dirY = 0.0;
    double length = 0.0;
    int line = 0;
};

struct Loop {
    bool isArc = false;
    std::size_t startSeg = 0;
    std::size_t endSeg = 0;
    int startLine = 0;
    int endLine = 0;
    InterpolationPlane plane = InterpolationPlane::XY;
    double cx = 0.0;
    double cy = 0.0;
    double size = 0.0; // radius for arcs, mean centroid distance for polygons
    double length = 0.0;
    std::size_t segCount = 0;
};

struct SpiralPattern {
    std::size_t startSeg = 0;
    std::size_t endSeg = 0;
    int startLine = 0;
    int endLine = 0;
    double cx = 0.0;
    double cy = 0.0;
    double radiusStart = 0.0;
    double radiusEnd = 0.0;
    double angleRange = 0.0;
    double length = 0.0;
    std::size_t segCount = 0;
};

struct ConcentricPattern {
    std::size_t startSeg = 0;
    std::size_t endSeg = 0;
    int startLine = 0;
    int endLine = 0;
    double cx = 0.0;
    double cy = 0.0;
    double minSize = 0.0;
    double maxSize = 0.0;
    std::size_t loopCount = 0;
    std::size_t segCount = 0;
    double length = 0.0;
};

struct ZigzagPattern {
    std::size_t startSeg = 0;
    std::size_t endSeg = 0;
    int startLine = 0;
    int endLine = 0;
    std::size_t strokeCount = 0;
    double avgSpacing = 0.0;
    double length = 0.0;
    double dirX = 0.0;
    double dirY = 0.0;
};

struct PatternEvent {
    std::string type;
    int startLine = 0;
    int endLine = 0;
    double metricValue = 0.0; // length, or arc/linear ratio for aggregate events
    std::string message;
    std::string detailsJson;
};

std::vector<Seg2D> buildSegments(const std::vector<PlanningSegment>& planning,
                                 const std::vector<tether::gcode::SegmentSpeed>& speeds) {
    std::vector<Seg2D> out;
    const std::size_t n = std::min(planning.size(), speeds.size());
    out.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        const auto& seg = planning[i];
        const auto& ss = speeds[i];

        if (seg.isRapid) continue;
        if (seg.motionType != SegmentMotionType::Linear && !seg.isArc()) continue;

        Seg2D s;
        s.seg = &seg;
        s.speed = &ss;
        s.index = i;
        s.isArc = seg.isArc();
        s.isLinear = (seg.motionType == SegmentMotionType::Linear);
        s.line = ss.lineNumber;
        s.length = seg.segmentLength;
        s.plane = seg.plane;

        getPlaneAxes(seg.plane, s.u, s.v, s.w);

        s.startX = posCoord(seg.start, s.u);
        s.startY = posCoord(seg.start, s.v);
        s.endX = posCoord(seg.end, s.u);
        s.endY = posCoord(seg.end, s.v);

        if (s.isArc) {
            s.centerX = posCoord(seg.center, s.u);
            s.centerY = posCoord(seg.center, s.v);
        }

        double dx = s.endX - s.startX;
        double dy = s.endY - s.startY;
        double len = std::hypot(dx, dy);
        if (len > 1e-12) {
            s.dirX = dx / len;
            s.dirY = dy / len;
        }

        out.push_back(s);
    }

    return out;
}

inline double twoDDistance(double x1, double y1, double x2, double y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

inline double pointToLineDistance(double px, double py,
                                  double lx, double ly,
                                  double dX, double dY) {
    // |(p - l) x d|
    double cross = (px - lx) * dY - (py - ly) * dX;
    double dirLen = std::hypot(dX, dY);
    if (dirLen < 1e-12) return 0.0;
    return std::abs(cross) / dirLen;
}

std::vector<SpiralPattern> detectSpirals(const std::vector<Seg2D>& segs) {
    std::vector<SpiralPattern> spirals;
    const std::size_t n = segs.size();
    std::size_t i = 0;

    while (i < n) {
        if (!segs[i].isArc) { ++i; continue; }

        double sumCx = segs[i].centerX;
        double sumCy = segs[i].centerY;
        double sumSweep = std::abs(segs[i].seg->arcSweep);
        double sumLen = segs[i].length;
        double prevR = segs[i].seg->arcRadius;
        int trend = 0; // 1 = increasing, -1 = decreasing
        std::size_t arcCount = 1;
        const int firstDir = segs[i].seg->arcDirection();
        const InterpolationPlane plane = segs[i].plane;

        std::size_t j = i + 1;
        for (; j < n; ++j) {
            const auto& sg = segs[j];
            if (!sg.isArc) break;
            if (sg.plane != plane) break;
            if (sg.seg->arcDirection() != firstDir) break;

            const double r = sg.seg->arcRadius;
            if (r < 0.01) break;

            const double avgCx = sumCx / static_cast<double>(arcCount);
            const double avgCy = sumCy / static_cast<double>(arcCount);
            const double dCenter = twoDDistance(sg.centerX, sg.centerY, avgCx, avgCy);
            if (dCenter > kCenterTol + kRadiusRelTol * r) break;

            const double dr = r - prevR;
            const double radiusTol = 0.01 * prevR + 0.005;
            if (dr > radiusTol) {
                if (trend == -1) break;
                trend = 1;
            } else if (dr < -radiusTol) {
                if (trend == 1) break;
                trend = -1;
            }

            sumCx += sg.centerX;
            sumCy += sg.centerY;
            sumSweep += std::abs(sg.seg->arcSweep);
            sumLen += sg.length;
            prevR = r;
            ++arcCount;
        }

        if (arcCount >= kMinSpiralArcs && trend != 0 &&
            std::abs(prevR - segs[i].seg->arcRadius) > 0.01) {
            SpiralPattern p;
            p.startSeg = i;
            p.endSeg = j - 1;
            p.startLine = segs[i].line;
            p.endLine = segs[j - 1].line;
            p.cx = sumCx / static_cast<double>(arcCount);
            p.cy = sumCy / static_cast<double>(arcCount);
            p.radiusStart = segs[i].seg->arcRadius;
            p.radiusEnd = prevR;
            p.angleRange = sumSweep;
            p.length = sumLen;
            p.segCount = arcCount;
            spirals.push_back(p);
            i = j;
        } else {
            ++i;
        }
    }

    return spirals;
}

std::optional<Loop> tryCloseLoop(const std::vector<Seg2D>& segs, std::size_t s) {
    const std::size_t n = segs.size();
    const std::size_t limit = std::min(s + kMaxLoopWindow, n);

    double arcSweep = 0.0;
    double sumArcCx = 0.0;
    double sumArcCy = 0.0;
    double sumArcR = 0.0;
    std::size_t arcCount = 0;
    int firstArcDir = 0;
    InterpolationPlane firstPlane = segs[s].plane;

    double linearLen = 0.0;

    for (std::size_t e = s; e < limit; ++e) {
        const auto& sg = segs[e];

        if (sg.isArc) {
            if (linearLen > 0.0) break; // cannot mix after linear

            if (arcCount > 0) {
                if (sg.plane != firstPlane) break;
                if (sg.seg->arcDirection() != firstArcDir) break;

                const double avgR = sumArcR / static_cast<double>(arcCount);
                const double avgCx = sumArcCx / static_cast<double>(arcCount);
                const double avgCy = sumArcCy / static_cast<double>(arcCount);
                const double dr = std::abs(sg.seg->arcRadius - avgR);
                const double dc = twoDDistance(sg.centerX, sg.centerY, avgCx, avgCy);
                if (dr > kRadiusRelTol * avgR + 0.01) break;
                if (dc > kCenterTol + kRadiusRelTol * avgR) break;
            } else {
                if (sg.seg->arcRadius <= 0.01) break;
                firstArcDir = sg.seg->arcDirection();
                firstPlane = sg.plane;
            }

            arcSweep += std::abs(sg.seg->arcSweep);
            sumArcCx += sg.centerX;
            sumArcCy += sg.centerY;
            sumArcR += sg.seg->arcRadius;
            ++arcCount;

            if (arcSweep >= 1.75 * std::numbers::pi) {
                const double closeDist = twoDDistance(sg.endX, sg.endY, segs[s].startX, segs[s].startY);
                if (closeDist < kCloseTol || arcSweep >= 1.95 * std::numbers::pi) {
                    Loop loop;
                    loop.isArc = true;
                    loop.plane = firstPlane;
                    loop.startSeg = s;
                    loop.endSeg = e;
                    loop.startLine = segs[s].line;
                    loop.endLine = segs[e].line;
                    loop.cx = sumArcCx / static_cast<double>(arcCount);
                    loop.cy = sumArcCy / static_cast<double>(arcCount);
                    loop.size = sumArcR / static_cast<double>(arcCount);
                    double len = 0.0;
                    for (std::size_t k = s; k <= e; ++k) len += segs[k].length;
                    loop.length = len;
                    loop.segCount = arcCount;
                    return loop;
                }
            }
            if (arcSweep > 2.25 * std::numbers::pi) break;
            continue;
        }

        if (sg.isLinear) {
            if (arcCount > 0) break; // cannot mix
            linearLen += sg.length;

            if (e >= s + 2) {
                const double closeDist = twoDDistance(sg.endX, sg.endY, segs[s].startX, segs[s].startY);
                if (closeDist < kCloseTol && linearLen > kMinLoopLength) {
                    Loop loop;
                    loop.isArc = false;
                    loop.plane = firstPlane;
                    loop.startSeg = s;
                    loop.endSeg = e;
                    loop.startLine = segs[s].line;
                    loop.endLine = segs[e].line;

                    double sumVx = 0.0;
                    double sumVy = 0.0;
                    for (std::size_t k = s; k <= e; ++k) {
                        sumVx += segs[k].startX;
                        sumVy += segs[k].startY;
                    }
                    sumVx += sg.endX;
                    sumVy += sg.endY;
                    const double vCount = static_cast<double>(e - s + 2);
                    const double cx = sumVx / vCount;
                    const double cy = sumVy / vCount;

                    double avgDist = 0.0;
                    for (std::size_t k = s; k <= e; ++k) {
                        avgDist += twoDDistance(segs[k].startX, segs[k].startY, cx, cy);
                    }
                    avgDist += twoDDistance(sg.endX, sg.endY, cx, cy);
                    avgDist /= vCount;

                    loop.cx = cx;
                    loop.cy = cy;
                    loop.size = avgDist;
                    loop.length = linearLen;
                    loop.segCount = e - s + 1;
                    return loop;
                }
            }
            continue;
        }

        break; // unsupported segment type
    }

    return std::nullopt;
}

std::vector<Loop> detectLoops(const std::vector<Seg2D>& segs) {
    std::vector<Loop> loops;
    const std::size_t n = segs.size();
    std::size_t s = 0;
    while (s < n) {
        auto loop = tryCloseLoop(segs, s);
        if (loop) {
            loops.push_back(*loop);
            s = loop->endSeg + 1;
        } else {
            ++s;
        }
    }
    return loops;
}

std::vector<ConcentricPattern> detectConcentric(const std::vector<Loop>& loops) {
    std::vector<ConcentricPattern> concentrics;
    const std::size_t m = loops.size();
    std::size_t i = 0;

    while (i < m) {
        double sumCx = loops[i].cx;
        double sumCy = loops[i].cy;
        double minSize = loops[i].size;
        double maxSize = loops[i].size;
        double prevSize = loops[i].size;
        int trend = 0;
        std::size_t count = 1;
        std::size_t bestEnd = i;

        for (std::size_t j = i + 1; j < m; ++j) {
            if (loops[j].plane != loops[i].plane) break;

            const double avgCx = sumCx / static_cast<double>(count);
            const double avgCy = sumCy / static_cast<double>(count);
            const double dCenter = twoDDistance(loops[j].cx, loops[j].cy, avgCx, avgCy);
            if (dCenter > kConcentricCenterTol + kRadiusRelTol * loops[j].size) break;

            const double ds = loops[j].size - prevSize;
            const double sizeTol = kRadiusRelTol * prevSize + 0.01;
            if (std::abs(ds) < sizeTol) break;

            if (trend == 0) {
                trend = (ds > 0.0) ? 1 : -1;
            } else if ((trend == 1 && ds < -sizeTol) || (trend == -1 && ds > sizeTol)) {
                break;
            }

            sumCx += loops[j].cx;
            sumCy += loops[j].cy;
            minSize = std::min(minSize, loops[j].size);
            maxSize = std::max(maxSize, loops[j].size);
            prevSize = loops[j].size;
            ++count;
            bestEnd = j;
        }

        if (count >= 2) {
            ConcentricPattern p;
            p.startSeg = loops[i].startSeg;
            p.endSeg = loops[bestEnd].endSeg;
            p.startLine = loops[i].startLine;
            p.endLine = loops[bestEnd].endLine;
            p.cx = sumCx / static_cast<double>(count);
            p.cy = sumCy / static_cast<double>(count);
            p.minSize = minSize;
            p.maxSize = maxSize;
            p.loopCount = count;
            p.segCount = 0;
            p.length = 0.0;
            for (std::size_t k = i; k <= bestEnd; ++k) {
                p.segCount += loops[k].segCount;
                p.length += loops[k].length;
            }
            concentrics.push_back(p);
            i = bestEnd + 1;
        } else {
            ++i;
        }
    }

    return concentrics;
}

std::vector<ZigzagPattern> detectZigzag(const std::vector<Seg2D>& segs) {
    std::vector<ZigzagPattern> zigzags;

    std::vector<std::size_t> strokes;
    for (std::size_t i = 0; i < segs.size(); ++i) {
        if (segs[i].isLinear && segs[i].length >= kMinZigzagStrokeLength) {
            strokes.push_back(i);
        }
    }

    if (strokes.size() < kMinZigzagStrokes) return zigzags;

    std::size_t s = 0;
    const std::size_t m = strokes.size();
    while (s + kMinZigzagStrokes - 1 < m) {
        const auto& first = segs[strokes[s]];
        std::size_t e = s + 1;
        double avgSpacing = 0.0;
        std::size_t spacingCount = 0;

        while (e < m) {
            const auto& prev = segs[strokes[e - 1]];
            const auto& cur = segs[strokes[e]];

            if (cur.plane != first.plane) break;

            const double dot = prev.dirX * cur.dirX + prev.dirY * cur.dirY;
            if (dot > -0.7) break; // not a 180-degree direction change

            const bool even = ((e - s) % 2) == 0;
            const double dotFirst = first.dirX * cur.dirX + first.dirY * cur.dirY;
            if (even) {
                if (dotFirst < 0.7) break;
            } else {
                if (dotFirst > -0.7) break;
            }

            if (e >= s + 2) {
                const auto& twoBack = segs[strokes[e - 2]];
                const double d = pointToLineDistance(cur.startX, cur.startY,
                                                     twoBack.startX, twoBack.startY,
                                                     twoBack.dirX, twoBack.dirY);
                if (d < 0.05) break; // not meaningful spacing
                if (spacingCount == 0) {
                    avgSpacing = d;
                } else {
                    if (std::abs(d - avgSpacing) > kSpacingRelTol * avgSpacing + 0.1) break;
                    avgSpacing = (avgSpacing * static_cast<double>(spacingCount) + d) /
                                 static_cast<double>(spacingCount + 1);
                }
                ++spacingCount;
            }

            ++e;
        }

        if (e - s >= kMinZigzagStrokes) {
            ZigzagPattern p;
            p.startSeg = strokes[s];
            p.endSeg = strokes[e - 1];
            p.startLine = segs[strokes[s]].line;
            p.endLine = segs[strokes[e - 1]].line;
            p.strokeCount = e - s;
            p.avgSpacing = (spacingCount > 0) ? avgSpacing : 0.0;
            double len = 0.0;
            for (std::size_t k = s; k < e; ++k) len += segs[strokes[k]].length;
            p.length = len;
            p.dirX = first.dirX;
            p.dirY = first.dirY;
            zigzags.push_back(p);
            s = e;
        } else {
            ++s;
        }
    }

    return zigzags;
}

} // namespace

Section analyzePatterns(const std::vector<GCode::PlanningSegment>& planningSegments,
                        const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                        const std::vector<std::string>& /*gcodeLines*/,
                        const Options& options) {
    Section section;

    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return section;
    }

    const auto segs = buildSegments(planningSegments, segmentSpeeds);
    if (segs.empty()) {
        return section;
    }

    const std::size_t n = std::min(planningSegments.size(), segmentSpeeds.size());

    std::int64_t arcCount = 0;
    std::int64_t linearCount = 0;
    double totalArcLength = 0.0;
    double totalLinearLength = 0.0;
    double totalArcAngle = 0.0;
    double arcCenterWeightSumX = 0.0;
    double arcCenterWeightSumY = 0.0;
    double arcAngleWeight = 0.0;
    int firstArcLine = 0;
    int lastArcLine = 0;
    int firstLinearLine = 0;
    int lastLinearLine = 0;

    for (std::size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        if (seg.isRapid) continue;
        if (seg.isArc()) {
            ++arcCount;
            totalArcLength += seg.segmentLength;
            const double sweep = std::abs(seg.arcSweep);
            totalArcAngle += sweep;
            int u = 0, v = 0, w = 0;
            getPlaneAxes(seg.plane, u, v, w);
            const double cx = posCoord(seg.center, u);
            const double cy = posCoord(seg.center, v);
            arcCenterWeightSumX += cx * sweep;
            arcCenterWeightSumY += cy * sweep;
            arcAngleWeight += sweep;
            if (firstArcLine == 0) firstArcLine = segmentSpeeds[i].lineNumber;
            lastArcLine = segmentSpeeds[i].lineNumber;
        } else if (seg.motionType == SegmentMotionType::Linear) {
            ++linearCount;
            totalLinearLength += seg.segmentLength;
            if (firstLinearLine == 0) firstLinearLine = segmentSpeeds[i].lineNumber;
            lastLinearLine = segmentSpeeds[i].lineNumber;
        }
    }

    const auto spirals = detectSpirals(segs);
    const auto loops = detectLoops(segs);
    const auto concentrics = detectConcentric(loops);
    const auto zigzags = detectZigzag(segs);

    std::vector<PatternEvent> events;
    events.reserve(spirals.size() + concentrics.size() + zigzags.size() + 2);

    for (const auto& sp : spirals) {
        PatternEvent ev;
        ev.type = "spiral";
        ev.startLine = sp.startLine;
        ev.endLine = sp.endLine;
        ev.metricValue = sp.length;
        ev.message = std::format("Spiral toolpath from line {} to {} ({:.1f} mm)",
                                 sp.startLine, sp.endLine, sp.length);
        ev.detailsJson = std::format(
            R"!({{"start_line":{},"end_line":{},"center_x":{:.3f},"center_y":{:.3f},"angle_range_rad":{:.4f},"radius_start_mm":{:.3f},"radius_end_mm":{:.3f},"line_count":{},"length_mm":{:.3f}}})!",
            sp.startLine, sp.endLine, sp.cx, sp.cy, sp.angleRange,
            sp.radiusStart, sp.radiusEnd, sp.segCount, sp.length);
        events.push_back(std::move(ev));
    }

    for (const auto& c : concentrics) {
        PatternEvent ev;
        ev.type = "concentric";
        ev.startLine = c.startLine;
        ev.endLine = c.endLine;
        ev.metricValue = c.length;
        ev.message = std::format(
            "Concentric contour ({} loops, size {:.2f}-{:.2f} mm) from line {} to {}",
            c.loopCount, c.minSize, c.maxSize, c.startLine, c.endLine);
        ev.detailsJson = std::format(
            R"!({{"start_line":{},"end_line":{},"center_x":{:.3f},"center_y":{:.3f},"size_min_mm":{:.3f},"size_max_mm":{:.3f},"loop_count":{},"line_count":{},"length_mm":{:.3f}}})!",
            c.startLine, c.endLine, c.cx, c.cy, c.minSize, c.maxSize,
            c.loopCount, c.segCount, c.length);
        events.push_back(std::move(ev));
    }

    for (const auto& z : zigzags) {
        PatternEvent ev;
        ev.type = "zigzag";
        ev.startLine = z.startLine;
        ev.endLine = z.endLine;
        ev.metricValue = z.length;
        ev.message = std::format(
            "Zigzag pattern ({} strokes, spacing {:.2f} mm) from line {} to {}",
            z.strokeCount, z.avgSpacing, z.startLine, z.endLine);
        ev.detailsJson = std::format(
            R"!({{"start_line":{},"end_line":{},"stroke_count":{},"avg_spacing_mm":{:.3f},"line_count":{},"length_mm":{:.3f}}})!",
            z.startLine, z.endLine, z.strokeCount, z.avgSpacing, z.strokeCount, z.length);
        events.push_back(std::move(ev));
    }

    const double arcLinearRatio = totalLinearLength > 1e-12 ? totalArcLength / totalLinearLength : 0.0;

    if (arcCount > 0) {
        PatternEvent ev;
        ev.type = "arc";
        ev.startLine = firstArcLine;
        ev.endLine = lastArcLine;
        ev.metricValue = arcLinearRatio;
        ev.message = std::format("{} arc segment{} ({:.1f} mm, {:.2f} rad total)",
                                 arcCount, (arcCount == 1 ? "" : "s"),
                                 totalArcLength, totalArcAngle);
        double avgCx = 0.0;
        double avgCy = 0.0;
        if (arcAngleWeight > 1e-12) {
            avgCx = arcCenterWeightSumX / arcAngleWeight;
            avgCy = arcCenterWeightSumY / arcAngleWeight;
        }
        ev.detailsJson = std::format(
            R"!({{"start_line":{},"end_line":{},"center_x":{:.3f},"center_y":{:.3f},"angle_range_rad":{:.4f},"line_count":{},"length_mm":{:.3f}}})!",
            firstArcLine, lastArcLine, avgCx, avgCy, totalArcAngle, arcCount, totalArcLength);
        events.push_back(std::move(ev));
    }

    if (linearCount > 0) {
        PatternEvent ev;
        ev.type = "linear";
        ev.startLine = firstLinearLine;
        ev.endLine = lastLinearLine;
        ev.metricValue = arcLinearRatio;
        ev.message = std::format("{} linear segment{} ({:.1f} mm)",
                                 linearCount, (linearCount == 1 ? "" : "s"),
                                 totalLinearLength);
        ev.detailsJson = std::format(
            R"!({{"start_line":{},"end_line":{},"line_count":{},"length_mm":{:.3f}}})!",
            firstLinearLine, lastLinearLine, linearCount, totalLinearLength);
        events.push_back(std::move(ev));
    }

    // Detail level handling
    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const std::size_t topLimit = fullEvents
                                     ? std::numeric_limits<std::size_t>::max()
                                     : (options.topEventLimit > 0 ? options.topEventLimit : 64);

    if (!summaryOnly) {
        std::sort(events.begin(), events.end(),
                  [](const PatternEvent& a, const PatternEvent& b) {
                      return a.metricValue > b.metricValue;
                  });
    }

    const std::size_t eventCount = summaryOnly ? 0 : std::min(topLimit, events.size());

    // Score: 100 by default; lower if the toolpath has no interesting patterns
    // or is extremely simple (all linear).
    double score = 100.0;
    if (spirals.empty() && concentrics.empty() && zigzags.empty()) {
        score -= 15.0;
    }
    if (linearCount > 0 && arcCount == 0) {
        score -= 20.0; // all linear is a very simple toolpath
    } else if (arcCount > 0 && linearCount == 0 && spirals.empty() && concentrics.empty()) {
        score -= 5.0; // only circles/arcs without recognizable higher-order pattern
    }
    if (n < 10) {
        score -= 10.0;
    }
    score = std::clamp(score, 0.0, 100.0);

    section.name = "pattern_analysis";
    section.displayName = "Pattern Analysis";
    section.score = score;
    section.totalEventCount = events.size();
    section.hasMoreEvents = events.size() > eventCount;

    section.metrics.push_back(makeMetric("spiral_count", static_cast<std::int64_t>(spirals.size())));
    section.metrics.push_back(makeMetric("concentric_count", static_cast<std::int64_t>(concentrics.size())));
    section.metrics.push_back(makeMetric("zigzag_count", static_cast<std::int64_t>(zigzags.size())));
    section.metrics.push_back(makeMetric("arc_count", arcCount));
    section.metrics.push_back(makeMetric("linear_count", linearCount));

    for (std::size_t i = 0; i < eventCount; ++i) {
        const auto& ev = events[i];

        Event e;
        e.id = std::format("{}:{}-{}-{:d}", ev.type, ev.startLine, ev.endLine,
                           static_cast<std::int64_t>(i));
        e.type = ev.type;
        e.severity = Severity::Info;
        e.message = ev.message;
        e.metricValue = ev.metricValue;
        e.detailsJson = ev.detailsJson;

        section.events.push_back(std::move(e));
    }

    return section;
}

} // namespace tether::gcode::analysis
