/// @file StockCollisionAnalyzer.cpp
/// @brief Stock, fixture and build-plate collision/clearance analysis.

#include "tether/gcode/analysis/StockCollisionAnalyzer.hpp"
#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace tether::gcode::analysis {

namespace {

struct Bounds3D {
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0, minZ = 0.0, maxZ = 0.0;
    bool valid = false;
    bool circularBed = false;
    bool isStock = false;
};

struct Violation {
    std::string type;
    Severity severity;
    std::string message;
    int lineNumber = 0;
    double x = 0.0, y = 0.0, z = 0.0;
};

std::optional<double> parseCommentValue(const std::string& line,
                                        const std::string& prefix,
                                        char /*delimiter*/) {
    const std::string uline = toUpper(line);
    const std::string uprefix = toUpper(prefix);
    size_t pos = uline.find(uprefix);
    if (pos == std::string::npos) return std::nullopt;
    pos += uprefix.size();
    if (pos < line.size() && (line[pos] == ':' || line[pos] == ',' || line[pos] == '=')) {
        ++pos;
        while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
        size_t end = pos;
        while (end < line.size() && (std::isdigit(static_cast<unsigned char>(line[end])) ||
                                     line[end] == '.' || line[end] == '-' || line[end] == '+')) {
            ++end;
        }
        if (end > pos) {
            try { return std::stod(line.substr(pos, end - pos)); } catch (...) {}
        }
    }
    return std::nullopt;
}

Bounds3D parseStockOrBed(const std::vector<std::string>& gcodeLines) {
    Bounds3D b;
    b.valid = false;
    b.circularBed = false;

    double stockX = 0.0, stockY = 0.0, stockZ = 0.0;
    double bedX = 0.0, bedY = 0.0;
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;
    bool hasStock = false, hasBed = false, hasBounds = false;

    for (size_t i = 0; i < std::min<size_t>(gcodeLines.size(), 500); ++i) {
        const std::string& line = gcodeLines[i];

        if (auto v = parseCommentValue(line, "STOCK_X", ':')) { stockX = *v; hasStock = true; }
        if (auto v = parseCommentValue(line, "STOCK_Y", ':')) { stockY = *v; hasStock = true; }
        if (auto v = parseCommentValue(line, "STOCK_Z", ':')) { stockZ = *v; hasStock = true; }
        if (auto v = parseCommentValue(line, "BED_X", ':')) { bedX = *v; hasBed = true; }
        if (auto v = parseCommentValue(line, "BED_Y", ':')) { bedY = *v; hasBed = true; }

        if (auto v = parseCommentValue(line, "print_area_min_x", ':')) { minX = *v; hasBounds = true; }
        if (auto v = parseCommentValue(line, "print_area_max_x", ':')) { maxX = *v; hasBounds = true; }
        if (auto v = parseCommentValue(line, "print_area_min_y", ':')) { minY = *v; hasBounds = true; }
        if (auto v = parseCommentValue(line, "print_area_max_y", ':')) { maxY = *v; hasBounds = true; }

        if (auto v = parseCommentValue(line, "bed_shape", '=')) {
            std::string s = toUpper(std::string(line));
            if (s.find("CIRCLE") != std::string::npos || s.find("CIRCULAR") != std::string::npos) {
                b.circularBed = true;
            } else if (s.find("0X0") != std::string::npos) {
                // PrusaSlicer bed_shape = 0x0,200x0,200x200,0x200
                // We just treat the bed as a rectangle aligned with origin; parse from comment is non-trivial.
                hasBounds = true;
            }
        }
    }

    if (hasStock) {
        b.minX = -stockX * 0.5;
        b.maxX = stockX * 0.5;
        b.minY = -stockY * 0.5;
        b.maxY = stockY * 0.5;
        b.minZ = 0.0;
        b.maxZ = stockZ;
        b.valid = true;
        b.isStock = true;
    } else if (hasBounds) {
        b.minX = minX;
        b.maxX = maxX;
        b.minY = minY;
        b.maxY = maxY;
        b.minZ = 0.0;
        b.maxZ = 1e6;
        b.valid = true;
    } else if (hasBed) {
        b.minX = -bedX * 0.5;
        b.maxX = bedX * 0.5;
        b.minY = -bedY * 0.5;
        b.maxY = bedY * 0.5;
        b.minZ = 0.0;
        b.maxZ = 1e6;
        b.valid = true;
    }

    return b;
}

bool insideBox(const Bounds3D& b, double x, double y, double z) {
    return x >= b.minX && x <= b.maxX && y >= b.minY && y <= b.maxY && z >= b.minZ && z <= b.maxZ;
}

bool insideCircle(const Bounds3D& b, double x, double y) {
    const double cx = 0.5 * (b.minX + b.maxX);
    const double cy = 0.5 * (b.minY + b.maxY);
    const double rx = 0.5 * (b.maxX - b.minX);
    const double ry = 0.5 * (b.maxY - b.minY);
    const double r = std::min(rx, ry);
    return ((x - cx) * (x - cx) + (y - cy) * (y - cy)) <= r * r;
}

} // namespace

Section analyzeStockCollision(const std::vector<GCode::PlanningSegment>& planningSegments,
                              const std::vector<tether::gcode::SegmentSpeed>& segmentSpeeds,
                              const std::vector<std::string>& gcodeLines,
                              const Options& options) {
    if (planningSegments.empty() || segmentSpeeds.empty()) {
        return {};
    }

    const Bounds3D bounds = parseStockOrBed(gcodeLines);
    const size_t n = std::min(planningSegments.size(), segmentSpeeds.size());

    std::vector<Violation> violations;

    // Auto-detect a safe Z: either 10 mm above stock max or, if no stock, 1 mm above max path Z.
    double maxPathZ = 0.0;
    for (const auto& seg : planningSegments) {
        maxPathZ = std::max(maxPathZ, std::max(seg.start.z(), seg.end.z()));
    }
    const double safeZ = bounds.valid ? (bounds.maxZ + 10.0) : (maxPathZ + 1.0);

    for (size_t i = 0; i < n; ++i) {
        const auto& seg = planningSegments[i];
        const auto& ss = segmentSpeeds[i];
        const int lineNumber = ss.lineNumber;

        // Check both ends of the segment.
        auto check = [&](double x, double y, double z) {
            if (!bounds.valid) return;

            if (bounds.isStock) {
                if (seg.isRapid) {
                    if (insideBox(bounds, x, y, z) ||
                        (bounds.circularBed && z < bounds.maxZ && insideCircle(bounds, x, y))) {
                        violations.push_back({"rapid_collision", Severity::High,
                                              std::format("Rapid move into stock at line {} (Z={:.2f})", lineNumber, z),
                                              lineNumber, x, y, z});
                    }
                }
                if (z < bounds.minZ) {
                    violations.push_back({"cut_below_stock", Severity::High,
                                          std::format("Cut below stock bottom at line {} (Z={:.2f})", lineNumber, z),
                                          lineNumber, x, y, z});
                }

                // Rapid clearance check for CNC-like motion: rapids below safe Z.
                if (seg.isRapid && z < safeZ && z > 0.0) {
                    violations.push_back({"clearance_violation", Severity::Medium,
                                          std::format("Rapid below safe Z ({:.2f} < {:.2f}) at line {}", z, safeZ, lineNumber),
                                          lineNumber, x, y, z});
                }
            }

            if (!seg.isRapid) {
                // Cutting/extruding move outside stock/bed
                bool inside = bounds.circularBed ? insideCircle(bounds, x, y)
                                                 : (x >= bounds.minX && x <= bounds.maxX &&
                                                    y >= bounds.minY && y <= bounds.maxY);
                if (!inside) {
                    violations.push_back({"out_of_bounds", Severity::High,
                                          std::format("Move outside build area at line {} (X={:.1f},Y={:.1f})",
                                                      lineNumber, x, y),
                                          lineNumber, x, y, z});
                }
            }
        };

        check(seg.start.x(), seg.start.y(), seg.start.z());
        check(seg.end.x(), seg.end.y(), seg.end.z());
    }

    Section section;
    section.name = "stock_collision";
    section.displayName = "Stock & Fixture Clearance";

    const bool summaryOnly = (options.detailLevel == "summary");
    const bool fullEvents = (options.detailLevel == "full");
    const size_t topLimit = (options.topEventLimit > 0)
                                ? options.topEventLimit
                                : (fullEvents ? std::numeric_limits<size_t>::max() : 64);

    size_t rapidCollisions = 0;
    size_t clearance = 0;
    size_t outOfBounds = 0;
    size_t belowStock = 0;
    for (const auto& v : violations) {
        if (v.type == "rapid_collision") ++rapidCollisions;
        else if (v.type == "clearance_violation") ++clearance;
        else if (v.type == "out_of_bounds") ++outOfBounds;
        else if (v.type == "cut_below_stock") ++belowStock;
    }

    section.metrics.push_back(makeMetric("stock_min_x", bounds.minX));
    section.metrics.push_back(makeMetric("stock_max_x", bounds.maxX));
    section.metrics.push_back(makeMetric("stock_min_y", bounds.minY));
    section.metrics.push_back(makeMetric("stock_max_y", bounds.maxY));
    section.metrics.push_back(makeMetric("stock_min_z", bounds.minZ));
    section.metrics.push_back(makeMetric("stock_max_z", bounds.maxZ));
    section.metrics.push_back(makeMetric("safe_z", safeZ));
    section.metrics.push_back(makeMetric("rapid_collision_count", static_cast<int64_t>(rapidCollisions)));
    section.metrics.push_back(makeMetric("clearance_violation_count", static_cast<int64_t>(clearance)));
    section.metrics.push_back(makeMetric("out_of_bounds_count", static_cast<int64_t>(outOfBounds)));
    section.metrics.push_back(makeMetric("cut_below_stock_count", static_cast<int64_t>(belowStock)));
    section.metrics.push_back(makeMetric("total_violations", static_cast<int64_t>(violations.size())));

    double score = 100.0;
    score -= static_cast<double>(rapidCollisions) * 20.0;
    score -= static_cast<double>(belowStock) * 20.0;
    score -= static_cast<double>(outOfBounds) * 15.0;
    score -= static_cast<double>(clearance) * 10.0;
    section.score = std::clamp(score, 0.0, 100.0);

    section.totalEventCount = violations.size();
    section.hasMoreEvents = violations.size() > topLimit;

    if (summaryOnly) return section;

    std::ranges::sort(violations,
                      [](const Violation& a, const Violation& b) {
                          return static_cast<int>(a.severity) > static_cast<int>(b.severity);
                      });

    const size_t eventCount = std::min(topLimit, violations.size());
    section.events.reserve(eventCount);
    for (size_t i = 0; i < eventCount; ++i) {
        const auto& v = violations[i];
        section.events.push_back(Event{
            .id = std::format("{}:line{}:z{:.2f}", v.type, v.lineNumber, v.z),
            .type = v.type,
            .severity = v.severity,
            .message = v.message,
            .metricValue = 0.0,
            .detailsJson = std::format(
                R"({{"line":{},"x":{:.3f},"y":{:.3f},"z":{:.3f},"type":"{}","safe_z":{:.2f}}})" ,
                v.lineNumber, v.x, v.y, v.z, v.type, safeZ)});
    }

    return section;
}

} // namespace tether::gcode::analysis
