/**
 * @file TestPathSVG.hpp
 * @brief General-purpose SVG visualization for motion planner / G-code tests
 *
 * Provides a reusable SVG canvas that renders:
 * - Grid (configurable spacing)
 * - G-code toolpaths (lines + arcs) from GCode::PlanningSegment
 * - Synthetic G-code text labels at each segment endpoint
 * - Multiple overlaid paths (original vs blended)
 * - Info panel with metadata
 *
 * Output is sorted into subject directories under a configurable root
 * (env TEST_SVG_DIR, default "test_output/svgs").
 *
 * Used by G64 corner/arc tests, motion planner integration tests, and
 * other PlanningSegment-based test files.
 */

#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <array>
#include <limits>
#include <filesystem>
#include <fstream>
#include <cstdlib>

#include "tether/gcode/motion/InterpolationStrategy.hpp"

namespace TestPathSVG {

// ============================================================================
// Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;

// ============================================================================
// Output directory management
// ============================================================================

inline std::string getOutputRoot() {
    const char* env = std::getenv("TEST_SVG_DIR");
    if (env && env[0]) return std::string(env);
    return "test_output/svgs";
}

inline std::string subjectDir(const std::string& subject) {
    return getOutputRoot() + "/" + subject;
}

inline void ensureDir(const std::string& dir) {
    if (!std::filesystem::exists(dir)) {
        std::filesystem::create_directories(dir);
    }
}

inline std::string subjectPath(const std::string& subject,
                               const std::string& filename) {
    std::string dir = subjectDir(subject);
    ensureDir(dir);
    return dir + "/" + filename;
}

// ============================================================================
// SVG canvas — coordinate transform + primitives
// ============================================================================

struct Bounds {
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    void include(double x, double y) {
        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
    }
    double width() const { return maxX - minX; }
    double height() const { return maxY - minY; }
};

class Canvas {
public:
    Canvas(int w = 900, int h = 700, double margin = 50.0)
        : width_(w), height_(h), margin_(margin) {}

    void setBounds(const Bounds& b) {
        bounds_ = b;
        if (bounds_.width() < 1e-9) { bounds_.minX -= 1; bounds_.maxX += 1; }
        if (bounds_.height() < 1e-9) { bounds_.minY -= 1; bounds_.maxY += 1; }
        computeTransform();
    }

    void addBounds(const Bounds& b) {
        bounds_.include(b.minX, b.minY);
        bounds_.include(b.maxX, b.maxY);
        computeTransform();
    }

    double tx(double x) const {
        return margin_ + (x - bounds_.minX) * scale_ + offX_;
    }
    double ty(double y) const {
        // Flip Y: SVG origin is top-left, math origin is bottom-left
        return height_ - margin_ - (y - bounds_.minY) * scale_ - offY_;
    }

    std::string header(const std::string& title) const {
        std::ostringstream s;
        s << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        s << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
          << "width=\"" << width_ << "\" height=\"" << height_ << "\" "
          << "viewBox=\"0 0 " << width_ << " " << height_ << "\">\n";
        s << "  <rect width=\"100%\" height=\"100%\" fill=\"#fafafa\"/>\n";
        s << "  <title>" << escapeXml(title) << "</title>\n";
        s << "  <defs>\n";
        s << "    <marker id=\"arrow\" markerWidth=\"8\" markerHeight=\"8\" "
          << "refX=\"6\" refY=\"3\" orient=\"auto\">\n";
        s << "      <path d=\"M0,0 L0,6 L7,3 z\" fill=\"#666\"/>\n";
        s << "    </marker>\n";
        s << "  </defs>\n";
        return s.str();
    }

    std::string footer() const { return "</svg>\n"; }

    std::string grid(double spacing = 5.0) const {
        std::ostringstream s;
        s << "  <g id=\"grid\" stroke=\"#e0e0e0\" stroke-width=\"0.5\">\n";
        // Vertical lines
        for (double x = std::floor(bounds_.minX / spacing) * spacing;
             x <= bounds_.maxX + spacing * 0.5; x += spacing) {
            s << "    <line x1=\"" << tx(x) << "\" y1=\"" << ty(bounds_.minY)
              << "\" x2=\"" << tx(x) << "\" y2=\"" << ty(bounds_.maxY) << "\"/>\n";
        }
        // Horizontal lines
        for (double y = std::floor(bounds_.minY / spacing) * spacing;
             y <= bounds_.maxY + spacing * 0.5; y += spacing) {
            s << "    <line x1=\"" << tx(bounds_.minX) << "\" y1=\"" << ty(y)
              << "\" x2=\"" << tx(bounds_.maxX) << "\" y2=\"" << ty(y) << "\"/>\n";
        }
        s << "  </g>\n";
        return s.str();
    }

    std::string text(int x, int y, const std::string& str,
                     const std::string& fill = "#333",
                     int size = 11) const {
        std::ostringstream s;
        s << "  <text x=\"" << x << "\" y=\"" << y
          << "\" font-family=\"monospace\" font-size=\"" << size
          << "\" fill=\"" << fill << "\">" << escapeXml(str) << "</text>\n";
        return s.str();
    }

    std::string line(double x1, double y1, double x2, double y2,
                     const std::string& stroke = "#333",
                     double sw = 1.0,
                     const std::string& dash = "") const {
        std::ostringstream s;
        s << "  <line x1=\"" << tx(x1) << "\" y1=\"" << ty(y1)
          << "\" x2=\"" << tx(x2) << "\" y2=\"" << ty(y2)
          << "\" stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"";
        if (!dash.empty()) s << " stroke-dasharray=\"" << dash << "\"";
        s << "/>\n";
        return s.str();
    }

    std::string circle(double cx, double cy, double r,
                       const std::string& fill = "#c33",
                       const std::string& stroke = "none",
                       double sw = 0) const {
        std::ostringstream s;
        s << "  <circle cx=\"" << tx(cx) << "\" cy=\"" << ty(cy)
          << "\" r=\"" << r << "\" fill=\"" << fill << "\"";
        if (stroke != "none") s << " stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"";
        s << "/>\n";
        return s.str();
    }

    std::string polyline(const std::vector<std::pair<double,double>>& pts,
                         const std::string& stroke = "#0066ff",
                         double sw = 1.5,
                         bool fill = false,
                         const std::string& fillColor = "none") const {
        if (pts.empty()) return "";
        std::ostringstream s;
        s << "  <polyline points=\"";
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) s << " ";
            s << std::fixed << std::setprecision(2)
              << tx(pts[i].first) << "," << ty(pts[i].second);
        }
        s << "\" stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"";
        if (fill) s << " fill=\"" << fillColor << "\"";
        else s << " fill=\"none\"";
        s << "/>\n";
        return s.str();
    }

    std::string pathFromPoints(const std::vector<std::pair<double,double>>& pts,
                               const std::string& stroke = "#0066ff",
                               double sw = 1.5) const {
        if (pts.empty()) return "";
        std::ostringstream s;
        s << "  <path d=\"M " << std::fixed << std::setprecision(2)
          << tx(pts[0].first) << " " << ty(pts[0].second);
        for (size_t i = 1; i < pts.size(); ++i) {
            s << " L " << tx(pts[i].first) << " " << ty(pts[i].second);
        }
        s << "\" stroke=\"" << stroke << "\" stroke-width=\"" << sw
          << "\" fill=\"none\"/>\n";
        return s.str();
    }

    std::string labelPoint(double x, double y, const std::string& label,
                           const std::string& color = "#333",
                           int dx = 6, int dy = -6) const {
        std::ostringstream s;
        s << circle(x, y, 3, color);
        s << text(static_cast<int>(tx(x)) + dx,
                  static_cast<int>(ty(y)) + dy, label, color, 10);
        return s.str();
    }

    std::string infoPanel(const std::vector<std::string>& lines,
                          int x = 10, int y = 20) const {
        std::ostringstream s;
        // Background
        int panelH = static_cast<int>(lines.size()) * 14 + 8;
        s << "  <rect x=\"" << (x - 4) << "\" y=\"" << (y - 12)
          << "\" width=\"280\" height=\"" << panelH
          << "\" fill=\"white\" fill-opacity=\"0.85\" stroke=\"#ccc\" "
          << "stroke-width=\"0.5\" rx=\"4\"/>\n";
        int yPos = y;
        for (const auto& l : lines) {
            s << text(x, yPos, l, "#444", 11);
            yPos += 14;
        }
        return s.str();
    }

    std::string gcodePanel(const std::vector<std::string>& gcodeLines,
                           int x = 0, int y = 0) const {
        if (gcodeLines.empty()) return "";
        if (x == 0) x = width_ - 270;
        if (y == 0) y = height_ - static_cast<int>(gcodeLines.size()) * 13 - 30;

        std::ostringstream s;
        int panelH = static_cast<int>(gcodeLines.size()) * 13 + 22;
        int panelW = 260;
        s << "  <rect x=\"" << (x - 5) << "\" y=\"" << (y - 15)
          << "\" width=\"" << panelW << "\" height=\"" << panelH
          << "\" fill=\"white\" fill-opacity=\"0.9\" stroke=\"#ccc\" "
          << "stroke-width=\"0.5\" rx=\"4\"/>\n";
        s << text(x, y, "G-Code:", "#666", 10);
        int yPos = y + 13;
        for (const auto& l : gcodeLines) {
            s << text(x, yPos, l, "#0066cc", 10);
            yPos += 13;
        }
        return s.str();
    }

    int width() const { return width_; }
    int height() const { return height_; }
    double scale() const { return scale_; }

private:
    void computeTransform() {
        double availW = width_ - 2 * margin_;
        double availH = height_ - 2 * margin_;
        double dw = bounds_.width();
        double dh = bounds_.height();
        if (dw < 1e-9) dw = 1;
        if (dh < 1e-9) dh = 1;
        scale_ = std::min(availW / dw, availH / dh);
        offX_ = (availW - dw * scale_) / 2.0;
        offY_ = (availH - dh * scale_) / 2.0;
    }

    static std::string escapeXml(const std::string& s) {
        std::string r;
        for (char c : s) {
            switch (c) {
                case '<': r += "&lt;"; break;
                case '>': r += "&gt;"; break;
                case '&': r += "&amp;"; break;
                case '"': r += "&quot;"; break;
                default: r += c;
            }
        }
        return r;
    }

    int width_, height_;
    double margin_;
    Bounds bounds_;
    double scale_ = 1.0;
    double offX_ = 0, offY_ = 0;
};

// ============================================================================
// Segment helpers — convert PlanningSegment to points and G-code text
// ============================================================================

using Pt = std::pair<double, double>;

inline std::string fmt(double v) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(3) << v;
    return s.str();
}

/**
 * @brief Generate synthetic G-code string from a PlanningSegment.
 */
inline std::string segmentToGCode(const GCode::PlanningSegment& seg) {
    std::ostringstream s;
    if (seg.isRapid) {
        s << "G0";
    } else if (seg.motionType == GCode::SegmentMotionType::ArcCW) {
        s << "G2";
    } else if (seg.motionType == GCode::SegmentMotionType::ArcCCW) {
        s << "G3";
    } else {
        s << "G1";
    }
    s << " X" << fmt(seg.end[0]) << " Y" << fmt(seg.end[1]);
    if (std::abs(seg.end[2]) > 1e-9) s << " Z" << fmt(seg.end[2]);

    if (seg.isArc()) {
        // I/J are relative to start
        double i = seg.center[0] - seg.start[0];
        double j = seg.center[1] - seg.start[1];
        s << " I" << fmt(i) << " J" << fmt(j);
    }
    if (seg.feedRate > 0 && !seg.isRapid) {
        s << " F" << fmt(seg.feedRate);
    }
    return s.str();
}

/**
 * @brief Sample an arc segment into points for polyline rendering.
 */
inline std::vector<Pt> sampleArc(const GCode::PlanningSegment& seg,
                                  int numPoints = 48) {
    std::vector<Pt> pts;
    if (!seg.isArc()) return pts;

    double cx = seg.center[0];
    double cy = seg.center[1];
    double r = seg.arcRadius;
    double startAngle = std::atan2(seg.start[1] - cy, seg.start[0] - cx);
    double endAngle = std::atan2(seg.end[1] - cy, seg.end[0] - cx);
    double sweep = endAngle - startAngle;

    // Normalize sweep based on direction
    int dir = seg.arcDirection();  // +1 for CCW, -1 for CW
    if (dir > 0) {
        // CCW: sweep should be positive
        while (sweep < 0) sweep += 2.0 * PI;
        while (sweep > 2.0 * PI) sweep -= 2.0 * PI;
    } else {
        // CW: sweep should be negative
        while (sweep > 0) sweep -= 2.0 * PI;
        while (sweep < -2.0 * PI) sweep += 2.0 * PI;
    }

    for (int i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints;
        double angle = startAngle + sweep * t;
        pts.push_back({cx + r * std::cos(angle), cy + r * std::sin(angle)});
    }
    return pts;
}

/**
 * @brief Convert a PlanningSegment to a list of 2D points for rendering.
 */
inline std::vector<Pt> segmentToPoints(const GCode::PlanningSegment& seg,
                                        int arcSamples = 48) {
    std::vector<Pt> pts;
    if (seg.isArc()) {
        pts = sampleArc(seg, arcSamples);
    } else {
        pts.push_back({seg.start[0], seg.start[1]});
        pts.push_back({seg.end[0], seg.end[1]});
    }
    return pts;
}

/**
 * @brief Compute bounds from a set of segments.
 */
inline Bounds computeSegmentBounds(const std::vector<GCode::PlanningSegment>& segs) {
    Bounds b;
    for (const auto& seg : segs) {
        b.include(seg.start[0], seg.start[1]);
        b.include(seg.end[0], seg.end[1]);
        if (seg.isArc()) {
            // Include arc extent
            auto pts = sampleArc(seg, 16);
            for (const auto& p : pts) b.include(p.first, p.second);
        }
    }
    return b;
}

// ============================================================================
// High-level SVG generation
// ============================================================================

struct PathSVGData {
    std::string title;
    std::vector<GCode::PlanningSegment> originalSegments;
    std::vector<GCode::PlanningSegment> blendedSegments;  // optional
    std::vector<Pt> blendedPath;  // optional pre-sampled blended path
    std::vector<std::string> infoLines;
    std::vector<std::string> extraGCodeLines;  // additional G-code context
    bool showGCodeLabels = true;
    double gridSpacing = 5.0;
};

inline void generateSVG(const PathSVGData& data,
                        const std::string& subject,
                        const std::string& filename) {
    Canvas canvas(1000, 750, 50);

    // Compute bounds
    Bounds bounds;
    if (!data.originalSegments.empty()) {
        bounds = computeSegmentBounds(data.originalSegments);
    }
    if (!data.blendedSegments.empty()) {
        auto b = computeSegmentBounds(data.blendedSegments);
        bounds.include(b.minX, b.minY);
        bounds.include(b.maxX, b.maxY);
    }
    for (const auto& p : data.blendedPath) bounds.include(p.first, p.second);

    // Add margin
    double margin = 2.0;
    bounds.minX -= margin; bounds.minY -= margin;
    bounds.maxX += margin; bounds.maxY += margin;

    canvas.setBounds(bounds);

    std::ostringstream s;
    s << canvas.header(data.title);
    s << canvas.grid(data.gridSpacing);

    // Original path (dashed)
    if (!data.originalSegments.empty()) {
        s << "  <!-- Original path -->\n";
        s << "  <g id=\"original-path\">\n";
        for (const auto& seg : data.originalSegments) {
            auto pts = segmentToPoints(seg);
            if (pts.size() >= 2) {
                s << canvas.polyline(pts, "#999", 1.0, false);
            }
        }
        s << "  </g>\n";
    }

    // Blended segments (solid, different color)
    if (!data.blendedSegments.empty()) {
        s << "  <!-- Blended segments -->\n";
        s << "  <g id=\"blended-segments\">\n";
        for (const auto& seg : data.blendedSegments) {
            auto pts = segmentToPoints(seg);
            if (pts.size() >= 2) {
                s << canvas.polyline(pts, "#00cc00", 2.0, false);
            }
        }
        s << "  </g>\n";
    }

    // Blended path (solid, blue)
    if (!data.blendedPath.empty()) {
        s << "  <!-- Blended path -->\n";
        s << canvas.pathFromPoints(data.blendedPath, "#0066ff", 2.5);
    }

    // Waypoint markers + G-code labels for original segments
    if (data.showGCodeLabels && !data.originalSegments.empty()) {
        s << "  <!-- Waypoints + G-code labels -->\n";
        // Mark start of first segment
        if (!data.originalSegments.empty()) {
            s << canvas.labelPoint(
                data.originalSegments[0].start[0],
                data.originalSegments[0].start[1],
                "Start", "#006600");
        }
        // Mark each segment endpoint
        for (size_t i = 0; i < data.originalSegments.size(); ++i) {
            const auto& seg = data.originalSegments[i];
            s << canvas.circle(seg.end[0], seg.end[1], 3, "#cc0000");
            // G-code label
            std::string gcode = segmentToGCode(seg);
            s << canvas.text(
                static_cast<int>(canvas.tx(seg.end[0])) + 6,
                static_cast<int>(canvas.ty(seg.end[1])) - 6,
                gcode, "#0066cc", 9);
        }
    }

    // Info panel
    if (!data.infoLines.empty()) {
        s << canvas.infoPanel(data.infoLines, 10, 20);
    }

    // G-code panel
    if (data.showGCodeLabels) {
        std::vector<std::string> gcodeLines;
        for (const auto& seg : data.originalSegments) {
            gcodeLines.push_back(segmentToGCode(seg));
        }
        for (const auto& extra : data.extraGCodeLines) {
            gcodeLines.push_back(extra);
        }
        s << canvas.gcodePanel(gcodeLines);
    }

    s << canvas.footer();

    // Write to file
    std::string path = subjectPath(subject, filename);
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    if (!dir.empty() && !std::filesystem::exists(dir)) {
        std::filesystem::create_directories(dir);
    }
    std::ofstream file(path);
    if (file.is_open()) {
        file << s.str();
        file.close();
    }
}

} // namespace TestPathSVG
