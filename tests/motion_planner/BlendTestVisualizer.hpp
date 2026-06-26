/**
 * @file BlendTestVisualizer.hpp
 * @brief Structured SVG output for blend test visualization
 *
 * Generates rich SVG files showing:
 * - Original path (dashed) vs blended path (solid)
 * - Blend curve control points and polygon
 * - Corner point, entry/exit points with labels
 * - Tolerance zone (shaded)
 * - Curvature profile plot
 * - Dimension annotations (angle, radius, entry/exit distances)
 *
 * Works with both MotionPlanner (Vec<Dim,T>) and GCode (Position) types
 * via the BlendVec adapter from BlendCore.
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

#include "tether/motion_planner/BlendCore.hpp"

namespace BlendTest {

// ============================================================================
// SVG canvas and coordinate transform
// ============================================================================

struct SVGBounds {
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

class SVGCanvas {
public:
    SVGCanvas(int width = 800, int height = 600, double margin = 40.0)
        : width_(width), height_(height), margin_(margin) {}

    void setBounds(const SVGBounds& b) {
        bounds_ = b;
        if (bounds_.width() < 1e-9) { bounds_.minX -= 1; bounds_.maxX += 1; }
        if (bounds_.height() < 1e-9) { bounds_.minY -= 1; bounds_.maxY += 1; }
        computeTransform();
    }

    void addBounds(const SVGBounds& b) {
        bounds_.include(b.minX, b.minY);
        bounds_.include(b.maxX, b.maxY);
        computeTransform();
    }

    double tx(double x) const { return margin_ + (x - bounds_.minX) * scale_ + offX_; }
    double ty(double y) const {
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
        s << "    <marker id=\"dot\" markerWidth=\"6\" markerHeight=\"6\" refX=\"3\" refY=\"3\">\n";
        s << "      <circle cx=\"3\" cy=\"3\" r=\"2.5\" fill=\"#c33\"/>\n";
        s << "    </marker>\n";
        s << "  </defs>\n";
        return s.str();
    }

    std::string footer() const { return "</svg>\n"; }

    std::string grid(double spacing = 5.0) const {
        std::ostringstream s;
        s << "  <g id=\"grid\" stroke=\"#e0e0e0\" stroke-width=\"0.5\">\n";
        for (double x = std::floor(bounds_.minX / spacing) * spacing;
             x <= bounds_.maxX; x += spacing) {
            s << "    <line x1=\"" << tx(x) << "\" y1=\"" << ty(bounds_.minY)
              << "\" x2=\"" << tx(x) << "\" y2=\"" << ty(bounds_.maxY) << "\"/>\n";
        }
        for (double y = std::floor(bounds_.minY / spacing) * spacing;
             y <= bounds_.maxY; y += spacing) {
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
                     double width = 1.0,
                     const std::string& dash = "") const {
        std::ostringstream s;
        s << "  <line x1=\"" << tx(x1) << "\" y1=\"" << ty(y1)
          << "\" x2=\"" << tx(x2) << "\" y2=\"" << ty(y2)
          << "\" stroke=\"" << stroke << "\" stroke-width=\"" << width << "\"";
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

    std::string polyline(const std::vector<tether::blend::BlendVec>& pts,
                         const std::string& stroke = "#0066ff",
                         double width = 1.5,
                         bool fill = false,
                         const std::string& fillColor = "none") const {
        if (pts.empty()) return "";
        std::ostringstream s;
        s << "  <polyline points=\"";
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) s << " ";
            s << std::fixed << std::setprecision(2) << tx(pts[i].x) << "," << ty(pts[i].y);
        }
        s << "\" stroke=\"" << stroke << "\" stroke-width=\"" << width << "\"";
        if (fill) s << " fill=\"" << fillColor << "\"";
        else s << " fill=\"none\"";
        s << "/>\n";
        return s.str();
    }

    std::string pathFromPoints(const std::vector<tether::blend::BlendVec>& pts,
                               const std::string& stroke = "#0066ff",
                               double width = 1.5) const {
        if (pts.empty()) return "";
        std::ostringstream s;
        s << "  <path d=\"M " << std::fixed << std::setprecision(2)
          << tx(pts[0].x) << " " << ty(pts[0].y);
        for (size_t i = 1; i < pts.size(); ++i) {
            s << " L " << tx(pts[i].x) << " " << ty(pts[i].y);
        }
        s << "\" stroke=\"" << stroke << "\" stroke-width=\"" << width
          << "\" fill=\"none\"/>\n";
        return s.str();
    }

    std::string labelPoint(double x, double y, const std::string& label,
                           const std::string& color = "#333",
                           int dx = 6, int dy = -6) const {
        std::ostringstream s;
        s << circle(x, y, 3, color);
        s << text(static_cast<int>(tx(x)) + dx, static_cast<int>(ty(y)) + dy,
                  label, color, 10);
        return s.str();
    }

    std::string infoPanel(const std::vector<std::string>& lines,
                          int x = 10, int y = 20) const {
        std::ostringstream s;
        int yPos = y;
        for (const auto& line : lines) {
            s << text(x, yPos, line, "#444", 11);
            yPos += 14;
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
    SVGBounds bounds_;
    double scale_ = 1.0;
    double offX_ = 0, offY_ = 0;
};

// ============================================================================
// Blend visualizer — generates full SVG for a blend test case
// ============================================================================

struct BlendVisualizationData {
    // Original path points (start -> corner -> end)
    tether::blend::BlendVec pathStart;
    tether::blend::BlendVec cornerPoint;
    tether::blend::BlendVec pathEnd;

    // Blend curve sampled points
    std::vector<tether::blend::BlendVec> blendPoints;

    // Control points (for quintic Bézier: 6 points)
    std::vector<tether::blend::BlendVec> controlPoints;

    // Blend entry/exit
    tether::blend::BlendVec blendEntry;
    tether::blend::BlendVec blendExit;

    // Curvature profile (t, curvature) pairs along blend
    std::vector<std::pair<double, double>> curvatureProfile;

    // Metadata
    std::string testName;
    std::string transitionType;  // "Line-Line", "Line-Arc", etc.
    double cornerAngleDeg = 0;
    double tolerance = 0;
    double blendRadius = 0;
    double entryDistance = 0;
    double exitDistance = 0;
    double maxCurvature = 0;
    bool canBlend = false;
    bool c1Continuous = false;
    bool c2Continuous = false;
};

inline std::string generateBlendSVG(const BlendVisualizationData& data,
                                    const std::string& outputPath) {
    SVGCanvas canvas(900, 700, 50);

    // Compute bounds from all points
    SVGBounds bounds;
    bounds.include(data.pathStart.x, data.pathStart.y);
    bounds.include(data.cornerPoint.x, data.cornerPoint.y);
    bounds.include(data.pathEnd.x, data.pathEnd.y);
    for (const auto& p : data.blendPoints) bounds.include(p.x, p.y);
    for (const auto& p : data.controlPoints) bounds.include(p.x, p.y);
    // Add tolerance margin
    double margin = std::max(data.tolerance * 2, 1.0);
    bounds.minX -= margin; bounds.minY -= margin;
    bounds.maxX += margin; bounds.maxY += margin;

    canvas.setBounds(bounds);

    std::ostringstream s;
    s << canvas.header(data.testName);

    // Grid
    s << canvas.grid(5.0);

    // Tolerance zone (shaded area around original path)
    if (data.tolerance > 0) {
        s << "  <!-- Tolerance zone -->\n";
        s << "  <g id=\"tolerance\" fill=\"#fff3e0\" fill-opacity=\"0.5\" stroke=\"none\">\n";
        // Approximate tolerance as offset around corner
        double tol = data.tolerance;
        s << "    <circle cx=\"" << canvas.tx(data.cornerPoint.x)
          << "\" cy=\"" << canvas.ty(data.cornerPoint.y)
          << "\" r=\"" << tol * canvas.scale() << "\" fill=\"#ffe0b2\" fill-opacity=\"0.3\"/>\n";
        s << "  </g>\n";
    }

    // Original path (dashed)
    s << "  <!-- Original path -->\n";
    s << "  <g id=\"original-path\">\n";
    s << canvas.line(data.pathStart.x, data.pathStart.y,
                     data.cornerPoint.x, data.cornerPoint.y,
                     "#999", 1.0, "4,3");
    s << canvas.line(data.cornerPoint.x, data.cornerPoint.y,
                     data.pathEnd.x, data.pathEnd.y,
                     "#999", 1.0, "4,3");
    s << "  </g>\n";

    // Trimmed original path (solid up to blend entry/exit)
    if (data.canBlend) {
        s << "  <!-- Trimmed path segments -->\n";
        s << canvas.line(data.pathStart.x, data.pathStart.y,
                         data.blendEntry.x, data.blendEntry.y,
                         "#666", 1.5);
        s << canvas.line(data.blendExit.x, data.blendExit.y,
                         data.pathEnd.x, data.pathEnd.y,
                         "#666", 1.5);
    }

    // Blend curve
    if (!data.blendPoints.empty()) {
        s << "  <!-- Blend curve -->\n";
        s << canvas.pathFromPoints(data.blendPoints, "#0066ff", 2.5);
    }

    // Control polygon
    if (data.controlPoints.size() >= 2) {
        s << "  <!-- Control polygon -->\n";
        s << "  <g id=\"control-polygon\" stroke=\"#cc9900\" stroke-width=\"0.8\" "
          << "stroke-dasharray=\"2,2\" fill=\"none\">\n";
        for (size_t i = 0; i < data.controlPoints.size() - 1; ++i) {
            s << canvas.line(data.controlPoints[i].x, data.controlPoints[i].y,
                             data.controlPoints[i+1].x, data.controlPoints[i+1].y,
                             "#cc9900", 0.8, "2,2");
        }
        s << "  </g>\n";

        // Control point dots
        for (size_t i = 0; i < data.controlPoints.size(); ++i) {
            s << canvas.circle(data.controlPoints[i].x, data.controlPoints[i].y, 3,
                               "#ff9900", "#996600", 0.5);
            s << canvas.text(
                static_cast<int>(canvas.tx(data.controlPoints[i].x)) + 4,
                static_cast<int>(canvas.ty(data.controlPoints[i].y)) + 4,
                "P" + std::to_string(i), "#996600", 9);
        }
    }

    // Key points
    s << canvas.labelPoint(data.pathStart.x, data.pathStart.y, "Start", "#006600");
    s << canvas.labelPoint(data.cornerPoint.x, data.cornerPoint.y, "Corner", "#cc0000");
    s << canvas.labelPoint(data.pathEnd.x, data.pathEnd.y, "End", "#006600");

    if (data.canBlend) {
        s << canvas.labelPoint(data.blendEntry.x, data.blendEntry.y, "Entry", "#0066cc");
        s << canvas.labelPoint(data.blendExit.x, data.blendExit.y, "Exit", "#0066cc");
    }

    // Info panel
    std::vector<std::string> info;
    info.push_back("Test: " + data.testName);
    info.push_back("Type: " + data.transitionType);
    info.push_back("Angle: " + std::to_string(data.cornerAngleDeg) + " deg");
    info.push_back("Tolerance: " + std::to_string(data.tolerance) + " mm");
    info.push_back("Blend radius: " + std::to_string(data.blendRadius) + " mm");
    info.push_back("Entry dist: " + std::to_string(data.entryDistance) + " mm");
    info.push_back("Exit dist: " + std::to_string(data.exitDistance) + " mm");
    info.push_back("Max curvature: " + std::to_string(data.maxCurvature) + " /mm");
    info.push_back("Can blend: " + std::string(data.canBlend ? "YES" : "NO"));
    info.push_back("C1: " + std::string(data.c1Continuous ? "PASS" : "FAIL"));
    info.push_back("C2: " + std::string(data.c2Continuous ? "PASS" : "FAIL"));
    s << canvas.infoPanel(info, 10, 20);

    // Curvature profile subplot (bottom-right)
    if (!data.curvatureProfile.empty()) {
        int subplotX = canvas.width() - 250;
        int subplotY = canvas.height() - 150;
        int subplotW = 230;
        int subplotH = 120;

        s << "  <!-- Curvature profile -->\n";
        s << "  <rect x=\"" << subplotX - 5 << "\" y=\"" << subplotY - 15
          << "\" width=\"" << subplotW + 10 << "\" height=\"" << subplotH + 25
          << "\" fill=\"white\" stroke=\"#ccc\" stroke-width=\"0.5\" rx=\"4\"/>\n";
        s << canvas.text(subplotX, subplotY - 2, "Curvature profile", "#666", 10);

        double maxK = 0;
        for (const auto& [t, k] : data.curvatureProfile)
            maxK = std::max(maxK, std::abs(k));
        if (maxK < 1e-12) maxK = 1.0;

        // Zero line
        s << "  <line x1=\"" << subplotX << "\" y1=\"" << subplotY + subplotH/2
          << "\" x2=\"" << subplotX + subplotW << "\" y2=\"" << subplotY + subplotH/2
          << "\" stroke=\"#ddd\" stroke-width=\"0.5\"/>\n";

        // Curvature curve
        s << "  <polyline points=\"";
        for (size_t i = 0; i < data.curvatureProfile.size(); ++i) {
            const auto& [t, k] = data.curvatureProfile[i];
            double px = subplotX + t * static_cast<double>(subplotW);
            double py = subplotY + static_cast<double>(subplotH) / 2.0 - (k / maxK) * (static_cast<double>(subplotH) / 2.0 - 5.0);
            if (i > 0) s << " ";
            s << std::fixed << std::setprecision(1) << px << "," << py;
        }
        s << "\" stroke=\"#0066cc\" stroke-width=\"1.2\" fill=\"none\"/>\n";
    }

    s << canvas.footer();

    // Write to file
    if (!outputPath.empty()) {
        std::filesystem::path dir = std::filesystem::path(outputPath).parent_path();
        if (!dir.empty() && !std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
        }
        std::ofstream file(outputPath);
        if (file.is_open()) {
            file << s.str();
            file.close();
        }
    }

    return s.str();
}

// ============================================================================
// Multi-blend visualizer — shows a full path with multiple blends
// ============================================================================

struct MultiBlendVisualizationData {
    std::string testName;
    std::vector<tether::blend::BlendVec> originalPath;  // All waypoints
    std::vector<tether::blend::BlendVec> blendedPath;   // All output points
    std::vector<tether::blend::BlendVec> blendRegions;  // Start/end of each blend
    std::vector<std::string> labels;
};

inline std::string generateMultiBlendSVG(const MultiBlendVisualizationData& data,
                                         const std::string& outputPath) {
    SVGCanvas canvas(1000, 700, 50);

    SVGBounds bounds;
    for (const auto& p : data.originalPath) bounds.include(p.x, p.y);
    for (const auto& p : data.blendedPath) bounds.include(p.x, p.y);
    bounds.minX -= 2; bounds.minY -= 2;
    bounds.maxX += 2; bounds.maxY += 2;

    canvas.setBounds(bounds);

    std::ostringstream s;
    s << canvas.header(data.testName);
    s << canvas.grid(5.0);

    // Original path (dashed)
    if (data.originalPath.size() >= 2) {
        s << "  <!-- Original path -->\n";
        for (size_t i = 0; i < data.originalPath.size() - 1; ++i) {
            s << canvas.line(data.originalPath[i].x, data.originalPath[i].y,
                             data.originalPath[i+1].x, data.originalPath[i+1].y,
                             "#ccc", 1.0, "4,3");
        }
    }

    // Blended path (solid)
    if (!data.blendedPath.empty()) {
        s << "  <!-- Blended path -->\n";
        s << canvas.pathFromPoints(data.blendedPath, "#0066ff", 2.0);
    }

    // Waypoint markers
    for (size_t i = 0; i < data.originalPath.size(); ++i) {
        std::string label = (i < data.labels.size()) ? data.labels[i] :
                            ("W" + std::to_string(i));
        s << canvas.labelPoint(data.originalPath[i].x, data.originalPath[i].y,
                               label, "#cc0000");
    }

    // Blend region markers
    for (size_t i = 0; i + 1 < data.blendRegions.size(); i += 2) {
        s << canvas.circle(data.blendRegions[i].x, data.blendRegions[i].y, 3, "#00cc00");
        s << canvas.circle(data.blendRegions[i+1].x, data.blendRegions[i+1].y, 3, "#00cc00");
    }

    s << canvas.infoPanel({data.testName,
                           "Waypoints: " + std::to_string(data.originalPath.size()),
                           "Blended points: " + std::to_string(data.blendedPath.size())},
                          10, 20);

    s << canvas.footer();

    if (!outputPath.empty()) {
        std::filesystem::path dir = std::filesystem::path(outputPath).parent_path();
        if (!dir.empty() && !std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
        }
        std::ofstream file(outputPath);
        if (file.is_open()) {
            file << s.str();
            file.close();
        }
    }

    return s.str();
}

} // namespace BlendTest
