// SPDX-License-Identifier: MIT

#include "tether/motion_replanner/SvgCanvas.hpp"
#include "tether/motion_replanner/SvgExporter.hpp"  // SvgConfig

#include <algorithm>
#include <cmath>
#include <format>
#include <iomanip>
#include <sstream>

namespace MotionReplanner {

//=============================================================================
// Number formatting
//=============================================================================

std::string SvgCanvas::fmt(double v) const {
    if (std::abs(v) < 1e-12) v = 0.0;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(config().precision > 0 ? config().precision : 4) << v;
    return ss.str();
}

//=============================================================================
// Bounds and transforms
//=============================================================================

SvgCanvas::AxisBounds SvgCanvas::computeBounds(
    const std::vector<std::pair<double,double>>& points) const {
    AxisBounds b;
    if (points.empty()) return b;
    b.minX = b.maxX = points[0].first;
    b.minY = b.maxY = points[0].second;
    for (const auto& [x, y] : points) {
        b.minX = std::min(b.minX, x);
        b.maxX = std::max(b.maxX, x);
        b.minY = std::min(b.minY, y);
        b.maxY = std::max(b.maxY, y);
    }
    // Add 5% padding
    double padX = (b.maxX - b.minX) * 0.05;
    double padY = (b.maxY - b.minY) * 0.05;
    if (padX < 1e-9) padX = 1.0;
    if (padY < 1e-9) padY = 1.0;
    b.minX -= padX; b.maxX += padX;
    b.minY -= padY; b.maxY += padY;
    return b;
}

SvgCanvas::AxisBounds SvgCanvas::computeBoundsMulti(
    const std::vector<std::vector<std::pair<double,double>>>& allPoints) const {
    AxisBounds b;
    bool first = true;
    for (const auto& pts : allPoints) {
        for (const auto& [x, y] : pts) {
            if (first) { b.minX = b.maxX = x; b.minY = b.maxY = y; first = false; }
            b.minX = std::min(b.minX, x);
            b.maxX = std::max(b.maxX, x);
            b.minY = std::min(b.minY, y);
            b.maxY = std::max(b.maxY, y);
        }
    }
    if (first) return b;
    double padX = (b.maxX - b.minX) * 0.05;
    double padY = (b.maxY - b.minY) * 0.05;
    if (padX < 1e-9) padX = 1.0;
    if (padY < 1e-9) padY = 1.0;
    b.minX -= padX; b.maxX += padX;
    b.minY -= padY; b.maxY += padY;
    return b;
}

std::pair<double,double> SvgCanvas::transform(
    double dataX, double dataY, const AxisBounds& bounds,
    double svgWidth, double svgHeight, double margin) const {

    double rangeX = bounds.maxX - bounds.minX;
    double rangeY = bounds.maxY - bounds.minY;
    if (rangeX < 1e-15) rangeX = 1.0;
    if (rangeY < 1e-15) rangeY = 1.0;

    // Maintain aspect ratio
    double plotW = svgWidth - 2.0 * margin;
    double plotH = svgHeight - 2.0 * margin;
    double scaleX = plotW / rangeX;
    double scaleY = plotH / rangeY;
    double scale = std::min(scaleX, scaleY);

    double px = margin + (dataX - bounds.minX) * scale;
    // SVG Y is inverted (top = 0)
    double py = svgHeight - margin - (dataY - bounds.minY) * scale;
    return {px, py};
}

//=============================================================================
// Low-level SVG element writers
//=============================================================================

void SvgCanvas::writeHeader(std::ostream& out, int width, int height) const {
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" height=\"" << height << "\" "
        << "viewBox=\"0 0 " << width << " " << height << "\">\n";
}

void SvgCanvas::writeFooter(std::ostream& out) const {
    out << "</svg>\n";
}

void SvgCanvas::writeBackground(std::ostream& out, int width, int height) const {
    out << "<rect x=\"0\" y=\"0\" width=\"" << width << "\" height=\"" << height
        << "\" fill=\"" << config().backgroundColor << "\"/>\n";
}

void SvgCanvas::writeRect(std::ostream& out, double x, double y,
                            double w, double h, const std::string& fill,
                            const std::string& stroke, double strokeWidth) const {
    out << "<rect x=\"" << fmt(x) << "\" y=\"" << fmt(y)
        << "\" width=\"" << fmt(w) << "\" height=\"" << fmt(h)
        << "\" fill=\"" << fill << "\"";
    if (stroke != "none") {
        out << " stroke=\"" << stroke << "\" stroke-width=\"" << fmt(strokeWidth) << "\"";
    }
    out << "/>\n";
}

void SvgCanvas::writeLine(std::ostream& out, double x1, double y1,
                            double x2, double y2,
                            const std::string& stroke, double width) const {
    out << "<line x1=\"" << fmt(x1) << "\" y1=\"" << fmt(y1)
        << "\" x2=\"" << fmt(x2) << "\" y2=\"" << fmt(y2)
        << "\" stroke=\"" << stroke << "\" stroke-width=\"" << fmt(width) << "\"/>\n";
}

void SvgCanvas::writeDashedLine(std::ostream& out, double x1, double y1,
                                  double x2, double y2,
                                  const std::string& stroke, double width) const {
    out << "<line x1=\"" << fmt(x1) << "\" y1=\"" << fmt(y1)
        << "\" x2=\"" << fmt(x2) << "\" y2=\"" << fmt(y2)
        << "\" stroke=\"" << stroke << "\" stroke-width=\"" << fmt(width)
        << "\" stroke-dasharray=\"4,4\"/>\n";
}

void SvgCanvas::writePolyline(std::ostream& out,
                                const std::vector<std::pair<double,double>>& points,
                                const std::string& stroke, double width,
                                bool fill, const std::string& fillColor) const {
    if (points.empty()) return;

    out << "<polyline points=\"";
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i > 0) out << " ";
        out << fmt(points[i].first) << "," << fmt(points[i].second);
    }
    out << "\" fill=\"" << (fill ? fillColor : "none")
        << "\" stroke=\"" << stroke
        << "\" stroke-width=\"" << fmt(width) << "\"";
    if (fill) {
        out << " fill-opacity=\"0.15\"";
    }
    out << "/>\n";
}

void SvgCanvas::writeText(std::ostream& out, double x, double y,
                            const std::string& text, int size,
                            const std::string& fill,
                            const std::string& anchor) const {
    out << "<text x=\"" << fmt(x) << "\" y=\"" << fmt(y)
        << "\" font-family=\"sans-serif\" font-size=\"" << size
        << "\" fill=\"" << fill << "\" text-anchor=\"" << anchor << "\">"
        << text << "</text>\n";
}

void SvgCanvas::writeCircle(std::ostream& out, double cx, double cy, double r,
                              const std::string& fill,
                              const std::string& stroke, double strokeWidth) const {
    out << "<circle cx=\"" << fmt(cx) << "\" cy=\"" << fmt(cy)
        << "\" r=\"" << fmt(r) << "\" fill=\"" << fill << "\"";
    if (stroke != "none") {
        out << " stroke=\"" << stroke << "\" stroke-width=\"" << fmt(strokeWidth) << "\"";
    }
    out << "/>\n";
}

//=============================================================================
// Axis, grid, legend, title
//=============================================================================

void SvgCanvas::renderGrid(std::ostream& out, const AxisBounds& bounds,
                              int svgW, int svgH, int margin) const {
    if (!config().includeGrid) return;

    int numGridLines = 8;
    for (int i = 0; i <= numGridLines; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(numGridLines);
        // Vertical grid lines
        double dataX = bounds.minX + t * (bounds.maxX - bounds.minX);
        auto [px1, py1] = transform(dataX, bounds.minY, bounds, svgW, svgH, margin);
        auto [px2, py2] = transform(dataX, bounds.maxY, bounds, svgW, svgH, margin);
        writeLine(out, px1, py1, px2, py2, config().gridColor, config().gridLineWidth);

        // Horizontal grid lines
        double dataY = bounds.minY + t * (bounds.maxY - bounds.minY);
        auto [px3, py3] = transform(bounds.minX, dataY, bounds, svgW, svgH, margin);
        auto [px4, py4] = transform(bounds.maxX, dataY, bounds, svgW, svgH, margin);
        writeLine(out, px3, py3, px4, py4, config().gridColor, config().gridLineWidth);
    }
}

void SvgCanvas::renderAxes(std::ostream& out, const AxisBounds& bounds,
                              int svgW, int svgH, int margin,
                              const std::string& xLabel, const std::string& yLabel) const {
    if (!config().includeAxes) return;

    // X axis (bottom)
    auto [bx1, by1] = transform(bounds.minX, bounds.minY, bounds, svgW, svgH, margin);
    auto [bx2, by2] = transform(bounds.maxX, bounds.minY, bounds, svgW, svgH, margin);
    writeLine(out, bx1, by1, bx2, by2, config().axisColor, config().axisLineWidth);

    // Y axis (left)
    auto [ay1, ax1] = transform(bounds.minX, bounds.minY, bounds, svgW, svgH, margin);
    auto [ay2, ax2] = transform(bounds.minX, bounds.maxY, bounds, svgW, svgH, margin);
    writeLine(out, ay1, ax1, ay2, ax2, config().axisColor, config().axisLineWidth);

    // Axis labels
    if (!xLabel.empty()) {
        writeText(out, svgW / 2.0, svgH - margin / 3.0, xLabel,
                  config().fontSize, config().textColor, "middle");
    }
    if (!yLabel.empty()) {
        // Rotated Y label
        out << "<text x=\"" << fmt(margin / 3.0) << "\" y=\""
            << fmt(svgH / 2.0) << "\" font-family=\"sans-serif\" font-size=\""
            << config().fontSize << "\" fill=\"" << config().textColor
            << "\" text-anchor=\"middle\" transform=\"rotate(-90 "
            << fmt(margin / 3.0) << " " << fmt(svgH / 2.0) << ")\">"
            << yLabel << "</text>\n";
    }

    // Tick labels (min/max)
    writeText(out, bx1, by1 + 20, std::format("{:.2f}", bounds.minX),
              config().fontSize - 2, config().textColor, "middle");
    writeText(out, bx2, by2 + 20, std::format("{:.2f}", bounds.maxX),
              config().fontSize - 2, config().textColor, "middle");
    writeText(out, ay1 - 10, ax1 + 5, std::format("{:.2f}", bounds.minY),
              config().fontSize - 2, config().textColor, "end");
    writeText(out, ay2 - 10, ax2 + 5, std::format("{:.2f}", bounds.maxY),
              config().fontSize - 2, config().textColor, "end");
}

void SvgCanvas::renderLegend(std::ostream& out, int svgW, int svgH,
                               const std::vector<std::pair<std::string, std::string>>& entries) const {
    if (!config().includeLegend || entries.empty()) return;

    double lx = svgW - 150;
    double ly = 20;
    double lh = static_cast<double>(entries.size()) * 20.0 + 10.0;

    out << "<rect x=\"" << fmt(lx) << "\" y=\"" << fmt(ly)
        << "\" width=\"130\" height=\"" << fmt(lh)
        << "\" fill=\"" << config().backgroundColor
        << "\" stroke=\"" << config().axisColor
        << "\" stroke-width=\"0.5\" fill-opacity=\"0.8\"/>\n";

    for (std::size_t i = 0; i < entries.size(); ++i) {
        double ey = ly + 15.0 + static_cast<double>(i) * 20.0;
        writeLine(out, lx + 10, ey, lx + 30, ey, entries[i].second, config().lineWidth);
        writeText(out, lx + 35, ey + 4, entries[i].first,
                  config().fontSize - 2, config().textColor, "start");
    }
}

void SvgCanvas::renderTitle(std::ostream& out, const std::string& title,
                               int svgW, int margin) const {
    if (!config().includeTitle || title.empty()) return;
    writeText(out, svgW / 2.0, margin / 2.0, title,
              config().titleFontSize, config().textColor, "middle");
}

} // namespace MotionReplanner
