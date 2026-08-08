// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file SvgCanvas.hpp
 * @brief Low-level SVG element writers extracted from SvgExporter
 *
 * @details
 * Encapsulates the SVG primitive rendering sub-responsibility of SvgExporter:
 *  - XML header/footer generation
 *  - Background fill
 *  - Primitive shape writers (rect, line, dashed line, polyline, text, circle)
 *  - Number formatting helper (fmt)
 *  - Coordinate transforms (data → SVG pixel space)
 *  - Axis/grid/legend/title rendering
 *
 * The canvas holds a const reference to the SvgConfig for colors, fonts, and
 * display options. It is stateless beyond that reference.
 */

#include <cstddef>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace MotionReplanner {

// Forward declaration — defined in SvgExporter.hpp
struct SvgConfig;

class SvgCanvas {
public:
    explicit SvgCanvas(const SvgConfig& config) : config_(&config) {}
    void updateConfig(const SvgConfig& config) { config_ = &config; }

    //--- Axis bounds (shared with SvgExporter) ---

    struct AxisBounds {
        double minX = 0, maxX = 1, minY = 0, maxY = 1;
    };

    //--- Number formatting ---

    std::string fmt(double v) const;

    //--- Low-level SVG element writers ---

    void writeHeader(std::ostream& out, int width, int height) const;
    void writeFooter(std::ostream& out) const;
    void writeBackground(std::ostream& out, int width, int height) const;
    void writeRect(std::ostream& out, double x, double y, double w, double h,
                   const std::string& fill,
                   const std::string& stroke = "none",
                   double strokeWidth = 0) const;
    void writeLine(std::ostream& out, double x1, double y1, double x2, double y2,
                   const std::string& stroke, double width) const;
    void writePolyline(std::ostream& out,
                       const std::vector<std::pair<double,double>>& points,
                       const std::string& stroke, double width,
                       bool fill = false,
                       const std::string& fillColor = "none") const;
    void writeText(std::ostream& out, double x, double y,
                   const std::string& text, int size,
                   const std::string& fill,
                   const std::string& anchor = "start") const;
    void writeCircle(std::ostream& out, double cx, double cy, double r,
                     const std::string& fill,
                     const std::string& stroke = "none",
                     double strokeWidth = 0) const;
    void writeDashedLine(std::ostream& out, double x1, double y1,
                         double x2, double y2,
                         const std::string& stroke, double width) const;

    //--- Coordinate transforms ---

    /// Transform data coordinates to SVG pixel coordinates.
    std::pair<double,double> transform(
        double dataX, double dataY, const AxisBounds& bounds,
        double svgWidth, double svgHeight, double margin) const;

    //--- Axis and grid rendering ---

    void renderAxes(std::ostream& out, const AxisBounds& bounds,
                    int svgW, int svgH, int margin,
                    const std::string& xLabel, const std::string& yLabel) const;
    void renderGrid(std::ostream& out, const AxisBounds& bounds,
                    int svgW, int svgH, int margin) const;
    void renderLegend(std::ostream& out, int svgW, int svgH,
                      const std::vector<std::pair<std::string, std::string>>& entries) const;
    void renderTitle(std::ostream& out, const std::string& title,
                     int svgW, int margin) const;

    //--- Bounds computation ---

    AxisBounds computeBounds(
        const std::vector<std::pair<double,double>>& points) const;
    AxisBounds computeBoundsMulti(
        const std::vector<std::vector<std::pair<double,double>>>& allPoints) const;

    const SvgConfig& config() const { return *config_; }

protected:
    const SvgConfig* config_;
};

} // namespace MotionReplanner
