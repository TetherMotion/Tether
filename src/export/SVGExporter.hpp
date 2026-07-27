/**
 * @file SVGExporter.hpp
 * @brief SVG export for G-code toolpaths
 */

#pragma once

#include "TrajectoryAnalyzer.hpp"
#include <string>
#include <ostream>
#include <vector>
#include <tether/motion_planner/geometry/NurbsCurve.hpp>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>

namespace GCodeExport {

/**
 * @brief Configuration for SVG export
 */
struct SVGConfig {
    // Canvas settings
    double width = 800.0;                       ///< SVG width (pixels)
    double height = 600.0;                      ///< SVG height (pixels)
    double margin = 50.0;                       ///< Margin around content
    bool autoScale = true;                      ///< Auto-fit to canvas
    
    // View settings
    int primaryAxis1 = 0;                       ///< First axis for 2D view (0=X, 1=Y, 2=Z)
    int primaryAxis2 = 1;                       ///< Second axis for 2D view
    bool flipY = true;                          ///< Flip Y axis (SVG origin is top-left)
    
    // Styling
    std::string rapidColor = "#ff6600";         ///< Rapid move color
    std::string linearColor = "#0066ff";        ///< Linear move color
    std::string arcCWColor = "#00cc00";         ///< CW arc color
    std::string arcCCWColor = "#cc00cc";        ///< CCW arc color
    double rapidStrokeWidth = 0.5;
    double feedStrokeWidth = 1.0;
    std::string rapidDash = "2,2";              ///< Dash pattern for rapids
    
    // Options
    bool showRapids = true;
    bool showStartPoint = true;
    bool showEndPoint = true;
    bool showDirectionArrows = false;
    bool showSegmentNumbers = false;
    bool showGrid = true;
    double gridSpacing = 10.0;                  ///< Grid line spacing (mm)
    std::string gridColor = "#dddddd";
    
    // Color coding
    bool colorByVelocity = false;               ///< Color code by velocity
    bool colorByAcceleration = false;           ///< Color code by acceleration
    double velocityColorMin = 0.0;
    double velocityColorMax = 100.0;            ///< mm/s
};

struct RenderableBezierPath {
    std::vector<tether::motion::NurbsCurve> path;
    std::string color;
    double width;
};

/**
 * @brief Exports G-code paths to SVG format
 */
class SVGExporter {
public:
    explicit SVGExporter(const SVGConfig& config = {});
    
    /**
     * @brief Export trajectory samples to SVG
     * @param samples Trajectory samples from analyzer
     * @param filename Output filename
     * @return true on success
     */
    bool exportToFile(const std::vector<TrajectorySample>& samples, const std::string& filename);
    
    /**
     * @brief Export trajectory samples to stream
     */
    void exportToStream(const std::vector<TrajectorySample>& samples, std::ostream& out);
    
    /**
     * @brief Export from motion segments directly
     */
    bool exportSegments(const std::vector<GCode::PlanningSegment>& segments, const std::string& filename);

    void configure(const SVGConfig& config) { config_ = config; }

    /**
     * @brief Export Bezier paths to SVG (Direct <path> elements)
     */
    bool exportBezierPaths(const std::vector<struct RenderableBezierPath>& paths, const std::string& filename);

    /**
     * @brief Export a PiecewiseNurbsPath to SVG by Bézier decomposition
     */
    bool exportNURBSPath(const tether::motion::PiecewiseNurbsPath& nurbsPath,
                         const std::string& filename,
                         const std::string& color = "#0066ff",
                         double width = 1.0,
                         double maxApproxError = 0.01);
    
private:
    SVGConfig config_;
    
    struct Bounds {
        double minX, maxX, minY, maxY;
    };
    
    Bounds computeBounds(const std::vector<TrajectorySample>& samples);
    
    void writeHeader(std::ostream& out, const Bounds& bounds);
    void writeGrid(std::ostream& out, const Bounds& bounds, double scale, double offsetX, double offsetY);
    void writePath(std::ostream& out, const std::vector<TrajectorySample>& samples, 
                   double scale, double offsetX, double offsetY);
    void writeMarkers(std::ostream& out, const std::vector<TrajectorySample>& samples,
                      double scale, double offsetX, double offsetY);
    void writeFooter(std::ostream& out);
    
    std::string velocityToColor(double velocity);
    std::string escapeXml(const std::string& str);
    
    double transformX(double x, double scale, double offset) const;
    double transformY(double y, double scale, double offset) const;
};

} // namespace GCodeExport
