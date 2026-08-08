/**
 * @file SvgExporter.hpp
 * @brief SVG vector graphics export for trajectory comparison and evaluation.
 *
 * @details
 * Generates standalone .svg files for visualizing desired vs actual path
 * comparisons, error profiles, spectral analysis, and more. Pure C++ SVG
 * generation with no external dependencies — SVG is XML-based and can be
 * generated with string formatting.
 *
 * ## Plot types
 *
 * - **Trajectory projections**: XY, XZ, YZ (2D), and 3D isometric views
 *   showing desired vs actual paths with deviation envelopes.
 * - **Error profiles**: contour/lag/combined error vs arc length or time.
 * - **Error histogram**: distribution of contour error with percentile
 *   markers.
 * - **Error envelope**: magnified deviation band around the desired path.
 * - **Spectral plots**: FFT magnitude and phase vs frequency, with peak
 *   markers and path-geometry frequency lines.
 * - **Kinematic profiles**: velocity and acceleration vs time.
 * - **Phase portrait**: position vs velocity phase space.
 * - **Dashboard**: all plots combined in a single SVG file.
 *
 * ## Projection features
 *
 * - **Deviation envelope**: draws a magnified error band around the
 *   desired path, making small deviations visible.
 * - **Closest-point lines**: optionally draws lines from each actual
 *   point to its closest point on the desired path.
 * - **Multi-plane**: XY, XZ, YZ projections each as separate SVGs with
 *   independent axis bounds.
 * - **3D isometric**: simple isometric projection (30° azimuth, 30°
 *   elevation) for a quick 3D preview.
 *
 * @see PathEvaluator.hpp for the evaluation results.
 * @see PathRelativeFFT.hpp for the spectral results.
 */

#pragma once

#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/motion_replanner/PathEvaluator.hpp"
#include "tether/motion_replanner/PathRelativeFFT.hpp"
#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"
#include "tether/motion_replanner/SvgCanvas.hpp"

#include <string>
#include <vector>
#include <ostream>
#include <cstddef>

namespace MotionReplanner {

//=============================================================================
// Configuration
//=============================================================================

/// SVG export configuration.
struct SvgConfig {
    //--- Dimensions ---
    int width = 1200;
    int height = 800;
    int margin = 60;  ///< Margin around plot area (pixels)

    //--- Display options ---
    bool includeAxes = true;
    bool includeGrid = true;
    bool includeLegend = true;
    bool includeTitle = true;
    int fontSize = 12;
    int titleFontSize = 16;

    //--- Colors ---
    std::string desiredColor = "#0066CC";   ///< Blue
    std::string actualColor = "#CC0000";    ///< Red
    std::string errorColor = "#FF6600";     ///< Orange
    std::string spectralColor = "#006633";  ///< Green
    std::string backgroundColor = "#FFFFFF";
    std::string gridColor = "#E0E0E0";
    std::string axisColor = "#333333";
    std::string textColor = "#333333";
    std::string peakColor = "#9933CC";      ///< Purple for peak markers
    std::string geometryColor = "#666666";  ///< Gray for geometry lines

    //--- Line widths ---
    double lineWidth = 1.5;
    double gridLineWidth = 0.5;
    double axisLineWidth = 1.0;

    //--- Numeric formatting ---
    int precision = 4;  ///< Decimal places for SVG coordinate output

    //--- Projection features ---
    /// Draw a magnified error band around the desired path.
    bool showDeviationEnvelope = true;
    /// Visual magnification factor for deviations (10× = 10µm shows as 100µm).
    double envelopeScale = 10.0;
    /// Draw lines from actual points to closest path points.
    bool showClosestPointLines = false;
    /// Downsample factor for polyline rendering (1 = all points).
    /// Higher values reduce SVG file size for large trajectories.
    std::size_t renderDownsample = 1;

    //--- Spectral plot options ---
    /// Use log scale for magnitude axis.
    bool spectralLogScale = false;
    /// Show path-geometry frequency lines on spectral plots.
    bool showGeometryLines = true;

    //--- KDE heatmap options ---
    /// Colormap for KDE heatmaps.
    tether::motion::replanner::KdeColormap kdeColormap =
        tether::motion::replanner::KdeColormap::Viridis;
    /// Use log scale for KDE density.
    bool kdeLogScale = false;
    /// Show marginal distributions on KDE heatmap edges.
    bool kdeShowMarginals = true;
    /// Show conditional mean line on KDE heatmap.
    bool kdeShowConditionalMean = true;
    /// Show quantile contour lines (5%, 25%, 50%, 75%, 95%).
    bool kdeShowContours = true;
    /// Show scatter points overlaid on KDE heatmap.
    bool kdeShowScatter = false;
    /// Opacity of scatter points (0-1).
    double kdeScatterAlpha = 0.3;
};

//=============================================================================
// Plot types
//=============================================================================

/// Available SVG plot types.
enum class SvgPlotType {
    TrajectoryXY,         ///< 2D XY plane projection
    TrajectoryXZ,         ///< 2D XZ plane projection
    TrajectoryYZ,         ///< 2D YZ plane projection
    Trajectory3D,         ///< Isometric 3D projection
    ErrorVsPathLength,    ///< Error components vs arc length
    ErrorVsTime,          ///< Error components vs time
    ErrorHistogram,       ///< Distribution of contour error
    ErrorEnvelope,        ///< Deviation envelope around path (zoomed)
    SpectralMagnitude,    ///< FFT magnitude vs frequency
    SpectralPhase,        ///< FFT phase vs frequency
    SpectralWaterfall,    ///< All components stacked
    PhasePortrait,        ///< Position vs velocity phase space
    VelocityProfile,      ///< Desired vs actual velocity vs time
    AccelerationProfile,  ///< Desired vs actual acceleration vs time
    KdeHeatmap,           ///< KDE derivative-vs-deviation heatmap
    KdeConditional,       ///< Conditional stats (mean, quantiles) vs derivative
    KdeDashboard,         ///< KDE heatmap + marginals + conditional stats
};

//=============================================================================
// SvgExporter class
//=============================================================================

/// Generates SVG files for trajectory comparison and evaluation visualization.
///
/// Usage:
/// ```cpp
/// SvgExporter exporter;
/// exporter.exportPlot("trajectory_xy.svg", SvgPlotType::TrajectoryXY,
///                      desired, actual);
///
/// // Or export all plots at once:
/// exporter.exportAllPlots("output/", "test1", desired, actual, quant, spectral);
///
/// // Or a single dashboard:
/// exporter.exportDashboard("dashboard.svg", desired, actual, quant, spectral);
/// ```
class SvgExporter {
public:
    explicit SvgExporter(SvgConfig config = {});

    //--- Single plot export ---

    /// Export a single plot to a file.
    ///
    /// @param filename Output .svg filename.
    /// @param type Which plot type to generate.
    /// @param desired Desired trajectory samples.
    /// @param actual Actual trajectory samples.
    /// @param quant Optional quantitative evaluation (needed for histogram).
    /// @param spectral Optional spectral evaluation (needed for spectral plots).
    /// @return True if file was written successfully.
    bool exportPlot(const std::string& filename, SvgPlotType type,
                    const std::vector<GCodeExport::TrajectorySample>& desired,
                    const std::vector<GCodeExport::TrajectorySample>& actual,
                    const tether::motion::replanner::QuantitativeEvaluation* quant = nullptr,
                    const tether::motion::replanner::SpectralEvaluation* spectral = nullptr) const;

    //--- Batch export ---

    /// Export all standard plots to a directory.
    ///
    /// @param outputDir Directory to write files to (must exist).
    /// @param filePrefix Prefix for filenames (e.g., "test1" → "test1_trajectory_xy.svg").
    /// @param desired Desired trajectory samples.
    /// @param actual Actual trajectory samples.
    /// @param quant Quantitative evaluation results.
    /// @param spectral Spectral evaluation results.
    /// @return List of generated file paths.
    std::vector<std::string> exportAllPlots(
        const std::string& outputDir,
        const std::string& filePrefix,
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual,
        const tether::motion::replanner::QuantitativeEvaluation& quant,
        const tether::motion::replanner::SpectralEvaluation& spectral) const;

    //--- Dashboard ---

    /// Export a combined SVG with multiple sub-plots in a single file.
    ///
    /// Includes: trajectory XY/XZ/YZ/3D, error vs path length, error
    /// histogram, spectral magnitude, velocity profile.
    ///
    /// @param filename Output .svg filename.
    /// @param desired Desired trajectory samples.
    /// @param actual Actual trajectory samples.
    /// @param quant Quantitative evaluation results.
    /// @param spectral Spectral evaluation results.
    /// @return True if file was written successfully.
    bool exportDashboard(const std::string& filename,
                         const std::vector<GCodeExport::TrajectorySample>& desired,
                         const std::vector<GCodeExport::TrajectorySample>& actual,
                         const tether::motion::replanner::QuantitativeEvaluation& quant,
                         const tether::motion::replanner::SpectralEvaluation& spectral) const;

    //--- KDE export ---

    /// Export a KDE heatmap to a file.
    ///
    /// @param filename Output .svg filename.
    /// @param kde The KDE evaluation to visualize.
    /// @return True if file was written successfully.
    bool exportKdeHeatmap(const std::string& filename,
                          const tether::motion::replanner::KdeEvaluation& kde) const;

    /// Export conditional statistics (mean, quantiles) vs derivative.
    ///
    /// @param filename Output .svg filename.
    /// @param kde The KDE evaluation.
    /// @return True if file was written successfully.
    bool exportKdeConditional(const std::string& filename,
                              const tether::motion::replanner::KdeEvaluation& kde) const;

    /// Export a KDE dashboard (heatmap + marginals + conditional stats).
    ///
    /// @param filename Output .svg filename.
    /// @param kde The KDE evaluation.
    /// @return True if file was written successfully.
    bool exportKdeDashboard(const std::string& filename,
                            const tether::motion::replanner::KdeEvaluation& kde) const;

    /// Export all KDE plots to a directory.
    ///
    /// @param outputDir Directory to write files to (must exist).
    /// @param filePrefix Prefix for filenames.
    /// @param kde The KDE evaluation.
    /// @return List of generated file paths.
    std::vector<std::string> exportAllKdePlots(
        const std::string& outputDir,
        const std::string& filePrefix,
        const tether::motion::replanner::KdeEvaluation& kde) const;

    const SvgConfig& config() const { return config_; }
    void setConfig(const SvgConfig& config) { config_ = config; canvas_.updateConfig(config_); }

private:
    SvgConfig config_;
    SvgCanvas canvas_;

    //--- Low-level SVG element writers (delegated to canvas_) ---

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

    //--- Coordinate transforms (delegated to canvas_) ---

    using AxisBounds = SvgCanvas::AxisBounds;

    AxisBounds computeBounds(
        const std::vector<std::pair<double,double>>& points) const;
    AxisBounds computeBoundsMulti(
        const std::vector<std::vector<std::pair<double,double>>>& allPoints) const;

    /// Transform data coordinates to SVG pixel coordinates.
    std::pair<double,double> transform(
        double dataX, double dataY, const AxisBounds& bounds,
        double svgWidth, double svgHeight, double margin) const;

    //--- Axis and grid rendering (delegated to canvas_) ---

    void renderAxes(std::ostream& out, const AxisBounds& bounds,
                    int svgW, int svgH, int margin,
                    const std::string& xLabel, const std::string& yLabel) const;
    void renderGrid(std::ostream& out, const AxisBounds& bounds,
                    int svgW, int svgH, int margin) const;
    void renderLegend(std::ostream& out, int svgW, int svgH,
                      const std::vector<std::pair<std::string, std::string>>& entries) const;
    void renderTitle(std::ostream& out, const std::string& title,
                     int svgW, int margin) const;

    //--- Individual plot renderers ---

    void renderTrajectory2D(std::ostream& out, int plane,
                            int svgW, int svgH,
                            const std::vector<GCodeExport::TrajectorySample>& desired,
                            const std::vector<GCodeExport::TrajectorySample>& actual) const;
    void renderTrajectory3D(std::ostream& out,
                            int svgW, int svgH,
                            const std::vector<GCodeExport::TrajectorySample>& desired,
                            const std::vector<GCodeExport::TrajectorySample>& actual) const;
    void renderErrorProfile(std::ostream& out, bool vsPathLength,
                             int svgW, int svgH,
                             const std::vector<GCodeExport::TrajectorySample>& desired,
                             const std::vector<GCodeExport::TrajectorySample>& actual) const;
    void renderErrorHistogram(std::ostream& out,
                              int svgW, int svgH,
                              const tether::motion::replanner::QuantitativeEvaluation& quant) const;
    void renderErrorEnvelope(std::ostream& out, int plane,
                              int svgW, int svgH,
                              const std::vector<GCodeExport::TrajectorySample>& desired,
                              const std::vector<GCodeExport::TrajectorySample>& actual) const;
    void renderSpectralPlot(std::ostream& out, bool magnitude,
                             int svgW, int svgH,
                             const tether::motion::replanner::SpectralEvaluation& spectral,
                             tether::motion::replanner::SpectralComponent component,
                             tether::motion::replanner::SpectralDomain domain) const;
    void renderVelocityProfile(std::ostream& out,
                               int svgW, int svgH,
                               const std::vector<GCodeExport::TrajectorySample>& desired,
                               const std::vector<GCodeExport::TrajectorySample>& actual) const;
    void renderAccelerationProfile(std::ostream& out,
                                   int svgW, int svgH,
                                   const std::vector<GCodeExport::TrajectorySample>& desired,
                                   const std::vector<GCodeExport::TrajectorySample>& actual) const;
    void renderPhasePortrait(std::ostream& out,
                             int svgW, int svgH,
                             const std::vector<GCodeExport::TrajectorySample>& actual) const;

    //--- KDE renderers ---

    void renderKdeHeatmap(std::ostream& out,
                          int svgW, int svgH,
                          const tether::motion::replanner::KdeEvaluation& kde) const;
    void renderKdeConditional(std::ostream& out,
                              int svgW, int svgH,
                              const tether::motion::replanner::KdeEvaluation& kde) const;
    void renderKdeMarginalX(std::ostream& out,
                            int svgW, int svgH,
                            const tether::motion::replanner::KdeEvaluation& kde) const;
    void renderKdeMarginalY(std::ostream& out,
                            int svgW, int svgH,
                            const tether::motion::replanner::KdeEvaluation& kde) const;

    /// Convert a density value to a colormap color string for SVG.
    std::string densityColor(double normalizedDensity) const;

    //--- Helpers ---

    /// Extract 2D points from trajectory for a given plane (0=XY, 1=XZ, 2=YZ).
    std::vector<std::pair<double,double>> extractPoints2D(
        const std::vector<GCodeExport::TrajectorySample>& samples,
        int plane) const;

    /// 3D isometric projection: (x,y,z) → (2D screen x, y).
    std::pair<double,double> project3D(double x, double y, double z) const;

    /// Format a double for SVG output.
    std::string fmt(double v) const;
};

} // namespace MotionReplanner
