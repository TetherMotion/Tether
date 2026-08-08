// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file KdeSvgRenderer.hpp
 * @brief KDE-specific SVG plot rendering extracted from SvgExporter
 *
 * @details
 * Encapsulates the KDE visualization sub-responsibility of SvgExporter:
 *  - KDE heatmap (density grid + colorbar + marginals + scatter overlay)
 *  - Conditional statistics plot (quantile bands + median/mean lines)
 *  - Marginal X / Y density plots
 *  - KDE dashboard (2x2 grid of the above)
 *  - Export helpers (file writers for individual plots + dashboard)
 *
 * The renderer holds a reference to a SvgCanvas for low-level drawing
 * and a const SvgConfig for layout/colors. It is stateless beyond those
 * references.
 */

#include "tether/motion_replanner/SvgCanvas.hpp"
#include "tether/motion_replanner/SvgExporter.hpp" // SvgConfig, AxisBounds

#include <string>
#include <vector>

namespace MotionReplanner {

class KdeSvgRenderer {
public:
    KdeSvgRenderer(const SvgConfig& config, const SvgCanvas& canvas);

    // --- KDE plot rendering ---

    void renderHeatmap(std::ostream& out,
                       int svgW, int svgH,
                       const tether::motion::replanner::KdeEvaluation& kde) const;

    void renderConditional(std::ostream& out,
                           int svgW, int svgH,
                           const tether::motion::replanner::KdeEvaluation& kde) const;

    void renderMarginalX(std::ostream& out,
                         int svgW, int svgH,
                         const tether::motion::replanner::KdeEvaluation& kde) const;

    void renderMarginalY(std::ostream& out,
                         int svgW, int svgH,
                         const tether::motion::replanner::KdeEvaluation& kde) const;

    // --- File export helpers ---

    bool exportHeatmap(const std::string& filename,
                       const tether::motion::replanner::KdeEvaluation& kde) const;

    bool exportConditional(const std::string& filename,
                           const tether::motion::replanner::KdeEvaluation& kde) const;

    bool exportDashboard(const std::string& filename,
                         const tether::motion::replanner::KdeEvaluation& kde) const;

    std::vector<std::string> exportAll(
        const std::string& outputDir,
        const std::string& filePrefix,
        const tether::motion::replanner::KdeEvaluation& kde) const;

    /// Convert a normalized density [0,1] to an SVG color string.
    std::string densityColor(double normalizedDensity) const;

private:
    const SvgConfig& config_;
    const SvgCanvas& canvas_;
};

} // namespace MotionReplanner
