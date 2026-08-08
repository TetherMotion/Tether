// SPDX-License-Identifier: MIT

/**
 * @file KdeSvgRenderer.cpp
 * @brief KDE-specific SVG plot rendering implementation
 */

#include "tether/motion_replanner/KdeSvgRenderer.hpp"
#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <sstream>

namespace MotionReplanner {

using namespace tether::motion::replanner;

KdeSvgRenderer::KdeSvgRenderer(const SvgConfig& config, const SvgCanvas& canvas)
    : config_(config), canvas_(canvas) {}

//=============================================================================
// Color helper
//=============================================================================

std::string KdeSvgRenderer::densityColor(double normalizedDensity) const {
    auto [r, g, b] = KdeDerivativeAnalyzer::colormapColor(config_.kdeColormap, normalizedDensity);
    std::ostringstream ss;
    ss << "rgb(" << r << "," << g << "," << b << ")";
    return ss.str();
}

//=============================================================================
// KDE Heatmap
//=============================================================================

void KdeSvgRenderer::renderHeatmap(std::ostream& out,
                                   int svgW, int svgH,
                                   const KdeEvaluation& kde) const {
    if (kde.grid.density.empty() || kde.grid.xBins.empty() || kde.grid.yBins.empty()) {
        canvas_.renderTitle(out, "KDE Heatmap (no data)", svgW, config_.margin);
        return;
    }

    const auto& grid = kde.grid;
    auto nX = grid.xBins.size();
    auto nY = grid.yBins.size();

    // Compute density range
    double maxD = grid.maxDensity();
    if (maxD < 1e-15) maxD = 1.0;
    double minD = 0.0;
    if (config_.kdeLogScale) {
        // Log scale: map log(density) to [0, 1]
        minD = maxD * 1e-6;  // Floor for log scale
    }

    // Layout: main heatmap area + optional marginal strips
    int margin = config_.margin;
    int marginalSize = config_.kdeShowMarginals ? 60 : 0;
    int plotW = svgW - 2 * margin - marginalSize;
    int plotH = svgH - 2 * margin - marginalSize;
    int plotX = margin;
    int plotY = margin;

    // Background
    canvas_.writeBackground(out, svgW, svgH);

    // Title
    std::string title = std::format("KDE: {} vs {}",
                                    toString(kde.derivativeAxis),
                                    toString(kde.deviationAxis));
    canvas_.renderTitle(out, title, svgW, margin);

    // Render heatmap as a grid of small rectangles
    double dx = static_cast<double>(plotW) / static_cast<double>(nX);
    double dy = static_cast<double>(plotH) / static_cast<double>(nY);

    for (std::size_t iy = 0; iy < nY; ++iy) {
        for (std::size_t ix = 0; ix < nX; ++ix) {
            double d = grid.at(ix, iy);
            double normD;
            if (config_.kdeLogScale) {
                normD = (d > minD) ? std::log(d / minD) / std::log(maxD / minD) : 0.0;
            } else {
                normD = d / maxD;
            }
            normD = std::clamp(normD, 0.0, 1.0);

            // SVG Y is flipped (top = high Y)
            double px = plotX + static_cast<double>(ix) * dx;
            double py = plotY + static_cast<double>(nY - 1 - iy) * dy;
            std::string color = densityColor(normD);
            canvas_.writeRect(out, px, py, dx + 0.5, dy + 0.5, color);
        }
    }

    // Axes
    SvgCanvas::AxisBounds bounds;
    bounds.minX = grid.xBins.front();
    bounds.maxX = grid.xBins.back();
    bounds.minY = grid.yBins.front();
    bounds.maxY = grid.yBins.back();

    // Draw axes around the heatmap
    // X axis (bottom)
    canvas_.writeLine(out, plotX, plotY + plotH, plotX + plotW, plotY + plotH,
              config_.axisColor, config_.axisLineWidth);
    // Y axis (left)
    canvas_.writeLine(out, plotX, plotY, plotX, plotY + plotH,
              config_.axisColor, config_.axisLineWidth);

    // Axis labels
    std::string xLabel = std::format("{} ({})", toString(kde.derivativeAxis),
                                      unitString(kde.derivativeAxis));
    std::string yLabel = std::format("{} ({})", toString(kde.deviationAxis),
                                      unitString(kde.deviationAxis));
    canvas_.writeText(out, plotX + plotW / 2.0, plotY + plotH + 35, xLabel,
              config_.fontSize, config_.textColor, "middle");
    // Y label rotated
    out << "<text x=\"" << canvas_.fmt(plotX - 40) << "\" y=\""
        << canvas_.fmt(plotY + plotH / 2.0) << "\""
        << " font-family=\"sans-serif\" font-size=\"" << config_.fontSize << "\""
        << " fill=\"" << config_.textColor << "\""
        << " text-anchor=\"middle\""
        << " transform=\"rotate(-90 " << canvas_.fmt(plotX - 40) << " "
        << canvas_.fmt(plotY + plotH / 2.0) << ")\""
        << ">" << yLabel << "</text>\n";

    // Tick labels (X)
    int numXTicks = 5;
    for (int i = 0; i <= numXTicks; ++i) {
        double val = bounds.minX + (bounds.maxX - bounds.minX) * static_cast<double>(i) / numXTicks;
        double px = plotX + static_cast<double>(i) * plotW / numXTicks;
        canvas_.writeLine(out, px, plotY + plotH, px, plotY + plotH + 5,
                  config_.axisColor, config_.axisLineWidth);
        canvas_.writeText(out, px, plotY + plotH + 18, std::format("{:.2g}", val),
                  config_.fontSize - 2, config_.textColor, "middle");
    }
    // Tick labels (Y)
    int numYTicks = 5;
    for (int i = 0; i <= numYTicks; ++i) {
        double val = bounds.minY + (bounds.maxY - bounds.minY) * static_cast<double>(i) / numYTicks;
        double py = plotY + plotH - static_cast<double>(i) * plotH / numYTicks;
        canvas_.writeLine(out, plotX - 5, py, plotX, py, config_.axisColor, config_.axisLineWidth);
        canvas_.writeText(out, plotX - 8, py + 4, std::format("{:.2g}", val),
                  config_.fontSize - 2, config_.textColor, "end");
    }

    // Conditional mean line
    if (config_.kdeShowConditionalMean) {
        std::vector<std::pair<double, double>> meanLine;
        for (const auto& cs : kde.conditional) {
            if (cs.valid) {
                meanLine.emplace_back(cs.xValue, cs.meanY);
            }
        }
        if (meanLine.size() > 1) {
            // Transform to SVG coords
            std::vector<std::pair<double, double>> svgPts;
            for (const auto& [dx, dy] : meanLine) {
                double px = plotX + (dx - bounds.minX) / (bounds.maxX - bounds.minX) * plotW;
                double py = plotY + plotH - (dy - bounds.minY) / (bounds.maxY - bounds.minY) * plotH;
                svgPts.emplace_back(px, py);
            }
            canvas_.writePolyline(out, svgPts, "#FFFFFF", 2.0);
        }
    }

    // Scatter overlay
    if (config_.kdeShowScatter && !kde.derivatives.empty()) {
        double alpha = config_.kdeScatterAlpha;
        for (std::size_t i = 0; i < kde.derivatives.size(); ++i) {
            double dx = kde.derivatives[i];
            double dy = kde.deviations[i];
            if (dx < bounds.minX || dx > bounds.maxX) continue;
            if (dy < bounds.minY || dy > bounds.maxY) continue;
            double px = plotX + (dx - bounds.minX) / (bounds.maxX - bounds.minX) * plotW;
            double py = plotY + plotH - (dy - bounds.minY) / (bounds.maxY - bounds.minY) * plotH;
            out << "<circle cx=\"" << canvas_.fmt(px) << "\" cy=\"" << canvas_.fmt(py)
                << "\" r=\"1.5\" fill=\"black\" fill-opacity=\"" << canvas_.fmt(alpha) << "\"/>\n";
        }
    }

    // Colorbar (right side)
    int cbX = plotX + plotW + 20;
    int cbW = 15;
    int cbH = plotH;
    for (int i = 0; i < cbH; ++i) {
        double t = 1.0 - static_cast<double>(i) / static_cast<double>(cbH - 1);
        std::string color = densityColor(t);
        canvas_.writeRect(out, cbX, plotY + i, cbW, 1.0 + 0.5, color);
    }
    canvas_.writeLine(out, cbX, plotY, cbX, plotY + cbH, config_.axisColor, config_.axisLineWidth);
    canvas_.writeLine(out, cbX + cbW, plotY, cbX + cbW, plotY + cbH, config_.axisColor, config_.axisLineWidth);

    // Colorbar labels
    for (int i = 0; i <= 4; ++i) {
        double t = static_cast<double>(i) / 4.0;
        double val = config_.kdeLogScale ? minD * std::pow(maxD / minD, t) : maxD * t;
        double py = plotY + cbH - t * cbH;
        canvas_.writeText(out, cbX + cbW + 5, py + 4, std::format("{:.2g}", val),
                  config_.fontSize - 2, config_.textColor, "start");
    }

    // Marginal distributions
    if (config_.kdeShowMarginals) {
        // X marginal (top strip)
        auto margX = grid.marginalX();
        double maxMargX = *std::max_element(margX.begin(), margX.end());
        if (maxMargX < 1e-15) maxMargX = 1.0;
        int margY = plotY - marginalSize + 5;
        std::vector<std::pair<double, double>> margXPts;
        for (std::size_t ix = 0; ix < nX; ++ix) {
            double px = plotX + static_cast<double>(ix) * dx + dx / 2.0;
            double py = margY + marginalSize - 5 - (margX[ix] / maxMargX) * (marginalSize - 10);
            margXPts.emplace_back(px, py);
        }
        canvas_.writePolyline(out, margXPts, config_.desiredColor, 1.0);

        // Y marginal (right strip, above colorbar)
        auto margY_vec = grid.marginalY();
        double maxMargY = *std::max_element(margY_vec.begin(), margY_vec.end());
        if (maxMargY < 1e-15) maxMargY = 1.0;
        int margX_pos = plotX + plotW + 40 + cbW + 10;
        std::vector<std::pair<double, double>> margYPts;
        for (std::size_t iy = 0; iy < nY; ++iy) {
            double py = plotY + plotH - static_cast<double>(iy) * dy - dy / 2.0;
            double px = margX_pos + (margY_vec[iy] / maxMargY) * (marginalSize - 10);
            margYPts.emplace_back(px, py);
        }
        canvas_.writePolyline(out, margYPts, config_.desiredColor, 1.0);
    }

    // Info text
    std::string info = std::format("n={} | h_x={:.3g} h_y={:.3g} | MI={:.3f} bits | r={:.3f} | η²={:.3f}",
                                   kde.grid.sampleCount,
                                   kde.grid.bandwidthX, kde.grid.bandwidthY,
                                   kde.mutualInformation,
                                   kde.pearsonCorrelation,
                                   kde.correlationRatio);
    canvas_.writeText(out, margin, svgH - 15, info, config_.fontSize - 1, config_.textColor, "start");
}

//=============================================================================
// KDE Conditional
//=============================================================================

void KdeSvgRenderer::renderConditional(std::ostream& out,
                                       int svgW, int svgH,
                                       const KdeEvaluation& kde) const {
    if (kde.conditional.empty()) {
        canvas_.renderTitle(out, "Conditional Stats (no data)", svgW, config_.margin);
        return;
    }

    // Find bounds
    double minX = kde.conditional.front().xValue;
    double maxX = kde.conditional.back().xValue;
    double maxY = 0.0;
    for (const auto& cs : kde.conditional) {
        if (!cs.valid) continue;
        maxY = std::max(maxY, cs.p95Y);
    }
    if (maxY < 1e-15) maxY = 1.0;
    double minY = 0.0;

    SvgCanvas::AxisBounds bounds;
    bounds.minX = minX;
    bounds.maxX = maxX;
    bounds.minY = minY;
    bounds.maxY = maxY * 1.1;

    canvas_.renderGrid(out, bounds, svgW, svgH, config_.margin);
    canvas_.renderAxes(out, bounds, svgW, svgH, config_.margin,
               std::format("{} ({})", toString(kde.derivativeAxis), unitString(kde.derivativeAxis)),
               std::format("{} ({})", toString(kde.deviationAxis), unitString(kde.deviationAxis)));

    std::string title = std::format("Conditional {} vs {} (quantile bands)",
                                    toString(kde.deviationAxis),
                                    toString(kde.derivativeAxis));
    canvas_.renderTitle(out, title, svgW, config_.margin);

    // Helper to transform data to SVG
    auto tf = [&](double x, double y) -> std::pair<double, double> {
        return canvas_.transform(x, y, bounds, svgW, svgH, config_.margin);
    };

    // Draw quantile bands (P5-P95, P25-P75)
    auto drawBand = [&](auto pLow, auto pHigh, const std::string& color, double alpha) {
        std::vector<std::pair<double, double>> top, bottom;
        for (const auto& cs : kde.conditional) {
            if (!cs.valid) continue;
            auto [px, py1] = tf(cs.xValue, pHigh(cs));
            auto [_, py2] = tf(cs.xValue, pLow(cs));
            top.emplace_back(px, py1);
            bottom.emplace_back(px, py2);
        }
        if (top.empty()) return;
        // Build polygon: top forward + bottom reverse
        std::vector<std::pair<double, double>> polygon = top;
        polygon.insert(polygon.end(), bottom.rbegin(), bottom.rend());
        out << "<polygon points=\"";
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            if (i > 0) out << " ";
            out << canvas_.fmt(polygon[i].first) << "," << canvas_.fmt(polygon[i].second);
        }
        out << "\" fill=\"" << color << "\" fill-opacity=\"" << canvas_.fmt(alpha)
            << "\" stroke=\"none\"/>\n";
    };

    drawBand([](const auto& cs) { return cs.p05Y; },
             [](const auto& cs) { return cs.p95Y; },
             "#FF6600", 0.15);
    drawBand([](const auto& cs) { return cs.p25Y; },
             [](const auto& cs) { return cs.p75Y; },
             "#FF6600", 0.25);

    // Draw quantile lines
    auto drawLine = [&](auto pFunc, const std::string& color, double width) {
        std::vector<std::pair<double, double>> pts;
        for (const auto& cs : kde.conditional) {
            if (!cs.valid) continue;
            auto [px, py] = tf(cs.xValue, pFunc(cs));
            pts.emplace_back(px, py);
        }
        if (pts.size() > 1) canvas_.writePolyline(out, pts, color, width);
    };

    drawLine([](const auto& cs) { return cs.p95Y; }, "#FF6600", 0.8);
    drawLine([](const auto& cs) { return cs.p05Y; }, "#FF6600", 0.8);
    drawLine([](const auto& cs) { return cs.p75Y; }, "#CC6600", 0.8);
    drawLine([](const auto& cs) { return cs.p25Y; }, "#CC6600", 0.8);
    drawLine([](const auto& cs) { return cs.medianY; }, "#0066CC", 1.2);
    drawLine([](const auto& cs) { return cs.meanY; }, "#CC0000", 2.0);

    // Legend
    if (config_.includeLegend) {
        std::vector<std::pair<std::string, std::string>> entries = {
            {"Mean", "#CC0000"},
            {"Median", "#0066CC"},
            {"P25-P75", "#CC6600"},
            {"P5-P95", "#FF6600"},
        };
        canvas_.renderLegend(out, svgW, svgH, entries);
    }
}

//=============================================================================
// KDE Marginal X
//=============================================================================

void KdeSvgRenderer::renderMarginalX(std::ostream& out,
                                     int svgW, int svgH,
                                     const KdeEvaluation& kde) const {
    if (kde.grid.xBins.empty()) return;

    auto margX = kde.grid.marginalX();
    double maxMarg = *std::max_element(margX.begin(), margX.end());
    if (maxMarg < 1e-15) maxMarg = 1.0;

    SvgCanvas::AxisBounds bounds;
    bounds.minX = kde.grid.xBins.front();
    bounds.maxX = kde.grid.xBins.back();
    bounds.minY = 0;
    bounds.maxY = maxMarg * 1.1;

    canvas_.renderGrid(out, bounds, svgW, svgH, config_.margin);
    canvas_.renderAxes(out, bounds, svgW, svgH, config_.margin,
               std::format("{} ({})", toString(kde.derivativeAxis), unitString(kde.derivativeAxis)),
               "Density");

    canvas_.renderTitle(out, std::format("Marginal: {}", toString(kde.derivativeAxis)),
                svgW, config_.margin);

    std::vector<std::pair<double, double>> pts;
    for (std::size_t i = 0; i < kde.grid.xBins.size(); ++i) {
        auto [px, py] = canvas_.transform(kde.grid.xBins[i], margX[i], bounds, svgW, svgH, config_.margin);
        pts.emplace_back(px, py);
    }
    canvas_.writePolyline(out, pts, config_.desiredColor, config_.lineWidth);
}

//=============================================================================
// KDE Marginal Y
//=============================================================================

void KdeSvgRenderer::renderMarginalY(std::ostream& out,
                                     int svgW, int svgH,
                                     const KdeEvaluation& kde) const {
    if (kde.grid.yBins.empty()) return;

    auto margY = kde.grid.marginalY();
    double maxMarg = *std::max_element(margY.begin(), margY.end());
    if (maxMarg < 1e-15) maxMarg = 1.0;

    SvgCanvas::AxisBounds bounds;
    bounds.minX = 0;
    bounds.maxX = maxMarg * 1.1;
    bounds.minY = kde.grid.yBins.front();
    bounds.maxY = kde.grid.yBins.back();

    canvas_.renderGrid(out, bounds, svgW, svgH, config_.margin);
    canvas_.renderAxes(out, bounds, svgW, svgH, config_.margin,
               "Density",
               std::format("{} ({})", toString(kde.deviationAxis), unitString(kde.deviationAxis)));

    canvas_.renderTitle(out, std::format("Marginal: {}", toString(kde.deviationAxis)),
                svgW, config_.margin);

    std::vector<std::pair<double, double>> pts;
    for (std::size_t i = 0; i < kde.grid.yBins.size(); ++i) {
        auto [px, py] = canvas_.transform(margY[i], kde.grid.yBins[i], bounds, svgW, svgH, config_.margin);
        pts.emplace_back(px, py);
    }
    canvas_.writePolyline(out, pts, config_.desiredColor, config_.lineWidth);
}

//=============================================================================
// File export helpers
//=============================================================================

bool KdeSvgRenderer::exportHeatmap(const std::string& filename,
                                   const KdeEvaluation& kde) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    canvas_.writeHeader(file, config_.width, config_.height);
    renderHeatmap(file, config_.width, config_.height, kde);
    canvas_.writeFooter(file);
    return file.good();
}

bool KdeSvgRenderer::exportConditional(const std::string& filename,
                                       const KdeEvaluation& kde) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    canvas_.writeHeader(file, config_.width, config_.height);
    renderConditional(file, config_.width, config_.height, kde);
    canvas_.writeFooter(file);
    return file.good();
}

bool KdeSvgRenderer::exportDashboard(const std::string& filename,
                                     const KdeEvaluation& kde) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;

    // Dashboard: 2x2 grid
    int totalW = config_.width;
    int totalH = config_.height;
    // Make it wider for dashboard
    int dashW = std::max(totalW, 1400);
    int dashH = std::max(totalH, 1000);

    canvas_.writeHeader(file, dashW, dashH);
    canvas_.writeBackground(file, dashW, dashH);

    canvas_.renderTitle(file, std::format("KDE Dashboard: {} vs {}",
                                  toString(kde.derivativeAxis),
                                  toString(kde.deviationAxis)),
                dashW, config_.margin);

    int margin = config_.margin;
    int gap = 10;
    int subW = (dashW - 3 * margin - gap) / 2;
    int subH = (dashH - 3 * margin - gap - 40) / 2;

    // Cell positions
    auto cellPos = [&](int row, int col) -> std::pair<int, int> {
        int x = margin + col * (subW + gap);
        int y = margin + 30 + row * (subH + gap);
        return {x, y};
    };

    // Row 0: Heatmap (spans both columns), Marginal Y
    {
        auto [x, y] = cellPos(0, 0);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderHeatmap(file, subW, subH, kde);
        file << "</g>\n";
    }
    {
        auto [x, y] = cellPos(0, 1);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderMarginalY(file, subW, subH, kde);
        file << "</g>\n";
    }

    // Row 1: Marginal X, Conditional
    {
        auto [x, y] = cellPos(1, 0);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderMarginalX(file, subW, subH, kde);
        file << "</g>\n";
    }
    {
        auto [x, y] = cellPos(1, 1);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderConditional(file, subW, subH, kde);
        file << "</g>\n";
    }

    canvas_.writeFooter(file);
    return file.good();
}

std::vector<std::string> KdeSvgRenderer::exportAll(
    const std::string& outputDir,
    const std::string& filePrefix,
    const KdeEvaluation& kde) const {

    std::vector<std::string> files;

    auto tryExport = [&](const std::string& name, auto fn) {
        std::string path = outputDir + "/" + filePrefix + "_" + name + ".svg";
        if (fn(path)) files.push_back(path);
    };

    tryExport("kde_heatmap", [&](const std::string& p) {
        return exportHeatmap(p, kde);
    });
    tryExport("kde_conditional", [&](const std::string& p) {
        return exportConditional(p, kde);
    });
    tryExport("kde_marginal_x", [&](const std::string& p) {
        std::ofstream f(p);
        if (!f.is_open()) return false;
        canvas_.writeHeader(f, config_.width, config_.height);
        renderMarginalX(f, config_.width, config_.height, kde);
        canvas_.writeFooter(f);
        return f.good();
    });
    tryExport("kde_marginal_y", [&](const std::string& p) {
        std::ofstream f(p);
        if (!f.is_open()) return false;
        canvas_.writeHeader(f, config_.width, config_.height);
        renderMarginalY(f, config_.width, config_.height, kde);
        canvas_.writeFooter(f);
        return f.good();
    });
    tryExport("kde_dashboard", [&](const std::string& p) {
        return exportDashboard(p, kde);
    });

    return files;
}

} // namespace MotionReplanner
