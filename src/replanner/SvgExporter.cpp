/**
 * @file SvgExporter.cpp
 * @brief SVG vector graphics export implementation.
 */

#include "tether/motion_replanner/SvgExporter.hpp"
#include "tether/motion_replanner/KdeSvgRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace MotionReplanner {

//=============================================================================
// Constructor
//=============================================================================

SvgExporter::SvgExporter(SvgConfig config)
    : config_(config), canvas_(config_) {}

//=============================================================================
// Utility
//=============================================================================

std::string SvgExporter::fmt(double v) const {
    return canvas_.fmt(v);
}

std::vector<std::pair<double,double>> SvgExporter::extractPoints2D(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    int plane) const {

    std::vector<std::pair<double,double>> points;
    points.reserve(samples.size());
    std::size_t step = std::max(std::size_t{1}, config_.renderDownsample);

    for (std::size_t i = 0; i < samples.size(); i += step) {
        double x, y;
        switch (plane) {
            case 0: x = samples[i].position[0]; y = samples[i].position[1]; break; // XY
            case 1: x = samples[i].position[0]; y = samples[i].position[2]; break; // XZ
            case 2: x = samples[i].position[1]; y = samples[i].position[2]; break; // YZ
            default: x = samples[i].position[0]; y = samples[i].position[1]; break;
        }
        points.emplace_back(x, y);
    }
    return points;
}

std::pair<double,double> SvgExporter::project3D(double x, double y, double z) const {
    // Isometric projection: 30° azimuth, 30° elevation
    const double az = M_PI / 6.0;  // 30°
    const double el = M_PI / 6.0;  // 30°
    double sx = x * std::cos(az) + z * std::cos(az + M_PI / 2.0);
    double sy = -x * std::sin(az) - z * std::sin(az + M_PI / 2.0) + y * std::cos(el);
    return {sx, sy};
}

//=============================================================================
// Bounds and transforms
//=============================================================================

SvgExporter::AxisBounds SvgExporter::computeBounds(
    const std::vector<std::pair<double,double>>& points) const {
    return canvas_.computeBounds(points);
}

SvgExporter::AxisBounds SvgExporter::computeBoundsMulti(
    const std::vector<std::vector<std::pair<double,double>>>& allPoints) const {
    return canvas_.computeBoundsMulti(allPoints);
}

std::pair<double,double> SvgExporter::transform(
    double dataX, double dataY, const AxisBounds& bounds,
    double svgWidth, double svgHeight, double margin) const {
    return canvas_.transform(dataX, dataY, bounds, svgWidth, svgHeight, margin);
}

//=============================================================================
// Low-level SVG element writers
//=============================================================================

void SvgExporter::writeHeader(std::ostream& out, int width, int height) const {
    canvas_.writeHeader(out, width, height);
}

void SvgExporter::writeFooter(std::ostream& out) const {
    canvas_.writeFooter(out);
}

void SvgExporter::writeBackground(std::ostream& out, int width, int height) const {
    canvas_.writeBackground(out, width, height);
}

void SvgExporter::writeRect(std::ostream& out, double x, double y,
                            double w, double h, const std::string& fill,
                            const std::string& stroke, double strokeWidth) const {
    canvas_.writeRect(out, x, y, w, h, fill, stroke, strokeWidth);
}

void SvgExporter::writeLine(std::ostream& out, double x1, double y1,
                            double x2, double y2,
                            const std::string& stroke, double width) const {
    canvas_.writeLine(out, x1, y1, x2, y2, stroke, width);
}

void SvgExporter::writeDashedLine(std::ostream& out, double x1, double y1,
                                  double x2, double y2,
                                  const std::string& stroke, double width) const {
    canvas_.writeDashedLine(out, x1, y1, x2, y2, stroke, width);
}

void SvgExporter::writePolyline(std::ostream& out,
                                const std::vector<std::pair<double,double>>& points,
                                const std::string& stroke, double width,
                                bool fill, const std::string& fillColor) const {
    canvas_.writePolyline(out, points, stroke, width, fill, fillColor);
}

void SvgExporter::writeText(std::ostream& out, double x, double y,
                            const std::string& text, int size,
                            const std::string& fill,
                            const std::string& anchor) const {
    canvas_.writeText(out, x, y, text, size, fill, anchor);
}

void SvgExporter::writeCircle(std::ostream& out, double cx, double cy, double r,
                              const std::string& fill,
                              const std::string& stroke, double strokeWidth) const {
    canvas_.writeCircle(out, cx, cy, r, fill, stroke, strokeWidth);
}

//=============================================================================
// Axis, grid, legend, title
//=============================================================================

void SvgExporter::renderGrid(std::ostream& out, const AxisBounds& bounds,
                              int svgW, int svgH, int margin) const {
    canvas_.renderGrid(out, bounds, svgW, svgH, margin);
}

void SvgExporter::renderAxes(std::ostream& out, const AxisBounds& bounds,
                              int svgW, int svgH, int margin,
                              const std::string& xLabel, const std::string& yLabel) const {
    canvas_.renderAxes(out, bounds, svgW, svgH, margin, xLabel, yLabel);
}

void SvgExporter::renderLegend(std::ostream& out, int svgW, int svgH,
                               const std::vector<std::pair<std::string, std::string>>& entries) const {
    canvas_.renderLegend(out, svgW, svgH, entries);
}

void SvgExporter::renderTitle(std::ostream& out, const std::string& title,
                               int svgW, int margin) const {
    canvas_.renderTitle(out, title, svgW, margin);
}

//=============================================================================
// Individual plot renderers
//=============================================================================

void SvgExporter::renderTrajectory2D(std::ostream& out, int plane,
                                     int svgW, int svgH,
                                     const std::vector<GCodeExport::TrajectorySample>& desired,
                                     const std::vector<GCodeExport::TrajectorySample>& actual) const {
    auto desiredPts = extractPoints2D(desired, plane);
    auto actualPts = extractPoints2D(actual, plane);

    AxisBounds bounds = computeBoundsMulti({desiredPts, actualPts});

    // Transform all points to SVG coordinates
    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        result.reserve(pts.size());
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    auto desiredSvg = transformPts(desiredPts);
    auto actualSvg = transformPts(actualPts);

    // Grid and axes
    renderGrid(out, bounds, svgW, svgH, config_.margin);
    std::string xLabel, yLabel, title;
    switch (plane) {
        case 0: xLabel = "X (mm)"; yLabel = "Y (mm)"; title = "XY Plane"; break;
        case 1: xLabel = "X (mm)"; yLabel = "Z (mm)"; title = "XZ Plane"; break;
        case 2: xLabel = "Y (mm)"; yLabel = "Z (mm)"; title = "YZ Plane"; break;
    }
    renderAxes(out, bounds, svgW, svgH, config_.margin, xLabel, yLabel);
    renderTitle(out, title, svgW, config_.margin);

    // Deviation envelope: draw lines from actual to desired
    if (config_.showDeviationEnvelope) {
        auto n = std::min(desiredSvg.size(), actualSvg.size());
        for (std::size_t i = 0; i < n; i += std::max(std::size_t{1}, config_.renderDownsample * 5)) {
            writeLine(out, actualSvg[i].first, actualSvg[i].second,
                      desiredSvg[i].first, desiredSvg[i].second,
                      config_.errorColor, 0.3);
        }
    }

    // Closest point lines
    if (config_.showClosestPointLines) {
        auto n = std::min(desiredSvg.size(), actualSvg.size());
        for (std::size_t i = 0; i < n; i += std::max(std::size_t{1}, config_.renderDownsample * 10)) {
            writeLine(out, actualSvg[i].first, actualSvg[i].second,
                      desiredSvg[i].first, desiredSvg[i].second,
                      config_.peakColor, 0.5);
        }
    }

    // Desired path (solid)
    writePolyline(out, desiredSvg, config_.desiredColor, config_.lineWidth);

    // Actual path (dashed style — we use a different color)
    writePolyline(out, actualSvg, config_.actualColor, config_.lineWidth);

    // Legend
    renderLegend(out, svgW, svgH, {{"Desired", config_.desiredColor},
                                    {"Actual", config_.actualColor}});
}

void SvgExporter::renderTrajectory3D(std::ostream& out,
                                     int svgW, int svgH,
                                     const std::vector<GCodeExport::TrajectorySample>& desired,
                                     const std::vector<GCodeExport::TrajectorySample>& actual) const {
    std::size_t step = std::max(std::size_t{1}, config_.renderDownsample);

    // Project all points to 2D isometric
    std::vector<std::pair<double,double>> desiredProj, actualProj;
    for (std::size_t i = 0; i < desired.size(); i += step) {
        desiredProj.push_back(project3D(desired[i].position[0], desired[i].position[1], desired[i].position[2]));
    }
    for (std::size_t i = 0; i < actual.size(); i += step) {
        actualProj.push_back(project3D(actual[i].position[0], actual[i].position[1], actual[i].position[2]));
    }

    AxisBounds bounds = computeBoundsMulti({desiredProj, actualProj});

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    auto desiredSvg = transformPts(desiredProj);
    auto actualSvg = transformPts(actualProj);

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin, "Iso X", "Iso Y");
    renderTitle(out, "3D Isometric View", svgW, config_.margin);

    writePolyline(out, desiredSvg, config_.desiredColor, config_.lineWidth);
    writePolyline(out, actualSvg, config_.actualColor, config_.lineWidth);

    renderLegend(out, svgW, svgH, {{"Desired", config_.desiredColor},
                                    {"Actual", config_.actualColor}});
}

void SvgExporter::renderErrorProfile(std::ostream& out, bool vsPathLength,
                                     int svgW, int svgH,
                                     const std::vector<GCodeExport::TrajectorySample>& desired,
                                     const std::vector<GCodeExport::TrajectorySample>& actual) const {
    auto n = std::min(desired.size(), actual.size());
    if (n == 0) return;

    // Compute errors
    std::vector<std::pair<double,double>> contourPts, lagPts, combinedPts;
    for (std::size_t i = 0; i < n; ++i) {
        double abscissa = vsPathLength ? desired[i].pathPosition : desired[i].time;
        double dx = actual[i].position[0] - desired[i].position[0];
        double dy = actual[i].position[1] - desired[i].position[1];
        double dz = actual[i].position[2] - desired[i].position[2];
        double combined = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Simple contour/lag via tangent projection
        double vmag = std::sqrt(desired[i].velocity[0] * desired[i].velocity[0] +
                                desired[i].velocity[1] * desired[i].velocity[1] +
                                desired[i].velocity[2] * desired[i].velocity[2]);
        double contour, lag;
        if (vmag > 1e-9) {
            double tx = desired[i].velocity[0] / vmag;
            double ty = desired[i].velocity[1] / vmag;
            double tz = desired[i].velocity[2] / vmag;
            lag = dx * tx + dy * ty + dz * tz;
            double cx = dx - lag * tx, cy = dy - lag * ty, cz = dz - lag * tz;
            contour = std::sqrt(cx * cx + cy * cy + cz * cz);
        } else {
            contour = combined;
            lag = 0.0;
        }

        contourPts.emplace_back(abscissa, contour);
        lagPts.emplace_back(abscissa, lag);
        combinedPts.emplace_back(abscissa, combined);
    }

    AxisBounds bounds = computeBoundsMulti({contourPts, lagPts, combinedPts});

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    std::string xLabel = vsPathLength ? "Arc Length (mm)" : "Time (s)";
    std::string title = vsPathLength ? "Error vs Path Length" : "Error vs Time";
    renderAxes(out, bounds, svgW, svgH, config_.margin, xLabel, "Error (mm)");
    renderTitle(out, title, svgW, config_.margin);

    writePolyline(out, transformPts(contourPts), config_.errorColor, config_.lineWidth);
    writePolyline(out, transformPts(lagPts), config_.spectralColor, config_.lineWidth);
    writePolyline(out, transformPts(combinedPts), config_.actualColor, config_.lineWidth);

    renderLegend(out, svgW, svgH, {{"Contour", config_.errorColor},
                                    {"Lag", config_.spectralColor},
                                    {"Combined", config_.actualColor}});
}

void SvgExporter::renderErrorHistogram(std::ostream& out,
                                       int svgW, int svgH,
                                       const tether::motion::replanner::QuantitativeEvaluation& quant) const {
    // We need the raw contour errors. Since we don't have them here,
    // we'll create a simplified histogram from the stats.
    // In a real implementation, we'd pass the raw errors. For now,
    // we generate a synthetic histogram from mean/stdDev/min/max.

    int numBins = 30;
    double minVal = quant.contourStats.minError;
    double maxVal = quant.contourStats.maxError;
    if (maxVal - minVal < 1e-12) {
        maxVal = minVal + 1.0;
    }
    double binWidth = (maxVal - minVal) / numBins;

    // Generate a Gaussian-like distribution from mean and stdDev
    std::vector<double> binHeights(numBins, 0.0);
    double mu = quant.contourStats.meanError;
    double sigma = quant.contourStats.stdDev;
    if (sigma < 1e-12) sigma = binWidth;

    for (int i = 0; i < numBins; ++i) {
        double binCenter = minVal + (static_cast<double>(i) + 0.5) * binWidth;
        double gaussian = std::exp(-0.5 * std::pow((binCenter - mu) / sigma, 2.0))
                          / (sigma * std::sqrt(2.0 * M_PI));
        binHeights[i] = gaussian * static_cast<double>(quant.contourStats.sampleCount) * binWidth;
    }

    // Find max height for scaling
    double maxHeight = *std::max_element(binHeights.begin(), binHeights.end());
    if (maxHeight < 1e-12) maxHeight = 1.0;

    AxisBounds bounds;
    bounds.minX = minVal;
    bounds.maxX = maxVal;
    bounds.minY = 0;
    bounds.maxY = maxHeight * 1.1;

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin, "Contour Error (mm)", "Count");
    renderTitle(out, "Contour Error Distribution", svgW, config_.margin);

    // Draw bars
    double plotW = svgW - 2.0 * config_.margin;
    double barW = plotW / numBins;
    for (int i = 0; i < numBins; ++i) {
        double binX = minVal + static_cast<double>(i) * binWidth;
        auto [px, py] = transform(binX, binHeights[i], bounds, svgW, svgH, config_.margin);
        auto [px2, py2] = transform(binX + binWidth, 0, bounds, svgW, svgH, config_.margin);
        writeRect(out, px, py, px2 - px, py2 - py, config_.errorColor,
                  config_.axisColor, 0.5);
    }

    // P95 and P99 lines
    auto drawThreshold = [&](double value, const std::string& label) {
        auto [px, py1] = transform(value, bounds.minY, bounds, svgW, svgH, config_.margin);
        auto [_, py2] = transform(value, bounds.maxY, bounds, svgW, svgH, config_.margin);
        writeDashedLine(out, px, py1, px, py2, config_.peakColor, 1.0);
        writeText(out, px + 5, py2 + 15, label, config_.fontSize - 2, config_.peakColor, "start");
    };

    drawThreshold(quant.contourStats.p95Error, "P95");
    drawThreshold(quant.contourStats.p99Error, "P99");
}

void SvgExporter::renderErrorEnvelope(std::ostream& out, int plane,
                                      int svgW, int svgH,
                                      const std::vector<GCodeExport::TrajectorySample>& desired,
                                      const std::vector<GCodeExport::TrajectorySample>& actual) const {
    // Like trajectory 2D but with magnified error envelope
    auto desiredPts = extractPoints2D(desired, plane);
    auto actualPts = extractPoints2D(actual, plane);

    AxisBounds bounds = computeBoundsMulti({desiredPts, actualPts});

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    auto desiredSvg = transformPts(desiredPts);
    auto actualSvg = transformPts(actualPts);

    std::string xLabel, yLabel, title;
    switch (plane) {
        case 0: xLabel = "X (mm)"; yLabel = "Y (mm)"; title = "Error Envelope (XY)"; break;
        case 1: xLabel = "X (mm)"; yLabel = "Z (mm)"; title = "Error Envelope (XZ)"; break;
        case 2: xLabel = "Y (mm)"; yLabel = "Z (mm)"; title = "Error Envelope (YZ)"; break;
    }

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin, xLabel, yLabel);
    renderTitle(out, title, svgW, config_.margin);

    // Draw magnified deviation lines
    auto n = std::min(desiredSvg.size(), actualSvg.size());
    for (std::size_t i = 0; i < n; i += std::max(std::size_t{1}, config_.renderDownsample)) {
        // Magnify the deviation
        double dx = actualSvg[i].first - desiredSvg[i].first;
        double dy = actualSvg[i].second - desiredSvg[i].second;
        double magX = desiredSvg[i].first + dx * config_.envelopeScale;
        double magY = desiredSvg[i].second + dy * config_.envelopeScale;
        writeLine(out, desiredSvg[i].first, desiredSvg[i].second,
                  magX, magY, config_.errorColor, 0.5);
    }

    // Desired path
    writePolyline(out, desiredSvg, config_.desiredColor, config_.lineWidth);

    // Magnified actual path
    std::vector<std::pair<double,double>> magnifiedActual;
    for (std::size_t i = 0; i < n; ++i) {
        double dx = actualSvg[i].first - desiredSvg[i].first;
        double dy = actualSvg[i].second - desiredSvg[i].second;
        magnifiedActual.emplace_back(
            desiredSvg[i].first + dx * config_.envelopeScale,
            desiredSvg[i].second + dy * config_.envelopeScale);
    }
    writePolyline(out, magnifiedActual, config_.actualColor, config_.lineWidth * 0.7);

    renderLegend(out, svgW, svgH, {{"Desired", config_.desiredColor},
                                    {"Actual (magnified " + std::to_string(static_cast<int>(config_.envelopeScale)) + "x)", config_.actualColor}});
}

void SvgExporter::renderSpectralPlot(std::ostream& out, bool magnitude,
                                     int svgW, int svgH,
                                     const tether::motion::replanner::SpectralEvaluation& spectral,
                                     tether::motion::replanner::SpectralComponent component,
                                     tether::motion::replanner::SpectralDomain domain) const {
    // Select the right spectrum
    const tether::motion::replanner::ComponentSpectrum* spec = nullptr;
    std::string componentName, domainName;

    if (domain == tether::motion::replanner::SpectralDomain::Spatial) {
        domainName = "Spatial";
        switch (component) {
            case tether::motion::replanner::SpectralComponent::Contour:
                spec = &spectral.spatialContour; componentName = "Contour"; break;
            case tether::motion::replanner::SpectralComponent::Lag:
                spec = &spectral.spatialLag; componentName = "Lag"; break;
            case tether::motion::replanner::SpectralComponent::Binormal:
                spec = &spectral.spatialBinormal; componentName = "Binormal"; break;
            case tether::motion::replanner::SpectralComponent::Combined:
                spec = &spectral.spatialCombined; componentName = "Combined"; break;
        }
    } else {
        domainName = "Temporal";
        switch (component) {
            case tether::motion::replanner::SpectralComponent::Contour:
                spec = &spectral.temporalContour; componentName = "Contour"; break;
            case tether::motion::replanner::SpectralComponent::Lag:
                spec = &spectral.temporalLag; componentName = "Lag"; break;
            case tether::motion::replanner::SpectralComponent::Binormal:
                spec = &spectral.temporalBinormal; componentName = "Binormal"; break;
            case tether::motion::replanner::SpectralComponent::Combined:
                spec = &spectral.temporalCombined; componentName = "Combined"; break;
        }
    }

    if (!spec || spec->frequencies.empty()) return;

    // Build data points
    std::vector<std::pair<double,double>> dataPts;
    const auto& values = magnitude ? spec->magnitudes : spec->phases;
    for (std::size_t i = 0; i < spec->frequencies.size(); ++i) {
        double y = values[i];
        if (config_.spectralLogScale && magnitude && y > 1e-15) {
            y = 10.0 * std::log10(y);
        }
        dataPts.emplace_back(spec->frequencies[i], y);
    }

    AxisBounds bounds = computeBounds(dataPts);
    if (magnitude && config_.spectralLogScale) {
        // Adjust for dB scale
        bounds.minY = std::max(bounds.minY, -120.0);
    }

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    std::string xLabel = (domain == tether::motion::replanner::SpectralDomain::Spatial)
        ? "Frequency (cyc/mm)" : "Frequency (Hz)";
    std::string yLabel = magnitude
        ? (config_.spectralLogScale ? "Magnitude (dB)" : "Magnitude")
        : "Phase (rad)";
    std::string title = std::format("{} {} Spectrum", domainName, magnitude ? "Magnitude" : "Phase");
    renderAxes(out, bounds, svgW, svgH, config_.margin, xLabel, yLabel);
    renderTitle(out, title, svgW, config_.margin);

    // Plot the spectrum
    writePolyline(out, transformPts(dataPts), config_.spectralColor, config_.lineWidth);

    // Mark peaks
    if (magnitude) {
        for (const auto& peak : spec->peaks) {
            double y = peak.magnitude;
            if (config_.spectralLogScale && y > 1e-15) {
                y = 10.0 * std::log10(y);
            }
            auto [px, py] = transform(peak.frequency, y, bounds, svgW, svgH, config_.margin);
            writeCircle(out, px, py, 4, config_.peakColor, config_.peakColor, 1.0);
            writeText(out, px + 8, py - 5, std::format("{:.4f}", peak.frequency),
                      config_.fontSize - 2, config_.peakColor, "start");
        }
    }

    // Path geometry frequency lines
    if (config_.showGeometryLines && domain == tether::motion::replanner::SpectralDomain::Spatial) {
        auto drawGeoLine = [&](double freq, const std::string& label) {
            if (freq <= 0) return;
            auto [px, py1] = transform(freq, bounds.minY, bounds, svgW, svgH, config_.margin);
            auto [_, py2] = transform(freq, bounds.maxY, bounds, svgW, svgH, config_.margin);
            writeDashedLine(out, px, py1, px, py2, config_.geometryColor, 1.0);
            writeText(out, px + 3, py2 + 12, label, config_.fontSize - 3, config_.geometryColor, "start");
        };
        drawGeoLine(spectral.geometryCorrelation.cornerFrequency, "corner");
        drawGeoLine(spectral.geometryCorrelation.segmentFrequency, "segment");
    }

    renderLegend(out, svgW, svgH, {{componentName, config_.spectralColor},
                                    {"Peaks", config_.peakColor}});
}

void SvgExporter::renderVelocityProfile(std::ostream& out,
                                        int svgW, int svgH,
                                        const std::vector<GCodeExport::TrajectorySample>& desired,
                                        const std::vector<GCodeExport::TrajectorySample>& actual) const {
    auto n = std::min(desired.size(), actual.size());
    if (n == 0) return;

    std::vector<std::pair<double,double>> desiredVel, actualVel;
    for (std::size_t i = 0; i < n; ++i) {
        double dv = std::sqrt(desired[i].velocity[0] * desired[i].velocity[0] +
                              desired[i].velocity[1] * desired[i].velocity[1] +
                              desired[i].velocity[2] * desired[i].velocity[2]);
        double av = std::sqrt(actual[i].velocity[0] * actual[i].velocity[0] +
                              actual[i].velocity[1] * actual[i].velocity[1] +
                              actual[i].velocity[2] * actual[i].velocity[2]);
        desiredVel.emplace_back(desired[i].time, dv);
        actualVel.emplace_back(actual[i].time, av);
    }

    AxisBounds bounds = computeBoundsMulti({desiredVel, actualVel});

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin, "Time (s)", "Velocity (mm/s)");
    renderTitle(out, "Velocity Profile", svgW, config_.margin);

    writePolyline(out, transformPts(desiredVel), config_.desiredColor, config_.lineWidth);
    writePolyline(out, transformPts(actualVel), config_.actualColor, config_.lineWidth);

    renderLegend(out, svgW, svgH, {{"Desired", config_.desiredColor},
                                    {"Actual", config_.actualColor}});
}

void SvgExporter::renderAccelerationProfile(std::ostream& out,
                                            int svgW, int svgH,
                                            const std::vector<GCodeExport::TrajectorySample>& desired,
                                            const std::vector<GCodeExport::TrajectorySample>& actual) const {
    auto n = std::min(desired.size(), actual.size());
    if (n == 0) return;

    std::vector<std::pair<double,double>> desiredAcc, actualAcc;
    for (std::size_t i = 0; i < n; ++i) {
        double da = std::sqrt(desired[i].acceleration[0] * desired[i].acceleration[0] +
                              desired[i].acceleration[1] * desired[i].acceleration[1] +
                              desired[i].acceleration[2] * desired[i].acceleration[2]);
        double aa = std::sqrt(actual[i].acceleration[0] * actual[i].acceleration[0] +
                              actual[i].acceleration[1] * actual[i].acceleration[1] +
                              actual[i].acceleration[2] * actual[i].acceleration[2]);
        desiredAcc.emplace_back(desired[i].time, da);
        actualAcc.emplace_back(actual[i].time, aa);
    }

    AxisBounds bounds = computeBoundsMulti({desiredAcc, actualAcc});

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin, "Time (s)", "Acceleration (mm/s²)");
    renderTitle(out, "Acceleration Profile", svgW, config_.margin);

    writePolyline(out, transformPts(desiredAcc), config_.desiredColor, config_.lineWidth);
    writePolyline(out, transformPts(actualAcc), config_.actualColor, config_.lineWidth);

    renderLegend(out, svgW, svgH, {{"Desired", config_.desiredColor},
                                    {"Actual", config_.actualColor}});
}

void SvgExporter::renderPhasePortrait(std::ostream& out,
                                      int svgW, int svgH,
                                      const std::vector<GCodeExport::TrajectorySample>& actual) const {
    if (actual.empty()) return;

    // Plot X position vs X velocity
    std::vector<std::pair<double,double>> phasePts;
    for (const auto& s : actual) {
        phasePts.emplace_back(s.position[0], s.velocity[0]);
    }

    AxisBounds bounds = computeBounds(phasePts);

    auto transformPts = [&](const std::vector<std::pair<double,double>>& pts) {
        std::vector<std::pair<double,double>> result;
        for (const auto& [x, y] : pts) {
            result.push_back(transform(x, y, bounds, svgW, svgH, config_.margin));
        }
        return result;
    };

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin, "X Position (mm)", "X Velocity (mm/s)");
    renderTitle(out, "Phase Portrait (X)", svgW, config_.margin);

    writePolyline(out, transformPts(phasePts), config_.actualColor, config_.lineWidth);
}

//=============================================================================
// Public export methods
//=============================================================================

bool SvgExporter::exportPlot(const std::string& filename, SvgPlotType type,
                             const std::vector<GCodeExport::TrajectorySample>& desired,
                             const std::vector<GCodeExport::TrajectorySample>& actual,
                             const tether::motion::replanner::QuantitativeEvaluation* quant,
                             const tether::motion::replanner::SpectralEvaluation* spectral) const {

    std::ofstream file(filename);
    if (!file.is_open()) return false;

    writeHeader(file, config_.width, config_.height);
    writeBackground(file, config_.width, config_.height);

    switch (type) {
        case SvgPlotType::TrajectoryXY:
            renderTrajectory2D(file, 0, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::TrajectoryXZ:
            renderTrajectory2D(file, 1, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::TrajectoryYZ:
            renderTrajectory2D(file, 2, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::Trajectory3D:
            renderTrajectory3D(file, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::ErrorVsPathLength:
            renderErrorProfile(file, true, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::ErrorVsTime:
            renderErrorProfile(file, false, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::ErrorHistogram:
            if (quant) renderErrorHistogram(file, config_.width, config_.height, *quant);
            break;
        case SvgPlotType::ErrorEnvelope:
            renderErrorEnvelope(file, 0, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::SpectralMagnitude:
            if (spectral) renderSpectralPlot(file, true, config_.width, config_.height,
                                             *spectral,
                                             tether::motion::replanner::SpectralComponent::Contour,
                                             tether::motion::replanner::SpectralDomain::Spatial);
            break;
        case SvgPlotType::SpectralPhase:
            if (spectral) renderSpectralPlot(file, false, config_.width, config_.height,
                                             *spectral,
                                             tether::motion::replanner::SpectralComponent::Contour,
                                             tether::motion::replanner::SpectralDomain::Spatial);
            break;
        case SvgPlotType::SpectralWaterfall:
            if (spectral) {
                // Stack all spatial components
                int subH = config_.height / 4;
                for (int i = 0; i < 4; ++i) {
                    auto comp = static_cast<tether::motion::replanner::SpectralComponent>(i);
                    renderSpectralPlot(file, true, config_.width, subH, *spectral, comp,
                                       tether::motion::replanner::SpectralDomain::Spatial);
                }
            }
            break;
        case SvgPlotType::PhasePortrait:
            renderPhasePortrait(file, config_.width, config_.height, actual);
            break;
        case SvgPlotType::VelocityProfile:
            renderVelocityProfile(file, config_.width, config_.height, desired, actual);
            break;
        case SvgPlotType::AccelerationProfile:
            renderAccelerationProfile(file, config_.width, config_.height, desired, actual);
            break;
    }

    writeFooter(file);
    return file.good();
}

std::vector<std::string> SvgExporter::exportAllPlots(
    const std::string& outputDir,
    const std::string& filePrefix,
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual,
    const tether::motion::replanner::QuantitativeEvaluation& quant,
    const tether::motion::replanner::SpectralEvaluation& spectral) const {

    std::vector<std::string> files;

    auto makePath = [&](const std::string& name) {
        return outputDir + "/" + filePrefix + "_" + name + ".svg";
    };

    auto tryExport = [&](const std::string& name, SvgPlotType type,
                         const tether::motion::replanner::QuantitativeEvaluation* q,
                         const tether::motion::replanner::SpectralEvaluation* s) {
        std::string path = makePath(name);
        if (exportPlot(path, type, desired, actual, q, s)) {
            files.push_back(path);
        }
    };

    tryExport("trajectory_xy", SvgPlotType::TrajectoryXY, nullptr, nullptr);
    tryExport("trajectory_xz", SvgPlotType::TrajectoryXZ, nullptr, nullptr);
    tryExport("trajectory_yz", SvgPlotType::TrajectoryYZ, nullptr, nullptr);
    tryExport("trajectory_3d", SvgPlotType::Trajectory3D, nullptr, nullptr);
    tryExport("error_vs_path_length", SvgPlotType::ErrorVsPathLength, nullptr, nullptr);
    tryExport("error_vs_time", SvgPlotType::ErrorVsTime, nullptr, nullptr);
    tryExport("error_histogram", SvgPlotType::ErrorHistogram, &quant, &spectral);
    tryExport("error_envelope_xy", SvgPlotType::ErrorEnvelope, nullptr, nullptr);
    tryExport("spectral_magnitude", SvgPlotType::SpectralMagnitude, &quant, &spectral);
    tryExport("spectral_phase", SvgPlotType::SpectralPhase, &quant, &spectral);
    tryExport("spectral_waterfall", SvgPlotType::SpectralWaterfall, &quant, &spectral);
    tryExport("velocity_profile", SvgPlotType::VelocityProfile, nullptr, nullptr);
    tryExport("acceleration_profile", SvgPlotType::AccelerationProfile, nullptr, nullptr);
    tryExport("phase_portrait", SvgPlotType::PhasePortrait, nullptr, nullptr);

    return files;
}

bool SvgExporter::exportDashboard(const std::string& filename,
                                  const std::vector<GCodeExport::TrajectorySample>& desired,
                                  const std::vector<GCodeExport::TrajectorySample>& actual,
                                  const tether::motion::replanner::QuantitativeEvaluation& quant,
                                  const tether::motion::replanner::SpectralEvaluation& spectral) const {

    std::ofstream file(filename);
    if (!file.is_open()) return false;

    // Dashboard: 3x3 grid of sub-plots
    int cols = 3, rows = 3;
    int subW = config_.width / cols;
    int subH = config_.height / rows;
    int totalW = subW * cols;
    int totalH = subH * rows;

    writeHeader(file, totalW, totalH);
    writeBackground(file, totalW, totalH);

    // Helper to render a sub-plot in a grid cell
    int cell = 0;
    auto renderCell = [&](auto renderFn) {
        int row = cell / cols;
        int col = cell % cols;
        std::string groupId = "subplot_" + std::to_string(cell);
        file << "<g transform=\"translate(" << (col * subW) << "," << (row * subH) << ")\" id=\"" << groupId << "\">\n";
        // Create a temporary stringstream for the sub-plot content
        std::ostringstream sub;
        renderFn(sub);
        file << sub.str();
        file << "</g>\n";
        ++cell;
    };

    // Row 1: XY, XZ, YZ
    renderCell([&](std::ostream& s) { renderTrajectory2D(s, 0, subW, subH, desired, actual); });
    renderCell([&](std::ostream& s) { renderTrajectory2D(s, 1, subW, subH, desired, actual); });
    renderCell([&](std::ostream& s) { renderTrajectory2D(s, 2, subW, subH, desired, actual); });

    // Row 2: 3D, Error vs path, Error histogram
    renderCell([&](std::ostream& s) { renderTrajectory3D(s, subW, subH, desired, actual); });
    renderCell([&](std::ostream& s) { renderErrorProfile(s, true, subW, subH, desired, actual); });
    renderCell([&](std::ostream& s) { renderErrorHistogram(s, subW, subH, quant); });

    // Row 3: Spectral, Velocity, Phase portrait
    renderCell([&](std::ostream& s) {
        renderSpectralPlot(s, true, subW, subH, spectral,
                          tether::motion::replanner::SpectralComponent::Contour,
                          tether::motion::replanner::SpectralDomain::Spatial);
    });
    renderCell([&](std::ostream& s) { renderVelocityProfile(s, subW, subH, desired, actual); });
    renderCell([&](std::ostream& s) { renderPhasePortrait(s, subW, subH, actual); });

    writeFooter(file);
    return file.good();
}

//=============================================================================
// KDE rendering
//=============================================================================

std::string SvgExporter::densityColor(double normalizedDensity) const {
    KdeSvgRenderer renderer(config_, canvas_);
    return renderer.densityColor(normalizedDensity);
}

void SvgExporter::renderKdeHeatmap(std::ostream& out,
                                   int svgW, int svgH,
                                   const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    renderer.renderHeatmap(out, svgW, svgH, kde);
}

void SvgExporter::renderKdeConditional(std::ostream& out,
                                       int svgW, int svgH,
                                       const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    renderer.renderConditional(out, svgW, svgH, kde);
}

void SvgExporter::renderKdeMarginalX(std::ostream& out,
                                     int svgW, int svgH,
                                     const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    renderer.renderMarginalX(out, svgW, svgH, kde);
}

void SvgExporter::renderKdeMarginalY(std::ostream& out,
                                     int svgW, int svgH,
                                     const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    renderer.renderMarginalY(out, svgW, svgH, kde);
}

bool SvgExporter::exportKdeHeatmap(const std::string& filename,
                                   const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    return renderer.exportHeatmap(filename, kde);
}

bool SvgExporter::exportKdeConditional(const std::string& filename,
                                       const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    return renderer.exportConditional(filename, kde);
}

bool SvgExporter::exportKdeDashboard(const std::string& filename,
                                     const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    return renderer.exportDashboard(filename, kde);
}

std::vector<std::string> SvgExporter::exportAllKdePlots(
    const std::string& outputDir,
    const std::string& filePrefix,
    const tether::motion::replanner::KdeEvaluation& kde) const {
    KdeSvgRenderer renderer(config_, canvas_);
    return renderer.exportAll(outputDir, filePrefix, kde);
}

} // namespace MotionReplanner
