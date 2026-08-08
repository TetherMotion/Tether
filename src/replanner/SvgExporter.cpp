/**
 * @file SvgExporter.cpp
 * @brief SVG vector graphics export implementation.
 */

#include "tether/motion_replanner/SvgExporter.hpp"

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
    using namespace tether::motion::replanner;
    auto [r, g, b] = KdeDerivativeAnalyzer::colormapColor(config_.kdeColormap, normalizedDensity);
    std::ostringstream ss;
    ss << "rgb(" << r << "," << g << "," << b << ")";
    return ss.str();
}

void SvgExporter::renderKdeHeatmap(std::ostream& out,
                                   int svgW, int svgH,
                                   const tether::motion::replanner::KdeEvaluation& kde) const {
    using namespace tether::motion::replanner;

    if (kde.grid.density.empty() || kde.grid.xBins.empty() || kde.grid.yBins.empty()) {
        renderTitle(out, "KDE Heatmap (no data)", svgW, config_.margin);
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
    writeBackground(out, svgW, svgH);

    // Title
    std::string title = std::format("KDE: {} vs {}",
                                    toString(kde.derivativeAxis),
                                    toString(kde.deviationAxis));
    renderTitle(out, title, svgW, margin);

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
            writeRect(out, px, py, dx + 0.5, dy + 0.5, color);
        }
    }

    // Axes
    AxisBounds bounds;
    bounds.minX = grid.xBins.front();
    bounds.maxX = grid.xBins.back();
    bounds.minY = grid.yBins.front();
    bounds.maxY = grid.yBins.back();

    // Draw axes around the heatmap
    // X axis (bottom)
    writeLine(out, plotX, plotY + plotH, plotX + plotW, plotY + plotH,
              config_.axisColor, config_.axisLineWidth);
    // Y axis (left)
    writeLine(out, plotX, plotY, plotX, plotY + plotH,
              config_.axisColor, config_.axisLineWidth);

    // Axis labels
    std::string xLabel = std::format("{} ({})", toString(kde.derivativeAxis),
                                      unitString(kde.derivativeAxis));
    std::string yLabel = std::format("{} ({})", toString(kde.deviationAxis),
                                      unitString(kde.deviationAxis));
    writeText(out, plotX + plotW / 2.0, plotY + plotH + 35, xLabel,
              config_.fontSize, config_.textColor, "middle");
    // Y label rotated
    out << "<text x=\"" << fmt(plotX - 40) << "\" y=\""
        << fmt(plotY + plotH / 2.0) << "\""
        << " font-family=\"sans-serif\" font-size=\"" << config_.fontSize << "\""
        << " fill=\"" << config_.textColor << "\""
        << " text-anchor=\"middle\""
        << " transform=\"rotate(-90 " << fmt(plotX - 40) << " "
        << fmt(plotY + plotH / 2.0) << ")\""
        << ">" << yLabel << "</text>\n";

    // Tick labels (X)
    int numXTicks = 5;
    for (int i = 0; i <= numXTicks; ++i) {
        double val = bounds.minX + (bounds.maxX - bounds.minX) * static_cast<double>(i) / numXTicks;
        double px = plotX + static_cast<double>(i) * plotW / numXTicks;
        writeLine(out, px, plotY + plotH, px, plotY + plotH + 5,
                  config_.axisColor, config_.axisLineWidth);
        writeText(out, px, plotY + plotH + 18, std::format("{:.2g}", val),
                  config_.fontSize - 2, config_.textColor, "middle");
    }
    // Tick labels (Y)
    int numYTicks = 5;
    for (int i = 0; i <= numYTicks; ++i) {
        double val = bounds.minY + (bounds.maxY - bounds.minY) * static_cast<double>(i) / numYTicks;
        double py = plotY + plotH - static_cast<double>(i) * plotH / numYTicks;
        writeLine(out, plotX - 5, py, plotX, py, config_.axisColor, config_.axisLineWidth);
        writeText(out, plotX - 8, py + 4, std::format("{:.2g}", val),
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
            writePolyline(out, svgPts, "#FFFFFF", 2.0);
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
            out << "<circle cx=\"" << fmt(px) << "\" cy=\"" << fmt(py)
                << "\" r=\"1.5\" fill=\"black\" fill-opacity=\"" << fmt(alpha) << "\"/>\n";
        }
    }

    // Colorbar (right side)
    int cbX = plotX + plotW + 20;
    int cbW = 15;
    int cbH = plotH;
    for (int i = 0; i < cbH; ++i) {
        double t = 1.0 - static_cast<double>(i) / static_cast<double>(cbH - 1);
        std::string color = densityColor(t);
        writeRect(out, cbX, plotY + i, cbW, 1.0 + 0.5, color);
    }
    writeLine(out, cbX, plotY, cbX, plotY + cbH, config_.axisColor, config_.axisLineWidth);
    writeLine(out, cbX + cbW, plotY, cbX + cbW, plotY + cbH, config_.axisColor, config_.axisLineWidth);

    // Colorbar labels
    for (int i = 0; i <= 4; ++i) {
        double t = static_cast<double>(i) / 4.0;
        double val = config_.kdeLogScale ? minD * std::pow(maxD / minD, t) : maxD * t;
        double py = plotY + cbH - t * cbH;
        writeText(out, cbX + cbW + 5, py + 4, std::format("{:.2g}", val),
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
        writePolyline(out, margXPts, config_.desiredColor, 1.0);

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
        writePolyline(out, margYPts, config_.desiredColor, 1.0);
    }

    // Info text
    std::string info = std::format("n={} | h_x={:.3g} h_y={:.3g} | MI={:.3f} bits | r={:.3f} | η²={:.3f}",
                                   kde.grid.sampleCount,
                                   kde.grid.bandwidthX, kde.grid.bandwidthY,
                                   kde.mutualInformation,
                                   kde.pearsonCorrelation,
                                   kde.correlationRatio);
    writeText(out, margin, svgH - 15, info, config_.fontSize - 1, config_.textColor, "start");
}

void SvgExporter::renderKdeConditional(std::ostream& out,
                                       int svgW, int svgH,
                                       const tether::motion::replanner::KdeEvaluation& kde) const {
    using namespace tether::motion::replanner;

    if (kde.conditional.empty()) {
        renderTitle(out, "Conditional Stats (no data)", svgW, config_.margin);
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

    AxisBounds bounds;
    bounds.minX = minX;
    bounds.maxX = maxX;
    bounds.minY = minY;
    bounds.maxY = maxY * 1.1;

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin,
               std::format("{} ({})", toString(kde.derivativeAxis), unitString(kde.derivativeAxis)),
               std::format("{} ({})", toString(kde.deviationAxis), unitString(kde.deviationAxis)));

    std::string title = std::format("Conditional {} vs {} (quantile bands)",
                                    toString(kde.deviationAxis),
                                    toString(kde.derivativeAxis));
    renderTitle(out, title, svgW, config_.margin);

    // Helper to transform data to SVG
    auto tf = [&](double x, double y) -> std::pair<double, double> {
        return transform(x, y, bounds, svgW, svgH, config_.margin);
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
            out << fmt(polygon[i].first) << "," << fmt(polygon[i].second);
        }
        out << "\" fill=\"" << color << "\" fill-opacity=\"" << fmt(alpha)
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
        if (pts.size() > 1) writePolyline(out, pts, color, width);
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
        renderLegend(out, svgW, svgH, entries);
    }
}

void SvgExporter::renderKdeMarginalX(std::ostream& out,
                                     int svgW, int svgH,
                                     const tether::motion::replanner::KdeEvaluation& kde) const {
    using namespace tether::motion::replanner;
    if (kde.grid.xBins.empty()) return;

    auto margX = kde.grid.marginalX();
    double maxMarg = *std::max_element(margX.begin(), margX.end());
    if (maxMarg < 1e-15) maxMarg = 1.0;

    AxisBounds bounds;
    bounds.minX = kde.grid.xBins.front();
    bounds.maxX = kde.grid.xBins.back();
    bounds.minY = 0;
    bounds.maxY = maxMarg * 1.1;

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin,
               std::format("{} ({})", toString(kde.derivativeAxis), unitString(kde.derivativeAxis)),
               "Density");

    renderTitle(out, std::format("Marginal: {}", toString(kde.derivativeAxis)),
                svgW, config_.margin);

    std::vector<std::pair<double, double>> pts;
    for (std::size_t i = 0; i < kde.grid.xBins.size(); ++i) {
        auto [px, py] = transform(kde.grid.xBins[i], margX[i], bounds, svgW, svgH, config_.margin);
        pts.emplace_back(px, py);
    }
    writePolyline(out, pts, config_.desiredColor, config_.lineWidth);
}

void SvgExporter::renderKdeMarginalY(std::ostream& out,
                                     int svgW, int svgH,
                                     const tether::motion::replanner::KdeEvaluation& kde) const {
    using namespace tether::motion::replanner;
    if (kde.grid.yBins.empty()) return;

    auto margY = kde.grid.marginalY();
    double maxMarg = *std::max_element(margY.begin(), margY.end());
    if (maxMarg < 1e-15) maxMarg = 1.0;

    AxisBounds bounds;
    bounds.minX = 0;
    bounds.maxX = maxMarg * 1.1;
    bounds.minY = kde.grid.yBins.front();
    bounds.maxY = kde.grid.yBins.back();

    renderGrid(out, bounds, svgW, svgH, config_.margin);
    renderAxes(out, bounds, svgW, svgH, config_.margin,
               "Density",
               std::format("{} ({})", toString(kde.deviationAxis), unitString(kde.deviationAxis)));

    renderTitle(out, std::format("Marginal: {}", toString(kde.deviationAxis)),
                svgW, config_.margin);

    std::vector<std::pair<double, double>> pts;
    for (std::size_t i = 0; i < kde.grid.yBins.size(); ++i) {
        auto [px, py] = transform(margY[i], kde.grid.yBins[i], bounds, svgW, svgH, config_.margin);
        pts.emplace_back(px, py);
    }
    writePolyline(out, pts, config_.desiredColor, config_.lineWidth);
}

bool SvgExporter::exportKdeHeatmap(const std::string& filename,
                                   const tether::motion::replanner::KdeEvaluation& kde) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    writeHeader(file, config_.width, config_.height);
    renderKdeHeatmap(file, config_.width, config_.height, kde);
    writeFooter(file);
    return file.good();
}

bool SvgExporter::exportKdeConditional(const std::string& filename,
                                       const tether::motion::replanner::KdeEvaluation& kde) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    writeHeader(file, config_.width, config_.height);
    renderKdeConditional(file, config_.width, config_.height, kde);
    writeFooter(file);
    return file.good();
}

bool SvgExporter::exportKdeDashboard(const std::string& filename,
                                     const tether::motion::replanner::KdeEvaluation& kde) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;

    // Dashboard: 2x2 grid
    int totalW = config_.width;
    int totalH = config_.height;
    // Make it wider for dashboard
    int dashW = std::max(totalW, 1400);
    int dashH = std::max(totalH, 1000);

    writeHeader(file, dashW, dashH);
    writeBackground(file, dashW, dashH);

    renderTitle(file, std::format("KDE Dashboard: {} vs {}",
                                  tether::motion::replanner::toString(kde.derivativeAxis),
                                  tether::motion::replanner::toString(kde.deviationAxis)),
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

    // Use SVG groups with transforms for each cell
    auto renderCell = [&](int row, int col, auto renderFn) {
        auto [x, y] = cellPos(row, col);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        // Render into a sub-stream then embed
        std::ostringstream ss;
        renderFn(ss);
        file << ss.str();
        file << "</g>\n";
    };

    // Row 0: Heatmap (spans both columns), Marginal Y
    {
        auto [x, y] = cellPos(0, 0);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderKdeHeatmap(file, subW, subH, kde);
        file << "</g>\n";
    }
    {
        auto [x, y] = cellPos(0, 1);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderKdeMarginalY(file, subW, subH, kde);
        file << "</g>\n";
    }

    // Row 1: Marginal X, Conditional
    {
        auto [x, y] = cellPos(1, 0);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderKdeMarginalX(file, subW, subH, kde);
        file << "</g>\n";
    }
    {
        auto [x, y] = cellPos(1, 1);
        file << "<g transform=\"translate(" << x << "," << y << ")\">\n";
        renderKdeConditional(file, subW, subH, kde);
        file << "</g>\n";
    }

    writeFooter(file);
    return file.good();
}

std::vector<std::string> SvgExporter::exportAllKdePlots(
    const std::string& outputDir,
    const std::string& filePrefix,
    const tether::motion::replanner::KdeEvaluation& kde) const {

    std::vector<std::string> files;

    auto tryExport = [&](const std::string& name, auto fn) {
        std::string path = outputDir + "/" + filePrefix + "_" + name + ".svg";
        if (fn(path)) files.push_back(path);
    };

    tryExport("kde_heatmap", [&](const std::string& p) {
        return exportKdeHeatmap(p, kde);
    });
    tryExport("kde_conditional", [&](const std::string& p) {
        return exportKdeConditional(p, kde);
    });
    tryExport("kde_marginal_x", [&](const std::string& p) {
        std::ofstream f(p);
        if (!f.is_open()) return false;
        writeHeader(f, config_.width, config_.height);
        renderKdeMarginalX(f, config_.width, config_.height, kde);
        writeFooter(f);
        return f.good();
    });
    tryExport("kde_marginal_y", [&](const std::string& p) {
        std::ofstream f(p);
        if (!f.is_open()) return false;
        writeHeader(f, config_.width, config_.height);
        renderKdeMarginalY(f, config_.width, config_.height, kde);
        writeFooter(f);
        return f.good();
    });
    tryExport("kde_dashboard", [&](const std::string& p) {
        return exportKdeDashboard(p, kde);
    });

    return files;
}

} // namespace MotionReplanner
