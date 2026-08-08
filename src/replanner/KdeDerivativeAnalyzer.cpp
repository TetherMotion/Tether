/**
 * @file KdeDerivativeAnalyzer.cpp
 * @brief Implementation of kernel density estimation for derivative-vs-deviation.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"
#include "tether/motion_replanner/DependenceAnalyzer.hpp"
#include "tether/motion_replanner/BandwidthSelector.hpp"
#include "tether/motion_replanner/KdeStatistics.hpp"
#include "tether/motion_planner/geometry/Vector.hpp" // RVec

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numeric>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tether::motion::replanner {

//=============================================================================
// Anonymous namespace — internal helpers
//=============================================================================
namespace {

/// Compute mean of a vector.
double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

/// Compute standard deviation (sample, ddof=1).
double stdDev(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 2) return 0.0;
    double m = mean(v);
    double sq = 0.0;
    for (double x : v) sq += (x - m) * (x - m);
    return std::sqrt(sq / static_cast<double>(n - 1));
}

/// Compute the q-th quantile of a sorted vector.
double quantileSorted(const std::vector<double>& sorted, double q) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted[0];
    double pos = q * static_cast<double>(sorted.size() - 1);
    auto lo = static_cast<std::size_t>(std::floor(pos));
    auto hi = static_cast<std::size_t>(std::ceil(pos));
    if (lo == hi) return sorted[lo];
    double frac = pos - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

/// Compute skewness (population, g_1).
double skewness(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 3) return 0.0;
    double m = mean(v);
    double s = stdDev(v);
    if (s < 1e-15) return 0.0;
    double sk = 0.0;
    for (double x : v) sk += std::pow((x - m) / s, 3.0);
    return sk / static_cast<double>(n);
}

/// Compute excess kurtosis (population, g_2 - 3).
double kurtosis(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 4) return 0.0;
    double m = mean(v);
    double s = stdDev(v);
    if (s < 1e-15) return 0.0;
    double k = 0.0;
    for (double x : v) k += std::pow((x - m) / s, 4.0);
    return k / static_cast<double>(n) - 3.0;
}

/// Rank a vector (average ranks for ties).
std::vector<double> rank(const std::vector<double>& v) {
    auto n = v.size();
    std::vector<std::pair<double, std::size_t>> indexed(n);
    for (std::size_t i = 0; i < n; ++i) {
        indexed[i] = {v[i], i};
    }
    std::sort(indexed.begin(), indexed.end());
    std::vector<double> ranks(n);
    std::size_t i = 0;
    while (i < n) {
        std::size_t j = i;
        while (j + 1 < n && indexed[j + 1].first == indexed[i].first) ++j;
        double avgRank = static_cast<double>(i + j) / 2.0 + 1.0;
        for (std::size_t k = i; k <= j; ++k) {
            ranks[indexed[k].second] = avgRank;
        }
        i = j + 1;
    }
    return ranks;
}

/// Compute the CDF of the standard normal distribution.
double normalCdf(double x) {
    return 0.5 * (1.0 + std::erf(x / std::sqrt(2.0)));
}

/// Compute the PDF of the standard normal distribution.
double normalPdf(double x) {
    return std::exp(-0.5 * x * x) / std::sqrt(2.0 * M_PI);
}

/// Linear interpolation.
double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

/// Log-spaced linspace.
std::vector<double> logLinspace(double min, double max, std::size_t n) {
    std::vector<double> v(n);
    double lmin = std::log(min);
    double lmax = std::log(max);
    for (std::size_t i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(n - 1);
        v[i] = std::exp(lerp(lmin, lmax, t));
    }
    return v;
}

/// Linear linspace.
std::vector<double> linspace(double min, double max, std::size_t n) {
    std::vector<double> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(n - 1);
        v[i] = lerp(min, max, t);
    }
    return v;
}

} // anonymous namespace

//=============================================================================
// KdeGrid methods
//=============================================================================

double KdeGrid::maxDensity() const {
    if (density.empty()) return 0.0;
    return *std::max_element(density.begin(), density.end());
}

double KdeGrid::totalMass() const {
    if (density.empty() || xBins.empty() || yBins.empty()) return 0.0;
    double dx = (xBins.size() > 1) ? (xBins.back() - xBins.front()) / static_cast<double>(xBins.size() - 1) : 1.0;
    double dy = (yBins.size() > 1) ? (yBins.back() - yBins.front()) / static_cast<double>(yBins.size() - 1) : 1.0;
    double mass = 0.0;
    for (double d : density) mass += d;
    return mass * dx * dy;
}

std::vector<double> KdeGrid::marginalX() const {
    std::vector<double> m(xBins.size(), 0.0);
    if (yBins.empty()) return m;
    double dy = (yBins.size() > 1) ? (yBins.back() - yBins.front()) / static_cast<double>(yBins.size() - 1) : 1.0;
    for (std::size_t ix = 0; ix < xBins.size(); ++ix) {
        for (std::size_t iy = 0; iy < yBins.size(); ++iy) {
            m[ix] += at(ix, iy) * dy;
        }
    }
    return m;
}

std::vector<double> KdeGrid::marginalY() const {
    std::vector<double> m(yBins.size(), 0.0);
    if (xBins.empty()) return m;
    double dx = (xBins.size() > 1) ? (xBins.back() - xBins.front()) / static_cast<double>(xBins.size() - 1) : 1.0;
    for (std::size_t iy = 0; iy < yBins.size(); ++iy) {
        for (std::size_t ix = 0; ix < xBins.size(); ++ix) {
            m[iy] += at(ix, iy) * dx;
        }
    }
    return m;
}

//=============================================================================
// Utility functions
//=============================================================================

std::string toString(DerivativeAxis axis) {
    switch (axis) {
        case DerivativeAxis::Velocity:       return "Velocity";
        case DerivativeAxis::Acceleration:   return "Acceleration";
        case DerivativeAxis::Jerk:           return "Jerk";
        case DerivativeAxis::Curvature:      return "Curvature";
        case DerivativeAxis::FeedRate:       return "FeedRate";
        case DerivativeAxis::ArcLength:      return "ArcLength";
        case DerivativeAxis::Time:           return "Time";
    }
    return "Unknown";
}

std::string toString(DeviationAxis axis) {
    switch (axis) {
        case DeviationAxis::ContourError:      return "ContourError";
        case DeviationAxis::LagError:          return "LagError";
        case DeviationAxis::CombinedError:     return "CombinedError";
        case DeviationAxis::BinormalError:     return "BinormalError";
        case DeviationAxis::TrackingError:     return "TrackingError";
        case DeviationAxis::VelocityError:     return "VelocityError";
        case DeviationAxis::AccelerationError: return "AccelerationError";
    }
    return "Unknown";
}

std::string toString(KernelType kernel) {
    switch (kernel) {
        case KernelType::Gaussian:     return "Gaussian";
        case KernelType::Epanechnikov: return "Epanechnikov";
        case KernelType::Uniform:      return "Uniform";
        case KernelType::Triangular:   return "Triangular";
        case KernelType::Quartic:      return "Quartic";
        case KernelType::Cosine:       return "Cosine";
    }
    return "Unknown";
}

std::string toString(BandwidthMethod method) {
    switch (method) {
        case BandwidthMethod::Silverman:      return "Silverman";
        case BandwidthMethod::Scott:          return "Scott";
        case BandwidthMethod::ISJ:            return "ISJ";
        case BandwidthMethod::Fixed:          return "Fixed";
        case BandwidthMethod::LeastSquaresCV: return "LeastSquaresCV";
        case BandwidthMethod::LikelihoodCV:   return "LikelihoodCV";
    }
    return "Unknown";
}

std::string toString(KdeColormap cmap) {
    switch (cmap) {
        case KdeColormap::Viridis:   return "Viridis";
        case KdeColormap::Inferno:   return "Inferno";
        case KdeColormap::Plasma:    return "Plasma";
        case KdeColormap::Magma:     return "Magma";
        case KdeColormap::Jet:       return "Jet";
        case KdeColormap::Hot:       return "Hot";
        case KdeColormap::Cool:      return "Cool";
        case KdeColormap::Grayscale: return "Grayscale";
        case KdeColormap::BlueRed:   return "BlueRed";
    }
    return "Unknown";
}

std::string unitString(DerivativeAxis axis) {
    switch (axis) {
        case DerivativeAxis::Velocity:     return "mm/s";
        case DerivativeAxis::Acceleration: return "mm/s^2";
        case DerivativeAxis::Jerk:         return "mm/s^3";
        case DerivativeAxis::Curvature:    return "1/mm";
        case DerivativeAxis::FeedRate:     return "mm/s";
        case DerivativeAxis::ArcLength:    return "mm";
        case DerivativeAxis::Time:         return "s";
    }
    return "";
}

std::string unitString(DeviationAxis axis) {
    switch (axis) {
        case DeviationAxis::ContourError:
        case DeviationAxis::LagError:
        case DeviationAxis::CombinedError:
        case DeviationAxis::BinormalError:
        case DeviationAxis::TrackingError:
            return "mm";
        case DeviationAxis::VelocityError:
            return "mm/s";
        case DeviationAxis::AccelerationError:
            return "mm/s^2";
    }
    return "";
}

//=============================================================================
// KdeDerivativeAnalyzer implementation
//=============================================================================

KdeDerivativeAnalyzer::KdeDerivativeAnalyzer(KdeConfig config)
    : config_(std::move(config)) {}

double KdeDerivativeAnalyzer::kernelValue(KernelType kernel, double u) const {
    double a = std::abs(u);
    switch (kernel) {
        case KernelType::Gaussian:
            return std::exp(-0.5 * u * u) / std::sqrt(2.0 * M_PI);
        case KernelType::Epanechnikov:
            return (a <= 1.0) ? 0.75 * (1.0 - u * u) : 0.0;
        case KernelType::Uniform:
            return (a <= 1.0) ? 0.5 : 0.0;
        case KernelType::Triangular:
            return (a <= 1.0) ? (1.0 - a) : 0.0;
        case KernelType::Quartic:
            return (a <= 1.0) ? (15.0 / 16.0) * std::pow(1.0 - u * u, 2.0) : 0.0;
        case KernelType::Cosine:
            return (a <= 1.0) ? (M_PI / 4.0) * std::cos(M_PI * u / 2.0) : 0.0;
    }
    return 0.0;
}

std::vector<int> KdeDerivativeAnalyzer::detectActiveAxes(
    const std::vector<GCodeExport::TrajectorySample>& desired) const {
    if (!config_.activeAxes.empty()) return config_.activeAxes;

    std::array<double, 9> minPos{}, maxPos{};
    for (double& v : minPos) v = std::numeric_limits<double>::max();
    for (double& v : maxPos) v = std::numeric_limits<double>::lowest();

    for (const auto& s : desired) {
        for (int i = 0; i < 9; ++i) {
            minPos[i] = std::min(minPos[i], s.position[i]);
            maxPos[i] = std::max(maxPos[i], s.position[i]);
        }
    }

    std::vector<int> active;
    for (int i = 0; i < 9; ++i) {
        if (maxPos[i] - minPos[i] > 1e-9) active.push_back(i);
    }
    if (active.empty()) active = {0};
    return active;
}

double KdeDerivativeAnalyzer::computeDerivative(
    const GCodeExport::TrajectorySample& desired,
    const GCodeExport::TrajectorySample& actual,
    std::size_t index,
    const std::vector<GCodeExport::TrajectorySample>& allActual,
    DerivativeAxis type) const {

    switch (type) {
        case DerivativeAxis::Velocity:
            return std::sqrt(actual.velocity[0] * actual.velocity[0] +
                             actual.velocity[1] * actual.velocity[1] +
                             actual.velocity[2] * actual.velocity[2]);
        case DerivativeAxis::Acceleration:
            return std::sqrt(actual.acceleration[0] * actual.acceleration[0] +
                             actual.acceleration[1] * actual.acceleration[1] +
                             actual.acceleration[2] * actual.acceleration[2]);
        case DerivativeAxis::Jerk:
            return std::sqrt(actual.jerk[0] * actual.jerk[0] +
                             actual.jerk[1] * actual.jerk[1] +
                             actual.jerk[2] * actual.jerk[2]);
        case DerivativeAxis::Curvature:
            return std::abs(desired.curvature);
        case DerivativeAxis::FeedRate:
            return std::sqrt(desired.velocity[0] * desired.velocity[0] +
                             desired.velocity[1] * desired.velocity[1] +
                             desired.velocity[2] * desired.velocity[2]);
        case DerivativeAxis::ArcLength:
            return desired.pathPosition;
        case DerivativeAxis::Time:
            return desired.time;
    }
    return 0.0;
}

double KdeDerivativeAnalyzer::computeDeviation(
    const GCodeExport::TrajectorySample& desired,
    const GCodeExport::TrajectorySample& actual,
    const PiecewiseNurbsPath& path,
    double sPath,
    DeviationAxis type) const {

    std::size_t dim = path.dim();
    auto axes = detectActiveAxes({desired, desired}); // quick detect
    // Use first dim axes
    std::vector<int> ax;
    for (std::size_t i = 0; i < dim && i < axes.size(); ++i) ax.push_back(axes[i]);
    if (ax.size() < dim) {
        for (int i = static_cast<int>(ax.size()); i < static_cast<int>(dim); ++i)
            ax.push_back(std::min(i, 2));
    }

    switch (type) {
        case DeviationAxis::ContourError: {
            RVec actualPos = RVec::zero(dim);
            for (std::size_t d = 0; d < dim; ++d)
                actualPos[d] = actual.position[ax[d]];
            if (config_.useCertifiedContourError) {
                try {
                    auto ce = computeCertifiedContourError(path, actualPos, sPath);
                    return ce.contourError;
                } catch (...) {
                    RVec desiredPos = path.evaluatePosition(sPath);
                    return actualPos.distanceTo(desiredPos);
                }
            } else {
                RVec desiredPos = path.evaluatePosition(sPath);
                double combined = actualPos.distanceTo(desiredPos);
                // Decompose
                double vmag = std::sqrt(desired.velocity[0] * desired.velocity[0] +
                                        desired.velocity[1] * desired.velocity[1] +
                                        desired.velocity[2] * desired.velocity[2]);
                if (vmag > 1e-9) {
                    double tx = desired.velocity[0] / vmag;
                    double ty = desired.velocity[1] / vmag;
                    double tz = desired.velocity[2] / vmag;
                    double ex = actual.position[0] - desired.position[0];
                    double ey = actual.position[1] - desired.position[1];
                    double ez = actual.position[2] - desired.position[2];
                    double lag = ex * tx + ey * ty + ez * tz;
                    double cx = ex - lag * tx, cy = ey - lag * ty, cz = ez - lag * tz;
                    return std::sqrt(cx * cx + cy * cy + cz * cz);
                }
                return combined;
            }
        }
        case DeviationAxis::LagError: {
            RVec actualPos = RVec::zero(dim);
            for (std::size_t d = 0; d < dim; ++d)
                actualPos[d] = actual.position[ax[d]];
            if (config_.useCertifiedContourError) {
                try {
                    auto ce = computeCertifiedContourError(path, actualPos, sPath);
                    return ce.lagError;
                } catch (...) {
                    return 0.0;
                }
            } else {
                double vmag = std::sqrt(desired.velocity[0] * desired.velocity[0] +
                                        desired.velocity[1] * desired.velocity[1] +
                                        desired.velocity[2] * desired.velocity[2]);
                if (vmag > 1e-9) {
                    double tx = desired.velocity[0] / vmag;
                    double ty = desired.velocity[1] / vmag;
                    double tz = desired.velocity[2] / vmag;
                    double ex = actual.position[0] - desired.position[0];
                    double ey = actual.position[1] - desired.position[1];
                    double ez = actual.position[2] - desired.position[2];
                    return ex * tx + ey * ty + ez * tz;
                }
                return 0.0;
            }
        }
        case DeviationAxis::CombinedError: {
            double dx = actual.position[0] - desired.position[0];
            double dy = actual.position[1] - desired.position[1];
            double dz = actual.position[2] - desired.position[2];
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        case DeviationAxis::BinormalError: {
            // Out-of-plane component (Z for XY paths)
            return std::abs(actual.position[2] - desired.position[2]);
        }
        case DeviationAxis::TrackingError: {
            double dx = actual.position[0] - desired.position[0];
            double dy = actual.position[1] - desired.position[1];
            double dz = actual.position[2] - desired.position[2];
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        case DeviationAxis::VelocityError: {
            double dvx = actual.velocity[0] - desired.velocity[0];
            double dvy = actual.velocity[1] - desired.velocity[1];
            double dvz = actual.velocity[2] - desired.velocity[2];
            return std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
        }
        case DeviationAxis::AccelerationError: {
            double dax = actual.acceleration[0] - desired.acceleration[0];
            double day = actual.acceleration[1] - desired.acceleration[1];
            double daz = actual.acceleration[2] - desired.acceleration[2];
            return std::sqrt(dax * dax + day * day + daz * daz);
        }
    }
    return 0.0;
}

KdeDerivativeAnalyzer::SamplePairs KdeDerivativeAnalyzer::extractPairs(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) const {

    SamplePairs pairs;
    auto n = std::min(desired.size(), actual.size());
    if (n == 0) return pairs;

    // Build NURBS path from desired
    ConverterConfig convConfig;
    PiecewiseNurbsPath path = convertTrajectory(desired, convConfig);

    // Compute scaling factor
    double maxPathPos = 0.0;
    for (const auto& d : desired) {
        maxPathPos = std::max(maxPathPos, d.pathPosition);
    }
    double pathTotalLen = path.totalLength();
    double sScale = (maxPathPos > 1e-12) ? pathTotalLen / maxPathPos : 1.0;

    pairs.derivatives.reserve(n);
    pairs.deviations.reserve(n);
    pairs.arcLengths.reserve(n);
    pairs.times.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        double sPath = desired[i].pathPosition * sScale;
        double d = computeDerivative(desired[i], actual[i], i, actual, config_.derivativeAxis);
        double e = computeDeviation(desired[i], actual[i], path, sPath, config_.deviationAxis);
        pairs.derivatives.push_back(d);
        pairs.deviations.push_back(e);
        pairs.arcLengths.push_back(desired[i].pathPosition);
        pairs.times.push_back(desired[i].time);
    }

    return pairs;
}

double KdeDerivativeAnalyzer::computeBandwidth(
    const std::vector<double>& data,
    BandwidthMethod method,
    double fixedValue,
    double scale) const {
    return BandwidthSelector::compute(data, method, fixedValue, scale);
}

double KdeDerivativeAnalyzer::isjBandwidth(const std::vector<double>& data) const {
    return BandwidthSelector::isj(data);
}

double KdeDerivativeAnalyzer::lscvBandwidth(const std::vector<double>& data) const {
    return BandwidthSelector::lscv(data);
}

double KdeDerivativeAnalyzer::likelihoodCvBandwidth(const std::vector<double>& data) const {
    return BandwidthSelector::likelihoodCv(data);
}

KdeGrid KdeDerivativeAnalyzer::evaluateKde(
    const std::vector<double>& derivatives,
    const std::vector<double>& deviations,
    double hX, double hY,
    double xMin, double xMax,
    double yMin, double yMax,
    std::size_t nX, std::size_t nY,
    KernelType kernel) const {

    KdeGrid grid;
    auto n = derivatives.size();
    if (n == 0 || nX < 2 || nY < 2) return grid;

    // Generate bin centers
    grid.xBins = linspace(xMin, xMax, nX);
    grid.yBins = linspace(yMin, yMax, nY);
    grid.bandwidthX = hX;
    grid.bandwidthY = hY;
    grid.sampleCount = n;
    grid.density.assign(nX * nY, 0.0);

    double invN = 1.0 / static_cast<double>(n);
    double invHX = 1.0 / hX;
    double invHY = 1.0 / hY;

    // For compact kernels, determine the cutoff radius
    bool isGaussian = (kernel == KernelType::Gaussian);
    double cutoffX = isGaussian ? 5.0 * hX : hX;  // 5σ for Gaussian, 1h for compact
    double cutoffY = isGaussian ? 5.0 * hY : hY;

    for (std::size_t i = 0; i < n; ++i) {
        double dx = derivatives[i];
        double dy = deviations[i];

        // Find the range of bins that could be affected
        std::size_t ixStart = 0, ixEnd = nX;
        std::size_t iyStart = 0, iyEnd = nY;

        if (isGaussian) {
            // Find bins within 5σ
            for (; ixStart < nX && grid.xBins[ixStart] < dx - cutoffX; ++ixStart);
            for (; ixEnd > 0 && grid.xBins[ixEnd - 1] > dx + cutoffX; --ixEnd);
            for (; iyStart < nY && grid.yBins[iyStart] < dy - cutoffY; ++iyStart);
            for (; iyEnd > 0 && grid.yBins[iyEnd - 1] > dy + cutoffY; --iyEnd);
        }

        for (std::size_t iy = iyStart; iy < iyEnd; ++iy) {
            double uy = (grid.yBins[iy] - dy) * invHY;
            double ky = kernelValue(kernel, uy);
            if (ky == 0.0) continue;
            for (std::size_t ix = ixStart; ix < ixEnd; ++ix) {
                double ux = (grid.xBins[ix] - dx) * invHX;
                double kx = kernelValue(kernel, ux);
                grid.density[iy * nX + ix] += kx * ky * invN;
            }
        }
    }

    // Normalize by bandwidth to get a proper PDF
    double norm = invHX * invHY;
    for (double& d : grid.density) d *= norm;

    return grid;
}

KdeGrid KdeDerivativeAnalyzer::binnedKde(
    const std::vector<double>& derivatives,
    const std::vector<double>& deviations,
    double hX, double hY,
    double xMin, double xMax,
    double yMin, double yMax,
    std::size_t nX, std::size_t nY,
    KernelType kernel) const {

    // Binned KDE: histogram the data, then convolve with the kernel
    auto n = derivatives.size();
    std::size_t histSize = config_.binnedHistogramSize;

    // Build 2D histogram
    std::vector<double> hist(histSize * histSize, 0.0);
    double dxHist = (xMax - xMin) / static_cast<double>(histSize);
    double dyHist = (yMax - yMin) / static_cast<double>(histSize);
    if (dxHist < 1e-15) dxHist = 1.0;
    if (dyHist < 1e-15) dyHist = 1.0;

    for (std::size_t i = 0; i < n; ++i) {
        std::size_t ix = static_cast<std::size_t>((derivatives[i] - xMin) / dxHist);
        std::size_t iy = static_cast<std::size_t>((deviations[i] - yMin) / dyHist);
        if (ix >= histSize) ix = histSize - 1;
        if (iy >= histSize) iy = histSize - 1;
        hist[iy * histSize + ix] += 1.0;
    }

    // Convolve with kernel (separable for Gaussian)
    KdeGrid grid;
    grid.xBins = linspace(xMin, xMax, nX);
    grid.yBins = linspace(yMin, yMax, nY);
    grid.bandwidthX = hX;
    grid.bandwidthY = hY;
    grid.sampleCount = n;
    grid.density.assign(nX * nY, 0.0);

    bool isGaussian = (kernel == KernelType::Gaussian);
    double cutoffX = isGaussian ? 5.0 * hX : hX;
    double cutoffY = isGaussian ? 5.0 * hY : hY;
    int kernelRadiusX = static_cast<int>(cutoffX / dxHist) + 1;
    int kernelRadiusY = static_cast<int>(cutoffY / dyHist) + 1;

    // Precompute kernel weights
    std::vector<double> kernelX(2 * kernelRadiusX + 1);
    std::vector<double> kernelY(2 * kernelRadiusY + 1);
    for (int k = -kernelRadiusX; k <= kernelRadiusX; ++k) {
        double u = k * dxHist / hX;
        kernelX[k + kernelRadiusX] = kernelValue(kernel, u) / hX;
    }
    for (int k = -kernelRadiusY; k <= kernelRadiusY; ++k) {
        double u = k * dyHist / hY;
        kernelY[k + kernelRadiusY] = kernelValue(kernel, u) / hY;
    }

    // Separable convolution: first along X, then along Y
    std::vector<double> temp(histSize * histSize, 0.0);
    for (std::size_t iy = 0; iy < histSize; ++iy) {
        for (std::size_t ix = 0; ix < histSize; ++ix) {
            double val = hist[iy * histSize + ix];
            if (val == 0.0) continue;
            for (int k = -kernelRadiusX; k <= kernelRadiusX; ++k) {
                int jx = static_cast<int>(ix) + k;
                if (jx < 0 || jx >= static_cast<int>(histSize)) continue;
                temp[iy * histSize + jx] += val * kernelX[k + kernelRadiusX];
            }
        }
    }

    // Convolve along Y and resample to output grid
    for (std::size_t iy = 0; iy < histSize; ++iy) {
        for (std::size_t ix = 0; ix < histSize; ++ix) {
            double val = temp[iy * histSize + ix];
            if (val == 0.0) continue;
            for (int k = -kernelRadiusY; k <= kernelRadiusY; ++k) {
                int jy = static_cast<int>(iy) + k;
                if (jy < 0 || jy >= static_cast<int>(histSize)) continue;
                double histY = yMin + (jy + 0.5) * dyHist;
                // Find output bin
                std::size_t oy = static_cast<std::size_t>(
                    (histY - yMin) / (yMax - yMin) * static_cast<double>(nY));
                if (oy >= nY) continue;
                std::size_t ox = static_cast<std::size_t>(
                    (xMin + (ix + 0.5) * dxHist - xMin) / (xMax - xMin) * static_cast<double>(nX));
                if (ox >= nX) continue;
                grid.density[oy * nX + ox] += val * kernelY[k + kernelRadiusY];
            }
        }
    }

    // Normalize
    double invN = 1.0 / static_cast<double>(n);
    for (double& d : grid.density) d *= invN;

    return grid;
}

MarginalStats KdeDerivativeAnalyzer::computeMarginalStats(
    const std::vector<double>& data) const {
    return KdeStatistics::computeMarginalStats(data);
}

std::vector<ConditionalStats> KdeDerivativeAnalyzer::computeConditionalStats(
    const KdeGrid& grid,
    const std::vector<double>& quantileLevels,
    double minMass) const {
    return KdeStatistics::computeConditionalStats(grid, quantileLevels, minMass);
}

double KdeDerivativeAnalyzer::spearmanRank(
    const std::vector<double>& x,
    const std::vector<double>& y) const {
    return DependenceAnalyzer::spearmanRank(x, y);
}

double KdeDerivativeAnalyzer::kendallTau(
    const std::vector<double>& x,
    const std::vector<double>& y) const {
    return DependenceAnalyzer::kendallTau(x, y);
}

double KdeDerivativeAnalyzer::distanceCorrelation(
    const std::vector<double>& x,
    const std::vector<double>& y) const {
    return DependenceAnalyzer::distanceCorrelation(x, y);
}

double KdeDerivativeAnalyzer::mutualInformationFromGrid(const KdeGrid& grid) const {
    return DependenceAnalyzer::mutualInformationFromGrid(grid);
}

KdeDerivativeAnalyzer::GridEntropy KdeDerivativeAnalyzer::entropyFromGrid(const KdeGrid& grid) const {
    auto de = DependenceAnalyzer::entropyFromGrid(grid);
    return GridEntropy{de.joint, de.conditional, de.normalizedMI};
}

DependenceMetrics KdeDerivativeAnalyzer::computeDependence(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const KdeGrid& grid) const {

    // Compute conditional stats needed for correlation ratio
    auto cond = computeConditionalStats(grid, {0.5}, 0.001);
    return DependenceAnalyzer::compute(x, y, grid, cond);
}

std::vector<DeviationThreshold> KdeDerivativeAnalyzer::extractThresholds(
    const std::vector<ConditionalStats>& conditional,
    const std::vector<double>& tolerances,
    double probability,
    DerivativeAxis derType,
    DeviationAxis devType) const {

    std::vector<DeviationThreshold> thresholds;
    for (double tol : tolerances) {
        DeviationThreshold t;
        t.tolerance = tol;
        t.derivativeType = derType;
        t.deviationType = devType;
        t.probability = probability;

        // Find the derivative value where P(e > tol | d) first exceeds `probability`
        // We scan from high derivative to low (or low to high) to find the crossing
        bool found = false;
        for (std::size_t i = 0; i < conditional.size(); ++i) {
            const auto& cs = conditional[i];
            if (!cs.valid) continue;
            // P(e > tol | d) ≈ 1 - CDF(tol) at this d
            // Approximate using the conditional mean and std
            if (cs.stdY < 1e-15) continue;
            double z = (tol - cs.meanY) / cs.stdY;
            double pExceed = 1.0 - normalCdf(z);
            if (pExceed >= probability) {
                t.derivativeValue = cs.xValue;
                t.found = true;
                t.description = std::format(
                    "{} threshold: {:.4f} {} (P({} > {:.4f} {} = {:.1f}%)",
                    toString(derType), cs.xValue, unitString(derType),
                    toString(devType), tol, unitString(devType),
                    pExceed * 100.0);
                found = true;
                break;
            }
        }

        if (!found) {
            t.description = std::format(
                "No {} threshold found for {} tolerance {:.4f} {} at P={:.1f}%",
                toString(derType), toString(devType), tol, unitString(devType),
                probability * 100.0);
        }
        thresholds.push_back(t);
    }
    return thresholds;
}

TailRisk KdeDerivativeAnalyzer::computeTailRisk(
    const std::vector<double>& deviations,
    double varPercentile) const {
    return KdeStatistics::computeTailRisk(deviations, varPercentile);
}

double KdeDerivativeAnalyzer::densityMode(
    const std::vector<double>& bins,
    const std::vector<double>& density) {
    return KdeStatistics::densityMode(bins, density);
}

double KdeDerivativeAnalyzer::densityQuantile(
    const std::vector<double>& bins,
    const std::vector<double>& density,
    double q) {
    return KdeStatistics::densityQuantile(bins, density, q);
}

double KdeDerivativeAnalyzer::densityEntropy(const std::vector<double>& density) {
    return KdeStatistics::densityEntropy(density);
}

std::array<int, 3> KdeDerivativeAnalyzer::colormapColor(KdeColormap cmap, double value) {
    return KdeStatistics::colormapColor(cmap, value);
}

std::string KdeDerivativeAnalyzer::generateSummary(const KdeEvaluation& eval) const {
    if (!eval.hasSufficientData) {
        return std::format("Insufficient data ({} samples) for reliable KDE estimation.",
                          eval.derivatives.size());
    }

    return std::format(
        "KDE of {} vs {}: {} samples, {} kernel, {} bandwidth "
        "(h_x={:.4g}, h_y={:.4g}). "
        "Pearson r={:.3f}, MI={:.3f} bits, η²={:.3f}, dCor={:.3f}. "
        "Mode: ({:.4g} {}, {:.4g} {}). "
        "VaR95={:.4f} {}, CVaR95={:.4f} {}.",
        toString(eval.derivativeAxis), toString(eval.deviationAxis),
        eval.derivatives.size(),
        toString(eval.kernel), toString(eval.bandwidthMethod),
        eval.grid.bandwidthX, eval.grid.bandwidthY,
        eval.pearsonCorrelation, eval.mutualInformation,
        eval.correlationRatio, eval.distanceCorrelation,
        eval.modeDerivative, unitString(eval.derivativeAxis),
        eval.modeDeviation, unitString(eval.deviationAxis),
        eval.var95, unitString(eval.deviationAxis),
        eval.conditionalVar95, unitString(eval.deviationAxis));
}

KdeEvaluation KdeDerivativeAnalyzer::evaluate(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) const {

    KdeEvaluation eval;
    eval.derivativeAxis = config_.derivativeAxis;
    eval.deviationAxis = config_.deviationAxis;
    eval.kernel = config_.kernel;
    eval.bandwidthMethod = config_.bandwidthMethod;

    auto n = std::min(desired.size(), actual.size());
    if (n < 5) {
        eval.summary = generateSummary(eval);
        return eval;
    }

    // 1. Extract (derivative, deviation) pairs
    auto pairs = extractPairs(desired, actual);
    eval.derivatives = pairs.derivatives;
    eval.deviations = pairs.deviations;
    eval.arcLengths = pairs.arcLengths;
    eval.times = pairs.times;

    eval.hasSufficientData = (n >= 30);

    // 2. Compute bandwidths
    double hX = computeBandwidth(eval.derivatives, config_.bandwidthMethod,
                                  config_.fixedBandwidthX, config_.bandwidthScale);
    double hY = computeBandwidth(eval.deviations, config_.bandwidthMethod,
                                  config_.fixedBandwidthY, config_.bandwidthScale);
    eval.recommendedBandwidthX = hX;
    eval.recommendedBandwidthY = hY;

    // 3. Determine grid bounds
    auto [xMin, xMax] = [&]() -> std::pair<double, double> {
        if (config_.xRange[1] > config_.xRange[0]) {
            return {config_.xRange[0], config_.xRange[1]};
        }
        double mn = *std::min_element(eval.derivatives.begin(), eval.derivatives.end());
        double mx = *std::max_element(eval.derivatives.begin(), eval.derivatives.end());
        double pad = (mx - mn) * config_.rangePadding;
        if (mx - mn < 1e-12) { mn -= 0.5; mx += 0.5; }
        return {mn - pad, mx + pad};
    }();

    auto [yMin, yMax] = [&]() -> std::pair<double, double> {
        if (config_.yRange[1] > config_.yRange[0]) {
            return {config_.yRange[0], config_.yRange[1]};
        }
        double mn = *std::min_element(eval.deviations.begin(), eval.deviations.end());
        double mx = *std::max_element(eval.deviations.begin(), eval.deviations.end());
        double pad = (mx - mn) * config_.rangePadding;
        if (mx - mn < 1e-12) { mn -= 0.5; mx += 0.5; }
        return {mn - pad, mx + pad};
    }();

    // Ensure non-zero range
    if (xMax - xMin < 1e-12) { xMin -= 0.5; xMax += 0.5; }
    if (yMax - yMin < 1e-12) { yMin -= 0.5; yMax += 0.5; }

    // Ensure bandwidth is at least 2x the grid spacing so the kernel
    // always reaches neighboring bins (prevents empty grids when variance ≈ 0)
    double gridSpacingX = (xMax - xMin) / static_cast<double>(config_.gridX);
    double gridSpacingY = (yMax - yMin) / static_cast<double>(config_.gridY);
    hX = std::max(hX, 2.0 * gridSpacingX);
    hY = std::max(hY, 2.0 * gridSpacingY);

    // 4. Evaluate KDE
    if (n > config_.binnedThreshold) {
        eval.grid = binnedKde(eval.derivatives, eval.deviations,
                              hX, hY, xMin, xMax, yMin, yMax,
                              config_.gridX, config_.gridY, config_.kernel);
    } else {
        eval.grid = evaluateKde(eval.derivatives, eval.deviations,
                                hX, hY, xMin, xMax, yMin, yMax,
                                config_.gridX, config_.gridY, config_.kernel);
    }

    // 5. Marginal statistics
    eval.derivativeMarginal = computeMarginalStats(eval.derivatives);
    eval.deviationMarginal = computeMarginalStats(eval.deviations);

    // 6. Conditional statistics
    eval.conditional = computeConditionalStats(
        eval.grid, config_.quantileLevels, config_.minConditionalMass);

    // 7. Dependence metrics
    auto dep = computeDependence(eval.derivatives, eval.deviations, eval.grid);
    eval.pearsonCorrelation = dep.pearson;
    eval.spearmanCorrelation = dep.spearman;
    eval.kendallTau = dep.kendall;
    eval.mutualInformation = dep.mutualInformation;
    eval.correlationRatio = dep.correlationRatio;
    eval.distanceCorrelation = dep.distanceCorrelation;
    eval.dependenceIndex = dep.dependenceIndex;
    eval.jointEntropy = dep.jointEntropy;
    eval.conditionalEntropy = dep.conditionalEntropy;
    eval.normalizedMutualInfo = dep.normalizedMutualInfo;

    // 8. Mode of joint density
    {
        auto maxIt = std::max_element(eval.grid.density.begin(), eval.grid.density.end());
        std::size_t maxIdx = static_cast<std::size_t>(maxIt - eval.grid.density.begin());
        std::size_t iy = maxIdx / eval.grid.xBins.size();
        std::size_t ix = maxIdx % eval.grid.xBins.size();
        eval.modeDerivative = eval.grid.xBins[ix];
        eval.modeDeviation = eval.grid.yBins[iy];
        eval.maxDensity = *maxIt;
    }

    // 9. Tail risk
    auto risk = computeTailRisk(eval.deviations, 0.95);
    eval.tailFraction = risk.tailFraction;
    eval.var95 = risk.var95;
    eval.expectedTailDeviation = risk.expectedTailDeviation;
    eval.conditionalVar95 = risk.conditionalVar95;

    // 10. Thresholds
    eval.thresholds = extractThresholds(
        eval.conditional, config_.tolerances, config_.thresholdProbability,
        config_.derivativeAxis, config_.deviationAxis);

    // 11. Summary
    eval.summary = generateSummary(eval);

    return eval;
}

} // namespace tether::motion::replanner
