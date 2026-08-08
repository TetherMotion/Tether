// SPDX-License-Identifier: MIT

/**
 * @file KdeGridEvaluator.cpp
 * @brief Implementation of 2D KDE grid evaluation
 */

#include "tether/motion_replanner/KdeGridEvaluator.hpp"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tether::motion::replanner {

//=============================================================================
// Private helpers
//=============================================================================

double KdeGridEvaluator::lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

std::vector<double> KdeGridEvaluator::linspace(double min, double max, std::size_t n) {
    std::vector<double> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(n - 1);
        v[i] = lerp(min, max, t);
    }
    return v;
}

//=============================================================================
// Kernel function
//=============================================================================

double KdeGridEvaluator::kernelValue(KernelType kernel, double u) {
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

//=============================================================================
// Direct 2D KDE evaluation
//=============================================================================

KdeGrid KdeGridEvaluator::evaluate(
    const std::vector<double>& derivatives,
    const std::vector<double>& deviations,
    double hX, double hY,
    double xMin, double xMax,
    double yMin, double yMax,
    std::size_t nX, std::size_t nY,
    KernelType kernel) {

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

//=============================================================================
// Binned KDE (for large N)
//=============================================================================

KdeGrid KdeGridEvaluator::binnedEvaluate(
    const std::vector<double>& derivatives,
    const std::vector<double>& deviations,
    double hX, double hY,
    double xMin, double xMax,
    double yMin, double yMax,
    std::size_t nX, std::size_t nY,
    KernelType kernel,
    std::size_t histSize) {

    // Binned KDE: histogram the data, then convolve with the kernel
    auto n = derivatives.size();

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

} // namespace tether::motion::replanner
