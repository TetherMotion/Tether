// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file KdeGridEvaluator.hpp
 * @brief 2D KDE grid evaluation extracted from KdeDerivativeAnalyzer
 *
 * @details
 * Encapsulates the core KDE grid computation sub-responsibility of
 * KdeDerivativeAnalyzer:
 *  - Direct 2D KDE evaluation on a regular grid
 *  - Binned KDE for large N (histogram + separable kernel convolution)
 *  - Kernel function evaluation (Gaussian, Epanechnikov, Uniform, etc.)
 *
 * All methods are static (stateless) and operate on raw data vectors.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp" // KdeGrid, KernelType

#include <vector>

namespace tether::motion::replanner {

class KdeGridEvaluator {
public:
    /// Evaluate a 1D kernel function at a normalized distance.
    /// @param kernel Kernel type.
    /// @param u Normalized distance (x - x_i) / h.
    /// @return Kernel weight K(u).
    static double kernelValue(KernelType kernel, double u);

    /// Evaluate the 2D KDE on a regular grid.
    /// @param derivatives X-axis sample values.
    /// @param deviations Y-axis sample values.
    /// @param hX X bandwidth.
    /// @param hY Y bandwidth.
    /// @param xMin, xMax, yMin, yMax Grid bounds.
    /// @param nX, nY Grid resolution.
    /// @param kernel Kernel type.
    /// @return The KDE grid.
    static KdeGrid evaluate(
        const std::vector<double>& derivatives,
        const std::vector<double>& deviations,
        double hX, double hY,
        double xMin, double xMax,
        double yMin, double yMax,
        std::size_t nX, std::size_t nY,
        KernelType kernel);

    /// Binned KDE for large N (histogram + kernel convolution).
    /// @param histSize Internal histogram resolution (typically 256).
    static KdeGrid binnedEvaluate(
        const std::vector<double>& derivatives,
        const std::vector<double>& deviations,
        double hX, double hY,
        double xMin, double xMax,
        double yMin, double yMax,
        std::size_t nX, std::size_t nY,
        KernelType kernel,
        std::size_t histSize = 256);

private:
    /// Linear interpolation.
    static double lerp(double a, double b, double t);

    /// Generate n linearly-spaced values in [min, max].
    static std::vector<double> linspace(double min, double max, std::size_t n);
};

} // namespace tether::motion::replanner
