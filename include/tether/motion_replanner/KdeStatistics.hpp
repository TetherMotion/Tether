// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file KdeStatistics.hpp
 * @brief Statistical helpers extracted from KdeDerivativeAnalyzer
 *
 * @details
 * Encapsulates the statistics-computation sub-responsibility of
 * KdeDerivativeAnalyzer:
 *  - Marginal statistics (mean, std, skewness, kurtosis, quantiles, mode)
 *  - Conditional statistics p(e | d) from a KDE grid
 *  - Tail risk metrics (VaR, CVaR, ETD)
 *  - 1D density helpers (mode, quantile, entropy)
 *  - Colormap color mapping
 *
 * All methods are static (stateless) and operate on raw data vectors
 * and/or a KdeGrid.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp" // KdeGrid, MarginalStats, ConditionalStats, KdeColormap, TailRisk

#include <array>
#include <vector>

namespace tether::motion::replanner {

class KdeStatistics {
public:
    //--- Marginal statistics ---

    /// Compute marginal statistics from a 1D sample.
    static MarginalStats computeMarginalStats(const std::vector<double>& data);

    //--- Conditional statistics ---

    /// Compute conditional statistics p(e | d) from the KDE grid.
    static std::vector<ConditionalStats> computeConditionalStats(
        const KdeGrid& grid,
        const std::vector<double>& quantileLevels,
        double minMass);

    //--- Tail risk ---

    /// Compute tail risk metrics (VaR, CVaR, ETD).
    static TailRisk computeTailRisk(
        const std::vector<double>& deviations,
        double varPercentile = 0.95);

    //--- 1D density helpers ---

    /// Compute the mode (peak) of a 1D density estimate.
    static double densityMode(const std::vector<double>& bins,
                              const std::vector<double>& density);

    /// Compute the quantile of a 1D density estimate.
    static double densityQuantile(const std::vector<double>& bins,
                                  const std::vector<double>& density,
                                  double q);

    /// Compute the entropy of a 1D density estimate (bits).
    static double densityEntropy(const std::vector<double>& density);

    //--- Colormap ---

    /// Convert a colormap enum to an RGB color for a value in [0, 1].
    /// @return (r, g, b) in [0, 255].
    static std::array<int, 3> colormapColor(KdeColormap cmap, double value);

private:
    //--- Statistical helpers ---

    static double mean(const std::vector<double>& v);
    static double stdDev(const std::vector<double>& v);
    static double skewness(const std::vector<double>& v);
    static double kurtosis(const std::vector<double>& v);
    static double quantileSorted(const std::vector<double>& sorted, double q);
    static double lerp(double a, double b, double t);
};

} // namespace tether::motion::replanner
