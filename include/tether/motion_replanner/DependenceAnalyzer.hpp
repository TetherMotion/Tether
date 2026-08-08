// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file DependenceAnalyzer.hpp
 * @brief Statistical dependence metrics extracted from KdeDerivativeAnalyzer
 *
 * @details
 * Encapsulates the dependence-measurement sub-responsibility of
 * KdeDerivativeAnalyzer:
 *  - Pearson linear correlation
 *  - Spearman rank correlation
 *  - Kendall's tau
 *  - Distance correlation (Szekely & Rizzo, 2007)
 *  - Mutual information from a 2D KDE grid
 *  - Joint / conditional / normalized entropy
 *  - Correlation ratio (eta-squared)
 *  - Composite dependence index
 *
 * All methods are static (stateless) and operate on raw data vectors
 * and/or a KdeGrid. The KdeDerivativeAnalyzer delegates its
 * computeDependence() call to this class.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp" // KdeGrid, DependenceMetrics, ConditionalStats

#include <vector>

namespace tether::motion::replanner {

class DependenceAnalyzer {
public:
    /// Compute the full dependence metrics from raw data and a KDE grid.
    /// @param x  X-axis sample values (e.g. derivatives).
    /// @param y  Y-axis sample values (e.g. deviations).
    /// @param grid  The 2D KDE grid for entropy/MI computation.
    /// @param conditional  Pre-computed conditional stats (for correlation
    ///                     ratio). If empty, correlationRatio will be 0.
    /// @return The complete DependenceMetrics struct.
    static DependenceMetrics compute(
        const std::vector<double>& x,
        const std::vector<double>& y,
        const KdeGrid& grid,
        const std::vector<ConditionalStats>& conditional = {});

    //--- Individual metrics (public for testing / reuse) ---

    /// Pearson linear correlation coefficient.
    static double pearson(const std::vector<double>& x,
                          const std::vector<double>& y);

    /// Spearman rank correlation coefficient.
    static double spearmanRank(const std::vector<double>& x,
                               const std::vector<double>& y);

    /// Kendall's tau rank correlation.
    static double kendallTau(const std::vector<double>& x,
                             const std::vector<double>& y);

    /// Distance correlation (Szekely & Rizzo, 2007).
    static double distanceCorrelation(const std::vector<double>& x,
                                      const std::vector<double>& y);

    /// Mutual information from a 2D KDE grid (bits).
    static double mutualInformationFromGrid(const KdeGrid& grid);

    /// Entropy values from a 2D KDE grid.
    struct GridEntropy {
        double joint = 0.0;
        double conditional = 0.0;
        double normalizedMI = 0.0;
    };
    static GridEntropy entropyFromGrid(const KdeGrid& grid);

    /// Correlation ratio η² = Var[E[y|x]] / Var[y].
    static double correlationRatio(
        const std::vector<double>& y,
        const std::vector<ConditionalStats>& conditional);

private:
    /// Compute mean of a vector.
    static double mean(const std::vector<double>& v);

    /// Compute ranks (average for ties).
    static std::vector<double> rank(const std::vector<double>& v);
};

} // namespace tether::motion::replanner
