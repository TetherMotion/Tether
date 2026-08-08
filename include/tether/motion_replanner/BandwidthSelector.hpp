// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file BandwidthSelector.hpp
 * @brief KDE bandwidth selection methods extracted from KdeDerivativeAnalyzer
 *
 * @details
 * Encapsulates the bandwidth-selection sub-responsibility of
 * KdeDerivativeAnalyzer. Supports:
 *  - Silverman's rule of thumb
 *  - Scott's rule
 *  - Improved Sheather-Jones (ISJ) plug-in
 *  - Fixed user-specified value
 *  - Least-squares cross-validation (LSCV)
 *  - Likelihood cross-validation (LCV)
 *
 * All methods are static (stateless) and operate on raw data vectors.
 */

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp" // BandwidthMethod

#include <vector>

namespace tether::motion::replanner {

class BandwidthSelector {
public:
    /// Compute bandwidth from data using the configured method.
    /// @param data The sample values.
    /// @param method Bandwidth selection method.
    /// @param fixedValue Fixed value (used when method = Fixed).
    /// @param scale Multiplier applied to the result.
    /// @return The selected bandwidth.
    static double compute(const std::vector<double>& data,
                          BandwidthMethod method,
                          double fixedValue,
                          double scale);

    //--- Individual methods (public for testing / reuse) ---

    /// Silverman's rule of thumb: h = 1.06 * σ * n^(-1/5)
    static double silverman(const std::vector<double>& data);

    /// Scott's rule: h = σ * n^(-1/5)
    static double scott(const std::vector<double>& data);

    /// Improved Sheather-Jones (simplified plug-in).
    static double isj(const std::vector<double>& data);

    /// Least-squares cross-validation (simplified grid search).
    static double lscv(const std::vector<double>& data);

    /// Likelihood cross-validation (simplified grid search).
    static double likelihoodCv(const std::vector<double>& data);

private:
    /// Compute standard deviation (sample, ddof=1).
    static double stdDev(const std::vector<double>& v);

    /// Compute mean of a vector.
    static double mean(const std::vector<double>& v);

    /// Quantile from a sorted vector (linear interpolation).
    static double quantileSorted(const std::vector<double>& sorted, double q);

    /// Standard normal PDF.
    static double normalPdf(double x);
};

} // namespace tether::motion::replanner
