// SPDX-License-Identifier: MIT

/**
 * @file BandwidthSelector.cpp
 * @brief Implementation of KDE bandwidth selection methods
 */

#include "tether/motion_replanner/BandwidthSelector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tether::motion::replanner {

//=============================================================================
// Private helpers
//=============================================================================

double BandwidthSelector::mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double BandwidthSelector::stdDev(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 2) return 0.0;
    double m = mean(v);
    double sq = 0.0;
    for (double x : v) sq += (x - m) * (x - m);
    return std::sqrt(sq / static_cast<double>(n - 1));
}

double BandwidthSelector::quantileSorted(const std::vector<double>& sorted, double q) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted[0];
    double pos = q * static_cast<double>(sorted.size() - 1);
    auto lo = static_cast<std::size_t>(std::floor(pos));
    auto hi = static_cast<std::size_t>(std::ceil(pos));
    if (lo == hi) return sorted[lo];
    double frac = pos - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

double BandwidthSelector::normalPdf(double x) {
    return std::exp(-0.5 * x * x) / std::sqrt(2.0 * M_PI);
}

//=============================================================================
// Individual methods
//=============================================================================

double BandwidthSelector::silverman(const std::vector<double>& data) {
    auto n = data.size();
    if (n < 2) return 1e-6;
    double sigma = stdDev(data);
    if (sigma < 1e-15) return 1e-6;
    return 1.06 * sigma * std::pow(static_cast<double>(n), -0.2);
}

double BandwidthSelector::scott(const std::vector<double>& data) {
    auto n = data.size();
    if (n < 2) return 1e-6;
    double sigma = stdDev(data);
    if (sigma < 1e-15) return 1e-6;
    return sigma * std::pow(static_cast<double>(n), -0.2);
}

double BandwidthSelector::isj(const std::vector<double>& data) {
    // Improved Sheather-Jones (simplified plug-in)
    auto n = data.size();
    if (n < 2) return 1e-6;
    double sigma = stdDev(data);
    double iqr = [&]() {
        auto sorted = data;
        std::sort(sorted.begin(), sorted.end());
        return quantileSorted(sorted, 0.75) - quantileSorted(sorted, 0.25);
    }();
    double sigmaHat = std::min(sigma, iqr / 1.34);
    if (sigmaHat < 1e-15) sigmaHat = sigma;

    // Silverman's rule as starting point
    double h0 = 1.06 * sigmaHat * std::pow(static_cast<double>(n), -0.2);

    // Pilot estimate of the roughness functional
    // φ^(4) integral ≈ 3 / (8 * sqrt(π) * σ^5) for Gaussian
    double phi4Integral = 3.0 / (8.0 * std::sqrt(M_PI) * std::pow(sigmaHat, 5.0));

    // ISJ formula: h = (1 / (2 * sqrt(π) * φ^(4) * n))^(1/5)
    double h = std::pow(1.0 / (2.0 * std::sqrt(M_PI) * phi4Integral * static_cast<double>(n)), 0.2);

    // Sanity bounds
    h = std::max(h, h0 * 0.1);
    h = std::min(h, h0 * 10.0);
    return h;
}

double BandwidthSelector::lscv(const std::vector<double>& data) {
    // Least-squares cross-validation (simplified grid search)
    auto n = data.size();
    if (n < 5) return silverman(data);

    double sigma = stdDev(data);
    double hMin = 0.1 * sigma * std::pow(static_cast<double>(n), -0.2);
    double hMax = 3.0 * sigma * std::pow(static_cast<double>(n), -0.2);

    // Coarse grid search
    double bestH = hMin;
    double bestScore = std::numeric_limits<double>::max();
    int steps = 20;
    for (int i = 0; i <= steps; ++i) {
        double h = hMin + (hMax - hMin) * static_cast<double>(i) / static_cast<double>(steps);
        // LSCV score: ∫f² - 2/n Σ f_(-i)(x_i)
        // Simplified: use leave-one-out
        double score = 0.0;
        for (std::size_t j = 0; j < n; ++j) {
            double fMinusI = 0.0;
            for (std::size_t k = 0; k < n; ++k) {
                if (k == j) continue;
                double u = (data[j] - data[k]) / h;
                fMinusI += normalPdf(u);
            }
            fMinusI /= static_cast<double>(n - 1) * h;
            score += fMinusI * fMinusI - 2.0 * fMinusI;
        }
        score /= static_cast<double>(n);
        if (score < bestScore) {
            bestScore = score;
            bestH = h;
        }
    }
    return bestH;
}

double BandwidthSelector::likelihoodCv(const std::vector<double>& data) {
    // Likelihood cross-validation (simplified grid search)
    auto n = data.size();
    if (n < 5) return silverman(data);

    double sigma = stdDev(data);
    double hMin = 0.1 * sigma * std::pow(static_cast<double>(n), -0.2);
    double hMax = 3.0 * sigma * std::pow(static_cast<double>(n), -0.2);

    double bestH = hMin;
    double bestScore = -std::numeric_limits<double>::max();
    int steps = 20;
    for (int i = 0; i <= steps; ++i) {
        double h = hMin + (hMax - hMin) * static_cast<double>(i) / static_cast<double>(steps);
        double score = 0.0;
        for (std::size_t j = 0; j < n; ++j) {
            double fMinusI = 0.0;
            for (std::size_t k = 0; k < n; ++k) {
                if (k == j) continue;
                double u = (data[j] - data[k]) / h;
                fMinusI += normalPdf(u);
            }
            fMinusI /= static_cast<double>(n - 1) * h;
            if (fMinusI > 1e-15) score += std::log(fMinusI);
        }
        if (score > bestScore) {
            bestScore = score;
            bestH = h;
        }
    }
    return bestH;
}

//=============================================================================
// Composite compute
//=============================================================================

double BandwidthSelector::compute(const std::vector<double>& data,
                                   BandwidthMethod method,
                                   double fixedValue,
                                   double scale) {
    auto n = data.size();
    if (n < 2) return 1e-6;

    double sigma = stdDev(data);
    if (sigma < 1e-15) return 1e-6;

    double h = 0.0;
    switch (method) {
        case BandwidthMethod::Silverman: {
            h = silverman(data);
            break;
        }
        case BandwidthMethod::Scott: {
            h = scott(data);
            break;
        }
        case BandwidthMethod::ISJ: {
            h = isj(data);
            break;
        }
        case BandwidthMethod::Fixed: {
            h = fixedValue;
            break;
        }
        case BandwidthMethod::LeastSquaresCV: {
            h = lscv(data);
            break;
        }
        case BandwidthMethod::LikelihoodCV: {
            h = likelihoodCv(data);
            break;
        }
    }

    // Apply scale and ensure positivity
    h = std::max(h * scale, 1e-12);
    return h;
}

} // namespace tether::motion::replanner
