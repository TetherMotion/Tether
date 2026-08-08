// SPDX-License-Identifier: MIT

/**
 * @file KdeStatistics.cpp
 * @brief Implementation of KDE statistical helpers
 */

#include "tether/motion_replanner/KdeStatistics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace tether::motion::replanner {

//=============================================================================
// Private helpers
//=============================================================================

double KdeStatistics::mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double KdeStatistics::stdDev(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 2) return 0.0;
    double m = mean(v);
    double sq = 0.0;
    for (double x : v) sq += (x - m) * (x - m);
    return std::sqrt(sq / static_cast<double>(n - 1));
}

double KdeStatistics::skewness(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 3) return 0.0;
    double m = mean(v);
    double s = stdDev(v);
    if (s < 1e-15) return 0.0;
    double sk = 0.0;
    for (double x : v) sk += std::pow((x - m) / s, 3.0);
    return sk / static_cast<double>(n);
}

double KdeStatistics::kurtosis(const std::vector<double>& v) {
    auto n = v.size();
    if (n < 4) return 0.0;
    double m = mean(v);
    double s = stdDev(v);
    if (s < 1e-15) return 0.0;
    double k = 0.0;
    for (double x : v) k += std::pow((x - m) / s, 4.0);
    return k / static_cast<double>(n) - 3.0;
}

double KdeStatistics::quantileSorted(const std::vector<double>& sorted, double q) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted[0];
    double pos = q * static_cast<double>(sorted.size() - 1);
    auto lo = static_cast<std::size_t>(std::floor(pos));
    auto hi = static_cast<std::size_t>(std::ceil(pos));
    if (lo == hi) return sorted[lo];
    double frac = pos - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

double KdeStatistics::lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

//=============================================================================
// Marginal statistics
//=============================================================================

MarginalStats KdeStatistics::computeMarginalStats(const std::vector<double>& data) {
    MarginalStats stats;
    auto n = data.size();
    if (n == 0) return stats;

    stats.mean = mean(data);
    stats.stdDev = stdDev(data);
    stats.skewness = skewness(data);
    stats.kurtosis = kurtosis(data);

    auto sorted = data;
    std::sort(sorted.begin(), sorted.end());
    stats.min = sorted.front();
    stats.max = sorted.back();
    stats.median = quantileSorted(sorted, 0.50);
    stats.p05 = quantileSorted(sorted, 0.05);
    stats.p25 = quantileSorted(sorted, 0.25);
    stats.p75 = quantileSorted(sorted, 0.75);
    stats.p95 = quantileSorted(sorted, 0.95);

    // Mode: peak of a simple histogram
    if (n > 10) {
        std::size_t numBins = std::min<std::size_t>(50, n / 5);
        double range = stats.max - stats.min;
        if (range > 1e-12) {
            double binW = range / static_cast<double>(numBins);
            std::vector<std::size_t> counts(numBins, 0);
            for (double x : data) {
                std::size_t b = static_cast<std::size_t>((x - stats.min) / binW);
                if (b >= numBins) b = numBins - 1;
                counts[b]++;
            }
            auto maxIt = std::max_element(counts.begin(), counts.end());
            std::size_t maxBin = static_cast<std::size_t>(maxIt - counts.begin());
            stats.mode = stats.min + (static_cast<double>(maxBin) + 0.5) * binW;
        } else {
            stats.mode = stats.mean;
        }
    } else {
        stats.mode = stats.mean;
    }

    return stats;
}

//=============================================================================
// Conditional statistics
//=============================================================================

std::vector<ConditionalStats> KdeStatistics::computeConditionalStats(
    const KdeGrid& grid,
    const std::vector<double>& quantileLevels,
    double minMass) {

    std::vector<ConditionalStats> result(grid.xBins.size());
    if (grid.yBins.empty()) return result;

    auto margX = grid.marginalX();
    double totalMass = grid.totalMass();
    if (totalMass < 1e-15) return result;

    double dy = (grid.yBins.size() > 1) ?
        (grid.yBins.back() - grid.yBins.front()) / static_cast<double>(grid.yBins.size() - 1) : 1.0;

    for (std::size_t ix = 0; ix < grid.xBins.size(); ++ix) {
        ConditionalStats& cs = result[ix];
        cs.xValue = grid.xBins[ix];
        cs.mass = margX[ix] * dy / totalMass;
        cs.valid = (cs.mass >= minMass);

        if (!cs.valid) continue;

        // Build conditional density p(e | d) = p(d, e) / p(d)
        std::vector<double> condDensity(grid.yBins.size());
        double margD = margX[ix];
        if (margD < 1e-15) {
            cs.valid = false;
            continue;
        }
        for (std::size_t iy = 0; iy < grid.yBins.size(); ++iy) {
            condDensity[iy] = grid.at(ix, iy) / margD;
        }

        // Compute conditional mean E[e | d]
        double sum = 0.0, sum2 = 0.0, totalP = 0.0;
        for (std::size_t iy = 0; iy < grid.yBins.size(); ++iy) {
            double p = condDensity[iy] * dy;
            sum += grid.yBins[iy] * p;
            sum2 += grid.yBins[iy] * grid.yBins[iy] * p;
            totalP += p;
        }
        if (totalP < 1e-15) {
            cs.valid = false;
            continue;
        }
        cs.meanY = sum / totalP;
        double varY = sum2 / totalP - cs.meanY * cs.meanY;
        cs.stdY = (varY > 0.0) ? std::sqrt(varY) : 0.0;

        // Mode of conditional density
        auto maxIt = std::max_element(condDensity.begin(), condDensity.end());
        std::size_t maxIdx = static_cast<std::size_t>(maxIt - condDensity.begin());
        cs.modeY = grid.yBins[maxIdx];

        // Quantiles from conditional CDF
        std::vector<double> cdf(grid.yBins.size());
        cdf[0] = condDensity[0] * dy;
        for (std::size_t iy = 1; iy < grid.yBins.size(); ++iy) {
            cdf[iy] = cdf[iy - 1] + condDensity[iy] * dy;
        }
        double cdfTotal = cdf.back();
        if (cdfTotal < 1e-15) {
            cs.valid = false;
            continue;
        }
        // Normalize CDF
        for (double& c : cdf) c /= cdfTotal;

        // Find quantiles
        auto findQuantile = [&](double q) -> double {
            for (std::size_t iy = 0; iy < cdf.size(); ++iy) {
                if (cdf[iy] >= q) {
                    if (iy == 0) return grid.yBins[0];
                    // Linear interpolation
                    double t = (q - cdf[iy - 1]) / (cdf[iy] - cdf[iy - 1] + 1e-15);
                    return lerp(grid.yBins[iy - 1], grid.yBins[iy], t);
                }
            }
            return grid.yBins.back();
        };

        for (double q : quantileLevels) {
            double val = findQuantile(q);
            if (q <= 0.051) cs.p05Y = val;
            if (q <= 0.251) cs.p25Y = val;
            if (q <= 0.501) cs.medianY = val;
            if (q <= 0.751) cs.p75Y = val;
            if (q <= 0.951) cs.p95Y = val;
        }
    }

    return result;
}

//=============================================================================
// Tail risk
//=============================================================================

TailRisk KdeStatistics::computeTailRisk(
    const std::vector<double>& deviations,
    double varPercentile) {

    TailRisk risk;
    auto n = deviations.size();
    if (n == 0) return risk;

    auto sorted = deviations;
    std::sort(sorted.begin(), sorted.end());

    risk.var95 = quantileSorted(sorted, varPercentile);
    risk.tailFraction = 1.0 - varPercentile;

    // Expected tail deviation
    double tailSum = 0.0;
    std::size_t tailCount = 0;
    for (double v : sorted) {
        if (v > risk.var95) {
            tailSum += v;
            ++tailCount;
        }
    }
    if (tailCount > 0) {
        risk.expectedTailDeviation = tailSum / static_cast<double>(tailCount);
        risk.conditionalVar95 = risk.expectedTailDeviation;
    }

    return risk;
}

//=============================================================================
// 1D density helpers
//=============================================================================

double KdeStatistics::densityMode(const std::vector<double>& bins,
                                  const std::vector<double>& density) {
    if (bins.empty() || density.empty()) return 0.0;
    auto maxIt = std::max_element(density.begin(), density.end());
    std::size_t maxIdx = static_cast<std::size_t>(maxIt - density.begin());
    return bins[maxIdx];
}

double KdeStatistics::densityQuantile(const std::vector<double>& bins,
                                      const std::vector<double>& density,
                                      double q) {
    if (bins.empty() || density.empty()) return 0.0;
    double total = std::accumulate(density.begin(), density.end(), 0.0);
    if (total < 1e-15) return bins[0];

    double cumulative = 0.0;
    for (std::size_t i = 0; i < density.size(); ++i) {
        cumulative += density[i] / total;
        if (cumulative >= q) {
            if (i == 0) return bins[0];
            double t = (q - (cumulative - density[i] / total)) / (density[i] / total + 1e-15);
            return lerp(bins[i - 1], bins[i], t);
        }
    }
    return bins.back();
}

double KdeStatistics::densityEntropy(const std::vector<double>& density) {
    double total = std::accumulate(density.begin(), density.end(), 0.0);
    if (total < 1e-15) return 0.0;
    double h = 0.0;
    for (double d : density) {
        double p = d / total;
        if (p > 1e-15) h += -p * std::log2(p);
    }
    return h;
}

//=============================================================================
// Colormap
//=============================================================================

std::array<int, 3> KdeStatistics::colormapColor(KdeColormap cmap, double value) {
    // Clamp value to [0, 1]
    value = std::clamp(value, 0.0, 1.0);

    auto clamp255 = [](double v) { return static_cast<int>(std::clamp(v, 0.0, 255.0)); };

    switch (cmap) {
        case KdeColormap::Viridis: {
            // Simplified Viridis: purple → blue → green → yellow
            double r, g, b;
            if (value < 0.25) {
                double t = value / 0.25;
                r = 68 + t * (59 - 68);
                g = 1 + t * (82 - 1);
                b = 84 + t * (139 - 84);
            } else if (value < 0.5) {
                double t = (value - 0.25) / 0.25;
                r = 59 + t * (33 - 59);
                g = 82 + t * (145 - 82);
                b = 139 + t * (140 - 139);
            } else if (value < 0.75) {
                double t = (value - 0.5) / 0.25;
                r = 33 + t * (94 - 33);
                g = 145 + t * (201 - 145);
                b = 140 + t * (98 - 140);
            } else {
                double t = (value - 0.75) / 0.25;
                r = 94 + t * (253 - 94);
                g = 201 + t * (231 - 201);
                b = 98 + t * (37 - 98);
            }
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Inferno: {
            // Black → purple → red → orange → yellow
            double r, g, b;
            if (value < 0.33) {
                double t = value / 0.33;
                r = t * 87;
                g = t * 16;
                b = t * 110;
            } else if (value < 0.66) {
                double t = (value - 0.33) / 0.33;
                r = 87 + t * (188 - 87);
                g = 16 + t * (55 - 16);
                b = 110 + t * (84 - 110);
            } else {
                double t = (value - 0.66) / 0.34;
                r = 188 + t * (252 - 188);
                g = 55 + t * (236 - 55);
                b = 84 + t * (74 - 84);
            }
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Plasma: {
            double r, g, b;
            if (value < 0.33) {
                double t = value / 0.33;
                r = 13 + t * (126 - 13);
                g = 8 + t * (3 - 8);
                b = 135 + t * (139 - 135);
            } else if (value < 0.66) {
                double t = (value - 0.33) / 0.33;
                r = 126 + t * (204 - 126);
                g = 3 + t * (71 - 3);
                b = 139 + t * (114 - 139);
            } else {
                double t = (value - 0.66) / 0.34;
                r = 204 + t * (240 - 204);
                g = 71 + t * (215 - 71);
                b = 114 + t * (50 - 114);
            }
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Magma: {
            double r, g, b;
            if (value < 0.33) {
                double t = value / 0.33;
                r = t * 80;
                g = t * 18;
                b = t * 120;
            } else if (value < 0.66) {
                double t = (value - 0.33) / 0.33;
                r = 80 + t * (180 - 80);
                g = 18 + t * (50 - 18);
                b = 120 + t * (90 - 120);
            } else {
                double t = (value - 0.66) / 0.34;
                r = 180 + t * (252 - 180);
                g = 50 + t * (220 - 50);
                b = 90 + t * (180 - 90);
            }
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Jet: {
            double r, g, b;
            if (value < 0.125) {
                r = 0; g = 0; b = 128 + value * 8 * 127;
            } else if (value < 0.375) {
                double t = (value - 0.125) / 0.25;
                r = 0; g = t * 255; b = 255;
            } else if (value < 0.625) {
                double t = (value - 0.375) / 0.25;
                r = t * 255; g = 255; b = 255 * (1 - t);
            } else if (value < 0.875) {
                double t = (value - 0.625) / 0.25;
                r = 255; g = 255 * (1 - t); b = 0;
            } else {
                double t = (value - 0.875) / 0.125;
                r = 255 * (1 - t * 0.5); g = 0; b = 0;
            }
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Hot: {
            double r, g, b;
            if (value < 0.33) {
                double t = value / 0.33;
                r = t * 255; g = 0; b = 0;
            } else if (value < 0.66) {
                double t = (value - 0.33) / 0.33;
                r = 255; g = t * 255; b = 0;
            } else {
                double t = (value - 0.66) / 0.34;
                r = 255; g = 255; b = t * 255;
            }
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Cool: {
            double r = value * 255;
            double g = 255 - value * 255;
            double b = 255;
            return {clamp255(r), clamp255(g), clamp255(b)};
        }
        case KdeColormap::Grayscale: {
            int v = clamp255(value * 255);
            return {v, v, v};
        }
        case KdeColormap::BlueRed: {
            double r = value * 255;
            double b = (1.0 - value) * 255;
            return {clamp255(r), 0, clamp255(b)};
        }
    }
    return {0, 0, 0};
}

} // namespace tether::motion::replanner
