// SPDX-License-Identifier: MIT

/**
 * @file DependenceAnalyzer.cpp
 * @brief Implementation of statistical dependence metrics
 */

#include "tether/motion_replanner/DependenceAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace tether::motion::replanner {

//=============================================================================
// Private helpers
//=============================================================================

double DependenceAnalyzer::mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

std::vector<double> DependenceAnalyzer::rank(const std::vector<double>& v) {
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

//=============================================================================
// Individual metrics
//=============================================================================

double DependenceAnalyzer::pearson(const std::vector<double>& x,
                                   const std::vector<double>& y) {
    auto n = std::min(x.size(), y.size());
    if (n < 2) return 0.0;

    double mx = mean(x), my = mean(y);
    double sxy = 0.0, sxx = 0.0, syy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double dx = x[i] - mx, dy = y[i] - my;
        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    if (sxx < 1e-15 || syy < 1e-15) return 0.0;
    return sxy / std::sqrt(sxx * syy);
}

double DependenceAnalyzer::spearmanRank(const std::vector<double>& x,
                                        const std::vector<double>& y) {
    auto n = x.size();
    if (n < 2) return 0.0;

    auto rx = rank(x);
    auto ry = rank(y);

    double mx = mean(rx), my = mean(ry);
    double sxy = 0.0, sxx = 0.0, syy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double dx = rx[i] - mx, dy = ry[i] - my;
        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    if (sxx < 1e-15 || syy < 1e-15) return 0.0;
    return sxy / std::sqrt(sxx * syy);
}

double DependenceAnalyzer::kendallTau(const std::vector<double>& x,
                                      const std::vector<double>& y) {
    auto n = x.size();
    if (n < 2) return 0.0;

    long long concordant = 0, discordant = 0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            double dx = x[i] - x[j];
            double dy = y[i] - y[j];
            if (dx * dy > 0) ++concordant;
            else if (dx * dy < 0) ++discordant;
        }
    }
    double total = static_cast<double>(n) * (static_cast<double>(n) - 1) / 2.0;
    if (total < 1e-15) return 0.0;
    return static_cast<double>(concordant - discordant) / total;
}

double DependenceAnalyzer::distanceCorrelation(const std::vector<double>& x,
                                               const std::vector<double>& y) {
    auto n = x.size();
    if (n < 2) return 0.0;

    // Compute distance matrices
    std::vector<double> aMat(n * n), bMat(n * n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            aMat[i * n + j] = std::abs(x[i] - x[j]);
            bMat[i * n + j] = std::abs(y[i] - y[j]);
        }
    }

    // Double-center
    auto doubleCenter = [&](std::vector<double>& m) {
        std::vector<double> rowMean(n, 0.0), colMean(n, 0.0);
        double grandMean = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                rowMean[i] += m[i * n + j];
                colMean[j] += m[i * n + j];
                grandMean += m[i * n + j];
            }
        }
        for (double& v : rowMean) v /= static_cast<double>(n);
        for (double& v : colMean) v /= static_cast<double>(n);
        grandMean /= static_cast<double>(n * n);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                m[i * n + j] -= rowMean[i] + colMean[j] - grandMean;
            }
        }
    };

    doubleCenter(aMat);
    doubleCenter(bMat);

    // Compute dCov², dVar²(X), dVar²(Y)
    double dCov2 = 0.0, dVarX2 = 0.0, dVarY2 = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            dCov2 += aMat[i * n + j] * bMat[i * n + j];
            dVarX2 += aMat[i * n + j] * aMat[i * n + j];
            dVarY2 += bMat[i * n + j] * bMat[i * n + j];
        }
    }
    dCov2 /= static_cast<double>(n * n);
    dVarX2 /= static_cast<double>(n * n);
    dVarY2 /= static_cast<double>(n * n);

    if (dVarX2 < 1e-15 || dVarY2 < 1e-15) return 0.0;
    double dCor2 = dCov2 / std::sqrt(dVarX2 * dVarY2);
    return std::sqrt(std::abs(dCor2));
}

double DependenceAnalyzer::mutualInformationFromGrid(const KdeGrid& grid) {
    if (grid.density.empty()) return 0.0;

    auto margX = grid.marginalX();
    auto margY = grid.marginalY();
    double total = grid.totalMass();
    if (total < 1e-15) return 0.0;

    double dx = (grid.xBins.size() > 1) ?
        (grid.xBins.back() - grid.xBins.front()) / static_cast<double>(grid.xBins.size() - 1) : 1.0;
    double dy = (grid.yBins.size() > 1) ?
        (grid.yBins.back() - grid.yBins.front()) / static_cast<double>(grid.yBins.size() - 1) : 1.0;

    double mi = 0.0;
    for (std::size_t ix = 0; ix < grid.xBins.size(); ++ix) {
        for (std::size_t iy = 0; iy < grid.yBins.size(); ++iy) {
            double pxy = grid.at(ix, iy) * dx * dy / total;
            if (pxy < 1e-15) continue;
            double px = margX[ix] * dy / total;
            double py = margY[iy] * dx / total;
            if (px < 1e-15 || py < 1e-15) continue;
            mi += pxy * std::log2(pxy / (px * py));
        }
    }
    return mi;
}

DependenceAnalyzer::GridEntropy DependenceAnalyzer::entropyFromGrid(const KdeGrid& grid) {
    GridEntropy ent;
    if (grid.density.empty()) return ent;

    auto margX = grid.marginalX();
    auto margY = grid.marginalY();
    double total = grid.totalMass();
    if (total < 1e-15) return ent;

    double dx = (grid.xBins.size() > 1) ?
        (grid.xBins.back() - grid.xBins.front()) / static_cast<double>(grid.xBins.size() - 1) : 1.0;
    double dy = (grid.yBins.size() > 1) ?
        (grid.yBins.back() - grid.yBins.front()) / static_cast<double>(grid.yBins.size() - 1) : 1.0;

    // Joint entropy H(d, e)
    for (double d : grid.density) {
        double p = d * dx * dy / total;
        if (p > 1e-15) ent.joint += -p * std::log2(p);
    }

    // Marginal entropies
    double hX = 0.0, hY = 0.0;
    for (double m : margX) {
        double p = m * dy / total;
        if (p > 1e-15) hX += -p * std::log2(p);
    }
    for (double m : margY) {
        double p = m * dx / total;
        if (p > 1e-15) hY += -p * std::log2(p);
    }

    // Conditional entropy H(e | d) = H(d, e) - H(d)
    ent.conditional = ent.joint - hX;

    // Normalized MI = 2 * I(d;e) / (H(d) + H(e))
    double mi = hX + hY - ent.joint;
    double denom = hX + hY;
    ent.normalizedMI = (denom > 1e-15) ? (2.0 * mi / denom) : 0.0;

    return ent;
}

double DependenceAnalyzer::correlationRatio(
    const std::vector<double>& y,
    const std::vector<ConditionalStats>& conditional) {

    auto n = y.size();
    if (n < 2 || conditional.empty()) return 0.0;

    double meanE = mean(y);
    double varE = 0.0;
    for (double v : y) varE += (v - meanE) * (v - meanE);
    varE /= static_cast<double>(n);

    double varCondMean = 0.0;
    double totalMass = 0.0;
    for (const auto& cs : conditional) {
        if (!cs.valid) continue;
        varCondMean += cs.mass * (cs.meanY - meanE) * (cs.meanY - meanE);
        totalMass += cs.mass;
    }
    if (totalMass > 1e-15 && varE > 1e-15) {
        return (varCondMean / totalMass) / varE;
    }
    return 0.0;
}

//=============================================================================
// Composite compute
//=============================================================================

DependenceMetrics DependenceAnalyzer::compute(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const KdeGrid& grid,
    const std::vector<ConditionalStats>& conditional) {

    DependenceMetrics dep;
    auto n = std::min(x.size(), y.size());
    if (n < 2) return dep;

    dep.pearson = pearson(x, y);
    dep.spearman = spearmanRank(x, y);
    dep.kendall = kendallTau(x, y);
    dep.distanceCorrelation = distanceCorrelation(x, y);
    dep.mutualInformation = mutualInformationFromGrid(grid);

    auto ent = entropyFromGrid(grid);
    dep.jointEntropy = ent.joint;
    dep.conditionalEntropy = ent.conditional;
    dep.normalizedMutualInfo = ent.normalizedMI;

    dep.correlationRatio = correlationRatio(y, conditional);

    // Dependence index = normalized MI (symmetric, [0, 1])
    dep.dependenceIndex = dep.normalizedMutualInfo;

    return dep;
}

} // namespace tether::motion::replanner
