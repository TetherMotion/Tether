/**
 * @file Bernstein.cpp
 * @brief Implementation of tether::motion::bernstein (see header for docs).
 *
 * Equation numbers (G.x) refer to docs/motion/GeometryFoundations.md.
 */

#include "tether/motion_planner/geometry/Bernstein.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tether::motion::bernstein {

namespace {

// Binomial coefficient as double (exact for n ≤ ~50 in double precision).
double binom(int n, int k) {
    if (k < 0 || k > n) return 0.0;
    if (k > n - k) k = n - k;
    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
        result = result * static_cast<double>(n - k + i) / static_cast<double>(i);
    }
    return result;
}

constexpr int kMaxSubdivisionDepth = 60;

void isolateRecursive(const std::vector<double>& coeffs, double lo, double hi,
                      double tol, int depth, std::vector<Interval>& out) {
    // Convex-hull pruning: strict single sign ⇒ no root inside (eq. (G.6)).
    double mn = coeffs[0], mx = coeffs[0];
    for (double c : coeffs) {
        mn = std::min(mn, c);
        mx = std::max(mx, c);
    }
    if (mn > 0.0 || mx < 0.0) return;

    if (hi - lo <= tol || depth >= kMaxSubdivisionDepth) {
        out.push_back({lo, hi});
        return;
    }

    const double mid = 0.5 * (lo + hi);
    auto halves = subdivide(coeffs, 0.5);
    isolateRecursive(halves.first, lo, mid, tol, depth + 1, out);
    isolateRecursive(halves.second, mid, hi, tol, depth + 1, out);
}

} // namespace

double evaluate(const std::vector<double>& coeffs, double t) {
    // De Casteljau: repeated linear interpolation of the coefficients.
    if (coeffs.empty()) return 0.0;
    std::vector<double> d(coeffs);
    const std::size_t n = d.size() - 1;
    for (std::size_t r = 1; r <= n; ++r) {
        for (std::size_t i = 0; i + r <= n; ++i) {
            d[i] = (1.0 - t) * d[i] + t * d[i + 1];
        }
    }
    return d[0];
}

std::vector<double> derivative(const std::vector<double>& coeffs) {
    if (coeffs.size() < 2) return {0.0};
    const double n = static_cast<double>(coeffs.size() - 1);
    std::vector<double> out(coeffs.size() - 1);
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = n * (coeffs[i + 1] - coeffs[i]); // eq. (G.4)
    }
    return out;
}

std::pair<std::vector<double>, std::vector<double>>
subdivide(const std::vector<double>& coeffs, double t) {
    if (coeffs.empty()) return {{}, {}};
    // Full De Casteljau triangle: left edge = first column, right edge = last.
    const std::size_t n = coeffs.size() - 1;
    std::vector<std::vector<double>> d(n + 1);
    d[0] = coeffs;
    for (std::size_t r = 1; r <= n; ++r) {
        d[r].resize(n + 1 - r);
        for (std::size_t i = 0; i + r <= n; ++i) {
            d[r][i] = (1.0 - t) * d[r - 1][i] + t * d[r - 1][i + 1];
        }
    }
    std::vector<double> left(n + 1), right(n + 1);
    for (std::size_t i = 0; i <= n; ++i) {
        left[i] = d[i][0];
        right[i] = d[n - i][i];
    }
    return {std::move(left), std::move(right)};
}

std::vector<double> multiply(const std::vector<double>& a,
                             const std::vector<double>& b) {
    if (a.empty() || b.empty()) return {};
    const int m = static_cast<int>(a.size()) - 1;
    const int n = static_cast<int>(b.size()) - 1;
    std::vector<double> out(m + n + 1, 0.0);
    for (int k = 0; k <= m + n; ++k) {
        double sum = 0.0;
        const int iLo = std::max(0, k - n);
        const int iHi = std::min(m, k);
        for (int i = iLo; i <= iHi; ++i) {
            const int j = k - i;
            // eq. (G.7): C(m,i)·C(n,j)/C(m+n,k)
            sum += binom(m, i) * binom(n, j) / binom(m + n, k) * a[i] * b[j];
        }
        out[k] = sum;
    }
    return out;
}

std::vector<double> powerToBernstein(const std::vector<double>& power) {
    if (power.empty()) return {};
    const int n = static_cast<int>(power.size()) - 1;
    std::vector<double> out(n + 1, 0.0);
    for (int i = 0; i <= n; ++i) {
        double sum = 0.0;
        for (int j = 0; j <= i; ++j) {
            sum += power[j] * binom(i, j) / binom(n, j); // eq. (G.8)
        }
        out[i] = sum;
    }
    return out;
}

std::vector<double> bernsteinToPower(const std::vector<double>& coeffs) {
    if (coeffs.empty()) return {};
    const int n = static_cast<int>(coeffs.size()) - 1;
    std::vector<double> out(n + 1, 0.0);
    for (int m = 0; m <= n; ++m) {
        double sum = 0.0;
        for (int i = 0; i <= m; ++i) {
            const double sign = ((m - i) % 2 == 0) ? 1.0 : -1.0;
            sum += coeffs[i] * binom(n, i) * binom(n - i, m - i) * sign; // (G.9)
        }
        out[m] = sum;
    }
    return out;
}

std::vector<Interval> isolateRoots(const std::vector<double>& coeffs,
                                   double tol) {
    if (!(tol > 0.0)) {
        throw std::invalid_argument("bernstein::isolateRoots: tol must be > 0");
    }
    if (coeffs.empty()) return {};

    // Degenerate zero polynomial: every point is a root; report the whole
    // interval once (callers special-case this).
    bool allZero = true;
    for (double c : coeffs) {
        if (c != 0.0) {
            allZero = false;
            break;
        }
    }
    if (allZero) return {{0.0, 1.0}};

    std::vector<Interval> raw;
    isolateRecursive(coeffs, 0.0, 1.0, tol, 0, raw);

    // Merge adjacent/overlapping dyadic intervals: they cover one root or an
    // unresolvable cluster (documented near-multiple-root behavior).
    std::vector<Interval> merged;
    for (const Interval& iv : raw) {
        if (!merged.empty() && iv.lo <= merged.back().hi + tol) {
            merged.back().hi = std::max(merged.back().hi, iv.hi);
        } else {
            merged.push_back(iv);
        }
    }
    return merged;
}

} // namespace tether::motion::bernstein
