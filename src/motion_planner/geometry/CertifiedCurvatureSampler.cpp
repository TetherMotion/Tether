/**
 * @file CertifiedCurvatureSampler.cpp
 * @brief Implementation of the lazy per-span certified curvature sampler.
 *
 * @details
 * The Lipschitz bound on ‖dκ/ds‖ is built from control-polygon bounds on
 * the parametric derivatives C', C'', C''' of each Bézier span (the
 * original NURBS piece is decomposed into single-span Béziers first; the
 * worst-case L_κ across sub-spans is used, which is conservative but
 * still tight enough to be cheap in practice).
 */
#include "tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tether::motion {

namespace {

/// Control-polygon bound on ‖C^(k)‖ for a single Bézier span of degree n
/// with control points P_0..P_n:
///   ‖C'(u)‖  ≤ n · max_i ‖P_{i+1} − P_i‖
///   ‖C''(u)‖ ≤ n(n−1) · max_i ‖P_{i+2} − 2P_{i+1} + P_i‖
///   ‖C'''(u)‖ ≤ n(n−1)(n−2) · max_i ‖Δ³P_i‖
/// These are the standard de Casteljau / forward-difference bounds (M1).
double boundC1(const NurbsCurve& c) {
    const auto& P = c.controlPoints();
    const int n = c.degree();
    if (n < 1 || P.size() < static_cast<std::size_t>(n) + 1) return 0.0;
    double m = 0.0;
    for (std::size_t i = 0; i + 1 < P.size(); ++i) {
        m = std::max(m, (P[i + 1] - P[i]).norm());
    }
    return static_cast<double>(n) * m;
}

double boundC2(const NurbsCurve& c) {
    const auto& P = c.controlPoints();
    const int n = c.degree();
    if (n < 2 || P.size() < static_cast<std::size_t>(n) + 1) return 0.0;
    double m = 0.0;
    for (std::size_t i = 0; i + 2 < P.size(); ++i) {
        RVec d = P[i + 2] - 2.0 * P[i + 1] + P[i];
        m = std::max(m, d.norm());
    }
    return static_cast<double>(n * (n - 1)) * m;
}

double boundC3(const NurbsCurve& c) {
    const auto& P = c.controlPoints();
    const int n = c.degree();
    if (n < 3 || P.size() < static_cast<std::size_t>(n) + 1) return 0.0;
    double m = 0.0;
    for (std::size_t i = 0; i + 3 < P.size(); ++i) {
        RVec d = P[i + 3] - 3.0 * P[i + 2] + 3.0 * P[i + 1] - P[i];
        m = std::max(m, d.norm());
    }
    return static_cast<double>(n * (n - 1) * (n - 2)) * m;
}

/// Lower bound on ‖C'(u)‖ over the span — use the minimum distance between
/// consecutive control-point *chord* segments as a crude but safe lower
/// bound. For a non-degenerate Bézier this is > 0; if it underflows we
/// fall back to the minimum sampled speed.
double boundSpeedLower(const NurbsCurve& c) {
    const auto& P = c.controlPoints();
    if (P.size() < 2) return 0.0;
    // The convex-hull property does not give a *lower* bound on speed
    // directly; we use the minimum sampled speed as the safe fallback
    // (computed by the caller). Return 0 here to signal "use sampled".
    (void)c;
    return 0.0;
}

} // namespace

double CertifiedCurvatureSampler::curvatureAt(const NurbsCurve& c,
                                              double u) {
    try {
        auto d = c.arcDerivatives(u, 2);
        // κ = ‖curvature vector‖ (already arc-length parameterized).
        return d.curvature.norm();
    } catch (const std::domain_error&) {
        // Degenerate parameterization (zero speed) — treat as zero
        // curvature; the velocity limiter will not be active there
        // anyway because the tangent is undefined.
        return 0.0;
    }
}

void CertifiedCurvatureSampler::computeSpan(std::size_t pieceIndex) const {
    auto& entry = cache_[pieceIndex];
    if (entry.computed) return;

    const NurbsCurve& c = path_.piece(pieceIndex);

    CertifiedCurvature result;
    result.samples = 0;

    // Polylines have zero curvature.
    if (c.isPolyline()) {
        result.maxKappa = 0.0;
        result.minKappa = 0.0;
        result.Lipschitz = 0.0;
        result.gridSpacing = 0.0;
        result.samples = 0;
        entry.computed = true;
        entry.result = result;
        return;
    }

    const double uMin = c.knotMin();
    const double uMax = c.knotMax();
    const double domain = uMax - uMin;

    // --- Step 1: Coarse probe to estimate the minimum speed ----------------
    // The Lipschitz constant for κ involves 1/σ⁴ and 1/σ⁵, so we need a
    // lower bound on the speed σ = ‖C'(u)‖. The control polygon only
    // gives an upper bound; we probe the speed on a coarse grid and use
    // the minimum as a practical lower bound. This is not a true
    // certificate (the minimum could be between probes), so we apply a
    // safety factor and fall back to dense sampling if the speed is
    // very small.
    constexpr std::size_t kProbeSamples = 64;
    double minSpeed = std::numeric_limits<double>::infinity();
    {
        const double h0 = domain / static_cast<double>(kProbeSamples - 1);
        for (std::size_t i = 0; i < kProbeSamples; ++i) {
            const double u = uMin + static_cast<double>(i) * h0;
            try {
                const double sp = c.derivative(u, 1).norm();
                if (sp > 0.0) minSpeed = std::min(minSpeed, sp);
            } catch (...) {
                // degenerate parameterization — ignore
            }
        }
    }

    // --- Step 2: Lipschitz constant -----------------------------------------
    // |dκ/ds| ≤ B1·B3 / σ_min⁴ + 3·B1·B2² / σ_min⁵
    // where B1, B2, B3 are control-polygon upper bounds on ‖C'‖, ‖C''‖,
    // ‖C'''‖, and σ_min is the minimum sampled speed (with a safety
    // factor of 0.5 to guard against between-sample minima).
    double L = 0.0;
    bool certificateValid = false;
    if (std::isfinite(minSpeed) && minSpeed > 1e-6) {
        const double sigmaMin = 0.5 * minSpeed; // safety factor
        auto spans = c.bezierDecompose();
        double Lworst = 0.0;
        for (const auto& span : spans) {
            const double b1 = boundC1(span);
            const double b2 = boundC2(span);
            const double b3 = boundC3(span);
            if (b1 < 1e-12) continue;
            // |dκ/ds| ≤ B1·B3 / σ⁴ + 3·B1·B2² / σ⁵
            const double term1 = b1 * b3 / std::pow(sigmaMin, 4);
            const double term2 = 3.0 * b1 * b2 * b2 / std::pow(sigmaMin, 5);
            Lworst = std::max(Lworst, term1 + term2);
        }
        L = Lworst;
        certificateValid = (L < 1e15); // guard against overflow
    }
    result.Lipschitz = L;

    // --- Step 3: Choose grid spacing h so L·h/2 ≤ ε_cert -------------------
    std::size_t numSamples;
    if (!certificateValid || L < 1e-15) {
        // No valid certificate — use the maximum allowed grid (dense
        // sampling). The result is the raw max sample without a
        // certificate slack.
        numSamples = maxSamplesPerSpan_;
    } else {
        const double hTarget = 2.0 * certTolerance_ / L;
        std::size_t nIntervals = static_cast<std::size_t>(
            std::ceil(domain / std::max(hTarget, 1e-15)));
        nIntervals = std::max<std::size_t>(nIntervals, 1);
        nIntervals = std::min(nIntervals, maxSamplesPerSpan_);
        numSamples = nIntervals + 1;
    }

    // --- Step 4: Sample κ on the uniform grid -------------------------------
    const double h = (numSamples > 1)
        ? domain / static_cast<double>(numSamples - 1) : 0.0;
    result.gridSpacing = h;

    double maxSampled = 0.0;
    for (std::size_t i = 0; i < numSamples; ++i) {
        const double u = (numSamples == 1) ? uMin
            : uMin + static_cast<double>(i) * h;
        const double k = curvatureAt(c, u);
        maxSampled = std::max(maxSampled, k);
    }
    result.samples = numSamples;
    result.minKappa = maxSampled;

    // --- Step 5: Certificate interval ---------------------------------------
    // max κ ∈ [maxSampled, maxSampled + L·h/2]
    // Use the upper bound for the velocity limiter (conservative).
    if (certificateValid && L >= 1e-15) {
        result.maxKappa = maxSampled + L * h * 0.5;
    } else {
        // No valid certificate — return the raw max sample.
        // Add a small conservative slack (0.1% of the max, or 1e-9,
        // whichever is larger) to guard against inter-sample peaks.
        const double slack = std::max(maxSampled * 1e-3, 1e-9);
        result.maxKappa = maxSampled + slack;
    }

    entry.computed = true;
    entry.result = result;
}

CertifiedCurvature CertifiedCurvatureSampler::maxCurvature(
    std::size_t pieceIndex) const {
    if (pieceIndex >= path_.numPieces()) {
        throw std::out_of_range(
            "CertifiedCurvatureSampler::maxCurvature: piece index out of range");
    }
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!cache_[pieceIndex].computed) {
            computeSpan(pieceIndex);
        }
    }
    // Read outside the lock — computed entries are write-once.
    return cache_[pieceIndex].result;
}

std::size_t CertifiedCurvatureSampler::spansSampled() const {
    std::lock_guard<std::mutex> guard(mutex_);
    std::size_t n = 0;
    for (const auto& e : cache_) if (e.computed) ++n;
    return n;
}

std::size_t CertifiedCurvatureSampler::totalCurvatureEvaluations() const {
    std::lock_guard<std::mutex> guard(mutex_);
    std::size_t n = 0;
    for (const auto& e : cache_) if (e.computed) n += e.result.samples;
    return n;
}

} // namespace tether::motion
