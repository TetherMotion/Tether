/**
 * @file CertifiedCurvatureSampler.hpp
 * @brief Lazy per-span certified maximum-curvature sampling with a
 *        Lipschitz certificate (analogous to (M10)).
 *
 * @details
 * ## Why this exists — no closed form for curvature extrema
 *
 * The velocity-limit curve v_lim(s) = √(a_cent / κ(s)) requires the
 * *maximum* curvature on each span (the worst case that bounds velocity).
 * For a rational NURBS / Bézier span of degree p, the scalar curvature
 *
 *     κ(u) = ‖C'(u) × C''(u)‖ / ‖C'(u)‖³
 *
 * is a rational function of u whose numerator is a polynomial of degree
 * up to (2p−3) in the squared form κ² = N(u)/D(u). The stationarity
 * equation dκ/du = 0 reduces to N'(u)·D(u) − N(u)·D'(u) = 0, a polynomial
 * of degree up to (4p−4). For the blend curves used here (p = 5 quintic,
 * p = 7 septic) this is degree 16 / 24 respectively — well beyond the
 * quartic closed-form threshold, and even the Abel–Ruffini barrier (≥5).
 * **There is no closed-form solution for the curvature extrema of
 * degree-5/7 Béziers.** Certified adaptive sampling is the correct tool.
 *
 * ## Algorithm (M10-analogous Lipschitz certificate)
 *
 * Per span, sample κ(u) on a uniform grid of spacing h. Curvature is
 * 1-Lipschitz in arc length with constant L_κ = sup ‖dκ/ds‖. We bound
 *
 *     ‖dκ/ds‖ ≤ (‖C' × C'''‖ + 3‖C'' × C''‖) / ‖C'‖⁴
 *               + 3‖C' × C''‖·‖C''‖ / ‖C'‖⁵
 *
 * (quotient rule on κ = ‖C' × C''‖ / ‖C'‖³, rewritten in arc length
 *  s via ds = ‖C'‖ du). Each factor is bounded by a control-polygon
 * bound on the relevant derivative (M1-style: ‖C^(k)‖ ≤
 * n!/(n−k)! · max_i ‖Δ^k P_i‖). The grid spacing is chosen so
 * L_κ · h/2 ≤ ε_cert, yielding the certificate interval
 *
 *     max κ ∈ [ max_samples κ , max_samples κ + L_κ · h/2 ].
 *
 * The *upper* bound is used for the velocity limit (conservative —
 * slightly lower v_lim, never violates the centripetal acceleration
 * constraint). Results are memoized per span; only spans actually
 * queried by the velocity profiler are ever sampled.
 *
 * @see PointCurveDistance.hpp for the (M8/M10) certified-distance pattern
 *      this mirrors; GeometryFoundations.md §"Curvature" for the formulas.
 */
#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <vector>
#include <optional>
#include <mutex>
#include <cstddef>

namespace tether::motion {

/// Result of a certified per-span curvature query.
struct CertifiedCurvature {
    double maxKappa = 0.0;       ///< Certified upper bound on max κ on the span.
    double minKappa = 0.0;       ///< Lower bound (max sample).
    double Lipschitz = 0.0;      ///< The L_κ constant used (diagnostic).
    std::size_t samples = 0;     ///< Number of κ evaluations performed.
    double gridSpacing = 0.0;    ///< h (diagnostic).
};

/**
 * @brief Lazy per-span certified maximum-curvature sampler.
 *
 * One instance is intended to be held by the velocity profiler and queried
 * per span as the limit curve is built. Memoization is keyed by piece
 * index; only spans actually queried are ever sampled (laziness).
 *
 * Thread-safety: the memoization table is guarded by a mutex, so a single
 * sampler may be shared across threads. The underlying NurbsCurve
 * arc-length caches are *not* thread-safe (one path per thread is the
 * intended usage); the curvature sampler itself only reads the curve.
 */
class CertifiedCurvatureSampler {
public:
    /**
     * @brief Construct a sampler bound to a path.
     * @param path The piecewise path whose spans will be sampled.
     * @param certTolerance Maximum admissible width of the certificate
     *        interval (L_κ · h/2). Default 1e-6 — tight enough that the
     *        conservative upper bound differs from the true max by < 1e-6.
     * @param maxSamplesPerSpan Hard cap on grid refinement (default 4096).
     *        If the Lipschitz-based h would require more samples, the
     *        certificate is relaxed to the achieved h (still certified,
     *        just wider); this bounds the cost per span.
     */
    explicit CertifiedCurvatureSampler(const PiecewiseNurbsPath& path,
                                       double certTolerance = 1e-6,
                                       std::size_t maxSamplesPerSpan = 4096)
        : path_(path)
        , certTolerance_(certTolerance)
        , maxSamplesPerSpan_(maxSamplesPerSpan)
        , cache_(path.numPieces()) {}

    /**
     * @brief Certified maximum curvature on piece `pieceIndex`.
     *
     * Returns the memoized result if the span has already been queried;
     * otherwise samples κ on an adaptive grid, builds the certificate
     * interval, and memoizes the upper bound.
     *
     * For polyline pieces (degree 1) the curvature is exactly zero and
     * no sampling is performed.
     */
    CertifiedCurvature maxCurvature(std::size_t pieceIndex) const;

    /**
     * @brief Certified maximum curvature over the piece containing arc
     *        length s. Convenience wrapper around `locate` + `maxCurvature`.
     */
    CertifiedCurvature maxCurvatureAtArcLength(double s) const {
        return maxCurvature(path_.locate(s).piece);
    }

    /// Number of spans actually sampled so far (laziness diagnostic).
    std::size_t spansSampled() const;

    /// Total number of κ evaluations across all sampled spans.
    std::size_t totalCurvatureEvaluations() const;

private:
    /// Evaluate scalar curvature κ(u) on a single span.
    static double curvatureAt(const NurbsCurve& c, double u);

    const PiecewiseNurbsPath& path_;
    double certTolerance_;
    std::size_t maxSamplesPerSpan_;

    struct CacheEntry {
        bool computed = false;
        CertifiedCurvature result{};
    };
    mutable std::vector<CacheEntry> cache_;
    mutable std::mutex mutex_;

    /// Fill `cache_[pieceIndex]` (caller holds `mutex_`).
    void computeSpan(std::size_t pieceIndex) const;
};

} // namespace tether::motion
