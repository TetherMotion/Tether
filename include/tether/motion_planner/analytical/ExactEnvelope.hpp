/**
 * @file ExactEnvelope.hpp
 * @brief Certified velocity-envelope analysis for piecewise NURBS paths —
 *        no dense sampling anywhere.
 *
 * @details
 * The velocity limit on a piecewise path is a pointwise minimum of terms
 * with very different structure:
 *
 *   - global caps       — feed rate, path velocity limit        (constant)
 *   - per-segment feeds — piecewise-constant adapter data       (discrete)
 *   - corner stamps     — point constraints at piece junctions  (discrete)
 *   - curvature         — v ≤ sqrt(a_cent / κ(s))               (rational in t)
 *   - per-axis velocity — v ≤ vmax_i / |T_i(s)|                 (rational in t)
 *
 * The discrete terms are already exact data; the two geometric terms are
 * *closed-form rational functions of the curve parameter t* on every
 * Bézier span of every piece. Their extrema and all pairwise crossings are
 * therefore exactly computable: on a polynomial Bézier span of degree n,
 *
 *   κ² = P/R³        with P = |C'×C''|²  (deg 4n−6), R = |C'|² (deg 2n−2)
 *   T_i² = G_i/R     with G_i = C_i'²
 *
 * and stationarity of κ² reduces to  P'R − 3PR' = 0  (deg ≤ 6n−9 = 9 for
 * cubics); stationarity of T_i² to  G_i'R − G_iR' = 0 (deg ≤ 4n−5 = 7).
 * For rational spans (non-constant weights — circular arcs) the same
 * equations hold in homogeneous form:
 *
 *   X'_d = B_d/w²,   B_d = A'_d w − A_d w'    (deg 2n−1)
 *   X''_d = E_d/w³,  E_d = B'_d w − 2B_d w'   (deg 3n−2)
 *   κ² = F/R³  with  F = U·w², U = Σ(B_iE_j − B_jE_i)² (deg 12n−6),
 *        R = Σ B_d² (deg 4n−2)
 *
 * Term crossings (where the binding term of the envelope changes) are
 * likewise polynomial equations: cap↔κ, cap↔axis_i, axis_i↔axis_j,
 * κ↔axis_i all reduce to one Bernstein coefficient vector per span.
 *
 * All polynomials are built *directly in Bernstein basis* on each span
 * (exact products via bernstein::multiply), roots are found by certified
 * Bernstein root isolation (never misses a root), polished by Newton, and
 * unresolvable clusters contribute their interval ends (bounded by the
 * isolating width — conservative via the neighboring candidates).
 * The only non-symbolic step is the s ↔ t arc-length map on each piece
 * (adaptive quadrature + Newton — inherent to NURBS, amortized by the
 * piece's LUT).
 *
 * Output: the sorted candidate set (all envelope extrema + endpoints +
 * discrete breakpoints), its local minima/maxima, and the certified
 * global minimum — everything the snapspace solver needs for bounds and
 * segmentation, at a few evaluations per piece instead of a 1024-point
 * scan plus per-sub-path grids.
 */
#pragma once

#include "../PathAdapter.hpp"
#include "ConstraintEvaluator.hpp"
#include "tether/motion_planner/geometry/Bernstein.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace MotionPlanner::analytical {

namespace envelope_detail {

namespace bn = tether::motion::bernstein;
using tether::motion::NurbsCurve;

/** Bernstein coefficients of p². */
inline std::vector<double> square(const std::vector<double>& p) {
    return bn::multiply(p, p);
}

/** c += k·p (sizes must match). */
inline void addScaled(std::vector<double>& c,
                      const std::vector<double>& p, double k) {
    for (std::size_t i = 0; i < c.size(); ++i) c[i] += k * p[i];
}

/** f − g coefficient-wise (equal sizes). */
inline std::vector<double> diff(const std::vector<double>& f,
                                const std::vector<double>& g) {
    std::vector<double> d(f.size());
    for (std::size_t i = 0; i < f.size(); ++i) d[i] = f[i] - g[i];
    return d;
}

/** True when the polynomial is numerically the zero polynomial. */
inline bool isZero(const std::vector<double>& p) {
    double m = 0.0;
    for (double c : p) m = std::max(m, std::abs(c));
    return m <= 1e-14;
}

/**
 * Per-span Bernstein data. Unified forms used by every equation:
 *   κ²(t) = F(t)/R(t)³,   T_i²(t) = G_i(t)/R(t)
 * (polynomial curve: F = |C'×C''|², R = |C'|², G_i = C_i'²;
 *  rational curve:   F = U·w²,       R = Σ B_d², G_i = B_i²
 *  with B_d = A'_d w − A_d w' etc. — see file header.)
 */
struct SpanPolys {
    double ua = 0.0, ub = 1.0;
    bool rational = false;
    std::vector<double> R;
    std::vector<double> F;
    std::vector<std::vector<double>> G;
    /**
     * Per-axis signed curvature-component numerators (polynomial spans
     * only): κ_i = M_i/R² with M_i = C_i''·R − C_i'·Q, Q = Σ C_d'·C_d''.
     * Empty on rational spans (arcs have constant κ — no probes needed).
     */
    std::vector<std::vector<double>> M;
    const NurbsCurve* curve = nullptr;  /**< span curve for evaluation */
};

/**
 * Build the Bernstein polynomials of one Bézier span. Control points are
 * normalized (translate to cp[0], scale by max extent) before polynomial
 * construction — the rational-span degrees are high and coefficient
 * conditioning improves by orders of magnitude in unit coordinates.
 * Roots live in t ∈ [0,1] regardless of normalization; candidate values
 * are evaluated on the un-normalized span curve.
 */
inline SpanPolys buildSpan(const NurbsCurve& span, std::size_t dim) {
    SpanPolys sp;
    sp.ua = span.knotMin();
    sp.ub = span.knotMax();
    sp.curve = &span;
    const std::size_t n = static_cast<std::size_t>(span.degree());
    const auto& cps = span.controlPoints();
    const auto& wts = span.weights();

    double scale = 0.0;
    for (const auto& p : cps) {
        for (std::size_t d = 0; d < dim && d < p.dim(); ++d) {
            scale = std::max(scale,
                std::abs(p.unchecked(d) - cps[0].unchecked(d)));
        }
    }
    if (!(scale > 0.0)) scale = 1.0;

    sp.rational = false;
    for (double w : wts) {
        if (std::abs(w - wts[0]) > 1e-12) { sp.rational = true; break; }
    }

    // Normalized control polygon + homogeneous numerator A_d = w·C_d.
    std::vector<std::vector<double>> cp(dim), a(dim);
    for (std::size_t d = 0; d < dim; ++d) {
        cp[d].resize(n + 1);
        a[d].resize(n + 1);
        for (std::size_t i = 0; i <= n; ++i) {
            const double pn =
                (d < cps[i].dim()
                     ? (cps[i].unchecked(d) - cps[0].unchecked(d)) / scale
                     : 0.0);
            cp[d][i] = pn;
            a[d][i] = wts[i] * pn;
        }
    }

    if (!sp.rational) {
        // Polynomial Bézier: the curve's own CPs are cp (weights uniform).
        std::vector<std::vector<double>> D(dim), E(dim);
        for (std::size_t d = 0; d < dim; ++d) {
            D[d] = bn::derivative(cp[d]);          // deg n−1
            E[d] = bn::derivative(D[d]);           // deg n−2
        }
        sp.R.assign(n >= 1 ? 2 * n - 1 : 1, 0.0);   // Σ D_d², deg 2n−2
        sp.G.resize(dim);
        for (std::size_t d = 0; d < dim; ++d) {
            sp.G[d] = square(D[d]);
            addScaled(sp.R, sp.G[d], 1.0);
        }
        const std::size_t fDeg = (n >= 2) ? 4 * n - 6 : 0;
        sp.F.assign(fDeg + 1, 0.0);                  // |C'×C''|², deg 4n−6
        for (std::size_t i = 0; i < dim; ++i) {
            for (std::size_t j = i + 1; j < dim; ++j) {
                auto tij = diff(bn::multiply(D[i], E[j]),
                                bn::multiply(D[j], E[i]));
                addScaled(sp.F, square(tij), 1.0);
            }
        }
        // Per-axis curvature-component numerators for authority probes:
        // Q = C'·C'' (deg 2n−3), M_i = C_i''·R − C_i'·Q (deg 3n−4).
        if (n >= 2) {
            std::vector<double> Q(2 * n - 2, 0.0);
            for (std::size_t d = 0; d < dim; ++d) {
                addScaled(Q, bn::multiply(D[d], E[d]), 1.0);
            }
            sp.M.resize(dim);
            for (std::size_t i = 0; i < dim; ++i) {
                sp.M[i] = diff(bn::multiply(E[i], sp.R),
                               bn::multiply(D[i], Q));
            }
        }
        return sp;
    }

    // Rational curve — homogeneous construction.
    std::vector<double> w(wts.begin(), wts.end());
    const std::vector<double> wP = bn::derivative(w);
    std::vector<std::vector<double>> B(dim), E(dim);
    for (std::size_t d = 0; d < dim; ++d) {
        const auto aP = bn::derivative(a[d]);
        B[d] = diff(bn::multiply(aP, w),              // A'_d w − A_d w'
                    bn::multiply(a[d], wP));
        const auto bP = bn::derivative(B[d]);
        auto e = bn::multiply(bP, w);                 // B'_d w − 2 B_d w'
        const auto bw = bn::multiply(B[d], wP);
        for (std::size_t i = 0; i < e.size(); ++i) e[i] -= 2.0 * bw[i];
        E[d] = std::move(e);
    }
    sp.R.assign(4 * n - 1, 0.0);                      // Σ B_d², deg 4n−2
    sp.G.resize(dim);
    for (std::size_t d = 0; d < dim; ++d) {
        sp.G[d] = square(B[d]);
        addScaled(sp.R, sp.G[d], 1.0);
    }
    std::vector<double> U(10 * n - 5, 0.0);           // Σ cross², deg 10n−6
    for (std::size_t i = 0; i < dim; ++i) {
        for (std::size_t j = i + 1; j < dim; ++j) {
            auto tij = diff(bn::multiply(B[i], E[j]),
                            bn::multiply(B[j], E[i]));
            addScaled(U, square(tij), 1.0);
        }
    }
    sp.F = bn::multiply(U, square(w));                // deg 12n−6
    return sp;
}

/** isolateRoots + Newton polish; degenerate/empty polys yield no roots. */
inline std::vector<double>
findRoots(const std::vector<double>& p, double tol) {
    std::vector<double> out;
    if (p.empty() || isZero(p)) return out;
    const auto iv = bn::isolateRoots(p, tol);
    const auto dp = bn::derivative(p);
    for (const auto& i : iv) {
        double t = 0.5 * (i.lo + i.hi);
        for (int k = 0; k < 25; ++k) {
            const double fv = bn::evaluate(p, t);
            const double dfv = bn::evaluate(dp, t);
            if (dfv == 0.0) break;
            const double tn = std::clamp(t - fv / dfv, i.lo, i.hi);
            if (std::abs(tn - t) <= 1e-15) { t = tn; break; }
            t = tn;
        }
        out.push_back(std::clamp(t, 0.0, 1.0));
    }
    return out;
}

} // namespace envelope_detail

/**
 * @brief Certified geometric velocity-envelope analysis of a path.
 *
 * One pass yields every point where the velocity limit can be minimal or
 * change its binding term: all stationary points of the curvature and
 * per-axis terms, all pairwise term crossings, all discrete breakpoints
 * (segment-feed steps, corner stamps), and the span/piece endpoints.
 * Between consecutive candidates the envelope is single-term and
 * monotone, so local extrema classify by neighbor comparison.
 */
template <size_t Dim, typename T = double>
class ExactEnvelope {
public:
    using Path = PathAdapter<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;
    using Segment = PathAdapterSegment<Dim, T>;
    using NurbsCurve = tether::motion::NurbsCurve;

    enum class Term : uint8_t {
        Cap,        /**< feed/path/segment cap */
        Curvature,  /**< centripetal term sqrt(a_c/κ) */
        Axis,       /**< per-axis velocity term */
        Stamp       /**< corner-velocity stamp */
    };

    struct Candidate {
        double s = 0.0;
        double v = 0.0;      /**< envelope value at s (stamps folded) */
        double vg = 0.0;     /**< geometric value at s (pre-stamp) */
        Term term = Term::Cap;
    };

    struct Result {
        std::vector<Candidate> candidates;
        std::vector<Candidate> localMinima;
        std::vector<Candidate> localMaxima;
        double vMin = std::numeric_limits<double>::infinity();
        double sAtMin = 0.0;
        /**
         * s-positions where the coupled per-axis bounds can dip WITHOUT
         * the velocity envelope itself moving: stationary points of the
         * per-axis curvature components κ_i² and of the products
         * κ²·v_axis_j⁴ (the centripetal consumption |κ_i|v² that eats
         * acceleration authority, and its jerk-level analogues). The
         * planner evaluates accelerationBounds/etaBounds at
         * candidates ∪ authorityProbes — strictly better placed than a
         * uniform grid, and covers interior authority dips that pure
         * envelope extrema miss (κ rising while an axis term binds).
         */
        std::vector<double> authorityProbes;
    };

    struct Config {
        double rootTol = 1e-9;      /**< t-space isolation width per span */
        double mergeTolS = 0.0;     /**< s-space dedupe window (0 → auto) */
    };

    /**
     * @brief Analyse the path over [sA, sB].
     */
    Result analyze(const Path& path, const Limits& limits, T feedRate,
                   double sA, double sB,
                   const Config& cfg = Config{}) const {
        Result res;
        const auto& inner = path.inner();
        if (inner.numPieces() == 0 || !(sB > sA)) return res;

        const double capGlobal = std::min(
            static_cast<double>(feedRate),
            static_cast<double>(limits.path.maxPathVelocity));
        const double aCent =
            static_cast<double>(limits.path.maxCentripetalAcceleration);

        std::vector<Candidate>& cand = res.candidates;

        double sPiece = 0.0;
        for (std::size_t pi = 0; pi < inner.numPieces(); ++pi) {
            const NurbsCurve& piece = inner.pieces()[pi];
            const double len = piece.length();
            const double sLo = sPiece, sHi = sPiece + len;
            sPiece = sHi;
            if (sHi < sA - 1e-12 || sLo > sB + 1e-12 || !(len > 0.0))
                continue;

            const double sL = std::max(sLo, sA);
            const double sR = std::min(sHi, sB);
            const double uLo = (sL <= sLo + 1e-12)
                ? piece.knotMin()
                : piece.invertLength(sL - sLo);
            const double uHi = (sR >= sHi - 1e-12)
                ? piece.knotMax()
                : piece.invertLength(sR - sLo);

            // Per-piece constant cap (segment feeds are piecewise-const —
            // a step can only occur at a piece boundary).
            const double capPiece = std::min(
                capGlobal,
                static_cast<double>(path.maxVelocityAtArcLength(
                    static_cast<T>(0.5 * (sLo + sHi)))));

            if (piece.degree() >= 2) {
                for (const NurbsCurve& span : piece.bezierDecompose()) {
                    if (span.knotMax() < uLo - 1e-12 ||
                        span.knotMin() > uHi + 1e-12) continue;
                    emitSpanCandidates(
                        envelope_detail::buildSpan(span, Dim),
                        path, piece, sLo, sL, sR, uLo, uHi,
                        capPiece, aCent, limits, cfg, cand,
                        res.authorityProbes);
                }
            } else {
                // Degree-1 piece: κ = 0, T constant — only endpoints.
                for (double u : {piece.knotMin(), piece.knotMax()}) {
                    const double s = sLo + piece.arcLengthTo(u);
                    if (s >= sL - 1e-9 && s <= sR + 1e-9) {
                        cand.push_back(evalAt(path, piece, u, s,
                                              capPiece, aCent, limits));
                    }
                }
            }
        }

        foldStamps(path, path.segments(), path.cornerVelocities(),
                   sA, sB, capGlobal, aCent, limits, cand);

        std::sort(cand.begin(), cand.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.s < b.s;
                  });
        const double mergeTol = cfg.mergeTolS > 0.0
            ? cfg.mergeTolS
            : 1e-7 * std::max(1.0, sB - sA);
        std::vector<Candidate> merged;
        merged.reserve(cand.size());
        for (const auto& c : cand) {
            if (!merged.empty() && c.s - merged.back().s < mergeTol) {
                // Coincident candidates are one-sided evaluations at a
                // junction: keep the minimum folded value, but merge the
                // geometric value independently — the interior infimum
                // needs the TIGHTER side's limit, not whichever value the
                // survivor happened to carry (both fold to the same stamp
                // and either vg could otherwise survive).
                const double vgMin = std::min(merged.back().vg, c.vg);
                if (c.v < merged.back().v) merged.back() = c;
                merged.back().vg = vgMin;
            } else {
                merged.push_back(c);
            }
        }
        cand = std::move(merged);

        for (std::size_t i = 0; i < cand.size(); ++i) {
            const auto& c = cand[i];
            if (c.v < res.vMin) { res.vMin = c.v; res.sAtMin = c.s; }
            const bool lo = (i > 0 && i + 1 < cand.size() &&
                             c.v < cand[i - 1].v && c.v < cand[i + 1].v);
            const bool hi = (i > 0 && i + 1 < cand.size() &&
                             c.v > cand[i - 1].v && c.v > cand[i + 1].v);
            if (lo) res.localMinima.push_back(c);
            if (hi) res.localMaxima.push_back(c);
        }

        std::sort(res.authorityProbes.begin(), res.authorityProbes.end());
        res.authorityProbes.erase(
            std::unique(res.authorityProbes.begin(), res.authorityProbes.end(),
                        [mergeTol](double a, double b) {
                            return std::abs(a - b) < mergeTol;
                        }),
            res.authorityProbes.end());
        return res;
    }

private:
    /**
     * Find every candidate parameter on one span (stationary points of
     * κ and T_i, all pairwise term crossings, plus span endpoints) and
     * push evaluated candidates in [sL,sR]. The span's own curve
     * evaluates the candidates, so junction points get both one-sided
     * values across neighboring spans.
     */
    void emitSpanCandidates(
        const envelope_detail::SpanPolys& sp,
        const Path& path, const NurbsCurve& piece,
        double sLo, double sL, double sR, double uLo, double uHi,
        double capPiece, double aCent,
        const Limits& limits, const Config& cfg,
        std::vector<Candidate>& cand,
        std::vector<double>& probes) const {
        namespace bn = tether::motion::bernstein;
        namespace ed = envelope_detail;
        const double width = sp.ub - sp.ua;
        std::vector<double> ts = {0.0, 1.0};
        const auto addRoots = [&](const std::vector<double>& poly) {
            for (double t : ed::findRoots(poly, cfg.rootTol)) {
                if (t > 1e-10 && t < 1.0 - 1e-10) ts.push_back(t);
            }
        };

        // κ stationarity: F'R − 3FR' = 0.
        {
            auto s = bn::multiply(bn::derivative(sp.F), sp.R);
            const auto fr3 = bn::multiply(sp.F, bn::derivative(sp.R));
            for (std::size_t i = 0; i < s.size(); ++i) s[i] -= 3.0 * fr3[i];
            addRoots(s);
        }
        // T_i stationarity: G_i'R − G_iR' = 0 per axis.
        {
            const auto rP = bn::derivative(sp.R);
            for (std::size_t i = 0; i < Dim; ++i) {
                if (!(static_cast<double>(limits.axis.maxVelocity[i]) > 0.0))
                    continue;
                auto g = bn::multiply(bn::derivative(sp.G[i]), sp.R);
                const auto gr = bn::multiply(sp.G[i], rP);
                for (std::size_t k = 0; k < g.size(); ++k) g[k] -= gr[k];
                addRoots(g);
            }
        }
        // Term crossings.
        const bool hasKappa = !ed::isZero(sp.F);
        const bool hasCap = std::isfinite(capPiece);
        const double ac2 = aCent * aCent;
        const double c2 = capPiece * capPiece;
        std::vector<bool> hasAxis(Dim, false);
        for (std::size_t i = 0; i < Dim; ++i) {
            hasAxis[i] = !ed::isZero(sp.G[i]) &&
                static_cast<double>(limits.axis.maxVelocity[i]) > 0.0;
        }
        if (hasKappa && aCent > 0.0 && hasCap) {
            // cap ↔ κ:  c⁴·F − a_c²·R³ = 0
            auto r3 = bn::multiply(bn::multiply(sp.R, sp.R), sp.R);
            for (double& v : r3) v *= -ac2;
            auto x = sp.F;
            for (double& v : x) v *= c2 * c2;
            if (r3.size() > x.size()) x.resize(r3.size(), 0.0);
            for (std::size_t i = 0; i < r3.size(); ++i) x[i] += r3[i];
            addRoots(x);
        }
        for (std::size_t i = 0; i < Dim; ++i) {
            if (!hasAxis[i]) continue;
            const double vi =
                static_cast<double>(limits.axis.maxVelocity[i]);
            const double vi2 = vi * vi, vi4 = vi2 * vi2;
            if (hasCap) {
                // cap ↔ axis_i:  c²·G_i − vmax_i²·R = 0
                auto x = sp.G[i];
                for (double& v : x) v *= c2;
                auto r = sp.R;
                for (double& v : r) v *= -vi2;
                for (std::size_t k = 0; k < x.size(); ++k) x[k] += r[k];
                addRoots(x);
            }
            for (std::size_t j = i + 1; j < Dim; ++j) {
                if (!hasAxis[j]) continue;
                // axis_i ↔ axis_j:  vmax_i²·G_j − vmax_j²·G_i = 0
                const double vj =
                    static_cast<double>(limits.axis.maxVelocity[j]);
                auto x = sp.G[j];
                for (double& v : x) v *= vi2;
                auto g = sp.G[i];
                for (double& v : g) v *= -vj * vj;
                for (std::size_t k = 0; k < x.size(); ++k) x[k] += g[k];
                addRoots(x);
            }
            if (hasKappa && aCent > 0.0) {
                // κ ↔ axis_i:  vmax_i⁴·F − a_c²·G_i²·R = 0
                auto x = sp.F;
                for (double& v : x) v *= vi4;
                auto g2r = bn::multiply(ed::square(sp.G[i]), sp.R);
                for (double& v : g2r) v *= -ac2;
                const std::size_t sz = std::max(x.size(), g2r.size());
                x.resize(sz, 0.0);
                g2r.resize(sz, 0.0);
                for (std::size_t k = 0; k < sz; ++k) x[k] += g2r[k];
                addRoots(x);
            }
        }

        // Authority probes: interior points where centripetal consumption
        // can spike without an envelope extremum. κ_i² stationarity
        // (M_i'·R − 2·M_i·R' = 0) covers per-axis curvature peaks on
        // polynomial spans; κ²·v_axis_j⁴ stationarity covers the product
        // rising while axis j binds: κ²·v⁴ = v_j⁴·F/(R·G_j²), so
        // F'·R·G_j − F·R'·G_j − 2·F·R·G_j' = 0.
        {
            const auto addProbeRoots = [&](const std::vector<double>& p) {
                for (double t : ed::findRoots(p, cfg.rootTol)) {
                    if (t <= 1e-10 || t >= 1.0 - 1e-10) continue;
                    const double u = sp.ua + t * width;
                    if (u < uLo - 1e-12 || u > uHi + 1e-12) continue;
                    const double s = sLo + piece.arcLengthTo(u);
                    if (s >= sL - 1e-9 && s <= sR + 1e-9)
                        probes.push_back(s);
                }
            };
            if (!sp.M.empty()) {
                const auto rP = bn::derivative(sp.R);
                for (std::size_t i = 0; i < Dim; ++i) {
                    if (!(static_cast<double>(
                              limits.axis.maxAcceleration[i]) > 0.0))
                        continue;
                    auto x = bn::multiply(bn::derivative(sp.M[i]), sp.R);
                    const auto mr = bn::multiply(sp.M[i], rP);
                    for (std::size_t k = 0; k < x.size(); ++k)
                        x[k] -= 2.0 * mr[k];
                    addProbeRoots(x);
                }
            }
            if (hasKappa) {
                const auto rP = bn::derivative(sp.R);
                const auto fP = bn::derivative(sp.F);
                for (std::size_t j = 0; j < Dim; ++j) {
                    if (!hasAxis[j]) continue;
                    const auto gP = bn::derivative(sp.G[j]);
                    // Three same-degree terms (8n−12 for cubics).
                    auto x = bn::multiply(bn::multiply(fP, sp.R), sp.G[j]);
                    const auto t2 =
                        bn::multiply(bn::multiply(sp.F, rP), sp.G[j]);
                    const auto t3 =
                        bn::multiply(bn::multiply(sp.F, sp.R), gP);
                    const std::size_t sz =
                        std::max({x.size(), t2.size(), t3.size()});
                    x.resize(sz, 0.0);
                    auto t2r = t2, t3r = t3;
                    t2r.resize(sz, 0.0);
                    t3r.resize(sz, 0.0);
                    for (std::size_t k = 0; k < sz; ++k)
                        x[k] -= t2r[k] + 2.0 * t3r[k];
                    addProbeRoots(x);
                }
            }
        }

        std::sort(ts.begin(), ts.end());
        ts.erase(std::unique(ts.begin(), ts.end(),
                             [](double a, double b) {
                                 return std::abs(a - b) < 1e-9;
                             }),
                 ts.end());
        for (double t : ts) {
            const double u = sp.ua + t * width;
            if (u < uLo - 1e-12 || u > uHi + 1e-12) continue;
            const double s = sLo + piece.arcLengthTo(u);
            if (s < sL - 1e-9 || s > sR + 1e-9) continue;
            cand.push_back(evalAt(path, *sp.curve, u, s,
                                  capPiece, aCent, limits));
        }
    }

    /**
     * Envelope value at (curve, u) mapped to arc length s:
     * min(cap, κ-term, axis-terms, segment feed at s). Evaluated on the
     * physical curve — no rescaling needed.
     */
    Candidate evalAt(const Path& path, const NurbsCurve& curve, double u,
                     double s, double cap, double aCent,
                     const Limits& limits) const {
        Candidate c;
        c.s = s;
        double v = cap;
        Term term = Term::Cap;
        try {
            const auto d = curve.arcDerivatives(u, 2);
            const double kappa = d.curvature.norm();
            if (aCent > 0.0 && kappa > 1e-12) {
                const double vc = std::sqrt(aCent / kappa);
                if (vc < v) { v = vc; term = Term::Curvature; }
            }
            for (std::size_t i = 0; i < Dim && i < d.tangent.dim(); ++i) {
                const double ti = std::abs(d.tangent.unchecked(i));
                const double vm =
                    static_cast<double>(limits.axis.maxVelocity[i]);
                if (ti > 1e-12 && vm > 0.0) {
                    const double va = vm / ti;
                    if (va < v) { v = va; term = Term::Axis; }
                }
            }
        } catch (...) {
            // Degenerate point (|C'| ~ 0, cusp): no regular tangent —
            // the only honest limit is a full stop.
            v = 0.0;
            term = Term::Curvature;
        }
        const double seg = static_cast<double>(
            path.maxVelocityAtArcLength(static_cast<T>(s)));
        if (seg < v) { v = seg; term = Term::Cap; }
        c.v = std::max(v, 0.0);
        c.vg = c.v;  // stamps fold later; keep the geometric value
        c.term = term;
        return c;
    }

    /**
     * Corner stamps are point dips at junction arc lengths. Cap any
     * candidate sitting at a stamp position, and insert a candidate at
     * every interior finite stamp (value = min(stamp, geometric
     * envelope) — the stamp is only a cap, never a floor).
     */
    void foldStamps(const Path& path,
                    const std::vector<Segment>& segs,
                    const std::vector<double>& corners,
                    double sA, double sB, double capGlobal, double aCent,
                    const Limits& limits,
                    std::vector<Candidate>& cand) const {
        if (corners.empty()) return;
        for (std::size_t i = 1; i + 1 < corners.size(); ++i) {
            const double vCorner = static_cast<double>(corners[i]);
            if (!std::isfinite(vCorner)) continue;
            const double s = (i < segs.size())
                ? static_cast<double>(segs[i].cumulativeArcLength)
                : sB;
            if (s < sA - 1e-9 || s > sB + 1e-9) continue;
            bool have = false;
            for (auto& c : cand) {
                if (std::abs(c.s - s) < 1e-9) {
                    if (vCorner < c.v) { c.v = vCorner; c.term = Term::Stamp; }
                    have = true;
                }
            }
            if (!have) {
                const auto loc = path.inner().locate(s);
                const auto& piece = path.inner().pieces()[loc.piece];
                const double u = piece.invertLength(
                    std::clamp(loc.localS, 0.0, piece.length()));
                Candidate c = evalAt(path, piece, u, s,
                                     capGlobal, aCent, limits);
                if (vCorner < c.v) { c.v = vCorner; c.term = Term::Stamp; }
                cand.push_back(c);
            }
        }
    }
};

} // namespace MotionPlanner::analytical
