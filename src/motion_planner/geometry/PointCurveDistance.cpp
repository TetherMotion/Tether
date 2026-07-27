/**
 * @file PointCurveDistance.cpp
 * @brief Implementation of certified point-to-NURBS distance.
 *
 * Equation numbers (G.x) refer to docs/motion/GeometryFoundations.md.
 */

#include "tether/motion_planner/geometry/PointCurveDistance.hpp"

#include "tether/motion_planner/geometry/Bernstein.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tether::motion {

namespace {

// Squared distance |C(t) − p|² on one Bézier span, evaluated exactly by
// De Casteljau. `aCoeffs[d]` holds the Bernstein coefficients of the
// homogeneous numerator A_d = w·C_d (NOT the rational control points C_d),
// so C_d(t) = A_d(t) / w(t) is computed by one division.
double squaredDistance(const std::vector<std::vector<double>>& aCoeffs,
                       const std::vector<double>& wCoeffs, const RVec& p,
                       double t) {
    const double w = bernstein::evaluate(wCoeffs, t);
    double d2 = 0.0;
    for (std::size_t d = 0; d < p.dim(); ++d) {
        const double s = bernstein::evaluate(aCoeffs[d], t) / w; // A_d/w = C_d
        const double diff = s - p.unchecked(d);
        d2 += diff * diff;
    }
    return d2;
}

// One Newton-polish of a root of the Bernstein polynomial N inside [lo, hi].
double polishRoot(const std::vector<double>& n,
                  const std::vector<double>& nPrime, double lo, double hi) {
    double t = 0.5 * (lo + hi);
    for (int i = 0; i < 25; ++i) {
        const double fv = bernstein::evaluate(n, t);
        const double dfv = bernstein::evaluate(nPrime, t);
        if (dfv == 0.0) break; // multiple root; midpoint is good enough
        double tn = t - fv / dfv;
        tn = std::max(lo, std::min(tn, hi));
        if (std::abs(tn - t) <= 1e-15) {
            t = tn;
            break;
        }
        t = tn;
    }
    return t;
}

} // namespace

DistanceResult pointCurveDistance(const NurbsCurve& c, const RVec& p) {
    if (p.dim() != c.dim()) {
        throw std::invalid_argument(
            "pointCurveDistance: dimension mismatch between point and curve");
    }
    const std::size_t dim = c.dim();

    double bestD2 = std::numeric_limits<double>::infinity();
    DistanceResult best;
    best.distance = std::numeric_limits<double>::infinity();
    best.u = c.knotMin();
    best.closestPoint = c.startPoint();

    for (const NurbsCurve& span : c.bezierDecompose()) {
        const int deg = span.degree();
        const double ua = span.knotMin();
        const double ub = span.knotMax();

        // Bernstein coefficients on t ∈ [0,1] of:
        //   w(t)   — the scalar weight polynomial (degree deg)
        //   A_d(t) — the homogeneous numerator polynomials A_d = w·C_d
        //             (degree deg; NOT the rational control points C_d)
        // The rational curve is C_d(t) = A_d(t) / w(t).
        std::vector<double> wCoeffs(deg + 1);
        for (int i = 0; i <= deg; ++i) wCoeffs[i] = span.weights()[i];

        std::vector<std::vector<double>> aCoeffs(dim);
        for (std::size_t d = 0; d < dim; ++d) {
            aCoeffs[d].resize(deg + 1);
            for (int i = 0; i <= deg; ++i) {
                aCoeffs[d][i] =
                    wCoeffs[i] * span.controlPoints()[i].unchecked(d);
            }
        }

        // N(t) = Σ_d (A_d − p_d·w)·(A_d′·w − A_d·w′), eq. (G.30).
        // This is the numerator of D′(t) = 2·N(t)/w(t)³; since w > 0 on the
        // span, the roots of D′ are exactly the roots of N. Each factor:
        //   A_d − p_d·w    — degree deg
        //   A_d′·w − A_d·w′ — degree 2·deg−1
        //   N              — degree 3·deg−1
        const std::vector<double> wPrime = bernstein::derivative(wCoeffs);
        std::vector<double> nPoly;
        for (std::size_t d = 0; d < dim; ++d) {
            // F_d = A_d − p_d·w  (degree deg)
            std::vector<double> fD(deg + 1);
            for (int i = 0; i <= deg; ++i) {
                fD[i] = aCoeffs[d][i] - p.unchecked(d) * wCoeffs[i];
            }
            // G_d = A_d′·w − A_d·w′  (degree 2·deg−1)
            const std::vector<double> aPrime = bernstein::derivative(aCoeffs[d]);
            std::vector<double> gD = bernstein::multiply(aPrime, wCoeffs);
            const std::vector<double> aTimesWPrime =
                bernstein::multiply(aCoeffs[d], wPrime);
            for (std::size_t i = 0; i < gD.size(); ++i) gD[i] -= aTimesWPrime[i];

            // N += F_d·G_d  (degree 3·deg−1)
            const std::vector<double> term = bernstein::multiply(fD, gD);
            if (nPoly.empty()) {
                nPoly = term;
            } else {
                for (std::size_t i = 0; i < nPoly.size(); ++i) nPoly[i] += term[i];
            }
        }

        // Candidates: span endpoints + all roots of N (certified isolation).
        std::vector<double> candidates = {0.0, 1.0};
        const std::vector<bernstein::Interval> roots =
            bernstein::isolateRoots(nPoly, 1e-9);
        const std::vector<double> nPrime = bernstein::derivative(nPoly);
        for (const bernstein::Interval& iv : roots) {
            candidates.push_back(polishRoot(nPoly, nPrime, iv.lo, iv.hi));
        }

        for (double t : candidates) {
            const double d2 = squaredDistance(aCoeffs, wCoeffs, p, t);
            if (d2 < bestD2) {
                const double u = ua + t * (ub - ua);
                bestD2 = d2;
                best.distance = std::sqrt(d2);
                best.u = u;
                best.closestPoint = span.evaluate(u);
            }
        }
    }

    return best;
}

} // namespace tether::motion
