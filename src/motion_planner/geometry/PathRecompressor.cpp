/**
 * @file PathRecompressor.cpp
 * @brief Error-bounded geometric recompression of a PiecewiseNurbsPath
 *        (see PathRecompressor.hpp for the pipeline description).
 */

#include "tether/motion_planner/geometry/PathRecompressor.hpp"
#include "tether/motion_planner/geometry/PointCurveDistance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tether::motion {

namespace {

constexpr double kTol = 1e-14;
constexpr double kDegToRad = M_PI / 180.0;

// ============================================================================
// B-spline basis (Piegl & Tiller A2.1/A2.2) — mirrored from ProfileSplineFitter
// ============================================================================

int findSpan(const std::vector<double>& knots, int degree, double u, int ncp) {
    const int n = ncp - 1;
    if (u >= knots[n + 1]) return n;
    if (u <= knots[degree]) return degree;
    int lo = degree, hi = n + 1, mid = (lo + hi) / 2;
    while (u < knots[mid] || u >= knots[mid + 1]) {
        if (u < knots[mid]) hi = mid; else lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

void basisFunctions(int span, double u, int degree,
                    const std::vector<double>& knots,
                    double* N /* size degree+1 */) {
    double left[8], right[8];
    N[0] = 1.0;
    for (int j = 1; j <= degree; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
}

// ============================================================================
// Polyline distance helpers
// ============================================================================

double pointSegDist(const RVec& p, const RVec& a, const RVec& b) {
    const RVec ab = b - a;
    const double l2 = ab.normSq();
    if (l2 < kTol) return p.distanceTo(a);
    const double t = std::clamp((p - a).dot(ab) / l2, 0.0, 1.0);
    return p.distanceTo(a + ab * t);
}

double polylineDist(const RVec& p, const std::vector<RVec>& vts) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < vts.size(); ++i)
        best = std::min(best, pointSegDist(p, vts[i], vts[i + 1]));
    return best;
}

/** Max distance of interior vertices to the chord [vts.front, vts.back]. */
double maxChordDeviation(const std::vector<RVec>& vts) {
    double worst = 0.0;
    for (std::size_t i = 1; i + 1 < vts.size(); ++i)
        worst = std::max(worst,
                         pointSegDist(vts[i], vts.front(), vts.back()));
    return worst;
}

// ============================================================================
// Least-squares clamped B-spline fit (endpoints pinned)
// ============================================================================

struct LSQFit {
    bool ok = false;
    std::vector<RVec> cps;
    std::vector<double> knots;
};

/**
 * Build a clamped knot vector distributed over the sample parameters by
 * data density (Piegl & Tiller eq. 9.68–9.69). Interior knots are
 * deduplicated to strictly increasing values — when ncp is large
 * relative to the sample count the raw formula can emit a knot equal
 * to an existing one (multiplicity > degree+1 → 0/0 basis values).
 * Returns the knot vector; `ncpUsed` gives the adjusted control-point
 * count matching it.
 */
std::vector<double> lsqKnots(const std::vector<double>& params,
                             int degree, int ncp, int& ncpUsed) {
    const int m = static_cast<int>(params.size()) - 1; // samples 0..m
    const int n = ncp - 1;                             // cps 0..n
    const double d = static_cast<double>(m + 1) / (n - degree + 1);
    std::vector<double> interior;
    interior.reserve(n - degree);
    const double eps = 1e-12;
    for (int j = 1; j <= n - degree; ++j) {
        const double jd = j * d;
        int i = static_cast<int>(jd);
        const double alpha = jd - i;
        i = std::clamp(i, 1, m);
        const double k = (1.0 - alpha) * params[i - 1] + alpha * params[i];
        if (k <= eps || k >= 1.0 - eps) continue;
        if (!interior.empty() && k <= interior.back() + eps) continue;
        interior.push_back(k);
    }
    ncpUsed = degree + 1 + static_cast<int>(interior.size());
    std::vector<double> knots(ncpUsed + degree + 1, 0.0);
    for (int i = 0; i <= degree; ++i)
        knots[knots.size() - 1 - i] = 1.0;
    std::copy(interior.begin(), interior.end(), knots.begin() + degree + 1);
    return knots;
}

/**
 * Solve min over interior CPs of Σ_s |r_s − Σ_j N_j(t_s)·P_j|² where
 * r_s = Q_s − N_0(t_s)·Q_0 − N_n(t_s)·Q_m (endpoint contributions are
 * pinned and folded into the RHS). Dense Gaussian elimination with
 * partial pivoting — ncp ≤ maxControlPoints (small).
 */
LSQFit fitLeastSquares(const std::vector<RVec>& pts,
                       const std::vector<double>& params,
                       int degree, int ncp) {
    LSQFit fit;
    fit.knots = lsqKnots(params, degree, ncp, ncp);
    const int nint = ncp - 2;
    const int dim = static_cast<int>(pts.front().dim());

    // Augmented normal-equation matrix: [nint x nint | dim RHS columns]
    std::vector<std::vector<double>> A(
        nint, std::vector<double>(nint + dim, 0.0));

    double N[8];
    for (std::size_t s = 0; s < pts.size(); ++s) {
        const int span = findSpan(fit.knots, degree, params[s], ncp);
        basisFunctions(span, params[s], degree, fit.knots, N);
        // Fold pinned endpoint contributions into the residual.
        RVec r = pts[s];
        for (int j = 0; j <= degree; ++j) {
            const int col = span - degree + j;
            if (col == 0) r -= N[j] * pts.front();
            else if (col == ncp - 1) r -= N[j] * pts.back();
        }
        for (int j = 0; j <= degree; ++j) {
            const int colA = span - degree + j;
            if (colA < 1 || colA > nint) continue;
            for (int k = j; k <= degree; ++k) {
                const int colB = span - degree + k;
                if (colB < 1 || colB > nint) continue;
                A[colA - 1][colB - 1] += N[j] * N[k];
            }
            for (int dd = 0; dd < dim; ++dd)
                A[colA - 1][nint + dd] += N[j] * r[dd];
        }
    }
    // Mirror the accumulated upper triangle — SPD normal equations.
    for (int a = 0; a < nint; ++a)
        for (int b = a + 1; b < nint; ++b)
            A[b][a] = A[a][b];

    // Gaussian elimination with partial pivoting on the augmented matrix.
    for (int k = 0; k < nint; ++k) {
        int piv = k;
        for (int i = k + 1; i < nint; ++i)
            if (std::abs(A[i][k]) > std::abs(A[piv][k])) piv = i;
        if (piv != k) std::swap(A[k], A[piv]);
        if (std::abs(A[k][k]) < 1e-12) return fit; // singular → caller retries/splits
        for (int i = k + 1; i < nint; ++i) {
            const double f = A[i][k] / A[k][k];
            for (int j = k; j < nint + dim; ++j) A[i][j] -= f * A[k][j];
        }
    }

    std::vector<std::vector<double>> sol(
        dim, std::vector<double>(nint, 0.0));
    for (int dd = 0; dd < dim; ++dd) {
        for (int i = nint - 1; i >= 0; --i) {
            double sum = A[i][nint + dd];
            for (int j = i + 1; j < nint; ++j)
                sum -= A[i][j] * sol[dd][j];
            sol[dd][i] = sum / A[i][i];
        }
    }
    fit.cps.resize(ncp);
    fit.cps.front() = pts.front();
    fit.cps.back() = pts.back();
    for (int i = 1; i <= nint; ++i) {
        RVec p = pts.front();
        for (int dd = 0; dd < dim; ++dd) p[dd] = sol[dd][i - 1];
        fit.cps[i] = p;
    }
    fit.ok = true;
    return fit;
}

// ============================================================================
// Certification: fitted curve ↔ source polyline, both Hausdorff directions
// ============================================================================

/**
 * Derivative control-point magnitudes |Q_i| of the fitted (non-rational)
 * B-spline: Q_i = p·(P_{i+1} − P_i)/(U_{i+p+1} − U_{i+1}). The convex-hull
 * property gives |C'(u)| ≤ max_i |Q_i| over the active index range.
 */
std::vector<double> derivMagnitudes(const std::vector<RVec>& cps,
                                    const std::vector<double>& knots,
                                    int degree) {
    const std::size_t n = cps.size() - 1;
    std::vector<double> q(n, 0.0);
    for (std::size_t i = 0; i + 1 < cps.size(); ++i) {
        const double den = knots[i + degree + 1] - knots[i + 1];
        if (den <= kTol) continue;
        q[i] = degree * (cps[i + 1] - cps[i]).norm() / den;
    }
    return q;
}

struct Cert {
    bool ok = false;
    double deviation = std::numeric_limits<double>::infinity();
    /** Source-vertex index near the worst violation (split hint). */
    std::size_t splitHint = 0;
};

/**
 * Both-direction bound on the Hausdorff distance between the fitted
 * curve and the source polyline:
 *  - dir1: every interior vertex → certified pointCurveDistance to the
 *    curve (Bernstein root isolation — no missed roots).
 *  - dir2: uniform samples on the curve → distance to the polyline,
 *    with the unsampled gaps closed by the convex-hull speed bound:
 *    dist(C(u)) ≤ max_j dist(C(u_j)) + |C'|_bound·h/2.
 */
Cert certify(const NurbsCurve& curve, const std::vector<RVec>& vts,
             const std::vector<double>& params, double tol) {
    Cert res;
    double worst = 0.0;

    // Direction 1: source vertices → curve. |C(t_i) − v_i| is itself a
    // valid upper bound on the vertex's distance to the curve — when it
    // is ≤ tol the vertex is already certified, so the root-isolated
    // pointCurveDistance is only needed as a last-chance rescue when
    // the residual exceeds tol (the fitted curve may still pass near
    // the vertex at a different parameter).
    for (std::size_t i = 1; i + 1 < vts.size(); ++i) {
        const double res_i =
            curve.evaluate(params[i]).distanceTo(vts[i]);
        double d = res_i;
        if (res_i > tol)
            d = pointCurveDistance(curve, vts[i]).distance;
        if (d > worst) { worst = d; res.splitHint = i; }
        if (worst > tol) { res.deviation = worst; return res; }
    }

    // Direction 2: curve → polyline.
    //
    // polylineDist is O(nVerts) per sample — too slow as an inner loop.
    // Since params[] is monotone, the sample at curve parameter u has
    // its nearest polyline segment near the segment index implied by u;
    // a window of ±kWin segments around that index is scanned instead.
    // A windowed minimum is ≥ the true distance (fewer candidates), so
    // every bound below stays conservative — over-estimates only ever
    // make us reject, never accept. When the windowed value exceeds tol
    // the exact full scan is used (accurate rejection + split hint).
    //
    // Interval refinement: rather than one uniform dense grid, each
    // interval [ua,ub] is bounded by
    //   max(da,db) + 0.5·(ub−ua)·max|C'|_[ua,ub]
    // with the *local* derivative-control-point bound for that span.
    // Only intervals whose bound can hide a violation are bisected —
    // good fits stay coarse everywhere except their tight spots.
    const double uMin = curve.knotMin(), uMax = curve.knotMax();
    const int p = curve.degree();
    const auto& cpRef = curve.controlPoints();
    const auto& knRef = curve.knots();
    const int ncpI = static_cast<int>(curve.numControlPoints());
    const std::vector<double> qmag = derivMagnitudes(cpRef, knRef, p);
    auto localSpeed = [&](double ua, double ub) {
        const int sa = findSpan(knRef, p, ua, ncpI);
        const int sb = findSpan(knRef, p, ub, ncpI);
        const int lo = std::max(0, sa - p);
        const int hi = std::min(static_cast<int>(qmag.size()) - 1,
                                std::max(sb - 1, lo));
        double s = 0.0;
        for (int i = lo; i <= hi; ++i) s = std::max(s, qmag[i]);
        return s;
    };
    const std::size_t nSeg = vts.size() - 1;
    constexpr std::size_t kWin = 8;
    auto windowedDist = [&](const RVec& pt, double u) {
        const auto it = std::upper_bound(params.begin(), params.end(), u);
        const std::size_t segIdx =
            std::min<std::size_t>(
                it == params.begin() ? 0
                                     : std::size_t(it - params.begin()) - 1,
                nSeg - 1);
        const std::size_t lo = segIdx > kWin ? segIdx - kWin : 0;
        const std::size_t hi = std::min(segIdx + kWin, nSeg - 1);
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = lo; i <= hi; ++i)
            best = std::min(best, pointSegDist(pt, vts[i], vts[i + 1]));
        if (best > tol) best = polylineDist(pt, vts);
        return best;
    };

    struct Iv { double ua, ub, da, db; };
    std::vector<Iv> work;
    const std::size_t K0 = std::max<std::size_t>(
        32, 2 * (curve.numControlPoints() - curve.degree()));
    const double h0 = (uMax - uMin) / K0;
    std::vector<double> d0(K0 + 1, 0.0);
    double bound = 0.0, worstU = uMin;
    {
        bool earlyFail = false;
        for (std::size_t k = 0; k <= K0; ++k) {
            const double u = uMin + h0 * k;
            d0[k] = windowedDist(curve.evaluate(u), u);
            if (d0[k] > tol) {          // confirmed violation — fail fast
                worst = std::max(worst, d0[k]);
                worstU = u;
                earlyFail = true;
                break;
            }
        }
        if (!earlyFail) {
            for (std::size_t k = 0; k < K0; ++k)
                work.push_back({uMin + h0 * k, uMin + h0 * (k + 1),
                                d0[k], d0[k + 1]});
            constexpr std::size_t kMaxEvals = 8192;
            std::size_t evals = K0 + 1;
            while (!work.empty()) {
                const Iv iv = work.back(); work.pop_back();
                const double locB = std::max(iv.da, iv.db)
                    + 0.5 * (iv.ub - iv.ua) * localSpeed(iv.ua, iv.ub);
                if (locB <= tol || evals >= kMaxEvals) {
                    if (locB > bound) {
                        bound = locB;
                        worstU = 0.5 * (iv.ua + iv.ub);
                    }
                    continue;
                }
                const double um = 0.5 * (iv.ua + iv.ub);
                const double dm = windowedDist(curve.evaluate(um), um);
                ++evals;
                if (dm > tol) { bound = dm; worstU = um; break; }
                work.push_back({iv.ua, um, iv.da, dm});
                work.push_back({um, iv.ub, dm, iv.db});
            }
        }
    }
    worst = std::max(worst, bound);
    {
        auto it = std::lower_bound(params.begin(), params.end(), worstU);
        res.splitHint = std::min<std::size_t>(
            it == params.end() ? vts.size() - 1 : it - params.begin(),
            vts.size() - 1);
    }
    res.deviation = worst;
    res.ok = worst <= tol;
    return res;
}

} // anonymous namespace

// ============================================================================
// Run processing
// ============================================================================

namespace {

struct RunOutput {
    std::vector<NurbsCurve> pieces;
    std::vector<RecompressPieceMap> maps;
    double maxDeviation = 0.0;
    std::size_t fitted = 0;
    std::size_t verbatim = 0;
};

/** Emit the source pieces [b, e) unchanged (certified deviation 0). */
void emitVerbatim(const PiecewiseNurbsPath& src, std::size_t b,
                  std::size_t e, RunOutput& out) {
    for (std::size_t i = b; i < e; ++i) {
        out.pieces.push_back(src.piece(i));
        out.maps.push_back({i, i + 1});
        ++out.verbatim;
    }
}

/** Polyline vertices for the run [b, e): piece b's start + every end. */
std::vector<RVec> runVertices(const PiecewiseNurbsPath& src,
                              std::size_t b, std::size_t e) {
    std::vector<RVec> vts;
    vts.reserve(e - b + 1);
    vts.push_back(src.piece(b).evaluate(src.piece(b).knotMin()));
    for (std::size_t i = b; i < e; ++i)
        vts.push_back(src.piece(i).evaluate(src.piece(i).knotMax()));
    return vts;
}

/** Normalized chord-length parameters for the vertex chain. */
std::vector<double> chordParams(const std::vector<RVec>& vts) {
    std::vector<double> t(vts.size(), 0.0);
    for (std::size_t i = 1; i < vts.size(); ++i)
        t[i] = t[i - 1] + vts[i].distanceTo(vts[i - 1]);
    const double total = t.back();
    if (total > kTol)
        for (auto& v : t) v /= total;
    else
        for (std::size_t i = 0; i < t.size(); ++i)
            t[i] = static_cast<double>(i) / (t.size() - 1);
    return t;
}

void fitRun(const PiecewiseNurbsPath& src, std::size_t b, std::size_t e,
            const RecompressConfig& cfg, int depth, RunOutput& out,
            std::size_t& splits) {
    const std::size_t nPieces = e - b;
    const auto vts = runVertices(src, b, e);

    // Near-collinear run → a single line is already within tolerance.
    // (Closed runs have front == back — skip the shortcut there.)
    if (vts.front().distanceTo(vts.back()) > 1e-9 &&
        maxChordDeviation(vts) <= cfg.tolerance) {
        out.pieces.push_back(NurbsCurve::fromLine(vts.front(), vts.back()));
        out.maps.push_back({b, e});
        ++out.fitted;
        return;
    }

    // Too few points to constrain a degree-p fit → keep originals.
    if (nPieces < cfg.minRunPieces ||
        vts.size() < static_cast<std::size_t>(cfg.degree) + 2) {
        emitVerbatim(src, b, e, out);
        return;
    }

    const auto params = chordParams(vts);

    // Adaptive schedule: grow control points until certified or capped.
    std::size_t ncp = std::min<std::size_t>(
        cfg.maxControlPoints, std::max<std::size_t>(cfg.degree + 5, 8));
    Cert cert;
    while (ncp <= cfg.maxControlPoints &&
           ncp <= vts.size() + static_cast<std::size_t>(cfg.degree)) {
        const int ncpInt = static_cast<int>(ncp);
        if (ncpInt < cfg.degree + 1) break;
        const LSQFit bestFit = fitLeastSquares(vts, params, cfg.degree,
                                             ncpInt);
        if (!bestFit.ok) break;
        try {
            NurbsCurve candidate(bestFit.cps,
                                 std::vector<double>(bestFit.cps.size(), 1.0),
                                 bestFit.knots, cfg.degree);
            cert = certify(candidate, vts, params, cfg.tolerance);
            if (cert.ok) {
                out.pieces.push_back(std::move(candidate));
                out.maps.push_back({b, e});
                ++out.fitted;
                out.maxDeviation =
                    std::max(out.maxDeviation, cert.deviation);
                return;
            }
        } catch (...) {
            break; // degenerate candidate — split instead
        }
        if (ncp == cfg.maxControlPoints) break;
        ncp = std::min(cfg.maxControlPoints, ncp * 2);
    }

    // Split near the worst-fitting vertex and recurse — bounded by
    // depth. The hint may sit on a run boundary (the fitted curve
    // typically blows up at the far end when the shape is unfit); clamp
    // to a real interior boundary so progress is guaranteed.
    if (depth < cfg.maxSplitDepth && nPieces >= 2) {
        const std::size_t hint =
            std::clamp(cert.splitHint, std::size_t{1}, nPieces - 1);
        ++splits;
        const std::size_t mid = b + hint;
        fitRun(src, b, mid, cfg, depth + 1, out, splits);
        fitRun(src, mid, e, cfg, depth + 1, out, splits);
        return;
    }

    // Fall back to verbatim pieces — always within tolerance.
    emitVerbatim(src, b, e, out);
}

} // anonymous namespace

// ============================================================================
// Public entry point
// ============================================================================

RecompressResult recompressPiecewiseNurbs(
    const PiecewiseNurbsPath& src,
    const RecompressConfig& cfg,
    const std::vector<std::size_t>& hardBreakPieces) {
    const std::size_t n = src.numPieces();
    if (n == 0)
        throw std::invalid_argument("recompressPiecewiseNurbs: empty path");
    if (cfg.tolerance <= 0.0)
        throw std::invalid_argument("recompressPiecewiseNurbs: tolerance ≤ 0");
    if (cfg.degree < 1)
        throw std::invalid_argument("recompressPiecewiseNurbs: degree < 1");
    if (cfg.degree > 6)
        throw std::invalid_argument("recompressPiecewiseNurbs: degree > 6");

    // Hard-break lookup: piece index → must start a new run.
    std::vector<char> hardBreak(n, 0);
    for (const std::size_t k : hardBreakPieces) {
        if (k < n) hardBreak[k] = 1;
    }

    const double cosCorner =
        std::cos(cfg.cornerAngleDeg * kDegToRad);

    // Piece direction for degree-1 pieces (chord direction).
    auto pieceDir = [&](std::size_t i) {
        const NurbsCurve& c = src.piece(i);
        const RVec d = c.evaluate(c.knotMax()) - c.evaluate(c.knotMin());
        const double len = d.norm();
        return len > kTol ? d / len : RVec::zero(c.dim());
    };

    RunOutput out;
    out.pieces.reserve(n);
    out.maps.reserve(n);
    std::size_t splits = 0;

    std::size_t runBegin = 0;
    bool runOpen = false;      // currently accumulating degree-1 pieces
    RVec prevDir;
    const double breakTol = cfg.tolerance * 0.1;

    for (std::size_t i = 0; i < n; ++i) {
        const NurbsCurve& piece = src.piece(i);
        const bool isLine = (piece.degree() == 1);

        // Non-polyline pieces are emitted verbatim and break runs.
        if (!isLine) {
            if (runOpen) {
                fitRun(src, runBegin, i, cfg, 0, out, splits);
                runOpen = false;
            }
            out.pieces.push_back(piece);
            out.maps.push_back({i, i + 1});
            ++out.verbatim;
            continue;
        }

        // Discontinuity check: never fit across a G0 gap.
        bool boundary = hardBreak[i];
        if (i > 0 && src.piece(i - 1).degree() == 1) {
            const RVec prevEnd =
                src.piece(i - 1).evaluate(src.piece(i - 1).knotMax());
            const RVec curStart = piece.evaluate(piece.knotMin());
            if (prevEnd.distanceTo(curStart) >
                std::max(breakTol, 1e-9)) {
                boundary = true;
            }
        }
        // Corner check against the previous run piece.
        const RVec dir = pieceDir(i);
        if (runOpen && !boundary) {
            const double dlen = dir.norm();
            if (dlen > kTol && prevDir.norm() > kTol &&
                dir.normalized().dot(prevDir) < cosCorner) {
                boundary = true;
            }
        }

        if (runOpen && boundary) {
            fitRun(src, runBegin, i, cfg, 0, out, splits);
            runOpen = false;
        }
        if (!runOpen) {
            runBegin = i;
            runOpen = true;
        }
        prevDir = dir;
    }
    if (runOpen) fitRun(src, runBegin, n, cfg, 0, out, splits);

    RecompressResult result(PiecewiseNurbsPath(std::move(out.pieces)));
    result.sourceMap = std::move(out.maps);
    result.maxCertifiedDeviation = out.maxDeviation;
    result.runsFitted = out.fitted;
    result.runsSplit = splits;
    result.piecesKeptVerbatim = out.verbatim;
    return result;
}

} // namespace tether::motion
