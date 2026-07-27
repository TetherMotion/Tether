/**
 * @file NurbsCurve.cpp
 * @brief Implementation of tether::motion::NurbsCurve (see header for docs).
 *
 * Equation numbers (G.x) refer to docs/motion/GeometryFoundations.md.
 */

#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace tether::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 8-point Gauss–Legendre nodes/weights on [-1, 1] ( Abramowitz & Stegun ).
constexpr double kGLNodes[4] = {0.18343464249564980, 0.52553240991632899,
                                0.79666647741362674, 0.96028985649753623};
constexpr double kGLWeights[4] = {0.36268378337836198, 0.31370664587788729,
                                  0.22238103445337448, 0.10122853629037626};

} // namespace

// ============================================================================
// Construction / validation
// ============================================================================

NurbsCurve::NurbsCurve(std::vector<RVec> controlPoints,
                       std::vector<double> weights,
                       std::vector<double> knots, int degree)
    : controlPoints_(std::move(controlPoints)),
      weights_(std::move(weights)),
      knots_(std::move(knots)),
      degree_(degree),
      dim_(0) {
    if (controlPoints_.empty()) {
        throw std::invalid_argument("NurbsCurve: no control points");
    }
    if (degree < 1) {
        throw std::invalid_argument("NurbsCurve: degree must be >= 1");
    }
    if (controlPoints_.size() < static_cast<std::size_t>(degree) + 1) {
        throw std::invalid_argument(
            "NurbsCurve: need at least degree+1 control points");
    }
    dim_ = controlPoints_[0].dim();
    if (dim_ < 1 || dim_ > RVec::kMaxDim) {
        throw std::invalid_argument("NurbsCurve: control point dim must be 1..5");
    }
    for (const RVec& p : controlPoints_) {
        if (p.dim() != dim_) {
            throw std::invalid_argument(
                "NurbsCurve: control point dimension mismatch");
        }
    }
    if (weights_.size() != controlPoints_.size()) {
        throw std::invalid_argument(
            "NurbsCurve: weights size must equal number of control points");
    }
    for (double w : weights_) {
        if (!(w > 0.0) || !std::isfinite(w)) {
            throw std::invalid_argument(
                "NurbsCurve: weights must be positive and finite");
        }
    }
    if (knots_.size() != controlPoints_.size() + static_cast<std::size_t>(degree) + 1) {
        throw std::invalid_argument(
            "NurbsCurve: knot vector size must be n + p + 2");
    }
    for (std::size_t i = 1; i < knots_.size(); ++i) {
        if (knots_[i] < knots_[i - 1]) {
            throw std::invalid_argument(
                "NurbsCurve: knot vector must be non-decreasing");
        }
        if (!std::isfinite(knots_[i])) {
            throw std::invalid_argument("NurbsCurve: knots must be finite");
        }
    }
    if (knotMax() <= knotMin()) {
        throw std::invalid_argument("NurbsCurve: empty parameter domain");
    }
}

NurbsCurve NurbsCurve::fromLine(const RVec& a, const RVec& b) {
    if (a.dim() != b.dim()) {
        throw std::invalid_argument("NurbsCurve::fromLine: dim mismatch");
    }
    if (a == b) {
        throw std::invalid_argument("NurbsCurve::fromLine: zero-length line");
    }
    return NurbsCurve({a, b}, {1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, 1);
}

NurbsCurve NurbsCurve::fromArc(const RVec& center, double radius,
                               const RVec& axis1, const RVec& axis2,
                               double startAngle, double sweepAngle) {
    const std::size_t d = center.dim();
    if (axis1.dim() != d || axis2.dim() != d) {
        throw std::invalid_argument("NurbsCurve::fromArc: dim mismatch");
    }
    if (!(radius > 0.0) || !std::isfinite(radius)) {
        throw std::invalid_argument("NurbsCurve::fromArc: radius must be > 0");
    }
    if (sweepAngle == 0.0 || std::abs(sweepAngle) > 2.0 * kPi + 1e-12) {
        throw std::invalid_argument(
            "NurbsCurve::fromArc: sweep must be non-zero, |sweep| <= 2*pi");
    }
    const double tol = 1e-9;
    if (std::abs(axis1.norm() - 1.0) > tol || std::abs(axis2.norm() - 1.0) > tol ||
        std::abs(axis1.dot(axis2)) > tol) {
        throw std::invalid_argument(
            "NurbsCurve::fromArc: axis1/axis2 must be orthonormal");
    }

    // One span covers at most 120° (2π/3): the middle weight is cos(span/2),
    // which stays safely away from 0 (cos(60°) = 0.5). At 180° the weight
    // would be cos(90°) ≈ 0 — numerically degenerate. The plan specifies
    // ≤ 120° per span (§4.1.2, §M6).
    constexpr double kMaxSpanSweep = 2.0 * kPi / 3.0;
    const int nSpans =
        std::max(1, static_cast<int>(std::ceil(std::abs(sweepAngle) / kMaxSpanSweep)));
    const double spanSweep = sweepAngle / nSpans;
    const double half = spanSweep / 2.0;
    const double w = std::cos(half); // middle weight, proof P0
    const double invCosHalf = 1.0 / w;

    std::vector<RVec> cps;
    std::vector<double> wts;
    cps.reserve(2 * nSpans + 1);
    wts.reserve(2 * nSpans + 1);

    for (int s = 0; s < nSpans; ++s) {
        const double a0 = startAngle + s * spanSweep;
        const double a1 = a0 + half;
        const double a2 = a0 + spanSweep;

        const RVec onCircle0 =
            center + (axis1 * std::cos(a0) + axis2 * std::sin(a0)) * radius;
        const RVec shoulder =
            center + (axis1 * std::cos(a1) + axis2 * std::sin(a1)) *
                         (radius * invCosHalf);
        const RVec onCircle2 =
            center + (axis1 * std::cos(a2) + axis2 * std::sin(a2)) * radius;

        if (s == 0) {
            cps.push_back(onCircle0);
            wts.push_back(1.0);
        }
        cps.push_back(shoulder);
        wts.push_back(w);
        cps.push_back(onCircle2);
        wts.push_back(1.0);
    }

    // Clamped knot vector with full-multiplicity (p=2) internal knots: each
    // span is an independent rational-quadratic Bézier. The spans lie on one
    // circle, so they meet with G¹ continuity (same tangent direction and
    // curvature) even though the parameterization is C⁰ at the junctions.
    // Knot count: (p+1) + (nSpans-1)·p + (p+1) = 2·nSpans + 4 = n + p + 2. ✓
    std::vector<double> knots = {0.0, 0.0, 0.0};
    for (int i = 1; i < nSpans; ++i) {
        const double k = static_cast<double>(i) / nSpans;
        knots.push_back(k);
        knots.push_back(k); // full multiplicity p = 2
    }
    knots.insert(knots.end(), {1.0, 1.0, 1.0});

    return NurbsCurve(std::move(cps), std::move(wts), std::move(knots), 2);
}

// ============================================================================
// Span finding / domain helpers
// ============================================================================

int NurbsCurve::findSpan(double u) const {
    // Piegl & Tiller, The NURBS Book, Algorithm A2.1.
    const int n = static_cast<int>(controlPoints_.size()) - 1;
    if (u >= knots_[n + 1]) return n;
    if (u <= knots_[degree_]) return degree_;

    int low = degree_;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (u < knots_[mid] || u >= knots_[mid + 1]) {
        if (u < knots_[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

int NurbsCurve::knotMultiplicity(double u) const {
    int count = 0;
    for (double k : knots_) {
        if (k == u) ++count;
    }
    return count;
}

double NurbsCurve::clampToDomain(double u) const {
    return std::max(knotMin(), std::min(u, knotMax()));
}

// ============================================================================
// Evaluation
// ============================================================================

NurbsCurve::HomPoint NurbsCurve::evaluateHomogeneous(double u) const {
    // De Boor applied to the homogeneous control points (w·P, w),
    // GeometryFoundations.md eq. (G.12).
    const int p = degree_;
    const int span = findSpan(u);

    std::vector<HomPoint> d(p + 1, HomPoint{});
    for (int j = 0; j <= p; ++j) {
        const int idx = span - p + j;
        const double w = weights_[idx];
        for (std::size_t c = 0; c < dim_; ++c) {
            d[j].c[c] = controlPoints_[idx].unchecked(c) * w;
        }
        d[j].c[dim_] = w;
    }

    for (int r = 1; r <= p; ++r) {
        for (int j = p; j >= r; --j) {
            const int i = span - p + j;
            const double denom = knots_[i + p + 1 - r] - knots_[i];
            const double alpha = (denom > 0.0) ? (u - knots_[i]) / denom : 0.0;
            for (std::size_t c = 0; c <= dim_; ++c) {
                d[j].c[c] = (1.0 - alpha) * d[j - 1].c[c] + alpha * d[j].c[c];
            }
        }
    }
    return d[p];
}

RVec NurbsCurve::evaluate(double u) const {
    u = clampToDomain(u);
    const HomPoint h = evaluateHomogeneous(u);
    const double w = h.c[dim_];
    if (std::abs(w) < std::numeric_limits<double>::min()) {
        throw std::domain_error("NurbsCurve::evaluate: vanishing weight");
    }
    RVec result = RVec::zero(dim_);
    for (std::size_t c = 0; c < dim_; ++c) {
        result[c] = h.c[c] / w;
    }
    return result;
}

// ============================================================================
// Derivatives
// ============================================================================

void NurbsCurve::homogeneousDerivatives(double u, int k,
                                        std::vector<HomPoint>& out) const {
    // Basis function derivatives, Piegl & Tiller Algorithm A2.3, applied to
    // the homogeneous control points. Orders above the degree are zero.
    const int p = degree_;
    const int span = findSpan(u);
    const int nd = std::min(k, p);

    // --- A2.3: ders[kk][j] = d^kk N_{span-p+j,p} / du^kk at u -------------
    std::vector<std::vector<double>> ndu(p + 1, std::vector<double>(p + 1, 0.0));
    std::vector<double> left(p + 1), right(p + 1);

    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - knots_[span + 1 - j];
        right[j] = knots_[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp =
                (ndu[j][r] != 0.0) ? ndu[r][j - 1] / ndu[j][r] : 0.0;
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    std::vector<std::vector<double>> ders(nd + 1, std::vector<double>(p + 1, 0.0));
    for (int j = 0; j <= p; ++j) ders[0][j] = ndu[j][p];

    std::vector<std::vector<double>> a(2, std::vector<double>(p + 1, 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int kk = 1; kk <= nd; ++kk) {
            double d = 0.0;
            const int rk = r - kk;
            const int pk = p - kk;
            if (r >= kk) {
                const double denom = ndu[pk + 1][rk];
                a[s2][0] = (denom != 0.0) ? a[s1][0] / denom : 0.0;
                d = a[s2][0] * ndu[rk][pk];
            }
            const int j1 = (rk >= -1) ? 1 : -rk;
            const int j2 = (r - 1 <= pk) ? kk - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                const double denom = ndu[pk + 1][rk + j];
                a[s2][j] = (denom != 0.0)
                               ? (a[s1][j] - a[s1][j - 1]) / denom
                               : 0.0;
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                const double denom = ndu[pk + 1][r];
                a[s2][kk] = (denom != 0.0) ? -a[s1][kk - 1] / denom : 0.0;
                d += a[s2][kk] * ndu[r][pk];
            }
            ders[kk][r] = d;
            std::swap(s1, s2);
        }
    }

    double factor = static_cast<double>(p);
    for (int kk = 1; kk <= nd; ++kk) {
        for (int j = 0; j <= p; ++j) ders[kk][j] *= factor;
        factor *= static_cast<double>(p - kk);
    }

    // --- Combine with homogeneous control points ---------------------------
    out.assign(k + 1, HomPoint{});
    for (int kk = 0; kk <= nd; ++kk) {
        for (int j = 0; j <= p; ++j) {
            const int idx = span - p + j;
            const double w = weights_[idx];
            const double coeff = ders[kk][j];
            for (std::size_t c = 0; c < dim_; ++c) {
                out[kk].c[c] += coeff * w * controlPoints_[idx].unchecked(c);
            }
            out[kk].c[dim_] += coeff * w;
        }
    }
    // Orders above the degree stay zero (out initialized to zero).
}

RVec NurbsCurve::derivative(double u, int order) const {
    if (order < 0 || order > 3) {
        throw std::invalid_argument("NurbsCurve::derivative: order must be 0..3");
    }
    u = clampToDomain(u);
    if (order == 0) return evaluate(u);
    // NOTE: do NOT short-circuit `order > degree_` to zero. For *rational*
    // curves the parametric derivative above the degree is NONZERO: the
    // homogeneous derivatives A⁽ᵏ⁾, w⁽ᵏ⁾ are zero for k > p, but the quotient
    // rule (G.14)–(G.16) combines lower-order terms (e.g. C‴ = −(3w′C″ +
    // 3w″C′)/w for a degree-2 rational curve). For polynomial curves
    // (w ≡ 1) the quotient rule naturally yields zero, so removing the
    // check is safe for both cases.

    std::vector<HomPoint> hd;
    homogeneousDerivatives(u, order, hd);

    // Quotient rule on C = A/w with A = w·C, eq. (G.14)–(G.16).
    auto A = [&](int i) {
        RVec v = RVec::zero(dim_);
        for (std::size_t c = 0; c < dim_; ++c) v[c] = hd[i].c[c];
        return v;
    };
    auto w = [&](int i) { return hd[i].c[dim_]; };

    const double w0 = w(0);
    if (std::abs(w0) < std::numeric_limits<double>::min()) {
        throw std::domain_error("NurbsCurve::derivative: vanishing weight");
    }

    RVec C[4];
    C[0] = A(0) / w0;
    if (order >= 1) C[1] = (A(1) - C[0] * w(1)) / w0;
    if (order >= 2) C[2] = (A(2) - C[1] * (2.0 * w(1)) - C[0] * w(2)) / w0;
    if (order >= 3)
        C[3] = (A(3) - C[2] * (3.0 * w(1)) - C[1] * (3.0 * w(2)) -
                C[0] * w(3)) /
               w0;
    return C[order];
}

ArcDerivatives NurbsCurve::arcDerivatives(double u, int order) const {
    if (order < 0 || order > 3) {
        throw std::invalid_argument(
            "NurbsCurve::arcDerivatives: order must be 0..3");
    }
    u = clampToDomain(u);

    ArcDerivatives out;
    out.position = evaluate(u);
    if (order == 0) return out;

    // Parametric derivatives v = C', a = C'', j = C''' (zero above degree).
    const RVec v = derivative(u, 1);
    const double s1 = v.norm(); // ds/du = |C'(u)|, eq. (G.17)

    // Degenerate-parameterization guard (duplicate control points, cusps).
    double scale = 0.0;
    for (std::size_t i = 1; i < controlPoints_.size(); ++i) {
        scale += controlPoints_[i].distanceTo(controlPoints_[i - 1]);
    }
    scale = std::max(scale, std::numeric_limits<double>::min());
    if (!(s1 > 1e-14 * scale)) {
        throw std::domain_error(
            "NurbsCurve::arcDerivatives: |C'(u)| ~ 0 (degenerate "
            "parameterization)");
    }

    // Eq. (G.18): unit tangent.
    out.tangent = v / s1;
    if (order == 1) return out;

    const RVec a = derivative(u, 2);
    const double s2 = s1 * s1;
    const double s4 = s2 * s2;
    const double va = v.dot(a);

    // Eq. (G.19): curvature vector κ⃗ = d²p/ds².
    out.curvature = a / s2 - v * (va / s4);
    if (order == 2) return out;

    const RVec j = derivative(u, 3);
    const double s3 = s2 * s1;
    const double s5 = s4 * s1;
    const double s7 = s4 * s3;
    const double aa = a.normSq();
    const double vj = v.dot(j);

    // Eq. (G.20): jounce vector j⃗ = d³p/ds³.
    out.jounce = j / s3 - a * (3.0 * va / s5) - v * ((aa + vj) / s5) +
                 v * (4.0 * va * va / s7);
    return out;
}

// ============================================================================
// Arc length
// ============================================================================

double NurbsCurve::gaussLegendre8(double a, double b) const {
    const double mid = 0.5 * (a + b);
    const double half = 0.5 * (b - a);
    double sum = 0.0;
    for (int i = 0; i < 4; ++i) {
        const double dx = half * kGLNodes[i];
        sum += kGLWeights[i] * (speed(mid - dx) + speed(mid + dx));
    }
    return half * sum;
}

double NurbsCurve::quadratureRecursive(double a, double b, double tol,
                                       int depth) const {
    const double whole = gaussLegendre8(a, b);
    const double mid = 0.5 * (a + b);
    const double refined = gaussLegendre8(a, mid) + gaussLegendre8(mid, b);

    // Error estimate: difference between the 8-point rule on [a,b] and the
    // composite 2×8-point rule, eq. (G.25).
    if (std::abs(whole - refined) <= tol || depth >= 40) {
        return refined;
    }
    return quadratureRecursive(a, mid, 0.5 * tol, depth + 1) +
           quadratureRecursive(mid, b, 0.5 * tol, depth + 1);
}

double NurbsCurve::quadrature(double a, double b, double tol) const {
    ++arcLengthComputations_;
    return quadratureRecursive(a, b, tol, 0);
}

double NurbsCurve::length() const {
    if (cachedLength_ >= 0.0) return cachedLength_;
    double result;
    if (isPolyline()) {
        result = polylineArcLengthTo(knotMax()); // exact, no quadrature
    } else {
        // Tolerance relative to the control-polygon scale.
        double scale = 0.0;
        for (std::size_t i = 1; i < controlPoints_.size(); ++i) {
            scale += controlPoints_[i].distanceTo(controlPoints_[i - 1]);
        }
        const double tol = 1e-11 * std::max(scale, 1.0);
        result = quadrature(knotMin(), knotMax(), tol);
    }
    cachedLength_ = result;
    return result;
}

double NurbsCurve::arcLengthTo(double u) const {
    u = clampToDomain(u);
    if (u <= knotMin()) return 0.0;
    if (u >= knotMax()) return length();
    if (isPolyline()) return polylineArcLengthTo(u);

    double scale = 0.0;
    for (std::size_t i = 1; i < controlPoints_.size(); ++i) {
        scale += controlPoints_[i].distanceTo(controlPoints_[i - 1]);
    }
    const double tol = 1e-11 * std::max(scale, 1.0);
    return quadrature(knotMin(), u, tol);
}

double NurbsCurve::invertLength(double s) const {
    const double L = length();
    if (!(L > 0.0)) {
        throw std::domain_error("NurbsCurve::invertLength: zero-length curve");
    }
    s = std::max(0.0, std::min(s, L));
    if (s <= 0.0) return knotMin();
    if (s >= L) return knotMax();
    if (isPolyline()) return polylineInvertLength(s);

    // Newton–Raphson on f(u) = arcLengthTo(u) − s, f'(u) = |C'(u)| > 0,
    // bracketed by bisection (monotone, eq. (G.26)).
    const double tolS = 1e-12 * std::max(L, 1.0);
    const double tolU = 1e-14 * (knotMax() - knotMin()) +
                        4.0 * std::numeric_limits<double>::epsilon() *
                            std::max(std::abs(knotMin()), std::abs(knotMax()));

    double a = knotMin();
    double b = knotMax();
    double u = a + (b - a) * (s / L);

    for (int iter = 0; iter < 60; ++iter) {
        const double f = arcLengthTo(u) - s;
        if (std::abs(f) <= tolS) return u;
        if (f > 0.0) {
            b = u;
        } else {
            a = u;
        }
        if (b - a <= tolU) return 0.5 * (a + b);

        const double sp = speed(u);
        double un = u;
        if (sp > 0.0) un = u - f / sp; // Newton step
        if (!(un > a && un < b)) un = 0.5 * (a + b); // stay in bracket
        u = un;
    }
    return 0.5 * (a + b); // converged bracket (best effort)
}

double NurbsCurve::polylineArcLengthTo(double u) const {
    // Degree-1 B-spline == control polygon with knot-mapped parameter.
    const int n = static_cast<int>(controlPoints_.size()) - 1;
    double acc = 0.0;
    for (int i = 1; i <= n; ++i) {
        const double u0 = knots_[i];
        const double u1 = knots_[i + 1];
        if (u1 <= u0) continue; // zero-width span (repeated knot)
        const double segLen =
            controlPoints_[i].distanceTo(controlPoints_[i - 1]);
        if (u >= u1) {
            acc += segLen;
        } else if (u > u0) {
            acc += segLen * (u - u0) / (u1 - u0);
            break;
        } else {
            break;
        }
    }
    return acc;
}

double NurbsCurve::polylineInvertLength(double s) const {
    const int n = static_cast<int>(controlPoints_.size()) - 1;
    double acc = 0.0;
    for (int i = 1; i <= n; ++i) {
        const double u0 = knots_[i];
        const double u1 = knots_[i + 1];
        if (u1 <= u0) continue;
        const double segLen =
            controlPoints_[i].distanceTo(controlPoints_[i - 1]);
        if (segLen <= 0.0) continue; // duplicate control points
        if (acc + segLen >= s) {
            return u0 + (u1 - u0) * (s - acc) / segLen;
        }
        acc += segLen;
    }
    return knotMax();
}

std::size_t NurbsCurve::estimatedMemoryBytes() const noexcept {
    return sizeof(NurbsCurve) +
           controlPoints_.capacity() * sizeof(RVec) +
           weights_.capacity() * sizeof(double) +
           knots_.capacity() * sizeof(double);
}

// ============================================================================
// Knot insertion / split / trim / Bézier decomposition
// ============================================================================

NurbsCurve NurbsCurve::insertKnot(double u) const {
    // Boehm's algorithm in homogeneous coordinates, eq. (G.27).
    const int p = degree_;
    const int n = static_cast<int>(controlPoints_.size()) - 1;
    const int span = findSpan(u);
    const int s = knotMultiplicity(u);
    if (s >= p) return *this;

    std::vector<double> newKnots(knots_.size() + 1);
    for (int i = 0; i <= span; ++i) newKnots[i] = knots_[i];
    newKnots[span + 1] = u;
    for (std::size_t i = span + 1; i < knots_.size(); ++i) {
        newKnots[i + 1] = knots_[i];
    }

    std::vector<RVec> newPts(controlPoints_.size() + 1);
    std::vector<double> newWts(weights_.size() + 1);

    for (int i = 0; i <= span - p; ++i) {
        newPts[i] = controlPoints_[i];
        newWts[i] = weights_[i];
    }
    for (int i = span - s; i <= n; ++i) {
        newPts[i + 1] = controlPoints_[i];
        newWts[i + 1] = weights_[i];
    }
    for (int i = span - p + 1; i <= span - s; ++i) {
        const double denom = knots_[i + p] - knots_[i];
        const double alpha = (denom > 0.0) ? (u - knots_[i]) / denom : 0.0;
        // Interpolate in homogeneous coordinates, then project back.
        const double w0 = weights_[i - 1];
        const double w1 = weights_[i];
        const double newW = (1.0 - alpha) * w0 + alpha * w1;
        RVec newP = RVec::zero(dim_);
        for (std::size_t c = 0; c < dim_; ++c) {
            newP[c] = ((1.0 - alpha) * controlPoints_[i - 1].unchecked(c) * w0 +
                       alpha * controlPoints_[i].unchecked(c) * w1) /
                      newW;
        }
        newPts[i] = newP;
        newWts[i] = newW;
    }

    return NurbsCurve(std::move(newPts), std::move(newWts),
                      std::move(newKnots), p);
}

std::pair<NurbsCurve, NurbsCurve> NurbsCurve::split(double u) const {
    if (!(u > knotMin() && u < knotMax())) {
        throw std::invalid_argument(
            "NurbsCurve::split: u must lie inside the open domain");
    }
    const int p = degree_;

    // Insert u to multiplicity == p (full multiplicity): the curve then
    // interpolates the junction control point, eq. (G.28).
    NurbsCurve refined = *this;
    const int s = refined.knotMultiplicity(u);
    for (int i = s; i < p; ++i) {
        refined = refined.insertKnot(u);
    }

    // u-block: knots[k-p+1 .. k] == u (p copies); junction cp index j = k-p.
    int k = -1;
    for (int i = static_cast<int>(refined.knots_.size()) - 1; i >= 0; --i) {
        if (refined.knots_[i] == u) {
            k = i;
            break;
        }
    }
    if (k < 0) {
        throw std::runtime_error("NurbsCurve::split: knot insertion failed");
    }
    const int j = k - p;

    std::vector<RVec> leftPts(refined.controlPoints_.begin(),
                              refined.controlPoints_.begin() + j + 1);
    std::vector<double> leftWts(refined.weights_.begin(),
                                refined.weights_.begin() + j + 1);
    std::vector<double> leftKnots(refined.knots_.begin(),
                                  refined.knots_.begin() + k + 1);
    leftKnots.push_back(u); // clamp the left curve at its end

    std::vector<RVec> rightPts(refined.controlPoints_.begin() + j,
                               refined.controlPoints_.end());
    std::vector<double> rightWts(refined.weights_.begin() + j,
                                 refined.weights_.end());
    std::vector<double> rightKnots(p + 1, u); // clamp the right curve at start
    rightKnots.insert(rightKnots.end(), refined.knots_.begin() + k + 1,
                      refined.knots_.end());

    return {NurbsCurve(std::move(leftPts), std::move(leftWts),
                       std::move(leftKnots), p),
            NurbsCurve(std::move(rightPts), std::move(rightWts),
                       std::move(rightKnots), p)};
}

NurbsCurve NurbsCurve::trim(double s0, double s1) const {
    const double L = length();
    s0 = std::max(0.0, std::min(s0, L));
    s1 = std::max(0.0, std::min(s1, L));
    if (s1 < s0) std::swap(s0, s1);
    if (s1 - s0 <= 0.0) {
        throw std::invalid_argument(
            "NurbsCurve::trim: zero-length trim is not representable");
    }

    const double u0 = invertLength(s0);
    const double u1 = invertLength(s1);

    NurbsCurve result = *this;
    if (u1 < knotMax()) {
        result = result.split(u1).first;
    }
    if (u0 > result.knotMin() && u0 < result.knotMax()) {
        // u0 is a knot value of the original curve; after the first split it
        // remains a valid interior parameter of the left part.
        result = result.split(u0).second;
    }
    return result;
}

std::vector<NurbsCurve> NurbsCurve::bezierDecompose() const {
    const int p = degree_;

    // Raise every internal knot to multiplicity == p.
    NurbsCurve refined = *this;
    std::vector<double> uniqueInternal;
    for (std::size_t i = degree_ + 1; i < knots_.size() - degree_ - 1; ++i) {
        if (uniqueInternal.empty() || knots_[i] != uniqueInternal.back()) {
            uniqueInternal.push_back(knots_[i]);
        }
    }
    for (double uk : uniqueInternal) {
        const int mult = refined.knotMultiplicity(uk);
        for (int i = mult; i < p; ++i) {
            refined = refined.insertKnot(uk);
        }
    }

    // After refinement: m spans × p control points + 1; span b uses control
    // points [b·p, b·p + p] and knots {u_b × (p+1), u_{b+1} × (p+1)}.
    const std::size_t numSpans = (refined.numControlPoints() - 1) / p;

    // Distinct knot values in order.
    std::vector<double> breaks;
    for (double kv : refined.knots_) {
        if (breaks.empty() || kv != breaks.back()) breaks.push_back(kv);
    }

    std::vector<NurbsCurve> pieces;
    pieces.reserve(numSpans);
    for (std::size_t b = 0; b < numSpans; ++b) {
        std::vector<RVec> pts(refined.controlPoints_.begin() + b * p,
                              refined.controlPoints_.begin() + b * p + p + 1);
        std::vector<double> wts(refined.weights_.begin() + b * p,
                                refined.weights_.begin() + b * p + p + 1);
        std::vector<double> kn(p + 1, breaks[b]);
        kn.insert(kn.end(), p + 1, breaks[b + 1]);
        pieces.emplace_back(std::move(pts), std::move(wts), std::move(kn), p);
    }
    return pieces;
}

} // namespace tether::motion
