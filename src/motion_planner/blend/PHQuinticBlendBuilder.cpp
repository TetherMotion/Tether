/**
 * @file PHQuinticBlendBuilder.cpp
 * @brief Opt-in PH quintic blend fast path (D6, M16–M19).
 *
 * Equation numbers (M.x) refer to docs/motion/BlendingAlgorithm.md.
 * References: [R1] Wang et al. 2010, [R2] Farouki & Shah 1996,
 *             [R3] Moon, Farouki & Choi 2001.
 */

#include "tether/motion_planner/blend/PHQuinticBlendBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace tether::motion {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Complex square root: √d = √|d| · e^{i·arg(d)/2}. Returns both ± roots.
std::pair<std::complex<double>, std::complex<double>>
csqrt(std::complex<double> d) {
    const double r = std::abs(d);
    if (r == 0.0) {
        return {std::complex<double>(0.0, 0.0),
                std::complex<double>(0.0, 0.0)};
    }
    const double rootR = std::sqrt(r);
    const double halfArg = 0.5 * std::arg(d);
    const std::complex<double> root =
        std::polar(rootR, halfArg);
    return {root, -root};
}

/// Convert an ℝᴺ point to a corner-plane complex coordinate.
std::complex<double> toComplex(const RVec& p, const RVec& origin,
                               const RVec& e1, const RVec& e2) {
    const RVec d = p - origin;
    return std::complex<double>(d.dot(e1), d.dot(e2));
}

/// Convert a corner-plane complex coordinate back to ℝᴺ.
RVec fromComplex(std::complex<double> z, const RVec& origin,
                 const RVec& e1, const RVec& e2) {
    return origin + e1 * z.real() + e2 * z.imag();
}

/// Hodograph Bernstein coefficients h_k (M16): h₀ = ω₀², h₁ = ω₀ω₁,
/// h₂ = (2ω₁² + ω₀ω₂)/3, h₃ = ω₁ω₂, h₄ = ω₂².
std::array<std::complex<double>, 5>
hodographCoeffs(const std::complex<double>& w0,
                const std::complex<double>& w1,
                const std::complex<double>& w2) {
    return {w0 * w0,
            w0 * w1,
            (2.0 * w1 * w1 + w0 * w2) / 3.0,
            w1 * w2,
            w2 * w2};
}

/// Parametric speed σ(ξ) = |ω(ξ)|² = u²+v² as Bernstein coefficients
/// of degree 4 (M16).
std::array<double, 5> speedCoeffs(const std::complex<double>& w0,
                                  const std::complex<double>& w1,
                                  const std::complex<double>& w2) {
    const double u0 = w0.real(), v0 = w0.imag();
    const double u1 = w1.real(), v1 = w1.imag();
    const double u2 = w2.real(), v2 = w2.imag();
    // σ_k from (M16).
    return {u0 * u0 + v0 * v0,
            u0 * u1 + v0 * v1,
            (2.0 * (u1 * u1 + v1 * v1) + u0 * u2 + v0 * v2) / 3.0,
            u1 * u2 + v1 * v2,
            u2 * u2 + v2 * v2};
}

/// Evaluate a degree-4 Bernstein polynomial at ξ.
double evalBernstein4(const std::array<double, 5>& c, double xi) {
    // de Casteljau
    double b[5];
    for (int i = 0; i < 5; ++i) b[i] = c[i];
    for (int r = 1; r <= 4; ++r)
        for (int i = 0; i <= 4 - r; ++i)
            b[i] = (1.0 - xi) * b[i] + xi * b[i + 1];
    return b[0];
}

/// Check if σ(ξ) touches zero anywhere on [0,1] (degenerate candidate).
bool isDegenerate(const std::array<double, 5>& sigma) {
    // σ ≥ 0 always (it's |ω|²). Degenerate iff min Bernstein coeff ≈ 0
    // AND the curve of σ dips to zero. Conservative: check if all coeffs
    // are near zero (σ ≡ 0) or if the minimum is below a tiny threshold.
    double minC = sigma[0];
    for (int i = 1; i < 5; ++i) minC = std::min(minC, sigma[i]);
    // If the minimum Bernstein coefficient is ≤ 0 (allowing for fp noise),
    // σ may touch zero. Use a small positive threshold.
    return minC <= 1e-15;
}

} // namespace

std::vector<PHQuinticBlendBuilder::Result>
PHQuinticBlendBuilder::buildCandidates(const BoundaryConditions& entry,
                                       const BoundaryConditions& exit,
                                       const RVec& planeE1,
                                       const RVec& planeE2) {
    if (entry.position.dim() != exit.position.dim()) {
        throw std::invalid_argument(
            "PHQuinticBlendBuilder: entry/exit dimension mismatch");
    }
    if (planeE1.dim() != planeE2.dim() ||
        planeE1.dim() != entry.position.dim()) {
        throw std::invalid_argument(
            "PHQuinticBlendBuilder: plane basis dimension mismatch");
    }

    // Corner-plane coordinates. The origin is the entry trim point.
    const RVec origin = entry.position;
    const std::complex<double> r0(0.0, 0.0); // entry at origin
    const std::complex<double> r1 = toComplex(exit.position, origin,
                                              planeE1, planeE2);

    // Endpoint tangent DIRECTIONS in the corner plane (unit complex).
    const std::complex<double> t0 = toComplex(entry.tangent, RVec::zero(entry.position.dim()),
                                              planeE1, planeE2);
    const std::complex<double> t1 = toComplex(exit.tangent, RVec::zero(exit.position.dim()),
                                              planeE1, planeE2);

    // Tangent magnitudes: c·‖unit‖ with c = 2θ/π as [R1] default.
    // θ is the corner angle; we recover it from the tangent directions.
    const double dot = std::max(-1.0, std::min(1.0,
        (t0 / std::abs(t0) * std::conj(t1 / std::abs(t1))).real()));
    const double theta = std::acos(dot);
    const double c = 2.0 * theta / kPi;
    const std::complex<double> d0 = t0 * c; // magnitude c, direction t0
    const std::complex<double> d1 = t1 * c;

    // (M17) Hermite construction — four candidates from ± choices.
    //   ω₀ = ±√d₀,  ω₂ = ±√d₁
    //   ω₁ = −3(ω₀+ω₂)/4 ± (1/4)√(120(r₁−r₀) − 15ω₀² − 15ω₂² + 10ω₀ω₂)
    const auto [w0p, w0m] = csqrt(d0);
    const auto [w2p, w2m] = csqrt(d1);
    const std::complex<double> dr = r1 - r0;

    struct SignChoice { std::complex<double> w0, w2; };
    const SignChoice signs[4] = {
        {w0p, w2p}, {w0p, w2m}, {w0m, w2p}, {w0m, w2m}
    };

    std::vector<Result> results;
    results.reserve(8); // up to 8 (4 sign pairs × 2 for ω₁ ±)

    for (const auto& sc : signs) {
        const std::complex<double>& w0 = sc.w0;
        const std::complex<double>& w2 = sc.w2;
        const std::complex<double> radicand =
            120.0 * dr - 15.0 * w0 * w0 - 15.0 * w2 * w2 + 10.0 * w0 * w2;
        const auto [rp, rm] = csqrt(radicand);
        const std::complex<double> w1p = -0.75 * (w0 + w2) + 0.25 * rp;
        const std::complex<double> w1m = -0.75 * (w0 + w2) + 0.25 * rm;

        for (const std::complex<double>& w1 : {w1p, w1m}) {
            const auto sigma = speedCoeffs(w0, w1, w2);
            const bool degen = isDegenerate(sigma);

            // Build the Bézier control points by integrating the hodograph
            // (M16): r₀ = entry point, r_k = r_{k-1} + h_{k-1}/5.
            const auto h = hodographCoeffs(w0, w1, w2);
            std::array<std::complex<double>, 6> r;
            r[0] = r0;
            r[1] = r[0] + h[0] / 5.0;
            r[2] = r[1] + h[1] / 5.0;
            r[3] = r[2] + h[2] / 5.0;
            r[4] = r[3] + h[3] / 5.0;
            r[5] = r[4] + h[4] / 5.0;

            // Lift to ℝᴺ.
            std::vector<RVec> cps(6);
            for (int k = 0; k < 6; ++k) {
                cps[k] = fromComplex(r[k], origin, planeE1, planeE2);
            }

            // Build the NURBS (weights 1, degree 5, clamped knots).
            std::vector<double> wts(6, 1.0);
            std::vector<double> knots(12, 0.0);
            for (int i = 6; i < 12; ++i) knots[i] = 1.0;

            Result res{NurbsCurve(cps, wts, knots, 5), PHData{}, degen};
            res.ph.w0 = w0;
            res.ph.w1 = w1;
            res.ph.w2 = w2;
            res.ph.planeE1 = planeE1;
            res.ph.planeE2 = planeE2;
            res.ph.origin = origin;
            results.push_back(std::move(res));
        }
    }

    return results;
}

double PHQuinticBlendBuilder::arcLength(const PHData& ph, double xi) {
    // (M16): s(ξ) = ∫₀^ξ σ(τ) dτ = Σ_{k=0}^5 s_k B_k^5(ξ), s₀ = 0,
    // s_k = (1/5) Σ_{i=0}^{k-1} σ_i.
    const auto sigma = speedCoeffs(ph.w0, ph.w1, ph.w2);
    std::array<double, 6> s;
    s[0] = 0.0;
    for (int k = 1; k <= 5; ++k) {
        double acc = 0.0;
        for (int i = 0; i < k; ++i) acc += sigma[i];
        s[k] = acc / 5.0;
    }
    // Evaluate degree-5 Bernstein at xi (de Casteljau).
    double b[6];
    for (int i = 0; i < 6; ++i) b[i] = s[i];
    for (int r = 1; r <= 5; ++r)
        for (int i = 0; i <= 5 - r; ++i)
            b[i] = (1.0 - xi) * b[i] + xi * b[i + 1];
    return b[0];
}

double PHQuinticBlendBuilder::invertArcLength(const PHData& ph, double s) {
    // (M19): Newton–Raphson on s(ξ) − s_target = 0, f'(ξ) = σ(ξ) > 0.
    // Bracketed by bisection on [0,1].
    const auto sigma = speedCoeffs(ph.w0, ph.w1, ph.w2);
    const double sTotal = arcLength(ph, 1.0);
    if (sTotal <= 0.0) {
        throw std::domain_error(
            "PHQuinticBlendBuilder::invertArcLength: zero-length PH curve");
    }
    s = std::max(0.0, std::min(s, sTotal));
    if (s <= 0.0) return 0.0;
    if (s >= sTotal) return 1.0;

    double a = 0.0, b = 1.0;
    double xi = s / sTotal; // initial guess
    for (int iter = 0; iter < 60; ++iter) {
        const double f = arcLength(ph, xi) - s;
        if (std::abs(f) <= 1e-12 * std::max(sTotal, 1.0)) return xi;
        const double df = evalBernstein4(sigma, xi);
        if (f > 0.0) b = xi; else a = xi;
        if (b - a <= 1e-14) return 0.5 * (a + b);
        double xn = xi;
        if (df > 0.0) xn = xi - f / df;
        if (!(xn > a && xn < b)) xn = 0.5 * (a + b);
        xi = xn;
    }
    return 0.5 * (a + b);
}

double PHQuinticBlendBuilder::curvature(const PHData& ph, double xi) {
    // (M16): κ(ξ) = 2(u v' − u' v) / σ²(ξ).
    // ω(ξ) = ω₀(1−ξ)² + 2ω₁(1−ξ)ξ + ω₂ξ²
    // ω'(ξ) = 2[ω₁(1−ξ) + (ω₂−ω₁)ξ] ... more precisely:
    //   ω(ξ)  = ω₀(1−ξ)² + 2ω₁(1−ξ)ξ + ω₂ξ²
    //   ω'(ξ) = −2ω₀(1−ξ) + 2ω₁(1−2ξ) + 2ω₂ξ
    const double u = (ph.w0 * std::pow(1.0 - xi, 2) +
                      2.0 * ph.w1 * (1.0 - xi) * xi +
                      ph.w2 * xi * xi).real();
    const double v = (ph.w0 * std::pow(1.0 - xi, 2) +
                      2.0 * ph.w1 * (1.0 - xi) * xi +
                      ph.w2 * xi * xi).imag();
    const std::complex<double> omegaPrime =
        -2.0 * ph.w0 * (1.0 - xi) +
        2.0 * ph.w1 * (1.0 - 2.0 * xi) +
        2.0 * ph.w2 * xi;
    const double up = omegaPrime.real();
    const double vp = omegaPrime.imag();
    const auto sigma = speedCoeffs(ph.w0, ph.w1, ph.w2);
    const double sig = evalBernstein4(sigma, xi);
    if (sig <= 1e-15) {
        throw std::domain_error(
            "PHQuinticBlendBuilder::curvature: degenerate (σ ≈ 0)");
    }
    return 2.0 * (u * vp - up * v) / (sig * sig);
}

} // namespace tether::motion
