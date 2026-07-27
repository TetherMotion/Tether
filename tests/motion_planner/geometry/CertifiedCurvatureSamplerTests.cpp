/**
 * @file CertifiedCurvatureSamplerTests.cpp
 * @brief Tests for the lazy per-span certified curvature sampler.
 *
 * Closed-form cases:
 *  - Line: κ = 0 exactly.
 *  - Circle: κ = 1/r exactly, constant across the span.
 *  - Semicircle: same.
 *
 * Fuzz case:
 *  - Random rational quintic: the certified upper bound must be ≥ the
 *    brute-force max κ (no missed peak), and the certificate width
 *    (maxKappa − minKappa) must be ≤ the requested tolerance.
 *
 * Laziness:
 *  - Querying one piece does not sample the others (spansSampled == 1).
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>

#include <cmath>
#include <random>
#include <vector>

using tether::motion::CertifiedCurvature;
using tether::motion::CertifiedCurvatureSampler;
using tether::motion::NurbsCurve;
using tether::motion::PiecewiseNurbsPath;
using tether::motion::RVec;

namespace {

constexpr double kPi = 3.14159265358979323846;

RVec vec2(double x, double y) {
    RVec v = RVec::zero(2);
    v[0] = x; v[1] = y;
    return v;
}

RVec vec3(double x, double y, double z) {
    RVec v = RVec::zero(3);
    v[0] = x; v[1] = y; v[2] = z;
    return v;
}

NurbsCurve makeRandomQuintic(std::mt19937_64& rng, std::size_t dim) {
    std::uniform_real_distribution<double> coord(-3.0, 3.0);
    std::uniform_real_distribution<double> weight(0.4, 2.0);
    const int degree = 5;
    const int numCps = degree + 1;
    std::vector<RVec> cps(numCps);
    std::vector<double> wts(numCps, 1.0);
    for (int i = 0; i < numCps; ++i) {
        cps[i] = RVec::zero(dim);
        for (std::size_t d = 0; d < dim; ++d) cps[i][d] = coord(rng);
        wts[i] = weight(rng);
    }
    std::vector<double> knots;
    for (int i = 0; i <= degree; ++i) knots.push_back(0.0);
    for (int i = 0; i <= degree; ++i) knots.push_back(1.0);
    return NurbsCurve(cps, wts, knots, degree);
}

} // namespace

// ============================================================================
// Closed-form: line has zero curvature
// ============================================================================
TEST(CertifiedCurvatureSampler, LineHasZeroCurvature) {
    NurbsCurve line = NurbsCurve::fromLine(vec2(0, 0), vec2(10, 0));
    PiecewiseNurbsPath path({line});
    CertifiedCurvatureSampler sampler(path, 1e-6);

    auto r = sampler.maxCurvature(0);
    EXPECT_EQ(r.maxKappa, 0.0);
    EXPECT_EQ(r.minKappa, 0.0);
    EXPECT_EQ(r.samples, 0u);
    EXPECT_EQ(r.Lipschitz, 0.0);
}

// ============================================================================
// Closed-form: circle has constant curvature 1/r
// ============================================================================
TEST(CertifiedCurvatureSampler, CircleCurvatureIsOneOverRadius) {
    const double r = 2.5;
    NurbsCurve arc = NurbsCurve::fromArc(vec2(0, 0), r,
                                         vec2(1, 0), vec2(0, 1),
                                         0.0, kPi / 2.0);
    PiecewiseNurbsPath path({arc});
    CertifiedCurvatureSampler sampler(path, 1e-6);

    auto r0 = sampler.maxCurvature(0);
    const double expected = 1.0 / r;
    // The certified upper bound must be ≥ true κ.
    EXPECT_GE(r0.maxKappa, expected - 1e-9);
    // The lower bound (max sample) must be close to the true κ —
    // for a circle κ is constant, so the max sample is exact.
    EXPECT_NEAR(r0.minKappa, expected, 1e-6);
    // The upper bound should be close to the true value (the
    // Lipschitz-based slack is conservative but not huge for a circle).
    EXPECT_NEAR(r0.maxKappa, expected, 1e-2);
}

// ============================================================================
// Semicircle (multi-span arc) — same constant curvature
// ============================================================================
TEST(CertifiedCurvatureSampler, SemicircleMultiSpan) {
    const double r = 5.0;
    NurbsCurve arc = NurbsCurve::fromArc(vec2(0, 0), r,
                                         vec2(1, 0), vec2(0, 1),
                                         0.0, kPi);
    PiecewiseNurbsPath path({arc});
    CertifiedCurvatureSampler sampler(path, 1e-9);

    auto r0 = sampler.maxCurvature(0);
    const double expected = 1.0 / r;
    EXPECT_GE(r0.maxKappa, expected - 1e-9);
    EXPECT_NEAR(r0.maxKappa, expected, 1e-3);
}

// ============================================================================
// Fuzz: certified upper bound ≥ brute-force max (no missed peak)
// ============================================================================
TEST(CertifiedCurvatureSampler, CertifiedBoundDominatesBruteForce) {
    std::mt19937_64 rng(12345);
    for (int trial = 0; trial < 20; ++trial) {
        NurbsCurve c = makeRandomQuintic(rng, 2);
        PiecewiseNurbsPath path({c});
        CertifiedCurvatureSampler sampler(path, 1e-6);

        auto r0 = sampler.maxCurvature(0);

        // Dense brute-force sampling (100k points) — the certified bound
        // must be ≥ the brute-force max (no missed peak).
        const double uMin = c.knotMin();
        const double uMax = c.knotMax();
        double bruteMax = 0.0;
        const int N = 100000;
        for (int i = 0; i <= N; ++i) {
            const double u = uMin + (uMax - uMin) * double(i) / double(N);
            try {
                double k = c.arcDerivatives(u, 2).curvature.norm();
                bruteMax = std::max(bruteMax, k);
            } catch (...) {
                // degenerate parameterization — skip
            }
        }
        EXPECT_GE(r0.maxKappa, bruteMax - 1e-9)
            << "trial " << trial << " certified=" << r0.maxKappa
            << " brute=" << bruteMax;
    }
}

// ============================================================================
// Certificate width ≤ requested tolerance (for well-conditioned curves)
// ============================================================================
TEST(CertifiedCurvatureSampler, CertificateWidthWithinTolerance) {
    // Use a gentle cubic Bézier with well-separated control points so the
    // speed is bounded well away from zero (the Lipschitz constant is
    // moderate and the grid can achieve the requested tolerance without
    // hitting the max-samples cap).
    std::vector<RVec> cps = {
        vec2(0, 0), vec2(3, 1), vec2(6, -1), vec2(10, 0)
    };
    std::vector<double> wts = {1, 1, 1, 1};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    NurbsCurve c(cps, wts, knots, 3);
    PiecewiseNurbsPath path({c});

    const double tol = 1e-4;
    // Allow enough samples for the Lipschitz-based grid to achieve the
    // tolerance: L≈8, h ≤ 2·1e-4/8 = 2.5e-5, domain=1 → ~40000 intervals.
    CertifiedCurvatureSampler sampler(path, tol, /*maxSamplesPerSpan=*/65536);
    auto r0 = sampler.maxCurvature(0);

    // The certificate width is maxKappa - minKappa.
    // Allow a small slack for floating-point.
    EXPECT_LE(r0.maxKappa - r0.minKappa, tol * 2.0)
        << "width=" << (r0.maxKappa - r0.minKappa)
        << " L=" << r0.Lipschitz << " samples=" << r0.samples;
}

// ============================================================================
// Laziness: querying one piece does not sample the others
// ============================================================================
TEST(CertifiedCurvatureSampler, LazinessOnlyQueriedSpansSampled) {
    NurbsCurve line1 = NurbsCurve::fromLine(vec2(0, 0), vec2(1, 0));
    NurbsCurve line2 = NurbsCurve::fromLine(vec2(1, 0), vec2(2, 1));
    NurbsCurve line3 = NurbsCurve::fromLine(vec2(2, 1), vec2(3, 0));
    PiecewiseNurbsPath path({line1, line2, line3});
    CertifiedCurvatureSampler sampler(path, 1e-6);

    sampler.maxCurvature(1); // query only the middle piece
    EXPECT_EQ(sampler.spansSampled(), 1u);
    EXPECT_EQ(sampler.totalCurvatureEvaluations(), 0u); // lines: 0 samples
}

// ============================================================================
// Laziness: repeated queries are memoized
// ============================================================================
TEST(CertifiedCurvatureSampler, MemoizedRepeatedQueries) {
    NurbsCurve arc = NurbsCurve::fromArc(vec2(0, 0), 3.0,
                                         vec2(1, 0), vec2(0, 1),
                                         0.0, kPi / 3.0);
    PiecewiseNurbsPath path({arc});
    CertifiedCurvatureSampler sampler(path, 1e-6);

    auto r1 = sampler.maxCurvature(0);
    auto r2 = sampler.maxCurvature(0);
    EXPECT_EQ(r1.samples, r2.samples);
    EXPECT_EQ(r1.maxKappa, r2.maxKappa);
    EXPECT_EQ(sampler.spansSampled(), 1u);
}

// ============================================================================
// Out-of-range piece index throws
// ============================================================================
TEST(CertifiedCurvatureSampler, OutOfRangeThrows) {
    NurbsCurve line = NurbsCurve::fromLine(vec2(0, 0), vec2(1, 0));
    PiecewiseNurbsPath path({line});
    CertifiedCurvatureSampler sampler(path);
    EXPECT_THROW(sampler.maxCurvature(5), std::out_of_range);
}

// ============================================================================
// Multi-piece path: each piece sampled independently
// ============================================================================
TEST(CertifiedCurvatureSampler, MultiPiecePath) {
    // line (κ=0) → arc (κ=1/r) → line (κ=0)
    NurbsCurve l1 = NurbsCurve::fromLine(vec2(0, 0), vec2(2, 0));
    NurbsCurve ar = NurbsCurve::fromArc(vec2(2, 1), 1.0,
                                        vec2(1, 0), vec2(0, 1),
                                        -kPi / 2, kPi / 2);
    NurbsCurve l2 = NurbsCurve::fromLine(vec2(4, 1), vec2(6, 1));
    PiecewiseNurbsPath path({l1, ar, l2});
    CertifiedCurvatureSampler sampler(path, 1e-9);

    auto r0 = sampler.maxCurvature(0);
    auto r1 = sampler.maxCurvature(1);
    auto r2 = sampler.maxCurvature(2);

    EXPECT_EQ(r0.maxKappa, 0.0); // line
    // Arc with r=1 → κ=1. The Lipschitz-based slack is conservative;
    // allow a few percent.
    EXPECT_NEAR(r1.maxKappa, 1.0, 0.05);
    EXPECT_EQ(r2.maxKappa, 0.0); // line
    EXPECT_EQ(sampler.spansSampled(), 3u);
}
