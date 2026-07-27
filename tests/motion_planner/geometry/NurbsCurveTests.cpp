/**
 * @file NurbsCurveTests.cpp
 * @brief Unit tests for tether::motion::NurbsCurve
 *
 * Covers: line/arc factories, exact evaluation, derivatives ≤ 3 vs closed
 * forms, arc-length derivatives vs circle closed form, split/trim exactness,
 * invertLength round-trips, 5-D curves, degenerate-input diagnostics.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/geometry/NurbsCurve.hpp>

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

using tether::motion::ArcDerivatives;
using tether::motion::NurbsCurve;
using tether::motion::RVec;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTight = 1e-12;

void expectVecNear(const RVec& a, const RVec& b, double tol) {
    ASSERT_EQ(a.dim(), b.dim());
    for (std::size_t i = 0; i < a.dim(); ++i) {
        EXPECT_NEAR(a[i], b[i], tol) << "component " << i;
    }
}

// Quarter-circle arc, radius 2, center origin, from (2,0) to (0,2).
NurbsCurve makeQuarterArc(double radius = 2.0) {
    return NurbsCurve::fromArc(RVec{0.0, 0.0}, radius, RVec{1.0, 0.0},
                               RVec{0.0, 1.0}, 0.0, kPi / 2.0);
}

// Seeded random clamped NURBS (used in several tests).
NurbsCurve makeRandomNurbs(std::mt19937_64& rng, std::size_t dim, int degree,
                           int numCps, bool rational) {
    std::uniform_real_distribution<double> coord(-5.0, 5.0);
    std::uniform_real_distribution<double> weight(0.3, 3.0);

    std::vector<RVec> cps(numCps);
    std::vector<double> wts(numCps, 1.0);
    for (int i = 0; i < numCps; ++i) {
        cps[i] = RVec::zero(dim);
        for (std::size_t d = 0; d < dim; ++d) cps[i][d] = coord(rng);
        if (rational) wts[i] = weight(rng);
    }

    // Clamped knot vector, quasi-uniform interior knots.
    std::vector<double> knots;
    for (int i = 0; i <= degree; ++i) knots.push_back(0.0);
    const int interior = numCps - degree - 1;
    for (int i = 1; i <= interior; ++i) {
        knots.push_back(static_cast<double>(i) / (interior + 1));
    }
    for (int i = 0; i <= degree; ++i) knots.push_back(1.0);
    return NurbsCurve(std::move(cps), std::move(wts), std::move(knots), degree);
}

} // namespace

// ============================================================================
// Construction / validation
// ============================================================================

TEST(NurbsCurveGeom, ValidationRejectsBadInput) {
    RVec a{0.0, 0.0}, b{1.0, 0.0}, c3{1.0, 0.0, 0.0};
    EXPECT_THROW((NurbsCurve({}, {1.0}, {0.0, 0.0}, 1)),
                 std::invalid_argument);
    // dim mismatch among control points
    EXPECT_THROW((NurbsCurve({a, c3}, {1.0, 1.0}, {0, 0, 1, 1}, 1)),
                 std::invalid_argument);
    // wrong knot count
    EXPECT_THROW((NurbsCurve({a, b}, {1.0, 1.0}, {0, 0, 1}, 1)),
                 std::invalid_argument);
    // wrong weights count
    EXPECT_THROW((NurbsCurve({a, b}, {1.0}, {0, 0, 1, 1}, 1)),
                 std::invalid_argument);
    // non-positive weight
    EXPECT_THROW((NurbsCurve({a, b}, {1.0, -1.0}, {0, 0, 1, 1}, 1)),
                 std::invalid_argument);
    // decreasing knots
    EXPECT_THROW((NurbsCurve({a, b}, {1.0, 1.0}, {1, 1, 0, 0}, 1)),
                 std::invalid_argument);
    // too few control points for the degree
    EXPECT_THROW((NurbsCurve({a, b}, {1.0, 1.0}, {0, 0, 1, 1}, 2)),
                 std::invalid_argument);
}

TEST(NurbsCurveGeom, DegenerateInputsDiagnosedNotCrash) {
    RVec a{1.0, 1.0};
    // Zero-length line factory.
    EXPECT_THROW(NurbsCurve::fromLine(a, a), std::invalid_argument);

    // Duplicate control points: curve valid, but arcDerivatives at the cusp
    // (|C'| = 0) must throw, not divide by zero.
    NurbsCurve cusp({RVec{0.0, 0.0}, RVec{0.0, 0.0}, RVec{1.0, 0.0}},
                    {1.0, 1.0, 1.0}, {0, 0, 0, 1, 1, 1}, 2);
    EXPECT_THROW(cusp.arcDerivatives(0.0, 1), std::domain_error);
    EXPECT_NO_THROW(cusp.evaluate(0.0));
    EXPECT_GT(cusp.length(), 0.0);
}

// ============================================================================
// Line
// ============================================================================

TEST(NurbsCurveGeom, LineClosedForm) {
    NurbsCurve line = NurbsCurve::fromLine(RVec{0.0, 0.0}, RVec{10.0, 0.0});
    EXPECT_EQ(line.degree(), 1);
    EXPECT_TRUE(line.isPolyline());

    expectVecNear(line.evaluate(0.5), RVec{5.0, 0.0}, kTight);
    expectVecNear(line.derivative(0.5, 1), RVec{10.0, 0.0}, kTight);
    expectVecNear(line.derivative(0.5, 2), RVec{0.0, 0.0}, kTight);
    expectVecNear(line.derivative(0.5, 3), RVec{0.0, 0.0}, kTight);

    ArcDerivatives ad = line.arcDerivatives(0.25, 3);
    expectVecNear(ad.position, RVec{2.5, 0.0}, kTight);
    expectVecNear(ad.tangent, RVec{1.0, 0.0}, kTight);
    expectVecNear(ad.curvature, RVec{0.0, 0.0}, kTight);
    expectVecNear(ad.jounce, RVec{0.0, 0.0}, kTight);

    EXPECT_NEAR(line.length(), 10.0, kTight);
    EXPECT_NEAR(line.arcLengthTo(0.3), 3.0, kTight);
    EXPECT_NEAR(line.invertLength(3.0), 0.3, kTight);
    // Line arc length is exact: no quadrature ever runs.
    EXPECT_EQ(line.arcLengthComputationCount(), 0u);
}

// ============================================================================
// Rational arc exactness
// ============================================================================

TEST(NurbsCurveGeom, ArcPointsOnCircleAndWeightsExact) {
    const double R = 2.0;
    NurbsCurve arc = makeQuarterArc(R);

    // Exact middle weight cos(sweep/2) = cos(pi/4).
    ASSERT_EQ(arc.weights().size(), 3u);
    EXPECT_DOUBLE_EQ(arc.weights()[1], std::cos(kPi / 4.0));

    // Every evaluated point lies exactly on the circle (relative 1e-12).
    for (int i = 0; i <= 100; ++i) {
        const double u = i / 100.0;
        const RVec p = arc.evaluate(u);
        EXPECT_NEAR(p.norm(), R, R * kTight) << "u = " << u;
    }
    // Exact endpoints.
    expectVecNear(arc.startPoint(), RVec{R, 0.0}, kTight);
    expectVecNear(arc.endPoint(), RVec{0.0, R}, kTight);
    // Exact arc length = R * sweep.
    EXPECT_NEAR(arc.length(), R * kPi / 2.0, 1e-10);
}

TEST(NurbsCurveGeom, ArcMultiSpanExactness) {
    // Sweep 3*pi/2 > pi forces multiple spans; the curve must still be an
    // exact circle and C1 across span junctions.
    const double R = 1.5;
    NurbsCurve arc = NurbsCurve::fromArc(RVec{1.0, -2.0}, R, RVec{1.0, 0.0},
                                         RVec{0.0, 1.0}, 0.3, 1.5 * kPi);
    for (int i = 0; i <= 200; ++i) {
        const double u = i / 200.0;
        const RVec p = arc.evaluate(u);
        EXPECT_NEAR(p.distanceTo(RVec{1.0, -2.0}), R, R * 1e-12)
            << "u = " << u;
    }
    EXPECT_NEAR(arc.length(), R * 1.5 * kPi, 1e-9);
}

// ============================================================================
// Arc-length derivatives vs circle closed form
// ============================================================================

TEST(NurbsCurveGeom, ArcDerivativesVsCircleClosedForm) {
    const double R = 2.0;
    // Half circle (single span, sweep = pi).
    NurbsCurve arc = NurbsCurve::fromArc(RVec{0.0, 0.0}, R, RVec{1.0, 0.0},
                                         RVec{0.0, 1.0}, 0.0, kPi);

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    for (int i = 0; i < 50; ++i) {
        const double u = uni(rng);
        const ArcDerivatives ad = arc.arcDerivatives(u, 3);

        // Point on circle, angle from position.
        const double theta = std::atan2(ad.position[1], ad.position[0]);
        // Closed form: T = (-sinθ, cosθ), |κ⃗| = 1/R, |j⃗| = 1/R²,
        // κ⃗·T = 0, j⃗·κ⃗ = 0, j⃗ = -(1/R²)·T.
        expectVecNear(ad.tangent, RVec{-std::sin(theta), std::cos(theta)},
                      1e-10);
        EXPECT_NEAR(ad.curvature.norm(), 1.0 / R, 1e-10);
        EXPECT_NEAR(ad.jounce.norm(), 1.0 / (R * R), 1e-9);
        EXPECT_NEAR(ad.curvature.dot(ad.tangent), 0.0, 1e-10);
        EXPECT_NEAR(ad.jounce.dot(ad.curvature), 0.0, 1e-9);
        // κ⃗ points at the center: κ⃗ = -(1/R²)·p.
        expectVecNear(ad.curvature, ad.position * (-1.0 / (R * R)), 1e-10);
    }
}

// ============================================================================
// Parametric derivatives vs finite differences (rational, degree 3)
// ============================================================================

TEST(NurbsCurveGeom, ParametricDerivativesMatchFiniteDifferences) {
    std::mt19937_64 rng(7);
    NurbsCurve c = makeRandomNurbs(rng, 3, 3, 6, /*rational=*/true);

    const double h = 1e-6;
    std::uniform_real_distribution<double> uni(0.01, 0.99);
    for (int i = 0; i < 20; ++i) {
        const double u = uni(rng);
        const RVec d1 = c.derivative(u, 1);
        const RVec d2 = c.derivative(u, 2);
        const RVec d3 = c.derivative(u, 3);

        const RVec fdm1 = (c.evaluate(u + h) - c.evaluate(u - h)) / (2 * h);
        const RVec fdm2 =
            (c.derivative(u + h, 1) - c.derivative(u - h, 1)) / (2 * h);
        const RVec fdm3 =
            (c.derivative(u + h, 2) - c.derivative(u - h, 2)) / (2 * h);

        expectVecNear(d1, fdm1, 1e-5);
        expectVecNear(d2, fdm2, 1e-4);
        expectVecNear(d3, fdm3, 1e-3);
    }
}

// ============================================================================
// Split / trim / invertLength
// ============================================================================

TEST(NurbsCurveGeom, SplitReproducesOriginalCurve) {
    std::mt19937_64 rng(11);
    NurbsCurve c = makeRandomNurbs(rng, 2, 3, 6, /*rational=*/true);

    const double uSplit = 0.37;
    auto halves = c.split(uSplit);

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    for (int i = 0; i < 50; ++i) {
        const double u = uni(rng);
        const RVec ref = c.evaluate(u);
        if (u <= uSplit) {
            expectVecNear(halves.first.evaluate(u), ref, 1e-11);
        } else {
            expectVecNear(halves.second.evaluate(u), ref, 1e-11);
        }
    }
    // Junction points coincide.
    expectVecNear(halves.first.endPoint(), halves.second.startPoint(), kTight);
    // Derivatives also reproduce (exact representation).
    expectVecNear(c.derivative(0.2, 2), halves.first.derivative(0.2, 2), 1e-9);
    expectVecNear(c.derivative(0.8, 2), halves.second.derivative(0.8, 2), 1e-9);
}

TEST(NurbsCurveGeom, InvertLengthRoundTrip) {
    std::mt19937_64 rng(13);
    NurbsCurve c = makeRandomNurbs(rng, 3, 3, 5, /*rational=*/false);
    const double L = c.length();

    std::uniform_real_distribution<double> uni(0.0, L);
    for (int i = 0; i < 20; ++i) {
        const double s = uni(rng);
        const double u = c.invertLength(s);
        EXPECT_NEAR(c.arcLengthTo(u), s, 1e-10) << "s = " << s;
    }
    EXPECT_DOUBLE_EQ(c.invertLength(0.0), c.knotMin());
    EXPECT_DOUBLE_EQ(c.invertLength(L), c.knotMax());
}

TEST(NurbsCurveGeom, TrimRoundTrip) {
    std::mt19937_64 rng(17);
    NurbsCurve c = makeRandomNurbs(rng, 2, 4, 6, /*rational=*/true);
    const double L = c.length();

    const double s0 = 0.2 * L, s1 = 0.7 * L;
    NurbsCurve t = c.trim(s0, s1);
    EXPECT_NEAR(t.length(), s1 - s0, 1e-9);

    // Endpoints of the trimmed curve match the original at the same lengths.
    expectVecNear(t.startPoint(), c.evaluate(c.invertLength(s0)), 1e-9);
    expectVecNear(t.endPoint(), c.evaluate(c.invertLength(s1)), 1e-9);

    // Trim of the full range reproduces the curve.
    NurbsCurve full = c.trim(0.0, L);
    std::mt19937_64 rng2(19);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    for (int i = 0; i < 10; ++i) {
        const double u = uni(rng2);
        expectVecNear(full.evaluate(u), c.evaluate(u), 1e-10);
    }
}

// ============================================================================
// Bézier decomposition
// ============================================================================

TEST(NurbsCurveGeom, BezierDecomposeReproducesCurve) {
    std::mt19937_64 rng(23);
    NurbsCurve c = makeRandomNurbs(rng, 2, 3, 7, /*rational=*/true);
    auto pieces = c.bezierDecompose();
    ASSERT_EQ(pieces.size(), 4u); // 7 cps, degree 3 → 4 spans
    for (const NurbsCurve& p : pieces) {
        EXPECT_EQ(p.numControlPoints(), 4u);
    }
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    for (int i = 0; i < 50; ++i) {
        const double u = uni(rng);
        const RVec ref = c.evaluate(u);
        // Find the piece covering u.
        bool checked = false;
        for (const NurbsCurve& p : pieces) {
            if (u >= p.knotMin() && u <= p.knotMax()) {
                expectVecNear(p.evaluate(u), ref, 1e-11);
                checked = true;
                break;
            }
        }
        EXPECT_TRUE(checked);
    }
}

// ============================================================================
// 5-D curve through all functions
// ============================================================================

TEST(NurbsCurveGeom, FiveDimensionalCurve) {
    // 5-D arc embedded in the (e1, e3) plane.
    RVec center = RVec::zero(5);
    RVec ax1 = RVec::zero(5); ax1[0] = 1.0;
    RVec ax2 = RVec::zero(5); ax2[2] = 1.0;
    const double R = 3.0;
    NurbsCurve arc = NurbsCurve::fromArc(center, R, ax1, ax2, 0.0, kPi / 2.0);

    EXPECT_EQ(arc.dim(), 5u);
    for (int i = 0; i <= 20; ++i) {
        const RVec p = arc.evaluate(i / 20.0);
        EXPECT_NEAR(p.norm(), R, R * 1e-12);
        EXPECT_DOUBLE_EQ(p[1], 0.0);
        EXPECT_DOUBLE_EQ(p[3], 0.0);
        EXPECT_DOUBLE_EQ(p[4], 0.0);
    }
    EXPECT_NEAR(arc.length(), R * kPi / 2.0, 1e-10);

    ArcDerivatives ad = arc.arcDerivatives(0.3, 3);
    EXPECT_NEAR(ad.tangent.norm(), 1.0, 1e-12);
    EXPECT_NEAR(ad.curvature.norm(), 1.0 / R, 1e-10);
    EXPECT_NEAR(ad.jounce.norm(), 1.0 / (R * R), 1e-9);

    // Generic 5-D rational NURBS through eval/derivative/length/split/invert.
    std::mt19937_64 rng(29);
    NurbsCurve c = makeRandomNurbs(rng, 5, 3, 6, /*rational=*/true);
    const double L = c.length();
    EXPECT_GT(L, 0.0);
    const double uMid = c.invertLength(0.5 * L);
    auto halves = c.split(uMid);
    expectVecNear(halves.first.endPoint(), c.evaluate(uMid), 1e-9);
    EXPECT_NEAR(halves.first.length() + halves.second.length(), L, 1e-8);
    EXPECT_NO_THROW(c.arcDerivatives(0.5, 3));
}
