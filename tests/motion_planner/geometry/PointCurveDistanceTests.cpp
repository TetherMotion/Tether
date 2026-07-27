/**
 * @file PointCurveDistanceTests.cpp
 * @brief Tests for certified point-to-NURBS distance
 *
 * Closed-form cases (line, arc) plus a seeded fuzz comparison against dense
 * brute-force sampling: the certified result must never exceed the
 * brute-force minimum (no missed stationary points), and the brute-force
 * minimum must bound the certified result from above within the sampling
 * error.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/geometry/PointCurveDistance.hpp>

#include <cmath>
#include <random>
#include <vector>

using tether::motion::DistanceResult;
using tether::motion::NurbsCurve;
using tether::motion::RVec;
using tether::motion::pointCurveDistance;

namespace {

constexpr double kPi = 3.14159265358979323846;

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
    std::vector<double> knots;
    for (int i = 0; i <= degree; ++i) knots.push_back(0.0);
    const int interior = numCps - degree - 1;
    for (int i = 1; i <= interior; ++i) {
        knots.push_back(static_cast<double>(i) / (interior + 1));
    }
    for (int i = 0; i <= degree; ++i) knots.push_back(1.0);
    return NurbsCurve(std::move(cps), std::move(wts), std::move(knots), degree);
}

double bruteForceDistance(const NurbsCurve& c, const RVec& p, int samples) {
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= samples; ++i) {
        const double u = c.knotMin() +
                         (c.knotMax() - c.knotMin()) * i / samples;
        best = std::min(best, c.evaluate(u).distanceTo(p));
    }
    return best;
}

} // namespace

TEST(PointCurveDistance, LineClosedForm) {
    NurbsCurve line = NurbsCurve::fromLine(RVec{0.0, 0.0}, RVec{10.0, 0.0});

    DistanceResult r1 = pointCurveDistance(line, RVec{5.0, 3.0});
    EXPECT_NEAR(r1.distance, 3.0, 1e-10);
    EXPECT_NEAR(r1.u, 0.5, 1e-9);
    EXPECT_NEAR(r1.closestPoint[0], 5.0, 1e-9);
    EXPECT_NEAR(r1.closestPoint[1], 0.0, 1e-9);

    // Projection beyond the start: endpoint wins.
    DistanceResult r2 = pointCurveDistance(line, RVec{-2.0, 1.0});
    EXPECT_NEAR(r2.distance, std::sqrt(5.0), 1e-10);
    EXPECT_NEAR(r2.u, 0.0, 1e-12);

    // Point on the curve.
    DistanceResult r3 = pointCurveDistance(line, RVec{7.5, 0.0});
    EXPECT_NEAR(r3.distance, 0.0, 1e-9);
}

TEST(PointCurveDistance, ArcClosedForm) {
    // Quarter circle R=2 centered at origin in the first quadrant.
    NurbsCurve arc = NurbsCurve::fromArc(RVec{0.0, 0.0}, 2.0, RVec{1.0, 0.0},
                                         RVec{0.0, 1.0}, 0.0, kPi / 2.0);

    // Radial closest point for p = (3,3): (√2, √2), distance 3√2 − 2.
    DistanceResult r = pointCurveDistance(arc, RVec{3.0, 3.0});
    EXPECT_NEAR(r.distance, 3.0 * std::sqrt(2.0) - 2.0, 1e-9);
    EXPECT_NEAR(r.closestPoint[0], std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(r.closestPoint[1], std::sqrt(2.0), 1e-9);

    // Point beyond the arc's angular range: the nearest endpoint wins.
    // (-1, 0.1) is closer to the arc end (0, 2) than to the arc start (2, 0):
    //   dist to (0, 2) = sqrt(1 + 3.61) = sqrt(4.61) ≈ 2.147
    //   dist to (2, 0) = sqrt(9 + 0.01) = sqrt(9.01) ≈ 3.002
    DistanceResult r2 = pointCurveDistance(arc, RVec{-1.0, 0.1});
    EXPECT_NEAR(r2.distance, (RVec{-1.0, 0.1}.distanceTo(RVec{0.0, 2.0})), 1e-9);

    // Consistency: |closestPoint − p| == distance.
    EXPECT_NEAR(r.closestPoint.distanceTo(RVec{3.0, 3.0}), r.distance, 1e-10);
}

TEST(PointCurveDistance, CertifiedVsBruteForceRandomNurbs) {
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> coord(-5.0, 5.0);

    const std::size_t dims[3] = {2, 3, 5};
    int failures = 0;
    for (int tc = 0; tc < 100; ++tc) {
        const std::size_t dim = dims[tc % 3];
        const int degree = 2 + tc % 3;           // 2..4
        const int numCps = degree + 2 + tc % 3;  // a few spans
        const bool rational = (tc % 2) == 0;

        NurbsCurve c = makeRandomNurbs(rng, dim, degree, numCps, rational);
        RVec p = RVec::zero(dim);
        for (std::size_t d = 0; d < dim; ++d) p[d] = coord(rng);

        const DistanceResult certified = pointCurveDistance(c, p);
        const double brute = bruteForceDistance(c, p, 100000);

        // Certification property: certified ≤ brute-force minimum
        // (brute force can only overestimate the true minimum).
        EXPECT_LE(certified.distance, brute + 1e-9)
            << "case " << tc << " (dim " << dim << ", deg " << degree << ")";

        // Brute-force minimum bounds the true value within the sampling
        // error. Error bound: max over samples of |d dist/du|·Δu where
        // Δu = domain/samples and |d dist/du| ≤ max |C'(u)|; we estimate the
        // speed bound by the control-polygon edge maximum times degree.
        double maxEdge = 0.0;
        for (std::size_t i = 1; i < c.controlPoints().size(); ++i) {
            maxEdge = std::max(
                maxEdge, c.controlPoints()[i].distanceTo(c.controlPoints()[i - 1]));
        }
        const double speedBound =
            degree * maxEdge / (c.knotMax() - c.knotMin()) * numCps;
        const double samplingError =
            speedBound * (c.knotMax() - c.knotMin()) / 100000.0 + 1e-9;
        EXPECT_GE(brute, certified.distance - 1e-9);
        EXPECT_LE(brute, certified.distance + samplingError)
            << "case " << tc << " (dim " << dim << ", deg " << degree << ")";

        // Closest point consistency.
        EXPECT_NEAR(certified.closestPoint.distanceTo(p), certified.distance,
                    1e-8);
        if (certified.distance > brute + 1e-9) ++failures;
    }
    EXPECT_EQ(failures, 0);
}

TEST(PointCurveDistance, DimensionMismatchThrows) {
    NurbsCurve line = NurbsCurve::fromLine(RVec{0.0, 0.0}, RVec{1.0, 0.0});
    EXPECT_THROW((pointCurveDistance(line, RVec{0.0, 0.0, 0.0})),
                 std::invalid_argument);
}
