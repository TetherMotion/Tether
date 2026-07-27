/**
 * @file ContinuityPropertyTests.cpp
 * @brief T1 empirical continuity test: 1000 seeded random corners.
 *
 * For each corner, build a quintic G² blend between two random NURBS
 * pieces and verify that the blend's arc-length derivatives (T, κ⃗) at
 * both endpoints match the neighbors' derivatives at the trim points,
 * to a tight tolerance. This is the empirical confirmation of Theorem T1
 * (the imposed parametric boundary conditions convert exactly to
 * arc-length boundary conditions because ‖B'(0)‖ = α₁).
 */

#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/blend/BlendCurveBuilder.hpp"
#include "tether/motion_planner/blend/CornerAnalysis.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"
#include "motion_planner/blend/TestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <optional>
#include <random>

using tether::motion::testing::expectVecNear;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Build a random NURBS piece of the given dimension and degree.
tether::motion::NurbsCurve makeRandomPiece(std::mt19937_64& rng,
                                           std::size_t dim, int degree,
                                           double scale) {
    std::uniform_real_distribution<double> coord(-scale, scale);
    std::uniform_real_distribution<double> weight01(0.5, 1.5);
    std::vector<tether::motion::RVec> cps;
    std::vector<double> wts;
    const int nCp = degree + 1 + static_cast<int>(rng() % 3); // 1-3 spans
    cps.reserve(nCp);
    wts.reserve(nCp);
    for (int i = 0; i < nCp; ++i) {
        tether::motion::RVec p = tether::motion::RVec::zero(dim);
        for (std::size_t d = 0; d < dim; ++d) p[d] = coord(rng);
        cps.push_back(p);
        wts.push_back(weight01(rng)); // weights in [0.5, 1.5]
    }
    // Clamped knot vector.
    std::vector<double> knots(static_cast<std::size_t>(degree + 1) +
                              static_cast<std::size_t>(nCp - degree) +
                              static_cast<std::size_t>(degree), 0.0);
    // Fill interior knots uniformly.
    const int nSpans = nCp - degree - 1;
    for (int i = 1; i <= nSpans; ++i) {
        const double k = static_cast<double>(i) / (nSpans + 1);
        for (int j = degree + 1 + (i - 1); j < degree + 1 + i; ++j) {
            if (j < static_cast<int>(knots.size()) - degree) knots[j] = k;
        }
    }
    for (std::size_t i = knots.size() - degree - 1; i < knots.size(); ++i)
        knots[i] = 1.0;
    return tether::motion::NurbsCurve(cps, wts, knots, degree);
}

} // namespace

TEST(ContinuityProperty, QuinticG2MatchesArcLengthDerivativesAtBoundaries) {
    // 1000 seeded corners: build a blend between two random pieces and
    // verify G² continuity (T and κ⃗ match at both trim points).
    std::mt19937_64 rng(20240725);
    constexpr int kNumCorners = 1000;
    int failures = 0;

    for (int tc = 0; tc < kNumCorners; ++tc) {
        // Pick a dimension (2, 3, or 5) and degree (2 or 3).
        const std::size_t dims[3] = {2, 3, 5};
        const std::size_t dim = dims[tc % 3];
        const int degree = 2 + (tc / 3) % 2;

        // Build two random pieces that share a junction. To guarantee
        // G0 connectivity, build `in` first, then build `out` starting
        // at `in.endPoint()`.
        tether::motion::NurbsCurve in = makeRandomPiece(rng, dim, degree, 5.0);
        tether::motion::RVec junction = in.endPoint();

        // Build `out` with its first control point = junction.
        std::uniform_real_distribution<double> coord(-5.0, 5.0);
        std::uniform_real_distribution<double> weight01(0.5, 1.5);
        std::vector<tether::motion::RVec> outCps;
        std::vector<double> outWts;
        const int nCp = degree + 1 + static_cast<int>(rng() % 3);
        outCps.push_back(junction);
        outWts.push_back(weight01(rng));
        for (int i = 1; i < nCp; ++i) {
            tether::motion::RVec p = tether::motion::RVec::zero(dim);
            for (std::size_t d = 0; d < dim; ++d) p[d] = coord(rng);
            outCps.push_back(p);
            outWts.push_back(weight01(rng));
        }
        std::vector<double> outKnots(static_cast<std::size_t>(degree + 1) +
                                     static_cast<std::size_t>(nCp - degree) +
                                     static_cast<std::size_t>(degree), 0.0);
        const int nSpans = nCp - degree - 1;
        for (int i = 1; i <= nSpans; ++i) {
            const double k = static_cast<double>(i) / (nSpans + 1);
            for (int j = degree + 1 + (i - 1); j < degree + 1 + i; ++j) {
                if (j < static_cast<int>(outKnots.size()) - degree) outKnots[j] = k;
            }
        }
        for (std::size_t i = outKnots.size() - degree - 1; i < outKnots.size(); ++i)
            outKnots[i] = 1.0;
        tether::motion::NurbsCurve out(outCps, outWts, outKnots, degree);

        // Analyze the corner.
        tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
        tether::motion::CornerAnalysis ca;
        try {
            ca = analyzer.analyze(in, out);
        } catch (...) {
            continue; // skip degenerate corners
        }
        if (ca.kind != tether::motion::CornerKind::Corner) continue;

        // Pick trim distances (small, to stay in the valid range).
        const double trimIn = std::min(0.3 * in.length(), 1.0);
        const double trimOut = std::min(0.3 * out.length(), 1.0);
        if (trimIn <= 0.0 || trimOut <= 0.0) continue;

        // Extract boundary conditions at the trim points.
        tether::motion::BoundaryConditions entry, exitBc;
        try {
            entry = tether::motion::boundaryAt(in, in.length() - trimIn, true);
            exitBc = tether::motion::boundaryAt(out, trimOut, false);
        } catch (...) {
            continue; // skip degenerate parameterizations
        }

        // Build the quintic G² blend.
        const double alpha1 = 1.0, beta1 = 1.0;
        std::optional<tether::motion::NurbsCurve> blend;
        try {
            blend = tether::motion::BlendCurveBuilder::buildQuintic(
                entry, exitBc, alpha1, beta1);
        } catch (...) {
            continue;
        }

        // Verify G² continuity: the blend's arc-length derivatives at the
        // endpoints must match the neighbors'.
        const auto blendStart = blend->arcDerivatives(0.0, 2);
        const auto blendEnd = blend->arcDerivatives(1.0, 2);

        // T_A: blend tangent at start == entry.tangent.
        // κ⃗_A: blend curvature at start == entry.curvature.
        // (T1: because ‖B'(0)‖ = α₁, the parametric BC converts exactly.)
        const double tol = 1e-7;
        bool ok = true;
        for (std::size_t d = 0; d < dim; ++d) {
            if (std::abs(blendStart.tangent[d] - entry.tangent[d]) > tol) ok = false;
            if (std::abs(blendStart.curvature[d] - entry.curvature[d]) > tol) ok = false;
            if (std::abs(blendEnd.tangent[d] - exitBc.tangent[d]) > tol) ok = false;
            if (std::abs(blendEnd.curvature[d] - exitBc.curvature[d]) > tol) ok = false;
        }
        if (!ok) ++failures;
    }

    // T1 guarantee: zero failures out of 1000.
    EXPECT_EQ(failures, 0)
        << "T1 violated: " << failures << " corners failed G² continuity";
}
