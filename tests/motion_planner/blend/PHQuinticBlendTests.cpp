/**
 * @file PHQuinticBlendTests.cpp
 * @brief T4 tests for the PH quintic fast path (M16–M19).
 */

#include "tether/motion_planner/blend/PHQuinticBlendBuilder.hpp"
#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"
#include "motion_planner/blend/TestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>

using tether::motion::testing::expectVecNear;

namespace {

constexpr double kPi = 3.14159265358979323846;

tether::motion::BoundaryConditions makeBC(const tether::motion::RVec& p,
                                          const tether::motion::RVec& T) {
    tether::motion::BoundaryConditions bc;
    bc.position = p;
    bc.tangent = T.normalized();
    bc.curvature = tether::motion::RVec::zero(p.dim());
    bc.jounce = tether::motion::RVec::zero(p.dim());
    bc.hasJounce = true;
    return bc;
}

} // namespace

TEST(PHQuinticBlend, FourCandidatesProduced) {
    using tether::motion::RVec;
    // Symmetric corner in 2D.
    auto entry = makeBC(RVec{0, 0}, RVec{1, 0});
    auto exit  = makeBC(RVec{1, 1}, RVec{0, 1});
    RVec e1{1, 0}, e2{0, 1};

    auto candidates = tether::motion::PHQuinticBlendBuilder::buildCandidates(
        entry, exit, e1, e2);

    // 4 sign pairs × 2 ω₁ signs = 8 candidates.
    EXPECT_EQ(candidates.size(), 8u);

    // At least one should be non-degenerate.
    int nonDegenerate = 0;
    for (const auto& c : candidates) {
        if (!c.degenerate) ++nonDegenerate;
    }
    EXPECT_GE(nonDegenerate, 1);
}

TEST(PHQuinticBlend, NonDegenerateCandidateMatchesEndpoints) {
    using tether::motion::RVec;
    auto entry = makeBC(RVec{0, 0}, RVec{1, 0});
    auto exit  = makeBC(RVec{2, 2}, RVec{0, 1});
    RVec e1{1, 0}, e2{0, 1};

    auto candidates = tether::motion::PHQuinticBlendBuilder::buildCandidates(
        entry, exit, e1, e2);

    // Find a non-degenerate candidate.
    const tether::motion::PHQuinticBlendBuilder::Result* best = nullptr;
    for (const auto& c : candidates) {
        if (!c.degenerate) { best = &c; break; }
    }
    ASSERT_NE(best, nullptr);

    // Endpoints match.
    expectVecNear(best->curve.evaluate(0.0), entry.position, 1e-9);
    expectVecNear(best->curve.evaluate(1.0), exit.position, 1e-9);

    // Endpoint tangent DIRECTIONS match (magnitudes may differ from 1).
    RVec t0 = best->curve.derivative(0.0, 1);
    RVec t1 = best->curve.derivative(1.0, 1);
    EXPECT_NEAR(t0.normalized()[0], entry.tangent[0], 1e-9);
    EXPECT_NEAR(t1.normalized()[1], exit.tangent[1], 1e-9);
}

TEST(PHQuinticBlend, ArcLengthIsPolynomialAndMatchesQuadrature) {
    using tether::motion::RVec;
    auto entry = makeBC(RVec{0, 0}, RVec{1, 0});
    auto exit  = makeBC(RVec{1, 1}, RVec{0, 1});
    RVec e1{1, 0}, e2{0, 1};

    auto candidates = tether::motion::PHQuinticBlendBuilder::buildCandidates(
        entry, exit, e1, e2);

    const tether::motion::PHQuinticBlendBuilder::Result* best = nullptr;
    for (const auto& c : candidates) {
        if (!c.degenerate) { best = &c; break; }
    }
    ASSERT_NE(best, nullptr);

    // PH arc length (closed form) vs NURBS arc length (quadrature).
    const double phLen = tether::motion::PHQuinticBlendBuilder::arcLength(
        best->ph, 1.0);
    const double nurbsLen = best->curve.length();
    EXPECT_NEAR(phLen, nurbsLen, 1e-8);
}

TEST(PHQuinticBlend, InvertArcLengthRoundTrip) {
    using tether::motion::RVec;
    auto entry = makeBC(RVec{0, 0}, RVec{1, 0});
    auto exit  = makeBC(RVec{1, 1}, RVec{0, 1});
    RVec e1{1, 0}, e2{0, 1};

    auto candidates = tether::motion::PHQuinticBlendBuilder::buildCandidates(
        entry, exit, e1, e2);

    const tether::motion::PHQuinticBlendBuilder::Result* best = nullptr;
    for (const auto& c : candidates) {
        if (!c.degenerate) { best = &c; break; }
    }
    ASSERT_NE(best, nullptr);

    const double totalLen = tether::motion::PHQuinticBlendBuilder::arcLength(
        best->ph, 1.0);
    for (int i = 0; i <= 20; ++i) {
        const double s = totalLen * i / 20.0;
        const double xi = tether::motion::PHQuinticBlendBuilder::invertArcLength(
            best->ph, s);
        const double sCheck = tether::motion::PHQuinticBlendBuilder::arcLength(
            best->ph, xi);
        EXPECT_NEAR(sCheck, s, 1e-8 * std::max(totalLen, 1.0));
    }
}

TEST(PHQuinticBlend, CurvatureFiniteOnInterior) {
    using tether::motion::RVec;
    auto entry = makeBC(RVec{0, 0}, RVec{1, 0});
    auto exit  = makeBC(RVec{1, 1}, RVec{0, 1});
    RVec e1{1, 0}, e2{0, 1};

    auto candidates = tether::motion::PHQuinticBlendBuilder::buildCandidates(
        entry, exit, e1, e2);

    const tether::motion::PHQuinticBlendBuilder::Result* best = nullptr;
    for (const auto& c : candidates) {
        if (!c.degenerate) { best = &c; break; }
    }
    ASSERT_NE(best, nullptr);

    for (int i = 1; i < 20; ++i) {
        const double xi = static_cast<double>(i) / 20.0;
        const double kappa = tether::motion::PHQuinticBlendBuilder::curvature(
            best->ph, xi);
        EXPECT_TRUE(std::isfinite(kappa)) << "xi = " << xi;
    }
}
