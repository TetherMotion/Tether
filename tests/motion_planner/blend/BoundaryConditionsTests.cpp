/**
 * @file BoundaryConditionsTests.cpp
 * @brief Tests for boundaryAt (G.18)–(G.21) extraction.
 */

#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-10;

} // namespace

TEST(BoundaryConditions, LineStartAndEnd) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    NurbsCurve line = NurbsCurve::fromLine(RVec{0, 0}, RVec{10, 0});

    // Start (s=0, atEnd=false): position (0,0), tangent (1,0), κ=0, j=0.
    auto bc0 = tether::motion::boundaryAt(line, 0.0, false);
    EXPECT_NEAR(bc0.position[0], 0.0, kTol);
    EXPECT_NEAR(bc0.tangent[0], 1.0, kTol);
    EXPECT_NEAR(bc0.curvature.norm(), 0.0, kTol);
    EXPECT_TRUE(bc0.hasJounce);

    // End (s=10, atEnd=true): position (10,0), tangent (1,0), κ=0.
    auto bc1 = tether::motion::boundaryAt(line, 10.0, true);
    EXPECT_NEAR(bc1.position[0], 10.0, kTol);
    EXPECT_NEAR(bc1.tangent[0], 1.0, kTol);
    EXPECT_NEAR(bc1.curvature.norm(), 0.0, kTol);

    // Midpoint.
    auto bcm = tether::motion::boundaryAt(line, 5.0, false);
    EXPECT_NEAR(bcm.position[0], 5.0, kTol);
}

TEST(BoundaryConditions, ArcCurvatureMatchesClosedForm) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    const double R = 2.0;
    // Quarter arc from (R, 0) to (0, R), centered at origin.
    NurbsCurve arc = NurbsCurve::fromArc(RVec{0, 0}, R,
        RVec{1, 0}, RVec{0, 1}, 0.0, kPi / 2.0);

    // At the start (s=0): position (R, 0), tangent (0, 1), |κ| = 1/R.
    auto bc = tether::motion::boundaryAt(arc, 0.0, false);
    EXPECT_NEAR(bc.position[0], R, kTol);
    EXPECT_NEAR(bc.position[1], 0.0, kTol);
    EXPECT_NEAR(bc.tangent[0], 0.0, kTol);
    EXPECT_NEAR(bc.tangent[1], 1.0, kTol);
    EXPECT_NEAR(bc.curvature.norm(), 1.0 / R, 1e-9);
    // κ⃗ points at the center: κ⃗ = -(1/R²)·p.
    EXPECT_NEAR(bc.curvature[0], -1.0 / (R * R) * R, 1e-9);
    EXPECT_NEAR(bc.curvature[1], 0.0, 1e-9);
}

TEST(BoundaryConditions, OutOfRangeThrows) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    NurbsCurve line = NurbsCurve::fromLine(RVec{0, 0}, RVec{1, 0});
    EXPECT_THROW(tether::motion::boundaryAt(line, -0.1, false),
                 std::invalid_argument);
    EXPECT_THROW(tether::motion::boundaryAt(line, 2.0, true),
                 std::invalid_argument);
}
