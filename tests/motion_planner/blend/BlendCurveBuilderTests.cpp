/**
 * @file BlendCurveBuilderTests.cpp
 * @brief T1 golden-value tests for the (M11)/(M12) control-point formulas.
 *
 * Verifies that the constructed Bézier matches the imposed boundary
 * conditions exactly (to machine precision) at both endpoints, for both
 * the quintic (G²) and septic (G³) builders, in 2D and 5D.
 */

#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/blend/BlendCurveBuilder.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"
#include "motion_planner/blend/TestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>

using tether::motion::testing::expectVecNear;

namespace {

constexpr double kTol = 1e-12;

// Build boundary conditions for a synthetic corner.
tether::motion::BoundaryConditions makeBC(const tether::motion::RVec& p,
                                          const tether::motion::RVec& T,
                                          const tether::motion::RVec& k,
                                          const tether::motion::RVec& j) {
    tether::motion::BoundaryConditions bc;
    bc.position = p;
    bc.tangent = T.normalized();
    bc.curvature = k;
    bc.jounce = j;
    bc.hasJounce = true;
    return bc;
}

} // namespace

// ============================================================================
// Quintic (G²) — (M11)
// ============================================================================

TEST(BlendCurveBuilder, QuinticMatchesBoundaryPositions) {
    using tether::motion::RVec;
    const RVec pA{1.0, 2.0};
    const RVec pB{4.0, 6.0};
    const RVec TA{1.0, 0.0};
    const RVec TB{0.0, 1.0};
    const RVec kA{0.0, 0.0};
    const RVec kB{0.0, 0.0};
    auto entry = makeBC(pA, TA, kA, RVec{0, 0});
    auto exit  = makeBC(pB, TB, kB, RVec{0, 0});

    const double alpha1 = 2.0, beta1 = 3.0;
    tether::motion::NurbsCurve c =
        tether::motion::BlendCurveBuilder::buildQuintic(entry, exit,
                                                        alpha1, beta1);

    EXPECT_EQ(c.degree(), 5);
    EXPECT_EQ(c.numControlPoints(), 6u);

    // B(0) = p_A, B(1) = p_B exactly.
    expectVecNear(c.evaluate(0.0), pA, kTol);
    expectVecNear(c.evaluate(1.0), pB, kTol);
}

TEST(BlendCurveBuilder, QuinticMatchesTangentsAndCurvatures) {
    using tether::motion::RVec;
    const RVec pA{0.0, 0.0};
    const RVec pB{5.0, 0.0};
    const RVec TA{1.0, 0.0};       // unit
    const RVec TB{1.0, 0.0};       // unit (straight blend)
    const RVec kA{0.0, 2.0};
    const RVec kB{0.0, -1.0};
    auto entry = makeBC(pA, TA, kA, RVec{0, 0});
    auto exit  = makeBC(pB, TB, kB, RVec{0, 0});

    const double alpha1 = 1.5, beta1 = 2.5;
    tether::motion::NurbsCurve c =
        tether::motion::BlendCurveBuilder::buildQuintic(entry, exit,
                                                        alpha1, beta1);

    // B'(0) = α₁ T_A, B'(1) = β₁ T_B.
    expectVecNear(c.derivative(0.0, 1), TA * alpha1, kTol);
    expectVecNear(c.derivative(1.0, 1), TB * beta1, kTol);
    // B''(0) = α₁² κ⃗_A, B''(1) = β₁² κ⃗_B.
    expectVecNear(c.derivative(0.0, 2), kA * (alpha1 * alpha1), kTol);
    expectVecNear(c.derivative(1.0, 2), kB * (beta1 * beta1), kTol);
}

TEST(BlendCurveBuilder, QuinticFiveDimensional) {
    using tether::motion::RVec;
    const RVec pA{0, 0, 0, 0, 0};
    const RVec pB{1, 2, 3, 4, 5};
    const RVec TA{1, 0, 0, 0, 0};
    const RVec TB{0, 1, 0, 0, 0};
    const RVec kA{0, 0, 1, 0, 0};
    const RVec kB{0, 0, 0, 1, 0};
    auto entry = makeBC(pA, TA, kA, RVec{0, 0, 0, 0, 0});
    auto exit  = makeBC(pB, TB, kB, RVec{0, 0, 0, 0, 0});

    tether::motion::NurbsCurve c =
        tether::motion::BlendCurveBuilder::buildQuintic(entry, exit, 1.0, 1.0);

    EXPECT_EQ(c.dim(), 5u);
    expectVecNear(c.evaluate(0.0), pA, kTol);
    expectVecNear(c.evaluate(1.0), pB, kTol);
    expectVecNear(c.derivative(0.0, 2), kA, kTol);
    expectVecNear(c.derivative(1.0, 2), kB, kTol);
}

TEST(BlendCurveBuilder, QuinticRejectsNonPositiveSpeeds) {
    using tether::motion::RVec;
    auto entry = makeBC(RVec{0, 0}, RVec{1, 0}, RVec{0, 0}, RVec{0, 0});
    auto exit  = makeBC(RVec{1, 0}, RVec{1, 0}, RVec{0, 0}, RVec{0, 0});
    EXPECT_THROW(tether::motion::BlendCurveBuilder::buildQuintic(entry, exit, 0.0, 1.0),
                 std::invalid_argument);
    EXPECT_THROW(tether::motion::BlendCurveBuilder::buildQuintic(entry, exit, 1.0, -1.0),
                 std::invalid_argument);
}

// ============================================================================
// Septic (G³) — (M12)
// ============================================================================

TEST(BlendCurveBuilder, SepticMatchesAllBoundaryData) {
    using tether::motion::RVec;
    const RVec pA{0.0, 0.0};
    const RVec pB{4.0, 2.0};
    const RVec TA{1.0, 0.0};
    const RVec TB{0.0, 1.0};
    const RVec kA{0.0, 0.5};
    const RVec kB{-0.3, 0.0};
    const RVec jA{0.1, -0.2};
    const RVec jB{0.0, 0.4};
    auto entry = makeBC(pA, TA, kA, jA);
    auto exit  = makeBC(pB, TB, kB, jB);

    const double alpha1 = 1.3, beta1 = 2.1;
    tether::motion::NurbsCurve c =
        tether::motion::BlendCurveBuilder::buildSeptic(entry, exit,
                                                       alpha1, beta1);

    EXPECT_EQ(c.degree(), 7);
    EXPECT_EQ(c.numControlPoints(), 8u);

    expectVecNear(c.evaluate(0.0), pA, kTol);
    expectVecNear(c.evaluate(1.0), pB, kTol);
    expectVecNear(c.derivative(0.0, 1), TA * alpha1, kTol);
    expectVecNear(c.derivative(1.0, 1), TB * beta1, kTol);
    expectVecNear(c.derivative(0.0, 2), kA * (alpha1 * alpha1), kTol);
    expectVecNear(c.derivative(1.0, 2), kB * (beta1 * beta1), kTol);
    // B'''(0) = α₁³ j⃗_A, B'''(1) = β₁³ j⃗_B  (note: the formula has a
    // MINUS on the exit side, but the resulting B'''(1) equals +β₁³ j⃗_B
    // because of the alternating signs in the septic endpoint identity).
    expectVecNear(c.derivative(0.0, 3), jA * (alpha1 * alpha1 * alpha1), kTol);
    expectVecNear(c.derivative(1.0, 3), jB * (beta1 * beta1 * beta1), kTol);
}

TEST(BlendCurveBuilder, SepticRequiresJounce) {
    using tether::motion::RVec;
    tether::motion::BoundaryConditions entry =
        makeBC(RVec{0, 0}, RVec{1, 0}, RVec{0, 0}, RVec{0, 0});
    tether::motion::BoundaryConditions exit =
        makeBC(RVec{1, 0}, RVec{1, 0}, RVec{0, 0}, RVec{0, 0});
    exit.hasJounce = false;
    EXPECT_THROW(tether::motion::BlendCurveBuilder::buildSeptic(entry, exit, 1.0, 1.0),
                 std::invalid_argument);
}
