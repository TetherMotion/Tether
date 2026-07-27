/**
 * @file CornerAnalysisTests.cpp
 * @brief Tests for CornerAnalyzer (M13 plane basis, classification).
 */

#include "tether/motion_planner/blend/CornerAnalysis.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-12;

// Two lines meeting at the origin with a known turning angle.
// Line A: from (-d, 0) to (0, 0) — tangent at end = (1, 0).
// Line B: from (0, 0) to (d·cosθ, d·sinθ) — tangent at start = (cosθ, sinθ).
struct CornerSetup {
    double angle;
    double len;
    tether::motion::NurbsCurve in;
    tether::motion::NurbsCurve out;
    CornerSetup(double angleRad, double length)
        : angle(angleRad), len(length),
          in(tether::motion::NurbsCurve::fromLine(
              tether::motion::RVec{-length, 0.0}, tether::motion::RVec{0.0, 0.0})),
          out(tether::motion::NurbsCurve::fromLine(
              tether::motion::RVec{0.0, 0.0},
              tether::motion::RVec{length * std::cos(angleRad),
                           length * std::sin(angleRad)})) {}
};

} // namespace

TEST(CornerAnalysis, RightAngleClassificationAndBasis) {
    CornerSetup cs(kPi / 2.0, 10.0);
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    tether::motion::CornerAnalysis ca = analyzer.analyze(cs.in, cs.out);

    EXPECT_EQ(ca.kind, tether::motion::CornerKind::Corner);
    EXPECT_NEAR(ca.angleRad, kPi / 2.0, kTol);

    // e₁ = t_in = (1, 0)
    EXPECT_NEAR(ca.planeE1[0], 1.0, kTol);
    EXPECT_NEAR(ca.planeE1[1], 0.0, kTol);
    // e₂ = normalize(t_out − (t_out·e₁)e₁) = normalize((0,1) − 0) = (0,1)
    EXPECT_NEAR(ca.planeE2[0], 0.0, kTol);
    EXPECT_NEAR(ca.planeE2[1], 1.0, kTol);

    // Orthonormality.
    EXPECT_NEAR(ca.planeE1.dot(ca.planeE2), 0.0, kTol);
    EXPECT_NEAR(ca.planeE1.norm(), 1.0, kTol);
    EXPECT_NEAR(ca.planeE2.norm(), 1.0, kTol);
}

TEST(CornerAnalysis, StraightCornerClassified) {
    // Nearly collinear: angle 0.001 rad < minAngle 0.01.
    CornerSetup cs(0.001, 10.0);
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    tether::motion::CornerAnalysis ca = analyzer.analyze(cs.in, cs.out);
    EXPECT_EQ(ca.kind, tether::motion::CornerKind::Straight);
}

TEST(CornerAnalysis, CuspClassified) {
    // Near-reversal: angle 3.14 rad > maxAngle 3.13.
    CornerSetup cs(3.14, 10.0);
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    tether::motion::CornerAnalysis ca = analyzer.analyze(cs.in, cs.out);
    EXPECT_EQ(ca.kind, tether::motion::CornerKind::Cusp);
}

TEST(CornerAnalysis, ThreeDimensionalCornerBasisIsPlanar) {
    // 3D corner: in along x, out at 45° in the xy-plane.
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    NurbsCurve in = NurbsCurve::fromLine(RVec{-5, 0, 0}, RVec{0, 0, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 0, 0},
                                          RVec{5, 5, 0});
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    tether::motion::CornerAnalysis ca = analyzer.analyze(in, out);

    EXPECT_EQ(ca.kind, tether::motion::CornerKind::Corner);
    EXPECT_NEAR(ca.angleRad, kPi / 4.0, kTol);
    // e₁ = (1,0,0), e₂ = normalize((1,1,0)/√2 − (1/√2)(1,0,0)) = (0,1,0)
    EXPECT_NEAR(ca.planeE1[0], 1.0, kTol);
    EXPECT_NEAR(ca.planeE2[1], 1.0, kTol);
    EXPECT_NEAR(ca.planeE2[2], 0.0, kTol);
    // The z-component of both basis vectors is zero (planar corner).
    EXPECT_NEAR(ca.planeE1[2], 0.0, kTol);
}

TEST(CornerAnalysis, DimensionMismatchThrows) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    NurbsCurve in = NurbsCurve::fromLine(RVec{0, 0}, RVec{1, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 0, 0}, RVec{1, 0, 0});
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    EXPECT_THROW(analyzer.analyze(in, out), std::invalid_argument);
}

TEST(CornerAnalysis, DisconnectedJunctionThrows) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // End of `in` is (0,0); start of `out` is (1,0) — not connected.
    NurbsCurve in = NurbsCurve::fromLine(RVec{-1, 0}, RVec{0, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{1, 0}, RVec{2, 0});
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    EXPECT_THROW(analyzer.analyze(in, out), std::invalid_argument);
}

TEST(CornerAnalysis, ArcCornerHasNonzeroCurvature) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // In: line into origin. Out: quarter arc from origin.
    // Actually NURBS arcs are centered elsewhere; build a simpler case:
    // two arcs meeting at a point. Use lines for the in piece and an arc
    // for the out piece, sharing the junction.
    NurbsCurve in = NurbsCurve::fromLine(RVec{-5, 0}, RVec{0, 0});
    // Arc centered at (0, -2), radius 2, from angle 90° sweeping -90°:
    // start = (0, 0), end = (2, -2)... let's just use a quarter arc
    // starting at (0,0).
    NurbsCurve out = NurbsCurve::fromArc(RVec{0, -2}, 2.0,
        RVec{1, 0}, RVec{0, 1}, kPi / 2.0, -kPi / 2.0);
    // out.startPoint() should be (0, 0).
    EXPECT_NEAR(out.startPoint()[0], 0.0, 1e-10);
    EXPECT_NEAR(out.startPoint()[1], 0.0, 1e-10);

    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    tether::motion::CornerAnalysis ca = analyzer.analyze(in, out);
    // The arc has curvature 1/R = 0.5 at its start.
    EXPECT_NEAR(ca.curvatureOut.norm(), 0.5, 1e-9);
    // The line has zero curvature.
    EXPECT_NEAR(ca.curvatureIn.norm(), 0.0, 1e-12);
}
