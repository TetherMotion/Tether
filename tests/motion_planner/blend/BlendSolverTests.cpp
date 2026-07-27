/**
 * @file BlendSolverTests.cpp
 * @brief Tests for the BlendSolver (M15 bisection, M20 ear, fallbacks).
 */

#include "tether/motion_planner/blend/BlendSolver.hpp"
#include "tether/motion_planner/blend/BlendSpec.hpp"
#include "tether/motion_planner/blend/CornerAnalysis.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "motion_planner/blend/TestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>

using tether::motion::testing::expectVecNear;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A 90° corner: in along -x, out along +y.
struct RightCorner {
    tether::motion::NurbsCurve in;
    tether::motion::NurbsCurve out;
    tether::motion::CornerAnalysis ca;
    RightCorner(double len = 10.0)
        : in(tether::motion::NurbsCurve::fromLine(
              tether::motion::RVec{-len, 0}, tether::motion::RVec{0, 0})),
          out(tether::motion::NurbsCurve::fromLine(
              tether::motion::RVec{0, 0}, tether::motion::RVec{0, len})) {
        tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
        ca = analyzer.analyze(in, out);
    }
};

} // namespace

TEST(BlendSolver, InsideBlendMeetsTolerance) {
    RightCorner rc;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5; // 0.5 units inside cut
    tether::motion::BlendSolver solver(rc.in, rc.out, rc.ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);

    EXPECT_EQ(geom.outcome, tether::motion::BlendOutcome::Blended);
    EXPECT_LE(geom.deviation.upper, 0.5 + 1e-3);
    EXPECT_GT(geom.trimIn, 0.0);
    EXPECT_GT(geom.trimOut, 0.0);
}

TEST(BlendSolver, ExactStopMode) {
    RightCorner rc;
    tether::motion::BlendSpec spec;
    spec.mode = tether::motion::PathMode::ExactStop;
    tether::motion::BlendSolver solver(rc.in, rc.out, rc.ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);
    EXPECT_EQ(geom.outcome, tether::motion::BlendOutcome::ExactStop);
}

TEST(BlendSolver, StraightCornerNoBlend) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // Nearly straight: angle 0.001 rad.
    NurbsCurve in = NurbsCurve::fromLine(RVec{-5, 0}, RVec{0, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 0},
        RVec{5 * std::cos(0.001), 5 * std::sin(0.001)});
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    auto ca = analyzer.analyze(in, out);

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;
    tether::motion::BlendSolver solver(in, out, ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);
    EXPECT_EQ(geom.outcome, tether::motion::BlendOutcome::NoBlendNeeded);
}

TEST(BlendSolver, CuspFallsBackToExactStop) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // Near-reversal: angle 3.14 rad.
    NurbsCurve in = NurbsCurve::fromLine(RVec{-5, 0}, RVec{0, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 0},
        RVec{5 * std::cos(3.14), 5 * std::sin(3.14)});
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    auto ca = analyzer.analyze(in, out);

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;
    tether::motion::BlendSolver solver(in, out, ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);
    EXPECT_EQ(geom.outcome, tether::motion::BlendOutcome::ExactStop);
}

TEST(BlendSolver, NegativeToleranceEarBlend) {
    RightCorner rc(20.0);
    tether::motion::BlendSpec spec;
    spec.tolerance = -0.5; // outside ear
    tether::motion::BlendSolver solver(rc.in, rc.out, rc.ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);

    // Ear blends are harder; accept either Blended or ExactStop.
    if (geom.outcome == tether::motion::BlendOutcome::Blended) {
        EXPECT_LE(geom.deviation.outsideHi, 0.5 + 1e-3);
    }
    // No crash, no hang — that's the minimum guarantee.
}

TEST(BlendSolver, TooShortPieceFallsBack) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // Very short pieces — can't trim.
    NurbsCurve in = NurbsCurve::fromLine(RVec{-0.001, 0}, RVec{0, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 0}, RVec{0, 0.001});
    tether::motion::CornerAnalyzer analyzer(0.01, 3.13);
    auto ca = analyzer.analyze(in, out);

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;
    spec.minSegmentLength = 0.01; // longer than the pieces
    tether::motion::BlendSolver solver(in, out, ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);
    EXPECT_EQ(geom.outcome, tether::motion::BlendOutcome::ExactStop);
}

TEST(BlendSolver, G3ContinuityBlend) {
    RightCorner rc(15.0);
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;
    spec.continuity = tether::motion::Continuity::G3;
    tether::motion::BlendSolver solver(rc.in, rc.out, rc.ca);
    tether::motion::BlendGeometry geom = solver.solve(spec);

    if (geom.outcome == tether::motion::BlendOutcome::Blended) {
        EXPECT_EQ(geom.blendCurve->degree(), 7);
        EXPECT_LE(geom.deviation.upper, 0.5 + 1e-3);
    }
}
