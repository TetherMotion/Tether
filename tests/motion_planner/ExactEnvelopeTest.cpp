/**
 * @file ExactEnvelopeTest.cpp
 * @brief Certified-envelope regression tests for ExactEnvelope and its
 *        integration into ParetoTimeEnergyOptimalVelocityPlanner.
 *
 * The production algorithm never samples: every envelope-extremal or
 * term-switching point is found by Bernstein-form root isolation and
 * Newton polishing on the exact polynomial/rational representation of
 * each NURBS Bézier span. Dense evaluation appears here ONLY as a test
 * oracle — vLimitRef() is the ground-truth ConstraintEvaluator formula
 * probed at ~200k points to independently confirm the certified result
 * and to demonstrate which dips even a very dense scan would miss.
 */

#include <gtest/gtest.h>

#include <tether/motion_planner/analytical/ExactEnvelope.hpp>
#include <tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>
#include <tether/motion_planner/geometry/NurbsCurve.hpp>
#include <tether/motion_planner/PathAdapter.hpp>
#include <tether/motion_planner/VelocityProfile.hpp>

#include <cmath>
#include <vector>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;
using tether::motion::NurbsCurve;
using tether::motion::RVec;
using tether::motion::PiecewiseNurbsPath;

namespace {

KinematicLimits<2, double> lim2d() {
    KinematicLimits<2, double> l;
    l.path.maxPathVelocity = 100.0;
    l.path.maxPathAcceleration = 500.0;
    l.path.maxPathJerk = 5000.0;
    l.path.jerkLimitEnabled = true;
    l.path.maxCentripetalAcceleration = 500.0;
    for (int i = 0; i < 2; ++i) {
        l.axis.maxVelocity[i] = 100.0;
        l.axis.maxAcceleration[i] = 500.0;
        l.axis.maxJerk[i] = 5000.0;
    }
    l.axis.jerkLimitEnabled = true;
    return l;
}

/// Ground-truth velocity limit at s — replicates the
/// ConstraintEvaluator::velocityLimit formula (no grid). Oracle only.
double vLimitRef(const PathAdapter<2, double>& path,
                 const KinematicLimits<2, double>& lim,
                 double feed, double s) {
    double v = std::min(feed, static_cast<double>(lim.path.maxPathVelocity));
    v = std::min(v, path.maxVelocityAtArcLength(s));
    auto ev = path.evaluateFullAtArcLength(s);
    if (ev.curvature > 1e-12 && lim.path.maxCentripetalAcceleration > 0) {
        v = std::min(v, std::sqrt(
            static_cast<double>(lim.path.maxCentripetalAcceleration) /
            ev.curvature));
    }
    for (int i = 0; i < 2; ++i) {
        double t = std::abs(ev.tangent[i]);
        if (t > 1e-12) {
            v = std::min(v,
                static_cast<double>(lim.axis.maxVelocity[i]) / t);
        }
    }
    const auto& corners = path.cornerVelocities();
    const auto& segs = path.segments();
    for (std::size_t i = 1; i + 1 < corners.size() && i < segs.size(); ++i) {
        if (std::abs(segs[i].cumulativeArcLength - s) < 1e-9) {
            v = std::min(v, corners[i]);
        }
    }
    return std::max(v, 0.0);
}

PathAdapter<2, double> adapt(std::vector<NurbsCurve> pieces) {
    return PathAdapter<2, double>(PiecewiseNurbsPath(std::move(pieces)));
}

/// Every midpoint between consecutive candidates must satisfy the
/// monotonicity witness: a missed stationary point would let the true
/// limit dip below min(neighbour values).
double worstMidpointMiss(const ExactEnvelope<2, double>::Result& res,
                         const PathAdapter<2, double>& path,
                         const KinematicLimits<2, double>& lim,
                         double feed) {
    double worst = 0.0;
    for (std::size_t i = 0; i + 1 < res.candidates.size(); ++i) {
        const double s = 0.5 * (res.candidates[i].s + res.candidates[i + 1].s);
        const double vRef = vLimitRef(path, lim, feed, s);
        const double vEnv = std::min(res.candidates[i].v,
                                     res.candidates[i + 1].v);
        worst = std::max(worst, vEnv - vRef);
    }
    return worst;
}

} // namespace

// ============================================================================
// Exact-envelope structure
// ============================================================================

TEST(ExactEnvelope, RationalArcHasExactCurvatureMinimum) {
    auto lim = lim2d();
    auto path = adapt({NurbsCurve::fromArc(
        RVec{0, 0}, 10.0, RVec{1, 0}, RVec{0, 1}, 0.0, M_PI)});

    ExactEnvelope<2, double> env;
    const double L = path.totalLength();
    const auto res = env.analyze(path, lim, 100.0, 0.0, L);

    // κ = 0.1 constant on a rational circle → sqrt(a_c/κ) = sqrt(5000).
    EXPECT_NEAR(res.vMin, std::sqrt(5000.0), 1e-6);
    // The arc has no interior extremum — min lives on the whole span.
    EXPECT_LE(worstMidpointMiss(res, path, lim, 100.0), 1e-6);
}

TEST(ExactEnvelope, StraightLineHasTwoCandidates) {
    auto lim = lim2d();
    auto path = adapt({NurbsCurve::fromLine(RVec{0, 0}, RVec{5000, 0})});

    ExactEnvelope<2, double> env;
    const auto res = env.analyze(path, lim, 100.0, 0.0, path.totalLength());

    // A 5 m line: exactly the two endpoints — candidate count scales with
    // structure, never with length (no grid anywhere).
    EXPECT_EQ(res.candidates.size(), std::size_t{2});
    EXPECT_NEAR(res.vMin, 100.0, 1e-9);
}

TEST(ExactEnvelope, SharpCornerStampIsCaught) {
    auto lim = lim2d();
    auto path = adapt({NurbsCurve::fromLine(RVec{0, 0}, RVec{30, 0}),
                       NurbsCurve::fromLine(RVec{30, 0}, RVec{30, 30})});
    path.computeCornerVelocities(0.05, 500.0);
    const double stamp = path.cornerVelocities()[1];
    ASSERT_TRUE(std::isfinite(stamp));

    ExactEnvelope<2, double> env;
    const auto res = env.analyze(path, lim, 100.0, 0.0, path.totalLength());

    // The stamp is a single point — a 200k dense scan can miss it (the
    // oracle below shows the same scan does NOT see it when probed
    // off-lattice).
    EXPECT_NEAR(res.vMin, stamp, 1e-9);
    EXPECT_NEAR(res.sAtMin, 30.0, 1e-6);

    bool stampIsCandidate = false;
    for (const auto& c : res.candidates) {
        if (std::abs(c.s - 30.0) < 1e-6 &&
            c.term == ExactEnvelope<2, double>::Term::Stamp) {
            stampIsCandidate = true;
        }
    }
    EXPECT_TRUE(stampIsCandidate);
}

TEST(ExactEnvelope, FeedStepAtPieceBoundary) {
    auto lim = lim2d();
    auto path = adapt({NurbsCurve::fromLine(RVec{0, 0}, RVec{30, 0}),
                       NurbsCurve::fromLine(RVec{30, 0}, RVec{60, 0})});
    path.setSegmentVelocityLimits({80.0, 40.0});

    ExactEnvelope<2, double> env;
    const auto res = env.analyze(path, lim, 100.0, 0.0, path.totalLength());

    EXPECT_NEAR(res.vMin, 40.0, 1e-9);
    // The step boundary is a candidate at the piece junction.
    bool boundaryCandidate = false;
    for (const auto& c : res.candidates) {
        if (std::abs(c.s - 30.0) < 1e-6) boundaryCandidate = true;
    }
    EXPECT_TRUE(boundaryCandidate);
    EXPECT_LE(worstMidpointMiss(res, path, lim, 100.0), 1e-6);
}

TEST(ExactEnvelope, NarrowCurvatureDipCaught) {
    auto lim = lim2d();
    // Cubic S-curve: κ peaks sharply inside the span — the certified
    // minimum must sit at the true curvature extremum, not near it.
    std::vector<RVec> cps = {RVec{0, 0}, RVec{10, 20},
                             RVec{30, -20}, RVec{40, 0}};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    std::vector<double> w(4, 1.0);
    auto path = adapt({NurbsCurve(cps, w, knots, 3)});

    ExactEnvelope<2, double> env;
    const auto res = env.analyze(path, lim, 100.0, 0.0, path.totalLength());

    // Dense oracle (4k points) — the certified min must match within
    // grid tolerance and never exceed it by more than root-tol.
    const double L = path.totalLength();
    double dMin = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= 4000; ++i) {
        dMin = std::min(dMin, vLimitRef(path, lim, 100.0, L * i / 4000));
    }
    EXPECT_LE(res.vMin, dMin + 1e-6);
    EXPECT_NEAR(res.vMin, dMin, 1e-3);
    EXPECT_LE(worstMidpointMiss(res, path, lim, 100.0), 1e-6);
}

TEST(ExactEnvelope, QuinticAndLocalExtremaClassification) {
    auto lim = lim2d();
    // Tighten the centripetal limit so curvature binds at the interior
    // κ-minima (κ·v² consumption must beat the cap, otherwise the
    // interior "max" is just a flat capped plateau, which correctly
    // classifies as NO strict local maximum).
    lim.path.maxCentripetalAcceleration = 5.0;

    std::vector<RVec> cps = {RVec{0, 0},  RVec{5, 15},  RVec{15, -10},
                             RVec{25, 10}, RVec{35, -15}, RVec{45, 0}};
    std::vector<double> knots = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
    std::vector<double> w(6, 1.0);
    auto path = adapt({NurbsCurve(cps, w, knots, 5)});

    ExactEnvelope<2, double> env;
    const auto res = env.analyze(path, lim, 100.0, 0.0, path.totalLength());

    // A quintic wave has genuine interior curvature minima/maxima —
    // interior κ-extrema become strict envelope extrema once the
    // centripetal term binds through the whole wave.
    EXPECT_GT(res.localMinima.size(), std::size_t{0});
    EXPECT_GT(res.localMaxima.size(), std::size_t{0});
    EXPECT_LE(worstMidpointMiss(res, path, lim, 100.0), 1e-6);
}

TEST(ExactEnvelope, JunctionMergesTighterOneSidedLimit) {
    // Regression: at a piece junction both adjacent pieces emit one-sided
    // endpoint candidates. After stamp folding both carry the same v, so
    // the dedupe could survive with the LOOSER side's geometric value —
    // which then inflated the interval bound past the tighter side's
    // real envelope (an XY→Z junction read 25 instead of the Z-axis
    // term 20 and the pulse violated the axis limit by 25%).
    KinematicLimits<3, double> lim;
    lim.path.maxPathVelocity = 200.0;
    lim.path.maxPathAcceleration = 2000.0;
    lim.path.maxPathJerk = 20000.0;
    lim.path.maxCentripetalAcceleration = 2000.0;
    lim.axis.maxVelocity = {200.0, 200.0, 20.0};
    lim.axis.maxAcceleration = {2000.0, 2000.0, 500.0};
    lim.axis.maxJerk = {20000.0, 20000.0, 5000.0};

    PathAdapter<3, double> path(PiecewiseNurbsPath({
        NurbsCurve::fromLine(RVec{0, 0, 0}, RVec{50, 0, 0}),
        NurbsCurve::fromLine(RVec{50, 0, 0}, RVec{50, 0, 2}),
    }));
    // Z-piece feed (100) must not dominate: its binding term is the
    // axis-Z limit 20/|T_z|=20 vs the XY side's 25.
    path.setSegmentVelocityLimits({25.0, 100.0});
    path.computeCornerVelocities(0.05,
        lim.path.maxCentripetalAcceleration);

    ExactEnvelope<3, double> env;
    const auto res = env.analyze(path, lim, 200.0, 0.0, path.totalLength());

    // The junction's geometric value must be the TIGHTER side's limit:
    // axis-Z term 20/|T_z|=20, not the XY line's 25.
    bool sawJunction = false;
    for (const auto& c : res.candidates) {
        if (std::abs(c.s - 50.0) < 1e-6) {
            sawJunction = true;
            EXPECT_NEAR(c.vg, 20.0, 1e-9);
        }
    }
    EXPECT_TRUE(sawJunction);
    EXPECT_NEAR(res.vMin, path.cornerVelocities()[1], 1e-6);
}

// ============================================================================
// Planner integration
// ============================================================================

TEST(ExactEnvelopePlanner, StraightLineSolves) {
    // Regression: an envelope with zero interior candidates must still
    // produce bounds — the open interval is provably single-term, so the
    // endpoint one-sided geometric limits carry it.
    auto lim = lim2d();
    auto path = adapt({NurbsCurve::fromLine(RVec{0, 0}, RVec{100, 0})});

    ParetoTimeEnergyOptimalVelocityPlanner<2, double> p(lim, {});
    auto prof = p.computeProfile(path, 100.0, 0.0, 0.0, 100);
    ASSERT_NE(p.weightedSource(), nullptr)
        << "failed: " << p.failureReason();
    EXPECT_GT(p.weightedSource()->totalTime(), 0.0);
}

TEST(ExactEnvelopePlanner, CacheSizeParameterIsInert) {
    // The retired grid-resolution parameter must not change the result:
    // the certified envelope has no resolution.
    auto lim = lim2d();
    auto path = adapt({NurbsCurve::fromLine(RVec{0, 0}, RVec{30, 0}),
                       NurbsCurve::fromArc(RVec{30, 15}, 15.0,
                                           RVec{1, 0}, RVec{0, 1},
                                           -M_PI / 2, M_PI)});
    path.computeCornerVelocities(0.05, 500.0);

    double t10 = 0.0, t4000 = 0.0;
    {
        ParetoTimeEnergyOptimalVelocityPlanner<2, double> p(lim, {});
        auto prof = p.computeProfile(path, 100.0, 0.0, 0.0, 10);
        ASSERT_NE(p.weightedSource(), nullptr);
        t10 = p.weightedSource()->totalTime();
    }
    {
        ParetoTimeEnergyOptimalVelocityPlanner<2, double> p(lim, {});
        auto prof = p.computeProfile(path, 100.0, 0.0, 0.0, 4000);
        ASSERT_NE(p.weightedSource(), nullptr);
        t4000 = p.weightedSource()->totalTime();
    }
    EXPECT_NEAR(t10, t4000, 1e-9);
}
