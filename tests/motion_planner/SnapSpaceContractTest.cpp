/**
 * @file SnapSpaceContractTest.cpp
 * @brief Property-heavy contract tests for the snap-limited WSS trajectory.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/analytical/AnalyticalJerkLimitedTOPPRA.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;

namespace {

PathAdapter<2, double> makeLine(double length) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{length, 0.0}, 80.0));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.05;
    spec.continuity = tether::motion::Continuity::G2;
    auto result = builder.build(segments, spec);
    return result.success ? std::move(result.path) : PathAdapter<2, double>{};
}

PathAdapter<2, double> makeArc(double radius) {
    MotionSegmentList segments;
    segments.append(MotionSegment::arcCCW(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{2.0 * radius, 0.0},
        Vec<2, double>{radius, 0.0}, 80.0, ArcPlane::XY));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.05;
    spec.continuity = tether::motion::Continuity::G2;
    auto result = builder.build(segments, spec);
    return result.success ? std::move(result.path) : PathAdapter<2, double>{};
}

KinematicLimits<2, double> limits(double v = 80.0, double a = 400.0,
                                  double j = 4000.0, double snap = 40000.0) {
    KinematicLimits<2, double> result;
    result.path.maxPathVelocity = v;
    result.path.maxPathAcceleration = a;
    result.path.maxPathJerk = j;
    result.path.maxPathSnap = snap;
    result.path.maxCentripetalAcceleration = a;
    result.path.jerkLimitEnabled = true;
    for (size_t axis = 0; axis < 2; ++axis) {
        result.axis.maxVelocity[axis] = v;
        result.axis.maxAcceleration[axis] = a;
        result.axis.maxJerk[axis] = j;
    }
    result.axis.jerkLimitEnabled = true;
    return result;
}

struct ArcState {
    double s;
    double v;
    double a;
    double j;
};

ArcState stateAt(const WeightedArc& arc, double tau) {
    if (arc.type == WeightedArcType::SINGULAR) {
        return {arc.s0 + SingularJSeg::ds(arc.v0, arc.a0, arc.j_star, tau),
                SingularJSeg::v(arc.v0, arc.a0, arc.j_star, tau),
                SingularJSeg::a(arc.a0, arc.j_star, tau), arc.j_star};
    }
    return {arc.s0 + SnapSeg::ds(arc.v0, arc.a0, arc.j0, arc.sigma, tau),
            SnapSeg::v(arc.v0, arc.a0, arc.j0, arc.sigma, tau),
            SnapSeg::a(arc.a0, arc.j0, arc.sigma, tau),
            SnapSeg::j(arc.j0, arc.sigma, tau)};
}

double simpson(const std::function<double(double)>& f, double end, int n = 2048) {
    if (end == 0.0) return 0.0;
    if (n % 2 != 0) ++n;
    const double h = end / n;
    double sum = f(0.0) + f(end);
    for (int i = 1; i < n; ++i) sum += (i % 2 == 0 ? 2.0 : 4.0) * f(i * h);
    return sum * h / 3.0;
}

void expectWssContract(const WeightedSwitchingStructure<2>& wss,
                       const PathAdapter<2, double>& path,
                       const KinematicLimits<2, double>& kinematics,
                       double feed, double startVelocity, double endVelocity) {
    ASSERT_FALSE(wss.arcs().empty());
    const auto start = wss.startState();
    const auto end = wss.endState();
    EXPECT_NEAR(start.s, 0.0, 1e-10);
    EXPECT_NEAR(start.v, startVelocity, 1e-9);
    EXPECT_NEAR(start.a, 0.0, 1e-9);
    EXPECT_NEAR(start.j, 0.0, 1e-9);
    EXPECT_NEAR(end.s, path.totalLength(), 1e-7 * (1.0 + path.totalLength()));
    EXPECT_NEAR(end.v, endVelocity, 1e-9);
    EXPECT_NEAR(end.a, 0.0, 1e-9);
    EXPECT_NEAR(end.j, 0.0, 1e-9);

    double previousT = -1.0;
    double previousS = -1.0;
    for (size_t i = 0; i < wss.arcs().size(); ++i) {
        const auto& arc = wss.arcs()[i];
        ASSERT_GT(arc.duration, 0.0);
        const auto actualEnd = stateAt(arc, arc.duration);
        EXPECT_NEAR(actualEnd.s, arc.s1, 1e-9 * (1.0 + arc.s1));
        EXPECT_NEAR(actualEnd.v, arc.v1, 1e-9 * (1.0 + std::abs(arc.v1)));
        EXPECT_NEAR(actualEnd.a, arc.a1, 1e-9 * (1.0 + std::abs(arc.a1)));
        EXPECT_NEAR(actualEnd.j, arc.j1, 1e-9 * (1.0 + std::abs(arc.j1)));
        if (i != 0) {
            const auto& previous = wss.arcs()[i - 1];
            EXPECT_NEAR(arc.s0, previous.s1, 1e-9 * (1.0 + arc.s0));
            EXPECT_NEAR(arc.v0, previous.v1, 1e-9 * (1.0 + std::abs(arc.v0)));
            EXPECT_NEAR(arc.a0, previous.a1, 1e-9 * (1.0 + std::abs(arc.a0)));
            EXPECT_NEAR(arc.j0, previous.j1, 1e-9 * (1.0 + std::abs(arc.j0)));
        }

        for (double fraction : {0.0, 0.125, 0.5, 0.875, 1.0}) {
            const auto state = stateAt(arc, arc.duration * fraction);
            EXPECT_GE(state.v, -1e-9);
            EXPECT_LE(state.v, feed + 1e-7);
            EXPECT_LE(std::abs(state.a), kinematics.path.maxPathAcceleration + 1e-7);
            EXPECT_LE(std::abs(state.j), kinematics.path.maxPathJerk + 1e-7);
            EXPECT_LE(std::abs(arc.sigma), kinematics.path.maxPathSnap + 1e-7);
        }
    }

    for (int i = 0; i <= 200; ++i) {
        const double t = wss.totalTime() * i / 200.0;
        const double s = wss.arcLength(t);
        const double v = wss.pathVelocity(t);
        EXPECT_TRUE(std::isfinite(s));
        EXPECT_TRUE(std::isfinite(v));
        EXPECT_GE(t, previousT);
        EXPECT_GE(s, previousS - 1e-9);
        EXPECT_GE(v, -1e-9);
        previousT = t;
        previousS = s;

        const double tRoundTrip = wss.timeAtArcLength(s);
        EXPECT_NEAR(tRoundTrip, t, 5e-8 * (1.0 + wss.totalTime()));
    }
}

}  // namespace

TEST(SnapSpacePrimitiveContract, SnapPropagationAndInversionAcrossSigns) {
    const std::array cases{
        std::tuple{3.0, 2.0, 1.0, 8.0, 0.4},
        std::tuple{7.0, -1.0, 2.0, -5.0, 0.2},
        std::tuple{1.0, 0.0, 0.0, 12.0, 0.3},
        std::tuple{9.0, 1.0, -2.0, 4.0, 0.25},
    };
    for (const auto& [v0, a0, j0, sigma, duration] : cases) {
        const double distance = SnapSeg::ds(v0, a0, j0, sigma, duration);
        ASSERT_GT(distance, 0.0);
        const double inverted = SnapSeg::tau_for_ds(v0, a0, j0, sigma, distance);
        EXPECT_NEAR(inverted, duration, 1e-10 * (1.0 + duration));
        const double h = 1e-6;
        EXPECT_NEAR((SnapSeg::ds(v0, a0, j0, sigma, duration + h) -
                     SnapSeg::ds(v0, a0, j0, sigma, duration - h)) / (2.0 * h),
                    SnapSeg::v(v0, a0, j0, sigma, duration), 1e-5);
        EXPECT_NEAR((SnapSeg::v(v0, a0, j0, sigma, duration + h) -
                     SnapSeg::v(v0, a0, j0, sigma, duration - h)) / (2.0 * h),
                    SnapSeg::a(a0, j0, sigma, duration), 1e-5);
    }
}

TEST(SnapSpacePrimitiveContract, SingularPropagationAndInversionAcrossSigns) {
    const std::array cases{
        std::tuple{3.0, 2.0, 1.0, 0.4},
        std::tuple{8.0, -1.0, 2.0, 0.2},
        std::tuple{5.0, 0.0, 0.0, 0.5},
    };
    for (const auto& [v0, a0, jerk, duration] : cases) {
        const double distance = SingularJSeg::ds(v0, a0, jerk, duration);
        ASSERT_GT(distance, 0.0);
        const double inverted = SingularJSeg::tau_for_ds(v0, a0, jerk, distance);
        EXPECT_NEAR(inverted, duration, 1e-10 * (1.0 + duration));
    }
}

TEST(SnapSpacePrimitiveContract, InversionRejectsDistancePastTheForwardStop) {
    EXPECT_TRUE(std::isnan(SingularJSeg::tau_for_ds(1.0, -4.0, 0.0, 1.0)));
    EXPECT_TRUE(std::isnan(SnapSeg::tau_for_ds(1.0, -4.0, 0.0, 0.0, 1.0)));
}

TEST(SampledVelocityProfileContract, RepeatedCoordinatesRemainFiniteAndUseRightPoint) {
    SampledVelocityProfile profile;
    profile.addPoint({.arcLength = 0.0, .velocity = 0.0, .time = 0.0});
    profile.addPoint({.arcLength = 0.0, .velocity = 0.0, .time = 2.0});
    profile.addPoint({.arcLength = 5.0, .velocity = 10.0, .time = 3.0});

    EXPECT_DOUBLE_EQ(profile.arcLengthAt(1.0), 0.0);
    EXPECT_DOUBLE_EQ(profile.timeAt(0.0), 2.0);
    EXPECT_TRUE(std::isfinite(profile.arcLengthAt(2.0)));
    EXPECT_TRUE(std::isfinite(profile.timeAt(0.0)));
}

class SnapSpaceTrajectoryContract : public ::testing::TestWithParam<std::tuple<double, double, double, double>> {};

TEST_P(SnapSpaceTrajectoryContract, RestBoundariesContinuityInverseAndConstraints) {
    const auto [length, feed, acceleration, snap] = GetParam();
    auto path = makeLine(length);
    ASSERT_GT(path.totalLength(), 0.0);
    auto kinematics = limits(feed, acceleration, 4000.0, snap);
    ParetoTimeEnergyOptimalVelocityPlanner<2> planner(kinematics, {1.0, 0.01, 0.02});
    auto profile = planner.computeProfile(path, feed, 0.0, 0.0, 301);
    ASSERT_FALSE(profile->points().empty()) << planner.failureReason();
    ASSERT_EQ(profile->derivativeOrder(), ProfileDerivativeOrder::Snap);
    auto wss = planner.weightedSource();
    ASSERT_NE(wss, nullptr);
    expectWssContract(*wss, path, kinematics, feed, 0.0, 0.0);
}

INSTANTIATE_TEST_SUITE_P(
    NormalAndAdverseScales, SnapSpaceTrajectoryContract,
    ::testing::Values(
        std::make_tuple(0.01, 20.0, 50.0, 500.0),
        std::make_tuple(0.1, 40.0, 80.0, 1000.0),
        std::make_tuple(1.0, 50.0, 200.0, 5000.0),
        std::make_tuple(10.0, 80.0, 400.0, 40000.0),
        std::make_tuple(100.0, 80.0, 400.0, 40000.0)));

TEST(SnapSpaceTrajectoryContract, SupportsExactFlyingVelocityBoundaries) {
    auto path = makeLine(100.0);
    auto kinematics = limits();
    ParetoTimeEnergyOptimalVelocityPlanner<2> planner(kinematics, {1.0, 0.01, 0.02});
    auto profile = planner.computeProfile(path, 80.0, 12.0, 18.0, 211);
    ASSERT_FALSE(profile->points().empty()) << planner.failureReason();
    auto wss = planner.weightedSource();
    ASSERT_NE(wss, nullptr);
    expectWssContract(*wss, path, kinematics, 80.0, 12.0, 18.0);
}

TEST(SnapSpaceTrajectoryContract, CurveUsesConservativePathWideEnvelope) {
    auto path = makeArc(10.0);
    ASSERT_GT(path.totalLength(), 0.0);
    auto kinematics = limits(80.0, 100.0, 800.0, 8000.0);
    ParetoTimeEnergyOptimalVelocityPlanner<2> planner(kinematics, {1.0, 0.01, 0.02});
    auto profile = planner.computeProfile(path, 80.0, 0.0, 0.0, 251);
    ASSERT_FALSE(profile->points().empty()) << planner.failureReason();
    auto wss = planner.weightedSource();
    ASSERT_NE(wss, nullptr);
    expectWssContract(*wss, path, kinematics, 80.0, 0.0, 0.0);
    for (int i = 0; i <= 100; ++i) {
        const double t = wss->totalTime() * i / 100.0;
        const double v = wss->pathVelocity(t);
        const double s = wss->arcLength(t);
        EXPECT_LE(v, ConstraintEvaluator<2>(kinematics, 80.0).velocityLimit(s, path) + 1e-7);
    }
}

TEST(SnapSpaceTrajectoryContract, ClosedFormCostMatchesNumericalQuadrature) {
    auto path = makeLine(10.0);
    auto kinematics = limits();
    const CostWeights weights{1.0, 0.03, 0.05};
    ParetoTimeEnergyOptimalVelocityPlanner<2> planner(kinematics, weights);
    planner.computeProfile(path, 80.0, 0.0, 0.0, 301);
    auto wss = planner.weightedSource();
    ASSERT_NE(wss, nullptr) << planner.failureReason();
    const double numericalCost = simpson([&](double t) {
        const double a = wss->pathAcceleration(t);
        const double j = wss->pathJerk(t);
        return weights.w_t + weights.w_a * a * a + weights.w_j * j * j;
    }, wss->totalTime());
    EXPECT_NEAR(wss->costValue(), numericalCost,
                2e-6 * (1.0 + std::abs(numericalCost)));
}

TEST(SnapSpaceTrajectoryContract, RejectsAxisSnapWithoutFourthGeometryDerivative) {
    auto path = makeLine(10.0);
    auto kinematics = limits();
    kinematics.axis.snapLimitEnabled = true;
    ParetoTimeEnergyOptimalVelocityPlanner<2> planner(kinematics);
    auto profile = planner.computeProfile(path, 50.0, 0.0, 0.0, 101);
    EXPECT_TRUE(profile->points().empty());
    EXPECT_NE(planner.failureReason().find("fourth-order"), std::string::npos);
}

TEST(SnapSpaceTrajectoryContract, FailedRequestClearsThePreviousWss) {
    auto path = makeLine(10.0);
    ParetoTimeEnergyOptimalVelocityPlanner<2> planner{limits()};
    ASSERT_FALSE(planner.computeProfile(path, 50.0, 0.0, 0.0, 101)->points().empty());
    ASSERT_NE(planner.weightedSource(), nullptr);

    auto failed = planner.computeProfile(path, 0.0, 0.0, 0.0, 101);
    EXPECT_TRUE(failed->points().empty());
    EXPECT_EQ(planner.weightedSource(), nullptr);
    EXPECT_NE(planner.failureReason().find("Feed rate"), std::string::npos);
}

TEST(SnapSpaceTrajectoryContract, SplitProfilersExposeDifferentDerivativeContracts) {
    auto path = makeLine(30.0);
    auto kinematics = limits();

    AnalyticalTOPPRA<2> secondOrder(kinematics);
    auto secondProfile = secondOrder.computeProfile(path, 50.0, 0.0, 0.0, 101);
    ASSERT_FALSE(secondProfile->points().empty());
    EXPECT_EQ(secondProfile->derivativeOrder(), ProfileDerivativeOrder::Acceleration);
    EXPECT_FALSE(secondProfile->hasJerk());

    AnalyticalJerkLimitedTOPPRA<2> thirdOrder(kinematics);
    auto thirdProfile = thirdOrder.computeProfile(path, 50.0, 0.0, 0.0, 101);
    ASSERT_FALSE(thirdProfile->points().empty());
    EXPECT_EQ(thirdProfile->derivativeOrder(), ProfileDerivativeOrder::Jerk);
    EXPECT_TRUE(thirdProfile->hasJerk());
}

TEST(SnapSpaceTrajectoryContract, NoJerkLimitSelectsOnlyTheSecondOrderContract) {
    auto path = makeLine(30.0);
    auto kinematics = limits();
    kinematics.path.jerkLimitEnabled = false;

    AnalyticalTOPPRA<2> secondOrder(kinematics);
    auto secondProfile = secondOrder.computeProfile(path, 50.0, 0.0, 0.0, 101);
    EXPECT_FALSE(secondProfile->points().empty());
    EXPECT_FALSE(secondProfile->hasJerk());

    AnalyticalJerkLimitedTOPPRA<2> thirdOrder(kinematics);
    auto thirdProfile = thirdOrder.computeProfile(path, 50.0, 0.0, 0.0, 101);
    EXPECT_TRUE(thirdProfile->points().empty());
}
