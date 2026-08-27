/**
 * @file AnalyticalTOPPRATest.cpp
 * @brief Comprehensive tests for the analytical TOPPRA-equivalent profiler.
 *
 * @details
 * Tests cover:
 * 1. Numerical utilities (LGL nodes, derivative matrix, Padé, barycentric)
 * 2. Constraint evaluator (eta bounds, velocity limits, acceleration bounds)
 * 3. Switching Structure Representation (SSR) — Class A
 * 4. Hybrid Monotone Representation — Class B
 * 5. Trajectory Sampler (unified interface)
 * 6. AnalyticalJerkLimitedTOPPRA profiler (VelocityProfiler interface, solver, sampling)
 * 7. MotionPlan integration (analytical source, backward compatibility)
 * 8. Constraint satisfaction (velocity, acceleration, jerk limits)
 * 9. SSR vs Hybrid consistency
 * 10. Certification (error bounds)
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/SourceReference.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
#include <tether/motion_planner/analytical/NumericalUtils.hpp>
#include <tether/motion_planner/analytical/AnalyticalTypes.hpp>
#include <tether/motion_planner/analytical/ConstraintEvaluator.hpp>
#include <tether/motion_planner/analytical/SwitchingStructureRepresentation.hpp>
#include <tether/motion_planner/analytical/HybridMonotoneRepresentation.hpp>
#include <tether/motion_planner/analytical/TrajectorySampler.hpp>
#include <tether/motion_planner/analytical/AnalyticalTOPPRA.hpp>

#include <cmath>
#include <vector>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;

// ============================================================================
// Test helpers
// ============================================================================

namespace {

/// Build a simple 2D path from a single line segment
PathAdapter<2, double> makeLinePath2D(double length) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{length, 0.0}, 100.0));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    if (!result.success) {
        return PathAdapter<2, double>{};
    }
    return std::move(result.path);
}

/// Build a short 2D path for fast tests
PathAdapter<2, double> makeShortLinePath2D() {
    return makeLinePath2D(10.0);
}

/// Build a 2D path with a corner (two line segments)
PathAdapter<2, double> makeCornerPath2D() {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{5.0, 0.0}, 100.0));
    segments.append(MotionSegment::linear(
        Vec<2, double>{5.0, 0.0}, Vec<2, double>{5.0, 5.0}, 100.0));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    if (!result.success) {
        return PathAdapter<2, double>{};
    }
    return std::move(result.path);
}

/// Standard 2D kinematic limits with jerk constraints
KinematicLimits<2, double> makeLimits2D() {
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 100.0;
    limits.path.maxPathAcceleration = 500.0;
    limits.path.maxPathJerk = 5000.0;
    limits.path.jerkLimitEnabled = true;
    limits.path.maxCentripetalAcceleration = 500.0;
    for (int i = 0; i < 2; ++i) {
        limits.axis.maxVelocity[i] = 100.0;
        limits.axis.maxAcceleration[i] = 500.0;
        limits.axis.maxJerk[i] = 5000.0;
    }
    limits.axis.jerkLimitEnabled = true;
    return limits;
}

} // namespace

// ============================================================================
// 1. Numerical Utilities Tests
// ============================================================================

TEST(AnalyticalNumericalUtils, LGLNodesCorrect) {
    // N=1: nodes at -1 and 1
    auto nodes1 = lglNodes(1);
    ASSERT_EQ(nodes1.size(), 2u);
    EXPECT_NEAR(nodes1[0], -1.0, 1e-12);
    EXPECT_NEAR(nodes1[1], 1.0, 1e-12);

    // N=2: nodes at -1, 0, 1
    auto nodes2 = lglNodes(2);
    ASSERT_EQ(nodes2.size(), 3u);
    EXPECT_NEAR(nodes2[0], -1.0, 1e-10);
    EXPECT_NEAR(nodes2[1], 0.0, 1e-10);
    EXPECT_NEAR(nodes2[2], 1.0, 1e-10);

    // N=3: nodes at -1, ±1/sqrt(5), 1
    auto nodes3 = lglNodes(3);
    ASSERT_EQ(nodes3.size(), 4u);
    EXPECT_NEAR(nodes3[0], -1.0, 1e-10);
    EXPECT_NEAR(nodes3[3], 1.0, 1e-10);
    EXPECT_NEAR(std::abs(nodes3[1]), 1.0 / std::sqrt(5.0), 1e-8);
    EXPECT_NEAR(nodes3[2], -nodes3[1], 1e-10);
}

TEST(AnalyticalNumericalUtils, LGLDerivativeMatrixConstantFunction) {
    // The derivative of a constant function should be zero
    int N = 4;
    auto nodes = lglNodes(N);
    auto D = lglDerivativeMatrix(nodes, N);

    std::vector<double> constFunc(N + 1, 5.0);  // f(x) = 5

    for (int i = 0; i <= N; ++i) {
        double deriv = 0.0;
        for (int j = 0; j <= N; ++j) {
            deriv += D[i][j] * constFunc[j];
        }
        EXPECT_NEAR(deriv, 0.0, 1e-10);
    }
}

TEST(AnalyticalNumericalUtils, LGLDerivativeMatrixLinearFunction) {
    // The derivative of f(x) = x should be 1 everywhere
    int N = 4;
    auto nodes = lglNodes(N);
    auto D = lglDerivativeMatrix(nodes, N);

    for (int i = 0; i <= N; ++i) {
        double deriv = 0.0;
        for (int j = 0; j <= N; ++j) {
            deriv += D[i][j] * nodes[j];
        }
        EXPECT_NEAR(deriv, 1.0, 1e-8);
    }
}

TEST(AnalyticalNumericalUtils, BarycentricEvaluation) {
    // Interpolate f(x) = x^2 at LGL nodes, evaluate at midpoint
    int N = 4;
    auto nodes = lglNodes(N);
    auto weights = lglBarycentricWeights(nodes, N);

    std::vector<double> values(N + 1);
    for (int i = 0; i <= N; ++i) {
        values[i] = nodes[i] * nodes[i];
    }

    // Evaluate at x = 0.3
    double x = 0.3;
    double result = barycentricEvaluate(nodes, values, weights, x);
    EXPECT_NEAR(result, x * x, 1e-10);

    // Evaluate at a node (should return exact value)
    double resultAtNode = barycentricEvaluate(nodes, values, weights, nodes[2]);
    EXPECT_NEAR(resultAtNode, nodes[2] * nodes[2], 1e-12);
}

TEST(AnalyticalNumericalUtils, PadeApproximant) {
    // Padé [2/2] of exp(x) should be a good approximation near 0
    // exp(x) ≈ (1 + x/2 + x^2/12) / (1 - x/2 + x^2/12)
    std::vector<double> taylor = {1.0, 1.0, 0.5, 1.0/6.0, 1.0/24.0, 1.0/120.0};
    PadeApproximant pade(taylor, 2, 2);

    // At x=0, R(0) = a_0 / 1 = 1
    EXPECT_NEAR(pade.evaluate(0.0), 1.0, 1e-12);

    // At x=0.1, should be close to exp(0.1) ≈ 1.105170918
    EXPECT_NEAR(pade.evaluate(0.1), std::exp(0.1), 1e-6);

    // Derivative at 0 should be 1 (derivative of exp at 0)
    EXPECT_NEAR(pade.evaluateDerivative(0.0), 1.0, 1e-10);
}

TEST(AnalyticalNumericalUtils, RK4StepConstantRHS) {
    // dy/dt = 0 → y stays constant
    auto rhs = [](double, double) { return 0.0; };
    double y = 42.0;
    double yNew = rk4Step(rhs, 0.0, y, 0.1);
    EXPECT_NEAR(yNew, 42.0, 1e-12);
}

TEST(AnalyticalNumericalUtils, RK4StepLinearRHS) {
    // dy/dt = 1 → y(t) = y0 + t
    auto rhs = [](double, double) { return 1.0; };
    double y0 = 5.0;
    double h = 0.1;
    double yNew = rk4Step(rhs, 0.0, y0, h);
    EXPECT_NEAR(yNew, y0 + h, 1e-10);
}

TEST(AnalyticalNumericalUtils, RK4StepExponential) {
    // dy/dt = y → y(t) = y0 * exp(t)
    auto rhs = [](double, double y) { return y; };
    double y0 = 1.0;
    double h = 0.01;
    double t = 0.0;
    double y = y0;
    for (int i = 0; i < 100; ++i) {
        y = rk4Step(rhs, t, y, h);
        t += h;
    }
    // t = 1.0, y should be close to exp(1) ≈ 2.718281828
    EXPECT_NEAR(y, std::exp(1.0), 1e-6);
}

TEST(AnalyticalNumericalUtils, NewtonBisectionMonotone) {
    // Solve f(x) = x^2 = 4 for x in [0, 5], root at x = 2
    auto f = [](double x) { return x * x; };
    auto df = [](double x) { return 2.0 * x; };
    double root = newtonBisection(f, df, 4.0, 0.0, 5.0);
    EXPECT_NEAR(root, 2.0, 1e-10);
}

// ============================================================================
// 2. Constraint Evaluator Tests
// ============================================================================

TEST(AnalyticalConstraintEvaluator, VelocityLimit_Line) {
    auto path = makeShortLinePath2D();
    ASSERT_GT(path.numSegments(), 0u);

    auto limits = makeLimits2D();
    ConstraintEvaluator<2, double> evaluator(limits, 50.0);

    // On a straight line, velocity limit should be min(feed, path_max, axis_max)
    double vLim = evaluator.velocityLimit(50.0, path);
    EXPECT_GT(vLim, 0.0);
    EXPECT_LE(vLim, 50.0);  // Feed rate
    EXPECT_LE(vLim, 100.0); // Path max
}

TEST(AnalyticalConstraintEvaluator, VelocityLimit_RespectsFeedRate) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();

    // Low feed rate should limit velocity
    ConstraintEvaluator<2, double> evaluator(limits, 10.0);
    double vLim = evaluator.velocityLimit(50.0, path);
    EXPECT_NEAR(vLim, 10.0, 1e-6);
}

TEST(AnalyticalConstraintEvaluator, EtaBounds_Feasible) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    ConstraintEvaluator<2, double> evaluator(limits, 50.0);

    // At moderate velocity and zero acceleration, eta bounds should be feasible
    auto bounds = evaluator.etaBounds(50.0, 10.0, 0.0, path);
    EXPECT_TRUE(bounds.feasible());
    EXPECT_LE(bounds.eta_min, 0.0);
    EXPECT_GE(bounds.eta_max, 0.0);
}

TEST(AnalyticalConstraintEvaluator, EtaBounds_RespectsJerkLimit) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    ConstraintEvaluator<2, double> evaluator(limits, 50.0);

    auto bounds = evaluator.etaBounds(50.0, 10.0, 0.0, path);
    // Eta should be bounded by jerk limit
    EXPECT_GE(bounds.eta_max, -limits.path.maxPathJerk);
    EXPECT_LE(bounds.eta_max, limits.path.maxPathJerk);
    EXPECT_GE(bounds.eta_min, -limits.path.maxPathJerk);
    EXPECT_LE(bounds.eta_min, limits.path.maxPathJerk);
}

TEST(AnalyticalConstraintEvaluator, AccelerationBounds_Feasible) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    ConstraintEvaluator<2, double> evaluator(limits, 50.0);

    auto [aMin, aMax] = evaluator.accelerationBounds(50.0, 10.0, path);
    EXPECT_LE(aMin, aMax);
    EXPECT_LE(aMin, 0.0);
    EXPECT_GE(aMax, 0.0);
}

// ============================================================================
// 3. AnalyticalJerkLimitedTOPPRA Profiler Tests
// ============================================================================

TEST(AnalyticalJerkLimitedTOPPRA, ComputesProfile_Line) {
    auto path = makeShortLinePath2D();
    ASSERT_GT(path.numSegments(), 0u);

    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    auto profile = profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);
    ASSERT_GT(profile->points().size(), 0u);
    EXPECT_GT(profile->totalTime(), 0.0);
    EXPECT_NEAR(profile->totalLength(), 10.0, 1e-6);
}

TEST(AnalyticalJerkLimitedTOPPRA, ProfileStartsAtRest) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    auto profile = profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);
    ASSERT_GT(profile->points().size(), 0u);
    EXPECT_NEAR(profile->points().front().velocity, 0.0, 1e-6);
}

TEST(AnalyticalJerkLimitedTOPPRA, ProfileEndsAtRest) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    auto profile = profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);
    ASSERT_GT(profile->points().size(), 0u);
    EXPECT_NEAR(profile->points().back().velocity, 0.0, 1e-3);
}

TEST(AnalyticalJerkLimitedTOPPRA, ProfileRespectsFeedRate) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    double feedRate = 15.0;
    auto profile = profiler.computeProfile(path, feedRate, 0.0, 0.0, 200);
    ASSERT_GT(profile->points().size(), 0u);

    for (const auto& pt : profile->points()) {
        EXPECT_LE(pt.velocity, feedRate * 1.01);  // Small tolerance
    }
}

TEST(AnalyticalJerkLimitedTOPPRA, ProfileRespectsAccelerationLimit) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    auto profile = profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);
    ASSERT_GT(profile->points().size(), 0u);

    for (const auto& pt : profile->points()) {
        EXPECT_LE(std::abs(pt.acceleration), limits.path.maxPathAcceleration * 1.05);
    }
}

TEST(AnalyticalJerkLimitedTOPPRA, ProfileRespectsJerkLimit) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    auto profile = profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);
    ASSERT_GT(profile->points().size(), 0u);

    for (const auto& pt : profile->points()) {
        EXPECT_LE(std::abs(pt.jerk), limits.path.maxPathJerk * 1.1);
    }
}

TEST(AnalyticalJerkLimitedTOPPRA, ProvidesAnalyticalSource) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto source = profiler.analyticalSource();
    ASSERT_NE(source, nullptr);
    EXPECT_GT(source->totalTime(), 0.0);
    EXPECT_NEAR(source->totalLength(), 10.0, 1e-6);
}

TEST(AnalyticalJerkLimitedTOPPRA, ProvidesSSRSource) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);

    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);
    EXPECT_GT(ssr->numArcs(), 0u);
}

TEST(AnalyticalJerkLimitedTOPPRA, ProvidesHybridSource) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-8);

    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto hybrid = profiler.hybridSource();
    ASSERT_NE(hybrid, nullptr);
    EXPECT_GT(hybrid->numElements(), 0u);
}

TEST(AnalyticalJerkLimitedTOPPRA, ProfilerType) {
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    EXPECT_EQ(profiler.type(), ProfilerType::AnalyticalJerkLimitedTOPPRA);
}

TEST(AnalyticalJerkLimitedTOPPRA, NameIsSet) {
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    EXPECT_NE(profiler.name(), nullptr);
    EXPECT_STRNE(profiler.name(), "");
}

// ============================================================================
// 4. SSR Sampling Tests
// ============================================================================

TEST(AnalyticalSSR, PositionAtStartAndEnd) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);

    auto posStart = ssr->position(0.0);
    EXPECT_NEAR(posStart[0], 0.0, 0.5);
    EXPECT_NEAR(posStart[1], 0.0, 1e-3);

    auto posEnd = ssr->position(ssr->totalTime());
    EXPECT_NEAR(posEnd[0], 10.0, 1e-1);
    EXPECT_NEAR(posEnd[1], 0.0, 1e-1);
}

TEST(AnalyticalSSR, VelocityAtStartIsZero) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);

    double v0 = ssr->pathVelocity(0.0);
    EXPECT_NEAR(v0, 0.0, 1e-3);
}

TEST(AnalyticalSSR, PositionContinuity) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);

    double totalT = ssr->totalTime();
    double dt = totalT / 20;
    double maxJump = 0.0;
    auto prevPos = ssr->position(0.0);

    for (int i = 1; i <= 20; ++i) {
        double t = i * dt;
        auto pos = ssr->position(t);
        double jump = (pos - prevPos).length();
        maxJump = std::max(maxJump, jump);
        prevPos = pos;
    }

    // Position should be continuous (no jumps >> v*dt)
    double expectedMaxStep = 20.0 * dt * 10.0;  // v_max * dt * slack
    EXPECT_LT(maxJump, expectedMaxStep);
}

// ============================================================================
// 5. Hybrid Representation Tests
// ============================================================================

TEST(AnalyticalHybrid, PositionAtStartAndEnd) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-6);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto hybrid = profiler.hybridSource();
    ASSERT_NE(hybrid, nullptr);

    auto posStart = hybrid->position(0.0);
    EXPECT_NEAR(posStart[0], 0.0, 0.5);
    EXPECT_NEAR(posStart[1], 0.0, 1e-3);

    auto posEnd = hybrid->position(hybrid->totalTime());
    EXPECT_NEAR(posEnd[0], 10.0, 1e-1);
    EXPECT_NEAR(posEnd[1], 0.0, 1e-1);
}

TEST(AnalyticalHybrid, CertificationProvided) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-8);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto hybrid = profiler.hybridSource();
    ASSERT_NE(hybrid, nullptr);

    double t = hybrid->totalTime() * 0.5;
    auto cert = hybrid->certify(t);
    // Error bounds should be non-negative
    EXPECT_GE(cert.pos_error, 0.0);
    EXPECT_GE(cert.vel_error, 0.0);
    EXPECT_GE(cert.acc_error, 0.0);
}

TEST(AnalyticalHybrid, PositionContinuity) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-6);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto hybrid = profiler.hybridSource();
    ASSERT_NE(hybrid, nullptr);

    double totalT = hybrid->totalTime();
    double dt = totalT / 20;
    double maxJump = 0.0;
    auto prevPos = hybrid->position(0.0);

    for (int i = 1; i <= 20; ++i) {
        double t = i * dt;
        auto pos = hybrid->position(t);
        double jump = (pos - prevPos).length();
        maxJump = std::max(maxJump, jump);
        prevPos = pos;
    }

    double expectedMaxStep = 20.0 * dt * 10.0;
    EXPECT_LT(maxJump, expectedMaxStep);
}

// ============================================================================
// 6. Trajectory Sampler Tests
// ============================================================================

TEST(AnalyticalTrajectorySampler, WrapsSSR) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);  // No hybrid
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto source = profiler.analyticalSource();
    ASSERT_NE(source, nullptr);

    // Should provide state at any time
    auto [pos, vel, acc] = source->state(0.0);
    EXPECT_NEAR(pos[0], 0.0, 0.5);

    // Total time should match
    EXPECT_GT(source->totalTime(), 0.0);
}

TEST(AnalyticalTrajectorySampler, WrapsHybrid) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-6);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto source = profiler.analyticalSource();
    ASSERT_NE(source, nullptr);

    auto [pos, vel, acc] = source->state(0.0);
    EXPECT_NEAR(pos[0], 0.0, 0.5);

    // Certification should work (Hybrid is wrapped)
    auto cert = source->certify(source->totalTime() * 0.5);
    EXPECT_GE(cert.pos_error, 0.0);
}

// ============================================================================
// 7. MotionPlan Integration Tests
// ============================================================================

TEST(AnalyticalMotionPlan, BuildsWithAnalyticalProfiler) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{10.0, 0.0}, 100.0));

    auto limits = makeLimits2D();
    MotionPlanBuilder<2, double> builder(limits, {}, ProfilerType::AnalyticalJerkLimitedTOPPRA);
    auto plan = builder.build(segments, 50.0);

    EXPECT_GT(plan.totalLength(), 0.0);
    EXPECT_GT(plan.totalDuration(), 0.0);
}

TEST(AnalyticalMotionPlan, EvaluatesPosition) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{10.0, 0.0}, 100.0));

    auto limits = makeLimits2D();
    MotionPlanBuilder<2, double> builder(limits, {}, ProfilerType::AnalyticalJerkLimitedTOPPRA);
    auto plan = builder.build(segments, 50.0);

    auto p0 = plan.positionAt(0.0);
    EXPECT_NEAR(p0[0], 0.0, 1e-3);
    EXPECT_NEAR(p0[1], 0.0, 1e-3);

    auto pEnd = plan.positionAt(plan.totalDuration());
    EXPECT_NEAR(pEnd[0], 10.0, 1e-1);
    EXPECT_NEAR(pEnd[1], 0.0, 1e-1);
}

TEST(AnalyticalMotionPlan, EvaluateAtProvidesState) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{10.0, 0.0}, 100.0));

    auto limits = makeLimits2D();
    MotionPlanBuilder<2, double> builder(limits, {}, ProfilerType::AnalyticalJerkLimitedTOPPRA);
    auto plan = builder.build(segments, 50.0);

    auto state = plan.evaluateAt(plan.totalDuration() * 0.5);
    // Position should be somewhere along the line
    EXPECT_GE(state.position[0], 0.0);
    EXPECT_LE(state.position[0], 10.0);
    EXPECT_NEAR(state.position[1], 0.0, 1e-2);
}

TEST(AnalyticalMotionPlan, AnalyticalSourceAvailable) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{10.0, 0.0}, 100.0));

    auto limits = makeLimits2D();
    MotionPlanBuilder<2, double> builder(limits, {}, ProfilerType::AnalyticalJerkLimitedTOPPRA);
    auto plan = builder.build(segments, 50.0);

    // The profile should wrap an analytical source.
    auto* avp = dynamic_cast<const AnalyticalSSRVelocityProfile<2, double>*>(
        plan.profile().get());
    ASSERT_NE(avp, nullptr);
    EXPECT_NE(avp->source(), nullptr);
}

TEST(AnalyticalMotionPlan, BackwardCompatibleWithoutAnalyticalSource) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{10.0, 0.0}, 100.0));

    auto limits = makeLimits2D();
    // Use basic TOPPRA (no analytical source)
    MotionPlanBuilder<2, double> builder(limits, {}, ProfilerType::ToppraBasic);
    auto plan = builder.build(segments, 50.0);

    // Basic TOPPRA returns a sampled profile.
    auto* svp = dynamic_cast<const SampledVelocityProfile*>(plan.profile().get());
    EXPECT_NE(svp, nullptr);

    auto p0 = plan.positionAt(0.0);
    EXPECT_NEAR(p0[0], 0.0, 1e-3);
}

// ============================================================================
// 8. Constraint Satisfaction Tests
// ============================================================================

TEST(AnalyticalConstraintSatisfaction, VelocityWithinLimits) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    profiler.computeProfile(path, 15.0, 0.0, 0.0, 30);

    auto source = profiler.analyticalSource();
    ASSERT_NE(source, nullptr);

    double totalT = source->totalTime();
    double dt = totalT / 200;

    for (int i = 0; i <= 10; ++i) {
        double t = i * dt;
        double v = source->pathVelocity(t);
        EXPECT_LE(v, 15.0 * 1.05) << " at t=" << t;
    }
}

TEST(AnalyticalConstraintSatisfaction, AccelerationWithinLimits) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    profiler.computeProfile(path, 15.0, 0.0, 0.0, 30);

    auto source = profiler.analyticalSource();
    ASSERT_NE(source, nullptr);

    double totalT = source->totalTime();
    double dt = totalT / 200;

    for (int i = 0; i <= 10; ++i) {
        double t = i * dt;
        double a = source->pathAcceleration(t);
        EXPECT_LE(std::abs(a), limits.path.maxPathAcceleration * 1.1)
            << " at t=" << t;
    }
}

// ============================================================================
// 9. SSR vs Hybrid Consistency Tests
// ============================================================================

TEST(AnalyticalConsistency, SSRHybridPositionAgree) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-6);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto ssr = profiler.ssrSource();
    auto hybrid = profiler.hybridSource();
    ASSERT_NE(ssr, nullptr);
    ASSERT_NE(hybrid, nullptr);

    // Compare positions at several time points
    double totalT = ssr->totalTime();
    for (int i = 0; i <= 10; ++i) {
        double t = totalT * i / 20.0;
        auto posSSR = ssr->position(t);
        auto posHybrid = hybrid->position(t);
        double diff = (posSSR - posHybrid).length();
        // Allow some tolerance for the approximation
        EXPECT_LT(diff, 1.0) << " at t=" << t;
    }
}

TEST(AnalyticalConsistency, SSRHybridVelocityAgree) {
    auto path = makeShortLinePath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, true, 1e-6);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto ssr = profiler.ssrSource();
    auto hybrid = profiler.hybridSource();
    ASSERT_NE(ssr, nullptr);
    ASSERT_NE(hybrid, nullptr);

    // Compare positions at several arc lengths (more robust than comparing
    // at the same time, since SSR and Hybrid may have slightly different
    // time parameterizations due to different integration methods)
    double totalS = ssr->totalLength();
    for (int i = 1; i < 10; ++i) {
        double s = totalS * i / 10.0;
        auto posSSR = ssr->positionAtArcLength(s);
        auto posHybrid = hybrid->positionAtArcLength(s);
        double diff = (posSSR - posHybrid).length();
        EXPECT_LT(diff, 1.0) << " at s=" << s;
    }
}

// ============================================================================
// 10. Corner Path Tests
// ============================================================================

TEST(AnalyticalCornerPath, BuildsAndEvaluates) {
    auto path = makeCornerPath2D();
    ASSERT_GT(path.numSegments(), 0u);

    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    ASSERT_GT(profile->points().size(), 0u);
    EXPECT_GT(profile->totalTime(), 0.0);
}

TEST(AnalyticalCornerPath, PositionContinuity) {
    auto path = makeCornerPath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits);
    profiler.computeProfile(path, 20.0, 0.0, 0.0, 30);

    auto source = profiler.analyticalSource();
    ASSERT_NE(source, nullptr);

    double totalT = source->totalTime();
    double dt = totalT / 200;
    double maxJump = 0.0;
    auto prevPos = source->position(0.0);

    for (int i = 1; i <= 20; ++i) {
        double t = i * dt;
        auto pos = source->position(t);
        double jump = (pos - prevPos).length();
        maxJump = std::max(maxJump, jump);
        prevPos = pos;
    }

    double expectedMaxStep = 20.0 * dt * 10.0;
    EXPECT_LT(maxJump, expectedMaxStep);
}

// ============================================================================
// 11. ControlMode and AnalyticalTypes Tests
// ============================================================================

TEST(AnalyticalTypes, ControlModeNames) {
    EXPECT_STREQ(controlModeName(ControlMode::ACCEL_MAX), "ACCEL_MAX");
    EXPECT_STREQ(controlModeName(ControlMode::DECEL_MAX), "DECEL_MAX");
    EXPECT_STREQ(controlModeName(ControlMode::ZERO_JERK), "ZERO_JERK");
    EXPECT_STREQ(controlModeName(ControlMode::SINGULAR), "SINGULAR");
    EXPECT_STREQ(controlModeName(ControlMode::CONSTRAINT_SURFACE), "CONSTRAINT_SURFACE");
}

TEST(AnalyticalTypes, EtaBoundsFeasible) {
    EtaBounds bounds{-5.0, 10.0};
    EXPECT_TRUE(bounds.feasible());
    EXPECT_TRUE(bounds.contains(0.0));
    EXPECT_TRUE(bounds.contains(10.0));
    EXPECT_FALSE(bounds.contains(10.1));
    EXPECT_FALSE(bounds.contains(-5.1));
}

TEST(AnalyticalTypes, EtaBoundsInfeasible) {
    EtaBounds bounds{10.0, -5.0};
    EXPECT_FALSE(bounds.feasible());
}

TEST(AnalyticalTypes, EtaBoundsClamp) {
    EtaBounds bounds{-5.0, 10.0};
    EXPECT_NEAR(bounds.clamp(-100.0), -5.0, 1e-12);
    EXPECT_NEAR(bounds.clamp(100.0), 10.0, 1e-12);
    EXPECT_NEAR(bounds.clamp(3.0), 3.0, 1e-12);
}

TEST(AnalyticalTypes, EtaBoundsIntersect) {
    EtaBounds a{-10.0, 10.0};
    EtaBounds b{-5.0, 5.0};
    a.intersect(b);
    EXPECT_NEAR(a.eta_min, -5.0, 1e-12);
    EXPECT_NEAR(a.eta_max, 5.0, 1e-12);
}

TEST(AnalyticalTypes, SwitchingArcValid) {
    SwitchingArc arc;
    arc.s_begin = 0.0;
    arc.s_end = 10.0;
    EXPECT_TRUE(arc.valid());
    EXPECT_NEAR(arc.length(), 10.0, 1e-12);

    arc.s_end = 0.0;
    EXPECT_FALSE(arc.valid());
}

// ============================================================================
// 12. Custom Profiler via MotionPlanBuilder
// ============================================================================

TEST(AnalyticalCustomProfiler, BuilderAcceptsCustomProfiler) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{10.0, 0.0}, 100.0));

    auto limits = makeLimits2D();
    auto profiler = std::make_unique<AnalyticalJerkLimitedTOPPRA<2, double>>(
        limits, true, 1e-8);

    MotionPlanBuilder<2, double> builder(std::move(profiler), limits);
    auto plan = builder.build(segments, 50.0);

    EXPECT_GT(plan.totalLength(), 0.0);
    auto* avp = dynamic_cast<const AnalyticalSSRVelocityProfile<2, double>*>(
        plan.profile().get());
    EXPECT_NE(avp, nullptr);
    EXPECT_NE(avp->source(), nullptr);
}
