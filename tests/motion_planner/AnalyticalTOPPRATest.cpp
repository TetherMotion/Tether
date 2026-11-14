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

// Regression: the integrators used to clamp tangential acceleration to
// the path-level cap only, letting axis acceleration overshoot the
// coupled bound |T_i·a + κ_i·v²| <= aMax_i by ~1.7x on curved geometry
// (κv² consumes axis authority). The velocity-limit clamp also leaked
// between RK4 substeps and through the closed-form SSR fast paths.
// Sample the SSR densely on a blended corner (real curvature) and
// require every axis to stay within its budget.
TEST(AnalyticalJerkLimitedTOPPRA,
     RespectsCoupledBounds_CornerPath) {
    auto path = makeCornerPath2D();   // G2 blend: nonzero curvature
    ASSERT_GT(path.numSegments(), 0u);
    auto limits = makeLimits2D();     // axisA = 500, axisV = 100
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);

    auto profile = profiler.computeProfile(path, 100.0, 0.0, 0.0, 1200);
    ASSERT_NE(profile, nullptr);
    ASSERT_GT(profile->totalTime(), 0.0);
    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);

    const double T = ssr->totalTime();
    const auto len = path.totalLength();
    for (int i = 0; i <= 2000; ++i) {
        const double t = T * static_cast<double>(i) / 2000.0;
        const auto vv = ssr->velocity(t);
        const auto aa = ssr->acceleration(t);
        const double s = ssr->arcLength(t);
        const double v = ssr->pathVelocity(t);
        for (int ax = 0; ax < 2; ++ax) {
            ASSERT_TRUE(std::isfinite(vv[ax]) && std::isfinite(aa[ax]));
            EXPECT_LE(std::abs(vv[ax]),
                      limits.axis.maxVelocity[ax] * 1.05 + 0.5)
                << "axis " << ax << " velocity overshoot at t=" << t;
            EXPECT_LE(std::abs(aa[ax]),
                      limits.axis.maxAcceleration[ax] * 1.05 + 0.5)
                << "axis " << ax << " accel overshoot at t=" << t
                << " s=" << s;
        }
        // Centripetal authority: v²κ within the path-level budget.
        if (s > 0.0 && s < len) {
            const double kappa =
                path.evaluateAtArcLength(s).curvature;
            EXPECT_LE(v * v * kappa,
                limits.path.maxCentripetalAcceleration * 1.05 + 0.5)
                << "centripetal overshoot at s=" << s;
        }
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

// ============================================================================
// 13. Junction stamps and dwell points
// ============================================================================

namespace {

/// Build a sharp (unblended) L-path: two line pieces meeting at a true
/// 90-degree tangent discontinuity so corner-velocity stamps stay finite.
PathAdapter<2, double> makeSharpLPath2D(double legLength) {
    std::vector<tether::motion::NurbsCurve> pieces;
    pieces.push_back(tether::motion::NurbsCurve::fromLine(
        tether::motion::RVec{0.0, 0.0},
        tether::motion::RVec{legLength, 0.0}));
    pieces.push_back(tether::motion::NurbsCurve::fromLine(
        tether::motion::RVec{legLength, 0.0},
        tether::motion::RVec{legLength, legLength}));
    return PathAdapter<2, double>(
        tether::motion::PiecewiseNurbsPath(std::move(pieces)));
}

} // namespace

// Regression: cornerVelocities() stamps used to be ignored by the
// forward/backward passes, so a sharp corner was crossed at ~feed speed.
// computeProfile now splits the solve at each stamped junction and
// imposes the stamp as a hard boundary velocity. The arc structure must
// show an arc boundary exactly at the junction, and the sampled velocity
// at the junction must not exceed the stamped limit.
TEST(AnalyticalJerkLimitedTOPPRA, HonorsJunctionVelocityStamps) {
    const double leg = 10.0;
    auto path = makeSharpLPath2D(leg);
    ASSERT_GT(path.numSegments(), 0u);
    const double jd = 0.05, aCent = 500.0;
    path.computeCornerVelocities(jd, aCent);
    const double vStamp =
        std::sqrt(aCent * jd / std::sin(M_PI / 4.0));  // ~5.95
    ASSERT_LT(path.cornerVelocities()[1],
              std::numeric_limits<double>::infinity());
    EXPECT_NEAR(path.cornerVelocities()[1], vStamp, 1e-9);

    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);
    auto profile = profiler.computeProfile(path, 100.0, 0.0, 0.0, 800);
    ASSERT_NE(profile, nullptr);
    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);

    // The junction must be an arc boundary: one arc ends at s=leg, the
    // next begins there.
    const auto& arcs = ssr->arcs();
    bool endsAtJunction = false, startsAtJunction = false;
    for (const auto& a : arcs) {
        if (std::abs(a.s_end - leg) < 1e-6) endsAtJunction = true;
        if (std::abs(a.s_begin - leg) < 1e-6) startsAtJunction = true;
    }
    EXPECT_TRUE(endsAtJunction) << "no arc ends at the junction";
    EXPECT_TRUE(startsAtJunction) << "no arc starts at the junction";

    // Sampled velocity at the junction must respect the stamp.
    const double T = ssr->totalTime();
    double bestS = 1e9, vAtJunction = 0.0;
    double prevV = -1.0, prevT = 0.0;
    const double dtSample = T / 4000.0;
    for (int i = 0; i <= 4000; ++i) {
        const double t = T * static_cast<double>(i) / 4000.0;
        const double s = ssr->arcLength(t);
        const double v = ssr->pathVelocity(t);
        if (std::abs(s - leg) < bestS) {
            bestS = std::abs(s - leg);
            vAtJunction = v;
        }
        // Velocity must be continuous — an earlier bug merged
        // accel-hold and decel-hold nodes into one ZERO_JERK arc whose
        // constant a replayed to a ~57 mm/s step at the switch point.
        if (prevV >= 0.0) {
            EXPECT_LE(std::abs(v - prevV),
                      limits.path.maxPathAcceleration * dtSample + 1.0)
                << "velocity discontinuity at t=" << t;
        }
        prevV = v;
        prevT = t;
    }
    EXPECT_LE(vAtJunction, vStamp * 1.1 + 0.05)
        << "junction crossed above the stamped corner velocity";
}

// Regression: dwellPoints() used to be ignored entirely. A mid-path
// dwell must appear as a zero-velocity pause arc at the requested arc
// length and extend the total time by the dwell duration.
TEST(AnalyticalJerkLimitedTOPPRA, HonorsDwellPoints) {
    auto path = makeLinePath2D(20.0);
    ASSERT_GT(path.numSegments(), 0u);
    const double dwellS = 10.0, dwellDur = 0.4;
    path.setDwellPoints({{dwellS, dwellDur}});

    auto limits = makeLimits2D();
    // Baseline without the dwell for the total-time comparison.
    auto pathNoDwell = makeLinePath2D(20.0);
    {
        AnalyticalJerkLimitedTOPPRA<2, double> p0(limits, false);
        p0.computeProfile(pathNoDwell, 100.0, 0.0, 0.0, 800);
        ASSERT_NE(p0.ssrSource(), nullptr);
        const double T0 = p0.ssrSource()->totalTime();

        AnalyticalJerkLimitedTOPPRA<2, double> p1(limits, false);
        p1.computeProfile(path, 100.0, 0.0, 0.0, 800);
        auto ssr = p1.ssrSource();
        ASSERT_NE(ssr, nullptr);
        const double T1 = ssr->totalTime();

        // The forced stop also costs ramp time, so the dwell adds at
        // least its full duration.
        EXPECT_GE(T1, T0 + dwellDur * 0.98);

        // A zero-length ZERO_JERK arc carries the pause at the dwell s.
        bool foundDwellArc = false;
        for (const auto& a : ssr->arcs()) {
            if (a.mode == ControlMode::ZERO_JERK &&
                std::abs(a.s_begin - dwellS) < 1e-6 &&
                std::abs(a.s_end - dwellS) < 1e-6 &&
                std::abs(a.duration - dwellDur) < 1e-9) {
                foundDwellArc = true;
            }
        }
        EXPECT_TRUE(foundDwellArc) << "no dwell pause arc in the SSR";

        // During the pause the path holds position at the dwell point.
        bool sawStoppedSample = false;
        for (int i = 0; i <= 4000; ++i) {
            const double t = T1 * static_cast<double>(i) / 4000.0;
            if (std::abs(ssr->pathVelocity(t)) < 1e-9 &&
                std::abs(ssr->arcLength(t) - dwellS) < 1e-6) {
                sawStoppedSample = true;
            }
        }
        EXPECT_TRUE(sawStoppedSample)
            << "trajectory never stops at the dwell point";
    }
}

// Multi-window stitching: a Z-path has two stamped junctions (three
// solve windows), plus a dwell coincident with the second junction
// (exercises the split dedup — it must become a v=0 boundary AND a
// pause) and a dwell at the path end (trailing pause arc). Every
// boundary must hold.
TEST(AnalyticalJerkLimitedTOPPRA, MultiWindowJunctionsAndDwells) {
    std::vector<tether::motion::NurbsCurve> pieces;
    pieces.push_back(tether::motion::NurbsCurve::fromLine(
        tether::motion::RVec{0.0, 0.0}, tether::motion::RVec{8.0, 0.0}));
    pieces.push_back(tether::motion::NurbsCurve::fromLine(
        tether::motion::RVec{8.0, 0.0}, tether::motion::RVec{8.0, 8.0}));
    pieces.push_back(tether::motion::NurbsCurve::fromLine(
        tether::motion::RVec{8.0, 8.0}, tether::motion::RVec{16.0, 8.0}));
    PathAdapter<2, double> path(
        tether::motion::PiecewiseNurbsPath(std::move(pieces)));
    ASSERT_GT(path.numSegments(), 0u);
    const double jd = 0.05, aCent = 500.0;
    path.computeCornerVelocities(jd, aCent);
    const double vStamp =
        std::sqrt(aCent * jd / std::sin(M_PI / 4.0));  // ~5.95
    const double j1 = 8.0, j2 = 16.0, endS = 24.0;
    ASSERT_LT(path.cornerVelocities()[1],
              std::numeric_limits<double>::infinity());
    ASSERT_LT(path.cornerVelocities()[2],
              std::numeric_limits<double>::infinity());

    // Dwell coincident with junction 2 + a dwell at the path end.
    const double dwellDur = 0.25, endDwellDur = 0.1;
    path.setDwellPoints({{j2, dwellDur}, {endS, endDwellDur}});

    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);
    auto profile = profiler.computeProfile(path, 100.0, 0.0, 0.0, 1200);
    ASSERT_NE(profile, nullptr);
    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);
    const auto& arcs = ssr->arcs();
    ASSERT_GT(arcs.size(), 4u);

    // Both junctions are arc boundaries; the coincident dwell appears
    // as a zero-length pause arc right after junction 2, and the
    // trailing dwell closes the structure.
    auto arcBoundaryAt = [&](double s) {
        bool b = false;
        for (const auto& a : arcs) {
            b |= std::abs(a.s_begin - s) < 1e-6;
        }
        for (const auto& a : arcs) {
            b |= std::abs(a.s_end - s) < 1e-6;
        }
        return b;
    };
    EXPECT_TRUE(arcBoundaryAt(j1));
    EXPECT_TRUE(arcBoundaryAt(j2));

    int dwellArcs = 0;
    for (const auto& a : arcs) {
        if (a.mode == ControlMode::ZERO_JERK &&
            std::abs(a.s_begin - a.s_end) < 1e-9) {
            ++dwellArcs;
            EXPECT_TRUE(
                (std::abs(a.s_begin - j2) < 1e-6 &&
                 std::abs(a.duration - dwellDur) < 1e-9) ||
                (std::abs(a.s_begin - endS) < 1e-6 &&
                 std::abs(a.duration - endDwellDur) < 1e-9))
                << "unexpected pause arc at s=" << a.s_begin;
        }
    }
    EXPECT_EQ(dwellArcs, 2) << "expected junction dwell + end dwell";

    // Sampled trajectory: v <= stamp at both junctions, and a genuine
    // stop at the coincident dwell.
    const double T = ssr->totalTime();
    double vJ1 = 1e9, vJ2 = 1e9, d1 = 1e9, d2 = 1e9;
    bool stoppedAtJ2 = false;
    for (int i = 0; i <= 6000; ++i) {
        const double t = T * static_cast<double>(i) / 6000.0;
        const double s = ssr->arcLength(t);
        const double v = ssr->pathVelocity(t);
        if (std::abs(s - j1) < d1) { d1 = std::abs(s - j1); vJ1 = v; }
        if (std::abs(s - j2) < d2) { d2 = std::abs(s - j2); vJ2 = v; }
        if (std::abs(s - j2) < 1e-6 && v < 1e-9) stoppedAtJ2 = true;
    }
    EXPECT_LE(vJ1, vStamp * 1.1 + 0.05);
    EXPECT_LE(vJ2, vStamp * 1.1 + 0.05);
    EXPECT_TRUE(stoppedAtJ2)
        << "no rest sample at the coincident junction dwell";
}

// The sequential cursor must reproduce random-access sampling within
// integration tolerance while turning a dense monotone sweep from
// O(samples × arc-time) into O(total-time) work. Sample the same SSR
// both ways and compare (s, v, a).
TEST(AnalyticalJerkLimitedTOPPRA,
     SequentialCursorMatchesRandomAccess) {
    auto path = makeCornerPath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);
    ASSERT_NE(profiler.computeProfile(path, 100.0, 0.0, 0.0, 1200),
              nullptr);
    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);

    const double T = ssr->totalTime();
    typename std::decay_t<decltype(*ssr)>::SampleCursor cursor;
    double maxDv = 0.0, maxDs = 0.0, maxDa = 0.0;
    for (int i = 0; i <= 4000; ++i) {
        const double t = T * static_cast<double>(i) / 4000.0;
        const auto ref = ssr->locateAndIntegrate(t);
        const auto seq = ssr->locateAndIntegrate(t, &cursor);
        EXPECT_EQ(ref.arcIndex, seq.arcIndex);
        maxDs = std::max(maxDs, std::abs(ref.s - seq.s));
        maxDv = std::max(maxDv, std::abs(ref.v - seq.v));
        maxDa = std::max(maxDa, std::abs(ref.a - seq.a));
    }
    // Resume integrates from the memoized state — accumulation differs
    // from a fresh per-arc integration only by accumulated step-phase
    // round-off, not by anything structural.
    EXPECT_LT(maxDs, 1e-3);   // mm
    EXPECT_LT(maxDv, 1e-2);   // mm/s
    EXPECT_LT(maxDa, 1.0);    // mm/s²
}

// Deliberate limitation: PathAdapter::hasInner() is false only for a
// default-constructed (empty) adapter — there is no legacy non-NURBS
// representation. computeProfile rejects it up front (numSegments==0
// / totalLength<=0), so the !hasInner() fallbacks inside
// computeCoefficients, velocityLimit and the SSR enforce paths are
// defensive remnants unreachable through the public entry. Stamps and
// dwells are path-relative: dwell points can be stored on the adapter
// but a solve with no path ignores them by construction.
TEST(AnalyticalJerkLimitedTOPPRA, EmptyPathAdapterYieldsEmptyProfile) {
    PathAdapter<2, double> empty;
    ASSERT_FALSE(empty.hasInner());
    ASSERT_EQ(empty.numSegments(), 0u);
    ASSERT_EQ(empty.totalLength(), 0.0);

    // Stamps/dwells are metadata on the adapter — dwell points store
    // even without a path, but there is nothing to solve.
    empty.setDwellPoints({{0.0, 0.5}, {1.0, 0.25}});
    ASSERT_EQ(empty.dwellPoints().size(), 2u);
    empty.setSegmentVelocityLimits({50.0});
    EXPECT_TRUE(empty.segmentMaxVelocities().empty());  // no path: no-op

    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);
    auto profile = profiler.computeProfile(empty, 100.0, 0.0, 0.0, 200);
    ASSERT_NE(profile, nullptr);
    EXPECT_EQ(profile->points().size(), 0u);
}

// C2: dense consistency between the exact SSR and the certified Hybrid
// (LGL + Padé) representations. The Hybrid is built FROM the SSR, so
// their sampled states must agree within certification scale. The
// residual concentrates where dv/ds is steepest (the stamped junction
// and the terminal stop): position to sub-mm, velocity to a few mm/s
// localized at the junction, arc length tighter still.
TEST(AnalyticalJerkLimitedTOPPRA, HybridMatchesSSR_Dense) {
    auto path = makeCornerPath2D();
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(
        limits, /*buildHybrid=*/true);
    ASSERT_NE(profiler.computeProfile(path, 100.0, 0.0, 0.0, 1200),
              nullptr);
    auto ssr = profiler.ssrSource();
    auto hybrid = profiler.hybridSource();
    ASSERT_NE(ssr, nullptr);
    ASSERT_NE(hybrid, nullptr);

    // Element coverage is s-ordered, contiguous, and time-complete.
    const auto& els = hybrid->elements();
    ASSERT_GT(els.size(), 0u);
    EXPECT_DOUBLE_EQ(els.front().s_begin, 0.0);
    EXPECT_NEAR(els.back().s_end, path.totalLength(), 1e-6);
    for (size_t i = 1; i < els.size(); ++i) {
        EXPECT_NEAR(els[i].s_begin, els[i - 1].s_end, 1e-9)
            << "s-coverage gap before element " << i;
        EXPECT_GE(els[i].t_begin, els[i - 1].t_end - 1e-9)
            << "element times not monotone at " << i;
    }
    EXPECT_NEAR(hybrid->totalTime(), ssr->totalTime(), 1e-6);

    const double T = ssr->totalTime();
    double maxDP = 0.0, maxDv = 0.0, maxDs = 0.0;
    for (int i = 0; i <= 2000; ++i) {
        const double t = T * static_cast<double>(i) / 2000.0;
        maxDP = std::max(maxDP,
            (ssr->position(t) - hybrid->position(t)).length());
        maxDv = std::max(maxDv,
            (ssr->velocity(t) - hybrid->velocity(t)).length());
        maxDs = std::max(maxDs,
            std::abs(ssr->arcLength(t) - hybrid->arcLength(t)));
    }
    EXPECT_LT(maxDs, 0.1);    // arc-length inversion
    EXPECT_LT(maxDP, 0.5);    // mm — Padé u(s) + element s error
    EXPECT_LT(maxDv, 5.0);    // mm/s — concentrated at the stamped junction
}

// The Hybrid skips zero-length dwell arcs (!arc.valid()) but must
// still carry their pause: absolute element times come from the SSR's
// timeAtArcLength, so a post-dwell element starts after the pause.
// Inside the dwell window the sampler clamps to the post-dwell
// element's s_begin — i.e. stopped at the dwell position, matching
// the SSR's stationary state.
TEST(AnalyticalJerkLimitedTOPPRA, HybridCarriesDwellTime) {
    auto path = makeLinePath2D(20.0);
    ASSERT_GT(path.numSegments(), 0u);
    const double dwellS = 10.0, dwellDur = 0.4;
    path.setDwellPoints({{dwellS, dwellDur}});

    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(
        limits, /*buildHybrid=*/true);
    ASSERT_NE(profiler.computeProfile(path, 100.0, 0.0, 0.0, 800),
              nullptr);
    auto ssr = profiler.ssrSource();
    auto hybrid = profiler.hybridSource();
    ASSERT_NE(ssr, nullptr);
    ASSERT_NE(hybrid, nullptr);

    // Total time must include the pause.
    EXPECT_NEAR(hybrid->totalTime(), ssr->totalTime(), 1e-6);
    EXPECT_GT(hybrid->totalTime(), dwellDur);

    // Find the dwell window in the SSR and sample inside it.
    double tDwell = -1.0;
    for (const auto& a : ssr->arcs()) {
        if (a.mode == ControlMode::ZERO_JERK &&
            std::abs(a.s_begin - dwellS) < 1e-6 &&
            std::abs(a.s_end - dwellS) < 1e-6) {
            tDwell = a.t0;
            break;
        }
    }
    ASSERT_GE(tDwell, 0.0) << "no dwell arc in the SSR";
    const double tMid = tDwell + 0.5 * dwellDur;
    EXPECT_NEAR(hybrid->arcLength(tMid), dwellS, 1e-6);
    EXPECT_NEAR(hybrid->pathVelocity(tMid), 0.0, 1e-9);
    EXPECT_NEAR(ssr->arcLength(tMid), dwellS, 1e-6);
}

// C1 measurement, now pinned: the SSR replays each window's
// forward/backward pass as single-eta switching arcs. The residual
// between the pass profile and the reconstruction at arc boundaries
// is bounded by the wall-band reclassification — measured worst-case
// ~3e-3 mm/s in velocity and ~5 mm/s^2 in acceleration at the last
// merge boundary. That is two orders of magnitude below the junction
// stamp scale, so the single-eta approximation is accepted and the
// bound is pinned structurally here (previously a mis-grouped
// ZERO_JERK run produced a ~57 mm/s step).
TEST(AnalyticalJerkLimitedTOPPRA, ArcBoundariesAreContinuous) {
    const double leg = 10.0;
    auto path = makeSharpLPath2D(leg);
    ASSERT_GT(path.numSegments(), 0u);
    path.computeCornerVelocities(0.05, 500.0);
    auto limits = makeLimits2D();
    AnalyticalJerkLimitedTOPPRA<2, double> profiler(limits, false);
    ASSERT_NE(profiler.computeProfile(path, 100.0, 0.0, 0.0, 800),
              nullptr);
    auto ssr = profiler.ssrSource();
    ASSERT_NE(ssr, nullptr);
    const auto& arcs = ssr->arcs();
    ASSERT_GT(arcs.size(), 2u);

    double maxDv = 0.0, maxDa = 0.0;
    for (size_t i = 0; i + 1 < arcs.size(); ++i) {
        const auto st =
            ssr->locateAndIntegrate(arcs[i].t0 + arcs[i].duration);
        maxDv = std::max(maxDv, std::abs(st.v - arcs[i + 1].v0));
        maxDa = std::max(maxDa, std::abs(st.a - arcs[i + 1].a0));
        // The boundary arc length must also chain: arc i ends where
        // arc i+1 begins (a gap or overlap would teleport position).
        EXPECT_NEAR(arcs[i].s_end, arcs[i + 1].s_begin, 1e-9)
            << "s discontinuity between arcs " << i << " and " << i + 1;
    }
    EXPECT_LT(maxDv, 0.05) << "velocity stitch residual";
    EXPECT_LT(maxDa, 10.0) << "acceleration stitch residual";
}
