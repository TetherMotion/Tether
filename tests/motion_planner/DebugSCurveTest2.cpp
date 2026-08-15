/**
 * @file VelocityProfilerInterfaceTest.cpp
 * @brief Tests for the IVelocityProfiler interface and both profiler
 *        implementations (JerkLimitedVelocityProfiler and
 *        SCurveVelocityProfiler).
 *
 * Verifies:
 *   1. Both profilers implement the IVelocityProfiler interface.
 *   2. JerkLimitedVelocityProfiler: jerk bounded by j_max everywhere.
 *   3. JerkLimitedVelocityProfiler: acceleration bounded by a_max.
 *   4. JerkLimitedVelocityProfiler: velocity bounded by v_lim / feedRate.
 *   5. SCurveVelocityProfiler: jerk bounded by j_max everywhere.
 *   6. SCurveVelocityProfiler: acceleration bounded by a_max.
 *   7. Both profilers: position trajectory starts and ends at correct endpoints.
 *   8. Both profilers: velocity starts and ends at rest (v=0).
 *   9. MotionPlanBuilder: profiler type selection works.
 *   10. Curvature-based velocity slowdown on arc segments (both profilers).
 *   11. Custom profiler instance can be passed to MotionPlanBuilder.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/JerkLimitedVelocityProfiler.hpp>
#include <tether/motion_planner/SCurveVelocityProfiler.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>

#include <cmath>
#include <memory>
#include <vector>

using namespace MotionPlanner;

namespace {

/// Build a 2D plan with a specific profiler type.
MotionPlan2D buildPlanWithProfiler(
    const MotionSegmentList& segments,
    double feedrate,
    KinematicLimits<2, double> limits,
    ProfilerType profilerType) {
    MotionPlanBuilder2D builder(limits, {}, profilerType);
    return builder.build(segments, feedrate);
}

/// Build a path for testing (without building a plan, for direct profiler tests).
PathAdapter<2, double> buildPath(const MotionSegmentList& segments) {
    tether::motion::BlendSpec blendSpec;
    blendSpec.mode = tether::motion::PathMode::Blend;
    blendSpec.tolerance = 0.1;
    blendSpec.continuity = tether::motion::Continuity::G2;
    blendSpec.maxBlendFraction = 0.25;
    blendSpec.curveType = tether::motion::BlendCurveType::BezierGk;

    PathBuilderAdapter<2, double> pathBuilder;
    auto result = pathBuilder.build(segments, blendSpec);
    return std::move(result.path);
}

} // namespace

// ============================================================================
// 1. Interface compliance: both profilers are IVelocityProfiler
// ============================================================================
TEST(VelocityProfilerInterface, BothProfilersImplementInterface) {
    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    std::unique_ptr<IVelocityProfiler<2, double>> jerkLimited =
        std::make_unique<JerkLimitedVelocityProfiler<2, double>>(limits);
    std::unique_ptr<IVelocityProfiler<2, double>> scurve =
        std::make_unique<SCurveVelocityProfiler<2, double>>(limits);

    EXPECT_EQ(jerkLimited->type(), ProfilerType::ToppraJerkLimited);
    EXPECT_EQ(scurve->type(), ProfilerType::SCurve);
    EXPECT_NE(jerkLimited->name(), nullptr);
    EXPECT_NE(scurve->name(), nullptr);
}

// ============================================================================
// 2. JerkLimitedVelocityProfiler: jerk bounded by j_max
// ============================================================================
TEST(JerkLimitedVelocityProfilerTest, JerkBounded) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{50, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.maxPathJerk = 2000.0;
    limits.path.jerkLimitEnabled = true;

    auto path = buildPath(segments);
    JerkLimitedVelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 100.0, 0, 0, 200);

    ASSERT_GT(profile.points().size(), 1u);

    double maxJerk = 0.0;
    for (const auto& pt : profile.points()) {
        maxJerk = std::max(maxJerk, std::abs(pt.jerk));
    }
    EXPECT_LE(maxJerk, 2000.0 * 1.05)
        << "Jerk exceeded limit (max = " << maxJerk << ")";
}

// ============================================================================
// 3. JerkLimitedVelocityProfiler: acceleration bounded by a_max
// ============================================================================
TEST(JerkLimitedVelocityProfilerTest, AccelerationBounded) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{50, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.maxPathAcceleration = 200.0;
    limits.path.maxPathJerk = 5000.0;
    limits.path.jerkLimitEnabled = true;

    auto path = buildPath(segments);
    JerkLimitedVelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 100.0, 0, 0, 200);

    ASSERT_GT(profile.points().size(), 1u);

    double maxAccel = 0.0;
    for (const auto& pt : profile.points()) {
        maxAccel = std::max(maxAccel, std::abs(pt.acceleration));
    }
    EXPECT_LE(maxAccel, 200.0 * 1.05)
        << "Acceleration exceeded limit (max = " << maxAccel << ")";
}

// ============================================================================
// 4. JerkLimitedVelocityProfiler: velocity bounded by feedRate
// ============================================================================
TEST(JerkLimitedVelocityProfilerTest, VelocityBounded) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{50, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    auto path = buildPath(segments);
    JerkLimitedVelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 80.0, 0, 0, 200);

    ASSERT_GT(profile.points().size(), 1u);

    double maxVel = 0.0;
    for (const auto& pt : profile.points()) {
        maxVel = std::max(maxVel, std::abs(pt.velocity));
    }
    EXPECT_LE(maxVel, 80.0 * 1.01)
        << "Velocity exceeded feed rate (max = " << maxVel << ")";
}

// ============================================================================
// 5. SCurveVelocityProfiler: jerk bounded by j_max
// ============================================================================
TEST(SCurveVelocityProfilerTest, JerkBounded) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{50, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.maxPathJerk = 2000.0;
    limits.path.jerkLimitEnabled = true;

    auto path = buildPath(segments);
    SCurveVelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 100.0, 0, 0, 200);

    ASSERT_GT(profile.points().size(), 1u);

    double maxJerk = 0.0;
    for (const auto& pt : profile.points()) {
        maxJerk = std::max(maxJerk, std::abs(pt.jerk));
    }
    EXPECT_LE(maxJerk, 2000.0 * 1.05)
        << "Jerk exceeded limit (max = " << maxJerk << ")";
}

// ============================================================================
// 6. SCurveVelocityProfiler: acceleration bounded by a_max
// ============================================================================
TEST(SCurveVelocityProfilerTest, AccelerationBounded) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{50, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.maxPathAcceleration = 200.0;
    limits.path.maxPathJerk = 5000.0;
    limits.path.jerkLimitEnabled = true;

    auto path = buildPath(segments);
    SCurveVelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 100.0, 0, 0, 200);

    ASSERT_GT(profile.points().size(), 1u);

    double maxAccel = 0.0;
    for (const auto& pt : profile.points()) {
        maxAccel = std::max(maxAccel, std::abs(pt.acceleration));
    }
    EXPECT_LE(maxAccel, 200.0 * 1.05)
        << "Acceleration exceeded limit (max = " << maxAccel << ")";
}

// ============================================================================
// 7. Both profilers: position endpoints correct
// ============================================================================
TEST(VelocityProfilerInterface, PositionEndpointsCorrect) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{10, 0}, 100.0));
    segments.append(MotionSegment::linear(Vec2{10, 0}, Vec2{10, 10}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    for (ProfilerType pt : {ProfilerType::ToppraJerkLimited,
                            ProfilerType::SCurve,
                            ProfilerType::ToppraBasic}) {
        auto plan = buildPlanWithProfiler(segments, 100.0, limits, pt);
        ASSERT_GT(plan.totalLength(), 0.0);

        auto p0 = plan.positionAt(0.0);
        auto pEnd = plan.positionAt(plan.totalDuration());

        EXPECT_NEAR(p0[0], 0.0, 1e-3);
        EXPECT_NEAR(p0[1], 0.0, 1e-3);
        EXPECT_NEAR(pEnd[0], 10.0, 1e-3);
        EXPECT_NEAR(pEnd[1], 10.0, 1e-3);
    }
}

// ============================================================================
// 8. Both profilers: velocity starts and ends at rest
// ============================================================================
TEST(VelocityProfilerInterface, StartAndEndAtRest) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{10, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    for (ProfilerType pt : {ProfilerType::ToppraJerkLimited,
                            ProfilerType::SCurve,
                            ProfilerType::ToppraBasic}) {
        auto plan = buildPlanWithProfiler(segments, 100.0, limits, pt);
        ASSERT_GT(plan.totalDuration(), 0.0);

        auto s0 = plan.evaluateAt(0.0);
        auto sEnd = plan.evaluateAt(plan.totalDuration());

        EXPECT_NEAR(s0.pathVelocity, 0.0, 1e-3)
            << "Start velocity not zero for profiler type "
            << static_cast<int>(pt);
        EXPECT_NEAR(sEnd.pathVelocity, 0.0, 1e-3)
            << "End velocity not zero for profiler type "
            << static_cast<int>(pt);
    }
}

// ============================================================================
// 9. MotionPlanBuilder: profiler type selection works
// ============================================================================
TEST(VelocityProfilerInterface, BuilderProfilerTypeSelection) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{10, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    // Build with jerk-limited profiler
    MotionPlanBuilder2D builderJerk(limits, {}, ProfilerType::ToppraJerkLimited);
    auto planJerk = builderJerk.build(segments, 100.0);
    ASSERT_GT(planJerk.totalDuration(), 0.0);

    // Build with S-curve profiler
    MotionPlanBuilder2D builderSCurve(limits, {}, ProfilerType::SCurve);
    auto planSCurve = builderSCurve.build(segments, 100.0);
    ASSERT_GT(planSCurve.totalDuration(), 0.0);

    // Build with basic TOPP-RA
    MotionPlanBuilder2D builderBasic(limits, {}, ProfilerType::ToppraBasic);
    auto planBasic = builderBasic.build(segments, 100.0);
    ASSERT_GT(planBasic.totalDuration(), 0.0);

    // All should produce valid plans with correct endpoints
    for (auto* plan : {&planJerk, &planSCurve, &planBasic}) {
        auto p0 = plan->positionAt(0.0);
        auto pEnd = plan->positionAt(plan->totalDuration());
        EXPECT_NEAR(p0[0], 0.0, 1e-3);
        EXPECT_NEAR(pEnd[0], 10.0, 1e-3);
    }
}

// ============================================================================
// 10. Curvature-based velocity slowdown (both profilers)
// ============================================================================
TEST(VelocityProfilerInterface, ArcSegmentVelocitySlowdown) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{10, 0}, 100.0));
    segments.append(MotionSegment::arcCW(
        Vec2{10, 0}, Vec2{20, 0}, Vec2{15, 0}, 100.0));
    segments.append(MotionSegment::linear(Vec2{20, 0}, Vec2{30, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    for (ProfilerType pt : {ProfilerType::ToppraJerkLimited,
                            ProfilerType::SCurve,
                            ProfilerType::ToppraBasic}) {
        auto plan = buildPlanWithProfiler(segments, 100.0, limits, pt);
        ASSERT_GT(plan.totalDuration(), 0.0);

        double maxVelOnArc = 0.0;
        const double dt = 0.001;
        for (double t = 0.0; t <= plan.totalDuration() + 1e-9; t += dt) {
            auto state = plan.evaluateAt(t);
            if (state.position[0] > 11.0 && state.position[0] < 19.0) {
                maxVelOnArc = std::max(maxVelOnArc,
                                       std::abs(state.pathVelocity));
            }
        }
        EXPECT_LT(maxVelOnArc, 100.0)
            << "Arc velocity should be reduced by curvature for type "
            << static_cast<int>(pt) << " (max = " << maxVelOnArc << ")";
    }
}

// ============================================================================
// 11. Custom profiler instance can be passed to MotionPlanBuilder
// ============================================================================
TEST(VelocityProfilerInterface, CustomProfilerInstance) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{10, 0}, 100.0));

    KinematicLimits<2, double> limits;
    limits.path.jerkLimitEnabled = true;

    auto customProfiler =
        std::make_unique<JerkLimitedVelocityProfiler<2, double>>(limits);

    MotionPlanBuilder2D builder(std::move(customProfiler), limits);
    auto plan = builder.build(segments, 100.0);

    ASSERT_GT(plan.totalDuration(), 0.0);
    auto p0 = plan.positionAt(0.0);
    auto pEnd = plan.positionAt(plan.totalDuration());
    EXPECT_NEAR(p0[0], 0.0, 1e-3);
    EXPECT_NEAR(pEnd[0], 10.0, 1e-3);
}

// ============================================================================
// 12. JerkLimitedVelocityProfiler: custom limits respected
// ============================================================================
TEST(JerkLimitedVelocityProfilerTest, CustomLimitsRespected) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(Vec2{0, 0}, Vec2{50, 0}, 200.0));

    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 200.0;
    limits.path.maxPathAcceleration = 100.0;
    limits.path.maxPathJerk = 1000.0;
    limits.path.jerkLimitEnabled = true;

    auto path = buildPath(segments);
    JerkLimitedVelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(path, 200.0, 0, 0, 200);

    ASSERT_GT(profile.points().size(), 1u);

    double maxAccel = 0.0, maxJerk = 0.0;
    for (const auto& pt : profile.points()) {
        maxAccel = std::max(maxAccel, std::abs(pt.acceleration));
        maxJerk = std::max(maxJerk, std::abs(pt.jerk));
    }
    EXPECT_LE(maxAccel, 100.0 * 1.05)
        << "Custom acceleration limit not respected";
    EXPECT_LE(maxJerk, 1000.0 * 1.05)
        << "Custom jerk limit not respected";
}
