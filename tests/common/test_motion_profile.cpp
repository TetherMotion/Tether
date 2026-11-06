// MotionProfile: per-shape plan/evaluate correctness, limits, factories.
#include <gtest/gtest.h>

#include <cmath>

#include "tether/common/MotionProfile.hpp"

using namespace tether::common;

namespace {

constexpr double kEps = 1e-6;

MotionLimits limits(uint32_t vel, uint32_t acc, uint32_t dec, uint32_t jerk = 0) {
    MotionLimits l;
    l.maxVelocity = vel;
    l.maxAcceleration = acc;
    l.maxDeceleration = dec;
    l.maxJerk = jerk;
    return l;
}

// LinearProfile

TEST(MotionProfileLinear, TypeAndPlan) {
    LinearProfile p;
    EXPECT_EQ(p.getType(), ProfileType::Linear);
    p.setLimits(limits(100, 1000, 1000));
    double dur = p.plan(0.0, 1000.0);
    EXPECT_DOUBLE_EQ(dur, 10.0);           // 1000 units at 100 u/s
    EXPECT_DOUBLE_EQ(p.getDuration(), dur);
    EXPECT_DOUBLE_EQ(p.getStartPosition(), 0.0);
    EXPECT_DOUBLE_EQ(p.getEndPosition(), 1000.0);
    EXPECT_DOUBLE_EQ(p.getDistance(), 1000.0);
    EXPECT_FALSE(p.isComplete(9.9));
    EXPECT_TRUE(p.isComplete(10.0));
}

TEST(MotionProfileLinear, EvaluatePhases) {
    LinearProfile p;
    p.setLimits(limits(100, 1000, 1000));
    p.plan(0.0, 1000.0);

    MotionState s0 = p.evaluate(-0.5);
    EXPECT_DOUBLE_EQ(s0.position, 0.0);
    EXPECT_DOUBLE_EQ(s0.velocity, 0.0);
    EXPECT_FALSE(s0.complete);

    MotionState mid = p.evaluate(5.0);
    EXPECT_DOUBLE_EQ(mid.position, 500.0);
    EXPECT_DOUBLE_EQ(mid.velocity, 100.0);
    EXPECT_DOUBLE_EQ(mid.acceleration, 0.0);
    EXPECT_DOUBLE_EQ(mid.jerk, 0.0);
    EXPECT_FALSE(mid.complete);

    MotionState end = p.evaluate(10.5);
    EXPECT_DOUBLE_EQ(end.position, 1000.0);
    EXPECT_DOUBLE_EQ(end.velocity, 0.0);
    EXPECT_TRUE(end.complete);
}

TEST(MotionProfileLinear, ReverseDirection) {
    LinearProfile p;
    p.setLimits(limits(100, 1000, 1000));
    p.plan(1000.0, 0.0);
    EXPECT_DOUBLE_EQ(p.getDuration(), 10.0);
    MotionState mid = p.evaluate(5.0);
    EXPECT_DOUBLE_EQ(mid.position, 500.0);
    EXPECT_DOUBLE_EQ(mid.velocity, -100.0);
}

TEST(MotionProfileLinear, ZeroDistanceAndZeroVelocity) {
    LinearProfile p;
    p.setLimits(limits(100, 1000, 1000));
    EXPECT_DOUBLE_EQ(p.plan(50.0, 50.0), 0.0);

    p.setLimits(limits(0, 1000, 1000));
    EXPECT_DOUBLE_EQ(p.plan(0.0, 100.0), 0.0);  // no velocity → no duration
}

// TrapezoidalProfile

TEST(MotionProfileTrap, FullTrapezoid) {
    TrapezoidalProfile p;
    p.setLimits(limits(100, 500, 500));
    double dur = p.plan(0.0, 10000.0);
    EXPECT_GT(dur, 0.0);
    EXPECT_FALSE(p.isTriangular());

    double ta, tc, td;
    p.getPhaseTimes(ta, tc, td);
    EXPECT_NEAR(ta, 0.2, kEps);              // 100/500
    EXPECT_GT(tc, 0.0);
    EXPECT_NEAR(td, 0.2, kEps);
    EXPECT_NEAR(ta + tc + td, dur, kEps);

    // Continuity: position at end equals target
    EXPECT_DOUBLE_EQ(p.evaluate(dur).position, 10000.0);
    EXPECT_TRUE(p.evaluate(dur).complete);
}

TEST(MotionProfileTrap, ShortMoveIsTriangular) {
    TrapezoidalProfile p;
    p.setLimits(limits(10000, 500, 500));
    p.plan(0.0, 100.0);                      // too short to reach maxVel
    EXPECT_TRUE(p.isTriangular());
    double ta, tc, td;
    p.getPhaseTimes(ta, tc, td);
    EXPECT_DOUBLE_EQ(tc, 0.0);
    EXPECT_DOUBLE_EQ(p.evaluate(p.getDuration()).position, 100.0);
}

TEST(MotionProfileTrap, EvaluateAllPhases) {
    TrapezoidalProfile p;
    p.setLimits(limits(100, 500, 500));
    p.plan(0.0, 10000.0);

    double ta, tc, td;
    p.getPhaseTimes(ta, tc, td);

    // Before start
    MotionState s0 = p.evaluate(-1.0);
    EXPECT_DOUBLE_EQ(s0.position, 0.0);
    EXPECT_DOUBLE_EQ(s0.velocity, 0.0);

    // Acceleration phase: v grows linearly (v = a·t)
    MotionState acc = p.evaluate(ta / 2);
    EXPECT_NEAR(acc.acceleration, 500.0, kEps);
    EXPECT_NEAR(acc.velocity, 500.0 * ta / 2, kEps);

    // Coast phase: constant velocity
    MotionState coast = p.evaluate(ta + tc / 2);
    EXPECT_NEAR(coast.acceleration, 0.0, kEps);
    EXPECT_NEAR(coast.velocity, 100.0, kEps);

    // Deceleration phase
    MotionState dec = p.evaluate(ta + tc + td / 2);
    EXPECT_NEAR(dec.acceleration, -500.0, kEps);
    EXPECT_NEAR(dec.velocity, 50.0, kEps);

    // Monotonic position overall
    double prev = -1.0;
    for (double t = 0.0; t <= p.getDuration(); t += p.getDuration() / 50) {
        double pos = p.evaluate(t).position;
        EXPECT_GE(pos, prev);
        prev = pos;
    }
}

TEST(MotionProfileTrap, SpeedFactorScalesVelocity) {
    TrapezoidalProfile p;
    p.setLimits(limits(100, 500, 500));
    p.setSpeedFactor(0.5);
    p.plan(0.0, 10000.0);
    MotionState coast = p.evaluate(p.getDuration() / 2);
    EXPECT_NEAR(std::abs(coast.velocity), 50.0, kEps);
}

TEST(MotionProfileTrap, NegativeSpeedFactorReversesPlayback) {
    TrapezoidalProfile p;
    p.setLimits(limits(100, 500, 500));
    p.setSpeedFactor(-1.0);
    p.plan(0.0, 10000.0);
    // time=0 with negative factor evaluates the end of the profile
    EXPECT_DOUBLE_EQ(p.evaluate(0.0).position, 10000.0);
    EXPECT_DOUBLE_EQ(p.evaluate(p.getDuration()).position, 0.0);
}

TEST(MotionProfileTrap, ZeroDistance) {
    TrapezoidalProfile p;
    p.setLimits(limits(100, 500, 500));
    EXPECT_DOUBLE_EQ(p.plan(5.0, 5.0), 0.0);
    EXPECT_FALSE(p.isTriangular());
}

// TriangularProfile

TEST(MotionProfileTri, PlanAndEvaluate) {
    TriangularProfile p;
    p.setLimits(limits(10000, 500, 500));
    double dur = p.plan(0.0, 1000.0);
    EXPECT_GT(dur, 0.0);

    // Peak = sqrt(2*a*d*dist/(a+d)) = sqrt(500*1000) ≈ 707.1 < maxVel
    EXPECT_DOUBLE_EQ(p.evaluate(dur).position, 1000.0);
    EXPECT_TRUE(p.evaluate(dur).complete);

    MotionState s0 = p.evaluate(-1.0);
    EXPECT_DOUBLE_EQ(s0.position, 0.0);

    // Acceleration half
    double ta, tc, td;
    (void)tc;
    // First half accelerating, second decelerating
    MotionState early = p.evaluate(dur * 0.25);
    EXPECT_GT(early.acceleration, 0.0);
    MotionState late = p.evaluate(dur * 0.75);
    EXPECT_LT(late.acceleration, 0.0);
    EXPECT_DOUBLE_EQ(p.evaluate(dur * 1.1).position, 1000.0);
}

TEST(MotionProfileTri, PeakVelocityClampedToLimit) {
    TriangularProfile p;
    p.setLimits(limits(100, 500, 500));        // maxVel 100 << natural peak
    p.plan(0.0, 100000.0);
    double ta;
    // Peak should clamp at 100: t1 = 100/500 = 0.2s
    // duration = 0.2 + 0.2 = 0.4s
    EXPECT_DOUBLE_EQ(p.getDuration(), 0.4);
}

TEST(MotionProfileTri, ZeroDistance) {
    TriangularProfile p;
    p.setLimits(limits(100, 500, 500));
    EXPECT_DOUBLE_EQ(p.plan(3.0, 3.0), 0.0);
}

// SCurveProfile

TEST(MotionProfileSCurve, PlanAndEvaluate) {
    SCurveProfile p;
    p.setLimits(limits(1000, 2000, 2000, 50000));
    double dur = p.plan(0.0, 10000.0);
    EXPECT_GT(dur, 0.0);
    EXPECT_EQ(p.getPhaseCount(), 7);

    const double* t = p.getPhaseTimes();
    for (int i = 1; i < 8; ++i) EXPECT_GE(t[i], t[i - 1]);

    // Ends reach target
    EXPECT_NEAR(p.evaluate(dur).position, 10000.0, 1.0);
    EXPECT_TRUE(p.evaluate(dur).complete);

    // Before start
    MotionState s0 = p.evaluate(-1.0);
    EXPECT_DOUBLE_EQ(s0.position, 0.0);
    EXPECT_DOUBLE_EQ(s0.jerk, 50000.0);      // first-phase jerk value

    // Position should be monotonically non-decreasing
    double prev = -1.0;
    for (double ts = 0.0; ts <= dur; ts += dur / 100) {
        double pos = p.evaluate(ts).position;
        EXPECT_GE(pos, prev);
        prev = pos;
    }
}

TEST(MotionProfileSCurve, ShortMoveReducesPeak) {
    SCurveProfile p;
    p.setLimits(limits(10000, 2000, 2000, 50000));
    double dur = p.plan(0.0, 10.0);          // forces the d_coast<0 branch
    EXPECT_GT(dur, 0.0);
    EXPECT_NEAR(p.evaluate(dur).position, 10.0, 0.5);
}

TEST(MotionProfileSCurve, ZeroDistance) {
    SCurveProfile p;
    p.setLimits(limits(1000, 2000, 2000, 50000));
    EXPECT_DOUBLE_EQ(p.plan(2.0, 2.0), 0.0);
}

// PolynomialProfile

TEST(MotionProfilePoly, CubicBoundaryConditions) {
    PolynomialProfile p(PolynomialProfile::Order::Cubic);
    p.setLimits(limits(1000, 1000, 1000));
    p.setDuration(2.0);
    double dur = p.plan(0.0, 1000.0);
    EXPECT_DOUBLE_EQ(dur, 2.0);

    EXPECT_NEAR(p.evaluate(0.0).position, 0.0, kEps);
    EXPECT_NEAR(p.evaluate(2.0).position, 1000.0, kEps);
    EXPECT_NEAR(p.evaluate(2.0).velocity, 0.0, kEps);
    EXPECT_TRUE(p.evaluate(2.0).complete);
}

TEST(MotionProfilePoly, QuinticBoundaryConditions) {
    PolynomialProfile p(PolynomialProfile::Order::Quintic);
    p.setDuration(1.0);
    p.plan(0.0, 500.0);

    EXPECT_NEAR(p.evaluate(0.0).position, 0.0, kEps);
    EXPECT_NEAR(p.evaluate(1.0).position, 500.0, kEps);
    EXPECT_NEAR(p.evaluate(1.0).velocity, 0.0, 1e-4);
    // Quintic with zero boundary accel: acceleration at ends ≈ 0
    EXPECT_NEAR(p.evaluate(0.0).acceleration, 0.0, kEps);
}

TEST(MotionProfilePoly, SepticIncludesJerkTerms) {
    PolynomialProfile p(PolynomialProfile::Order::Septic);
    p.setDuration(1.0);
    p.setBoundaryJerk(10.0, -10.0);
    p.plan(0.0, 100.0);
    const double* c = p.getCoefficients();
    EXPECT_NE(c[6], 0.0);                    // jerk terms populated
    EXPECT_NE(c[7], 0.0);
    EXPECT_NEAR(p.evaluate(1.0).position, 100.0, 1e-3);
}

TEST(MotionProfilePoly, AutoDurationFromVelocityLimit) {
    PolynomialProfile p;
    p.setLimits(limits(100, 1000, 1000));      // avgVel = 50
    p.plan(0.0, 1000.0);
    EXPECT_DOUBLE_EQ(p.getDuration(), 20.0);   // 1000/50
}

TEST(MotionProfilePoly, NonZeroBoundaryVelocities) {
    PolynomialProfile p(PolynomialProfile::Order::Quintic);
    p.setDuration(2.0);
    p.plan(0.0, 1000.0, 10.0, 20.0);
    EXPECT_NEAR(p.evaluate(0.0).velocity, 10.0, kEps);
    EXPECT_NEAR(p.evaluate(2.0).velocity, 20.0, kEps);
}

// Limits / setters / factory

TEST(MotionProfileCommon, LimitSetters) {
    LinearProfile p;
    p.setMaxVelocity(42.9);
    p.setMaxAcceleration(11.0);
    p.setMaxDeceleration(22.0);
    p.setMaxJerk(33.0);
    EXPECT_EQ(p.getLimits().maxVelocity, 42u);
    EXPECT_EQ(p.getLimits().maxAcceleration, 11u);
    EXPECT_EQ(p.getLimits().maxDeceleration, 22u);
    EXPECT_EQ(p.getLimits().maxJerk, 33u);
    p.setSpeedFactor(0.25);
    EXPECT_DOUBLE_EQ(p.getSpeedFactor(), 0.25);
}

TEST(MotionProfileFactory, CreatesEachType) {
    EXPECT_EQ(createProfile(ProfileType::Linear)->getType(), ProfileType::Linear);
    EXPECT_EQ(createProfile(ProfileType::Trapezoidal)->getType(), ProfileType::Trapezoidal);
    EXPECT_EQ(createProfile(ProfileType::Triangular)->getType(), ProfileType::Triangular);
    EXPECT_EQ(createProfile(ProfileType::SCurve)->getType(), ProfileType::SCurve);
    EXPECT_EQ(createProfile(ProfileType::Polynomial)->getType(), ProfileType::Polynomial);
    EXPECT_EQ(createProfile(ProfileType::Custom)->getType(), ProfileType::Trapezoidal);
}

TEST(MotionProfileFactory, AppliesLimits) {
    auto p = createProfile(ProfileType::Linear, limits(77, 5, 5));
    EXPECT_EQ(p->getLimits().maxVelocity, 77u);
}

TEST(MotionProfileSelect, VeryShortMoveChoosesLinear) {
    // d_tri = vel²/acc = 100²/500 = 20; 1% of that = 0.2
    EXPECT_EQ(selectOptimalProfile(0.1, limits(100, 500, 500)), ProfileType::Linear);
}

TEST(MotionProfileSelect, JerkLimitedChoosesSCurve) {
    EXPECT_EQ(selectOptimalProfile(1000.0, limits(100, 500, 500, 1000)),
              ProfileType::SCurve);
}

TEST(MotionProfileSelect, DefaultTrapezoidal) {
    // t_jerk = acc/jerk ≤ 0.01 needs jerk ≥ 50000
    EXPECT_EQ(selectOptimalProfile(1000.0, limits(100, 500, 500, 500000)),
              ProfileType::Trapezoidal);
}

}  // namespace
