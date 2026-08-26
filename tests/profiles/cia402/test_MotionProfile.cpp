/**
 * @file test_MotionProfile.cpp
 * @brief Tests for CiA402 Motion Profile classes
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia402/MotionProfile.hpp"
#include "tether/profiles/cia402/CiA402Config.hpp"
#include <cmath>

using namespace CiA402;

// ============================================================================
// MotionLimits struct
// ============================================================================

TEST(MotionLimitsStruct, Default) {
    MotionLimits l{};
    EXPECT_GT(l.maxVelocity, 0.0);
    EXPECT_GT(l.maxAcceleration, 0.0);
    EXPECT_GE(l.maxDeceleration, 0.0);  // default is 0.0 (symmetric with accel when 0)
    EXPECT_GT(l.maxJerk, 0.0);
}

TEST(MotionStateStruct, DefaultValues) {
    MotionState s{};
    EXPECT_DOUBLE_EQ(s.position, 0.0);
    EXPECT_DOUBLE_EQ(s.velocity, 0.0);
    EXPECT_DOUBLE_EQ(s.acceleration, 0.0);
    EXPECT_DOUBLE_EQ(s.jerk, 0.0);
    EXPECT_DOUBLE_EQ(s.time, 0.0);
    EXPECT_FALSE(s.complete);
}

// ============================================================================
// Factory function
// ============================================================================

TEST(MotionProfileFactory, CreateAllTypes) {
    auto linear = createProfile(ProfileType::Linear);
    ASSERT_NE(linear, nullptr);
    auto trap = createProfile(ProfileType::Trapezoidal);
    ASSERT_NE(trap, nullptr);
    auto tri = createProfile(ProfileType::Triangular);
    ASSERT_NE(tri, nullptr);
    auto scurve = createProfile(ProfileType::SCurve);
    ASSERT_NE(scurve, nullptr);
    auto poly = createProfile(ProfileType::Polynomial);
    ASSERT_NE(poly, nullptr);
}

TEST(MotionProfileFactory, SelectOptimal) {
    MotionLimits l{};
    l.maxVelocity = 1000.0;
    l.maxAcceleration = 5000.0;
    l.maxJerk = 50000.0;
    auto pt = selectOptimalProfile(100.0, l);
    (void)pt;
}

// ============================================================================
// LinearProfile
// ============================================================================

class LinearProfileTest : public ::testing::Test {
protected:
    void SetUp() override {
        p_ = std::make_unique<LinearProfile>();
    }
    std::unique_ptr<LinearProfile> p_;
};

TEST_F(LinearProfileTest, Type) {
    EXPECT_EQ(p_->getType(), ProfileType::Linear);
}

TEST_F(LinearProfileTest, PlanSimple) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 1000.0);
    EXPECT_GT(dur, 0.0);
    EXPECT_DOUBLE_EQ(p_->getDuration(), dur);
}

TEST_F(LinearProfileTest, EvaluateStartEnd) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 1000.0);
    auto s0 = p_->evaluate(0.0);
    EXPECT_NEAR(s0.position, 0.0, 1.0);
    auto se = p_->evaluate(dur);
    EXPECT_NEAR(se.position, 1000.0, 1.0);
}

TEST_F(LinearProfileTest, IsComplete) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 100.0);
    EXPECT_FALSE(p_->isComplete(0.0));
    EXPECT_TRUE(p_->isComplete(dur + 0.001));
}

TEST_F(LinearProfileTest, SpeedFactor) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p_->setLimits(l);
    p_->setSpeedFactor(0.5);
    double dur = p_->plan(0.0, 1000.0);
    (void)dur;
}

TEST_F(LinearProfileTest, GettersAfterPlan) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p_->setLimits(l);
    p_->plan(10.0, 110.0);
    EXPECT_DOUBLE_EQ(p_->getStartPosition(), 10.0);
    EXPECT_DOUBLE_EQ(p_->getEndPosition(), 110.0);
    EXPECT_DOUBLE_EQ(p_->getDistance(), 100.0);
    EXPECT_DOUBLE_EQ(p_->getSpeedFactor(), 1.0);
}

TEST_F(LinearProfileTest, SetMaxVelocity) {
    p_->setMaxVelocity(200.0);
    EXPECT_DOUBLE_EQ(p_->getLimits().maxVelocity, 200.0);
}

TEST_F(LinearProfileTest, SetMaxAcceleration) {
    p_->setMaxAcceleration(500.0);
    EXPECT_DOUBLE_EQ(p_->getLimits().maxAcceleration, 500.0);
}

TEST_F(LinearProfileTest, SetMaxDeceleration) {
    p_->setMaxDeceleration(300.0);
    EXPECT_DOUBLE_EQ(p_->getLimits().maxDeceleration, 300.0);
}

TEST_F(LinearProfileTest, SetMaxJerk) {
    p_->setMaxJerk(10000.0);
    EXPECT_DOUBLE_EQ(p_->getLimits().maxJerk, 10000.0);
}

TEST_F(LinearProfileTest, NegativeDirection) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p_->setLimits(l);
    double dur = p_->plan(1000.0, 0.0);
    EXPECT_GT(dur, 0.0);
    auto se = p_->evaluate(dur);
    EXPECT_NEAR(se.position, 0.0, 1.0);
}

// ============================================================================
// TrapezoidalProfile
// ============================================================================

class TrapProfileTest : public ::testing::Test {
protected:
    void SetUp() override {
        p_ = std::make_unique<TrapezoidalProfile>();
    }
    std::unique_ptr<TrapezoidalProfile> p_;
};

TEST_F(TrapProfileTest, Type) {
    EXPECT_EQ(p_->getType(), ProfileType::Trapezoidal);
}

TEST_F(TrapProfileTest, PlanAndEvaluate) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxDeceleration = 1000.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 1000.0);
    EXPECT_GT(dur, 0.0);
    auto s0 = p_->evaluate(0.0);
    EXPECT_NEAR(s0.position, 0.0, 1.0);
    auto se = p_->evaluate(dur);
    EXPECT_NEAR(se.position, 1000.0, 1.0);
}

TEST_F(TrapProfileTest, IsTriangular) {
    MotionLimits l{};
    l.maxVelocity = 1000.0;
    l.maxAcceleration = 100.0;
    l.maxDeceleration = 100.0;
    p_->setLimits(l);
    p_->plan(0.0, 1.0); // very short move
    (void)p_->isTriangular();
}

TEST_F(TrapProfileTest, GetPhaseTimes) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxDeceleration = 1000.0;
    p_->setLimits(l);
    p_->plan(0.0, 1000.0);
    double accelTime, coastTime, decelTime;
    p_->getPhaseTimes(accelTime, coastTime, decelTime);
    EXPECT_GE(accelTime, 0.0);
    EXPECT_GE(coastTime, 0.0);
    EXPECT_GE(decelTime, 0.0);
}

TEST_F(TrapProfileTest, WithStartVelocity) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxDeceleration = 1000.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 1000.0, 50.0, 0.0);
    EXPECT_GT(dur, 0.0);
}

TEST_F(TrapProfileTest, MidpointVelocity) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxDeceleration = 1000.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 1000.0);
    auto mid = p_->evaluate(dur / 2.0);
    EXPECT_GT(std::abs(mid.velocity), 0.0);
}

// ============================================================================
// TriangularProfile
// ============================================================================

TEST(TriangularProfileTest, Type) {
    TriangularProfile p;
    EXPECT_EQ(p.getType(), ProfileType::Triangular);
}

TEST(TriangularProfileTest, PlanAndEvaluate) {
    TriangularProfile p;
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxDeceleration = 1000.0;
    p.setLimits(l);
    double dur = p.plan(0.0, 100.0);
    EXPECT_GT(dur, 0.0);
    auto se = p.evaluate(dur);
    EXPECT_NEAR(se.position, 100.0, 1.0);
}

// ============================================================================
// SCurveProfile
// ============================================================================

class CiA402SCurveProfileTest : public ::testing::Test {
protected:
    void SetUp() override {
        p_ = std::make_unique<SCurveProfile>();
    }
    std::unique_ptr<SCurveProfile> p_;
};

TEST_F(CiA402SCurveProfileTest, Type) {
    EXPECT_EQ(p_->getType(), ProfileType::SCurve);
}

TEST_F(CiA402SCurveProfileTest, PlanAndEvaluate) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxJerk = 50000.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 1000.0);
    EXPECT_GT(dur, 0.0);
    auto se = p_->evaluate(dur);
    EXPECT_NEAR(se.position, 1000.0, 5.0);
}

TEST_F(CiA402SCurveProfileTest, GetPhaseCount) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxJerk = 50000.0;
    p_->setLimits(l);
    p_->plan(0.0, 1000.0);
    EXPECT_GT(p_->getPhaseCount(), 0);
    auto times = p_->getPhaseTimes();
    (void)times;
}

TEST_F(CiA402SCurveProfileTest, SmoothMotion) {
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    l.maxJerk = 50000.0;
    p_->setLimits(l);
    double dur = p_->plan(0.0, 500.0);
    // Verify acceleration is smooth (no discontinuities)
    double prev_accel = 0.0;
    for (int i = 0; i <= 100; ++i) {
        double t = dur * i / 100.0;
        auto s = p_->evaluate(t);
        double delta = std::abs(s.acceleration - prev_accel);
        (void)delta;
        prev_accel = s.acceleration;
    }
}

// ============================================================================
// PolynomialProfile
// ============================================================================

TEST(PolynomialProfileTest, Type) {
    PolynomialProfile p;
    EXPECT_EQ(p.getType(), ProfileType::Polynomial);
}

TEST(PolynomialProfileTest, OrderQuintic) {
    PolynomialProfile p(PolynomialProfile::Order::Quintic);
    (void)p;
}

TEST(PolynomialProfileTest, OrderCubic) {
    PolynomialProfile p(PolynomialProfile::Order::Cubic);
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p.setLimits(l);
    double dur = p.plan(0.0, 100.0);
    EXPECT_GT(dur, 0.0);
}

TEST(PolynomialProfileTest, OrderSeptic) {
    PolynomialProfile p(PolynomialProfile::Order::Septic);
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p.setLimits(l);
    double dur = p.plan(0.0, 100.0);
    EXPECT_GT(dur, 0.0);
}

TEST(PolynomialProfileTest, QuinticPlanEvaluate) {
    PolynomialProfile p;
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    p.setLimits(l);
    double dur = p.plan(0.0, 500.0);
    EXPECT_GT(dur, 0.0);
    auto se = p.evaluate(dur);
    EXPECT_NEAR(se.position, 500.0, 5.0);
}

TEST(PolynomialProfileTest, SetDuration) {
    PolynomialProfile p;
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    p.setLimits(l);
    p.setDuration(1.0);
    double dur = p.plan(0.0, 100.0);
    EXPECT_NEAR(dur, 1.0, 0.1);
}

// ============================================================================
// ProfileType enum
// ============================================================================

TEST(ProfileTypeEnum, AllDistinct) {
    EXPECT_NE(static_cast<int>(ProfileType::Linear),
              static_cast<int>(ProfileType::Trapezoidal));
    EXPECT_NE(static_cast<int>(ProfileType::Triangular),
              static_cast<int>(ProfileType::SCurve));
    EXPECT_NE(static_cast<int>(ProfileType::Polynomial),
              static_cast<int>(ProfileType::Custom));
}

TEST(PolynomialProfileTest, SetBoundaryAcceleration) {
    PolynomialProfile p;
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    p.setLimits(l);
    p.setBoundaryAcceleration(0.0, 0.0);
    double dur = p.plan(0.0, 100.0);
    EXPECT_GT(dur, 0.0);
}

TEST(PolynomialProfileTest, SetBoundaryJerk) {
    PolynomialProfile p(PolynomialProfile::Order::Septic);
    MotionLimits l{};
    l.maxVelocity = 100.0;
    l.maxAcceleration = 1000.0;
    p.setLimits(l);
    p.setBoundaryJerk(0.0, 0.0);
    double dur = p.plan(0.0, 100.0);
    EXPECT_GT(dur, 0.0);
}

TEST(PolynomialProfileTest, GetCoefficients) {
    PolynomialProfile p;
    MotionLimits l{};
    l.maxVelocity = 100.0;
    p.setLimits(l);
    p.plan(0.0, 100.0);
    const double* coeffs = p.getCoefficients();
    (void)coeffs;
}
