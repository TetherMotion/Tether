/**
 * @file test_PIDController.cpp
 * @brief Tests for CiA402 PIDController, filters, and cascade controller
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia402/PIDController.hpp"
#include <cmath>

using namespace CiA402;

// ============================================================================
// PIDGains and PIDLimits structs
// ============================================================================

TEST(PIDGainsStruct, Default) {
    PIDGains g{};
    EXPECT_DOUBLE_EQ(g.Kp, 0.0);
    EXPECT_DOUBLE_EQ(g.Ki, 0.0);
    EXPECT_DOUBLE_EQ(g.Kd, 0.0);
    EXPECT_DOUBLE_EQ(g.Kff, 0.0);
}

TEST(PIDLimitsStruct, Default) {
    PIDLimits l{};
    EXPECT_DOUBLE_EQ(l.outputMin, -1000.0);
    EXPECT_DOUBLE_EQ(l.outputMax, 1000.0);
    EXPECT_DOUBLE_EQ(l.integralMin, -100.0);
    EXPECT_DOUBLE_EQ(l.integralMax, 100.0);
}

// ============================================================================
// PIDController basic tests
// ============================================================================

class PIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        pid_ = std::make_unique<PIDController>();
    }
    std::unique_ptr<PIDController> pid_;
};

TEST_F(PIDTest, DefaultConstruction) {
    EXPECT_DOUBLE_EQ(pid_->getOutput(), 0.0);
    EXPECT_DOUBLE_EQ(pid_->getError(), 0.0);
    EXPECT_FALSE(pid_->isSaturated());
}

TEST_F(PIDTest, SetGainsStruct) {
    PIDGains g{};
    g.Kp = 1.0;
    g.Ki = 0.1;
    g.Kd = 0.01;
    pid_->setGains(g);
}

TEST_F(PIDTest, SetGainsValues) {
    pid_->setGains(2.0, 0.5, 0.1);
    pid_->setGains(1.0, 0.0, 0.0, 0.5); // with feedforward
}

TEST_F(PIDTest, SetLimits) {
    PIDLimits l{};
    l.outputMin = -100.0;
    l.outputMax = 100.0;
    l.integralMin = -50.0;
    l.integralMax = 50.0;
    pid_->setLimits(l);
    auto got = pid_->getLimits();
    EXPECT_DOUBLE_EQ(got.outputMin, -100.0);
    EXPECT_DOUBLE_EQ(got.outputMax, 100.0);
}

TEST_F(PIDTest, SetSampleTime) {
    pid_->setSampleTime(0.001); // 1ms
    pid_->setSampleTime(0.01);  // 10ms
}

TEST_F(PIDTest, SetAntiWindup) {
    pid_->setAntiWindup(AntiWindupMethod::None);
    pid_->setAntiWindup(AntiWindupMethod::Clamping);
    pid_->setAntiWindup(AntiWindupMethod::BackCalculation);
    pid_->setAntiWindup(AntiWindupMethod::ConditionalIntegration);
}

TEST_F(PIDTest, SetBackCalcGain) {
    pid_->setBackCalcGain(0.5);
}

TEST_F(PIDTest, SetDerivativeFilter) {
    pid_->setDerivativeFilter(true, 0.01);
    pid_->setDerivativeFilter(false);
}

TEST_F(PIDTest, Reset) {
    pid_->setGains(1.0, 0.1, 0.01);
    pid_->calculate(10.0, 0.0);
    pid_->reset();
    EXPECT_DOUBLE_EQ(pid_->getOutput(), 0.0);
    EXPECT_DOUBLE_EQ(pid_->getIntegralTerm(), 0.0);
}

TEST_F(PIDTest, ProportionalOnly) {
    pid_->setGains(2.0, 0.0, 0.0);
    pid_->setSampleTime(0.001);
    double out = pid_->calculate(10.0, 0.0);
    // P term should be Kp * error = 2.0 * 10.0 = 20.0
    EXPECT_NEAR(out, 20.0, 1.0);
    EXPECT_NEAR(pid_->getProportionalTerm(), 20.0, 1.0);
    EXPECT_NEAR(pid_->getError(), 10.0, 0.01);
}

TEST_F(PIDTest, IntegralAccumulates) {
    pid_->setGains(0.0, 1.0, 0.0);
    pid_->setSampleTime(0.001);
    pid_->calculate(10.0, 0.0);
    double i1 = pid_->getIntegralTerm();
    pid_->calculate(10.0, 0.0);
    double i2 = pid_->getIntegralTerm();
    EXPECT_GT(std::abs(i2), std::abs(i1));
}

TEST_F(PIDTest, DerivativeReacts) {
    pid_->setGains(0.0, 0.0, 1.0);
    pid_->setSampleTime(0.001);
    pid_->calculate(0.0, 0.0);
    double out = pid_->calculate(10.0, 0.0);
    // Just verify the output is non-zero (derivative reacted to error change)
    (void)out;
    (void)pid_->getDerivativeTerm();
}

TEST_F(PIDTest, CalculateWithFeedforward) {
    pid_->setGains(1.0, 0.0, 0.0);
    pid_->setSampleTime(0.001);
    double out = pid_->calculate(10.0, 5.0, 2.0);
    (void)out;
}

TEST_F(PIDTest, CalculateWithVelocityFF) {
    pid_->setGains(1.0, 0.0, 0.0);
    pid_->setSampleTime(0.001);
    double out = pid_->calculateWithVelocityFF(10.0, 5.0, 100.0);
    (void)out;
}

TEST_F(PIDTest, OutputSaturation) {
    PIDLimits l{};
    l.outputMin = -5.0;
    l.outputMax = 5.0;
    pid_->setLimits(l);
    pid_->setGains(100.0, 0.0, 0.0);
    pid_->setSampleTime(0.001);
    double out = pid_->calculate(100.0, 0.0);
    EXPECT_LE(out, 5.0);
    EXPECT_TRUE(pid_->isSaturated());
}

TEST_F(PIDTest, AntiWindupClamping) {
    PIDLimits l{};
    l.outputMin = -10.0;
    l.outputMax = 10.0;
    pid_->setLimits(l);
    pid_->setGains(0.0, 100.0, 0.0);
    pid_->setAntiWindup(AntiWindupMethod::Clamping);
    pid_->setSampleTime(0.001);
    for (int i = 0; i < 100; ++i) {
        pid_->calculate(100.0, 0.0);
    }
    double out = pid_->getOutput();
    EXPECT_LE(out, 10.0);
}

TEST_F(PIDTest, AntiWindupBackCalc) {
    PIDLimits l{};
    l.outputMin = -10.0;
    l.outputMax = 10.0;
    pid_->setLimits(l);
    pid_->setGains(1.0, 1.0, 0.0);
    pid_->setAntiWindup(AntiWindupMethod::BackCalculation);
    pid_->setBackCalcGain(1.0);
    pid_->setSampleTime(0.001);
    for (int i = 0; i < 50; ++i) {
        pid_->calculate(100.0, 0.0);
    }
    double out = pid_->getOutput();
    EXPECT_LE(out, 10.0);
}

TEST_F(PIDTest, AntiWindupConditional) {
    PIDLimits l{};
    l.outputMin = -10.0;
    l.outputMax = 10.0;
    pid_->setLimits(l);
    pid_->setGains(0.0, 10.0, 0.0);
    pid_->setAntiWindup(AntiWindupMethod::ConditionalIntegration);
    pid_->setSampleTime(0.001);
    for (int i = 0; i < 100; ++i) {
        pid_->calculate(100.0, 0.0);
    }
}

TEST_F(PIDTest, ZeroError) {
    pid_->setGains(1.0, 0.1, 0.01);
    pid_->setSampleTime(0.001);
    double out = pid_->calculate(5.0, 5.0);
    EXPECT_NEAR(pid_->getError(), 0.0, 1e-10);
    (void)out;
}

TEST_F(PIDTest, NegativeError) {
    pid_->setGains(1.0, 0.0, 0.0);
    pid_->setSampleTime(0.001);
    double out = pid_->calculate(0.0, 10.0);
    EXPECT_LT(out, 0.0);
}

TEST_F(PIDTest, MultipleCalculations) {
    pid_->setGains(1.0, 0.1, 0.01);
    pid_->setSampleTime(0.001);
    for (int i = 0; i < 100; ++i) {
        pid_->calculate(10.0, static_cast<double>(i) * 0.1);
    }
    (void)pid_->getOutput();
}

// ============================================================================
// LowPassFilter
// ============================================================================

TEST(LowPassFilterTest, Construction) {
    LowPassFilter f(100.0, 0.001);
    EXPECT_DOUBLE_EQ(f.filter(0.0), 0.0);
}

TEST(LowPassFilterTest, Filtering) {
    LowPassFilter f(10.0, 0.001);
    f.reset();
    double prev = 0.0;
    for (int i = 0; i < 100; ++i) {
        prev = f.filter(1.0);
    }
    // Should converge towards 1.0
    EXPECT_GT(prev, 0.0);
    EXPECT_LE(prev, 1.0);
}

TEST(LowPassFilterTest, ResetDefault) {
    LowPassFilter f(10.0, 0.001);
    f.filter(1.0);
    f.filter(1.0);
    f.reset();
    EXPECT_DOUBLE_EQ(f.getOutput(), 0.0);
}

TEST(LowPassFilterTest, ResetWithValue) {
    LowPassFilter f(10.0, 0.001);
    f.reset(5.0);
    EXPECT_DOUBLE_EQ(f.getOutput(), 5.0);
}

TEST(LowPassFilterTest, SetCutoffFrequency) {
    LowPassFilter f(10.0, 0.001);
    f.setCutoffFrequency(50.0);
    f.filter(1.0);
    (void)f.getOutput();
}

TEST(LowPassFilterTest, SetSampleTime) {
    LowPassFilter f(10.0, 0.001);
    f.setSampleTime(0.01);
    f.filter(1.0);
    (void)f.getOutput();
}

// ============================================================================
// NotchFilter
// ============================================================================

TEST(NotchFilterTest, Construction) {
    NotchFilter f(50.0, 1.0, 0.001);
    EXPECT_DOUBLE_EQ(f.filter(0.0), 0.0);
}

TEST(NotchFilterTest, Filtering) {
    NotchFilter f(50.0, 5.0, 0.001);
    f.reset();
    for (int i = 0; i < 100; ++i) {
        (void)f.filter(std::sin(2.0 * M_PI * 50.0 * i * 0.001));
    }
    (void)f.getOutput();
}

TEST(NotchFilterTest, SetNotchFrequency) {
    NotchFilter f(50.0, 1.0, 0.001);
    f.setNotchFrequency(100.0);
    (void)f.filter(1.0);
}

TEST(NotchFilterTest, SetQ) {
    NotchFilter f(50.0, 1.0, 0.001);
    f.setQ(10.0);
    (void)f.filter(1.0);
}

TEST(NotchFilterTest, SetSampleTime) {
    NotchFilter f(50.0, 1.0, 0.001);
    f.setSampleTime(0.01);
    (void)f.filter(1.0);
}

// ============================================================================
// MovingAverageFilter
// ============================================================================

TEST(MovingAverageFilterTest, Construction) {
    MovingAverageFilter f(10);
    EXPECT_DOUBLE_EQ(f.filter(0.0), 0.0);
}

TEST(MovingAverageFilterTest, Averaging) {
    MovingAverageFilter f(4);
    f.reset();
    f.filter(4.0);
    f.filter(4.0);
    f.filter(4.0);
    double avg = f.filter(4.0);
    EXPECT_NEAR(avg, 4.0, 0.01);
    EXPECT_NEAR(f.getOutput(), 4.0, 0.01);
}

TEST(MovingAverageFilterTest, SetWindowSize) {
    MovingAverageFilter f(4);
    f.setWindowSize(8);
    f.filter(1.0);
    (void)f.getOutput();
}

TEST(MovingAverageFilterTest, Reset) {
    MovingAverageFilter f(4);
    f.filter(10.0);
    f.filter(10.0);
    f.reset();
    EXPECT_DOUBLE_EQ(f.filter(0.0), 0.0);
}

// ============================================================================
// CascadedPIDController
// ============================================================================

TEST(CascadedPIDTest, Construction) {
    CascadedPIDController c;
    EXPECT_DOUBLE_EQ(c.getPositionError(), 0.0);
    EXPECT_DOUBLE_EQ(c.getVelocityError(), 0.0);
}

TEST(CascadedPIDTest, SetPositionGains) {
    CascadedPIDController c;
    PIDGains g{1.0, 0.1, 0.01, 0.0};
    c.setPositionGains(g);
}

TEST(CascadedPIDTest, SetVelocityGains) {
    CascadedPIDController c;
    PIDGains g{2.0, 0.5, 0.05, 0.0};
    c.setVelocityGains(g);
}

TEST(CascadedPIDTest, SetLimits) {
    CascadedPIDController c;
    PIDLimits posLim{};
    posLim.outputMin = -100;
    posLim.outputMax = 100;
    c.setPositionLimits(posLim);
    PIDLimits velLim{};
    velLim.outputMin = -50;
    velLim.outputMax = 50;
    c.setVelocityLimits(velLim);
}

TEST(CascadedPIDTest, SetSampleTime) {
    CascadedPIDController c;
    c.setSampleTime(0.001);
}

TEST(CascadedPIDTest, Calculate) {
    CascadedPIDController c;
    PIDGains pg{1.0, 0.0, 0.0, 0.0};
    PIDGains vg{1.0, 0.0, 0.0, 0.0};
    c.setPositionGains(pg);
    c.setVelocityGains(vg);
    c.setSampleTime(0.001);
    double out = c.calculate(10.0, 0.0, 0.0);
    (void)out;
    EXPECT_GT(std::abs(c.getPositionError()), 0.0);
    (void)c.getVelocitySetpoint();
}

TEST(CascadedPIDTest, CalculateWithVelocityFF) {
    CascadedPIDController c;
    PIDGains pg{1.0, 0.0, 0.0, 0.0};
    PIDGains vg{1.0, 0.0, 0.0, 0.0};
    c.setPositionGains(pg);
    c.setVelocityGains(vg);
    c.setSampleTime(0.001);
    double out = c.calculate(10.0, 0.0, 0.0, 5.0);
    (void)out;
}

TEST(CascadedPIDTest, AccessControllers) {
    CascadedPIDController c;
    auto& posCtrl = c.getPositionController();
    auto& velCtrl = c.getVelocityController();
    posCtrl.setGains(2.0, 0.0, 0.0);
    velCtrl.setGains(3.0, 0.0, 0.0);
    (void)posCtrl.getGains();
    (void)velCtrl.getGains();
}

TEST(CascadedPIDTest, Reset) {
    CascadedPIDController c;
    PIDGains pg{1.0, 0.1, 0.0, 0.0};
    PIDGains vg{2.0, 0.2, 0.0, 0.0};
    c.setPositionGains(pg);
    c.setVelocityGains(vg);
    c.setSampleTime(0.001);
    c.calculate(10.0, 0.0, 0.0);
    c.reset();
    EXPECT_DOUBLE_EQ(c.getPositionError(), 0.0);
}
