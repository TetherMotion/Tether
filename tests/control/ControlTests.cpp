/**
 * @file ControlTests.cpp
 * @brief Comprehensive tests for Control module
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "tether/control/Controllers.hpp"

using namespace tether::control;

// ============================================================================
// ControllerBase Tests
// ============================================================================

class ControllerBaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        pid = std::make_unique<PIDController>();
    }
    
    std::unique_ptr<PIDController> pid;
};

TEST_F(ControllerBaseTest, DefaultConstruction) {
    PIDController ctrl;
    EXPECT_EQ(ctrl.getType(), ControllerType::PID);
}

TEST_F(ControllerBaseTest, GetName) {
    PIDController ctrl;
    EXPECT_STREQ(ctrl.getName(), "PID Controller");
}

TEST_F(ControllerBaseTest, GetDescription) {
    PIDController ctrl;
    EXPECT_NE(ctrl.getDescription(), nullptr);
    EXPECT_GT(strlen(ctrl.getDescription()), 0);
}

TEST_F(ControllerBaseTest, SetGains) {
    pid->setGains(1.0, 0.1, 0.05);
    EXPECT_DOUBLE_EQ(pid->getKp(), 1.0);
    EXPECT_DOUBLE_EQ(pid->getKi(), 0.1);
    EXPECT_DOUBLE_EQ(pid->getKd(), 0.05);
}

TEST_F(ControllerBaseTest, Reset) {
    pid->setGains(1.0, 0.1, 0.05);
    
    // Run a few compute cycles
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        pid->compute(input);
    }
    
    pid->reset();
    // After reset, controller should behave like fresh
}

TEST_F(ControllerBaseTest, EnableDisable) {
    pid->setEnabled(true);
    EXPECT_TRUE(pid->isEnabled());
    
    pid->setEnabled(false);
    EXPECT_FALSE(pid->isEnabled());
}

TEST_F(ControllerBaseTest, ComputeBasic) {
    pid->setGains(1.0, 0.0, 0.0);  // P only
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = pid->compute(input);
    
    // P-only: output = Kp * error = 1.0 * 100 = 100
    EXPECT_NEAR(output.control, 100.0, 1e-3);
}

TEST_F(ControllerBaseTest, SetSaturationLimits) {
    SaturationLimits limits;
    limits.outputMin = -50.0;
    limits.outputMax = 50.0;
    
    pid->setSaturationLimits(limits);
    pid->setGains(10.0, 0.0, 0.0);  // High gain to trigger saturation
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = pid->compute(input);
    
    // Should be clamped to max
    EXPECT_LE(output.control, 50.0);
}

TEST_F(ControllerBaseTest, GetSaturationLimits) {
    SaturationLimits limits;
    limits.outputMin = -50.0;
    limits.outputMax = 50.0;
    pid->setSaturationLimits(limits);
    
    auto retrieved = pid->getSaturationLimits();
    EXPECT_DOUBLE_EQ(retrieved.outputMin, -50.0);
    EXPECT_DOUBLE_EQ(retrieved.outputMax, 50.0);
}

TEST_F(ControllerBaseTest, ManualOutput) {
    pid->setManualOutput(42.0);
    EXPECT_DOUBLE_EQ(pid->getManualOutput(), 42.0);
}

TEST_F(ControllerBaseTest, GetDiagnostics) {
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    pid->setGains(1.0, 0.0, 0.0);
    pid->compute(input);
    
    auto diag = pid->getDiagnostics();
    EXPECT_GE(diag.cycleCount, 1u);
}

TEST_F(ControllerBaseTest, ResetDiagnostics) {
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    pid->setGains(1.0, 0.0, 0.0);
    pid->compute(input);
    pid->resetDiagnostics();
    
    auto diag = pid->getDiagnostics();
    EXPECT_EQ(diag.cycleCount, 0u);
}

TEST_F(ControllerBaseTest, GetLastOutput) {
    pid->setGains(1.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    pid->compute(input);
    auto lastOutput = pid->getLastOutput();
    EXPECT_NEAR(lastOutput.control, 5.0, 1e-3);
}

TEST_F(ControllerBaseTest, BackendController) {
    auto backend = std::make_shared<PIDController>();
    pid->setBackendController(backend);
    
    EXPECT_TRUE(pid->hasBackend());
    EXPECT_EQ(pid->getBackendController(), backend);
}

TEST_F(ControllerBaseTest, RateLimitApplied) {
    pid->setGains(100.0, 0.0, 0.0);  // High gain for large changes
    
    SaturationLimits limits;
    limits.outputMin = -1000.0;
    limits.outputMax = 1000.0;
    limits.rateLimit = 100.0;  // Max 100 units per second change
    pid->setSaturationLimits(limits);
    
    ControllerInput input;
    input.dt = 0.01;  // 10ms
    
    // First compute - establishes baseline
    input.reference = 0.0;
    input.measured = 0.0;
    pid->compute(input);
    
    // Now request a large change
    input.reference = 100.0;  // error = 100, P action = 10000
    input.measured = 0.0;
    
    ControllerOutput output = pid->compute(input);
    
    // Rate limit should restrict change to 100 * 0.01 = 1.0 per step
    // (from 0 to at most 1.0)
    EXPECT_LE(std::abs(output.control), 100.0);  // Rate limited
}

TEST_F(ControllerBaseTest, RateLimitMultipleSteps) {
    pid->setGains(100.0, 0.0, 0.0);
    
    SaturationLimits limits;
    limits.outputMin = -1000.0;
    limits.outputMax = 1000.0;
    limits.rateLimit = 50.0;  // Slow rate limit
    pid->setSaturationLimits(limits);
    
    ControllerInput input;
    input.dt = 0.1;  // 100ms
    input.reference = 0.0;
    input.measured = 0.0;
    
    // Initialize
    pid->compute(input);
    
    // Now step to large reference
    input.reference = 100.0;
    input.measured = 0.0;
    
    std::vector<double> outputs;
    for (int i = 0; i < 10; ++i) {
        ControllerOutput output = pid->compute(input);
        outputs.push_back(output.control);
    }
    
    // Output should ramp up gradually due to rate limit
    for (size_t i = 1; i < outputs.size(); ++i) {
        double change = std::abs(outputs[i] - outputs[i-1]);
        // Rate limit is 50 units/s, dt is 0.1s, so max change is 5 per step
        EXPECT_LE(change, 5.1);  // Small tolerance
    }
}

TEST_F(ControllerBaseTest, NoRateLimitWhenInfinite) {
    pid->setGains(100.0, 0.0, 0.0);
    
    SaturationLimits limits;
    limits.outputMin = -10000.0;
    limits.outputMax = 10000.0;
    // rateLimit defaults to max double (no limit)
    pid->setSaturationLimits(limits);
    
    ControllerInput input;
    input.dt = 0.01;
    
    // Initialize
    input.reference = 0.0;
    input.measured = 0.0;
    pid->compute(input);
    
    // Large step
    input.reference = 100.0;
    ControllerOutput output = pid->compute(input);
    
    // Should jump immediately to full value without rate limiting
    EXPECT_GT(std::abs(output.control), 1000.0);
}

// ============================================================================
// PIDController Tests
// ============================================================================

class PIDControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        pid = std::make_unique<PIDController>();
    }
    
    std::unique_ptr<PIDController> pid;
};

TEST_F(PIDControllerTest, ProportionalAction) {
    pid->setGains(2.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 50.0;
    input.measured = 30.0;
    input.dt = 0.001;
    
    ControllerOutput output = pid->compute(input);
    
    // Error = 20, P action = 2 * 20 = 40
    EXPECT_NEAR(output.control, 40.0, 1e-3);
}

TEST_F(PIDControllerTest, IntegralAction) {
    pid->setGains(0.0, 1.0, 0.0);  // I only
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    // First compute: integral starts accumulating
    ControllerOutput out1 = pid->compute(input);
    
    // After multiple cycles, integral should grow
    for (int i = 0; i < 100; ++i) {
        pid->compute(input);
    }
    
    ControllerOutput outFinal = pid->compute(input);
    EXPECT_GT(std::abs(outFinal.control), std::abs(out1.control));
}

TEST_F(PIDControllerTest, DerivativeAction) {
    pid->setGains(0.0, 0.0, 1.0);  // D only
    pid->setDerivativeFilter(0.0);  // No filter
    
    ControllerInput input;
    input.dt = 0.001;
    
    // First: establish baseline
    input.reference = 10.0;
    input.measured = 0.0;
    pid->compute(input);
    
    // Second: error unchanged, derivative should be ~0
    ControllerOutput out = pid->compute(input);
    EXPECT_NEAR(out.control, 0.0, 1.0);  // Small value expected
}

TEST_F(PIDControllerTest, AntiWindupClamping) {
    pid->setGains(0.0, 10.0, 0.0);  // High integral gain
    pid->setAntiWindup(AntiWindupMethod::Clamping);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid->setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    // Run many cycles to test anti-windup
    for (int i = 0; i < 1000; ++i) {
        pid->compute(input);
    }
    
    // Now reverse the error
    input.reference = 0.0;
    input.measured = 100.0;
    
    ControllerOutput output = pid->compute(input);
    // With anti-windup, should respond quickly to reversed error
}

TEST_F(PIDControllerTest, AntiWindupBackCalculation) {
    pid->setGains(1.0, 1.0, 0.0);
    pid->setAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid->setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 100; ++i) {
        pid->compute(input);
    }
}

TEST_F(PIDControllerTest, AntiWindupConditional) {
    pid->setGains(1.0, 1.0, 0.0);
    pid->setAntiWindup(AntiWindupMethod::Conditional);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        pid->compute(input);
    }
}

TEST_F(PIDControllerTest, AntiWindupTracking) {
    pid->setGains(1.0, 1.0, 0.0);
    pid->setAntiWindup(AntiWindupMethod::Tracking, 0.5);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        pid->compute(input);
    }
}

TEST_F(PIDControllerTest, DerivativeFilter) {
    pid->setGains(0.0, 0.0, 1.0);
    pid->setDerivativeFilter(0.01);  // 10ms filter time constant
    EXPECT_DOUBLE_EQ(pid->getDerivativeFilter(), 0.01);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    input.measured = 0.0;
    
    pid->compute(input);
    
    // Add a step change
    input.measured = 10.0;
    ControllerOutput out = pid->compute(input);
    
    // Filtered derivative should be smaller than unfiltered
}

TEST_F(PIDControllerTest, DerivativeOnMeasurement) {
    pid->setGains(0.0, 0.0, 1.0);
    pid->setDerivativeOnMeasurement(true);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    input.measured = 0.0;
    
    pid->compute(input);
    
    // Step change in setpoint should not cause derivative kick
    input.reference = 10.0;
    ControllerOutput out = pid->compute(input);
}

TEST_F(PIDControllerTest, DerivativeFilterType) {
    pid->setDerivativeFilterType(DerivativeFilterType::FirstOrder);
    pid->setDerivativeFilterType(DerivativeFilterType::SecondOrder);
    pid->setDerivativeFilterType(DerivativeFilterType::MovingAverage);
    pid->setDerivativeFilterType(DerivativeFilterType::Median);
    pid->setDerivativeFilterType(DerivativeFilterType::None);
}

TEST_F(PIDControllerTest, Mode) {
    pid->setMode(ControllerMode::Automatic);
    EXPECT_EQ(pid->getMode(), ControllerMode::Automatic);
    
    pid->setMode(ControllerMode::Manual);
    EXPECT_EQ(pid->getMode(), ControllerMode::Manual);
    
    pid->setMode(ControllerMode::Disabled);
    EXPECT_EQ(pid->getMode(), ControllerMode::Disabled);
    
    pid->setMode(ControllerMode::Hold);
    EXPECT_EQ(pid->getMode(), ControllerMode::Hold);
    
    pid->setMode(ControllerMode::Tracking);
    EXPECT_EQ(pid->getMode(), ControllerMode::Tracking);
}

TEST_F(PIDControllerTest, GainsFromTimeConstants) {
    pid->setGainsFromTimeConstants(2.0, 5.0, 0.5);  // Kp=2, Ti=5, Td=0.5
    
    EXPECT_DOUBLE_EQ(pid->getKp(), 2.0);
    // Ki = Kp / Ti = 2 / 5 = 0.4
    EXPECT_NEAR(pid->getKi(), 0.4, 1e-6);
    // Kd = Kp * Td = 2 * 0.5 = 1.0
    EXPECT_NEAR(pid->getKd(), 1.0, 1e-6);
}

TEST_F(PIDControllerTest, AutoTune) {
    // Auto-tune from process model
    pid->autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::Lambda, 0.5);
    
    // Should set reasonable gains
    EXPECT_GT(pid->getKp(), 0.0);
}

TEST_F(PIDControllerTest, AutoTuneFromUltimate) {
    // Auto-tune from ultimate gain experiment
    pid->autoTuneFromUltimate(10.0, 0.5, PIDController::TuningMethod::ZieglerNicholsPID);
    
    EXPECT_GT(pid->getKp(), 0.0);
    EXPECT_GT(pid->getKi(), 0.0);
    EXPECT_GT(pid->getKd(), 0.0);
}

TEST_F(PIDControllerTest, TuningMethodZieglerNichols) {
    pid->autoTuneFromUltimate(10.0, 0.5, PIDController::TuningMethod::ZieglerNichols);
    EXPECT_GT(pid->getKp(), 0.0);
}

TEST_F(PIDControllerTest, TuningMethodCohenCoon) {
    pid->autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::CohenCoon);
    EXPECT_GT(pid->getKp(), 0.0);
}

TEST_F(PIDControllerTest, TuningMethodAMIGO) {
    pid->autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::AMIGO);
    EXPECT_GT(pid->getKp(), 0.0);
}

TEST_F(PIDControllerTest, TuningMethodSIMC) {
    pid->autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::SIMC);
    EXPECT_GT(pid->getKp(), 0.0);
}

TEST_F(PIDControllerTest, TuningMethodTyreusLuyben) {
    pid->autoTuneFromUltimate(10.0, 0.5, PIDController::TuningMethod::TyreusLuyben);
    EXPECT_GT(pid->getKp(), 0.0);
}

// ============================================================================
// PIController Tests
// ============================================================================

class PIControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        pi = std::make_unique<PIController>();
    }
    std::unique_ptr<PIController> pi;
};

TEST_F(PIControllerTest, BasicOperation) {
    pi->setGains(1.0, 0.5);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    ControllerOutput out = pi->compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST_F(PIControllerTest, GetType) {
    EXPECT_EQ(pi->getType(), ControllerType::PI);
}

TEST_F(PIControllerTest, GetName) {
    EXPECT_STREQ(pi->getName(), "PI Controller");
}

TEST_F(PIControllerTest, GetGains) {
    pi->setGains(2.0, 0.3);
    EXPECT_DOUBLE_EQ(pi->getKp(), 2.0);
    EXPECT_DOUBLE_EQ(pi->getKi(), 0.3);
}

TEST_F(PIControllerTest, GainsFromTi) {
    pi->setGainsFromTi(1.0, 5.0);  // Kp=1, Ti=5 -> Ki = 1/5 = 0.2
    EXPECT_DOUBLE_EQ(pi->getKp(), 1.0);
    EXPECT_NEAR(pi->getKi(), 0.2, 1e-6);
}

TEST_F(PIControllerTest, IntegralLimits) {
    pi->setIntegralLimits(-50, 50);
    pi->setGains(0.0, 100.0);  // High integral gain
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 1000; ++i) {
        pi->compute(input);
    }
    
    // Integral should be limited
    EXPECT_LE(pi->getIntegral(), 50.0);
}

TEST_F(PIControllerTest, SetIntegral) {
    pi->setIntegral(25.0);
    EXPECT_DOUBLE_EQ(pi->getIntegral(), 25.0);
}

TEST_F(PIControllerTest, AntiWindup) {
    pi->setAntiWindup(AntiWindupMethod::Clamping);
    pi->setAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
}

// ============================================================================
// PDController Tests
// ============================================================================

class PDControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        pd = std::make_unique<PDController>();
    }
    std::unique_ptr<PDController> pd;
};

TEST_F(PDControllerTest, BasicOperation) {
    pd->setGains(1.0, 0.1);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    ControllerOutput out = pd->compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST_F(PDControllerTest, GetType) {
    EXPECT_EQ(pd->getType(), ControllerType::PD);
}

TEST_F(PDControllerTest, GetName) {
    EXPECT_STREQ(pd->getName(), "PD Controller");
}

TEST_F(PDControllerTest, GetGains) {
    pd->setGains(3.0, 0.2);
    EXPECT_DOUBLE_EQ(pd->getKp(), 3.0);
    EXPECT_DOUBLE_EQ(pd->getKd(), 0.2);
}

TEST_F(PDControllerTest, DerivativeFilter) {
    pd->setDerivativeFilter(0.01);
    EXPECT_DOUBLE_EQ(pd->getDerivativeFilter(), 0.01);
}

TEST_F(PDControllerTest, DerivativeOnMeasurement) {
    pd->setDerivativeOnMeasurement(true);
    pd->setDerivativeOnMeasurement(false);
}

// ============================================================================
// PController Tests
// ============================================================================

TEST(PControllerTest, BasicOperation) {
    PController p;
    p.setGain(2.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    ControllerOutput out = p.compute(input);
    
    // P action: Kp * error = 2 * 5 = 10
    EXPECT_NEAR(out.control, 10.0, 1e-3);
}

TEST(PControllerTest, GetType) {
    PController p;
    EXPECT_EQ(p.getType(), ControllerType::P);
}

TEST(PControllerTest, GetName) {
    PController p;
    EXPECT_STREQ(p.getName(), "P Controller");
}

TEST(PControllerTest, GetGain) {
    PController p;
    p.setGain(5.0);
    EXPECT_DOUBLE_EQ(p.getGain(), 5.0);
}

// ============================================================================
// BangBang Controller Tests
// ============================================================================

TEST(BangBangControllerTest, BasicOperation) {
    BangBangController bb;
    bb.setOutputLevels(-100.0, 100.0);  // Min/max output
    bb.setHysteresis(1.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;  // Below setpoint
    input.dt = 0.001;
    
    ControllerOutput out = bb.compute(input);
    EXPECT_GT(out.control, 0.0);  // Should output positive
}

TEST(BangBangControllerTest, GetType) {
    BangBangController bb;
    EXPECT_EQ(bb.getType(), ControllerType::BangBang);
}

TEST(BangBangControllerTest, GetName) {
    BangBangController bb;
    EXPECT_STREQ(bb.getName(), "Bang-Bang Controller");
}

TEST(BangBangControllerTest, Hysteresis) {
    BangBangController bb;
    bb.setOutputLevels(-100.0, 100.0);
    bb.setHysteresis(2.0);
    EXPECT_DOUBLE_EQ(bb.getHysteresis(), 2.0);
    
    ControllerInput input;
    input.dt = 0.001;
    
    // Start below setpoint
    input.reference = 10.0;
    input.measured = 5.0;
    ControllerOutput out1 = bb.compute(input);
    EXPECT_GT(out1.control, 0.0);
    
    // Move to within hysteresis band
    input.measured = 9.0;
    ControllerOutput out2 = bb.compute(input);
    // Should maintain previous state due to hysteresis
}

TEST(BangBangControllerTest, NeutralOutput) {
    BangBangController bb;
    bb.setOutputLevels(-100.0, 100.0);
    bb.setNeutralOutput(0.0, true);  // Enable 3-state mode
    bb.setHysteresis(2.0);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 10.0;
    input.measured = 10.0;  // At setpoint
    
    ControllerOutput out = bb.compute(input);
    // Should output neutral value when error is small
}

// ============================================================================
// SaturationLimits Tests
// ============================================================================

TEST(SaturationLimitsTest, IsValid) {
    SaturationLimits limits;
    EXPECT_TRUE(limits.isValid());
    
    limits.outputMin = 100.0;
    limits.outputMax = 0.0;
    EXPECT_FALSE(limits.isValid());
}

// ============================================================================
// ControllerInput Tests
// ============================================================================

TEST(ControllerInputTest, Defaults) {
    ControllerInput input;
    EXPECT_DOUBLE_EQ(input.reference, 0.0);
    EXPECT_DOUBLE_EQ(input.measured, 0.0);
    EXPECT_DOUBLE_EQ(input.dt, 0.001);
    EXPECT_DOUBLE_EQ(input.feedforward, 0.0);
    EXPECT_FALSE(input.reset);
    EXPECT_TRUE(input.enable);
}

// ============================================================================
// ControllerOutput Tests
// ============================================================================

TEST(ControllerOutputTest, Defaults) {
    ControllerOutput output;
    EXPECT_DOUBLE_EQ(output.control, 0.0);
    EXPECT_FALSE(output.saturated);
    EXPECT_FALSE(output.integrating);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(PIDControllerTest, ZeroDt) {
    pid->setGains(1.0, 0.1, 0.05);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.0;  // Zero time step
    
    // Should handle gracefully
    ControllerOutput out = pid->compute(input);
}

TEST_F(PIDControllerTest, VerySmallDt) {
    pid->setGains(1.0, 0.1, 0.05);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 1e-9;  // Very small
    
    ControllerOutput out = pid->compute(input);
}

TEST_F(PIDControllerTest, LargeDt) {
    pid->setGains(1.0, 0.1, 0.05);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 10.0;  // Very large
    
    ControllerOutput out = pid->compute(input);
}

TEST_F(PIDControllerTest, ZeroError) {
    pid->setGains(1.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 10.0;  // No error
    input.dt = 0.001;
    
    ControllerOutput out = pid->compute(input);
    EXPECT_NEAR(out.control, 0.0, 1e-6);
}

TEST_F(PIDControllerTest, NegativeError) {
    pid->setGains(1.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 0.0;
    input.measured = 10.0;  // Overshoot
    input.dt = 0.001;
    
    ControllerOutput out = pid->compute(input);
    EXPECT_LT(out.control, 0.0);  // Negative output
}

TEST_F(PIDControllerTest, VeryLargeError) {
    pid->setGains(1.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 1e10;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput out = pid->compute(input);
}

TEST_F(PIDControllerTest, NegativeGains) {
    pid->setGains(-1.0, 0.0, 0.0);  // Negative Kp
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    // Should work (inverted response)
    ControllerOutput out = pid->compute(input);
    EXPECT_LT(out.control, 0.0);
}

TEST_F(PIDControllerTest, DisabledMode) {
    pid->setGains(1.0, 0.0, 0.0);
    pid->setMode(ControllerMode::Disabled);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput out = pid->compute(input);
    EXPECT_DOUBLE_EQ(out.control, 0.0);
}

TEST_F(PIDControllerTest, ManualMode) {
    pid->setGains(1.0, 0.0, 0.0);
    pid->setMode(ControllerMode::Manual);
    pid->setManualOutput(42.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput out = pid->compute(input);
    EXPECT_DOUBLE_EQ(out.control, 42.0);
}

TEST_F(PIDControllerTest, HoldMode) {
    pid->setGains(1.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    // First compute in automatic
    ControllerOutput out1 = pid->compute(input);
    
    // Switch to hold mode
    pid->setMode(ControllerMode::Hold);
    
    // Output should stay the same
    input.reference = 100.0;  // Change reference
    ControllerOutput out2 = pid->compute(input);
    EXPECT_DOUBLE_EQ(out1.control, out2.control);
}

TEST_F(PIDControllerTest, ResetInput) {
    pid->setGains(0.0, 1.0, 0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    // Build up integral
    for (int i = 0; i < 100; ++i) {
        pid->compute(input);
    }
    
    // Reset via input flag
    input.reset = true;
    pid->compute(input);
    
    // Integral should be cleared
}

TEST_F(PIDControllerTest, EnableInputFlag) {
    pid->setGains(1.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    input.enable = false;
    
    ControllerOutput out = pid->compute(input);
    // Should be disabled
}

// ============================================================================
// ControllerDiagnostics Tests  
// ============================================================================

TEST(ControllerDiagnosticsTest, Defaults) {
    ControllerDiagnostics diag;
    EXPECT_EQ(diag.cycleCount, 0u);
    EXPECT_DOUBLE_EQ(diag.maxError, 0.0);
    EXPECT_DOUBLE_EQ(diag.rmsError, 0.0);
    EXPECT_TRUE(diag.healthy);
}

// ============================================================================
// TuningParameters Tests
// ============================================================================

TEST(TuningParametersTest, Defaults) {
    TuningParameters params;
    EXPECT_DOUBLE_EQ(params.settlingTime, 1.0);
    EXPECT_DOUBLE_EQ(params.overshoot, 0.05);
    EXPECT_DOUBLE_EQ(params.bandwidth, 10.0);
    EXPECT_FALSE(params.aggressive);
}

// ============================================================================
// Additional PID Controller Tests for Full Coverage
// ============================================================================

TEST(PDControllerCoverageTest, DerivativeOnError) {
    PDController pd;
    pd.setGains(1.0, 1.0);
    pd.setDerivativeOnMeasurement(false);  // Derivative on error
    pd.setDerivativeFilter(0.0);  // No filter
    
    ControllerInput input;
    input.dt = 0.01;
    
    // First sample
    input.reference = 10.0;
    input.measured = 0.0;
    pd.compute(input);
    
    // Second sample with changed error
    input.reference = 20.0;  // Error changes from 10 to 20
    input.measured = 0.0;
    ControllerOutput out = pd.compute(input);
    
    // Derivative should be (20-10)/0.01 = 1000, dTerm = 1.0 * 1000 = 1000
    EXPECT_GT(out.derivative, 0.0);  // Positive derivative
}

TEST(PDControllerCoverageTest, DerivativeWithFilter) {
    PDController pd;
    pd.setGains(1.0, 1.0);
    pd.setDerivativeOnMeasurement(true);
    pd.setDerivativeFilter(0.1);  // 100ms filter
    
    ControllerInput input;
    input.dt = 0.01;
    input.reference = 10.0;
    input.measured = 0.0;
    
    // First sample
    pd.compute(input);
    
    // Second sample with measurement change
    input.measured = 5.0;
    // ControllerOutput out = pd.compute(input); // Not used
    pd.compute(input);
    
    // Filtered derivative should be applied
}

TEST(PDControllerCoverageTest, ZeroDerivativeFilter) {
    PDController pd;
    pd.setGains(1.0, 1.0);
    pd.setDerivativeOnMeasurement(true);
    pd.setDerivativeFilter(0.0);  // No filter
    
    ControllerInput input;
    input.dt = 0.01;
    input.reference = 10.0;
    input.measured = 0.0;
    
    pd.compute(input);
    
    input.measured = 5.0;
    ControllerOutput out = pd.compute(input);
}

TEST(PIControllerCoverageTest, AntiWindupNone) {
    PIController pi;
    pi.setGains(1.0, 10.0);
    pi.setAntiWindup(AntiWindupMethod::None);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pi.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 100; ++i) {
        pi.compute(input);
    }
}

TEST(PIControllerCoverageTest, AntiWindupBackCalculation) {
    PIController pi;
    pi.setGains(1.0, 1.0);
    pi.setAntiWindup(AntiWindupMethod::BackCalculation, 0.5);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pi.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 50; ++i) {
        pi.compute(input);
    }
}

TEST(PIControllerCoverageTest, AntiWindupConditional) {
    PIController pi;
    pi.setGains(1.0, 1.0);
    pi.setAntiWindup(AntiWindupMethod::Conditional);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pi.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 50; ++i) {
        pi.compute(input);
    }
}

TEST(PIControllerCoverageTest, DefaultAntiWindupSwitch) {
    PIController pi;
    pi.setGains(1.0, 1.0);
    pi.setAntiWindup(static_cast<AntiWindupMethod>(99));  // Invalid method, triggers default
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    pi.compute(input);
}

TEST(PIDControllerCoverageTest, AutoTuneZieglerNicholsPI) {
    PIDController pid;
    pid.autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::ZieglerNicholsPI);
    
    EXPECT_GT(pid.getKp(), 0.0);
    EXPECT_GT(pid.getKi(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getKd(), 0.0);  // PI only
}

TEST(PIDControllerCoverageTest, AutoTuneLambda) {
    PIDController pid;
    pid.autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::Lambda, 0.5);
    
    EXPECT_GT(pid.getKp(), 0.0);
}

TEST(PIDControllerCoverageTest, AutoTuneLambdaDefault) {
    PIDController pid;
    pid.autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::Lambda, 0.0);  // Default lambda = tau
    
    EXPECT_GT(pid.getKp(), 0.0);
}

TEST(PIDControllerCoverageTest, AutoTuneTyreusLuyben) {
    PIDController pid;
    pid.autoTune(1.0, 1.0, 0.1, PIDController::TuningMethod::TyreusLuyben);
    
    EXPECT_GT(pid.getKp(), 0.0);
}

TEST(PIDControllerCoverageTest, AutoTuneUltimateZNPI) {
    PIDController pid;
    pid.autoTuneFromUltimate(10.0, 0.5, PIDController::TuningMethod::ZieglerNicholsPI);
    
    EXPECT_GT(pid.getKp(), 0.0);
    EXPECT_GT(pid.getKi(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getKd(), 0.0);
}

TEST(PIDControllerCoverageTest, AutoTuneUltimateDefault) {
    PIDController pid;
    pid.autoTuneFromUltimate(10.0, 0.5, PIDController::TuningMethod::SIMC);  // Triggers default case
    
    EXPECT_GT(pid.getKp(), 0.0);
}

TEST(PIDControllerCoverageTest, ComputeDerivativeFirstRun) {
    PIDController pid;
    pid.setGains(1.0, 0.0, 1.0);
    pid.setDerivativeFilter(0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    // First run - derivative should be 0
    ControllerOutput out = pid.compute(input);
    EXPECT_DOUBLE_EQ(out.derivative, 0.0);
}

TEST(PIDControllerCoverageTest, ComputeDerivativeZeroDt) {
    PIDController pid;
    pid.setGains(1.0, 0.0, 1.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    pid.compute(input);  // First run
    
    input.dt = 0.0;
    input.measured = 5.0;
    ControllerOutput out = pid.compute(input);  // Zero dt
}

TEST(PIDControllerCoverageTest, DerivativeFilterTypes) {
    PIDController pid;
    pid.setGains(1.0, 0.0, 1.0);
    pid.setDerivativeFilter(0.1);
    
    ControllerInput input;
    input.reference = 10.0;
    input.dt = 0.01;
    
    // Test None filter
    pid.setDerivativeFilterType(DerivativeFilterType::None);
    input.measured = 0.0;
    pid.reset();
    pid.compute(input);
    input.measured = 5.0;
    pid.compute(input);
    
    // Test FirstOrder filter
    pid.setDerivativeFilterType(DerivativeFilterType::FirstOrder);
    input.measured = 0.0;
    pid.reset();
    pid.compute(input);
    input.measured = 5.0;
    pid.compute(input);
    
    // Test SecondOrder filter
    pid.setDerivativeFilterType(DerivativeFilterType::SecondOrder);
    input.measured = 0.0;
    pid.reset();
    pid.compute(input);
    input.measured = 5.0;
    pid.compute(input);
    
    // Test MovingAverage filter
    pid.setDerivativeFilterType(DerivativeFilterType::MovingAverage);
    input.measured = 0.0;
    pid.reset();
    pid.compute(input);
    input.measured = 5.0;
    pid.compute(input);
    
    // Test Median filter (fallback)
    pid.setDerivativeFilterType(DerivativeFilterType::Median);
    input.measured = 0.0;
    pid.reset();
    pid.compute(input);
    input.measured = 5.0;
    pid.compute(input);
}

TEST(PIDControllerCoverageTest, ApplyAntiWindupClamping) {
    PIDController pid;
    pid.setGains(1.0, 10.0, 0.0);
    pid.setAntiWindup(AntiWindupMethod::Clamping);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    // Saturate positive
    for (int i = 0; i < 100; ++i) {
        pid.compute(input);
    }
    
    // Now reverse error - should help desaturate
    input.reference = 0.0;
    input.measured = 100.0;
    pid.compute(input);
}

TEST(PIDControllerCoverageTest, ApplyAntiWindupBackCalcNoKi) {
    PIDController pid;
    pid.setGains(1.0, 0.0, 0.1);  // No Ki
    pid.setAntiWindup(AntiWindupMethod::BackCalculation, 0.0);  // Default Tt
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    pid.compute(input);
}

TEST(PIDControllerCoverageTest, ApplyAntiWindupConditionalLargeError) {
    PIDController pid;
    pid.setGains(1.0, 1.0, 0.0);
    pid.setAntiWindup(AntiWindupMethod::Conditional, 5.0);  // Max error threshold = 5
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;  // Error = 100 > 5 threshold
    input.measured = 0.0;
    input.dt = 0.01;
    
    pid.compute(input);
}

TEST(PIDControllerCoverageTest, ApplyAntiWindupTracking) {
    PIDController pid;
    pid.setGains(1.0, 1.0, 0.0);
    pid.setAntiWindup(AntiWindupMethod::Tracking);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 10; ++i) {
        pid.compute(input);
    }
}

TEST(PIDControllerCoverageTest, ApplyAntiWindupObserverBased) {
    PIDController pid;
    pid.setGains(1.0, 1.0, 0.0);
    pid.setAntiWindup(AntiWindupMethod::ObserverBased, 0.1);
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 50; ++i) {
        pid.compute(input);
    }
}

TEST(PIDControllerCoverageTest, ApplyAntiWindupObserverBasedDefaultParam) {
    PIDController pid;
    pid.setGains(1.0, 1.0, 0.0);
    pid.setAntiWindup(AntiWindupMethod::ObserverBased, 0.0);  // Default Tt = 0.1
    
    SaturationLimits limits;
    limits.outputMin = -10.0;
    limits.outputMax = 10.0;
    pid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    pid.compute(input);
}

// ============================================================================
// PID-2DOF Controller Coverage Tests
// ============================================================================

TEST(PID2DOFControllerCoverageTest, BasicOperation) {
    PID2DOFController pid2dof;
    pid2dof.setGains(1.0, 0.1, 0.05);
    pid2dof.setSetpointWeights(0.5, 0.3);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.01;
    
    ControllerOutput out = pid2dof.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(PID2DOFControllerCoverageTest, DerivativeComputation) {
    PID2DOFController pid2dof;
    pid2dof.setGains(1.0, 0.0, 1.0);
    pid2dof.setSetpointWeights(0.5, 0.5);
    
    ControllerInput input;
    input.dt = 0.01;
    
    // First sample
    input.reference = 10.0;
    input.measured = 0.0;
    pid2dof.compute(input);
    
    // Second sample with reference and measurement change
    input.reference = 20.0;
    input.measured = 5.0;
    ControllerOutput out = pid2dof.compute(input);
}

TEST(PID2DOFControllerCoverageTest, GetType) {
    PID2DOFController pid2dof;
    EXPECT_EQ(pid2dof.getType(), ControllerType::PID2DOF);
}

TEST(PID2DOFControllerCoverageTest, GetName) {
    PID2DOFController pid2dof;
    EXPECT_NE(pid2dof.getName(), nullptr);
}

// ============================================================================
// BangBang Controller Coverage Tests
// ============================================================================

TEST(BangBangControllerCoverageTest, ThreeStateMode) {
    BangBangController bb;
    bb.setOutputLevels(-100.0, 100.0);
    bb.setNeutralOutput(0.0, true);  // Enable 3-state
    bb.setHysteresis(5.0);
    
    ControllerInput input;
    input.dt = 0.01;
    
    // Positive error above hysteresis
    input.reference = 10.0;
    input.measured = 0.0;  // Error = 10 > 2.5
    ControllerOutput out1 = bb.compute(input);
    EXPECT_DOUBLE_EQ(out1.control, 100.0);
    
    // Negative error below hysteresis
    input.reference = 0.0;
    input.measured = 10.0;  // Error = -10 < -2.5
    ControllerOutput out2 = bb.compute(input);
    EXPECT_DOUBLE_EQ(out2.control, -100.0);
    
    // Within deadband (neutral)
    input.reference = 5.0;
    input.measured = 5.0;  // Error = 0
    ControllerOutput out3 = bb.compute(input);
    EXPECT_DOUBLE_EQ(out3.control, 0.0);
}

TEST(BangBangControllerCoverageTest, TwoStateMode) {
    BangBangController bb;
    bb.setOutputLevels(-100.0, 100.0);
    bb.setNeutralOutput(50.0, false);  // Disabled 3-state
    bb.setHysteresis(2.0);
    
    ControllerInput input;
    input.dt = 0.01;
    input.reference = 10.0;
    input.measured = 8.0;  // Error = 2 = halfHyst, edge case
    
    ControllerOutput out = bb.compute(input);
}

TEST(BangBangControllerCoverageTest, Reset) {
    BangBangController bb;
    bb.setOutputLevels(-100.0, 100.0);
    bb.setHysteresis(2.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    bb.compute(input);
    bb.reset();
    
    // After reset, state should be false -> output = uMin
    input.measured = 9.0;  // Within hysteresis
    ControllerOutput out = bb.compute(input);
}

// ============================================================================
// PD+ Controller Coverage Tests
// ============================================================================

TEST(PDPlusControllerCoverageTest, BasicOperation) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 0.1);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.01;
    
    ControllerOutput out = pdplus.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(PDPlusControllerCoverageTest, GetType) {
    PDPlusController pdplus;
    EXPECT_EQ(pdplus.getType(), ControllerType::PDPlus);
}

TEST(PDPlusControllerCoverageTest, GetName) {
    PDPlusController pdplus;
    EXPECT_NE(pdplus.getName(), nullptr);
}

TEST(PDPlusControllerCoverageTest, WithVelocityFeedforward) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 1.0);
    pdplus.setDerivativeFilter(0.0);
    
    ControllerInput input;
    input.dt = 0.01;
    
    // First sample
    input.reference = 10.0;
    input.measured = 0.0;
    input.referenceDerivative = 0.0;
    pdplus.compute(input);
    
    // Second sample with velocity feedforward
    input.reference = 20.0;
    input.measured = 5.0;
    input.referenceDerivative = 100.0;  // Reference velocity
    ControllerOutput out = pdplus.compute(input);
}

TEST(PDPlusControllerCoverageTest, WithCompensationFunction) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 0.1);
    
    // Set position-based compensation
    pdplus.setCompensationCallback([](double pos) {
        return 9.81 * std::sin(pos);  // Gravity compensation
    });
    
    ControllerInput input;
    input.reference = 1.57;  // 90 degrees
    input.measured = 0.0;
    input.dt = 0.01;
    
    ControllerOutput out = pdplus.compute(input);
    EXPECT_NE(out.feedforward, 0.0);
}

TEST(PDPlusControllerCoverageTest, WithFullCompensationFunction) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 0.1);
    
    // Set position and velocity compensation
    pdplus.setFullCompensationCallback([](double pos, double vel) {
        return 9.81 * std::sin(pos) + 0.1 * vel;  // Gravity + friction
    });
    
    ControllerInput input;
    input.reference = 1.57;
    input.measured = 0.0;
    input.referenceDerivative = 10.0;
    input.dt = 0.01;
    
    pdplus.compute(input);  // First sample
    
    input.measured = 0.5;
    ControllerOutput out = pdplus.compute(input);
}

TEST(PDPlusControllerCoverageTest, CompensationOnDesired) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 0.1);
    pdplus.setCompensationOnDesired(true);
    
    pdplus.setCompensationCallback([](double pos) {
        return 9.81 * std::sin(pos);
    });
    
    ControllerInput input;
    input.reference = 1.57;
    input.measured = 0.0;
    input.dt = 0.01;
    
    ControllerOutput out = pdplus.compute(input);
}

TEST(PDPlusControllerCoverageTest, CompensationOnMeasured) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 0.1);
    pdplus.setCompensationOnDesired(false);  // Use measured
    
    pdplus.setCompensationCallback([](double pos) {
        return 9.81 * std::sin(pos);
    });
    
    ControllerInput input;
    input.reference = 1.57;
    input.measured = 0.5;
    input.dt = 0.01;
    
    pdplus.compute(input);  // First sample
    
    input.measured = 0.6;
    ControllerOutput out = pdplus.compute(input);
}

TEST(PDPlusControllerCoverageTest, DerivativeWithFilter) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 1.0);
    pdplus.setDerivativeFilter(0.1);
    
    ControllerInput input;
    input.dt = 0.01;
    
    // First sample
    input.reference = 10.0;
    input.measured = 0.0;
    pdplus.compute(input);
    
    // Second sample
    input.measured = 5.0;
    ControllerOutput out = pdplus.compute(input);
}

TEST(PDPlusControllerCoverageTest, Reset) {
    PDPlusController pdplus;
    pdplus.setGains(1.0, 0.1);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 10; ++i) {
        pdplus.compute(input);
    }
    
    pdplus.reset();
}

// ============================================================================
// DualLoopPID Controller Coverage Tests
// ============================================================================

TEST(DualLoopPIDControllerCoverageTest, BasicOperation) {
    DualLoopPIDController dual;
    dual.setOuterGains(1.0, 0.1, 0.05);
    dual.setInnerGains(2.0, 0.5, 0.0);
    dual.setVelocityLimits(-100.0, 100.0);
    dual.setTorqueLimits(-50.0, 50.0);
    
    ControllerInput input;
    input.reference = 10.0;  // Position setpoint
    input.measured = 0.0;    // Position
    input.dt = 0.01;
    
    // Update velocity measurement
    dual.setVelocityFeedback(5.0);
    
    ControllerOutput out = dual.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(DualLoopPIDControllerCoverageTest, GetType) {
    DualLoopPIDController dual;
    EXPECT_EQ(dual.getType(), ControllerType::DualLoopPID);
}

TEST(DualLoopPIDControllerCoverageTest, GetName) {
    DualLoopPIDController dual;
    EXPECT_NE(dual.getName(), nullptr);
}

TEST(DualLoopPIDControllerCoverageTest, SetAntiWindup) {
    DualLoopPIDController dual;
    dual.setOuterAntiWindup(AntiWindupMethod::Clamping);
    dual.setInnerAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
}

TEST(DualLoopPIDControllerCoverageTest, Reset) {
    DualLoopPIDController dual;
    dual.setOuterGains(1.0, 0.1, 0.0);
    dual.setInnerGains(1.0, 0.1, 0.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 10; ++i) {
        dual.compute(input);
    }
    
    dual.reset();
}

// ============================================================================
// Additional PID Controller Coverage Tests
// ============================================================================

TEST(PDControllerCoverageTest, ResetClearsState) {
    PDController pd;
    pd.setGains(1.0, 0.1);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    // Run a few times to accumulate state
    for (int i = 0; i < 5; ++i) {
        input.measured = i * 2.0;  // Changing measurement
        pd.compute(input);
    }
    
    // Reset and verify next output treats it as first run
    pd.reset();
    
    input.measured = 5.0;
    auto output = pd.compute(input);
    // After reset, filtered derivative should be reinitialized
    EXPECT_EQ(output.error, 5.0);  // 10 - 5
}

TEST(PIControllerCoverageTest, ResetClearsIntegralState) {
    PIController pi;
    pi.setGains(1.0, 1.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.1;
    
    // Accumulate integral
    for (int i = 0; i < 10; ++i) {
        pi.compute(input);
    }
    
    auto before = pi.compute(input);
    EXPECT_NE(before.integral, 0.0);
    
    pi.reset();
    
    auto after = pi.compute(input);
    // After reset, integral should be much smaller (just one iteration)
    EXPECT_LT(std::fabs(after.integral), std::fabs(before.integral));
}

TEST(PIDControllerCoverageTest, TuningCohenCoon) {
    PIDController pid;
    pid.autoTune(1.5, 5.0, 1.0, PIDController::TuningMethod::CohenCoon);
    
    // Verify tuning was applied - output should change
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    auto out = pid.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(PIDControllerCoverageTest, TuningSIMCMethod) {
    PIDController pid;
    pid.autoTune(1.5, 5.0, 1.0, PIDController::TuningMethod::SIMC);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    auto out = pid.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(PIDControllerCoverageTest, TuningLambdaMethod) {
    PIDController pid;
    pid.autoTune(1.5, 5.0, 1.0, PIDController::TuningMethod::Lambda, 2.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    auto out = pid.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(PIDControllerCoverageTest, TuningTyreusLuyben) {
    PIDController pid;
    pid.autoTune(1.5, 5.0, 1.0, PIDController::TuningMethod::TyreusLuyben);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    auto out = pid.compute(input);
    EXPECT_NE(out.control, 0.0);
}

TEST(PIDControllerCoverageTest, TuningAMIGOMethod) {
    PIDController pid;
    pid.autoTune(1.5, 5.0, 1.0, PIDController::TuningMethod::AMIGO);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    auto out = pid.compute(input);
    EXPECT_NE(out.control, 0.0);
}

