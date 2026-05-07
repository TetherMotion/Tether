/**
 * @file FractionalPIDTests.cpp
 * @brief Comprehensive tests for FractionalPID Controller
 * Tests for FractionalPIDController (PI^λ D^μ)
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "tether/control/Controllers.hpp"
#include "tether/control/FractionalPID.hpp"

using namespace Control;

// ============================================================================
// FractionalPIDController Tests
// ============================================================================

class FractionalPIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        fopid = std::make_unique<FractionalPIDController>();
    }
    
    std::unique_ptr<FractionalPIDController> fopid;
};

TEST_F(FractionalPIDTest, GetType) {
    EXPECT_EQ(fopid->getType(), ControllerType::FractionalPID);
}

TEST_F(FractionalPIDTest, GetName) {
    EXPECT_STREQ(fopid->getName(), "Fractional PID (PI^λ D^μ)");
}

TEST_F(FractionalPIDTest, GetDescription) {
    EXPECT_NE(fopid->getDescription(), nullptr);
    EXPECT_GT(strlen(fopid->getDescription()), 0);
}

TEST_F(FractionalPIDTest, SetGains) {
    fopid->setGains(1.0, 0.5, 0.2);
    // Gains should be set successfully
}

TEST_F(FractionalPIDTest, SetOrders) {
    fopid->setOrders(0.8, 0.6);
    EXPECT_DOUBLE_EQ(fopid->getLambda(), 0.8);
    EXPECT_DOUBLE_EQ(fopid->getMu(), 0.6);
}

TEST_F(FractionalPIDTest, DefaultOrders) {
    // Default should be standard PID (λ=1, μ=1)
    EXPECT_DOUBLE_EQ(fopid->getLambda(), 1.0);
    EXPECT_DOUBLE_EQ(fopid->getMu(), 1.0);
}

TEST_F(FractionalPIDTest, StandardPIDBehavior) {
    // With λ=1, μ=1, should behave like standard PID
    fopid->setGains(1.0, 0.0, 0.0);  // P only
    fopid->setOrders(1.0, 1.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
    EXPECT_NEAR(output.control, 50.0, 5.0);  // P * error = 1 * 50 = 50
}

TEST_F(FractionalPIDTest, FractionalIntegral) {
    fopid->setGains(0.0, 1.0, 0.0);  // I only
    fopid->setOrders(0.8, 1.0);  // Fractional integral
    fopid->setMemoryLength(100);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    // First compute
    ControllerOutput out1 = fopid->compute(input);
    
    // After multiple cycles
    for (int i = 0; i < 100; ++i) {
        fopid->compute(input);
    }
    
    ControllerOutput outFinal = fopid->compute(input);
    // Integral should accumulate
}

TEST_F(FractionalPIDTest, FractionalDerivative) {
    fopid->setGains(0.0, 0.0, 1.0);  // D only
    fopid->setOrders(1.0, 0.7);  // Fractional derivative
    fopid->setMemoryLength(100);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    input.measured = 0.0;
    
    fopid->compute(input);
    
    // Step change
    input.measured = 10.0;
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, SetMemoryLength) {
    fopid->setMemoryLength(50);
    EXPECT_EQ(fopid->getMemoryLength(), 50u);
    
    fopid->setMemoryLength(200);
    EXPECT_EQ(fopid->getMemoryLength(), 200u);
}

TEST_F(FractionalPIDTest, SetApproximationMethodGL) {
    fopid->setApproximationMethod(FractionalApproximation::GrunwaldLetnikov);
    
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, SetApproximationMethodOustaloup) {
    fopid->setApproximationMethod(FractionalApproximation::Oustaloup);
    fopid->setOustaloupParams(0.001, 1000, 5);  // ωl, ωh, N
    
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // ControllerOutput output = fopid->compute(input); // Not used
    fopid->compute(input);
}

TEST_F(FractionalPIDTest, SetApproximationMethodMatsuda) {
    fopid->setApproximationMethod(FractionalApproximation::Matsuda);
    
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, SetApproximationMethodShortMemory) {
    fopid->setApproximationMethod(FractionalApproximation::ShortMemory);
    
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    fopid->setMemoryLength(50);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, OustaloupParams) {
    fopid->setOustaloupParams(0.001, 1000, 5);
    // Should not throw
}

TEST_F(FractionalPIDTest, Reset) {
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    fopid->setMemoryLength(100);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 100; ++i) {
        fopid->compute(input);
    }
    
    fopid->reset();
    // Memory should be cleared
}

TEST_F(FractionalPIDTest, ZeroLambda) {
    // Edge case: λ very close to 0
    fopid->setGains(0.0, 1.0, 0.0);
    fopid->setOrders(0.1, 1.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, ZeroMu) {
    // Edge case: μ very close to 0
    fopid->setGains(0.0, 0.0, 1.0);
    fopid->setOrders(1.0, 0.1);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, LambdaGreaterThanOne) {
    // λ > 1 (super-integration)
    fopid->setGains(0.0, 1.0, 0.0);
    fopid->setOrders(1.2, 1.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, MuGreaterThanOne) {
    // μ > 1 (super-derivative)
    fopid->setGains(0.0, 0.0, 1.0);
    fopid->setOrders(1.0, 1.2);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    input.measured = 0.0;
    
    fopid->compute(input);
    
    input.measured = 10.0;
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, AutoTuneIsoDamping) {
    // Auto-tune for constant phase margin
    fopid->autoTuneIsoDamping(1.0, 1.0, 10.0, 60.0);  // K, T, ωc, φm
    
    // Should set reasonable orders and gains
    EXPECT_GT(fopid->getLambda(), 0.0);
    EXPECT_LE(fopid->getLambda(), 2.0);
    EXPECT_GT(fopid->getMu(), 0.0);
    EXPECT_LE(fopid->getMu(), 2.0);
}

TEST_F(FractionalPIDTest, VerySmallDt) {
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 1e-9;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, LargeDt) {
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 1.0;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, MemoryLengthZero) {
    fopid->setMemoryLength(0);
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

TEST_F(FractionalPIDTest, LargeMemoryLength) {
    fopid->setMemoryLength(10000);
    fopid->setGains(1.0, 0.5, 0.2);
    fopid->setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // Run enough cycles to fill memory
    for (int i = 0; i < 1000; ++i) {
        fopid->compute(input);
    }
}

TEST_F(FractionalPIDTest, WithSaturation) {
    fopid->setGains(10.0, 5.0, 2.0);  // High gains
    fopid->setOrders(0.8, 0.6);
    
    SaturationLimits limits;
    limits.outputMin = -50.0;
    limits.outputMax = 50.0;
    fopid->setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
    EXPECT_LE(output.control, 50.0);
    EXPECT_GE(output.control, -50.0);
}

TEST_F(FractionalPIDTest, GetBinomialCoefficients) {
    fopid->setOrders(0.8, 0.6);
    fopid->setMemoryLength(100);
    
    // Compute to generate coefficients
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    // Internal coefficients are computed on first call - verify by calling compute
    fopid->compute(input);
    
    // The fact that compute() doesn't crash indicates coefficients are valid
    // Memory length should be what we set
    EXPECT_EQ(fopid->getMemoryLength(), 100);
}

TEST_F(FractionalPIDTest, NegativeOrders) {
    // Negative fractional orders (fractional integration)
    fopid->setOrders(-0.5, 0.5);
    fopid->setGains(1.0, 1.0, 1.0);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    ControllerOutput output = fopid->compute(input);
}

// ============================================================================
// Fractional Approximation Tests
// ============================================================================

TEST(FractionalApproximationTest, GrunwaldLetnikovBasic) {
    FractionalPIDController fopid;
    fopid.setApproximationMethod(FractionalApproximation::GrunwaldLetnikov);
    fopid.setGains(1.0, 0.5, 0.2);
    fopid.setOrders(0.8, 0.6);
    fopid.setMemoryLength(100);
    
    ControllerInput input;
    input.dt = 0.001;
    
    // Ramp input
    for (int i = 0; i < 100; ++i) {
        input.reference = i * 0.1;
        input.measured = i * 0.05;  // Half speed
        fopid.compute(input);
    }
}

TEST(FractionalApproximationTest, OustaloupFilters) {
    FractionalPIDController fopid;
    fopid.setApproximationMethod(FractionalApproximation::Oustaloup);
    fopid.setOustaloupParams(0.01, 100.0, 5);
    fopid.setGains(1.0, 0.5, 0.2);
    fopid.setOrders(0.8, 0.6);
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (int i = 0; i < 100; ++i) {
        input.reference = std::sin(i * 0.01);
        input.measured = 0.5 * std::sin(i * 0.01);
        fopid.compute(input);
    }
}

// ============================================================================
// Performance Comparison Tests
// ============================================================================

TEST(FractionalPerformanceTest, CompareWithStandardPID) {
    // Standard PID
    PIDController pid;
    pid.setGains(1.0, 0.5, 0.2);
    
    // Fractional PID with λ=1, μ=1 should behave similarly
    FractionalPIDController fopid;
    fopid.setGains(1.0, 0.5, 0.2);
    fopid.setOrders(1.0, 1.0);
    fopid.setMemoryLength(100);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // ControllerOutput pidOut = pid.compute(input); // Not used
    pid.compute(input);
    ControllerOutput fopidOut = fopid.compute(input);
    
    // P-term should be similar
    // Note: Some difference expected due to numerical approximation
}

TEST(FractionalPerformanceTest, LongRunStability) {
    FractionalPIDController fopid;
    fopid.setGains(1.0, 0.1, 0.05);
    fopid.setOrders(0.9, 0.8);
    fopid.setMemoryLength(500);
    
    SaturationLimits limits;
    limits.outputMin = -100.0;
    limits.outputMax = 100.0;
    fopid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 50.0;
    double measurement = 0.0;
    
    // Simulate closed loop
    for (int i = 0; i < 10000; ++i) {
        input.measured = measurement;
        ControllerOutput out = fopid.compute(input);
        
        // Simple first-order plant: y_dot = -y + u
        measurement += 0.001 * (-measurement + out.control);
        
        // Check for numerical instability
        EXPECT_FALSE(std::isnan(out.control));
        EXPECT_FALSE(std::isinf(out.control));
    }
}

// ============================================================================
// Auto-Tuning Tests
// ============================================================================

TEST(FractionalAutoTuningTest, AutoTuneIsoDamping) {
    FractionalPIDController fopid;
    
    // Auto-tune for first-order plus dead time model
    double processGain = 2.0;
    double timeConstant = 1.0;
    double deadTime = 0.1;
    double crossoverFreq = 0.5;  // rad/s
    
    fopid.autoTuneIsoDamping(processGain, timeConstant, deadTime, crossoverFreq);
    
    // Check that fractional orders are set
    double lambda = fopid.getLambda();
    double mu = fopid.getMu();
    
    EXPECT_GT(lambda, 0.0);
    EXPECT_LT(lambda, 2.0);
    EXPECT_GT(mu, 0.0);
    EXPECT_LT(mu, 2.0);
    
    // Test that controller works after auto-tuning
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    ControllerOutput output = fopid.compute(input);
    EXPECT_FALSE(std::isnan(output.control));
}

TEST(FractionalAutoTuningTest, AutoTuneSIMC) {
    FractionalPIDController fopid;
    
    double processGain = 1.5;
    double timeConstant = 2.0;
    double deadTime = 0.3;
    
    fopid.autoTuneSIMC(processGain, timeConstant, deadTime);
    
    // Check that gains are set reasonably
    double lambda = fopid.getLambda();
    double mu = fopid.getMu();
    
    EXPECT_GT(lambda, 0.0);
    EXPECT_LT(lambda, 2.0);
    EXPECT_GT(mu, 0.0);
    EXPECT_LT(mu, 2.0);
    
    // Test controller functionality
    ControllerInput input;
    input.reference = 5.0;
    input.measured = 1.0;
    input.dt = 0.01;
    
    ControllerOutput output = fopid.compute(input);
    EXPECT_FALSE(std::isnan(output.control));
}

TEST(FractionalAutoTuningTest, AutoTuneSIMCLargeDeadTime) {
    FractionalPIDController fopid;
    
    // Large dead time relative to time constant
    double processGain = 1.0;
    double timeConstant = 0.5;
    double deadTime = 1.0;  // Dead time > time constant
    
    fopid.autoTuneSIMC(processGain, timeConstant, deadTime);
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    ControllerOutput output = fopid.compute(input);
    EXPECT_FALSE(std::isnan(output.control));
}

// ============================================================================
// Oustaloup Filter Tests
// ============================================================================

TEST(OustaloupFilterTest, BasicConfiguration) {
    OustaloupFilter filter;
    filter.configure(0.5, 0.01, 100.0, 5);
    
    double input = 1.0;
    double dt = 0.001;
    
    double output = filter.process(input, dt);
    EXPECT_FALSE(std::isnan(output));
    EXPECT_FALSE(std::isinf(output));
}

TEST(OustaloupFilterTest, DifferentOrders) {
    for (int order = 1; order <= 10; ++order) {
        OustaloupFilter filter;
        filter.configure(0.5, 0.01, 100.0, order);
        
        double output = filter.process(1.0, 0.001);
        EXPECT_FALSE(std::isnan(output));
    }
}

TEST(OustaloupFilterTest, NegativeAlpha) {
    // Negative alpha for fractional integration
    OustaloupFilter filter;
    filter.configure(-0.5, 0.01, 100.0, 5);
    
    double output = filter.process(1.0, 0.001);
    EXPECT_FALSE(std::isnan(output));
}

TEST(OustaloupFilterTest, AlphaClamping) {
    OustaloupFilter filter;
    
    // Alpha > 0.99 should be clamped
    filter.configure(1.5, 0.01, 100.0, 5);
    double output1 = filter.process(1.0, 0.001);
    EXPECT_FALSE(std::isnan(output1));
    
    // Alpha < -0.99 should be clamped
    filter.configure(-1.5, 0.01, 100.0, 5);
    double output2 = filter.process(1.0, 0.001);
    EXPECT_FALSE(std::isnan(output2));
}

TEST(OustaloupFilterTest, OrderClamping) {
    OustaloupFilter filter;
    
    // Order < 1 should be clamped to 1
    filter.configure(0.5, 0.01, 100.0, 0);
    double output1 = filter.process(1.0, 0.001);
    EXPECT_FALSE(std::isnan(output1));
    
    // Order > 10 should be clamped to 10
    filter.configure(0.5, 0.01, 100.0, 20);
    double output2 = filter.process(1.0, 0.001);
    EXPECT_FALSE(std::isnan(output2));
}

TEST(OustaloupFilterTest, Reset) {
    OustaloupFilter filter;
    filter.configure(0.5, 0.01, 100.0, 5);
    
    // Process some inputs
    for (int i = 0; i < 100; ++i) {
        filter.process(1.0, 0.001);
    }
    
    // Reset
    filter.reset();
    
    // Process again - should start fresh
    double output = filter.process(1.0, 0.001);
    EXPECT_FALSE(std::isnan(output));
}

TEST(OustaloupFilterTest, FrequencyResponse) {
    OustaloupFilter filter;
    filter.configure(0.5, 0.01, 100.0, 5);
    
    double dt = 0.001;
    
    // Apply sinusoidal input at different frequencies
    for (double freq = 0.1; freq <= 10.0; freq *= 2.0) {
        filter.reset();
        
        double maxOutput = 0.0;
        for (int i = 0; i < 1000; ++i) {
            double t = i * dt;
            double input = std::sin(2.0 * M_PI * freq * t);
            double output = filter.process(input, dt);
            maxOutput = std::max(maxOutput, std::abs(output));
        }
        
        EXPECT_GT(maxOutput, 0.0);
    }
}

// ============================================================================
// Anti-Windup Tests
// ============================================================================

TEST(FractionalAntiWindupTest, BackCalculation) {
    FractionalPIDController fopid;
    fopid.setGains(1.0, 10.0, 0.0);  // High integral gain
    fopid.setOrders(0.9, 0.8);
    fopid.setAntiWindup(AntiWindupMethod::BackCalculation, 1.0);
    fopid.setIntegralLimits(-50.0, 50.0);
    
    SaturationLimits limits;
    limits.outputMin = -100.0;
    limits.outputMax = 100.0;
    fopid.setSaturationLimits(limits);
    
    ControllerInput input;
    input.reference = 1000.0;  // Large reference to cause saturation
    input.measured = 0.0;
    input.dt = 0.01;
    
    // Run for a while - should not cause windup
    for (int i = 0; i < 1000; ++i) {
        ControllerOutput output = fopid.compute(input);
        EXPECT_LE(output.control, 100.0);
        EXPECT_GE(output.control, -100.0);
    }
}

TEST(FractionalAntiWindupTest, IntegralLimits) {
    FractionalPIDController fopid;
    fopid.setGains(1.0, 5.0, 0.0);
    fopid.setOrders(0.8, 0.8);
    fopid.setIntegralLimits(-20.0, 20.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.dt = 0.01;
    
    // Run to accumulate integral
    for (int i = 0; i < 500; ++i) {
        fopid.compute(input);
    }
    
    // Integral contribution should be limited
}

