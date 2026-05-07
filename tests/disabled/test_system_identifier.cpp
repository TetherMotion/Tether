/**
 * @file test_system_identifier.cpp
 * @brief Unit tests for SystemIdentifier class
 */

#include <gtest/gtest.h>
#include "SystemIdentifier.hpp"
#include <cmath>
#include <random>

using namespace MotionReplanner;

class SystemIdentifierTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Seed for reproducible tests
        rng.seed(42);
    }
    
    std::mt19937 rng;
};

// Test delay identification via cross-correlation
TEST_F(SystemIdentifierTest, DelayIdentification) {
    SystemIdentifier identifier;
    
    double samplePeriod = 0.001;  // 1ms
    double actualDelay = 0.005;   // 5ms = 5 samples
    
    std::vector<IdentificationSample> samples;
    
    // Generate step response with known delay
    double signalValue = 0.0;
    int stepTime = 50;  // Step occurs at sample 50
    
    for (int i = 0; i < 200; ++i) {
        double command = (i >= stepTime) ? 100.0 : 0.0;
        
        // Actual follows command with delay
        int delayedIndex = i - static_cast<int>(actualDelay / samplePeriod);
        double actual = (delayedIndex >= stepTime) ? 100.0 : 0.0;
        
        IdentificationSample sample;
        sample.timestamp = i * samplePeriod;
        sample.commandedPosition = command;
        sample.actualPosition = actual;
        sample.commandedVelocity = (i == stepTime) ? 100.0 / samplePeriod : 0.0;
        sample.actualVelocity = (delayedIndex == stepTime) ? 100.0 / samplePeriod : 0.0;
        
        samples.push_back(sample);
    }
    
    auto result = identifier.identifyDelay(samples, samplePeriod);
    
    // Should identify ~5ms delay
    EXPECT_NEAR(result.estimatedDelay, actualDelay, 0.001);
    EXPECT_GT(result.confidence, 0.9);
}

// Test delay identification with noisy signal
TEST_F(SystemIdentifierTest, DelayIdentificationNoisy) {
    SystemIdentifier identifier;
    
    double samplePeriod = 0.001;
    double actualDelay = 0.003;  // 3ms
    
    std::normal_distribution<double> noise(0.0, 1.0);
    std::vector<IdentificationSample> samples;
    
    for (int i = 0; i < 500; ++i) {
        double t = i * samplePeriod;
        
        // Sinusoidal input
        double command = 50.0 * std::sin(2.0 * M_PI * 10.0 * t);
        
        // Delayed output with noise
        int delayedIndex = i - static_cast<int>(actualDelay / samplePeriod);
        double actual = (delayedIndex >= 0) ? 
            50.0 * std::sin(2.0 * M_PI * 10.0 * delayedIndex * samplePeriod) + noise(rng) : 0.0;
        
        IdentificationSample sample;
        sample.timestamp = t;
        sample.commandedPosition = command;
        sample.actualPosition = actual;
        
        samples.push_back(sample);
    }
    
    auto result = identifier.identifyDelay(samples, samplePeriod);
    
    // Should be reasonably close even with noise
    EXPECT_NEAR(result.estimatedDelay, actualDelay, 0.002);
}

// Test friction model identification (Coulomb)
TEST_F(SystemIdentifierTest, CoulombFrictionIdentification) {
    SystemIdentifier identifier;
    
    double coulombFriction = 5.0;  // Constant friction force
    
    std::vector<IdentificationSample> samples;
    
    // Generate velocity vs force data for Coulomb friction
    std::uniform_real_distribution<double> velDist(-100.0, 100.0);
    
    for (int i = 0; i < 100; ++i) {
        double velocity = velDist(rng);
        
        // Coulomb friction: F = Fc * sign(v)
        double friction = (velocity > 0) ? coulombFriction : 
                         (velocity < 0) ? -coulombFriction : 0.0;
        
        IdentificationSample sample;
        sample.actualVelocity = velocity;
        sample.frictionForce = friction;
        
        samples.push_back(sample);
    }
    
    auto result = identifier.identifyFriction(samples, FrictionModelType::Coulomb);
    
    EXPECT_EQ(result.modelType, FrictionModelType::Coulomb);
    EXPECT_NEAR(result.params.coulombForce, coulombFriction, 0.5);
}

// Test friction model identification (Viscous)
TEST_F(SystemIdentifierTest, ViscousFrictionIdentification) {
    SystemIdentifier identifier;
    
    double viscousCoeff = 0.1;  // F = B * v
    
    std::vector<IdentificationSample> samples;
    
    std::uniform_real_distribution<double> velDist(-100.0, 100.0);
    std::normal_distribution<double> noise(0.0, 0.1);
    
    for (int i = 0; i < 100; ++i) {
        double velocity = velDist(rng);
        
        // Viscous friction: F = B * v (with small noise)
        double friction = viscousCoeff * velocity + noise(rng);
        
        IdentificationSample sample;
        sample.actualVelocity = velocity;
        sample.frictionForce = friction;
        
        samples.push_back(sample);
    }
    
    auto result = identifier.identifyFriction(samples, FrictionModelType::Viscous);
    
    EXPECT_EQ(result.modelType, FrictionModelType::Viscous);
    EXPECT_NEAR(result.params.viscousCoeff, viscousCoeff, 0.02);
}

// Test combined Coulomb + Viscous friction
TEST_F(SystemIdentifierTest, CoulombViscousFrictionIdentification) {
    SystemIdentifier identifier;
    
    double coulombFriction = 3.0;
    double viscousCoeff = 0.05;
    
    std::vector<IdentificationSample> samples;
    
    std::uniform_real_distribution<double> velDist(-100.0, 100.0);
    
    for (int i = 0; i < 200; ++i) {
        double velocity = velDist(rng);
        
        // Combined friction: F = Fc * sign(v) + B * v
        double sign = (velocity > 0) ? 1.0 : (velocity < 0) ? -1.0 : 0.0;
        double friction = coulombFriction * sign + viscousCoeff * velocity;
        
        IdentificationSample sample;
        sample.actualVelocity = velocity;
        sample.frictionForce = friction;
        
        samples.push_back(sample);
    }
    
    auto result = identifier.identifyFriction(samples, FrictionModelType::CoulombViscous);
    
    EXPECT_EQ(result.modelType, FrictionModelType::CoulombViscous);
    EXPECT_NEAR(result.params.coulombForce, coulombFriction, 1.0);
    EXPECT_NEAR(result.params.viscousCoeff, viscousCoeff, 0.01);
}

// Test PID tuning assessment
TEST_F(SystemIdentifierTest, PIDTuningAssessment) {
    SystemIdentifier identifier;
    
    std::vector<IdentificationSample> samples;
    
    // Generate step response with some overshoot and settling
    double setpoint = 100.0;
    double kp = 1.0, ki = 0.1, kd = 0.01;  // Example PID gains
    
    double position = 0.0;
    double velocity = 0.0;
    double integral = 0.0;
    double lastError = 0.0;
    double dt = 0.001;
    
    for (int i = 0; i < 2000; ++i) {
        double error = setpoint - position;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        
        double output = kp * error + ki * integral + kd * derivative;
        
        // Simple first-order dynamics
        velocity = velocity * 0.95 + output * 0.05;
        position += velocity * dt;
        
        IdentificationSample sample;
        sample.timestamp = i * dt;
        sample.commandedPosition = setpoint;
        sample.actualPosition = position;
        sample.actualVelocity = velocity;
        
        samples.push_back(sample);
        lastError = error;
    }
    
    auto result = identifier.assessPIDTuning(samples);
    
    // Should detect some overshoot and settling time
    EXPECT_GT(result.overshoot, 0.0);
    EXPECT_GT(result.settlingTime, 0.0);
    EXPECT_GT(result.riseTime, 0.0);
    EXPECT_LT(result.steadyStateError, 1.0);  // Should settle reasonably
}

// Test online delay estimator
TEST_F(SystemIdentifierTest, OnlineDelayEstimator) {
    OnlineDelayEstimator estimator(0.001, 0.02);  // 1ms sample period, 20ms max delay
    
    double actualDelay = 0.005;  // 5ms
    int delaySamples = 5;
    
    std::queue<double> commandHistory;
    
    for (int i = 0; i < 100; ++i) {
        double command = (i >= 20) ? 100.0 : 0.0;
        
        // Delayed actual
        commandHistory.push(command);
        double actual = 0.0;
        if (commandHistory.size() > static_cast<size_t>(delaySamples)) {
            while (commandHistory.size() > static_cast<size_t>(delaySamples + 1)) {
                commandHistory.pop();
            }
            actual = commandHistory.front();
        }
        
        estimator.addSample(command, actual);
    }
    
    double estimated = estimator.getEstimatedDelay();
    double confidence = estimator.getConfidence();
    
    // Should estimate delay reasonably well
    EXPECT_NEAR(estimated, actualDelay, 0.002);
    EXPECT_GT(confidence, 0.5);
}

// Test Stribeck calculator
TEST_F(SystemIdentifierTest, StribeckCalculator) {
    StribeckCalculator calc;
    
    double stribeckVelocity = 10.0;
    double staticFriction = 10.0;
    double coulombFriction = 5.0;
    double viscousCoeff = 0.02;
    
    // Generate Stribeck curve data
    std::vector<double> velocities, frictions;
    
    for (double v = 0.1; v <= 100.0; v += 1.0) {
        // Stribeck model: F = Fc + (Fs - Fc) * exp(-v/vs) + Bv
        double f = coulombFriction + 
                   (staticFriction - coulombFriction) * std::exp(-v / stribeckVelocity) +
                   viscousCoeff * v;
        
        velocities.push_back(v);
        frictions.push_back(f);
    }
    
    auto params = calc.fitStribeckCurve(velocities, frictions);
    
    EXPECT_NEAR(params.coulombForce, coulombFriction, 1.0);
    EXPECT_NEAR(params.staticFriction, staticFriction, 1.0);
    EXPECT_NEAR(params.stribeckVelocity, stribeckVelocity, 2.0);
}

// Test relay auto-tuner
TEST_F(SystemIdentifierTest, RelayAutoTuner) {
    RelayAutoTuner tuner(5.0);  // Relay amplitude
    
    // Simulate relay oscillation
    double position = 0.0;
    double velocity = 0.0;
    double setpoint = 50.0;
    double dt = 0.001;
    
    for (int i = 0; i < 5000; ++i) {
        double error = setpoint - position;
        double output = tuner.computeOutput(error);
        
        // Simple dynamics
        velocity = velocity * 0.99 + output * 0.01;
        position += velocity * dt;
        
        tuner.recordSample(i * dt, position);
    }
    
    auto result = tuner.calculatePIDGains();
    
    // Should produce some PID gains
    EXPECT_GT(result.Kp, 0.0);
    EXPECT_GT(result.Ki, 0.0);
    EXPECT_GT(result.Kd, 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
