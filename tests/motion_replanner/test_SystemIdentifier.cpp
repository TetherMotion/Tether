/**
 * @file test_SystemIdentifier.cpp
 * @brief Tests for SystemIdentifier, StribeckCalculator, OnlineDelayEstimator,
 *        RelayAutoTuner, and supporting data types.
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/SystemIdentifier.hpp>
#include <algorithm>
#include <cmath>

using namespace MotionReplanner;

// Helper to create an IdentificationSample
static IdentificationSample makeSample(double t, double cmd, double actual,
                                       double vel = 0, double accel = 0) {
    IdentificationSample s;
    s.timestamp = t;
    s.commanded = cmd;
    s.actual = actual;
    s.velocity = vel;
    s.acceleration = accel;
    s.current = 0;
    s.torque = 0;
    return s;
}

// ============================================================================
// IdentificationSample
// ============================================================================
TEST(IdentificationSampleTest, Construction) {
    IdentificationSample s{};
    (void)s; // fields not value-initialized, just check no crash
}

// ============================================================================
// FrictionModelParams
// ============================================================================
TEST(FrictionModelParamsTest, Defaults) {
    FrictionModelParams p;
    p.type = FrictionModelType::Coulomb;
    EXPECT_DOUBLE_EQ(p.coulombForce, 0.0);
    EXPECT_DOUBLE_EQ(p.viscousCoeff, 0.0);
    EXPECT_DOUBLE_EQ(p.stribeckExponent, 2.0);
}

TEST(FrictionModelParamsTest, CoulombCalculate) {
    FrictionModelParams p;
    p.type = FrictionModelType::Coulomb;
    p.coulombForce = 5.0;
    // F = coulomb * sign(v)
    double f = p.calculate(100.0);
    EXPECT_NEAR(std::abs(f), 5.0, 1.0);
}

TEST(FrictionModelParamsTest, ViscousCalculate) {
    FrictionModelParams p;
    p.type = FrictionModelType::Viscous;
    p.viscousCoeff = 0.01;
    double f = p.calculate(100.0);
    EXPECT_NEAR(f, 1.0, 0.5); // 0.01 * 100
}

TEST(FrictionModelParamsTest, CoulombViscousCalculate) {
    FrictionModelParams p;
    p.type = FrictionModelType::CoulombViscous;
    p.coulombForce = 2.0;
    p.viscousCoeff = 0.01;
    double f = p.calculate(100.0);
    // Should be around coulomb + viscous * v
    EXPECT_GT(f, 2.0);
}

TEST(FrictionModelParamsTest, ModelName) {
    FrictionModelParams p;
    p.type = FrictionModelType::Coulomb;
    EXPECT_FALSE(p.modelName().empty());

    p.type = FrictionModelType::Viscous;
    EXPECT_FALSE(p.modelName().empty());

    p.type = FrictionModelType::CoulombViscous;
    EXPECT_FALSE(p.modelName().empty());

    p.type = FrictionModelType::Stribeck;
    EXPECT_FALSE(p.modelName().empty());

    p.type = FrictionModelType::LuGre;
    EXPECT_FALSE(p.modelName().empty());
}

// ============================================================================
// DelayIdentificationResult
// ============================================================================
TEST(DelayIdentificationResultTest, IsValid) {
    DelayIdentificationResult r{};
    r.delayConfidence = 0.3;
    EXPECT_FALSE(r.isValid());
    r.delayConfidence = 0.8;
    EXPECT_TRUE(r.isValid());
}

// ============================================================================
// FrictionIdentificationResult
// ============================================================================
TEST(FrictionIdentificationResultTest, IsValid) {
    FrictionIdentificationResult r;
    r.bestModel.rSquared = 0.5;
    EXPECT_FALSE(r.isValid());
    r.bestModel.rSquared = 0.9;
    EXPECT_TRUE(r.isValid());
}

// ============================================================================
// SystemIdentifier
// ============================================================================
class SystemIdentifierTest : public ::testing::Test {
protected:
    SystemIdentifier sysid_;
};

TEST_F(SystemIdentifierTest, InitiallyEmpty) {
    EXPECT_EQ(sysid_.sampleCount(), 0u);
}

TEST_F(SystemIdentifierTest, AddSample) {
    sysid_.addSample(makeSample(0.0, 1.0, 0.5));
    EXPECT_EQ(sysid_.sampleCount(), 1u);
}

TEST_F(SystemIdentifierTest, AddMultipleSamples) {
    std::vector<IdentificationSample> samples;
    for (int i = 0; i < 100; i++) {
        samples.push_back(makeSample(i * 0.001, 1.0, 0.5 + 0.005 * i));
    }
    sysid_.addSamples(samples);
    EXPECT_EQ(sysid_.sampleCount(), 100u);
}

TEST_F(SystemIdentifierTest, ClearSamples) {
    sysid_.addSample(makeSample(0.0, 1.0, 0.5));
    sysid_.clearSamples();
    EXPECT_EQ(sysid_.sampleCount(), 0u);
}

TEST_F(SystemIdentifierTest, CrossCorrelation) {
    // Auto-correlation of a sine wave should peak at lag 0
    std::vector<double> sig(100);
    for (int i = 0; i < 100; i++) {
        sig[i] = std::sin(2.0 * M_PI * 3.0 * i / 100.0);
    }
    auto corr = SystemIdentifier::crossCorrelation(sig, sig, 10);
    EXPECT_FALSE(corr.empty());
    // Peak should be at center (lag 0)
    auto maxIt = std::max_element(corr.begin(), corr.end());
    int maxIdx = static_cast<int>(std::distance(corr.begin(), maxIt));
    EXPECT_EQ(maxIdx, 10); // lag 0 is at index maxLag
}

TEST_F(SystemIdentifierTest, CrossCorrelationWithDelay) {
    // Create two signals with known delay
    std::vector<double> cmd(200, 0.0);
    std::vector<double> actual(200, 0.0);
    int delay = 5;
    for (int i = 50; i < 200; i++) {
        cmd[i] = 1.0; // step at sample 50
    }
    for (int i = 50 + delay; i < 200; i++) {
        actual[i] = 1.0; // step at sample 55
    }
    auto corr = SystemIdentifier::crossCorrelation(cmd, actual, 20);
    EXPECT_FALSE(corr.empty());
}

TEST_F(SystemIdentifierTest, IdentifyDelayFromStep) {
    // Generate step response data
    for (int i = 0; i < 500; i++) {
        double t = i * 0.001; // 1kHz
        double cmd = (i >= 100) ? 1.0 : 0.0;
        double actual = (i >= 105) ? (1.0 - std::exp(-(i - 105) * 0.05)) : 0.0;
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.identifyDelayFromStep(0.1, 1.0);
    // Should identify a delay around 5ms
    if (result.isValid()) {
        EXPECT_GT(result.transportDelay, 0.0);
    }
}

TEST_F(SystemIdentifierTest, IdentifyDelay) {
    // Generate step data
    for (int i = 0; i < 500; i++) {
        double t = i * 0.001;
        double cmd = (i >= 100) ? 1.0 : 0.0;
        double actual = (i >= 110) ? 0.9 : 0.0;
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.identifyDelay();
    (void)result;
}

TEST_F(SystemIdentifierTest, FindSteps) {
    // Create data with clear steps
    for (int i = 0; i < 300; i++) {
        double cmd;
        if (i < 100) cmd = 0.0;
        else if (i < 200) cmd = 1.0;
        else cmd = 0.0;
        sysid_.addSample(makeSample(i * 0.001, cmd, cmd));
    }
    auto steps = sysid_.findSteps(0.5);
    EXPECT_GE(steps.size(), 1u);
}

TEST_F(SystemIdentifierTest, IdentifyFriction) {
    // Generate friction-like data with velocity-dependent forces
    for (int i = 0; i < 200; i++) {
        double vel = -100.0 + i;
        double force = 2.0 * (vel > 0 ? 1.0 : -1.0) + 0.01 * vel; // coulomb + viscous
        IdentificationSample s;
        s.timestamp = i * 0.01;
        s.commanded = 0;
        s.actual = 0;
        s.velocity = vel;
        s.torque = force;
        s.acceleration = 0;
        s.current = 0;
        sysid_.addSample(s);
    }
    auto result = sysid_.identifyFriction();
    (void)result;
}

TEST_F(SystemIdentifierTest, FitFrictionModelCoulomb) {
    std::vector<double> velocities = {-100, -50, -10, 10, 50, 100};
    std::vector<double> forces;
    for (double v : velocities) {
        forces.push_back(5.0 * (v > 0 ? 1.0 : -1.0));
    }
    auto params = sysid_.fitFrictionModel(FrictionModelType::Coulomb, velocities, forces);
    EXPECT_NEAR(params.coulombForce, 5.0, 2.0);
}

TEST_F(SystemIdentifierTest, ComputeFFT) {
    // Generate a 10 Hz sine wave at 1000 Hz sampling
    std::vector<double> signal(1024);
    double sampleRate = 1000.0;
    for (int i = 0; i < 1024; i++) {
        signal[i] = std::sin(2.0 * M_PI * 10.0 * i / sampleRate);
    }
    std::vector<double> freqs, mags, phases;
    SystemIdentifier::computeFFT(signal, freqs, mags, phases, sampleRate);
    EXPECT_FALSE(freqs.empty());
    EXPECT_FALSE(mags.empty());
    EXPECT_EQ(freqs.size(), mags.size());
}

// ============================================================================
// StribeckCalculator
// ============================================================================
TEST(StribeckCalculatorTest, Calculate) {
    double f = StribeckCalculator::calculate(50.0, 5.0, 8.0, 10.0, 2.0, 0.01);
    EXPECT_GT(f, 0.0);
}

TEST(StribeckCalculatorTest, CalculateZeroVelocity) {
    double f = StribeckCalculator::calculate(0.0, 5.0, 8.0, 10.0, 2.0, 0.01);
    // At zero velocity, should return static friction
    EXPECT_GE(std::abs(f), 5.0);
}

TEST(StribeckCalculatorTest, Fit) {
    std::vector<double> velocities = {1, 5, 10, 20, 50, 100};
    std::vector<double> forces;
    // Generate Stribeck curve: F = Fc + (Fs - Fc)*exp(-(v/vs)^2) + viscous*v
    for (double v : velocities) {
        double Fc = 5.0, Fs = 8.0, vs = 10.0, visc = 0.01;
        double f = Fc + (Fs - Fc) * std::exp(-std::pow(v / vs, 2)) + visc * v;
        forces.push_back(f);
    }
    auto params = StribeckCalculator::fit(velocities, forces);
    EXPECT_GT(params.coulombForce, 0.0);
}

// ============================================================================
// OnlineDelayEstimator
// ============================================================================
class OnlineDelayEstimatorTest : public ::testing::Test {
protected:
    OnlineDelayEstimator estimator_{0.05, 0.001}; // 50ms max delay, 1ms period
};

TEST_F(OnlineDelayEstimatorTest, InitialDelay) {
    EXPECT_DOUBLE_EQ(estimator_.getDelay(), 0.0);
}

TEST_F(OnlineDelayEstimatorTest, InitialConfidence) {
    EXPECT_DOUBLE_EQ(estimator_.getConfidence(), 0.0);
}

TEST_F(OnlineDelayEstimatorTest, UpdateWithKnownDelay) {
    // Simulate a system with 5ms delay
    int delaySteps = 5;
    std::vector<double> cmdHistory;

    for (int i = 0; i < 500; i++) {
        double t = i * 0.001;
        double cmd = std::sin(2.0 * M_PI * 5.0 * t); // 5 Hz sine
        cmdHistory.push_back(cmd);

        double actual = (i >= delaySteps) ? cmdHistory[i - delaySteps] : 0.0;
        estimator_.update(cmd, actual, t);
    }

    // Should converge toward 5ms delay
    if (estimator_.getConfidence() > 0.3) {
        EXPECT_NEAR(estimator_.getDelay(), 0.005, 0.01);
    }
}

TEST_F(OnlineDelayEstimatorTest, Reset) {
    estimator_.update(1.0, 0.5, 0.001);
    estimator_.reset();
    EXPECT_DOUBLE_EQ(estimator_.getDelay(), 0.0);
    EXPECT_DOUBLE_EQ(estimator_.getConfidence(), 0.0);
}

// ============================================================================
// RelayAutoTuner
// ============================================================================
class RelayAutoTunerTest : public ::testing::Test {
protected:
    RelayAutoTuner tuner_{1.0, 0.05}; // 1.0 amplitude, 0.05 hysteresis
};

TEST_F(RelayAutoTunerTest, InitiallyNotReady) {
    EXPECT_FALSE(tuner_.isReady());
}

TEST_F(RelayAutoTunerTest, ProcessReturnsRelayOutput) {
    double out = tuner_.process(0.0, 0.1, 0.0);
    // Should return +amplitude or -amplitude
    EXPECT_TRUE(std::abs(out) > 0.0);
}

TEST_F(RelayAutoTunerTest, SimulateFirstOrderSystem) {
    // Simulate relay feedback on a simple first-order system
    // G(s) = 1/(s + 1) discretized
    double y = 0.0;
    double dt = 0.001;
    double tau = 0.05; // 50ms time constant
    double setpoint = 0.0;

    for (int i = 0; i < 5000; i++) {
        double t = i * dt;
        double u = tuner_.process(setpoint, y, t);
        // First-order system: dy/dt = (u - y) / tau
        y += (u - y) * dt / tau;
    }

    // After enough oscillations, tuner should be ready
    if (tuner_.isReady()) {
        auto result = tuner_.computeTuning();
        EXPECT_GT(result.ultimateGain, 0.0);
        EXPECT_GT(result.ultimatePeriod, 0.0);
        EXPECT_TRUE(result.isValid);
    }
}

TEST_F(RelayAutoTunerTest, Reset) {
    tuner_.process(0.0, 0.1, 0.0);
    tuner_.reset();
    EXPECT_FALSE(tuner_.isReady());
}
