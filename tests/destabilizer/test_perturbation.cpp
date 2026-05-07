/// @file test_perturbation.cpp
/// @brief Unit tests for perturbation signal parameterizations.

#include <gtest/gtest.h>
#include "tether/destabilizer/Perturbation.hpp"
#include "tether/destabilizer/DestabilizerTypes.hpp"
#include <cmath>
#include <numeric>

using namespace Destabilizer;

// ---------------------------------------------------------------------------
// PiecewiseConstant
// ---------------------------------------------------------------------------

TEST(PiecewiseConstantPerturbation, ParameterCount) {
    PiecewiseConstantPerturbation p(10, 5.0);
    EXPECT_EQ(p.parameterCount(), 10);
}

TEST(PiecewiseConstantPerturbation, EvaluateMiddleOfSegment) {
    PiecewiseConstantPerturbation p(4, 4.0); // 4 segments, each 1s
    std::vector<double> theta = {1.0, 2.0, 3.0, 4.0};
    EXPECT_DOUBLE_EQ(p.evaluate(0.5, theta), 1.0);
    EXPECT_DOUBLE_EQ(p.evaluate(1.5, theta), 2.0);
    EXPECT_DOUBLE_EQ(p.evaluate(2.5, theta), 3.0);
    EXPECT_DOUBLE_EQ(p.evaluate(3.5, theta), 4.0);
}

TEST(PiecewiseConstantPerturbation, BoundaryHandling) {
    PiecewiseConstantPerturbation p(2, 2.0);
    std::vector<double> theta = {1.0, -1.0};
    // t=0 should be segment 0
    EXPECT_DOUBLE_EQ(p.evaluate(0.0, theta), 1.0);
    // t=2.0 (at horizon exactly) clamps to last segment
    EXPECT_DOUBLE_EQ(p.evaluate(2.0, theta), -1.0);
    // Negative t returns 0
    EXPECT_DOUBLE_EQ(p.evaluate(-0.5, theta), 0.0);
}

TEST(PiecewiseConstantPerturbation, SingleSegment) {
    PiecewiseConstantPerturbation p(1, 10.0);
    std::vector<double> theta = {42.0};
    EXPECT_EQ(p.parameterCount(), 1);
    EXPECT_DOUBLE_EQ(p.evaluate(5.0, theta), 42.0);
}

TEST(PiecewiseConstantPerturbation, AllZeroTheta) {
    PiecewiseConstantPerturbation p(5, 5.0);
    std::vector<double> theta(5, 0.0);
    for (double t = 0.0; t < 5.0; t += 0.1) {
        EXPECT_DOUBLE_EQ(p.evaluate(t, theta), 0.0);
    }
}

// ---------------------------------------------------------------------------
// PiecewiseLinear
// ---------------------------------------------------------------------------

TEST(PiecewiseLinearPerturbation, ParameterCount) {
    PiecewiseLinearPerturbation p(5, 10.0);
    EXPECT_EQ(p.parameterCount(), 5);
}

TEST(PiecewiseLinearPerturbation, InterpolatesBetweenPoints) {
    PiecewiseLinearPerturbation p(3, 2.0); // 3 breakpoints at t=0, 1, 2
    std::vector<double> theta = {0.0, 2.0, 0.0};
    // At midpoint between breakpoints 0 and 1
    EXPECT_NEAR(p.evaluate(0.5, theta), 1.0, 0.01);
    // At breakpoint 1
    EXPECT_NEAR(p.evaluate(1.0, theta), 2.0, 0.01);
    // At midpoint between breakpoints 1 and 2
    EXPECT_NEAR(p.evaluate(1.5, theta), 1.0, 0.01);
}

TEST(PiecewiseLinearPerturbation, ConstantSignal) {
    PiecewiseLinearPerturbation p(4, 3.0);
    std::vector<double> theta(4, 5.0);
    for (double t = 0.0; t < 3.0; t += 0.25) {
        EXPECT_NEAR(p.evaluate(t, theta), 5.0, 1e-10);
    }
}

// ---------------------------------------------------------------------------
// Fourier
// ---------------------------------------------------------------------------

TEST(FourierPerturbation, ParameterCount) {
    FourierPerturbation f(3, 5.0);
    EXPECT_EQ(f.parameterCount(), 9); // 3 harmonics * 3 (amp, freq, phase)
}

TEST(FourierPerturbation, ZeroAmplitudes) {
    FourierPerturbation f(2, 5.0);
    std::vector<double> theta = {0.0, 1.0, 0.0, 0.0, 2.0, 0.0};
    // All amplitudes zero → signal is zero
    for (double t = 0.0; t < 5.0; t += 0.1) {
        EXPECT_DOUBLE_EQ(f.evaluate(t, theta), 0.0);
    }
}

TEST(FourierPerturbation, SingleSinusoid) {
    FourierPerturbation f(1, 10.0);
    double amplitude = 2.0;
    double freq = 1.0;
    double phase = 0.0;
    std::vector<double> theta = {amplitude, freq, phase};
    // u(t) = 2.0 * sin(2π * 1.0 * t + 0) = 2.0 * sin(2πt)
    EXPECT_NEAR(f.evaluate(0.0, theta), 0.0, 1e-10);
    EXPECT_NEAR(f.evaluate(0.25, theta), 2.0, 1e-10);
    EXPECT_NEAR(f.evaluate(0.5, theta), 0.0, 1e-10);
    EXPECT_NEAR(f.evaluate(0.75, theta), -2.0, 1e-10);
}

TEST(FourierPerturbation, PhaseShift) {
    FourierPerturbation f(1, 10.0);
    double pi = M_PI;
    std::vector<double> theta = {1.0, 1.0, pi / 2.0};
    // u(t) = sin(2πt + π/2) = cos(2πt)
    EXPECT_NEAR(f.evaluate(0.0, theta), 1.0, 1e-10);
    EXPECT_NEAR(f.evaluate(0.25, theta), 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// ImpulseTrain
// ---------------------------------------------------------------------------

TEST(ImpulseTrainPerturbation, ParameterCount) {
    ImpulseTrainPerturbation p(4, 10.0);
    EXPECT_EQ(p.parameterCount(), 12); // 4 impulses * 3 (time, amp, dur)
}

TEST(ImpulseTrainPerturbation, NoImpulses) {
    ImpulseTrainPerturbation p(0, 5.0);
    // min(1, 0) = 1 internally, so 3 parameters
    EXPECT_EQ(p.parameterCount(), 3);
    // Zero-amplitude impulse: output zero
    std::vector<double> theta = {0.5, 0.0, 0.1};
    EXPECT_DOUBLE_EQ(p.evaluate(1.0, theta), 0.0);
}

TEST(ImpulseTrainPerturbation, SingleImpulse) {
    ImpulseTrainPerturbation p(1, 10.0);
    // time=0.5, amplitude=3.0, duration=0.1 (normalized to horizon)
    std::vector<double> theta = {0.5, 3.0, 0.1};
    // Evaluate within the impulse window
    double val = p.evaluate(5.0, theta); // t=5.0, normalized time = 0.5
    EXPECT_NE(val, 0.0); // Should be active
}

// ---------------------------------------------------------------------------
// BangBang
// ---------------------------------------------------------------------------

TEST(BangBangPerturbation, ParameterCount) {
    BangBangPerturbation p(5, 10.0);
    EXPECT_EQ(p.parameterCount(), 5);
}

TEST(BangBangPerturbation, AlternatesSigns) {
    BangBangPerturbation p(2, 10.0);
    // Two switching times at t=3 and t=7 (normalized: 0.3, 0.7)
    std::vector<double> theta = {0.3, 0.7};
    // Before first switch: +1
    EXPECT_DOUBLE_EQ(p.evaluate(1.0, theta), 1.0);
    // Between switches: -1
    EXPECT_DOUBLE_EQ(p.evaluate(5.0, theta), -1.0);
    // After second switch: +1
    EXPECT_DOUBLE_EQ(p.evaluate(8.0, theta), 1.0);
}

TEST(BangBangPerturbation, NoSwitches) {
    BangBangPerturbation p(0, 10.0);
    std::vector<double> theta;
    // Constant +1
    EXPECT_DOUBLE_EQ(p.evaluate(5.0, theta), 1.0);
}

// ---------------------------------------------------------------------------
// MLP
// ---------------------------------------------------------------------------

TEST(MLPPerturbation, ParameterCount) {
    MLPPerturbation mlp(2, 4, 1); // 2 inputs, 4 hidden, 1 layer
    EXPECT_GT(mlp.parameterCount(), 0);
}

TEST(MLPPerturbation, ZeroWeightsGiveZero) {
    MLPPerturbation mlp(2, 4, 1);
    std::vector<double> theta(mlp.parameterCount(), 0.0);
    std::vector<double> state = {1.0, 2.0};
    // Zero weights → zero output (tanh(0) = 0 throughout)
    EXPECT_DOUBLE_EQ(mlp.evaluate(0.0, theta, state), 0.0);
}

TEST(MLPPerturbation, NonzeroOutput) {
    MLPPerturbation mlp(2, 4, 1);
    std::vector<double> theta(mlp.parameterCount(), 0.1);
    std::vector<double> state = {1.0, 2.0};
    double val = mlp.evaluate(0.0, theta, state);
    // Should produce some non-zero output
    EXPECT_NE(val, 0.0);
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

TEST(PerturbationFactory, CreateAllTypes) {
    PerturbationConfig config;
    config.numSegments = 10;
    config.numHarmonics = 3;
    config.numImpulses = 4;
    config.mlpHiddenSize = 8;
    config.mlpNumLayers = 1;

    auto pc = PerturbationSignal::create(PerturbationType::PiecewiseConstant, config, 5.0);
    ASSERT_NE(pc, nullptr);
    EXPECT_EQ(pc->parameterCount(), 10);

    auto pl = PerturbationSignal::create(PerturbationType::PiecewiseLinear, config, 5.0);
    ASSERT_NE(pl, nullptr);
    EXPECT_EQ(pl->parameterCount(), 10);

    auto ff = PerturbationSignal::create(PerturbationType::FourierSpectral, config, 5.0);
    ASSERT_NE(ff, nullptr);
    EXPECT_EQ(ff->parameterCount(), 9);

    auto mlp = PerturbationSignal::create(PerturbationType::NeuralMLP, config, 5.0);
    ASSERT_NE(mlp, nullptr);
    EXPECT_GT(mlp->parameterCount(), 0);

    auto it = PerturbationSignal::create(PerturbationType::ImpulseTrain, config, 5.0);
    ASSERT_NE(it, nullptr);
    EXPECT_EQ(it->parameterCount(), 12);

    auto bb = PerturbationSignal::create(PerturbationType::BangBang, config, 5.0);
    ASSERT_NE(bb, nullptr);
    EXPECT_EQ(bb->parameterCount(), 10);
}

TEST(PerturbationFactory, GenerateSignal) {
    PiecewiseConstantPerturbation p(4, 4.0);
    std::vector<double> theta = {1.0, -1.0, 1.0, -1.0};
    std::vector<double> times = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    auto signal = p.generateSignal(times, theta);
    EXPECT_EQ(signal.size(), times.size());
    EXPECT_DOUBLE_EQ(signal[0], 1.0);
    EXPECT_DOUBLE_EQ(signal[1], 1.0);
    EXPECT_DOUBLE_EQ(signal[2], -1.0);
    EXPECT_DOUBLE_EQ(signal[3], -1.0);
}
