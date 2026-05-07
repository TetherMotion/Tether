/// @file test_constraint_projector.cpp
/// @brief Unit tests for constraint projection.

#include <gtest/gtest.h>
#include "tether/destabilizer/ConstraintProjector.hpp"
#include "tether/destabilizer/DestabilizerTypes.hpp"
#include <cmath>
#include <numeric>

using namespace Destabilizer;

// ---------------------------------------------------------------------------
// Static projection methods
// ---------------------------------------------------------------------------

TEST(ConstraintProjector, AmplitudeClamp) {
    std::vector<double> theta = {5.0, -3.0, 1.0, -0.5};
    ConstraintProjector::projectAmplitude(theta, 2.0);
    EXPECT_DOUBLE_EQ(theta[0], 2.0);
    EXPECT_DOUBLE_EQ(theta[1], -2.0);
    EXPECT_DOUBLE_EQ(theta[2], 1.0);
    EXPECT_DOUBLE_EQ(theta[3], -0.5);
}

TEST(ConstraintProjector, AmplitudeAlreadyInBounds) {
    std::vector<double> theta = {0.5, -0.5, 0.0};
    ConstraintProjector::projectAmplitude(theta, 1.0);
    EXPECT_DOUBLE_EQ(theta[0], 0.5);
    EXPECT_DOUBLE_EQ(theta[1], -0.5);
    EXPECT_DOUBLE_EQ(theta[2], 0.0);
}

TEST(ConstraintProjector, EnergyProjectionScales) {
    std::vector<double> theta = {10.0, 10.0};
    double segDur = 1.0;
    // Energy = 10^2*1 + 10^2*1 = 200
    ConstraintProjector::projectEnergy(theta, 50.0, segDur);
    // After: energy should be ≤ 50
    double energy = 0.0;
    for (double v : theta) energy += v * v * segDur;
    EXPECT_LE(energy, 50.0 + 1e-6);
}

TEST(ConstraintProjector, EnergyBelowLimitUnchanged) {
    std::vector<double> theta = {1.0, 1.0};
    double segDur = 1.0;
    auto original = theta;
    ConstraintProjector::projectEnergy(theta, 100.0, segDur);
    EXPECT_EQ(theta, original);
}

TEST(ConstraintProjector, RateProjection) {
    std::vector<double> theta = {0.0, 100.0}; // Big jump
    double segDur = 1.0;
    double rateMax = 5.0;
    ConstraintProjector::projectRate(theta, rateMax, segDur);
    // |theta[1] - theta[0]| should be ≤ rateMax * segDur = 5
    EXPECT_LE(std::abs(theta[1] - theta[0]), rateMax * segDur + 1e-6);
}

TEST(ConstraintProjector, FrequencyProjection) {
    // Fourier: θ = [a1, f1, φ1, a2, f2, φ2, ...]
    std::vector<double> theta = {1.0, 0.5, 0.0, 1.0, 200.0, 0.0};
    ConstraintProjector::projectFrequency(theta, 1.0, 100.0);
    // f1 should be clamped to [1, 100]: 0.5 → 1.0
    EXPECT_GE(theta[1], 1.0);
    // f2 should be clamped: 200 → 100
    EXPECT_LE(theta[4], 100.0);
}

TEST(ConstraintProjector, DutyCycleProjection) {
    std::vector<double> theta = {1.0, 1.0, 1.0, 1.0, 1.0};
    // 100% active, dMax = 0.4 → only 40% should remain nonzero
    ConstraintProjector::projectDutyCycle(theta, 0.4, 5.0);
    int nonzero = 0;
    for (double v : theta) if (std::abs(v) > 1e-10) nonzero++;
    EXPECT_LE(nonzero, 2); // 40% of 5 = 2
}

TEST(ConstraintProjector, IsAmplitudeActive) {
    std::vector<double> theta = {1.0, 2.0, 3.0};
    EXPECT_FALSE(ConstraintProjector::isAmplitudeActive(theta, 5.0));
    EXPECT_TRUE(ConstraintProjector::isAmplitudeActive(theta, 3.0));
}

// ---------------------------------------------------------------------------
// Configured projector
// ---------------------------------------------------------------------------

TEST(ConstraintProjector, ConfigureAndProject) {
    ConstraintProjector proj;
    ChannelConstraints c;
    c.amplitudeMax = 2.0;
    c.rateMax = 1e6;
    c.energyMax = 1e6;

    PerturbationConfig pc;
    pc.type = PerturbationType::PiecewiseConstant;
    pc.numSegments = 4;

    proj.configure(c, pc, 4.0, 0.001);

    std::vector<double> theta = {5.0, -3.0, 1.0, -0.5};
    auto active = proj.project(theta);
    EXPECT_EQ(active.size(), 5u);
    EXPECT_TRUE(active[0]); // Amplitude was clamped
    EXPECT_DOUBLE_EQ(theta[0], 2.0);
    EXPECT_DOUBLE_EQ(theta[1], -2.0);
}

TEST(ConstraintProjector, IdempotenceProperty) {
    ConstraintProjector proj;
    ChannelConstraints c;
    c.amplitudeMax = 1.5;
    c.rateMax = 10.0;
    c.energyMax = 5.0;

    PerturbationConfig pc;
    pc.type = PerturbationType::PiecewiseConstant;
    pc.numSegments = 8;

    proj.configure(c, pc, 4.0, 0.001);

    std::vector<double> theta = {3.0, -2.0, 5.0, -1.0, 0.5, 4.0, -3.0, 2.0};
    EXPECT_TRUE(proj.verifyIdempotence(theta));
}

TEST(ConstraintProjector, EmptyTheta) {
    ConstraintProjector proj;
    ChannelConstraints c;
    PerturbationConfig pc;
    pc.type = PerturbationType::PiecewiseConstant;
    proj.configure(c, pc, 5.0, 0.001);

    std::vector<double> theta;
    auto active = proj.project(theta);
    EXPECT_EQ(active.size(), 5u);
    EXPECT_TRUE(theta.empty());
}

TEST(ConstraintProjector, FourierFreqConstraint) {
    ConstraintProjector proj;
    ChannelConstraints c;
    c.amplitudeMax = 100.0;
    c.freqMin = 0.5;
    c.freqMax = 50.0;

    PerturbationConfig pc;
    pc.type = PerturbationType::FourierSpectral;
    pc.numHarmonics = 2;

    proj.configure(c, pc, 5.0, 0.001);

    // θ = [a1, f1, φ1, a2, f2, φ2]
    std::vector<double> theta = {1.0, 0.01, 0.0, 1.0, 200.0, 0.0};
    proj.project(theta);
    EXPECT_GE(theta[1], 0.5 - 1e-6);
    EXPECT_LE(theta[4], 50.0 + 1e-6);
}
