/// @file test_destabilizer_types.cpp
/// @brief Unit tests for DestabilizerTypes (enums, config structs, defaults).

#include <gtest/gtest.h>
#include "tether/destabilizer/DestabilizerTypes.hpp"

using namespace Destabilizer;

TEST(DestabilizerTypes, ChannelConstraintsDefaults) {
    ChannelConstraints c;
    EXPECT_DOUBLE_EQ(c.amplitudeMax, 1.0);
    EXPECT_GT(c.rateMax, 0.0);
    EXPECT_GT(c.energyMax, 0.0);
    EXPECT_DOUBLE_EQ(c.freqMin, 0.0);
    EXPECT_GT(c.freqMax, 0.0);
    EXPECT_DOUBLE_EQ(c.dutyCycleMax, 1.0);
}

TEST(DestabilizerTypes, PerturbationChannelDefaults) {
    PerturbationChannel ch;
    EXPECT_EQ(ch.inputIndex, 0);
    EXPECT_TRUE(ch.name.empty());
}

TEST(DestabilizerTypes, WeightedMetricDefaults) {
    WeightedMetric wm;
    EXPECT_EQ(wm.type, MetricType::PeakStateDeviation);
    EXPECT_DOUBLE_EQ(wm.weight, 1.0);
}

TEST(DestabilizerTypes, OptimizerConfigDefaults) {
    OptimizerConfig c;
    EXPECT_EQ(c.type, OptimizerType::Adam);
    EXPECT_GT(c.learningRate, 0.0);
    EXPECT_TRUE(c.centralDifferences);
    EXPECT_GT(c.adam_beta1, 0.0);
    EXPECT_LT(c.adam_beta1, 1.0);
    EXPECT_GT(c.adam_beta2, 0.0);
    EXPECT_LT(c.adam_beta2, 1.0);
}

TEST(DestabilizerTypes, PerturbationConfigDefaults) {
    PerturbationConfig c;
    EXPECT_EQ(c.type, PerturbationType::PiecewiseConstant);
    EXPECT_GT(c.numSegments, 0);
    EXPECT_GT(c.numHarmonics, 0);
}

TEST(DestabilizerTypes, DestabilizerConfigDefaults) {
    DestabilizerConfig c;
    EXPECT_EQ(c.systemId, -1);
    EXPECT_GT(c.horizon, 0.0);
    EXPECT_GT(c.dt, 0.0);
    EXPECT_GT(c.maxIterations, 0);
    EXPECT_GT(c.seed, 0u);
}

TEST(DestabilizerTypes, IterationResultDefaults) {
    IterationResult ir;
    EXPECT_EQ(ir.iteration, 0);
    EXPECT_DOUBLE_EQ(ir.bestJ, 0.0);
    EXPECT_DOUBLE_EQ(ir.currentJ, 0.0);
}

TEST(DestabilizerTypes, DestabilizerResultDefaults) {
    DestabilizerResult r;
    EXPECT_EQ(r.verdict, Verdict::Inconclusive);
    EXPECT_DOUBLE_EQ(r.bestJ, 0.0);
    EXPECT_TRUE(r.bestTheta.empty());
    EXPECT_TRUE(r.history.empty());
}

TEST(DestabilizerTypes, VerdictEnumValues) {
    // Verify enum values exist and are distinct
    EXPECT_NE(Verdict::Destabilized, Verdict::Robust);
    EXPECT_NE(Verdict::Robust, Verdict::Inconclusive);
    EXPECT_NE(Verdict::Destabilized, Verdict::Inconclusive);
}

TEST(DestabilizerTypes, PerturbationTypeEnumValues) {
    EXPECT_NE(PerturbationType::PiecewiseConstant, PerturbationType::PiecewiseLinear);
    EXPECT_NE(PerturbationType::FourierSpectral, PerturbationType::NeuralMLP);
    EXPECT_NE(PerturbationType::ImpulseTrain, PerturbationType::BangBang);
}

TEST(DestabilizerTypes, SafeSetBoundDefaults) {
    SafeSetBound sb;
    EXPECT_EQ(sb.stateIndex, 0);
    EXPECT_LT(sb.lowerBound, 0.0);
    EXPECT_GT(sb.upperBound, 0.0);
}
