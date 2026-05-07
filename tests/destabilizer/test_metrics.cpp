/// @file test_metrics.cpp
/// @brief Unit tests for instability metrics.

#include <gtest/gtest.h>
#include "tether/destabilizer/InstabilityMetrics.hpp"
#include "tether/destabilizer/DestabilizerTypes.hpp"
#include <cmath>
#include <numeric>

using namespace Destabilizer;

// ---------------------------------------------------------------------------
// Test helper: create a simple trajectory
// ---------------------------------------------------------------------------

static RolloutTrajectory makeConstantTrajectory(int steps, int stateDim,
                                                  double dt,
                                                  double stateValue = 0.0) {
    RolloutTrajectory traj;
    traj.dt = dt;
    traj.stateDim = stateDim;
    traj.outputDim = stateDim;
    traj.inputDim = 1;
    traj.referenceState.assign(stateDim, 0.0);

    for (int i = 0; i < steps; ++i) {
        traj.times.push_back(i * dt);
        traj.states.push_back(std::vector<double>(stateDim, stateValue));
        traj.outputs.push_back(std::vector<double>(stateDim, stateValue));
        traj.controlInputs.push_back({0.0});
    }
    return traj;
}

static RolloutTrajectory makeDivergingTrajectory(int steps, double dt,
                                                   double rate) {
    RolloutTrajectory traj;
    traj.dt = dt;
    traj.stateDim = 1;
    traj.outputDim = 1;
    traj.inputDim = 1;
    traj.referenceState = {0.0};

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double val = std::exp(rate * t);
        traj.times.push_back(t);
        traj.states.push_back({val});
        traj.outputs.push_back({val});
        traj.controlInputs.push_back({0.0});
    }
    return traj;
}

static RolloutTrajectory makeOscillatingTrajectory(int steps, double dt,
                                                     double amplitude, double freq) {
    RolloutTrajectory traj;
    traj.dt = dt;
    traj.stateDim = 1;
    traj.outputDim = 1;
    traj.inputDim = 1;
    traj.referenceState = {0.0};

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double val = amplitude * std::sin(2.0 * M_PI * freq * t);
        traj.times.push_back(t);
        traj.states.push_back({val});
        traj.outputs.push_back({val});
        traj.controlInputs.push_back({0.0});
    }
    return traj;
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

TEST(MetricUtils, VectorNorm) {
    EXPECT_DOUBLE_EQ(vectorNorm({3.0, 4.0}), 5.0);
    EXPECT_DOUBLE_EQ(vectorNorm({0.0}), 0.0);
    EXPECT_DOUBLE_EQ(vectorNorm({}), 0.0);
}

TEST(MetricUtils, VectorDiff) {
    auto d = vectorDiff({3.0, 5.0}, {1.0, 2.0});
    EXPECT_DOUBLE_EQ(d[0], 2.0);
    EXPECT_DOUBLE_EQ(d[1], 3.0);
}

TEST(MetricUtils, VectorDiffEmpty) {
    auto d = vectorDiff({1.0}, {});
    EXPECT_EQ(d.size(), 0u); // min(1, 0) = 0
}

// ---------------------------------------------------------------------------
// Peak State Deviation
// ---------------------------------------------------------------------------

TEST(PeakStateDeviation, ZeroDeviation) {
    auto traj = makeConstantTrajectory(100, 2, 0.01, 0.0);
    PeakStateDeviationMetric m;
    EXPECT_DOUBLE_EQ(m.compute(traj), 0.0);
}

TEST(PeakStateDeviation, NonzeroDeviation) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    PeakStateDeviationMetric m;
    EXPECT_DOUBLE_EQ(m.compute(traj), 5.0);
}

TEST(PeakStateDeviation, DivergingTrajectory) {
    auto traj = makeDivergingTrajectory(1000, 0.01, 1.0);
    PeakStateDeviationMetric m;
    double val = m.compute(traj);
    // exp(1.0 * 9.99) ≈ 21898
    EXPECT_GT(val, 100.0);
}

// ---------------------------------------------------------------------------
// Terminal State Deviation
// ---------------------------------------------------------------------------

TEST(TerminalStateDeviation, ZeroDeviation) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    TerminalStateDeviationMetric m;
    EXPECT_DOUBLE_EQ(m.compute(traj), 0.0);
}

TEST(TerminalStateDeviation, NonzeroDeviation) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 3.0);
    TerminalStateDeviationMetric m;
    EXPECT_DOUBLE_EQ(m.compute(traj), 3.0);
}

// ---------------------------------------------------------------------------
// ISE
// ---------------------------------------------------------------------------

TEST(ISE, ZeroError) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    ISEMetric m;
    EXPECT_NEAR(m.compute(traj), 0.0, 1e-10);
}

TEST(ISE, ConstantError) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 2.0);
    ISEMetric m;
    double val = m.compute(traj);
    // ∫ (2.0)² dt from 0 to ~1s = 4.0 * 1.0 = 4.0 (approximately)
    EXPECT_NEAR(val, 4.0 * 0.99, 0.1); // 99 intervals * 0.01
}

// ---------------------------------------------------------------------------
// IAE
// ---------------------------------------------------------------------------

TEST(IAE, ZeroError) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    IAEMetric m;
    EXPECT_NEAR(m.compute(traj), 0.0, 1e-10);
}

TEST(IAE, ConstantError) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 2.0);
    IAEMetric m;
    double val = m.compute(traj);
    // ∫ |2.0| dt ≈ 2.0 * 0.99 ≈ 1.98
    EXPECT_NEAR(val, 2.0 * 0.99, 0.1);
}

// ---------------------------------------------------------------------------
// Time-Weighted ISE
// ---------------------------------------------------------------------------

TEST(TimeWeightedISE, ZeroError) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    ITSEMetric m;
    EXPECT_NEAR(m.compute(traj), 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// Overshoot
// ---------------------------------------------------------------------------

TEST(Overshoot, NoOvershoot) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    OvershootMetric m;
    EXPECT_DOUBLE_EQ(m.compute(traj), 0.0);
}

// ---------------------------------------------------------------------------
// Energy Injected
// ---------------------------------------------------------------------------

TEST(EnergyInjected, ZeroEnergy) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    EnergyInjectedMetric m;
    EXPECT_NEAR(m.compute(traj), 0.0, 1e-10);
}

TEST(EnergyInjected, ConstantState) {
    auto traj = makeConstantTrajectory(100, 1, 0.01, 3.0);
    EnergyInjectedMetric m;
    double val = m.compute(traj);
    // ∫ ||x||² dt = 9.0 * 0.99 ≈ 8.91
    EXPECT_GT(val, 0.0);
}

// ---------------------------------------------------------------------------
// Oscillation / spectral metrics
// ---------------------------------------------------------------------------

TEST(FFTMagnitude, BasicSinusoid) {
    // Generate a pure sinusoid at 10 Hz, dt=0.001, 1024 samples
    double dt = 0.001;
    int N = 1024;
    double freq = 10.0;
    std::vector<double> signal(N);
    for (int i = 0; i < N; ++i) {
        signal[i] = std::sin(2.0 * M_PI * freq * i * dt);
    }
    auto mag = computeFFTMagnitude(signal, dt);
    EXPECT_GT(mag.size(), 0u);
    // Peak should be around index corresponding to 10 Hz
}

// ---------------------------------------------------------------------------
// CombinedMetricEvaluator
// ---------------------------------------------------------------------------

TEST(CombinedMetricEvaluator, EmptyMetrics) {
    CombinedMetricEvaluator eval;
    eval.configure({}, {}, -1e6, 1e6);
    auto traj = makeConstantTrajectory(100, 1, 0.01, 1.0);
    EXPECT_DOUBLE_EQ(eval.evaluate(traj), 0.0);
}

TEST(CombinedMetricEvaluator, SingleMetric) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    eval.configure({wm}, {}, -1e6, 1e6);

    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    EXPECT_DOUBLE_EQ(eval.evaluate(traj), 5.0);
}

TEST(CombinedMetricEvaluator, WeightedCombination) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm1, wm2;
    wm1.type = MetricType::PeakStateDeviation;
    wm1.weight = 2.0;
    wm2.type = MetricType::TerminalStateDeviation;
    wm2.weight = 1.0;
    eval.configure({wm1, wm2}, {}, -1e6, 1e6);

    auto traj = makeConstantTrajectory(100, 1, 0.01, 3.0);
    // J = 2.0*3.0 + 1.0*3.0 = 9.0
    EXPECT_DOUBLE_EQ(eval.evaluate(traj), 9.0);
}

TEST(CombinedMetricEvaluator, Breakdown) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm1, wm2;
    wm1.type = MetricType::PeakStateDeviation;
    wm1.weight = 1.0;
    wm2.type = MetricType::TerminalStateDeviation;
    wm2.weight = 1.0;
    eval.configure({wm1, wm2}, {}, -1e6, 1e6);

    auto traj = makeConstantTrajectory(100, 1, 0.01, 4.0);
    auto breakdown = eval.evaluateBreakdown(traj);
    ASSERT_EQ(breakdown.size(), 2u);
    EXPECT_DOUBLE_EQ(breakdown[0], 4.0);
    EXPECT_DOUBLE_EQ(breakdown[1], 4.0);
}

TEST(CombinedMetricEvaluator, MetricCount) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    eval.configure({wm, wm, wm}, {}, -1e6, 1e6);
    EXPECT_EQ(eval.metricCount(), 3);
}

// ---------------------------------------------------------------------------
// createMetric factory
// ---------------------------------------------------------------------------

TEST(CreateMetric, AllMetricTypes) {
    // Test that all metric types can be created without crashing
    std::vector<MetricType> types = {
        MetricType::PeakStateDeviation,
        MetricType::TerminalStateDeviation,
        MetricType::IntegratedSquaredError,
        MetricType::IntegratedAbsoluteError,
        MetricType::TimeWeightedISE,
        MetricType::ExponentialDivergenceRate,
        MetricType::ControlSaturationTime,
        MetricType::OscillationAmplitudeGrowth,
        MetricType::LimitCycleEscapeCount,
        MetricType::PhaseSpaceVolumeExpansion,
        MetricType::SpectralRadiusSensitivity,
        MetricType::SettlingTimeViolation,
        MetricType::OvershootMagnitude,
        MetricType::EnergyInjected,
        MetricType::TimeToInstability,
        MetricType::ControllerBandwidthExceedance,
        MetricType::CovarianceGrowth,
        MetricType::NonlinearDistortion,
    };

    for (auto t : types) {
        WeightedMetric wm;
        wm.type = t;
        wm.weight = 1.0;
        auto metric = createMetric(wm);
        ASSERT_NE(metric, nullptr) << "Failed to create metric type " << static_cast<int>(t);

        // Compute on a simple trajectory — should not crash
        auto traj = makeOscillatingTrajectory(256, 0.01, 1.0, 5.0);
        double val = metric->compute(traj);
        EXPECT_TRUE(std::isfinite(val) || val >= 0.0)
            << "Metric type " << static_cast<int>(t) << " returned " << val;
    }
}

// ---------------------------------------------------------------------------
// Additional coverage tests
// ---------------------------------------------------------------------------

TEST(MetricUtils, VectorNormInf) {
    EXPECT_DOUBLE_EQ(vectorNormInf({3.0, -4.0, 2.0}), 4.0);
    EXPECT_DOUBLE_EQ(vectorNormInf({0.0}), 0.0);
    EXPECT_DOUBLE_EQ(vectorNormInf({}), 0.0);
}

TEST(ROAEscapeMetric, NoViolation) {
    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = -10.0;
    b.upperBound = 10.0;
    ROAEscapeMetric m({b});
    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    EXPECT_DOUBLE_EQ(m.compute(traj), 0.0);
}

TEST(ROAEscapeMetric, UpperViolation) {
    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = -1.0;
    b.upperBound = 1.0;
    ROAEscapeMetric m({b});
    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    double val = m.compute(traj);
    EXPECT_GT(val, 0.0); // 5.0 > 1.0 upper bound
}

TEST(ROAEscapeMetric, LowerViolation) {
    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = 0.0;
    b.upperBound = 10.0;
    ROAEscapeMetric m({b});
    auto traj = makeConstantTrajectory(100, 1, 0.01, -3.0);
    double val = m.compute(traj);
    EXPECT_GT(val, 0.0); // -3.0 < 0.0 lower bound
}

TEST(ConstraintViolationMetric, NoViolation) {
    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = -10.0;
    b.upperBound = 10.0;
    ConstraintViolationMetric m({b});
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    EXPECT_DOUBLE_EQ(m.compute(traj), 0.0);
}

TEST(ConstraintViolationMetric, Violation) {
    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = -1.0;
    b.upperBound = 1.0;
    ConstraintViolationMetric m({b});
    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    double val = m.compute(traj);
    EXPECT_GT(val, 0.0);
}

TEST(ControlSaturationMetric, NoSaturation) {
    ControlSaturationMetric m(-100.0, 100.0);
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    double val = m.compute(traj);
    EXPECT_NEAR(val, 0.0, 1e-6);
}

TEST(TimeToInstabilityMetric, StableTrajectory) {
    TimeToInstabilityMetric m(100.0);
    auto traj = makeConstantTrajectory(100, 1, 0.01, 0.0);
    double val = m.compute(traj);
    // Stable trajectory: metric should be 0
    EXPECT_NEAR(val, 0.0, 1e-6);
}

TEST(TimeToInstabilityMetric, DivergingTrajectory) {
    TimeToInstabilityMetric m(10.0);
    auto traj = makeDivergingTrajectory(500, 0.01, 1.0);
    double val = m.compute(traj);
    EXPECT_GT(val, 0.0); // Should detect instability
}

TEST(BandwidthExceedanceMetric, LowFreqSignal) {
    BandwidthExceedanceMetric m(100.0);
    auto traj = makeOscillatingTrajectory(256, 0.001, 1.0, 5.0); // 5 Hz, bandwidth 100 Hz
    double val = m.compute(traj);
    EXPECT_GE(val, 0.0);
}

TEST(CombinedMetricEvaluator, WithSafeSet) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm;
    wm.type = MetricType::RegionOfAttractionEscape;
    wm.weight = 1.0;

    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = -1.0;
    b.upperBound = 1.0;

    eval.configure({wm}, {b}, -1e6, 1e6);
    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    double val = eval.evaluate(traj);
    EXPECT_GT(val, 0.0); // Violates safe set
}

TEST(CombinedMetricEvaluator, WithControlSaturation) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm;
    wm.type = MetricType::ControlSaturationTime;
    wm.weight = 1.0;

    eval.configure({wm}, {}, -0.1, 0.1);
    auto traj = makeOscillatingTrajectory(100, 0.01, 1.0, 5.0);
    double val = eval.evaluate(traj);
    EXPECT_GE(val, 0.0);
}

TEST(CombinedMetricEvaluator, WithConstraintViolation) {
    CombinedMetricEvaluator eval;
    WeightedMetric wm;
    wm.type = MetricType::ConstraintViolationIntegral;
    wm.weight = 1.0;

    SafeSetBound b;
    b.stateIndex = 0;
    b.lowerBound = -1.0;
    b.upperBound = 1.0;

    eval.configure({wm}, {b}, -1e6, 1e6);
    auto traj = makeConstantTrajectory(100, 1, 0.01, 5.0);
    double val = eval.evaluate(traj);
    EXPECT_GT(val, 0.0);
}
