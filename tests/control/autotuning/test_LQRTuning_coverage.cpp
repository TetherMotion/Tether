/**
 * @file test_LQRTuning_coverage.cpp
 * @brief Extended coverage tests for LQRTuning: DataDrivenUtils, isCompatible(),
 *        IFT iteration loop, RegionalPolePlacement, FRI custom optimizer, VRF, LTR
 */

#include "tether/control/autotuning/LQRTuning.hpp"
#include "tether/control/autotuning/OptimizationAlgorithms.hpp"
#include "TestHelpers.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <memory>

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

// ============================================================================
// DataDrivenUtils - pure utility functions
// ============================================================================

TEST(DataDrivenUtilsCovTest, FilterSimple) {
    std::vector<double> signal = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> num = {0.5, 0.5};
    std::vector<double> den = {1.0};
    auto result = DataDrivenUtils::filter(signal, num, den);
    EXPECT_EQ(result.size(), signal.size());
}

TEST(DataDrivenUtilsCovTest, FilterIdentity) {
    std::vector<double> signal = {1.0, 2.0, 3.0};
    std::vector<double> num = {1.0};
    std::vector<double> den = {1.0};
    auto result = DataDrivenUtils::filter(signal, num, den);
    EXPECT_EQ(result.size(), 3u);
    for (size_t i = 0; i < signal.size(); i++) {
        EXPECT_NEAR(result[i], signal[i], 1e-10);
    }
}

TEST(DataDrivenUtilsCovTest, FilterWithFeedback) {
    // Simple low-pass: y[n] = 0.5*x[n] + 0.5*y[n-1]
    std::vector<double> signal = {1.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> num = {0.5};
    std::vector<double> den = {1.0, -0.5};
    auto result = DataDrivenUtils::filter(signal, num, den);
    EXPECT_EQ(result.size(), 5u);
    EXPECT_NEAR(result[0], 0.5, 1e-10);
}

TEST(DataDrivenUtilsCovTest, FilterHighOrder) {
    // Higher order filter: 3 numerator, 3 denominator
    std::vector<double> signal(20, 1.0);
    std::vector<double> num = {0.25, 0.5, 0.25};
    std::vector<double> den = {1.0, -0.5, 0.1};
    auto result = DataDrivenUtils::filter(signal, num, den);
    EXPECT_EQ(result.size(), 20u);
}

TEST(DataDrivenUtilsCovTest, Derivative) {
    // Linear ramp: derivative should be constant
    std::vector<double> signal = {0.0, 1.0, 2.0, 3.0, 4.0};
    double dt = 1.0;
    auto result = DataDrivenUtils::derivative(signal, dt);
    EXPECT_EQ(result.size(), signal.size());
    // Interior values should be ~1.0 (central difference)
    for (size_t i = 1; i < result.size() - 1; i++) {
        EXPECT_NEAR(result[i], 1.0, 0.1);
    }
}

TEST(DataDrivenUtilsCovTest, DerivativeQuadratic) {
    // Quadratic: y = t^2 => dy/dt = 2t
    std::vector<double> signal;
    double dt = 0.01;
    for (int i = 0; i < 100; i++) {
        double t = i * dt;
        signal.push_back(t * t);
    }
    auto result = DataDrivenUtils::derivative(signal, dt);
    EXPECT_EQ(result.size(), signal.size());
    // At t=0.5 (index 50), derivative ≈ 1.0
    EXPECT_NEAR(result[50], 1.0, 0.1);
}

TEST(DataDrivenUtilsCovTest, Integrate) {
    // Constant: integral should be linear
    std::vector<double> signal = {1.0, 1.0, 1.0, 1.0, 1.0};
    double dt = 1.0;
    auto result = DataDrivenUtils::integrate(signal, dt);
    EXPECT_EQ(result.size(), signal.size());
    // Values should be monotonically increasing
    for (size_t i = 1; i < result.size(); i++) {
        EXPECT_GE(result[i], result[i-1]);
    }
}

TEST(DataDrivenUtilsCovTest, IntegrateLinear) {
    // Linear ramp: integral should be quadratic
    std::vector<double> signal;
    double dt = 0.01;
    for (int i = 0; i < 100; i++) {
        signal.push_back(i * dt); // y = t
    }
    auto result = DataDrivenUtils::integrate(signal, dt);
    EXPECT_EQ(result.size(), 100u);
    EXPECT_GT(result.back(), 0.0);
}

TEST(DataDrivenUtilsCovTest, CrossCorrelation) {
    // Signal with itself = autocorrelation, peak at lag 0
    std::vector<double> x = {1.0, 0.0, -1.0, 0.0, 1.0};
    std::vector<double> y = x;
    auto result = DataDrivenUtils::crossCorrelation(x, y);
    EXPECT_FALSE(result.empty());
}

TEST(DataDrivenUtilsCovTest, CrossCorrelationShifted) {
    std::vector<double> x = {0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<double> y = {0.0, 0.0, 1.0, 0.0, 0.0};
    auto result = DataDrivenUtils::crossCorrelation(x, y);
    EXPECT_FALSE(result.empty());
}

TEST(DataDrivenUtilsCovTest, CrossCorrelationLong) {
    std::vector<double> x(200), y(200);
    for (size_t i = 0; i < 200; i++) {
        x[i] = std::sin(2.0 * M_PI * i / 50.0);
        y[i] = std::sin(2.0 * M_PI * i / 50.0 + 0.5);
    }
    auto result = DataDrivenUtils::crossCorrelation(x, y);
    EXPECT_FALSE(result.empty());
}

TEST(DataDrivenUtilsCovTest, LeastSquares) {
    std::vector<std::vector<double>> A = {{1,0},{0,1},{1,1}};
    std::vector<double> b = {2.0, 3.0, 5.0};
    auto result = DataDrivenUtils::leastSquares(A, b);
    EXPECT_EQ(result.size(), 2u);
    EXPECT_NEAR(result[0], 2.0, 0.1);
    EXPECT_NEAR(result[1], 3.0, 0.1);
}

TEST(DataDrivenUtilsCovTest, LeastSquaresIdentity) {
    std::vector<std::vector<double>> A = {{1,0,0},{0,1,0},{0,0,1}};
    std::vector<double> b = {1.0, 2.0, 3.0};
    auto result = DataDrivenUtils::leastSquares(A, b);
    EXPECT_EQ(result.size(), 3u);
    EXPECT_NEAR(result[0], 1.0, 1e-4);
    EXPECT_NEAR(result[1], 2.0, 1e-4);
    EXPECT_NEAR(result[2], 3.0, 1e-4);
}

// ============================================================================
// isCompatible() tests for all tuner classes
// ============================================================================

TEST(IsCompatCovTest, QROptimizer) {
    QROptimizer qr;
    TestPIDController ctrl;
    EXPECT_TRUE(qr.isCompatible(ctrl));
}

TEST(IsCompatCovTest, LoopTransferRecovery) {
    LoopTransferRecovery ltr;
    TestPIDController ctrl;
    EXPECT_TRUE(ltr.isCompatible(ctrl));
}

TEST(IsCompatCovTest, RegionalPolePlacement) {
    RegionalPolePlacement rpp;
    TestPIDController ctrl;
    EXPECT_TRUE(rpp.isCompatible(ctrl));
}

TEST(IsCompatCovTest, IterativeFeedbackTuning) {
    IterativeFeedbackTuning ift;
    TestPIDController ctrl;
    EXPECT_TRUE(ift.isCompatible(ctrl));
}

TEST(IsCompatCovTest, VRFTuning) {
    VRFTuning vrft;
    TestPIDController ctrl;
    EXPECT_TRUE(vrft.isCompatible(ctrl));
}

TEST(IsCompatCovTest, FRITuning) {
    FRITuning fri;
    TestPIDController ctrl;
    EXPECT_TRUE(fri.isCompatible(ctrl));
}

// ============================================================================
// IFT - step size, finite difference step, gradient access, phases
// ============================================================================

TEST(IFTCovTest, SetStepSize) {
    IterativeFeedbackTuning ift;
    ift.setStepSize(0.01);
}

TEST(IFTCovTest, SetFiniteDifferenceStep) {
    IterativeFeedbackTuning ift;
    ift.setFiniteDifferenceStep(0.001);
}

TEST(IFTCovTest, SetIterations) {
    IterativeFeedbackTuning ift;
    ift.setIterations(5);
}

TEST(IFTCovTest, GetCurrentPhaseIdle) {
    IterativeFeedbackTuning ift;
    EXPECT_EQ(ift.getCurrentPhase(), IterativeFeedbackTuning::Phase::Idle);
}

TEST(IFTCovTest, GetGradientEmpty) {
    IterativeFeedbackTuning ift;
    auto grad = ift.getGradient();
    EXPECT_TRUE(grad.empty());
}

TEST(IFTCovTest, StartChangesPhase) {
    IterativeFeedbackTuning ift;
    ift.setReferenceSignal(std::vector<double>(100, 1.0));
    ift.start();
    auto phase = ift.getCurrentPhase();
    EXPECT_NE(phase, IterativeFeedbackTuning::Phase::Idle);
}

TEST(IFTCovTest, StopReturnsToIdle) {
    IterativeFeedbackTuning ift;
    ift.setReferenceSignal(std::vector<double>(100, 1.0));
    ift.start();
    ift.stop();
    EXPECT_EQ(ift.getCurrentPhase(), IterativeFeedbackTuning::Phase::Idle);
}

TEST(IFTCovTest, UpdateAccumulatesData) {
    IterativeFeedbackTuning ift;
    ift.setReferenceSignal(std::vector<double>(100, 1.0));
    ift.setWeights(1.0, 0.1);
    ift.setStepSize(0.001);
    ift.start();

    for (int i = 0; i < 100; i++) {
        ift.update(0.5, 1.0, 0.3, 0.001);
    }
    EXPECT_FALSE(ift.isComplete());
}

TEST(IFTCovTest, TuneWithController) {
    IterativeFeedbackTuning ift;
    TestPIDController ctrl;

    std::vector<double> ref(100, 1.0);
    ift.setReferenceSignal(ref);
    ift.setWeights(1.0, 0.1);
    ift.setStepSize(0.001);
    ift.setIterations(2);
    ift.setFiniteDifferenceStep(0.01);

    auto result = ift.tune(ctrl);
    (void)result;
}

TEST(IFTCovTest, GetIntermediateResult) {
    IterativeFeedbackTuning ift;
    auto result = ift.getIntermediateResult();
    (void)result;
}

// ============================================================================
// RegionalPolePlacement
// ============================================================================

TEST(RPPCovTest, AddDiskRegion) {
    RegionalPolePlacement rpp;
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Disk;
    region.diskCenterReal = 0.0;
    region.diskRadius = 1.0;
    rpp.addRegion(region);
    rpp.clearRegions();
}

TEST(RPPCovTest, AddStripRegion) {
    RegionalPolePlacement rpp;
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Strip;
    region.stripLeft = -5.0;
    region.stripRight = -0.1;
    rpp.addRegion(region);
}

TEST(RPPCovTest, AddSectorRegion) {
    RegionalPolePlacement rpp;
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Sector;
    region.sectorAngle = 0.7;
    rpp.addRegion(region);
}

TEST(RPPCovTest, TuneWithRegions) {
    RegionalPolePlacement rpp;
    TestPIDController ctrl;

    std::vector<std::vector<double>> A = {{0, 1}, {0, 0}};
    std::vector<std::vector<double>> B = {{0}, {1}};
    rpp.setStateSpaceModel(A, B);

    RegionalPolePlacement::LMIRegion disk;
    disk.type = RegionalPolePlacement::LMIRegion::Disk;
    disk.diskCenterReal = 0.0;
    disk.diskRadius = 5.0;
    rpp.addRegion(disk);

    auto result = rpp.tune(ctrl);
    (void)result;
}

TEST(RPPCovTest, TuneMultipleRegions) {
    RegionalPolePlacement rpp;
    TestPIDController ctrl;

    std::vector<std::vector<double>> A = {{0, 1}, {-1, -1}};
    std::vector<std::vector<double>> B = {{0}, {1}};
    rpp.setStateSpaceModel(A, B);

    RegionalPolePlacement::LMIRegion disk;
    disk.type = RegionalPolePlacement::LMIRegion::Disk;
    disk.diskCenterReal = 0.0;
    disk.diskRadius = 10.0;
    rpp.addRegion(disk);

    RegionalPolePlacement::LMIRegion strip;
    strip.type = RegionalPolePlacement::LMIRegion::Strip;
    strip.stripLeft = -10.0;
    strip.stripRight = -0.1;
    rpp.addRegion(strip);

    auto result = rpp.tune(ctrl);
    (void)result;
}

TEST(RPPCovTest, TuneWithSector) {
    RegionalPolePlacement rpp;
    TestPIDController ctrl;

    std::vector<std::vector<double>> A = {{0, 1}, {-2, -3}};
    std::vector<std::vector<double>> B = {{0}, {1}};
    rpp.setStateSpaceModel(A, B);

    RegionalPolePlacement::LMIRegion sector;
    sector.type = RegionalPolePlacement::LMIRegion::Sector;
    sector.sectorAngle = 0.5;
    rpp.addRegion(sector);

    auto result = rpp.tune(ctrl);
    (void)result;
}

TEST(RPPCovTest, TuneIntersection) {
    RegionalPolePlacement rpp;
    TestPIDController ctrl;

    std::vector<std::vector<double>> A = {{0, 1}, {-1, -2}};
    std::vector<std::vector<double>> B = {{0}, {1}};
    rpp.setStateSpaceModel(A, B);

    // Intersection: all constraints must be met
    RegionalPolePlacement::LMIRegion inter;
    inter.type = RegionalPolePlacement::LMIRegion::Intersection;
    inter.diskCenterReal = 0.0;
    inter.diskRadius = 10.0;
    inter.stripLeft = -10.0;
    inter.stripRight = -0.1;
    inter.sectorAngle = 0.7;
    rpp.addRegion(inter);

    auto result = rpp.tune(ctrl);
    (void)result;
}

TEST(RPPCovTest, GetPoles) {
    RegionalPolePlacement rpp;
    auto poles = rpp.getPoles();
    EXPECT_TRUE(poles.empty());
}

TEST(RPPCovTest, SetGainStructure) {
    RegionalPolePlacement rpp;
    rpp.setGainStructure({true, true, false});
}

TEST(RPPCovTest, TuneNoModel) {
    RegionalPolePlacement rpp;
    TestPIDController ctrl;
    auto result = rpp.tune(ctrl);
    EXPECT_FALSE(result.success);
}

// ============================================================================
// LoopTransferRecovery
// ============================================================================

TEST(LTRCovTest, TuneWithoutModel) {
    LoopTransferRecovery ltr;
    TestPIDController ctrl;
    auto result = ltr.tune(ctrl);
    EXPECT_FALSE(result.success);
}

TEST(LTRCovTest, SetRecoveryParameter) {
    LoopTransferRecovery ltr;
    ltr.setRecoveryParameter(1e3);
}

TEST(LTRCovTest, GetKalmanGainEmpty) {
    LoopTransferRecovery ltr;
    auto kf = ltr.getKalmanGain();
    EXPECT_TRUE(kf.empty());
}

TEST(LTRCovTest, GetRecoveryErrorDefault) {
    LoopTransferRecovery ltr;
    EXPECT_NEAR(ltr.getRecoveryError(), 0.0, 1e-10);
}

TEST(LTRCovTest, TuneWithModel) {
    LoopTransferRecovery ltr;
    TestPIDController ctrl;

    std::vector<std::vector<double>> A = {{0, 1}, {-2, -3}};
    std::vector<std::vector<double>> B = {{0}, {1}};
    std::vector<std::vector<double>> C = {{1, 0}};
    ltr.setStateSpaceModel(A, B, C);
    ltr.setTargetGains({1.0, 2.0});
    ltr.setRecoveryParameter(1e4);

    auto result = ltr.tune(ctrl);
    (void)result;
}

TEST(LTRCovTest, TuneWithNoiseMatrices) {
    LoopTransferRecovery ltr;
    TestPIDController ctrl;

    std::vector<std::vector<double>> A = {{-1}};
    std::vector<std::vector<double>> B = {{1}};
    std::vector<std::vector<double>> C = {{1}};
    ltr.setStateSpaceModel(A, B, C);
    ltr.setTargetGains({1.0});
    ltr.setProcessNoise({{1.0}});
    ltr.setMeasurementNoise({{0.1}});

    auto result = ltr.tune(ctrl);
    (void)result;
}

// ============================================================================
// VRFTuning
// ============================================================================

TEST(VRFTCovTest, SetDataAndTune) {
    VRFTuning vrft;
    TestPIDController ctrl;

    std::vector<double> u(100, 1.0);
    std::vector<double> y;
    for (int i = 0; i < 100; i++) {
        y.push_back(1.0 - std::exp(-0.05 * i));
    }
    vrft.setData(u, y, 0.01);
    vrft.setReferenceModel(1.0, 0.1);
    vrft.setControllerStructure({"Kp", "Ki", "Kd"});

    auto result = vrft.tune(ctrl);
    (void)result;
}

TEST(VRFTCovTest, GetVirtualReference) {
    VRFTuning vrft;
    auto vr = vrft.getVirtualReference();
    EXPECT_TRUE(vr.empty());
}

TEST(VRFTCovTest, GetFitQuality) {
    VRFTuning vrft;
    EXPECT_NEAR(vrft.getFitQuality(), 0.0, 1e-10);
}

TEST(VRFTCovTest, SecondOrderRefModel) {
    VRFTuning vrft;
    TestPIDController ctrl;

    std::vector<double> u(50, 1.0);
    std::vector<double> y(50, 0.5);
    vrft.setData(u, y, 0.01);
    vrft.setReferenceModel(1.0, 10.0, 0.707);
    vrft.setControllerStructure({"Kp", "Ki", "Kd"});

    auto result = vrft.tune(ctrl);
    (void)result;
}

TEST(VRFTCovTest, WithPrefilter) {
    VRFTuning vrft;
    TestPIDController ctrl;

    std::vector<double> u(100, 1.0);
    std::vector<double> y;
    for (int i = 0; i < 100; i++) {
        y.push_back(1.0 - std::exp(-0.05 * i));
    }
    vrft.setData(u, y, 0.01);
    vrft.setReferenceModel(1.0, 0.5);
    vrft.setControllerStructure({"Kp", "Ki", "Kd"});
    vrft.setPrefilter({1.0}, {1.0, -0.9});

    auto result = vrft.tune(ctrl);
    (void)result;
}

// ============================================================================
// FRITuning
// ============================================================================

TEST(FRICovTest, SetDataAndTune) {
    FRITuning fri;
    TestPIDController ctrl;

    std::vector<double> r(100, 1.0);
    std::vector<double> u(100, 0.5);
    std::vector<double> y;
    for (int i = 0; i < 100; i++) {
        y.push_back(1.0 - std::exp(-0.05 * i));
    }
    fri.setData(r, u, y, 0.01);
    fri.setReferenceModel(1.0, 0.1);

    auto result = fri.tune(ctrl);
    (void)result;
}

TEST(FRICovTest, GetMatchingError) {
    FRITuning fri;
    EXPECT_NEAR(fri.getMatchingError(), 0.0, 1e-10);
}

TEST(FRICovTest, SecondOrderRefModel) {
    FRITuning fri;
    TestPIDController ctrl;

    std::vector<double> r(50, 1.0);
    std::vector<double> u(50, 0.5);
    std::vector<double> y(50, 0.3);
    fri.setData(r, u, y, 0.01);
    fri.setReferenceModel(1.0, 10.0, 0.707);

    auto result = fri.tune(ctrl);
    (void)result;
}

TEST(FRICovTest, SetCustomOptimizer) {
    FRITuning fri;
    TestPIDController ctrl;

    std::vector<double> r(100, 1.0);
    std::vector<double> u(100, 0.5);
    std::vector<double> y;
    for (int i = 0; i < 100; i++) {
        y.push_back(1.0 - std::exp(-0.05 * i));
    }
    fri.setData(r, u, y, 0.01);
    fri.setReferenceModel(1.0, 0.1);

    // Set NelderMead as custom optimizer
    fri.setOptimizer(std::make_unique<NelderMead>());

    auto result = fri.tune(ctrl);
    (void)result;
}

// ============================================================================
// QROptimizer - tune coverage
// ============================================================================

TEST(QROptCovTest, TuneWithProcessModel) {
    QROptimizer qr;
    TestPIDController ctrl;

    TestFOPDTProcessModel process(2.0, 0.5, 0.01);
    auto result = qr.tune(ctrl, &process);
    (void)result;
}

TEST(QROptCovTest, TuneWithoutModel) {
    QROptimizer qr;
    TestPIDController ctrl;

    auto result = qr.tune(ctrl, nullptr);
    EXPECT_FALSE(result.success);
}
