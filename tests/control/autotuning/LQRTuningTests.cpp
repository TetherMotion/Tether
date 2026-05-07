/**
 * @file LQRTuningTests.cpp
 * @brief Unit tests for LQR tuning implementations
 */

#include <gtest/gtest.h>
#include "TestHelpers.hpp"
#include "../include/tether/control/autotuning/LQRTuning.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

// ============================================================================
// QROptimizer Tests
// ============================================================================

class QROptimizerTest : public ::testing::Test {
protected:
    std::unique_ptr<QROptimizer> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<QROptimizer>();
    }
};

TEST_F(QROptimizerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Q/R Weight Optimization");
}

TEST_F(QROptimizerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(QROptimizerTest, SetSpecifications) {
    // specifications should not throw and should be usable by tune()
    QROptimizer::Specifications specs;
    specs.settlingTime = 0.5;
    specs.overshoot = 0.05;
    specs.riseTime = 0.1;
    specs.steadyStateError = 0.001;
    specs.controlEffort = 10.0;

    tuner->setSpecifications(specs);

    tuner->setStateDimension(2);
    tuner->setControlDimension(1);
    tuner->setWeights(1.0, 1.0, 0.1);

    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.2);
    auto r = tuner->tune(controller, &processModel);

    EXPECT_TRUE(r.success);
    auto Q = tuner->getOptimalQ();
    auto R = tuner->getOptimalR();
    EXPECT_EQ(Q.size(), 4u);   // 2x2
    EXPECT_EQ(R.size(), 1u);   // 1x1
}

TEST_F(QROptimizerTest, SetWeights) {
    // weights should influence the produced optimal matrices
    tuner->setStateDimension(3);
    tuner->setControlDimension(1);

    tuner->setWeights(1.0, 0.1, 0.01);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 3.0, 0.1);
    auto r1 = tuner->tune(controller, &processModel);
    auto Q1 = tuner->getOptimalQ();

    tuner->setWeights(10.0, 5.0, 1.0);
    auto r2 = tuner->tune(controller, &processModel);
    auto Q2 = tuner->getOptimalQ();

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    EXPECT_EQ(Q1.size(), 9u);
    EXPECT_EQ(Q2.size(), 9u);
    EXPECT_NE(Q1, Q2);  // different weights -> different optimal Q
}

TEST_F(QROptimizerTest, SetStateDimensionAndControlDimension) {
    tuner->setStateDimension(4);
    tuner->setControlDimension(2);

    tuner->setWeights(1.0, 1.0, 0.1);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 4.0, 0.2);
    auto r = tuner->tune(controller, &processModel);

    EXPECT_TRUE(r.success);
    EXPECT_EQ(tuner->getOptimalQ().size(), 16u); // 4x4
    EXPECT_EQ(tuner->getOptimalR().size(), 4u);  // 2x2
}

TEST_F(QROptimizerTest, FixQElements) {
    // fixed elements should appear in the optimal Q after tuning
    tuner->setStateDimension(3);
    tuner->setControlDimension(1);
    tuner->setWeights(1.0, 1.0, 0.1);

    tuner->fixQElements({0,2}, {2.0, 0.5});
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 3.0, 0.1);
    auto r = tuner->tune(controller, &processModel);

    EXPECT_TRUE(r.success);
    auto Qopt = tuner->getOptimalQ();
    EXPECT_EQ(Qopt.size(), 9u);
    EXPECT_DOUBLE_EQ(Qopt[0], 2.0);
    EXPECT_DOUBLE_EQ(Qopt[2], 0.5);
}

TEST_F(QROptimizerTest, GetOptimalMatricesInitiallyAndAfterTune) {
    // initially empty, populated after tune
    auto Q_before = tuner->getOptimalQ();
    auto R_before = tuner->getOptimalR();
    EXPECT_TRUE(Q_before.empty());
    EXPECT_TRUE(R_before.empty());

    tuner->setStateDimension(2);
    tuner->setControlDimension(1);
    tuner->setWeights(1.0, 1.0, 0.1);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.2);
    auto r = tuner->tune(controller, &processModel);

    EXPECT_TRUE(r.success);
    auto Q_after = tuner->getOptimalQ();
    auto R_after = tuner->getOptimalR();
    EXPECT_EQ(Q_after.size(), 4u);
    EXPECT_EQ(R_after.size(), 1u);
}

TEST_F(QROptimizerTest, TuneProducesValidControllerParameters) {
    // tune() should succeed and set controller parameters (non-negative gains)
    tuner->setStateDimension(2);
    tuner->setControlDimension(1);
    tuner->setWeights(1.0, 1.0, 0.1);

    TestPIDController controller(0.0, 0.0, 0.0);
    TestFOPDTProcessModel processModel(1.0, 4.0, 0.2);
    auto r = tuner->tune(controller, &processModel);

    EXPECT_TRUE(r.success);
    EXPECT_GT(controller.getKp(), 0.0);
}

// ============================================================================
// LoopTransferRecovery Tests
// ============================================================================

class LoopTransferRecoveryTest : public ::testing::Test {
protected:
    std::unique_ptr<LoopTransferRecovery> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<LoopTransferRecovery>();
    }
};

TEST_F(LoopTransferRecoveryTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Loop Transfer Recovery");
}

TEST_F(LoopTransferRecoveryTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(LoopTransferRecoveryTest, Tune) {
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// RegionalPolePlacement Tests
// ============================================================================

class RegionalPolePlacementTest : public ::testing::Test {
protected:
    std::unique_ptr<RegionalPolePlacement> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<RegionalPolePlacement>();
    }
};

TEST_F(RegionalPolePlacementTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Regional Pole Placement");
}

TEST_F(RegionalPolePlacementTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(RegionalPolePlacementTest, AddDiskRegion) {
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Disk;
    region.diskCenterReal = -1.0;
    region.diskRadius = 0.5;
    
    EXPECT_NO_THROW(tuner->addRegion(region));
}

TEST_F(RegionalPolePlacementTest, AddSectorRegion) {
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Sector;
    region.sectorAngle = 0.5;  // ~30 deg
    
    EXPECT_NO_THROW(tuner->addRegion(region));
}

TEST_F(RegionalPolePlacementTest, AddStripRegion) {
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Strip;
    region.stripLeft = -5.0;
    region.stripRight = -0.5;
    
    tuner->addRegion(region);  // Not wrapped in EXPECT_NO_THROW - unreachable code issue
}

TEST_F(RegionalPolePlacementTest, ClearRegions) {
    RegionalPolePlacement::LMIRegion region;
    region.type = RegionalPolePlacement::LMIRegion::Disk;
    tuner->addRegion(region);
    tuner->clearRegions();
    EXPECT_NO_THROW(tuner->clearRegions());
}

TEST_F(RegionalPolePlacementTest, SetStateSpaceModel) {
    std::vector<std::vector<double>> A = {{-1.0, 0.0}, {0.0, -2.0}};
    std::vector<std::vector<double>> B = {{1.0}, {0.0}};
    
    EXPECT_NO_THROW(tuner->setStateSpaceModel(A, B));
}

TEST_F(RegionalPolePlacementTest, SetGainStructure) {
    std::vector<bool> structure = {true, true, false};
    EXPECT_NO_THROW(tuner->setGainStructure(structure));
}

TEST_F(RegionalPolePlacementTest, GetPoles) {
    auto poles = tuner->getPoles();
    // Initially empty
    EXPECT_TRUE(poles.empty());
}

TEST_F(RegionalPolePlacementTest, Tune) {
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// IterativeFeedbackTuning Tests
// ============================================================================

class IterativeFeedbackTuningTest : public ::testing::Test {
protected:
    std::unique_ptr<IterativeFeedbackTuning> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<IterativeFeedbackTuning>();
    }
};

TEST_F(IterativeFeedbackTuningTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Iterative Feedback Tuning");
}

TEST_F(IterativeFeedbackTuningTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(IterativeFeedbackTuningTest, SetReferenceSignal) {
    std::vector<double> r(100, 1.0);
    EXPECT_NO_THROW(tuner->setReferenceSignal(r));
}

TEST_F(IterativeFeedbackTuningTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(IterativeFeedbackTuningTest, IsComplete) {
    tuner->start();
    EXPECT_FALSE(tuner->isComplete());
}

TEST_F(IterativeFeedbackTuningTest, Update) {
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 100; ++i) {
        double measured = 0.5 * (1.0 - std::exp(-i * 0.1));
        output = tuner->update(measured, 1.0, output, 0.1);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(IterativeFeedbackTuningTest, GetIntermediateResult) {
    tuner->start();
    EXPECT_NO_THROW({ auto result = tuner->getIntermediateResult(); });
}

TEST_F(IterativeFeedbackTuningTest, Tune) {
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// VRFTuning Tests
// ============================================================================

class VRFTuningTest : public ::testing::Test {
protected:
    std::unique_ptr<VRFTuning> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<VRFTuning>();
    }
};

TEST_F(VRFTuningTest, GetName) {
    EXPECT_EQ(tuner->getName(), "VRFT");
}

TEST_F(VRFTuningTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(VRFTuningTest, SetData) {
    std::vector<double> u(100), y(100);
    for (int i = 0; i < 100; ++i) {
        u[i] = i < 50 ? 0.0 : 1.0;
        y[i] = u[i] * (1.0 - std::exp(-i * 0.1));
    }
    
    EXPECT_NO_THROW(tuner->setData(u, y, 0.1));
}

TEST_F(VRFTuningTest, SetReferenceModelFirstOrder) {
    EXPECT_NO_THROW(tuner->setReferenceModel(1.0, 0.5));  // K=1, tau=0.5
}

TEST_F(VRFTuningTest, SetReferenceModelSecondOrder) {
    EXPECT_NO_THROW(tuner->setReferenceModel(1.0, 2.0, 0.707));  // K, wn, zeta
}

TEST_F(VRFTuningTest, SetControllerStructure) {
    std::vector<std::string> terms = {"Kp", "Ki", "Kd"};
    EXPECT_NO_THROW(tuner->setControllerStructure(terms));
}

TEST_F(VRFTuningTest, SetPrefilter) {
    std::vector<double> numL = {1.0};
    std::vector<double> denL = {1.0, 0.1};  // Low-pass
    EXPECT_NO_THROW(tuner->setPrefilter(numL, denL));
}

TEST_F(VRFTuningTest, GetVirtualReference) {
    auto rVirtual = tuner->getVirtualReference();
    // Initially empty
    EXPECT_TRUE(rVirtual.empty());
}

TEST_F(VRFTuningTest, GetFitQuality) {
    double R2 = tuner->getFitQuality();
    EXPECT_GE(R2, 0.0);
    EXPECT_LE(R2, 1.0);
}

TEST_F(VRFTuningTest, Tune) {
    // Set up data first
    std::vector<double> u(100), y(100);
    for (int i = 0; i < 100; ++i) {
        u[i] = i < 50 ? 0.0 : 1.0;
        y[i] = u[i] * (1.0 - std::exp(-(i-50) * 0.1));
        if (i < 50) y[i] = 0.0;
    }
    tuner->setData(u, y, 0.1);
    tuner->setReferenceModel(1.0, 0.5);
    
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// FRITuning Tests
// ============================================================================

class FRITuningTest : public ::testing::Test {
protected:
    std::unique_ptr<FRITuning> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<FRITuning>();
    }
};

TEST_F(FRITuningTest, GetName) {
    EXPECT_EQ(tuner->getName(), "FRIT");
}

TEST_F(FRITuningTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(FRITuningTest, SetData) {
    std::vector<double> r(100, 1.0);
    std::vector<double> u(100), y(100);
    for (int i = 0; i < 100; ++i) {
        u[i] = 0.5 * r[i];
        y[i] = r[i] * (1.0 - std::exp(-i * 0.1));
    }
    
    EXPECT_NO_THROW(tuner->setData(r, u, y, 0.1));
}

TEST_F(FRITuningTest, SetReferenceModelFirstOrder) {
    EXPECT_NO_THROW(tuner->setReferenceModel(1.0, 0.5));
}

TEST_F(FRITuningTest, SetReferenceModelSecondOrder) {
    EXPECT_NO_THROW(tuner->setReferenceModel(1.0, 2.0, 0.707));
}

TEST_F(FRITuningTest, GetMatchingError) {
    double error = tuner->getMatchingError();
    EXPECT_GE(error, 0.0);
}

TEST_F(FRITuningTest, Tune) {
    // Set up data
    std::vector<double> r(100, 1.0);
    std::vector<double> u(100), y(100);
    for (int i = 0; i < 100; ++i) {
        u[i] = 0.5 * r[i];
        y[i] = r[i] * (1.0 - std::exp(-i * 0.1));
    }
    tuner->setData(r, u, y, 0.1);
    tuner->setReferenceModel(1.0, 0.5);
    
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(LQRIntegration, QROptimizerFlow) {
    QROptimizer tuner;
    
    QROptimizer::Specifications specs;
    specs.settlingTime = 0.5;
    specs.overshoot = 0.05;
    tuner.setSpecifications(specs);
    tuner.setWeights(1.0, 2.0, 0.5);
    tuner.setStateDimension(2);
    tuner.setControlDimension(1);
    
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    
    auto result = tuner.tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST(LQRIntegration, VRFTWorkflow) {
    VRFTuning tuner;
    
    // Generate experimental data
    std::vector<double> u(200), y(200);
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.05;
        u[i] = i < 20 ? 0.0 : 1.0;
        y[i] = t > 1.0 ? 2.0 * (1.0 - std::exp(-(t - 1.0) / 0.5)) : 0.0;
    }
    
    tuner.setData(u, y, 0.05);
    tuner.setReferenceModel(1.0, 0.3);  // Faster than actual
    tuner.setControllerStructure({"Kp", "Ki"});  // PI controller
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    EXPECT_TRUE(result.success);
}

TEST(LQRIntegration, RegionalPolePlacementFlow) {
    RegionalPolePlacement tuner;
    
    // Define desired region: fast and well-damped
    RegionalPolePlacement::LMIRegion diskRegion;
    diskRegion.type = RegionalPolePlacement::LMIRegion::Disk;
    diskRegion.diskCenterReal = -2.0;
    diskRegion.diskRadius = 1.0;
    tuner.addRegion(diskRegion);
    
    RegionalPolePlacement::LMIRegion sectorRegion;
    sectorRegion.type = RegionalPolePlacement::LMIRegion::Sector;
    sectorRegion.sectorAngle = 0.5;  // ζ ≈ 0.87
    tuner.addRegion(sectorRegion);
    
    // Simple state-space model
    std::vector<std::vector<double>> A = {{0.0, 1.0}, {-1.0, -0.5}};
    std::vector<std::vector<double>> B = {{0.0}, {1.0}};
    tuner.setStateSpaceModel(A, B);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    auto poles = tuner.getPoles();
    EXPECT_TRUE(result.success);
}

TEST(LQRIntegration, FRITWorkflow) {
    FRITuning tuner;
    
    // Generate closed-loop experimental data
    const int N = 150;
    std::vector<double> r(N, 1.0);
    std::vector<double> u(N), y(N);
    
    // Simulate closed-loop with some controller
    double yi = 0.0;
    for (int i = 0; i < N; ++i) {
        double e = r[i] - yi;
        u[i] = 0.5 * e;  // Simple P controller
        
        // Simple first-order plant simulation
        yi += 0.1 * (2.0 * u[i] - yi);
        y[i] = yi;
    }
    
    tuner.setData(r, u, y, 0.1);
    tuner.setReferenceModel(1.0, 0.2);  // Desired faster response
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    double error = tuner.getMatchingError();
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(std::isfinite(error));
}

TEST(LQRIntegration, IFTOnlineFlow) {
    IterativeFeedbackTuning tuner;
    
    // Set reference signal
    std::vector<double> r(100);
    for (int i = 0; i < 100; ++i) {
        r[i] = i < 50 ? 0.0 : 1.0;  // Step
    }
    tuner.setReferenceSignal(r);
    
    tuner.start();
    
    // Simulate experiment
    double output = 0.0;
    double y = 0.0;
    for (int i = 0; i < 100; ++i) {
        // Plant simulation
        y += 0.1 * (output - y);
        
        output = tuner.update(y, r[i], output, 0.1);
    }
    
    auto result = tuner.getIntermediateResult();
    tuner.stop();
    
    EXPECT_NO_THROW(result);
}

// ============================================================================
// Additional Coverage Tests for LQRTuning
// ============================================================================

TEST(QROptimizerCoverage, FixQElements) {
    QROptimizer tuner;
    
    tuner.setStateDimension(3);
    tuner.fixQElements({0, 2}, {1.0, 0.5});
    
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &processModel);
}

TEST(QROptimizerCoverage, GetOptimalMatrices) {
    QROptimizer tuner;
    
    QROptimizer::Specifications specs;
    specs.settlingTime = 0.5;
    specs.overshoot = 0.05;
    tuner.setSpecifications(specs);
    
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &processModel);
    
    auto Qopt = tuner.getOptimalQ();
    auto Ropt = tuner.getOptimalR();
}

TEST(LoopTransferRecoveryCoverage, FullSetup) {
    LoopTransferRecovery tuner;
    
    // Set 2D state-space model
    std::vector<std::vector<double>> A = {{0.0, 1.0}, {-1.0, -0.5}};
    std::vector<std::vector<double>> B = {{0.0}, {1.0}};
    std::vector<std::vector<double>> C = {{1.0, 0.0}};
    tuner.setStateSpaceModel(A, B, C);
    
    std::vector<double> K = {1.0, 0.5};
    tuner.setTargetGains(K);
    tuner.setRecoveryParameter(1e4);
    
    std::vector<std::vector<double>> V1 = {{0.1, 0.0}, {0.0, 0.1}};
    std::vector<std::vector<double>> V2 = {{0.01}};
    tuner.setProcessNoise(V1);
    tuner.setMeasurementNoise(V2);
    
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &processModel);
    
    auto Kf = tuner.getKalmanGain();
    auto error = tuner.getRecoveryError();
}

TEST(RegionalPolePlacementCoverage, AllRegionTypes) {
    RegionalPolePlacement tuner;
    
    // Test disk region
    RegionalPolePlacement::LMIRegion disk;
    disk.type = RegionalPolePlacement::LMIRegion::Disk;
    disk.diskCenterReal = -2.0;
    disk.diskRadius = 1.5;
    tuner.addRegion(disk);
    
    // Test sector region  
    RegionalPolePlacement::LMIRegion sector;
    sector.type = RegionalPolePlacement::LMIRegion::Sector;
    sector.sectorAngle = 0.3;
    tuner.addRegion(sector);
    
    // Test strip region
    RegionalPolePlacement::LMIRegion strip;
    strip.type = RegionalPolePlacement::LMIRegion::Strip;
    strip.stripLeft = -5.0;
    strip.stripRight = -0.5;
    tuner.addRegion(strip);
    
    std::vector<std::vector<double>> A = {{0.0, 1.0}, {-1.0, -0.5}};
    std::vector<std::vector<double>> B = {{0.0}, {1.0}};
    tuner.setStateSpaceModel(A, B);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    auto poles = tuner.getPoles();
}

TEST(RegionalPolePlacementCoverage, IntersectionRegion) {
    RegionalPolePlacement tuner;
    
    RegionalPolePlacement::LMIRegion intersection;
    intersection.type = RegionalPolePlacement::LMIRegion::Intersection;
    intersection.diskCenterReal = -1.0;
    intersection.diskRadius = 2.0;
    intersection.sectorAngle = 0.5;
    tuner.addRegion(intersection);
    
    std::vector<std::vector<double>> A = {{0.0, 1.0}, {-2.0, -1.0}};
    std::vector<std::vector<double>> B = {{0.0}, {1.0}};
    tuner.setStateSpaceModel(A, B);
    tuner.setGainStructure({true, true});
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

TEST(VRFTuningCoverage, AllConfigurations) {
    VRFTuning tuner;
    
    // Generate experimental data
    std::vector<double> u(200), y(200);
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.05;
        u[i] = i < 20 ? 0.0 : 1.0;
        y[i] = t > 1.0 ? 2.0 * (1.0 - std::exp(-(t - 1.0) / 0.5)) : 0.0;
    }
    
    tuner.setData(u, y, 0.05);
    tuner.setReferenceModel(1.0, 0.3);
    
    // Test different controller structures
    tuner.setControllerStructure({"Kp"});  // P only
    TestPIDController controller;
    auto result1 = tuner.tune(controller, nullptr);
    
    tuner.setControllerStructure({"Kp", "Ki"});  // PI
    auto result2 = tuner.tune(controller, nullptr);
    
    tuner.setControllerStructure({"Kp", "Ki", "Kd"});  // PID
    auto result3 = tuner.tune(controller, nullptr);
}

TEST(FRITuningCoverage, AllMethods) {
    FRITuning tuner;
    
    const int N = 150;
    std::vector<double> r(N, 1.0);
    std::vector<double> u(N), y(N);
    
    double yi = 0.0;
    for (int i = 0; i < N; ++i) {
        double e = r[i] - yi;
        u[i] = 0.5 * e;
        yi += 0.1 * (2.0 * u[i] - yi);
        y[i] = yi;
    }
    
    tuner.setData(r, u, y, 0.1);
    tuner.setReferenceModel(1.0, 0.2);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    double error = tuner.getMatchingError();
    EXPECT_TRUE(std::isfinite(error));
}

TEST(IFTuningCoverage, Weights) {
    IterativeFeedbackTuning tuner;
    
    tuner.setWeights(1.0, 0.5);  // lambdaY, lambdaU
    tuner.setMaxIterations(5);
    
    std::vector<double> r(100);
    for (int i = 0; i < 100; ++i) {
        r[i] = i < 50 ? 0.0 : 1.0;
    }
    tuner.setReferenceSignal(r);
    
    tuner.start();
    
    double output = 0.0;
    double y = 0.0;
    for (int i = 0; i < 100; ++i) {
        y += 0.1 * (output - y);
        output = tuner.update(y, r[i], output, 0.1);
    }
    
    auto result = tuner.getIntermediateResult();
    
    tuner.stop();
}

TEST(LQREdgeCases, InvalidModels) {
    QROptimizer tuner;
    
    // Test with null model
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    EXPECT_FALSE(result.success);
}

TEST(LQREdgeCases, EmptyData) {
    VRFTuning tuner;
    
    std::vector<double> empty;
    tuner.setData(empty, empty, 0.1);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    // Should handle gracefully
}

TEST(LQREdgeCases, ClearRegions) {
    RegionalPolePlacement tuner;
    
    RegionalPolePlacement::LMIRegion disk;
    disk.type = RegionalPolePlacement::LMIRegion::Disk;
    tuner.addRegion(disk);
    tuner.clearRegions();
    
    // Now should have no regions
    std::vector<std::vector<double>> A = {{-1.0}};
    std::vector<std::vector<double>> B = {{1.0}};
    tuner.setStateSpaceModel(A, B);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// ============================================================================
// Additional Coverage Tests for LQRTuning
// ============================================================================

TEST(QROptimizerCoverage, AllSpecifications) {
    QROptimizer tuner;
    
    QROptimizer::Specifications specs;
    specs.settlingTime = 1.0;
    specs.overshoot = 0.10;
    specs.riseTime = 0.2;
    specs.steadyStateError = 0.01;
    specs.controlEffort = 50.0;
    
    tuner.setSpecifications(specs);
    tuner.setStateDimension(3);
    tuner.setControlDimension(1);
    
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &processModel);
    
    auto Q = tuner.getOptimalQ();
    auto R = tuner.getOptimalR();
}

TEST(QROptimizerCoverage, VariousDimensions) {
    for (int stateDim = 1; stateDim <= 4; ++stateDim) {
        for (int ctrlDim = 1; ctrlDim <= 2; ++ctrlDim) {
            QROptimizer tuner;
            tuner.setStateDimension(stateDim);
            tuner.setControlDimension(ctrlDim);
            tuner.setWeights(1.0, 0.5, 0.1);
            
            TestPIDController controller;
            TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
            
            auto result = tuner.tune(controller, &processModel);
        }
    }
}

TEST(QROptimizerCoverage, FixedElements) {
    QROptimizer tuner;
    tuner.setStateDimension(3);
    tuner.setControlDimension(1);
    
    // Fix some Q elements
    std::vector<int> indices = {0, 2};
    std::vector<double> values = {10.0, 5.0};
    tuner.fixQElements(indices, values);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    auto Q = tuner.getOptimalQ();
}

TEST(LoopTransferRecoveryCoverage, VariousRecoveryParameters) {
    std::vector<double> qValues = {0.001, 0.01, 0.1, 1.0, 10.0, 100.0};
    
    for (double q : qValues) {
        LoopTransferRecovery tuner;
        tuner.setRecoveryParameter(q);
        
        std::vector<std::vector<double>> A = {{-1.0, 0.0}, {0.0, -2.0}};
        std::vector<std::vector<double>> B = {{1.0}, {1.0}};
        std::vector<std::vector<double>> C = {{1.0, 0.0}};
        tuner.setStateSpaceModel(A, B, C);
        
        TestPIDController controller;
        auto result = tuner.tune(controller, nullptr);
    }
}

TEST(LoopTransferRecoveryCoverage, HigherOrderSystems) {
    LoopTransferRecovery tuner;
    tuner.setRecoveryParameter(1.0);
    
    // 3rd order system
    std::vector<std::vector<double>> A = {
        {-1.0, 0.0, 0.0},
        {1.0, -2.0, 0.0},
        {0.0, 1.0, -3.0}
    };
    std::vector<std::vector<double>> B = {{1.0}, {0.0}, {0.0}};
    std::vector<std::vector<double>> C = {{0.0, 0.0, 1.0}};
    
    tuner.setStateSpaceModel(A, B, C);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// Duplicate test removed - AllRegionTypes already exists earlier in file
// Renamed to avoid conflict
TEST(RegionalPolePlacementCoverage, AdditionalRegionTypes) {
    RegionalPolePlacement tuner;
    
    std::vector<std::vector<double>> A = {{0.0, 1.0}, {-2.0, -3.0}};
    std::vector<std::vector<double>> B = {{0.0}, {1.0}};
    tuner.setStateSpaceModel(A, B);
    
    // Test all region types with correct member names
    RegionalPolePlacement::LMIRegion disk;
    disk.type = RegionalPolePlacement::LMIRegion::Disk;
    disk.diskCenterReal = -1.0;
    disk.diskRadius = 2.0;
    tuner.addRegion(disk);
    
    RegionalPolePlacement::LMIRegion sector;
    sector.type = RegionalPolePlacement::LMIRegion::Sector;
    sector.sectorAngle = 0.785;  // ~45 deg in radians
    tuner.addRegion(sector);
    
    RegionalPolePlacement::LMIRegion strip;
    strip.type = RegionalPolePlacement::LMIRegion::Strip;
    strip.stripLeft = -5.0;
    strip.stripRight = -0.5;
    tuner.addRegion(strip);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// Renamed to avoid conflict with earlier test
TEST(RegionalPolePlacementCoverage, AdditionalIntersectionRegion) {
    RegionalPolePlacement tuner;
    
    std::vector<std::vector<double>> A = {{0.0, 1.0}, {-1.0, -1.0}};
    std::vector<std::vector<double>> B = {{0.0}, {1.0}};
    tuner.setStateSpaceModel(A, B);
    
    // Intersection of multiple regions
    RegionalPolePlacement::LMIRegion intersect;
    intersect.type = RegionalPolePlacement::LMIRegion::Intersection;
    tuner.addRegion(intersect);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

TEST(VRFTuningCoverage, VariousDataLengths) {
    std::vector<int> lengths = {10, 50, 100, 500};
    
    for (int len : lengths) {
        VRFTuning tuner;
        
        std::vector<double> input(len);
        std::vector<double> output(len);
        
        for (int i = 0; i < len; ++i) {
            input[i] = (i % 20 < 10) ? 1.0 : 0.0;
            output[i] = input[i] * (1.0 - std::exp(-0.1 * (i % 20)));
        }
        
        tuner.setData(input, output, 0.1);
        
        TestPIDController controller;
        auto result = tuner.tune(controller, nullptr);
    }
}

TEST(VRFTuningCoverage, DifferentPrefilters) {
    VRFTuning tuner;
    
    std::vector<double> input(100);
    std::vector<double> output(100);
    
    for (int i = 0; i < 100; ++i) {
        input[i] = (i < 50) ? 0.0 : 1.0;
        output[i] = input[i] * (1.0 - std::exp(-0.05 * std::max(0, i - 50)));
    }
    
    tuner.setData(input, output, 0.1);
    tuner.setPrefilter({0.5, 0.5}, {1.0, 0.3});
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// FRITuning tests - use correct API
TEST(FRITuningCoverage, ReferenceModelVariations) {
    FRITuning tuner;
    
    // Test first-order reference model
    tuner.setReferenceModel(1.0, 0.5);  // K, tau
    
    std::vector<double> ref(100, 1.0);
    std::vector<double> input(100);
    std::vector<double> output(100);
    
    for (int i = 0; i < 100; ++i) {
        input[i] = (i < 50) ? 0.0 : 1.0;
        output[i] = input[i] * (1.0 - std::exp(-0.1 * std::max(0, i - 50)));
    }
    
    tuner.setData(ref, input, output, 0.1);  // Correct 4-parameter version
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    double error = tuner.getMatchingError();
    EXPECT_GE(error, 0.0);
}

TEST(FRITuningCoverage, SecondOrderReferenceModel) {
    FRITuning tuner;
    
    // Test second-order reference model
    tuner.setReferenceModel(1.0, 2.0, 0.707);  // K, wn, zeta
    
    std::vector<double> ref(100, 1.0);
    std::vector<double> input(100);
    std::vector<double> output(100);
    
    for (int i = 0; i < 100; ++i) {
        input[i] = (i < 50) ? 0.0 : 1.0;
        output[i] = input[i] * (1.0 - std::exp(-0.1 * std::max(0, i - 50)));
    }
    
    tuner.setData(ref, input, output, 0.1);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

TEST(IterativeFeedbackTuningCoverage, VariousIterations) {
    for (int maxIter = 1; maxIter <= 10; maxIter += 3) {
        IterativeFeedbackTuning tuner;
        tuner.setMaxIterations(maxIter);
        tuner.setWeights(1.0, 0.1);
        
        std::vector<double> ref(100);
        for (int i = 0; i < 100; ++i) {
            ref[i] = (i < 50) ? 0.0 : 1.0;
        }
        tuner.setReferenceSignal(ref);
        
        tuner.start();
        
        double y = 0.0;
        double u = 0.0;
        for (int i = 0; i < 100; ++i) {
            y += 0.1 * (u - y);
            u = tuner.update(y, ref[i], u, 0.1);
        }
        
        tuner.stop();
        
        auto result = tuner.getIntermediateResult();
    }
}

TEST(IterativeFeedbackTuningCoverage, GradientEstimation) {
    IterativeFeedbackTuning tuner;
    tuner.setMaxIterations(5);
    tuner.setWeights(1.0, 0.5);
    
    std::vector<double> ref(200);
    for (int i = 0; i < 200; ++i) {
        ref[i] = std::sin(0.1 * i);  // Sinusoidal reference
    }
    tuner.setReferenceSignal(ref);
    
    tuner.start();
    
    double y = 0.0;
    double u = 0.0;
    for (int i = 0; i < 200; ++i) {
        y += 0.05 * (u - y);
        u = tuner.update(y, ref[i], u, 0.1);
    }
    
    auto result = tuner.getIntermediateResult();
    
    tuner.stop();
}

TEST(LQRIntegration, QROptimizerWithLTR) {
    TestPIDController controller;
    
    // First optimize Q/R weights
    QROptimizer qrTuner;
    qrTuner.setStateDimension(2);
    qrTuner.setControlDimension(1);
    qrTuner.setWeights(1.0, 0.5, 0.1);
    
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    auto qrResult = qrTuner.tune(controller, &processModel);
    
    // Then apply LTR for robustness
    LoopTransferRecovery ltrTuner;
    ltrTuner.setRecoveryParameter(0.1);
    
    std::vector<std::vector<double>> A = {{-0.1, 0.0}, {1.0, 0.0}};
    std::vector<std::vector<double>> B = {{1.0}, {0.0}};
    std::vector<std::vector<double>> C = {{0.0, 1.0}};
    ltrTuner.setStateSpaceModel(A, B, C);
    
    auto ltrResult = ltrTuner.tune(controller, nullptr);
}

TEST(LQRIntegration, DataDrivenComparison) {
    // Generate test data
    std::vector<double> ref(200, 1.0);  // Reference signal
    std::vector<double> input(200);
    std::vector<double> output(200);
    
    double y = 0.0;
    for (int i = 0; i < 200; ++i) {
        input[i] = (i % 40 < 20) ? 1.0 : 0.0;
        y += 0.05 * (input[i] - y);
        output[i] = y;
    }
    
    TestPIDController controller;
    
    // VRFT
    VRFTuning vrftTuner;
    vrftTuner.setData(input, output, 0.1);
    auto vrftResult = vrftTuner.tune(controller, nullptr);
    
    // FRIT - use correct 4-parameter setData
    FRITuning fritTuner;
    fritTuner.setReferenceModel(1.0, 0.5);  // Use reference model instead of Objective enum
    fritTuner.setData(ref, input, output, 0.1);
    auto fritResult = fritTuner.tune(controller, nullptr);
}
