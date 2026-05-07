/**
 * @file HybridMethodsTests.cpp
 * @brief Comprehensive unit tests for hybrid tuning methods
 */

#include <gtest/gtest.h>
#include "TestHelpers.hpp"
#include "tether/control/autotuning/HybridMethods.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

// ============================================================================
// ZNWithOptimization Tests
// ============================================================================

class ZNWithOptimizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<ZNWithOptimization>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
        model = std::make_unique<TestFOPDTProcessModel>(1.0, 10.0, 2.0);
    }
    
    std::unique_ptr<ZNWithOptimization> tuner;
    std::shared_ptr<TestPIDController> controller;
    std::unique_ptr<TestFOPDTProcessModel> model;
};

TEST_F(ZNWithOptimizationTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Z-N + Optimization");
}

TEST_F(ZNWithOptimizationTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(ZNWithOptimizationTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(ZNWithOptimizationTest, SetInitialMethodZNStep) {
    tuner->setInitialMethod(ZNWithOptimization::InitialMethod::ZNStepResponse);
}

TEST_F(ZNWithOptimizationTest, SetInitialMethodZNUltimate) {
    tuner->setInitialMethod(ZNWithOptimization::InitialMethod::ZNUltimateCycle);
}

TEST_F(ZNWithOptimizationTest, SetInitialMethodCohenCoon) {
    tuner->setInitialMethod(ZNWithOptimization::InitialMethod::CohenCoon);
}

TEST_F(ZNWithOptimizationTest, SetInitialMethodIMC) {
    tuner->setInitialMethod(ZNWithOptimization::InitialMethod::IMC);
}

TEST_F(ZNWithOptimizationTest, SetRelativeBounds) {
    tuner->setRelativeBounds(0.5, 2.0);
}

TEST_F(ZNWithOptimizationTest, SetMaxIterations) {
    tuner->setMaxIterations(200);
}

TEST_F(ZNWithOptimizationTest, GetInitialGains) {
    auto gains = tuner->getInitialGains();
}

TEST_F(ZNWithOptimizationTest, GetImprovementRatio) {
    double ratio = tuner->getImprovementRatio();
    EXPECT_TRUE(std::isfinite(ratio) || std::isnan(ratio));
}

TEST_F(ZNWithOptimizationTest, Tune) {
    tuner->setMaxIterations(20);
    auto result = tuner->tune(*controller, model.get());
}

// ============================================================================
// IMCWithRelay Tests
// ============================================================================

class IMCWithRelayTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<IMCWithRelay>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<IMCWithRelay> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(IMCWithRelayTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "IMC + Relay");
}

TEST_F(IMCWithRelayTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(IMCWithRelayTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(IMCWithRelayTest, SetRelayAmplitude) {
    tuner->setRelayAmplitude(1.0);
}

TEST_F(IMCWithRelayTest, SetRelayHysteresis) {
    tuner->setRelayHysteresis(0.1);
}

TEST_F(IMCWithRelayTest, SetIMCFactor) {
    tuner->setIMCFactor(2.0);
}

TEST_F(IMCWithRelayTest, StartAndUpdate) {
    tuner->start();
    
    double result = tuner->update(0.5, 1.0, 0.3, DT);
    EXPECT_TRUE(std::isfinite(result));
    
    tuner->stop();
}

TEST_F(IMCWithRelayTest, IsComplete) {
    bool complete = tuner->isComplete();
}

TEST_F(IMCWithRelayTest, GetIdentifiedModel) {
    auto model = tuner->getIdentifiedModel();
}

// ============================================================================
// FuzzyPID Tests
// ============================================================================

class FuzzyPIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<FuzzyPID>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<FuzzyPID> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(FuzzyPIDTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Fuzzy PID");
}

TEST_F(FuzzyPIDTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(FuzzyPIDTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(FuzzyPIDTest, SetBaseGains) {
    tuner->setBaseGains(1.0, 0.1, 0.01);
}

TEST_F(FuzzyPIDTest, SetAdjustmentFactors) {
    tuner->setAdjustmentFactors(0.5, 0.5, 0.5);
}

TEST_F(FuzzyPIDTest, SetErrorScale) {
    tuner->setErrorScale(2.0);
}

TEST_F(FuzzyPIDTest, GetCurrentGains) {
    auto gains = tuner->getCurrentGains();
}

TEST_F(FuzzyPIDTest, SetRuleComplexity) {
    tuner->setRuleComplexity(FuzzyPID::RuleComplexity::Simple);
    tuner->setRuleComplexity(FuzzyPID::RuleComplexity::Medium);
    tuner->setRuleComplexity(FuzzyPID::RuleComplexity::Complex);
}

TEST_F(FuzzyPIDTest, StartAndUpdate) {
    tuner->start();
    
    double result = tuner->update(0.5, 1.0, 0.3, DT);
    EXPECT_TRUE(std::isfinite(result));
    
    tuner->stop();
}

// ============================================================================
// GAPIDTuning Tests
// ============================================================================

class GAPIDTuningTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<GAPIDTuning>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
        model = std::make_unique<TestFOPDTProcessModel>(1.0, 10.0, 2.0);
    }
    
    std::unique_ptr<GAPIDTuning> tuner;
    std::shared_ptr<TestPIDController> controller;
    std::unique_ptr<TestFOPDTProcessModel> model;
};

TEST_F(GAPIDTuningTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "GA-Tuned PID");
}

TEST_F(GAPIDTuningTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(GAPIDTuningTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(GAPIDTuningTest, SetPopulationSize) {
    tuner->setPopulationSize(50);
}

TEST_F(GAPIDTuningTest, SetGenerations) {
    tuner->setGenerations(100);
}

TEST_F(GAPIDTuningTest, SetMutationRate) {
    tuner->setMutationRate(0.1);
}

TEST_F(GAPIDTuningTest, SetCrossoverRate) {
    tuner->setCrossoverRate(0.8);
}

TEST_F(GAPIDTuningTest, SetBounds) {
    tuner->setBounds(0.1, 10.0, 0.01, 1.0, 0.001, 0.1);
}

TEST_F(GAPIDTuningTest, SetElitism) {
    tuner->setElitism(0.15);
}

TEST_F(GAPIDTuningTest, GetBestFitnessHistory) {
    auto history = tuner->getBestFitnessHistory();
}

TEST_F(GAPIDTuningTest, Tune) {
    tuner->setGenerations(10);
    tuner->setPopulationSize(20);
    auto result = tuner->tune(*controller, model.get());
}

// ============================================================================
// PSOPIDTuning Tests
// ============================================================================

class PSOPIDTuningTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<PSOPIDTuning>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
        model = std::make_unique<TestFOPDTProcessModel>(1.0, 10.0, 2.0);
    }
    
    std::unique_ptr<PSOPIDTuning> tuner;
    std::shared_ptr<TestPIDController> controller;
    std::unique_ptr<TestFOPDTProcessModel> model;
};

TEST_F(PSOPIDTuningTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "PSO-Tuned PID");
}

TEST_F(PSOPIDTuningTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(PSOPIDTuningTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(PSOPIDTuningTest, SetSwarmSize) {
    tuner->setSwarmSize(40);
}

TEST_F(PSOPIDTuningTest, SetIterations) {
    tuner->setIterations(150);
}

TEST_F(PSOPIDTuningTest, SetInertia) {
    tuner->setInertia(0.7);
}

TEST_F(PSOPIDTuningTest, SetCognitiveCoeff) {
    tuner->setCognitiveCoeff(1.5);
}

TEST_F(PSOPIDTuningTest, SetSocialCoeff) {
    tuner->setSocialCoeff(1.5);
}

TEST_F(PSOPIDTuningTest, SetBounds) {
    tuner->setBounds(0.1, 10.0, 0.01, 1.0, 0.001, 0.1);
}

TEST_F(PSOPIDTuningTest, Tune) {
    tuner->setIterations(10);
    tuner->setSwarmSize(20);
    auto result = tuner->tune(*controller, model.get());
}

// ============================================================================
// NeuralPID Tests
// ============================================================================

class NeuralPIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<NeuralPID>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<NeuralPID> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(NeuralPIDTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Neural PID");
}

TEST_F(NeuralPIDTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(NeuralPIDTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(NeuralPIDTest, SetBaseGains) {
    tuner->setBaseGains(1.0, 0.1, 0.01);
}

TEST_F(NeuralPIDTest, SetNetworkArchitecture) {
    tuner->setNetworkArchitecture({8, 4});
}

TEST_F(NeuralPIDTest, SetLearningRate) {
    tuner->setLearningRate(0.01);
}

TEST_F(NeuralPIDTest, SetNNLimit) {
    tuner->setNNLimit(0.3);
}

TEST_F(NeuralPIDTest, EnableOnlineLearning) {
    tuner->enableOnlineLearning(true);
    tuner->enableOnlineLearning(false);
}

TEST_F(NeuralPIDTest, GetNNOutput) {
    double output = tuner->getNNOutput();
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(NeuralPIDTest, StartAndUpdate) {
    tuner->start();
    
    double result = tuner->update(0.5, 1.0, 0.3, DT);
    EXPECT_TRUE(std::isfinite(result));
    
    tuner->stop();
}

// ============================================================================
// CascadeAutotuner Tests
// ============================================================================

class CascadeAutotunerTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<CascadeAutotuner>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<CascadeAutotuner> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(CascadeAutotunerTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Cascade Autotuner");
}

TEST_F(CascadeAutotunerTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(CascadeAutotunerTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(CascadeAutotunerTest, SetBandwidthRatio) {
    tuner->setBandwidthRatio(0.2);
}

TEST_F(CascadeAutotunerTest, GetInnerGains) {
    // auto gains = tuner->getInnerGains(); // Not used
    tuner->getInnerGains();
}

TEST_F(CascadeAutotunerTest, GetOuterGains) {
    auto gains = tuner->getOuterGains();
}

// ============================================================================
// DecentralizedTuning Tests
// ============================================================================

class DecentralizedTuningTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<DecentralizedTuning>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<DecentralizedTuning> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(DecentralizedTuningTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Decentralized MIMO Tuning");
}

TEST_F(DecentralizedTuningTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(DecentralizedTuningTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(DecentralizedTuningTest, SetGainMatrix) {
    std::vector<std::vector<double>> K = {{1.0, 0.5}, {0.3, 1.2}};
    tuner->setGainMatrix(K);
}

TEST_F(DecentralizedTuningTest, SetTimeConstantMatrix) {
    std::vector<std::vector<double>> tau = {{5.0, 3.0}, {2.0, 4.0}};
    tuner->setTimeConstantMatrix(tau);
}

TEST_F(DecentralizedTuningTest, SetDelayMatrix) {
    std::vector<std::vector<double>> theta = {{0.5, 0.3}, {0.2, 0.4}};
    tuner->setDelayMatrix(theta);
}

TEST_F(DecentralizedTuningTest, SetDetuningFactor) {
    tuner->setDetuningFactor(0.6);
}

TEST_F(DecentralizedTuningTest, GetRGA) {
    auto rga = tuner->getRGA();
}

TEST_F(DecentralizedTuningTest, GetLoopPairing) {
    auto pairing = tuner->getLoopPairing();
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(HybridIntegration, ZNOptimizationFlow) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    ZNWithOptimization tuner;
    tuner.setInitialMethod(ZNWithOptimization::InitialMethod::ZNStepResponse);
    tuner.setMaxIterations(20);
    tuner.setRelativeBounds(0.5, 2.0);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    auto result = tuner.tune(*controller, &model);
}

TEST(HybridIntegration, GAPIDFlow) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    GAPIDTuning tuner;
    tuner.setPopulationSize(20);
    tuner.setGenerations(10);
    tuner.setMutationRate(0.1);
    tuner.setCrossoverRate(0.8);
    tuner.setBounds(0.1, 10.0, 0.01, 1.0, 0.001, 0.1);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    auto result = tuner.tune(*controller, &model);
}

TEST(HybridIntegration, FuzzyPIDFlow) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    FuzzyPID tuner;
    tuner.setBaseGains(1.0, 0.1, 0.01);
    tuner.setAdjustmentFactors(0.5, 0.5, 0.5);
    tuner.setErrorScale(10.0);
    tuner.setRuleComplexity(FuzzyPID::RuleComplexity::Medium);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    
    tuner.start();
    for (int i = 0; i < 100; ++i) {
        tuner.update(0.5 * (1.0 - 0.01 * i), 1.0, 0.5, DT);
    }
    tuner.stop();
}

TEST(HybridIntegration, NeuralPIDFlow) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    NeuralPID tuner;
    tuner.setBaseGains(1.0, 0.1, 0.01);
    tuner.setNetworkArchitecture({8, 4});
    tuner.setLearningRate(0.01);
    tuner.enableOnlineLearning(true);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    
    tuner.start();
    for (int i = 0; i < 100; ++i) {
        double error = std::sin(i * 0.1);
        tuner.update(50.0 + error, 50.0, 50.0, DT);
    }
    tuner.stop();
}

TEST(HybridIntegration, PSOPIDFlow) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    PSOPIDTuning tuner;
    tuner.setSwarmSize(20);
    tuner.setIterations(10);
    tuner.setBounds(0.1, 10.0, 0.01, 1.0, 0.001, 0.1);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    auto result = tuner.tune(*controller, &model);
}

// ============================================================================
// Additional Coverage Tests
// ============================================================================

TEST(ZNWithOptimizationCompute, TuneWithAllInitialMethods) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    // Test all initial methods
    ZNWithOptimization tuner;
    tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
    tuner.setMaxIterations(5);
    tuner.setRelativeBounds(0.5, 2.0);
    
    // ZN Step Response
    tuner.setInitialMethod(ZNWithOptimization::InitialMethod::ZNStepResponse);
    auto result1 = tuner.tune(*controller, &model);
    EXPECT_TRUE(result1.success);
    EXPECT_GT(tuner.getImprovementRatio(), -1.0);
    
    // ZN Ultimate Cycle
    tuner.setInitialMethod(ZNWithOptimization::InitialMethod::ZNUltimateCycle);
    auto result2 = tuner.tune(*controller, &model);
    
    // Cohen-Coon
    tuner.setInitialMethod(ZNWithOptimization::InitialMethod::CohenCoon);
    auto result3 = tuner.tune(*controller, &model);
    
    // IMC
    tuner.setInitialMethod(ZNWithOptimization::InitialMethod::IMC);
    auto result4 = tuner.tune(*controller, &model);
}

TEST(ZNWithOptimizationCompute, TuneWithNoCostFunction) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    ZNWithOptimization tuner;
    // Don't set cost function
    auto result = tuner.tune(*controller, &model);
    EXPECT_FALSE(result.success);
}

TEST(ZNWithOptimizationCompute, TuneWithNoModel) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    ZNWithOptimization tuner;
    tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
    auto result = tuner.tune(*controller, nullptr);
    EXPECT_FALSE(result.success);
}

TEST(ZNWithOptimizationCompute, GetInitialGains) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    ZNWithOptimization tuner;
    tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
    tuner.setMaxIterations(5);
    auto result = tuner.tune(*controller, &model);
    
    PIDGains initial = tuner.getInitialGains();
    // No getOptimizedGains() accessor - just verify result has parameters
    EXPECT_FALSE(result.parameters.empty());
}

TEST(IMCWithRelayCompute, FullRelaySequence) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    IMCWithRelay tuner;
    tuner.setRelayAmplitude(1.0);
    tuner.setRelayHysteresis(0.1);
    tuner.setIMCFactor(2.0);
    
    EXPECT_FALSE(tuner.isComplete());
    
    tuner.start();
    
    // Simulate relay test (oscillating signal)
    for (int i = 0; i < 1000; ++i) {
        double t = i * DT;
        double measured = std::sin(t) + 0.01 * std::sin(10 * t);  // Simulated oscillation
        tuner.update(measured, 0.0, 0.0, DT);
    }
    
    tuner.stop();
    
    // Get intermediate result
    auto result = tuner.getIntermediateResult();
    
    // Get identified model
    auto model = tuner.getIdentifiedModel();
}

TEST(FuzzyPIDCompute, ComputeFullRange) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    FuzzyPID tuner;
    tuner.setBaseGains(1.0, 0.1, 0.01);
    tuner.setAdjustmentFactors(0.8, 0.8, 0.8);
    tuner.setErrorScale(5.0);
    tuner.setRuleComplexity(FuzzyPID::RuleComplexity::Complex);
    
    tuner.start();
    
    // Test with various error/derivative combinations
    std::vector<std::pair<double, double>> testCases = {
        {-5.0, -2.0},  // Negative error, negative derivative
        {-5.0, 2.0},   // Negative error, positive derivative
        {5.0, -2.0},   // Positive error, negative derivative
        {5.0, 2.0},    // Positive error, positive derivative
        {0.0, 0.0},    // Zero error
        {0.5, 0.0},    // Small positive error
        {-0.5, 0.0},   // Small negative error
    };
    
    for (const auto& [error, deriv] : testCases) {
        double reference = 50.0;
        double measured = reference - error;
        double result = tuner.update(measured, reference, measured + deriv, DT);
        EXPECT_TRUE(std::isfinite(result));
    }
    
    tuner.stop();
    
    auto gains = tuner.getCurrentGains();
}

TEST(NeuralPIDCompute, OnlineLearning) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    NeuralPID tuner;
    tuner.setBaseGains(1.0, 0.1, 0.01);
    tuner.setNetworkArchitecture({16, 8, 4});
    tuner.setLearningRate(0.05);
    tuner.setNNLimit(0.5);
    tuner.enableOnlineLearning(true);
    
    tuner.start();
    
    // Training sequence with varied inputs
    for (int i = 0; i < 200; ++i) {
        double t = i * DT;
        double reference = 50.0 + 10.0 * std::sin(t * 0.5);
        double measured = reference * (1.0 - 0.1 * std::exp(-t));
        double prev = i > 0 ? reference * (1.0 - 0.1 * std::exp(-(i-1) * DT)) : 0.0;
        
        double result = tuner.update(measured, reference, prev, DT);
        EXPECT_TRUE(std::isfinite(result));
    }
    
    tuner.stop();
    
    double nnOutput = tuner.getNNOutput();
    EXPECT_TRUE(std::isfinite(nnOutput));
}

TEST(GAPIDTuningCompute, TuneWithCustomBounds) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    GAPIDTuning tuner;
    tuner.setPopulationSize(30);
    tuner.setGenerations(20);
    tuner.setMutationRate(0.15);
    tuner.setCrossoverRate(0.7);
    tuner.setElitism(0.1);
    tuner.setBounds(0.1, 20.0, 0.01, 2.0, 0.0, 0.2);
    
    auto result = tuner.tune(*controller, &model);
    
    auto history = tuner.getBestFitnessHistory();
}

TEST(PSOPIDTuningCompute, TuneWithInertiaDecay) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    PSOPIDTuning tuner;
    tuner.setSwarmSize(30);
    tuner.setIterations(25);
    tuner.setInertia(0.9);
    tuner.setCognitiveCoeff(2.0);
    tuner.setSocialCoeff(2.0);
    tuner.setBounds(0.1, 20.0, 0.01, 2.0, 0.0, 0.2);
    
    auto result = tuner.tune(*controller, &model);
}

TEST(CascadeAutotunerCompute, TuneSequentially) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    CascadeAutotuner tuner;
    tuner.setBandwidthRatio(0.15);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    
    // auto innerGains = tuner.getInnerGains(); // Not used
    tuner.getInnerGains();
    auto outerGains = tuner.getOuterGains();
}

TEST(DecentralizedTuningCompute, SetMatricesAndCompute) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    DecentralizedTuning tuner;
    
    // 2x2 MIMO system
    std::vector<std::vector<double>> K = {{2.0, 0.5}, {0.3, 1.5}};
    std::vector<std::vector<double>> tau = {{5.0, 3.0}, {2.5, 4.0}};
    std::vector<std::vector<double>> theta = {{0.5, 0.3}, {0.2, 0.4}};
    
    tuner.setGainMatrix(K);
    tuner.setTimeConstantMatrix(tau);
    tuner.setDelayMatrix(theta);
    tuner.setDetuningFactor(0.5);
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    
    auto rga = tuner.getRGA();
    auto pairing = tuner.getLoopPairing();
}

// Test different cost function combinations
TEST(HybridMethodsCostFunctions, ZNWithDifferentCosts) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    // Test with simple quadratic cost
    {
        ZNWithOptimization tuner;
        tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
        tuner.setMaxIterations(5);
        auto result = tuner.tune(*controller, &model);
    }
}

// ============================================================================
// Additional Coverage Tests for HybridMethods
// ============================================================================

TEST(IMCWithRelayCoverage, FullOnlineSequence) {
    IMCWithRelay tuner;
    tuner.setRelayAmplitude(2.0);
    tuner.setRelayHysteresis(0.5);
    tuner.setIMCFactor(1.2);
    
    tuner.start();
    
    // Run relay experiment
    for (int i = 0; i < 400; ++i) {
        double t = i * 0.1;
        double measured = 50.0 + 3.0 * std::sin(0.5 * t);
        tuner.update(measured, 50.0, 0.0, 0.1);
    }
    
    tuner.stop();
    
    // bool complete = tuner.isComplete(); // Not used
    tuner.isComplete();
    auto result = tuner.getIntermediateResult();
    // auto model = tuner.getIdentifiedModel(); // Not used
    tuner.getIdentifiedModel();
}

TEST(FuzzyPIDCoverage, AllRuleComplexities) {
    std::vector<FuzzyPID::RuleComplexity> complexities = {
        FuzzyPID::RuleComplexity::Simple,
        FuzzyPID::RuleComplexity::Medium,
        FuzzyPID::RuleComplexity::Complex
    };
    
    for (auto complexity : complexities) {
        FuzzyPID tuner;
        tuner.setBaseGains(2.0, 0.5, 0.1);
        tuner.setAdjustmentFactors(0.6, 0.4, 0.3);
        tuner.setErrorScale(2.0);
        tuner.setRuleComplexity(complexity);
        
        tuner.start();
        
        for (int i = 0; i < 100; ++i) {
            double t = i * 0.1;
            double reference = 50.0;
            double measured = reference - 5.0 * std::exp(-t * 0.5);
            double prev = i > 0 ? reference - 5.0 * std::exp(-(i-1) * 0.1 * 0.5) : 0.0;
            
            tuner.update(measured, reference, prev, 0.1);
        }
        
        tuner.stop();
        
        auto currentGains = tuner.getCurrentGains();
    }
}

TEST(NeuralPIDCoverage, PretrainAndRun) {
    NeuralPID tuner;
    tuner.setBaseGains(1.5, 0.2, 0.05);
    tuner.setNetworkArchitecture({12, 8});
    tuner.setLearningRate(0.02);
    tuner.setNNLimit(0.3);
    tuner.enableOnlineLearning(true);
    
    // Pretrain with some data
    std::vector<std::vector<double>> inputs;
    std::vector<double> targets;
    
    for (int i = 0; i < 50; ++i) {
        double err = (i - 25.0) / 25.0;
        double derr = 0.1 * std::cos(i * 0.1);
        inputs.push_back({err, derr});
        targets.push_back(0.1 * err + 0.05 * derr);
    }
    
    tuner.pretrain(inputs, targets);
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        double reference = 50.0;
        double measured = reference - 3.0 * std::exp(-t * 0.3);
        double prev = i > 0 ? reference - 3.0 * std::exp(-(i-1) * 0.1 * 0.3) : 0.0;
        
        double result = tuner.update(measured, reference, prev, 0.1);
        EXPECT_TRUE(std::isfinite(result));
    }
    
    tuner.stop();
    
    bool complete = tuner.isComplete();
    auto result = tuner.getIntermediateResult();
}

TEST(GAPIDTuningCoverage, DifferentCostFunctions) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    GAPIDTuning tuner;
    tuner.setPopulationSize(20);
    tuner.setGenerations(10);
    tuner.setMutationRate(0.2);
    tuner.setCrossoverRate(0.75);
    tuner.setElitism(0.15);
    tuner.setBounds(0.5, 15.0, 0.05, 1.5, 0.0, 0.15);
    tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
    
    auto result = tuner.tune(*controller, &model);
    
    auto history = tuner.getBestFitnessHistory();
}

TEST(PSOPIDTuningCoverage, DifferentCostFunctions) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    PSOPIDTuning tuner;
    tuner.setSwarmSize(20);
    tuner.setIterations(15);
    tuner.setInertia(0.8);
    tuner.setCognitiveCoeff(1.8);
    tuner.setSocialCoeff(1.8);
    tuner.setBounds(0.5, 15.0, 0.05, 1.5, 0.0, 0.15);
    tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
    
    auto result = tuner.tune(*controller, &model);
}

TEST(CascadeAutotunerCoverage, WithCustomTuners) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    CascadeAutotuner tuner;
    tuner.setBandwidthRatio(0.2);
    
    auto innerTuner = std::make_unique<ZNWithOptimization>();
    innerTuner->setMaxIterations(5);
    tuner.setInnerTuner(std::move(innerTuner));
    
    auto outerTuner = std::make_unique<ZNWithOptimization>();
    outerTuner->setMaxIterations(5);
    tuner.setOuterTuner(std::move(outerTuner));
    
    EXPECT_TRUE(tuner.isCompatible(*controller));
    
    auto innerGains = tuner.getInnerGains();
    auto outerGains = tuner.getOuterGains();
}

TEST(DecentralizedTuningCoverage, FullMIMOSetup) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    DecentralizedTuning tuner;
    
    // 3x3 MIMO system
    std::vector<std::vector<double>> K = {
        {2.0, 0.5, 0.2},
        {0.3, 1.5, 0.4},
        {0.1, 0.3, 1.8}
    };
    std::vector<std::vector<double>> tau = {
        {5.0, 3.0, 2.0},
        {2.5, 4.0, 3.0},
        {3.0, 2.5, 5.5}
    };
    std::vector<std::vector<double>> theta = {
        {0.5, 0.3, 0.2},
        {0.2, 0.4, 0.3},
        {0.3, 0.25, 0.5}
    };
    
    tuner.setGainMatrix(K);
    tuner.setTimeConstantMatrix(tau);
    tuner.setDelayMatrix(theta);
    tuner.setDetuningFactor(0.4);
    
    auto sisoTuner = std::make_unique<ZNWithOptimization>();
    sisoTuner->setMaxIterations(5);
    tuner.setSISOTuner(std::move(sisoTuner));
    
    auto rga = tuner.getRGA();
    auto pairing = tuner.getLoopPairing();
}

TEST(ZNWithOptimizationCoverage, AllInitialMethods) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    std::vector<ZNWithOptimization::InitialMethod> methods = {
        ZNWithOptimization::InitialMethod::ZNStepResponse,
        ZNWithOptimization::InitialMethod::ZNUltimateCycle,
        ZNWithOptimization::InitialMethod::CohenCoon,
        ZNWithOptimization::InitialMethod::IMC
    };
    
    for (auto method : methods) {
        ZNWithOptimization tuner;
        tuner.setInitialMethod(method);
        tuner.setRelativeBounds(0.3, 3.0);
        tuner.setMaxIterations(10);
        tuner.setCostFunction(std::make_unique<SimpleQuadraticCost>());
        
        auto result = tuner.tune(*controller, &model);
        
        auto initialGains = tuner.getInitialGains();
        double ratio = tuner.getImprovementRatio();
        EXPECT_TRUE(std::isfinite(ratio) || ratio > 0.0 || std::isnan(ratio));
    }
}

TEST(HybridMethodsFactoryCoverage, CreateAllTuners) {
    auto znOpt = std::make_unique<ZNWithOptimization>();
    auto imcRelay = std::make_unique<IMCWithRelay>();
    auto fuzzyPID = std::make_unique<FuzzyPID>();
    auto gaPID = std::make_unique<GAPIDTuning>();
    auto psoPID = std::make_unique<PSOPIDTuning>();
    auto neuralPID = std::make_unique<NeuralPID>();
    auto cascadeAuto = std::make_unique<CascadeAutotuner>();
    auto decentralized = std::make_unique<DecentralizedTuning>();
    
    EXPECT_FALSE(znOpt->getName().empty());
    EXPECT_FALSE(imcRelay->getName().empty());
    EXPECT_FALSE(fuzzyPID->getName().empty());
    EXPECT_FALSE(gaPID->getName().empty());
    EXPECT_FALSE(psoPID->getName().empty());
    EXPECT_FALSE(neuralPID->getName().empty());
    EXPECT_FALSE(cascadeAuto->getName().empty());
    EXPECT_FALSE(decentralized->getName().empty());
    
    EXPECT_FALSE(znOpt->getDescription().empty());
    EXPECT_FALSE(imcRelay->getDescription().empty());
    EXPECT_FALSE(fuzzyPID->getDescription().empty());
    EXPECT_FALSE(gaPID->getDescription().empty());
    EXPECT_FALSE(psoPID->getDescription().empty());
    EXPECT_FALSE(neuralPID->getDescription().empty());
    EXPECT_FALSE(cascadeAuto->getDescription().empty());
    EXPECT_FALSE(decentralized->getDescription().empty());
}

// ============================================================================
// Additional Hybrid Methods Coverage Tests
// ============================================================================

TEST(IMCWithRelayCoverage, FullRelayExperiment) {
    IMCWithRelay tuner;
    
    tuner.setRelayAmplitude(1.0);
    tuner.setRelayHysteresis(0.05);
    tuner.setIMCFactor(0.5);
    
    tuner.start();
    
    // Simulate relay oscillation
    double y = 50.0;
    double u = 0.0;
    for (int i = 0; i < 500; ++i) {
        // Plant simulation
        double e = 50.0 - y;
        u = tuner.update(y, 50.0, u, DT);
        y += DT * (0.5 * u - 0.1 * y);
    }
    
    auto result = tuner.getIntermediateResult();
    auto model = tuner.getIdentifiedModel();
    
    tuner.stop();
}

TEST(IMCWithRelayCoverage, DifferentAmplitudes) {
    IMCWithRelay tuner;
    
    tuner.setRelayAmplitude(1.5);
    tuner.setRelayHysteresis(0.02);
    tuner.setIMCFactor(0.8);
    
    TestPIDController controller;
    TestFOPDTProcessModel model(1.5, 8.0, 1.5);
    
    auto result = tuner.tune(controller, &model);
    
    tuner.start();
    for (int i = 0; i < 300; ++i) {
        tuner.update(50.0 + 3.0 * std::sin(0.2 * i), 50.0, 0.5, DT);
    }
    tuner.stop();
}

TEST(FuzzyPIDCoverage, AllRuleBases) {
    std::vector<FuzzyPID::RuleComplexity> complexities = {
        FuzzyPID::RuleComplexity::Simple,
        FuzzyPID::RuleComplexity::Medium,
        FuzzyPID::RuleComplexity::Complex
    };
    
    for (auto complexity : complexities) {
        FuzzyPID fuzzy;
        fuzzy.setRuleComplexity(complexity);
        fuzzy.setBaseGains(1.0, 0.1, 0.05);
        fuzzy.setAdjustmentFactors(0.5, 0.3, 0.2);
        fuzzy.setErrorScale(1.0);
        
        TestPIDController controller;
        TestFOPDTProcessModel model(1.0, 5.0, 1.0);
        
        auto result = fuzzy.tune(controller, &model);
        
        fuzzy.start();
        
        for (int i = 0; i < 100; ++i) {
            double y = 50.0 + 5.0 * std::sin(0.1 * i);
            fuzzy.update(y, 55.0, 0.5, DT);
        }
        
        fuzzy.stop();
    }
}

TEST(FuzzyPIDCoverage, OnlineGains) {
    FuzzyPID fuzzy;
    fuzzy.setRuleComplexity(FuzzyPID::RuleComplexity::Medium);
    fuzzy.setBaseGains(1.0, 0.1, 0.05);
    fuzzy.setAdjustmentFactors(0.3, 0.2, 0.1);
    
    fuzzy.start();
    
    // Simulate with changing conditions
    for (int i = 0; i < 200; ++i) {
        double y = 50.0 + 5.0 * std::sin(0.1 * i);
        double ref = (i < 100) ? 50.0 : 60.0;  // Step change
        fuzzy.update(y, ref, 0.0, DT);
    }
    
    auto gains = fuzzy.getCurrentGains();
    fuzzy.stop();
}

TEST(GAPIDTuningCoverage, AdvancedGenetics) {
    GAPIDTuning ga;
    
    ga.setPopulationSize(40);
    ga.setGenerations(30);
    ga.setMutationRate(0.15);
    ga.setCrossoverRate(0.85);
    ga.setElitism(0.1);
    ga.setBounds(0.1, 20.0, 0.01, 2.0, 0.0, 0.2);
    
    ga.setCostFunction(std::make_unique<SimpleQuadraticCost>());
    
    TestPIDController controller;
    TestFOPDTProcessModel model(2.0, 15.0, 3.0);
    
    auto result = ga.tune(controller, &model);
    
    auto history = ga.getBestFitnessHistory();
}

TEST(GAPIDTuningCoverage, DifferentParameters) {
    for (int p = 20; p <= 40; p += 10) {
        GAPIDTuning ga;
        ga.setPopulationSize(p);
        ga.setGenerations(15);
        ga.setMutationRate(0.1);
        ga.setCrossoverRate(0.8);
        ga.setBounds(0.1, 10.0, 0.01, 1.0, 0.0, 0.1);
        
        TestPIDController controller;
        TestFOPDTProcessModel model(1.0, 5.0, 1.0);
        
        auto result = ga.tune(controller, &model);
    }
}

TEST(PSOPIDTuningCoverage, AdvancedPSO) {
    PSOPIDTuning pso;
    
    pso.setSwarmSize(35);
    pso.setIterations(40);
    pso.setInertia(0.85);
    pso.setCognitiveCoeff(2.0);
    pso.setSocialCoeff(2.0);
    pso.setBounds(0.1, 25.0, 0.01, 2.5, 0.0, 0.25);
    
    TestPIDController controller;
    TestFOPDTProcessModel model(1.5, 12.0, 2.5);
    
    auto result = pso.tune(controller, &model);
}

TEST(PSOPIDTuningCoverage, WithParameters) {
    PSOPIDTuning pso;
    
    pso.setSwarmSize(25);
    pso.setIterations(25);
    pso.setInertia(0.7);
    pso.setCognitiveCoeff(1.8);
    pso.setSocialCoeff(1.8);
    pso.setBounds(0.1, 10.0, 0.01, 1.0, 0.0, 0.1);
    
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0, 8.0, 1.5);
    
    auto result = pso.tune(controller, &model);
}

TEST(NeuralPIDCoverage, DifferentArchitectures) {
    std::vector<std::vector<int>> archs = {
        {4, 3},
        {8, 6, 3},
        {12, 8, 6, 3}
    };
    
    for (const auto& arch : archs) {
        NeuralPID neural;
        neural.setNetworkArchitecture(arch);
        neural.setLearningRate(0.01);
        
        TestPIDController controller;
        TestFOPDTProcessModel model(1.0, 5.0, 1.0);
        
        auto result = neural.tune(controller, &model);
        
        neural.start();
        for (int i = 0; i < 50; ++i) {
            double y = 50.0 + 3.0 * std::sin(0.1 * i);
            neural.update(y, 55.0, 0.5, DT);
        }
        neural.stop();
    }
}

TEST(NeuralPIDCoverage, OnlineLearning) {
    NeuralPID neural;
    neural.setNetworkArchitecture({8, 6, 3});
    neural.setLearningRate(0.005);
    neural.enableOnlineLearning(true);
    
    neural.start();
    
    for (int i = 0; i < 300; ++i) {
        double setpoint = (i < 100) ? 50.0 : ((i < 200) ? 60.0 : 45.0);
        double y = 50.0 + (i > 100 ? (1.0 - std::exp(-(i-100)*0.02)) * 10.0 : 0.0);
        neural.update(y, setpoint, 0.5, DT);
    }
    
    auto result = neural.getIntermediateResult();
    neural.stop();
}

TEST(CascadeAutotunerCoverage, FullCascade) {
    CascadeAutotuner cascade;
    
    cascade.setBandwidthRatio(0.15);
    
    // Inner loop tuner
    auto inner = std::make_unique<ZNWithOptimization>();
    inner->setMaxIterations(10);
    inner->setInitialMethod(ZNWithOptimization::InitialMethod::IMC);
    cascade.setInnerTuner(std::move(inner));
    
    // Outer loop tuner
    auto outer = std::make_unique<ZNWithOptimization>();
    outer->setMaxIterations(10);
    outer->setInitialMethod(ZNWithOptimization::InitialMethod::CohenCoon);
    cascade.setOuterTuner(std::move(outer));
    
    TestPIDController controller;
    TestFOPDTProcessModel model(2.0, 10.0, 1.5);
    auto result = cascade.tune(controller, &model);
    
    auto innerGains = cascade.getInnerGains();
    // auto outerGains = cascade.getOuterGains(); // Not used
    cascade.getOuterGains();
}

TEST(DecentralizedTuningCoverage, RGAAnalysis) {
    DecentralizedTuning tuner;
    
    // 2x2 system with significant interaction
    std::vector<std::vector<double>> K = {{2.0, 0.8}, {0.6, 1.5}};
    std::vector<std::vector<double>> tau = {{5.0, 3.0}, {4.0, 6.0}};
    std::vector<std::vector<double>> theta = {{1.0, 0.5}, {0.8, 1.2}};
    
    tuner.setGainMatrix(K);
    tuner.setTimeConstantMatrix(tau);
    tuner.setDelayMatrix(theta);
    
    auto rga = tuner.getRGA();
    
    auto pairing = tuner.getLoopPairing();
}

TEST(DecentralizedTuningCoverage, MultiLoopDetuning) {
    DecentralizedTuning tuner;
    
    // 3x3 system
    std::vector<std::vector<double>> K = {
        {1.0, 0.3, 0.1},
        {0.2, 1.2, 0.2},
        {0.15, 0.25, 0.9}
    };
    std::vector<std::vector<double>> tau = {
        {5.0, 3.0, 2.0},
        {3.5, 6.0, 4.0},
        {2.5, 3.5, 5.0}
    };
    std::vector<std::vector<double>> theta = {
        {0.5, 0.3, 0.2},
        {0.4, 0.6, 0.3},
        {0.3, 0.4, 0.5}
    };
    
    tuner.setGainMatrix(K);
    tuner.setTimeConstantMatrix(tau);
    tuner.setDelayMatrix(theta);
    tuner.setDetuningFactor(0.5);
    
    auto sisoTuner = std::make_unique<ZNWithOptimization>();
    sisoTuner->setMaxIterations(10);
    tuner.setSISOTuner(std::move(sisoTuner));
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

TEST(HybridEdgeCases, NullModel) {
    ZNWithOptimization tuner;
    tuner.setMaxIterations(5);
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

TEST(HybridEdgeCases, EmptyGainMatrix) {
    DecentralizedTuning tuner;
    
    // Empty matrices
    tuner.setGainMatrix({});
    tuner.setTimeConstantMatrix({});
    tuner.setDelayMatrix({});
    
    auto rga = tuner.getRGA();
    auto pairing = tuner.getLoopPairing();
}

TEST(HybridEdgeCases, ZeroParameterBounds) {
    GAPIDTuning ga;
    ga.setPopulationSize(10);
    ga.setGenerations(5);
    ga.setBounds(0.0, 10.0, 0.0, 1.0, 0.0, 0.1);
    
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0, 5.0, 1.0);
    
    auto result = ga.tune(controller, &model);
}

TEST(HybridIntegration, GAFollowedByPSO) {
    TestPIDController controller;
    TestFOPDTProcessModel model(1.5, 10.0, 2.0);
    
    // First pass with GA
    GAPIDTuning ga;
    ga.setPopulationSize(20);
    ga.setGenerations(15);
    ga.setBounds(0.1, 10.0, 0.01, 1.0, 0.0, 0.1);
    
    auto gaResult = ga.tune(controller, &model);
    
    // Refine with PSO
    PSOPIDTuning pso;
    pso.setSwarmSize(15);
    pso.setIterations(20);
    
    // Narrow bounds around GA solution
    if (gaResult.parameters.size() >= 3) {
        double kp = gaResult.parameters[0];
        double ki = gaResult.parameters[1];
        double kd = gaResult.parameters[2];
        pso.setBounds(
            0.5 * kp, 1.5 * kp,
            0.5 * ki, 1.5 * ki,
            0.5 * kd, 1.5 * kd + 0.01
        );
    }
    
    auto psoResult = pso.tune(controller, &model);
}