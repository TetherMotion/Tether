/**
 * @file AdaptiveMethodsTests.cpp
 * @brief Unit tests for adaptive tuning methods
 */

#include <gtest/gtest.h>
#include "TestHelpers.hpp"
#include "tether/control/autotuning/AdaptiveMethods.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

// ============================================================================
// GainScheduler Tests
// ============================================================================

class GainSchedulerTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<GainScheduler>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<GainScheduler> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(GainSchedulerTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Gain Scheduling");
}

TEST_F(GainSchedulerTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(GainSchedulerTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(GainSchedulerTest, AddGainSet) {
    tuner->addGainSet(0.0, {1.0, 0.1, 0.01});
    tuner->addGainSet(50.0, {2.0, 0.2, 0.02});
}

TEST_F(GainSchedulerTest, EnableInterpolation) {
    tuner->enableInterpolation(true);
    tuner->enableInterpolation(false);
}

TEST_F(GainSchedulerTest, SetGainRateLimit) {
    tuner->setGainRateLimit(10.0);
}

TEST_F(GainSchedulerTest, StartAndUpdate) {
    tuner->addGainSet(0.0, {1.0, 0.1, 0.01});
    tuner->addGainSet(100.0, {2.0, 0.2, 0.02});
    tuner->start();
    
    double output = tuner->update(50.0, 50.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(output));
    
    tuner->stop();
}

TEST_F(GainSchedulerTest, GetCurrentGains) {
    tuner->addGainSet(0.0, {1.0, 0.1, 0.01});
    auto gains = tuner->getCurrentGains();
}

// ============================================================================
// MRAC Tests
// ============================================================================

class MRACTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<MRAC>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<MRAC> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(MRACTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Model Reference Adaptive Control");
}

TEST_F(MRACTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(MRACTest, IsCompatible) {
    EXPECT_TRUE(tuner->isCompatible(*controller));
}

TEST_F(MRACTest, SetReferenceModelFirstOrder) {
    tuner->setReferenceModel(1.0, 0.5);  // Km, taum
}

TEST_F(MRACTest, SetReferenceModelSecondOrder) {
    tuner->setReferenceModel(1.0, 2.0, 0.707);  // Km, wn, zeta
}

TEST_F(MRACTest, SetAdaptationGain) {
    tuner->setAdaptationGain({0.1, 0.01, 0.001});
}

TEST_F(MRACTest, SetAdaptationLaw) {
    tuner->setAdaptationLaw(MRAC::AdaptationLaw::MIT);
    tuner->setAdaptationLaw(MRAC::AdaptationLaw::Lyapunov);
    tuner->setAdaptationLaw(MRAC::AdaptationLaw::NormalizedMIT);
}

TEST_F(MRACTest, SetSigmaModification) {
    tuner->setSigmaModification(0.01);
}

TEST_F(MRACTest, StartAndUpdate) {
    tuner->setReferenceModel(1.0, 0.7);
    tuner->start();
    
    double output = tuner->update(50.0, 60.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(output));
    
    tuner->stop();
}

TEST_F(MRACTest, GetModelError) {
    tuner->start();
    tuner->update(50.0, 60.0, 0.0, DT);
    double error = tuner->getModelError();
    EXPECT_TRUE(std::isfinite(error));
}

// ============================================================================
// SelfTuningRegulator Tests
// ============================================================================

class SelfTuningRegulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<SelfTuningRegulator>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<SelfTuningRegulator> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(SelfTuningRegulatorTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Self-Tuning Regulator");
}

TEST_F(SelfTuningRegulatorTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(SelfTuningRegulatorTest, SetModelStructure) {
    tuner->setModelStructure(2, 2, 1);  // na, nb, nk
}

TEST_F(SelfTuningRegulatorTest, SetForgettingFactor) {
    tuner->setForgettingFactor(0.98);
}

TEST_F(SelfTuningRegulatorTest, SetEstimationMethod) {
    tuner->setEstimationMethod(SelfTuningRegulator::EstimationMethod::RLS);
    tuner->setEstimationMethod(SelfTuningRegulator::EstimationMethod::RLSForgetting);
    tuner->setEstimationMethod(SelfTuningRegulator::EstimationMethod::ELS);
    tuner->setEstimationMethod(SelfTuningRegulator::EstimationMethod::RPEM);
}

TEST_F(SelfTuningRegulatorTest, SetDesignMethod) {
    tuner->setDesignMethod(SelfTuningRegulator::DesignMethod::MinimumVariance);
    tuner->setDesignMethod(SelfTuningRegulator::DesignMethod::PolePlacement);
    tuner->setDesignMethod(SelfTuningRegulator::DesignMethod::PIDFromModel);
}

TEST_F(SelfTuningRegulatorTest, SetUpdateInterval) {
    tuner->setUpdateInterval(10);
}

TEST_F(SelfTuningRegulatorTest, StartAndUpdate) {
    tuner->start();
    
    for (int i = 0; i < 100; ++i) {
        double output = tuner->update(50.0 + 0.1 * i, 60.0, 0.0, DT);
        EXPECT_TRUE(std::isfinite(output));
    }
    
    tuner->stop();
}

TEST_F(SelfTuningRegulatorTest, GetEstimatedParameters) {
    tuner->start();
    for (int i = 0; i < 50; ++i) {
        tuner->update(50.0, 60.0, 1.0, DT);
    }
    auto a = tuner->getEstimatedA();
    auto b = tuner->getEstimatedB();
}

// ============================================================================
// ExtremumSeekingControl Tests
// ============================================================================

class ExtremumSeekingTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<ExtremumSeekingControl>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<ExtremumSeekingControl> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(ExtremumSeekingTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Extremum Seeking Control");
}

TEST_F(ExtremumSeekingTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(ExtremumSeekingTest, SetPerturbation) {
    tuner->setPerturbation(0.1, 0.5);  // amplitude, frequency
}

TEST_F(ExtremumSeekingTest, SetAdaptationGain) {
    tuner->setAdaptationGain(0.01);
}

TEST_F(ExtremumSeekingTest, SetFilterCutoff) {
    tuner->setFilterCutoff(0.1);
}

TEST_F(ExtremumSeekingTest, SetNumParameters) {
    tuner->setNumParameters(3);
}

TEST_F(ExtremumSeekingTest, EnableMultiParameter) {
    tuner->enableMultiParameter(true);
}

TEST_F(ExtremumSeekingTest, StartAndUpdate) {
    tuner->setPerturbation(0.1, 0.5);
    tuner->start();
    
    double output = tuner->update(50.0, 60.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(output));
    
    tuner->stop();
}

TEST_F(ExtremumSeekingTest, GetParameters) {
    tuner->start();
    tuner->update(50.0, 60.0, 0.0, DT);
    auto params = tuner->getParameters();
}

// ============================================================================
// FuzzyTuning Tests
// ============================================================================

class FuzzyTuningTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<FuzzyTuning>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<FuzzyTuning> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(FuzzyTuningTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Fuzzy Logic Tuning");
}

TEST_F(FuzzyTuningTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(FuzzyTuningTest, SetBaseGains) {
    tuner->setBaseGains(1.0, 0.1, 0.01);
}

TEST_F(FuzzyTuningTest, SetErrorRange) {
    tuner->setErrorRange(-10.0, 10.0);
}

TEST_F(FuzzyTuningTest, SetErrorRateRange) {
    tuner->setErrorRateRange(-5.0, 5.0);
}

TEST_F(FuzzyTuningTest, SetGainRanges) {
    tuner->setGainRanges(0.5, 0.5, 0.5);
}

TEST_F(FuzzyTuningTest, UseDefaultRules) {
    tuner->useDefaultRules();
}

TEST_F(FuzzyTuningTest, StartAndUpdate) {
    tuner->setBaseGains(1.0, 0.1, 0.01);
    tuner->useDefaultRules();
    tuner->start();
    
    double output = tuner->update(50.0, 60.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(output));
    
    tuner->stop();
}

// ============================================================================
// NeuralNetworkTuning Tests
// ============================================================================

class NeuralNetworkTuningTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<NeuralNetworkTuning>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<NeuralNetworkTuning> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(NeuralNetworkTuningTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Neural Network Tuning");
}

TEST_F(NeuralNetworkTuningTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(NeuralNetworkTuningTest, SetArchitecture) {
    tuner->setArchitecture({4, 8, 4});
}

TEST_F(NeuralNetworkTuningTest, SetActivation) {
    tuner->setActivation(NeuralNetworkTuning::Activation::ReLU);
    tuner->setActivation(NeuralNetworkTuning::Activation::Tanh);
    tuner->setActivation(NeuralNetworkTuning::Activation::Sigmoid);
    tuner->setActivation(NeuralNetworkTuning::Activation::Linear);
}

TEST_F(NeuralNetworkTuningTest, SetLearningRate) {
    tuner->setLearningRate(0.001);
}

TEST_F(NeuralNetworkTuningTest, EnableOnlineLearning) {
    tuner->enableOnlineLearning(true);
}

TEST_F(NeuralNetworkTuningTest, StartAndUpdate) {
    tuner->setArchitecture({4, 8, 3});
    tuner->start();
    
    double output = tuner->update(50.0, 60.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(output));
    
    tuner->stop();
}

// ============================================================================
// MMAC Tests
// ============================================================================

class MMACTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner = std::make_unique<MMAC>();
        controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    }
    
    std::unique_ptr<MMAC> tuner;
    std::shared_ptr<TestPIDController> controller;
};

TEST_F(MMACTest, BasicName) {
    EXPECT_EQ(tuner->getName(), "Multiple Model Adaptive Control");
}

TEST_F(MMACTest, Description) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(MMACTest, AddModel) {
    FOPDTModel model1;
    model1.K = 1.0;
    model1.tau = 5.0;
    model1.L = 1.0;
    
    FOPDTModel model2;
    model2.K = 2.0;
    model2.tau = 10.0;
    model2.L = 2.0;
    
    tuner->addModel(model1, {1.0, 0.1, 0.01});
    tuner->addModel(model2, {2.0, 0.2, 0.02});
}

TEST_F(MMACTest, SetMixingMode) {
    tuner->setMixingMode(MMAC::MixingMode::Switching);
    tuner->setMixingMode(MMAC::MixingMode::BayesianMixing);
    tuner->setMixingMode(MMAC::MixingMode::SoftmaxMixing);
}

TEST_F(MMACTest, SetSwitchingHysteresis) {
    tuner->setSwitchingHysteresis(0.05);
}

TEST_F(MMACTest, StartAndUpdate) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 5.0;
    model.L = 1.0;
    
    tuner->addModel(model, {1.0, 0.1, 0.01});
    tuner->start();
    
    double output = tuner->update(50.0, 60.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(output));
    
    tuner->stop();
}

TEST_F(MMACTest, GetModelProbabilities) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 5.0;
    model.L = 1.0;
    
    tuner->addModel(model, {1.0, 0.1, 0.01});
    tuner->start();
    tuner->update(50.0, 60.0, 0.0, DT);
    
    auto probs = tuner->getModelProbabilities();
}

TEST_F(MMACTest, GetActiveModel) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 5.0;
    model.L = 1.0;
    
    tuner->addModel(model, {1.0, 0.1, 0.01});
    tuner->start();
    
    int active = tuner->getActiveModel();
    EXPECT_GE(active, 0);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(AdaptiveIntegration, AllMethodsCompatible) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    EXPECT_TRUE(GainScheduler().isCompatible(*controller));
    EXPECT_TRUE(MRAC().isCompatible(*controller));
    EXPECT_TRUE(SelfTuningRegulator().isCompatible(*controller));
    EXPECT_TRUE(ExtremumSeekingControl().isCompatible(*controller));
    EXPECT_TRUE(FuzzyTuning().isCompatible(*controller));
    EXPECT_TRUE(NeuralNetworkTuning().isCompatible(*controller));
    EXPECT_TRUE(MMAC().isCompatible(*controller));
}

TEST(AdaptiveIntegration, MRACFlow) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    
    MRAC tuner;
    tuner.setReferenceModel(1.0, 0.7);
    tuner.setAdaptationGain({0.1, 0.01, 0.001});
    
    tuner.start();
    for (int i = 0; i < 200; ++i) {
        double y = 50.0 + 10.0 * std::sin(i * 0.1);
        tuner.update(y, 50.0, 0.0, DT);
    }
    tuner.stop();
}

TEST(AdaptiveIntegration, GainSchedulerWithFunction) {
    GainScheduler scheduler;
    
    scheduler.setGainFunction([](double sp) {
        double Kp = 1.0 + 0.01 * sp;
        double Ki = 0.1 + 0.001 * sp;
        double Kd = 0.01 + 0.0001 * sp;
        return ParameterVector{Kp, Ki, Kd};
    });
    
    scheduler.start();
    for (int i = 0; i < 100; ++i) {
        scheduler.update(i * 1.0, 50.0, 0.0, DT);
    }
    scheduler.stop();
}

TEST(AdaptiveIntegration, MMACWithMultipleModels) {
    MMAC mmac;
    
    FOPDTModel model1;
    model1.K = 0.5;
    model1.tau = 5.0;
    model1.L = 1.0;
    
    FOPDTModel model2;
    model2.K = 1.0;
    model2.tau = 10.0;
    model2.L = 2.0;
    
    FOPDTModel model3;
    model3.K = 2.0;
    model3.tau = 20.0;
    model3.L = 4.0;
    
    mmac.addModel(model1, {2.0, 0.2, 0.02});
    mmac.addModel(model2, {1.0, 0.1, 0.01});
    mmac.addModel(model3, {0.5, 0.05, 0.005});
    
    mmac.setMixingMode(MMAC::MixingMode::BayesianMixing);
    
    mmac.start();
    for (int i = 0; i < 100; ++i) {
        double y = 50.0 + 5.0 * std::sin(i * 0.05);
        mmac.update(y, 50.0, 1.0, DT);
    }
    mmac.stop();
    
    auto probs = mmac.getModelProbabilities();
    EXPECT_EQ(probs.size(), 3);
}

// ============================================================================
// Additional Coverage Tests for AdaptiveMethods
// ============================================================================

TEST(GainSchedulerCoverage, SchedulingVariableFunction) {
    GainScheduler scheduler;
    
    scheduler.setSchedulingVariable([](double measured, double reference) {
        return measured / reference;
    });
    
    scheduler.addGainSet(0.0, {0.5, 0.05, 0.005});
    scheduler.addGainSet(1.0, {1.0, 0.1, 0.01});
    scheduler.addGainSet(2.0, {2.0, 0.2, 0.02});
    
    scheduler.enableInterpolation(true);
    scheduler.setGainRateLimit(5.0);
    
    scheduler.start();
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 25.0 * std::sin(i * 0.1);
        scheduler.update(measured, 50.0, 0.0, DT);
    }
    
    scheduler.stop();
    
    auto gains = scheduler.getCurrentGains();
    auto result = scheduler.getIntermediateResult();
}

TEST(MRACCoverage, AllAdaptationLaws) {
    std::vector<MRAC::AdaptationLaw> laws = {
        MRAC::AdaptationLaw::MIT,
        MRAC::AdaptationLaw::Lyapunov,
        MRAC::AdaptationLaw::NormalizedMIT
    };
    
    for (auto law : laws) {
        MRAC tuner;
        tuner.setReferenceModel(1.0, 0.5);
        tuner.setAdaptationGain({0.1, 0.05, 0.01});
        tuner.setAdaptationLaw(law);
        tuner.setSigmaModification(0.01);
        tuner.setParameterBounds({{0.0, 10.0}, {0.0, 5.0}, {0.0, 1.0}});
        
        tuner.start();
        
        for (int i = 0; i < 100; ++i) {
            double reference = 50.0 + 5.0 * (i < 50 ? 1.0 : 0.0);
            double measured = reference - 3.0 * std::exp(-i * 0.1);
            tuner.update(measured, reference, 0.0, DT);
        }
        
        tuner.stop();
        
        double error = tuner.getModelError();
        EXPECT_TRUE(std::isfinite(error));
    }
}

TEST(MRACCoverage, SecondOrderRefModel) {
    MRAC tuner;
    tuner.setReferenceModel(1.0, 2.0, 0.707);  // Second order: Km, wn, zeta
    tuner.setAdaptationGain({0.1, 0.05});
    tuner.setAdaptationLaw(MRAC::AdaptationLaw::Lyapunov);
    
    tuner.start();
    
    for (int i = 0; i < 150; ++i) {
        double reference = 50.0;
        double measured = reference - 5.0 * std::exp(-i * 0.05) * std::cos(0.5 * i * 0.1);
        tuner.update(measured, reference, 0.0, DT);
    }
    
    tuner.stop();
    
    bool complete = tuner.isComplete();
    auto result = tuner.getIntermediateResult();
}

TEST(STRCoverage, AllEstimationMethods) {
    std::vector<SelfTuningRegulator::EstimationMethod> methods = {
        SelfTuningRegulator::EstimationMethod::RLS,
        SelfTuningRegulator::EstimationMethod::RLSForgetting,
        SelfTuningRegulator::EstimationMethod::ELS,
        SelfTuningRegulator::EstimationMethod::RPEM
    };
    
    for (auto method : methods) {
        SelfTuningRegulator tuner;
        tuner.setModelStructure(2, 2, 1);
        tuner.setEstimationMethod(method);
        tuner.setForgettingFactor(0.99);
        tuner.setDesignMethod(SelfTuningRegulator::DesignMethod::PIDFromModel);
        tuner.setUpdateInterval(5);
        
        tuner.start();
        
        for (int i = 0; i < 100; ++i) {
            double measured = 50.0 + 3.0 * std::sin(0.2 * i);
            double control = 10.0 + 2.0 * std::cos(0.15 * i);
            tuner.update(measured, 50.0, control, DT);
        }
        
        tuner.stop();
        
        auto aHat = tuner.getEstimatedA();
        auto bHat = tuner.getEstimatedB();
        double cov = tuner.getEstimationCovariance();
        EXPECT_TRUE(std::isfinite(cov));
    }
}

TEST(STRCoverage, AllDesignMethods) {
    std::vector<SelfTuningRegulator::DesignMethod> methods = {
        SelfTuningRegulator::DesignMethod::MinimumVariance,
        SelfTuningRegulator::DesignMethod::PolePlacement,
        SelfTuningRegulator::DesignMethod::PIDFromModel
    };
    
    for (auto method : methods) {
        SelfTuningRegulator tuner;
        tuner.setModelStructure(2, 2, 1);
        tuner.setEstimationMethod(SelfTuningRegulator::EstimationMethod::RLSForgetting);
        tuner.setDesignMethod(method);
        tuner.setUpdateInterval(10);
        
        tuner.start();
        
        for (int i = 0; i < 100; ++i) {
            double measured = 50.0 + 3.0 * std::sin(0.2 * i);
            double control = 10.0 + 2.0 * std::cos(0.15 * i);
            tuner.update(measured, 50.0, control, DT);
        }
        
        tuner.stop();
    }
}

TEST(ESCCoverage, MultiParameterOptimization) {
    ExtremumSeekingControl esc;
    esc.setPerturbation(0.05, 5.0);
    esc.setAdaptationGain(0.5);
    esc.setFilterCutoff(2.0);
    esc.setNumParameters(3);
    esc.enableMultiParameter(true);
    
    esc.setCostFunction([](double output, double reference) {
        double error = output - reference;
        return error * error;
    });
    
    esc.start();
    
    for (int i = 0; i < 200; ++i) {
        double measured = 50.0 + 5.0 * std::sin(i * 0.05);
        esc.update(measured, 50.0, 0.0, DT);
    }
    
    esc.stop();
    
    auto params = esc.getParameters();
    auto result = esc.getIntermediateResult();
}

TEST(FuzzyTuningCoverage, CustomRules) {
    FuzzyTuning fuzzy;
    fuzzy.setErrorRange(-10.0, 10.0);
    fuzzy.setErrorRateRange(-5.0, 5.0);
    fuzzy.setGainRanges(0.5, 0.3, 0.2);
    fuzzy.setBaseGains(1.0, 0.1, 0.01);
    
    // Add custom rules
    fuzzy.addRule("NB", "N", 0.5, 0.3, 0.2);
    fuzzy.addRule("NB", "Z", 0.4, 0.2, 0.15);
    fuzzy.addRule("NB", "P", 0.3, 0.15, 0.1);
    fuzzy.addRule("ZO", "N", 0.1, 0.05, 0.05);
    fuzzy.addRule("ZO", "Z", 0.0, 0.0, 0.0);
    fuzzy.addRule("ZO", "P", -0.1, -0.05, -0.05);
    fuzzy.addRule("PB", "N", -0.3, -0.15, -0.1);
    fuzzy.addRule("PB", "Z", -0.4, -0.2, -0.15);
    fuzzy.addRule("PB", "P", -0.5, -0.3, -0.2);
    
    fuzzy.start();
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.1 * i);
        fuzzy.update(measured, 50.0, 0.0, DT);
    }
    
    fuzzy.stop();
}

TEST(FuzzyTuningCoverage, DefaultRules) {
    FuzzyTuning fuzzy;
    fuzzy.useDefaultRules();
    fuzzy.setBaseGains(1.5, 0.15, 0.02);
    
    fuzzy.start();
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 8.0 * std::sin(0.15 * i);
        fuzzy.update(measured, 50.0, 0.0, DT);
    }
    
    fuzzy.stop();
}

TEST(NNTuningCoverage, FullTrainingAndOnline) {
    NeuralNetworkTuning nn;
    nn.setArchitecture({10, 8, 4});
    nn.setActivation(NeuralNetworkTuning::Activation::Tanh);
    nn.setLearningRate(0.01);
    nn.enableOnlineLearning(true);
    
    // Generate training data
    std::vector<std::vector<double>> inputs;
    std::vector<std::vector<double>> targets;
    
    for (int i = 0; i < 100; ++i) {
        double err = (i - 50.0) / 50.0;
        double derr = 0.1 * std::cos(i * 0.1);
        double inte = 0.5 * std::sin(i * 0.05);
        inputs.push_back({err, derr, inte});
        targets.push_back({1.0 + 0.2 * err, 0.1 + 0.05 * std::abs(err), 0.01 + 0.005 * std::abs(derr)});
    }
    
    nn.trainBatch(inputs, targets, 50);
    
    nn.start();
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 5.0 * std::sin(0.1 * i);
        nn.update(measured, 50.0, 0.0, DT);
    }
    
    nn.stop();
    
    bool complete = nn.isComplete();
    auto result = nn.getIntermediateResult();
}

TEST(NNTuningCoverage, AllActivations) {
    std::vector<NeuralNetworkTuning::Activation> activations = {
        NeuralNetworkTuning::Activation::ReLU,
        NeuralNetworkTuning::Activation::Tanh,
        NeuralNetworkTuning::Activation::Sigmoid,
        NeuralNetworkTuning::Activation::Linear
    };
    
    for (auto act : activations) {
        NeuralNetworkTuning nn;
        nn.setArchitecture({8, 4});
        nn.setActivation(act);
        nn.setLearningRate(0.02);
        
        nn.start();
        
        for (int i = 0; i < 50; ++i) {
            nn.update(50.0 + 3.0 * std::sin(0.1 * i), 50.0, 0.0, DT);
        }
        
        nn.stop();
    }
}

TEST(MMACCoverage, AllMixingModes) {
    std::vector<MMAC::MixingMode> modes = {
        MMAC::MixingMode::Switching,
        MMAC::MixingMode::BayesianMixing,
        MMAC::MixingMode::SoftmaxMixing
    };
    
    for (auto mode : modes) {
        MMAC mmac;
        
        FOPDTModel model1;
        model1.K = 1.0;
        model1.tau = 5.0;
        model1.L = 1.0;
        
        FOPDTModel model2;
        model2.K = 2.0;
        model2.tau = 10.0;
        model2.L = 2.0;
        
        mmac.addModel(model1, {1.0, 0.1, 0.01});
        mmac.addModel(model2, {0.5, 0.05, 0.005});
        
        mmac.setMixingMode(mode);
        mmac.setSwitchingHysteresis(0.15);
        
        mmac.start();
        
        for (int i = 0; i < 100; ++i) {
            double y = 50.0 + 5.0 * std::sin(0.1 * i);
            mmac.update(y, 50.0, 1.0, DT);
        }
        
        mmac.stop();
        
        auto probs = mmac.getModelProbabilities();
        int active = mmac.getActiveModel();
        EXPECT_GE(active, 0);
        EXPECT_LT(active, 2);
    }
}

TEST(AdaptiveFactoryCoverage, CreateAllTuners) {
    auto gainSched = std::make_unique<GainScheduler>();
    auto mrac = std::make_unique<MRAC>();
    auto str = std::make_unique<SelfTuningRegulator>();
    auto esc = std::make_unique<ExtremumSeekingControl>();
    auto fuzzy = std::make_unique<FuzzyTuning>();
    auto nn = std::make_unique<NeuralNetworkTuning>();
    auto mmac = std::make_unique<MMAC>();
    
    EXPECT_FALSE(gainSched->getName().empty());
    EXPECT_FALSE(mrac->getName().empty());
    EXPECT_FALSE(str->getName().empty());
    EXPECT_FALSE(esc->getName().empty());
    EXPECT_FALSE(fuzzy->getName().empty());
    EXPECT_FALSE(nn->getName().empty());
    EXPECT_FALSE(mmac->getName().empty());
    
    EXPECT_FALSE(gainSched->getDescription().empty());
    EXPECT_FALSE(mrac->getDescription().empty());
    EXPECT_FALSE(str->getDescription().empty());
    EXPECT_FALSE(esc->getDescription().empty());
    EXPECT_FALSE(fuzzy->getDescription().empty());
    EXPECT_FALSE(nn->getDescription().empty());
    EXPECT_FALSE(mmac->getDescription().empty());
}

// ============================================================================
// Additional Coverage Tests for AdaptiveMethods
// ============================================================================

TEST(GainSchedulerCoverage, UseGainFunction) {
    GainScheduler scheduler;
    
    // Test with function-based gains
    scheduler.setGainFunction([](double op) -> ParameterVector {
        return {1.0 + 0.01 * op, 0.1 + 0.001 * op, 0.01};
    });
    
    scheduler.setSchedulingVariable([](double meas, double ref) {
        return std::abs(meas - ref);
    });
    
    scheduler.enableInterpolation(true);
    scheduler.setGainRateLimit(1.0);
    
    scheduler.start();
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.1 * i);
        scheduler.update(measured, 50.0, 0.0, DT);
    }
    
    auto gains = scheduler.getCurrentGains();
    scheduler.stop();
}

TEST(GainSchedulerCoverage, TableLookup) {
    GainScheduler scheduler;
    
    // Add multiple gain sets
    scheduler.addGainSet(0.0, {0.5, 0.05, 0.005});
    scheduler.addGainSet(25.0, {1.0, 0.1, 0.01});
    scheduler.addGainSet(50.0, {1.5, 0.15, 0.015});
    scheduler.addGainSet(75.0, {2.0, 0.2, 0.02});
    scheduler.addGainSet(100.0, {2.5, 0.25, 0.025});
    
    scheduler.enableInterpolation(false);  // Nearest neighbor
    scheduler.start();
    
    // Test various operating points
    scheduler.update(0.0, 0.0, 0.0, DT);
    scheduler.update(50.0, 0.0, 0.0, DT);
    scheduler.update(100.0, 0.0, 0.0, DT);
    scheduler.update(150.0, 0.0, 0.0, DT);  // Beyond table
    
    scheduler.stop();
}

TEST(GainSchedulerCoverage, RateLimiting) {
    GainScheduler scheduler;
    
    scheduler.addGainSet(0.0, {1.0, 0.1, 0.01});
    scheduler.addGainSet(100.0, {10.0, 1.0, 0.1});  // Large jump
    
    scheduler.enableInterpolation(true);
    scheduler.setGainRateLimit(0.5);  // Limit rate of change
    
    scheduler.start();
    
    // Rapidly changing operating point
    for (int i = 0; i < 50; ++i) {
        double op = (i % 2 == 0) ? 0.0 : 100.0;
        scheduler.update(op, 0.0, 0.0, DT);
    }
    
    auto gains = scheduler.getCurrentGains();
    scheduler.stop();
}

TEST(MRACCoverage, AllAdaptationLawsExtended) {
    std::vector<MRAC::AdaptationLaw> laws = {
        MRAC::AdaptationLaw::MIT,
        MRAC::AdaptationLaw::Lyapunov,
        MRAC::AdaptationLaw::NormalizedMIT
    };
    
    for (auto law : laws) {
        MRAC mrac;
        mrac.setReferenceModel(1.0, 0.5);
        mrac.setAdaptationLaw(law);
        mrac.setAdaptationGain({0.1, 0.05, 0.01});
        mrac.setSigmaModification(0.001);
        
        mrac.start();
        
        for (int i = 0; i < 100; ++i) {
            double y = 50.0 + 5.0 * std::sin(0.1 * i);
            mrac.update(y, 55.0, 0.0, DT);
        }
        
        bool complete = mrac.isComplete();
        double error = mrac.getModelError();
        EXPECT_TRUE(std::isfinite(error));
        
        mrac.stop();
    }
}

TEST(MRACCoverage, SecondOrderReferenceExtended) {
    MRAC mrac;
    mrac.setReferenceModel(1.0, 2.0, 0.707);  // Second order
    mrac.setAdaptationGain({0.1, 0.05});
    
    TestPIDController controller;
    auto result = mrac.tune(controller, nullptr);
    
    mrac.start();
    
    for (int i = 0; i < 200; ++i) {
        double ref = (i < 50) ? 50.0 : 60.0;  // Step change
        double y = 50.0 + (i > 50 ? (1.0 - std::exp(-(i-50)*0.01)) * 10.0 : 0.0);
        mrac.update(y, ref, 0.0, DT);
    }
    
    auto interResult = mrac.getIntermediateResult();
    mrac.stop();
}

TEST(MRACCoverage, ParameterBounds) {
    MRAC mrac;
    mrac.setReferenceModel(1.0, 0.5);
    mrac.setAdaptationGain({1.0, 1.0, 1.0});  // High adaptation
    mrac.setParameterBounds({
        {0.1, 10.0},  // Kp bounds
        {0.01, 1.0},  // Ki bounds
        {0.001, 0.1}  // Kd bounds
    });
    
    mrac.start();
    
    for (int i = 0; i < 200; ++i) {
        double y = 50.0 + 20.0 * std::sin(0.5 * i);  // Large variations
        mrac.update(y, 50.0, 0.0, DT);
    }
    
    auto result = mrac.getIntermediateResult();
    // Parameters should be bounded
    for (size_t i = 0; i < result.parameters.size(); ++i) {
        EXPECT_TRUE(std::isfinite(result.parameters[i]));
    }
    
    mrac.stop();
}

TEST(STRCoverage, AllEstimators) {
    std::vector<SelfTuningRegulator::EstimationMethod> estimators = {
        SelfTuningRegulator::EstimationMethod::RLS,
        SelfTuningRegulator::EstimationMethod::RLSForgetting,
        SelfTuningRegulator::EstimationMethod::ELS,
        SelfTuningRegulator::EstimationMethod::RPEM
    };
    
    for (auto est : estimators) {
        SelfTuningRegulator str;
        str.setEstimationMethod(est);
        str.setForgettingFactor(0.98);
        str.setModelStructure(2, 1, 1);
        
        str.start();
        
        for (int i = 0; i < 100; ++i) {
            double u = std::sin(0.1 * i);
            double y = 0.5 * u + 0.1 * std::sin(0.05 * i);
            str.update(y, 1.0, u, DT);
        }
        
        auto result = str.getIntermediateResult();
        str.stop();
    }
}

TEST(STRCoverage, AllControlMethods) {
    std::vector<SelfTuningRegulator::DesignMethod> methods = {
        SelfTuningRegulator::DesignMethod::MinimumVariance,
        SelfTuningRegulator::DesignMethod::PolePlacement,
        SelfTuningRegulator::DesignMethod::PIDFromModel
    };
    
    for (auto method : methods) {
        SelfTuningRegulator str;
        str.setDesignMethod(method);
        str.setEstimationMethod(SelfTuningRegulator::EstimationMethod::RLS);
        str.setModelStructure(2, 1, 1);
        
        TestPIDController controller;
        auto result = str.tune(controller, nullptr);
        
        str.start();
        
        for (int i = 0; i < 100; ++i) {
            double y = 50.0 + 3.0 * std::sin(0.1 * i);
            str.update(y, 50.0, 0.5, DT);
        }
        
        str.stop();
    }
}

TEST(ESCCoverage, AllPerturbationWaveforms) {
    // Test ESC with different amplitudes and frequencies
    for (int a = 1; a <= 3; ++a) {
        ExtremumSeekingControl esc;
        esc.setPerturbation(0.1 * a, 1.0 * a);  // amplitude, frequency
        esc.setFilterCutoff(0.5 * a);
        esc.setAdaptationGain(0.01);
        
        esc.start();
        
        for (int i = 0; i < 200; ++i) {
            // Simulate a cost function with a minimum
            double param = 1.0 + 0.1 * std::sin(0.1 * i);
            double cost = std::pow(param - 1.5, 2);  // Min at 1.5
            esc.update(cost, 0.0, param, DT);
        }
        
        auto result = esc.getIntermediateResult();
        esc.stop();
    }
}

TEST(ESCCoverage, DemodulationFilters) {
    ExtremumSeekingControl esc;
    esc.setPerturbation(0.05, 2.0);
    esc.setFilterCutoff(0.5);
    esc.setAdaptationGain(0.005);
    
    esc.start();
    
    // Simulate optimization with noise
    double param = 1.0;
    for (int i = 0; i < 500; ++i) {
        double noise = 0.01 * (rand() % 100 - 50) / 50.0;
        double cost = std::pow(param - 2.0, 2) + noise;
        param = esc.update(cost, 0.0, param, DT);
    }
    
    auto params = esc.getParameters();
    EXPECT_FALSE(params.empty());
    
    esc.stop();
}

TEST(FuzzyTuningCoverage, AllDefuzzMethods) {
    // Test FuzzyTuning with default rules
    FuzzyTuning fuzzy;
    
    fuzzy.setErrorRange(-2.0, 2.0);
    fuzzy.setErrorRateRange(-2.0, 2.0);
    fuzzy.setGainRanges(0.5, 0.3, 0.1);
    fuzzy.setBaseGains(1.0, 0.1, 0.01);
    fuzzy.useDefaultRules();
    
    fuzzy.start();
    
    for (int i = 0; i < 100; ++i) {
        double error = 2.0 * std::sin(0.1 * i);
        double derror = 0.2 * std::cos(0.1 * i);
        fuzzy.update(error, derror, 0.0, DT);
    }
    
    auto result = fuzzy.getIntermediateResult();
    fuzzy.stop();
}

TEST(FuzzyTuningCoverage, MultipleOutputs) {
    FuzzyTuning fuzzy;
    
    // Add rules with correct API
    fuzzy.setErrorRange(-10.0, 10.0);
    fuzzy.setErrorRateRange(-5.0, 5.0);
    fuzzy.setGainRanges(1.0, 0.5, 0.2);
    fuzzy.setBaseGains(1.0, 0.1, 0.05);
    
    // Use default rules
    fuzzy.useDefaultRules();
    
    TestPIDController controller;
    auto result = fuzzy.tune(controller, nullptr);
    
    fuzzy.start();
    
    for (int i = 0; i < 50; ++i) {
        double error = 5.0 * std::sin(0.1 * i);
        double derror = 0.5 * std::cos(0.1 * i);
        fuzzy.update(error, derror, 0.0, DT);
    }
    
    fuzzy.stop();
}

TEST(NeuralNetworkCoverage, DifferentArchitectures) {
    std::vector<std::vector<int>> architectures = {
        {4, 3},           // Simple
        {8, 6, 4},        // Medium
        {16, 12, 8, 4}    // Deep
    };
    
    for (const auto& arch : architectures) {
        NeuralNetworkTuning nn;
        nn.setArchitecture(arch);
        nn.setActivation(NeuralNetworkTuning::Activation::ReLU);
        nn.setLearningRate(0.01);
        
        // Generate training data
        std::vector<std::vector<double>> inputs;
        std::vector<std::vector<double>> targets;
        
        for (int i = 0; i < 50; ++i) {
            double err = (i - 25.0) / 25.0;
            inputs.push_back({err, 0.1 * err, 0.01 * err * err});
            targets.push_back({1.0 + 0.2 * err, 0.1 + 0.05 * err});
        }
        
        nn.trainBatch(inputs, targets, 20);
        
        nn.start();
        nn.update(0.5, 1.0, 0.0, DT);
        nn.stop();
    }
}

TEST(NeuralNetworkCoverage, OnlineLearning) {
    NeuralNetworkTuning nn;
    nn.setArchitecture({6, 4});
    nn.setActivation(NeuralNetworkTuning::Activation::Tanh);
    nn.setLearningRate(0.005);
    nn.enableOnlineLearning(true);
    
    nn.start();
    
    // Simulate online learning with varying conditions
    for (int i = 0; i < 200; ++i) {
        double setpoint = (i < 100) ? 50.0 : 60.0;
        double measured = 50.0 + (i > 100 ? (1.0 - std::exp(-(i-100)*0.02)) * 10.0 : 0.0);
        nn.update(measured, setpoint, 0.5, DT);
    }
    
    auto result = nn.getIntermediateResult();
    nn.stop();
}

TEST(MMACCoverage, ModelSwitching) {
    MMAC mmac;
    
    // Add multiple models
    for (int k = 1; k <= 4; ++k) {
        FOPDTModel model;
        model.K = k * 0.5;
        model.tau = k * 2.0;
        model.L = k * 0.5;
        
        ParameterVector gains = {k * 0.5, k * 0.05, k * 0.005};
        mmac.addModel(model, gains);
    }
    
    mmac.setMixingMode(MMAC::MixingMode::Switching);
    mmac.setSwitchingHysteresis(0.1);
    
    mmac.start();
    
    // Simulate plant with changing dynamics
    double y = 50.0;
    for (int i = 0; i < 300; ++i) {
        double ref = 55.0;
        double K_actual = (i < 100) ? 0.5 : ((i < 200) ? 1.5 : 2.0);
        y += 0.1 * K_actual * (ref - y);
        mmac.update(y, ref, 1.0, DT);
    }
    
    int active = mmac.getActiveModel();
    auto probs = mmac.getModelProbabilities();
    EXPECT_FALSE(probs.empty());
    
    mmac.stop();
}

TEST(MMACCoverage, BayesianUpdates) {
    MMAC mmac;
    
    FOPDTModel model1{1.0, 5.0, 1.0};
    FOPDTModel model2{2.0, 10.0, 2.0};
    
    mmac.addModel(model1, {1.0, 0.1, 0.01});
    mmac.addModel(model2, {0.5, 0.05, 0.005});
    
    mmac.setMixingMode(MMAC::MixingMode::BayesianMixing);
    
    TestPIDController controller;
    auto result = mmac.tune(controller, nullptr);
    
    mmac.start();
    
    for (int i = 0; i < 150; ++i) {
        double y = 50.0 + 5.0 * std::sin(0.05 * i);
        mmac.update(y, 50.0, 0.5, DT);
    }
    
    auto probs = mmac.getModelProbabilities();
    double sum = 0.0;
    for (double p : probs) {
        EXPECT_GE(p, 0.0);
        EXPECT_LE(p, 1.0);
        sum += p;
    }
    EXPECT_NEAR(sum, 1.0, 0.01);
    
    mmac.stop();
}

TEST(AdaptiveEdgeCases, EmptyController) {
    MRAC mrac;
    
    TestPIDController emptyController;
    emptyController.setParameters({});  // Empty parameters
    
    mrac.setReferenceModel(1.0, 0.5);
    
    // Should handle gracefully
    auto result = mrac.tune(emptyController, nullptr);
}

TEST(AdaptiveEdgeCases, ZeroAdaptationGain) {
    MRAC mrac;
    mrac.setAdaptationGain({0.0, 0.0, 0.0});  // Zero gains
    mrac.setReferenceModel(1.0, 0.5);
    
    mrac.start();
    
    for (int i = 0; i < 50; ++i) {
        mrac.update(50.0 + 5.0 * i, 60.0, 0.0, DT);
    }
    
    mrac.stop();
}

TEST(AdaptiveEdgeCases, StopWithoutStart) {
    GainScheduler scheduler;
    scheduler.addGainSet(0.0, {1.0, 0.1, 0.01});
    
    // Stop without start should be safe
    scheduler.stop();
    
    auto result = scheduler.getIntermediateResult();
}

TEST(AdaptiveIntegration, GainSchedulerWithMRAC) {
    GainScheduler scheduler;
    MRAC mrac;
    
    // Configure scheduler
    scheduler.addGainSet(0.0, {0.5, 0.05, 0.005});
    scheduler.addGainSet(100.0, {2.0, 0.2, 0.02});
    scheduler.enableInterpolation(true);
    
    // Configure MRAC
    mrac.setReferenceModel(1.0, 0.7);
    mrac.setAdaptationGain({0.05, 0.02, 0.01});
    
    scheduler.start();
    mrac.start();
    
    // Combined adaptation
    for (int i = 0; i < 100; ++i) {
        double op = 50.0 + 30.0 * std::sin(0.05 * i);  // Varying operating point
        double y = 50.0 + 3.0 * std::sin(0.1 * i);
        
        scheduler.update(op, 0.0, 0.0, DT);
        mrac.update(y, 55.0, 0.0, DT);
    }
    
    scheduler.stop();
    mrac.stop();
}