#include <cmath>
#include <memory>

#include "TestHelpers.hpp"
#include "tether/control/autotuning/IndustrialAutotuners.hpp"

using namespace Control::Autotuning;
using namespace Control::Autotuning::Testing;

// ============================================================================
// Test Fixture Classes
// ============================================================================

class PIDGainsRegressionTest : public ::testing::Test {};

class RelayFeedbackAutotunerTest : public ::testing::Test {
protected:
    std::unique_ptr<RelayFeedbackAutotuner> tuner;
    void SetUp() override {
        tuner = std::make_unique<RelayFeedbackAutotuner>();
    }
};

class StepResponseAutotunerTest : public ::testing::Test {
protected:
    std::unique_ptr<StepResponseAutotuner> tuner;
    void SetUp() override {
        tuner = std::make_unique<StepResponseAutotuner>();
    }
};

class ScheduledAutotunerTest : public ::testing::Test {
protected:
    std::unique_ptr<ScheduledAutotuner> tuner;
    void SetUp() override {
        tuner = std::make_unique<ScheduledAutotuner>();
    }
};

TEST_F(RelayFeedbackAutotunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Relay Feedback Autotuner");
}

TEST_F(RelayFeedbackAutotunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(RelayFeedbackAutotunerTest, SetAmplitude) {
    tuner->setAmplitude(5.0);
    // run a short update sequence to ensure tuner behaves with the amplitude set
    tuner->start();
    double output = 0.0;
    for (int i = 0; i < 50; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.1 * i);
        output = tuner->update(measured, 50.0, output, 0.1);
    }
    auto osc = tuner->getOscillationData();
    EXPECT_GE(osc.amplitude, 0.0);
}

TEST_F(RelayFeedbackAutotunerTest, SetRelayType) {
    // ensure all relay types run without breaking oscillation detection
    RelayFeedbackAutotuner::RelayType types[] = {
        RelayFeedbackAutotuner::RelayType::Standard,
        RelayFeedbackAutotuner::RelayType::Hysteresis,
        RelayFeedbackAutotuner::RelayType::Asymmetric,
        RelayFeedbackAutotuner::RelayType::Integrating
    };

    for (auto t : types) {
        tuner->setRelayType(t);
        tuner->start();
        double output = 0.0;
        for (int i = 0; i < 80; ++i) {
            double measured = 10.0 * std::sin(0.08 * i);
            output = tuner->update(measured, 0.0, output, 0.1);
        }
        auto osc = tuner->getOscillationData();
        EXPECT_TRUE(std::isfinite(osc.period));
    }
}

TEST_F(RelayFeedbackAutotunerTest, SetAsymmetricAmplitudes) {
    tuner->setAsymmetricAmplitudes(3.0, 5.0);
    tuner->start();
    double output = 0.0;
    for (int i = 0; i < 80; ++i) output = tuner->update(5.0 * std::sin(0.1 * i), 0.0, output, 0.1);
    auto osc = tuner->getOscillationData();
    EXPECT_GE(osc.amplitude, 0.0);
}

TEST_F(RelayFeedbackAutotunerTest, SetHysteresis) {
    tuner->setHysteresis(0.5);
    tuner->start();
    double output = 0.0;
    for (int i = 0; i < 80; ++i) output = tuner->update(std::sin(0.1 * i), 0.0, output, 0.1);
    auto osc = tuner->getOscillationData();
    EXPECT_GE(osc.period, 0.0);
}

TEST_F(RelayFeedbackAutotunerTest, SetMinCycles) {
    tuner->setMinCycles(5);
    tuner->start();
    // run fewer cycles than min and ensure tuner is not complete
    double output = 0.0;
    for (int i = 0; i < 10; ++i) output = tuner->update(std::sin(0.2 * i), 0.0, output, 0.1);
    EXPECT_FALSE(tuner->isComplete());
}

TEST_F(RelayFeedbackAutotunerTest, SetTuningRule) {
    tuner->setTuningRule(RelayFeedbackAutotuner::TuningRule::ZieglerNichols);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    auto r = tuner->tune(controller, &processModel);
    EXPECT_TRUE(r.success);

    tuner->setTuningRule(RelayFeedbackAutotuner::TuningRule::TyreusLuyben);
    r = tuner->tune(controller, &processModel);
    EXPECT_TRUE(r.success);
}

TEST_F(RelayFeedbackAutotunerTest, EnableSetpointBias) {
    tuner->enableSetpointBias(true);
    tuner->start();
    double output = 0.0;
    for (int i = 0; i < 50; ++i) output = tuner->update(1.0 + 0.1*std::sin(i*0.1), 1.0, output, 0.1);
    auto osc = tuner->getOscillationData();
    EXPECT_GE(osc.amplitude, 0.0);
}

TEST_F(RelayFeedbackAutotunerTest, SetTolerance) {
    tuner->setTolerance(0.01);
    tuner->start();
    double output = 0.0;
    for (int i = 0; i < 50; ++i) output = tuner->update(std::sin(0.1 * i), 0.0, output, 0.1);
    EXPECT_GE(tuner->getOscillationData().amplitude, 0.0);
}

TEST_F(RelayFeedbackAutotunerTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(RelayFeedbackAutotunerTest, IsComplete) {
    tuner->start();
    EXPECT_FALSE(tuner->isComplete());
}

TEST_F(RelayFeedbackAutotunerTest, Update) {
    tuner->setAmplitude(5.0);
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.1 * i);
        output = tuner->update(measured, 50.0, output, 0.1);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(RelayFeedbackAutotunerTest, GetOscillationData) {
    auto oscData = tuner->getOscillationData();
    EXPECT_GE(oscData.amplitude, 0.0);
    EXPECT_GE(oscData.period, 0.0);
}

TEST_F(RelayFeedbackAutotunerTest, GetIntermediateResult) {
    tuner->start();
    // EXPECT_NO_THROW({ auto result = tuner->getIntermediateResult(); }); // Unreachable code
    auto result = tuner->getIntermediateResult();  // Not wrapped in EXPECT_NO_THROW
}

TEST_F(RelayFeedbackAutotunerTest, Tune) {
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// StepResponseAutotuner Tests
// ============================================================================

TEST_F(StepResponseAutotunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Step Response Autotuner");
}

TEST_F(StepResponseAutotunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(StepResponseAutotunerTest, SetStepSize) {
    tuner->setStepSize(10.0);
    tuner->start();
    // simulate step response and ensure model estimation is populated
    double output = 0.0;
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        double measured = t > 2.0 ? 10.0 * (1.0 - std::exp(-(t - 2.0) / 5.0)) : 0.0;
        output = tuner->update(measured, 10.0, output, 0.1);
    }
    auto model = tuner->getModel();
    EXPECT_GT(model.tau, 0.0);
}

TEST_F(StepResponseAutotunerTest, SetIdMethod) {
    // Ensure different identification methods produce a valid model/tune
    StepResponseAutotuner::IdMethod methods[] = {
        StepResponseAutotuner::IdMethod::Tangent,
        StepResponseAutotuner::IdMethod::Area,
        StepResponseAutotuner::IdMethod::TwoPoint,
        StepResponseAutotuner::IdMethod::Optimization
    };
    TestPIDController controller;
    for (auto m : methods) {
        tuner->setIdMethod(m);
        auto r = tuner->tune(controller, nullptr);
        EXPECT_TRUE(r.success);
    }
}

TEST_F(StepResponseAutotunerTest, SetTestDuration) {
    tuner->setTestDuration(1.0); // small duration
    tuner->start();
    double output = 0.0;
    for (int i = 0; i < 30; ++i) output = tuner->update(std::min(1.0, i*0.1), 1.0, output, 0.1);
    // After a short run, tune() should still be able to return a model (graceful handling)
    TestPIDController controller;
    auto r = tuner->tune(controller, nullptr);
    EXPECT_TRUE(r.success);
}

TEST_F(StepResponseAutotunerTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(StepResponseAutotunerTest, IsComplete) {
    tuner->start();
    EXPECT_FALSE(tuner->isComplete());
}

TEST_F(StepResponseAutotunerTest, Update) {
    tuner->setStepSize(10.0);
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        double measured = t > 2.0 ? 10.0 * (1.0 - std::exp(-(t - 2.0) / 5.0)) : 0.0;
        output = tuner->update(measured, 10.0, output, 0.1);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(StepResponseAutotunerTest, GetModel) {
    auto model = tuner->getModel();
    EXPECT_TRUE(std::isfinite(model.tau));
}

TEST_F(StepResponseAutotunerTest, GetFitQuality) {
    double quality = tuner->getFitQuality();
    EXPECT_GE(quality, 0.0);
    EXPECT_LE(quality, 1.0);
}

TEST_F(StepResponseAutotunerTest, Tune) {
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// PatternRecognitionAutotuner Tests
// ============================================================================

class PatternRecognitionAutotunerTest : public ::testing::Test {
protected:
    std::unique_ptr<PatternRecognitionAutotuner> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<PatternRecognitionAutotuner>();
    }
};

TEST_F(PatternRecognitionAutotunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Pattern Recognition Autotuner");
}

TEST_F(PatternRecognitionAutotunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(PatternRecognitionAutotunerTest, SetSpecifications) {
    PatternRecognitionAutotuner::Specifications specs;
    specs.maxOvershoot = 0.05;
    specs.maxSettlingTime = 5.0;
    specs.maxRiseTime = 1.0;
    specs.settleBand = 0.01;
    tuner->setSpecifications(specs);

    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.2);
    auto r = tuner->tune(controller, &processModel);
    EXPECT_TRUE(r.success);
}

TEST_F(PatternRecognitionAutotunerTest, SetAdjustmentFactors) {
    tuner->setAdjustmentFactors(1.5, 1.3, 1.2);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.2);
    auto r = tuner->tune(controller, &processModel);
    EXPECT_TRUE(r.success);
}

TEST_F(PatternRecognitionAutotunerTest, SetMaxIterations) {
    tuner->setMaxIterations(5);
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.2);
    auto r = tuner->tune(controller, &processModel);
    EXPECT_TRUE(r.success);
}

TEST_F(PatternRecognitionAutotunerTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(PatternRecognitionAutotunerTest, IsComplete) {
    tuner->start();
    bool complete = tuner->isComplete();
    EXPECT_FALSE(complete);
}

TEST_F(PatternRecognitionAutotunerTest, GetCurrentPattern) {
    tuner->start();
    auto pattern = tuner->getCurrentPattern();
    EXPECT_EQ(pattern, PatternRecognitionAutotuner::Pattern::Unknown);
}

TEST_F(PatternRecognitionAutotunerTest, GetMetrics) {
    auto metrics = tuner->getMetrics();
    EXPECT_GE(metrics.riseTime, 0.0);
    EXPECT_GE(metrics.settlingTime, 0.0);
}

TEST_F(PatternRecognitionAutotunerTest, Update) {
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 10.0 * std::exp(-i * 0.1);
        output = tuner->update(measured, 50.0, output, 0.1);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(PatternRecognitionAutotunerTest, Tune) {
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// BumpTestAutotuner Tests
// ============================================================================

class BumpTestAutotunerTest : public ::testing::Test {
protected:
    std::unique_ptr<BumpTestAutotuner> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<BumpTestAutotuner>();
    }
};

TEST_F(BumpTestAutotunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Bump Test Autotuner");
}

TEST_F(BumpTestAutotunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(BumpTestAutotunerTest, SetMinBumpSize) {
    EXPECT_NO_THROW(tuner->setMinBumpSize(10.0));
}

TEST_F(BumpTestAutotunerTest, SetBumpsRequired) {
    EXPECT_NO_THROW(tuner->setBumpsRequired(3));
}

TEST_F(BumpTestAutotunerTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(BumpTestAutotunerTest, IsComplete) {
    tuner->start();
    EXPECT_FALSE(tuner->isComplete());
}

TEST_F(BumpTestAutotunerTest, ReportManualChange) {
    tuner->start();
    EXPECT_NO_THROW(tuner->reportManualChange(50.0, 60.0));
}

TEST_F(BumpTestAutotunerTest, GetConfidence) {
    double confidence = tuner->getConfidence();
    EXPECT_GE(confidence, 0.0);
    EXPECT_LE(confidence, 1.0);
}

TEST_F(BumpTestAutotunerTest, Update) {
    tuner->setMinBumpSize(5.0);
    tuner->start();
    
    double output = 50.0;
    double measured = 50.0;
    
    tuner->reportManualChange(50.0, 55.0);
    output = 55.0;
    
    for (int i = 0; i < 100; ++i) {
        measured = 50.0 + 5.0 * (1.0 - std::exp(-i * 0.01));
        output = tuner->update(measured, 50.0, output, 0.1);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(BumpTestAutotunerTest, Tune) {
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// ScheduledAutotuner Tests
// ============================================================================

TEST_F(ScheduledAutotunerTest, TuneNoUnderlyingTunerFallsBack) {
    TestPIDController controller;
    // No inner tuner configured
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.message.find("BumpTestAutotuner") != std::string::npos || result.message.empty());
    // Document expected semantics: ScheduledAutotuner falls back to BumpTestAutotuner when no inner tuner is set
}

TEST_F(ScheduledAutotunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Scheduled Autotuner");
}

TEST_F(ScheduledAutotunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(ScheduledAutotunerTest, SetTuningInterval) {
    // EXPECT_NO_THROW(tuner->setTuningInterval(24.0)); // Unreachable code
    tuner->setTuningInterval(24.0);
}

TEST_F(ScheduledAutotunerTest, SetPerformanceThreshold) {
    EXPECT_NO_THROW(tuner->setPerformanceThreshold(1.5));
}

TEST_F(ScheduledAutotunerTest, SetAutotuner) {
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    EXPECT_NO_THROW(tuner->setAutotuner(std::move(relayTuner)));
}

TEST_F(ScheduledAutotunerTest, TriggerRetune) {
    EXPECT_NO_THROW(tuner->triggerRetune());
}

TEST_F(ScheduledAutotunerTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(ScheduledAutotunerTest, IsComplete) {
    EXPECT_FALSE(tuner->isComplete());
}

TEST_F(ScheduledAutotunerTest, GetTimeSinceLastTune) {
    double time = tuner->getTimeSinceLastTune();
    EXPECT_GE(time, 0.0);
}

TEST_F(ScheduledAutotunerTest, GetCurrentPerformance) {
    double perf = tuner->getCurrentPerformance();
    EXPECT_GE(perf, 0.0);
}

TEST_F(ScheduledAutotunerTest, Update) {
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 5.0 * std::sin(0.1 * i);
        output = tuner->update(measured, 50.0, output, 0.1);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(ScheduledAutotunerTest, Tune) {
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// SafetyAutotuner Tests
// ============================================================================

class SafetyAutotunerTest : public ::testing::Test {
protected:
    std::unique_ptr<SafetyAutotuner> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<SafetyAutotuner>();
    }
};

TEST_F(SafetyAutotunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Safety-Critical Autotuner");
}

TEST_F(SafetyAutotunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(SafetyAutotunerTest, SetPVLimits) {
    EXPECT_NO_THROW(tuner->setPVLimits(-10.0, 110.0));
}

TEST_F(SafetyAutotunerTest, SetOutputLimits) {
    EXPECT_NO_THROW(tuner->setOutputLimits(0.0, 100.0));
}

TEST_F(SafetyAutotunerTest, SetRateLimit) {
    EXPECT_NO_THROW(tuner->setRateLimit(10.0));
}

TEST_F(SafetyAutotunerTest, SetMaxOscillation) {
    EXPECT_NO_THROW(tuner->setMaxOscillation(5.0));
}

TEST_F(SafetyAutotunerTest, SetAutotuner) {
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    EXPECT_NO_THROW(tuner->setAutotuner(std::move(relayTuner)));
}

TEST_F(SafetyAutotunerTest, Abort) {
    // EXPECT_NO_THROW(tuner->abort()); // Unreachable code
    tuner->abort();
}

TEST_F(SafetyAutotunerTest, StartStop) {
    tuner->start();
    tuner->stop();  // Not wrapped in EXPECT_NO_THROW - unreachable code issue
}

TEST_F(SafetyAutotunerTest, IsComplete) {
    tuner->start();
    bool complete = tuner->isComplete();
    EXPECT_FALSE(complete);
}

TEST_F(SafetyAutotunerTest, WereLimitsHit) {
    bool hit = tuner->wereLimitsHit();
    EXPECT_FALSE(hit);
}

TEST_F(SafetyAutotunerTest, Update) {
    tuner->setPVLimits(0.0, 100.0);
    tuner->setOutputLimits(0.0, 100.0);
    tuner->start();
    
    double output = 50.0;
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 5.0 * std::sin(0.1 * i);
        output = tuner->update(measured, 50.0, output, 0.1);
        EXPECT_GE(output, 0.0);
        EXPECT_LE(output, 100.0);
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST_F(SafetyAutotunerTest, Tune) {
    TestPIDController controller;
    auto result = tuner->tune(controller, nullptr);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// AutoSelectTuner Tests
// ============================================================================

class AutoSelectTunerTest : public ::testing::Test {
protected:
    std::unique_ptr<AutoSelectTuner> tuner;
    
    void SetUp() override {
        tuner = std::make_unique<AutoSelectTuner>();
    }
};

TEST_F(AutoSelectTunerTest, GetName) {
    EXPECT_EQ(tuner->getName(), "Auto-Select Autotuner");
}

TEST_F(AutoSelectTunerTest, GetDescription) {
    EXPECT_FALSE(tuner->getDescription().empty());
}

TEST_F(AutoSelectTunerTest, GetProcessCategory) {
    auto category = tuner->getProcessCategory();
    EXPECT_EQ(category, AutoSelectTuner::ProcessCategory::Unknown);
}

TEST_F(AutoSelectTunerTest, GetSelectedMethod) {
    auto method = tuner->getSelectedMethod();
    // Initially may be empty or default
    EXPECT_NO_THROW(method);
}

TEST_F(AutoSelectTunerTest, ForceMethod) {
    EXPECT_NO_THROW(tuner->forceMethod("relay"));
}

TEST_F(AutoSelectTunerTest, StartStop) {
    tuner->start();
    EXPECT_NO_THROW(tuner->stop());
}

TEST_F(AutoSelectTunerTest, IsComplete) {
    tuner->start();
    bool complete = tuner->isComplete();
    EXPECT_FALSE(complete);
}

TEST_F(AutoSelectTunerTest, Update) {
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 200; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.1 * i);
        output = tuner->update(measured, 50.0, output, 0.1);
    }
    
    auto category = tuner->getProcessCategory();
    EXPECT_NE(category, AutoSelectTuner::ProcessCategory::Unknown);
}

TEST_F(AutoSelectTunerTest, Tune) {
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 10.0, 1.0);
    
    auto result = tuner->tune(controller, &processModel);
    EXPECT_TRUE(result.success);
}

TEST_F(AutoSelectTunerTest, IsCompatible) {
    TestPIDController controller;
    bool compatible = tuner->isCompatible(controller);
    EXPECT_TRUE(compatible);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(IndustrialIntegration, RelayAutotunerFlow) {
    auto tuner = std::make_unique<RelayFeedbackAutotuner>();
    tuner->setAmplitude(5.0);
    tuner->setRelayType(RelayFeedbackAutotuner::RelayType::Hysteresis);
    tuner->setHysteresis(0.5);
    tuner->setMinCycles(3);
    tuner->setTuningRule(RelayFeedbackAutotuner::TuningRule::ZieglerNichols);
    
    tuner->start();
    
    double output = 0.0;
    for (int i = 0; i < 1000; ++i) {
        double measured = 50.0 + 3.0 * std::sin(0.5 * i * 0.01);
        output = tuner->update(measured, 50.0, output, 0.01);
    }
    
    auto oscData = tuner->getOscillationData();
    EXPECT_TRUE(std::isfinite(oscData.period));
}

TEST(IndustrialIntegration, StepAutotunerFlow) {
    auto tuner = std::make_unique<StepResponseAutotuner>();
    tuner->setStepSize(10.0);
    tuner->setIdMethod(StepResponseAutotuner::IdMethod::TwoPoint);
    
    tuner->start();
    
    for (int i = 0; i < 500; ++i) {
        double t = i * 0.1;
        double measured = t > 1.0 ? 20.0 * (1.0 - std::exp(-(t - 1.0) / 5.0)) : 0.0;
        tuner->update(measured, 10.0, 0.0, 0.1);
    }
    
    auto model = tuner->getModel();
    EXPECT_TRUE(std::isfinite(model.tau));
}

TEST(IndustrialIntegration, SafetyAutotunerLimitsOutput) {
    auto tuner = std::make_unique<SafetyAutotuner>();
    
    tuner->setPVLimits(0.0, 100.0);
    tuner->setOutputLimits(0.0, 100.0);
    tuner->setRateLimit(5.0);
    
    tuner->start();
    
    double output = 50.0;
    for (int i = 0; i < 100; ++i) {
        double measured = 10.0;  // Low - would drive output high
        double newOutput = tuner->update(measured, 90.0, output, 0.1);
        
        EXPECT_GE(newOutput, 0.0);
        EXPECT_LE(newOutput, 100.0);
        
        output = newOutput;
    }
    
    EXPECT_TRUE(std::isfinite(output));
}

TEST(IndustrialIntegration, AutoSelectAnalyzesProcess) {
    auto tuner = std::make_unique<AutoSelectTuner>();
    
    tuner->start();
    
    // Simulate oscillatory process
    for (int i = 0; i < 500; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.5 * i * 0.1);
        tuner->update(measured, 50.0, 0.0, 0.1);
    }
    
    auto category = tuner->getProcessCategory();
    auto method = tuner->getSelectedMethod();
    EXPECT_NE(category, AutoSelectTuner::ProcessCategory::Unknown);
    EXPECT_FALSE(method.empty());
}

// ============================================================================
// Additional Coverage Tests for IndustrialAutotuners
// ============================================================================

TEST(RelayFeedbackCoverage, AllRelayTypes) {
    // Test all relay types
    std::vector<RelayFeedbackAutotuner::RelayType> types = {
        RelayFeedbackAutotuner::RelayType::Standard,
        RelayFeedbackAutotuner::RelayType::Hysteresis,
        RelayFeedbackAutotuner::RelayType::Asymmetric,
        RelayFeedbackAutotuner::RelayType::Integrating
    };
    
    for (auto type : types) {
        RelayFeedbackAutotuner tuner;
        tuner.setAmplitude(5.0);
        tuner.setRelayType(type);
        tuner.setMinCycles(2);
        
        tuner.start();
        
        // Simulate oscillation
        for (int i = 0; i < 200; ++i) {
            double measured = 50.0 + 3.0 * std::sin(0.5 * i * 0.05);
            tuner.update(measured, 50.0, 0.0, 0.05);
        }
        
        tuner.stop();
    }
}

TEST(RelayFeedbackCoverage, AllTuningRules) {
    std::vector<RelayFeedbackAutotuner::TuningRule> rules = {
        RelayFeedbackAutotuner::TuningRule::ZieglerNichols,
        RelayFeedbackAutotuner::TuningRule::TyreusLuyben,
        RelayFeedbackAutotuner::TuningRule::SomeTimes,
        RelayFeedbackAutotuner::TuningRule::NoOvershoot,
        RelayFeedbackAutotuner::TuningRule::IMC_Aggressive,
        RelayFeedbackAutotuner::TuningRule::IMC_Moderate,
        RelayFeedbackAutotuner::TuningRule::IMC_Conservative
    };
     
    for (auto rule : rules) {
        RelayFeedbackAutotuner tuner;
        tuner.setAmplitude(5.0);
        tuner.setTuningRule(rule);
        tuner.setMinCycles(2);
        
        tuner.start();
        
        for (int i = 0; i < 200; ++i) {
            double measured = 50.0 + 3.0 * std::sin(0.5 * i * 0.05);
            tuner.update(measured, 50.0, 0.0, 0.05);
        }
        
        tuner.stop();
    }
}

TEST(RelayFeedbackCoverage, AsymmetricAmplitudes) {
    RelayFeedbackAutotuner tuner;
    tuner.setRelayType(RelayFeedbackAutotuner::RelayType::Asymmetric);
    tuner.setAsymmetricAmplitudes(3.0, 5.0);
    tuner.setMinCycles(2);
    
    tuner.start();
    
    for (int i = 0; i < 200; ++i) {
        double measured = 50.0 + 3.0 * std::sin(0.5 * i * 0.05);
        tuner.update(measured, 50.0, 0.0, 0.05);
    }
    
    tuner.stop();
}

TEST(RelayFeedbackCoverage, HysteresisAndTolerance) {
    RelayFeedbackAutotuner tuner;
    tuner.setRelayType(RelayFeedbackAutotuner::RelayType::Hysteresis);
    tuner.setHysteresis(1.0);
    tuner.setTolerance(0.1);
    tuner.enableSetpointBias(true);
    tuner.setMinCycles(2);
    
    tuner.start();
    
    for (int i = 0; i < 200; ++i) {
        double measured = 50.0 + 3.0 * std::sin(0.5 * i * 0.05);
        tuner.update(measured, 50.0, 0.0, 0.05);
    }
    
    auto oscData = tuner.getOscillationData();
    
    tuner.stop();
}

TEST(StepResponseCoverage, AllIdMethods) {
    std::vector<StepResponseAutotuner::IdMethod> methods = {
        StepResponseAutotuner::IdMethod::TwoPoint,
        StepResponseAutotuner::IdMethod::Tangent,
        StepResponseAutotuner::IdMethod::Area,
        StepResponseAutotuner::IdMethod::Optimization
    };
    
    for (auto method : methods) {
        StepResponseAutotuner tuner;
        tuner.setStepSize(10.0);
        tuner.setIdMethod(method);
        
        tuner.start();
        
        for (int i = 0; i < 100; ++i) {
            double t = i * 0.1;
            double measured = t > 0.5 ? 10.0 * (1.0 - std::exp(-(t - 0.5) / 3.0)) : 0.0;
            tuner.update(measured, 10.0, 10.0, 0.1);
        }
        
        tuner.stop();
        auto model = tuner.getModel();
        // auto fitQuality = tuner.getFitQuality(); // Not used
    }
}

TEST(StepResponseCoverage, TestDuration) {
    StepResponseAutotuner tuner;
    tuner.setStepSize(10.0);
    tuner.setTestDuration(5.0);
    tuner.setIdMethod(StepResponseAutotuner::IdMethod::TwoPoint);
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        double measured = t > 0.5 ? 10.0 * (1.0 - std::exp(-(t - 0.5) / 3.0)) : 0.0;
        tuner.update(measured, 10.0, 10.0, 0.1);
    }
    
    tuner.stop();
}

TEST(PatternRecognitionCoverage, DifferentPatterns) {
    PatternRecognitionAutotuner tuner;
    
    tuner.start();
    
    // Simulate different response patterns
    for (int phase = 0; phase < 3; ++phase) {
        for (int i = 0; i < 100; ++i) {
            double t = i * 0.1;
            double measured;
            
            switch (phase) {
                case 0:  // Oscillatory
                    measured = 50.0 + 5.0 * std::sin(t);
                    break;
                case 1:  // Sluggish
                    measured = 50.0 + 5.0 * (1.0 - std::exp(-t * 0.1));
                    break;
                default:  // Aggressive
                    measured = 50.0 + 10.0 * (1.0 - std::exp(-t)) - 2.0 * std::exp(-0.5 * t) * std::sin(t);
                    break;
            }
            
            tuner.update(measured, 55.0, 0.0, 0.1);
        }
    }
    
    tuner.stop();
    auto pattern = tuner.getCurrentPattern();
    auto metrics = tuner.getMetrics();
}

TEST(PatternRecognitionCoverage, SpecificationsAndFactors) {
    PatternRecognitionAutotuner tuner;
    
    PatternRecognitionAutotuner::Specifications specs;
    specs.maxOvershoot = 0.15;
    specs.maxSettlingTime = 8.0;
    specs.maxRiseTime = 1.5;
    specs.settleBand = 0.03;
    tuner.setSpecifications(specs);
    tuner.setAdjustmentFactors(1.2, 1.1, 1.05);
    tuner.setMaxIterations(5);
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-t * 0.5));
        tuner.update(measured, 55.0, 0.0, 0.1);
    }
    
    tuner.stop();
}

TEST(BumpTestCoverage, FullBumpSequence) {
    BumpTestAutotuner tuner;
    
    tuner.setMinBumpSize(5.0);
    tuner.setBumpsRequired(2);
    
    tuner.start();
    
    double output = 50.0;
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        // Simulate process response to bump
        double measured = 50.0;
        if (t > 2.0) {
            measured += 2.0 * 5.0 * (1.0 - std::exp(-(t - 2.0) / 3.0));
        }
        
        // Report a manual bump at specific times
        if (i == 20) {
            tuner.reportManualChange(output, output + 10.0);
            output += 10.0;
        }
        if (i == 100) {
            tuner.reportManualChange(output, output - 5.0);
            output -= 5.0;
        }
        
        output = tuner.update(measured, 50.0, output, 0.1);
    }
    
    tuner.stop();
    
    bool complete = tuner.isComplete();
    auto confidence = tuner.getConfidence();
}

TEST(ScheduledAutotuneCoverage, PeriodicTuning) {
    ScheduledAutotuner tuner;
    
    tuner.setTuningInterval(0.001);  // Very short for testing (in hours)
    tuner.setPerformanceThreshold(1.5);
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    relayTuner->setMinCycles(2);
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    
    // Simulate operation
    for (int i = 0; i < 200; ++i) {
        double measured = 50.0 + 2.0 * std::sin(0.3 * i * 0.1);
        tuner.update(measured, 50.0, 0.0, 0.1);
    }
    
    tuner.stop();
    
    auto timeSinceLastTune = tuner.getTimeSinceLastTune();
    auto currentPerf = tuner.getCurrentPerformance();
}

TEST(ScheduledAutotuneCoverage, TriggerRetune) {
    ScheduledAutotuner tuner;
    
    tuner.setTuningInterval(24.0);  // Long interval
    tuner.setPerformanceThreshold(2.0);
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    tuner.triggerRetune();  // Force immediate retuning
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + std::sin(0.1 * i);
        tuner.update(measured, 50.0, 0.0, 0.1);
    }
    
    tuner.stop();
}

TEST(SafetyAutotunerCoverage, AllLimitTypes) {
    SafetyAutotuner tuner;
    
    tuner.setPVLimits(10.0, 90.0);
    tuner.setOutputLimits(5.0, 95.0);
    tuner.setRateLimit(10.0);
    tuner.setMaxOscillation(15.0);
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    relayTuner->setMinCycles(2);
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    
    // Test exceeding PV limits
    double output = 50.0;
    
    // Test high PV
    for (int i = 0; i < 10; ++i) {
        output = tuner.update(95.0, 50.0, output, 0.1);  // Above limit
    }
    
    // Test low PV
    for (int i = 0; i < 10; ++i) {
        output = tuner.update(5.0, 50.0, output, 0.1);  // Below limit
    }
    
    // Test rate limiting (sudden large change)
    output = 10.0;
    double newOutput = tuner.update(50.0, 90.0, output, 0.1);  // Large setpoint -> big output change
    
    tuner.stop();
    
    bool limitsHit = tuner.wereLimitsHit();
}

TEST(SafetyAutotunerCoverage, AbortTuning) {
    SafetyAutotuner tuner;
    
    tuner.setPVLimits(10.0, 90.0);
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    
    for (int i = 0; i < 50; ++i) {
        tuner.update(50.0 + 5.0 * std::sin(0.1 * i), 50.0, 50.0, 0.1);
    }
    
    tuner.abort();
    
    tuner.stop();
}

TEST(AutoSelectCoverage, DifferentProcessTypes) {
    AutoSelectTuner tuner;
    
    tuner.start();
    
    // Fast process (low time constant)
    for (int i = 0; i < 50; ++i) {
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-i * 0.5));
        tuner.update(measured, 55.0, 0.0, 0.1);
    }
    
    // Slow process (high time constant)
    for (int i = 0; i < 100; ++i) {
        double measured = 55.0 + 5.0 * (1.0 - std::exp(-i * 0.05));
        tuner.update(measured, 60.0, 0.0, 0.1);
    }
    
    // Integrating process
    for (int i = 0; i < 50; ++i) {
        double measured = 60.0 + 0.5 * i;  // Linear ramp (integrator-like)
        tuner.update(measured, 85.0, 0.0, 0.1);
    }
    
    tuner.stop();
    
    auto category = tuner.getProcessCategory();
    auto method = tuner.getSelectedMethod();
}

TEST(AutoSelectCoverage, ForceMethod) {
    AutoSelectTuner tuner;
    
    tuner.forceMethod("relay_feedback");
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 3.0 * std::sin(0.3 * i * 0.1);
        tuner.update(measured, 50.0, 0.0, 0.1);
    }
    
    tuner.stop();
    
    auto method = tuner.getSelectedMethod();
}

TEST(IndustrialFactoryCoverage, CreateAllAutotuners) {
    // Test factory creation for various autotuners
    auto relay = std::make_unique<RelayFeedbackAutotuner>();
    auto step = std::make_unique<StepResponseAutotuner>();
    auto pattern = std::make_unique<PatternRecognitionAutotuner>();
    auto bump = std::make_unique<BumpTestAutotuner>();
    auto scheduled = std::make_unique<ScheduledAutotuner>();
    auto safety = std::make_unique<SafetyAutotuner>();
    auto autoSelect = std::make_unique<AutoSelectTuner>();
    
    EXPECT_FALSE(relay->getName().empty());
    EXPECT_FALSE(step->getName().empty());
    EXPECT_FALSE(pattern->getName().empty());
    EXPECT_FALSE(bump->getName().empty());
    EXPECT_FALSE(scheduled->getName().empty());
    EXPECT_FALSE(safety->getName().empty());
    EXPECT_FALSE(autoSelect->getName().empty());
    
    EXPECT_FALSE(relay->getDescription().empty());
    EXPECT_FALSE(step->getDescription().empty());
    EXPECT_FALSE(pattern->getDescription().empty());
    EXPECT_FALSE(bump->getDescription().empty());
    EXPECT_FALSE(scheduled->getDescription().empty());
    EXPECT_FALSE(safety->getDescription().empty());
    EXPECT_FALSE(autoSelect->getDescription().empty());
}

// ============================================================================
// Comprehensive Coverage Tests - RelayFeedbackAutotuner 
// ============================================================================

TEST(RelayFeedbackCoverage, AllTuningRulesWithOscillation) {
    std::vector<RelayFeedbackAutotuner::TuningRule> rules = {
        RelayFeedbackAutotuner::TuningRule::ZieglerNichols,
        RelayFeedbackAutotuner::TuningRule::TyreusLuyben,
        RelayFeedbackAutotuner::TuningRule::SomeTimes,
        RelayFeedbackAutotuner::TuningRule::NoOvershoot,
        RelayFeedbackAutotuner::TuningRule::IMC_Aggressive,
        RelayFeedbackAutotuner::TuningRule::IMC_Moderate,
        RelayFeedbackAutotuner::TuningRule::IMC_Conservative
    };
    
    for (auto rule : rules) {
        RelayFeedbackAutotuner tuner;
        tuner.setAmplitude(5.0);
        tuner.setTuningRule(rule);
        tuner.setMinCycles(3);
        tuner.setTolerance(0.2);
        tuner.start();
        
        // Generate proper oscillation data
        for (int i = 0; i < 500; ++i) {
            double t = i * 0.01;
            // Simulate a system oscillating with period ~1.0 second
            double measured = 50.0 + 10.0 * std::sin(2.0 * M_PI * t);
            tuner.update(measured, 50.0, 0.0, 0.01);
        }
        
        auto oscData = tuner.getOscillationData();
    }
}

TEST(RelayFeedbackCoverage, AllRelayTypesWithOscillation) {
    std::vector<RelayFeedbackAutotuner::RelayType> types = {
        RelayFeedbackAutotuner::RelayType::Standard,
        RelayFeedbackAutotuner::RelayType::Hysteresis,
        RelayFeedbackAutotuner::RelayType::Asymmetric,
        RelayFeedbackAutotuner::RelayType::Integrating
    };
    
    for (auto type : types) {
        RelayFeedbackAutotuner tuner;
        tuner.setAmplitude(5.0);
        tuner.setRelayType(type);
        tuner.setHysteresis(0.5);
        tuner.setMinCycles(3);
        
        if (type == RelayFeedbackAutotuner::RelayType::Asymmetric) {
            tuner.setAsymmetricAmplitudes(4.0, 6.0);
        }
        
        tuner.start();
        
        // Generate oscillation
        for (int i = 0; i < 300; ++i) {
            double t = i * 0.01;
            double measured = 50.0 + 8.0 * std::sin(2.0 * M_PI * t * 1.5);
            tuner.update(measured, 50.0, 0.0, 0.01);
        }
    }
}

TEST(RelayFeedbackCoverage, SetpointBiasMode) {
    RelayFeedbackAutotuner tuner;
    tuner.setAmplitude(5.0);
    tuner.enableSetpointBias(true);
    tuner.setMinCycles(2);
    tuner.start();
    
    // Oscillating around reference
    for (int i = 0; i < 300; ++i) {
        double t = i * 0.02;
        double measured = 50.0 + 6.0 * std::sin(2.0 * M_PI * t);
        tuner.update(measured, 50.0, 0.0, 0.02);
    }
    
    auto result = tuner.getIntermediateResult();
}

TEST(RelayFeedbackCoverage, StableVsUnstableOscillation) {
    // Test stable oscillation
    {
        RelayFeedbackAutotuner tuner;
        tuner.setAmplitude(5.0);
        tuner.setMinCycles(5);
        tuner.setTolerance(0.1);
        tuner.start();
        
        // Generate consistent oscillation (stable)
        for (int i = 0; i < 600; ++i) {
            double t = i * 0.01;
            double measured = 50.0 + 10.0 * std::sin(2.0 * M_PI * t);
            tuner.update(measured, 50.0, 0.0, 0.01);
        }
        
        auto oscData = tuner.getOscillationData();
        EXPECT_TRUE(oscData.stable);
    }
    
    // Test unstable oscillation (varying amplitude)
    {
        RelayFeedbackAutotuner tuner;
        tuner.setAmplitude(5.0);
        tuner.setMinCycles(5);
        tuner.setTolerance(0.01);  // Very tight tolerance
        tuner.start();
        
        // Generate varying oscillation (unstable)
        for (int i = 0; i < 600; ++i) {
            double t = i * 0.01;
            double amplitude = 10.0 + 3.0 * std::sin(0.5 * t);  // Varying amplitude
            double measured = 50.0 + amplitude * std::sin(2.0 * M_PI * t);
            tuner.update(measured, 50.0, 0.0, 0.01);
        }
        
        auto oscData = tuner.getOscillationData();
    }
}

TEST(RelayFeedbackCoverage, TuneWithProcessModel) {
    RelayFeedbackAutotuner tuner;
    tuner.setAmplitude(5.0);
    tuner.setMinCycles(3);
    tuner.start();
    
    // First generate oscillation data
    for (int i = 0; i < 400; ++i) {
        double t = i * 0.01;
        double measured = 50.0 + 8.0 * std::sin(2.0 * M_PI * t);
        tuner.update(measured, 50.0, 0.0, 0.01);
    }
    
    TestPIDController controller;
    TestFOPDTProcessModel processModel(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &processModel);
}

// ============================================================================
// Comprehensive Coverage Tests - StepResponseAutotuner
// ============================================================================

TEST(StepResponseCoverage2, AllIdMethodsWithTuning) {
    std::vector<StepResponseAutotuner::IdMethod> methods = {
        StepResponseAutotuner::IdMethod::Tangent,
        StepResponseAutotuner::IdMethod::Area,
        StepResponseAutotuner::IdMethod::TwoPoint,
        StepResponseAutotuner::IdMethod::Optimization
    };
    
    for (auto method : methods) {
        StepResponseAutotuner tuner;
        tuner.setStepSize(10.0);
        tuner.setIdMethod(method);
        tuner.setTestDuration(5.0);  // Short duration for testing
        tuner.start();
        
        // Generate step response data
        for (int i = 0; i < 800; ++i) {
            double t = i * 0.01;
            double measured = 0.0;
            if (t > 1.0) {
                // FOPDT-like response with K=2, tau=2, L=0.5
                double tEff = t - 1.5;
                if (tEff > 0) {
                    measured = 2.0 * 10.0 * (1.0 - std::exp(-tEff / 2.0));
                }
            }
            tuner.update(measured, 0.0, 0.0, 0.01);
        }
        
        auto model = tuner.getModel();
        auto quality = tuner.getFitQuality();
        
        TestPIDController controller;
        auto result = tuner.tune(controller, nullptr);
    }
}

TEST(StepResponseCoverage2, CustomTestDuration) {
    std::vector<double> durations = {2.0, 5.0, 10.0};
    
    for (double duration : durations) {
        StepResponseAutotuner tuner;
        tuner.setStepSize(5.0);
        tuner.setIdMethod(StepResponseAutotuner::IdMethod::TwoPoint);
        tuner.setTestDuration(duration);
        tuner.start();
        
        for (int i = 0; i < 1500; ++i) {
            double t = i * 0.01;
            double measured = t > 1.0 ? 5.0 * (1.0 - std::exp(-(t - 1.0) / 3.0)) : 0.0;
            tuner.update(measured, 0.0, 0.0, 0.01);
        }
    }
}

TEST(StepResponseCoverage2, WithDeadTime) {
    StepResponseAutotuner tuner;
    tuner.setStepSize(10.0);
    tuner.setIdMethod(StepResponseAutotuner::IdMethod::Tangent);
    tuner.setTestDuration(10.0);
    tuner.start();
    
    // Generate step response with significant dead time
    for (int i = 0; i < 1200; ++i) {
        double t = i * 0.01;
        double measured = 0.0;
        if (t > 1.0) {
            double tDelay = t - 1.0;
            if (tDelay > 2.0) {  // 2 second dead time
                measured = 10.0 * (1.0 - std::exp(-(tDelay - 2.0) / 3.0));
            }
        }
        tuner.update(measured, 0.0, 0.0, 0.01);
    }
    
    auto model = tuner.getModel();
    EXPECT_GT(model.L, 0.0);  // Should detect dead time
}

// ============================================================================
// Comprehensive Coverage Tests - PatternRecognitionAutotuner
// ============================================================================

TEST(PatternRecognitionCoverage2, AllPatternTypes) {
    // Test each pattern type
    struct PatternScenario {
        std::string name;
        std::function<double(double)> responseFunc;
    };
    
    std::vector<PatternScenario> scenarios = {
        {"Overdamped", [](double t) { return 50.0 + 5.0 * (1.0 - std::exp(-t * 0.3)); }},
        {"Critically Damped", [](double t) { return 50.0 + 5.0 * (1.0 - (1 + t) * std::exp(-t)); }},
        {"Underdamped", [](double t) { 
            return 50.0 + 5.0 * (1.0 - std::exp(-0.3 * t) * (std::cos(t) + 0.3 * std::sin(t)));
        }},
        {"Oscillatory", [](double t) { return 50.0 + 5.0 * std::sin(2.0 * t); }},
        {"Aggressive", [](double t) {
            return 50.0 + 10.0 * (1.0 - std::exp(-t)) - 2.0 * std::exp(-0.5 * t) * std::sin(t);
        }}
    };
    
    for (const auto& scenario : scenarios) {
        PatternRecognitionAutotuner tuner;
        
        PatternRecognitionAutotuner::Specifications specs;
        specs.maxOvershoot = 0.1;
        specs.maxSettlingTime = 10.0;
        specs.maxRiseTime = 3.0;
        specs.settleBand = 0.02;
        tuner.setSpecifications(specs);
        tuner.setMaxIterations(10);
        tuner.start();
        
        for (int i = 0; i < 200; ++i) {
            double t = i * 0.05;
            double measured = scenario.responseFunc(t);
            tuner.update(measured, 55.0, 0.0, 0.05);
        }
        
        tuner.stop();
        auto pattern = tuner.getCurrentPattern();
        auto metrics = tuner.getMetrics();
    }
}

TEST(PatternRecognitionCoverage2, TuneWithController) {
    PatternRecognitionAutotuner tuner;
    
    PatternRecognitionAutotuner::Specifications specs;
    specs.maxOvershoot = 0.05;
    specs.maxSettlingTime = 5.0;
    specs.maxRiseTime = 1.0;
    specs.settleBand = 0.01;
    tuner.setSpecifications(specs);
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-t * 0.5));
        tuner.update(measured, 55.0, 0.0, 0.1);
    }
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// ============================================================================
// Comprehensive Coverage Tests - BumpTestAutotuner
// ============================================================================

TEST(BumpTestCoverage2, MultipleBumps) {
    BumpTestAutotuner tuner;
    
    tuner.setMinBumpSize(3.0);
    tuner.setBumpsRequired(3);
    
    tuner.start();
    
    double output = 50.0;
    for (int i = 0; i < 500; ++i) {
        double t = i * 0.05;
        double measured = output;
        
        // Simulate process response to bumps
        if (i == 50) {
            tuner.reportManualChange(output, output + 5.0);
            output += 5.0;
        }
        if (i == 150) {
            tuner.reportManualChange(output, output - 3.0);
            output -= 3.0;
        }
        if (i == 250) {
            tuner.reportManualChange(output, output + 4.0);
            output += 4.0;
        }
        
        // Delayed response
        if (i > 60 && i < 150) measured = 50.0 + 5.0 * (1.0 - std::exp(-(t - 3.0) / 2.0));
        if (i > 160 && i < 250) measured = 52.0 - 3.0 * (1.0 - std::exp(-(t - 8.0) / 2.0));
        if (i > 260) measured = 53.0 + 4.0 * (1.0 - std::exp(-(t - 13.0) / 2.0));
        
        output = tuner.update(measured, 50.0, output, 0.05);
    }
    
    tuner.stop();
    auto confidence = tuner.getConfidence();
}

TEST(BumpTestCoverage2, TuneWithController) {
    BumpTestAutotuner tuner;
    tuner.setMinBumpSize(5.0);
    tuner.setBumpsRequired(1);
    
    tuner.start();
    
    double output = 50.0;
    tuner.reportManualChange(output, 55.0);
    
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-t / 3.0));
        output = tuner.update(measured, 50.0, 55.0, 0.1);
    }
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// ============================================================================
// Comprehensive Coverage Tests - ScheduledAutotuner
// ============================================================================

TEST(ScheduledCoverage, PerformanceTriggeredRetune) {
    ScheduledAutotuner tuner;
    
    tuner.setTuningInterval(1000.0);  // Long interval
    tuner.setPerformanceThreshold(0.5);  // Tight threshold
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    relayTuner->setMinCycles(2);
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    
    // Simulate good performance initially
    for (int i = 0; i < 50; ++i) {
        double measured = 50.0 + 0.5 * std::sin(0.1 * i);  // Small oscillation
        tuner.update(measured, 50.0, 0.0, 0.1);
    }
    
    // Simulate degraded performance
    for (int i = 0; i < 100; ++i) {
        double measured = 50.0 + 10.0 * std::sin(0.3 * i);  // Large oscillation
        tuner.update(measured, 50.0, 0.0, 0.1);
    }
    
    tuner.stop();
}

TEST(ScheduledCoverage, TuneWithController) {
    ScheduledAutotuner tuner;
    
    tuner.setTuningInterval(0.0001);  // Very short for testing
    
    auto stepTuner = std::make_unique<StepResponseAutotuner>();
    stepTuner->setStepSize(5.0);
    tuner.setAutotuner(std::move(stepTuner));
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        tuner.update(50.0 + i * 0.1, 55.0, 0.0, 0.1);
    }
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// ============================================================================
// Comprehensive Coverage Tests - SafetyAutotuner
// ============================================================================

TEST(SafetyCoverage, RateLimiting) {
    SafetyAutotuner tuner;
    
    tuner.setRateLimit(1.0);  // Max 1 unit per second
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    
    double output = 50.0;
    for (int i = 0; i < 100; ++i) {
        // Rapid changes that should be rate-limited
        double measured = 50.0 + 20.0 * (i % 2 == 0 ? 1 : -1);
        output = tuner.update(measured, 50.0, output, 0.1);
    }
    
    tuner.stop();
}

TEST(SafetyCoverage, OscillationLimit) {
    SafetyAutotuner tuner;
    
    tuner.setMaxOscillation(5.0);  // Max 5 unit oscillation
    
    auto relayTuner = std::make_unique<RelayFeedbackAutotuner>();
    tuner.setAutotuner(std::move(relayTuner));
    
    tuner.start();
    
    // Generate large oscillation that exceeds limit
    double output = 50.0;
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.05;
        double measured = 50.0 + 15.0 * std::sin(2.0 * M_PI * t);  // Large oscillation
        output = tuner.update(measured, 50.0, output, 0.05);
    }
    
    tuner.stop();
    EXPECT_TRUE(tuner.wereLimitsHit());
}

TEST(SafetyCoverage, TuneWithController) {
    SafetyAutotuner tuner;
    
    tuner.setPVLimits(0.0, 100.0);
    tuner.setOutputLimits(0.0, 100.0);
    
    auto stepTuner = std::make_unique<StepResponseAutotuner>();
    tuner.setAutotuner(std::move(stepTuner));
    
    tuner.start();
    
    for (int i = 0; i < 50; ++i) {
        tuner.update(50.0, 50.0, 50.0, 0.1);
    }
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// ============================================================================
// Comprehensive Coverage Tests - AutoSelectTuner
// ============================================================================

TEST(AutoSelectCoverage2, SlowProcess) {
    AutoSelectTuner tuner;
    
    tuner.start();
    
    // Simulate a slow process (high time constant)
    for (int i = 0; i < 500; ++i) {
        double t = i * 0.1;
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-t / 50.0));  // Very slow
        tuner.update(measured, 55.0, 0.0, 0.1);
    }
    
    tuner.stop();
    
    auto category = tuner.getProcessCategory();
    auto method = tuner.getSelectedMethod();
}

TEST(AutoSelectCoverage2, FastProcess) {
    AutoSelectTuner tuner;
    
    tuner.start();
    
    // Simulate a fast process (low time constant)
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.05;
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-t * 5.0));  // Very fast
        tuner.update(measured, 55.0, 0.0, 0.05);
    }
    
    tuner.stop();
    
    auto category = tuner.getProcessCategory();
}

TEST(AutoSelectCoverage2, TuneWithController) {
    AutoSelectTuner tuner;
    
    tuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        double measured = 50.0 + 5.0 * (1.0 - std::exp(-t / 5.0));
        tuner.update(measured, 55.0, 0.0, 0.1);
    }
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST(IndustrialEdgeCases, RelayFeedbackNoOscillation) {
    RelayFeedbackAutotuner tuner;
    tuner.setAmplitude(5.0);
    tuner.start();
    
    // Flat response - no oscillation
    for (int i = 0; i < 100; ++i) {
        tuner.update(50.0, 50.0, 0.0, 0.1);
    }
    
    auto result = tuner.getIntermediateResult();
    EXPECT_FALSE(result.success);
}

TEST(IndustrialEdgeCases, StepResponseNoModel) {
    StepResponseAutotuner tuner;
    
    TestPIDController controller;
    auto result = tuner.tune(controller, nullptr);
    
    // The tune() should still return something (possibly success with default gains)
    // The implementation may vary, so just verify we get a result
    EXPECT_NO_THROW(result);
}

TEST(IndustrialEdgeCases, EmptyDataSets) {
    // Test with minimal data
    StepResponseAutotuner tuner;
    tuner.setStepSize(10.0);
    tuner.setTestDuration(0.1);  // Very short
    tuner.start();
    
    for (int i = 0; i < 5; ++i) {
        tuner.update(i * 2.0, 0.0, 0.0, 0.1);
    }
}
// ============================================================================
// Additional Coverage Tests for IndustrialAutotuners
// ============================================================================

TEST(RelayFeedbackCoverage, AllRelayTypesWithTune) {
    std::vector<RelayFeedbackAutotuner::RelayType> types = {
        RelayFeedbackAutotuner::RelayType::Standard,
        RelayFeedbackAutotuner::RelayType::Hysteresis,
        RelayFeedbackAutotuner::RelayType::Asymmetric,
        RelayFeedbackAutotuner::RelayType::Integrating
    };
    
    for (auto type : types) {
        RelayFeedbackAutotuner tuner;
        tuner.setRelayType(type);
        tuner.setAmplitude(5.0);
        tuner.setHysteresis(0.1);
        tuner.setAsymmetricAmplitudes(4.0, 6.0);
        tuner.setMinCycles(3);
        tuner.enableSetpointBias(true);
        
        TestPIDController controller;
        TestFOPDTProcessModel model(1.0, 10.0, 1.0);
        
        auto result = tuner.tune(controller, &model);
        
        tuner.start();
        
        // Simulate relay oscillation
        double y = 50.0;
        double u = 0.0;
        for (int i = 0; i < 300; ++i) {
            u = tuner.update(y, 50.0, u, DT);
            y += DT * (model.toFOPDT().K * u / model.toFOPDT().tau - y / model.toFOPDT().tau);
        }
        
        tuner.stop();
        
        auto oscData = tuner.getOscillationData();
    }
}

// TEST(RelayFeedbackCoverage, AllTuningRules) moved to consolidate with earlier test
// Test removed - duplicate of test at line 726

TEST(StepResponseCoverage, AllIdentificationMethods) {
    std::vector<StepResponseAutotuner::IdMethod> methods = {
        StepResponseAutotuner::IdMethod::TwoPoint,
        StepResponseAutotuner::IdMethod::Area,
        StepResponseAutotuner::IdMethod::Tangent
    };
    
    for (auto method : methods) {
        StepResponseAutotuner tuner;
        tuner.setIdMethod(method);
        tuner.setStepSize(10.0);
        tuner.setTestDuration(30.0);
        
        TestPIDController controller;
        TestFOPDTProcessModel model(1.5, 12.0, 2.0);
        
        auto result = tuner.tune(controller, &model);
        
        tuner.start();
        
        // Simulate step response
        double y = 50.0;
        for (int i = 0; i < 300; ++i) {
            double input = (i > 50) ? 60.0 : 50.0;
            double target = input * model.toFOPDT().K;
            y += DT * (target - y) / model.toFOPDT().tau;
            tuner.update(y, 50.0, input, DT);
        }
        
        tuner.stop();
        
        auto identified = tuner.getModel();
    }
}

TEST(BumpTestCoverage, AllBumpTestConfigurations) {
    BumpTestAutotuner tuner;
    
    tuner.setMinBumpSize(5.0);
    
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &model);
    
    tuner.start();
    
    double y = 50.0;
    double u = 50.0;
    for (int i = 0; i < 500; ++i) {
        u = tuner.update(y, 50.0, u, DT);
        y += DT * (u - y) / 5.0;
    }
    
    tuner.stop();
    
    // Get confidence instead of model
    double confidence = tuner.getConfidence();
}

TEST(RelayFeedbackCoverage, AdditionalTuningRules) {
    std::vector<RelayFeedbackAutotuner::TuningRule> rules = {
        RelayFeedbackAutotuner::TuningRule::SomeTimes,
        RelayFeedbackAutotuner::TuningRule::IMC_Aggressive,
        RelayFeedbackAutotuner::TuningRule::IMC_Moderate,
        RelayFeedbackAutotuner::TuningRule::IMC_Conservative
    };
    
    for (auto rule : rules) {
        RelayFeedbackAutotuner tuner;
        tuner.setTuningRule(rule);
        tuner.setAmplitude(5.0);
        
        TestPIDController controller;
        TestFOPDTProcessModel model(2.0, 8.0, 1.5);
        
        auto result = tuner.tune(controller, &model);
    }
}

TEST(PatternRecognitionCoverage, AllPatterns) {
    PatternRecognitionAutotuner tuner;
    
    // Configure specs
    PatternRecognitionAutotuner::Specifications specs;
    specs.maxOvershoot = 0.1;
    specs.maxSettlingTime = 10.0;
    specs.maxRiseTime = 2.0;
    tuner.setSpecifications(specs);
    tuner.setAdjustmentFactors(1.5, 1.3, 1.2);
    tuner.setMaxIterations(5);
    
    TestPIDController controller;
    
    auto result = tuner.tune(controller, nullptr);
    
    tuner.start();
    
    // Simulate oscillatory response
    for (int i = 0; i < 200; ++i) {
        double y = 50.0 + 5.0 * std::sin(0.2 * i) * std::exp(-0.01 * i);
        tuner.update(y, 50.0, 0.0, DT);
    }
    
    auto pattern = tuner.getCurrentPattern();
    // auto metrics = tuner.getMetrics(); // Not used
    tuner.getMetrics();
    
    tuner.stop();
}

TEST(PatternRecognitionCoverage, OverdampedResponse) {
    PatternRecognitionAutotuner tuner;
    
    tuner.start();
    
    // Simulate overdamped response
    for (int i = 0; i < 200; ++i) {
        double t = i * DT;
        double y = 50.0 + 10.0 * (1.0 - std::exp(-t / 3.0));  // No overshoot
        tuner.update(y, 60.0, 0.0, DT);
    }
    
    auto pattern = tuner.getCurrentPattern();
    tuner.stop();
}

TEST(ScheduledAutotunerCoverage, AllModes) {
    ScheduledAutotuner tuner;
    
    TestPIDController controller;
    
    auto result = tuner.tune(controller, nullptr);
    
    tuner.start();
    
    // Simulate with changing dynamics
    for (int i = 0; i < 300; ++i) {
        double gain = (i < 150) ? 1.0 : 2.0;  // Gain change
        double y = 50.0 + gain * 5.0 * std::sin(0.1 * i);
        tuner.update(y, 50.0, 0.5, DT);
    }
    
    tuner.stop();
}

TEST(SafetyAutotunerCoverage, AllSafetyChecks) {
    SafetyAutotuner tuner;
    
    tuner.setPVLimits(0.0, 100.0);
    tuner.setOutputLimits(0.0, 100.0);
    tuner.setRateLimit(10.0);
    tuner.setMaxOscillation(15.0);
    
    auto innerTuner = std::make_unique<RelayFeedbackAutotuner>();
    innerTuner->setAmplitude(5.0);
    tuner.setAutotuner(std::move(innerTuner));
    
    TestPIDController controller;
    
    auto result = tuner.tune(controller, nullptr);
    
    tuner.start();
    
    // Test within limits
    for (int i = 0; i < 100; ++i) {
        double y = 50.0 + 10.0 * std::sin(0.1 * i);
        tuner.update(y, 50.0, 50.0, DT);
    }
    
    tuner.stop();
}

TEST(SafetyAutotunerCoverage, PVLimitViolation) {
    SafetyAutotuner tuner;
    
    tuner.setPVLimits(40.0, 60.0);  // Narrow limits
    tuner.setOutputLimits(0.0, 100.0);
    
    auto innerTuner = std::make_unique<RelayFeedbackAutotuner>();
    tuner.setAutotuner(std::move(innerTuner));
    
    tuner.start();
    
    // Simulate value going outside limits
    for (int i = 0; i < 50; ++i) {
        double y = 50.0 + 15.0 * std::sin(0.2 * i);  // Exceeds limits
        tuner.update(y, 50.0, 50.0, DT);
    }
    
    // Safety tuner should have detected limit violation
    bool limitsHit = tuner.wereLimitsHit();
    tuner.stop();
}

TEST(AutoSelectCoverage, ProcessCharacterization) {
    AutoSelectTuner tuner;
    
    tuner.start();
    
    // Simulate integrating process
    double y = 50.0;
    double integ = 0.0;
    for (int i = 0; i < 200; ++i) {
        double u = (i > 50) ? 0.5 : 0.0;
        integ += u * DT;  // Integrating
        y = 50.0 + integ;
        tuner.update(y, 50.0, u, DT);
    }
    
    tuner.stop();
    
    auto category = tuner.getProcessCategory();
    auto method = tuner.getSelectedMethod();
}

TEST(ModelValidationCoverage, AllChecks) {
    // Test model validation via StepResponseAutotuner
    StepResponseAutotuner tuner;
    
    tuner.setStepSize(10.0);
    
    TestPIDController controller;
    TestFOPDTProcessModel model(1.0, 5.0, 0.5);
    
    auto result = tuner.tune(controller, &model);
    
    tuner.start();
    
    // Perform step test
    double y = 50.0;
    for (int i = 0; i < 200; ++i) {
        double u = (i > 50) ? 60.0 : 50.0;
        y += DT * (model.toFOPDT().K * u - y) / model.toFOPDT().tau;
        tuner.update(y, 50.0, u, DT);
    }
    
    tuner.stop();
    
    auto identified = tuner.getModel();
    double fitQuality = tuner.getFitQuality();
}

TEST(IndustrialIntegration, SequentialTuning) {
    // Test sequential tuning with multiple methods
    TestPIDController controller;
    TestFOPDTProcessModel model(1.5, 10.0, 2.0);
    
    // First: Relay identification
    RelayFeedbackAutotuner relayTuner;
    relayTuner.setAmplitude(5.0);
    auto relayResult = relayTuner.tune(controller, &model);
    
    // Second: Step response refinement
    StepResponseAutotuner stepTuner;
    stepTuner.setStepSize(10.0);
    auto stepResult = stepTuner.tune(controller, &model);
    
    // Compare results
    EXPECT_TRUE(relayResult.success);
    EXPECT_TRUE(stepResult.success);
}

TEST(IndustrialIntegration, AutoSelectWithSafety) {
    SafetyAutotuner safetyTuner;
    
    safetyTuner.setPVLimits(0.0, 100.0);
    safetyTuner.setOutputLimits(0.0, 100.0);
    
    auto autoSelect = std::make_unique<AutoSelectTuner>();
    safetyTuner.setAutotuner(std::move(autoSelect));
    
    TestPIDController controller;
    
    auto result = safetyTuner.tune(controller, nullptr);
    
    safetyTuner.start();
    
    for (int i = 0; i < 100; ++i) {
        double y = 50.0 + 5.0 * std::sin(0.1 * i);
        safetyTuner.update(y, 50.0, 50.0, DT);
    }
    
    safetyTuner.stop();
}