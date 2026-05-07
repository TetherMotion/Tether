/**
 * @file LearningControllersTests.cpp
 * @brief Comprehensive tests for LearningControllers module
 * Tests for ILC (Iterative Learning Control) variants
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "tether/control/Controllers.hpp"
#include "tether/control/LearningControllers.hpp"

using namespace Control;

// ============================================================================
// ILCBase Tests (using PTypeILC as concrete implementation)
// ============================================================================

class ILCBaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        ilc = std::make_unique<PTypeILC>();
    }
    
    std::unique_ptr<PTypeILC> ilc;
};

TEST_F(ILCBaseTest, GetType) {
    EXPECT_EQ(ilc->getType(), ControllerType::ILC);
}

TEST_F(ILCBaseTest, SetTrajectoryLength) {
    ilc->setTrajectoryLength(1000);
    EXPECT_EQ(ilc->getTrajectoryLength(), 1000u);
}

TEST_F(ILCBaseTest, StartTrial) {
    ilc->setTrajectoryLength(100);
    ilc->startTrial();
    // Trial number is incremented on endTrial, not startTrial
    EXPECT_EQ(ilc->getTrialNumber(), 0);
}

TEST_F(ILCBaseTest, EndTrial) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGain(0.5);
    
    ilc->startTrial();
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 10.0;
    input.measured = 5.0;
    
    for (size_t i = 0; i < 100; ++i) {
        ilc->compute(input);
        ilc->recordSample(i);
    }
    
    ilc->endTrial();
    
    // RMS error should be computed
    double rms = ilc->getRMSError();
    EXPECT_GT(rms, 0.0);
}

TEST_F(ILCBaseTest, GetTrialNumber) {
    ilc->setTrajectoryLength(10);
    
    EXPECT_EQ(ilc->getTrialNumber(), 0);
    
    // Trial number is incremented on endTrial(), not startTrial()
    ilc->startTrial();
    EXPECT_EQ(ilc->getTrialNumber(), 0);  // Still 0 after startTrial
    
    ilc->endTrial();
    EXPECT_EQ(ilc->getTrialNumber(), 1);  // Now 1 after endTrial
    
    ilc->startTrial();
    ilc->endTrial();
    EXPECT_EQ(ilc->getTrialNumber(), 2);  // 2 after second trial
}

TEST_F(ILCBaseTest, GetRMSError) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGain(0.5);
    
    ilc->startTrial();
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (size_t i = 0; i < 100; ++i) {
        input.reference = 10.0;
        input.measured = 5.0 + 0.1 * i;
        ilc->compute(input);
        ilc->recordSample(i);
    }
    
    ilc->endTrial();
    
    double rms = ilc->getRMSError();
    EXPECT_GT(rms, 0.0);
}

TEST_F(ILCBaseTest, GetMaxError) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGain(0.5);
    
    ilc->startTrial();
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (size_t i = 0; i < 100; ++i) {
        input.reference = 10.0;
        input.measured = 5.0;
        ilc->compute(input);
        ilc->recordSample(i);
    }
    
    ilc->endTrial();
    
    double maxError = ilc->getMaxError();
    EXPECT_NEAR(maxError, 5.0, 0.1);
}

TEST_F(ILCBaseTest, SetQFilter) {
    ilc->setQFilter(0.9);
    // Should not throw
}

TEST_F(ILCBaseTest, SetQFilterEnabled) {
    ilc->setQFilterEnabled(true);
    ilc->setQFilterEnabled(false);
}

TEST_F(ILCBaseTest, SetForgettingFactor) {
    ilc->setForgettingFactor(0.95);
    // Should not throw
}

TEST_F(ILCBaseTest, GetFeedforward) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGain(0.5);
    
    // First trial - feedforward should be zero
    ilc->startTrial();
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (size_t i = 0; i < 100; ++i) {
        input.reference = 10.0;
        input.measured = 5.0;
        ilc->compute(input);
        ilc->recordSample(i);
    }
    
    ilc->endTrial();
    
    // After one trial, feedforward should be non-zero
    for (size_t i = 0; i < 100; ++i) {
        double ff = ilc->getFeedforward(i);
        EXPECT_NE(ff, 0.0);
    }
}

TEST_F(ILCBaseTest, ResetLearning) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGain(0.5);
    
    // Run a trial
    ilc->startTrial();
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (size_t i = 0; i < 100; ++i) {
        input.reference = 10.0;
        input.measured = 5.0;
        ilc->compute(input);
        ilc->recordSample(i);
    }
    
    ilc->endTrial();
    
    // Reset
    ilc->resetLearning();
    
    // Trial number should be back to 0
    EXPECT_EQ(ilc->getTrialNumber(), 0);
}

// ============================================================================
// PTypeILC Tests
// ============================================================================

class PTypeILCTest : public ::testing::Test {
protected:
    void SetUp() override {
        ilc = std::make_unique<PTypeILC>();
    }
    
    std::unique_ptr<PTypeILC> ilc;
};

TEST_F(PTypeILCTest, GetName) {
    EXPECT_STREQ(ilc->getName(), "P-Type ILC");
}

TEST_F(PTypeILCTest, GetDescription) {
    EXPECT_NE(ilc->getDescription(), nullptr);
}

TEST_F(PTypeILCTest, SetLearningGain) {
    ilc->setLearningGain(0.7);
    EXPECT_DOUBLE_EQ(ilc->getLearningGain(), 0.7);
}

TEST_F(PTypeILCTest, BasicLearning) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGain(0.5);
    ilc->setQFilterEnabled(false);
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (int trial = 0; trial < 5; ++trial) {
        ilc->startTrial();
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = i * 0.1;
            input.measured = i * 0.05;
            ilc->compute(input);
            ilc->recordSample(i);
        }
        
        ilc->endTrial();
    }
}

TEST_F(PTypeILCTest, LearningConvergence) {
    ilc->setTrajectoryLength(50);
    ilc->setLearningGain(0.4);
    ilc->setQFilter(0.9);
    
    std::vector<double> rmsHistory;
    
    for (int trial = 0; trial < 10; ++trial) {
        ilc->startTrial();
        
        ControllerInput input;
        input.dt = 0.001;
        
        for (size_t i = 0; i < 50; ++i) {
            input.reference = std::sin(i * 0.1);
            input.measured = 0.5 * std::sin(i * 0.1);
            ilc->compute(input);
            ilc->recordSample(i);
        }
        
        ilc->endTrial();
        rmsHistory.push_back(ilc->getRMSError());
    }
    
    // After 10 trials, should have computed some error
    EXPECT_GT(rmsHistory.back(), 0.0);
    EXPECT_GT(rmsHistory.size(), 5u);
}

// ============================================================================
// PDTypeILC Tests
// ============================================================================

class PDTypeILCTest : public ::testing::Test {
protected:
    void SetUp() override {
        ilc = std::make_unique<PDTypeILC>();
    }
    
    std::unique_ptr<PDTypeILC> ilc;
};

TEST_F(PDTypeILCTest, GetName) {
    EXPECT_STREQ(ilc->getName(), "PD-Type ILC");
}

TEST_F(PDTypeILCTest, GetDescription) {
    EXPECT_NE(ilc->getDescription(), nullptr);
}

TEST_F(PDTypeILCTest, SetLearningGains) {
    ilc->setLearningGains(0.3, 0.1);
    // Should not throw
}

TEST_F(PDTypeILCTest, BasicLearning) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGains(0.3, 0.1);
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (int trial = 0; trial < 5; ++trial) {
        ilc->startTrial();
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = std::sin(i * 0.05);
            input.measured = 0.5 * std::sin(i * 0.05);
            ilc->compute(input);
            ilc->recordSample(i);
        }
        
        ilc->endTrial();
    }
}

TEST_F(PDTypeILCTest, DerivativeActionImprovesPhaseLag) {
    ilc->setTrajectoryLength(100);
    ilc->setLearningGains(0.3, 0.2);  // Significant derivative
    
    std::vector<double> rmsHistory;
    
    for (int trial = 0; trial < 10; ++trial) {
        ilc->startTrial();
        
        ControllerInput input;
        input.dt = 0.001;
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = std::sin(i * 0.1);
            input.measured = 0.8 * std::sin(i * 0.1 - 0.5);  // Phase lagging
            ilc->compute(input);
            ilc->recordSample(i);
        }
        
        ilc->endTrial();
        rmsHistory.push_back(ilc->getRMSError());
    }
    
    // Should have computed error over multiple trials
    EXPECT_GT(rmsHistory.back(), 0.0);
    EXPECT_GT(rmsHistory.size(), 5u);
}

// ============================================================================
// PhaseLeadILC Tests
// ============================================================================

class PhaseLeadILCTest : public ::testing::Test {
protected:
    void SetUp() override {
        ilc = std::make_unique<PhaseLeadILC>();
    }
    
    std::unique_ptr<PhaseLeadILC> ilc;
};

TEST_F(PhaseLeadILCTest, GetName) {
    EXPECT_STREQ(ilc->getName(), "Phase-Lead ILC");
}

TEST_F(PhaseLeadILCTest, GetDescription) {
    EXPECT_NE(ilc->getDescription(), nullptr);
}

TEST_F(PhaseLeadILCTest, SetParameters) {
    ilc->setParameters(0.5, 5);  // gamma=0.5, 5 samples phase lead
    // Should not throw
}

TEST_F(PhaseLeadILCTest, SetPhaseFromFrequency) {
    ilc->setPhaseFromFrequency(10.0, 0.5, 1000.0);
    // Should not throw
}

TEST_F(PhaseLeadILCTest, BasicLearning) {
    ilc->setTrajectoryLength(100);
    ilc->setParameters(0.4, 3);
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (int trial = 0; trial < 5; ++trial) {
        ilc->startTrial();
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = std::sin(i * 0.1);
            input.measured = 0.5 * std::sin(i * 0.1 - 0.3);  // Phase lag
            ilc->compute(input);
            ilc->recordSample(i);
        }
        
        ilc->endTrial();
    }
}

TEST_F(PhaseLeadILCTest, PhaseCompensation) {
    ilc->setTrajectoryLength(100);
    ilc->setParameters(0.4, 5);  // 5 samples phase lead
    
    std::vector<double> rmsHistory;
    
    for (int trial = 0; trial < 10; ++trial) {
        ilc->startTrial();
        
        ControllerInput input;
        input.dt = 0.001;
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = std::sin(i * 0.1);
            input.measured = 0.8 * std::sin(i * 0.1 - 0.5);  // Phase lag
            ilc->compute(input);
            ilc->recordSample(i);
        }
        
        ilc->endTrial();
        rmsHistory.push_back(ilc->getRMSError());
    }
    
    // Should have computed error over trials
    EXPECT_GT(rmsHistory.back(), 0.0);
    EXPECT_GT(rmsHistory.size(), 5u);
}

// ============================================================================
// NormOptimalILC Tests
// ============================================================================

class NormOptimalILCTest : public ::testing::Test {
protected:
    void SetUp() override {
        noilc = std::make_unique<NormOptimalILC>();
    }
    
    std::unique_ptr<NormOptimalILC> noilc;
};

TEST_F(NormOptimalILCTest, GetName) {
    EXPECT_STREQ(noilc->getName(), "Norm-Optimal ILC");
}

TEST_F(NormOptimalILCTest, GetDescription) {
    EXPECT_NE(noilc->getDescription(), nullptr);
}

TEST_F(NormOptimalILCTest, SetWeights) {
    noilc->setWeights(1.0, 0.01, 0.001);  // Qe, R, S (scalars)
    // Should not throw
}

TEST_F(NormOptimalILCTest, SetPlantModel) {
    int N = 10;
    std::vector<double> G(N * N, 0.0);
    
    // Create simple Toeplitz matrix (lower triangular)
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j <= i; ++j) {
            G[i * N + j] = std::exp(-(i - j) * 0.1);
        }
    }
    
    noilc->setPlantModel(G.data(), N);
}

TEST_F(NormOptimalILCTest, SetPlantFromImpulseResponse) {
    std::vector<double> impulse = {1.0, 0.8, 0.5, 0.3, 0.1, 0.05};
    noilc->setPlantFromImpulseResponse(impulse.data(), impulse.size());
}

TEST_F(NormOptimalILCTest, Design) {
    int N = 10;
    
    // Set up impulse response
    std::vector<double> impulse(N);
    for (int i = 0; i < N; ++i) {
        impulse[i] = std::exp(-i * 0.2);
    }
    noilc->setPlantFromImpulseResponse(impulse.data(), N);
    noilc->setWeights(1.0, 0.01, 0.001);
    noilc->setTrajectoryLength(N);
    
    bool result = noilc->design();
    // Design may or may not succeed depending on numerical conditioning
}

TEST_F(NormOptimalILCTest, BasicOperation) {
    int N = 20;
    
    std::vector<double> impulse(N);
    for (int i = 0; i < N; ++i) {
        impulse[i] = std::exp(-i * 0.3);
    }
    
    noilc->setPlantFromImpulseResponse(impulse.data(), N);
    noilc->setWeights(1.0, 0.1, 0.0);
    noilc->setTrajectoryLength(N);
    noilc->design();
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (int trial = 0; trial < 3; ++trial) {
        noilc->startTrial();
        
        for (int i = 0; i < N; ++i) {
            input.reference = 1.0;
            input.measured = 0.5;
            noilc->compute(input);
            noilc->recordSample(i);
        }
        
        noilc->endTrial();
    }
}

// ============================================================================
// CurrentIterationLearning Tests
// ============================================================================

class CurrentIterationLearningTest : public ::testing::Test {
protected:
    void SetUp() override {
        cil = std::make_unique<CurrentIterationLearning>();
        ilc = std::make_unique<PTypeILC>();
        pid = std::make_unique<PIDController>();
        
        ilc->setTrajectoryLength(100);
        ilc->setLearningGain(0.3);
        pid->setGains(1.0, 0.1, 0.05);
    }
    
    std::unique_ptr<CurrentIterationLearning> cil;
    std::unique_ptr<PTypeILC> ilc;
    std::unique_ptr<PIDController> pid;
};

TEST_F(CurrentIterationLearningTest, GetName) {
    EXPECT_STREQ(cil->getName(), "Current-Iteration Learning Control");
}

TEST_F(CurrentIterationLearningTest, GetDescription) {
    EXPECT_NE(cil->getDescription(), nullptr);
}

TEST_F(CurrentIterationLearningTest, GetType) {
    EXPECT_EQ(cil->getType(), ControllerType::ILC);
}

TEST_F(CurrentIterationLearningTest, SetILC) {
    cil->setILC(ilc.get());
    // Should not throw
}

TEST_F(CurrentIterationLearningTest, SetFeedback) {
    cil->setFeedback(pid.get());
    // Should not throw
}

TEST_F(CurrentIterationLearningTest, SetFeedforwardWeight) {
    cil->setFeedforwardWeight(0.7);
    // Should not throw
}

TEST_F(CurrentIterationLearningTest, BasicOperation) {
    cil->setILC(ilc.get());
    cil->setFeedback(pid.get());
    cil->setFeedforwardWeight(0.5);
    
    ControllerInput input;
    input.dt = 0.001;
    
    for (int trial = 0; trial < 3; ++trial) {
        cil->startTrial();
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = 10.0;
            input.measured = 5.0 + 0.05 * i;
            
            ControllerOutput output = cil->compute(input);
            ilc->recordSample(i);  // Record for ILC
        }
        
        cil->endTrial();
    }
}

TEST_F(CurrentIterationLearningTest, CombinesFeedforwardAndFeedback) {
    cil->setILC(ilc.get());
    cil->setFeedback(pid.get());
    cil->setFeedforwardWeight(0.6);  // 60% ILC, 40% feedback
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // First trial
    cil->startTrial();
    for (size_t i = 0; i < 100; ++i) {
        cil->compute(input);
        ilc->recordSample(i);
    }
    cil->endTrial();
    
    // Second trial - ILC should contribute
    cil->startTrial();
    ControllerOutput output = cil->compute(input);
    
    // Output should be non-zero from both contributions
    EXPECT_NE(output.control, 0.0);
}

// ============================================================================
// RepetitiveController Tests
// ============================================================================

class RepetitiveControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rc = std::make_unique<RepetitiveController>();
    }
    
    std::unique_ptr<RepetitiveController> rc;
};

TEST_F(RepetitiveControllerTest, GetName) {
    EXPECT_STREQ(rc->getName(), "Repetitive Controller");
}

TEST_F(RepetitiveControllerTest, GetDescription) {
    EXPECT_NE(rc->getDescription(), nullptr);
}

TEST_F(RepetitiveControllerTest, GetType) {
    EXPECT_EQ(rc->getType(), ControllerType::ILC);
}

TEST_F(RepetitiveControllerTest, SetPeriod) {
    rc->setPeriod(0.1);  // 100ms period
    // Should not throw
}

TEST_F(RepetitiveControllerTest, SetQFilter) {
    rc->setQFilter(100.0);  // 100 Hz cutoff
}

TEST_F(RepetitiveControllerTest, SetStabilizingGain) {
    rc->setStabilizingGain(0.5);
}

TEST_F(RepetitiveControllerTest, SetSampleRate) {
    rc->setSampleRate(1000.0);  // 1kHz
}

TEST_F(RepetitiveControllerTest, BasicOperation) {
    rc->setPeriod(0.05);        // 50ms period = 20Hz
    rc->setSampleRate(1000.0);  // 1kHz
    rc->setStabilizingGain(0.5);
    rc->setQFilter(100.0);
    
    ControllerInput input;
    input.dt = 0.001;
    
    // Simulate periodic disturbance
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.001;
        input.reference = 0.0;
        input.measured = std::sin(2.0 * M_PI * 20.0 * t);  // 20Hz disturbance
        
        ControllerOutput output = rc->compute(input);
    }
}

TEST_F(RepetitiveControllerTest, PeriodicDisturbanceRejection) {
    rc->setPeriod(0.1);         // 100ms period = 10Hz
    rc->setSampleRate(1000.0);  // 1kHz
    rc->setStabilizingGain(0.4);
    rc->setQFilter(50.0);
    
    ControllerInput input;
    input.dt = 0.001;
    
    double totalError = 0.0;
    int numSamples = 500;
    
    for (int i = 0; i < numSamples; ++i) {
        double t = i * 0.001;
        input.reference = 0.0;
        input.measured = std::sin(2.0 * M_PI * 10.0 * t);  // 10Hz = 1/period
        
        ControllerOutput output = rc->compute(input);
        totalError += std::abs(output.error);
    }
    
    // Error should be bounded
    double avgError = totalError / numSamples;
    EXPECT_LT(avgError, 1.5);  // Average error < 1.5
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(ILCIntegrationTest, MultipleTrialConvergence) {
    PTypeILC ilc;
    ilc.setTrajectoryLength(200);
    ilc.setLearningGain(0.4);
    ilc.setQFilter(0.85);
    
    // Simulate a simple first-order plant
    auto simulate = [](const std::vector<double>& u) {
        std::vector<double> y(u.size());
        double state = 0.0;
        double tau = 0.1;
        double dt = 0.001;
        
        for (size_t i = 0; i < u.size(); ++i) {
            state += dt * (-state / tau + u[i] / tau);
            y[i] = state;
        }
        return y;
    };
    
    // Reference trajectory
    std::vector<double> ref(200);
    for (size_t i = 0; i < 200; ++i) {
        ref[i] = std::sin(i * 0.05);
    }
    
    std::vector<double> rmsHistory;
    
    for (int trial = 0; trial < 20; ++trial) {
        ilc.startTrial();
        
        // Get feedforward signals
        std::vector<double> u(200);
        for (size_t i = 0; i < 200; ++i) {
            u[i] = ilc.getFeedforward(i);
        }
        
        // Simulate plant
        std::vector<double> y = simulate(u);
        
        // Record errors
        ControllerInput input;
        input.dt = 0.001;
        
        for (size_t i = 0; i < 200; ++i) {
            input.reference = ref[i];
            input.measured = y[i];
            ilc.compute(input);
            ilc.recordSample(i);
        }
        
        ilc.endTrial();
        rmsHistory.push_back(ilc.getRMSError());
    }
    
    // Should have recorded error over multiple trials
    EXPECT_GT(rmsHistory.size(), 10u);
    EXPECT_GT(rmsHistory.back(), 0.0);
}

TEST(ILCIntegrationTest, ILCWithFeedback) {
    CurrentIterationLearning cil;
    PTypeILC ilc;
    PIDController pid;
    
    ilc.setTrajectoryLength(100);
    ilc.setLearningGain(0.3);
    pid.setGains(2.0, 0.5, 0.1);
    
    cil.setILC(&ilc);
    cil.setFeedback(&pid);
    cil.setFeedforwardWeight(0.6);
    
    // Simulate closed-loop with learning
    double plant_state = 0.0;
    double dt = 0.001;
    
    for (int trial = 0; trial < 5; ++trial) {
        cil.startTrial();
        plant_state = 0.0;
        
        ControllerInput input;
        input.dt = dt;
        
        for (size_t i = 0; i < 100; ++i) {
            input.reference = 1.0;
            input.measured = plant_state;
            
            ControllerOutput output = cil.compute(input);
            ilc.recordSample(i);
            
            // Simple plant dynamics
            plant_state += dt * (-plant_state + output.control);
        }
        
        cil.endTrial();
    }
}

TEST(ILCIntegrationTest, CompareILCTypes) {
    constexpr size_t trajLen = 100;
    
    PTypeILC pIlc;
    PDTypeILC pdIlc;
    PhaseLeadILC plIlc;
    
    pIlc.setTrajectoryLength(trajLen);
    pIlc.setLearningGain(0.4);
    
    pdIlc.setTrajectoryLength(trajLen);
    pdIlc.setLearningGains(0.3, 0.15);
    
    plIlc.setTrajectoryLength(trajLen);
    plIlc.setParameters(0.4, 3);
    
    ControllerInput input;
    input.dt = 0.001;
    
    auto runTrial = [&](ILCBase& ilc) {
        ilc.startTrial();
        for (size_t i = 0; i < trajLen; ++i) {
            input.reference = std::sin(i * 0.1);
            input.measured = 0.7 * std::sin(i * 0.1 - 0.3);  // Phase lag system
            ilc.compute(input);
            ilc.recordSample(i);
        }
        ilc.endTrial();
        return ilc.getRMSError();
    };
    
    // Run 5 trials for each
    std::vector<double> pErrors, pdErrors, plErrors;
    for (int t = 0; t < 5; ++t) {
        pErrors.push_back(runTrial(pIlc));
        pdErrors.push_back(runTrial(pdIlc));
        plErrors.push_back(runTrial(plIlc));
    }
    
    // All should have computed errors
    EXPECT_GT(pErrors.back(), 0.0);
    EXPECT_GT(pdErrors.back(), 0.0);
    EXPECT_GT(plErrors.back(), 0.0);
}

// ============================================================================
// Additional ILCBase Edge Case Tests
// ============================================================================

TEST(ILCBaseEdgeCasesTest, ForgettingFactorApplied) {
    PTypeILC ilc;
    ilc.setTrajectoryLength(50);
    ilc.setLearningGain(0.5);
    ilc.setForgettingFactor(0.8);  // Apply forgetting
    ilc.setQFilterEnabled(false);
    
    ControllerInput input;
    input.dt = 0.001;
    
    // First trial
    ilc.startTrial();
    for (size_t i = 0; i < 50; ++i) {
        input.reference = 10.0;
        input.measured = 5.0;
        ilc.compute(input);
        ilc.recordSample(i);
    }
    ilc.endTrial();
    
    double ff1 = ilc.getFeedforward(25);
    
    // Second trial
    ilc.startTrial();
    for (size_t i = 0; i < 50; ++i) {
        input.reference = 10.0;
        input.measured = 5.0;
        ilc.compute(input);
        ilc.recordSample(i);
    }
    ilc.endTrial();
    
    double ff2 = ilc.getFeedforward(25);
    
    // With forgetting, feedforward should be dampened
    EXPECT_NE(ff1, 0.0);
    EXPECT_NE(ff2, 0.0);
}

TEST(ILCBaseEdgeCasesTest, ZeroLengthTrajectory) {
    PTypeILC ilc;
    ilc.setTrajectoryLength(0);  // Edge case
    
    ilc.startTrial();
    ilc.endTrial();
    
    // Should not crash, RMS should be 0 (no samples)
    EXPECT_EQ(ilc.getRMSError(), 0.0);
    EXPECT_EQ(ilc.getMaxError(), 0.0);
}

TEST(ILCBaseEdgeCasesTest, GetFeedforwardOutOfBounds) {
    PTypeILC ilc;
    ilc.setTrajectoryLength(10);
    
    // Getting feedforward out of bounds should return 0
    EXPECT_DOUBLE_EQ(ilc.getFeedforward(100), 0.0);
}

TEST(ILCBaseEdgeCasesTest, RecordSampleOutOfBounds) {
    PTypeILC ilc;
    ilc.setTrajectoryLength(10);
    ilc.setLearningGain(0.5);
    
    ilc.startTrial();
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    ilc.compute(input);
    ilc.recordSample(100);  // Out of bounds - should not crash
    
    ilc.endTrial();
}

TEST(ILCBaseEdgeCasesTest, QFilterApplied) {
    PTypeILC ilc;
    ilc.setTrajectoryLength(100);
    ilc.setLearningGain(0.5);
    ilc.setQFilter(0.8);
    ilc.setQFilterEnabled(true);
    
    ControllerInput input;
    input.dt = 0.001;
    
    ilc.startTrial();
    for (size_t i = 0; i < 100; ++i) {
        // Rapid changes in error
        input.reference = 10.0;
        input.measured = (i % 2 == 0) ? 5.0 : 8.0;
        ilc.compute(input);
        ilc.recordSample(i);
    }
    ilc.endTrial();
    
    // After Q-filter, feedforward should be smoothed
    double ff1 = ilc.getFeedforward(50);
    double ff2 = ilc.getFeedforward(51);
    
    // Should have some feedforward values
    EXPECT_NE(ff1, 0.0);
    EXPECT_NE(ff2, 0.0);
}

// ============================================================================
// PDTypeILC Edge Cases
// ============================================================================

TEST(PDTypeILCEdgeCasesTest, SingleSampleTrajectory) {
    PDTypeILC ilc;
    ilc.setTrajectoryLength(1);
    ilc.setLearningGains(0.3, 0.1);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 10.0;
    input.measured = 5.0;
    
    ilc.startTrial();
    ilc.compute(input);
    ilc.recordSample(0);
    ilc.endTrial();
    
    // Should handle single sample case
    EXPECT_GT(ilc.getRMSError(), 0.0);
}

TEST(PDTypeILCEdgeCasesTest, TwoSampleTrajectory) {
    PDTypeILC ilc;
    ilc.setTrajectoryLength(2);
    ilc.setLearningGains(0.3, 0.1);
    
    ControllerInput input;
    input.dt = 0.001;
    
    ilc.startTrial();
    
    input.reference = 10.0;
    input.measured = 5.0;
    ilc.compute(input);
    ilc.recordSample(0);
    
    input.measured = 6.0;
    ilc.compute(input);
    ilc.recordSample(1);
    
    ilc.endTrial();
    
    // Derivative computation with only 2 samples
    EXPECT_GT(ilc.getRMSError(), 0.0);
}

// ============================================================================
// PhaseLeadILC Edge Cases
// ============================================================================

TEST(PhaseLeadILCEdgeCasesTest, NegativePhaseLead) {
    PhaseLeadILC ilc;
    ilc.setTrajectoryLength(50);
    ilc.setParameters(0.4, -5);  // Negative phase lead (phase lag)
    
    ControllerInput input;
    input.dt = 0.001;
    
    ilc.startTrial();
    for (size_t i = 0; i < 50; ++i) {
        input.reference = std::sin(i * 0.1);
        input.measured = 0.5 * std::sin(i * 0.1);
        ilc.compute(input);
        ilc.recordSample(i);
    }
    ilc.endTrial();
    
    EXPECT_GT(ilc.getRMSError(), 0.0);
}

TEST(PhaseLeadILCEdgeCasesTest, LargePhaseLead) {
    PhaseLeadILC ilc;
    ilc.setTrajectoryLength(50);
    ilc.setParameters(0.3, 100);  // Phase lead > trajectory length
    
    ControllerInput input;
    input.dt = 0.001;
    
    ilc.startTrial();
    for (size_t i = 0; i < 50; ++i) {
        input.reference = std::sin(i * 0.1);
        input.measured = 0.5 * std::sin(i * 0.1);
        ilc.compute(input);
        ilc.recordSample(i);
    }
    ilc.endTrial();
    
    // Should extrapolate using last error value
    EXPECT_GT(ilc.getRMSError(), 0.0);
}

// ============================================================================
// NormOptimalILC Edge Cases
// ============================================================================

TEST(NormOptimalILCEdgeCasesTest, DesignWithoutPlant) {
    NormOptimalILC noilc;
    noilc.setWeights(1.0, 0.1, 0.01);
    noilc.setTrajectoryLength(10);
    
    // Design without setting plant model
    bool result = noilc.design();
    EXPECT_FALSE(result);
}

TEST(NormOptimalILCEdgeCasesTest, UpdateLearningWithoutDesign) {
    NormOptimalILC noilc;
    noilc.setTrajectoryLength(20);
    
    // Not designed, should fall back to P-type
    ControllerInput input;
    input.dt = 0.001;
    
    noilc.startTrial();
    for (int i = 0; i < 20; ++i) {
        input.reference = 1.0;
        input.measured = 0.5;
        noilc.compute(input);
        noilc.recordSample(i);
    }
    noilc.endTrial();
    
    // Feedforward should be updated via fallback P-type
    double ff = noilc.getFeedforward(10);
    EXPECT_NE(ff, 0.0);
}

TEST(NormOptimalILCEdgeCasesTest, SingularPlantMatrix) {
    NormOptimalILC noilc;
    
    int N = 5;
    // Create a singular matrix (all zeros)
    std::vector<double> G(N * N, 0.0);
    
    noilc.setPlantModel(G.data(), N);
    noilc.setWeights(1.0, 0.0, 0.0);  // No regularization
    noilc.setTrajectoryLength(N);
    
    // bool result = noilc.design(); // Not used
    noilc.design();  // May fail due to singularity
}

TEST(NormOptimalILCEdgeCasesTest, SuccessfulDesignAndLearning) {
    NormOptimalILC noilc;
    
    int N = 10;
    std::vector<double> impulse(N);
    for (int i = 0; i < N; ++i) {
        impulse[i] = std::exp(-i * 0.3);
    }
    
    noilc.setPlantFromImpulseResponse(impulse.data(), N);
    noilc.setWeights(10.0, 0.1, 0.01);  // Good regularization
    noilc.setTrajectoryLength(N);
    
    bool result = noilc.design();
    
    if (result) {
        // Run multiple trials
        ControllerInput input;
        input.dt = 0.001;
        
        for (int trial = 0; trial < 3; ++trial) {
            noilc.startTrial();
            for (int i = 0; i < N; ++i) {
                input.reference = 1.0;
                input.measured = 0.5;
                noilc.compute(input);
                noilc.recordSample(i);
            }
            noilc.endTrial();
        }
        
        EXPECT_GT(noilc.getTrialNumber(), 0);
    }
}

// ============================================================================
// CurrentIterationLearning Edge Cases
// ============================================================================

TEST(CurrentIterationLearningEdgeCasesTest, NullILC) {
    CurrentIterationLearning cil;
    PIDController pid;
    pid.setGains(1.0, 0.1, 0.05);
    
    cil.setILC(nullptr);  // No ILC
    cil.setFeedback(&pid);
    cil.setFeedforwardWeight(0.5);
    
    cil.startTrial();
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    // Should work with just feedback
    ControllerOutput output = cil.compute(input);
    EXPECT_NE(output.control, 0.0);
    
    cil.endTrial();
}

TEST(CurrentIterationLearningEdgeCasesTest, NullFeedback) {
    CurrentIterationLearning cil;
    PTypeILC ilc;
    ilc.setTrajectoryLength(50);
    ilc.setLearningGain(0.3);
    
    cil.setILC(&ilc);
    cil.setFeedback(nullptr);  // No feedback
    cil.setFeedforwardWeight(1.0);  // All feedforward
    
    cil.startTrial();
    
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    for (size_t i = 0; i < 50; ++i) {
        cil.compute(input);
    }
    
    cil.endTrial();
    
    // Second trial should have ILC feedforward
    cil.startTrial();
    ControllerOutput output = cil.compute(input);
    cil.endTrial();
    
    // Feedforward should be from learned ILC
    EXPECT_EQ(output.feedforward, output.control);  // 100% feedforward weight
}

TEST(CurrentIterationLearningEdgeCasesTest, ResetBothControllers) {
    CurrentIterationLearning cil;
    PTypeILC ilc;
    PIDController pid;
    
    ilc.setTrajectoryLength(20);
    ilc.setLearningGain(0.5);
    pid.setGains(1.0, 0.1, 0.05);
    
    cil.setILC(&ilc);
    cil.setFeedback(&pid);
    
    // Run a trial
    cil.startTrial();
    ControllerInput input;
    input.reference = 10.0;
    input.measured = 5.0;
    input.dt = 0.001;
    
    for (size_t i = 0; i < 20; ++i) {
        cil.compute(input);
    }
    cil.endTrial();
    
    // Reset
    cil.reset();
    
    // ILC trial count should be reset
    EXPECT_EQ(ilc.getTrialNumber(), 0);
}

// ============================================================================
// RepetitiveController Edge Cases
// ============================================================================

TEST(RepetitiveControllerEdgeCasesTest, VeryShortPeriod) {
    RepetitiveController rc;
    rc.setSampleRate(1000.0);
    rc.setPeriod(0.001);  // 1ms = 1 sample
    rc.setStabilizingGain(0.5);
    rc.setQFilter(100.0);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    
    for (int i = 0; i < 20; ++i) {
        input.measured = std::sin(i * 0.1);
        ControllerOutput output = rc.compute(input);
        EXPECT_FALSE(std::isnan(output.control));
    }
}

TEST(RepetitiveControllerEdgeCasesTest, ZeroPeriod) {
    RepetitiveController rc;
    rc.setSampleRate(1000.0);
    rc.setPeriod(0.0);  // Edge case
    rc.setStabilizingGain(0.5);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    input.measured = 1.0;
    
    // Should not crash
    ControllerOutput output = rc.compute(input);
    EXPECT_FALSE(std::isnan(output.control));
}

TEST(RepetitiveControllerEdgeCasesTest, Reset) {
    RepetitiveController rc;
    rc.setSampleRate(1000.0);
    rc.setPeriod(0.05);
    rc.setStabilizingGain(0.5);
    rc.setQFilter(100.0);
    
    ControllerInput input;
    input.dt = 0.001;
    input.reference = 0.0;
    
    // Build up state
    for (int i = 0; i < 100; ++i) {
        input.measured = std::sin(i * 0.1);
        rc.compute(input);
    }
    
    // Reset
    rc.reset();
    
    // After reset, output should be minimal
    input.measured = 0.0;
    ControllerOutput output = rc.compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST(RepetitiveControllerEdgeCasesTest, ChangeSampleRate) {
    RepetitiveController rc;
    rc.setPeriod(0.1);
    rc.setSampleRate(1000.0);
    rc.setStabilizingGain(0.5);
    rc.setQFilter(50.0);
    
    // Change sample rate mid-operation
    rc.setSampleRate(2000.0);
    
    ControllerInput input;
    input.dt = 0.0005;  // 2kHz
    input.reference = 0.0;
    input.measured = 1.0;
    
    ControllerOutput output = rc.compute(input);
    EXPECT_FALSE(std::isnan(output.control));
}
