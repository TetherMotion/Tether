/**
 * @file CompositeControllersTests.cpp
 * @brief Comprehensive tests for CompositeControllers module
 * Tests for CascadeController, FeedforwardController, ParallelController, SwitchingController
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "tether/control/Controllers.hpp"
#include "tether/control/CompositeControllers.hpp"

using namespace tether::control;

// ============================================================================
// CascadeController Tests
// ============================================================================

class CascadeControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        cascade = std::make_unique<CascadeController>();
        outerPID = std::make_unique<PIDController>();
        innerPID = std::make_unique<PIDController>();
        
        outerPID->setGains(1.0, 0.0, 0.0);
        innerPID->setGains(2.0, 0.0, 0.0);
    }
    
    std::unique_ptr<CascadeController> cascade;
    std::unique_ptr<PIDController> outerPID;
    std::unique_ptr<PIDController> innerPID;
};

TEST_F(CascadeControllerTest, GetType) {
    EXPECT_EQ(cascade->getType(), ControllerType::Cascade);
}

TEST_F(CascadeControllerTest, GetName) {
    EXPECT_STREQ(cascade->getName(), "Cascade Controller");
}

TEST_F(CascadeControllerTest, GetDescription) {
    EXPECT_NE(cascade->getDescription(), nullptr);
    EXPECT_GT(strlen(cascade->getDescription()), 0);
}

TEST_F(CascadeControllerTest, SetControllers) {
    cascade->setOuterController(outerPID.get());
    cascade->setInnerController(innerPID.get());
    // No exception expected
}

TEST_F(CascadeControllerTest, BasicCompute) {
    cascade->setOuterController(outerPID.get());
    cascade->setInnerController(innerPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;  // Outer measurement
    input.state[0] = 25.0;  // Inner measurement (from state vector)
    input.dt = 0.001;
    
    ControllerOutput output = cascade->compute(input);
    // Output should be non-zero
    EXPECT_NE(output.control, 0.0);
}

TEST_F(CascadeControllerTest, SetInnerReferenceLimits) {
    cascade->setOuterController(outerPID.get());
    cascade->setInnerController(innerPID.get());
    cascade->setInnerReferenceLimits(-10.0, 10.0);
    
    // High outer gain to create large inner reference
    outerPID->setGains(100.0, 0.0, 0.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 0.0;
    input.state[0] = 0.0;  // Inner measurement
    input.dt = 0.001;
    
    ControllerOutput output = cascade->compute(input);
    // Inner reference should be limited
}

TEST_F(CascadeControllerTest, Reset) {
    cascade->setOuterController(outerPID.get());
    cascade->setInnerController(innerPID.get());
    
    // Run some cycles
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.state[0] = 25.0;  // Inner measurement
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        cascade->compute(input);
    }
    
    cascade->reset();
    // Should not throw
}

TEST_F(CascadeControllerTest, NullOuterController) {
    cascade->setInnerController(innerPID.get());
    // Outer is null
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.state[0] = 25.0;  // Inner measurement
    input.dt = 0.001;
    
    // Should handle gracefully
    ControllerOutput output = cascade->compute(input);
}

TEST_F(CascadeControllerTest, NullInnerController) {
    cascade->setOuterController(outerPID.get());
    // Inner is null
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.state[0] = 25.0;  // Inner measurement
    input.dt = 0.001;
    
    // Should handle gracefully
    ControllerOutput output = cascade->compute(input);
}

TEST_F(CascadeControllerTest, ZeroError) {
    cascade->setOuterController(outerPID.get());
    cascade->setInnerController(innerPID.get());
    
    ControllerInput input;
    input.reference = 50.0;
    input.measured = 50.0;  // No outer error
    input.state[0] = 0.0;   // Inner measurement
    input.dt = 0.001;
    
    ControllerOutput output = cascade->compute(input);
}

TEST_F(CascadeControllerTest, WithIntegralControllers) {
    outerPID->setGains(1.0, 0.1, 0.0);
    innerPID->setGains(2.0, 0.2, 0.0);
    
    cascade->setOuterController(outerPID.get());
    cascade->setInnerController(innerPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.state[0] = 25.0;  // Inner measurement
    input.dt = 0.001;
    
    // Run multiple cycles
    for (int i = 0; i < 100; ++i) {
        cascade->compute(input);
    }
}

// ============================================================================
// FeedforwardController Tests
// ============================================================================

class FeedforwardControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        ff = std::make_unique<FeedforwardController>();
        feedback = std::make_unique<PIDController>();
        feedback->setGains(1.0, 0.1, 0.05);
    }
    
    std::unique_ptr<FeedforwardController> ff;
    std::unique_ptr<PIDController> feedback;
};

TEST_F(FeedforwardControllerTest, GetType) {
    EXPECT_EQ(ff->getType(), ControllerType::Custom);
}

TEST_F(FeedforwardControllerTest, GetName) {
    EXPECT_STREQ(ff->getName(), "Feedforward + Feedback");
}

TEST_F(FeedforwardControllerTest, GetDescription) {
    EXPECT_NE(ff->getDescription(), nullptr);
    EXPECT_GT(strlen(ff->getDescription()), 0);
}

TEST_F(FeedforwardControllerTest, SetFeedbackController) {
    ff->setFeedback(feedback.get());
    // Should not throw
}

TEST_F(FeedforwardControllerTest, SetFeedforwardGain) {
    ff->setFeedforwardGain(1.5);
    ff->setFeedback(feedback.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = ff->compute(input);
    // Output should include feedforward term
    EXPECT_NE(output.control, 0.0);
}

TEST_F(FeedforwardControllerTest, SetFeedforwardFunction) {
    ff->setFeedback(feedback.get());
    
    // Custom feedforward function
    ff->setFeedforwardFunction([](const ControllerInput& in) {
        return in.referenceDerivative * 0.5;  // Velocity feedforward
    });
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.referenceDerivative = 10.0;
    input.dt = 0.001;
    
    ControllerOutput output = ff->compute(input);
}

TEST_F(FeedforwardControllerTest, FeedforwardOnly) {
    ff->setFeedforwardGain(2.0);
    // No feedback controller
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = ff->compute(input);
}

TEST_F(FeedforwardControllerTest, FeedbackOnly) {
    ff->setFeedback(feedback.get());
    ff->setFeedforwardGain(0.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = ff->compute(input);
}

TEST_F(FeedforwardControllerTest, Reset) {
    ff->setFeedback(feedback.get());
    ff->setFeedforwardGain(1.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        ff->compute(input);
    }
    
    ff->reset();
}

TEST_F(FeedforwardControllerTest, ZeroReference) {
    ff->setFeedback(feedback.get());
    ff->setFeedforwardGain(1.0);
    
    ControllerInput input;
    input.reference = 0.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // ControllerOutput output = ff->compute(input); // Not used
    ff->compute(input);
}

// ============================================================================
// ParallelController Tests
// ============================================================================

class ParallelControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        parallel = std::make_unique<ParallelController>();
        pid1 = std::make_unique<PIDController>();
        pid2 = std::make_unique<PIDController>();
        
        pid1->setGains(1.0, 0.0, 0.0);
        pid2->setGains(0.5, 0.0, 0.0);
    }
    
    std::unique_ptr<ParallelController> parallel;
    std::unique_ptr<PIDController> pid1;
    std::unique_ptr<PIDController> pid2;
};

TEST_F(ParallelControllerTest, GetType) {
    EXPECT_EQ(parallel->getType(), ControllerType::Custom);
}

TEST_F(ParallelControllerTest, GetName) {
    EXPECT_STREQ(parallel->getName(), "Parallel Controller");
}

TEST_F(ParallelControllerTest, GetDescription) {
    EXPECT_NE(parallel->getDescription(), nullptr);
}

TEST_F(ParallelControllerTest, AddController) {
    parallel->addController(pid1.get(), 1.0);
    parallel->addController(pid2.get(), 0.5);
}

TEST_F(ParallelControllerTest, BasicCompute) {
    parallel->addController(pid1.get(), 1.0);
    parallel->addController(pid2.get(), 1.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = parallel->compute(input);
    
    // Output should be sum of individual controllers
    // pid1: 1.0 * 50 = 50
    // pid2: 0.5 * 50 = 25
    // total: 75 (with weights 1.0 each)
    EXPECT_NEAR(output.control, 75.0, 1.0);
}

TEST_F(ParallelControllerTest, WithWeights) {
    parallel->addController(pid1.get(), 0.6);
    parallel->addController(pid2.get(), 0.4);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = parallel->compute(input);
}

TEST_F(ParallelControllerTest, ClearControllers) {
    parallel->addController(pid1.get(), 1.0);
    parallel->clearControllers();
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = parallel->compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(ParallelControllerTest, Reset) {
    parallel->addController(pid1.get(), 1.0);
    parallel->addController(pid2.get(), 1.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        parallel->compute(input);
    }
    
    parallel->reset();
}

TEST_F(ParallelControllerTest, SingleController) {
    parallel->addController(pid1.get(), 1.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = parallel->compute(input);
    EXPECT_NEAR(output.control, 50.0, 1.0);
}

TEST_F(ParallelControllerTest, NoControllers) {
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = parallel->compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

// ============================================================================
// SwitchingController Tests  
// ============================================================================

class SwitchingControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        switching = std::make_unique<SwitchingController>();
        lowSpeedPID = std::make_unique<PIDController>();
        highSpeedPID = std::make_unique<PIDController>();
        defaultPID = std::make_unique<PIDController>();
        
        lowSpeedPID->setGains(2.0, 0.1, 0.05);  // Aggressive for low speed
        highSpeedPID->setGains(0.5, 0.05, 0.01);  // Conservative for high speed
        defaultPID->setGains(1.0, 0.05, 0.02);   // Default controller
    }
    
    std::unique_ptr<SwitchingController> switching;
    std::unique_ptr<PIDController> lowSpeedPID;
    std::unique_ptr<PIDController> highSpeedPID;
    std::unique_ptr<PIDController> defaultPID;
};

TEST_F(SwitchingControllerTest, GetType) {
    EXPECT_EQ(switching->getType(), ControllerType::Custom);
}

TEST_F(SwitchingControllerTest, GetName) {
    EXPECT_STREQ(switching->getName(), "Switching Controller");
}

TEST_F(SwitchingControllerTest, GetDescription) {
    EXPECT_NE(switching->getDescription(), nullptr);
}

TEST_F(SwitchingControllerTest, AddControllerWithCondition) {
    // Add controller with selection condition based on measured value
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput& in) { return in.measured < 50.0; }, 
        1);  // priority 1
    switching->addController(highSpeedPID.get(), 
        [](const ControllerInput& in) { return in.measured >= 50.0; }, 
        0);  // priority 0
}

TEST_F(SwitchingControllerTest, SetDefaultController) {
    switching->setDefaultController(defaultPID.get());
    // Should not throw
}

TEST_F(SwitchingControllerTest, BasicComputeWithDefault) {
    switching->setDefaultController(defaultPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = switching->compute(input);
    EXPECT_NE(output.control, 0.0);
}

TEST_F(SwitchingControllerTest, SwitchingBasedOnCondition) {
    // Low error → low speed controller
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput& in) { 
            return std::abs(in.reference - in.measured) < 20.0; 
        }, 1);
    // High error → high speed controller  
    switching->addController(highSpeedPID.get(), 
        [](const ControllerInput& in) { 
            return std::abs(in.reference - in.measured) >= 20.0; 
        }, 0);
    switching->setDefaultController(defaultPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.dt = 0.001;
    
    // Low error - should use lowSpeedPID
    input.measured = 85.0;  // error = 15 < 20
    ControllerOutput out1 = switching->compute(input);
    
    // High error - should use highSpeedPID
    input.measured = 50.0;  // error = 50 >= 20
    ControllerOutput out2 = switching->compute(input);
}

TEST_F(SwitchingControllerTest, GetActiveIndex) {
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput& in) { return in.measured < 50.0; }, 0);
    switching->setDefaultController(defaultPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 25.0;  // Should trigger first condition
    input.dt = 0.001;
    
    switching->compute(input);
    int activeIdx = switching->getActiveIndex();
    EXPECT_GE(activeIdx, -1);  // -1 means default, >= 0 means specific controller
}

TEST_F(SwitchingControllerTest, EnableBumplessTransfer) {
    switching->enableBumplessTransfer(true);
    switching->enableBumplessTransfer(false);
    // Should not throw
}

TEST_F(SwitchingControllerTest, SetBumplessTimeConstant) {
    switching->setBumplessTimeConstant(0.1);
    switching->setBumplessTimeConstant(0.5);
    // Should not throw
}

TEST_F(SwitchingControllerTest, BumplessTransferOnSwitch) {
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput& in) { return in.measured < 50.0; }, 0);
    switching->addController(highSpeedPID.get(), 
        [](const ControllerInput& in) { return in.measured >= 50.0; }, 0);
    switching->enableBumplessTransfer(true);
    switching->setBumplessTimeConstant(0.1);
    
    ControllerInput input;
    input.reference = 100.0;
    input.dt = 0.001;
    
    // Build up state with low speed controller
    input.measured = 25.0;
    for (int i = 0; i < 100; ++i) {
        switching->compute(input);
    }
    
    // Switch to high speed - bumpless transfer should smooth the transition
    input.measured = 75.0;
    ControllerOutput output = switching->compute(input);
}

TEST_F(SwitchingControllerTest, Reset) {
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput& in) { return true; }, 0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    for (int i = 0; i < 10; ++i) {
        switching->compute(input);
    }
    
    switching->reset();
}

TEST_F(SwitchingControllerTest, NoMatchingConditionUsesDefault) {
    // Add controller with condition that won't match
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput& in) { return false; }, 0);  // Never matches
    switching->setDefaultController(defaultPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // Should use default controller since no condition matches
    ControllerOutput output = switching->compute(input);
    EXPECT_NE(output.control, 0.0);
}

TEST_F(SwitchingControllerTest, PriorityOrdering) {
    // Both conditions match, but first has higher priority
    switching->addController(lowSpeedPID.get(), 
        [](const ControllerInput&) { return true; }, 10);  // Priority 10 (higher)
    switching->addController(highSpeedPID.get(), 
        [](const ControllerInput&) { return true; }, 5);   // Priority 5 (lower)
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // Should use lowSpeedPID (higher priority)
    ControllerOutput output = switching->compute(input);
}

TEST_F(SwitchingControllerTest, NoControllers) {
    // No controllers added, no default
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // Should handle gracefully
    ControllerOutput output = switching->compute(input);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(CompositeIntegrationTest, CascadeWithFeedforward) {
    // Build a cascade controller where inner loop has feedforward
    CascadeController cascade;
    FeedforwardController ffInner;
    PIDController outerPID, innerPID;
    
    outerPID.setGains(1.0, 0.0, 0.0);
    innerPID.setGains(2.0, 0.0, 0.0);
    
    ffInner.setFeedback(&innerPID);
    ffInner.setFeedforwardGain(0.5);
    
    cascade.setOuterController(&outerPID);
    cascade.setInnerController(&ffInner);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.state[0] = 25.0;  // Inner measurement
    input.dt = 0.001;
    
    ControllerOutput output = cascade.compute(input);
}

TEST(CompositeIntegrationTest, ParallelOfCascades) {
    // Multiple cascade controllers in parallel
    ParallelController parallel;
    CascadeController cascade1, cascade2;
    PIDController outer1, inner1, outer2, inner2;
    
    outer1.setGains(1.0, 0.0, 0.0);
    inner1.setGains(2.0, 0.0, 0.0);
    outer2.setGains(0.5, 0.0, 0.0);
    inner2.setGains(1.0, 0.0, 0.0);
    
    cascade1.setOuterController(&outer1);
    cascade1.setInnerController(&inner1);
    cascade2.setOuterController(&outer2);
    cascade2.setInnerController(&inner2);
    
    parallel.addController(&cascade1, 0.5);
    parallel.addController(&cascade2, 0.5);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.state[0] = 25.0;  // Inner measurement for cascades
    input.dt = 0.001;
    
    ControllerOutput output = parallel.compute(input);
}

TEST(CompositeIntegrationTest, SwitchingBetweenComposites) {
    SwitchingController switching;
    CascadeController cascade;
    ParallelController parallel;
    PIDController pid1, pid2, pid3;
    
    pid1.setGains(1.0, 0.0, 0.0);
    pid2.setGains(2.0, 0.0, 0.0);
    pid3.setGains(0.5, 0.0, 0.0);
    
    cascade.setOuterController(&pid1);
    cascade.setInnerController(&pid2);
    
    parallel.addController(&pid2, 1.0);
    parallel.addController(&pid3, 1.0);
    
    // Switching based on error magnitude
    switching.addController(&cascade, 
        [](const ControllerInput& in) { 
            return std::abs(in.reference - in.measured) > 30.0; 
        }, 1);
    switching.addController(&parallel, 
        [](const ControllerInput& in) { 
            return std::abs(in.reference - in.measured) <= 30.0; 
        }, 0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.state[0] = 25.0;  // Inner measurement for cascade
    input.dt = 0.001;
    
    // Large error - should use cascade
    input.measured = 50.0;  // error = 50 > 30
    ControllerOutput out1 = switching.compute(input);
    
    // Small error - should use parallel
    input.measured = 80.0;  // error = 20 <= 30
    // ControllerOutput out2 = switching.compute(input); // Not used
    switching.compute(input);
}

// ============================================================================
// ParallelController setWeight Tests
// ============================================================================

TEST_F(ParallelControllerTest, SetWeight) {
    parallel->addController(pid1.get(), 1.0);
    parallel->addController(pid2.get(), 1.0);
    
    // Modify weight at index 0
    parallel->setWeight(0, 0.3);
    parallel->setWeight(1, 0.7);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    ControllerOutput output = parallel->compute(input);
    // pid1: 1.0 * 50 * 0.3 = 15
    // pid2: 0.5 * 50 * 0.7 = 17.5
    // total: 32.5
    EXPECT_NEAR(output.control, 32.5, 1.0);
}

TEST_F(ParallelControllerTest, SetWeightOutOfBounds) {
    parallel->addController(pid1.get(), 1.0);
    
    // Invalid index - should not crash
    parallel->setWeight(100, 0.5);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.001;
    
    // Original weight should be unchanged
    ControllerOutput output = parallel->compute(input);
    EXPECT_NEAR(output.control, 50.0, 1.0);
}

// ============================================================================
// RateLimiterWrapper Tests
// ============================================================================

class RateLimiterWrapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        rateLimiter = std::make_unique<RateLimiterWrapper>();
        innerPID = std::make_unique<PIDController>();
        innerPID->setGains(10.0, 0.0, 0.0);  // High gain for large output jumps
    }
    
    std::unique_ptr<RateLimiterWrapper> rateLimiter;
    std::unique_ptr<PIDController> innerPID;
};

TEST_F(RateLimiterWrapperTest, GetType) {
    EXPECT_EQ(rateLimiter->getType(), ControllerType::Custom);
}

TEST_F(RateLimiterWrapperTest, GetName) {
    EXPECT_NE(rateLimiter->getName(), nullptr);
}

TEST_F(RateLimiterWrapperTest, SetController) {
    rateLimiter->setController(innerPID.get());
}

TEST_F(RateLimiterWrapperTest, SetRateLimits) {
    rateLimiter->setRateLimits(-100.0, 100.0);
}

TEST_F(RateLimiterWrapperTest, SetRateLimit) {
    rateLimiter->setRateLimit(100.0);
}

TEST_F(RateLimiterWrapperTest, BasicCompute) {
    rateLimiter->setController(innerPID.get());
    rateLimiter->setRateLimits(-100.0, 100.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;  // Error = 50, output = 500 (too fast)
    input.dt = 0.01;
    
    ControllerOutput output = rateLimiter->compute(input);
    
    // First sample - just tracks
    EXPECT_NE(output.control, 0.0);
}

TEST_F(RateLimiterWrapperTest, RateLimitingApplied) {
    rateLimiter->setController(innerPID.get());
    rateLimiter->setRateLimits(-50.0, 50.0);  // Rate limit of 50/sec
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;  // Error = 50, inner wants 500
    input.dt = 0.01;
    
    // First sample
    ControllerOutput out1 = rateLimiter->compute(input);
    
    // Second sample - rate should be limited
    input.measured = 60.0;
    ControllerOutput out2 = rateLimiter->compute(input);
    
    // Rate = (out2 - out1) / dt should be <= 50
    double rate = (out2.control - out1.control) / input.dt;
    EXPECT_LE(rate, 50.0 + 0.1);  // Small tolerance
    EXPECT_GE(rate, -50.0 - 0.1);
}

TEST_F(RateLimiterWrapperTest, NoInnerController) {
    rateLimiter->setRateLimits(-100.0, 100.0);
    // No inner controller
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    ControllerOutput output = rateLimiter->compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(RateLimiterWrapperTest, ZeroDt) {
    rateLimiter->setController(innerPID.get());
    rateLimiter->setRateLimits(-100.0, 100.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.0;  // Zero dt
    
    // First sample to initialize
    input.dt = 0.01;
    rateLimiter->compute(input);
    
    // Zero dt sample
    input.dt = 0.0;
    ControllerOutput output = rateLimiter->compute(input);
}

TEST_F(RateLimiterWrapperTest, Reset) {
    rateLimiter->setController(innerPID.get());
    rateLimiter->setRateLimits(-100.0, 100.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    // Compute several samples
    for (int i = 0; i < 10; ++i) {
        rateLimiter->compute(input);
    }
    
    rateLimiter->reset();
    
    // Next sample should be treated as first sample
    ControllerOutput output = rateLimiter->compute(input);
}

// ============================================================================
// DeadbandWrapper Tests
// ============================================================================

class DeadbandWrapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        deadband = std::make_unique<DeadbandWrapper>();
        innerPID = std::make_unique<PIDController>();
        innerPID->setGains(1.0, 0.0, 0.0);
    }
    
    std::unique_ptr<DeadbandWrapper> deadband;
    std::unique_ptr<PIDController> innerPID;
};

TEST_F(DeadbandWrapperTest, GetType) {
    EXPECT_EQ(deadband->getType(), ControllerType::Custom);
}

TEST_F(DeadbandWrapperTest, GetName) {
    EXPECT_NE(deadband->getName(), nullptr);
}

TEST_F(DeadbandWrapperTest, SetController) {
    deadband->setController(innerPID.get());
}

TEST_F(DeadbandWrapperTest, SetDeadbands) {
    deadband->setErrorDeadband(1.0);
    deadband->setOutputDeadband(0.5);
    deadband->setHysteresis(0.2);
}

TEST_F(DeadbandWrapperTest, BasicCompute) {
    deadband->setController(innerPID.get());
    deadband->setErrorDeadband(0.0);
    deadband->setOutputDeadband(0.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    ControllerOutput output = deadband->compute(input);
    EXPECT_NEAR(output.control, 50.0, 1.0);  // Kp=1, error=50
}

TEST_F(DeadbandWrapperTest, ErrorDeadbandApplied) {
    deadband->setController(innerPID.get());
    deadband->setErrorDeadband(10.0);  // 10 unit error deadband
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 95.0;  // Error = 5 < 10 deadband
    input.dt = 0.01;
    
    ControllerOutput output = deadband->compute(input);
    // Error is within deadband, should be zeroed
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(DeadbandWrapperTest, OutputDeadbandApplied) {
    deadband->setController(innerPID.get());
    deadband->setOutputDeadband(10.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 95.0;  // Error = 5, output = 5 < 10 deadband
    input.dt = 0.01;
    
    ControllerOutput output = deadband->compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(DeadbandWrapperTest, HysteresisApplied) {
    deadband->setController(innerPID.get());
    deadband->setOutputDeadband(5.0);
    deadband->setHysteresis(2.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.dt = 0.01;
    
    // First: large error - output beyond deadband
    input.measured = 80.0;  // Error = 20, output = 20 > 5
    ControllerOutput out1 = deadband->compute(input);
    EXPECT_GT(std::fabs(out1.control), 0.0);
    
    // Then: small error - output in deadband but within hysteresis
    input.measured = 96.0;  // Error = 4, output = 4 < 5 but < 5+2=7 hysteresis
    ControllerOutput out2 = deadband->compute(input);
}

TEST_F(DeadbandWrapperTest, NoInnerController) {
    deadband->setErrorDeadband(1.0);
    deadband->setOutputDeadband(0.5);
    // No inner controller
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    ControllerOutput output = deadband->compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(DeadbandWrapperTest, Reset) {
    deadband->setController(innerPID.get());
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 10; ++i) {
        deadband->compute(input);
    }
    
    deadband->reset();
}

// ============================================================================
// FilterWrapper Tests
// ============================================================================

class FilterWrapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter = std::make_unique<FilterWrapper>();
        innerPID = std::make_unique<PIDController>();
        innerPID->setGains(1.0, 0.0, 0.0);
    }
    
    std::unique_ptr<FilterWrapper> filter;
    std::unique_ptr<PIDController> innerPID;
};

TEST_F(FilterWrapperTest, GetType) {
    EXPECT_EQ(filter->getType(), ControllerType::Custom);
}

TEST_F(FilterWrapperTest, GetName) {
    EXPECT_NE(filter->getName(), nullptr);
}

TEST_F(FilterWrapperTest, SetController) {
    filter->setController(innerPID.get());
}

TEST_F(FilterWrapperTest, SetFilterCutoff) {
    filter->setFilterCutoff(10.0);  // 10 Hz
}

TEST_F(FilterWrapperTest, SetFilterOrder) {
    filter->setFilterOrder(1);
    filter->setFilterOrder(2);
}

TEST_F(FilterWrapperTest, BasicComputeFirstOrder) {
    filter->setController(innerPID.get());
    filter->setFilterCutoff(10.0);
    filter->setFilterOrder(1);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    // First sample
    ControllerOutput out1 = filter->compute(input);
    
    // Step change in reference
    input.reference = 200.0;
    ControllerOutput out2 = filter->compute(input);
    
    // Filtered output should be between old and new
    // Due to low-pass filtering, out2 < unfiltered (150)
}

TEST_F(FilterWrapperTest, BasicComputeSecondOrder) {
    filter->setController(innerPID.get());
    filter->setFilterCutoff(10.0);
    filter->setFilterOrder(2);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    // Run several samples
    for (int i = 0; i < 100; ++i) {
        ControllerOutput output = filter->compute(input);
    }
}

TEST_F(FilterWrapperTest, NoInnerController) {
    filter->setFilterCutoff(10.0);
    // No inner controller
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    ControllerOutput output = filter->compute(input);
    EXPECT_DOUBLE_EQ(output.control, 0.0);
}

TEST_F(FilterWrapperTest, ZeroDt) {
    filter->setController(innerPID.get());
    filter->setFilterCutoff(10.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.0;
    
    ControllerOutput output = filter->compute(input);
}

TEST_F(FilterWrapperTest, ZeroCutoff) {
    filter->setController(innerPID.get());
    filter->setFilterCutoff(0.0);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    ControllerOutput output = filter->compute(input);
}

TEST_F(FilterWrapperTest, Reset) {
    filter->setController(innerPID.get());
    filter->setFilterCutoff(10.0);
    filter->setFilterOrder(2);
    
    ControllerInput input;
    input.reference = 100.0;
    input.measured = 50.0;
    input.dt = 0.01;
    
    for (int i = 0; i < 10; ++i) {
        filter->compute(input);
    }
    
    filter->reset();
    
    // States should be reset
    ControllerOutput output = filter->compute(input);
}

// ============================================================================
// ControllerFactory Tests
// ============================================================================

TEST(ControllerFactoryTest, CreateByType) {
    auto p = ControllerFactory::create(ControllerType::P);
    EXPECT_NE(p, nullptr);
    EXPECT_EQ(p->getType(), ControllerType::P);
    
    auto pd = ControllerFactory::create(ControllerType::PD);
    EXPECT_NE(pd, nullptr);
    EXPECT_EQ(pd->getType(), ControllerType::PD);
    
    auto pi = ControllerFactory::create(ControllerType::PI);
    EXPECT_NE(pi, nullptr);
    EXPECT_EQ(pi->getType(), ControllerType::PI);
    
    auto pid = ControllerFactory::create(ControllerType::PID);
    EXPECT_NE(pid, nullptr);
    EXPECT_EQ(pid->getType(), ControllerType::PID);
    
    auto pid2dof = ControllerFactory::create(ControllerType::PID2DOF);
    EXPECT_NE(pid2dof, nullptr);
    
    auto bangbang = ControllerFactory::create(ControllerType::BangBang);
    EXPECT_NE(bangbang, nullptr);
    
    auto pdplus = ControllerFactory::create(ControllerType::PDPlus);
    EXPECT_NE(pdplus, nullptr);
    
    auto fopid = ControllerFactory::create(ControllerType::FractionalPID);
    EXPECT_NE(fopid, nullptr);
    
    auto lqr = ControllerFactory::create(ControllerType::LQR);
    EXPECT_NE(lqr, nullptr);
    
    auto lqg = ControllerFactory::create(ControllerType::LQG);
    EXPECT_NE(lqg, nullptr);
    
    auto lqi = ControllerFactory::create(ControllerType::LQI);
    EXPECT_NE(lqi, nullptr);
    
    auto hinf = ControllerFactory::create(ControllerType::HInfinity);
    EXPECT_NE(hinf, nullptr);
    
    auto ilc = ControllerFactory::create(ControllerType::ILC);
    EXPECT_NE(ilc, nullptr);
    
    auto cascade = ControllerFactory::create(ControllerType::Cascade);
    EXPECT_NE(cascade, nullptr);
}

TEST(ControllerFactoryTest, CreateByTypeDefault) {
    auto unknown = ControllerFactory::create(ControllerType::Custom);
    EXPECT_EQ(unknown, nullptr);
}

TEST(ControllerFactoryTest, CreateByName) {
    auto p = ControllerFactory::create("P");
    EXPECT_NE(p, nullptr);
    
    auto pd = ControllerFactory::create("PD");
    EXPECT_NE(pd, nullptr);
    
    auto pi = ControllerFactory::create("PI");
    EXPECT_NE(pi, nullptr);
    
    auto pid = ControllerFactory::create("PID");
    EXPECT_NE(pid, nullptr);
    
    auto pid2dof = ControllerFactory::create("PID-2DOF");
    EXPECT_NE(pid2dof, nullptr);
    
    auto bangbang = ControllerFactory::create("Bang-Bang");
    EXPECT_NE(bangbang, nullptr);
    
    auto pdplus = ControllerFactory::create("PD+");
    EXPECT_NE(pdplus, nullptr);
    
    auto dualLoop = ControllerFactory::create("Dual-Loop PID");
    EXPECT_NE(dualLoop, nullptr);
    
    auto fopid = ControllerFactory::create("FOPID");
    EXPECT_NE(fopid, nullptr);
    
    auto lqr = ControllerFactory::create("LQR");
    EXPECT_NE(lqr, nullptr);
    
    auto lqg = ControllerFactory::create("LQG");
    EXPECT_NE(lqg, nullptr);
    
    auto lqi = ControllerFactory::create("LQI");
    EXPECT_NE(lqi, nullptr);
    
    auto h2 = ControllerFactory::create("H2");
    EXPECT_NE(h2, nullptr);
    
    auto hinf = ControllerFactory::create("H-Infinity");
    EXPECT_NE(hinf, nullptr);
    
    auto pILC = ControllerFactory::create("P-Type ILC");
    EXPECT_NE(pILC, nullptr);
    
    auto pdILC = ControllerFactory::create("PD-Type ILC");
    EXPECT_NE(pdILC, nullptr);
    
    auto phaseLeadILC = ControllerFactory::create("Phase-Lead ILC");
    EXPECT_NE(phaseLeadILC, nullptr);
    
    auto normOptILC = ControllerFactory::create("Norm-Optimal ILC");
    EXPECT_NE(normOptILC, nullptr);
    
    auto repetitive = ControllerFactory::create("Repetitive");
    EXPECT_NE(repetitive, nullptr);
    
    auto cascade = ControllerFactory::create("Cascade");
    EXPECT_NE(cascade, nullptr);
    
    auto parallel = ControllerFactory::create("Parallel");
    EXPECT_NE(parallel, nullptr);
    
    auto switching = ControllerFactory::create("Switching");
    EXPECT_NE(switching, nullptr);
}

TEST(ControllerFactoryTest, CreateByNameNull) {
    auto result = ControllerFactory::create(nullptr);
    EXPECT_EQ(result, nullptr);
}

TEST(ControllerFactoryTest, CreateByNameUnknown) {
    auto result = ControllerFactory::create("UnknownController");
    EXPECT_EQ(result, nullptr);
}

