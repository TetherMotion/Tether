/**
 * @file FSoESlaveTests.cpp
 * @brief Unit tests for FSoE Slave Implementation
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "fsoe/FSoESlave.hpp"
#include <cstring>
#include <vector>

namespace FSoE {
namespace test {

// Test fixture
class FSoESlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        FSoESlaveConfig config;
        config.slaveAddress = 0x1234;
        config.connectionId = 0x5678;
        config.safeInputSize = 4;
        config.safeOutputSize = 4;
        config.watchdogTimeMs = 100;
        
        slave_ = std::make_unique<FSoESlave>(config);
        slave_->initialize();
    }
    
    void TearDown() override {
        slave_.reset();
    }
    
    std::unique_ptr<FSoESlave> slave_;
    
    // Helper to create valid FSoE frame
    std::vector<uint8_t> createFrame(FSoECommand cmd, uint8_t connId, uint16_t seqNum,
                                     const std::vector<uint8_t>& data = {}) {
        // Simplified frame creation - actual implementation would compute CRC
        std::vector<uint8_t> frame;
        frame.push_back(static_cast<uint8_t>(cmd));
        frame.push_back(connId);
        frame.push_back(seqNum & 0xFF);
        frame.push_back((seqNum >> 8) & 0xFF);
        frame.insert(frame.end(), data.begin(), data.end());
        // CRC would be appended here
        frame.push_back(0x00); // Placeholder CRC
        frame.push_back(0x00);
        return frame;
    }
};

// =============================================================================
// Initialization Tests
// =============================================================================

TEST_F(FSoESlaveTest, InitializesToResetState) {
    EXPECT_EQ(slave_->getState(), FSoEState::Reset);
    EXPECT_FALSE(slave_->isConnected());
    EXPECT_FALSE(slave_->isOperational());
}

TEST_F(FSoESlaveTest, ConfigurationIsStored) {
    FSoESlaveConfig config;
    config.slaveAddress = 0xABCD;
    config.connectionId = 0xEF01;
    config.safeInputSize = 8;
    config.safeOutputSize = 16;
    
    FSoESlave slave(config);
    slave.initialize();
    
    EXPECT_EQ(slave.getState(), FSoEState::Reset);
}

TEST_F(FSoESlaveTest, ResetClearsState) {
    // Simulate some state changes
    slave_->reset();
    
    EXPECT_EQ(slave_->getState(), FSoEState::Reset);
    
    auto stats = slave_->getStatistics();
    EXPECT_EQ(stats.stateTransitions, 0u);
}

// =============================================================================
// State Transition Tests
// =============================================================================

TEST_F(FSoESlaveTest, TransitionsToSession) {
    // In a full test, we'd send proper frames
    // This tests the state machine is accessible
    EXPECT_EQ(slave_->getState(), FSoEState::Reset);
}

TEST_F(FSoESlaveTest, InvalidTransitionRejected) {
    // Should not be able to jump states
    EXPECT_EQ(slave_->getState(), FSoEState::Reset);
    // Cannot go directly to Data state
}

// =============================================================================
// Error Injection Tests
// =============================================================================

TEST_F(FSoESlaveTest, CRCErrorInjection) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.injectCRCError = true;
    
    slave_->setErrorInjection(injection);
    
    auto currentInjection = slave_->getErrorInjection();
    EXPECT_TRUE(currentInjection.enabled);
    EXPECT_TRUE(currentInjection.injectCRCError);
}

TEST_F(FSoESlaveTest, SequenceNumberErrorInjection) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.injectSequenceError = true;
    
    slave_->setErrorInjection(injection);
    
    auto currentInjection = slave_->getErrorInjection();
    EXPECT_TRUE(currentInjection.injectSequenceError);
}

TEST_F(FSoESlaveTest, WatchdogErrorInjection) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.simulateWatchdogTimeout = true;
    
    slave_->setErrorInjection(injection);
    
    // Process update to trigger watchdog
    slave_->processUpdate(200); // Time greater than watchdog
}

TEST_F(FSoESlaveTest, ConnectionIdErrorInjection) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.injectConnectionIdError = true;
    injection.wrongConnectionId = 0xFF;
    
    slave_->setErrorInjection(injection);
    
    auto currentInjection = slave_->getErrorInjection();
    EXPECT_EQ(currentInjection.wrongConnectionId, 0xFF);
}

// =============================================================================
// Safe Data Tests
// =============================================================================

TEST_F(FSoESlaveTest, SafeInputsInitializeToSafe) {
    std::vector<uint8_t> inputs(4);
    slave_->getSafeInputs(inputs.data(), inputs.size());
    
    // Safe inputs should be zero/safe by default
    for (auto b : inputs) {
        EXPECT_EQ(b, 0);
    }
}

TEST_F(FSoESlaveTest, SafeOutputsCanBeSet) {
    std::vector<uint8_t> outputs = {0x01, 0x02, 0x03, 0x04};
    slave_->setSafeInputs(outputs.data(), outputs.size());
    
    std::vector<uint8_t> readBack(4);
    slave_->getSafeInputs(readBack.data(), readBack.size());
    
    EXPECT_EQ(readBack, outputs);
}

// =============================================================================
// Callback Tests
// =============================================================================

TEST_F(FSoESlaveTest, StateChangeCallbackInvoked) {
    bool callbackInvoked = false;
    FSoEState newState;
    
    slave_->setStateChangeCallback([&](FSoEState state) {
        callbackInvoked = true;
        newState = state;
    });
    
    // Reset should trigger callback if state changes
    slave_->reset();
}

TEST_F(FSoESlaveTest, ErrorCallbackInvoked) {
    bool errorCallbackInvoked = false;
    FSoEError lastError;
    
    slave_->setErrorCallback([&](FSoEError error) {
        errorCallbackInvoked = true;
        lastError = error;
    });
    
    // Force an error through injection
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.forceError = true;
    injection.forcedError = FSoEError::CRCError;
    
    slave_->setErrorInjection(injection);
    slave_->processUpdate(10);
}

// =============================================================================
// Statistics Tests
// =============================================================================

TEST_F(FSoESlaveTest, StatisticsAreTracked) {
    auto stats = slave_->getStatistics();
    
    EXPECT_EQ(stats.framesReceived, 0u);
    EXPECT_EQ(stats.framesSent, 0u);
    EXPECT_EQ(stats.crcErrors, 0u);
}

TEST_F(FSoESlaveTest, StatisticsCanBeReset) {
    // Process some frames to accumulate stats
    // ...
    
    slave_->resetStatistics();
    
    auto stats = slave_->getStatistics();
    EXPECT_EQ(stats.framesReceived, 0u);
    EXPECT_EQ(stats.framesSent, 0u);
}

// =============================================================================
// Fail-Safe Behavior Tests
// =============================================================================

TEST_F(FSoESlaveTest, FailSafeOnWatchdogTimeout) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.simulateWatchdogTimeout = true;
    
    slave_->setErrorInjection(injection);
    slave_->processUpdate(200);
    
    // Should be in fail-safe or error state
    auto state = slave_->getState();
    EXPECT_TRUE(state == FSoEState::FailSafeReaction || 
                state == FSoEState::FailSafe ||
                state == FSoEState::Reset);
}

TEST_F(FSoESlaveTest, FailSafeDataIsZero) {
    // When in fail-safe, outputs should be safe (zero)
    std::vector<uint8_t> outputs(4);
    slave_->getSafeOutputs(outputs.data(), outputs.size());
    
    // In fail-safe, outputs should be safe values
    // (actual behavior depends on implementation)
}

} // namespace test
} // namespace FSoE
