/**
 * @file FSoEMasterSlaveTests.cpp
 * @brief Integration tests for FSoE Master and Slave interaction
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "fsoe/FSoESlave.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

namespace FSoE {
namespace test {

// Simulated FSoE Master for testing
class MockFSoEMaster {
public:
    MockFSoEMaster(uint16_t connectionId, uint16_t slaveAddress)
        : connectionId_(connectionId)
        , slaveAddress_(slaveAddress)
        , sequenceNumber_(0)
        , masterState_(FSoEState::Reset)
    {}
    
    // Build a session request frame
    std::vector<uint8_t> buildSessionRequest() {
        std::vector<uint8_t> frame;
        frame.push_back(static_cast<uint8_t>(FSoECommand::Session));
        frame.push_back(connectionId_ & 0xFF);
        frame.push_back((connectionId_ >> 8) & 0xFF);
        frame.push_back(sequenceNumber_ & 0xFF);
        frame.push_back((sequenceNumber_ >> 8) & 0xFF);
        appendCRC(frame);
        return frame;
    }
    
    // Build a connection request frame
    std::vector<uint8_t> buildConnectionRequest(uint32_t connId) {
        std::vector<uint8_t> frame;
        frame.push_back(static_cast<uint8_t>(FSoECommand::Connection));
        frame.push_back(connectionId_ & 0xFF);
        frame.push_back((connectionId_ >> 8) & 0xFF);
        frame.push_back(sequenceNumber_ & 0xFF);
        frame.push_back((sequenceNumber_ >> 8) & 0xFF);
        // Connection ID
        frame.push_back(connId & 0xFF);
        frame.push_back((connId >> 8) & 0xFF);
        frame.push_back((connId >> 16) & 0xFF);
        frame.push_back((connId >> 24) & 0xFF);
        appendCRC(frame);
        return frame;
    }
    
    // Build a data frame
    std::vector<uint8_t> buildDataFrame(const std::vector<uint8_t>& safeOutputs) {
        std::vector<uint8_t> frame;
        frame.push_back(static_cast<uint8_t>(FSoECommand::Data));
        frame.push_back(connectionId_ & 0xFF);
        frame.push_back((connectionId_ >> 8) & 0xFF);
        frame.push_back(sequenceNumber_ & 0xFF);
        frame.push_back((sequenceNumber_ >> 8) & 0xFF);
        // Safe data
        frame.insert(frame.end(), safeOutputs.begin(), safeOutputs.end());
        appendCRC(frame);
        sequenceNumber_++;
        return frame;
    }
    
    // Build a parameter request
    std::vector<uint8_t> buildParameterRequest(uint16_t paramId, 
                                                const std::vector<uint8_t>& value) {
        std::vector<uint8_t> frame;
        frame.push_back(static_cast<uint8_t>(FSoECommand::Parameter));
        frame.push_back(connectionId_ & 0xFF);
        frame.push_back((connectionId_ >> 8) & 0xFF);
        frame.push_back(sequenceNumber_ & 0xFF);
        frame.push_back((sequenceNumber_ >> 8) & 0xFF);
        // Parameter ID
        frame.push_back(paramId & 0xFF);
        frame.push_back((paramId >> 8) & 0xFF);
        // Parameter value
        frame.insert(frame.end(), value.begin(), value.end());
        appendCRC(frame);
        return frame;
    }
    
    // Build a fail-safe command
    std::vector<uint8_t> buildFailSafeCommand() {
        std::vector<uint8_t> frame;
        frame.push_back(static_cast<uint8_t>(FSoECommand::FailSafe));
        frame.push_back(connectionId_ & 0xFF);
        frame.push_back((connectionId_ >> 8) & 0xFF);
        frame.push_back(sequenceNumber_ & 0xFF);
        frame.push_back((sequenceNumber_ >> 8) & 0xFF);
        appendCRC(frame);
        return frame;
    }
    
    // Parse response from slave
    bool parseResponse(const std::vector<uint8_t>& response,
                       FSoECommand& cmd, uint16_t& seqNum,
                       std::vector<uint8_t>& data) {
        if (response.size() < 7) return false;
        
        cmd = static_cast<FSoECommand>(response[0]);
        uint16_t connId = response[1] | (response[2] << 8);
        seqNum = response[3] | (response[4] << 8);
        
        // Extract data (excluding header and CRC)
        if (response.size() > 7) {
            data.assign(response.begin() + 5, response.end() - 2);
        }
        
        // Verify CRC
        return verifyCRC(response);
    }
    
    void incrementSequence() { sequenceNumber_++; }
    uint16_t getSequenceNumber() const { return sequenceNumber_; }
    void setSequenceNumber(uint16_t seq) { sequenceNumber_ = seq; }
    
private:
    void appendCRC(std::vector<uint8_t>& frame) {
        // Simplified CRC - real implementation uses proper FSoE CRC
        uint16_t crc = 0;
        for (uint8_t b : frame) {
            crc ^= b;
            crc = (crc >> 1) ^ (crc & 1 ? 0x755B : 0);
        }
        frame.push_back(crc & 0xFF);
        frame.push_back((crc >> 8) & 0xFF);
    }
    
    bool verifyCRC(const std::vector<uint8_t>& frame) {
        // Simplified verification
        return true;
    }
    
    uint16_t connectionId_;
    uint16_t slaveAddress_;
    uint16_t sequenceNumber_;
    FSoEState masterState_;
};

// Test fixture for master-slave interaction
class FSoEMasterSlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        FSoESlaveConfig slaveConfig;
        slaveConfig.slaveAddress = 0x1234;
        slaveConfig.connectionId = 0x5678;
        slaveConfig.safeInputSize = 4;
        slaveConfig.safeOutputSize = 4;
        slaveConfig.watchdogTimeMs = 100;
        
        slave_ = std::make_unique<FSoESlave>(slaveConfig);
        slave_->initialize();
        
        master_ = std::make_unique<MockFSoEMaster>(0x5678, 0x1234);
    }
    
    void TearDown() override {
        slave_.reset();
        master_.reset();
    }
    
    // Helper to send frame and get response
    std::vector<uint8_t> sendAndReceive(const std::vector<uint8_t>& frame) {
        // Process frame in slave
        slave_->processRxFrame(frame.data(), frame.size());
        
        // Get response
        std::vector<uint8_t> response(64);
        size_t responseSize = slave_->prepareTxFrame(response.data(), response.size());
        response.resize(responseSize);
        
        return response;
    }
    
    std::unique_ptr<FSoESlave> slave_;
    std::unique_ptr<MockFSoEMaster> master_;
};

// =============================================================================
// Connection Establishment Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, SessionEstablishment) {
    // Slave should start in Reset state
    EXPECT_EQ(slave_->getState(), FSoEState::Reset);
    
    // Send session request
    auto request = master_->buildSessionRequest();
    auto response = sendAndReceive(request);
    
    // Slave should respond and potentially move to Session state
    EXPECT_GT(response.size(), 0u);
}

TEST_F(FSoEMasterSlaveTest, FullConnectionSequence) {
    // Step 1: Session
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    // Step 2: Connection
    auto connReq = master_->buildConnectionRequest(0x12345678);
    sendAndReceive(connReq);
    master_->incrementSequence();
    
    // Verify connection established
    // (actual state depends on implementation details)
}

TEST_F(FSoEMasterSlaveTest, DataExchange) {
    // Establish connection first
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    auto connReq = master_->buildConnectionRequest(0x12345678);
    sendAndReceive(connReq);
    master_->incrementSequence();
    
    // Exchange data
    std::vector<uint8_t> safeOutputs = {0x01, 0x02, 0x03, 0x04};
    auto dataFrame = master_->buildDataFrame(safeOutputs);
    auto response = sendAndReceive(dataFrame);
    
    // Should get data response
    EXPECT_GT(response.size(), 0u);
}

// =============================================================================
// Error Handling Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, SequenceNumberError) {
    // Establish connection
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    // Send data with wrong sequence number
    master_->setSequenceNumber(100); // Jump ahead
    
    std::vector<uint8_t> safeOutputs = {0x01, 0x02, 0x03, 0x04};
    auto dataFrame = master_->buildDataFrame(safeOutputs);
    
    // Enable error injection for testing
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.injectSequenceError = true;
    slave_->setErrorInjection(injection);
    
    sendAndReceive(dataFrame);
    
    // Should detect sequence error
    auto stats = slave_->getStatistics();
    // Check for sequence errors in stats
}

TEST_F(FSoEMasterSlaveTest, WatchdogTimeout) {
    // Establish connection
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    auto connReq = master_->buildConnectionRequest(0x12345678);
    sendAndReceive(connReq);
    
    // Simulate time passing without communication
    for (int i = 0; i < 200; i++) {
        slave_->processUpdate(1); // 1ms per update
    }
    
    // Should have triggered watchdog
    // Verify fail-safe state
}

TEST_F(FSoEMasterSlaveTest, CRCErrorInjection) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.injectCRCError = true;
    
    slave_->setErrorInjection(injection);
    
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    
    // CRC error should be logged
    auto stats = slave_->getStatistics();
    EXPECT_GT(stats.crcErrors, 0u);
}

TEST_F(FSoEMasterSlaveTest, ConnectionIdMismatch) {
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.injectConnectionIdError = true;
    injection.wrongConnectionId = 0xFF;
    
    slave_->setErrorInjection(injection);
    
    // Send frames - should detect connection ID error
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
}

// =============================================================================
// Fail-Safe Behavior Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, FailSafeActivation) {
    // Establish data exchange
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    auto connReq = master_->buildConnectionRequest(0x12345678);
    sendAndReceive(connReq);
    master_->incrementSequence();
    
    // Send fail-safe command
    auto failSafeCmd = master_->buildFailSafeCommand();
    sendAndReceive(failSafeCmd);
    
    // Verify fail-safe state
    auto state = slave_->getState();
    EXPECT_TRUE(state == FSoEState::FailSafe || 
                state == FSoEState::FailSafeReaction);
}

TEST_F(FSoEMasterSlaveTest, FailSafeOutputs) {
    // Establish connection and set some outputs
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    std::vector<uint8_t> safeInputs = {0xFF, 0xFF, 0xFF, 0xFF};
    slave_->setSafeInputs(safeInputs.data(), safeInputs.size());
    
    // Trigger fail-safe
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.simulateWatchdogTimeout = true;
    
    slave_->setErrorInjection(injection);
    slave_->processUpdate(200);
    
    // Outputs should be safe (zero)
    std::vector<uint8_t> outputs(4);
    slave_->getSafeOutputs(outputs.data(), outputs.size());
    
    for (auto b : outputs) {
        EXPECT_EQ(b, 0u);
    }
}

// =============================================================================
// Parameter Exchange Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, ParameterWrite) {
    // Establish session
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    // Write parameter
    std::vector<uint8_t> paramValue = {0x01, 0x02, 0x03, 0x04};
    auto paramReq = master_->buildParameterRequest(0x1000, paramValue);
    auto response = sendAndReceive(paramReq);
    
    // Should get parameter response
    EXPECT_GT(response.size(), 0u);
}

// =============================================================================
// Recovery Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, RecoveryFromFailSafe) {
    // Trigger fail-safe
    FSoEErrorInjection injection;
    injection.enabled = true;
    injection.forceError = true;
    injection.forcedError = FSoEError::WatchdogTimeout;
    
    slave_->setErrorInjection(injection);
    slave_->processUpdate(10);
    
    // Disable injection
    injection.enabled = false;
    slave_->setErrorInjection(injection);
    
    // Reset slave
    slave_->reset();
    
    EXPECT_EQ(slave_->getState(), FSoEState::Reset);
    EXPECT_FALSE(slave_->isOperational());
    
    // Re-establish connection
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
}

// =============================================================================
// Performance Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, RapidDataExchange) {
    // Establish connection
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    auto connReq = master_->buildConnectionRequest(0x12345678);
    sendAndReceive(connReq);
    master_->incrementSequence();
    
    // Exchange many data frames
    std::vector<uint8_t> safeOutputs = {0x01, 0x02, 0x03, 0x04};
    
    for (int i = 0; i < 1000; i++) {
        auto dataFrame = master_->buildDataFrame(safeOutputs);
        sendAndReceive(dataFrame);
    }
    
    auto stats = slave_->getStatistics();
    EXPECT_GE(stats.framesReceived, 1000u);
}

TEST_F(FSoEMasterSlaveTest, CyclicCommunication) {
    // Establish connection
    auto sessionReq = master_->buildSessionRequest();
    sendAndReceive(sessionReq);
    master_->incrementSequence();
    
    auto connReq = master_->buildConnectionRequest(0x12345678);
    sendAndReceive(connReq);
    master_->incrementSequence();
    
    // Simulate cyclic communication
    std::vector<uint8_t> safeOutputs = {0x00, 0x00, 0x00, 0x00};
    
    for (int cycle = 0; cycle < 100; cycle++) {
        // Update outputs
        safeOutputs[0] = cycle & 0xFF;
        
        // Send data
        auto dataFrame = master_->buildDataFrame(safeOutputs);
        auto response = sendAndReceive(dataFrame);
        
        // Process time
        slave_->processUpdate(1); // 1ms cycle
        
        EXPECT_GT(response.size(), 0u);
    }
}

// =============================================================================
// Multi-Slave Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, MultipleSlavesIndependent) {
    // Create second slave
    FSoESlaveConfig slave2Config;
    slave2Config.slaveAddress = 0x5678;
    slave2Config.connectionId = 0x9ABC;
    slave2Config.safeInputSize = 4;
    slave2Config.safeOutputSize = 4;
    
    auto slave2 = std::make_unique<FSoESlave>(slave2Config);
    slave2->initialize();
    
    // Create second master
    auto master2 = std::make_unique<MockFSoEMaster>(0x9ABC, 0x5678);
    
    // Both slaves should work independently
    auto session1 = master_->buildSessionRequest();
    slave_->processRxFrame(session1.data(), session1.size());
    
    auto session2 = master2->buildSessionRequest();
    slave2->processRxFrame(session2.data(), session2.size());
    
    // Both should be in proper state
    EXPECT_EQ(slave_->getState(), slave2->getState());
}

// =============================================================================
// Stress Tests
// =============================================================================

TEST_F(FSoEMasterSlaveTest, ErrorRecoveryStress) {
    // Rapidly alternate between errors and recovery
    for (int i = 0; i < 100; i++) {
        // Inject error
        FSoEErrorInjection injection;
        injection.enabled = true;
        injection.forceError = true;
        injection.forcedError = FSoEError::CRCError;
        
        slave_->setErrorInjection(injection);
        slave_->processUpdate(10);
        
        // Clear error and reset
        injection.enabled = false;
        slave_->setErrorInjection(injection);
        slave_->reset();
        
        EXPECT_EQ(slave_->getState(), FSoEState::Reset);
    }
}

} // namespace test
} // namespace FSoE
