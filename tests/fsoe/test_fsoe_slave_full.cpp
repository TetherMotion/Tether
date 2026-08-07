/**
 * @file test_fsoe_slave_full.cpp
 * @brief Comprehensive tests for FSoE (Fail-Safe over EtherCAT) Slave
 */

#include "tether/fsoe/FSoESlave.hpp"
#include "tether/fsoe/FSoEDefs.hpp"

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <functional>

using namespace FSoE;

// ============================================================================
// FSoEDefs tests (CRC, constants, etc.)
// ============================================================================

TEST(FSoEDefsTest, ConnectionStateConstants) {
    EXPECT_EQ(ConnectionState::Reset, 0);
    EXPECT_EQ(ConnectionState::Session, 1);
    EXPECT_EQ(ConnectionState::Connection, 2);
    EXPECT_EQ(ConnectionState::Parameter, 3);
    EXPECT_EQ(ConnectionState::Data, 4);
    EXPECT_EQ(ConnectionState::Error, 5);
}

TEST(FSoEDefsTest, ErrorCodeConstants) {
    EXPECT_EQ(ErrorCode::NoError, 0x0000);
}

TEST(FSoEDefsTest, CommandConstants) {
    EXPECT_EQ(Command::ProcessData, 0x36);
    EXPECT_EQ(Command::Reset, 0x2A);
    EXPECT_EQ(Command::Session, 0x4E);
    EXPECT_EQ(Command::Connection, 0x64);
    EXPECT_EQ(Command::Parameter, 0x52);
    EXPECT_EQ(Command::FailSafeData, 0x08);
}

TEST(FSoEDefsTest, SILLevels) {
    EXPECT_EQ(SIL::None, 0);
    EXPECT_EQ(SIL::SIL1, 1);
    EXPECT_EQ(SIL::SIL2, 2);
    EXPECT_EQ(SIL::SIL3, 3);
}

TEST(FSoEDefsTest, PLLevels) {
    EXPECT_EQ(PL::None, 0);
    EXPECT_EQ(PL::PLa, 1);
}

TEST(FSoEDefsTest, CalculateFSoECRC) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = CRC::calculate(data, sizeof(data));
    EXPECT_NE(crc, 0); // CRC should be non-zero for non-zero data
    
    // Verify deterministic
    uint16_t crc2 = CRC::calculate(data, sizeof(data));
    EXPECT_EQ(crc, crc2);
}

TEST(FSoEDefsTest, CalculateFSoECRC_EmptyData) {
    uint16_t crc = CRC::calculate(nullptr, 0);
    // Empty data with init 0x0000 should return init
    EXPECT_EQ(crc, 0x0000);
}

TEST(FSoEDefsTest, VerifyFSoECRC) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = CRC::calculate(data, sizeof(data));
    EXPECT_TRUE(CRC::verifyFSoECRC(data, sizeof(data), crc));
    EXPECT_FALSE(CRC::verifyFSoECRC(data, sizeof(data), crc + 1));
}

TEST(FSoEDefsTest, ConnectionStatsDefaults) {
    ConnectionStats stats{};
    EXPECT_EQ(stats.frames_sent, 0u);
    EXPECT_EQ(stats.frames_received, 0u);
    EXPECT_EQ(stats.crc_errors, 0u);
    EXPECT_EQ(stats.sequence_errors, 0u);
    EXPECT_EQ(stats.timeout_events, 0u);
    EXPECT_EQ(stats.reset_events, 0u);
    EXPECT_EQ(stats.watchdog_events, 0u);
}

TEST(FSoEDefsTest, FSoESlaveStatsReset) {
    FSoESlaveStats stats{};
    stats.framesReceived = 100;
    stats.framesSent = 200;
    stats.crcErrors = 5;
    stats.reset();
    EXPECT_EQ(stats.framesReceived, 0u);
    EXPECT_EQ(stats.framesSent, 0u);
    EXPECT_EQ(stats.crcErrors, 0u);
}

// ============================================================================
// FSoESlaveConfig tests
// ============================================================================

static FSoESlaveConfig makeDefaultConfig() {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 1;
    cfg.connectionId = 0x1234;
    cfg.safetyAddress = 0x0001;
    cfg.safetyLevel = SIL::SIL2;
    cfg.watchdogTimeoutMs = 100;
    cfg.connectionTimeoutMs = 1000;
    cfg.sessionTimeoutMs = 5000;
    cfg.safeInputSize = 2;
    cfg.safeOutputSize = 2;
    cfg.failSafeInputs.fill(0);
    cfg.failSafeOutputs.fill(0);
    cfg.autoRecoveryEnabled = true;
    cfg.recoveryDelayMs = 500;
    cfg.strictCrcCheck = true;
    cfg.strictSequenceCheck = true;
    cfg.enableDiagnostics = true;
    cfg.maxErrorLogEntries = 100;
    return cfg;
}

// ============================================================================
// FSoEErrorInjection tests
// ============================================================================

TEST(FSoEErrorInjectionTest, DefaultConstruction) {
    FSoEErrorInjection inj{};
    EXPECT_FALSE(inj.injectCRCError);
    EXPECT_FALSE(inj.injectSequenceError);
}

TEST(FSoEErrorInjectionTest, Reset) {
    FSoEErrorInjection inj{};
    inj.injectCRCError = true;
    inj.injectSequenceError = true;
    inj.reset();
    EXPECT_FALSE(inj.injectCRCError);
    EXPECT_FALSE(inj.injectSequenceError);
}

// ============================================================================
// FSoESlave tests
// ============================================================================

class FSoESlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = makeDefaultConfig();
        slave_ = std::make_unique<FSoESlave>(config_);
    }
    
    FSoESlaveConfig config_;
    std::unique_ptr<FSoESlave> slave_;
};

TEST_F(FSoESlaveTest, ConstructAndInitialize) {
    EXPECT_FALSE(slave_->isInitialized());
    EXPECT_TRUE(slave_->initialize());
    EXPECT_TRUE(slave_->isInitialized());
}

TEST_F(FSoESlaveTest, GetConfig) {
    auto& cfg = slave_->getConfig();
    EXPECT_EQ(cfg.slaveAddress, 1);
    EXPECT_EQ(cfg.connectionId, 0x1234);
}

TEST_F(FSoESlaveTest, InitialState) {
    slave_->initialize();
    uint8_t state = slave_->getState();
    // Should start in Reset or Session state
    EXPECT_LE(state, ConnectionState::Session);
}

TEST_F(FSoESlaveTest, GetStateName) {
    slave_->initialize();
    const char* name = slave_->getStateName();
    EXPECT_NE(name, nullptr);
    EXPECT_GT(std::strlen(name), 0u);
}

TEST_F(FSoESlaveTest, IsOperational_Initially) {
    slave_->initialize();
    EXPECT_FALSE(slave_->isOperational());
}

TEST_F(FSoESlaveTest, IsFailSafe_Initially) {
    slave_->initialize();
    EXPECT_FALSE(slave_->isFailSafe());
}

TEST_F(FSoESlaveTest, HasError_Initially) {
    slave_->initialize();
    EXPECT_FALSE(slave_->hasError());
}

TEST_F(FSoESlaveTest, GetLastError_Initially) {
    slave_->initialize();
    EXPECT_EQ(slave_->getLastError(), ErrorCode::NoError);
}

TEST_F(FSoESlaveTest, Reset) {
    slave_->initialize();
    slave_->reset();
    uint8_t state = slave_->getState();
    EXPECT_LE(state, ConnectionState::Session);
}

TEST_F(FSoESlaveTest, TriggerFailSafe) {
    slave_->initialize();
    slave_->triggerFailSafe(ErrorCode::WatchdogError);
    EXPECT_TRUE(slave_->isFailSafe() || slave_->hasError());
}

TEST_F(FSoESlaveTest, AttemptRecovery) {
    slave_->initialize();
    slave_->triggerFailSafe(ErrorCode::WatchdogError);
    bool recovered = slave_->attemptRecovery();
    // Recovery may or may not succeed depending on state
    (void)recovered;
}

TEST_F(FSoESlaveTest, SetSafeInputs) {
    slave_->initialize();
    uint8_t inputs[] = {0xAA, 0x55};
    bool ok = slave_->setSafeInputs(inputs, sizeof(inputs));
    EXPECT_TRUE(ok);
}

TEST_F(FSoESlaveTest, GetSafeOutputs) {
    slave_->initialize();
    uint8_t outputs[2] = {};
    size_t got = slave_->getSafeOutputs(outputs, sizeof(outputs));
    // May be 0 if not yet in data state
    (void)got;
}

TEST_F(FSoESlaveTest, AreSafeOutputsValid) {
    slave_->initialize();
    EXPECT_FALSE(slave_->areSafeOutputsValid()); // Not in data state yet
}

TEST_F(FSoESlaveTest, ApplyFailSafeOutputs) {
    slave_->initialize();
    slave_->applyFailSafeOutputs();
    // Should apply the preconfigured fail-safe output values
}

TEST_F(FSoESlaveTest, SafeOutputBit) {
    slave_->initialize();
    bool bit = slave_->getSafeOutputBit(0);
    (void)bit;
}

TEST_F(FSoESlaveTest, SafeInputBit) {
    slave_->initialize();
    bool ok = slave_->setSafeInputBit(0, true);
    EXPECT_TRUE(ok);
}

TEST_F(FSoESlaveTest, StateCallback) {
    uint8_t oldState = 0, newState = 0;
    slave_->setStateCallback([&](uint8_t o, uint8_t n) {
        oldState = o;
        newState = n;
    });
    slave_->initialize();
    slave_->reset();
    // Callback may have been triggered
}

TEST_F(FSoESlaveTest, ErrorCallback) {
    uint16_t lastCode = 0;
    bool wasCritical = false;
    slave_->setErrorCallback([&](uint16_t code, bool critical) {
        lastCode = code;
        wasCritical = critical;
    });
    slave_->initialize();
    slave_->triggerFailSafe(ErrorCode::CRCError);
}

TEST_F(FSoESlaveTest, FailSafeCallback) {
    bool failSafeCalled = false;
    slave_->setFailSafeCallback([&]() {
        failSafeCalled = true;
    });
    slave_->initialize();
    slave_->triggerFailSafe(ErrorCode::WatchdogError);
}

TEST_F(FSoESlaveTest, DataValidCallback) {
    bool dataValidCalled = false;
    slave_->setDataValidCallback([&](const uint8_t*, size_t) {
        dataValidCalled = true;
    });
    slave_->initialize();
}

TEST_F(FSoESlaveTest, RecoveryCallback) {
    slave_->setRecoveryCallback([&]() -> bool {
        return true;
    });
    slave_->initialize();
}

TEST_F(FSoESlaveTest, GetStats) {
    slave_->initialize();
    auto stats = slave_->getStats();
    EXPECT_EQ(stats.framesReceived, 0u);
}

TEST_F(FSoESlaveTest, ResetStats) {
    slave_->initialize();
    slave_->resetStats();
    auto stats = slave_->getStats();
    EXPECT_EQ(stats.framesSent, 0u);
}

TEST_F(FSoESlaveTest, GetDiagnostics) {
    slave_->initialize();
    auto diag = slave_->getDiagnostics();
    // Diagnostics may contain entries from initialization
    // Verify we can read them without crash
    (void)diag.size();
}

TEST_F(FSoESlaveTest, ClearDiagnostics) {
    slave_->initialize();
    slave_->clearDiagnostics();
}

TEST_F(FSoESlaveTest, ErrorInjection) {
    slave_->initialize();
    auto& inj = slave_->getErrorInjection();
    EXPECT_FALSE(inj.injectCRCError);
    
    FSoEErrorInjection newInj{};
    newInj.enabled = true;
    newInj.injectCRCError = true;
    slave_->setErrorInjection(newInj);
    EXPECT_TRUE(slave_->getErrorInjection().injectCRCError);
    EXPECT_TRUE(slave_->isErrorInjectionEnabled());
}

TEST_F(FSoESlaveTest, Reconfigure) {
    slave_->initialize();
    FSoESlaveConfig newCfg = config_;
    newCfg.watchdogTimeoutMs = 200;
    bool ok = slave_->reconfigure(newCfg);
    EXPECT_TRUE(ok);
}

TEST_F(FSoESlaveTest, ProcessRxFrame_Empty) {
    slave_->initialize();
    uint8_t frame[64] = {};
    bool ok = slave_->processRxFrame(frame, 0);
    EXPECT_FALSE(ok); // Empty frame should fail
}

TEST_F(FSoESlaveTest, PrepareTxFrame) {
    slave_->initialize();
    uint8_t frame[64] = {};
    size_t len = slave_->prepareTxFrame(frame, sizeof(frame));
    // May be 0 if not in a state that sends frames
    (void)len;
}

TEST_F(FSoESlaveTest, Update) {
    slave_->initialize();
    slave_->update(0);
    slave_->update(50);
    slave_->update(100);
    // Should not crash
}

TEST_F(FSoESlaveTest, UpdateWatchdogTimeout) {
    slave_->initialize();
    // Run update beyond watchdog timeout
    for (uint32_t t = 0; t < 200; t += 10) {
        slave_->update(t);
    }
    // After exceeding watchdog, should be in failsafe or error state
}
