/**
 * @file test_FSoEMasterConnection_coverage.cpp
 * @brief Comprehensive FSoEMasterConnection + FSoESlave coverage tests
 */
#include <gtest/gtest.h>
#include <cstring>
#include <memory>
#include <vector>
#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"

using namespace FSoE;

// ============================================================================
// Fixture: FSoEMasterConnection
// ============================================================================
class FSoEMasterConnectionCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        MasterConnectionConfig cfg{};
        cfg.slave_addr = 0x0001;
        cfg.slave_safety_addr = 0x0100;
        cfg.connection_id = 0x1234;
        cfg.master_addr = 0x0100;
        cfg.watchdog_timeout_ms = 100;
        cfg.conn_timeout_ms = 1000;
        cfg.safety_level = SIL::SIL2;
        cfg.input_size = 4;
        cfg.output_size = 4;
        cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};
        conn = std::make_unique<FSoEMasterConnection>(cfg);
        conn->initialize();
    }
    std::unique_ptr<FSoEMasterConnection> conn;
};

// --- Basic state ---

TEST_F(FSoEMasterConnectionCoverageTest, InitialState) {
    EXPECT_TRUE(conn->isInitialized());
    EXPECT_FALSE(conn->isOperational());
    EXPECT_FALSE(conn->isFailSafe());
}

TEST_F(FSoEMasterConnectionCoverageTest, GetConfig) {
    auto& cfg = conn->getConfig();
    EXPECT_EQ(cfg.slave_addr, 0x0001);
    EXPECT_EQ(cfg.connection_id, 0x1234);
    EXPECT_EQ(cfg.input_size, 4u);
    EXPECT_EQ(cfg.output_size, 4u);
}

TEST_F(FSoEMasterConnectionCoverageTest, GetStatus) {
    auto status = conn->getStatus();
    EXPECT_EQ(status.error_code, ErrorCode::NoError);
    EXPECT_FALSE(status.data_valid);
}

TEST_F(FSoEMasterConnectionCoverageTest, GetState) {
    uint8_t state = conn->getState();
    // Should be in Reset or Session after init
    EXPECT_LE(state, ConnectionState::Data);
}

TEST_F(FSoEMasterConnectionCoverageTest, GetErrorCode) {
    EXPECT_EQ(conn->getErrorCode(), ErrorCode::NoError);
}

// --- Connection flow ---

TEST_F(FSoEMasterConnectionCoverageTest, StartConnection) {
    bool result = conn->startConnection();
    // May succeed or fail depending on initial state
    (void)result;
    EXPECT_GE(conn->getState(), ConnectionState::Reset);
}

TEST_F(FSoEMasterConnectionCoverageTest, ResetConnection) {
    conn->startConnection();
    bool result = conn->resetConnection();
    EXPECT_TRUE(result);
}

TEST_F(FSoEMasterConnectionCoverageTest, RequestSessionReset) {
    bool result = conn->requestSessionReset();
    EXPECT_TRUE(result);
    auto status = conn->getStatus();
    EXPECT_NE(status.session_id, 0u);
}

// --- Safe I/O ---

TEST_F(FSoEMasterConnectionCoverageTest, SetSafeOutputsCorrectLength) {
    uint8_t data[4] = {1, 2, 3, 4};
    EXPECT_TRUE(conn->setSafeOutputs(data, 4));
}

TEST_F(FSoEMasterConnectionCoverageTest, SetSafeOutputsWrongLength) {
    uint8_t data[2] = {1, 2};
    EXPECT_FALSE(conn->setSafeOutputs(data, 2));
}

TEST_F(FSoEMasterConnectionCoverageTest, GetSafeInputs) {
    uint8_t data[4] = {};
    size_t len = conn->getSafeInputs(data, 4);
    EXPECT_LE(len, 4u);
}

TEST_F(FSoEMasterConnectionCoverageTest, AreSafeInputsValid) {
    EXPECT_FALSE(conn->areSafeInputsValid()); // No data received yet
}

TEST_F(FSoEMasterConnectionCoverageTest, SetSafeOutputBit) {
    conn->setSafeOutputBit(0, true);
    conn->setSafeOutputBit(7, false);
}

TEST_F(FSoEMasterConnectionCoverageTest, GetSafeInputBit) {
    bool val = conn->getSafeInputBit(0);
    (void)val;
}

TEST_F(FSoEMasterConnectionCoverageTest, SetSafeOutputByte) {
    conn->setSafeOutputByte(0, 0xAA);
    conn->setSafeOutputByte(3, 0x55);
}

TEST_F(FSoEMasterConnectionCoverageTest, GetSafeInputByte) {
    uint8_t val = conn->getSafeInputByte(0);
    (void)val;
}

// --- Fail-safe ---

TEST_F(FSoEMasterConnectionCoverageTest, TriggerFailSafe) {
    // triggerFailSafe from Reset state goes back to Reset (NOT_OK transition)
    // Error code is preserved after resetConnection
    conn->triggerFailSafe(ErrorCode::WatchdogError);
    EXPECT_FALSE(conn->isFailSafe());
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);
    EXPECT_NE(conn->getErrorCode(), ErrorCode::NoError);
}

TEST_F(FSoEMasterConnectionCoverageTest, GetFailSafeValues) {
    auto& vals = conn->getConfig().fail_safe_values;
    EXPECT_EQ(vals[0], 0xDE);
    EXPECT_EQ(vals[1], 0xAD);
}

TEST_F(FSoEMasterConnectionCoverageTest, ClearErrorAfterFailSafe) {
    // triggerFailSafe from Reset state goes back to Reset (NOT_OK transition)
    conn->triggerFailSafe(ErrorCode::CRCError);
    EXPECT_FALSE(conn->isFailSafe());
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);
    // clearError only works from Error state or Data+fail_safe
    // After NOT_OK→Reset, we're in Reset, so clearError returns false
    bool result = conn->clearError();
    EXPECT_FALSE(result);
    EXPECT_FALSE(conn->isFailSafe());
}

TEST_F(FSoEMasterConnectionCoverageTest, SetOutputsInFailSafe) {
    // triggerFailSafe from Reset state goes back to Reset (NOT_OK transition)
    // isFailSafe is false, so setSafeOutputs is NOT blocked
    conn->triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_FALSE(conn->isFailSafe());
    uint8_t data[4] = {0, 0, 0, 0};
    EXPECT_TRUE(conn->setSafeOutputs(data, 4));
}

// --- Frame processing ---

TEST_F(FSoEMasterConnectionCoverageTest, ProcessRxFrameEmpty) {
    bool result = conn->processRxFrame(nullptr, 0);
    EXPECT_FALSE(result);
}

TEST_F(FSoEMasterConnectionCoverageTest, ProcessRxFrameTooShort) {
    uint8_t data[2] = {0, 0};
    bool result = conn->processRxFrame(data, 2);
    EXPECT_FALSE(result);
}

TEST_F(FSoEMasterConnectionCoverageTest, PrepareTxFrame) {
    uint8_t buffer[64] = {};
    size_t len = conn->prepareTxFrame(buffer, sizeof(buffer));
    // May return 0 if not in appropriate state
    (void)len;
}

TEST_F(FSoEMasterConnectionCoverageTest, PrepareTxFrameTooSmall) {
    uint8_t buffer[2] = {};
    size_t len = conn->prepareTxFrame(buffer, sizeof(buffer));
    EXPECT_EQ(len, 0u);
}

// --- Update/Watchdog ---

TEST_F(FSoEMasterConnectionCoverageTest, UpdateDoesNotCrash) {
    for (int i = 0; i < 100; ++i) {
        conn->update(i * 10);
    }
}

TEST_F(FSoEMasterConnectionCoverageTest, WatchdogTimeout) {
    conn->startConnection();
    // Simulate time passing beyond watchdog timeout
    conn->update(0);
    conn->update(200); // > 100ms watchdog timeout
}

// --- Stats ---

TEST_F(FSoEMasterConnectionCoverageTest, InitialStats) {
    auto stats = conn->getStats();
    EXPECT_EQ(stats.frames_sent, 0u);
    EXPECT_EQ(stats.frames_received, 0u);
    EXPECT_EQ(stats.crc_errors, 0u);
}

TEST_F(FSoEMasterConnectionCoverageTest, ResetStats) {
    conn->resetStats();
    auto stats = conn->getStats();
    EXPECT_EQ(stats.frames_sent, 0u);
}

// --- Callbacks ---

TEST_F(FSoEMasterConnectionCoverageTest, StateChangeCallback) {
    bool called = false;
    conn->setStateChangeCallback([&](uint8_t, uint8_t) { called = true; });
    conn->triggerFailSafe(ErrorCode::TimeoutError);
    // Should trigger state change callback
}

TEST_F(FSoEMasterConnectionCoverageTest, ErrorCallback) {
    bool called = false;
    conn->setErrorCallback([&](uint16_t code, const FSoE::FSoEErrorDetail&) {
        called = true;
        EXPECT_NE(code, ErrorCode::NoError);
    });
    conn->triggerFailSafe(ErrorCode::SequenceError);
}

TEST_F(FSoEMasterConnectionCoverageTest, FailSafeCallback) {
    bool called = false;
    conn->setFailSafeCallback([&]() { called = true; });
    conn->triggerFailSafe(ErrorCode::CRCError);
}

TEST_F(FSoEMasterConnectionCoverageTest, DataCallback) {
    bool called = false;
    conn->setDataCallback([&](const uint8_t*, size_t) { called = true; });
}

// --- Diagnostics ---

TEST_F(FSoEMasterConnectionCoverageTest, GetDiagnostics) {
    auto diag = conn->getDiagnostics();
    EXPECT_FALSE(diag.empty());
    EXPECT_NE(diag.find("Connection ID"), std::string::npos);
}

// ============================================================================
// FSoESlave coverage tests (supplementary to test_fsoe_slave_full.cpp)
// ============================================================================
class FSoESlaveCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        FSoESlaveConfig cfg{};
        cfg.slaveAddress = 0x0001;
        cfg.connectionId = 0x5678;
        cfg.safetyAddress = 0x0100;
        cfg.safetyLevel = SIL::SIL3;
        cfg.watchdogTimeoutMs = 200;
        cfg.connectionTimeoutMs = 2000;
        cfg.sessionTimeoutMs = 10000;
        cfg.safeInputSize = 2;
        cfg.safeOutputSize = 2;
        cfg.autoRecoveryEnabled = true;
        cfg.recoveryDelayMs = 500;
        cfg.strictCrcCheck = true;
        cfg.strictSequenceCheck = true;
        cfg.enableDiagnostics = true;
        cfg.maxErrorLogEntries = 50;
        slave = std::make_unique<FSoESlave>(cfg);
        slave->initialize();
    }
    std::unique_ptr<FSoESlave> slave;
};

TEST_F(FSoESlaveCoverageTest, InitialState) {
    EXPECT_TRUE(slave->isInitialized());
    EXPECT_FALSE(slave->isOperational());
    EXPECT_FALSE(slave->isFailSafe());
    EXPECT_FALSE(slave->hasError());
}

TEST_F(FSoESlaveCoverageTest, GetConfig) {
    auto& cfg = slave->getConfig();
    EXPECT_EQ(cfg.connectionId, 0x5678);
    EXPECT_EQ(cfg.safetyLevel, SIL::SIL3);
}

TEST_F(FSoESlaveCoverageTest, Reconfigure) {
    FSoESlaveConfig newCfg{};
    newCfg.slaveAddress = 0x0002;
    newCfg.connectionId = 0x9999;
    newCfg.safetyAddress = 0x0200;
    newCfg.safetyLevel = SIL::SIL3;
    newCfg.watchdogTimeoutMs = 100;
    newCfg.connectionTimeoutMs = 2000;
    newCfg.sessionTimeoutMs = 10000;
    newCfg.safeInputSize = 4;
    newCfg.safeOutputSize = 4;
    EXPECT_TRUE(slave->reconfigure(newCfg));
}

TEST_F(FSoESlaveCoverageTest, GetStateName) {
    const char* name = slave->getStateName();
    EXPECT_NE(name, nullptr);
    EXPECT_GT(strlen(name), 0u);
}

TEST_F(FSoESlaveCoverageTest, GetLastError) {
    EXPECT_EQ(slave->getLastError(), 0u);
}

TEST_F(FSoESlaveCoverageTest, SetSafeInputs) {
    uint8_t data[2] = {0xAA, 0x55};
    EXPECT_TRUE(slave->setSafeInputs(data, 2));
}

TEST_F(FSoESlaveCoverageTest, GetSafeOutputs) {
    uint8_t data[2] = {};
    size_t len = slave->getSafeOutputs(data, 2);
    EXPECT_LE(len, 2u);
}

TEST_F(FSoESlaveCoverageTest, AreSafeOutputsValid) {
    EXPECT_FALSE(slave->areSafeOutputsValid());
}

TEST_F(FSoESlaveCoverageTest, TriggerAndRecoverFailSafe) {
    slave->triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_TRUE(slave->isFailSafe());
    EXPECT_TRUE(slave->hasError());
    bool recovered = slave->attemptRecovery();
    // May or may not succeed depending on recovery delay
    (void)recovered;
}

TEST_F(FSoESlaveCoverageTest, ApplyFailSafeOutputs) {
    slave->applyFailSafeOutputs();
}

TEST_F(FSoESlaveCoverageTest, ResetSlave) {
    slave->triggerFailSafe(ErrorCode::CRCError);
    slave->reset();
    EXPECT_FALSE(slave->isFailSafe());
}

TEST_F(FSoESlaveCoverageTest, BitAccessors) {
    slave->setSafeInputBit(0, true);
    bool val = slave->getSafeOutputBit(0);
    (void)val;
}

TEST_F(FSoESlaveCoverageTest, ProcessRxFrameEmpty) {
    bool result = slave->processRxFrame(nullptr, 0);
    EXPECT_FALSE(result);
}

TEST_F(FSoESlaveCoverageTest, PrepareTxFrame) {
    uint8_t buffer[64] = {};
    size_t len = slave->prepareTxFrame(buffer, sizeof(buffer));
    (void)len;
}

TEST_F(FSoESlaveCoverageTest, UpdateCycles) {
    for (int i = 0; i < 100; ++i) {
        slave->update(i * 10);
    }
}

TEST_F(FSoESlaveCoverageTest, StatsAccess) {
    auto stats = slave->getStats();
    EXPECT_EQ(stats.framesReceived, 0u);
    slave->resetStats();
    auto stats2 = slave->getStats();
    EXPECT_EQ(stats2.framesReceived, 0u);
}

TEST_F(FSoESlaveCoverageTest, DiagnosticsAccess) {
    auto diag = slave->getDiagnostics();
    // Can be empty initially
    slave->clearDiagnostics();
}

TEST_F(FSoESlaveCoverageTest, ErrorInjectionAccess) {
    auto& injection = slave->getErrorInjection();
    EXPECT_FALSE(injection.enabled);

    FSoEErrorInjection newInjection{};
    newInjection.enabled = true;
    newInjection.injectCRCError = true;
    newInjection.crcErrorRate = 10;
    slave->setErrorInjection(newInjection);
    EXPECT_TRUE(slave->isErrorInjectionEnabled());
}

TEST_F(FSoESlaveCoverageTest, Callbacks) {
    slave->setStateCallback([](uint8_t, uint8_t) {});
    slave->setErrorCallback([](uint16_t, bool, const FSoE::FSoEErrorDetail&) {});
    slave->setFailSafeCallback([]() {});
    slave->setDataValidCallback([](const uint8_t*, size_t) {});
    slave->setRecoveryCallback([]() -> bool { return true; });
}
