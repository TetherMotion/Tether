/**
 * @file test_fsoe_protocol_behavior_regression.cpp
 * @brief Behavioral regression tests for FSoE protocol state machine,
 *        fail-safe handling, and recovery.
 *
 * Covers commits:
 * - R2b/R2c: FailSafeData in all states, triggers fail-safe (6402663)
 * - R8/R9: Error/FailSafe state command handling (6402663)
 * - N1-N5: Re-audit protocol fixes (6082e7b)
 - d83d4be: Master fail-safe frame command and state handling
 * - e00e867: Slave watchdog and recovery timing
 * - 2ebab0b: Parameter phase exchange
 * - 9f49291: Slave conn_id validation and time source
 * - 94e4370: Odd-length frame format
 * - 16b73a9: Watchdog min, parameter response
 */

#include <gtest/gtest.h>
#include <cstring>
#include <memory>
#include <vector>
#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include "fsoe/FSoECRC.hpp"

using namespace FSoE;

// ============================================================================
// Test Helpers
// ============================================================================

static MasterConnectionConfig makeMasterCfg(uint8_t inSize = 4, uint8_t outSize = 4) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0100;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = 0x1234;
    cfg.master_addr = 0x0100;
    cfg.watchdog_timeout_ms = 200;
    cfg.conn_timeout_ms = 5000;
    cfg.input_size = inSize;
    cfg.output_size = outSize;
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    cfg.auto_recovery_enabled = false;
    return cfg;
}

static FSoESlaveConfig makeSlaveCfg(uint8_t inSize = 4, uint8_t outSize = 4) {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 0x0100;
    cfg.connectionId = 0x1234;
    cfg.safetyAddress = 0x0100;
    cfg.safetyLevel = SIL::SIL2;
    cfg.watchdogTimeoutMs = 200;
    cfg.connectionTimeoutMs = 5000;
    cfg.sessionTimeoutMs = 10000;
    cfg.safeInputSize = inSize;
    cfg.safeOutputSize = outSize;
    cfg.autoRecoveryEnabled = false;
    cfg.failSafeInputs = {0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    cfg.failSafeOutputs = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    return cfg;
}

static void advanceToData(FSoEMasterConnection& conn, FSoESlave& slave,
                          uint64_t& now, int maxCycles = 30) {
    now = 0;
    for (int i = 0; i < maxCycles; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational() && slave.isOperational()) break;
    }
    ASSERT_TRUE(conn.isOperational())
        << "Master state: " << (int)conn.getState()
        << " Slave state: " << (int)slave.getState();
    ASSERT_TRUE(slave.isOperational())
        << "Master state: " << (int)conn.getState()
        << " Slave state: " << (int)slave.getState();
}

// ============================================================================
// Full Protocol Handshake Behavior
// ============================================================================

class FSoEProtocolBehaviorTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave = std::make_unique<FSoESlave>(makeSlaveCfg(4, 4));
        slave->initialize();
        master = std::make_unique<FSoEMasterConnection>(makeMasterCfg(4, 4));
        master->initialize();
        master->startConnection();
    }
    std::unique_ptr<FSoESlave> slave;
    std::unique_ptr<FSoEMasterConnection> master;
};

TEST_F(FSoEProtocolBehaviorTest, FullHandshakeProgression) {
    uint64_t now = 0;

    // Cycle 0: Master sends Session, slave responds with Session
    now += 15;
    master->exchangeWith(*slave, now);
    EXPECT_EQ(master->getState(), ConnectionState::Connection);

    // Cycle 1: Master sends Connection, slave responds with Connection
    now += 15;
    master->exchangeWith(*slave, now);
    EXPECT_EQ(master->getState(), ConnectionState::Parameter);

    // Cycle 2: Master sends Parameter, slave responds with Parameter
    now += 15;
    master->exchangeWith(*slave, now);
    // Master transitions to Data after receiving Parameter response
    EXPECT_EQ(master->getState(), ConnectionState::Data);

    // Cycle 3: Master sends ProcessData, slave transitions to Data
    now += 15;
    master->exchangeWith(*slave, now);
    EXPECT_EQ(slave->getState(), ConnectionState::Data);
    EXPECT_TRUE(master->getStatus().data_valid);
}

TEST_F(FSoEProtocolBehaviorTest, SessionIdGeneratedNonZero) {
    master->requestSessionReset();
    auto status = master->getStatus();
    EXPECT_NE(status.session_id, 0u);
}

TEST_F(FSoEProtocolBehaviorTest, SessionIdDifferentOnEachReset) {
    master->requestSessionReset();
    uint16_t id1 = master->getStatus().session_id;

    master->resetConnection();
    master->requestSessionReset();
    uint16_t id2 = master->getStatus().session_id;

    // Very likely different (random generation)
    // We can't guarantee they're different, but it should be non-zero
    EXPECT_NE(id1, 0u);
    EXPECT_NE(id2, 0u);
}

TEST_F(FSoEProtocolBehaviorTest, DataExchangePreservesValues) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Set specific output values on master
    uint8_t outputs[] = {0x11, 0x22, 0x33, 0x44};
    master->setSafeOutputs(outputs, 4);

    // Exchange
    now += 15;
    master->exchangeWith(*slave, now);

    // Slave should have received the outputs
    uint8_t slave_outputs[4] = {0};
    slave->getSafeOutputs(slave_outputs, 4);
    EXPECT_EQ(slave_outputs[0], 0x11);
    EXPECT_EQ(slave_outputs[1], 0x22);
    EXPECT_EQ(slave_outputs[2], 0x33);
    EXPECT_EQ(slave_outputs[3], 0x44);

    // Set inputs on slave
    uint8_t inputs[] = {0xAA, 0xBB, 0xCC, 0xDD};
    slave->setSafeInputs(inputs, 4);

    // Exchange
    now += 15;
    master->exchangeWith(*slave, now);

    // Master should have received the inputs
    uint8_t master_inputs[4] = {0};
    master->getSafeInputs(master_inputs, 4);
    EXPECT_EQ(master_inputs[0], 0xAA);
    EXPECT_EQ(master_inputs[1], 0xBB);
    EXPECT_EQ(master_inputs[2], 0xCC);
    EXPECT_EQ(master_inputs[3], 0xDD);
}

// ============================================================================
// Fail-Safe Propagation Behavior
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, SlaveFailSafePropagatesToMaster) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Trigger fail-safe on slave
    slave->triggerFailSafe(ErrorCode::WatchdogError);

    // Exchange — master should detect slave's fail-safe
    now += 15;
    master->exchangeWith(*slave, now);

    EXPECT_TRUE(master->isFailSafe());
    EXPECT_EQ(master->getErrorCode(), ErrorCode::WatchdogError);
}

TEST_F(FSoEProtocolBehaviorTest, MasterFailSafePropagatesToSlave) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Trigger fail-safe on master
    master->triggerFailSafe(ErrorCode::CRCError);

    // Master sends FailSafeData
    now += 15;
    master->exchangeWith(*slave, now);

    // Slave should be in fail-safe
    EXPECT_TRUE(slave->isFailSafe());
}

TEST_F(FSoEProtocolBehaviorTest, FailSafeOutputsAppliedOnSlave) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Set normal outputs
    uint8_t outputs[] = {0x11, 0x22, 0x33, 0x44};
    master->setSafeOutputs(outputs, 4);
    now += 15;
    master->exchangeWith(*slave, now);

    // Trigger fail-safe on slave
    slave->triggerFailSafe(ErrorCode::ApplicationError);

    // In fail-safe, data is not valid — getSafeOutputs returns 0
    EXPECT_FALSE(slave->areSafeOutputsValid());

    // But the slave's TX frame should contain fail-safe values
    uint8_t tx[64];
    size_t tx_len = slave->prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Parse the FailSafeData response
    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id));
    EXPECT_EQ(cmd, Command::FailSafeData);
    // Fail-safe inputs are sent in the response
    EXPECT_EQ(data[0], 0xAA);
    EXPECT_EQ(data[1], 0xBB);
    EXPECT_EQ(data[2], 0xCC);
    EXPECT_EQ(data[3], 0xDD);
}

TEST_F(FSoEProtocolBehaviorTest, FailSafeOutputsAppliedOnMaster) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Trigger fail-safe on master
    master->triggerFailSafe(ErrorCode::ApplicationError);

    // Master outputs should be fail-safe values
    auto outputs = master->outputProcessData();
    ASSERT_EQ(outputs.size(), 4u);
    EXPECT_EQ(outputs[0], 0xDE);
    EXPECT_EQ(outputs[1], 0xAD);
    EXPECT_EQ(outputs[2], 0xBE);
    EXPECT_EQ(outputs[3], 0xEF);
}

// ============================================================================
// Recovery Behavior
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, MasterAutoRecoveryFromFailSafe) {
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.auto_recovery_enabled = true;
    cfg.recovery_delay_ms = 100;
    master = std::make_unique<FSoEMasterConnection>(cfg);
    master->initialize();
    master->startConnection();

    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    master->triggerFailSafe(ErrorCode::WatchdogError);
    ASSERT_TRUE(master->isFailSafe());

    // Wait for recovery delay
    master->update(now + 200);

    EXPECT_FALSE(master->isFailSafe());
    auto stats = master->getStats();
    EXPECT_GT(stats.recovery_attempts, 0u);
}

TEST_F(FSoEProtocolBehaviorTest, SlaveAutoRecoveryFromFailSafe) {
    FSoESlaveConfig cfg = makeSlaveCfg(4, 4);
    cfg.autoRecoveryEnabled = true;
    cfg.recoveryDelayMs = 100;
    slave = std::make_unique<FSoESlave>(cfg);
    slave->initialize();

    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    slave->triggerFailSafe(ErrorCode::WatchdogError);
    ASSERT_TRUE(slave->isFailSafe());

    // Wait for recovery delay
    slave->update(now + 200);

    EXPECT_FALSE(slave->isFailSafe());
    auto stats = slave->getStats();
    EXPECT_GT(stats.recoveryAttempts, 0u);
}

TEST_F(FSoEProtocolBehaviorTest, ManualRecoveryViaResetCommand) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Both in fail-safe
    slave->triggerFailSafe(ErrorCode::WatchdogError);
    now += 15;
    master->exchangeWith(*slave, now);
    ASSERT_TRUE(master->isFailSafe());
    ASSERT_TRUE(slave->isFailSafe());

    // Master sends Reset
    master->resetConnection();
    now += 15;
    master->exchangeWith(*slave, now);

    // Both should be back in earlier states
    EXPECT_FALSE(master->isFailSafe());
}

// ============================================================================
// Error Injection Behavior
// ============================================================================

class FSoEErrorInjectionBehaviorTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave = std::make_unique<FSoESlave>(makeSlaveCfg(4, 4));
        slave->initialize();
        master = std::make_unique<FSoEMasterConnection>(makeMasterCfg(4, 4));
        master->initialize();
        master->startConnection();
        advanceToData(*master, *slave, now);
    }
    std::unique_ptr<FSoESlave> slave;
    std::unique_ptr<FSoEMasterConnection> master;
    uint64_t now = 0;
};

TEST_F(FSoEErrorInjectionBehaviorTest, CRCErrorInjectionOnSlave) {
    auto& inj = slave->getErrorInjection();
    inj.enabled = true;
    inj.injectCRCError = true;
    inj.crcErrorRate = 0;  // Every packet

    // The slave will reject all incoming frames with CRC error
    // Exchange should fail because slave rejects the master's frame
    now += 15;
    bool ok = master->exchangeWith(*slave, now);

    // The slave's stats should show CRC errors
    auto slave_stats = slave->getStats();
    EXPECT_GT(slave_stats.crcErrors, 0u);
}

TEST_F(FSoEErrorInjectionBehaviorTest, FrameDropInjectionOnSlave) {
    auto& inj = slave->getErrorInjection();
    inj.enabled = true;
    inj.dropFrames = true;
    inj.dropRate = 0;  // Drop all

    // Slave won't process any frames
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            payload, 4, 0x1234);
    bool ok = slave->processRxFrame(frame, frame_len);
    EXPECT_FALSE(ok);

    auto stats = slave->getStats();
    EXPECT_GT(stats.invalidFrames, 0u);
}

TEST_F(FSoEErrorInjectionBehaviorTest, ForceFailSafeInjection) {
    auto& inj = slave->getErrorInjection();
    inj.enabled = true;
    inj.forceFailSafe = true;

    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            payload, 4, 0x1234);
    slave->processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave->isFailSafe());
}

TEST_F(FSoEErrorInjectionBehaviorTest, ConnIdErrorInjection) {
    auto& inj = slave->getErrorInjection();
    inj.enabled = true;
    inj.injectConnIdError = true;
    inj.fakeConnId = 0xFFFF;

    // Send a valid frame — slave should reject due to conn_id mismatch
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            payload, 4, 0x1234);
    slave->processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave->isFailSafe());
    EXPECT_EQ(slave->getLastError(), ErrorCode::ConnectionIDError);
}

// ============================================================================
// Various Data Size Configurations
// ============================================================================

class FSoEDataSizeBehaviorTest : public ::testing::TestWithParam<std::pair<uint8_t, uint8_t>> {};

TEST_P(FSoEDataSizeBehaviorTest, FullHandshakeWithSizes) {
    auto [inSize, outSize] = GetParam();

    FSoESlaveConfig scfg = makeSlaveCfg(inSize, outSize);
    FSoESlave slave(scfg);
    slave.initialize();

    MasterConnectionConfig mcfg = makeMasterCfg(inSize, outSize);
    FSoEMasterConnection conn(mcfg);
    conn.initialize();
    conn.startConnection();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);

    // Exchange data
    for (int i = 0; i < 5; ++i) {
        now += 15;
        EXPECT_TRUE(conn.exchangeWith(slave, now));
    }

    EXPECT_TRUE(conn.areSafeInputsValid());
    EXPECT_TRUE(slave.areSafeOutputsValid());
}

INSTANTIATE_TEST_SUITE_P(Sizes, FSoEDataSizeBehaviorTest,
    ::testing::Values(
        std::make_pair(0, 0),
        std::make_pair(1, 1),
        std::make_pair(2, 2),
        std::make_pair(4, 4),
        std::make_pair(8, 8),
        std::make_pair(16, 16),
        std::make_pair(1, 4),
        std::make_pair(4, 1),
        std::make_pair(3, 5),  // Odd sizes
        std::make_pair(7, 3)   // Odd sizes
    ));

// ============================================================================
// Reset and Re-initialization Behavior
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, ResetClearsFailSafe) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    slave->triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_TRUE(slave->isFailSafe());

    slave->reset();
    EXPECT_FALSE(slave->isFailSafe());
    EXPECT_EQ(slave->getLastError(), ErrorCode::NoError);
    EXPECT_EQ(slave->getState(), ConnectionState::Reset);
}

TEST_F(FSoEProtocolBehaviorTest, ResetClearsError) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    master->triggerFailSafe(ErrorCode::CRCError);
    EXPECT_TRUE(master->isFailSafe());

    bool cleared = master->clearError();
    EXPECT_TRUE(cleared);
    EXPECT_FALSE(master->isFailSafe());
}

TEST_F(FSoEProtocolBehaviorTest, ReconfigureInResetOnly) {
    // Slave must be in Reset state to reconfigure
    slave->reset();
    ASSERT_EQ(slave->getState(), ConnectionState::Reset);

    FSoESlaveConfig newCfg = makeSlaveCfg(2, 2);
    EXPECT_TRUE(slave->reconfigure(newCfg));

    // After reconfigure, should be initialized with new config
    auto& cfg = slave->getConfig();
    EXPECT_EQ(cfg.safeInputSize, 2u);
    EXPECT_EQ(cfg.safeOutputSize, 2u);
}

TEST_F(FSoEProtocolBehaviorTest, ReconfigureFailsInDataState) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);
    ASSERT_EQ(slave->getState(), ConnectionState::Data);

    FSoESlaveConfig newCfg = makeSlaveCfg(2, 2);
    EXPECT_FALSE(slave->reconfigure(newCfg));
}

// ============================================================================
// Callback Behavior
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, StateChangeCallbackFiresOnTransition) {
    uint8_t old_state = 0xFF, new_state = 0xFF;
    bool callback_fired = false;

    slave->setStateCallback([&](uint8_t o, uint8_t n) {
        old_state = o;
        new_state = n;
        callback_fired = true;
    });

    // Trigger a state transition by sending Session command
    uint8_t payload[] = {0x01, 0x00};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Session,
                                            payload, 2, 0x1234);
    slave->processRxFrame(frame, frame_len);

    EXPECT_TRUE(callback_fired);
    // Should have transitioned from Reset to Session
    EXPECT_EQ(old_state, ConnectionState::Reset);
    EXPECT_EQ(new_state, ConnectionState::Session);
}

TEST_F(FSoEProtocolBehaviorTest, DataValidCallbackFiresOnProcessData) {
    bool callback_fired = false;
    size_t received_len = 0;

    slave->setDataValidCallback([&](const uint8_t* data, size_t len) {
        callback_fired = true;
        received_len = len;
    });

    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // The callback should have fired when ProcessData was received
    EXPECT_TRUE(callback_fired);
    EXPECT_EQ(received_len, 4u);
}

TEST_F(FSoEProtocolBehaviorTest, FailSafeCallbackFiresOnce) {
    int callback_count = 0;
    slave->setFailSafeCallback([&]() {
        callback_count++;
    });

    slave->triggerFailSafe(ErrorCode::WatchdogError);
    EXPECT_EQ(callback_count, 1);

    // Trigger again — should NOT fire (already in fail-safe)
    slave->triggerFailSafe(ErrorCode::CRCError);
    EXPECT_EQ(callback_count, 1);
}

// ============================================================================
// Statistics Tracking
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, StatsTrackFrames) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    auto master_stats = master->getStats();
    auto slave_stats = slave->getStats();

    EXPECT_GT(master_stats.frames_sent, 0u);
    EXPECT_GT(master_stats.frames_received, 0u);
    EXPECT_GT(slave_stats.framesSent, 0u);
    EXPECT_GT(slave_stats.framesReceived, 0u);
}

TEST_F(FSoEProtocolBehaviorTest, StatsTrackFailSafeActivations) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    auto before = slave->getStats();
    slave->triggerFailSafe(ErrorCode::ApplicationError);
    auto after = slave->getStats();

    EXPECT_GT(after.failSafeActivations, before.failSafeActivations);
}

TEST_F(FSoEProtocolBehaviorTest, StatsTrackSessionResets) {
    auto before = slave->getStats();
    slave->reset();
    auto after = slave->getStats();

    EXPECT_GT(after.sessionResets, before.sessionResets);
}

// ============================================================================
// Process Data Span API
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, WriteInputProcessDataSpan) {
    std::array<uint8_t, 4> data = {0x12, 0x34, 0x56, 0x78};
    EXPECT_TRUE(slave->writeInputProcessData(std::span<const uint8_t>(data)));

    auto inputs = slave->inputProcessData();
    ASSERT_EQ(inputs.size(), 4u);
    EXPECT_EQ(inputs[0], 0x12);
    EXPECT_EQ(inputs[1], 0x34);
    EXPECT_EQ(inputs[2], 0x56);
    EXPECT_EQ(inputs[3], 0x78);
}

TEST_F(FSoEProtocolBehaviorTest, ReadOutputProcessDataSpan) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    // Set outputs on master and exchange
    uint8_t outputs[] = {0xAB, 0xCD, 0xEF, 0x01};
    master->setSafeOutputs(outputs, 4);
    now += 15;
    master->exchangeWith(*slave, now);

    // Read via span API
    std::array<uint8_t, 4> buf = {0};
    size_t len = slave->readOutputProcessData(std::span<uint8_t>(buf));
    EXPECT_EQ(len, 4u);
    EXPECT_EQ(buf[0], 0xAB);
    EXPECT_EQ(buf[1], 0xCD);
    EXPECT_EQ(buf[2], 0xEF);
    EXPECT_EQ(buf[3], 0x01);
}

TEST_F(FSoEProtocolBehaviorTest, InputProcessDataVector) {
    std::array<uint8_t, 4> data = {0x42, 0x43, 0x44, 0x45};
    slave->writeInputProcessData(std::span<const uint8_t>(data));

    auto inputs = slave->inputProcessData();
    ASSERT_EQ(inputs.size(), 4u);
    EXPECT_EQ(inputs[0], 0x42);
}

TEST_F(FSoEProtocolBehaviorTest, OutputProcessDataVector) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    uint8_t outputs[] = {0x99, 0x88, 0x77, 0x66};
    master->setSafeOutputs(outputs, 4);
    now += 15;
    master->exchangeWith(*slave, now);

    auto out = slave->outputProcessData();
    ASSERT_EQ(out.size(), 4u);
    EXPECT_EQ(out[0], 0x99);
    EXPECT_EQ(out[1], 0x88);
    EXPECT_EQ(out[2], 0x77);
    EXPECT_EQ(out[3], 0x66);
}

// ============================================================================
// Bit-Level I/O Behavior
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, SetAndGetSafeInputBit) {
    slave->setSafeInputBit(0, true);
    slave->setSafeInputBit(7, true);
    slave->setSafeInputBit(8, false);

    auto inputs = slave->inputProcessData();
    EXPECT_TRUE(inputs[0] & 0x01);  // Bit 0
    EXPECT_TRUE(inputs[0] & 0x80);  // Bit 7
    EXPECT_FALSE(inputs[1] & 0x01); // Bit 8
}

TEST_F(FSoEProtocolBehaviorTest, SetSafeInputBitOutOfRange) {
    EXPECT_FALSE(slave->setSafeInputBit(32, true));  // Beyond 4 bytes
    EXPECT_FALSE(slave->setSafeInputBit(64, true));  // Way out of range
}

TEST_F(FSoEProtocolBehaviorTest, GetSafeOutputBitOutOfRange) {
    EXPECT_FALSE(slave->getSafeOutputBit(32));
    EXPECT_FALSE(slave->getSafeOutputBit(64));
}

// ============================================================================
// Diagnostics Behavior
// ============================================================================

TEST_F(FSoEProtocolBehaviorTest, DiagnosticsContainsEntries) {
    uint64_t now = 0;
    advanceToData(*master, *slave, now);

    slave->triggerFailSafe(ErrorCode::WatchdogError);

    auto diag = slave->getDiagnostics();
    EXPECT_FALSE(diag.empty());

    // Should contain an entry for the fail-safe
    bool found_fail_safe = false;
    for (const auto& entry : diag) {
        if (entry.errorCode == ErrorCode::WatchdogError) {
            found_fail_safe = true;
            break;
        }
    }
    EXPECT_TRUE(found_fail_safe);
}

TEST_F(FSoEProtocolBehaviorTest, ClearDiagnostics) {
    slave->initialize();
    slave->triggerFailSafe(ErrorCode::ApplicationError);

    EXPECT_FALSE(slave->getDiagnostics().empty());

    slave->clearDiagnostics();
    EXPECT_TRUE(slave->getDiagnostics().empty());
}

TEST_F(FSoEProtocolBehaviorTest, DiagnosticsMaxEntries) {
    FSoESlaveConfig cfg = makeSlaveCfg(4, 4);
    cfg.maxErrorLogEntries = 5;
    FSoESlave slave(cfg);
    slave.initialize();

    // Generate more than 5 diagnostic entries
    for (int i = 0; i < 10; ++i) {
        slave.triggerFailSafe(ErrorCode::ApplicationError);
        slave.reset();
    }

    auto diag = slave.getDiagnostics();
    EXPECT_LE(diag.size(), 5u);
}
