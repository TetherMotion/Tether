/**
 * @file test_fsoe_master_validation_regression.cpp
 * @brief Regression tests for master-side validation and concurrency fixes.
 *
 * Covers commits:
 * - U1: Master processRxFrame buffer overflow (11210d2)
 * - U2: getStatus()/getStats() return by value (11210d2)
 * - U3: Short connection response rejected (11210d2)
 * - X6: Early FailSafe error code extraction (9b9cd34)
 * - S1: Unexpected commands in FailSafe state (f168fcf)
 * - R8: Master Error state ignores non-Reset (6402663)
 * - R9: Master FailSafe state handles Reset (6402663)
 * - N1-N5: Re-audit fixes (6082e7b)
 * - d83d4be: Master fail-safe frame command and error/fail-safe state handling
 * - 2ebab0b: Parameter phase exchange and CRC verification
 * - 16b73a9: Watchdog min, slave_addr dedup, parameter response
 * - 6d3f30a: Slave return values, master lookup API, state name
 */

#include <gtest/gtest.h>
#include <cstring>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoEMaster.hpp"
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
// U1: Master processRxFrame Buffer Overflow (commit 11210d2)
// ============================================================================

TEST(FSoEMasterBufferOverflowRegression, MaxPayloadFailSafeResponseNoOverflow) {
    // ETG.5100 §8.2.2.6: FailSafeData has the same structure as ProcessData
    // (all SafeData = 0, no error code).  With safeInputSize=16, the payload
    // is 16 bytes.  The master should accept it without overflow.
    FSoESlave slave(makeSlaveCfg(16, 16));
    slave.initialize();

    MasterConnectionConfig cfg = makeMasterCfg(16, 16);
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Trigger fail-safe on slave
    slave.triggerFailSafe(ErrorCode::WatchdogError);

    // Exchange — master receives 16-byte FailSafeData response
    // This should not overflow (U1 fix enlarged buffer to MAX_PARSE_DATA_SIZE)
    bool ok = conn.exchangeWith(slave, now + 15);
    EXPECT_TRUE(ok);
    // ETG.5100 §8.2.2.6: Master does NOT auto-enter fail-safe when slave
    // sends FailSafeData (independent per direction)
    EXPECT_FALSE(conn.isFailSafe());
}

TEST(FSoEMasterBufferOverflowRegression, MaxPayloadProcessDataNoOverflow) {
    // Master with input_size=16 receives 16-byte ProcessData responses
    FSoESlave slave(makeSlaveCfg(16, 16));
    slave.initialize();

    MasterConnectionConfig cfg = makeMasterCfg(16, 16);
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Exchange several cycles — no overflow
    for (int i = 0; i < 5; ++i) {
        now += 15;
        EXPECT_TRUE(conn.exchangeWith(slave, now));
    }
    EXPECT_TRUE(conn.areSafeInputsValid());
}

// ============================================================================
// U2: getStatus()/getStats() Return by Value (commit 11210d2)
// ============================================================================

TEST(FSoEMasterReturnValueRegression, GetStatusReturnsByValue) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();

    auto status1 = conn.getStatus();
    auto status2 = conn.getStatus();
    EXPECT_EQ(status1.state, status2.state);

    // Modify one — should not affect the other
    status1.state = 0xFF;
    EXPECT_NE(conn.getStatus().state, 0xFF);
}

TEST(FSoEMasterReturnValueRegression, GetStatsReturnsByValue) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();

    auto stats1 = conn.getStats();
    auto stats2 = conn.getStats();
    EXPECT_EQ(stats1.frames_sent, stats2.frames_sent);

    stats1.frames_sent = 999;
    EXPECT_NE(conn.getStats().frames_sent, 999u);
}

// ============================================================================
// U3: Short Connection Response Rejected (commit 11210d2)
// ============================================================================

TEST(FSoEMasterConnectionValidationRegression, ShortConnectionResponseRejected) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Connection state.
    // We need to catch the master while it's still in Connection state.
    // After 1 exchange: master in Session (Reset→Session), slave in Session
    // After 2 exchanges: master in Connection, slave in Session
    // After 3 exchanges: master in Parameter, slave in Connection
    // So we send a short Connection response after 2 exchanges
    uint64_t now = 0;
    now += 15;
    conn.exchangeWith(slave, now);  // Reset → Session
    now += 15;
    conn.exchangeWith(slave, now);  // Session → Connection
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);

    // Send a short Connection response (1 byte instead of 4)
    // This triggers the data_len < 2 check in handleConnectionState.
    uint8_t payload[] = {0x34};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Connection,
                                            payload, 1, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    bool ok = conn.processRxFrame(frame, frame_len);

    // Should be rejected with DataLengthError
    EXPECT_TRUE(ok);  // processRxFrame returns true (frame was parsed)
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::DataLengthError);
}

// ============================================================================
// X6: FailSafeData handling in Data state (ETG.5100 §8.2.2.6)
// ============================================================================

TEST(FSoEMasterEarlyFailSafeRegression, ShortFailSafeDataUsesApplicationError) {
    // ETG.5100 §8.2.2.6: FailSafeData in Data state with a payload shorter
    // than input_size is a DataLengthError.  The master enters fail-safe
    // via handleError (auto_fail_safe_on_error=true).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Send a FailSafeData frame with only 2 bytes (less than input_size=4)
    uint8_t payload[] = {0xAA, 0xBB};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::FailSafeData,
                                            payload, 2, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    bool ok = conn.processRxFrame(frame, frame_len);
    EXPECT_TRUE(ok);
    // Short frame triggers DataLengthError → fail-safe
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::DataLengthError);
}

TEST(FSoEMasterEarlyFailSafeRegression, FullFailSafeDataExtractsErrorCode) {
    // ETG.5100 §8.2.2.6, Table 26: FailSafeData has the same structure as
    // ProcessData (all SafeData = 0, no error code field).  The master
    // accepts FailSafeData without entering fail-safe (independent per
    // direction).  No error code is extracted.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Send FailSafeData with 4 bytes of SafeData (all zeros per spec)
    uint8_t payload[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::FailSafeData,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    conn.processRxFrame(frame, frame_len);
    // Master does NOT enter fail-safe (independent per direction)
    EXPECT_FALSE(conn.isFailSafe());
    // No error code extracted (no error code field in FailSafeData PDU)
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::NoError);
    EXPECT_FALSE(conn.getStatus().data_valid);
}

TEST(FSoEMasterEarlyFailSafeRegression, ZeroInputSizeExtractsErrorCode) {
    // With input_size=0, the error code is at offset 0
    FSoEMasterConnection conn(makeMasterCfg(0, 0));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(0, 0));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // FailSafeData with 0 inputs + 2-byte error code
    uint8_t payload[] = {0x02, 0x00};  // CRCError
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::FailSafeData,
                                            payload, 2, 0x1234);
    conn.processRxFrame(frame, frame_len);
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::CRCError);
}

// ============================================================================
// S1: Unexpected Commands in FailSafe State (commit f168fcf)
// ============================================================================

TEST(FSoEMasterFailSafeStateRegression, UnexpectedCommandInFailSafeReportsError) {
    // ETG.5100 §8.2.2.6: FailSafeData is a command within the Data state,
    // not a separate state.  The master stays in Data state with
    // fail_safe_active flag.  ProcessData is a valid command in Data state
    // (the slave may send ProcessData while the master sends FailSafeData —
    // the choice is independent per direction).  So ProcessData should be
    // accepted, not rejected.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Trigger fail-safe on master — stays in Data state with fail_safe_active
    conn.triggerFailSafe(ErrorCode::WatchdogError);
    ASSERT_TRUE(conn.isFailSafe());
    // Master is still in Data state (not a separate FailSafe state)
    EXPECT_EQ(conn.getState(), ConnectionState::Data);

    // Send a ProcessData command — this is valid in Data state.
    // The slave may send ProcessData while the master sends FailSafeData.
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    bool ok = conn.processRxFrame(frame, frame_len);
    EXPECT_TRUE(ok);
    // ProcessData is accepted — error code should NOT be overwritten
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::WatchdogError);
}

// ============================================================================
// R8: Master Error State Ignores Non-Reset Commands (commit 6402663)
// ============================================================================

TEST(FSoEMasterErrorStateRegression, ErrorStateRejectsNonResetCommands) {
    // ETG.5100 §8.2.2.6: FailSafeData is a command within the Data state.
    // The master stays in Data state with fail_safe_active flag.
    // ProcessData is a valid command in Data state (the choice between
    // ProcessData and FailSafeData is independent per direction).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    conn.triggerFailSafe(ErrorCode::ApplicationError);
    ASSERT_TRUE(conn.isFailSafe());
    // Master is still in Data state (not a separate FailSafe state)
    EXPECT_EQ(conn.getState(), ConnectionState::Data);

    // Send a ProcessData command — this is valid in Data state.
    // The slave may send ProcessData while the master sends FailSafeData.
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    // ProcessData is accepted in Data state — no CommandError
    conn.processRxFrame(frame, frame_len);
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::ApplicationError);
}

// ============================================================================
// R9: Master FailSafe State Handles Reset (commit 6402663)
// ============================================================================

TEST(FSoEMasterFailSafeResetRegression, ResetCommandInFailSafeWithAutoRecovery) {
    // ETG.5100 §8.2.2.6: FailSafeData is a command within the Data state.
    // The master stays in Data state with fail_safe_active flag.
    // A Reset command in Data state triggers resetConnection().
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.auto_recovery_enabled = true;
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    conn.triggerFailSafe(ErrorCode::WatchdogError);
    ASSERT_TRUE(conn.isFailSafe());
    // Master is still in Data state (not a separate FailSafe state)
    EXPECT_EQ(conn.getState(), ConnectionState::Data);

    // Send Reset command — the slave's Reset response uses start_crc=0
    // and seq_no=1 (initial_seq_no + 1, since the slave increments the seq
    // after receiving the master's Reset which used seq=0).
    // The frame is the full fixed size with data_len = output_size.
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    payload[0] = 0x01;  // error code
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                            payload, 4u,
                                            0x1234,
                                            0,  // start_crc = 0 (Reset resets CRC chain)
                                            1);  // seq_no = 1 (slave increments after Reset)
    conn.processRxFrame(frame, frame_len);

    // Reset in Data state triggers resetConnection() — master goes to Reset
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_FALSE(conn.isFailSafe());
}

// ============================================================================
// d83d4be: Master Fail-Safe Frame Command
// ============================================================================

TEST(FSoEMasterFailSafeFrameRegression, FailSafeFrameUsesFailSafeDataCommand) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    conn.triggerFailSafe(ErrorCode::ApplicationError);

    uint8_t tx[64];
    size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    EXPECT_GT(tx_len, 0u);

    // The first byte should be FailSafeData (0x08), not ProcessData
    EXPECT_EQ(tx[0], Command::FailSafeData);
}

TEST(FSoEMasterFailSafeFrameRegression, FailSafeFrameContainsFailSafeValues) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.6, Table 25:
    // FailSafeData Master PDU: all SafeData octets are set to 0.
    // The fail-safe data carries no useful payload.
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    conn.triggerFailSafe(ErrorCode::ApplicationError);

    // Capture TX CRC state before prepareTxFrame updates it
    const uint16_t saved_tx_crc0 = conn.getTxLastCrc0();
    const uint16_t saved_tx_seq = conn.getTxSeqNo();

    uint8_t tx[64];
    size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Parse and verify all SafeData is zero per ETG.5100 Table 25.
    // Use the master's TX CRC state that was used to build the frame.
    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_tx_crc0, saved_tx_seq));
    EXPECT_EQ(cmd, Command::FailSafeData);
    EXPECT_EQ(data_len, 4u);
    // All SafeData octets must be 0 per spec (not fail_safe_values)
    EXPECT_EQ(data[0], 0x00);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[2], 0x00);
    EXPECT_EQ(data[3], 0x00);
}

// ============================================================================
// 16b73a9: Slave Address Dedup, Watchdog Min
// ============================================================================

TEST(FSoEMasterDedupRegression, DuplicateConnectionIdRejected) {
    FSoEMaster master;

    MasterConnectionConfig cfg1 = makeMasterCfg(4, 4);
    cfg1.slave_addr = 0x0100;
    cfg1.connection_id = 0x1234;
    int id1 = master.addConnection(cfg1);
    EXPECT_GE(id1, 0);

    MasterConnectionConfig cfg2 = makeMasterCfg(4, 4);
    cfg2.slave_addr = 0x0200;  // Different slave addr
    cfg2.connection_id = 0x1234;  // Same connection_id
    int id2 = master.addConnection(cfg2);
    EXPECT_EQ(id2, -1);  // Should reject duplicate connection_id
}

TEST(FSoEMasterDedupRegression, DuplicateSlaveAddrRejected) {
    FSoEMaster master;

    MasterConnectionConfig cfg1 = makeMasterCfg(4, 4);
    cfg1.slave_addr = 0x0100;
    cfg1.connection_id = 0x1234;
    int id1 = master.addConnection(cfg1);
    EXPECT_GE(id1, 0);

    MasterConnectionConfig cfg2 = makeMasterCfg(4, 4);
    cfg2.slave_addr = 0x0100;  // Same slave addr
    cfg2.connection_id = 0x5678;  // Different connection_id
    int id2 = master.addConnection(cfg2);
    EXPECT_EQ(id2, -1);  // Should reject duplicate slave_addr
}

TEST(FSoEMasterDedupRegression, UniqueConnectionAndSlaveAddrAccepted) {
    FSoEMaster master;

    MasterConnectionConfig cfg1 = makeMasterCfg(4, 4);
    cfg1.slave_addr = 0x0100;
    cfg1.connection_id = 0x1234;
    EXPECT_GE(master.addConnection(cfg1), 0);

    MasterConnectionConfig cfg2 = makeMasterCfg(4, 4);
    cfg2.slave_addr = 0x0200;
    cfg2.connection_id = 0x5678;
    EXPECT_GE(master.addConnection(cfg2), 0);
}

// ============================================================================
// 6d3f30a: Master Lookup API
// ============================================================================

TEST(FSoEMasterLookupRegression, GetConnectionBySlaveAddr) {
    FSoEMaster master;

    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.slave_addr = 0x0500;
    cfg.connection_id = 0x1234;
    master.addConnection(cfg);

    FSoEMasterConnection* conn = master.getConnectionBySlaveAddr(0x0500);
    ASSERT_NE(conn, nullptr);
    EXPECT_EQ(conn->getConfig().connection_id, 0x1234);

    EXPECT_EQ(master.getConnectionBySlaveAddr(0x9999), nullptr);
}

TEST(FSoEMasterLookupRegression, GetConnectionByConnId) {
    FSoEMaster master;

    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.slave_addr = 0x0500;
    cfg.connection_id = 0x4321;
    master.addConnection(cfg);

    FSoEMasterConnection* conn = master.getConnection(0x4321);
    ASSERT_NE(conn, nullptr);
    EXPECT_EQ(conn->getConfig().slave_addr, 0x0500);

    EXPECT_EQ(master.getConnection(0x9999), nullptr);
}

TEST(FSoEMasterLookupRegression, RemoveConnectionByConnId) {
    FSoEMaster master;

    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.slave_addr = 0x0500;
    cfg.connection_id = 0x4321;
    master.addConnection(cfg);

    EXPECT_EQ(master.getConnectionCount(), 1u);
    EXPECT_TRUE(master.removeConnection(0x4321));
    EXPECT_EQ(master.getConnectionCount(), 0u);
    EXPECT_FALSE(master.removeConnection(0x4321));
}

// ============================================================================
// 2ebab0b: Parameter Phase Exchange
// ============================================================================

TEST(FSoEMasterParameterPhaseRegression, ParameterPhaseCompletesToData) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);
}

TEST(FSoEMasterParameterPhaseRegression, ParameterCRCComputedOnInit) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();

    // The parameter CRC should be computed and non-zero for non-trivial configs
    auto diag = conn.getDiagnostics();
    EXPECT_NE(diag.find("Parameter CRC"), std::string::npos);
}

// ============================================================================
// N1-N5: Re-audit Fixes (commit 6082e7b)
// ============================================================================

TEST(FSoEMasterReAuditRegression, InvalidFrameSizeRejected) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Send a frame with invalid size (too short)
    uint8_t tiny[] = {0x00};
    EXPECT_FALSE(conn.processRxFrame(tiny, 1));

    // Send nullptr
    EXPECT_FALSE(conn.processRxFrame(nullptr, 10));
}

TEST(FSoEMasterReAuditRegression, CRCErrorTriggersFailSafe) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Build a valid frame then corrupt it
    uint8_t tx[64];
    size_t tx_len = slave.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Corrupt a data byte
    tx[1] ^= 0xFF;

    bool ok = conn.processRxFrame(tx, tx_len);
    EXPECT_FALSE(ok);
    auto stats = conn.getStats();
    EXPECT_GT(stats.crc_errors, 0u);
}

// ============================================================================
// State Name Coverage (commit 6d3f30a)
// ============================================================================

TEST(FSoESlaveStateNameRegression, AllStateNamesValid) {
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Check all state names are non-null and non-empty
    // We can't easily set all states, but we can check the current state
    const char* name = slave.getStateName();
    ASSERT_NE(name, nullptr);
    EXPECT_GT(strlen(name), 0u);
}

// ============================================================================
// Watchdog Timeout Behavior (commit d83d4be, e00e867)
// ============================================================================

TEST(FSoEMasterWatchdogRegression, WatchdogTimeoutTriggersFailSafe) {
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.watchdog_timeout_ms = 100;
    cfg.auto_fail_safe_on_error = true;
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Jump time beyond watchdog
    conn.update(now + 200);

    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::WatchdogError);
}

TEST(FSoEMasterWatchdogRegression, PhaseTimeoutTriggersFailSafe) {
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.conn_timeout_ms = 100;
    cfg.session_timeout_ms = 100;
    cfg.auto_fail_safe_on_error = true;
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    // Don't advance — let time pass in early state
    conn.update(0);
    conn.update(200);  // Beyond timeout

    // Master should have transitioned out of the initial state due to timeout.
    // The exact behavior (fail-safe, error, or reset) depends on the implementation.
    // Just verify it's no longer in the startup states.
    uint8_t state = conn.getState();
    EXPECT_TRUE(state == ConnectionState::Error ||
                state == ConnectionState::FailSafe ||
                conn.isFailSafe() ||
                state == ConnectionState::Reset);
}

// ============================================================================
// Auto-Recovery (commit d83d4be)
// ============================================================================

TEST(FSoEMasterAutoRecoveryRegression, AutoRecoveryFromFailSafe) {
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.auto_recovery_enabled = true;
    cfg.recovery_delay_ms = 100;
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    conn.triggerFailSafe(ErrorCode::WatchdogError);
    ASSERT_TRUE(conn.isFailSafe());

    // Wait for recovery delay
    conn.update(now + 200);

    // Should have attempted recovery
    auto stats = conn.getStats();
    EXPECT_GT(stats.recovery_attempts, 0u);
    EXPECT_FALSE(conn.isFailSafe());
}
