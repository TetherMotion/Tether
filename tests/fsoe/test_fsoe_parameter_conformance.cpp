/**
 * @file test_fsoe_parameter_conformance.cpp
 * @brief Regression tests for ETG.5100 S (D) V1.2.0 §8.2.2.5 Parameter
 *        state PDU conformance.
 *
 * Verifies the FSoE Parameter PDU structure and behavior as described in:
 *   https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
 *
 * Tests cover:
 *   #1 Correct SafeData layout per Table 18 (comm param len, watchdog,
 *      app param len, app param bytes)
 *   #2 Multi-cycle transfer for payloads > safetyDataLen
 *   #3 Slave echoes back received safety data (Tables 20/22)
 *   #4 Master validates the slave's echo
 *   #5 Slave validates comm param length = 2
 *   #6 Slave validates watchdog range
 *   #7 Slave validates app parameters against expected values
 *   #8 Application parameter support (user-configurable)
 *   #9 0-octet safety data skips parameter transfer
 *   #10 Conn_Id field is set to actual Connection ID
 *   #11 Asymmetric sizes complete handshake
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

static MasterConnectionConfig makeMasterCfg(uint8_t inSize = 4,
                                             uint8_t outSize = 4,
                                             uint16_t connId = 0x1234,
                                             uint16_t watchdog = 200,
                                             std::vector<uint8_t> appParams = {}) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0100;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = connId;
    cfg.master_addr = 0x0100;
    cfg.watchdog_timeout_ms = watchdog;
    cfg.conn_timeout_ms = 5000;
    cfg.input_size = inSize;
    cfg.output_size = outSize;
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0};
    cfg.auto_recovery_enabled = false;
    cfg.app_parameters = appParams;
    return cfg;
}

static FSoESlaveConfig makeSlaveCfg(uint8_t inSize = 4,
                                     uint8_t outSize = 4,
                                     uint16_t connId = 0x1234,
                                     std::vector<uint8_t> expectedAppParams = {}) {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 0x0100;
    cfg.connectionId = connId;
    cfg.safetyAddress = 0x0100;
    cfg.safetyLevel = SIL::SIL2;
    cfg.watchdogTimeoutMs = 200;
    cfg.connectionTimeoutMs = 5000;
    cfg.sessionTimeoutMs = 10000;
    cfg.safeInputSize = inSize;
    cfg.safeOutputSize = outSize;
    cfg.autoRecoveryEnabled = false;
    cfg.failSafeInputs = {0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0};
    cfg.failSafeOutputs = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0};
    cfg.expectedAppParameters = expectedAppParams;
    return cfg;
}

static void advanceToData(FSoEMasterConnection& conn, FSoESlave& slave,
                          uint64_t& now, int maxCycles = 50) {
    now = 0;
    for (int i = 0; i < maxCycles; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational() && slave.isOperational()) break;
    }
    ASSERT_TRUE(conn.isOperational());
    ASSERT_TRUE(slave.isOperational());
}

/// Advance the master to the Parameter state.
/// Handles multi-cycle Connection transfer for safety data < 4 octets.
static void advanceToParameter(FSoEMasterConnection& conn, FSoESlave& slave,
                                uint64_t& now) {
    now = 0;
    // Reset → Session
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);
    // Session → Connection (1-octet data: 2 cycles for Session ID transfer)
    now += 15;
    conn.exchangeWith(slave, now);
    if (conn.getState() != ConnectionState::Connection) {
        now += 15;
        conn.exchangeWith(slave, now);
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);
    // Connection → Parameter (multi-cycle for < 4 octets)
    while (conn.getState() == ConnectionState::Connection) {
        now += 15;
        conn.exchangeWith(slave, now);
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Parameter);
}

/// Extract Conn_Id from an FSoE frame (last 2 bytes, little-endian).
static uint16_t extractConnId(const uint8_t* frame, size_t len) {
    if (len < 2) return 0;
    return static_cast<uint16_t>(frame[len - 2]) |
           (static_cast<uint16_t>(frame[len - 1]) << 8);
}

/// Extract SafeData from an FSoE frame using the proper CRC-aware parser.
static bool extractSafeData(const uint8_t* frame, size_t frameLen,
                            uint8_t* data, size_t dataLen) {
    uint8_t cmd = 0;
    size_t extracted_len = 0;
    uint16_t conn_id = 0;
    return CRC::extractFSoEFrame(frame, frameLen, cmd, data, extracted_len,
                                  conn_id) && extracted_len >= dataLen;
}

// ============================================================================
// #1: Correct SafeData layout per Table 18
// ============================================================================

TEST(FSoEParameterConformance, MasterFrameHasCorrectTable18Layout) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.5, Table 18:
    //   SafeData[0-1]: comm param length (always 2, LE)
    //   SafeData[2-3]: FSoE watchdog (ms, LE)
    //   SafeData[4-5]: app param length (LE)
    //   SafeData[6+]:  app param bytes
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    uint8_t frame[64];
    size_t frame_len = conn.prepareTxFrame(frame, sizeof(frame));
    ASSERT_GT(frame_len, 0u);

    uint8_t safe_data[4] = {};
    ASSERT_TRUE(extractSafeData(frame, frame_len, safe_data, 4));

    // First cycle: bytes 0-3 = comm param len (2) + watchdog (200)
    EXPECT_EQ(safe_data[0], 0x02);  // comm param len low = 2
    EXPECT_EQ(safe_data[1], 0x00);  // comm param len high = 0
    EXPECT_EQ(safe_data[2], 0xC8);  // watchdog low = 200
    EXPECT_EQ(safe_data[3], 0x00);  // watchdog high = 0
}

TEST(FSoEParameterConformance, MasterFrameHasAppParamLenInSecondCycle) {
    // With 4-octet data and no app params, the 6-byte payload takes 2 cycles.
    // Second cycle: bytes 0-1 = app param len (0), bytes 2-3 = padding (0)
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // First cycle: send bytes 0-3, receive echo
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Parameter);

    // Second cycle: send bytes 4-5 (app param len = 0) + padding
    uint8_t frame[64];
    size_t frame_len = conn.prepareTxFrame(frame, sizeof(frame));
    ASSERT_GT(frame_len, 0u);

    uint8_t safe_data[4] = {};
    ASSERT_TRUE(extractSafeData(frame, frame_len, safe_data, 4));

    EXPECT_EQ(safe_data[0], 0x00);  // app param len low = 0
    EXPECT_EQ(safe_data[1], 0x00);  // app param len high = 0
    EXPECT_EQ(safe_data[2], 0x00);  // padding
    EXPECT_EQ(safe_data[3], 0x00);  // padding
}

// ============================================================================
// #2: Multi-cycle transfer
// ============================================================================

TEST(FSoEParameterConformance, FourOctetDataTakesTwoCycles) {
    // 4-octet data, 6-byte payload (no app params) → ceil(6/4) = 2 cycles
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // 1st cycle: still in Parameter (only 4 of 6 bytes transferred)
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Parameter);

    // 2nd cycle: all 6 bytes transferred and echoed → Data
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}

TEST(FSoEParameterConformance, TwoOctetDataTakesThreeCycles) {
    // 2-octet data, 6-byte payload → ceil(6/2) = 3 cycles
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(2, 2));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    for (int i = 0; i < 2; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        EXPECT_EQ(conn.getState(), ConnectionState::Parameter);
    }
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}

TEST(FSoEParameterConformance, OneOctetDataTakesSixCycles) {
    // 1-octet data, 6-byte payload → ceil(6/1) = 6 cycles
    FSoEMasterConnection conn(makeMasterCfg(1, 1));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(1, 1));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    for (int i = 0; i < 5; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        EXPECT_EQ(conn.getState(), ConnectionState::Parameter);
    }
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}

TEST(FSoEParameterConformance, AppParamsIncreaseCycleCount) {
    // 4-octet data, 2 app param bytes → payload = 8 → ceil(8/4) = 2 cycles
    // (same as no app params, since 6 bytes already need 2 cycles)
    // 4-octet data, 4 app param bytes → payload = 10 → ceil(10/4) = 3 cycles
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200,
        {0xAA, 0xBB, 0xCC, 0xDD}));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234,
        {0xAA, 0xBB, 0xCC, 0xDD}));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    for (int i = 0; i < 2; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        EXPECT_EQ(conn.getState(), ConnectionState::Parameter);
    }
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}

// ============================================================================
// #3: Slave echoes back received safety data
// ============================================================================

TEST(FSoEParameterConformance, SlaveEchoesReceivedData) {
    // The slave's Parameter response should echo the received SafeData.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // Manually step through the first Parameter exchange to capture
    // the slave's response.
    uint8_t master_tx[64];
    size_t master_tx_len = conn.prepareTxFrame(master_tx, sizeof(master_tx));
    ASSERT_GT(master_tx_len, 0u);

    // Slave processes master's Parameter frame.
    slave.processRxFrame(master_tx, master_tx_len);
    ASSERT_EQ(slave.getState(), ConnectionState::Parameter);

    // Build slave's response — this should echo the first 4 bytes.
    uint8_t resp[64];
    size_t resp_len = slave.prepareTxFrame(resp, sizeof(resp));
    ASSERT_GT(resp_len, 0u);

    uint8_t resp_data[4] = {};
    ASSERT_TRUE(extractSafeData(resp, resp_len, resp_data, 4));

    // First 4 bytes of parameter payload:
    // 0x02, 0x00 (comm param len), 0xC8, 0x00 (watchdog=200)
    EXPECT_EQ(resp_data[0], 0x02);
    EXPECT_EQ(resp_data[1], 0x00);
    EXPECT_EQ(resp_data[2], 0xC8);
    EXPECT_EQ(resp_data[3], 0x00);
}

// ============================================================================
// #4: Master validates the slave's echo
// ============================================================================

TEST(FSoEParameterConformance, MasterRejectsEchoMismatch) {
    // The master should detect when the slave's echo doesn't match.
    // Use 8-octet data so the 6-byte payload fits in one cycle and
    // validation triggers immediately.
    FSoEMasterConnection conn(makeMasterCfg(8, 8, 0x1234, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(8, 8));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // Manually step: master sends Parameter frame, slave processes it,
    // then we corrupt the slave's response before sending to master.
    uint8_t master_tx[64];
    size_t master_tx_len = conn.prepareTxFrame(master_tx, sizeof(master_tx));
    ASSERT_GT(master_tx_len, 0u);

    slave.processRxFrame(master_tx, master_tx_len);

    uint8_t resp[64];
    size_t resp_len = slave.prepareTxFrame(resp, sizeof(resp));
    ASSERT_GT(resp_len, 0u);

    // Corrupt the SafeData bytes in the response (bytes 1-6 after CMD).
    // The SafeData starts at byte 1 (after the 1-byte command).
    for (size_t i = 1; i <= 6 && i < resp_len; ++i) {
        resp[i] ^= 0xFF;
    }
    // Recalculate CRC for the corrupted frame.
    uint8_t corrupted[64];
    uint8_t safe_data[8] = {0};
    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::extractFSoEFrame(resp, resp_len, cmd, safe_data,
                                       data_len, conn_id));
    // Flip all safe data bytes.
    for (size_t i = 0; i < data_len; ++i) {
        safe_data[i] ^= 0xFF;
    }
    size_t corrupted_len = CRC::buildFSoEFrame(corrupted, cmd,
                                                safe_data, data_len,
                                                conn_id,
                                                conn.getRxLastCrc0(),
                                                conn.getRxSeqNo());
    conn.processRxFrame(corrupted, corrupted_len);

    // Error during Parameter state goes back to Reset (NOT_OK transition)
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::ParameterError);
}

// ============================================================================
// #5: Slave validates comm param length = 2
// ============================================================================

TEST(FSoEParameterConformance, SlaveRejectsWrongCommParamLen) {
    // ETG.5100 §8.2.2.5: comm param length is always 2.
    // The slave should reject a frame with wrong comm param length.
    // Use 8-octet data so the full 6-byte header fits in one frame.
    FSoEMasterConnection conn(makeMasterCfg(8, 8, 0x1234, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(8, 8));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // Build a Parameter frame with comm param len = 3 (wrong).
    // Layout: [comm_len_lo=3] [comm_len_hi=0] [wd_lo] [wd_hi] [app_len_lo=0] [app_len_hi=0] [pad] [pad]
    uint8_t payload[] = {0x03, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Parameter,
                                            payload, 8, 0x1234,
                                            slave.getRxLastCrc0(),
                                            slave.getRxSeqNo());
    slave.processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave.isFailSafe());
    EXPECT_EQ(slave.getLastError(), ErrorCode::ParameterError);
}

// ============================================================================
// #6: Slave validates watchdog range
// ============================================================================

TEST(FSoEParameterConformance, SlaveRejectsWatchdogOutOfRange) {
    // ETG.5100: watchdog must be in [50, 60000] ms.
    // The slave should reject a watchdog value outside this range.
    // Use 8-octet data so the full 6-byte header fits in one frame.
    FSoEMasterConnection conn(makeMasterCfg(8, 8, 0x1234, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(8, 8));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // Build a Parameter frame with watchdog = 10 (too low).
    // Layout: [comm_len=2] [wd=10] [app_len=0] [pad] [pad]
    uint8_t payload[] = {0x02, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Parameter,
                                            payload, 8, 0x1234,
                                            slave.getRxLastCrc0(),
                                            slave.getRxSeqNo());
    slave.processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave.isFailSafe());
    EXPECT_EQ(slave.getLastError(), ErrorCode::ParameterError);
}

// ============================================================================
// #7: Slave validates app parameters against expected values
// ============================================================================

TEST(FSoEParameterConformance, SlaveRejectsWrongAppParams) {
    // The slave should reject app parameters that don't match expected.
    // Use 8-octet data with 2 app params: payload = 8 bytes, fits in 1 cycle.
    FSoEMasterConnection conn(makeMasterCfg(8, 8, 0x1234, 200,
        {0xAA, 0xBB}));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(8, 8, 0x1234,
        {0xAA, 0xBB}));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // Build a Parameter frame with correct header but wrong app params.
    // Layout: [comm_len=2] [wd=200] [app_len=2] [CC] [DD] [pad] [pad] [pad]
    uint8_t payload[] = {0x02, 0x00, 0xC8, 0x00, 0x02, 0x00, 0xCC, 0xDD};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Parameter,
                                            payload, 8, 0x1234,
                                            slave.getRxLastCrc0(),
                                            slave.getRxSeqNo());
    slave.processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave.isFailSafe());
    EXPECT_EQ(slave.getLastError(), ErrorCode::ParameterError);
}

TEST(FSoEParameterConformance, SlaveRejectsWrongAppParamLen) {
    // The slave should reject an app param length that doesn't match.
    // Use 8-octet data with 2 app params: payload = 8 bytes, fits in 1 cycle.
    FSoEMasterConnection conn(makeMasterCfg(8, 8, 0x1234, 200,
        {0xAA, 0xBB}));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(8, 8, 0x1234,
        {0xAA, 0xBB}));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    // Build a Parameter frame with wrong app param len (3 instead of 2).
    // Layout: [comm_len=2] [wd=200] [app_len=3] [AA] [BB] [pad] [pad] [pad]
    uint8_t payload[] = {0x02, 0x00, 0xC8, 0x00, 0x03, 0x00, 0xAA, 0xBB};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Parameter,
                                            payload, 8, 0x1234,
                                            slave.getRxLastCrc0(),
                                            slave.getRxSeqNo());
    slave.processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave.isFailSafe());
    EXPECT_EQ(slave.getLastError(), ErrorCode::ParameterError);
}

// ============================================================================
// #8: Application parameter support (user-configurable)
// ============================================================================

TEST(FSoEParameterConformance, AppParamsTransferredAndValidated) {
    // Full handshake with 2 app param bytes should complete successfully.
    std::vector<uint8_t> app_params = {0xAA, 0xBB};
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200, app_params));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, app_params));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);
}

TEST(FSoEParameterConformance, LargeAppParamsMultiCycle) {
    // 4-octet data, 10 app param bytes → payload = 16 → ceil(16/4) = 4 cycles
    std::vector<uint8_t> app_params = {0x01, 0x02, 0x03, 0x04,
                                        0x05, 0x06, 0x07, 0x08,
                                        0x09, 0x0A};
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200, app_params));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, app_params));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);
}

TEST(FSoEParameterConformance, EmptyAppParamsCompleteHandshake) {
    // No app params → payload = 6, should complete successfully.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 200, {}));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, {}));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);
}

// ============================================================================
// #9: 0-octet safety data skips parameter transfer
// ============================================================================

TEST(FSoEParameterConformance, ZeroOctetDataSkipsParameterTransfer) {
    // 0-octet safety data: no SafeData to transfer.  The Parameter state
    // should complete immediately.
    FSoEMasterConnection conn(makeMasterCfg(0, 0));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(0, 0));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);
}

// ============================================================================
// #10: Conn_Id field is set to actual Connection ID
// ============================================================================

TEST(FSoEParameterConformance, ConnIdFieldIsActualId) {
    // The Conn_Id field should be the actual Connection ID.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x5678, 200));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x5678));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToParameter(conn, slave, now);

    uint8_t frame[64];
    size_t frame_len = conn.prepareTxFrame(frame, sizeof(frame));
    ASSERT_GT(frame_len, 0u);

    uint16_t conn_id_field = extractConnId(frame, frame_len);
    EXPECT_EQ(conn_id_field, 0x5678);
}

// ============================================================================
// #11: Asymmetric sizes complete handshake
// ============================================================================

TEST(FSoEParameterConformance, AsymmetricSizesCompleteHandshake) {
    // Test various asymmetric size combinations.
    for (auto [inSize, outSize] : {std::make_pair(1, 4), std::make_pair(4, 1),
                                    std::make_pair(2, 4), std::make_pair(4, 2),
                                    std::make_pair(1, 2), std::make_pair(2, 1),
                                    std::make_pair(1, 8), std::make_pair(8, 1),
                                    std::make_pair(2, 8), std::make_pair(8, 2),
                                    std::make_pair(4, 8), std::make_pair(8, 4)}) {
        FSoEMasterConnection conn(makeMasterCfg(inSize, outSize));
        ASSERT_TRUE(conn.initialize());
        conn.startConnection();

        FSoESlave slave(makeSlaveCfg(inSize, outSize));
        ASSERT_TRUE(slave.initialize());

        uint64_t now = 0;
        advanceToData(conn, slave, now);

        EXPECT_EQ(conn.getState(), ConnectionState::Data);
        EXPECT_EQ(slave.getState(), ConnectionState::Data);

        // Exchange data — need enough cycles for the slave response delay
        // plus a few cycles for data to stabilize.
        for (int i = 0; i < 10; ++i) {
            now += 15;
            EXPECT_TRUE(conn.exchangeWith(slave, now));
        }

        EXPECT_TRUE(conn.areSafeInputsValid());
        EXPECT_TRUE(slave.areSafeOutputsValid());
    }
}

// ============================================================================
// #12: Slave updates watchdog from master's parameter frame
// ============================================================================

TEST(FSoEParameterConformance, SlaveUpdatesWatchdogFromMaster) {
    // The slave should update its watchdog from the master's parameter frame.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 300));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());
    // Slave starts with default watchdog (200)
    EXPECT_EQ(slave.getConfig().watchdogTimeoutMs, 200u);

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // After handshake, slave's watchdog should be updated to 300
    EXPECT_EQ(slave.getConfig().watchdogTimeoutMs, 300u);
}
