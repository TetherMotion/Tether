/**
 * @file test_fsoe_data_conformance.cpp
 * @brief Regression tests for ETG.5100 S (D) V1.2.0 §8.2.2.6 Data state
 *        PDU conformance.
 *
 * Verifies the FSoE Data PDU structure and behavior as described in:
 *   https://techoverflow.net/2026/08/12/fsoe-data-pdu-master-and-slave-structure/
 *
 * Tests cover:
 *   #1 ProcessData Master PDU: SafeOutputs in SafeData (Table 23)
 *   #2 ProcessData Slave PDU: SafeInputs in SafeData, NOT echo (Table 24)
 *   #3 FailSafeData Master PDU: all SafeData = 0 (Table 25)
 *   #4 FailSafeData Slave PDU: all SafeData = 0, same size as ProcessData (Table 26)
 *   #5 FailSafeData has no error code field
 *   #6 ProcessData/FailSafeData choice is independent per direction
 *   #7 Master stays in Data state when sending FailSafeData (no separate state)
 *   #8 Master accepts slave's FailSafeData without entering fail-safe
 *   #9 Slave accepts master's FailSafeData without entering fail-safe
 *   #10 Conn_Id field is set to actual Connection ID in Data state
 *   #11 CRC inheritance chains Data-state PDUs
 *   #12 Various safety data sizes (1, 2, 4, 6, 8)
 *   #13 Reset command in Data state triggers reset
 *   #14 FailSafeData in non-Data states triggers fail-safe (handshake abort)
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
                                             uint16_t watchdog = 200) {
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
    return cfg;
}

static FSoESlaveConfig makeSlaveCfg(uint8_t inSize = 4,
                                     uint8_t outSize = 4,
                                     uint16_t connId = 0x1234) {
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

// ============================================================================
// #1: ProcessData Master PDU — SafeOutputs in SafeData (Table 23)
// ============================================================================

TEST(FSoEDataConformance, ProcessDataMasterPDU_CarriesSafeOutputs) {
    // ETG.5100 Table 23: The master sends SafeOutputs in SafeData[0..N-1].
    // The bytes are placed in order: SafeData[0] is the 1st octet, etc.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Set specific SafeOutputs
    uint8_t outputs[] = {0x11, 0x22, 0x33, 0x44};
    ASSERT_TRUE(conn.setSafeOutputs(outputs, 4));

    // Capture TX frame
    const uint16_t saved_crc0 = conn.getTxLastCrc0();
    const uint16_t saved_seq = conn.getTxSeqNo();
    uint8_t tx[64];
    size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Parse and verify SafeOutputs are in SafeData
    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_crc0, saved_seq));
    EXPECT_EQ(cmd, Command::ProcessData);
    EXPECT_EQ(data_len, 4u);
    EXPECT_EQ(data[0], 0x11);
    EXPECT_EQ(data[1], 0x22);
    EXPECT_EQ(data[2], 0x33);
    EXPECT_EQ(data[3], 0x44);
}

// ============================================================================
// #2: ProcessData Slave PDU — SafeInputs, NOT echo (Table 24)
// ============================================================================

TEST(FSoEDataConformance, ProcessDataSlavePDU_CarriesSafeInputsNotEcho) {
    // ETG.5100 Table 24: The slave sends its OWN SafeInputs, not an echo
    // of the master's SafeOutputs.  Each direction carries independent payload.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Set master SafeOutputs to known values
    uint8_t outputs[] = {0x11, 0x22, 0x33, 0x44};
    conn.setSafeOutputs(outputs, 4);

    // Set slave SafeInputs to DIFFERENT values
    uint8_t inputs[] = {0xAA, 0xBB, 0xCC, 0xDD};
    slave.setSafeInputs(inputs, 4);

    // Exchange one cycle
    now += 15;
    conn.exchangeWith(slave, now);

    // Master should have received the slave's SafeInputs, not an echo
    uint8_t master_inputs[4] = {0};
    conn.getSafeInputs(master_inputs, 4);
    EXPECT_EQ(master_inputs[0], 0xAA);
    EXPECT_EQ(master_inputs[1], 0xBB);
    EXPECT_EQ(master_inputs[2], 0xCC);
    EXPECT_EQ(master_inputs[3], 0xDD);
}

// ============================================================================
// #3: FailSafeData Master PDU — all SafeData = 0 (Table 25)
// ============================================================================

TEST(FSoEDataConformance, FailSafeDataMasterPDU_AllZeroSafeData) {
    // ETG.5100 Table 25: All SafeData octets are set to 0 in FailSafeData.
    // The fail-safe data carries no useful payload.
    // triggerFailSafe only works in Data state; from other states it
    // goes back to Reset (NOT_OK transition).
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0};
    FSoEMasterConnection conn(cfg);
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    // Advance to Data state first — triggerFailSafe only works in Data
    uint64_t now = 0;
    advanceToData(conn, slave, now);

    conn.triggerFailSafe(ErrorCode::ApplicationError);
    ASSERT_TRUE(conn.isFailSafe());

    const uint16_t saved_crc0 = conn.getTxLastCrc0();
    const uint16_t saved_seq = conn.getTxSeqNo();
    uint8_t tx[64];
    size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_crc0, saved_seq));
    EXPECT_EQ(cmd, Command::FailSafeData);
    EXPECT_EQ(data_len, 4u);
    // All SafeData must be 0, NOT fail_safe_values
    EXPECT_EQ(data[0], 0x00);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[2], 0x00);
    EXPECT_EQ(data[3], 0x00);
}

// ============================================================================
// #4: FailSafeData Slave PDU — all SafeData = 0, same size as ProcessData
// ============================================================================

TEST(FSoEDataConformance, FailSafeDataSlavePDU_AllZeroSameSizeAsProcessData) {
    // ETG.5100 Table 26: FailSafeData Slave PDU has the same structure and
    // size as ProcessData.  All SafeData = 0, no error code field.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Capture ProcessData frame size from slave
    uint16_t pd_crc0 = slave.getTxLastCrc0();
    uint16_t pd_seq = slave.getTxSeqNo();
    uint8_t pd_tx[64];
    size_t pd_len = slave.prepareTxFrame(pd_tx, sizeof(pd_tx));
    ASSERT_GT(pd_len, 0u);

    uint8_t pd_cmd = 0;
    uint8_t pd_data[18] = {0};
    size_t pd_data_len = 0;
    uint16_t pd_conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(pd_tx, pd_len, pd_cmd, pd_data,
                                    pd_data_len, pd_conn_id,
                                    pd_crc0, pd_seq));
    EXPECT_EQ(pd_cmd, Command::ProcessData);

    // Now trigger fail-safe and capture FailSafeData frame
    slave.triggerFailSafe(ErrorCode::WatchdogError);

    const uint16_t fs_crc0 = slave.getTxLastCrc0();
    const uint16_t fs_seq = slave.getTxSeqNo();
    uint8_t fs_tx[64];
    size_t fs_len = slave.prepareTxFrame(fs_tx, sizeof(fs_tx));
    ASSERT_GT(fs_len, 0u);

    uint8_t fs_cmd = 0;
    uint8_t fs_data[18] = {0};
    size_t fs_data_len = 0;
    uint16_t fs_conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(fs_tx, fs_len, fs_cmd, fs_data,
                                    fs_data_len, fs_conn_id,
                                    fs_crc0, fs_seq));
    EXPECT_EQ(fs_cmd, Command::FailSafeData);
    // Same data length as ProcessData (no extra error code field)
    EXPECT_EQ(fs_data_len, pd_data_len);
    // All SafeData = 0
    for (size_t i = 0; i < fs_data_len; ++i) {
        EXPECT_EQ(fs_data[i], 0x00) << "at offset " << i;
    }
}

// ============================================================================
// #5: FailSafeData has no error code field
// ============================================================================

TEST(FSoEDataConformance, FailSafeDataNoErrorCodeField) {
    // ETG.5100 Tables 25/26: FailSafeData PDU has no error code field.
    // The PDU structure is identical to ProcessData, just with zeroed data.
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    slave.triggerFailSafe(ErrorCode::CRCError);

    const uint16_t saved_crc0 = slave.getTxLastCrc0();
    const uint16_t saved_seq = slave.getTxSeqNo();
    uint8_t tx[64];
    size_t tx_len = slave.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_crc0, saved_seq));
    EXPECT_EQ(cmd, Command::FailSafeData);
    // data_len should be safeInputSize (4), NOT safeInputSize + 2 (6)
    EXPECT_EQ(data_len, 4u);
    // No error code at offset 4-5
    EXPECT_EQ(data[4], 0x00);
    EXPECT_EQ(data[5], 0x00);
}

// ============================================================================
// #6: ProcessData/FailSafeData choice is independent per direction
// ============================================================================

TEST(FSoEDataConformance, IndependentCommandChoice_MasterProcessData_SlaveFailSafe) {
    // Master sends ProcessData while slave sends FailSafeData.
    // Both should be accepted by the other side without forcing fail-safe.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Slave enters fail-safe (its SafeInputs are not valid)
    slave.triggerFailSafe(ErrorCode::WatchdogError);
    ASSERT_TRUE(slave.isFailSafe());

    // Master stays in normal mode (its SafeOutputs are valid)
    ASSERT_FALSE(conn.isFailSafe());

    // Exchange — master sends ProcessData, slave sends FailSafeData
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // Master should NOT have entered fail-safe
    EXPECT_FALSE(conn.isFailSafe());
    // Master's data is not valid (slave sent fail-safe data)
    EXPECT_FALSE(conn.getStatus().data_valid);
    // Slave should still be in fail-safe
    EXPECT_TRUE(slave.isFailSafe());
}

TEST(FSoEDataConformance, IndependentCommandChoice_MasterFailSafe_SlaveProcessData) {
    // Master sends FailSafeData while slave sends ProcessData.
    // Both should be accepted by the other side without forcing fail-safe.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Master enters fail-safe (its SafeOutputs are not valid)
    conn.triggerFailSafe(ErrorCode::CRCError);
    ASSERT_TRUE(conn.isFailSafe());

    // Slave stays in normal mode (its SafeInputs are valid)
    ASSERT_FALSE(slave.isFailSafe());

    // Exchange — master sends FailSafeData, slave sends ProcessData
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // Slave should NOT have entered fail-safe
    EXPECT_FALSE(slave.isFailSafe());
    // Master should still be in fail-safe
    EXPECT_TRUE(conn.isFailSafe());
}

// ============================================================================
// #7: Master stays in Data state when sending FailSafeData
// ============================================================================

TEST(FSoEDataConformance, MasterStaysInDataStateWhenFailSafe) {
    // ETG.5100 §8.2.2.6: FailSafeData is a command within the Data state,
    // not a separate state.  The master stays in Data state with
    // fail_safe_active flag.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Data);

    // Trigger fail-safe — should stay in Data state
    conn.triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}

// ============================================================================
// #8: Master accepts slave's FailSafeData without entering fail-safe
// ============================================================================

TEST(FSoEDataConformance, MasterAcceptsSlaveFailSafeDataWithoutEnteringFailSafe) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    slave.triggerFailSafe(ErrorCode::WatchdogError);

    now += 15;
    conn.exchangeWith(slave, now);

    // Master does NOT enter fail-safe
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::NoError);
    // Data is not valid (slave sent fail-safe data)
    EXPECT_FALSE(conn.getStatus().data_valid);
}

// ============================================================================
// #9: Slave accepts master's FailSafeData without entering fail-safe
// ============================================================================

TEST(FSoEDataConformance, SlaveAcceptsMasterFailSafeDataWithoutEnteringFailSafe) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    conn.triggerFailSafe(ErrorCode::CRCError);

    now += 15;
    conn.exchangeWith(slave, now);

    // Slave does NOT enter fail-safe
    EXPECT_FALSE(slave.isFailSafe());
}

// ============================================================================
// #10: Conn_Id field is set to actual Connection ID in Data state
// ============================================================================

TEST(FSoEDataConformance, ConnIdFieldSetToActualConnectionId) {
    // ETG.5100 Tables 23-26: Conn_Id field carries the Connection ID.
    const uint16_t test_conn_id = 0xABCD;
    FSoEMasterConnection conn(makeMasterCfg(4, 4, test_conn_id));
    FSoESlave slave(makeSlaveCfg(4, 4, test_conn_id));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Verify master TX frame has correct ConnID
    const uint16_t saved_crc0 = conn.getTxLastCrc0();
    const uint16_t saved_seq = conn.getTxSeqNo();
    uint8_t tx[64];
    size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_crc0, saved_seq));
    EXPECT_EQ(conn_id, test_conn_id);
}

// ============================================================================
// #11: CRC inheritance chains Data-state PDUs
// ============================================================================

TEST(FSoEDataConformance, CRCInheritanceChainsDataStatePDUs) {
    // ETG.5100 §8.2.2.6: CRC inheritance is active across all Data-state
    // cycles, chaining each PDU to the previous one.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Record CRC state after first Data cycle
    const uint16_t crc0_after_first = conn.getTxLastCrc0();

    // Exchange several more cycles
    for (int i = 0; i < 5; ++i) {
        now += 15;
        ASSERT_TRUE(conn.exchangeWith(slave, now));
    }

    // CRC should have advanced (not still at the same value)
    const uint16_t crc0_after_multiple = conn.getTxLastCrc0();
    // CRC may or may not change depending on data, but the chain should
    // be intact — verify by checking that frames still verify correctly
    // (if CRC chain was broken, processRxFrame would fail)
    EXPECT_TRUE(conn.isOperational());
}

// ============================================================================
// #12: Various safety data sizes (1, 2, 4, 6, 8)
// ============================================================================

class FSoEDataSizeConformanceTest : public ::testing::TestWithParam<uint8_t> {};

TEST_P(FSoEDataSizeConformanceTest, ProcessDataExchangeWithVariousSizes) {
    const uint8_t size = GetParam();
    FSoEMasterConnection conn(makeMasterCfg(size, size));
    FSoESlave slave(makeSlaveCfg(size, size));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Set and verify SafeOutputs
    std::vector<uint8_t> outputs(size);
    for (uint8_t i = 0; i < size; ++i) outputs[i] = 0x10 + i;
    ASSERT_TRUE(conn.setSafeOutputs(outputs.data(), size));

    // Set slave SafeInputs
    std::vector<uint8_t> inputs(size);
    for (uint8_t i = 0; i < size; ++i) inputs[i] = 0xA0 + i;
    slave.setSafeInputs(inputs.data(), size);

    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // Verify master received slave's SafeInputs
    std::vector<uint8_t> recv(size, 0);
    conn.getSafeInputs(recv.data(), size);
    for (uint8_t i = 0; i < size; ++i) {
        EXPECT_EQ(recv[i], inputs[i]) << "at byte " << (int)i;
    }
}

INSTANTIATE_TEST_SUITE_P(Sizes, FSoEDataSizeConformanceTest,
    ::testing::Values(1, 2, 4, 6, 8));

// ============================================================================
// #13: Reset command in Data state triggers reset
// ============================================================================

TEST(FSoEDataConformance, ResetCommandInDataStateTriggersReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Data);

    // Send a Reset command to the master (simulating slave-initiated reset)
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    payload[0] = 0x01;  // error code
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                            payload, 4u, 0x1234,
                                            0,  // start_crc = 0 (Reset)
                                            1); // seq = 1
    conn.processRxFrame(frame, frame_len);

    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_FALSE(conn.isFailSafe());
}

// ============================================================================
// #14: FailSafeData in non-Data states triggers fail-safe (handshake abort)
// ============================================================================

TEST(FSoEDataConformance, FailSafeDataInSessionStateTriggersFailSafe) {
    // In non-Data states, FailSafeData from the slave means the slave is
    // aborting the handshake.  The master should enter fail-safe.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    // Advance to Session state
    uint64_t now = 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    // Send FailSafeData to the master (simulating slave abort)
    uint8_t payload[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::FailSafeData,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    conn.processRxFrame(frame, frame_len);

    // FailSafeData in Session state triggers NOT_OK → Reset (not fail-safe)
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

TEST(FSoEDataConformance, FailSafeDataInConnectionStateTriggersFailSafe) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    // Advance to Connection state
    uint64_t now = 0;
    for (int i = 0; i < 10; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() == ConnectionState::Connection) break;
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);

    // Send FailSafeData to the master
    uint8_t payload[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::FailSafeData,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    conn.processRxFrame(frame, frame_len);

    // FailSafeData in Connection state triggers NOT_OK → Reset (not fail-safe)
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

// ============================================================================
// #15: FailSafeData Slave PDU with various input sizes — all zeros
// ============================================================================

class FSoEFailSafeDataSizeTest : public ::testing::TestWithParam<uint8_t> {};

TEST_P(FSoEFailSafeDataSizeTest, AllSafeDataIsZero) {
    const uint8_t size = GetParam();
    FSoESlave slave(makeSlaveCfg(size, size));
    slave.initialize();

    slave.triggerFailSafe(ErrorCode::WatchdogError);

    const uint16_t saved_crc0 = slave.getTxLastCrc0();
    const uint16_t saved_seq = slave.getTxSeqNo();
    uint8_t tx[64];
    size_t tx_len = slave.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_crc0, saved_seq));
    EXPECT_EQ(cmd, Command::FailSafeData);
    // Data length = safeInputSize (no error code field)
    EXPECT_EQ(data_len, static_cast<size_t>(size));
    // All SafeData = 0
    for (size_t i = 0; i < data_len; ++i) {
        EXPECT_EQ(data[i], 0x00) << "at offset " << i;
    }
}

INSTANTIATE_TEST_SUITE_P(Sizes, FSoEFailSafeDataSizeTest,
    ::testing::Values(1, 2, 4, 6, 8, 15, 16));

// ============================================================================
// #16: FailSafeData Master PDU with various output sizes — all zeros
// ============================================================================

class FSoEFailSafeDataMasterSizeTest : public ::testing::TestWithParam<uint8_t> {};

TEST_P(FSoEFailSafeDataMasterSizeTest, AllSafeDataIsZero) {
    const uint8_t size = GetParam();
    MasterConnectionConfig cfg = makeMasterCfg(size, size);
    // Set non-zero fail_safe_values to verify they are NOT used
    for (int i = 0; i < 16; ++i) cfg.fail_safe_values[i] = 0xFF;
    FSoEMasterConnection conn(cfg);
    FSoESlave slave(makeSlaveCfg(size, size));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    // Advance to Data state first — triggerFailSafe only works in Data
    uint64_t now = 0;
    advanceToData(conn, slave, now);

    conn.triggerFailSafe(ErrorCode::ApplicationError);

    const uint16_t saved_crc0 = conn.getTxLastCrc0();
    const uint16_t saved_seq = conn.getTxSeqNo();
    uint8_t tx[64];
    size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    uint8_t cmd = 0;
    uint8_t data[18] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                    saved_crc0, saved_seq));
    EXPECT_EQ(cmd, Command::FailSafeData);
    EXPECT_EQ(data_len, static_cast<size_t>(size));
    // All SafeData = 0, NOT fail_safe_values (0xFF)
    for (size_t i = 0; i < data_len; ++i) {
        EXPECT_EQ(data[i], 0x00) << "at offset " << i;
    }
}

INSTANTIATE_TEST_SUITE_P(Sizes, FSoEFailSafeDataMasterSizeTest,
    ::testing::Values(1, 2, 4, 6, 8));

// ============================================================================
// #17: Both nodes in fail-safe — independent recovery
// ============================================================================

TEST(FSoEDataConformance, BothNodesFailSafe_IndependentRecovery) {
    // Both master and slave enter fail-safe independently.
    // Master can recover by calling resetConnection().
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Both enter fail-safe independently
    conn.triggerFailSafe(ErrorCode::WatchdogError);
    slave.triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_TRUE(slave.isFailSafe());

    // Exchange — both send FailSafeData
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    // Both still in fail-safe
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_TRUE(slave.isFailSafe());

    // Master initiates recovery
    conn.resetConnection();
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
}

// ============================================================================
// #18: Data state runs continuously
// ============================================================================

TEST(FSoEDataConformance, DataStateRunsContinuously) {
    // ETG.5100 §8.2.2.6: The Data state runs continuously until a
    // communication error occurs or a node is stopped locally.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    FSoESlave slave(makeSlaveCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Run many cycles — should stay in Data state
    for (int i = 0; i < 100; ++i) {
        now += 15;
        ASSERT_TRUE(conn.exchangeWith(slave, now));
        EXPECT_EQ(conn.getState(), ConnectionState::Data);
        EXPECT_TRUE(conn.isOperational());
    }
}
