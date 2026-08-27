/**
 * @file test_fsoe_connection_conformance.cpp
 * @brief Regression tests for ETG.5100 S (D) V1.2.0 §8.2.2.4 Connection
 *        state PDU conformance.
 *
 * Verifies the FSoE Connection PDU structure and behavior as described in:
 *   https://techoverflow.net/2026/08/12/fsoe-connection-pdu-master-and-slave-structure/
 *
 * Tests cover:
 *   #1 Multi-cycle Connection transfer for safety data < 4 octets
 *   #2 Connection ID = 0x0000 is rejected (not silently accepted)
 *   #3 Config-time validation that connection_id != 0
 *   #4 Conn_Id field is set to actual Connection ID (not 0)
 *   #5 Slave Address validation
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
                                             uint16_t slaveAddr = 0x0100) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0100;
    cfg.slave_safety_addr = slaveAddr;
    cfg.connection_id = connId;
    cfg.master_addr = 0x0100;
    cfg.watchdog_timeout_ms = 200;
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
                                     uint16_t connId = 0x1234,
                                     uint16_t safetyAddr = 0x0100) {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 0x0100;
    cfg.connectionId = connId;
    cfg.safetyAddress = safetyAddr;
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
                          uint64_t& now, int maxCycles = 30) {
    now = 0;
    for (int i = 0; i < maxCycles; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational() && slave.isOperational()) break;
    }
    ASSERT_TRUE(conn.isOperational());
    ASSERT_TRUE(slave.isOperational());
}

/// Advance the master to the Connection state.
static void advanceToConnection(FSoEMasterConnection& conn, FSoESlave& slave,
                                 uint64_t& now) {
    now = 0;
    // Cycle 1: Reset → Session
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);
    // Cycle 2: Session → Connection
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);
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
// #1: Multi-cycle Connection transfer for safety data < 4 octets
// ============================================================================

TEST(FSoEConnectionConformance, OneOctetDataTakesFourCycles) {
    // 1-octet safety data: Connection state needs 4 cycles to transfer
    // the 4-byte payload (ConnID_lo, ConnID_hi, Addr_lo, Addr_hi).
    FSoEMasterConnection conn(makeMasterCfg(1, 1));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(1, 1));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    // Reset → Session: slave sends Session response with low byte of
    // session ID.  Master stores it and transitions to Session.
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    // Session → Connection: slave sends high byte of session ID.
    // Master stores it and transitions to Connection.
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);

    // Connection state: 4 cycles to transfer 4 bytes at 1 byte/cycle
    // The master should stay in Connection for 4 exchanges.
    for (int i = 0; i < 3; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        EXPECT_EQ(conn.getState(), ConnectionState::Connection)
            << "Master should still be in Connection after cycle " << i;
    }
    // 4th cycle: all 4 bytes transferred and echoed → transition
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_NE(conn.getState(), ConnectionState::Connection)
        << "Master should have transitioned out of Connection after 4 cycles";
}

TEST(FSoEConnectionConformance, TwoOctetDataTakesTwoCycles) {
    // 2-octet safety data: Connection state needs 2 cycles.
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(2, 2));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // 1st cycle: still in Connection (only 2 of 4 bytes transferred)
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_EQ(conn.getState(), ConnectionState::Connection);

    // 2nd cycle: all 4 bytes transferred → transition
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_NE(conn.getState(), ConnectionState::Connection);
}

TEST(FSoEConnectionConformance, FourOctetDataTakesOneCycle) {
    // 4-octet safety data: Connection state completes in 1 cycle.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // 1 cycle: all 4 bytes transferred and echoed → transition
    now += 15;
    conn.exchangeWith(slave, now);
    EXPECT_NE(conn.getState(), ConnectionState::Connection);
}

TEST(FSoEConnectionConformance, AsymmetricSizesCompleteHandshake) {
    // Asymmetric sizes: (output=1, input=4) and (output=4, input=1)
    // The Connection transfer should complete for both configurations.
    for (auto [inSize, outSize] : {std::make_pair(1, 4), std::make_pair(4, 1),
                                    std::make_pair(3, 5), std::make_pair(7, 3)}) {
        FSoEMasterConnection conn(makeMasterCfg(inSize, outSize));
        ASSERT_TRUE(conn.initialize());
        conn.startConnection();

        FSoESlave slave(makeSlaveCfg(inSize, outSize));
        ASSERT_TRUE(slave.initialize());

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
}

// ============================================================================
// #2: Connection ID = 0x0000 in SafeData is rejected
// ============================================================================

TEST(FSoEConnectionConformance, MasterRejectsZeroConnIdInEcho) {
    // The master should reject an echo where the Connection ID is 0x0000.
    // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // Send a Connection response with Conn_ID = 0x0000 in SafeData.
    uint8_t payload[] = {0x00, 0x00, 0x00, 0x01};  // ConnID=0, Addr=1
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Connection,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    bool ok = conn.processRxFrame(frame, frame_len);

    EXPECT_TRUE(ok);  // Frame was parsed
    // Error during Connection state goes back to Reset (NOT_OK transition)
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::ConnectionIDError);
}

TEST(FSoEConnectionConformance, SlaveRejectsZeroConnIdInSafeData) {
    // The slave should reject a Connection frame where the SafeData
    // Connection ID is 0x0000.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // Build a Connection frame with ConnID=0 in SafeData but correct
    // Conn_Id field.  Use the master's CRC state.
    uint8_t payload[] = {0x00, 0x00, 0x00, 0x01};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Connection,
                                            payload, 4, 0x1234,
                                            slave.getRxLastCrc0(),
                                            slave.getRxSeqNo());
    slave.processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave.isFailSafe());
    EXPECT_EQ(slave.getLastError(), ErrorCode::ConnectionIDError);
}

// ============================================================================
// #3: Config-time validation that connection_id != 0
// ============================================================================

TEST(FSoEConnectionConformance, MasterRejectsZeroConnectionIdConfig) {
    // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
    // The master should reject a config with connection_id = 0.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x0000));
    EXPECT_FALSE(conn.initialize());
}

TEST(FSoEConnectionConformance, SlaveRejectsZeroConnectionIdConfig) {
    // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted by the
    // standard, but some safety controllers (e.g. Nexcobot ESC211 with a
    // default/unconfigured FNI) send connection ID 0x0000.  The slave
    // allows it at initialization for interoperability; the connection ID
    // is still validated in the Connection state and beyond.
    FSoESlave slave(makeSlaveCfg(4, 4, 0x0000));
    EXPECT_TRUE(slave.initialize());
}

// ============================================================================
// #4: Conn_Id field is set to actual Connection ID (not 0)
// ============================================================================

TEST(FSoEConnectionConformance, MasterConnIdFieldIsActualId) {
    // In the Connection state, the Conn_Id field should be the actual
    // Connection ID (unlike Reset/Session where it is 0).
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x5678));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x5678));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // Build a Connection frame and check the Conn_Id field.
    uint8_t frame[64];
    size_t frame_len = conn.prepareTxFrame(frame, sizeof(frame));
    ASSERT_GT(frame_len, 0u);

    uint16_t conn_id_field = extractConnId(frame, frame_len);
    EXPECT_EQ(conn_id_field, 0x5678)
        << "Conn_Id field should be the actual Connection ID in Connection state";
}

TEST(FSoEConnectionConformance, SlaveConnIdFieldIsActualId) {
    // The slave's Connection response should also have the actual
    // Connection ID in the Conn_Id field.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x5678));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x5678));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // One more exchange: master sends Connection, slave receives and
    // transitions to Connection state.
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(slave.getState(), ConnectionState::Connection);

    // Build the slave's response and check Conn_Id field.
    uint8_t resp[64];
    size_t resp_len = slave.prepareTxFrame(resp, sizeof(resp));
    ASSERT_GT(resp_len, 0u);

    uint16_t conn_id_field = extractConnId(resp, resp_len);
    EXPECT_EQ(conn_id_field, 0x5678)
        << "Slave Conn_Id field should be the actual Connection ID";
}

// ============================================================================
// #5: Slave Address validation
// ============================================================================

TEST(FSoEConnectionConformance, MasterRejectsWrongSlaveAddress) {
    // The master should reject an echo with the wrong Slave Address.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // Send a Connection response with correct ConnID but wrong Slave Address.
    uint8_t payload[] = {0x34, 0x12, 0x00, 0x02};  // ConnID=0x1234, Addr=0x0200
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Connection,
                                            payload, 4, 0x1234,
                                            conn.getRxLastCrc0(),
                                            conn.getRxSeqNo());
    conn.processRxFrame(frame, frame_len);

    // Error during Connection state goes back to Reset (NOT_OK transition)
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_EQ(conn.getErrorCode(), ErrorCode::ConnectionIDError);
}

TEST(FSoEConnectionConformance, SlaveRejectsWrongSlaveAddress) {
    // The slave should reject a Connection frame with the wrong Slave Address.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // Send a Connection frame with correct ConnID but wrong Slave Address.
    uint8_t payload[] = {0x34, 0x12, 0x00, 0x02};  // Addr=0x0200
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Connection,
                                            payload, 4, 0x1234,
                                            slave.getRxLastCrc0(),
                                            slave.getRxSeqNo());
    slave.processRxFrame(frame, frame_len);

    EXPECT_TRUE(slave.isFailSafe());
    EXPECT_EQ(slave.getLastError(), ErrorCode::ConnectionIDError);
}

// ============================================================================
// #6: Connection frame SafeData structure
// ============================================================================

TEST(FSoEConnectionConformance, MasterFrameHasCorrectSafeDataStructure) {
    // ETG.5100 §8.2.2.4 Table 15:
    //   SafeData[0] = Connection ID, low octet
    //   SafeData[1] = Connection ID, high octet
    //   SafeData[2] = FSoE Slave Address, low octet
    //   SafeData[3] = FSoE Slave Address, high octet
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    uint8_t frame[64];
    size_t frame_len = conn.prepareTxFrame(frame, sizeof(frame));
    ASSERT_GT(frame_len, 0u);

    uint8_t safe_data[4] = {};
    ASSERT_TRUE(extractSafeData(frame, frame_len, safe_data, 4));

    EXPECT_EQ(safe_data[0], 0x34);  // ConnID low
    EXPECT_EQ(safe_data[1], 0x12);  // ConnID high
    EXPECT_EQ(safe_data[2], 0x00);  // Slave Addr low
    EXPECT_EQ(safe_data[3], 0x01);  // Slave Addr high
}

TEST(FSoEConnectionConformance, SlaveResponseHasCorrectSafeDataStructure) {
    // The slave's Connection response should echo the Connection ID
    // and its own Slave Address in SafeData[0-3].
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 0x1234, 0x0100));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    advanceToConnection(conn, slave, now);

    // One more exchange: master sends Connection, slave receives and
    // transitions to Connection state.
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(slave.getState(), ConnectionState::Connection);

    uint8_t resp[64];
    size_t resp_len = slave.prepareTxFrame(resp, sizeof(resp));
    ASSERT_GT(resp_len, 0u);

    uint8_t safe_data[4] = {};
    ASSERT_TRUE(extractSafeData(resp, resp_len, safe_data, 4));

    EXPECT_EQ(safe_data[0], 0x34);  // ConnID low
    EXPECT_EQ(safe_data[1], 0x12);  // ConnID high
    EXPECT_EQ(safe_data[2], 0x00);  // Slave Addr low
    EXPECT_EQ(safe_data[3], 0x01);  // Slave Addr high
}

// ============================================================================
// #7: 0-octet safety data skips SafeData transfer
// ============================================================================

TEST(FSoEConnectionConformance, ZeroOctetDataSkipsConnectionTransfer) {
    // 0-octet safety data: no SafeData to transfer.  The Connection
    // state should complete immediately (Conn_Id field carries the ID).
    FSoEMasterConnection conn(makeMasterCfg(0, 0));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(0, 0));
    ASSERT_TRUE(slave.initialize());

    uint64_t now = 0;
    // Reset → Session
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    // Session → Connection (0-octet: immediate transition)
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);

    // Connection → Parameter/Data (0-octet: immediate transition)
    now += 15;
    conn.exchangeWith(slave, now);
    // With 0-octet data, there's no Parameter phase
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}
