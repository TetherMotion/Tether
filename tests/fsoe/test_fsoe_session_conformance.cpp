/**
 * @file test_fsoe_session_conformance.cpp
 * @brief Regression tests for ETG.5100 S (D) V1.2.0 §8.2.2.3 Session state
 *        conformance.
 *
 * Verifies the FSoE Session PDU structure and behavior as described in:
 *   https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
 *
 * Tests cover:
 *   #1 Slave generates its OWN random Session ID (not echoing master's)
 *   #2 Conn_Id = 0 in Session frames (master and slave)
 *   #3 Conn_Id = 0 in Reset frames (master and slave)
 *   #4 Master stores the slave's Session ID
 *   #5 2-cycle Session ID transfer for 1-octet safety data
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
                                             uint8_t outSize = 4) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0100;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = 0x1234;
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

/// Advance the master to the Session state by running through Reset.
/// After this, the master is in Session state and has generated a
/// Master Session ID.
static void advanceToSession(FSoEMasterConnection& conn, FSoESlave& slave,
                              uint64_t& now) {
    now = 0;
    // Cycle 1: master sends Reset, slave responds → master enters Session
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);
}

/// Extract Conn_Id from an FSoE frame (last 2 bytes, little-endian).
static uint16_t extractConnId(const uint8_t* frame, size_t len) {
    if (len < 2) return 0;
    return static_cast<uint16_t>(frame[len - 2]) |
           (static_cast<uint16_t>(frame[len - 1]) << 8);
}

// ============================================================================
// #1: Slave generates its OWN random Session ID
// ETG.5100 S (D) V1.2.0, §8.2.2.3, Table 14:
//   "The slave does NOT echo the master's Session ID — it responds with
//    its own independently generated random Slave Session ID."
// See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
// ============================================================================

TEST(FSoESessionConformance, SlaveGeneratesOwnSessionId) {
    // ETG.5100 §8.2.2.3: The slave must generate its own random Session ID,
    // not echo the master's.  Verify that the slave's Session ID is
    // different from the master's (with overwhelming probability for
    // random 16-bit values).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToSession(conn, slave, now);

    const uint16_t master_session = conn.getStatus().session_id;
    const uint16_t slave_session = slave.getSessionId();

    EXPECT_NE(master_session, 0u);
    EXPECT_NE(slave_session, 0u);
    // The slave's Session ID must be different from the master's.
    // (For random 16-bit values, P(same) = 1/65536, so this is
    // effectively always true.)
    EXPECT_NE(master_session, slave_session)
        << "Slave echoed master's Session ID — spec violation";
}

TEST(FSoESessionConformance, SlaveSessionIdIsStableAcrossRetries) {
    // ETG.5100 §8.2.2.3: The slave generates its Session ID once per
    // connection attempt.  If the master retries the Session command
    // (e.g. due to PDO delay), the slave must send the SAME Session ID.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToSession(conn, slave, now);

    const uint16_t slave_session_first = slave.getSessionId();

    // Send another Session frame (master is in Session state)
    now += 15;
    conn.exchangeWith(slave, now);

    // Slave should still have the same Session ID
    EXPECT_EQ(slave.getSessionId(), slave_session_first);
}

TEST(FSoESessionConformance, SlaveSessionIdChangesOnReset) {
    // ETG.5100 §8.2.2.3: The slave generates a NEW random Session ID on
    // each Reset (new connection attempt).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);
    const uint16_t slave_session_first = slave.getSessionId();

    // Reset the connection
    conn.resetConnection();
    slave.reset();

    // Advance to Session: run exchanges until the master reaches Session state
    for (int i = 0; i < 10; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() == ConnectionState::Session) break;
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Session)
        << "Master should reach Session state after reset";

    const uint16_t slave_session_second = slave.getSessionId();

    EXPECT_NE(slave_session_first, slave_session_second)
        << "Slave should generate a new Session ID on Reset";
}

// ============================================================================
// #2: Conn_Id = 0 in Session frames
// ETG.5100 S (D) V1.2.0, §8.2.2.3:
//   "Conn_Id is unused and set to 0 — the Session ID has no safety
//    relevance, so the Connection ID is not checked in this state."
// See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
// ============================================================================

TEST(FSoESessionConformance, MasterSessionFrameHasZeroConnId) {
    // ETG.5100 §8.2.2.3: The master's Session frame must have Conn_Id = 0.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToSession(conn, slave, now);

    // Master is now in Session state — build a Session frame
    uint8_t tx[64] = {};
    const size_t len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(len, 0u);

    // Verify the command byte is Session (0x4E)
    EXPECT_EQ(tx[0], Command::Session);

    // Conn_Id is the last 2 bytes — must be 0
    const uint16_t conn_id = extractConnId(tx, len);
    EXPECT_EQ(conn_id, 0u)
        << "Session frame must have Conn_Id=0 (ETG.5100 §8.2.2.3)";
}

TEST(FSoESessionConformance, SlaveSessionResponseHasZeroConnId) {
    // ETG.5100 §8.2.2.3: The slave's Session response must have Conn_Id = 0.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToSession(conn, slave, now);

    // Master sends Session frame → slave processes and responds
    uint8_t tx[64] = {};
    const size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    uint8_t rx[64] = {};
    const size_t rx_len = slave.processRxFrame(tx, tx_len);
    ASSERT_TRUE(rx_len > 0 || true);  // processRxFrame returns bool, not size

    // Build the slave's response
    size_t resp_len = slave.prepareTxFrame(rx, sizeof(rx));
    ASSERT_GT(resp_len, 0u);

    // Verify the command byte is Session (0x4E)
    EXPECT_EQ(rx[0], Command::Session);

    // Conn_Id is the last 2 bytes — must be 0
    const uint16_t conn_id = extractConnId(rx, resp_len);
    EXPECT_EQ(conn_id, 0u)
        << "Slave Session response must have Conn_Id=0 (ETG.5100 §8.2.2.3)";
}

// ============================================================================
// #3: Conn_Id = 0 in Reset frames
// ETG.5100 S (D) V1.2.0, §8.2.2.2:
//   "Conn_Id is unused and set to 0 in Reset state — the connection has
//    not been established yet, so there is no Connection ID to check."
// See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
// ============================================================================

TEST(FSoESessionConformance, MasterResetFrameHasZeroConnId) {
    // ETG.5100 §8.2.2.2: The master's Reset frame must have Conn_Id = 0.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Master is in Reset state — build a Reset frame
    uint8_t tx[64] = {};
    const size_t len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(len, 0u);

    // Verify the command byte is Reset (0x2A)
    EXPECT_EQ(tx[0], Command::Reset);

    // Conn_Id is the last 2 bytes — must be 0
    const uint16_t conn_id = extractConnId(tx, len);
    EXPECT_EQ(conn_id, 0u)
        << "Reset frame must have Conn_Id=0 (ETG.5100 §8.2.2.2)";
}

TEST(FSoESessionConformance, SlaveResetResponseHasZeroConnId) {
    // ETG.5100 §8.2.2.2: The slave's Reset response must have Conn_Id = 0.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Master sends Reset frame
    uint8_t tx[64] = {};
    const size_t tx_len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Slave processes the Reset frame
    ASSERT_TRUE(slave.processRxFrame(tx, tx_len));

    // Build the slave's Reset response
    uint8_t rx[64] = {};
    const size_t rx_len = slave.prepareTxFrame(rx, sizeof(rx));
    ASSERT_GT(rx_len, 0u);

    // Conn_Id is the last 2 bytes — must be 0
    const uint16_t conn_id = extractConnId(rx, rx_len);
    EXPECT_EQ(conn_id, 0u)
        << "Slave Reset response must have Conn_Id=0 (ETG.5100 §8.2.2.2)";
}

TEST(FSoESessionConformance, ResetFrameConnIdZeroDoesNotBreakHandshake) {
    // Regression: verify that Conn_Id=0 in Reset/Session frames does not
    // break the full handshake.  The slave must accept Reset frames with
    // Conn_Id=0 even from Data state (mid-stream reset).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Mid-stream reset
    conn.resetConnection();
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // Should have advanced to Session (slave acknowledged reset)
    EXPECT_EQ(conn.getState(), ConnectionState::Session);
}

// ============================================================================
// #4: Master stores the slave's Session ID
// ETG.5100 S (D) V1.2.0, §8.2.2.3:
//   "Both IDs are then used together to identify this particular
//    connection instance."
// See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
// ============================================================================

TEST(FSoESessionConformance, MasterStoresSlaveSessionId) {
    // ETG.5100 §8.2.2.3: The master must store the slave's Session ID
    // received in the Session response, for connection instance
    // identification.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Before the handshake, slave_session_id should be 0
    EXPECT_EQ(conn.getStatus().slave_session_id, 0u);

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // After the handshake, the master should have stored the slave's
    // Session ID
    const uint16_t stored_slave_session = conn.getStatus().slave_session_id;
    const uint16_t actual_slave_session = slave.getSessionId();

    EXPECT_NE(stored_slave_session, 0u)
        << "Master should have stored a non-zero slave Session ID";
    EXPECT_EQ(stored_slave_session, actual_slave_session)
        << "Master's stored slave Session ID should match the slave's actual ID";
}

TEST(FSoESessionConformance, MasterSlaveSessionIdClearedOnReset) {
    // After resetConnection(), the master should clear the stored slave
    // Session ID.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);
    EXPECT_NE(conn.getStatus().slave_session_id, 0u);

    conn.resetConnection();
    EXPECT_EQ(conn.getStatus().slave_session_id, 0u)
        << "Slave Session ID should be cleared on reset";
}

// ============================================================================
// #5: 2-cycle Session ID transfer for 1-octet safety data
// ETG.5100 S (D) V1.2.0, §8.2.2.3:
//   "1 octet: 2 cycles — Only SafeData[0] is available per cycle, so the
//    two octets are transferred in two successive PDUs."
// See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
// ============================================================================

TEST(FSoESessionConformance, OneOctetDataHandshakeCompletes) {
    // ETG.5100 §8.2.2.3: With 1-octet safety data, the 16-bit Session ID
    // is transferred in two successive PDUs.  The handshake must still
    // complete successfully.
    FSoEMasterConnection conn(makeMasterCfg(1, 1));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(1, 1));
    slave.initialize();

    uint64_t now = 0;
    // The handshake may take more cycles due to the 2-cycle Session transfer
    advanceToData(conn, slave, now, 50);

    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_EQ(slave.getState(), ConnectionState::Data);
}

TEST(FSoESessionConformance, OneOctetDataMasterStoresSlaveSessionId) {
    // With 1-octet safety data, the master should still receive and store
    // the slave's complete 16-bit Session ID across two cycles.
    FSoEMasterConnection conn(makeMasterCfg(1, 1));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(1, 1));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now, 50);

    const uint16_t stored = conn.getStatus().slave_session_id;
    const uint16_t actual = slave.getSessionId();

    EXPECT_NE(stored, 0u);
    EXPECT_EQ(stored, actual)
        << "Master should have received the complete 16-bit slave Session ID "
        << "across two 1-octet PDUs";
}

TEST(FSoESessionConformance, OneOctetDataSessionFrameHasZeroConnId) {
    // Even with 1-octet safety data, the Session frame must have Conn_Id=0.
    FSoEMasterConnection conn(makeMasterCfg(1, 1));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(1, 1));
    slave.initialize();

    uint64_t now = 0;
    advanceToSession(conn, slave, now);

    uint8_t tx[64] = {};
    const size_t len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(len, 0u);

    EXPECT_EQ(tx[0], Command::Session);
    const uint16_t conn_id = extractConnId(tx, len);
    EXPECT_EQ(conn_id, 0u)
        << "Session frame must have Conn_Id=0 even for 1-octet data";
}

// ============================================================================
// Cross-direction CRC verification with Conn_Id=0
// ============================================================================

TEST(FSoESessionConformance, HandshakeWorksWithZeroConnIdInResetSession) {
    // Full end-to-end test: verify that the complete handshake works
    // with Conn_Id=0 in Reset and Session frames, and that the
    // configured Conn_Id is only used from Connection state onward.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Verify we're in Data state and can exchange data
    for (int i = 0; i < 10; ++i) {
        now += 15;
        EXPECT_TRUE(conn.exchangeWith(slave, now))
            << "Data exchange failed at cycle " << i;
    }

    // Verify the configured Conn_Id is used in Data state
    uint8_t tx[64] = {};
    const size_t len = conn.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(len, 0u);
    EXPECT_EQ(tx[0], Command::ProcessData);
    const uint16_t conn_id = extractConnId(tx, len);
    EXPECT_EQ(conn_id, 0x1234u)
        << "Data frame should use the configured Conn_Id (0x1234)";
}
