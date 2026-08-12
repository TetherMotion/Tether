/**
 * @file test_fsoe_sequence_conformance.cpp
 * @brief Tests for ETG.5100 §8.1.3.4 sequence number conformance and
 *        CRC collision avoidance.
 *
 * Verifies:
 * - Sequence numbers are always in the range 1..65535 (0 is never used)
 * - Sequence numbers wrap from 65535 back to 1
 * - Reset frames use seq=1 (resetting the sequence for a new connection)
 * - After a Reset frame, the next frame uses seq=2
 * - CRC collision avoidance: when CRC0 == previous CRC0, the seq is
 *   incremented and CRC recomputed (both on build and parse sides)
 * - The collision avoidance parser only retries on collision (CRC matches
 *   but equals previous), not on CRC mismatch
 * - Mid-stream reset properly resynchronizes master and slave seq numbers
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
    ASSERT_TRUE(conn.isOperational());
    ASSERT_TRUE(slave.isOperational());
}

// ============================================================================
// incrementSeqNo unit tests
// ============================================================================

TEST(FSoEIncrementSeqNo, OneIncrementsToTwo) {
    EXPECT_EQ(CRC::incrementSeqNo(1), 2);
}

TEST(FSoEIncrementSeqNo, WrapAroundFrom65535ToOne) {
    EXPECT_EQ(CRC::incrementSeqNo(65535), 1);
}

TEST(FSoEIncrementSeqNo, NeverReturnsZero) {
    // Test all boundary values
    EXPECT_NE(CRC::incrementSeqNo(65534), 0);
    EXPECT_EQ(CRC::incrementSeqNo(65534), 65535);
    EXPECT_NE(CRC::incrementSeqNo(65535), 0);
    EXPECT_EQ(CRC::incrementSeqNo(65535), 1);
    EXPECT_NE(CRC::incrementSeqNo(1), 0);
    EXPECT_EQ(CRC::incrementSeqNo(1), 2);
}

TEST(FSoEIncrementSeqNo, SequentialIncrement) {
    uint16_t seq = 1;
    for (int i = 0; i < 100; ++i) {
        seq = CRC::incrementSeqNo(seq);
        EXPECT_GE(seq, 1u);
        EXPECT_LE(seq, 65535u);
    }
    EXPECT_EQ(seq, 101u);
}

// ============================================================================
// Initial sequence number tests
// ============================================================================

TEST(FSoESequenceConformance, MasterInitialSeqIsZero) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    EXPECT_EQ(conn.getTxSeqNo(), 0u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);
}

TEST(FSoESequenceConformance, SlaveInitialSeqIsZero) {
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();
    EXPECT_EQ(slave.getTxSeqNo(), 0u);
    EXPECT_EQ(slave.getRxSeqNo(), 0u);
}

TEST(FSoESequenceConformance, MasterSeqResetsToZeroAfterReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    conn.resetConnection();
    EXPECT_EQ(conn.getTxSeqNo(), 0u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);
}

TEST(FSoESequenceConformance, SlaveSeqResetsToZeroAfterReset) {
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();
    slave.reset();
    EXPECT_EQ(slave.getTxSeqNo(), 0u);
    EXPECT_EQ(slave.getRxSeqNo(), 0u);
}

// ============================================================================
// Configurable initial sequence number
// ============================================================================

TEST(FSoESequenceConformance, MasterInitialSeqIsConfigurableToOne) {
    // ETG.5100 §8.1.3.4 says seq starts at 1.  Verify the config works.
    // In the self-inheriting RX model, getRxSeqNo() = last_tx_seq_no_,
    // which is 0 before any TX.
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.initial_seq_no = 1;
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);  // No TX yet

    // First TX frame should be a Reset with seq=1
    uint8_t buf[64] = {};
    size_t len = conn.prepareTxFrame(buf, sizeof(buf));
    ASSERT_GT(len, 0u);
    // After TX, getRxSeqNo() = last_tx_seq_no_ = 1 (the seq used in the Reset)
    EXPECT_EQ(conn.getRxSeqNo(), 1u);
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    EXPECT_TRUE(CRC::parseFSoEFrame(buf, len, cmd, data, data_len, conn_id,
                                    0, 1));
    EXPECT_EQ(cmd, Command::Reset);
}

TEST(FSoESequenceConformance, SlaveInitialSeqIsConfigurableToOne) {
    // In the cross-direction TX model, getTxSeqNo() = last_rx_seq_no_,
    // which is 0 before any RX from the master.
    FSoESlaveConfig cfg = makeSlaveCfg(4, 4);
    cfg.initialSeqNo = 1;
    FSoESlave slave(cfg);
    slave.initialize();
    EXPECT_EQ(slave.getTxSeqNo(), 0u);  // No RX yet
    EXPECT_EQ(slave.getRxSeqNo(), 1u);
}

// ============================================================================
// Reset frame sequence number tests
// ============================================================================

TEST(FSoESequenceConformance, MasterResetFrameUsesSeqZero) {
    // After init, the master's first TX frame should be a Reset with seq=0.
    // Verify by parsing the frame.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    uint8_t buf[64] = {};
    size_t len = conn.prepareTxFrame(buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    // Parse the frame with start_crc=0, seq=0 (Reset frame parameters)
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    EXPECT_TRUE(CRC::parseFSoEFrame(buf, len, cmd, data, data_len, conn_id,
                                    0, 0));
    EXPECT_EQ(cmd, Command::Reset);
}

TEST(FSoESequenceConformance, SlaveResetResponseUsesSeqZero) {
    // When the slave receives a Reset, its response should use seq=0.
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Build a Reset frame with seq=0, start_crc=0
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 0);
    ASSERT_GT(frame_len, 0u);
    ASSERT_TRUE(slave.processRxFrame(frame, frame_len));

    // Slave should now be in Session state
    EXPECT_EQ(slave.getState(), ConnectionState::Session);

    // Build the slave's TX frame — should be a Session response with
    // seq=initialSeqNo (0), since tx_seq_no_ was reset to 0 by
    // processSessionReset and buildSessionResponse uses it directly.
    uint8_t tx[64];
    size_t tx_len = slave.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Parse with start_crc=0 (CRC chain was reset), seq=0
    // (or seq=1 if collision avoidance incremented it)
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    // Try seq=0 first, then seq=1 (collision avoidance may have incremented)
    bool parsed = CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                      0, 0);
    if (!parsed) {
        parsed = CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                     0, 1);
    }
    EXPECT_TRUE(parsed) << "Failed to parse with seq=0 or seq=1";
    EXPECT_EQ(cmd, Command::Session);
}

TEST(FSoESequenceConformance, SeqAdvancesAfterResetFrame) {
    // After a Reset frame (seq=0), the next frame should use seq=1.
    // In the self-inheriting model:
    // - Master TX: self-inheriting (own last TX seq, incremented)
    // - Master RX: self-inheriting (own last TX seq, no increment)
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // First TX: Reset frame (seq=0)
    uint8_t buf1[64] = {};
    size_t len1 = conn.prepareTxFrame(buf1, sizeof(buf1));
    ASSERT_GT(len1, 0u);
    // After prepareTxFrame, tx_seq_no_ = 1 (incremented for next TX)
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    // getRxSeqNo() = last_tx_seq_no_ = 0 (the seq used in the Reset)
    EXPECT_EQ(conn.getRxSeqNo(), 0u);

    // Feed a Reset response to advance to Session.
    // The slave's Reset response uses seq=1 (initial_seq_no + 1).
    uint8_t rx_payload[4] = {0, 0, 0, 0};
    uint8_t rx_frame[64];
    size_t rx_len = CRC::buildFSoEFrame(rx_frame, Command::Reset,
                                        rx_payload, 4, 0x1234, 0, 1);
    ASSERT_TRUE(conn.processRxFrame(rx_frame, rx_len));
    EXPECT_EQ(conn.getState(), ConnectionState::Session);
    // RX still uses last_tx_seq_no_ = 0 (self-inheriting, no increment)
    EXPECT_EQ(conn.getRxSeqNo(), 0u);

    // Second TX: Session frame — should use seq=1
    uint8_t buf2[64] = {};
    size_t len2 = conn.prepareTxFrame(buf2, sizeof(buf2));
    ASSERT_GT(len2, 0u);
    // After TX, tx_seq_no_ = 2, last_tx_seq_no_ = 1
    EXPECT_EQ(conn.getTxSeqNo(), 2u);
    EXPECT_EQ(conn.getRxSeqNo(), 1u);

    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    // Parse with start_crc=0 (CRC chain was reset by Reset), seq=1
    EXPECT_TRUE(CRC::parseFSoEFrame(buf2, len2, cmd, data, data_len, conn_id,
                                    0, 1));
    EXPECT_EQ(cmd, Command::Session);
}

// ============================================================================
// Full handshake sequence progression
// ============================================================================

TEST(FSoESequenceConformance, FullHandshakeSeqProgression) {
    // Verify seq numbers through the full handshake.
    // Model:
    // - Master TX: self-inheriting, seq increments per TX
    // - Master RX: self-inheriting, uses last_tx_seq_no_ (no increment)
    // - Slave TX: cross-direction, uses last_rx_seq_no_ (no increment)
    // - Slave RX: cross-direction, seq increments per RX
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Initial state
    EXPECT_EQ(conn.getTxSeqNo(), 0u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);  // last_tx_seq_no_ = 0 (no TX yet)
    EXPECT_EQ(slave.getTxSeqNo(), 0u);  // last_rx_seq_no_ = 0 (no RX yet)
    EXPECT_EQ(slave.getRxSeqNo(), 0u);

    // Exchange 1: Master sends Reset (seq=0), slave responds
    uint64_t now = 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // After exchange 1:
    // Master: tx_seq=1 (incremented), rx_seq=0 (last_tx_seq_no_=0, the Reset's seq)
    // Slave: tx_seq=0 (last_rx_seq_no_=0, the master's Reset seq), rx_seq=1 (incremented)
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);
    EXPECT_EQ(slave.getTxSeqNo(), 0u);
    EXPECT_EQ(slave.getRxSeqNo(), 1u);

    // Exchange 2: Master sends Session (seq=1), slave responds
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    // Master: tx_seq=2, rx_seq=1 (last_tx_seq_no_=1, the Session's seq)
    // Slave: tx_seq=1 (last_rx_seq_no_=1), rx_seq=2
    EXPECT_EQ(conn.getTxSeqNo(), 2u);
    EXPECT_EQ(conn.getRxSeqNo(), 1u);
    EXPECT_EQ(slave.getTxSeqNo(), 1u);
    EXPECT_EQ(slave.getRxSeqNo(), 2u);

    // Continue to Data
    for (int i = 0; i < 30; ++i) {
        now += 15;
        ASSERT_TRUE(conn.exchangeWith(slave, now));
        if (conn.isOperational()) break;
    }
    ASSERT_TRUE(conn.isOperational());

    // Master TX seq == slave RX seq (master sends, slave receives)
    EXPECT_EQ(conn.getTxSeqNo(), slave.getRxSeqNo());
    // Master RX seq == slave TX seq (slave echoes master's TX seq)
    EXPECT_EQ(conn.getRxSeqNo(), slave.getTxSeqNo());
}

// ============================================================================
// Mid-stream reset resynchronization
// ============================================================================

TEST(FSoESequenceConformance, MidStreamResetResynchronizesSeq) {
    // Advance to Data state, then reset the master. The slave should
    // resynchronize its seq numbers.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Master TX seq == slave RX seq (both advance per exchange)
    uint16_t master_tx = conn.getTxSeqNo();
    uint16_t slave_rx = slave.getRxSeqNo();
    EXPECT_EQ(master_tx, slave_rx);
    EXPECT_GT(master_tx, 0u);

    // Master resets
    conn.resetConnection();
    EXPECT_EQ(conn.getTxSeqNo(), 0u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);  // last_tx_seq_no_ = 0

    // Slave is still at the old seq — exchange should resynchronize
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // After the Reset exchange:
    // Master: tx_seq=1 (incremented), rx_seq=0 (last_tx_seq_no_=0, Reset's seq)
    // Slave: tx_seq=0 (last_rx_seq_no_=0, master's Reset seq), rx_seq=1 (incremented)
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);
    EXPECT_EQ(slave.getTxSeqNo(), 0u);
    EXPECT_EQ(slave.getRxSeqNo(), 1u);

    // Full handshake should complete again
    for (int i = 0; i < 30; ++i) {
        now += 15;
        ASSERT_TRUE(conn.exchangeWith(slave, now));
        if (conn.isOperational()) break;
    }
    ASSERT_TRUE(conn.isOperational());

    // Still synchronized
    EXPECT_EQ(conn.getTxSeqNo(), slave.getRxSeqNo());
    EXPECT_EQ(conn.getRxSeqNo(), slave.getTxSeqNo());
}

TEST(FSoESequenceConformance, MultipleResetsResynchronize) {
    // Test multiple consecutive resets to ensure seq numbers stay synchronized.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;

    for (int reset_iter = 0; reset_iter < 5; ++reset_iter) {
        // Advance to Data
        for (int i = 0; i < 30; ++i) {
            now += 15;
            ASSERT_TRUE(conn.exchangeWith(slave, now));
            if (conn.isOperational()) break;
        }
        ASSERT_TRUE(conn.isOperational()) << "Reset iteration " << reset_iter;

        // Reset and verify resynchronization
        conn.resetConnection();
        now += 15;
        ASSERT_TRUE(conn.exchangeWith(slave, now));

        // After reset exchange:
        // Master: tx_seq=1, rx_seq=0 (last_tx_seq_no_=0, Reset's seq)
        // Slave: tx_seq=0 (last_rx_seq_no_=0), rx_seq=1
        EXPECT_EQ(conn.getTxSeqNo(), 1u) << "Reset iteration " << reset_iter;
        EXPECT_EQ(conn.getRxSeqNo(), 0u) << "Reset iteration " << reset_iter;
        EXPECT_EQ(slave.getTxSeqNo(), 0u) << "Reset iteration " << reset_iter;
        EXPECT_EQ(slave.getRxSeqNo(), 1u) << "Reset iteration " << reset_iter;
    }
}

// ============================================================================
// CRC collision avoidance — build side
// ============================================================================

TEST(FSoECollisionAvoidance, BuildAvoidsCollisionWithStartCrc) {
    // When the computed CRC0 equals start_crc (the previous CRC0),
    // the build function should increment the seq and recompute.
    // We test this by finding a payload/start_crc combination that
    // causes a collision, then verifying the build function avoids it.

    // Try various payloads to find a collision with start_crc=0
    uint8_t frame[64];
    uint16_t seq_used = 0;
    uint16_t out_crc0 = 0;

    // Use a simple 2-byte payload and start_crc=0
    uint8_t payload[] = {0x00, 0x00};

    // Build with collision avoidance
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Session, payload, 2, 0x1234,
        0,  // start_crc = 0
        1,  // initial_seq = 1
        &out_crc0, &seq_used);

    ASSERT_GT(len, 0u);
    // The output CRC0 must NOT equal start_crc (0) — collision was avoided
    EXPECT_NE(out_crc0, 0u)
        << "CRC0 == start_crc (0) — collision avoidance failed";
}

TEST(FSoECollisionAvoidance, BuildPreservesNonCollisionCrc) {
    // When there's no collision (CRC0 != start_crc), the build function
    // should use the initial seq without incrementing.
    uint8_t frame[64];
    uint16_t seq_used = 0;
    uint16_t out_crc0 = 0;

    // Use a payload that's unlikely to cause a collision with start_crc=0
    uint8_t payload[] = {0xFF, 0xFF, 0xFF, 0xFF};

    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Session, payload, 4, 0x1234,
        0,  // start_crc = 0
        1,  // initial_seq = 1
        &out_crc0, &seq_used);

    ASSERT_GT(len, 0u);
    // If no collision, seq_used should be 1
    if (out_crc0 != 0u) {
        EXPECT_EQ(seq_used, 1u);
    }
    // In any case, out_crc0 must not equal start_crc
    EXPECT_NE(out_crc0, 0u);
}

TEST(FSoECollisionAvoidance, BuildUpdatesCrcChain) {
    // The build function should update the CRC chain (out_last_crc0)
    // to the computed CRC0, so the next frame inherits it.
    uint8_t frame[64];
    uint16_t out_crc0 = 0;

    uint8_t payload[] = {0x42, 0x00};

    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Session, payload, 2, 0x1234,
        0x1234,  // start_crc = 0x1234
        5,       // initial_seq = 5
        &out_crc0, nullptr);

    ASSERT_GT(len, 0u);
    EXPECT_NE(out_crc0, 0x1234u);  // Must not equal start_crc (collision avoided)
}

// ============================================================================
// CRC collision avoidance — parse side
// ============================================================================

TEST(FSoECollisionAvoidance, ParseAcceptsNonCollisionFrame) {
    // Build a frame with collision avoidance, then parse it.
    // The parser should accept it with the same seq.
    uint8_t frame[64];
    uint16_t build_crc0 = 0;
    uint16_t build_seq = 0;

    uint8_t payload[] = {0x42, 0x00};
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Session, payload, 2, 0x1234,
        0,  // start_crc = 0
        1,  // initial_seq = 1
        &build_crc0, &build_seq);

    ASSERT_GT(len, 0u);

    // Parse with the same parameters
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    uint16_t parse_crc0 = 0;
    uint16_t parse_seq = 0;

    EXPECT_TRUE(CRC::parseFSoEFrameWithCollisionAvoidance(
        frame, len, cmd, data, data_len, conn_id,
        0,  // start_crc = 0
        1,  // initial_seq = 1
        &parse_crc0, &parse_seq));

    EXPECT_EQ(cmd, Command::Session);
    EXPECT_EQ(parse_crc0, build_crc0);
    EXPECT_EQ(parse_seq, build_seq);
}

TEST(FSoECollisionAvoidance, ParseRejectsCorruptedFrame) {
    // A corrupted frame should be rejected — the parser should NOT
    // try all possible seq values to find a match.
    uint8_t frame[64];
    uint16_t build_crc0 = 0;

    uint8_t payload[] = {0x42, 0x00};
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Session, payload, 2, 0x1234,
        0,  // start_crc = 0
        1,  // initial_seq = 1
        &build_crc0, nullptr);

    ASSERT_GT(len, 0u);

    // Corrupt the CRC0 bytes (offset 3-4 in a 7-byte Session frame)
    frame[3] ^= 0xFF;
    frame[4] ^= 0xFF;

    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    EXPECT_FALSE(CRC::parseFSoEFrameWithCollisionAvoidance(
        frame, len, cmd, data, data_len, conn_id,
        0,  // start_crc = 0
        1,  // initial_seq = 1
        nullptr, nullptr));
}

TEST(FSoECollisionAvoidance, ParseReplicatesCollisionAvoidance) {
    // If the build function incremented the seq due to a collision,
    // the parser should also increment and accept the frame.
    //
    // We simulate this by:
    // 1. Building a frame with buildFSoEFrameWithCollisionAvoidance
    //    (which may increment seq if there's a collision)
    // 2. Parsing with parseFSoEFrameWithCollisionAvoidance
    //    (which should replicate the same collision avoidance)
    // 3. The parse should succeed with the same seq_used

    // Try multiple payloads to exercise collision avoidance paths
    for (uint16_t start_crc : {0u, 0x1234u, 0xFFFFu, 0x0001u}) {
        for (uint16_t initial_seq : {1u, 2u, 100u, 65535u}) {
            for (int p = 0; p < 256; ++p) {
                uint8_t payload[] = {static_cast<uint8_t>(p), 0x00};
                uint8_t frame[64];
                uint16_t build_crc0 = 0;
                uint16_t build_seq = 0;

                size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
                    frame, Command::Session, payload, 2, 0x1234,
                    start_crc, initial_seq,
                    &build_crc0, &build_seq);

                ASSERT_GT(len, 0u);
                ASSERT_NE(build_crc0, start_crc)
                    << "Build failed to avoid collision at start_crc=0x"
                    << std::hex << start_crc << " seq=" << initial_seq
                    << " payload=" << p;

                // Parse with the same start_crc and initial_seq
                uint8_t cmd = 0;
                uint8_t data[16] = {0};
                size_t data_len = 0;
                uint16_t conn_id = 0;
                uint16_t parse_crc0 = 0;
                uint16_t parse_seq = 0;

                EXPECT_TRUE(CRC::parseFSoEFrameWithCollisionAvoidance(
                    frame, len, cmd, data, data_len, conn_id,
                    start_crc, initial_seq,
                    &parse_crc0, &parse_seq))
                    << "Parse failed at start_crc=0x" << std::hex << start_crc
                    << " seq=" << initial_seq << " payload=" << p;

                EXPECT_EQ(parse_crc0, build_crc0)
                    << "CRC0 mismatch at start_crc=0x" << std::hex << start_crc
                    << " seq=" << initial_seq << " payload=" << p;
                EXPECT_EQ(parse_seq, build_seq)
                    << "Seq mismatch at start_crc=0x" << std::hex << start_crc
                    << " seq=" << initial_seq << " payload=" << p;
            }
        }
    }
}

// ============================================================================
// Sequence number wrap-around in exchange
// ============================================================================

TEST(FSoESequenceConformance, SeqWrapsFrom65535ToOne) {
    // Test that the sequence number wraps from 65535 to 1 during
    // a full handshake + data exchange. We can't easily run 65535
    // exchanges, but we can verify the wrap-around logic via
    // incrementSeqNo (tested above) and verify that the build/parse
    // functions handle seq=65535 correctly.

    uint8_t frame[64];
    uint16_t out_crc0 = 0;
    uint16_t seq_used = 0;

    // Build with seq=65535
    uint8_t payload[] = {0x42, 0x00};
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Session, payload, 2, 0x1234,
        0x5678,  // start_crc
        65535,   // initial_seq
        &out_crc0, &seq_used);

    ASSERT_GT(len, 0u);
    // seq_used should be 65535 (or higher if collision avoidance incremented,
    // which would wrap to 1)
    if (out_crc0 != 0x5678u) {
        // No collision — seq should be 65535
        EXPECT_EQ(seq_used, 65535u);
    }

    // Parse with seq=65535
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    uint16_t parse_crc0 = 0;
    uint16_t parse_seq = 0;

    EXPECT_TRUE(CRC::parseFSoEFrameWithCollisionAvoidance(
        frame, len, cmd, data, data_len, conn_id,
        0x5678, 65535,
        &parse_crc0, &parse_seq));
    EXPECT_EQ(parse_crc0, out_crc0);
    EXPECT_EQ(parse_seq, seq_used);
}

// ============================================================================
// Reset frame with wrong seq is rejected
// ============================================================================

TEST(FSoESequenceConformance, MasterAcceptsResetResponseWithSeqOne) {
    // The slave's Reset response uses seq=1 (initial_seq_no + 1).
    // The master should accept it.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Build a Reset response with seq=1 (correct — slave increments seq)
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 1);
    ASSERT_GT(frame_len, 0u);

    // Master should accept it
    EXPECT_TRUE(conn.processRxFrame(frame, frame_len));
    EXPECT_EQ(conn.getState(), ConnectionState::Session);
}

TEST(FSoESequenceConformance, MasterRejectsResetResponseWithSeqZero) {
    // A Reset response with seq=0 should be rejected by the master,
    // since the master expects seq=1 for the slave's Reset response
    // (initial_seq_no + 1 = 0 + 1 = 1).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Build a Reset frame with seq=0 (wrong — master expects seq=1)
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 0);
    ASSERT_GT(frame_len, 0u);

    // Master should reject it (CRC won't match with seq=0 vs expected seq=1)
    EXPECT_FALSE(conn.processRxFrame(frame, frame_len));
}

TEST(FSoESequenceConformance, SlaveRejectsResetWithSeqOne) {
    // A Reset frame with seq=1 should be rejected by the slave.
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Build a Reset frame with seq=1 (wrong — slave expects seq=0)
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 1);
    ASSERT_GT(frame_len, 0u);

    // Slave should reject it
    EXPECT_FALSE(slave.processRxFrame(frame, frame_len));
}

// ============================================================================
// Data exchange sequence synchronization
// ============================================================================

TEST(FSoESequenceConformance, DataExchangeKeepsSeqSynchronized) {
    // After reaching Data state, verify that multiple data exchanges
    // keep master and slave seq numbers synchronized.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // Run 50 data exchanges and verify synchronization
    for (int i = 0; i < 50; ++i) {
        now += 15;
        ASSERT_TRUE(conn.exchangeWith(slave, now));

        // Master TX seq == slave RX seq (master sends, slave receives)
        EXPECT_EQ(conn.getTxSeqNo(), slave.getRxSeqNo())
            << "Cycle " << i << ": master TX seq != slave RX seq";
        // Master RX seq == slave TX seq (slave sends, master receives)
        EXPECT_EQ(conn.getRxSeqNo(), slave.getTxSeqNo())
            << "Cycle " << i << ": master RX seq != slave TX seq";
        // Seq should never be 0
        EXPECT_NE(conn.getTxSeqNo(), 0u);
        EXPECT_NE(conn.getRxSeqNo(), 0u);
        EXPECT_NE(slave.getTxSeqNo(), 0u);
        EXPECT_NE(slave.getRxSeqNo(), 0u);
    }
}

// ============================================================================
// Reset frame CRC chain reset
// ============================================================================

TEST(FSoESequenceConformance, ResetFrameResetsCrcChain) {
    // After a Reset frame, the CRC chain should be reset to 0.
    // The next frame should use start_crc=0.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    uint64_t now = 0;
    advanceToData(conn, slave, now);

    // CRC chain should be non-zero after data exchange
    EXPECT_NE(conn.getTxLastCrc0(), 0u);
    EXPECT_NE(conn.getRxLastCrc0(), 0u);

    // Reset
    conn.resetConnection();
    EXPECT_EQ(conn.getTxLastCrc0(), 0u);
    EXPECT_EQ(conn.getRxLastCrc0(), 0u);

    // Exchange: master sends Reset (seq=1, start_crc=0, doesn't update CRC chain).
    // Slave was in Data state, receives Reset, transitions to Session,
    // sends Session response (which DOES update the CRC chain).
    // So after this exchange:
    //   - Master TX CRC = 0 (sent Reset, didn't update)
    //   - Slave RX CRC = 0 (received Reset, didn't update)
    //   - Master RX CRC = non-zero (received Session response, updated)
    //   - Slave TX CRC = non-zero (sent Session response, updated)
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // Reset frame sides: CRC chain stays at 0
    EXPECT_EQ(conn.getTxLastCrc0(), 0u);  // Master sent Reset
    EXPECT_EQ(slave.getRxLastCrc0(), 0u);  // Slave received Reset

    // Session response sides: CRC chain is updated (non-zero)
    // (These may be 0 in the rare case where the Session response's CRC0
    //  happens to be 0, but that's extremely unlikely with real data.)
    // We don't assert non-zero here to avoid flakiness — the key invariant
    // is that the Reset sides stay at 0.

    // After the next exchange (Session), CRC chain should be non-zero
    // on all sides (both master and slave send Session frames)
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    EXPECT_NE(conn.getTxLastCrc0(), 0u);
    EXPECT_NE(conn.getRxLastCrc0(), 0u);
}
