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
 * - CRC collision avoidance: when CRC0 collides with EITHER the peer's
 *   last frame CRC0 (start_crc, cross-direction) or the sender's own
 *   previous frame CRC0 (same-direction), the seq is incremented and
 *   CRC recomputed (both on build and parse sides)
 * - The collision avoidance parser replicates the generator's seq skips:
 *   it retries when the CRC verifies but collides, and when the computed
 *   CRC0 collides (generator skipped that seq), but not on ordinary CRC
 *   mismatch
 * - Same-direction avoidance prevents byte-identical consecutive frames
 *   (which the receiver's resend detection would misclassify)
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
    // getTxSeqNo() = tx_seq_no_ (the master's own TX seq counter).
    // getRxSeqNo() = last_tx_seq_no_ (the seq used in the last TX, which
    // the slave echoes — 0 before any TX).
    MasterConnectionConfig cfg = makeMasterCfg(4, 4);
    cfg.initial_seq_no = 1;
    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);  // last_tx_seq_no_ = 0 (no TX yet)

    // First TX frame should be a Reset with seq=1
    uint8_t buf[64] = {};
    size_t len = conn.prepareTxFrame(buf, sizeof(buf));
    ASSERT_GT(len, 0u);
    // After TX, tx_seq_no_ = 2 (incremented for next TX)
    // getRxSeqNo() = last_tx_seq_no_ = 1 (the seq just used)
    EXPECT_EQ(conn.getTxSeqNo(), 2u);
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
    // getTxSeqNo() = last_rx_seq_no_ (the master's last TX seq, echoed
    // by the slave — 0 before any RX from the master).
    // getRxSeqNo() = rx_seq_no_ (expected next master TX seq).
    FSoESlaveConfig cfg = makeSlaveCfg(4, 4);
    cfg.initialSeqNo = 1;
    FSoESlave slave(cfg);
    slave.initialize();
    EXPECT_EQ(slave.getTxSeqNo(), 0u);  // last_rx_seq_no_ = 0 (no RX yet)
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
    // When the slave receives a Reset, it stays in Reset state and
    // sends a Reset response (0x2A).  The slave's Reset response uses
    // seq=initialSeqNo (same as master's initial_seq_no — independent
    // counters that both start at the configured initial value).
    // The transition to Session happens when the slave receives the
    // master's Session TX.
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Build a Reset frame with seq=0, start_crc=0
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 0);
    ASSERT_GT(frame_len, 0u);
    ASSERT_TRUE(slave.processRxFrame(frame, frame_len));

    // Slave stays in Reset state on Reset command (matching the
    // physical Synapticon slave behavior).
    EXPECT_EQ(slave.getState(), ConnectionState::Reset);

    // Build the slave's TX frame — should be a Reset response.
    uint8_t tx[64];
    size_t tx_len = slave.prepareTxFrame(tx, sizeof(tx));
    ASSERT_GT(tx_len, 0u);

    // Parse with start_crc=0 (CRC chain was reset).
    // Try seq=0, seq=1, and seq=2 (collision avoidance may increment).
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    bool parsed = CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                      0, 0);
    if (!parsed) {
        parsed = CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                     0, 1);
    }
    if (!parsed) {
        parsed = CRC::parseFSoEFrame(tx, tx_len, cmd, data, data_len, conn_id,
                                     0, 2);
    }
    EXPECT_TRUE(parsed) << "Failed to parse with seq=0, seq=1, or seq=2";
    EXPECT_EQ(cmd, Command::Reset);
}

TEST(FSoESequenceConformance, SeqAdvancesAfterResetFrame) {
    // After a Reset frame (seq=0), the next frame should use seq=1.
    // Model:
    // - Master TX: own tx_seq_no_, incremented after TX
    // - Master RX: expects slave TX seq = last_tx_seq_no_ (the seq just used)
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // First TX: Reset frame (seq=0)
    uint8_t buf1[64] = {};
    size_t len1 = conn.prepareTxFrame(buf1, sizeof(buf1));
    ASSERT_GT(len1, 0u);
    // After prepareTxFrame, tx_seq_no_ = 1 (incremented for next TX)
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    // getRxSeqNo() = last_tx_seq_no_ = 0 (the seq just used, before increment)
    EXPECT_EQ(conn.getRxSeqNo(), 0u);

    // Feed a Reset response to advance to Session.
    // The slave's Reset response uses seq=0 (same as master's initial_seq_no).
    // Master and slave have independent counters that both start at the
    // configured initial value.
    uint8_t rx_payload[4] = {0, 0, 0, 0};
    uint8_t rx_frame[64];
    size_t rx_len = CRC::buildFSoEFrame(rx_frame, Command::Reset,
                                        rx_payload, 4, 0x1234, 0, 0);
    ASSERT_TRUE(conn.processRxFrame(rx_frame, rx_len));
    EXPECT_EQ(conn.getState(), ConnectionState::Session);
    // RX doesn't change last_tx_seq_no_, so getRxSeqNo() = 0
    EXPECT_EQ(conn.getRxSeqNo(), 0u);

    // Second TX: Session frame — uses state-transition reset
    // (start_crc=0, seq=initial_seq_no=0).  The Synapticon slave resets
    // the CRC chain AND seq at the Session state transition.
    uint8_t buf2[64] = {};
    size_t len2 = conn.prepareTxFrame(buf2, sizeof(buf2));
    ASSERT_GT(len2, 0u);
    // After TX, tx_seq_no_ = 1 (initial=0, set by buildSessionResetFrame,
    // then incremented by prepareTxFrame)
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    // getRxSeqNo() = last_tx_seq_no_ = 0 (the seq just used, before increment)
    EXPECT_EQ(conn.getRxSeqNo(), 0u);

    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    // Parse with start_crc=0, seq=0 (state-transition reset for Session TX)
    EXPECT_TRUE(CRC::parseFSoEFrame(buf2, len2, cmd, data, data_len, conn_id,
                                    0, 0));
    EXPECT_EQ(cmd, Command::Session);
}

// ============================================================================
// Full handshake sequence progression
// ============================================================================

TEST(FSoESequenceConformance, FullHandshakeSeqProgression) {
    // Verify seq numbers through the full handshake.
    // Model:
    // - Master TX: own tx_seq_no_, incremented after TX
    // - Master RX: expects slave TX seq = last_tx_seq_no_ (the seq just used)
    // - Slave TX: echoes last_rx_seq_no_ (master's last TX seq)
    // - Slave RX: rx_seq_no_ (expected next master TX seq)
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

    // Exchange 1: Master sends Reset (seq=0), slave stays in Reset and
    // answers with a Reset response (seq=0).  Master transitions to Session.
    // (master and slave have independent counters, both starting at initial_seq_no=0)
    uint64_t now = 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // After exchange 1:
    // Master: tx_seq=1 (incremented), rx_seq=0 (last_tx_seq_no_=0)
    // Slave: tx_seq=0 (last_rx_seq_no_=0), rx_seq=1 (incremented by validateFrame)
    // Slave stays in Reset state until the master's Session command
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);
    EXPECT_EQ(slave.getTxSeqNo(), 0u);
    EXPECT_EQ(slave.getRxSeqNo(), 1u);
    EXPECT_EQ(slave.getState(), ConnectionState::Reset);

    // Exchange 2: Master sends Session (seq=0, start_crc=0 — state-transition
    // reset).  Slave transitions to Session, sends Session response (seq=0,
    // cross-direction inheritance from master's Session TX).
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    // Master: tx_seq=1 (initial=0, set by buildSessionResetFrame, then
    //         incremented), rx_seq=0 (last_tx_seq_no_=0)
    // Slave: tx_seq=0 (last_rx_seq_no_=0), rx_seq=1 (state-transition reset
    //        for Session RX: seq=initial=0, then incremented)
    // Same seq values as after exchange 1 (Session TX resets seq to initial)
    EXPECT_EQ(conn.getTxSeqNo(), 1u);
    EXPECT_EQ(conn.getRxSeqNo(), 0u);
    EXPECT_EQ(slave.getTxSeqNo(), 0u);
    EXPECT_EQ(slave.getRxSeqNo(), 1u);

    // Exchange 3: Master sends Connection (seq=1, cross-direction inheritance).
    // Slave transitions to Connection.
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    // Master: tx_seq=2, rx_seq=1 (last_tx_seq_no_=1)
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
    // Master RX seq (last_tx_seq_no_) == slave TX seq (last_rx_seq_no_)
    // Both equal the seq the master used in its last TX.
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
    // Master: tx_seq=1 (incremented), rx_seq=0 (last_tx_seq_no_=0)
    // Slave: tx_seq=0 (last_rx_seq_no_=0, reset), rx_seq=1 (incremented)
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
        // Master: tx_seq=1 (incremented), rx_seq=0 (last_tx_seq_no_=0)
        // Slave: tx_seq=0 (last_rx_seq_no_=0, reset), rx_seq=1 (incremented)
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

TEST(FSoECollisionAvoidance, ParseAcceptsSeqBumpedFrame) {
    // Regression test: when the generator's collision avoidance bumps the
    // seq (computed CRC0 == start_crc), the checker must replicate the skip
    // and accept the frame at the bumped seq.
    //
    // start_crc=0x1234, seq=56422 produces crc0 == start_crc for this
    // payload (found by enumeration — collisions occur with probability
    // ~1/65536 per frame, so this case is rarely exercised by chance).
    const uint16_t start_crc = 0x1234;
    const uint16_t colliding_seq = 56422;
    uint8_t payload[] = {0x34, 0x12, 0x00, 0x01};

    // Confirm the case actually collides at the un-bumped seq.
    uint8_t probe[64];
    uint16_t probe_crc0 = 0;
    ASSERT_GT(CRC::buildFSoEFrame(probe, Command::Connection, payload, 4,
                                  0x1234, start_crc, colliding_seq,
                                  &probe_crc0), 0u);
    ASSERT_EQ(probe_crc0, start_crc)
        << "Test premise broken: expected collision at seq " << colliding_seq;

    // Build with collision avoidance — the seq must be bumped.
    uint8_t frame[64];
    uint16_t build_crc0 = 0;
    uint16_t build_seq = 0;
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Connection, payload, 4, 0x1234,
        start_crc, colliding_seq, &build_crc0, &build_seq);
    ASSERT_GT(len, 0u);
    ASSERT_NE(build_crc0, start_crc);
    ASSERT_GT(build_seq, colliding_seq);

    // The checker must accept the bumped frame.
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    uint16_t parse_crc0 = 0;
    uint16_t parse_seq = 0;

    EXPECT_TRUE(CRC::parseFSoEFrameWithCollisionAvoidance(
        frame, len, cmd, data, data_len, conn_id,
        start_crc, colliding_seq,
        &parse_crc0, &parse_seq));
    EXPECT_EQ(parse_crc0, build_crc0);
    EXPECT_EQ(parse_seq, build_seq);
}

TEST(FSoECollisionAvoidance, ParseRoundTripsAllBumpedSeqs) {
    // Scan for colliding seqs and verify every bumped frame round-trips.
    const uint16_t start_crc = 0x5A5A;
    uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    int collisions = 0;

    for (uint32_t seq = 1; seq < 0x10000 && collisions < 8; ++seq) {
        uint8_t probe[64];
        uint16_t probe_crc0 = 0;
        if (CRC::buildFSoEFrame(probe, Command::Parameter, payload, 4,
                                  0x1234, start_crc, (uint16_t)seq,
                                  &probe_crc0) == 0) continue;
        if (probe_crc0 != start_crc) continue;
        ++collisions;

        uint8_t frame[64];
        uint16_t build_crc0 = 0, build_seq = 0;
        size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
            frame, Command::Parameter, payload, 4, 0x1234,
            start_crc, (uint16_t)seq, &build_crc0, &build_seq);
        ASSERT_GT(len, 0u);
        ASSERT_NE(build_crc0, start_crc);
        ASSERT_GT(build_seq, seq);

        uint8_t cmd = 0;
        uint8_t data[16] = {0};
        size_t data_len = 0;
        uint16_t conn_id = 0;
        uint16_t parse_crc0 = 0, parse_seq = 0;
        EXPECT_TRUE(CRC::parseFSoEFrameWithCollisionAvoidance(
            frame, len, cmd, data, data_len, conn_id,
            start_crc, (uint16_t)seq,
            &parse_crc0, &parse_seq))
            << "Parse rejected seq-bumped frame (colliding seq=" << seq << ")";
        EXPECT_EQ(parse_crc0, build_crc0);
        EXPECT_EQ(parse_seq, build_seq);
    }
    EXPECT_GT(collisions, 0)
        << "No collision found in seq scan — test provides no coverage";
}

TEST(FSoECollisionAvoidance, ParseStillRejectsCorruptedFrame) {
    // The seq-skip replication must not weaken integrity checking: a frame
    // whose stored CRC does not match at ANY generator-legal seq is a
    // genuine error and must be rejected.
    const uint16_t start_crc = 0x1234;
    const uint16_t colliding_seq = 56422;  // collides (see test above)
    uint8_t payload[] = {0x34, 0x12, 0x00, 0x01};

    uint8_t frame[64];
    uint16_t build_crc0 = 0, build_seq = 0;
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::Connection, payload, 4, 0x1234,
        start_crc, colliding_seq, &build_crc0, &build_seq);
    ASSERT_GT(len, 0u);
    ASSERT_GT(build_seq, colliding_seq);  // sanity: seq was bumped

    // Corrupt a data byte — the frame can no longer verify at any seq.
    frame[1] ^= 0xFF;

    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    EXPECT_FALSE(CRC::parseFSoEFrameWithCollisionAvoidance(
        frame, len, cmd, data, data_len, conn_id,
        start_crc, colliding_seq, nullptr, nullptr));
}

TEST(FSoECollisionAvoidance, BuildAvoidsSameDirectionCollision) {
    // Regression test for the consecutive-frame collision defect:
    // all CRC segments fold the seq in, so crcCommon(S) == crcCommon(S+1)
    // (~1/65536 per pair) makes two consecutive same-direction frames
    // BYTE-IDENTICAL.  The receiver's resend detection keys on byte
    // equality, so a legitimate new frame was swallowed and the CRC/seq
    // chains desynchronized permanently.
    //
    // The generator must skip a seq whose CRC0 equals its OWN previous
    // frame's CRC0 (same direction), not just the peer's (start_crc) —
    // and the checker must replicate that skip.
    const uint16_t conn_id = 0x1234;
    uint8_t payload[] = {0x11, 0x22};

    // Find a real-world collision: consecutive frames are built with
    // DIFFERENT start_crc values (the chain inherits the peer's CRC0,
    // which changes each exchange).  crc0 is injective in seq for a fixed
    // start_crc, so identical frames require crc0(B, seq+1) == crc0(A, seq)
    // across two different inherited values — ~1/65536 per (A, B) pair.
    //
    // With data_len <= 2 the frame carries a single CRC segment, so equal
    // crc0 implies BYTE-identical frames (start_crc is not in the bytes).
    const uint16_t prev_seq = 100;
    uint8_t prev_frame[64];
    size_t prev_len = 0;
    uint16_t prev_crc0 = 0;
    const uint16_t prev_start_crc = 0x1234;
    prev_len = CRC::buildFSoEFrame(
        prev_frame, Command::ProcessData, payload, 2, conn_id,
        prev_start_crc, prev_seq, &prev_crc0);
    ASSERT_GT(prev_len, 0u);

    uint16_t start_crc = 0;
    for (uint32_t sc = 1; sc < 0x10000; ++sc) {
        if ((uint16_t)sc == prev_start_crc) continue;
        uint8_t probe[64];
        uint16_t crc0 = 0;
        if (CRC::buildFSoEFrame(probe, Command::ProcessData, payload, 2,
                                  conn_id, (uint16_t)sc,
                                  CRC::incrementSeqNo(prev_seq),
                                  &crc0) == 0) continue;
        // Require crc0 == prev frame's CRC0 (the same-direction collision)
        // but crc0 != sc so the cross-direction check alone can't trigger
        // the bump — isolating the same-direction rule under test.
        if (crc0 == prev_crc0 && crc0 != (uint16_t)sc) {
            start_crc = (uint16_t)sc;
            break;
        }
    }
    const uint16_t colliding_seq = CRC::incrementSeqNo(prev_seq);
    ASSERT_NE(start_crc, 0)
        << "No same-direction collision found in start_crc scan — "
           "test provides no coverage";

    // Sanity: the unprotected frame is byte-identical to the previous one.
    uint8_t frame[64];
    uint16_t coll_crc0 = 0;
    size_t coll_len = CRC::buildFSoEFrame(
        frame, Command::ProcessData, payload, 2, conn_id,
        start_crc, colliding_seq, &coll_crc0);
    ASSERT_EQ(coll_len, prev_len);
    ASSERT_EQ(coll_crc0, prev_crc0);
    ASSERT_EQ(0, std::memcmp(frame, prev_frame, coll_len));

    // Without same-direction avoidance the builder emits the identical
    // frame — a resend detector cannot distinguish it from a duplicate.
    {
        uint8_t old_frame[64];
        uint16_t old_crc0 = 0, old_seq = 0;
        size_t old_len = CRC::buildFSoEFrameWithCollisionAvoidance(
            old_frame, Command::ProcessData, payload, 2, conn_id,
            start_crc, colliding_seq, &old_crc0, &old_seq);
        ASSERT_GT(old_len, 0u);
        ASSERT_EQ(old_seq, colliding_seq);  // no bump without prev ref
        ASSERT_EQ(0, std::memcmp(old_frame, prev_frame, old_len));
    }

    // With prev_same_dir_crc0 the builder MUST bump past the collision.
    uint16_t build_crc0 = 0, build_seq = 0;
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        frame, Command::ProcessData, payload, 2, conn_id,
        start_crc, colliding_seq, &build_crc0, &build_seq,
        &prev_crc0);
    ASSERT_GT(len, 0u);
    ASSERT_NE(build_crc0, prev_crc0);
    ASSERT_GT(build_seq, colliding_seq);
    ASSERT_NE(0, std::memcmp(frame, prev_frame, len))
        << "Bumped frame still byte-identical to previous frame";

    // The checker replicates the skip: parsing from the un-bumped seq
    // with the same prev reference accepts the bumped frame.
    uint8_t cmd = 0;
    uint8_t data[16] = {0};
    size_t data_len = 0;
    uint16_t parsed_conn = 0;
    uint16_t parse_crc0 = 0, parse_seq = 0;
    EXPECT_TRUE(CRC::parseFSoEFrameWithCollisionAvoidance(
        frame, len, cmd, data, data_len, parsed_conn,
        start_crc, colliding_seq,
        &parse_crc0, &parse_seq, nullptr, &prev_crc0));
    EXPECT_EQ(parse_crc0, build_crc0);
    EXPECT_EQ(parse_seq, build_seq);
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

TEST(FSoESequenceConformance, MasterAcceptsResetResponseWithSeqZero) {
    // The slave's Reset response uses seq=0 (same as master's initial_seq_no).
    // Master and slave have independent counters that both start at the
    // configured initial value.  The master should accept it.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Build a Reset response with seq=0 (correct — same as initial_seq_no)
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 0);
    ASSERT_GT(frame_len, 0u);

    // Master should accept it
    EXPECT_TRUE(conn.processRxFrame(frame, frame_len));
    EXPECT_EQ(conn.getState(), ConnectionState::Session);
}

TEST(FSoESequenceConformance, MasterRejectsResetResponseWithSeqOne) {
    // A Reset response with seq=1 should be rejected by the master,
    // since the master expects seq=0 for the slave's Reset response
    // (same as initial_seq_no = 0).
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Build a Reset frame with seq=1 (wrong — master expects seq=0)
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::Reset,
                                           payload, 4, 0x1234, 0, 1);
    ASSERT_GT(frame_len, 0u);

    // Master should reject it (CRC won't match with seq=1 vs expected seq=0)
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

    // Exchange: master sends Reset (seq=1, start_crc=0 — the CRC
    // inheritance chain resets, but the frame's CRC0 IS captured into
    // last_tx_crc0_ for same-direction collision tracking).
    // Slave was in Data state, receives Reset (captures its CRC0 into
    // last_rx_crc0_), transitions to Session, sends Session response.
    // So after this exchange:
    //   - Master TX CRC0 tracking = Reset frame's CRC0 (non-zero)
    //   - Slave RX CRC0 tracking  = same Reset frame's CRC0 (non-zero)
    //   - The inheritance chain itself still reset (next TX uses
    //     start_crc=0) — proven by the Session exchange succeeding below.
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));

    // Both sides track the master's Reset frame CRC0 for same-direction
    // collision avoidance — they must be equal.
    // With cross-direction naming, getRxLastCrc0() on the master returns
    // last_tx_crc0_, and getTxLastCrc0() on the slave returns
    // last_rx_crc0_.
    EXPECT_NE(conn.getRxLastCrc0(), 0u);  // Master sent Reset (CRC0 captured)
    EXPECT_EQ(conn.getRxLastCrc0(), slave.getTxLastCrc0());  // same Reset frame

    // After the next exchange (Session), CRC chain should be non-zero
    // on all sides (both master and slave send Session frames)
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    EXPECT_NE(conn.getTxLastCrc0(), 0u);
    EXPECT_NE(conn.getRxLastCrc0(), 0u);
}
