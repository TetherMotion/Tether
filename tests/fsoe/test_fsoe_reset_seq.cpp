// test_fsoe_reset_seq.cpp
//
// Tests that the FSoE master correctly accepts a slave Reset response
// that uses the SAME sequence number as the master's Reset TX (not +1).
//
// Per ETG.5100, master and slave have INDEPENDENT sequence counters.
// Both start at their own initial seq value. The slave's Reset response
// uses the slave's own initial seq — which is the SAME value as the
// master's initial seq (both are configured to the same value).
//
// The previous code incorrectly used incrementSeqNo(initial_seq_no) for
// parsing the slave's Reset response, causing a CRC mismatch with real
// Synapticon hardware (which uses initial_seq_no, not +1).

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoESlave.hpp"
#include "tether/fsoe/FSoECRC.hpp"
#include "tether/fsoe/FSoEDefs.hpp"

using namespace FSoE;

// ============================================================================
// Helpers
// ============================================================================

static MasterConnectionConfig makeMasterCfg(uint8_t inSize = 4,
                                             uint8_t outSize = 4,
                                             uint16_t initSeq = 1) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0100;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = 0x1234;
    cfg.master_addr = 0x0001;
    cfg.input_size = inSize;
    cfg.output_size = outSize;
    cfg.watchdog_timeout_ms = 200;
    cfg.conn_timeout_ms = 100;
    cfg.safety_level = 0x02;
    cfg.auto_fail_safe_on_error = true;
    cfg.auto_recovery_enabled = false;
    cfg.initial_seq_no = initSeq;
    return cfg;
}

static FSoESlaveConfig makeSlaveCfg(uint8_t inSize = 4, uint8_t outSize = 4,
                                     uint16_t initSeq = 1) {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 0x0100;
    cfg.connectionId = 0x1234;
    cfg.safetyAddress = 0x0100;
    cfg.safetyLevel = SIL::SIL2;
    cfg.safeInputSize = inSize;
    cfg.safeOutputSize = outSize;
    cfg.watchdogTimeoutMs = 200;
    cfg.connectionTimeoutMs = 100;
    cfg.initialSeqNo = initSeq;
    return cfg;
}

// Build a slave Reset response frame with a specific seq number,
// simulating a real slave that uses its own initial seq (not +1).
static std::vector<uint8_t> buildSlaveResetFrame(uint8_t inputSize,
                                                  uint16_t seqNo) {
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    uint8_t buf[64] = {0};
    uint16_t seq_used = 0;
    size_t len = CRC::buildFSoEFrameWithCollisionAvoidance(
        buf, Command::Reset, payload, inputSize,
        0,  // conn_id = 0 in Reset
        0,  // start_crc = 0 (Reset resets chain)
        seqNo,
        nullptr,  // don't need out_crc0
        &seq_used);
    return std::vector<uint8_t>(buf, buf + len);
}

// ============================================================================
// Tests: Master accepts slave Reset response with SAME seq (not +1)
// ============================================================================

TEST(FSoEResetSeq, MasterAcceptsSlaveResetWithSameSeq) {
    // Simulate a real slave that responds with Reset using the SAME
    // initial seq as the master (not initial_seq_no + 1).
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 1));
    conn.initialize();
    conn.startConnection();
    ASSERT_EQ(conn.getState(), ConnectionState::Reset);

    // Master builds its Reset TX frame
    uint8_t txBuf[64] = {0};
    size_t txLen = conn.prepareTxFrame(txBuf, sizeof(txBuf));
    ASSERT_GT(txLen, 0u);

    // Build a slave Reset response with seq=1 (same as master's initial_seq_no)
    // This is what real Synapticon hardware does.
    auto slaveReset = buildSlaveResetFrame(4, 1);
    ASSERT_FALSE(slaveReset.empty());

    // The master must accept this frame and transition to Session
    bool ok = conn.processRxFrame(slaveReset.data(), slaveReset.size());
    EXPECT_TRUE(ok) << "Master must accept slave Reset response with same seq";
    EXPECT_NE(conn.getState(), ConnectionState::Reset)
        << "Master must transition out of Reset after valid slave Reset response";
}

TEST(FSoEResetSeq, MasterAcceptsSlaveResetWithSameSeqInit0) {
    // Same test but with initial_seq_no = 0 (Synapticon convention)
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 0));
    conn.initialize();
    conn.startConnection();
    ASSERT_EQ(conn.getState(), ConnectionState::Reset);

    uint8_t txBuf[64] = {0};
    conn.prepareTxFrame(txBuf, sizeof(txBuf));

    // Slave responds with seq=0 (same as master's initial_seq_no=0)
    // Note: collision avoidance may increment, but the initial attempt is 0.
    // buildFSoEFrameWithCollisionAvoidance handles this.
    auto slaveReset = buildSlaveResetFrame(4, 0);
    ASSERT_FALSE(slaveReset.empty());

    bool ok = conn.processRxFrame(slaveReset.data(), slaveReset.size());
    EXPECT_TRUE(ok) << "Master must accept slave Reset with seq=initial (0)";
}

TEST(FSoEResetSeq, MasterAcceptsSlaveResetWithSameSeqInit5) {
    // Same test but with a non-standard initial_seq_no = 5
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 5));
    conn.initialize();
    conn.startConnection();
    ASSERT_EQ(conn.getState(), ConnectionState::Reset);

    uint8_t txBuf[64] = {0};
    conn.prepareTxFrame(txBuf, sizeof(txBuf));

    // Slave responds with seq=5 (same as master's initial_seq_no=5)
    auto slaveReset = buildSlaveResetFrame(4, 5);
    ASSERT_FALSE(slaveReset.empty());

    bool ok = conn.processRxFrame(slaveReset.data(), slaveReset.size());
    EXPECT_TRUE(ok) << "Master must accept slave Reset with seq=initial (5)";
}

// ============================================================================
// Tests: Full handshake with slave using same-seq Reset
// ============================================================================

TEST(FSoEResetSeq, FullHandshakeWithSameSeqReset) {
    // The slave emulator currently uses incrementSeqNo for Reset responses.
    // After fixing the slave emulator, the full exchangeWith handshake
    // should work with both master and slave using the same initial seq.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 1));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4, 1));
    slave.initialize();

    ASSERT_EQ(conn.getState(), ConnectionState::Reset);
    ASSERT_EQ(slave.getState(), ConnectionState::Reset);

    // Run the handshake
    uint64_t now = 0;
    for (int i = 0; i < 50; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() == ConnectionState::Data) break;
    }

    EXPECT_EQ(conn.getState(), ConnectionState::Data)
        << "Full handshake must complete with same-seq Reset";
    EXPECT_TRUE(conn.isOperational())
        << "Master must be operational after handshake";
}

// ============================================================================
// Tests: Master rejects slave Reset with WRONG seq (not just different from +1)
// ============================================================================

TEST(FSoEResetSeq, MasterRejectsSlaveResetWithTrulyWrongSeq) {
    // The master should still reject frames with a genuinely wrong seq
    // (not just seq != initial_seq_no + 1).
    // With collision avoidance, the parse function tries the initial seq
    // and increments if CRC0 == start_crc. So a frame built with a very
    // different seq (e.g. 100) should fail CRC verification.
    FSoEMasterConnection conn(makeMasterCfg(4, 4, 1));
    conn.initialize();
    conn.startConnection();
    ASSERT_EQ(conn.getState(), ConnectionState::Reset);

    uint8_t txBuf[64] = {0};
    conn.prepareTxFrame(txBuf, sizeof(txBuf));

    // Build a slave Reset response with seq=100 (clearly wrong)
    auto slaveReset = buildSlaveResetFrame(4, 100);
    ASSERT_FALSE(slaveReset.empty());

    // The master should reject this (CRC won't match with seq=1 or any
    // collision-avoidance increment of seq=1 that lands near 1)
    bool ok = conn.processRxFrame(slaveReset.data(), slaveReset.size());
    // Note: collision avoidance in parseFSoEFrameWithCollisionAvoidance
    // tries seq, seq+1, seq+2... until CRC matches or max_attempts.
    // With seq=100, the CRC will be completely different from what
    // seq=1 produces, so it should fail.
    EXPECT_FALSE(ok) << "Master must reject slave Reset with wrong seq";
}
