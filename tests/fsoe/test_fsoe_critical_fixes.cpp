/**
 * @file test_fsoe_critical_fixes.cpp
 * @brief Tests for critical FSoE fixes: fail-safe response recognition,
 *        non-strict sequence validation, and thread safety.
 */

#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <vector>

#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoEMaster.hpp"
#include "fsoe/FSoESlave.hpp"

using namespace FSoE;

// ============================================================================
// Fix #3: Master recognizes slave fail-safe response (0x80)
// ============================================================================

class FSoEFailSafeResponseTest : public ::testing::Test {
protected:
    void SetUp() override {
        MasterConnectionConfig cfg{};
        cfg.slave_addr = 0x0100;
        cfg.slave_safety_addr = 0x0100;
        cfg.connection_id = 0x1234;
        cfg.master_addr = 0x0100;
        cfg.watchdog_timeout_ms = 100;
        cfg.input_size = 4;
        cfg.output_size = 4;
        cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};
        conn = std::make_unique<FSoEMasterConnection>(cfg);
        conn->initialize();
        conn->startConnection();

        FSoESlaveConfig slave_cfg{};
        slave_cfg.slaveAddress = 0x0100;
        slave_cfg.connectionId = 0x1234;
        slave_cfg.safetyAddress = 0x0100;
        slave_cfg.safetyLevel = SIL::SIL2;
        slave_cfg.watchdogTimeoutMs = 200;
        slave_cfg.connectionTimeoutMs = 2000;
        slave_cfg.sessionTimeoutMs = 10000;
        slave_cfg.safeInputSize = 4;
        slave_cfg.safeOutputSize = 4;
        slave_cfg.autoRecoveryEnabled = false;
        slave_cfg.strictCrcCheck = true;
        slave_cfg.strictSequenceCheck = true;
        slave = std::make_unique<FSoESlave>(slave_cfg);
        slave->initialize();
    }

    void advanceToData() {
        uint64_t now = 0;
        for (int i = 0; i < 20; ++i) {
            now += 15;
            ASSERT_TRUE(conn->exchangeWith(*slave, now));
            if (conn->isOperational()) break;
        }
        last_time = now;
        ASSERT_TRUE(conn->isOperational())
            << "Master state: " << (int)conn->getState()
            << " Slave state: " << (int)slave->getState();
    }

    std::unique_ptr<FSoEMasterConnection> conn;
    std::unique_ptr<FSoESlave> slave;
    uint64_t last_time = 0;
};

TEST_F(FSoEFailSafeResponseTest, MasterRecognizesSlaveFailSafeResponse) {
    advanceToData();
    EXPECT_TRUE(conn->isOperational());
    EXPECT_FALSE(conn->isFailSafe());

    // Trigger fail-safe on slave
    slave->triggerFailSafe(ErrorCode::WatchdogError);
    EXPECT_TRUE(slave->isFailSafe());

    // Exchange one cycle — slave will send FailSafeData response (0x08)
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time + 15));

    // ETG.5100 §8.2.2.6: The choice between ProcessData and FailSafeData is
    // INDEPENDENT in each direction.  The master does NOT auto-enter fail-safe
    // when the slave sends FailSafeData.  The master accepts the slave's
    // fail-safe inputs (all zeros) and stays in its current mode.
    EXPECT_FALSE(conn->isFailSafe());
    EXPECT_FALSE(conn->getStatus().data_valid);
    // Master keeps its own error code (NoError) — the slave's FailSafeData
    // PDU has no error code field per ETG.5100 Table 26.
    EXPECT_EQ(conn->getErrorCode(), ErrorCode::NoError);
}

TEST_F(FSoEFailSafeResponseTest, MasterFailSafeResponseExtractsErrorCode) {
    advanceToData();

    slave->triggerFailSafe(ErrorCode::CRCError);
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time + 15));

    // ETG.5100 §8.2.2.6: FailSafeData PDU has no error code field.
    // The master does NOT enter fail-safe and does NOT extract an error code.
    // The master stays in Data state with data_valid=false.
    EXPECT_FALSE(conn->isFailSafe());
    EXPECT_FALSE(conn->getStatus().data_valid);
    EXPECT_EQ(conn->getErrorCode(), ErrorCode::NoError);
}

TEST_F(FSoEFailSafeResponseTest, FailSafeDataConstant) {
    EXPECT_EQ(Command::FailSafeData, 0x08);
    EXPECT_NE(Command::FailSafeData, Command::ProcessData);
}

// ============================================================================
// Fix #4: Sequence numbers removed per ETG.5100 — frames use interleaved CRC
// ============================================================================

class FSoESequenceNonStrictTest : public ::testing::Test {
protected:
    void SetUp() override {
        FSoESlaveConfig cfg{};
        cfg.slaveAddress = 0x0001;
        cfg.connectionId = 0x5678;
        cfg.safetyAddress = 0x0100;
        cfg.safetyLevel = SIL::SIL2;
        cfg.watchdogTimeoutMs = 200;
        cfg.connectionTimeoutMs = 2000;
        cfg.sessionTimeoutMs = 10000;
        cfg.safeInputSize = 2;
        cfg.safeOutputSize = 2;
        cfg.autoRecoveryEnabled = false;
        cfg.strictCrcCheck = true;
        slave = std::make_unique<FSoESlave>(cfg);
        slave->initialize();

        // Set up a master connection to advance the slave to Data state
        MasterConnectionConfig mcfg{};
        mcfg.slave_addr = 0x0001;
        mcfg.slave_safety_addr = 0x0100;
        mcfg.connection_id = 0x5678;
        mcfg.master_addr = 0x0100;
        mcfg.watchdog_timeout_ms = 200;
        mcfg.conn_timeout_ms = 2000;
        mcfg.input_size = 2;
        mcfg.output_size = 2;
        mcfg.safety_level = SIL::SIL2;
        mcfg.fail_safe_values = {0, 0, 0, 0, 0, 0, 0, 0};
        conn = std::make_unique<FSoEMasterConnection>(mcfg);
        conn->initialize();
        conn->startConnection();
    }

    void advanceToData() {
        uint64_t now = 0;
        for (int i = 0; i < 20; ++i) {
            now += 15;
            ASSERT_TRUE(conn->exchangeWith(*slave, now));
            if (conn->isOperational()) break;
        }
        ASSERT_TRUE(conn->isOperational())
            << "Master state: " << (int)conn->getState()
            << " Slave state: " << (int)slave->getState();
    }

    std::unique_ptr<FSoESlave> slave;
    std::unique_ptr<FSoEMasterConnection> conn;
};

TEST_F(FSoESequenceNonStrictTest, FrameAcceptedWithoutSequenceField) {
    // ETG.5100 does not define a sequence number field.
    // Frame integrity is ensured via interleaved CRC + watchdog.
    // Advance slave to Data state first via proper handshake
    advanceToData();

    // Build a valid ProcessData frame with new format:
    //   [CMD] [Data0(2B)] [CRC0(2B)] [ConnID(2B)]
    // Use the slave's current RX CRC state (non-zero after advanceToData).
    uint8_t data[2] = {0xAA, 0x55};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            data, 2, 0x5678,
                                            slave->getRxLastCrc0(),
                                            slave->getRxSeqNo());
    EXPECT_GT(frame_len, 0);

    // Slave should accept the frame (no sequence check needed)
    EXPECT_TRUE(slave->processRxFrame(frame, frame_len));
}

TEST_F(FSoESequenceNonStrictTest, NoSequenceErrorsCounter) {
    // Since ETG.5100 has no sequence field, sequence errors should never increment
    advanceToData();

    uint8_t data[2] = {0xAA, 0x55};
    uint8_t frame[64];
    size_t frame_len = CRC::buildFSoEFrame(frame, Command::ProcessData,
                                            data, 2, 0x5678);

    auto stats_before = slave->getStats().sequenceErrors;
    slave->processRxFrame(frame, frame_len);
    auto stats_after = slave->getStats().sequenceErrors;

    // No sequence errors should be recorded
    EXPECT_EQ(stats_after, stats_before);
}

// ============================================================================
// Fix #5: Thread safety — concurrent access doesn't crash
// ============================================================================

TEST(FSoEThreadSafetyTest, ConcurrentProcessAndPrepare) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0001;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = 0x1234;
    cfg.input_size = 4;
    cfg.output_size = 4;
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};

    FSoEMasterConnection conn(cfg);
    conn.initialize();
    conn.startConnection();

    FSoESlaveConfig slave_cfg{};
    slave_cfg.slaveAddress = 0x0001;
    slave_cfg.connectionId = 0x1234;
    slave_cfg.safetyAddress = 0x0100;
    slave_cfg.safeInputSize = 4;
    slave_cfg.safeOutputSize = 4;
    slave_cfg.autoRecoveryEnabled = false;
    slave_cfg.strictSequenceCheck = true;
    FSoESlave slave(slave_cfg);
    slave.initialize();

    std::atomic<bool> running{true};
    std::atomic<int> iterations{0};

    // Thread 1: exchangeWith cycles
    std::thread t1([&]() {
        uint64_t now = 0;
        while (running) {
            now += 15;
            conn.exchangeWith(slave, now);
            iterations++;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    // Thread 2: read state
    std::thread t2([&]() {
        while (running) {
            (void)conn.getState();
            (void)conn.isOperational();
            (void)conn.isFailSafe();
            (void)conn.getErrorCode();
            (void)conn.areSafeInputsValid();
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    // Thread 3: read diagnostics
    std::thread t3([&]() {
        while (running) {
            (void)conn.getDiagnostics();
            (void)conn.getStats().frames_sent;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    // Thread 4: write safe outputs
    std::thread t4([&]() {
        uint8_t data[4] = {1, 2, 3, 4};
        while (running) {
            conn.setSafeOutputs(data, 4);
            conn.setSafeOutputBit(0, true);
            conn.setSafeOutputByte(1, 0xAA);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    running = false;

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    // If we get here without a crash or data race sanitizer error, the test passes
    EXPECT_GT(iterations.load(), 0);
}

TEST(FSoEThreadSafetyTest, ConcurrentMasterAccess) {
    FSoEMaster master;

    MasterConnectionConfig cfg1{};
    cfg1.slave_addr = 0x0001;
    cfg1.slave_safety_addr = 0x0100;
    cfg1.connection_id = 0x1111;
    cfg1.input_size = 2;
    cfg1.output_size = 2;

    MasterConnectionConfig cfg2{};
    cfg2.slave_addr = 0x0002;
    cfg2.slave_safety_addr = 0x0200;
    cfg2.connection_id = 0x2222;
    cfg2.input_size = 2;
    cfg2.output_size = 2;

    master.addConnection(cfg1);
    master.addConnection(cfg2);

    std::atomic<bool> running{true};

    std::thread t1([&]() {
        while (running) {
            master.update(100);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    std::thread t2([&]() {
        while (running) {
            (void)master.allOperational();
            (void)master.anyFailSafe();
            (void)master.getConnectionCount();
            (void)master.getDiagnostics();
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::thread t3([&]() {
        while (running) {
            (void)master.getConnection(0x1111);
            (void)master.getConnectionBySlaveAddr(0x0001);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    running = false;

    t1.join();
    t2.join();
    t3.join();

    SUCCEED();
}

// ============================================================================
// Fix #4: Master skips all-zeros TxPDO on first PDO cycle
// ============================================================================
//
// When using exchangeViaPDO() with a real drive, the TxPDO buffer is all
// zeros on the first cycle(s) before the slave has written a response.
// An all-zero frame passes CRC trivially (CRC-16 of {0,0} with init 0x0000
// is 0x0000) and would otherwise trigger a spurious ConnectionIDError
// (conn_id=0 != configured).  The master must silently skip frames with
// unrecognized command bytes (0x00 is not a valid FSoE command) and keep
// retrying instead of going to fail-safe.

class FSoEStaleTxPDOTest : public ::testing::Test {
protected:
    void SetUp() override {
        MasterConnectionConfig cfg{};
        cfg.slave_addr = 0x0100;
        cfg.slave_safety_addr = 0x0100;
        cfg.connection_id = 0x1234;
        cfg.master_addr = 0x0100;
        cfg.watchdog_timeout_ms = 100;
        cfg.conn_timeout_ms = 2000;
        cfg.session_timeout_ms = 5000;
        cfg.input_size = 4;
        cfg.output_size = 4;
        cfg.fail_safe_values = {0, 0, 0, 0, 0, 0, 0, 0};
        // Use a generous stale budget so the stale-exhaustion logic
        // doesn't fire during the test (these tests verify invalid
        // command byte handling, not stale budget exhaustion).
        cfg.slave_response_delay_cycles = 20;
        conn = std::make_unique<FSoEMasterConnection>(cfg);
        conn->initialize();
        conn->startConnection();
    }

    std::unique_ptr<FSoEMasterConnection> conn;
};

TEST_F(FSoEStaleTxPDOTest, AllZeroTxPDODoesNotTriggerFailSafe) {
    // Simulate the first few PDO cycles where the drive hasn't populated
    // the TxPDO buffer yet.  Use 31 bytes (the real FSoE TxPDO size for
    // Synapticon).  Build a frame with cmd=0x00 (invalid command) and
    // valid CRCs so the frame passes CRC verification but is silently
    // skipped by the command check — the master must not enter fail-safe.
    constexpr size_t kTxPdoSize = 31;
    std::vector<uint8_t> rx_pdo(64, 0);
    std::vector<uint8_t> tx_pdo(kTxPdoSize, 0);

    uint64_t now = 0;
    for (int i = 0; i < 10; ++i) {
        now += 1;  // 1 ms per cycle

        // Build a stale-response frame with invalid command (0x00) but
        // correct FSoE CRCs.  31-byte frame → 14 data bytes.
        std::vector<uint8_t> stale_data(14, 0);
        CRC::buildFSoEFrame(tx_pdo.data(), 0x00,
                            stale_data.data(), 14, 0,
                            conn->getRxLastCrc0(), conn->getRxSeqNo());

        // exchangeViaPDO should return false (frame skipped or no valid
        // response) but NOT trigger fail-safe or any error.
        const bool ok = conn->exchangeViaPDO(
            rx_pdo.data(), rx_pdo.size(),
            tx_pdo.data(), tx_pdo.size(),
            now);

        EXPECT_FALSE(ok) << "Cycle " << i << ": exchange should return false for stale TxPDO";
        EXPECT_FALSE(conn->isFailSafe())
            << "Cycle " << i << ": master must not enter fail-safe from stale TxPDO";
        EXPECT_EQ(conn->getErrorCode(), ErrorCode::NoError)
            << "Cycle " << i << ": no error code expected from stale TxPDO";
    }

    // The master should still be in Session state (or Reset→Session),
    // waiting for a valid slave response.
    EXPECT_NE(conn->getState(), ConnectionState::FailSafe);
    EXPECT_NE(conn->getState(), ConnectionState::Error);
}

TEST_F(FSoEStaleTxPDOTest, MasterRecoversWhenSlaveResponds) {
    // After several stale TxPDO cycles, verify the master can still
    // complete the handshake.  We use the synchronous exchangeWith()
    // path (emulator) for the recovery phase because the FSoE handshake
    // state machine expects same-cycle responses (as in DC-synchronized
    // PDO exchange).  The key assertion is that stale frames don't
    // leave the master in an unrecoverable error/fail-safe state.
    constexpr size_t kPdoSize = 31;
    std::vector<uint8_t> rx_pdo(kPdoSize, 0);
    std::vector<uint8_t> tx_pdo(kPdoSize, 0);

    // Set up a slave emulator for the recovery phase
    FSoESlaveConfig slave_cfg{};
    slave_cfg.slaveAddress = 0x0100;
    slave_cfg.connectionId = 0x1234;
    slave_cfg.safetyAddress = 0x0100;
    slave_cfg.safetyLevel = SIL::SIL2;
    slave_cfg.watchdogTimeoutMs = 200;
    slave_cfg.connectionTimeoutMs = 2000;
    slave_cfg.sessionTimeoutMs = 10000;
    slave_cfg.safeInputSize = 4;
    slave_cfg.safeOutputSize = 4;
    slave_cfg.autoRecoveryEnabled = false;
    slave_cfg.strictCrcCheck = true;
    slave_cfg.strictSequenceCheck = true;
    auto slave = std::make_unique<FSoESlave>(slave_cfg);
    slave->initialize();

    uint64_t now = 0;

    // First 3 cycles: stale TxPDO via exchangeViaPDO.
    // Build frames with cmd=0x00 (invalid command) and valid CRCs so
    // they pass CRC but are silently skipped — no fail-safe.
    for (int i = 0; i < 3; ++i) {
        now += 1;
        std::vector<uint8_t> stale_data(14, 0);
        CRC::buildFSoEFrame(tx_pdo.data(), 0x00,
                            stale_data.data(), 14, 0,
                            conn->getRxLastCrc0(), conn->getRxSeqNo());
        conn->exchangeViaPDO(rx_pdo.data(), rx_pdo.size(),
                             tx_pdo.data(), tx_pdo.size(), now);
        ASSERT_FALSE(conn->isFailSafe()) << "Stale cycle " << i;
    }

    // The master should still be in a non-error state.
    ASSERT_NE(conn->getState(), ConnectionState::FailSafe);
    ASSERT_NE(conn->getState(), ConnectionState::Error);

    // Reset the connection to synchronise CRC state with the fresh slave,
    // then complete the handshake via the synchronous exchange path.
    conn->resetConnection();
    for (int i = 0; i < 30; ++i) {
        now += 1;
        ASSERT_TRUE(conn->exchangeWith(*slave, now))
            << "exchangeWith failed at cycle " << i
            << " (master state=" << (int)conn->getState() << ")";
        if (conn->isOperational()) break;
    }

    EXPECT_TRUE(conn->isOperational())
        << "Master should reach Data state after slave starts responding";
}

// ============================================================================
// Fix #5: Master ignores duplicate frames (slave re-sends same response)
// ============================================================================
//
// In cyclic PDO exchange, the slave may re-send the same response frame if
// it hasn't seen a new master frame yet (e.g. due to PDO pipeline delay).
// The master must not re-process identical frames — doing so could cause
// spurious state transitions, watchdog resets, or error handling.  Duplicates
// are counted for diagnostics but otherwise ignored.

class FSoEDuplicateFrameTest : public ::testing::Test {
protected:
    void SetUp() override {
        MasterConnectionConfig cfg{};
        cfg.slave_addr = 0x0100;
        cfg.slave_safety_addr = 0x0100;
        cfg.connection_id = 0x1234;
        cfg.master_addr = 0x0100;
        cfg.watchdog_timeout_ms = 100;
        cfg.input_size = 4;
        cfg.output_size = 4;
        cfg.fail_safe_values = {0, 0, 0, 0, 0, 0, 0, 0};
        conn = std::make_unique<FSoEMasterConnection>(cfg);
        conn->initialize();
        conn->startConnection();

        FSoESlaveConfig slave_cfg{};
        slave_cfg.slaveAddress = 0x0100;
        slave_cfg.connectionId = 0x1234;
        slave_cfg.safetyAddress = 0x0100;
        slave_cfg.safetyLevel = SIL::SIL2;
        slave_cfg.watchdogTimeoutMs = 200;
        slave_cfg.connectionTimeoutMs = 2000;
        slave_cfg.sessionTimeoutMs = 10000;
        slave_cfg.safeInputSize = 4;
        slave_cfg.safeOutputSize = 4;
        slave_cfg.autoRecoveryEnabled = false;
        slave_cfg.strictCrcCheck = true;
        slave_cfg.strictSequenceCheck = true;
        slave = std::make_unique<FSoESlave>(slave_cfg);
        slave->initialize();
    }

    void advanceToData() {
        uint64_t now = 0;
        for (int i = 0; i < 20; ++i) {
            now += 15;
            ASSERT_TRUE(conn->exchangeWith(*slave, now));
            if (conn->isOperational()) break;
        }
        last_time = now;
        ASSERT_TRUE(conn->isOperational());
    }

    std::unique_ptr<FSoEMasterConnection> conn;
    std::unique_ptr<FSoESlave> slave;
    uint64_t last_time = 0;
};

TEST_F(FSoEDuplicateFrameTest, DuplicateFrameIsCountedNotProcessed) {
    advanceToData();

    // Do one more exchange so the master has a fresh last_rx_frame_
    last_time += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time));

    // Force the master back to handshake phase to test duplicate
    // detection.  In Data state, duplicates are expected (constant inputs)
    // and must be processed for watchdog updates.
    conn->resetConnection();
    slave->reset();
    // Two exchanges to reach Connection state:
    //   1: Reset→Session (master sends Reset, slave responds with Session)
    //   2: Session→Connection (master sends Session, slave responds with Session)
    last_time += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time));
    ASSERT_EQ(conn->getState(), ConnectionState::Session);

    // Do the second exchange manually so we can capture the slave's
    // exact response bytes (CRC state changes with each frame, so
    // calling prepareTxFrame again would build a different frame).
    std::array<uint8_t, 64> master_tx{};
    std::array<uint8_t, 64> slave_resp{};
    size_t master_tx_len = conn->prepareTxFrame(master_tx.data(), master_tx.size());
    ASSERT_GT(master_tx_len, 0u);
    ASSERT_TRUE(slave->processRxFrame(master_tx.data(), master_tx_len));
    size_t resp_len = slave->prepareTxFrame(slave_resp.data(), slave_resp.size());
    ASSERT_GT(resp_len, 0u);
    // Master processes the slave's response → transitions to Connection.
    ASSERT_TRUE(conn->processRxFrame(slave_resp.data(), resp_len));

    const auto stats_before = conn->getStats();
    const auto state_before = conn->getState();

    // Re-send the exact same bytes — this is a duplicate in a handshake
    // state and should be counted, not processed.
    ASSERT_FALSE(conn->processRxFrame(slave_resp.data(), resp_len))
        << "Duplicate frame in handshake state should return false";

    const auto stats_after = conn->getStats();

    EXPECT_EQ(stats_after.duplicate_frames, stats_before.duplicate_frames + 1);
    EXPECT_EQ(stats_after.frames_received, stats_before.frames_received + 1);
    EXPECT_EQ(conn->getState(), state_before);
    EXPECT_FALSE(conn->isFailSafe());
    EXPECT_EQ(conn->getErrorCode(), ErrorCode::NoError);
}

TEST_F(FSoEDuplicateFrameTest, MultipleDuplicatesAllCounted) {
    advanceToData();

    // Reset to handshake phase
    conn->resetConnection();
    slave->reset();
    // Two exchanges to reach Connection state (Reset→Session→Connection)
    last_time += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time));

    // Do the second exchange manually to capture the slave's exact response.
    std::array<uint8_t, 64> master_tx{};
    std::array<uint8_t, 64> slave_resp{};
    size_t master_tx_len = conn->prepareTxFrame(master_tx.data(), master_tx.size());
    ASSERT_GT(master_tx_len, 0u);
    ASSERT_TRUE(slave->processRxFrame(master_tx.data(), master_tx_len));
    size_t resp_len = slave->prepareTxFrame(slave_resp.data(), slave_resp.size());
    ASSERT_GT(resp_len, 0u);
    ASSERT_TRUE(conn->processRxFrame(slave_resp.data(), resp_len));
    // Master is now in Connection state.

    // Send the same frame 5 times — all should be duplicates in handshake state
    for (int i = 0; i < 5; ++i) {
        EXPECT_FALSE(conn->processRxFrame(slave_resp.data(), resp_len))
            << "Duplicate " << i << " should not be processed";
    }

    const auto stats = conn->getStats();
    EXPECT_EQ(stats.duplicate_frames, 5u);
    EXPECT_FALSE(conn->isFailSafe());
}

TEST_F(FSoEDuplicateFrameTest, DifferentFrameIsProcessed) {
    advanceToData();

    // Reset to handshake phase
    conn->resetConnection();
    slave->reset();
    // Two exchanges to reach Connection state (Reset→Session→Connection)
    last_time += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time));

    // Do the second exchange manually to capture the slave's exact response.
    std::array<uint8_t, 64> master_tx{};
    std::array<uint8_t, 64> slave_resp{};
    size_t master_tx_len = conn->prepareTxFrame(master_tx.data(), master_tx.size());
    ASSERT_GT(master_tx_len, 0u);
    ASSERT_TRUE(slave->processRxFrame(master_tx.data(), master_tx_len));
    size_t len = slave->prepareTxFrame(slave_resp.data(), slave_resp.size());
    ASSERT_GT(len, 0u);
    // Master processes the slave's response.
    ASSERT_TRUE(conn->processRxFrame(slave_resp.data(), len));
    // Master is in Connection state.

    // Verify the same bytes are indeed a duplicate
    ASSERT_FALSE(conn->processRxFrame(slave_resp.data(), len));

    // Build a different but valid FSoE frame using the master's current
    // RX CRC state (which was NOT updated by the duplicate above).
    // Use a Connection response with different data.
    uint8_t diff_payload[] = {0x00, 0x02, 0x00, 0x00};
    std::array<uint8_t, 64> tx_new{};
    size_t new_len = CRC::buildFSoEFrame(tx_new.data(), Command::Connection,
                                          diff_payload, 4, 0x1234,
                                          conn->getRxLastCrc0(),
                                          conn->getRxSeqNo());
    ASSERT_GT(new_len, 0u);

    // Different frame should be processed (not a duplicate)
    const auto stats_before = conn->getStats();
    const bool ok = conn->processRxFrame(tx_new.data(), new_len);
    const auto stats_after = conn->getStats();

    EXPECT_TRUE(ok);
    EXPECT_EQ(stats_after.duplicate_frames, stats_before.duplicate_frames);
}

TEST_F(FSoEDuplicateFrameTest, DataStateDuplicateIsProcessed) {
    advanceToData();

    // In Data state, duplicate frames must still be processed to update
    // the watchdog timestamp.  The duplicate counter should NOT increment.
    last_time += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time));

    // Capture the slave's response (same as what was just processed)
    std::array<uint8_t, 64> tx{};
    const size_t resp_len = slave->prepareTxFrame(tx.data(), tx.size());
    ASSERT_GT(resp_len, 0u);

    const auto stats_before = conn->getStats();
    const auto status_before = conn->getStatus();

    // In Data state, this duplicate should be processed (not skipped)
    const bool ok = conn->processRxFrame(tx.data(), resp_len);

    const auto stats_after = conn->getStats();
    const auto status_after = conn->getStatus();

    // It should be processed successfully
    EXPECT_TRUE(ok);
    // Duplicate counter should NOT increment in Data state
    EXPECT_EQ(stats_after.duplicate_frames, stats_before.duplicate_frames);
    // Watchdog timestamp should be updated
    EXPECT_GE(status_after.last_valid_frame_ms, status_before.last_valid_frame_ms);
    EXPECT_FALSE(conn->isFailSafe());
}

// ============================================================================
// Fix #6: Master sends Reset command before starting Session handshake
// ============================================================================
//
// Per ETG.5100, the master must first send a Reset command (0x2A) to force
// the slave back to its initial state before beginning the Session handshake.
// This is critical when the slave is in a higher state (Connection, Parameter,
// Data, or Error) from a previous run that didn't shut down cleanly — a
// Session command alone would be rejected by a slave in Connection/Parameter
// state.

class FSoEResetFirstTest : public ::testing::Test {
protected:
    void SetUp() override {
        MasterConnectionConfig cfg{};
        cfg.slave_addr = 0x0100;
        cfg.slave_safety_addr = 0x0100;
        cfg.connection_id = 0x1234;
        cfg.master_addr = 0x0100;
        cfg.watchdog_timeout_ms = 100;
        cfg.conn_timeout_ms = 2000;
        cfg.session_timeout_ms = 5000;
        cfg.input_size = 4;
        cfg.output_size = 4;
        cfg.fail_safe_values = {0, 0, 0, 0, 0, 0, 0, 0};
        conn = std::make_unique<FSoEMasterConnection>(cfg);
        conn->initialize();
        conn->startConnection();

        FSoESlaveConfig slave_cfg{};
        slave_cfg.slaveAddress = 0x0100;
        slave_cfg.connectionId = 0x1234;
        slave_cfg.safetyAddress = 0x0100;
        slave_cfg.safetyLevel = SIL::SIL2;
        slave_cfg.watchdogTimeoutMs = 200;
        slave_cfg.connectionTimeoutMs = 2000;
        slave_cfg.sessionTimeoutMs = 10000;
        slave_cfg.safeInputSize = 4;
        slave_cfg.safeOutputSize = 4;
        slave_cfg.autoRecoveryEnabled = false;
        slave_cfg.strictCrcCheck = true;
        slave_cfg.strictSequenceCheck = true;
        slave = std::make_unique<FSoESlave>(slave_cfg);
        slave->initialize();
    }

    std::unique_ptr<FSoEMasterConnection> conn;
    std::unique_ptr<FSoESlave> slave;
};

TEST_F(FSoEResetFirstTest, MasterSendsResetCommandFirst) {
    // After initialize+startConnection, the master is in Reset state.
    // prepareTxFrame must build a Reset command (0x2A), NOT a Session
    // command (0x4E).
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);

    std::array<uint8_t, 64> tx{};
    const size_t len = conn->prepareTxFrame(tx.data(), tx.size());
    ASSERT_GT(len, 0u);

    // First byte is the command byte — must be Reset (0x2A)
    EXPECT_EQ(tx[0], Command::Reset)
        << "Master must send Reset command (0x2A) first, got 0x"
        << std::hex << static_cast<int>(tx[0]);

    // Reset frame is the full fixed PDO size: CMD(1) + data(6) + CRCs(3) + ConnID(2) = 15 bytes
    // (with output_size=4, fsoeFrameSize(4)=11)
    EXPECT_EQ(len, CRC::fsoeFrameSize(4));

    // Master should still be in Reset state (not auto-transitioned to Session)
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);
}

TEST_F(FSoEResetFirstTest, MasterTransitionsToSessionAfterSlaveAck) {
    // Master sends Reset → slave processes it → slave responds with Session
    // → master transitions to Session.
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);

    // One full exchange cycle via exchangeWith (synchronous):
    //   master prepares Reset frame
    //   slave processes Reset → transitions to Session
    //   slave prepares Session response
    //   master processes Session response → transitions to Session
    uint64_t now = 10;
    ASSERT_TRUE(conn->exchangeWith(*slave, now));

    // Master should now be in Session state
    EXPECT_EQ(conn->getState(), ConnectionState::Session);
}

TEST_F(FSoEResetFirstTest, FullHandshakeCompletesAfterReset) {
    // The Reset-then-Session sequence should still allow the full handshake
    // to complete to Data state.
    uint64_t now = 0;
    for (int i = 0; i < 30; ++i) {
        now += 15;
        ASSERT_TRUE(conn->exchangeWith(*slave, now))
            << "exchangeWith failed at cycle " << i
            << " (master state=" << static_cast<int>(conn->getState()) << ")";
        if (conn->isOperational()) break;
    }

    EXPECT_TRUE(conn->isOperational())
        << "Master should reach Data state after Reset→Session→Connection→Parameter→Data";
}

TEST_F(FSoEResetFirstTest, ResetRecoversSlaveFromDataState) {
    // Simulate a slave left in Data state from a previous run.
    // The master's Reset command must bring it back to Session so the
    // handshake can proceed.
    uint64_t now = 0;
    for (int i = 0; i < 30; ++i) {
        now += 15;
        ASSERT_TRUE(conn->exchangeWith(*slave, now));
        if (conn->isOperational() && slave->getState() == ConnectionState::Data) break;
    }
    ASSERT_TRUE(conn->isOperational());
    ASSERT_EQ(slave->getStateName(), std::string("DATA"));

    // Now reset the master connection — it should send Reset first
    conn->resetConnection();
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);

    // The slave is still in Data state. The master sends Reset, the slave
    // (in Data state) accepts it and transitions back to Session.
    now += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, now));

    // Master should have transitioned to Session (slave acknowledged reset)
    EXPECT_EQ(conn->getState(), ConnectionState::Session);

    // Slave should be back in Session state
    EXPECT_EQ(slave->getStateName(), std::string("SESSION"));

    // Full handshake should complete again
    for (int i = 0; i < 30; ++i) {
        now += 15;
        ASSERT_TRUE(conn->exchangeWith(*slave, now));
        if (conn->isOperational()) break;
    }
    EXPECT_TRUE(conn->isOperational());
}

TEST_F(FSoEResetFirstTest, ResetRecoversSlaveFromConnectionState) {
    // Advance the slave to Connection state, then verify the master's
    // Reset command brings it back.
    uint64_t now = 0;
    // Three exchanges to get the slave to Connection state:
    //   1: Reset→Session (master), Reset→Session (slave)
    //   2: Session→Connection (master), Session→Session (slave)
    //   3: Connection→Parameter (master), Session→Connection (slave)
    for (int i = 0; i < 3; ++i) {
        now += 15;
        ASSERT_TRUE(conn->exchangeWith(*slave, now));
    }
    // Slave should be in Connection state now
    ASSERT_EQ(slave->getStateName(), std::string("CONNECTION"));

    // Reset master and verify it sends Reset to the slave
    conn->resetConnection();
    EXPECT_EQ(conn->getState(), ConnectionState::Reset);

    // Master sends Reset → slave in Connection state must accept it
    now += 15;
    ASSERT_TRUE(conn->exchangeWith(*slave, now));

    EXPECT_EQ(conn->getState(), ConnectionState::Session);
    EXPECT_EQ(slave->getStateName(), std::string("SESSION"));
}

TEST_F(FSoEResetFirstTest, ResetFrameHasCorrectConnectionID) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.2:
    // The Reset frame must have Conn_Id = 0 — the connection has not been
    // established yet, so there is no Connection ID to check.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    std::array<uint8_t, 64> tx{};
    const size_t len = conn->prepareTxFrame(tx.data(), tx.size());
    ASSERT_EQ(len, CRC::fsoeFrameSize(4));

    // ConnID is the last 2 bytes (little-endian) — must be 0 in Reset state
    const uint16_t conn_id = tx[len - 2] | (tx[len - 1] << 8);
    EXPECT_EQ(conn_id, 0u);
}
