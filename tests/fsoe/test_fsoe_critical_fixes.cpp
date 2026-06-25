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

#include "fsoe/FSoEConnection.hpp"
#include "fsoe/FSoESlave.hpp"

using namespace FSoE;

// ============================================================================
// Fix #3: Master recognizes slave fail-safe response (0x80)
// ============================================================================

class FSoEFailSafeResponseTest : public ::testing::Test {
protected:
    void SetUp() override {
        ConnectionConfig cfg{};
        cfg.slave_addr = 0x0100;
        cfg.slave_safety_addr = 0x0100;
        cfg.connection_id = 0x1234;
        cfg.master_addr = 0x0100;
        cfg.watchdog_timeout_ms = 100;
        cfg.input_size = 4;
        cfg.output_size = 4;
        cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};
        conn = std::make_unique<FSoEConnection>(cfg);
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

    std::unique_ptr<FSoEConnection> conn;
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

    // Exchange one cycle — slave will send fail-safe response (0x80)
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time + 15));

    // Master should now be in fail-safe
    EXPECT_TRUE(conn->isFailSafe());
    EXPECT_EQ(conn->getErrorCode(), ErrorCode::WatchdogError);
}

TEST_F(FSoEFailSafeResponseTest, MasterFailSafeResponseExtractsErrorCode) {
    advanceToData();

    slave->triggerFailSafe(ErrorCode::CRCError);
    ASSERT_TRUE(conn->exchangeWith(*slave, last_time + 15));

    EXPECT_TRUE(conn->isFailSafe());
    EXPECT_EQ(conn->getErrorCode(), ErrorCode::CRCError);
}

TEST_F(FSoEFailSafeResponseTest, FailSafeDataConstant) {
    EXPECT_EQ(Command::FailSafeData, 0x08);
    EXPECT_NE(Command::FailSafeData, Command::ProcessData);
}

// ============================================================================
// Fix #4: Slave non-strict sequence validation doesn't resync
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
        cfg.strictSequenceCheck = false;  // Non-strict mode
        cfg.treatSequenceErrorAsCritical = false;
        slave = std::make_unique<FSoESlave>(cfg);
        slave->initialize();
    }

    std::unique_ptr<FSoESlave> slave;
};

TEST_F(FSoESequenceNonStrictTest, NonStrictAcceptsWrongSequenceWithoutResync) {
    // Build a valid data frame with wrong sequence number
    // Slave expects sequence 0 initially
    uint8_t frame[64];
    frame[0] = Command::ProcessData;
    frame[1] = 0x78;  // conn_id low
    frame[2] = 0x56 | ((5 & 0x0F) << 4);  // conn_id high with seq=5 (wrong, expected 0)
    frame[3] = 0xAA;  // data byte 1
    frame[4] = 0x55;  // data byte 2
    uint16_t crc = CRC::calculateFSoECRC(frame, 5);
    frame[5] = crc & 0xFF;
    frame[6] = (crc >> 8) & 0xFF;

    // Slave should accept the frame (non-strict) but NOT resync
    EXPECT_TRUE(slave->processRxFrame(frame, 7));

    // Now send a frame with the CORRECT expected sequence (0)
    frame[2] = 0x56 | ((0 & 0x0F) << 4);  // seq=0 (still expected)
    crc = CRC::calculateFSoECRC(frame, 5);
    frame[5] = crc & 0xFF;
    frame[6] = (crc >> 8) & 0xFF;

    // Should still be accepted because expected sequence wasn't changed
    EXPECT_TRUE(slave->processRxFrame(frame, 7));

    // After accepting seq=0, expected should advance to 1
    frame[2] = 0x56 | ((1 & 0x0F) << 4);  // seq=1
    crc = CRC::calculateFSoECRC(frame, 5);
    frame[5] = crc & 0xFF;
    frame[6] = (crc >> 8) & 0xFF;
    EXPECT_TRUE(slave->processRxFrame(frame, 7));
}

TEST_F(FSoESequenceNonStrictTest, NonStrictDoesNotIncrementSequenceErrors) {
    uint8_t frame[64];
    frame[0] = Command::ProcessData;
    frame[1] = 0x78;
    frame[2] = 0x56 | ((9 & 0x0F) << 4);  // Wrong sequence
    frame[3] = 0xAA;
    frame[4] = 0x55;
    uint16_t crc = CRC::calculateFSoECRC(frame, 5);
    frame[5] = crc & 0xFF;
    frame[6] = (crc >> 8) & 0xFF;

    auto stats_before = slave->getStats().sequenceErrors;
    EXPECT_TRUE(slave->processRxFrame(frame, 7));
    auto stats_after = slave->getStats().sequenceErrors;

    // Non-strict mode should not increment sequence error counter
    EXPECT_EQ(stats_after, stats_before);
}

// ============================================================================
// Fix #5: Thread safety — concurrent access doesn't crash
// ============================================================================

TEST(FSoEThreadSafetyTest, ConcurrentProcessAndPrepare) {
    ConnectionConfig cfg{};
    cfg.slave_addr = 0x0001;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = 0x1234;
    cfg.input_size = 4;
    cfg.output_size = 4;
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};

    FSoEConnection conn(cfg);
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

    ConnectionConfig cfg1{};
    cfg1.slave_addr = 0x0001;
    cfg1.slave_safety_addr = 0x0100;
    cfg1.connection_id = 0x1111;
    cfg1.input_size = 2;
    cfg1.output_size = 2;

    ConnectionConfig cfg2{};
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
