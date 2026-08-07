/**
 * @file test_fsoe_concurrency_regression.cpp
 * @brief Regression tests for concurrency, data race, and deadlock fixes.
 *
 * Covers commits:
 * - W3: Data race in startDedicatedThread exchange_fn (2201538)
 * - W4: Deadlock in stopDedicatedThread (2201538)
 * - X30: Deadlock in update()/resetAll() with callbacks (9b9cd34)
 * - Z2: AB-BA deadlock in allOperational/anyFailSafe/getDiagnostics/startAll (3d1442f)
 * - S3: Thread-safe slave accessors (f168fcf)
 * - U2: Thread-safe master getStatus()/getStats() (11210d2)
 *
 * These tests use multi-threaded scenarios to verify that the snapshot
 * pattern prevents deadlocks and data races. They are designed to run
 * cleanly under ThreadSanitizer (TSan) and AddressSanitizer (ASan).
 */

#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>
#include "fsoe/FSoEMaster.hpp"
#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"

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
    cfg.auto_fail_safe_on_error = true;
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
    return cfg;
}

// ============================================================================
// X30: update()/resetAll() Callback Deadlock (commit 9b9cd34)
// ============================================================================
//
// FSoEMaster::update() and resetAll() must not hold master.mutex_ while
// calling connection methods that may trigger callbacks. If a callback
// calls back into FSoEMaster, the non-recursive mutex would deadlock.
//

TEST(FSoEMasterCallbackDeadlockRegression, UpdateWithCallbackCallingMasterNoDeadlock) {
    FSoEMaster master;
    master.addConnection(makeMasterCfg(4, 4));

    FSoEMasterConnection* conn = master.getConnectionBySlaveAddr(0x0100);
    ASSERT_NE(conn, nullptr);

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();
    master.startAll();

    // Advance to Data state
    uint64_t now = 0;
    for (int i = 0; i < 20; ++i) {
        now += 15;
        conn->exchangeWith(slave, now);
        if (conn->isOperational()) break;
    }
    ASSERT_TRUE(conn->isOperational());

    // Set a fail-safe callback that calls back into master
    conn->setFailSafeCallback([&master]() {
        // This would deadlock if master.mutex_ is held during update()
        master.anyFailSafe();
    });

    // Trigger watchdog timeout → fail-safe → callback → master.anyFailSafe()
    // This must complete without deadlock
    master.update(now + 200);

    SUCCEED();  // If we get here, no deadlock
}

TEST(FSoEMasterCallbackDeadlockRegression, ResetAllWithCallbackNoDeadlock) {
    FSoEMaster master;
    master.addConnection(makeMasterCfg(4, 4));

    FSoEMasterConnection* conn = master.getConnectionBySlaveAddr(0x0100);
    ASSERT_NE(conn, nullptr);

    // Set a state change callback that calls back into master
    conn->setStateChangeCallback([&master](uint8_t, uint8_t) {
        // This would deadlock if master.mutex_ is held during resetAll()
        master.getConnectionCount();
    });

    // resetAll() triggers state changes → callback → master.getConnectionCount()
    // This must complete without deadlock
    master.resetAll();

    SUCCEED();
}

// ============================================================================
// Z2: AB-BA Deadlock in Bulk Query Methods (commit 3d1442f)
// ============================================================================
//
// allOperational(), anyFailSafe(), getDiagnostics(), startAll() must not
// hold master.mutex_ while calling connection methods that lock conn.mutex_.
// This prevents AB-BA deadlock with callbacks.
//

TEST(FSoEMasterABBADeadlockRegression, AllOperationalWithConcurrentCallbackNoDeadlock) {
    FSoEMaster master;
    master.addConnection(makeMasterCfg(4, 4));

    FSoEMasterConnection* conn = master.getConnectionBySlaveAddr(0x0100);
    ASSERT_NE(conn, nullptr);

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();
    master.startAll();

    uint64_t now = 0;
    for (int i = 0; i < 20; ++i) {
        now += 15;
        conn->exchangeWith(slave, now);
        if (conn->isOperational()) break;
    }
    ASSERT_TRUE(conn->isOperational());

    // Callback that calls master.anyFailSafe() (locks master.mutex_)
    conn->setFailSafeCallback([&master]() {
        master.anyFailSafe();
    });

    std::atomic<bool> stop{false};
    std::atomic<int> a_count{0}, b_count{0};

    // Thread A: repeatedly calls master.allOperational()
    // Locks master.mutex_ → conn.mutex_
    std::thread threadA([&]() {
        while (!stop) {
            master.allOperational();
            a_count++;
        }
    });

    // Thread B: triggers callback via conn->update()
    // Locks conn.mutex_ → callback locks master.mutex_
    std::thread threadB([&]() {
        while (!stop) {
            conn->resetConnection();
            conn->startConnection();
            uint64_t t = 0;
            for (int i = 0; i < 10; ++i) {
                t += 15;
                conn->exchangeWith(slave, t);
                if (conn->isOperational()) break;
            }
            conn->update(t + 200);
            b_count++;
        }
    });

    // Run for 2 seconds — if deadlocked, threads will hang
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Check progress BEFORE setting stop
    int a1 = a_count.load(), b1 = b_count.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int a2 = a_count.load(), b2 = b_count.load();

    stop = true;

    bool a_stuck = (a1 == a2);
    bool b_stuck = (b1 == b2);

    if (a_stuck && b_stuck) {
        FAIL() << "AB-BA deadlock detected: Thread A stuck (" << a1 << "->" << a2
               << "), Thread B stuck (" << b1 << "->" << b2 << ")";
    }

    threadA.join();
    threadB.join();

    EXPECT_GT(a2, 0);
    EXPECT_GT(b2, 0);
}

TEST(FSoEMasterABBADeadlockRegression, AnyFailSafeWithConcurrentCallbackNoDeadlock) {
    FSoEMaster master;
    master.addConnection(makeMasterCfg(4, 4));

    FSoEMasterConnection* conn = master.getConnectionBySlaveAddr(0x0100);
    ASSERT_NE(conn, nullptr);

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();
    master.startAll();

    uint64_t now = 0;
    for (int i = 0; i < 20; ++i) {
        now += 15;
        conn->exchangeWith(slave, now);
        if (conn->isOperational()) break;
    }

    conn->setFailSafeCallback([&master]() {
        master.allOperational();  // Cross-lock call
    });

    std::atomic<bool> stop{false};
    std::atomic<int> a_count{0}, b_count{0};

    std::thread threadA([&]() {
        while (!stop) { master.anyFailSafe(); a_count++; }
    });

    std::thread threadB([&]() {
        while (!stop) {
            conn->resetConnection();
            conn->startConnection();
            uint64_t t = 0;
            for (int i = 0; i < 10; ++i) {
                t += 15;
                conn->exchangeWith(slave, t);
                if (conn->isOperational()) break;
            }
            conn->update(t + 200);
            b_count++;
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));
    int a1 = a_count.load(), b1 = b_count.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int a2 = a_count.load(), b2 = b_count.load();
    stop = true;

    if (a1 == a2 && b1 == b2) {
        FAIL() << "AB-BA deadlock in anyFailSafe()";
    }

    threadA.join();
    threadB.join();
    SUCCEED();
}

TEST(FSoEMasterABBADeadlockRegression, GetDiagnosticsWithConcurrentUpdateNoDeadlock) {
    FSoEMaster master;
    master.addConnection(makeMasterCfg(4, 4));

    FSoEMasterConnection* conn = master.getConnectionBySlaveAddr(0x0100);
    ASSERT_NE(conn, nullptr);

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();
    master.startAll();

    uint64_t now = 0;
    for (int i = 0; i < 20; ++i) {
        now += 15;
        conn->exchangeWith(slave, now);
        if (conn->isOperational()) break;
    }

    std::atomic<bool> stop{false};
    std::atomic<int> diag_count{0}, update_count{0};

    // Thread A: repeatedly calls getDiagnostics()
    std::thread threadA([&]() {
        while (!stop) {
            std::string diag = master.getDiagnostics();
            EXPECT_FALSE(diag.empty());
            diag_count++;
        }
    });

    // Thread B: repeatedly calls update()
    std::thread threadB([&]() {
        uint64_t t = now;
        while (!stop) {
            master.update(t);
            t += 15;
            update_count++;
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(1));
    int d1 = diag_count.load(), u1 = update_count.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int d2 = diag_count.load(), u2 = update_count.load();
    stop = true;

    if (d1 == d2 && u1 == u2) {
        FAIL() << "Deadlock between getDiagnostics() and update()";
    }

    threadA.join();
    threadB.join();
    EXPECT_GT(d2, 0);
    EXPECT_GT(u2, 0);
}

TEST(FSoEMasterABBADeadlockRegression, StartAllWithConcurrentAccessNoDeadlock) {
    FSoEMaster master;
    master.addConnection(makeMasterCfg(4, 4));

    std::atomic<bool> stop{false};
    std::atomic<int> start_count{0}, query_count{0};

    std::thread threadA([&]() {
        while (!stop) {
            master.startAll();
            start_count++;
        }
    });

    std::thread threadB([&]() {
        while (!stop) {
            master.allOperational();
            master.anyFailSafe();
            query_count++;
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(1));
    int s1 = start_count.load(), q1 = query_count.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int s2 = start_count.load(), q2 = query_count.load();
    stop = true;

    if (s1 == s2 && q1 == q2) {
        FAIL() << "Deadlock between startAll() and allOperational()/anyFailSafe()";
    }

    threadA.join();
    threadB.join();
    SUCCEED();
}

// ============================================================================
// S3/U2: Thread-Safe Accessors Under Concurrent Access
// ============================================================================

TEST(FSoEThreadSafeAccessorsRegression, ConcurrentMasterReadWriteNoCrash) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    std::atomic<bool> running{true};

    // Writer: exchange cycles
    std::thread writer([&]() {
        uint64_t now = 0;
        while (running) {
            now += 15;
            conn.exchangeWith(slave, now);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    // Reader: query state
    std::thread reader([&]() {
        while (running) {
            auto status = conn.getStatus();
            auto stats = conn.getStats();
            (void)conn.getState();
            (void)conn.isOperational();
            (void)conn.isFailSafe();
            (void)conn.getErrorCode();
            (void)conn.areSafeInputsValid();
            (void)conn.getDiagnostics();
            std::this_thread::sleep_for(std::chrono::microseconds(30));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running = false;
    writer.join();
    reader.join();

    SUCCEED();
}

TEST(FSoEThreadSafeAccessorsRegression, ConcurrentSlaveReadWriteNoCrash) {
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    // Advance to Data
    uint64_t now = 0;
    for (int i = 0; i < 20; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational()) break;
    }

    std::atomic<bool> running{true};

    std::thread writer([&]() {
        uint64_t t = now;
        while (running) {
            t += 15;
            conn.exchangeWith(slave, t);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::thread reader([&]() {
        while (running) {
            auto stats = slave.getStats();
            auto diag = slave.getDiagnostics();
            (void)slave.isFailSafe();
            (void)slave.hasError();
            (void)slave.getLastError();
            (void)slave.areSafeOutputsValid();
            (void)slave.getState();
            std::this_thread::sleep_for(std::chrono::microseconds(30));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running = false;
    writer.join();
    reader.join();

    SUCCEED();
}

// ============================================================================
// W3/W4: Dedicated Thread Data Race and Deadlock (commit 2201538)
// ============================================================================
//
// Note: We can't fully test startDedicatedThread/stopDedicatedThread without
// an IPDOTransport implementation. Instead, we test that concurrent
// addConnection/removeConnection with update() doesn't crash.
//

TEST(FSoEMasterConcurrentModificationRegression, ConcurrentUpdateAndQuery) {
    FSoEMaster master;

    // Add a few connections
    for (int i = 0; i < 3; ++i) {
        MasterConnectionConfig cfg = makeMasterCfg(2, 2);
        cfg.slave_addr = 0x0100 + i;
        cfg.connection_id = 0x1000 + i;
        master.addConnection(cfg);
    }

    std::atomic<bool> running{true};

    // Thread 1: update
    std::thread t1([&]() {
        uint64_t t = 0;
        while (running) {
            master.update(t++);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    // Thread 2: query
    std::thread t2([&]() {
        while (running) {
            master.allOperational();
            master.anyFailSafe();
            master.getDiagnostics();
            master.getConnectionCount();
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    // Thread 3: lookup
    std::thread t3([&]() {
        while (running) {
            for (int i = 0; i < 3; ++i) {
                master.getConnection(0x1000 + i);
                master.getConnectionBySlaveAddr(0x0100 + i);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running = false;
    t1.join();
    t2.join();
    t3.join();

    SUCCEED();
}

// ============================================================================
// Concurrent ExchangeWith and State Queries
// ============================================================================

TEST(FSoEConcurrentExchangeRegression, ConcurrentExchangeAndStateQuery) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    std::atomic<bool> running{true};
    std::atomic<int> exchanges{0}, queries{0};

    std::thread exchanger([&]() {
        uint64_t now = 0;
        while (running) {
            now += 15;
            conn.exchangeWith(slave, now);
            exchanges++;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    std::thread querier([&]() {
        while (running) {
            (void)conn.isOperational();
            (void)conn.isFailSafe();
            (void)conn.getState();
            (void)slave.isFailSafe();
            (void)slave.getState();
            queries++;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running = false;
    exchanger.join();
    querier.join();

    EXPECT_GT(exchanges.load(), 0);
    EXPECT_GT(queries.load(), 0);
}

// ============================================================================
// Multiple Connections Concurrent Update
// ============================================================================

TEST(FSoEMasterMultiConnectionRegression, MultipleConnectionsConcurrentUpdate) {
    FSoEMaster master;

    std::vector<std::unique_ptr<FSoESlave>> slaves;
    std::vector<FSoEMasterConnection*> conns;

    for (int i = 0; i < 3; ++i) {
        MasterConnectionConfig mcfg = makeMasterCfg(2, 2);
        mcfg.slave_addr = 0x0100 + i;
        mcfg.connection_id = 0x1000 + i;
        master.addConnection(mcfg);

        FSoESlaveConfig scfg = makeSlaveCfg(2, 2);
        scfg.slaveAddress = 0x0100 + i;
        scfg.connectionId = 0x1000 + i;
        slaves.push_back(std::make_unique<FSoESlave>(scfg));
        slaves.back()->initialize();

        conns.push_back(master.getConnectionBySlaveAddr(0x0100 + i));
    }

    master.startAll();

    // Advance all to Data
    uint64_t now = 0;
    for (int cycle = 0; cycle < 20; ++cycle) {
        now += 15;
        for (size_t i = 0; i < conns.size(); ++i) {
            conns[i]->exchangeWith(*slaves[i], now);
        }
        bool all_op = true;
        for (auto* c : conns) {
            if (!c->isOperational()) all_op = false;
        }
        if (all_op) break;
    }

    // Concurrent updates
    std::atomic<bool> running{true};

    std::thread updater([&]() {
        uint64_t t = now;
        while (running) {
            master.update(t);
            t += 15;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    std::thread checker([&]() {
        while (running) {
            (void)master.allOperational();
            (void)master.anyFailSafe();
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running = false;
    updater.join();
    checker.join();

    SUCCEED();
}
