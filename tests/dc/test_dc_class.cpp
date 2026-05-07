/**
 * @file test_dc_class.cpp
 * @brief Unit tests for class-based EtherCATDC implementation
 *
 * Tests the EtherCATDC class focusing on:
 * - Multiple independent DC instances
 * - Instance-based state management
 * - Thread-safety and RAII
 *
 * Note: Move semantics are deleted (reference member prevents move).
 * All tests now use IDCTransport-based constructor.
 */

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <atomic>

#include "tether/ethercat/EtherCATDCClass.hpp"
#include "tether/ethercat/IDCTransport.hpp"

using namespace EtherCAT;

// ============================================================================
// Minimal stub transport for these tests
// ============================================================================

class StubDCTransport : public IDCTransport {
public:
    bool readRegister(uint16_t, uint16_t, void* data, uint16_t size, unsigned int) override {
        std::memset(data, 0, size); return true;
    }
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t, unsigned int) override {
        return true;
    }
    bool sendSyncDatagram(uint16_t, uint16_t, const void*, uint16_t) override {
        return true;
    }
    uint64_t getMasterTimeNs() override { return 1000000000ULL; }
    void delayMs(uint32_t) override {}
};

// ============================================================================
// Test Fixture
// ============================================================================

class DCClassTest : public ::testing::Test {
protected:
    StubDCTransport transport_;
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(DCClassTest, ConstructorWithDefaults) {
    EtherCATDC dc(transport_, 1, nullptr);
    EXPECT_EQ(dc.getState(), DCState::Disabled);
    EXPECT_FALSE(dc.isPDOEnabled());
}

TEST_F(DCClassTest, ConstructorWithCustomConfig) {
    DCConfig cfg = DCConfig::defaults();
    cfg.cycle_period_us = 2000;
    cfg.sync_interval_cycles = 5;
    EtherCATDC dc(transport_, 1, &cfg);
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCClassTest, ConstructorZeroSlaves) {
    EtherCATDC dc(transport_, 0, nullptr);
    EXPECT_EQ(dc.getState(), DCState::Error);
}

// ============================================================================
// Multiple Instance Tests
// ============================================================================

TEST_F(DCClassTest, MultipleIndependentInstances) {
    StubDCTransport transport_b;

    EtherCATDC dc1(transport_, 1);
    EtherCATDC dc2(transport_b, 2);

    EXPECT_EQ(dc1.getState(), DCState::Disabled);
    EXPECT_EQ(dc2.getState(), DCState::Disabled);
}

// ============================================================================
// State Management Tests
// ============================================================================

TEST_F(DCClassTest, StatsIndependence) {
    StubDCTransport transport_b;
    EtherCATDC dc1(transport_, 1);
    EtherCATDC dc2(transport_b, 1);

    DCLoopStats stats1 = dc1.getStats();
    DCLoopStats stats2 = dc2.getStats();
    EXPECT_EQ(stats1.cycle_count, 0u);
    EXPECT_EQ(stats2.cycle_count, 0u);
}

// ============================================================================
// RAII Tests
// ============================================================================

TEST_F(DCClassTest, DestructorStopsAutomatically) {
    {
        EtherCATDC dc(transport_, 1);
        dc.init();
        dc.start();
    }
    // destructor completed without hang — create a fresh instance to validate transport still usable
    EtherCATDC dc2(transport_, 1);
    EXPECT_EQ(dc2.getState(), DCState::Disabled);
}

TEST_F(DCClassTest, ExplicitStopBeforeDestroy) {
    EtherCATDC dc(transport_, 1);
    dc.init();
    dc.start();
    dc.stop();
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

// ============================================================================
// Thread-Safety Tests
// ============================================================================

TEST_F(DCClassTest, ConcurrentGetState) {
    EtherCATDC dc(transport_, 1);

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&dc]() {
            for (int j = 0; j < 100; j++) {
                auto state = dc.getState();
                (void)state;
            }
        });
    }
    for (auto& t : threads) t.join();
    // Ensure calls completed and object still in valid state
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCClassTest, ConcurrentGetStats) {
    EtherCATDC dc(transport_, 1);

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&dc]() {
            for (int j = 0; j < 100; j++) {
                auto stats = dc.getStats();
                (void)stats;
            }
        });
    }
    for (auto& t : threads) t.join();
    auto stats = dc.getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
}

TEST_F(DCClassTest, ConcurrentPDOEnableDisable) {
    EtherCATDC dc(transport_, 1);

    std::atomic<bool> stop{false};
    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&dc, &stop, i]() {
            while (!stop.load()) {
                dc.setPDOEnabled(i % 2 == 0);
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop.store(true);
    for (auto& t : threads) t.join();
    // After concurrent toggles, PDO should remain disabled when loop not running
    EXPECT_FALSE(dc.isPDOEnabled());
}

// ============================================================================
// Slave Query Tests
// ============================================================================

TEST_F(DCClassTest, IsSlaveSupported) {
    EtherCATDC dc(transport_, 3);
    EXPECT_FALSE(dc.isSlaveSupported(0));
    EXPECT_FALSE(dc.isSlaveSupported(1));
    EXPECT_FALSE(dc.isSlaveSupported(2));
    EXPECT_FALSE(dc.isSlaveSupported(999));
}

TEST_F(DCClassTest, GetSlaveOffset) {
    EtherCATDC dc(transport_, 1);
    EXPECT_EQ(dc.getSlaveOffset(0), 0);
    EXPECT_EQ(dc.getSlaveOffset(999), 0);
}

// ============================================================================
// Control Functions Tests
// ============================================================================

TEST_F(DCClassTest, ForceSync) {
    EtherCATDC dc(transport_, 1);
    dc.forceSync();
    // forceSync is a no-op in this context; ensure call is safe
    EXPECT_NO_THROW(dc.forceSync());
}

TEST_F(DCClassTest, PDOEnableDisable) {
    EtherCATDC dc(transport_, 1);
    EXPECT_FALSE(dc.isPDOEnabled());
    dc.setPDOEnabled(true);
    // PDO only enabled when realtime loop is running
    // Without start(), there's no loop to delegate to
    EXPECT_FALSE(dc.isPDOEnabled());
}

TEST_F(DCClassTest, ReconfigureSync) {
    EtherCATDC dc(transport_, 1);
    bool result = dc.reconfigureSync(0);
    EXPECT_TRUE(result);
    bool result_invalid = dc.reconfigureSync(999);
    EXPECT_FALSE(result_invalid);
}

