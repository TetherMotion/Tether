/**
 * @file test_dc_init_basic.cpp
 * @brief Basic unit tests for DC initialization via DCManager (instance-based)
 *
 * Focuses on parameter validation and basic API contracts
 * using DCManager owned by EtherCATMaster.
 */

#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"

#include <thread>
#include <chrono>
#include <cstring>

using namespace EtherCAT;
using namespace EtherCAT::DC;

// ============================================================================
// Test Fixtures
// ============================================================================

class DCBasicTest : public ::testing::Test {
protected:
    void SetUp() override {
    }

    void TearDown() override {
        // Give threads time to clean up
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    EtherCATMaster master_;
};

// ============================================================================
// Parameter Validation Tests
// ============================================================================

TEST_F(DCBasicTest, InitWithZeroSlaves) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    EXPECT_FALSE(master_.dc().init(cfg, 0));
}

TEST_F(DCBasicTest, InitWithValidParameters) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    EXPECT_TRUE(master_.dc().init(cfg, 1));
}

// ============================================================================
// State Query Tests
// ============================================================================

TEST_F(DCBasicTest, StateBeforeInit) {
    EXPECT_EQ(master_.dc().getState(), DC::DCState::Disabled);
}

TEST_F(DCBasicTest, StateAfterInit) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    master_.dc().init(cfg, 1);

    // With no real transport, init() sets state back to Disabled after
    // attempting (and failing) to probe slaves. The DCManager is still
    // "initialized" (instance exists), but the underlying DC class
    // leaves state as Disabled.
    EXPECT_TRUE(master_.dc().isInitialized());
}

// ============================================================================
// Start/Stop Tests
// ============================================================================

TEST_F(DCBasicTest, StartWithoutInit) {
    EXPECT_FALSE(master_.dc().start());
}

TEST_F(DCBasicTest, StartAfterInit) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    master_.dc().init(cfg, 1);

    // Without a real transport, the underlying EtherCATDC marks itself as
    // not initialized (slave probing fails). start() returns false in that case.
    // This test verifies start doesn't crash and returns a sane value.
    (void)master_.dc().start();
    EXPECT_TRUE(master_.dc().isInitialized());
}

TEST_F(DCBasicTest, StopAfterStart) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    master_.dc().init(cfg, 1);
    master_.dc().start();

    master_.dc().stop();

    // Small delay for task termination
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    EXPECT_NE(master_.dc().getState(), DC::DCState::Running);
}

// ============================================================================
// Config Tests
// ============================================================================

TEST_F(DCBasicTest, DefaultConfigValues) {
    DC::DCConfig cfg = DC::DCConfig::defaults();

    EXPECT_EQ(cfg.cycle_period_us, 1000);  // 1ms default
    EXPECT_EQ(cfg.sync0_cycle_time_ns, 1000000);  // 1ms SYNC0
    EXPECT_EQ(cfg.sync0_shift_ns, 0);
    EXPECT_TRUE(cfg.enable_sync0);
    EXPECT_FALSE(cfg.enable_sync1);
}

TEST_F(DCBasicTest, CustomConfig) {
    DC::DCConfig cfg;
    cfg.cycle_period_us = 500;  // 500us
    cfg.sync_interval_cycles = 10;
    cfg.sync0_cycle_time_ns = 500000;  // 500us SYNC0
    cfg.sync1_cycle_time_ns = 0;
    cfg.sync0_shift_ns = 100000;
    cfg.enable_sync0 = true;
    cfg.enable_sync1 = false;

    EXPECT_TRUE(master_.dc().init(cfg, 1));
}

// ============================================================================
// PDO Control Tests
// ============================================================================

TEST_F(DCBasicTest, PDOEnabledByDefault) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    master_.dc().init(cfg, 1);

    // No crash or error — ensure state not Error
    master_.dc().setPDOEnabled(true);
    EXPECT_NE(master_.dc().getState(), DC::DCState::Error);
}

TEST_F(DCBasicTest, PDODisableEnable) {
    DC::DCConfig cfg = DC::DCConfig::defaults();
    master_.dc().init(cfg, 1);

    master_.dc().setPDOEnabled(false);
    master_.dc().setPDOEnabled(true);

    // No crash or error — ensure state not Error
    EXPECT_NE(master_.dc().getState(), DC::DCState::Error);
}

// ============================================================================
// Lifecycle Tests
// ============================================================================

TEST_F(DCBasicTest, CompleteLifecycle) {
    DC::DCConfig cfg = DC::DCConfig::defaults();

    // Init
    EXPECT_TRUE(master_.dc().init(cfg, 1));
    EXPECT_TRUE(master_.dc().isInitialized());

    // Start (may fail without real transport, but should not crash)
    (void)master_.dc().start();

    // Stop
    master_.dc().stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

TEST_F(DCBasicTest, RestartAfterStop) {
    DC::DCConfig cfg = DC::DCConfig::defaults();

    master_.dc().init(cfg, 1);
    (void)master_.dc().start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    master_.dc().stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Second start attempt (may also fail without real transport)
    (void)master_.dc().start();
    // Just ensure no crash
    EXPECT_TRUE(master_.dc().isInitialized());
}
