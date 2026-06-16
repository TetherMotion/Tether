/**
 * @file test_dc_manager_coverage.cpp
 * @brief Coverage tests for DCManager.cpp
 *
 * Tests the null-guard paths (methods called before init), the static
 * convert helpers (exercised through getState/getStats), the destructor
 * path, and the init-then-use path.
 */
#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATDC.hpp"

using namespace EtherCAT;

// ============================================================================
// Null-guard paths (methods called without init)
// ============================================================================

TEST(DCManagerCoverage, StartWithoutInit) {
    Master master;
    DCManager dc(master);
    EXPECT_FALSE(dc.start());
}

TEST(DCManagerCoverage, StopWithoutInit) {
    Master master;
    DCManager dc(master);
    dc.stop(); // no crash
}

TEST(DCManagerCoverage, GetStateWithoutInit) {
    Master master;
    DCManager dc(master);
    EXPECT_EQ(dc.getState(), DC::DCState::Disabled);
}

TEST(DCManagerCoverage, GetStatsWithoutInit) {
    Master master;
    DCManager dc(master);
    auto stats = dc.getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
    EXPECT_EQ(stats.sync_count, 0u);
}

TEST(DCManagerCoverage, ForceSyncWithoutInit) {
    Master master;
    DCManager dc(master);
    dc.forceSync(); // no crash
}

TEST(DCManagerCoverage, SetPDOEnabledWithoutInit) {
    Master master;
    DCManager dc(master);
    dc.setPDOEnabled(true); // no crash
}

TEST(DCManagerCoverage, ReconfigureSyncWithoutInit) {
    Master master;
    DCManager dc(master);
    EXPECT_FALSE(dc.reconfigureSync(0));
}

TEST(DCManagerCoverage, GetReturnsSentinelWhenNotInit) {
    Master master;
    DCManager dc(master);
    /* DCManager::get() must never return nullptr — a sentinel instance is
       returned that logs a critical error on any use. */
    EXPECT_NE(dc.get(), nullptr);
    uint8_t buf[8] = {0};
    EXPECT_FALSE(dc.get()->readRegister(0, DCRegisters::DCSyncAct, buf, 1));
}

// ============================================================================
// After init — exercising convert helpers and real instance paths
// ============================================================================

TEST(DCManagerCoverage, InitAndGetState) {
    Master master;
    DCManager dc(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc.init(cfg, 1));

    // convertState should map something sensible
    auto state = dc.getState();
    // Just verify it returns a valid enum value (not out-of-range)
    EXPECT_GE(static_cast<int>(state), static_cast<int>(DC::DCState::Disabled));
    EXPECT_LE(static_cast<int>(state), static_cast<int>(DC::DCState::Error));
}

TEST(DCManagerCoverage, InitAndGetStats) {
    Master master;
    DCManager dc(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc.init(cfg, 1));

    auto stats = dc.getStats();
    // convertStats maps fields — verify structure is populated
    EXPECT_EQ(stats.cycle_count, 0u); // freshly created, no cycles yet
}

TEST(DCManagerCoverage, InitStartStop) {
    Master master;
    DCManager dc(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc.init(cfg, 1));
    dc.start(); // may or may not succeed, but should not crash
    dc.stop();
}

TEST(DCManagerCoverage, InitForceSync) {
    Master master;
    DCManager dc(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc.init(cfg, 1));
    dc.forceSync();
}

TEST(DCManagerCoverage, InitSetPDOEnabled) {
    Master master;
    DCManager dc(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc.init(cfg, 1));
    dc.setPDOEnabled(true);
    dc.setPDOEnabled(false);
}

TEST(DCManagerCoverage, InitReconfigureSync) {
    Master master;
    DCManager dc(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc.init(cfg, 1));
    // Will call dc_instance_->reconfigureSync(0) — may return true or false
    dc.reconfigureSync(0);
}

// ============================================================================
// Destructor with initialized instance (exercises stop+reset path)
// ============================================================================

TEST(DCManagerCoverage, DestructorAfterInit) {
    Master master;
    auto dc = std::make_unique<DCManager>(master);
    DC::DCConfig cfg = DC::DCConfig::defaults();
    ASSERT_TRUE(dc->init(cfg, 1));
    dc.reset(); // triggers destructor with active dc_instance_
}
