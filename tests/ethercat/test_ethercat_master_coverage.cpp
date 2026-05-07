/**
 * @file test_ethercat_master_coverage.cpp
 * @brief Coverage tests for EtherCATMaster.cpp
 *
 * Targets: getECStateName, parseEtherCATFrame, watchdog methods,
 * ensureRxQueues, flushRxQueue, getStats, and sub-manager accessors.
 */
#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATMaster.hpp"
#include <cstring>

using namespace EtherCAT;

// ============================================================================
// getECStateName — pure static function
// ============================================================================

TEST(EtherCATMasterCoverage, GetECStateName) {
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x01), "INIT");
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x02), "PRE_OP");
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x03), "BOOT");
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x04), "SAFE_OP");
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x08), "OP");
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x00), "UNKNOWN");
    // With error bit (0x10) — only lower nibble matters
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x11), "INIT");
    EXPECT_STREQ(EtherCATMaster::getECStateName(0x18), "OP");
}

// ============================================================================
// Queue management — rxQueue and txpdoRxQueue are public accessors
// ============================================================================

TEST(EtherCATMasterCoverage, RxQueuesAvailable) {
    EtherCATMaster master;
    // Queues are created in the constructor
    EXPECT_NE(master.rxQueue(), nullptr);
    EXPECT_NE(master.txpdoRxQueue(), nullptr);
}

// ============================================================================
// getStats
// ============================================================================

#if TETHER_ENABLE_ETHERCAT_STATS
TEST(EtherCATMasterCoverage, GetStats) {
    EtherCATMaster master;
    auto stats = master.getStats();
    EXPECT_EQ(stats.tx_retry_count, 0u);
    EXPECT_EQ(stats.tx_fail_count, 0u);
    EXPECT_EQ(stats.rx_frame_count, 0u);
}
#endif

// ============================================================================
// Sub-manager accessors
// ============================================================================

TEST(EtherCATMasterCoverage, SubManagerAccessors) {
    EtherCATMaster master;
    // These should all return valid references
    auto& pdo = master.pdo();
    (void)pdo;
    auto& sdo = master.sdoManager();
    (void)sdo;
    auto& dc = master.dc();
    (void)dc;
    auto& foe = master.foe();
    (void)foe;
    auto& voe = master.voe();
    (void)voe;
    auto& eoe = master.eoe();
    (void)eoe;
    auto& faults = master.faults();
    (void)faults;
    auto& router = master.packetRouter();
    (void)router;
}

// ============================================================================
// Watchdog methods (via APWR/APRD test callbacks)
// ============================================================================

TEST(EtherCATMasterCoverage, ConfigureWatchdogsSuccess) {
    EtherCATMaster master;
    // Set APWR callback to succeed
    master.setApwrTestCallback([](uint16_t adp, uint16_t ado,
                                   const void* data, uint16_t len,
                                   unsigned int timeout_ms) -> bool {
        return true;
    });
    EXPECT_TRUE(master.configureWatchdogs(0, 1000, 2000));
}

TEST(EtherCATMasterCoverage, ConfigureWatchdogsFail) {
    EtherCATMaster master;
    master.setApwrTestCallback([](uint16_t adp, uint16_t ado,
                                   const void* data, uint16_t len,
                                   unsigned int timeout_ms) -> bool {
        return false;
    });
    EXPECT_FALSE(master.configureWatchdogs(0, 1000, 2000));
}

TEST(EtherCATMasterCoverage, DisableWatchdogs) {
    EtherCATMaster master;
    master.setApwrTestCallback([](uint16_t adp, uint16_t ado,
                                   const void* data, uint16_t len,
                                   unsigned int timeout_ms) -> bool {
        return true;
    });
    EXPECT_TRUE(master.disableWatchdogs(0));
}

TEST(EtherCATMasterCoverage, ReadWatchdogStatus) {
    EtherCATMaster master;
    uint8_t wd_status = 0xFF, pdi_cnt = 0xFF, pdata_cnt = 0xFF;

    // Provide APRD test callback that fills data
    master.setAprdTestCallback([](uint16_t adp, uint16_t ado,
                                   void* data, uint16_t len,
                                   unsigned int timeout_ms) -> bool {
        if (len >= 1) {
            std::memset(data, 0x42, len);
        }
        return true;
    });

    EXPECT_TRUE(master.readWatchdogStatus(0, wd_status, pdi_cnt, pdata_cnt));
    EXPECT_EQ(wd_status, 0x42u);
}

TEST(EtherCATMasterCoverage, ReadWatchdogStatusFails) {
    EtherCATMaster master;
    uint8_t wd_status = 0, pdi_cnt = 0, pdata_cnt = 0;
    // No callback set, ecAprd fails
    EXPECT_FALSE(master.readWatchdogStatus(0, wd_status, pdi_cnt, pdata_cnt));
}

// ============================================================================
// wasFaultDiagnosed
// ============================================================================

TEST(EtherCATMasterCoverage, WasFaultDiagnosed) {
    EtherCATMaster master;
    EXPECT_FALSE(master.wasFaultDiagnosed(0));
}

// ============================================================================
// setMailboxOverride
// ============================================================================

TEST(EtherCATMasterCoverage, SetMailboxOverride) {
    EtherCATMaster master;
    // Just verify it doesn't crash; override with some values
    master.setMailboxOverride(0, 0x1000, 128, 0x1400, 128, 0x0004);
}

// ============================================================================
// allocIdx / resetIdx — already partially tested, but cover edge cases
// ============================================================================

TEST(EtherCATMasterCoverage, AllocAndResetIdx) {
    EtherCATMaster master;
    uint8_t idx1 = master.allocIdx();
    uint8_t idx2 = master.allocIdx();
    EXPECT_NE(idx1, idx2);
    master.resetIdx();
    uint8_t idx3 = master.allocIdx();
    // After reset, should start from beginning again
    EXPECT_EQ(idx3, idx1);
}
