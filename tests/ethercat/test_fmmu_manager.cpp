/**
 * @file test_fmmu_manager.cpp
 * @brief Unit tests for FMMUManager (instance-based, no globals).
 *
 * Uses a GMock transport so no real I/O is required.
 * Linked into tether_ethercat_common_tests.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/fmmu/FMMUConfiguration.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"

#include <cstring>
#include <vector>

using namespace EtherCAT::fmmu;
using ::testing::_;
using ::testing::Return;

namespace {

// ============================================================================
// Mock transport
// ============================================================================

class MockFMMUTransport : public IFMMUTransport {
public:
    MOCK_METHOD(bool, apwr,
                (uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(bool, aprd,
                (uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(uint16_t, adpForSlaveIndex, (uint16_t slave_index), (override));
};

// ============================================================================
// Test fixture
// ============================================================================

class FMMUManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mgr_ = std::make_unique<FMMUManager>(transport_);
        mgr_->init();

        ON_CALL(transport_, adpForSlaveIndex(_))
            .WillByDefault([](uint16_t idx) -> uint16_t {
                return static_cast<uint16_t>(0u - idx);
            });
    }

    MockFMMUTransport transport_;
    std::unique_ptr<FMMUManager> mgr_;
};

// ============================================================================
// Init / state tests
// ============================================================================

TEST_F(FMMUManagerTest, InitResetsCleansState) {
    EXPECT_TRUE(mgr_->isInitialized());
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 0u);

    // Every slave should have cleared config with its own index.
    for (uint16_t i = 0; i < kMaxFMMUSlaves; i++) {
        auto* cfg = mgr_->getConfig(i);
        ASSERT_NE(cfg, nullptr);
        EXPECT_EQ(cfg->slave_index, i);
        EXPECT_EQ(cfg->fmmu_count, 0u);
    }
}

TEST_F(FMMUManagerTest, GetConfigOutOfRangeReturnsNull) {
    EXPECT_EQ(mgr_->getConfig(kMaxFMMUSlaves), nullptr);
    EXPECT_EQ(mgr_->getConfig(kMaxFMMUSlaves + 1), nullptr);
}

TEST_F(FMMUManagerTest, GetSlaveConfigsReturnsBasePointer) {
    auto* base = mgr_->getSlaveConfigs();
    ASSERT_NE(base, nullptr);
    EXPECT_EQ(base, mgr_->getConfig(0));
}

// ============================================================================
// configureManual tests
// ============================================================================

TEST_F(FMMUManagerTest, ConfigureManualOutputOnly) {
    bool ok = mgr_->configureManual(0, 0x1800, 23, 0, 0, 0x0000);
    EXPECT_TRUE(ok);

    auto* cfg = mgr_->getConfig(0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->fmmu_count, 1u);
    EXPECT_EQ(cfg->fmmus[0].physical_start_addr, 0x1800);
    EXPECT_EQ(cfg->fmmus[0].length, 23u);
    EXPECT_EQ(cfg->fmmus[0].logical_start_addr, 0u);
    // Output FMMU → type should include Write flag
    EXPECT_NE(cfg->fmmus[0].type & FMMURegType::Write, 0);
}

TEST_F(FMMUManagerTest, ConfigureManualInputOnly) {
    bool ok = mgr_->configureManual(0, 0, 0, 0x1C00, 25, 0x0000);
    EXPECT_TRUE(ok);

    auto* cfg = mgr_->getConfig(0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->fmmu_count, 1u);
    EXPECT_EQ(cfg->fmmus[0].physical_start_addr, 0x1C00);
    EXPECT_EQ(cfg->fmmus[0].length, 25u);
    EXPECT_NE(cfg->fmmus[0].type & FMMURegType::Read, 0);
}

TEST_F(FMMUManagerTest, ConfigureManualBoth) {
    bool ok = mgr_->configureManual(0, 0x1800, 23, 0x1C00, 25, 0x0000);
    EXPECT_TRUE(ok);

    auto* cfg = mgr_->getConfig(0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->fmmu_count, 2u);

    // Output comes first
    EXPECT_EQ(cfg->fmmus[0].length, 23u);
    // Input second
    EXPECT_EQ(cfg->fmmus[1].length, 25u);

    // Logical addresses are contiguous
    EXPECT_EQ(cfg->fmmus[0].logical_start_addr, 0u);
    EXPECT_EQ(cfg->fmmus[1].logical_start_addr, 23u);

    // Total logical = 23 + 25
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 48u);
}

TEST_F(FMMUManagerTest, ConfigureManualOutOfRangeFails) {
    bool ok = mgr_->configureManual(kMaxFMMUSlaves, 0x1800, 23, 0, 0, 0);
    EXPECT_FALSE(ok);
}

// ============================================================================
// Address queries
// ============================================================================

TEST_F(FMMUManagerTest, LogicalAddresses) {
    mgr_->configureManual(0, 0x1800, 10, 0x1C00, 20, 0x100);

    EXPECT_EQ(mgr_->getOutputLogicalAddr(0), 0x100u);
    EXPECT_EQ(mgr_->getInputLogicalAddr(0), 0x100u + 10u);
}

TEST_F(FMMUManagerTest, LogicalAddressOutOfRange) {
    EXPECT_EQ(mgr_->getOutputLogicalAddr(kMaxFMMUSlaves), 0u);
    EXPECT_EQ(mgr_->getInputLogicalAddr(kMaxFMMUSlaves), 0u);
}

// ============================================================================
// writeToSlave tests
// ============================================================================

TEST_F(FMMUManagerTest, WriteToSlaveNoFMMUsIsNoop) {
    // No FMMUs configured → should succeed trivially, no transport calls.
    EXPECT_CALL(transport_, apwr(_, _, _, _, _)).Times(0);
    EXPECT_TRUE(mgr_->writeToSlave(0));
}

TEST_F(FMMUManagerTest, WriteToSlaveCallsApwr) {
    mgr_->configureManual(0, 0x1800, 23, 0x1C00, 25, 0);

    // Expect 2 FMMU write calls + up to kMaxFMMUs-2 clear calls (we don't
    // care about the clear calls here, accept any).
    EXPECT_CALL(transport_, adpForSlaveIndex(0)).WillRepeatedly(Return(0xFFFF));

    EXPECT_CALL(transport_, apwr(0xFFFF, _, _, _, _))
        .WillRepeatedly(Return(true));

    EXPECT_TRUE(mgr_->writeToSlave(0));
    auto* cfg = mgr_->getConfig(0);
    EXPECT_TRUE(cfg->configured);
}

TEST_F(FMMUManagerTest, WriteToSlaveFailureReturnsFalse) {
    mgr_->configureManual(0, 0x1800, 23, 0, 0, 0);

    EXPECT_CALL(transport_, adpForSlaveIndex(0)).WillRepeatedly(Return(0xFFFF));
    // Fail the FMMU register write
    EXPECT_CALL(transport_, apwr(0xFFFF, _, _, _, _))
        .WillRepeatedly(Return(false));

    EXPECT_FALSE(mgr_->writeToSlave(0));
}

// ============================================================================
// readFromSlave tests
// ============================================================================

TEST_F(FMMUManagerTest, ReadFromSlaveReturnsZeroOnNullOrZero) {
    EXPECT_EQ(mgr_->readFromSlave(0, nullptr, 4), 0u);

    FMMUConfig buf[4];
    EXPECT_EQ(mgr_->readFromSlave(0, buf, 0), 0u);
}

TEST_F(FMMUManagerTest, ReadFromSlavePopulatesConfigs) {
    FMMURegBlock mock_regs{};
    mock_regs.logical_start_le = 0x100;
    mock_regs.length_le = 23;
    mock_regs.physical_start_le = 0x1800;
    mock_regs.type = FMMURegType::Write;
    mock_regs.activate = FMMUActivate::Enable;

    EXPECT_CALL(transport_, adpForSlaveIndex(0)).WillRepeatedly(Return(0xFFFF));

    // Use a lambda via WillOnce to copy mock register block into output buffer.
    EXPECT_CALL(transport_, aprd(0xFFFF, _, _, _, _))
        .WillOnce([&mock_regs](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &mock_regs, std::min(static_cast<size_t>(len), sizeof(mock_regs)));
            return true;
        })
        .WillRepeatedly(Return(false));  // Stop after first

    FMMUConfig buf[4];
    size_t count = mgr_->readFromSlave(0, buf, 4);
    EXPECT_EQ(count, 1u);
    EXPECT_EQ(buf[0].logical_start_addr, 0x100u);
    EXPECT_EQ(buf[0].length, 23u);
    EXPECT_EQ(buf[0].physical_start_addr, 0x1800);
}

// ============================================================================
// disableAll tests
// ============================================================================

TEST_F(FMMUManagerTest, DisableAllClearsConfiguredFlag) {
    mgr_->configureManual(0, 0x1800, 23, 0, 0, 0);

    EXPECT_CALL(transport_, adpForSlaveIndex(0)).WillRepeatedly(Return(0xFFFF));
    EXPECT_CALL(transport_, apwr(_, _, _, _, _)).WillRepeatedly(Return(true));

    mgr_->writeToSlave(0);
    EXPECT_TRUE(mgr_->getConfig(0)->configured);

    // Now disable
    mgr_->disableAll(0);
    EXPECT_FALSE(mgr_->getConfig(0)->configured);
}

// ============================================================================
// Multiple independent instances
// ============================================================================

TEST_F(FMMUManagerTest, TwoManagersHaveIndependentState) {
    MockFMMUTransport t2;
    FMMUManager mgr2(t2);
    mgr2.init();

    mgr_->configureManual(0, 0x1800, 10, 0, 0, 0);
    mgr2.configureManual(0, 0x2000, 50, 0, 0, 0x1000);

    EXPECT_EQ(mgr_->getConfig(0)->fmmus[0].physical_start_addr, 0x1800);
    EXPECT_EQ(mgr2.getConfig(0)->fmmus[0].physical_start_addr, 0x2000);

    EXPECT_EQ(mgr_->getTotalLogicalSize(), 10u);
    EXPECT_EQ(mgr2.getTotalLogicalSize(), 0x1000u + 50u);
}

// ============================================================================
// configureFromSii (basic path — no SII data, SM config only)
// ============================================================================

TEST_F(FMMUManagerTest, ConfigureFromSiiWithSmConfig) {
    EtherCAT::PDO::SlaveConfig sm_config{};
    sm_config.sm[2].phys_start_addr = 0x1800;
    sm_config.sm[2].length = 12;
    sm_config.sm[3].phys_start_addr = 0x1C00;
    sm_config.sm[3].length = 8;

    bool ok = mgr_->configureFromSii(0, nullptr, &sm_config, 0);
    EXPECT_TRUE(ok);

    auto* cfg = mgr_->getConfig(0);
    ASSERT_NE(cfg, nullptr);
    // Should have output + input FMMUs
    EXPECT_GE(cfg->fmmu_count, 2u);
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 20u);
}

TEST_F(FMMUManagerTest, ConfigureFromSiiOutOfRangeFails) {
    EXPECT_FALSE(mgr_->configureFromSii(kMaxFMMUSlaves, nullptr, nullptr, 0));
}

} // anonymous namespace
