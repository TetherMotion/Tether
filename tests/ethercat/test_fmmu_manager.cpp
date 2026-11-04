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
#include "tether/ethercat/PDOManager.hpp"

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
                (uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(bool, aprd,
                (uint16_t ado, void* out, uint16_t len, unsigned int timeout_ms),
                (override));
};

// ============================================================================
// Test fixture
// ============================================================================

class FMMUManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mgr_ = std::make_unique<FMMUManager>(transport_);
    }

    MockFMMUTransport transport_;
    std::unique_ptr<FMMUManager> mgr_;
};

// ============================================================================
// Init / state tests
// ============================================================================

TEST_F(FMMUManagerTest, InitResetsCleansState) {
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 0u);

    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 0u);
    EXPECT_EQ(cfg.configured, false);
}

TEST_F(FMMUManagerTest, ConfigReturnsReference) {
    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 0u);
}

// ============================================================================
// configureManual tests
// ============================================================================

TEST_F(FMMUManagerTest, ConfigureManualOutputOnly) {
    bool ok = mgr_->configureManual(0x1800, 23, 0, 0, 0x0000);
    EXPECT_TRUE(ok);

    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 1u);
    EXPECT_EQ(cfg.fmmus[0].physical_start_addr, 0x1800);
    EXPECT_EQ(cfg.fmmus[0].length, 23u);
    EXPECT_EQ(cfg.fmmus[0].logical_start_addr, 0u);
    // Output FMMU → type should include Write flag
    EXPECT_TRUE(cfg.fmmus[0].type.write_enable);
}

TEST_F(FMMUManagerTest, ConfigureManualInputOnly) {
    bool ok = mgr_->configureManual(0, 0, 0x1C00, 25, 0x0000);
    EXPECT_TRUE(ok);

    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 1u);
    EXPECT_EQ(cfg.fmmus[0].physical_start_addr, 0x1C00);
    EXPECT_EQ(cfg.fmmus[0].length, 25u);
    EXPECT_TRUE(cfg.fmmus[0].type.read_enable);
}

TEST_F(FMMUManagerTest, ConfigureManualBoth) {
    bool ok = mgr_->configureManual(0x1800, 23, 0x1C00, 25, 0x0000);
    EXPECT_TRUE(ok);

    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 2u);

    // Output comes first
    EXPECT_EQ(cfg.fmmus[0].length, 23u);
    // Input second
    EXPECT_EQ(cfg.fmmus[1].length, 25u);

    // Logical addresses are contiguous
    EXPECT_EQ(cfg.fmmus[0].logical_start_addr, 0u);
    EXPECT_EQ(cfg.fmmus[1].logical_start_addr, 23u);

    // Total logical = 23 + 25
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 48u);
}

TEST_F(FMMUManagerTest, ConfigureManualZeroLengthIsNoop) {
    bool ok = mgr_->configureManual(0, 0, 0, 0, 0);
    EXPECT_TRUE(ok);
    EXPECT_EQ(mgr_->config().fmmu_count, 0u);
}

// ============================================================================
// Address queries
// ============================================================================

TEST_F(FMMUManagerTest, LogicalAddresses) {
    mgr_->configureManual(0x1800, 10, 0x1C00, 20, 0x100);

    EXPECT_EQ(mgr_->getOutputLogicalAddr(), 0x100u);
    EXPECT_EQ(mgr_->getInputLogicalAddr(), 0x100u + 10u);
}

// ============================================================================
// writeToSlave tests
// ============================================================================

TEST_F(FMMUManagerTest, WriteToSlaveNoFMMUsIsNoop) {
    // No FMMUs configured → should succeed trivially, no transport calls.
    EXPECT_CALL(transport_, apwr(_, _, _, _)).Times(0);
    EXPECT_TRUE(mgr_->writeToSlave());
}

TEST_F(FMMUManagerTest, WriteToSlaveCallsApwr) {
    mgr_->configureManual(0x1800, 23, 0x1C00, 25, 0);

    // Expect 2 FMMU write calls + up to kMaxFMMUs-2 clear calls (accept any).
    EXPECT_CALL(transport_, apwr(_, _, _, _))
        .WillRepeatedly(Return(true));

    EXPECT_TRUE(mgr_->writeToSlave());
    EXPECT_TRUE(mgr_->config().configured);
}

TEST_F(FMMUManagerTest, WriteToSlaveFailureReturnsFalse) {
    mgr_->configureManual(0x1800, 23, 0, 0, 0);

    // Fail the FMMU register write
    EXPECT_CALL(transport_, apwr(_, _, _, _))
        .WillRepeatedly(Return(false));

    EXPECT_FALSE(mgr_->writeToSlave());
}

// ============================================================================
// readFromSlave tests
// ============================================================================

TEST_F(FMMUManagerTest, ReadFromSlaveReturnsZeroOnNullOrZero) {
    EXPECT_EQ(mgr_->readFromSlave(nullptr, 4), 0u);

    FMMUConfig buf[4];
    EXPECT_EQ(mgr_->readFromSlave(buf, 0), 0u);
}

TEST_F(FMMUManagerTest, ReadFromSlavePopulatesConfigs) {
    FMMURegBlock mock_regs{};
    mock_regs.logical_start_le = 0x100;
    mock_regs.length_le = 23;
    mock_regs.physical_start_le = 0x1800;
    mock_regs.type = FMMURegType::Write;
    mock_regs.activate = FMMUActivate::Enable;

    // Use a lambda via WillOnce to copy mock register block into output buffer.
    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillOnce([&mock_regs](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &mock_regs, std::min(static_cast<size_t>(len), sizeof(mock_regs)));
            return true;
        })
        .WillRepeatedly(Return(false));  // Stop after first

    FMMUConfig buf[4];
    size_t count = mgr_->readFromSlave(buf, 4);
    EXPECT_EQ(count, 1u);
    EXPECT_EQ(buf[0].logical_start_addr, 0x100u);
    EXPECT_EQ(buf[0].length, 23u);
    EXPECT_EQ(buf[0].physical_start_addr, 0x1800);
}

// ============================================================================
// disableAll tests
// ============================================================================

TEST_F(FMMUManagerTest, DisableAllClearsConfiguredFlag) {
    mgr_->configureManual(0x1800, 23, 0, 0, 0);

    EXPECT_CALL(transport_, apwr(_, _, _, _)).WillRepeatedly(Return(true));

    mgr_->writeToSlave();
    EXPECT_TRUE(mgr_->config().configured);

    // Now disable
    mgr_->disableAll();
    EXPECT_FALSE(mgr_->config().configured);
}

// ============================================================================
// Multiple independent instances
// ============================================================================

TEST_F(FMMUManagerTest, TwoManagersHaveIndependentState) {
    MockFMMUTransport t2;
    FMMUManager mgr2(t2);

    mgr_->configureManual(0x1800, 10, 0, 0, 0);
    mgr2.configureManual(0x2000, 50, 0, 0, 0x1000);

    EXPECT_EQ(mgr_->config().fmmus[0].physical_start_addr, 0x1800);
    EXPECT_EQ(mgr2.config().fmmus[0].physical_start_addr, 0x2000);

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

    bool ok = mgr_->configureFromSii(nullptr, &sm_config, 0);
    EXPECT_TRUE(ok);

    auto& cfg = mgr_->config();
    // Should have output + input FMMUs
    EXPECT_GE(cfg.fmmu_count, 2u);
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 20u);
}

// ============================================================================
// readFromSlave: input (read-enable) FMMU type attribution
// ============================================================================

TEST_F(FMMUManagerTest, ReadFromSlaveDetectsInputType) {
    FMMURegBlock mock_regs{};
    mock_regs.logical_start_le = 0x200;
    mock_regs.length_le = 12;
    mock_regs.physical_start_le = 0x1C00;
    mock_regs.type = FMMURegType::Read;
    mock_regs.activate = FMMUActivate::Enable;

    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillOnce([&mock_regs](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &mock_regs, std::min(static_cast<size_t>(len), sizeof(mock_regs)));
            return true;
        })
        .WillRepeatedly(Return(false));

    FMMUConfig buf[4];
    ASSERT_EQ(mgr_->readFromSlave(buf, 4), 1u);
    EXPECT_EQ(buf[0].sii_type, FMMUType::Input);
    EXPECT_TRUE(buf[0].isEnabled());
    EXPECT_TRUE(buf[0].isInput());
}

TEST_F(FMMUManagerTest, ReadFromSlaveDisabledFmmuKeepsUnusedType) {
    FMMURegBlock mock_regs{};
    mock_regs.logical_start_le = 0x300;
    mock_regs.length_le = 4;
    mock_regs.type = FMMURegType::Write;
    mock_regs.activate = FMMUActivate::Disable;

    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillOnce([&mock_regs](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &mock_regs, std::min(static_cast<size_t>(len), sizeof(mock_regs)));
            return true;
        })
        .WillRepeatedly(Return(false));

    FMMUConfig buf[4];
    ASSERT_EQ(mgr_->readFromSlave(buf, 4), 1u);
    EXPECT_EQ(buf[0].sii_type, FMMUType::Unused);
    EXPECT_FALSE(buf[0].isEnabled());
}

// ============================================================================
// verify(): hardware readback comparison
// ============================================================================

namespace {

/** Build a register block matching a configured FMMU. */
FMMURegBlock blockFor(const FMMUConfig& cfg) {
    FMMURegBlock regs{};
    regs.logical_start_le = cfg.logical_start_addr;
    regs.length_le = cfg.length;
    regs.logical_start_bit = cfg.logical_start_bit;
    regs.logical_end_bit = cfg.logical_end_bit;
    regs.physical_start_le = cfg.physical_start_addr;
    regs.physical_start_bit = cfg.physical_start_bit;
    regs.type = std::bit_cast<uint8_t>(cfg.type);
    regs.activate = std::bit_cast<uint8_t>(cfg.activate);
    return regs;
}

} // namespace

TEST_F(FMMUManagerTest, VerifyWithNoFmmusIsTrue) {
    EXPECT_CALL(transport_, aprd(_, _, _, _)).Times(0);
    EXPECT_TRUE(mgr_->verify());
}

TEST_F(FMMUManagerTest, VerifyMatchesHardware) {
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    auto& cfg = mgr_->config();
    ASSERT_EQ(cfg.fmmu_count, 2u);

    const FMMURegBlock blocks[2] = {blockFor(cfg.fmmus[0]), blockFor(cfg.fmmus[1])};

    size_t call = 0;
    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillRepeatedly([&](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            if (call >= 2) return false;
            std::memcpy(out, &blocks[call], std::min(static_cast<size_t>(len), sizeof(FMMURegBlock)));
            ++call;
            return true;
        });

    EXPECT_TRUE(mgr_->verify());
}

TEST_F(FMMUManagerTest, VerifyMismatchReturnsFalse) {
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    auto& cfg = mgr_->config();

    FMMURegBlock blocks[2] = {blockFor(cfg.fmmus[0]), blockFor(cfg.fmmus[1])};
    blocks[1].length_le = 99;  // hardware disagrees

    size_t call = 0;
    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillRepeatedly([&](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            if (call >= 2) return false;
            std::memcpy(out, &blocks[call], std::min(static_cast<size_t>(len), sizeof(FMMURegBlock)));
            ++call;
            return true;
        });

    EXPECT_FALSE(mgr_->verify());
}

TEST_F(FMMUManagerTest, VerifyShortReadReturnsFalse) {
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    auto& cfg = mgr_->config();
    const FMMURegBlock block = blockFor(cfg.fmmus[0]);

    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillOnce([&block](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &block, std::min(static_cast<size_t>(len), sizeof(block)));
            return true;
        })
        .WillRepeatedly(Return(false));

    EXPECT_FALSE(mgr_->verify());
}

// ============================================================================
// verifyFromSlave(): slave-truth readback (no cached config needed)
// ============================================================================

TEST_F(FMMUManagerTest, VerifyFromSlaveReadFailsReturnsFalse) {
    EXPECT_CALL(transport_, aprd(_, _, _, _)).WillRepeatedly(Return(false));
    EXPECT_FALSE(mgr_->verifyFromSlave());
}

TEST_F(FMMUManagerTest, VerifyFromSlaveNoEnabledFmmusIsTrue) {
    FMMURegBlock zero{};
    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillRepeatedly([&zero](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &zero, std::min(static_cast<size_t>(len), sizeof(zero)));
            return true;
        });

    EXPECT_TRUE(mgr_->verifyFromSlave());
}

TEST_F(FMMUManagerTest, VerifyFromSlaveMatchesHardware) {
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    auto& cfg = mgr_->config();
    const FMMURegBlock blocks[2] = {blockFor(cfg.fmmus[0]), blockFor(cfg.fmmus[1])};

    size_t call = 0;
    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillRepeatedly([&](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            if (call >= 2) return false;
            std::memcpy(out, &blocks[call], std::min(static_cast<size_t>(len), sizeof(FMMURegBlock)));
            ++call;
            return true;
        });

    EXPECT_TRUE(mgr_->verifyFromSlave());
}

TEST_F(FMMUManagerTest, VerifyFromSlaveMismatchReturnsFalse) {
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    auto& cfg = mgr_->config();
    FMMURegBlock blocks[2] = {blockFor(cfg.fmmus[0]), blockFor(cfg.fmmus[1])};
    blocks[0].physical_start_le = 0x9999;

    size_t call = 0;
    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillRepeatedly([&](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            if (call >= 2) return false;
            std::memcpy(out, &blocks[call], std::min(static_cast<size_t>(len), sizeof(FMMURegBlock)));
            ++call;
            return true;
        });

    EXPECT_FALSE(mgr_->verifyFromSlave());
}

TEST_F(FMMUManagerTest, VerifyFromSlaveShortReadReturnsFalse) {
    // Slave has an enabled FMMU but only one is readable while two are expected.
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    auto& cfg = mgr_->config();
    const FMMURegBlock block = blockFor(cfg.fmmus[0]);

    EXPECT_CALL(transport_, aprd(_, _, _, _))
        .WillOnce([&block](uint16_t, void* out, uint16_t len, unsigned int) -> bool {
            std::memcpy(out, &block, std::min(static_cast<size_t>(len), sizeof(block)));
            return true;
        })
        .WillRepeatedly(Return(false));

    EXPECT_FALSE(mgr_->verifyFromSlave());
}

// ============================================================================
// writeToSlave(): debug logging + hex dump arm
// ============================================================================

TEST_F(FMMUManagerTest, WriteToSlaveDebugArmSucceeds) {
    ASSERT_TRUE(mgr_->configureManual(0x1800, 8, 0x1C00, 4, 0x10000));
    EXPECT_CALL(transport_, apwr(_, _, _, _)).WillRepeatedly(Return(true));

    EXPECT_TRUE(mgr_->writeToSlave(/*fmmu_debug=*/true));
    EXPECT_TRUE(mgr_->config().configured);
}

} // anonymous namespace
