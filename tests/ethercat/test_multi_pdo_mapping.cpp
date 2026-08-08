/**
 * @file test_multi_pdo_mapping.cpp
 * @brief Unit tests for multi-PDO mapping support per sync manager.
 *
 * Tests cover:
 * - PDOMappingRegion and MultiPDOSyncManagerConfig types
 * - FMMUManager::configureFromMultiPDO with per-PDO logical address tracking
 * - SlaveFMMUConfig::addOutputWithPDOs / addInputWithPDOs
 * - FMMUManager per-PDO address queries (getPDOLogicalAddr, getPDOSize, findPDOEntry)
 * - LogicalAddressManager::buildAddressMapFromMultiPDO
 * - LogicalAddressManager per-PDO address queries
 * - SyncManagerAccessor write/clear/read-all PDO assignments (compile + types)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cstring>
#include <vector>
#include <initializer_list>

#include "tether/ethercat/PDOMappingConfig.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using namespace EtherCAT::fmmu;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

// ============================================================================
// Mock transports
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

class MockPDOTransport : public IPDOTransport {
public:
    MOCK_METHOD(bool, writeRegister,
                (uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(bool, readRegister,
                (uint16_t adp, uint16_t ado, void* data, uint16_t len, unsigned int timeout_ms),
                (override));
    MOCK_METHOD(bool, sendSingleDatagram,
                (Command cmd, uint8_t idx, uint16_t adp, uint16_t ado,
                 const void* data, uint16_t datalen, bool roundtrip),
                (override));
    MOCK_METHOD(size_t, sendMultiDatagram,
                (const MultiDatagramSpec* specs, size_t count),
                (override));
    MOCK_METHOD(bool, waitForResponseIdx,
                (uint8_t idx, unsigned int timeout_ms, RxDatagram& out),
                (override));
    MOCK_METHOD(size_t, preRegisterResponseWaiter,
                (uint8_t idx, uint8_t* buffer, size_t buffer_size),
                (override));
    MOCK_METHOD(bool, waitForPreRegistered,
                (size_t slot, unsigned int timeout_ms, RxDatagram& out),
                (override));
    MOCK_METHOD(uint8_t, allocIdx, (), (override));
    MOCK_METHOD(uint16_t, adpForSlaveIndex, (uint16_t slave_index), (override));
};

// ============================================================================
// Part 1: PDOMappingRegion and MultiPDOSyncManagerConfig
// ============================================================================

TEST(PDOMappingRegionTest, DefaultConstruction) {
    PDOMappingRegion r;
    EXPECT_EQ(r.pdo_index, 0u);
    EXPECT_EQ(r.size_bytes, 0u);
}

TEST(PDOMappingRegionTest, ParameterizedConstruction) {
    PDOMappingRegion r(0x1600, 8);
    EXPECT_EQ(r.pdo_index, 0x1600u);
    EXPECT_EQ(r.size_bytes, 8u);
}

TEST(MultiPDOSyncManagerConfigTest, DefaultIsSM2ProcessOutput) {
    MultiPDOSyncManagerConfig cfg;
    EXPECT_EQ(cfg.sm_index, 2u);
    EXPECT_EQ(cfg.type, SyncManagerType::ProcessOutput);
    EXPECT_TRUE(cfg.enable);
}

TEST(MultiPDOSyncManagerConfigTest, ProcessOutputFactory) {
    auto cfg = MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    });
    EXPECT_EQ(cfg.sm_index, 2u);
    EXPECT_EQ(cfg.phys_start_addr, 0x1800u);
    EXPECT_EQ(cfg.type, SyncManagerType::ProcessOutput);
    EXPECT_EQ(cfg.pdo_mappings.size(), 2u);
    EXPECT_EQ(cfg.totalLength(), 12u);
}

TEST(MultiPDOSyncManagerConfigTest, ProcessInputFactory) {
    auto cfg = MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
    });
    EXPECT_EQ(cfg.sm_index, 3u);
    EXPECT_EQ(cfg.phys_start_addr, 0x1C00u);
    EXPECT_EQ(cfg.type, SyncManagerType::ProcessInput);
    EXPECT_EQ(cfg.pdo_mappings.size(), 1u);
    EXPECT_EQ(cfg.totalLength(), 16u);
}

TEST(MultiPDOSyncManagerConfigTest, AddPDOMapping) {
    MultiPDOSyncManagerConfig cfg;
    cfg.addPDOMapping(0x1600, 8);
    cfg.addPDOMapping(0x1601, 4);
    cfg.addPDOMapping(0x1602, 2);
    EXPECT_EQ(cfg.pdo_mappings.size(), 3u);
    EXPECT_EQ(cfg.totalLength(), 14u);
}

TEST(MultiPDOSyncManagerConfigTest, TotalLengthEmpty) {
    MultiPDOSyncManagerConfig cfg;
    EXPECT_EQ(cfg.totalLength(), 0u);
}

TEST(MultiPDOSyncManagerConfigTest, ToLegacyConfig) {
    auto cfg = MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    });
    auto leg = cfg.toLegacyConfig();
    EXPECT_EQ(leg.phys_start_addr, 0x1800u);
    EXPECT_EQ(leg.length, 12u);
    EXPECT_EQ(leg.type, SyncManagerType::ProcessOutput);
    EXPECT_TRUE(leg.enable);
}

TEST(MultiPDOSyncManagerConfigTest, CustomSMIndex) {
    auto cfg = MultiPDOSyncManagerConfig::process_output(0x2000, {
        {0x1600, 8},
    });
    cfg.sm_index = 4;  // Use SM4 explicitly
    EXPECT_EQ(cfg.sm_index, 4u);
}

// ============================================================================
// Part 2: FMMUManager multi-PDO configuration
// ============================================================================

class FMMUMultiPDOTest : public ::testing::Test {
protected:
    void SetUp() override {
        mgr_ = std::make_unique<FMMUManager>(transport_);
    }

    NiceMock<MockFMMUTransport> transport_;
    std::unique_ptr<FMMUManager> mgr_;
};

TEST_F(FMMUMultiPDOTest, ConfigureFromMultiPODSingleSM) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    configs.push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));

    bool ok = mgr_->configureFromMultiPDO(configs, 0x10000);
    EXPECT_TRUE(ok);

    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 1u);
    EXPECT_EQ(cfg.pdo_entry_count, 2u);
    EXPECT_EQ(cfg.fmmus[0].length, 12u);
    EXPECT_EQ(cfg.fmmus[0].physical_start_addr, 0x1800u);
    EXPECT_EQ(cfg.fmmus[0].logical_start_addr, 0x10000u);
    EXPECT_EQ(cfg.fmmus[0].associated_sm, 2u);
}

TEST_F(FMMUMultiPDOTest, ConfigureFromMultiPDOOutputAndInput) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    configs.push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));
    configs.push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
        {0x1A01, 8},
    }));

    bool ok = mgr_->configureFromMultiPDO(configs, 0x10000);
    EXPECT_TRUE(ok);

    auto& cfg = mgr_->config();
    EXPECT_EQ(cfg.fmmu_count, 2u);
    EXPECT_EQ(cfg.pdo_entry_count, 4u);

    // Output FMMU: 12 bytes at 0x10000
    EXPECT_EQ(cfg.fmmus[0].length, 12u);
    EXPECT_EQ(cfg.fmmus[0].logical_start_addr, 0x10000u);

    // Input FMMU: 24 bytes at 0x1000C
    EXPECT_EQ(cfg.fmmus[1].length, 24u);
    EXPECT_EQ(cfg.fmmus[1].logical_start_addr, 0x1000Cu);

    // getTotalLogicalSize returns next_logical_addr which includes the base
    EXPECT_EQ(mgr_->getTotalLogicalSize(), 0x10024u);
}

TEST_F(FMMUMultiPDOTest, PerPDOLogicalAddresses) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    configs.push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));

    mgr_->configureFromMultiPDO(configs, 0x10000);

    // PDO 0x1600: logical_addr = 0x10000, size = 8
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0x1600), 0x10000u);
    EXPECT_EQ(mgr_->getPDOSize(0x1600), 8u);

    // PDO 0x1601: logical_addr = 0x10008, size = 4
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0x1601), 0x10008u);
    EXPECT_EQ(mgr_->getPDOSize(0x1601), 4u);

    // Non-existent PDO
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0x9999), 0u);
    EXPECT_EQ(mgr_->getPDOSize(0x9999), 0u);
}

TEST_F(FMMUMultiPDOTest, FindPDOEntry) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    configs.push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
        {0x1A01, 8},
    }));

    mgr_->configureFromMultiPDO(configs, 0);

    const FMMUPDOEntry* entry = mgr_->findPDOEntry(0x1A01);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->pdo_index, 0x1A01u);
    EXPECT_EQ(entry->size_bytes, 8u);
    EXPECT_EQ(entry->physical_offset, 16u);
    EXPECT_EQ(entry->sm_index, 3u);

    EXPECT_EQ(mgr_->findPDOEntry(0xFFFF), nullptr);
}

TEST_F(FMMUMultiPDOTest, ConfigureFromMultiPDOEmptyIsNoop) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    bool ok = mgr_->configureFromMultiPDO(configs, 0);
    EXPECT_FALSE(ok);
    EXPECT_EQ(mgr_->config().fmmu_count, 0u);
}

TEST_F(FMMUMultiPDOTest, ConfigureFromMultiPDOSkipsEmptyMappings) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    MultiPDOSyncManagerConfig empty_sm;
    empty_sm.sm_index = 2;
    empty_sm.type = SyncManagerType::ProcessOutput;
    // No PDO mappings
    configs.push_back(empty_sm);

    configs.push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
    }));

    bool ok = mgr_->configureFromMultiPDO(configs, 0);
    EXPECT_TRUE(ok);
    EXPECT_EQ(mgr_->config().fmmu_count, 1u);
}

TEST_F(FMMUMultiPDOTest, ClearResetsPDOEntries) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    configs.push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
    }));

    mgr_->configureFromMultiPDO(configs, 0);
    EXPECT_EQ(mgr_->config().pdo_entry_count, 1u);

    mgr_->config().clear();
    EXPECT_EQ(mgr_->config().pdo_entry_count, 0u);
    EXPECT_EQ(mgr_->config().fmmu_count, 0u);
}

TEST_F(FMMUMultiPDOTest, WriteToSlaveWithMultiPDO) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    configs.push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));
    configs.push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
    }));

    mgr_->configureFromMultiPDO(configs, 0);

    EXPECT_CALL(transport_, apwr(_, _, _, _)).WillRepeatedly(Return(true));
    EXPECT_TRUE(mgr_->writeToSlave());
    EXPECT_TRUE(mgr_->config().configured);
}

TEST_F(FMMUMultiPDOTest, CustomSMIndex4) {
    std::vector<MultiPDOSyncManagerConfig> configs;
    auto cfg = MultiPDOSyncManagerConfig::process_output(0x2000, {
        {0x1600, 8},
    });
    cfg.sm_index = 4;
    configs.push_back(cfg);

    bool ok = mgr_->configureFromMultiPDO(configs, 0);
    EXPECT_TRUE(ok);
    EXPECT_EQ(mgr_->config().fmmus[0].associated_sm, 4u);

    const FMMUPDOEntry* entry = mgr_->findPDOEntry(0x1600);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->sm_index, 4u);
}

// ============================================================================
// Part 3: SlaveFMMUConfig direct addOutputWithPDOs / addInputWithPDOs
// ============================================================================

TEST(SlaveFMMUConfigMultiPDOTest, AddOutputWithPDOs) {
    SlaveFMMUConfig cfg;
    PDOMappingRegion pdos[] = {{0x1600, 8}, {0x1601, 4}};
    bool ok = cfg.addOutputWithPDOs(0x1800, 2, pdos, 2);
    EXPECT_TRUE(ok);
    EXPECT_EQ(cfg.fmmu_count, 1u);
    EXPECT_EQ(cfg.pdo_entry_count, 2u);
    EXPECT_EQ(cfg.fmmus[0].length, 12u);

    // Check per-PDO entries
    const FMMUPDOEntry* e0 = cfg.findPDOByIndex(0x1600);
    ASSERT_NE(e0, nullptr);
    EXPECT_EQ(e0->logical_addr, 0u);
    EXPECT_EQ(e0->physical_offset, 0u);
    EXPECT_EQ(e0->size_bytes, 8u);

    const FMMUPDOEntry* e1 = cfg.findPDOByIndex(0x1601);
    ASSERT_NE(e1, nullptr);
    EXPECT_EQ(e1->logical_addr, 8u);
    EXPECT_EQ(e1->physical_offset, 8u);
    EXPECT_EQ(e1->size_bytes, 4u);
}

TEST(SlaveFMMUConfigMultiPDOTest, AddInputWithPDOs) {
    SlaveFMMUConfig cfg;
    PDOMappingRegion pdos[] = {{0x1A00, 16}, {0x1A01, 8}};
    bool ok = cfg.addInputWithPDOs(0x1C00, 3, pdos, 2);
    EXPECT_TRUE(ok);
    EXPECT_EQ(cfg.fmmu_count, 1u);
    EXPECT_EQ(cfg.pdo_entry_count, 2u);
    EXPECT_EQ(cfg.fmmus[0].length, 24u);
}

TEST(SlaveFMMUConfigMultiPDOTest, AddWithPDOsEmptyReturnsFalse) {
    SlaveFMMUConfig cfg;
    PDOMappingRegion pdos[] = {{0x1600, 0}};
    bool ok = cfg.addOutputWithPDOs(0x1800, 2, pdos, 1);
    EXPECT_FALSE(ok);
}

TEST(SlaveFMMUConfigMultiPDOTest, AddWithPDOsZeroCountReturnsFalse) {
    SlaveFMMUConfig cfg;
    bool ok = cfg.addOutputWithPDOs(0x1800, 2, nullptr, 0);
    EXPECT_FALSE(ok);
}

TEST(SlaveFMMUConfigMultiPDOTest, FindPDOByIndexNotFound) {
    SlaveFMMUConfig cfg;
    EXPECT_EQ(cfg.findPDOByIndex(0x1600), nullptr);
}

TEST(SlaveFMMUConfigMultiPDOTest, ClearPDOEntries) {
    SlaveFMMUConfig cfg;
    PDOMappingRegion pdos[] = {{0x1600, 8}};
    cfg.addOutputWithPDOs(0x1800, 2, pdos, 1);
    EXPECT_EQ(cfg.pdo_entry_count, 1u);

    cfg.clearPDOEntries();
    EXPECT_EQ(cfg.pdo_entry_count, 0u);
    // FMMUs should still be there
    EXPECT_EQ(cfg.fmmu_count, 1u);
}

// ============================================================================
// Part 4: LogicalAddressManager multi-PDO
// ============================================================================

class LogicalAddrMultiPDOTest : public ::testing::Test {
protected:
    void SetUp() override {
        mgr_ = std::make_unique<LogicalAddressManager>(transport_);
        mgr_->init();
    }

    NiceMock<MockPDOTransport> transport_;
    std::unique_ptr<LogicalAddressManager> mgr_;
};

TEST_F(LogicalAddrMultiPDOTest, BuildFromMultiPDOSingleSlave) {
    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs(1);
    configs[0].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));
    configs[0].push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
    }));

    bool ok = mgr_->buildAddressMapFromMultiPDO(configs.data(), 1);
    EXPECT_TRUE(ok);

    EXPECT_EQ(mgr_->totalRxPDOBytes(), 12u);
    EXPECT_EQ(mgr_->totalTxPDOBytes(), 16u);
    EXPECT_EQ(mgr_->totalLogicalSize(), 28u);

    // Per-PDO queries
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0x1600), 0x10000u);
    EXPECT_EQ(mgr_->getPDOLength(0, 0x1600), 8u);
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0x1601), 0x10008u);
    EXPECT_EQ(mgr_->getPDOLength(0, 0x1601), 4u);
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0x1A00), 0x1000Cu);
    EXPECT_EQ(mgr_->getPDOLength(0, 0x1A00), 16u);
}

TEST_F(LogicalAddrMultiPDOTest, BuildFromMultiPDOMultiSlave) {
    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs(2);

    // Slave 0: 2 RxPDOs (12 bytes), 1 TxPDO (16 bytes)
    configs[0].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));
    configs[0].push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
    }));

    // Slave 1: 1 RxPDO (6 bytes), 1 TxPDO (8 bytes)
    configs[1].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 6},
    }));
    configs[1].push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 8},
    }));

    bool ok = mgr_->buildAddressMapFromMultiPDO(configs.data(), 2);
    EXPECT_TRUE(ok);

    EXPECT_EQ(mgr_->totalRxPDOBytes(), 18u);  // 12 + 6
    EXPECT_EQ(mgr_->totalTxPDOBytes(), 24u);  // 16 + 8

    // Slave 0 PDOs
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0x1600), 0x10000u);
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0x1601), 0x10008u);
    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0x1A00), 0x10012u);  // 0x10000 + 18

    // Slave 1 PDOs
    EXPECT_EQ(mgr_->getPDOLogicalAddr(1, 0x1600), 0x1000Cu);  // 0x10000 + 12
    EXPECT_EQ(mgr_->getPDOLogicalAddr(1, 0x1A00), 0x10022u);  // 0x10012 + 16
}

TEST_F(LogicalAddrMultiPDOTest, GetSlavePDOLogicalAddrs) {
    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs(1);
    configs[0].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
        {0x1601, 4},
    }));
    configs[0].push_back(MultiPDOSyncManagerConfig::process_input(0x1C00, {
        {0x1A00, 16},
    }));

    mgr_->buildAddressMapFromMultiPDO(configs.data(), 1);

    auto entries = mgr_->getSlavePDOLogicalAddrs(0);
    EXPECT_EQ(entries.size(), 3u);

    EXPECT_EQ(entries[0].pdo_index, 0x1600u);
    EXPECT_EQ(entries[0].is_output, true);
    EXPECT_EQ(entries[0].sm_index, 2u);

    EXPECT_EQ(entries[1].pdo_index, 0x1601u);
    EXPECT_EQ(entries[1].is_output, true);

    EXPECT_EQ(entries[2].pdo_index, 0x1A00u);
    EXPECT_EQ(entries[2].is_output, false);
    EXPECT_EQ(entries[2].sm_index, 3u);
}

TEST_F(LogicalAddrMultiPDOTest, GetSlavePDOLogicalAddrsInvalidSlave) {
    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs(1);
    configs[0].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
    }));

    mgr_->buildAddressMapFromMultiPDO(configs.data(), 1);

    auto entries = mgr_->getSlavePDOLogicalAddrs(99);
    EXPECT_TRUE(entries.empty());
}

TEST_F(LogicalAddrMultiPDOTest, GetPDOLogicalAddrNotFound) {
    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs(1);
    configs[0].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
    }));

    mgr_->buildAddressMapFromMultiPDO(configs.data(), 1);

    EXPECT_EQ(mgr_->getPDOLogicalAddr(0, 0xFFFF), 0u);
    EXPECT_EQ(mgr_->getPDOLength(0, 0xFFFF), 0u);
    EXPECT_EQ(mgr_->getPDOLogicalAddr(99, 0x1600), 0u);
}

TEST_F(LogicalAddrMultiPDOTest, BuildFromMultiPDOEmptyReturnsFalse) {
    bool ok = mgr_->buildAddressMapFromMultiPDO(nullptr, 1);
    EXPECT_FALSE(ok);

    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs;
    ok = mgr_->buildAddressMapFromMultiPDO(configs.data(), 0);
    EXPECT_FALSE(ok);
}

TEST_F(LogicalAddrMultiPDOTest, HasSlavePDOsAfterMultiPDOBuild) {
    std::vector<std::vector<MultiPDOSyncManagerConfig>> configs(2);
    configs[0].push_back(MultiPDOSyncManagerConfig::process_output(0x1800, {
        {0x1600, 8},
    }));
    // Slave 1 has no PDOs

    mgr_->buildAddressMapFromMultiPDO(configs.data(), 2);

    EXPECT_TRUE(mgr_->hasSlavePDOs(0));
    EXPECT_FALSE(mgr_->hasSlavePDOs(1));
}

// ============================================================================
// Part 5: FMMUPDOEntry struct
// ============================================================================

TEST(FMMUPDOEntryTest, DefaultConstruction) {
    FMMUPDOEntry entry;
    EXPECT_EQ(entry.pdo_index, 0u);
    EXPECT_EQ(entry.logical_addr, 0u);
    EXPECT_EQ(entry.physical_offset, 0u);
    EXPECT_EQ(entry.size_bytes, 0u);
    EXPECT_EQ(entry.sm_index, 0xFFu);
    EXPECT_FALSE(entry.isValid());
}

TEST(FMMUPDOEntryTest, ValidEntry) {
    FMMUPDOEntry entry;
    entry.pdo_index = 0x1600;
    entry.size_bytes = 8;
    EXPECT_TRUE(entry.isValid());
}
