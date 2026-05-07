/**
 * @file test_pdo_manager.cpp
 * @brief Comprehensive tests for the refactored PDOManager (no global state).
 *
 * Uses a MockPDOTransport to verify behavior without any real network I/O.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cstring>

#include "tether/ethercat/EtherCATPDO.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;

// ============================================================================
// MockPDOTransport
// ============================================================================

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
    MOCK_METHOD(bool, waitForResponseIdx,
                (uint8_t idx, unsigned int timeout_ms, RxDatagram& out),
                (override));
    MOCK_METHOD(uint8_t, allocIdx, (), (override));
    MOCK_METHOD(uint16_t, adpForSlaveIndex, (uint16_t slave_index), (override));
};

// ============================================================================
// NiceMock convenience alias
// ============================================================================
using NiceMockTransport = ::testing::NiceMock<MockPDOTransport>;

// ============================================================================
// PDOMapping Tests
// ============================================================================

class PDOMappingTest : public ::testing::Test {
protected:
    PDOMapping mapping;
};

TEST_F(PDOMappingTest, InitialStateIsEmpty) {
    EXPECT_EQ(mapping.entry_count(), 0u);
    EXPECT_EQ(mapping.total_rxpdo_bytes(), 0u);
    EXPECT_EQ(mapping.total_txpdo_bytes(), 0u);
    EXPECT_EQ(mapping.get_entry(0), nullptr);
}

TEST_F(PDOMappingTest, AddRxPDOSuccess) {
    uint32_t buf = 0;
    int idx = mapping.add_rxpdo(0, &buf, sizeof(buf));
    ASSERT_GE(idx, 0);
    EXPECT_EQ(mapping.entry_count(), 1u);
    EXPECT_EQ(mapping.total_rxpdo_bytes(), sizeof(buf));
    EXPECT_EQ(mapping.total_txpdo_bytes(), 0u);

    const PDOEntry* e = mapping.get_entry(static_cast<size_t>(idx));
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->slave_index, 0u);
    EXPECT_EQ(e->direction, PDODirection::RxPDO);
    EXPECT_EQ(e->address_mode, PDOAddressMode::Position);
    EXPECT_EQ(e->data_size, sizeof(buf));
    EXPECT_TRUE(e->enabled);
    EXPECT_EQ(e->error_count, 0u);
    EXPECT_EQ(e->success_count, 0u);
}

TEST_F(PDOMappingTest, AddTxPDOSuccess) {
    uint16_t buf = 0;
    int idx = mapping.add_txpdo(1, &buf, sizeof(buf), 0x1A00, PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_EQ(mapping.entry_count(), 1u);
    EXPECT_EQ(mapping.total_txpdo_bytes(), sizeof(buf));

    const PDOEntry* e = mapping.get_entry(static_cast<size_t>(idx));
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->direction, PDODirection::TxPDO);
    EXPECT_EQ(e->address_mode, PDOAddressMode::ConfiguredAddress);
    EXPECT_EQ(e->slave_index, 1u);
}

TEST_F(PDOMappingTest, NullBufferReturnsError) {
    EXPECT_EQ(mapping.add_rxpdo(0, nullptr, 4), -1);
    EXPECT_EQ(mapping.add_txpdo(0, nullptr, 4), -1);
    EXPECT_EQ(mapping.entry_count(), 0u);
}

TEST_F(PDOMappingTest, ZeroSizeReturnsError) {
    uint8_t buf = 0;
    EXPECT_EQ(mapping.add_rxpdo(0, &buf, 0), -1);
    EXPECT_EQ(mapping.add_txpdo(0, &buf, 0), -1);
}

TEST_F(PDOMappingTest, OversizedReturnsError) {
    uint8_t buf[1] = {0};
    EXPECT_EQ(mapping.add_rxpdo(0, buf, static_cast<uint16_t>(kMaxPDOSize + 1)), -1);
}

TEST_F(PDOMappingTest, CapacityLimitEnforced) {
    uint8_t buf = 0;
    for (size_t i = 0; i < kMaxPDOEntries; i++) {
        ASSERT_GE(mapping.add_rxpdo(0, &buf, 1), 0) << "Failed at entry " << i;
    }
    // Next add should fail
    EXPECT_EQ(mapping.add_rxpdo(0, &buf, 1), -1);
    EXPECT_EQ(mapping.entry_count(), kMaxPDOEntries);
}

TEST_F(PDOMappingTest, ClearResetsState) {
    uint8_t buf = 0;
    mapping.add_rxpdo(0, &buf, 1);
    mapping.add_txpdo(0, &buf, 1);
    ASSERT_GT(mapping.entry_count(), 0u);

    mapping.clear();
    EXPECT_EQ(mapping.entry_count(), 0u);
    EXPECT_EQ(mapping.total_rxpdo_bytes(), 0u);
    EXPECT_EQ(mapping.total_txpdo_bytes(), 0u);
}

TEST_F(PDOMappingTest, BroadcastEntries) {
    uint32_t buf = 0;
    int rx_idx = mapping.add_broadcast_rxpdo(&buf, sizeof(buf), 0x1000);
    ASSERT_GE(rx_idx, 0);
    const PDOEntry* e = mapping.get_entry(static_cast<size_t>(rx_idx));
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->address_mode, PDOAddressMode::Broadcast);
    EXPECT_EQ(e->physical_offset, 0x1000u);
    EXPECT_EQ(e->slave_index, 0xFFFFu);

    int tx_idx = mapping.add_broadcast_txpdo(&buf, sizeof(buf), 0x1100);
    ASSERT_GE(tx_idx, 0);
    EXPECT_EQ(mapping.entry_count(), 2u);
}

TEST_F(PDOMappingTest, SetSlaveConfiguredAddressUpdatesExisting) {
    uint32_t buf = 0;
    mapping.add_rxpdo(2, &buf, sizeof(buf));
    mapping.set_slave_configured_address(2, 0x1002);

    const PDOEntry* e = mapping.get_entry(0);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->configured_address, 0x1002u);
}

TEST_F(PDOMappingTest, GetEntryMutable) {
    uint8_t buf = 0;
    mapping.add_rxpdo(0, &buf, 1);
    PDOEntry* mut = mapping.get_entry_mut(0);
    ASSERT_NE(mut, nullptr);
    mut->enabled = false;
    EXPECT_FALSE(mapping.get_entry(0)->enabled);
}

TEST_F(PDOMappingTest, OutOfBoundsReturnsNull) {
    EXPECT_EQ(mapping.get_entry(0), nullptr);
    EXPECT_EQ(mapping.get_entry_mut(0), nullptr);
    EXPECT_EQ(mapping.get_entry(999), nullptr);
}

// ============================================================================
// PDOManager Lifecycle Tests
// ============================================================================

class PDOManagerTest : public ::testing::Test {
protected:
    NiceMockTransport transport;
    PDOManager mgr{transport};

    void SetUp() override {
        // Default: adpForSlaveIndex returns negated index (auto-increment)
        ON_CALL(transport, adpForSlaveIndex(_))
            .WillByDefault([](uint16_t idx) { return static_cast<uint16_t>(0u - idx); });
        ON_CALL(transport, allocIdx())
            .WillByDefault(Return(1));
    }
};

TEST_F(PDOManagerTest, InitAndDeinit) {
    EXPECT_FALSE(mgr.isInitialized());
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());
    // Double init is idempotent
    EXPECT_TRUE(mgr.init());
    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());
}

TEST_F(PDOManagerTest, MappingAccess) {
    mgr.init();
    uint32_t buf = 0;
    mgr.mapping().add_rxpdo(0, &buf, sizeof(buf));
    EXPECT_EQ(mgr.mapping().entry_count(), 1u);
}

TEST_F(PDOManagerTest, SlaveConfigArrayAccess) {
    mgr.init();
    auto* configs = mgr.slaveConfigs();
    ASSERT_NE(configs, nullptr);

    configs[0].configured_address = 0x1001;
    configs[0].vendor_id = 0xDEAD;
    EXPECT_EQ(mgr.slaveConfigs()[0].configured_address, 0x1001u);
    EXPECT_EQ(mgr.slaveConfigs()[0].vendor_id, 0xDEADu);
}

TEST_F(PDOManagerTest, SlaveCount) {
    mgr.init();
    EXPECT_EQ(mgr.slaveCount(), 0u);
    mgr.setSlaveCount(3);
    EXPECT_EQ(mgr.slaveCount(), 3u);
}

TEST_F(PDOManagerTest, StatsInitiallyZero) {
    mgr.init();
    auto s = mgr.getStats();
    EXPECT_EQ(s.total_cycles, 0u);
    EXPECT_EQ(s.rxpdo_frames_sent, 0u);
    EXPECT_EQ(s.txpdo_frames_recv, 0u);
    EXPECT_EQ(s.rxpdo_errors, 0u);
    EXPECT_EQ(s.txpdo_errors, 0u);
}

TEST_F(PDOManagerTest, StatsReset) {
    mgr.init();
    mgr.statsRef().total_cycles = 42;
    auto s = mgr.getStats();
    EXPECT_EQ(s.total_cycles, 42u);
    mgr.resetStats();
    s = mgr.getStats();
    EXPECT_EQ(s.total_cycles, 0u);
}

TEST_F(PDOManagerTest, ModeSettings) {
    mgr.init();
    EXPECT_FALSE(mgr.getSeparateMode());
    EXPECT_FALSE(mgr.getPhysicalMode());

    mgr.setSeparateMode(true);
    EXPECT_TRUE(mgr.getSeparateMode());
    mgr.setSeparateMode(false);
    EXPECT_FALSE(mgr.getSeparateMode());

    mgr.setPhysicalMode(true);
    EXPECT_TRUE(mgr.getPhysicalMode());
    mgr.setPhysicalMode(false);
    EXPECT_FALSE(mgr.getPhysicalMode());
}

TEST_F(PDOManagerTest, TransportAccess) {
    EXPECT_EQ(&mgr.transport(), &transport);
}

// ============================================================================
// SM Configuration Tests
// ============================================================================

TEST_F(PDOManagerTest, ConfigureSlavesSMsCallsTransport) {
    mgr.init();

    auto* cfg = &mgr.slaveConfigs()[0];
    cfg->sm[2] = SyncManagerConfig::process_output(0x1100, 8);
    cfg->sm[3] = SyncManagerConfig::process_input(0x1180, 4);

    // Each SM write: disable(1) + phys_addr(1) + length(1) + control(1) + activate(1) = 5 calls per SM
    // SM0 and SM1 are Unused, SM2 and SM3 are configured → 10 register writes
    EXPECT_CALL(transport, writeRegister(_, _, _, _, _))
        .Times(::testing::AtLeast(10))
        .WillRepeatedly(Return(true));

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_TRUE(cfg->configured);
}

TEST_F(PDOManagerTest, ConfigureSlavesSMsInvalidIndex) {
    mgr.init();
    EXPECT_FALSE(mgr.configureSlavesSMs(static_cast<uint16_t>(kMaxPDOSlaves)));
}

TEST_F(PDOManagerTest, ConfigureSlavesSMsFailsPropagates) {
    // Use a separate strict mock to avoid interfering with the fixture's NiceMock.
    NiceMockTransport strict_transport;
    PDOManager strict_mgr(strict_transport);
    strict_mgr.init();

    ON_CALL(strict_transport, adpForSlaveIndex(_))
        .WillByDefault(Return(0));

    auto* cfg = &strict_mgr.slaveConfigs()[0];
    cfg->sm[2] = SyncManagerConfig::process_output(0x1100, 8);

    // First call (disable) succeeds, second (phys_addr) fails
    EXPECT_CALL(strict_transport, writeRegister(_, _, _, _, _))
        .WillOnce(Return(true))   // disable
        .WillOnce(Return(false)); // phys_addr → failure

    EXPECT_FALSE(strict_mgr.configureSlavesSMs(0));
}

TEST_F(PDOManagerTest, ConfigureAllSlaveSMs) {
    mgr.init();
    // Configure SM for 2 slaves
    for (int i = 0; i < 2; i++) {
        mgr.slaveConfigs()[i].sm[2] = SyncManagerConfig::process_output(0x1100, 8);
    }

    EXPECT_CALL(transport, writeRegister(_, _, _, _, _))
        .WillRepeatedly(Return(true));

    uint16_t cnt = mgr.configureAllSlaveSMs(2);
    EXPECT_EQ(cnt, 2u);
    EXPECT_EQ(mgr.slaveCount(), 2u);
}

// ============================================================================
// SM Register Address Tests
// ============================================================================

TEST_F(PDOManagerTest, WriteSMConfigUsesCorrectRegisters) {
    mgr.init();

    auto* cfg = &mgr.slaveConfigs()[0];
    cfg->sm[2] = SyncManagerConfig::process_output(0x1100, 8);

    // SM2 base = 0x0810.  Expect writes to:
    //   0x0816 (activate/disable), 0x0810 (phys_addr), 0x0812 (length),
    //   0x0814 (control), 0x0816 (activate/enable)
    std::vector<uint16_t> addresses_written;

    ON_CALL(transport, writeRegister(_, _, _, _, _))
        .WillByDefault([&addresses_written](uint16_t, uint16_t ado, const void*, uint16_t, unsigned) {
            addresses_written.push_back(ado);
            return true;
        });

    EXPECT_TRUE(mgr.configureSlavesSMs(0));

    ASSERT_GE(addresses_written.size(), 5u);
    EXPECT_EQ(addresses_written[0], 0x0816u);  // SM2 activate (disable)
    EXPECT_EQ(addresses_written[1], 0x0810u);  // SM2 phys_addr
    EXPECT_EQ(addresses_written[2], 0x0812u);  // SM2 length
    EXPECT_EQ(addresses_written[3], 0x0814u);  // SM2 control
    EXPECT_EQ(addresses_written[4], 0x0816u);  // SM2 activate (enable)
}

// ============================================================================
// Mapping Finalization Tests
// ============================================================================

TEST_F(PDOManagerTest, FinalizeMappingSetsPhysicalOffsets) {
    mgr.init();

    auto* cfg = &mgr.slaveConfigs()[0];
    cfg->sm[2] = SyncManagerConfig::process_output(0x1100, 0);
    cfg->sm[3] = SyncManagerConfig::process_input(0x1180, 0);

    uint32_t rxbuf = 0, txbuf = 0;
    mgr.mapping().add_rxpdo(0, &rxbuf, sizeof(rxbuf));
    mgr.mapping().add_txpdo(0, &txbuf, sizeof(txbuf));

    EXPECT_TRUE(mgr.finalizeMapping(0));

    // Check that entry offsets match SM addresses
    const PDOEntry* rx = mgr.mapping().get_entry(0);
    ASSERT_NE(rx, nullptr);
    EXPECT_EQ(rx->physical_offset, 0x1100u);

    const PDOEntry* tx = mgr.mapping().get_entry(1);
    ASSERT_NE(tx, nullptr);
    EXPECT_EQ(tx->physical_offset, 0x1180u);

    // SM lengths updated
    EXPECT_EQ(cfg->sm[2].length, sizeof(rxbuf));
    EXPECT_EQ(cfg->rxpdo_size, sizeof(rxbuf));
    EXPECT_EQ(cfg->sm[3].length, sizeof(txbuf));
    EXPECT_EQ(cfg->txpdo_size, sizeof(txbuf));
}

TEST_F(PDOManagerTest, FinalizeMappingInvalidSlaveIndex) {
    mgr.init();
    EXPECT_FALSE(mgr.finalizeMapping(static_cast<uint16_t>(kMaxPDOSlaves)));
}

// ============================================================================
// PDO Transfer Tests
// ============================================================================

TEST_F(PDOManagerTest, SendRxPDOPositionMode) {
    mgr.init();

    uint32_t buf = 0xDEADBEEF;
    int idx = mgr.mapping().add_rxpdo(0, &buf, sizeof(buf), 0x1600, PDOAddressMode::Position);
    ASSERT_GE(idx, 0);

    // Position mode uses sendSingleDatagram with APWR
    EXPECT_CALL(transport, sendSingleDatagram(Command::APWR, _, _, _, _, sizeof(buf), _))
        .WillOnce(Return(true));

    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(mgr.getStats().rxpdo_frames_sent, 1u);
}

TEST_F(PDOManagerTest, ReceiveTxPDOPositionMode) {
    mgr.init();

    uint32_t buf = 0;
    int idx = mgr.mapping().add_txpdo(0, &buf, sizeof(buf), 0x1A00, PDOAddressMode::Position);
    ASSERT_GE(idx, 0);

    EXPECT_CALL(transport, sendSingleDatagram(Command::APRD, _, _, _, _, sizeof(buf), true))
        .WillOnce(Return(true));

    EXPECT_TRUE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(mgr.getStats().txpdo_frames_recv, 1u);
}

TEST_F(PDOManagerTest, SendRxPDOConfiguredMode) {
    mgr.init();

    uint32_t buf = 0xAA;
    int idx = mgr.mapping().add_rxpdo(0, &buf, sizeof(buf), 0x1600, PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    // ConfiguredAddress mode uses sendSingleDatagram(FPWR) then waitForResponseIdx
    EXPECT_CALL(transport, sendSingleDatagram(Command::FPWR, _, _, _, _, _, true))
        .WillOnce(Return(true));
    RxDatagram resp{};
    resp.wkc = 1;
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(::testing::DoAll(::testing::SetArgReferee<2>(resp), Return(true)));

    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
}

TEST_F(PDOManagerTest, ReceiveTxPDOConfiguredMode) {
    mgr.init();

    uint32_t buf = 0;
    int idx = mgr.mapping().add_txpdo(0, &buf, sizeof(buf), 0x1A00, PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    EXPECT_CALL(transport, sendSingleDatagram(Command::FPRD, _, _, _, _, _, true))
        .WillOnce(Return(true));

    RxDatagram resp{};
    resp.wkc = 1;
    resp.datalen = sizeof(buf);
    uint32_t response_data = 0x12345678;
    std::memcpy(resp.data, &response_data, sizeof(response_data));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(::testing::DoAll(::testing::SetArgReferee<2>(resp), Return(true)));

    EXPECT_TRUE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(buf, 0x12345678u);
}

TEST_F(PDOManagerTest, SendRxPDOBroadcastMode) {
    mgr.init();

    uint32_t buf = 0xBB;
    int idx = mgr.mapping().add_broadcast_rxpdo(&buf, sizeof(buf), 0x2000);
    ASSERT_GE(idx, 0);

    EXPECT_CALL(transport, sendSingleDatagram(Command::BWR, _, _, _, _, _, true))
        .WillOnce(Return(true));
    RxDatagram resp{};
    resp.wkc = 1;
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(::testing::DoAll(::testing::SetArgReferee<2>(resp), Return(true)));

    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
}

TEST_F(PDOManagerTest, ReceiveTxPDOBroadcastMode) {
    mgr.init();

    uint32_t buf = 0;
    int idx = mgr.mapping().add_broadcast_txpdo(&buf, sizeof(buf), 0x2100);
    ASSERT_GE(idx, 0);

    EXPECT_CALL(transport, sendSingleDatagram(Command::BRD, _, _, _, _, _, true))
        .WillOnce(Return(true));

    RxDatagram resp{};
    resp.wkc = 1;
    resp.datalen = sizeof(buf);
    uint32_t resp_data = 0xCAFEBABE;
    std::memcpy(resp.data, &resp_data, sizeof(resp_data));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(::testing::DoAll(::testing::SetArgReferee<2>(resp), Return(true)));

    EXPECT_TRUE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(buf, 0xCAFEBABEu);
}

TEST_F(PDOManagerTest, SendRxPDOLogicalModeNotImplemented) {
    mgr.init();

    uint32_t buf = 0;
    int idx = mgr.mapping().add_rxpdo(0, &buf, sizeof(buf), 0x1600, PDOAddressMode::Logical);
    ASSERT_GE(idx, 0);

    // Logical mode returns false (not yet implemented)
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(mgr.getStats().rxpdo_errors, 1u);
}

TEST_F(PDOManagerTest, SendRxPDOInvalidIndex) {
    mgr.init();
    EXPECT_FALSE(mgr.sendRxPDO(999));
}

TEST_F(PDOManagerTest, SendRxPDOWrongDirection) {
    mgr.init();
    uint32_t buf = 0;
    int idx = mgr.mapping().add_txpdo(0, &buf, sizeof(buf));
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
}

TEST_F(PDOManagerTest, ReceiveTxPDOWrongDirection) {
    mgr.init();
    uint32_t buf = 0;
    int idx = mgr.mapping().add_rxpdo(0, &buf, sizeof(buf));
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
}

// ============================================================================
// ExchangeAll Tests
// ============================================================================

TEST_F(PDOManagerTest, ExchangeAllSendsAndReceives) {
    mgr.init();

    uint32_t tx_buf = 0xAA, rx_buf = 0;
    mgr.mapping().add_rxpdo(0, &tx_buf, sizeof(tx_buf), 0x1600, PDOAddressMode::Position);
    mgr.mapping().add_txpdo(0, &rx_buf, sizeof(rx_buf), 0x1A00, PDOAddressMode::Position);

    EXPECT_CALL(transport, sendSingleDatagram(Command::APWR, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, sendSingleDatagram(Command::APRD, _, _, _, _, _, _))
        .WillOnce(Return(true));

    // exchangeAll should succeed and increment total_cycles
    bool ok = mgr.exchangeAll();
    EXPECT_TRUE(ok);
    EXPECT_EQ(mgr.getStats().total_cycles, 1u);
    EXPECT_EQ(mgr.getStats().rxpdo_frames_sent, 1u);
    EXPECT_EQ(mgr.getStats().txpdo_frames_recv, 1u);
}

TEST_F(PDOManagerTest, ExchangeAllWithDisabledEntry) {
    mgr.init();

    uint32_t buf = 0;
    int idx = mgr.mapping().add_rxpdo(0, &buf, sizeof(buf));
    ASSERT_GE(idx, 0);
    mgr.mapping().get_entry_mut(static_cast<size_t>(idx))->enabled = false;

    // Disabled entries should be skipped
    bool ok = mgr.exchangeAll();
    EXPECT_TRUE(ok);
    EXPECT_EQ(mgr.getStats().rxpdo_frames_sent, 0u);
}

// ============================================================================
// ExchangePhysical Tests
// ============================================================================

TEST_F(PDOManagerTest, ExchangePhysicalZeroSlaves) {
    mgr.init();
    EXPECT_TRUE(mgr.exchangePhysical(0));
}

TEST_F(PDOManagerTest, ExchangePhysicalSendsAndReceives) {
    mgr.init();

    auto* cfg = &mgr.slaveConfigs()[0];
    cfg->configured = true;
    cfg->sm[2] = SyncManagerConfig::process_output(0x1100, 4);
    cfg->sm[3] = SyncManagerConfig::process_input(0x1180, 4);

    uint32_t out_buf = 0xBBCC, in_buf = 0;
    mgr.mapping().add_rxpdo(0, &out_buf, sizeof(out_buf));
    mgr.mapping().add_txpdo(0, &in_buf, sizeof(in_buf));

    // writeRegister for RxPDO (FPWR-style)
    EXPECT_CALL(transport, writeRegister(_, 0x1100, _, 4, _))
        .WillOnce(Return(true));

    // readRegister for TxPDO (FPRD-style)
    uint32_t hw_data = 0x55AA55AA;
    EXPECT_CALL(transport, readRegister(_, 0x1180, _, 4, _))
        .WillOnce([&hw_data](uint16_t, uint16_t, void* data, uint16_t, unsigned) {
            std::memcpy(data, &hw_data, sizeof(hw_data));
            return true;
        });

    EXPECT_TRUE(mgr.exchangePhysical(1));

    auto ps = mgr.getPhysicalStats();
    EXPECT_EQ(ps.fpwr_success, 1u);
    EXPECT_EQ(ps.fprd_success, 1u);

    EXPECT_EQ(in_buf, 0x55AA55AAu);
}

// ============================================================================
// ExchangeLRW / ExchangeSeparate placeholder behavior
// ============================================================================

TEST_F(PDOManagerTest, ExchangeLRWZeroSlavesReturnsTrue) {
    mgr.init();
    EXPECT_TRUE(mgr.exchangeLRW(0));
}

TEST_F(PDOManagerTest, ExchangeSeparateZeroSlavesReturnsTrue) {
    mgr.init();
    EXPECT_TRUE(mgr.exchangeSeparate(0));
}

// ============================================================================
// Multiple Independent Instances
// ============================================================================

TEST(PDOManagerIndependence, TwoInstancesAreIsolated) {
    NiceMockTransport t1, t2;
    ON_CALL(t1, adpForSlaveIndex(_)).WillByDefault(Return(0));
    ON_CALL(t2, adpForSlaveIndex(_)).WillByDefault(Return(0));

    PDOManager m1(t1), m2(t2);
    m1.init();
    m2.init();

    uint32_t b1 = 0, b2 = 0;
    m1.mapping().add_rxpdo(0, &b1, sizeof(b1));
    EXPECT_EQ(m1.mapping().entry_count(), 1u);
    EXPECT_EQ(m2.mapping().entry_count(), 0u);

    m1.slaveConfigs()[0].vendor_id = 0xAAAA;
    m2.slaveConfigs()[0].vendor_id = 0xBBBB;
    EXPECT_EQ(m1.slaveConfigs()[0].vendor_id, 0xAAAAu);
    EXPECT_EQ(m2.slaveConfigs()[0].vendor_id, 0xBBBBu);

    m1.setSeparateMode(true);
    EXPECT_TRUE(m1.getSeparateMode());
    EXPECT_FALSE(m2.getSeparateMode());

    m1.statsRef().total_cycles = 100;
    EXPECT_EQ(m1.getStats().total_cycles, 100u);
    EXPECT_EQ(m2.getStats().total_cycles, 0u);
}

// ============================================================================
// Per-Mode Stats Tests
// ============================================================================

TEST_F(PDOManagerTest, LRWStatsInitiallyZero) {
    mgr.init();
    auto s = mgr.getLRWStats();
    EXPECT_EQ(s.lrw_success, 0u);
    EXPECT_EQ(s.lrw_wkc_errors, 0u);
    EXPECT_EQ(s.lrw_send_errors, 0u);
    EXPECT_EQ(s.lrw_timeout_errors, 0u);
}

TEST_F(PDOManagerTest, SeparateStatsInitiallyZero) {
    mgr.init();
    auto s = mgr.getSeparateStats();
    EXPECT_EQ(s.lwr_success, 0u);
    EXPECT_EQ(s.lwr_wkc_errors, 0u);
    EXPECT_EQ(s.lrd_success, 0u);
    EXPECT_EQ(s.lrd_wkc_errors, 0u);
}

TEST_F(PDOManagerTest, PhysicalStatsInitiallyZero) {
    mgr.init();
    auto s = mgr.getPhysicalStats();
    EXPECT_EQ(s.fpwr_success, 0u);
    EXPECT_EQ(s.fpwr_wkc_errors, 0u);
    EXPECT_EQ(s.fprd_success, 0u);
    EXPECT_EQ(s.fprd_wkc_errors, 0u);
}

TEST_F(PDOManagerTest, TransferStatsInitiallyZero) {
    mgr.init();
    auto s = mgr.getTransferStats();
    EXPECT_EQ(s.rxpdo_debug_count, 0u);
    EXPECT_EQ(s.rxpdo_confirmed_ok, 0u);
    EXPECT_EQ(s.rxpdo_confirmed_fail, 0u);
    EXPECT_EQ(s.txpdo_debug_count, 0u);
}

// ============================================================================
// Backward-Compatible Free Functions (PDOManager& overloads)
// ============================================================================

TEST(PDOFreeFunctions, DelegateCorrectly) {
    NiceMockTransport t;
    ON_CALL(t, adpForSlaveIndex(_)).WillByDefault(Return(0));
    PDOManager m(t);

    EXPECT_TRUE(pdo_init(m));
    EXPECT_TRUE(m.isInitialized());

    auto& mapping = pdo_get_mapping(m);
    uint32_t buf = 0;
    mapping.add_rxpdo(0, &buf, sizeof(buf));
    EXPECT_EQ(mapping.entry_count(), 1u);

    auto* configs = pdo_get_slave_configs(m);
    EXPECT_NE(configs, nullptr);

    pdo_set_separate_mode(m, true);
    EXPECT_TRUE(pdo_get_separate_mode(m));

    pdo_set_physical_mode(m, true);
    EXPECT_TRUE(pdo_get_physical_mode(m));

    auto stats = pdo_get_stats(m);
    EXPECT_EQ(stats.total_cycles, 0u);

    pdo_reset_stats(m);

    pdo_deinit(m);
    EXPECT_FALSE(m.isInitialized());
}

// ============================================================================
// Legacy No-Arg Functions → Instance-Based
// ============================================================================

TEST(PDOLegacy, NoArgFunctionsWork) {
    // Use a local PDOManager instance instead of the global default
    NiceMockTransport transport;
    PDOManager mgr(transport);

    EXPECT_TRUE(mgr.init());
    auto& mapping = mgr.mapping();
    (void)mapping; // access is valid

    auto* configs = mgr.slaveConfigs();
    EXPECT_NE(configs, nullptr);

    mgr.setSeparateMode(true);
    EXPECT_TRUE(mgr.getSeparateMode());
    mgr.setSeparateMode(false);

    mgr.setPhysicalMode(true);
    EXPECT_TRUE(mgr.getPhysicalMode());
    mgr.setPhysicalMode(false);

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.rxpdo_errors, 0u);

    mgr.resetStats();
    mgr.deinit();
}

// ============================================================================
// SyncManagerConfig Static Factories
// ============================================================================

TEST(SyncManagerConfig, MailboxWriteFactory) {
    auto cfg = SyncManagerConfig::mailbox_write(0x1000, 128);
    EXPECT_EQ(cfg.phys_start_addr, 0x1000u);
    EXPECT_EQ(cfg.length, 128u);
    EXPECT_TRUE(cfg.enable);
    EXPECT_EQ(cfg.type, SyncManagerType::MailboxWrite);
}

TEST(SyncManagerConfig, MailboxReadFactory) {
    auto cfg = SyncManagerConfig::mailbox_read(0x1080, 128);
    EXPECT_EQ(cfg.phys_start_addr, 0x1080u);
    EXPECT_EQ(cfg.length, 128u);
    EXPECT_TRUE(cfg.enable);
    EXPECT_EQ(cfg.type, SyncManagerType::MailboxRead);
}

TEST(SyncManagerConfig, ProcessOutputFactory) {
    auto cfg = SyncManagerConfig::process_output(0x1100, 16);
    EXPECT_EQ(cfg.phys_start_addr, 0x1100u);
    EXPECT_EQ(cfg.length, 16u);
    EXPECT_TRUE(cfg.enable);
    EXPECT_EQ(cfg.type, SyncManagerType::ProcessOutput);
    // Output SM should have write direction and watchdog
    EXPECT_NE(cfg.control & SM_CTRL_DIR_WRITE, 0);
    EXPECT_NE(cfg.control & SM_CTRL_WATCHDOG, 0);
}

TEST(SyncManagerConfig, ProcessInputFactory) {
    auto cfg = SyncManagerConfig::process_input(0x1180, 8);
    EXPECT_EQ(cfg.phys_start_addr, 0x1180u);
    EXPECT_EQ(cfg.length, 8u);
    EXPECT_TRUE(cfg.enable);
    EXPECT_EQ(cfg.type, SyncManagerType::ProcessInput);
    // Input SM should have read direction
    EXPECT_EQ(cfg.control & SM_CTRL_DIR_WRITE, 0);
}

// ============================================================================
// Destructor Calls Deinit
// ============================================================================

TEST(PDOManagerLifecycle, DestructorCallsDeinit) {
    NiceMockTransport t;
    {
        PDOManager m(t);
        m.init();
        EXPECT_TRUE(m.isInitialized());
    }
    // After destruction, creating a new manager on same transport should succeed
    PDOManager m2(t);
    m2.init();
    EXPECT_TRUE(m2.isInitialized());
}

// ============================================================================
// Error Counting in Transfer
// ============================================================================

TEST_F(PDOManagerTest, TransferErrorsIncrementStats) {
    mgr.init();

    uint32_t buf = 0;
    int idx = mgr.mapping().add_rxpdo(0, &buf, sizeof(buf), 0x1600, PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    // Fail the send
    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false));

    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(mgr.getStats().rxpdo_errors, 1u);
    EXPECT_EQ(mgr.getStats().rxpdo_frames_sent, 0u);

    // Verify entry error count
    const PDOEntry* e = mgr.mapping().get_entry(static_cast<size_t>(idx));
    EXPECT_EQ(e->error_count, 1u);
}

