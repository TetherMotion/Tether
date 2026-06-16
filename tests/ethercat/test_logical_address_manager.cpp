/**
 * @file test_logical_address_manager.cpp
 * @brief Unit tests for LogicalAddressManager
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cstring>

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::NiceMock;

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

using NiceMockTransport = ::testing::NiceMock<MockPDOTransport>;

// ============================================================================
// AddressMap tests
// ============================================================================

class LogicalAddressManagerTest : public ::testing::Test {
protected:
    NiceMock<MockPDOTransport> transport;
    LogicalAddressManager mgr{transport};

    void SetUp() override {
        mgr.init();
    }
};

TEST_F(LogicalAddressManagerTest, InitState) {
    EXPECT_TRUE(mgr.isInitialized());
    EXPECT_EQ(mgr.totalLogicalSize(), 0u);
    EXPECT_EQ(mgr.totalRxPDOBytes(), 0u);
    EXPECT_EQ(mgr.totalTxPDOBytes(), 0u);
}

TEST_F(LogicalAddressManagerTest, BuildAddressMapSingleSlave) {
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 8);
    configs[0].rxpdo_size = 8;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 12);
    configs[0].txpdo_size = 12;

    EXPECT_TRUE(mgr.buildAddressMap(configs, 1));

    EXPECT_EQ(mgr.totalRxPDOBytes(), 8u);
    EXPECT_EQ(mgr.totalTxPDOBytes(), 12u);
    EXPECT_EQ(mgr.totalLogicalSize(), 20u);

    EXPECT_EQ(mgr.getRxPDOLogicalAddr(0), 0x10000u);
    EXPECT_EQ(mgr.getRxPDOLength(0), 8u);
    EXPECT_EQ(mgr.getTxPDOLogicalAddr(0), 0x10008u);
    EXPECT_EQ(mgr.getTxPDOLength(0), 12u);
    EXPECT_TRUE(mgr.hasSlavePDOs(0));
}

TEST_F(LogicalAddressManagerTest, BuildAddressMapMultiSlave) {
    SlaveConfig configs[kMaxPDOSlaves] = {};

    // Slave 0: 8 byte RxPDO, 12 byte TxPDO
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 8);
    configs[0].rxpdo_size = 8;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 12);
    configs[0].txpdo_size = 12;

    // Slave 1: 4 byte RxPDO, 6 byte TxPDO
    configs[1].configured = true;
    configs[1].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
    configs[1].rxpdo_size = 4;
    configs[1].sm[3] = SyncManagerConfig::process_input(0x1C00, 6);
    configs[1].txpdo_size = 6;

    EXPECT_TRUE(mgr.buildAddressMap(configs, 2));

    EXPECT_EQ(mgr.totalRxPDOBytes(), 12u);  // 8 + 4
    EXPECT_EQ(mgr.totalTxPDOBytes(), 18u);  // 12 + 6
    EXPECT_EQ(mgr.totalLogicalSize(), 30u);

    // Slave 0 addresses
    EXPECT_EQ(mgr.getRxPDOLogicalAddr(0), 0x10000u);
    EXPECT_EQ(mgr.getRxPDOLength(0), 8u);
    EXPECT_EQ(mgr.getTxPDOLogicalAddr(0), 0x1000Cu);  // 0x10000 + 12
    EXPECT_EQ(mgr.getTxPDOLength(0), 12u);

    // Slave 1 addresses
    EXPECT_EQ(mgr.getRxPDOLogicalAddr(1), 0x10008u);
    EXPECT_EQ(mgr.getRxPDOLength(1), 4u);
    EXPECT_EQ(mgr.getTxPDOLogicalAddr(1), 0x10018u);  // 0x1000C + 12
    EXPECT_EQ(mgr.getTxPDOLength(1), 6u);
}

TEST_F(LogicalAddressManagerTest, BuildAddressMapEmptySlave) {
    SlaveConfig configs[kMaxPDOSlaves] = {};
    // Slave 0 has no PDOs
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 0);
    configs[0].rxpdo_size = 0;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 0);
    configs[0].txpdo_size = 0;

    EXPECT_TRUE(mgr.buildAddressMap(configs, 1));
    EXPECT_EQ(mgr.totalLogicalSize(), 0u);
    EXPECT_FALSE(mgr.hasSlavePDOs(0));
}

TEST_F(LogicalAddressManagerTest, BuildAddressMapRebuild) {
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 8);
    configs[0].rxpdo_size = 8;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 12);
    configs[0].txpdo_size = 12;

    EXPECT_TRUE(mgr.buildAddressMap(configs, 1));
    EXPECT_EQ(mgr.totalLogicalSize(), 20u);

    // Rebuild with different sizes
    configs[0].rxpdo_size = 16;
    configs[0].txpdo_size = 24;
    EXPECT_TRUE(mgr.buildAddressMap(configs, 1));
    EXPECT_EQ(mgr.totalLogicalSize(), 40u);
    EXPECT_EQ(mgr.getRxPDOLength(0), 16u);
    EXPECT_EQ(mgr.getTxPDOLength(0), 24u);
}

// ============================================================================
// LRW Exchange tests
// ============================================================================

class LRWExchangeTest : public ::testing::Test {
protected:
    NiceMock<MockPDOTransport> transport;
    LogicalAddressManager mgr{transport};
    PDOMapping mapping;

    void SetUp() override {
        mgr.init();

        // Configure one slave with 4-byte RxPDO and 8-byte TxPDO
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        mgr.buildAddressMap(configs, 1);

        // Add PDO entries to mapping
        rx_buf = 0xAABBCCDD;
        tx_buf = 0;
        mapping.add_rxpdo(0, &rx_buf, 4, 0x1600, PDOAddressMode::Logical);
        mapping.add_txpdo(0, &tx_buf, 8, 0x1A00, PDOAddressMode::Logical);
    }

    uint32_t rx_buf;
    uint64_t tx_buf;
};

TEST_F(LRWExchangeTest, ExchangeAllLRWSuccess) {
    // Capture the LRW payload and simulate response
    uint8_t captured_payload[64];
    uint16_t captured_len = 0;

    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 42, 0, 1, _, _, true))
        .WillOnce(Invoke([&](Command, uint8_t, uint16_t, uint16_t,
                              const void* data, uint16_t datalen, bool) -> bool {
            std::memcpy(captured_payload, data, datalen);
            captured_len = datalen;
            return true;
        }));
    EXPECT_CALL(transport, waitForResponseIdx(42, _, _))
        .WillOnce(Invoke([&](uint8_t, unsigned int, RxDatagram& out) -> bool {
            // Simulate response: first 4 bytes are RxPDO (echoed), next 8 are TxPDO from slave
            out.wkc = 1;
            out.datalen = 12;
            uint8_t resp_data[12] = {0xDD, 0xCC, 0xBB, 0xAA,  // RxPDO echo
                                      0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}; // TxPDO
            std::memcpy(out.data, resp_data, 12);
            return true;
        }));

    EXPECT_TRUE(mgr.exchangeAllLRW(mapping));

    // Verify payload: first 4 bytes = RxPDO data (little-endian)
    EXPECT_EQ(captured_len, 12u);
    EXPECT_EQ(captured_payload[0], 0xDD);
    EXPECT_EQ(captured_payload[1], 0xCC);
    EXPECT_EQ(captured_payload[2], 0xBB);
    EXPECT_EQ(captured_payload[3], 0xAA);

    // Verify TxPDO data was copied back
    uint8_t* tx_bytes = reinterpret_cast<uint8_t*>(&tx_buf);
    EXPECT_EQ(tx_bytes[0], 0x11);
    EXPECT_EQ(tx_bytes[1], 0x22);
    EXPECT_EQ(tx_bytes[2], 0x33);
    EXPECT_EQ(tx_bytes[3], 0x44);
    EXPECT_EQ(tx_bytes[4], 0x55);
    EXPECT_EQ(tx_bytes[5], 0x66);
    EXPECT_EQ(tx_bytes[6], 0x77);
    EXPECT_EQ(tx_bytes[7], 0x88);
}

TEST_F(LRWExchangeTest, ExchangeAllLRWWkcError) {
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(Invoke([](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 0;  // Working counter error
            return true;
        }));

    EXPECT_FALSE(mgr.exchangeAllLRW(mapping));

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.wkc_errors, 1u);
}

TEST_F(LRWExchangeTest, ExchangeAllLRWTimeout) {
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(Return(false));  // Timeout

    EXPECT_FALSE(mgr.exchangeAllLRW(mapping));

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.timeout_errors, 1u);
}

TEST_F(LRWExchangeTest, ExchangeAllLRWSendFail) {
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(false));  // Send failure

    EXPECT_FALSE(mgr.exchangeAllLRW(mapping));

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.send_errors, 1u);
}

TEST_F(LRWExchangeTest, ExchangeLRWForSlaves) {
    // Set up two slaves
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
    configs[0].rxpdo_size = 4;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
    configs[0].txpdo_size = 8;
    configs[1].configured = true;
    configs[1].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
    configs[1].rxpdo_size = 4;
    configs[1].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
    configs[1].txpdo_size = 8;
    mgr.buildAddressMap(configs, 2);

    PDOMapping multi_mapping;
    uint32_t rx0 = 0x11111111, rx1 = 0x22222222;
    uint64_t tx0 = 0, tx1 = 0;
    multi_mapping.add_rxpdo(0, &rx0, 4, 0x1600, PDOAddressMode::Logical);
    multi_mapping.add_txpdo(0, &tx0, 8, 0x1A00, PDOAddressMode::Logical);
    multi_mapping.add_rxpdo(1, &rx1, 4, 0x1600, PDOAddressMode::Logical);
    multi_mapping.add_txpdo(1, &tx1, 8, 0x1A00, PDOAddressMode::Logical);

    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 42, 0, 1, _, 12, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(Invoke([](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 12;
            std::memset(out.data, 0xAB, 12);
            return true;
        }));

    // Query only slave 1
    EXPECT_TRUE(mgr.exchangeLRWForSlaves(multi_mapping, 0x2));

    // tx0 should be unchanged (slave 0 not queried)
    EXPECT_EQ(tx0, 0u);
    // tx1 should have received data
    uint8_t* tx1_bytes = reinterpret_cast<uint8_t*>(&tx1);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(tx1_bytes[i], 0xAB);
    }
}

TEST_F(LRWExchangeTest, EmptyMappingReturnsTrue) {
    // Build map with zero-size PDOs
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 0);
    configs[0].rxpdo_size = 0;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 0);
    configs[0].txpdo_size = 0;
    mgr.buildAddressMap(configs, 1);

    EXPECT_TRUE(mgr.exchangeAllLRW(mapping));
}

// ============================================================================
// Stats tests
// ============================================================================

TEST_F(LogicalAddressManagerTest, StatsReset) {
    auto stats = mgr.getStats();
    EXPECT_EQ(stats.success, 0u);
    EXPECT_EQ(stats.wkc_errors, 0u);
    EXPECT_EQ(stats.send_errors, 0u);
    EXPECT_EQ(stats.timeout_errors, 0u);

    mgr.resetStats();
    stats = mgr.getStats();
    EXPECT_EQ(stats.success, 0u);
}

TEST_F(LogicalAddressManagerTest, OutOfRangeQueries) {
    EXPECT_EQ(mgr.getRxPDOLogicalAddr(99), 0u);
    EXPECT_EQ(mgr.getRxPDOLength(99), 0u);
    EXPECT_EQ(mgr.getTxPDOLogicalAddr(99), 0u);
    EXPECT_EQ(mgr.getTxPDOLength(99), 0u);
    EXPECT_FALSE(mgr.hasSlavePDOs(99));
}
