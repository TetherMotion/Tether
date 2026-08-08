#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using ::testing::_;
using ::testing::Return;

// Minimal mock transport for PDOManager tests
class MockPDOTransportApi : public IPDOTransport {
public:
    MOCK_METHOD(bool, writeRegister, (uint16_t, uint16_t, const void*, uint16_t, unsigned int), (override));
    MOCK_METHOD(bool, readRegister, (uint16_t, uint16_t, void*, uint16_t, unsigned int), (override));
    MOCK_METHOD(bool, sendSingleDatagram, (Command, uint8_t, uint16_t, uint16_t, const void*, uint16_t, bool), (override));
    MOCK_METHOD(size_t, sendMultiDatagram, (const MultiDatagramSpec*, size_t), (override));
    MOCK_METHOD(bool, waitForResponseIdx, (uint8_t, unsigned int, RxDatagram&), (override));
    MOCK_METHOD(size_t, preRegisterResponseWaiter, (uint8_t, uint8_t*, size_t), (override));
    MOCK_METHOD(bool, waitForPreRegistered, (size_t, unsigned int, RxDatagram&), (override));
    MOCK_METHOD(uint8_t, allocIdx, (), (override));
    MOCK_METHOD(uint16_t, adpForSlaveIndex, (uint16_t), (override));
};
using NiceMockTransportApi = ::testing::NiceMock<MockPDOTransportApi>;

TEST(PDOApiTest, SendAndReceiveLogicalEntriesFailGracefullyAndUpdateStats) {
    NiceMockTransportApi transport;
    PDOManager mgr(transport);
    ASSERT_TRUE(mgr.init());
    mgr.resetStats();

    auto& mapping = mgr.mapping();
    mapping.clear();

    uint32_t out_buf = 0xAABBCCDD;
    uint32_t in_buf = 0;

    const int rx_idx = mapping.add_rxpdo(0, &out_buf, sizeof(out_buf), 0x1600, PDOAddressMode::Logical);
    const int tx_idx = mapping.add_txpdo(0, &in_buf, sizeof(in_buf), 0x1A00, PDOAddressMode::Logical);
    ASSERT_GE(rx_idx, 0);
    ASSERT_GE(tx_idx, 0);

    // Logical addressing is not implemented → should return false without crashing.
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(rx_idx)));
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(tx_idx)));

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.rxpdo_frames_sent, 0u);
    EXPECT_EQ(stats.txpdo_frames_recv, 0u);
    EXPECT_GE(stats.rxpdo_errors, 1u);
    EXPECT_GE(stats.txpdo_errors, 1u);

    // Exercise exchangeAll bookkeeping without invoking raw I/O.
    EXPECT_FALSE(mgr.exchangeAll());
    stats = mgr.getStats();
    EXPECT_EQ(stats.total_cycles, 1u);

    mgr.deinit();
}

TEST(PDOApiTest, InvalidEntryIndexOrDirectionReturnsFalse) {
    NiceMockTransportApi transport;
    PDOManager mgr(transport);
    ASSERT_TRUE(mgr.init());
    mgr.resetStats();

    auto& mapping = mgr.mapping();
    mapping.clear();

    // No entries yet.
    EXPECT_FALSE(mgr.sendRxPDO(0));
    EXPECT_FALSE(mgr.receiveTxPDO(0));

    // Add a TxPDO and ensure sendRxPDO rejects wrong direction.
    uint16_t buf = 0;
    const int tx_idx = mapping.add_txpdo(0, &buf, sizeof(buf), 0x1A00, PDOAddressMode::Logical);
    ASSERT_GE(tx_idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(tx_idx)));

    // Add an RxPDO and ensure receiveTxPDO rejects wrong direction.
    const int rx_idx = mapping.add_rxpdo(0, &buf, sizeof(buf), 0x1600, PDOAddressMode::Logical);
    ASSERT_GE(rx_idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(rx_idx)));

    mgr.deinit();
}
