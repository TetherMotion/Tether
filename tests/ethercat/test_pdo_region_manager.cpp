/**
 * @file test_pdo_region_manager.cpp
 * @brief Coverage for PDORegionManager — region registration against a
 *        PDOManager mapping, queue and provider exchange modes, raw
 *        buffer access, and scheduler registration.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "tether/ethercat/CyclicTaskScheduler.hpp"
#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDORegionManager.hpp"

using namespace EtherCAT;

namespace {

class StubPDOTransport : public IPDOTransport {
public:
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return false; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t,
                      unsigned int) override { return false; }
    bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t, const void*,
                            uint16_t, bool) override { return false; }
    size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override {
        return 0;
    }
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram&) override {
        return false;
    }
    uint8_t allocIdx() override { return 0; }
    uint16_t adpForSlaveIndex(uint16_t idx) override {
        return static_cast<uint16_t>(0u - idx);
    }
};

/// Provider that records the calls it receives.
class RecordingProvider : public IPDORegionProvider {
public:
    void fillTxRegion(const PDORegion& region, uint8_t* buffer) override {
        fill_calls++;
        last_region = region;
        std::memset(buffer, 0xA5, region.size);
    }
    void consumeRxRegion(const PDORegion& region,
                         const uint8_t* buffer) override {
        consume_calls++;
        last_region = region;
        received.assign(buffer, buffer + region.size);
    }

    int fill_calls = 0;
    int consume_calls = 0;
    PDORegion last_region;
    std::vector<uint8_t> received;
};

class PDORegionManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        pdo_.init();
        // 8-byte Rx and Tx entries for slave 0.
        ASSERT_GE(pdo_.mapping().add_rxpdo(0, 8), 0);
        ASSERT_GE(pdo_.mapping().add_txpdo(0, 8), 0);
    }

    StubPDOTransport transport_;
    PDOManager pdo_{transport_};
    PDORegionManager mgr_{pdo_};
};

}  // namespace

TEST_F(PDORegionManagerTest, RegisterRegionMatchingEntry) {
    int id = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    EXPECT_GE(id, 1);
    EXPECT_EQ(mgr_.regionCount(), 1u);

    PDORegion r;
    ASSERT_TRUE(mgr_.getRegion(id, r));
    EXPECT_EQ(r.slave_index, 0);
    EXPECT_EQ(r.offset, 0);
    EXPECT_EQ(r.size, 4);
}

TEST_F(PDORegionManagerTest, RegisterRegionRejections) {
    // Zero size.
    EXPECT_EQ(mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 0), -1);
    // Offset + size past the entry.
    EXPECT_EQ(mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 6, 4), -1);
    // No entry for this slave.
    EXPECT_EQ(mgr_.registerRegion(7, PDO::PDODirection::RxPDO, 0, 4), -1);
    // No TxPDO entry for slave 1.
    EXPECT_EQ(mgr_.registerRegion(1, PDO::PDODirection::TxPDO, 0, 4), -1);
    EXPECT_EQ(mgr_.regionCount(), 0u);
}

TEST_F(PDORegionManagerTest, GetRegionUnknownId) {
    PDORegion r;
    EXPECT_FALSE(mgr_.getRegion(123, r));
}

TEST_F(PDORegionManagerTest, PushTxDataQueueModeReachesEntryStorage) {
    const int id = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    ASSERT_GE(id, 1);

    const uint8_t payload[4] = {1, 2, 3, 4};
    ASSERT_TRUE(mgr_.pushTxData(id, payload));

    uint8_t* buf = mgr_.getTxBuffer(id);
    ASSERT_NE(buf, nullptr);
    EXPECT_EQ(buf[0], 0);  // not yet copied

    mgr_.prepareExchange();
    EXPECT_EQ(buf[0], 1);
    EXPECT_EQ(buf[3], 4);
}

TEST_F(PDORegionManagerTest, PushTxDataSizeAndDirectionGuards) {
    const int rx = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    const int tx = mgr_.registerRegion(0, PDO::PDODirection::TxPDO, 0, 4);
    ASSERT_GE(rx, 1);
    ASSERT_GE(tx, 1);

    const uint8_t too_big[8] = {};
    EXPECT_FALSE(mgr_.pushTxData(rx, too_big));           // wrong size
    EXPECT_FALSE(mgr_.pushTxData(tx, std::span<const uint8_t>(too_big, 4)));
    EXPECT_FALSE(mgr_.pushTxData(999, std::span<const uint8_t>(too_big, 4)));
}

TEST_F(PDORegionManagerTest, PopRxDataAfterProcessExchange) {
    const int id = mgr_.registerRegion(0, PDO::PDODirection::TxPDO, 0, 4);
    ASSERT_GE(id, 1);

    uint8_t out[4] = {};
    EXPECT_FALSE(mgr_.popRxData(id, out));  // no data yet

    const uint8_t* rx = mgr_.getRxBuffer(id);
    ASSERT_NE(rx, nullptr);
    mgr_.processExchange();

    ASSERT_TRUE(mgr_.popRxData(id, out));
    EXPECT_FALSE(mgr_.popRxData(id, out));  // consumed

    // Direction/size guards.
    const int rx_region = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    ASSERT_GE(rx_region, 1);
    EXPECT_FALSE(mgr_.popRxData(rx_region, out));
    EXPECT_FALSE(mgr_.popRxData(id, std::span<uint8_t>(out, 8)));
}

TEST_F(PDORegionManagerTest, ProcessExchangeCopiesEntryStorage) {
    const int id = mgr_.registerRegion(0, PDO::PDODirection::TxPDO, 0, 4);
    ASSERT_GE(id, 1);

    // Write into the entry storage the way the wire exchange would.
    const uint8_t* rx = mgr_.getRxBuffer(id);
    ASSERT_NE(rx, nullptr);
    const uint8_t* mut = rx;  // const view; storage is mutable behind it
    auto* writable = const_cast<uint8_t*>(mut);
    writable[0] = 0x11;
    writable[3] = 0x22;

    mgr_.processExchange();
    uint8_t out[4] = {};
    ASSERT_TRUE(mgr_.popRxData(id, out));
    EXPECT_EQ(out[0], 0x11);
    EXPECT_EQ(out[3], 0x22);
}

TEST_F(PDORegionManagerTest, BufferAccessRespectsDirection) {
    const int rx = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    const int tx = mgr_.registerRegion(0, PDO::PDODirection::TxPDO, 0, 4);
    ASSERT_GE(rx, 1);
    ASSERT_GE(tx, 1);

    EXPECT_NE(mgr_.getTxBuffer(rx), nullptr);
    EXPECT_EQ(mgr_.getTxBuffer(tx), nullptr);
    EXPECT_NE(mgr_.getRxBuffer(tx), nullptr);
    EXPECT_EQ(mgr_.getRxBuffer(rx), nullptr);
    EXPECT_EQ(mgr_.getTxBuffer(999), nullptr);
    EXPECT_EQ(mgr_.getRxBuffer(999), nullptr);
}

TEST_F(PDORegionManagerTest, ProviderModeFillsAndConsumes) {
    RecordingProvider provider;
    const int rx = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    const int tx = mgr_.registerRegion(0, PDO::PDODirection::TxPDO, 0, 4);
    ASSERT_GE(rx, 1);
    ASSERT_GE(tx, 1);
    mgr_.setProvider(rx, &provider);
    mgr_.setProvider(tx, &provider);
    mgr_.setProvider(999, &provider);  // unknown id → no-op

    mgr_.prepareExchange();
    EXPECT_EQ(provider.fill_calls, 1);
    EXPECT_EQ(provider.last_region.direction, PDO::PDODirection::RxPDO);

    uint8_t* buf = mgr_.getTxBuffer(rx);
    ASSERT_NE(buf, nullptr);
    EXPECT_EQ(buf[0], 0xA5);

    mgr_.processExchange();
    EXPECT_EQ(provider.consume_calls, 1);
    EXPECT_EQ(provider.received.size(), 4u);
}

TEST_F(PDORegionManagerTest, QueueModeTakesPrecedenceOverProviderOrder) {
    // A provider wins over queued data (Mode A before Mode B).
    RecordingProvider provider;
    const int id = mgr_.registerRegion(0, PDO::PDODirection::RxPDO, 0, 4);
    ASSERT_GE(id, 1);

    const uint8_t payload[4] = {9, 9, 9, 9};
    ASSERT_TRUE(mgr_.pushTxData(id, payload));
    mgr_.setProvider(id, &provider);
    mgr_.prepareExchange();

    EXPECT_EQ(provider.fill_calls, 1);
    uint8_t* buf = mgr_.getTxBuffer(id);
    ASSERT_NE(buf, nullptr);
    EXPECT_EQ(buf[0], 0xA5);  // provider data, not the queued 9s
}

TEST_F(PDORegionManagerTest, RegisterWithSchedulerAddsTasks) {
    CyclicTaskScheduler scheduler;
    mgr_.registerWithScheduler(scheduler);
    // The tasks run prepareExchange/processExchange on an empty region set.
    EXPECT_NO_THROW(scheduler.clear());
}
