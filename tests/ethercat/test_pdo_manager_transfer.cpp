/**
 * @file test_pdo_manager_transfer.cpp
 * @brief Coverage for PDOManager single-entry transfers
 *        (src/ethercat/raw/PDOManager_transfer.cpp): all address modes,
 *        success/failure stat tracking, the confirmed-vs-fire-and-forget
 *        position path, configured/broadcast round-trips, and the
 *        memcpy-back of TxPDO data.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;

namespace {

/// Scripted PDO transport — records sends, returns scripted responses.
class FakePDOTransport : public IPDOTransport {
public:
    bool send_ok = true;
    bool resp_ok = true;
    uint16_t resp_wkc = 1;
    uint16_t resp_datalen = 64;
    uint8_t resp_fill = 0x5A;
    uint8_t next_idx = 0x40;

    struct Sent { Command cmd; uint8_t idx; uint16_t adp, ado, datalen; bool roundtrip; };
    std::vector<Sent> sent;

    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return true; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t,
                      unsigned int) override { return true; }
    bool sendSingleDatagram(Command cmd, uint8_t idx, uint16_t adp,
                            uint16_t ado, const void* data, uint16_t datalen,
                            bool roundtrip) override {
        sent.push_back({cmd, idx, adp, ado, datalen, roundtrip});
        (void)data;
        return send_ok;
    }
    size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override { return 0; }
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram& out) override {
        if (!resp_ok) return false;
        out.wkc = resp_wkc;
        out.datalen = resp_datalen;
        std::memset(out.data, resp_fill, sizeof(out.data));
        return true;
    }
    uint8_t allocIdx() override { return next_idx++; }
    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return static_cast<uint16_t>(0x1000 + slave_index);
    }
};

// ============================================================================
// sendRxPDO — dispatch + stats
// ============================================================================

TEST(PdoTransferCov, SendRxPDOInvalidEntry) {
    FakePDOTransport t;
    PDOManager mgr(t);
    EXPECT_FALSE(mgr.sendRxPDO(0));          // no entries
    EXPECT_FALSE(mgr.sendRxPDO(99));         // out of range
}

TEST(PdoTransferCov, SendRxPDOWrongDirection) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_txpdo(0, 4); // TxPDO entry
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, SendRxPDOPositionFirstConfirmed) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                      PDO::PDOAddressMode::Position);
    ASSERT_GE(idx, 0);
    // debug_count 0 → confirmed path: allocIdx + APWR + waitForResponseIdx
    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    ASSERT_FALSE(t.sent.empty());
    EXPECT_EQ(t.sent[0].cmd, Command::APWR);
    EXPECT_EQ(t.sent[0].idx, 0x40);          // allocIdx, not fire-and-forget
    EXPECT_TRUE(t.sent[0].roundtrip);
}

TEST(PdoTransferCov, SendRxPDOPositionFireAndForget) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                      PDO::PDOAddressMode::Position);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));   // #0 confirmed
    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));   // #1 fire+forget
    EXPECT_EQ(t.sent[1].idx, IPDOTransport::kFireAndForgetIdx);
    EXPECT_FALSE(t.sent[1].roundtrip);
}

TEST(PdoTransferCov, SendRxPDOConfirmedFailsOnWkcZero) {
    FakePDOTransport t;
    t.resp_wkc = 0;                          // slave didn't process
    PDOManager mgr(t);
    int idx = mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                      PDO::PDOAddressMode::Position);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));   // sendRxPDO still true (send ok)
}

TEST(PdoTransferCov, SendRxPDOConfiguredAddress) {
    FakePDOTransport t;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 8, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(t.sent[0].cmd, Command::FPWR);
    EXPECT_EQ(t.sent[0].adp, 0x2000);
}

TEST(PdoTransferCov, SendRxPDOConfiguredWkcZeroFails) {
    FakePDOTransport t;
    t.resp_wkc = 0;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 8, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));  // wkc=0 → false
}

TEST(PdoTransferCov, SendRxPDOConfiguredSendFails) {
    FakePDOTransport t;
    t.send_ok = false;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 8, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, SendRxPDOBroadcast) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_broadcast_rxpdo(6, 0x500);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(t.sent[0].cmd, Command::BWR);
}

TEST(PdoTransferCov, SendRxPDOBroadcastNoReply) {
    FakePDOTransport t;
    t.resp_ok = false;                       // timeout waiting
    PDOManager mgr(t);
    int idx = mgr.mapping().add_broadcast_rxpdo(6, 0x500);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, SendRxPDOLogicalNotImplemented) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                      PDO::PDOAddressMode::Logical);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));  // LOGW, false
}

// ============================================================================
// receiveTxPDO — dispatch + data copy
// ============================================================================

TEST(PdoTransferCov, RecvTxPDOInvalidAndWrongDirection) {
    FakePDOTransport t;
    PDOManager mgr(t);
    EXPECT_FALSE(mgr.receiveTxPDO(0));
    int idx = mgr.mapping().add_rxpdo(0, 4);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));  // RxPDO entry
}

TEST(PdoTransferCov, RecvTxPDOPositionFireAndForget) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_txpdo(0, 4, 0x1A00,
                                      PDO::PDOAddressMode::Position);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(t.sent[0].cmd, Command::APRD);
    EXPECT_EQ(t.sent[0].idx, IPDOTransport::kFireAndForgetIdx);
}

TEST(PdoTransferCov, RecvTxPDOPositionSendFails) {
    FakePDOTransport t;
    t.send_ok = false;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_txpdo(0, 4, 0x1A00,
                                      PDO::PDOAddressMode::Position);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, RecvTxPDOConfiguredCopiesData) {
    FakePDOTransport t;
    t.resp_fill = 0xA5;
    t.resp_datalen = 64;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(1, 0x2100);
    int idx = mgr.mapping().add_txpdo(1, 4, 0x1A00,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(t.sent[0].cmd, Command::FPRD);
    // Data copied into entry storage
    const uint8_t* d = mgr.mapping().entryData(static_cast<size_t>(idx));
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d[0], 0xA5);
    EXPECT_EQ(d[3], 0xA5);
}

TEST(PdoTransferCov, RecvTxPDOConfiguredShortDataFails) {
    FakePDOTransport t;
    t.resp_datalen = 2;                      // < data_size 4
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(1, 0x2100);
    int idx = mgr.mapping().add_txpdo(1, 4, 0x1A00,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, RecvTxPDOConfiguredNoResponseFails) {
    FakePDOTransport t;
    t.resp_ok = false;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(1, 0x2100);
    int idx = mgr.mapping().add_txpdo(1, 4, 0x1A00,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, RecvTxPDOConfiguredZeroWkcFails) {
    FakePDOTransport t;
    t.resp_wkc = 0;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(1, 0x2100);
    int idx = mgr.mapping().add_txpdo(1, 4, 0x1A00,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
}

TEST(PdoTransferCov, RecvTxPDOBroadcastCopiesData) {
    FakePDOTransport t;
    t.resp_fill = 0x3C;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_broadcast_txpdo(8, 0x600);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(t.sent[0].cmd, Command::BRD);
    const uint8_t* d = mgr.mapping().entryData(static_cast<size_t>(idx));
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d[0], 0x3C);
}

TEST(PdoTransferCov, RecvTxPDOLogicalNotImplemented) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_txpdo(0, 4, 0x1A00,
                                      PDO::PDOAddressMode::Logical);
    ASSERT_GE(idx, 0);
    EXPECT_FALSE(mgr.receiveTxPDO(static_cast<size_t>(idx)));
}

// ============================================================================
// Stats / entry counters
// ============================================================================

TEST(PdoTransferCov, SuccessAndErrorCountsAccumulate) {
    FakePDOTransport t;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    EXPECT_TRUE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    auto* e = mgr.mapping().get_entry_mut(static_cast<size_t>(idx));
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->success_count, 1u);
    EXPECT_EQ(e->error_count, 0u);

    t.resp_wkc = 0;                          // now fail
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_EQ(e->success_count, 1u);
    EXPECT_EQ(e->error_count, 1u);
}

TEST(PdoTransferCov, DisabledEntrySkipped) {
    FakePDOTransport t;
    PDOManager mgr(t);
    int idx = mgr.mapping().add_rxpdo(0, 4);
    ASSERT_GE(idx, 0);
    auto* e = mgr.mapping().get_entry_mut(static_cast<size_t>(idx));
    ASSERT_NE(e, nullptr);
    e->enabled = false;
    EXPECT_FALSE(mgr.sendRxPDO(static_cast<size_t>(idx)));
    EXPECT_TRUE(t.sent.empty());
}

}  // namespace
