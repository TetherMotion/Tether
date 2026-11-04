/**
 * @file test_logical_address_manager.cpp
 * @brief Unit tests for LogicalAddressManager
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <set>
#include <vector>
#include <cstring>

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/DebugGate.hpp"

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

    // Per-slave sticky windows: each slave gets [RxPDO][TxPDO] packed
    // contiguously — slave 0's window is [0x10000, 0x10014), slave 1's
    // starts right after it.
    EXPECT_EQ(mgr.getRxPDOLogicalAddr(0), 0x10000u);
    EXPECT_EQ(mgr.getRxPDOLength(0), 8u);
    EXPECT_EQ(mgr.getTxPDOLogicalAddr(0), 0x10008u);  // 0x10000 + 8
    EXPECT_EQ(mgr.getTxPDOLength(0), 12u);

    EXPECT_EQ(mgr.getRxPDOLogicalAddr(1), 0x10014u);  // 0x10000 + 20
    EXPECT_EQ(mgr.getRxPDOLength(1), 4u);
    EXPECT_EQ(mgr.getTxPDOLogicalAddr(1), 0x10018u);  // 0x10014 + 4
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

    // Rebuild with different sizes — the first window stays allocated
    // (sticky: its FMMU is already programmed), so the new 40-byte window
    // is appended at offset 20 and the image extent grows to 60.
    configs[0].rxpdo_size = 16;
    configs[0].txpdo_size = 24;
    EXPECT_TRUE(mgr.buildAddressMap(configs, 1));
    EXPECT_EQ(mgr.totalLogicalSize(), 60u);
    EXPECT_EQ(mgr.getRxPDOLength(0), 16u);
    EXPECT_EQ(mgr.getTxPDOLength(0), 24u);
    EXPECT_EQ(mgr.getRxPDOLogicalAddr(0), 0x10014u);  // appended, not repacked
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

        // Route the pre-registered wait path back through waitForResponseIdx
        // so existing expectations keep working while exercising the
        // pre-registration code path (pre-registered slot 0).
        ON_CALL(transport, preRegisterResponseWaiter(_, _, _))
            .WillByDefault(Invoke([this](uint8_t idx, uint8_t*, size_t) -> size_t {
                last_idx_ = idx;
                return 0;
            }));
        ON_CALL(transport, waitForPreRegistered(_, _, _))
            .WillByDefault(Invoke([this](size_t, unsigned int t, RxDatagram& out) -> bool {
                return transport.waitForResponseIdx(last_idx_, t, out);
            }));

        // Configure one slave with 4-byte RxPDO and 8-byte TxPDO
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        mgr.buildAddressMap(configs, 1);

        // Add PDO entries to mapping — bind views into entry storage.
        int rxi = mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        int txi = mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        rx_buf = mapping.entryDataAs<uint32_t>(static_cast<size_t>(rxi));
        tx_buf = mapping.entryDataAs<uint64_t>(static_cast<size_t>(txi));
        *rx_buf = 0xAABBCCDD;
        *tx_buf = 0;
    }

    uint32_t* rx_buf = nullptr;   ///< into entry storage
    uint64_t* tx_buf = nullptr;
    uint8_t last_idx_ = 0;
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
    uint8_t* tx_bytes = reinterpret_cast<uint8_t*>(tx_buf);
    EXPECT_EQ(tx_bytes[0], 0x11);
    EXPECT_EQ(tx_bytes[1], 0x22);
    EXPECT_EQ(tx_bytes[2], 0x33);
    EXPECT_EQ(tx_bytes[3], 0x44);
    EXPECT_EQ(tx_bytes[4], 0x55);
    EXPECT_EQ(tx_bytes[5], 0x66);
    EXPECT_EQ(tx_bytes[6], 0x77);
    EXPECT_EQ(tx_bytes[7], 0x88);
}

TEST_F(LRWExchangeTest, WaiterRegisteredBeforeSend) {
    // Regression: the response waiter must be pre-registered BEFORE the LRW
    // frame is sent.  A response that returns before registration is dropped
    // as "unrouted" and causes a spurious timeout.
    ::testing::Sequence seq;
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, preRegisterResponseWaiter(42, _, _))
        .InSequence(seq)
        .WillOnce(Return(0));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 42, 0, 1, _, _, true))
        .InSequence(seq)
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForPreRegistered(0, _, _))
        .WillOnce(Invoke([](size_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 12;
            return true;
        }));

    EXPECT_TRUE(mgr.exchangeAllLRW(mapping));
}

TEST_F(LRWExchangeTest, PreRegisterUnsupportedFallsBack) {
    // If the transport does not support pre-registration, the exchange must
    // fall back to waitForResponseIdx.
    EXPECT_CALL(transport, preRegisterResponseWaiter(_, _, _))
        .WillOnce(Return(IPDOTransport::kPreRegInvalid));
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(_, _, _, _, _, _, _))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(42, _, _))
        .WillOnce(Invoke([](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 12;
            return true;
        }));

    EXPECT_TRUE(mgr.exchangeAllLRW(mapping));
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
    int r0 = multi_mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
    int t0 = multi_mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
    int r1 = multi_mapping.add_rxpdo(1, 4, 0x1600, PDOAddressMode::Logical);
    int t1 = multi_mapping.add_txpdo(1, 8, 0x1A00, PDOAddressMode::Logical);
    auto& rx0 = *multi_mapping.entryDataAs<uint32_t>(static_cast<size_t>(r0));
    auto& rx1 = *multi_mapping.entryDataAs<uint32_t>(static_cast<size_t>(r1));
    auto& tx0 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t0));
    auto& tx1 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t1));
    rx0 = 0x11111111; rx1 = 0x22222222;
    tx0 = 0; tx1 = 0;

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

TEST_F(LRWExchangeTest, MultipleTxPDOEntriesSameSlave) {
    // Regression test: multiple TxPDO entries on the same slave must each
    // receive their own data from distinct offsets in the LRW response.
    // Previously, all entries for one slave used the same addr_map_ offset,
    // causing all modules on a slave to read identical data.

    // Reconfigure: slave 0 with 24-byte TxPDO (3 × 8-byte entries).
    // Fresh manager — sticky windows keep the fixture map's first window
    // allocated, so rebuilding `mgr` would append a second window instead
    // of landing this one at the image base.
    LogicalAddressManager fresh_mgr{transport};
    fresh_mgr.init();
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 0);
    configs[0].rxpdo_size = 0;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 24);
    configs[0].txpdo_size = 24;
    fresh_mgr.buildAddressMap(configs, 1);

    PDOMapping multi_mapping;
    int t0 = multi_mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
    int t1 = multi_mapping.add_txpdo(0, 8, 0x1A01, PDOAddressMode::Logical);
    int t2 = multi_mapping.add_txpdo(0, 8, 0x1A02, PDOAddressMode::Logical);
    auto& tx0 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t0));
    auto& tx1 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t1));
    auto& tx2 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t2));

    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 42, 0, 1, _, 24, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(Invoke([](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 24;
            // Three distinct 8-byte blocks
            uint8_t resp[24] = {
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // entry 0
                0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, // entry 1
                0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28  // entry 2
            };
            std::memcpy(out.data, resp, 24);
            return true;
        }));

    EXPECT_TRUE(fresh_mgr.exchangeAllLRW(multi_mapping));

    // Each entry should have received its own distinct data
    uint8_t* b0 = reinterpret_cast<uint8_t*>(&tx0);
    uint8_t* b1 = reinterpret_cast<uint8_t*>(&tx1);
    uint8_t* b2 = reinterpret_cast<uint8_t*>(&tx2);

    EXPECT_EQ(b0[0], 0x01);
    EXPECT_EQ(b0[7], 0x08);
    EXPECT_EQ(b1[0], 0x11);
    EXPECT_EQ(b1[7], 0x18);
    EXPECT_EQ(b2[0], 0x21);
    EXPECT_EQ(b2[7], 0x28);

    // Ensure entries are not all identical (the bug)
    EXPECT_NE(tx0, tx1);
    EXPECT_NE(tx1, tx2);
    EXPECT_NE(tx0, tx2);
}

TEST_F(LRWExchangeTest, ResizedSlaveWindowExtendsExchangeSpan) {
    // Sticky windows: reconfiguring slave 0 to a larger window appends a
    // fresh window at the end (the old window stays allocated — its FMMU
    // is already programmed).  The exchange must span the image EXTENT
    // (next_free_log_), not just the sum of live PDO bytes — otherwise
    // entries in the appended window would silently never be exchanged.
    //
    // Fixture map: slave 0 Rx[0,4) Tx[4,12) → extent 12.  Rebuild with
    // rxpdo=0 txpdo=24 → new window [12,36), extent grows to 36.
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 0);
    configs[0].rxpdo_size = 0;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 24);
    configs[0].txpdo_size = 24;
    ASSERT_TRUE(mgr.buildAddressMap(configs, 1));
    EXPECT_EQ(mgr.totalLogicalSize(), 36u);   // 12 dead + 24 live

    PDOMapping multi_mapping;
    int t0 = multi_mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
    int t1 = multi_mapping.add_txpdo(0, 8, 0x1A01, PDOAddressMode::Logical);
    int t2 = multi_mapping.add_txpdo(0, 8, 0x1A02, PDOAddressMode::Logical);
    auto& tx0 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t0));
    auto& tx1 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t1));
    auto& tx2 = *multi_mapping.entryDataAs<uint64_t>(static_cast<size_t>(t2));

    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(42));
    // Extent, not live-byte sum: the LRW must cover all 36 bytes.
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 42, 0, 1, _, 36, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(_, _, _))
        .WillOnce(Invoke([](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 36;
            // Entry blocks sit at image offsets 12, 20, 28.
            uint8_t resp[36] = {};
            for (int i = 0; i < 8; ++i) {
                resp[12 + i] = static_cast<uint8_t>(0x01 + i);  // entry 0
                resp[20 + i] = static_cast<uint8_t>(0x11 + i);  // entry 1
                resp[28 + i] = static_cast<uint8_t>(0x21 + i);  // entry 2
            }
            std::memcpy(out.data, resp, 36);
            return true;
        }));

    EXPECT_TRUE(mgr.exchangeAllLRW(multi_mapping));

    const uint8_t* b0 = reinterpret_cast<const uint8_t*>(&tx0);
    const uint8_t* b1 = reinterpret_cast<const uint8_t*>(&tx1);
    const uint8_t* b2 = reinterpret_cast<const uint8_t*>(&tx2);
    EXPECT_EQ(b0[0], 0x01);
    EXPECT_EQ(b0[7], 0x08);
    EXPECT_EQ(b1[0], 0x11);
    EXPECT_EQ(b1[7], 0x18);
    EXPECT_EQ(b2[0], 0x21);
    EXPECT_EQ(b2[7], 0x28);
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
// Partial LRW slice tests
// ============================================================================

TEST_F(LRWExchangeTest, MaxSliceLengthAccountsForDatagramOverhead) {
    // Mock transport default frame payload is 1498 → 1498 - 12 = 1486.
    EXPECT_EQ(mgr.maxSliceLength(), 1486u);
}

TEST_F(LRWExchangeTest, DescribeEntries) {
    auto entries = mgr.describeEntries(mapping);
    ASSERT_EQ(entries.size(), 2u);
    EXPECT_EQ(entries[0].pdo_index, 0x1600u);
    EXPECT_EQ(entries[0].direction, PDODirection::RxPDO);
    EXPECT_EQ(entries[0].offset, 0u);
    EXPECT_EQ(entries[0].length, 4u);
    EXPECT_EQ(entries[1].pdo_index, 0x1A00u);
    EXPECT_EQ(entries[1].direction, PDODirection::TxPDO);
    EXPECT_EQ(entries[1].offset, 4u);
    EXPECT_EQ(entries[1].length, 8u);
}

TEST_F(LRWExchangeTest, ExchangeLRWSliceTxOnly) {
    uint8_t captured[64];
    uint16_t captured_len = 0;

    // Slice [4,12) is the TxPDO region: logical address base+4.
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(7));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 7, 0x0004, 0x0001, _, 8, true))
        .WillOnce(Invoke([&](Command, uint8_t, uint16_t, uint16_t,
                              const void* data, uint16_t datalen, bool) -> bool {
            std::memcpy(captured, data, datalen);
            captured_len = datalen;
            return true;
        }));
    EXPECT_CALL(transport, waitForResponseIdx(7, _, _))
        .WillOnce(Invoke([&](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 8;
            uint8_t resp[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
            std::memcpy(out.data, resp, 8);
            return true;
        }));

    EXPECT_TRUE(mgr.exchangeLRWSlice(mapping, 4, 8));

    EXPECT_EQ(captured_len, 8u);
    // RxPDO entry lies outside the slice and must be untouched.
    EXPECT_EQ(*rx_buf, 0xAABBCCDDu);
    uint8_t* tb = reinterpret_cast<uint8_t*>(tx_buf);
    EXPECT_EQ(tb[0], 0x11);
    EXPECT_EQ(tb[7], 0x88);
}

TEST_F(LRWExchangeTest, ExchangeLRWSliceRxOnly) {
    uint8_t captured[64];
    uint16_t captured_len = 0;

    // Slice [0,4) is the RxPDO region.
    EXPECT_CALL(transport, allocIdx()).WillOnce(Return(9));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 9, 0x0000, 0x0001, _, 4, true))
        .WillOnce(Invoke([&](Command, uint8_t, uint16_t, uint16_t,
                              const void* data, uint16_t datalen, bool) -> bool {
            std::memcpy(captured, data, datalen);
            captured_len = datalen;
            return true;
        }));
    EXPECT_CALL(transport, waitForResponseIdx(9, _, _))
        .WillOnce(Invoke([&](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1;
            out.datalen = 4;
            std::memset(out.data, 0, 4);
            return true;
        }));

    EXPECT_TRUE(mgr.exchangeLRWSlice(mapping, 0, 4));

    EXPECT_EQ(captured_len, 4u);
    EXPECT_EQ(captured[0], 0xDD);  // rx_buf 0xAABBCCDD little-endian
    // TxPDO entry lies outside the slice and must be untouched.
    EXPECT_EQ(*tx_buf, 0u);
}

TEST_F(LRWExchangeTest, ExchangeLRWSliceRejectsTooLarge) {
    EXPECT_FALSE(mgr.exchangeLRWSlice(mapping, 0, mgr.maxSliceLength() + 1));
    EXPECT_EQ(mgr.getStats().send_errors, 1u);
}

TEST_F(LRWExchangeTest, ExchangeLRWSliceOutOfRange) {
    EXPECT_FALSE(mgr.exchangeLRWSlice(mapping, 8, 8));  // total image is 12
    EXPECT_EQ(mgr.getStats().send_errors, 1u);
}

TEST_F(LRWExchangeTest, ExchangeLRWSliceComposesWholeImage) {
    // Two slices covering the whole 12-byte image, exchanged separately.
    EXPECT_CALL(transport, allocIdx())
        .WillOnce(Return(1))
        .WillOnce(Return(2));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 1, 0x0000, 0x0001, _, 4, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, sendSingleDatagram(Command::LRW, 2, 0x0004, 0x0001, _, 8, true))
        .WillOnce(Return(true));
    EXPECT_CALL(transport, waitForResponseIdx(1, _, _))
        .WillOnce(Invoke([&](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1; out.datalen = 4; std::memset(out.data, 0, 4); return true;
        }));
    EXPECT_CALL(transport, waitForResponseIdx(2, _, _))
        .WillOnce(Invoke([&](uint8_t, unsigned int, RxDatagram& out) -> bool {
            out.wkc = 1; out.datalen = 8; std::memset(out.data, 0xCD, 8); return true;
        }));

    EXPECT_TRUE(mgr.exchangeLRWSlice(mapping, 0, 4));
    EXPECT_TRUE(mgr.exchangeLRWSlice(mapping, 4, 8));

    uint8_t* tb = reinterpret_cast<uint8_t*>(tx_buf);
    for (int i = 0; i < 8; i++) EXPECT_EQ(tb[i], 0xCD);
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

// ============================================================================
// Cyclic split-path tests — Q3 mask wait, Q6/Q7 WKC derive/sentinel/reset
// ============================================================================

/// Hand-rolled fast-path transport: records slice sends, serves scripted
/// slot responses through the rotating-pool wait.  Response scripts are
/// keyed by SLICE (logical offset order), not wire position — the LAM
/// rotates pool positions every send, so the stub maps each LRW
/// datagram's logical address back to its slice index:
///   slice = ((ado<<16)|adp - base) / max_slice_len
/// The LRD counter datagram lands above the image → echoed verbatim
/// (slaves pass unmapped logical bytes through).  The APRD DC read
/// serves the dedicated dc_resp_* script.
class CyclicStubTransport : public IPDOTransport {
public:
    // ---- fast-path surface ----
    bool supportsCyclicFastPath() const override {
        return fast_path_;
    }
    size_t maxEtherCATPayloadPerFrame() const override { return payload_; }
    uint64_t cyclicSlotToken(uint8_t slot) override {
        return slot <= kCyclicDcPoolPos ? tokens_[slot] : 0;
    }
    uint8_t cyclicSlotGen(uint8_t slot) override {
        return slot <= kCyclicDcPoolPos ? gens_[slot] : 0;
    }
    bool sendCyclicDatagram(Command cmd, uint8_t pos, uint16_t adp,
                            uint16_t ado, const void* data,
                            uint16_t datalen, bool) override {
        ++send_calls_total_;
        if (cmd == Command::LRW) ++send_calls;
        recordSent(pos, cmd, adp, ado, data, datalen);
        return send_ok_;
    }
    bool sendPoolFrame(const CyclicDgramSpec* dgs,
                       size_t count) override {
        if (!pool_frames_ok_) return false;
        ++pool_frame_calls;
        last_frame_dgrams_ = count;
        last_frame_cmds_.clear();
        last_frame_idx_.clear();
        for (size_t i = 0; i < count; ++i) {
            const uint8_t pos = cyclicWirePoolPos(dgs[i].idx);
            last_frame_cmds_.push_back(dgs[i].cmd);
            last_frame_idx_.push_back(dgs[i].idx);
            if (dgs[i].cmd == Command::LRW) ++send_calls;
            recordSent(pos, dgs[i].cmd, dgs[i].adp, dgs[i].ado,
                       dgs[i].data,
                       static_cast<uint16_t>(dgs[i].datalen +
                                             dgs[i].tail_len));
        }
        return send_ok_;
    }
    uint32_t waitCyclicSlotMask(uint32_t mask, const uint64_t*,
                                uint32_t, CyclicSlotView* views) override {
        ++mask_calls;
        uint32_t arrived = 0;
        for (uint8_t s = 0; s < 32; ++s) {
            if (!(mask & (1u << s))) continue;
            CyclicSlotView v{};
            bool ar = false;
            serve(s, v, ar);
            if (!ar) continue;
            views[s] = v;
            arrived |= 1u << s;
        }
        return arrived;
    }
    uint8_t waitCyclicPool(const uint8_t* positions, const uint64_t*,
                           uint8_t count, uint32_t,
                           CyclicSlotView* views, bool* arrived) override {
        ++mask_calls;
        uint8_t n = 0;
        for (uint8_t i = 0; i < count; ++i) {
            serve(positions[i], views[i], arrived[i]);
            if (arrived[i]) ++n;
        }
        return n;
    }

private:
    void recordSent(uint8_t pos, Command cmd, uint16_t adp, uint16_t ado,
                    const void* data, uint16_t datalen) {
        if (pos > kCyclicDcPoolPos) return;
        tokens_[pos]++;
        gens_[pos] ^= 1;
        sent_cmd_[pos] = cmd;
        sent_off_[pos] =
            (static_cast<uint32_t>(ado) << 16 | adp) - base_;
        sent_len_[pos] = datalen;
        if (data && datalen)
            std::memcpy(sent_buf_[pos], data,
                        std::min<size_t>(datalen, sizeof(sent_buf_[0])));
    }

    /// Resolve a sent LRW datagram's slice index from its logical offset
    /// (the LAM sends slice s at s*max_slice; the counter LRD lands
    /// above the image so it never collides with a scripted slice).
    uint32_t sliceOf(uint8_t pos) const {
        const uint32_t msl = payload_ >= 12 ? payload_ - 12 : 1;
        return sent_off_[pos] / msl;
    }

    void serve(uint8_t pos, CyclicSlotView& v, bool& arrived) {
        arrived = false;
        if (pos > kCyclicDcPoolPos || sent_cmd_[pos] == Command::NOP)
            return;
        const Command cmd = sent_cmd_[pos];
        v.gen = gens_[pos];
        v.stamp_ns = stamp_ns_;
        if (cmd == Command::LRD) {          // counter trailer — echo back
            if (!cnt_respond_) return;
            v.payload = cnt_corrupt_ ? corrupt_buf_ : sent_buf_[pos];
            v.datalen = sent_len_[pos];
            v.wkc     = 0;                  // unmapped — no slave counts
        } else if (cmd == Command::APRD) {  // DC System Time read
            if (!dc_respond_) return;
            v.payload = dc_resp_buf_;
            v.datalen = 8;
            v.wkc     = dc_resp_wkc_;
        } else {                            // LRW image slice
            const uint32_t s = sliceOf(pos);
            if (s >= kMaxScripted || !respond_[s]) return;
            v.payload = resp_buf_[s];
            v.datalen = resp_len_[s];
            v.wkc     = resp_wkc_[s];
            if (stale_[s]) {                // echo previous generation
                v.gen ^= 1;
                stale_[s]--;
            }
        }
        arrived = true;
    }

public:
    // ---- unused surface ----
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return false; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t,
                      unsigned int) override { return false; }
    bool sendSingleDatagram(Command cmd, uint8_t idx, uint16_t, uint16_t,
                            const void*, uint16_t len, bool) override {
        ++single_send_calls;
        last_single_cmd_ = cmd;
        last_single_idx_ = idx;
        last_single_len_ = len;
        return single_send_ok_;
    }
    size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override {
        return 0;
    }
    bool waitForResponseIdx(uint8_t, unsigned int,
                            RxDatagram& out) override {
        if (!single_respond_) return false;
        out.wkc     = single_resp_wkc_;
        out.datalen = single_resp_len_;
        std::memcpy(out.data, single_resp_buf_, single_resp_len_);
        return true;
    }
    size_t preRegisterResponseWaiter(uint8_t, uint8_t*,
                                     size_t) override {
        return IPDOTransport::kPreRegInvalid;   // fall back to waitForResponseIdx
    }
    bool waitForPreRegistered(size_t, unsigned int,
                              RxDatagram&) override { return false; }
    uint8_t  allocIdx() override { return 0x30; }
    uint16_t adpForSlaveIndex(uint16_t) override { return 0; }

    // ---- PDO-slice fast-path surface (idx pool 0xE0..0xEF) ----
    // Shared-slot pool: lowest-free-bit claim, mirroring CyclicDatapath.
    int acquireSliceSlot() override {
        for (uint8_t s = 0; s < 16; ++s) {
            if (!(slice_claimed_ & (1u << s))) {
                slice_claimed_ = static_cast<uint16_t>(
                    slice_claimed_ | (1u << s));
                return s;
            }
        }
        return -1;
    }
    void releaseSliceSlot(uint8_t slice) override {
        if (slice < 16)
            slice_claimed_ = static_cast<uint16_t>(
                slice_claimed_ & ~(1u << slice));
    }
    uint64_t sliceSlotToken(uint8_t slice) override {
        return slice < 16 ? slice_tokens_[slice] : 0;
    }
    uint8_t sliceSlotGen(uint8_t slice) override {
        return slice < 16 ? slice_gen_[slice] : 0;
    }
    bool sendSliceDatagram(Command, uint8_t slice, uint16_t adp,
                           uint16_t ado, const void* data,
                           uint16_t datalen, bool) override {
        ++slice_send_calls;
        slice_tokens_[slice]++;              // deposits bump the token
        slice_gen_[slice] ^= 1;              // send gen toggles
        last_slice_slot_ = slice;
        last_slice_adp_  = adp;
        last_slice_ado_  = ado;
        if (data && datalen)
            std::memcpy(slice_sent_[slice], data,
                        std::min<size_t>(datalen, sizeof(slice_sent_[0])));
        slice_sent_len_[slice] = datalen;
        return send_ok_;
    }
    uint32_t waitSliceSlotMask(uint32_t mask, const uint64_t*,
                               uint32_t, CyclicSlotView* views) override {
        ++slice_mask_calls;
        uint32_t arrived = 0;
        for (uint8_t s = 0; s < 16; ++s) {
            if (!(mask & (1u << s)) || !slice_respond_[s]) continue;
            views[s].payload = slice_resp_buf_[s];
            views[s].datalen = slice_resp_len_[s];
            views[s].wkc     = slice_resp_wkc_[s];
            // Echo the send gen; a scripted stale response echoes the
            // previous generation once then recovers.
            views[s].gen = slice_stale_[s]
                ? static_cast<uint8_t>(slice_gen_[s] ^ 1)
                : slice_gen_[s];
            slice_stale_[s] = false;
            views[s].stamp_ns = stamp_ns_;
            arrived |= 1u << s;
        }
        return arrived;
    }

    // ---- script ----
    static constexpr size_t kMaxScripted = 16;
    size_t   payload_ = 1498;
    uint32_t base_ = 0x10000;               // LAM default logical base
    uint64_t tokens_[kCyclicDcPoolPos + 1]{};
    uint8_t  gens_[kCyclicDcPoolPos + 1]{};
    Command  sent_cmd_[kCyclicDcPoolPos + 1]{};
    uint32_t sent_off_[kCyclicDcPoolPos + 1]{};
    uint16_t sent_len_[kCyclicDcPoolPos + 1]{};
    uint8_t  sent_buf_[kCyclicDcPoolPos + 1][64]{};
    bool     respond_[kMaxScripted];
    int      stale_[kMaxScripted]{};        // stale echoes remaining
    bool     fast_path_ = true;
    uint64_t stamp_ns_ = 0;                 // scripted deposit stamp
    uint8_t  resp_buf_[kMaxScripted][64]{};
    uint16_t resp_len_[kMaxScripted]{};
    uint16_t resp_wkc_[kMaxScripted]{};
    bool     send_ok_ = true;
    int      send_calls = 0;        ///< image LRW datagrams only
    int      send_calls_total_ = 0; ///< every datagram incl. counter/DC
    int      mask_calls = 0;
    // counter-trailer + DC-timepoint scripts
    bool     cnt_respond_ = true;
    bool     cnt_corrupt_ = false;  ///< echo wrong bytes (stale frame)
    uint8_t  corrupt_buf_[8] = {0xDE, 0xAD, 0xBE, 0xEF,
                              0xDE, 0xAD, 0xBE, 0xEF};
    bool     dc_respond_ = true;
    uint8_t  dc_resp_buf_[8]{};
    uint16_t dc_resp_wkc_ = 1;
    bool     pool_frames_ok_ = true;
    int      pool_frame_calls = 0;
    size_t   last_frame_dgrams_ = 0;
    std::vector<Command> last_frame_cmds_;
    std::vector<uint8_t> last_frame_idx_;

    CyclicStubTransport() {
        std::fill(std::begin(respond_), std::end(respond_), true);
        std::fill(std::begin(sent_cmd_), std::end(sent_cmd_),
                  Command::NOP);
    }

    // slice script
    uint64_t slice_tokens_[16]{};
    uint8_t  slice_gen_[16]{};
    bool     slice_respond_[16] = {true, true, true, true, true, true,
                                   true, true, true, true, true, true,
                                   true, true, true, true};
    bool     slice_stale_[16]{};
    uint8_t  slice_resp_buf_[16][64]{};
    uint16_t slice_resp_len_[16]{};
    uint16_t slice_resp_wkc_[16]{};
    uint8_t  slice_sent_[16][64]{};
    uint16_t slice_sent_len_[16]{};
    uint16_t slice_claimed_ = 0;
    uint8_t  last_slice_slot_ = 0xFF;
    uint16_t last_slice_adp_ = 0, last_slice_ado_ = 0;
    int      slice_send_calls = 0;
    int      slice_mask_calls = 0;

    // async-path script (exchangePDOSlice / exchangeLRWSlice)
    bool     single_send_ok_ = true;
    bool     single_respond_ = true;
    Command  last_single_cmd_ = Command::NOP;
    uint8_t  last_single_idx_ = 0;
    uint16_t last_single_len_ = 0;
    uint16_t single_resp_wkc_ = 1;
    uint16_t single_resp_len_ = 0;
    uint8_t  single_resp_buf_[64]{};
    int      single_send_calls = 0;
};

class CyclicWkcTest : public ::testing::Test {
protected:
    CyclicStubTransport transport;
    LogicalAddressManager mgr{transport};
    PDOMapping mapping;

    /// Two slaves: s0 = Rx4B+Tx8B (wkc +3/slice), s1 = Rx4B only (+1).
    void buildTwoSlaveMap() {
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        configs[1].configured = true;
        configs[1].sm[2] = SyncManagerConfig::process_output(0x1801, 4);
        configs[1].rxpdo_size = 4;
        configs[1].sm[3] = SyncManagerConfig::process_input(0x1C01, 0);
        configs[1].txpdo_size = 0;
        ASSERT_TRUE(mgr.buildAddressMap(configs, 2));
        mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        mapping.add_rxpdo(1, 4, 0x1601, PDOAddressMode::Logical);
    }

    void SetUp() override { mgr.init(); }
};

TEST_F(CyclicWkcTest, DeriveWkcCountsSlavesPerDirection) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    // Single slice (16 B < 1486): s0 writes Rx +2, reads Tx +1; s1 writes
    // Rx +2 → expected WKC = 2 + 1 + 2 = 5.
    EXPECT_EQ(mgr.expectedWkc(0), 5u);
    EXPECT_EQ(mgr.expectedWkc(1), LogicalAddressManager::kWkcUnknown);
}

TEST_F(CyclicWkcTest, StrictWkcMismatchFailsCollect) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 2;   // expected 5
    transport.resp_len_[0] = 16;
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().wkc_errors, 1u);
}

TEST_F(CyclicWkcTest, CorrectWkcPassesCollect) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 5;
    transport.resp_len_[0] = 16;
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(transport.mask_calls, 1);   // single-wake mask wait used
    EXPECT_EQ(mgr.getStats().wkc_errors, 0u);
}

TEST_F(CyclicWkcTest, SentinelLearnsFromFirstResponse) {
    buildTwoSlaveMap();
    mgr.resetExpectedWkc();                    // all slices → learn mode
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    // cyclicSend re-derives — force learn mode again post-send to prove
    // the sentinel path in collect.
    mgr.resetExpectedWkc();
    ASSERT_EQ(mgr.expectedWkc(0), LogicalAddressManager::kWkcUnknown);
    transport.resp_wkc_[0] = 4;
    transport.resp_len_[0] = 16;
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.expectedWkc(0), 4u);    // learned
    // Next cycle with a different WKC now fails strict.
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 9;
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
}

TEST_F(CyclicWkcTest, ResetReturnsToLearnMode) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    ASSERT_EQ(mgr.expectedWkc(0), 5u);
    mgr.resetExpectedWkc();
    EXPECT_EQ(mgr.expectedWkc(0), LogicalAddressManager::kWkcUnknown);
    mgr.setExpectedWkc(0, 7);
    EXPECT_EQ(mgr.expectedWkc(0), 7u);
    mgr.resetExpectedWkc();
    EXPECT_EQ(mgr.expectedWkc(0), LogicalAddressManager::kWkcUnknown);
}

TEST_F(CyclicWkcTest, MultiSliceDerivationPerSlice) {
    // Force 2 slices: payload 18 → max_slice = 6; image is 16 B → 3 slices
    // actually ([0,6) [6,12) [12,16)) — compute expectations per slice.
    transport.payload_ = 18;   // maxSliceLength = 18 - 12 = 6
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(mgr.cyclicSliceCount(), 3u);
    // Per-slave sticky layout: s0 Rx [0,4), s0 Tx [4,12), s1 Rx [12,16).
    // slice0 [0,6):  s0 Rx + s0 Tx ∩ → 2 + 1 = 3
    EXPECT_EQ(mgr.expectedWkc(0), 3u);
    // slice1 [6,12): s0 Tx [4,12) ∩ only → 1
    EXPECT_EQ(mgr.expectedWkc(1), 1u);
    // slice2 [12,16): s1 Rx only → 2
    EXPECT_EQ(mgr.expectedWkc(2), 2u);
    EXPECT_EQ(transport.send_calls, 3);   // one LRW per slice
}

TEST_F(CyclicWkcTest, PartialMaskTimeoutCountsPerSlot) {
    transport.payload_ = 18;
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.respond_[2] = false;         // slice 2 never arrives
    transport.resp_wkc_[0] = 2;
    transport.resp_wkc_[1] = 3;
    transport.resp_len_[0] = 6;
    transport.resp_len_[1] = 6;
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().timeout_errors, 1u);
}

// ============================================================================
// Cyclic pool — rotating positions, 64-bit counter trailer, DC timepoint
// ============================================================================

static_assert(kNumFastSlots == 100);
static_assert(kNumCyclicSlots == 82);          // rotating pool (excl. DC pos)
static_assert(kCyclicDcPoolPos == 82);
static_assert(cyclicPoolWireIdx(0) == 0x9C);
static_assert(cyclicPoolWireIdx(67) == 0xDF);
static_assert(cyclicPoolWireIdx(68) == 0xF0);  // wraps past slice band 0xE0..EF
static_assert(cyclicPoolWireIdx(81) == 0xFD);  // 0xFE fire-and-forget skipped
static_assert(cyclicPoolWireIdx(kCyclicDcPoolPos) == kDcTimeIdx);   // 0xFF

class CyclicPoolTrailerTest : public ::testing::Test {
protected:
    CyclicStubTransport transport;
    LogicalAddressManager mgr{transport};
    PDOMapping mapping;

    void SetUp() override {
        mgr.init();
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 8);
        configs[0].rxpdo_size = 8;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        ASSERT_TRUE(mgr.buildAddressMap(configs, 1));
        mapping.add_rxpdo(0, 8, 0x1600, PDOAddressMode::Logical);
        mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        transport.resp_len_[0] = 16;
        transport.resp_wkc_[0] = 3;
    }
};

/// The last frame of a cycle is one multi-datagram frame carrying the
/// image LRW, the LRD counter trailer, and — when configured — the APRD
/// DC System Time read, each on its own dedicated pool idx.
TEST_F(CyclicPoolTrailerTest, OneFrameCarriesLrwCounterAndDc) {
    mgr.setCyclicDcTimeSlave(0);
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    ASSERT_EQ(transport.pool_frame_calls, 1);
    ASSERT_EQ(transport.last_frame_dgrams_, 3u);
    EXPECT_EQ(transport.last_frame_cmds_[0], Command::LRW);
    EXPECT_EQ(transport.last_frame_cmds_[1], Command::LRD);
    EXPECT_EQ(transport.last_frame_cmds_[2], Command::APRD);
    EXPECT_EQ(transport.last_frame_idx_[2], kDcTimeIdx);

    const uint64_t dc_expect = 0x1122334455667788ull;
    std::memcpy(transport.dc_resp_buf_, &dc_expect, 8);
    ASSERT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.lastLrwCounter(), 1u);
    EXPECT_TRUE(mgr.dcTimeValid());
    EXPECT_EQ(mgr.lastDcTimeNs(), dc_expect);
}

/// Without a configured DC slave the trailer frame is just LRW + counter.
TEST_F(CyclicPoolTrailerTest, DcDisabledByDefaultLeavesItOut) {
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    ASSERT_EQ(transport.pool_frame_calls, 1);
    ASSERT_EQ(transport.last_frame_dgrams_, 2u);
    EXPECT_EQ(transport.last_frame_cmds_[0], Command::LRW);
    EXPECT_EQ(transport.last_frame_cmds_[1], Command::LRD);
    ASSERT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_FALSE(mgr.dcTimeValid());
}

/// An echoed counter that doesn't match what was sent is a stale echo —
/// the exchange fails and the counter is NOT published.
TEST_F(CyclicPoolTrailerTest, CorruptCounterEchoFailsExchange) {
    transport.cnt_corrupt_ = true;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().counter_mismatches, 1u);
    EXPECT_EQ(mgr.lastLrwCounter(), 0u);
}

/// No counter response at all is likewise a stale/missed trailer.
TEST_F(CyclicPoolTrailerTest, MissingCounterEchoFailsExchange) {
    transport.cnt_respond_ = false;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().counter_mismatches, 1u);
}

/// The verified counter is exposed to consumers and monotonically
/// advances once per successful exchange.
TEST_F(CyclicPoolTrailerTest, CounterAdvancesEveryCycle) {
    for (int i = 1; i <= 3; ++i) {
        ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
        ASSERT_TRUE(mgr.cyclicCollect(mapping, nullptr));
        EXPECT_EQ(mgr.lastLrwCounter(), static_cast<uint64_t>(i));
    }
}

/// A missing DC response is informational only — the PDO data is still
/// valid and the verified counter is still published.
TEST_F(CyclicPoolTrailerTest, DcTimeoutDoesNotFailExchange) {
    mgr.setCyclicDcTimeSlave(0);
    transport.dc_respond_ = false;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_FALSE(mgr.dcTimeValid());
    EXPECT_EQ(mgr.getStats().dc_timeouts, 1u);
    EXPECT_EQ(mgr.lastLrwCounter(), 1u);
}

/// Each cycle draws fresh pool positions — an in-flight backlog of up
/// to kNumCyclicSlots requests is supported before any wire idx is
/// reused, and only the position being sent is re-armed.
TEST_F(CyclicPoolTrailerTest, PositionsRotateAcrossCycles) {
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.sent_cmd_[0], Command::LRW);
    EXPECT_EQ(transport.sent_cmd_[1], Command::LRD);
    ASSERT_TRUE(mgr.cyclicCollect(mapping, nullptr));

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.sent_cmd_[2], Command::LRW);
    EXPECT_EQ(transport.sent_cmd_[3], Command::LRD);
    // The cycle-1 positions were not re-armed by the second send.
    EXPECT_EQ(transport.sent_cmd_[0], Command::LRW);
    EXPECT_EQ(transport.sent_cmd_[1], Command::LRD);
}

// ============================================================================
// PDO slices — user-declared image subsets on dedicated wire idx 0xE0..0xEF
// ============================================================================

class PdoSliceTest : public ::testing::Test {
protected:
    static constexpr uint32_t kInvalid = 0xFFFFFFFFu;
    CyclicStubTransport transport;
    LogicalAddressManager mgr{transport};
    PDOMapping mapping;

    /// Same two-slave config as CyclicWkcTest — per-slave sticky layout:
    /// s0 Rx[0,4) Tx[4,12), s1 Rx[12,16) — 16 B image.
    /// describeEntries order: 0=s0 Rx, 1=s0 Tx, 2=s1 Rx.
    void buildMap() {
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        configs[1].configured = true;
        configs[1].sm[2] = SyncManagerConfig::process_output(0x1801, 4);
        configs[1].rxpdo_size = 4;
        configs[1].sm[3] = SyncManagerConfig::process_input(0x1C01, 0);
        configs[1].txpdo_size = 0;
        ASSERT_TRUE(mgr.buildAddressMap(configs, 2));
        mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        mapping.add_rxpdo(1, 4, 0x1601, PDOAddressMode::Logical);
    }

    void SetUp() override {
        mgr.init();
        buildMap();
    }

    /// Burn the cycle-0 image exchange — `cycle % every_n == 0` always
    /// fires on the first cycle, so tests wanting "only slice traffic"
    /// run this first (image response scripted for a clean collect).
    void burnImageCycle() {
        transport.resp_len_[0] = 16;
        transport.resp_wkc_[0] = 5;
        mgr.cyclicSend(mapping, nullptr, 1'000'000);
        mgr.cyclicCollect(mapping, nullptr);
        transport.send_calls = 0;
        transport.mask_calls = 0;
    }
};

TEST_F(PdoSliceTest, EntrySpecSendsOnDedicatedSliceSlot) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};                    // s1 Rx → run [4,8)
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    EXPECT_EQ(mgr.pdoSliceCount(), 1u);
    mgr.setImageExchangeDecimation(1000);  // image exchange out of the way

    // Gathered payload must carry the entry's app-side output bytes.
    std::memset(mapping.get_entry(2)->storage, 0x5A, 4);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;      // derived: s1 Rx write → +2

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.slice_send_calls, 1);
    EXPECT_EQ(transport.last_slice_slot_, 0);   // wire idx 0xE0
    EXPECT_EQ(transport.send_calls, 0);         // no image datagrams
    EXPECT_EQ(transport.slice_sent_len_[0], 4u);
    EXPECT_EQ(transport.slice_sent_[0][0], 0x5A);
    // Only the slice is in flight — pending still reports it.
    EXPECT_TRUE(mgr.cyclicExchangePending());

    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_FALSE(mgr.cyclicExchangePending());
    EXPECT_EQ(transport.slice_mask_calls, 1);   // single-wake mask wait
    EXPECT_EQ(mgr.getStats().wkc_errors, 0u);
}

TEST_F(PdoSliceTest, AdjacentEntriesMergeIntoOneRun) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {0, 1};                 // [0,4)+[4,12) → one run [0,12)
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 12;
    transport.slice_resp_wkc_[0] = 3;      // s0 Rx write +2, s0 Tx read +1

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.slice_send_calls, 1);   // merged — one datagram
    EXPECT_EQ(transport.slice_sent_len_[0], 12u);
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().wkc_errors, 0u);
}

TEST_F(PdoSliceTest, DisjointEntriesConsumeOneSlotEach) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {0, 2};                 // [0,4) and [12,16) → 2 runs
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;      // s0 RxPDO write → +2
    transport.slice_resp_len_[1] = 4;
    transport.slice_resp_wkc_[1] = 2;      // s1 RxPDO write → +2

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.slice_send_calls, 2);   // slots 0 and 1
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().wkc_errors, 0u);
}

TEST_F(PdoSliceTest, EveryNDecimatesSliceExchange) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    spec.every_n = 3;
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;

    for (int c = 0; c < 6; ++c) {
        ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
        mgr.cyclicCollect(mapping, nullptr);
    }
    // cycle_mod: skip, skip, fire — 2 emits in 6 cycles.
    EXPECT_EQ(transport.slice_send_calls, 2);
}

TEST_F(PdoSliceTest, OnExchangeCallbackFiresPerRun) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {0, 2};                 // two disjoint runs
    int calls = 0;
    uint8_t  last_run = 0xFF;
    uint16_t last_len = 0, last_wkc = 0;
    spec.on_exchange = [&](uint8_t r, const uint8_t*, uint16_t len,
                           uint16_t w) {
        ++calls; last_run = r; last_len = len; last_wkc = w;
    };
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;  transport.slice_resp_wkc_[0] = 2;
    transport.slice_resp_len_[1] = 4;  transport.slice_resp_wkc_[1] = 2;

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(last_run, 1);
    EXPECT_EQ(last_len, 4u);
    EXPECT_EQ(last_wkc, 2u);
}

TEST_F(PdoSliceTest, StaleGenDroppedThenRetrySucceeds) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;
    transport.slice_stale_[0] = true;      // first view echoes prev gen

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().stale_responses, 1u);
    EXPECT_EQ(transport.slice_mask_calls, 2);   // one retry wait
    EXPECT_EQ(mgr.getStats().timeout_errors, 0u);
}

TEST_F(PdoSliceTest, MissingSliceResponseCountsTimeout) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_respond_[0] = false;   // deposit never arrives

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().timeout_errors, 1u);
}

TEST_F(PdoSliceTest, WkcMismatchCountsError) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 9;      // derived expectation is 2

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().wkc_errors, 1u);
}

TEST_F(PdoSliceTest, SliceResponseScattersToEntryStorage) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {1};                    // s0 Tx [8,16)
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 8;
    transport.slice_resp_wkc_[0] = 1;      // TxPDO read → +1
    std::memset(transport.slice_resp_buf_[0], 0xCD, 8);

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    const auto* e1 = mapping.get_entry(1);
    for (int i = 0; i < 8; ++i) EXPECT_EQ(e1->storage[i], 0xCD);
}

TEST_F(PdoSliceTest, RejectsEmptySpecAndOutOfImageRange) {
    PDOSliceSpec empty;
    EXPECT_EQ(mgr.definePDOSlice(mapping, empty), kInvalid);

    PDOSliceSpec r;
    r.ranges = {{8, 9}};                   // [8,17) exceeds the 16 B image
    EXPECT_EQ(mgr.definePDOSlice(mapping, r), kInvalid);
    r.ranges = {{8, 8}};                   // exactly to the end — ok
    EXPECT_NE(mgr.definePDOSlice(mapping, r), kInvalid);
}

TEST_F(PdoSliceTest, RejectsRunExceedingOneDatagram) {
    transport.payload_ = 18;               // maxSliceLength() = 6
    PDOSliceSpec r;
    r.ranges = {{0, 8}};
    EXPECT_EQ(mgr.definePDOSlice(mapping, r), kInvalid);
    r.ranges = {{0, 6}};
    EXPECT_NE(mgr.definePDOSlice(mapping, r), kInvalid);
}

TEST_F(PdoSliceTest, SliceSlotPoolExhaustionFailsDefine) {
    // A 24 B single-slave image → 16 one-byte ranges fit the pool,
    // the 17th define must fail.
    LogicalAddressManager mgr2{transport};
    mgr2.init();
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 24);
    configs[0].rxpdo_size = 24;
    ASSERT_TRUE(mgr2.buildAddressMap(configs, 1));
    PDOMapping m2;
    m2.add_rxpdo(0, 24, 0x1600, PDOAddressMode::Logical);

    for (uint32_t i = 0; i < 16; ++i) {
        PDOSliceSpec s;
        s.ranges = {{i, 1}};
        ASSERT_NE(mgr2.definePDOSlice(m2, s), kInvalid) << "define " << i;
    }
    EXPECT_EQ(mgr2.pdoSliceCount(), 16u);
    PDOSliceSpec one_too_many;
    one_too_many.ranges = {{16, 1}};
    EXPECT_EQ(mgr2.definePDOSlice(m2, one_too_many), kInvalid);
    EXPECT_EQ(mgr2.pdoSliceCount(), 16u);
}

TEST_F(PdoSliceTest, MappingEpochChangeReplans) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};                    // s1 Rx
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    mgr.cyclicCollect(mapping, nullptr);
    EXPECT_EQ(transport.slice_send_calls, 1);

    // Removing slave 1's entries bumps the mapping epoch — the next
    // send re-resolves the spec (zero runs → slice goes dormant, no
    // stale-offset datagram on the wire).
    mapping.remove_entries_for_slave(1);
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    mgr.cyclicCollect(mapping, nullptr);
    EXPECT_EQ(transport.slice_send_calls, 1);   // no new send
}

TEST_F(PdoSliceTest, ClearDropsSlicesAndPending) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_respond_[0] = false;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicExchangePending());

    EXPECT_TRUE(mgr.clearPDOSlices());
    EXPECT_EQ(mgr.pdoSliceCount(), 0u);
    EXPECT_FALSE(mgr.cyclicExchangePending());
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));   // nothing pending
}

TEST_F(PdoSliceTest, OneOffExchangeUsesAsyncPath) {
    PDOSliceSpec spec;
    spec.entries = {2};                    // resolves to [4,8)
    transport.single_resp_len_ = 4;
    transport.single_resp_wkc_ = 1;

    EXPECT_TRUE(mgr.exchangePDOSlice(mapping, spec));
    // One classic single-datagram round trip — no slice slots consumed,
    // nothing defined for the cyclic loop.
    EXPECT_EQ(transport.single_send_calls, 1);
    EXPECT_EQ(transport.last_single_cmd_, Command::LRW);
    EXPECT_EQ(transport.last_single_len_, 4u);
    EXPECT_EQ(transport.slice_send_calls, 0);
    EXPECT_EQ(mgr.pdoSliceCount(), 0u);
}

TEST_F(PdoSliceTest, SliceOverlayBeatsDecimatedImageData) {
    // Image runs every 3rd cycle; the slice covers the same bytes every
    // cycle — collect order must leave the slice's newer data on top.
    PDOSliceSpec spec;
    spec.ranges = {{0, 4}};                // overlaps s0 Rx [0,4)
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(3);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;

    // Cycle 0: image + slice both emit; both collect.
    transport.resp_len_[0] = 16;
    transport.resp_wkc_[0] = 5;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.send_calls, 1);        // image slice on slot 0
    EXPECT_EQ(transport.slice_send_calls, 1);  // slice on idx 0xE0
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    // Cycles 1,2: only the slice emits.
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(transport.send_calls, 1);
    EXPECT_EQ(transport.slice_send_calls, 2);
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
}

// ============================================================================
// CyclicSliceHealth — per-slice protocol liveness (wire loss vs WKC vs stale)
// ============================================================================

TEST_F(CyclicWkcTest, SliceHealthReportsOkWithWkc) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 5;
    transport.resp_len_[0] = 16;
    ASSERT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    const auto h = mgr.sliceHealth(0);
    EXPECT_EQ(h.last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(h.expected_wkc, 5u);
    EXPECT_EQ(h.last_wkc, 5u);
    EXPECT_EQ(h.consecutive_failures, 0u);
}

TEST_F(CyclicWkcTest, SliceHealthTimeoutTracksStreak) {
    buildTwoSlaveMap();
    transport.respond_[0] = false;         // datagram never circulates back
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.sliceHealth(0).last_status, CyclicSliceStatus::Timeout);
    EXPECT_EQ(mgr.sliceHealth(0).consecutive_failures, 1u);

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.sliceHealth(0).consecutive_failures, 2u);

    transport.respond_[0] = true;
    transport.resp_wkc_[0] = 5;
    transport.resp_len_[0] = 16;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.sliceHealth(0).last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(mgr.sliceHealth(0).consecutive_failures, 0u);
}

TEST_F(CyclicWkcTest, SliceHealthWkcErrorKeepsLastWkc) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 2;            // expected 5 — slave dropped work
    transport.resp_len_[0] = 16;
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    const auto h = mgr.sliceHealth(0);
    EXPECT_EQ(h.last_status, CyclicSliceStatus::WkcError);
    EXPECT_EQ(h.expected_wkc, 5u);
    EXPECT_EQ(h.last_wkc, 2u);             // "arrived but short" is visible
    EXPECT_EQ(h.consecutive_failures, 1u);
}

TEST_F(PdoSliceTest, SliceRunHealthReportsTimeoutThenRecovery) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_respond_[0] = false;

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    auto ph = mgr.pdoSliceHealth();
    ASSERT_EQ(ph.size(), 1u);
    EXPECT_EQ(ph[0].last_status, CyclicSliceStatus::Timeout);
    EXPECT_EQ(ph[0].consecutive_failures, 1u);

    transport.slice_respond_[0] = true;
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 2;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    ph = mgr.pdoSliceHealth();
    ASSERT_EQ(ph.size(), 1u);
    EXPECT_EQ(ph[0].last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(ph[0].expected_wkc, 2u);
    EXPECT_EQ(ph[0].last_wkc, 2u);
    EXPECT_EQ(ph[0].consecutive_failures, 0u);
}

// ============================================================================
// PDOManager cyclic/​slice wrapper coverage
// ============================================================================

namespace {

class PDOManagerCyclicTest : public ::testing::Test {
protected:
    static constexpr uint32_t kInvalid = 0xFFFFFFFFu;
    CyclicStubTransport transport;
    PDOManager pdo_mgr{transport};
    LogicalAddressManager lam{transport};

    void SetUp() override {
        lam.init();
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        ASSERT_TRUE(lam.buildAddressMap(configs, 1));
        std::memcpy(pdo_mgr.slaveConfigs(), configs, sizeof(configs));
        pdo_mgr.mapping().add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        pdo_mgr.mapping().add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        pdo_mgr.setLogicalAddressManager(&lam);
        transport.resp_len_[0] = 12;
        transport.resp_wkc_[0] = 3;
        transport.single_resp_wkc_ = 3;
        transport.single_resp_len_ = 12;
    }
};

}  // namespace

TEST_F(PDOManagerCyclicTest, ExchangeLRWSliceRequiresLAM) {
    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    EXPECT_FALSE(lone.exchangeLRWSlice(0, 4));
}

TEST_F(PDOManagerCyclicTest, ExchangeLRWSliceDelegatesAndCountsEntries) {
    EXPECT_TRUE(pdo_mgr.exchangeLRWSlice(0, 4));
    // Slice [0,4) covers only the RxPDO entry → request counter only.
    EXPECT_GE(pdo_mgr.getSlavePDORequestCount(0), 1u);
    EXPECT_EQ(pdo_mgr.getSlavePDOReplyCount(0), 0u);
}

TEST_F(PDOManagerCyclicTest, ExchangeConfiguredSlaveWindowsDelegates) {
    EXPECT_TRUE(pdo_mgr.exchangeConfiguredSlaveWindows());
    EXPECT_GE(pdo_mgr.getSlavePDORequestCount(0), 1u);
    EXPECT_GE(pdo_mgr.getSlavePDOReplyCount(0), 1u);
}

TEST_F(PDOManagerCyclicTest, ExchangeConfiguredSlaveWindowsFallsBackWithoutLAM) {
    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    // No LAM: falls back to exchangeAll(), which is a no-op without a
    // configured process-image window.
    EXPECT_TRUE(lone.exchangeConfiguredSlaveWindows());
}

TEST_F(PDOManagerCyclicTest, ExchangeAllLRWCyclicDelegatesAndCounts) {
    EXPECT_TRUE(pdo_mgr.exchangeAllLRWCyclic(1'000'000, nullptr));
    EXPECT_GE(pdo_mgr.getSlavePDORequestCount(0), 1u);
    EXPECT_GE(pdo_mgr.getSlavePDOReplyCount(0), 1u);
}

TEST_F(PDOManagerCyclicTest, ExchangeAllLRWCyclicFallsBackWithoutLAM) {
    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    EXPECT_TRUE(lone.exchangeAllLRWCyclic(1'000'000, nullptr));
}

TEST_F(PDOManagerCyclicTest, CyclicSendCollectAndPending) {
    EXPECT_TRUE(pdo_mgr.cyclicSend(nullptr, 1'000'000));
    EXPECT_TRUE(pdo_mgr.cyclicExchangePending());
    EXPECT_TRUE(pdo_mgr.cyclicCollect(nullptr));
    EXPECT_FALSE(pdo_mgr.cyclicExchangePending());
    EXPECT_GE(pdo_mgr.getSlavePDORequestCount(0), 1u);
    EXPECT_GE(pdo_mgr.getSlavePDOReplyCount(0), 1u);
}

TEST_F(PDOManagerCyclicTest, CyclicSendCollectWithoutLAM) {
    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    EXPECT_TRUE(lone.cyclicSend(nullptr, 1'000'000));
    EXPECT_FALSE(lone.cyclicExchangePending());
    EXPECT_TRUE(lone.cyclicCollect(nullptr));
}

TEST_F(PDOManagerCyclicTest, PDOSliceLifecycle) {
    EXPECT_EQ(pdo_mgr.pdoSliceCount(), 0u);
    PDOSliceSpec spec;
    spec.entries = {0};
    ASSERT_NE(pdo_mgr.definePDOSlice(spec), kInvalid);
    EXPECT_EQ(pdo_mgr.pdoSliceCount(), 1u);
    EXPECT_TRUE(pdo_mgr.exchangePDOSlice(spec));
    EXPECT_TRUE(pdo_mgr.clearPDOSlices());
    EXPECT_EQ(pdo_mgr.pdoSliceCount(), 0u);
}

TEST_F(PDOManagerCyclicTest, PDOSliceCallsFailWithoutLAM) {
    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    PDOSliceSpec spec;
    spec.entries = {0};
    EXPECT_EQ(lone.definePDOSlice(spec), kInvalid);
    EXPECT_FALSE(lone.exchangePDOSlice(spec));
    EXPECT_FALSE(lone.clearPDOSlices());
    EXPECT_EQ(lone.pdoSliceCount(), 0u);
}

TEST_F(PDOManagerCyclicTest, CyclicKnobsAndAccessors) {
    pdo_mgr.setImageExchangeDecimation(4);
    pdo_mgr.setCyclicStrictWkc(true);
    EXPECT_GT(pdo_mgr.maxLogicalSliceLength(), 0u);
    EXPECT_EQ(pdo_mgr.describeLogicalEntries().size(), 2u);

    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    lone.setImageExchangeDecimation(4);
    lone.setCyclicStrictWkc(false);
    EXPECT_EQ(lone.maxLogicalSliceLength(), 0u);
    EXPECT_TRUE(lone.describeLogicalEntries().empty());
}

TEST_F(PDOManagerCyclicTest, ConfigureProcessImageRequiresLAM) {
    CyclicStubTransport lone_transport;
    PDOManager lone{lone_transport};
    ProcessImage image;
    EXPECT_FALSE(lone.configureProcessImage(image, ImageMode::Buffered, nullptr));
}

// ============================================================================
// PDOManager queue mode (Mode 2) and callback mode (Mode 3)
// ============================================================================

TEST_F(PDOManagerCyclicTest, QueueCycleRequiresQueueMode) {
    EXPECT_FALSE(pdo_mgr.queueCycle());
}

TEST_F(PDOManagerCyclicTest, QueueCyclePushesRxAndUnderrunEvents) {
    QueueModeConfig cfg;
    pdo_mgr.configureQueueMode(cfg);

    int underruns = 0;
    pdo_mgr.setUnderrunCallback([&](uint16_t, uint32_t) { ++underruns; });

    EXPECT_TRUE(pdo_mgr.queueCycle());
    EXPECT_EQ(underruns, 1);   // empty TX queue → underrun fired once

    // One RxReceived + one Underrun event on the event queue.
    std::shared_ptr<PDOEvent> ev;
    std::vector<PDOEvent::Type> seen;
    while (pdo_mgr.tryPollEvent(ev)) seen.push_back(ev->type);
    EXPECT_TRUE(std::find(seen.begin(), seen.end(),
                          PDOEvent::Type::RxReceived) != seen.end());
    EXPECT_TRUE(std::find(seen.begin(), seen.end(),
                          PDOEvent::Type::Underrun) != seen.end());

    // The TxPDO entry's storage was pushed to the RX queue.
    std::shared_ptr<PDOFrame> frame;
    ASSERT_TRUE(pdo_mgr.tryDequeueRx(1, frame));
    EXPECT_EQ(frame->data.size(), 8u);
}

TEST_F(PDOManagerCyclicTest, QueueCycleDequeuesTxIntoStorage) {
    QueueModeConfig cfg;
    pdo_mgr.configureQueueMode(cfg);

    auto frame = std::make_shared<PDOFrame>();
    frame->data = {0xDE, 0xAD, 0xBE, 0xEF};
    ASSERT_TRUE(pdo_mgr.enqueueTx(0, frame));

    EXPECT_TRUE(pdo_mgr.queueCycle());
    const auto* e = pdo_mgr.mapping().get_entry(0);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(std::memcmp(e->storage, "\xDE\xAD\xBE\xEF", 4), 0);

    // Second cycle with an empty queue: RepeatLastFrame keeps the data.
    EXPECT_TRUE(pdo_mgr.queueCycle());
    EXPECT_EQ(std::memcmp(e->storage, "\xDE\xAD\xBE\xEF", 4), 0);
}

TEST_F(PDOManagerCyclicTest, QueueUnderrunPolicies) {
    QueueModeConfig cfg;
    cfg.safe_state_buffer = {0xAA, 0xBB, 0xCC, 0xDD};

    // SafeState
    cfg.underrun_policy = UnderrunPolicy::SafeState;
    pdo_mgr.configureQueueMode(cfg);
    EXPECT_TRUE(pdo_mgr.queueCycle());
    const auto* e = pdo_mgr.mapping().get_entry(0);
    EXPECT_EQ(std::memcmp(e->storage, "\xAA\xBB\xCC\xDD", 4), 0);

    // SkipCycle disables then re-enables the entry for the cycle
    cfg.underrun_policy = UnderrunPolicy::SkipCycle;
    pdo_mgr.configureQueueMode(cfg);
    EXPECT_TRUE(pdo_mgr.queueCycle());
    EXPECT_TRUE(pdo_mgr.mapping().get_entry(0)->enabled);

    // Custom — user callback decides; storage untouched
    cfg.underrun_policy = UnderrunPolicy::Custom;
    pdo_mgr.configureQueueMode(cfg);
    pdo_mgr.setUnderrunCallback([](uint16_t, uint32_t) {});
    e->storage[0] = 0x55;
    EXPECT_TRUE(pdo_mgr.queueCycle());
    EXPECT_EQ(e->storage[0], 0x55);
}

TEST_F(PDOManagerCyclicTest, EnqueueDequeueBounds) {
    QueueModeConfig cfg;
    cfg.tx_queue_capacity = 1;
    pdo_mgr.configureQueueMode(cfg);

    auto f1 = std::make_shared<PDOFrame>();
    f1->data = {1, 2, 3, 4};
    EXPECT_TRUE(pdo_mgr.enqueueTx(0, f1));

    std::shared_ptr<PDOFrame> frame;
    EXPECT_FALSE(pdo_mgr.tryDequeueRx(0, frame));   // nothing received yet
    std::shared_ptr<PDOEvent> ev;
    EXPECT_FALSE(pdo_mgr.tryPollEvent(ev));
}

TEST_F(PDOManagerCyclicTest, CallbackModeConfigures) {
    CallbackModeConfig cfg;
    pdo_mgr.configureCallbackMode(cfg);
    pdo_mgr.setTxSentCallback(0, [](uint16_t, uint32_t, uint64_t) {});
    pdo_mgr.setRxReceivedCallback(1, [](uint16_t, const uint8_t*, size_t,
                                       uint32_t, uint64_t) {});
    // Out-of-range indices are ignored.
    pdo_mgr.setTxSentCallback(PDO::kMaxPDOEntries + 5,
                              [](uint16_t, uint32_t, uint64_t) {});
    pdo_mgr.setRxReceivedCallback(PDO::kMaxPDOEntries + 5,
                                  [](uint16_t, const uint8_t*, size_t,
                                     uint32_t, uint64_t) {});
}

// ============================================================================
// PDOManager SM configuration coverage
// ============================================================================

namespace {

class SMConfigStubTransport : public CyclicStubTransport {
public:
    struct RegWrite {
        uint16_t adp;
        uint16_t ado;
        uint16_t len;
        uint8_t  data[8];
    };
    struct RegRead {
        uint16_t adp;
        uint16_t ado;
    };

    bool writeRegister(uint16_t adp, uint16_t ado, const void* data,
                       uint16_t len, unsigned int) override {
        RegWrite w{adp, ado, len, {}};
        if (data && len)
            std::memcpy(w.data, data,
                        std::min<size_t>(len, sizeof(w.data)));
        writes.push_back(w);
        return fail_writes.count(writes.size()) == 0;
    }
    bool readRegister(uint16_t adp, uint16_t ado, void* data,
                      uint16_t len, unsigned int) override {
        reads.push_back({adp, ado});
        if (data && len) std::memset(data, read_byte_, len);
        return read_ok_;
    }

    // Batched SM-activation surface (configureSlavesSMs pre-registers one
    // waiter per SM then sends a single multi-datagram frame).
    size_t preRegisterResponseWaiter(uint8_t, uint8_t*,
                                     size_t) override {
        if (prereg_busy_)
            return IPDOTransport::kPreRegBusy;
        if (waited_slots.size() == prereg_fail_at ||
            !prereg_ok_)
            return IPDOTransport::kPreRegInvalid;
        waited_slots.push_back(next_slot_);
        return next_slot_++;
    }
    size_t sendMultiDatagram(const MultiDatagramSpec* specs,
                             size_t count) override {
        if (!multi_send_ok_) return 0;
        last_multi_adp.clear();
        last_multi_ado.clear();
        last_multi_data.clear();
        last_multi_payloads.clear();
        for (size_t i = 0; i < count; ++i) {
            last_multi_adp.push_back(specs[i].adp);
            last_multi_ado.push_back(specs[i].ado);
            last_multi_data.push_back(
                specs[i].data
                    ? *static_cast<const uint8_t*>(specs[i].data)
                    : 0xFF);
            if (specs[i].data && specs[i].datalen)
                last_multi_payloads.emplace_back(
                    static_cast<const uint8_t*>(specs[i].data),
                    static_cast<const uint8_t*>(specs[i].data) +
                        specs[i].datalen);
            else
                last_multi_payloads.emplace_back();
        }
        return count;
    }
    bool waitForPreRegistered(size_t slot, unsigned int,
                              RxDatagram& out) override {
        confirmed_slots.push_back(slot);
        if (wait_prereg_ok_) {
            const size_t call = prereg_wait_calls_++;
            const uint16_t wkc =
                (call < prereg_wkc_plan_.size()) ? prereg_wkc_plan_[call]
                                                : prereg_wkc_;
            out.wkc     = wkc;
            out.datalen = prereg_resp_len_;
            if (prereg_resp_len_)
                std::memcpy(out.data, prereg_resp_buf_, prereg_resp_len_);
        }
        return wait_prereg_ok_;
    }
    void cancelPreRegistered(size_t slot) override {
        cancelled_slots.push_back(slot);
    }
    uint16_t adpForSlaveIndex(uint16_t i) override {
        return static_cast<uint16_t>(0x1000 + i);
    }

    std::vector<RegWrite> writes;
    std::vector<RegRead>  reads;
    std::set<size_t>      fail_writes;      // 1-based call indices to fail
    bool                  read_ok_  = true;
    uint8_t               read_byte_ = 0;

    bool   prereg_ok_      = false;         // support the batched path?
    bool   prereg_busy_    = false;         // every claim returns busy
    bool   multi_send_ok_  = true;
    bool   wait_prereg_ok_ = true;
    size_t prereg_fail_at  = SIZE_MAX;      // slot index that fails prereg
    size_t next_slot_      = 0;
    std::vector<size_t>   waited_slots;
    std::vector<size_t>   confirmed_slots;
    std::vector<size_t>   cancelled_slots;
    size_t                prereg_wait_calls_ = 0;
    uint16_t              prereg_wkc_ = 1;
    std::vector<uint16_t> prereg_wkc_plan_;
    uint16_t              prereg_resp_len_ = 0;
    uint8_t               prereg_resp_buf_[512]{};
    std::vector<uint16_t> last_multi_adp;
    std::vector<uint16_t> last_multi_ado;
    std::vector<uint8_t>  last_multi_data;
    std::vector<std::vector<uint8_t>> last_multi_payloads;
};

class PDOManagerSMConfigTest : public ::testing::Test {
protected:
    static constexpr uint16_t kAdp = 0x1000;
    SMConfigStubTransport transport;
    PDOManager pdo_mgr{transport};

    void SetUp() override {
        auto* cfgs = pdo_mgr.slaveConfigs();
        cfgs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        cfgs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        cfgs[0].rxpdo_size = 4;
        cfgs[0].txpdo_size = 8;
    }
};

}  // namespace

TEST_F(PDOManagerSMConfigTest, ConfigureSlavesSMsRejectsInvalidIndex) {
    EXPECT_FALSE(pdo_mgr.configureSlavesSMs(
        static_cast<uint16_t>(PDO::kMaxPDOSlaves)));
    EXPECT_TRUE(transport.writes.empty());
}

TEST_F(PDOManagerSMConfigTest, ProgramsAndActivatesSequentially) {
    // No pre-registered waiters -> sequential activate fallback.
    ASSERT_TRUE(pdo_mgr.configureSlavesSMs(0));
    ASSERT_EQ(transport.writes.size(), 6u);

    // SM2 (base 0x0810, process output): disable then block.
    EXPECT_EQ(transport.writes[0].adp, kAdp);
    EXPECT_EQ(transport.writes[0].ado, 0x0816);
    EXPECT_EQ(transport.writes[0].len, 1u);
    EXPECT_EQ(transport.writes[0].data[0], 0x00);

    EXPECT_EQ(transport.writes[1].ado, 0x0810);
    ASSERT_EQ(transport.writes[1].len, 5u);
    EXPECT_EQ(transport.writes[1].data[0], 0x00);   // phys addr LE
    EXPECT_EQ(transport.writes[1].data[1], 0x18);
    EXPECT_EQ(transport.writes[1].data[2], 0x04);   // length LE
    EXPECT_EQ(transport.writes[1].data[3], 0x00);
    EXPECT_EQ(transport.writes[1].data[4], 0x44);   // buffered|dir-write|repeat

    // SM3 (base 0x0818, process input): disable then block.
    EXPECT_EQ(transport.writes[2].ado, 0x081E);
    EXPECT_EQ(transport.writes[2].data[0], 0x00);
    EXPECT_EQ(transport.writes[3].ado, 0x0818);
    EXPECT_EQ(transport.writes[3].data[0], 0x00);
    EXPECT_EQ(transport.writes[3].data[1], 0x1C);
    EXPECT_EQ(transport.writes[3].data[2], 0x08);
    EXPECT_EQ(transport.writes[3].data[4], 0x40);   // buffered|dir-read|repeat

    // Activation phase comes after all SMs are programmed.
    EXPECT_EQ(transport.writes[4].ado, 0x0816);
    EXPECT_EQ(transport.writes[4].data[0], 0x01);   // SM_ACT_ENABLE
    EXPECT_EQ(transport.writes[5].ado, 0x081E);
    EXPECT_EQ(transport.writes[5].data[0], 0x01);

    EXPECT_TRUE(pdo_mgr.slaveConfigs()[0].configured);
}

TEST_F(PDOManagerSMConfigTest, DisableFailureIsNonFatal) {
    transport.fail_writes = {1, 3};   // both SM disable writes
    EXPECT_TRUE(pdo_mgr.configureSlavesSMs(0));
    EXPECT_EQ(transport.writes.size(), 6u);   // kept going
}

TEST_F(PDOManagerSMConfigTest, BlockWriteFailureAborts) {
    transport.fail_writes = {2};              // SM2 register block
    EXPECT_FALSE(pdo_mgr.configureSlavesSMs(0));
    EXPECT_EQ(transport.writes.size(), 2u);   // nothing past the block
    EXPECT_FALSE(pdo_mgr.slaveConfigs()[0].configured);
}

TEST_F(PDOManagerSMConfigTest, ActivateWriteFailureAborts) {
    transport.fail_writes = {5};              // SM2 sequential activate
    EXPECT_FALSE(pdo_mgr.configureSlavesSMs(0));
    EXPECT_EQ(transport.writes.size(), 5u);
    EXPECT_FALSE(pdo_mgr.slaveConfigs()[0].configured);
}

TEST_F(PDOManagerSMConfigTest, BatchedActivationFrame) {
    transport.prereg_ok_ = true;
    ASSERT_TRUE(pdo_mgr.configureSlavesSMs(0));

    // Per SM: disable + block only — activation went out in one frame.
    ASSERT_EQ(transport.writes.size(), 4u);
    EXPECT_EQ(transport.writes[1].ado, 0x0810);
    EXPECT_EQ(transport.writes[3].ado, 0x0818);

    ASSERT_EQ(transport.last_multi_ado.size(), 2u);
    EXPECT_EQ(transport.last_multi_adp[0], kAdp);
    EXPECT_EQ(transport.last_multi_ado[0], 0x0816);
    EXPECT_EQ(transport.last_multi_ado[1], 0x081E);
    EXPECT_EQ(transport.last_multi_data[0], 0x01);
    EXPECT_EQ(transport.last_multi_data[1], 0x01);
    EXPECT_EQ(transport.confirmed_slots.size(), 2u);
    EXPECT_TRUE(transport.cancelled_slots.empty());
}

TEST_F(PDOManagerSMConfigTest, BatchSendFailureFallsBackToSequential) {
    transport.prereg_ok_     = true;
    transport.multi_send_ok_ = false;
    EXPECT_TRUE(pdo_mgr.configureSlavesSMs(0));

    EXPECT_EQ(transport.cancelled_slots.size(), 2u);
    ASSERT_EQ(transport.writes.size(), 6u);
    EXPECT_EQ(transport.writes[4].ado, 0x0816);   // sequential activates
    EXPECT_EQ(transport.writes[5].ado, 0x081E);
}

TEST_F(PDOManagerSMConfigTest, BatchWaitFailureFallsBackToSequential) {
    transport.prereg_ok_      = true;
    transport.wait_prereg_ok_ = false;
    EXPECT_TRUE(pdo_mgr.configureSlavesSMs(0));

    EXPECT_EQ(transport.cancelled_slots.size(), 2u);
    EXPECT_EQ(transport.writes.size(), 6u);
}

TEST_F(PDOManagerSMConfigTest, MidBatchPreregFailureFallsBack) {
    transport.prereg_ok_     = true;
    transport.prereg_fail_at = 1;    // SM3's preRegister fails
    EXPECT_TRUE(pdo_mgr.configureSlavesSMs(0));

    EXPECT_EQ(transport.cancelled_slots.size(), 1u);   // only SM2's slot
    EXPECT_EQ(transport.writes.size(), 6u);
}

TEST_F(PDOManagerSMConfigTest, ConfigureAllCountsSuccesses) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[1].sm[0] = SyncManagerConfig::mailbox_write(0x1000, 0x80);
    cfgs[1].sm[1] = SyncManagerConfig::mailbox_read(0x1080, 0x80);

    EXPECT_EQ(pdo_mgr.configureAllSlaveSMs(2), 2u);
    EXPECT_EQ(pdo_mgr.slaveCount(), 2u);

    // Slave 1's mailbox SMs went to SM0/SM1 bases under adp 0x1001.
    bool saw_sm0 = false, saw_sm1 = false;
    for (const auto& w : transport.writes) {
        if (w.adp == 0x1001 && w.ado == 0x0800) saw_sm0 = true;
        if (w.adp == 0x1001 && w.ado == 0x0808) saw_sm1 = true;
    }
    EXPECT_TRUE(saw_sm0);
    EXPECT_TRUE(saw_sm1);
}

TEST_F(PDOManagerSMConfigTest, ConfigureAllCountsPartialFailure) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[1].sm[2] = SyncManagerConfig::process_output(0x1900, 4);

    transport.fail_writes = {8};    // slave 1's first block write
    EXPECT_EQ(pdo_mgr.configureAllSlaveSMs(2), 1u);
    EXPECT_TRUE(pdo_mgr.slaveConfigs()[0].configured);
    EXPECT_FALSE(pdo_mgr.slaveConfigs()[1].configured);
}

TEST_F(PDOManagerSMConfigTest, ConfigureAllClampsToMaxSlaves) {
    // Slaves without SMs configure trivially — returns the clamp count.
    EXPECT_EQ(pdo_mgr.configureAllSlaveSMs(
                  static_cast<uint16_t>(PDO::kMaxPDOSlaves + 2)),
              static_cast<uint16_t>(PDO::kMaxPDOSlaves));
    EXPECT_EQ(pdo_mgr.slaveCount(),
              static_cast<size_t>(PDO::kMaxPDOSlaves) + 2);
}

TEST_F(PDOManagerSMConfigTest, DebugFlagsExerciseLoggingArms) {
    EtherCATMasterDebugFlags flags;
    flags.pdoSm = true;
    flags.rxPDO = true;
    flags.txPDO = true;
    flags.mailboxConfiguration = true;
    flags.pdoConfiguration = true;
    pdo_mgr.setDebugFlags(&flags);

    // Mailbox SMs hit the is_mbx debug arm; all four SM types plus extra
    // control bits hit the type-switch and IRQ/watchdog log arms.
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].sm[0] = SyncManagerConfig::mailbox_write(0x1000, 0x80);
    cfgs[0].sm[1] = SyncManagerConfig::mailbox_read(0x1080, 0x80);
    cfgs[0].sm[2].control.ecat_irq = 1;
    cfgs[0].sm[2].control.pdi_irq  = 1;
    cfgs[0].sm[2].control.watchdog = 1;
    cfgs[0].sm[3].control.mode     = 0x3;   // 3-PDO mode string arm

    EXPECT_TRUE(pdo_mgr.configureSlavesSMs(0));
    EXPECT_EQ(transport.writes.size(), 12u);
}

// ============================================================================
// PDOManager physical (FPWR/FPRD) exchange coverage
// ============================================================================

namespace {

class PDOManagerPhysicalTest : public ::testing::Test {
protected:
    SMConfigStubTransport transport;
    PDOManager pdo_mgr{transport};
    EtherCATMasterDebugFlags debug_flags;

    void SetUp() override {
        auto* cfgs = pdo_mgr.slaveConfigs();
        cfgs[0].configured = true;
        cfgs[0].configured_address = 0x2000;
        cfgs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 16);
        cfgs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 12);
        cfgs[0].rxpdo_size = 16;
        cfgs[0].txpdo_size = 12;

        ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 16, 0x1704), 0);
        ASSERT_GE(pdo_mgr.mapping().add_txpdo(0, 12, 0x1B03), 0);
        pdo_mgr.mapping().get_entry_mut(0)->physical_offset = 0x1800;
        pdo_mgr.mapping().get_entry_mut(1)->physical_offset = 0x1C00;

        transport.prereg_ok_ = true;
        transport.prereg_wkc_ = 1;
        transport.prereg_resp_len_ = 12;
        for (size_t i = 0; i < 12; ++i)
            transport.prereg_resp_buf_[i] = static_cast<uint8_t>(0xA0 + i);
    }
};

}  // namespace

TEST_F(PDOManagerPhysicalTest, ExchangePhysicalZeroCountSucceeds) {
    EXPECT_TRUE(pdo_mgr.exchangePhysical(0));
    EXPECT_TRUE(transport.writes.empty());
}

TEST_F(PDOManagerPhysicalTest, BatchedWriteReadRoundTrip) {
    ASSERT_TRUE(pdo_mgr.exchangePhysical(1));

    ASSERT_EQ(transport.last_multi_ado.size(), 2u);
    EXPECT_EQ(transport.last_multi_adp[0], 0x2000);   // configured address
    EXPECT_EQ(transport.last_multi_ado[0], 0x1800);   // FPWR -> SM2 phys
    EXPECT_EQ(transport.last_multi_ado[1], 0x1C00);   // FPRD -> SM3 phys
    EXPECT_EQ(transport.confirmed_slots.size(), 2u);

    // The FPRD payload landed in the TxPDO entry's storage.
    const auto* e = pdo_mgr.mapping().get_entry(1);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(std::memcmp(e->storage, transport.prereg_resp_buf_, 12), 0);
    EXPECT_EQ(e->success_count, 1u);
}

TEST_F(PDOManagerPhysicalTest, RxStorageGatheredIntoWriteFrame) {
    auto* e = pdo_mgr.mapping().get_entry_mut(0);
    std::memset(e->storage, 0, 16);
    e->storage[0] = 0x5A;
    e->storage[15] = 0xA5;

    ASSERT_TRUE(pdo_mgr.exchangePhysical(1));
    ASSERT_EQ(transport.last_multi_payloads.size(), 2u);
    ASSERT_EQ(transport.last_multi_payloads[0].size(), 16u);
    EXPECT_EQ(transport.last_multi_payloads[0][0], 0x5A);
    EXPECT_EQ(transport.last_multi_payloads[0][15], 0xA5);
}

TEST_F(PDOManagerPhysicalTest, MultiSendFailureCountsBothErrors) {
    transport.multi_send_ok_ = false;
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
    // Both preregistered waiters were drained as cancellation.
    EXPECT_EQ(transport.confirmed_slots.size(), 2u);
}

TEST_F(PDOManagerPhysicalTest, WaitTimeoutFailsExchange) {
    transport.wait_prereg_ok_ = false;
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
    const auto* e = pdo_mgr.mapping().get_entry(1);
    EXPECT_EQ(e->success_count, 0u);
}

TEST_F(PDOManagerPhysicalTest, ZeroWkcFailsExchange) {
    transport.prereg_wkc_ = 0;
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, WriteWkcZeroStillCopiesRead) {
    // Write datagram WKC 0 fails the write side but the read side still
    // scatters its payload into entry storage.
    transport.prereg_wkc_plan_ = {0, 1};
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
    const auto* e = pdo_mgr.mapping().get_entry(1);
    EXPECT_EQ(std::memcmp(e->storage, transport.prereg_resp_buf_, 12), 0);
}

TEST_F(PDOManagerPhysicalTest, NonPreregWaitPath) {
    transport.prereg_ok_ = false;   // falls back to waitForResponseIdx
    transport.single_resp_wkc_ = 1;
    transport.single_resp_len_ = 12;
    std::memcpy(transport.single_resp_buf_, transport.prereg_resp_buf_, 12);
    EXPECT_TRUE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, NonPreregWaitTimeoutFails) {
    transport.prereg_ok_ = false;
    transport.single_respond_ = false;
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, WriteOnlyUsesWriteRegister) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].sm[3].type = SyncManagerType::Unused;
    ASSERT_TRUE(pdo_mgr.exchangePhysical(1));
    ASSERT_EQ(transport.writes.size(), 1u);
    EXPECT_EQ(transport.writes[0].ado, 0x1800);
    EXPECT_EQ(transport.writes[0].len, 16u);
    EXPECT_EQ(transport.writes[0].adp, 0x1000);   // adpForSlaveIndex
}

TEST_F(PDOManagerPhysicalTest, WriteOnlyFailure) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].sm[3].type = SyncManagerType::Unused;
    transport.fail_writes = {1};
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, ReadOnlyUsesReadRegister) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].sm[2].type = SyncManagerType::Unused;
    transport.read_byte_ = 0x7B;
    ASSERT_TRUE(pdo_mgr.exchangePhysical(1));
    ASSERT_EQ(transport.reads.size(), 1u);
    EXPECT_EQ(transport.reads[0].ado, 0x1C00);
    const auto* e = pdo_mgr.mapping().get_entry(1);
    EXPECT_EQ(e->storage[0], 0x7B);
    EXPECT_EQ(e->success_count, 1u);
}

TEST_F(PDOManagerPhysicalTest, ReadOnlyFailure) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].sm[2].type = SyncManagerType::Unused;
    transport.read_ok_ = false;
    EXPECT_FALSE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, SkipsUnconfiguredSlaves) {
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].configured = false;
    EXPECT_TRUE(pdo_mgr.exchangePhysical(1));
    EXPECT_TRUE(transport.writes.empty());
    EXPECT_TRUE(transport.reads.empty());
}

TEST_F(PDOManagerPhysicalTest, PeriodicWireLogDecodesPdoFields) {
    // should_log_wire fires on the 1000th successful cycle with debug
    // flags set — exercises the CiA402 controlword/statusword decode,
    // the RxPDO-layout switch (0x1704: tq@10 opmode@12), the mode arms,
    // and the digital-input TxPDO arm (0x1B03).
    debug_flags.rxPDO = true;
    debug_flags.txPDO = true;
    pdo_mgr.setDebugFlags(&debug_flags);

    auto* rx = pdo_mgr.mapping().get_entry_mut(0);
    std::memset(rx->storage, 0, 16);
    rx->storage[0] = 0x0F;                    // controlword bits
    rx->storage[12] = 8;                      // opmode = CSP

    for (int i = 0; i < 1000; ++i)
        ASSERT_TRUE(pdo_mgr.exchangePhysical(1));

    // Second 1000-cycle block with a different mode byte hits another
    // mode arm (CSV) of the interpreter log.
    rx->storage[12] = 9;
    for (int i = 0; i < 1000; ++i)
        ASSERT_TRUE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, WireLogRemainingDecodeArms) {
    debug_flags.rxPDO = true;
    debug_flags.txPDO = true;
    pdo_mgr.setDebugFlags(&debug_flags);

    // Rebuild the mapping with a 0x1705 RxPDO (opmode @10) + 0x1B04
    // TxPDO (trailing field = velocity, not digital inputs).
    pdo_mgr.mapping().clear();
    ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 16, 0x1705), 0);
    ASSERT_GE(pdo_mgr.mapping().add_txpdo(0, 12, 0x1B04), 0);
    pdo_mgr.mapping().get_entry_mut(0)->physical_offset = 0x1800;
    pdo_mgr.mapping().get_entry_mut(1)->physical_offset = 0x1C00;

    auto* rx = pdo_mgr.mapping().get_entry_mut(0);
    std::memset(rx->storage, 0, 16);
    rx->storage[10] = 10;                     // opmode = CST
    for (int i = 0; i < 1000; ++i)
        ASSERT_TRUE(pdo_mgr.exchangePhysical(1));

    rx->storage[10] = 4;                      // unrecognized mode arm
    for (int i = 0; i < 1000; ++i)
        ASSERT_TRUE(pdo_mgr.exchangePhysical(1));
}

TEST_F(PDOManagerPhysicalTest, WireLogNoModeFieldLayout) {
    debug_flags.rxPDO = true;
    debug_flags.txPDO = true;
    pdo_mgr.setDebugFlags(&debug_flags);

    // 0x1701: no mode field, no target velocity — the raw-targets arm.
    pdo_mgr.mapping().clear();
    ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 16, 0x1701), 0);
    ASSERT_GE(pdo_mgr.mapping().add_txpdo(0, 12, 0x1B02), 0);
    pdo_mgr.mapping().get_entry_mut(0)->physical_offset = 0x1800;
    pdo_mgr.mapping().get_entry_mut(1)->physical_offset = 0x1C00;

    for (int i = 0; i < 1000; ++i)
        ASSERT_TRUE(pdo_mgr.exchangePhysical(1));
}

// ============================================================================
// PDOManager bulk sendAll/receiveAll (non-LRW batched path)
// ============================================================================

namespace {

class PDOManagerBulkTest : public ::testing::Test {
protected:
    SMConfigStubTransport transport;
    PDOManager pdo_mgr{transport};
    EtherCATMasterDebugFlags debug_flags;

    /// Six entries covering every addressing arm on both phases:
    ///   0 rx Position (fire-and-forget APWR)
    ///   1 rx ConfiguredAddress (confirmed FPWR)
    ///   2 rx Broadcast (confirmed BWR)
    ///   3 tx Position (fire-and-forget APRD)
    ///   4 tx ConfiguredAddress (confirmed FPRD)
    ///   5 tx Broadcast (confirmed BRD)
    void buildSixEntryMap() {
        pdo_mgr.mapping().clear();
        ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                            PDOAddressMode::Position), 0);
        ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 4, 0x1601,
                                            PDOAddressMode::ConfiguredAddress), 0);
        ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 4, 0x1602,
                                            PDOAddressMode::Broadcast), 0);
        ASSERT_GE(pdo_mgr.mapping().add_txpdo(0, 4, 0x1A00,
                                            PDOAddressMode::Position), 0);
        ASSERT_GE(pdo_mgr.mapping().add_txpdo(0, 4, 0x1A01,
                                            PDOAddressMode::ConfiguredAddress), 0);
        ASSERT_GE(pdo_mgr.mapping().add_txpdo(0, 4, 0x1A02,
                                            PDOAddressMode::Broadcast), 0);
        for (size_t i = 0; i < 6; ++i) {
            auto* e = pdo_mgr.mapping().get_entry_mut(i);
            e->physical_offset = static_cast<uint16_t>(0x1800 + i * 0x10);
            e->configured_address = 0x2000;
            e->data_size = 4;
        }
        transport.prereg_ok_ = true;
        transport.prereg_wkc_ = 1;
        transport.prereg_resp_len_ = 4;
        std::memset(transport.prereg_resp_buf_, 0x5C, 4);
    }
};

}  // namespace

TEST_F(PDOManagerBulkTest, SendAllBuildsBatchedRxFrame) {
    buildSixEntryMap();
    ASSERT_TRUE(pdo_mgr.sendAll());

    // One multi frame, three Rx specs covering all address modes.
    ASSERT_EQ(transport.last_multi_adp.size(), 3u);
    EXPECT_EQ(transport.last_multi_adp[0], 0x1000);   // APWR position
    EXPECT_EQ(transport.last_multi_ado[0], 0x1800);
    EXPECT_EQ(transport.last_multi_adp[1], 0x2000);   // FPWR configured
    EXPECT_EQ(transport.last_multi_ado[1], 0x1810);
    EXPECT_EQ(transport.last_multi_adp[2], 0x0000);   // BWR broadcast
    EXPECT_EQ(transport.last_multi_ado[2], 0x1820);

    // Fire-and-forget entry counted immediately; confirmed wait for
    // receiveAll.
    const auto* e0 = pdo_mgr.mapping().get_entry(0);
    const auto* e1 = pdo_mgr.mapping().get_entry(1);
    EXPECT_EQ(e0->success_count, 1u);
    EXPECT_EQ(e1->success_count, 0u);

    EXPECT_TRUE(pdo_mgr.receiveAll());
    EXPECT_EQ(e1->success_count, 1u);
    // Confirmed tx data scattered into storage.
    const auto* e4 = pdo_mgr.mapping().get_entry(4);
    EXPECT_EQ(e4->storage[0], 0x5C);
    EXPECT_EQ(e4->success_count, 1u);
}

TEST_F(PDOManagerBulkTest, SendAllSendFailureMarksErrors) {
    buildSixEntryMap();
    transport.multi_send_ok_ = false;
    EXPECT_FALSE(pdo_mgr.sendAll());
    for (size_t i = 0; i < 3; ++i)
        EXPECT_EQ(pdo_mgr.mapping().get_entry(i)->error_count, 1u);
}

TEST_F(PDOManagerBulkTest, SendAllLogicalOnlyEntriesFail) {
    pdo_mgr.mapping().clear();
    ASSERT_GE(pdo_mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                        PDOAddressMode::Logical), 0);
    EXPECT_FALSE(pdo_mgr.sendAll());
    EXPECT_EQ(pdo_mgr.mapping().get_entry(0)->error_count, 1u);
}

TEST_F(PDOManagerBulkTest, SendAllSkipsDisabledEntries) {
    buildSixEntryMap();
    pdo_mgr.mapping().get_entry_mut(0)->enabled = false;
    EXPECT_TRUE(pdo_mgr.sendAll());
    ASSERT_EQ(transport.last_multi_adp.size(), 2u);
    EXPECT_TRUE(pdo_mgr.receiveAll());
}

TEST_F(PDOManagerBulkTest, ReceiveAllConfirmedTimeoutMarksErrors) {
    buildSixEntryMap();
    ASSERT_TRUE(pdo_mgr.sendAll());
    transport.wait_prereg_ok_ = false;
    EXPECT_FALSE(pdo_mgr.receiveAll());
    EXPECT_EQ(pdo_mgr.mapping().get_entry(1)->error_count, 1u);
    EXPECT_EQ(pdo_mgr.mapping().get_entry(4)->error_count, 1u);
}

TEST_F(PDOManagerBulkTest, ReceiveAllTxSendFailureMarksErrors) {
    buildSixEntryMap();
    ASSERT_TRUE(pdo_mgr.sendAll());
    transport.multi_send_ok_ = false;      // tx frame fails
    EXPECT_FALSE(pdo_mgr.receiveAll());
    for (size_t i = 3; i < 6; ++i)
        EXPECT_EQ(pdo_mgr.mapping().get_entry(i)->error_count, 1u);
}

TEST_F(PDOManagerBulkTest, ReceiveAllShortResponseMarksError) {
    buildSixEntryMap();
    transport.prereg_resp_len_ = 2;        // < data_size (4)
    ASSERT_TRUE(pdo_mgr.sendAll());
    EXPECT_FALSE(pdo_mgr.receiveAll());
    EXPECT_EQ(pdo_mgr.mapping().get_entry(4)->error_count, 1u);
}

TEST_F(PDOManagerBulkTest, NonPreregWaitPathUsesResponseIdx) {
    buildSixEntryMap();
    transport.prereg_ok_ = false;
    transport.single_resp_wkc_ = 1;
    transport.single_resp_len_ = 4;
    std::memset(transport.single_resp_buf_, 0x5C, 4);
    ASSERT_TRUE(pdo_mgr.sendAll());
    EXPECT_TRUE(pdo_mgr.receiveAll());
    EXPECT_EQ(pdo_mgr.mapping().get_entry(4)->storage[0], 0x5C);
}

TEST_F(PDOManagerBulkTest, ExchangeAllCombines) {
    buildSixEntryMap();
    EXPECT_TRUE(pdo_mgr.exchangeAll());
}

TEST_F(PDOManagerBulkTest, DebugFlagsAndGateCheckpoints) {
    debug_flags.rxPDO = true;
    debug_flags.txPDO = true;
    pdo_mgr.setDebugFlags(&debug_flags);
    DebugGate gate;
    pdo_mgr.setDebugGate(&gate);
    buildSixEntryMap();
    EXPECT_TRUE(pdo_mgr.exchangeAll());
    // Second cycle — first-rxpdo/first-txpdo already emitted.
    EXPECT_TRUE(pdo_mgr.exchangeAll());
}

TEST_F(PDOManagerBulkTest, CallbackModeFiresOnBulkExchange) {
    buildSixEntryMap();
    CallbackModeConfig cbc;
    pdo_mgr.configureCallbackMode(cbc);

    int tx_sent = 0, rx_received = 0;
    for (size_t i = 0; i < 6; ++i) {
        pdo_mgr.setTxSentCallback(i, [&](uint16_t, uint32_t, uint64_t) {
            ++tx_sent;
        });
        pdo_mgr.setRxReceivedCallback(i,
            [&](uint16_t, const uint8_t*, size_t, uint32_t, uint64_t) {
                ++rx_received;
            });
    }

    EXPECT_TRUE(pdo_mgr.exchangeAll());
    // 1 fire-and-forget RxPDO + 2 confirmed RxPDOs.
    EXPECT_EQ(tx_sent, 3);
    // 1 fire-and-forget TxPDO + 2 confirmed TxPDOs.
    EXPECT_EQ(rx_received, 3);
}

TEST_F(PDOManagerBulkTest, LrwSlicedSendForLargeImage) {
    // Reuse the LAM: shrink the stub's per-frame payload below the
    // 12-byte image so sendAll() takes the sliced-LRW path.
    LogicalAddressManager lam{transport};
    lam.init();
    SlaveConfig configs[kMaxPDOSlaves] = {};
    configs[0].configured = true;
    configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
    configs[0].rxpdo_size = 4;
    configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
    configs[0].txpdo_size = 8;
    ASSERT_TRUE(lam.buildAddressMap(configs, 1));

    pdo_mgr.mapping().clear();
    pdo_mgr.mapping().add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
    pdo_mgr.mapping().add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
    pdo_mgr.setLogicalAddressManager(&lam);

    transport.payload_ = 16;    // max slice = 4 bytes < 12-byte image
    for (size_t s = 0; s < 4; ++s) {
        transport.resp_len_[s] = 4;
        transport.resp_wkc_[s] = 3;
    }
    EXPECT_TRUE(pdo_mgr.sendAll());
    // LRW mode -> receiveAll returns the send phase result.
    EXPECT_TRUE(pdo_mgr.receiveAll());
}

// ============================================================================
// PDOManager core accessors and pdo_* test hooks
// ============================================================================

TEST_F(PDOManagerBulkTest, CoreAccessorsAndWrappers) {
    buildSixEntryMap();

    // Free-function test hooks.
    EXPECT_EQ(pdo_get_mapping(pdo_mgr).entry_count(), 6u);
    EXPECT_NE(pdo_get_slave_configs(pdo_mgr), nullptr);
    EXPECT_TRUE(pdo_configure_slave_sms(pdo_mgr, 0));
    EXPECT_EQ(pdo_configure_all_slave_sms(pdo_mgr, 1), 1u);
    EXPECT_TRUE(pdo_exchange_all(pdo_mgr));
    EXPECT_FALSE(pdo_exchange_lrw(pdo_mgr, 1));   // no LAM -> error
    EXPECT_TRUE(pdo_exchange_lrw(pdo_mgr, 0));

    pdo_set_separate_mode(pdo_mgr, true);
    EXPECT_TRUE(pdo_get_separate_mode(pdo_mgr));
    pdo_set_separate_mode(pdo_mgr, false);
    EXPECT_FALSE(pdo_get_separate_mode(pdo_mgr));
    EXPECT_FALSE(pdo_exchange_separate(pdo_mgr, 1));
    EXPECT_TRUE(pdo_exchange_separate(pdo_mgr, 0));

    pdo_set_physical_mode(pdo_mgr, true);
    EXPECT_TRUE(pdo_get_physical_mode(pdo_mgr));

    uint32_t a = 0, b = 0, c = 0, d = 0;
    pdo_get_lrw_stats(pdo_mgr, &a, &b, &c, &d);
    EXPECT_EQ(c, 1u);   // the failed LRW attempt above
    pdo_get_lrw_stats(pdo_mgr, nullptr, nullptr, nullptr, nullptr);
    pdo_get_separate_stats(pdo_mgr, &a, &b, &c, &d);   // all zero
    pdo_get_separate_stats(pdo_mgr, nullptr, nullptr, nullptr, nullptr);

    // Physical exchange via the wrapper drives FPWR/FPRD stats.
    auto* cfgs = pdo_mgr.slaveConfigs();
    cfgs[0].configured = true;
    cfgs[0].configured_address = 0x2000;
    cfgs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
    cfgs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 4);
    for (size_t i = 0; i < 6; ++i)
        pdo_mgr.mapping().get_entry_mut(i)->physical_offset =
            (i < 3) ? static_cast<uint16_t>(0x1800 + i)
                    : static_cast<uint16_t>(0x1C00 + i - 3);
    EXPECT_TRUE(pdo_exchange_physical(pdo_mgr, 1));
    pdo_get_physical_stats(pdo_mgr, &a, &b, &c, &d);
    EXPECT_EQ(a, 1u);
    EXPECT_EQ(c, 1u);
    pdo_get_physical_stats(pdo_mgr, nullptr, nullptr, nullptr, nullptr);

    EXPECT_TRUE(pdo_finalize_mapping(pdo_mgr, 0));

    PDOStats stats = pdo_get_stats(pdo_mgr);
    EXPECT_GT(stats.total_cycles, 0u);
    pdo_reset_stats(pdo_mgr);
    EXPECT_EQ(pdo_get_stats(pdo_mgr).total_cycles, 0u);
}

TEST_F(PDOManagerBulkTest, SendRecvSingleEntryAndEntryLookup) {
    buildSixEntryMap();
    EXPECT_TRUE(pdo_mgr.hasSlavePDOEntries(0));
    EXPECT_FALSE(pdo_mgr.hasSlavePDOEntries(5));

    // Single-entry helpers are wrappers over the same transport.
    pdo_send_rxpdo(pdo_mgr, 0);
    pdo_receive_txpdo(pdo_mgr, 3);
}

TEST_F(PDOManagerBulkTest, DumpStatsAndDeinitShutdown) {
    debug_flags.shutdown = true;
    pdo_mgr.setDebugFlags(&debug_flags);
    pdo_mgr.init();
    pdo_mgr.dumpStats("test");
    EXPECT_TRUE(pdo_mgr.isInitialized());
    pdo_mgr.deinit();
    EXPECT_FALSE(pdo_mgr.isInitialized());
}

TEST_F(PDOManagerBulkTest, SlaveCountSetterAndCounts) {
    pdo_mgr.setSlaveCount(7);
    EXPECT_EQ(pdo_mgr.slaveCount(), 7u);
    EXPECT_EQ(pdo_mgr.getSlavePDORequestCount(PDO::kMaxPDOSlaves), 0u);
    EXPECT_EQ(pdo_mgr.getSlavePDOReplyCount(PDO::kMaxPDOSlaves), 0u);
}

// ============================================================================
// LogicalAddressManager polled-path error coverage
// ============================================================================

namespace {

class LAMErrorPathsTest : public ::testing::Test {
protected:
    SMConfigStubTransport transport;
    LogicalAddressManager lam{transport};
    PDOMapping mapping;
    EtherCATMasterDebugFlags debug_flags;

    void buildMap() {
        lam.init();
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        ASSERT_TRUE(lam.buildAddressMap(configs, 1));
        mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
        transport.single_resp_wkc_ = 3;
        transport.single_resp_len_ = 12;
    }
};

}  // namespace

TEST_F(LAMErrorPathsTest, ExchangeFailsWhenNotInitialized) {
    // exchangeAllLRW early-outs on a zero-byte image; the slice and
    // per-slave entry points reject up front.
    EXPECT_FALSE(lam.exchangeLRWSlice(mapping, 0, 4));
    lam.init();
    EXPECT_FALSE(lam.exchangeLRWSlice(mapping, 0, 4));   // no slaves built
    EXPECT_GT(lam.getStats().send_errors, 0u);
}

TEST_F(LAMErrorPathsTest, ExchangeLRWForSlavesFailsWhenNotInitialized) {
    lam.init();
    EXPECT_FALSE(lam.exchangeLRWForSlaves(mapping, 1));
}

TEST_F(LAMErrorPathsTest, BuildAddressMapRejectsEmptyInput) {
    lam.init();
    SlaveConfig configs[kMaxPDOSlaves] = {};
    EXPECT_FALSE(lam.buildAddressMap(configs, 0));
}

TEST_F(LAMErrorPathsTest, InitDeinitWithShutdownFlag) {
    debug_flags.shutdown = true;
    lam.setDebugFlags(&debug_flags);
    buildMap();
    EXPECT_TRUE(lam.isInitialized());
    lam.deinit();
    EXPECT_FALSE(lam.isInitialized());
    // Sliced/per-slave exchanges reject after deinit.
    EXPECT_FALSE(lam.exchangeLRWSlice(mapping, 0, 4));
    EXPECT_FALSE(lam.exchangeLRWForSlaves(mapping, 1));
}

TEST_F(LAMErrorPathsTest, SliceOutOfRangeAndOversizeRejected) {
    buildMap();
    EXPECT_FALSE(lam.exchangeLRWSlice(mapping, 20, 4));     // past image
    EXPECT_FALSE(lam.exchangeLRWSlice(mapping, 0,
                                      lam.maxSliceLength() + 1));
    EXPECT_TRUE(lam.exchangeLRWSlice(mapping, 0, 0));       // empty ok
    EXPECT_EQ(lam.getStats().send_errors, 2u);
}

TEST_F(LAMErrorPathsTest, SendFailureCountsAndCancelsSlot) {
    buildMap();
    transport.prereg_ok_ = true;           // take the claimed-slot arm
    transport.single_send_ok_ = false;
    EXPECT_FALSE(lam.exchangeAllLRW(mapping));
    EXPECT_EQ(lam.getStats().send_errors, 1u);
}

TEST_F(LAMErrorPathsTest, ResponseTimeoutCountsAndDrains) {
    buildMap();
    transport.single_respond_ = false;
    EXPECT_FALSE(lam.exchangeAllLRW(mapping));
    EXPECT_EQ(lam.getStats().timeout_errors, 1u);
    EXPECT_EQ(lam.getStats().consecutive_timeouts, 1u);
}

TEST_F(LAMErrorPathsTest, ZeroWkcCountsAsError) {
    buildMap();
    transport.single_resp_wkc_ = 0;
    EXPECT_FALSE(lam.exchangeAllLRW(mapping));
    EXPECT_EQ(lam.getStats().wkc_errors, 1u);
}

TEST_F(LAMErrorPathsTest, SuccessResetsConsecutiveTimeouts) {
    buildMap();
    transport.single_respond_ = false;
    EXPECT_FALSE(lam.exchangeAllLRW(mapping));
    transport.single_respond_ = true;
    EXPECT_TRUE(lam.exchangeAllLRW(mapping));
    EXPECT_EQ(lam.getStats().consecutive_timeouts, 0u);
}

TEST_F(LAMErrorPathsTest, BusyWaiterSlotFailsExchange) {
    buildMap();
    transport.prereg_busy_ = true;
    EXPECT_FALSE(lam.exchangeAllLRW(mapping));
    EXPECT_EQ(lam.getStats().send_errors, 1u);
}

TEST_F(LAMErrorPathsTest, ForSlavesPaths) {
    buildMap();
    EXPECT_TRUE(lam.exchangeLRWForSlaves(mapping, 0));   // empty mask

    transport.single_send_ok_ = false;
    EXPECT_FALSE(lam.exchangeLRWForSlaves(mapping, 1));
    transport.single_send_ok_ = true;

    transport.single_respond_ = false;
    EXPECT_FALSE(lam.exchangeLRWForSlaves(mapping, 1));
    transport.single_respond_ = true;

    transport.single_resp_wkc_ = 0;
    EXPECT_FALSE(lam.exchangeLRWForSlaves(mapping, 1));
    transport.single_resp_wkc_ = 3;

    // Success: tx region copied into entry storage.
    std::memset(transport.single_resp_buf_, 0x33, 12);
    EXPECT_TRUE(lam.exchangeLRWForSlaves(mapping, 1));
    EXPECT_EQ(mapping.get_entry(1)->storage[0], 0x33);
}

TEST_F(LAMErrorPathsTest, ForSlavesInactiveMaskSucceedsTrivially) {
    buildMap();
    // Mask hits a slave with no active map region.
    EXPECT_TRUE(lam.exchangeLRWForSlaves(mapping, 1u << 5));
}

TEST_F(LAMErrorPathsTest, KnobAccessors) {
    lam.setLrwResponseTimeoutMs(20);
    EXPECT_EQ(lam.lrwResponseTimeoutMs(), 20u);
    lam.setLrwResponseTimeoutMs(0);
    EXPECT_EQ(lam.lrwResponseTimeoutMs(), 1u);
    lam.setStallDetectionGapUs(1234);
    EXPECT_EQ(lam.stallDetectionGapUs(), 1234u);
    lam.setEscalateAfterTimeouts(4);
}

// ============================================================================
// LAM cyclic/slice error-path coverage
// ============================================================================

TEST_F(LAMErrorPathsTest, CyclicSendRejectsWhenUninitialized) {
    lam.init();   // initialized but no slaves
    EXPECT_FALSE(lam.cyclicSend(mapping, nullptr, 1'000'000));
}

TEST_F(LAMErrorPathsTest, CyclicSendFallsBackWithoutFastPath) {
    transport.fast_path_ = false;
    buildMap();
    // Legacy path: single polled LRW datagram.
    EXPECT_TRUE(lam.cyclicSend(mapping, nullptr, 1'000'000));
    // Collect is a no-op true on the non-fast-path transport.
    EXPECT_TRUE(lam.cyclicCollect(mapping, nullptr));
}

TEST_F(LAMErrorPathsTest, CyclicSendDatagramFailure) {
    buildMap();
    transport.send_ok_ = false;
    EXPECT_FALSE(lam.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_EQ(lam.getStats().send_errors, 1u);
}

TEST_F(LAMErrorPathsTest, CyclicCollectWithNothingPending) {
    buildMap();
    EXPECT_TRUE(lam.cyclicCollect(mapping, nullptr));
}

TEST_F(LAMErrorPathsTest, CyclicStaleEchoRetriesAndRecovers) {
    buildMap();
    transport.resp_len_[0] = 12;
    transport.resp_wkc_[0] = 3;
    transport.stale_[0] = 1;    // first deposit echoes the previous gen
    ASSERT_TRUE(lam.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(lam.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(lam.getStats().stale_responses, 1u);
}

TEST_F(LAMErrorPathsTest, CyclicDoubleStaleCountsMiss) {
    buildMap();
    transport.resp_len_[0] = 12;
    transport.resp_wkc_[0] = 3;
    transport.stale_[0] = 2;    // both deposits stale -> slice times out
    ASSERT_TRUE(lam.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(lam.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(lam.getStats().stale_responses, 2u);
    EXPECT_EQ(lam.getStats().timeout_errors, 1u);
}

TEST_F(LAMErrorPathsTest, CyclicMissingResponseTimesOut) {
    buildMap();
    transport.respond_[0] = false;
    ASSERT_TRUE(lam.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_FALSE(lam.cyclicCollect(mapping, nullptr));
    EXPECT_GE(lam.getStats().timeout_errors, 1u);
}

TEST_F(LAMErrorPathsTest, CyclicRttStatsAccumulate) {
    buildMap();
    transport.stamp_ns_ = UINT64_MAX;
    transport.resp_len_[0] = 12;
    transport.resp_wkc_[0] = 3;
    ASSERT_TRUE(lam.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(lam.cyclicCollect(mapping, nullptr));
    EXPECT_GE(lam.getStats().rtt_samples, 1u);
}

TEST_F(LAMErrorPathsTest, DefinePDOSliceRejectsBadSpecs) {
    // Not initialized.
    PDOSliceSpec spec;
    spec.entries = {0};
    EXPECT_EQ(lam.definePDOSlice(mapping, spec), 0xFFFFFFFFu);

    buildMap();
    // Empty spec.
    PDOSliceSpec empty;
    EXPECT_EQ(lam.definePDOSlice(mapping, empty), 0xFFFFFFFFu);
    // Spec selecting a nonexistent entry resolves to zero runs.
    PDOSliceSpec bogus;
    bogus.entries = {42};
    EXPECT_EQ(lam.definePDOSlice(mapping, bogus), 0xFFFFFFFFu);
}

TEST_F(LAMErrorPathsTest, ExchangePDOSliceFailures) {
    buildMap();
    PDOSliceSpec spec;
    spec.entries = {0};
    ASSERT_NE(lam.definePDOSlice(mapping, spec), 0xFFFFFFFFu);

    // One-off slices ride the polled path (sendSingleDatagram +
    // waitForResponseIdx) — send failure and timeout both fail it.
    transport.single_send_ok_ = false;
    EXPECT_FALSE(lam.exchangePDOSlice(mapping, spec));
    transport.single_send_ok_ = true;

    transport.single_respond_ = false;
    EXPECT_FALSE(lam.exchangePDOSlice(mapping, spec));
    transport.single_respond_ = true;

    // A spec that resolves to zero runs fails immediately.
    PDOSliceSpec bogus;
    bogus.entries = {99};
    EXPECT_FALSE(lam.exchangePDOSlice(mapping, bogus));

    transport.single_resp_wkc_ = 3;
    transport.single_resp_len_ = 12;
    EXPECT_TRUE(lam.exchangePDOSlice(mapping, spec));
}

// ============================================================================
// TxFailureDiagnostics — worker categories, escalation, wire probe
// ============================================================================

#include "raw/TxFailureDiagnostics.hpp"
#if defined(__linux__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

namespace {

// Drain one worker poll (~10 ms cadence) — returns when `seen` hits target
// or after ~2 s.
bool waitFor(std::atomic<int>& seen, int target) {
    for (int i = 0; i < 400 && seen.load() < target; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return seen.load() >= target;
}

}  // namespace

TEST(TxFailureDiagnosticsTest, StartIsIdempotentAndStopsCleanly) {
    TxFailureDiagnostics diag(nullptr, nullptr, nullptr, "txdiag_test");
    diag.start();
    diag.start();      // already running — no second spawn
    EXPECT_EQ(diag.spawns(), 1u);
    diag.stop();
    diag.start();      // restart reaps the stopped worker
    EXPECT_EQ(diag.spawns(), 2u);
    diag.stop();
}

TEST(TxFailureDiagnosticsTest, WorkerDrainsAllLogCategories) {
    std::atomic<int> probes{0};
    std::atomic<int> escalations{0};
    TxFailureDiagnostics diag(
        [&] { ++probes; return std::string("r | DIAGNOSIS: x"); },
        [] { return 111; /* ECONNREFUSED */ },
        [&] { ++escalations; return std::string("ALIVE"); },
        "txdiag_test");
    diag.setEscalateAfterTimeouts(2);
    diag.start();

    // Every category with a suppressed second hit.
    for (int i = 0; i < 2; ++i) {
        diag.noteSendFailure(0x3);
        diag.noteSendFailure(0);        // mask-less arm
        diag.noteNoResponseSlot();
        diag.noteTimeout("exchangeLRW");
        diag.noteWkcZero("exchangeLRW");
        diag.noteStall(5'000'000);
    }
    // First emits happen immediately (rate-limit baseline is "never");
    // give the worker two polls, then feed the suppressed-count arms.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    for (int i = 0; i < 3; ++i) {
        diag.noteSendFailure(0x1);
        diag.noteNoResponseSlot();
        diag.noteTimeout("exchangeLRWForSlaves");
        diag.noteWkcZero("exchangeLRWForSlaves");
        diag.noteStall(9'000'000);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Escalation fires when the timeout streak crosses the threshold.
    diag.noteTimeout("esc");
    diag.noteTimeout("esc");
    EXPECT_TRUE(waitFor(escalations, 1));
    diag.noteSuccess();               // resets the streak
    diag.noteTimeout("esc");
    diag.noteTimeout("esc");
    EXPECT_TRUE(waitFor(escalations, 2));

    // Send-fail streak of 10 fires the wire probe.
    for (int i = 0; i < 10; ++i) diag.noteSendFailure();
    EXPECT_TRUE(waitFor(probes, 1));
    diag.noteSendOk();                // streak reset
    diag.stop();
}

TEST(TxFailureDiagnosticsTest, EscalationVerdictChangeAndEmpty) {
    std::atomic<int> calls{0};
    std::atomic<bool> empty{false};
    TxFailureDiagnostics diag(
        nullptr, nullptr,
        [&] {
            ++calls;
            return empty.load() ? std::string() : std::string("OK");
        },
        "txdiag_test");
    diag.setEscalateAfterTimeouts(1);
    diag.start();
    diag.noteTimeout("x");            // ct=1 >= esc=1 -> verdict emit
    EXPECT_TRUE(waitFor(calls, 1));
    empty = true;
    diag.noteSuccess();
    // The worker must observe ct==0 before the next streak or the
    // escalation quotient never resets.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    diag.noteTimeout("x");            // empty verdict -> no log but called
    EXPECT_TRUE(waitFor(calls, 2));
    diag.stop();
}

#if defined(__linux__)
TEST(TxFailureDiagnosticsTest, ProbeRejectsBadAndNonPacketFds) {
    EXPECT_NE(TxFailureDiagnostics::probe(-1).find("no TX socket fd"),
              std::string::npos);

    int udp = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(udp, 0);
    EXPECT_NE(TxFailureDiagnostics::probe(udp).find("no interface"),
              std::string::npos);
    ::close(udp);
}

TEST(TxFailureDiagnosticsTest, ProbeRealPacketSocketIfAvailable) {
    // Needs CAP_NET_RAW — degrade to a soft check when unavailable.
    int fd = ::socket(AF_PACKET, SOCK_RAW, 0);
    if (fd < 0) {
        GTEST_SKIP() << "no CAP_NET_RAW";
    }
    const std::string report = TxFailureDiagnostics::probe(fd);
    ::close(fd);
    // Either unbound (no interface) or a full report — both are
    // deterministic outcomes of the environment.
    EXPECT_FALSE(report.empty());
}
#endif

// ============================================================================
// PDOManager single-entry transfer coverage
// ============================================================================

TEST_F(PDOManagerBulkTest, SingleEntryRejectsBadIndexAndDirection) {
    buildSixEntryMap();
    EXPECT_FALSE(pdo_mgr.sendRxPDO(99));          // out of range
    EXPECT_FALSE(pdo_mgr.sendRxPDO(3));           // TxPDO entry
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(0));        // RxPDO entry
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(99));
    pdo_mgr.mapping().get_entry_mut(0)->enabled = false;
    EXPECT_FALSE(pdo_mgr.sendRxPDO(0));           // disabled
}

TEST_F(PDOManagerBulkTest, SingleEntryAllAddressModes) {
    buildSixEntryMap();
    transport.single_respond_ = true;
    transport.single_resp_wkc_ = 1;
    transport.single_resp_len_ = 4;
    std::memset(transport.single_resp_buf_, 0x7E, 4);

    // Rx: Position (confirmed first call, fire-and-forget after),
    // ConfiguredAddress FPWR, Broadcast BWR.
    EXPECT_TRUE(pdo_mgr.sendRxPDO(0));
    EXPECT_TRUE(pdo_mgr.sendRxPDO(0));
    EXPECT_TRUE(pdo_mgr.sendRxPDO(1));
    EXPECT_TRUE(pdo_mgr.sendRxPDO(2));

    // Tx: Position APRD, ConfiguredAddress FPRD (+storage copy), Broadcast.
    EXPECT_TRUE(pdo_mgr.receiveTxPDO(3));
    EXPECT_TRUE(pdo_mgr.receiveTxPDO(4));
    EXPECT_EQ(pdo_mgr.mapping().get_entry(4)->storage[0], 0x7E);
    EXPECT_TRUE(pdo_mgr.receiveTxPDO(5));

    EXPECT_GT(pdo_mgr.getSlavePDORequestCount(0), 0u);
    EXPECT_GT(pdo_mgr.getSlavePDOReplyCount(0), 0u);
}

TEST_F(PDOManagerBulkTest, SingleEntryFailurePaths) {
    buildSixEntryMap();

    transport.single_send_ok_ = false;
    EXPECT_TRUE(pdo_mgr.sendRxPDO(0));   // confirmed-path failure only counts
    EXPECT_FALSE(pdo_mgr.sendRxPDO(0));  // fire-and-forget send failure
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(3));        // APRD send fails
    EXPECT_FALSE(pdo_mgr.sendRxPDO(1));           // FPWR send fails
    EXPECT_FALSE(pdo_mgr.sendRxPDO(2));           // BWR send fails
    transport.single_send_ok_ = true;

    // Response timeout / zero WKC / short data.
    transport.single_respond_ = false;
    EXPECT_TRUE(pdo_mgr.sendRxPDO(0));   // fire-and-forget: no wait at all
    EXPECT_FALSE(pdo_mgr.sendRxPDO(1));
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(4));
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(5));
    transport.single_respond_ = true;

    transport.single_resp_wkc_ = 0;
    EXPECT_FALSE(pdo_mgr.sendRxPDO(1));
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(4));
    transport.single_resp_wkc_ = 1;

    transport.single_resp_len_ = 2;               // < data_size 4
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(4));
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(5));
    transport.single_resp_len_ = 4;

    // Error counters accumulated across the failures.
    EXPECT_GT(pdo_mgr.mapping().get_entry(1)->error_count, 0u);
    EXPECT_GT(pdo_mgr.mapping().get_entry(4)->error_count, 0u);
}

TEST_F(PDOManagerBulkTest, SingleEntryDebugLogging) {
    debug_flags.rxPDO = true;
    debug_flags.txPDO = true;
    pdo_mgr.setDebugFlags(&debug_flags);
    buildSixEntryMap();
    transport.single_respond_ = true;
    transport.single_resp_wkc_ = 1;
    transport.single_resp_len_ = 4;
    std::memset(transport.single_resp_buf_, 0x5A, 4);

    EXPECT_TRUE(pdo_mgr.sendRxPDO(0));
    EXPECT_TRUE(pdo_mgr.sendRxPDO(1));
    EXPECT_TRUE(pdo_mgr.sendRxPDO(2));
    EXPECT_TRUE(pdo_mgr.receiveTxPDO(3));
    EXPECT_TRUE(pdo_mgr.receiveTxPDO(4));
    EXPECT_TRUE(pdo_mgr.receiveTxPDO(5));

    // Debug arms on the failure side too.
    transport.single_respond_ = false;
    EXPECT_FALSE(pdo_mgr.sendRxPDO(1));
    EXPECT_FALSE(pdo_mgr.receiveTxPDO(4));
}
