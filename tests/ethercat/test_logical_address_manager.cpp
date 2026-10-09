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
    bool supportsCyclicFastPath() const override { return true; }
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
                stale_[s] = false;
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
    bool     stale_[kMaxScripted]{};
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
    // Single slice (16 B < 1486): s0 writes Rx +1, reads Tx +2; s1 writes
    // Rx +1 → expected WKC = 1 + 2 + 1 = 4.
    EXPECT_EQ(mgr.expectedWkc(0), 4u);
    EXPECT_EQ(mgr.expectedWkc(1), LogicalAddressManager::kWkcUnknown);
}

TEST_F(CyclicWkcTest, StrictWkcMismatchFailsCollect) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 2;   // expected 4
    transport.resp_len_[0] = 16;
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.getStats().wkc_errors, 1u);
}

TEST_F(CyclicWkcTest, CorrectWkcPassesCollect) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 4;
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
    ASSERT_EQ(mgr.expectedWkc(0), 4u);
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
    // slice0 [0,6):  s0 Rx + s0 Tx ∩ → 1 + 2 = 3
    EXPECT_EQ(mgr.expectedWkc(0), 3u);
    // slice1 [6,12): s0 Tx [4,12) ∩ only → 2
    EXPECT_EQ(mgr.expectedWkc(1), 2u);
    // slice2 [12,16): s1 Rx only → 1
    EXPECT_EQ(mgr.expectedWkc(2), 1u);
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
        transport.resp_wkc_[0] = 4;
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
    transport.slice_resp_wkc_[0] = 1;      // derived: s1 writes → +1

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
    transport.slice_resp_wkc_[0] = 3;      // s0 Rx write +1, s0 Tx read +2

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
    transport.slice_resp_wkc_[0] = 1;      // s0 writes RxPDO → +1
    transport.slice_resp_len_[1] = 4;
    transport.slice_resp_wkc_[1] = 1;      // s1 writes RxPDO → +1

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
    transport.slice_resp_wkc_[0] = 1;

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
    transport.slice_resp_len_[0] = 4;  transport.slice_resp_wkc_[0] = 1;
    transport.slice_resp_len_[1] = 4;  transport.slice_resp_wkc_[1] = 1;

    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(last_run, 1);
    EXPECT_EQ(last_len, 4u);
    EXPECT_EQ(last_wkc, 1u);
}

TEST_F(PdoSliceTest, StaleGenDroppedThenRetrySucceeds) {
    burnImageCycle();
    PDOSliceSpec spec;
    spec.entries = {2};
    ASSERT_NE(mgr.definePDOSlice(mapping, spec), kInvalid);
    mgr.setImageExchangeDecimation(1000);
    transport.slice_resp_len_[0] = 4;
    transport.slice_resp_wkc_[0] = 1;
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
    transport.slice_resp_wkc_[0] = 9;      // derived expectation is 1

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
    transport.slice_resp_wkc_[0] = 2;      // TxPDO read → +2
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
    transport.slice_resp_wkc_[0] = 1;
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
    transport.slice_resp_wkc_[0] = 1;

    // Cycle 0: image + slice both emit; both collect.
    transport.resp_len_[0] = 16;
    transport.resp_wkc_[0] = 4;
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
    transport.resp_wkc_[0] = 4;
    transport.resp_len_[0] = 16;
    ASSERT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    const auto h = mgr.sliceHealth(0);
    EXPECT_EQ(h.last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(h.expected_wkc, 4u);
    EXPECT_EQ(h.last_wkc, 4u);
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
    transport.resp_wkc_[0] = 4;
    transport.resp_len_[0] = 16;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    EXPECT_EQ(mgr.sliceHealth(0).last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(mgr.sliceHealth(0).consecutive_failures, 0u);
}

TEST_F(CyclicWkcTest, SliceHealthWkcErrorKeepsLastWkc) {
    buildTwoSlaveMap();
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    transport.resp_wkc_[0] = 2;            // expected 4 — slave dropped work
    transport.resp_len_[0] = 16;
    EXPECT_FALSE(mgr.cyclicCollect(mapping, nullptr));
    const auto h = mgr.sliceHealth(0);
    EXPECT_EQ(h.last_status, CyclicSliceStatus::WkcError);
    EXPECT_EQ(h.expected_wkc, 4u);
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
    transport.slice_resp_wkc_[0] = 1;
    ASSERT_TRUE(mgr.cyclicSend(mapping, nullptr, 1'000'000));
    EXPECT_TRUE(mgr.cyclicCollect(mapping, nullptr));
    ph = mgr.pdoSliceHealth();
    ASSERT_EQ(ph.size(), 1u);
    EXPECT_EQ(ph[0].last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(ph[0].expected_wkc, 1u);
    EXPECT_EQ(ph[0].last_wkc, 1u);
    EXPECT_EQ(ph[0].consecutive_failures, 0u);
}
