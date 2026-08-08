/**
 * @file test_pdo_modes.cpp
 * @brief Tests for PDOManager multi-mode API: split send/receive, callback, queue.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cstring>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::DoAll;
using ::testing::SetArgReferee;

// ============================================================================
// MockPDOTransport (same as test_pdo_manager.cpp)
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
// Test fixture
// ============================================================================

class PDOModesTest : public ::testing::Test {
protected:
    NiceMockTransport transport;
    PDOManager mgr{transport};

    void SetUp() override {
        ON_CALL(transport, adpForSlaveIndex(_))
            .WillByDefault([](uint16_t idx) { return static_cast<uint16_t>(0u - idx); });
        ON_CALL(transport, allocIdx())
            .WillByDefault(Return(1));
    }
};

// ============================================================================
// Mode 1: Split send/receive tests
// ============================================================================

TEST_F(PDOModesTest, DefaultModeIsDirect) {
    EXPECT_EQ(mgr.getMode(), PDOMode::Direct);
}

TEST_F(PDOModesTest, SendAllIncrementsCycleCount) {
    mgr.init();

    uint32_t rx_buf = 0xDEAD;
    mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillOnce(Return(1));

    EXPECT_TRUE(mgr.sendAll());
    EXPECT_EQ(mgr.getStats().total_cycles, 1u);
}

TEST_F(PDOModesTest, ExchangeAllCallsSendThenReceive) {
    mgr.init();

    uint32_t tx_buf = 0xAA, rx_buf = 0;
    mgr.mapping().add_rxpdo(0, &tx_buf, sizeof(tx_buf), 0x1600, PDOAddressMode::Position);
    mgr.mapping().add_txpdo(0, &rx_buf, sizeof(rx_buf), 0x1A00, PDOAddressMode::Position);

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillRepeatedly(Return(1));

    bool ok = mgr.exchangeAll();
    // May fail due to mock not providing response data, but should not crash
    EXPECT_EQ(mgr.getStats().total_cycles, 1u);
}

// ============================================================================
// Mode 3: Callback mode tests
// ============================================================================

TEST_F(PDOModesTest, ConfigureCallbackModeSetsMode) {
    mgr.configureCallbackMode();
    EXPECT_EQ(mgr.getMode(), PDOMode::Callback);
}

TEST_F(PDOModesTest, CallbackFiresOnTxSent) {
    mgr.init();
    mgr.configureCallbackMode();

    uint32_t rx_buf = 0xBEEF;
    int idx = mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);
    ASSERT_GE(idx, 0);

    std::atomic<int> callback_count{0};
    std::atomic<uint16_t> cb_slave_index{0xFFFF};

    mgr.setTxSentCallback(static_cast<size_t>(idx), [&](uint16_t slave_index,
                                                         uint32_t cycle_count,
                                                         uint64_t timestamp_ns) {
        callback_count.fetch_add(1);
        cb_slave_index.store(slave_index);
    });

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillOnce(Return(1));

    mgr.sendAll();

    EXPECT_EQ(callback_count.load(), 1);
    EXPECT_EQ(cb_slave_index.load(), 0u);
}

TEST_F(PDOModesTest, CallbackFiresOnRxReceived) {
    mgr.init();
    mgr.configureCallbackMode();

    uint32_t tx_buf = 0;
    // Use ConfiguredAddress mode so the TxPDO goes through the confirmed response path
    int idx = mgr.mapping().add_txpdo(0, &tx_buf, sizeof(tx_buf), 0x1A00, PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    std::atomic<int> callback_count{0};

    mgr.setRxReceivedCallback(static_cast<size_t>(idx), [&](uint16_t slave_index,
                                                            const uint8_t* data,
                                                            size_t size,
                                                            uint32_t cycle_count,
                                                            uint64_t timestamp_ns) {
        callback_count.fetch_add(1);
    });

    // Mock sendMultiDatagram to succeed (called in both sendAll and receiveAll)
    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillRepeatedly(Return(1));

    // Pre-registration: return valid slot handles
    EXPECT_CALL(transport, preRegisterResponseWaiter(_, _, _))
        .WillRepeatedly([](uint8_t, uint8_t*, size_t) -> size_t { return 0; });

    RxDatagram resp{};
    resp.wkc = 1;
    resp.datalen = sizeof(uint32_t);
    uint32_t resp_data = 0xCAFEBABE;
    std::memcpy(resp.data, &resp_data, sizeof(resp_data));
    EXPECT_CALL(transport, waitForPreRegistered(_, _, _))
        .WillOnce(DoAll(SetArgReferee<2>(resp), Return(true)));

    mgr.sendAll();
    mgr.receiveAll();

    EXPECT_EQ(callback_count.load(), 1);
    EXPECT_EQ(tx_buf, 0xCAFEBABEu);
}

TEST_F(PDOModesTest, CallbackDisabledByConfig) {
    mgr.init();

    CallbackModeConfig config;
    config.fire_on_tx_sent = false;
    config.fire_on_rx_received = false;
    mgr.configureCallbackMode(config);

    uint32_t rx_buf = 0xBEEF;
    int idx = mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);
    ASSERT_GE(idx, 0);

    std::atomic<int> callback_count{0};
    mgr.setTxSentCallback(static_cast<size_t>(idx), [&](uint16_t, uint32_t, uint64_t) {
        callback_count.fetch_add(1);
    });

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillOnce(Return(1));

    mgr.sendAll();

    EXPECT_EQ(callback_count.load(), 0);
}

// ============================================================================
// Mode 2: Queue mode tests
// ============================================================================

TEST_F(PDOModesTest, ConfigureQueueModeSetsMode) {
    QueueModeConfig config;
    config.tx_queue_capacity = 4;
    config.rx_queue_capacity = 4;
    config.event_queue_capacity = 8;
    mgr.configureQueueMode(config);
    EXPECT_EQ(mgr.getMode(), PDOMode::Queue);
}

TEST_F(PDOModesTest, EnqueueTxAndQueueCycle) {
    mgr.init();

    QueueModeConfig config;
    config.tx_queue_capacity = 4;
    config.rx_queue_capacity = 4;
    config.event_queue_capacity = 16;
    config.underrun_policy = UnderrunPolicy::RepeatLastFrame;
    mgr.configureQueueMode(config);

    uint32_t rx_buf = 0;
    int rx_idx = mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);
    ASSERT_GE(rx_idx, 0);

    // Enqueue a TX frame
    auto frame = std::make_shared<PDOFrame>();
    frame->data.resize(sizeof(uint32_t));
    uint32_t val = 0x12345678;
    std::memcpy(frame->data.data(), &val, sizeof(val));
    frame->cycle_count = 42;

    EXPECT_TRUE(mgr.enqueueTx(static_cast<size_t>(rx_idx), frame));

    // Mock the bus send
    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillOnce(Return(1));

    // Run one queue cycle
    bool ok = mgr.queueCycle();
    // queueCycle should succeed (send phase at least)
    EXPECT_TRUE(ok);

    // Verify data was copied to app buffer
    EXPECT_EQ(rx_buf, 0x12345678u);
}

TEST_F(PDOModesTest, QueueModeUnderrunRepeatsLastFrame) {
    mgr.init();

    QueueModeConfig config;
    config.tx_queue_capacity = 4;
    config.rx_queue_capacity = 4;
    config.event_queue_capacity = 16;
    config.underrun_policy = UnderrunPolicy::RepeatLastFrame;
    mgr.configureQueueMode(config);

    uint32_t rx_buf = 0;
    int rx_idx = mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);
    ASSERT_GE(rx_idx, 0);

    // First cycle: enqueue data
    auto frame = std::make_shared<PDOFrame>();
    frame->data.resize(sizeof(uint32_t));
    uint32_t val = 0xAABBCCDD;
    std::memcpy(frame->data.data(), &val, sizeof(val));
    mgr.enqueueTx(static_cast<size_t>(rx_idx), frame);

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillRepeatedly(Return(1));

    // First cycle consumes the queued frame
    mgr.queueCycle();
    EXPECT_EQ(rx_buf, 0xAABBCCDDu);

    // Second cycle: no new data in queue → underrun → repeat last frame
    mgr.queueCycle();
    EXPECT_EQ(rx_buf, 0xAABBCCDDu);  // Should still be the last value
}

TEST_F(PDOModesTest, QueueModeEventsGenerated) {
    mgr.init();

    QueueModeConfig config;
    config.tx_queue_capacity = 4;
    config.rx_queue_capacity = 4;
    config.event_queue_capacity = 32;
    config.enable_tx_sent_events = true;
    config.enable_rx_received_events = true;
    mgr.configureQueueMode(config);

    uint32_t rx_buf = 0;
    int rx_idx = mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);
    ASSERT_GE(rx_idx, 0);

    auto frame = std::make_shared<PDOFrame>();
    frame->data.resize(sizeof(uint32_t));
    uint32_t val = 0x11112222;
    std::memcpy(frame->data.data(), &val, sizeof(val));
    mgr.enqueueTx(static_cast<size_t>(rx_idx), frame);

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillOnce(Return(1));

    mgr.queueCycle();

    // Should have at least a TxSent event
    std::shared_ptr<PDOEvent> ev;
    bool found_tx_sent = false;
    while (mgr.tryPollEvent(ev)) {
        if (ev->type == PDOEvent::Type::TxSent) {
            found_tx_sent = true;
            EXPECT_EQ(ev->slave_index, 0u);
        }
    }
    EXPECT_TRUE(found_tx_sent);
}

TEST_F(PDOModesTest, QueueModeUnderrunEventGenerated) {
    mgr.init();

    QueueModeConfig config;
    config.tx_queue_capacity = 4;
    config.rx_queue_capacity = 4;
    config.event_queue_capacity = 32;
    config.underrun_policy = UnderrunPolicy::SkipCycle;
    mgr.configureQueueMode(config);

    uint32_t rx_buf = 0;
    mgr.mapping().add_rxpdo(0, &rx_buf, sizeof(rx_buf), 0x1600, PDOAddressMode::Position);

    // sendMultiDatagram may or may not be called (all entries may be skipped by SkipCycle)
    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillRepeatedly(Return(1));

    // No data enqueued → underrun
    mgr.queueCycle();

    std::shared_ptr<PDOEvent> ev;
    bool found_underrun = false;
    while (mgr.tryPollEvent(ev)) {
        if (ev->type == PDOEvent::Type::Underrun) {
            found_underrun = true;
        }
    }
    EXPECT_TRUE(found_underrun);
}

TEST_F(PDOModesTest, QueueModeTryDequeueRx) {
    mgr.init();

    QueueModeConfig config;
    config.tx_queue_capacity = 4;
    config.rx_queue_capacity = 4;
    config.event_queue_capacity = 16;
    mgr.configureQueueMode(config);

    uint32_t tx_buf = 0;
    // Use ConfiguredAddress mode so the TxPDO goes through the confirmed response path
    int tx_idx = mgr.mapping().add_txpdo(0, &tx_buf, sizeof(tx_buf), 0x1A00, PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(tx_idx, 0);

    EXPECT_CALL(transport, sendMultiDatagram(_, _))
        .WillRepeatedly(Return(1));

    // Pre-registration: return valid slot handles
    EXPECT_CALL(transport, preRegisterResponseWaiter(_, _, _))
        .WillRepeatedly([](uint8_t, uint8_t*, size_t) -> size_t { return 0; });

    RxDatagram resp{};
    resp.wkc = 1;
    resp.datalen = sizeof(uint32_t);
    uint32_t resp_data = 0xDEAD1234;
    std::memcpy(resp.data, &resp_data, sizeof(resp_data));
    EXPECT_CALL(transport, waitForPreRegistered(_, _, _))
        .WillOnce(DoAll(SetArgReferee<2>(resp), Return(true)));

    mgr.queueCycle();

    // Should be able to dequeue the RX frame
    std::shared_ptr<PDOFrame> rx_frame;
    EXPECT_TRUE(mgr.tryDequeueRx(static_cast<size_t>(tx_idx), rx_frame));
    ASSERT_TRUE(rx_frame != nullptr);
    ASSERT_EQ(rx_frame->data.size(), sizeof(uint32_t));
    uint32_t received_val;
    std::memcpy(&received_val, rx_frame->data.data(), sizeof(received_val));
    EXPECT_EQ(received_val, 0xDEAD1234u);
}

TEST_F(PDOModesTest, QueueModeRejectsEnqueueWhenNotConfigured) {
    mgr.init();
    auto frame = std::make_shared<PDOFrame>();
    EXPECT_FALSE(mgr.enqueueTx(0, frame));
    EXPECT_EQ(mgr.getMode(), PDOMode::Direct);
}

TEST_F(PDOModesTest, QueueCycleFailsWhenNotInQueueMode) {
    mgr.init();
    EXPECT_FALSE(mgr.queueCycle());
}
