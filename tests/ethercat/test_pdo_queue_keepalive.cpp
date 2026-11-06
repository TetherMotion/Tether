/**
 * @file test_pdo_queue_keepalive.cpp
 * @brief Coverage for raw/PDOManager_queue.cpp (queue mode: configure,
 *        enqueue/dequeue/poll, queueCycle phases, all underrun policies,
 *        events, callback-mode config) and raw/PDOKeepAlive.cpp
 *        (start/stop lifecycle).
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>

#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;

namespace {

class FakePDOTransport : public IPDOTransport {
public:
    bool send_ok = true;
    bool resp_ok = true;
    uint16_t resp_wkc = 1;
    uint16_t resp_datalen = 64;
    uint8_t resp_fill = 0x5A;
    uint8_t next_idx = 0x40;
    size_t sends = 0;

    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return true; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t,
                      unsigned int) override { return true; }
    bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t,
                            const void*, uint16_t, bool) override {
        ++sends;
        return send_ok;
    }
    size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count) override {
        for (size_t i = 0; i < count; ++i) sent_cmds.push_back(specs[i].cmd);
        ++sends;
        return send_ok ? count : 0;
    }
    std::vector<Command> sent_cmds;
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram& out) override {
        if (!resp_ok) return false;
        out.wkc = resp_wkc;
        out.datalen = resp_datalen;
        std::memset(out.data, resp_fill, sizeof(out.data));
        return true;
    }
    uint8_t allocIdx() override { return next_idx++; }
    uint16_t adpForSlaveIndex(uint16_t s) override {
        return static_cast<uint16_t>(0x1000 + s);
    }
};

// ============================================================================
// Queue mode — configuration + queue accessors
// ============================================================================

TEST(PdoQueueCov, QueueModeRejectsWrongMode) {
    FakePDOTransport t;
    PDOManager mgr(t);                       // default mode (not Queue)
    auto f = std::make_shared<PDOFrame>();
    EXPECT_FALSE(mgr.enqueueTx(0, f));
    std::shared_ptr<PDOFrame> out;
    EXPECT_FALSE(mgr.tryDequeueRx(0, out));
    std::shared_ptr<PDOEvent> ev;
    EXPECT_FALSE(mgr.tryPollEvent(ev));
    EXPECT_FALSE(mgr.queueCycle());
}

TEST(PdoQueueCov, QueueModeEnqueueDequeue) {
    FakePDOTransport t;
    PDOManager mgr(t);
    QueueModeConfig qc;
    qc.tx_queue_capacity = 4;
    mgr.configureQueueMode(qc);

    auto f = std::make_shared<PDOFrame>();
    f->data = {1, 2, 3, 4};
    EXPECT_TRUE(mgr.enqueueTx(0, f));
    EXPECT_FALSE(mgr.enqueueTx(99999, f));   // out-of-range index

    // TX queue consumed by queueCycle, not tryDequeueTx — but RX side pops:
    std::shared_ptr<PDOFrame> out;
    EXPECT_FALSE(mgr.tryDequeueRx(0, out));  // empty rx queue → false
    EXPECT_FALSE(mgr.tryDequeueRx(99999, out));
    std::shared_ptr<PDOEvent> ev;
    EXPECT_FALSE(mgr.tryPollEvent(ev));      // nothing yet
}

TEST(PdoQueueCov, QueueCapacityDropsFrames) {
    FakePDOTransport t;
    PDOManager mgr(t);
    QueueModeConfig qc;
    qc.tx_queue_capacity = 2;                // AtomicQueueB2 clamps to ≥4096
    mgr.configureQueueMode(qc);
    auto f = [] { auto x = std::make_shared<PDOFrame>(); x->data = {1}; return x; };
    // Bounded queue: pushes eventually fail. Bound generously — the B2
    // queue rounds capacity up (min 4096).
    int pushed = 0;
    while (pushed < 65536 && mgr.enqueueTx(0, f())) ++pushed;
    EXPECT_GE(pushed, 4096);
    EXPECT_LT(pushed, 65536);                // filled before the bound
}

// ============================================================================
// queueCycle — phases
// ============================================================================

TEST(PdoQueueCov, QueueCycleDrainsTxIntoStorage) {
    FakePDOTransport t;
    PDOManager mgr(t);
    QueueModeConfig qc;
    mgr.configureQueueMode(qc);

    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 4, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    auto f = std::make_shared<PDOFrame>();
    f->data = {0xAA, 0xBB, 0xCC, 0xDD};
    ASSERT_TRUE(mgr.enqueueTx(static_cast<size_t>(idx), f));

    EXPECT_TRUE(mgr.queueCycle());
    // storage got the queued frame
    const uint8_t* d = mgr.mapping().entryData(static_cast<size_t>(idx));
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d[0], 0xAA);
    EXPECT_EQ(d[3], 0xDD);
}

TEST(PdoQueueCov, QueueCycleUnderrunRepeatLast) {
    FakePDOTransport t;
    PDOManager mgr(t);
    QueueModeConfig qc;
    qc.underrun_policy = UnderrunPolicy::RepeatLastFrame;
    mgr.configureQueueMode(qc);

    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 2, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    auto f = std::make_shared<PDOFrame>();
    f->data = {0x11, 0x22};
    ASSERT_TRUE(mgr.enqueueTx(static_cast<size_t>(idx), f));
    EXPECT_TRUE(mgr.queueCycle());           // consumes frame
    EXPECT_TRUE(mgr.queueCycle());           // underrun → repeats last
    const uint8_t* d = mgr.mapping().entryData(static_cast<size_t>(idx));
    EXPECT_EQ(d[0], 0x11);                   // still the last frame
}

TEST(PdoQueueCov, QueueCycleUnderrunSafeState) {
    FakePDOTransport t;
    PDOManager mgr(t);
    QueueModeConfig qc;
    qc.underrun_policy = UnderrunPolicy::SafeState;
    qc.safe_state_buffer = {0x00, 0x00};
    mgr.configureQueueMode(qc);

    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 2, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    auto f = std::make_shared<PDOFrame>();
    f->data = {0xFF, 0xFF};
    ASSERT_TRUE(mgr.enqueueTx(static_cast<size_t>(idx), f));
    EXPECT_TRUE(mgr.queueCycle());
    EXPECT_TRUE(mgr.queueCycle());           // underrun → safe buffer
    const uint8_t* d = mgr.mapping().entryData(static_cast<size_t>(idx));
    EXPECT_EQ(d[0], 0x00);
}

TEST(PdoQueueCov, QueueCycleUnderrunSkipAndCustom) {
    FakePDOTransport t;
    PDOManager mgr(t);

    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 2, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);

    int cb = 0;
    QueueModeConfig qc;
    qc.underrun_policy = UnderrunPolicy::SkipCycle;
    mgr.configureQueueMode(qc);
    mgr.setUnderrunCallback([&](uint16_t, uint32_t) { ++cb; });
    EXPECT_TRUE(mgr.queueCycle());           // underrun → skip + re-enable
    EXPECT_GE(cb, 1);
    const auto* e = mgr.mapping().get_entry(static_cast<size_t>(idx));
    EXPECT_TRUE(e->enabled);                 // re-enabled next pass

    QueueModeConfig qc2;
    qc2.underrun_policy = UnderrunPolicy::Custom;
    mgr.configureQueueMode(qc2);             // NOTE: resets underrun_callback_
    mgr.setUnderrunCallback([&](uint16_t, uint32_t) { ++cb; });
    EXPECT_TRUE(mgr.queueCycle());
    EXPECT_GE(cb, 2);
}

TEST(PdoQueueCov, UnderrunEventQueued) {
    FakePDOTransport t;
    PDOManager mgr(t);
    QueueModeConfig qc;
    mgr.configureQueueMode(qc);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_rxpdo(0, 2, 0x1600,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.queueCycle());           // underrun → event
    std::shared_ptr<PDOEvent> ev;
    bool saw_underrun = false;
    while (mgr.tryPollEvent(ev)) {
        if (ev && ev->type == PDOEvent::Type::Underrun) saw_underrun = true;
    }
    EXPECT_TRUE(saw_underrun);
}

TEST(PdoQueueCov, RxFrameQueuedOnReceive) {
    FakePDOTransport t;
    t.resp_fill = 0x3C;
    PDOManager mgr(t);
    QueueModeConfig qc;
    mgr.configureQueueMode(qc);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    int idx = mgr.mapping().add_txpdo(0, 4, 0x1A00,
                                      PDO::PDOAddressMode::ConfiguredAddress);
    ASSERT_GE(idx, 0);
    EXPECT_TRUE(mgr.queueCycle());
    std::shared_ptr<PDOFrame> out;
    ASSERT_TRUE(mgr.tryDequeueRx(static_cast<size_t>(idx), out));
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->data[0], 0x3C);           // copied from received storage
}

// ============================================================================
// Callback mode
// ============================================================================

TEST(PdoQueueCov, CallbackModeConfigAndSetters) {
    FakePDOTransport t;
    PDOManager mgr(t);
    CallbackModeConfig cc;
    mgr.configureCallbackMode(cc);
    int tx = 0, rx = 0;
    mgr.setTxSentCallback(0, [&](uint16_t, uint32_t, uint64_t) { ++tx; });
    mgr.setRxReceivedCallback(0, [&](uint16_t, const uint8_t*, size_t,
                                   uint32_t, uint64_t) { ++rx; });
    mgr.setTxSentCallback(99999, nullptr);   // out of range → ignored
    SUCCEED();
}

// ============================================================================
// PDOKeepAlive
// ============================================================================

TEST(PdoQueueCov, KeepAliveStartStop) {
    FakePDOTransport t;
    PDOManager mgr(t);
    {
        auto ka = mgr.startKeepAlive(std::chrono::milliseconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ka->stop();
        ka->stop();                          // idempotent
    }                                        // destructor stops
    SUCCEED();
}

TEST(PdoQueueCov, KeepAliveRunsExchange) {
    FakePDOTransport t;
    PDOManager mgr(t);
    mgr.mapping().set_slave_configured_address(0, 0x2000);
    mgr.mapping().add_rxpdo(0, 2, 0x1600,
                            PDO::PDOAddressMode::ConfiguredAddress);
    auto ka = mgr.startKeepAlive(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    ka->stop();
    EXPECT_GT(t.sends, 0u);                  // worker exchanged
}

}  // namespace
