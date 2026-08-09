// Tests for the generic EventSource<T> template and the FSoEMasterConnection
// frame event sources (txFrameEvents / rxFrameEvents).

#include <gtest/gtest.h>

#include "tether/utils/EventSource.hpp"
#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoESlave.hpp"

#include <atomic>
#include <memory>
#include <vector>

using namespace FSoE;
using Tether::Utils::EventSource;

// ============================================================================
// EventSource<T> unit tests
// ============================================================================

TEST(EventSourceBasic, EmptyByDefault) {
    EventSource<std::vector<uint8_t>> es;
    EXPECT_TRUE(es.empty());
    EXPECT_EQ(es.size(), 0u);
}

TEST(EventSourceBasic, AddListenerReturnsNonZeroHandle) {
    EventSource<std::vector<uint8_t>> es;
    auto h = es.addListener([](auto) {});
    EXPECT_NE(h, 0u);
    EXPECT_FALSE(es.empty());
    EXPECT_EQ(es.size(), 1u);
}

TEST(EventSourceBasic, AddNullListenerReturnsZero) {
    EventSource<std::vector<uint8_t>> es;
    auto h = es.addListener({});
    EXPECT_EQ(h, 0u);
    EXPECT_TRUE(es.empty());
}

TEST(EventSourceBasic, RemoveListenerByHandle) {
    EventSource<std::vector<uint8_t>> es;
    auto h = es.addListener([](auto) {});
    EXPECT_TRUE(es.removeListener(h));
    EXPECT_TRUE(es.empty());
    // Removing again fails
    EXPECT_FALSE(es.removeListener(h));
}

TEST(EventSourceBasic, ClearRemovesAll) {
    EventSource<std::vector<uint8_t>> es;
    es.addListener([](auto) {});
    es.addListener([](auto) {});
    EXPECT_EQ(es.size(), 2u);
    es.clear();
    EXPECT_TRUE(es.empty());
}

// ============================================================================
// No-copy-if-no-listeners fast path
// ============================================================================

TEST(EventSourceFastPath, FactoryNotCalledWhenEmpty) {
    EventSource<std::vector<uint8_t>> es;
    bool factory_called = false;
    es.emit([&] {
        factory_called = true;
        return std::make_shared<const std::vector<uint8_t>>();
    });
    EXPECT_FALSE(factory_called);
}

TEST(EventSourceFastPath, FactoryCalledWhenListenerExists) {
    EventSource<std::vector<uint8_t>> es;
    es.addListener([](auto) {});
    bool factory_called = false;
    es.emit([&] {
        factory_called = true;
        return std::make_shared<const std::vector<uint8_t>>();
    });
    EXPECT_TRUE(factory_called);
}

// ============================================================================
// Multiple listeners share the same immutable copy
// ============================================================================

TEST(EventSourceMultiListener, AllListenersReceiveSameSharedPtr) {
    EventSource<std::vector<uint8_t>> es;
    std::shared_ptr<const std::vector<uint8_t>> p1;
    std::shared_ptr<const std::vector<uint8_t>> p2;
    es.addListener([&](auto d) { p1 = d; });
    es.addListener([&](auto d) { p2 = d; });

    const uint8_t bytes[] = {0x01, 0x02, 0x03};
    es.emit([&] {
        return std::make_shared<const std::vector<uint8_t>>(bytes, bytes + 3);
    });

    ASSERT_TRUE(p1);
    ASSERT_TRUE(p2);
    EXPECT_EQ(p1.get(), p2.get());  // same address => same shared copy
    EXPECT_EQ(*p1, (std::vector<uint8_t>{0x01, 0x02, 0x03}));
}

TEST(EventSourceMultiListener, IndividualRemoval) {
    EventSource<std::vector<uint8_t>> es;
    int calls1 = 0;
    int calls2 = 0;
    auto h1 = es.addListener([&](auto) { calls1++; });
    auto h2 = es.addListener([&](auto) { calls2++; });

    es.emit([&] { return std::make_shared<const std::vector<uint8_t>>(); });
    EXPECT_EQ(calls1, 1);
    EXPECT_EQ(calls2, 1);

    EXPECT_TRUE(es.removeListener(h1));
    es.emit([&] { return std::make_shared<const std::vector<uint8_t>>(); });
    EXPECT_EQ(calls1, 1);  // not called again
    EXPECT_EQ(calls2, 2);
}

// ============================================================================
// Listener reentrancy: a listener that (un)registers others during dispatch
// does not invalidate the iteration.
// ============================================================================

TEST(EventSourceReentrancy, ListenerRemovingAnotherDuringDispatchIsSafe) {
    EventSource<std::vector<uint8_t>> es;
    int calls2 = 0;
    EventSource<std::vector<uint8_t>>::ListenerHandle h2;

    // First listener removes the second one mid-dispatch.
    es.addListener([&](auto) { es.removeListener(h2); });
    h2 = es.addListener([&](auto) { calls2++; });

    es.emit([&] { return std::make_shared<const std::vector<uint8_t>>(); });
    // The second listener was registered before the snapshot was taken, so
    // it is still invoked once during this emit even though it was removed
    // by the first listener.
    EXPECT_EQ(calls2, 1);

    // On the next emit, only the first listener remains.
    es.emit([&] { return std::make_shared<const std::vector<uint8_t>>(); });
    EXPECT_EQ(calls2, 1);
}

// ============================================================================
// FSoEMasterConnection frame event sources
// ============================================================================

namespace {

MasterConnectionConfig makeMasterCfg(uint8_t in_size, uint8_t out_size) {
    MasterConnectionConfig cfg;
    cfg.connection_id = 0x1234;
    cfg.slave_addr = 0x100;
    cfg.slave_safety_addr = 0x100;
    cfg.master_addr = 0x100;
    cfg.input_size = in_size;
    cfg.output_size = out_size;
    cfg.fail_safe_values = {};
    return cfg;
}

FSoESlaveConfig makeSlaveCfg(uint8_t in_size, uint8_t out_size) {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 0x100;
    cfg.connectionId = 0x1234;
    cfg.safetyAddress = 0x100;
    cfg.safetyLevel = SIL::SIL2;
    cfg.watchdogTimeoutMs = 100;
    cfg.connectionTimeoutMs = 1000;
    cfg.sessionTimeoutMs = 5000;
    cfg.safeInputSize = in_size;
    cfg.safeOutputSize = out_size;
    cfg.autoRecoveryEnabled = true;
    cfg.recoveryDelayMs = 100;
    cfg.strictCrcCheck = true;
    cfg.strictSequenceCheck = true;
    cfg.treatCrcErrorAsCritical = true;
    cfg.treatSequenceErrorAsCritical = true;
    cfg.treatTimeoutAsCritical = true;
    cfg.treatConnIdErrorAsCritical = true;
    cfg.enableDiagnostics = true;
    cfg.maxErrorLogEntries = 10;
    return cfg;
}

} // namespace

TEST(FSoEFrameEvents, TxEventFiresOnPrepareTxFrame) {
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    std::shared_ptr<const std::vector<uint8_t>> captured;
    conn.txFrameEvents().addListener([&](auto d) { captured = d; });

    uint8_t buf[64] = {};
    const size_t len = conn.prepareTxFrame(buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    ASSERT_TRUE(captured);
    EXPECT_EQ(captured->size(), len);
    EXPECT_EQ(captured->front(), buf[0]);
    // First byte is the command; in Session state it is the Session command.
    EXPECT_EQ((*captured)[0], Command::Session);
}

TEST(FSoEFrameEvents, RxEventFiresOnProcessRxFrame) {
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    // Transition to Session state by building a TX frame (auto-transitions
    // out of Reset).
    uint8_t buf[64] = {};
    ASSERT_GT(conn.prepareTxFrame(buf, sizeof(buf)), 0u);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    std::shared_ptr<const std::vector<uint8_t>> captured;
    conn.rxFrameEvents().addListener([&](auto d) { captured = d; });

    // Build a Session response frame (2-byte session id payload).
    uint8_t payload[] = {0x34, 0x12};
    uint8_t frame[64];
    const size_t frame_len = CRC::buildFSoEFrame(frame, Command::Session,
                                                  payload, 2, 0x1234);
    ASSERT_TRUE(conn.processRxFrame(frame, frame_len));

    ASSERT_TRUE(captured);
    EXPECT_EQ(captured->size(), frame_len);
    EXPECT_EQ((*captured)[0], Command::Session);
    // Verify the full frame matches.
    for (size_t i = 0; i < frame_len; ++i) {
        EXPECT_EQ((*captured)[i], frame[i]) << "byte " << i;
    }
}

TEST(FSoEFrameEvents, NoListenersNoAllocation) {
    // With no listeners registered, prepareTxFrame/processRxFrame must not
    // allocate.  We verify indirectly: stats are unaffected and the call
    // succeeds.  The no-copy fast path is exercised; if it regressed, the
    // factory would still run but the result would be discarded.  We at
    // least confirm the connection still functions correctly.
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    EXPECT_TRUE(conn.txFrameEvents().empty());
    EXPECT_TRUE(conn.rxFrameEvents().empty());

    uint8_t buf[64] = {};
    const size_t len = conn.prepareTxFrame(buf, sizeof(buf));
    ASSERT_GT(len, 0u);
    EXPECT_EQ(conn.getStats().frames_sent, 1u);

    // Build a Session response to feed back.
    uint8_t payload[] = {0x34, 0x12};
    uint8_t frame[64];
    const size_t frame_len = CRC::buildFSoEFrame(frame, Command::Session,
                                                  payload, 2, 0x1234);
    ASSERT_TRUE(conn.processRxFrame(frame, frame_len));
    EXPECT_EQ(conn.getStats().frames_received, 1u);
}

TEST(FSoEFrameEvents, RxEventFiresEvenForInvalidFrame) {
    // The RX event fires before CRC validation, so listeners see every
    // received frame — including ones that will be rejected.  This is
    // intentional for diagnostics/pcap-style capture.
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    std::shared_ptr<const std::vector<uint8_t>> captured;
    conn.rxFrameEvents().addListener([&](auto d) { captured = d; });

    // A frame with a wrong connection ID — will be rejected, but the
    // listener should still see the bytes.
    uint8_t payload[] = {0x34, 0x12};
    uint8_t frame[64];
    const size_t frame_len = CRC::buildFSoEFrame(frame, Command::Session,
                                                  payload, 2, 0xFFFF);
    bool ok = conn.processRxFrame(frame, frame_len);
    EXPECT_FALSE(ok);  // rejected due to connection ID mismatch

    ASSERT_TRUE(captured);
    EXPECT_EQ(captured->size(), frame_len);
}

TEST(FSoEFrameEvents, MultipleTxListenersAllFire) {
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    std::atomic<int> calls{0};
    conn.txFrameEvents().addListener([&](auto) { calls++; });
    conn.txFrameEvents().addListener([&](auto) { calls++; });

    uint8_t buf[64] = {};
    ASSERT_GT(conn.prepareTxFrame(buf, sizeof(buf)), 0u);
    EXPECT_EQ(calls.load(), 2);
}

TEST(FSoEFrameEvents, RemoveTxListenerStopsCalls) {
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    int calls = 0;
    auto h = conn.txFrameEvents().addListener([&](auto) { calls++; });

    uint8_t buf[64] = {};
    ASSERT_GT(conn.prepareTxFrame(buf, sizeof(buf)), 0u);
    EXPECT_EQ(calls, 1);

    EXPECT_TRUE(conn.txFrameEvents().removeListener(h));
    ASSERT_GT(conn.prepareTxFrame(buf, sizeof(buf)), 0u);
    EXPECT_EQ(calls, 1);  // not called after removal
}

TEST(FSoEFrameEvents, TxAndRxAreIndependent) {
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    int tx_calls = 0;
    int rx_calls = 0;
    conn.txFrameEvents().addListener([&](auto) { tx_calls++; });
    conn.rxFrameEvents().addListener([&](auto) { rx_calls++; });

    // TX only
    uint8_t buf[64] = {};
    ASSERT_GT(conn.prepareTxFrame(buf, sizeof(buf)), 0u);
    EXPECT_EQ(tx_calls, 1);
    EXPECT_EQ(rx_calls, 0);

    // RX only (Session response)
    uint8_t payload[] = {0x34, 0x12};
    uint8_t frame[64];
    const size_t frame_len = CRC::buildFSoEFrame(frame, Command::Session,
                                                  payload, 2, 0x1234);
    ASSERT_TRUE(conn.processRxFrame(frame, frame_len));
    EXPECT_EQ(tx_calls, 1);
    EXPECT_EQ(rx_calls, 1);
}

TEST(FSoEFrameEvents, FullHandshakeEmitsFrames) {
    // Drive the master through the full Session→Connection→Parameter→Data
    // handshake with a real FSoESlave and count the emitted TX/RX frames.
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    ASSERT_TRUE(conn.initialize());
    conn.startConnection();

    FSoESlave slave(makeSlaveCfg(2, 2));
    ASSERT_TRUE(slave.initialize());

    int tx_calls = 0;
    int rx_calls = 0;
    conn.txFrameEvents().addListener([&](auto) { tx_calls++; });
    conn.rxFrameEvents().addListener([&](auto) { rx_calls++; });

    uint64_t now = 0;
    // Session
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);
    // Connection
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    ASSERT_EQ(conn.getState(), ConnectionState::Parameter);
    // Parameter
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    ASSERT_EQ(conn.getState(), ConnectionState::Data);
    // Data
    now += 15;
    ASSERT_TRUE(conn.exchangeWith(slave, now));
    ASSERT_EQ(conn.getState(), ConnectionState::Data);

    // 4 exchanges => 4 TX frames and 4 RX frames.
    EXPECT_EQ(tx_calls, 4);
    EXPECT_EQ(rx_calls, 4);
    EXPECT_EQ(conn.getStats().frames_sent, 4u);
    EXPECT_EQ(conn.getStats().frames_received, 4u);
}
