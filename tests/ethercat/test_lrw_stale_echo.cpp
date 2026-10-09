/**
 * @file test_lrw_stale_echo.cpp
 * @brief Reproduces the original LRW exchange issue end-to-end: a late
 *        response echo that outlives its request aliases into the NEXT
 *        exchange when the datagram index is reused.
 *
 * On a non-RT host (observed on Raspberry Pi) a scheduling stall breaks the
 * polled exchange like this:
 *
 *   1. exchange N sends LRW idx=A; the host stalls before the echo returns.
 *      The wait times out; the echo sits in the kernel socket backlog.
 *   2. exchange N+1 reuses idx=A (the 8-bit idx space wraps after 224
 *      allocations) and pre-registers a waiter for it.
 *   3. The RX thread catches up and routes the STALE echo — it satisfies the
 *      new waiter.  exchangeAllLRW() returns success but the process image
 *      now holds the old frame's payload: silent PDO data corruption.
 *
 * These tests drive LogicalAddressManager against a REAL TransactionRouter
 * behind a wire model (WireSimTransport): frames the slave "returns" collect
 * in a kernel-queue deque and are routed either when the caller blocks in a
 * wait (the RX thread catching up) or by drainWire() (the self-heal).  No
 * mock — the router's idx matching decides what each echo satisfies.
 *
 *   StaleEcho_AliasesReusedIdx_SilentCorruption  — the bug, pre-fix behavior
 *   SelfHeal_DrainBlocksStaleEcho                — the fix prevents it
 */
#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <deque>
#include <functional>
#include <optional>
#include <thread>

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/TransactionRouter.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;
using namespace std::chrono_literals;

// ============================================================================
// WireSimTransport — IPDOTransport over a real TransactionRouter.
//
// wire_queue_ models the kernel socket backlog: response frames "arrive" when
// sendSingleDatagram() runs them through the responder script (or when the
// test injects one via wireArrive() to model an echo landing during a stall).
// Routing happens lazily: each wait first pumps the queue through the real
// router — the equivalent of the RX thread catching up while the caller
// blocks.  drainWire() is the self-heal's explicit early pump.
// ============================================================================
class WireSimTransport : public IPDOTransport {
public:
    WireSimTransport()  { router_.init(); }
    ~WireSimTransport() override { router_.shutdown(); }

    /// Response script: called per sendSingleDatagram.  Returning a datagram
    /// makes the echo land on the wire immediately (idx is stamped with the
    /// request's idx); nullopt = response lost / arrives later (inject it
    /// manually via wireArrive()).
    std::function<std::optional<RxDatagram>(uint8_t idx)> responder;

    /// Inject a frame that physically arrived while the host/RX thread was
    /// stalled — it now sits in the kernel backlog awaiting routing.
    void wireArrive(const RxDatagram& dg) { wire_queue_.push_back(dg); }

    static RxDatagram mkEcho(uint8_t idx, uint8_t fill, uint16_t datalen) {
        RxDatagram dg{};
        dg.idx = idx;
        dg.cmd = Command::LRW;
        dg.wkc = 1;
        dg.datalen = datalen;
        std::memset(dg.data, fill, datalen);
        return dg;
    }

    // ---- IPDOTransport ----------------------------------------------------
    uint8_t allocIdx() override {
        // Small wrap (1,2,1,2,...) forces idx reuse within a few exchanges —
        // the real 224-entry space takes minutes to wrap on hardware.
        return static_cast<uint8_t>((idx_counter_++ % idx_wrap_) + 1);
    }
    uint16_t adpForSlaveIndex(uint16_t) override { return 0; }

    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return true; }
    bool readRegister(uint16_t, uint16_t, void* data, uint16_t len,
                      unsigned int) override {
        std::memset(data, 0, len);
        return true;
    }

    bool sendSingleDatagram(Command, uint8_t idx, uint16_t, uint16_t,
                            const void*, uint16_t, bool) override {
        if (responder) {
            if (auto dg = responder(idx)) wire_queue_.push_back(*dg);
        }
        return true;
    }
    size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override {
        return 0;
    }

    size_t preRegisterResponseWaiter(uint8_t idx, uint8_t* buffer,
                                     size_t buffer_size) override {
        const size_t s = router_.preRegisterWaiter(
            PacketFilter::byIndex(idx), buffer, buffer_size);
        return s == TransactionRouter::kNumSlots ? kPreRegBusy : s;
    }

    bool waitForPreRegistered(size_t slot, unsigned int timeout_ms,
                              RxDatagram& out) override {
        pumpRx();   // RX thread catches up while we block
        const WaitResult wr = router_.waitForPreRegistered(slot, timeout_ms);
        if (!wr.success) return false;
        fill(wr, out);
        return true;
    }

    bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                            RxDatagram& out) override {
        pumpRx();
        const WaitResult wr = router_.waitForPacket(
            PacketFilter::byIndex(idx), out.data, sizeof(out.data), timeout_ms);
        if (!wr.success) return false;
        fill(wr, out);
        return true;
    }

    int drainWire(int max_frames) override { return pumpRx(max_frames); }
    void purgePendingResponses() override { router_.purgeAllPending(); }
    bool isCancelRequested() const override { return false; }

    TransactionRouter& router() { return router_; }

private:
    /// Route up to max_frames queued echoes through the real router
    /// (-1 = all).  Matches Master::drainWire() semantics.
    int pumpRx(int max_frames = -1) {
        int n = 0;
        while (!wire_queue_.empty() && (max_frames < 0 || n < max_frames)) {
            router_.routePacket(wire_queue_.front());
            wire_queue_.pop_front();
            ++n;
        }
        return n;
    }

    static void fill(const WaitResult& wr, RxDatagram& out) {
        out.idx     = wr.idx;
        out.cmd     = wr.cmd;
        out.adp     = wr.adp;
        out.ado     = wr.ado;
        out.wkc     = wr.wkc;
        out.datalen = wr.data_length;
        // out.data was already filled via the pre-registered slot buffer.
    }

    TransactionRouter       router_;
    std::deque<RxDatagram>  wire_queue_;
    uint32_t                idx_counter_ = 0;
    uint8_t                 idx_wrap_ = 2;
};

// ============================================================================
// Fixture — same single-slave shape as the self-heal tests: 4 B RxPDO + 8 B
// TxPDO, logical addressing, 1 ms response budget so scripted losses time
// out fast.
// ============================================================================
class LRWStaleEchoTest : public ::testing::Test {
protected:
    WireSimTransport     transport;
    LogicalAddressManager mgr{transport};
    PDOMapping           mapping;
    int                  tx_entry = -1;

    void SetUp() override {
        mgr.init();
        mgr.setLrwResponseTimeoutMs(1);      // scripted losses time out fast
        mgr.setEscalateAfterTimeouts(0);     // keep the ring probe out of this

        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 4);
        configs[0].rxpdo_size = 4;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        mgr.buildAddressMap(configs, 1);

        mapping.add_rxpdo(0, 4, 0x1600, PDOAddressMode::Logical);
        tx_entry = mapping.add_txpdo(0, 8, 0x1A00, PDOAddressMode::Logical);
    }

    bool exchange() { return mgr.exchangeAllLRW(mapping); }

    /// TxPDO payload byte as seen by the application (storage[i]).
    uint8_t txByte(int i) const {
        return mapping.get_entry(tx_entry)->storage[i];
    }
};

// ============================================================================
// THE BUG (historical) — a late echo aliased a NEW waiter on a reused idx.
//
// Sequence:
//   exchange 1 (idx 1): echo lost → timeout.
//   exchange 2 (idx 2): clean round-trip.
//   (stall)             the stale echo 1 lands in the kernel backlog while
//                       the host/RX thread is starved.
//   exchange 3 (idx 1): pre-selective-prune this wait pumped the backlog
//                       and the STALE echo satisfied the new waiter.
//
// THE FIX — the exchange drains the wire BEFORE claiming its slot
// (claimExchangeWaiter), so the stale echo is routed while idx 1 has no
// pending waiter and drops as an unrouted stray.  Exchange 3 then times
// out honestly instead of consuming stale data — and the backlog frame
// was never a live waiter's to lose, so no pruning was needed anywhere.
// ============================================================================
TEST_F(LRWStaleEchoTest, StaleEcho_ClaimDrainBlocksAlias) {
    mgr.setStallDetectionGapUs(0);   // fix does not depend on stall detect

    int sends = 0;
    transport.responder = [&](uint8_t idx) -> std::optional<RxDatagram> {
        ++sends;
        // Echoes 1 and 3 are late — request 3's response is still in flight
        // when the stale backlog echo is flushed.
        if (sends == 1 || sends == 3) return std::nullopt;
        return WireSimTransport::mkEcho(idx, 0x11, 12);
    };

    ASSERT_FALSE(exchange());                      // 1 (idx 1): timeout
    ASSERT_TRUE(exchange());                       // 2 (idx 2): clean
    // The stale echo lands in the kernel backlog during the stall — after
    // exchange 2's wait, before exchange 3's send.
    transport.wireArrive(WireSimTransport::mkEcho(1, 0xAA, 12));

    // 3 (idx 1): the claim-time drain routes the stale echo to a dead slot
    // (dropped as unrouted) BEFORE the new waiter registers — the exchange
    // then honestly times out waiting for its own response.
    EXPECT_FALSE(exchange());
    for (int i = 0; i < 8; ++i) EXPECT_EQ(txByte(i), 0x11)
        << "stale echo aliased the reused idx — PDO data corrupted at byte " << i;

    // Request 3's real echo, whenever it finally lands, is orphaned too —
    // its waiter is already gone.
    transport.wireArrive(WireSimTransport::mkEcho(1, 0x33, 12));
    ASSERT_TRUE(exchange());                       // 4 (idx 2): fresh
    for (int i = 0; i < 8; ++i) EXPECT_EQ(txByte(i), 0x11);
    EXPECT_EQ(transport.router().getStats().packets_dropped, 2u); // stale + late
    EXPECT_EQ(mgr.getStats().timeout_errors, 2u);
}

// ============================================================================
// Companion FIX path — after a detected host stall, stallCheck drains the
// backlog (live waiters are fulfilled, dead echoes dropped) BEFORE the
// claim-time drain runs again.  Same sequence as above plus a host stall
// the manager can detect; the exchange recovers instead of timing out.
// ============================================================================
TEST_F(LRWStaleEchoTest, SelfHeal_DrainBlocksStaleEcho) {
    // 3 ms threshold: above the ~1-2 ms a timed-out exchange takes (so the
    // 1→2 gap is NOT a stall) but well below the 15 ms stall below.
    mgr.setStallDetectionGapUs(3000);

    int sends = 0;
    transport.responder = [&](uint8_t idx) -> std::optional<RxDatagram> {
        ++sends;
        if (sends == 1) return std::nullopt;
        return WireSimTransport::mkEcho(idx, 0x11, 12);
    };

    ASSERT_FALSE(exchange());                      // 1: timeout
    ASSERT_TRUE(exchange());                       // 2: clean (idx 2)
    transport.wireArrive(WireSimTransport::mkEcho(1, 0xAA, 12)); // stale echo
    std::this_thread::sleep_for(15ms);             // host stall

    ASSERT_TRUE(exchange());  // 3: stallCheck purges + drains before send

    // Fresh data only — the stale echo was dropped during the drain.
    for (int i = 0; i < 8; ++i) EXPECT_EQ(txByte(i), 0x11)
        << "stale echo leaked into the process image at byte " << i;

    const auto stats = mgr.getStats();
    EXPECT_GE(stats.stall_events, 1u);
    EXPECT_GE(stats.drained_frames, 1u);   // the flushed stale echo
    EXPECT_EQ(stats.timeout_errors, 1u);
    EXPECT_EQ(stats.consecutive_timeouts, 0u);
}

// ============================================================================
// Companion: a stale echo whose idx is never re-registered is harmless — the
// routed frame finds no pending waiter and is dropped.  Only the alias on a
// REUSED idx corrupts data.
// ============================================================================
TEST_F(LRWStaleEchoTest, StaleEcho_NoReuse_IsUnrouted) {
    mgr.setStallDetectionGapUs(0);

    transport.responder = [&](uint8_t idx) -> std::optional<RxDatagram> {
        return WireSimTransport::mkEcho(idx, 0x11, 12);
    };

    ASSERT_TRUE(exchange());   // idx 1
    ASSERT_TRUE(exchange());   // idx 2

    // Stale echo for idx 9 — no waiter will ever be pending on it.
    transport.wireArrive(WireSimTransport::mkEcho(9, 0xAA, 12));

    ASSERT_TRUE(exchange());   // idx 1 — the wait pumps the queue…
    // …but the waiter only matches idx 1: the stale frame is unrouted and the
    // fresh echo still completes the exchange.
    for (int i = 0; i < 8; ++i) EXPECT_EQ(txByte(i), 0x11);
    EXPECT_EQ(transport.router().getStats().packets_dropped, 1u);
}
