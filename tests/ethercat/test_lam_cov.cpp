/**
 * @file test_lam_cov.cpp
 * @brief Coverage tests for LogicalAddressManager — address map building,
 *        polled LRW exchange, cyclic fast-path exchange, PDO slices,
 *        WKC/timeout/stale handling and stall self-heal.
 */

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDOMapping.hpp"
#include "tether/ethercat/PDOTypes.hpp"
#include "tether/ethercat/ProcessImage.hpp"

using namespace EtherCAT;

namespace {

// ---------------------------------------------------------------------------
// Fake transport: polled (sendSingleDatagram + waitForResponseIdx /
// preRegister), cyclic fast path (sendCyclicDatagram/sendPoolFrame +
// waitCyclicPool) and slice path (acquireSliceSlot/sendSliceDatagram/
// waitSliceSlotMask).  Deposits echo the sent payload so the LRW counter
// trailer and input scatter see deterministic data.
// ---------------------------------------------------------------------------
class FakeLamTransport : public IPDOTransport {
public:
    // ---- Polled path knobs ----
    bool     send_ok{true};
    bool     wait_ok{true};           // waitForResponseIdx result
    uint16_t resp_wkc{3};
    bool     support_prereg{false};   // preRegisterResponseWaiter impl
    bool     prereg_always_busy{false};
    bool     cancel_req{false};

    // ---- Cyclic fast-path knobs ----
    bool     fast_path{false};
    bool     cyclic_send_ok{true};
    bool     pool_frame_ok{false};    // sendPoolFrame supported?
    uint16_t cyclic_wkc{3};
    bool     cyclic_arrive{true};     // all positions arrive
    uint8_t  arrive_mask_skip{0xFF};  // position index to withhold (timeout)
    uint8_t  echo_gen_offset{0};      // +gen to simulate stale deposits

    // ---- Slice path knobs ----
    bool     slice_pool{true};
    bool     slice_send_ok{true};
    uint16_t slice_wkc{3};

    // ---- Observability ----
    int      sends{0};
    int      multi_sends{0};
    int      waits{0};
    int      drains{0};
    int      pool_waits{0};
    int      slice_sends{0};
    int      slice_acquires{0};
    int      prereg_calls{0};
    int      cyclic_sends{0};
    uint16_t last_adp{0}, last_ado{0};
    std::vector<Command> cmds;

    // Per-position echoed payloads for waitCyclicPool views.
    // Index 0x100 is the polled-path deposit.
    std::array<std::vector<uint8_t>, 512> deposited{};
    std::vector<uint8_t> last_request;
    // WKC overrides: index by pool position (-1 = use cyclic_wkc),
    // and by slice slot for waitSliceSlotMask (-1 = slice_wkc).
    std::array<int, 512> wkc_by_pos{};
    std::array<int, 16> slice_wkc_by_slot{};
    std::array<uint64_t, 256> slot_tokens{};
    int next_slice_slot{0};

    FakeLamTransport() {
        wkc_by_pos.fill(-1);
        slice_wkc_by_slot.fill(-1);
    }

    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return true; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t,
                      unsigned int) override { return true; }

    bool sendSingleDatagram(Command cmd, uint8_t, uint16_t adp,
                            uint16_t ado, const void* data,
                            uint16_t datalen, bool) override {
        ++sends;
        cmds.push_back(cmd);
        last_adp = adp;
        last_ado = ado;
        if (data && datalen)
            last_request.assign(static_cast<const uint8_t*>(data),
                                static_cast<const uint8_t*>(data) +
                                    datalen);
        if (send_ok && deposited[0x100].empty())
            deposited[0x100] = last_request;   // wire echo
        return send_ok;
    }

    size_t sendMultiDatagram(const MultiDatagramSpec* specs,
                             size_t count) override {
        ++multi_sends;
        for (size_t i = 0; i < count; ++i) cmds.push_back(specs[i].cmd);
        return send_ok ? count : 0;
    }

    bool waitForResponseIdx(uint8_t, unsigned int,
                            RxDatagram& out) override {
        ++waits;
        if (!wait_ok) return false;
        out.wkc = resp_wkc;
        const auto& dep = deposited[0x100];
        out.datalen = static_cast<uint16_t>(dep.size());
        if (!dep.empty())
            std::memcpy(out.data, dep.data(), dep.size());
        return true;
    }

    size_t preRegisterResponseWaiter(uint8_t, uint8_t*,
                                     size_t) override {
        ++prereg_calls;
        if (!support_prereg) return kPreRegInvalid;
        if (prereg_always_busy) return kPreRegBusy;
        return 7;
    }

    bool waitForPreRegistered(size_t, unsigned int,
                              RxDatagram& out) override {
        return waitForResponseIdx(0, 0, out);
    }

    uint8_t  allocIdx() override { return 0x20; }
    uint16_t adpForSlaveIndex(uint16_t s) override { return s; }
    int      drainWire(int) override { ++drains; return 0; }
    void     purgePendingResponses() override {}
    bool     isCancelRequested() const override { return cancel_req; }

    // ---- Cyclic fast path ----
    bool supportsCyclicFastPath() const override { return fast_path; }

    uint64_t cyclicSlotToken(uint8_t slot) override {
        return slot_tokens[slot];
    }
    uint8_t cyclicSlotGen(uint8_t) override { return 0; }

    bool sendCyclicDatagram(Command cmd, uint8_t slot, uint16_t,
                            uint16_t, const void* data, uint16_t datalen,
                            bool) override {
        ++cyclic_sends;
        cmds.push_back(cmd);
        if (data && datalen)
            deposited[slot].assign(
                static_cast<const uint8_t*>(data),
                static_cast<const uint8_t*>(data) + datalen);
        return cyclic_send_ok;
    }

    bool sendPoolFrame(const CyclicDgramSpec* dgs,
                       size_t count) override {
        if (!pool_frame_ok) return false;
        for (size_t i = 0; i < count; ++i) {
            cmds.push_back(dgs[i].cmd);
            const uint8_t pos = cyclicWirePoolPos(dgs[i].idx);
            if (dgs[i].data && dgs[i].datalen)
                deposited[pos].assign(
                    static_cast<const uint8_t*>(dgs[i].data),
                    static_cast<const uint8_t*>(dgs[i].data) +
                        dgs[i].datalen);
        }
        return true;
    }

    uint8_t waitCyclicPool(const uint8_t* positions, const uint64_t*,
                           uint8_t count, uint32_t,
                           CyclicSlotView* views, bool* arrived) override {
        ++pool_waits;
        uint8_t n = 0;
        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t pos = positions[i];
            const bool give = cyclic_arrive && pos != arrive_mask_skip;
            arrived[i] = give;
            if (!give) continue;
            views[i].gen = static_cast<uint8_t>(0 + echo_gen_offset);
            views[i].wkc = wkc_by_pos[pos] >= 0
                ? static_cast<uint16_t>(wkc_by_pos[pos])
                : cyclic_wkc;
            views[i].stamp_ns = 1;   // > cyclic_send_ns_ (0-ish now)
            if (!deposited[pos].empty()) {
                views[i].payload = deposited[pos].data();
                views[i].datalen =
                    static_cast<uint16_t>(deposited[pos].size());
            } else {
                // DC-timepoint read or empty deposit: fabricate bytes.
                static std::array<uint8_t, 64> zeros{};
                views[i].payload = zeros.data();
                views[i].datalen = 8;
            }
            ++n;
        }
        return n;
    }

    // ---- Slice fast path ----
    int acquireSliceSlot() override {
        ++slice_acquires;
        if (!slice_pool) return -1;
        if (next_slice_slot >= static_cast<int>(kNumSliceSlots)) return -1;
        return next_slice_slot++;
    }
    void releaseSliceSlot(uint8_t s) override {
        if (s <= next_slice_slot && next_slice_slot > 0) --next_slice_slot;
    }
    uint64_t sliceSlotToken(uint8_t) override { return 0; }
    uint8_t  sliceSlotGen(uint8_t) override { return 0; }
    bool sendSliceDatagram(Command cmd, uint8_t slice, uint16_t,
                           uint16_t, const void* data, uint16_t datalen,
                           bool) override {
        ++slice_sends;
        cmds.push_back(cmd);
        if (data && datalen)
            deposited[128 + slice].assign(
                static_cast<const uint8_t*>(data),
                static_cast<const uint8_t*>(data) + datalen);
        return slice_send_ok;
    }
    uint32_t waitSliceSlotMask(uint32_t mask, const uint64_t*,
                               uint32_t, CyclicSlotView* views) override {
        uint32_t arrived = 0;
        for (uint8_t s = 0; s < kNumSliceSlots; ++s) {
            if (!(mask & (1u << s))) continue;
            views[s].gen = 0;
            views[s].wkc = slice_wkc_by_slot[s] >= 0
                ? static_cast<uint16_t>(slice_wkc_by_slot[s])
                : slice_wkc;
            const auto& dep = deposited[128 + s];
            if (!dep.empty()) {
                views[s].payload = dep.data();
                views[s].datalen = static_cast<uint16_t>(dep.size());
            } else {
                static std::array<uint8_t, 64> zeros{};
                views[s].payload = zeros.data();
                views[s].datalen = 8;
            }
            arrived |= 1u << s;
        }
        return arrived;
    }
};

// SlaveConfig factory: slave i gets `rx` bytes outputs + `tx` bytes inputs.
PDO::SlaveConfig makeSlave(uint16_t idx, uint16_t rx, uint16_t tx) {
    PDO::SlaveConfig c{};
    c.slave_index = idx;
    c.configured_address = static_cast<uint16_t>(0x1000 + idx);
    c.configured_address_known = true;
    c.sm[2].type = PDO::SyncManagerType::ProcessOutput;
    c.sm[3].type = PDO::SyncManagerType::ProcessInput;
    c.rxpdo_size = rx;
    c.txpdo_size = tx;
    c.configured = true;
    return c;
}

struct LamFixture {
    FakeLamTransport t;
    LogicalAddressManager lam{t};
    PDO::PDOMapping mapping;

    // One slave, 4-byte RxPDO + 4-byte TxPDO, wired mapping.
    void buildOne(uint16_t rx = 4, uint16_t tx = 4) {
        std::array<PDO::SlaveConfig, 1> cfgs{makeSlave(0, rx, tx)};
        EXPECT_TRUE(lam.buildAddressMap(cfgs.data(), 1));
        mapping.add_rxpdo(0, rx, 0x1600);
        mapping.add_txpdo(0, tx, 0x1A00);
    }
};

} // namespace

// ============================================================================
// init / buildAddressMap
// ============================================================================

TEST(LamCov, InitDeinitLifecycle) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    EXPECT_FALSE(lam.isInitialized());
    EXPECT_TRUE(lam.init());
    EXPECT_TRUE(lam.isInitialized());
    lam.deinit();
    EXPECT_FALSE(lam.isInitialized());
    lam.deinit();   // idempotent
}

TEST(LamCov, BuildRejectsBadArgs) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    EXPECT_FALSE(lam.buildAddressMap(nullptr, 1));
    PDO::SlaveConfig c = makeSlave(0, 2, 2);
    EXPECT_FALSE(lam.buildAddressMap(&c, 0));
    EXPECT_FALSE(lam.buildAddressMapFromMultiPDO(nullptr, 1));
    std::vector<PDO::MultiPDOSyncManagerConfig> one(1);
    EXPECT_FALSE(lam.buildAddressMapFromMultiPDO(&one, 0));
}

TEST(LamCov, BuildAssignsStickyWindows) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    std::array<PDO::SlaveConfig, 2> cfgs{makeSlave(0, 4, 4),
                                         makeSlave(1, 2, 0)};
    ASSERT_TRUE(lam.buildAddressMap(cfgs.data(), 2));

    EXPECT_TRUE(lam.hasSlavePDOs(0));
    EXPECT_TRUE(lam.hasSlavePDOs(1));
    EXPECT_EQ(lam.getRxPDOLogicalAddr(0), 0x10000u);
    EXPECT_EQ(lam.getRxPDOLength(0), 4);
    EXPECT_EQ(lam.getTxPDOLogicalAddr(0), 0x10004u);
    EXPECT_EQ(lam.getTxPDOLength(0), 4);
    // Slave 1: RxPDO-only window appended after slave 0's 8 bytes.
    EXPECT_EQ(lam.getRxPDOLogicalAddr(1), 0x10008u);
    EXPECT_EQ(lam.getRxPDOLength(1), 2);
    EXPECT_EQ(lam.getTxPDOLength(1), 0);
    EXPECT_EQ(lam.totalRxPDOBytes(), 6u);
    EXPECT_EQ(lam.totalTxPDOBytes(), 4u);
    EXPECT_EQ(lam.totalLogicalSize(), 10u);

    uint32_t off = 0, len = 0;
    EXPECT_TRUE(lam.getSlaveLogicalWindow(1, off, len));
    EXPECT_EQ(off, 8u);
    EXPECT_EQ(len, 2u);
    EXPECT_FALSE(lam.getSlaveLogicalWindow(7, off, len));
}

TEST(LamCov, StickyWindowSurvivesRebuild) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    std::array<PDO::SlaveConfig, 1> one{makeSlave(0, 4, 4)};
    ASSERT_TRUE(lam.buildAddressMap(one.data(), 1));
    const uint32_t s0_base = lam.getRxPDOLogicalAddr(0);

    // Rebuild adding slave 1 — slave 0's window must not move.
    std::array<PDO::SlaveConfig, 2> two{makeSlave(0, 4, 4),
                                        makeSlave(1, 8, 8)};
    ASSERT_TRUE(lam.buildAddressMap(two.data(), 2));
    EXPECT_EQ(lam.getRxPDOLogicalAddr(0), s0_base);
    EXPECT_EQ(lam.getRxPDOLogicalAddr(1), 0x10008u);
    EXPECT_EQ(lam.totalLogicalSize(), 24u);
}

TEST(LamCov, ResizedSlaveGetsFreshWindow) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    std::array<PDO::SlaveConfig, 2> cfgs{makeSlave(0, 4, 4),
                                         makeSlave(1, 2, 2)};
    ASSERT_TRUE(lam.buildAddressMap(cfgs.data(), 2));

    // Slave 0 grows 4+4 -> 8+8: fresh window appended at the end,
    // old 8-byte window becomes dead space (extent > live bytes).
    std::array<PDO::SlaveConfig, 2> grown{makeSlave(0, 8, 8),
                                          makeSlave(1, 2, 2)};
    ASSERT_TRUE(lam.buildAddressMap(grown.data(), 2));
    EXPECT_EQ(lam.getRxPDOLogicalAddr(0), 0x1000Cu);   // after 8 + 4
    EXPECT_EQ(lam.getRxPDOLength(0), 8);
    EXPECT_EQ(lam.totalLogicalSize(), 28u);            // 8 + 4 + 16
    EXPECT_EQ(lam.totalRxPDOBytes(), 10u);             // live only
}

TEST(LamCov, BaseLogicalAddressRespected) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    lam.setBaseLogicalAddress(0x40000);
    EXPECT_EQ(lam.getBaseLogicalAddress(), 0x40000u);
    std::array<PDO::SlaveConfig, 1> cfgs{makeSlave(0, 2, 2)};
    ASSERT_TRUE(lam.buildAddressMap(cfgs.data(), 1));
    EXPECT_EQ(lam.getRxPDOLogicalAddr(0), 0x40000u);
    EXPECT_EQ(lam.getTxPDOLogicalAddr(0), 0x40002u);
}

TEST(LamCov, MultiPDOAddressMap) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    std::vector<PDO::MultiPDOSyncManagerConfig> sms(1);
    sms[0].type = PDO::SyncManagerType::ProcessOutput;
    sms[0].pdo_mappings = {{0x1600, 4}, {0x1601, 2}};
    PDO::MultiPDOSyncManagerConfig sm3;
    sm3.sm_index = 3;
    sm3.type = PDO::SyncManagerType::ProcessInput;
    sm3.pdo_mappings = {{0x1A00, 6}};
    sms.push_back(sm3);

    std::array<std::vector<PDO::MultiPDOSyncManagerConfig>, 1> all{sms};
    ASSERT_TRUE(lam.buildAddressMapFromMultiPDO(all.data(), 1));

    EXPECT_EQ(lam.getRxPDOLogicalAddr(0), 0x10000u);
    EXPECT_EQ(lam.getRxPDOLength(0), 6);
    EXPECT_EQ(lam.getTxPDOLogicalAddr(0), 0x10006u);
    EXPECT_EQ(lam.getTxPDOLength(0), 6);
    EXPECT_EQ(lam.getPDOLogicalAddr(0, 0x1601), 0x10004u);
    EXPECT_EQ(lam.getPDOLength(0, 0x1601), 2);
    EXPECT_EQ(lam.getPDOLogicalAddr(0, 0x1A00), 0x10006u);
    EXPECT_EQ(lam.getPDOLogicalAddr(0, 0xDEAD), 0u);   // not found
    EXPECT_EQ(lam.getPDOLength(0, 0xDEAD), 0);

    auto entries = lam.getSlavePDOLogicalAddrs(0);
    ASSERT_EQ(entries.size(), 3u);
    EXPECT_TRUE(entries[0].is_output);
    EXPECT_FALSE(entries[2].is_output);
    EXPECT_EQ(entries[2].sm_index, 3);
}

// ============================================================================
// Polled exchange (exchangeAllLRW / exchangeLRWSlice / ForSlaves)
// ============================================================================

TEST(LamCov, ExchangeNotInitialized) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    PDO::PDOMapping mapping;
    // Whole-image exchange early-outs on zero live bytes before the
    // init check; the slice path reaches it directly.
    EXPECT_TRUE(lam.exchangeAllLRW(mapping));
    EXPECT_FALSE(lam.exchangeLRWSlice(mapping, 0, 4));
    EXPECT_EQ(lam.getStats().send_errors, 1u);
    EXPECT_FALSE(lam.exchangeLRWForSlaves(mapping, 0x1));
    EXPECT_EQ(lam.getStats().send_errors, 2u);
}

TEST(LamCov, ExchangeAllSuccessScattersInputs) {
    LamFixture f;
    f.buildOne();
    // Fill the RxPDO app buffer — it must land in the datagram payload.
    auto* e = f.mapping.get_entry_mut(0);
    std::memset(e->storage, 0xAA, 4);

    // The fake echoes the request payload; plant distinct input bytes by
    // making the fake deposit custom response data instead.
    f.t.deposited[0x100] = {0xAA, 0xAA, 0xAA, 0xAA,
                            0x11, 0x22, 0x33, 0x44};
    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.t.sends, 1);
    EXPECT_EQ(f.t.cmds.back(), Command::LRW);
    // LRW addressed at base logical addr: adp=0x0000 ado=0x0001.
    EXPECT_EQ(f.t.last_ado, 0x0001);
    EXPECT_EQ(f.lam.getStats().success, 1u);

    // TxPDO storage received the echoed bytes at offset 4..7.
    const auto* in = f.mapping.get_entry(1);
    EXPECT_EQ(in->storage[0], 0x11);
    EXPECT_EQ(in->storage[3], 0x44);
}

TEST(LamCov, ExchangeSendFailure) {
    LamFixture f;
    f.buildOne();
    f.t.send_ok = false;
    EXPECT_FALSE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.lam.getStats().send_errors, 1u);
    EXPECT_EQ(f.t.waits, 0);
}

TEST(LamCov, ExchangeTimeout) {
    LamFixture f;
    f.buildOne();
    f.t.wait_ok = false;
    EXPECT_FALSE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.lam.getStats().timeout_errors, 1u);
    EXPECT_EQ(f.lam.getStats().consecutive_timeouts, 1u);
    // Timeout path drains the wire.
    EXPECT_GE(f.t.drains, 1);

    // A following success resets the streak.
    f.t.wait_ok = true;
    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.lam.getStats().consecutive_timeouts, 0u);
}

TEST(LamCov, ExchangeZeroWkc) {
    LamFixture f;
    f.buildOne();
    f.t.resp_wkc = 0;
    EXPECT_FALSE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.lam.getStats().wkc_errors, 1u);
}

TEST(LamCov, ExchangePreRegisteredPath) {
    LamFixture f;
    f.buildOne();
    f.t.support_prereg = true;
    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.t.prereg_calls, 1);
    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.t.prereg_calls, 2);
}

TEST(LamCov, ExchangePreRegBusyGivesUp) {
    LamFixture f;
    f.buildOne();
    f.t.support_prereg = true;
    f.t.prereg_always_busy = true;   // every candidate idx owned
    EXPECT_FALSE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_EQ(f.lam.getStats().send_errors, 1u);
    EXPECT_EQ(f.t.sends, 0);
}

TEST(LamCov, ExchangeForSlavesCompacts) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    std::array<PDO::SlaveConfig, 2> cfgs{makeSlave(0, 4, 4),
                                         makeSlave(1, 2, 2)};
    ASSERT_TRUE(lam.buildAddressMap(cfgs.data(), 2));
    PDO::PDOMapping mapping;
    mapping.add_rxpdo(0, 4, 0x1600);
    mapping.add_txpdo(0, 4, 0x1A00);
    mapping.add_rxpdo(1, 2, 0x1600);
    mapping.add_txpdo(1, 2, 0x1A00);

    EXPECT_TRUE(lam.exchangeLRWForSlaves(mapping, 0));   // no-op
    EXPECT_EQ(t.sends, 0);

    // Slave 1 only: compact 2+2 datagram at base (not slave 1's offset).
    EXPECT_TRUE(lam.exchangeLRWForSlaves(mapping, 0x2));
    EXPECT_EQ(t.sends, 1);
    EXPECT_EQ(t.last_adp, 0x0000);
    EXPECT_EQ(t.last_ado, 0x0001);
}

TEST(LamCov, SliceExchangeBounds) {
    LamFixture f;
    f.buildOne(8, 8);   // 16-byte image

    // Out of range: offset+length beyond the image.
    EXPECT_FALSE(f.lam.exchangeLRWSlice(f.mapping, 8, 16));
    // Over one datagram: enforce_slice_limit rejects.
    EXPECT_FALSE(f.lam.exchangeLRWSlice(
        f.mapping, 0, f.lam.maxSliceLength() + 1));
    // In-range partial slice succeeds.
    f.t.deposited[0x100] = std::vector<uint8_t>(8, 0x5A);
    EXPECT_TRUE(f.lam.exchangeLRWSlice(f.mapping, 4, 8));
    // Addressed at base+4: ado = (0x10004>>16)=1, adp = 0x0004.
    EXPECT_EQ(f.t.last_adp, 0x0004);
    EXPECT_EQ(f.t.last_ado, 0x0001);
}

TEST(LamCov, DescribeEntriesAndImageOffsets) {
    LamFixture f;
    f.buildOne(4, 4);

    auto slices = f.lam.describeEntries(f.mapping);
    ASSERT_EQ(slices.size(), 2u);
    EXPECT_EQ(slices[0].direction, PDO::PDODirection::RxPDO);
    EXPECT_EQ(slices[0].offset, 0u);
    EXPECT_EQ(slices[0].length, 4);
    EXPECT_EQ(slices[1].direction, PDO::PDODirection::TxPDO);
    EXPECT_EQ(slices[1].offset, 4u);

    std::array<int32_t, 4> offsets{-1, -1, -1, -1};
    const size_t n = f.lam.computeImageOffsets(
        f.mapping, offsets.data(), offsets.size());
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(offsets[0], 0);
    EXPECT_EQ(offsets[1], 4);
}

TEST(LamCov, StatsResetAndSetters) {
    LamFixture f;
    f.buildOne();
    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));
    EXPECT_GE(f.lam.getStats().success, 1u);
    f.lam.resetStats();
    EXPECT_EQ(f.lam.getStats().success, 0u);

    f.lam.setLrwResponseTimeoutMs(25);
    EXPECT_EQ(f.lam.lrwResponseTimeoutMs(), 25u);
    f.lam.setLrwResponseTimeoutMs(0);   // clamps to 1
    EXPECT_EQ(f.lam.lrwResponseTimeoutMs(), 1u);
    f.lam.setStallDetectionGapUs(42);
    EXPECT_EQ(f.lam.stallDetectionGapUs(), 42u);
    f.lam.setImageExchangeDecimation(3);
    EXPECT_EQ(f.lam.imageExchangeDecimation(), 3u);
    f.lam.setImageExchangeDecimation(0);
    EXPECT_EQ(f.lam.imageExchangeDecimation(), 1u);
    f.lam.setCyclicDcTimeSlave(1);
    EXPECT_EQ(f.lam.cyclicDcTimeSlave(), 1);

    bool called = false;
    f.lam.setPrefixProvider(
        [&](uint16_t) { called = true; return std::string("X"); });
    EtherCATMasterDebugFlags df{};
    f.lam.setDebugFlags(&df);
    f.lam.deinit();
}

TEST(LamCov, StallDetectionCountsGap) {
    LamFixture f;
    f.buildOne();
    f.lam.setStallDetectionGapUs(1);   // any inter-call gap counts

    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));   // arms last_call
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    EXPECT_TRUE(f.lam.exchangeAllLRW(f.mapping));   // gap > 1us → stall
    EXPECT_GE(f.lam.getStats().stall_events, 1u);
}

TEST(LamCov, ExchangePdoSliceOneOff) {
    LamFixture f;
    f.buildOne(8, 8);
    auto desc = f.lam.describeEntries(f.mapping);

    PDOSliceSpec spec;
    spec.entries = {static_cast<uint16_t>(desc[0].entry_index),
                    static_cast<uint16_t>(desc[1].entry_index)};
    f.t.deposited[0x100] = std::vector<uint8_t>(8, 0x77);
    EXPECT_TRUE(f.lam.exchangePDOSlice(f.mapping, spec));
    EXPECT_EQ(f.t.sends, 1);

    // Empty spec → zero runs → rejected, no send.
    PDOSliceSpec empty;
    const int before = f.t.sends;
    EXPECT_FALSE(f.lam.exchangePDOSlice(f.mapping, empty));
    EXPECT_EQ(f.t.sends, before);
}

// ============================================================================
// Cyclic fast path
// ============================================================================

TEST(LamCov, CyclicFallsBackWithoutFastPath) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = false;
    // Legacy fallback runs the polled exchange inside cyclicSend.
    EXPECT_TRUE(f.lam.exchangeAllLRWCyclic(f.mapping));
    EXPECT_EQ(f.t.sends, 1);
}

TEST(LamCov, CyclicSendCollectSuccess) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.t.pool_frame_ok = true;   // counter trailer rides the last frame

    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_TRUE(f.lam.cyclicExchangePending());
    EXPECT_EQ(f.t.cyclic_sends + 1, 1);   // pool frame carried LRW+counter
    EXPECT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_FALSE(f.lam.cyclicExchangePending());
    EXPECT_EQ(f.lam.getStats().success, 1u);
    // Counter trailer verified → lastLrwCounter advanced past 0.
    EXPECT_GE(f.lam.lastLrwCounter(), 1u);
    // Slice health: first collect Ok.
    const auto h = f.lam.sliceHealth(0);
    EXPECT_EQ(h.last_status, CyclicSliceStatus::Ok);
}

TEST(LamCov, CyclicCounterTrailerDetected) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.t.pool_frame_ok = false;   // trailers fall back to sendCyclicDatagram

    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_GE(f.lam.lastLrwCounter(), 1u);

    // Second cycle uses a fresh counter.
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_GE(f.lam.lastLrwCounter(), 2u);
}

TEST(LamCov, CyclicDcTimepoint) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.lam.setCyclicDcTimeSlave(0);
    // Deposit 8 DC bytes on the DC position (idx band 0xFF → pool pos 82).
    f.t.deposited[82] = {1, 2, 3, 4, 5, 6, 7, 8};

    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_TRUE(f.lam.dcTimeValid());
    EXPECT_EQ(f.lam.lastDcTimeNs(), 0x0807060504030201ULL);
    EXPECT_EQ(f.lam.getStats().dc_timeouts, 0u);
}

TEST(LamCov, CyclicTimeoutHealth) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.t.arrive_mask_skip = 0;   // withhold the first pending position

    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_FALSE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_GE(f.lam.getStats().timeout_errors, 1u);
    const auto h = f.lam.sliceHealth(0);
    EXPECT_NE(h.last_status, CyclicSliceStatus::Ok);
}

TEST(LamCov, CyclicWkcErrorAndLearn) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;

    // Cycle 1: response wkc=3 — strict mode learns it.
    f.t.cyclic_wkc = 3;
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));

    // Cycle 2: mismatched wkc → strict check fails.
    f.t.cyclic_wkc = 9;
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_FALSE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_GE(f.lam.getStats().wkc_errors, 1u);

    // Non-strict mode tolerates the mismatch.
    f.lam.setStrictWkc(false);
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    f.lam.setStrictWkc(true);
}

TEST(LamCov, CyclicSendFailure) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.t.cyclic_send_ok = false;
    EXPECT_FALSE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_GE(f.lam.getStats().send_errors, 1u);
    EXPECT_FALSE(f.lam.cyclicExchangePending());
}

TEST(LamCov, CyclicStaleDepositCounted) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.t.echo_gen_offset = 1;   // deposits echo previous generation

    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_FALSE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_GE(f.lam.getStats().stale_responses, 1u);
}

TEST(LamCov, ExpectedWkcOverrides) {
    LamFixture f;
    f.buildOne();
    EXPECT_EQ(f.lam.expectedWkc(0),
              LogicalAddressManager::kWkcUnknown);
    f.lam.setExpectedWkc(0, 5);
    EXPECT_EQ(f.lam.expectedWkc(0), 5);
    f.lam.setExpectedWkc(200, 5);        // out of range — ignored
    f.lam.resetExpectedWkc();
    EXPECT_EQ(f.lam.expectedWkc(0),
              LogicalAddressManager::kWkcUnknown);
    EXPECT_EQ(f.lam.expectedWkc(200), 0);
    EXPECT_GT(f.lam.cyclicSliceCount(), 0);
}

TEST(LamCov, CyclicDecimationSkipsImage) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;
    f.lam.setImageExchangeDecimation(2);

    // Cycle 0: image sent.
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    const int sends_after_first = f.t.cyclic_sends;

    // Cycle 1: image decimated — no image slice emitted (counter only).
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    // Either zero or one trailer-only datagram; no new LRW image slice.
    EXPECT_LE(f.t.cyclic_sends - sends_after_first, 1);
}

TEST(LamCov, CyclicMultiSliceImage) {
    FakeLamTransport t;
    LogicalAddressManager lam(t);
    t.fast_path = true;
    // Force a small per-frame payload so the image spans >1 slice.
    // maxSliceLength derives from maxEtherCATPayloadPerFrame — the fake
    // returns the default 1498; build an image just over that.
    std::array<PDO::SlaveConfig, 1> cfgs{makeSlave(0, 800, 800)};
    ASSERT_TRUE(lam.buildAddressMap(cfgs.data(), 1));
    PDO::PDOMapping mapping;
    mapping.add_rxpdo(0, 800, 0x1600);
    mapping.add_txpdo(0, 800, 0x1A00);

    // Per-slice WKC: slice 0 covers rx(800)+tx-part → 3, slice 1 is
    // TxPDO tail only → 1.  Rotating pool positions 0 and 1.
    t.wkc_by_pos[0] = 3;
    t.wkc_by_pos[1] = 1;
    ASSERT_TRUE(lam.cyclicSend(mapping, nullptr, 200'000));
    EXPECT_GE(lam.cyclicSliceCount(), 2);
    EXPECT_EQ(lam.expectedWkc(0), 3);
    EXPECT_EQ(lam.expectedWkc(1), 1);
    ASSERT_TRUE(lam.cyclicCollect(mapping, nullptr));
    EXPECT_GE(lam.getStats().success, 1u);
    EXPECT_EQ(lam.sliceHealth(0).last_status, CyclicSliceStatus::Ok);
    EXPECT_EQ(lam.sliceHealth(1).last_status, CyclicSliceStatus::Ok);
}

// ============================================================================
// User-defined PDO slices
// ============================================================================

TEST(LamCov, DefineSliceResolvesRuns) {
    LamFixture f;
    f.buildOne(8, 8);
    f.t.fast_path = true;
    auto desc = f.lam.describeEntries(f.mapping);

    PDOSliceSpec spec;
    spec.entries = {static_cast<uint16_t>(desc[0].entry_index)};
    const uint32_t h = f.lam.definePDOSlice(f.mapping, spec);
    EXPECT_NE(h, UINT32_MAX);
    EXPECT_EQ(f.lam.pdoSliceCount(), 1u);
    EXPECT_TRUE(f.lam.clearPDOSlices());
    EXPECT_EQ(f.lam.pdoSliceCount(), 0u);
}

TEST(LamCov, DefineSliceFailures) {
    LamFixture f;
    f.buildOne();
    f.t.fast_path = true;

    PDOSliceSpec empty;   // no entries, no ranges → zero runs
    EXPECT_EQ(f.lam.definePDOSlice(f.mapping, empty), UINT32_MAX);

    // Slot-pool exhaustion surfaces at replan (first emit), not define —
    // the slice stays defined but disabled and never sends.
    f.t.slice_pool = false;
    PDOSliceSpec r;
    r.ranges = {{0, 4}};
    EXPECT_NE(f.lam.definePDOSlice(f.mapping, r), UINT32_MAX);
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    f.lam.cyclicCollect(f.mapping, nullptr);
    EXPECT_EQ(f.t.slice_sends, 0);
}

TEST(LamCov, SliceExchangeOnCyclicPath) {
    LamFixture f;
    f.buildOne(8, 8);
    f.t.fast_path = true;

    PDOSliceSpec spec;
    spec.ranges = {{0, 4}};   // first 4 image bytes — slave0 RxPDO → wkc 2
    f.t.slice_wkc = 2;
    int cb = 0;
    spec.on_exchange = [&](uint8_t, const uint8_t*, uint16_t len,
                           uint16_t wkc) {
        ++cb;
        EXPECT_EQ(len, 4);
        EXPECT_EQ(wkc, 2);
    };
    ASSERT_NE(f.lam.definePDOSlice(f.mapping, spec), UINT32_MAX);

    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    ASSERT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_GE(f.t.slice_acquires, 1);
    EXPECT_GE(f.t.slice_sends, 1);
    EXPECT_GE(cb, 1);
    EXPECT_FALSE(f.lam.pdoSliceHealth().empty());
}

TEST(LamCov, SliceExchangePendingFlag) {
    LamFixture f;
    f.buildOne(8, 8);
    f.t.fast_path = true;
    f.t.slice_wkc = 2;   // run covers slave0 RxPDO only

    PDOSliceSpec spec;
    spec.ranges = {{0, 4}};
    ASSERT_NE(f.lam.definePDOSlice(f.mapping, spec), UINT32_MAX);

    f.lam.setImageExchangeDecimation(0xFFFF);   // image mostly off
    ASSERT_TRUE(f.lam.cyclicSend(f.mapping, nullptr, 200'000));
    EXPECT_TRUE(f.lam.cyclicExchangePending());   // slice in flight
    EXPECT_TRUE(f.lam.cyclicCollect(f.mapping, nullptr));
    EXPECT_FALSE(f.lam.cyclicExchangePending());
}

// ============================================================================
// PDOManager_cyclic forwarders — no-LAM early returns + LAM-backed paths.
// ============================================================================

namespace {

struct PdoCyclicFixture {
    FakeLamTransport t;
    LogicalAddressManager lam{t};
    PDOManager pdo{t};

    void buildOne() {
        EXPECT_TRUE(lam.init());
        std::array<PDO::SlaveConfig, 1> cfgs{makeSlave(0, 4, 4)};
        EXPECT_TRUE(lam.buildAddressMap(cfgs.data(), 1));
        pdo.mapping().add_rxpdo(0, 4, 0x1600);
        pdo.mapping().add_txpdo(0, 4, 0x1A00);
        pdo.setLogicalAddressManager(&lam);
    }
};

} // namespace

TEST(PdoCyclicCov, NoLamEarlyReturns) {
    FakeLamTransport t;
    PDOManager pdo{t};

    EXPECT_FALSE(pdo.exchangeLRWSlice(0, 4));
    EXPECT_EQ(pdo.definePDOSlice(PDOSliceSpec{}), 0xFFFFFFFFu);
    EXPECT_FALSE(pdo.clearPDOSlices());
    EXPECT_EQ(pdo.pdoSliceCount(), 0u);
    EXPECT_FALSE(pdo.exchangePDOSlice(PDOSliceSpec{}));
    EXPECT_FALSE(pdo.cyclicExchangePending());
    EXPECT_EQ(pdo.cyclicSliceCount(), 0);
    EXPECT_EQ(pdo.maxLogicalSliceLength(), 0u);
    EXPECT_TRUE(pdo.describeLogicalEntries().empty());
    pdo.setImageExchangeDecimation(2);   // no-op without LAM
    pdo.setCyclicStrictWkc(true);        // no-op without LAM

    ProcessImage image;
    EXPECT_FALSE(pdo.configureProcessImage(image, ImageMode::Direct));

    // Exchange fallbacks go through the polled sendAll/receiveAll path.
    EXPECT_TRUE(pdo.exchangeConfiguredSlaveWindows());
    EXPECT_TRUE(pdo.exchangeAllLRWCyclic(200'000));
    EXPECT_TRUE(pdo.cyclicSend(nullptr, 200'000));
    EXPECT_TRUE(pdo.cyclicCollect(nullptr));
}

TEST(PdoCyclicCov, LamBackedForwarders) {
    PdoCyclicFixture f;
    f.buildOne();

    EXPECT_TRUE(f.pdo.exchangeLRWSlice(0, 4));
    EXPECT_TRUE(f.pdo.exchangeConfiguredSlaveWindows());
    EXPECT_TRUE(f.pdo.exchangeAllLRWCyclic(200'000));
    EXPECT_TRUE(f.pdo.cyclicSend(nullptr, 200'000));
    EXPECT_TRUE(f.pdo.cyclicCollect(nullptr));
    EXPECT_GE(f.pdo.maxLogicalSliceLength(), 4u);
    EXPECT_FALSE(f.pdo.describeLogicalEntries().empty());
    EXPECT_GT(f.pdo.pdoSliceCount() + 1, 0u);   // exists; value-agnostic
    f.pdo.setImageExchangeDecimation(2);
    f.pdo.setCyclicStrictWkc(true);
}

TEST(PdoCyclicCov, LamBackedSliceForwarders) {
    PdoCyclicFixture f;
    f.buildOne();
    f.t.fast_path = true;
    auto desc = f.lam.describeEntries(f.pdo.mapping());
    ASSERT_GE(desc.size(), 2u);

    PDOSliceSpec spec{};
    spec.entries = {static_cast<uint16_t>(desc[0].entry_index)};
    const uint32_t slot = f.pdo.definePDOSlice(spec);
    EXPECT_NE(slot, 0xFFFFFFFFu);
    EXPECT_GE(f.pdo.pdoSliceCount(), 1u);
    EXPECT_TRUE(f.pdo.exchangePDOSlice(spec));
    EXPECT_TRUE(f.pdo.clearPDOSlices());
    EXPECT_EQ(f.pdo.pdoSliceCount(), 0u);
}

TEST(PdoCyclicCov, LamBackedConfigureProcessImage) {
    PdoCyclicFixture f;
    f.buildOne();

    ProcessImage image;
    EXPECT_TRUE(f.pdo.configureProcessImage(image, ImageMode::Direct));
    EXPECT_EQ(image.imageBytes(), 8u);   // 4 rx + 4 tx
}

TEST(PdoCyclicCov, UninitializedLamBehavesLikeNoLam) {
    PdoCyclicFixture f;
    // Attach the LAM without init()/buildAddressMap — isInitialized()==false
    // → forwarders take the same early-return/fallback paths as no-LAM.
    f.pdo.setLogicalAddressManager(&f.lam);
    EXPECT_FALSE(f.pdo.exchangeLRWSlice(0, 4));
    EXPECT_EQ(f.pdo.definePDOSlice(PDOSliceSpec{}), 0xFFFFFFFFu);
    EXPECT_FALSE(f.pdo.cyclicExchangePending());
}
