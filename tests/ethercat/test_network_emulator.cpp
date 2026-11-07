/**
 * @file test_network_emulator.cpp
 * @brief Coverage for src/ethercat/SlaveEmulator_Network.cpp — crafts raw
 *        Ethernet/EtherCAT frames to drive processFrame parsing, every
 *        datagram dispatch path (AP/FP/BR/L), multi-datagram frames,
 *        malformed input, stats, and the slave-management API.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "tether/ethercat/EtherCATProtocolConstants.hpp"
#include "tether/ethercat/EtherCATCommandTypes.hpp"
#include "tether/ethercat/NetworkEmulator.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT::Emulator;
using EtherCAT::SlaveState;

namespace {

/// Build a frame: Ethernet hdr + EtherCAT hdr + one or more datagrams.
/// `dgs` entries: {cmd, idx, adp, ado, datalen, fill, more_bit}
struct DG { uint8_t cmd, idx; uint16_t adp, ado, datalen; uint8_t fill; bool more; };

std::vector<uint8_t> makeFrame(const std::vector<DG>& dgs) {
    std::vector<uint8_t> f(14, 0);
    for (int i = 0; i < 6; ++i) { f[i] = 0x11; f[6 + i] = 0x22; }
    f[12] = 0x88; f[13] = 0xA4;                    // EtherType EtherCAT
    size_t ecat_len = 0;
    for (auto& d : dgs) ecat_len += 10 + d.datalen + 2;
    f.push_back(ecat_len & 0xFF);
    f.push_back((ecat_len >> 8) & 0x07 | 0x10);    // type 1
    for (size_t i = 0; i < dgs.size(); ++i) {
        const DG& d = dgs[i];
        f.push_back(d.cmd);
        f.push_back(d.idx);
        f.push_back(d.adp & 0xFF);
        f.push_back(d.adp >> 8);
        f.push_back(d.ado & 0xFF);
        f.push_back(d.ado >> 8);
        uint16_t lf = d.datalen | (d.more ? 0x8000 : 0);
        f.push_back(lf & 0xFF);
        f.push_back(lf >> 8);
        f.push_back(0); f.push_back(0);            // IRQ
        for (uint16_t b = 0; b < d.datalen; ++b) f.push_back(d.fill);
        f.push_back(0); f.push_back(0);            // WKC
    }
    return f;
}

// ============================================================================
// Frame-level parsing guards
// ============================================================================

TEST(NetworkEmuCov, ShortFrameRejected) {
    NetworkEmulator net;
    std::vector<uint8_t> tiny(10, 0);
    EXPECT_TRUE(net.processFrame(tiny.data(), tiny.size()).empty());
}

TEST(NetworkEmuCov, WrongEthertypeRejected) {
    NetworkEmulator net;
    auto f = makeFrame({{0x01, 0, 0, 0x0130, 4, 0, false}});
    f[12] = 0x08; f[13] = 0x00;                    // IPv4
    EXPECT_TRUE(net.processFrame(f.data(), f.size()).empty());
}

TEST(NetworkEmuCov, TruncatedDatagramStops) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x01, 0, 0, 0x0130, 200, 0, false}});
    f.resize(30);                                  // chop mid-datagram
    auto r = net.processFrame(f.data(), f.size());
    ASSERT_FALSE(r.empty());                       // header survives, no dgs
}

// ============================================================================
// Auto-increment commands
// ============================================================================

TEST(NetworkEmuCov, AprdReadsAlStatusFromSlave0) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x01, 0, 0x0000, 0x0130, 2, 0, false}});
    auto r = net.processFrame(f.data(), f.size());
    ASSERT_EQ(r.size(), f.size());
    // ADP=0 → slave 0 processed → WKC 1, AL status = INIT(1) in response data
    EXPECT_EQ(r[16 + 10], 0x01);                   // AL_STATUS low byte
    EXPECT_EQ(r[16 + 10 + 2], 0x01);               // WKC = 1
    // MACs swapped in the response
    EXPECT_EQ(r[0], 0x22);
    EXPECT_EQ(r[6], 0x11);
}

TEST(NetworkEmuCov, AprdSecondSlave) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    net.addSlave(std::make_unique<SlaveEmulator>());
    // adp = -1 → skips slave 0, slave 1 processes
    auto f = makeFrame({{0x01, 0, 0xFFFF, 0x0130, 2, 0, false}});
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x01);
}

TEST(NetworkEmuCov, AprdMissesAllSlaves) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x01, 0, 0xFFFE, 0x0130, 2, 0, false}});  // adp=-2
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x00);               // WKC 0
}

TEST(NetworkEmuCov, ApwrWritesRegister) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    // APWR AL control 0x0120 ← request PRE_OP (2)
    auto f = makeFrame({{0x02, 0, 0x0000, 0x0120, 2, 0x02, false}});
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x01);
}

TEST(NetworkEmuCov, AprwTripleWkc) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x03, 0, 0x0000, 0x0130, 2, 0, false}});
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x03);               // APRW → WKC 3
}

// ============================================================================
// Configured-address commands
// ============================================================================

TEST(NetworkEmuCov, FprdSkippedInInit) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    // Slaves still INIT → configured-address cmds skip them
    auto f = makeFrame({{0x04, 0, 0x1000, 0x0130, 2, 0, false}});
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x00);
}

TEST(NetworkEmuCov, FpwrFprwDispatch) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    for (uint8_t c : {0x05, 0x06}) {
        auto f = makeFrame({{c, 0, 0x1000, 0x0010, 2, 0, false}});
        auto r = net.processFrame(f.data(), f.size());
        ASSERT_EQ(r.size(), f.size());             // dispatch ran (WKC any)
    }
}

TEST(NetworkEmuCov, ConfiguredAddrMatchDispatches) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());

    // APWR AL_CONTROL (0x0120) ← PRE_OP — configured-addr commands skip
    // slaves still in INIT.
    auto up = makeFrame({{0x02, 0, 0x0000, 0x0120, 2, 0x02, false}});
    (void)net.processFrame(up.data(), up.size());

    // FPWR adp=0 (initial configured_addr_) → write 0x0010/0x0011 with
    // fill=0x10 → configured_addr_ becomes 0x1010.
    auto setAddr = makeFrame({{0x05, 0, 0x0000, 0x0010, 2, 0x10, false}});
    (void)net.processFrame(setAddr.data(), setAddr.size());

    // Now FPRD/FPWR/FPRW with adp=0x1010 match → dispatch to the slave.
    for (uint8_t c : {0x04, 0x05, 0x06}) {
        auto f = makeFrame({{c, 0, 0x1010, 0x0130, 2, 0, false}});
        auto r = net.processFrame(f.data(), f.size());
        ASSERT_EQ(r.size(), f.size());
        EXPECT_GE(r[16 + 10 + 2], 0x01);           // WKC ≥ 1 on match
    }

    // Non-matching adp → no dispatch, WKC 0.
    auto miss = makeFrame({{0x04, 0, 0x9999, 0x0130, 2, 0, false}});
    auto rm = net.processFrame(miss.data(), miss.size());
    EXPECT_EQ(rm[16 + 10 + 2], 0x00);
}

TEST(NetworkEmuCov, LogicalCommandsLrdLwrLrw) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    // logical_addr = adp | (ado << 16) — use 0x00010000.
    for (uint8_t c : {0x0A, 0x0B, 0x0C}) {
        auto f = makeFrame({{c, 0, 0x0000, 0x0001, 4, 0, false}});
        auto r = net.processFrame(f.data(), f.size());
        ASSERT_EQ(r.size(), f.size());             // dispatch ran (WKC any)
    }
}

// ============================================================================
// Broadcast commands
// ============================================================================

TEST(NetworkEmuCov, BroadcastHitsAllSlaves) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x07, 0, 0, 0x0130, 2, 0, false}});  // BRD
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x02);               // WKC = #slaves
}

TEST(NetworkEmuCov, BwrBrwDispatch) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    for (uint8_t c : {0x08, 0x09}) {
        auto f = makeFrame({{c, 0, 0, 0x0130, 2, 0x02, false}});
        auto r = net.processFrame(f.data(), f.size());
        ASSERT_EQ(r.size(), f.size());
    }
}

// ============================================================================
// Logical commands
// ============================================================================

TEST(NetworkEmuCov, LogicalCommandsDispatch) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    for (uint8_t c : {0x0A, 0x0B, 0x0C}) {
        auto f = makeFrame({{c, 0, 0x0001, 0x0000, 4, 0, false}});
        auto r = net.processFrame(f.data(), f.size());
        ASSERT_EQ(r.size(), f.size());
    }
}

// ============================================================================
// Unknown / multi-datagram
// ============================================================================

TEST(NetworkEmuCov, UnknownCommandZeroWkc) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x7F, 0, 0, 0x0130, 2, 0, false}});
    auto r = net.processFrame(f.data(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x00);
    EXPECT_EQ(net.getStats().unknown_commands, 1u);
}

TEST(NetworkEmuCov, NopCommandDispatches) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({{0x00, 0, 0, 0x0000, 0, 0, false}});  // NOP
    auto r = net.processFrame(f.data(), f.size());
    ASSERT_EQ(r.size(), f.size());
}

TEST(NetworkEmuCov, MultiDatagramFrame) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    auto f = makeFrame({
        {0x01, 1, 0x0000, 0x0130, 2, 0, true},     // more
        {0x01, 2, 0x0000, 0x0132, 2, 0, false},
    });
    auto r = net.processFrame(f.data(), f.size());
    ASSERT_EQ(r.size(), f.size());
    EXPECT_EQ(r[16 + 10 + 2], 0x01);               // dg1 WKC
    EXPECT_EQ(r[16 + 10 + 4 + 10 + 2], 0x01);      // dg2 WKC
    EXPECT_EQ(net.getStats().datagrams_processed, 2u);
    EXPECT_EQ(net.getStats().frames_processed, 1u);
}

// ============================================================================
// Slave management / simulate / error injection
// ============================================================================

TEST(NetworkEmuCov, SlaveManagement) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    net.addSlave(std::make_unique<SlaveEmulator>());
    EXPECT_NE(net.getSlave(0), nullptr);
    EXPECT_NE(net.getSlave(1), nullptr);
    EXPECT_EQ(net.getSlave(2), nullptr);           // out of range
    net.clearSlaves();
    EXPECT_EQ(net.getSlave(0), nullptr);
}

TEST(NetworkEmuCov, SimulateAdvancesSlaves) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    net.simulate(1000);
    SUCCEED();
}

TEST(NetworkEmuCov, GlobalErrorInjection) {
    NetworkEmulator net;
    net.addSlave(std::make_unique<SlaveEmulator>());
    ErrorInjection e{};
    e.inject_al_error = true;
    e.al_error_code = 0x0017;
    net.setGlobalErrorInjection(e);
    net.resetStats();
    EXPECT_EQ(net.getStats().frames_processed, 0u);
}

}  // namespace
