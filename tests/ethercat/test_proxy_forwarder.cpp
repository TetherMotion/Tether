/**
 * @file test_proxy_forwarder.cpp
 * @brief Tests for the external-source PDO forwarding path
 *        ("EtherCAT proxy").
 *
 * Coverage:
 *  - ProxyForwardConfig validation via Master::setProxyForwarding().
 *  - allocIdx() skips the configured band while armed, never otherwise.
 *  - frameIsInBandPdo(): PDO-only + in-band validation over synthesized
 *    frames, incl. multi-datagram chains and VLAN tags.
 *  - Composed async BPF (encap ∧ idx∉fastpath ∧ idx∉proxy-band):
 *    userspace cBPF interpreter + real kernel execution on an
 *    AF_UNIX socketpair.
 *  - Proxy-side in-band / monitor programs via cbpfExecute.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOMappingConfig.hpp"
#include "tether/ethercat/ProxyForwardConfig.hpp"
#include "tether/ethercat/Types.hpp"
#include "ethercat/raw/ProxyForwarder.hpp"
#include "ethercat/raw/WireBpfSpec.hpp"

#ifdef __linux__
#include <linux/filter.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

using namespace EtherCAT;

namespace {

// ============================================================================
// Frame synthesis — [eth 14][+tag 4][ecat hdr 2][dg hdr 10][data][wkc]×n
// ============================================================================

struct DgSpec {
    uint8_t cmd;
    uint8_t idx;
    uint16_t datalen = 0;
    uint32_t adr = 0;                    // logical address (L* commands)
    std::vector<uint8_t> data;           // payload — overrides datalen
};

std::vector<uint8_t> makeFrame(std::initializer_list<DgSpec> dgs,
                               bool tagged = false, uint16_t vid = 1)
{
    std::vector<uint8_t> f(64, 0);   // min ethernet frame
    size_t off = 12;
    if (tagged) {
        f[off]     = 0x81; f[off + 1] = 0x00;         // TPID
        f[off + 2] = static_cast<uint8_t>(vid >> 8);
        f[off + 3] = static_cast<uint8_t>(vid & 0xFF);
        off += 4;
        f[off]     = 0x88; f[off + 1] = 0xA4;         // inner EtherType
    } else {
        f[off]     = 0x88; f[off + 1] = 0xA4;
    }
    off += 2;                                        // EtherType done

    auto dgLen = [](const DgSpec& d) {
        return static_cast<uint16_t>(
            d.data.empty() ? d.datalen : d.data.size());
    };
    uint16_t ec_len = 0;
    for (const DgSpec& d : dgs)
        ec_len = static_cast<uint16_t>(ec_len + 10 + dgLen(d) + 2);
    const size_t ecat = off;                          // frame header pos
    f[off]     = static_cast<uint8_t>(ec_len & 0xFF);
    f[off + 1] = static_cast<uint8_t>((ec_len >> 8) | (1u << 4));  // type=1
    off += 2;

    size_t i = 0;
    for (const DgSpec& d : dgs) {
        const bool last = (++i == dgs.size());
        const uint16_t dl = dgLen(d);
        f[off]     = d.cmd;
        f[off + 1] = d.idx;
        f[off + 2] = static_cast<uint8_t>(d.adr & 0xFF);          // ADP lo
        f[off + 3] = static_cast<uint8_t>((d.adr >> 8) & 0xFF);   // ADP hi
        f[off + 4] = static_cast<uint8_t>((d.adr >> 16) & 0xFF);  // ADO lo
        f[off + 5] = static_cast<uint8_t>((d.adr >> 24) & 0xFF);  // ADO hi
        const uint16_t lf = static_cast<uint16_t>(
            dl | (last ? 0 : 0x8000));
        f[off + 6] = static_cast<uint8_t>(lf & 0xFF);
        f[off + 7] = static_cast<uint8_t>(lf >> 8);
        if (!d.data.empty())
            std::memcpy(f.data() + off + 10, d.data.data(), dl);
        off += 10 + dl + 2;                          // hdr + data + wkc
    }
    f.resize(std::max<size_t>(off, 60));
    (void)ecat;
    return f;
}

constexpr uint8_t kLRD = static_cast<uint8_t>(Command::LRD);  // 0x0A
constexpr uint8_t kLWR = static_cast<uint8_t>(Command::LWR);  // 0x0B
constexpr uint8_t kLRW = static_cast<uint8_t>(Command::LRW);  // 0x0C
constexpr uint8_t kFPRD = static_cast<uint8_t>(Command::FPRD);
constexpr uint8_t kAPWR = static_cast<uint8_t>(Command::APWR);

// Proxy band used throughout: [0x40, 0x4F].
constexpr uint8_t kLo = 0x40, kHi = 0x4F;

} // namespace

// ============================================================================
// Config validation / runtime API
// ============================================================================

TEST(ProxyConfig, ValidationRejectsBadBands) {
    Master m;
    Master::ProxyForwardConfig c;
    c.enabled   = true;
    c.ifname    = "lo";
    c.idx_base  = kLo;
    c.idx_count = 0;
    EXPECT_FALSE(m.setProxyForwarding(c));            // zero count

    c.idx_count = 16;
    c.idx_base  = 0x99;                               // end 0xA8 ≥ 0x9C
    EXPECT_FALSE(m.setProxyForwarding(c));

    c.idx_base  = 0x9C;                               // entirely cyclic
    c.idx_count = 4;
    EXPECT_FALSE(m.setProxyForwarding(c));

    c.idx_base  = 0;
    c.idx_count = 0x9C;                               // whole async space
    EXPECT_FALSE(m.setProxyForwarding(c));

    c.idx_base  = kLo;
    c.idx_count = 16;
    c.ifname.clear();
    EXPECT_FALSE(m.setProxyForwarding(c));            // no interface name
}

TEST(ProxyConfig, ValidConfigStoredWhileStopped) {
    Master m;
    EXPECT_FALSE(m.isProxyForwardingEnabled());
    Master::ProxyForwardConfig c;
    c.enabled   = true;
    c.ifname    = "lo";
    c.idx_base  = kLo;
    c.idx_count = 16;
    EXPECT_TRUE(m.setProxyForwarding(c));
    EXPECT_TRUE(m.proxyForwardingConfig().enabled);
    EXPECT_EQ(m.proxyForwardingConfig().idx_base, kLo);
    EXPECT_EQ(m.proxyForwardingConfig().end(), kHi);
    EXPECT_FALSE(m.isProxyForwardingEnabled());       // not started yet

    m.clearProxyForwarding();
    EXPECT_FALSE(m.proxyForwardingConfig().enabled);
}

// ============================================================================
// allocIdx band exclusion
// ============================================================================

TEST(ProxyIdxAlloc, DisabledAllocatorCoversWholeAsyncRange) {
    Master m;
    bool saw[kFastSlotBaseIdx] = {};
    for (int i = 0; i < 400; ++i) {
        const uint8_t idx = m.allocIdx();
        ASSERT_LT(idx, kFastSlotBaseIdx);
        saw[idx] = true;
    }
    // 400 draws over 156 usable indexes — every async idx cycles through.
    for (uint8_t i = 0; i < kFastSlotBaseIdx; ++i)
        EXPECT_TRUE(saw[i]) << "idx 0x" << std::hex << (int)i << " never allocated";
}

TEST(ProxyIdxAlloc, ArmedBandIsSkipped) {
    Master m;
    Master::ProxyForwardConfig c;
    c.enabled   = true;
    c.ifname    = "lo";
    c.idx_base  = kLo;
    c.idx_count = 16;
    ASSERT_TRUE(m.setProxyForwarding(c));

    for (int i = 0; i < 400; ++i) {
        const uint8_t idx = m.allocIdx();
        ASSERT_LT(idx, kFastSlotBaseIdx);
        EXPECT_FALSE(idx >= kLo && idx <= kHi)
            << "allocated proxy idx 0x" << std::hex << (int)idx;
    }

    m.clearProxyForwarding();
    bool saw_band = false;
    for (int i = 0; i < 400; ++i) {
        const uint8_t idx = m.allocIdx();
        saw_band |= (idx >= kLo && idx <= kHi);
    }
    EXPECT_TRUE(saw_band) << "band never allocated after disable";
}

// ============================================================================
// frameIsInBandPdo — userspace validation
// ============================================================================

TEST(ProxyValidation, SinglePdoDatagrams) {
    EXPECT_TRUE(ProxyForwarder::frameIsInBandPdo(
        makeFrame({{kLRW, kLo, 8}}).data(), 60, kLo, kHi));
    EXPECT_TRUE(ProxyForwarder::frameIsInBandPdo(
        makeFrame({{kLRD, kHi, 0}}).data(), 60, kLo, kHi));
    EXPECT_TRUE(ProxyForwarder::frameIsInBandPdo(
        makeFrame({{kLWR, 0x45, 32}}).data(), 60, kLo, kHi));
}

TEST(ProxyValidation, RejectsNonPdoCommands) {
    for (uint8_t cmd : {kFPRD, kAPWR, (uint8_t)0x00, (uint8_t)0x0D}) {
        const auto f = makeFrame({{cmd, 0x45, 4}});
        EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(
            f.data(), f.size(), kLo, kHi)) << "cmd 0x" << std::hex << (int)cmd;
    }
}

TEST(ProxyValidation, RejectsOutOfBandIdx) {
    const auto lo = makeFrame({{kLRW, (uint8_t)(kLo - 1), 4}});
    const auto hi = makeFrame({{kLRW, (uint8_t)(kHi + 1), 4}});
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(
        lo.data(), lo.size(), kLo, kHi));
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(
        hi.data(), hi.size(), kLo, kHi));
}

TEST(ProxyValidation, MultiDatagramChain) {
    const auto good = makeFrame({{kLRW, kLo, 8}, {kLRD, 0x4A, 4},
                                 {kLWR, kHi, 16}});
    EXPECT_TRUE(ProxyForwarder::frameIsInBandPdo(
        good.data(), good.size(), kLo, kHi));

    // First dg in-band (BPF passes) but second dg out-of-band → reject.
    const auto mixed = makeFrame({{kLRW, kLo, 8}, {kLRW, 0x10, 4}});
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(
        mixed.data(), mixed.size(), kLo, kHi));

    // Non-PDO second datagram → reject.
    const auto mixed_cmd = makeFrame({{kLRW, kLo, 8}, {kFPRD, 0x4A, 4}});
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(
        mixed_cmd.data(), mixed_cmd.size(), kLo, kHi));
}

TEST(ProxyValidation, MalformedChainsRejected) {
    // M bit set on the last datagram — chain truncated.
    auto f = makeFrame({{kLRW, kLo, 4}});
    f[16 + 7] |= 0x80;          // lenFlags hi byte (dg off +7) M bit
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(f.data(), f.size(), kLo, kHi));

    // Datagram length overrun — dg claims more bytes than ec_len holds.
    auto g = makeFrame({{kLRW, kLo, 4}});
    g[16 + 6] = 0xFF;           // lenFlags lo byte → datalen 255 > ec_len
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(g.data(), g.size(), kLo, kHi));

    // Truncated frame / non-ECAT / empty.
    const std::vector<uint8_t> tiny(20, 0);
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(tiny.data(), tiny.size(), kLo, kHi));
    auto h = makeFrame({{kLRW, kLo, 4}});
    h[12] = 0x08; h[13] = 0x00;  // IPv4
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(h.data(), h.size(), kLo, kHi));
}

TEST(ProxyValidation, VlanTaggedFrames) {
    const auto good = makeFrame({{kLRW, 0x45, 8}}, /*tagged=*/true, /*vid=*/42);
    EXPECT_TRUE(ProxyForwarder::frameIsInBandPdo(
        good.data(), good.size(), kLo, kHi));
    const auto bad = makeFrame({{kFPRD, 0x45, 8}}, true, 42);
    EXPECT_FALSE(ProxyForwarder::frameIsInBandPdo(
        bad.data(), bad.size(), kLo, kHi));
}

// ============================================================================
// Composed BPF — userspace interpreter verdicts
// ============================================================================

namespace {

/// Async-socket spec under proxy: encap ∧ idx∉fastpath ∧ idx∉band.
std::vector<CBPFInsn> asyncProxyProg(const WireEncap& we, uint8_t lo,
                                     uint8_t hi)
{
    CBPFSpec spec = wireEncapBpfSpec(we);
    spec.first_idx_range   = fastPathIdxRange();
    spec.first_idx_exclude = true;
    spec.first_idx_deny    = CBPFIdxRange{lo, hi};
    return CBPFProgramFactory::build(spec);
}

std::vector<uint8_t> ecatDgram(uint8_t idx, bool tagged = false,
                             uint16_t et = 0x88A4)
{
    std::vector<uint8_t> d(64, 0);
    if (tagged) {
        d[12] = 0x81; d[13] = 0x00;
        d[16] = static_cast<uint8_t>(et >> 8);
        d[17] = static_cast<uint8_t>(et & 0xFF);
        d[21] = idx;
    } else {
        d[12] = static_cast<uint8_t>(et >> 8);
        d[13] = static_cast<uint8_t>(et & 0xFF);
        d[17] = idx;
    }
    return d;
}

} // namespace

TEST(ProxyBpf, AsyncComposedProgramVerdicts) {
    const WireEncap we{};   // untagged direct EtherCAT
    const auto prog = asyncProxyProg(we, kLo, kHi);
    ASSERT_FALSE(prog.empty());

    auto accepts = [&](uint8_t idx, bool tagged = false,
                       uint16_t et = 0x88A4) {
        const auto d = ecatDgram(idx, tagged, et);
        return cbpfExecute(prog, d.data(), d.size()) != 0;
    };

    // Async traffic — outside fastpath, outside the band.
    EXPECT_TRUE(accepts(0x00));
    EXPECT_TRUE(accepts(0x10));
    EXPECT_TRUE(accepts(0x3F));
    EXPECT_TRUE(accepts(0x50));
    EXPECT_TRUE(accepts(0x9B));
    EXPECT_TRUE(accepts(0xFE));   // fire-and-forget hole stays async

    // Proxy band — denied.
    EXPECT_FALSE(accepts(kLo));
    EXPECT_FALSE(accepts(0x45));
    EXPECT_FALSE(accepts(kHi));

    // Cyclic fastpath — still excluded.
    EXPECT_FALSE(accepts(0x9C));
    EXPECT_FALSE(accepts(0xE0));
    EXPECT_FALSE(accepts(0xFF));

    // Same verdicts for inline-tagged frames.
    for (uint8_t idx : {0x10, 0x45, 0x9C, 0xFE})
        EXPECT_EQ(accepts(idx), accepts(idx, true))
            << "idx 0x" << std::hex << (int)idx;

    // Non-EtherCAT rejected (composed program, encap semantics).
    EXPECT_FALSE(accepts(0x10, false, 0x0800));
}

TEST(ProxyBpf, ProxySidePrograms) {
    CBPFSpec ps;
    ps.first_idx_range = CBPFIdxRange{kLo, kHi};
    const auto inband = CBPFProgramFactory::build(ps);
    ps.first_idx_exclude = true;
    const auto mon = CBPFProgramFactory::build(ps);
    ASSERT_FALSE(inband.empty());
    ASSERT_FALSE(mon.empty());

    auto verdict = [&](const std::vector<CBPFInsn>& p, uint8_t idx,
                       bool tagged = false) {
        const auto d = ecatDgram(idx, tagged);
        return cbpfExecute(p, d.data(), d.size()) != 0;
    };

    // In-band forwarder: only the band.
    EXPECT_TRUE (verdict(inband, kLo));
    EXPECT_TRUE (verdict(inband, kHi));
    EXPECT_TRUE (verdict(inband, 0x45, true));
    EXPECT_FALSE(verdict(inband, 0x3F));
    EXPECT_FALSE(verdict(inband, 0x50));
    EXPECT_FALSE(verdict(inband, 0x9C));

    // Monitor: the complement — anything NOT in the band.
    EXPECT_FALSE(verdict(mon, kLo));
    EXPECT_FALSE(verdict(mon, 0x45));
    EXPECT_TRUE (verdict(mon, 0x10));   // async idx → counted + logged
    EXPECT_TRUE (verdict(mon, 0x9C));   // cyclic idx → counted + logged
    EXPECT_TRUE (verdict(mon, 0x9C, true));
    EXPECT_TRUE (verdict(mon, 0xFE));
}

// ============================================================================
// ProxyPdoFilter — address whitelist + value constraints
// ============================================================================

namespace {

ProxyPdoFilter acl(std::initializer_list<ProxyPdoAccessRule> a,
                   std::initializer_list<ProxyPdoValueRule> v = {}) {
    ProxyPdoFilter f;
    f.access.assign(a.begin(), a.end());
    f.values.assign(v.begin(), v.end());
    return f;
}

/// Write frame: LWR datagram at logical `adr` carrying `data`.
std::vector<uint8_t> lwrFrame(uint32_t adr, std::vector<uint8_t> data) {
    return makeFrame({{kLWR, 0x45, 0, adr, std::move(data)}});
}
std::vector<uint8_t> lrdFrame(uint32_t adr, uint16_t dlen) {
    return makeFrame({{kLRD, 0x45, dlen, adr}});
}
std::vector<uint8_t> lrwFrame(uint32_t adr, std::vector<uint8_t> data) {
    return makeFrame({{kLRW, 0x45, 0, adr, std::move(data)}});
}

/// Convenience — one-stop validation with an armed filter.
bool pass(const std::vector<uint8_t>& f, const ProxyPdoFilter& flt) {
    return ProxyForwarder::framePassesFilter(
        f.data(), f.size(), kLo, kHi, &flt);
}

} // namespace

TEST(ProxyFilter, ValidationRejectsBadRules) {
    std::string err;
    EXPECT_TRUE(acl({{0x1000, 8, true, true}}).validate(err));

    EXPECT_FALSE(acl({{0x1000, 0, true, false}}).validate(err)); // len 0
    EXPECT_FALSE(acl({{0x1000, 8, false, false}}).validate(err));// no caps
    EXPECT_FALSE(acl({{0xFFFFFFF0u, 0x20, true, false}})
                 .validate(err));                              // wraps 4G

    EXPECT_FALSE(acl({{0x1000, 8, false, true}},
                     {{0x1000, 0, 1, 0}}).validate(err));        // width 0
    EXPECT_FALSE(acl({{0x1000, 8, false, true}},
                     {{0x1000, 9, 1, 0}}).validate(err));        // width 9
    EXPECT_FALSE(acl({{0x1000, 8, false, true}},
                     {{0x1000, 2, 0x1FFFF, 0}}).validate(err));  // mask > width
    EXPECT_FALSE(acl({{0x1000, 8, false, true}},
                     {{0x1000, 2, 0xFF00, 0x00FF}})
                 .validate(err));                              // expected ∉ mask
}

TEST(ProxyFilter, ConfigRejectsInvalidFilter) {
    Master m;
    Master::ProxyForwardConfig c;
    c.enabled   = true;
    c.ifname    = "lo";
    c.idx_base  = kLo;
    c.idx_count = 16;
    c.filter    = acl({{0x1000, 8, false, false}});   // grants nothing
    EXPECT_FALSE(m.setProxyForwarding(c));
}

TEST(ProxyFilter, WriteWindows) {
    const auto flt = acl({{0x1000, 16, false, true}});
    EXPECT_TRUE (pass(lwrFrame(0x1000, {1, 2, 3, 4}), flt));
    EXPECT_TRUE (pass(lwrFrame(0x100C, {1, 2, 3, 4}), flt));   // tail of window
    EXPECT_FALSE(pass(lwrFrame(0x0FFC, {1, 2, 3, 4}), flt));   // before window
    EXPECT_FALSE(pass(lwrFrame(0x100D, {1, 2, 3, 4}), flt));   // crosses top
    EXPECT_FALSE(pass(lwrFrame(0x2000, {1}), flt));            // outside
}

TEST(ProxyFilter, ReadNeedsReadCapability) {
    const auto wr_only = acl({{0x1000, 16, false, true}});
    const auto rd      = acl({{0x1000, 16, true, false}});

    // LRD datagrams may only read `read` windows.
    EXPECT_FALSE(pass(lrdFrame(0x1000, 4), wr_only));
    EXPECT_TRUE (pass(lrdFrame(0x1000, 4), rd));

    // LWR datagrams conversely need `write` coverage.
    EXPECT_FALSE(pass(lwrFrame(0x1000, {1, 2, 3, 4}), rd));
}

TEST(ProxyFilter, LrwNeedsBothCapabilities) {
    const auto rd_only = acl({{0x1000, 16, true, false}});
    const auto wr_only = acl({{0x1000, 16, false, true}});
    const auto rw      = acl({{0x1000, 16, true, true}});
    // Union coverage: read rule + write rule together grant LRW too.
    const auto split   = acl({{0x1000, 16, true, false},
                              {0x1000, 16, false, true}});

    EXPECT_FALSE(pass(lrwFrame(0x1000, {1, 2}), rd_only));
    EXPECT_FALSE(pass(lrwFrame(0x1000, {1, 2}), wr_only));
    EXPECT_TRUE (pass(lrwFrame(0x1000, {1, 2}), rw));
    EXPECT_TRUE (pass(lrwFrame(0x1000, {1, 2}), split));
}

TEST(ProxyFilter, SpanCoverageFromAdjacentWindows) {
    // Union coverage [0x1000,0x1010) from two adjacent windows.
    const auto flt = acl({{0x1000, 8, false, true},
                          {0x1008, 8, false, true}});
    // Span straddling the seam → covered.
    EXPECT_TRUE(pass(lwrFrame(0x1004, {1, 2, 3, 4, 5, 6, 7, 8}), flt));
    // Right at the union's edge (end-exclusive) → covered.
    EXPECT_TRUE(pass(lwrFrame(0x1006,
                              {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}), flt));
    // One byte past the union → reject.
    EXPECT_FALSE(pass(lwrFrame(0x1006,
                               {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}), flt));
    // Hole between windows → reject.
    const auto holed = acl({{0x1000, 8, false, true},
                            {0x1010, 8, false, true}});
    EXPECT_FALSE(pass(lwrFrame(0x1006,
                               {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}), holed));
}

TEST(ProxyFilter, EmptyFilterDeniesEverything) {
    const ProxyPdoFilter empty{};
    EXPECT_FALSE(pass(lwrFrame(0x1000, {1}), empty));
    EXPECT_FALSE(pass(lrdFrame(0x1000, 1), empty));
}

TEST(ProxyFilter, ValueCellExactMatch) {
    const auto flt = acl({{0x1000, 16, false, true}},
                         {{0x1004, 2, 0xFFFF, 0x0006}});
    // Cell [0x1004,+2) must equal 0x0006 — datagram writes [0x1000,+8).
    EXPECT_TRUE (pass(lwrFrame(0x1000, {0, 0, 0, 0, 0x06, 0x00, 9, 9}), flt));
    EXPECT_FALSE(pass(lwrFrame(0x1000, {0, 0, 0, 0, 0x07, 0x00, 9, 9}), flt));
}

TEST(ProxyFilter, ValueCellAlternativesOr) {
    // Enumerated whitelist {0x0006, 0x0009} on the same cell.
    const auto flt = acl({{0x1000, 8, false, true}},
                         {{0x1000, 2, 0xFFFF, 0x0006},
                          {0x1000, 2, 0xFFFF, 0x0009}});
    EXPECT_TRUE (pass(lwrFrame(0x1000, {0x06, 0x00, 0, 0}), flt));
    EXPECT_TRUE (pass(lwrFrame(0x1000, {0x09, 0x00, 0, 0}), flt));
    EXPECT_FALSE(pass(lwrFrame(0x1000, {0x05, 0x00, 0, 0}), flt));
}

TEST(ProxyFilter, ValueCellMaskDontCare) {
    // Only the high byte is pinned: 0x06xx with any low byte.
    const auto flt = acl({{0x1000, 8, false, true}},
                         {{0x1000, 2, 0xFF00, 0x0600}});
    EXPECT_TRUE (pass(lwrFrame(0x1000, {0xAA, 0x06, 1, 2}), flt));
    EXPECT_TRUE (pass(lwrFrame(0x1000, {0x00, 0x06, 1, 2}), flt));
    EXPECT_FALSE(pass(lwrFrame(0x1000, {0xAA, 0x07, 1, 2}), flt));
}

TEST(ProxyFilter, PartialCellCoverageRejected) {
    const auto flt = acl({{0x1000, 16, false, true}},
                         {{0x1004, 4, 0xFFFFFFFFu, 0xDEADBEEF}});
    // Datagram covers [0x1002,0x1006) — the cell's upper half (0x1006,
    // 0x1007) is outside the datagram → reject; a fragment cannot
    // prove the constrained value.
    EXPECT_FALSE(pass(lwrFrame(0x1002, {0xAA, 0xAA, 0xEF, 0xBE}), flt));
    // Full coverage with the right value → pass.
    EXPECT_TRUE (pass(lwrFrame(0x1000,
                               {0, 0, 0, 0, 0xEF, 0xBE, 0xAD, 0xDE}), flt));
}

TEST(ProxyFilter, UnoverlappedCellSkipped) {
    // Constraint lives at 0x2000 — a write to 0x1000 never touches it.
    const auto flt = acl({{0x1000, 8, false, true},
                          {0x2000, 8, false, true}},
                         {{0x2000, 4, 0xFFFFFFFFu, 0x1}});
    EXPECT_TRUE (pass(lwrFrame(0x1000, {9, 9, 9, 9}), flt));
}

TEST(ProxyFilter, ReadsSkipValueConstraints) {
    // Value rules constrain writes only — reads of covered windows pass.
    const auto flt = acl({{0x1000, 8, true, true}},
                         {{0x1000, 2, 0xFFFF, 0x0006}});
    EXPECT_TRUE(pass(lrdFrame(0x1000, 4), flt));
}

TEST(ProxyFilter, NoFilterKeepsPdoOnly) {
    const auto f = lwrFrame(0xDEADBEEF, {1, 2, 3, 4});
    EXPECT_TRUE(ProxyForwarder::framePassesFilter(
        f.data(), f.size(), kLo, kHi, nullptr));
}

TEST(ProxyFilter, MultiDatagramEachChecked) {
    const auto flt = acl({{0x1000, 8, true, true},
                          {0x2000, 8, true, true}});
    const auto good = makeFrame({{kLWR, kLo, 4, 0x1000},
                                 {kLRD, 0x4A, 4, 0x2008 - 4}});
    EXPECT_TRUE(pass(good, flt));

    // Second datagram escapes the whitelist → whole frame rejected.
    const auto bad = makeFrame({{kLWR, kLo, 4, 0x1000},
                                {kLRD, 0x4A, 4, 0x9000}});
    EXPECT_FALSE(pass(bad, flt));
}

// ============================================================================
// Auto-generation from the PDO map — Master::buildProxyPdoFilter()
// ============================================================================

namespace {

/// Populate the master's default LAM with a single-PDO map:
/// slave 0 → RxPDO 8B @ 0x10000, TxPDO 12B @ 0x10008.
void buildSinglePdoMap(Master& m) {
    PDO::SlaveConfig cfgs[PDO::kMaxPDOSlaves] = {};
    cfgs[0].configured   = true;
    cfgs[0].sm[2] = PDO::SyncManagerConfig::process_output(0x1800, 8);
    cfgs[0].rxpdo_size = 8;
    cfgs[0].sm[3] = PDO::SyncManagerConfig::process_input(0x1C00, 12);
    cfgs[0].txpdo_size = 12;
    ASSERT_TRUE(m.logicalAddressManager().buildAddressMap(cfgs, 1));
}

/// Populate the default LAM with a two-PDO-per-direction multi map:
/// slave 0 → out {0x1600:8B, 0x1601:4B} @ 0x10000, in {0x1A00:16B} @ 0x1000C.
void buildMultiPdoMap(Master& m) {
    std::vector<std::vector<PDO::MultiPDOSyncManagerConfig>> cfgs(1);
    cfgs[0].push_back(PDO::MultiPDOSyncManagerConfig::process_output(
        0x1800, {{0x1600, 8}, {0x1601, 4}}));
    cfgs[0].push_back(PDO::MultiPDOSyncManagerConfig::process_input(
        0x1C00, {{0x1A00, 16}}));
    ASSERT_TRUE(m.logicalAddressManager()
                .buildAddressMapFromMultiPDO(cfgs.data(), 1));
}

} // namespace

TEST(ProxyAutogen, EmptyMapYieldsDenyAll) {
    Master m;
    const auto flt = m.buildProxyPdoFilter();
    EXPECT_TRUE(flt.access.empty());
    EXPECT_FALSE(pass(lwrFrame(0x10000, {1}), flt));
}

TEST(ProxyAutogen, SinglePdoMapWindows) {
    Master m;
    buildSinglePdoMap(m);
    const auto flt = m.buildProxyPdoFilter();
    ASSERT_EQ(flt.access.size(), 2u);

    // Default bidirectional: every window grants read+write — LRW over
    // the whole slave image passes; anything outside does not.
    const auto img = lrwFrame(0x10000,
                              {1, 2, 3, 4, 5, 6, 7, 8,           // Rx region
                               9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9}); // Tx
    EXPECT_TRUE(pass(img, flt));
    EXPECT_FALSE(pass(lwrFrame(0x10014, {1}), flt));          // past image
    EXPECT_FALSE(pass(lwrFrame(0x0FFFC, {1, 2, 3, 4}), flt)); // before image
}

TEST(ProxyAutogen, DirectionalSplit) {
    Master m;
    buildSinglePdoMap(m);
    ProxyPdoSelection sel;
    sel.bidirectional = false;
    const auto flt = m.buildProxyPdoFilter(sel);
    ASSERT_EQ(flt.access.size(), 2u);

    // Write to the RxPDO (output) region → OK; read → denied.
    EXPECT_TRUE (pass(lwrFrame(0x10000, {1, 2, 3, 4}), flt));
    EXPECT_FALSE(pass(lrdFrame(0x10000, 4), flt));
    // Read from the TxPDO (input) region → OK; write → denied.
    EXPECT_TRUE (pass(lrdFrame(0x10008, 4), flt));
    EXPECT_FALSE(pass(lwrFrame(0x10008, {1, 2, 3, 4}), flt));
    // LRW spanning both regions is denied under direction split.
    EXPECT_FALSE(pass(lrwFrame(0x10000,
                               {1, 2, 3, 4, 5, 6, 7, 8,
                                9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9}),
                      flt));
}

TEST(ProxyAutogen, RestrictToSlaves) {
    Master m;
    buildSinglePdoMap(m);
    ProxyPdoSelection sel;
    sel.slaves = {1};                       // slave 1 has no map
    EXPECT_TRUE(m.buildProxyPdoFilter(sel).access.empty());
    sel.slaves = {0};
    EXPECT_EQ(m.buildProxyPdoFilter(sel).access.size(), 2u);
}

TEST(ProxyAutogen, RestrictToPdoIndexes) {
    Master m;
    buildMultiPdoMap(m);
    ProxyPdoSelection sel;
    sel.pdo_indexes = {0x1600};
    auto flt = m.buildProxyPdoFilter(sel);
    ASSERT_EQ(flt.access.size(), 1u);
    EXPECT_EQ(flt.access[0].addr, 0x10000u);
    EXPECT_EQ(flt.access[0].len, 8u);

    // An LWR to the excluded 0x1601 region fails.
    EXPECT_FALSE(pass(lwrFrame(0x10008, {1, 2, 3, 4}), flt));
    EXPECT_TRUE (pass(lwrFrame(0x10000, {1, 2, 3, 4}), flt));
}

TEST(ProxyAutogen, RestrictToDirection) {
    Master m;
    buildSinglePdoMap(m);
    ProxyPdoSelection sel;
    sel.include_inputs = false;
    const auto flt = m.buildProxyPdoFilter(sel);
    ASSERT_EQ(flt.access.size(), 1u);
    EXPECT_EQ(flt.access[0].addr, 0x10000u);   // only the RxPDO window
    EXPECT_FALSE(pass(lrdFrame(0x10008, 4), flt));
}

TEST(ProxyAutogen, SelectionRoundTripsThroughConfig) {
    // pdo_selection is honoured by applyProxyConfig — verifiable here
    // only via the config round-trip (the forwarder needs real NICs).
    Master m;
    Master::ProxyForwardConfig c;
    c.enabled   = true;
    c.ifname    = "lo";
    c.idx_base  = kLo;
    c.idx_count = 16;
    c.pdo_selection.slaves = {3, 5};
    c.pdo_selection.include_inputs = false;
    c.pdo_selection.bidirectional  = false;
    ASSERT_TRUE(m.setProxyForwarding(c));
    const auto stored = m.proxyForwardingConfig();
    EXPECT_EQ(stored.pdo_selection.slaves.size(), 2u);
    EXPECT_FALSE(stored.pdo_selection.include_inputs);
    EXPECT_FALSE(stored.pdo_selection.bidirectional);
}

#ifdef __linux__
// ============================================================================
// Real kernel BPF execution — SO_ATTACH_FILTER on AF_UNIX SOCK_DGRAM.
// ============================================================================

TEST(ProxyBpfKernel, UnixSocketpairRunsAsyncProxyProgram) {
    int sv[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    const auto prog = asyncProxyProg(WireEncap{}, kLo, kHi);
    ASSERT_FALSE(prog.empty());
    sock_fprog fp{static_cast<unsigned short>(prog.size()),
                  reinterpret_cast<sock_filter*>(
                      const_cast<CBPFInsn*>(prog.data()))};
    ASSERT_EQ(setsockopt(sv[1], SOL_SOCKET, SO_ATTACH_FILTER,
                       &fp, sizeof(fp)), 0)
        << "kernel refused composed async+proxy BPF: " << strerror(errno);

    auto sendDgram = [&](uint8_t idx, uint16_t et = 0x88A4) {
        const auto d = ecatDgram(idx, false, et);
        ASSERT_EQ(send(sv[0], d.data(), d.size(), 0),
                  (ssize_t)d.size());
    };
    auto received = [&]() -> bool {
        uint8_t buf[2048];
        pollfd pfd{sv[1], POLLIN, 0};
        if (poll(&pfd, 1, 50) <= 0) return false;
        return recv(sv[1], buf, sizeof(buf), 0) > 0;
    };

    sendDgram(0x10);  EXPECT_TRUE(received());    // async
    sendDgram(0x9B);  EXPECT_TRUE(received());    // async ceiling
    sendDgram(0xFE);  EXPECT_TRUE(received());    // fire-and-forget
    sendDgram(kLo);   EXPECT_FALSE(received());   // proxy band
    sendDgram(0x45);  EXPECT_FALSE(received());   // proxy band
    sendDgram(kHi);   EXPECT_FALSE(received());   // proxy band
    sendDgram(0x50);  EXPECT_TRUE(received());    // just past the band
    sendDgram(0x9C);  EXPECT_FALSE(received());   // cyclic fastpath
    sendDgram(0xFF);  EXPECT_FALSE(received());   // DC idx → cyclic
    sendDgram(0x10, 0x0800); EXPECT_FALSE(received()); // non-ECAT

    close(sv[0]); close(sv[1]);
}

TEST(ProxyBpfKernel, UnixSocketpairRunsProxySidePrograms) {
    int sv[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    CBPFSpec ps;
    ps.first_idx_range = CBPFIdxRange{kLo, kHi};
    const auto inband = CBPFProgramFactory::build(ps);
    sock_fprog fp{static_cast<unsigned short>(inband.size()),
                  reinterpret_cast<sock_filter*>(
                      const_cast<CBPFInsn*>(inband.data()))};
    ASSERT_EQ(setsockopt(sv[1], SOL_SOCKET, SO_ATTACH_FILTER,
                       &fp, sizeof(fp)), 0);

    auto sendDgram = [&](uint8_t idx, bool tagged = false) {
        const auto d = ecatDgram(idx, tagged);
        ASSERT_EQ(send(sv[0], d.data(), d.size(), 0), (ssize_t)d.size());
    };
    auto received = [&]() -> bool {
        uint8_t buf[2048];
        pollfd pfd{sv[1], POLLIN, 0};
        if (poll(&pfd, 1, 50) <= 0) return false;
        return recv(sv[1], buf, sizeof(buf), 0) > 0;
    };

    sendDgram(kLo);        EXPECT_TRUE(received());
    sendDgram(0x45);       EXPECT_TRUE(received());
    sendDgram(0x45, true); EXPECT_TRUE(received());   // tagged in-band
    sendDgram(0x3F);       EXPECT_FALSE(received());
    sendDgram(0x9C);       EXPECT_FALSE(received());
    sendDgram(0x10, true); EXPECT_FALSE(received());  // tagged out-of-band

    close(sv[0]); close(sv[1]);
}
#endif // __linux__
