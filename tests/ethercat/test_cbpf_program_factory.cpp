/**
 * @file test_cbpf_program_factory.cpp
 * @brief Tests for CBPFProgramFactory: generated programs, the user-space
 *        interpreter (kernel cBPF semantics), and a real kernel attach over
 *        an AF_UNIX socketpair (no CAP_NET_RAW required).
 */

#include <gtest/gtest.h>

#include "tether/ethercat/CBPFProgramFactory.hpp"

#include <cstdint>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace EtherCAT;
namespace cbpf = EtherCAT::cbpf;

namespace {

// ============================================================================
// Packet builders (Ethernet frames as they arrive on AF_PACKET)
// ============================================================================

void put16(std::vector<uint8_t>& f, size_t off, uint16_t v) {
    f[off]     = static_cast<uint8_t>(v >> 8);
    f[off + 1] = static_cast<uint8_t>(v & 0xFF);
}

/// Ethernet header + given EtherType + payload bytes.
std::vector<uint8_t> ethFrame(uint16_t ethertype,
                              std::vector<uint8_t> payload = {}) {
    std::vector<uint8_t> f(14 + payload.size(), 0);
    for (size_t i = 0; i < 6; ++i) f[i] = 0xFF;          // dst
    for (size_t i = 6; i < 12; ++i) f[i] = 0x11;         // src
    put16(f, 12, ethertype);
    std::copy(payload.begin(), payload.end(), f.begin() + 14);
    return f;
}

/// 802.1Q frame: eth + TPID + TCI(vid) + inner EtherType + payload.
std::vector<uint8_t> vlanFrame(uint16_t vid, uint16_t inner_ethertype,
                               std::vector<uint8_t> payload = {},
                               uint16_t tpid = kEtherType8021Q) {
    std::vector<uint8_t> f(18 + payload.size(), 0);
    for (size_t i = 0; i < 6; ++i) f[i] = 0xFF;
    for (size_t i = 6; i < 12; ++i) f[i] = 0x11;
    put16(f, 12, tpid);
    put16(f, 14, vid & 0x0FFF);                        // TCI (PCP/DEI = 0)
    put16(f, 16, inner_ethertype);
    std::copy(payload.begin(), payload.end(), f.begin() + 18);
    return f;
}

struct UdpOpts {
    uint16_t dst_port = kEtherCATUdpPort;
    uint16_t src_port = 4444;
    uint8_t  proto    = 17;        // UDP
    uint8_t  ihl      = 5;         // 32-bit words
    uint16_t frag     = 0x4000;    // DF set, offset 0
};

/// IPv4 header bytes (ihl*4), no Ethernet.
std::vector<uint8_t> ipv4Header(const UdpOpts& o, uint16_t payload_len) {
    // Build a full 20-byte header first, then size it to ihl*4 bytes.
    // IHL < 5 produces a deliberately truncated header (fields past the
    // truncation point are absent from the wire packet); IHL > 5 appends
    // zeroed option bytes.
    std::vector<uint8_t> ip(20, 0);
    ip[0] = static_cast<uint8_t>(0x40 | (o.ihl & 0x0F));
    put16(ip, 2, static_cast<uint16_t>(o.ihl * 4 + 8 + payload_len));
    put16(ip, 6, o.frag);
    ip[8] = 64;
    ip[9] = o.proto;
    ip[12] = 192; ip[13] = 168; ip[14] = 0; ip[15] = 1;
    ip[16] = 192; ip[17] = 168; ip[18] = 0; ip[19] = 2;
    ip.resize(static_cast<size_t>(o.ihl) * 4);
    return ip;
}

std::vector<uint8_t> udpHeader(const UdpOpts& o, uint16_t payload_len) {
    std::vector<uint8_t> u(8, 0);
    put16(u, 0, o.src_port);
    put16(u, 2, o.dst_port);
    put16(u, 4, static_cast<uint16_t>(8 + payload_len));
    return u;
}

/// eth + IPv4 + UDP frame.
std::vector<uint8_t> udpFrame(const UdpOpts& o = {},
                              std::vector<uint8_t> payload = {}) {
    auto ip = ipv4Header(o, static_cast<uint16_t>(payload.size()));
    auto u  = udpHeader(o, static_cast<uint16_t>(payload.size()));
    std::vector<uint8_t> body;
    body.insert(body.end(), ip.begin(), ip.end());
    body.insert(body.end(), u.begin(), u.end());
    body.insert(body.end(), payload.begin(), payload.end());
    return ethFrame(kEtherTypeIPv4, std::move(body));
}

/// eth + 802.1Q + IPv4 + UDP frame.
std::vector<uint8_t> vlanUdpFrame(uint16_t vid, const UdpOpts& o = {},
                                 std::vector<uint8_t> payload = {}) {
    auto ip = ipv4Header(o, static_cast<uint16_t>(payload.size()));
    auto u  = udpHeader(o, static_cast<uint16_t>(payload.size()));
    std::vector<uint8_t> body;
    body.insert(body.end(), ip.begin(), ip.end());
    body.insert(body.end(), u.begin(), u.end());
    body.insert(body.end(), payload.begin(), payload.end());
    return vlanFrame(vid, kEtherTypeIPv4, std::move(body));
}

std::vector<uint8_t> ecatPayload(size_t n = 32) {
    return std::vector<uint8_t>(n, 0xA5);
}

bool accepted(const std::vector<CBPFInsn>& prog,
              const std::vector<uint8_t>& pkt) {
    return cbpfExecute(prog, pkt.data(), pkt.size()) != 0;
}

CBPFInsn insn(uint16_t code, uint8_t jt, uint8_t jf, uint32_t k) {
    return CBPFInsn{code, jt, jf, k};
}

} // namespace

// ============================================================================
// Interpreter — instruction coverage
// ============================================================================

TEST(CBpfVmTest, ReturnConstant) {
    std::vector<CBPFInsn> p = {insn(cbpf::RET | cbpf::K, 0, 0, 0x1234)};
    uint8_t pkt[4] = {};
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 0x1234u);
}

TEST(CBpfVmTest, ReturnA) {
    std::vector<CBPFInsn> p = {
        insn(cbpf::LD | cbpf::IMM, 0, 0, 77),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    uint8_t pkt[4] = {};
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 77u);
}

TEST(CBpfVmTest, LoadsAreBigEndian) {
    uint8_t pkt[] = {0x00, 0x11, 0x22, 0x33, 0x44};
    std::vector<CBPFInsn> b = {
        insn(cbpf::LD | cbpf::B | cbpf::ABS, 0, 0, 2),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    std::vector<CBPFInsn> h = {
        insn(cbpf::LD | cbpf::H | cbpf::ABS, 0, 0, 1),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    std::vector<CBPFInsn> w = {
        insn(cbpf::LD | cbpf::W | cbpf::ABS, 0, 0, 0),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    EXPECT_EQ(cbpfExecute(b, pkt, sizeof(pkt)), 0x22u);
    EXPECT_EQ(cbpfExecute(h, pkt, sizeof(pkt)), 0x1122u);
    EXPECT_EQ(cbpfExecute(w, pkt, sizeof(pkt)), 0x00112233u);
}

TEST(CBpfVmTest, OutOfBoundsLoadDrops) {
    uint8_t pkt[] = {0x00, 0x11};
    std::vector<CBPFInsn> p = {
        insn(cbpf::LD | cbpf::H | cbpf::ABS, 0, 0, 10),  // past end
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 0u);
    // halfword straddling the end
    std::vector<CBPFInsn> q = {
        insn(cbpf::LD | cbpf::H | cbpf::ABS, 0, 0, 1),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    EXPECT_EQ(cbpfExecute(q, pkt, 1), 0u);
}

TEST(CBpfVmTest, LenModeAndInd) {
    uint8_t pkt[] = {0x25, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    // X = 4*(pkt[0]&0xF) = 0x14... pkt[0]=0x25 → ihl nibble 5 → X=20: beyond
    // packet. Use pkt[0]=0x21 → X=4 → IND ldh [X+1] = pkt[5,6] = 0xDDEE.
    pkt[0] = 0x21;
    std::vector<CBPFInsn> p = {
        insn(cbpf::LDX | cbpf::MSH | cbpf::B, 0, 0, 0),
        insn(cbpf::LD | cbpf::H | cbpf::IND, 0, 0, 1),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 0xDDEEu);

    std::vector<CBPFInsn> lp = {
        insn(cbpf::LD | cbpf::LEN, 0, 0, 0),
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    EXPECT_EQ(cbpfExecute(lp, pkt, sizeof(pkt)), 8u);
}

TEST(CBpfVmTest, AluOps) {
    auto run = [](std::vector<CBPFInsn> p, uint8_t* pkt, size_t n) {
        p.push_back(insn(cbpf::RET | cbpf::A, 0, 0, 0));
        return cbpfExecute(p, pkt, n);
    };
    uint8_t pkt[] = {0x05};

    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,10),
                   insn(cbpf::ALU|cbpf::ADD|cbpf::K,0,0,5)}, pkt, 1), 15u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,10),
                   insn(cbpf::ALU|cbpf::SUB|cbpf::K,0,0,15)}, pkt, 1),
              0xFFFFFFFBu);                                     // wraps
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,6),
                   insn(cbpf::ALU|cbpf::MUL|cbpf::K,0,0,7)}, pkt, 1), 42u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,42),
                   insn(cbpf::ALU|cbpf::DIV|cbpf::K,0,0,6)}, pkt, 1), 7u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,1),
                   insn(cbpf::ALU|cbpf::DIV|cbpf::K,0,0,0)}, pkt, 1), 0u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,0xFF),
                   insn(cbpf::ALU|cbpf::AND|cbpf::K,0,0,0x0F)}, pkt, 1), 0x0Fu);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,0xF0),
                   insn(cbpf::ALU|cbpf::OR|cbpf::K,0,0,0x0F)}, pkt, 1), 0xFFu);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,1),
                   insn(cbpf::ALU|cbpf::LSH|cbpf::K,0,0,4)}, pkt, 1), 16u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,0x100),
                   insn(cbpf::ALU|cbpf::RSH|cbpf::K,0,0,4)}, pkt, 1), 0x10u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,5),
                   insn(cbpf::ALU|cbpf::NEG,0,0,0)}, pkt, 1), 0xFFFFFFFBu);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,0xF0),
                   insn(cbpf::ALU|cbpf::XOR|cbpf::K,0,0,0xFF)}, pkt, 1), 0x0Fu);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,13),
                   insn(cbpf::ALU|cbpf::MOD|cbpf::K,0,0,5)}, pkt, 1), 3u);
    EXPECT_EQ(run({insn(cbpf::LD|cbpf::IMM,0,0,13),
                   insn(cbpf::ALU|cbpf::MOD|cbpf::K,0,0,0)}, pkt, 1), 0u);
}

TEST(CBpfVmTest, ScratchMemoryAndMisc) {
    uint8_t pkt[] = {0x07};
    // A=7 → M[0]; A=0 → X=10 via IMM; A=X (TXA); A+=M[0]? (no — MEM is LD only)
    std::vector<CBPFInsn> p = {
        insn(cbpf::LD | cbpf::B | cbpf::ABS, 0, 0, 0),   // A = 7
        insn(cbpf::ST, 0, 0, 0),                          // M[0] = 7
        insn(cbpf::LDX | cbpf::IMM, 0, 0, 3),             // X = 3
        insn(cbpf::STX, 0, 0, 1),                         // M[1] = 3
        insn(cbpf::MISC | cbpf::TXA, 0, 0, 0),            // A = X = 3
        insn(cbpf::LDX | cbpf::MEM, 0, 0, 0),             // X = M[0] = 7
        insn(cbpf::ALU | cbpf::ADD | cbpf::X, 0, 0, 0),   // A = 3 + 7
        insn(cbpf::MISC | cbpf::TAX, 0, 0, 0),            // X = A = 10
        insn(cbpf::LD | cbpf::MEM, 0, 0, 1),              // A = 3
        insn(cbpf::ALU | cbpf::ADD | cbpf::X, 0, 0, 0),   // A = 13
        insn(cbpf::RET | cbpf::A, 0, 0, 0),
    };
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 13u);
}

TEST(CBpfVmTest, JumpSemantics) {
    uint8_t pkt[] = {0x88, 0xA4};
    // if (pkt[0..1] == 0x88A4) return 1 else return 2
    std::vector<CBPFInsn> p = {
        insn(cbpf::LD | cbpf::H | cbpf::ABS, 0, 0, 0),
        insn(cbpf::JMP | cbpf::JEQ | cbpf::K, 1, 0, 0x88A4), // jt skips RET 2
        insn(cbpf::RET | cbpf::K, 0, 0, 2),
        insn(cbpf::RET | cbpf::K, 0, 0, 1),
    };
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 1u);
    pkt[1] = 0x00;
    EXPECT_EQ(cbpfExecute(p, pkt, sizeof(pkt)), 2u);

    // JA skips two instructions
    std::vector<CBPFInsn> q = {
        insn(cbpf::JMP | cbpf::JA | cbpf::K, 0, 0, 2),   // pc += 2
        insn(cbpf::RET | cbpf::K, 0, 0, 9),
        insn(cbpf::RET | cbpf::K, 0, 0, 9),
        insn(cbpf::RET | cbpf::K, 0, 0, 5),
    };
    EXPECT_EQ(cbpfExecute(q, pkt, sizeof(pkt)), 5u);
}

TEST(CBpfVmTest, JsetAndJgtJge) {
    uint8_t pkt[] = {0x10};
    std::vector<CBPFInsn> s = {
        insn(cbpf::LD | cbpf::B | cbpf::ABS, 0, 0, 0),
        insn(cbpf::JMP | cbpf::JSET | cbpf::K, 1, 0, 0x10),
        insn(cbpf::RET | cbpf::K, 0, 0, 0),
        insn(cbpf::RET | cbpf::K, 0, 0, 1),
    };
    EXPECT_EQ(cbpfExecute(s, pkt, sizeof(pkt)), 1u);
    pkt[0] = 0x01;
    EXPECT_EQ(cbpfExecute(s, pkt, sizeof(pkt)), 0u);
}

TEST(CBpfVmTest, FallOffEndAndBadOpcodeDrop) {
    uint8_t pkt[] = {0x01};
    std::vector<CBPFInsn> no_ret = {
        insn(cbpf::LD | cbpf::B | cbpf::ABS, 0, 0, 0),   // no RET
    };
    EXPECT_EQ(cbpfExecute(no_ret, pkt, sizeof(pkt)), 0u);

    std::vector<CBPFInsn> bad = {
        insn(0xFF, 0, 0, 0),
        insn(cbpf::RET | cbpf::K, 0, 0, 1),
    };
    EXPECT_EQ(cbpfExecute(bad, pkt, sizeof(pkt)), 0u);
}

// ============================================================================
// ethercatFilter() — untagged + any-VID tagged EtherCAT
// ============================================================================

class EthercatFilterTest : public ::testing::Test {
protected:
    std::vector<CBPFInsn> prog = CBPFProgramFactory::ethercatFilter();
};

TEST_F(EthercatFilterTest, AcceptsUntaggedEthercat) {
    EXPECT_TRUE(accepted(prog, ethFrame(kEtherTypeEtherCAT, ecatPayload())));
    // Bare minimum: EtherType alone is enough (payload not inspected)
    EXPECT_TRUE(accepted(prog, ethFrame(kEtherTypeEtherCAT)));
}

TEST_F(EthercatFilterTest, AcceptsTaggedEthercatAnyVid) {
    for (uint16_t vid : {1, 100, 1999, 4094, 4095}) {
        EXPECT_TRUE(accepted(prog, vlanFrame(vid, kEtherTypeEtherCAT,
                                             ecatPayload())))
            << "vid=" << vid;
    }
}

TEST_F(EthercatFilterTest, RejectsNonEthercat) {
    EXPECT_FALSE(accepted(prog, ethFrame(0x0806)));    // ARP
    EXPECT_FALSE(accepted(prog, ethFrame(0x0800)));    // IPv4 (UDP off)
    EXPECT_FALSE(accepted(prog, ethFrame(0x86DD)));    // IPv6
    EXPECT_FALSE(accepted(prog, ethFrame(0x0000)));
    EXPECT_FALSE(accepted(prog, udpFrame()));          // UDP/34980: off here
}

TEST_F(EthercatFilterTest, RejectsTaggedNonEthercat) {
    EXPECT_FALSE(accepted(prog, vlanFrame(100, 0x0806)));
    EXPECT_FALSE(accepted(prog, vlanFrame(100, kEtherTypeIPv4)));
    EXPECT_FALSE(accepted(prog, vlanFrame(100, 0x88A8)));  // nested tag ≠ ECAT
}

TEST_F(EthercatFilterTest, RejectsQinQOuterAndTruncated) {
    // 0x88A8 (802.1ad) outer TPID is not matched
    EXPECT_FALSE(accepted(prog,
        vlanFrame(100, kEtherTypeEtherCAT, {}, 0x88A8)));
    // shorter than an Ethernet header
    EXPECT_FALSE(accepted(prog, {0xFF, 0xFF, 0x88}));
    // EtherType field straddles end
    EXPECT_FALSE(accepted(prog, std::vector<uint8_t>(13, 0xFF)));
    // tag claims inner EtherType but frame ends inside it
    auto t = vlanFrame(100, kEtherTypeEtherCAT);
    t.resize(17);
    EXPECT_FALSE(accepted(prog, t));
}

// ============================================================================
// vlanFilter() — tagged-only, exact VID
// ============================================================================

class VlanFilterTest : public ::testing::Test {
protected:
    static constexpr uint16_t kVid = 1999;
    std::vector<CBPFInsn> prog = CBPFProgramFactory::vlanFilter(kVid);
};

TEST_F(VlanFilterTest, AcceptsMatchingVidWithEthercatInner) {
    EXPECT_TRUE(accepted(prog, vlanFrame(kVid, kEtherTypeEtherCAT,
                                         ecatPayload())));
}

TEST_F(VlanFilterTest, RejectsOtherVids) {
    for (uint16_t vid : {0, 1, 1998, 2000, 4095})
        EXPECT_FALSE(accepted(prog, vlanFrame(vid, kEtherTypeEtherCAT)))
            << "vid=" << vid;
    // VID is masked to 12 bits — PCP/DEI bits must not leak into the compare
    auto pcp = vlanFrame(kVid, kEtherTypeEtherCAT);
    pcp[14] |= 0xE0;                                    // PCP=7, DEI=1
    EXPECT_TRUE(accepted(prog, pcp));
}

TEST_F(VlanFilterTest, RejectsUntaggedEvenEthercat) {
    // Documented --rx-vlan semantics: untagged frames are dropped.
    EXPECT_FALSE(accepted(prog, ethFrame(kEtherTypeEtherCAT, ecatPayload())));
    EXPECT_FALSE(accepted(prog, ethFrame(0x0806)));
}

TEST_F(VlanFilterTest, RejectsMatchingVidWithNonEthercatInner) {
    EXPECT_FALSE(accepted(prog, vlanFrame(kVid, 0x0806)));
    EXPECT_FALSE(accepted(prog, vlanFrame(kVid, kEtherTypeIPv4)));
    EXPECT_FALSE(accepted(prog, vlanUdpFrame(kVid)));    // UDP-ECAT rejected too
    EXPECT_FALSE(accepted(prog, vlanFrame(kVid, 0x88A8)));  // QinQ inner
}

TEST_F(VlanFilterTest, RejectsTruncated) {
    EXPECT_FALSE(accepted(prog, std::vector<uint8_t>(13, 0xFF)));
    auto t = vlanFrame(kVid, kEtherTypeEtherCAT);
    t.resize(15);   // ends inside TCI
    EXPECT_FALSE(accepted(prog, t));
    t.resize(17);   // ends inside inner EtherType
    EXPECT_FALSE(accepted(prog, t));
}

TEST_F(VlanFilterTest, InvalidVidRejected) {
    EXPECT_TRUE(CBPFProgramFactory::vlanFilter(4096).empty());
    EXPECT_TRUE(CBPFProgramFactory::vlanFilter(0xFFFF).empty());
}

// ============================================================================
// vlanRangeFilter()
// ============================================================================

TEST(VlanRangeFilterTest, AcceptsOnlyRange) {
    auto prog = CBPFProgramFactory::vlanRangeFilter(100, 200);
    EXPECT_TRUE(accepted(prog, vlanFrame(100, kEtherTypeEtherCAT)));
    EXPECT_TRUE(accepted(prog, vlanFrame(200, kEtherTypeEtherCAT)));
    EXPECT_TRUE(accepted(prog, vlanFrame(150, kEtherTypeEtherCAT)));
    EXPECT_FALSE(accepted(prog, vlanFrame(99,  kEtherTypeEtherCAT)));
    EXPECT_FALSE(accepted(prog, vlanFrame(201, kEtherTypeEtherCAT)));
    EXPECT_FALSE(accepted(prog, ethFrame(kEtherTypeEtherCAT)));
}

TEST(VlanRangeFilterTest, InvalidRangeRejected) {
    EXPECT_TRUE(CBPFProgramFactory::vlanRangeFilter(200, 100).empty());
    EXPECT_TRUE(CBPFProgramFactory::vlanRangeFilter(100, 5000).empty());
}

// ============================================================================
// Stripped-tag (SKF_AD_VLAN_*) legs — models kernel RX VLAN untagging
//
// On the real RX path the kernel removes the 802.1Q tag before packet
// sockets see the data: [12] then shows the INNER EtherType and the TCI
// is only in skb auxdata.  These tests feed the interpreter an untagged-
// looking buffer plus the auxdata the kernel would have reported.
// ============================================================================

namespace {

bool acceptedAux(const std::vector<CBPFInsn>& prog,
                 const std::vector<uint8_t>& pkt, int tci = -1) {
    CBPFAuxData aux;
    if (tci >= 0) aux.vlan_tci = static_cast<uint16_t>(tci);
    return cbpfExecute(prog, pkt.data(), pkt.size(), &aux) != 0;
}

} // namespace

TEST(VlanStrippedTagTest, ProgramEmitsAuxLoads) {
    const auto prog = CBPFProgramFactory::vlanFilter(1999);
    bool has_present = false, has_tag = false;
    for (const auto& i : prog) {
        if (i.code == (cbpf::LD | cbpf::W | cbpf::ABS)) {
            if (i.k == kSkfAdVlanTagPresent) has_present = true;
            if (i.k == kSkfAdVlanTag)        has_tag = true;
        }
    }
    EXPECT_TRUE(has_present);
    EXPECT_TRUE(has_tag);
}

TEST(VlanStrippedTagTest, FilterAcceptsStrippedMatchingTag) {
    const auto prog = CBPFProgramFactory::vlanFilter(1999);
    const auto pkt  = ethFrame(kEtherTypeEtherCAT, ecatPayload());
    EXPECT_TRUE(acceptedAux(prog, pkt, 0x07CF));       // TCI = VID 1999
    EXPECT_TRUE(acceptedAux(prog, pkt, 0x07CF | 0xE000)); // PCP 7 + VID
}

TEST(VlanStrippedTagTest, FilterRejectsStrippedWrongVidAndUntagged) {
    const auto prog = CBPFProgramFactory::vlanFilter(1999);
    const auto pkt  = ethFrame(kEtherTypeEtherCAT, ecatPayload());
    EXPECT_FALSE(acceptedAux(prog, pkt, 2000));        // wrong VID
    EXPECT_FALSE(acceptedAux(prog, pkt, 0));           // VID 0
    EXPECT_FALSE(acceptedAux(prog, pkt));              // genuinely untagged
    EXPECT_FALSE(acceptedAux(prog, ethFrame(0x0806), 0x07CF)); // ARP+tag
}

TEST(VlanStrippedTagTest, RangeFilterHonorsStrippedTci) {
    const auto prog = CBPFProgramFactory::vlanRangeFilter(100, 200);
    const auto pkt  = ethFrame(kEtherTypeEtherCAT, ecatPayload());
    EXPECT_TRUE(acceptedAux(prog, pkt, 100));
    EXPECT_TRUE(acceptedAux(prog, pkt, 200));
    EXPECT_FALSE(acceptedAux(prog, pkt, 99));
    EXPECT_FALSE(acceptedAux(prog, pkt, 201));
    EXPECT_FALSE(acceptedAux(prog, pkt));              // untagged → reject
}

TEST(VlanStrippedTagTest, CatchAllSpecAcceptsAnyStrippedTag) {
    CBPFSpec s{};
    s.untagged_ethercat = false;
    s.tagged_ethercat   = true;                    // vlan:any — no range
    const auto prog = CBPFProgramFactory::build(s);
    ASSERT_FALSE(prog.empty());
    const auto pkt = ethFrame(kEtherTypeEtherCAT, ecatPayload());
    EXPECT_TRUE(acceptedAux(prog, pkt, 1));
    EXPECT_TRUE(acceptedAux(prog, pkt, 4095));
    EXPECT_FALSE(acceptedAux(prog, pkt));          // still rejects untagged
}

TEST(VlanStrippedTagTest, VlanUdpSpecAcceptsStrippedTaggedUdp) {
    // vlan:1999 + udp → stripped IPv4 frame + auxdata must reach the
    // UDP check (base 14, since the data shows the inner EtherType).
    CBPFSpec s{};
    s.untagged_ethercat = false;
    s.tagged_ethercat   = true;
    s.tagged_udp        = true;
    s.untagged_udp      = false;
    s.vlan_range        = CBPFVlanRange{1999, 1999};
    const auto prog = CBPFProgramFactory::build(s);
    ASSERT_FALSE(prog.empty());

    EXPECT_TRUE(acceptedAux(prog, udpFrame(UdpOpts{}, ecatPayload()), 1999));
    UdpOpts wrong; wrong.dst_port = 9999;
    EXPECT_FALSE(acceptedAux(prog, udpFrame(wrong, ecatPayload()), 1999));
    EXPECT_FALSE(acceptedAux(prog, udpFrame(UdpOpts{}, ecatPayload()), 2000));
    EXPECT_FALSE(acceptedAux(prog, udpFrame(UdpOpts{}, ecatPayload())));
}

TEST(VlanStrippedTagTest, DefaultFilterIgnoresAuxData) {
    // ethercatFilter() accepts both states anyway — no aux loads needed
    // and none emitted (the wire leg jumps straight to accept).
    const auto prog = CBPFProgramFactory::ethercatFilter();
    for (const auto& i : prog)
        EXPECT_NE(i.code, static_cast<uint16_t>(cbpf::LD | cbpf::W | cbpf::ABS))
            << "unexpected SKF_AD load in the default program";
    const auto pkt = ethFrame(kEtherTypeEtherCAT, ecatPayload());
    EXPECT_TRUE(acceptedAux(prog, pkt, 77));       // tagged-but-stripped: ok
    EXPECT_TRUE(acceptedAux(prog, pkt));           // untagged: ok
}

// ============================================================================
// CBPFSpec::first_idx_range — composed encap ∧ first-datagram-idx demux
//
// The cyclic datapath's socket pair replaces SO_ATTACH_FILTER wholesale,
// so VLAN acceptance and the fastpath-idx check must live in ONE program:
// socket A gets include-mode (encap ∧ idx∈range), socket B exclude-mode
// (encap ∧ idx∉range).  First idx sits at byte 17 untagged, 21 tagged.
// ============================================================================

namespace {

/// EtherCAT frame whose first datagram idx is `idx` (payload[3] =
/// ecat-hdr(2) + cmd(1) offset into the payload).
std::vector<uint8_t> ecatIdxFrame(uint8_t idx) {
    auto payload = ecatPayload(32);
    payload[3] = idx;
    return ethFrame(kEtherTypeEtherCAT, std::move(payload));
}

/// 802.1Q-tagged EtherCAT frame, first idx at absolute offset 21.
std::vector<uint8_t> vlanIdxFrame(uint16_t vid, uint8_t idx) {
    auto payload = ecatPayload(32);
    payload[3] = idx;
    return vlanFrame(vid, kEtherTypeEtherCAT, std::move(payload));
}

CBPFSpec fastpathVlanSpec() {
    CBPFSpec s{};
    s.untagged_ethercat = false;
    s.tagged_ethercat   = true;
    s.vlan_range        = CBPFVlanRange{1999, 1999};
    s.first_idx_range   = CBPFIdxRange{kSliceSlotBaseIdx, kFastSlotEndIdx};
    return s;
}

} // namespace

TEST(FirstIdxRangeTest, UntaggedIncludeDemuxesFastpath) {
    CBPFSpec s{};
    s.first_idx_range = CBPFIdxRange{kSliceSlotBaseIdx, kFastSlotEndIdx};
    const auto prog = CBPFProgramFactory::build(s);
    ASSERT_FALSE(prog.empty());

    for (uint8_t idx : {0xE0, 0xE7, 0xEF, 0xF0, 0xF7, 0xF8, 0xFD})
        EXPECT_TRUE(accepted(prog, ecatIdxFrame(idx)))
            << "idx 0x" << std::hex << (int)idx;
    for (uint8_t idx : {0x00, 0x42, 0xDF, 0xFE, 0xFF})
        EXPECT_FALSE(accepted(prog, ecatIdxFrame(idx)))
            << "idx 0x" << std::hex << (int)idx;
}

TEST(FirstIdxRangeTest, UntaggedExcludeMirrorsDemux) {
    CBPFSpec s{};
    s.first_idx_range   = CBPFIdxRange{kSliceSlotBaseIdx, kFastSlotEndIdx};
    s.first_idx_exclude = true;
    const auto prog = CBPFProgramFactory::build(s);
    ASSERT_FALSE(prog.empty());

    // Exclude mode: everything the include program accepts is rejected —
    // including the 0xF0..0xF7 reserved gap (in-range, but slotless).
    for (uint8_t idx : {0xE0, 0xEF, 0xF0, 0xF7, 0xF8, 0xFD})
        EXPECT_FALSE(accepted(prog, ecatIdxFrame(idx)))
            << "idx 0x" << std::hex << (int)idx;
    for (uint8_t idx : {0x00, 0x42, 0xDF, 0xFE, 0xFF})
        EXPECT_TRUE(accepted(prog, ecatIdxFrame(idx)))
            << "idx 0x" << std::hex << (int)idx;
}

TEST(FirstIdxRangeTest, TaggedIncludeReadsIdxAtOffset21) {
    const auto prog = CBPFProgramFactory::build(fastpathVlanSpec());
    ASSERT_FALSE(prog.empty());

    // VID 1999 + fastpath idx → accepted (idx at byte 21, not 17).
    EXPECT_TRUE(accepted(prog, vlanIdxFrame(1999, 0xE0)));
    EXPECT_TRUE(accepted(prog, vlanIdxFrame(1999, 0xFD)));
    // VID ok but async idx → rejected.
    EXPECT_FALSE(accepted(prog, vlanIdxFrame(1999, 0x42)));
    EXPECT_FALSE(accepted(prog, vlanIdxFrame(1999, 0xFE)));
    // Fastpath idx but wrong VID → rejected (encap clause dominates).
    EXPECT_FALSE(accepted(prog, vlanIdxFrame(2000, 0xE0)));
    // Untagged fastpath → rejected (untagged_ethercat=false).
    EXPECT_FALSE(accepted(prog, ecatIdxFrame(0xE0)));
    // Non-EtherCAT inner type → rejected regardless of bytes.
    EXPECT_FALSE(accepted(prog, vlanFrame(1999, 0x0806, ecatPayload(32))));
}

TEST(FirstIdxRangeTest, TaggedExcludeKeepsAsyncOnly) {
    auto s = fastpathVlanSpec();
    s.first_idx_exclude = true;
    const auto prog = CBPFProgramFactory::build(s);
    ASSERT_FALSE(prog.empty());

    EXPECT_FALSE(accepted(prog, vlanIdxFrame(1999, 0xE0)));
    EXPECT_FALSE(accepted(prog, vlanIdxFrame(1999, 0xFD)));
    EXPECT_TRUE(accepted(prog, vlanIdxFrame(1999, 0x42)));
    EXPECT_TRUE(accepted(prog, vlanIdxFrame(1999, 0xFE)));
    EXPECT_FALSE(accepted(prog, vlanIdxFrame(2000, 0x42)));  // wrong VID
}

TEST(FirstIdxRangeTest, StrippedTagLegUsesUntaggedOffset) {
    // A kernel-stripped VLAN frame presents the untagged layout in the
    // data buffer (idx at 17) plus the TCI in auxdata — the wire leg's
    // EtherType accept must funnel into the offset-17 idx check.
    const auto prog = CBPFProgramFactory::build(fastpathVlanSpec());
    ASSERT_FALSE(prog.empty());

    EXPECT_TRUE(acceptedAux(prog, ecatIdxFrame(0xE0), 1999));
    EXPECT_TRUE(acceptedAux(prog, ecatIdxFrame(0xFD), 1999));
    EXPECT_FALSE(acceptedAux(prog, ecatIdxFrame(0x42), 1999)); // async idx
    EXPECT_FALSE(acceptedAux(prog, ecatIdxFrame(0xE0), 2000)); // wrong VID
    EXPECT_FALSE(acceptedAux(prog, ecatIdxFrame(0xE0)));       // no tag
}

TEST(FirstIdxRangeTest, IncludeCoversSliceAndCyclicPoolsDisjointly) {
    // Mirror of the socket-pair split: build both programs from one spec
    // and prove every idx in 0..255 lands on exactly one side.
    auto spec = fastpathVlanSpec();
    const auto acc  = CBPFProgramFactory::build(spec);
    spec.first_idx_exclude = true;
    const auto asy  = CBPFProgramFactory::build(spec);
    ASSERT_FALSE(acc.empty());
    ASSERT_FALSE(asy.empty());

    for (int idx = 0; idx < 256; ++idx) {
        const auto frame = vlanIdxFrame(1999, static_cast<uint8_t>(idx));
        const bool a = accepted(acc, frame);
        const bool b = accepted(asy, frame);
        EXPECT_TRUE(a != b) << "idx 0x" << std::hex << idx
                            << " double-delivered or dropped";
    }
}

// ============================================================================
// ethercatFilterWithUdp() — adds IPv4/UDP dst-port matching
// ============================================================================

class UdpFilterTest : public ::testing::Test {
protected:
    std::vector<CBPFInsn> prog = CBPFProgramFactory::ethercatFilterWithUdp();
};

TEST_F(UdpFilterTest, AcceptsEthercatOverUdp) {
    EXPECT_TRUE(accepted(prog, udpFrame(UdpOpts{}, ecatPayload())));
}

TEST_F(UdpFilterTest, AcceptsUdpWithIpOptions) {
    UdpOpts o; o.ihl = 8;   // 12 bytes of options
    EXPECT_TRUE(accepted(prog, udpFrame(o, ecatPayload())));
    o.ihl = 15;
    EXPECT_TRUE(accepted(prog, udpFrame(o, ecatPayload())));
}

TEST_F(UdpFilterTest, RejectsIhlBelowFive) {
    UdpOpts o; o.ihl = 4;
    auto pkt = udpFrame(o, ecatPayload());
    // ihl=4 makes ipv4Header only 16 bytes — UDP header still appended;
    // filter must reject on IHL < 5 regardless.
    EXPECT_FALSE(accepted(prog, pkt));
}

TEST_F(UdpFilterTest, RejectsNonEthercatPortsAndProtos) {
    UdpOpts o; o.dst_port = 34981;
    EXPECT_FALSE(accepted(prog, udpFrame(o, ecatPayload())));
    o.dst_port = 80;
    EXPECT_FALSE(accepted(prog, udpFrame(o, ecatPayload())));
    o.proto = 6;            // TCP
    o.dst_port = kEtherCATUdpPort;
    EXPECT_FALSE(accepted(prog, udpFrame(o, ecatPayload())));
}

TEST_F(UdpFilterTest, FragmentHandling) {
    UdpOpts first; first.frag = 0x2000;        // MF set, offset 0
    EXPECT_TRUE(accepted(prog, udpFrame(first, ecatPayload())));
    UdpOpts nonfirst; nonfirst.frag = 0x2001;  // offset != 0 — no UDP hdr
    EXPECT_FALSE(accepted(prog, udpFrame(nonfirst, ecatPayload())));
}

TEST_F(UdpFilterTest, RejectsTruncatedUdp) {
    auto pkt = udpFrame(UdpOpts{}, ecatPayload());
    pkt.resize(14 + 20 + 3);   // dst port (UDP bytes 2-3) straddles the end
    EXPECT_FALSE(accepted(prog, pkt));
    pkt.resize(14 + 20 + 1);   // ends before dst port field
    EXPECT_FALSE(accepted(prog, pkt));
    pkt.resize(14 + 10);       // ends inside IP header
    EXPECT_FALSE(accepted(prog, pkt));
}

TEST_F(UdpFilterTest, TaggedUdpAndEthercatStillWork) {
    EXPECT_TRUE(accepted(prog, vlanUdpFrame(42, UdpOpts{}, ecatPayload())));
    EXPECT_TRUE(accepted(prog, vlanFrame(42, kEtherTypeEtherCAT)));
    EXPECT_TRUE(accepted(prog, ethFrame(kEtherTypeEtherCAT)));
    UdpOpts o; o.dst_port = 9999;
    EXPECT_FALSE(accepted(prog, vlanUdpFrame(42, o, ecatPayload())));
}

TEST_F(UdpFilterTest, CustomPort) {
    auto p = CBPFProgramFactory::ethercatFilterWithUdp(12345);
    UdpOpts o; o.dst_port = 12345;
    EXPECT_TRUE(accepted(p, udpFrame(o)));
    EXPECT_FALSE(accepted(p, udpFrame(UdpOpts{})));
}

// ============================================================================
// Spec-level edge cases
// ============================================================================

TEST(CBpfSpecTest, EmptySpecProducesEmptyProgram) {
    CBPFSpec s;
    s.untagged_ethercat = false;
    s.tagged_ethercat   = false;
    EXPECT_TRUE(CBPFProgramFactory::build(s).empty());
}

TEST(CBpfSpecTest, TaggedUdpWithoutEthercat) {
    CBPFSpec s;
    s.untagged_ethercat = false;
    s.tagged_ethercat   = false;
    s.tagged_udp        = true;
    auto p = CBPFProgramFactory::build(s);
    ASSERT_FALSE(p.empty());
    EXPECT_TRUE(accepted(p, vlanUdpFrame(7, UdpOpts{}, ecatPayload())));
    EXPECT_FALSE(accepted(p, vlanFrame(7, kEtherTypeEtherCAT)));
    EXPECT_FALSE(accepted(p, ethFrame(kEtherTypeEtherCAT)));
}

TEST(CBpfSpecTest, AlternativeTpid) {
    CBPFSpec s;
    s.vlan_tpid = 0x88A8;   // 802.1ad provider bridging
    auto p = CBPFProgramFactory::build(s);
    EXPECT_TRUE(accepted(p, vlanFrame(5, kEtherTypeEtherCAT, {}, 0x88A8)));
    EXPECT_FALSE(accepted(p, vlanFrame(5, kEtherTypeEtherCAT, {}, 0x8100)));
    EXPECT_TRUE(accepted(p, ethFrame(kEtherTypeEtherCAT)));
}

// ============================================================================
// Property-ish sweep: random garbage must terminate with a verdict
// ============================================================================

TEST(CBpfProgramRobustness, RandomPacketsNeverCrashOrHang) {
    const auto progs = {
        CBPFProgramFactory::ethercatFilter(),
        CBPFProgramFactory::ethercatFilterWithUdp(),
        CBPFProgramFactory::vlanFilter(1999),
        CBPFProgramFactory::vlanRangeFilter(10, 20),
    };
    uint32_t seed = 0x88A4;
    auto rnd = [&] { seed = seed * 1664525u + 1013904223u; return seed >> 24; };
    for (const auto& p : progs) {
        for (int i = 0; i < 2000; ++i) {
            std::vector<uint8_t> pkt(rnd() % 200);
            for (auto& b : pkt) b = static_cast<uint8_t>(rnd());
            const uint32_t r = cbpfExecute(p, pkt.data(), pkt.size());
            EXPECT_TRUE(r == 0 || r == 0xFFFFFFFFu);
        }
    }
}

// ============================================================================
// Kernel-level test: attach to a real socket (AF_UNIX pair — no CAP_NET_RAW)
// ============================================================================

#if defined(__linux__)
class KernelAttachTest : public ::testing::Test {
protected:
    int sv_[2] = {-1, -1};
    void SetUp() override {
        ASSERT_EQ(::socketpair(AF_UNIX, SOCK_DGRAM, 0, sv_), 0);
        fcntl(sv_[0], F_SETFL, O_NONBLOCK);
        fcntl(sv_[1], F_SETFL, O_NONBLOCK);
    }
    void TearDown() override {
        if (sv_[0] >= 0) ::close(sv_[0]);
        if (sv_[1] >= 0) ::close(sv_[1]);
    }
    /// true when a datagram arrives within 50ms
    bool rxReady(int ms = 50) {
        struct pollfd pfd{sv_[0], POLLIN, 0};
        return ::poll(&pfd, 1, ms) > 0 && (pfd.revents & POLLIN);
    }
    void send(const std::vector<uint8_t>& f) {
        ASSERT_EQ(::send(sv_[1], f.data(), f.size(), 0), (ssize_t)f.size());
    }
};

TEST_F(KernelAttachTest, EthercatFilterPassesInKernel) {
    auto prog = CBPFProgramFactory::ethercatFilter();
    ASSERT_TRUE(CBPFProgramFactory::attach(sv_[0], prog));

    send(ethFrame(0x0806, std::vector<uint8_t>(28, 0)));       // ARP → dropped
    EXPECT_FALSE(rxReady());
    send(ethFrame(kEtherTypeEtherCAT, ecatPayload()));
    ASSERT_TRUE(rxReady());
    uint8_t buf[2048];
    EXPECT_GT(::recv(sv_[0], buf, sizeof(buf), 0), 0);
}

TEST_F(KernelAttachTest, VlanFilterEnforcedByKernel) {
    auto prog = CBPFProgramFactory::vlanFilter(1999);
    ASSERT_TRUE(CBPFProgramFactory::attach(sv_[0], prog));

    send(ethFrame(kEtherTypeEtherCAT, ecatPayload()));          // untagged → drop
    EXPECT_FALSE(rxReady());
    send(vlanFrame(2000, kEtherTypeEtherCAT, ecatPayload()));   // wrong VID
    EXPECT_FALSE(rxReady());
    send(vlanFrame(1999, 0x0806));                              // right VID, ARP
    EXPECT_FALSE(rxReady());
    send(vlanFrame(1999, kEtherTypeEtherCAT, ecatPayload()));
    EXPECT_TRUE(rxReady());
}
#endif // __linux__
