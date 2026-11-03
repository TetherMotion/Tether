/**
 * @file test_frame_interpreter.cpp
 * @brief Coverage for PCAP::FrameInterpreter — link-type dispatch
 *        (Ethernet / Linux SLL / raw IP / BSD null), VLAN stripping,
 *        EtherCAT datagram decoding, and UDP-encapsulated EtherCAT.
 *        Frames are built byte-wise (same wire layout as production).
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "tether/packetloggers/pcap/FrameInterpreter.hpp"
#include "tether/packetloggers/pcap/PCAPNGReader.hpp"

using namespace Tether::PacketLoggers::PCAP;
using EtherCAT::Command;

namespace {

constexpr uint16_t kEtherTypeEtherCAT = 0x88A4;
constexpr uint16_t kEtherTypeIPv4 = 0x0800;
constexpr uint16_t kEtherTypeVlan = 0x8100;
constexpr uint16_t kEtherTypeVlanQinQ = 0x88A8;

void put16be(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(static_cast<uint8_t>(x >> 8));
    v.push_back(static_cast<uint8_t>(x & 0xFF));
}
void put16le(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>(x >> 8));
}
void put32be(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(static_cast<uint8_t>(x >> 24));
    v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    v.push_back(static_cast<uint8_t>(x & 0xFF));
}

// EtherCAT datagram: cmd,idx | adp(LE16) | ado(LE16) | len11/res3/circ1/more1
//                    | irq(LE16) | data | wkc(LE16)
void putDatagram(std::vector<uint8_t>& v, uint8_t cmd, uint8_t idx,
                 uint16_t adp, uint16_t ado, uint16_t irq,
                 const std::vector<uint8_t>& data, uint16_t wkc,
                 bool more) {
    v.push_back(cmd);
    v.push_back(idx);
    put16le(v, adp);
    put16le(v, ado);
    const uint16_t lenword = static_cast<uint16_t>(
        (data.size() & 0x7FF) | (more ? 0x8000 : 0));
    put16le(v, lenword);
    put16le(v, irq);
    v.insert(v.end(), data.begin(), data.end());
    put16le(v, wkc);
}

// EtherCAT frame header: len11/res4/type1 (LE16) — type must be 1.
void putEcatHeader(std::vector<uint8_t>& v, uint16_t totalLen) {
    put16le(v, (totalLen & 0x7FF) | (1 << 12));
}

std::vector<uint8_t> makeEthernet(uint16_t etherType,
                                  const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> f = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,   // dst
                              0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // src
    put16be(f, etherType);
    f.insert(f.end(), payload.begin(), payload.end());
    return f;
}

std::vector<uint8_t> makeEcatPayload(
    const std::vector<std::vector<uint8_t>>& dgrams) {
    std::vector<uint8_t> p;
    // compute total: 2-byte ecat header + dgrams
    uint16_t total = 2;
    for (auto& d : dgrams) total += static_cast<uint16_t>(d.size());
    putEcatHeader(p, total);
    for (auto& d : dgrams) p.insert(p.end(), d.begin(), d.end());
    return p;
}

std::vector<uint8_t> makeDatagram(uint8_t cmd, uint16_t adp, uint16_t ado,
                                  const std::vector<uint8_t>& data,
                                  uint16_t wkc, bool more) {
    std::vector<uint8_t> d;
    putDatagram(d, cmd, 0x42, adp, ado, 0x5555, data, wkc, more);
    return d;
}

// IPv4 header (20 bytes, no options) + UDP dst port 34980 carrying payload.
std::vector<uint8_t> makeUdpPacket(const std::vector<uint8_t>& payload,
                                   uint16_t srcPort, uint16_t dstPort) {
    std::vector<uint8_t> ip;
    const uint16_t udpLen = static_cast<uint16_t>(8 + payload.size());
    const uint16_t ipLen = static_cast<uint16_t>(20 + udpLen);
    ip.push_back(0x45);                       // v4 + IHL5
    ip.push_back(0x00);                       // DSCP
    put16be(ip, ipLen);
    put16be(ip, 0x1234);                      // id
    put16be(ip, 0x0000);                      // flags/frag
    ip.push_back(64);                         // TTL
    ip.push_back(0x11);                       // proto UDP
    put16be(ip, 0x0000);                      // checksum (not verified)
    put32be(ip, 0xC0A80101);                  // src 192.168.1.1
    put32be(ip, 0xC0A80102);                  // dst 192.168.1.2
    put16be(ip, srcPort);
    put16be(ip, dstPort);
    put16be(ip, udpLen);
    put16be(ip, 0x0000);                      // udp checksum
    ip.insert(ip.end(), payload.begin(), payload.end());
    return ip;
}

class FrameInterpreterTest : public ::testing::Test {
protected:
    std::vector<PCAPNGInterfaceInfo> ifaces_ = {
        PCAPNGInterfaceInfo{.id = 0, .linkType = 1, .fcsLen = 4},
        PCAPNGInterfaceInfo{.id = 1, .linkType = 113, .fcsLen = 0},
        PCAPNGInterfaceInfo{.id = 2, .linkType = 0, .fcsLen = 0},
    };
    FrameInterpreter interp_{ifaces_};
};

}  // namespace

// ============================================================================
// Interface lookup
// ============================================================================

TEST_F(FrameInterpreterTest, InterfaceLookups) {
    EXPECT_EQ(interp_.interfaceFcsLen(0), 4);
    EXPECT_EQ(interp_.interfaceLinkType(0), 1);
    EXPECT_EQ(interp_.interfaceLinkType(1), 113);
    EXPECT_EQ(interp_.interfaceFcsLen(9), 0);       // unknown → 0
    EXPECT_EQ(interp_.interfaceLinkType(9), 1);     // unknown → ETHERNET
}

// ============================================================================
// Ethernet dispatch + EtherCAT decoding
// ============================================================================

TEST_F(FrameInterpreterTest, EthernetEtherCATFrameDecodes) {
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::LWR),
                              0x1111, 0x2222, {0xDE, 0xAD}, 3, false);
    auto payload = makeEcatPayload({dgram});
    auto frame = makeEthernet(kEtherTypeEtherCAT, payload);

    InterpretedFrame out;
    out.interfaceId = 0;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    EXPECT_TRUE(out.isEtherCAT);
    EXPECT_EQ(out.ecatFrameType, 1);
    ASSERT_EQ(out.datagrams.size(), 1u);
    EXPECT_EQ(out.datagrams[0].cmd, Command::LWR);
    EXPECT_EQ(out.datagrams[0].idx, 0x42);
    EXPECT_EQ(out.datagrams[0].adp, 0x1111);
    EXPECT_EQ(out.datagrams[0].ado, 0x2222);
    EXPECT_EQ(out.datagrams[0].irq, 0x5555);
    EXPECT_EQ(out.datagrams[0].wkc, 3);
    EXPECT_EQ(out.datagrams[0].data.size(), 2u);
    EXPECT_EQ(out.datagrams[0].logicalAddress(), 0x22221111u);
    EXPECT_TRUE(out.hasEtherCAT());
}

TEST_F(FrameInterpreterTest, EthernetNonEthercatParsesHeaderOnly) {
    auto frame = makeEthernet(kEtherTypeIPv4, {0, 1, 2, 3});
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    EXPECT_FALSE(out.isEtherCAT);
    EXPECT_EQ(out.innerEtherType, kEtherTypeIPv4);
    EXPECT_FALSE(out.hasEtherCAT());
}

TEST_F(FrameInterpreterTest, EthernetShortFrameRejected) {
    std::vector<uint8_t> tiny(10, 0);
    InterpretedFrame out;
    EXPECT_FALSE(interp_.interpretEthernetFrame(tiny.data(), tiny.size(), out));
}

TEST_F(FrameInterpreterTest, EthernetMacAddressesDecoded) {
    auto frame = makeEthernet(kEtherTypeIPv4, {0, 0, 0, 0});
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    EXPECT_EQ(out.dstMac[0], 0x01);
    EXPECT_EQ(out.srcMac[0], 0xAA);
}

TEST_F(FrameInterpreterTest, VlanTagStripped) {
    std::vector<uint8_t> inner;
    putEcatHeader(inner, 2);
    std::vector<uint8_t> tagged = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                   0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    put16be(tagged, kEtherTypeVlan);
    put16be(tagged, 0xA123);                    // PCP=5, VID=0x123
    put16be(tagged, kEtherTypeEtherCAT);
    tagged.insert(tagged.end(), inner.begin(), inner.end());

    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(tagged.data(), tagged.size(),
                                               out));
    ASSERT_TRUE(out.vlanId.has_value());
    EXPECT_EQ(*out.vlanId, 0x123);
    EXPECT_EQ(out.vlanPcp, 5);
    EXPECT_TRUE(out.isEtherCAT);
}

TEST_F(FrameInterpreterTest, QinQOuterTagStripped) {
    std::vector<uint8_t> f = {0,0,0,0,0,0, 0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    put16be(f, kEtherTypeVlanQinQ);             // 0x88A8 service tag
    put16be(f, 0x0010);                         // outer VID 16
    put16be(f, kEtherTypeIPv4);                 // inner etherType IPv4
    f.insert(f.end(), {0,0,0,0});
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(f.data(), f.size(), out));
    EXPECT_EQ(out.vlanId.value_or(0), 0x10);
    EXPECT_EQ(out.innerEtherType, kEtherTypeIPv4);
}

// ============================================================================
// Datagram parsing edge cases
// ============================================================================

TEST_F(FrameInterpreterTest, MultiDatagramFrame) {
    auto d1 = makeDatagram(static_cast<uint8_t>(Command::LRD), 1, 2, {1}, 1,
                           true);
    auto d2 = makeDatagram(static_cast<uint8_t>(Command::BRD), 3, 4, {2, 3}, 2,
                           false);
    auto payload = makeEcatPayload({d1, d2});
    auto frame = makeEthernet(kEtherTypeEtherCAT, payload);
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    ASSERT_EQ(out.datagrams.size(), 2u);
    EXPECT_EQ(out.datagrams[0].cmd, Command::LRD);
    EXPECT_TRUE(out.datagrams[0].more);
    EXPECT_EQ(out.datagrams[1].cmd, Command::BRD);
    EXPECT_FALSE(out.datagrams[1].more);
    EXPECT_EQ(out.datagrams[1].data.size(), 2u);
}

TEST_F(FrameInterpreterTest, TruncatedDatagramStopsParsing) {
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::LWR), 1, 2,
                              {0xAA, 0xBB, 0xCC}, 1, false);
    dgram.pop_back();                            // clip the wkc byte
    auto payload = makeEcatPayload({dgram});
    auto frame = makeEthernet(kEtherTypeEtherCAT, payload);
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    EXPECT_TRUE(out.datagrams.empty());          // truncated → not emitted
}

TEST_F(FrameInterpreterTest, ZeroLengthDatagram) {
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::NOP), 0, 0, {}, 0,
                              false);
    auto payload = makeEcatPayload({dgram});
    auto frame = makeEthernet(kEtherTypeEtherCAT, payload);
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    ASSERT_EQ(out.datagrams.size(), 1u);
    EXPECT_TRUE(out.datagrams[0].data.empty());
    EXPECT_EQ(out.datagrams[0].cmd, Command::NOP);
}

TEST_F(FrameInterpreterTest, NonStandardFrameTypeStillDecoded) {
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::BRD), 1, 2, {8}, 1,
                              false);
    std::vector<uint8_t> payload;
    put16le(payload, (static_cast<uint16_t>(2 + dgram.size()) & 0x7FF) |
                     (7 << 12));                 // type=7 — implementation
    payload.insert(payload.end(), dgram.begin(), dgram.end());
    auto frame = makeEthernet(kEtherTypeEtherCAT, payload);
    InterpretedFrame out;
    ASSERT_TRUE(interp_.interpretEthernetFrame(frame.data(), frame.size(), out));
    EXPECT_TRUE(out.isEtherCAT);
    EXPECT_EQ(out.ecatFrameType, 7);             // recorded, not enforced
    ASSERT_EQ(out.datagrams.size(), 1u);         // datagrams still walked
}

// ============================================================================
// Link-type dispatch (SLL / raw IP / BSD null)
// ============================================================================

TEST_F(FrameInterpreterTest, SllFrameDispatchesToEtherType) {
    std::vector<uint8_t> sll(16, 0);
    sll[14] = 0x88; sll[15] = 0xA4;               // EtherType at SLL offset 14
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::LRD), 9, 9, {7}, 1,
                              false);
    auto payload = makeEcatPayload({dgram});
    sll.insert(sll.end(), payload.begin(), payload.end());

    InterpretedFrame out;
    out.interfaceId = 1;                          // linkType 113
    interp_.interpretFrameByLinkType(sll.data(), sll.size(), out);
    EXPECT_TRUE(out.isEtherCAT);
    ASSERT_EQ(out.datagrams.size(), 1u);
}

TEST_F(FrameInterpreterTest, RawIpUdpEncapsulatedEtherCAT) {
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::LWR), 5, 6, {9}, 1,
                              false);
    auto payload = makeEcatPayload({dgram});
    auto udp = makeUdpPacket(payload, 40000, 0x88A4);

    std::vector<PCAPNGInterfaceInfo> rawIf{
        PCAPNGInterfaceInfo{.id = 0, .linkType = 101}};
    FrameInterpreter rawInterp{rawIf};
    InterpretedFrame out;
    out.interfaceId = 0;
    rawInterp.interpretFrameByLinkType(udp.data(), udp.size(), out);
    EXPECT_TRUE(out.isEtherCATOverUDP);
    EXPECT_TRUE(out.isEtherCAT);
    EXPECT_EQ(out.ipVersion, 4);
    EXPECT_EQ(out.dstPort, 0x88A4);
    EXPECT_EQ(out.srcIp, 0xC0A80101u);
    EXPECT_EQ(out.dstIp, 0xC0A80102u);
    ASSERT_EQ(out.datagrams.size(), 1u);
}

TEST_F(FrameInterpreterTest, NullFrameUdpEncapsulatedEtherCAT) {
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::BRD), 0, 0, {1}, 0,
                              false);
    auto payload = makeEcatPayload({dgram});
    auto udp = makeUdpPacket(payload, 1, 0x88A4);
    std::vector<uint8_t> nullf = {2, 0, 0, 0};    // AF_INET (BSD LE)
    nullf.insert(nullf.end(), udp.begin(), udp.end());

    std::vector<PCAPNGInterfaceInfo> nullIf{
        PCAPNGInterfaceInfo{.id = 0, .linkType = 0}};
    FrameInterpreter nullInterp{nullIf};
    InterpretedFrame out;
    out.interfaceId = 0;
    nullInterp.interpretFrameByLinkType(nullf.data(), nullf.size(), out);
    EXPECT_TRUE(out.isEtherCATOverUDP);
    ASSERT_EQ(out.datagrams.size(), 1u);
    EXPECT_EQ(out.datagrams[0].cmd, Command::BRD);
}

TEST_F(FrameInterpreterTest, NonUdpPayloadNotEtherCAT) {
    std::vector<uint8_t> tcp = makeUdpPacket({1, 2, 3}, 80, 443);
    tcp[9] = 0x06;                                // proto TCP, not UDP
    std::vector<PCAPNGInterfaceInfo> rawIf{
        PCAPNGInterfaceInfo{.id = 0, .linkType = 101}};
    FrameInterpreter rawInterp{rawIf};
    InterpretedFrame out;
    out.interfaceId = 0;
    rawInterp.interpretFrameByLinkType(tcp.data(), tcp.size(), out);
    EXPECT_FALSE(out.isEtherCATOverUDP);
    EXPECT_FALSE(out.isEtherCAT);
}

TEST_F(FrameInterpreterTest, UdpToNonEcatPortNotEncapsulated) {
    auto udp = makeUdpPacket({0, 1, 2, 3}, 1234, 53);   // DNS, not 34980
    std::vector<PCAPNGInterfaceInfo> rawIf{
        PCAPNGInterfaceInfo{.id = 0, .linkType = 101}};
    FrameInterpreter rawInterp{rawIf};
    InterpretedFrame out;
    out.interfaceId = 0;
    rawInterp.interpretFrameByLinkType(udp.data(), udp.size(), out);
    EXPECT_FALSE(out.isEtherCATOverUDP);
}

TEST_F(FrameInterpreterTest, EthernetFrameOnSllInterfaceStillEthernet) {
    // interface 0 is linkType 1 → dispatch picks the Ethernet path.
    auto dgram = makeDatagram(static_cast<uint8_t>(Command::LRD), 1, 1, {5}, 1,
                              false);
    auto payload = makeEcatPayload({dgram});
    auto frame = makeEthernet(kEtherTypeEtherCAT, payload);
    InterpretedFrame out;
    out.interfaceId = 0;
    interp_.interpretFrameByLinkType(frame.data(), frame.size(), out);
    EXPECT_EQ(out.linkType, 1);
    EXPECT_TRUE(out.isEtherCAT);
}
