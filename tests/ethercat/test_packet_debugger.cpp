/**
 * @file test_packet_debugger.cpp
 * @brief Coverage for the stateless EtherCAT frame pretty-printer
 *        (src/ethercat/raw/PacketDebugger.cpp).
 *
 * The functions only emit log lines, so the tests exercise every decode
 * branch — EtherType table, Ethernet header, short frames, direct
 * EtherCAT datagrams (single/multi), truncation variants — and assert
 * they complete without crashing.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "../src/ethercat/raw/PacketDebugger.hpp"

namespace {

/// Append little-endian u16.
void put16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>(x >> 8));
}

/// Ethernet header + EC frame header + datagrams (10B header + data + WKC).
std::vector<uint8_t> makeFrame(uint16_t ether_type,
                               const std::vector<uint16_t>& dg_lens) {
    std::vector<uint8_t> f;
    for (int i = 0; i < 6; ++i) f.push_back(0xA0 + i);  // dst MAC
    for (int i = 0; i < 6; ++i) f.push_back(0x50 + i);  // src MAC
    f.push_back(static_cast<uint8_t>(ether_type >> 8));
    f.push_back(static_cast<uint8_t>(ether_type & 0xFF));

    uint16_t payload_len = 0;
    for (uint16_t l : dg_lens) payload_len += 10 + l + 2;
    put16(f, static_cast<uint16_t>(payload_len | (1u << 12)));  // len + type

    for (size_t i = 0; i < dg_lens.size(); ++i) {
        const uint16_t dlen = dg_lens[i];
        const bool more = (i + 1 < dg_lens.size());
        f.push_back(0x04);                                   // cmd = FPRD
        f.push_back(static_cast<uint8_t>(i));                // idx
        put16(f, 0x1000 + static_cast<uint16_t>(i));           // adp
        put16(f, 0x0130);                                    // ado
        put16(f, static_cast<uint16_t>(dlen | (more ? 0x8000 : 0)));  // lenFlags
        put16(f, 0);                                         // irq
        for (uint16_t b = 0; b < dlen; ++b)
            f.push_back(static_cast<uint8_t>(b & 0xFF));
        put16(f, 3);                                         // WKC
    }
    return f;
}

TEST(PacketDebuggerTest, EtherTypeTableCoversKnownValues) {
    using namespace EtherCAT::PacketDebugger;
    EXPECT_STREQ(etherTypeToString(0x0800), "IPv4");
    EXPECT_STREQ(etherTypeToString(0x88A4), "EtherCAT");
    EXPECT_STREQ(etherTypeToString(0x8100), "VLAN (802.1Q)");
    EXPECT_STREQ(etherTypeToString(0x86DD), "IPv6");
    EXPECT_STREQ(etherTypeToString(0x0806), "ARP");
    EXPECT_EQ(etherTypeToString(0x0000), nullptr);
    EXPECT_EQ(etherTypeToString(0xFFFF), nullptr);
}

TEST(PacketDebuggerTest, TooShortForEthernetHeader) {
    uint8_t tiny[4] = {0};
    EtherCAT::PacketDebugger::printEtherCATFrame(tiny, sizeof(tiny), true, true);
}

TEST(PacketDebuggerTest, ShortEtherCatHeader) {
    auto f = makeFrame(0x88A4, {});
    f.resize(15);  // ethernet + 1 byte — below EC header minimum
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, false);
}

TEST(PacketDebuggerTest, DirectEtherCatSingleDatagram) {
    auto f = makeFrame(0x88A4, {8});
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, true);
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), false, true);
}

TEST(PacketDebuggerTest, DirectEtherCatMultipleDatagrams) {
    auto f = makeFrame(0x88A4, {4, 16, 0});
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, false);
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), false, false);
}

TEST(PacketDebuggerTest, TruncatedDatagramData) {
    auto f = makeFrame(0x88A4, {32});
    f.resize(f.size() - 20);  // chop inside the data + WKC
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, false);
}

TEST(PacketDebuggerTest, NonEtherCatEtherTypeStillDecodes) {
    auto f = makeFrame(0x0800, {8});  // IPv4 etherType — decoder doesn't gate
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, false);
}

TEST(PacketDebuggerTest, EmptyDatagramList) {
    auto f = makeFrame(0x88A4, {});
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, true);
}

TEST(PacketDebuggerTest, ZeroLengthDatagram) {
    auto f = makeFrame(0x88A4, {0});
    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(), true, false);
}

// Every named EtherType in the table resolves to a string; unknowns
// return nullptr.
TEST(PacketDebuggerTest, EtherTypeTableAllArms) {
    const uint16_t known[] = {
        0x0800, 0x0806, 0x0842, 0x22F3, 0x22EA, 0x6003, 0x8035, 0x809B,
        0x80F3, 0x8100, 0x8204, 0x86DD, 0x8808, 0x8809, 0x8819, 0x8847,
        0x8848, 0x8863, 0x8864, 0x887B, 0x888E, 0x8892, 0x889A, 0x88A2,
        0x88A4, 0x88A8, 0x88AB, 0x88B8, 0x88B9, 0x88BA, 0x88BF, 0x88CC,
        0x88CD, 0x88E1, 0x88E3, 0x88E5, 0x88E7, 0x88F7, 0x88F8, 0x88FB,
        0x8902, 0x8906, 0x8914, 0x8915, 0x891D, 0x892F, 0x8932, 0x9000,
        0x9100,
    };
    for (uint16_t t : known)
        EXPECT_NE(EtherCAT::PacketDebugger::etherTypeToString(t), nullptr) << std::hex << t;
    EXPECT_EQ(EtherCAT::PacketDebugger::etherTypeToString(0x1234), nullptr);
}

// A datagram whose payload is a CoE mailbox packet is run through the
// CoE interpreter — the decoded lines land in the output.
TEST(PacketDebuggerTest, CoEMailboxPayloadInterpreted) {
    // CoE mailbox payload: len=10, addr=1, chan=0,pri=0, type=3(CoE),cnt=0
    // + SDO expedited upload req (0x40) for 0x1018:01.
    std::vector<uint8_t> f;
    for (int i = 0; i < 6; ++i) f.push_back(0xA0 + i);
    for (int i = 0; i < 6; ++i) f.push_back(0x50 + i);
    f.push_back(0x88); f.push_back(0xA4);                    // EtherCAT

    // Mailbox datagram: 6B mbx hdr + 10B CoE SDO = 16 bytes payload.
    const uint16_t dlen = 16;
    put16(f, static_cast<uint16_t>((10 + dlen + 2) | (1u << 12)));

    f.push_back(0x05);                                     // cmd = FPRD-ish
    f.push_back(0x00);                                     // idx
    put16(f, 0x1000); put16(f, 0x0200);                    // adp/ado
    put16(f, dlen);                                        // lenFlags (last)
    put16(f, 0);                                           // irq
    // Mailbox header: len=10, addr=1, pri/chan=0, type=3 cnt=0.
    put16(f, 10); put16(f, 1);
    f.push_back(0x00); f.push_back(0x03);
    // CoE SDO expedited download request: 0x2F 0x18 0x10 0x01 + 4B data.
    f.push_back(0x2F); f.push_back(0x18); f.push_back(0x10);
    f.push_back(0x01);
    f.push_back(0x01); f.push_back(0x00); f.push_back(0x00);
    f.push_back(0x00);
    // pad to dlen (we wrote 6+10=16 — already there).
    put16(f, 1);                                           // WKC

    EtherCAT::PacketDebugger::printEtherCATFrame(f.data(), f.size(),
                                               true, true);
}

} // namespace
