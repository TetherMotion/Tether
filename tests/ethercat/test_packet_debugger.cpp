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

} // namespace
