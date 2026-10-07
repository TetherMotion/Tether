// Unit tests for VlanProbe's reply classifier — the pure wire-format
// logic (no sockets, no root needed).  The probe path itself requires a
// live EtherCAT segment and is exercised by ec_filter_probe / privileged
// integration tests.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "tether/ethercat/VlanProbe.hpp"

namespace {

/// Minimal Ethernet frame: dst/src MACs + ethertype + payload.
std::vector<uint8_t> frame(uint16_t ethertype, std::vector<uint8_t> payload = {}) {
    std::vector<uint8_t> f(14);
    f[12] = static_cast<uint8_t>(ethertype >> 8);
    f[13] = static_cast<uint8_t>(ethertype);
    f.insert(f.end(), payload.begin(), payload.end());
    return f;
}

/// 802.1Q-tagged frame: [12]=tpid [14:15]=TCI [16:17]=inner ethertype.
std::vector<uint8_t> taggedFrame(uint16_t tpid, uint16_t tci,
                                 uint16_t inner = 0x88A4) {
    auto f = frame(tpid);
    f.resize(18);
    f[14] = static_cast<uint8_t>(tci >> 8);
    f[15] = static_cast<uint8_t>(tci);
    f[16] = static_cast<uint8_t>(inner >> 8);
    f[17] = static_cast<uint8_t>(inner);
    return f;
}

struct Out {
    EtherCAT::VlanTagDelivery d;
    uint16_t vid, tpid;
};

Out classify(const std::vector<uint8_t>& f, bool aux_valid = false,
             uint16_t aux_tci = 0, uint16_t aux_tpid = 0) {
    Out o{};
    o.d = EtherCAT::classifyVlanReply(f.data(), f.size(), aux_valid,
                                      aux_tci, aux_tpid, o.vid, o.tpid);
    return o;
}

} // namespace

TEST(VlanClassifyTest, UntaggedEthercatReply) {
    const auto o = classify(frame(0x88A4));
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::Untagged);
    EXPECT_EQ(o.vid, 0u);
    EXPECT_EQ(o.tpid, 0u);
}

TEST(VlanClassifyTest, Inline8100Tag) {
    const auto o = classify(taggedFrame(0x8100, 1999));
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::InlineTag);
    EXPECT_EQ(o.vid, 1999u);
    EXPECT_EQ(o.tpid, 0x8100u);
}

TEST(VlanClassifyTest, InlineTagCarriesPcp) {
    // PCP=5 (0b101 << 13) + VID 1999 — VID must be masked to 12 bits.
    const auto o = classify(taggedFrame(0x8100, (5u << 13) | 1999));
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::InlineTag);
    EXPECT_EQ(o.vid, 1999u);
}

TEST(VlanClassifyTest, InlineAlternativeTpids) {
    for (uint16_t tpid : {0x88A8, 0x9100, 0x9200}) {
        const auto o = classify(taggedFrame(tpid, 100));
        EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::InlineTag) << tpid;
        EXPECT_EQ(o.tpid, tpid);
    }
}

TEST(VlanClassifyTest, StrippedTagFromAuxdata) {
    // Kernel-stripped: data shows inner EtherType at [12], VID in auxdata.
    const auto o = classify(frame(0x88A4), /*aux_valid=*/true,
                            /*aux_tci=*/1999, /*aux_tpid=*/0x8100);
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::StrippedTag);
    EXPECT_EQ(o.vid, 1999u);
    EXPECT_EQ(o.tpid, 0x8100u);
}

TEST(VlanClassifyTest, StrippedTagWithoutAuxTpid) {
    // Kernels without TP_STATUS_VLAN_TPID_VALID report no aux TPID.
    const auto o = classify(frame(0x88A4), true, 1999, 0);
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::StrippedTag);
    EXPECT_EQ(o.vid, 1999u);
    EXPECT_EQ(o.tpid, 0u);
}

TEST(VlanClassifyTest, InlineTagWinsOverAuxdata) {
    // Double-tagged with only the outer stripped: data still shows an
    // inline tag — report the inline presentation (the inner tag).
    const auto o = classify(taggedFrame(0x8100, 5), true, 1999, 0x8100);
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::InlineTag);
    EXPECT_EQ(o.vid, 5u);
}

TEST(VlanClassifyTest, TruncatedAndEmpty) {
    uint16_t vid = 9, tpid = 9;
    EXPECT_EQ(EtherCAT::classifyVlanReply(nullptr, 0, false, 0, 0, vid, tpid),
              EtherCAT::VlanTagDelivery::NoReply);
    const std::vector<uint8_t> tiny(13, 0x88);
    const auto o = classify(tiny);
    EXPECT_EQ(o.d, EtherCAT::VlanTagDelivery::NoReply);
    // TPID at [12] but frame ends before the TCI — not a valid inline tag.
    const std::vector<uint8_t> trunc(16, 0);
    const_cast<uint8_t&>(trunc[12]) = 0x81;
    const_cast<uint8_t&>(trunc[13]) = 0x00;
    const auto o2 = classify(trunc);
    EXPECT_EQ(o2.d, EtherCAT::VlanTagDelivery::Untagged);
}

TEST(VlanClassifyTest, ToStringCoversAll) {
    EXPECT_STREQ(EtherCAT::toString(EtherCAT::VlanTagDelivery::NoReply),
                 "no-reply");
    EXPECT_STREQ(EtherCAT::toString(EtherCAT::VlanTagDelivery::Untagged),
                 "untagged");
    EXPECT_STREQ(EtherCAT::toString(EtherCAT::VlanTagDelivery::InlineTag),
                 "inline-tag");
    EXPECT_STREQ(EtherCAT::toString(EtherCAT::VlanTagDelivery::StrippedTag),
                 "stripped-tag");
}
