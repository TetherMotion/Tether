/**
 * @file test_coe_interpreter.cpp
 * @brief Coverage for PacketInterpreters::CoEPacketInterpreter — mailbox
 *        header decode, CoE service names, every SDO command specifier
 *        branch (initiate/segmented/abort, expedited vs. sized), and all
 *        truncation paths.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "tether/packet_interpreters/CoE.hpp"

using EtherCAT::PacketInterpreters::CoEPacketInterpreter;

namespace {

void put16le(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>(x >> 8));
}
void put32le(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));
}

// Mailbox header: len(u16) | addr(u16) | ch[6]+prio[2] | type[4]+cnt[4]
std::vector<uint8_t> mbx(uint8_t type, uint8_t cnt = 0, uint8_t ch = 1,
                         uint8_t prio = 0, uint16_t addr = 0x1000) {
    std::vector<uint8_t> v;
    put16le(v, 0);            // length field (informational in output)
    put16le(v, addr);
    v.push_back(static_cast<uint8_t>((prio << 6) | (ch & 0x3F)));
    v.push_back(static_cast<uint8_t>((cnt << 4) | (type & 0x0F)));
    return v;
}

void appendCoe(std::vector<uint8_t>& v, uint8_t service, uint16_t number = 0) {
    put16le(v, static_cast<uint16_t>((service << 12) | (number & 0x01FF)));
}

std::string str(const std::vector<uint8_t>& v) {
    return CoEPacketInterpreter(v.data(), v.size()).toString();
}

// Build an SDO packet: mailbox(CoE) + CoE header + sdo bytes.
std::vector<uint8_t> sdoPacket(uint8_t service, uint8_t cmd,
                               const std::vector<uint8_t>& rest = {}) {
    auto v = mbx(0x03);
    appendCoe(v, service);
    v.push_back(cmd);
    v.insert(v.end(), rest.begin(), rest.end());
    return v;
}

} // namespace

TEST(CoEInterpreter, TruncatedMailboxHeader) {
    EXPECT_NE(str({0x01, 0x02}).find("Truncated packet"), std::string::npos);
}

TEST(CoEInterpreter, MailboxTypeNames) {
    const struct { uint8_t t; const char* name; } cases[] = {
        {0x00, "ERR"}, {0x01, "AOE"}, {0x02, "EOE"}, {0x03, "CoE"},
        {0x04, "FoE"}, {0x05, "SoE"}, {0x0F, "VoE"}, {0x09, "UNKNOWN"},
    };
    for (const auto& c : cases) {
        auto s = str(mbx(c.t));
        EXPECT_NE(s.find(std::string("type=") + c.name), std::string::npos)
            << "type " << int(c.t);
    }
}

TEST(CoEInterpreter, NonCoEMailboxStopsEarly) {
    auto s = str(mbx(0x04));
    EXPECT_NE(s.find("(Not a CoE packet)"), std::string::npos);
    // Channel/priority/counter fields are decoded too.
    auto s2 = str(mbx(0x04, /*cnt=*/5, /*ch=*/9, /*prio=*/2));
    EXPECT_NE(s2.find("cnt=0x05"), std::string::npos);
    EXPECT_NE(s2.find("ch=9"), std::string::npos);
    EXPECT_NE(s2.find("prio=2"), std::string::npos);
}

TEST(CoEInterpreter, TruncatedCoeHeader) {
    auto s = str(mbx(0x03));  // mailbox only, no CoE header
    EXPECT_NE(s.find("missing CoE header"), std::string::npos);
}

TEST(CoEInterpreter, NoSdoData) {
    auto v = mbx(0x03);
    appendCoe(v, 0x02);
    EXPECT_NE(str(v).find("(No SDO data)"), std::string::npos);
}

TEST(CoEInterpreter, CoeServiceNames) {
    const struct { uint8_t s; const char* name; } cases[] = {
        {0x01, "EMERGENCY"}, {0x02, "SDO-REQ"}, {0x03, "SDO-RES"},
        {0x04, "TX-PDO"},    {0x05, "RX-PDO"}, {0x06, "TX-PDO-REMOTE"},
        {0x07, "RX-PDO-REMOTE"}, {0x08, "SDO-INFO"}, {0x09, "UNKNOWN"},
    };
    for (const auto& c : cases) {
        auto v = mbx(0x03);
        appendCoe(v, c.s, 0x1AB);
        auto s = str(v);
        EXPECT_NE(s.find(std::string("service=") + c.name), std::string::npos)
            << "service " << int(c.s);
        EXPECT_NE(s.find("number=0x1AB"), std::string::npos);
    }
}

TEST(CoEInterpreter, NonSdoServiceStopsAtServiceLine) {
    auto v = sdoPacket(0x04, 0xFF);
    EXPECT_NE(str(v).find("(Non-SDO CoE service"), std::string::npos);
}

// --- SDO request command specifiers -----------------------------------------

TEST(CoEInterpreter, DownloadInitRequestExpedited) {
    // cmd 0x23: ccs=1 (DOWNLOAD-INIT-REQ), e=1, s=1
    std::vector<uint8_t> rest;
    put16le(rest, 0x6060);
    rest.push_back(0x00);
    put32le(rest, 0xDEADBEEF);
    auto s = str(sdoPacket(0x02, 0x23, rest));
    EXPECT_NE(s.find("DOWNLOAD-INIT-REQ"), std::string::npos);
    EXPECT_NE(s.find("index=0x6060"), std::string::npos);
    EXPECT_NE(s.find("data=0xDEADBEEF"), std::string::npos);
    EXPECT_NE(s.find("e=1 s=1"), std::string::npos);
}

TEST(CoEInterpreter, DownloadInitRequestNoDataWhenNotExpedited) {
    std::vector<uint8_t> rest;
    put16le(rest, 0x6060);
    rest.push_back(0x00);
    put32le(rest, 0xDEADBEEF);
    auto s = str(sdoPacket(0x02, 0x21, rest));  // e=0
    EXPECT_NE(s.find("DOWNLOAD-INIT-REQ"), std::string::npos);
    EXPECT_EQ(s.find("data="), std::string::npos);
}

TEST(CoEInterpreter, UploadInitRequest) {
    std::vector<uint8_t> rest;
    put16le(rest, 0x1008);
    rest.push_back(0x00);
    put32le(rest, 0);
    auto s = str(sdoPacket(0x02, 0x40, rest));  // ccs=2
    EXPECT_NE(s.find("UPLOAD-INIT-REQ"), std::string::npos);
    EXPECT_NE(s.find("index=0x1008"), std::string::npos);
}

TEST(CoEInterpreter, DownloadSegmentRequest) {
    // ccs=0, toggle bit set, last set, seg_n=2 → 5 bytes
    auto s = str(sdoPacket(0x02, 0x15, {1, 2, 3, 4, 5}));
    EXPECT_NE(s.find("DOWNLOAD-SEG-REQ"), std::string::npos);
    EXPECT_NE(s.find("toggle=1"), std::string::npos);
    EXPECT_NE(s.find("last=1"), std::string::npos);
    EXPECT_NE(s.find("seg_bytes=5"), std::string::npos);
}

TEST(CoEInterpreter, UploadSegmentRequest) {
    auto s = str(sdoPacket(0x02, 0x60));  // ccs=3
    EXPECT_NE(s.find("UPLOAD-SEG-REQ"), std::string::npos);
    EXPECT_NE(s.find("toggle=0 last=0 seg_bytes=7"), std::string::npos);
}

TEST(CoEInterpreter, AbortRequestDecodesCode) {
    std::vector<uint8_t> rest;
    put16le(rest, 0x6060);
    rest.push_back(0x01);
    put32le(rest, 0x06020000);
    auto s = str(sdoPacket(0x02, 0x80, rest));  // ccs=4
    EXPECT_NE(s.find("ABORT"), std::string::npos);
    EXPECT_NE(s.find("index=0x6060 sub=0x01 code=0x06020000"), std::string::npos);
}

TEST(CoEInterpreter, TruncatedAbort) {
    auto s = str(sdoPacket(0x02, 0x80, {0x60}));
    EXPECT_NE(s.find("truncated abort"), std::string::npos);
}

TEST(CoEInterpreter, UnknownRequestCcs) {
    auto s = str(sdoPacket(0x02, 0xA0));  // ccs=5
    EXPECT_NE(s.find("cmd=UNKNOWN"), std::string::npos);
}

// --- SDO response command specifiers ----------------------------------------

TEST(CoEInterpreter, UploadInitResponseExpedited) {
    // cmd 0x47: ccs=2, e=1, s=1, n=1 → 3 data bytes
    std::vector<uint8_t> rest;
    put16le(rest, 0x1008);
    rest.push_back(0x00);
    put32le(rest, 0x00414243);
    auto s = str(sdoPacket(0x03, 0x47, rest));
    EXPECT_NE(s.find("UPLOAD-INIT-RES"), std::string::npos);
    EXPECT_NE(s.find("index=0x1008"), std::string::npos);
    EXPECT_NE(s.find("data=0x00414243 (3 bytes)"), std::string::npos);
}

TEST(CoEInterpreter, UploadInitResponseSize) {
    // cmd 0x41: ccs=2, e=0, s=1 → size field
    std::vector<uint8_t> rest;
    put16le(rest, 0x1008);
    rest.push_back(0x00);
    put32le(rest, 0x200);
    auto s = str(sdoPacket(0x03, 0x41, rest));
    EXPECT_NE(s.find("size=512"), std::string::npos);
}

TEST(CoEInterpreter, UploadInitResponseNotExpeditedNotSized) {
    std::vector<uint8_t> rest;
    put16le(rest, 0x1008);
    rest.push_back(0x00);
    put32le(rest, 0x200);
    auto s = str(sdoPacket(0x03, 0x40, rest));  // e=0 s=0
    EXPECT_NE(s.find("UPLOAD-INIT-RES"), std::string::npos);
    EXPECT_EQ(s.find("data="), std::string::npos);
    EXPECT_EQ(s.find("size="), std::string::npos);
}

TEST(CoEInterpreter, DownloadInitResponse) {
    std::vector<uint8_t> rest;
    put16le(rest, 0x6060);
    rest.push_back(0x00);
    put32le(rest, 0);
    auto s = str(sdoPacket(0x03, 0x60, rest));  // ccs=3
    EXPECT_NE(s.find("DOWNLOAD-INIT-RES"), std::string::npos);
    EXPECT_NE(s.find("index=0x6060"), std::string::npos);
}

TEST(CoEInterpreter, UploadSegmentResponse) {
    auto s = str(sdoPacket(0x03, 0x1F, {9, 9, 9}));  // ccs=0, seg_n=7 → 0 bytes
    EXPECT_NE(s.find("UPLOAD-SEG-RES"), std::string::npos);
    EXPECT_NE(s.find("seg_bytes=0"), std::string::npos);
}

TEST(CoEInterpreter, DownloadSegmentResponse) {
    auto s = str(sdoPacket(0x03, 0x20));  // ccs=1
    EXPECT_NE(s.find("DOWNLOAD-SEG-RES"), std::string::npos);
}

TEST(CoEInterpreter, AbortResponse) {
    std::vector<uint8_t> rest;
    put16le(rest, 0x1800);
    rest.push_back(0x02);
    put32le(rest, 0x08000000);
    auto s = str(sdoPacket(0x03, 0x80, rest));
    EXPECT_NE(s.find("ABORT"), std::string::npos);
    EXPECT_NE(s.find("code=0x08000000"), std::string::npos);
}

TEST(CoEInterpreter, InitTransferIndexOmittedWhenShort) {
    // ccs=1 download-init but only 2 bytes after cmd — no index printed.
    auto s = str(sdoPacket(0x02, 0x23, {0x60, 0x60}));
    EXPECT_NE(s.find("DOWNLOAD-INIT-REQ"), std::string::npos);
    EXPECT_EQ(s.find("index="), std::string::npos);
}
