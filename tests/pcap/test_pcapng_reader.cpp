/**
 * @file test_pcapng_reader.cpp
 * @brief Unit tests for the pcapng reader and EtherCAT interpreter
 */

#include <gtest/gtest.h>

#include "tether/ethercat/Types.hpp"
#include "tether/packetloggers/pcap/PCAPNGReader.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace Tether::PacketLoggers::PCAP;
using namespace EtherCAT;

namespace {

// Append little-endian multi-byte values to a byte vector.
void appendU8(std::vector<uint8_t>& out, uint8_t v) { out.push_back(v); }
void appendU16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v));
    out.push_back(static_cast<uint8_t>(v >> 8));
}
void appendU32(std::vector<uint8_t>& out, uint32_t v) {
    appendU16(out, static_cast<uint16_t>(v));
    appendU16(out, static_cast<uint16_t>(v >> 16));
}
void appendU64(std::vector<uint8_t>& out, uint64_t v) {
    appendU32(out, static_cast<uint32_t>(v));
    appendU32(out, static_cast<uint32_t>(v >> 32));
}
void appendU16BE(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}
void appendBytes(std::vector<uint8_t>& out, const uint8_t* data, size_t len) {
    out.insert(out.end(), data, data + len);
}
void appendStringOption(std::vector<uint8_t>& opts, uint16_t code, const std::string& str) {
    appendU16(opts, code);
    appendU16(opts, static_cast<uint16_t>(str.size()));
    appendBytes(opts, reinterpret_cast<const uint8_t*>(str.data()), str.size());
    while (opts.size() % 4 != 0) opts.push_back(0);
}
void appendEndOption(std::vector<uint8_t>& opts) {
    appendU16(opts, 0);
    appendU16(opts, 0);
}

void appendBlockHeader(std::vector<uint8_t>& out, uint32_t type,
                       const std::vector<uint8_t>& body) {
    uint32_t totalLength = static_cast<uint32_t>(4 + 4 + body.size() + 4);
    appendU32(out, type);
    appendU32(out, totalLength);
    appendBytes(out, body.data(), body.size());
    // Pad body to 32-bit boundary (already done by options padding, but be safe).
    while ((out.size() + 4) % 4 != 0) out.push_back(0);
    appendU32(out, totalLength);
}

std::vector<uint8_t> makeSectionHeaderBlock(const std::string& hardware = "",
                                            const std::string& os = "",
                                            const std::string& app = "") {
    std::vector<uint8_t> body;
    appendU32(body, 0x1A2B3C4D); // byte order magic
    appendU16(body, 1);          // major version
    appendU16(body, 0);          // minor version
    appendU64(body, 0xFFFFFFFFFFFFFFFFull); // section length unknown

    std::vector<uint8_t> opts;
    if (!hardware.empty()) appendStringOption(opts, PCAPNG::SHB_HARDWARE, hardware);
    if (!os.empty()) appendStringOption(opts, PCAPNG::SHB_OS, os);
    if (!app.empty()) appendStringOption(opts, PCAPNG::SHB_USERAPPL, app);
    appendEndOption(opts);
    appendBytes(body, opts.data(), opts.size());

    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_SHB, body);
    return out;
}

std::vector<uint8_t> makeInterfaceDescriptionBlock(uint16_t linkType = 1,
                                                   uint32_t snapLen = 65535,
                                                   const std::string& name = "") {
    std::vector<uint8_t> body;
    appendU16(body, linkType);
    appendU16(body, 0); // reserved
    appendU32(body, snapLen);

    std::vector<uint8_t> opts;
    if (!name.empty()) appendStringOption(opts, PCAPNG::IDB_NAME, name);
    appendEndOption(opts);
    appendBytes(body, opts.data(), opts.size());

    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_IDB, body);
    return out;
}

// Build an IPv4/UDP/EtherCAT-over-UDP packet with no Ethernet header (raw IP).
std::vector<uint8_t> makeRawIpv4UdpEtherCAT(const std::vector<uint8_t>& ecatPayload,
                                            uint16_t dstPort = 0x88A4) {
    std::vector<uint8_t> ip(20, 0);
    ip[0] = 0x45;  // version=4, IHL=5
    ip[9] = 0x11;  // protocol = UDP
    ip[12] = 192; ip[13] = 168; ip[14] = 1; ip[15] = 1;
    ip[16] = 192; ip[17] = 168; ip[18] = 1; ip[19] = 255;
    uint16_t udpLen = static_cast<uint16_t>(8 + ecatPayload.size());
    uint16_t ipTotalLen = static_cast<uint16_t>(20 + udpLen);
    ip[2] = static_cast<uint8_t>(ipTotalLen >> 8);
    ip[3] = static_cast<uint8_t>(ipTotalLen & 0xFF);

    std::vector<uint8_t> pkt;
    appendBytes(pkt, ip.data(), ip.size());
    appendU16BE(pkt, 0x1234);       // src port
    appendU16BE(pkt, dstPort);      // dst port
    appendU16BE(pkt, udpLen);       // UDP length
    appendU16BE(pkt, 0);            // checksum
    appendBytes(pkt, ecatPayload.data(), ecatPayload.size());
    return pkt;
}

// Build a Linux SLL header + raw IPv4/UDP/EtherCAT-over-UDP payload.
std::vector<uint8_t> makeLinuxSllEtherCAT(const std::vector<uint8_t>& ipPayload,
                                          uint16_t etherType = 0x0800) {
    std::vector<uint8_t> sll;
    appendU16BE(sll, 0);            // packet type: unicast
    appendU16BE(sll, 1);            // ARPHRD: Ethernet
    appendU16BE(sll, 6);            // address length
    appendBytes(sll, std::array<uint8_t,8>{0x00,0x11,0x22,0x33,0x44,0x55,0,0}.data(), 8);
    appendU16BE(sll, etherType);    // protocol
    appendBytes(sll, ipPayload.data(), ipPayload.size());
    return sll;
}

// IDB with an if_fcslen option declaring trailing FCS bytes.
std::vector<uint8_t> makeInterfaceDescriptionBlockWithFcs(uint8_t fcsLen,
                                                          uint16_t linkType = 1) {
    std::vector<uint8_t> body;
    appendU16(body, linkType);
    appendU16(body, 0); // reserved
    appendU32(body, 65535);

    std::vector<uint8_t> opts;
    appendU16(opts, PCAPNG::IDB_FCSLEN);
    appendU16(opts, 1);
    appendU8(opts, fcsLen);
    appendU8(opts, 0); appendU8(opts, 0); appendU8(opts, 0); // pad to 4
    appendEndOption(opts);
    appendBytes(body, opts.data(), opts.size());

    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_IDB, body);
    return out;
}

// IDB with a custom if_tsresol option (power-of-ten resolution).
std::vector<uint8_t> makeInterfaceDescriptionBlockWithTsResol(uint8_t tsResol,
                                                              uint16_t linkType = 1) {
    std::vector<uint8_t> body;
    appendU16(body, linkType);
    appendU16(body, 0); // reserved
    appendU32(body, 65535);

    std::vector<uint8_t> opts;
    appendU16(opts, PCAPNG::IDB_TSRESOL);
    appendU16(opts, 1);
    appendU8(opts, tsResol);
    appendU8(opts, 0); appendU8(opts, 0); appendU8(opts, 0); // pad to 4
    appendEndOption(opts);
    appendBytes(body, opts.data(), opts.size());

    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_IDB, body);
    return out;
}

std::vector<uint8_t> makeEnhancedPacketBlock(uint32_t interfaceId,
                                             uint64_t timestamp,
                                             const uint8_t* packetData,
                                             size_t packetLen,
                                             const std::vector<uint8_t>& options = {}) {
    std::vector<uint8_t> body;
    appendU32(body, interfaceId);
    appendU32(body, static_cast<uint32_t>(timestamp >> 32));
    appendU32(body, static_cast<uint32_t>(timestamp & 0xFFFFFFFFull));
    appendU32(body, static_cast<uint32_t>(packetLen));
    appendU32(body, static_cast<uint32_t>(packetLen));

    appendBytes(body, packetData, packetLen);
    while (body.size() % 4 != 0) body.push_back(0);

    appendBytes(body, options.data(), options.size());

    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_EPB, body);
    return out;
}

// Build an obsolete Packet Block (type 0x00000002).
std::vector<uint8_t> makePacketBlock(uint16_t interfaceId,
                                     uint16_t drops,
                                     uint32_t timestampSeconds,
                                     const uint8_t* packetData,
                                     size_t packetLen,
                                     const std::vector<uint8_t>& options = {}) {
    std::vector<uint8_t> body;
    appendU16(body, interfaceId);
    appendU16(body, drops);
    appendU32(body, timestampSeconds);
    appendU32(body, static_cast<uint32_t>(packetLen));
    appendU32(body, static_cast<uint32_t>(packetLen));

    appendBytes(body, packetData, packetLen);
    while (body.size() % 4 != 0) body.push_back(0);

    appendBytes(body, options.data(), options.size());

    std::vector<uint8_t> out;
    appendBlockHeader(out, 0x00000002, body);
    return out;
}

// Build an Interface Statistics Block.
std::vector<uint8_t> makeInterfaceStatsBlock(uint32_t interfaceId,
                                             uint64_t timestamp,
                                             const std::vector<uint8_t>& options = {}) {
    std::vector<uint8_t> body;
    appendU32(body, interfaceId);
    appendU32(body, static_cast<uint32_t>(timestamp >> 32));
    appendU32(body, static_cast<uint32_t>(timestamp & 0xFFFFFFFFull));
    appendBytes(body, options.data(), options.size());
    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_ISB, body);
    return out;
}

// Build a Name Resolution Block with IPv4 records.
std::vector<uint8_t> makeNameResolutionBlock(
    const std::vector<std::pair<std::array<uint8_t,4>, std::string>>& ipv4Records,
    const std::string& comment = "") {
    std::vector<uint8_t> body;
    for (const auto& [ip, name] : ipv4Records) {
        appendU16(body, 1); // nrb_ipv4
        uint16_t recLen = static_cast<uint16_t>(4 + name.size());
        appendU16(body, recLen);
        appendBytes(body, ip.data(), 4);
        appendBytes(body, reinterpret_cast<const uint8_t*>(name.data()), name.size());
        while (body.size() % 4 != 0) body.push_back(0);
    }
    // nrb_end_record
    appendU16(body, 0);
    appendU16(body, 0);
    // Optional comment
    if (!comment.empty()) {
        appendStringOption(body, PCAPNG::OPT_COMMENT, comment);
    }
    appendEndOption(body);
    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_NRB, body);
    return out;
}

// Build a Decryption Secrets Block.
std::vector<uint8_t> makeDecryptionSecretsBlock(uint16_t secretsType,
                                                const std::vector<uint8_t>& secretsData,
                                                const std::string& comment = "") {
    std::vector<uint8_t> body;
    appendU16(body, secretsType);
    appendU16(body, static_cast<uint16_t>(secretsData.size()));
    appendBytes(body, secretsData.data(), secretsData.size());
    while (body.size() % 4 != 0) body.push_back(0);
    if (!comment.empty()) {
        appendStringOption(body, PCAPNG::OPT_COMMENT, comment);
    }
    appendEndOption(body);
    std::vector<uint8_t> out;
    appendBlockHeader(out, PCAPNG::BLOCK_TYPE_DSB, body);
    return out;
}

// Build a raw Ethernet + EtherCAT frame with one APRD datagram.
std::vector<uint8_t> makeEtherCATFrame(const std::array<uint8_t, 6>& dst,
                                       const std::array<uint8_t, 6>& src,
                                       std::optional<uint16_t> vlanId = std::nullopt,
                                       uint16_t irq = 0) {
    std::vector<uint8_t> frame;
    appendBytes(frame, dst.data(), 6);
    appendBytes(frame, src.data(), 6);

    if (vlanId.has_value()) {
        appendU16BE(frame, 0x8100); // TPID
        uint16_t tci = *vlanId & 0x0FFF;
        appendU16BE(frame, tci);    // TCI (big-endian on wire)
    }

    appendU16BE(frame, 0x88A4); // EtherType (big-endian)

    // One APRD datagram: read AL Status (0x0130) at position 0.
    uint8_t cmd = static_cast<uint8_t>(Command::APRD);
    uint8_t idx = 0xAB;
    uint16_t adp = 0x0000;
    uint16_t ado = 0x0130;
    uint16_t dataLen = 2;
    std::vector<uint8_t> payload = {0x04, 0x00}; // SAFE-OP
    uint16_t wkc = 0x0001;

    uint16_t ecatLen = static_cast<uint16_t>(sizeof(DatagramHeader) + dataLen + sizeof(uint16_t));
    appendU8(frame, static_cast<uint8_t>(ecatLen));
    appendU8(frame, static_cast<uint8_t>(0x10 | ((ecatLen >> 8) & 0x07))); // type=1

    appendU8(frame, cmd);
    appendU8(frame, idx);
    appendU16(frame, adp);
    appendU16(frame, ado);
    appendU16(frame, dataLen); // lenFlags: length, no M/C
    appendU16(frame, irq);     // irq
    appendBytes(frame, payload.data(), payload.size());
    appendU16(frame, wkc);

    return frame;
}

// Build the EtherCAT frame payload only (no Ethernet header) — used as the
// UDP payload for EtherCAT-over-UDP encapsulation tests.
std::vector<uint8_t> makeEtherCATPayloadOnly() {
    std::vector<uint8_t> ecat;
    uint16_t dataLen = 2;
    std::vector<uint8_t> payload = {0x04, 0x00}; // SAFE-OP
    uint16_t wkc = 0x0001;
    uint16_t ecatLen = static_cast<uint16_t>(sizeof(DatagramHeader) + dataLen + sizeof(uint16_t));
    appendU8(ecat, static_cast<uint8_t>(ecatLen));
    appendU8(ecat, static_cast<uint8_t>(0x10 | ((ecatLen >> 8) & 0x07))); // type=1
    appendU8(ecat, static_cast<uint8_t>(Command::APRD));
    appendU8(ecat, 0xAB);
    appendU16(ecat, 0x0000);
    appendU16(ecat, 0x0130);
    appendU16(ecat, dataLen);
    appendU16(ecat, 0); // irq
    appendBytes(ecat, payload.data(), payload.size());
    appendU16(ecat, wkc);
    return ecat;
}

// Build an Ethernet/IPv4/UDP frame carrying an EtherCAT-over-UDP payload.
std::vector<uint8_t> makeEtherCATOverUdpFrame(const std::array<uint8_t, 6>& dst,
                                              const std::array<uint8_t, 6>& src,
                                              const std::vector<uint8_t>& ecatPayload,
                                              uint16_t dstPort = 0x88A4) {
    std::vector<uint8_t> frame;
    appendBytes(frame, dst.data(), 6);
    appendBytes(frame, src.data(), 6);
    appendU16BE(frame, 0x0800); // EtherType = IPv4

    // IPv4 header (20 bytes, no options).
    std::vector<uint8_t> ip(20, 0);
    ip[0] = 0x45;  // version=4, IHL=5
    ip[9] = 0x11;  // protocol = UDP
    // src IP = 192.168.1.1, dst IP = 192.168.1.255 (big-endian / network order)
    ip[12] = 192; ip[13] = 168; ip[14] = 1; ip[15] = 1;
    ip[16] = 192; ip[17] = 168; ip[18] = 1; ip[19] = 255;
    uint16_t udpLen = static_cast<uint16_t>(8 + ecatPayload.size());
    uint16_t ipTotalLen = static_cast<uint16_t>(20 + udpLen);
    ip[2] = static_cast<uint8_t>(ipTotalLen >> 8);
    ip[3] = static_cast<uint8_t>(ipTotalLen & 0xFF);
    appendBytes(frame, ip.data(), ip.size());

    // UDP header (8 bytes, big-endian fields).
    appendU16BE(frame, 0x1234);        // src port
    appendU16BE(frame, dstPort);       // dst port (0x88A4 = EtherCAT-over-UDP)
    appendU16BE(frame, udpLen);        // UDP length
    appendU16BE(frame, 0);             // checksum (0 = not computed)

    // EtherCAT payload.
    appendBytes(frame, ecatPayload.data(), ecatPayload.size());
    return frame;
}

std::vector<uint8_t> makeBigEndianSectionHeaderBlock() {
    // Helper to append big-endian values.
    auto appendU16BE = [](std::vector<uint8_t>& out, uint16_t v) {
        out.push_back(static_cast<uint8_t>(v >> 8));
        out.push_back(static_cast<uint8_t>(v));
    };
    auto appendU32BE = [&](std::vector<uint8_t>& out, uint32_t v) {
        appendU16BE(out, static_cast<uint16_t>(v >> 16));
        appendU16BE(out, static_cast<uint16_t>(v));
    };
    auto appendU64BE = [&](std::vector<uint8_t>& out, uint64_t v) {
        appendU32BE(out, static_cast<uint32_t>(v >> 32));
        appendU32BE(out, static_cast<uint32_t>(v));
    };

    std::vector<uint8_t> body;
    appendU32BE(body, 0x1A2B3C4D); // swapped magic on little-endian host
    appendU16BE(body, 1);
    appendU16BE(body, 0);
    appendU64BE(body, 0xFFFFFFFFFFFFFFFFull);

    // No options.
    std::vector<uint8_t> out;
    uint32_t totalLength = static_cast<uint32_t>(4 + 4 + body.size() + 4);
    appendU32BE(out, PCAPNG::BLOCK_TYPE_SHB);
    appendU32BE(out, totalLength);
    appendBytes(out, body.data(), body.size());
    appendU32BE(out, totalLength);
    return out;
}

} // anonymous namespace

// ============================================================================
// Basic open/read tests
// ============================================================================

TEST(PCAPNGReader, OpenMissingFile) {
    PCAPNGReader reader;
    EXPECT_FALSE(reader.open("/nonexistent/path/to/file.pcapng"));
}

TEST(PCAPNGReader, EmptyBufferFails) {
    PCAPNGReader reader;
    EXPECT_FALSE(reader.open(std::vector<uint8_t>{}));
}

TEST(PCAPNGReader, ReadSimpleEthernetFrame) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock("TestHW", "TestOS", "TestApp");
    auto idb = makeInterfaceDescriptionBlock(1, 65535, "eth0");
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    std::vector<uint8_t> ethFrame;
    appendBytes(ethFrame, dst.data(), 6);
    appendBytes(ethFrame, src.data(), 6);
    appendU16BE(ethFrame, 0x0800); // IPv4 (big-endian)
    appendBytes(ethFrame, std::vector<uint8_t>(20, 0).data(), 20); // fake IPv4 header

    auto epb = makeEnhancedPacketBlock(0, 123456789, ethFrame.data(), ethFrame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));

    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(reader.sectionInfo().hardware, "TestHW");
    EXPECT_EQ(reader.sectionInfo().os, "TestOS");
    EXPECT_EQ(reader.sectionInfo().application, "TestApp");
    ASSERT_EQ(reader.interfaces().size(), 1u);
    EXPECT_EQ(reader.interfaces()[0].name, "eth0");

    EXPECT_EQ(frames[0].timestampNs, 123456789u);
    EXPECT_EQ(frames[0].capturedLength, ethFrame.size());
    EXPECT_EQ(frames[0].dstMac, dst);
    EXPECT_EQ(frames[0].srcMac, src);
    EXPECT_EQ(frames[0].innerEtherType, 0x0800u);
    EXPECT_FALSE(frames[0].isEtherCAT);
}

TEST(PCAPNGReader, ReadEtherCATFrameWithoutVlan) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);

    auto epb = makeEnhancedPacketBlock(0, 1000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_TRUE(frames[0].isEtherCAT);
    EXPECT_FALSE(frames[0].vlanId.has_value());
    ASSERT_EQ(frames[0].datagrams.size(), 1u);

    const auto& dg = frames[0].datagrams[0];
    EXPECT_EQ(dg.cmd, Command::APRD);
    EXPECT_EQ(dg.idx, 0xAB);
    EXPECT_EQ(dg.adp, 0x0000);
    EXPECT_EQ(dg.ado, 0x0130);
    EXPECT_EQ(dg.dataLength, 2u);
    EXPECT_EQ(dg.wkc, 1u);
    ASSERT_EQ(dg.data.size(), 2u);
    EXPECT_EQ(dg.data[0], 0x04);
    EXPECT_EQ(dg.data[1], 0x00);
}

TEST(PCAPNGReader, ReadEtherCATFrameWithVlan) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, 100);

    auto epb = makeEnhancedPacketBlock(0, 2000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    ASSERT_TRUE(frames[0].vlanId.has_value());
    EXPECT_EQ(*frames[0].vlanId, 100u);
    EXPECT_TRUE(frames[0].isEtherCAT);
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    EXPECT_EQ(frames[0].datagrams[0].cmd, Command::APRD);
}

TEST(PCAPNGReader, ReadEtherCATOverUdpFrame) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto ecatPayload = makeEtherCATPayloadOnly();
    auto frame = makeEtherCATOverUdpFrame(dst, src, ecatPayload);

    auto epb = makeEnhancedPacketBlock(0, 5000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    // The frame should be recognised as EtherCAT carried over UDP.
    EXPECT_TRUE(frames[0].isEtherCAT);
    EXPECT_TRUE(frames[0].isEtherCATOverUDP);
    EXPECT_EQ(frames[0].innerEtherType, 0x0800u); // IPv4
    EXPECT_EQ(frames[0].dstPort, 0x88A4u);
    EXPECT_EQ(frames[0].srcPort, 0x1234u);
    // 192.168.1.1 = 0xC0A80101, 192.168.1.255 = 0xC0A801FF
    EXPECT_EQ(frames[0].srcIp, 0xC0A80101u);
    EXPECT_EQ(frames[0].dstIp, 0xC0A801FFu);

    // The embedded EtherCAT datagram must be decoded.
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    const auto& dg = frames[0].datagrams[0];
    EXPECT_EQ(dg.cmd, Command::APRD);
    EXPECT_EQ(dg.idx, 0xAB);
    EXPECT_EQ(dg.adp, 0x0000);
    EXPECT_EQ(dg.ado, 0x0130);
    EXPECT_EQ(dg.dataLength, 2u);
    EXPECT_EQ(dg.wkc, 1u);
    ASSERT_EQ(dg.data.size(), 2u);
    EXPECT_EQ(dg.data[0], 0x04);
    EXPECT_EQ(dg.data[1], 0x00);
}

TEST(PCAPNGReader, NonEtherCatUdpFrameNotMarked) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto ecatPayload = makeEtherCATPayloadOnly();
    // Use a non-EtherCAT UDP destination port (e.g. 12345).
    auto frame = makeEtherCATOverUdpFrame(dst, src, ecatPayload, 12345);

    auto epb = makeEnhancedPacketBlock(0, 6000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    // Not EtherCAT because the UDP port does not match.
    EXPECT_FALSE(frames[0].isEtherCAT);
    EXPECT_FALSE(frames[0].isEtherCATOverUDP);
}

TEST(PCAPNGReader, DirectionAndCustomOptions) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);

    // Build EPB options: flags=outbound, dropcount=5, custom slave=7, pdo=1, wc=3.
    std::vector<uint8_t> opts;
    appendU16(opts, PCAPNG::EPB_FLAGS);
    appendU16(opts, 4);
    appendU32(opts, 0x00000002); // outbound
    appendU16(opts, PCAPNG::EPB_DROPCOUNT);
    appendU16(opts, 8);
    appendU64(opts, 5);
    appendU16(opts, PCAPNG::EPB_ETHERCAT_SLAVE);
    appendU16(opts, 2);
    appendU16(opts, 7);
    appendU8(opts, 0); appendU8(opts, 0); // pad to 4 bytes
    appendU16(opts, PCAPNG::EPB_ETHERCAT_WC);
    appendU16(opts, 1);
    appendU8(opts, 3);
    appendU8(opts, 0); appendU8(opts, 0); appendU8(opts, 0); // pad
    appendU16(opts, PCAPNG::EPB_ETHERCAT_PDO);
    appendU16(opts, 1);
    appendU8(opts, 1);
    appendU8(opts, 0); appendU8(opts, 0); appendU8(opts, 0); // pad
    appendEndOption(opts);

    auto epb = makeEnhancedPacketBlock(0, 3000, frame.data(), frame.size(), opts);
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(frames[0].direction, PacketDirection::Outbound);
    EXPECT_EQ(frames[0].dropCount, 5u);
    EXPECT_EQ(frames[0].slaveAddress, 7u);
    EXPECT_TRUE(frames[0].isProcessData);
    EXPECT_EQ(frames[0].workingCounter, 3u);
}

TEST(PCAPNGReader, BigEndianSectionHeader) {
    auto shb = makeBigEndianSectionHeaderBlock();
    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(shb));
    EXPECT_TRUE(reader.readAll([](const InterpretedFrame&) {}));
    EXPECT_TRUE(reader.sectionInfo().byteOrderSwapped);
}

TEST(PCAPNGReader, FormatAndJson) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, 42);

    auto epb = makeEnhancedPacketBlock(0, 4000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    std::string text = formatInterpretedFrame(frames[0], true, 64);
    EXPECT_NE(text.find("APRD"), std::string::npos);
    EXPECT_NE(text.find("VLAN: 42"), std::string::npos);

    std::string json = frameToJson(frames[0]);
    EXPECT_NE(json.find("\"cmd\": \"APRD\""), std::string::npos);
    EXPECT_NE(json.find("\"vlanId\": 42"), std::string::npos);
}

// ============================================================================
// FCS stripping (if_fcslen)
// ============================================================================

TEST(PCAPNGReader, StripsFcsFromEtherCATFrame) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlockWithFcs(4); // 4-byte FCS
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);
    // Append 4 bogus FCS bytes that would corrupt datagram parsing if not stripped.
    frame.push_back(0xDE);
    frame.push_back(0xAD);
    frame.push_back(0xBE);
    frame.push_back(0xEF);

    auto epb = makeEnhancedPacketBlock(0, 1000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(frames[0].fcsLength, 4u);
    // frameData should have the 4 FCS bytes removed.
    EXPECT_EQ(frames[0].frameData.size(), frame.size() - 4u);
    // capturedLength stays as the raw on-wire value (incl. FCS).
    EXPECT_EQ(frames[0].capturedLength, frame.size());
    // The EtherCAT datagram must still decode correctly.
    EXPECT_TRUE(frames[0].isEtherCAT);
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    EXPECT_EQ(frames[0].datagrams[0].cmd, Command::APRD);
    EXPECT_EQ(frames[0].datagrams[0].wkc, 1u);
}

TEST(PCAPNGReader, NoFcsStrippingWhenFcsLenZero) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock(); // no if_fcslen option
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);

    auto epb = makeEnhancedPacketBlock(0, 1000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(frames[0].fcsLength, 0u);
    EXPECT_EQ(frames[0].frameData.size(), frame.size());
}

// ============================================================================
// Link-type dispatch (LINKTYPE_LINUX_SLL, LINKTYPE_RAW, LINKTYPE_NULL)
// ============================================================================

TEST(PCAPNGReader, LinuxSllLinkTypeDecodesEtherCATOverUdp) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock(113); // LINKTYPE_LINUX_SLL
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    auto ecatPayload = makeEtherCATPayloadOnly();
    auto rawIp = makeRawIpv4UdpEtherCAT(ecatPayload);
    auto sllFrame = makeLinuxSllEtherCAT(rawIp, 0x0800);

    auto epb = makeEnhancedPacketBlock(0, 7000, sllFrame.data(), sllFrame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(frames[0].linkType, 113u);
    EXPECT_TRUE(frames[0].isEtherCAT);
    EXPECT_TRUE(frames[0].isEtherCATOverUDP);
    EXPECT_EQ(frames[0].innerEtherType, 0x0800u);
    EXPECT_EQ(frames[0].dstPort, 0x88A4u);
    // SLL source MAC should be populated from the address field.
    EXPECT_EQ(frames[0].srcMac, (std::array<uint8_t,6>{0x00,0x11,0x22,0x33,0x44,0x55}));
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    EXPECT_EQ(frames[0].datagrams[0].cmd, Command::APRD);
}

TEST(PCAPNGReader, RawIpLinkTypeDecodesEtherCATOverUdp) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock(101); // LINKTYPE_RAW
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    auto ecatPayload = makeEtherCATPayloadOnly();
    auto rawIp = makeRawIpv4UdpEtherCAT(ecatPayload);

    auto epb = makeEnhancedPacketBlock(0, 8000, rawIp.data(), rawIp.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(frames[0].linkType, 101u);
    EXPECT_TRUE(frames[0].isEtherCAT);
    EXPECT_TRUE(frames[0].isEtherCATOverUDP);
    EXPECT_EQ(frames[0].innerEtherType, 0x0800u);
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    EXPECT_EQ(frames[0].datagrams[0].cmd, Command::APRD);
}

TEST(PCAPNGReader, UnknownLinkTypeNotInterpreted) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock(999); // unknown link type
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::vector<uint8_t> junk = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    auto epb = makeEnhancedPacketBlock(0, 9000, junk.data(), junk.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    EXPECT_EQ(frames[0].linkType, 999u);
    EXPECT_FALSE(frames[0].isEtherCAT);
    // frameData preserved as-is, no interpretation attempted.
    EXPECT_EQ(frames[0].frameData, junk);
}

// ============================================================================
// Multi-section SHB handling
// ============================================================================

TEST(PCAPNGReader, MultipleSectionsResetInterfaces) {
    std::vector<uint8_t> data;

    // Section 1: IDB with tsResol=9 (nanoseconds, default).
    auto shb1 = makeSectionHeaderBlock("HW1", "OS1", "App1");
    auto idb1 = makeInterfaceDescriptionBlockWithTsResol(9);
    appendBytes(data, shb1.data(), shb1.size());
    appendBytes(data, idb1.data(), idb1.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame1 = makeEtherCATFrame(dst, src, std::nullopt);
    // raw timestamp = 1000 ticks; with tsResol=9 (ns), timestampNs = 1000.
    auto epb1 = makeEnhancedPacketBlock(0, 1000, frame1.data(), frame1.size());
    appendBytes(data, epb1.data(), epb1.size());

    // Section 2: new SHB + IDB with tsResol=6 (microseconds).
    auto shb2 = makeSectionHeaderBlock("HW2", "OS2", "App2");
    auto idb2 = makeInterfaceDescriptionBlockWithTsResol(6);
    appendBytes(data, shb2.data(), shb2.size());
    appendBytes(data, idb2.data(), idb2.size());

    auto frame2 = makeEtherCATFrame(dst, src, std::nullopt);
    // raw timestamp = 1000 ticks; with tsResol=6 (us), timestampNs = 1000 * 1000.
    auto epb2 = makeEnhancedPacketBlock(0, 1000, frame2.data(), frame2.size());
    appendBytes(data, epb2.data(), epb2.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 2u);

    // Section 2's metadata should be the final section info.
    EXPECT_EQ(reader.sectionInfo().hardware, "HW2");
    EXPECT_EQ(reader.sectionInfo().os, "OS2");
    EXPECT_EQ(reader.sectionInfo().application, "App2");
    // Only the second section's IDB should remain.
    ASSERT_EQ(reader.interfaces().size(), 1u);
    EXPECT_EQ(reader.interfaces()[0].tsResol, 6u);

    // Frame 1: tsResol=9 → 1000 ns.
    EXPECT_EQ(frames[0].timestampNs, 1000u);
    // Frame 2: tsResol=6 → 1000 ticks * 1000 ns/tick = 1,000,000 ns.
    EXPECT_EQ(frames[1].timestampNs, 1000000u);
}

// ============================================================================
// Obsolete Packet Block (type 0x00000002)
// ============================================================================

TEST(PCAPNGReader, ObsoletePacketBlockDecodesEtherCAT) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);

    // PB with drops=3, timestamp=5 seconds.
    auto pb = makePacketBlock(0, 3, 5, frame.data(), frame.size());
    appendBytes(data, pb.data(), pb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);

    // Timestamp 5 seconds → 5,000,000,000 ns.
    EXPECT_EQ(frames[0].timestampNs, 5000000000ull);
    EXPECT_EQ(frames[0].dropCount, 3u);
    EXPECT_TRUE(frames[0].isEtherCAT);
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    EXPECT_EQ(frames[0].datagrams[0].cmd, Command::APRD);
}

// ============================================================================
// EtherCAT datagram IRQ field
// ============================================================================

TEST(PCAPNGReader, DatagramIrqFieldSurfaced) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt, 0x1234);

    auto epb = makeEnhancedPacketBlock(0, 1000, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    auto frames = reader.readAll();
    ASSERT_EQ(frames.size(), 1u);
    ASSERT_EQ(frames[0].datagrams.size(), 1u);
    EXPECT_EQ(frames[0].datagrams[0].irq, 0x1234u);

    // Verify IRQ appears in text and JSON output.
    std::string text = formatInterpretedFrame(frames[0], false, 0);
    EXPECT_NE(text.find("irq=0x1234"), std::string::npos);
    std::string json = frameToJson(frames[0]);
    EXPECT_NE(json.find("\"irq\": 4660"), std::string::npos); // 0x1234 = 4660
}

// ============================================================================
// ISB / NRB / DSB data surfacing
// ============================================================================

TEST(PCAPNGReader, InterfaceStatisticsBlockSurfaced) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    // ISB with options: ifrecv=100, ifdrop=5, comment.
    std::vector<uint8_t> opts;
    appendU16(opts, 4); // isb_ifrecv
    appendU16(opts, 8);
    appendU64(opts, 100);
    appendU16(opts, 5); // isb_ifdrop
    appendU16(opts, 8);
    appendU64(opts, 5);
    appendStringOption(opts, PCAPNG::OPT_COMMENT, "stats comment");
    appendEndOption(opts);

    auto isb = makeInterfaceStatsBlock(0, 12345, opts);
    appendBytes(data, isb.data(), isb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    ASSERT_TRUE(reader.readAll([](const InterpretedFrame&) {}));

    const auto& stats = reader.interfaceStats();
    ASSERT_EQ(stats.size(), 1u);
    EXPECT_EQ(stats[0].interfaceId, 0u);
    EXPECT_EQ(stats[0].timestampNs, 12345u);
    EXPECT_EQ(stats[0].ifRecv, 100u);
    EXPECT_EQ(stats[0].ifDrop, 5u);
    EXPECT_EQ(stats[0].comment, "stats comment");
}

TEST(PCAPNGReader, NameResolutionBlockSurfaced) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    appendBytes(data, shb.data(), shb.size());

    std::vector<std::pair<std::array<uint8_t,4>, std::string>> records = {
        {{192, 168, 1, 1}, "slave0"},
        {{10, 0, 0, 5}, "master"},
    };
    auto nrb = makeNameResolutionBlock(records, "nrb comment");
    appendBytes(data, nrb.data(), nrb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    ASSERT_TRUE(reader.readAll([](const InterpretedFrame&) {}));

    const auto& nr = reader.nameResolutionRecords();
    // 2 IPv4 records + 1 comment record.
    ASSERT_EQ(nr.size(), 3u);
    EXPECT_EQ(nr[0].type, PCAPNGNameResolutionRecord::Type::Ipv4);
    EXPECT_EQ(nr[0].ipv4, (std::array<uint8_t,4>{192, 168, 1, 1}));
    EXPECT_EQ(nr[0].name, "slave0");
    EXPECT_EQ(nr[1].type, PCAPNGNameResolutionRecord::Type::Ipv4);
    EXPECT_EQ(nr[1].ipv4, (std::array<uint8_t,4>{10, 0, 0, 5}));
    EXPECT_EQ(nr[1].name, "master");
    EXPECT_EQ(nr[2].type, PCAPNGNameResolutionRecord::Type::Comment);
    EXPECT_EQ(nr[2].name, "nrb comment");
}

TEST(PCAPNGReader, DecryptionSecretsBlockSurfaced) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    appendBytes(data, shb.data(), shb.size());

    std::vector<uint8_t> secrets = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    auto dsb = makeDecryptionSecretsBlock(0x0001, secrets, "tls key log");
    appendBytes(data, dsb.data(), dsb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    ASSERT_TRUE(reader.readAll([](const InterpretedFrame&) {}));

    const auto& ds = reader.decryptionSecrets();
    ASSERT_EQ(ds.size(), 1u);
    EXPECT_EQ(ds[0].secretsType, 0x0001u);
    EXPECT_EQ(ds[0].secretsData, secrets);
    EXPECT_EQ(ds[0].comment, "tls key log");
}

// ============================================================================
// Streaming reader API (readNext / reset)
// ============================================================================

TEST(PCAPNGReader, ReadNextYieldsFramesIncrementally) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame1 = makeEtherCATFrame(dst, src, std::nullopt);
    auto frame2 = makeEtherCATFrame(dst, src, std::nullopt);

    auto epb1 = makeEnhancedPacketBlock(0, 100, frame1.data(), frame1.size());
    auto epb2 = makeEnhancedPacketBlock(0, 200, frame2.data(), frame2.size());
    appendBytes(data, epb1.data(), epb1.size());
    appendBytes(data, epb2.data(), epb2.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));

    InterpretedFrame f;
    ASSERT_TRUE(reader.readNext(f));
    EXPECT_EQ(f.timestampNs, 100u);
    EXPECT_TRUE(f.isEtherCAT);

    ASSERT_TRUE(reader.readNext(f));
    EXPECT_EQ(f.timestampNs, 200u);
    EXPECT_TRUE(f.isEtherCAT);

    EXPECT_FALSE(reader.readNext(f)); // EOF
}

TEST(PCAPNGReader, ResetAllowsReiteration) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);
    auto epb = makeEnhancedPacketBlock(0, 42, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));

    InterpretedFrame f;
    ASSERT_TRUE(reader.readNext(f));
    EXPECT_EQ(f.timestampNs, 42u);
    EXPECT_FALSE(reader.readNext(f)); // EOF

    reader.reset();
    ASSERT_TRUE(reader.readNext(f));
    EXPECT_EQ(f.timestampNs, 42u);
}

// ============================================================================
// Error-recovery mode
// ============================================================================

TEST(PCAPNGReader, RecoveryModeSkipsBadBlock) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    // Insert 12 bytes of garbage (invalid block header with huge totalLength).
    appendU32(data, 0xDEADBEEF);       // bogus block type
    appendU32(data, 0xFFFFFFFF);       // bogus total length (too large)
    appendU32(data, 0);                // padding

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);
    auto epb = makeEnhancedPacketBlock(0, 999, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    // Without recovery, parsing fails.
    {
        PCAPNGReader reader;
        ASSERT_TRUE(reader.open(data));
        auto frames = reader.readAll();
        EXPECT_TRUE(frames.empty());
    }

    // With recovery, the bad block is skipped and the EPB is found.
    {
        PCAPNGReader reader;
        ASSERT_TRUE(reader.open(data));
        size_t errorCount = 0;
        reader.setRecoveryMode(true, [&errorCount](size_t, const std::string&) {
            ++errorCount;
        });
        auto frames = reader.readAll();
        ASSERT_EQ(frames.size(), 1u);
        EXPECT_EQ(frames[0].timestampNs, 999u);
        EXPECT_TRUE(frames[0].isEtherCAT);
        EXPECT_EQ(reader.skippedBlockCount(), 1u);
        EXPECT_EQ(errorCount, 1u);
    }
}

TEST(PCAPNGReader, StrictModeAbortsOnBadBlock) {
    std::vector<uint8_t> data;
    auto shb = makeSectionHeaderBlock();
    auto idb = makeInterfaceDescriptionBlock();
    appendBytes(data, shb.data(), shb.size());
    appendBytes(data, idb.data(), idb.size());

    // Insert garbage.
    appendU32(data, 0xDEADBEEF);
    appendU32(data, 0xFFFFFFFF);
    appendU32(data, 0);

    std::array<uint8_t, 6> dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 6> src = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    auto frame = makeEtherCATFrame(dst, src, std::nullopt);
    auto epb = makeEnhancedPacketBlock(0, 999, frame.data(), frame.size());
    appendBytes(data, epb.data(), epb.size());

    PCAPNGReader reader;
    ASSERT_TRUE(reader.open(data));
    // Default mode is strict — readAll returns false (no frames collected).
    EXPECT_FALSE(reader.readAll([](const InterpretedFrame&) {}));
    EXPECT_EQ(reader.skippedBlockCount(), 0u);
}
