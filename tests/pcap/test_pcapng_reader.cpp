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

// Build a raw Ethernet + EtherCAT frame with one APRD datagram.
std::vector<uint8_t> makeEtherCATFrame(const std::array<uint8_t, 6>& dst,
                                       const std::array<uint8_t, 6>& src,
                                       std::optional<uint16_t> vlanId = std::nullopt) {
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
    appendU16(frame, 0);       // irq
    appendBytes(frame, payload.data(), payload.size());
    appendU16(frame, wkc);

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
