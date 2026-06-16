/**
 * @file test_PcapLogger_coverage.cpp
 * @brief Comprehensive PcapNgLogger coverage tests
 */
#include <gtest/gtest.h>
#include <cstring>
#include <cstdio>
#include <memory>
#include <vector>
#include <string>
#include "packetloggers/pcap/PCAPWriter.hpp"

using namespace Tether::PacketLoggers::PCAP;

// ============================================================================
// Fixture: PCAPWriter Tests
// ============================================================================
class PCAPWriterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Generate unique temp file for each test
        tmpFile = "/tmp/test_pcap_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".pcapng";
    }
    void TearDown() override {
        std::remove(tmpFile.c_str());
    }
    std::string tmpFile;
};

// --- Construction and lifecycle ---

TEST_F(PCAPWriterTest, DefaultConstruction) {
    PCAPWriter logger;
    EXPECT_FALSE(logger.isOpen());
    EXPECT_TRUE(logger.isEnabled());
}

TEST_F(PCAPWriterTest, OpenClose) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    EXPECT_TRUE(logger.isOpen());
    logger.close();
    EXPECT_FALSE(logger.isOpen());
}

TEST_F(PCAPWriterTest, OpenNonExistentPath) {
    PCAPWriter logger;
    bool result = logger.open("/nonexistent/dir/file.pcapng");
    // May fail or succeed depending on filesystem
    if (!result) {
        EXPECT_FALSE(logger.isOpen());
    }
}

TEST_F(PCAPWriterTest, DoubleOpen) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    // Opening again should close and reopen or just succeed
    logger.open(tmpFile);
    EXPECT_TRUE(logger.isOpen());
    logger.close();
}

TEST_F(PCAPWriterTest, CloseWithoutOpen) {
    PCAPWriter logger;
    logger.close(); // should not crash
    EXPECT_FALSE(logger.isOpen());
}

// --- Section header ---

TEST_F(PCAPWriterTest, SetSectionHeader) {
    PCAPWriter logger;
    SectionHeaderInfo info;
    info.hardware = "TestHW";
    info.os = "Linux";
    info.application = "UnitTest";
    logger.setSectionHeader(info);
    EXPECT_TRUE(logger.open(tmpFile));
    logger.close();
}

// --- Interface ---

TEST_F(PCAPWriterTest, AddInterface) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    iface.name = "eth0";
    iface.description = "Test interface";
    iface.linkType = 1; // Ethernet
    uint8_t id = logger.addInterface(iface);
    EXPECT_EQ(id, 0u);
    logger.close();
}

TEST_F(PCAPWriterTest, AddMultipleInterfaces) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface1, iface2;
    iface1.name = "if0";
    iface2.name = "if1";
    uint8_t id1 = logger.addInterface(iface1);
    uint8_t id2 = logger.addInterface(iface2);
    EXPECT_EQ(id1, 0u);
    EXPECT_EQ(id2, 1u);
    logger.close();
}

// --- Packet logging ---

TEST_F(PCAPWriterTest, LogPacketBasic) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    iface.name = "eth0";
    logger.addInterface(iface);

    std::vector<uint8_t> packet(64, 0xAA);
    PacketMetadata meta;
    meta.interfaceId = 0;
    meta.direction = PacketDirection::Outbound;
    EXPECT_TRUE(logger.logPacket(packet.data(), packet.size(), meta));
    EXPECT_EQ(logger.getPacketCount(), 1u);
    logger.close();
}

TEST_F(PCAPWriterTest, LogPacketWithDirection) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    logger.addInterface(iface);

    std::vector<uint8_t> packet(100, 0xBB);
    EXPECT_TRUE(logger.logPacket(packet.data(), packet.size(), PacketDirection::Inbound, 0));
    EXPECT_EQ(logger.getPacketCount(), 1u);
    logger.close();
}

TEST_F(PCAPWriterTest, LogManyPackets) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    logger.addInterface(iface);

    std::vector<uint8_t> packet(64, 0xCC);
    for (int i = 0; i < 100; ++i) {
        logger.logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
    }
    EXPECT_EQ(logger.getPacketCount(), 100u);
    EXPECT_GT(logger.getByteCount(), 0u);
    logger.close();
}

TEST_F(PCAPWriterTest, LogPacketWhenDisabled) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    logger.addInterface(iface);

    logger.setEnabled(false);
    EXPECT_FALSE(logger.isEnabled());
    std::vector<uint8_t> packet(64, 0xDD);
    logger.logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
    // Should not count when disabled
    logger.close();
}

TEST_F(PCAPWriterTest, LogPacketWithoutOpen) {
    PCAPWriter logger;
    std::vector<uint8_t> packet(64, 0xEE);
    bool result = logger.logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
    EXPECT_FALSE(result);
}

// --- Flush ---

TEST_F(PCAPWriterTest, FlushOpenFile) {
    PCAPWriter logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    logger.addInterface(iface);
    std::vector<uint8_t> packet(64, 0xFF);
    logger.logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
    EXPECT_TRUE(logger.flush());
    logger.close();
}

TEST_F(PCAPWriterTest, FlushWithoutOpen) {
    PCAPWriter logger;
    logger.flush(); // should not crash
}

// --- Counters ---

TEST_F(PCAPWriterTest, InitialCountersZero) {
    PCAPWriter logger;
    EXPECT_EQ(logger.getPacketCount(), 0u);
    EXPECT_EQ(logger.getByteCount(), 0u);
    EXPECT_EQ(logger.getDropCount(), 0u);
}

// --- Buffer ---

TEST_F(PCAPWriterTest, SetBufferSize) {
    PCAPWriter logger;
    logger.setBufferSize(4096);
}

TEST_F(PCAPWriterTest, GetBufferUsage) {
    PCAPWriter logger;
    EXPECT_EQ(logger.getBufferUsage(), 0u);
}

// --- Compression ---

TEST_F(PCAPWriterTest, CompressionSetting) {
    PCAPWriter logger;
    logger.setCompressionEnabled(true);
    logger.setCompressionEnabled(false);
}

// --- File rotation ---

TEST_F(PCAPWriterTest, MaxFileSize) {
    PCAPWriter logger;
    logger.setMaxFileSize(1024 * 1024); // 1MB
}

TEST_F(PCAPWriterTest, RotationEnabled) {
    PCAPWriter logger;
    logger.setRotationEnabled(true, 5);
    logger.setRotationEnabled(false, 0);
}

// --- Factory functions ---

TEST_F(PCAPWriterTest, CreatePCAPWriterDefault) {
    auto writer = createPCAPWriter();
    ASSERT_NE(writer, nullptr);
    EXPECT_FALSE(writer->isOpen());
}

TEST_F(PCAPWriterTest, CreatePCAPWriterWithFile) {
    SectionHeaderInfo info;
    info.application = "Test";
    auto writer = createPCAPWriter(tmpFile, info);
    ASSERT_NE(writer, nullptr);
    EXPECT_TRUE(writer->isOpen());
    writer->close();
}

TEST_F(PCAPWriterTest, CreateNullPCAPWriter) {
    auto writer = createNullPCAPWriter();
    ASSERT_NE(writer, nullptr);
    // Null writer should accept but discard packets
    std::vector<uint8_t> packet(64, 0);
    writer->logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
}

TEST_F(PCAPWriterTest, CreateMemoryPCAPWriter) {
    auto writer = createMemoryPCAPWriter(4096);
    ASSERT_NE(writer, nullptr);
}

// --- Utility functions ---

TEST_F(PCAPWriterTest, GetTimestamp) {
    uint64_t ts = getPCAPTimestampNs();
    EXPECT_GT(ts, 0u);
}

TEST_F(PCAPWriterTest, ExtractEtherCATMetadataEmpty) {
    uint8_t data[14] = {};
    auto meta = extractEtherCATMetadata(data, 14);
    (void)meta; // Just check it doesn't crash
}

TEST_F(PCAPWriterTest, ExtractEtherCATMetadataValid) {
    // Build a minimal Ethernet frame with EtherCAT EtherType
    uint8_t frame[64] = {};
    frame[12] = 0x88;
    frame[13] = 0xA4;
    // EtherCAT header
    frame[14] = 0x10; // length = 16
    frame[15] = 0x10; // type 1
    auto meta = extractEtherCATMetadata(frame, 64);
    (void)meta;
}
