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
#include "pcap/PcapLogger.hpp"

using namespace EtherCAT;

// ============================================================================
// Fixture: PcapNgLogger Tests
// ============================================================================
class PcapLoggerTest : public ::testing::Test {
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

TEST_F(PcapLoggerTest, DefaultConstruction) {
    PcapNgLogger logger;
    EXPECT_FALSE(logger.isOpen());
    EXPECT_TRUE(logger.isEnabled());
}

TEST_F(PcapLoggerTest, OpenClose) {
    PcapNgLogger logger;
    EXPECT_TRUE(logger.open(tmpFile));
    EXPECT_TRUE(logger.isOpen());
    logger.close();
    EXPECT_FALSE(logger.isOpen());
}

TEST_F(PcapLoggerTest, OpenNonExistentPath) {
    PcapNgLogger logger;
    bool result = logger.open("/nonexistent/dir/file.pcapng");
    // May fail or succeed depending on filesystem
    if (!result) {
        EXPECT_FALSE(logger.isOpen());
    }
}

TEST_F(PcapLoggerTest, DoubleOpen) {
    PcapNgLogger logger;
    EXPECT_TRUE(logger.open(tmpFile));
    // Opening again should close and reopen or just succeed
    logger.open(tmpFile);
    EXPECT_TRUE(logger.isOpen());
    logger.close();
}

TEST_F(PcapLoggerTest, CloseWithoutOpen) {
    PcapNgLogger logger;
    logger.close(); // should not crash
    EXPECT_FALSE(logger.isOpen());
}

// --- Section header ---

TEST_F(PcapLoggerTest, SetSectionHeader) {
    PcapNgLogger logger;
    SectionHeaderInfo info;
    info.hardware = "TestHW";
    info.os = "Linux";
    info.application = "UnitTest";
    logger.setSectionHeader(info);
    EXPECT_TRUE(logger.open(tmpFile));
    logger.close();
}

// --- Interface ---

TEST_F(PcapLoggerTest, AddInterface) {
    PcapNgLogger logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    iface.name = "eth0";
    iface.description = "Test interface";
    iface.linkType = 1; // Ethernet
    uint8_t id = logger.addInterface(iface);
    EXPECT_EQ(id, 0u);
    logger.close();
}

TEST_F(PcapLoggerTest, AddMultipleInterfaces) {
    PcapNgLogger logger;
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

TEST_F(PcapLoggerTest, LogPacketBasic) {
    PcapNgLogger logger;
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

TEST_F(PcapLoggerTest, LogPacketWithDirection) {
    PcapNgLogger logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    logger.addInterface(iface);

    std::vector<uint8_t> packet(100, 0xBB);
    EXPECT_TRUE(logger.logPacket(packet.data(), packet.size(), PacketDirection::Inbound, 0));
    EXPECT_EQ(logger.getPacketCount(), 1u);
    logger.close();
}

TEST_F(PcapLoggerTest, LogManyPackets) {
    PcapNgLogger logger;
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

TEST_F(PcapLoggerTest, LogPacketWhenDisabled) {
    PcapNgLogger logger;
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

TEST_F(PcapLoggerTest, LogPacketWithoutOpen) {
    PcapNgLogger logger;
    std::vector<uint8_t> packet(64, 0xEE);
    bool result = logger.logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
    EXPECT_FALSE(result);
}

// --- Flush ---

TEST_F(PcapLoggerTest, FlushOpenFile) {
    PcapNgLogger logger;
    EXPECT_TRUE(logger.open(tmpFile));
    InterfaceInfo iface;
    logger.addInterface(iface);
    std::vector<uint8_t> packet(64, 0xFF);
    logger.logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
    EXPECT_TRUE(logger.flush());
    logger.close();
}

TEST_F(PcapLoggerTest, FlushWithoutOpen) {
    PcapNgLogger logger;
    logger.flush(); // should not crash
}

// --- Counters ---

TEST_F(PcapLoggerTest, InitialCountersZero) {
    PcapNgLogger logger;
    EXPECT_EQ(logger.getPacketCount(), 0u);
    EXPECT_EQ(logger.getByteCount(), 0u);
    EXPECT_EQ(logger.getDropCount(), 0u);
}

// --- Buffer ---

TEST_F(PcapLoggerTest, SetBufferSize) {
    PcapNgLogger logger;
    logger.setBufferSize(4096);
}

TEST_F(PcapLoggerTest, GetBufferUsage) {
    PcapNgLogger logger;
    EXPECT_EQ(logger.getBufferUsage(), 0u);
}

// --- Compression ---

TEST_F(PcapLoggerTest, CompressionSetting) {
    PcapNgLogger logger;
    logger.setCompressionEnabled(true);
    logger.setCompressionEnabled(false);
}

// --- File rotation ---

TEST_F(PcapLoggerTest, MaxFileSize) {
    PcapNgLogger logger;
    logger.setMaxFileSize(1024 * 1024); // 1MB
}

TEST_F(PcapLoggerTest, RotationEnabled) {
    PcapNgLogger logger;
    logger.setRotationEnabled(true, 5);
    logger.setRotationEnabled(false, 0);
}

// --- Factory functions ---

TEST_F(PcapLoggerTest, CreatePcapLoggerDefault) {
    auto logger = createPcapLogger();
    ASSERT_NE(logger, nullptr);
    EXPECT_FALSE(logger->isOpen());
}

TEST_F(PcapLoggerTest, CreatePcapLoggerWithFile) {
    SectionHeaderInfo info;
    info.application = "Test";
    auto logger = createPcapLogger(tmpFile, info);
    ASSERT_NE(logger, nullptr);
    EXPECT_TRUE(logger->isOpen());
    logger->close();
}

TEST_F(PcapLoggerTest, CreateNullPcapLogger) {
    auto logger = createNullPcapLogger();
    ASSERT_NE(logger, nullptr);
    // Null logger should accept but discard packets
    std::vector<uint8_t> packet(64, 0);
    logger->logPacket(packet.data(), packet.size(), PacketDirection::Outbound, 0);
}

TEST_F(PcapLoggerTest, CreateMemoryPcapLogger) {
    auto logger = createMemoryPcapLogger(4096);
    ASSERT_NE(logger, nullptr);
}

// --- Utility functions ---

TEST_F(PcapLoggerTest, GetTimestamp) {
    uint64_t ts = getPcapTimestampNs();
    EXPECT_GT(ts, 0u);
}

TEST_F(PcapLoggerTest, ExtractEtherCATMetadataEmpty) {
    uint8_t data[14] = {};
    auto meta = extractEtherCATMetadata(data, 14);
    (void)meta; // Just check it doesn't crash
}

TEST_F(PcapLoggerTest, ExtractEtherCATMetadataValid) {
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
