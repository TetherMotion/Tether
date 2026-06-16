/**
 * @file HALTests.cpp
 * @brief Unit tests for EtherCAT slave HAL implementations
 */

#include "slave/hal/ISlaveHAL.hpp"
#include "pcap/PcapLogger.hpp"

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>

namespace EtherCAT {
namespace Slave {

// Forward declarations for factory functions
std::unique_ptr<ISlaveHAL> createDirectLoopbackHAL();
std::unique_ptr<ISlaveHAL> createThreadedLoopbackHAL();

#ifdef __linux__
std::unique_ptr<ISlaveHAL> createFIFOLoopbackHAL();
std::unique_ptr<ISlaveHAL> createUDPLoopbackHAL();
std::unique_ptr<ISlaveHAL> createMasterFIFOHAL();
std::unique_ptr<ISlaveHAL> createSlaveFIFOHAL();
#endif

namespace Test {

// ============================================================================
// Direct Loopback HAL Tests
// ============================================================================

class DirectLoopbackHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createDirectLoopbackHAL();
    }
    
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(DirectLoopbackHALTest, OpenClose) {
    ASSERT_NE(hal_, nullptr);
    
    EXPECT_FALSE(hal_->isOpen());
    EXPECT_TRUE(hal_->open("loopback"));
    EXPECT_TRUE(hal_->isOpen());
    
    hal_->close();
    EXPECT_FALSE(hal_->isOpen());
}

TEST_F(DirectLoopbackHALTest, OpenMultipleTimes) {
    EXPECT_TRUE(hal_->open("loopback"));
    EXPECT_TRUE(hal_->isOpen());
    
    // Should be able to open again (reopen)
    EXPECT_TRUE(hal_->open("loopback"));
    EXPECT_TRUE(hal_->isOpen());
    
    hal_->close();
}

TEST_F(DirectLoopbackHALTest, SetPromiscuous) {
    EXPECT_TRUE(hal_->open("loopback"));
    EXPECT_TRUE(hal_->setPromiscuous(true));
    EXPECT_TRUE(hal_->setPromiscuous(false));
    
    hal_->close();
}

TEST_F(DirectLoopbackHALTest, StatisticsInitiallyZero) {
    EXPECT_TRUE(hal_->open("loopback"));
    
    EXPECT_EQ(hal_->getTxCount(), 0);
    EXPECT_EQ(hal_->getRxCount(), 0);
    EXPECT_EQ(hal_->getErrorCount(), 0);
    
    hal_->close();
}

// ============================================================================
// Threaded Loopback HAL Tests
// ============================================================================

class ThreadedLoopbackHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createThreadedLoopbackHAL();
    }
    
    void TearDown() override {
        if (hal_ && hal_->isOpen()) {
            hal_->close();
        }
    }
    
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(ThreadedLoopbackHALTest, OpenClose) {
    ASSERT_NE(hal_, nullptr);
    
    EXPECT_FALSE(hal_->isOpen());
    EXPECT_TRUE(hal_->open("loopback"));
    EXPECT_TRUE(hal_->isOpen());
    
    hal_->close();
    EXPECT_FALSE(hal_->isOpen());
}

TEST_F(ThreadedLoopbackHALTest, CloseWithoutOpen) {
    // Should not crash
    hal_->close();
    EXPECT_FALSE(hal_->isOpen());
}

// ============================================================================
// FIFO HAL Tests (Linux only)
// ============================================================================

#ifdef __linux__

class FIFOLoopbackHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createFIFOLoopbackHAL();
        basePath_ = "/tmp/ethercat_hal_test";
    }
    
    void TearDown() override {
        if (hal_ && hal_->isOpen()) {
            hal_->close();
        }
        // Clean up FIFOs
        unlink((basePath_ + "_tx").c_str());
        unlink((basePath_ + "_rx").c_str());
    }
    
    std::unique_ptr<ISlaveHAL> hal_;
    std::string basePath_;
};

TEST_F(FIFOLoopbackHALTest, OpenCreatesFIFOs) {
    EXPECT_TRUE(hal_->open(basePath_));
    EXPECT_TRUE(hal_->isOpen());
    
    // Check FIFOs exist
    struct stat st;
    EXPECT_EQ(stat((basePath_ + "_tx").c_str(), &st), 0);
    EXPECT_TRUE(S_ISFIFO(st.st_mode));
    
    EXPECT_EQ(stat((basePath_ + "_rx").c_str(), &st), 0);
    EXPECT_TRUE(S_ISFIFO(st.st_mode));
}

TEST_F(FIFOLoopbackHALTest, CloseRemovesFIFOs) {
    EXPECT_TRUE(hal_->open(basePath_));
    hal_->close();
    
    // FIFOs should be removed by default
    struct stat st;
    EXPECT_NE(stat((basePath_ + "_tx").c_str(), &st), 0);
    EXPECT_NE(stat((basePath_ + "_rx").c_str(), &st), 0);
}

// ============================================================================
// UDP Loopback HAL Tests
// ============================================================================

class UDPLoopbackHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createUDPLoopbackHAL();
    }
    
    void TearDown() override {
        if (hal_ && hal_->isOpen()) {
            hal_->close();
        }
    }
    
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(UDPLoopbackHALTest, OpenWithSinglePort) {
    EXPECT_TRUE(hal_->open("12345"));
    EXPECT_TRUE(hal_->isOpen());
    
    hal_->close();
    EXPECT_FALSE(hal_->isOpen());
}

TEST_F(UDPLoopbackHALTest, OpenWithPortPair) {
    EXPECT_TRUE(hal_->open("12345:12346"));
    EXPECT_TRUE(hal_->isOpen());
    
    hal_->close();
}

TEST_F(UDPLoopbackHALTest, SendReceiveLoopback) {
    // Need two sockets for loopback test
    auto hal1 = createUDPLoopbackHAL();
    auto hal2 = createUDPLoopbackHAL();
    
    EXPECT_TRUE(hal1->open("13001:13002"));
    EXPECT_TRUE(hal2->open("13002:13001"));
    
    // Send from hal1
    uint8_t txData[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    EXPECT_TRUE(hal1->sendFrame(txData, sizeof(txData)));
    
    // Receive on hal2
    uint8_t rxData[64];
    size_t rxLen = sizeof(rxData);
    EXPECT_TRUE(hal2->receiveFrame(rxData, rxLen, 1000));
    
    EXPECT_EQ(rxLen, sizeof(txData));
    EXPECT_EQ(std::memcmp(txData, rxData, sizeof(txData)), 0);
    
    hal1->close();
    hal2->close();
}

// ============================================================================
// Master/Slave FIFO Communication Tests
// ============================================================================

class MasterSlaveFIFOTest : public ::testing::Test {
protected:
    void SetUp() override {
        basePath_ = "/tmp/ethercat_ms_fifo_test";
    }
    
    void TearDown() override {
        // Clean up FIFOs
        unlink((basePath_ + "_tx").c_str());
        unlink((basePath_ + "_rx").c_str());
    }
    
    std::string basePath_;
};

TEST_F(MasterSlaveFIFOTest, BidirectionalCommunication) {
    std::atomic<bool> slaveReady{false};
    std::atomic<bool> testPassed{false};
    
    // Slave thread
    std::thread slaveThread([this, &slaveReady, &testPassed] {
        auto slaveHAL = createSlaveFIFOHAL();
        
        // Set up frame callback
        auto* slaveHALPtr = dynamic_cast<void*>(slaveHAL.get());
        // Note: Would need to cast to correct type and set callback
        
        EXPECT_TRUE(slaveHAL->open(basePath_));
        slaveReady = true;
        
        // Wait for test to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        slaveHAL->close();
    });
    
    // Wait for slave to be ready
    while (!slaveReady) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Master side
    auto masterHAL = createMasterFIFOHAL();
    
    // Note: This test requires proper callback setup which would need
    // specific casting to the implementation types
    
    slaveThread.join();
}

#endif  // __linux__

// ============================================================================
// PcapNG Logger Integration Tests
// ============================================================================

class HALPcapLoggingTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = createPcapLogger();
        hal_ = createDirectLoopbackHAL();
    }
    
    void TearDown() override {
        if (hal_ && hal_->isOpen()) {
            hal_->close();
        }
        logger_->close();
        
        // Clean up log file
        unlink("/tmp/hal_test.pcapng");
    }
    
    std::unique_ptr<IPcapLogger> logger_;
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(HALPcapLoggingTest, LogFrames) {
    EXPECT_TRUE(logger_->open("/tmp/hal_test.pcapng"));
    
    // Add interface
    InterfaceInfo info;
    info.name = "loopback0";
    info.description = "Test loopback interface";
    info.linkType = 1;  // Ethernet
    uint8_t ifaceId = logger_->addInterface(info);
    
    EXPECT_TRUE(hal_->open("loopback"));
    
    // Log some test packets
    uint8_t testPacket[64];
    std::memset(testPacket, 0xAA, sizeof(testPacket));
    
    for (int i = 0; i < 100; i++) {
        PacketMetadata meta;
        meta.direction = PacketDirection::Outbound;
        meta.interfaceId = ifaceId;
        meta.timestampNs = getPcapTimestampNs();
        
        EXPECT_TRUE(logger_->logPacket(testPacket, sizeof(testPacket), meta));
    }
    
    EXPECT_EQ(logger_->getPacketCount(), 100);
    EXPECT_EQ(logger_->getDropCount(), 0);
    
    logger_->flush();
}

// ============================================================================
// Concurrent HAL Access Tests
// ============================================================================

class ConcurrentHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createThreadedLoopbackHAL();
    }
    
    void TearDown() override {
        if (hal_ && hal_->isOpen()) {
            hal_->close();
        }
    }
    
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(ConcurrentHALTest, ConcurrentSends) {
    EXPECT_TRUE(hal_->open("loopback"));
    
    constexpr int numThreads = 4;
    constexpr int sendsPerThread = 100;
    std::atomic<int> successCount{0};
    
    std::vector<std::thread> threads;
    
    for (int t = 0; t < numThreads; t++) {
        threads.emplace_back([this, &successCount, t] {
            uint8_t data[32];
            std::memset(data, static_cast<uint8_t>(t), sizeof(data));
            
            for (int i = 0; i < sendsPerThread; i++) {
                if (hal_->sendFrame(data, sizeof(data))) {
                    successCount++;
                }
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // At least some sends should succeed
    EXPECT_GT(successCount, 0);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

class HALErrorHandlingTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createDirectLoopbackHAL();
    }
    
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(HALErrorHandlingTest, SendWithoutOpen) {
    uint8_t data[32];
    EXPECT_FALSE(hal_->sendFrame(data, sizeof(data)));
}

TEST_F(HALErrorHandlingTest, ReceiveWithoutOpen) {
    uint8_t data[32];
    size_t len = sizeof(data);
    EXPECT_FALSE(hal_->receiveFrame(data, len, 100));
}

TEST_F(HALErrorHandlingTest, ZeroLengthSend) {
    EXPECT_TRUE(hal_->open("loopback"));
    
    uint8_t data[1];
    // Depending on implementation, may succeed or fail
    hal_->sendFrame(data, 0);
    
    hal_->close();
}

// ============================================================================
// Memory Logger Tests
// ============================================================================

TEST(MemoryPcapLoggerTest, LogToMemory) {
    auto logger = createMemoryPcapLogger(1024 * 1024);
    EXPECT_TRUE(logger->open("memory"));
    
    // Add interface
    InterfaceInfo info;
    info.name = "mem0";
    info.linkType = 1;
    logger->addInterface(info);
    
    // Log packets
    uint8_t packet[64];
    for (int i = 0; i < 1000; i++) {
        std::memset(packet, static_cast<uint8_t>(i), sizeof(packet));
        EXPECT_TRUE(logger->logPacket(packet, sizeof(packet), 
                                       PacketDirection::Outbound, 0));
    }
    
    EXPECT_EQ(logger->getPacketCount(), 1000);
}

TEST(NullPcapLoggerTest, DiscardPackets) {
    auto logger = createNullPcapLogger();
    EXPECT_TRUE(logger->open("null"));
    EXPECT_TRUE(logger->isOpen());
    
    uint8_t packet[64];
    for (int i = 0; i < 1000; i++) {
        EXPECT_TRUE(logger->logPacket(packet, sizeof(packet),
                                       PacketDirection::Outbound, 0));
    }
    
    // Null logger discards everything
    EXPECT_EQ(logger->getPacketCount(), 0);
    EXPECT_EQ(logger->getDropCount(), 0);
}

}  // namespace Test
}  // namespace Slave
}  // namespace EtherCAT

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
