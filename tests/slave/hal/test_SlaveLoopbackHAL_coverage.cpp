/**
 * @file test_SlaveLoopbackHAL_coverage.cpp
 * @brief Tests for slave-side LoopbackHAL (DirectLoopbackHAL, FIFOLoopbackHAL, factories)
 */

#include "slave/hal/ISlaveHAL.hpp"
#include "hal/HALTypes.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace EtherCAT;
using namespace EtherCAT::slave;

// ============================================================================
// DirectLoopbackHAL Tests
// ============================================================================

class DirectSlaveHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createDirectSlaveHAL();
        ASSERT_NE(hal_, nullptr);
    }
    std::unique_ptr<ISlaveHAL> hal_;
};

TEST_F(DirectSlaveHALTest, InitSuccess) {
    SlaveHALConfig config;
    auto err = hal_->init(config);
    EXPECT_EQ(err, HAL::Error::OK);
    EXPECT_TRUE(hal_->isInitialized());
}

TEST_F(DirectSlaveHALTest, InitAlreadyInitialized) {
    SlaveHALConfig config;
    EXPECT_EQ(hal_->init(config), HAL::Error::OK);
    // Second init should fail
    EXPECT_EQ(hal_->init(config), HAL::Error::AlreadyInitialized);
}

TEST_F(DirectSlaveHALTest, ShutdownAndReinit) {
    SlaveHALConfig config;
    EXPECT_EQ(hal_->init(config), HAL::Error::OK);
    hal_->shutdown();
    EXPECT_FALSE(hal_->isInitialized());
    // Can reinit after shutdown
    EXPECT_EQ(hal_->init(config), HAL::Error::OK);
    EXPECT_TRUE(hal_->isInitialized());
}

TEST_F(DirectSlaveHALTest, GetSetMacAddress) {
    SlaveHALConfig config;
    config.macAddress = HAL::MacAddress(0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE);
    hal_->init(config);
    
    HAL::MacAddress mac;
    EXPECT_EQ(hal_->getMacAddress(mac), HAL::Error::OK);
    EXPECT_EQ(mac.bytes[1], 0xAA);
    
    HAL::MacAddress newMac(0x02, 0x11, 0x22, 0x33, 0x44, 0x55);
    EXPECT_EQ(hal_->setMacAddress(newMac), HAL::Error::OK);
    EXPECT_EQ(hal_->getMacAddress(mac), HAL::Error::OK);
    EXPECT_EQ(mac.bytes[1], 0x11);
}

TEST_F(DirectSlaveHALTest, TransmitNotInitialized) {
    uint8_t frame[] = {1, 2, 3, 4};
    EXPECT_EQ(hal_->transmit(frame, sizeof(frame)), HAL::Error::NotInitialized);
}

TEST_F(DirectSlaveHALTest, TransmitInitialized) {
    SlaveHALConfig config;
    hal_->init(config);
    uint8_t frame[] = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0x01, 0x02, 0x03, 0x04};
    EXPECT_EQ(hal_->transmit(frame, sizeof(frame)), HAL::Error::OK);
}

TEST_F(DirectSlaveHALTest, TransmitStats) {
    SlaveHALConfig config;
    hal_->init(config);
    uint8_t frame[64] = {};
    hal_->transmit(frame, sizeof(frame));
    hal_->transmit(frame, sizeof(frame));
    
    auto stats = hal_->getStats();
    EXPECT_EQ(stats.txFrames, 2u);
    EXPECT_EQ(stats.txBytes, 128u);
}

TEST_F(DirectSlaveHALTest, ResetStats) {
    SlaveHALConfig config;
    hal_->init(config);
    uint8_t frame[64] = {};
    hal_->transmit(frame, sizeof(frame));
    
    hal_->resetStats();
    auto stats = hal_->getStats();
    EXPECT_EQ(stats.txFrames, 0u);
    EXPECT_EQ(stats.txBytes, 0u);
}

TEST_F(DirectSlaveHALTest, ReceiveEmpty) {
    SlaveHALConfig config;
    hal_->init(config);
    
    uint8_t buffer[256];
    size_t received = 0;
    EXPECT_EQ(hal_->receive(buffer, sizeof(buffer), received, 0),
              HAL::Error::WouldBlock);
    EXPECT_EQ(received, 0u);
}

TEST_F(DirectSlaveHALTest, PollEmpty) {
    SlaveHALConfig config;
    hal_->init(config);
    EXPECT_EQ(hal_->poll(0), 0);
}

TEST_F(DirectSlaveHALTest, LinkStatus) {
    SlaveHALConfig config;
    hal_->init(config);
    
    auto status = hal_->getLinkStatus();
    EXPECT_TRUE(status.up);
    EXPECT_EQ(status.speedMbps, 1000u);
    EXPECT_TRUE(status.fullDuplex);
}

TEST_F(DirectSlaveHALTest, LinkStatusNotInitialized) {
    auto status = hal_->getLinkStatus();
    EXPECT_FALSE(status.up);
}

TEST_F(DirectSlaveHALTest, WaitForLinkUp_Initialized) {
    SlaveHALConfig config;
    hal_->init(config);
    EXPECT_EQ(hal_->waitForLinkUp(100), HAL::Error::OK);
}

TEST_F(DirectSlaveHALTest, WaitForLinkUp_NotInitialized) {
    EXPECT_EQ(hal_->waitForLinkUp(100), HAL::Error::LinkDown);
}

TEST_F(DirectSlaveHALTest, PcapLogging) {
    SlaveHALConfig config;
    hal_->init(config);
    
    HAL::PcapLoggerConfig pcapConfig;
    EXPECT_EQ(hal_->enablePcapLogging(pcapConfig), HAL::Error::OK);
    EXPECT_FALSE(hal_->isPcapLoggingEnabled());
    EXPECT_EQ(hal_->getPcapLogger(), nullptr);
    hal_->disablePcapLogging();
}

TEST_F(DirectSlaveHALTest, SetRxCallback) {
    SlaveHALConfig config;
    hal_->init(config);
    
    int callCount = 0;
    hal_->setRxCallback([](const uint8_t*, size_t, void* ud) {
        (*static_cast<int*>(ud))++;
    }, &callCount);
    
    // Callback gets called when there's a target sending to us... 
    // but in direct mode we need a loopback target
}

TEST_F(DirectSlaveHALTest, IsInitialized_Default) {
    EXPECT_FALSE(hal_->isInitialized());
}

// ============================================================================
// Loopback connection test: two DirectLoopbackHALs connected
// ============================================================================

TEST(DirectLoopbackPairTest, ConnectAndSendReceive) {
    auto hal1 = createDirectSlaveHAL();
    auto hal2 = createDirectSlaveHAL();
    ASSERT_NE(hal1, nullptr);
    ASSERT_NE(hal2, nullptr);
    
    SlaveHALConfig config1, config2;
    EXPECT_EQ(hal1->init(config1), HAL::Error::OK);
    EXPECT_EQ(hal2->init(config2), HAL::Error::OK);
    
    // Cast to ILoopbackTarget and connect
    auto* target2 = dynamic_cast<ILoopbackTarget*>(hal2.get());
    if (target2) {
        // Send from HAL1, received by HAL2 via loopback target
        uint8_t frame[] = {0xAA, 0xBB, 0xCC, 0xDD};
        target2->onFrameReceived(frame, sizeof(frame));
        
        // HAL2 should have it in queue
        uint8_t buf[256];
        size_t len = 0;
        auto err = hal2->receive(buf, sizeof(buf), len, 0);
        EXPECT_EQ(err, HAL::Error::OK);
        EXPECT_EQ(len, sizeof(frame));
        EXPECT_EQ(buf[0], 0xAA);
    }
}

TEST(DirectLoopbackPairTest, LoopbackTargetWithCallback) {
    auto hal = createDirectSlaveHAL();
    SlaveHALConfig config;
    hal->init(config);
    
    std::vector<uint8_t> received;
    hal->setRxCallback([](const uint8_t* frame, size_t len, void* ud) {
        auto* vec = static_cast<std::vector<uint8_t>*>(ud);
        vec->assign(frame, frame + len);
    }, &received);
    
    auto* target = dynamic_cast<ILoopbackTarget*>(hal.get());
    if (target) {
        uint8_t frame[] = {0x01, 0x02, 0x03};
        target->onFrameReceived(frame, sizeof(frame));
        EXPECT_EQ(received.size(), 3u);
        EXPECT_EQ(received[0], 0x01);
    }
}

TEST(DirectLoopbackPairTest, LoopbackTargetStats) {
    auto hal = createDirectSlaveHAL();
    SlaveHALConfig config;
    hal->init(config);
    
    auto* target = dynamic_cast<ILoopbackTarget*>(hal.get());
    if (target) {
        uint8_t frame[100] = {};
        target->onFrameReceived(frame, sizeof(frame));
        target->onFrameReceived(frame, sizeof(frame));
        
        auto stats = hal->getStats();
        EXPECT_EQ(stats.rxFrames, 2u);
        EXPECT_EQ(stats.rxBytes, 200u);
    }
}

// ============================================================================
// FIFO Loopback HAL Tests
// ============================================================================

TEST(FIFOSlaveHALFactoryTest, FactoryCreation) {
    auto hal = createFIFOSlaveHAL();
#ifdef __linux__
    EXPECT_NE(hal, nullptr);
#endif
}

#ifdef __linux__
class FIFOSlaveHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        hal_ = createFIFOSlaveHAL();
        ASSERT_NE(hal_, nullptr);
    }
    void TearDown() override {
        if (hal_ && hal_->isInitialized()) {
            hal_->shutdown();
        }
        // Clean up temporary FIFOs
        unlink(rxPath_.c_str());
        unlink(txPath_.c_str());
    }
    std::unique_ptr<ISlaveHAL> hal_;
    std::string rxPath_ = "/tmp/test_fifo_rx_" + std::to_string(getpid());
    std::string txPath_ = "/tmp/test_fifo_tx_" + std::to_string(getpid());
};

TEST_F(FIFOSlaveHALTest, IsInitializedDefault) {
    EXPECT_FALSE(hal_->isInitialized());
}

TEST_F(FIFOSlaveHALTest, GetSetMacAddress) {
    // Even without init, mac operations may work
    HAL::MacAddress mac(0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE);
    hal_->setMacAddress(mac);
    HAL::MacAddress read;
    hal_->getMacAddress(read);
    EXPECT_EQ(read.bytes[1], 0xAA);
}

TEST_F(FIFOSlaveHALTest, LinkStatus) {
    auto status = hal_->getLinkStatus();
    EXPECT_FALSE(status.up); // Not initialized
}

TEST_F(FIFOSlaveHALTest, WaitForLinkDown) {
    EXPECT_EQ(hal_->waitForLinkUp(10), HAL::Error::LinkDown);
}

TEST_F(FIFOSlaveHALTest, PcapLogging) {
    HAL::PcapLoggerConfig pcapConfig;
    EXPECT_EQ(hal_->enablePcapLogging(pcapConfig), HAL::Error::OK);
    EXPECT_FALSE(hal_->isPcapLoggingEnabled());
    EXPECT_EQ(hal_->getPcapLogger(), nullptr);
    hal_->disablePcapLogging();
}

TEST_F(FIFOSlaveHALTest, Stats) {
    auto stats = hal_->getStats();
    EXPECT_EQ(stats.txFrames, 0u);
    hal_->resetStats();
}
#endif

// ============================================================================
// Network Slave HAL (returns nullptr on most platforms)
// ============================================================================

TEST(NetworkSlaveHALTest, FactoryReturnsNull) {
    auto hal = createNetworkSlaveHAL();
    EXPECT_EQ(hal, nullptr);
}

// ============================================================================
// Factory functions
// ============================================================================

TEST(SlaveHALFactoryTest, CreateDirectNotNull) {
    auto hal = createDirectSlaveHAL();
    EXPECT_NE(hal, nullptr);
}

TEST(SlaveHALFactoryTest, CreateFIFONotNull) {
    auto hal = createFIFOSlaveHAL();
#ifdef __linux__
    EXPECT_NE(hal, nullptr);
#else
    EXPECT_EQ(hal, nullptr);
#endif
}

TEST(SlaveHALFactoryTest, CreateNetworkNull) {
    auto hal = createNetworkSlaveHAL();
    EXPECT_EQ(hal, nullptr);
}

// ============================================================================
// MacAddress utility tests
// ============================================================================

TEST(MacAddressCovTest, DefaultConstructor) {
    HAL::MacAddress mac;
    EXPECT_TRUE(mac.isZero());
}

TEST(MacAddressCovTest, ByteConstructor) {
    HAL::MacAddress mac(0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE);
    EXPECT_FALSE(mac.isZero());
    EXPECT_EQ(mac.bytes[0], 0x02);
    EXPECT_EQ(mac.bytes[5], 0xEE);
}

TEST(MacAddressCovTest, PtrConstructor) {
    uint8_t raw[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    HAL::MacAddress mac(raw);
    EXPECT_EQ(mac.bytes[0], 0x01);
    EXPECT_EQ(mac.bytes[5], 0x06);
}

TEST(MacAddressCovTest, Equality) {
    HAL::MacAddress a(0x01, 0x02, 0x03, 0x04, 0x05, 0x06);
    HAL::MacAddress b(0x01, 0x02, 0x03, 0x04, 0x05, 0x06);
    HAL::MacAddress c(0x01, 0x02, 0x03, 0x04, 0x05, 0x07);
    EXPECT_TRUE(a == b);
    EXPECT_TRUE(a != c);
}

TEST(MacAddressCovTest, IsBroadcast) {
    HAL::MacAddress bcast(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
    EXPECT_TRUE(bcast.isBroadcast());
    
    HAL::MacAddress notBcast(0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
    EXPECT_FALSE(notBcast.isBroadcast());
}

TEST(MacAddressCovTest, IsMulticast) {
    HAL::MacAddress mcast(0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
    EXPECT_TRUE(mcast.isMulticast());
    
    HAL::MacAddress unicast(0x02, 0x00, 0x00, 0x00, 0x00, 0x00);
    EXPECT_FALSE(unicast.isMulticast());
}

TEST(MacAddressCovTest, IsLocallyAdministered) {
    HAL::MacAddress local(0x02, 0x00, 0x00, 0x00, 0x00, 0x00);
    EXPECT_TRUE(local.isLocallyAdministered());
    
    HAL::MacAddress global(0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    EXPECT_FALSE(global.isLocallyAdministered());
}

TEST(MacAddressCovTest, EtherCATBroadcast) {
    auto bcast = HAL::MacAddress::etherCATBroadcast();
    EXPECT_TRUE(bcast.isBroadcast());
}

TEST(MacAddressCovTest, EtherCATMulticast) {
    auto mcast = HAL::MacAddress::etherCATMulticast();
    EXPECT_TRUE(mcast.isMulticast());
}

// ============================================================================
// HAL Error utilities
// ============================================================================

TEST(HALErrorCovTest, ErrorToString) {
    EXPECT_STREQ(HAL::errorToString(HAL::Error::OK), "OK");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::Timeout), "Timeout");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::NotInitialized), "Not initialized");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::AlreadyInitialized), "Already initialized");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::WouldBlock), "Would block");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::BufferTooSmall), "Buffer too small");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::LinkDown), "Link down");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::TransmitFailed), "Transmit failed");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::ReceiveFailed), "Receive failed");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::ConfigurationFailed), "Configuration failed");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::InternalError), "Internal error");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::NotSupported), "Not supported");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::InvalidArgument), "Invalid argument");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::NoMemory), "Out of memory");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::BufferFull), "Buffer full");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::Empty), "Empty");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::PermissionDenied), "Permission denied");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::InterfaceNotFound), "Interface not found");
    EXPECT_STREQ(HAL::errorToString(HAL::Error::Cancelled), "Operation cancelled");
}

// ============================================================================
// HAL Result<T> tests
// ============================================================================

TEST(HALResultCovTest, DefaultOk) {
    HAL::Result<int> r;
    EXPECT_TRUE(r.ok());
    EXPECT_TRUE(static_cast<bool>(r));
}

TEST(HALResultCovTest, ValueConstructor) {
    HAL::Result<int> r(42);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.value, 42);
}

TEST(HALResultCovTest, ErrorConstructor) {
    HAL::Result<int> r(HAL::Error::Timeout);
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error, HAL::Error::Timeout);
}

TEST(HALResultCovTest, ValueAndErrorConstructor) {
    HAL::Result<int> r(10, HAL::Error::OK);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.value, 10);
}

TEST(HALResultCovTest, VoidResult) {
    HAL::VoidResult ok;
    EXPECT_TRUE(ok.ok());
    
    HAL::VoidResult err(HAL::Error::NotInitialized);
    EXPECT_FALSE(err.ok());
}

// ============================================================================
// Byte-order utilities
// ============================================================================

TEST(ByteOrderCovTest, Bswap16) {
    EXPECT_EQ(HAL::bswap16(0x1234), 0x3412);
}

TEST(ByteOrderCovTest, Bswap32) {
    EXPECT_EQ(HAL::bswap32(0x12345678u), 0x78563412u);
}

TEST(ByteOrderCovTest, RoundTrip16) {
    uint16_t val = 0xABCD;
    // bswap16 applied twice should return original
    EXPECT_EQ(HAL::bswap16(HAL::bswap16(val)), val);
}

TEST(ByteOrderCovTest, RoundTrip32) {
    uint32_t val = 0xDEADBEEF;
    // bswap32 applied twice should return original
    EXPECT_EQ(HAL::bswap32(HAL::bswap32(val)), val);
}

// ============================================================================
// EthernetStats coverage (from IEthernet base - accessible via HALTypes)
// ============================================================================

TEST(EthernetStatsCovTest, DefaultValues) {
    HAL::EthernetStats stats{};
    EXPECT_EQ(stats.txFrames, 0u);
    EXPECT_EQ(stats.rxFrames, 0u);
    EXPECT_EQ(stats.txBytes, 0u);
    EXPECT_EQ(stats.rxBytes, 0u);
    EXPECT_EQ(stats.txErrors, 0u);
    EXPECT_EQ(stats.rxErrors, 0u);
}
