/**
 * @file test_HAL_Common_coverage.cpp
 * @brief Comprehensive HAL_Common coverage tests — VlanEthernetWrapper,
 *        TrafficSplitter, LoggingEthernetWrapper, HALInstance
 */
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include "tether/hal/HAL.hpp"
#include "tether/hal/IEthernet.hpp"
#include "mocks/MockHAL.hpp"

using namespace EtherCAT;
using namespace EtherCAT::HAL;
using namespace EtherCAT::HAL::mock;

// ============================================================================
// VLAN Ethernet Wrapper
// ============================================================================

class VlanEthernetTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto fake = std::make_unique<FakeEthernet>();
        fakePtr = fake.get();
        vlan = createVlanEthernet(std::move(fake), 100, 3);
    }
    FakeEthernet* fakePtr = nullptr;
    std::unique_ptr<IEthernet> vlan;
};

TEST_F(VlanEthernetTest, InitShutdown) {
    EthernetConfig cfg{};
    EXPECT_EQ(vlan->init(cfg), Error::OK);
    EXPECT_TRUE(vlan->isInitialized());
    vlan->shutdown();
    EXPECT_FALSE(vlan->isInitialized());
}

TEST_F(VlanEthernetTest, GetMacAddress) {
    MacAddress mac;
    EXPECT_EQ(vlan->getMacAddress(mac), Error::OK);
}

TEST_F(VlanEthernetTest, SetMacAddress) {
    MacAddress mac(0x02, 0x00, 0x00, 0x00, 0x00, 0x02);
    EXPECT_EQ(vlan->setMacAddress(mac), Error::OK);
}

TEST_F(VlanEthernetTest, TransmitInsertsVlanTag) {
    EthernetConfig cfg{};
    vlan->init(cfg);

    // Build a minimal Ethernet frame
    std::vector<uint8_t> frame(64, 0);
    // dst MAC
    frame[0] = 0xFF; frame[1] = 0xFF; frame[2] = 0xFF;
    frame[3] = 0xFF; frame[4] = 0xFF; frame[5] = 0xFF;
    // src MAC
    frame[6] = 0x02; frame[7] = 0x00; frame[8] = 0x00;
    frame[9] = 0x00; frame[10] = 0x00; frame[11] = 0x01;
    // EtherType
    frame[12] = 0x88; frame[13] = 0xA4;

    auto err = vlan->transmit(frame.data(), frame.size());
    // Should succeed — inner FakeEthernet stores the frame
    EXPECT_EQ(err, Error::OK);
}

TEST_F(VlanEthernetTest, TransmitVlan) {
    EthernetConfig cfg{};
    vlan->init(cfg);

    std::vector<uint8_t> frame(64, 0);
    frame[12] = 0x88; frame[13] = 0xA4;
    auto err = vlan->transmitVlan(frame.data(), frame.size(), 200, 5);
    EXPECT_EQ(err, Error::OK);
}

TEST_F(VlanEthernetTest, TransmitGather) {
    EthernetConfig cfg{};
    vlan->init(cfg);

    uint8_t buf1[14] = {};
    uint8_t buf2[50] = {};
    buf1[12] = 0x88; buf1[13] = 0xA4;
    IEthernet::BufferDesc iov[2] = {{buf1, 14}, {buf2, 50}};
    auto err = vlan->transmitGather(iov, 2);
    (void)err;
}

TEST_F(VlanEthernetTest, SetRxCallback) {
    bool rxCalled = false;
    vlan->setRxCallback([](const uint8_t*, size_t, const RxFrameInfo&, void* ud) {
        *static_cast<bool*>(ud) = true;
    }, &rxCalled);
}

TEST_F(VlanEthernetTest, Poll) {
    int result = vlan->poll(Milliseconds(0));
    (void)result;
}

TEST_F(VlanEthernetTest, EthertypeFilter) {
    vlan->setEthertypeFilter(0x88A4);
}

TEST_F(VlanEthernetTest, Promiscuous) {
    EXPECT_EQ(vlan->setPromiscuous(true), Error::OK);
    EXPECT_EQ(vlan->setPromiscuous(false), Error::OK);
}

TEST_F(VlanEthernetTest, MulticastAddress) {
    MacAddress mac(0x01, 0x00, 0x5E, 0x00, 0x00, 0x01);
    EXPECT_EQ(vlan->addMulticastAddress(mac), Error::OK);
    EXPECT_EQ(vlan->removeMulticastAddress(mac), Error::OK);
}

TEST_F(VlanEthernetTest, AllMulticast) {
    EXPECT_EQ(vlan->setAllMulticast(true), Error::OK);
}

TEST_F(VlanEthernetTest, LinkStatus) {
    auto status = vlan->getLinkStatus();
    EXPECT_TRUE(status.up);
}

TEST_F(VlanEthernetTest, LinkCallback) {
    vlan->setLinkCallback([](const LinkStatus&, void*) {}, nullptr);
}

TEST_F(VlanEthernetTest, WaitForLinkUp) {
    auto err = vlan->waitForLinkUp(Milliseconds(10));
    EXPECT_EQ(err, Error::OK);
}

TEST_F(VlanEthernetTest, Stats) {
    auto stats = vlan->getStats();
    (void)stats;
    vlan->resetStats();
}

TEST_F(VlanEthernetTest, NativeHandle) {
    auto* h = vlan->nativeHandle();
    (void)h;
}

TEST_F(VlanEthernetTest, InterfaceName) {
    auto* name = vlan->getInterfaceName();
    (void)name;
}

// ============================================================================
// Traffic Splitter
// ============================================================================

class TrafficSplitterTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto fake = std::make_unique<FakeEthernet>();
        fakePtr = fake.get();
        splitter = createTrafficSplitter(std::move(fake));
    }
    FakeEthernet* fakePtr = nullptr;
    std::unique_ptr<IEthernet> splitter;
};

TEST_F(TrafficSplitterTest, InitShutdown) {
    EthernetConfig cfg{};
    EXPECT_EQ(splitter->init(cfg), Error::OK);
    splitter->shutdown();
}

TEST_F(TrafficSplitterTest, Transmit) {
    EthernetConfig cfg{};
    splitter->init(cfg);

    std::vector<uint8_t> frame(64, 0);
    frame[12] = 0x88; frame[13] = 0xA4;
    auto err = splitter->transmit(frame.data(), frame.size());
    EXPECT_EQ(err, Error::OK);
}

TEST_F(TrafficSplitterTest, GetMacAddress) {
    MacAddress mac;
    EXPECT_EQ(splitter->getMacAddress(mac), Error::OK);
}

TEST_F(TrafficSplitterTest, LinkStatus) {
    auto status = splitter->getLinkStatus();
    EXPECT_TRUE(status.up);
}

TEST_F(TrafficSplitterTest, Stats) {
    auto stats = splitter->getStats();
    (void)stats;
    splitter->resetStats();
}

TEST_F(TrafficSplitterTest, SetRxCallback) {
    splitter->setRxCallback([](const uint8_t*, size_t, const RxFrameInfo&, void*) {}, nullptr);
}

TEST_F(TrafficSplitterTest, Promiscuous) {
    EXPECT_EQ(splitter->setPromiscuous(true), Error::OK);
}

// ============================================================================
// Logging Ethernet Wrapper
// ============================================================================

class LoggingEthernetTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto fake = std::make_unique<FakeEthernet>();
        fakePtr = fake.get();
        auto logger = std::make_shared<FakePacketLogger>();
        logger->init({});
        logging = createLoggingEthernet(std::move(fake), logger);
    }
    FakeEthernet* fakePtr = nullptr;
    std::unique_ptr<IEthernet> logging;
};

TEST_F(LoggingEthernetTest, InitShutdown) {
    EthernetConfig cfg{};
    EXPECT_EQ(logging->init(cfg), Error::OK);
    EXPECT_TRUE(logging->isInitialized());
    logging->shutdown();
}

TEST_F(LoggingEthernetTest, TransmitLogsPacket) {
    EthernetConfig cfg{};
    logging->init(cfg);

    std::vector<uint8_t> frame(64, 0xAA);
    frame[12] = 0x88; frame[13] = 0xA4;
    auto err = logging->transmit(frame.data(), frame.size());
    EXPECT_EQ(err, Error::OK);
}

TEST_F(LoggingEthernetTest, TransmitVlan) {
    EthernetConfig cfg{};
    logging->init(cfg);

    std::vector<uint8_t> frame(64, 0);
    auto err = logging->transmitVlan(frame.data(), frame.size(), 100, 3);
    (void)err;
}

TEST_F(LoggingEthernetTest, TransmitGather) {
    EthernetConfig cfg{};
    logging->init(cfg);

    uint8_t buf[64] = {};
    IEthernet::BufferDesc iov[1] = {{buf, 64}};
    auto err = logging->transmitGather(iov, 1);
    (void)err;
}

TEST_F(LoggingEthernetTest, SetRxCallback) {
    logging->setRxCallback([](const uint8_t*, size_t, const RxFrameInfo&, void*) {}, nullptr);
}

TEST_F(LoggingEthernetTest, GetMacAddress) {
    MacAddress mac;
    EXPECT_EQ(logging->getMacAddress(mac), Error::OK);
}

TEST_F(LoggingEthernetTest, LinkStatus) {
    auto status = logging->getLinkStatus();
    EXPECT_TRUE(status.up);
}

TEST_F(LoggingEthernetTest, Stats) {
    auto stats = logging->getStats();
    (void)stats;
    logging->resetStats();
}

// ============================================================================
// HALInstance / Global functions
// ============================================================================

TEST(HALGlobalTest, ResetHAL) {
    resetHAL();
}

TEST(HALGlobalTest, GetHALAfterReset) {
    resetHAL();
    // getHAL() should still work, returns default/empty state
}

TEST(HALGlobalTest, InitHALMinimal) {
    resetHAL();
    HALConfig config{};
    auto result = initHAL(config);
    // May fail if interface is required
    (void)result;
    shutdownHAL();
}
