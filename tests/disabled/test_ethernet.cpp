/**
 * @file test_ethernet.cpp
 * @brief Unit tests for Ethernet HAL interface
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tether/hal/IEthernet.hpp"
#include "mocks/MockHAL.hpp"

using namespace EtherCAT::HAL;
using namespace EtherCAT::HAL::mock;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::DoAll;
using ::testing::SetArgReferee;

// ============================================================================
// FakeEthernet Tests
// ============================================================================

class FakeEthernetTest : public ::testing::Test {
protected:
    void SetUp() override {
        eth = std::make_unique<FakeEthernet>();
    }

    std::unique_ptr<FakeEthernet> eth;
};

TEST_F(FakeEthernetTest, InitShutdown) {
    EthernetConfig config;
    EXPECT_EQ(eth->init(config), Error::OK);
    EXPECT_TRUE(eth->isInitialized());
    eth->shutdown();
    EXPECT_FALSE(eth->isInitialized());
}

TEST_F(FakeEthernetTest, GetMacAddress) {
    MacAddress mac;
    EXPECT_EQ(eth->getMacAddress(mac), Error::OK);
    // Should be locally administered (0x02 in first byte)
    EXPECT_TRUE(mac.isLocallyAdministered());
}

TEST_F(FakeEthernetTest, SetMacAddress) {
    MacAddress newMac(0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34);
    EXPECT_EQ(eth->setMacAddress(newMac), Error::OK);
    
    MacAddress retrieved;
    EXPECT_EQ(eth->getMacAddress(retrieved), Error::OK);
    EXPECT_EQ(retrieved, newMac);
}

TEST_F(FakeEthernetTest, LinkStatus) {
    LinkStatus status = eth->getLinkStatus();
    EXPECT_TRUE(status.up);
    EXPECT_EQ(status.speedMbps, 100u);
    
    eth->setLinkUp(false);
    status = eth->getLinkStatus();
    EXPECT_FALSE(status.up);
    
    eth->setLinkUp(true);
    status = eth->getLinkStatus();
    EXPECT_TRUE(status.up);
}

TEST_F(FakeEthernetTest, PromiscuousMode) {
    EXPECT_EQ(eth->setPromiscuous(true), Error::OK);
    EXPECT_EQ(eth->setPromiscuous(false), Error::OK);
}

TEST_F(FakeEthernetTest, TransmitFrame) {
    uint8_t txFrame[64];
    for (int i = 0; i < 64; i++) txFrame[i] = i;
    
    EXPECT_EQ(eth->transmit(txFrame, 64), Error::OK);
    
    // Frame should be queued for inspection
    auto sentFrame = eth->popTxFrame();
    ASSERT_EQ(sentFrame.size(), 64u);
    for (int i = 0; i < 64; i++) {
        EXPECT_EQ(sentFrame[i], i);
    }
}

TEST_F(FakeEthernetTest, InjectAndPoll) {
    uint8_t rxFrame[64];
    for (int i = 0; i < 64; i++) rxFrame[i] = 64 - i;
    
    bool callbackCalled = false;
    std::vector<uint8_t> receivedFrame;
    
    eth->setRxCallback([&](const uint8_t* frame, size_t len, 
                            const RxFrameInfo& info, void* userData) {
        (void)info; (void)userData;
        callbackCalled = true;
        receivedFrame.assign(frame, frame + len);
    }, nullptr);
    
    eth->injectFrame(rxFrame, 64);
    
    int count = eth->poll(100);
    EXPECT_GT(count, 0);
    EXPECT_TRUE(callbackCalled);
    ASSERT_EQ(receivedFrame.size(), 64u);
    for (int i = 0; i < 64; i++) {
        EXPECT_EQ(receivedFrame[i], 64 - i);
    }
}

TEST_F(FakeEthernetTest, PollTimeout) {
    int count = eth->poll(50);
    EXPECT_EQ(count, 0);
}

TEST_F(FakeEthernetTest, Stats) {
    uint8_t frame[64] = {0};
    
    eth->transmit(frame, 64);
    eth->transmit(frame, 32);
    
    eth->setRxCallback([](const uint8_t*, size_t, const RxFrameInfo&, void*) {}, nullptr);
    eth->injectFrame(frame, 48);
    eth->poll(10);
    
    EthernetStats stats = eth->getStats();
    EXPECT_EQ(stats.txFrames, 2u);
    EXPECT_EQ(stats.txBytes, 96u);
    EXPECT_EQ(stats.rxFrames, 1u);
    EXPECT_EQ(stats.rxBytes, 48u);
    
    eth->resetStats();
    stats = eth->getStats();
    EXPECT_EQ(stats.txFrames, 0u);
}

TEST_F(FakeEthernetTest, TransmitGather) {
    uint8_t buf1[] = {0x01, 0x02, 0x03};
    uint8_t buf2[] = {0x04, 0x05};
    uint8_t buf3[] = {0x06, 0x07, 0x08, 0x09};
    
    IEthernet::BufferDesc buffers[3] = {
        {buf1, 3},
        {buf2, 2},
        {buf3, 4}
    };
    
    EXPECT_EQ(eth->transmitGather(buffers, 3), Error::OK);
    
    auto frame = eth->popTxFrame();
    ASSERT_EQ(frame.size(), 9u);
    for (int i = 0; i < 9; i++) {
        EXPECT_EQ(frame[i], i + 1);
    }
}

TEST_F(FakeEthernetTest, MulticastAddress) {
    MacAddress mcast(0x01, 0x00, 0x5E, 0x00, 0x00, 0x01);
    
    EXPECT_EQ(eth->addMulticastAddress(mcast), Error::OK);
    EXPECT_EQ(eth->removeMulticastAddress(mcast), Error::OK);
}

TEST_F(FakeEthernetTest, InterfaceName) {
    EXPECT_STREQ(eth->getInterfaceName(), "fake0");
}

// ============================================================================
// MockEthernet Tests
// ============================================================================

TEST(MockEthernetTest, ExpectCalls) {
    MockEthernet mockEth;
    
    EthernetConfig config;
    EXPECT_CALL(mockEth, init(testing::_)).WillOnce(Return(Error::OK));
    EXPECT_CALL(mockEth, getLinkStatus()).WillRepeatedly(Return(LinkStatus{true, 100, true, false}));
    EXPECT_CALL(mockEth, transmit(_, _)).WillOnce(Return(Error::OK));
    EXPECT_CALL(mockEth, shutdown()).Times(1);
    
    EXPECT_EQ(mockEth.init(config), Error::OK);
    EXPECT_TRUE(mockEth.getLinkStatus().up);
    
    uint8_t frame[64] = {0};
    EXPECT_EQ(mockEth.transmit(frame, 64), Error::OK);
    
    mockEth.shutdown();
}

TEST(MockEthernetTest, SimulateError) {
    MockEthernet mockEth;
    
    EthernetConfig config;
    EXPECT_CALL(mockEth, init(_)).WillOnce(Return(Error::OK));
    EXPECT_CALL(mockEth, transmit(_, _))
        .WillOnce(Return(Error::LinkDown))
        .WillOnce(Return(Error::OK));
    
    mockEth.init(config);
    
    uint8_t frame[64] = {0};
    EXPECT_EQ(mockEth.transmit(frame, 64), Error::LinkDown);
    EXPECT_EQ(mockEth.transmit(frame, 64), Error::OK);
}

TEST(MockEthernetTest, GetMacAddressWithMock) {
    MockEthernet mockEth;
    
    MacAddress expectedMac(0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF);
    
    EXPECT_CALL(mockEth, getMacAddress(_))
        .WillOnce(DoAll(SetArgReferee<0>(expectedMac), Return(Error::OK)));
    
    MacAddress mac;
    EXPECT_EQ(mockEth.getMacAddress(mac), Error::OK);
    EXPECT_EQ(mac, expectedMac);
}

// ============================================================================
// Ethernet Frame Format Tests
// ============================================================================

TEST(EthernetFrameTest, MinimumSize) {
    // Minimum Ethernet frame (without FCS)
    constexpr size_t MIN_FRAME_SIZE = 60;
    constexpr size_t HEADER_SIZE = 14;
    constexpr size_t MIN_PAYLOAD = MIN_FRAME_SIZE - HEADER_SIZE;
    
    EXPECT_EQ(MIN_PAYLOAD, 46u);
}

TEST(EthernetFrameTest, MaximumSize) {
    // Maximum standard Ethernet frame (without FCS)
    constexpr size_t MAX_FRAME_SIZE = 1514;  // 14 header + 1500 payload
    constexpr size_t MTU = 1500;
    
    EXPECT_EQ(MAX_FRAME_SIZE - 14, MTU);
}

// ============================================================================
// BufferDesc Tests
// ============================================================================

TEST(BufferDescTest, Construction) {
    uint8_t data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    IEthernet::BufferDesc bd{data, 10};
    
    EXPECT_EQ(bd.data, data);
    EXPECT_EQ(bd.length, 10u);
}
