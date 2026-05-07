/**
 * @file test_traffic_splitter.cpp
 * @brief Unit tests for Traffic Splitter
 */

#include <gtest/gtest.h>
#include "tether/hal/IEthernet.hpp"
#include "mocks/MockHAL.hpp"

using namespace EtherCAT::HAL;
using namespace EtherCAT::HAL::mock;

class TrafficSplitterTest : public ::testing::Test {
protected:
    void SetUp() override {
        fakeEth = std::make_unique<FakeEthernet>();
        fakeEthPtr = fakeEth.get();
        
        splitter = std::make_unique<TrafficSplitter>(std::move(fakeEth));
        
        // Initialize
        EthernetConfig config;
        splitter->init(config);
    }
    
    FakeEthernet* fakeEthPtr;
    std::unique_ptr<FakeEthernet> fakeEth;
    std::unique_ptr<TrafficSplitter> splitter;
    
    // Helper to build Ethernet frame
    std::vector<uint8_t> buildFrame(uint16_t etherType, size_t payloadSize = 46) {
        std::vector<uint8_t> frame(14 + payloadSize);
        // Destination MAC
        std::memset(frame.data(), 0xFF, 6);
        // Source MAC
        std::memset(frame.data() + 6, 0x02, 6);
        // EtherType
        frame[12] = (etherType >> 8) & 0xFF;
        frame[13] = etherType & 0xFF;
        // Payload
        std::memset(frame.data() + 14, 0xAA, payloadSize);
        return frame;
    }
};

TEST_F(TrafficSplitterTest, AddRemoveRule) {
    TrafficRule rule;
    rule.ethertype = 0x88A4;  // EtherCAT
    rule.priority = 10;
    
    int ruleId = splitter->addRule(rule);
    EXPECT_GT(ruleId, 0);
    
    splitter->removeRule(ruleId);
    // Should not crash
}

TEST_F(TrafficSplitterTest, RouteByEtherType) {
    bool etherCATReceived = false;
    bool ipReceived = false;
    
    // Rule for EtherCAT
    TrafficRule etherCATRule;
    etherCATRule.ethertype = 0x88A4;
    etherCATRule.priority = 10;
    etherCATRule.callback = [&](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        etherCATReceived = true;
    };
    splitter->addRule(etherCATRule);
    
    // Rule for IPv4
    TrafficRule ipRule;
    ipRule.ethertype = 0x0800;
    ipRule.priority = 5;
    ipRule.callback = [&](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        ipReceived = true;
    };
    splitter->addRule(ipRule);
    
    // Setup splitter to receive
    splitter->setRxCallback(nullptr, nullptr);
    
    // Inject EtherCAT frame
    auto ecatFrame = buildFrame(0x88A4);
    fakeEthPtr->injectFrame(ecatFrame.data(), ecatFrame.size());
    splitter->poll(10);
    
    EXPECT_TRUE(etherCATReceived);
    EXPECT_FALSE(ipReceived);
    
    // Reset and inject IP frame
    etherCATReceived = false;
    auto ipFrame = buildFrame(0x0800);
    fakeEthPtr->injectFrame(ipFrame.data(), ipFrame.size());
    splitter->poll(10);
    
    EXPECT_FALSE(etherCATReceived);
    EXPECT_TRUE(ipReceived);
}

TEST_F(TrafficSplitterTest, DefaultCallback) {
    bool defaultCalled = false;
    
    splitter->setDefaultCallback([&](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        defaultCalled = true;
    }, nullptr);
    
    splitter->setRxCallback(nullptr, nullptr);
    
    // Inject frame with EtherType not matching any rule
    auto frame = buildFrame(0x9999);
    fakeEthPtr->injectFrame(frame.data(), frame.size());
    splitter->poll(10);
    
    EXPECT_TRUE(defaultCalled);
}

TEST_F(TrafficSplitterTest, RulePriority) {
    int callOrder = 0;
    int highPriorityOrder = 0;
    int lowPriorityOrder = 0;
    
    // Lower priority rule (added first)
    TrafficRule lowRule;
    lowRule.ethertype = 0x88A4;
    lowRule.priority = 1;
    lowRule.callback = [&](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        lowPriorityOrder = ++callOrder;
    };
    splitter->addRule(lowRule);
    
    // Higher priority rule (added second but should match first)
    TrafficRule highRule;
    highRule.ethertype = 0x88A4;
    highRule.priority = 10;
    highRule.callback = [&](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        highPriorityOrder = ++callOrder;
    };
    splitter->addRule(highRule);
    
    splitter->setRxCallback(nullptr, nullptr);
    
    auto frame = buildFrame(0x88A4);
    fakeEthPtr->injectFrame(frame.data(), frame.size());
    splitter->poll(10);
    
    // Higher priority should be called first (and only, since we return after first match)
    EXPECT_EQ(highPriorityOrder, 1);
    EXPECT_EQ(lowPriorityOrder, 0);  // Never called due to first match
}

TEST_F(TrafficSplitterTest, RouteByDestMac) {
    bool specificMacReceived = false;
    
    TrafficRule macRule;
    macRule.matchDstMac = true;
    macRule.dstMac = MacAddress(0x01, 0x02, 0x03, 0x04, 0x05, 0x06);
    macRule.priority = 10;
    macRule.callback = [&](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        specificMacReceived = true;
    };
    splitter->addRule(macRule);
    
    splitter->setRxCallback(nullptr, nullptr);
    
    // Frame with matching destination MAC
    std::vector<uint8_t> frame(60);
    frame[0] = 0x01; frame[1] = 0x02; frame[2] = 0x03;
    frame[3] = 0x04; frame[4] = 0x05; frame[5] = 0x06;
    std::memset(frame.data() + 6, 0x02, 6);
    frame[12] = 0x88; frame[13] = 0xA4;
    
    fakeEthPtr->injectFrame(frame.data(), frame.size());
    splitter->poll(10);
    
    EXPECT_TRUE(specificMacReceived);
    
    // Frame with non-matching destination MAC
    specificMacReceived = false;
    frame[5] = 0x07;  // Change last byte
    fakeEthPtr->injectFrame(frame.data(), frame.size());
    splitter->poll(10);
    
    EXPECT_FALSE(specificMacReceived);  // Rule didn't match
}

TEST_F(TrafficSplitterTest, TransmitPassThrough) {
    uint8_t frame[64];
    std::memset(frame, 0xAA, sizeof(frame));
    
    EXPECT_EQ(splitter->transmit(frame, 64), Error::OK);
    
    auto txFrame = fakeEthPtr->popTxFrame();
    EXPECT_EQ(txFrame.size(), 64u);
}

TEST_F(TrafficSplitterTest, ForwardsStats) {
    uint8_t frame[64] = {0};
    splitter->transmit(frame, 64);
    splitter->transmit(frame, 32);
    
    EthernetStats stats = splitter->getStats();
    EXPECT_EQ(stats.txFrames, 2u);
}

TEST_F(TrafficSplitterTest, ForwardsLinkStatus) {
    LinkStatus status = splitter->getLinkStatus();
    EXPECT_TRUE(status.up);
    
    fakeEthPtr->setLinkUp(false);
    status = splitter->getLinkStatus();
    EXPECT_FALSE(status.up);
}

TEST_F(TrafficSplitterTest, InterfaceName) {
    EXPECT_STREQ(splitter->getInterfaceName(), "fake0");
}
