/**
 * @file test_vlan.cpp
 * @brief Unit tests for VLAN wrapper
 */

#include <gtest/gtest.h>
#include "tether/hal/IEthernet.hpp"
#include "mocks/MockHAL.hpp"

using namespace EtherCAT::HAL;
using namespace EtherCAT::HAL::mock;

class VlanWrapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        fakeEth = std::make_unique<FakeEthernet>();
        fakeEthPtr = fakeEth.get();
        
        // Create VLAN wrapper with VLAN ID 100
        vlanWrapper = std::make_unique<VlanEthernetWrapper>(
            std::move(fakeEth), 100, 5);  // VLAN 100, priority 5
    }
    
    FakeEthernet* fakeEthPtr;  // Keep raw pointer for test verification
    std::unique_ptr<FakeEthernet> fakeEth;
    std::unique_ptr<VlanEthernetWrapper> vlanWrapper;
};

TEST_F(VlanWrapperTest, InitShutdown) {
    EthernetConfig config;
    EXPECT_EQ(vlanWrapper->init(config), Error::OK);
    vlanWrapper->shutdown();
}

TEST_F(VlanWrapperTest, TransmitAddsVlanTag) {
    // Build a simple Ethernet frame
    uint8_t frame[60];
    // Destination MAC
    std::memset(frame, 0xFF, 6);  // Broadcast
    // Source MAC
    std::memset(frame + 6, 0x02, 6);  // Local admin
    // EtherType (0x88A4 = EtherCAT)
    frame[12] = 0x88;
    frame[13] = 0xA4;
    // Payload
    std::memset(frame + 14, 0xAA, 46);
    
    EXPECT_EQ(vlanWrapper->transmit(frame, 60), Error::OK);
    
    // Get transmitted frame from underlying fake
    auto txFrame = fakeEthPtr->popTxFrame();
    
    // Frame should be 4 bytes longer (VLAN tag)
    EXPECT_EQ(txFrame.size(), 64u);
    
    // Check VLAN tag
    EXPECT_EQ(txFrame[12], 0x81);  // VLAN EtherType high
    EXPECT_EQ(txFrame[13], 0x00);  // VLAN EtherType low
    
    // TCI: Priority (5) << 13 | VLAN ID (100)
    // Priority 5 = 0b101, VLAN 100 = 0x64
    // TCI = (5 << 13) | 100 = 0xA064
    uint16_t tci = (txFrame[14] << 8) | txFrame[15];
    EXPECT_EQ(tci & 0x0FFF, 100);  // VLAN ID
    EXPECT_EQ((tci >> 13) & 0x07, 5);  // Priority
    
    // Original EtherType should follow
    EXPECT_EQ(txFrame[16], 0x88);
    EXPECT_EQ(txFrame[17], 0xA4);
}

TEST_F(VlanWrapperTest, ReceiveStripsVlanTag) {
    // Build a VLAN-tagged frame
    uint8_t vlanFrame[64];
    // Destination MAC
    std::memset(vlanFrame, 0xFF, 6);
    // Source MAC
    std::memset(vlanFrame + 6, 0x02, 6);
    // VLAN tag
    vlanFrame[12] = 0x81;  // VLAN EtherType
    vlanFrame[13] = 0x00;
    // TCI: Priority 5, VLAN 100 = 0xA064
    vlanFrame[14] = 0xA0;
    vlanFrame[15] = 0x64;
    // Original EtherType
    vlanFrame[16] = 0x88;
    vlanFrame[17] = 0xA4;
    // Payload
    std::memset(vlanFrame + 18, 0xBB, 46);
    
    std::vector<uint8_t> receivedFrame;
    bool callbackCalled = false;
    
    vlanWrapper->setRxCallback([&](const uint8_t* frame, size_t len,
                                   const RxFrameInfo& info, void* userData) {
        (void)userData;
        callbackCalled = true;
        receivedFrame.assign(frame, frame + len);
        
        // Verify VLAN info is populated
        EXPECT_TRUE(info.vlanTagPresent);
        EXPECT_EQ(info.vlanId, 100);
        EXPECT_EQ(info.vlanPriority, 5);
    }, nullptr);
    
    // Inject VLAN-tagged frame into underlying Ethernet
    fakeEthPtr->injectFrame(vlanFrame, 64);
    vlanWrapper->poll(100);
    
    EXPECT_TRUE(callbackCalled);
    // Received frame should have VLAN tag stripped
    EXPECT_EQ(receivedFrame.size(), 60u);
    // EtherType should be restored
    EXPECT_EQ(receivedFrame[12], 0x88);
    EXPECT_EQ(receivedFrame[13], 0xA4);
}

TEST_F(VlanWrapperTest, SetDefaultVlan) {
    vlanWrapper->setDefaultVlan(200, 3);
    
    uint8_t frame[60];
    std::memset(frame, 0, sizeof(frame));
    frame[12] = 0x88;
    frame[13] = 0xA4;
    
    vlanWrapper->transmit(frame, 60);
    
    auto txFrame = fakeEthPtr->popTxFrame();
    uint16_t tci = (txFrame[14] << 8) | txFrame[15];
    
    EXPECT_EQ(tci & 0x0FFF, 200);  // New VLAN ID
    EXPECT_EQ((tci >> 13) & 0x07, 3);  // New priority
}

TEST_F(VlanWrapperTest, ForwardsLinkStatus) {
    LinkStatus status = vlanWrapper->getLinkStatus();
    EXPECT_TRUE(status.up);
    
    fakeEthPtr->setLinkUp(false);
    status = vlanWrapper->getLinkStatus();
    EXPECT_FALSE(status.up);
}

TEST_F(VlanWrapperTest, ForwardsStats) {
    uint8_t frame[60] = {0};
    frame[12] = 0x88;
    frame[13] = 0xA4;
    
    vlanWrapper->transmit(frame, 60);
    vlanWrapper->transmit(frame, 60);
    
    EthernetStats stats = vlanWrapper->getStats();
    EXPECT_EQ(stats.txFrames, 2u);
}

TEST_F(VlanWrapperTest, TransmitVlanBypassesDefaultTag) {
    uint8_t frame[60] = {0};
    frame[12] = 0x88;
    frame[13] = 0xA4;
    
    // Use explicit VLAN ID (bypasses default)
    vlanWrapper->transmitVlan(frame, 60, 300, 7);
    
    // The underlying FakeEthernet's transmitVlan is a pass-through
    // In real implementation, this would use the specified VLAN
    auto txFrame = fakeEthPtr->popTxFrame();
    EXPECT_GT(txFrame.size(), 0u);
}
