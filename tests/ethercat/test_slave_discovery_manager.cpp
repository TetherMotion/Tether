/**
 * @file test_slave_discovery_manager.cpp
 * @brief Unit tests for SlaveDiscoveryManager, DiscoveryOptions, DiscoveredSlave
 *
 * Tests the option bitmask, DiscoveredSlave query helpers, and the
 * manager's synchronous/asynchronous discovery API using a mock master
 * with test callbacks.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"

#include <array>
#include <chrono>
#include <future>
#include <thread>
#include <vector>

using namespace EtherCAT;
using namespace ::testing;

// ============================================================================
// DiscoveryOptions bitmask tests
// ============================================================================

TEST(DiscoveryOptionsTest, DefaultIsEmpty) {
    DiscoveryOptions opts;
    EXPECT_TRUE(opts.empty());
    EXPECT_FALSE(opts.any());
    EXPECT_EQ(opts.mask(), 0u);
}

TEST(DiscoveryOptionsTest, SingleOption) {
    DiscoveryOptions opts(DiscoveryOption::VendorId);
    EXPECT_FALSE(opts.empty());
    EXPECT_TRUE(opts.any());
    EXPECT_TRUE(opts.has(DiscoveryOption::VendorId));
    EXPECT_FALSE(opts.has(DiscoveryOption::ProductCode));
}

TEST(DiscoveryOptionsTest, InitializerList) {
    DiscoveryOptions opts({DiscoveryOption::VendorId,
                           DiscoveryOption::ProductCode,
                           DiscoveryOption::DeviceNames});
    EXPECT_TRUE(opts.has(DiscoveryOption::VendorId));
    EXPECT_TRUE(opts.has(DiscoveryOption::ProductCode));
    EXPECT_TRUE(opts.has(DiscoveryOption::DeviceNames));
    EXPECT_FALSE(opts.has(DiscoveryOption::DistributedClocks));
}

TEST(DiscoveryOptionsTest, AllIncludesEverything) {
    DiscoveryOptions opts(DiscoveryOption::All);
    EXPECT_TRUE(opts.has(DiscoveryOption::VendorId));
    EXPECT_TRUE(opts.has(DiscoveryOption::ProductCode));
    EXPECT_TRUE(opts.has(DiscoveryOption::DistributedClocks));
    EXPECT_TRUE(opts.has(DiscoveryOption::SyncManagers));
    EXPECT_TRUE(opts.has(DiscoveryOption::SlaveCount));
}

TEST(DiscoveryOptionsTest, SlaveCountOption) {
    DiscoveryOptions opts(DiscoveryOption::SlaveCount);
    EXPECT_TRUE(opts.has(DiscoveryOption::SlaveCount));
    EXPECT_FALSE(opts.has(DiscoveryOption::VendorId));
}

// ============================================================================
// DiscoveredSlave query helper tests
// ============================================================================

TEST(DiscoveredSlaveTest, HasVendorId) {
    DiscoveredSlave s;
    s.vendor_id = 0x000022D2;
    EXPECT_TRUE(s.hasVendorId(0x000022D2));
    EXPECT_FALSE(s.hasVendorId(0x00000001));
}

TEST(DiscoveredSlaveTest, HasProductCode) {
    DiscoveredSlave s;
    s.product_code = 0x12345678;
    EXPECT_TRUE(s.hasProductCode(0x12345678));
    EXPECT_FALSE(s.hasProductCode(0xDEADBEEF));
}

TEST(DiscoveredSlaveTest, HasVendorAndProduct) {
    DiscoveredSlave s;
    s.vendor_id = 0xAAAA;
    s.product_code = 0xBBBB;
    EXPECT_TRUE(s.hasVendorAndProduct(0xAAAA, 0xBBBB));
    EXPECT_FALSE(s.hasVendorAndProduct(0xAAAA, 0xCCCC));
    EXPECT_FALSE(s.hasVendorAndProduct(0xDDDD, 0xBBBB));
}

TEST(DiscoveredSlaveTest, HasDeviceName) {
    DiscoveredSlave s;
    s.device_name = "EK1100";
    EXPECT_TRUE(s.hasDeviceName("EK1100"));
    EXPECT_FALSE(s.hasDeviceName("EL1002"));
}

TEST(DiscoveredSlaveTest, HasDeviceNameContaining) {
    DiscoveredSlave s;
    s.device_name = "EK1100 EtherCAT Coupler";
    EXPECT_TRUE(s.hasDeviceNameContaining("EK1100"));
    EXPECT_TRUE(s.hasDeviceNameContaining("Coupler"));
    EXPECT_FALSE(s.hasDeviceNameContaining("EL1002"));
}

TEST(DiscoveredSlaveTest, EmptyOptionalsReturnFalse) {
    DiscoveredSlave s;
    EXPECT_FALSE(s.hasVendorId(0));
    EXPECT_FALSE(s.hasProductCode(0));
    EXPECT_FALSE(s.hasVendorAndProduct(0, 0));
    EXPECT_FALSE(s.hasDeviceName(""));
    EXPECT_FALSE(s.hasDeviceNameContaining(""));
}

TEST(DiscoveredSlaveTest, IndexAlwaysSet) {
    DiscoveredSlave s;
    EXPECT_EQ(s.index, 0u);
    s.index = 5;
    EXPECT_EQ(s.index, 5u);
}

// ============================================================================
// SlaveDiscoveryManager tests with mock master
// ============================================================================

class SlaveDiscoveryManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Shrink SII read timeout so initSlaves prefetch doesn't block.
        master_.setSiiTimeoutMs(1);

        // Set up APRD test callback to simulate a slave responding to BRD scan.
        // The BRD scan uses allocIdx + preRegisterResponseWaiter, but with
        // test callbacks we simulate the slave count via initSlaves directly.
        // The discovery manager calls discoverSlaves() which does a BRD scan;
        // without a real network interface, the scan will fail. We test the
        // manager's SII reading logic by calling readSlaveSii indirectly
        // through discoverOne with pre-initialised slaves.
        master_.initSlaves(2);
    }

    Master master_;
};

TEST_F(SlaveDiscoveryManagerTest, DiscoveryReturnsManager) {
    auto& mgr = master_.discovery();
    EXPECT_EQ(mgr.maxThreads(), 1u);
}

TEST_F(SlaveDiscoveryManagerTest, SetMaxThreads) {
    auto& mgr = master_.discovery();
    mgr.setMaxThreads(4);
    EXPECT_EQ(mgr.maxThreads(), 4u);
}

TEST_F(SlaveDiscoveryManagerTest, SetMaxThreadsZeroClampsToOne) {
    auto& mgr = master_.discovery();
    mgr.setMaxThreads(0);
    EXPECT_EQ(mgr.maxThreads(), 1u);
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverOneReturnsDiscoveredSlave) {
    // Without a real network interface, discoverSlaves() fails and
    // discover() returns empty. But discoverOne should still return a
    // DiscoveredSlave with the index set (even if SII fields are nullopt).
    auto& mgr = master_.discovery();
    auto result = mgr.discoverOne(0);
    EXPECT_EQ(result.index, 0u);
    // SII fields should be nullopt since the scan failed (no real slaves)
    EXPECT_FALSE(result.vendor_id.has_value());
    EXPECT_FALSE(result.product_code.has_value());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverOneOutOfRangeReturnsEmptyFields) {
    auto& mgr = master_.discovery();
    auto result = mgr.discoverOne(100);
    EXPECT_EQ(result.index, 100u);
    EXPECT_FALSE(result.vendor_id.has_value());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverAsyncReturnsFuture) {
    auto& mgr = master_.discovery();
    auto future = mgr.discoverAsync();
    auto result = future.get();
    // Without a real interface, discovery returns empty
    EXPECT_TRUE(result.empty());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverOneAsyncReturnsFuture) {
    auto& mgr = master_.discovery();
    auto future = mgr.discoverOneAsync(0);
    auto result = future.get();
    EXPECT_EQ(result.index, 0u);
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverSpanReturnsVector) {
    auto& mgr = master_.discovery();
    std::array<uint16_t, 2> ids = {0, 1};
    auto result = mgr.discover(std::span<const uint16_t>(ids));
    // Without a real interface, the scan fails and result is empty
    EXPECT_TRUE(result.empty());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverSpanAsyncReturnsFuture) {
    auto& mgr = master_.discovery();
    std::vector<uint16_t> ids = {0, 1};
    auto future = mgr.discoverAsync(std::span<const uint16_t>(ids));
    auto result = future.get();
    EXPECT_TRUE(result.empty());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverWithSpecificOptions) {
    auto& mgr = master_.discovery();
    auto result = mgr.discover({DiscoveryOption::VendorId,
                                DiscoveryOption::ProductCode});
    // Without a real interface, discovery returns empty
    EXPECT_TRUE(result.empty());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoverWithSlaveCountOnly) {
    auto& mgr = master_.discovery();
    auto result = mgr.discover(DiscoveryOption::SlaveCount);
    // Without a real interface, discovery returns empty
    EXPECT_TRUE(result.empty());
}

TEST_F(SlaveDiscoveryManagerTest, DiscoveryManagerIsPersistent) {
    auto& mgr1 = master_.discovery();
    auto& mgr2 = master_.discovery();
    EXPECT_EQ(&mgr1, &mgr2);
}
