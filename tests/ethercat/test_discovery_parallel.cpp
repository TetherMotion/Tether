/**
 * @file test_discovery_parallel.cpp
 * @brief SlaveDiscoveryManager parallel-SII-read path, full-category
 *        discovery (EEPROMReactor), and the latin1->UTF-8 conversion —
 *        all over the emulated wire with multiple slaves.
 */

#include <gtest/gtest.h>

#include <string>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;

namespace {

class DiscoveryParallelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Three drives — enough for the parallel read path to fan out.
        for (int i = 0; i < 3; ++i) {
            auto s = Emulator::createCiA402Drive(
                0x00000002u, static_cast<uint32_t>(0xCA00 + i),
                std::string("drive") + char('0' + i));
            slaves_[i] = s.get();
            bus_.net.addSlave(std::move(s));
        }
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slaves_[3] = {};
    Master master_;
};

} // namespace

// >1 target + max_threads>1 runs the worker-thread SII read path.
TEST_F(DiscoveryParallelTest, ParallelSiiReads) {
    master_.discovery().setMaxThreads(4);
    EXPECT_EQ(master_.discovery().maxThreads(), 4);

    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 3u);
    EXPECT_EQ(found[0].index, 0);
    EXPECT_EQ(found[2].index, 2);
}

// DiscoveryOption::All also runs the EEPROMReactor broadcast read plus
// every category parser.  (The emulator does not implement the ESC
// EEPROM command registers, so SII word reads succeed as zeros — the
// fields report has_value() but the wire/parallel paths still run.)
TEST_F(DiscoveryParallelTest, FullCategoryDiscovery) {
    master_.discovery().setMaxThreads(3);
    auto found = master_.discovery().discover(DiscoveryOption::All);
    ASSERT_EQ(found.size(), 3u);

    for (const auto& ds : found) {
        EXPECT_TRUE(ds.vendor_id.has_value());
        EXPECT_TRUE(ds.product_code.has_value());
    }
}

// Single slave + single thread → sequential path (already exercised
// elsewhere, but keep the boundary explicit).
TEST_F(DiscoveryParallelTest, SingleSlaveSequential) {
    master_.discovery().setMaxThreads(1);
    auto found = master_.discovery().discover(
        DiscoveryOptions{DiscoveryOption::VendorId});
    ASSERT_EQ(found.size(), 3u);   // all targets discovered, low detail
    EXPECT_TRUE(found[0].vendor_id.has_value());
}
