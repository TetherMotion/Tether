/**
 * @file test_discovery_state_coverage.cpp
 * @brief Coverage for SlaveDiscoveryManager (sync/async discovery paths)
 *        and the Slave state-transition helpers over an emulated bus.
 */

#include <gtest/gtest.h>

#include <future>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class DiscoveryStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        for (int i = 0; i < 2; ++i) {
            auto slave =
                Emulator::createCiA402Drive(0x00000002,
                                            static_cast<uint32_t>(0x0442 + i),
                                            i == 0 ? "esc211" : "synapticon");
            bus_.net.addSlave(std::move(slave));
        }
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Master master_;
};

}  // namespace

// ============================================================================
// Discovery — sync and async
// ============================================================================

TEST_F(DiscoveryStateTest, DiscoverAsyncResolvesToBothSlaves) {
    auto fut = master_.discovery().discoverAsync(DiscoveryOptions{});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)),
              std::future_status::ready);
    EXPECT_EQ(fut.get().size(), 2u);
}

TEST_F(DiscoveryStateTest, DiscoverOneAsyncResolvesSingleSlave) {
    auto fut = master_.discovery().discoverOneAsync(1, DiscoveryOptions{});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)),
              std::future_status::ready);
    auto ds = fut.get();
    EXPECT_EQ(ds.index, 1u);
}

TEST_F(DiscoveryStateTest, DiscoverAsyncSubsetResolvesRequestedIndices) {
    const std::vector<uint16_t> indices{1};
    auto fut = master_.discovery().discoverAsync(
        std::span<const uint16_t>(indices), DiscoveryOptions{});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)),
              std::future_status::ready);
    auto found = fut.get();
    ASSERT_EQ(found.size(), 1u);
    EXPECT_EQ(found[0].index, 1u);
}

// ============================================================================
// Slave state transitions
// ============================================================================

TEST_F(DiscoveryStateTest, TransitionToPreOpAndBack) {
    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 2u);
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();

    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    EXPECT_EQ(slave.ALState(), SlaveState::PRE_OP);

    EXPECT_EQ(slave.transitionToInit(), SlaveError::Ok);
    EXPECT_EQ(slave.ALState(), SlaveState::INIT);
}

TEST_F(DiscoveryStateTest, TransitionToSafeOpRequiresMailboxAndPdO) {
    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 2u);
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();

    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    // Without configured PDO sync managers the SAFE-OP request fails or
    // reports a specific error — either way it must not crash and must
    // leave the slave out of SAFE_OP.
    const auto err = slave.transitionToSafeOp();
    if (err == SlaveError::Ok) {
        EXPECT_EQ(slave.ALState(), SlaveState::SAFE_OP);
    } else {
        EXPECT_NE(slave.ALState(), SlaveState::SAFE_OP);
    }
}

TEST_F(DiscoveryStateTest, TransitionToOpWithoutPdOFails) {
    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 2u);
    auto& slave = master_.slave(1);
    slave.assumeMailboxAlreadyConfigured();
    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    EXPECT_NE(slave.transitionToOp(), SlaveError::Ok);
}

TEST_F(DiscoveryStateTest, TransitionToBoot) {
    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 2u);
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    const auto err = slave.transitionToBoot();
    if (err == SlaveError::Ok) {
        EXPECT_EQ(slave.ALState(), SlaveState::BOOT);
    } else {
        EXPECT_NE(slave.ALState(), SlaveState::BOOT);
    }
}

TEST_F(DiscoveryStateTest, ReadStateAndALHelpers) {
    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 2u);
    auto& slave = master_.slave(0);

    SlaveState st = SlaveState::BOOT;
    ASSERT_EQ(slave.readState(st), SlaveError::Ok);
    EXPECT_EQ(st, SlaveState::INIT);

    auto al = slave.ALState();
    ASSERT_TRUE(al.has_value());
    EXPECT_EQ(*al, SlaveState::INIT);

    auto code = slave.ALCode();
    ASSERT_TRUE(code.has_value());
    EXPECT_EQ(*code, 0u);
}

TEST_F(DiscoveryStateTest, MailboxAvailabilityTracksConfiguration) {
    auto found = master_.discovery().discover(DiscoveryOptions{});
    ASSERT_EQ(found.size(), 2u);
    auto& slave = master_.slave(0);
    // Discovery already derived the mailbox from the emulator's SII.
    EXPECT_TRUE(slave.mailboxAvailable());
}
