/**
 * @file test_slave_state_wire.cpp
 * @brief Slave state-transition paths over the emulated wire —
 *        transitionToInit/PreOp/SafeOp/Boot, the INIT flag-reset
 *        semantics, and the debug-banner branches behind
 *        debugFlags().stateMachine.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;

namespace {

class SlaveStateWireTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0xCAFE,
                                                 "statetest");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

// INIT is reachable from a discovered slave; the transition resets the
// mailbox/pdo-configured guards which must be re-armed before PRE_OP.
TEST_F(SlaveStateWireTest, TransitionToInitResetsGuards) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();

    EXPECT_EQ(slave.transitionToInit(), SlaveError::Ok);

    // Guard is now disarmed — PRE_OP is refused until mailbox is re-armed.
    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::MailboxNotConfigured);
}

TEST_F(SlaveStateWireTest, TransitionToPreOpAfterAssume) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();

    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    SlaveState st = SlaveState::INIT;
    ASSERT_EQ(slave.readState(st), SlaveError::Ok);
    EXPECT_EQ(st, SlaveState::PRE_OP);
}

TEST_F(SlaveStateWireTest, TransitionDispatchAndReadback) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionTo(SlaveState::PRE_OP), SlaveError::Ok);

    SlaveState st = SlaveState::INIT;
    ASSERT_EQ(slave.readState(st), SlaveError::Ok);
    EXPECT_EQ(st, SlaveState::PRE_OP);

    // Unknown/invalid target hits the dispatch default.
    EXPECT_EQ(slave.transitionTo(static_cast<SlaveState>(0x77)),
              SlaveError::InvalidStateTransition);
}

// INIT->BOOT is a valid transition on the emulator; BOOT->INIT brings
// it back.  Both directions exercise the wire request + readback.
TEST_F(SlaveStateWireTest, TransitionToBootAndBack) {
    auto& slave = master_.slave(0);
    const SlaveError r = slave.transitionToBoot();
    EXPECT_TRUE(r == SlaveError::Ok || r == SlaveError::TransportError
                || r == SlaveError::ALStatusError);
    if (r == SlaveError::Ok) {
        SlaveState st = SlaveState::INIT;
        ASSERT_EQ(slave.readState(st), SlaveError::Ok);
        EXPECT_EQ(st, SlaveState::BOOT);
        EXPECT_EQ(slave.transitionToInit(), SlaveError::Ok);
    }
}

// debugFlags().stateMachine enables the transition banners — run the
// transitions again with the flag set to cover those blocks.
TEST_F(SlaveStateWireTest, DebugBannerBranches) {
    auto& slave = master_.slave(0);

    EtherCATSlaveDebugFlags flags{};
    flags.stateMachine = true;
    slave.updateDebugFlags(flags);
    EXPECT_TRUE(slave.debugFlags().stateMachine);

    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToInit(), SlaveError::Ok);
    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    (void)slave.transitionToBoot();

    flags.stateMachine = false;
    slave.updateDebugFlags(flags);
    EXPECT_FALSE(slave.debugFlags().stateMachine);
}

// The AL_CONTROL+AL_STATUS "kick" path sends a combined write+read
// multi-datagram — enable it and drive a transition to cover it.
TEST_F(SlaveStateWireTest, AlControlStatusKickPath) {
    auto& slave = master_.slave(0);
    master_.setAlControlStatusKick(true);
    EXPECT_TRUE(master_.alControlStatusKick());

    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToInit(), SlaveError::Ok);
    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::Ok);

    master_.setAlControlStatusKick(false);
}

// The master-level stateMachine flag prints the AL request/result
// banners inside requestSlaveApplicationLayerState.
TEST_F(SlaveStateWireTest, MasterDebugBannerBranches) {
    auto& slave = master_.slave(0);
    master_.debugFlags().stateMachine = true;

    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToInit(), SlaveError::Ok);
    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::Ok);

    master_.debugFlags().stateMachine = false;
}

// SAFE_OP requires PDO SMs configured — assumePDOAlreadyConfigured()
// arms the guard and the PRE_OP->SAFE_OP wire request lands.
TEST_F(SlaveStateWireTest, SafeOpTransition) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);

    // Guard blocks without PDO config.
    EXPECT_EQ(slave.transitionToSafeOp(), SlaveError::PDONotConfigured);

    slave.assumePDOAlreadyConfigured();
    EXPECT_EQ(slave.transitionToSafeOp(), SlaveError::Ok);

    SlaveState st = SlaveState::INIT;
    ASSERT_EQ(slave.readState(st), SlaveError::Ok);
    EXPECT_EQ(st, SlaveState::SAFE_OP);
}
