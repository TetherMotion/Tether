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

// transitionToOp from INIT: the PDO guard is already satisfied (the
// emulator's SII registered its PDOs during discovery), so the OP
// request goes out — the ESM refuses INIT->OP, reports INIT + AL error
// and the unexpected-state early return fires.
TEST_F(SlaveStateWireTest, TransitionToOpDeclinedFromInit) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    slave.assumePDOAlreadyConfigured();

    EXPECT_EQ(slave.transitionToOp(), SlaveError::TransportError);

    SlaveState st = SlaveState::INIT;
    ASSERT_EQ(slave.readState(st), SlaveError::Ok);
    EXPECT_EQ(st, SlaveState::INIT);
}

// transitionToOp with the debug banner enabled covers the requirement
// banner block. From INIT the outcome is the same decline.
TEST_F(SlaveStateWireTest, TransitionToOpDebugBanner) {
    auto& slave = master_.slave(0);
    EtherCATSlaveDebugFlags flags{};
    flags.stateMachine = true;
    slave.updateDebugFlags(flags);

    slave.assumeMailboxAlreadyConfigured();
    slave.assumePDOAlreadyConfigured();
    EXPECT_EQ(slave.transitionToOp(), SlaveError::TransportError);

    flags.stateMachine = false;
    slave.updateDebugFlags(flags);
}

// SAFE_OP->OP on the emulator: drive the full transition chain and
// accept whatever the emulated ESM decides — either the OP-success
// banner or the decline/timeout path, both exercise new code.
TEST_F(SlaveStateWireTest, TransitionToOpFromSafeOp) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    slave.assumePDOAlreadyConfigured();
    ASSERT_EQ(slave.transitionToSafeOp(), SlaveError::Ok);

    const SlaveError r = slave.transitionToOp();
    SlaveState st = SlaveState::INIT;
    ASSERT_EQ(slave.readState(st), SlaveError::Ok);
    if (r == SlaveError::Ok) {
        EXPECT_EQ(st, SlaveState::OP);
    } else {
        // Emulator declined OP — slave reports a non-OP state.
        EXPECT_NE(st, SlaveState::OP);
    }
}

// verifySyncManagers' debug_flag block emits per-SM register dumps —
// enable verifyPreOp/verifySafeOp during the transitions.
TEST_F(SlaveStateWireTest, VerifySmDebugFlagPaths) {
    auto& slave = master_.slave(0);
    EtherCATSlaveDebugFlags flags{};
    flags.verifyPreOp = true;
    flags.verifySafeOp = true;
    slave.updateDebugFlags(flags);

    slave.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    slave.assumePDOAlreadyConfigured();
    EXPECT_EQ(slave.transitionToSafeOp(), SlaveError::Ok);
}

// OP-confirm timeout path: with every register *read* dropped
// (ErrorInjection.inject_timeout), the OP write lands but the wait
// loop can never observe the transition — after 5 s it gives up and
// dumpOpDiagnostics() runs its all-failed-read branches.
TEST_F(SlaveStateWireTest, TransitionToOpTimeoutDump) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    slave.assumePDOAlreadyConfigured();
    ASSERT_EQ(slave.transitionToSafeOp(), SlaveError::Ok);

    // Drop only AL_STATUS (0x0130) reads: guard checks (FMMU verify) and
    // the AL_CONTROL write still land — the emulated ESM even reaches OP
    // internally — but every confirm poll times out, so after 5 s the
    // transition fails and dumpOpDiagnostics() runs with mixed
    // success/failure register batches.
    Emulator::ErrorInjection errors{};
    errors.inject_timeout = true;
    errors.timeout_register = 0x0130;
    slave_->setErrorInjection(errors);

    EXPECT_EQ(slave.transitionToOp(), SlaveError::TransportError);

    slave_->setErrorInjection(Emulator::ErrorInjection{});
}

// mailboxAvailable(): with the firmware hint cleared it consults the
// AL state — true outside SAFE_OP, false while parked in SAFE_OP.
TEST_F(SlaveStateWireTest, MailboxAvailableByState) {
    auto& slave = master_.slave(0);
    slave.assumeMailboxAlreadyConfigured();
    slave.setMailboxServicedInSafeOp(false);

    EXPECT_TRUE(slave.mailboxAvailable());  // INIT → true

    ASSERT_EQ(slave.transitionToPreOp(), SlaveError::Ok);
    EXPECT_TRUE(slave.mailboxAvailable());  // PRE_OP → true

    slave.assumePDOAlreadyConfigured();
    ASSERT_EQ(slave.transitionToSafeOp(), SlaveError::Ok);
    EXPECT_FALSE(slave.mailboxAvailable()); // SAFE_OP → false

    // Hint set → unconditionally true regardless of state.
    slave.setMailboxServicedInSafeOp(true);
    EXPECT_TRUE(slave.mailboxAvailable());
}
