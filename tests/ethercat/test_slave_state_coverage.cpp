/**
 * @file test_slave_state_coverage.cpp
 * @brief Emulator-backed coverage for Slave state-machine transitions —
 *        transitionTo dispatch, INIT/PRE_OP/SAFE_OP/OP/BOOT gates and
 *        the AL status query helpers.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class SlaveStateCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = std::make_unique<SlaveEmulator>();
        Emulator::SIIConfig cfg;
        cfg.vendor_id = 0x00000002;
        cfg.product_code = 0x0FA13052;
        cfg.mailbox_protocol = 0x04;
        slave->setSIIConfig(cfg);
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }

    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    Slave& sl() { return master_.slave(0); }

    void injectAllTimeouts() {
        Emulator::ErrorInjection inj;
        inj.inject_timeout = true;
        slave_->setErrorInjection(inj);
    }

    // Write-path failures: after master.stop() the transport is torn
    // down — every register write/report fails as a transport error.
    void injectBadWkc() { master_.stop(); }

    void toPreOp() {
        if (slave_->getState() != SlaveState::PRE_OP)
            slave_->requestState(SlaveState::PRE_OP);
        ASSERT_EQ(slave_->getState(), SlaveState::PRE_OP);
    }

    void toSafeOp() {
        toPreOp();
        slave_->requestState(SlaveState::SAFE_OP);
        ASSERT_EQ(slave_->getState(), SlaveState::SAFE_OP);
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

// ============================================================================
// transitionTo() dispatch
// ============================================================================

TEST_F(SlaveStateCoverageTest, TransitionToInitSucceeds) {
    EXPECT_EQ(sl().transitionTo(SlaveState::INIT), SlaveError::Ok);
    EXPECT_EQ(slave_->getState(), SlaveState::INIT);
    // INIT resets the configuration flags.
    EXPECT_FALSE(sl().isMailboxConfigured());
    EXPECT_FALSE(sl().isPDOConfigured());
}

TEST_F(SlaveStateCoverageTest, TransitionToInitResetsFlags) {
    sl().assumeMailboxAlreadyConfigured();
    sl().assumePDOAlreadyConfigured();
    EXPECT_EQ(sl().transitionToInit(), SlaveError::Ok);
    EXPECT_FALSE(sl().isMailboxConfigured());
    EXPECT_FALSE(sl().isPDOConfigured());
}

TEST_F(SlaveStateCoverageTest, TransitionToInitTransportError) {
    injectBadWkc();
    EXPECT_EQ(sl().transitionToInit(), SlaveError::TransportError);
}

TEST_F(SlaveStateCoverageTest, TransitionToRejectsUnknownState) {
    EXPECT_EQ(sl().transitionTo(static_cast<SlaveState>(0xEE)),
              SlaveError::InvalidStateTransition);
}

// ============================================================================
// transitionToPreOp — mailbox gate
// ============================================================================

TEST_F(SlaveStateCoverageTest, PreOpFailsWithoutMailbox) {
    EXPECT_EQ(sl().transitionToPreOp(), SlaveError::MailboxNotConfigured);
}

TEST_F(SlaveStateCoverageTest, PreOpSucceedsWithMailbox) {
    sl().assumeMailboxAlreadyConfigured();
    EXPECT_EQ(sl().transitionToPreOp(), SlaveError::Ok);
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveStateCoverageTest, PreOpViaTransitionToDispatch) {
    sl().markNoMailbox();
    EXPECT_EQ(sl().transitionTo(SlaveState::PRE_OP), SlaveError::Ok);
}

TEST_F(SlaveStateCoverageTest, PreOpTransportError) {
    sl().assumeMailboxAlreadyConfigured();
    injectBadWkc();
    EXPECT_EQ(sl().transitionToPreOp(), SlaveError::TransportError);
}

// ============================================================================
// transitionToSafeOp — PDO gate
// ============================================================================

TEST_F(SlaveStateCoverageTest, SafeOpFailsWithoutPdoConfig) {
    EXPECT_EQ(sl().transitionToSafeOp(), SlaveError::PDONotConfigured);
}

TEST_F(SlaveStateCoverageTest, SafeOpSucceedsWithPdoAssumed) {
    toPreOp();
    sl().assumePDOAlreadyConfigured();
    EXPECT_EQ(sl().transitionToSafeOp(), SlaveError::Ok);
    EXPECT_EQ(slave_->getState(), SlaveState::SAFE_OP);
}

TEST_F(SlaveStateCoverageTest, SafeOpTransportError) {
    toPreOp();
    sl().assumePDOAlreadyConfigured();
    injectBadWkc();
    EXPECT_EQ(sl().transitionToSafeOp(), SlaveError::TransportError);
}

// ============================================================================
// transitionToOp
// ============================================================================

TEST_F(SlaveStateCoverageTest, OpTransitionWarnsWithoutPdoButProceeds) {
    // With no PDO entries configured the PDO-counter gate is skipped and
    // the emulator grants OP.
    toSafeOp();
    EXPECT_EQ(sl().transitionTo(SlaveState::OP), SlaveError::Ok);
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
}

TEST_F(SlaveStateCoverageTest, OpTransitionCancelledWhileWaiting) {
    // Give the slave PDO entries so the PDO-counter wait loop runs, then
    // cancel immediately — the loop must observe the cancellation.
    auto& pdo = master_.pdoForSlave(0);
    pdo.mapping().add_rxpdo(0, 4, 0x1600, PDO::PDOAddressMode::Logical);
    pdo.mapping().add_txpdo(0, 4, 0x1A00, PDO::PDOAddressMode::Logical);
    master_.requestCancel();
    EXPECT_EQ(sl().transitionToOp(), SlaveError::Cancelled);
}

TEST_F(SlaveStateCoverageTest, OpTransitionTransportError) {
    injectBadWkc();
    EXPECT_EQ(sl().transitionToOp(), SlaveError::TransportError);
}

// ============================================================================
// transitionToBoot
// ============================================================================

TEST_F(SlaveStateCoverageTest, BootTransitionFromInit) {
    EXPECT_EQ(sl().transitionToBoot(), SlaveError::Ok);
    EXPECT_EQ(slave_->getState(), SlaveState::BOOT);
}

TEST_F(SlaveStateCoverageTest, BootTransitionTransportError) {
    injectBadWkc();
    EXPECT_EQ(sl().transitionToBoot(), SlaveError::TransportError);
}

// ============================================================================
// AL status helpers
// ============================================================================

TEST_F(SlaveStateCoverageTest, ReadStateAndHelpers) {
    SlaveState st = SlaveState::OP;
    EXPECT_EQ(sl().readState(st), SlaveError::Ok);
    EXPECT_EQ(st, slave_->getState());

    auto al = sl().ALState();
    ASSERT_TRUE(al.has_value());
    EXPECT_EQ(*al, slave_->getState());

    auto code = sl().ALCode();
    EXPECT_TRUE(code.has_value());

    uint16_t raw = 0xFFFF;
    EXPECT_EQ(sl().readALStatusCode(raw), SlaveError::Ok);
    EXPECT_NE(raw, 0xFFFF);
}


// ============================================================================
// Debug-flag arms (transition banners + verifySyncManagers)
// ============================================================================

TEST_F(SlaveStateCoverageTest, DebugFlaggedTransitions) {
    EtherCATSlaveDebugFlags flags{};
    flags.stateMachine = true;
    flags.verifySafeOp = true;
    flags.mailboxConfiguration = true;
    sl().updateDebugFlags(flags);

    EXPECT_EQ(sl().transitionToInit(), SlaveError::Ok);
    sl().assumeMailboxAlreadyConfigured();
    EXPECT_EQ(sl().transitionToPreOp(), SlaveError::Ok);
    sl().assumePDOAlreadyConfigured();
    EXPECT_EQ(sl().transitionToSafeOp(), SlaveError::Ok);
}

TEST_F(SlaveStateCoverageTest, SafeOpVerifyMismatchLogged) {
    toPreOp();
    sl().assumePDOAlreadyConfigured();
    // Seed an expected SM config that cannot match the emulator's SM
    // registers — verifySyncManagers must report the mismatch.
    EtherCATSlaveDebugFlags flags{};
    flags.verifySafeOp = true;
    sl().updateDebugFlags(flags);

    auto& pdo = master_.pdoForSlave(0);
    auto* cfgs = pdo.slaveConfigs();
    cfgs[0].sm[0].enable = true;
    cfgs[0].sm[0].phys_start_addr = 0xDEAD;   // emulator mailbox SM is at 0x1000
    cfgs[0].sm[0].length = 0xBEEF;
    cfgs[0].sm[1].enable = true;
    cfgs[0].sm[1].phys_start_addr = 0xDEAD;
    cfgs[0].sm[1].length = 0xBEEF;

    // The mismatch is logged but the transition still proceeds.
    EXPECT_EQ(sl().transitionToSafeOp(), SlaveError::Ok);
}

TEST_F(SlaveStateCoverageTest, OpTransitionFailsWhenSlaveStuckInInit) {
    // INIT -> OP is an invalid ESM jump: the emulator refuses the AL
    // control write and the confirm poll sees INIT, hitting the
    // unexpected-state abort arm.
    ASSERT_EQ(slave_->getState(), SlaveState::INIT);
    EXPECT_EQ(sl().transitionToOp(), SlaveError::TransportError);
}

TEST_F(SlaveStateCoverageTest, ReadHelpersFailOnTransportError) {
    injectAllTimeouts();
    SlaveState st;
    EXPECT_EQ(sl().readState(st), SlaveError::TransportError);
    EXPECT_FALSE(sl().ALState().has_value());
    EXPECT_FALSE(sl().ALCode().has_value());
    uint16_t raw;
    EXPECT_EQ(sl().readALStatusCode(raw), SlaveError::TransportError);
}

}  // namespace
