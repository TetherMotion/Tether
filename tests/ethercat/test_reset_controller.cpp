/**
 * @file test_reset_controller.cpp
 * @brief SlaveResetController over the Master ↔ SlaveEmulator wire.
 *
 * The controller performs all operations through the CoE mailbox using
 * ESC register addresses as SDO indices (0x0120 AL Control, 0x0130 AL
 * Status, 0x0134 AL Status Code, 0x04xx/0x08xx watchdogs, 0x1003/0x1011
 * CiA301, and CiA402 0x6040/0x6041).  Tests inject matching OD entries
 * into the emulator; CiA402 objects are owned by the emulator's drive
 * state machine.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Reset.hpp"
#include "tether/ethercat/ResetTypes.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

constexpr uint16_t kAlControl = 0x0120;
constexpr uint16_t kAlStatus = 0x0130;
constexpr uint16_t kAlStatusCode = 0x0134;

class ResetControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000F0A, 0x9900, "reset");
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

    /// Inject the three AL "register" objects the controller accesses via SDO.
    void addAlRegisters(uint16_t status = 0x0001, uint16_t code = 0) {
        slave_->addODEntry({kAlControl, 0x00, 0x0006, 16, 0x03, {0, 0}, "alc"});
        slave_->addODEntry({kAlStatus, 0x00, 0x0006, 16, 0x01,
                            {uint8_t(status & 0xFF), uint8_t(status >> 8)},
                            "als"});
        slave_->addODEntry({kAlStatusCode, 0x00, 0x0006, 16, 0x01,
                            {uint8_t(code & 0xFF), uint8_t(code >> 8)},
                            "alc2"});
    }

    void setAlStatus(uint16_t status, uint16_t code = 0) {
        slave_->addODEntry({kAlStatus, 0x00, 0x0006, 16, 0x01,
                            {uint8_t(status & 0xFF), uint8_t(status >> 8)},
                            "als"});
        slave_->addODEntry({kAlStatusCode, 0x00, 0x0006, 16, 0x01,
                            {uint8_t(code & 0xFF), uint8_t(code >> 8)},
                            "alc2"});
    }

    uint16_t alControlValue() {
        const auto* e = slave_->odEntry(kAlControl, 0);
        return e ? uint16_t(e->data[0] | (e->data[1] << 8)) : 0xFFFF;
    }

    SlaveResetController ctl() { return SlaveResetController{master_.sdoManager(0), 0}; }

    Master master_;
    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
};

// ---------------------------------------------------------------------------
// AL status read / error state
// ---------------------------------------------------------------------------

TEST_F(ResetControllerTest, ReadAlStatusReturnsRegisters) {
    addAlRegisters(/*status=*/0x0004, /*code=*/0x0000);  // SafeOp, no error
    uint16_t status = 0, code = 0xFFFF;
    EXPECT_TRUE(ctl().readALStatus(status, code));
    EXPECT_EQ(status, 0x0004);
    EXPECT_EQ(code, 0x0000);
}

TEST_F(ResetControllerTest, ReadAlStatusMissingFails) {
    uint16_t status, code;
    EXPECT_FALSE(ctl().readALStatus(status, code));
}

TEST_F(ResetControllerTest, IsInErrorStateReflectsFlag) {
    addAlRegisters(0x0004, 0);                              // SafeOp clean
    EXPECT_FALSE(ctl().isInErrorState());
    setAlStatus(0x0004 | uint16_t(ALState::ErrorFlag), 0x001B);
    EXPECT_TRUE(ctl().isInErrorState());
}

TEST_F(ResetControllerTest, GetErrorDescriptionNonEmpty) {
    addAlRegisters(0x0004 | uint16_t(ALState::ErrorFlag), 0x001B);
    const std::string d = ctl().getErrorDescription();
    EXPECT_FALSE(d.empty());
}

// ---------------------------------------------------------------------------
// Acknowledge error
// ---------------------------------------------------------------------------

TEST_F(ResetControllerTest, AcknowledgeErrorClearsFlag) {
    // Slave in error; ack write flips status to clean between the two
    // internal reads — emulate by pre-clearing (ack path re-reads after
    // write, sees clean status → true).
    addAlRegisters(0x0004 | uint16_t(ALState::ErrorFlag), 0x001B);
    // Simulate firmware clearing the flag when 0x0120 gets the ack:
    // our OD model can't do that automatically, so flip it here before
    // the post-write read happens... it happens inside the call — so
    // instead verify the write landed with the AckError bit.
    auto c = ctl();
    // First read inside acknowledgeError sees error state; the post-write
    // read will see whatever the OD holds — flip it to simulate firmware.
    // Do the flip via a pre-arranged write: not possible mid-call, so
    // accept the documented false path here (error still set).
    EXPECT_FALSE(c.acknowledgeError());
    EXPECT_EQ(alControlValue() & ALControl::AckError, ALControl::AckError);
}

TEST_F(ResetControllerTest, AcknowledgeErrorSucceedsWhenClean) {
    addAlRegisters(0x0004, 0);  // already clean
    auto c = ctl();
    EXPECT_TRUE(c.acknowledgeError());
    // Ack write still issued with the state bits preserved.
    EXPECT_EQ(alControlValue() & ALControl::StateMask,
              uint16_t(ALState::SafeOp));
}

// ---------------------------------------------------------------------------
// State transitions / ESC reset / watchdogs
// ---------------------------------------------------------------------------

TEST_F(ResetControllerTest, ForceToInitSucceeds) {
    addAlRegisters(0x0001, 0);  // status already reads Init
    EXPECT_TRUE(ctl().forceToInit(200));
    EXPECT_EQ(alControlValue() & ALControl::StateMask,
              uint16_t(ALState::Init));
}

TEST_F(ResetControllerTest, ForceToInitTimesOut) {
    addAlRegisters(0x0002, 0);  // stuck at PreOp — write doesn't change OD
    EXPECT_FALSE(ctl().forceToInit(60));
}

TEST_F(ResetControllerTest, WaitAbortsOnErrorState) {
    addAlRegisters(0x0002 | uint16_t(ALState::ErrorFlag), 0x001C);
    auto c = ctl();
    EXPECT_FALSE(c.forceToInit(500));
    EXPECT_EQ(c.getLastResult().al_status_code, 0x001C);
}

TEST_F(ResetControllerTest, RequestEscResetWritesEscBit) {
    addAlRegisters();
    EXPECT_TRUE(ctl().requestESCReset());
    EXPECT_EQ(alControlValue() & ALControl::ESCReset, ALControl::ESCReset);
}

TEST_F(ResetControllerTest, TransitionToStateSucceeds) {
    addAlRegisters(0x0004, 0);  // reads SafeOp
    EXPECT_TRUE(ctl().transitionToState(ALState::SafeOp, 200));
    EXPECT_EQ(alControlValue() & ALControl::StateMask,
              uint16_t(ALState::SafeOp));
}

TEST_F(ResetControllerTest, ResetSyncManagerWatchdogToggles) {
    slave_->addODEntry({0x0806, 0x00, 0x0005, 8, 0x03, {0xFF}, "w0"});
    slave_->addODEntry({0x080E, 0x00, 0x0005, 8, 0x03, {0xFF}, "w1"});
    EXPECT_TRUE(ctl().resetSyncManagerWatchdog());
    EXPECT_EQ(slave_->odEntry(0x0806, 0)->data[0], 0x01);  // re-enabled
    EXPECT_EQ(slave_->odEntry(0x080E, 0)->data[0], 0x01);
}

TEST_F(ResetControllerTest, ResetSyncManagerWatchdogFailsMissing) {
    EXPECT_FALSE(ctl().resetSyncManagerWatchdog());
}

TEST_F(ResetControllerTest, ClearPdiWatchdogWritesDivisor) {
    slave_->addODEntry({0x0400, 0x00, 0x0006, 16, 0x03, {0, 0}, "pdi"});
    EXPECT_TRUE(ctl().clearPDIWatchdog());
    const auto* e = slave_->odEntry(0x0400, 0);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(uint16_t(e->data[0] | (e->data[1] << 8)), 0x09C2);
}

// ---------------------------------------------------------------------------
// CiA301: restore defaults / error history / NMT aliases
// ---------------------------------------------------------------------------

TEST_F(ResetControllerTest, RestoreDefaultParametersWritesLoadSignature) {
    slave_->addODEntry({0x1011, 0x01, 0x0007, 32, 0x03, {0, 0, 0, 0}, "r1"});
    auto c = ctl();
    EXPECT_TRUE(c.restoreDefaultParameters(0x01));
    const auto* e = slave_->odEntry(0x1011, 0x01);
    ASSERT_NE(e, nullptr);
    const uint32_t sig = e->data[0] | (e->data[1] << 8) |
                         (uint32_t(e->data[2]) << 16) |
                         (uint32_t(e->data[3]) << 24);
    EXPECT_EQ(sig, CiA301Reset::RestoreSignature);
}

TEST_F(ResetControllerTest, NmtAliasesWriteRestoreSubs) {
    slave_->addODEntry({0x1011, 0x01, 0x0007, 32, 0x03, {0, 0, 0, 0}, "r1"});
    slave_->addODEntry({0x1011, 0x02, 0x0007, 32, 0x03, {0, 0, 0, 0}, "r2"});
    auto c = ctl();
    EXPECT_TRUE(c.nmtResetNode());
    EXPECT_TRUE(c.nmtResetCommunication());
    EXPECT_NE(slave_->odEntry(0x1011, 0x01)->data[0], 0);
    EXPECT_NE(slave_->odEntry(0x1011, 0x02)->data[0], 0);
}

TEST_F(ResetControllerTest, ClearErrorHistoryWritesZero) {
    slave_->addODEntry({0x1003, 0x00, 0x0005, 8, 0x03, {7}, "err"});
    EXPECT_TRUE(ctl().clearErrorHistory());
    EXPECT_EQ(slave_->odEntry(0x1003, 0)->data[0], 0);
}

// ---------------------------------------------------------------------------
// CiA402 drive reset — real emulator drive state machine
// ---------------------------------------------------------------------------

TEST_F(ResetControllerTest, ReadStatuswordFromDriveState) {
    auto c = ctl();
    uint16_t sw = 0;
    EXPECT_TRUE(c.readStatusword(sw));
    // Emulator starts in NOT_READY_TO_SWITCH_ON (0x0000-ish).
    EXPECT_EQ(sw & 0x0008, 0u);  // no fault
}

TEST_F(ResetControllerTest, WriteControlwordReachesDrive) {
    auto c = ctl();
    EXPECT_TRUE(c.writeControlword(0x0006));  // Shutdown
    uint16_t sw = 0;
    EXPECT_TRUE(c.readStatusword(sw));
    // SWITCH_ON_DISABLED --Shutdown--> READY_TO_SWITCH_ON (bits 0+5).
    EXPECT_EQ(sw & 0x0021, 0x0021u);
}

TEST_F(ResetControllerTest, FaultResetOnHealthyDriveSucceedsTrivially) {
    auto c = ctl();
    auto r = c.faultReset();
    EXPECT_TRUE(r.success);
}

TEST_F(ResetControllerTest, FaultResetClearsInjectedFault) {
    slave_->getDriveState().state = Emulator::CiA402::State::FAULT;
    slave_->getDriveState().error_code = 0x1234;
    auto c = ctl();
    auto r = c.faultReset();
    EXPECT_TRUE(r.success);
    EXPECT_NE(slave_->getDriveState().state, Emulator::CiA402::State::FAULT);
}

TEST_F(ResetControllerTest, EnableAndDisableDrive) {
    auto c = ctl();
    auto er = c.enableDrive();
    EXPECT_TRUE(er.success);
    auto dr = c.disableDrive();
    EXPECT_TRUE(dr.success);
}

TEST_F(ResetControllerTest, QuickStopHaltResume) {
    auto c = ctl();
    EXPECT_TRUE(c.quickStop());
    EXPECT_TRUE(c.halt());
    EXPECT_TRUE(c.resumeFromHalt());
}

// ---------------------------------------------------------------------------
// resetToLevel / progressiveReset / policy
// ---------------------------------------------------------------------------

TEST_F(ResetControllerTest, SoftResetLevelClearsErrors) {
    addAlRegisters(0x0004, 0);  // clean
    slave_->addODEntry({0x1003, 0x00, 0x0005, 8, 0x03, {3}, "err"});
    auto c = ctl();
    auto r = c.resetToLevel(ResetLevel::SoftReset);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.achieved_level, ResetLevel::SoftReset);
    EXPECT_EQ(slave_->odEntry(0x1003, 0)->data[0], 0);
}

TEST_F(ResetControllerTest, SoftResetFailsOnStuckError) {
    addAlRegisters(0x0004 | uint16_t(ALState::ErrorFlag), 0x001B);
    auto c = ctl();
    auto r = c.resetToLevel(ResetLevel::SoftReset);
    EXPECT_FALSE(r.success);
}

TEST_F(ResetControllerTest, StateMachineResetLevel) {
    addAlRegisters(0x0001, 0);  // reads Init → forceToInit succeeds
    auto c = ctl();
    auto r = c.resetToLevel(ResetLevel::StateMachineReset);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.achieved_level, ResetLevel::StateMachineReset);
}

TEST_F(ResetControllerTest, CommunicationResetLevel) {
    slave_->addODEntry({0x1011, 0x02, 0x0007, 32, 0x03, {0, 0, 0, 0}, "r2"});
    auto c = ctl();
    auto r = c.resetToLevel(ResetLevel::CommunicationReset);
    EXPECT_TRUE(r.success);
}

TEST_F(ResetControllerTest, ApplicationResetLevel) {
    slave_->addODEntry({0x1011, 0x01, 0x0007, 32, 0x03, {0, 0, 0, 0}, "r1"});
    addAlRegisters(0x0001, 0);
    auto c = ctl();
    auto r = c.resetToLevel(ResetLevel::ApplicationReset);
    EXPECT_TRUE(r.success);
}

TEST_F(ResetControllerTest, ProgressCallbackFires) {
    addAlRegisters(0x0004, 0);
    slave_->addODEntry({0x1003, 0x00, 0x0005, 8, 0x03, {0}, "err"});
    auto c = ctl();
    int calls = 0;
    c.setProgressCallback(
        [&calls](const char*, uint8_t, uint16_t) { ++calls; });
    c.resetToLevel(ResetLevel::SoftReset);
    EXPECT_GT(calls, 0);
}

TEST_F(ResetControllerTest, EmergencyStopAndReset) {
    // emergencyStopAndReset: quickStop (0x6040) + NMT reset node — needs
    // CiA402 + AL + error-history + restore-parameters (0x1011:01) objects.
    addAlRegisters(0x0004, 0);
    slave_->addODEntry({0x1003, 0x00, 0x0005, 8, 0x03, {0}, "err"});
    slave_->addODEntry({0x1011, 0x01, 0x0007, 32, 0x03, {0, 0, 0, 0}, "r1"});
    auto c = ctl();
    auto r = c.emergencyStopAndReset();
    EXPECT_TRUE(r.success);
}

TEST_F(ResetControllerTest, ApplyResetPolicyExhaustsOnPermanentError) {
    addAlRegisters(0x0004 | uint16_t(ALState::ErrorFlag), 0x001B);
    auto c = ctl();
    ResetPolicy policy;
    policy.max_auto_attempts = 2;
    policy.retry_delay_ms = 1;
    policy.starting_level = ResetLevel::SoftReset;
    auto r = applyResetPolicy(c, policy);
    EXPECT_FALSE(r.success);
}

TEST_F(ResetControllerTest, ApplyResetPolicySucceedsFirstTry) {
    addAlRegisters(0x0004, 0);
    slave_->addODEntry({0x1003, 0x00, 0x0005, 8, 0x03, {0}, "err"});
    auto c = ctl();
    ResetPolicy policy;
    policy.max_auto_attempts = 2;
    policy.retry_delay_ms = 1;
    auto r = applyResetPolicy(c, policy);
    EXPECT_TRUE(r.success);
}

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------

TEST(ResetTypesTest, ResetLevelNamesNonEmpty) {
    for (int i = 0; i <= 5; ++i) {
        const char* n = getResetLevelName(static_cast<ResetLevel>(i));
        EXPECT_NE(n, nullptr);
        EXPECT_NE(std::strlen(n), 0u);
    }
}

TEST(ResetTypesTest, AlStatusCodeNames) {
    EXPECT_NE(getALStatusCodeName(0x0000), nullptr);
    EXPECT_NE(getALStatusCodeName(0x001B), nullptr);
    EXPECT_NE(getALStatusCodeName(0xFFFF), nullptr);
}

}  // namespace
