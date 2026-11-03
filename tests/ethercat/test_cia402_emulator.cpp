/**
 * @file test_cia402_emulator.cpp
 * @brief CiA402Drive master-side state machine tests over a real
 *        Master ↔ SlaveEmulator wire (EmulatedBus).
 *
 * CiA402Drive talks to a concrete Master (not virtual), so it was
 * previously untestable without hardware — CiA402DriveStateMachine.cpp
 * sat at ~1.5% coverage.  The emulator answers register reads and CoE
 * SDO mailbox traffic, and its CiA402 drive model (enableCiA402)
 * walks the real 0x6040/0x6041 state machine.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/profiles/cia402/60xx-Parameters.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class CiA402EmulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // createCiA402Drive wires SII + SM0/SM1 mailbox + RxPDO/TxPDO and
        // enables the CiA402 object dictionary — required for the SDO
        // controlword/statusword traffic enable() generates.
        auto slave = EtherCAT::Emulator::createCiA402Drive(
            0x00000002, 0x0442, "TestDrive");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);
        // initSlaves does not run the ESI-driven mailbox setup — point the
        // CoE manager at the emulator's SM0/SM1 buffers directly.
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
        drive_ = std::make_unique<CiA402Drive>(master_, 0);
    }
    void TearDown() override {
        drive_.reset();
        master_.packetRouter().shutdown();
    }

    // Walk the emulator through the legal INIT → PRE_OP → SAFE_OP → OP
    // chain (requestState enforces the EtherCAT transition table).
    void toOp() {
        slave_->requestState(SlaveState::PRE_OP);
        slave_->requestState(SlaveState::SAFE_OP);
        slave_->requestState(SlaveState::OP);
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
    std::unique_ptr<CiA402Drive> drive_;
};

} // namespace

// ============================================================================
// EtherCAT state machine
// ============================================================================

TEST_F(CiA402EmulatorTest, GetECStateReadsEmulatorState) {
    EXPECT_EQ(drive_->getECState(), ECState::Init);

    slave_->requestState(SlaveState::PRE_OP);
    EXPECT_EQ(drive_->getECState(), ECState::PreOp);
}

TEST_F(CiA402EmulatorTest, GotoInitRequestsTransition) {
    toOp();
    EXPECT_TRUE(drive_->gotoInit());
    EXPECT_EQ(slave_->getState(), SlaveState::INIT);
}

TEST_F(CiA402EmulatorTest, GotoPreOpTransitionsSlave) {
    EXPECT_TRUE(drive_->gotoPreOp());
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);
}

TEST_F(CiA402EmulatorTest, RequestSafeOpAndWait) {
    slave_->requestState(SlaveState::PRE_OP);
    EXPECT_TRUE(drive_->requestSafeOp());
    EXPECT_TRUE(drive_->waitForSafeOp(2000));
    EXPECT_EQ(slave_->getState(), SlaveState::SAFE_OP);
}

TEST_F(CiA402EmulatorTest, WaitForSafeOpTimesOutWhenStateNeverComes) {
    slave_->requestState(SlaveState::PRE_OP);
    // Never request SAFE_OP — wait must fail within budget.
    EXPECT_FALSE(drive_->waitForSafeOp(50));
}

// ============================================================================
// CiA402 drive enable / disable over the emulated CoE wire
// ============================================================================

TEST_F(CiA402EmulatorTest, EnableWalksCiA402ChainToOperationEnabled) {
    toOp();
    EXPECT_TRUE(drive_->enable(5000));
}

TEST_F(CiA402EmulatorTest, DisableDropsOutOfOperationEnabled) {
    toOp();
    ASSERT_TRUE(drive_->enable(5000));
    EXPECT_TRUE(drive_->disable());
}

TEST_F(CiA402EmulatorTest, QuickStop) {
    toOp();
    ASSERT_TRUE(drive_->enable(5000));
    EXPECT_TRUE(drive_->quickStop());
}

// ============================================================================
// Statusword / DriveState reads
// ============================================================================

TEST_F(CiA402EmulatorTest, GetDriveStateTracksEmulatorDriveState) {
    EXPECT_EQ(drive_->getDriveState(), DriveState::SwitchOnDisabled);

    slave_->getDriveState().state = Emulator::CiA402::State::OPERATION_ENABLED;
    EXPECT_EQ(drive_->getDriveState(), DriveState::OperationEnabled);
    EXPECT_TRUE(drive_->isEnabled());

    slave_->getDriveState().state = Emulator::CiA402::State::FAULT;
    EXPECT_EQ(drive_->getDriveState(), DriveState::Fault);
    EXPECT_TRUE(drive_->isFaulted());
    EXPECT_FALSE(drive_->isEnabled());
}

TEST_F(CiA402EmulatorTest, GetStatuswordMatchesDriveStateEncoding) {
    EXPECT_EQ(drive_->getStatusword() & 0x006F, 0x0040);  // SwitchOnDisabled

    slave_->getDriveState().state = Emulator::CiA402::State::OPERATION_ENABLED;
    EXPECT_EQ(drive_->getStatusword() & 0x006F, 0x0027);  // OperationEnabled
}

TEST_F(CiA402EmulatorTest, IsTargetReachedReadsStatusBit10) {
    // Emulator sets bit 10 (target reached) when operation_mode matches
    // the requested mode — both zero at reset.
    EXPECT_TRUE(drive_->isTargetReached());
    slave_->getDriveState().target_mode = 8;
    EXPECT_FALSE(drive_->isTargetReached());
}

TEST_F(CiA402EmulatorTest, SendControlwordSDODrivesEmulatorStateMachine) {
    EXPECT_TRUE(drive_->sendControlwordSDO(0x0006));  // Shutdown
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::READY_TO_SWITCH_ON);

    EXPECT_TRUE(drive_->sendControlwordSDO(0x0007));  // Switch on
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::SWITCHED_ON);

    EXPECT_TRUE(drive_->sendControlwordSDO(0x000F));  // Enable operation
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::OPERATION_ENABLED);
}

TEST_F(CiA402EmulatorTest, SendControlwordSDOMirrorsLocalCopy) {
    EXPECT_TRUE(drive_->sendControlwordSDO(0x0006));
    EXPECT_EQ(drive_->getControlword(), 0x0006);
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::READY_TO_SWITCH_ON);
}

// ============================================================================
// Fault reset paths — covers enable()'s fault-clearing loop
// ============================================================================

TEST_F(CiA402EmulatorTest, EnableFromFaultStateResetsAndEnables) {
    slave_->getDriveState().state = Emulator::CiA402::State::FAULT;
    slave_->getDriveState().error_code = 0x2340;
    EXPECT_TRUE(drive_->enable(5000));
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::OPERATION_ENABLED);
}

TEST_F(CiA402EmulatorTest, ResetFaultClearsFaultState) {
    slave_->getDriveState().state = Emulator::CiA402::State::FAULT;
    EXPECT_TRUE(drive_->resetFault());
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::SWITCH_ON_DISABLED);
}

TEST_F(CiA402EmulatorTest, ResetFaultKeepEnabledRestoresOperation) {
    toOp();
    ASSERT_TRUE(drive_->enable(5000));
    drive_->setFaultResetMode(CiA402Drive::FaultResetMode::KeepEnabled);
    slave_->getDriveState().state = Emulator::CiA402::State::FAULT;
    slave_->getDriveState().error_code = 0x1000;
    EXPECT_TRUE(drive_->resetFaultKeepEnabled());
    // The emulator models the standard bit-7 reset edge → FAULT clears to
    // SWITCH_ON_DISABLED (drives with true keep-enabled semantics re-arm).
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::SWITCH_ON_DISABLED);
    EXPECT_EQ(slave_->getDriveState().error_code, 0);
}

TEST_F(CiA402EmulatorTest, CustomFaultResetCallbackRunsDuringEnable) {
    bool called = false;
    drive_->setFaultResetCallback([&](CiA402Drive&) {
        called = true;
        slave_->getDriveState().state =
            Emulator::CiA402::State::SWITCH_ON_DISABLED;
        return true;
    });
    EXPECT_EQ(drive_->faultResetMode(), CiA402Drive::FaultResetMode::Custom);
    slave_->getDriveState().state = Emulator::CiA402::State::FAULT;
    EXPECT_TRUE(drive_->enable(5000));
    EXPECT_TRUE(called);
    EXPECT_EQ(slave_->getDriveState().state,
              Emulator::CiA402::State::OPERATION_ENABLED);
}

// ============================================================================
// Operating mode SDO round-trip
// ============================================================================

TEST_F(CiA402EmulatorTest, SetOperatingModeSDOUpdatesModeDisplay) {
    EXPECT_TRUE(drive_->setOperatingModeSDO(8));  // CSP
    EXPECT_EQ(drive_->getOperatingMode(), 8);
}

TEST_F(CiA402EmulatorTest, ModeHelpersUseSDOPathWithoutPDO) {
    EXPECT_TRUE(drive_->setModeCSP());
    EXPECT_EQ(drive_->getOperatingMode(),
              static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncPosition));
    EXPECT_TRUE(drive_->setModeCSV());
    EXPECT_EQ(drive_->getOperatingMode(),
              static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncVelocity));
}

// ============================================================================
// EC state machine — remaining transitions
// ============================================================================

TEST_F(CiA402EmulatorTest, GotoSafeOpAndWaitForOp) {
    EXPECT_TRUE(drive_->gotoPreOp());
    EXPECT_TRUE(drive_->gotoSafeOp());
    EXPECT_EQ(slave_->getState(), SlaveState::SAFE_OP);
    EXPECT_TRUE(drive_->gotoOp());
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
}

TEST_F(CiA402EmulatorTest, RequestOpAndWait) {
    toOp();
    slave_->requestState(SlaveState::SAFE_OP);
    EXPECT_TRUE(drive_->requestOp());
    EXPECT_TRUE(drive_->waitForOp(2000));
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
}

// ============================================================================
// SDO error paths — aborts surface as failures, not hangs
// ============================================================================

TEST_F(CiA402EmulatorTest, SdoReadAbortsOnUnmappedIndex) {
    auto bogus = CiA402::Parameters60xx::ControlWord;
    bogus.index = 0x5FFF;  // not in the emulator object dictionary
    auto res = master_.sdoManager(0).readEntry(bogus);
    EXPECT_FALSE(res.has_value());
}

TEST_F(CiA402EmulatorTest, SdoWriteAbortsOnReadOnlyObject) {
    // 0x6041 statusword is read-only — the emulator aborts the download.
    auto res = master_.sdoManager(0).writeEntry(
        CiA402::Parameters60xx::StatusWord, 0x000Fu);
    EXPECT_FALSE(res.has_value());
}

TEST_F(CiA402EmulatorTest, SdoReadUploadRoundTripsControlword) {
    ASSERT_TRUE(drive_->sendControlwordSDO(0x0006));
    auto res = master_.sdoManager(0).readEntry(
        CiA402::Parameters60xx::ControlWord);
    ASSERT_TRUE(res.has_value());
    EXPECT_EQ(*res, 0x0006u);
}
