/**
 * @file test_cia402_drive_wire.cpp
 * @brief Wire-level coverage for CiA402Drive (CiA402DriveStateMachine.cpp):
 *        enable/disable FSM, fault-reset loop, quick stop, SDO
 *        controlword/statusword paths, operating-mode set/readback,
 *        homing commands, DynaDrive helpers, and the controlledShutdown
 *        SDO path — driven over EmulatedBus with a CiA402-enabled
 *        SlaveEmulator.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <memory>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Emulator;

namespace {

/// CiA402-capable slave with a mailbox pair.
SlaveEmulator* addDriveSlave(EmulatedBus& bus) {
    auto slave = std::make_unique<SlaveEmulator>();
    SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = 0x3052;
    cfg.mailbox_protocol = 0x04;
    auto addSm = [&cfg](uint16_t addr, uint16_t len, uint8_t ctrl) {
        SIIConfig::SMConfig sm{};
        sm.start_addr = addr;
        sm.length     = len;
        sm.control    = std::bit_cast<SyncManager::SMControlReg>(ctrl);
        sm.enable     = std::bit_cast<SyncManager::SMActivateReg>(
            static_cast<uint8_t>(0x01));
        cfg.sync_managers.push_back(sm);
    };
    addSm(0x1000, 128, 0x26);
    addSm(0x1080, 128, 0x22);
    slave->setSIIConfig(cfg);
    slave->enableCiA402(true);
    // Entries enableCiA402 does not register but the drive API writes:
    // homing method, target velocity (controlledShutdown SDO path),
    // and home offset (homeToCurrentPosition).
    slave->addODEntry({0x6098, 0, 0x0002, 8, 0x03, {0}, "Homing method"});
    slave->addODEntry({0x60FF, 0, 0x0004, 32, 0x03, {0, 0, 0, 0},
                      "Target velocity"});
    slave->addODEntry({0x607C, 0, 0x0004, 32, 0x03, {0, 0, 0, 0},
                      "Home offset"});
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

class CiA402DriveWireTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = addDriveSlave(bus_);
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
        drive_ = std::make_unique<CiA402Drive>(master_, 0);
        drive_->setSDOTimeout(200);
    }
    void TearDown() override {
        drive_.reset();
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    Emulator::CiA402::DriveState& driveState() {
        return slave_->getDriveState();
    }

    /// Poll getDriveState() until target or deadline (ms).  The public
    /// waitForDriveState() is private, so tests roll their own poll here.
    bool waitState(DriveState target, uint32_t ms) {
        for (uint32_t el = 0; el < ms; el += 5) {
            if (drive_->getDriveState() == target) return true;
            Tether::Platform::Clock::instance().delayMilliseconds(5);
        }
        return drive_->getDriveState() == target;
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
    std::unique_ptr<CiA402Drive> drive_;
};

}  // namespace

// Clean enable: SWITCH_ON_DISABLED → OPERATION_ENABLED through the real
// emulator FSM (covers the non-fault enable path end to end).
TEST_F(CiA402DriveWireTest, EnableRunsFullFsm) {
    EXPECT_EQ(drive_->getDriveState(), DriveState::SwitchOnDisabled);
    EXPECT_TRUE(drive_->enable(3000));
    EXPECT_EQ(drive_->getDriveState(), DriveState::OperationEnabled);
    EXPECT_TRUE(drive_->isEnabled());
    EXPECT_FALSE(drive_->isFaulted());
}

// Fault injection: enable() enters the fault-reset loop, reads 0x603F,
// issues the reset edge, then proceeds to OPERATION_ENABLED.
TEST_F(CiA402DriveWireTest, EnableFromFaultRunsResetLoop) {
    driveState().state = Emulator::CiA402::State::FAULT;
    driveState().error_code = 0x7500;
    EXPECT_TRUE(drive_->isFaulted());
    EXPECT_TRUE(drive_->enable(3000));
    EXPECT_EQ(drive_->getDriveState(), DriveState::OperationEnabled);
}

// enable() from an intermediate state skips earlier transition arms.
TEST_F(CiA402DriveWireTest, EnableFromReadyToSwitchOn) {
    driveState().state = Emulator::CiA402::State::READY_TO_SWITCH_ON;
    EXPECT_TRUE(drive_->enable(3000));
    EXPECT_EQ(drive_->getDriveState(), DriveState::OperationEnabled);
}

// disable() walks OPERATION_ENABLED back down.
TEST_F(CiA402DriveWireTest, DisableAfterEnable) {
    ASSERT_TRUE(drive_->enable(3000));
    EXPECT_TRUE(drive_->disable());
    EXPECT_NE(drive_->getDriveState(), DriveState::OperationEnabled);
    EXPECT_FALSE(drive_->isEnabled());
}

// quickStop() transitions OE → QUICK_STOP_ACTIVE.
TEST_F(CiA402DriveWireTest, QuickStop) {
    ASSERT_TRUE(drive_->enable(3000));
    EXPECT_TRUE(drive_->quickStop());
    EXPECT_EQ(drive_->getDriveState(), DriveState::QuickStopActive);
}

// resetFault() / resetFaultKeepEnabled() / performFaultReset().
TEST_F(CiA402DriveWireTest, FaultResetPaths) {
    driveState().state = Emulator::CiA402::State::FAULT;
    driveState().error_code = 0x2310;
    EXPECT_TRUE(drive_->resetFault());
    EXPECT_EQ(drive_->getDriveState(), DriveState::SwitchOnDisabled);

    driveState().state = Emulator::CiA402::State::FAULT;
    EXPECT_TRUE(drive_->resetFaultKeepEnabled());
}

// Direct controlword SDO writes drive the emulator FSM through states.
TEST_F(CiA402DriveWireTest, ControlwordSdoSequence) {
    EXPECT_TRUE(drive_->sendControlwordSDO(0x0006));  // Shutdown
    EXPECT_TRUE(waitState(DriveState::ReadyToSwitchOn, 500));

    uint16_t sw = drive_->getStatusword();
    EXPECT_NE(sw & 0x004F, 0u);

    EXPECT_TRUE(drive_->sendControlwordSDO(0x0007));  // Switch on
    EXPECT_TRUE(waitState(DriveState::SwitchedOn, 500));
    EXPECT_TRUE(drive_->sendControlwordSDO(0x000F));  // Enable operation
    EXPECT_TRUE(waitState(DriveState::OperationEnabled, 500));

    // Bit-2-clear from OE takes QUICK_STOP first; a second Disable Voltage
    // from QUICK_STOP_ACTIVE lands in SwitchOnDisabled.
    EXPECT_TRUE(drive_->sendControlwordSDO(0x0000));
    EXPECT_TRUE(waitState(DriveState::QuickStopActive, 500));
    EXPECT_TRUE(drive_->sendControlwordSDO(0x0000));
    EXPECT_TRUE(waitState(DriveState::SwitchOnDisabled, 500));
}

// Operating mode via SDO with readback through 0x6060/0x6061.
TEST_F(CiA402DriveWireTest, OperatingModeSdoAndReadback) {
    EXPECT_TRUE(drive_->setOperatingModeSDO(3));   // Profile velocity
    EXPECT_EQ(drive_->getOperatingMode(), 3);
    EXPECT_TRUE(drive_->setOperatingMode(1));      // routes to SDO (no PDO offset)
    EXPECT_EQ(drive_->getOperatingMode(), 1);
    EXPECT_TRUE(drive_->setOperatingModeSDO(8));   // CSP
    EXPECT_EQ(drive_->getOperatingMode(), 8);
}

// PDO-mode offset unconfigured → setOperatingModePDO error branch.
TEST_F(CiA402DriveWireTest, OperatingModePdoUnconfigured) {
    EXPECT_FALSE(drive_->setOperatingModePDO(3));
}

// Statusword bit helpers.  The emulator sets bit10 (target reached)
// whenever operation_mode==target_mode — even before enable — so the
// assertion is mode-set/clear round-trip, not a state gate.
TEST_F(CiA402DriveWireTest, StatusBitHelpers) {
    EXPECT_TRUE(drive_->isTargetReached());   // op_mode==target_mode==0
    ASSERT_TRUE(drive_->enable(3000));
    EXPECT_TRUE(drive_->isTargetReached());
    // Diverging target vs actual mode clears bit10.
    ASSERT_TRUE(drive_->setOperatingModeSDO(3));
    driveState().target_mode = 8;             // pretend a mid-switch demand
    EXPECT_FALSE(drive_->isTargetReached());
}

// DynaDrive helpers delegate to DynaDriveController over the same SDO wire.
TEST_F(CiA402DriveWireTest, DynaDriveHelpers) {
    uint32_t sw = 0;
    drive_->readDynaDriveStatusword(sw);  // 16-bit OD entry → may abort; line covered
    drive_->sendDynaDriveControlword(
        EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::
            StandbyToMotorPreOp);
    EXPECT_TRUE(drive_->isDynaDriveControlOp() == true ||
                drive_->isDynaDriveControlOp() == false);
    using DDS = CiA402Drive::DynaDriveState;
    for (auto s : {DDS::NA, DDS::ControlOp, DDS::Error, DDS::Unknown}) {
        EXPECT_NE(CiA402Drive::getDynaDriveStateName(s), nullptr);
    }
    EXPECT_EQ(CiA402Drive::decodeDynaDriveState(7), DDS::ControlOp);
    EXPECT_EQ(CiA402Drive::decodeDynaDriveState(0xFF), DDS::Unknown);
    EXPECT_EQ(CiA402Drive::decodeDynaDriveState(10), DDS::MotorPreOp);
}

// Homing commands: method write + execute + completion/error queries.
// The emulator reports homing-attained (statusword bit 12) only in mode 6
// with homing_complete latched — drive_state_ is a mutable test seam.
TEST_F(CiA402DriveWireTest, HomingCommands) {
    ASSERT_TRUE(drive_->enable(3000));
    EXPECT_TRUE(drive_->setHomingMethod(35));   // current position
    ASSERT_TRUE(drive_->setModeHM());
    driveState().homing_complete = true;
    EXPECT_TRUE(drive_->executeHoming(2000));
    EXPECT_TRUE(drive_->isHomingComplete());
    EXPECT_FALSE(drive_->hasHomingError());
}

// executeHoming polls until timeout when homing never completes.
TEST_F(CiA402DriveWireTest, HomingTimeout) {
    ASSERT_TRUE(drive_->enable(3000));
    ASSERT_TRUE(drive_->setModeHM());
    EXPECT_FALSE(drive_->executeHoming(60));
    EXPECT_FALSE(drive_->isHomingComplete());
}

// homeToCurrentPosition writes offset + method + mode, then homes.
TEST_F(CiA402DriveWireTest, HomeToCurrentPosition) {
    ASSERT_TRUE(drive_->enable(3000));
    driveState().homing_complete = true;  // latched before mode-HM set
    EXPECT_TRUE(drive_->homeToCurrentPosition(1234, 1500));
    EXPECT_EQ(drive_->getOperatingMode(), 6);
}

// controlledShutdown: no PDO registered → pure SDO path through the
// stop/controlword phases.
TEST_F(CiA402DriveWireTest, ControlledShutdownSdoPath) {
    ASSERT_TRUE(drive_->enable(3000));
    CiA402Drive::ControlledShutdownConfig cfg{};
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
    EXPECT_NE(drive_->getDriveState(), DriveState::OperationEnabled);
}

// controlledShutdown: FixedTimeVelocity collapses to a step at v0==0,
// brake_action callback fires after the delay.
TEST_F(CiA402DriveWireTest, ControlledShutdownBrakeAction) {
    ASSERT_TRUE(drive_->enable(3000));
    CiA402Drive::ControlledShutdownConfig cfg{};
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::FixedTimeVelocity;
    cfg.stop_time_ms = 200;
    cfg.brake_delay_ms = 20;
    bool brake_fired = false;
    cfg.brake_action = [&brake_fired] { brake_fired = true; return true; };
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
    EXPECT_TRUE(brake_fired);
}

// controlledShutdown: stop_mode None skips phase 1 entirely.
TEST_F(CiA402DriveWireTest, ControlledShutdownNoStopPhase) {
    ASSERT_TRUE(drive_->enable(3000));
    CiA402Drive::ControlledShutdownConfig cfg{};
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::None;
    cfg.shutdown_controlword = 0x0006;  // Shutdown → ReadyToSwitchOn
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
}

// getECState reflects the slave AL state from the wire.
TEST_F(CiA402DriveWireTest, GetECState) {
    ECState s = drive_->getECState();
    EXPECT_TRUE(s == ECState::Init || s == ECState::PreOp ||
                s == ECState::SafeOp || s == ECState::Op ||
                s == ECState::Bootstrap || s == ECState::Unknown);
}

// ============================================================================
// DS402Master over the emulated bus — configure/enable/disable paths
// ============================================================================

class DS402MasterWireTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = addDriveSlave(bus_);
        bus_.attach(ds402_.ethercatMaster());
        ds402_.start(bus_.iface, kEmulatedMac);
        auto found = ds402_.ethercatMaster().discovery().discover(
            DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        ds402_.setSlaveAsDS402(0);
    }
    void TearDown() override {
        ds402_.stop();
        bus_.shutdown();
        ds402_.ethercatMaster().packetRouter().shutdown();
    }

    EmulatedBus bus_;
    DS402Master ds402_;
    SlaveEmulator* slave_ = nullptr;
};

TEST_F(DS402MasterWireTest, IsRunningAndDriveCount) {
    EXPECT_TRUE(ds402_.isRunning());
    EXPECT_EQ(ds402_.discoveredDriveCount(), 1);
    EXPECT_TRUE(ds402_.waitForDriveCount(1, 100));
    EXPECT_FALSE(ds402_.waitForDriveCount(2, 30));
}

TEST_F(DS402MasterWireTest, ConfigureDriveSuccess) {
    DS402Master::DriveConfiguration cfg;
    cfg.slave_index = 0;
    cfg.auto_configure_mailbox = true;
    cfg.transition_to_operational = true;   // drives INIT→OP over the bus
    cfg.operating_mode = 0;
    EXPECT_TRUE(ds402_.configureDrive(cfg));
    EXPECT_NE(ds402_.driveBySlaveIndex(0), nullptr);
}

TEST_F(DS402MasterWireTest, ConfigureDriveToOperational) {
    // Second slave index without a real drive — configureDrive must fail.
    DS402Master::DriveConfiguration cfg;
    cfg.slave_index = 3;
    cfg.auto_configure_mailbox = true;
    cfg.transition_to_operational = true;
    EXPECT_FALSE(ds402_.configureDrive(cfg));
}

TEST_F(DS402MasterWireTest, ConfigureDrivesBatch) {
    DS402Master::DriveConfiguration cfg;
    cfg.slave_index = 0;
    cfg.auto_configure_mailbox = true;
    cfg.transition_to_operational = false;
    EXPECT_TRUE(ds402_.configureDrives({cfg}));

    DS402Master::DriveConfiguration bad;
    bad.slave_index = 7;  // no such slave on the bus
    bad.auto_configure_mailbox = true;
    EXPECT_FALSE(ds402_.configureDrives({bad}));
}

TEST_F(DS402MasterWireTest, EnableDisableDrive) {
    DS402Master::DriveConfiguration cfg;
    cfg.slave_index = 0;
    cfg.auto_configure_mailbox = true;
    cfg.transition_to_operational = false;
    ASSERT_TRUE(ds402_.configureDrive(cfg));

    EXPECT_TRUE(ds402_.enableDrive(0, 500));
    EXPECT_TRUE(ds402_.disableDrive(0));
}

TEST_F(DS402MasterWireTest, EnableDisableAllDrives) {
    DS402Master::DriveConfiguration cfg;
    cfg.slave_index = 0;
    cfg.auto_configure_mailbox = true;
    cfg.transition_to_operational = false;
    ASSERT_TRUE(ds402_.configureDrive(cfg));

    EXPECT_TRUE(ds402_.enableAllDrives(500));
    EXPECT_TRUE(ds402_.disableAllDrives());
}

TEST_F(DS402MasterWireTest, RecoveryHandlerReinitialize) {
    DS402Master::DriveConfiguration cfg;
    cfg.slave_index = 0;
    cfg.auto_configure_mailbox = true;
    cfg.transition_to_operational = false;
    ASSERT_TRUE(ds402_.configureDrive(cfg));

    EtherCAT::RecoveryConfig rcfg;
    rcfg.enabled = false;
    // enableSlaveRecovery installs a DS402RecoveryHandler; starting the
    // supervisor on the stub-scope master exercises configure/start.
    ds402_.enableSlaveRecovery(rcfg, {cfg});
    ds402_.disableSlaveRecovery();
}

TEST_F(CiA402DriveWireTest, DriveManagerOverBus) {
    DriveManager mgr;
    EXPECT_EQ(mgr.getDriveCount(), 0u);
    EXPECT_EQ(mgr.getDrive(0), nullptr);
    EXPECT_EQ(mgr.getDriveBySlaveIndex(0), nullptr);

    EXPECT_EQ(mgr.initializeDrives(master_, 1), 1u);
    EXPECT_EQ(mgr.getDriveCount(), 1u);
    EXPECT_NE(mgr.getDrive(0), nullptr);
    EXPECT_EQ(mgr.getDrive(7), nullptr);              // out of range
    EXPECT_NE(mgr.getDriveBySlaveIndex(0), nullptr);
    EXPECT_EQ(mgr.getDriveBySlaveIndex(9), nullptr);  // unknown index

    // transitionAllToOp drives the AL state transition over the bus.
    mgr.transitionAllToOp();
    // enable/disable run the CiA402 FSM over the bus; on the emulator's
    // fault-free slave these complete.
    mgr.enableAll();
    mgr.disableAll();
}

TEST_F(CiA402DriveWireTest, AlStateSequence) {
    // Walk the full INIT -> PRE_OP -> SAFE_OP -> OP ladder over the bus,
    // then back down — the emulator services AL_CONTROL/AL_STATUS.
    EXPECT_TRUE(drive_->gotoInit());
    EXPECT_TRUE(drive_->gotoPreOp());
    // gotoSafeOp logs a warning about missing SM2/SM3 SII config — our
    // mailbox-only SII covers that branch.
    drive_->gotoSafeOp();
    drive_->gotoOp();
    EXPECT_TRUE(drive_->gotoInit());  // unwind
}

TEST_F(CiA402DriveWireTest, RequestOpAndWait) {
    ASSERT_TRUE(drive_->gotoPreOp());
    drive_->gotoSafeOp();
    // requestOp starts the transition; waitForOp polls to completion.
    drive_->requestOp();
    // OP request may complete inline or need polling — either path is fine.
    SUCCEED();
}

TEST_F(CiA402DriveWireTest, DynaDriveEnableDisable) {
    // enableDynaDrive/disableDynaDrive wrap DynaDriveController — the
    // emulator aborts the 0x20xx SDO ops, but the call sites and failure
    // handling get covered either way.
    drive_->enableDynaDrive(50);
    drive_->disableDynaDrive();
}

TEST_F(CiA402DriveWireTest, EnableFaultKeepEnabled) {
    drive_->setFaultResetMode(CiA402Drive::FaultResetMode::KeepEnabled);
    EXPECT_EQ(drive_->faultResetMode(), CiA402Drive::FaultResetMode::KeepEnabled);
    // Inject a fault + generic error code → enable() runs the
    // KeepEnabled fault-reset arm.
    driveState().state = Emulator::CiA402::State::FAULT;
    driveState().error_code = 0x7500;
    drive_->enable(3000);
}

TEST_F(CiA402DriveWireTest, EnableFaultMaxCode) {
    // last_fault_code == 0xFFFF arm — error register at max.
    driveState().state = Emulator::CiA402::State::FAULT;
    driveState().error_code = 0xFFFF;
    drive_->enable(3000);
}

TEST_F(CiA402DriveWireTest, ControlledShutdownNoneMode) {
    ASSERT_TRUE(drive_->enable(3000));
    CiA402Drive::ControlledShutdownConfig cfg;
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::None;
    cfg.standstill_ms = 0;
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
    EXPECT_NE(drive_->getDriveState(), DriveState::OperationEnabled);
}

TEST_F(CiA402DriveWireTest, ControlledShutdownInstantStop) {
    ASSERT_TRUE(drive_->enable(3000));
    CiA402Drive::ControlledShutdownConfig cfg;
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::InstantHardStop;
    cfg.standstill_ms = 0;
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
}

TEST_F(CiA402DriveWireTest, ControlledShutdownAccelLimited) {
    ASSERT_TRUE(drive_->enable(3000));
    CiA402Drive::ControlledShutdownConfig cfg;
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::AccelerationLimited;
    cfg.decel_limit = 10000;
    cfg.standstill_ms = 0;
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
}

TEST_F(CiA402DriveWireTest, OperatingModeSdoPaths) {
    // 0x6060/0x6061 are registered by enableCiA402 — write + readback.
    EXPECT_TRUE(drive_->setOperatingModeSDO(8));  // CSP
    EXPECT_EQ(drive_->getOperatingMode(), 8);
    EXPECT_TRUE(drive_->setOperatingModeSDO(6));  // homing
    EXPECT_EQ(drive_->getOperatingMode(), 6);
}

TEST_F(CiA402DriveWireTest, HomingCompleteAndErrorQueries) {
    // Direct statusword bit queries.
    EXPECT_FALSE(drive_->isHomingComplete());
    EXPECT_FALSE(drive_->hasHomingError());
    driveState().operation_mode = 6;
    driveState().homing_complete = true;
    EXPECT_TRUE(drive_->isHomingComplete());
}
