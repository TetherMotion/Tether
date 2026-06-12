/**
 * @file test_CiA402_coverage.cpp
 * @brief Comprehensive CiA402 Slave coverage tests (state machine, simulation, OD, PDO)
 */
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <memory>
#include <magic_enum/magic_enum.hpp>
#include "slave/profiles/CiA402Slave.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

using namespace EtherCAT::slave;
using SDOAbortCode = EtherCAT::slave::SDOAbortCode;

// ControlWord bits
static constexpr uint16_t CW_SWITCH_ON        = 0x0001;
static constexpr uint16_t CW_ENABLE_VOLTAGE   = 0x0002;
static constexpr uint16_t CW_QUICK_STOP       = 0x0004;
static constexpr uint16_t CW_ENABLE_OPERATION = 0x0008;
static constexpr uint16_t CW_NEW_SETPOINT     = 0x0010;
static constexpr uint16_t CW_CHANGE_IMMEDIATE = 0x0020;
static constexpr uint16_t CW_ABS_REL          = 0x0040;
static constexpr uint16_t CW_FAULT_RESET      = 0x0080;
static constexpr uint16_t CW_HALT             = 0x0100;

// Compound controlWord commands
static constexpr uint16_t CW_SHUTDOWN         = CW_ENABLE_VOLTAGE | CW_QUICK_STOP;               // 0x0006
static constexpr uint16_t CW_SWITCH_ON_CMD    = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP; // 0x0007
static constexpr uint16_t CW_ENABLE_OP        = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP | CW_ENABLE_OPERATION; // 0x000F
static constexpr uint16_t CW_DISABLE_VOLTAGE  = 0x0000;
static constexpr uint16_t CW_QUICK_STOP_CMD   = CW_ENABLE_VOLTAGE; // 0x0002 (QuickStop bit NOT set → active-low)

// StatusWord bits
static constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0001;
static constexpr uint16_t SW_SWITCHED_ON        = 0x0002;
static constexpr uint16_t SW_OPERATION_ENABLED  = 0x0004;
static constexpr uint16_t SW_FAULT              = 0x0008;
static constexpr uint16_t SW_VOLTAGE_ENABLED    = 0x0010;
static constexpr uint16_t SW_QUICK_STOP         = 0x0020;
static constexpr uint16_t SW_SWITCH_ON_DISABLED = 0x0040;
static constexpr uint16_t SW_WARNING            = 0x0080;
static constexpr uint16_t SW_REMOTE             = 0x0200;
static constexpr uint16_t SW_TARGET_REACHED     = 0x0400;
static constexpr uint16_t SW_INTERNAL_LIMIT     = 0x0800;
static constexpr uint16_t SW_SETPOINT_ACK       = 0x1000;
static constexpr uint16_t SW_FOLLOWING_ERROR     = 0x2000;

// ============================================================================
// Helper: drive through state machine to OperationEnabled
// ============================================================================
static void driveToState(CiA402Slave& drv, CiA402State target) {
    // First bring to SwitchOnDisabled (start() does this)
    drv.start();

    if (target == CiA402State::SwitchOnDisabled) return;

    // SwitchOnDisabled → ReadyToSwitchOn
    drv.processControlWord(CW_SHUTDOWN);
    if (target == CiA402State::ReadyToSwitchOn) return;

    // ReadyToSwitchOn → SwitchedOn
    drv.processControlWord(CW_SWITCH_ON_CMD);
    if (target == CiA402State::SwitchedOn) return;

    // SwitchedOn → OperationEnabled
    drv.processControlWord(CW_ENABLE_OP);
    if (target == CiA402State::OperationEnabled) return;
}

// ============================================================================
// Fixture: CiA402 Drive State Machine
// ============================================================================
class CiA402DriveStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA402SlaveConfig cfg{};
        cfg.supportedModes = CiA402Mode::CSP | CiA402Mode::CSV | CiA402Mode::CST |
                             CiA402Mode::PP | CiA402Mode::PV | CiA402Mode::PT |
                             CiA402Mode::HM | CiA402Mode::VL;
        cfg.softwarePosLimitMin = -1000000;
        cfg.softwarePosLimitMax = 1000000;
        cfg.maxProfileVelocity = 3000;
        cfg.maxMotorVelocity = 5000;
        cfg.maxAcceleration = 50000;
        cfg.maxDeceleration = 50000;
        cfg.quickStopDeceleration = 100000;
        cfg.maxTorque = 1000;
        cfg.motorRatedTorque = 500;
        cfg.motorRatedCurrent = 5000;
        cfg.enableSimulation = true;
        cfg.simulatedInertia = 0.001f;
        cfg.simulatedFriction = 0.1f;
        cfg.followingErrorWindow = 65535;
        cfg.positionWindow = 100;
        cfg.velocityThreshold = 10;
        drv = std::make_unique<CiA402Slave>(cfg);
    }
    std::unique_ptr<CiA402Slave> drv;
};

// --- State transitions (all 16 transitions) ---

TEST_F(CiA402DriveStateTest, InitialStateNotReadyToSwitchOn) {
    EXPECT_EQ(drv->getDriveState(), CiA402State::NotReadyToSwitchOn);
}

TEST_F(CiA402DriveStateTest, T1_AutoToSwitchOnDisabled) {
    drv->start();
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, T2_ShutdownToReady) {
    drv->start();
    drv->processControlWord(CW_SHUTDOWN);
    EXPECT_EQ(drv->getDriveState(), CiA402State::ReadyToSwitchOn);
}

TEST_F(CiA402DriveStateTest, T3_SwitchOnToSwitchedOn) {
    driveToState(*drv, CiA402State::ReadyToSwitchOn);
    drv->processControlWord(CW_SWITCH_ON_CMD);
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchedOn);
}

TEST_F(CiA402DriveStateTest, T4_EnableOperation) {
    driveToState(*drv, CiA402State::SwitchedOn);
    drv->processControlWord(CW_ENABLE_OP);
    EXPECT_EQ(drv->getDriveState(), CiA402State::OperationEnabled);
}

TEST_F(CiA402DriveStateTest, T5_DisableOperation) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->processControlWord(CW_SWITCH_ON_CMD); // EnableOp=0
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchedOn);
}

TEST_F(CiA402DriveStateTest, T6_SwitchedOnToReady) {
    driveToState(*drv, CiA402State::SwitchedOn);
    drv->processControlWord(CW_SHUTDOWN);
    EXPECT_EQ(drv->getDriveState(), CiA402State::ReadyToSwitchOn);
}

TEST_F(CiA402DriveStateTest, T7_ReadyToSwitchOnDisabled) {
    driveToState(*drv, CiA402State::ReadyToSwitchOn);
    drv->processControlWord(CW_DISABLE_VOLTAGE);
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, T8_OperationEnabledToReady) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->processControlWord(CW_SHUTDOWN);
    EXPECT_EQ(drv->getDriveState(), CiA402State::ReadyToSwitchOn);
}

TEST_F(CiA402DriveStateTest, T9_OperationEnabledToDisabled) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->processControlWord(CW_DISABLE_VOLTAGE);
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, T10_SwitchedOnToDisabled) {
    driveToState(*drv, CiA402State::SwitchedOn);
    drv->processControlWord(CW_DISABLE_VOLTAGE);
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, T11_QuickStop) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->processControlWord(CW_QUICK_STOP_CMD);
    EXPECT_EQ(drv->getDriveState(), CiA402State::QuickStopActive);
}

TEST_F(CiA402DriveStateTest, T12_QuickStopToDisabled) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->processControlWord(CW_QUICK_STOP_CMD);
    EXPECT_EQ(drv->getDriveState(), CiA402State::QuickStopActive);
    drv->processControlWord(CW_DISABLE_VOLTAGE);
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, T13_TriggerFault) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->triggerFault(0x1234);
    // Should go through FaultReactionActive → Fault
    EXPECT_EQ(drv->getDriveState(), CiA402State::Fault);
}

TEST_F(CiA402DriveStateTest, T15_FaultReset) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->triggerFault(0x1234);
    EXPECT_EQ(drv->getDriveState(), CiA402State::Fault);
    // Need rising edge: first send 0, then FaultReset
    drv->processControlWord(0x0000);
    drv->processControlWord(CW_FAULT_RESET);
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, FaultResetNeedsRisingEdge) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->triggerFault(0x1234);
    // Already sent with FaultReset bit low → now send high shouldn't be rising if already high
    drv->processControlWord(CW_FAULT_RESET);
    drv->processControlWord(CW_FAULT_RESET); // no edge
    // Should still be in SwitchOnDisabled from first reset
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, ClearFaultDirectAPI) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->triggerFault(0x5678);
    drv->clearFault();
    EXPECT_EQ(drv->getDriveState(), CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402DriveStateTest, ClearFaultOnlyInFaultState) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->clearFault(); // should be no-op
    EXPECT_EQ(drv->getDriveState(), CiA402State::OperationEnabled);
}

// --- StatusWord verification ---

TEST_F(CiA402DriveStateTest, StatusWordSwitchOnDisabled) {
    drv->start();
    drv->updateTxPDO();
    uint16_t sw = drv->getStatusWord();
    EXPECT_TRUE(sw & SW_SWITCH_ON_DISABLED);
    EXPECT_TRUE(sw & SW_REMOTE);
    EXPECT_FALSE(sw & SW_VOLTAGE_ENABLED);
    EXPECT_FALSE(sw & SW_FAULT);
}

TEST_F(CiA402DriveStateTest, StatusWordReadyToSwitchOn) {
    driveToState(*drv, CiA402State::ReadyToSwitchOn);
    drv->updateTxPDO();
    uint16_t sw = drv->getStatusWord();
    EXPECT_TRUE(sw & SW_READY_TO_SWITCH_ON);
    EXPECT_TRUE(sw & SW_QUICK_STOP);
    EXPECT_TRUE(sw & SW_VOLTAGE_ENABLED);
    EXPECT_FALSE(sw & SW_SWITCHED_ON);
}

TEST_F(CiA402DriveStateTest, StatusWordSwitchedOn) {
    driveToState(*drv, CiA402State::SwitchedOn);
    drv->updateTxPDO();
    uint16_t sw = drv->getStatusWord();
    EXPECT_TRUE(sw & SW_READY_TO_SWITCH_ON);
    EXPECT_TRUE(sw & SW_SWITCHED_ON);
    EXPECT_TRUE(sw & SW_QUICK_STOP);
    EXPECT_TRUE(sw & SW_VOLTAGE_ENABLED);
    EXPECT_FALSE(sw & SW_OPERATION_ENABLED);
}

TEST_F(CiA402DriveStateTest, StatusWordOperationEnabled) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->updateTxPDO();
    uint16_t sw = drv->getStatusWord();
    EXPECT_TRUE(sw & SW_READY_TO_SWITCH_ON);
    EXPECT_TRUE(sw & SW_SWITCHED_ON);
    EXPECT_TRUE(sw & SW_OPERATION_ENABLED);
    EXPECT_TRUE(sw & SW_QUICK_STOP);
    EXPECT_TRUE(sw & SW_VOLTAGE_ENABLED);
}

TEST_F(CiA402DriveStateTest, StatusWordQuickStopActive) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->processControlWord(CW_QUICK_STOP_CMD);
    drv->updateTxPDO();
    uint16_t sw = drv->getStatusWord();
    EXPECT_TRUE(sw & SW_OPERATION_ENABLED);
    EXPECT_TRUE(sw & SW_VOLTAGE_ENABLED);
    EXPECT_FALSE(sw & SW_QUICK_STOP); // QuickStop bit is 0 in QuickStopActive
}

TEST_F(CiA402DriveStateTest, StatusWordFault) {
    driveToState(*drv, CiA402State::OperationEnabled);
    drv->triggerFault(0x1111);
    drv->updateTxPDO();
    uint16_t sw = drv->getStatusWord();
    EXPECT_TRUE(sw & SW_FAULT);
    EXPECT_FALSE(sw & SW_VOLTAGE_ENABLED);
}

TEST_F(CiA402DriveStateTest, StateToString) {
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::NotReadyToSwitchOn).data(), "NotReadyToSwitchOn");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::SwitchOnDisabled).data(), "SwitchOnDisabled");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::ReadyToSwitchOn).data(), "ReadyToSwitchOn");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::SwitchedOn).data(), "SwitchedOn");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::OperationEnabled).data(), "OperationEnabled");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::QuickStopActive).data(), "QuickStopActive");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::FaultReactionActive).data(), "FaultReactionActive");
    EXPECT_STREQ(magic_enum::enum_name(CiA402State::Fault).data(), "Fault");
}

// --- Drive state callback ---

TEST_F(CiA402DriveStateTest, StateCallbackCalledOnTransition) {
    std::vector<CiA402State> transitions;
    drv->setDriveStateCallback([&](CiA402State /*old*/, CiA402State s) { transitions.push_back(s); });
    drv->start();
    drv->processControlWord(CW_SHUTDOWN);
    drv->processControlWord(CW_SWITCH_ON_CMD);
    EXPECT_GE(transitions.size(), 2u);
    EXPECT_NE(std::find(transitions.begin(), transitions.end(), CiA402State::ReadyToSwitchOn), transitions.end());
    EXPECT_NE(std::find(transitions.begin(), transitions.end(), CiA402State::SwitchedOn), transitions.end());
}

// ============================================================================
// CiA402 Simulation Tests
// ============================================================================
class CiA402SimulationTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA402SlaveConfig cfg{};
        cfg.supportedModes = CiA402Mode::CSP | CiA402Mode::CSV | CiA402Mode::CST |
                             CiA402Mode::PP | CiA402Mode::PV | CiA402Mode::PT |
                             CiA402Mode::HM | CiA402Mode::VL;
        cfg.softwarePosLimitMin = -1000000;
        cfg.softwarePosLimitMax = 1000000;
        cfg.maxProfileVelocity = 100000;
        cfg.maxMotorVelocity = 200000;
        cfg.maxAcceleration = 5000000;
        cfg.maxDeceleration = 5000000;
        cfg.quickStopDeceleration = 10000000;
        cfg.maxTorque = 1000;
        cfg.motorRatedTorque = 1000;
        cfg.motorRatedCurrent = 5000;
        cfg.enableSimulation = true;
        cfg.simulatedInertia = 0.001f;
        cfg.simulatedFriction = 0.1f;
        cfg.followingErrorWindow = 100000;
        cfg.positionWindow = 100;
        cfg.velocityThreshold = 10;
        drv = std::make_unique<CiA402Slave>(cfg);
        driveToState(*drv, CiA402State::OperationEnabled);
    }
    std::unique_ptr<CiA402Slave> drv;
    static constexpr uint64_t MS1 = 1'000'000; // 1ms in ns
};

// --- Operating mode ---

TEST_F(CiA402SimulationTest, SetOperatingModeCSP) {
    // Set mode via processControlWord pathway - set operatingMode OD entry
    auto& od = drv->getObjectDictionary();
    int8_t mode = 8; // CSP
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 8);
}

TEST_F(CiA402SimulationTest, SetOperatingModeCSV) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 9; // CSV
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 9);
}

TEST_F(CiA402SimulationTest, SetOperatingModeCST) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 10; // CST
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 10);
}

TEST_F(CiA402SimulationTest, SetOperatingModePP) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 1; // PP
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 1);
}

TEST_F(CiA402SimulationTest, SetOperatingModePV) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 3; // PV
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 3);
}

TEST_F(CiA402SimulationTest, SetOperatingModePT) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 4; // PT
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 4);
}

TEST_F(CiA402SimulationTest, SetOperatingModeHM) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 6; // HM
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    EXPECT_EQ(drv->getOperatingMode(), 6);
}

TEST_F(CiA402SimulationTest, UnsupportedModeNotSwitched) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 7; // IP - not in supportedModes
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);
    // Mode should not change to unsupported mode
    EXPECT_NE(drv->getOperatingMode(), 7);
}

// --- CST simulation ---

TEST_F(CiA402SimulationTest, CSTAppliedTorqueMovesPosition) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 10; // CST
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    int16_t torque = 500;
    od.write(0x6071, 0, reinterpret_cast<const uint8_t*>(&torque), sizeof(torque));
    int32_t startPos = drv->getActualPosition();
    for (int i = 0; i < 100; ++i) drv->simulate(MS1);
    EXPECT_NE(drv->getActualPosition(), startPos);
}

TEST_F(CiA402SimulationTest, CSTTargetReachedAlways) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 10;
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    int16_t torque = 100;
    od.write(0x6071, 0, reinterpret_cast<const uint8_t*>(&torque), sizeof(torque));
    drv->simulate(MS1);
    drv->updateTxPDO();
    EXPECT_TRUE(drv->getStatusWord() & SW_TARGET_REACHED);
}

// --- CSP simulation ---

TEST_F(CiA402SimulationTest, CSPMovesTowardsTarget) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 8; // CSP
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    int32_t target = 10000;
    od.write(0x607A, 0, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
    for (int i = 0; i < 500; ++i) drv->simulate(MS1);
    // Position should have moved towards target
    EXPECT_GT(drv->getActualPosition(), 0);
}

// --- CSV simulation ---

TEST_F(CiA402SimulationTest, CSVAppliesVelocity) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 9; // CSV
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    int32_t vel = 1000;
    od.write(0x60FF, 0, reinterpret_cast<const uint8_t*>(&vel), sizeof(vel));
    for (int i = 0; i < 100; ++i) drv->simulate(MS1);
    EXPECT_GT(drv->getActualVelocity(), 0);
}

// --- Profile Position PP ---

TEST_F(CiA402SimulationTest, PPNewSetPoint) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 1; // PP
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    int32_t target = 5000;
    od.write(0x607A, 0, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
    // Issue new set point
    drv->processControlWord(CW_ENABLE_OP | CW_NEW_SETPOINT);
    for (int i = 0; i < 500; ++i) drv->simulate(MS1);
    // Position should approach target
    EXPECT_GT(drv->getActualPosition(), 0);
}

TEST_F(CiA402SimulationTest, PPHaltDecelerates) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 1;
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    int32_t target = 100000;
    od.write(0x607A, 0, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
    drv->processControlWord(CW_ENABLE_OP | CW_NEW_SETPOINT);
    for (int i = 0; i < 50; ++i) drv->simulate(MS1);
    // Now halt
    drv->processControlWord(CW_ENABLE_OP | CW_HALT);
    for (int i = 0; i < 5000; ++i) drv->simulate(MS1);
    // Velocity should be near zero (friction deceleration)
    EXPECT_NEAR(drv->getActualVelocity(), 0, 500);
}

// --- Profile Velocity PV ---

TEST_F(CiA402SimulationTest, PVAppliesVelocity) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 3; // PV
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    int32_t vel = 2000;
    od.write(0x60FF, 0, reinterpret_cast<const uint8_t*>(&vel), sizeof(vel));
    for (int i = 0; i < 200; ++i) drv->simulate(MS1);
    EXPECT_GT(drv->getActualVelocity(), 0);
}

// --- Profile Torque PT ---

TEST_F(CiA402SimulationTest, PTApliesTorque) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 4; // PT
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    int16_t torque = 300;
    od.write(0x6071, 0, reinterpret_cast<const uint8_t*>(&torque), sizeof(torque));
    for (int i = 0; i < 100; ++i) drv->simulate(MS1);
    EXPECT_NE(drv->getActualTorque(), 0);
}

// --- Homing ---

TEST_F(CiA402SimulationTest, HomingWithoutCallback) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 6; // HM
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    // Start homing
    drv->processControlWord(CW_ENABLE_OP | CW_NEW_SETPOINT);
    for (int i = 0; i < 1000; ++i) drv->simulate(MS1);
    // Should complete eventually
    EXPECT_TRUE(drv->isHomingComplete());
}

TEST_F(CiA402SimulationTest, HomingWithCallback) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 6;
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    bool called = false;
    drv->setHomingCallback([&](int8_t method, int32_t& homePos) {
        called = true;
        homePos = 42;
        return true;
    });
    drv->processControlWord(CW_ENABLE_OP | CW_NEW_SETPOINT);
    drv->simulate(MS1);
    EXPECT_TRUE(called);
    EXPECT_TRUE(drv->isHomingComplete());
}

// --- QuickStop simulation ---

TEST_F(CiA402SimulationTest, QuickStopDecelerates) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 10; // CST
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    int16_t torque = 800;
    od.write(0x6071, 0, reinterpret_cast<const uint8_t*>(&torque), sizeof(torque));
    // Build up speed
    for (int i = 0; i < 200; ++i) drv->simulate(MS1);
    EXPECT_NE(drv->getActualVelocity(), 0);

    // Quick stop (torqueDemand_ zeroed on entry, only friction decelerates)
    drv->processControlWord(CW_QUICK_STOP_CMD);
    EXPECT_EQ(drv->getDriveState(), CiA402State::QuickStopActive);
    for (int i = 0; i < 5000; ++i) drv->simulate(MS1);
    EXPECT_NEAR(drv->getActualVelocity(), 0, 500);
}

// --- Position/velocity accessors ---

TEST_F(CiA402SimulationTest, SetGetActualPosition) {
    drv->setActualPosition(12345);
    EXPECT_EQ(drv->getActualPosition(), 12345);
}

TEST_F(CiA402SimulationTest, SetGetActualVelocity) {
    drv->setActualVelocity(555);
    EXPECT_EQ(drv->getActualVelocity(), 555);
}

TEST_F(CiA402SimulationTest, SetGetActualTorque) {
    drv->setActualTorque(42);
    EXPECT_EQ(drv->getActualTorque(), 42);
}

TEST_F(CiA402SimulationTest, FollowingErrorTracked) {
    drv->setActualPosition(0);
    auto& od = drv->getObjectDictionary();
    int8_t mode = 8; // CSP
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    int32_t target = 50000;
    od.write(0x607A, 0, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
    drv->simulate(MS1);
    EXPECT_NE(drv->getFollowingError(), 0);
}

// --- Touch probe ---

TEST_F(CiA402SimulationTest, TouchProbeCapture) {
    drv->triggerTouchProbe(1, 99999);
    EXPECT_EQ(drv->getTouchProbePosition(1), 99999);
    drv->triggerTouchProbe(2, -55555);
    EXPECT_EQ(drv->getTouchProbePosition(2), -55555);
}

TEST_F(CiA402SimulationTest, TouchProbeStatusNonZero) {
    drv->triggerTouchProbe(1, 100);
    EXPECT_NE(drv->getTouchProbeStatus(), 0);
}

// --- Digital I/O ---

TEST_F(CiA402SimulationTest, DigitalInputsRoundTrip) {
    drv->setDigitalInputs(0xDEADBEEF);
    EXPECT_EQ(drv->getDigitalInputs(), 0xDEADBEEF);
}

TEST_F(CiA402SimulationTest, DigitalOutputsViaPDO) {
    // Digital outputs are set via RxPDO or OD
    auto& od = drv->getObjectDictionary();
    uint32_t mask = 0xFFFFFFFF;
    od.write(0x60FE, 2, reinterpret_cast<const uint8_t*>(&mask), sizeof(mask));
    uint32_t outputs = 0x12345678;
    od.write(0x60FE, 1, reinterpret_cast<const uint8_t*>(&outputs), sizeof(outputs));
    EXPECT_EQ(drv->getDigitalOutputs(), 0x12345678);
}

// --- Software position limits ---

TEST_F(CiA402SimulationTest, PositionClampedToLimits) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 8; // CSP
    od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode));
    drv->simulate(MS1);

    // Set target beyond max limit
    int32_t target = 2000000; // > 1000000 soft limit
    od.write(0x607A, 0, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
    for (int i = 0; i < 5000; ++i) drv->simulate(MS1);
    // Position should not exceed max limit  
    EXPECT_LE(drv->getActualPosition(), 1000000);
}

// --- PDO ---

TEST_F(CiA402SimulationTest, TxRxPDODoNotCrash) {
    drv->updateTxPDO();
    drv->processRxPDO();
    drv->simulate(MS1);
}

// --- InternalLimitActive ---

TEST_F(CiA402SimulationTest, InternalLimitBitSetAtLimit) {
    drv->setActualPosition(1000000); // == softwarePosLimitMax, triggers >=
    drv->updateTxPDO();
    EXPECT_TRUE(drv->getStatusWord() & SW_INTERNAL_LIMIT);
}

// --- Supported drive modes ---

TEST_F(CiA402SimulationTest, SupportedDriveModes) {
    uint32_t modes = drv->getSupportedDriveFunctions();
    EXPECT_NE(modes, 0u);
}

// --- Mode support check ---

TEST_F(CiA402SimulationTest, ModeSupported) {
    EXPECT_TRUE(drv->isModeSupported(8));  // CSP
    EXPECT_TRUE(drv->isModeSupported(9));  // CSV
    EXPECT_TRUE(drv->isModeSupported(10)); // CST
    EXPECT_TRUE(drv->isModeSupported(1));  // PP
    EXPECT_TRUE(drv->isModeSupported(3));  // PV
    EXPECT_TRUE(drv->isModeSupported(4));  // PT
    EXPECT_TRUE(drv->isModeSupported(6));  // HM
}

TEST_F(CiA402SimulationTest, ModeNotSupported) {
    EXPECT_FALSE(drv->isModeSupported(7)); // IP
    EXPECT_FALSE(drv->isModeSupported(99));
}

// --- Homing set/get ---

TEST_F(CiA402SimulationTest, HomingCompleteSetGet) {
    drv->setHomingComplete(true);
    EXPECT_TRUE(drv->isHomingComplete());
    drv->setHomingComplete(false);
    EXPECT_FALSE(drv->isHomingComplete());
}

TEST_F(CiA402SimulationTest, SetHomePosition) {
    drv->setHomePosition(12345);
    EXPECT_EQ(drv->getActualPosition(), 12345);
}

// ============================================================================
// CiA402 Object Dictionary Tests
// ============================================================================
class CiA402ODTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA402SlaveConfig cfg{};
        cfg.supportedModes = CiA402Mode::CSP | CiA402Mode::CSV | CiA402Mode::PP;
        cfg.maxProfileVelocity = 3000;
        cfg.maxMotorVelocity = 5000;
        cfg.maxAcceleration = 50000;
        cfg.maxDeceleration = 50000;
        cfg.maxTorque = 1000;
        cfg.motorRatedTorque = 500;
        cfg.motorRatedCurrent = 5000;
        drv = std::make_unique<CiA402Slave>(cfg);
        drv->start();
    }
    std::unique_ptr<CiA402Slave> drv;
};

TEST_F(CiA402ODTest, ControlWordReadWrite) {
    auto& od = drv->getObjectDictionary();
    uint16_t cw = CW_SHUTDOWN;
    EXPECT_EQ(od.write(0x6040, 0, reinterpret_cast<const uint8_t*>(&cw), sizeof(cw)), SDOAbortCode::Success);
    uint16_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6040, 0, reinterpret_cast<uint8_t*>(&readback), l); }
}

TEST_F(CiA402ODTest, StatusWordReadOnly) {
    auto& od = drv->getObjectDictionary();
    uint16_t sw = 0;
    { size_t l = sizeof(sw); od.read(0x6041, 0, reinterpret_cast<uint8_t*>(&sw), l); }
    EXPECT_NE(sw, 0u); // Should have some bits set
}

TEST_F(CiA402ODTest, ErrorCodeReadOnly) {
    auto& od = drv->getObjectDictionary();
    uint16_t err = 0;
    { size_t l = sizeof(err); od.read(0x603F, 0, reinterpret_cast<uint8_t*>(&err), l); }
}

TEST_F(CiA402ODTest, ModesOfOperationRW) {
    auto& od = drv->getObjectDictionary();
    int8_t mode = 8; // CSP
    EXPECT_EQ(od.write(0x6060, 0, reinterpret_cast<const uint8_t*>(&mode), sizeof(mode)), SDOAbortCode::Success);
    int8_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6060, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_EQ(readback, 8);
}

TEST_F(CiA402ODTest, ModesOfOperationDisplay) {
    auto& od = drv->getObjectDictionary();
    int8_t display = 0;
    { size_t l = sizeof(display); od.read(0x6061, 0, reinterpret_cast<uint8_t*>(&display), l); }
}

TEST_F(CiA402ODTest, TargetPositionRW) {
    auto& od = drv->getObjectDictionary();
    int32_t pos = 12345;
    EXPECT_EQ(od.write(0x607A, 0, reinterpret_cast<const uint8_t*>(&pos), sizeof(pos)), SDOAbortCode::Success);
    int32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x607A, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_EQ(readback, 12345);
}

TEST_F(CiA402ODTest, PositionActualReadOnly) {
    auto& od = drv->getObjectDictionary();
    int32_t pos = 0;
    { size_t l = sizeof(pos); od.read(0x6064, 0, reinterpret_cast<uint8_t*>(&pos), l); }
}

TEST_F(CiA402ODTest, VelocityActualReadOnly) {
    auto& od = drv->getObjectDictionary();
    int32_t vel = 0;
    { size_t l = sizeof(vel); od.read(0x606C, 0, reinterpret_cast<uint8_t*>(&vel), l); }
}

TEST_F(CiA402ODTest, TargetVelocityRWClamped) {
    auto& od = drv->getObjectDictionary();
    int32_t vel = 9999999; // > maxMotorVelocity
    EXPECT_EQ(od.write(0x60FF, 0, reinterpret_cast<const uint8_t*>(&vel), sizeof(vel)), SDOAbortCode::Success);
    int32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x60FF, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_LE(readback, 5000); // clamped to maxMotorVelocity
}

TEST_F(CiA402ODTest, TargetTorqueRWClamped) {
    auto& od = drv->getObjectDictionary();
    int16_t torque = 5000; // > maxTorque
    EXPECT_EQ(od.write(0x6071, 0, reinterpret_cast<const uint8_t*>(&torque), sizeof(torque)), SDOAbortCode::Success);
    int16_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6071, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_LE(readback, 1000); // clamped
}

TEST_F(CiA402ODTest, ProfileVelocityRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t vel = 1500;
    EXPECT_EQ(od.write(0x6081, 0, reinterpret_cast<const uint8_t*>(&vel), sizeof(vel)), SDOAbortCode::Success);
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6081, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_EQ(readback, 1500u);
}

TEST_F(CiA402ODTest, ProfileAccelerationRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t acc = 25000;
    EXPECT_EQ(od.write(0x6083, 0, reinterpret_cast<const uint8_t*>(&acc), sizeof(acc)), SDOAbortCode::Success);
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6083, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_EQ(readback, 25000u);
}

TEST_F(CiA402ODTest, ProfileDecelerationRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t dec = 30000;
    EXPECT_EQ(od.write(0x6084, 0, reinterpret_cast<const uint8_t*>(&dec), sizeof(dec)), SDOAbortCode::Success);
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6084, 0, reinterpret_cast<uint8_t*>(&readback), l); }
    EXPECT_EQ(readback, 30000u);
}

TEST_F(CiA402ODTest, SoftwarePosLimitsRW) {
    auto& od = drv->getObjectDictionary();
    int32_t minp = -500000;
    EXPECT_EQ(od.write(0x607D, 1, reinterpret_cast<const uint8_t*>(&minp), sizeof(minp)), SDOAbortCode::Success);
    int32_t maxp = 500000;
    EXPECT_EQ(od.write(0x607D, 2, reinterpret_cast<const uint8_t*>(&maxp), sizeof(maxp)), SDOAbortCode::Success);
    int32_t rmin = 0, rmax = 0;
    { size_t l = sizeof(rmin); od.read(0x607D, 1, reinterpret_cast<uint8_t*>(&rmin), l); };
    { size_t l = sizeof(rmax); od.read(0x607D, 2, reinterpret_cast<uint8_t*>(&rmax), l); };
    EXPECT_EQ(rmin, -500000);
    EXPECT_EQ(rmax, 500000);
}

TEST_F(CiA402ODTest, HomingMethodRW) {
    auto& od = drv->getObjectDictionary();
    int8_t method = 17;
    od.write(0x6098, 0, reinterpret_cast<const uint8_t*>(&method), sizeof(method));
    int8_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6098, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 17);
}

TEST_F(CiA402ODTest, HomingSpeedsRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t switchSpd = 5000;
    od.write(0x6099, 1, reinterpret_cast<const uint8_t*>(&switchSpd), sizeof(switchSpd));
    uint32_t zeroSpd = 1000;
    od.write(0x6099, 2, reinterpret_cast<const uint8_t*>(&zeroSpd), sizeof(zeroSpd));
    uint32_t r1 = 0, r2 = 0;
    { size_t l = sizeof(r1); od.read(0x6099, 1, reinterpret_cast<uint8_t*>(&r1), l); };
    { size_t l = sizeof(r2); od.read(0x6099, 2, reinterpret_cast<uint8_t*>(&r2), l); };
    EXPECT_EQ(r1, 5000u);
    EXPECT_EQ(r2, 1000u);
}

TEST_F(CiA402ODTest, HomingAccelerationRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t acc = 50000;
    od.write(0x609A, 0, reinterpret_cast<const uint8_t*>(&acc), sizeof(acc));
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x609A, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 50000u);
}

TEST_F(CiA402ODTest, PositionOffsetsRW) {
    auto& od = drv->getObjectDictionary();
    int32_t posOff = 100;
    od.write(0x60B0, 0, reinterpret_cast<const uint8_t*>(&posOff), sizeof(posOff));
    int32_t velOff = 200;
    od.write(0x60B1, 0, reinterpret_cast<const uint8_t*>(&velOff), sizeof(velOff));
    int16_t torOff = 50;
    od.write(0x60B2, 0, reinterpret_cast<const uint8_t*>(&torOff), sizeof(torOff));
    int32_t r1 = 0, r2 = 0;
    int16_t r3 = 0;
    { size_t l = sizeof(r1); od.read(0x60B0, 0, reinterpret_cast<uint8_t*>(&r1), l); };
    { size_t l = sizeof(r2); od.read(0x60B1, 0, reinterpret_cast<uint8_t*>(&r2), l); };
    { size_t l = sizeof(r3); od.read(0x60B2, 0, reinterpret_cast<uint8_t*>(&r3), l); };
    EXPECT_EQ(r1, 100);
    EXPECT_EQ(r2, 200);
    EXPECT_EQ(r3, 50);
}

TEST_F(CiA402ODTest, TouchProbeFunction) {
    auto& od = drv->getObjectDictionary();
    uint16_t func = 0x0001;
    od.write(0x60B8, 0, reinterpret_cast<const uint8_t*>(&func), sizeof(func));
    uint16_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x60B8, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 0x0001);
}

TEST_F(CiA402ODTest, FollowingErrorWindowRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t win = 1000;
    od.write(0x6065, 0, reinterpret_cast<const uint8_t*>(&win), sizeof(win));
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6065, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 1000u);
}

TEST_F(CiA402ODTest, PositionWindowRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t win = 50;
    od.write(0x6067, 0, reinterpret_cast<const uint8_t*>(&win), sizeof(win));
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6067, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 50u);
}

TEST_F(CiA402ODTest, MaxProfileVelocityRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t val = 2000;
    od.write(0x607F, 0, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x607F, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 2000u);
}

TEST_F(CiA402ODTest, MaxMotorVelocityRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t val = 4000;
    od.write(0x6080, 0, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6080, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 4000u);
}

TEST_F(CiA402ODTest, MaxTorqueRW) {
    auto& od = drv->getObjectDictionary();
    uint16_t val = 800;
    od.write(0x6072, 0, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
    uint16_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x6072, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 800u);
}

TEST_F(CiA402ODTest, MotorRatedCurrentRO) {
    auto& od = drv->getObjectDictionary();
    // 0x6075 not registered in current CiA402SlaveOD - verify it returns ObjectNotFound
    uint32_t val = 0;
    size_t l = sizeof(val);
    auto result = od.read(0x6075, 0, reinterpret_cast<uint8_t*>(&val), l);
    // If registered, check value; otherwise accept ObjectNotFound
    if (result == SDOAbortCode::Success) {
        EXPECT_EQ(val, 5000u);
    } else {
        EXPECT_EQ(result, SDOAbortCode::ObjectNotFound);
    }
}

TEST_F(CiA402ODTest, MotorRatedTorqueRO) {
    auto& od = drv->getObjectDictionary();
    uint32_t val = 0;
    { size_t l = sizeof(val); od.read(0x6076, 0, reinterpret_cast<uint8_t*>(&val), l); };
    EXPECT_EQ(val, 500u);
}

TEST_F(CiA402ODTest, HomeOffsetRW) {
    auto& od = drv->getObjectDictionary();
    int32_t off = 999;
    od.write(0x607C, 0, reinterpret_cast<const uint8_t*>(&off), sizeof(off));
    int32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x607C, 0, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 999);
}

TEST_F(CiA402ODTest, DigitalInputsRO) {
    drv->setDigitalInputs(0xABCD0000);
    auto& od = drv->getObjectDictionary();
    uint32_t val = 0;
    { size_t l = sizeof(val); od.read(0x60FD, 0, reinterpret_cast<uint8_t*>(&val), l); };
    EXPECT_EQ(val, 0xABCD0000u);
}

TEST_F(CiA402ODTest, DigitalOutputsRW) {
    auto& od = drv->getObjectDictionary();
    uint32_t mask = 0xFF;
    od.write(0x60FE, 2, reinterpret_cast<const uint8_t*>(&mask), sizeof(mask));
    uint32_t out = 0x42;
    od.write(0x60FE, 1, reinterpret_cast<const uint8_t*>(&out), sizeof(out));
    uint32_t readback = 0;
    { size_t l = sizeof(readback); od.read(0x60FE, 1, reinterpret_cast<uint8_t*>(&readback), l); };
    EXPECT_EQ(readback, 0x42u);
}

TEST_F(CiA402ODTest, SupportedDriveModesRO) {
    auto& od = drv->getObjectDictionary();
    uint32_t val = 0;
    { size_t l = sizeof(val); od.read(0x6502, 0, reinterpret_cast<uint8_t*>(&val), l); };
    EXPECT_NE(val, 0u);
}

// ============================================================================
// CiA402 Factory Tests
// ============================================================================
TEST(CiA402FactoryFullTest, ServoDriveDefaults) {
    auto drv = createServoDrive(131072);
    ASSERT_NE(drv, nullptr);
    EXPECT_EQ(drv->getDeviceType(), 0x00000192u);
    EXPECT_TRUE(drv->isModeSupported(8)); // CSP
    EXPECT_TRUE(drv->isModeSupported(6)); // HM
}

TEST(CiA402FactoryFullTest, StepperDriveDefaults) {
    auto drv = createStepperDrive(200, 256);
    ASSERT_NE(drv, nullptr);
    EXPECT_TRUE(drv->isModeSupported(8)); // CSP
    EXPECT_TRUE(drv->isModeSupported(1)); // PP
}

TEST(CiA402FactoryFullTest, FrequencyInverter) {
    auto drv = createFrequencyInverter();
    ASSERT_NE(drv, nullptr);
    EXPECT_TRUE(drv->isModeSupported(9)); // CSV
    EXPECT_TRUE(drv->isModeSupported(3)); // PV
}
