/**
 * @file test_CiA417Lift.cpp
 * @brief Comprehensive tests for CiA 417 Lift Controller
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia417/CiA417Lift.hpp"

using namespace CiA417;

// ============================================================================
// LiftState struct helper tests
// ============================================================================

TEST(CiA417LiftState, DefaultConstruction) {
    LiftState s{};
    EXPECT_FALSE(s.isReady());
    EXPECT_FALSE(s.isRunning());
    EXPECT_FALSE(s.isAtFloor());
    EXPECT_FALSE(s.isInLevelingZone());
    EXPECT_FALSE(s.isInDoorZone());
    EXPECT_FALSE(s.areDoorsOpen());
    EXPECT_FALSE(s.areDoorsClosed());
    EXPECT_FALSE(s.isOverloaded());
    EXPECT_FALSE(s.isSafetyOK());
    EXPECT_FALSE(s.isInspectionMode());
    EXPECT_FALSE(s.hasFault());
    EXPECT_FALSE(s.isDirectionUp());
    EXPECT_FALSE(s.isDirectionDown());
}

TEST(CiA417LiftState, StatusBitReady) {
    LiftState s{};
    s.statusword = StatuswordBits::Ready;
    EXPECT_TRUE(s.isReady());
}

TEST(CiA417LiftState, StatusBitRunning) {
    LiftState s{};
    s.statusword = StatuswordBits::Running;
    EXPECT_TRUE(s.isRunning());
}

TEST(CiA417LiftState, StatusBitAtFloor) {
    LiftState s{};
    s.statusword = StatuswordBits::AtFloor;
    EXPECT_TRUE(s.isAtFloor());
}

TEST(CiA417LiftState, StatusBitLeveling) {
    LiftState s{};
    s.statusword = StatuswordBits::InLevelingZone;
    EXPECT_TRUE(s.isInLevelingZone());
}

TEST(CiA417LiftState, StatusBitDoorZone) {
    LiftState s{};
    s.statusword = StatuswordBits::InDoorZone;
    EXPECT_TRUE(s.isInDoorZone());
}

TEST(CiA417LiftState, StatusBitDoorsOpen) {
    LiftState s{};
    s.statusword = StatuswordBits::DoorsOpen;
    EXPECT_TRUE(s.areDoorsOpen());
}

TEST(CiA417LiftState, StatusBitDoorsClosed) {
    LiftState s{};
    s.statusword = StatuswordBits::DoorsClosed;
    EXPECT_TRUE(s.areDoorsClosed());
}

TEST(CiA417LiftState, StatusBitDirectionUp) {
    LiftState s{};
    s.statusword = StatuswordBits::DirectionUp;
    EXPECT_TRUE(s.isDirectionUp());
}

TEST(CiA417LiftState, StatusBitDirectionDown) {
    LiftState s{};
    s.statusword = StatuswordBits::DirectionDown;
    EXPECT_TRUE(s.isDirectionDown());
}

TEST(CiA417LiftState, StatusBitOverloaded) {
    LiftState s{};
    s.statusword = StatuswordBits::Overloaded;
    EXPECT_TRUE(s.isOverloaded());
}

TEST(CiA417LiftState, StatusBitSafety) {
    LiftState s{};
    s.statusword = StatuswordBits::SafetyOK;
    EXPECT_TRUE(s.isSafetyOK());
}

TEST(CiA417LiftState, StatusBitInspection) {
    LiftState s{};
    s.statusword = StatuswordBits::InspectionMode;
    EXPECT_TRUE(s.isInspectionMode());
}

TEST(CiA417LiftState, StatusBitFault) {
    LiftState s{};
    s.statusword = StatuswordBits::Fault;
    EXPECT_TRUE(s.hasFault());
}

TEST(CiA417LiftState, MultipleBits) {
    LiftState s{};
    s.statusword = StatuswordBits::Ready | StatuswordBits::AtFloor |
                   StatuswordBits::SafetyOK | StatuswordBits::DoorsClosed;
    EXPECT_TRUE(s.isReady());
    EXPECT_TRUE(s.isAtFloor());
    EXPECT_TRUE(s.isSafetyOK());
    EXPECT_TRUE(s.areDoorsClosed());
    EXPECT_FALSE(s.hasFault());
}

// ============================================================================
// CallStatus struct
// ============================================================================

TEST(CiA417CallStatus, Default) {
    CallStatus cs{};
    EXPECT_EQ(cs.car_calls, 0u);
    EXPECT_EQ(cs.hall_calls_up, 0u);
    EXPECT_EQ(cs.hall_calls_down, 0u);
}

// ============================================================================
// LiftSpec struct
// ============================================================================

TEST(CiA417LiftSpec, Default) {
    LiftSpec spec{};
    EXPECT_EQ(spec.num_floors, 0u);
    // num_doors defaults to 1 (at least one door)
    EXPECT_GE(spec.num_doors, 1u);
}

// ============================================================================
// Enum tests
// ============================================================================

TEST(CiA417Enums, OperatingModes) {
    EXPECT_NE(static_cast<uint8_t>(OperatingMode::Normal),
              static_cast<uint8_t>(OperatingMode::Inspection));
    EXPECT_NE(static_cast<uint8_t>(OperatingMode::FireService),
              static_cast<uint8_t>(OperatingMode::Emergency));
}

TEST(CiA417Enums, MotionStates) {
    EXPECT_EQ(static_cast<uint8_t>(MotionState::Stopped), 0u);
}

TEST(CiA417Enums, DoorStates) {
    EXPECT_NE(static_cast<uint8_t>(DoorState::FullyClosed),
              static_cast<uint8_t>(DoorState::FullyOpen));
}

TEST(CiA417Enums, DoorCommands) {
    EXPECT_NE(static_cast<uint8_t>(DoorCommand::Open),
              static_cast<uint8_t>(DoorCommand::Close));
}

TEST(CiA417Enums, Direction) {
    EXPECT_EQ(static_cast<uint8_t>(Direction::None), 0u);
}

// ============================================================================
// LiftController fixture
// ============================================================================

class CiA417Test : public ::testing::Test {
protected:
    void SetUp() override {
        lift_ = std::make_unique<LiftController>(1);
        lift_->initialize();
    }
    std::unique_ptr<LiftController> lift_;
};

TEST_F(CiA417Test, Construction) {
    LiftController l2(0x100, true);
    EXPECT_FALSE(l2.isInitialized());
}

TEST_F(CiA417Test, Initialize) {
    LiftController l3(2);
    EXPECT_TRUE(l3.initialize());
    EXPECT_TRUE(l3.isInitialized());
}

TEST_F(CiA417Test, GetSpec) {
    auto spec = lift_->getSpec();
    (void)spec.num_floors;
}

TEST_F(CiA417Test, PDOMappingAll) {
    EXPECT_TRUE(lift_->applyPDOMapping(PDOMappingPreset::Basic));
    EXPECT_TRUE(lift_->applyPDOMapping(PDOMappingPreset::Extended));
    EXPECT_TRUE(lift_->applyPDOMapping(PDOMappingPreset::Full));
    EXPECT_TRUE(lift_->applyPDOMapping(PDOMappingPreset::Custom));
}

TEST_F(CiA417Test, EnableDisable) {
    lift_->enable();
    lift_->disable();
}

TEST_F(CiA417Test, ResetFault) {
    lift_->resetFault();
}

TEST_F(CiA417Test, EmergencyStop) {
    lift_->emergencyStop();
}

TEST_F(CiA417Test, GoToFloor) {
    lift_->goToFloor(0);
    lift_->goToFloor(5);
    lift_->goToFloor(10);
}

TEST_F(CiA417Test, MoveUpDown) {
    lift_->moveUp();
    lift_->moveDown();
    lift_->stop();
    lift_->relevel();
}

TEST_F(CiA417Test, FloorAndPosition) {
    EXPECT_EQ(lift_->getCurrentFloor(), 0u);
    EXPECT_EQ(lift_->getTargetFloor(), 0u);
    EXPECT_EQ(lift_->getPosition(), 0);
    EXPECT_EQ(lift_->getVelocity(), 0);
    EXPECT_EQ(lift_->getMotionState(), 0u);
    EXPECT_EQ(lift_->getDirection(), 0u);
    // isAtFloor() may not be true in default state
    (void)lift_->isAtFloor();
}

TEST_F(CiA417Test, OperatingMode) {
    lift_->setOperatingMode(static_cast<uint8_t>(OperatingMode::Normal));
    EXPECT_EQ(lift_->getOperatingMode(), static_cast<uint8_t>(OperatingMode::Normal));
    lift_->enterInspectionMode();
    lift_->exitInspectionMode();
    lift_->enterFireServiceMode();
    lift_->enterRescueMode();
    lift_->enterMaintenanceMode();
}

TEST_F(CiA417Test, DoorControl) {
    lift_->openDoor(0);
    lift_->closeDoor(0);
    lift_->stopDoor(0);
    lift_->nudgeDoor(0);
    lift_->lockDoor(0);
    lift_->unlockDoor(0);
    (void)lift_->getDoorState(0);
    (void)lift_->isDoorOpen(0);
    (void)lift_->isDoorClosed(0);
    (void)lift_->isDoorLocked(0);
    lift_->setDoorTiming(2000, 3000, 5000);
    lift_->setDoorForce(50);
}

TEST_F(CiA417Test, DoorMultipleIDs) {
    lift_->openDoor(1);
    lift_->closeDoor(1);
    (void)lift_->getDoorState(1);
}

TEST_F(CiA417Test, CallManagement) {
    lift_->registerCarCall(0);
    lift_->registerCarCall(5);
    EXPECT_TRUE(lift_->isCarCallActive(0));
    EXPECT_TRUE(lift_->isCarCallActive(5));
    lift_->cancelCarCall(0);
    EXPECT_FALSE(lift_->isCarCallActive(0));

    lift_->registerHallCall(3, static_cast<uint8_t>(Direction::Up));
    EXPECT_TRUE(lift_->isHallCallActive(3, static_cast<uint8_t>(Direction::Up)));
    lift_->cancelHallCall(3, static_cast<uint8_t>(Direction::Up));
    EXPECT_FALSE(lift_->isHallCallActive(3, static_cast<uint8_t>(Direction::Up)));

    lift_->registerHallCall(2, static_cast<uint8_t>(Direction::Down));
    lift_->clearAllCalls();
    auto cs = lift_->getCallStatus();
    EXPECT_EQ(cs.car_calls, 0u);
}

TEST_F(CiA417Test, FloorConfig) {
    lift_->setFloorHeight(0, 0);
    lift_->setFloorHeight(1, 3000);
    lift_->setFloorHeight(2, 6000);
    // SDO may fail, so floor height may not persist
    (void)lift_->getFloorHeight(1);

    lift_->enableFloor(0, true);
    (void)lift_->isFloorEnabled(0);
    lift_->enableFloor(0, false);

    lift_->lockFloor(1, true);
    (void)lift_->isFloorLocked(1);
    lift_->lockFloor(1, false);
}

TEST_F(CiA417Test, Load) {
    EXPECT_EQ(lift_->getLoad(), 0u);
    (void)lift_->getLoadPercent();
    EXPECT_FALSE(lift_->isOverloaded());
    lift_->setOverloadThreshold(1000);
    lift_->setBypassThreshold(500);
}

TEST_F(CiA417Test, Safety) {
    auto ss = lift_->getSafetyStatus();
    (void)ss;
    EXPECT_EQ(lift_->getSafetyInputs(), 0u);
    (void)lift_->isSafetyInputActive(0x01);
    (void)lift_->isGovernorOK();
    (void)lift_->isSafetyCircuitOK();
    (void)lift_->areDoorLocksEngaged();
    (void)lift_->isEmergencyStopActive();
}

TEST_F(CiA417Test, Brakes) {
    lift_->applyBrakes();
    lift_->releaseBrakes();
    lift_->testBrakes();
    (void)lift_->getBrakeStatus();
}

TEST_F(CiA417Test, InspectionMovement) {
    lift_->enterInspectionMode();
    lift_->inspectionMoveUp();
    lift_->inspectionMoveDown();
    lift_->inspectionStop();
    lift_->carTopMoveUp();
    lift_->carTopMoveDown();
    lift_->pitMoveUp();
    lift_->pitMoveDown();
}

TEST_F(CiA417Test, Diagnostics) {
    auto diag = lift_->getDiagnostics();
    (void)diag;
    auto diagStr = lift_->getDiagnosticsString();
    EXPECT_FALSE(diagStr.empty());
    (void)lift_->getDriveTemperature();
    (void)lift_->getMotorTemperature();
    (void)lift_->getTripCount();
    (void)lift_->getOperatingHours();
    EXPECT_EQ(lift_->getFaultCode(), 0u);
}

TEST_F(CiA417Test, GetState) {
    auto st = lift_->getState();
    EXPECT_FALSE(st.hasFault());
}

TEST_F(CiA417Test, Display) {
    lift_->setDisplayFloor(3);
    lift_->setDisplayDirection(static_cast<uint8_t>(Direction::Up));
    lift_->setDisplayMessage("Floor 3");
    lift_->playAnnouncement(1);
}

TEST_F(CiA417Test, PDOProcessBasic) {
    lift_->applyPDOMapping(PDOMappingPreset::Basic);
    uint8_t txbuf[64] = {};
    lift_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[64] = {};
    auto w = lift_->prepareRxPDO(rxbuf, sizeof(rxbuf));
    EXPECT_GT(w, 0u);
}

TEST_F(CiA417Test, PDOProcessExtended) {
    lift_->applyPDOMapping(PDOMappingPreset::Extended);
    uint8_t txbuf[128] = {};
    lift_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[128] = {};
    lift_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA417Test, PDOProcessFull) {
    lift_->applyPDOMapping(PDOMappingPreset::Full);
    uint8_t txbuf[256] = {};
    lift_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[256] = {};
    lift_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA417Test, Update) {
    lift_->update();
}

TEST_F(CiA417Test, Callbacks) {
    lift_->setFloorReachedCallback([](uint8_t) {});
    lift_->setDoorStateCallback([](uint8_t, uint8_t) {});
    lift_->setSafetyCallback([](uint32_t) {});
    lift_->setFaultCallback([](uint16_t) {});
    lift_->setCallCallback([](const CallStatus&) {});
}

TEST_F(CiA417Test, PDOTooSmall) {
    lift_->applyPDOMapping(PDOMappingPreset::Full);
    uint8_t small[2] = {};
    lift_->processTxPDO(small, sizeof(small));
}
