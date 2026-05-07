/**
 * @file test_CiA430_CiA417.cpp
 * @brief Unit tests for CiA 430 (Power Supply) and CiA 417 (Lift Controller) slave profiles.
 */

#include "gtest/gtest.h"
#include "tether/slave/profiles/CiA430Slave.hpp"
#include "tether/slave/profiles/CiA417Slave.hpp"

using namespace EtherCAT::slave;

// ############################################################################
//  CiA 430 — Power Supply Slave
// ############################################################################

class CiA430SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA430SlaveConfig cfg;
        cfg.numberOfChannels = 2;
        cfg.channels[0].nominalVoltage = 24000; // 24V
        cfg.channels[0].nominalCurrent = 5000;  // 5A
        cfg.channels[0].ovpThreshold = 28000;
        cfg.channels[0].ocpThreshold = 5500;
        cfg.channels[0].uvpThreshold = 20000;
        cfg.channels[1].nominalVoltage = 12000;
        cfg.channels[1].nominalCurrent = 3000;
        cfg.channels[1].ocpThreshold = 3500;
        cfg.hasTemperatureSensor = true;
        cfg.maxTemperature = 70;
        psu = createCiA430Slave(cfg);
    }
    std::unique_ptr<CiA430Slave> psu;
};

// ---------- Profile info ----------

TEST_F(CiA430SlaveTest, ProfileNameAndType) {
    EXPECT_STREQ(psu->getProfileName(), "CiA 430");
    EXPECT_EQ(psu->getDeviceType(), 0x000001AEu);
}

// ---------- Factory helpers ----------

TEST(CiA430FactoryTest, SingleChannel) {
    auto p = createSingleChannelPSU(48000, 10000);
    ASSERT_NE(p, nullptr);
    EXPECT_STREQ(p->getProfileName(), "CiA 430");
}

TEST(CiA430FactoryTest, DualChannel) {
    auto p = createDualChannelPSU(24000, 5000, 12000, 3000);
    ASSERT_NE(p, nullptr);
    EXPECT_STREQ(p->getProfileName(), "CiA 430");
}

// ---------- Initial state ----------

TEST_F(CiA430SlaveTest, InitialState) {
    // All channels should be off/ready, voltage/current = 0
    EXPECT_FALSE(psu->isOutputEnabled(0));
    EXPECT_FALSE(psu->isOutputEnabled(1));
    EXPECT_EQ(psu->getActualVoltage(0), 0u);
    EXPECT_EQ(psu->getActualCurrent(0), 0u);
    EXPECT_EQ(psu->getActualPower(0), 0u);
    EXPECT_EQ(psu->getTotalPower(), 0u);
}

// ---------- Output enable/disable ----------

TEST_F(CiA430SlaveTest, EnableOutput) {
    psu->setOutputEnable(0, true);
    EXPECT_TRUE(psu->isOutputEnabled(0));
    EXPECT_FALSE(psu->isOutputEnabled(1));
}

TEST_F(CiA430SlaveTest, DisableOutput) {
    psu->setOutputEnable(0, true);
    psu->setOutputEnable(0, false);
    EXPECT_FALSE(psu->isOutputEnabled(0));
    EXPECT_EQ(psu->getActualVoltage(0), 0u);
}

TEST_F(CiA430SlaveTest, SetAllOutputs) {
    psu->setAllOutputs(true);
    EXPECT_TRUE(psu->isOutputEnabled(0));
    EXPECT_TRUE(psu->isOutputEnabled(1));
    psu->setAllOutputs(false);
    EXPECT_FALSE(psu->isOutputEnabled(0));
    EXPECT_FALSE(psu->isOutputEnabled(1));
}

// ---------- Voltage control ----------

TEST_F(CiA430SlaveTest, SetTargetVoltage) {
    psu->setTargetVoltage(0, 24000);
    EXPECT_EQ(psu->getTargetVoltage(0), 24000u);
}

TEST_F(CiA430SlaveTest, VoltageRampOnSimulate) {
    psu->setTargetVoltage(0, 24000);
    psu->setOutputEnable(0, true);
    // Simulate 100ms  — ramp rate ~1V/ms = 1000mV/ms → 100ms = 100V = 100000mV
    // Should reach target since 24V < 100V
    psu->simulate(100'000'000); // 100ms in ns
    EXPECT_GT(psu->getActualVoltage(0), 0u);
}

// ---------- Current ----------

TEST_F(CiA430SlaveTest, SetCurrentLimit) {
    psu->setCurrentLimit(0, 4000);
    EXPECT_EQ(psu->getCurrentLimit(0), 4000u);
}

TEST_F(CiA430SlaveTest, SetActualCurrent) {
    psu->setActualCurrent(0, 2500);
    EXPECT_EQ(psu->getActualCurrent(0), 2500u);
}

// ---------- Power ----------

TEST_F(CiA430SlaveTest, PowerCalculation) {
    psu->setActualVoltage(0, 24000); // 24V
    psu->setActualCurrent(0, 2000); // 2A
    // Power = (24000 * 2000) / 1000 = 48000 mW = 48W
    EXPECT_EQ(psu->getActualPower(0), 48000u);
}

TEST_F(CiA430SlaveTest, TotalPower) {
    psu->setActualVoltage(0, 24000);
    psu->setActualCurrent(0, 1000);
    psu->setActualVoltage(1, 12000);
    psu->setActualCurrent(1, 500);
    EXPECT_EQ(psu->getTotalPower(), 24000u + 6000u); // 24W + 6W
}

// ---------- Temperature ----------

TEST_F(CiA430SlaveTest, Temperature) {
    EXPECT_EQ(psu->getTemperature(), 25); // default 25°C
    psu->setTemperature(45);
    EXPECT_EQ(psu->getTemperature(), 45);
}

// ---------- Protection: OVP ----------

TEST_F(CiA430SlaveTest, OvervoltageProtection) {
    psu->setOutputEnable(0, true);
    psu->setTargetVoltage(0, 30000);  // keep target high so ramp doesn't reduce voltage
    psu->setActualVoltage(0, 30000);  // > ovpThreshold (28000)

    psu->simulate(1'000'000); // 1ms
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OVP));
    EXPECT_EQ(psu->getChannelState(0), PowerSupplyState::Fault);
}

TEST_F(CiA430SlaveTest, ClearProtection) {
    psu->setOutputEnable(0, true);
    psu->setTargetVoltage(0, 30000);
    psu->setActualVoltage(0, 30000);
    psu->simulate(1'000'000);
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OVP));

    // Clear protection
    psu->clearProtection(0);
    EXPECT_FALSE(psu->isProtectionTripped(0, ProtectionType::OVP));
    EXPECT_NE(psu->getChannelState(0), PowerSupplyState::Fault);
}

TEST_F(CiA430SlaveTest, ClearAllProtection) {
    psu->setAllOutputs(true);
    psu->setTargetVoltage(0, 30000);
    psu->setActualVoltage(0, 30000);
    psu->setActualCurrent(1, 6000); // > OCP for ch1 (5500)
    psu->simulate(1'000'000);

    psu->clearAllProtection();
    EXPECT_FALSE(psu->isProtectionTripped(0, ProtectionType::OVP));
    EXPECT_FALSE(psu->isProtectionTripped(1, ProtectionType::OCP));
}

// ---------- Protection: OCP ----------

TEST_F(CiA430SlaveTest, OvercurrentProtection) {
    psu->setOutputEnable(0, true);
    psu->setActualCurrent(0, 6000); // > ocpThreshold (5500)
    psu->simulate(1'000'000);
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OCP));
}

// ---------- Protection: OTP ----------

TEST_F(CiA430SlaveTest, OvertemperatureProtection) {
    psu->setAllOutputs(true);
    psu->setTemperature(80); // > maxTemperature (70)
    psu->simulate(1'000'000);
    // OTP should trip all channels
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OTP));
    EXPECT_TRUE(psu->isProtectionTripped(1, ProtectionType::OTP));
    EXPECT_EQ(psu->getState(), PowerSupplyState::Fault);
}

// ---------- State transitions ----------

TEST_F(CiA430SlaveTest, StateTransitions) {
    // Initially Ready or Off
    auto initialState = psu->getState();
    EXPECT_TRUE(initialState == PowerSupplyState::Off ||
                initialState == PowerSupplyState::Ready);

    // Enable → Running
    psu->setOutputEnable(0, true);
    psu->simulate(1'000'000);
    EXPECT_EQ(psu->getState(), PowerSupplyState::Running);

    // Disable → Ready
    psu->setOutputEnable(0, false);
    psu->simulate(1'000'000);
    auto afterDisable = psu->getState();
    EXPECT_TRUE(afterDisable == PowerSupplyState::Off ||
                afterDisable == PowerSupplyState::Ready);
}

// ---------- Channel callback ----------

TEST_F(CiA430SlaveTest, ChannelCallback) {
    bool called = false;
    psu->setChannelCallback([&](uint8_t ch, uint32_t& v, uint32_t& c) {
        called = true;
        v = 24000;
        c = 1000;
    });
    psu->setOutputEnable(0, true);
    psu->simulate(1'000'000);
    EXPECT_TRUE(called);
}

// ---------- PDO ----------

TEST_F(CiA430SlaveTest, TxRxPDO) {
    psu->updateTxPDO();
    psu->processRxPDO();
    // Should not crash
}

// ---------- Protection status byte ----------

TEST_F(CiA430SlaveTest, ProtectionStatusByte) {
    EXPECT_EQ(psu->getProtectionStatus(0), 0);
    psu->setOutputEnable(0, true);
    psu->setTargetVoltage(0, 30000);
    psu->setActualVoltage(0, 30000);
    psu->simulate(1'000'000);
    EXPECT_NE(psu->getProtectionStatus(0), 0);
}

// ############################################################################
//  CiA 417 — Lift Controller Slave
// ############################################################################

class CiA417SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA417SlaveConfig cfg;
        cfg.numberOfFloors = 5;
        cfg.numberOfDoors = 2;
        cfg.floorHeight = 3000;  // 3m
        cfg.nominalSpeed = 1000; // 1 m/s
        cfg.levelingSpeed = 50;
        cfg.levelingTolerance = 5;
        cfg.maxLoad = 1000;      // 1000 kg
        cfg.hasLoadSensor = true;
        cfg.hasSafetyCircuit = true;
        lift = createCiA417Slave(cfg);
    }
    std::unique_ptr<CiA417Slave> lift;
};

// ---------- Profile info ----------

TEST_F(CiA417SlaveTest, ProfileNameAndType) {
    EXPECT_STREQ(lift->getProfileName(), "CiA 417");
    EXPECT_EQ(lift->getDeviceType(), 0x000001A1u);
}

// ---------- Factory ----------

TEST(CiA417FactoryTest, CreateLiftController) {
    auto l = createLiftController(10);
    ASSERT_NE(l, nullptr);
    EXPECT_STREQ(l->getProfileName(), "CiA 417");
}

// ---------- Initial state ----------

TEST_F(CiA417SlaveTest, InitialState) {
    EXPECT_EQ(lift->getCurrentFloor(), 0u);
    EXPECT_EQ(lift->getPosition(), 0);
    EXPECT_EQ(lift->getSpeed(), 0);
    EXPECT_EQ(lift->getLiftState(), LiftState::Idle);
    EXPECT_FALSE(lift->isMoving());
    EXPECT_EQ(lift->getLoad(), 0u);
    EXPECT_FALSE(lift->isOverloaded());
    EXPECT_TRUE(lift->isSafetyCircuitOk());
    EXPECT_FALSE(lift->isEmergencyStop());
}

// ---------- Floor requests ----------

TEST_F(CiA417SlaveTest, FloorRequests) {
    lift->addFloorRequest(3);
    EXPECT_TRUE(lift->hasFloorRequest(3));
    EXPECT_EQ(lift->getFloorRequests() & (1u << 3), (1u << 3));

    lift->clearFloorRequest(3);
    EXPECT_FALSE(lift->hasFloorRequest(3));
}

TEST_F(CiA417SlaveTest, MultipleFloorRequests) {
    lift->addFloorRequest(1);
    lift->addFloorRequest(4);
    EXPECT_TRUE(lift->hasFloorRequest(1));
    EXPECT_TRUE(lift->hasFloorRequest(4));
    EXPECT_FALSE(lift->hasFloorRequest(2));
}

// ---------- Position ----------

TEST_F(CiA417SlaveTest, SetPosition) {
    lift->setPosition(6000); // 6m → floor 2
    EXPECT_EQ(lift->getPosition(), 6000);
    EXPECT_EQ(lift->getCurrentFloor(), 2u);
}

TEST_F(CiA417SlaveTest, IsAtFloor) {
    lift->setPosition(0); // Exactly at floor 0
    EXPECT_TRUE(lift->isAtFloor());

    lift->setPosition(1500); // Between floors
    EXPECT_FALSE(lift->isAtFloor());
}

// ---------- Target floor & movement ----------

TEST_F(CiA417SlaveTest, SetTargetFloor) {
    lift->setTargetFloor(3);
    EXPECT_EQ(lift->getTargetFloor(), 3u);
    // Should start moving from Idle
    EXPECT_TRUE(lift->isMoving());
    EXPECT_TRUE(lift->isMovingUp());
}

TEST_F(CiA417SlaveTest, MovingDown) {
    lift->setPosition(9000); // floor 3
    lift->setTargetFloor(1);
    EXPECT_TRUE(lift->isMovingDown());
}

TEST_F(CiA417SlaveTest, SimulateMovement) {
    lift->setTargetFloor(2);
    // Simulate enough time to reach floor 2 (6000mm at 1000mm/s = 6s)
    for (int i = 0; i < 70; i++) {
        lift->simulate(100'000'000); // 100ms steps
    }
    // Should have reached floor 2 (position ~6000)
    EXPECT_GE(lift->getPosition(), 5990);
}

// ---------- Door control ----------

TEST_F(CiA417SlaveTest, OpenDoor) {
    // Can open door when not moving
    EXPECT_FALSE(lift->isMoving());
    lift->openDoor(0);
    EXPECT_EQ(lift->getDoorState(0), DoorState::Opening);
}

TEST_F(CiA417SlaveTest, OpenDoorWhileMoving) {
    lift->setTargetFloor(3);
    EXPECT_TRUE(lift->isMoving());
    lift->openDoor(0);
    // Should NOT open while moving
    EXPECT_NE(lift->getDoorState(0), DoorState::Opening);
}

TEST_F(CiA417SlaveTest, CloseDoor) {
    lift->openDoor(0);
    lift->closeDoor(0);
    EXPECT_EQ(lift->getDoorState(0), DoorState::Closing);
}

TEST_F(CiA417SlaveTest, DoorBlocked) {
    lift->openDoor(0);
    lift->setDoorBlocked(0, true);
    lift->closeDoor(0);
    // Door should NOT close when blocked
    EXPECT_NE(lift->getDoorState(0), DoorState::Closing);
}

TEST_F(CiA417SlaveTest, MultipleDoors) {
    lift->openDoor(0);
    lift->openDoor(1);
    EXPECT_EQ(lift->getDoorState(0), DoorState::Opening);
    EXPECT_EQ(lift->getDoorState(1), DoorState::Opening);
}

// ---------- Load ----------

TEST_F(CiA417SlaveTest, LoadSensor) {
    lift->setLoad(500); // 50 kg (0.1 kg units)
    EXPECT_EQ(lift->getLoad(), 500u);
    EXPECT_FALSE(lift->isOverloaded());
}

TEST_F(CiA417SlaveTest, Overloaded) {
    lift->setLoad(10001); // > maxLoad * 10 = 10000
    EXPECT_TRUE(lift->isOverloaded());
}

// ---------- Safety ----------

TEST_F(CiA417SlaveTest, SafetyCircuit) {
    EXPECT_TRUE(lift->isSafetyCircuitOk());
    lift->setSafetyCircuit(false);
    EXPECT_FALSE(lift->isSafetyCircuitOk());
    EXPECT_EQ(lift->getLiftState(), LiftState::Emergency);
    EXPECT_EQ(lift->getSpeed(), 0);
}

TEST_F(CiA417SlaveTest, EmergencyStop) {
    lift->setEmergencyStop(true);
    EXPECT_TRUE(lift->isEmergencyStop());
    EXPECT_EQ(lift->getLiftState(), LiftState::Emergency);
    EXPECT_EQ(lift->getSpeed(), 0);
}

TEST_F(CiA417SlaveTest, EmergencyStopDuringMovement) {
    lift->setTargetFloor(4);
    EXPECT_TRUE(lift->isMoving());
    lift->setEmergencyStop(true);
    lift->simulate(1'000'000);
    EXPECT_EQ(lift->getLiftState(), LiftState::Emergency);
    EXPECT_EQ(lift->getSpeed(), 0);
}

TEST_F(CiA417SlaveTest, SafetyCircuitFailDuringMovement) {
    lift->setTargetFloor(3);
    lift->simulate(100'000'000); // Start moving
    EXPECT_TRUE(lift->isMoving());
    lift->setSafetyCircuit(false);
    lift->simulate(1'000'000);
    EXPECT_EQ(lift->getLiftState(), LiftState::Emergency);
}

// ---------- Position callback ----------

TEST_F(CiA417SlaveTest, PositionCallback) {
    bool called = false;
    lift->setPositionCallback([&](int32_t& pos, int32_t& spd) {
        called = true;
        pos = 3000; // set to floor 1
        spd = 0;
    });
    lift->simulate(1'000'000);
    EXPECT_TRUE(called);
    EXPECT_EQ(lift->getPosition(), 3000);
}

// ---------- PDO ----------

TEST_F(CiA417SlaveTest, TxRxPDO) {
    lift->updateTxPDO();
    lift->processRxPDO();
    // Should not crash
}

// ---------- Additional edge cases ----------

TEST_F(CiA417SlaveTest, TargetFloorOutOfRange) {
    // Floor >= numberOfFloors should be handled gracefully
    lift->setTargetFloor(100);
    // Should not crash; target may be clamped or ignored
    lift->simulate(1'000'000);
}

TEST_F(CiA417SlaveTest, FloorRequestOutOfRange) {
    lift->addFloorRequest(100);
    // Should not crash
    lift->simulate(1'000'000);
}

TEST_F(CiA417SlaveTest, SimulateIdleDoesNotCrash) {
    for (int i = 0; i < 100; i++) {
        lift->simulate(1'000'000);
    }
    EXPECT_EQ(lift->getLiftState(), LiftState::Idle);
}

TEST_F(CiA417SlaveTest, SimulateMovementToFloorCompletes) {
    lift->addFloorRequest(1);
    // The idle state machine should pick up the request
    // Simulate enough time for lift to reach floor 1 (3000mm at 1000mm/s = 3s)
    for (int i = 0; i < 40; i++) {
        lift->simulate(100'000'000); // 100ms steps = 4s total
    }
    // Should have processed the request
    // Position should be near 3000mm
    EXPECT_GE(lift->getPosition(), 2990);
}
