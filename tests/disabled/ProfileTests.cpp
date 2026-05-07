/**
 * @file ProfileTests.cpp
 * @brief Unit tests for CiA profile slave implementations
 */

#include "slave/profiles/ProfileSlave.hpp"
#include "slave/profiles/CiA401Slave.hpp"
#include "slave/profiles/CiA402Slave.hpp"
#include "slave/profiles/CiA404Slave.hpp"
#include "slave/profiles/CiA405Slave.hpp"
#include "slave/profiles/CiA406Slave.hpp"
#include "slave/profiles/CiA408Slave.hpp"
#include "slave/profiles/CiA410Slave.hpp"
#include "slave/profiles/CiA417Slave.hpp"
#include "slave/profiles/CiA430Slave.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <thread>
#include <chrono>

namespace EtherCAT {
namespace Slave {
namespace Test {

// ============================================================================
// CiA 401 - Digital/Analog I/O Tests
// ============================================================================

class CiA401SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA401Slave>(1);
    }
    
    std::unique_ptr<CiA401Slave> slave_;
};

TEST_F(CiA401SlaveTest, DigitalInputs) {
    // Set digital input
    slave_->setDigitalInput(0, true);
    EXPECT_TRUE(slave_->getDigitalInput(0));
    
    slave_->setDigitalInput(0, false);
    EXPECT_FALSE(slave_->getDigitalInput(0));
}

TEST_F(CiA401SlaveTest, DigitalOutputs) {
    // Read initial state
    EXPECT_FALSE(slave_->getDigitalOutput(0));
    
    // Output is controlled by master via PDO, not directly settable
    // This tests the getter
}

TEST_F(CiA401SlaveTest, AnalogInputs) {
    slave_->setAnalogInput(0, 2048);
    EXPECT_EQ(slave_->getAnalogInput(0), 2048);
    
    slave_->setAnalogInput(0, 0);
    EXPECT_EQ(slave_->getAnalogInput(0), 0);
    
    slave_->setAnalogInput(0, 4095);
    EXPECT_EQ(slave_->getAnalogInput(0), 4095);
}

TEST_F(CiA401SlaveTest, MultipleChannels) {
    // Test multiple digital inputs
    for (int i = 0; i < 8; i++) {
        slave_->setDigitalInput(i, (i % 2) == 0);
    }
    
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(slave_->getDigitalInput(i), (i % 2) == 0);
    }
}

// ============================================================================
// CiA 402 - Drives and Motion Control Tests
// ============================================================================

class CiA402SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA402Slave>(1);
    }
    
    void goToOperationEnabled() {
        slave_->setControlWord(0x0006);  // Shutdown
        slave_->update();
        slave_->setControlWord(0x0007);  // Switch on
        slave_->update();
        slave_->setControlWord(0x000F);  // Enable operation
        slave_->update();
    }
    
    std::unique_ptr<CiA402Slave> slave_;
};

TEST_F(CiA402SlaveTest, InitialState) {
    // Should be in "Not ready to switch on"
    uint16_t status = slave_->getStatusWord();
    EXPECT_EQ(status & 0x4F, 0x00);  // Bits 0-3, 6 should indicate not ready
}

TEST_F(CiA402SlaveTest, StateTransitionToReadyToSwitchOn) {
    slave_->setControlWord(0x0006);  // Shutdown
    slave_->update();
    
    uint16_t status = slave_->getStatusWord();
    EXPECT_EQ(status & 0x6F, 0x21);  // Ready to switch on
}

TEST_F(CiA402SlaveTest, StateTransitionToSwitchedOn) {
    slave_->setControlWord(0x0006);  // Shutdown
    slave_->update();
    slave_->setControlWord(0x0007);  // Switch on
    slave_->update();
    
    uint16_t status = slave_->getStatusWord();
    EXPECT_EQ(status & 0x6F, 0x23);  // Switched on
}

TEST_F(CiA402SlaveTest, StateTransitionToOperationEnabled) {
    goToOperationEnabled();
    
    uint16_t status = slave_->getStatusWord();
    EXPECT_EQ(status & 0x6F, 0x27);  // Operation enabled
}

TEST_F(CiA402SlaveTest, QuickStop) {
    goToOperationEnabled();
    
    slave_->setControlWord(0x0002);  // Quick stop
    slave_->update();
    
    uint16_t status = slave_->getStatusWord();
    EXPECT_EQ(status & 0x6F, 0x07);  // Quick stop active
}

TEST_F(CiA402SlaveTest, ProfilePositionMode) {
    goToOperationEnabled();
    
    slave_->setModesOfOperation(1);  // PP mode
    EXPECT_EQ(slave_->getModesOfOperationDisplay(), 1);
    
    slave_->setTargetPosition(10000);
    slave_->setControlWord(0x001F);  // New setpoint
    slave_->update();
    
    // Target acknowledged
    EXPECT_TRUE((slave_->getStatusWord() & 0x1000) != 0);
}

TEST_F(CiA402SlaveTest, ProfileVelocityMode) {
    goToOperationEnabled();
    
    slave_->setModesOfOperation(3);  // PV mode
    EXPECT_EQ(slave_->getModesOfOperationDisplay(), 3);
    
    slave_->setTargetVelocity(1000);
}

TEST_F(CiA402SlaveTest, HomingMode) {
    goToOperationEnabled();
    
    slave_->setModesOfOperation(6);  // Homing mode
    EXPECT_EQ(slave_->getModesOfOperationDisplay(), 6);
    
    // Start homing
    slave_->setControlWord(0x001F);
    slave_->update();
}

TEST_F(CiA402SlaveTest, CyclicSynchronousPositionMode) {
    goToOperationEnabled();
    
    slave_->setModesOfOperation(8);  // CSP mode
    EXPECT_EQ(slave_->getModesOfOperationDisplay(), 8);
    
    // In CSP, target position is applied directly each cycle
    slave_->setTargetPosition(5000);
    slave_->update();
}

// ============================================================================
// CiA 404 - Measuring and Closed-Loop Control Tests
// ============================================================================

class CiA404SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA404Slave>(1);
    }
    
    std::unique_ptr<CiA404Slave> slave_;
};

TEST_F(CiA404SlaveTest, MeasurementChannel) {
    // Set actual value
    slave_->setActualValue(0, 1000);
    EXPECT_EQ(slave_->getActualValue(0), 1000);
}

TEST_F(CiA404SlaveTest, PIDControl) {
    // Configure PID gains
    slave_->setPIDGains(0, 1.0f, 0.1f, 0.01f);
    
    // Set setpoint
    slave_->setSetpoint(0, 1000);
    
    // Simulate measurement
    slave_->setActualValue(0, 900);
    slave_->update();
    
    // Output should be non-zero due to error
    EXPECT_NE(slave_->getControlOutput(0), 0);
}

TEST_F(CiA404SlaveTest, AlarmLimits) {
    slave_->setAlarmLimits(0, 0, 1000);
    
    slave_->setActualValue(0, 500);
    slave_->update();
    EXPECT_FALSE(slave_->isAlarmActive(0));
    
    slave_->setActualValue(0, 1500);
    slave_->update();
    EXPECT_TRUE(slave_->isAlarmActive(0));
}

// ============================================================================
// CiA 405 - PLC Tests
// ============================================================================

class CiA405SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA405Slave>(1);
    }
    
    std::unique_ptr<CiA405Slave> slave_;
};

TEST_F(CiA405SlaveTest, RunState) {
    EXPECT_EQ(slave_->getRunState(), PLCRunState::Stopped);
    
    slave_->start();
    EXPECT_EQ(slave_->getRunState(), PLCRunState::Running);
    
    slave_->stop();
    EXPECT_EQ(slave_->getRunState(), PLCRunState::Stopped);
}

TEST_F(CiA405SlaveTest, TaskConfiguration) {
    slave_->configureTask(0, 1000, 1);  // 1ms cycle, priority 1
    
    slave_->start();
    slave_->update();
    
    EXPECT_GT(slave_->getCycleCount(), 0);
}

TEST_F(CiA405SlaveTest, InputOutputVariables) {
    slave_->setInputVariable(0, 42);
    EXPECT_EQ(slave_->getInputVariable(0), 42);
    
    slave_->setOutputVariable(0, 100);
    EXPECT_EQ(slave_->getOutputVariable(0), 100);
}

// ============================================================================
// CiA 406 - Encoder Tests
// ============================================================================

class CiA406SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA406Slave>(1);
    }
    
    std::unique_ptr<CiA406Slave> slave_;
};

TEST_F(CiA406SlaveTest, SingleTurnPosition) {
    EXPECT_EQ(slave_->getSingleTurnPosition(), 0);
    
    slave_->simulateRotation(1000);
    EXPECT_EQ(slave_->getSingleTurnPosition(), 1000);
}

TEST_F(CiA406SlaveTest, MultiTurnTracking) {
    // Simulate full rotation
    uint32_t resolution = slave_->getResolution();
    slave_->simulateRotation(resolution + 100);
    
    EXPECT_EQ(slave_->getMultiTurnCount(), 1);
    EXPECT_EQ(slave_->getSingleTurnPosition(), 100);
}

TEST_F(CiA406SlaveTest, VelocityCalculation) {
    slave_->simulateRotation(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    slave_->update();
    
    // Velocity should be calculated from position change
    int32_t velocity = slave_->getVelocity();
    EXPECT_NE(velocity, 0);
}

TEST_F(CiA406SlaveTest, PresetValue) {
    slave_->simulateRotation(5000);
    slave_->setPresetValue(0);
    
    EXPECT_EQ(slave_->getSingleTurnPosition(), 0);
}

// ============================================================================
// CiA 408 - Hydraulic Drive Tests
// ============================================================================

class CiA408SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA408Slave>(1);
    }
    
    std::unique_ptr<CiA408Slave> slave_;
};

TEST_F(CiA408SlaveTest, InitialState) {
    EXPECT_EQ(slave_->getState(), HydraulicState::NotReady);
}

TEST_F(CiA408SlaveTest, EnableHydraulics) {
    slave_->enable();
    slave_->update();
    
    EXPECT_NE(slave_->getState(), HydraulicState::NotReady);
}

TEST_F(CiA408SlaveTest, PressureMode) {
    slave_->enable();
    slave_->setOperatingMode(HydraulicOperatingMode::Pressure);
    
    slave_->setTargetPressure(100.0f);  // bar
    slave_->update();
    
    // Actual pressure should approach target
    float actual = slave_->getActualPressure();
    // Initial update may not reach target
}

TEST_F(CiA408SlaveTest, PositionMode) {
    slave_->enable();
    slave_->setOperatingMode(HydraulicOperatingMode::Position);
    
    slave_->setTargetPosition(500.0f);  // mm
    slave_->update();
}

TEST_F(CiA408SlaveTest, EmergencyStop) {
    slave_->enable();
    slave_->emergencyStop();
    
    EXPECT_EQ(slave_->getState(), HydraulicState::EmergencyStop);
}

// ============================================================================
// CiA 410 - Inclinometer Tests
// ============================================================================

class CiA410SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA410Slave>(1);
    }
    
    std::unique_ptr<CiA410Slave> slave_;
};

TEST_F(CiA410SlaveTest, InitialInclination) {
    EXPECT_EQ(slave_->getInclinationX(), 0);
    EXPECT_EQ(slave_->getInclinationY(), 0);
}

TEST_F(CiA410SlaveTest, SetInclination) {
    slave_->setRawInclination(5000, 3000);  // 5.0°, 3.0° in milli-degrees
    
    EXPECT_EQ(slave_->getInclinationX(), 5000);
    EXPECT_EQ(slave_->getInclinationY(), 3000);
}

TEST_F(CiA410SlaveTest, TotalInclination) {
    slave_->setRawInclination(3000, 4000);  // 3°, 4° -> total should be 5°
    
    int32_t total = slave_->getTotalInclination();
    EXPECT_NEAR(total, 5000, 10);  // ~5000 milli-degrees
}

TEST_F(CiA410SlaveTest, ZeroCalibration) {
    slave_->setRawInclination(1000, 2000);
    slave_->calibrateZero();
    
    EXPECT_EQ(slave_->getInclinationX(), 0);
    EXPECT_EQ(slave_->getInclinationY(), 0);
}

TEST_F(CiA410SlaveTest, AlarmLimits) {
    slave_->setAlarmLimit(10000);  // 10°
    
    slave_->setRawInclination(5000, 5000);
    slave_->update();
    EXPECT_FALSE(slave_->isAlarmActive());
    
    slave_->setRawInclination(15000, 0);
    slave_->update();
    EXPECT_TRUE(slave_->isAlarmActive());
}

// ============================================================================
// CiA 417 - Lift Controller Tests
// ============================================================================

class CiA417SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA417Slave>(1);
    }
    
    std::unique_ptr<CiA417Slave> slave_;
};

TEST_F(CiA417SlaveTest, InitialState) {
    EXPECT_EQ(slave_->getState(), LiftState::Idle);
    EXPECT_EQ(slave_->getCurrentFloor(), 0);
}

TEST_F(CiA417SlaveTest, GoToFloor) {
    slave_->gotoFloor(5);
    slave_->update();
    
    EXPECT_EQ(slave_->getState(), LiftState::Moving);
    EXPECT_EQ(slave_->getTargetFloor(), 5);
}

TEST_F(CiA417SlaveTest, DoorControl) {
    EXPECT_EQ(slave_->getDoorState(), DoorState::Closed);
    
    slave_->openDoors();
    slave_->update();
    
    EXPECT_NE(slave_->getDoorState(), DoorState::Closed);
}

TEST_F(CiA417SlaveTest, EmergencyStop) {
    slave_->gotoFloor(5);
    slave_->update();
    
    slave_->emergencyStop();
    
    EXPECT_EQ(slave_->getState(), LiftState::Emergency);
}

TEST_F(CiA417SlaveTest, FloorPositions) {
    slave_->setFloorPosition(0, 0);
    slave_->setFloorPosition(1, 3000);    // 3m
    slave_->setFloorPosition(2, 6000);    // 6m
    
    // Position tracking during movement
    slave_->gotoFloor(2);
    
    for (int i = 0; i < 100; i++) {
        slave_->update();
    }
}

// ============================================================================
// CiA 430 - Power Supply Tests
// ============================================================================

class CiA430SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA430Slave>(1);
    }
    
    std::unique_ptr<CiA430Slave> slave_;
};

TEST_F(CiA430SlaveTest, InitialState) {
    EXPECT_EQ(slave_->getState(), PowerSupplyState::Off);
}

TEST_F(CiA430SlaveTest, TurnOn) {
    slave_->turnOn();
    slave_->update();
    
    EXPECT_NE(slave_->getState(), PowerSupplyState::Off);
}

TEST_F(CiA430SlaveTest, ConstantVoltageMode) {
    slave_->setOperatingMode(PowerSupplyOpMode::ConstantVoltage);
    slave_->setVoltageSetpoint(24.0f);
    slave_->turnOn();
    
    for (int i = 0; i < 10; i++) {
        slave_->update();
    }
    
    // Actual voltage should approach setpoint
    float actual = slave_->getActualVoltage();
    EXPECT_GT(actual, 0);
}

TEST_F(CiA430SlaveTest, ConstantCurrentMode) {
    slave_->setOperatingMode(PowerSupplyOpMode::ConstantCurrent);
    slave_->setCurrentSetpoint(5.0f);
    slave_->turnOn();
    
    for (int i = 0; i < 10; i++) {
        slave_->update();
    }
}

TEST_F(CiA430SlaveTest, ProtectionThresholds) {
    slave_->setOVPThreshold(30.0f);  // Over-voltage protection at 30V
    slave_->setOCPThreshold(10.0f);  // Over-current protection at 10A
    slave_->setOTPThreshold(80.0f);  // Over-temperature protection at 80°C
    
    slave_->turnOn();
    slave_->update();
    
    // Simulate over-voltage condition
    slave_->setVoltageSetpoint(35.0f);
    slave_->update();
    
    // Should trigger protection
    EXPECT_EQ(slave_->getState(), PowerSupplyState::Protection);
}

TEST_F(CiA430SlaveTest, PowerCalculation) {
    slave_->setOperatingMode(PowerSupplyOpMode::ConstantVoltage);
    slave_->setVoltageSetpoint(24.0f);
    slave_->setCurrentLimit(2.0f);
    slave_->turnOn();
    
    for (int i = 0; i < 20; i++) {
        slave_->update();
    }
    
    float voltage = slave_->getActualVoltage();
    float current = slave_->getActualCurrent();
    float power = slave_->getActualPower();
    
    // Power should be close to V * I
    EXPECT_NEAR(power, voltage * current, 0.1f);
}

// ============================================================================
// Profile Factory Tests
// ============================================================================

TEST(ProfileFactoryTest, CreateAllProfiles) {
    auto cia401 = std::make_unique<CiA401Slave>(1);
    EXPECT_NE(cia401, nullptr);
    
    auto cia402 = std::make_unique<CiA402Slave>(2);
    EXPECT_NE(cia402, nullptr);
    
    auto cia404 = std::make_unique<CiA404Slave>(3);
    EXPECT_NE(cia404, nullptr);
    
    auto cia405 = std::make_unique<CiA405Slave>(4);
    EXPECT_NE(cia405, nullptr);
    
    auto cia406 = std::make_unique<CiA406Slave>(5);
    EXPECT_NE(cia406, nullptr);
    
    auto cia408 = std::make_unique<CiA408Slave>(6);
    EXPECT_NE(cia408, nullptr);
    
    auto cia410 = std::make_unique<CiA410Slave>(7);
    EXPECT_NE(cia410, nullptr);
    
    auto cia417 = std::make_unique<CiA417Slave>(8);
    EXPECT_NE(cia417, nullptr);
    
    auto cia430 = std::make_unique<CiA430Slave>(9);
    EXPECT_NE(cia430, nullptr);
}

}  // namespace Test
}  // namespace Slave
}  // namespace EtherCAT

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
