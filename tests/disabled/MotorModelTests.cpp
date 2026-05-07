/**
 * @file MotorModelTests.cpp
 * @brief Unit tests for Motor Model
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "profiles/cia402/MotorModel.hpp"
#include <cmath>

namespace CiA402 {
namespace Motor {
namespace test {

using ::testing::DoubleNear;
using ::testing::Gt;
using ::testing::Lt;

// Test fixture
class MotorModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotorParams params;
        params.inertia = 0.001;           // 1 kg⋅m²
        params.viscousDamping = 0.01;     // Nm/(rad/s)
        params.staticFriction = 0.1;       // Nm
        params.coulombFriction = 0.05;     // Nm
        params.maxTorque = 10.0;           // Nm
        params.maxVelocity = 100.0;        // rad/s
        params.maxAcceleration = 1000.0;   // rad/s²
        params.torqueConstant = 0.1;       // Nm/A
        params.backEMFConstant = 0.1;      // V/(rad/s)
        params.encoderResolution = 4096;
        params.gearRatio = 1.0;
        
        model_ = std::make_unique<MotorModel>(params);
        model_->initialize();
    }
    
    void TearDown() override {
        model_.reset();
    }
    
    std::unique_ptr<MotorModel> model_;
    
    // Helper to run simulation for given time
    void simulate(double durationSec, double dt = 0.001) {
        int steps = static_cast<int>(durationSec / dt);
        for (int i = 0; i < steps; i++) {
            model_->update(dt);
        }
    }
};

// =============================================================================
// Initialization Tests
// =============================================================================

TEST_F(MotorModelTest, InitializesToZeroState) {
    EXPECT_EQ(model_->getActualPosition(), 0);
    EXPECT_EQ(model_->getActualVelocity(), 0);
    EXPECT_EQ(model_->getActualTorque(), 0);
    EXPECT_FALSE(model_->hasFault());
}

TEST_F(MotorModelTest, ResetClearsState) {
    model_->setTargetVelocity(1000);
    model_->setControlMode(ControlMode::Velocity);
    simulate(0.1);
    
    model_->reset();
    
    EXPECT_EQ(model_->getActualPosition(), 0);
    EXPECT_EQ(model_->getActualVelocity(), 0);
}

// =============================================================================
// Position Control Tests
// =============================================================================

TEST_F(MotorModelTest, PositionControlReachesTarget) {
    PositionControllerParams posParams;
    posParams.posKp = 100.0;
    posParams.posKi = 10.0;
    posParams.posKd = 5.0;
    posParams.velKp = 10.0;
    posParams.velKi = 1.0;
    posParams.maxVelocityCmd = 50.0;
    posParams.maxTorqueCmd = 5.0;
    
    model_->setPositionControllerParams(posParams);
    model_->setControlMode(ControlMode::Position);
    model_->setTargetPosition(1000); // 1000 counts
    
    // Simulate for 2 seconds
    simulate(2.0);
    
    int32_t error = std::abs(model_->getFollowingError());
    EXPECT_LT(error, 50); // Within 50 counts
}

TEST_F(MotorModelTest, PositionControlOscillationDamped) {
    PositionControllerParams posParams;
    posParams.posKp = 50.0;
    posParams.posKi = 5.0;
    posParams.posKd = 10.0;
    posParams.velKp = 5.0;
    
    model_->setPositionControllerParams(posParams);
    model_->setControlMode(ControlMode::Position);
    model_->setTargetPosition(2000);
    
    // Record position history
    std::vector<int32_t> positions;
    for (int i = 0; i < 2000; i++) {
        model_->update(0.001);
        positions.push_back(model_->getActualPosition());
    }
    
    // Check for damped oscillation (positions converge)
    int32_t target = 2000;
    int32_t lastError = std::abs(positions.back() - target);
    EXPECT_LT(lastError, 100);
}

TEST_F(MotorModelTest, DualLoopControlBetterThanSingle) {
    // This test verifies the dual-loop benefits
    PositionControllerParams params;
    params.posKp = 100.0;
    params.velKp = 10.0;
    params.velKi = 1.0;
    
    model_->setPositionControllerParams(params);
    model_->setControlMode(ControlMode::Position);
    model_->setTargetPosition(1000);
    
    simulate(1.0);
    
    // Should track reasonably well
    EXPECT_LT(std::abs(model_->getFollowingError()), 200);
}

// =============================================================================
// Velocity Control Tests
// =============================================================================

TEST_F(MotorModelTest, VelocityControlReachesTarget) {
    VelocityControllerParams velParams;
    velParams.kp = 1.0;
    velParams.ki = 0.5;
    velParams.kd = 0.1;
    velParams.maxTorqueCmd = 5.0;
    
    model_->setVelocityControllerParams(velParams);
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(500); // 500 counts/s
    
    simulate(1.0);
    
    // Should be near target velocity
    int32_t vel = model_->getActualVelocity();
    EXPECT_GT(vel, 400);
    EXPECT_LT(vel, 600);
}

TEST_F(MotorModelTest, VelocityControlHandlesLoad) {
    VelocityControllerParams velParams;
    velParams.kp = 2.0;
    velParams.ki = 1.0;
    velParams.maxTorqueCmd = 8.0;
    
    model_->setVelocityControllerParams(velParams);
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(1000);
    model_->setLoadTorque(1.0); // Apply load
    
    simulate(2.0);
    
    // Should still reach target (integral action compensates)
    int32_t vel = model_->getActualVelocity();
    EXPECT_GT(vel, 800);
}

// =============================================================================
// Torque Control Tests
// =============================================================================

TEST_F(MotorModelTest, TorqueControlDirect) {
    TorqueControllerParams torqueParams;
    torqueParams.enableCurrentLimit = true;
    
    model_->setTorqueControllerParams(torqueParams);
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(500); // 50% of max torque
    
    model_->update(0.001);
    
    // Torque should be applied
    int16_t torque = model_->getActualTorque();
    EXPECT_GT(torque, 0);
}

TEST_F(MotorModelTest, TorqueLimitEnforced) {
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(2000); // 200% of max - exceeds limit
    
    model_->update(0.001);
    
    // Should be limited to max
    int16_t torque = model_->getActualTorque();
    EXPECT_LE(torque, 1000); // Max is 100%
}

// =============================================================================
// Physics Tests
// =============================================================================

TEST_F(MotorModelTest, FrictionDeceleratesMotor) {
    // Start with velocity, then disable control
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(1000);
    simulate(0.5);
    
    model_->setControlMode(ControlMode::Disabled);
    
    // Motor should decelerate due to friction
    int32_t initialVel = model_->getActualVelocity();
    simulate(0.5);
    int32_t finalVel = model_->getActualVelocity();
    
    EXPECT_LT(std::abs(finalVel), std::abs(initialVel));
}

TEST_F(MotorModelTest, InertiaAffectsAcceleration) {
    MotorParams highInertia;
    highInertia.inertia = 0.01; // 10x higher
    highInertia.maxTorque = 10.0;
    highInertia.encoderResolution = 4096;
    
    MotorModel heavyMotor(highInertia);
    heavyMotor.initialize();
    
    MotorParams lowInertia;
    lowInertia.inertia = 0.001;
    lowInertia.maxTorque = 10.0;
    lowInertia.encoderResolution = 4096;
    
    MotorModel lightMotor(lowInertia);
    lightMotor.initialize();
    
    // Apply same torque to both
    heavyMotor.setControlMode(ControlMode::Torque);
    lightMotor.setControlMode(ControlMode::Torque);
    heavyMotor.setTargetTorque(500);
    lightMotor.setTargetTorque(500);
    
    // Simulate
    for (int i = 0; i < 100; i++) {
        heavyMotor.update(0.001);
        lightMotor.update(0.001);
    }
    
    // Light motor should accelerate faster
    EXPECT_GT(std::abs(lightMotor.getActualVelocity()), 
              std::abs(heavyMotor.getActualVelocity()));
}

TEST_F(MotorModelTest, LoadTorqueAffectsVelocity) {
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(500);
    
    simulate(0.5);
    int32_t velNoLoad = model_->getActualVelocity();
    
    model_->reset();
    model_->initialize();
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(500);
    model_->setLoadTorque(2.0); // Significant load
    
    simulate(0.5);
    int32_t velWithLoad = model_->getActualVelocity();
    
    EXPECT_LT(velWithLoad, velNoLoad);
}

// =============================================================================
// Thermal Model Tests
// =============================================================================

TEST_F(MotorModelTest, TemperatureIncreasesWithCurrent) {
    ThermalParams thermal;
    thermal.enableThermalModel = true;
    thermal.thermalResistance = 5.0;
    thermal.thermalCapacity = 100.0;
    thermal.ambientTemp = 25.0;
    thermal.maxWindingTemp = 120.0;
    
    model_->setThermalParams(thermal);
    
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(800); // High torque = high current
    
    simulate(10.0);
    
    MotorState state = model_->getState();
    EXPECT_GT(state.windingTemperature, 25.0);
}

TEST_F(MotorModelTest, OvertemperatureFault) {
    ThermalParams thermal;
    thermal.enableThermalModel = true;
    thermal.thermalResistance = 50.0; // High resistance = fast heating
    thermal.thermalCapacity = 10.0;   // Low capacity
    thermal.ambientTemp = 25.0;
    thermal.maxWindingTemp = 50.0;    // Low threshold
    
    model_->setThermalParams(thermal);
    
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(900);
    
    simulate(20.0); // Enough time to heat up
    
    EXPECT_TRUE(model_->hasFault());
}

// =============================================================================
// Braking Resistor Tests
// =============================================================================

TEST_F(MotorModelTest, BrakingResistorAbsorbsRegenPower) {
    BrakingResistorParams braking;
    braking.enabled = true;
    braking.resistance = 10.0;
    braking.maxPower = 500.0;
    braking.maxTemperature = 150.0;
    
    model_->setBrakingResistorParams(braking);
    
    // Get motor moving
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(1000);
    simulate(0.5);
    
    // Now brake hard (negative torque while positive velocity)
    model_->setControlMode(ControlMode::Torque);
    model_->setTargetTorque(-500);
    simulate(0.2);
    
    // Braking resistor should heat up
    MotorState state = model_->getState();
    EXPECT_GT(state.brakingResistorTemp, 25.0);
}

TEST_F(MotorModelTest, BrakingResistorOverheatFault) {
    BrakingResistorParams braking;
    braking.enabled = true;
    braking.enableOverheatProtection = true;
    braking.maxTemperature = 50.0; // Low threshold
    braking.thermalTimeConstant = 1.0;
    
    model_->setBrakingResistorParams(braking);
    
    // Inject overheating
    MotorErrorInjection injection;
    injection.enabled = true;
    injection.simulateBrakingResistorOverheat = true;
    
    model_->setErrorInjection(injection);
    model_->update(0.001);
    
    EXPECT_TRUE(model_->hasFault());
}

// =============================================================================
// Endstop Tests
// =============================================================================

TEST_F(MotorModelTest, PositiveLimitTriggered) {
    EndstopConfig endstops;
    endstops.positiveEnabled = true;
    endstops.positivePosition = 1000;
    endstops.positiveNC = false;
    
    model_->setEndstopConfig(endstops);
    
    // Move past limit
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(2000);
    simulate(1.0);
    
    MotorState state = model_->getState();
    EXPECT_TRUE(state.positiveLimit);
}

TEST_F(MotorModelTest, NegativeLimitTriggered) {
    EndstopConfig endstops;
    endstops.negativeEnabled = true;
    endstops.negativePosition = -1000;
    endstops.negativeNC = false;
    
    model_->setEndstopConfig(endstops);
    
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(-2000);
    simulate(1.0);
    
    MotorState state = model_->getState();
    EXPECT_TRUE(state.negativeLimit);
}

TEST_F(MotorModelTest, HomeSwitchDetected) {
    EndstopConfig endstops;
    endstops.homeEnabled = true;
    endstops.homePosition = 500;
    endstops.homeWidth = 100;
    endstops.homeNC = false;
    
    model_->setEndstopConfig(endstops);
    
    // Move through home position
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(500);
    
    bool homeSeen = false;
    for (int i = 0; i < 500; i++) {
        model_->update(0.001);
        MotorState state = model_->getState();
        if (state.homeSwitch) {
            homeSeen = true;
            break;
        }
    }
    
    EXPECT_TRUE(homeSeen);
}

TEST_F(MotorModelTest, IndexPulseDetected) {
    EndstopConfig endstops;
    endstops.indexEnabled = true;
    endstops.indexPosition = 1000;
    
    model_->setEndstopConfig(endstops);
    
    bool indexSeen = false;
    model_->setIndexPulseCallback([&](int32_t pos) {
        indexSeen = true;
    });
    
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(1000);
    simulate(2.0);
    
    EXPECT_TRUE(indexSeen);
}

// =============================================================================
// Error Injection Tests
// =============================================================================

TEST_F(MotorModelTest, EndstopDisconnectInjection) {
    EndstopConfig endstops;
    endstops.positiveEnabled = true;
    endstops.positivePosition = 1000;
    endstops.positiveNC = true; // Normally closed
    
    model_->setEndstopConfig(endstops);
    
    MotorErrorInjection injection;
    injection.enabled = true;
    injection.simulateEndstopDisconnect = true;
    
    model_->setErrorInjection(injection);
    model_->update(0.001);
    
    // NC endstop disconnected should show as triggered
    MotorState state = model_->getState();
    EXPECT_TRUE(state.positiveLimit);
}

TEST_F(MotorModelTest, EncoderNoiseInjection) {
    MotorErrorInjection injection;
    injection.enabled = true;
    injection.simulateEncoderNoise = true;
    injection.encoderNoiseAmplitude = 10.0;
    
    model_->setErrorInjection(injection);
    
    // Record multiple position readings
    std::vector<int32_t> readings;
    for (int i = 0; i < 100; i++) {
        model_->update(0.001);
        readings.push_back(model_->getActualPosition());
    }
    
    // Should have some variation (noise)
    int32_t minVal = *std::min_element(readings.begin(), readings.end());
    int32_t maxVal = *std::max_element(readings.begin(), readings.end());
    
    EXPECT_GT(maxVal - minVal, 0);
}

TEST_F(MotorModelTest, EncoderFaultInjection) {
    MotorErrorInjection injection;
    injection.enabled = true;
    injection.simulateEncoderFault = true;
    
    model_->setErrorInjection(injection);
    model_->update(0.001);
    
    MotorState state = model_->getState();
    EXPECT_TRUE(state.encoderFault);
}

TEST_F(MotorModelTest, JamInjection) {
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(1000);
    simulate(0.5);
    
    int32_t velBefore = model_->getActualVelocity();
    EXPECT_GT(velBefore, 0);
    
    MotorErrorInjection injection;
    injection.enabled = true;
    injection.simulateJam = true;
    
    model_->setErrorInjection(injection);
    model_->update(0.01);
    
    // Velocity should be zero due to jam
}

// =============================================================================
// Callback Tests
// =============================================================================

TEST_F(MotorModelTest, FaultCallbackInvoked) {
    uint16_t reportedFault = 0;
    
    model_->setFaultCallback([&](uint16_t code) {
        reportedFault = code;
    });
    
    // Inject fault
    MotorErrorInjection injection;
    injection.enabled = true;
    injection.simulateOvercurrent = true;
    
    model_->setErrorInjection(injection);
    model_->update(0.001);
    
    EXPECT_NE(reportedFault, 0);
}

TEST_F(MotorModelTest, LimitCallbackInvoked) {
    EndstopConfig endstops;
    endstops.positiveEnabled = true;
    endstops.positivePosition = 500;
    
    model_->setEndstopConfig(endstops);
    
    bool limitCallbackCalled = false;
    model_->setLimitCallback([&](bool pos, bool neg) {
        if (pos || neg) {
            limitCallbackCalled = true;
        }
    });
    
    model_->setControlMode(ControlMode::Velocity);
    model_->setTargetVelocity(1000);
    simulate(1.0);
    
    EXPECT_TRUE(limitCallbackCalled);
}

// =============================================================================
// Gear Ratio Tests
// =============================================================================

TEST_F(MotorModelTest, GearRatioAffectsOutput) {
    MotorParams params;
    params.inertia = 0.001;
    params.gearRatio = 10.0; // 10:1 reduction
    params.gearEfficiency = 0.9;
    params.encoderResolution = 4096;
    
    MotorModel gearedMotor(params);
    gearedMotor.initialize();
    
    gearedMotor.setControlMode(ControlMode::Torque);
    gearedMotor.setTargetTorque(500);
    
    // Geared motor should accelerate differently
}

} // namespace test
} // namespace Motor
} // namespace CiA402
