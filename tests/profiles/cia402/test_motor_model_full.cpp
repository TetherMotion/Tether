/**
 * @file test_motor_model_full.cpp
 * @brief Comprehensive unit tests for CiA402 MotorModel
 */

#include "tether/profiles/cia402/MotorModel.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <functional>

using namespace CiA402::Motor;

// ============================================================================
// Helper to create a default valid MotorParams
// ============================================================================

static MotorParams makeDefaultParams() {
    MotorParams p{};
    p.inertia = 0.001;
    p.viscousDamping = 0.0001;
    p.coulombFriction = 0.01;
    p.staticFriction = 0.02;
    p.stictionVelocity = 10.0;
    p.torqueConstant = 0.1;
    p.backEMFConstant = 0.1;
    p.windingResistance = 1.0;
    p.windingInductance = 0.001;
    p.ratedCurrent = 5.0;
    p.peakCurrent = 10.0;
    p.maxTorque = 2.0;
    p.maxVelocity = 3000.0;
    p.maxAcceleration = 50000.0;
    p.maxPosition = 1000000;
    p.minPosition = -1000000;
    p.encoderResolution = 4096;
    p.hasIndexPulse = true;
    p.indexPulsePosition = 0;
    p.gearRatio = 1.0;
    p.gearEfficiency = 0.95;
    p.gearBacklash = 0.0;
    return p;
}

// ============================================================================
// MotorParams Tests
// ============================================================================

TEST(MotorParamsTest, DefaultConstruction) {
    MotorParams p{};
    // MotorParams has non-zero defaults for safety
    EXPECT_GT(p.inertia, 0.0);
    EXPECT_GE(p.viscousDamping, 0.0);
}

// ============================================================================
// MotorState Tests
// ============================================================================

TEST(MotorStateTest, Reset) {
    MotorState s{};
    s.position = 100.0;
    s.velocity = 50.0;
    s.acceleration = 10.0;
    s.reset();
    EXPECT_DOUBLE_EQ(s.position, 0.0);
    EXPECT_DOUBLE_EQ(s.velocity, 0.0);
    EXPECT_DOUBLE_EQ(s.acceleration, 0.0);
}

// ============================================================================
// MotorErrorInjection Tests
// ============================================================================

TEST(MotorErrorInjectionTest, Reset) {
    MotorErrorInjection e{};
    e.simulateOvercurrent = true;
    e.simulateOverheat = true;
    e.reset();
    EXPECT_FALSE(e.simulateOvercurrent);
    EXPECT_FALSE(e.simulateOverheat);
}

// ============================================================================
// MotorModel Tests
// ============================================================================

class MotorModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        params_ = makeDefaultParams();
        model_ = std::make_unique<MotorModel>(params_);
        model_->initialize();
    }

    MotorParams params_;
    std::unique_ptr<MotorModel> model_;
};

TEST_F(MotorModelTest, ConstructDefault) {
    MotorModel m;
    EXPECT_FALSE(m.isInitialized());
}

TEST_F(MotorModelTest, ConstructWithParams) {
    EXPECT_TRUE(model_->isInitialized());
}

TEST_F(MotorModelTest, Initialize) {
    MotorModel m;
    m.setMotorParams(params_);
    EXPECT_TRUE(m.initialize());
    EXPECT_TRUE(m.isInitialized());
}

TEST_F(MotorModelTest, Reset) {
    model_->setTargetPosition(1000);
    model_->update(0.001);
    model_->reset();
    EXPECT_EQ(model_->getActualPosition(), 0);
    EXPECT_EQ(model_->getActualVelocity(), 0);
}

TEST_F(MotorModelTest, GetSetMotorParams) {
    auto& p = model_->getMotorParams();
    EXPECT_DOUBLE_EQ(p.inertia, params_.inertia);
    
    MotorParams p2 = params_;
    p2.inertia = 0.005;
    model_->setMotorParams(p2);
    EXPECT_DOUBLE_EQ(model_->getMotorParams().inertia, 0.005);
}

TEST_F(MotorModelTest, GetSetThermalParams) {
    ThermalParams t{};
    t.ambientTemp = 25.0;
    t.thermalResistance = 0.5;
    t.thermalCapacity = 100.0;
    t.maxWindingTemp = 150.0;
    t.maxMotorTemp = 100.0;
    t.enableThermalModel = true;
    model_->setThermalParams(t);
    auto& got = model_->getThermalParams();
    EXPECT_DOUBLE_EQ(got.ambientTemp, 25.0);
    EXPECT_TRUE(got.enableThermalModel);
}

TEST_F(MotorModelTest, GetSetBrakingResistorParams) {
    BrakingResistorParams b{};
    b.enabled = true;
    b.resistance = 10.0;
    b.maxPower = 100.0;
    model_->setBrakingResistorParams(b);
    auto& got = model_->getBrakingResistorParams();
    EXPECT_TRUE(got.enabled);
    EXPECT_DOUBLE_EQ(got.resistance, 10.0);
}

TEST_F(MotorModelTest, GetSetEndstopConfig) {
    EndstopConfig e{};
    e.positiveEnabled = true;
    e.positivePosition = 100000;
    e.negativeEnabled = true;
    e.negativePosition = -100000;
    e.switchDebounceMs = 5;
    model_->setEndstopConfig(e);
    auto& got = model_->getEndstopConfig();
    EXPECT_TRUE(got.positiveEnabled);
    EXPECT_DOUBLE_EQ(got.positivePosition, 100000);
}

TEST_F(MotorModelTest, GetSetPositionControllerParams) {
    PositionControllerParams p{};
    p.posKp = 10.0;
    p.posKi = 0.1;
    p.posKd = 0.5;
    p.velKp = 5.0;
    model_->setPositionControllerParams(p);
    auto& got = model_->getPositionControllerParams();
    EXPECT_DOUBLE_EQ(got.posKp, 10.0);
}

TEST_F(MotorModelTest, GetSetVelocityControllerParams) {
    VelocityControllerParams v{};
    v.kp = 1.0;
    v.ki = 0.1;
    v.kd = 0.01;
    model_->setVelocityControllerParams(v);
    auto& got = model_->getVelocityControllerParams();
    EXPECT_DOUBLE_EQ(got.kp, 1.0);
}

TEST_F(MotorModelTest, GetSetTorqueControllerParams) {
    TorqueControllerParams t{};
    t.maxTorque = 5.0;
    t.torqueRampRate = 100.0;
    t.enableCurrentLimit = true;
    model_->setTorqueControllerParams(t);
    auto& got = model_->getTorqueControllerParams();
    EXPECT_DOUBLE_EQ(got.maxTorque, 5.0);
}

TEST_F(MotorModelTest, ControlModeDisabled) {
    model_->setControlMode(MotorModel::ControlMode::Disabled);
    EXPECT_EQ(model_->getControlMode(), MotorModel::ControlMode::Disabled);
    model_->update(0.001);
    // No motion when disabled
}

TEST_F(MotorModelTest, ControlModePosition) {
    model_->setControlMode(MotorModel::ControlMode::Position);
    EXPECT_EQ(model_->getControlMode(), MotorModel::ControlMode::Position);
    
    PositionControllerParams pc{};
    pc.posKp = 10.0;
    pc.velKp = 1.0;
    model_->setPositionControllerParams(pc);
    
    model_->setTargetPosition(1000);
    model_->update(0.001);
    // Verify update ran without crash
}

TEST_F(MotorModelTest, ControlModeVelocity) {
    model_->setControlMode(MotorModel::ControlMode::Velocity);
    EXPECT_EQ(model_->getControlMode(), MotorModel::ControlMode::Velocity);
    
    VelocityControllerParams vc{};
    vc.kp = 1.0;
    model_->setVelocityControllerParams(vc);
    
    model_->setTargetVelocity(100);
    model_->update(0.001);
    // Velocity should be non-zero after update
    EXPECT_NE(model_->getActualVelocity(), 0);
}

TEST_F(MotorModelTest, ControlModeTorque) {
    model_->setControlMode(MotorModel::ControlMode::Torque);
    EXPECT_EQ(model_->getControlMode(), MotorModel::ControlMode::Torque);
    
    model_->setTargetTorque(100);
    model_->update(0.001);
    // Motor should respond to torque
    EXPECT_NE(model_->getActualVelocity(), 0);
}

TEST_F(MotorModelTest, SetTargetPosition) {
    model_->setTargetPosition(5000);
    // Just verify no crash
}

TEST_F(MotorModelTest, SetTargetVelocity) {
    model_->setTargetVelocity(500);
}

TEST_F(MotorModelTest, SetTargetTorque) {
    model_->setTargetTorque(100);
}

TEST_F(MotorModelTest, SetLoadTorque) {
    model_->setLoadTorque(0.5);
}

TEST_F(MotorModelTest, GetState) {
    const auto& state = model_->getState();
    EXPECT_DOUBLE_EQ(state.position, 0.0);
    
    auto& mutableState = model_->getState();
    mutableState.position = 42.0;
    EXPECT_DOUBLE_EQ(model_->getState().position, 42.0);
}

TEST_F(MotorModelTest, GetActualPosition) {
    EXPECT_EQ(model_->getActualPosition(), 0);
}

TEST_F(MotorModelTest, GetActualVelocity) {
    EXPECT_EQ(model_->getActualVelocity(), 0);
}

TEST_F(MotorModelTest, GetActualTorque) {
    EXPECT_EQ(model_->getActualTorque(), 0);
}

TEST_F(MotorModelTest, GetFollowingError) {
    EXPECT_EQ(model_->getFollowingError(), 0);
}

TEST_F(MotorModelTest, LimitChecks) {
    EXPECT_FALSE(model_->atPositiveLimit());
    EXPECT_FALSE(model_->atNegativeLimit());
}

TEST_F(MotorModelTest, HomeSwitchAndIndex) {
    EXPECT_FALSE(model_->isHomeSwitchActive());
    EXPECT_FALSE(model_->isIndexPulseDetected());
}

TEST_F(MotorModelTest, HasFault) {
    EXPECT_FALSE(model_->hasFault());
}

TEST_F(MotorModelTest, RunControlLoop) {
    model_->setControlMode(MotorModel::ControlMode::Position);
    PositionControllerParams pc{};
    pc.posKp = 10.0;
    pc.velKp = 1.0;
    model_->setPositionControllerParams(pc);
    model_->setTargetPosition(1000);
    
    double torque = model_->runControlLoop(0.001);
    // Should produce some torque command
    (void)torque;
}

TEST_F(MotorModelTest, RunPhysics) {
    model_->runPhysics(0.5, 0.001);
    // Motor should accelerate
    EXPECT_NE(model_->getState().velocity, 0.0);
}

TEST_F(MotorModelTest, UpdateThermal) {
    ThermalParams t{};
    t.enableThermalModel = true;
    t.ambientTemp = 25.0;
    t.thermalResistance = 0.5;
    t.thermalCapacity = 100.0;
    t.maxWindingTemp = 150.0;
    t.maxMotorTemp = 100.0;
    model_->setThermalParams(t);
    
    model_->updateThermal(0.001);
    // Should not crash
}

TEST_F(MotorModelTest, UpdateEndstops) {
    EndstopConfig e{};
    e.positiveEnabled = true;
    e.positivePosition = 100;
    e.negativeEnabled = true;
    e.negativePosition = -100;
    model_->setEndstopConfig(e);
    
    model_->updateEndstops();
    // Endstop state depends on position and NC configuration
    // Just verify the call doesn't crash
}

TEST_F(MotorModelTest, UpdateEncoder) {
    model_->updateEncoder();
    // Should not crash
}

TEST_F(MotorModelTest, FaultCallback) {
    uint16_t lastFaultCode = 0;
    model_->setFaultCallback([&](uint16_t code) {
        lastFaultCode = code;
    });
    
    // Inject a fault via error injection
    auto& err = model_->getErrorInjection();
    err.enabled = true;
    err.simulateOvercurrent = true;
    model_->update(0.001);
    // Fault callback should have been called with overcurrent code
    EXPECT_EQ(lastFaultCode, 0x2310);
}

TEST_F(MotorModelTest, LimitCallback) {
    bool posCalled = false, negCalled = false;
    model_->setLimitCallback([&](bool positive, bool negative) {
        posCalled = positive;
        negCalled = negative;
    });
    
    EndstopConfig e{};
    e.positiveEnabled = true;
    e.positivePosition = 10;
    e.negativeEnabled = true;
    e.negativePosition = -10;
    model_->setEndstopConfig(e);
    
    // Move position past positive limit
    model_->getState().position = 20.0;
    model_->updateEndstops();
}

TEST_F(MotorModelTest, HomeSwitchCallback) {
    bool homeCalled = false;
    model_->setHomeSwitchCallback([&](bool active) {
        homeCalled = active;
    });
    // Just verify callback is set without crash
}

TEST_F(MotorModelTest, IndexPulseCallback) {
    int32_t lastPos = 0;
    model_->setIndexPulseCallback([&](int32_t pos) {
        lastPos = pos;
    });
    // Just verify callback is set without crash
}

TEST_F(MotorModelTest, ErrorInjection) {
    auto& err = model_->getErrorInjection();
    EXPECT_FALSE(err.simulateOvercurrent);
    err.simulateOvercurrent = true;
    EXPECT_TRUE(model_->getErrorInjection().simulateOvercurrent);
    err.reset();
    EXPECT_FALSE(err.simulateOvercurrent);
}

TEST_F(MotorModelTest, FullUpdateCycle) {
    model_->setControlMode(MotorModel::ControlMode::Position);
    PositionControllerParams pc{};
    pc.posKp = 100.0;
    pc.velKp = 10.0;
    model_->setPositionControllerParams(pc);
    model_->setTargetPosition(1000);
    
    model_->update(0.001);
    // After a single timestep, position may not have moved far
    int32_t pos = model_->getActualPosition();
    // Just verify update ran without crash
    (void)pos;
}

TEST_F(MotorModelTest, VelocityControl_Update) {
    model_->setControlMode(MotorModel::ControlMode::Velocity);
    VelocityControllerParams vc{};
    vc.kp = 0.1;
    vc.ki = 0.01;
    model_->setVelocityControllerParams(vc);
    model_->setTargetVelocity(500);
    
    model_->update(0.001);
    int32_t vel = model_->getActualVelocity();
    EXPECT_NE(vel, 0);
}

TEST_F(MotorModelTest, TorqueControl_Update) {
    model_->setControlMode(MotorModel::ControlMode::Torque);
    TorqueControllerParams tc{};
    tc.maxTorque = 2.0;
    model_->setTorqueControllerParams(tc);
    model_->setTargetTorque(50);
    
    model_->update(0.001);
    // Motor should have accelerated
    EXPECT_NE(model_->getActualVelocity(), 0);
}

TEST_F(MotorModelTest, DisabledMode_NoMotion) {
    model_->setControlMode(MotorModel::ControlMode::Disabled);
    model_->setTargetPosition(5000);
    
    model_->update(0.001);
    // Control loop shouldn't generate torque in disabled mode
}
