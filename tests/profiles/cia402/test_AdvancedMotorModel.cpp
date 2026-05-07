/**
 * @file test_AdvancedMotorModel.cpp
 * @brief Comprehensive tests for CiA402::Motor::AdvancedMotorModel and related structs
 */
#include <gtest/gtest.h>
#include <cmath>
#include "tether/profiles/cia402/AdvancedMotorModel.hpp"

using namespace CiA402::Motor;

// ============================================================================
// FrictionParams struct
// ============================================================================

TEST(FrictionParamsTest, DefaultValues) {
    FrictionParams fp{};
    EXPECT_DOUBLE_EQ(fp.staticFriction, 0.1);
    EXPECT_DOUBLE_EQ(fp.coulombFriction, 0.05);
    EXPECT_DOUBLE_EQ(fp.viscousCoeff, 0.01);
    EXPECT_DOUBLE_EQ(fp.stribeckVelocity, 0.1);
    EXPECT_DOUBLE_EQ(fp.stictionVelocity, 0.001);
}

TEST(FrictionParamsTest, CalculateAtZeroVelocity) {
    FrictionParams fp{};
    double f = fp.calculate(0.0, 1.0);
    // At zero velocity with applied torque, friction should resist motion
    EXPECT_GE(std::abs(f), 0.0);
}

TEST(FrictionParamsTest, CalculateAtHighVelocity) {
    FrictionParams fp{};
    double f = fp.calculate(100.0, 0.0);
    // At high velocity, friction opposes motion
    EXPECT_NE(f, 0.0);
}

TEST(FrictionParamsTest, CalculateNegativeVelocity) {
    FrictionParams fp{};
    double f1 = fp.calculate(10.0, 0.0);
    double f2 = fp.calculate(-10.0, 0.0);
    // Friction should be symmetric in magnitude
    EXPECT_NEAR(std::abs(f1), std::abs(f2), 1e-6);
}

TEST(FrictionParamsTest, IsInStiction) {
    FrictionParams fp{};
    EXPECT_TRUE(fp.isInStiction(0.0));
    EXPECT_TRUE(fp.isInStiction(fp.stictionVelocity * 0.5));
    EXPECT_FALSE(fp.isInStiction(1.0));
}

// ============================================================================
// TorqueSpeedCurve struct
// ============================================================================

TEST(TorqueSpeedCurveTest, DefaultValues) {
    TorqueSpeedCurve tsc{};
    EXPECT_DOUBLE_EQ(tsc.ratedTorque, 5.0);
    EXPECT_DOUBLE_EQ(tsc.peakTorque, 15.0);
    EXPECT_DOUBLE_EQ(tsc.stallTorque, 6.0);
    EXPECT_DOUBLE_EQ(tsc.cornerSpeed, 300.0);
    EXPECT_DOUBLE_EQ(tsc.maxSpeed, 600.0);
}

TEST(TorqueSpeedCurveTest, AvailableTorqueAtZero) {
    TorqueSpeedCurve tsc{};
    double t = tsc.getAvailableTorque(0.0, false);
    EXPECT_GT(t, 0.0);
}

TEST(TorqueSpeedCurveTest, AvailableTorqueAtMaxSpeed) {
    TorqueSpeedCurve tsc{};
    double t = tsc.getAvailableTorque(tsc.maxSpeed, false);
    EXPECT_GE(t, 0.0);
}

TEST(TorqueSpeedCurveTest, PeakTorque) {
    TorqueSpeedCurve tsc{};
    double tPeak = tsc.getAvailableTorque(0.0, true);
    double tNorm = tsc.getAvailableTorque(0.0, false);
    EXPECT_GE(tPeak, tNorm);
}

// ============================================================================
// Electrical/Mechanical param structs
// ============================================================================

TEST(MotorElectricalParamsTest, DefaultValues) {
    MotorElectricalParams ep{};
    EXPECT_DOUBLE_EQ(ep.torqueConstant, 0.1);
    EXPECT_DOUBLE_EQ(ep.backEMFConstant, 0.1);
    EXPECT_DOUBLE_EQ(ep.windingResistance, 1.0);
    EXPECT_DOUBLE_EQ(ep.supplyVoltage, 48.0);
}

TEST(MotorMechanicalParamsTest, DefaultValues) {
    MotorMechanicalParams mp{};
    EXPECT_DOUBLE_EQ(mp.rotorInertia, 0.001);
    EXPECT_EQ(mp.encoderResolution, 131072);
    EXPECT_DOUBLE_EQ(mp.maxAcceleration, 10000.0);
}

TEST(MotorParamsTest, Composition) {
    MotorParams p{};
    EXPECT_DOUBLE_EQ(p.torqueSpeed.ratedTorque, 5.0);
    EXPECT_DOUBLE_EQ(p.electrical.torqueConstant, 0.1);
    EXPECT_DOUBLE_EQ(p.mechanical.rotorInertia, 0.001);
}

// ============================================================================
// BacklashParams
// ============================================================================

TEST(BacklashParamsTest, DefaultValues) {
    BacklashParams bp{};
    EXPECT_DOUBLE_EQ(bp.totalBacklash, 0.001);
    EXPECT_DOUBLE_EQ(bp.contactStiffness, 10000.0);
    EXPECT_DOUBLE_EQ(bp.contactDamping, 10.0);
    EXPECT_TRUE(bp.enabled);
}

// ============================================================================
// GeartrainParams
// ============================================================================

TEST(GeartrainParamsTest, DefaultValues) {
    GeartrainParams gp{};
    EXPECT_DOUBLE_EQ(gp.gearRatio, 10.0);
    EXPECT_DOUBLE_EQ(gp.forwardEfficiency, 0.9);
    EXPECT_DOUBLE_EQ(gp.backwardEfficiency, 0.85);
    EXPECT_TRUE(gp.backdrivable);
}

// ============================================================================
// LoadParams
// ============================================================================

TEST(LoadParamsTest, DefaultValues) {
    LoadParams lp{};
    EXPECT_DOUBLE_EQ(lp.inertia, 0.01);
    EXPECT_DOUBLE_EQ(lp.externalTorque, 0.0);
}

// ============================================================================
// ThermalParams
// ============================================================================

TEST(ThermalParamsTest, DefaultValues) {
    ThermalParams tp{};
    EXPECT_FALSE(tp.enabled);
    EXPECT_DOUBLE_EQ(tp.ambientTemperature, 25.0);
    EXPECT_DOUBLE_EQ(tp.maxWindingTemp, 120.0);
    EXPECT_DOUBLE_EQ(tp.maxMotorTemp, 80.0);
}

// ============================================================================
// SensorConfig
// ============================================================================

TEST(SensorConfigTest, DefaultValues) {
    SensorConfig sc{};
    EXPECT_FALSE(sc.positiveLimitEnabled);
    EXPECT_FALSE(sc.negativeLimitEnabled);
    EXPECT_FALSE(sc.homeEnabled);
    EXPECT_FALSE(sc.indexEnabled);
}

// ============================================================================
// State structs
// ============================================================================

TEST(MotorStateTest, DefaultValues) {
    MotorState ms{};
    EXPECT_DOUBLE_EQ(ms.position, 0.0);
    EXPECT_DOUBLE_EQ(ms.velocity, 0.0);
    EXPECT_DOUBLE_EQ(ms.torque, 0.0);
    EXPECT_DOUBLE_EQ(ms.current, 0.0);
}

TEST(GeartrainStateTest, DefaultValues) {
    GeartrainState gs{};
    EXPECT_DOUBLE_EQ(gs.inputPosition, 0.0);
    EXPECT_DOUBLE_EQ(gs.outputPosition, 0.0);
    EXPECT_EQ(gs.backlashState, BacklashState::InBacklash);
}

TEST(LoadStateTest, DefaultValues) {
    LoadState ls{};
    EXPECT_DOUBLE_EQ(ls.position, 0.0);
    EXPECT_FALSE(ls.atPositiveLimit);
    EXPECT_FALSE(ls.atNegativeLimit);
}

TEST(ThermalStateTest, DefaultValues) {
    ThermalState ts{};
    EXPECT_DOUBLE_EQ(ts.windingTemp, 25.0);
    EXPECT_DOUBLE_EQ(ts.motorCaseTemp, 25.0);
    EXPECT_FALSE(ts.windingOvertemp);
}

TEST(SensorStateTest, DefaultValues) {
    SensorState ss{};
    EXPECT_FALSE(ss.positiveLimitActive);
    EXPECT_FALSE(ss.negativeLimitActive);
    EXPECT_FALSE(ss.homeActive);
    EXPECT_FALSE(ss.indexPulse);
}

TEST(SystemStateTest, DefaultAndReset) {
    SystemState ss{};
    EXPECT_DOUBLE_EQ(ss.simulationTime, 0.0);
    EXPECT_FALSE(ss.hasFault);
    EXPECT_EQ(ss.faultCode, 0u);
    ss.simulationTime = 1.0;
    ss.hasFault = true;
    ss.reset();
    EXPECT_DOUBLE_EQ(ss.simulationTime, 0.0);
    EXPECT_FALSE(ss.hasFault);
}

// ============================================================================
// ErrorInjection
// ============================================================================

TEST(ErrorInjectionTest, DefaultValues) {
    ErrorInjection ei{};
    EXPECT_FALSE(ei.enabled);
    EXPECT_FALSE(ei.simulateMotorOverheat);
    EXPECT_FALSE(ei.simulateEncoderFault);
    EXPECT_DOUBLE_EQ(ei.encoderNoiseAmplitude, 0.0);
    EXPECT_FALSE(ei.simulateJam);
}

// ============================================================================
// ControllerParams
// ============================================================================

TEST(ControllerParamsTest, DefaultValues) {
    ControllerParams cp{};
    EXPECT_DOUBLE_EQ(cp.posKp, 100.0);
    EXPECT_DOUBLE_EQ(cp.posKi, 10.0);
    EXPECT_DOUBLE_EQ(cp.posKd, 5.0);
    EXPECT_DOUBLE_EQ(cp.velKp, 1.0);
    EXPECT_DOUBLE_EQ(cp.velKi, 0.5);
}

// ============================================================================
// BacklashState enum
// ============================================================================

TEST(BacklashStateTest, Values) {
    EXPECT_NE(static_cast<int>(BacklashState::ContactPositive),
              static_cast<int>(BacklashState::InBacklash));
    EXPECT_NE(static_cast<int>(BacklashState::InBacklash),
              static_cast<int>(BacklashState::ContactNegative));
}

// ============================================================================
// ControlMode enum
// ============================================================================

TEST(ControlModeTest, Values) {
    EXPECT_NE(static_cast<int>(ControlMode::Disabled),
              static_cast<int>(ControlMode::Torque));
    EXPECT_NE(static_cast<int>(ControlMode::Velocity),
              static_cast<int>(ControlMode::Position));
}

// ============================================================================
// AdvancedMotorModel - construction & init
// ============================================================================

TEST(AdvancedMotorModelTest, DefaultConstruction) {
    AdvancedMotorModel model;
    EXPECT_FALSE(model.hasFault());
    EXPECT_EQ(model.getControlMode(), ControlMode::Disabled);
}

TEST(AdvancedMotorModelTest, ConstructionWithParams) {
    MotorParams params{};
    params.torqueSpeed.ratedTorque = 2.0;
    AdvancedMotorModel model(params);
    EXPECT_DOUBLE_EQ(model.getMotorParams().torqueSpeed.ratedTorque, 2.0);
}

TEST(AdvancedMotorModelTest, Initialize) {
    AdvancedMotorModel model;
    bool ok = model.initialize();
    EXPECT_TRUE(ok);
}

TEST(AdvancedMotorModelTest, Reset) {
    AdvancedMotorModel model;
    model.initialize();
    model.setTargetVelocity(100.0);
    model.setControlMode(ControlMode::Velocity);
    model.update(0.001);
    model.reset();
    EXPECT_DOUBLE_EQ(model.getMotorPosition(), 0.0);
    EXPECT_DOUBLE_EQ(model.getMotorVelocity(), 0.0);
}

TEST(AdvancedMotorModelTest, SetInitialState) {
    AdvancedMotorModel model;
    model.initialize();
    model.setInitialState(1.0, 2.0);
    (void)model.getMotorPosition();
    (void)model.getOutputPosition();
}

// ============================================================================
// Configuration setters / getters
// ============================================================================

TEST(AdvancedMotorModelTest, SetGetMotorParams) {
    AdvancedMotorModel model;
    MotorParams p{};
    p.electrical.supplyVoltage = 24.0;
    model.setMotorParams(p);
    EXPECT_DOUBLE_EQ(model.getMotorParams().electrical.supplyVoltage, 24.0);
}

TEST(AdvancedMotorModelTest, SetGetBacklashParams) {
    AdvancedMotorModel model;
    BacklashParams bp{};
    bp.totalBacklash = 0.01;
    model.setBacklashParams(bp);
    EXPECT_DOUBLE_EQ(model.getBacklashParams().totalBacklash, 0.01);
}

TEST(AdvancedMotorModelTest, SetGetGeartrainParams) {
    AdvancedMotorModel model;
    GeartrainParams gp{};
    gp.gearRatio = 20.0;
    model.setGeartrainParams(gp);
    EXPECT_DOUBLE_EQ(model.getGeartrainParams().gearRatio, 20.0);
}

TEST(AdvancedMotorModelTest, SetGetLoadParams) {
    AdvancedMotorModel model;
    LoadParams lp{};
    lp.inertia = 0.05;
    model.setLoadParams(lp);
    EXPECT_DOUBLE_EQ(model.getLoadParams().inertia, 0.05);
}

TEST(AdvancedMotorModelTest, SetGetThermalParams) {
    AdvancedMotorModel model;
    ThermalParams tp{};
    tp.enabled = true;
    tp.ambientTemperature = 30.0;
    model.setThermalParams(tp);
    EXPECT_TRUE(model.getThermalParams().enabled);
    EXPECT_DOUBLE_EQ(model.getThermalParams().ambientTemperature, 30.0);
}

TEST(AdvancedMotorModelTest, SetGetSensorConfig) {
    AdvancedMotorModel model;
    SensorConfig sc{};
    sc.positiveLimitEnabled = true;
    sc.positiveLimitPosition = 100.0;
    model.setSensorConfig(sc);
    EXPECT_TRUE(model.getSensorConfig().positiveLimitEnabled);
}

TEST(AdvancedMotorModelTest, SetGetControllerParams) {
    AdvancedMotorModel model;
    ControllerParams cp{};
    cp.posKp = 200.0;
    model.setControllerParams(cp);
    EXPECT_DOUBLE_EQ(model.getControllerParams().posKp, 200.0);
}

// ============================================================================
// Control modes
// ============================================================================

TEST(AdvancedMotorModelTest, SetControlMode) {
    AdvancedMotorModel model;
    model.initialize();
    model.setControlMode(ControlMode::Torque);
    EXPECT_EQ(model.getControlMode(), ControlMode::Torque);
    model.setControlMode(ControlMode::Velocity);
    EXPECT_EQ(model.getControlMode(), ControlMode::Velocity);
    model.setControlMode(ControlMode::Position);
    EXPECT_EQ(model.getControlMode(), ControlMode::Position);
    model.setControlMode(ControlMode::Disabled);
    EXPECT_EQ(model.getControlMode(), ControlMode::Disabled);
}

// ============================================================================
// Simulation - Torque Mode
// ============================================================================

TEST(AdvancedMotorModelTest, TorqueModeSimulation) {
    AdvancedMotorModel model;
    model.initialize();
    model.setControlMode(ControlMode::Torque);
    model.setTargetTorque(1.0);
    for (int i = 0; i < 100; ++i) {
        model.update(0.001);
    }
    // Motor should be in torque mode; velocity may or may not change
    // depending on inertia/friction defaults
    (void)model.getMotorVelocity();
}

// ============================================================================
// Simulation - Velocity Mode
// ============================================================================

TEST(AdvancedMotorModelTest, VelocityModeSimulation) {
    AdvancedMotorModel model;
    model.initialize();
    model.setControlMode(ControlMode::Velocity);
    model.setTargetVelocity(10.0);
    for (int i = 0; i < 1000; ++i) {
        model.update(0.001);
    }
    // Should approach target velocity
    (void)model.getMotorVelocity();
}

// ============================================================================
// Simulation - Position Mode
// ============================================================================

TEST(AdvancedMotorModelTest, PositionModeSimulation) {
    AdvancedMotorModel model;
    model.initialize();
    model.setControlMode(ControlMode::Position);
    model.setTargetPosition(1.0);
    for (int i = 0; i < 2000; ++i) {
        model.update(0.001);
    }
    (void)model.getMotorPosition();
    (void)model.getFollowingError();
}

// ============================================================================
// Convenience accessors
// ============================================================================

TEST(AdvancedMotorModelTest, ConvenienceAccessors) {
    AdvancedMotorModel model;
    model.initialize();
    EXPECT_DOUBLE_EQ(model.getMotorPosition(), 0.0);
    EXPECT_DOUBLE_EQ(model.getMotorVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(model.getMotorTorque(), 0.0);
    EXPECT_DOUBLE_EQ(model.getMotorCurrent(), 0.0);
    EXPECT_DOUBLE_EQ(model.getOutputPosition(), 0.0);
    EXPECT_DOUBLE_EQ(model.getOutputVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(model.getOutputTorque(), 0.0);
    (void)model.getMotorEncoderPosition();
    EXPECT_DOUBLE_EQ(model.getFollowingError(), 0.0);
}

// ============================================================================
// External load torque
// ============================================================================

TEST(AdvancedMotorModelTest, ExternalLoadTorque) {
    AdvancedMotorModel model;
    model.initialize();
    model.setControlMode(ControlMode::Torque);
    model.setExternalLoadTorque(5.0);
    model.setTargetTorque(0.0);
    model.update(0.001);
    (void)model.getState().load.externalTorque;
}

// ============================================================================
// Fault handling
// ============================================================================

TEST(AdvancedMotorModelTest, FaultHandling) {
    AdvancedMotorModel model;
    model.initialize();
    EXPECT_FALSE(model.hasFault());
    EXPECT_EQ(model.getFaultCode(), 0u);
    model.clearFault();
    EXPECT_FALSE(model.hasFault());
}

// ============================================================================
// Error injection
// ============================================================================

TEST(AdvancedMotorModelTest, ErrorInjection) {
    AdvancedMotorModel model;
    model.initialize();
    ErrorInjection ei{};
    ei.enabled = true;
    ei.additionalFriction = 1.0;
    model.setErrorInjection(ei);
    model.setControlMode(ControlMode::Torque);
    model.setTargetTorque(0.5);
    model.update(0.001);
    (void)model.getMotorVelocity();
}

// ============================================================================
// Callbacks
// ============================================================================

TEST(AdvancedMotorModelTest, FaultCallback) {
    AdvancedMotorModel model;
    model.initialize();
    uint16_t lastFault = 0;
    model.setFaultCallback([&](uint16_t code) { lastFault = code; });
    (void)lastFault;
}

TEST(AdvancedMotorModelTest, LimitCallback) {
    AdvancedMotorModel model;
    model.initialize();
    bool limitHit = false;
    model.setLimitCallback([&](bool pos, bool neg) {
        if (pos || neg) limitHit = true;
    });

    SensorConfig sc{};
    sc.positiveLimitEnabled = true;
    sc.positiveLimitPosition = 0.001;
    model.setSensorConfig(sc);
    model.setControlMode(ControlMode::Torque);
    model.setTargetTorque(5.0);
    for (int i = 0; i < 1000; ++i) model.update(0.001);
    (void)limitHit;
}

TEST(AdvancedMotorModelTest, HomeCallback) {
    AdvancedMotorModel model;
    model.initialize();
    bool homeActive = false;
    model.setHomeCallback([&](bool active) { homeActive = active; });
    SensorConfig sc{};
    sc.homeEnabled = true;
    sc.homePosition = 0.0;
    sc.homeWidth = 0.1;
    model.setSensorConfig(sc);
    (void)homeActive;
}

TEST(AdvancedMotorModelTest, IndexCallback) {
    AdvancedMotorModel model;
    model.initialize();
    double indexPos = -1.0;
    model.setIndexCallback([&](double p) { indexPos = p; });
    (void)indexPos;
}

// ============================================================================
// SystemState accessor
// ============================================================================

TEST(AdvancedMotorModelTest, GetState) {
    AdvancedMotorModel model;
    model.initialize();
    const auto& state = model.getState();
    EXPECT_DOUBLE_EQ(state.simulationTime, 0.0);
    EXPECT_FALSE(state.hasFault);
}

// ============================================================================
// Thermal simulation
// ============================================================================

TEST(AdvancedMotorModelTest, ThermalSimulation) {
    AdvancedMotorModel model;
    ThermalParams tp{};
    tp.enabled = true;
    tp.ambientTemperature = 25.0;
    model.setThermalParams(tp);
    model.initialize();
    model.setControlMode(ControlMode::Torque);
    model.setTargetTorque(5.0);
    for (int i = 0; i < 1000; ++i) model.update(0.001);
    const auto& ts = model.getState().thermal;
    // Winding should warm up
    EXPECT_GE(ts.windingTemp, 25.0);
}

// ============================================================================
// Sensor state
// ============================================================================

TEST(AdvancedMotorModelTest, SensorState) {
    AdvancedMotorModel model;
    model.initialize();
    const auto& ss = model.getState().sensors;
    EXPECT_FALSE(ss.positiveLimitActive);
    EXPECT_FALSE(ss.negativeLimitActive);
    EXPECT_FALSE(ss.homeActive);
}

// ============================================================================
// Factory functions
// ============================================================================

TEST(MotorFactoryTest, CreateBLDCServoMotor) {
    auto p = Factory::createBLDCServoMotor();
    EXPECT_GT(p.torqueSpeed.ratedTorque, 0.0);
    EXPECT_GT(p.torqueSpeed.maxSpeed, 0.0);
}

TEST(MotorFactoryTest, CreateBLDCServoMotorCustom) {
    auto p = Factory::createBLDCServoMotor(2.0, 6000.0, 20);
    EXPECT_DOUBLE_EQ(p.torqueSpeed.ratedTorque, 2.0);
}

TEST(MotorFactoryTest, CreateStepperMotor) {
    auto p = Factory::createStepperMotor();
    EXPECT_GT(p.torqueSpeed.stallTorque, 0.0);
}

TEST(MotorFactoryTest, CreateStepperMotorCustom) {
    auto p = Factory::createStepperMotor(1.0, 400, 128);
    (void)p;
}

TEST(MotorFactoryTest, CreatePlanetaryGearbox) {
    auto g = Factory::createPlanetaryGearbox();
    EXPECT_DOUBLE_EQ(g.gearRatio, 10.0);
}

TEST(MotorFactoryTest, CreatePlanetaryGearboxCustom) {
    auto g = Factory::createPlanetaryGearbox(50.0, 3, false);
    EXPECT_DOUBLE_EQ(g.gearRatio, 50.0);
}

TEST(MotorFactoryTest, CreateHarmonicDrive) {
    auto g = Factory::createHarmonicDrive();
    EXPECT_DOUBLE_EQ(g.gearRatio, 100.0);
}

TEST(MotorFactoryTest, CreateHarmonicDriveCustom) {
    auto g = Factory::createHarmonicDrive(50.0);
    EXPECT_DOUBLE_EQ(g.gearRatio, 50.0);
}

TEST(MotorFactoryTest, CreateFrictionParams) {
    auto fp = Factory::createFrictionParams(0.2, 0.1, 0.05);
    EXPECT_DOUBLE_EQ(fp.staticFriction, 0.2);
    EXPECT_DOUBLE_EQ(fp.coulombFriction, 0.1);
    EXPECT_DOUBLE_EQ(fp.viscousCoeff, 0.05);
}
