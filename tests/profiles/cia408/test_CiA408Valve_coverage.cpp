/**
 * @file test_CiA408Valve_coverage.cpp
 * @brief Extended coverage tests for CiA408 ValveController — exercises
 *        uncovered struct helpers, conversion functions, diagnostics string,
 *        enum-to-string functions, and additional state/callback branches.
 */

#include "tether/profiles/cia408/CiA408Valve.hpp"
#include <gtest/gtest.h>
#include <set>
#include <string>

using namespace CiA408;

// ============================================================================
// ValveState helper methods — all branches
// ============================================================================

TEST(Valve408CovTest, ValveState_Default) {
    ValveState s{};
    EXPECT_FALSE(s.isReady());
    EXPECT_FALSE(s.isEnabled());
    EXPECT_FALSE(s.hasFault());
    EXPECT_FALSE(s.hasWarning());
    EXPECT_FALSE(s.isTargetReached());
    EXPECT_FALSE(s.isInPosition());
    EXPECT_FALSE(s.isOverloaded());
    EXPECT_FLOAT_EQ(s.getSetpointPercent(), 0.0f);
    EXPECT_FLOAT_EQ(s.getActualPercent(), 0.0f);
}

TEST(Valve408CovTest, ValveState_Ready) {
    ValveState s{};
    s.statusword = 0x0001; // Ready bit
    EXPECT_TRUE(s.isReady());
}

TEST(Valve408CovTest, ValveState_Enabled) {
    ValveState s{};
    s.statusword = 0x0002; // Enabled bit
    EXPECT_TRUE(s.isEnabled());
}

TEST(Valve408CovTest, ValveState_Fault) {
    ValveState s{};
    s.statusword = 0x0004; // Fault bit
    EXPECT_TRUE(s.hasFault());
}

TEST(Valve408CovTest, ValveState_Warning) {
    ValveState s{};
    s.statusword = 0x0008; // Warning bit
    EXPECT_TRUE(s.hasWarning());
}

TEST(Valve408CovTest, ValveState_TargetReached) {
    ValveState s{};
    s.statusword = 0x0010; // TargetReached bit
    EXPECT_TRUE(s.isTargetReached());
}

TEST(Valve408CovTest, ValveState_InPosition) {
    ValveState s{};
    s.statusword = 0x0020; // InPosition bit
    EXPECT_TRUE(s.isInPosition());
}

TEST(Valve408CovTest, ValveState_Overloaded) {
    ValveState s{};
    s.statusword = StatuswordBits::Overload; // 0x0800
    EXPECT_TRUE(s.isOverloaded());
}

TEST(Valve408CovTest, ValveState_Setpoint) {
    ValveState s{};
    s.setpoint = 10000;
    EXPECT_NEAR(s.getSetpointPercent(), 100.0f, 0.01f);
}

TEST(Valve408CovTest, ValveState_Actual) {
    ValveState s{};
    s.actual_value = -10000;
    EXPECT_NEAR(s.getActualPercent(), -100.0f, 0.01f);
}

TEST(Valve408CovTest, ValveState_Pressure) {
    ValveState s{};
    s.pressure_a = 5000;
    s.pressure_b = 3000;
    EXPECT_GT(s.getPressureABar(), 0.0f);
    EXPECT_GT(s.getPressureBBar(), 0.0f);
}

TEST(Valve408CovTest, ValveState_Temperature) {
    ValveState s{};
    s.temperature = 250; // 25.0°C (units are °C * 10)
    EXPECT_NEAR(s.getTemperatureCelsius(), 25.0f, 0.01f);
}

// ============================================================================
// ValveSpec struct
// ============================================================================

TEST(Valve408CovTest, ValveSpec_Default) {
    ValveSpec spec{};
    EXPECT_EQ(spec.valve_type, 0);
    EXPECT_EQ(spec.nominal_flow, 0u);
    EXPECT_EQ(spec.nominal_pressure, 0u);
    EXPECT_EQ(spec.nominal_stroke, 0);
    EXPECT_EQ(spec.response_time, 0u);
    EXPECT_EQ(spec.hysteresis, 0);
    EXPECT_EQ(spec.repeatability, 0);
}

TEST(Valve408CovTest, ValveSpec_Set) {
    ValveSpec spec{};
    spec.valve_type = 1;
    spec.nominal_flow = 400;      // L/min * 10
    spec.nominal_pressure = 2100; // bar * 10
    spec.nominal_stroke = 1000;   // µm
    spec.response_time = 20;      // ms
    spec.hysteresis = 5;          // 0.1%
    spec.repeatability = 1;       // 0.1%
    EXPECT_EQ(spec.valve_type, 1);
    EXPECT_EQ(spec.nominal_flow, 400u);
}

// ============================================================================
// ControllerParams struct
// ============================================================================

TEST(Valve408CovTest, ControllerParams_Default) {
    ControllerParams p{};
    EXPECT_EQ(p.pos_kp, 0);
    EXPECT_EQ(p.pos_ki, 0);
    EXPECT_EQ(p.pos_kd, 0);
    EXPECT_EQ(p.vel_kp, 0);
    EXPECT_EQ(p.vel_ki, 0);
    EXPECT_EQ(p.prs_kp, 0);
    EXPECT_EQ(p.prs_ki, 0);
}

TEST(Valve408CovTest, ControllerParams_Set) {
    ControllerParams p{};
    p.pos_kp = 100;
    p.pos_ki = 10;
    p.pos_kd = 5;
    p.pos_kv = 2;
    p.pos_ka = 1;
    p.pos_limit = 32767;
    p.vel_kp = 50;
    p.vel_ki = 20;
    p.vel_limit = 16000;
    p.prs_kp = 30;
    p.prs_ki = 15;
    p.prs_limit = 8000;
    EXPECT_EQ(p.pos_kp, 100);
    EXPECT_EQ(p.prs_limit, 8000);
}

// ============================================================================
// DitherConfig struct
// ============================================================================

TEST(Valve408CovTest, DitherConfig_Default) {
    DitherConfig d{};
    EXPECT_EQ(d.amplitude, 0);
    EXPECT_EQ(d.frequency, 0);
    EXPECT_FALSE(d.enabled);
}

TEST(Valve408CovTest, DitherConfig_Set) {
    DitherConfig d{};
    d.amplitude = 500;
    d.frequency = 100;
    d.enabled = true;
    EXPECT_EQ(d.amplitude, 500);
    EXPECT_TRUE(d.enabled);
}

// ============================================================================
// ValveCapabilities
// ============================================================================

TEST(Valve408CovTest, ValveCapabilities_Default) {
    ValveCapabilities c{};
    EXPECT_FALSE(c.has_position_feedback);
    EXPECT_FALSE(c.has_pressure_sensors);
    EXPECT_FALSE(c.has_dual_coils);
    EXPECT_FALSE(c.supports_closed_loop);
    EXPECT_FALSE(c.supports_pressure_control);
    EXPECT_FALSE(c.supports_force_control);
    EXPECT_FALSE(c.supports_dither);
    EXPECT_EQ(c.num_channels, 1u);  // Default is 1
}

TEST(Valve408CovTest, ValveCapabilities_AllTrue) {
    ValveCapabilities c{};
    c.has_position_feedback = true;
    c.has_pressure_sensors = true;
    c.has_dual_coils = true;
    c.supports_closed_loop = true;
    c.supports_pressure_control = true;
    c.supports_force_control = true;
    c.supports_dither = true;
    c.num_channels = 4;
    EXPECT_TRUE(c.has_position_feedback);
    EXPECT_EQ(c.num_channels, 4);
}

// ============================================================================
// Conversion functions (from CiA408Defs.hpp)
// ============================================================================

TEST(Valve408CovTest, RawToPercent) {
    EXPECT_FLOAT_EQ(rawToPercent(0), 0.0f);
    float pct = rawToPercent(10000);
    EXPECT_GT(pct, 0.0f);
    float neg = rawToPercent(-10000);
    EXPECT_LT(neg, 0.0f);
}

TEST(Valve408CovTest, PercentToRaw) {
    EXPECT_EQ(percentToRaw(0.0f), 0);
    int16_t raw = percentToRaw(50.0f);
    EXPECT_GT(raw, 0);
    int16_t neg = percentToRaw(-50.0f);
    EXPECT_LT(neg, 0);
}

TEST(Valve408CovTest, RawToBar) {
    EXPECT_FLOAT_EQ(rawToBar(0), 0.0f);
    EXPECT_NEAR(rawToBar(1000), 100.0f, 0.01f);  // 1000 raw = 100.0 bar
}

TEST(Valve408CovTest, BarToRaw) {
    EXPECT_EQ(barToRaw(0.0f), 0);
    int16_t raw = barToRaw(100.0f);  // Returns int16_t
    EXPECT_EQ(raw, 1000);  // 100.0 bar * 10 = 1000
}

// ============================================================================
// Enum-to-string functions
// ============================================================================

TEST(Valve408CovTest, OperatingModeName) {
    // All 8 modes + unknown default
    EXPECT_STREQ(getOperatingModeName(OperatingModes::OpenLoop), "Open Loop");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::PositionControl), "Position Control");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::VelocityControl), "Velocity Control");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::PressureControl), "Pressure Control");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::ForceControl), "Force Control");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::FlowControl), "Flow Control");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::Synchronized), "Synchronized");
    EXPECT_STREQ(getOperatingModeName(OperatingModes::Manual), "Manual");
    EXPECT_STREQ(getOperatingModeName(255), "Unknown");
}

TEST(Valve408CovTest, ValveTypeName) {
    // All 7 types + unknown
    EXPECT_STREQ(getValveTypeName(ValveTypes::ProportionalDirectional), "Proportional Directional");
    EXPECT_STREQ(getValveTypeName(ValveTypes::ServoValve), "Servo Valve");
    EXPECT_STREQ(getValveTypeName(ValveTypes::ProportionalPressure), "Proportional Pressure");
    EXPECT_STREQ(getValveTypeName(ValveTypes::ProportionalFlow), "Proportional Flow");
    EXPECT_STREQ(getValveTypeName(ValveTypes::VariablePump), "Variable Pump");
    EXPECT_STREQ(getValveTypeName(ValveTypes::ProportionalThrottle), "Proportional Throttle");
    EXPECT_STREQ(getValveTypeName(ValveTypes::OnOffValve), "On/Off Valve");
    EXPECT_STREQ(getValveTypeName(0), "Unknown");
    EXPECT_STREQ(getValveTypeName(255), "Unknown");
}

TEST(Valve408CovTest, FaultName_AllCodes) {
    // Exercise every fault code branch in getFaultName
    EXPECT_STREQ(getFaultName(FaultCodes::None), "None");
    EXPECT_STREQ(getFaultName(FaultCodes::Overvoltage), "Overvoltage");
    EXPECT_STREQ(getFaultName(FaultCodes::Undervoltage), "Undervoltage");
    EXPECT_STREQ(getFaultName(FaultCodes::Overcurrent), "Overcurrent");
    EXPECT_STREQ(getFaultName(FaultCodes::CoilOpenA), "Coil A Open");
    EXPECT_STREQ(getFaultName(FaultCodes::CoilOpenB), "Coil B Open");
    EXPECT_STREQ(getFaultName(FaultCodes::CoilShortA), "Coil A Short");
    EXPECT_STREQ(getFaultName(FaultCodes::CoilShortB), "Coil B Short");
    EXPECT_STREQ(getFaultName(FaultCodes::Overtemperature), "Overtemperature");
    EXPECT_STREQ(getFaultName(FaultCodes::LVDTFault), "LVDT Fault");
    EXPECT_STREQ(getFaultName(FaultCodes::PressureSensorFault), "Pressure Sensor Fault");
    EXPECT_STREQ(getFaultName(FaultCodes::FollowingError), "Following Error");
    EXPECT_STREQ(getFaultName(FaultCodes::PressureOverload), "Pressure Overload");
    EXPECT_STREQ(getFaultName(FaultCodes::InternalFault), "Internal Fault");
    EXPECT_STREQ(getFaultName(FaultCodes::CommunicationFault), "Communication Fault");
}

// ============================================================================
// Enum values distinct
// ============================================================================

TEST(Valve408CovTest, PDOMappingPresetValues) {
    std::set<int> vals;
    vals.insert(static_cast<int>(PDOMappingPreset::Basic));
    vals.insert(static_cast<int>(PDOMappingPreset::Extended));
    vals.insert(static_cast<int>(PDOMappingPreset::Position));
    vals.insert(static_cast<int>(PDOMappingPreset::Full));
    vals.insert(static_cast<int>(PDOMappingPreset::Custom));
    EXPECT_EQ(vals.size(), 5u);
}

// ============================================================================
// ValveController via extern C stubs — all methods
// ============================================================================

TEST(Valve408CovTest, ControllerConstruction) {
    ValveController v(1);
    EXPECT_FALSE(v.isInitialized());
}

TEST(Valve408CovTest, ControllerConstruction_ConfigAddr) {
    ValveController v(0x1001, true);
    EXPECT_FALSE(v.isInitialized());
}

TEST(Valve408CovTest, InitAndCapabilities) {
    ValveController v(1);
    v.initialize();
    auto caps = v.getCapabilities();
    (void)caps;
    auto spec = v.getValveSpec();
    (void)spec;
}

TEST(Valve408CovTest, EnableDisableResetFastStop) {
    ValveController v(1);
    v.initialize();
    v.enable();
    v.disable();
    v.resetFault();
    v.fastStop();
    EXPECT_FALSE(v.isEnabled());
}

TEST(Valve408CovTest, OperatingMode) {
    ValveController v(1);
    v.initialize();
    v.setOperatingMode(1);
    EXPECT_EQ(v.getOperatingMode(), 0u); // Stub fails reads
}

TEST(Valve408CovTest, ClosedLoopAndPressureComp) {
    ValveController v(1);
    v.initialize();
    v.enableClosedLoop(true);
    v.enableClosedLoop(false);
    v.enablePressureCompensation(true);
    v.enablePressureCompensation(false);
}

TEST(Valve408CovTest, SetpointMethods) {
    ValveController v(1);
    v.initialize();
    v.setSetpoint(50.0f);
    // setSetpoint stores target locally even when SDO write fails
    EXPECT_NEAR(v.getSetpoint(), 50.0f, 0.01f);
    v.setSetpointRaw(10000);
    EXPECT_EQ(v.getSetpointRaw(), 10000);
    // getActualValue reads from state_ which is zero
    EXPECT_FLOAT_EQ(v.getActualValue(), 0.0f);
    EXPECT_EQ(v.getActualValueRaw(), 0);
    v.setSetpointRamp(10.0f);
}

TEST(Valve408CovTest, PositionMethods) {
    ValveController v(1);
    v.initialize();
    v.setPosition(5000);
    EXPECT_EQ(v.getPosition(), 0);
    v.setPositionWindow(100, 50);
    EXPECT_FALSE(v.isInPosition());
}

TEST(Valve408CovTest, VelocityMethods) {
    ValveController v(1);
    v.initialize();
    v.setVelocity(100.0f);
    EXPECT_FLOAT_EQ(v.getVelocity(), 0.0f);
}

TEST(Valve408CovTest, PressureMethods) {
    ValveController v(1);
    v.initialize();
    v.setPressureSetpoint(100.0f);
    EXPECT_FLOAT_EQ(v.getPressureA(), 0.0f);
    EXPECT_FLOAT_EQ(v.getPressureB(), 0.0f);
    EXPECT_FLOAT_EQ(v.getSupplyPressure(), 0.0f);
    EXPECT_FLOAT_EQ(v.getDifferentialPressure(), 0.0f);
    v.setMaxPressure(300.0f);
}

TEST(Valve408CovTest, ControllerGains) {
    ValveController v(1);
    v.initialize();
    v.setPositionGains(100, 10, 5);
    v.setVelocityGains(50, 20);
    v.setPressureGains(30, 15);
    auto p = v.getControllerParams();
    (void)p;
    ControllerParams cp{};
    cp.pos_kp = 200;
    v.setControllerParams(cp);
}

TEST(Valve408CovTest, DitherControl) {
    ValveController v(1);
    v.initialize();
    v.configureDither(500, 100);
    v.enableDither(true);
    v.enableDither(false);
    auto d = v.getDitherConfig();
    (void)d;
}

TEST(Valve408CovTest, Limits) {
    ValveController v(1);
    v.initialize();
    v.setPositionLimits(-10000, 10000);
    v.setVelocityLimits(100.0f, 50.0f);
    v.setCurrentLimits(1000, 1000);
}

TEST(Valve408CovTest, DiagnosticValues) {
    ValveController v(1);
    v.initialize();
    EXPECT_FLOAT_EQ(v.getCoilCurrentA(), 0.0f);
    EXPECT_FLOAT_EQ(v.getCoilCurrentB(), 0.0f);
    EXPECT_FLOAT_EQ(v.getCoilTemperature(), 0.0f);
    EXPECT_FLOAT_EQ(v.getSupplyVoltage(), 0.0f);
    EXPECT_FLOAT_EQ(v.getFollowingError(), 0.0f);
    EXPECT_EQ(v.getOperatingHours(), 0u);
    EXPECT_EQ(v.getCycleCount(), 0u);
    EXPECT_EQ(v.getFaultCode(), 0u);
    EXPECT_EQ(v.getWarningCode(), 0u);
}

TEST(Valve408CovTest, DiagnosticsString) {
    ValveController v(1);
    v.initialize();
    auto diag = v.getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST(Valve408CovTest, CalibrationMethods) {
    ValveController v(1);
    v.initialize();
    v.startNullCalibration();
    v.startGainCalibration();
    v.startAutoTune();
    v.storeCalibration();
    v.resetCalibration();
    auto status = v.getCalibrationStatus();
    (void)status;
    v.setNullOffset(100);
    v.setDeadband(50);
}

TEST(Valve408CovTest, Callbacks) {
    ValveController v(1);
    int stateCount = 0, faultCount = 0, reachedCount = 0;
    v.setStateChangeCallback([&](uint16_t, uint16_t) { stateCount++; });
    v.setFaultCallback([&](uint16_t) { faultCount++; });
    v.setTargetReachedCallback([&]() { reachedCount++; });
    // Callbacks won't fire without proper PDO updates
}

TEST(Valve408CovTest, PDOProcess_Basic) {
    ValveController v(1);
    v.applyPDOMapping(PDOMappingPreset::Basic);
    uint8_t txData[4] = {};
    v.processTxPDO(txData, sizeof(txData));
    uint8_t rxData[4] = {};
    auto size = v.prepareRxPDO(rxData, sizeof(rxData));
    EXPECT_GT(size, 0u);
}

TEST(Valve408CovTest, PDOProcess_Extended) {
    ValveController v(1);
    v.applyPDOMapping(PDOMappingPreset::Extended);
    uint8_t txData[8] = {};
    v.processTxPDO(txData, sizeof(txData));
    uint8_t rxData[8] = {};
    auto size = v.prepareRxPDO(rxData, sizeof(rxData));
    EXPECT_GT(size, 0u);
}

TEST(Valve408CovTest, PDOProcess_Position) {
    ValveController v(1);
    v.applyPDOMapping(PDOMappingPreset::Position);
    uint8_t txData[8] = {};
    v.processTxPDO(txData, sizeof(txData));
    uint8_t rxData[8] = {};
    auto size = v.prepareRxPDO(rxData, sizeof(rxData));
    EXPECT_GT(size, 0u);
}

TEST(Valve408CovTest, PDOProcess_Full) {
    ValveController v(1);
    v.applyPDOMapping(PDOMappingPreset::Full);
    uint8_t txData[16] = {};
    v.processTxPDO(txData, sizeof(txData));
    uint8_t rxData[8] = {};
    auto size = v.prepareRxPDO(rxData, sizeof(rxData));
    EXPECT_GT(size, 0u);
}

TEST(Valve408CovTest, Update) {
    ValveController v(1);
    v.initialize();
    v.update();
}

TEST(Valve408CovTest, GetState) {
    ValveController v(1);
    v.initialize();
    auto state = v.getState();
    EXPECT_EQ(state.statusword, 0u);
}
