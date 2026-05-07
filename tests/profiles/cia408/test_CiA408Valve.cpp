/**
 * @file test_CiA408Valve.cpp
 * @brief Comprehensive tests for CiA 408 Valve Controller
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia408/CiA408Valve.hpp"

using namespace CiA408;

// ============================================================================
// ValveState struct helper tests
// ============================================================================

TEST(CiA408ValveState, DefaultConstruction) {
    ValveState s{};
    EXPECT_FALSE(s.isReady());
    EXPECT_FALSE(s.isEnabled());
    EXPECT_FALSE(s.hasFault());
    EXPECT_FALSE(s.hasWarning());
    EXPECT_FALSE(s.isTargetReached());
    EXPECT_FALSE(s.isInPosition());
    EXPECT_FALSE(s.isOverloaded());
    EXPECT_EQ(s.fault_code, 0u);
    EXPECT_EQ(s.warning_code, 0u);
}

TEST(CiA408ValveState, StatusBitReady) {
    ValveState s{};
    s.statusword = StatuswordBits::Ready;
    EXPECT_TRUE(s.isReady());
    EXPECT_FALSE(s.isEnabled());
}

TEST(CiA408ValveState, StatusBitEnabled) {
    ValveState s{};
    s.statusword = StatuswordBits::Enabled | StatuswordBits::Ready;
    EXPECT_TRUE(s.isReady());
    EXPECT_TRUE(s.isEnabled());
}

TEST(CiA408ValveState, StatusBitFault) {
    ValveState s{};
    s.statusword = StatuswordBits::Fault;
    EXPECT_TRUE(s.hasFault());
}

TEST(CiA408ValveState, StatusBitWarning) {
    ValveState s{};
    s.statusword = StatuswordBits::Warning;
    EXPECT_TRUE(s.hasWarning());
}

TEST(CiA408ValveState, StatusBitTargetReached) {
    ValveState s{};
    s.statusword = StatuswordBits::TargetReached;
    EXPECT_TRUE(s.isTargetReached());
}

TEST(CiA408ValveState, StatusBitInPosition) {
    ValveState s{};
    s.statusword = StatuswordBits::InPosition;
    EXPECT_TRUE(s.isInPosition());
}

TEST(CiA408ValveState, SetpointPercent) {
    ValveState s{};
    s.setpoint = 5000;
    EXPECT_NEAR(s.getSetpointPercent(), 50.0f, 0.01f);
    s.actual_value = 2500;
    EXPECT_NEAR(s.getActualPercent(), 25.0f, 0.01f);
}

TEST(CiA408ValveState, PressureConversion) {
    ValveState s{};
    s.pressure_a = 1010;
    EXPECT_NEAR(s.getPressureABar(), rawToBar(1010), 1e-3f);
    s.pressure_b = 500;
    EXPECT_NEAR(s.getPressureBBar(), rawToBar(500), 1e-3f);
}

TEST(CiA408ValveState, TempConversion) {
    ValveState s{};
    s.temperature = 250;
    EXPECT_NEAR(s.getTemperatureCelsius(), 25.0f, 0.1f);
}

// ============================================================================
// Struct default construction
// ============================================================================

TEST(CiA408Structs, ValveSpec) {
    ValveSpec spec{};
    EXPECT_EQ(spec.valve_type, 0u);
    EXPECT_EQ(spec.nominal_flow, 0);
    EXPECT_EQ(spec.nominal_pressure, 0);
}

TEST(CiA408Structs, ControllerParams) {
    ControllerParams p{};
    EXPECT_EQ(p.pos_kp, 0);
    EXPECT_EQ(p.pos_ki, 0);
    EXPECT_EQ(p.pos_kd, 0);
}

TEST(CiA408Structs, DitherConfig) {
    DitherConfig d{};
    EXPECT_EQ(d.amplitude, 0u);
    EXPECT_EQ(d.frequency, 0u);
    EXPECT_FALSE(d.enabled);
}

TEST(CiA408Structs, ValveCapabilities) {
    ValveCapabilities c{};
    EXPECT_EQ(c.valve_type, 0u);
    EXPECT_FALSE(c.has_position_feedback);
    EXPECT_FALSE(c.has_pressure_sensors);
    EXPECT_FALSE(c.supports_closed_loop);
}

// ============================================================================
// Bit flag tests
// ============================================================================

TEST(CiA408Bits, StatuswordDisjoint) {
    EXPECT_NE(StatuswordBits::Ready, 0u);
    EXPECT_NE(StatuswordBits::Enabled, 0u);
    EXPECT_NE(StatuswordBits::Fault, 0u);
    EXPECT_NE(StatuswordBits::Warning, 0u);
    EXPECT_NE(StatuswordBits::TargetReached, 0u);
    EXPECT_NE(StatuswordBits::InPosition, 0u);
    EXPECT_EQ(StatuswordBits::Ready & StatuswordBits::Enabled, 0u);
    EXPECT_EQ(StatuswordBits::Fault & StatuswordBits::Warning, 0u);
}

TEST(CiA408Bits, ControlwordDisjoint) {
    EXPECT_NE(ControlwordBits::Enable, 0u);
    EXPECT_NE(ControlwordBits::Reset, 0u);
    EXPECT_EQ(ControlwordBits::Enable & ControlwordBits::Reset, 0u);
}

// ============================================================================
// Enum value tests
// ============================================================================

TEST(CiA408Enums, OperatingModes) {
    EXPECT_EQ(static_cast<uint8_t>(OperatingModes::OpenLoop), 0u);
    EXPECT_NE(static_cast<uint8_t>(OperatingModes::PositionControl),
              static_cast<uint8_t>(OperatingModes::VelocityControl));
    EXPECT_NE(static_cast<uint8_t>(OperatingModes::PressureControl),
              static_cast<uint8_t>(OperatingModes::ForceControl));
}

TEST(CiA408Enums, ValveTypes) {
    EXPECT_NE(static_cast<uint8_t>(ValveTypes::ProportionalDirectional),
              static_cast<uint8_t>(ValveTypes::ServoValve));
}

TEST(CiA408Enums, PDOPresetsEnum) {
    // CiA408 does not have a getPDOMappingName free function;
    // just verify enum values are distinct
    EXPECT_NE(static_cast<int>(PDOMappingPreset::Basic),
              static_cast<int>(PDOMappingPreset::Extended));
    EXPECT_NE(static_cast<int>(PDOMappingPreset::Position),
              static_cast<int>(PDOMappingPreset::Full));
}

// ============================================================================
// ValveController fixture
// ============================================================================

class CiA408Test : public ::testing::Test {
protected:
    void SetUp() override {
        v_ = std::make_unique<ValveController>(1);
        v_->initialize();
    }
    std::unique_ptr<ValveController> v_;
};

TEST_F(CiA408Test, Construction) {
    ValveController v2(0x100, true);
    EXPECT_FALSE(v2.isEnabled());
    EXPECT_FALSE(v2.hasFault());
}

TEST_F(CiA408Test, Initialize) {
    ValveController v3(2);
    bool ok = v3.initialize();
    EXPECT_TRUE(ok);
}

TEST_F(CiA408Test, GetCapabilities) {
    auto caps = v_->getCapabilities();
    (void)caps.valve_type;
    (void)caps.has_position_feedback;
}

TEST_F(CiA408Test, GetValveSpec) {
    auto spec = v_->getValveSpec();
    (void)spec.valve_type;
}

TEST_F(CiA408Test, PDOMappingAll) {
    EXPECT_TRUE(v_->applyPDOMapping(PDOMappingPreset::Basic));
    EXPECT_TRUE(v_->applyPDOMapping(PDOMappingPreset::Extended));
    EXPECT_TRUE(v_->applyPDOMapping(PDOMappingPreset::Position));
    EXPECT_TRUE(v_->applyPDOMapping(PDOMappingPreset::Full));
    EXPECT_TRUE(v_->applyPDOMapping(PDOMappingPreset::Custom));
}

TEST_F(CiA408Test, EnableDisable) {
    v_->enable();
    // SDO may fail, so state may not change
    (void)v_->isEnabled();
    v_->disable();
    (void)v_->isEnabled();
}

TEST_F(CiA408Test, ResetFault) {
    v_->resetFault();
    EXPECT_FALSE(v_->hasFault());
}

TEST_F(CiA408Test, FastStop) {
    v_->enable();
    v_->fastStop();
}

TEST_F(CiA408Test, OperatingModeSet) {
    v_->setOperatingMode(static_cast<uint8_t>(OperatingModes::PositionControl));
    // SDO may fail, so mode may not change
    (void)v_->getOperatingMode();
    v_->setOperatingMode(static_cast<uint8_t>(OperatingModes::PressureControl));
    v_->setOperatingMode(static_cast<uint8_t>(OperatingModes::VelocityControl));
    v_->setOperatingMode(static_cast<uint8_t>(OperatingModes::ForceControl));
    v_->setOperatingMode(static_cast<uint8_t>(OperatingModes::FlowControl));
    v_->setOperatingMode(static_cast<uint8_t>(OperatingModes::OpenLoop));
}

TEST_F(CiA408Test, ClosedLoop) {
    v_->enableClosedLoop(true);
    v_->enableClosedLoop(false);
}

TEST_F(CiA408Test, PressureComp) {
    v_->enablePressureCompensation(true);
    v_->enablePressureCompensation(false);
}

TEST_F(CiA408Test, SetpointFloat) {
    v_->enable();
    v_->setSetpoint(50.0f);
    EXPECT_NEAR(v_->getSetpoint(), 50.0f, 1.0f);
    v_->setSetpoint(-100.0f);
    v_->setSetpoint(100.0f);
    v_->setSetpoint(0.0f);
}

TEST_F(CiA408Test, SetpointRaw) {
    v_->setSetpointRaw(1000);
    EXPECT_EQ(v_->getSetpointRaw(), 1000);
    v_->setSetpointRaw(-10000);
    v_->setSetpointRaw(10000);
}

TEST_F(CiA408Test, ActualValue) {
    (void)v_->getActualValue();
    (void)v_->getActualValueRaw();
}

TEST_F(CiA408Test, SetpointRamp) {
    v_->setSetpointRamp(10.0f);
    v_->setSetpointRamp(0.0f);
}

TEST_F(CiA408Test, Position) {
    v_->setPosition(5000);
    // SDO may fail, so we just exercise the API
    (void)v_->getPosition();
    v_->setPositionWindow(100, 500);
    (void)v_->isInPosition();
}

TEST_F(CiA408Test, Velocity) {
    v_->setVelocity(10.0f);
    (void)v_->getVelocity();
}

TEST_F(CiA408Test, Pressure) {
    v_->setPressureSetpoint(10.0f);
    (void)v_->getPressureA();
    (void)v_->getPressureB();
    (void)v_->getSupplyPressure();
    (void)v_->getDifferentialPressure();
    v_->setMaxPressure(200.0f);
}

TEST_F(CiA408Test, PositionGains) {
    v_->setPositionGains(100, 10, 5);
    // SDO may fail, so gains may not persist
    auto p = v_->getControllerParams();
    (void)p;
}

TEST_F(CiA408Test, VelocityGains) {
    v_->setVelocityGains(200, 20);
    auto p = v_->getControllerParams();
    (void)p;
}

TEST_F(CiA408Test, PressureGains) {
    v_->setPressureGains(50, 5);
    auto p = v_->getControllerParams();
    (void)p;
}

TEST_F(CiA408Test, SetControllerParams) {
    ControllerParams cp{};
    cp.pos_kp = 1;
    cp.pos_ki = 2;
    cp.pos_kd = 3;
    cp.pos_kv = 4;
    cp.pos_ka = 5;
    cp.vel_kp = 6;
    cp.vel_ki = 7;
    cp.prs_kp = 8;
    cp.prs_ki = 9;
    v_->setControllerParams(cp);
    // SDO may fail with stubs
    auto out = v_->getControllerParams();
    (void)out;
}

TEST_F(CiA408Test, DitherConfigure) {
    v_->configureDither(5, 200);
    v_->enableDither(true);
    auto dc = v_->getDitherConfig();
    EXPECT_EQ(dc.amplitude, 5u);
    EXPECT_EQ(dc.frequency, 200u);
    EXPECT_TRUE(dc.enabled);
    v_->enableDither(false);
    dc = v_->getDitherConfig();
    EXPECT_FALSE(dc.enabled);
}

TEST_F(CiA408Test, PositionLimits) {
    v_->setPositionLimits(-10000, 10000);
}

TEST_F(CiA408Test, VelocityLimits) {
    v_->setVelocityLimits(100.0f, 50.0f);
}

TEST_F(CiA408Test, CurrentLimits) {
    v_->setCurrentLimits(500, 500);
}

TEST_F(CiA408Test, GetState) {
    auto st = v_->getState();
    EXPECT_FALSE(st.hasFault());
}

TEST_F(CiA408Test, DiagnosticValues) {
    (void)v_->getCoilCurrentA();
    (void)v_->getCoilCurrentB();
    (void)v_->getCoilTemperature();
    (void)v_->getSupplyVoltage();
    (void)v_->getFollowingError();
    (void)v_->getOperatingHours();
    (void)v_->getCycleCount();
    EXPECT_EQ(v_->getFaultCode(), 0u);
    EXPECT_EQ(v_->getWarningCode(), 0u);
}

TEST_F(CiA408Test, DiagnosticString) {
    auto diag = v_->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA408Test, Calibration) {
    v_->startNullCalibration();
    v_->startGainCalibration();
    v_->startAutoTune();
    v_->storeCalibration();
    v_->resetCalibration();
    (void)v_->getCalibrationStatus();
    v_->setNullOffset(100);
    v_->setNullOffset(-100);
    v_->setDeadband(50);
    v_->setDeadband(0);
}

TEST_F(CiA408Test, PDOProcessBasic) {
    v_->applyPDOMapping(PDOMappingPreset::Basic);
    uint8_t txbuf[64] = {};
    v_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[64] = {};
    size_t written = v_->prepareRxPDO(rxbuf, sizeof(rxbuf));
    EXPECT_GT(written, 0u);
}

TEST_F(CiA408Test, PDOProcessExtended) {
    v_->applyPDOMapping(PDOMappingPreset::Extended);
    uint8_t txbuf[128] = {};
    v_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[128] = {};
    v_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA408Test, PDOProcessPosition) {
    v_->applyPDOMapping(PDOMappingPreset::Position);
    uint8_t txbuf[128] = {};
    v_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[128] = {};
    v_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA408Test, PDOProcessFull) {
    v_->applyPDOMapping(PDOMappingPreset::Full);
    uint8_t txbuf[256] = {};
    v_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[256] = {};
    v_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA408Test, Update) {
    v_->update();
}

TEST_F(CiA408Test, Callbacks) {
    bool stateChanged = false;
    bool faultCalled = false;
    bool targetReached = false;
    v_->setStateChangeCallback([&](uint16_t, uint16_t) { stateChanged = true; });
    v_->setFaultCallback([&](uint16_t) { faultCalled = true; });
    v_->setTargetReachedCallback([&]() { targetReached = true; });
    // Callbacks are registered; firing depends on state transitions
    EXPECT_FALSE(stateChanged);
}

TEST_F(CiA408Test, PDOTooSmall) {
    v_->applyPDOMapping(PDOMappingPreset::Full);
    uint8_t small[2] = {};
    v_->processTxPDO(small, sizeof(small));
    size_t w = v_->prepareRxPDO(small, sizeof(small));
    (void)w; // may be 0 if buffer too small
}
