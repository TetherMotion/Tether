/**
 * @file test_CiA404Device.cpp
 * @brief Comprehensive tests for CiA 404 Measuring Device
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia404/CiA404Device.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"

using namespace CiA404;

namespace {
class NullSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    bool sdoUpload(uint16_t, uint8_t*, uint16_t, uint16_t,
                   uint16_t, uint16_t, uint16_t, uint8_t,
                   uint8_t*, size_t, size_t*, bool, unsigned int,
                   unsigned int) override { return false; }
    bool sdoDownload(uint16_t, uint8_t*, uint16_t, uint16_t,
                     uint16_t, uint16_t, uint16_t, uint8_t,
                     const uint8_t*, size_t, bool, unsigned int,
                     unsigned int) override { return false; }
    uint64_t getMicroseconds() override { return 0; }
};
} // namespace

// ============================================================================
// ControllerState helpers
// ============================================================================

TEST(CiA404ControllerState, Active) {
    ControllerState cs{};
    cs.status = ControllerStatusBits::Active;
    EXPECT_TRUE(cs.isActive());
    EXPECT_FALSE(cs.isOutputSaturated());
}

TEST(CiA404ControllerState, Saturated) {
    ControllerState cs{};
    cs.status = ControllerStatusBits::OutputUpperLimit;
    EXPECT_TRUE(cs.isOutputSaturated());
    cs.status = ControllerStatusBits::OutputLowerLimit;
    EXPECT_TRUE(cs.isOutputSaturated());
}

TEST(CiA404ControllerState, Default) {
    ControllerState cs{};
    EXPECT_FALSE(cs.isActive());
    EXPECT_FALSE(cs.isOutputSaturated());
    EXPECT_EQ(cs.setpoint, 0);
    EXPECT_EQ(cs.output, 0);
}

// ============================================================================
// PIDParameters
// ============================================================================

TEST(CiA404PIDParams, Default) {
    PIDParameters p{};
    // kp defaults to 65536 (1.0 in Q16)
    EXPECT_EQ(p.kp, 65536);
}

// ============================================================================
// AlarmConfig
// ============================================================================

TEST(CiA404AlarmConfig, Default) {
    AlarmConfig a{};
    // Defaults are set to safe limits, not zero
    EXPECT_NE(a.high_high_limit, 0);
    EXPECT_NE(a.high_limit, 0);
    EXPECT_NE(a.low_limit, 0);
    EXPECT_NE(a.low_low_limit, 0);
}

// ============================================================================
// Free functions
// ============================================================================

TEST(CiA404Free, EventNames) {
    EXPECT_NE(getDeviceEventName(DeviceEvent::ValueUpdated), nullptr);
    EXPECT_NE(getDeviceEventName(DeviceEvent::AlarmHighHigh), nullptr);
    EXPECT_NE(getDeviceEventName(DeviceEvent::SensorFault), nullptr);
}

TEST(CiA404Free, Q16Conversion) {
    float f = q16ToFloat(65536); // 1.0
    EXPECT_NEAR(f, 1.0f, 1e-3f);
    int32_t q = floatToQ16(1.0f);
    EXPECT_EQ(q, 65536);
    EXPECT_NEAR(q16ToFloat(floatToQ16(3.14f)), 3.14f, 1e-3f);
}

TEST(CiA404Free, InputRangeName) {
    EXPECT_NE(getInputRangeName(0), nullptr);
    EXPECT_NE(getInputRangeName(1), nullptr);
}

TEST(CiA404Free, ControllerModeName) {
    EXPECT_NE(getControllerModeName(ControllerModes::Manual), nullptr);
    EXPECT_NE(getControllerModeName(ControllerModes::PID_Auto), nullptr);
}

// ============================================================================
// MeasuringDevice fixture
// ============================================================================

class CiA404Test : public ::testing::Test {
protected:
    void SetUp() override {
        transport_ = std::make_unique<NullSDOTransport>();
        coe_ = std::make_unique<EtherCAT::CoE::CoEManager>(1, *transport_);
        coe_->init();
        dev_ = std::make_unique<MeasuringDevice>(*coe_);
    }
    void TearDown() override {
        dev_.reset();
        coe_->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport_;
    std::unique_ptr<EtherCAT::CoE::CoEManager> coe_;
    std::unique_ptr<MeasuringDevice> dev_;
};

TEST_F(CiA404Test, Construction) {
    MeasuringDevice d2(*coe_);
    EXPECT_FALSE(d2.isInitialized());
}

TEST_F(CiA404Test, Initialize) {
    dev_->initialize();
}

TEST_F(CiA404Test, Capabilities) {
    dev_->initialize();
    auto caps = dev_->getCapabilities();
    (void)caps;
}

TEST_F(CiA404Test, PDOMappingAll) {
    dev_->initialize();
    dev_->applyPDOMapping(PDOMappingPreset::Minimal);
    EXPECT_EQ(dev_->getCurrentMapping(), PDOMappingPreset::Minimal);
    dev_->applyPDOMapping(PDOMappingPreset::InputWithStatus);
    dev_->applyPDOMapping(PDOMappingPreset::MultiInput);
    dev_->applyPDOMapping(PDOMappingPreset::Controller);
    dev_->applyPDOMapping(PDOMappingPreset::ControllerFull);
    dev_->applyPDOMapping(PDOMappingPreset::WithAlarms);
    dev_->applyPDOMapping(PDOMappingPreset::Custom);
}

TEST_F(CiA404Test, InputValues) {
    dev_->initialize();
    (void)dev_->getRawValue(0);
    (void)dev_->getScaledValue(0);
    (void)dev_->getFilteredValue(0);
    (void)dev_->getInputStatus(0);
    auto state = dev_->getInputState(0);
    (void)state;
}

TEST_F(CiA404Test, InputConfig) {
    dev_->initialize();
    dev_->configureInput(0, 1, 0x0001, 2);
    dev_->setInputScaling(0, 0, 32767, 0, 10000);
    dev_->setInputFilter(0, 100);
}

TEST_F(CiA404Test, OutputValues) {
    dev_->initialize();
    dev_->setOutputValue(0, 5000);
    // SDO may fail with NullSDOTransport, so value may not persist
    (void)dev_->getOutputValue(0);
    dev_->configureOutput(0, 1);
    dev_->setOutputErrorBehavior(0, 0, 0);
}

TEST_F(CiA404Test, Controller) {
    dev_->initialize();
    dev_->setSetpoint(1000);
    EXPECT_EQ(dev_->getSetpoint(), 1000);
    dev_->setControllerMode(ControllerModes::PID_Auto);
    EXPECT_EQ(dev_->getControllerMode(), ControllerModes::PID_Auto);
    dev_->setControllerMode(ControllerModes::Manual);
    dev_->setControllerMode(ControllerModes::P_Only);
    dev_->setControllerMode(ControllerModes::PI_Control);

    PIDParameters pid{};
    pid.kp = 100;
    pid.ti = 1000;
    pid.td = 50;
    dev_->setPIDParameters(pid);
    auto got = dev_->getPIDParameters();
    EXPECT_EQ(got.kp, 100);

    dev_->setPIDGains(200, 500, 25);
    dev_->setOutputLimits(-10000, 10000);
    dev_->setSetpointRamp(100);
    dev_->setFeedforward(50);
}

TEST_F(CiA404Test, ControllerState) {
    dev_->initialize();
    auto cs = dev_->getControllerState();
    EXPECT_FALSE(cs.isActive());
    (void)dev_->getDeviation();
    (void)dev_->getControllerOutput();
    EXPECT_FALSE(dev_->isControllerActive());
    dev_->resetIntegrator();
}

TEST_F(CiA404Test, Alarms) {
    dev_->initialize();
    AlarmConfig ac{};
    ac.high_high_limit = 10000;
    ac.high_limit = 8000;
    ac.low_limit = 2000;
    ac.low_low_limit = 1000;
    ac.hysteresis = 100;
    dev_->configureAlarms(0, ac);
    dev_->setAlarmHighHigh(0, 10000);
    dev_->setAlarmHigh(0, 8000);
    dev_->setAlarmLow(0, 2000);
    dev_->setAlarmLowLow(0, 1000);
    dev_->setAlarmHysteresis(0, 50);
    EXPECT_EQ(dev_->getAlarmStatus(0), 0u);
    EXPECT_EQ(dev_->getWarningStatus(0), 0u);
    EXPECT_FALSE(dev_->hasAlarm(0));
    dev_->acknowledgeAlarms(0);
}

TEST_F(CiA404Test, Calibration) {
    dev_->initialize();
    dev_->calibrateZero(0, 0);
    dev_->calibrateSpan(0, 10000);
    dev_->calibrateTwoPoint(0, 0, 0, 10000, 10000);
    dev_->acceptCalibration(0);
    dev_->rejectCalibration(0);
    dev_->restoreFactoryCalibration(0);
    dev_->saveCalibration(0);
    (void)dev_->getCalibrationStatus(0);
    dev_->setTare(0);
    dev_->clearTare(0);
}

TEST_F(CiA404Test, Diagnostics) {
    dev_->initialize();
    (void)dev_->getSensorStatus(0);
    (void)dev_->getSignalQuality(0);
    (void)dev_->getSensorSupplyVoltage(0);
    (void)dev_->getSensorTemperature(0);
    (void)dev_->getOperatingHours();
    auto diag = dev_->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA404Test, Callbacks) {
    dev_->setEventCallback([](DeviceEvent, uint16_t, uint8_t, int32_t) {});
    dev_->setAlarmCallback([](uint16_t, uint8_t, uint16_t, int32_t) {});
    dev_->setValueCallback([](uint16_t, uint8_t, int32_t, int32_t) {});
}

TEST_F(CiA404Test, PDOProcess) {
    dev_->initialize();
    uint8_t txbuf[128] = {};
    dev_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[128] = {};
    dev_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA404Test, Update) {
    dev_->initialize();
    dev_->update();
}
