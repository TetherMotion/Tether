/**
 * @file test_CiA404_coverage.cpp
 * @brief Extended CiA404 MeasuringDevice coverage tests
 */
#include <gtest/gtest.h>
#include <memory>
#include "tether/profiles/cia404/CiA404Device.hpp"
#include "tether/profiles/cia404/CiA404Defs.hpp"
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

class CiA404CovTest : public ::testing::Test {
protected:
    void SetUp() override {
        transport = std::make_unique<NullSDOTransport>();
        coe = std::make_unique<EtherCAT::CoE::CoEManager>(1, *transport);
        coe->init();
        dev = std::make_unique<MeasuringDevice>(*coe);
    }
    void TearDown() override {
        dev.reset();
        coe->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport;
    std::unique_ptr<EtherCAT::CoE::CoEManager> coe;
    std::unique_ptr<MeasuringDevice> dev;
};

// --- Process Input ---

TEST_F(CiA404CovTest, GetRawValue) {
    int32_t val = dev->getRawValue(0);
    EXPECT_EQ(val, 0);
}

TEST_F(CiA404CovTest, GetScaledValue) {
    int32_t val = dev->getScaledValue(0);
    (void)val;
}

TEST_F(CiA404CovTest, GetFilteredValue) {
    int32_t val = dev->getFilteredValue(0);
    (void)val;
}

TEST_F(CiA404CovTest, GetInputStatus) {
    auto val = dev->getInputStatus(0);
    (void)val;
}

TEST_F(CiA404CovTest, GetInputState) {
    auto& state = dev->getInputState(0);
    (void)state;
}

TEST_F(CiA404CovTest, ConfigureInput) {
    bool result = dev->configureInput(0, 0, 0, 0);
    (void)result;
}

TEST_F(CiA404CovTest, SetInputScaling) {
    dev->setInputScaling(0, -10000, 10000, -10000, 10000);
}

TEST_F(CiA404CovTest, SetInputFilter) {
    dev->setInputFilter(0, 10);
}

// --- Process Output ---

TEST_F(CiA404CovTest, SetOutputValue) {
    dev->setOutputValue(0, 5000);
}

TEST_F(CiA404CovTest, GetOutputValue) {
    int32_t val = dev->getOutputValue(0);
    (void)val;
}

TEST_F(CiA404CovTest, ConfigureOutput) {
    bool result = dev->configureOutput(0, 0);
    (void)result;
}

TEST_F(CiA404CovTest, SetOutputErrorBehavior) {
    dev->setOutputErrorBehavior(0, 0, 0);
}

// --- PID Controller ---

TEST_F(CiA404CovTest, SetGetSetpoint) {
    dev->setSetpoint(1000);
    int32_t sp = dev->getSetpoint();
    (void)sp;
}

TEST_F(CiA404CovTest, SetControllerMode) {
    dev->setControllerMode(0); // Off
    dev->setControllerMode(1); // Automatic
    dev->setControllerMode(2); // Manual
}

TEST_F(CiA404CovTest, SetPIDParameters) {
    PIDParameters params{};
    params.kp = 1000;
    params.ti = 100;
    params.td = 10;
    dev->setPIDParameters(params);
}

TEST_F(CiA404CovTest, GetPIDParameters) {
    auto params = dev->getPIDParameters();
    (void)params;
}

TEST_F(CiA404CovTest, SetPIDGains) {
    dev->setPIDGains(2000, 500, 100);
}

TEST_F(CiA404CovTest, SetOutputLimits) {
    dev->setOutputLimits(-10000, 10000);
}

TEST_F(CiA404CovTest, SetSetpointRamp) {
    dev->setSetpointRamp(100);
}

TEST_F(CiA404CovTest, SetFeedforward) {
    dev->setFeedforward(1000);
}

TEST_F(CiA404CovTest, GetControllerState) {
    auto& state = dev->getControllerState();
    (void)state;
}

TEST_F(CiA404CovTest, GetDeviation) {
    int32_t val = dev->getDeviation();
    (void)val;
}

TEST_F(CiA404CovTest, GetControllerOutput) {
    int32_t val = dev->getControllerOutput();
    (void)val;
}

TEST_F(CiA404CovTest, IsControllerActive) {
    bool active = dev->isControllerActive();
    EXPECT_FALSE(active);
}

TEST_F(CiA404CovTest, ResetIntegrator) {
    dev->resetIntegrator();
}

// --- Alarms ---

TEST_F(CiA404CovTest, ConfigureAlarms) {
    AlarmConfig cfg{};
    cfg.high_high_limit = 9000;
    cfg.high_limit = 8000;
    cfg.low_limit = 2000;
    cfg.low_low_limit = 1000;
    cfg.hysteresis = 100;
    dev->configureAlarms(0, cfg);
}

TEST_F(CiA404CovTest, SetAlarmThresholds) {
    dev->setAlarmHighHigh(0, 9500);
    dev->setAlarmHigh(0, 8500);
    dev->setAlarmLow(0, 1500);
    dev->setAlarmLowLow(0, 500);
    dev->setAlarmHysteresis(0, 50);
}

TEST_F(CiA404CovTest, GetAlarmStatus) {
    auto status = dev->getAlarmStatus(0);
    (void)status;
}

TEST_F(CiA404CovTest, GetWarningStatus) {
    auto status = dev->getWarningStatus(0);
    (void)status;
}

TEST_F(CiA404CovTest, HasAlarm) {
    EXPECT_FALSE(dev->hasAlarm(0));
}

TEST_F(CiA404CovTest, AcknowledgeAlarms) {
    dev->acknowledgeAlarms(0);
}

// --- Calibration ---

TEST_F(CiA404CovTest, CalibrateZero) {
    bool result = dev->calibrateZero(0, 0);
    (void)result;
}

TEST_F(CiA404CovTest, CalibrateSpan) {
    bool result = dev->calibrateSpan(0, 10000);
    (void)result;
}

TEST_F(CiA404CovTest, CalibrateTwoPoint) {
    bool result = dev->calibrateTwoPoint(0, 0, 0, 10000, 10000);
    (void)result;
}

TEST_F(CiA404CovTest, AcceptCalibration) {
    dev->acceptCalibration(0);
}

TEST_F(CiA404CovTest, RejectCalibration) {
    dev->rejectCalibration(0);
}

TEST_F(CiA404CovTest, RestoreFactoryCalibration) {
    dev->restoreFactoryCalibration(0);
}

TEST_F(CiA404CovTest, SaveCalibration) {
    dev->saveCalibration(0);
}

TEST_F(CiA404CovTest, GetCalibrationStatus) {
    auto status = dev->getCalibrationStatus(0);
    (void)status;
}

TEST_F(CiA404CovTest, SetTare) {
    dev->setTare(0);
}

TEST_F(CiA404CovTest, ClearTare) {
    dev->clearTare(0);
}

// --- Diagnostics ---

TEST_F(CiA404CovTest, GetSensorStatus) {
    auto status = dev->getSensorStatus(0);
    (void)status;
}

TEST_F(CiA404CovTest, GetSensorSupplyVoltage) {
    auto v = dev->getSensorSupplyVoltage(0);
    (void)v;
}

TEST_F(CiA404CovTest, GetSensorTemperature) {
    auto t = dev->getSensorTemperature(0);
    (void)t;
}

TEST_F(CiA404CovTest, GetSignalQuality) {
    auto q = dev->getSignalQuality(0);
    (void)q;
}

TEST_F(CiA404CovTest, GetOperatingHours) {
    auto h = dev->getOperatingHours();
    (void)h;
}

TEST_F(CiA404CovTest, GetDiagnostics) {
    auto diag = dev->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA404CovTest, IsInitialized) {
    EXPECT_FALSE(dev->isInitialized());
}

TEST_F(CiA404CovTest, GetCapabilities) {
    auto& caps = dev->getCapabilities();
    (void)caps;
}

// --- Callbacks ---

TEST_F(CiA404CovTest, SetEventCallback) {
    dev->setEventCallback([](DeviceEvent, uint16_t, uint8_t, int32_t) {});
}

TEST_F(CiA404CovTest, SetAlarmCallback) {
    dev->setAlarmCallback([](uint16_t, uint8_t, uint16_t, int32_t) {});
}

TEST_F(CiA404CovTest, SetValueCallback) {
    dev->setValueCallback([](uint16_t, uint8_t, int32_t, int32_t) {});
}

// --- PDO ---

TEST_F(CiA404CovTest, ApplyPDOMappingMinimal) {
    dev->applyPDOMapping(PDOMappingPreset::Minimal);
}

TEST_F(CiA404CovTest, GetCurrentMapping) {
    auto m = dev->getCurrentMapping();
    (void)m;
}

TEST_F(CiA404CovTest, ProcessTxPDO) {
    uint8_t data[16] = {};
    dev->processTxPDO(data, 16);
}

TEST_F(CiA404CovTest, PrepareRxPDO) {
    uint8_t data[16] = {};
    size_t len = dev->prepareRxPDO(data, 16);
    EXPECT_LE(len, 16u);
}

TEST_F(CiA404CovTest, Update) {
    dev->update();
}

// --- Free functions ---

TEST_F(CiA404CovTest, GetDeviceEventName) {
    auto* name = getDeviceEventName(DeviceEvent::ValueUpdated);
    EXPECT_NE(name, nullptr);
}

TEST_F(CiA404CovTest, GetInputRangeName) {
    auto* name = getInputRangeName(0);
    EXPECT_NE(name, nullptr);
}

TEST_F(CiA404CovTest, GetControllerModeName) {
    auto* name = getControllerModeName(0);
    EXPECT_NE(name, nullptr);
}

TEST_F(CiA404CovTest, Q16Conversions) {
    float f = q16ToFloat(65536);
    EXPECT_NEAR(f, 1.0f, 0.001f);
    int32_t q = floatToQ16(1.0f);
    EXPECT_EQ(q, 65536);
}

TEST_F(CiA404CovTest, Initialize) {
    bool result = dev->initialize();
    (void)result;
}
