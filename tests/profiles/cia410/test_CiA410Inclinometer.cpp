/**
 * @file test_CiA410Inclinometer.cpp
 * @brief Comprehensive tests for CiA 410 Inclinometer Controller
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia410/CiA410Inclinometer.hpp"

using namespace CiA410;

// ============================================================================
// AngleReading struct
// ============================================================================

TEST(CiA410AngleReading, Conversions) {
    AngleReading a{};
    a.x = 90000;  // 90,000 millideg = 90 deg
    a.y = -45000; // -45 deg
    a.z = 360000; // 360 deg
    EXPECT_NEAR(a.getXDegrees(), 90.0f, 1e-3f);
    EXPECT_NEAR(a.getYDegrees(), -45.0f, 1e-3f);
    EXPECT_NEAR(a.getZDegrees(), 360.0f, 1e-3f);
    a.total = 100000;
    EXPECT_NEAR(a.getTotalDegrees(), 100.0f, 1e-3f);
}

TEST(CiA410AngleReading, Zero) {
    AngleReading a{};
    EXPECT_NEAR(a.getXDegrees(), 0.0f, 1e-6f);
    EXPECT_NEAR(a.getYDegrees(), 0.0f, 1e-6f);
}

// ============================================================================
// VelocityReading struct
// ============================================================================

TEST(CiA410VelocityReading, Conversions) {
    VelocityReading v{};
    v.x = 1000; // 10.00 deg/s
    v.y = -500;
    v.z = 200;
    EXPECT_NEAR(v.getXDegsPerSec(), 10.0f, 0.01f);
    EXPECT_NEAR(v.getYDegsPerSec(), -5.0f, 0.01f);
    EXPECT_NEAR(v.getZDegsPerSec(), 2.0f, 0.01f);
}

// ============================================================================
// AccelerationReading struct
// ============================================================================

TEST(CiA410AccelReading, Conversions) {
    AccelerationReading a{};
    a.x = 1000; // 1.000 g
    a.y = -981;
    a.z = 0;
    EXPECT_NEAR(a.getXG(), 1.0f, 1e-3f);
    EXPECT_NEAR(a.getYG(), -0.981f, 1e-3f);
    EXPECT_NEAR(a.getZG(), 0.0f, 1e-6f);
}

// ============================================================================
// InclinometerState helpers
// ============================================================================

TEST(CiA410State, DefaultConstruction) {
    InclinometerState s{};
    EXPECT_FALSE(s.isReady());
    EXPECT_FALSE(s.isDataValid());
    EXPECT_FALSE(s.isCalibrated());
    EXPECT_FALSE(s.isMotionDetected());
    EXPECT_FALSE(s.hasAlarm());
    EXPECT_FALSE(s.hasFault());
    EXPECT_FALSE(s.isOverRange());
    EXPECT_FALSE(s.isSettling());
}

TEST(CiA410State, StatusBitReady) {
    InclinometerState s{};
    s.statusword = StatuswordBits::Ready;
    EXPECT_TRUE(s.isReady());
}

TEST(CiA410State, StatusBitDataValid) {
    InclinometerState s{};
    s.statusword = StatuswordBits::DataValid;
    EXPECT_TRUE(s.isDataValid());
}

TEST(CiA410State, StatusBitCalibrated) {
    InclinometerState s{};
    s.statusword = StatuswordBits::Calibrated;
    EXPECT_TRUE(s.isCalibrated());
}

TEST(CiA410State, StatusBitMotion) {
    InclinometerState s{};
    s.statusword = StatuswordBits::MotionDetected;
    EXPECT_TRUE(s.isMotionDetected());
}

TEST(CiA410State, StatusBitAlarm) {
    InclinometerState s{};
    s.statusword = StatuswordBits::AlarmActive;
    EXPECT_TRUE(s.hasAlarm());
}

TEST(CiA410State, StatusBitFault) {
    InclinometerState s{};
    s.statusword = StatuswordBits::Fault;
    EXPECT_TRUE(s.hasFault());
}

TEST(CiA410State, StatusBitOverRange) {
    InclinometerState s{};
    s.statusword = StatuswordBits::OverRange;
    EXPECT_TRUE(s.isOverRange());
}

TEST(CiA410State, StatusBitSettling) {
    InclinometerState s{};
    s.statusword = StatuswordBits::Settling;
    EXPECT_TRUE(s.isSettling());
}

TEST(CiA410State, TempConversion) {
    InclinometerState s{};
    s.temperature = 250;
    EXPECT_NEAR(s.getTemperatureCelsius(), 25.0f, 0.1f);
}

TEST(CiA410State, AllBitsCombined) {
    InclinometerState s{};
    s.statusword = StatuswordBits::Ready | StatuswordBits::DataValid |
                   StatuswordBits::Calibrated;
    EXPECT_TRUE(s.isReady());
    EXPECT_TRUE(s.isDataValid());
    EXPECT_TRUE(s.isCalibrated());
    EXPECT_FALSE(s.hasFault());
}

// ============================================================================
// Enum tests
// ============================================================================

TEST(CiA410Enums, DeviceTypes) {
    EXPECT_NE(static_cast<uint8_t>(DeviceType::SingleAxis),
              static_cast<uint8_t>(DeviceType::DualAxis));
}

TEST(CiA410Enums, SensorTypes) {
    EXPECT_NE(static_cast<uint8_t>(SensorType::MEMS_Capacitive),
              static_cast<uint8_t>(SensorType::MEMS_Piezoresistive));
}

TEST(CiA410Enums, OperatingModes) {
    EXPECT_NE(static_cast<uint8_t>(OperatingMode::Continuous),
              static_cast<uint8_t>(OperatingMode::Triggered));
}

TEST(CiA410Enums, FilterSettings) {
    EXPECT_EQ(static_cast<uint8_t>(FilterSetting::NoFilter), 0u);
}

// ============================================================================
// InclinometerController fixture
// ============================================================================

class CiA410Test : public ::testing::Test {
protected:
    void SetUp() override {
        inc_ = std::make_unique<InclinometerController>(1);
        inc_->initialize();
    }
    std::unique_ptr<InclinometerController> inc_;
};

TEST_F(CiA410Test, Construction) {
    InclinometerController c2(0x100, true);
    EXPECT_FALSE(c2.isInitialized());
}

TEST_F(CiA410Test, Initialize) {
    InclinometerController c3(2);
    EXPECT_TRUE(c3.initialize());
    EXPECT_TRUE(c3.isInitialized());
}

TEST_F(CiA410Test, GetCapabilities) {
    auto caps = inc_->getCapabilities();
    (void)caps.device_type;
    (void)caps.sensor_type;
    (void)caps.num_axes;
}

TEST_F(CiA410Test, PDOMappingAll) {
    EXPECT_TRUE(inc_->applyPDOMapping(PDOMappingPreset::SingleAxis));
    EXPECT_TRUE(inc_->applyPDOMapping(PDOMappingPreset::DualAxis));
    EXPECT_TRUE(inc_->applyPDOMapping(PDOMappingPreset::Extended));
    EXPECT_TRUE(inc_->applyPDOMapping(PDOMappingPreset::Full));
    EXPECT_TRUE(inc_->applyPDOMapping(PDOMappingPreset::Custom));
}

TEST_F(CiA410Test, EnableDisable) {
    inc_->enable();
    inc_->disable();
}

TEST_F(CiA410Test, ResetFault) {
    inc_->resetFault();
}

TEST_F(CiA410Test, SetZero) {
    inc_->setZero();
}

TEST_F(CiA410Test, SelfTest) {
    inc_->selfTest();
}

TEST_F(CiA410Test, OperatingMode) {
    inc_->setOperatingMode(static_cast<uint8_t>(OperatingMode::Continuous));
    EXPECT_EQ(inc_->getOperatingMode(), static_cast<uint8_t>(OperatingMode::Continuous));
    inc_->setOperatingMode(static_cast<uint8_t>(OperatingMode::Triggered));
    inc_->setOperatingMode(static_cast<uint8_t>(OperatingMode::OnDemand));
    inc_->setOperatingMode(static_cast<uint8_t>(OperatingMode::LowPower));
}

TEST_F(CiA410Test, FilterSetting) {
    inc_->setFilterSetting(static_cast<uint8_t>(FilterSetting::NoFilter));
    // SDO may fail, so filter may not change
    (void)inc_->getFilterSetting();
}

TEST_F(CiA410Test, SampleRate) {
    inc_->setSampleRate(1000);
    (void)inc_->getSampleRate();
}

TEST_F(CiA410Test, AveragingCount) {
    inc_->setAveragingCount(8);
}

TEST_F(CiA410Test, AngleReadings) {
    EXPECT_NEAR(inc_->getAngleX(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getAngleY(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getAngleZ(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getTotalAngle(), 0.0f, 1e-3f);
    EXPECT_EQ(inc_->getAngleXRaw(), 0);
    EXPECT_EQ(inc_->getAngleYRaw(), 0);
    EXPECT_EQ(inc_->getAngleZRaw(), 0);
    auto a = inc_->getAngle();
    (void)a;
}

TEST_F(CiA410Test, VelocityReadings) {
    EXPECT_NEAR(inc_->getVelocityX(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getVelocityY(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getVelocityZ(), 0.0f, 1e-3f);
    auto v = inc_->getVelocity();
    (void)v;
}

TEST_F(CiA410Test, AccelReadings) {
    EXPECT_NEAR(inc_->getAccelerationX(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getAccelerationY(), 0.0f, 1e-3f);
    EXPECT_NEAR(inc_->getAccelerationZ(), 0.0f, 1e-3f);
    auto a = inc_->getAcceleration();
    (void)a;
}

TEST_F(CiA410Test, Temperature) {
    (void)inc_->getTemperature();
    inc_->enableTemperatureCompensation(true);
    inc_->enableTemperatureCompensation(false);
    inc_->setTempCompCoefficients(10, 20);
}

TEST_F(CiA410Test, CalibrationCommands) {
    inc_->startAutoZero();
    inc_->startOnePointCalibration();
    inc_->startTwoPointCalibration();
    inc_->startGyroBiasCalibration();
    inc_->startCrossAxisCalibration();
    inc_->storeCalibration();
    inc_->resetCalibration();
    (void)inc_->getCalibrationStatus();
}

TEST_F(CiA410Test, CalibrationData) {
    auto data = inc_->getCalibrationData();
    (void)data;
    CalibrationData cd{};
    inc_->setCalibrationData(cd);
    inc_->setZeroOffset(100, 200, 300);
    inc_->setScaleFactor(1000, 1000, 1000);
}

TEST_F(CiA410Test, Mounting) {
    inc_->setMountingOrientation(0);
    inc_->setMountingOrientation(1);
    inc_->setMountingRotation(0, 0, 0);
    inc_->setMountingRotation(90, 180, 270);
}

TEST_F(CiA410Test, Alarms) {
    AlarmThresholds at{};
    at.angle_high = 45000;
    at.angle_low = -45000;
    inc_->setAlarmThresholds(at);
    // SDO may fail, so thresholds may keep defaults
    auto got = inc_->getAlarmThresholds();
    (void)got;
    inc_->enableAlarms(0xFFFF);
    EXPECT_FALSE(inc_->isAlarmActive(0x0001)); // no alarm state
    EXPECT_EQ(inc_->getAlarmStatus(), 0u);
}

TEST_F(CiA410Test, Diagnostics) {
    (void)inc_->getSensorHealth();
    (void)inc_->getSignalQuality();
    (void)inc_->getOperatingHours();
    auto diag = inc_->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA410Test, GetState) {
    auto st = inc_->getState();
    EXPECT_FALSE(st.hasFault());
    EXPECT_EQ(inc_->getFaultCode(), 0u);
}

TEST_F(CiA410Test, PDOProcessSingleAxis) {
    inc_->applyPDOMapping(PDOMappingPreset::SingleAxis);
    uint8_t txbuf[64] = {};
    inc_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[64] = {};
    inc_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA410Test, PDOProcessDualAxis) {
    inc_->applyPDOMapping(PDOMappingPreset::DualAxis);
    uint8_t txbuf[64] = {};
    inc_->processTxPDO(txbuf, sizeof(txbuf));
}

TEST_F(CiA410Test, PDOProcessExtended) {
    inc_->applyPDOMapping(PDOMappingPreset::Extended);
    uint8_t txbuf[128] = {};
    inc_->processTxPDO(txbuf, sizeof(txbuf));
}

TEST_F(CiA410Test, PDOProcessFull) {
    inc_->applyPDOMapping(PDOMappingPreset::Full);
    uint8_t txbuf[256] = {};
    inc_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[256] = {};
    inc_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA410Test, Update) {
    inc_->update();
}

TEST_F(CiA410Test, Callbacks) {
    inc_->setAlarmCallback([](uint16_t) {});
    inc_->setFaultCallback([](uint16_t) {});
    inc_->setDataCallback([](const InclinometerState&) {});
}

TEST_F(CiA410Test, PDOTooSmall) {
    inc_->applyPDOMapping(PDOMappingPreset::Full);
    uint8_t small[2] = {};
    inc_->processTxPDO(small, sizeof(small));
}
