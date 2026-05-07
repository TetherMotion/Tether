/**
 * @file test_CiA410Slave_coverage.cpp
 * @brief Coverage tests for CiA410Slave.cpp — exercises factory functions,
 *        getters/setters, calibration, alarms, filter, simulate, and more.
 */

#include "tether/slave/profiles/CiA410Slave.hpp"
#include "tether/profiles/cia410/CiA410Defs.hpp"
#include <gtest/gtest.h>
#include <memory>

using namespace EtherCAT::slave;

// ============================================================================
// Factory functions
// ============================================================================

TEST(CiA410SlaveCovTest, CreateSingleAxisInclinometer) {
    auto slave = createSingleAxisInclinometer();
    ASSERT_NE(slave, nullptr);
    EXPECT_STREQ(slave->getProfileName(), "CiA 410");
    EXPECT_EQ(slave->getDeviceType(), 0x0000019Au);
}

TEST(CiA410SlaveCovTest, CreateDualAxisInclinometer) {
    auto slave = createDualAxisInclinometer();
    ASSERT_NE(slave, nullptr);
    EXPECT_STREQ(slave->getProfileName(), "CiA 410");
}

TEST(CiA410SlaveCovTest, CreateFromConfig) {
    CiA410SlaveConfig cfg;
    cfg.type = InclinometerType::DualAxis;
    cfg.measurementRangeMin = -45000;
    cfg.measurementRangeMax = 45000;
    cfg.resolution = 1;
    cfg.hasTemperatureSensor = true;
    auto slave = createCiA410Slave(cfg);
    ASSERT_NE(slave, nullptr);
}

// ============================================================================
// Construction with various configs
// ============================================================================

TEST(CiA410SlaveCovTest, ConstructSingleAxis) {
    CiA410SlaveConfig cfg;
    cfg.type = InclinometerType::SingleAxis;
    CiA410Slave slave(cfg);
    EXPECT_STREQ(slave.getProfileName(), "CiA 410");
}

TEST(CiA410SlaveCovTest, ConstructDualAxis) {
    CiA410SlaveConfig cfg;
    cfg.type = InclinometerType::DualAxis;
    CiA410Slave slave(cfg);
    EXPECT_EQ(slave.getDeviceType(), 0x0000019Au);
}

TEST(CiA410SlaveCovTest, ConstructWithNoTemp) {
    CiA410SlaveConfig cfg;
    cfg.hasTemperatureSensor = false;
    CiA410Slave slave(cfg);
    // Just exercises the constructor path
}

// ============================================================================
// Inclination getters/setters
// ============================================================================

TEST(CiA410SlaveCovTest, SetAndGetInclination) {
    auto slave = createDualAxisInclinometer();
    slave->setInclination(12345, -6789);
    // Raw values are set; after simulate they get processed
    // Before simulate, getInclination returns processed values
    // Let's exercise simulate to process them
    slave->simulate(1000);
    // The exact values depend on filter/calibration — just check type
    int32_t x = slave->getInclinationX();
    int32_t y = slave->getInclinationY();
    (void)x;
    (void)y;
}

TEST(CiA410SlaveCovTest, SetInclinationZero) {
    auto slave = createDualAxisInclinometer();
    slave->setInclination(0, 0);
    slave->simulate(1000);
    EXPECT_EQ(slave->getInclinationX(), 0);
    EXPECT_EQ(slave->getInclinationY(), 0);
}

// ============================================================================
// Temperature
// ============================================================================

TEST(CiA410SlaveCovTest, SetAndGetTemperature) {
    auto slave = createDualAxisInclinometer();
    slave->setTemperature(500); // 50.0°C
    EXPECT_EQ(slave->getTemperature(), 500);
    slave->setTemperature(-100); // -10.0°C
    EXPECT_EQ(slave->getTemperature(), -100);
}

// ============================================================================
// Operating status
// ============================================================================

TEST(CiA410SlaveCovTest, OperatingStatusDefault) {
    auto slave = createDualAxisInclinometer();
    auto status = slave->getOperatingStatus();
    // Default has Ready bit set (0x0001)
    EXPECT_NE(status & 0x0001, 0);
    EXPECT_TRUE(slave->isReady());
}

TEST(CiA410SlaveCovTest, IsCalibrating) {
    auto slave = createDualAxisInclinometer();
    EXPECT_FALSE(slave->isCalibrating());
    slave->startCalibration();
    EXPECT_TRUE(slave->isCalibrating());
}

// ============================================================================
// Calibration
// ============================================================================

TEST(CiA410SlaveCovTest, StartCalibration) {
    auto slave = createDualAxisInclinometer();
    slave->startCalibration();
    EXPECT_TRUE(slave->isCalibrating());
    auto status = slave->getOperatingStatus();
    EXPECT_NE(status & 0x0010, 0); // CalibrationActive bit
}

TEST(CiA410SlaveCovTest, CalibrationProcess_NotEnoughTime) {
    auto slave = createDualAxisInclinometer();
    slave->setInclination(1000, 2000);
    slave->startCalibration();
    // Simulate for less than 1 second — calibration should not complete
    for (int i = 0; i < 100; ++i) {
        slave->simulate(1000000); // 1ms each, total 100ms
    }
    EXPECT_TRUE(slave->isCalibrating());
}

TEST(CiA410SlaveCovTest, CalibrationProcess_Completes) {
    auto slave = createDualAxisInclinometer();
    slave->setInclination(1000, 2000);
    slave->startCalibration();
    // Simulate for more than 1 second to complete calibration
    // 1e9 ns = 1 second
    for (int i = 0; i < 200; ++i) {
        slave->simulate(10000000); // 10ms each, total 2s
    }
    EXPECT_FALSE(slave->isCalibrating());
}

TEST(CiA410SlaveCovTest, SetCalibrationOffset) {
    auto slave = createDualAxisInclinometer();
    slave->setCalibrationOffset(100, -200);
    slave->setInclination(1000, 2000);
    slave->simulate(1000);
    // Calibration offset should be applied
    int32_t x = slave->getInclinationX();
    int32_t y = slave->getInclinationY();
    (void)x;
    (void)y;
}

// ============================================================================
// Alarms
// ============================================================================

TEST(CiA410SlaveCovTest, AlarmDefaultInactive) {
    auto slave = createDualAxisInclinometer();
    EXPECT_FALSE(slave->isAlarmActive());
    EXPECT_EQ(slave->getAlarmStatus(), 0);
}

TEST(CiA410SlaveCovTest, AlarmTriggered_X) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(0); // No filter, alpha=1
    slave->setAlarmLimits(-1000, 1000, -90000, 90000);
    slave->setInclination(5000, 0); // X out of range
    slave->simulate(1000);
    EXPECT_TRUE(slave->isAlarmActive());
    EXPECT_NE(slave->getAlarmStatus() & 0x01, 0);
}

TEST(CiA410SlaveCovTest, AlarmTriggered_Y) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(0); // No filter
    slave->setAlarmLimits(-90000, 90000, -1000, 1000);
    slave->setInclination(0, 5000); // Y out of range
    slave->simulate(1000);
    EXPECT_TRUE(slave->isAlarmActive());
    EXPECT_NE(slave->getAlarmStatus() & 0x02, 0);
}

TEST(CiA410SlaveCovTest, AlarmTriggered_Both) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(0); // No filter
    slave->setAlarmLimits(-1000, 1000, -1000, 1000);
    slave->setInclination(5000, 5000);
    slave->simulate(1000);
    EXPECT_TRUE(slave->isAlarmActive());
    EXPECT_NE(slave->getAlarmStatus() & 0x03, 0);
}

TEST(CiA410SlaveCovTest, AlarmClearedWhenInRange) {
    auto slave = createDualAxisInclinometer();
    slave->setAlarmLimits(-10000, 10000, -10000, 10000);
    slave->setInclination(500, 500); // Within range
    slave->simulate(1000);
    EXPECT_FALSE(slave->isAlarmActive());
}

TEST(CiA410SlaveCovTest, SetAlarmLimits) {
    auto slave = createDualAxisInclinometer();
    slave->setAlarmLimits(-5000, 5000, -3000, 3000);
    // Just exercises the setter
}

// ============================================================================
// Filter
// ============================================================================

TEST(CiA410SlaveCovTest, GetFilterSettingDefault) {
    CiA410SlaveConfig cfg;
    cfg.filterSetting = 4;
    auto slave = createCiA410Slave(cfg);
    EXPECT_EQ(slave->getFilterSetting(), 4);
}

TEST(CiA410SlaveCovTest, SetFilterSetting) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(2);
    EXPECT_EQ(slave->getFilterSetting(), 2);
}

TEST(CiA410SlaveCovTest, SetFilterSetting_Masked) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(0xFF); // Should mask to 0x07
    EXPECT_EQ(slave->getFilterSetting(), 7);
}

TEST(CiA410SlaveCovTest, FilterApplied_LowSetting) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(0); // No filtering (alpha = 1.0)
    slave->setInclination(10000, 20000);
    slave->simulate(1000);
    // With no filter, output should match input more closely
}

TEST(CiA410SlaveCovTest, FilterApplied_HighSetting) {
    auto slave = createDualAxisInclinometer();
    slave->setFilterSetting(7); // Heavy filtering
    slave->setInclination(10000, 20000);
    slave->simulate(1000);
    // With heavy filter, output changes slowly
}

// ============================================================================
// Simulate — various paths
// ============================================================================

TEST(CiA410SlaveCovTest, Simulate_Basic) {
    auto slave = createDualAxisInclinometer();
    slave->setInclination(1000, 2000);
    slave->simulate(1000000); // 1ms
}

TEST(CiA410SlaveCovTest, Simulate_Multiple) {
    auto slave = createDualAxisInclinometer();
    for (int i = 0; i < 100; ++i) {
        slave->setInclination(i * 10, -i * 5);
        slave->simulate(1000000);
    }
}

TEST(CiA410SlaveCovTest, Simulate_WithCallback) {
    auto slave = createDualAxisInclinometer();
    int callCount = 0;
    slave->setInclinationCallback([&callCount](int32_t& x, int32_t& y) {
        x = 500;
        y = -500;
        callCount++;
    });
    slave->simulate(1000000);
    EXPECT_EQ(callCount, 1);
}

TEST(CiA410SlaveCovTest, Simulate_CallbackModifiesValues) {
    auto slave = createDualAxisInclinometer();
    slave->setInclinationCallback([](int32_t& x, int32_t& y) {
        x = 42000;
        y = -42000;
    });
    slave->simulate(1000000);
    // Values have been modified by callback and then processed
}

TEST(CiA410SlaveCovTest, Simulate_ZeroDelta) {
    auto slave = createDualAxisInclinometer();
    slave->simulate(0);
}

// ============================================================================
// PDO update/process (stubs but still exercise them)
// ============================================================================

TEST(CiA410SlaveCovTest, UpdateTxPDO) {
    auto slave = createDualAxisInclinometer();
    slave->updateTxPDO();
}

TEST(CiA410SlaveCovTest, ProcessRxPDO) {
    auto slave = createDualAxisInclinometer();
    slave->processRxPDO();
}

// ============================================================================
// CiA410Defs — conversion functions
// ============================================================================

TEST(CiA410SlaveCovTest, MillidegToDeg) {
    EXPECT_NEAR(CiA410::millidegToDeg(45000), 45.0f, 0.001f);
    EXPECT_NEAR(CiA410::millidegToDeg(-90000), -90.0f, 0.001f);
    EXPECT_NEAR(CiA410::millidegToDeg(0), 0.0f, 0.001f);
}

TEST(CiA410SlaveCovTest, DegToMillideg) {
    EXPECT_EQ(CiA410::degToMillideg(45.0f), 45000);
    EXPECT_EQ(CiA410::degToMillideg(-90.0f), -90000);
    EXPECT_EQ(CiA410::degToMillideg(0.0f), 0);
}

TEST(CiA410SlaveCovTest, MilligToG) {
    EXPECT_NEAR(CiA410::milligToG(1000), 1.0f, 0.001f);
    EXPECT_NEAR(CiA410::milligToG(-500), -0.5f, 0.001f);
}

TEST(CiA410SlaveCovTest, GToMillig) {
    EXPECT_EQ(CiA410::gToMillig(1.0f), 1000);
    EXPECT_EQ(CiA410::gToMillig(-0.5f), -500);
}

TEST(CiA410SlaveCovTest, RawToDegsPerSec) {
    float dps = CiA410::rawToDegsPerSec(1000);
    EXPECT_GT(dps, 0.0f);
}

TEST(CiA410SlaveCovTest, DegsPerSecToRaw) {
    float dps = CiA410::rawToDegsPerSec(1000);
    int16_t raw = CiA410::degsPerSecToRaw(dps);
    EXPECT_EQ(raw, 1000);
}

TEST(CiA410SlaveCovTest, RawToTempC) {
    EXPECT_NEAR(CiA410::rawToTempC(250), 25.0f, 0.1f);
    EXPECT_NEAR(CiA410::rawToTempC(0), 0.0f, 0.1f);
}

TEST(CiA410SlaveCovTest, TempCToRaw) {
    EXPECT_EQ(CiA410::tempCToRaw(25.0f), 250);
    EXPECT_EQ(CiA410::tempCToRaw(0.0f), 0);
}

// ============================================================================
// InclinometerType enum
// ============================================================================

TEST(CiA410SlaveCovTest, InclinometerTypeValues) {
    EXPECT_EQ(static_cast<uint8_t>(InclinometerType::SingleAxis), 1);
    EXPECT_EQ(static_cast<uint8_t>(InclinometerType::DualAxis), 2);
}

// ============================================================================
// CiA410SlaveConfig defaults
// ============================================================================

TEST(CiA410SlaveCovTest, ConfigDefaults) {
    CiA410SlaveConfig cfg;
    EXPECT_EQ(cfg.type, InclinometerType::DualAxis);
    EXPECT_EQ(cfg.measurementRangeMin, -90000);
    EXPECT_EQ(cfg.measurementRangeMax, 90000);
    EXPECT_EQ(cfg.resolution, 10);
    EXPECT_TRUE(cfg.hasTemperatureSensor);
    EXPECT_EQ(cfg.filterSetting, 4);
    EXPECT_TRUE(cfg.supportsCalibration);
}
