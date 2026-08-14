/**
 * @file test_klipper_e3_features.cpp
 * @brief Tests for E3 features: [printer] config, kinematics transforms,
 *        18 new config sections, Spoolman proxy, and 4 new printer objects.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/kinematics/DeltaPrinter.hpp"
#include "tether/klipper/config/ConfigParser.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

using namespace tether::klipper::klippy;
using namespace tether::klipper::motion;
using namespace tether::klipper::config;

// ============================================================================
// Helper: create a unique socket path
// ============================================================================
static std::string uniqueSocketPath() {
    return "/tmp/tether_test_e3_" + std::to_string(getpid()) + ".sock";
}

// ============================================================================
// Helper: create a temporary config file
// ============================================================================
static std::string createTempConfig(const std::string& content) {
    std::string path = "/tmp/tether_test_e3_cfg_" + std::to_string(getpid()) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

// ============================================================================
// E3: [printer] config section tests
// ============================================================================

class E3PrinterConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        socketPath = uniqueSocketPath();
        UdsServerConfig udsCfg;
        udsCfg.socketPath = socketPath;
        KlippyInstanceConfig cfg;
        cfg.udsConfig = udsCfg;
        instance = std::make_unique<KlippyInstance>(cfg);
    }

    void TearDown() override {
        instance.reset();
        ::unlink(socketPath.c_str());
        if (!configPath.empty()) {
            std::filesystem::remove(configPath);
        }
    }

    std::string socketPath;
    std::string configPath;
    std::unique_ptr<KlippyInstance> instance;
};

TEST_F(E3PrinterConfigTest, PrinterSectionDefaults) {
    configPath = createTempConfig(R"(
[printer]
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, Kinematics::Cartesian);
    EXPECT_NEAR(s.maxVelocity, 3000.0, 1e-9);
    EXPECT_NEAR(s.maxAccel, 3000.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, PrinterSectionCartesian) {
    configPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 5000
max_accel = 2000
max_accel_to_decel = 1000
square_corner_velocity = 8.0
max_z_velocity = 100
max_z_accel = 50
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, Kinematics::Cartesian);
    EXPECT_NEAR(s.maxVelocity, 5000.0, 1e-9);
    EXPECT_NEAR(s.maxAccel, 2000.0, 1e-9);
    EXPECT_NEAR(s.maxAccelToDecel, 1000.0, 1e-9);
    EXPECT_NEAR(s.squareCornerVelocity, 8.0, 1e-9);
    EXPECT_NEAR(s.maxZVelocity, 100.0, 1e-9);
    EXPECT_NEAR(s.maxZAccel, 50.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, PrinterSectionCoreXY) {
    configPath = createTempConfig(R"(
[printer]
kinematics = corexy
max_velocity = 4000
max_accel = 5000
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, Kinematics::CoreXY);
    EXPECT_NEAR(s.maxVelocity, 4000.0, 1e-9);
    EXPECT_NEAR(s.maxAccel, 5000.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, PrinterSectionDelta) {
    configPath = createTempConfig(R"(
[printer]
kinematics = delta
max_velocity = 2000
max_accel = 1500
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, Kinematics::Delta);
}

TEST_F(E3PrinterConfigTest, PrinterSectionCoreXZ) {
    configPath = createTempConfig(R"(
[printer]
kinematics = corexz
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    EXPECT_EQ(instance->settings().kinematics, Kinematics::CoreXZ);
}

TEST_F(E3PrinterConfigTest, KinematicsStringConversion) {
    EXPECT_EQ(kinematicsToString(Kinematics::Cartesian), "cartesian");
    EXPECT_EQ(kinematicsToString(Kinematics::CoreXY), "corexy");
    EXPECT_EQ(kinematicsToString(Kinematics::CoreXZ), "corexz");
    EXPECT_EQ(kinematicsToString(Kinematics::Delta), "delta");
    EXPECT_EQ(kinematicsFromString("cartesian"), Kinematics::Cartesian);
    EXPECT_EQ(kinematicsFromString("corexy"), Kinematics::CoreXY);
    EXPECT_EQ(kinematicsFromString("delta"), Kinematics::Delta);
    EXPECT_EQ(kinematicsFromString("unknown"), Kinematics::Cartesian);
}

// ============================================================================
// E3: New config sections tests
// ============================================================================

TEST_F(E3PrinterConfigTest, SafeZHomeSection) {
    configPath = createTempConfig(R"(
[safe_z_home]
home_xy_position = 100,100
z_hop = 10
z_hop_speed = 5
xy_home_speed = 50
move_to_previous = True
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_EQ(s.safeZHomeXYPosition, "100,100");
    EXPECT_NEAR(s.safeZHomeZHop, 10.0, 1e-9);
    EXPECT_NEAR(s.safeZHomeZHopSpeed, 5.0, 1e-9);
    EXPECT_NEAR(s.safeZHomeXYHomeSpeed, 50.0, 1e-9);
    EXPECT_TRUE(s.safeZHomeMoveToPrevious);
}

TEST_F(E3PrinterConfigTest, IdleTimeoutSection) {
    configPath = createTempConfig(R"(
[idle_timeout]
timeout = 300
gcode = TURN_OFF_HEATERS
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_NEAR(s.idleTimeout, 300.0, 1e-9);
    EXPECT_EQ(s.idleTimeoutGcode, "TURN_OFF_HEATERS");
}

TEST_F(E3PrinterConfigTest, PauseResumeSection) {
    configPath = createTempConfig(R"(
[pause_resume]
recover_from_subtract = True
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_TRUE(s.pauseResumeEnabled);
    EXPECT_TRUE(s.pauseResumeRecoverFromSubtract);
}

TEST_F(E3PrinterConfigTest, DisplayStatusSection) {
    configPath = createTempConfig(R"(
[display_status]
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    EXPECT_TRUE(instance->settings().displayStatusEnabled);
}

TEST_F(E3PrinterConfigTest, OutputPinSection) {
    configPath = createTempConfig(R"(
[output_pin led]
pin = PA0
value = 0.5
pwm = True
cycle_time = 0.01
scale = 1.0
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.outputPins.size(), 1u);
    ASSERT_TRUE(s.outputPins.count("led"));
    EXPECT_EQ(s.outputPins.at("led").pin, "PA0");
    EXPECT_NEAR(s.outputPins.at("led").value, 0.5, 1e-9);
    EXPECT_TRUE(s.outputPins.at("led").pwm);
    EXPECT_NEAR(s.outputPins.at("led").cycleTime, 0.01, 1e-9);
}

TEST_F(E3PrinterConfigTest, ServoSection) {
    configPath = createTempConfig(R"(
[servo my_servo]
pin = PA1
minimum_pulse_width = 0.001
maximum_pulse_width = 0.002
minimum_angle = 0
maximum_angle = 180
initial_angle = 90
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.servos.size(), 1u);
    ASSERT_TRUE(s.servos.count("my_servo"));
    EXPECT_EQ(s.servos.at("my_servo").pin, "PA1");
    EXPECT_NEAR(s.servos.at("my_servo").minPulseWidth, 0.001, 1e-9);
    EXPECT_NEAR(s.servos.at("my_servo").maxPulseWidth, 0.002, 1e-9);
    EXPECT_NEAR(s.servos.at("my_servo").initialAngle, 90.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, TemperatureSensorSection) {
    configPath = createTempConfig(R"(
[temperature_sensor chamber]
sensor_type = NTC 100K
sensor_pin = PA2
min_temp = 0
max_temp = 80
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.temperatureSensors.size(), 1u);
    ASSERT_TRUE(s.temperatureSensors.count("chamber"));
    EXPECT_EQ(s.temperatureSensors.at("chamber").sensorType, "NTC 100K");
    EXPECT_EQ(s.temperatureSensors.at("chamber").sensorPin, "PA2");
    EXPECT_NEAR(s.temperatureSensors.at("chamber").maxTemp, 80.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, TemperatureFanSection) {
    configPath = createTempConfig(R"(
[temperature_fan exhaust]
pin = PA3
sensor_type = NTC 100K
sensor_pin = PA4
max_power = 0.8
target_temp = 40
min_temp = 0
max_temp = 100
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.temperatureFans.size(), 1u);
    ASSERT_TRUE(s.temperatureFans.count("exhaust"));
    EXPECT_NEAR(s.temperatureFans.at("exhaust").maxPower, 0.8, 1e-9);
    EXPECT_NEAR(s.temperatureFans.at("exhaust").targetTemp, 40.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, HeaterFanSection) {
    configPath = createTempConfig(R"(
[heater_fan hotend_fan]
pin = PA5
max_power = 1.0
heater = extruder
heater_temp = 50.0
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.heaterFans.size(), 1u);
    ASSERT_TRUE(s.heaterFans.count("hotend_fan"));
    EXPECT_EQ(s.heaterFans.at("hotend_fan").heater, "extruder");
    EXPECT_NEAR(s.heaterFans.at("hotend_fan").heaterTemp, 50.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, ControllerFanSection) {
    configPath = createTempConfig(R"(
[controller_fan board_fan]
pin = PA6
max_power = 0.6
idle_speed = 0.3
idle_timeout = 60
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.controllerFans.size(), 1u);
    ASSERT_TRUE(s.controllerFans.count("board_fan"));
    EXPECT_NEAR(s.controllerFans.at("board_fan").maxPower, 0.6, 1e-9);
    EXPECT_NEAR(s.controllerFans.at("board_fan").idleSpeed, 0.3, 1e-9);
    EXPECT_NEAR(s.controllerFans.at("board_fan").idleTimeout, 60.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, Tmc2209Section) {
    configPath = createTempConfig(R"(
[tmc2209 stepper_x]
uart_pin = PA7
run_current = 0.8
hold_current = 0.5
stealthchop_threshold = 999
interpolate = True
uart_address = 0
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_EQ(s.tmcDrivers.size(), 1u);
    ASSERT_TRUE(s.tmcDrivers.count("stepper_x"));
    EXPECT_EQ(s.tmcDrivers.at("stepper_x").driverType, "tmc2209");
    EXPECT_EQ(s.tmcDrivers.at("stepper_x").uartPin, "PA7");
    EXPECT_NEAR(s.tmcDrivers.at("stepper_x").runCurrent, 0.8, 1e-9);
    EXPECT_TRUE(s.tmcDrivers.at("stepper_x").interpolate);
}

TEST_F(E3PrinterConfigTest, Tmc5160Section) {
    configPath = createTempConfig(R"(
[tmc5160 stepper_y]
spi_bus = spi1
cs_pin = PA8
run_current = 1.2
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    ASSERT_TRUE(s.tmcDrivers.count("stepper_y"));
    EXPECT_EQ(s.tmcDrivers.at("stepper_y").driverType, "tmc5160");
    EXPECT_EQ(s.tmcDrivers.at("stepper_y").spiBus, "spi1");
    EXPECT_NEAR(s.tmcDrivers.at("stepper_y").runCurrent, 1.2, 1e-9);
}

TEST_F(E3PrinterConfigTest, Adxl345Section) {
    configPath = createTempConfig(R"(
[adxl345]
spi_bus = spi1
cs_pin = PA9
rate = 1600
axes_map = xyz
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_TRUE(s.adxl345Configured);
    EXPECT_EQ(s.adxl345SpiBus, "spi1");
    EXPECT_EQ(s.adxl345CsPin, "PA9");
    EXPECT_EQ(s.adxl345Rate, 1600);
    EXPECT_EQ(s.adxl345AxesMap, "xyz");
}

TEST_F(E3PrinterConfigTest, InputShaperSection) {
    configPath = createTempConfig(R"(
[input_shaper]
shaper_freq_x = 35.5
shaper_freq_y = 42.0
shaper_type_x = ei
shaper_type_y = mzv
damping_ratio_x = 0.15
damping_ratio_y = 0.12
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_NEAR(s.inputShaperFreqX, 35.5, 1e-9);
    EXPECT_NEAR(s.inputShaperFreqY, 42.0, 1e-9);
    EXPECT_EQ(s.inputShaperTypeX, "ei");
    EXPECT_EQ(s.inputShaperTypeY, "mzv");
    EXPECT_NEAR(s.inputShaperDampingX, 0.15, 1e-9);
}

TEST_F(E3PrinterConfigTest, SkewCorrectionSection) {
    configPath = createTempConfig(R"(
[skew_correction]
xy_skew = 0.01
xz_skew = 0.02
yz_skew = 0.03
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_NEAR(s.skewParams.xy, 0.01, 1e-9);
    EXPECT_NEAR(s.skewParams.xz, 0.02, 1e-9);
    EXPECT_NEAR(s.skewParams.yz, 0.03, 1e-9);
}

TEST_F(E3PrinterConfigTest, ZTiltSection) {
    configPath = createTempConfig(R"(
[z_tilt]
z_positions = 0,0
              200,0
              100,200
retries = 3
retry_tolerance = 0.05
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_TRUE(s.zTiltEnabled);
    EXPECT_EQ(s.zTiltRetries, 3);
    EXPECT_NEAR(s.zTiltRetryTolerance, 0.05, 1e-9);
}

TEST_F(E3PrinterConfigTest, QuadGantryLevelSection) {
    configPath = createTempConfig(R"(
[quad_gantry_level]
z_positions = 0,0
              200,0
              200,200
              0,200
retries = 2
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_TRUE(s.qglEnabled);
    EXPECT_EQ(s.qglRetries, 2);
}

TEST_F(E3PrinterConfigTest, BedScrewsSection) {
    configPath = createTempConfig(R"(
[bed_screws]
screws = 10,10
          190,10
          10,190
          190,190
probe_speed = 5
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_TRUE(s.bedScrewsEnabled);
    EXPECT_NEAR(s.bedScrewsProbeSpeed, 5.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, ScrewsTiltAdjustSection) {
    configPath = createTempConfig(R"(
[screws_tilt_adjust]
screws = 10,10
          190,10
          10,190
          190,190
screw_thread = CW-M3
horizontal_move_z = 5
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_TRUE(s.screwsTiltEnabled);
    EXPECT_EQ(s.screwsTiltThread, "CW-M3");
    EXPECT_NEAR(s.screwsTiltHorizontalZ, 5.0, 1e-9);
}

TEST_F(E3PrinterConfigTest, MultipleSectionsCombined) {
    configPath = createTempConfig(R"(
[printer]
kinematics = corexy
max_velocity = 3000

[safe_z_home]
z_hop = 5

[idle_timeout]
timeout = 600

[tmc2209 stepper_x]
run_current = 0.8

[output_pin fan2]
pin = PB0
)");
    ASSERT_TRUE(instance->loadConfig(configPath));
    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, Kinematics::CoreXY);
    EXPECT_NEAR(s.safeZHomeZHop, 5.0, 1e-9);
    EXPECT_NEAR(s.idleTimeout, 600.0, 1e-9);
    ASSERT_TRUE(s.tmcDrivers.count("stepper_x"));
    ASSERT_TRUE(s.outputPins.count("fan2"));
}

// ============================================================================
// E3: KinematicsTransform tests
// ============================================================================

class E3KinematicsTest : public ::testing::Test {};

TEST_F(E3KinematicsTest, CartesianIdentity) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::Cartesian);
    auto result = kt.forwardActuatorKinematics(10.0, 20.0, 30.0);
    EXPECT_NEAR(result[0], 10.0, 1e-9);
    EXPECT_NEAR(result[1], 20.0, 1e-9);
    EXPECT_NEAR(result[2], 30.0, 1e-9);
}

TEST_F(E3KinematicsTest, CartesianInverseIdentity) {
    KinematicsTransform kt;
    auto result = kt.inverseActuatorKinematics(10.0, 20.0, 30.0);
    EXPECT_NEAR(result[0], 10.0, 1e-9);
    EXPECT_NEAR(result[1], 20.0, 1e-9);
    EXPECT_NEAR(result[2], 30.0, 1e-9);
}

TEST_F(E3KinematicsTest, CoreXYForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreXY);
    auto result = kt.forwardActuatorKinematics(100.0, 50.0, 10.0);
    // A = X + Y = 150, B = X - Y = 50, C = Z = 10
    EXPECT_NEAR(result[0], 150.0, 1e-9);
    EXPECT_NEAR(result[1], 50.0, 1e-9);
    EXPECT_NEAR(result[2], 10.0, 1e-9);
}

TEST_F(E3KinematicsTest, CoreXYInverse) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreXY);
    auto result = kt.inverseActuatorKinematics(150.0, 50.0, 10.0);
    // X = (A + B) / 2 = 100, Y = (A - B) / 2 = 50, Z = C = 10
    EXPECT_NEAR(result[0], 100.0, 1e-9);
    EXPECT_NEAR(result[1], 50.0, 1e-9);
    EXPECT_NEAR(result[2], 10.0, 1e-9);
}

TEST_F(E3KinematicsTest, CoreXYRoundTrip) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreXY);
    double x = 73.5, y = -42.3, z = 15.7;
    auto fwd = kt.forwardActuatorKinematics(x, y, z);
    auto inv = kt.inverseActuatorKinematics(fwd[0], fwd[1], fwd[2]);
    EXPECT_NEAR(inv[0], x, 1e-9);
    EXPECT_NEAR(inv[1], y, 1e-9);
    EXPECT_NEAR(inv[2], z, 1e-9);
}

TEST_F(E3KinematicsTest, CoreXZForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreXZ);
    auto result = kt.forwardActuatorKinematics(100.0, 50.0, 30.0);
    // A = X + Z = 130, B = X - Z = 70, C = Y = 50
    EXPECT_NEAR(result[0], 130.0, 1e-9);
    EXPECT_NEAR(result[1], 70.0, 1e-9);
    EXPECT_NEAR(result[2], 50.0, 1e-9);
}

TEST_F(E3KinematicsTest, CoreXZInverse) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreXZ);
    auto result = kt.inverseActuatorKinematics(130.0, 70.0, 50.0);
    EXPECT_NEAR(result[0], 100.0, 1e-9);
    EXPECT_NEAR(result[1], 50.0, 1e-9);
    EXPECT_NEAR(result[2], 30.0, 1e-9);
}

TEST_F(E3KinematicsTest, CoreYZForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreYZ);
    auto result = kt.forwardActuatorKinematics(100.0, 50.0, 30.0);
    // A = Y + Z = 80, B = Y - Z = 20, C = X = 100
    EXPECT_NEAR(result[0], 80.0, 1e-9);
    EXPECT_NEAR(result[1], 20.0, 1e-9);
    EXPECT_NEAR(result[2], 100.0, 1e-9);
}

TEST_F(E3KinematicsTest, CoreYZInverse) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreYZ);
    auto result = kt.inverseActuatorKinematics(80.0, 20.0, 100.0);
    EXPECT_NEAR(result[0], 100.0, 1e-9);
    EXPECT_NEAR(result[1], 50.0, 1e-9);
    EXPECT_NEAR(result[2], 30.0, 1e-9);
}

TEST_F(E3KinematicsTest, DeltaForwardAtCenter) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::Delta);
    DeltaPrinter dp;
    DeltaGeometry geo;
    geo.deltaRadius = 100.0;
    geo.armLength = 200.0;
    dp.setGeometry(geo);
    kt.setDeltaPrinter(&dp);
    // At center (0, 0, 0), all towers should be equal
    auto result = kt.forwardActuatorKinematics(0.0, 0.0, 0.0);
    EXPECT_NEAR(result[0], result[1], 1e-6);
    EXPECT_NEAR(result[1], result[2], 1e-6);
}

TEST_F(E3KinematicsTest, DeltaRoundTrip) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::Delta);
    DeltaPrinter dp;
    DeltaGeometry geo;
    geo.deltaRadius = 100.0;
    geo.armLength = 200.0;
    dp.setGeometry(geo);
    kt.setDeltaPrinter(&dp);
    double x = 30.0, y = 20.0, z = 100.0;
    auto fwd = kt.forwardActuatorKinematics(x, y, z);
    auto inv = kt.inverseActuatorKinematics(fwd[0], fwd[1], fwd[2]);
    EXPECT_NEAR(inv[0], x, 1e-4);
    EXPECT_NEAR(inv[1], y, 1e-4);
    EXPECT_NEAR(inv[2], z, 1e-4);
}

TEST_F(E3KinematicsTest, GetKinematicsType) {
    KinematicsTransform kt;
    EXPECT_EQ(kt.kinematics(), Kinematics::Cartesian);
    kt.setKinematics(Kinematics::CoreXY);
    EXPECT_EQ(kt.kinematics(), Kinematics::CoreXY);
    kt.setKinematics(Kinematics::Delta);
    EXPECT_EQ(kt.kinematics(), Kinematics::Delta);
}

// ============================================================================
// E3: MotionTranslator with kinematics tests
// ============================================================================

TEST_F(E3KinematicsTest, MotionTranslatorAcceptsKinematics) {
    std::array<AxisConfig, 4> configs = {{
        {80.0, false}, {80.0, false}, {400.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> oids = {0, 1, 2, 3};
    MotionTranslator<4> mt(configs, oids);
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::CoreXY);
    mt.setKinematicsTransform(kt);
    EXPECT_EQ(mt.kinematicsTransform().kinematics(), Kinematics::CoreXY);
}

// ============================================================================
// E3: Spoolman proxy tests
// ============================================================================

class E3SpoolmanTest : public ::testing::Test {
protected:
    void SetUp() override {
        socketPath = uniqueSocketPath();
        UdsServerConfig udsCfg;
        udsCfg.socketPath = socketPath;
        server = std::make_unique<KlippyServer>(udsCfg);
    }

    void TearDown() override {
        server.reset();
        ::unlink(socketPath.c_str());
    }

    std::string socketPath;
    std::unique_ptr<KlippyServer> server;
};

TEST_F(E3SpoolmanTest, ProxyReturnsNotConnectedWhenUrlEmpty) {
    // Without setting spoolman URL, proxy should return not_connected
    JsonValue params(std::map<std::string, JsonValue>{
        {"path", JsonValue(std::string("/spool"))}
    });
    // Call the spoolman proxy endpoint
    auto response = server->callEndpoint("spoolman/proxy", params);
    ASSERT_TRUE(response.isObject());
    const auto* result = response.find("result");
    ASSERT_NE(result, nullptr);
    ASSERT_TRUE(result->isObject());
    const auto* status = result->find("status");
    ASSERT_NE(status, nullptr);
    ASSERT_TRUE(status->isString());
    EXPECT_EQ(status->asString(), "not_connected");
}

TEST_F(E3SpoolmanTest, ProxyUrlConfigured) {
    UdsServerConfig udsCfg;
    udsCfg.socketPath = uniqueSocketPath() + "_2";
    udsCfg.spoolmanUrl = "http://localhost:8080";
    auto s = std::make_unique<KlippyServer>(udsCfg);
    s->callEndpoint("spoolman/info", JsonValue(std::map<std::string, JsonValue>{}));
    // Just verify it doesn't crash; the URL is stored internally
    SUCCEED();
    ::unlink(udsCfg.socketPath.c_str());
}

TEST_F(E3SpoolmanTest, SetSpoolmanConnected) {
    server->setSpoolmanConnected(true, "http://localhost:8080");
    // Verify info endpoint reflects connection
    auto response = server->callEndpoint("spoolman/info",
        JsonValue(std::map<std::string, JsonValue>{}));
    ASSERT_TRUE(response.isObject());
    const auto* result = response.find("result");
    ASSERT_NE(result, nullptr);
    ASSERT_TRUE(result->isObject());
    const auto* connected = result->find("connected");
    ASSERT_NE(connected, nullptr);
    ASSERT_TRUE(connected->isBool());
    EXPECT_TRUE(connected->asBool());
}

// ============================================================================
// E3: New printer objects tests
// ============================================================================

class E3PrinterObjectsTest : public ::testing::Test {};

// Note: Adxl345Object already exists in BedMeshPrinterObject.hpp
// and is tested elsewhere. Here we test the new E3 objects.

TEST_F(E3PrinterObjectsTest, DelayedGcodeObject) {
    DelayedGcodeObject obj("delayed_gcode my_gcode");
    EXPECT_EQ(obj.name(), "delayed_gcode my_gcode");
    obj.setEnabled(true);
    obj.setRemainingDuration(15.5);
    obj.setGcode("G28");
    auto status = obj.status({});
    EXPECT_TRUE(status["enabled"].asBool());
    EXPECT_NEAR(status["remaining_duration"].asDouble(), 15.5, 1e-9);
    EXPECT_EQ(status["gcode"].asString(), "G28");
}

TEST_F(E3PrinterObjectsTest, DelayedGcodeObjectFields) {
    DelayedGcodeObject obj("test");
    auto fields = obj.availableFields();
    ASSERT_GE(fields.size(), 3u);
}

TEST_F(E3PrinterObjectsTest, SaveVariablesObject) {
    SaveVariablesObject obj;
    EXPECT_EQ(obj.name(), "save_variables");
    obj.setVariable("foo", "bar");
    obj.setVariable("count", "42");
    auto status = obj.status({});
    ASSERT_TRUE(status.count("variables"));
    ASSERT_TRUE(status["variables"].isObject());
    const auto* foo = status["variables"].find("foo");
    ASSERT_NE(foo, nullptr);
    EXPECT_EQ(foo->asString(), "bar");
}

TEST_F(E3PrinterObjectsTest, SaveVariablesObjectRemove) {
    SaveVariablesObject obj;
    obj.setVariable("foo", "bar");
    obj.removeVariable("foo");
    auto status = obj.status({});
    auto it = status.find("variables");
    ASSERT_NE(it, status.end());
    ASSERT_TRUE(it->second.isObject());
    EXPECT_EQ(it->second.find("foo"), nullptr);
}

TEST_F(E3PrinterObjectsTest, BoardPinsObject) {
    BoardPinsObject obj;
    EXPECT_EQ(obj.name(), "board_pins");
    obj.setMcuName("mcu");
    obj.addAlias("HEATER0", "PA0");
    obj.addAlias("FAN0", "PA1");
    auto status = obj.status({});
    EXPECT_EQ(status["mcu"].asString(), "mcu");
    ASSERT_TRUE(status["aliases"].isObject());
    const auto* heater = status["aliases"].find("HEATER0");
    ASSERT_NE(heater, nullptr);
    EXPECT_EQ(heater->asString(), "PA0");
}

TEST_F(E3PrinterObjectsTest, BoardPinsObjectFields) {
    BoardPinsObject obj;
    auto fields = obj.availableFields();
    ASSERT_GE(fields.size(), 2u);
    bool hasMcu = std::find(fields.begin(), fields.end(), "mcu") != fields.end();
    bool hasAliases = std::find(fields.begin(), fields.end(), "aliases") != fields.end();
    EXPECT_TRUE(hasMcu);
    EXPECT_TRUE(hasAliases);
}
