/**
 * @file test_klipper_e4_wiring.cpp
 * @brief Tests for E4 features: config-to-runtime wiring, applySettings(),
 *        new G-codes (G32/G33/G34), idle timeout, safe Z home,
 *        HybridCoreXY/HybridCoreXZ transforms, RotaryDelta/Polar/Winch,
 *        [spoolman] config, ADXL345 auto-creation, and printer object
 *        registration.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/klippy/PrinterObjectsE2.hpp"
#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/klipper/klippy/DeltaPrinter.hpp"
#include "tether/klipper/klippy/InputShaper.hpp"
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
// Helpers
// ============================================================================

static std::string uniqueSocketPath() {
    return "/tmp/tether_test_e4_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfig(const std::string& content) {
    std::string path = "/tmp/tether_test_e4_cfg_" + std::to_string(getpid()) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

// ============================================================================
// E4: Printer object registration tests
// ============================================================================

class E4ObjectRegistrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPath();
        instance = std::make_unique<KlippyInstance>(cfg);
    }
    void TearDown() override {
        instance.reset();
        std::filesystem::remove(cfgPath);
    }
    std::unique_ptr<KlippyInstance> instance;
    std::string cfgPath;
};

TEST_F(E4ObjectRegistrationTest, DelayedGcodeObjectRegistered) {
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "delayed_gcode") != objects.end();
    EXPECT_TRUE(found);
}

TEST_F(E4ObjectRegistrationTest, SaveVariablesObjectRegistered) {
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "save_variables") != objects.end();
    EXPECT_TRUE(found);
}

TEST_F(E4ObjectRegistrationTest, BoardPinsObjectRegistered) {
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "board_pins") != objects.end();
    EXPECT_TRUE(found);
}

TEST_F(E4ObjectRegistrationTest, DelayedGcodeObjectQueryable) {
    std::map<std::string, std::vector<std::string>> query = {
        {"delayed_gcode", {}}
    };
    auto status = instance->server().queryObjects(query);
    ASSERT_TRUE(status.count("delayed_gcode"));
    // Should have enabled and remaining_duration fields
    EXPECT_TRUE(status["delayed_gcode"].count("enabled") ||
                status["delayed_gcode"].count("remaining_duration"));
}

TEST_F(E4ObjectRegistrationTest, SaveVariablesObjectQueryable) {
    std::map<std::string, std::vector<std::string>> query = {
        {"save_variables", {}}
    };
    auto status = instance->server().queryObjects(query);
    ASSERT_TRUE(status.count("save_variables"));
}

TEST_F(E4ObjectRegistrationTest, BoardPinsObjectQueryable) {
    std::map<std::string, std::vector<std::string>> query = {
        {"board_pins", {}}
    };
    auto status = instance->server().queryObjects(query);
    ASSERT_TRUE(status.count("board_pins"));
}

// ============================================================================
// E4: Motion limits wiring tests
// ============================================================================

class E4MotionLimitsTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 500
max_accel = 3000
max_accel_to_decel = 1500
square_corner_velocity = 5.0
)");
        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPath();
        instance = std::make_unique<KlippyInstance>(cfg);
        instance->loadConfig(cfgPath);
    }
    void TearDown() override {
        instance.reset();
        std::filesystem::remove(cfgPath);
    }
    std::unique_ptr<KlippyInstance> instance;
    std::string cfgPath;
};

TEST_F(E4MotionLimitsTest, MaxVelocityApplied) {
    auto& s = instance->settings();
    EXPECT_NEAR(s.maxVelocity, 500, 1e-9);
    // Check toolhead object has the value
    auto status = instance->server().queryObjects({{"toolhead", {"max_velocity"}}});
    ASSERT_TRUE(status.count("toolhead"));
    ASSERT_TRUE(status["toolhead"].count("max_velocity"));
    EXPECT_NEAR(status["toolhead"]["max_velocity"].asDouble(), 500, 1e-9);
}

TEST_F(E4MotionLimitsTest, MaxAccelApplied) {
    auto status = instance->server().queryObjects({{"toolhead", {"max_accel"}}});
    ASSERT_TRUE(status.count("toolhead"));
    ASSERT_TRUE(status["toolhead"].count("max_accel"));
    EXPECT_NEAR(status["toolhead"]["max_accel"].asDouble(), 3000, 1e-9);
}

TEST_F(E4MotionLimitsTest, MaxAccelToDecelApplied) {
    auto status = instance->server().queryObjects({{"toolhead", {"max_accel_to_decel"}}});
    ASSERT_TRUE(status.count("toolhead"));
    ASSERT_TRUE(status["toolhead"].count("max_accel_to_decel"));
    EXPECT_NEAR(status["toolhead"]["max_accel_to_decel"].asDouble(), 1500, 1e-9);
}

// ============================================================================
// E4: Kinematics wiring tests
// ============================================================================

class E4KinematicsWiringTest : public ::testing::Test {
protected:
    void TearDown() override {
        instance.reset();
        std::filesystem::remove(cfgPath);
    }
    std::unique_ptr<KlippyInstance> instance;
    std::string cfgPath;
};

TEST_F(E4KinematicsWiringTest, CoreXYKinematicsWired) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = corexy
max_velocity = 300
max_accel = 2000
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    EXPECT_EQ(instance->settings().kinematics, Kinematics::CoreXY);
}

TEST_F(E4KinematicsWiringTest, DeltaKinematicsWired) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = delta
max_velocity = 300
max_accel = 2000
delta_radius = 100
[stepper_a]
position_endstop = 250
arm_length = 200
[stepper_b]
position_endstop = 250
arm_length = 200
[stepper_c]
position_endstop = 250
arm_length = 200
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    EXPECT_EQ(instance->settings().kinematics, Kinematics::Delta);
}

// ============================================================================
// E4: Hybrid kinematics transform tests
// ============================================================================

class E4HybridKinematicsTest : public ::testing::Test {};

TEST_F(E4HybridKinematicsTest, HybridCoreXYForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::HybridCoreXY);
    auto result = kt.transform(100, 50, 10);
    // A = X + Y = 150, B = X - Y = 50, C = Z = 10
    EXPECT_NEAR(result[0], 150, 1e-9);
    EXPECT_NEAR(result[1], 50, 1e-9);
    EXPECT_NEAR(result[2], 10, 1e-9);
}

TEST_F(E4HybridKinematicsTest, HybridCoreXYInverse) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::HybridCoreXY);
    auto result = kt.inverseTransform(150, 50, 10);
    // X = (A + B) / 2 = 100, Y = (A - B) / 2 = 50, Z = C = 10
    EXPECT_NEAR(result[0], 100, 1e-9);
    EXPECT_NEAR(result[1], 50, 1e-9);
    EXPECT_NEAR(result[2], 10, 1e-9);
}

TEST_F(E4HybridKinematicsTest, HybridCoreXYRoundTrip) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::HybridCoreXY);
    for (double x : {0.0, 50.0, -30.0, 100.0}) {
        for (double y : {0.0, 25.0, -15.0, 80.0}) {
            auto stepper = kt.transform(x, y, 10);
            auto cart = kt.inverseTransform(stepper[0], stepper[1], stepper[2]);
            EXPECT_NEAR(cart[0], x, 1e-9);
            EXPECT_NEAR(cart[1], y, 1e-9);
            EXPECT_NEAR(cart[2], 10, 1e-9);
        }
    }
}

TEST_F(E4HybridKinematicsTest, HybridCoreXZForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::HybridCoreXZ);
    auto result = kt.transform(100, 50, 30);
    // A = X + Z = 130, B = X - Z = 70, C = Y = 50
    EXPECT_NEAR(result[0], 130, 1e-9);
    EXPECT_NEAR(result[1], 70, 1e-9);
    EXPECT_NEAR(result[2], 50, 1e-9);
}

TEST_F(E4HybridKinematicsTest, HybridCoreXZInverse) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::HybridCoreXZ);
    auto result = kt.inverseTransform(130, 70, 50);
    // X = (A + B) / 2 = 100, Y = C = 50, Z = (A - B) / 2 = 30
    EXPECT_NEAR(result[0], 100, 1e-9);
    EXPECT_NEAR(result[1], 50, 1e-9);
    EXPECT_NEAR(result[2], 30, 1e-9);
}

TEST_F(E4HybridKinematicsTest, HybridCoreXZRoundTrip) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::HybridCoreXZ);
    for (double x : {0.0, 50.0, -30.0}) {
        for (double z : {0.0, 25.0, -15.0}) {
            auto stepper = kt.transform(x, 50, z);
            auto cart = kt.inverseTransform(stepper[0], stepper[1], stepper[2]);
            EXPECT_NEAR(cart[0], x, 1e-9);
            EXPECT_NEAR(cart[2], z, 1e-9);
        }
    }
}

// ============================================================================
// E4: Niche kinematics transform tests (RotaryDelta, Polar, Winch)
// ============================================================================

class E4NicheKinematicsTest : public ::testing::Test {};

TEST_F(E4NicheKinematicsTest, PolarForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::Polar);
    auto result = kt.transform(100, 0, 10);
    // radius = 100, angle = 0, z = 10
    EXPECT_NEAR(result[0], 100, 1e-9);
    EXPECT_NEAR(result[1], 0, 1e-9);
    EXPECT_NEAR(result[2], 10, 1e-9);
}

TEST_F(E4NicheKinematicsTest, PolarForward45deg) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::Polar);
    auto result = kt.transform(10, 10, 5);
    // radius = sqrt(200) ~ 14.14, angle = 45
    EXPECT_NEAR(result[0], std::sqrt(200.0), 1e-9);
    EXPECT_NEAR(result[1], 45.0, 1e-9);
    EXPECT_NEAR(result[2], 5, 1e-9);
}

TEST_F(E4NicheKinematicsTest, WinchForward) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::Winch);
    auto result = kt.transform(0, 0, 0);
    // All cable lengths should be equal (distance from center to each anchor + height)
    // Anchor at (500, 0), height 300: length = sqrt(500^2 + 300^2) = sqrt(340000)
    double expectedLen = std::sqrt(500.0*500.0 + 300.0*300.0);
    EXPECT_NEAR(result[0], expectedLen, 1e-6);
    EXPECT_NEAR(result[1], expectedLen, 1e-6);
    EXPECT_NEAR(result[2], expectedLen, 1e-6);
}

TEST_F(E4NicheKinematicsTest, RotaryDeltaForwardAtCenter) {
    KinematicsTransform kt;
    kt.setKinematics(Kinematics::RotaryDelta);
    RotaryDeltaPrinter rdp;
    RotaryDeltaGeometry geo;
    geo.upperArmLength = 170.0;
    geo.forearmLength = 320.0;
    geo.baseRadius = 90.0;
    geo.effectorRadius = 24.0;
    geo.baseHeight = 0.0;
    rdp.setGeometry(geo);
    kt.setRotaryDeltaPrinter(&rdp);
    auto result = kt.transform(0, 0, -200);
    // At center, all arm angles should be equal
    EXPECT_NEAR(result[0], result[1], 1e-9);
    EXPECT_NEAR(result[1], result[2], 1e-9);
}

// ============================================================================
// E4: New G-code tests (G32, G33, G34)
// ============================================================================

class E4GcodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPath();
        instance = std::make_unique<KlippyInstance>(cfg);
    }
    void TearDown() override {
        instance.reset();
    }
    std::unique_ptr<KlippyInstance> instance;
};

TEST_F(E4GcodeTest, G32AutoBedLevel) {
    // G32 should execute without error even without a probe
    bool ok = instance->gcode().execute("G32");
    EXPECT_TRUE(ok);
}

TEST_F(E4GcodeTest, G33DeltaCalibrationNonDelta) {
    // G33 on non-delta should return true (graceful handling)
    bool ok = instance->gcode().execute("G33");
    EXPECT_TRUE(ok);
}

TEST_F(E4GcodeTest, G34ZTiltWithoutConfig) {
    // G34 without [z_tilt] config should return true (graceful)
    bool ok = instance->gcode().execute("G34");
    EXPECT_TRUE(ok);
}

TEST_F(E4GcodeTest, G34ZTiltWithConfig) {
    std::string path = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[z_tilt]
z_positions = 0,100,200
retries = 3
retry_tolerance = 0.05
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto inst = std::make_unique<KlippyInstance>(cfg);
    inst->loadConfig(path);
    bool ok = inst->gcode().execute("G34");
    EXPECT_TRUE(ok);
    std::filesystem::remove(path);
}

// ============================================================================
// E4: Idle timeout tests
// ============================================================================

class E4IdleTimeoutTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[idle_timeout]
timeout = 1
gcode = TURN_OFF_HEATERS
)");
        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPath();
        instance = std::make_unique<KlippyInstance>(cfg);
        instance->loadConfig(cfgPath);
    }
    void TearDown() override {
        instance.reset();
        std::filesystem::remove(cfgPath);
    }
    std::unique_ptr<KlippyInstance> instance;
    std::string cfgPath;
};

TEST_F(E4IdleTimeoutTest, IdleTimeoutConfigured) {
    EXPECT_NEAR(instance->settings().idleTimeout, 1, 1e-9);
    EXPECT_EQ(instance->settings().idleTimeoutGcode, "TURN_OFF_HEATERS");
}

TEST_F(E4IdleTimeoutTest, NoteActivityResetsTimer) {
    instance->noteActivity();
    // Should not timeout immediately after activity
    auto status = instance->server().queryObjects({{"idle_timeout", {"state"}}});
    ASSERT_TRUE(status.count("idle_timeout"));
    ASSERT_TRUE(status["idle_timeout"].count("state"));
    // State should be "Ready" after activity
    EXPECT_EQ(status["idle_timeout"]["state"].asString(), "Ready");
}

TEST_F(E4IdleTimeoutTest, TickProcessesIdleTimeout) {
    // Set a very short timeout and tick
    instance->settings().idleTimeout = 0.001; // 1ms
    instance->settings().idleTimeoutGcode = "M84"; // Disable motors
    // Wait a bit so idle time exceeds timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    instance->tick();
    // After timeout, state should be "Idle"
    auto status = instance->server().queryObjects({{"idle_timeout", {"state"}}});
    ASSERT_TRUE(status.count("idle_timeout"));
    ASSERT_TRUE(status["idle_timeout"].count("state"));
    EXPECT_EQ(status["idle_timeout"]["state"].asString(), "Idle");
}

// ============================================================================
// E4: Safe Z home tests
// ============================================================================

class E4SafeZHomeTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[safe_z_home]
home_xy_position = 100, 100
z_hop = 10
z_hop_speed = 20
xy_home_speed = 50
move_to_previous = True
)");
        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPath();
        instance = std::make_unique<KlippyInstance>(cfg);
        instance->loadConfig(cfgPath);
    }
    void TearDown() override {
        instance.reset();
        std::filesystem::remove(cfgPath);
    }
    std::unique_ptr<KlippyInstance> instance;
    std::string cfgPath;
};

TEST_F(E4SafeZHomeTest, SafeZHomeConfigured) {
    EXPECT_EQ(instance->settings().safeZHomeXYPosition, "100, 100");
    EXPECT_NEAR(instance->settings().safeZHomeZHop, 10, 1e-9);
    EXPECT_NEAR(instance->settings().safeZHomeZHopSpeed, 20, 1e-9);
    EXPECT_NEAR(instance->settings().safeZHomeXYHomeSpeed, 50, 1e-9);
    EXPECT_TRUE(instance->settings().safeZHomeMoveToPrevious);
}

TEST_F(E4SafeZHomeTest, SafeZHomeObjectWired) {
    auto status = instance->server().queryObjects({{"safe_z_home", {}}});
    ASSERT_TRUE(status.count("safe_z_home"));
    ASSERT_TRUE(status["safe_z_home"].count("z_hop"));
    EXPECT_NEAR(status["safe_z_home"]["z_hop"].asDouble(), 10, 1e-9);
    ASSERT_TRUE(status["safe_z_home"].count("z_hop_speed"));
    EXPECT_NEAR(status["safe_z_home"]["z_hop_speed"].asDouble(), 20, 1e-9);
}

TEST_F(E4SafeZHomeTest, G28WithSafeZHome) {
    // G28 Z should execute without error when safe_z_home is configured
    bool ok = instance->gcode().execute("G28 Z");
    EXPECT_TRUE(ok);
}

// ============================================================================
// E4: Config section wiring tests
// ============================================================================

class E4ConfigWiringTest : public ::testing::Test {
protected:
    void TearDown() override {
        instance.reset();
        std::filesystem::remove(cfgPath);
    }
    std::unique_ptr<KlippyInstance> instance;
    std::string cfgPath;
};

TEST_F(E4ConfigWiringTest, OutputPinWired) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[output_pin my_pin]
pin = PA1
value = 0.5
pwm = True
scale = 1.0
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    ASSERT_EQ(s.outputPins.size(), 1u);
    ASSERT_TRUE(s.outputPins.count("my_pin"));
    EXPECT_EQ(s.outputPins["my_pin"].pin, "PA1");
    EXPECT_NEAR(s.outputPins["my_pin"].value, 0.5, 1e-9);
    EXPECT_TRUE(s.outputPins["my_pin"].pwm);
}

TEST_F(E4ConfigWiringTest, ServoWired) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[servo my_servo]
pin = PA2
minimum_pulse_width = 0.001
maximum_pulse_width = 0.002
initial_angle = 90
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    ASSERT_EQ(s.servos.size(), 1u);
    ASSERT_TRUE(s.servos.count("my_servo"));
    EXPECT_EQ(s.servos["my_servo"].pin, "PA2");
    EXPECT_NEAR(s.servos["my_servo"].initialAngle, 90, 1e-9);
}

TEST_F(E4ConfigWiringTest, TmcDriverWired) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[tmc2209 stepper_x]
uart_pin = PA3
run_current = 0.8
hold_current = 0.4
stealthchop_threshold = 0
interpolate = True
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    ASSERT_EQ(s.tmcDrivers.size(), 1u);
    ASSERT_TRUE(s.tmcDrivers.count("stepper_x"));
    EXPECT_EQ(s.tmcDrivers["stepper_x"].driverType, "tmc2209");
    EXPECT_EQ(s.tmcDrivers["stepper_x"].uartPin, "PA3");
    EXPECT_NEAR(s.tmcDrivers["stepper_x"].runCurrent, 0.8, 1e-9);
    EXPECT_TRUE(s.tmcDrivers["stepper_x"].interpolate);
}

TEST_F(E4ConfigWiringTest, TemperatureSensorWired) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[temperature_sensor chamber]
sensor_type = NTC 100K
sensor_pin = PA4
min_temp = 0
max_temp = 80
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    ASSERT_EQ(s.temperatureSensors.size(), 1u);
    ASSERT_TRUE(s.temperatureSensors.count("chamber"));
    EXPECT_EQ(s.temperatureSensors["chamber"].sensorType, "NTC 100K");
    // The sensor should be registered as a printer object
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor chamber") != objects.end();
    EXPECT_TRUE(found);
}

TEST_F(E4ConfigWiringTest, SpoolmanConfigSection) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[spoolman]
server = http://localhost:8080
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    // The spoolman URL should be set on the server after loadConfig
    EXPECT_EQ(instance->server().spoolmanUrl(), "http://localhost:8080");
}

TEST_F(E4ConfigWiringTest, Adxl345AutoCreated) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[adxl345]
cs_pin = PA5
spi_bus = spi1
rate = 3200
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    EXPECT_TRUE(s.adxl345Configured);
    // ADXL345 object should be registered
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "adxl345") != objects.end();
    EXPECT_TRUE(found);
}

TEST_F(E4ConfigWiringTest, InputShaperConfigured) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[input_shaper]
shaper_freq_x = 35.5
shaper_type_x = ei
damping_ratio_x = 0.1
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    EXPECT_NEAR(s.inputShaperFreqX, 35.5, 1e-9);
    EXPECT_EQ(s.inputShaperTypeX, "ei");
    // Input shaper should be active
    EXPECT_TRUE(instance->inputShaper().isActive());
}

TEST_F(E4ConfigWiringTest, ZTiltConfigured) {
    cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[z_tilt]
z_positions = 0,100,200
retries = 3
retry_tolerance = 0.05
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);
    auto& s = instance->settings();
    EXPECT_TRUE(s.zTiltEnabled);
    EXPECT_EQ(s.zTiltPositions, "0,100,200");
    EXPECT_EQ(s.zTiltRetries, 3);
}

// ============================================================================
// E4: applySettings() integration test
// ============================================================================

TEST(E4ApplySettingsTest, MultipleSettingsAppliedTogether) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = corexy
max_velocity = 400
max_accel = 2500
max_accel_to_decel = 1250
[idle_timeout]
timeout = 600
gcode = TURN_OFF_HEATERS
[output_pin fan2]
pin = PB0
value = 0
[servo probe_servo]
pin = PB1
initial_angle = 0
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    instance->loadConfig(cfgPath);

    // Check kinematics
    EXPECT_EQ(instance->settings().kinematics, Kinematics::CoreXY);
    // Check motion limits
    EXPECT_NEAR(instance->settings().maxVelocity, 400, 1e-9);
    EXPECT_NEAR(instance->settings().maxAccel, 2500, 1e-9);
    // Check idle timeout
    EXPECT_NEAR(instance->settings().idleTimeout, 600, 1e-9);
    // Check output pin
    ASSERT_EQ(instance->settings().outputPins.size(), 1u);
    EXPECT_EQ(instance->settings().outputPins["fan2"].pin, "PB0");
    // Check servo
    ASSERT_EQ(instance->settings().servos.size(), 1u);
    EXPECT_EQ(instance->settings().servos["probe_servo"].pin, "PB1");

    std::filesystem::remove(cfgPath);
}
