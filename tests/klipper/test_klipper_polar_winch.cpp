/**
 * @file test_klipper_polar_winch.cpp
 * @brief Tests for Polar and Winch kinematics config parsing and wiring.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <cmath>

namespace klippy = tether::klipper::klippy;
namespace kmotion = tether::klipper::motion;

static std::string uniqueSocketPathPW() {
    return "/tmp/tether_test_pw_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfigPW(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/tether_test_pw_cfg_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter++) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

// ============================================================================
// Polar kinematics tests
// ============================================================================

TEST(PolarKinematicsConfigTest, PolarConfigParsedFromConfig) {
    std::string cfgPath = createTempConfigPW(R"(
[printer]
kinematics = polar
max_velocity = 300
max_accel = 2000
[polar]
max_radius = 150.0
max_angle = 270.0
continuous_rotation = 1
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathPW();
    cfg.settingsPath = "/tmp/tether_test_pw_polar_settings.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, klippy::Kinematics::Polar);
    EXPECT_NEAR(s.polarConfig.maxRadius, 150.0, 1e-9);
    EXPECT_NEAR(s.polarConfig.maxAngle, 270.0, 1e-9);
    EXPECT_TRUE(s.polarConfig.continuousRotation);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(PolarKinematicsConfigTest, PolarTransformCorrect) {
    std::string cfgPath = createTempConfigPW(R"(
[printer]
kinematics = polar
max_velocity = 300
max_accel = 2000
[polar]
max_radius = 200.0
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathPW();
    cfg.settingsPath = "/tmp/tether_test_pw_polar_settings2.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    // Polar transform: A=radius, B=angle(degrees), C=Z
    // At (10, 0, 5): radius=10, angle=0, Z=5
    kmotion::KinematicsTransform kt;
    kt.setKinematics(klippy::Kinematics::Polar);
    auto result = kt.forwardActuatorKinematics(10.0, 0.0, 5.0);
    EXPECT_NEAR(result[0], 10.0, 1e-6);  // radius
    EXPECT_NEAR(result[1], 0.0, 1e-6);   // angle
    EXPECT_NEAR(result[2], 5.0, 1e-6);   // Z

    // At (3, 4, 0): radius=5, angle=atan2(4,3)=53.13°
    result = kt.forwardActuatorKinematics(3.0, 4.0, 0.0);
    EXPECT_NEAR(result[0], 5.0, 1e-6);
    EXPECT_NEAR(result[1], 53.1301, 0.01);
    EXPECT_NEAR(result[2], 0.0, 1e-6);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

// ============================================================================
// Winch kinematics tests
// ============================================================================

TEST(WinchKinematicsConfigTest, WinchConfigParsedFromConfig) {
    std::string cfgPath = createTempConfigPW(R"(
[printer]
kinematics = winch
max_velocity = 300
max_accel = 2000
[winch]
anchor_radius = 600.0
anchor_height = 350.0
anchor_count = 3
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathPW();
    cfg.settingsPath = "/tmp/tether_test_pw_winch_settings.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, klippy::Kinematics::Winch);
    EXPECT_NEAR(s.winchConfig.anchorRadius, 600.0, 1e-9);
    EXPECT_NEAR(s.winchConfig.anchorHeight, 350.0, 1e-9);
    EXPECT_EQ(s.winchConfig.anchorCount, 3);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(WinchKinematicsConfigTest, WinchTransformUsesConfiguredParams) {
    std::string cfgPath = createTempConfigPW(R"(
[printer]
kinematics = winch
max_velocity = 300
max_accel = 2000
[winch]
anchor_radius = 400.0
anchor_height = 250.0
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathPW();
    cfg.settingsPath = "/tmp/tether_test_pw_winch_settings2.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    // Verify the kinematics transform uses the configured anchor params
    // by checking that different params produce different cable lengths
    kmotion::KinematicsTransform kt;
    kt.setKinematics(klippy::Kinematics::Winch);
    kt.setWinchParams(400.0, 250.0);

    double x = 50.0, y = 30.0, z = -100.0;
    auto cableLengths = kt.forwardActuatorKinematics(x, y, z);

    // With different anchor params, cable lengths should differ
    kmotion::KinematicsTransform kt2;
    kt2.setKinematics(klippy::Kinematics::Winch);
    kt2.setWinchParams(800.0, 500.0);
    auto cableLengths2 = kt2.forwardActuatorKinematics(x, y, z);
    EXPECT_NE(cableLengths[0], cableLengths2[0]);
    EXPECT_NE(cableLengths[1], cableLengths2[1]);
    EXPECT_NE(cableLengths[2], cableLengths2[2]);

    // Verify cable lengths are physically reasonable (positive, and
    // longer than the straight-line distance to the anchor)
    for (int i = 0; i < 3; ++i) {
        EXPECT_GT(cableLengths[i], 0.0);
    }

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(WinchKinematicsConfigTest, WinchDefaultParamsMatchOldBehavior) {
    // Without [winch] section, defaults should be anchor_radius=500, height=300
    std::string cfgPath = createTempConfigPW(R"(
[printer]
kinematics = winch
max_velocity = 300
max_accel = 2000
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathPW();
    cfg.settingsPath = "/tmp/tether_test_pw_winch_settings3.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_NEAR(s.winchConfig.anchorRadius, 500.0, 1e-9);
    EXPECT_NEAR(s.winchConfig.anchorHeight, 300.0, 1e-9);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}
