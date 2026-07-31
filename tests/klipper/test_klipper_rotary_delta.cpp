/**
 * @file test_klipper_rotary_delta.cpp
 * @brief Tests for klippy::RotaryDeltaPrinter kinematics: round-trip accuracy and
 *        known-position verification.
 */

#include <gtest/gtest.h>
#include "tether/kinematics/RotaryDeltaPrinter.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"

#include <cmath>

namespace klippy = tether::klipper::klippy;
namespace kmotion = tether::klipper::motion;

class RotaryDeltaTest : public ::testing::Test {
protected:
    klippy::RotaryDeltaPrinter printer;

    void SetUp() override {
        klippy::RotaryDeltaGeometry geo;
        geo.upperArmLength = 170.0;
        geo.forearmLength = 320.0;
        geo.baseRadius = 90.0;
        geo.effectorRadius = 24.0;
        geo.baseHeight = 0.0;
        printer.setGeometry(geo);
    }
};

// At the center (0,0), all three arms should have the same angle.
TEST_F(RotaryDeltaTest, CenterPositionSymmetric) {
    auto angles = printer.forwardActuatorKinematics(0.0, 0.0, -200.0);
    // All three angles should be equal (symmetric position).
    EXPECT_NEAR(angles[0], angles[1], 1e-6);
    EXPECT_NEAR(angles[1], angles[2], 1e-6);
}

// Round-trip: cartesian -> angles -> cartesian should recover the original.
TEST_F(RotaryDeltaTest, RoundTripCenter) {
    double x = 0.0, y = 0.0, z = -200.0;
    auto angles = printer.forwardActuatorKinematics(x, y, z);
    auto recovered = printer.inverseActuatorKinematics(angles[0], angles[1], angles[2]);
    EXPECT_NEAR(recovered[0], x, 0.1);
    EXPECT_NEAR(recovered[1], y, 0.1);
    EXPECT_NEAR(recovered[2], z, 0.1);
}

TEST_F(RotaryDeltaTest, RoundTripOffset) {
    double x = 30.0, y = 20.0, z = -180.0;
    auto angles = printer.forwardActuatorKinematics(x, y, z);
    auto recovered = printer.inverseActuatorKinematics(angles[0], angles[1], angles[2]);
    EXPECT_NEAR(recovered[0], x, 0.5);
    EXPECT_NEAR(recovered[1], y, 0.5);
    EXPECT_NEAR(recovered[2], z, 0.5);
}

TEST_F(RotaryDeltaTest, RoundTripFarOffset) {
    double x = 50.0, y = -40.0, z = -150.0;
    auto angles = printer.forwardActuatorKinematics(x, y, z);
    auto recovered = printer.inverseActuatorKinematics(angles[0], angles[1], angles[2]);
    EXPECT_NEAR(recovered[0], x, 1.0);
    EXPECT_NEAR(recovered[1], y, 1.0);
    EXPECT_NEAR(recovered[2], z, 1.0);
}

// Different Z heights should produce different angles.
TEST_F(RotaryDeltaTest, DifferentZProducesDifferentAngles) {
    auto a1 = printer.forwardActuatorKinematics(0.0, 0.0, -100.0);
    auto a2 = printer.forwardActuatorKinematics(0.0, 0.0, -300.0);
    // Lower Z (more negative) should produce different angles than higher Z.
    EXPECT_NE(a1[0], a2[0]);
}

// MotionTranslator integration: RotaryDelta kinematics should use the printer.
TEST_F(RotaryDeltaTest, MotionTranslatorUsesRotaryDelta) {
    kmotion::KinematicsTransform kt;
    kt.setKinematics(klippy::Kinematics::RotaryDelta);
    kt.setRotaryDeltaPrinter(&printer);

    auto stepperPos = kt.forwardActuatorKinematics(10.0, 5.0, -200.0);
    auto recovered = kt.inverseActuatorKinematics(stepperPos[0], stepperPos[1], stepperPos[2]);

    EXPECT_NEAR(recovered[0], 10.0, 0.5);
    EXPECT_NEAR(recovered[1], 5.0, 0.5);
    EXPECT_NEAR(recovered[2], -200.0, 0.5);
}

// Endstop adjustment should shift the angles.
TEST_F(RotaryDeltaTest, EndstopAdjustmentShiftsAngles) {
    klippy::RotaryDeltaEndstopAdjust adj;
    adj.adjA = 0.1; // ~5.7° offset on tower A
    printer.setEndstopAdjust(adj);

    auto angles = printer.forwardActuatorKinematics(0.0, 0.0, -200.0);
    // Tower A angle should differ from B and C by the adjustment.
    EXPECT_NEAR(angles[0] - angles[1], 0.1, 1e-6);
    EXPECT_NEAR(angles[0] - angles[2], 0.1, 1e-6);
}

// ============================================================================
// Config-driven wiring tests
// ============================================================================

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include <fstream>
#include <filesystem>
#include <string>

static std::string uniqueSocketPathRD() {
    return "/tmp/tether_test_rd_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfigRD(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/tether_test_rd_cfg_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter++) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

TEST(RotaryDeltaConfigTest, GeometryParsedFromConfig) {
    std::string cfgPath = createTempConfigRD(R"(
[printer]
kinematics = rotary_delta
max_velocity = 300
max_accel = 2000
[rotary_delta]
upper_arm_length = 180.0
forearm_length = 340.0
base_radius = 95.0
effector_radius = 25.0
base_height = 10.0
tower_a_angle = 210.0
tower_b_angle = 330.0
tower_c_angle = 90.0
angle_a = 0.5
angle_b = -0.3
angle_c = 0.1
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathRD();
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_EQ(s.kinematics, klippy::Kinematics::RotaryDelta);
    EXPECT_NEAR(s.rotaryDeltaGeometry.upperArmLength, 180.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.forearmLength, 340.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.baseRadius, 95.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.effectorRadius, 25.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.baseHeight, 10.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.towerAngleA, 210.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.towerAngleB, 330.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaGeometry.towerAngleC, 90.0, 1e-9);
    // Endstop adjustments are in radians (degrees * pi/180)
    EXPECT_NEAR(s.rotaryDeltaEndstopAdjust.adjA, 0.5 * M_PI / 180.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaEndstopAdjust.adjB, -0.3 * M_PI / 180.0, 1e-9);
    EXPECT_NEAR(s.rotaryDeltaEndstopAdjust.adjC, 0.1 * M_PI / 180.0, 1e-9);

    std::filesystem::remove(cfgPath);
}

TEST(RotaryDeltaConfigTest, KinematicsTransformWired) {
    std::string cfgPath = createTempConfigRD(R"(
[printer]
kinematics = rotary_delta
max_velocity = 300
max_accel = 2000
[rotary_delta]
upper_arm_length = 170.0
forearm_length = 320.0
base_radius = 90.0
effector_radius = 24.0
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathRD();
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    // The rotaryDeltaPrinter should have the geometry from config
    auto& rdp = instance->rotaryDeltaPrinter();
    EXPECT_NEAR(rdp.geometry().upperArmLength, 170.0, 1e-9);
    EXPECT_NEAR(rdp.geometry().forearmLength, 320.0, 1e-9);
    EXPECT_NEAR(rdp.geometry().baseRadius, 90.0, 1e-9);
    EXPECT_NEAR(rdp.geometry().effectorRadius, 24.0, 1e-9);

    // Round-trip through the printer should work
    auto angles = rdp.forwardActuatorKinematics(0.0, 0.0, -200.0);
    auto recovered = rdp.inverseActuatorKinematics(angles[0], angles[1], angles[2]);
    EXPECT_NEAR(recovered[0], 0.0, 0.5);
    EXPECT_NEAR(recovered[1], 0.0, 0.5);
    EXPECT_NEAR(recovered[2], -200.0, 0.5);

    std::filesystem::remove(cfgPath);
}
