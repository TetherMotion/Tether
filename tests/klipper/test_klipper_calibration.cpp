/**
 * @file test_klipper_calibration.cpp
 * @brief Tests for bed leveling, delta calibration, and Z tilt leveling
 *        with real probing implementations.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/BedLevel.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>

namespace klippy = tether::klipper::klippy;
namespace objects = tether::klipper::objects;

static std::string uniqueSocketPathCal() {
    return "/tmp/tether_test_cal_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfigCal(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/tether_test_cal_cfg_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter++) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

class CalibrationTest : public ::testing::Test {
protected:
    std::string cfgPath;
    std::unique_ptr<klippy::KlippyInstance> instance;

    void SetUp() override {
        cfgPath = createTempConfigCal(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[stepper_x]
position_max = 200
[stepper_y]
position_max = 200
[stepper_z]
position_max = 200
)");
        klippy::KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPathCal();
        cfg.settingsPath = "/tmp/tether_test_cal_settings.cfg";
        instance = std::make_unique<klippy::KlippyInstance>(cfg);
        instance->loadConfig(cfgPath);
    }

    void TearDown() override {
        std::filesystem::remove(cfgPath);
        std::filesystem::remove("/tmp/tether_test_cal_settings.cfg");
    }

    void setupProbe(double zOffset = 0.0) {
        auto probe = std::make_shared<objects::Probe>(0, []() { return false; });
        probe->setZOffset(zOffset);
        instance->setProbe(probe);
    }
};

// ============================================================================
// Bed leveling tests
// ============================================================================

TEST_F(CalibrationTest, BedMeshCalibrateFillsMesh) {
    setupProbe(0.0);
    instance->executeGcode("BED_MESH_CALIBRATE");

    auto& mesh = instance->bedMesh();
    EXPECT_GT(mesh.xPoints(), 0);
    EXPECT_GT(mesh.yPoints(), 0);
    EXPECT_TRUE(instance->settings().bedMeshEnabled);
}

TEST_F(CalibrationTest, G29ProbesAndFillsMesh) {
    setupProbe(0.5);
    instance->executeGcode("G29");

    auto& mesh = instance->bedMesh();
    EXPECT_GT(mesh.xPoints(), 0);
    EXPECT_GT(mesh.yPoints(), 0);
    EXPECT_TRUE(instance->settings().bedMeshEnabled);
}

TEST_F(CalibrationTest, BedLevelCallbackProbesMesh) {
    setupProbe(0.0);
    instance->executeGcode("G29");

    auto& mesh = instance->bedMesh();
    EXPECT_TRUE(instance->settings().bedMeshEnabled);
    auto points = mesh.points();
    EXPECT_FALSE(points.empty());
}

TEST_F(CalibrationTest, BedMeshCalibrateWithoutProbe) {
    instance->executeGcode("BED_MESH_CALIBRATE");
    EXPECT_TRUE(instance->settings().bedMeshEnabled);
}

// ============================================================================
// Z tilt leveling tests
// ============================================================================

TEST_F(CalibrationTest, ZTiltAdjustComputesAdjustments) {
    setupProbe(0.0);
    instance->executeGcode("Z_TILT_ADJUST");

    auto& adj = instance->settings().zTiltAdjustments;
    EXPECT_EQ(adj.size(), 3u);
    // With a uniform probe (all zeros), adjustments should be zero
    EXPECT_NEAR(adj[0], 0.0, 1e-9);
    EXPECT_NEAR(adj[1], 0.0, 1e-9);
    EXPECT_NEAR(adj[2], 0.0, 1e-9);
}

TEST_F(CalibrationTest, ZTiltAdjustWithoutProbe) {
    instance->executeGcode("Z_TILT_ADJUST");
    EXPECT_TRUE(instance->settings().zTiltAdjustments.empty());
}

// ============================================================================
// Delta calibration tests
// ============================================================================

class DeltaCalibrationTest : public ::testing::Test {
protected:
    std::string cfgPath;
    std::unique_ptr<klippy::KlippyInstance> instance;

    void SetUp() override {
        cfgPath = createTempConfigCal(R"(
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
        klippy::KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = uniqueSocketPathCal();
        cfg.settingsPath = "/tmp/tether_test_cal_delta_settings.cfg";
        instance = std::make_unique<klippy::KlippyInstance>(cfg);
        instance->loadConfig(cfgPath);
    }

    void TearDown() override {
        std::filesystem::remove(cfgPath);
        std::filesystem::remove("/tmp/tether_test_cal_delta_settings.cfg");
    }

    void setupProbe(double zOffset = 0.0) {
        auto probe = std::make_shared<objects::Probe>(0, []() { return false; });
        probe->setZOffset(zOffset);
        instance->setProbe(probe);
    }
};

TEST_F(DeltaCalibrationTest, DeltaCalibrateAdjustsEndstops) {
    setupProbe(0.0);

    auto& adjBefore = instance->settings().deltaEndstopAdjust;
    double initialX = adjBefore.adjX;
    double initialY = adjBefore.adjY;
    double initialZ = adjBefore.adjZ;

    instance->executeGcode("DELTA_CALIBRATE");

    // With uniform probe (all zeros), adjustments should be zero delta
    auto& adjAfter = instance->settings().deltaEndstopAdjust;
    EXPECT_NEAR(adjAfter.adjX, initialX, 1e-9);
    EXPECT_NEAR(adjAfter.adjY, initialY, 1e-9);
    EXPECT_NEAR(adjAfter.adjZ, initialZ, 1e-9);

    // The delta printer should have the updated adjustments
    auto& printer = instance->deltaPrinter();
    EXPECT_NEAR(printer.endstopAdjust().adjX, adjAfter.adjX, 1e-9);
    EXPECT_NEAR(printer.endstopAdjust().adjY, adjAfter.adjY, 1e-9);
    EXPECT_NEAR(printer.endstopAdjust().adjZ, adjAfter.adjZ, 1e-9);
}

TEST_F(DeltaCalibrationTest, DeltaCalibrateWithoutProbe) {
    instance->executeGcode("DELTA_CALIBRATE");
    auto& adj = instance->settings().deltaEndstopAdjust;
    EXPECT_NEAR(adj.adjX, 0.0, 1e-9);
    EXPECT_NEAR(adj.adjY, 0.0, 1e-9);
    EXPECT_NEAR(adj.adjZ, 0.0, 1e-9);
}
