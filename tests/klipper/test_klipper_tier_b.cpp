/**
 * @file test_klipper_tier_b.cpp
 * @brief Tests for Tier B features: extended commands, UDS endpoints,
 *        printer objects, and MotionReconstructor improvements.
 */

#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/motion/MotionReconstructor.hpp"
#include "tether/klipper/objects/Stepper.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace tether::klipper::klippy;
using namespace tether::klipper::motion;
using namespace tether::klipper::objects;

// ============================================================================
// B1: Extended G-code command tests
// ============================================================================

TEST(TierBExtendedCommands, ParsingKeyValueParams) {
    std::string line = "SET_SERVO SERVO=my_servo ANGLE=45.0";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;

    EXPECT_EQ(g.code, "SET_SERVO");
    EXPECT_TRUE(g.isExtendedCommand());
    EXPECT_TRUE(g.hasNamed("SERVO"));
    EXPECT_EQ(g.getNamed("SERVO"), "my_servo");
    EXPECT_TRUE(g.hasNamed("ANGLE"));
    EXPECT_NEAR(g.getNamedDouble("ANGLE"), 45.0, 0.001);
}

TEST(TierBExtendedCommands, NoParams) {
    std::string line = "BED_MESH_CLEAR";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;

    EXPECT_EQ(g.code, "BED_MESH_CLEAR");
    EXPECT_TRUE(g.isExtendedCommand());
    EXPECT_TRUE(g.namedParams.empty());
}

TEST(TierBExtendedCommands, MultipleParams) {
    std::string line = "SET_GCODE_OFFSET X=10.5 Y=20.0 Z=0.2 ADJUST=1";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;

    EXPECT_EQ(g.code, "SET_GCODE_OFFSET");
    EXPECT_TRUE(g.isExtendedCommand());
    EXPECT_NEAR(g.getNamedDouble("X"), 10.5, 0.001);
    EXPECT_NEAR(g.getNamedDouble("Y"), 20.0, 0.001);
    EXPECT_NEAR(g.getNamedDouble("Z"), 0.2, 0.001);
    EXPECT_EQ(g.getNamed("ADJUST"), "1");
}

TEST(TierBExtendedCommands, GetNamedInt) {
    std::string line = "SET_IDLE_TIMEOUT TIMEOUT=300";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;

    EXPECT_EQ(g.code, "SET_IDLE_TIMEOUT");
    EXPECT_EQ(g.getNamedInt("TIMEOUT", 0), 300);
}

TEST(TierBExtendedCommands, DefaultValues) {
    std::string line = "SET_HEATER_TEMPERATURE HEATER=extruder";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;

    EXPECT_EQ(g.code, "SET_HEATER_TEMPERATURE");
    EXPECT_EQ(g.getNamed("HEATER", ""), "extruder");
    EXPECT_NEAR(g.getNamedDouble("TARGET", 200.0), 200.0, 0.001);
}

TEST(TierBExtendedCommands, RegularGcodeNotExtended) {
    std::string line = "G1 X10 Y20 F1500";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;

    EXPECT_FALSE(g.isExtendedCommand());
    EXPECT_TRUE(g.namedParams.empty());
}

// ============================================================================
// B2: UDS endpoint tests
// ============================================================================

TEST(TierBUds, TemperatureStoreRecord) {
    KlippyServer server;
    server.recordTemperature("extruder", 25.0, 0.0);
    server.recordTemperature("extruder", 30.0, 200.0);
    server.recordTemperature("heater_bed", 60.0, 60.0);

    // Just verify it doesn't crash
    SUCCEED();
}

TEST(TierBUds, GcodeStoreRecord) {
    KlippyServer server;
    server.emitGcodeResponse("Test message 1");
    server.emitGcodeResponse("Test message 2");

    auto endpoints = server.listEndpoints();
    auto it = std::find(endpoints.begin(), endpoints.end(), "server/gcode_store");
    EXPECT_NE(it, endpoints.end());
}

TEST(TierBUds, PowerDevices) {
    KlippyServer server;
    server.registerPowerDevice("psu", "off");
    server.registerPowerDevice("light", "on");

    EXPECT_TRUE(server.setPowerDeviceState("psu", "on"));
    EXPECT_FALSE(server.setPowerDeviceState("nonexistent", "on"));
}

TEST(TierBUds, NewEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();

    auto hasEndpoint = [&](const std::string& name) {
        return std::find(endpoints.begin(), endpoints.end(), name) != endpoints.end();
    };

    EXPECT_TRUE(hasEndpoint("server/temperature_store"));
    EXPECT_TRUE(hasEndpoint("server/gcode_store"));
    EXPECT_TRUE(hasEndpoint("server/files/directory"));
    EXPECT_TRUE(hasEndpoint("server/files/move"));
    EXPECT_TRUE(hasEndpoint("server/files/copy"));
    EXPECT_TRUE(hasEndpoint("server/files/delete"));
    EXPECT_TRUE(hasEndpoint("server/files/upload"));
    EXPECT_TRUE(hasEndpoint("machine/reboot"));
    EXPECT_TRUE(hasEndpoint("machine/shutdown"));
    EXPECT_TRUE(hasEndpoint("machine/update/status"));
    EXPECT_TRUE(hasEndpoint("machine/device_power/devices"));
    EXPECT_TRUE(hasEndpoint("machine/device_power/state"));
    EXPECT_TRUE(hasEndpoint("printer/info"));
    EXPECT_TRUE(hasEndpoint("printer/subscriptions"));

    // Printer alias endpoints
    EXPECT_TRUE(hasEndpoint("printer/gcode/script"));
    EXPECT_TRUE(hasEndpoint("printer/objects/list"));
    EXPECT_TRUE(hasEndpoint("printer/objects/query"));
    EXPECT_TRUE(hasEndpoint("printer/objects/subscribe"));
    EXPECT_TRUE(hasEndpoint("printer/print/start"));
    EXPECT_TRUE(hasEndpoint("printer/print/cancel"));
    EXPECT_TRUE(hasEndpoint("printer/print/pause"));
    EXPECT_TRUE(hasEndpoint("printer/print/resume"));
}

TEST(TierBUds, FilesListWithTempDir) {
    std::string testDir = "/tmp/tether_test_files_" + std::to_string(getpid());
    std::filesystem::create_directories(testDir);
    std::ofstream f1(testDir + "/test1.gcode");
    f1 << "G28\nG1 X10 Y10\n";
    f1.close();

    KlippyServer server;
    server.setFileRoot(testDir);

    auto endpoints = server.listEndpoints();
    auto it = std::find(endpoints.begin(), endpoints.end(), "server/files/list");
    EXPECT_NE(it, endpoints.end());

    std::filesystem::remove_all(testDir);
}

TEST(TierBUds, UpdateStatus) {
    KlippyServer server;
    server.setUpdateStatus("klipper", "up_to_date");
    server.setUpdateStatus("moonraker", "update_available");
    SUCCEED();
}

// ============================================================================
// B3+B4: Printer object tests
// ============================================================================

TEST(TierBPrinterObjects, NewObjectsExist) {
    EXPECT_EQ(std::make_shared<OutputPinObject>("output_pin")->name(), "output_pin");
    EXPECT_EQ(std::make_shared<PwmToolObject>("pwm_tool")->name(), "pwm_tool");
    EXPECT_EQ(std::make_shared<TemperatureFanObject>("temperature_fan")->name(), "temperature_fan");
    EXPECT_EQ(std::make_shared<ControllerFanObject>("controller_fan")->name(), "controller_fan");
    EXPECT_EQ(std::make_shared<HeaterFanObject>("heater_fan")->name(), "heater_fan");
    EXPECT_EQ(std::make_shared<FanGenericObject>("fan_generic")->name(), "fan_generic");
    EXPECT_EQ(std::make_shared<LedObject>("led")->name(), "led");
    EXPECT_EQ(std::make_shared<DotstarObject>("dotstar")->name(), "dotstar");
    EXPECT_EQ(std::make_shared<ServoObject>("servo")->name(), "servo");
    EXPECT_EQ(std::make_shared<BltouchObject>()->name(), "bltouch");
    EXPECT_EQ(std::make_shared<ZTiltObject>()->name(), "z_tilt");
    EXPECT_EQ(std::make_shared<QuadGantryLevelObject>()->name(), "quad_gantry_level");
    EXPECT_EQ(std::make_shared<ScrewsTiltAdjustObject>()->name(), "screws_tilt_adjust");
    EXPECT_EQ(std::make_shared<BedScrewsObject>()->name(), "bed_screws");
    EXPECT_EQ(std::make_shared<DeltaCalibrateObject>()->name(), "delta_calibrate");
    EXPECT_EQ(std::make_shared<SkewCorrectionObject>()->name(), "skew_correction");
    EXPECT_EQ(std::make_shared<InputShaperObject>()->name(), "input_shaper");
    EXPECT_EQ(std::make_shared<PressureAdvanceObject>()->name(), "pressure_advance");
    EXPECT_EQ(std::make_shared<ExcludeObjectObject>()->name(), "exclude_object");
    EXPECT_EQ(std::make_shared<ZThermalAdjustObject>()->name(), "z_thermal_adjust");
    EXPECT_EQ(std::make_shared<HeaterGenericObject>("heater_generic")->name(), "heater_generic");
    EXPECT_EQ(std::make_shared<TemperatureProbeObject>("temperature_probe")->name(), "temperature_probe");
    EXPECT_EQ(std::make_shared<ForceMoveObject>()->name(), "force_move");
    EXPECT_EQ(std::make_shared<DualCarriageObject>()->name(), "dual_carriage");
    EXPECT_EQ(std::make_shared<ExtruderStepperObject>("extruder_stepper")->name(), "extruder_stepper");
    EXPECT_EQ(std::make_shared<ManualStepperObject>("manual_stepper")->name(), "manual_stepper");
    EXPECT_EQ(std::make_shared<EndstopPhaseObject>("endstop_phase")->name(), "endstop_phase");
    EXPECT_EQ(std::make_shared<SafeZHomeObject>()->name(), "safe_z_home");
    EXPECT_EQ(std::make_shared<BedTiltObject>()->name(), "bed_tilt");
    EXPECT_EQ(std::make_shared<MultiPinObject>("multi_pin")->name(), "multi_pin");
    EXPECT_EQ(std::make_shared<ButtonObject>("button")->name(), "button");
    EXPECT_EQ(std::make_shared<SmartEffectorObject>()->name(), "smart_effector");
    EXPECT_EQ(std::make_shared<TmcDriverObject>("tmc2209")->name(), "tmc2209");
}

TEST(TierBPrinterObjects, ServoFields) {
    auto servo = std::make_shared<ServoObject>("servo");
    servo->setAngle(90.0);
    servo->setWidth(1500.0);

    auto status = servo->status({});
    ASSERT_TRUE(status.find("angle") != status.end());
    EXPECT_NEAR(status["angle"].asDouble(), 90.0, 0.001);
    ASSERT_TRUE(status.find("width") != status.end());
    EXPECT_NEAR(status["width"].asDouble(), 1500.0, 0.001);
}

TEST(TierBPrinterObjects, TemperatureFanFields) {
    auto tempFan = std::make_shared<TemperatureFanObject>("temperature_fan");
    tempFan->setTemperature(45.0);
    tempFan->setTarget(50.0);
    tempFan->setSpeed(0.75);

    auto status = tempFan->status({});
    EXPECT_NEAR(status["temperature"].asDouble(), 45.0, 0.001);
    EXPECT_NEAR(status["target"].asDouble(), 50.0, 0.001);
    EXPECT_NEAR(status["speed"].asDouble(), 0.75, 0.001);
}

TEST(TierBPrinterObjects, InputShaperFields) {
    auto inputShaper = std::make_shared<InputShaperObject>();
    inputShaper->setShaperFreqX(45.0);
    inputShaper->setShaperTypeX("ei");
    inputShaper->setShaperFreqY(55.0);
    inputShaper->setShaperTypeY("mzv");

    auto status = inputShaper->status({});
    EXPECT_NEAR(status["shaper_freq_x"].asDouble(), 45.0, 0.001);
    EXPECT_EQ(status["shaper_type_x"].asString(), "ei");
    EXPECT_NEAR(status["shaper_freq_y"].asDouble(), 55.0, 0.001);
    EXPECT_EQ(status["shaper_type_y"].asString(), "mzv");
}

TEST(TierBPrinterObjects, ToolheadNewFields) {
    ToolheadObject toolhead;
    toolhead.setMaxVelocity(500.0);
    toolhead.setMaxAccel(3000.0);
    toolhead.setMaxAccelToDecel(1500.0);
    toolhead.setPrintTime(10.5);
    toolhead.setEstimatedPrintTime(12.0);
    toolhead.setStalls(2);

    auto status = toolhead.status({});
    ASSERT_TRUE(status.find("max_velocity") != status.end());
    EXPECT_NEAR(status["max_velocity"].asDouble(), 500.0, 0.001);
    ASSERT_TRUE(status.find("max_accel_to_decel") != status.end());
    EXPECT_NEAR(status["max_accel_to_decel"].asDouble(), 1500.0, 0.001);
    ASSERT_TRUE(status.find("print_time") != status.end());
    EXPECT_NEAR(status["print_time"].asDouble(), 10.5, 0.001);
    ASSERT_TRUE(status.find("stalls") != status.end());
    EXPECT_EQ(status["stalls"].asInt(), 2);
}

TEST(TierBPrinterObjects, ConfigfilePendingItems) {
    ConfigfileObject configfile;
    configfile.setSaveConfigPending(true);
    std::map<std::string, std::string> items;
    items["stepper_z position_endstop"] = "0.5";
    items["probe z_offset"] = "0.1";
    configfile.setSaveConfigPendingItems(items);

    auto status = configfile.status({});
    ASSERT_TRUE(status.find("save_config_pending") != status.end());
    EXPECT_TRUE(status["save_config_pending"].asBool());
    ASSERT_TRUE(status.find("save_config_pending_items") != status.end());
}

TEST(TierBPrinterObjects, GcodeMoveNewFields) {
    GcodeMoveObject gcodeMove;
    gcodeMove.setPosition({10, 20, 30, 0});
    gcodeMove.setGcodePosition({10, 20, 30, 0});

    auto status = gcodeMove.status({});
    ASSERT_TRUE(status.find("homing_origin") != status.end());
    ASSERT_TRUE(status.find("scale") != status.end());
}

TEST(TierBPrinterObjects, ExcludeObjectFields) {
    auto excludeObj = std::make_shared<ExcludeObjectObject>();
    excludeObj->setCurrentObject("cube");
    excludeObj->setExcludedObjects({"cube", "cylinder"});
    excludeObj->setObjects({"cube", "cylinder", "sphere"});

    auto status = excludeObj->status({});
    EXPECT_EQ(status["current_object"].asString(), "cube");
    ASSERT_TRUE(status.find("objects") != status.end());
}

// ============================================================================
// B6: MotionReconstructor tests
// ============================================================================

TEST(TierBMotionReconstructor, BasicReconstruction) {
    std::vector<StepCommand> steps;
    StepCommand cmd1;
    cmd1.interval = 1000;
    cmd1.add = 0;
    cmd1.count = 10;
    steps.push_back(cmd1);

    auto traj = MotionReconstructor::reconstruct(steps, 0, 100.0);
    EXPECT_EQ(traj.size(), 10u);
    EXPECT_NEAR(traj[0].velocity, 1.0 / 1000.0, 0.0001);
    EXPECT_NEAR(traj[9].velocity, 1.0 / 1000.0, 0.0001);
}

TEST(TierBMotionReconstructor, AccelerationComputation) {
    std::vector<StepCommand> steps;
    StepCommand cmd;
    cmd.interval = 500;
    cmd.add = 10;  // interval increases = deceleration
    cmd.count = 20;
    steps.push_back(cmd);

    auto traj = MotionReconstructor::reconstruct(steps, 0, 100.0);
    EXPECT_EQ(traj.size(), 20u);

    auto accel = MotionReconstructor::computeAcceleration(traj);
    EXPECT_EQ(accel.size(), 20u);
    EXPECT_NEAR(accel[0], 0.0, 0.0001);
    // With increasing intervals, velocity decreases -> negative acceleration
    EXPECT_LT(accel[10], 0.0);
}

TEST(TierBMotionReconstructor, VelocitySmoothing) {
    std::vector<StepCommand> steps;
    StepCommand cmd1{500, 5, 0};   // interval=500, count=5, add=0
    steps.push_back(cmd1);
    StepCommand cmd2{1000, 5, 0};  // interval=1000, count=5, add=0
    steps.push_back(cmd2);
    StepCommand cmd3{500, 5, 0};   // interval=500, count=5, add=0
    steps.push_back(cmd3);

    auto traj = MotionReconstructor::reconstruct(steps, 0, 100.0);
    EXPECT_EQ(traj.size(), 15u);

    double beforeMaxDiff = 0.0;
    for (size_t i = 1; i < traj.size(); ++i) {
        double diff = std::abs(traj[i].velocity - traj[i - 1].velocity);
        if (diff > beforeMaxDiff) beforeMaxDiff = diff;
    }

    MotionReconstructor::smoothVelocities(traj, 5);

    double afterMaxDiff = 0.0;
    for (size_t i = 1; i < traj.size(); ++i) {
        double diff = std::abs(traj[i].velocity - traj[i - 1].velocity);
        if (diff > afterMaxDiff) afterMaxDiff = diff;
    }

    EXPECT_LT(afterMaxDiff, beforeMaxDiff);
}

TEST(TierBMotionReconstructor, StatsComputation) {
    std::vector<StepCommand> steps;
    StepCommand cmd;
    cmd.interval = 1000;
    cmd.add = 0;
    cmd.count = 100;
    steps.push_back(cmd);

    auto traj = MotionReconstructor::reconstruct(steps, 0, 100.0);
    auto stats = MotionReconstructor::computeStats(traj);

    EXPECT_GT(stats.maxVelocity, 0.0);
    EXPECT_GE(stats.minVelocity, 0.0);
    EXPECT_GT(stats.avgVelocity, 0.0);
    EXPECT_GT(stats.totalDistance, 0.0);
    EXPECT_GT(stats.totalTime, 0.0);

    // For constant velocity, max == min == avg
    EXPECT_NEAR(stats.maxVelocity, stats.minVelocity, 0.0001);
    EXPECT_NEAR(stats.avgVelocity, stats.maxVelocity, 0.0001);
}

TEST(TierBMotionReconstructor, EmptyTrajectory) {
    std::vector<StepCommand> empty;
    auto traj = MotionReconstructor::reconstruct(empty, 0, 100.0);
    EXPECT_TRUE(traj.empty());

    auto stats = MotionReconstructor::computeStats(traj);
    EXPECT_NEAR(stats.maxVelocity, 0.0, 0.0001);
    EXPECT_NEAR(stats.totalDistance, 0.0, 0.0001);

    auto accel = MotionReconstructor::computeAcceleration(traj);
    EXPECT_TRUE(accel.empty());
}

TEST(TierBMotionReconstructor, SinglePoint) {
    std::vector<StepCommand> steps;
    StepCommand cmd;
    cmd.interval = 1000;
    cmd.add = 0;
    cmd.count = 1;
    steps.push_back(cmd);

    auto traj = MotionReconstructor::reconstruct(steps, 0, 100.0);
    EXPECT_EQ(traj.size(), 1u);

    auto accel = MotionReconstructor::computeAcceleration(traj);
    EXPECT_EQ(accel.size(), 1u);
    EXPECT_NEAR(accel[0], 0.0, 0.0001);
}
