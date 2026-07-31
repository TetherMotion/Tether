/**
 * @file test_klipper_missing_features.cpp
 * @brief Tests for delta printer, TMC config, filament load/unload,
 *        multi-MCU, config validation, peripheral wrappers, and
 *        additional missing G-codes.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/objects/TmcUart.hpp"
#include "tether/klipper/config/ConfigParser.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>

using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;
using namespace tether::klipper::config;

// ============================================================================
// Helper
// ============================================================================
static std::string uniqueSocketPath() {
    return "/tmp/tether_test_mf_" + std::to_string(getpid()) + ".sock";
}

// ============================================================================
// Delta printer tests
// ============================================================================
TEST(KlipperDelta, DefaultGeometry) {
    DeltaPrinter delta;
    auto geo = delta.geometry();
    EXPECT_NEAR(geo.armLength, 250.0, 0.1);
    EXPECT_NEAR(geo.deltaRadius, 125.0, 0.1);
}

TEST(KlipperDelta, SetGeometry) {
    DeltaPrinter delta;
    DeltaGeometry geo;
    geo.armLength = 300.0;
    geo.deltaRadius = 150.0;
    geo.towerAngleA = 1.0;
    geo.towerAngleB = -1.0;
    geo.towerAngleC = 0.5;
    delta.setGeometry(geo);
    EXPECT_NEAR(delta.geometry().armLength, 300.0, 0.1);
    EXPECT_NEAR(delta.geometry().deltaRadius, 150.0, 0.1);
    EXPECT_NEAR(delta.geometry().towerAngleA, 1.0, 0.01);
}

TEST(KlipperDelta, SetEndstopAdjust) {
    DeltaPrinter delta;
    DeltaEndstopAdjust adj;
    adj.adjX = 0.1;
    adj.adjY = -0.2;
    adj.adjZ = 0.3;
    delta.setEndstopAdjust(adj);
    EXPECT_NEAR(delta.endstopAdjust().adjX, 0.1, 0.001);
    EXPECT_NEAR(delta.endstopAdjust().adjY, -0.2, 0.001);
    EXPECT_NEAR(delta.endstopAdjust().adjZ, 0.3, 0.001);
}

TEST(KlipperDelta, CartesianToTowerAtCenter) {
    DeltaPrinter delta;
    // At center (0, 0, 0), all towers should be at sqrt(armLength^2 - deltaRadius^2)
    auto towers = delta.forwardActuatorKinematics(0, 0, 0);
    double expected = std::sqrt(250.0 * 250.0 - 125.0 * 125.0);
    EXPECT_NEAR(towers[0], expected, 0.1);
    EXPECT_NEAR(towers[1], expected, 0.1);
    EXPECT_NEAR(towers[2], expected, 0.1);
}

TEST(KlipperDelta, CartesianToTowerWithEndstopAdjust) {
    DeltaPrinter delta;
    DeltaEndstopAdjust adj;
    adj.adjX = 1.0;
    adj.adjY = 2.0;
    adj.adjZ = 3.0;
    delta.setEndstopAdjust(adj);
    auto towers = delta.forwardActuatorKinematics(0, 0, 0);
    double baseExpected = std::sqrt(250.0 * 250.0 - 125.0 * 125.0);
    EXPECT_NEAR(towers[0], baseExpected + 1.0, 0.1);
    EXPECT_NEAR(towers[1], baseExpected + 2.0, 0.1);
    EXPECT_NEAR(towers[2], baseExpected + 3.0, 0.1);
}

TEST(KlipperDelta, TowerToCartesianRoundTrip) {
    DeltaPrinter delta;
    // Forward: Cartesian -> Tower
    double x = 10.0, y = 5.0, z = 50.0;
    auto towers = delta.forwardActuatorKinematics(x, y, z);
    // Inverse: Tower -> Cartesian
    auto cartesian = delta.inverseActuatorKinematics(towers[0], towers[1], towers[2]);
    EXPECT_NEAR(cartesian[0], x, 0.5);
    EXPECT_NEAR(cartesian[1], y, 0.5);
    EXPECT_NEAR(cartesian[2], z, 0.5);
}

TEST(KlipperDelta, M665SetsGeometry) {
    GcodeCallbacks cb;
    DeltaPrinter delta;
    cb.setDeltaGeometry = [&](double arm, double radius,
                               double a, double b, double c) {
        DeltaGeometry geo;
        geo.armLength = arm;
        geo.deltaRadius = radius;
        geo.towerAngleA = a;
        geo.towerAngleB = b;
        geo.towerAngleC = c;
        delta.setGeometry(geo);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M665 L300 R150 A1 B-1 C0.5");
    EXPECT_NEAR(delta.geometry().armLength, 300.0, 0.1);
    EXPECT_NEAR(delta.geometry().deltaRadius, 150.0, 0.1);
    EXPECT_NEAR(delta.geometry().towerAngleA, 1.0, 0.01);
}

TEST(KlipperDelta, M666SetsEndstopAdjust) {
    GcodeCallbacks cb;
    DeltaPrinter delta;
    cb.setDeltaEndstopAdjust = [&](double x, double y, double z) {
        DeltaEndstopAdjust adj;
        adj.adjX = x; adj.adjY = y; adj.adjZ = z;
        delta.setEndstopAdjust(adj);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M666 X0.1 Y-0.2 Z0.3");
    EXPECT_NEAR(delta.endstopAdjust().adjX, 0.1, 0.001);
    EXPECT_NEAR(delta.endstopAdjust().adjY, -0.2, 0.001);
    EXPECT_NEAR(delta.endstopAdjust().adjZ, 0.3, 0.001);
}

// ============================================================================
// TMC driver configuration tests
// ============================================================================
TEST(KlipperTmc, DefaultParams) {
    TmcDriverConfig tmc;
    auto params = tmc.params("x");
    EXPECT_NEAR(params.runCurrent, 800.0, 0.1);
    EXPECT_NEAR(params.holdCurrent, 400.0, 0.1);
    EXPECT_FALSE(params.stealthChop);
}

TEST(KlipperTmc, SetRunCurrent) {
    TmcDriverConfig tmc;
    tmc.setRunCurrent("x", 1000.0);
    EXPECT_NEAR(tmc.params("x").runCurrent, 1000.0, 0.1);
}

TEST(KlipperTmc, SetHoldCurrent) {
    TmcDriverConfig tmc;
    tmc.setHoldCurrent("y", 500.0);
    EXPECT_NEAR(tmc.params("y").holdCurrent, 500.0, 0.1);
}

TEST(KlipperTmc, SetStealthChop) {
    TmcDriverConfig tmc;
    tmc.setStealthChop("x", true);
    EXPECT_TRUE(tmc.params("x").stealthChop);
    tmc.setStealthChop("x", false);
    EXPECT_FALSE(tmc.params("x").stealthChop);
}

TEST(KlipperTmc, SetSpreadThreshold) {
    TmcDriverConfig tmc;
    tmc.setSpreadThreshold("z", 100.0);
    EXPECT_NEAR(tmc.params("z").spreadThreshold, 100.0, 0.1);
}

TEST(KlipperTmc, SetBumpSensitivity) {
    TmcDriverConfig tmc;
    tmc.setBumpSensitivity("e", 5);
    EXPECT_EQ(tmc.params("e").bumpSensitivity, 5);
}

TEST(KlipperTmc, SetDiagPin) {
    TmcDriverConfig tmc;
    tmc.setDiagPin("x", 3);
    EXPECT_EQ(tmc.params("x").diagPin, 3);
}

TEST(KlipperTmc, M907SetsCurrent) {
    GcodeCallbacks cb;
    TmcDriverConfig tmc;
    cb.setTmcCurrent = [&](const std::string& axis, double cur) {
        tmc.setRunCurrent(axis, cur);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M907 X900 Y800");
    EXPECT_NEAR(tmc.params("x").runCurrent, 900.0, 0.1);
    EXPECT_NEAR(tmc.params("y").runCurrent, 800.0, 0.1);
}

TEST(KlipperTmc, M911SetsStealthChop) {
    GcodeCallbacks cb;
    TmcDriverConfig tmc;
    cb.setTmcStealthChop = [&](const std::string& axis, bool enable) {
        tmc.setStealthChop(axis, enable);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M911 S1 X");
    EXPECT_TRUE(tmc.params("x").stealthChop);
}

// ============================================================================
// Filament load/unload tests
// ============================================================================
TEST(KlipperFilament, LoadUnload) {
    FilamentLoader loader;
    int loadedExtruder = -1;
    int unloadedExtruder = -1;
    loader.setLoadCallback([&](int e, double, double) { loadedExtruder = e; });
    loader.setUnloadCallback([&](int e, double, double) { unloadedExtruder = e; });

    loader.loadFilament(0);
    EXPECT_EQ(loadedExtruder, 0);
    EXPECT_TRUE(loader.isLoaded(0));

    loader.unloadFilament(0);
    EXPECT_EQ(unloadedExtruder, 0);
    EXPECT_FALSE(loader.isLoaded(0));
}

TEST(KlipperFilament, LoadToTool) {
    FilamentLoader loader;
    int tool = -1;
    loader.setLoadCallback([&](int t, double, double) { tool = t; });
    loader.loadToTool(1);
    EXPECT_EQ(tool, 1);
}

TEST(KlipperFilament, Purge) {
    FilamentLoader loader;
    int purged = -1;
    loader.setPurgeCallback([&](int e, double, double) { purged = e; });
    loader.purge(0);
    EXPECT_EQ(purged, 0);
}

TEST(KlipperFilament, Retract) {
    FilamentLoader loader;
    int retracted = -1;
    loader.setRetractCallback([&](int e, double, double) { retracted = e; });
    loader.retract(0);
    EXPECT_EQ(retracted, 0);
}

TEST(KlipperFilament, SensorState) {
    FilamentLoader loader;
    loader.setSensorState(0, true);
    loader.setSensorState(1, false);
    auto report = loader.reportSensorState();
    EXPECT_TRUE(report.find("S0:ON") != std::string::npos);
    EXPECT_TRUE(report.find("S1:OFF") != std::string::npos);
}

TEST(KlipperFilament, M701LoadFilament) {
    GcodeCallbacks cb;
    FilamentLoader loader;
    cb.loadFilament = [&](int e) { loader.loadFilament(e); };
    GCodeExecutor exec(cb);
    exec.executeLine("M701 T0");
    EXPECT_TRUE(loader.isLoaded(0));
}

TEST(KlipperFilament, M702UnloadFilament) {
    GcodeCallbacks cb;
    FilamentLoader loader;
    cb.loadFilament = [&](int e) { loader.loadFilament(e); };
    cb.unloadFilament = [&](int e) { loader.unloadFilament(e); };
    GCodeExecutor exec(cb);
    exec.executeLine("M701 T0");
    EXPECT_TRUE(loader.isLoaded(0));
    exec.executeLine("M702 T0");
    EXPECT_FALSE(loader.isLoaded(0));
}

TEST(KlipperFilament, M708ReportSensorState) {
    std::string output;
    GcodeCallbacks cb;
    FilamentLoader loader;
    loader.setSensorState(0, true);
    cb.reportFilamentSensorState = [&]() { return loader.reportSensorState(); };
    cb.output = [&](const std::string& msg) { output = msg; };
    GCodeExecutor exec(cb);
    exec.executeLine("M708");
    EXPECT_TRUE(output.find("S0:ON") != std::string::npos);
}

// ============================================================================
// Multi-MCU coordination tests
// ============================================================================
TEST(KlipperMultiMcu, SetSerialPath) {
    MultiMcuManager mgr;
    mgr.setSerialPath(1, "/dev/ttyUSB1");
    auto mcu = mgr.getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_EQ(mcu->serialPath, "/dev/ttyUSB1");
}

TEST(KlipperMultiMcu, SetBaudRate) {
    MultiMcuManager mgr;
    mgr.setBaudRate(1, 115200);
    auto mcu = mgr.getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_EQ(mcu->baudRate, 115200);
}

TEST(KlipperMultiMcu, EnableDisable) {
    MultiMcuManager mgr;
    mgr.setEnabled(1, true);
    EXPECT_TRUE(mgr.getMcu(1)->connected);
    mgr.setEnabled(1, false);
    EXPECT_FALSE(mgr.getMcu(1)->connected);
}

TEST(KlipperMultiMcu, SetClockFreq) {
    MultiMcuManager mgr;
    mgr.setClockFreq(1, 72000000);
    auto mcu = mgr.getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_EQ(mcu->clockFreq, 72000000u);
}

TEST(KlipperMultiMcu, GetStatus) {
    MultiMcuManager mgr;
    mgr.setSerialPath(1, "/dev/ttyUSB1");
    mgr.setBaudRate(1, 250000);
    mgr.setEnabled(1, true);
    auto status = mgr.getStatus(1);
    EXPECT_TRUE(status.find("connected") != std::string::npos);
    EXPECT_TRUE(status.find("/dev/ttyUSB1") != std::string::npos);
}

TEST(KlipperMultiMcu, GetStatusNotConfigured) {
    MultiMcuManager mgr;
    auto status = mgr.getStatus(99);
    EXPECT_TRUE(status.find("not configured") != std::string::npos);
}

TEST(KlipperMultiMcu, UpdateStats) {
    MultiMcuManager mgr;
    mgr.updateStats(1, 1024, 2048, 3);
    auto mcu = mgr.getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_EQ(mcu->bytesRead, 1024u);
    EXPECT_EQ(mcu->bytesWrite, 2048u);
    EXPECT_EQ(mcu->retransmits, 3u);
}

TEST(KlipperMultiMcu, M860SetsSerial) {
    GcodeCallbacks cb;
    MultiMcuManager mgr;
    cb.setSecondaryMcuSerial = [&](int id, const std::string& path) {
        mgr.setSerialPath(id, path);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M860 S1 P1234");
    // The path is converted from the P parameter as a string
    auto mcu = mgr.getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_FALSE(mcu->serialPath.empty());
}

TEST(KlipperMultiMcu, M876GetStatus) {
    std::string output;
    GcodeCallbacks cb;
    MultiMcuManager mgr;
    mgr.setSerialPath(1, "/dev/ttyUSB1");
    mgr.setEnabled(1, true);
    cb.getSecondaryMcuStatus = [&](int id) { return mgr.getStatus(id); };
    cb.output = [&](const std::string& msg) { output = msg; };
    GCodeExecutor exec(cb);
    exec.executeLine("M876 S1");
    EXPECT_TRUE(output.find("connected") != std::string::npos);
}

// ============================================================================
// Config validation tests
// ============================================================================
TEST(KlipperConfigValidator, ValidStepperSection) {
    ConfigParser parser;
    parser.parse("[stepper_x]\n"
                 "step_pin: PA0\n"
                 "dir_pin: PA1\n"
                 "rotation_distance: 40\n"
                 "microsteps: 16\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
    EXPECT_TRUE(results[0].errors.empty());
}

TEST(KlipperConfigValidator, MissingRequiredKey) {
    ConfigParser parser;
    parser.parse("[stepper_x]\n"
                 "step_pin: PA0\n"
                 "dir_pin: PA1\n"
                 "# Missing rotation_distance\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results[0].valid);
    EXPECT_TRUE(results[0].errors.size() > 0);
    EXPECT_TRUE(results[0].errors[0].find("rotation_distance") != std::string::npos);
}

TEST(KlipperConfigValidator, InvalidMicrosteps) {
    ConfigParser parser;
    parser.parse("[stepper_x]\n"
                 "step_pin: PA0\n"
                 "dir_pin: PA1\n"
                 "rotation_distance: 40\n"
                 "microsteps: 0\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results[0].valid);
}

TEST(KlipperConfigValidator, ValidPrinterSection) {
    ConfigParser parser;
    parser.parse("[printer]\n"
                 "kinematics: cartesian\n"
                 "max_velocity: 300\n"
                 "max_accel: 3000\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
}

TEST(KlipperConfigValidator, UnknownKinematics) {
    ConfigParser parser;
    parser.parse("[printer]\n"
                 "kinematics: weird_kinematics\n"
                 "max_velocity: 300\n"
                 "max_accel: 3000\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid); // Warning, not error
    EXPECT_TRUE(results[0].warnings.size() > 0);
}

TEST(KlipperConfigValidator, ValidExtruderSection) {
    ConfigParser parser;
    parser.parse("[extruder]\n"
                 "step_pin: PA2\n"
                 "dir_pin: PA3\n"
                 "rotation_distance: 33.5\n"
                 "nozzle_diameter: 0.4\n"
                 "filament_diameter: 1.75\n"
                 "heater_pin: PB1\n"
                 "sensor_type: EPCOS 100K B57560G104F\n"
                 "sensor_pin: PA0\n"
                 "min_temp: 0\n"
                 "max_temp: 300\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
}

TEST(KlipperConfigValidator, MinTempGreaterThanMaxTemp) {
    ConfigParser parser;
    parser.parse("[heater_bed]\n"
                 "heater_pin: PB2\n"
                 "sensor_type: EPCOS 100K B57560G104F\n"
                 "sensor_pin: PA1\n"
                 "min_temp: 100\n"
                 "max_temp: 50\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results[0].valid);
}

TEST(KlipperConfigValidator, ValidBedMeshSection) {
    ConfigParser parser;
    parser.parse("[bed_mesh]\n"
                 "mesh_min: 10, 10\n"
                 "mesh_max: 200, 200\n"
                 "probe_count: 5, 5\n"
                 "mesh_speed: 120\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
}

TEST(KlipperConfigValidator, ValidMcuSection) {
    ConfigParser parser;
    parser.parse("[mcu]\n"
                 "serial: /dev/serial/by-id/usb-Klipper\n"
                 "baud: 250000\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
}

TEST(KlipperConfigValidator, NonStandardBaudWarning) {
    ConfigParser parser;
    parser.parse("[mcu]\n"
                 "serial: /dev/ttyUSB0\n"
                 "baud: 12345\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid); // Warning, not error
    EXPECT_TRUE(results[0].warnings.size() > 0);
}

TEST(KlipperConfigValidator, UnknownSectionWarning) {
    ConfigParser parser;
    parser.parse("[some_unknown_section]\n"
                 "key: value\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
    EXPECT_TRUE(results[0].warnings.size() > 0);
}

TEST(KlipperConfigValidator, AllValid) {
    ConfigParser parser;
    parser.parse("[printer]\n"
                 "kinematics: cartesian\n"
                 "max_velocity: 300\n"
                 "max_accel: 3000\n"
                 "[stepper_x]\n"
                 "step_pin: PA0\n"
                 "dir_pin: PA1\n"
                 "rotation_distance: 40\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    EXPECT_TRUE(validator.allValid(results));
}

TEST(KlipperConfigValidator, FormatErrors) {
    ConfigParser parser;
    parser.parse("[stepper_x]\n"
                 "step_pin: PA0\n"
                 "# Missing required keys\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    auto errors = validator.formatErrors(results);
    EXPECT_TRUE(errors.find("ERROR") != std::string::npos);
    EXPECT_TRUE(errors.find("stepper_x") != std::string::npos);
}

TEST(KlipperConfigValidator, TmcSectionValidation) {
    ConfigParser parser;
    parser.parse("[tmc2209 stepper_x]\n"
                 "uart_pin: PA0\n"
                 "run_current: 800\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
}

TEST(KlipperConfigValidator, TmcMissingUartPin) {
    ConfigParser parser;
    parser.parse("[tmc2209 stepper_x]\n"
                 "run_current: 800\n"
                 "# Missing uart_pin\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results[0].valid);
}

TEST(KlipperConfigValidator, DeltaKinematics) {
    ConfigParser parser;
    parser.parse("[printer]\n"
                 "kinematics: delta\n"
                 "max_velocity: 300\n"
                 "max_accel: 3000\n");
    ConfigValidator validator;
    auto results = validator.validate(parser);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].valid);
    EXPECT_TRUE(results[0].warnings.empty());
}

// ============================================================================
// Skew correction tests
// ============================================================================
TEST(KlipperSkew, DefaultNoCorrection) {
    SkewCorrection skew;
    EXPECT_FALSE(skew.isActive());
    auto corrected = skew.correct(10, 20, 30);
    EXPECT_NEAR(corrected[0], 10, 0.001);
    EXPECT_NEAR(corrected[1], 20, 0.001);
    EXPECT_NEAR(corrected[2], 30, 0.001);
}

TEST(KlipperSkew, WithCorrection) {
    SkewCorrection skew;
    SkewParams params;
    params.xy = 1.0; // 1 degree XY skew
    skew.setParams(params);
    EXPECT_TRUE(skew.isActive());
    auto corrected = skew.correct(10, 20, 30);
    // X should be adjusted by y * tan(1 degree)
    EXPECT_NE(corrected[0], 10.0);
}

TEST(KlipperSkew, M852SetsParams) {
    GcodeCallbacks cb;
    SkewCorrection skew;
    cb.setSkewCorrection = [&](double xy, double xz, double yz) {
        SkewParams params;
        params.xy = xy; params.xz = xz; params.yz = yz;
        skew.setParams(params);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M852 X0.5 Y0.3 Z0.2");
    EXPECT_NEAR(skew.params().xy, 0.5, 0.001);
    EXPECT_NEAR(skew.params().xz, 0.3, 0.001);
    EXPECT_NEAR(skew.params().yz, 0.2, 0.001);
}

// ============================================================================
// Case light tests
// ============================================================================
TEST(KlipperCaseLight, DefaultOff) {
    CaseLight light;
    EXPECT_FALSE(light.isOn());
    EXPECT_NEAR(light.brightness(), 1.0, 0.01);
}

TEST(KlipperCaseLight, TurnOn) {
    CaseLight light;
    light.setState(true, 0.5);
    EXPECT_TRUE(light.isOn());
    EXPECT_NEAR(light.brightness(), 0.5, 0.01);
}

TEST(KlipperCaseLight, BrightnessClamped) {
    CaseLight light;
    light.setState(true, 2.0);
    EXPECT_NEAR(light.brightness(), 1.0, 0.01);
    light.setState(true, -1.0);
    EXPECT_NEAR(light.brightness(), 0.0, 0.01);
}

TEST(KlipperCaseLight, M355SetsState) {
    GcodeCallbacks cb;
    CaseLight light;
    cb.setCaseLight = [&](bool on, double brightness) {
        light.setState(on, brightness);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M355 S1 P50");
    EXPECT_TRUE(light.isOn());
    EXPECT_NEAR(light.brightness(), 0.5, 0.01);
}

// ============================================================================
// Additional missing G-code tests
// ============================================================================
TEST(KlipperMissingGcodes, G92_1ResetOffsets) {
    bool called = false;
    int mode = 0;
    GcodeCallbacks cb;
    cb.resetG92Offsets = [&](int m) { called = true; mode = m; };
    GCodeExecutor exec(cb);
    exec.executeLine("G92.1");
    EXPECT_TRUE(called);
    EXPECT_EQ(mode, 1);
}

TEST(KlipperMissingGcodes, G92_2ResetOffsets) {
    int mode = 0;
    GcodeCallbacks cb;
    cb.resetG92Offsets = [&](int m) { mode = m; };
    GCodeExecutor exec(cb);
    exec.executeLine("G92.2");
    EXPECT_EQ(mode, 2);
}

TEST(KlipperMissingGcodes, G92_3RestoreOffsets) {
    int mode = 0;
    GcodeCallbacks cb;
    cb.resetG92Offsets = [&](int m) { mode = m; };
    GCodeExecutor exec(cb);
    exec.executeLine("G92.3");
    EXPECT_EQ(mode, 3);
}

TEST(KlipperMissingGcodes, M226WaitForPin) {
    bool called = false;
    GcodeCallbacks cb;
    cb.waitForPinState = [&](int, int) { called = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M226 P5 S1");
    EXPECT_TRUE(called);
}

TEST(KlipperMissingGcodes, M240TriggerCamera) {
    bool called = false;
    GcodeCallbacks cb;
    cb.triggerCamera = [&]() { called = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M240");
    EXPECT_TRUE(called);
}

TEST(KlipperMissingGcodes, M250LcdContrast) {
    int contrast = 0;
    GcodeCallbacks cb;
    cb.setLcdContrast = [&](int c) { contrast = c; };
    GCodeExecutor exec(cb);
    exec.executeLine("M250 S128");
    EXPECT_EQ(contrast, 128);
}

TEST(KlipperMissingGcodes, M260SendI2c) {
    bool called = false;
    GcodeCallbacks cb;
    cb.sendI2cData = [&](uint8_t addr, const std::vector<uint8_t>&) {
        called = true;
        EXPECT_EQ(addr, 0x50);
    };
    GCodeExecutor exec(cb);
    exec.executeLine("M260 A80 B42");
    EXPECT_TRUE(called);
}

TEST(KlipperMissingGcodes, M261RequestI2c) {
    GcodeCallbacks cb;
    cb.requestI2cData = [](uint8_t, size_t len) {
        return std::vector<uint8_t>(len, 0xAB);
    };
    std::string output;
    cb.output = [&](const std::string& msg) { output = msg; };
    GCodeExecutor exec(cb);
    exec.executeLine("M261 A80 B4");
    EXPECT_TRUE(output.find("ab") != std::string::npos); // hex output
}

TEST(KlipperMissingGcodes, M428SetHomeOffsetFromPosition) {
    bool called = false;
    GcodeCallbacks cb;
    cb.setHomeOffsetFromPosition = [&]() { called = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M428");
    EXPECT_TRUE(called);
}

TEST(KlipperMissingGcodes, M524AbortSdPrint) {
    bool called = false;
    GcodeCallbacks cb;
    cb.abortSdPrint = [&]() { called = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M524");
    EXPECT_TRUE(called);
}

TEST(KlipperMissingGcodes, M650ClearBedMesh) {
    bool called = false;
    GcodeCallbacks cb;
    cb.clearBedMesh = [&]() { called = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M650");
    EXPECT_TRUE(called);
}

TEST(KlipperMissingGcodes, M853ProbeCalibration) {
    double zOffset = 0;
    GcodeCallbacks cb;
    cb.setProbeCalibration = [&](double z) { zOffset = z; };
    GCodeExecutor exec(cb);
    exec.executeLine("M853 Z0.15");
    EXPECT_NEAR(zOffset, 0.15, 0.001);
}

// ============================================================================
// KlippyInstance integration tests for new features
// ============================================================================
class MissingFeaturesInstance : public ::testing::Test {
protected:
    void SetUp() override {
        socketPath_ = uniqueSocketPath();
        sdDir_ = "/tmp/tether_test_mf_sd_" + std::to_string(getpid());
        std::filesystem::create_directories(sdDir_);

        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = socketPath_;
        cfg.sdcardDir = sdDir_;
        instance_ = std::make_unique<KlippyInstance>(cfg);
    }
    void TearDown() override {
        instance_.reset();
        ::unlink(socketPath_.c_str());
        std::filesystem::remove_all(sdDir_);
    }
    std::string socketPath_, sdDir_;
    std::unique_ptr<KlippyInstance> instance_;
};

TEST_F(MissingFeaturesInstance, M665SetsDeltaGeometry) {
    instance_->executeGcode("M665 L300 R150 A1 B-1 C0.5");
    EXPECT_NEAR(instance_->settings().deltaGeometry.armLength, 300.0, 0.1);
    EXPECT_NEAR(instance_->settings().deltaGeometry.deltaRadius, 150.0, 0.1);
    EXPECT_NEAR(instance_->deltaPrinter().geometry().armLength, 300.0, 0.1);
}

TEST_F(MissingFeaturesInstance, M666SetsDeltaEndstopAdjust) {
    instance_->executeGcode("M666 X0.1 Y-0.2 Z0.3");
    EXPECT_NEAR(instance_->settings().deltaEndstopAdjust.adjX, 0.1, 0.001);
    EXPECT_NEAR(instance_->deltaPrinter().endstopAdjust().adjX, 0.1, 0.001);
}

TEST_F(MissingFeaturesInstance, M907SetsTmcCurrent) {
    instance_->executeGcode("M907 X900 Y800");
    EXPECT_NEAR(instance_->tmcConfig().params("x").runCurrent, 900.0, 0.1);
    EXPECT_NEAR(instance_->tmcConfig().params("y").runCurrent, 800.0, 0.1);
}

TEST_F(MissingFeaturesInstance, M911SetsStealthChop) {
    instance_->executeGcode("M911 S1 X");
    EXPECT_TRUE(instance_->tmcConfig().params("x").stealthChop);
}

TEST_F(MissingFeaturesInstance, M701LoadFilament) {
    instance_->executeGcode("M701 T0");
    EXPECT_TRUE(instance_->filamentLoader().isLoaded(0));
}

TEST_F(MissingFeaturesInstance, M702UnloadFilament) {
    instance_->executeGcode("M701 T0");
    EXPECT_TRUE(instance_->filamentLoader().isLoaded(0));
    instance_->executeGcode("M702 T0");
    EXPECT_FALSE(instance_->filamentLoader().isLoaded(0));
}

TEST_F(MissingFeaturesInstance, M860SetsSecondaryMcu) {
    instance_->executeGcode("M860 S1 P1234");
    auto mcu = instance_->multiMcuManager().getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_FALSE(mcu->serialPath.empty());
}

TEST_F(MissingFeaturesInstance, M862EnablesSecondaryMcu) {
    instance_->executeGcode("M862 S1 P1");
    auto mcu = instance_->multiMcuManager().getMcu(1);
    ASSERT_NE(mcu, nullptr);
    EXPECT_TRUE(mcu->connected);
}

TEST_F(MissingFeaturesInstance, M355SetsCaseLight) {
    instance_->executeGcode("M355 S1 P50");
    EXPECT_TRUE(instance_->caseLight().isOn());
    EXPECT_NEAR(instance_->caseLight().brightness(), 0.5, 0.01);
    EXPECT_TRUE(instance_->settings().caseLightOn);
}

TEST_F(MissingFeaturesInstance, M852SetsSkewCorrection) {
    instance_->executeGcode("M852 X0.5 Y0.3 Z0.2");
    EXPECT_NEAR(instance_->settings().skewParams.xy, 0.5, 0.001);
    EXPECT_NEAR(instance_->skewCorrection().params().xy, 0.5, 0.001);
}

TEST_F(MissingFeaturesInstance, M524AbortsSdPrint) {
    instance_->executeGcode("M524");
    EXPECT_EQ(instance_->printStatsObject()->status({})["state"].asString(), "cancelled");
}

TEST_F(MissingFeaturesInstance, M650ClearsBedMesh) {
    instance_->executeGcode("M420 S1");
    EXPECT_TRUE(instance_->settings().bedMeshEnabled);
    instance_->executeGcode("M650");
    EXPECT_FALSE(instance_->settings().bedMeshEnabled);
}

TEST_F(MissingFeaturesInstance, M428SetsHomeOffsetFromPosition) {
    instance_->executeGcode("G1 X10 Y20 Z30");
    instance_->executeGcode("M428");
    EXPECT_NEAR(instance_->settings().homeOffset["x"], 10.0, 0.1);
    EXPECT_NEAR(instance_->settings().homeOffset["y"], 20.0, 0.1);
}

TEST_F(MissingFeaturesInstance, M42SetsPinState) {
    instance_->executeGcode("M42 P5 S128");
    // No crash — pin state stored internally
    SUCCEED();
}

TEST_F(MissingFeaturesInstance, M280SetsServoAngle) {
    instance_->executeGcode("M280 P0 S90");
    SUCCEED();
}

TEST_F(MissingFeaturesInstance, M150SetsLedColor) {
    instance_->executeGcode("M150 R255 G128 B64 W32");
    SUCCEED();
}

TEST_F(MissingFeaturesInstance, M300Beep) {
    instance_->executeGcode("M300 S440 P200");
    SUCCEED();
}

TEST_F(MissingFeaturesInstance, RegisterTemperatureSensor) {
    // Create a simple thermistor-based temperature sensor
    Thermistor::Params params;
    auto sensor = std::make_shared<Thermistor>(0, params, []() { return 2048.0; });
    instance_->registerTemperatureSensor("temperature_sensor_chamber", sensor);
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "temperature_sensor_chamber") { found = true; break; }
    }
    EXPECT_TRUE(found);
}

TEST_F(MissingFeaturesInstance, RegisterFilamentSwitchSensor) {
    auto sensor = std::make_shared<FilamentSensor>(0, []() { return false; });
    instance_->registerFilamentSwitchSensor("filament_switch_sensor", sensor);
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "filament_switch_sensor") { found = true; break; }
    }
    EXPECT_TRUE(found);
}

TEST_F(MissingFeaturesInstance, RegisterDigitalOut) {
    auto dev = std::make_shared<DigitalOut>(0);
    instance_->registerDigitalOut("output_pin_beep", dev);
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "output_pin_beep") { found = true; break; }
    }
    EXPECT_TRUE(found);
}

TEST_F(MissingFeaturesInstance, RegisterNeopixel) {
    auto dev = std::make_shared<Neopixel>(0, 16, [](std::span<const uint8_t>) {});
    instance_->registerNeopixel("neopixel_leds", dev);
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "neopixel_leds") { found = true; break; }
    }
    EXPECT_TRUE(found);
}

TEST_F(MissingFeaturesInstance, RegisterPulseCounter) {
    auto dev = std::make_shared<PulseCounter>(0);
    instance_->registerPulseCounter("pulse_counter", dev);
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "pulse_counter") { found = true; break; }
    }
    EXPECT_TRUE(found);
}
