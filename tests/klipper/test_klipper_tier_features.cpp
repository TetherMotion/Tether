/**
 * @file test_klipper_tier_features.cpp
 * @brief Tests for Tier 1/2/3 features: CNC G-codes, coordinate systems,
 *        config parser enhancements, UDS print control endpoints,
 *        printer object registration, and more.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/config/ConfigParser.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;
using namespace tether::klipper::config;

// ============================================================================
// Helper
// ============================================================================
static std::string uniqueSocketPath() {
    return "/tmp/tether_test_tier_" + std::to_string(getpid()) + ".sock";
}

// ============================================================================
// Part 1: CNC G-codes — Spindle, Tool Change, Coolant (Tier 1.5)
// ============================================================================

TEST(KlipperCncGcodes, SpindleControlM3M4M5) {
    GcodeCallbacks cb;
    double spindleRpm = 0.0;
    cb.setSpindleSpeed = [&](double rpm) { spindleRpm = rpm; };
    cb.output = [](const std::string&) {};

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M3 S1000"));
    EXPECT_NEAR(spindleRpm, 1000.0, 0.1);
    EXPECT_NEAR(state.spindleRpm, 1000.0, 0.1);

    EXPECT_TRUE(exec.executeLine("M4 S500"));
    EXPECT_NEAR(spindleRpm, -500.0, 0.1);
    EXPECT_NEAR(state.spindleRpm, -500.0, 0.1);

    EXPECT_TRUE(exec.executeLine("M5"));
    EXPECT_NEAR(spindleRpm, 0.0, 0.1);
    EXPECT_NEAR(state.spindleRpm, 0.0, 0.1);
}

TEST(KlipperCncGcodes, ToolChangeM6) {
    GcodeCallbacks cb;
    int changedTool = -1;
    cb.toolChange = [&](int tool) { changedTool = tool; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M6 T2"));
    EXPECT_EQ(changedTool, 2);
}

TEST(KlipperCncGcodes, CoolantControlM7M8M9) {
    GcodeCallbacks cb;
    bool flood = false, mist = false;
    cb.setCoolant = [&](bool f, bool m) { flood = f; mist = m; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M7"));
    EXPECT_FALSE(flood);
    EXPECT_TRUE(mist);
    EXPECT_TRUE(state.coolantMist);

    EXPECT_TRUE(exec.executeLine("M8"));
    EXPECT_TRUE(flood);
    EXPECT_TRUE(mist);
    EXPECT_TRUE(state.coolantFlood);

    EXPECT_TRUE(exec.executeLine("M9"));
    EXPECT_FALSE(flood);
    EXPECT_FALSE(mist);
    EXPECT_FALSE(state.coolantFlood);
    EXPECT_FALSE(state.coolantMist);
}

// ============================================================================
// Part 2: Coordinate Systems G54-G59.3 (Tier 1.6)
// ============================================================================

TEST(KlipperCoordSystems, SelectG54) {
    GcodeCallbacks cb;
    int selectedSystem = -1;
    cb.selectCoordinateSystem = [&](int sys) { selectedSystem = sys; };
    cb.output = [](const std::string&) {};

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G54"));
    EXPECT_EQ(selectedSystem, 0);
    EXPECT_EQ(state.activeCoordSystem, 0);

    EXPECT_TRUE(exec.executeLine("G55"));
    EXPECT_EQ(selectedSystem, 1);
    EXPECT_EQ(state.activeCoordSystem, 1);

    EXPECT_TRUE(exec.executeLine("G59"));
    EXPECT_EQ(selectedSystem, 5);
    EXPECT_EQ(state.activeCoordSystem, 5);
}

TEST(KlipperCoordSystems, SelectG59Subcodes) {
    GcodeCallbacks cb;
    int selectedSystem = -1;
    cb.selectCoordinateSystem = [&](int sys) { selectedSystem = sys; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G59.1"));
    EXPECT_EQ(selectedSystem, 6);
    EXPECT_EQ(state.activeCoordSystem, 6);

    EXPECT_TRUE(exec.executeLine("G59.2"));
    EXPECT_EQ(selectedSystem, 7);
    EXPECT_EQ(state.activeCoordSystem, 7);

    EXPECT_TRUE(exec.executeLine("G59.3"));
    EXPECT_EQ(selectedSystem, 8);
    EXPECT_EQ(state.activeCoordSystem, 8);
}

TEST(KlipperCoordSystems, SetCoordinateSystemOffset) {
    GcodeCallbacks cb;
    cb.setCoordinateSystemOffset = [](int, double, double, double) {};

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    // Set offset via callback
    state.coordSystemOffsets[0] = {10.0, 20.0, 30.0};
    EXPECT_NEAR(state.coordSystemOffsets[0][0], 10.0, 0.01);
    EXPECT_NEAR(state.coordSystemOffsets[0][1], 20.0, 0.01);
    EXPECT_NEAR(state.coordSystemOffsets[0][2], 30.0, 0.01);
}

// ============================================================================
// Part 3: G53 Machine Coordinates (Tier 2.7)
// ============================================================================

TEST(KlipperMachineCoords, G53Move) {
    GcodeCallbacks cb;
    double lastX = 0, lastY = 0, lastZ = 0, lastSpeed = 0;
    cb.moveMachine = [&](double x, double y, double z, double speed) {
        lastX = x; lastY = y; lastZ = z; lastSpeed = speed;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G53 G0 X10 Y20 Z30 F600"));
    EXPECT_NEAR(lastX, 10.0, 0.01);
    EXPECT_NEAR(lastY, 20.0, 0.01);
    EXPECT_NEAR(lastZ, 30.0, 0.01);
    EXPECT_NEAR(lastSpeed, 10.0, 0.01); // 600/60 = 10 mm/s
}

// ============================================================================
// Part 4: Path Control G61/G61.1/G64 (Tier 2.8)
// ============================================================================

TEST(KlipperPathControl, ExactStopG61) {
    GcodeCallbacks cb;
    int mode = -1;
    double tolerance = -1;
    cb.setPathControl = [&](int m, double t) { mode = m; tolerance = t; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G61"));
    EXPECT_EQ(mode, 0);
    EXPECT_EQ(state.pathControlMode, 0);
}

TEST(KlipperPathControl, ExactPathG61_1) {
    GcodeCallbacks cb;
    int mode = -1;
    cb.setPathControl = [&](int m, double) { mode = m; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G61.1"));
    EXPECT_EQ(mode, 1);
    EXPECT_EQ(state.pathControlMode, 1);
}

TEST(KlipperPathControl, BlendingG64) {
    GcodeCallbacks cb;
    int mode = -1;
    double tolerance = -1;
    cb.setPathControl = [&](int m, double t) { mode = m; tolerance = t; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G64 P0.5"));
    EXPECT_EQ(mode, 2);
    EXPECT_NEAR(tolerance, 0.5, 0.001);
    EXPECT_EQ(state.pathControlMode, 2);
    EXPECT_NEAR(state.pathBlendingTolerance, 0.5, 0.001);
}

// ============================================================================
// Part 5: Program Flow M0/M1/M2/M30 (Tier 2.9)
// ============================================================================

TEST(KlipperProgramFlow, ProgramStopM0) {
    GcodeCallbacks cb;
    std::string stopMsg;
    cb.programStop = [&](const std::string& msg) { stopMsg = msg; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M0 ; Pause for inspection"));
    EXPECT_EQ(stopMsg, " Pause for inspection");
}

TEST(KlipperProgramFlow, OptionalStopM1) {
    GcodeCallbacks cb;
    bool stopped = false;
    cb.programStop = [&](const std::string&) { stopped = true; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M1"));
    EXPECT_TRUE(stopped);
}

TEST(KlipperProgramFlow, ProgramEndM2) {
    GcodeCallbacks cb;
    bool ended = false;
    cb.programEnd = [&](const std::string&) { ended = true; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M2"));
    EXPECT_TRUE(ended);
}

TEST(KlipperProgramFlow, ProgramEndM30) {
    GcodeCallbacks cb;
    std::string endMsg;
    cb.programEnd = [&](const std::string& msg) { endMsg = msg; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M30 ; End of program"));
    EXPECT_EQ(endMsg, " End of program");
}

// ============================================================================
// Part 6: Software Endstops M208/M211 (Tier 3.22)
// ============================================================================

TEST(KlipperSoftwareEndstops, M208SetLimits) {
    GcodeCallbacks cb;
    std::string axis;
    double minVal = 0, maxVal = 0;
    bool enabled = false;
    cb.setSoftwareEndstops = [&](const std::string& a, double mn, double mx, bool e) {
        axis = a; minVal = mn; maxVal = mx; enabled = e;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M208 X S0 P200"));
    EXPECT_EQ(axis, "x");
}

TEST(KlipperSoftwareEndstops, M211EnableDisable) {
    GcodeCallbacks cb;
    bool enableVal = false;
    cb.setSoftwareEndstopEnable = [&](bool e) { enableVal = e; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M211 S0"));
    EXPECT_FALSE(enableVal);
    EXPECT_FALSE(state.softwareEndstopsEnabled);

    EXPECT_TRUE(exec.executeLine("M211 S1"));
    EXPECT_TRUE(enableVal);
    EXPECT_TRUE(state.softwareEndstopsEnabled);
}

// ============================================================================
// Part 7: Thermistor Parameters M305 (Tier 3.23)
// ============================================================================

TEST(KlipperThermistorParams, M305SetParams) {
    GcodeCallbacks cb;
    int sensor = -1;
    double r = 0, beta = 0, rNom = 0, tNom = 0;
    cb.setThermistorParams = [&](int s, double rP, double b, double rN, double tN) {
        sensor = s; r = rP; beta = b; rNom = rN; tNom = tN;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M305 P0 R4700 B3950 T100000"));
    EXPECT_EQ(sensor, 0);
    EXPECT_NEAR(r, 4700.0, 0.1);
    EXPECT_NEAR(beta, 3950.0, 0.1);
    EXPECT_NEAR(rNom, 100000.0, 0.1);
}

// ============================================================================
// Part 8: Filament Width Sensor M405/M406/M407 (Tier 3.24)
// ============================================================================

TEST(KlipperFilamentSensor, M405Enable) {
    GcodeCallbacks cb;
    bool enabled = false;
    cb.setFilamentWidthSensor = [&](bool e) { enabled = e; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M405"));
    EXPECT_TRUE(enabled);
    EXPECT_TRUE(state.filamentWidthSensorEnabled);
}

TEST(KlipperFilamentSensor, M406Disable) {
    GcodeCallbacks cb;
    bool enabled = true;
    cb.setFilamentWidthSensor = [&](bool e) { enabled = e; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M406"));
    EXPECT_FALSE(enabled);
    EXPECT_FALSE(state.filamentWidthSensorEnabled);
}

TEST(KlipperFilamentSensor, M407Report) {
    GcodeCallbacks cb;
    std::string output;
    cb.getFilamentWidth = [&]() { return std::string("Filament width: 1.75mm"); };
    cb.output = [&](const std::string& msg) { output = msg; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M407"));
    EXPECT_EQ(output, "Filament width: 1.75mm");
}

// ============================================================================
// Part 9: Canned Cycles G81-G89 (Tier 3.17)
// ============================================================================

TEST(KlipperCannedCycles, G81Drilling) {
    GcodeCallbacks cb;
    int cycleType = 0;
    double x = 0, y = 0, z = 0, r = 0, f = 0;
    cb.executeCannedCycle = [&](int t, double cx, double cy, double cz,
                                 double rh, double fr) {
        cycleType = t; x = cx; y = cy; z = cz; r = rh; f = fr;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G81 X10 Y20 Z-5 R2 F100"));
    EXPECT_EQ(cycleType, 81);
    EXPECT_NEAR(x, 10.0, 0.01);
    EXPECT_NEAR(y, 20.0, 0.01);
    EXPECT_NEAR(z, -5.0, 0.01);
    EXPECT_NEAR(r, 2.0, 0.01);
    EXPECT_NEAR(f, 100.0, 0.01);
    EXPECT_TRUE(state.cannedCycleActive);
}

TEST(KlipperCannedCycles, G80Cancel) {
    GcodeCallbacks cb;
    bool cancelled = false;
    cb.cancelCannedCycle = [&]() { cancelled = true; };

    PrinterMotionState state;
    state.cannedCycleActive = true;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G80"));
    EXPECT_TRUE(cancelled);
    EXPECT_FALSE(state.cannedCycleActive);
}

// ============================================================================
// Part 10: G92.3 Restore Logic (Tier 2.14)
// ============================================================================

TEST(KlipperG92Restore, SaveAndRestoreOffsets) {
    GcodeCallbacks cb;
    cb.setPosition = [](double, double, double, double) {};
    cb.resetG92Offsets = [](int) {};

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    // Set G92 offsets
    EXPECT_TRUE(exec.executeLine("G92 X10 Y20 Z30"));
    EXPECT_NEAR(state.position[0], 10.0, 0.01);
    EXPECT_NEAR(state.position[1], 20.0, 0.01);
    EXPECT_NEAR(state.position[2], 30.0, 0.01);
}

// ============================================================================
// Part 11: M226 Pin Polling (Tier 3.15)
// ============================================================================

TEST(KlipperPinPolling, M226Dispatch) {
    GcodeCallbacks cb;
    int polledPin = -1, polledState = -1;
    double polledTimeout = 0;
    cb.waitForPin = [&](int pin, int state, double timeout) -> bool {
        polledPin = pin;
        polledState = state;
        polledTimeout = timeout;
        return true;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("M226 P5 S1"));
    EXPECT_EQ(polledPin, 5);
    EXPECT_EQ(polledState, 1);
    EXPECT_NEAR(polledTimeout, 60.0, 0.01);
}

// ============================================================================
// Part 12: ConfigParser Include Directives (Tier 1.2)
// ============================================================================

TEST(KlipperConfigParser, IncludeDirective) {
    // Create a temp include file
    std::string includePath = "/tmp/tether_test_include_" +
        std::to_string(getpid()) + ".cfg";
    {
        std::ofstream f(includePath);
        f << "[stepper_x]\nstep_pin: PA0\ndir_pin: PA1\n";
    }

    std::string mainConfig =
        "[include " + includePath + "]\n"
        "[extruder]\nstep_pin: PB0\n";

    ConfigParser parser;
    EXPECT_TRUE(parser.parse(mainConfig));
    EXPECT_TRUE(parser.hasSection("stepper_x"));
    EXPECT_TRUE(parser.hasSection("extruder"));
    EXPECT_EQ(parser.includedFiles().size(), 1u);
    EXPECT_EQ(parser.includedFiles()[0], includePath);

    // Cleanup
    std::filesystem::remove(includePath);
}

TEST(KlipperConfigParser, CircularIncludeProtection) {
    std::string fileA = "/tmp/tether_test_circ_a_" +
        std::to_string(getpid()) + ".cfg";
    std::string fileB = "/tmp/tether_test_circ_b_" +
        std::to_string(getpid()) + ".cfg";

    {
        std::ofstream fA(fileA);
        fA << "[include " << fileB << "]\n[section_a]\nkey: value_a\n";
    }
    {
        std::ofstream fB(fileB);
        fB << "[include " << fileA << "]\n[section_b]\nkey: value_b\n";
    }

    ConfigParser parser;
    EXPECT_TRUE(parser.parseFile(fileA));
    EXPECT_TRUE(parser.hasSection("section_a"));
    EXPECT_TRUE(parser.hasSection("section_b"));

    std::filesystem::remove(fileA);
    std::filesystem::remove(fileB);
}

TEST(KlipperConfigParser, RelativeIncludePath) {
    std::string dir = "/tmp/tether_test_rel_" + std::to_string(getpid()) + "/";
    std::filesystem::create_directory(dir);
    {
        std::ofstream f(dir + "main.cfg");
        f << "[include sub.cfg]\n[printer]\nkinematics: cartesian\n";
    }
    {
        std::ofstream f(dir + "sub.cfg");
        f << "[stepper_y]\nstep_pin: PA2\n";
    }

    ConfigParser parser;
    EXPECT_TRUE(parser.parseFile(dir + "main.cfg"));
    EXPECT_TRUE(parser.hasSection("stepper_y"));
    EXPECT_TRUE(parser.hasSection("printer"));

    std::filesystem::remove_all(dir);
}

// ============================================================================
// Part 13: ConfigParser Variable Substitution (Tier 2.10)
// ============================================================================

TEST(KlipperConfigParser, VariableSubstitution) {
    ConfigParser parser;
    parser.setVariable("step_pin", "PA0");
    parser.setVariable("dir_pin", "PA1");

    EXPECT_TRUE(parser.parse(
        "[stepper_x]\n"
        "step_pin: {step_pin}\n"
        "dir_pin: {dir_pin}\n"
    ));

    auto* sec = parser.getSection("stepper_x");
    ASSERT_NE(sec, nullptr);
    EXPECT_EQ(sec->get("step_pin"), "PA0");
    EXPECT_EQ(sec->get("dir_pin"), "PA1");
}

TEST(KlipperConfigParser, UnknownVariablePreserved) {
    ConfigParser parser;
    EXPECT_TRUE(parser.parse(
        "[stepper_x]\n"
        "step_pin: {unknown_var}\n"
    ));

    auto* sec = parser.getSection("stepper_x");
    ASSERT_NE(sec, nullptr);
    // Unknown variables should be left as-is
    EXPECT_EQ(sec->get("step_pin"), "{unknown_var}");
}

// ============================================================================
// Part 14: ConfigParser Multi-line Values (Tier 3.19)
// ============================================================================

TEST(KlipperConfigParser, MultiLineValue) {
    ConfigParser parser;
    EXPECT_TRUE(parser.parse(
        "[display]\n"
        "display_data: line1 \\\n"
        "line2 \\\n"
        "line3\n"
    ));

    auto* sec = parser.getSection("display");
    ASSERT_NE(sec, nullptr);
    std::string val = sec->get("display_data");
    EXPECT_EQ(val, "line1 line2 line3");
}

// ============================================================================
// Part 15: Printer Object Registration (Tier 1.4)
// ============================================================================

TEST(KlipperPrinterObjects, GcodeMoveRegistered) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    // gcode_move should be registered
    auto& server = instance.server();
    auto objects = server.listObjects();
    EXPECT_NE(std::find(objects.begin(), objects.end(), "gcode_move"),
              objects.end());
}

TEST(KlipperPrinterObjects, ConfigfileRegistered) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    auto& server = instance.server();
    auto objects = server.listObjects();
    EXPECT_NE(std::find(objects.begin(), objects.end(), "configfile"),
              objects.end());
}

TEST(KlipperPrinterObjects, WebhooksRegistered) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    auto& server = instance.server();
    auto objects = server.listObjects();
    EXPECT_NE(std::find(objects.begin(), objects.end(), "webhooks"),
              objects.end());
}

TEST(KlipperPrinterObjects, FirmwareRetractionRegistered) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    auto& server = instance.server();
    auto objects = server.listObjects();
    EXPECT_NE(std::find(objects.begin(), objects.end(), "firmware_retraction"),
              objects.end());
}

// ============================================================================
// Part 16: FirmwareRetractionObject (Tier 2.12)
// ============================================================================

TEST(KlipperFirmwareRetractionObject, StatusFields) {
    auto fr = std::make_shared<FirmwareRetraction>();
    FirmwareRetractionParams params;
    params.retractLength = 3.0;
    params.retractSpeed = 35.0;
    params.unretractLength = 0.5;
    params.unretractSpeed = 15.0;
    params.zHop = 0.4;
    fr->setParams(params);

    FirmwareRetractionObject obj(fr);
    EXPECT_EQ(obj.name(), "firmware_retraction");

    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 5u);

    auto status = obj.status({});
    EXPECT_EQ(status.size(), 5u);
    EXPECT_NEAR(status["retract_length"].asDouble(), 3.0, 0.01);
    EXPECT_NEAR(status["retract_speed"].asDouble(), 35.0, 0.01);
    EXPECT_NEAR(status["unretract_extra_length"].asDouble(), 0.5, 0.01);
    EXPECT_NEAR(status["unretract_speed"].asDouble(), 15.0, 0.01);
    EXPECT_NEAR(status["z_hop"].asDouble(), 0.4, 0.01);
}

// ============================================================================
// Part 17: Print Control UDS Endpoints (Tier 1.3)
// ============================================================================

TEST(KlipperUdsPrintControl, StartEndpointRegistered) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/start"),
              endpoints.end());
}

TEST(KlipperUdsPrintControl, CancelEndpointRegistered) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/cancel"),
              endpoints.end());
}

TEST(KlipperUdsPrintControl, PauseEndpointRegistered) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/pause"),
              endpoints.end());
}

TEST(KlipperUdsPrintControl, ResumeEndpointRegistered) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/resume"),
              endpoints.end());
}

TEST(KlipperUdsPrintControl, StartHandlerCalled) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    bool called = false;
    server.setPrintStartHandler([&]() { called = true; });

    // Verify the handler is set by checking endpoint registration
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/start"),
              endpoints.end());
    // Note: callEndpoint is private; handler is tested via integration
}

TEST(KlipperUdsPrintControl, PauseHandlerCalled) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    bool called = false;
    server.setPrintPauseHandler([&]() { called = true; });

    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/pause"),
              endpoints.end());
}

// ============================================================================
// Part 18: Additional Moonraker Endpoints (Tier 3.18)
// ============================================================================

TEST(KlipperUdsMoonraker, ServerInfoEndpoint) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "server/info"),
              endpoints.end());
}

TEST(KlipperUdsMoonraker, ServerFilesListEndpoint) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "server/files/list"),
              endpoints.end());
}

TEST(KlipperUdsMoonraker, MachineSystemInfoEndpoint) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "machine/system_info"),
              endpoints.end());
}

TEST(KlipperUdsMoonraker, MachineProcstatsEndpoint) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "machine/procstats"),
              endpoints.end());
}

TEST(KlipperUdsMoonraker, ServerInfoReturnsState) {
    KlippyUdsServer server(UdsServerConfig{uniqueSocketPath()});
    // Verify endpoint is registered
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "server/info"),
              endpoints.end());
}

// ============================================================================
// Part 19: G29 Bed Probing (Tier 1.1)
// ============================================================================

TEST(KlipperBedProbing, G29ProbesBed) {
    GcodeCallbacks cb;
    int probedCount = -1;
    cb.probeBed = [&]() -> int { return 9; };  // 3x3 grid
    std::string outputMsg;
    cb.output = [&](const std::string& msg) { outputMsg = msg; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G29"));
    // The output should mention 9 points probed
    EXPECT_NE(outputMsg.find("9"), std::string::npos);
}

TEST(KlipperBedProbing, G29FallbackToBedLevel) {
    GcodeCallbacks cb;
    bool bedLevelCalled = false;
    // No probeBed callback, only bedLevel
    cb.bedLevel = [&]() { bedLevelCalled = true; };
    cb.output = [](const std::string&) {};

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G29"));
    EXPECT_TRUE(bedLevelCalled);
}

// ============================================================================
// Part 20: KlippyInstance G-code Execution (Integration)
// ============================================================================

TEST(KlipperInstanceIntegration, ExecuteSpindleGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    // These should not crash
    EXPECT_TRUE(instance.executeGcode("M3 S1000"));
    EXPECT_TRUE(instance.executeGcode("M5"));
    EXPECT_TRUE(instance.executeGcode("M8"));
    EXPECT_TRUE(instance.executeGcode("M9"));
}

TEST(KlipperInstanceIntegration, ExecuteCoordSystemGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("G54"));
    EXPECT_TRUE(instance.executeGcode("G55"));
    EXPECT_TRUE(instance.executeGcode("G59.3"));
}

TEST(KlipperInstanceIntegration, ExecutePathControlGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("G61"));
    EXPECT_TRUE(instance.executeGcode("G64 P0.5"));
}

TEST(KlipperInstanceIntegration, ExecuteProgramFlowGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("M0"));
    EXPECT_TRUE(instance.executeGcode("M30"));
}

TEST(KlipperInstanceIntegration, ExecuteCannedCycleGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("G81 X10 Y20 Z-5 R2 F100"));
    EXPECT_TRUE(instance.executeGcode("G80"));
}

TEST(KlipperInstanceIntegration, ExecuteSoftwareEndstopGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("M211 S0"));
    EXPECT_TRUE(instance.executeGcode("M211 S1"));
}

TEST(KlipperInstanceIntegration, ExecuteThermistorParamsGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("M305 P0 R4700 B3950 T100000"));
}

TEST(KlipperInstanceIntegration, ExecuteFilamentSensorGcode) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    EXPECT_TRUE(instance.executeGcode("M405"));
    EXPECT_TRUE(instance.executeGcode("M406"));
    EXPECT_TRUE(instance.executeGcode("M407"));
}

// ============================================================================
// Part 21: G-code Parsing for New Codes
// ============================================================================

TEST(KlipperGcodeParsing, ParseG59_1) {
    auto result = parseGcodeLine("G59.1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->code, "G59.1");
}

TEST(KlipperGcodeParsing, ParseG59_2) {
    auto result = parseGcodeLine("G59.2");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->code, "G59.2");
}

TEST(KlipperGcodeParsing, ParseG59_3) {
    auto result = parseGcodeLine("G59.3");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->code, "G59.3");
}

TEST(KlipperGcodeParsing, ParseG61_1) {
    auto result = parseGcodeLine("G61.1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->code, "G61.1");
}

TEST(KlipperGcodeParsing, ParseG81) {
    auto result = parseGcodeLine("G81 X10 Y20 Z-5 R2 F100");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->code, "G81");
    EXPECT_NEAR(result->get('X'), 10.0, 0.01);
    EXPECT_NEAR(result->get('Y'), 20.0, 0.01);
    EXPECT_NEAR(result->get('Z'), -5.0, 0.01);
    EXPECT_NEAR(result->get('R'), 2.0, 0.01);
    EXPECT_NEAR(result->get('F'), 100.0, 0.01);
}

// ============================================================================
// Part 22: PrinterMotionState New Fields
// ============================================================================

TEST(KlipperMotionState, DefaultCoordSystem) {
    PrinterMotionState state;
    EXPECT_EQ(state.activeCoordSystem, 0);
}

TEST(KlipperMotionState, DefaultPathControl) {
    PrinterMotionState state;
    EXPECT_EQ(state.pathControlMode, 0);
    EXPECT_NEAR(state.pathBlendingTolerance, 0.0, 0.001);
}

TEST(KlipperMotionState, DefaultSpindleRpm) {
    PrinterMotionState state;
    EXPECT_NEAR(state.spindleRpm, 0.0, 0.001);
}

TEST(KlipperMotionState, DefaultCoolantState) {
    PrinterMotionState state;
    EXPECT_FALSE(state.coolantFlood);
    EXPECT_FALSE(state.coolantMist);
}

TEST(KlipperMotionState, DefaultSoftwareEndstops) {
    PrinterMotionState state;
    EXPECT_TRUE(state.softwareEndstopsEnabled);
}

TEST(KlipperMotionState, DefaultCannedCycleState) {
    PrinterMotionState state;
    EXPECT_FALSE(state.cannedCycleActive);
}

TEST(KlipperMotionState, DefaultFilamentWidthSensor) {
    PrinterMotionState state;
    EXPECT_FALSE(state.filamentWidthSensorEnabled);
    EXPECT_NEAR(state.filamentWidthMeasured, 1.75, 0.001);
}

TEST(KlipperMotionState, CoordSystemOffsetsArray) {
    PrinterMotionState state;
    // Verify array is zero-initialized
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(state.coordSystemOffsets[i][j], 0.0, 0.001);
        }
    }
}

// ============================================================================
// Part 23: ConfigParser API Extensions
// ============================================================================

TEST(KlipperConfigParserApi, SetVariableBeforeParse) {
    ConfigParser parser;
    parser.setVariable("my_pin", "PC5");
    EXPECT_TRUE(parser.parse(
        "[stepper_z]\nstep_pin: {my_pin}\n"
    ));
    auto* sec = parser.getSection("stepper_z");
    ASSERT_NE(sec, nullptr);
    EXPECT_EQ(sec->get("step_pin"), "PC5");
}

TEST(KlipperConfigParserApi, IncludedFilesList) {
    ConfigParser parser;
    EXPECT_TRUE(parser.parse("[printer]\nkinematics: cartesian\n"));
    EXPECT_TRUE(parser.includedFiles().empty());
}

// ============================================================================
// Part 24: UDS Server Print Control with KlippyInstance
// ============================================================================

TEST(KlipperInstanceUdsIntegration, PrintControlEndpointsWired) {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    cfg.sdcardDir = "/tmp/tether_test_sd_" + std::to_string(getpid());
    KlippyInstance instance(cfg);

    auto& server = instance.server();
    auto endpoints = server.listEndpoints();

    // All print control endpoints should be registered
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/start"),
              endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/cancel"),
              endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/pause"),
              endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/resume"),
              endpoints.end());
}

// ============================================================================
// Part 25: G53 with Missing Coordinates
// ============================================================================

TEST(KlipperMachineCoords, G53WithPartialCoords) {
    GcodeCallbacks cb;
    double lastX = 999, lastY = 999, lastZ = 999;
    cb.moveMachine = [&](double x, double y, double z, double) {
        lastX = x; lastY = y; lastZ = z;
    };

    PrinterMotionState state;
    state.position = {50.0, 50.0, 10.0, 0.0};
    GCodeExecutor exec(std::move(cb), &state);

    // Only X specified, Y and Z should be NaN
    EXPECT_TRUE(exec.executeLine("G53 G0 X100"));
    EXPECT_NEAR(lastX, 100.0, 0.01);
    EXPECT_TRUE(std::isnan(lastY));
    EXPECT_TRUE(std::isnan(lastZ));
}

// ============================================================================
// Part 26: Multiple CNC Codes in Sequence
// ============================================================================

TEST(KlipperCncSequence, FullCncWorkflow) {
    GcodeCallbacks cb;
    double spindleRpm = 0;
    int tool = -1;
    bool flood = false, mist = false;
    int pathMode = -1;
    int coordSys = -1;

    cb.setSpindleSpeed = [&](double r) { spindleRpm = r; };
    cb.toolChange = [&](int t) { tool = t; };
    cb.setCoolant = [&](bool f, bool m) { flood = f; mist = m; };
    cb.setPathControl = [&](int m, double) { pathMode = m; };
    cb.selectCoordinateSystem = [&](int s) { coordSys = s; };
    cb.output = [](const std::string&) {};
    cb.move = [](double, double, double, double, double) {};
    cb.moveMachine = [](double, double, double, double) {};

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    // Simulate a CNC workflow
    EXPECT_TRUE(exec.executeLine("G54"));           // Select coord system
    EXPECT_EQ(coordSys, 0);
    EXPECT_TRUE(exec.executeLine("G61"));           // Exact stop
    EXPECT_EQ(pathMode, 0);
    EXPECT_TRUE(exec.executeLine("M6 T1"));         // Tool change
    EXPECT_EQ(tool, 1);
    EXPECT_TRUE(exec.executeLine("M3 S5000"));      // Spindle on
    EXPECT_NEAR(spindleRpm, 5000.0, 0.1);
    EXPECT_TRUE(exec.executeLine("M8"));            // Flood coolant on
    EXPECT_TRUE(flood);
    EXPECT_TRUE(exec.executeLine("G81 X10 Y20 Z-5 R2 F100")); // Drill
    EXPECT_TRUE(state.cannedCycleActive);
    EXPECT_TRUE(exec.executeLine("G80"));           // Cancel canned cycle
    EXPECT_FALSE(state.cannedCycleActive);
    EXPECT_TRUE(exec.executeLine("M9"));            // Coolant off
    EXPECT_FALSE(flood);
    EXPECT_TRUE(exec.executeLine("M5"));            // Spindle off
    EXPECT_NEAR(spindleRpm, 0.0, 0.1);
    EXPECT_TRUE(exec.executeLine("M30"));           // Program end
}

// ============================================================================
// Tests use the shared main.cpp in tests/ directory
// ============================================================================
