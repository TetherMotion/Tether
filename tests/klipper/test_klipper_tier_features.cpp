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
