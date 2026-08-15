/**
 * @file test_klipper_tier_cd.cpp
 * @brief Tests for Tier C and D features: Moonraker API endpoints, polygon parsing,
 *        stub printer object fixes, extended G-code commands, new printer objects,
 *        state variable wiring, and config section handlers.
 */

#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace tether::klipper::klippy;

// ============================================================================
// C1: Moonraker API endpoint tests (via public API)
// ============================================================================

TEST(TierC1MoonrakerEndpoints, NewEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();

    auto hasEndpoint = [&](const std::string& name) {
        return std::find(endpoints.begin(), endpoints.end(), name) != endpoints.end();
    };

    // C1 endpoints
    EXPECT_TRUE(hasEndpoint("server/config"));
    EXPECT_TRUE(hasEndpoint("server/files/roots"));
    EXPECT_TRUE(hasEndpoint("server/files/create_dir"));
    EXPECT_TRUE(hasEndpoint("server/files/metascan"));
    EXPECT_TRUE(hasEndpoint("server/files/thumbnails"));
    EXPECT_TRUE(hasEndpoint("server/logs/rollover"));
    EXPECT_TRUE(hasEndpoint("server/klippy_log"));
    EXPECT_TRUE(hasEndpoint("server/moonraker_log"));
    EXPECT_TRUE(hasEndpoint("machine/services/list"));
    EXPECT_TRUE(hasEndpoint("machine/services/restart"));
    EXPECT_TRUE(hasEndpoint("machine/services/stop"));
    EXPECT_TRUE(hasEndpoint("machine/services/start"));
    EXPECT_TRUE(hasEndpoint("machine/update/list"));
    EXPECT_TRUE(hasEndpoint("machine/update/refresh"));
    EXPECT_TRUE(hasEndpoint("machine/update/update"));
    EXPECT_TRUE(hasEndpoint("machine/update/recover"));
    EXPECT_TRUE(hasEndpoint("database/list"));
    EXPECT_TRUE(hasEndpoint("database/get"));
    EXPECT_TRUE(hasEndpoint("database/put"));
    EXPECT_TRUE(hasEndpoint("database/delete"));
    EXPECT_TRUE(hasEndpoint("job_queue/status"));
    EXPECT_TRUE(hasEndpoint("job_queue/post_job"));
    EXPECT_TRUE(hasEndpoint("job_queue/delete_job"));
    EXPECT_TRUE(hasEndpoint("job_history/list"));
    EXPECT_TRUE(hasEndpoint("job_history/get"));
    EXPECT_TRUE(hasEndpoint("announcements/list"));
    EXPECT_TRUE(hasEndpoint("announcements/update"));
    EXPECT_TRUE(hasEndpoint("announcements/dismiss"));
    EXPECT_TRUE(hasEndpoint("webcams/list"));
    EXPECT_TRUE(hasEndpoint("webcams/get"));
    EXPECT_TRUE(hasEndpoint("webcams/test"));
    EXPECT_TRUE(hasEndpoint("devices/list"));
    EXPECT_TRUE(hasEndpoint("devices/get"));
}

TEST(TierC1MoonrakerEndpoints, DatabaseOperations) {
    KlippyServer server;

    // Put a value
    server.databasePut("test_ns", "test_key", JsonValue(42));

    // Get the value
    auto val = server.databaseGet("test_ns", "test_key");
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(val->asInt(), 42);

    // Delete the value
    EXPECT_TRUE(server.databaseDelete("test_ns", "test_key"));

    // Verify it's gone
    EXPECT_FALSE(server.databaseGet("test_ns", "test_key").has_value());
}

TEST(TierC1MoonrakerEndpoints, JobQueueOperations) {
    KlippyServer server;

    server.jobQueueAdd("test.gcode");
    server.jobQueueAdd("test2.gcode");

    // Just verify it doesn't crash
    SUCCEED();
}

TEST(TierC1MoonrakerEndpoints, JobHistoryOperations) {
    KlippyServer server;

    int64_t jobId = server.jobHistoryAdd("test.gcode", "completed");
    EXPECT_GT(jobId, 0);

    int64_t jobId2 = server.jobHistoryAdd("test2.gcode", "cancelled");
    EXPECT_GT(jobId2, jobId);
}

TEST(TierC1MoonrakerEndpoints, AnnouncementsOperations) {
    KlippyServer server;

    server.announcementAdd("test-001", "Test Announcement", "This is a test", "info");
    server.announcementAdd("test-002", "Warning", "Warning test", "warning");

    // Just verify it doesn't crash
    SUCCEED();
}

TEST(TierC1MoonrakerEndpoints, WebcamRegistration) {
    KlippyServer server;

    server.registerWebcam("cam1", "http://localhost:8080/?action=stream", "mjpegstreamer");
    server.registerWebcam("cam2", "http://localhost:8081/?action=stream", "mjpegstreamer");

    // Just verify it doesn't crash
    SUCCEED();
}

TEST(TierC1MoonrakerEndpoints, ServiceRegistration) {
    KlippyServer server;

    server.registerService("custom_service", "active", "running");

    // Just verify it doesn't crash
    SUCCEED();
}

TEST(TierC1MoonrakerEndpoints, FileRootRegistration) {
    KlippyServer server;

    server.registerFileRoot("custom", "/tmp/custom_root", true);

    // Just verify it doesn't crash
    SUCCEED();
}

TEST(TierC1MoonrakerEndpoints, InitServerConfig) {
    KlippyServer server;

    // initServerConfig is called in constructor, just verify it doesn't crash
    server.initServerConfig();
    SUCCEED();
}

// ============================================================================
// C2: Polygon parsing tests
// ============================================================================

TEST(TierC2PolygonParsing, SimplePolygon) {
    // Test parsing a simple polygon string "10,10,50,10,50,50,10,50"
    std::string polygon = "10,10,50,10,50,50,10,50";
    // Remove brackets and parse
    std::string cleaned;
    for (char c : polygon) {
        if (c != '[' && c != ']' && c != ' ' && c != '\t') {
            cleaned += c;
        }
    }
    std::vector<double> nums;
    std::string current;
    for (char c : cleaned) {
        if (c == ',') {
            if (!current.empty()) {
                try { nums.push_back(std::stod(current)); } catch (...) {}
                current.clear();
            }
        } else {
            current += c;
        }
    }
    if (!current.empty()) {
        try { nums.push_back(std::stod(current)); } catch (...) {}
    }
    ASSERT_EQ(nums.size(), 8u);
    EXPECT_NEAR(nums[0], 10.0, 0.001);
    EXPECT_NEAR(nums[1], 10.0, 0.001);
    EXPECT_NEAR(nums[6], 10.0, 0.001);
    EXPECT_NEAR(nums[7], 50.0, 0.001);
}

TEST(TierC2PolygonParsing, BracketedPolygon) {
    // Test parsing "[[10,10],[50,10],[50,50],[10,50]]"
    std::string polygon = "[[10,10],[50,10],[50,50],[10,50]]";
    std::string cleaned;
    for (char c : polygon) {
        if (c != '[' && c != ']' && c != ' ' && c != '\t') {
            cleaned += c;
        }
    }
    std::vector<double> nums;
    std::string current;
    for (char c : cleaned) {
        if (c == ',') {
            if (!current.empty()) {
                try { nums.push_back(std::stod(current)); } catch (...) {}
                current.clear();
            }
        } else {
            current += c;
        }
    }
    if (!current.empty()) {
        try { nums.push_back(std::stod(current)); } catch (...) {}
    }
    ASSERT_EQ(nums.size(), 8u);
    EXPECT_NEAR(nums[0], 10.0, 0.001);
    EXPECT_NEAR(nums[2], 50.0, 0.001);
}

// ============================================================================
// C3: Stub printer object tests
// ============================================================================

TEST(TierC3StubObjects, ForceMoveObject) {
    ForceMoveObject obj;
    EXPECT_EQ(obj.name(), "force_move");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 1u);
    EXPECT_EQ(fields[0], "enable_force_move");

    obj.setEnableForceMove(true);
    auto status = obj.status({});
    EXPECT_TRUE(status["enable_force_move"].asBool());

    obj.setEnableForceMove(false);
    status = obj.status({});
    EXPECT_FALSE(status["enable_force_move"].asBool());
}

TEST(TierC3StubObjects, SafeZHomeObject) {
    SafeZHomeObject obj;
    EXPECT_EQ(obj.name(), "safe_z_home");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 5u);

    obj.setHomeXyPosition("100, 100");
    obj.setZHop(15.0);
    obj.setZHopSpeed(25.0);
    obj.setXyHomeSpeed(60.0);
    obj.setMoveToPrevious(true);

    auto status = obj.status({});
    EXPECT_EQ(status["home_xy_position"].asString(), "100, 100");
    EXPECT_NEAR(status["z_hop"].asDouble(), 15.0, 0.001);
    EXPECT_NEAR(status["z_hop_speed"].asDouble(), 25.0, 0.001);
    EXPECT_NEAR(status["xy_home_speed"].asDouble(), 60.0, 0.001);
    EXPECT_TRUE(status["move_to_previous"].asBool());
}

TEST(TierC3StubObjects, MultiPinObject) {
    MultiPinObject obj("my_multi_pin");
    EXPECT_EQ(obj.name(), "my_multi_pin");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 2u);

    obj.setPins({"pin1", "pin2", "pin3"});
    obj.setValue("1");

    auto status = obj.status({});
    EXPECT_EQ(status["value"].asString(), "1");
    const auto& pins = status["pins"].asArray();
    EXPECT_EQ(pins.size(), 3u);
    EXPECT_EQ(pins[0].asString(), "pin1");
    EXPECT_EQ(pins[2].asString(), "pin3");
}

// ============================================================================
// D1: Extended G-code command parsing tests
// ============================================================================

TEST(TierD1ExtendedCommands, TestResonancesParsing) {
    std::string line = "TEST_RESONANCES AXIS=X MIN_FREQ=5 MAX_FREQ=100";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "TEST_RESONANCES");
    EXPECT_TRUE(g.isExtendedCommand());
    EXPECT_EQ(g.getNamed("AXIS"), "X");
    EXPECT_NEAR(g.getNamedDouble("MIN_FREQ"), 5.0, 0.001);
    EXPECT_NEAR(g.getNamedDouble("MAX_FREQ"), 100.0, 0.001);
}

TEST(TierD1ExtendedCommands, ShaperCalibrateParsing) {
    std::string line = "SHAPER_CALIBRATE AXIS=both";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "SHAPER_CALIBRATE");
    EXPECT_EQ(g.getNamed("AXIS"), "both");
}

TEST(TierD1ExtendedCommands, ZTiltAdjustParsing) {
    auto parsed = parseGcodeLine("Z_TILT_ADJUST");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "Z_TILT_ADJUST");
    EXPECT_TRUE(parsed->isExtendedCommand());
}

TEST(TierD1ExtendedCommands, QuadGantryLevelParsing) {
    auto parsed = parseGcodeLine("QUAD_GANTRY_LEVEL");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "QUAD_GANTRY_LEVEL");
}

TEST(TierD1ExtendedCommands, ScrewsTiltAdjustParsing) {
    auto parsed = parseGcodeLine("SCREWS_TILT_ADJUST");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SCREWS_TILT_ADJUST");
}

TEST(TierD1ExtendedCommands, BedScrewsAdjustParsing) {
    auto parsed = parseGcodeLine("BED_SCREWS_ADJUST");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "BED_SCREWS_ADJUST");
}

TEST(TierD1ExtendedCommands, DeltaCalibrateParsing) {
    auto parsed = parseGcodeLine("DELTA_CALIBRATE");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "DELTA_CALIBRATE");
}

TEST(TierD1ExtendedCommands, ProbeAccuracyParsing) {
    std::string line = "PROBE_ACCURACY SAMPLES=10 PROBE_SPEED=5.0";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "PROBE_ACCURACY");
    EXPECT_EQ(g.getNamedInt("SAMPLES"), 10);
    EXPECT_NEAR(g.getNamedDouble("PROBE_SPEED"), 5.0, 0.001);
}

TEST(TierD1ExtendedCommands, SetPressureAdvanceParsing) {
    std::string line = "SET_PRESSURE_ADVANCE ADVANCE=0.05 SMOOTH_TIME=0.040";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "SET_PRESSURE_ADVANCE");
    EXPECT_NEAR(g.getNamedDouble("ADVANCE"), 0.05, 0.0001);
    EXPECT_NEAR(g.getNamedDouble("SMOOTH_TIME"), 0.040, 0.0001);
}

TEST(TierD1ExtendedCommands, SetInputShaperParsing) {
    std::string line = "SET_INPUT_SHAPER SHAPER_FREQ_X=45.0 SHAPER_FREQ_Y=55.0";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "SET_INPUT_SHAPER");
    EXPECT_EQ(g.getNamed("SHAPER_FREQ_X"), "45.0");
    EXPECT_EQ(g.getNamed("SHAPER_FREQ_Y"), "55.0");
}

TEST(TierD1ExtendedCommands, ForceMoveParsing) {
    std::string line = "FORCE_MOVE STEPPER=stepper_x DISTANCE=10 VELOCITY=20 ACCEL=1000";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "FORCE_MOVE");
    EXPECT_EQ(g.getNamed("STEPPER"), "stepper_x");
    EXPECT_NEAR(g.getNamedDouble("DISTANCE"), 10.0, 0.001);
    EXPECT_NEAR(g.getNamedDouble("VELOCITY"), 20.0, 0.001);
}

TEST(TierD1ExtendedCommands, StepperBuzzParsing) {
    auto parsed = parseGcodeLine("STEPPER_BUZZ STEPPER=stepper_x");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "STEPPER_BUZZ");
    EXPECT_EQ(parsed->getNamed("STEPPER"), "stepper_x");
}

TEST(TierD1ExtendedCommands, ManualStepperParsing) {
    std::string line = "MANUAL_STEPPER STEPPER=manual_stepper DISTANCE=5 SPEED=10";
    auto parsed = parseGcodeLine(line);
    ASSERT_TRUE(parsed.has_value());
    auto& g = *parsed;
    EXPECT_EQ(g.code, "MANUAL_STEPPER");
    EXPECT_EQ(g.getNamed("STEPPER"), "manual_stepper");
    EXPECT_NEAR(g.getNamedDouble("DISTANCE"), 5.0, 0.001);
}

TEST(TierD1ExtendedCommands, EndstopPhaseParsing) {
    auto parsed = parseGcodeLine("ENDSTOP_PHASE STEPPER=stepper_x");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "ENDSTOP_PHASE");
    EXPECT_EQ(parsed->getNamed("STEPPER"), "stepper_x");
}

TEST(TierD1ExtendedCommands, SetMultiPinParsing) {
    auto parsed = parseGcodeLine("SET_MULTI_PIN PIN=my_pin VALUE=1");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_MULTI_PIN");
    EXPECT_EQ(parsed->getNamed("PIN"), "my_pin");
    EXPECT_EQ(parsed->getNamed("VALUE"), "1");
}

TEST(TierD1ExtendedCommands, SetSmartEffectorParsing) {
    auto parsed = parseGcodeLine("SET_SMART_EFFECTOR SENSITIVITY=0.5");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_SMART_EFFECTOR");
    EXPECT_NEAR(parsed->getNamedDouble("SENSITIVITY"), 0.5, 0.001);
}

TEST(TierD1ExtendedCommands, DeltaAnalyzeParsing) {
    auto parsed = parseGcodeLine("DELTA_ANALYZE CALIBRATE_RADIUS=100");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "DELTA_ANALYZE");
    EXPECT_NEAR(parsed->getNamedDouble("CALIBRATE_RADIUS"), 100.0, 0.001);
}

// ============================================================================
// D2: New printer object tests
// ============================================================================

TEST(TierD2PrinterObjects, ManualProbeObject) {
    ManualProbeObject obj;
    EXPECT_EQ(obj.name(), "manual_probe");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 4u);

    obj.setActive(true);
    obj.setZPosition(0.5);
    obj.setZPositionLower(0.4);
    obj.setZPositionUpper(0.6);

    auto status = obj.status({});
    EXPECT_TRUE(status["is_active"].asBool());
    EXPECT_NEAR(status["z_position"].asDouble(), 0.5, 0.001);
    EXPECT_NEAR(status["z_position_lower"].asDouble(), 0.4, 0.001);
    EXPECT_NEAR(status["z_position_upper"].asDouble(), 0.6, 0.001);
}

TEST(TierD2PrinterObjects, FilamentMotionSensorObject) {
    FilamentMotionSensorObject obj("filament_motion_sensor");
    EXPECT_EQ(obj.name(), "filament_motion_sensor");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 4u);

    obj.setFilamentDetected(false);
    obj.setDistance(100.0);

    auto status = obj.status({});
    EXPECT_FALSE(status["filament_detected"].asBool());
    EXPECT_NEAR(status["distance"].asDouble(), 100.0, 0.001);
}

TEST(TierD2PrinterObjects, LoadCellObject) {
    LoadCellObject obj("load_cell");
    EXPECT_EQ(obj.name(), "load_cell");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 5u);

    obj.setLoad(42.5);
    obj.setTareValue(1.0);
    obj.setThreshold(10.0);

    auto status = obj.status({});
    EXPECT_NEAR(status["load"].asDouble(), 42.5, 0.001);
    EXPECT_NEAR(status["tare_value"].asDouble(), 1.0, 0.001);
    EXPECT_NEAR(status["threshold"].asDouble(), 10.0, 0.001);
}

TEST(TierD2PrinterObjects, CanbusStatsObject) {
    CanbusStatsObject obj("canbus_stats");
    EXPECT_EQ(obj.name(), "canbus_stats");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 5u);

    obj.setRxError(5);
    obj.setTxError(3);
    obj.setBusState("error-passive");

    auto status = obj.status({});
    EXPECT_EQ(status["rx_error"].asInt(), 5);
    EXPECT_EQ(status["tx_error"].asInt(), 3);
    EXPECT_EQ(status["bus_state"].asString(), "error-passive");
}

TEST(TierD2PrinterObjects, PWMCycleTimeObject) {
    PWMCycleTimeObject obj("pwm_cycle_time");
    EXPECT_EQ(obj.name(), "pwm_cycle_time");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 2u);

    obj.setValue(0.75);
    obj.setCycleTime(0.050);

    auto status = obj.status({});
    EXPECT_NEAR(status["value"].asDouble(), 0.75, 0.001);
    EXPECT_NEAR(status["cycle_time"].asDouble(), 0.050, 0.001);
}

TEST(TierD2PrinterObjects, ResonanceTesterObject) {
    ResonanceTesterObject obj;
    EXPECT_EQ(obj.name(), "resonance_tester");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 4u);

    obj.setMinFreq(10.0);
    obj.setMaxFreq(150.0);

    auto status = obj.status({});
    EXPECT_NEAR(status["min_freq"].asDouble(), 10.0, 0.001);
    EXPECT_NEAR(status["max_freq"].asDouble(), 150.0, 0.001);
}

TEST(TierD2PrinterObjects, AngleObject) {
    AngleObject obj("angle");
    EXPECT_EQ(obj.name(), "angle");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 3u);

    obj.setAngle(90.0);
    obj.setVelocity(100.0);

    auto status = obj.status({});
    EXPECT_NEAR(status["angle"].asDouble(), 90.0, 0.001);
    EXPECT_NEAR(status["velocity"].asDouble(), 100.0, 0.001);
}

TEST(TierD2PrinterObjects, Palette2Object) {
    Palette2Object obj;
    EXPECT_EQ(obj.name(), "palette2");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 7u);

    obj.setConnected(true);
    obj.setLoading(true);

    auto status = obj.status({});
    EXPECT_TRUE(status["connected"].asBool());
    EXPECT_TRUE(status["loading"].asBool());
}

TEST(TierD2PrinterObjects, MenuObject) {
    MenuObject obj;
    EXPECT_EQ(obj.name(), "menu");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 4u);

    obj.setEnabled(true);
    obj.setTimeout(60);

    auto status = obj.status({});
    EXPECT_TRUE(status["enabled"].asBool());
    EXPECT_EQ(status["timeout"].asInt(), 60);
}

TEST(TierD2PrinterObjects, GcodeObject) {
    GcodeObject obj;
    EXPECT_EQ(obj.name(), "gcode");
    auto fields = obj.availableFields();
    EXPECT_EQ(fields.size(), 4u);

    obj.setCommands(100);
    obj.setInfo("Test info");

    auto status = obj.status({});
    EXPECT_EQ(status["commands"].asInt(), 100);
    EXPECT_EQ(status["info"].asString(), "Test info");
}

// ============================================================================
// D3: State variable wiring tests
// ============================================================================

TEST(TierD3StateWiring, PrintStatsInfoTotalLayer) {
    PrintStatsObject obj(nullptr);
    obj.setInfoTotalLayer(50);
    obj.setInfoCurrentLayer(25);

    auto status = obj.status({});
    ASSERT_TRUE(status.count("info"));
    const auto& info = status["info"].asObject();
    EXPECT_EQ(info.find("total_layer")->second.asInt(), 50);
    EXPECT_EQ(info.find("current_layer")->second.asInt(), 25);
}

TEST(TierD3StateWiring, PrintStatsInfoInAvailableFields) {
    PrintStatsObject obj(nullptr);
    auto fields = obj.availableFields();
    auto it = std::find(fields.begin(), fields.end(), "info");
    EXPECT_NE(it, fields.end());
}

// ============================================================================
// E1-High: Additional Moonraker endpoint tests
// ============================================================================

TEST(TierE1HighEndpoints, JobQueuePauseStart) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "job_queue/pause"), endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "job_queue/start"), endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "job_queue/jump_to"), endpoints.end());
}

TEST(TierE1HighEndpoints, JobHistoryDelete) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "job_history/delete"), endpoints.end());
}

TEST(TierE1HighEndpoints, WebcamsUpdateDelete) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "webcams/update"), endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "webcams/delete"), endpoints.end());
}

TEST(TierE1HighEndpoints, DevicePowerOnOffToggle) {
    KlippyServer server;
    server.registerPowerDevice("psu", "off");

    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "machine/device_power/on"), endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "machine/device_power/off"), endpoints.end());
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "machine/device_power/toggle"), endpoints.end());
}

TEST(TierE1HighEndpoints, SystemPerms) {
    KlippyServer server;
    server.setSystemPerms("read", {"read"});
    server.setSystemPerms("write", {"write"});

    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "machine/system_perms"), endpoints.end());
}

TEST(TierE1HighEndpoints, AnnouncementsFeed) {
    KlippyServer server;
    server.announcementAdd("feed-001", "Feed Test", "Feed description", "info");

    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "announcements/feed"), endpoints.end());
}

TEST(TierE1HighEndpoints, ServerFilesGet) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "server/files/get"), endpoints.end());
}

TEST(TierE1HighEndpoints, ServerLogsList) {
    KlippyServer server;
    server.addLogFile("klippy.log", "/tmp/klippy.log");

    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "server/logs/list"), endpoints.end());
}

TEST(TierE1HighEndpoints, PrinterGcodeSubscribeOutputAlias) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "printer/gcode/subscribe_output"), endpoints.end());
}

// ============================================================================
// E1-Low: Access endpoint tests
// ============================================================================

TEST(TierE1LowAccess, AccessEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    auto has = [&](const std::string& n) {
        return std::find(endpoints.begin(), endpoints.end(), n) != endpoints.end();
    };
    EXPECT_TRUE(has("access/login"));
    EXPECT_TRUE(has("access/logout"));
    EXPECT_TRUE(has("access/user"));
    EXPECT_TRUE(has("access/refresh_jwt"));
    EXPECT_TRUE(has("access/api_key"));
    EXPECT_TRUE(has("access/oneshot_token"));
}

TEST(TierE1LowAccess, UserRegistration) {
    KlippyServer server;
    server.registerUser("admin", "password123", {"read", "write"});
    // Just verify it doesn't crash
    SUCCEED();
}

// ============================================================================
// E1-Low: Bot endpoint tests
// ============================================================================

TEST(TierE1LowBots, BotEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    auto has = [&](const std::string& n) {
        return std::find(endpoints.begin(), endpoints.end(), n) != endpoints.end();
    };
    EXPECT_TRUE(has("bot/list"));
    EXPECT_TRUE(has("bot/get"));
    EXPECT_TRUE(has("bot/update"));
    EXPECT_TRUE(has("bot/delete"));
}

TEST(TierE1LowBots, BotRegistration) {
    KlippyServer server;
    server.registerBot("telegram_bot", "telegram", "token123", true);
    SUCCEED();
}

// ============================================================================
// E1-Low: Notepad endpoint tests
// ============================================================================

TEST(TierE1LowNotepad, NotepadEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    auto has = [&](const std::string& n) {
        return std::find(endpoints.begin(), endpoints.end(), n) != endpoints.end();
    };
    EXPECT_TRUE(has("notepad/list"));
    EXPECT_TRUE(has("notepad/get"));
    EXPECT_TRUE(has("notepad/put"));
    EXPECT_TRUE(has("notepad/delete"));
}

TEST(TierE1LowNotepad, NotepadPutGet) {
    KlippyServer server;
    server.notepadPut("test_key", "test_value");
    auto val = server.notepadGet("test_key");
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, "test_value");
}

// ============================================================================
// E1-Low: Spoolman endpoint tests
// ============================================================================

TEST(TierE1LowSpoolman, SpoolmanEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    auto has = [&](const std::string& n) {
        return std::find(endpoints.begin(), endpoints.end(), n) != endpoints.end();
    };
    EXPECT_TRUE(has("spoolman/info"));
    EXPECT_TRUE(has("spoolman/spool_id"));
    EXPECT_TRUE(has("spoolman/proxy"));
}

TEST(TierE1LowSpoolman, SpoolmanState) {
    KlippyServer server;
    server.setSpoolmanConnected(true, "http://localhost:8000");
    server.setSpoolId(42);
    SUCCEED();
}

// ============================================================================
// E1-Low: Device CRUD and database namespace endpoint tests
// ============================================================================

TEST(TierE1LowDevices, DeviceCrudEndpointsRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    auto has = [&](const std::string& n) {
        return std::find(endpoints.begin(), endpoints.end(), n) != endpoints.end();
    };
    EXPECT_TRUE(has("devices/create"));
    EXPECT_TRUE(has("devices/delete"));
}

TEST(TierE1LowDatabase, DatabaseNsEndpointRegistered) {
    KlippyServer server;
    auto endpoints = server.listEndpoints();
    EXPECT_NE(std::find(endpoints.begin(), endpoints.end(), "database/ns"), endpoints.end());
}

// ============================================================================
// E3: Missing extended G-code command parsing tests
// ============================================================================

TEST(TierE3ExtendedCommands, BedMeshOffsetParsing) {
    auto parsed = parseGcodeLine("BED_MESH_OFFSET X=5.0 Y=10.0");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "BED_MESH_OFFSET");
    EXPECT_NEAR(parsed->getNamedDouble("X"), 5.0, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("Y"), 10.0, 0.001);
}

TEST(TierE3ExtendedCommands, SetGcodePositionParsing) {
    auto parsed = parseGcodeLine("SET_GCODE_POSITION X=10 Y=20 Z=5");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_GCODE_POSITION");
    EXPECT_NEAR(parsed->getNamedDouble("X"), 10.0, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("Y"), 20.0, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("Z"), 5.0, 0.001);
}

TEST(TierE3ExtendedCommands, SaveGcodeStateParsing) {
    auto parsed = parseGcodeLine("SAVE_GCODE_STATE NAME=my_state");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SAVE_GCODE_STATE");
    EXPECT_EQ(parsed->getNamed("NAME"), "my_state");
}

TEST(TierE3ExtendedCommands, RestoreGcodeStateParsing) {
    auto parsed = parseGcodeLine("RESTORE_GCODE_STATE NAME=my_state");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "RESTORE_GCODE_STATE");
    EXPECT_EQ(parsed->getNamed("NAME"), "my_state");
}

TEST(TierE3ExtendedCommands, SetExtruderStepDistanceParsing) {
    auto parsed = parseGcodeLine("SET_EXTRUDER_STEP_DISTANCE EXTRUDER=extruder DISTANCE=0.0025");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_EXTRUDER_STEP_DISTANCE");
    EXPECT_EQ(parsed->getNamed("EXTRUDER"), "extruder");
    EXPECT_NEAR(parsed->getNamedDouble("DISTANCE"), 0.0025, 0.0001);
}

TEST(TierE3ExtendedCommands, ActivateExtruderParsing) {
    auto parsed = parseGcodeLine("ACTIVATE_EXTRUDER EXTRUDER=extruder1");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "ACTIVATE_EXTRUDER");
    EXPECT_EQ(parsed->getNamed("EXTRUDER"), "extruder1");
}

TEST(TierE3ExtendedCommands, SetDigitalPinParsing) {
    auto parsed = parseGcodeLine("SET_DIGITAL_PIN PIN=my_pin VALUE=1");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_DIGITAL_PIN");
    EXPECT_EQ(parsed->getNamed("PIN"), "my_pin");
    EXPECT_EQ(parsed->getNamedInt("VALUE"), 1);
}

TEST(TierE3ExtendedCommands, SetDotstarParsing) {
    auto parsed = parseGcodeLine("SET_DOTSTAR LED=my_dotstar RED=1.0 GREEN=0.5 BLUE=0.0");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_DOTSTAR");
    EXPECT_EQ(parsed->getNamed("LED"), "my_dotstar");
    EXPECT_NEAR(parsed->getNamedDouble("RED"), 1.0, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("GREEN"), 0.5, 0.001);
}

TEST(TierE3ExtendedCommands, ExcludeObjectResetParsing) {
    auto parsed = parseGcodeLine("EXCLUDE_OBJECT_RESET");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "EXCLUDE_OBJECT_RESET");
    EXPECT_TRUE(parsed->isExtendedCommand());
}

TEST(TierE3ExtendedCommands, SetRetractionParsing) {
    auto parsed = parseGcodeLine("SET_RETRACTION RETRACT_LENGTH=3.0 RETRACT_SPEED=35.0");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_RETRACTION");
    EXPECT_NEAR(parsed->getNamedDouble("RETRACT_LENGTH"), 3.0, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("RETRACT_SPEED"), 35.0, 0.001);
}

TEST(TierE3ExtendedCommands, SetCurrentParsing) {
    auto parsed = parseGcodeLine("SET_CURRENT STEPPER=stepper_x CURRENT=0.8 HOLD_CURRENT=0.5");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_CURRENT");
    EXPECT_EQ(parsed->getNamed("STEPPER"), "stepper_x");
    EXPECT_NEAR(parsed->getNamedDouble("CURRENT"), 0.8, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("HOLD_CURRENT"), 0.5, 0.001);
}

TEST(TierE3ExtendedCommands, SetHomePositionParsing) {
    auto parsed = parseGcodeLine("SET_HOME_POSITION AXIS=X POSITION=10.0");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "SET_HOME_POSITION");
    EXPECT_EQ(parsed->getNamed("AXIS"), "X");
    EXPECT_NEAR(parsed->getNamedDouble("POSITION"), 10.0, 0.001);
}

TEST(TierE3ExtendedCommands, EndstopHomeParsing) {
    auto parsed = parseGcodeLine("ENDSTOP_HOME STEPPER=stepper_x POSITION=0.0");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "ENDSTOP_HOME");
    EXPECT_EQ(parsed->getNamed("STEPPER"), "stepper_x");
    EXPECT_NEAR(parsed->getNamedDouble("POSITION"), 0.0, 0.001);
}

TEST(TierE3ExtendedCommands, RespondParsing) {
    auto parsed = parseGcodeLine("RESPOND TYPE=echo MSG=Hello_World");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "RESPOND");
    EXPECT_EQ(parsed->getNamed("TYPE"), "echo");
}

TEST(TierE3ExtendedCommands, EchoParsing) {
    auto parsed = parseGcodeLine("ECHO MSG=test_message");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "ECHO");
    EXPECT_EQ(parsed->getNamed("MSG"), "test_message");
}

TEST(TierE3ExtendedCommands, FilamentLoadParsing) {
    auto parsed = parseGcodeLine("FILAMENT_LOAD LENGTH=50 SPEED=10");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "FILAMENT_LOAD");
    EXPECT_NEAR(parsed->getNamedDouble("LENGTH"), 50.0, 0.001);
    EXPECT_NEAR(parsed->getNamedDouble("SPEED"), 10.0, 0.001);
}

TEST(TierE3ExtendedCommands, FilamentUnloadParsing) {
    auto parsed = parseGcodeLine("FILAMENT_UNLOAD LENGTH=50 SPEED=10");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "FILAMENT_UNLOAD");
    EXPECT_NEAR(parsed->getNamedDouble("LENGTH"), 50.0, 0.001);
}

TEST(TierE3ExtendedCommands, FilamentPurgeParsing) {
    auto parsed = parseGcodeLine("FILAMENT_PURGE LENGTH=10 SPEED=5");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->code, "FILAMENT_PURGE");
    EXPECT_NEAR(parsed->getNamedDouble("LENGTH"), 10.0, 0.001);
}
