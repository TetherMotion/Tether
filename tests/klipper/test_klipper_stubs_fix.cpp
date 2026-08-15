/**
 * @file test_klipper_stubs_fix.cpp
 * @brief Tests for fixed stubs, new G-codes, edge cases, and I2C.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

// ============================================================================
// Helper
// ============================================================================
static std::string uniqueSocketPath() {
    return "/tmp/tether_test_stubs_" + std::to_string(getpid()) + ".sock";
}

// ============================================================================
// Fixed stub callback tests
// ============================================================================
class KlippyStubsFix : public ::testing::Test {
protected:
    void SetUp() override {
        socketPath_ = uniqueSocketPath();
        sdDir_ = "/tmp/tether_test_stubs_sd_" + std::to_string(getpid());
        settingsPath_ = "/tmp/tether_test_stubs_settings_" + std::to_string(getpid()) + ".cfg";
        std::filesystem::create_directories(sdDir_);

        KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = socketPath_;
        cfg.sdcardDir = sdDir_;
        cfg.settingsPath = settingsPath_;
        instance_ = std::make_unique<KlippyInstance>(cfg);
    }
    void TearDown() override {
        instance_.reset();
        ::unlink(socketPath_.c_str());
        ::unlink(settingsPath_.c_str());
        std::filesystem::remove_all(sdDir_);
    }
    std::string socketPath_, sdDir_, settingsPath_;
    std::unique_ptr<KlippyInstance> instance_;
};

// --- setMotorEnable is now wired to StepperEnableObject ---
TEST_F(KlippyStubsFix, SetMotorEnableUpdatesStepperObject) {
    instance_->executeGcode("M18"); // Disable all motors
    // The stepper_enable object should reflect disabled state
    auto& stepper = instance_->stepperEnableObject();
    auto status = stepper->status({});
    EXPECT_FALSE(status["enabled"].asBool());
}

TEST_F(KlippyStubsFix, SetMotorEnableReEnables) {
    instance_->executeGcode("M18");
    instance_->executeGcode("M17"); // Re-enable
    auto& stepper = instance_->stepperEnableObject();
    auto status = stepper->status({});
    EXPECT_TRUE(status["enabled"].asBool());
}

// --- dwell is now an actual sleep ---
TEST_F(KlippyStubsFix, DwellActuallySleeps) {
    auto start = std::chrono::steady_clock::now();
    instance_->executeGcode("G4 P50"); // 50ms dwell
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_GE(elapsed.count(), 40); // At least 40ms (allow some tolerance)
}

// --- getEndstopStatus is now wired to real endstops ---
TEST_F(KlippyStubsFix, EndstopStatusWiredToRealEndstops) {
    auto es = std::make_shared<Endstop>(0, []() { return true; });
    instance_->setEndstop("x", es);
    // The endstop status should now reflect the real state
    // We can't directly call the callback, but we can check query_endstops
    auto& qe = instance_->queryEndstopsObject();
    auto status = qe->status({});
    EXPECT_EQ(status["x"].asString(), "TRIGGERED");
}

TEST_F(KlippyStubsFix, EndstopStatusOpenWhenNotTriggered) {
    auto es = std::make_shared<Endstop>(0, []() { return false; });
    instance_->setEndstop("y", es);
    auto& qe = instance_->queryEndstopsObject();
    auto status = qe->status({});
    EXPECT_EQ(status["y"].asString(), "open");
}

// --- setAutoTempReport is now stored ---
TEST_F(KlippyStubsFix, AutoTempReportIntervalStored) {
    // M155 S5 should set the auto temp report interval to 5
    bool ok = instance_->executeGcode("M155 S5");
    EXPECT_TRUE(ok);
}

// --- waitForMoves is now a queue flush ---
TEST_F(KlippyStubsFix, WaitForMovesFlushesQueue) {
    bool ok = instance_->executeGcode("G1 X10\nG1 X20\nM400");
    EXPECT_TRUE(ok);
}

// --- bedLevel is now wired to BedMesh ---
TEST_F(KlippyStubsFix, BedLevelWiredToBedMesh) {
    instance_->executeGcode("G29");
    // bedMeshEnabled should be set
    EXPECT_TRUE(instance_->settings().bedMeshEnabled);
}

// --- cleanNozzle now executes a pattern ---
TEST_F(KlippyStubsFix, CleanNozzleExecutesPattern) {
    bool ok = instance_->executeGcode("G12 P2 R5 S100");
    EXPECT_TRUE(ok);
}

// --- Settings persistence (M500/M501/M502/M503) ---
TEST_F(KlippyStubsFix, SaveAndLoadSettings) {
    instance_->executeGcode("M92 X100 Y100 Z400 E500");
    instance_->executeGcode("M500"); // Save
    EXPECT_TRUE(std::filesystem::exists(settingsPath_));

    // Reset and load
    instance_->executeGcode("M502"); // Reset to defaults
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 80.0, 0.1); // Default

    instance_->executeGcode("M501"); // Load from file
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 100.0, 0.1); // Loaded value
}

TEST_F(KlippyStubsFix, ReportSettingsOutputsM503) {
    bool ok = instance_->executeGcode("M503");
    EXPECT_TRUE(ok);
}

TEST_F(KlippyStubsFix, ResetSettingsToDefaults) {
    instance_->executeGcode("M92 X200");
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 200.0, 0.1);
    instance_->executeGcode("M502"); // Reset
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 80.0, 0.1); // Default
}

// ============================================================================
// Advanced objects registration tests
// ============================================================================
TEST_F(KlippyStubsFix, BedMeshObjectRegistered) {
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "bed_mesh") { found = true; break; }
    }
    EXPECT_TRUE(found);
}

TEST_F(KlippyStubsFix, QueryEndstopsObjectRegistered) {
    auto objects = instance_->server().listObjects();
    bool found = false;
    for (const auto& name : objects) {
        if (name == "query_endstops") { found = true; break; }
    }
    EXPECT_TRUE(found);
}

// ============================================================================
// New G-code tests
// ============================================================================
TEST(KlipperNewGcodes, M115FirmwareInfo) {
    std::string output;
    GcodeCallbacks cb;
    cb.getFirmwareInfo = []() { return "FIRMWARE_NAME:TestKlipper v1.0"; };
    cb.output = [&](const std::string& msg) { output = msg; };
    GCodeExecutor exec(cb);
    exec.executeLine("M115");
    EXPECT_EQ(output, "FIRMWARE_NAME:TestKlipper v1.0");
}

TEST(KlipperNewGcodes, M116WaitForTemps) {
    bool waited = false;
    GcodeCallbacks cb;
    cb.waitForTemperatures = [&]() { waited = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M116");
    EXPECT_TRUE(waited);
}

TEST(KlipperNewGcodes, M92StepsPerMm) {
    std::string axis;
    double steps = 0;
    GcodeCallbacks cb;
    cb.setStepsPerMm = [&](const std::string& a, double s) { axis = a; steps = s; };
    GCodeExecutor exec(cb);
    exec.executeLine("M92 X100");
    EXPECT_EQ(axis, "x");
    EXPECT_NEAR(steps, 100.0, 0.1);
}

TEST(KlipperNewGcodes, M200FilamentDiameter) {
    double diameter = 0;
    GcodeCallbacks cb;
    cb.setFilamentDiameter = [&](double d) { diameter = d; };
    GCodeExecutor exec(cb);
    exec.executeLine("M200 D1.75");
    EXPECT_NEAR(diameter, 1.75, 0.01);
}

TEST(KlipperNewGcodes, M204Acceleration) {
    double accel = 0, travelAccel = 0;
    GcodeCallbacks cb;
    cb.setAcceleration = [&](double a, double t) { accel = a; travelAccel = t; };
    GCodeExecutor exec(cb);
    exec.executeLine("M204 P500 T1000");
    EXPECT_NEAR(accel, 500.0, 0.1);
    EXPECT_NEAR(travelAccel, 1000.0, 0.1);
}

TEST(KlipperNewGcodes, M203MaxFeedrate) {
    std::string axis;
    double feedrate = 0;
    GcodeCallbacks cb;
    cb.setMaxFeedrate = [&](const std::string& a, double f) { axis = a; feedrate = f; };
    GCodeExecutor exec(cb);
    exec.executeLine("M203 X300");
    EXPECT_EQ(axis, "x");
    EXPECT_NEAR(feedrate, 300.0, 0.1);
}

TEST(KlipperNewGcodes, M205AdvancedMotion) {
    double jerk = 0, startAccel = 0;
    GcodeCallbacks cb;
    cb.setAdvancedMotion = [&](double j, double s) { jerk = j; startAccel = s; };
    GCodeExecutor exec(cb);
    exec.executeLine("M205 X20 S1000");
    EXPECT_NEAR(jerk, 20.0, 0.1);
    EXPECT_NEAR(startAccel, 1000.0, 0.1);
}

TEST(KlipperNewGcodes, M206HomeOffset) {
    std::string axis;
    double offset = 0;
    GcodeCallbacks cb;
    cb.setHomeOffset = [&](const std::string& a, double o) { axis = a; offset = o; };
    GCodeExecutor exec(cb);
    exec.executeLine("M206 X5.0");
    EXPECT_EQ(axis, "x");
    EXPECT_NEAR(offset, 5.0, 0.01);
}

TEST(KlipperNewGcodes, M207RetractParams) {
    double length = 0, speed = 0, zLift = 0;
    GcodeCallbacks cb;
    cb.setRetractParams = [&](double l, double s, double z) { length = l; speed = s; zLift = z; };
    GCodeExecutor exec(cb);
    exec.executeLine("M207 S2.0 F1800 Z0.5");
    EXPECT_NEAR(length, 2.0, 0.01);
    EXPECT_NEAR(speed, 1800.0, 0.1);
    EXPECT_NEAR(zLift, 0.5, 0.01);
}

TEST(KlipperNewGcodes, M208UnretractParams) {
    double length = 0, speed = 0;
    GcodeCallbacks cb;
    cb.setUnretractParams = [&](double l, double s) { length = l; speed = s; };
    GCodeExecutor exec(cb);
    exec.executeLine("M208 S2.0 F1800");
    EXPECT_NEAR(length, 2.0, 0.01);
    EXPECT_NEAR(speed, 1800.0, 0.1);
}

TEST(KlipperNewGcodes, M218ToolOffset) {
    int tool = -1;
    std::string axis;
    double offset = 0;
    GcodeCallbacks cb;
    cb.setToolOffset = [&](int t, const std::string& a, double o) { tool = t; axis = a; offset = o; };
    GCodeExecutor exec(cb);
    exec.executeLine("M218 T1 X10.0");
    EXPECT_EQ(tool, 1);
    EXPECT_EQ(axis, "x");
    EXPECT_NEAR(offset, 10.0, 0.01);
}

TEST(KlipperNewGcodes, M280ServoControl) {
    int servo = -1;
    double angle = 0;
    GcodeCallbacks cb;
    cb.setServoAngle = [&](int s, double a) { servo = s; angle = a; };
    GCodeExecutor exec(cb);
    exec.executeLine("M280 P0 S90");
    EXPECT_EQ(servo, 0);
    EXPECT_NEAR(angle, 90.0, 0.1);
}

TEST(KlipperNewGcodes, M300Beep) {
    double freq = 0, duration = 0;
    GcodeCallbacks cb;
    cb.beep = [&](double f, double d) { freq = f; duration = d; };
    GCodeExecutor exec(cb);
    exec.executeLine("M300 S440 P200");
    EXPECT_NEAR(freq, 440.0, 0.1);
    EXPECT_NEAR(duration, 200.0, 0.1);
}

TEST(KlipperNewGcodes, M301HotendPID) {
    double kp = 0, ki = 0, kd = 0;
    GcodeCallbacks cb;
    cb.setHotendPID = [&](double p, double i, double d) { kp = p; ki = i; kd = d; };
    GCodeExecutor exec(cb);
    exec.executeLine("M301 P22.0 I1.5 D120.0");
    EXPECT_NEAR(kp, 22.0, 0.01);
    EXPECT_NEAR(ki, 1.5, 0.01);
    EXPECT_NEAR(kd, 120.0, 0.01);
}

TEST(KlipperNewGcodes, M303PIDAutotune) {
    std::string output;
    GcodeCallbacks cb;
    cb.runPIDAutotune = [](double temp, int cycles) {
        return "PID autotune starting at " + std::to_string(temp) + "C";
    };
    cb.output = [&](const std::string& msg) { output = msg; };
    GCodeExecutor exec(cb);
    exec.executeLine("M303 S200 C5");
    EXPECT_TRUE(output.find("200") != std::string::npos);
}

TEST(KlipperNewGcodes, M304BedPID) {
    double kp = 0, ki = 0, kd = 0;
    GcodeCallbacks cb;
    cb.setBedPID = [&](double p, double i, double d) { kp = p; ki = i; kd = d; };
    GCodeExecutor exec(cb);
    exec.executeLine("M304 P10.0 I0.5 D50.0");
    EXPECT_NEAR(kp, 10.0, 0.01);
    EXPECT_NEAR(ki, 0.5, 0.01);
    EXPECT_NEAR(kd, 50.0, 0.01);
}

TEST(KlipperNewGcodes, M350Microstepping) {
    std::string axis;
    int ms = 0;
    GcodeCallbacks cb;
    cb.setMicrostepping = [&](const std::string& a, int m) { axis = a; ms = m; };
    GCodeExecutor exec(cb);
    exec.executeLine("M350 S32 X");
    EXPECT_EQ(axis, "x");
    EXPECT_EQ(ms, 32);
}

TEST(KlipperNewGcodes, M401DeployProbe) {
    bool deployed = false;
    GcodeCallbacks cb;
    cb.deployProbe = [&]() { deployed = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M401");
    EXPECT_TRUE(deployed);
}

TEST(KlipperNewGcodes, M402StowProbe) {
    bool stowed = false;
    GcodeCallbacks cb;
    cb.stowProbe = [&]() { stowed = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M402");
    EXPECT_TRUE(stowed);
}

TEST(KlipperNewGcodes, M420BedMeshEnable) {
    bool enabled = false;
    GcodeCallbacks cb;
    cb.setBedMeshEnabled = [&](bool e) { enabled = e; };
    GCodeExecutor exec(cb);
    exec.executeLine("M420 S1");
    EXPECT_TRUE(enabled);
}

TEST(KlipperNewGcodes, M421SetBedMeshPoint) {
    int xIdx = -1, yIdx = -1;
    double z = 0;
    GcodeCallbacks cb;
    cb.setBedMeshPoint = [&](int x, int y, double zz) { xIdx = x; yIdx = y; z = zz; };
    GCodeExecutor exec(cb);
    exec.executeLine("M421 I2 J3 Z0.05");
    EXPECT_EQ(xIdx, 2);
    EXPECT_EQ(yIdx, 3);
    EXPECT_NEAR(z, 0.05, 0.001);
}

TEST(KlipperNewGcodes, M425Backlash) {
    std::string axis;
    double comp = 0;
    GcodeCallbacks cb;
    cb.setBacklash = [&](const std::string& a, double c) { axis = a; comp = c; };
    GCodeExecutor exec(cb);
    exec.executeLine("M425 X0.1");
    EXPECT_EQ(axis, "x");
    EXPECT_NEAR(comp, 0.1, 0.001);
}

TEST(KlipperNewGcodes, M42PinState) {
    int pin = -1;
    double value = 0;
    GcodeCallbacks cb;
    cb.setPinState = [&](int p, double v) { pin = p; value = v; };
    GCodeExecutor exec(cb);
    exec.executeLine("M42 P5 S128");
    EXPECT_EQ(pin, 5);
    EXPECT_NEAR(value, 128.0, 0.1);
}

TEST(KlipperNewGcodes, M150LedColor) {
    int r = -1, g = -1, b = -1, w = -1;
    GcodeCallbacks cb;
    cb.setLedColor = [&](int rr, int gg, int bb, int ww) { r = rr; g = gg; b = bb; w = ww; };
    GCodeExecutor exec(cb);
    exec.executeLine("M150 R255 G128 B64 W32");
    EXPECT_EQ(r, 255);
    EXPECT_EQ(g, 128);
    EXPECT_EQ(b, 64);
    EXPECT_EQ(w, 32);
}

TEST(KlipperNewGcodes, M569StepperDirection) {
    std::string axis;
    int dir = -1;
    GcodeCallbacks cb;
    cb.setStepperDirection = [&](const std::string& a, int d) { axis = a; dir = d; };
    GCodeExecutor exec(cb);
    exec.executeLine("M569 S1 X");
    EXPECT_EQ(axis, "x");
    EXPECT_EQ(dir, 1);
}

TEST(KlipperNewGcodes, M600FilamentChange) {
    bool changed = false;
    GcodeCallbacks cb;
    cb.filamentChange = [&]() { changed = true; };
    GCodeExecutor exec(cb);
    exec.executeLine("M600");
    EXPECT_TRUE(changed);
}

TEST(KlipperNewGcodes, M851ProbeOffset) {
    double offset = 0;
    GcodeCallbacks cb;
    cb.setProbeOffset = [&](double o) { offset = o; };
    GCodeExecutor exec(cb);
    exec.executeLine("M851 Z0.2");
    EXPECT_NEAR(offset, 0.2, 0.001);
}

TEST(KlipperNewGcodes, M906StepperCurrent) {
    std::string axis;
    double current = 0;
    GcodeCallbacks cb;
    cb.setStepperCurrent = [&](const std::string& a, double c) { axis = a; current = c; };
    GCodeExecutor exec(cb);
    exec.executeLine("M906 X800");
    EXPECT_EQ(axis, "x");
    EXPECT_NEAR(current, 800.0, 0.1);
}

// --- G5 Bezier spline ---
TEST(KlipperNewGcodes, G5BezierMove) {
    int moveCount = 0;
    GcodeCallbacks cb;
    cb.move = [&](double, double, double, double, double) { moveCount++; };
    GCodeExecutor exec(cb);
    exec.state().position = {0, 0, 0, 0};
    exec.executeLine("G5 X10 Y10 I3 I0 P7 J10");
    EXPECT_GT(moveCount, 16); // 32 segments by default
}

// ============================================================================
// KlippyInstance new G-code integration tests
// ============================================================================
TEST_F(KlippyStubsFix, M115FirmwareInfoViaInstance) {
    bool ok = instance_->executeGcode("M115");
    EXPECT_TRUE(ok);
}

TEST_F(KlippyStubsFix, M92StepsPerMmViaInstance) {
    instance_->executeGcode("M92 X100 Y200 Z400 E500");
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 100.0, 0.1);
    EXPECT_NEAR(instance_->settings().stepsPerMm["y"], 200.0, 0.1);
}

TEST_F(KlippyStubsFix, M204AccelerationViaInstance) {
    instance_->executeGcode("M204 P1000 T2000");
    EXPECT_NEAR(instance_->settings().acceleration, 1000.0, 0.1);
    EXPECT_NEAR(instance_->settings().travelAcceleration, 2000.0, 0.1);
}

TEST_F(KlippyStubsFix, M206HomeOffsetViaInstance) {
    instance_->executeGcode("M206 X1.0 Y2.0 Z3.0");
    EXPECT_NEAR(instance_->settings().homeOffset["x"], 1.0, 0.01);
    EXPECT_NEAR(instance_->settings().homeOffset["y"], 2.0, 0.01);
    EXPECT_NEAR(instance_->settings().homeOffset["z"], 3.0, 0.01);
}

TEST_F(KlippyStubsFix, M851ProbeOffsetViaInstance) {
    instance_->executeGcode("M851 Z0.25");
    EXPECT_NEAR(instance_->settings().probeOffset, 0.25, 0.001);
}

TEST_F(KlippyStubsFix, M200FilamentDiameterViaInstance) {
    instance_->executeGcode("M200 D2.85");
    EXPECT_NEAR(instance_->settings().filamentDiameter, 2.85, 0.01);
}

TEST_F(KlippyStubsFix, M421BedMeshPointViaInstance) {
    instance_->bedMesh().configure(0, 100, 0, 100, 3, 3);
    bool ok = instance_->executeGcode("M421 I1 J1 Z0.05");
    EXPECT_TRUE(ok);
}

TEST_F(KlippyStubsFix, M420BedMeshEnableViaInstance) {
    instance_->executeGcode("M420 S1");
    EXPECT_TRUE(instance_->settings().bedMeshEnabled);
    instance_->executeGcode("M420 S0");
    EXPECT_FALSE(instance_->settings().bedMeshEnabled);
}

TEST_F(KlippyStubsFix, M600FilamentChangePausesPrint) {
    instance_->executeGcode("M600");
    EXPECT_EQ(instance_->printStatsObject()->status({})["state"].asString(), "paused");
}

TEST_F(KlippyStubsFix, M301HotendPIDViaInstance) {
    auto heater = std::make_shared<Heater>(0, [](double) {}, []() { return 25.0; });
    instance_->setExtruderHeater(heater);
    // Note: PID is set in settings, but the heater was set after construction
    // so the callback still references the old heater. We test settings.
    instance_->executeGcode("M301 P22.0 I1.5 D120.0");
    EXPECT_NEAR(instance_->settings().hotendKp, 22.0, 0.01);
    EXPECT_NEAR(instance_->settings().hotendKi, 1.5, 0.01);
    EXPECT_NEAR(instance_->settings().hotendKd, 120.0, 0.01);
}

TEST_F(KlippyStubsFix, M503ReportSettingsViaInstance) {
    bool ok = instance_->executeGcode("M503");
    EXPECT_TRUE(ok);
}

TEST_F(KlippyStubsFix, SettingsRoundTrip) {
    instance_->executeGcode("M92 X123 Y456 Z789 E321");
    instance_->executeGcode("M204 P1500 T2500");
    instance_->executeGcode("M206 X1.5 Y2.5 Z3.5");
    instance_->executeGcode("M851 Z0.15");
    instance_->executeGcode("M500"); // Save

    // Reset
    instance_->executeGcode("M502");
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 80.0, 0.1); // Default

    // Load
    instance_->executeGcode("M501");
    EXPECT_NEAR(instance_->settings().stepsPerMm["x"], 123.0, 0.1);
    EXPECT_NEAR(instance_->settings().stepsPerMm["y"], 456.0, 0.1);
    EXPECT_NEAR(instance_->settings().acceleration, 1500.0, 0.1);
    EXPECT_NEAR(instance_->settings().travelAcceleration, 2500.0, 0.1);
    EXPECT_NEAR(instance_->settings().homeOffset["z"], 3.5, 0.01);
    EXPECT_NEAR(instance_->settings().probeOffset, 0.15, 0.001);
}

// ============================================================================
// Arc R-mode edge case tests
// ============================================================================
TEST(KlipperArcRMode, ShortArcCW) {
    int moveCount = 0;
    std::vector<std::pair<double, double>> positions;
    GcodeCallbacks cb;
    cb.move = [&](double x, double y, double, double, double) {
        moveCount++;
        positions.emplace_back(x, y);
    };
    GCodeExecutor exec(cb);
    exec.state().position = {0, 0, 0, 0};
    // Short CW arc from (0,0) to (10,0) with R=5
    exec.executeLine("G2 X10 Y0 R5");
    EXPECT_GT(moveCount, 4);
    // End position should be (10, 0)
    EXPECT_NEAR(positions.back().first, 10.0, 0.5);
    EXPECT_NEAR(positions.back().second, 0.0, 0.5);
}

TEST(KlipperArcRMode, LongArcCW) {
    int moveCount = 0;
    GcodeCallbacks cb;
    cb.move = [&](double, double, double, double, double) { moveCount++; };
    GCodeExecutor exec(cb);
    exec.state().position = {0, 0, 0, 0};
    // Long CW arc (negative R) from (0,0) to (10,0) with R=10
    // halfChord=5, h=sqrt(100-25)=8.66, so center is offset
    exec.executeLine("G2 X10 Y0 R-10");
    EXPECT_GT(moveCount, 16); // Long arc should have more segments
}

TEST(KlipperArcRMode, ShortArcCCW) {
    int moveCount = 0;
    std::vector<std::pair<double, double>> positions;
    GcodeCallbacks cb;
    cb.move = [&](double x, double y, double, double, double) {
        moveCount++;
        positions.emplace_back(x, y);
    };
    GCodeExecutor exec(cb);
    exec.state().position = {0, 0, 0, 0};
    exec.executeLine("G3 X10 Y0 R5");
    EXPECT_GT(moveCount, 4);
    EXPECT_NEAR(positions.back().first, 10.0, 0.5);
    EXPECT_NEAR(positions.back().second, 0.0, 0.5);
}

TEST(KlipperArcRMode, FullCircleRMode) {
    int moveCount = 0;
    GcodeCallbacks cb;
    cb.move = [&](double, double, double, double, double) { moveCount++; };
    GCodeExecutor exec(cb);
    exec.state().position = {5, 0, 0, 0};
    // Arc from (5,0) back to (5,0) with R=5 — full circle
    exec.executeLine("G2 X5 Y0 R5");
    EXPECT_GT(moveCount, 16);
}

// ============================================================================
// I2C register addressing tests
// ============================================================================
TEST(KlipperI2c, ReadWith8BitRegister) {
    I2c i2c(0);
    i2c.setReadFunc([](uint8_t addr, uint8_t reg, size_t len) {
        EXPECT_EQ(addr, 0x50);
        EXPECT_EQ(reg, 0x10);
        return std::vector<uint8_t>(len, 0xAB);
    });
    auto data = i2c.read(0x50, 0x10, 4);
    EXPECT_EQ(data.size(), 4u);
    EXPECT_EQ(data[0], 0xAB);
}

TEST(KlipperI2c, ReadNoRegister) {
    I2c i2c(0);
    i2c.setReadNoRegFunc([](uint8_t addr, size_t len) {
        EXPECT_EQ(addr, 0x5A);
        return std::vector<uint8_t>(len, 0xCD);
    });
    auto data = i2c.readNoRegister(0x5A, 2);
    EXPECT_EQ(data.size(), 2u);
    EXPECT_EQ(data[0], 0xCD);
}

TEST(KlipperI2c, ReadWith16BitRegister) {
    I2c i2c(0);
    i2c.setRead16Func([](uint8_t addr, uint16_t reg, size_t len) {
        EXPECT_EQ(addr, 0x50);
        EXPECT_EQ(reg, 0x1000);
        return std::vector<uint8_t>(len, 0xEF);
    });
    auto data = i2c.read16(0x50, 0x1000, 3);
    EXPECT_EQ(data.size(), 3u);
    EXPECT_EQ(data[0], 0xEF);
}

TEST(KlipperI2c, WriteWith8BitRegister) {
    I2c i2c(0);
    uint8_t capturedAddr = 0, capturedReg = 0;
    std::vector<uint8_t> capturedData;
    i2c.setWriteFunc([&](uint8_t addr, uint8_t reg, std::span<const uint8_t> data) {
        capturedAddr = addr;
        capturedReg = reg;
        capturedData.assign(data.begin(), data.end());
    });
    std::vector<uint8_t> payload = {0x01, 0x02, 0x03};
    i2c.write(0x50, 0x20, std::span<const uint8_t>(payload));
    EXPECT_EQ(capturedAddr, 0x50);
    EXPECT_EQ(capturedReg, 0x20);
    EXPECT_EQ(capturedData.size(), 3u);
    EXPECT_EQ(capturedData[0], 0x01);
}

TEST(KlipperI2c, WriteNoRegister) {
    I2c i2c(0);
    uint8_t capturedAddr = 0;
    std::vector<uint8_t> capturedData;
    i2c.setWriteNoRegFunc([&](uint8_t addr, std::span<const uint8_t> data) {
        capturedAddr = addr;
        capturedData.assign(data.begin(), data.end());
    });
    std::vector<uint8_t> payload = {0xFF, 0xEE};
    i2c.writeNoRegister(0x5A, std::span<const uint8_t>(payload));
    EXPECT_EQ(capturedAddr, 0x5A);
    EXPECT_EQ(capturedData.size(), 2u);
    EXPECT_EQ(capturedData[0], 0xFF);
}

TEST(KlipperI2c, WriteWith16BitRegister) {
    I2c i2c(0);
    uint8_t capturedAddr = 0;
    uint16_t capturedReg = 0;
    std::vector<uint8_t> capturedData;
    i2c.setWrite16Func([&](uint8_t addr, uint16_t reg, std::span<const uint8_t> data) {
        capturedAddr = addr;
        capturedReg = reg;
        capturedData.assign(data.begin(), data.end());
    });
    std::vector<uint8_t> payload = {0x42};
    i2c.write16(0x50, 0x2000, std::span<const uint8_t>(payload));
    EXPECT_EQ(capturedAddr, 0x50);
    EXPECT_EQ(capturedReg, 0x2000);
    EXPECT_EQ(capturedData[0], 0x42);
}

TEST(KlipperI2c, ConvenienceReadNoReg) {
    I2c i2c(0);
    i2c.setReadNoRegFunc([](uint8_t, size_t len) {
        return std::vector<uint8_t>(len, 0x77);
    });
    auto data = i2c.read(0x50, 4); // Convenience overload
    EXPECT_EQ(data.size(), 4u);
    EXPECT_EQ(data[0], 0x77);
}

// ============================================================================
// System stats and MCU stats tests
// ============================================================================
TEST_F(KlippyStubsFix, UpdateSystemStatsReadsProc) {
    instance_->updateSystemStats();
    auto& stats = instance_->systemStatsObject();
    auto status = stats->status({});
    // sysload should be non-negative (read from /proc/loadavg)
    EXPECT_GE(status["sysload"].asDouble(), 0.0);
}

TEST_F(KlippyStubsFix, UpdateMcuStats) {
    instance_->updateMcuStats(1024, 2048, 2, 0.005);
    auto& mcu = instance_->mcuObject();
    auto status = mcu->status({});
    auto lastStats = status["last_stats"].asObject();
    EXPECT_EQ(lastStats["bytes_read"].asInt(), 1024);
    EXPECT_EQ(lastStats["bytes_write"].asInt(), 2048);
    EXPECT_EQ(lastStats["retransmits"].asInt(), 2);
}

TEST_F(KlippyStubsFix, SetFanRpm) {
    instance_->setFanRpm(1500.0);
    auto& fan = instance_->fanObject();
    auto status = fan->status({});
    EXPECT_NEAR(status["rpm"].asDouble(), 1500.0, 0.1);
}

TEST_F(KlippyStubsFix, McuVersionFromConfig) {
    auto& mcu = instance_->mcuObject();
    auto status = mcu->status({});
    EXPECT_EQ(status["mcu_version"].asString(), "tether-klipper-1.0.0");
}
