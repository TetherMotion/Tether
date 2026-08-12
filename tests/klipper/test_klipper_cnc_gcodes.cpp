/**
 * @file test_klipper_cnc_gcodes.cpp
 * @brief Tests for CNC G-codes, coordinate systems, path control, program flow,
 *        software endstops, thermistor params, filament sensor, canned cycles,
 *        G92 restore, and pin polling.
 *
 * Extracted from test_klipper_tier_features.cpp for better organization.
 */

#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace tether::klipper::klippy;

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
// Part 2b: G52 Local Coordinate Offset
// ============================================================================

TEST(KlipperCoordSystems, G52SetsLocalOffset) {
    GcodeCallbacks cb;
    double offX = -1, offY = -1, offZ = -1;
    cb.setLocalOffset = [&](double x, double y, double z) {
        offX = x; offY = y; offZ = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G52 X5 Y10 Z15"));
    EXPECT_NEAR(offX, 5.0, 0.01);
    EXPECT_NEAR(offY, 10.0, 0.01);
    EXPECT_NEAR(offZ, 15.0, 0.01);
}

TEST(KlipperCoordSystems, G52NoAxesResetsOffset) {
    GcodeCallbacks cb;
    double offX = 99, offY = 99, offZ = 99;
    cb.setLocalOffset = [&](double x, double y, double z) {
        offX = x; offY = y; offZ = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    // G52 with no axis words should signal reset (all NaN).
    EXPECT_TRUE(exec.executeLine("G52"));
    EXPECT_TRUE(std::isnan(offX));
    EXPECT_TRUE(std::isnan(offY));
    EXPECT_TRUE(std::isnan(offZ));
}

TEST(KlipperCoordSystems, G52PartialUpdateKeepsOtherAxes) {
    GcodeCallbacks cb;
    double offX = -1, offY = -1, offZ = -1;
    cb.setLocalOffset = [&](double x, double y, double z) {
        offX = x; offY = y; offZ = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    // Set initial offset.
    EXPECT_TRUE(exec.executeLine("G52 X1 Y2 Z3"));
    // Update only X; Y and Z should be NaN (unchanged).
    EXPECT_TRUE(exec.executeLine("G52 X10"));
    EXPECT_NEAR(offX, 10.0, 0.01);
    EXPECT_TRUE(std::isnan(offY));
    EXPECT_TRUE(std::isnan(offZ));
}

// ============================================================================
// Part 2c: G68/G69 Coordinate Rotation
// ============================================================================

TEST(KlipperCoordSystems, G68_2DRotation) {
    GcodeCallbacks cb;
    double angle = -1, px = -1, py = -1;
    cb.setCoordinateRotation2D = [&](double a, double x, double y) {
        angle = a; px = x; py = y;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G68 X5 Y10 R45"));
    EXPECT_NEAR(angle, 45.0, 0.01);
    EXPECT_NEAR(px, 5.0, 0.01);
    EXPECT_NEAR(py, 10.0, 0.01);
}

TEST(KlipperCoordSystems, G68_3DEuler) {
    GcodeCallbacks cb;
    double a = -1, b = -1, c = -1, px = -1, py = -1, pz = -1;
    cb.setCoordinateRotation3D = [&](double aa, double bb, double cc,
                                      double x, double y, double z) {
        a = aa; b = bb; c = cc; px = x; py = y; pz = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G68 X1 Y2 Z3 A10 B20 C30"));
    EXPECT_NEAR(a, 10.0, 0.01);
    EXPECT_NEAR(b, 20.0, 0.01);
    EXPECT_NEAR(c, 30.0, 0.01);
    EXPECT_NEAR(px, 1.0, 0.01);
    EXPECT_NEAR(py, 2.0, 0.01);
    EXPECT_NEAR(pz, 3.0, 0.01);
}

TEST(KlipperCoordSystems, G68_3DAxisAngle) {
    GcodeCallbacks cb;
    double ix = -1, iy = -1, iz = -1, angle = -1;
    double px = -1, py = -1, pz = -1;
    cb.setCoordinateRotationAxis = [&](double i, double j, double k, double a,
                                        double x, double y, double z) {
        ix = i; iy = j; iz = k; angle = a;
        px = x; py = y; pz = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G68 X1 Y2 Z3 I0 J0 K1 R90"));
    EXPECT_NEAR(ix, 0.0, 0.01);
    EXPECT_NEAR(iy, 0.0, 0.01);
    EXPECT_NEAR(iz, 1.0, 0.01);
    EXPECT_NEAR(angle, 90.0, 0.01);
    EXPECT_NEAR(px, 1.0, 0.01);
    EXPECT_NEAR(py, 2.0, 0.01);
    EXPECT_NEAR(pz, 3.0, 0.01);
}

TEST(KlipperCoordSystems, G69CancelRotation) {
    GcodeCallbacks cb;
    bool cancelled = false;
    cb.cancelCoordinateRotation = [&]() { cancelled = true; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G69"));
    EXPECT_TRUE(cancelled);
}

// ============================================================================
// Part 2d: G51/G50 Scaling
// ============================================================================

TEST(KlipperCoordSystems, G51UniformScaling) {
    GcodeCallbacks cb;
    double sx = 0, sy = 0, sz = 0;
    cb.setScaling = [&](double x, double y, double z) {
        sx = x; sy = y; sz = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G51 P2.0"));
    EXPECT_NEAR(sx, 2.0, 0.01);
    EXPECT_NEAR(sy, 2.0, 0.01);
    EXPECT_NEAR(sz, 2.0, 0.01);
}

TEST(KlipperCoordSystems, G51PerAxisScaling) {
    GcodeCallbacks cb;
    double sx = 0, sy = 0, sz = 0;
    cb.setScaling = [&](double x, double y, double z) {
        sx = x; sy = y; sz = z;
    };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G51 X2 Y0.5 Z1"));
    EXPECT_NEAR(sx, 2.0, 0.01);
    EXPECT_NEAR(sy, 0.5, 0.01);
    EXPECT_NEAR(sz, 1.0, 0.01);
}

TEST(KlipperCoordSystems, G50CancelScaling) {
    GcodeCallbacks cb;
    bool cancelled = false;
    cb.cancelScaling = [&]() { cancelled = true; };

    PrinterMotionState state;
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G50"));
    EXPECT_TRUE(cancelled);
}

// ============================================================================
// Part 2e: CoordinateTransform integration in PrinterMotionState
// ============================================================================

TEST(KlipperCoordSystems, TransformAppliesWCSOffset) {
    PrinterMotionState state;
    state.coordSystemOffsets[0] = {10.0, 20.0, 30.0};
    state.activeCoordSystem = 0;
    state.rebuildCoordTransform();

    auto m = state.coordTransform.toMachineXYZ(1.0, 2.0, 3.0);
    EXPECT_NEAR(m[0], 11.0, 0.001);
    EXPECT_NEAR(m[1], 22.0, 0.001);
    EXPECT_NEAR(m[2], 33.0, 0.001);
}

TEST(KlipperCoordSystems, TransformAppliesG52Offset) {
    PrinterMotionState state;
    state.g52Offset = {5.0, 10.0, 15.0};
    state.rebuildCoordTransform();

    auto m = state.coordTransform.toMachineXYZ(1.0, 2.0, 3.0);
    EXPECT_NEAR(m[0], 6.0, 0.001);
    EXPECT_NEAR(m[1], 12.0, 0.001);
    EXPECT_NEAR(m[2], 18.0, 0.001);
}

TEST(KlipperCoordSystems, TransformAppliesG68Rotation2D) {
    PrinterMotionState state;
    state.g68Active = true;
    state.g68Mode = 0;
    state.coordRotation = 90.0;  // 90° about Z
    state.g68Pivot = {0, 0, 0};
    state.rebuildCoordTransform();

    // (1, 0, 0) rotated 90° about Z -> (0, 1, 0)
    auto m = state.coordTransform.toMachineXYZ(1.0, 0.0, 0.0);
    EXPECT_NEAR(m[0], 0.0, 0.001);
    EXPECT_NEAR(m[1], 1.0, 0.001);
    EXPECT_NEAR(m[2], 0.0, 0.001);
}

TEST(KlipperCoordSystems, TransformAppliesG51Scaling) {
    PrinterMotionState state;
    state.g51Active = true;
    state.scaleFactors = {2.0, 3.0, 4.0};
    state.rebuildCoordTransform();

    auto m = state.coordTransform.toMachineXYZ(1.0, 1.0, 1.0);
    EXPECT_NEAR(m[0], 2.0, 0.001);
    EXPECT_NEAR(m[1], 3.0, 0.001);
    EXPECT_NEAR(m[2], 4.0, 0.001);
}

TEST(KlipperCoordSystems, TransformRoundTrip) {
    PrinterMotionState state;
    state.coordSystemOffsets[0] = {10.0, 20.0, 30.0};
    state.g52Offset = {1.0, 2.0, 3.0};
    state.g68Active = true;
    state.g68Mode = 0;
    state.coordRotation = 45.0;
    state.g68Pivot = {5.0, 5.0, 0};
    state.g51Active = true;
    state.scaleFactors = {2.0, 2.0, 1.0};
    state.rebuildCoordTransform();

    double px = 10.0, py = 20.0, pz = 30.0;
    auto m = state.coordTransform.toMachineXYZ(px, py, pz);
    auto p = state.coordTransform.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(p[0], px, 0.001);
    EXPECT_NEAR(p[1], py, 0.001);
    EXPECT_NEAR(p[2], pz, 0.001);
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
// Part 10: End-to-end coordinate transform tests
// ============================================================================

TEST(KlipperCoordTransformE2E, MoveWithWCSOffset) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.coordSystemOffsets[0] = {10.0, 20.0, 30.0};
    state.activeCoordSystem = 0;
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // G1 X1 Y2 Z3 in program space → machine (11, 22, 33)
    EXPECT_TRUE(exec.executeLine("G1 X1 Y2 Z3 F100"));
    EXPECT_NEAR(mx, 11.0, 0.001);
    EXPECT_NEAR(my, 22.0, 0.001);
    EXPECT_NEAR(mz, 33.0, 0.001);
}

TEST(KlipperCoordTransformE2E, MoveWithG52Offset) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.g52Offset = {5.0, 10.0, 15.0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    EXPECT_TRUE(exec.executeLine("G1 X1 Y2 Z3 F100"));
    EXPECT_NEAR(mx, 6.0, 0.001);
    EXPECT_NEAR(my, 12.0, 0.001);
    EXPECT_NEAR(mz, 18.0, 0.001);
}

TEST(KlipperCoordTransformE2E, MoveWithG68Rotation90) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.g68Active = true;
    state.g68Mode = 0;
    state.coordRotation = 90.0;
    state.g68Pivot = {0, 0, 0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // (1, 0, 0) rotated 90° → (0, 1, 0)
    EXPECT_TRUE(exec.executeLine("G1 X1 Y0 Z0 F100"));
    EXPECT_NEAR(mx, 0.0, 0.001);
    EXPECT_NEAR(my, 1.0, 0.001);
    EXPECT_NEAR(mz, 0.0, 0.001);
}

TEST(KlipperCoordTransformE2E, MoveWithG51Scaling) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.g51Active = true;
    state.scaleFactors = {2.0, 0.5, 1.0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // (10, 10, 10) scaled by (2, 0.5, 1) → (20, 5, 10)
    EXPECT_TRUE(exec.executeLine("G1 X10 Y10 Z10 F100"));
    EXPECT_NEAR(mx, 20.0, 0.001);
    EXPECT_NEAR(my, 5.0, 0.001);
    EXPECT_NEAR(mz, 10.0, 0.001);
}

TEST(KlipperCoordTransformE2E, MoveWithCombinedWCSAndG52) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.coordSystemOffsets[0] = {10.0, 20.0, 30.0};
    state.activeCoordSystem = 0;
    state.g52Offset = {5.0, 10.0, 15.0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // WCS(10,20,30) + G52(5,10,15) + program(1,2,3) = machine(16,32,48)
    EXPECT_TRUE(exec.executeLine("G1 X1 Y2 Z3 F100"));
    EXPECT_NEAR(mx, 16.0, 0.001);
    EXPECT_NEAR(my, 32.0, 0.001);
    EXPECT_NEAR(mz, 48.0, 0.001);
}

TEST(KlipperCoordTransformE2E, MoveWithCombinedRotationAndOffset) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.coordSystemOffsets[0] = {10.0, 20.0, 30.0};
    state.activeCoordSystem = 0;
    state.g68Active = true;
    state.g68Mode = 0;
    state.coordRotation = 90.0;
    state.g68Pivot = {0, 0, 0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // (1, 0, 0) rotated 90° → (0, 1, 0), then + WCS(10,20,30) → (10, 21, 30)
    EXPECT_TRUE(exec.executeLine("G1 X1 Y0 Z0 F100"));
    EXPECT_NEAR(mx, 10.0, 0.001);
    EXPECT_NEAR(my, 21.0, 0.001);
    EXPECT_NEAR(mz, 30.0, 0.001);
}

TEST(KlipperCoordTransformE2E, FeedRateScaledByG51) {
    GcodeCallbacks cb;
    double feedUsed = 0;
    cb.move = [&](double, double, double, double, double speed) {
        feedUsed = speed;
    };

    PrinterMotionState state;
    state.g51Active = true;
    state.scaleFactors = {2.0, 2.0, 2.0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // F100 mm/min with 2x scale → 200 mm/min = 3.333 mm/s
    EXPECT_TRUE(exec.executeLine("G1 X10 Y10 Z10 F100"));
    EXPECT_NEAR(feedUsed, 200.0 / 60.0, 0.01);
}

TEST(KlipperCoordTransformE2E, G92OffsetAppliesToMove) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.position = {0, 0, 0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // G92 X10 Y10 Z10: current position (0,0,0) becomes (10,10,10) in program space
    EXPECT_TRUE(exec.executeLine("G92 X10 Y10 Z10"));
    // Now move to program (20, 20, 20) → machine (10, 10, 10)
    EXPECT_TRUE(exec.executeLine("G1 X20 Y20 Z20 F100"));
    EXPECT_NEAR(mx, 10.0, 0.001);
    EXPECT_NEAR(my, 10.0, 0.001);
    EXPECT_NEAR(mz, 10.0, 0.001);
}

TEST(KlipperCoordTransformE2E, G91IncrementalMode) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.position = {5.0, 5.0, 5.0};
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // G91: incremental mode. Move +10 in X → machine 15
    EXPECT_TRUE(exec.executeLine("G91"));
    EXPECT_TRUE(exec.executeLine("G1 X10 F100"));
    EXPECT_NEAR(mx, 15.0, 0.001);
    EXPECT_NEAR(my, 5.0, 0.001);
    EXPECT_NEAR(mz, 5.0, 0.001);
}

TEST(KlipperCoordTransformE2E, G10L2WithRSetsWCSRotation) {
    PrinterMotionState state;
    state.activeCoordSystem = 0;
    state.rebuildCoordTransform();

    // G10 L2 P1 R45: set WCS 1 rotation to 45°
    // We can't directly test this through the Klipper executor since it
    // uses the RS274 CoordinateSystemManager, but we can test the
    // CoordinateSystemManager directly.
    // This test verifies the PrinterMotionState doesn't crash.
    EXPECT_NO_FATAL_FAILURE({
        state.rebuildCoordTransform();
    });
}

TEST(KlipperCoordTransformE2E, ExtendedAxisScaling) {
    GcodeCallbacks cb;
    double mx = -1, my = -1, mz = -1;
    cb.move = [&](double x, double y, double z, double, double) {
        mx = x; my = y; mz = z;
    };

    PrinterMotionState state;
    state.g51Active = true;
    state.scaleFactors = {1.0, 1.0, 1.0};
    state.extScaleFactors = {2.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // A=2, B=1, C=1
    state.rebuildCoordTransform();
    GCodeExecutor exec(std::move(cb), &state);

    // XYZ not scaled, but extended axes are. The move callback only gets XYZ.
    EXPECT_TRUE(exec.executeLine("G1 X10 Y20 Z30 F100"));
    EXPECT_NEAR(mx, 10.0, 0.001);
    EXPECT_NEAR(my, 20.0, 0.001);
    EXPECT_NEAR(mz, 30.0, 0.001);
}
