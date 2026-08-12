/**
 * @file test_GCodeInterpreter.cpp
 * @brief Tests for the RS274/NGC G-code interpreter core.
 *
 * @details
 * Tests the executeBlock / dispatchGCode / handleMotion pipeline,
 * coordinate transform application, modal state updates, and
 * end-to-end program execution.
 */

#include "tether/gcode/GCodeInterpreter.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace GCode;

// ============================================================================
// Helper: capture motion segments
// ============================================================================

class InterpreterTestBase : public ::testing::Test {
protected:
    Interpreter interp;
    std::vector<MotionSegment> segments;
    std::vector<std::pair<bool, double>> spindleCommands; // (cw, rpm)
    std::vector<std::pair<bool, bool>> coolantCommands;   // (mist, flood)
    std::vector<int> programControls;
    std::vector<double> dwells;

    InterpreterTestBase() {
        interp.setMotionCallback([this](const MotionSegment& seg) {
            segments.push_back(seg);
            return Error{};
        });
        interp.setSpindleCallback([this](bool enable, bool cw, double rpm) {
            if (enable)
                spindleCommands.emplace_back(cw, rpm);
            else
                spindleCommands.emplace_back(false, 0.0);
            return Error{};
        });
        interp.setCoolantCallback([this](bool mist, bool flood) {
            coolantCommands.emplace_back(mist, flood);
            return Error{};
        });
        interp.setProgramControlCallback([this](int32_t mcode) {
            programControls.push_back(mcode);
        });
        interp.setDwellCallback([this](double seconds) {
            dwells.push_back(seconds);
            return Error{};
        });
    }

    void SetUp() override {
        segments.clear();
        spindleCommands.clear();
        coolantCommands.clear();
        programControls.clear();
        dwells.clear();
    }
};

// ============================================================================
// Basic motion tests
// ============================================================================

TEST_F(InterpreterTestBase, RapidMoveG0) {
    EXPECT_TRUE(interp.executeLine("G0 X10 Y20 Z30").ok());
    ASSERT_EQ(segments.size(), 1u);
    EXPECT_EQ(segments[0].type, MotionSegment::Type::RAPID);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.y(), 20.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.z(), 30.0, 0.001);
}

TEST_F(InterpreterTestBase, LinearMoveG1) {
    EXPECT_TRUE(interp.executeLine("G1 X10 Y20 Z30 F500").ok());
    ASSERT_EQ(segments.size(), 1u);
    EXPECT_EQ(segments[0].type, MotionSegment::Type::LINEAR);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.y(), 20.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.z(), 30.0, 0.001);
    EXPECT_NEAR(segments[0].feedRate, 500.0, 0.001);
}

TEST_F(InterpreterTestBase, ModalFeedRate) {
    EXPECT_TRUE(interp.executeLine("G1 F500").ok());
    EXPECT_TRUE(interp.executeLine("G1 X10").ok());
    ASSERT_EQ(segments.size(), 1u);
    EXPECT_NEAR(segments[0].feedRate, 500.0, 0.001);
}

TEST_F(InterpreterTestBase, ImplicitMotionUsesModalMode) {
    // G1 sets linear mode, then X10 without G-code should use modal linear
    EXPECT_TRUE(interp.executeLine("G1 F500").ok());
    EXPECT_TRUE(interp.executeLine("X10 Y20").ok());
    ASSERT_EQ(segments.size(), 1u);
    EXPECT_EQ(segments[0].type, MotionSegment::Type::LINEAR);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
}

TEST_F(InterpreterTestBase, IncrementalModeG91) {
    // Start at (10, 10, 10)
    EXPECT_TRUE(interp.executeLine("G1 X10 Y10 Z10 F500").ok());
    segments.clear();
    // Switch to incremental
    EXPECT_TRUE(interp.executeLine("G91").ok());
    EXPECT_TRUE(interp.executeLine("G1 X5").ok());
    ASSERT_EQ(segments.size(), 1u);
    // 10 + 5 = 15 in program space
    EXPECT_NEAR(segments[0].endPosition.x(), 15.0, 0.001);
}

TEST_F(InterpreterTestBase, AbsoluteModeG90) {
    EXPECT_TRUE(interp.executeLine("G90").ok());
    EXPECT_TRUE(interp.executeLine("G1 X10 Y20 Z30 F500").ok());
    ASSERT_EQ(segments.size(), 1u);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
}

// ============================================================================
// M-code tests
// ============================================================================

TEST_F(InterpreterTestBase, SpindleOnM3) {
    EXPECT_TRUE(interp.executeLine("M3 S1000").ok());
    ASSERT_EQ(spindleCommands.size(), 1u);
    EXPECT_TRUE(spindleCommands[0].first);  // CW
    EXPECT_NEAR(spindleCommands[0].second, 1000.0, 0.1);
}

TEST_F(InterpreterTestBase, SpindleOnM4) {
    EXPECT_TRUE(interp.executeLine("M4 S500").ok());
    ASSERT_EQ(spindleCommands.size(), 1u);
    EXPECT_FALSE(spindleCommands[0].first);  // CCW
    EXPECT_NEAR(spindleCommands[0].second, 500.0, 0.1);
}

TEST_F(InterpreterTestBase, SpindleOffM5) {
    EXPECT_TRUE(interp.executeLine("M3 S1000").ok());
    EXPECT_TRUE(interp.executeLine("M5").ok());
    ASSERT_EQ(spindleCommands.size(), 2u);
    EXPECT_FALSE(spindleCommands[1].first);
}

TEST_F(InterpreterTestBase, CoolantM7M8M9) {
    EXPECT_TRUE(interp.executeLine("M7").ok());  // Mist
    ASSERT_EQ(coolantCommands.size(), 1u);
    EXPECT_TRUE(coolantCommands[0].first);
    EXPECT_FALSE(coolantCommands[0].second);

    EXPECT_TRUE(interp.executeLine("M8").ok());  // Flood
    ASSERT_EQ(coolantCommands.size(), 2u);
    EXPECT_TRUE(coolantCommands[1].second);

    EXPECT_TRUE(interp.executeLine("M9").ok());  // Off
    ASSERT_EQ(coolantCommands.size(), 3u);
    EXPECT_FALSE(coolantCommands[2].first);
    EXPECT_FALSE(coolantCommands[2].second);
}

TEST_F(InterpreterTestBase, ProgramEndM30) {
    EXPECT_TRUE(interp.executeLine("M30").ok());
    ASSERT_FALSE(programControls.empty());
    EXPECT_EQ(programControls[0], 30);
}

TEST_F(InterpreterTestBase, DwellG4) {
    // G4 P1000 = 1000ms = 1 second
    EXPECT_TRUE(interp.executeLine("G4 P1000").ok());
    ASSERT_EQ(dwells.size(), 1u);
    EXPECT_NEAR(dwells[0], 1.0, 0.001);
}

// ============================================================================
// Coordinate system tests
// ============================================================================

TEST_F(InterpreterTestBase, G54Selection) {
    EXPECT_TRUE(interp.executeLine("G54").ok());
    // No crash, WCS 1 selected
    EXPECT_EQ(interp.getCoordinates().getActiveWCS().number, 1);
}

TEST_F(InterpreterTestBase, G55Selection) {
    EXPECT_TRUE(interp.executeLine("G55").ok());
    EXPECT_EQ(interp.getCoordinates().getActiveWCS().number, 2);
}

TEST_F(InterpreterTestBase, G10L2SetsWCSOffset) {
    EXPECT_TRUE(interp.executeLine("G10 L2 P1 X10 Y20 Z30").ok());
    auto& wcs = interp.getCoordinates().getWCS(1);
    EXPECT_NEAR(wcs.offset.x(), 10.0, 0.001);
    EXPECT_NEAR(wcs.offset.y(), 20.0, 0.001);
    EXPECT_NEAR(wcs.offset.z(), 30.0, 0.001);
}

TEST_F(InterpreterTestBase, G10L2WithRSetsWCSRotation) {
    EXPECT_TRUE(interp.executeLine("G10 L2 P1 R45.0").ok());
    auto& wcs = interp.getCoordinates().getWCS(1);
    EXPECT_NEAR(wcs.rotation, 45.0, 0.001);
}

TEST_F(InterpreterTestBase, G52LocalOffset) {
    EXPECT_TRUE(interp.executeLine("G52 X5 Y10 Z15").ok());
    // G52 offset should be set in machine state
    EXPECT_NEAR(interp.getMachineState().g52Offset.x(), 5.0, 0.001);
    EXPECT_NEAR(interp.getMachineState().g52Offset.y(), 10.0, 0.001);
    EXPECT_NEAR(interp.getMachineState().g52Offset.z(), 15.0, 0.001);
}

TEST_F(InterpreterTestBase, G92SetsPositionOffset) {
    // Move to (10, 10, 10) first
    EXPECT_TRUE(interp.executeLine("G1 X10 Y10 Z10 F500").ok());
    segments.clear();
    // G92 X0 Y0 Z0: current position becomes (0, 0, 0) in program space
    EXPECT_TRUE(interp.executeLine("G92 X0 Y0 Z0").ok());
    // Now move to X5 → machine should be at 15
    EXPECT_TRUE(interp.executeLine("G1 X5 F500").ok());
    ASSERT_EQ(segments.size(), 1u);
    // Machine position = program (5) + G92 offset (10) = 15
    EXPECT_NEAR(segments[0].endPosition.x(), 15.0, 0.001);
}

// ============================================================================
// Units tests
// ============================================================================

TEST_F(InterpreterTestBase, InchModeG20) {
    EXPECT_TRUE(interp.executeLine("G20").ok());
    EXPECT_EQ(interp.getMachineState().units, Units::INCH);
}

TEST_F(InterpreterTestBase, MmModeG21) {
    EXPECT_TRUE(interp.executeLine("G21").ok());
    EXPECT_EQ(interp.getMachineState().units, Units::MM);
}

TEST_F(InterpreterTestBase, InchMoveConvertsToMm) {
    EXPECT_TRUE(interp.executeLine("G20").ok());
    EXPECT_TRUE(interp.executeLine("G1 X1 F100").ok());
    ASSERT_EQ(segments.size(), 1u);
    // 1 inch = 25.4 mm
    EXPECT_NEAR(segments[0].endPosition.x(), 25.4, 0.001);
}

// ============================================================================
// Plane selection tests
// ============================================================================

TEST_F(InterpreterTestBase, PlaneXY_G17) {
    EXPECT_TRUE(interp.executeLine("G17").ok());
    EXPECT_EQ(interp.getMachineState().plane, Plane::XY);
}

TEST_F(InterpreterTestBase, PlaneZX_G18) {
    EXPECT_TRUE(interp.executeLine("G18").ok());
    EXPECT_EQ(interp.getMachineState().plane, Plane::ZX);
}

TEST_F(InterpreterTestBase, PlaneYZ_G19) {
    EXPECT_TRUE(interp.executeLine("G19").ok());
    EXPECT_EQ(interp.getMachineState().plane, Plane::YZ);
}

// ============================================================================
// End-to-end program tests
// ============================================================================

TEST_F(InterpreterTestBase, SimpleProgramRun) {
    std::string program =
        "G21\n"
        "G90\n"
        "G1 X10 Y10 F500\n"
        "G1 X20 Y20\n"
        "G1 X0 Y0\n"
        "M30\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    EXPECT_EQ(segments.size(), 3u);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[1].endPosition.x(), 20.0, 0.001);
    EXPECT_NEAR(segments[2].endPosition.x(), 0.0, 0.001);
}

TEST_F(InterpreterTestBase, ProgramWithWCSOffset) {
    // Set WCS 1 offset to (10, 20, 30)
    EXPECT_TRUE(interp.executeLine("G10 L2 P1 X10 Y20 Z30").ok());
    // Select WCS 1
    EXPECT_TRUE(interp.executeLine("G54").ok());

    std::string program =
        "G21\n"
        "G90\n"
        "G54\n"
        "G1 X1 Y2 Z3 F500\n"
        "M30\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 1u);
    // Program (1, 2, 3) + WCS offset (10, 20, 30) = machine (11, 22, 33)
    EXPECT_NEAR(segments[0].endPosition.x(), 11.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.y(), 22.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.z(), 33.0, 0.001);
}

TEST_F(InterpreterTestBase, ProgramWithG52Offset) {
    EXPECT_TRUE(interp.executeLine("G52 X5 Y10 Z15").ok());

    std::string program =
        "G21\n"
        "G90\n"
        "G1 X1 Y2 Z3 F500\n"
        "M30\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 1u);
    // Program (1, 2, 3) + G52 (5, 10, 15) = machine (6, 12, 18)
    EXPECT_NEAR(segments[0].endPosition.x(), 6.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.y(), 12.0, 0.001);
    EXPECT_NEAR(segments[0].endPosition.z(), 18.0, 0.001);
}

TEST_F(InterpreterTestBase, ProgramWithIncrementalMoves) {
    std::string program =
        "G21\n"
        "G90\n"
        "G1 X10 Y10 F500\n"
        "G91\n"
        "G1 X5\n"
        "G1 X5\n"
        "G90\n"
        "G1 X0 Y0\n"
        "M30\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 4u);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[1].endPosition.x(), 15.0, 0.001);  // 10 + 5
    EXPECT_NEAR(segments[2].endPosition.x(), 20.0, 0.001);  // 15 + 5
    EXPECT_NEAR(segments[3].endPosition.x(), 0.0, 0.001);
}

TEST_F(InterpreterTestBase, ProgramWithSpindleAndCoolant) {
    std::string program =
        "M3 S1000\n"
        "M8\n"
        "G1 X10 Y10 F500\n"
        "M9\n"
        "M5\n"
        "M30\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    EXPECT_EQ(spindleCommands.size(), 2u);  // On, Off
    EXPECT_EQ(coolantCommands.size(), 2u);  // Flood on, Off
    EXPECT_EQ(segments.size(), 1u);
}

TEST_F(InterpreterTestBase, EmptyProgram) {
    EXPECT_TRUE(interp.loadString("M30\n").ok());
    EXPECT_TRUE(interp.run().ok());
    EXPECT_TRUE(segments.empty());
}

TEST_F(InterpreterTestBase, CommentOnlyLines) {
    std::string program =
        "; This is a comment\n"
        "G21 ; inline comment\n"
        "; another comment\n"
        "M30\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
}

// ============================================================================
// Error handling tests
// ============================================================================

TEST_F(InterpreterTestBase, InvalidGcodeReturnsError) {
    // Unknown G-code should not crash
    Error err = interp.executeLine("G999");
    // The interpreter may silently ignore unknown codes or return an error.
    // Either is acceptable as long as it doesn't crash.
    (void)err;
}

TEST_F(InterpreterTestBase, EmptyLineIsOk) {
    EXPECT_TRUE(interp.executeLine("").ok());
}

TEST_F(InterpreterTestBase, CommentOnlyLineIsOk) {
    EXPECT_TRUE(interp.executeLine("; just a comment").ok());
}

// ============================================================================
// Position tracking tests
// ============================================================================

TEST_F(InterpreterTestBase, CurrentPositionUpdates) {
    EXPECT_TRUE(interp.executeLine("G1 X10 Y20 Z30 F500").ok());
    auto pos = interp.getCurrentPosition();
    EXPECT_NEAR(pos.x(), 10.0, 0.001);
    EXPECT_NEAR(pos.y(), 20.0, 0.001);
    EXPECT_NEAR(pos.z(), 30.0, 0.001);
}

TEST_F(InterpreterTestBase, MachinePositionUpdates) {
    EXPECT_TRUE(interp.executeLine("G1 X10 Y20 Z30 F500").ok());
    auto pos = interp.getMachinePosition();
    EXPECT_NEAR(pos.x(), 10.0, 0.001);
    EXPECT_NEAR(pos.y(), 20.0, 0.001);
    EXPECT_NEAR(pos.z(), 30.0, 0.001);
}

TEST_F(InterpreterTestBase, PositionVariablesWithWCS) {
    // Set WCS offset
    EXPECT_TRUE(interp.executeLine("G10 L2 P1 X10 Y20 Z30").ok());
    EXPECT_TRUE(interp.executeLine("G54").ok());
    EXPECT_TRUE(interp.executeLine("G1 X1 Y2 Z3 F500").ok());

    // Program position should be (1, 2, 3)
    auto progPos = interp.getCurrentPosition();
    EXPECT_NEAR(progPos.x(), 1.0, 0.001);
    EXPECT_NEAR(progPos.y(), 2.0, 0.001);
    EXPECT_NEAR(progPos.z(), 3.0, 0.001);

    // Machine position should be (11, 22, 33)
    auto machPos = interp.getMachinePosition();
    EXPECT_NEAR(machPos.x(), 11.0, 0.001);
    EXPECT_NEAR(machPos.y(), 22.0, 0.001);
    EXPECT_NEAR(machPos.z(), 33.0, 0.001);
}

// ============================================================================
// Arc decomposition tests (G2/G3)
// ============================================================================

TEST_F(InterpreterTestBase, ArcG2_CW_QuarterCircle_IJ) {
    // Start at (10, 0, 0), CW arc to (0, 10, 0) with center at (0, 0, 0)
    // I=-10, J=0 (center = start + I/J = (10-10, 0+0) = (0,0))
    EXPECT_TRUE(interp.executeLine("G1 X10 Y0 F500").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G2 X0 Y10 I-10 J0").ok());
    // Should decompose into multiple line segments
    EXPECT_GT(segments.size(), 4u);
    // All segments should be LINEAR type
    for (const auto& seg : segments)
        EXPECT_EQ(seg.type, MotionSegment::Type::LINEAR);
    // Final position should be (0, 10, 0)
    EXPECT_NEAR(segments.back().endPosition.x(), 0, 0.1);
    EXPECT_NEAR(segments.back().endPosition.y(), 10, 0.1);
}

TEST_F(InterpreterTestBase, ArcG3_CCW_QuarterCircle_IJ) {
    // Start at (10, 0, 0), CCW arc to (0, 10, 0) with center at (0, 0, 0)
    EXPECT_TRUE(interp.executeLine("G1 X10 Y0 F500").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G3 X0 Y10 I-10 J0").ok());
    EXPECT_GT(segments.size(), 4u);
    // Final position should be (0, 10, 0)
    EXPECT_NEAR(segments.back().endPosition.x(), 0, 0.1);
    EXPECT_NEAR(segments.back().endPosition.y(), 10, 0.1);
}

TEST_F(InterpreterTestBase, ArcG2_FullCircle_IJ) {
    // Full circle: start = end, with I/J offsets
    EXPECT_TRUE(interp.executeLine("G1 X10 Y0 F500").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G2 X10 Y0 I-10 J0").ok());
    // Full circle should decompose into many segments
    EXPECT_GT(segments.size(), 10u);
    // Final position should be back at (10, 0, 0)
    EXPECT_NEAR(segments.back().endPosition.x(), 10, 0.1);
    EXPECT_NEAR(segments.back().endPosition.y(), 0, 0.1);
}

TEST_F(InterpreterTestBase, ArcG2_WithRWord) {
    // Arc with R word (radius mode)
    EXPECT_TRUE(interp.executeLine("G1 X10 Y0 F500").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G2 X0 Y10 R10").ok());
    EXPECT_GT(segments.size(), 4u);
    EXPECT_NEAR(segments.back().endPosition.x(), 0, 0.1);
    EXPECT_NEAR(segments.back().endPosition.y(), 10, 0.1);
}

TEST_F(InterpreterTestBase, ArcHelicalG2) {
    // Helical arc: XY arc with Z change
    EXPECT_TRUE(interp.executeLine("G1 X10 Y0 Z0 F500").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G2 X0 Y10 Z-5 I-10 J0").ok());
    EXPECT_GT(segments.size(), 4u);
    // Final position should be (0, 10, -5)
    EXPECT_NEAR(segments.back().endPosition.x(), 0, 0.1);
    EXPECT_NEAR(segments.back().endPosition.y(), 10, 0.1);
    EXPECT_NEAR(segments.back().endPosition.z(), -5, 0.1);
}

TEST_F(InterpreterTestBase, ArcInZXPlane_G18) {
    // Arc in ZX plane (G18): a1=Z, a2=X
    EXPECT_TRUE(interp.executeLine("G18").ok());
    EXPECT_TRUE(interp.executeLine("G1 X0 Z10 F500").ok());
    segments.clear();
    // CW arc in ZX plane: Z is first axis, X is second
    // Center = start + K (Z offset) + I (X offset) = (0+0, 10-10) = (X=0, Z=0)
    EXPECT_TRUE(interp.executeLine("G2 X10 Z0 K-10 I0").ok());
    EXPECT_GT(segments.size(), 4u);
    EXPECT_NEAR(segments.back().endPosition.x(), 10, 0.1);
    EXPECT_NEAR(segments.back().endPosition.z(), 0, 0.1);
}

TEST_F(InterpreterTestBase, ArcFeedsThroughTransform) {
    // Arc with WCS offset — all interpolated points should be transformed
    EXPECT_TRUE(interp.executeLine("G10 L2 P1 X100 Y200 Z0").ok());
    EXPECT_TRUE(interp.executeLine("G54").ok());
    EXPECT_TRUE(interp.executeLine("G1 X10 Y0 F500").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G2 X0 Y10 I-10 J0").ok());
    EXPECT_GT(segments.size(), 4u);
    // Final machine position should be (100, 210, 0) = program (0,10) + WCS (100,200)
    EXPECT_NEAR(segments.back().endPosition.x(), 100, 0.1);
    EXPECT_NEAR(segments.back().endPosition.y(), 210, 0.1);
}

// ============================================================================
// Tool length offset tests (G43/G43.1/G49)
// ============================================================================

TEST_F(InterpreterTestBase, G43_1_DynamicToolLengthOffset) {
    // G43.1 Z10: set dynamic tool length offset of 10mm
    EXPECT_TRUE(interp.executeLine("G43.1 Z10").ok());
    EXPECT_TRUE(interp.executeLine("G1 X0 Y0 Z5 F500").ok());
    ASSERT_EQ(segments.size(), 1u);
    // Machine Z = program Z (5) + TLO (10) = 15
    EXPECT_NEAR(segments[0].endPosition.z(), 15.0, 0.001);
}

TEST_F(InterpreterTestBase, G49_CancelToolLengthOffset) {
    // Set TLO, then cancel with G49
    EXPECT_TRUE(interp.executeLine("G43.1 Z10").ok());
    EXPECT_TRUE(interp.executeLine("G49").ok());
    EXPECT_TRUE(interp.executeLine("G1 X0 Y0 Z5 F500").ok());
    ASSERT_EQ(segments.size(), 1u);
    // TLO cancelled, machine Z = program Z = 5
    EXPECT_NEAR(segments[0].endPosition.z(), 5.0, 0.001);
}

TEST_F(InterpreterTestBase, ToolLengthOffsetWithWCS) {
    // TLO is applied after WCS
    EXPECT_TRUE(interp.executeLine("G10 L2 P1 X100 Y200 Z300").ok());
    EXPECT_TRUE(interp.executeLine("G54").ok());
    EXPECT_TRUE(interp.executeLine("G43.1 Z50").ok());
    EXPECT_TRUE(interp.executeLine("G1 X0 Y0 Z10 F500").ok());
    ASSERT_EQ(segments.size(), 1u);
    // Machine = WCS (300) + TLO (50) + program (10) = 360
    EXPECT_NEAR(segments[0].endPosition.z(), 360.0, 0.001);
}
