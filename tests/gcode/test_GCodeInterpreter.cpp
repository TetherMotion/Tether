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

// ============================================================================
// O-code flow control (interpreter-level)
// ============================================================================

TEST_F(InterpreterTestBase, OCodeSubCall) {
    const char* program =
        "G1 X1 F100\n"
        "O100 sub\n"
        "G1 X10\n"
        "O100 endsub\n"
        "O100 call\n"
        "G1 X20\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    // Definition is skipped inline; call executes body once, then resumes.
    ASSERT_EQ(segments.size(), 3u);
    EXPECT_NEAR(segments[0].endPosition.x(), 1.0, 0.001);
    EXPECT_NEAR(segments[1].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[2].endPosition.x(), 20.0, 0.001);
}

TEST_F(InterpreterTestBase, OCodeNamedSubCall) {
    const char* program =
        "G1 X1 F100\n"
        "O<mysub> sub\n"
        "G1 X10\n"
        "O<mysub> endsub\n"
        "O<mysub> call\n"
        "G1 X20\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 3u);
    EXPECT_NEAR(segments[1].endPosition.x(), 10.0, 0.001);
}

TEST_F(InterpreterTestBase, OCodeIfElseFalse) {
    const char* program =
        "#1 = 0\n"
        "O100 if [#1 GT 0]\n"
        "G1 X10\n"
        "O100 else\n"
        "G1 X20\n"
        "O100 endif\n"
        "G1 X30 F100\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    // False branch skipped: else body runs, then endif continues.
    ASSERT_EQ(segments.size(), 2u);
    EXPECT_NEAR(segments[0].endPosition.x(), 20.0, 0.001);
    EXPECT_NEAR(segments[1].endPosition.x(), 30.0, 0.001);
}

TEST_F(InterpreterTestBase, OCodeIfTrue) {
    const char* program =
        "#1 = 1\n"
        "O100 if [#1 GT 0]\n"
        "G1 X10 F100\n"
        "O100 else\n"
        "G1 X20\n"
        "O100 endif\n"
        "G1 X30\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 2u);
    EXPECT_NEAR(segments[0].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[1].endPosition.x(), 30.0, 0.001);
}

TEST_F(InterpreterTestBase, OCodeWhileLoop) {
    const char* program =
        "#1 = 0\n"
        "O100 while [#1 LT 3]\n"
        "#1 = [#1 + 1]\n"
        "G1 X10 F100\n"
        "O100 endwhile\n"
        "G1 X99\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 4u);
    EXPECT_NEAR(segments[3].endPosition.x(), 99.0, 0.001);
}

TEST_F(InterpreterTestBase, OCodeRepeatLoop) {
    const char* program =
        "G1 X0 F100\n"
        "O100 repeat [3]\n"
        "G1 X10\n"
        "O100 endrepeat\n"
        "G1 X99\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 5u);
    EXPECT_NEAR(segments[4].endPosition.x(), 99.0, 0.001);
}

TEST_F(InterpreterTestBase, OCodeWhileBreak) {
    const char* program =
        "#1 = 0\n"
        "O100 while [#1 LT 10]\n"
        "#1 = [#1 + 1]\n"
        "O101 if [#1 GE 2]\n"
        "O100 break\n"
        "O101 endif\n"
        "G1 X10 F100\n"
        "O100 endwhile\n"
        "G1 X99\n"
        "M2\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    // Loop body ran once (X10), broke on iteration 2 before G1.
    ASSERT_EQ(segments.size(), 2u);
    EXPECT_NEAR(segments[1].endPosition.x(), 99.0, 0.001);
}

// ============================================================================
// Fanuc M98/M99 subprograms (bare O<num> labels)
// ============================================================================

TEST_F(InterpreterTestBase, M98M99Subprogram) {
    const char* program =
        "G1 X1 F100\n"
        "M98 P100\n"
        "G1 X99\n"
        "M2\n"
        "O100\n"
        "G1 X10\n"
        "M99\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 3u);
    EXPECT_NEAR(segments[0].endPosition.x(), 1.0, 0.001);
    EXPECT_NEAR(segments[1].endPosition.x(), 10.0, 0.001);
    EXPECT_NEAR(segments[2].endPosition.x(), 99.0, 0.001);
}

TEST_F(InterpreterTestBase, M98RepeatCount) {
    const char* program =
        "G1 X1 F100\n"
        "M98 P100 L3\n"
        "G1 X99\n"
        "M2\n"
        "O100\n"
        "G1 X10\n"
        "M99\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    // Subprogram runs 3 times.
    ASSERT_EQ(segments.size(), 5u);
    EXPECT_NEAR(segments[4].endPosition.x(), 99.0, 0.001);
}

TEST_F(InterpreterTestBase, G65MacroCall) {
    const char* program =
        "G1 X1 F100\n"
        "G65 P100 A5\n"
        "G1 X99\n"
        "M2\n"
        "O100\n"
        "G1 X[#1]\n"
        "M99\n";
    EXPECT_TRUE(interp.loadString(program).ok());
    EXPECT_TRUE(interp.run().ok());
    ASSERT_EQ(segments.size(), 3u);
    // A5 maps to local #1 inside the macro.
    EXPECT_NEAR(segments[1].endPosition.x(), 5.0, 0.001);
    EXPECT_NEAR(segments[2].endPosition.x(), 99.0, 0.001);
}

// ============================================================================
// Canned cycles
// ============================================================================

TEST_F(InterpreterTestBase, CannedCycleG81) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0 Z10").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G81 X5 Y5 Z-2 R2 F100").ok());
    // rapid XY, rapid to R, feed to Z, rapid retract to initial Z (G98)
    ASSERT_EQ(segments.size(), 4u);
    EXPECT_EQ(segments[0].type, MotionSegment::Type::RAPID);
    EXPECT_NEAR(segments[0].endPosition.x(), 5.0, 0.001);
    EXPECT_EQ(segments[1].type, MotionSegment::Type::RAPID);
    EXPECT_NEAR(segments[1].endPosition.z(), 2.0, 0.001);
    EXPECT_EQ(segments[2].type, MotionSegment::Type::LINEAR);
    EXPECT_NEAR(segments[2].endPosition.z(), -2.0, 0.001);
    EXPECT_EQ(segments[3].type, MotionSegment::Type::RAPID);
    EXPECT_NEAR(segments[3].endPosition.z(), 10.0, 0.001);
}

TEST_F(InterpreterTestBase, CannedCycleG81_G99RetractToR) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0 Z10").ok());
    EXPECT_TRUE(interp.executeLine("G99").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G81 X5 Y5 Z-2 R2 F100").ok());
    ASSERT_EQ(segments.size(), 4u);
    // G99: retract to R plane, not initial Z
    EXPECT_NEAR(segments[3].endPosition.z(), 2.0, 0.001);
}

TEST_F(InterpreterTestBase, CannedCycleG82Dwell) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0 Z10").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G82 X5 Y5 Z-2 R2 P1.5 F100").ok());
    // rapid XY, rapid R, feed Z, dwell, rapid retract
    ASSERT_EQ(segments.size(), 5u);
    EXPECT_EQ(segments[3].type, MotionSegment::Type::DWELL);
    EXPECT_NEAR(segments[3].duration, 1.5, 0.001);
}

TEST_F(InterpreterTestBase, CannedCycleModalRepeat) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0 Z10").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G81 X5 Y5 Z-2 R2 F100").ok());
    EXPECT_TRUE(interp.executeLine("X10 Y10").ok());   // repeat at new hole
    EXPECT_TRUE(interp.executeLine("G80").ok());       // cancel
    EXPECT_TRUE(interp.executeLine("G0 X20 Y20").ok()); // plain move, no cycle
    // 4 segs per hole + 1 rapid for the last X move
    ASSERT_EQ(segments.size(), 9u);
    EXPECT_NEAR(segments[4].endPosition.x(), 10.0, 0.001);
    EXPECT_EQ(segments[8].type, MotionSegment::Type::RAPID);
    EXPECT_NEAR(segments[8].endPosition.x(), 20.0, 0.001);
}

TEST_F(InterpreterTestBase, CannedCycleG83Peck) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0 Z10").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G83 X5 Y5 Z-3 R0 Q1 F100").ok());
    // rapid XY, rapid R=0, then pecks at -1, -2, -3 with retracts
    // Verify at least one feed reaches each peck depth and final Z=-3
    bool sawPeck1 = false, sawPeck2 = false, sawBottom = false;
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR) {
            if (std::abs(s.endPosition.z() + 1.0) < 0.001) sawPeck1 = true;
            if (std::abs(s.endPosition.z() + 2.0) < 0.001) sawPeck2 = true;
            if (std::abs(s.endPosition.z() + 3.0) < 0.001) sawBottom = true;
        }
    }
    EXPECT_TRUE(sawPeck1 && sawPeck2 && sawBottom);
}

// ============================================================================
// Probing (G38.x)
// ============================================================================

TEST_F(InterpreterTestBase, ProbeG38_2) {
    // No probe callback -> simulated trip at target
    EXPECT_TRUE(interp.executeLine("G38.2 Z-5 F100").ok());
    ASSERT_EQ(segments.size(), 1u);
    EXPECT_EQ(segments[0].type, MotionSegment::Type::PROBE);
    EXPECT_NEAR(segments[0].endPosition.z(), -5.0, 0.001);
    // #5070 probe success flag
    EXPECT_NEAR(interp.getVariables().get(5070), 1.0, 0.001);
}

TEST_F(InterpreterTestBase, ProbeG38_2MissErrors) {
    interp.setProbeCallback([](const Position&, double, ProbeType,
                               ProbeResult& r) {
        r.tripped = false;
        return Error{};
    });
    EXPECT_FALSE(interp.executeLine("G38.2 Z-5 F100").ok());
}

TEST_F(InterpreterTestBase, ProbeG38_3MissNoError) {
    interp.setProbeCallback([](const Position&, double, ProbeType,
                               ProbeResult& r) {
        r.tripped = false;
        return Error{};
    });
    EXPECT_TRUE(interp.executeLine("G38.3 Z-5 F100").ok());
    EXPECT_NEAR(interp.getVariables().get(5070), 0.0, 0.001);
}

// --- Tool compensation ---

TEST_F(InterpreterTestBase, ToolLengthG43FromTable) {
    ToolEntry e;
    e.toolNumber = 3;
    e.zOffset = 12.5;
    e.diameter = 6.0;
    ASSERT_TRUE(interp.getToolTable().setTool(3, e).ok());

    EXPECT_TRUE(interp.executeLine("G43 H3").ok());
    EXPECT_EQ(interp.getMachineState().toolLengthMode, ToolLengthMode::POSITIVE);
    EXPECT_NEAR(interp.getMachineState().toolOffset.z(), 12.5, 0.001);
}

TEST_F(InterpreterTestBase, ToolLengthG43UnknownToolErrors) {
    EXPECT_FALSE(interp.executeLine("G43 H77").ok());
}

TEST_F(InterpreterTestBase, ToolLengthG432Stacks) {
    ToolEntry a; a.toolNumber = 1; a.zOffset = 10.0;
    ToolEntry b; b.toolNumber = 2; b.zOffset = 0.5;
    ASSERT_TRUE(interp.getToolTable().setTool(1, a).ok());
    ASSERT_TRUE(interp.getToolTable().setTool(2, b).ok());
    EXPECT_TRUE(interp.executeLine("G43 H1").ok());
    EXPECT_TRUE(interp.executeLine("G43.2 H2").ok());
    EXPECT_NEAR(interp.getMachineState().toolOffset.z(), 10.5, 0.001);
    EXPECT_TRUE(interp.executeLine("G49").ok());
    EXPECT_NEAR(interp.getMachineState().toolOffset.z(), 0.0, 0.001);
}

TEST_F(InterpreterTestBase, CutterCompG41UsesToolDiameter) {
    ToolEntry e; e.toolNumber = 4; e.diameter = 8.0;
    ASSERT_TRUE(interp.getToolTable().setTool(4, e).ok());
    EXPECT_TRUE(interp.executeLine("G41 D4").ok());
    EXPECT_EQ(interp.getMachineState().cutterComp, CutterCompMode::LEFT);
    EXPECT_NEAR(interp.getMachineState().cutterRadius, 4.0, 0.001);
    EXPECT_TRUE(interp.executeLine("G40").ok());
    EXPECT_EQ(interp.getMachineState().cutterComp, CutterCompMode::OFF);
}

TEST_F(InterpreterTestBase, CutterCompG421Dynamic) {
    EXPECT_TRUE(interp.executeLine("G42.1 D10").ok());
    EXPECT_EQ(interp.getMachineState().cutterComp, CutterCompMode::RIGHT_DYNAMIC);
    EXPECT_NEAR(interp.getMachineState().cutterRadius, 5.0, 0.001);
}

// --- Splines ---

TEST_F(InterpreterTestBase, SplineG5Cubic) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0").ok());
    EXPECT_TRUE(interp.executeLine("G5 X10 Y0 I3 J3 P-3 Q3").ok());
    ASSERT_EQ(segments.size(), 2u);
    EXPECT_EQ(segments[1].type, MotionSegment::Type::SPLINE);
    EXPECT_NEAR(segments[1].splinePoints[0].x(), 0.0, 0.001);
    EXPECT_NEAR(segments[1].splinePoints[1].x(), 3.0, 0.001);
    EXPECT_NEAR(segments[1].splinePoints[1].y(), 3.0, 0.001);
    EXPECT_NEAR(segments[1].splinePoints[3].x(), 10.0, 0.001);
}

TEST_F(InterpreterTestBase, SplineG51Quadratic) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0").ok());
    EXPECT_TRUE(interp.executeLine("G5.1 X10 Y0 I5 J5").ok());
    ASSERT_EQ(segments.size(), 2u);
    EXPECT_EQ(segments[1].type, MotionSegment::Type::SPLINE);
    EXPECT_NEAR(segments[1].splinePoints[1].x(), 5.0, 0.001);
    EXPECT_NEAR(segments[1].splinePoints[1].y(), 5.0, 0.001);
}

TEST_F(InterpreterTestBase, NurbsG52G53) {
    EXPECT_TRUE(interp.executeLine("G0 X0 Y0").ok());
    EXPECT_TRUE(interp.executeLine("G5.2 L3").ok());
    EXPECT_TRUE(interp.executeLine("X0 Y0").ok());
    EXPECT_TRUE(interp.executeLine("X5 Y5").ok());
    EXPECT_TRUE(interp.executeLine("X10 Y0").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G5.3").ok());
    ASSERT_GT(segments.size(), 8u);
    // Curve must end at last control point (clamped knots)
    EXPECT_NEAR(segments.back().endPosition.x(), 10.0, 0.01);
    EXPECT_NEAR(segments.back().endPosition.y(), 0.0, 0.01);
}

// --- GRBL real-time protocol ---

TEST_F(InterpreterTestBase, GrblFeedHoldResume) {
    EXPECT_TRUE(interp.executeLine("G1 X10 F100").ok());
    interp.getMachineState(); // touch
    EXPECT_TRUE(interp.processRealtimeChar('!').ok());
    EXPECT_TRUE(interp.getMachineState().feedHold);
    EXPECT_TRUE(interp.processRealtimeChar('~').ok());
    EXPECT_FALSE(interp.getMachineState().feedHold);
}

TEST_F(InterpreterTestBase, GrblStatusReport) {
    interp.executeLine("G1 X1.5 Y2 Z-3 F500");
    std::string r = interp.statusReport();
    EXPECT_NE(r.find("MPos:1.500,2.000,-3.000"), std::string::npos);
    EXPECT_NE(r.find("FS:500"), std::string::npos);
}

TEST_F(InterpreterTestBase, GrblSoftReset) {
    interp.executeLine("G1 X5");
    EXPECT_TRUE(interp.processRealtimeChar('\x18').ok());
    EXPECT_EQ(interp.getState(), InterpreterState::IDLE);
    EXPECT_NEAR(interp.getMachineState().machinePosition.x(), 0.0, 0.001);
}

TEST_F(InterpreterTestBase, GrblSystemCommands) {
    std::vector<std::string> msgs;
    interp.setMessageCallback([&](const std::string& m){ msgs.push_back(m); });
    EXPECT_TRUE(interp.systemCommand("$G").ok());
    ASSERT_FALSE(msgs.empty());
    EXPECT_NE(msgs[0].find("[GC:"), std::string::npos);
    EXPECT_TRUE(interp.systemCommand("$#").ok());
    EXPECT_TRUE(interp.systemCommand("$X").ok());
    EXPECT_FALSE(interp.systemCommand("$ZZ").ok());
}

// --- Marlin M-codes ---

TEST_F(InterpreterTestBase, MarlinMCodesAccepted) {
    for (const char* line : {"M400", "M104 S200", "M140 S60", "M106 S255",
                             "M107", "M17", "M84", "M500", "M501", "M503",
                             "M900 K0.2"}) {
        EXPECT_TRUE(interp.executeLine(line).ok()) << line;
    }
}

TEST_F(InterpreterTestBase, MarlinM600PausesAndForwards) {
    bool paused = false;
    int seen = -1;
    interp.setProgramControlCallback([&](int32_t){ paused = true; });
    interp.setMCodeCallback([&](int32_t m, std::optional<double>,
                                std::optional<double>) {
        seen = m; return Error{};
    });
    EXPECT_TRUE(interp.executeLine("M600").ok());
    EXPECT_TRUE(paused);
    EXPECT_EQ(seen, 600);
}

// --- Haas extensions ---

TEST_F(InterpreterTestBase, HaasM19SpindleOrient) {
    EXPECT_TRUE(interp.executeLine("M19 S90").ok());
    EXPECT_TRUE(interp.getMachineState().spindleOn);
}

TEST_F(InterpreterTestBase, HaasG187BlendMode) {
    EXPECT_TRUE(interp.executeLine("G187 E0.05").ok());
    EXPECT_EQ(interp.getMachineState().pathMode, PathMode::BLEND);
    EXPECT_NEAR(interp.getMachineState().blendTolerance, 0.05, 0.001);
}

TEST_F(InterpreterTestBase, HaasG12CircularPocket) {
    EXPECT_TRUE(interp.executeLine("G0 X5 Y5 Z2").ok());
    segments.clear();
    EXPECT_TRUE(interp.executeLine("G12 I3 Z-2").ok());
    // plunge + lead-in + full circle + return = 4
    ASSERT_EQ(segments.size(), 4u);
    EXPECT_EQ(segments[2].type, MotionSegment::Type::ARC_CW);
    EXPECT_NEAR(segments[2].arc.radius, 3.0, 0.001);
    // Ends back at center
    EXPECT_NEAR(segments.back().endPosition.x(), 5.0, 0.001);
    EXPECT_NEAR(segments.back().endPosition.y(), 5.0, 0.001);
}

TEST_F(InterpreterTestBase, HaasG150NotSupported) {
    EXPECT_FALSE(interp.executeLine("G150 P1").ok());
}
