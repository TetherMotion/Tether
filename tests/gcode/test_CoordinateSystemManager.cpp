/**
 * @file test_CoordinateSystemManager.cpp
 * @brief Tests for the CoordinateSystemManager: G52, G68/G69, G51/G50,
 *        G92, G10 L2/L20, WCS selection, and the composed CoordinateTransform.
 */

#include "tether/gcode/motion/GCodeCoordinates.hpp"
#include "tether/gcode/GCodeVariables.hpp"
#include "tether/gcode/GCodeParser.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace GCode;

// ============================================================================
// Helper: set a word in a Block.
// ============================================================================

static void setWord(Block& b, WordLetter letter, double value) {
    const size_t idx = static_cast<size_t>(letter);
    if (idx < b.words.size()) {
        b.words[idx].letter = letter;
        b.words[idx].value = value;
        b.words[idx].present = true;
    }
}

// ============================================================================
// Helper: create a MachineState with default plane (XY) and absolute mode.
// ============================================================================

static MachineState makeState() {
    MachineState s;
    s.plane = Plane::XY;
    s.distanceMode = DistanceMode::ABSOLUTE;
    s.units = Units::MM;
    s.scaleFactors = Position::ones();
    return s;
}

// ============================================================================
// WCS selection (G54-G59.3)
// ============================================================================

TEST(CoordinateSystemManager, SelectG54) {
    CoordinateSystemManager csm;
    MachineState s = makeState();
    Error e = csm.selectWCS(54.0);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_EQ(csm.getActiveWCS().number, 1);
}

TEST(CoordinateSystemManager, SelectG59_3) {
    CoordinateSystemManager csm;
    Error e = csm.selectWCS(59.3);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_EQ(csm.getActiveWCS().number, 9);
}

TEST(CoordinateSystemManager, SelectInvalidWCS) {
    CoordinateSystemManager csm;
    Error e = csm.selectWCS(100.0);
    EXPECT_NE(e.code, ErrorCode::OK);
}

// ============================================================================
// G52 local offset
// ============================================================================

TEST(CoordinateSystemManager, G52SetsOffset) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::X, 5.0);
    setWord(b, WordLetter::Y, 10.0);
    setWord(b, WordLetter::Z, 15.0);

    Error e = csm.processG52(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_NEAR(s.g52Offset.x(), 5.0, 0.001);
    EXPECT_NEAR(s.g52Offset.y(), 10.0, 0.001);
    EXPECT_NEAR(s.g52Offset.z(), 15.0, 0.001);
}

TEST(CoordinateSystemManager, G52NoAxesResets) {
    CoordinateSystemManager csm;
    MachineState s = makeState();
    s.g52Offset.x() = 99.0;

    Block b;
    // No axis words.

    Error e = csm.processG52(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_NEAR(s.g52Offset.x(), 0.0, 0.001);
}

TEST(CoordinateSystemManager, G52ClearExplicit) {
    CoordinateSystemManager csm;
    MachineState s = makeState();
    s.g52Offset.x() = 99.0;

    Error e = csm.clearG52(s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_NEAR(s.g52Offset.x(), 0.0, 0.001);
}

// ============================================================================
// G68/G69 coordinate rotation
// ============================================================================

TEST(CoordinateSystemManager, G68_2DRotation) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::X, 5.0);
    setWord(b, WordLetter::Y, 10.0);
    setWord(b, WordLetter::R, 45.0);

    Error e = csm.processG68(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_TRUE(s.g68Active);
    EXPECT_EQ(s.g68Mode, 0);
    EXPECT_NEAR(s.coordRotation, 45.0, 0.001);
}

TEST(CoordinateSystemManager, G68_3DEuler) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::A, 10.0);
    setWord(b, WordLetter::B, 20.0);
    setWord(b, WordLetter::C, 30.0);

    Error e = csm.processG68(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_TRUE(s.g68Active);
    EXPECT_EQ(s.g68Mode, 1);
    EXPECT_NEAR(s.g68Euler[0], 10.0, 0.001);
    EXPECT_NEAR(s.g68Euler[1], 20.0, 0.001);
    EXPECT_NEAR(s.g68Euler[2], 30.0, 0.001);
}

TEST(CoordinateSystemManager, G68_3DAxisAngle) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::I, 0.0);
    setWord(b, WordLetter::J, 0.0);
    setWord(b, WordLetter::K, 1.0);
    setWord(b, WordLetter::R, 90.0);

    Error e = csm.processG68(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_TRUE(s.g68Active);
    EXPECT_EQ(s.g68Mode, 2);
    EXPECT_NEAR(s.g68Axis[2], 1.0, 0.001);
    EXPECT_NEAR(s.g68AxisAngle, 90.0, 0.001);
}

TEST(CoordinateSystemManager, G69CancelsRotation) {
    CoordinateSystemManager csm;
    MachineState s = makeState();
    s.g68Active = true;
    s.coordRotation = 45.0;

    Error e = csm.processG69(s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_FALSE(s.g68Active);
    EXPECT_NEAR(s.coordRotation, 0.0, 0.001);
}

// ============================================================================
// G51/G50 scaling
// ============================================================================

TEST(CoordinateSystemManager, G51UniformScaling) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::P, 2.0);

    Error e = csm.processG51(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_TRUE(s.g51Active);
    EXPECT_NEAR(s.scaleFactors.x(), 2.0, 0.001);
    EXPECT_NEAR(s.scaleFactors.y(), 2.0, 0.001);
    EXPECT_NEAR(s.scaleFactors.z(), 2.0, 0.001);
}

TEST(CoordinateSystemManager, G51PerAxisScaling) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::X, 2.0);
    setWord(b, WordLetter::Y, 0.5);
    setWord(b, WordLetter::Z, 1.0);

    Error e = csm.processG51(b, s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_TRUE(s.g51Active);
    EXPECT_NEAR(s.scaleFactors.x(), 2.0, 0.001);
    EXPECT_NEAR(s.scaleFactors.y(), 0.5, 0.001);
    EXPECT_NEAR(s.scaleFactors.z(), 1.0, 0.001);
}

TEST(CoordinateSystemManager, G50CancelsScaling) {
    CoordinateSystemManager csm;
    MachineState s = makeState();
    s.g51Active = true;
    s.scaleFactors.x() = 2.0;

    Error e = csm.processG50(s);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_FALSE(s.g51Active);
    EXPECT_NEAR(s.scaleFactors.x(), 1.0, 0.001);
}

// ============================================================================
// G10 L2 / L20 (set WCS offsets)
// ============================================================================

TEST(CoordinateSystemManager, G10L2SetsWCSOffset) {
    CoordinateSystemManager csm;
    VariableSystem vars;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::X, 10.0);
    setWord(b, WordLetter::Y, 20.0);
    setWord(b, WordLetter::Z, 30.0);

    Error e = csm.processG10L2(1, b, vars);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_NEAR(csm.getWCS(1).offset.x(), 10.0, 0.001);
    EXPECT_NEAR(csm.getWCS(1).offset.y(), 20.0, 0.001);
    EXPECT_NEAR(csm.getWCS(1).offset.z(), 30.0, 0.001);
}

TEST(CoordinateSystemManager, G10L20SetsFromCurrentPos) {
    CoordinateSystemManager csm;
    VariableSystem vars;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::X, 5.0);
    // Current machine pos is (15, 0, 0); WCS offset = 15 - 5 = 10.
    Position machinePos;
    machinePos.x() = 15.0;

    Error e = csm.processG10L20(1, b, machinePos, vars);
    EXPECT_EQ(e.code, ErrorCode::OK);
    EXPECT_NEAR(csm.getWCS(1).offset.x(), 10.0, 0.001);
}

// ============================================================================
// Composed transform: toMachineCoords / toProgramCoords
// ============================================================================

TEST(CoordinateSystemManager, TransformWithWCSOnly) {
    CoordinateSystemManager csm;
    VariableSystem vars;
    MachineState s = makeState();

    // Set WCS 1 offset to (10, 20, 30).
    Block b;
    setWord(b, WordLetter::X, 10.0);
    setWord(b, WordLetter::Y, 20.0);
    setWord(b, WordLetter::Z, 30.0);
    csm.processG10L2(1, b, vars);
    csm.selectWCS(1);
    csm.syncTransform(s);

    Position program;
    program.x() = 1.0; program.y() = 2.0; program.z() = 3.0;
    Position machine = csm.toMachineCoords(program);
    EXPECT_NEAR(machine.x(), 11.0, 0.001);
    EXPECT_NEAR(machine.y(), 22.0, 0.001);
    EXPECT_NEAR(machine.z(), 33.0, 0.001);

    Position back = csm.toProgramCoords(machine);
    EXPECT_NEAR(back.x(), 1.0, 0.001);
    EXPECT_NEAR(back.y(), 2.0, 0.001);
    EXPECT_NEAR(back.z(), 3.0, 0.001);
}

TEST(CoordinateSystemManager, TransformWithG52AndWCS) {
    CoordinateSystemManager csm;
    VariableSystem vars;
    MachineState s = makeState();

    // WCS 1 offset = (10, 0, 0)
    Block b;
    setWord(b, WordLetter::X, 10.0);
    csm.processG10L2(1, b, vars);
    csm.selectWCS(1);

    // G52 offset = (1, 2, 3)
    Block b52;
    setWord(b52, WordLetter::X, 1.0);
    setWord(b52, WordLetter::Y, 2.0);
    setWord(b52, WordLetter::Z, 3.0);
    csm.processG52(b52, s);

    Position program;
    program.x() = 5.0; program.y() = 5.0; program.z() = 5.0;
    Position machine = csm.toMachineCoords(program);
    // machine = WCS(10) + G52(1) + program(5) = 16
    EXPECT_NEAR(machine.x(), 16.0, 0.001);
    EXPECT_NEAR(machine.y(), 7.0, 0.001);
    EXPECT_NEAR(machine.z(), 8.0, 0.001);
}

TEST(CoordinateSystemManager, TransformWithG68Rotation2D) {
    CoordinateSystemManager csm;
    VariableSystem vars;
    MachineState s = makeState();

    // G68 2D rotation 90° about origin.
    Block b;
    setWord(b, WordLetter::R, 90.0);
    csm.processG68(b, s);

    // (1, 0, 0) rotated 90° about Z -> (0, 1, 0)
    Position program;
    program.x() = 1.0; program.y() = 0.0; program.z() = 0.0;
    Position machine = csm.toMachineCoords(program);
    EXPECT_NEAR(machine.x(), 0.0, 0.001);
    EXPECT_NEAR(machine.y(), 1.0, 0.001);
    EXPECT_NEAR(machine.z(), 0.0, 0.001);
}

TEST(CoordinateSystemManager, TransformWithG51Scaling) {
    CoordinateSystemManager csm;
    MachineState s = makeState();

    Block b;
    setWord(b, WordLetter::X, 2.0);
    setWord(b, WordLetter::Y, 3.0);
    setWord(b, WordLetter::Z, 4.0);
    csm.processG51(b, s);

    Position program;
    program.x() = 1.0; program.y() = 1.0; program.z() = 1.0;
    Position machine = csm.toMachineCoords(program);
    EXPECT_NEAR(machine.x(), 2.0, 0.001);
    EXPECT_NEAR(machine.y(), 3.0, 0.001);
    EXPECT_NEAR(machine.z(), 4.0, 0.001);
}

TEST(CoordinateSystemManager, TransformFullCompositionRoundTrip) {
    CoordinateSystemManager csm;
    VariableSystem vars;
    MachineState s = makeState();

    // WCS 1 offset = (10, 20, 30)
    Block b1;
    setWord(b1, WordLetter::X, 10.0);
    setWord(b1, WordLetter::Y, 20.0);
    setWord(b1, WordLetter::Z, 30.0);
    csm.processG10L2(1, b1, vars);
    csm.selectWCS(1);

    // G52 offset = (1, 2, 3)
    Block b52;
    setWord(b52, WordLetter::X, 1.0);
    setWord(b52, WordLetter::Y, 2.0);
    setWord(b52, WordLetter::Z, 3.0);
    csm.processG52(b52, s);

    // G68 2D rotation 45° about pivot (5, 5)
    Block b68;
    setWord(b68, WordLetter::X, 5.0);
    setWord(b68, WordLetter::Y, 5.0);
    setWord(b68, WordLetter::R, 45.0);
    csm.processG68(b68, s);

    // G51 scaling (2, 2, 1)
    Block b51;
    setWord(b51, WordLetter::X, 2.0);
    setWord(b51, WordLetter::Y, 2.0);
    setWord(b51, WordLetter::Z, 1.0);
    csm.processG51(b51, s);

    Position program;
    program.x() = 10.0; program.y() = 20.0; program.z() = 30.0;
    Position machine = csm.toMachineCoords(program);
    Position back = csm.toProgramCoords(machine);
    EXPECT_NEAR(back.x(), 10.0, 0.001);
    EXPECT_NEAR(back.y(), 20.0, 0.001);
    EXPECT_NEAR(back.z(), 30.0, 0.001);
}

// ============================================================================
// Modal group classification
// ============================================================================

TEST(CoordinateSystemManager, ModalGroupG52) {
    EXPECT_EQ(getModalGroup(520), ModalGroup::LOCAL_OFFSET);
}

TEST(CoordinateSystemManager, ModalGroupG68G69) {
    EXPECT_EQ(getModalGroup(680), ModalGroup::COORD_ROTATION);
    EXPECT_EQ(getModalGroup(690), ModalGroup::COORD_ROTATION);
}

TEST(CoordinateSystemManager, ModalGroupG51G50) {
    EXPECT_EQ(getModalGroup(510), ModalGroup::SCALING);
    EXPECT_EQ(getModalGroup(500), ModalGroup::SCALING);
}
