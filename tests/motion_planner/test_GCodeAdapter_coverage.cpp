/**
 * @file test_GCodeAdapter_coverage.cpp
 * @brief Coverage tests for GCodeToMotionConverter — exercises arc processing,
 *        incremental mode, BSPLINE/NURBS parsing, plane selection, feed modes,
 *        modal motion, parseDoubleList, parseBSPLINELine, and edge cases.
 */

#include <vector>
#include <optional>
#include <cmath>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <gtest/gtest.h>

using namespace MotionPlanner;

// ============================================================================
// ParsedGCodeCommand struct helpers
// ============================================================================

TEST(GCodeAdapterCovTest, ParsedGCodeCommand_DefaultHasNoCoordinates) {
    ParsedGCodeCommand cmd;
    EXPECT_EQ(cmd.gCode, -1);
    EXPECT_EQ(cmd.mCode, -1);
    EXPECT_FALSE(cmd.hasAnyCoordinate());
    for (size_t i = 0; i < MAX_MOTION_AXES; ++i) {
        EXPECT_FALSE(cmd.hasCoordinate(i));
    }
}

TEST(GCodeAdapterCovTest, ParsedGCodeCommand_HasCoordinate) {
    ParsedGCodeCommand cmd;
    cmd.coordinates[0] = 10.0;
    EXPECT_TRUE(cmd.hasCoordinate(0));
    EXPECT_TRUE(cmd.hasAnyCoordinate());
    EXPECT_FALSE(cmd.hasCoordinate(1));
}

TEST(GCodeAdapterCovTest, ParsedGCodeCommand_OptionalFields) {
    ParsedGCodeCommand cmd;
    EXPECT_FALSE(cmd.feedRate.has_value());
    EXPECT_FALSE(cmd.dwellTime.has_value());
    EXPECT_FALSE(cmd.arcRadius.has_value());
    EXPECT_FALSE(cmd.spindleSpeed.has_value());
    EXPECT_FALSE(cmd.toolNumber.has_value());
    EXPECT_FALSE(cmd.nLineNumber.has_value());
    EXPECT_FALSE(cmd.isComment);
    EXPECT_FALSE(cmd.isBSPLINE);
    EXPECT_FALSE(cmd.isNURBSCmd);
}

// ============================================================================
// GCodeModalState defaults
// ============================================================================

TEST(GCodeAdapterCovTest, ModalState_Defaults) {
    GCodeModalState s;
    EXPECT_EQ(s.activePlane, Plane::XY);
    EXPECT_EQ(s.distanceMode, DistanceMode::Absolute);
    EXPECT_EQ(s.feedMode, FeedMode::UnitsPerMinute);
    EXPECT_EQ(s.motionMode, MotionMode::Rapid);
    EXPECT_DOUBLE_EQ(s.feedRate, 100.0);
    EXPECT_TRUE(s.isMetric);
}

// ============================================================================
// Constructor and reset
// ============================================================================

TEST(GCodeAdapterCovTest, Constructor_Default) {
    GCodeToMotionConverter converter;
    EXPECT_EQ(converter.state().distanceMode, DistanceMode::Absolute);
}

TEST(GCodeAdapterCovTest, Constructor_WithSourceFile) {
    auto file = std::make_shared<SourceFile>("test.gcode");
    GCodeToMotionConverter converter(file);
    EXPECT_EQ(converter.state().distanceMode, DistanceMode::Absolute);
}

TEST(GCodeAdapterCovTest, Reset) {
    GCodeToMotionConverter converter;
    // Change state
    ParsedGCodeCommand cmd;
    cmd.gCode = 91;  // Incremental
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().distanceMode, DistanceMode::Incremental);
    // Reset
    converter.reset();
    EXPECT_EQ(converter.state().distanceMode, DistanceMode::Absolute);
}

TEST(GCodeAdapterCovTest, SetSourceFile) {
    GCodeToMotionConverter converter;
    auto file = std::make_shared<SourceFile>("new.gcode");
    converter.setSourceFile(file);
    // Just exercise the path
}

// ============================================================================
// Modal state updates from commands
// ============================================================================

TEST(GCodeAdapterCovTest, ModalUpdate_PlaneXZ) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 18;  // G18 XZ plane
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().activePlane, Plane::XZ);
}

TEST(GCodeAdapterCovTest, ModalUpdate_PlaneYZ) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 19;  // G19 YZ plane
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().activePlane, Plane::YZ);
}

TEST(GCodeAdapterCovTest, ModalUpdate_Incremental) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 91;  // G91
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().distanceMode, DistanceMode::Incremental);
}

TEST(GCodeAdapterCovTest, ModalUpdate_InverseTime) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 93;  // G93
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().feedMode, FeedMode::InverseTime);
}

TEST(GCodeAdapterCovTest, ModalUpdate_UnitsPerMinute) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 94;  // G94
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().feedMode, FeedMode::UnitsPerMinute);
}

TEST(GCodeAdapterCovTest, ModalUpdate_Inches) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 20;  // G20 inches
    converter.processCommand(cmd);
    EXPECT_FALSE(converter.state().isMetric);
}

TEST(GCodeAdapterCovTest, ModalUpdate_Metric) {
    GCodeToMotionConverter converter;
    // First set inches
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 20;
    converter.processCommand(cmd1);
    EXPECT_FALSE(converter.state().isMetric);
    // Then back to metric
    ParsedGCodeCommand cmd2;
    cmd2.gCode = 21;
    converter.processCommand(cmd2);
    EXPECT_TRUE(converter.state().isMetric);
}

TEST(GCodeAdapterCovTest, ModalUpdate_FeedRate) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 1;
    cmd.coordinates[0] = 10.0;
    cmd.feedRate = 500.0;
    converter.processCommand(cmd);
    EXPECT_DOUBLE_EQ(converter.state().feedRate, 500.0);
}

TEST(GCodeAdapterCovTest, ModalUpdate_SpindleSpeed) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.spindleSpeed = 12000.0;
    converter.processCommand(cmd);
    EXPECT_DOUBLE_EQ(converter.state().spindleSpeed, 12000.0);
}

TEST(GCodeAdapterCovTest, ModalUpdate_Tool) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.toolNumber = 5;
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().currentTool, 5);
}

TEST(GCodeAdapterCovTest, ModalUpdate_ArcCW) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 2;
    // No coords => no movement but modal state updated
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().motionMode, MotionMode::ArcCW);
}

TEST(GCodeAdapterCovTest, ModalUpdate_ArcCCW) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 3;
    converter.processCommand(cmd);
    EXPECT_EQ(converter.state().motionMode, MotionMode::ArcCCW);
}

// ============================================================================
// Rapid (G0)
// ============================================================================

TEST(GCodeAdapterCovTest, Rapid_ZeroMovement) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 0;
    // No coordinates => no movement
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, Rapid_WithMovement) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 0;
    cmd.coordinates[0] = 50.0;
    cmd.coordinates[1] = 30.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::Rapid);
    EXPECT_NEAR(seg->endPosition[0], 50.0, 1e-10);
    EXPECT_NEAR(seg->endPosition[1], 30.0, 1e-10);
}

// ============================================================================
// Linear (G1)
// ============================================================================

TEST(GCodeAdapterCovTest, Linear_ZeroMovement) {
    GCodeToMotionConverter converter;
    // First move to position
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 0;
    cmd1.coordinates[0] = 10.0;
    converter.processCommand(cmd1);
    // Then G1 to same position
    ParsedGCodeCommand cmd2;
    cmd2.gCode = 1;
    cmd2.coordinates[0] = 10.0;
    auto seg = converter.processCommand(cmd2);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, Linear_WithFeedRate) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 1;
    cmd.coordinates[0] = 20.0;
    cmd.feedRate = 600.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::Linear);
    // Feed rate converted from mm/min to mm/sec
    EXPECT_NEAR(seg->feedrate, 10.0, 1e-10);
}

// ============================================================================
// Incremental mode (G91)
// ============================================================================

TEST(GCodeAdapterCovTest, IncrementalMode) {
    GCodeToMotionConverter converter;
    // Set incremental mode
    ParsedGCodeCommand g91;
    g91.gCode = 91;
    converter.processCommand(g91);
    // Move from 0 by +10,+20
    ParsedGCodeCommand cmd;
    cmd.gCode = 1;
    cmd.coordinates[0] = 10.0;
    cmd.coordinates[1] = 20.0;
    cmd.feedRate = 100.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_NEAR(seg->endPosition[0], 10.0, 1e-10);
    EXPECT_NEAR(seg->endPosition[1], 20.0, 1e-10);
    // Second incremental move
    ParsedGCodeCommand cmd2;
    cmd2.gCode = 1;
    cmd2.coordinates[0] = 5.0;
    auto seg2 = converter.processCommand(cmd2);
    ASSERT_TRUE(seg2.has_value());
    EXPECT_NEAR(seg2->startPosition[0], 10.0, 1e-10);
    EXPECT_NEAR(seg2->endPosition[0], 15.0, 1e-10);
}

// ============================================================================
// Arc CW (G2) — I,J offset format
// ============================================================================

TEST(GCodeAdapterCovTest, ArcCW_IJFormat) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 2;
    cmd.coordinates[0] = 10.0;  // End X
    cmd.coordinates[1] = 0.0;   // End Y
    cmd.arcOffsets[0] = 5.0;    // I (center offset X)
    cmd.arcOffsets[1] = 0.0;    // J (center offset Y)
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCW);
    EXPECT_NEAR(seg->endPosition[0], 10.0, 1e-10);
}

// ============================================================================
// Arc CCW (G3) — I,J offset format
// ============================================================================

TEST(GCodeAdapterCovTest, ArcCCW_IJFormat) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 3;
    cmd.coordinates[0] = 10.0;
    cmd.coordinates[1] = 0.0;
    cmd.arcOffsets[0] = 5.0;
    cmd.arcOffsets[1] = 0.0;
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCCW);
}

// ============================================================================
// Arc — R format
// ============================================================================

TEST(GCodeAdapterCovTest, ArcCW_RFormat) {
    GCodeToMotionConverter converter;
    // Start at (0,0), end at (10,0), radius 5
    ParsedGCodeCommand cmd;
    cmd.gCode = 2;
    cmd.coordinates[0] = 10.0;
    cmd.coordinates[1] = 0.0;
    cmd.arcRadius = 5.0;
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCW);
}

TEST(GCodeAdapterCovTest, ArcCCW_RFormat_NegRadius) {
    GCodeToMotionConverter converter;
    // Negative radius selects the major arc
    ParsedGCodeCommand cmd;
    cmd.gCode = 3;
    cmd.coordinates[0] = 10.0;
    cmd.coordinates[1] = 0.0;
    cmd.arcRadius = -10.0;
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCCW);
}

TEST(GCodeAdapterCovTest, ArcCW_RFormat_InvalidRadius) {
    GCodeToMotionConverter converter;
    // Chord longer than diameter → invalid arc
    ParsedGCodeCommand cmd;
    cmd.gCode = 2;
    cmd.coordinates[0] = 100.0;  // Very far
    cmd.coordinates[1] = 0.0;
    cmd.arcRadius = 1.0;         // Tiny radius
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

// ============================================================================
// Arc in different planes
// ============================================================================

TEST(GCodeAdapterCovTest, Arc_XZPlane) {
    GCodeToMotionConverter converter;
    // Select XZ plane
    ParsedGCodeCommand g18;
    g18.gCode = 18;
    converter.processCommand(g18);
    // Arc in XZ plane
    ParsedGCodeCommand cmd;
    cmd.gCode = 2;
    cmd.coordinates[0] = 10.0;  // X
    cmd.coordinates[2] = 0.0;   // Z
    cmd.arcOffsets[0] = 5.0;    // I
    cmd.arcOffsets[1] = 0.0;    // K → mapped to planeAxis2
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCW);
}

TEST(GCodeAdapterCovTest, Arc_YZPlane) {
    GCodeToMotionConverter converter;
    // Select YZ plane
    ParsedGCodeCommand g19;
    g19.gCode = 19;
    converter.processCommand(g19);
    // Arc in YZ plane
    ParsedGCodeCommand cmd;
    cmd.gCode = 3;
    cmd.coordinates[1] = 10.0;  // Y
    cmd.coordinates[2] = 0.0;   // Z
    cmd.arcOffsets[0] = 5.0;    // J
    cmd.arcOffsets[1] = 0.0;    // K
    cmd.feedRate = 300.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCCW);
}

// ============================================================================
// Dwell (G4)
// ============================================================================

TEST(GCodeAdapterCovTest, Dwell_Valid) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 4;
    cmd.dwellTime = 1.5;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::Dwell);
    EXPECT_DOUBLE_EQ(seg->dwellDuration, 1.5);
}

TEST(GCodeAdapterCovTest, Dwell_ZeroDuration) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 4;
    cmd.dwellTime = 0.0;
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, Dwell_NegativeDuration) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 4;
    cmd.dwellTime = -1.0;
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, Dwell_NoDwellTimeField) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 4;
    // dwellTime not set → value_or(0.0) → 0.0 → returns nullopt
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

// ============================================================================
// Unknown G-code
// ============================================================================

TEST(GCodeAdapterCovTest, UnknownGCode) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.gCode = 99;  // Unknown
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

// ============================================================================
// Comment-only commands
// ============================================================================

TEST(GCodeAdapterCovTest, CommentOnly) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isComment = true;
    cmd.comment = "; This is a comment";
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

// ============================================================================
// Modal motion (coords without G-code)
// ============================================================================

TEST(GCodeAdapterCovTest, ModalMotion_Rapid) {
    GCodeToMotionConverter converter;
    // Set mode to Rapid with G0
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 0;
    cmd1.coordinates[0] = 10.0;
    converter.processCommand(cmd1);
    // Modal motion: just coords, no G-code
    ParsedGCodeCommand cmd2;
    cmd2.coordinates[0] = 20.0;
    auto seg = converter.processCommand(cmd2);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::Rapid);
    EXPECT_NEAR(seg->endPosition[0], 20.0, 1e-10);
}

TEST(GCodeAdapterCovTest, ModalMotion_Linear) {
    GCodeToMotionConverter converter;
    // Set mode to Linear with G1
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 1;
    cmd1.coordinates[0] = 10.0;
    cmd1.feedRate = 600.0;
    converter.processCommand(cmd1);
    // Modal linear motion
    ParsedGCodeCommand cmd2;
    cmd2.coordinates[0] = 20.0;
    auto seg = converter.processCommand(cmd2);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::Linear);
}

TEST(GCodeAdapterCovTest, ModalMotion_ArcCW) {
    GCodeToMotionConverter converter;
    // Set mode to ArcCW
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 2;
    cmd1.coordinates[0] = 10.0;
    cmd1.coordinates[1] = 0.0;
    cmd1.arcOffsets[0] = 5.0;
    cmd1.arcOffsets[1] = 0.0;
    cmd1.feedRate = 300.0;
    converter.processCommand(cmd1);
    // Modal arc motion
    ParsedGCodeCommand cmd2;
    cmd2.coordinates[0] = 20.0;
    cmd2.coordinates[1] = 0.0;
    cmd2.arcOffsets[0] = 5.0;
    cmd2.arcOffsets[1] = 0.0;
    auto seg = converter.processCommand(cmd2);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCW);
}

TEST(GCodeAdapterCovTest, ModalMotion_ArcCCW) {
    GCodeToMotionConverter converter;
    // Set mode to ArcCCW
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 3;
    cmd1.coordinates[0] = 10.0;
    cmd1.coordinates[1] = 0.0;
    cmd1.arcOffsets[0] = 5.0;
    cmd1.arcOffsets[1] = 0.0;
    cmd1.feedRate = 300.0;
    converter.processCommand(cmd1);
    // Modal arc motion
    ParsedGCodeCommand cmd2;
    cmd2.coordinates[0] = 20.0;
    cmd2.coordinates[1] = 0.0;
    cmd2.arcOffsets[0] = 5.0;
    cmd2.arcOffsets[1] = 0.0;
    auto seg = converter.processCommand(cmd2);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::ArcCCW);
}

// ============================================================================
// No motion and no coord: returns nullopt
// ============================================================================

TEST(GCodeAdapterCovTest, EmptyCommand) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

// ============================================================================
// BSPLINE processing
// ============================================================================

TEST(GCodeAdapterCovTest, BSPLINE_Valid) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    // 3 poles in 2D: (0,0), (5,10), (10,0)
    cmd.bsplinePoles = {0.0, 0.0, 5.0, 10.0, 10.0, 0.0};
    // Clamped knot vector: n+p+1 = 3+2+1 = 6
    cmd.bsplineKnots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    cmd.feedRate = 600.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::NURBS);
}

TEST(GCodeAdapterCovTest, BSPLINE_EmptyPoles) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    // Empty poles → invalid
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, BSPLINE_ZeroDegree) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 0;
    cmd.bsplinePoles = {0, 0, 10, 0};
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, BSPLINE_AutoKnotVector) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    // 4 poles in 2D → knots auto-generated
    cmd.bsplinePoles = {0.0, 0.0, 3.0, 5.0, 7.0, 5.0, 10.0, 0.0};
    // No knots → auto-generated
    cmd.feedRate = 600.0;
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
}

TEST(GCodeAdapterCovTest, BSPLINE_AutoWeights) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    cmd.bsplinePoles = {0.0, 0.0, 5.0, 10.0, 10.0, 0.0};
    cmd.bsplineKnots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    // No weights → auto-assigned as 1.0
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
}

TEST(GCodeAdapterCovTest, BSPLINE_InvalidKnotCount) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    cmd.bsplinePoles = {0.0, 0.0, 5.0, 10.0, 10.0, 0.0};
    // Wrong number of knots (should be 6, providing 4)
    cmd.bsplineKnots = {0.0, 0.0, 1.0, 1.0};
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, BSPLINE_InvalidPoleDivisibility) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    // 5 values — not divisible by MAX_MOTION_AXES or 2
    cmd.bsplinePoles = {0.0, 0.0, 5.0, 10.0, 10.0};
    auto seg = converter.processCommand(cmd);
    EXPECT_FALSE(seg.has_value());
}

TEST(GCodeAdapterCovTest, BSPLINE_WithWeights) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isBSPLINE = true;
    cmd.bsplineDegree = 2;
    cmd.bsplinePoles = {0.0, 0.0, 5.0, 10.0, 10.0, 0.0};
    cmd.bsplineKnots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    cmd.bsplineWeights = {1.0, 2.0, 1.0};
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
}

// ============================================================================
// NURBS command processing
// ============================================================================

TEST(GCodeAdapterCovTest, NURBS_Valid) {
    GCodeToMotionConverter converter;
    ParsedGCodeCommand cmd;
    cmd.isNURBSCmd = true;
    cmd.bsplineDegree = 2;
    cmd.bsplinePoles = {0.0, 0.0, 5.0, 10.0, 10.0, 0.0};
    cmd.bsplineKnots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    cmd.bsplineWeights = {1.0, 1.0, 1.0};
    auto seg = converter.processCommand(cmd);
    ASSERT_TRUE(seg.has_value());
    EXPECT_EQ(seg->type, MotionSegmentType::NURBS);
}

// ============================================================================
// parseDoubleList and parseBSPLINELine are private — tested indirectly
// via BSPLINE command processing above
// ============================================================================

// ============================================================================
// processCommands batch
// ============================================================================

TEST(GCodeAdapterCovTest, ProcessCommands_Batch) {
    GCodeToMotionConverter converter;
    std::vector<ParsedGCodeCommand> commands;

    ParsedGCodeCommand cmd1;
    cmd1.lineNumber = 1;
    cmd1.gCode = 0;
    cmd1.coordinates[0] = 10.0;
    commands.push_back(cmd1);

    ParsedGCodeCommand cmd2;
    cmd2.lineNumber = 2;
    cmd2.gCode = 1;
    cmd2.coordinates[1] = 20.0;
    cmd2.feedRate = 600.0;
    commands.push_back(cmd2);

    ParsedGCodeCommand cmd3;
    cmd3.lineNumber = 3;
    cmd3.gCode = 4;
    cmd3.dwellTime = 1.0;
    commands.push_back(cmd3);

    auto segments = converter.processCommands(commands);
    EXPECT_EQ(segments.size(), 3u);
}

TEST(GCodeAdapterCovTest, ProcessCommands_SkipsNonMotion) {
    GCodeToMotionConverter converter;
    std::vector<ParsedGCodeCommand> commands;

    ParsedGCodeCommand cmd1;
    cmd1.gCode = 17;  // Plane select, no motion
    commands.push_back(cmd1);

    ParsedGCodeCommand cmd2;
    cmd2.gCode = 1;
    cmd2.coordinates[0] = 10.0;
    cmd2.feedRate = 100.0;
    commands.push_back(cmd2);

    auto segments = converter.processCommands(commands);
    EXPECT_EQ(segments.size(), 1u);
}

// ============================================================================
// Enum value checks
// ============================================================================

TEST(GCodeAdapterCovTest, PlaneEnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(Plane::XY), 17);
    EXPECT_EQ(static_cast<uint8_t>(Plane::XZ), 18);
    EXPECT_EQ(static_cast<uint8_t>(Plane::YZ), 19);
}

TEST(GCodeAdapterCovTest, DistanceModeValues) {
    EXPECT_EQ(static_cast<uint8_t>(DistanceMode::Absolute), 90);
    EXPECT_EQ(static_cast<uint8_t>(DistanceMode::Incremental), 91);
}

TEST(GCodeAdapterCovTest, FeedModeValues) {
    EXPECT_EQ(static_cast<uint8_t>(FeedMode::UnitsPerMinute), 94);
    EXPECT_EQ(static_cast<uint8_t>(FeedMode::InverseTime), 93);
}

TEST(GCodeAdapterCovTest, MotionModeValues) {
    EXPECT_EQ(static_cast<uint8_t>(MotionMode::Rapid), 0);
    EXPECT_EQ(static_cast<uint8_t>(MotionMode::Linear), 1);
    EXPECT_EQ(static_cast<uint8_t>(MotionMode::ArcCW), 2);
    EXPECT_EQ(static_cast<uint8_t>(MotionMode::ArcCCW), 3);
}
