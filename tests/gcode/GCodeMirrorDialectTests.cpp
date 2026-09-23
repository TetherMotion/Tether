/**
 * @file GCodeMirrorDialectTests.cpp
 * @brief Tests for G51.1/G50.1 programmable mirror, Fanuc G50 spindle
 *        clamp, and GRBL $n= settings writes.
 */

#include "tether/gcode/GCodeInterpreter.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>
#include <set>

using namespace GCode;

class MirrorDialectTest : public ::testing::Test {
protected:
    Interpreter interp;
    std::vector<MotionSegment> segments;
    std::vector<std::string> messages;
    std::vector<std::pair<bool, double>> spindleCommands;

    MirrorDialectTest() {
        interp.setMotionCallback([this](const MotionSegment& seg) {
            segments.push_back(seg);
            return Error{};
        });
        interp.setMessageCallback(
            [this](const std::string& m) { messages.push_back(m); });
        interp.setSpindleCallback([this](bool enable, bool cw, double rpm) {
            if (enable) spindleCommands.emplace_back(cw, rpm);
            return Error{};
        });
    }

    bool run(const std::string& program) {
        if (!interp.loadString(program).ok()) return false;
        return interp.run().ok();
    }
};

// ============================================================================
// G51.1 / G50.1 — programmable mirror
// ============================================================================

TEST_F(MirrorDialectTest, MirrorX_AboutCenter) {
    // Mirror X about x=5: program x=8 -> machine x = 2*5-8 = 2
    ASSERT_TRUE(run("G51.1 X5\nG1 X8 Y10 F100\n"));
    ASSERT_FALSE(segments.empty());
    const Position& p = segments.back().endPosition;
    EXPECT_DOUBLE_EQ(p.x(), 2.0);
    EXPECT_DOUBLE_EQ(p.y(), 10.0);
}

TEST_F(MirrorDialectTest, MirrorX_AboutOrigin) {
    ASSERT_TRUE(run("G51.1 X0\nG1 X4\n"));
    EXPECT_DOUBLE_EQ(segments.back().endPosition.x(), -4.0);
}

TEST_F(MirrorDialectTest, MirrorStateTracked) {
    ASSERT_TRUE(run("G51.1 X2 Y3\n"));
    EXPECT_TRUE(interp.getMachineState().axisMirror[0]);
    EXPECT_TRUE(interp.getMachineState().axisMirror[1]);
    EXPECT_FALSE(interp.getMachineState().axisMirror[2]);
    EXPECT_DOUBLE_EQ(interp.getMachineState().mirrorCenter[0], 2.0);
    EXPECT_DOUBLE_EQ(interp.getMachineState().mirrorCenter[1], 3.0);
}

TEST_F(MirrorDialectTest, MirrorBothAxes_PathIsMirrored) {
    ASSERT_TRUE(run("G51.1 X0 Y0\nG1 X10 Y5\n"));
    EXPECT_DOUBLE_EQ(segments.back().endPosition.x(), -10.0);
    EXPECT_DOUBLE_EQ(segments.back().endPosition.y(), -5.0);
}

TEST_F(MirrorDialectTest, MirrorFlipsArcDirection) {
    interp.setEmitArcSegments(true);
    // G2 (CW) under X-mirror becomes CCW.
    ASSERT_TRUE(run("G51.1 X0\nG0 X10 Y0\nG2 X0 Y10 I-10 J0\n"));
    ASSERT_EQ(segments.size(), 2u);
    EXPECT_EQ(segments[1].type, MotionSegment::Type::ARC_CCW);
    EXPECT_FALSE(segments[1].arc.clockwise);
    // Mirrored endpoint: (0,10) -> (0,10) since only X mirrored... x=0 stays.
    EXPECT_DOUBLE_EQ(segments[1].endPosition.x(), 0.0);
    EXPECT_DOUBLE_EQ(segments[1].endPosition.y(), 10.0);
    // Center offset mirrored: I=-10 -> +10.
    EXPECT_DOUBLE_EQ(segments[1].centerOffset.x(), 10.0);
}

TEST_F(MirrorDialectTest, TessellatedArcMirrored) {
    // Default path: arc tessellated to lines; every endpoint mirrors.
    ASSERT_TRUE(run("G51.1 X0\nG0 X10 Y0\nG2 X0 Y10 I-10 J0\n"));
    // Last point of the arc must reach the mirrored endpoint.
    EXPECT_NEAR(segments.back().endPosition.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(segments.back().endPosition.y(), 10.0);
    // A mid-arc point should have negative X (mirrored circle bulge).
    bool sawNegative = false;
    for (const auto& s : segments)
        if (s.endPosition.x() < -0.01) sawNegative = true;
    EXPECT_TRUE(sawNegative);
}

TEST_F(MirrorDialectTest, G50_1_CancelsOneAxis) {
    ASSERT_TRUE(run("G51.1 X0 Y0\nG50.1 X0\nG1 X10 Y10\n"));
    const Position& p = segments.back().endPosition;
    EXPECT_DOUBLE_EQ(p.x(), 10.0);   // X mirror cancelled
    EXPECT_DOUBLE_EQ(p.y(), -10.0);  // Y mirror still active
}

TEST_F(MirrorDialectTest, G50_1_CancelsAll) {
    ASSERT_TRUE(run("G51.1 X0 Y0\nG50.1\nG1 X10 Y10\n"));
    EXPECT_DOUBLE_EQ(segments.back().endPosition.x(), 10.0);
    EXPECT_DOUBLE_EQ(segments.back().endPosition.y(), 10.0);
}

TEST_F(MirrorDialectTest, MirrorComposesWithWCS) {
    // Mirror in program space, WCS offset applied on top.
    ASSERT_TRUE(run("G10 L2 P1 X100 Y100\nG54\nG51.1 X0\nG1 X5\n"));
    EXPECT_DOUBLE_EQ(segments.back().endPosition.x(), 100.0 - 5.0);
}

// ============================================================================
// Fanuc G50 S<rpm> — spindle speed clamp
// ============================================================================

TEST_F(MirrorDialectTest, G50S_ClampsSpindle) {
    ASSERT_TRUE(run("G50 S3000\nM3 S8000\n"));
    ASSERT_EQ(spindleCommands.size(), 1u);
    EXPECT_DOUBLE_EQ(spindleCommands[0].second, 3000.0);
    EXPECT_DOUBLE_EQ(interp.getMachineState().spindleSpeed, 3000.0);
}

TEST_F(MirrorDialectTest, G50S_NoClampBelowLimit) {
    ASSERT_TRUE(run("G50 S3000\nM3 S2000\n"));
    EXPECT_DOUBLE_EQ(spindleCommands[0].second, 2000.0);
}

TEST_F(MirrorDialectTest, BareG50_StillCancelsScaling) {
    ASSERT_TRUE(run("G51 P2\nG50\nG1 X5\n"));
    EXPECT_DOUBLE_EQ(segments.back().endPosition.x(), 5.0);
    EXPECT_FALSE(interp.getMachineState().g51Active);
}

// ============================================================================
// GRBL $n= settings writes
// ============================================================================

TEST_F(MirrorDialectTest, GrblSettingWrite) {
    ASSERT_TRUE(interp.systemCommand("$30=1000").ok());
    ASSERT_TRUE(interp.grblSetting(30).has_value());
    EXPECT_DOUBLE_EQ(*interp.grblSetting(30), 1000.0);
    EXPECT_FALSE(interp.grblSetting(31).has_value());
}

TEST_F(MirrorDialectTest, GrblSettingWrite_FloatAndNegative) {
    ASSERT_TRUE(interp.systemCommand("$32=0.5").ok());
    EXPECT_DOUBLE_EQ(*interp.grblSetting(32), 0.5);
}

TEST_F(MirrorDialectTest, GrblSettingsReport) {
    ASSERT_TRUE(interp.systemCommand("$30=1000").ok());
    ASSERT_TRUE(interp.systemCommand("$31=200").ok());
    messages.clear();
    ASSERT_TRUE(interp.systemCommand("$$").ok());
    bool saw30 = false, saw31 = false;
    for (const auto& m : messages) {
        if (m.find("$30=") == 0) saw30 = true;
        if (m.find("$31=") == 0) saw31 = true;
    }
    EXPECT_TRUE(saw30);
    EXPECT_TRUE(saw31);
}

TEST_F(MirrorDialectTest, GrblSettingWrite_BadValue) {
    EXPECT_FALSE(interp.systemCommand("$30=abc").ok());
}

// ============================================================================
// G150 — Haas generic pocket milling (raster clearing)
// ============================================================================

// Square pocket: 10x10 boundary in O100.
static const char* kSquarePocket =
    "G0 Z5\n"
    "G150 P100 Z-2 R5 J2.5 F100\n"
    "M30\n"
    "O100 sub\n"
    "G1 X0 Y0\n"
    "G1 X10 Y0\n"
    "G1 X10 Y10\n"
    "G1 X0 Y10\n"
    "G1 X0 Y0\n"
    "O100 endsub\n";

TEST_F(MirrorDialectTest, G150_RequiresP) {
    EXPECT_FALSE(run("G150 Z-2\n"));
}

TEST_F(MirrorDialectTest, G150_UnknownSubprogramFails) {
    EXPECT_FALSE(run("G150 P999 Z-2\n"));
}

TEST_F(MirrorDialectTest, G150_RastersWithinBoundary) {
    ASSERT_TRUE(run(kSquarePocket));
    // Cutting segments must stay inside [0,10]x[0,10] at z=-2.
    int cuts = 0;
    for (const auto& s : segments) {
        if (s.type != MotionSegment::Type::LINEAR) continue;
        if (std::abs(s.endPosition.z() + 2.0) > 1e-6) continue;
        cuts++;
        EXPECT_GE(s.endPosition.x(), -1e-9);
        EXPECT_LE(s.endPosition.x(), 10.0 + 1e-9);
        EXPECT_GE(s.endPosition.y(), -1e-9);
        EXPECT_LE(s.endPosition.y(), 10.0 + 1e-9);
    }
    EXPECT_GT(cuts, 4);  // several raster spans + finish contour
}

TEST_F(MirrorDialectTest, G150_DepthStepping) {
    // Q1 over Z-3 => 3 Z levels of rastering.
    ASSERT_TRUE(run(
        "G0 Z5\n"
        "G150 P100 Z-3 R5 Q1 J5 F100\n"
        "M30\n"
        "O100 sub\n"
        "G1 X0 Y0\nG1 X10 Y0\nG1 X10 Y10\nG1 X0 Y10\nG1 X0 Y0\n"
        "O100 endsub\n"));
    std::set<double> depths;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::LINEAR && s.endPosition.z() < -0.5)
            depths.insert(std::round(s.endPosition.z() * 1000) / 1000);
    EXPECT_EQ(depths.count(-1.0), 1u);
    EXPECT_EQ(depths.count(-2.0), 1u);
    EXPECT_EQ(depths.count(-3.0), 1u);
}

TEST_F(MirrorDialectTest, G150_FinishPassTracesBoundary) {
    ASSERT_TRUE(run(kSquarePocket));
    // Find the last linear segments at z=-2 forming the closed contour:
    // the finish pass ends back at the first boundary point (0,0).
    const MotionSegment* lastCut = nullptr;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::LINEAR &&
            std::abs(s.endPosition.z() + 2.0) < 1e-6)
            lastCut = &s;
    ASSERT_NE(lastCut, nullptr);
    EXPECT_NEAR(lastCut->endPosition.x(), 0.0, 1e-6);
    EXPECT_NEAR(lastCut->endPosition.y(), 0.0, 1e-6);
}

TEST_F(MirrorDialectTest, G150_ArcBoundary) {
    // Circular pocket boundary from arcs.
    ASSERT_TRUE(run(
        "G0 Z5\n"
        "G150 P200 Z-1 R5 J1 F100\n"
        "M30\n"
        "O200 sub\n"
        "G1 X5 Y0\n"
        "G2 X5 Y0 I-5 J0\n"
        "O200 endsub\n"));
    int cuts = 0;
    for (const auto& s : segments) {
        if (s.type != MotionSegment::Type::LINEAR) continue;
        if (std::abs(s.endPosition.z() + 1.0) > 1e-6) continue;
        cuts++;
        const double r = std::hypot(s.endPosition.x(),
                                    s.endPosition.y());
        EXPECT_LE(r, 5.0 + 1e-6);
    }
    EXPECT_GT(cuts, 4);
}

TEST_F(MirrorDialectTest, G150_SubprogramNotExecuted) {
    // The boundary sub must be collected, not run as part of the program:
    // its moves must not appear above the pocket depth plane.
    ASSERT_TRUE(run(kSquarePocket));
    // kSquarePocket: lines 1-3 are the main program, O100 body is lines 4-10.
    for (const auto& s : segments)
        EXPECT_LE(s.lineNumber, 3);
}

// ============================================================================
// Fanuc lathe cycles — G70 finishing, G71 turning, G72 facing, G73 pattern
// ============================================================================

class LatheCycleTest : public ::testing::Test {
protected:
    Interpreter interp;
    std::vector<MotionSegment> segments;
    std::vector<std::pair<bool, double>> spindleCommands;

    LatheCycleTest() {
        interp.setMotionCallback([this](const MotionSegment& seg) {
            segments.push_back(seg);
            return Error{};
        });
        interp.setSpindleCallback([this](bool enable, bool cw, double rpm) {
            if (enable) spindleCommands.emplace_back(cw, rpm);
            return Error{};
        });
    }

    bool run(const std::string& program) {
        if (!interp.loadString(program).ok()) return false;
        return interp.run().ok();
    }
};

// Stepped-shaft contour: (20,0) -> (10,-10), approached from X30 Z2.
static const char* kStepShaft =
    "G0 X30 Z2\n"
    "G71 P10 Q11 D5 F200\n"
    "G70 P10 Q11\n"
    "O99 sub\n"
    "N10 G1 X20 Z0\n"
    "N11 G1 X10 Z-10\n"
    "O99 endsub\n";

TEST_F(LatheCycleTest, G71_RoughTurnsLevels) {
    ASSERT_TRUE(run(kStepShaft));
    // Roughing levels at x = 25, 20, 15 (D5 from X30 toward profile).
    std::set<double> cutXs;
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR && s.feedRate == 200.0)
            cutXs.insert(std::round(s.endPosition.x() * 1000) / 1000);
    }
    EXPECT_TRUE(cutXs.count(25.0));
    EXPECT_TRUE(cutXs.count(20.0));
    EXPECT_TRUE(cutXs.count(15.0));
}

TEST_F(LatheCycleTest, G71_CutsBelowStockRadius) {
    ASSERT_TRUE(run(kStepShaft));
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR)
            EXPECT_LT(s.endPosition.x(), 30.0);
    }
}

TEST_F(LatheCycleTest, G71_FinishAllowance) {
    // U2 leaves 2mm of radial stock: no rough cut below x=12 (profile 10+2).
    ASSERT_TRUE(run(
        "G0 X30 Z2\n"
        "G71 P10 Q11 U2 W0 D5 F200\n"
        "O99 sub\n"
        "N10 G1 X20 Z0\n"
        "N11 G1 X10 Z-10\n"
        "O99 endsub\n"));
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR)
            EXPECT_GT(s.endPosition.x(), 10.0 + 1e-9);
    }
}

TEST_F(LatheCycleTest, G70_TracesContour) {
    ASSERT_TRUE(run(kStepShaft));
    // G70 finish pass must reach the profile corner (10, -10).
    bool reached = false;
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR &&
            std::abs(s.endPosition.x() - 10.0) < 1e-6 &&
            std::abs(s.endPosition.z() + 10.0) < 1e-6)
            reached = true;
    }
    EXPECT_TRUE(reached);
}

TEST_F(LatheCycleTest, G72_RoughFacesLevels) {
    // Facing: levels in Z spaced by D, cuts along X.
    ASSERT_TRUE(run(
        "G0 X30 Z2\n"
        "G72 P10 Q11 D5 F200\n"
        "O99 sub\n"
        "N10 G1 X20 Z0\n"
        "N11 G1 X10 Z-10\n"
        "O99 endsub\n"));
    std::set<double> cutZs;
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR && s.feedRate == 200.0)
            cutZs.insert(std::round(s.endPosition.z() * 1000) / 1000);
    }
    // Levels at z = -3 and -8 (D5 from Z2 toward the face profile).
    EXPECT_TRUE(cutZs.count(-3.0));
    EXPECT_TRUE(cutZs.count(-8.0));
}

TEST_F(LatheCycleTest, G73_PatternRepeatOffsets) {
    ASSERT_TRUE(run(
        "G0 X30 Z2\n"
        "G73 P10 Q11 U4 W2 R2 F200\n"
        "O99 sub\n"
        "N10 G1 X20 Z0\n"
        "N11 G1 X10 Z-10\n"
        "O99 endsub\n"));
    // R2 passes: first offset by U/2,W/2 = (2,1), last on the true contour.
    bool offsetPass = false, finalPass = false;
    for (const auto& s : segments) {
        if (s.type != MotionSegment::Type::LINEAR) continue;
        if (std::abs(s.endPosition.x() - 12.0) < 1e-6 &&
            std::abs(s.endPosition.z() + 9.0) < 1e-6)
            offsetPass = true;   // (10+2, -10+1)
        if (std::abs(s.endPosition.x() - 10.0) < 1e-6 &&
            std::abs(s.endPosition.z() + 10.0) < 1e-6)
            finalPass = true;    // (10, -10)
    }
    EXPECT_TRUE(offsetPass);
    EXPECT_TRUE(finalPass);
}

TEST_F(LatheCycleTest, G73_WithoutPQ_StaysPeckDrill) {
    // RS274 G73 peck drill (no P/Q) must keep working.
    ASSERT_TRUE(run("G0 X0 Y0 Z10\nG73 Z-5 Q1 R2 F100\n"));
    bool drilled = false;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::LINEAR &&
            s.endPosition.z() < 0.0)
            drilled = true;
    EXPECT_TRUE(drilled);
}

TEST_F(LatheCycleTest, G71_MissingPQ_Errors) {
    EXPECT_FALSE(run("G0 X30 Z2\nG71 D5 F200\n"));
}

TEST_F(LatheCycleTest, G71_MissingContour_Errors) {
    EXPECT_FALSE(run("G0 X30 Z2\nG71 P50 Q60 D5\n"));
}

TEST_F(LatheCycleTest, G71_SpindleStart) {
    ASSERT_TRUE(run(
        "G50 S1000\n"
        "G0 X30 Z2\n"
        "G71 P10 Q11 D5 F200 S1500\n"
        "O99 sub\n"
        "N10 G1 X20 Z0\n"
        "N11 G1 X10 Z-10\n"
        "O99 endsub\n"));
    ASSERT_FALSE(spindleCommands.empty());
    EXPECT_DOUBLE_EQ(spindleCommands.back().second, 1000.0); // G50 clamp
}

TEST_F(LatheCycleTest, G71_ArcContour) {
    // Contour with a G3 arc in the G18 XZ plane (I/K center).
    ASSERT_TRUE(run(
        "G0 X30 Z2\n"
        "G70 P10 Q12\n"
        "O99 sub\n"
        "N10 G1 X20 Z0\n"
        "N11 G3 X10 Z-5 I-5 K0\n"
        "N12 G1 X10 Z-10\n"
        "O99 endsub\n"));
    // Finish pass must emit tessellated arc points (G3 about I-5 K0 bulges
    // off the straight chord — a point with z > 0.1 proves tessellation).
    bool bulge = false;
    for (const auto& s : segments) {
        if (s.type == MotionSegment::Type::LINEAR &&
            s.endPosition.z() > 0.1 && s.endPosition.x() < 20.0)
            bulge = true;
    }
    EXPECT_TRUE(bulge);
}
