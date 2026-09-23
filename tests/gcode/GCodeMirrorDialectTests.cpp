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
    std::vector<std::string> messages;
    std::vector<std::pair<bool, double>> spindleCommands;

    LatheCycleTest() {
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

// ============================================================================
// G33 threading / G33.1 rigid tapping
// ============================================================================

TEST_F(LatheCycleTest, G33_ThreadingEmitsSyncSegment) {
    ASSERT_TRUE(run("M3 S500\nG0 X20 Z5\nG33 Z-15 K2\n"));
    bool found = false;
    for (const auto& s : segments) {
        if (s.type != MotionSegment::Type::THREADING) continue;
        found = true;
        EXPECT_DOUBLE_EQ(s.pitch, 2.0);
        EXPECT_DOUBLE_EQ(s.feedRate, 1000.0);  // pitch * rpm
        EXPECT_DOUBLE_EQ(s.endPosition.z(), -15.0);
    }
    EXPECT_TRUE(found);
}

TEST_F(LatheCycleTest, G33_TaperedThreadUsesIPitch) {
    ASSERT_TRUE(run("M3 S600\nG0 X20 Z5\nG33 X15 Z-15 I1.5\n"));
    bool found = false;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::THREADING) {
            found = true;
            EXPECT_DOUBLE_EQ(s.pitch, 1.5);
        }
    EXPECT_TRUE(found);
}

TEST_F(LatheCycleTest, G33_NoSpindle_Errors) {
    EXPECT_FALSE(run("G0 X20 Z5\nG33 Z-15 K2\n"));
}

TEST_F(LatheCycleTest, G33_NoPitch_Errors) {
    EXPECT_FALSE(run("M3 S500\nG0 X20 Z5\nG33 Z-15\n"));
}

TEST_F(LatheCycleTest, G33_1_RigidTapInAndOut) {
    ASSERT_TRUE(run("M3 S300\nG0 X0 Z5\nG33.1 Z-10 K1.25\n"));
    std::vector<const MotionSegment*> taps;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::THREADING) taps.push_back(&s);
    ASSERT_EQ(taps.size(), 2u);
    EXPECT_DOUBLE_EQ(taps[0]->endPosition.z(), -10.0);
    EXPECT_DOUBLE_EQ(taps[0]->pitch, 1.25);
    EXPECT_DOUBLE_EQ(taps[1]->endPosition.z(), 5.0);
    EXPECT_DOUBLE_EQ(taps[1]->pitch, -1.25);  // reverse stroke
    EXPECT_DOUBLE_EQ(taps[0]->feedRate, 375.0);
}

TEST_F(LatheCycleTest, G33_1_MissingK_Errors) {
    EXPECT_FALSE(run("M3 S300\nG0 Z5\nG33.1 Z-10\n"));
}

// ============================================================================
// Feature flags — G30_PROBE, G80_CANCEL_LEVELING, G76_LATHE_THREADING
// ============================================================================

TEST_F(LatheCycleTest, FeatureG30Probe_ProbesAndReports) {
    interp.enableFeature(Feature::G30_PROBE);
    ASSERT_TRUE(run("G0 X0 Y0 Z10\nG30 X5 Y7 Z-2 F100\n"));
    bool probed = false;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::PROBE) probed = true;
    EXPECT_TRUE(probed);
    bool reported = false;
    for (const auto& m : messages)
        if (m.find("Bed X:") != std::string::npos) reported = true;
    EXPECT_TRUE(reported);
}

TEST_F(LatheCycleTest, FeatureG30Probe_OffByDefault) {
    // Without the feature, G30 rapids to the stored reference point.
    ASSERT_TRUE(run("G0 X1 Y2 Z3\nG30\n"));
    for (const auto& s : segments)
        EXPECT_NE(s.type, MotionSegment::Type::PROBE);
}

TEST_F(LatheCycleTest, FeatureG80CancelLeveling) {
    interp.enableFeature(Feature::G80_CANCEL_LEVELING);
    ASSERT_TRUE(run("M420 S1\nG80\n"));
    EXPECT_FALSE(interp.marlinState().bedLevelingEnabled);
}

TEST_F(LatheCycleTest, FeatureG80_StillCancelsCannedCycle) {
    interp.enableFeature(Feature::G80_CANCEL_LEVELING);
    // G81 then G80: a following bare XY line must not repeat the cycle.
    ASSERT_TRUE(run("G0 Z10\nG81 X0 Y0 Z-5 R2 F100\nG80\nX10 Y10\n"));
    int drills = 0;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::LINEAR &&
            s.endPosition.z() < 0.0)
            ++drills;
    EXPECT_EQ(drills, 1);  // only the G81 hole itself
}

TEST_F(LatheCycleTest, FeatureG76_ThreadingPasses) {
    interp.enableFeature(Feature::G76_LATHE_THREADING);
    ASSERT_TRUE(run("M3 S500\nG0 X30 Z2\nG76 X28 Z-10 D1 F2\n"));
    std::vector<const MotionSegment*> cuts;
    for (const auto& s : segments)
        if (s.type == MotionSegment::Type::THREADING) cuts.push_back(&s);
    // Constant-area passes: d = sqrt(i) until total 2mm -> 4 passes.
    ASSERT_EQ(cuts.size(), 4u);
    EXPECT_NEAR(cuts[0]->endPosition.x(), 29.0, 1e-6);   // 30 - 1*sqrt(1)
    EXPECT_NEAR(cuts[3]->endPosition.x(), 28.0, 1e-6);   // full depth
    for (const auto* s : cuts) {
        EXPECT_DOUBLE_EQ(s->pitch, 2.0);
        EXPECT_DOUBLE_EQ(s->feedRate, 1000.0);
        EXPECT_DOUBLE_EQ(s->endPosition.z(), -10.0);
    }
}

TEST_F(LatheCycleTest, FeatureG76_OffByDefault_StaysFineBore) {
    // Without the feature G76 is the RS274 fine-boring canned cycle.
    ASSERT_TRUE(run("G0 X0 Y0 Z10\nG76 Z-5 R2 Q0.5 F100\n"));
    for (const auto& s : segments)
        EXPECT_NE(s.type, MotionSegment::Type::THREADING);
}

TEST_F(LatheCycleTest, FeatureG76_NoSpindle_Errors) {
    interp.enableFeature(Feature::G76_LATHE_THREADING);
    EXPECT_FALSE(run("G0 X30 Z2\nG76 X28 Z-10 D1 F2\n"));
}

// ============================================================================
// G41/G42 cutter compensation — geometric offset
// ============================================================================

class CutterCompTest : public ::testing::Test {
protected:
    Interpreter interp;
    std::vector<MotionSegment> segments;

    CutterCompTest() {
        interp.setMotionCallback([this](const MotionSegment& s) {
            segments.push_back(s);
            return Error{};
        });
    }

    bool run(const std::string& program) {
        if (!interp.loadString(program).ok()) return false;
        return interp.run().ok();
    }
};

// CCW square 10x10 with G41.1 D4 (r=2, left comp = inside offsets).
static const char* kCompSquare =
    "G41.1 D4\n"
    "G1 X10 F100\n"
    "G1 Y10\n"
    "G1 X0\n"
    "G1 Y0\n"
    "G40\n";

TEST_F(CutterCompTest, G41_OffsetsInsideCCWSquare) {
    ASSERT_TRUE(run(kCompSquare));
    // Inside offsets: nothing leaves the 10x10 stock, and the inside
    // corner joins land on the shrunken square (8,2) and (2,8).
    for (const auto& s : segments) {
        EXPECT_GE(s.endPosition.x(), -1e-6);
        EXPECT_LE(s.endPosition.x(), 10.0 + 1e-6);
        EXPECT_GE(s.endPosition.y(), -1e-6);
        EXPECT_LE(s.endPosition.y(), 10.0 + 1e-6);
    }
    auto near = [&](double x, double y) {
        for (const auto& s : segments)
            if (std::abs(s.endPosition.x() - x) < 1e-6 &&
                std::abs(s.endPosition.y() - y) < 1e-6)
                return true;
        return false;
    };
    EXPECT_TRUE(near(8.0, 2.0));
    EXPECT_TRUE(near(2.0, 8.0));
}

TEST_F(CutterCompTest, G41_LeadInMove) {
    ASSERT_TRUE(run(kCompSquare));
    // First emitted move is the lead-in to the offset start (0,2).
    ASSERT_FALSE(segments.empty());
    EXPECT_NEAR(segments.front().endPosition.x(), 0.0, 1e-6);
    EXPECT_NEAR(segments.front().endPosition.y(), 2.0, 1e-6);
}

TEST_F(CutterCompTest, G42_OffsetsOutsideCCWSquare) {
    // Right comp on a CCW square = outside offsets (radius grows).
    ASSERT_TRUE(run(
        "G42.1 D4\n"
        "G1 X10 F100\n"
        "G1 Y10\n"
        "G1 X0\n"
        "G1 Y0\n"
        "G40\n"));
    bool outside = false;
    for (const auto& s : segments)
        if (s.endPosition.y() < -1.9 || s.endPosition.x() > 11.9)
            outside = true;
    EXPECT_TRUE(outside);
}

TEST_F(CutterCompTest, G40_LeadOut) {
    ASSERT_TRUE(run(kCompSquare));
    // G40 emits a lead-out back to the programmed point (0,0).
    EXPECT_NEAR(segments.back().endPosition.x(), 0.0, 1e-6);
    EXPECT_NEAR(segments.back().endPosition.y(), 0.0, 1e-6);
}

TEST_F(CutterCompTest, G41_ConvexCornerRollsArc) {
    // CW square with left comp = outside corners -> arc roll at vertices.
    ASSERT_TRUE(run(
        "G41.1 D4\n"
        "G1 X10 F100\n"
        "G1 Y-10\n"
        "G1 X0\n"
        "G1 Y0\n"
        "G40\n"));
    // At the (10,0) outside corner the tool rolls around: some emitted
    // point should have both x>10 and y>0 (arc bulge past the corner).
    bool bulge = false;
    for (const auto& s : segments)
        if (s.endPosition.x() > 10.0 + 1e-6 &&
            s.endPosition.y() > 1e-6)
            bulge = true;
    EXPECT_TRUE(bulge);
}

TEST_F(CutterCompTest, G41_ArcCompensated) {
    // CCW arc about (0,0) r=10 under left comp -> inside, offset radius 8.
    ASSERT_TRUE(run(
        "G0 X10 Y0\n"
        "G41.1 D4\n"
        "G3 X-10 Y0 I-10 J0 F100\n"
        "G40\n"));
    int count = 0;
    for (const auto& s : segments) {
        if (s.type != MotionSegment::Type::LINEAR) continue;
        const double r = std::hypot(s.endPosition.x(), s.endPosition.y());
        if (r > 9.0) continue;  // lead-in / lead-out moves
        ++count;
        EXPECT_NEAR(r, 8.0, 0.15);  // tessellation chord sag tolerance
    }
    EXPECT_GT(count, 4);
}

TEST_F(CutterCompTest, CompOff_NoOffset) {
    ASSERT_TRUE(run("G1 X5 F100\nG1 Y5\n"));
    EXPECT_DOUBLE_EQ(segments.back().endPosition.x(), 5.0);
    EXPECT_DOUBLE_EQ(segments.back().endPosition.y(), 5.0);
}

TEST_F(LatheCycleTest, FeatureG70G71Units) {
    interp.enableFeature(Feature::G70_G71_UNITS);
    // G70 selects inch: a following G1 X1 moves 25.4mm in machine coords.
    ASSERT_TRUE(run("G70\nG1 X1 F100\n"));
    EXPECT_NEAR(segments.back().endPosition.x(), 25.4, 1e-6);
    // G71 selects metric.
    ASSERT_TRUE(interp.loadString("G71\nG1 X1 F100\n").ok());
    segments.clear();
    ASSERT_TRUE(interp.run().ok());
    EXPECT_NEAR(segments.back().endPosition.x(), 1.0, 1e-6);
}
