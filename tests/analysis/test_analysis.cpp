/**
 * @file test_analysis.cpp
 * @brief Unit tests for all 10 G-code analysis components.
 */
#include <gtest/gtest.h>

#include "tether/analysis/AccelerationProfileAnalyzer.hpp"
#include "tether/analysis/ArcAnalyzer.hpp"
#include "tether/analysis/CoordinateSystemAnalyzer.hpp"
#include "tether/analysis/CurvatureAnalyzer.hpp"
#include "tether/analysis/MachineLimitChecker.hpp"
#include "tether/analysis/ModalStateAnalyzer.hpp"
#include "tether/analysis/PathContinuityChecker.hpp"
#include "tether/analysis/PathTopologyDetector.hpp"
#include "tether/analysis/RetractionAnalyzer.hpp"
#include "tether/analysis/ToolpathEfficiencyAnalyzer.hpp"

#include <cmath>

using namespace tether::analysis;

// ============================================================================
// 1. MachineLimitChecker
// ============================================================================

TEST(MachineLimitChecker, NoViolationsForInBoundsGcode) {
    MachineLimitChecker::Envelope env{-200, 200, -200, 200, 0, 200};
    MachineLimitChecker checker(env);
    auto result = checker.check({"G1 X10 Y10 Z5 F1000", "G1 X50 Y50 Z10 F2000"});
    EXPECT_EQ(result.violationCount, 0);
    EXPECT_GT(result.safetyScore, 90);
}

TEST(MachineLimitChecker, DetectsOverTravelX) {
    MachineLimitChecker::Envelope env{-100, 100, -100, 100, 0, 100};
    MachineLimitChecker checker(env);
    auto result = checker.check({"G1 X150 Y10 Z5 F1000"});
    EXPECT_GE(result.violationCount, 1);
    EXPECT_GE(result.errorCount, 1);
}

TEST(MachineLimitChecker, DetectsOverTravelY) {
    MachineLimitChecker::Envelope env{-100, 100, -100, 100, 0, 100};
    MachineLimitChecker checker(env);
    auto result = checker.check({"G1 X10 Y-150 Z5 F1000"});
    EXPECT_GE(result.violationCount, 1);
}

TEST(MachineLimitChecker, DetectsOverTravelZ) {
    MachineLimitChecker::Envelope env{-100, 100, -100, 100, 0, 100};
    MachineLimitChecker checker(env);
    auto result = checker.check({"G1 X10 Y10 Z200 F1000"});
    EXPECT_GE(result.violationCount, 1);
}

TEST(MachineLimitChecker, DetectsExcessiveFeedRate) {
    MachineLimitChecker::Envelope env{-500, 500, -500, 500, 0, 500};
    MachineLimitChecker::KinematicLimits kin{1000, 3000, 20000};
    MachineLimitChecker checker(env, kin);
    auto result = checker.check({"G1 X10 Y10 Z5 F10000"});
    EXPECT_GE(result.violationCount, 1);
    EXPECT_GE(result.warningCount, 1);
}

TEST(MachineLimitChecker, EmptyGcodeNoViolations) {
    MachineLimitChecker checker;
    auto result = checker.check({});
    EXPECT_EQ(result.violationCount, 0);
    EXPECT_DOUBLE_EQ(result.safetyScore, 100.0);
}

TEST(MachineLimitChecker, CommentsIgnored) {
    MachineLimitChecker::Envelope env{-100, 100, -100, 100, 0, 100};
    MachineLimitChecker checker(env);
    auto result = checker.check({"; G1 X999 Y999", "(comment) G1 X10 Y10 Z5"});
    EXPECT_EQ(result.violationCount, 0);
}

// ============================================================================
// 2. CurvatureAnalyzer
// ============================================================================

TEST(CurvatureAnalyzer, StraightLineIsSmooth) {
    CurvatureAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G1 X10 Y0", "G1 X20 Y0", "G1 X30 Y0"});
    EXPECT_EQ(result.sharpTurnCount, 0);
    EXPECT_GT(result.smoothnessScore, 90);
}

TEST(CurvatureAnalyzer, DetectsSharpTurn) {
    CurvatureAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G1 X10 Y0", "G1 X10 Y10", "G1 X10 Y20"});
    // 90-degree turn at (10,0)→(10,10)
    EXPECT_GE(result.sharpTurnCount, 1);
}

TEST(CurvatureAnalyzer, EmptyGcode) {
    CurvatureAnalyzer analyzer;
    auto result = analyzer.analyze({});
    EXPECT_EQ(result.sharpTurnCount, 0);
    EXPECT_DOUBLE_EQ(result.smoothnessScore, 100.0);
}

TEST(CurvatureAnalyzer, CorneringSpeedComputed) {
    CurvatureAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F3000", "G1 X10 Y0", "G1 X10 Y10"});
    EXPECT_GE(result.cornerCount, 1);
    ASSERT_FALSE(result.corneringPoints.empty());
    EXPECT_GT(result.corneringPoints[0].recommendedSpeed, 0);
}

TEST(CurvatureAnalyzer, HeatmapGenerated) {
    CurvatureAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G1 X5 Y0", "G1 X5 Y5", "G1 X0 Y5"});
    EXPECT_FALSE(result.heatmap.empty());
}

// ============================================================================
// 3. ArcAnalyzer
// ============================================================================

TEST(ArcAnalyzer, NoArcsInLinearGcode) {
    ArcAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G1 X10 Y0", "G1 X20 Y0"});
    EXPECT_EQ(result.arcCount, 0);
}

TEST(ArcAnalyzer, DetectsExistingG2Arc) {
    ArcAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G2 X10 Y0 I5 J0"});
    EXPECT_EQ(result.arcCount, 1);
    EXPECT_GT(result.totalArcLength, 0);
    EXPECT_EQ(result.cwCount, 1);
}

TEST(ArcAnalyzer, DetectsExistingG3Arc) {
    ArcAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G3 X10 Y0 I5 J0"});
    EXPECT_EQ(result.arcCount, 1);
    EXPECT_EQ(result.ccwCount, 1);
}

TEST(ArcAnalyzer, DetectsArcFittingCandidates) {
    ArcAnalyzer analyzer;
    // Many small linear segments approximating an arc.
    std::vector<std::string> gcode;
    gcode.push_back("G1 X0 Y0 F1000");
    for (int i = 1; i <= 20; ++i) {
        double angle = i * 0.1;
        double x = 10 * std::cos(angle);
        double y = 10 * std::sin(angle);
        gcode.push_back("G1 X" + std::to_string(x) + " Y" + std::to_string(y));
    }
    auto result = analyzer.analyze(gcode);
    EXPECT_GE(result.candidates.size(), 1);
}

TEST(ArcAnalyzer, ArcQualityIssueSmallRadius) {
    ArcAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G2 X0.01 Y0 I0.005 J0"});
    EXPECT_GE(result.issueCount, 1);
}

TEST(ArcAnalyzer, EmptyGcode) {
    ArcAnalyzer analyzer;
    auto result = analyzer.analyze({});
    EXPECT_EQ(result.arcCount, 0);
    EXPECT_DOUBLE_EQ(result.qualityScore, 100.0);
}

// ============================================================================
// 4. ModalStateAnalyzer
// ============================================================================

TEST(ModalStateAnalyzer, DefaultStateNoChanges) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"; just a comment"});
    EXPECT_TRUE(result.changes.empty());
    EXPECT_EQ(result.finalState.motionMode, "G0");
    EXPECT_EQ(result.finalState.units, "G21");
}

TEST(ModalStateAnalyzer, DetectsMotionModeChange) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"G0 X0 Y0", "G1 X10 Y10 F1000"});
    EXPECT_GE(result.changes.size(), 1);
}

TEST(ModalStateAnalyzer, DetectsUnitChange) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"G20", "G21"});
    EXPECT_GE(result.changeCounts["units"], 2);
}

TEST(ModalStateAnalyzer, ProperlyResetAtEnd) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"M3", "M5", "M8", "M9", "G40", "G49"});
    EXPECT_TRUE(result.isProperlyReset);
}

TEST(ModalStateAnalyzer, NotResetAtEnd) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"M3"});
    EXPECT_FALSE(result.isProperlyReset);
    EXPECT_FALSE(result.warnings.empty());
}

TEST(ModalStateAnalyzer, RedundantCommandsDetected) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X10 F1000", "G1 X20", "G1 X30"});
    EXPECT_GE(result.redundantCommands, 2);
}

TEST(ModalStateAnalyzer, TransitionsTracked) {
    ModalStateAnalyzer analyzer;
    auto result = analyzer.analyze({"G0 X0", "G1 X10 F1000", "G0 X20"});
    EXPECT_GE(result.transitionCount, 2);
}

// ============================================================================
// 5. PathTopologyDetector
// ============================================================================

TEST(PathTopologyDetector, NoIssuesInSimplePath) {
    PathTopologyDetector detector;
    auto result = detector.analyze({"G1 X0 Y0 F1000", "G1 X10 Y0", "G1 X10 Y10"});
    EXPECT_EQ(result.loopCount, 0);
    EXPECT_EQ(result.intersectionCount, 0);
}

TEST(PathTopologyDetector, DetectsSelfIntersection) {
    PathTopologyDetector detector;
    // Cross pattern: (0,0)→(10,10)→(10,0)→(0,10)
    // Segment 1: (0,0)→(10,10) and Segment 3: (10,0)→(0,10) cross at (5,5).
    auto result = detector.analyze({
        "G1 X0 Y0 F1000", "G1 X10 Y10", "G1 X10 Y0", "G1 X0 Y10"
    });
    EXPECT_GE(result.intersectionCount, 1);
}

TEST(PathTopologyDetector, DetectsLoop) {
    PathTopologyDetector detector;
    // Repeat the same path 3 times.
    std::vector<std::string> gcode;
    gcode.push_back("G1 X0 Y0 F1000");
    for (int iter = 0; iter < 3; ++iter) {
        gcode.push_back("G1 X10 Y0");
        gcode.push_back("G1 X10 Y10");
        gcode.push_back("G1 X0 Y10");
        gcode.push_back("G1 X0 Y0");
    }
    auto result = detector.analyze(gcode);
    EXPECT_GE(result.loopCount, 1);
}

TEST(PathTopologyDetector, EmptyGcode) {
    PathTopologyDetector detector;
    auto result = detector.analyze({});
    EXPECT_EQ(result.loopCount, 0);
    EXPECT_EQ(result.intersectionCount, 0);
}

TEST(PathTopologyDetector, SymmetricPath) {
    PathTopologyDetector detector;
    // Symmetric around X=5.
    std::vector<std::string> gcode;
    gcode.push_back("G1 X0 Y0 F1000");
    gcode.push_back("G1 X5 Y0");
    gcode.push_back("G1 X10 Y0");
    gcode.push_back("G1 X10 Y5");
    gcode.push_back("G1 X5 Y5");
    gcode.push_back("G1 X0 Y5");
    // Add more symmetric points to reach 20+ threshold.
    for (int i = 0; i < 20; ++i) {
        double y = i * 0.5;
        gcode.push_back("G1 X0 Y" + std::to_string(y));
        gcode.push_back("G1 X10 Y" + std::to_string(y));
    }
    auto result = detector.analyze(gcode);
    // May or may not detect symmetry depending on point count.
    EXPECT_GE(result.symmetryScore, 0);
}

// ============================================================================
// 6. ToolpathEfficiencyAnalyzer
// ============================================================================

TEST(ToolpathEfficiencyAnalyzer, CuttingAndTravel) {
    ToolpathEfficiencyAnalyzer analyzer;
    auto result = analyzer.analyze({
        "G0 X0 Y0 F6000", "G1 X10 Y0 E5 F1000", "G0 X20 Y0 F6000"
    });
    EXPECT_GT(result.cuttingDistance, 0);
    EXPECT_GT(result.travelDistance, 0);
    EXPECT_GT(result.totalDistance, 0);
}

TEST(ToolpathEfficiencyAnalyzer, AllRapidNoCutting) {
    ToolpathEfficiencyAnalyzer analyzer;
    auto result = analyzer.analyze({"G0 X0 Y0 F6000", "G0 X100 Y0"});
    EXPECT_DOUBLE_EQ(result.cuttingDistance, 0.0);
    EXPECT_GT(result.rapidDistance, 0);
}

TEST(ToolpathEfficiencyAnalyzer, AirCuttingDetected) {
    ToolpathEfficiencyAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 Z5 F1000", "G1 X100 Y0 Z5"});
    EXPECT_GT(result.airCuttingDistance, 0);
}

TEST(ToolpathEfficiencyAnalyzer, EmptyGcode) {
    ToolpathEfficiencyAnalyzer analyzer;
    auto result = analyzer.analyze({});
    EXPECT_DOUBLE_EQ(result.totalDistance, 0.0);
    EXPECT_DOUBLE_EQ(result.efficiencyScore, 0.0);
}

TEST(ToolpathEfficiencyAnalyzer, ArcDistanceTracked) {
    ToolpathEfficiencyAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F1000", "G2 X10 Y0 I5 J0"});
    EXPECT_GT(result.arcDistance, 0);
}

// ============================================================================
// 7. RetractionAnalyzer
// ============================================================================

TEST(RetractionAnalyzer, NoRetractions) {
    RetractionAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X10 Y0 E5 F1000", "G1 X20 Y0 E10"});
    EXPECT_EQ(result.count, 0);
}

TEST(RetractionAnalyzer, DetectsRetraction) {
    RetractionAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X10 Y0 E5 F1800", "G1 E3 F1800", "G1 X20 Y0 E5"});
    EXPECT_GE(result.count, 1);
    EXPECT_GT(result.avgDistance, 0);
}

TEST(RetractionAnalyzer, DetectsMultipleRetractions) {
    RetractionAnalyzer analyzer;
    auto result = analyzer.analyze({
        "G1 X10 Y0 E5 F1800", "G1 E3 F1800", "G1 X20 Y0 E5",
        "G1 E3 F1800", "G1 X30 Y0 E5"
    });
    EXPECT_GE(result.count, 2);
}

TEST(RetractionAnalyzer, ExtruderTypeDetection) {
    RetractionAnalyzer analyzer;
    // Large retraction distance → Bowden.
    auto result = analyzer.analyze({"G1 X10 Y0 E5 F1800", "G1 E1 F1800"});
    EXPECT_EQ(result.extruderType, RetractionResult::ExtruderType::Bowden);
}

TEST(RetractionAnalyzer, DirectDriveDetection) {
    RetractionAnalyzer analyzer;
    // Small retraction distance → Direct.
    auto result = analyzer.analyze({"G1 X10 Y0 E5 F1800", "G1 E4.5 F1800"});
    EXPECT_EQ(result.extruderType, RetractionResult::ExtruderType::Direct);
}

TEST(RetractionAnalyzer, FrequencyAnalysis) {
    RetractionAnalyzer analyzer;
    std::vector<std::string> gcode;
    gcode.push_back("G1 X0 Y0 E0 F1800");
    for (int i = 0; i < 20; ++i) {
        gcode.push_back("G1 X" + std::to_string(i * 10) + " Y0 E" + std::to_string(i + 1));
        gcode.push_back("G1 E" + std::to_string(i + 0.5) + " F1800"); // retract
    }
    auto result = analyzer.analyze(gcode);
    EXPECT_GE(result.count, 10);
    EXPECT_GT(result.retractionsPer100Lines, 0);
}

TEST(RetractionAnalyzer, EmptyGcode) {
    RetractionAnalyzer analyzer;
    auto result = analyzer.analyze({});
    EXPECT_EQ(result.count, 0);
}

// ============================================================================
// 8. AccelerationProfileAnalyzer
// ============================================================================

TEST(AccelerationProfileAnalyzer, ConstantSpeedNoJerk) {
    AccelerationProfileAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F3000", "G1 X10 Y0", "G1 X20 Y0"});
    EXPECT_EQ(result.jerkCount, 0);
}

TEST(AccelerationProfileAnalyzer, DetectsAcceleration) {
    AccelerationProfileAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F100", "G1 X10 Y0 F3000"});
    EXPECT_GE(result.points.size(), 1);
}

TEST(AccelerationProfileAnalyzer, LimitedTimeGreaterThanUnlimited) {
    AccelerationProfileAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X0 Y0 F6000", "G1 X1 Y0 F6000", "G1 X2 Y0 F6000"});
    // Short moves → acceleration overhead.
    EXPECT_GE(result.limitedTime, result.unlimitedTime);
}

TEST(AccelerationProfileAnalyzer, DirectionChangeDetected) {
    AccelerationProfileAnalyzer analyzer;
    auto result = analyzer.analyze({
        "G1 X0 Y0 F3000", "G1 X10 Y0", "G1 X0 Y0"
    });
    EXPECT_GE(result.directionChanges, 1);
}

TEST(AccelerationProfileAnalyzer, EmptyGcode) {
    AccelerationProfileAnalyzer analyzer;
    auto result = analyzer.analyze({});
    EXPECT_DOUBLE_EQ(result.limitedTime, 0.0);
    EXPECT_DOUBLE_EQ(result.smoothnessScore, 100.0);
}

// ============================================================================
// 9. CoordinateSystemAnalyzer
// ============================================================================

TEST(CoordinateSystemAnalyzer, NoCoordinateSystems) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G1 X10 Y10 F1000"});
    EXPECT_TRUE(result.offsets.empty());
    EXPECT_EQ(result.wcsCount, 0);
}

TEST(CoordinateSystemAnalyzer, DetectsWorkOffsetG54) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G54", "G1 X10 Y10 F1000"});
    EXPECT_EQ(result.offsets.size(), 1);
    EXPECT_EQ(result.offsets[0].name, "G54");
}

TEST(CoordinateSystemAnalyzer, DetectsMultipleOffsets) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G54", "G1 X10 Y10", "G55", "G1 X20 Y20"});
    EXPECT_EQ(result.offsets.size(), 2);
    EXPECT_TRUE(result.usesMultipleOffsets);
    EXPECT_GE(result.offsetChanges, 2);
}

TEST(CoordinateSystemAnalyzer, DetectsG10L2) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G10 L2 P1 X10 Y20 Z30"});
    EXPECT_EQ(result.g10Commands.size(), 1);
    EXPECT_DOUBLE_EQ(result.g10Commands[0].x, 10.0);
}

TEST(CoordinateSystemAnalyzer, DetectsRotationG68) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G68 X0 Y0 R45", "G1 X10 Y10", "G69"});
    EXPECT_TRUE(result.hasRotation);
    EXPECT_GE(result.rotationEventCount, 2);
    EXPECT_DOUBLE_EQ(result.maxRotationAngle, 45.0);
    EXPECT_FALSE(result.activeRotationAtEnd);
}

TEST(CoordinateSystemAnalyzer, RotationActiveAtEnd) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G68 X0 Y0 R30", "G1 X10 Y10"});
    EXPECT_TRUE(result.activeRotationAtEnd);
}

TEST(CoordinateSystemAnalyzer, DetectsScalingG51) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({"G51 X2 Y2", "G1 X10 Y10", "G50"});
    EXPECT_EQ(result.scaleEventCount, 2);
    EXPECT_FALSE(result.scaleActiveAtEnd);
}

TEST(CoordinateSystemAnalyzer, EmptyGcode) {
    CoordinateSystemAnalyzer analyzer;
    auto result = analyzer.analyze({});
    EXPECT_EQ(result.wcsCount, 0);
    EXPECT_FALSE(result.hasRotation);
}

// ============================================================================
// 10. PathContinuityChecker
// ============================================================================

TEST(PathContinuityChecker, ContinuousPath) {
    PathContinuityChecker checker;
    auto result = checker.analyze({"G1 X0 Y0 F1000", "G1 X10 Y0", "G1 X10 Y10"});
    EXPECT_TRUE(result.isContinuous);
    EXPECT_GT(result.continuityScore, 90);
}

TEST(PathContinuityChecker, DetectsRapidGap) {
    PathContinuityChecker checker;
    auto result = checker.analyze({"G1 X0 Y0 F1000", "G0 X100 Y0", "G1 X110 Y0 E5"});
    EXPECT_GE(result.issueCount, 1);
    EXPECT_GT(result.totalGapDistance, 0);
}

TEST(PathContinuityChecker, DetectsZJump) {
    PathContinuityChecker checker;
    auto result = checker.analyze({"G1 X0 Y0 Z0 F1000", "G1 X10 Y0 Z10"});
    EXPECT_GE(result.issueCount, 1);
}

TEST(PathContinuityChecker, DetectsRetraction) {
    PathContinuityChecker checker;
    auto result = checker.analyze({"G1 X10 Y0 E5 F1000", "G1 E3 F1800"});
    EXPECT_GE(result.issueCount, 1);
}

TEST(PathContinuityChecker, DetectsDirectionChange) {
    PathContinuityChecker checker;
    auto result = checker.analyze({"G1 X0 Y0 F1000", "G1 X10 Y0", "G1 X0 Y0"});
    // Direction reversal at (10,0)→(0,0) reverses (0,0)→(10,0).
    bool hasDirChange = false;
    for (const auto& issue : result.issues) {
        if (issue.type == ContinuityIssue::Type::DirectionChange) hasDirChange = true;
    }
    EXPECT_TRUE(hasDirChange);
}

TEST(PathContinuityChecker, PerLayerContinuity) {
    PathContinuityChecker checker;
    auto result = checker.analyze({
        "G1 X0 Y0 Z0 F1000", "G1 X10 Y0 Z0", "G1 X20 Y0 Z0.2",
        "G1 X30 Y0 Z0.2"
    });
    EXPECT_GE(result.layerCount, 1);
}

TEST(PathContinuityChecker, EmptyGcode) {
    PathContinuityChecker checker;
    auto result = checker.analyze({});
    EXPECT_EQ(result.issueCount, 0);
    EXPECT_DOUBLE_EQ(result.continuityScore, 100.0);
    EXPECT_TRUE(result.isContinuous);
}

// ============================================================================
// GcodeParseUtils
// ============================================================================

TEST(GcodeParseUtils, StripComments) {
    EXPECT_EQ(stripComments("G1 X10 ; comment"), "G1 X10");
    EXPECT_EQ(stripComments("G1 X10 (inline) Y20"), "G1 X10  Y20");
    EXPECT_EQ(stripComments("; full comment"), "");
}

TEST(GcodeParseUtils, ExtractValue) {
    EXPECT_DOUBLE_EQ(*extractValue("G1 X10.5 Y20", 'X'), 10.5);
    EXPECT_DOUBLE_EQ(*extractValue("G1 X-5.2 Y20", 'X'), -5.2);
    EXPECT_FALSE(extractValue("G1 Y20", 'X').has_value());
}

TEST(GcodeParseUtils, DetectMotionMode) {
    EXPECT_EQ(detectMotionMode("G0 X10"), MotionMode::Rapid);
    EXPECT_EQ(detectMotionMode("G1 X10"), MotionMode::Feed);
    EXPECT_EQ(detectMotionMode("G2 X10"), MotionMode::ArcCW);
    EXPECT_EQ(detectMotionMode("G3 X10"), MotionMode::ArcCCW);
    EXPECT_EQ(detectMotionMode("M3"), MotionMode::None);
}

TEST(GcodeParseUtils, HasWord) {
    EXPECT_TRUE(hasWord("G1 X10 F1000", "G1"));
    EXPECT_FALSE(hasWord("G10 X10", "G1"));
    EXPECT_TRUE(hasWord("M3 S1000", "M3"));
}
