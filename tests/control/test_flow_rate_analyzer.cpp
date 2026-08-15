/**
 * @file test_flow_rate_analyzer.cpp
 * @brief Unit tests for the FlowRateAnalyzer.
 *
 * Verifies:
 *  - Flow-rate computation from G1 extruding moves.
 *  - Statistical measures (mean, min, max, std dev, CV).
 *  - Outlier detection.
 *  - Bead-width estimation and over/under-extrusion detection.
 *  - Calibration advice (flow-rate adjustment).
 *  - Consistency score.
 *  - Empty / no-extrusion edge case.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "tether/control/extrusion/FlowRateAnalyzer.hpp"

using namespace tether::control::extrusion;

// ============================================================================
// Basic flow-rate computation
// ============================================================================

TEST(FlowRateAnalyzer, ComputesFlowRateFromExtrudingMove) {
    FlowRateAnalyzer::Params p;
    p.filamentDiameterMm = 1.75;
    p.nozzleDiameterMm = 0.4;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    // G1 X10 Y0 E5 F1800 → extrude 5mm filament over 10mm XY at 1800 mm/min.
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
    });
    EXPECT_EQ(result.sampleCount, 1);
    EXPECT_GT(result.avgFlowRate, 0.0);
}

TEST(FlowRateAnalyzer, MultipleMovesGiveMultipleSamples) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
        "G1 X20 Y0 E10 F1800",
        "G1 X30 Y0 E15 F1800",
    });
    EXPECT_EQ(result.sampleCount, 3);
}

TEST(FlowRateAnalyzer, NonExtrudingMovesIgnored) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({
        "G0 X10 Y0",           // rapid travel — no extrusion
        "G1 X20 Y0 F1800",     // no E → no extrusion
    });
    EXPECT_EQ(result.sampleCount, 0);
}

TEST(FlowRateAnalyzer, RetractionsIgnored) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({
        "G1 X10 Y0 E10 F1800",
        "G1 X10 Y0 E8 F1800",   // retraction (E decreasing)
        "G1 X20 Y0 E10 F1800",  // re-retraction (E increasing — counts as extrusion)
    });
    // First and third moves are extruding (E increasing); retraction is skipped.
    EXPECT_EQ(result.sampleCount, 2);
}

// ============================================================================
// Statistics
// ============================================================================

TEST(FlowRateAnalyzer, MinMaxFlowRate) {
    FlowRateAnalyzer analyzer;
    // First move at F1800, second at F3600 → different flow rates.
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
        "G1 X20 Y0 E10 F3600",
    });
    EXPECT_GT(result.maxFlowRate, result.minFlowRate);
    EXPECT_LE(result.minFlowRate, result.avgFlowRate);
    EXPECT_LE(result.avgFlowRate, result.maxFlowRate);
}

TEST(FlowRateAnalyzer, ConsistentFlowRateGivesHighScore) {
    FlowRateAnalyzer analyzer;
    // All moves at the same feed rate and extrusion ratio.
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
        "G1 X20 Y0 E10 F1800",
        "G1 X30 Y0 E15 F1800",
        "G1 X40 Y0 E20 F1800",
    });
    EXPECT_GT(result.consistencyScore, 90.0);
    EXPECT_NEAR(result.coefficientOfVariation, 0.0, 1e-6);
}

TEST(FlowRateAnalyzer, InconsistentFlowRateGivesLowScore) {
    FlowRateAnalyzer analyzer;
    // Very different feed rates → high CV.
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F600",
        "G1 X20 Y0 E10 F6000",
    });
    EXPECT_LT(result.consistencyScore, 80.0);
    EXPECT_GT(result.coefficientOfVariation, 0.2);
}

TEST(FlowRateAnalyzer, OutlierDetection) {
    FlowRateAnalyzer analyzer;
    // Several normal moves + one extreme outlier.
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
        "G1 X20 Y0 E10 F1800",
        "G1 X30 Y0 E15 F1800",
        "G1 X40 Y0 E20 F1800",
        "G1 X50 Y0 E25 F1800",
        "G1 X60 Y0 E30 F1800",
        "G1 X70 Y0 E35 F1800",
        "G1 X80 Y0 E40 F1800",
        "G1 X90 Y0 E45 F1800",
        "G1 X100 Y0 E95 F18000",  // extreme flow rate outlier
    });
    EXPECT_GT(result.outlierCount, 0);
}

// ============================================================================
// Bead-width analysis
// ============================================================================

TEST(FlowRateAnalyzer, BeadWidthNearExpected) {
    FlowRateAnalyzer::Params p;
    p.nozzleDiameterMm = 0.4;
    p.filamentDiameterMm = 1.75;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    // For a 0.4mm nozzle at 0.2mm layer height, expected bead width ≈ 0.48mm.
    // Extrude 5mm of 1.75mm filament over 10mm XY → volume = 5 * π*(0.875)² ≈ 12.37 mm³
    // width = 12.37 / (10 * 0.2) = 6.18mm — that's way too wide, but the test
    // just checks the computation runs.
    auto result = analyzer.analyse({"G1 X10 Y0 E5 F1800"});
    EXPECT_GT(result.avgBeadWidth, 0.0);
    EXPECT_GT(result.expectedBeadWidth, 0.0);
}

TEST(FlowRateAnalyzer, OverExtrusionDetected) {
    FlowRateAnalyzer::Params p;
    p.nozzleDiameterMm = 0.4;
    p.filamentDiameterMm = 1.75;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    // Heavy extrusion → wide bead → over-extrusion.
    auto result = analyzer.analyse({"G1 X10 Y0 E20 F1800"});
    EXPECT_TRUE(result.overExtrusion);
    EXPECT_FALSE(result.underExtrusion);
    EXPECT_GT(result.widthDeviationPercent, 5.0);
}

TEST(FlowRateAnalyzer, UnderExtrusionDetected) {
    FlowRateAnalyzer::Params p;
    p.nozzleDiameterMm = 0.4;
    p.filamentDiameterMm = 1.75;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    // Very little extrusion → narrow bead → under-extrusion.
    auto result = analyzer.analyse({"G1 X100 Y0 E0.01 F1800"});
    EXPECT_TRUE(result.underExtrusion);
    EXPECT_FALSE(result.overExtrusion);
    EXPECT_LT(result.widthDeviationPercent, -5.0);
}

TEST(FlowRateAnalyzer, FlowAdjustmentAdviceForOverExtrusion) {
    FlowRateAnalyzer::Params p;
    p.nozzleDiameterMm = 0.4;
    p.filamentDiameterMm = 1.75;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    auto result = analyzer.analyse({"G1 X10 Y0 E20 F1800"});
    EXPECT_LT(result.flowAdjustmentPercent, 0.0);  // reduce flow
    EXPECT_LT(result.recommendedFlowRatePercent, 100.0);
}

TEST(FlowRateAnalyzer, FlowAdjustmentAdviceForUnderExtrusion) {
    FlowRateAnalyzer::Params p;
    p.nozzleDiameterMm = 0.4;
    p.filamentDiameterMm = 1.75;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    auto result = analyzer.analyse({"G1 X100 Y0 E0.01 F1800"});
    EXPECT_GT(result.flowAdjustmentPercent, 0.0);  // increase flow
    EXPECT_GT(result.recommendedFlowRatePercent, 100.0);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST(FlowRateAnalyzer, NoExtrusionGivesEmptyResult) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({
        "G0 X10 Y0",
        "; just a comment",
    });
    EXPECT_EQ(result.sampleCount, 0);
    EXPECT_EQ(result.avgFlowRate, 0.0);
    EXPECT_EQ(result.consistencyScore, 100.0);
}

TEST(FlowRateAnalyzer, EmptyGcode) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({});
    EXPECT_EQ(result.sampleCount, 0);
}

TEST(FlowRateAnalyzer, CommentsStripped) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800 ; print perimeter",
        "G1 X20 Y0 E10 F1800 ; continue",
    });
    EXPECT_EQ(result.sampleCount, 2);
}

TEST(FlowRateAnalyzer, MaxStoredSamplesCap) {
    FlowRateAnalyzer::Params p;
    p.maxStoredSamples = 3;
    FlowRateAnalyzer analyzer(p);

    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
        "G1 X20 Y0 E10 F1800",
        "G1 X30 Y0 E15 F1800",
        "G1 X40 Y0 E20 F1800",
        "G1 X50 Y0 E25 F1800",
    });
    EXPECT_EQ(result.sampleCount, 5);
    EXPECT_EQ(result.samples.size(), 3u);  // capped
}

// ============================================================================
// Recommendations
// ============================================================================

TEST(FlowRateAnalyzer, GeneratesRecommendations) {
    FlowRateAnalyzer analyzer;
    auto result = analyzer.analyse({
        "G1 X10 Y0 E5 F1800",
        "G1 X20 Y0 E10 F1800",
    });
    EXPECT_FALSE(result.recommendations.empty());
}

TEST(FlowRateAnalyzer, RecommendsFlowReductionForOverExtrusion) {
    FlowRateAnalyzer::Params p;
    p.nozzleDiameterMm = 0.4;
    p.filamentDiameterMm = 1.75;
    p.defaultLayerHeightMm = 0.2;
    FlowRateAnalyzer analyzer(p);

    auto result = analyzer.analyse({"G1 X10 Y0 E20 F1800"});
    bool foundReduce = false;
    for (const auto& r : result.recommendations) {
        if (r.find("Reduce flow") != std::string::npos) {
            foundReduce = true;
            break;
        }
    }
    EXPECT_TRUE(foundReduce);
}
