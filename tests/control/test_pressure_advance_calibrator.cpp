/**
 * @file test_pressure_advance_calibrator.cpp
 * @brief Unit tests for the PressureAdvanceCalibrator.
 *
 * Verifies:
 *  - Heuristic PA table returns sensible values per material + extruder.
 *  - G-code PA detection (M900, SET_PRESSURE_ADVANCE, M572).
 *  - Extruder type classification from retraction statistics.
 *  - Risk scoring (ooze risk when PA too low, under-extrusion when too high).
 *  - Tuning-curve parabola fit finds the known optimum.
 *  - Material name parsing.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "tether/control/extrusion/PressureAdvanceCalibrator.hpp"

using namespace tether::control::extrusion;

// ============================================================================
// Heuristic PA table
// ============================================================================

TEST(PressureAdvanceCalibrator, HeuristicPaDirectDriveLowerThanBowden) {
    for (int m = 0; m <= static_cast<int>(FilamentMaterial::ASA); ++m) {
        const auto mat = static_cast<FilamentMaterial>(m);
        const double direct = PressureAdvanceCalibrator::heuristicPa(
            mat, ExtruderType::DirectDrive);
        const double bowden = PressureAdvanceCalibrator::heuristicPa(
            mat, ExtruderType::Bowden);
        EXPECT_GT(bowden, direct)
            << "Bowden PA should be higher than direct-drive for material " << m;
        EXPECT_GT(direct, 0.0);
        EXPECT_GT(bowden, 0.0);
    }
}

TEST(PressureAdvanceCalibrator, HeuristicPlaDirectDrive) {
    EXPECT_NEAR(PressureAdvanceCalibrator::heuristicPa(
        FilamentMaterial::PLA, ExtruderType::DirectDrive), 0.04, 1e-9);
}

TEST(PressureAdvanceCalibrator, HeuristicPlaBowden) {
    EXPECT_NEAR(PressureAdvanceCalibrator::heuristicPa(
        FilamentMaterial::PLA, ExtruderType::Bowden), 0.60, 1e-9);
}

TEST(PressureAdvanceCalibrator, HeuristicTpuLowerThanPla) {
    // TPU is flexible — lower PA to avoid filament buckling.
    const double tpu = PressureAdvanceCalibrator::heuristicPa(
        FilamentMaterial::TPU, ExtruderType::DirectDrive);
    const double pla = PressureAdvanceCalibrator::heuristicPa(
        FilamentMaterial::PLA, ExtruderType::DirectDrive);
    EXPECT_LT(tpu, pla);
}

// ============================================================================
// Material parsing
// ============================================================================

TEST(PressureAdvanceCalibrator, ParseMaterialPla) {
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("PLA"), FilamentMaterial::PLA);
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("pla"), FilamentMaterial::PLA);
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("PLA_Red"), FilamentMaterial::PLA);
}

TEST(PressureAdvanceCalibrator, ParseMaterialPetg) {
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("PETG"), FilamentMaterial::PETG);
}

TEST(PressureAdvanceCalibrator, ParseMaterialTpu) {
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("TPU"), FilamentMaterial::TPU);
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("Flexible"), FilamentMaterial::TPU);
}

TEST(PressureAdvanceCalibrator, ParseMaterialUnknown) {
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial("Unknown"), FilamentMaterial::Unknown);
    EXPECT_EQ(PressureAdvanceCalibrator::parseMaterial(""), FilamentMaterial::Unknown);
}

// ============================================================================
// G-code PA detection
// ============================================================================

TEST(PressureAdvanceCalibrator, DetectsMarlinM900) {
    PressureAdvanceCalibrator cal;
    auto result = cal.analyse({
        "G1 X10 Y10 E5 F1800",
        "M900 K0.045",
        "G1 X20 Y10 E10 F1800",
    });
    EXPECT_TRUE(result.paEnabled);
    EXPECT_NEAR(result.currentValue, 0.045, 1e-9);
    ASSERT_EQ(result.settings.size(), 1u);
    EXPECT_EQ(result.settings[0].command, "M900");
}

TEST(PressureAdvanceCalibrator, DetectsKlipperSetPressureAdvance) {
    PressureAdvanceCalibrator cal;
    auto result = cal.analyse({
        "G1 X10 Y10 E5 F1800",
        "SET_PRESSURE_ADVANCE ADVANCE=0.06",
    });
    EXPECT_TRUE(result.paEnabled);
    EXPECT_NEAR(result.currentValue, 0.06, 1e-9);
    ASSERT_EQ(result.settings.size(), 1u);
    EXPECT_EQ(result.settings[0].command, "SET_PRESSURE_ADVANCE");
}

TEST(PressureAdvanceCalibrator, DetectsRepRapM572) {
    PressureAdvanceCalibrator cal;
    auto result = cal.analyse({
        "M572 D0 S0.03",
        "G1 X10 Y10 E5 F1800",
    });
    EXPECT_TRUE(result.paEnabled);
    EXPECT_NEAR(result.currentValue, 0.03, 1e-9);
    ASSERT_EQ(result.settings.size(), 1u);
    EXPECT_EQ(result.settings[0].command, "M572");
}

TEST(PressureAdvanceCalibrator, NoPaCommandGivesDisabled) {
    PressureAdvanceCalibrator cal;
    auto result = cal.analyse({"G1 X10 Y10 E5 F1800"});
    EXPECT_FALSE(result.paEnabled);
    EXPECT_EQ(result.currentValue, 0.0);
    EXPECT_TRUE(result.settings.empty());
}

// ============================================================================
// Extruder classification from retractions
// ============================================================================

TEST(PressureAdvanceCalibrator, ClassifiesBowdenFromLongRetractions) {
    PressureAdvanceCalibrator::Params p;
    p.extruderType = ExtruderType::Unknown;
    p.material = FilamentMaterial::PLA;
    PressureAdvanceCalibrator cal(p);

    // Retractions of 5mm → Bowden.
    auto result = cal.analyse({
        "G1 X10 Y10 E10 F1800",
        "G1 X10 Y10 E5 F1800",    // 5mm retraction
        "G1 X20 Y10 E10 F1800",
        "G1 X20 Y10 E5 F1800",    // 5mm retraction
        "G1 X30 Y10 E10 F1800",
    });
    EXPECT_EQ(result.detectedExtruder, ExtruderType::Bowden);
    EXPECT_NEAR(result.avgRetractionDistance, 5.0, 1e-9);
    EXPECT_EQ(result.retractionCount, 2);
}

TEST(PressureAdvanceCalibrator, ClassifiesDirectDriveFromShortRetractions) {
    PressureAdvanceCalibrator::Params p;
    p.extruderType = ExtruderType::Unknown;
    p.material = FilamentMaterial::PLA;
    PressureAdvanceCalibrator cal(p);

    // Retractions of 1mm → direct-drive.
    auto result = cal.analyse({
        "G1 X10 Y10 E10 F1800",
        "G1 X10 Y10 E9 F1800",    // 1mm retraction
        "G1 X20 Y10 E10 F1800",
        "G1 X20 Y10 E9 F1800",    // 1mm retraction
    });
    EXPECT_EQ(result.detectedExtruder, ExtruderType::DirectDrive);
    EXPECT_NEAR(result.avgRetractionDistance, 1.0, 1e-9);
}

TEST(PressureAdvanceCalibrator, NoRetractionsKeepsUserType) {
    PressureAdvanceCalibrator::Params p;
    p.extruderType = ExtruderType::DirectDrive;
    p.material = FilamentMaterial::PLA;
    PressureAdvanceCalibrator cal(p);

    auto result = cal.analyse({"G1 X10 Y10 E5 F1800", "G1 X20 Y10 E10 F1800"});
    EXPECT_EQ(result.detectedExtruder, ExtruderType::DirectDrive);
    EXPECT_EQ(result.retractionCount, 0);
}

// ============================================================================
// Risk scoring
// ============================================================================

TEST(PressureAdvanceCalibrator, OozeRiskWhenPaTooLow) {
    PressureAdvanceCalibrator::Params p;
    p.material = FilamentMaterial::PLA;
    p.extruderType = ExtruderType::DirectDrive; // recommended = 0.04
    PressureAdvanceCalibrator cal(p);

    auto result = cal.analyse({"M900 K0.01"}); // PA too low
    EXPECT_GT(result.oozeRisk, 0.0);
    EXPECT_EQ(result.underExtrusionRisk, 0.0);
}

TEST(PressureAdvanceCalibrator, UnderExtrusionRiskWhenPaTooHigh) {
    PressureAdvanceCalibrator::Params p;
    p.material = FilamentMaterial::PLA;
    p.extruderType = ExtruderType::DirectDrive; // recommended = 0.04
    PressureAdvanceCalibrator cal(p);

    auto result = cal.analyse({"M900 K0.08"}); // PA too high
    EXPECT_EQ(result.oozeRisk, 0.0);
    EXPECT_GT(result.underExtrusionRisk, 0.0);
}

TEST(PressureAdvanceCalibrator, ConsistencyScoreHighWhenPaMatches) {
    PressureAdvanceCalibrator::Params p;
    p.material = FilamentMaterial::PLA;
    p.extruderType = ExtruderType::DirectDrive; // recommended = 0.04
    PressureAdvanceCalibrator cal(p);

    auto result = cal.analyse({"M900 K0.04"}); // exact match
    EXPECT_GT(result.consistencyScore, 80.0);
}

// ============================================================================
// Tuning-curve fit
// ============================================================================

TEST(PressureAdvanceCalibrator, FitTuningCurveFindsOptimum) {
    // Parabola with minimum at PA = 0.045.
    // y = 1000 * (x - 0.045)^2
    std::vector<CalibrationPoint> points;
    for (double pa : {0.02, 0.03, 0.04, 0.045, 0.05, 0.06, 0.07}) {
        const double err = 1000.0 * (pa - 0.045) * (pa - 0.045);
        points.push_back({pa, err});
    }
    const double optimum = PressureAdvanceCalibrator::fitTuningCurve(points);
    EXPECT_NEAR(optimum, 0.045, 1e-6);
}

TEST(PressureAdvanceCalibrator, FitTuningCurveTooFewPoints) {
    std::vector<CalibrationPoint> points{{0.04, 0.1}, {0.05, 0.05}};
    EXPECT_EQ(PressureAdvanceCalibrator::fitTuningCurve(points), 0.0);
}

TEST(PressureAdvanceCalibrator, FitTuningCurveLinearData) {
    // Linear data — no curvature, no optimum.
    std::vector<CalibrationPoint> points{{0.02, 0.1}, {0.04, 0.2}, {0.06, 0.3}};
    EXPECT_EQ(PressureAdvanceCalibrator::fitTuningCurve(points), 0.0);
}

// ============================================================================
// Recommendations
// ============================================================================

TEST(PressureAdvanceCalibrator, GeneratesRecommendations) {
    PressureAdvanceCalibrator::Params p;
    p.material = FilamentMaterial::PLA;
    p.extruderType = ExtruderType::DirectDrive;
    PressureAdvanceCalibrator cal(p);

    auto result = cal.analyse({"G1 X10 Y10 E5 F1800"});
    EXPECT_FALSE(result.recommendations.empty());
}

TEST(PressureAdvanceCalibrator, RecommendsEnablingPaWhenDisabled) {
    PressureAdvanceCalibrator::Params p;
    p.material = FilamentMaterial::PLA;
    p.extruderType = ExtruderType::DirectDrive;
    PressureAdvanceCalibrator cal(p);

    auto result = cal.analyse({"G1 X10 Y10 E5 F1800"});
    bool foundEnable = false;
    for (const auto& r : result.recommendations) {
        if (r.find("consider enabling") != std::string::npos) {
            foundEnable = true;
            break;
        }
    }
    EXPECT_TRUE(foundEnable);
}
