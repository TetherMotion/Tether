/**
 * @file ThermalCurveTests.cpp
 * @brief Tests for ThermalCurve: preset profiles, spec windows, and
 *        analyzeProfile/analyzeSamples metrics + violations.
 */

#include <gtest/gtest.h>

#include <vector>

#include "tether/control/thermal/ThermalCurve.hpp"

namespace {
using namespace tether::control::thermal;
}

TEST(ThermalCurve, PresetsProduceValidProfiles) {
    for (size_t i = 0; i < thermalCurvePresetCount; ++i) {
        auto p = presetProfile(static_cast<ThermalCurvePreset>(i));
        EXPECT_FALSE(p.empty()) << thermalCurvePresetName(
            static_cast<ThermalCurvePreset>(i));
        EXPECT_GT(p.duration(), 0.0);
        EXPECT_GT(p.value(p.duration()), 0.0);
    }
}

TEST(ThermalCurve, PresetSpecsAreSelfConsistent) {
    // Every preset's own reference profile should pass its spec —
    // the presets are designed to their windows.
    for (size_t i = 0; i < thermalCurvePresetCount; ++i) {
        auto preset = static_cast<ThermalCurvePreset>(i);
        auto rep = analyzeProfile(presetProfile(preset),
                                  specForPreset(preset));
        EXPECT_TRUE(rep.pass())
            << thermalCurvePresetName(preset) << ": "
            << (rep.violations.empty() ? "" : rep.violations.front());
    }
}

TEST(ThermalCurve, Sac305Metrics) {
    auto rep = analyzeProfile(
        presetProfile(ThermalCurvePreset::ReflowSac305),
        specForPreset(ThermalCurvePreset::ReflowSac305));
    EXPECT_NEAR(rep.peakC, 245.0, 1e-6);
    EXPECT_GT(rep.talS, 60.0);
    EXPECT_LT(rep.talS, 150.0);
    EXPECT_GT(rep.soakS, 60.0);
    EXPECT_LE(rep.heatRampMax, 3.0);
}

TEST(ThermalCurve, ViolationsDetected) {
    // Flat 300C profile: violates SAC305 peak + TAL + soak + ramps.
    TemperatureProfile bad(300.0);
    bad.addSoak(300.0);
    auto rep = analyzeProfile(
        bad, specForPreset(ThermalCurvePreset::ReflowSac305));
    EXPECT_FALSE(rep.pass());
    EXPECT_GT(rep.violations.size(), 2u);
}

TEST(ThermalCurve, AnalyzeSamplesMatchesProfile) {
    auto prof = presetProfile(ThermalCurvePreset::ReflowSnPb);
    // Sample the profile at 1 s — metrics should be close to exact.
    std::vector<double> t, T;
    for (double tt = 0.0; tt <= prof.duration(); tt += 1.0) {
        t.push_back(tt);
        T.push_back(prof.value(tt));
    }
    auto spec = specForPreset(ThermalCurvePreset::ReflowSnPb);
    auto exact = analyzeProfile(prof, spec);
    auto sampled = analyzeSamples(t, T, spec);
    EXPECT_NEAR(sampled.peakC, exact.peakC, 1e-6);
    EXPECT_NEAR(sampled.talS, exact.talS, 2.0);
    EXPECT_NEAR(sampled.soakS, exact.soakS, 2.0);
    EXPECT_TRUE(sampled.pass());
}

TEST(ThermalCurve, AnalyzeSamplesRejectsBadInput) {
    auto spec = specForPreset(ThermalCurvePreset::ReflowSac305);
    EXPECT_FALSE(analyzeSamples({0, 1}, {25}, spec).pass());
    EXPECT_FALSE(analyzeSamples({0, 2, 1}, {25, 50, 60}, spec).pass());
}
