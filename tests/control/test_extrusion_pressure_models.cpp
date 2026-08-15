/**
 * @file test_extrusion_pressure_models.cpp
 * @brief Unit tests for the position-offset extrusion pressure-advance models.
 *
 * Verifies:
 *  - Power-law model: Newtonian limit (n=1) reproduces the linear PA offset
 *    δe = PA · v_e exactly (regression test against classic Klipper PA).
 *  - Q→0 boundedness: offset → 0 as v_e → 0 (no singularity).
 *  - Monotonicity in velocity.
 *  - Smoothing reduces high-frequency jitter.
 *  - maxCompensation clamp.
 *  - Cross-WLF model: LUT interpolation accuracy and temperature dependence.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "tether/control/extrusion/ExtrusionPressureModels.hpp"
#include "tether/control/extrusion/CrossWlfRheology.hpp"
#include "tether/control/extrusion/PressureFlowLut.hpp"

using namespace tether::control::extrusion;

namespace {
constexpr double kPi = M_PI;
}

// ============================================================================
// PowerLawPressureAdvance
// ============================================================================

TEST(PowerLawPressureAdvance, NewtonianLimitMatchesLinearPA) {
    // For n=1: δe = K_base · Q = K_base · A_f · v_e.
    // Classic linear PA: δe = PA · v_e. So K_base · A_f == PA.
    FilamentGeometry fil{1.75};
    const double Af = fil.areaMm2();
    const double PA = 0.045; // classic PA in seconds
    PowerLawPressureAdvance::Params pp;
    pp.baseGain = PA / Af;   // K_base = PA / A_f
    pp.flowIndex = 1.0;
    pp.smoothTime = 0.0;
    pp.maxCompensation = 10.0;
    PowerLawPressureAdvance model(pp, fil);
    for (double v : {1.0, 5.0, 10.0, 20.0}) {
        const double offLinear = PA * v;
        const double offModel = model.offsetForVelocity(v);
        EXPECT_NEAR(offModel, offLinear, 1e-9 * (1.0 + offLinear));
    }
}

TEST(PowerLawPressureAdvance, ZeroVelocityGivesZeroOffset) {
    FilamentGeometry fil{1.75};
    PowerLawPressureAdvance::Params pp;
    pp.baseGain = 0.01;
    pp.flowIndex = 0.4; // shear-thinning
    PowerLawPressureAdvance model(pp, fil);
    EXPECT_EQ(model.offsetForVelocity(0.0), 0.0);
    EXPECT_EQ(model.offsetForVelocity(-1.0), 0.0); // negative flow clamped
}

TEST(PowerLawPressureAdvance, MonotonicInVelocity) {
    FilamentGeometry fil{1.75};
    PowerLawPressureAdvance::Params pp;
    pp.baseGain = 0.01;
    pp.flowIndex = 0.5;
    pp.maxCompensation = 100.0;
    PowerLawPressureAdvance model(pp, fil);
    double prev = 0.0;
    for (double v : {0.5, 1.0, 2.0, 5.0, 10.0, 20.0}) {
        const double off = model.offsetForVelocity(v);
        EXPECT_GT(off, prev);
        prev = off;
    }
}

TEST(PowerLawPressureAdvance, MaxCompensationClamp) {
    FilamentGeometry fil{1.75};
    PowerLawPressureAdvance::Params pp;
    pp.baseGain = 1.0;
    pp.flowIndex = 1.0;
    pp.maxCompensation = 0.05;
    pp.smoothTime = 0.0;
    PowerLawPressureAdvance model(pp, fil);
    // At v=100 mm/s, raw offset = K_base·A_f·v = 1·π·1.75²/4·100 ≈ 240 mm.
    EXPECT_NEAR(model.offsetForVelocity(100.0), 0.05, 1e-9);
}

TEST(PowerLawPressureAdvance, SmoothingReducesJitter) {
    FilamentGeometry fil{1.75};
    PowerLawPressureAdvance::Params pp;
    pp.baseGain = 0.001;
    pp.flowIndex = 1.0;
    pp.smoothTime = 0.020;
    pp.maxCompensation = 100.0;
    PowerLawPressureAdvance model(pp, fil);
    // Velocity series with a single spike.
    std::vector<double> vel{10.0, 10.0, 10.0, 100.0, 10.0, 10.0, 10.0};
    auto smoothed = model.smoothVelocity(vel, 0.001); // 1 ms sample
    // The spike sample should be pulled down toward the local mean.
    EXPECT_LT(smoothed[3], 100.0);
    EXPECT_GT(smoothed[3], 10.0);
    // Neighbors of the spike should be pulled up slightly.
    EXPECT_GT(smoothed[2], 10.0);
    EXPECT_GT(smoothed[4], 10.0);
}

TEST(PowerLawPressureAdvance, OffsetSeriesLengthMatchesInput) {
    FilamentGeometry fil{1.75};
    PowerLawPressureAdvance::Params pp;
    pp.baseGain = 0.001;
    pp.flowIndex = 0.5;
    pp.smoothTime = 0.0; // disable smoothing to test endpoint behavior
    PowerLawPressureAdvance model(pp, fil);
    std::vector<double> vel{0.0, 5.0, 10.0, 5.0, 0.0};
    auto off = model.offsetSeries(vel, 0.001);
    EXPECT_EQ(off.size(), vel.size());
    EXPECT_EQ(off.front(), 0.0);
    EXPECT_EQ(off.back(), 0.0);
}

// ============================================================================
// CrossWlfPressureAdvance
// ============================================================================

TEST(CrossWlfPressureAdvance, OffsetMatchesLutTimesCompressibility) {
    CrossWlfParams rp;
    NozzleGeometry g{0.2, 10.0};
    auto lut = std::make_shared<PressureFlowLut>();
    lut->build(rp, g, {1.0, 2.0, 4.0}, {200.0, 220.0, 240.0});

    FilamentGeometry fil{1.75};
    CrossWlfPressureAdvance::Params pp;
    pp.compressibilityOverArea = 1e-5; // mm/Pa
    pp.smoothTime = 0.0;
    pp.maxCompensation = 100.0;
    CrossWlfPressureAdvance model(lut, pp, fil);

    const double v = 5.0; // mm/s
    const double Q = v * fil.areaMm2();
    const double T = 220.0;
    const double P = lut->pressure(Q, T);
    const double expected = pp.compressibilityOverArea * P;
    EXPECT_NEAR(model.offsetForVelocity(v, T), expected, 1e-9 * expected);
}

TEST(CrossWlfPressureAdvance, OffsetDecreasesWithTemperature) {
    CrossWlfParams rp;
    NozzleGeometry g{0.2, 10.0};
    auto lut = std::make_shared<PressureFlowLut>();
    lut->build(rp, g, {1.0, 2.0, 4.0, 8.0}, {180.0, 200.0, 220.0, 240.0});

    FilamentGeometry fil{1.75};
    CrossWlfPressureAdvance::Params pp;
    pp.compressibilityOverArea = 1e-5;
    pp.maxCompensation = 100.0;
    CrossWlfPressureAdvance model(lut, pp, fil);
    const double v = 5.0;
    EXPECT_GT(model.offsetForVelocity(v, 180.0),
              model.offsetForVelocity(v, 240.0));
}

TEST(CrossWlfPressureAdvance, EmptyLutGivesZeroOffset) {
    auto lut = std::make_shared<PressureFlowLut>(); // empty
    FilamentGeometry fil{1.75};
    CrossWlfPressureAdvance::Params pp;
    pp.compressibilityOverArea = 1e-5;
    CrossWlfPressureAdvance model(lut, pp, fil);
    EXPECT_EQ(model.offsetForVelocity(5.0, 220.0), 0.0);
}

TEST(CrossWlfPressureAdvance, MaxCompensationClamp) {
    CrossWlfParams rp;
    NozzleGeometry g{0.2, 10.0};
    auto lut = std::make_shared<PressureFlowLut>();
    lut->build(rp, g, {1.0, 10.0, 100.0}, {180.0, 220.0});
    FilamentGeometry fil{1.75};
    CrossWlfPressureAdvance::Params pp;
    pp.compressibilityOverArea = 1.0; // huge → will clamp
    pp.maxCompensation = 0.02;
    CrossWlfPressureAdvance model(lut, pp, fil);
    EXPECT_NEAR(model.offsetForVelocity(50.0, 220.0), 0.02, 1e-9);
}

TEST(CrossWlfPressureAdvance, OffsetSeriesUsesTemperatureSeries) {
    CrossWlfParams rp;
    NozzleGeometry g{0.2, 10.0};
    auto lut = std::make_shared<PressureFlowLut>();
    lut->build(rp, g, {1.0, 2.0, 4.0, 8.0}, {180.0, 200.0, 220.0, 240.0});
    FilamentGeometry fil{1.75};
    CrossWlfPressureAdvance::Params pp;
    pp.compressibilityOverArea = 1e-5;
    pp.smoothTime = 0.0;
    pp.maxCompensation = 100.0;
    CrossWlfPressureAdvance model(lut, pp, fil);
    std::vector<double> vel{1.0, 5.0, 1.0};
    std::vector<double> T{240.0, 180.0, 240.0};
    auto off = model.offsetSeries(vel, T, 0.001);
    EXPECT_EQ(off.size(), vel.size());
    // Middle sample (high flow, cold) should be the largest.
    EXPECT_GT(off[1], off[0]);
    EXPECT_GT(off[1], off[2]);
}
