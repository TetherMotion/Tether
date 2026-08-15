/**
 * @file test_power_law_rheology.cpp
 * @brief Unit tests for the power-law rheology and the corrected C_n.
 *
 * Verifies:
 *  - Newtonian limit (n=1, K_p=η) reproduces Hagen-Poiseuille ΔP = 8ηLQ/(πR⁴).
 *  - Monotonicity of pressure in flow.
 *  - Inversion round-trip (pressure ↔ flow).
 *  - Shear-thinning (n<1) gives lower pressure than the Newtonian estimate at
 *    the same K_p (because the apparent viscosity drops with shear rate).
 */

#include <gtest/gtest.h>
#include <cmath>

#include "tether/control/extrusion/PowerLawRheology.hpp"
#include "tether/control/extrusion/CrossWlfRheology.hpp"
#include "tether/control/extrusion/PressureFlowLut.hpp"

using namespace tether::control::extrusion;

namespace {
constexpr double kPi = M_PI;
}

// ============================================================================
// PowerLawRheology
// ============================================================================

TEST(PowerLawRheology, NewtonianLimitMatchesHagenPoiseuille) {
    PowerLawParams p{1000.0, 1.0}; // η = 1000 Pa·s, n = 1
    NozzleGeometry g{0.2, 10.0};   // R=0.2 mm, L=10 mm
    const double Q = 1.0;          // mm³/s
    const double P = PowerLawRheology::pressureFromFlow(Q, p, g);
    // Hagen-Poiseuille: ΔP = 8 η L Q / (π R⁴)
    const double expected = 8.0 * p.consistencyIndex * g.lengthMm * Q /
                            (kPi * std::pow(g.radiusMm, 4.0));
    EXPECT_NEAR(P, expected, 1e-6 * expected);
}

TEST(PowerLawRheology, ResistanceConstantNewtonian) {
    PowerLawParams p{500.0, 1.0};
    NozzleGeometry g{0.25, 8.0};
    const double Cn = PowerLawRheology::resistanceConstant(p, g);
    const double expected = 8.0 * p.consistencyIndex * g.lengthMm /
                            (kPi * std::pow(g.radiusMm, 4.0));
    EXPECT_NEAR(Cn, expected, 1e-6 * expected);
}

TEST(PowerLawRheology, PressureMonotonicInFlow) {
    PowerLawParams p{2000.0, 0.5}; // shear-thinning
    NozzleGeometry g{0.2, 10.0};
    const double P1 = PowerLawRheology::pressureFromFlow(1.0, p, g);
    const double P2 = PowerLawRheology::pressureFromFlow(2.0, p, g);
    const double P3 = PowerLawRheology::pressureFromFlow(5.0, p, g);
    EXPECT_GT(P2, P1);
    EXPECT_GT(P3, P2);
}

TEST(PowerLawRheology, InversionRoundTrip) {
    PowerLawParams p{1500.0, 0.6};
    NozzleGeometry g{0.2, 10.0};
    for (double Q : {0.5, 1.0, 2.0, 5.0}) {
        const double P = PowerLawRheology::pressureFromFlow(Q, p, g);
        const double Qr = PowerLawRheology::flowFromPressure(P, p, g);
        EXPECT_NEAR(Qr, Q, 1e-6 * Q);
    }
}

TEST(PowerLawRheology, ZeroFlowGivesZeroPressure) {
    PowerLawParams p{1000.0, 0.5};
    NozzleGeometry g{0.2, 10.0};
    EXPECT_EQ(PowerLawRheology::pressureFromFlow(0.0, p, g), 0.0);
    EXPECT_EQ(PowerLawRheology::pressureFromFlow(-1.0, p, g), 0.0);
}

TEST(PowerLawRheology, ShearThinningLowerPressureThanNewtonianSameK) {
    // At the same K_p and Q, a shear-thinning fluid (n<1) has a lower
    // apparent viscosity at the wall shear rate, hence lower pressure than
    // the Newtonian prediction at the same K_p (which is not the same fluid,
    // but a useful sanity check on the exponent direction).
    NozzleGeometry g{0.2, 10.0};
    PowerLawParams newton{1000.0, 1.0};
    PowerLawParams thin{1000.0, 0.5};
    const double Q = 5.0;
    const double Pn = PowerLawRheology::pressureFromFlow(Q, newton, g);
    const double Pt = PowerLawRheology::pressureFromFlow(Q, thin, g);
    // P = C_n Q^n. For n<1, Q^n grows sub-linearly, but C_n also changes.
    // Empirically for these params the shear-thinning pressure is lower.
    EXPECT_LT(Pt, Pn);
}

// ============================================================================
// CrossWlfRheology
// ============================================================================

TEST(CrossWlfRheology, ZeroShearViscosityMatchesRefAtRefTemp) {
    CrossWlfParams p;
    EXPECT_NEAR(CrossWlfRheology::zeroShearViscosity(p.refTempC, p),
                p.zeroShearViscosityRef, 1e-6 * p.zeroShearViscosityRef);
}

TEST(CrossWlfRheology, ViscosityDecreasesWithTemperature) {
    CrossWlfParams p;
    const double etaLow = CrossWlfRheology::viscosity(10.0, 180.0, p);
    const double etaHigh = CrossWlfRheology::viscosity(10.0, 240.0, p);
    EXPECT_LT(etaHigh, etaLow);
}

TEST(CrossWlfRheology, ViscosityDecreasesWithShearRate) {
    CrossWlfParams p;
    const double eta0 = CrossWlfRheology::viscosity(1e-3, 210.0, p);
    const double etaHigh = CrossWlfRheology::viscosity(1e3, 210.0, p);
    EXPECT_LT(etaHigh, eta0);
}

TEST(CrossWlfRheology, PressureFromFlowMonotonicAndPositive) {
    CrossWlfParams p;
    NozzleGeometry g{0.2, 10.0};
    const double P1 = CrossWlfRheology::pressureFromFlow(1.0, 210.0, p, g);
    const double P2 = CrossWlfRheology::pressureFromFlow(3.0, 210.0, p, g);
    EXPECT_GT(P1, 0.0);
    EXPECT_GT(P2, P1);
}

TEST(CrossWlfRheology, PressureDecreasesWithTemperature) {
    CrossWlfParams p;
    NozzleGeometry g{0.2, 10.0};
    const double Pcold = CrossWlfRheology::pressureFromFlow(3.0, 180.0, p, g);
    const double Phot = CrossWlfRheology::pressureFromFlow(3.0, 240.0, p, g);
    EXPECT_LT(Phot, Pcold);
}

TEST(CrossWlfRheology, InversionRoundTrip) {
    CrossWlfParams p;
    NozzleGeometry g{0.2, 10.0};
    const double Q = 2.0;
    const double P = CrossWlfRheology::pressureFromFlow(Q, 210.0, p, g);
    const double Qr = CrossWlfRheology::flowFromPressure(P, 210.0, p, g);
    // Numerical inversion is less precise; allow 5% tolerance.
    EXPECT_NEAR(Qr, Q, 0.05 * Q);
}

// ============================================================================
// PressureFlowLut
// ============================================================================

TEST(PressureFlowLut, BuildAndInterpolateMatchesRheology) {
    CrossWlfParams p;
    NozzleGeometry g{0.2, 10.0};
    PressureFlowLut lut;
    lut.build(p, g,
              {0.5, 1.0, 2.0, 4.0, 8.0},          // flow axis
              {180.0, 200.0, 220.0, 240.0});     // temp axis
    ASSERT_FALSE(lut.empty());
    // At a grid node, the LUT must match the rheology exactly.
    const double Q = 2.0, T = 220.0;
    const double Plut = lut.pressure(Q, T);
    const double Prheo = CrossWlfRheology::pressureFromFlow(Q, T, p, g);
    EXPECT_NEAR(Plut, Prheo, 1e-6 * Prheo);
}

TEST(PressureFlowLut, BilinearInterpolationMidpoint) {
    CrossWlfParams p;
    NozzleGeometry g{0.2, 10.0};
    PressureFlowLut lut;
    lut.build(p, g, {1.0, 3.0}, {200.0, 240.0});
    // At the geometric center, bilinear interpolation should be the average
    // of the four corners.
    const double Qmid = 2.0, Tmid = 220.0;
    const double v00 = CrossWlfRheology::pressureFromFlow(1.0, 200.0, p, g);
    const double v01 = CrossWlfRheology::pressureFromFlow(3.0, 200.0, p, g);
    const double v10 = CrossWlfRheology::pressureFromFlow(1.0, 240.0, p, g);
    const double v11 = CrossWlfRheology::pressureFromFlow(3.0, 240.0, p, g);
    const double expected = 0.25 * (v00 + v01 + v10 + v11);
    EXPECT_NEAR(lut.pressure(Qmid, Tmid), expected, 1e-6 * expected);
}

TEST(PressureFlowLut, ClampsOutOfRange) {
    CrossWlfParams p;
    NozzleGeometry g{0.2, 10.0};
    PressureFlowLut lut;
    lut.build(p, g, {1.0, 3.0}, {200.0, 240.0});
    // Below the flow axis → returns the lowest corner.
    EXPECT_NEAR(lut.pressure(0.0, 200.0),
                CrossWlfRheology::pressureFromFlow(1.0, 200.0, p, g),
                1e-6);
    // Above the temp axis → returns the highest-T row.
    EXPECT_NEAR(lut.pressure(2.0, 999.0),
                lut.pressure(2.0, 240.0), 1e-6);
}

TEST(PressureFlowLut, AssignDirectly) {
    PressureFlowLut lut;
    std::vector<double> f{1.0, 2.0};
    std::vector<double> t{100.0, 200.0};
    std::vector<double> v{10.0, 20.0, 30.0, 40.0}; // t-major
    lut.assign(f, t, v);
    ASSERT_FALSE(lut.empty());
    EXPECT_NEAR(lut.pressure(1.0, 100.0), 10.0, 1e-9);
    EXPECT_NEAR(lut.pressure(2.0, 200.0), 40.0, 1e-9);
    EXPECT_NEAR(lut.pressure(1.5, 150.0), 25.0, 1e-9); // bilinear midpoint
}

TEST(PressureFlowLut, EmptyLutReturnsZero) {
    PressureFlowLut lut;
    EXPECT_EQ(lut.pressure(1.0, 200.0), 0.0);
}
