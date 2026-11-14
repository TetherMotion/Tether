/**
 * @file OvenGradientControllerTests.cpp
 * @brief Tests for the multi-zone temperature-gradient layer:
 *        TemperatureProfile::transformed, GradientSpec offset/shift
 *        generation + maxZoneDeltaC clamping, setpoint delta clamping,
 *        coupling-disturbance injection, gradient alarm, and closed-loop
 *        gradient tracking on a coupled 3-zone plant (MultiZoneOven
 *        equations).
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tether/control/thermal/OvenGradientController.hpp"
#include "tether/control/thermal/OvenThermalModel.hpp"
#include "tether/control/thermal/TemperatureProfile.hpp"
#include "tether/simulation/systems/thermal/MultiZoneOven.hpp"

namespace {

using namespace tether::control::thermal;

// Per-zone FOPDT equivalents of the MultiZoneOven plant
// (C=3000 J/K, eta=0.9, Pmax=3000 W -> alpha=0.9 C/s/u; ambient loss
// beta_i = 1/(R_a_i*C)). Coupling gammas = 1/(R_ij*C_i).
OvenModelParams zoneParams(double beta) {
    OvenModelParams p;
    p.alpha = 0.9;
    p.beta = beta;
    p.delay = 1.0;
    p.ambient = 25.0;
    p.uMin = 0.0;
    p.uMax = 1.0;
    return p;
}

GradientZoneConfig zoneCfg(double beta, double gL, double gR) {
    GradientZoneConfig z;
    z.controller.model = zoneParams(beta);
    z.controller.adaptParams = false;   // fixed plant: deterministic test
    z.controller.ditherAmp = 0.0;
    z.couplingLeft = gL;
    z.couplingRight = gR;
    return z;
}

// 3-zone coupled plant identical to Simulation::MultiZoneOven.
// u in [0,1] maps to Q = u*Pmax [W].
struct CoupledPlant {
    Simulation::MultiZoneOven sys;
    static constexpr double Pmax = 3000.0;
    std::vector<double> step(const std::vector<double>& u, double dt) {
        Simulation::StateVector s{T[0], T[1], T[2]};
        Simulation::StateVector in{u[0] * Pmax, u[1] * Pmax, u[2] * Pmax};
        auto ds = sys.dynamics(0.0, s, in);
        for (int i = 0; i < 3; ++i) T[i] += ds[i] * dt;
        return {T[0], T[1], T[2]};
    }
    double T[3]{25.0, 25.0, 25.0};
};

} // namespace

// ---------- TemperatureProfile::transformed ----------

TEST(OvenGradient, TransformedOffsetAndShift) {
    TemperatureProfile base(25.0);
    base.addRampSoak(100.0, 0.5, 100.0);   // ramp 25->100 in 150 s + soak
    base.addSoak(50.0);

    auto z = base.transformed(10.0, 20.0);
    EXPECT_NEAR(z.value(0.0), 35.0, 1e-9);    // holds initial+offset
    EXPECT_NEAR(z.value(20.0), 35.0, 1e-9);   // still holding pre-shift
    EXPECT_NEAR(z.value(20.0 + 75.0), 35.0 + 37.5, 1e-6); // mid ramp
    EXPECT_NEAR(z.value(20.0 + 150.0), 110.0, 1e-9);      // ramp end+off
    EXPECT_NEAR(z.duration(), base.duration() + 20.0, 1e-9);
    EXPECT_NEAR(z.derivative(20.0 + 75.0), 0.5, 1e-9);
    EXPECT_NEAR(z.derivative(5.0), 0.0, 1e-9);  // pre-shift: no rate
}

// ---------- GradientSpec generation + clamping ----------

TEST(OvenGradient, SpecBuildsTransformedZoneProfiles) {
    OvenGradientController gc;
    gc.addZone(zoneCfg(3.3e-4, 0.0, 6.7e-4));
    gc.addZone(zoneCfg(2.2e-4, 6.7e-4, 6.7e-4));
    gc.addZone(zoneCfg(3.3e-4, 6.7e-4, 0.0));

    TemperatureProfile base(25.0);
    base.addRampSoak(120.0, 0.4, 60.0);

    GradientSpec spec;
    spec.base = base;
    spec.tempOffsetC = {0.0, 10.0, 20.0};
    spec.timeShiftS = {0.0, 30.0, 60.0};
    spec.maxZoneDeltaC = 15.0;

    ASSERT_TRUE(gc.setGradientProfile(spec));
    // Zone 2's governed reference at t=0 should be initial+offset.
    auto u = gc.compute(std::vector<double>{25.0, 25.0, 25.0}, 0.5);
    ASSERT_EQ(u.size(), 3);
    const auto& st = gc.zoneStatus();
    EXPECT_NEAR(st[0].controller.reference, 25.0, 0.5);
    EXPECT_NEAR(st[1].controller.reference, 35.0, 0.5);
    EXPECT_NEAR(st[2].controller.reference, 45.0, 0.5);
    EXPECT_NEAR(gc.maxSetpointDelta(), 10.0, 1.0);
}

TEST(OvenGradient, SpecClampsAdjacentOffsets) {
    OvenGradientController gc;
    for (int i = 0; i < 3; ++i) gc.addZone(zoneCfg(3.0e-4, 0.0, 0.0));

    TemperatureProfile base(25.0);
    base.addSoak(10.0);

    GradientSpec spec;
    spec.base = base;
    spec.tempOffsetC = {0.0, 25.0, 50.0};   // deltas 25, 25 > limit 15
    spec.maxZoneDeltaC = 15.0;
    ASSERT_TRUE(gc.setGradientProfile(spec));

    gc.compute(std::vector<double>{25.0, 25.0, 25.0}, 0.5);
    const auto& st = gc.zoneStatus();
    // Forward clamp: off = {0, 15, 30}.
    EXPECT_NEAR(st[0].controller.reference, 25.0, 0.6);
    EXPECT_NEAR(st[1].controller.reference, 40.0, 0.6);
    EXPECT_NEAR(st[2].controller.reference, 55.0, 0.6);
    EXPECT_LE(gc.maxSetpointDelta(), 15.5);
}

TEST(OvenGradient, SetpointsClampAdjacentDelta) {
    OvenGradientController gc;
    for (int i = 0; i < 3; ++i) gc.addZone(zoneCfg(3.0e-4, 0.0, 0.0));
    gc.setZoneSetpoints({100.0, 200.0, 250.0}, 15.0);
    gc.compute(std::vector<double>{25.0, 25.0, 25.0}, 0.5);
    const auto& st = gc.zoneStatus();
    // {100, 115, 130}: each within 15 of previous.
    EXPECT_NEAR(st[0].controller.reference, 100.0, 0.6);
    EXPECT_NEAR(st[1].controller.reference, 115.0, 0.6);
    EXPECT_NEAR(st[2].controller.reference, 130.0, 0.6);
}

// ---------- Coupling injection ----------

TEST(OvenGradient, CouplingDisturbanceInjected) {
    OvenGradientController gc;
    gc.addZone(zoneCfg(3.0e-4, 0.0, 1e-3));    // zone0: only right nbr
    gc.addZone(zoneCfg(3.0e-4, 1e-3, 0.0));    // zone1: only left nbr

    gc.setZoneSetpoints({100.0, 100.0});
    // Steady unequal temps -> coupling flows hot->cold.
    gc.compute(std::vector<double>{120.0, 80.0}, 0.5);
    const auto& st = gc.zoneStatus();
    // zone0 gains from zone1? zone1 is colder: d0 = gR*(80-120) < 0
    EXPECT_NEAR(st[0].couplingDisturb, 1e-3 * (80.0 - 120.0), 1e-9);
    // zone1 gains from hotter zone0: d1 = gL*(120-80) > 0
    EXPECT_NEAR(st[1].couplingDisturb, 1e-3 * (120.0 - 80.0), 1e-9);
}

// ---------- Closed-loop gradient tracking on coupled plant ----------

TEST(OvenGradient, GradientTrackingOnCoupledPlant) {
    OvenGradientController gc;
    // gamma = 1/(R*C): R12=R23=0.5 K/W, C=3000 -> 6.67e-4 1/s
    const double g = 1.0 / (0.5 * 3000.0);
    gc.addZone(zoneCfg(1.0 / (1.0 * 3000.0), 0.0, g));   // zone0
    gc.addZone(zoneCfg(1.0 / (1.5 * 3000.0), g, g));      // zone1
    gc.addZone(zoneCfg(1.0 / (1.0 * 3000.0), g, 0.0));    // zone2
    gc.setGradientAlarmLimit(25.0);

    TemperatureProfile base(25.0);
    base.addRampSoak(100.0, 0.25, 400.0);  // 25->100 in 300 s, soak 400 s

    GradientSpec spec;
    spec.base = base;
    spec.tempOffsetC = {0.0, 8.0, 16.0};
    spec.timeShiftS = {0.0, 20.0, 40.0};
    spec.maxZoneDeltaC = 15.0;
    ASSERT_TRUE(gc.setGradientProfile(spec));

    CoupledPlant plant;
    const double dt = 0.5;
    const int steps = int((base.duration() + 80.0) / dt);
    std::vector<double> T = {25.0, 25.0, 25.0};
    std::vector<double> u;
    for (int k = 0; k < steps; ++k) {
        u = gc.compute(T, dt);
        ASSERT_EQ(u.size(), 3);
        T = plant.step(u, dt);
    }

    // Final soak: each zone near its offset reference.
    const auto& st = gc.zoneStatus();
    const double refs[3] = {100.0, 108.0, 116.0};
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(T[i], refs[i], 6.0)
            << "zone " << i << " final " << T[i] << " ref " << refs[i];
        EXPECT_NEAR(st[i].controller.output, u[i], 1e-9);
    }
    // Gradient stayed bounded through the run.
    EXPECT_LE(gc.maxMeasuredDelta(), 30.0);
}

TEST(OvenGradient, AlarmOnExcessiveMeasuredDelta) {
    OvenGradientController gc;
    gc.addZone(zoneCfg(3.0e-4, 0.0, 0.0));
    gc.addZone(zoneCfg(3.0e-4, 0.0, 0.0));
    gc.setZoneSetpoints({100.0, 100.0});
    gc.setGradientAlarmLimit(10.0);

    EXPECT_FALSE(gc.gradientAlarm());
    gc.compute(std::vector<double>{100.0, 105.0}, 0.5);   // delta 5 < 10
    EXPECT_FALSE(gc.gradientAlarm());
    gc.compute(std::vector<double>{100.0, 112.0}, 0.5);   // delta 12 > 10
    EXPECT_TRUE(gc.gradientAlarm());
    EXPECT_NEAR(gc.maxMeasuredDelta(), 12.0, 1e-9);
}

TEST(OvenGradient, SizeMismatchRejected) {
    OvenGradientController gc;
    gc.addZone(zoneCfg(3.0e-4, 0.0, 0.0));
    gc.addZone(zoneCfg(3.0e-4, 0.0, 0.0));
    EXPECT_TRUE(gc.compute(std::vector<double>{25.0, 25.0}, 0.5).size() == 2);
    EXPECT_TRUE(gc.compute(std::vector<double>{25.0}, 0.5).empty());
    double out[2];
    EXPECT_FALSE(gc.computeInto(std::vector<double>{25.0}, 0.5, out));
    EXPECT_TRUE(gc.computeInto(std::vector<double>{25.0, 26.0}, 0.5, out));
    EXPECT_GT(out[0] + out[1], -1e-9);
}
