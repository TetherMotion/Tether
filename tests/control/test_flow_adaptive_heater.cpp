/**
 * @file test_flow_adaptive_heater.cpp
 * @brief Unit tests for the flow-adaptive heater controller and melt-zone
 *        thermal observer.
 *
 * Verifies:
 *  - MeltZoneThermalObserver: steady-state melt temp drops with flow,
 *    recovers when flow stops; enthalpy term dominates at high flow.
 *  - FlowAdaptiveHeaterController: pre-emphasis adds power at flow onset,
 *    bounded by maxPreEmphasisPower; post-emphasis decays after flow stops;
 *    closed-loop with a simple first-order plant reaches target.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "tether/control/extrusion/MeltZoneThermalObserver.hpp"
#include "tether/control/extrusion/FlowAdaptiveHeaterController.hpp"

using namespace tether::control::extrusion;

// ============================================================================
// MeltZoneThermalObserver
// ============================================================================

TEST(MeltZoneThermalObserver, SteadyStateNoFlowMatchesHeaterBlock) {
    MeltZoneThermalParams p;
    p.heaterBlockCapacitance = 8.0;
    p.meltZoneCapacitance = 2.0;
    p.heaterMeltConductance = 0.8;
    p.filamentHeatCapacity = 2.1;
    p.inletTempC = 25.0;
    p.heaterPowerScale = 40.0;
    MeltZoneThermalObserver obs(p);
    obs.initialize(200.0, 200.0);
    // No flow, zero heater power, both states equal → no gradients, no
    // enthalpy removal → temperatures stay at 200.
    for (int i = 0; i < 1000; ++i) obs.update(0.0, 0.0, 0.01);
    EXPECT_NEAR(obs.heaterBlockTemp(), 200.0, 1e-6);
    EXPECT_NEAR(obs.meltTempEst(), 200.0, 1e-6);
    // With a gradient (heater block hotter than melt) but no flow, the
    // melt relaxes toward the heater block.
    obs.initialize(220.0, 200.0);
    for (int i = 0; i < 5000; ++i) obs.update(0.0, 0.0, 0.01);
    EXPECT_GT(obs.meltTempEst(), 200.0);
    EXPECT_LT(obs.heaterBlockTemp(), 220.0);
}

TEST(MeltZoneThermalObserver, FlowCoolsMeltZone) {
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    obs.initialize(220.0, 220.0);
    const double T0 = obs.meltTempEst();
    // Apply flow for a while with no heater power.
    for (int i = 0; i < 500; ++i) obs.update(0.0, 5.0, 0.01);
    EXPECT_LT(obs.meltTempEst(), T0);
}

TEST(MeltZoneThermalObserver, HeaterPowerRaisesTemperature) {
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    obs.initialize(25.0, 25.0);
    const double T0 = obs.meltTempEst();
    for (int i = 0; i < 500; ++i) obs.update(0.5, 0.0, 0.01);
    EXPECT_GT(obs.heaterBlockTemp(), T0);
    EXPECT_GT(obs.meltTempEst(), T0);
}

TEST(MeltZoneThermalObserver, ResetReturnsToInlet) {
    MeltZoneThermalParams p;
    p.inletTempC = 30.0;
    MeltZoneThermalObserver obs(p);
    obs.initialize(200.0, 200.0);
    obs.reset();
    EXPECT_NEAR(obs.heaterBlockTemp(), 30.0, 1e-9);
    EXPECT_NEAR(obs.meltTempEst(), 30.0, 1e-9);
}

// ============================================================================
// FlowAdaptiveHeaterController
// ============================================================================

TEST(FlowAdaptiveHeaterController, PreEmphasisAddsPowerAtFlowOnset) {
    FlowAdaptiveHeaterParams p;
    p.maxPreEmphasisPower = 0.4;
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(0.0, 0.0, 0.0); // disable PID to isolate feed-forward
    ::Control::ControllerInput in{};
    in.reference = 220.0;
    in.measured = 220.0; // at target → pre-emphasis allowed
    in.dt = 0.1;
    // No flow → no pre-emphasis.
    ctrl.setFlow(0.0);
    auto out0 = ctrl.compute(in);
    EXPECT_NEAR(out0.feedforward, 0.0, 1e-6);
    // With flow → pre-emphasis kicks in.
    ctrl.setFlow(5.0);
    auto outQ = ctrl.compute(in);
    EXPECT_GT(outQ.feedforward, 0.0);
    EXPECT_LE(outQ.feedforward, p.maxPreEmphasisPower + 1e-6);
}

TEST(FlowAdaptiveHeaterController, PreEmphasisBoundedByMax) {
    FlowAdaptiveHeaterParams p;
    p.maxPreEmphasisPower = 0.1;
    p.heaterPowerScale = 1.0; // tiny so P_ff_pwm is huge → clamp
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(0.0, 0.0, 0.0);
    ::Control::ControllerInput in{};
    in.reference = 220.0;
    in.measured = 220.0;
    in.dt = 0.1;
    ctrl.setFlow(100.0);
    auto out = ctrl.compute(in);
    EXPECT_LE(out.feedforward, p.maxPreEmphasisPower + 1e-6);
}

TEST(FlowAdaptiveHeaterController, PreEmphasisSuppressedDuringWarmup) {
    FlowAdaptiveHeaterParams p;
    p.maxHeaterOvershoot = 10.0;
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(0.0, 0.0, 0.0);
    ::Control::ControllerInput in{};
    in.reference = 220.0;
    in.measured = 25.0; // far from target
    in.dt = 0.1;
    ctrl.setFlow(5.0);
    auto out = ctrl.compute(in);
    // During warmup, pre-emphasis is suppressed (post-emphasis may still be 0).
    EXPECT_NEAR(out.feedforward, 0.0, 1e-6);
}

TEST(FlowAdaptiveHeaterController, PostEmphasisDecaysAfterFlowStops) {
    FlowAdaptiveHeaterParams p;
    p.debtTimeConstant = 1.0;
    p.maxPostEmphasisPower = 0.5;
    p.heaterPowerScale = 40.0;
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(0.0, 0.0, 0.0);
    ::Control::ControllerInput in{};
    in.reference = 220.0;
    in.measured = 220.0;
    in.dt = 0.1;
    // Run with flow to build up thermal debt.
    ctrl.setFlow(5.0);
    for (int i = 0; i < 50; ++i) ctrl.compute(in);
    const double postDuring = ctrl.emphasis().postEmphasisPwm;
    // Stop flow; post-emphasis should appear then decay.
    ctrl.setFlow(0.0);
    double postFirst = 0.0;
    for (int i = 0; i < 5; ++i) {
        auto out = ctrl.compute(in);
        postFirst = std::max(postFirst, out.feedforward);
    }
    EXPECT_GT(postFirst, 0.0); // post-emphasis appears after flow stops
    // Continue running; feed-forward should decay toward 0.
    for (int i = 0; i < 500; ++i) ctrl.compute(in);
    EXPECT_LT(ctrl.emphasis().postEmphasisPwm, postFirst);
    (void)postDuring;
}

TEST(FlowAdaptiveHeaterController, ClosedLoopReachesTarget) {
    // Simple first-order plant: C dT/dt = P_pwm * P_scale - loss
    // C = 8 J/K, P_scale = 120 W, loss = 0.5 W/K to ambient (25°C).
    // Steady-state at full power: 120 = 0.5*(T-25) → T = 265°C, so 200 is reachable.
    FlowAdaptiveHeaterParams p;
    p.heaterPowerScale = 120.0;
    p.maxHeaterOvershoot = 30.0;
    p.maxPreEmphasisPower = 0.5;
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(2.0, 0.5, 0.0);
    double T = 25.0;
    const double C = 8.0;
    const double loss = 0.5;
    const double ambient = 25.0;
    ::Control::ControllerInput in{};
    in.reference = 200.0;
    in.dt = 0.1;
    ctrl.setFlow(0.0);
    for (int i = 0; i < 5000; ++i) {
        in.measured = T;
        auto out = ctrl.compute(in);
        const double Pw = std::clamp(out.control, 0.0, 1.0) * p.heaterPowerScale;
        T += (Pw - loss * (T - ambient)) / C * in.dt;
    }
    EXPECT_NEAR(T, 200.0, 5.0);
}

TEST(FlowAdaptiveHeaterController, ResetClearsState) {
    FlowAdaptiveHeaterController ctrl;
    ctrl.setGains(1.0, 0.1, 0.0);
    ::Control::ControllerInput in{};
    in.reference = 200.0;
    in.measured = 25.0;
    in.dt = 0.1;
    ctrl.setFlow(5.0);
    ctrl.compute(in);
    ctrl.reset();
    EXPECT_NEAR(ctrl.flow(), 0.0, 1e-9);
    EXPECT_NEAR(ctrl.emphasis().preEmphasisPwm, 0.0, 1e-9);
    EXPECT_NEAR(ctrl.emphasis().thermalDebt, 0.0, 1e-9);
}
