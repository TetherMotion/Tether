/**
 * @file test_flow_adaptive_heater.cpp
 * @brief Unit tests for the three-state flow-adaptive heater controller and
 *        melt-zone thermal observer.
 *
 * Verifies:
 *  - MeltZoneThermalObserver (three-state):
 *    - Steady-state with no flow: all three states equal → no gradients.
 *    - Flow cools melt zone first, sensor second, heater block last.
 *    - Heater power raises all temperatures.
 *    - Luenberger correction converges observer to real sensor reading.
 *    - Reset returns all states to inlet temperature.
 *  - FlowAdaptiveHeaterController:
 *    - Pre-emphasis adds power at flow onset, bounded by maxPreEmphasisPower.
 *    - Pre-emphasis suppressed during warmup.
 *    - Post-emphasis decays after flow stops.
 *    - Sensor coupling alpha reduces feed-forward (PID covers some).
 *    - Closed-loop with a simple plant reaches target.
 *    - Reset clears state.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "tether/control/extrusion/MeltZoneThermalObserver.hpp"
#include "tether/control/extrusion/FlowAdaptiveHeaterController.hpp"

using namespace tether::control::extrusion;

// ============================================================================
// MeltZoneThermalObserver (three-state)
// ============================================================================

TEST(MeltZoneThermalObserver, SteadyStateNoFlowAllStatesEqual) {
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    obs.initialize(200.0);
    // No flow, zero heater power, all states equal → no gradients.
    for (int i = 0; i < 1000; ++i) obs.update(0.0, 0.0, 0.01);
    EXPECT_NEAR(obs.heaterBlockTemp(), 200.0, 1e-6);
    EXPECT_NEAR(obs.sensorTemp(), 200.0, 1e-6);
    EXPECT_NEAR(obs.meltTempEst(), 200.0, 1e-6);
}

TEST(MeltZoneThermalObserver, GradientRelaxesTowardHeater) {
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    // Heater block hotter than sensor, sensor hotter than melt.
    obs.initialize(220.0, 210.0, 200.0);
    for (int i = 0; i < 5000; ++i) obs.update(0.0, 0.0, 0.01);
    // All states should converge toward each other (no heat source, no flow).
    EXPECT_GT(obs.meltTempEst(), 200.0);
    EXPECT_LT(obs.heaterBlockTemp(), 220.0);
    // At equilibrium all three should be equal.
    EXPECT_NEAR(obs.heaterBlockTemp(), obs.sensorTemp(), 1e-3);
    EXPECT_NEAR(obs.sensorTemp(), obs.meltTempEst(), 1e-3);
}

TEST(MeltZoneThermalObserver, FlowCoolsMeltZoneFirst) {
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    obs.initialize(220.0);
    // Apply flow with no heater power.
    // After a short time, melt should be coolest, sensor in middle, heater hottest.
    for (int i = 0; i < 100; ++i) obs.update(0.0, 5.0, 0.01);
    EXPECT_LT(obs.meltTempEst(), obs.sensorTemp());
    EXPECT_LT(obs.sensorTemp(), obs.heaterBlockTemp());
}

TEST(MeltZoneThermalObserver, HeaterPowerRaisesAllTemperatures) {
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    obs.initialize(25.0);
    const double T0 = obs.meltTempEst();
    for (int i = 0; i < 500; ++i) obs.update(0.5, 0.0, 0.01);
    EXPECT_GT(obs.heaterBlockTemp(), T0);
    EXPECT_GT(obs.sensorTemp(), T0);
    EXPECT_GT(obs.meltTempEst(), T0);
    // Heater block should be hottest (directly heated).
    EXPECT_GT(obs.heaterBlockTemp(), obs.sensorTemp());
    EXPECT_GT(obs.sensorTemp(), obs.meltTempEst());
}

TEST(MeltZoneThermalObserver, LuenbergerCorrectionConverges) {
    MeltZoneThermalParams p;
    p.luenbergerGainSensor = 5.0; // aggressive correction
    MeltZoneThermalObserver obs(p);
    obs.initialize(200.0);
    // Simulate a real sensor reading that differs from the model.
    // Feed measurement = 210°C while the model starts at 200°C.
    for (int i = 0; i < 500; ++i) {
        obs.updateWithMeasurement(0.0, 0.0, 210.0, 0.01);
    }
    // The sensor state should converge toward the measurement.
    EXPECT_NEAR(obs.sensorTemp(), 210.0, 1.0);
    // The heater and melt states should also be pulled along.
    EXPECT_GT(obs.heaterBlockTemp(), 200.0);
    EXPECT_GT(obs.meltTempEst(), 200.0);
}

TEST(MeltZoneThermalObserver, ResetReturnsToInlet) {
    MeltZoneThermalParams p;
    p.inletTempC = 30.0;
    MeltZoneThermalObserver obs(p);
    obs.initialize(200.0);
    obs.reset();
    EXPECT_NEAR(obs.heaterBlockTemp(), 30.0, 1e-9);
    EXPECT_NEAR(obs.sensorTemp(), 30.0, 1e-9);
    EXPECT_NEAR(obs.meltTempEst(), 30.0, 1e-9);
}

TEST(MeltZoneThermalObserver, SensorSeesMeltDisturbanceBeforeHeaterBlock) {
    // When cold plastic flows, the sensor should drop before the heater block.
    MeltZoneThermalParams p;
    MeltZoneThermalObserver obs(p);
    obs.initialize(220.0);
    // Apply flow for a very short time (10 steps = 0.1s).
    obs.update(0.0, 10.0, 0.01);
    obs.update(0.0, 10.0, 0.01);
    obs.update(0.0, 10.0, 0.01);
    // After just 3 steps, the melt zone has cooled, the sensor has started
    // to cool, but the heater block hasn't changed much yet.
    const double dT_melt = 220.0 - obs.meltTempEst();
    const double dT_sensor = 220.0 - obs.sensorTemp();
    const double dT_heater = 220.0 - obs.heaterBlockTemp();
    // Melt cools most, sensor second, heater block least.
    EXPECT_GT(dT_melt, dT_sensor);
    EXPECT_GT(dT_sensor, dT_heater);
}

// ============================================================================
// FlowAdaptiveHeaterController
// ============================================================================

TEST(FlowAdaptiveHeaterController, PreEmphasisAddsPowerAtFlowOnset) {
    FlowAdaptiveHeaterParams p;
    p.maxPreEmphasisPower = 0.4;
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(0.0, 0.0, 0.0); // disable PID to isolate feed-forward
    ::tether::control::ControllerInput in{};
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
    ::tether::control::ControllerInput in{};
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
    ::tether::control::ControllerInput in{};
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
    ::tether::control::ControllerInput in{};
    in.reference = 220.0;
    in.measured = 220.0;
    in.dt = 0.1;
    // Run with flow to build up thermal debt.
    ctrl.setFlow(5.0);
    for (int i = 0; i < 50; ++i) ctrl.compute(in);
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
    EXPECT_LT(ctrl.emphasis().postEmphasisPWM, postFirst);
}

TEST(FlowAdaptiveHeaterController, SensorCouplingAlphaReducesFeedForward) {
    // With a high alpha (G_sm >> G_hs), the PID sees most of the disturbance,
    // so the feed-forward should be small. With a low alpha, feed-forward
    // should be large.
    FlowAdaptiveHeaterParams pHigh;
    pHigh.heaterSensorConductance = 0.1; // small G_hs
    pHigh.sensorMeltConductance = 10.0;  // large G_sm → α ≈ 0.99
    pHigh.heaterPowerScale = 40.0;
    pHigh.maxPreEmphasisPower = 1.0; // generous limit
    FlowAdaptiveHeaterController ctrlHigh(pHigh);
    ctrlHigh.setGains(0.0, 0.0, 0.0);

    FlowAdaptiveHeaterParams pLow;
    pLow.heaterSensorConductance = 10.0;  // large G_hs
    pLow.sensorMeltConductance = 0.1;     // small G_sm → α ≈ 0.01
    pLow.heaterPowerScale = 40.0;
    pLow.maxPreEmphasisPower = 1.0;
    FlowAdaptiveHeaterController ctrlLow(pLow);
    ctrlLow.setGains(0.0, 0.0, 0.0);

    ::tether::control::ControllerInput in{};
    in.reference = 220.0;
    in.measured = 220.0;
    in.dt = 0.1;
    ctrlHigh.setFlow(5.0);
    ctrlLow.setFlow(5.0);
    auto outHigh = ctrlHigh.compute(in); // high α → low feed-forward
    auto outLow = ctrlLow.compute(in);   // low α → high feed-forward
    EXPECT_LT(outHigh.feedforward, outLow.feedforward);
    // Also verify the alpha diagnostic.
    EXPECT_GT(ctrlHigh.emphasis().sensorCouplingAlpha,
              ctrlLow.emphasis().sensorCouplingAlpha);
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
    ::tether::control::ControllerInput in{};
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
    ::tether::control::ControllerInput in{};
    in.reference = 200.0;
    in.measured = 25.0;
    in.dt = 0.1;
    ctrl.setFlow(5.0);
    ctrl.compute(in);
    ctrl.reset();
    EXPECT_NEAR(ctrl.flow(), 0.0, 1e-9);
    EXPECT_NEAR(ctrl.emphasis().preEmphasisPWM, 0.0, 1e-9);
    EXPECT_NEAR(ctrl.emphasis().thermalDebt, 0.0, 1e-9);
}

TEST(FlowAdaptiveHeaterController, ObserverTracksMeasuredSensorTemp) {
    // After many cycles, the observer's sensor state should track
    // the measured temperature closely (Luenberger correction).
    FlowAdaptiveHeaterParams p;
    p.luenbergerGainSensor = 5.0;
    FlowAdaptiveHeaterController ctrl(p);
    ctrl.setGains(2.0, 0.5, 0.0);
    ::tether::control::ControllerInput in{};
    in.reference = 200.0;
    in.measured = 200.0;
    in.dt = 0.1;
    ctrl.setFlow(0.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.compute(in);
    }
    // The observer sensor temp should be close to the measured 200°C.
    EXPECT_NEAR(ctrl.sensorTemp(), 200.0, 2.0);
}
