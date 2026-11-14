/**
 * @file OvenControllerTests.cpp
 * @brief Tests for the generic oven thermal control stack:
 *        TemperatureProfile, OvenThermalModel, ActuationGovernor,
 *        ThermalParameterEstimator, OvenAutotune, OvenController.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <deque>
#include <vector>

#include "tether/control/thermal/ActuationGovernor.hpp"
#include "tether/control/thermal/OvenAutotune.hpp"
#include "tether/control/thermal/OvenController.hpp"
#include "tether/control/thermal/OvenCoupledModel.hpp"
#include "tether/control/thermal/OvenLQIBackend.hpp"
#include "tether/control/thermal/OvenThermalModel.hpp"
#include "tether/control/thermal/TemperatureProfile.hpp"
#include "tether/control/thermal/ThermalParameterEstimator.hpp"

namespace {

using namespace tether::control;
using namespace tether::control::thermal;

// Plant used in most tests: K = alpha/beta = 200 °C/u, tau = 200 s,
// delay = 4 s. At u=1 the chamber can rise ~200 °C above ambient.
OvenModelParams plantParams() {
    OvenModelParams p;
    p.alpha = 1.0;
    p.beta = 0.005;
    p.delay = 4.0;
    p.disturbance = 0.0;
    p.ambient = 25.0;
    p.uMin = 0.0;
    p.uMax = 1.0;
    return p;
}

// 25 -> 100 @0.3 C/s (250 s), soak 200 s, 100 -> 60 @-0.2 C/s (200 s),
// soak 150 s. All rates verified feasible for plantParams().
TemperatureProfile testProfile() {
    TemperatureProfile p(25.0);
    p.addRampSoak(100.0, 0.3, 200.0);
    p.addRampSoak(60.0, 0.2, 150.0);
    return p;
}

struct SimSample {
    double t, temp, ref, u, uFf, uFb;
};

struct SimResult {
    std::vector<SimSample> samples;
    double maxAbsErr{0.0};
    double soakErr{0.0};    // mean |err| over final soak
    double finalErr{0.0};
    OvenControllerStatus lastStatus{};
};

// Run the closed loop. `plantParamsOverride` sets the TRUE plant;
// controller gets `ctrlCfg`. noiseSigma [°C], disturbanceFn(t) -> °C/s.
SimResult simulate(OvenModelParams plantP,
                   const OvenControllerConfig& ctrlCfg,
                   const TemperatureProfile& profile,
                   double duration, double dt = 0.5,
                   double noiseSigma = 0.0,
                   const std::function<double(double)>& disturbanceFn = {}) {
    OvenThermalModel plant(plantP);
    plant.setParams(plantP);
    plant.reset();

    OvenController ctrl(ctrlCfg);
    ctrl.setProfile(profile);

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, noiseSigma);

    SimResult res;
    ControllerInput in;
    in.dt = dt;
    in.reference = 25.0;  // unused in profile mode

    for (double t = 0.0; t < duration; t += dt) {
        if (disturbanceFn) plant.params().disturbance = disturbanceFn(t);
        in.measured = plant.temperature() + (noiseSigma > 0.0 ? noise(rng) : 0.0);
        const ControllerOutput out = ctrl.compute(in);
        plant.step(out.control, dt);

        const auto st = ctrl.status();
        SimSample s{t, plant.temperature(), st.reference, out.control,
                    out.feedforward, out.control - out.feedforward};
        res.samples.push_back(s);
        const double err = std::abs(s.temp - s.ref);
        res.maxAbsErr = std::max(res.maxAbsErr, err);
    }

    // Final soak: last 120 s of the profile.
    double sum = 0.0;
    int n = 0;
    for (const auto& s : res.samples) {
        if (s.t > duration - 120.0) {
            sum += std::abs(s.temp - s.ref);
            ++n;
        }
    }
    res.soakErr = n ? sum / n : 0.0;
    res.finalErr = res.samples.empty() ? 0.0
                     : std::abs(res.samples.back().temp - res.samples.back().ref);
    res.lastStatus = ctrl.status();
    return res;
}

// Max |error| within a time window.
double maxErrIn(const SimResult& r, double t0, double t1) {
    double m = 0.0;
    for (const auto& s : r.samples)
        if (s.t >= t0 && s.t <= t1) m = std::max(m, std::abs(s.temp - s.ref));
    return m;
}

// ---------------------------------------------------------------------------
// Two-state coupled plant (non-collocated sensing): the sensor measures
// zone 1 (chamber air / heater side), the controlled variable is zone 2
// (product/load). Euler integration with input dead time.
// ---------------------------------------------------------------------------

OvenCoupledParams coupledPlantParams() {
    OvenCoupledParams p;
    p.alpha = 1.2;          // heater coeff into zone 1 [°C/s/u]
    p.betaSensor = 0.008;   // zone-1 ambient loss [1/s]
    p.betaProduct = 0.002;  // zone-2 ambient loss [1/s]
    p.gammaSP = 0.02;       // coupling zone1->zone2 [1/s]
    p.gammaPS = 0.01;       // coupling zone2->zone1 [1/s]
    p.delay = 4.0;
    p.disturbance = 0.0;
    p.ambient = 25.0;
    p.uMin = 0.0;
    p.uMax = 1.0;
    return p;
}

struct CoupledPlant {
    OvenCoupledParams prm;
    double ts{25.0}, tp{25.0};   // zone-1 (sensor), zone-2 (product) temps
    double t{0.0};
    std::deque<std::pair<double, double>> hist;

    explicit CoupledPlant(const OvenCoupledParams& p) : prm(p) {}

    void step(double u, double dt) {
        hist.emplace_back(t, u);
        while (hist.size() > 2 &&
               hist.front().first < t - prm.delay - 2.0)
            hist.pop_front();
        double ud = 0.0;
        for (const auto& [ti, ui] : hist) {
            if (ti <= t - prm.delay) ud = ui;
            else break;
        }
        const double dx1 = prm.alpha * ud
            - prm.betaSensor * (ts - prm.ambient)
            - prm.gammaSP * (ts - tp);
        const double dx2 = prm.gammaPS * (ts - tp)
            - prm.betaProduct * (tp - prm.ambient)
            + prm.disturbance;
        ts += dx1 * dt;
        tp += dx2 * dt;
        t += dt;
    }
};

struct CoupledSimResult {
    std::vector<std::pair<double, double>> prod;  // (t, product temp)
    std::vector<std::pair<double, double>> sens;  // (t, sensor temp)
    std::vector<double> refs;
    double maxProdErr{0.0};
    double soakProdErr{0.0};
    double finalProdErr{0.0};
    double maxSensorProdGap{0.0};
    OvenControllerStatus lastStatus{};
};

// Run OvenController (any backend) against the coupled plant. The profile
// is interpreted as the PRODUCT-temperature trajectory.
CoupledSimResult simulateCoupled(
        const OvenCoupledParams& plantP,
        const OvenControllerConfig& ctrlCfg,
        const TemperatureProfile& profile,
        double duration, double dt = 0.5,
        double noiseSigma = 0.0,
        const std::function<double(double)>& disturbanceFn = {}) {
    CoupledPlant plant(plantP);
    plant.ts = plant.tp = profile.value(0.0);

    OvenController ctrl(ctrlCfg);
    ctrl.setProfile(profile);

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, noiseSigma);

    CoupledSimResult res;
    ControllerInput in;
    in.dt = dt;
    in.reference = 25.0;

    for (double t = 0.0; t < duration; t += dt) {
        if (disturbanceFn) plant.prm.disturbance = disturbanceFn(t);
        in.measured = plant.ts + (noiseSigma > 0.0 ? noise(rng) : 0.0);
        const ControllerOutput out = ctrl.compute(in);
        plant.step(std::clamp(out.control, plantP.uMin, plantP.uMax), dt);

        const auto st = ctrl.status();
        res.prod.emplace_back(t, plant.tp);
        res.sens.emplace_back(t, plant.ts);
        res.refs.push_back(st.reference);
        res.maxProdErr = std::max(res.maxProdErr,
                                  std::abs(plant.tp - st.reference));
        res.maxSensorProdGap =
            std::max(res.maxSensorProdGap, std::abs(plant.ts - plant.tp));
    }

    double sum = 0.0;
    int n = 0;
    for (size_t i = 0; i < res.prod.size(); ++i) {
        if (res.prod[i].first > duration - 120.0) {
            sum += std::abs(res.prod[i].second - res.refs[i]);
            ++n;
        }
    }
    res.soakProdErr = n ? sum / n : 0.0;
    res.finalProdErr = std::abs(res.prod.back().second - res.refs.back());
    res.lastStatus = ctrl.status();
    return res;
}

} // namespace

// ---------------------------------------------------------------------------
// TemperatureProfile
// ---------------------------------------------------------------------------

TEST(TemperatureProfileTest, RampSoakValues) {
    TemperatureProfile p(25.0);
    p.addRampSoak(100.0, 0.5, 100.0);   // ramp 150 s, soak 100 s
    p.addSoak(50.0);
    ASSERT_FALSE(p.empty());
    EXPECT_DOUBLE_EQ(p.value(0.0), 25.0);
    EXPECT_DOUBLE_EQ(p.value(75.0), 62.5);   // mid-ramp
    EXPECT_DOUBLE_EQ(p.value(200.0), 100.0); // soak
    EXPECT_DOUBLE_EQ(p.value(400.0), 100.0); // past end holds
    EXPECT_DOUBLE_EQ(p.derivative(75.0), 0.5);
    EXPECT_DOUBLE_EQ(p.derivative(200.0), 0.0);
    EXPECT_DOUBLE_EQ(p.duration(), 300.0);
}

TEST(TemperatureProfileTest, FromPointsPiecewiseLinear) {
    const auto p = TemperatureProfile::fromPoints(
        {{0.0, 25.0}, {100.0, 75.0}, {200.0, 75.0}});
    ASSERT_FALSE(p.empty());
    EXPECT_DOUBLE_EQ(p.value(50.0), 50.0);
    EXPECT_DOUBLE_EQ(p.value(150.0), 75.0);
    EXPECT_DOUBLE_EQ(p.derivative(50.0), 0.5);
    EXPECT_DOUBLE_EQ(p.derivative(150.0), 0.0);

    EXPECT_TRUE(TemperatureProfile::fromPoints({{0.0, 25.0}}).empty());
    EXPECT_TRUE(TemperatureProfile::fromPoints(
                    {{0.0, 25.0}, {50.0, 30.0}, {40.0, 35.0}}).empty());
}

// ---------------------------------------------------------------------------
// OvenThermalModel
// ---------------------------------------------------------------------------

TEST(OvenThermalModelTest, RequiredInputSteadyState) {
    const auto p = plantParams();
    OvenThermalModel m(p);
    // At steady state on the reference: dT/dt = 0 →
    //   u = beta*(T-Tamb)/alpha = (T-Tamb)/K.
    const double u = m.requiredInput(100.0, 0.0, 25.0, 0.0);
    EXPECT_NEAR(u, (100.0 - 25.0) / 200.0, 1e-9);

    // Ramp feedforward adds slope/alpha.
    const double uRamp = m.requiredInput(100.0, 0.3, 25.0, 0.0);
    EXPECT_NEAR(uRamp, u + 0.3 / p.alpha, 1e-9);

    // Disturbance (endothermic, d<0) raises required input.
    const double uDist = m.requiredInput(100.0, 0.0, 25.0, -0.05);
    EXPECT_GT(uDist, u);
}

TEST(OvenThermalModelTest, SimulateReachesSteadyState) {
    OvenThermalModel m(plantParams());
    const auto traj = m.simulate(0.5, 1.0, 1500, 25.0, 0.0);
    ASSERT_EQ(traj.size(), 1501u);
    // Steady-state: T = Tamb + K*u = 25 + 200*0.5 = 125.
    EXPECT_NEAR(traj.back(), 125.0, 2.0);
}

// ---------------------------------------------------------------------------
// ActuationGovernor
// ---------------------------------------------------------------------------

TEST(ActuationGovernorTest, LimitsCoolingToPassiveRate) {
    GovernorConfig gc;
    gc.alpha = 1.0;
    gc.beta = 0.005;
    gc.uMin = 0.0;
    gc.uMax = 1.0;
    gc.ambient = 25.0;
    ActuationGovernor gov(gc);
    gov.reset(100.0);   // start at the profile's initial temperature

    // Profile demands -1 C/s descent from 100 C: physically impossible
    // (passive cooling at 100 C is only beta*(100-25) = 0.375 C/s).
    TemperatureProfile p(100.0);
    p.addRampTo(40.0, 1.0);

    double maxRate = 0.0;
    for (int i = 0; i < 200; ++i) {
        gov.update(i * 0.5, p, 0.5);
        maxRate = std::min(maxRate, gov.rate());
    }
    // Governed rate must never exceed the achievable cooling bound.
    EXPECT_GE(maxRate, -0.375 - 1e-6);
    // And it must actually be descending (bounded, not frozen).
    EXPECT_LT(maxRate, -0.3);
}

// ---------------------------------------------------------------------------
// OvenAutotune
// ---------------------------------------------------------------------------

TEST(OvenAutotuneTest, RecoversFopdtFromStepResponse) {
    // True plant -> noise-free step response via the model simulator.
    const auto p = plantParams();
    OvenThermalModel plant(p);
    const double dt = 1.0;
    const int steps = 1400;   // ~7 tau: reaches steady state
    const auto traj = plant.simulate(0.4, dt, steps, 25.0, 0.0);

    StepResponseData data;
    data.uStep = 0.4;
    data.tAmb = 25.0;
    data.uMin = 0.0;
    data.uMax = 1.0;
    for (int i = 0; i <= steps; ++i) {
        data.time.push_back(i * dt);
        data.temp.push_back(traj[i]);
    }

    const auto result = autoTuneOven(data);
    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.model.K, 200.0, 20.0);       // ±10%
    EXPECT_NEAR(result.model.tau, 200.0, 30.0);     // ±15%
    EXPECT_NEAR(result.model.theta, 4.0, 1.5);
    EXPECT_NEAR(result.params.alpha, 1.0, 0.2);
    EXPECT_NEAR(result.params.beta, 0.005, 0.001);
    EXPECT_NEAR(result.params.delay, 4.0, 1.5);
    EXPECT_GT(result.kp, 0.0);
    EXPECT_GT(result.ki, 0.0);
}

TEST(OvenAutotuneTest, RejectsDegenerateStep) {
    StepResponseData data;
    for (int i = 0; i < 100; ++i) {
        data.time.push_back(i * 1.0);
        data.temp.push_back(25.0 + 0.001 * i);  // far too small
    }
    data.uStep = 0.5;
    const auto result = autoTuneOven(data, 0.0, 1.0);
    EXPECT_FALSE(result.success);
}

// ---------------------------------------------------------------------------
// OvenController — nominal tracking
// ---------------------------------------------------------------------------

TEST(OvenControllerTest, TracksRampAndSoak) {
    const auto p = plantParams();
    OvenControllerConfig cfg;
    cfg.model = p;
    cfg.adaptParams = false;   // isolate FF+PI from adaptation
    cfg.sensorTau = 5.0;

    const auto res = simulate(p, cfg, testProfile(), 800.0, 0.5);

    ASSERT_FALSE(res.samples.empty());
    for (const auto& s : res.samples) {
        ASSERT_TRUE(std::isfinite(s.u));
        EXPECT_GE(s.u, -1e-9);
        EXPECT_LE(s.u, 1.0 + 1e-9);
    }
    // Soak tracking: last 120 s mean |err| < 1.5 C.
    EXPECT_LT(res.soakErr, 1.5);
    EXPECT_LT(res.finalErr, 1.5);
    // Ramp tracking is looser (preview + filter lag) but bounded.
    EXPECT_LT(maxErrIn(res, 60.0, 240.0), 3.0);
    EXPECT_LT(maxErrIn(res, 460.0, 640.0), 3.0);
}

TEST(OvenControllerTest, TracksWithMeasurementNoise) {
    const auto p = plantParams();
    OvenControllerConfig cfg;
    cfg.model = p;
    cfg.adaptParams = false;
    cfg.sensorTau = 8.0;   // heavier filtering for noisy sensor

    const auto res = simulate(p, cfg, testProfile(), 800.0, 0.5,
                              /*noiseSigma=*/0.3);
    EXPECT_LT(res.soakErr, 2.0);
    EXPECT_LT(res.finalErr, 2.0);
    EXPECT_LT(maxErrIn(res, 60.0, 240.0), 4.0);
}

TEST(OvenControllerTest, RejectsLoadDisturbance) {
    const auto p = plantParams();
    OvenControllerConfig cfg;
    cfg.model = p;
    cfg.adaptParams = true;
    cfg.estimator.windowTime = 15.0;
    cfg.sensorTau = 5.0;

    // Endothermic load: -0.05 C/s starting at t=300 (mid soak), permanent.
    const auto dist = [](double t) { return t >= 300.0 ? -0.05 : 0.0; };
    const auto res = simulate(p, cfg, testProfile(), 800.0, 0.5, 0.0, dist);

    // The loop must recover: mean |err| over the last 120 s stays small
    // even though the load persists (integral action + d estimate).
    EXPECT_LT(res.soakErr, 2.0);
    EXPECT_LT(res.finalErr, 2.0);
    // During the disturbance transient the error is bounded.
    EXPECT_LT(maxErrIn(res, 300.0, 450.0), 8.0);
}

TEST(OvenControllerTest, AdaptsToParameterMismatch) {
    auto trueP = plantParams();
    trueP.alpha = 1.3;    // plant is 30% more responsive
    trueP.beta = 0.006;   // and loses heat faster

    OvenControllerConfig cfg;
    cfg.model = plantParams();   // nominal (wrong) model
    cfg.adaptParams = true;
    cfg.estimator.windowTime = 8.0;
    cfg.estimator.memTime = 150.0;
    cfg.estimator.smoothing = 0.5;
    cfg.ditherAmp = 0.04;
    cfg.sensorTau = 5.0;

    const auto res = simulate(trueP, cfg, testProfile(), 1000.0, 0.5);

    // The bounded estimator should converge near the true physical
    // parameters (alpha=1.3, beta=0.006).
    EXPECT_NEAR(res.lastStatus.alpha, 1.3, 0.3);
    EXPECT_NEAR(res.lastStatus.beta, 0.006, 0.0015);
    EXPECT_GT(res.lastStatus.windowsUsed, 10);

    // And tracking must have recovered via adapted model + retuned PI.
    EXPECT_LT(res.soakErr, 2.5);
    EXPECT_LT(res.finalErr, 2.5);
    EXPECT_LT(maxErrIn(res, 700.0, 950.0), 3.5);
}

// Open-loop estimator convergence on rich excitation: with independent
// input the sliding-window regression should recover {alpha,beta,d}
// essentially exactly — this validates the estimator math itself,
// separate from closed-loop identifiability.
TEST(ThermalParameterEstimatorTest, OpenLoopConvergence) {
    auto p = plantParams();
    p.alpha = 1.3; p.beta = 0.006;
    OvenThermalModel plant(p);
    plant.reset();

    EstimatorConfig ec;
    ec.windowTime = 15.0;
    ec.memTime = 150.0;
    ec.smoothing = 0.5;
    ThermalParameterEstimator est(ec);
    est.setPrior(1.0, 0.005, 0.0);

    std::deque<std::pair<double,double>> hist;
    const double dt = 0.5, delay = 4.0;
    for (int i = 0; i < 1600; ++i) {   // 800 s
        const double t = i * dt;
        double u = 0.35 + 0.25 * std::sin(2.0 * 3.14159265 * t / 47.0)
                      + 0.10 * std::sin(2.0 * 3.14159265 * t / 13.0);
        u = std::clamp(u, 0.0, 1.0);
        hist.emplace_back(t, u);
        while (hist.size() > 100 && t - hist.front().first > delay + 1.0)
            hist.pop_front();
        double uD = 0.0;
        for (auto& [ti, ui] : hist)
            if (ti <= t - delay) uD = ui;
        plant.step(u, dt);
        est.update(uD, plant.temperature(), 25.0, dt);
    }
    EXPECT_NEAR(est.alpha(), 1.3, 0.1);
    EXPECT_NEAR(est.beta(), 0.006, 0.0015);
    EXPECT_NEAR(est.disturbance(), 0.0, 0.1);
    EXPECT_GT(est.updatesUsed(), 500);
}

TEST(ThermalParameterEstimatorTest, HoldsDuringUnexcitedSoak) {
    // Constant input + constant temperature: windows are unidentifiable,
    // the estimate must not drift on noise alone.
    ThermalParameterEstimator est;
    est.setPrior(1.0, 0.005, 0.0);
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.1);
    for (int i = 0; i < 400; ++i) {
        est.update(0.4, 80.0 + noise(rng), 25.0, 0.5);
    }
    EXPECT_DOUBLE_EQ(est.alpha(), 1.0);
    EXPECT_DOUBLE_EQ(est.beta(), 0.005);
    EXPECT_EQ(est.updatesUsed(), 0);
}

TEST(OvenControllerTest, ConstantSetpointModeWithoutProfile) {
    const auto p = plantParams();
    OvenControllerConfig cfg;
    cfg.model = p;
    cfg.adaptParams = false;
    cfg.sensorTau = 5.0;

    OvenThermalModel plant(p);
    OvenController ctrl(cfg);
    // No profile: constant setpoint via input.reference.
    ControllerInput in;
    in.dt = 0.5;
    in.reference = 80.0;

    for (int i = 0; i < 1600; ++i) {   // 800 s
        in.measured = plant.temperature();
        const auto out = ctrl.compute(in);
        plant.step(out.control, 0.5);
    }
    EXPECT_NEAR(plant.temperature(), 80.0, 2.0);
}

TEST(OvenControllerTest, OverTemperatureShutdown) {
    const auto p = plantParams();
    OvenControllerConfig cfg;
    cfg.model = p;
    cfg.maxTempC = 90.0;
    cfg.overTempHysteresis = 3.0;
    cfg.adaptParams = false;

    // Profile demands 120 C but limit is 90 C.
    TemperatureProfile prof(25.0);
    prof.addRampSoak(120.0, 0.3, 400.0);

    const auto res = simulate(p, cfg, prof, 600.0, 0.5);
    for (const auto& s : res.samples) {
        // Hard interlock at 90 C; residual overshoot is physical coast-down
        // (passive cooling only, thermal lag + input delay).
        EXPECT_LE(s.temp, 100.0);
    }
    // Output must have been driven to minimum during the violation.
    bool sawShutdown = false;
    for (const auto& s : res.samples)
        if (s.temp > 90.0 && s.u <= 1e-9) sawShutdown = true;
    EXPECT_TRUE(sawShutdown);
}

// ---------------------------------------------------------------------------
// OvenLQIBackend — non-collocated sensing (product vs chamber sensor)
// ---------------------------------------------------------------------------

// Gentle product-temperature profile: 25 -> 90 C at 0.12 C/s, soak.
// The product zone cannot follow faster ramps (coupling-limited), so the
// rate is deliberately conservative; the soak is where precision matters.
TemperatureProfile coupledProfile() {
    TemperatureProfile p(25.0);
    p.addRampSoak(90.0, 0.12, 250.0);
    return p;
}

OvenControllerConfig lqiConfig(const OvenCoupledParams& plantP) {
    OvenControllerConfig cfg;
    // SISO FOPDT model for the feedforward path — approximate input→sensor
    // dynamics; the LQI feedback trims the residual.
    cfg.model.alpha = 1.0;
    cfg.model.beta = 0.01;
    cfg.model.delay = plantP.delay;
    cfg.model.ambient = plantP.ambient;
    cfg.model.uMin = plantP.uMin;
    cfg.model.uMax = plantP.uMax;
    cfg.feedbackBackend = OvenFeedbackBackendKind::LQI;
    cfg.coupled = plantP;
    cfg.adaptParams = false;   // isolate control law from adaptation
    cfg.sensorTau = 3.0;
    return cfg;
}

TEST(OvenLQIBackendTest, ControlsUnmeasuredProductTemperature) {
    const auto plantP = coupledPlantParams();
    const auto cfg = lqiConfig(plantP);

    const auto res = simulateCoupled(plantP, cfg, coupledProfile(), 800.0);

    EXPECT_STREQ(res.lastStatus.backendName, "LQI+Observer");
    // Product (zone 2) must track the profile — this is the whole point of
    // the non-collocated backend.
    EXPECT_LT(res.soakProdErr, 2.5);
    EXPECT_LT(res.finalProdErr, 2.5);
    EXPECT_LT(res.maxProdErr, 15.0);
    // Non-collocation proof: sensor and product genuinely differ (the
    // sensor runs hotter to push heat into the product zone).
    EXPECT_GT(res.maxSensorProdGap, 5.0);
}

TEST(OvenLQIBackendTest, PIBackendLeavesProductOffset) {
    // Same plant/profile with the default SIMC-PI backend: the PI regulates
    // the SENSOR temperature to the profile, so the product settles at a
    // steady-state offset — motivating the LQI backend.
    const auto plantP = coupledPlantParams();
    OvenControllerConfig cfg;
    cfg.model.alpha = 1.0;
    cfg.model.beta = 0.01;
    cfg.model.delay = plantP.delay;
    cfg.model.ambient = plantP.ambient;
    cfg.model.uMin = plantP.uMin;
    cfg.model.uMax = plantP.uMax;
    cfg.adaptParams = false;
    cfg.sensorTau = 3.0;

    const auto res = simulateCoupled(plantP, cfg, coupledProfile(), 800.0);

    EXPECT_STREQ(res.lastStatus.backendName, "SIMC-PI");
    // Sensor tracks the profile; product settles at a steady-state offset
    // (the coupling gradient needed to feed zone-2 losses) — ~11 C here.
    // This is exactly the failure mode the LQI backend exists to fix.
    EXPECT_GT(res.maxSensorProdGap, 5.0);
    EXPECT_GT(res.finalProdErr, 5.0);
    EXPECT_LT(res.soakProdErr, res.finalProdErr + 3.0);  // persistent, not transient
}

TEST(OvenLQIBackendTest, ObserverEstimatesBothZones) {
    const auto prm = coupledPlantParams();
    OvenLQIBackend be(prm, 0.01, 0.004, 0.004, 1000.0, 1e-4, 0.09, 0.5);
    CoupledPlant plant(prm);

    ControllerInput in;
    in.dt = 0.5;
    in.reference = 90.0;   // product setpoint

    for (int i = 0; i < 1600; ++i) {   // 800 s
        in.measured = plant.ts;
        const auto out = be.compute(in);
        const double u = std::clamp(out.control, prm.uMin, prm.uMax);
        plant.step(u, 0.5);
        be.notifyAppliedInput(u);
    }

    // Observer must reconstruct BOTH zones from the single sensor channel.
    EXPECT_NEAR(be.observer().xSensor(), plant.ts, 3.0);
    EXPECT_NEAR(be.observer().xProduct(), plant.tp, 4.0);
    // And the loop regulates the (unmeasured) product temperature.
    EXPECT_NEAR(plant.tp, 90.0, 3.0);
}

TEST(OvenLQIBackendTest, RejectsProductLoadDisturbance) {
    const auto plantP = coupledPlantParams();
    const auto cfg = lqiConfig(plantP);

    // Endothermic load on the product zone: -0.03 °C/s from t=450 on.
    const auto dist = [](double t) { return t >= 450.0 ? -0.03 : 0.0; };
    const auto res = simulateCoupled(plantP, cfg, coupledProfile(), 800.0,
                                     0.5, 0.0, dist);

    // Integral action on the product-temperature error must absorb the
    // persistent load: steady-state error stays small.
    EXPECT_LT(res.soakProdErr, 3.0);
    EXPECT_LT(res.finalProdErr, 3.0);
}

TEST(OvenLQIBackendTest, AdaptsToParameterMismatch) {
    // Controller nominal model is wrong; the bounded estimator should push
    // corrected {alpha,beta,d} into the backend, which redesigns observer
    // gains and LQI gains online.
    auto plantP = coupledPlantParams();
    plantP.alpha = 1.5;          // true plant 25% more responsive
    plantP.betaSensor = 0.010;   // and loses heat faster
    plantP.betaProduct = 0.0025;

    auto nominal = plantP;
    nominal.alpha = 1.2;
    nominal.betaSensor = 0.008;
    nominal.betaProduct = 0.002;

    OvenControllerConfig cfg = lqiConfig(nominal);
    cfg.adaptParams = true;
    cfg.estimator.windowTime = 10.0;
    cfg.estimator.memTime = 150.0;
    cfg.estimator.smoothing = 0.5;
    cfg.ditherAmp = 0.04;

    const auto res = simulateCoupled(plantP, cfg, coupledProfile(), 1200.0);

    // Smoke-level assertions: loop stays bounded, product converges, and
    // the estimator actually moved the parameters (retune path exercised).
    EXPECT_TRUE(std::isfinite(res.lastStatus.output));
    EXPECT_LT(res.soakProdErr, 4.0);
    EXPECT_LT(res.finalProdErr, 4.0);
    EXPECT_GT(res.lastStatus.windowsUsed, 10);
    EXPECT_NEAR(res.lastStatus.alpha, plantP.alpha, 0.6);
}

TEST(OvenControllerTest, BackendSwapAtRuntime) {
    const auto plantP = coupledPlantParams();
    auto cfg = lqiConfig(plantP);
    cfg.feedbackBackend = OvenFeedbackBackendKind::PI;   // start on PI

    CoupledPlant plant(plantP);
    plant.ts = plant.tp = 25.0;
    OvenController ctrl(cfg);
    ctrl.setProfile(coupledProfile());

    ControllerInput in;
    in.dt = 0.5;
    in.reference = 25.0;

    // Phase 1: PI backend for 300 s.
    for (double t = 0.0; t < 300.0; t += 0.5) {
        in.measured = plant.ts;
        const auto out = ctrl.compute(in);
        plant.step(std::clamp(out.control, 0.0, 1.0), 0.5);
    }
    EXPECT_STREQ(ctrl.feedbackBackendName(), "SIMC-PI");

    // Phase 2: swap to the LQI backend mid-run — no reset, bumpless-ish
    // handoff (integrator starts at zero; FF keeps supplying bulk power).
    ctrl.setFeedbackBackend(std::make_unique<OvenLQIBackend>(
        plantP, cfg.lqiQxSensor, cfg.lqiQxProduct, cfg.lqiQi, cfg.lqiR,
        cfg.observerQProc, cfg.observerRMeas, cfg.backendDt));
    EXPECT_STREQ(ctrl.feedbackBackendName(), "LQI+Observer");

    for (double t = 300.0; t < 800.0; t += 0.5) {
        in.measured = plant.ts;
        const auto out = ctrl.compute(in);
        plant.step(std::clamp(out.control, 0.0, 1.0), 0.5);
    }

    // After the swap the product must still converge to the profile soak.
    EXPECT_NEAR(plant.tp, 90.0, 4.0);
}
