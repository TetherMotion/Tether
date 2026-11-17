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
#include "tether/control/thermal/OvenSOPDTModel.hpp"
#include "tether/control/thermal/OvenThermalModel.hpp"
#include "tether/control/thermal/SOPDTParameterEstimator.hpp"
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
// `sensorFn`, when set, replaces the degree-C Gaussian path: it maps the
// true plant temperature to the measured temperature (e.g. voltage-domain
// noise through a nonlinear sensor inverse).
SimResult simulate(OvenModelParams plantP,
                   const OvenControllerConfig& ctrlCfg,
                   const TemperatureProfile& profile,
                   double duration, double dt = 0.5,
                   double noiseSigma = 0.0,
                   const std::function<double(double)>& disturbanceFn = {},
                   const std::function<double(double, std::mt19937&)>& sensorFn
                       = {}) {
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
        const double trueT = plant.temperature();
        in.measured = sensorFn ? sensorFn(trueT, rng)
                               : trueT + (noiseSigma > 0.0 ? noise(rng) : 0.0);
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
// Voltage-domain sensor noise models
// ---------------------------------------------------------------------------
// Real oven sensors do not emit temperature directly: a thermocouple produces
// a small EMF (mV) and a thermistor a divider voltage (V). ADC/front-end noise
// is Gaussian in that VOLTAGE domain, and the firmware maps voltage back to
// temperature through a nonlinear inverse equation. Because the inverse is
// curved, the temperature-domain noise is skewed and its gain varies with
// operating point — not the symmetric Gaussian noise used elsewhere in these
// tests. These models reproduce that chain: true T -> voltage -> +N(0,sigmaV)
// -> inverse -> measured T.

// Type-K thermocouple (NiCr-NiAl). NIST ITS-90 reference function E(T) [mV]
// for 0..1372 C plus the standard approximate inverse T(E) [degC] for
// 0..500 C. Cold junction compensated -> cancels, so only the measurement
// junction EMF matters. Type K is nearly linear in the oven range
// (Seebeck ~40 uV/C), so the induced skew is small — kept for contrast.
struct TypeKThermocouple {
    double sigmaV{0.010};   // ADC noise on EMF, [mV]

    // Forward: temperature -> EMF [mV].
    static double emf(double T) {
        static const double c[10] = {
            -0.0176004136860, 0.0389212049750, 0.00001855587700320,
            -9.9457592874e-8, 3.1840945719e-10, -5.6072844889e-13,
            5.6075059059e-16, -3.2020720003e-19, 9.7151147152e-23,
            -1.2104721275e-26};
        double p = 0.0, tp = 1.0;
        for (double ci : c) { p += ci * tp; tp *= T; }
        const double d = T - 126.9686;
        return p + 0.1185976 * std::exp(-0.0001183432 * d * d);
    }

    // Inverse: EMF -> temperature [degC] (NIST 0..500 C polynomial).
    static double temp(double E) {
        static const double d[10] = {
            0.0, 25.08355, 7.860106e-2, -2.503131e-1, 8.315270e-2,
            -1.228034e-2, 9.804036e-4, -4.413030e-5, 1.057734e-6,
            -1.052755e-8};
        double p = 0.0, ep = 1.0;
        for (double di : d) { p += di * ep; ep *= E; }
        return p;
    }

    double measure(double T, std::mt19937& rng) const {
        std::normal_distribution<double> n(0.0, sigmaV);
        return temp(emf(T) + n(rng));
    }
};

// NTC thermistor (beta-parameter model) in a resistor divider.
//   VCC -- NTC -- sense node -- Rf -- GND ;  V_node = VCC*Rf/(Rntc+Rf)
// Voltage rises with temperature (Rntc falls). Inverse: Rntc=Rf*(VCC-V)/V.
// Rf is chosen so the divider stays well inside the ADC rails over the oven
// range; a poorly chosen Rf pushes the sense voltage toward a rail where the
// temperature gain explodes. sigmaV is ADC noise in volts.
struct NtcThermistor {
    double R0{100e3};     // nominal resistance at T0 [ohm]
    double T0{298.15};    // reference temperature [K]
    double B{3950.0};     // beta constant [K]
    double Rf{4.7e3};     // divider fixed resistor [ohm]
    double VCC{3.3};      // supply / ADC full scale [V]
    double sigmaV{0.003}; // ADC noise [V]  (~0.2-0.5 C in the oven range)

    double resistance(double T) const {   // T in degC
        return R0 * std::exp(B * (1.0 / (T + 273.15) - 1.0 / T0));
    }
    double voltage(double T) const {
        return VCC * Rf / (resistance(T) + Rf);
    }
    double tempFromVoltage(double V) const {
        V = std::clamp(V, 1e-4, VCC - 1e-4);   // rail / domain guard
        const double r = Rf * (VCC - V) / V;
        return 1.0 / (1.0 / T0 + std::log(r / R0) / B) - 273.15;
    }
    double measure(double T, std::mt19937& rng) const {
        std::normal_distribution<double> n(0.0, sigmaV);
        return tempFromVoltage(voltage(T) + n(rng));
    }
};

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

// ---------------------------------------------------------------------------
// SOPDT cascade model
// ---------------------------------------------------------------------------

// SOPDT plant: stage-1 lossy mass {alpha=1.0, beta=0.005 -> tau1=200,
// K=200}, stage-2 pure lag tau2=300, delay=4. Effective dominant lag
// tau1+tau2 = 500 s.
OvenModelParamsSOPDT sopdtPlantParams() {
    OvenModelParamsSOPDT p;
    p.alpha = 1.0;
    p.beta = 0.005;
    p.tau2 = 300.0;
    p.delay = 4.0;
    p.disturbance = 0.0;
    p.ambient = 25.0;
    p.uMin = 0.0;
    p.uMax = 1.0;
    return p;
}

TEST(OvenSOPDTModelTest, StepResponseMatchesAnalytic) {
    // Constant input u: the cascade reaches steady state
    // T = Tamb + K*u with K = alpha/beta = 200.
    const auto p = sopdtPlantParams();
    OvenThermalModelSOPDT m(p);
    m.reset();
    const double dt = 0.5;
    for (int i = 0; i < 4000; ++i) m.step(0.4, dt);   // 2000 s >> tau_tot
    EXPECT_NEAR(m.temperature(), 25.0 + 200.0 * 0.4, 1.0);
}

TEST(OvenSOPDTModelTest, Tau2ZeroRecoversFopdt) {
    // tau2 -> 0 must collapse to the single-mass FOPDT response.
    auto p = sopdtPlantParams();
    p.tau2 = 1e-6;
    OvenThermalModelSOPDT ms(p);
    ms.reset();
    OvenModelParams f;
    f.alpha = p.alpha; f.beta = p.beta; f.delay = p.delay;
    f.disturbance = 0.0; f.ambient = p.ambient; f.uMin = 0; f.uMax = 1;
    OvenThermalModel mf(f);
    mf.reset();
    const double dt = 0.5;
    for (int i = 0; i < 800; ++i) {
        const double us = ms.step(0.5, dt);
        const double uf = mf.step(0.5, dt);
        EXPECT_NEAR(us, uf, 1e-6);
    }
}

TEST(OvenSOPDTModelTest, ExactInverseTracksRamp) {
    // Open-loop: feed the exact model-inverse FF for a ramp and check the
    // output tracks the reference once the stage-1 state has converged
    // onto the required trajectory (e^{-t/tau1} transient ~ 3*tau1).
    const auto p = sopdtPlantParams();
    OvenThermalModelSOPDT m(p);
    m.reset();
    const double dt = 0.5;
    const double rate = 0.1;                    // C/s ramp (keeps u<1)
    const double tEnd = 1400.0;
    double maxErr = 0.0;
    for (double t = 0.0; t < tEnd; t += dt) {
        const double r = 25.0 + rate * t;
        const double u = m.requiredInputClamped(r, rate, 0.0, 25.0, 0.0);
        m.step(u, dt);
        if (t > tEnd - 200.0)                  // converged tail only
            maxErr = std::max(maxErr, std::abs(m.temperature() - r));
    }
    EXPECT_LT(maxErr, 2.5);
}

// ---------------------------------------------------------------------------
// SOPDT parameter estimator (output-error fit)
// ---------------------------------------------------------------------------

TEST(SOPDTParameterEstimatorTest, OpenLoopConvergence) {
    const auto p = sopdtPlantParams();
    OvenThermalModelSOPDT plant(p);
    plant.reset();

    SOPDTEstimatorConfig ec;
    // windowTime is floored at windowLagFactor·(τ1+τ2) ≈ 4·500 = 2000 s —
    // the scored window must span several slowest-lag constants for the
    // two-lag split to be identifiable.
    ec.windowTime = 300.0;
    ec.windowLagFactor = 4.0;
    ec.spinupFactor = 4.0;
    ec.fitInterval = 15.0;
    ec.simStride = 2.0;
    ec.gridSize = 9;
    ec.smoothing = 0.4;
    SOPDTParameterEstimator est(ec);
    // Perturbed prior: estimator must move toward the truth.
    est.setPrior(0.8, 0.006, 200.0, 0.0);

    std::deque<std::pair<double, double>> hist;
    const double dt = 0.5, delay = p.delay;
    for (int i = 0; i < 12000; ++i) {         // 6000 s
        const double t = i * dt;
        double u = 0.4 + 0.3 * std::sin(2.0 * 3.14159265 * t / 47.0)
                  + 0.15 * std::sin(2.0 * 3.14159265 * t / 13.0)
                  + 0.10 * std::copysign(1.0, std::sin(2.0 * 3.14159265 * t / 29.0));
        u = std::clamp(u, 0.0, 1.0);
        hist.emplace_back(t, u);
        while (hist.size() > 100 && t - hist.front().first > delay + 1.0)
            hist.pop_front();
        double uD = 0.0;
        for (auto& [ti, ui] : hist)
            if (ti <= t - delay) uD = ui;
        // Same convention the controller uses: T at the START of the
        // interval, uDelayed driving [t, t+dt].
        const double Tnow = plant.temperature();
        plant.step(u, dt);
        est.update(uD, Tnow, 25.0, dt);
    }

    EXPECT_GT(est.updatesUsed(), 5);
    // The SOPDT I/O transfer K/((τ1s+1)(τ2s+1)) is symmetric in the two
    // lags, so the {τ1,τ2} *labeling* is unidentifiable from a single
    // output — what the OE fit must recover is the DC gain K and the
    // lag set (either order). Assert those, plus a bounded disturbance.
    const double Ktrue = p.alpha / p.beta;         // 200
    EXPECT_NEAR(est.gain(), Ktrue, 0.30 * Ktrue);
    double lags[2] = {est.tau1(), est.tau2()};
    double truth[2] = {1.0 / p.beta, p.tau2};      // {200, 300}
    std::sort(lags, lags + 2);
    std::sort(truth, truth + 2);
    EXPECT_NEAR(lags[0], truth[0], 0.45 * truth[0]);
    EXPECT_NEAR(lags[1], truth[1], 0.45 * truth[1]);
    EXPECT_NEAR(est.disturbance(), 0.0, 0.2);
}

TEST(SOPDTParameterEstimatorTest, HoldsDuringUnexcitedSoak) {
    SOPDTParameterEstimator est;
    est.setPrior(1.0, 0.005, 300.0, 0.0);
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.1);
    for (int i = 0; i < 4000; ++i) {
        est.update(0.4, 80.0 + noise(rng), 25.0, 0.5);
    }
    EXPECT_DOUBLE_EQ(est.alpha(), 1.0);
    EXPECT_DOUBLE_EQ(est.beta(), 0.005);
    EXPECT_DOUBLE_EQ(est.tau2(), 300.0);
    EXPECT_EQ(est.updatesUsed(), 0);
}

// ---------------------------------------------------------------------------
// SOPDT autotune
// ---------------------------------------------------------------------------

TEST(OvenAutotuneTest, IdentifiesSopdtStepResponse) {
    const auto p = sopdtPlantParams();
    OvenThermalModelSOPDT plant(p);
    plant.reset();
    const double dt = 1.0;
    const int steps = 3000;                    // ~6 tau_tot
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

    const auto result = autoTuneOvenSOPDT(data);
    ASSERT_TRUE(result.success);
    // Effective gain K = alpha/beta = 200; dominant lag ~ tau1+tau2 = 500.
    EXPECT_NEAR(result.params.gain(), 200.0, 40.0);
    EXPECT_GT(result.params.tau2, 20.0);        // second lag found
    EXPECT_GT(result.kp, 0.0);
    EXPECT_GT(result.ki, 0.0);
}

// ---------------------------------------------------------------------------
// OvenController — SOPDT end-to-end
// ---------------------------------------------------------------------------

// Closed-loop sim against the SOPDT plant (mirrors simulate() but with
// OvenThermalModelSOPDT as the true plant).
SimResult simulateSopdt(OvenModelParamsSOPDT plantP,
                        const OvenControllerConfig& ctrlCfg,
                        const TemperatureProfile& profile,
                        double duration, double dt = 0.5,
                        double noiseSigma = 0.0,
                        const std::function<double(double)>& disturbanceFn = {},
                        const std::function<double(double, std::mt19937&)>& sensorFn
                            = {}) {
    OvenThermalModelSOPDT plant(plantP);
    plant.reset();

    OvenController ctrl(ctrlCfg);
    ctrl.setProfile(profile);

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, noiseSigma);

    SimResult res;
    ControllerInput in;
    in.dt = dt;
    in.reference = 25.0;

    for (double t = 0.0; t < duration; t += dt) {
        if (disturbanceFn) plant.params().disturbance = disturbanceFn(t);
        const double trueT = plant.temperature();
        in.measured = sensorFn ? sensorFn(trueT, rng)
                               : trueT + (noiseSigma > 0.0 ? noise(rng) : 0.0);
        const ControllerOutput out = ctrl.compute(in);
        plant.step(out.control, dt);

        const auto st = ctrl.status();
        SimSample s{t, plant.temperature(), st.reference, out.control,
                    out.feedforward, out.control - out.feedforward};
        res.samples.push_back(s);
        const double err = std::abs(s.temp - s.ref);
        res.maxAbsErr = std::max(res.maxAbsErr, err);
    }

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

TEST(OvenControllerTest, SopdtTracksRampAndSoak) {
    const auto plantP = sopdtPlantParams();
    OvenControllerConfig cfg;
    cfg.modelOrder = OvenModelOrder::Sopdt;
    cfg.modelSopdt = plantP;                    // perfect model knowledge
    cfg.model = plantP.toFopdtParams();         // FOPDT projection for governor init
    cfg.adaptParams = false;
    cfg.sensorTau = 5.0;

    // Gentler, longer profile: the SOPDT plant is slow (tau_tot ~ 500 s)
    // and stage 1 runs far ahead of the output on ramps, so settling
    // after each maneuver takes several hundred seconds.
    TemperatureProfile prof(25.0);
    prof.addRampSoak(80.0, 0.12, 400.0);
    prof.addRampSoak(55.0, 0.12, 300.0);

    const auto res = simulateSopdt(plantP, cfg, prof, 2000.0, 0.5);
    ASSERT_FALSE(res.samples.empty());
    for (const auto& s : res.samples) {
        ASSERT_TRUE(std::isfinite(s.u));
        EXPECT_GE(s.u, -1e-9);
        EXPECT_LE(s.u, 1.0 + 1e-9);
    }
    // Bounded tracking on a 500 s-lag cascade: the FF does the bulk work,
    // feedback trims the residual — a few °C is expected, not <1.
    EXPECT_LT(res.soakErr, 5.0);
    EXPECT_LT(res.finalErr, 6.0);
    EXPECT_EQ(res.lastStatus.modelOrder, OvenModelOrder::Sopdt);
    EXPECT_GT(res.lastStatus.tau2, 50.0);
}

TEST(OvenControllerTest, SopdtStatusReportsMethods) {
    const auto plantP = sopdtPlantParams();
    OvenControllerConfig cfg;
    cfg.modelOrder = OvenModelOrder::Sopdt;
    cfg.modelSopdt = plantP;
    cfg.ffMethod = OvenFeedforwardMethod::DominantLag;
    cfg.estimateMethod = OvenEstimateMethod::SopdtOe;
    cfg.tuneMethod = OvenTuneMethod::ImcPi;
    cfg.adaptParams = false;

    OvenThermalModelSOPDT plant(plantP);
    plant.reset();
    OvenController ctrl(cfg);
    TemperatureProfile prof(25.0);
    prof.addRampSoak(80.0, 0.2, 200.0);
    ctrl.setProfile(prof);

    ControllerInput in;
    in.dt = 0.5;
    in.reference = 25.0;
    for (int i = 0; i < 200; ++i) {
        in.measured = plant.temperature();
        const auto out = ctrl.compute(in);
        plant.step(out.control, 0.5);
    }
    const auto st = ctrl.status();
    EXPECT_EQ(st.modelOrder, OvenModelOrder::Sopdt);
    EXPECT_EQ(st.ffMethod, OvenFeedforwardMethod::DominantLag);
    EXPECT_EQ(st.estimateMethod, OvenEstimateMethod::SopdtOe);
    EXPECT_EQ(st.tuneMethod, OvenTuneMethod::ImcPi);
}


// ---------------------------------------------------------------------------
// Voltage-domain sensor noise -> nonlinear temperature-domain noise
// ---------------------------------------------------------------------------

// mean / stddev / Fisher-Pearson skewness of a residual sample.
struct NoiseStats {
    double mean{0.0}, stddev{0.0}, skew{0.0};
    int n{0};
};
NoiseStats noiseStats(const std::vector<double>& resid) {
    NoiseStats st;
    st.n = static_cast<int>(resid.size());
    if (st.n == 0) return st;
    double m = 0.0;
    for (double v : resid) m += v;
    m /= st.n;
    st.mean = m;
    double v2 = 0.0, v3 = 0.0;
    for (double x : resid) {
        const double d = x - m;
        v2 += d * d;
        v3 += d * d * d;
    }
    const double sd = std::sqrt(v2 / st.n);
    st.stddev = sd;
    st.skew = sd > 0.0 ? (v3 / st.n) / (sd * sd * sd) : 0.0;
    return st;
}

// Collect measurement residuals (measured - true) at a fixed true
// temperature through a sensor model's measure().
template <typename Sensor>
NoiseStats sensorNoiseAt(const Sensor& sensor, double trueT, int nSamples,
                         std::mt19937& rng) {
    std::vector<double> resid;
    resid.reserve(nSamples);
    for (int i = 0; i < nSamples; ++i)
        resid.push_back(sensor.measure(trueT, rng) - trueT);
    return noiseStats(resid);
}

TEST(SensorNoiseModelTest, TypeKThermocoupleRoundTrip) {
    // Forward/inverse must be consistent: T(E(T)) == T over the oven range.
    for (double T : {0.0, 25.0, 100.0, 200.0, 400.0}) {
        EXPECT_NEAR(TypeKThermocouple::temp(TypeKThermocouple::emf(T)), T,
                    0.05);
    }
}

TEST(SensorNoiseModelTest, TypeKIsNearLinearInOvenRange) {
    // Type K's Seebeck coefficient is almost constant over 0..400 C, so the
    // inverse is locally linear and voltage noise stays ~symmetric: the
    // induced temperature skew is negligible. This documents the contrast
    // with the NTC case below.
    TypeKThermocouple tc;   // sigmaV = 0.01 mV -> ~0.24 C
    std::mt19937 rng(11);
    for (double T : {50.0, 150.0, 300.0}) {
        const auto st = sensorNoiseAt(tc, T, 60000, rng);
        EXPECT_GT(st.stddev, 0.1);
        EXPECT_LT(st.stddev, 0.5);
        EXPECT_NEAR(st.mean, 0.0, 0.05);       // ~unbiased
        EXPECT_LT(std::abs(st.skew), 0.05);     // ~symmetric
    }
}

TEST(SensorNoiseModelTest, NtcProducesSkewedTemperatureDependentNoise) {
    // The NTC divider inverse is strongly curved, so Gaussian voltage noise
    // becomes skewed temperature noise whose gain grows with temperature.
    // Use a larger sigmaV here so the skew is clearly resolvable above the
    // finite-sample noise floor.
    NtcThermistor ntc;
    ntc.sigmaV = 0.010;   // 10 mV ADC noise -> ~1-3 C (noisy front end)
    std::mt19937 rng(22);

    const auto lo = sensorNoiseAt(ntc, 40.0, 80000, rng);
    const auto hi = sensorNoiseAt(ntc, 200.0, 80000, rng);

    // Spread is temperature-dependent: the divider sensitivity rises with T.
    EXPECT_GT(hi.stddev, lo.stddev);
    EXPECT_LT(hi.stddev, 8.0);                 // sanity bound
    // Convex inverse -> positive skew, resolvable at this noise level.
    EXPECT_GT(hi.skew, 0.05);
    // And the noise is large enough to matter to the loop (not negligible).
    EXPECT_GT(hi.stddev, 0.5);
}

TEST(OvenControllerTest, FopdtTracksWithNtcSensorNoise) {
    const auto p = plantParams();
    NtcThermistor ntc;                 // sigmaV = 3 mV -> ~0.2-0.5 C
    OvenControllerConfig cfg;
    cfg.model = p;
    cfg.adaptParams = false;
    cfg.sensorTau = 8.0;               // heavier filtering for noisy sensor

    const auto sensorFn = [&ntc](double T, std::mt19937& rng) {
        return ntc.measure(T, rng);
    };
    const auto res = simulate(p, cfg, testProfile(), 800.0, 0.5, 0.0, {},
                              sensorFn);
    EXPECT_LT(res.soakErr, 2.5);
    EXPECT_LT(res.finalErr, 2.5);
    EXPECT_LT(maxErrIn(res, 60.0, 240.0), 5.0);
}

TEST(OvenControllerTest, SopdtTracksWithNtcSensorNoise) {
    const auto plantP = sopdtPlantParams();
    NtcThermistor ntc;
    OvenControllerConfig cfg;
    cfg.modelOrder = OvenModelOrder::Sopdt;
    cfg.modelSopdt = plantP;
    cfg.model = plantP.toFopdtParams();
    cfg.adaptParams = false;
    cfg.sensorTau = 8.0;

    TemperatureProfile prof(25.0);
    prof.addRampSoak(80.0, 0.12, 400.0);
    prof.addRampSoak(55.0, 0.12, 300.0);

    const auto sensorFn = [&ntc](double T, std::mt19937& rng) {
        return ntc.measure(T, rng);
    };
    const auto res = simulateSopdt(plantP, cfg, prof, 2000.0, 0.5, 0.0, {},
                                   sensorFn);
    ASSERT_FALSE(res.samples.empty());
    for (const auto& s : res.samples) {
        ASSERT_TRUE(std::isfinite(s.u));
        EXPECT_GE(s.u, -1e-9);
        EXPECT_LE(s.u, 1.0 + 1e-9);
    }
    // Slightly looser than the clean SOPDT case (soakErr<5, finalErr<6):
    // the skewed sensor noise adds a small tracking penalty.
    EXPECT_LT(res.soakErr, 6.5);
    EXPECT_LT(res.finalErr, 7.5);
}

TEST(SOPDTParameterEstimatorTest, OeUnderNtcSensorNoise) {
    // The output-error fit averages over a multi-tau window, so the small
    // skew/bias of realistic thermistor noise should not prevent recovery
    // of the identifiable quantities (DC gain K and the lag set).
    const auto p = sopdtPlantParams();
    OvenThermalModelSOPDT plant(p);
    plant.reset();
    NtcThermistor ntc;                 // sigmaV = 3 mV -> ~0.2-0.5 C
    std::mt19937 rng(99);

    SOPDTEstimatorConfig ec;
    ec.windowTime = 300.0;
    ec.windowLagFactor = 4.0;
    ec.spinupFactor = 4.0;
    ec.fitInterval = 15.0;
    ec.simStride = 2.0;
    ec.gridSize = 9;
    ec.smoothing = 0.4;
    SOPDTParameterEstimator est(ec);
    est.setPrior(0.8, 0.006, 200.0, 0.0);

    std::deque<std::pair<double, double>> hist;
    const double dt = 0.5, delay = p.delay;
    for (int i = 0; i < 12000; ++i) {         // 6000 s
        const double t = i * dt;
        double u = 0.4 + 0.3 * std::sin(2.0 * 3.14159265 * t / 47.0)
                  + 0.15 * std::sin(2.0 * 3.14159265 * t / 13.0)
                  + 0.10 * std::copysign(1.0, std::sin(2.0 * 3.14159265 * t / 29.0));
        u = std::clamp(u, 0.0, 1.0);
        hist.emplace_back(t, u);
        while (hist.size() > 100 && t - hist.front().first > delay + 1.0)
            hist.pop_front();
        double uD = 0.0;
        for (auto& [ti, ui] : hist)
            if (ti <= t - delay) uD = ui;
        const double Tnow = plant.temperature();
        const double Tmeas = ntc.measure(Tnow, rng);   // voltage->T noise
        plant.step(u, dt);
        est.update(uD, Tmeas, 25.0, dt);
    }

    EXPECT_GT(est.updatesUsed(), 5);
    const double Ktrue = p.alpha / p.beta;         // 200
    EXPECT_NEAR(est.gain(), Ktrue, 0.35 * Ktrue);
    double lags[2] = {est.tau1(), est.tau2()};
    double truth[2] = {1.0 / p.beta, p.tau2};      // {200, 300}
    std::sort(lags, lags + 2);
    std::sort(truth, truth + 2);
    EXPECT_NEAR(lags[0], truth[0], 0.5 * truth[0]);
    EXPECT_NEAR(lags[1], truth[1], 0.5 * truth[1]);
}

TEST(ThermalParameterEstimatorTest, OpenLoopUnderNtcSensorNoise) {
    // FOPDT integral estimator under the same voltage-domain noise: window
    // averaging should keep {alpha,beta,d} recovery essentially intact.
    auto p = plantParams();
    p.alpha = 1.3; p.beta = 0.006;
    OvenThermalModel plant(p);
    plant.reset();
    NtcThermistor ntc;
    std::mt19937 rng(55);

    EstimatorConfig ec;
    ec.windowTime = 15.0;
    ec.memTime = 150.0;
    ec.smoothing = 0.5;
    ThermalParameterEstimator est(ec);
    est.setPrior(1.0, 0.005, 0.0);

    std::deque<std::pair<double, double>> hist;
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
        const double Tmeas = ntc.measure(plant.temperature(), rng);
        est.update(uD, Tmeas, 25.0, dt);
    }
    EXPECT_NEAR(est.alpha(), 1.3, 0.15);
    EXPECT_NEAR(est.beta(), 0.006, 0.002);
    EXPECT_NEAR(est.disturbance(), 0.0, 0.15);
    EXPECT_GT(est.updatesUsed(), 500);
}
