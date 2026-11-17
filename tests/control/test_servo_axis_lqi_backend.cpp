/**
 * @file test_servo_axis_lqi_backend.cpp
 * @brief Closed-loop tests for ServoAxisLQIBackend: a torque-mode servo
 *        axis controlled by LQI state feedback with a [θ, ω, τ_d]
 *        steady-state Kalman observer, physics feedforward and optional
 *        disturbance feedforward.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <functional>
#include <vector>

#include "tether/control/motion/ServoAxisLQIBackend.hpp"

namespace {

using namespace tether::control;
using namespace tether::control::motion;

// ---------------------------------------------------------------------------
// Simulated torque-mode plant (the "real" axis):
//   J·θ̈ = Kt·u_d − b·ω − τ_c·sgn(ω) − τ_load
// Optional input dead time via a FIFO tap line.
// ---------------------------------------------------------------------------
struct AxisPlant {
    ServoAxisParams p;
    double theta{0.0};
    double omega{0.0};
    double loadTorque{0.0};
    std::deque<double> delayLine;

    explicit AxisPlant(const ServoAxisParams& plant, int delayTaps = 0)
        : p(plant), delayLine(static_cast<size_t>(delayTaps), 0.0) {}

    /** Apply torque command u for dt, return measured (true) position. */
    double step(double u, double dt) {
        delayLine.push_back(u);
        const double ud = delayLine.front();
        delayLine.pop_front();
        const double sgn = (omega > 1e-9) ? 1.0 : (omega < -1e-9 ? -1.0 : 0.0);
        const double alpha =
            (p.torqueConstant * ud - p.damping * omega -
             p.coulombFriction * sgn - loadTorque) / p.inertia;
        omega += alpha * dt;
        theta += omega * dt;
        return theta;
    }
};

ServoAxisParams makePlant() {
    ServoAxisParams p;
    p.inertia = 2e-4;           // 0.2 g·m²
    p.damping = 5e-4;           // viscous
    p.torqueConstant = 0.05;    // N·m per unit command
    p.coulombFriction = 0.0;
    p.delay = 0.0;
    p.uMin = -2.0;
    p.uMax = 2.0;
    return p;
}

ServoAxisTuning makeTuning(const ServoAxisParams& p) {
    auto t = ServoAxisTuning::fromBandwidth(p, 60.0, 1.0);  // ~10 Hz
    t.qAccel = 1e2;
    t.qDist = 1e-2;
    t.rMeas = 1e-10;
    return t;
}

constexpr double kDt = 1e-3;   // 1 kHz servo cycle

// Run a closed loop: plant + backend. `ref(t)` gives position reference;
// optional vel/acc feedforward hooks and load-torque schedule.
struct LoopResult {
    std::vector<double> t, ref, meas, u, err, velHat, tauHat;
};

LoopResult runLoop(ServoAxisLQIBackend& backend, AxisPlant& plant,
                   double duration,
                   const std::function<double(double)>& ref,
                   bool feedforward = true,
                   const std::function<double(double)>& load =
                       std::function<double(double)>()) {
    LoopResult res;
    const int n = static_cast<int>(duration / kDt);
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        if (load) plant.loadTorque = load(t);

        ControllerInput in;
        in.reference = ref(t);
        in.measured = plant.theta;
        in.dt = kDt;
        if (feedforward) {
            // Trajectory derivatives for physics FF.
            const double h = 1e-4;
            in.referenceDerivative = (ref(t + h) - ref(t - h)) / (2 * h);
            in.referenceAccel =
                (ref(t + h) - 2 * ref(t) + ref(t - h)) / (h * h);
        }
        ControllerOutput out = backend.compute(in);
        const double y = plant.step(out.control, kDt);

        res.t.push_back(t);
        res.ref.push_back(in.reference);
        res.meas.push_back(y);
        res.u.push_back(out.control);
        res.err.push_back(out.error);
        res.velHat.push_back(backend.observer().velocity());
        res.tauHat.push_back(backend.estimatedDisturbance());
    }
    return res;
}

// ---------------------------------------------------------------------------
// Design / validation
// ---------------------------------------------------------------------------

TEST(ServoAxisLQI, DesignProducesGains) {
    ServoAxisLQIBackend backend(makePlant(), makeTuning(makePlant()), kDt);
    double Kx[2] = {0, 0}, Ki[1] = {0};
    ASSERT_TRUE(backend.getGain(Kx, Ki));
    EXPECT_GT(Kx[0], 0.0);
    EXPECT_GT(Kx[1], 0.0);
    // xi accumulates the tracking error (r − θ̂); u = −Kx·z − Ki·xi, so a
    // NEGATIVE Ki is what produces restorative torque. Sign convention
    // verified by the zero-steady-state-error test below.
    EXPECT_LT(Ki[0], 0.0);
}

TEST(ServoAxisLQI, RejectsInvalidParameterPush) {
    ServoAxisLQIBackend backend(makePlant(), makeTuning(makePlant()), kDt);
    EXPECT_FALSE(backend.updateAxisParameters(0.0, 1e-3, 0.05, 0.0));  // J=0
    EXPECT_FALSE(backend.updateAxisParameters(1e-4, -1.0, 0.05, 0.0)); // b<0
    EXPECT_FALSE(backend.updateAxisParameters(1e-4, 1e-3, 0.0, 0.0));  // Kt=0
    EXPECT_TRUE(backend.updateAxisParameters(1e-4, 1e-3, 0.05, 0.01));
}

// ---------------------------------------------------------------------------
// Tracking
// ---------------------------------------------------------------------------

TEST(ServoAxisLQI, StepTracksWithZeroSteadyStateError) {
    ServoAxisParams p = makePlant();
    ServoAxisLQIBackend backend(p, makeTuning(p), kDt);
    AxisPlant plant(p);
    plant.loadTorque = 0.02;  // constant 20 mN·m load

    auto res = runLoop(backend, plant, 4.0, [](double t) {
        return t < 0.2 ? 0.0 : 1.0;
    });
    EXPECT_NEAR(res.meas.back(), 1.0, 0.01);
    // The observer should have identified the constant load torque.
    EXPECT_NEAR(res.tauHat.back(), 0.02, 0.005);
}

TEST(ServoAxisLQI, TrajectoryFeedforwardReducesTrackingError) {
    ServoAxisParams p = makePlant();
    const double A = 0.5, w = 2.0 * M_PI * 0.5;  // 0.5 Hz sinusoid
    auto ref = [A, w](double t) { return A * std::sin(w * t); };

    ServoAxisLQIBackend withFf(p, makeTuning(p), kDt);
    AxisPlant plantA(p);
    auto resFf = runLoop(withFf, plantA, 4.0, ref, true);
    double maxErrFf = 0.0;
    for (size_t i = resFf.err.size() / 2; i < resFf.err.size(); ++i)
        maxErrFf = std::max(maxErrFf, std::abs(resFf.err[i]));

    ServoAxisLQIBackend noFf(p, makeTuning(p), kDt);
    AxisPlant plantB(p);
    auto resNo = runLoop(noFf, plantB, 4.0, ref, false);
    double maxErrNo = 0.0;
    for (size_t i = resNo.err.size() / 2; i < resNo.err.size(); ++i)
        maxErrNo = std::max(maxErrNo, std::abs(resNo.err[i]));

    EXPECT_LT(maxErrFf, 0.15 * A);
    EXPECT_LT(maxErrFf, 0.5 * maxErrNo);
}

// ---------------------------------------------------------------------------
// Disturbance handling
// ---------------------------------------------------------------------------

TEST(ServoAxisLQI, DisturbanceFeedforwardCancelsLoadStep) {
    ServoAxisParams p = makePlant();
    auto load = [](double t) { return t >= 1.5 ? 0.03 : 0.0; };

    ServoAxisLQIBackend dobOn(p, makeTuning(p), kDt);
    dobOn.setDisturbanceFeedforward(true);
    AxisPlant plantA(p);
    auto resOn = runLoop(dobOn, plantA, 4.0,
                         [](double) { return 0.5; }, true, load);

    ServoAxisLQIBackend dobOff(p, makeTuning(p), kDt);
    dobOff.setDisturbanceFeedforward(false);
    AxisPlant plantB(p);
    auto resOff = runLoop(dobOff, plantB, 4.0,
                          [](double) { return 0.5; }, true, load);

    // Peak excursion of the TRUE position after the load step should be
    // smaller with DOB (estimation transient can make θ̂-based error
    // misleading, so compare ref − measured, not the observer error).
    double peakOn = 0.0, peakOff = 0.0;
    for (size_t i = 0; i < resOn.err.size(); ++i) {
        if (resOn.t[i] < 1.5) continue;
        peakOn = std::max(peakOn, std::abs(resOn.ref[i] - resOn.meas[i]));
        peakOff = std::max(peakOff, std::abs(resOff.ref[i] - resOff.meas[i]));
    }
    EXPECT_LT(peakOn, peakOff);
    // Both recover to the reference (integral action).
    EXPECT_NEAR(resOn.meas.back(), 0.5, 0.01);
    EXPECT_NEAR(resOff.meas.back(), 0.5, 0.01);
    // DOB channel converged on the applied load.
    EXPECT_NEAR(resOn.tauHat.back(), 0.03, 0.005);
}

TEST(ServoAxisLQI, CoulombFrictionFeedforwardCompensates) {
    ServoAxisParams p = makePlant();
    p.coulombFriction = 0.03;  // 30 mN·m — significant vs Kt·uMax=0.1

    // Backend knows the friction → FF compensates.
    ServoAxisLQIBackend withComp(p, makeTuning(p), kDt);
    AxisPlant plantA(p);
    // Slow constant-velocity move — Coulomb dominates.
    auto resA = runLoop(withComp, plantA, 3.0,
                        [](double t) { return 0.2 * t; });
    double tailErrA = 0.0;
    for (size_t i = resA.err.size() / 2; i < resA.err.size(); ++i)
        tailErrA += std::abs(resA.err[i]);
    tailErrA /= (resA.err.size() / 2);

    // Backend told friction = 0 → misfit left to the integrator.
    ServoAxisParams pNoFric = p;
    pNoFric.coulombFriction = 0.0;
    ServoAxisLQIBackend noComp(pNoFric, makeTuning(pNoFric), kDt);
    AxisPlant plantB(p);
    auto resB = runLoop(noComp, plantB, 3.0,
                        [](double t) { return 0.2 * t; });
    double tailErrB = 0.0;
    for (size_t i = resB.err.size() / 2; i < resB.err.size(); ++i)
        tailErrB += std::abs(resB.err[i]);
    tailErrB /= (resB.err.size() / 2);

    EXPECT_LT(tailErrA, tailErrB);
}

// ---------------------------------------------------------------------------
// Online retuning
// ---------------------------------------------------------------------------

TEST(ServoAxisLQI, ParameterPushResynthesizesAndStaysStable) {
    ServoAxisParams p = makePlant();
    ServoAxisLQIBackend backend(p, makeTuning(p), kDt);

    // Plant actually has 2x the modelled inertia; push corrects it.
    ServoAxisParams pTrue = p;
    pTrue.inertia = 2.0 * p.inertia;
    AxisPlant plant(pTrue);

    // First run with wrong model — tracks but suboptimal.
    auto resBad = runLoop(backend, plant, 1.0, [](double) { return 0.5; });

    // Push the identified (correct) inertia.
    ASSERT_TRUE(backend.updateAxisParameters(
        pTrue.inertia, pTrue.damping, pTrue.torqueConstant,
        pTrue.coulombFriction));
    double Kx[2], Ki[1];
    ASSERT_TRUE(backend.getGain(Kx, Ki));

    auto res = runLoop(backend, plant, 3.0, [](double t) {
        return t < 0.3 ? 0.5 : 1.0;
    });
    EXPECT_NEAR(res.meas.back(), 1.0, 0.01);
}

// ---------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------

TEST(ServoAxisLQI, AntiWindupRecoversFromSaturation) {
    ServoAxisParams p = makePlant();
    p.uMin = -0.05;  // tight authority — deep saturation on a big step
    p.uMax = 0.05;
    ServoAxisLQIBackend backend(p, makeTuning(p), kDt);
    AxisPlant plant(p);

    auto res = runLoop(backend, plant, 6.0, [](double t) {
        return t < 0.2 ? 0.0 : 2.0;  // way beyond what torque allows quickly
    });
    EXPECT_NEAR(res.meas.back(), 2.0, 0.02);
    // Output stayed inside the torque limits the whole time.
    for (double u : res.u) {
        EXPECT_LE(u, p.uMax + 1e-9);
        EXPECT_GE(u, p.uMin - 1e-9);
    }
}

TEST(ServoAxisLQI, AntiWindupNoneOvershootsMoreThanConditional) {
    ServoAxisParams p = makePlant();
    p.uMin = -0.05;
    p.uMax = 0.05;
    auto step = [](double t) { return t < 0.2 ? 0.0 : 2.0; };

    ServoAxisLQIBackend none(p, makeTuning(p), kDt);
    none.setAntiWindup(ServoAxisAWMode::kNone);
    AxisPlant plantA(p);
    auto resNone = runLoop(none, plantA, 6.0, step);

    ServoAxisLQIBackend cond(p, makeTuning(p), kDt);
    cond.setAntiWindup(ServoAxisAWMode::kConditional);
    AxisPlant plantB(p);
    auto resCond = runLoop(cond, plantB, 6.0, step);

    auto peakOvershoot = [](const LoopResult& r) {
        double m = 0.0;
        for (double v : r.meas) m = std::max(m, v - 2.0);
        return m;
    };
    const double ovNone = peakOvershoot(resNone);
    const double ovCond = peakOvershoot(resCond);
    // Disabling dynamic anti-windup must not beat conditional
    // integration — that's the whole point of the mechanism. With
    // bound-only clamping the integrator parks at its limit and the
    // loop settles at a windup equilibrium with a permanent offset.
    EXPECT_LE(ovCond, ovNone + 1e-6);
    EXPECT_GT(ovNone, 0.2);
    EXPECT_NEAR(resCond.meas.back(), 2.0, 0.05);
}

TEST(ServoAxisLQI, BackCalculationRecoversFromSaturation) {
    ServoAxisParams p = makePlant();
    p.uMin = -0.05;
    p.uMax = 0.05;
    ServoAxisLQIBackend backend(p, makeTuning(p), kDt);
    backend.setAntiWindup(ServoAxisAWMode::kBackCalculation);
    backend.setAntiWindupTrackingTime(5e-3);
    AxisPlant plant(p);

    auto res = runLoop(backend, plant, 6.0, [](double t) {
        return t < 0.2 ? 0.0 : 2.0;
    });
    // Back-calculation tracks the integrator to the saturated state —
    // overshoot must stay bounded and the loop must settle.
    double overshoot = 0.0;
    for (double v : res.meas) overshoot = std::max(overshoot, v - 2.0);
    EXPECT_LT(overshoot, 0.5);
    EXPECT_NEAR(res.meas.back(), 2.0, 0.05);
    for (double u : res.u) {
        EXPECT_LE(u, p.uMax + 1e-9);
        EXPECT_GE(u, p.uMin - 1e-9);
    }
}

TEST(ServoAxisLQI, OutputRateLimitBoundsSlew) {
    ServoAxisParams p = makePlant();
    const double rate = 300.0;  // u/s — limits the spike, above loop demand
    ServoAxisLQIBackend backend(p, makeTuning(p), kDt);
    backend.setOutputRateLimit(rate);
    AxisPlant plant(p);

    auto res = runLoop(backend, plant, 3.0, [](double t) {
        return t < 0.2 ? 0.0 : 1.0;
    });
    for (size_t i = 1; i < res.u.size(); ++i) {
        EXPECT_LE(std::abs(res.u[i] - res.u[i - 1]),
                  rate * kDt + 1e-9);
    }
    EXPECT_NEAR(res.meas.back(), 1.0, 0.02);
}

TEST(ServoAxisLQI, InputDelayStaysStable) {
    ServoAxisParams p = makePlant();
    p.delay = 2.0 * kDt;  // 2 ms EtherCAT-ish dead time
    ServoAxisLQIBackend backend(p, makeTuning(p), kDt);
    const int taps = static_cast<int>(std::lround(p.delay / kDt));
    AxisPlant plant(p, taps);

    auto res = runLoop(backend, plant, 4.0, [](double t) {
        return t < 0.3 ? 0.0 : 0.8;
    });
    EXPECT_NEAR(res.meas.back(), 0.8, 0.02);
}

TEST(ServoAxisLQI, EncoderQuantizationStillTracks) {
    ServoAxisParams p = makePlant();
    auto tuning = makeTuning(p);
    const double res = 1e-3;              // 1 mrad encoder resolution
    tuning.rMeas = res * res / 12.0;      // quantization variance
    ServoAxisLQIBackend backend(p, tuning, kDt);
    AxisPlant plant(p);

    const int n = static_cast<int>(4.0 / kDt);
    double lastMeas = 0.0;
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        ControllerInput in;
        in.reference = t < 0.3 ? 0.0 : 0.6;
        in.measured = std::round(plant.theta / res) * res;  // quantized
        in.dt = kDt;
        ControllerOutput out = backend.compute(in);
        lastMeas = plant.step(out.control, kDt);
    }
    EXPECT_NEAR(lastMeas, 0.6, 0.02);
    EXPECT_NEAR(backend.observer().position(), lastMeas, 0.01);
}

// ---------------------------------------------------------------------------
// Observer internals
// ---------------------------------------------------------------------------

TEST(ServoAxisLQI, ObserverConvergesOnVelocityAndDisturbance) {
    ServoAxisParams p = makePlant();
    ServoAxisObserver obs;
    obs.design(p, kDt, 1e2, 1e-2, 1e-10);
    obs.reset(0.0);

    AxisPlant plant(p);
    plant.loadTorque = 0.025;
    for (int i = 0; i < 2000; ++i) {
        obs.predict(0.5, kDt);          // constant torque command
        obs.correct(plant.step(0.5, kDt));
    }
    EXPECT_NEAR(obs.velocity(), plant.omega, std::abs(plant.omega) * 0.05 + 1e-3);
    EXPECT_NEAR(obs.disturbanceTorque(), 0.025, 0.005);
}

} // namespace
