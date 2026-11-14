#include "tether/control/thermal/OvenController.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace tether::control::thermal {

namespace {
constexpr size_t kMaxInputHistory = 8192;
}

OvenController::OvenController(const OvenControllerConfig& cfg)
    : m_cfg(cfg),
      m_model(cfg.model),
      m_est(cfg.estimator),
      m_gov(cfg.governor) {
    installBackend(cfg.feedbackBackend);

    // Initial PI gains: explicit config wins, otherwise SIMC from the
    // nominal physical model. (Meaningless for the LQI backend, which
    // synthesizes its own gains — applyPiGains guards on m_pidBackend.)
    double kp = cfg.kp, ki = cfg.ki;
    if (kp <= 0.0 || ki <= 0.0) {
        const auto [gkp, gki] = simcPiGains(cfg.model.gain(), cfg.model.tau(),
                                            cfg.model.delay, cfg.simcTauC);
        if (kp <= 0.0) kp = gkp;
        if (ki <= 0.0) ki = gki;
    }
    applyPiGains(kp, ki);
    m_lastKp = kp;
    m_lastKi = ki;
    m_lastAlpha = cfg.model.alpha;
    m_lastBeta = cfg.model.beta;

    // Outer loop: hard actuator bounds (safety net; computeImpl already
    // clamps). Rate limiting is applied inside computeImpl so the recorded
    // input history matches the applied output exactly.
    SaturationLimits lim;
    lim.outputMin = cfg.model.uMin;
    lim.outputMax = cfg.model.uMax;
    setSaturationLimits(lim);
}

void OvenController::installBackend(OvenFeedbackBackendKind kind) {
    m_backendKind = kind;
    if (kind == OvenFeedbackBackendKind::LQI) {
        auto lb = std::make_unique<OvenLQIBackend>(
            m_cfg.coupled, m_cfg.lqiQxSensor, m_cfg.lqiQxProduct,
            m_cfg.lqiQi, m_cfg.lqiR, m_cfg.observerQProc,
            m_cfg.observerRMeas, m_cfg.backendDt);
        m_feedbackAdaptive = lb.get();
        m_pidBackend = nullptr;
        m_feedback = std::move(lb);
    } else {
        auto pid = std::make_unique<PIDController>();
        m_pidBackend = pid.get();
        m_feedbackAdaptive = nullptr;
        m_feedback = std::move(pid);
    }
}

void OvenController::setFeedbackBackend(std::unique_ptr<ControllerBase> backend) {
    if (!backend) {
        installBackend(m_cfg.feedbackBackend);
        return;
    }
    m_feedbackAdaptive = dynamic_cast<IOvenAdaptiveFeedback*>(backend.get());
    m_pidBackend = dynamic_cast<PIDController*>(backend.get());
    if (m_pidBackend) {
        m_backendKind = OvenFeedbackBackendKind::PI;
        applyPiGains(m_lastKp, m_lastKi);   // carry over current tuning
    } else if (m_feedbackAdaptive) {
        m_backendKind = OvenFeedbackBackendKind::LQI;
    }
    m_feedback = std::move(backend);
}

const char* OvenController::feedbackBackendName() const {
    if (m_feedbackAdaptive) return m_feedbackAdaptive->backendName();
    if (m_pidBackend) return "SIMC-PI";
    return m_feedback ? m_feedback->getName() : "none";
}

void OvenController::setConfig(const OvenControllerConfig& cfg) {
    const bool kindChanged =
        (cfg.feedbackBackend != m_backendKind) ||
        (m_feedbackAdaptive == nullptr && m_pidBackend == nullptr);
    m_cfg = cfg;
    m_est.setConfig(cfg.estimator);
    if (kindChanged) installBackend(cfg.feedbackBackend);
    SaturationLimits lim;
    lim.outputMin = cfg.model.uMin;
    lim.outputMax = cfg.model.uMax;
    setSaturationLimits(lim);
}

void OvenController::setProfile(const TemperatureProfile& profile) {
    m_profile = profile;
    m_hasProfile = !profile.empty();
    m_time = 0.0;
    m_uHist.clear();
    if (m_hasProfile) {
        m_gov.reset(m_profile.value(0.0));
    }
}

void OvenController::clearProfile() {
    m_hasProfile = false;
    m_profile.clear();
}

void OvenController::setAmbient(double ambientC) {
    m_model.params().ambient = ambientC;
    auto gc = m_gov.config();
    gc.ambient = ambientC;
    m_gov.setConfig(gc);
}

void OvenController::setDisturbancePrior(double d) {
    m_model.params().disturbance = d;
    m_est.setPrior(m_est.alpha(), m_est.beta(), d);
}

void OvenController::setCouplingDisturbance(double d) {
    m_extDisturb = d;
}

void OvenController::setModelParams(const OvenModelParams& params) {
    m_model.setParams(params);
    m_model.params().disturbance = params.disturbance;
    m_est.setPrior(params.alpha, params.beta, params.disturbance);
    auto gc = m_cfg.governor;
    gc.alpha = params.alpha;
    gc.beta = params.beta;
    gc.disturbance = params.disturbance;
    gc.ambient = params.ambient;
    gc.uMin = params.uMin;
    gc.uMax = params.uMax;
    m_gov.setConfig(gc);
    // Keep the outer safety saturation in sync with the new u-bounds.
    SaturationLimits lim = getSaturationLimits();
    lim.outputMin = params.uMin;
    lim.outputMax = params.uMax;
    setSaturationLimits(lim);
}

void OvenController::applyAutotune(const OvenAutotuneResult& result) {
    if (!result.success) return;
    setModelParams(result.params);
    applyPiGains(result.kp, result.ki);
    m_lastKp = result.kp;
    m_lastKi = result.ki;
    m_lastAlpha = result.params.alpha;
    m_lastBeta = result.params.beta;
    if (m_feedbackAdaptive) {
        m_feedbackAdaptive->updateThermalParameters(result.params.alpha,
                                                    result.params.beta,
                                                    result.params.disturbance);
    }
    m_lastRetuneWindows = m_est.updatesUsed();
    m_lastRetuneTime = m_time;
}

OvenControllerStatus OvenController::status() const {
    OvenControllerStatus s;
    s.time = m_time;
    s.reference = m_hasProfile ? m_gov.value() : m_lastRef;
    s.measuredFiltered = m_Tf;
    s.alpha = m_est.alpha();
    s.beta = m_est.beta();
    s.disturbance = m_est.disturbance();
    s.windowsUsed = m_est.updatesUsed();
    s.overTemp = m_overTemp;
    s.adapting = m_cfg.adaptParams;
    s.backendName = feedbackBackendName();
    s.feedforward = m_lastOutput.feedforward;
    s.feedback = m_lastUfb;
    s.output = m_lastOutput.control;
    return s;
}

ControllerOutput OvenController::computeImpl(const ControllerInput& input) {
    ControllerOutput out;
    const double dt = input.dt;
    if (dt <= 0.0) {
        out.control = m_lastU;
        return out;
    }

    const OvenModelParams& mp = m_model.params();
    const double ambient = mp.ambient;
    const double uMin = mp.uMin, uMax = mp.uMax;

    // ---- 1. Sensor filtering (exact ZOH first-order) ----
    if (!m_TfInit) {
        m_Tf = input.measured;
        m_TfInit = true;
    } else if (m_cfg.sensorTau > 0.0) {
        const double a = 1.0 - std::exp(-dt / m_cfg.sensorTau);
        m_Tf += a * (input.measured - m_Tf);
    } else {
        m_Tf = input.measured;
    }

    // ---- 2. Governed reference ----
    // Known exogenous disturbance (e.g. neighbor-zone coupling): the
    // estimator's disturbance estimate already contains its steady-state
    // value with window lag, so only the deviation from a slow EMA is
    // added — transient-leading without steady-state double counting.
    if (!m_extDisturbInit) {
        m_extDisturbAvg = m_extDisturb;
        m_extDisturbInit = true;
    } else {
        const double tau = std::max(1.0, m_cfg.estimator.windowTime);
        const double a = 1.0 - std::exp(-dt / tau);
        m_extDisturbAvg += a * (m_extDisturb - m_extDisturbAvg);
    }
    const double ffDisturb =
        mp.disturbance + (m_extDisturb - m_extDisturbAvg);

    double rHat, rDot;
    if (m_hasProfile) {
        // Keep governor physics in sync with (possibly adapted) model.
        GovernorConfig gc = m_cfg.governor;
        gc.alpha = mp.alpha;
        gc.beta = mp.beta;
        gc.disturbance = ffDisturb;
        gc.ambient = ambient;
        gc.uMin = uMin;
        gc.uMax = uMax;
        m_gov.setConfig(gc);
        m_gov.update(m_time, m_profile, dt);
        rHat = m_gov.value();
        rDot = m_gov.rate();
    } else {
        rHat = input.reference;
        rDot = 0.0;
    }
    m_lastRef = rHat;

    // ---- 3. Model-inverse feedforward (previewed by dead time) ----
    double rFf, rDotFf;
    if (m_hasProfile) {
        // Preview the *governed* trajectory so FF never demands an
        // infeasible rate; linear extrapolation over θ is exact on ramps.
        rFf = m_gov.previewValue(mp.delay);
        rDotFf = rDot;
    } else {
        rFf = input.reference;
        rDotFf = 0.0;
    }
    const double uFf = m_model.requiredInputClamped(rFf, rDotFf,
                                                  ambient, ffDisturb);

    // ---- 4. Feedback on the remaining actuator authority ----
    // The backend is a polymorphic ControllerBase: one virtual call per
    // cycle (negligible at oven loop rates). piIn.feedforward carries the
    // feedforward term so adaptive backends (e.g. OvenLQIBackend) can set
    // their own remaining-authority limits identically to the PI path;
    // PIDController ignores the field outside Tracking mode.
    ControllerInput piIn;
    piIn.reference = rHat;
    piIn.measured = m_Tf;
    piIn.dt = dt;
    piIn.referenceDerivative = rDot;
    piIn.feedforward = uFf;

    SaturationLimits piLim;
    piLim.outputMin = uMin - uFf;
    piLim.outputMax = uMax - uFf;
    piLim.integralMin = -m_cfg.integralLimit;
    piLim.integralMax = m_cfg.integralLimit;
    m_feedback->setSaturationLimits(piLim);

    const ControllerOutput piOut = m_feedback->compute(piIn);
    const double uFb = piOut.control;
    m_lastUfb = uFb;

    // ---- 5. Sum, safety interlock, clamp ----
    const double uRaw = uFf + uFb;
    double u = uRaw;
    if (m_overTemp) {
        if (m_Tf < m_cfg.maxTempC - m_cfg.overTempHysteresis) m_overTemp = false;
        else u = uMin;
    } else if (m_Tf > m_cfg.maxTempC) {
        m_overTemp = true;
        u = uMin;
    }

    // Persistency-of-excitation dither: without it, closed-loop data makes
    // u a near-linear function of (T-Tamb) and {alpha,beta,d} are only
    // jointly identifiable up to a 1-D ambiguity. The ripple on the
    // measured temperature is ~alpha*A/omega (~0.1-0.3 C for typical oven
    // parameters) — small against control bands, and it exists purely to
    // decorrelate the estimator's regressors.
    if (m_cfg.adaptParams && m_cfg.ditherAmp > 0.0 && !m_overTemp) {
        u += m_cfg.ditherAmp *
             std::sin(2.0 * std::numbers::pi * m_time / m_cfg.ditherPeriod);
    }

    // Rate limit inside computeImpl: the value recorded in m_uHist and
    // returned to the caller is exactly what the actuator sees.
    if (m_cfg.uRateLimit > 0.0) {
        const double maxD = m_cfg.uRateLimit * dt;
        u = m_lastU + std::clamp(u - m_lastU, -maxD, maxD);
    }
    u = std::clamp(u, uMin, uMax);

    // Report the ACTUALLY applied output (post interlock/rate-limit/
    // saturation) so observer-based backends predict with the true input.
    if (m_feedbackAdaptive) m_feedbackAdaptive->notifyAppliedInput(u);

    // ---- 6. Online parameter estimation (delayed input) ----
    // The estimator runs on the RAW measurement: its integral-form window
    // regression is already noise-robust, and feeding it the filtered
    // signal would inject the filter's lag as unmodeled plant dynamics
    // and bias {alpha,beta,d} low. The filtered m_Tf is for feedback only.
    if (m_cfg.adaptParams) {
        const double uDelayed = delayedInput(m_time - mp.delay);
        m_est.update(uDelayed, input.measured, ambient, dt);
        updateModelFromEstimator();
        maybeRetune();
    }

    // ---- 7. Bookkeeping ----
    pushInput(m_time, u);
    m_lastU = u;
    m_time += dt;

    out.control = u;
    out.feedforward = uFf;
    out.proportional = piOut.proportional;
    out.integral = piOut.integral;
    out.derivative = piOut.derivative;
    out.error = rHat - m_Tf;
    // "Saturated" means actuator authority was exhausted — not merely that
    // dither/rate-limit perturbed the total.
    out.saturated = piOut.saturated || m_overTemp ||
                    (uRaw > uMax + 1e-12) || (uRaw < uMin - 1e-12);
    out.integrating = true;
    return out;
}

void OvenController::updateModelFromEstimator() {
    // Sync adapted physical parameters into the FF model. Bounds are
    // already enforced inside the estimator.
    OvenModelParams& p = m_model.params();
    p.alpha = m_est.alpha();
    p.beta = m_est.beta();
    p.disturbance = m_est.disturbance();
    // delay stays fixed (not identifiable online), ambient/u-bounds fixed.
}

void OvenController::maybeRetune() {
    if (!m_cfg.adaptParams) return;
    if (m_est.updatesUsed() < 3) return;
    if (m_time - m_lastRetuneTime < m_cfg.retuneInterval) return;
    if (m_est.updatesUsed() - m_lastRetuneWindows < m_cfg.retuneMinWindows)
        return;

    if (m_feedbackAdaptive) {
        // Adaptive state-space backend (e.g. LQI+observer): push the
        // bounded physical parameters; the backend re-derives its observer
        // gain and LQI gains internally. Gate on the same meaningful-
        // change rule, evaluated on {alpha, beta}.
        const double a = m_est.alpha(), b = m_est.beta();
        const double da = std::abs(a - m_lastAlpha) /
                          std::max(std::abs(m_lastAlpha), 1e-9);
        const double db = std::abs(b - m_lastBeta) /
                          std::max(std::abs(m_lastBeta), 1e-9);
        if (da < m_cfg.retuneMinDelta && db < m_cfg.retuneMinDelta) {
            m_lastRetuneWindows = m_est.updatesUsed();
            m_lastRetuneTime = m_time;
            return;
        }
        m_feedbackAdaptive->updateThermalParameters(a, b,
                                                     m_est.disturbance());
        m_lastAlpha = a;
        m_lastBeta = b;
        m_lastRetuneWindows = m_est.updatesUsed();
        m_lastRetuneTime = m_time;
        return;
    }

    if (!m_pidBackend) return;   // custom non-adaptive backend: no retune

    const double K = m_est.alpha() / std::max(m_est.beta(), 1e-9);
    const double tau = 1.0 / std::max(m_est.beta(), 1e-9);
    const auto [kp, ki] = simcPiGains(K, tau, m_model.params().delay,
                                      m_cfg.simcTauC);
    if (kp <= 0.0 || ki <= 0.0) return;

    // Only retune on a meaningful parameter change.
    const double dk = std::abs(kp - m_lastKp) / std::max(std::abs(m_lastKp), 1e-9);
    const double di = std::abs(ki - m_lastKi) / std::max(std::abs(m_lastKi), 1e-9);
    if (dk < m_cfg.retuneMinDelta && di < m_cfg.retuneMinDelta) {
        m_lastRetuneWindows = m_est.updatesUsed();
        m_lastRetuneTime = m_time;
        return;
    }

    applyPiGains(kp, ki);
    m_lastKp = kp;
    m_lastKi = ki;
    m_lastRetuneWindows = m_est.updatesUsed();
    m_lastRetuneTime = m_time;
}

void OvenController::applyPiGains(double kp, double ki) {
    if (!m_pidBackend) return;   // only meaningful for the SIMC-PI backend
    m_pidBackend->setGains(kp, ki, 0.0);
    // Back-calculation tracking time: explicit config wins, else Ti.
    double Tt = m_cfg.antiWindupTt;
    if (Tt <= 0.0) {
        const double Ti = (ki > 1e-12) ? kp / ki : 1.0;
        Tt = std::max(1.0, Ti);
    }
    m_pidBackend->setAntiWindup(AntiWindupMethod::BackCalculation, Tt);
    m_pidBackend->setIntegralLimits(-m_cfg.integralLimit, m_cfg.integralLimit);
    m_pidBackend->setDerivativeOnMeasurement(true);
}

void OvenController::resetImpl() {
    m_feedback->reset();
    m_est.reset();          // keeps learned theta, clears statistics
    m_time = 0.0;
    m_TfInit = false;
    m_lastU = 0.0;
    m_lastUfb = 0.0;
    m_overTemp = false;
    m_uHist.clear();
    m_lastRetuneTime = -1e9;
    m_lastRetuneWindows = 0;
    m_extDisturb = 0.0;
    m_extDisturbAvg = 0.0;
    m_extDisturbInit = false;
    if (m_hasProfile) m_gov.reset(m_profile.value(0.0));
}

double OvenController::delayedInput(double t) const {
    if (m_uHist.empty()) return 0.0;
    double u = m_uHist.front().second;
    for (const auto& [ti, ui] : m_uHist) {
        if (ti <= t) u = ui;
        else break;
    }
    return u;
}

void OvenController::pushInput(double t, double u) {
    m_uHist.emplace_back(t, u);
    while (m_uHist.size() > kMaxInputHistory) m_uHist.pop_front();
}

} // namespace tether::control::thermal
