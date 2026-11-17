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
      m_modelSopdt(cfg.modelSopdt),
      m_est(cfg.estimator),
      m_estSopdt(cfg.sopdtEstimator),
      m_gov(cfg.governor) {
    installBackend(cfg.feedbackBackend);
    m_estSopdt.setPrior(cfg.modelSopdt.alpha, cfg.modelSopdt.beta,
                        cfg.modelSopdt.tau2, cfg.modelSopdt.disturbance);

    // Initial gains: explicit config wins — for tuneMethod=Manual the
    // {kp,ki,kd} fields are used verbatim; for auto methods a fully
    // specified {kp,ki} pair is honored as an override. Otherwise gains
    // are designed from the physical model per tuneMethod.
    double kp = cfg.kp, ki = cfg.ki, kd = cfg.kd;
    const bool explicitGains =
        (cfg.tuneMethod == OvenTuneMethod::Manual) ||
        (kp > 0.0 && ki > 0.0);
    if (!explicitGains) designGains(kp, ki, kd);
    applyPiGains(kp, ki, kd);
    m_lastKp = kp;
    m_lastKi = ki;
    m_lastKd = kd;
    if (isSopdt()) {
        const auto& sp = m_modelSopdt.params();
        if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
            m_lastAlpha = sp.alpha;
            m_lastBeta = sp.beta;
        } else {
            // FOPDT-integral estimator works in effective space.
            const auto [ae, be] = sp.effectiveAlphaBeta();
            m_lastAlpha = ae;
            m_lastBeta = be;
        }
        m_lastTau2 = sp.tau2;
    } else {
        m_lastAlpha = cfg.model.alpha;
        m_lastBeta = cfg.model.beta;
        m_lastTau2 = 0.0;
    }

    // Outer loop: hard actuator bounds (safety net; computeImpl already
    // clamps). Rate limiting is applied inside computeImpl so the recorded
    // input history matches the applied output exactly.
    SaturationLimits lim;
    lim.outputMin = isSopdt() ? cfg.modelSopdt.uMin : cfg.model.uMin;
    lim.outputMax = isSopdt() ? cfg.modelSopdt.uMax : cfg.model.uMax;
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
    m_estSopdt.setConfig(cfg.sopdtEstimator);
    if (kindChanged) installBackend(cfg.feedbackBackend);
    // Actuator bounds follow the ACTIVE model order's params.
    const double uMin = isSopdt() ? m_modelSopdt.params().uMin
                                  : m_model.params().uMin;
    const double uMax = isSopdt() ? m_modelSopdt.params().uMax
                                  : m_model.params().uMax;
    SaturationLimits lim;
    lim.outputMin = uMin;
    lim.outputMax = uMax;
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
    m_modelSopdt.params().ambient = ambientC;
    auto gc = m_gov.config();
    gc.ambient = ambientC;
    m_gov.setConfig(gc);
}

void OvenController::setDisturbancePrior(double d) {
    m_model.params().disturbance = d;
    m_modelSopdt.params().disturbance = d;
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        m_estSopdt.setPrior(m_estSopdt.alpha(), m_estSopdt.beta(),
                            m_estSopdt.tau2(), d);
    } else {
        m_est.setPrior(m_est.alpha(), m_est.beta(), d);
        // FOPDT estimator also feeds the SOPDT stage-1 when projected.
    }
}

void OvenController::setCouplingDisturbance(double d) {
    m_extDisturb = d;
}

void OvenController::setModelParams(const OvenModelParams& params) {
    if (isSopdt()) {
        // Project onto the cascade: stage-1 {α,β,d,delay,amb,bounds}
        // from the FOPDT set, τ2 held at its current value.
        setModelParamsSOPDT(OvenModelParamsSOPDT::fromFopdtParams(
            params, m_modelSopdt.params().tau2));
        return;
    }
    m_model.setParams(params);
    m_model.params().disturbance = params.disturbance;
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        m_estSopdt.setPrior(params.alpha, params.beta,
                            m_estSopdt.tau2(), params.disturbance);
    } else {
        m_est.setPrior(params.alpha, params.beta, params.disturbance);
    }
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
    syncRetuneBaseline();
}

void OvenController::setModelParamsSOPDT(const OvenModelParamsSOPDT& params) {
    if (!isSopdt()) {
        // Project onto the FOPDT model (dominant-lag approximation).
        setModelParams(params.toFopdtParams());
        return;
    }
    m_modelSopdt.setParams(params);
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        m_estSopdt.setPrior(params.alpha, params.beta, params.tau2,
                            params.disturbance);
    } else {
        // The FOPDT-integral estimator works in EFFECTIVE space —
        // prime it with the cascade's quasi-static {αe,βe}, not the
        // raw stage-1 values.
        const auto [ea0, eb0] = params.effectiveAlphaBeta();
        m_est.setPrior(ea0, eb0, params.disturbance);
    }
    // Governor sees the quasi-static effective parameters.
    const auto [ae, be] = params.effectiveAlphaBeta();
    auto gc = m_cfg.governor;
    gc.alpha = ae;
    gc.beta = be;
    gc.disturbance = params.disturbance;
    gc.ambient = params.ambient;
    gc.uMin = params.uMin;
    gc.uMax = params.uMax;
    m_gov.setConfig(gc);
    SaturationLimits lim = getSaturationLimits();
    lim.outputMin = params.uMin;
    lim.outputMax = params.uMax;
    setSaturationLimits(lim);
    syncRetuneBaseline();
}

void OvenController::syncRetuneBaseline() {
    // m_last* must live in ESTIMATOR-OUTPUT space: the retune gate
    // compares estimator outputs against these values.
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        m_lastAlpha = m_estSopdt.alpha();
        m_lastBeta = m_estSopdt.beta();
        m_lastTau2 = m_estSopdt.tau2();
    } else {
        m_lastAlpha = m_est.alpha();
        m_lastBeta = m_est.beta();
        m_lastTau2 = isSopdt() ? m_modelSopdt.params().tau2 : 0.0;
    }
}

void OvenController::applyAutotune(const OvenAutotuneResult& result) {
    if (!result.success) return;
    setModelParams(result.params);
    applyPiGains(result.kp, result.ki, 0.0);
    m_lastKp = result.kp;
    m_lastKi = result.ki;
    m_lastKd = 0.0;
    syncRetuneBaseline();
    if (m_feedbackAdaptive) {
        // Effective {α,β} of the ACTIVE model (quasi-static for SOPDT).
        double ea, eb;
        if (isSopdt()) {
            const auto& p = m_modelSopdt.params();
            const auto [x, y] = p.effectiveAlphaBeta();
            ea = x; eb = y;
        } else {
            ea = result.params.alpha; eb = result.params.beta;
        }
        m_feedbackAdaptive->updateThermalParameters(ea, eb,
                                                    result.params.disturbance);
    }
    m_lastRetuneWindows = estUpdatesUsed();
    m_lastRetuneTime = m_time;
}

void OvenController::applyAutotuneSOPDT(const OvenAutotuneResultSOPDT& result) {
    if (!result.success) return;
    setModelParamsSOPDT(result.params);
    applyPiGains(result.kp, result.ki, result.kd);
    m_lastKp = result.kp;
    m_lastKi = result.ki;
    m_lastKd = result.kd;
    if (m_feedbackAdaptive) {
        // Feed the adaptive backend the ACTIVE model's effective
        // {α,β} — for SOPDT the quasi-static dominant parameters.
        double ea, eb, d;
        if (isSopdt()) {
            const auto& p = m_modelSopdt.params();
            const auto [x, y] = p.effectiveAlphaBeta();
            ea = x; eb = y; d = p.disturbance;
        } else {
            const auto& p = m_model.params();
            ea = p.alpha; eb = p.beta; d = p.disturbance;
        }
        m_feedbackAdaptive->updateThermalParameters(ea, eb, d);
    }
    m_lastRetuneWindows = estUpdatesUsed();
    m_lastRetuneTime = m_time;
}

OvenControllerStatus OvenController::status() const {
    OvenControllerStatus s;
    s.time = m_time;
    s.reference = m_hasProfile ? m_gov.value() : m_lastRef;
    s.measuredFiltered = m_Tf;
    s.modelOrder = m_cfg.modelOrder;
    s.ffMethod = m_cfg.ffMethod;
    s.estimateMethod = m_cfg.estimateMethod;
    s.tuneMethod = m_cfg.tuneMethod;
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        s.alpha = m_estSopdt.alpha();
        s.beta = m_estSopdt.beta();
        s.disturbance = m_estSopdt.disturbance();
        s.tau2 = m_estSopdt.tau2();
        s.windowsUsed = m_estSopdt.updatesUsed();
    } else {
        s.alpha = m_est.alpha();
        s.beta = m_est.beta();
        s.disturbance = m_est.disturbance();
        s.tau2 = isSopdt() ? m_modelSopdt.params().tau2 : 0.0;
        s.windowsUsed = m_est.updatesUsed();
    }
    s.overTemp = m_overTemp;
    s.adapting = m_cfg.adaptParams &&
                 m_cfg.estimateMethod != OvenEstimateMethod::Off;
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

    // ---- Unified plant view across model orders ----
    // effAlpha/effBeta are the quasi-static parameters the governor
    // uses for feasibility rates (identical to {α,β} for FOPDT).
    double delay, disturbance, ambient, uMin, uMax;
    double effAlpha, effBeta;
    if (isSopdt()) {
        const auto& p = m_modelSopdt.params();
        delay = p.delay;
        disturbance = p.disturbance; ambient = p.ambient;
        uMin = p.uMin; uMax = p.uMax;
        const auto [ae, be] = p.effectiveAlphaBeta();
        effAlpha = ae; effBeta = be;
    } else {
        const auto& p = m_model.params();
        delay = p.delay;
        disturbance = p.disturbance; ambient = p.ambient;
        uMin = p.uMin; uMax = p.uMax;
        effAlpha = p.alpha; effBeta = p.beta;
    }

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
        disturbance + (m_extDisturb - m_extDisturbAvg);

    double rHat, rDot;
    if (m_hasProfile) {
        // Keep governor physics in sync with (possibly adapted) model.
        // For SOPDT the governor sees the quasi-static effective
        // parameters — feasibility rates of the dominant dynamics.
        GovernorConfig gc = m_cfg.governor;
        gc.alpha = effAlpha;
        gc.beta = effBeta;
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
        rFf = m_gov.previewValue(delay);
        rDotFf = rDot;
    } else {
        rFf = input.reference;
        rDotFf = 0.0;
    }

    double uFf = 0.0;
    if (m_cfg.ffMethod != OvenFeedforwardMethod::None) {
        if (isSopdt() &&
            m_cfg.ffMethod == OvenFeedforwardMethod::ModelInverse) {
            // Exact two-stage inverse needs r̈. The profile is piecewise
            // linear, so r̈ is impulses at knots — estimated by central
            // difference of the analytic rate over a finite window
            // (integral-preserving: area = Δrate).
            double rAccFf = 0.0;
            if (m_hasProfile) {
                const double tPrev = m_time + delay;
                const double h = std::max(0.5, m_cfg.sopdtFFAccelWindow);
                rAccFf = (m_profile.derivative(tPrev + h) -
                          m_profile.derivative(tPrev - h)) / (2.0 * h);
            }
            uFf = m_modelSopdt.requiredInputClamped(rFf, rDotFf, rAccFf,
                                                  ambient, ffDisturb);
        } else if (isSopdt()) {
            // DominantLag: quasi-static FOPDT-equivalent inverse.
            uFf = std::clamp(
                m_modelSopdt.requiredInputDominantLag(rFf, rDotFf,
                                                      ambient, ffDisturb),
                uMin, uMax);
        } else {
            // FOPDT: ModelInverse and DominantLag coincide.
            uFf = m_model.requiredInputClamped(rFf, rDotFf,
                                              ambient, ffDisturb);
        }
    }

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
    // The FOPDT integral estimator runs on the RAW measurement: its
    // integral-form window regression is already noise-robust, and
    // feeding it the filtered signal would inject the filter's lag as
    // unmodeled plant dynamics and bias {alpha,beta,d} low. The OE
    // SOPDT estimator is the opposite — it re-simulates the cascade
    // against the measured trajectory, so it gets the FILTERED signal.
    // m_Tf serves feedback in both cases.
    if (m_cfg.adaptParams &&
        m_cfg.estimateMethod != OvenEstimateMethod::Off) {
        const double uDelayed = delayedInput(m_time - delay);
        if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
            m_estSopdt.update(uDelayed, m_Tf, ambient, dt);
        } else {
            m_est.update(uDelayed, input.measured, ambient, dt);
        }
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
    // already enforced inside the estimators.
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        if (isSopdt()) {
            // Estimator outputs ARE cascade stage params.
            auto& p = m_modelSopdt.params();
            p.alpha = m_estSopdt.alpha();
            p.beta = m_estSopdt.beta();
            p.tau2 = m_estSopdt.tau2();
            p.disturbance = m_estSopdt.disturbance();
        } else {
            // FOPDT model: use the DOMINANT pole — for a true FOPDT
            // plant the ARX second pole collapses and the fast/slow
            // assignment would otherwise invert.
            const double tauDom = std::max(m_estSopdt.tau1(),
                                           m_estSopdt.tau2());
            const double K = m_estSopdt.gain();
            auto& p = m_model.params();
            p.beta = 1.0 / std::max(tauDom, 1e-3);
            p.alpha = K * p.beta;
            p.disturbance = m_estSopdt.disturbance();
        }
        return;
    }
    // FopdtIntegral: estimates the EFFECTIVE {α,β,d}. With
    // modelOrder=Sopdt project onto stage 1 preserving the dominant
    // lag: τ_stage1 = τ_eff − τ2 (τ2 fixed — a single-lag fit cannot
    // identify it). See OvenModelParamsSOPDT::fromFopdtParams.
    if (isSopdt()) {
        auto& p = m_modelSopdt.params();
        const double tauEff = 1.0 / std::max(m_est.beta(), 1e-9);
        const double K = m_est.alpha() / std::max(m_est.beta(), 1e-9);
        const double tau1 = std::max(tauEff - p.tau2, 0.2 * tauEff);
        p.beta = 1.0 / tau1;
        p.alpha = K * p.beta;
        p.disturbance = m_est.disturbance();
    } else {
        auto& p = m_model.params();
        p.alpha = m_est.alpha();
        p.beta = m_est.beta();
        p.disturbance = m_est.disturbance();
    }
    // delay stays fixed (not identifiable online), ambient/u-bounds fixed.
}

int OvenController::estUpdatesUsed() const {
    return (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe)
        ? m_estSopdt.updatesUsed() : m_est.updatesUsed();
}

void OvenController::designGains(double& kp, double& ki, double& kd) const {
    kp = ki = kd = 0.0;
    if (m_cfg.tuneMethod == OvenTuneMethod::Manual) return;

    if (isSopdt()) {
        const auto& p = m_modelSopdt.params();
        const double K = p.gain();
        const double tauStage1 = p.tau1();          // 1/β
        const double tauDom = std::max(tauStage1, p.tau2);
        const double tauSec = std::min(tauStage1, p.tau2);
        switch (m_cfg.tuneMethod) {
        case OvenTuneMethod::Simc: {
            const auto g = simcGainsSopdt(K, tauDom, tauSec, p.delay,
                                          m_cfg.simcTauC);
            kp = g.kp; ki = g.ki; kd = g.kd;
            break;
        }
        case OvenTuneMethod::ImcPi: {
            const auto g = imcPiGainsSopdt(K, tauDom, tauSec, p.delay,
                                           m_cfg.simcTauC);
            kp = g.kp; ki = g.ki; kd = g.kd;
            break;
        }
        case OvenTuneMethod::ImcPid: {
            const auto g = imcPidGainsSopdt(K, tauDom, tauSec, p.delay,
                                            m_cfg.simcTauC);
            kp = g.kp; ki = g.ki; kd = g.kd;
            break;
        }
        case OvenTuneMethod::Manual: break;
        }
        return;
    }

    // FOPDT
    const auto& p = m_model.params();
    const double K = p.gain();
    const double tau = p.tau();
    const double theta = p.delay;
    switch (m_cfg.tuneMethod) {
    case OvenTuneMethod::Simc: {
        const auto [a, b] = simcPiGains(K, tau, theta, m_cfg.simcTauC);
        kp = a; ki = b; kd = 0.0;
        break;
    }
    case OvenTuneMethod::ImcPi:
    case OvenTuneMethod::ImcPid: {
        // IMC on FOPDT. λ default mirrors the SIMC convention (λ = θ,
        // or 0.1τ when θ≈0). PI: Kp = τ/(K(λ+θ)), Ti = τ.
        // PID adds the standard IMC derivative Td = τ·θ/(2τ+θ) with
        // Kp/Ti evaluated at λ+θ/2 (IMCDesign filterOrder=2 form).
        double lam = m_cfg.simcTauC;
        if (lam <= 0.0) lam = (theta > 1e-6) ? theta : 0.1 * tau;
        if (m_cfg.tuneMethod == OvenTuneMethod::ImcPi) {
            kp = tau / (std::abs(K) * (lam + theta));
            ki = (tau > 1e-9) ? kp / tau : 0.0;
        } else {
            const double lamEff = lam + theta / 2.0;
            kp = (tau + theta / 2.0) / (std::abs(K) * lamEff);
            const double Ti = tau + theta / 2.0;
            ki = (Ti > 1e-9) ? kp / Ti : 0.0;
            const double Td = (2.0 * tau + theta > 1e-9)
                ? tau * theta / (2.0 * tau + theta) : 0.0;
            kd = kp * Td;
        }
        break;
    }
    case OvenTuneMethod::Manual: break;
    }
}

void OvenController::maybeRetune() {
    if (!m_cfg.adaptParams) return;
    if (m_cfg.estimateMethod == OvenEstimateMethod::Off) return;
    if (m_cfg.tuneMethod == OvenTuneMethod::Manual) return;  // user owns gains
    if (estUpdatesUsed() < 3) return;
    if (m_time - m_lastRetuneTime < m_cfg.retuneInterval) return;
    if (estUpdatesUsed() - m_lastRetuneWindows < m_cfg.retuneMinWindows)
        return;

    // Current parameter estimates (per selected estimator).
    double a, b, t2, d;
    if (m_cfg.estimateMethod == OvenEstimateMethod::SopdtOe) {
        a = m_estSopdt.alpha(); b = m_estSopdt.beta();
        t2 = m_estSopdt.tau2(); d = m_estSopdt.disturbance();
    } else {
        a = m_est.alpha(); b = m_est.beta(); d = m_est.disturbance();
        t2 = isSopdt() ? m_modelSopdt.params().tau2 : 0.0;
    }

    // Gate on meaningful {α,β,τ2} change.
    const double da = std::abs(a - m_lastAlpha) /
                      std::max(std::abs(m_lastAlpha), 1e-9);
    const double db = std::abs(b - m_lastBeta) /
                      std::max(std::abs(m_lastBeta), 1e-9);
    const double dt2 = std::abs(t2 - m_lastTau2) /
                       std::max(std::abs(m_lastTau2), 1e-9);
    if (da < m_cfg.retuneMinDelta && db < m_cfg.retuneMinDelta &&
        dt2 < m_cfg.retuneMinDelta) {
        m_lastRetuneWindows = estUpdatesUsed();
        m_lastRetuneTime = m_time;
        return;
    }

    if (m_feedbackAdaptive) {
        // Adaptive state-space backend (e.g. LQI+observer): push the
        // bounded physical parameters; the backend re-derives its observer
        // gain and LQI gains internally. For SOPDT feed the quasi-static
        // effective parameters — the best match to its dominant dynamics.
        double ea = a, eb = b;
        if (isSopdt()) {
            const auto& p = m_modelSopdt.params();
            const auto [x, y] = p.effectiveAlphaBeta();
            ea = x; eb = y;
        }
        m_feedbackAdaptive->updateThermalParameters(ea, eb, d);
        m_lastAlpha = a;
        m_lastBeta = b;
        m_lastTau2 = t2;
        m_lastRetuneWindows = estUpdatesUsed();
        m_lastRetuneTime = m_time;
        return;
    }

    if (!m_pidBackend) return;   // custom non-adaptive backend: no retune

    double kp, ki, kd;
    designGains(kp, ki, kd);
    if ((kp <= 0.0 && ki <= 0.0) || !std::isfinite(kp) || !std::isfinite(ki))
        return;

    // Only retune on a meaningful gain change.
    const double dk = std::abs(kp - m_lastKp) / std::max(std::abs(m_lastKp), 1e-9);
    const double di = std::abs(ki - m_lastKi) / std::max(std::abs(m_lastKi), 1e-9);
    const double dd = std::abs(kd - m_lastKd) / std::max(std::abs(m_lastKd), 1e-9);
    if (dk < m_cfg.retuneMinDelta && di < m_cfg.retuneMinDelta &&
        dd < m_cfg.retuneMinDelta) {
        m_lastRetuneWindows = estUpdatesUsed();
        m_lastRetuneTime = m_time;
        return;
    }

    applyPiGains(kp, ki, kd);
    m_lastKp = kp;
    m_lastKi = ki;
    m_lastKd = kd;
    m_lastAlpha = a;
    m_lastBeta = b;
    m_lastTau2 = t2;
    m_lastRetuneWindows = estUpdatesUsed();
    m_lastRetuneTime = m_time;
}

void OvenController::applyPiGains(double kp, double ki, double kd) {
    if (!m_pidBackend) return;   // only meaningful for the SIMC-PI backend
    m_pidBackend->setGains(kp, ki, kd);
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
    m_estSopdt.reset();     // same for the ARX estimator
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
