/**
 * @file OvenController.hpp
 * @brief Generic profile-following oven controller with physical parameters.
 *
 * @details
 * Architecture (all tuning derived from physical parameters — no
 * hand-tuned PID gains anywhere in the interface):
 *
 * ```
 *            TemperatureProfile (ramp/soak or point list)
 *                     │
 *              ActuationGovernor  ← rate-limits reference into the
 *                     │             achievable band [coolRate, heatRate]
 *                     │             of the thermal model (one-sided heat)
 *        ┌────────────┴─────────────┐
 *        │                          │
 *  model-inverse FF          feedback backend
 *  u_ff = requiredInput(     (pluggable ControllerBase:
 *    r̂(t+θ), ṙ̂, T_amb, d̂)     SIMC-PI or LQI+observer)
 *        │                          │
 *        └───────────┬──────────────┘
 *                    ▼
 *              u = clamp(u_ff + u_fb + dither, uMin, uMax)
 *                    │
 *      ThermalParameterEstimator ──→ bounded {α, β, d} adaptation
 *      (windowed LS, excitation-gated)   ↺ updates model + retunes gains
 * ```
 *
 * Key properties:
 *  - Feedback gains are *outputs* of the physical model — SIMC rules for
 *    the PI backend, Riccati re-solves for the LQI backend — re-derived
 *    whenever the online estimator updates {α, β}.
 *  - Delay is handled by previewing the reference `θ` ahead in the
 *    feedforward (power arrives when needed) — conservative IMC-style,
 *    no Smith predictor needed at oven timescales.
 *  - The ActuationGovernor enforces feasibility: a descent faster than
 *    passive cooling allows is pre-shaped to the achievable rate; the
 *    feedforward simply saturates at uMin on such segments.
 *  - Noise: first-order sensor filter + windowed (integral-form)
 *    estimation — no differentiation of the temperature signal.
 *  - Anti-windup: the PI backend sees effective limits
 *    [uMin - u_ff, uMax - u_ff] so its back-calculation tracks the true
 *    remaining actuator authority; the outer loop saturates the sum.
 *  - Real-time safe: fixed-size buffers, no allocation in compute().
 */

#pragma once

#include "tether/control/ControllerBase.hpp"
#include "tether/control/PIDControllers.hpp"
#include "tether/control/thermal/ActuationGovernor.hpp"
#include "tether/control/thermal/OvenAutotune.hpp"
#include "tether/control/thermal/OvenCoupledModel.hpp"
#include "tether/control/thermal/OvenLQIBackend.hpp"
#include "tether/control/thermal/OvenSOPDTModel.hpp"
#include "tether/control/thermal/OvenThermalModel.hpp"
#include "tether/control/thermal/SOPDTParameterEstimator.hpp"
#include "tether/control/thermal/TemperatureProfile.hpp"
#include "tether/control/thermal/ThermalParameterEstimator.hpp"

#include <deque>
#include <memory>

namespace tether::control::thermal {

/**@brief Selectable feedback backend for OvenController. */
enum class OvenFeedbackBackendKind {
    PI,   /**< Default: SIMC-tuned PID on the filtered measurement */
    LQI   /**< LQI state feedback + Kalman observer for non-collocated
               sensing (estimates product-zone temperature) */
};

/**@brief Plant model order — user-selected, never inferred. */
enum class OvenModelOrder {
    Fopdt,  /**< Single lossy thermal mass (default, backward compatible) */
    Sopdt   /**< Cascade: lossy stage-1 mass + pure-lag stage-2 mass */
};

/**@brief Feedforward method — user-selected. */
enum class OvenFeedforwardMethod {
    ModelInverse,  /**< Exact model-inverse of the active model order.
                        FOPDT: analytic inverse. SOPDT: two-stage inverse
                        using reference acceleration (exact on ramp/soak
                        profiles; finite-difference r̈ at knots). */
    DominantLag,   /**< Quasi-static FOPDT-equivalent inverse — ignores
                        the τ2 compensation term. Simpler, slightly more
                        lag; identical to ModelInverse for FOPDT. */
    None           /**< No feedforward; feedback only. */
};

/**@brief Online parameter estimator — user-selected. */
enum class OvenEstimateMethod {
    FopdtIntegral, /**< Windowed integral regression → {α,β,d}.
                        With modelOrder=Sopdt this is a projection: it
                        updates stage-1 {α,β,d} and holds τ2 fixed. */
    SopdtOe,      /**< Trust-region output-error fit → {α,β,τ2,d}.
                        Simulates the exact cascade over the window and
                        solves {α,d} linearly for each lag candidate;
                        needs richer excitation than FopdtIntegral. */
    Off            /**< No online estimation (same as adaptParams=false). */
};

/**@brief Configuration for OvenController. */
struct OvenControllerConfig {
    // --- Plant model (nominal/initial physical parameters) ---
    OvenModelParams model;

    // --- Model order & method selection (user has full control) ---
    OvenModelOrder modelOrder{OvenModelOrder::Fopdt};
    OvenModelParamsSOPDT modelSopdt;   /**< Used when modelOrder=Sopdt.
        Build from a FOPDT set via
        `OvenModelParamsSOPDT::fromFopdtParams(model, tau2)` — only τ2
        is genuinely new; stage-1 params mirror `model`. */
    OvenFeedforwardMethod ffMethod{OvenFeedforwardMethod::ModelInverse};
    OvenEstimateMethod estimateMethod{OvenEstimateMethod::FopdtIntegral};
    OvenTuneMethod tuneMethod{OvenTuneMethod::Simc};
    SOPDTEstimatorConfig sopdtEstimator; /**< OE estimator tuning */

    // --- Subsystem configs ---
    GovernorConfig governor;
    EstimatorConfig estimator;

    // --- Feedback backend selection ---
    OvenFeedbackBackendKind feedbackBackend{OvenFeedbackBackendKind::PI};

    // --- LQI backend (non-collocated sensing) ---
    OvenCoupledParams coupled;      /**< Nominal 2-state plant params */
    // LQI weights are performance weights, not physical parameters — the
    // physical params flow through cfg.coupled. Defaults give a smooth,
    // non-oscillatory loop for typical oven time constants (verified in
    // tests); increase lqiR for even softer action, decrease for faster.
    double lqiQxSensor{0.01};       /**< LQI weight, sensor-zone state */
    double lqiQxProduct{0.004};     /**< LQI weight, product-zone state */
    double lqiQi{0.02};             /**< LQI integral weight */
    double lqiR{1000.0};            /**< LQI control weight */
    double observerQProc{1e-4};     /**< Observer process-noise density */
    double observerRMeas{0.09};     /**< Observer measurement variance [°C²] */
    double backendDt{0.5};          /**< Observer/LQI design step [s] */

    // --- Feedback PI ---
    double kp{0.0};           /**< PI proportional gain; 0 = auto (tuneMethod)
                                   For tuneMethod=Manual: used verbatim. */
    double ki{0.0};           /**< PI integral gain;     0 = auto (tuneMethod)
                                   For tuneMethod=Manual: used verbatim. */
    double kd{0.0};           /**< Derivative gain (PID backends / Manual /
                                   SIMC-SOPDT / IMC-PID). 0 = auto per
                                   tuneMethod; Manual uses it verbatim. */
    double simcTauC{0.0};     /**< SIMC/IMC closed-loop λ [s]; <=0 → λ = θ */
    double sopdtFFAccelWindow{4.0}; /**< Half-width [s] of the finite-
        difference window used to estimate r̈ for the exact SOPDT
        feedforward at profile knots. Larger = softer FF kicks. */
    double antiWindupTt{0.0}; /**< Back-calc tracking τ [s]; <=0 → auto */
    double integralLimit{10.0}; /**< |I-term| bound [u] */

    // --- Sensor filtering ---
    double sensorTau{3.0};    /**< First-order filter τ [s]; <=0 disables */

    // --- Online adaptation ---
    bool adaptParams{true};        /**< Enable bounded parameter estimation */
    double retuneInterval{30.0};   /**< Min seconds between gain retunes [s] */
    int retuneMinWindows{60};      /**< Excited estimator updates required
                                        between gain retunes */
    double retuneMinDelta{0.02};   /**< Min relative param change to retune */
    double ditherAmp{0.02};        /**< PE dither amplitude [u]; 0 disables.
                                        Only applied while adaptParams is on.
                                        Causes ~0.1-0.3 °C measurement ripple
                                        for typical oven parameters. */
    double ditherPeriod{37.0};     /**< Dither period [s] (non-commensurate
                                        with typical profile segments) */

    // --- Safety ---
    double maxTempC{400.0};        /**< Over-temperature shutdown [°C] */
    double overTempHysteresis{5.0};/**< Resume below maxTemp - hysteresis [°C] */
    double uRateLimit{0.2};        /**< Max |Δu| per second ([]/s]; <=0 off */
};

/**@brief Diagnostics snapshot for logging/monitoring. */
struct OvenControllerStatus {
    double time{0.0};
    double reference{0.0};      /**< Governed reference [°C] */
    double measuredFiltered{0.0};
    double feedforward{0.0};
    double feedback{0.0};
    double output{0.0};
    double alpha{0.0}, beta{0.0}, disturbance{0.0};
    double tau2{0.0};           /**< SOPDT stage-2 lag [s]; 0 in FOPDT mode */
    int windowsUsed{0};
    bool saturated{false};
    bool overTemp{false};
    bool adapting{false};
    const char* backendName{nullptr}; /**< Active feedback backend id */
    // Active method selections (inspectable — mirrors config)
    OvenModelOrder modelOrder{OvenModelOrder::Fopdt};
    OvenFeedforwardMethod ffMethod{OvenFeedforwardMethod::ModelInverse};
    OvenEstimateMethod estimateMethod{OvenEstimateMethod::FopdtIntegral};
    OvenTuneMethod tuneMethod{OvenTuneMethod::Simc};
};

class OvenController : public ControllerBase {
public:
    explicit OvenController(const OvenControllerConfig& cfg = {});

    // --- ControllerBase interface ---
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "OvenController"; }
    const char* getDescription() const override {
        return "Model-inverse feedforward + SIMC PI on a bounded adaptive "
               "physical thermal model, with feasibility governor.";
    }

    // --- Configuration ---
    void setConfig(const OvenControllerConfig& cfg);
    const OvenControllerConfig& config() const { return m_cfg; }

    /**@brief Install a temperature profile. Resets the profile clock so
     *        profile time t=0 aligns with the next compute() call. */
    void setProfile(const TemperatureProfile& profile);
    void clearProfile();
    bool hasProfile() const { return m_hasProfile; }

    /**@brief Override the constant setpoint used when no profile is set. */
    void setAmbient(double ambientC);
    void setDisturbancePrior(double d);

    /**@brief Inject a known exogenous heat-flow disturbance [°C/s]
     *        (e.g. neighbor-zone coupling in a multi-zone oven).
     *
     *        The estimator's disturbance estimate already converges to
     *        the TOTAL disturbance (load + coupling) with its window
     *        lag, so the feedforward adds only the deviation of `d`
     *        from its recent average — steady-state exact, transient
     *        leading. Pass 0 to disable. Intended to be called once per
     *        control cycle by a supervisory multi-zone controller with
     *        the instantaneous coupling estimate computed from measured
     *        neighbor temperatures. */
    void setCouplingDisturbance(double d);

    /**@brief Force (re)estimation prior — e.g. after commissioning autotune.
     *        Applies to the ACTIVE model order: with modelOrder=Sopdt the
     *        FOPDT params are projected onto the cascade keeping the
     *        current τ2 (see OvenModelParamsSOPDT::fromFopdtParams). */
    void setModelParams(const OvenModelParams& params);

    /**@brief SOPDT variant of setModelParams. With modelOrder=Fopdt the
     *        params are projected onto the FOPDT model via
     *        OvenModelParamsSOPDT::toFopdtParams() (dominant-lag
     *        approximation). */
    void setModelParamsSOPDT(const OvenModelParamsSOPDT& params);

    /**@brief Apply an autotune result: model params + PI gains. */
    void applyAutotune(const OvenAutotuneResult& result);

    /**@brief SOPDT variant: cascade params + {kp,ki,kd} gains. */
    void applyAutotuneSOPDT(const OvenAutotuneResultSOPDT& result);

    /**
     * @brief Install a custom feedback backend at runtime.
     *
     * The backend replaces the SIMC-PI (or configured LQI) feedback block;
     * the feedforward/governor/estimator/dither layers are unchanged.
     * Backends implementing IOvenAdaptiveFeedback additionally receive
     * the applied-input stream and bounded parameter updates. Detection
     * happens once here (single dynamic_cast) — the per-cycle cost is one
     * virtual ControllerBase::compute() call, identical to the built-in
     * path. Pass nullptr to restore the configured default backend.
     */
    void setFeedbackBackend(std::unique_ptr<ControllerBase> backend);

    /**@brief Name of the active feedback backend (diagnostics). */
    const char* feedbackBackendName() const;

    // --- Accessors ---
    const OvenThermalModel& model() const { return m_model; }
    const ThermalParameterEstimator& estimator() const { return m_est; }
    const ActuationGovernor& governor() const { return m_gov; }
    double filteredMeasurement() const { return m_Tf; }
    double currentTime() const { return m_time; }
    OvenControllerStatus status() const;

protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;

private:
    void updateModelFromEstimator();
    void maybeRetune();
    void applyPiGains(double kp, double ki, double kd = 0.0);
    void installBackend(OvenFeedbackBackendKind kind);
    double delayedInput(double t) const;
    void pushInput(double t, double u);
    bool isSopdt() const { return m_cfg.modelOrder == OvenModelOrder::Sopdt; }
    int estUpdatesUsed() const;
    /**@brief Snapshot the active estimator's current outputs into the
     *        retune-baseline bookkeeping (m_lastAlpha/Beta/Tau2). Call
     *        after any setPrior()/setModelParams*() so the meaningful-
     *        change gate compares like with like. */
    void syncRetuneBaseline();
    /**@brief Design feedback gains for the configured tuneMethod from
     *        the current parameter estimates. */
    void designGains(double& kp, double& ki, double& kd) const;

    OvenControllerConfig m_cfg;
    TemperatureProfile m_profile;
    bool m_hasProfile{false};

    OvenThermalModel m_model;
    OvenThermalModelSOPDT m_modelSopdt;
    ThermalParameterEstimator m_est;
    SOPDTParameterEstimator m_estSopdt;
    ActuationGovernor m_gov;

    // Feedback backend: polymorphic ControllerBase — one virtual call per
    // cycle (negligible at oven loop rates); swap via config or
    // setFeedbackBackend() without touching the rest of the loop.
    std::unique_ptr<ControllerBase> m_feedback;
    IOvenAdaptiveFeedback* m_feedbackAdaptive{nullptr};
    PIDController* m_pidBackend{nullptr};   /**< Non-owning; null for LQI/custom */
    OvenFeedbackBackendKind m_backendKind{OvenFeedbackBackendKind::PI};

    // Runtime state
    double m_time{0.0};
    double m_Tf{25.0};
    bool m_TfInit{false};
    double m_lastU{0.0};
    double m_lastRef{0.0};
    bool m_overTemp{false};

    // Command history for delayed-input lookup (bounded)
    std::deque<std::pair<double, double>> m_uHist;

    // Retune bookkeeping
    double m_lastRetuneTime{-1e9};
    int m_lastRetuneWindows{0};
    double m_lastKp{0.0}, m_lastKi{0.0}, m_lastKd{0.0};
    double m_lastAlpha{0.0}, m_lastBeta{0.0}, m_lastTau2{0.0};
    double m_lastUfb{0.0};

    // Exogenous (coupling) disturbance injection — see
    // setCouplingDisturbance(). m_extDisturbAvg is an EMA (~estimator
    // window) of m_extDisturb; the FF/governor see
    // model.disturbance + (m_extDisturb - m_extDisturbAvg).
    double m_extDisturb{0.0};
    double m_extDisturbAvg{0.0};
    bool m_extDisturbInit{false};
};

} // namespace tether::control::thermal
