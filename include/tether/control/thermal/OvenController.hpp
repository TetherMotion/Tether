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
 *  model-inverse FF          PI feedback trim
 *  u_ff = requiredInput(     on (r̂ - T_filt)
 *    r̂(t+θ), ṙ̂, T_amb, d̂)        │
 *        │                          │
 *        └───────────┬──────────────┘
 *                    ▼
 *              u = clamp(u_ff + u_fb, uMin, uMax)
 *                    │
 *      ThermalParameterEstimator ──→ bounded {α, β, d} adaptation
 *      (windowed LS, excitation-gated)   ↺ updates model + retunes PI
 * ```
 *
 * Key properties:
 *  - The feedback PI gains are *outputs* of the physical model via SIMC
 *    rules, re-derived whenever the online estimator updates {α, β}.
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
#include "tether/control/thermal/OvenThermalModel.hpp"
#include "tether/control/thermal/TemperatureProfile.hpp"
#include "tether/control/thermal/ThermalParameterEstimator.hpp"

#include <deque>
#include <memory>

namespace tether::control::thermal {

/**@brief Configuration for OvenController. */
struct OvenControllerConfig {
    // --- Plant model (nominal/initial physical parameters) ---
    OvenModelParams model;

    // --- Subsystem configs ---
    GovernorConfig governor;
    EstimatorConfig estimator;

    // --- Feedback PI ---
    double kp{0.0};           /**< PI proportional gain; 0 = auto (SIMC) */
    double ki{0.0};           /**< PI integral gain;     0 = auto (SIMC) */
    double simcTauC{0.0};     /**< SIMC closed-loop τc [s]; <=0 → τc = θ */
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
    int windowsUsed{0};
    bool saturated{false};
    bool overTemp{false};
    bool adapting{false};
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

    /**@brief Force (re)estimation prior — e.g. after commissioning autotune. */
    void setModelParams(const OvenModelParams& params);

    /**@brief Apply an autotune result: model params + PI gains. */
    void applyAutotune(const OvenAutotuneResult& result);

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
    void applyPiGains(double kp, double ki);
    double delayedInput(double t) const;
    void pushInput(double t, double u);

    OvenControllerConfig m_cfg;
    TemperatureProfile m_profile;
    bool m_hasProfile{false};

    OvenThermalModel m_model;
    ThermalParameterEstimator m_est;
    ActuationGovernor m_gov;
    PIDController m_pid;

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
    double m_lastKp{0.0}, m_lastKi{0.0};
};

} // namespace tether::control::thermal
