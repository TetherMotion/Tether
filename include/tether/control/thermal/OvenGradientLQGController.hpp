/**
 * @file OvenGradientLQGController.hpp
 * @brief Centralized MIMO LQG gradient controller for multi-zone ovens.
 *
 * @details
 * `OvenGradientController` coordinates N zones as N *independent* SISO
 * `OvenController`s: each zone tracks its own shifted reference and
 * inter-zone coupling is injected only as a feedforward disturbance. That
 * is decentralized — the loop never regulates the spatial gradient
 * `T_i − T_{i−1}` as a controlled quantity, and coupling is cancelled
 * reactively rather than exploited by the feedback.
 *
 * This controller replaces the per-zone feedback with ONE centralized
 * MIMO LQG on the coupled thermal state:
 *
 * ```
 *   dx/dt = A·x + B·u + c ,   x = [T_1 … T_N]  (zone temperatures)
 *   y     = C·x = x           (each zone sensor is collocated, C = I)
 * ```
 *
 * - **Coupled model.** `A` carries the inter-zone coupling rates
 *   (`couplingLeft`/`couplingRight` from each `GradientZoneConfig`) as
 *   off-diagonal terms plus each zone's loss `β_i`; `B` is diagonal
 *   (`α_i` per-zone heater); `c` folds ambient + per-zone disturbance.
 *   The model is zero-order-hold discretized once at design time.
 *
 * - **LQI feedback.** An augmented regulator (states + one integrator per
 *   zone) is designed by discrete ARE on `(Ad, Bd, C = I)`. Following the
 *   deviation-state convention of `OvenLQIBackend`, the controller feeds
 *   `z = x̂ − r` with `r` the governed reference vector, so the ARE's
 *   absolute-state penalty acts on the *tracking error* and each
 *   integrator accumulates `∫(r_i − x̂_i)`. The state weight `Qx`
 *   combines a per-zone diagonal term with a chain Laplacian term
 *   `qxGradient·L` — `z'Lz = Σ(z_i − z_{i−1})²` is exactly the squared
 *   *gradient tracking error*, which is what decentralized SISO PID
 *   cannot express.
 *
 * - **LQG observer.** A steady-state Kalman filter (`KalmanFilter`,
 *   discrete DARE) estimates all zone temperatures. It predicts with the
 *   *delayed, actually-applied* input vector — so actuator dead time and
 *   saturation are not misread as process noise — plus the known affine
 *   term `c` (folded in as a constant input column).
 *
 * - **Previewed feedforward.** Each zone keeps its own `TemperatureProfile`
 *   + `ActuationGovernor` + `OvenThermalModel` (the "future looking"
 *   inverse model): the feedforward inverts the *governed, dead-time-
 *   previewed* reference trajectory per zone, exactly like the SISO path.
 *   Coupling is deliberately NOT injected into the FF — it lives in `A`
 *   and is handled by the centralized feedback.
 *
 * Real-time notes: `computeInto()` is allocation-free after zone setup —
 * fixed-size Eigen maps over per-zone scratch, one Kalman predict/update
 * (n ≤ 6 → trivial), and the DARE/ARE solves happen only in `design()` /
 * `retune()`. Zone count is fixed after the first `computeInto()` and
 * capped at `MAX_CONTROL_DIM` (6).
 *
 * Online adaptation: with `adaptParams` enabled, an
 * `OvenCoupledParameterEstimator` re-identifies the per-zone physical
 * parameters {α, β, γL, γR, d} of the *coupled* plant from closed-loop
 * I/O every cycle (bounded integral regression, excitation-gated), and
 * the LQI/observer gains are re-solved at `adaptRedesignInterval`.
 * `delay`, `ambient`, `uMin`/`uMax` remain fixed commissioning constants.
 */

#pragma once

#include "tether/control/KalmanFilter.hpp"
#include "tether/control/StateSpaceControllers.hpp"
#include "tether/control/thermal/ActuationGovernor.hpp"
#include "tether/control/thermal/OvenCoupledParameterEstimator.hpp"
#include "tether/control/thermal/OvenGradientController.hpp"
#include "tether/control/thermal/OvenThermalModel.hpp"
#include "tether/control/thermal/TemperatureProfile.hpp"

#include <Eigen/Dense>

#include <deque>
#include <memory>
#include <span>
#include <vector>

namespace tether::control::thermal {

/**@brief Tunables for the centralized MIMO LQG gradient controller. */
struct GradientLQGConfig {
    // --- LQI regulator weights ---
    /**@brief Per-zone absolute tracking weight `q_x·I` on the deviation
     *        state z = x̂ − r. Larger → tighter per-zone tracking. */
    double qxZone{1.0};

    /**@brief Gradient-tracking weight. Builds `Qx += qxGradient·L` where
     *        `L` is the path-graph Laplacian, so the ARE penalizes
     *        `Σ(z_i − z_{i−1})²` — the squared deviation of the measured
     *        adjacent-zone delta from its reference delta. 0 disables
     *        explicit gradient weighting (plain decentralized-equivalent
     *        MIMO LQI). */
    double qxGradient{4.0};

    /**@brief Integral weight `Qi` (per-zone integrator on r_i − x̂_i).
     *        Larger → faster steady-state/disturbance rejection. */
    double qi{0.5};

    /**@brief Control-effort weight `R` (diagonal, per zone). Larger →
     *        gentler actuation, slower loop. */
    double r{1.0};

    // --- LQG observer ---
    /**@brief Process-noise spectral density for the steady-state Kalman
     *        filter [°C²/s]. Larger → trust measurements more (faster
     *        state tracking, more sensor-noise feedthrough). */
    double qProc{1e-3};

    /**@brief Measurement-noise variance [°C²]. Larger → trust the model
     *        more (smoother estimate, slower disturbance tracking). */
    double rMeas{0.05};

    /**@brief Design/sample period used for the discrete ARE/DARE solves
     *        [s]. Should match the intended control `dt`. */
    double dtDesign{0.5};

    // --- Actuation ---
    /**@brief Enable per-zone model-inverse feedforward (previewed). */
    bool feedforward{true};

    /**@brief Per-zone actuator rate limit [u/s]; <=0 disables. */
    double uRateLimit{0.0};

    /**@brief Bound on each zone's integrator state (anti-windup).
     *        <=0 auto-scales from the integral gain and authority. */
    double integralLimit{0.0};

    /**@brief Back-calculation anti-windup tracking time [s]. When an
     *        actuator saturates or rate-limits, each integrator is pulled
     *        toward the value consistent with the *delivered* (clipped)
     *        output at rate ~1/Tt — prevents windup through long saturated
     *        transients such as a cold-start heat-up. <=0 disables. */
    double antiWindupTt{50.0};

    // --- Online adaptation (coupled MIMO parameter estimation) ---
    /**@brief Enable online re-identification of the coupled plant
     *        {α, β, γL, γR, d} per zone via `OvenCoupledParameterEstimator`
     *        (bounded, excitation-gated integral regression). The estimator
     *        refines every control cycle; the LQI/observer gains are
     *        re-solved at `adaptRedesignInterval`. `delay`, `ambient` and
     *        `uMin`/`uMax` stay fixed commissioning constants. */
    bool adaptParams{true};

    /**@brief Interval [s] between parameter→gain retunes while adapting.
     *        Bounds the ARE/DARE re-solve cost; the estimator keeps
     *        refining every cycle between retunes. <=0 re-solves every
     *        cycle (only sensible at slow loop rates). */
    double adaptRedesignInterval{5.0};

    /**@brief Coupled-estimator tuning: window length, forgetting,
     *        excitation gating, ridge and physical bounds. */
    CoupledEstimatorConfig estimator;

    /**@brief PE dither amplitude [u] added to each zone while adapting —
     *        a small sinusoid with a per-zone phase offset that decorrelates
     *        the coupled estimator's regressors in closed loop (without it,
     *        u ≈ f(T) and {α,β,γ,d} are only jointly identifiable up to a
     *        low-dimensional ambiguity). The temperature ripple is
     *        ~α·A/ω (~0.1–0.3 °C) — small against control bands. 0 disables
     *        (not recommended while `adaptParams` is on). */
    double adaptDitherAmp{0.025};

    /**@brief PE dither period [s]; non-commensurate with typical oven
     *        dynamics so it does not average out inside the estimator
     *        window. */
    double adaptDitherPeriod{37.0};

    /**@brief Minimum relative parameter change before the LQI/observer
     *        gains are re-solved — avoids an ARE/DARE re-solve for
     *        negligible estimator drift. */
    double adaptRetuneMinDelta{0.02};
};

class OvenGradientLQGController {
public:
    explicit OvenGradientLQGController(const GradientLQGConfig& cfg = {});

    /**@brief Add a zone. Zones MUST be added in spatial order — zone i's
     *        couplingLeft/couplingRight refer to zones i−1/i+1. Zone count
     *        is fixed once computeInto() has run and capped at 6. The
     *        per-zone `controller.model` (FOPDT) and `couplingLeft/Right`
     *        define the MIMO plant; `controller.governor` supplies the
     *        feasibility governor tuning. */
    void addZone(const GradientZoneConfig& cfg);

    size_t zoneCount() const { return m_zones.size(); }

    /**@brief Install an independent profile on one zone (overrides any
     *        gradient-spec profile for that zone). */
    void setZoneProfile(size_t zone, const TemperatureProfile& profile);

    /**@brief Set a constant setpoint for one zone (clears its profile). */
    void setZoneSetpoint(size_t zone, double tempC);

    /**@brief Install constant setpoints for all zones at once, with
     *        optional adjacent-delta clamping (same semantics as
     *        OvenGradientController::setZoneSetpoints). */
    void setZoneSetpoints(const std::vector<double>& tempsC,
                          double maxDeltaC = -1.0);

    /**@brief Build per-zone profiles from a GradientSpec and install
     *        them (same offset + time-shift + delta-clamp semantics as
     *        OvenGradientController::setGradientProfile). */
    bool setGradientProfile(const GradientSpec& spec);

    /**@brief Advance all zones one control cycle (allocating convenience
     *        form — see computeInto for the RT path).
     * @param measured Measured zone temperatures [°C], spatial order,
     *                 size == zoneCount().
     * @param dt       Cycle time [s].
     * @return Per-zone actuator commands u_i in [uMin,uMax]. */
    std::vector<double> compute(std::span<const double> measured,
                                double dt);

    /**@brief Allocation-free form of compute(): writes per-zone commands
     *        into `out` (size == zoneCount()). Returns false without
     *        touching state when sizes mismatch or the design failed. */
    bool computeInto(std::span<const double> measured, double dt,
                     std::span<double> out);

    /**@brief Largest absolute measured adjacent-zone delta [°C]. */
    double maxMeasuredDelta() const;

    /**@brief Largest absolute reference adjacent-zone delta [°C] from the
     *        last computeInto(). */
    double maxSetpointDelta() const { return m_maxRefDelta; }

    /**@brief True when the MIMO design (ARE + DARE) succeeded. */
    bool designed() const { return m_designed; }

    /**@brief Per-zone observer state estimate [°C] from the last
     *        computeInto() (size == zoneCount() when designed). */
    std::span<const double> estimatedStates() const {
        return {m_xHat.data(), static_cast<size_t>(m_n)};
    }

    /**@brief Per-zone integral state (∫(r_i − x̂_i)) from the last
     *        computeInto(). */
    std::span<const double> integralStates() const {
        return {m_xi.data(), static_cast<size_t>(m_n)};
    }

    /**@brief True when online coupled-parameter adaptation is enabled. */
    bool adaptParamsEnabled() const { return m_cfg.adaptParams; }

    /**@brief Current coupled-estimator parameter estimate for a zone
     *        {α, β, γL, γR, d} (commissioning prior until the estimator
     *        has excited updates). Valid only when adaptParams is on. */
    CoupledZoneEstimate zoneParameterEstimate(size_t zone) const;

    /**@brief Whether the estimator counted the last cycle for `zone` as
     *        excited (identifiable). Diagnostics for adaptation monitoring. */
    bool zoneEstimatorExcited(size_t zone) const;

    /**@brief Reset all per-zone profiles' dynamic state, the observer,
     *        integrators and input history. The parameter estimator keeps
     *        its current estimates (they are plant properties). */
    void reset();

private:
    struct Zone {
        GradientZoneConfig cfg;
        TemperatureProfile profile;
        bool hasProfile{false};
        double setpoint{25.0};
        double uMin{0.0}, uMax{1.0};
        double alpha{0.0}, beta{0.0}, delay{0.0};
        double ambient{25.0}, disturbance{0.0};
        double couplingLeft{0.0}, couplingRight{0.0};

        ActuationGovernor gov;
        OvenThermalModel ffModel;
        // Per-zone applied-input history for the observer's dead-time
        // compensation: (time, u_i).
        std::deque<std::pair<double, double>> uHist;

        // Per-cycle scratch (no allocation in computeInto)
        double ref{0.0}, refDot{0.0};
        double u{0.0};
    };

    /**@brief Solve the coupled model + LQI/observer gains into the member
     *        matrices. Preserves all dynamic state (integrators, observer
     *        estimate, input history, clock) — used by both the initial
     *        `design()` and the adaptation `retune()`. */
    bool solveModel();

    /**@brief Full (re)design: solveModel() then reset integrators and
     *        observer seed. Used on zone-set changes / first compute. */
    bool design();

    /**@brief Adaptation retune: solveModel() + observer re-seed, but
     *        preserves integrator states and the clock/input history so a
     *        parameter update does not bump the loop. */
    bool retune();

    /**@brief Pull current estimator estimates into the per-zone physical
     *        parameters and refresh each zone's FF model + governor.
     * @return true if any parameter changed enough to warrant retune. */
    bool syncParamsFromEstimator();

    /**@brief Feed the estimator one cycle and schedule a retune. */
    void maybeAdapt(std::span<const double> measured, double dt);

    double delayedInput(size_t zone, double tLookback) const;

    GradientLQGConfig m_cfg;
    std::vector<std::unique_ptr<Zone>> m_zones;

    int m_n{0};   // number of zones (== states == inputs == outputs)
    bool m_designed{false};
    bool m_obsInit{false};

    // Discrete MIMO model (delay-free; dead time handled via delayed input)
    Eigen::MatrixXd m_Ad, m_Bd;      // n×n, n×n
    Eigen::VectorXd m_gd;            // constant affine response (n)
    Eigen::VectorXd m_c;             // continuous affine term (n)

    // LQI gains (from LQIController::design on the augmented system)
    Eigen::MatrixXd m_Kx;            // n×n state feedback
    Eigen::MatrixXd m_Ki;            // n×n integral feedback
    std::vector<double> m_xiLim;     // per-zone integrator clamp

    // LQG observer
    KalmanFilter m_kf;

    // Per-cycle state
    double m_time{0.0};
    double m_maxRefDelta{0.0};
    Eigen::VectorXd m_xHat;          // observer state estimate
    Eigen::VectorXd m_xi;            // integrator states
    std::vector<double> m_lastU;
    std::vector<double> m_lastMeasured;

    // Scratch Eigen vectors (allocated once at design)
    Eigen::VectorXd m_z, m_uAug, m_y, m_ufb;
    double m_maxDelay{0.0};

    // Online coupled-parameter adaptation
    OvenCoupledParameterEstimator m_est;
    bool m_estInit{false};
    double m_nextAdapt{0.0};
    std::vector<double> m_uDelayed;   // per-zone delayed applied input
    std::vector<double> m_ambient;    // per-zone ambient (estimator regressor)
};

} // namespace tether::control::thermal
