/**
 * @file ServoAxisLQIBackend.hpp
 * @brief LQI + steady-state Kalman observer feedback backend for a
 *        torque-mode servo axis.
 *
 * @details
 * Servo-axis analogue of OvenLQIBackend: a position servo that issues a
 * torque (current) command. The physical plant is the rigid-body
 * torque-mode axis
 *
 * ```
 *   J·θ̈ = Kt·u_d − b·θ̇ − τ_c·sgn(θ̇) − τ_d
 * ```
 *
 * where u is the torque command (delayed by the input dead time θ_d),
 * τ_c is Coulomb friction (compensated in feedforward) and τ_d is a
 * lumped disturbance torque — load torque, cable drag, friction misfit —
 * modelled as a random walk.
 *
 * ```
 *   encoder y = θ + ν ──► ServoAxisObserver ──► x̂ = [θ̂, ω̂, τ̂_d]
 *                            ▲                        │
 *            total applied u ─┘ (via delay line)       ▼
 *                                    LQIController on z = x̂ − x_ref
 *                                    u_fb = −Kx·z − Ki·∫(r−θ̂)
 *   u = u_ff(physics) + u_dob(τ̂_d) + u_fb
 * ```
 *
 * The observer is a steady-state Kalman filter over [θ, ω, τ_d]. It
 * predicts with the *total* applied torque command (supplied via
 * IServoAxisAdaptiveFeedback::notifyAppliedInput — feedforward does not
 * masquerade as process noise) and corrects with the encoder reading.
 * The disturbance-torque channel optionally feeds back as disturbance
 * feedforward −τ̂_d/Kt: constant/slow loads are cancelled almost
 * immediately instead of waiting for the integrator.
 *
 * Tuning philosophy: the *identification* parameters are physical —
 * inertia, viscous/coulomb friction, torque constant, dead time,
 * measurement variance (encoder quantization = res²/12). The *design*
 * knobs are Q/R weights with documented physical units, or a single
 * bandwidth λ + damping ζ via ServoAxisTuning::fromBandwidth() which
 * maps the rigid-plant LQR equivalence (k1=ω_n², k2=2ζω_n) onto weights.
 *
 * Online retuning: updateAxisParameters() re-derives the observer gain,
 * LQI gains and feedforward coefficients whenever the bounded estimator
 * reports new physical values — the same supervisory cadence as the
 * oven's SIMC-PI / OvenLQIBackend retune path. Parameter pushes are
 * validated (J > 0, Kt ≠ 0, b ≥ 0) before synthesis.
 *
 * Real-time: fixed-size delay buffer, no allocation in computeImpl();
 * Riccati solves happen only in design()/updateAxisParameters().
 */

#pragma once

#include "tether/control/ControllerBase.hpp"
#include "tether/control/StateSpaceControllers.hpp"

#include <array>

namespace tether::control::motion {

/**
 * @brief Physical parameters of a torque-mode servo axis.
 *
 * Rotary units (J in kg·m², torque in N·m, angle in rad); set
 * `linear=true` for a linear axis (mass in kg, force in N, position in
 * m) — the dynamics are identical, only the units change.
 */
struct ServoAxisParams {
    double inertia{1e-4};        /**< Reflected inertia J [kg·m²] / mass [kg] */
    double damping{1e-3};        /**< Viscous friction b [N·m·s] / [N·s/m] */
    double torqueConstant{1.0};  /**< Kt [N·m per unit command] */
    double coulombFriction{0.0}; /**< Coulomb friction τ_c [N·m] / [N] */
    double delay{0.0};           /**< Input dead time θ_d [s] */
    double uMin{-1.0};           /**< Minimum torque command */
    double uMax{1.0};            /**< Maximum torque command */
    bool linear{false};          /**< Linear axis (mass/force units) */

    /**@brief Sanity check for parameter pushes. */
    bool valid() const {
        return inertia > 0.0 && torqueConstant != 0.0 && damping >= 0.0 &&
               coulombFriction >= 0.0 && delay >= 0.0 && uMin < uMax;
    }
};

/**
 * @brief Observer and regulator tuning for the servo axis.
 *
 * Observer noise params are physical: rMeas is the encoder variance
 * (quantization = res²/12 for resolution `res`), qAccel is the spectral
 * density of unmodelled acceleration jitter, qDist is the random-walk
 * intensity of the disturbance torque (how fast τ_d may drift).
 *
 * The LQI weights are in physical units: qPos [1/rad²], qVel [s²/rad²],
 * qInt [1/(rad·s)²], r [1/u²]. fromBandwidth() derives them from a
 * desired closed-loop bandwidth + damping — the rigid-plant LQR
 * equivalence (dominant poles at ω_n, ζ; integrator placed ≈ λ/4) —
 * exact for the pure double-integrator plant, approximate with
 * damping/delay. Weights scale with (J/Kt)² to normalize the input.
 */
struct ServoAxisTuning {
    double qPos{1.0};    /**< LQI weight on position error [1/rad²] */
    double qVel{0.0};    /**< LQI weight on velocity error [s²/rad²] */
    double qInt{0.01};   /**< LQI weight on integral state [1/(rad·s)²] */
    double r{1.0};       /**< LQI weight on torque command [1/u²] */

    double qAccel{1.0};  /**< Observer accel process noise [(rad/s²)²·s] */
    double qDist{0.01};  /**< Observer disturbance-torque walk [(N·m)²·s] */
    double rMeas{1e-6};  /**< Encoder variance [rad²] (≈res²/12) */

    /**
     * @brief Derive weights from a desired closed-loop bandwidth.
     *
     * For the rigid plant θ̈ = (Kt/J)·u the continuous LQR gains satisfy
     * k1 = √(qPos/r_eff) ≈ ω_n², k2 ≈ 2ζω_n with r_eff = r/(J/Kt)².
     * Inverting gives qPos = (J/Kt)²·ω_n⁴·r and
     * qVel = (J/Kt)²·2ω_n²(2ζ²−1)·r (≥0). The integrator is placed a
     * factor ~4 below bandwidth: qInt = qPos·(ω_n/4)².
     *
     * @param p         Physical plant (for the J/Kt normalization)
     * @param lambdaRad Desired closed-loop bandwidth [rad/s]
     * @param damping   Desired damping ratio ζ (default 1.0, critical)
     */
    static ServoAxisTuning fromBandwidth(const ServoAxisParams& p,
                                         double lambdaRad,
                                         double damping = 1.0) {
        ServoAxisTuning t;
        const double wn = std::max(lambdaRad, 1e-3);
        const double z = std::max(damping, 0.2);
        const double jk = p.inertia / p.torqueConstant;
        t.r = 1.0;
        t.qPos = jk * jk * wn * wn * wn * wn;
        t.qVel = jk * jk * 2.0 * wn * wn *
                 std::max(2.0 * z * z - 1.0, 0.0);
        t.qInt = t.qPos * (wn * wn / 16.0);
        return t;
    }
};

/**
 * @brief Optional interface implemented by adaptive feedback backends.
 *
 * Mirrors IOvenAdaptiveFeedback: the outer loop wrapper reports the
 * actually-applied torque command (post-clamp/rate-limit) so the
 * observer sees the true input, and pushes bounded physical-parameter
 * updates for gain re-synthesis.
 */
/**
 * @brief Anti-windup mechanism for the LQI integrator.
 *
 * The authority-bounded integral state (|Ki·xi| ≤ ~1.2·uMax) is always
 * active and orthogonal to this choice — it only prevents *unbounded*
 * windup. The mode selects the dynamic behaviour while saturated:
 *
 * - kNone: no dynamic anti-windup. The integrator keeps accumulating
 *   up to its bound while saturated → overshoot on recovery. Useful as
 *   a diagnostic baseline, not for production.
 * - kConditional (default): conditional integration — the integrator
 *   freezes while the loop is saturated in the direction of the
 *   tracking error. Simplest robust scheme; zero extra knobs.
 * - kBackCalculation: tracking anti-windup — the integrator dynamics
 *   become xi_dot = e + (u_sat − u_desat)/Tt, driving the state toward
 *   the value consistent with the applied (saturated) input with time
 *   constant Tt. For an LQI regulator this is the Hanus observer-form
 *   conditioning — the standard scheme in industrial servo drives.
 */
enum class ServoAxisAWMode {
    kNone,
    kConditional,
    kBackCalculation,
};

class IServoAxisAdaptiveFeedback {
public:
    virtual ~IServoAxisAdaptiveFeedback() = default;

    /**
     * @brief Report the total torque command applied this cycle.
     *
     * Called by the wrapper after feedforward + feedback + limits — i.e.
     * exactly what the drive received. When the backend is used without
     * a wrapper, it falls back to its own last output.
     */
    virtual void notifyAppliedInput(double u) = 0;

    /**
     * @brief Push updated physical parameters for re-synthesis.
     *
     * {inertia, damping, torqueConstant, coulomb} come from the bounded
     * axis estimator (e.g. LiveTune's friction/dynamics identification).
     * Invalid sets (J ≤ 0, Kt = 0, b < 0) are rejected — returns false.
     * Called at most once per retune interval; the re-synthesis cost is
     * a small Riccati solve, off the hard real-time budget.
     */
    virtual bool updateAxisParameters(double inertia, double damping,
                                      double torqueConstant,
                                      double coulombFriction) = 0;

    /**@brief Human-readable backend identifier for status/diagnostics. */
    virtual const char* backendName() const = 0;
};

/**
 * @brief Steady-state Kalman observer for [θ, ω, τ_d], C = [1,0,0].
 *
 * Fixed-structure predict/correct filter: the 3×3 predictor Riccati is
 * iterated to convergence once per design(), so the per-cycle cost is a
 * handful of multiply-adds — same pattern as OvenThermalObserver.
 */
class ServoAxisObserver {
public:
    /**
     * @brief Design the observer for the given plant and noise levels.
     *
     * @param plant    Axis parameters (J, b, Kt)
     * @param dtDesign Nominal cycle time [s]
     * @param qAccel   Acceleration process-noise density [(rad/s²)²·s]
     * @param qDist    Disturbance-torque random-walk density [(N·m)²·s]
     * @param rMeas    Measurement variance [rad²] (quantization res²/12)
     */
    void design(const ServoAxisParams& plant, double dtDesign,
                double qAccel, double qDist, double rMeas);

    /**@brief Reset the estimate (θ to first measurement, ω and τ_d to 0). */
    void reset(double theta0);

    /**
     * @brief Prediction step: propagate the estimate over dt with the
     *        delayed applied torque uDelayed.
     */
    void predict(double uDelayed, double dt);

    /**@brief Measurement update with the raw encoder reading [rad]. */
    void correct(double y);

    double position() const { return m_x0; }           /**< θ̂ [rad] */
    double velocity() const { return m_x1; }           /**< ω̂ [rad/s] */
    double disturbanceTorque() const { return m_x2; }  /**< τ̂_d [N·m] */

private:
    // Continuous model coefficients (rebuilt on design/update)
    double m_bOverJ{0.0};  // b/J
    double m_ktOverJ{0.0}; // Kt/J
    double m_invJ{0.0};    // 1/J (disturbance channel)

    // Steady-state Kalman gain for C = [1,0,0]
    double m_L0{0}, m_L1{0}, m_L2{0};

    // State estimate
    double m_x0{0.0}, m_x1{0.0}, m_x2{0.0};
};

/**
 * @brief LQI position-servo backend on torque command.
 *
 * Feedback on the deviation state z = [θ̂−r, ω̂−ω_ref] plus an integrator
 * on position error; model-based feedforward u_ff = (J·α_ref + b·ω_ref
 * + τ_c·tanh(ω_ref/vEps))/Kt from the *current* physical parameters;
 * optional disturbance feedforward −τ̂_d/Kt; remaining-authority output
 * clamp [uMin−u_ff, uMax−u_ff].
 */
class ServoAxisLQIBackend : public ControllerBase,
                            public IServoAxisAdaptiveFeedback {
public:
    /**
     * @brief Construct the backend.
     *
     * @param plant     Nominal axis parameters
     * @param tuning    Observer + regulator tuning (see fromBandwidth())
     * @param dtDesign  Nominal cycle time [s]
     */
    ServoAxisLQIBackend(const ServoAxisParams& plant,
                        const ServoAxisTuning& tuning,
                        double dtDesign = 1e-3);

    // --- ControllerBase interface ---
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "ServoAxisLQIBackend"; }
    const char* getDescription() const override {
        return "LQI position servo on torque command with [θ, ω, τ_d] "
               "observer, physics feedforward and disturbance rejection.";
    }

    // --- IServoAxisAdaptiveFeedback ---
    void notifyAppliedInput(double u) override;
    bool updateAxisParameters(double inertia, double damping,
                              double torqueConstant,
                              double coulombFriction) override;
    const char* backendName() const override { return "LQI+DisturbanceObs"; }

    // --- Configuration ---
    /**@brief Update the full physical parameter set (validated). */
    bool setPlant(const ServoAxisParams& plant);
    /**@brief Update tuning weights and re-synthesize. */
    void setTuning(const ServoAxisTuning& tuning);
    /**
     * @brief Enable disturbance feedforward −τ̂_d/Kt (default on).
     * Cancels load torque ~immediately instead of via the integrator.
     * The estimate passes through a first-order Q-filter (see
     * setDisturbanceBandwidth) so observer transients are not injected
     * into the actuator — standard disturbance-observer practice.
     */
    void setDisturbanceFeedforward(bool enable) { m_dobFf = enable; }
    /**
     * @brief Disturbance-cancellation bandwidth [rad/s] — the DOB
     * Q-filter cutoff. Below this frequency the disturbance estimate is
     * passed through; above it is attenuated. Physical knob: how fast a
     * load change the FF may track. Default 15 rad/s.
     */
    void setDisturbanceBandwidth(double radPerSec) {
        m_dobBw = std::max(radPerSec, 1e-3);
    }
    /**
     * @brief Coulomb feedforward deadband velocity [rad/s] — tanh
     * smoothing of sgn(ω_ref); default 1e-3.
     */
    void setFrictionDeadband(double velEps) { m_velEps = std::abs(velEps); }

    // --- Anti-windup ---
    /**@brief Select the anti-windup mechanism (see ServoAxisAWMode). */
    void setAntiWindup(ServoAxisAWMode mode) { m_awMode = mode; }
    ServoAxisAWMode antiWindup() const { return m_awMode; }
    /**
     * @brief Tracking time Tt [s] for kBackCalculation — how fast the
     * integrator converges to the saturated-consistent state. Smaller →
     * faster desaturation, more sensitivity to measurement noise through
     * the feedback path. Rule of thumb: Tt ≈ 1/bandwidth. Default 4 ms.
     */
    void setAntiWindupTrackingTime(double seconds) {
        m_awTt = std::max(seconds, 1e-6);
    }
    /**
     * @brief Optional slew-rate limit on the torque command [u/s];
     * <=0 disables (default). Actuator protection — orthogonal to
     * anti-windup.
     */
    void setOutputRateLimit(double unitsPerSec) {
        m_rateLimit = unitsPerSec;
    }

    // --- Diagnostics ---
    const ServoAxisObserver& observer() const { return m_obs; }
    const ServoAxisParams& plant() const { return m_plant; }
    /**@brief Copy the discrete LQI gain K = [Kx(2) | Ki(1)]; false before design. */
    bool getGain(double* Kx, double* Ki) const;
    /**@brief Current observer disturbance-torque estimate [N·m]. */
    double estimatedDisturbance() const { return m_obs.disturbanceTorque(); }

protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;

private:
    void redesign(double dt);
    void pushApplied(double u);
    double delayedApplied(double theta) const;
    double frictionFeedforward(double velRef) const;

    // Plant + estimator
    ServoAxisParams m_plant;
    ServoAxisTuning m_tuning;
    ServoAxisObserver m_obs;
    LQIController m_lqi;

    double m_dtDesign;
    bool m_designed{false};
    bool m_dobFf{true};
    double m_dobBw{15.0};        /**< DOB Q-filter cutoff [rad/s] */
    double m_dobTau{0.0};        /**< Q-filtered disturbance estimate [N·m] */
    double m_velEps{1e-3};

    // Applied-input history for the observer's input delay (ring buffer)
    static constexpr int kDelayBufSize = 4096;
    std::array<double, kDelayBufSize> m_uBuf{};
    int m_uHead{0};
    int m_uCount{0};
    double m_uApplied{0.0};    /**< Applied input via notifyAppliedInput */
    double m_uLast{0.0};       /**< Last total output (standalone fallback) */
    bool m_notified{false};    /**< Wrapper called notify since last cycle */
    bool m_obsInit{false};     /**< Observer seeded with first measurement */
    bool m_saturatedHigh{false}; /**< Last cycle hit the upper authority rail */
    bool m_saturatedLow{false};  /**< Last cycle hit the lower authority rail */

    // Anti-windup state
    ServoAxisAWMode m_awMode{ServoAxisAWMode::kConditional};
    double m_awTt{4e-3};         /**< Back-calculation tracking time [s] */
    double m_rateLimit{0.0};     /**< Output slew limit [u/s], <=0 off */
};

} // namespace tether::control::motion
