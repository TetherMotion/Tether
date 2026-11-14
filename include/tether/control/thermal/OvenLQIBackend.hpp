/**
 * @file OvenLQIBackend.hpp
 * @brief LQI + steady-state Kalman observer feedback backend for
 *        non-collocated oven sensing.
 *
 * @details
 * Use this backend when the temperature sensor does NOT measure the
 * thermal mass that should follow the profile — e.g. chamber-air
 * thermistor vs. product/load core temperature. The backend wraps:
 *
 * ```
 *   measured y (zone 1) ──► OvenThermalObserver ──► x̂ = [T̂_s, T̂_p]
 *                              ▲                        │
 *              total applied u ─┘ (via delay line)       ▼
 *                                          LQIController (C = [0,1])
 *                                          u_fb = -Kx·x̂ - Ki·xᵢ
 * ```
 *
 * The observer is a steady-state Kalman filter for the two-state
 * OvenCoupledParams model: it predicts with the *total* actuator output
 * actually applied (feedforward + feedback + dither, after clamping and
 * rate limiting — supplied via IOvenAdaptiveFeedback::notifyAppliedInput)
 * and corrects with the raw measurement. Because it sees the true applied
 * input, feedforward does not appear as fictitious process noise.
 *
 * The controlled output is the product-zone temperature (C_ctrl = [0,1]):
 * the LQI integrator drives x̂_p to the reference, absorbing the steady
 * sensor↔product offset and load disturbances that the SISO feedforward
 * cannot see. State feedback on x̂_s provides damping of the fast zone.
 *
 * Online retuning: updateThermalParameters() re-derives the observer gain
 * and LQI gains whenever the bounded {α, β, d} estimator reports new
 * values — the same supervisory cadence as the SIMC-PI retune path.
 *
 * Real-time: fixed-size delay buffer, no allocation in computeImpl();
 * Riccati solves happen only in design()/updateThermalParameters().
 */

#pragma once

#include "tether/control/ControllerBase.hpp"
#include "tether/control/StateSpaceControllers.hpp"
#include "tether/control/thermal/OvenCoupledModel.hpp"

#include <array>

namespace tether::control::thermal {

/**
 * @brief Optional interface implemented by adaptive feedback backends.
 *
 * OvenController detects this interface once at backend-install time
 * (single dynamic_cast — zero per-cycle cost) and uses it to (a) report
 * the actually-applied actuator output so model-based observers see the
 * true input, and (b) push bounded physical-parameter updates for gain
 * re-synthesis.
 */
class IOvenAdaptiveFeedback {
public:
    virtual ~IOvenAdaptiveFeedback() = default;

    /**
     * @brief Report the total actuator output applied this cycle.
     *
     * Called by OvenController after feedforward + feedback + dither,
     * over-temperature interlock, rate limiting and saturation — i.e.
     * exactly what the plant received. Observer-based backends must use
     * this (not their own raw output) for prediction.
     */
    virtual void notifyAppliedInput(double u) = 0;

    /**
     * @brief Push updated physical parameters for re-synthesis.
     *
     * {alpha, beta, disturbance} come from the bounded thermal parameter
     * estimator (sensor-path FOPDT identification). Implementations map
     * them onto their internal model and redesign gains. Called at most
     * once per retuneInterval — never from the hard real-time path
     * budget beyond a small matrix solve.
     */
    virtual void updateThermalParameters(double alpha, double beta,
                                         double disturbance) = 0;

    /**@brief Human-readable backend identifier for status/diagnostics. */
    virtual const char* backendName() const = 0;
};

/**
 * @brief Steady-state Kalman observer for the two-state coupled model.
 *
 * Fixed-structure predict/correct filter: the Kalman gain is computed
 * once per design() by iterating the 2×2 predictor Riccati equation to
 * convergence, so the per-cycle cost is two multiply-adds.
 */
class OvenThermalObserver {
public:
    /**
     * @brief Design the observer for the given plant and noise levels.
     *
     * @param plant    Coupled-model parameters (A, B, c, delay)
     * @param dtDesign Nominal cycle time used for the steady-state gain
     * @param qProc    Process-noise spectral density [°C²/s³-ish];
     *                 larger → trust measurements more (faster tracking,
     *                 more noise feedthrough)
     * @param rMeas    Measurement-noise variance [°C²]
     */
    void design(const OvenCoupledParams& plant, double dtDesign,
                double qProc, double rMeas);

    /**@brief Reset the estimate (e.g. to ambient or first measurement). */
    void reset(double y0);

    /**
     * @brief Prediction step: propagate the estimate over dt with the
     *        delayed applied input uDelayed.
     */
    void predict(double uDelayed, double dt);

    /**@brief Measurement update with the raw sensor reading. */
    void correct(double y);

    double xSensor() const { return m_x1; }    /**< Zone-1 (sensor) estimate [°C] */
    double xProduct() const { return m_x2; }   /**< Zone-2 (product) estimate [°C] */

private:
    // Continuous model coefficients (rebuilt on design/update)
    double m_a11{0}, m_a12{0}, m_a21{0}, m_a22{0};
    double m_b1{0};
    double m_c1{0}, m_c2{0};
    double m_ambient{25.0};

    // Steady-state Kalman gain for C = [1, 0]
    double m_L1{0}, m_L2{0};

    // State estimate
    double m_x1{25.0}, m_x2{25.0};
};

/**
 * @brief LQI state-feedback backend with integrated thermal observer.
 *
 * Plugs into OvenController exactly like the default SIMC-PI backend —
 * same ControllerBase interface, same dynamic saturation limits, same
 * over-temperature path. The only differences are internal: an observer
 * reconstructs the unmeasured product temperature and the LQI gain
 * (re-synthesized on parameter updates) replaces the SIMC PI law.
 */
class OvenLQIBackend : public ControllerBase, public IOvenAdaptiveFeedback {
public:
    /**
     * @brief Construct the backend.
     *
     * @param plant      Nominal coupled-model parameters
     * @param qxSensor   LQI state weight on zone-1 temperature [1/°C²]
     * @param qxProduct  LQI state weight on zone-2 (product) temperature
     * @param qi         LQI integral weight
     * @param r          LQI control weight
     * @param qProc      Observer process-noise density
     * @param rMeas      Observer measurement-noise variance [°C²]
     * @param dtDesign   Nominal cycle time for observer/LQI design [s]
     */
    OvenLQIBackend(const OvenCoupledParams& plant,
                   double qxSensor, double qxProduct, double qi, double r,
                   double qProc, double rMeas, double dtDesign = 0.5);

    // --- ControllerBase interface ---
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "OvenLQIBackend"; }
    const char* getDescription() const override {
        return "LQI state feedback on observer-estimated product "
               "temperature (non-collocated sensing).";
    }

    // --- IOvenAdaptiveFeedback ---
    void notifyAppliedInput(double u) override;
    void updateThermalParameters(double alpha, double beta,
                                 double disturbance) override;
    const char* backendName() const override { return "LQI+Observer"; }

    // --- Diagnostics / tuning ---
    const OvenThermalObserver& observer() const { return m_obs; }
    const OvenCoupledParams& plant() const { return m_plant; }
    /**@brief Copy the discrete LQI gain K = [Kx | Ki]; false before design. */
    bool getGain(double* Kx, double* Ki) const;

protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;

private:
    void redesign(double dt);
    void pushApplied(double u);
    double delayedApplied(double theta) const;

    // Plant + estimator
    OvenCoupledParams m_plant;
    OvenThermalObserver m_obs;
    LQIController m_lqi;

    // LQI weights (fixed; physical params flow through the plant)
    double m_qxSensor, m_qxProduct, m_qi, m_r;
    double m_qProc, m_rMeas;
    double m_dtDesign;
    bool m_designed{false};

    // Applied-input history for the observer's input delay (ring buffer)
    static constexpr int kDelayBufSize = 4096;
    std::array<double, kDelayBufSize> m_uBuf{};
    int m_uHead{0};
    int m_uCount{0};
    double m_uApplied{0.0};   /**< Last applied total input (notifyAppliedInput) */
    double m_uFbLast{0.0};    /**< Last feedback output (fallback before design) */
    bool m_obsInit{false};    /**< Observer seeded with first measurement */
};

} // namespace tether::control::thermal
