/**
 * @file OvenThermalModel.hpp
 * @brief Lumped first-order-plus-dead-time thermal model for oven control.
 *
 * @details
 * Models a heated chamber as a single thermal mass with Newtonian heat
 * loss to ambient, heater input delay, and an additive disturbance term:
 *
 * ```
 * C_th · dT/dt = P_heater · u(t - θ) - (T - T_amb) / R_th + Q_load(t)
 * ```
 *
 * Normalized to unitless input `u` (typically PWM fraction in [0,1]):
 *
 * ```
 * dT/dt = α·u(t-θ) - β·(T - T_amb) + d(t)
 *
 *   α = η·P_max / C_th   [°C/s per unit u]  — heating authority per heat capacity
 *   β = 1 / (R_th·C_th)  [1/s]              — inverse thermal time constant
 *   d = Q_load / C_th    [°C/s]             — disturbance (load heat flow)
 * ```
 *
 * This is a FOPDT plant: DC gain K = α/β [°C/u], time constant τ = 1/β [s],
 * dead time θ [s]. The parameters are physical: α, β, θ can be estimated
 * from I/O data and bounded online (see ThermalParameterEstimator).
 *
 * Note on identifiability: from input/output data alone, C_th and R_th
 * cannot be separated — only the combinations α, β (equivalently K, τ)
 * are identifiable unless actual heater watts and a calibration experiment
 * are available. If the nominal power scale is known, an effective heat
 * capacity estimate is `C_eff = (η·P_max) / α`.
 */

#pragma once

#include <deque>
#include <utility>
#include <vector>

namespace tether::control::thermal {

/**@brief Physical parameters of the lumped oven thermal model. */
struct OvenModelParams {
    double alpha{0.02};       /**< Heating coefficient [°C/s per unit u] = η·P/C */
    double beta{0.005};       /**< Loss coefficient [1/s] = 1/(R·C) */
    double delay{2.0};        /**< Input dead time [s] */
    double disturbance{0.0};  /**< Additive disturbance estimate [°C/s] */
    double ambient{25.0};     /**< Ambient temperature [°C] */
    double uMin{0.0};         /**< Minimum actuator command */
    double uMax{1.0};         /**< Maximum actuator command */

    /**@brief DC gain K = α/β [°C per unit u] (clamped for β→0). */
    double gain() const {
        return beta > 1e-9 ? alpha / beta : 1e9;
    }

    /**@brief Time constant τ = 1/β [s] (clamped for β→0). */
    double tau() const {
        return beta > 1e-9 ? 1.0 / beta : 1e9;
    }

    /**@brief Effective heat capacity if power-per-unit-u is known [J/K]. */
    double effectiveHeatCapacity(double powerPerUnitU) const {
        return powerPerUnitU > 0.0 ? powerPerUnitU / alpha : 0.0;
    }
};

/**
 * @brief First-order-plus-dead-time oven thermal model.
 *
 * Used both as the internal prediction model of OvenController and as a
 * standalone simulation model for testing and preview.
 */
class OvenThermalModel {
public:
    explicit OvenThermalModel(const OvenModelParams& params = {})
        : m_params(params) {}

    /**@brief Replace all model parameters (resets delay line and state). */
    void setParams(const OvenModelParams& params) {
        m_params = params;
        m_delayLine.clear();
        m_T = params.ambient;
    }

    const OvenModelParams& params() const { return m_params; }
    OvenModelParams& params() { return m_params; }

    /**@brief Current simulated/measured-referenced temperature [°C]. */
    double temperature() const { return m_T; }

    /**@brief Advance the model one step.
     * @param u   Actuator command applied over [t, t+dt].
     * @param dt  Step size [s].
     * @return New model temperature [°C].
     *
     * The model keeps an internal clock advanced by dt; the input delay
     * line makes u affect the temperature `delay` seconds after issue.
     */
    double step(double u, double dt);

    /**@brief Current internal model time [s]. */
    double time() const { return m_time; }

    /**@brief Model-inverse feedforward: input needed to make dT/dt = tRefDot
     *        while T = tRef, evaluated with current ambient/disturbance.
     * @param tRef     Reference temperature [°C].
     * @param tRefDot  Reference temperature rate [°C/s].
     * @param ambient  Ambient temperature [°C].
     * @param disturb  Disturbance estimate [°C/s].
     * @return Required actuator command (unclamped unless clampOutput).
     */
    double requiredInput(double tRef, double tRefDot,
                         double ambient, double disturb) const;

    /**@brief Same as requiredInput but clamped to [uMin, uMax]. */
    double requiredInputClamped(double tRef, double tRefDot,
                                double ambient, double disturb) const;

    /**@brief Convert to FOPDT form {K, tau, L} for tuning rules. */
    std::pair<double, double> gainAndTau() const { return {m_params.gain(), m_params.tau()}; }

    /**@brief Simulate the model open-loop for `steps` steps.
     * @param uConst Constant input, or nullptr for zero input.
     * @param dt     Step size [s].
     * @param steps  Number of steps.
     * @param tAmb   Ambient temperature [°C] (constant during run).
     * @param d      Disturbance [°C/s] (constant during run).
     * @return Temperature trajectory (steps+1 samples, T[0]=initial).
     */
    std::vector<double> simulate(double uConst, double dt, int steps,
                                 double tAmb, double d) const;

    /**@brief Reset simulated state to ambient (keeps internal clock at 0). */
    void reset() { m_T = m_params.ambient; m_delayLine.clear(); m_time = 0.0; }

private:
    // Delay line: stores (time, u) pairs; value(tau) = u at latest sample <= tau.
    // Samples before the first recorded one return uInit — the assumed
    // input prior to t=0 (0 = heater off, the correct step-test semantics).
    struct DelayLine {
        std::deque<std::pair<double, double>> buf;
        double uInit{0.0};
        void clear() { buf.clear(); }
        void push(double t, double u) {
            buf.emplace_back(t, u);
            // bound memory: thermal delays are seconds; 8192 samples covers
            // >2h at dt=1s and is ample for any realistic control rate.
            while (buf.size() > 8192) buf.pop_front();
        }
        double value(double tau) const {
            if (buf.empty()) return uInit;
            if (tau < buf.front().first) return uInit;
            // latest sample with time <= tau
            double result = buf.front().second;
            for (const auto& [t, u] : buf) {
                if (t <= tau) result = u;
                else break;
            }
            return result;
        }
    };

    OvenModelParams m_params;
    double m_T{25.0};
    double m_time{0.0};
    DelayLine m_delayLine;
};

} // namespace tether::control::thermal
