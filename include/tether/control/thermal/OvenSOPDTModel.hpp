/**
 * @file OvenSOPDTModel.hpp
 * @brief Second-order-plus-dead-time oven thermal model.
 *
 * @details
 * Cascade realization of the SOPDT plant
 *
 * ```
 * K·e^(-Ls) / ((tau1·s + 1)·(tau2·s + 1))
 * ```
 *
 * built from the FOPDT thermal model plus one extra lag state:
 *
 * ```
 * dT1/dt = α·u(t−θ) − β·(T1 − T_amb) + d      (stage 1: lossy mass)
 * dT2/dt = (T1 − T2) / τ2                     (stage 2: sensor/product mass)
 * y      = T2
 * ```
 *
 * Stage 1 is exactly the FOPDT model (heater/air-side thermal mass with
 * Newtonian loss to ambient and the disturbance input). Stage 2 is a
 * pure first-order lag representing a second thermal mass — e.g. the
 * product/load or a remote sensor — coupled to stage 1 but with no
 * direct ambient loss of its own (its heat leaves through stage 1).
 *
 * Properties:
 *  - Transfer function u→y is exactly K·e^(−Ls)/((τ1s+1)(τ2s+1)) with
 *    K = α/β [°C/u] and τ1 = 1/β [s]. τ2→0 recovers the FOPDT model.
 *  - Finite DC gain and passive cooling are preserved (unlike a bare
 *    SOPDT transfer function, which has no restoring force).
 *  - Physical parameters: α, β, τ2, θ can be commissioned from step
 *    tests and {α, β, d} adapted online (see ThermalParameterEstimator
 *    and SOPDTParameterEstimator).
 *
 * The model-inverse feedforward uses the exact inverse: tracking r(t)
 * requires stage-1 to follow T1_req = r + τ2·ṙ, so
 *
 * ```
 * u_ff = [ṙ + τ2·r̈ + β·(r + τ2·ṙ − T_amb) − d] / α
 * ```
 *
 * evaluated on the dead-time-previewed reference. On ramp/soak profiles
 * r̈ = 0 almost everywhere, so the inverse is exact on segments.
 */

#pragma once

#include "tether/control/thermal/OvenThermalModel.hpp"

#include <cmath>
#include <deque>
#include <utility>
#include <vector>

namespace tether::control::thermal {

namespace detail {

/**
 * @brief Cached zero-order-hold coefficients for one cascade step.
 *
 * Exact ZOH propagation of the cascade in deviation coordinates
 * y_i = T_i - T_amb:
 *
 *   y1' = -beta*y1 + v          (v = alpha*u_d + d, constant over step)
 *   y2' = (y1 - y2)/tau2
 *
 * A = [[-beta, 0], [1/tau2, -1/tau2]] is lower triangular, so exp(A*dt)
 * and the ZOH input map are closed-form. Construct once per
 * (beta, tau2, dt) and reuse step() — used by both the model (one
 * step per call) and the OE parameter estimator (thousands of steps
 * per fit).
 */
struct CascadeZoh {
    /**@brief Precompute coefficients for (beta, tau2, dt). */
    void init(double beta_, double tau2_, double dt_);

    /**@brief Advance (y1,y2) one step with constant input v. */
    std::pair<double, double> step(double y1, double y2, double v) const {
        const double y1n = m_a1 * y1 + m_cv1 * v;
        const double y2n = m_e21 * y1 + m_a2 * y2 + m_cv2 * v;
        return {y1n, y2n};
    }

private:
    // Unified ZOH form for every case (locked stages, integrator,
    // resonant, general):
    //   y1n = a1*y1 + cv1*v
    //   y2n = e21*y1 + a2*y2 + cv2*v
    double m_a1{0.0}, m_cv1{0.0};
    double m_e21{0.0}, m_a2{0.0}, m_cv2{0.0};
};

} // namespace detail

/**@brief Physical parameters of the cascade SOPDT oven model. */
struct OvenModelParamsSOPDT {
    double alpha{0.02};       /**< Stage-1 heating coefficient [°C/s/u] */
    double beta{0.005};       /**< Stage-1 loss coefficient [1/s] */
    double tau2{60.0};        /**< Stage-2 lag time constant [s]; 0 = FOPDT */
    double delay{2.0};        /**< Input dead time [s] */
    double disturbance{0.0};  /**< Disturbance into stage 1 [°C/s] */
    double ambient{25.0};     /**< Ambient temperature [°C] */
    double uMin{0.0};         /**< Minimum actuator command */
    double uMax{1.0};         /**< Maximum actuator command */

    /**@brief Stage-1 time constant τ1 = 1/β [s]. */
    double tau1() const { return beta > 1e-9 ? 1.0 / beta : 1e9; }

    /**@brief DC gain K = α/β [°C/u] (same as FOPDT). */
    double gain() const { return beta > 1e-9 ? alpha / beta : 1e9; }

    /**@brief Quasi-static effective parameters for the OUTPUT stage.
     *
     *  At slow timescales the cascade behaves like a FOPDT with
     *  α_eff = α/(1+β·τ2), β_eff = β/(1+β·τ2): identical DC gain
     *  (α_eff/β_eff = α/β) and dominant time constant τ1+τ2 = 1/β_eff.
     *  Used by the actuation governor for feasibility rates. */
    std::pair<double, double> effectiveAlphaBeta() const {
        const double s = 1.0 + beta * tau2;
        return {alpha / s, beta / s};
    }

    /**@brief Convert to the equivalent FOPDT parameter set (τ2 folded
     *        into the dominant time constant). */
    OvenModelParams toFopdtParams() const {
        OvenModelParams f;
        const auto [ae, be] = effectiveAlphaBeta();
        f.alpha = ae;
        f.beta = be;
        f.delay = delay;
        f.disturbance = disturbance;
        f.ambient = ambient;
        f.uMin = uMin;
        f.uMax = uMax;
        return f;
    }

    /**@brief Project a FOPDT parameter set onto the cascade, preserving
     *        the FOPDT effective dynamics {K, τ_eff = τ_stage1 + τ2}.
     *
     *  The quasi-static effective parameters satisfy
     *  β_eff = β/(1+β·τ2), α_eff = α/(1+β·τ2), so matching
     *  {α_eff, β_eff} = {α_f, β_f} gives stage-1
     *  τ_stage1 = τ_f − τ2 (clamped to ≥ 0.2·τ_f when τ2 is large
     *  relative to the FOPDT lag — the projection is then only
     *  approximate; use the ARX estimator for well-posed splits).
     *  τ2 = 0 recovers the FOPDT params exactly. */
    static OvenModelParamsSOPDT fromFopdtParams(const OvenModelParams& f,
                                              double tau2New) {
        OvenModelParamsSOPDT s;
        const double tauF = f.tau();
        const double K = f.gain();
        double tau1;
        if (tau2New > 1e-9 && std::isfinite(tauF) && tauF < 1e8) {
            tau1 = std::max(tauF - tau2New, 0.2 * tauF);
        } else {
            tau1 = (std::isfinite(tauF) && tauF < 1e8) ? tauF : 200.0;
        }
        s.beta = 1.0 / tau1;
        s.alpha = K * s.beta;    // K = α/β preserved exactly
        s.tau2 = std::max(0.0, tau2New);
        s.delay = f.delay;
        s.disturbance = f.disturbance;
        s.ambient = f.ambient;
        s.uMin = f.uMin;
        s.uMax = f.uMax;
        return s;
    }
};

/**
 * @brief Cascade SOPDT oven thermal model.
 *
 * Simulation uses the exact zero-order-hold discretization of the
 * two-state cascade (detail::CascadeZoh): both stage temperatures are
 * propagated analytically over each step, exact for any dt and for the
 * degenerate cases (τ2→0 FOPDT, β→0 integrator, β=1/τ2 resonance).
 */
class OvenThermalModelSOPDT {
public:
    explicit OvenThermalModelSOPDT(const OvenModelParamsSOPDT& params = {})
        : m_params(params), m_T1(params.ambient), m_T2(params.ambient) {}

    /**@brief Replace all model parameters (resets delay line and states). */
    void setParams(const OvenModelParamsSOPDT& params) {
        m_params = params;
        m_delayLine.clear();
        m_T1 = params.ambient;
        m_T2 = params.ambient;
    }

    const OvenModelParamsSOPDT& params() const { return m_params; }
    OvenModelParamsSOPDT& params() { return m_params; }

    /**@brief Stage-1 (sensor-side mass) temperature [°C]. */
    double temperature1() const { return m_T1; }

    /**@brief Stage-2 (output / product mass) temperature [°C]. */
    double temperature2() const { return m_T2; }

    /**@brief Controlled output — stage-2 temperature [°C]. */
    double temperature() const { return m_T2; }

    /**@brief Advance the model one step.
     * @param u   Actuator command applied over [t, t+dt].
     * @param dt  Step size [s].
     * @return New stage-2 temperature [°C]. */
    double step(double u, double dt);

    /**@brief Current internal model time [s]. */
    double time() const { return m_time; }

    /**@brief Exact model-inverse feedforward for the SOPDT cascade.
     *
     * Inverts both stages analytically: stage 2 tracking r requires
     * stage 1 to follow r + τ2·ṙ; stage-1 inverse then gives u.
     *
     * @param tRef    Previewed reference temperature [°C].
     * @param tRefDot Previewed reference rate [°C/s].
     * @param tRefAcc Previewed reference acceleration [°C/s²]
     *                (0 on ramp/soak profiles).
     * @param ambient Ambient temperature [°C].
     * @param disturb Disturbance estimate [°C/s].
     * @return Required actuator command (unclamped).
     */
    double requiredInput(double tRef, double tRefDot, double tRefAcc,
                         double ambient, double disturb) const;

    /**@brief Same as requiredInput but clamped to [uMin, uMax]. */
    double requiredInputClamped(double tRef, double tRefDot, double tRefAcc,
                                double ambient, double disturb) const;

    /**@brief FOPDT-equivalent required input (dominant-lag approximation):
     *        ignores the τ2 compensation — simpler, slightly more lag. */
    double requiredInputDominantLag(double tRef, double tRefDot,
                                    double ambient, double disturb) const;

    /**@brief Convert to FOPDT form {K, tau, L} for SIMC-equivalent tuning. */
    std::pair<double, double> gainAndTau() const {
        const auto [ae, be] = m_params.effectiveAlphaBeta();
        return {be > 1e-9 ? ae / be : 1e9, be > 1e-9 ? 1.0 / be : 1e9};
    }

    /**@brief Simulate open-loop for `steps` steps at constant input.
     * @return Temperature trajectory of stage 2 (steps+1 samples). */
    std::vector<double> simulate(double uConst, double dt, int steps,
                                 double tAmb, double d) const;

    /**@brief Reset simulated state to ambient (keeps clock at 0). */
    void reset() {
        m_T1 = m_params.ambient;
        m_T2 = m_params.ambient;
        m_delayLine.clear();
        m_time = 0.0;
    }

private:
    struct DelayLine {
        std::deque<std::pair<double, double>> buf;
        double uInit{0.0};
        void clear() { buf.clear(); }
        void push(double t, double u) {
            buf.emplace_back(t, u);
            while (buf.size() > 8192) buf.pop_front();
        }
        double value(double tau) const {
            if (buf.empty()) return uInit;
            if (tau < buf.front().first) return uInit;
            double result = buf.front().second;
            for (const auto& [t, u] : buf) {
                if (t <= tau) result = u;
                else break;
            }
            return result;
        }
    };

    OvenModelParamsSOPDT m_params;
    double m_T1{25.0};
    double m_T2{25.0};
    double m_time{0.0};
    DelayLine m_delayLine;
};

} // namespace tether::control::thermal
