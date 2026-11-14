/**
 * @file ThermalParameterEstimator.hpp
 * @brief Bounded online estimator for the physical oven model parameters.
 *
 * @details
 * Estimates {α, β, d} of the lumped thermal model
 *
 *   dT/dt = α·u(t-θ) - β·(T - T_amb) + d
 *
 * from closed-loop I/O data using a sliding-window integral regression.
 * Each control cycle produces one equation by integrating the ODE over
 * the trailing `windowTime` seconds:
 *
 *   ΔT/Tw = α·mean(u_d) - β·mean(T - T_amb) + d
 *
 * Integral (rather than differential) form avoids differentiating the
 * noisy temperature signal. Because a fresh equation is produced every
 * cycle — not once per non-overlapping window — the estimator gets ~40x
 * more data and tracks load changes within seconds rather than minutes.
 *
 * Robustness features:
 *  - Parameter projection onto a physical box (α, β, d bounds).
 *  - Excitation gating: cycles whose window lacks input or temperature
 *    variation are skipped — during a steady soak the parameters are
 *    unidentifiable and updating would only fit noise.
 *  - Forgetting with time constant `memTime` so old data ages out.
 *  - Tikhonov regularization toward the current estimate.
 *  - EMA smoothing of accepted updates.
 *
 * Identifiability caveat: in closed loop u ≈ f(T-Tamb), so {α, β, d} are
 * only jointly identifiable when the input decorrelates from temperature
 * — profile transitions help, and OvenController adds a small PE dither
 * while adaptation is enabled. The estimator stays bounded regardless.
 */

#pragma once

#include "tether/control/thermal/OvenThermalModel.hpp"

#include <Eigen/Dense>

#include <deque>

namespace tether::control::thermal {

struct EstimatorConfig {
    double windowTime{15.0};      /**< Regression window length [s] */
    double memTime{300.0};        /**< Forgetting time constant [s] */
    double smoothing{0.5};        /**< EMA on accepted parameter update */
    double uExcursionMin{0.05};   /**< Min input range to count as excited */
    double tempSpanMin{2.0};      /**< Min temperature span [°C] (alt.) */
    double noiseFloor{0.2};       /**< Min span below which data is noise */
    double ridge{1e-3};           /**< Tikhonov regularization toward prior */
    size_t maxSamples{4096};      /**< Ring buffer cap (RT safety) */

    // Physical bounds for (α, β, d)
    double alphaMin{1e-4}, alphaMax{5.0};   // [°C/s per unit u]
    double betaMin{1e-5},  betaMax{0.5};    // [1/s]
    double dMin{-2.0},     dMax{2.0};       // [°C/s]
};

class ThermalParameterEstimator {
public:
    ThermalParameterEstimator() = default;
    explicit ThermalParameterEstimator(const EstimatorConfig& cfg) : m_cfg(cfg) {}

    void setConfig(const EstimatorConfig& cfg);
    const EstimatorConfig& config() const { return m_cfg; }

    /**@brief (Re)set the prior estimate — call after commissioning or a
     *        manual model change. Resets sufficient statistics and the
     *        sliding window. */
    void setPrior(double alpha, double beta, double disturbance = 0.0);

    /**@brief Feed one control cycle.
     * @param uDelayed  Applied input delayed by the model dead time [unit u].
     * @param tempFilt  Noise-filtered measured temperature [°C].
     * @param ambient   Ambient temperature [°C].
     * @param dt        Step size [s]. */
    void update(double uDelayed, double tempFilt, double ambient, double dt);

    // ---- Estimates ----
    double alpha() const { return m_theta.x(); }
    double beta() const { return m_theta.y(); }
    double disturbance() const { return m_theta.z(); }
    double tau() const { return m_theta.y() > 1e-9 ? 1.0 / m_theta.y() : 1e9; }
    double gain() const {
        return m_theta.y() > 1e-9 ? m_theta.x() / m_theta.y() : 1e9;
    }
    Eigen::Vector3d theta() const { return m_theta; }

    // ---- Diagnostics ----
    int updatesUsed() const { return m_updatesUsed; }   /**< Excited cycles */
    int cyclesSeen() const { return m_cycles; }
    bool lastUpdateExcited() const { return m_lastExcited; }
    double lastResidual() const { return m_lastResidual; }

    void reset();

private:
    struct Sample {
        double t;    // estimator clock [s]
        double u;    // delayed input
        double T;    // filtered temperature
    };

    EstimatorConfig m_cfg;
    Eigen::Vector3d m_theta{0.02, 0.005, 0.0};
    Eigen::Matrix3d m_S{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d m_s{Eigen::Vector3d::Zero()};

    std::deque<Sample> m_win;
    double m_time{0.0};

    int m_cycles{0};
    int m_updatesUsed{0};
    bool m_lastExcited{false};
    double m_lastResidual{0.0};
};

} // namespace tether::control::thermal
