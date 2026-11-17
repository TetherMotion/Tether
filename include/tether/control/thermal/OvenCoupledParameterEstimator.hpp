/**
 * @file OvenCoupledParameterEstimator.hpp
 * @brief Bounded online estimator for the coupled multi-zone oven model.
 *
 * @details
 * `ThermalParameterEstimator` identifies {α, β, d} of a *single* FOPDT
 * zone; `SOPDTParameterEstimator` identifies a two-lag cascade. Neither
 * can identify the *coupled* MIMO plant that `OvenGradientLQGController`
 * regulates:
 *
 *   dT_i/dt = α_i·u_i(t-θ_i) - β_i·(T_i - T_amb_i)
 *             + γL_i·(T_{i-1} - T_i) + γR_i·(T_{i+1} - T_i) + d_i
 *
 * This estimator identifies the per-zone physical parameters
 * {α_i, β_i, γL_i, γR_i, d_i} from closed-loop I/O of ALL zones. Written
 * in deviation form, every parameter is an *independent linear
 * coefficient*, so — exactly like the single-zone integral estimator —
 * each control cycle yields one noise-robust window-mean equation per
 * zone:
 *
 *   yRate_i = (T_i,back - T_i,front)/span
 *   phi_i   = [ mean(u_i,d),  -(mean(T_i)-T_amb_i),
 *               mean(T_{i-1}-T_i),  mean(T_{i+1}-T_i),  1 ]
 *   theta_i = [ α_i, β_i, γL_i, γR_i, d_i ]
 *
 * Boundary zones have only the inward neighbor, so the absent coupling
 * regressor is structurally zero and that coefficient stays at its
 * (zero) prior. For n = 1 the estimator reduces exactly to the FOPDT
 * {α, β, d} regression.
 *
 * Robustness features mirror `ThermalParameterEstimator`:
 *  - Recursive least squares over window-mean equations (integral form
 *    avoids differentiating noisy temperatures), one fresh equation per
 *    cycle.
 *  - Time-based forgetting (`memTime`) so old data ages out.
 *  - Tikhonov regularization toward the current estimate.
 *  - Per-zone excitation gating: a zone updates only when its own
 *    temperature moves AND (its input moves OR its temperature span is
 *    large OR a neighbor-delta moves — needed to identify coupling).
 *    During a steady soak all parameters are unidentifiable.
 *  - Projection onto a physical box (α, β, γ ≥ 0, d bounded).
 *  - EMA smoothing of accepted updates.
 *
 * Identifiability caveat: coupling γL/γR is only identifiable when the
 * inter-zone *difference* T_{i-1} − T_i actually varies — i.e. when the
 * spatial gradient changes. If all zones move together the coupling
 * coefficients are collinear with β and stay near their prior (bounded,
 * never diverging). Gradient profiles / setpoint offsets provide the
 * needed excitation.
 */

#pragma once

#include "tether/control/ControllerBase.hpp"

#include <Eigen/Dense>

#include <array>
#include <deque>
#include <span>
#include <vector>

namespace tether::control::thermal {

/**@brief Tunables for the coupled MIMO parameter estimator. */
struct CoupledEstimatorConfig {
    double windowTime{30.0};      /**< Regression window length [s] */
    double memTime{600.0};        /**< Forgetting time constant [s] */
    double smoothing{0.5};        /**< EMA on accepted parameter update */
    double uExcursionMin{0.05};   /**< Min input range to count as excited */
    double tempSpanMin{2.0};      /**< Min temperature span [°C] (alt.) */
    double noiseFloor{0.2};       /**< Min span below which data is noise */
    double ridge{1e-3};           /**< Tikhonov regularization toward prior */
    size_t maxSamples{8192};      /**< Ring buffer cap (RT safety) */

    // Physical bounds (shared across zones; per-zone prior sets the start)
    double alphaMin{1e-4}, alphaMax{5.0};        // [°C/s per unit u]
    double betaMin{1e-5},  betaMax{0.5};         // [1/s]
    double couplingMin{0.0}, couplingMax{0.2};   // [1/s]  (>=0 dissipative)
    double dMin{-2.0},     dMax{2.0};            // [°C/s]
};

/**@brief Per-zone prior / physical parameter set for the coupled model. */
struct CoupledZoneParams {
    double alpha{0.02};          /**< Heating coefficient [°C/s/u] */
    double beta{0.005};          /**< Loss coefficient [1/s] */
    double couplingLeft{0.0};    /**< Left-neighbor coupling rate [1/s] */
    double couplingRight{0.0};   /**< Right-neighbor coupling rate [1/s] */
    double disturbance{0.0};     /**< Additive disturbance [°C/s] */
    double ambient{25.0};        /**< Ambient temperature [°C] */
};

/**@brief Estimated per-zone physical parameters (coupling = 0 at edges). */
struct CoupledZoneEstimate {
    double alpha{0.0};
    double beta{0.0};
    double couplingLeft{0.0};
    double couplingRight{0.0};
    double disturbance{0.0};
};

class OvenCoupledParameterEstimator {
public:
    OvenCoupledParameterEstimator() = default;
    explicit OvenCoupledParameterEstimator(const CoupledEstimatorConfig& cfg)
        : m_cfg(cfg) {}

    void setConfig(const CoupledEstimatorConfig& cfg);
    const CoupledEstimatorConfig& config() const { return m_cfg; }

    /**@brief (Re)set the zone count and per-zone priors — call after
     *        commissioning or a manual model change. Resets sufficient
     *        statistics and the sliding window. `couplingLeft` of zone 0
     *        and `couplingRight` of the last zone are ignored (no neighbor)
     *        and pinned to zero, matching the plant assembly. */
    void setZones(const std::vector<CoupledZoneParams>& priors);

    /**@brief Feed one control cycle.
     * @param uDelayed  Per-zone applied input delayed by that zone's dead
     *                  time [unit u], spatial order, size == zoneCount().
     * @param measured  Measured zone temperatures [°C], size == zoneCount().
     * @param ambient   Per-zone ambient temperatures [°C], size == zoneCount().
     * @param dt        Step size [s]. */
    void update(std::span<const double> uDelayed,
                std::span<const double> measured,
                std::span<const double> ambient,
                double dt);

    // ---- Estimates ----
    size_t zoneCount() const { return m_zones.size(); }
    CoupledZoneEstimate estimate(size_t zone) const;

    // ---- Diagnostics ----
    int updatesUsed(size_t zone) const;
    int cyclesSeen() const { return m_cycles; }
    bool lastExcited(size_t zone) const;
    double lastResidual(size_t zone) const;

    void reset();

private:
    // Regression vector layout: [alpha, beta, couplingLeft, couplingRight, d]
    static constexpr int kP = 5;
    using Vec = Eigen::Matrix<double, kP, 1>;
    using Mat = Eigen::Matrix<double, kP, kP>;

    struct ZoneEst {
        Vec theta{Vec::Zero()};   // current estimate [alpha,beta,cl,cr,d]
        Mat S{Mat::Zero()};       // sufficient statistic sum phi*phi'
        Vec s{Vec::Zero()};       // sufficient statistic sum phi*yRate
        int updatesUsed{0};
        bool lastExcited{false};
        double lastResidual{0.0};
        bool hasLeft{false};
        bool hasRight{false};
    };

    struct Sample {
        double t;
        std::array<double, MAX_CONTROL_DIM> u;  // delayed inputs
        std::array<double, MAX_CONTROL_DIM> T;  // measured temps
    };

    CoupledEstimatorConfig m_cfg;
    int m_n{0};
    std::vector<ZoneEst> m_zones;
    std::deque<Sample> m_win;
    double m_time{0.0};
    int m_cycles{0};
};

} // namespace tether::control::thermal
