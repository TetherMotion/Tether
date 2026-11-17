/**
 * @file SOPDTParameterEstimator.hpp
 * @brief Bounded online estimator for the cascade SOPDT oven model.
 *
 * @details
 * Estimates {α, β, τ2, d} of the two-state cascade model
 *
 *   dT1/dt = α·u(t−θ) − β·(T1 − T_amb) + d
 *   dT2/dt = (T1 − T2) / τ2
 *   y      = T2
 *
 * whose input→output transfer function is the SOPDT
 * K·e^(−Ls)/((τ1s+1)(τ2s+1)) with K = α/β, τ1 = 1/β.
 *
 * Method: trust-region output-error (OE) fitting. Every `fitInterval`
 * seconds the estimator re-simulates the exact cascade (detail::CascadeZoh,
 * coefficient-cached) over a sliding history window for a grid of
 * candidate stage-1/stage-2 lags {τ1, τ2}. Because the cascade is
 * linear in v = α·u_d + d, the gain α and disturbance d are solved
 * in closed form (2×2 ridge-regularized least squares toward the
 * prior) for each lag candidate — only the two lags need a nonlinear
 * search. A pattern-search refinement then polishes the best grid
 * point inside a multiplicative trust region around the prior.
 *
 * Why OE rather than ARX: per-sample ARX regressors are dominated by
 * sensor noise on slow thermal plants (errors-in-variables bias; the
 * two discrete poles are nearly coincident at control rates), while
 * OE compares simulated output to measurement — consistent under
 * feedback and far less noise-sensitive. The lag search is confined
 * to a trust region so the fit is a bounded refinement, not a
 * global re-identification.
 *
 * Robustness features mirror ThermalParameterEstimator: physical-box
 * projection, excitation gating (windows without input or temperature
 * variation are skipped — parameters are unidentifiable during steady
 * soaks), EMA smoothing, and graceful fallback to the prior on
 * degenerate fits.
 *
 * Caveats:
 *  - θ is NOT estimated — dead time comes from commissioning autotune,
 *    same as the FOPDT estimator.
 *  - The disturbance d̂ conflates true disturbance with ambient-model
 *    error; treat as approximate.
 *  - The two lags are nearly symmetric in the input→output transfer;
 *    the trust region keeps the fitted labeling close to the prior
 *    (stage-1 lag ↔ β, stage-2 lag ↔ τ2).
 *  - Needs richer excitation than the FOPDT integral estimator —
 *    dither or setpoint changes. During steady soaks it correctly
 *    refuses to update.
 */

#pragma once

#include "tether/control/thermal/OvenSOPDTModel.hpp"

#include <deque>

namespace tether::control::thermal {

struct SOPDTEstimatorConfig {
    double windowTime{300.0};    /**< Minimum scored window length [s]. The
                                      effective window is
                                      max(windowTime, windowLagFactor·(τ1+τ2))
                                      — the two-lag split is only identifiable
                                      when the scored window spans several
                                      slowest-lag time constants, so for slow
                                      ovens this floor dominates. */
    double windowLagFactor{4.0}; /**< Scored-window floor as a multiple of the
                                      current lag sum τ1+τ2 [—]. ~4 gives the
                                      OE fit enough curvature to separate the
                                      two lags; smaller values under-determine
                                      the split and the fit drifts. */
    double spinupFactor{4.0};    /**< Extra history = spinupFactor·(τ1+τ2)
                                      before the scored window, so the
                                      assumed initial state decays. */
    double fitInterval{5.0};     /**< Seconds between OE fits [s]. */
    double simStride{4.0};       /**< Fit simulation decimation factor:
                                      the OE sim runs at dt·simStride
                                      (≈2 s at 0.5 s control rate) — ample
                                      resolution for τ ~ 100 s lags and
                                      ~4× cheaper. */
    int gridSize{13};            /**< Lag candidates per dimension
                                      (gridSize² evaluations per fit). */
    double lagSpanLo{0.35};      /**< Trust region: lower multiplier on
                                      prior lags for the search. */
    double lagSpanHi{2.8};       /**< Trust region: upper multiplier on
                                      prior lags for the search. */
    double wLag{0.15};           /**< Relative penalty on log-lag distance
                                      from prior (keeps fit local). */
    double wAD{0.05};            /**< Relative ridge weight pulling the
                                      linear {α,d} solve toward prior. */
    double smoothing{0.3};       /**< EMA on accepted parameter update */
    double uExcursionMin{0.05};  /**< Min input range to count as excited */
    double tempSpanMin{2.0};     /**< Min temperature span [°C] (alt.) */
    double noiseFloor{0.2};      /**< Min span below which data is noise */
    size_t maxSamples{8192};     /**< History ring buffer cap (RT safety) */

    // Physical bounds on the mapped parameters
    double alphaMin{1e-4}, alphaMax{5.0};   // [°C/s per unit u]
    double betaMin{1e-5},  betaMax{0.5};    // [1/s]
    double tau2Min{0.5},   tau2Max{5000.0}; // [s]
    double KMin{1e-3},     KMax{1e5};       // [°C/u] DC gain
    double dMin{-2.0},     dMax{2.0};       // [°C/s]
};

class SOPDTParameterEstimator {
public:
    SOPDTParameterEstimator() = default;
    explicit SOPDTParameterEstimator(const SOPDTEstimatorConfig& cfg)
        : m_cfg(cfg) {}

    void setConfig(const SOPDTEstimatorConfig& cfg);
    const SOPDTEstimatorConfig& config() const { return m_cfg; }

    /**@brief (Re)set the prior estimate — call after commissioning or a
     *        manual model change. Clears the history. */
    void setPrior(double alpha, double beta, double tau2,
                  double disturbance = 0.0);

    /**@brief Feed one control cycle.
     * @param uDelayed  Applied input delayed by the model dead time.
     * @param tempMeas  Measured temperature [°C] (filtered preferred —
     *                  OE is noise-tolerant but filtering helps).
     * @param ambient   Ambient temperature [°C].
     * @param dt        Step size [s] — assumed uniform. */
    void update(double uDelayed, double tempMeas, double ambient, double dt);

    // ---- Estimates (mapped cascade parameters) ----
    double alpha() const { return m_alpha; }
    double beta() const { return m_beta; }
    double tau2() const { return m_tau2; }
    double disturbance() const { return m_d; }
    double tau1() const { return m_beta > 1e-9 ? 1.0 / m_beta : 1e9; }
    double gain() const { return m_beta > 1e-9 ? m_alpha / m_beta : 1e9; }

    // ---- Diagnostics ----
    int updatesUsed() const { return m_updatesUsed; }
    int cyclesSeen() const { return m_cycles; }
    bool lastUpdateExcited() const { return m_lastExcited; }
    bool lastUpdateValid() const { return m_lastValid; }
    /**@brief RMS output-error of the last accepted fit [°C]. */
    double lastResidual() const { return m_lastResidual; }

    void reset();

    /**@brief Debug/test hook: evaluate the OE objective (raw MSE, not the
     *        penalized J) at arbitrary stage lags over the current fit
     *        window. Returns false if the window is not ready. */
    bool debugEval(double ta, double tb, double& mse,
                   double& alpha, double& d) const;

private:
    struct Sample {
        double t;
        double u;    // delayed input u_d
        double T;    // measured temperature [°C]
    };

    /**@brief One OE objective evaluation for candidate lags (ta,tb):
     *        simulates the cascade over hist[0..n) and solves {α,d}
     *        linearly on hist[iscore..n). Returns penalized objective;
     *        outputs the linear params and raw MSE via refs. */
    double evalCandidate(double ta, double tb, double dtFit,
                         const std::vector<Sample>& hist, size_t iscore,
                         double ambient, double& alphaOut, double& dOut,
                         double& mseOut) const;

    void runFit();

    SOPDTEstimatorConfig m_cfg;

    // Mapped parameter estimates (EMA-smoothed)
    double m_alpha{0.02}, m_beta{0.005}, m_tau2{60.0}, m_d{0.0};

    std::deque<Sample> m_hist;
    double m_time{0.0};
    double m_lastFitTime{-1e18};
    double m_dt{1.0};        // last control period [s]
    double m_strideAcc{0.0}; // fractional decimation accumulator
    double m_ambient{25.0};  // last ambient [°C]

    int m_cycles{0};
    int m_updatesUsed{0};
    bool m_lastExcited{false};
    bool m_lastValid{false};
    double m_lastResidual{0.0};
};

} // namespace tether::control::thermal
