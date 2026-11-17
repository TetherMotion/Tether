/**
 * @file OvenAutotune.hpp
 * @brief Commissioning autotune for the oven thermal loop.
 *
 * @details
 * Consumes a recorded step response (heater 0 → uStep at t=0, temperature
 * sampled at uniform dt) and produces:
 *
 *  - A FOPDT estimate {K, τ, θ} via the two-point (10%/90%) method.
 *  - Physical OvenModelParams {α, β, θ} = {K/τ, 1/τ, θ}.
 *  - SIMC PI gains derived from those physical parameters.
 *
 * Everything is self-contained in the thermal module — no dependency on
 * the autotuning component — so `tether_controls` stays standalone.
 */

#pragma once

#include "tether/control/thermal/OvenSOPDTModel.hpp"
#include "tether/control/thermal/OvenThermalModel.hpp"

#include <utility>
#include <vector>

namespace tether::control::thermal {

/**@brief Recorded step-response data for commissioning. */
struct StepResponseData {
    std::vector<double> time;   /**< Sample times [s], strictly increasing */
    std::vector<double> temp;   /**< Measured temperature [°C] */
    double uStep{0.5};          /**< Actuator step applied at time[0] [unit u] */
    double tAmb{25.0};          /**< Ambient temperature during test [°C] */
    double uMin{0.0};
    double uMax{1.0};
};

/**@brief FOPDT estimate: K [°C/u], tau [s], theta [s]. */
struct FopdtEstimate {
    double K{0.0};
    double tau{0.0};
    double theta{0.0};
    bool valid{false};
};

/**@brief Two-point (10%/90%) FOPDT identification.
 * @param data   Step response recording.
 * @param minDeltaT  Minimum steady-state temperature rise to trust [°C].
 * @return Estimate; valid=false if the response is too small or degenerate. */
FopdtEstimate identifyFopdt(const StepResponseData& data,
                            double minDeltaT = 1.0);

/**@brief SIMC PI tuning for a FOPDT process.
 * @param K      Process gain [°C/u].
 * @param tau    Time constant [s].
 * @param theta  Dead time [s].
 * @param tauC   Closed-loop time constant [s]; <=0 selects the SIMC
 *               default tauC = theta ("tight" tuning).
 * @return {Kp, Ki} in parallel form (Kd = 0).
 *
 * Rules (Skogestad): Kp = (1/K)·τ/(τc+θ), Ti = min(τ, 4(τc+θ)), Ki = Kp/Ti. */
std::pair<double, double> simcPiGains(double K, double tau, double theta,
                                      double tauC = 0.0);

/**@brief Full commissioning result. */
struct OvenAutotuneResult {
    bool success{false};
    FopdtEstimate model;
    OvenModelParams params;   /**< Physical model parameters */
    double kp{0.0};           /**< PI proportional gain */
    double ki{0.0};           /**< PI integral gain */
    double lambda{0.0};       /**< Closed-loop time constant used [s] */
};

/**@brief Identify FOPDT from a step response and produce model params +
 *        SIMC PI gains ready for OvenController. */
OvenAutotuneResult autoTuneOven(const StepResponseData& data,
                                double tauC = 0.0,
                                double minDeltaT = 1.0);

// ---------------------------------------------------------------------------
// SOPDT commissioning
// ---------------------------------------------------------------------------

/**@brief Tuning method selectable by the user for both autotune and the
 *        controller's online retune path. */
enum class OvenTuneMethod {
    Simc,    /**< SIMC rules. FOPDT→PI; SOPDT→PID with Td = τ_secondary. */
    ImcPi,   /**< IMC PI: Kp = Στ/(K(λ+θ)), Ti = Στ. */
    ImcPid,  /**< IMC PID: IMC-PI + Td = τ1·τ2/Στ. */
    Manual   /**< Use explicitly configured gains; no automatic tuning. */
};

/**@brief SOPDT estimate: K [°C/u], tau1 >= tau2 [s], theta [s].
 *        tau1 is the DOMINANT (slower) lag, tau2 the secondary. */
struct SopdtEstimate {
    double K{0.0};
    double tau1{0.0};   // dominant (slow) lag
    double tau2{0.0};   // secondary (fast) lag
    double theta{0.0};
    double fitRmse{0.0}; /**< Normalized step-response fit RMSE [0..1] */
    bool valid{false};
};

/**@brief PID gains in parallel form (ki = kp/Ti, kd = kp·Td). */
struct SopdtGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

/**@brief SOPDT identification from a step response.
 *
 *  Method: model-free dead-time detection + shape fit. The delay θ is
 *  estimated from the first detectable departure of the response from
 *  baseline; then (τ1, τ2) are found by grid search + pattern refinement
 *  minimizing the SSE between the analytic SOPDT unit-step response and
 *  the normalized measured rise. K comes from the steady-state gain.
 *
 *  The fit is over the normalized response, so it is insensitive to
 *  sensor gain errors; K still comes from the measured ΔT/uStep.
 *
 *  @param data       Step response recording (same as identifyFopdt).
 *  @param minDeltaT  Minimum steady-state rise to trust [°C].
 *  @return Estimate; valid=false if the response is too small or
 *          degenerate. fitRmse near 0 = excellent SOPDT fit; values
 *          above ~0.05 suggest the plant is not well described by
 *          SOPDT (higher-order dynamics, nonlinear loss, etc.). */
SopdtEstimate identifySopdt(const StepResponseData& data,
                            double minDeltaT = 1.0);

/**@brief SIMC tuning for SOPDT (Skogestad SOPDT rule).
 * @param K       DC gain [°C/u].
 * @param tauDom  Dominant (slow) time constant [s].
 * @param tauSec  Secondary (fast) time constant [s].
 * @param theta   Dead time [s].
 * @param tauC    Closed-loop time constant [s]; <=0 → tauC = theta.
 * @return {Kp, Ki, Kd}: Kp = tauDom/(K(τc+θ)), Ti = min(tauDom, 4(τc+θ)),
 *         Td = tauSec. */
SopdtGains simcGainsSopdt(double K, double tauDom, double tauSec,
                          double theta, double tauC = 0.0);

/**@brief IMC PI tuning for SOPDT.
 * @param lambda  IMC filter time constant [s]; <=0 → lambda = theta
 *                (or 0.1·Στ when θ≈0), matching the SIMC default.
 * @return Kp = (τ1+τ2)/(K(λ+θ)), Ti = τ1+τ2, Kd = 0. */
SopdtGains imcPiGainsSopdt(double K, double tauDom, double tauSec,
                           double theta, double lambda = 0.0);

/**@brief IMC PID tuning for SOPDT.
 * @return IMC-PI gains + Td = τ1·τ2/(τ1+τ2). */
SopdtGains imcPidGainsSopdt(double K, double tauDom, double tauSec,
                            double theta, double lambda = 0.0);

/**@brief Full SOPDT commissioning result. */
struct OvenAutotuneResultSOPDT {
    bool success{false};
    SopdtEstimate model;
    OvenModelParamsSOPDT params;  /**< Cascade physical parameters */
    double kp{0.0}, ki{0.0}, kd{0.0};
    double lambda{0.0};           /**< Closed-loop time constant used [s] */
};

/**@brief Identify SOPDT from a step response and produce cascade model
 *        params + gains ready for OvenController.
 *
 *  Pole→stage mapping convention (same as SOPDTParameterEstimator):
 *  the FASTER pole becomes stage 1 (β = 1/τfast, α = K·β) and the
 *  SLOWER pole becomes τ2 — matching the physical picture of a fast
 *  lossy air/heater mass driving a slower product mass. τ2→0 recovers
 *  the FOPDT model.
 *
 *  @param method  Tuning method for the returned gains (Manual → gains
 *                 are zero; set them explicitly). */
OvenAutotuneResultSOPDT autoTuneOvenSOPDT(const StepResponseData& data,
                                          OvenTuneMethod method =
                                              OvenTuneMethod::Simc,
                                          double tauC = 0.0,
                                          double minDeltaT = 1.0);

} // namespace tether::control::thermal
