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

} // namespace tether::control::thermal
