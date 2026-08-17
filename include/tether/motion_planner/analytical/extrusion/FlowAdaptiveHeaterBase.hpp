/**
 * @file FlowAdaptiveHeaterBase.hpp
 * @brief Common parameters and base class for flow-adaptive heater controllers.
 *
 * @details
 * Both the sampling and the analytical flow-adaptive heater controllers share
 * the same parameters and the same physical model.  They differ only in how
 * they determine the flow onset and flow stop times:
 *
 * - `SamplingFlowAdaptiveHeater` estimates them by dense point sampling.
 * - `AnalyticalFlowAdaptiveHeater` finds the first/last threshold crossings in
 *   closed form on the piecewise-polynomial flow Q(t).
 *
 * The output of both classes is a temperature delta [°C] to be added to the
 * extruder temperature target.  The delta is the transient thermal overshoot
 * (pre-emphasis + post-emphasis) divided by the effective heater→melt
 * conductance:
 *
 *   ΔT(t) = (P_pre(t) + P_post(t)) / G_eff
 *
 * where
 *   G_eff = (G_hs · G_sm) / (G_hs + G_sm)
 *   P_pre(t)  = (1 - α) · P_ff(t_onset)  for t ∈ [t_onset, t_onset + τ_pre]
 *   P_post(t) = (1 - α) · P_ff(t_stop) · exp(-(t - t_stop) / τ_debt)
 *               for t ≥ t_stop
 *   P_ff(t)   = ρ · c_p · Q(t) · (T_target - T_inlet)
 *   α         = G_sm / (G_hs + G_sm)
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §8
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for the flow-adaptive heater controller.
 */
struct AnalyticalFlowAdaptiveHeaterParams {
    // --- Filament and heater ---
    double filamentHeatCapacity = 2.1;   ///< ρ·c_p [J/(mm³·K)]
    double inletTempC           = 25.0;  ///< T_inlet [°C]
    double targetTempC          = 210.0; ///< T_target [°C]
    double heaterPowerScale     = 40.0;  ///< [W] at PWM=1

    // --- Three-state thermal model ---
    double heaterBlockCapacitance  = 8.0; ///< C_h [J/K]
    double sensorCapacitance       = 1.0; ///< C_s [J/K]
    double meltZoneCapacitance     = 2.0; ///< C_m [J/K]
    double heaterSensorConductance = 2.0; ///< G_hs [W/K]
    double sensorMeltConductance   = 1.5; ///< G_sm [W/K]

    // --- Feed-forward limits ---
    double debtTimeConstant      = 2.0;   ///< τ_debt [s]
    double maxPreEmphasisPower   = 0.4;   ///< [0-1 PWM]
    double maxPostEmphasisPower  = 0.2;   ///< [0-1 PWM]
    double maxHeaterOvershoot    = 10.0;  ///< [°C]

    // --- Pre-emphasis duration [s] ---
    double preEmphasisDuration   = 0.5;

    // --- Flow onset/stop threshold [mm³/s] ---
    double flowThresholdMm3PerS  = 0.5;

    // --- Filament diameter [mm] ---
    double filamentDiameterMm = 1.75;
};

/**
 * @brief Base class for sampling and analytical flow-adaptive heater
 *        controllers.
 *
 * @tparam Dim Path dimension
 * @tparam T   Scalar type
 * @tparam Derived CRTP derived class that supplies detectFlowTransitions()
 */
template<size_t Dim, typename T, typename Derived>
class FlowAdaptiveHeaterBase {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct from an extrusion trajectory and parameters.
     */
    FlowAdaptiveHeaterBase(const Traj& traj,
                           AnalyticalFlowAdaptiveHeaterParams params)
        : traj_(&traj), params_(params) {
        filamentAreaMm2_ = M_PI * params_.filamentDiameterMm
                         * params_.filamentDiameterMm / 4.0;
        static_cast<Derived*>(this)->detectFlowTransitions();
    }

    /**
     * @brief Compute the transient temperature delta at time t [°C].
     *
     * This is the temperature offset to add to the extruder target
     * temperature to compensate for flow transients.
     */
    double temperatureDeltaAtTime(double t) const {
        double P = excessPowerAtTime(t);
        double G = effectiveConductance();
        if (G <= 0.0 || P <= 0.0) return 0.0;
        double deltaT = P / G;
        return std::clamp(deltaT, 0.0, params_.maxHeaterOvershoot);
    }

    /**
     * @brief Compute the temperature delta at multiple time points.
     */
    std::vector<double> temperatureDeltaSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(temperatureDeltaAtTime(t));
        return result;
    }

    /// Flow onset time [s] (first time Q crosses threshold)
    double flowOnsetTime() const { return flowOnsetTime_; }
    bool hasFlowOnset() const { return hasFlowOnset_; }

    /// Flow stop time [s] (last time Q drops below threshold)
    double flowStopTime() const { return flowStopTime_; }
    bool hasFlowStop() const { return hasFlowStop_; }

    /// Parameters
    const AnalyticalFlowAdaptiveHeaterParams& params() const { return params_; }

    /// Filament area [mm²]
    double filamentAreaMm2() const { return filamentAreaMm2_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

protected:
    const Traj* traj_;
    AnalyticalFlowAdaptiveHeaterParams params_;
    double filamentAreaMm2_ = 0.0;

    double flowOnsetTime_ = 0.0;
    bool hasFlowOnset_ = false;
    double flowStopTime_ = 0.0;
    bool hasFlowStop_ = false;

    /// Steady-state enthalpy power required to maintain target at flow Q(t).
    double steadyStatePower(double t) const {
        double vE = traj_->extruderVelocityAtTime(t);
        double Q = vE * filamentAreaMm2_;
        double deltaT = params_.targetTempC - params_.inletTempC;
        return params_.filamentHeatCapacity * Q * deltaT;
    }

    /// Pre-emphasis transient power [W].
    double preEmphasisPower(double t) const {
        if (!hasFlowOnset_ || t < flowOnsetTime_
            || t >= flowOnsetTime_ + params_.preEmphasisDuration)
            return 0.0;

        double PffOnset = steadyStatePower(flowOnsetTime_);
        double Ppre = (1.0 - sensorCouplingAlpha()) * PffOnset;
        if (params_.heaterPowerScale > 0.0) {
            Ppre = std::clamp(Ppre, 0.0,
                              params_.maxPreEmphasisPower * params_.heaterPowerScale);
        } else {
            Ppre = std::clamp(Ppre, 0.0, 1.0);
        }
        return Ppre;
    }

    /// Post-emphasis transient power [W].
    double postEmphasisPower(double t) const {
        if (!hasFlowStop_ || t < flowStopTime_)
            return 0.0;

        double PffStop = steadyStatePower(flowStopTime_ - 1e-6);
        double D0 = PffStop;
        double dt = t - flowStopTime_;
        double D = D0 * std::exp(-dt / params_.debtTimeConstant);
        double Ppost = (1.0 - sensorCouplingAlpha()) * D;
        if (params_.heaterPowerScale > 0.0) {
            Ppost = std::clamp(Ppost, 0.0,
                               params_.maxPostEmphasisPower * params_.heaterPowerScale);
        } else {
            Ppost = std::clamp(Ppost, 0.0, 1.0);
        }
        return Ppost;
    }

    /// Total transient power (pre + post) [W].
    double excessPowerAtTime(double t) const {
        return preEmphasisPower(t) + postEmphasisPower(t);
    }

    /// Sensor coupling factor α = G_sm / (G_hs + G_sm).
    double sensorCouplingAlpha() const {
        double Gsum = params_.heaterSensorConductance
                    + params_.sensorMeltConductance;
        return (Gsum > 0.0) ? params_.sensorMeltConductance / Gsum : 1.0;
    }

    /// Effective series conductance from heater block to melt zone [W/K].
    double effectiveConductance() const {
        double Ghs = params_.heaterSensorConductance;
        double Gsm = params_.sensorMeltConductance;
        double Gsum = Ghs + Gsm;
        return (Gsum > 0.0) ? (Ghs * Gsm) / Gsum : 0.0;
    }
};

} // namespace MotionPlanner::analytical::extrusion
