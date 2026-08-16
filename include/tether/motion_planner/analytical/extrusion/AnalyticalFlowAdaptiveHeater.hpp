/**
 * @file AnalyticalFlowAdaptiveHeater.hpp
 * @brief Analytical flow-adaptive heater controller on WSS arcs.
 *
 * @details
 * The feed-forward power is:
 *
 *   P_ff(t) = ρ·c_p·Q(t)·(T_target - T_inlet)
 *
 * where Q(t) = α_e · A_f · v(t) is piecewise polynomial from the WSS.
 * Therefore P_ff(t) is piecewise polynomial of the same degree as v(t).
 *
 * Pre-emphasis (at flow onset): The steady-state enthalpy power is applied
 * before the melt zone cools.  The onset time t_onset is the first time
 * Q(t) crosses a threshold.  The pre-emphasis is a piecewise constant
 * function:
 *
 *   P_pre(t) = (1-α)·P_ff(t_onset) · 1_{[t_onset, t_onset+τ_pre]}(t)
 *
 * Post-emphasis (after flow stops): The thermal debt D(t) relaxes
 * exponentially:
 *
 *   D(t) = D_0·exp(-(t-t_stop)/τ_debt)
 *
 *   P_post(t) = (1-α)·D(t) · 1_{[t_stop, ∞)}(t)
 *
 * The total analytical feed-forward is P_ff(t) + P_pre(t) + P_post(t).
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §8
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"
#include "AnalyticalMeltZoneThermalObserver.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for the analytical flow-adaptive heater controller.
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
 * @brief Analytical flow-adaptive heater controller.
 *
 * Computes the feed-forward power P_ff(t) + P_pre(t) + P_post(t) in
 * closed form from the WSS piecewise-polynomial flow Q(t).
 */
template<size_t Dim, typename T = double>
class AnalyticalFlowAdaptiveHeater {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;
    using ThermalObs = AnalyticalMeltZoneThermalObserver<Dim, T>;

    /**
     * @brief Construct from trajectory and parameters.
     */
    AnalyticalFlowAdaptiveHeater(const Traj& traj,
                                  AnalyticalFlowAdaptiveHeaterParams params)
        : traj_(&traj), params_(params) {
        filamentAreaMm2_ = M_PI * params_.filamentDiameterMm
                           * params_.filamentDiameterMm / 4.0;
        detectFlowTransitions();
    }

    /**
     * @brief Compute the feed-forward power at time t [PWM fraction, 0-1].
     *
     * P_total(t) = P_ff(t) + P_pre(t) + P_post(t)
     */
    double feedforwardAtTime(double t) const {
        double pff = steadyStateFeedforward(t);
        double ppre = preEmphasis(t);
        double ppost = postEmphasis(t);
        double total = pff + ppre + ppost;

        // Clamp to [0, 1]
        return std::clamp(total, 0.0, 1.0);
    }

    /**
     * @brief Compute the steady-state enthalpy feed-forward P_ff(t) [PWM].
     *
     * P_ff(t) = ρ·c_p·Q(t)·(T_target - T_inlet) / heaterPowerScale
     */
    double steadyStateFeedforward(double t) const {
        double vE = traj_->extruderVelocityAtTime(t);
        double Q = vE * filamentAreaMm2_;
        double deltaT = params_.targetTempC - params_.inletTempC;
        double powerW = params_.filamentHeatCapacity * Q * deltaT;
        return powerW / params_.heaterPowerScale;
    }

    /**
     * @brief Compute the pre-emphasis power at time t [PWM].
     */
    double preEmphasis(double t) const {
        if (!hasFlowOnset_ || t < flowOnsetTime_
            || t >= flowOnsetTime_ + params_.preEmphasisDuration)
            return 0.0;

        // P_pre = (1-α)·P_ff(t_onset), bounded by maxPreEmphasisPower
        double alpha = sensorCouplingAlpha();
        double pffOnset = steadyStateFeedforward(flowOnsetTime_);
        double ppre = (1.0 - alpha) * pffOnset;
        return std::clamp(ppre, 0.0, params_.maxPreEmphasisPower);
    }

    /**
     * @brief Compute the post-emphasis power at time t [PWM].
     */
    double postEmphasis(double t) const {
        if (!hasFlowStop_ || t < flowStopTime_)
            return 0.0;

        // D(t) = D_0·exp(-(t-t_stop)/τ_debt)
        // D_0 = P_ff(t_stop⁻) (the enthalpy power just before flow stopped)
        double pffStop = steadyStateFeedforward(flowStopTime_ - 1e-6);
        double alpha = sensorCouplingAlpha();
        double D0 = pffStop;
        double dt = t - flowStopTime_;
        double D = D0 * std::exp(-dt / params_.debtTimeConstant);
        double ppost = (1.0 - alpha) * D;
        return std::clamp(ppost, 0.0, params_.maxPostEmphasisPower);
    }

    /**
     * @brief Compute the total heater output at time t [PWM].
     *
     * This is the feed-forward only.  In a real system, this would be
     * added to the PID output.
     */
    double heaterOutputAtTime(double t) const {
        return feedforwardAtTime(t);
    }

    /**
     * @brief Compute the feed-forward at multiple time points.
     */
    std::vector<double> feedforwardSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(feedforwardAtTime(t));
        return result;
    }

    /**
     * @brief Compute the flow rate Q(t) at time t [mm³/s].
     */
    double flowAtTime(double t) const {
        double vE = traj_->extruderVelocityAtTime(t);
        return vE * filamentAreaMm2_;
    }

    /// Flow onset time [s] (first time Q crosses threshold)
    double flowOnsetTime() const { return flowOnsetTime_; }
    bool hasFlowOnset() const { return hasFlowOnset_; }

    /// Flow stop time [s] (last time Q drops below threshold)
    double flowStopTime() const { return flowStopTime_; }
    bool hasFlowStop() const { return hasFlowStop_; }

    /// Sensor coupling factor α = G_sm / (G_hs + G_sm)
    double sensorCouplingAlpha() const {
        double Gsum = params_.heaterSensorConductance
                      + params_.sensorMeltConductance;
        return (Gsum > 0.0) ? params_.sensorMeltConductance / Gsum : 1.0;
    }

    /// Parameters
    const AnalyticalFlowAdaptiveHeaterParams& params() const { return params_; }

    /// Filament area [mm²]
    double filamentAreaMm2() const { return filamentAreaMm2_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalFlowAdaptiveHeaterParams params_;
    double filamentAreaMm2_ = 0.0;

    double flowOnsetTime_ = 0.0;
    bool hasFlowOnset_ = false;
    double flowStopTime_ = 0.0;
    bool hasFlowStop_ = false;

    void detectFlowTransitions() {
        const auto& arcs = traj_->arcs();
        double threshold = params_.flowThresholdMm3PerS;

        // Find first time Q crosses threshold (onset)
        for (const auto& a : arcs) {
            if (a.extrusionRatio <= 0.0) continue;
            // Check if Q crosses threshold within this arc
            // Q(τ) = α_e · A_f · v(τ) = α_e · A_f · (c0 + c1·τ + c2·τ²)
            // We sample a few points to detect crossing
            int nCheck = std::max(10, static_cast<int>(a.duration / 0.001));
            for (int i = 0; i <= nCheck; ++i) {
                double tau = a.duration * static_cast<double>(i) / nCheck;
                double Q = a.extruderVelocity(tau) * filamentAreaMm2_;
                if (Q > threshold) {
                    flowOnsetTime_ = a.t0 + tau;
                    hasFlowOnset_ = true;
                    break;
                }
            }
            if (hasFlowOnset_) break;
        }

        // Find last time Q drops below threshold (stop)
        double lastFlowTime = 0.0;
        for (const auto& a : arcs) {
            if (a.extrusionRatio <= 0.0) continue;
            int nCheck = std::max(10, static_cast<int>(a.duration / 0.001));
            for (int i = 0; i <= nCheck; ++i) {
                double tau = a.duration * static_cast<double>(i) / nCheck;
                double Q = a.extruderVelocity(tau) * filamentAreaMm2_;
                if (Q > threshold) {
                    lastFlowTime = a.t0 + tau;
                }
            }
        }
        if (lastFlowTime > 0.0) {
            flowStopTime_ = lastFlowTime;
            hasFlowStop_ = true;
        }
    }
};

} // namespace MotionPlanner::analytical::extrusion
