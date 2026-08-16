/**
 * @file AnalyticalLinearPressureAdvance.hpp
 * @brief Analytical linear pressure advance on WSS arcs.
 *
 * @details
 * The classic Klipper pressure-advance law in continuous-time form:
 *
 *   δe(t) = PressureAdvance · v_e(t) = PressureAdvance · α_e · v(t)
 *
 * Since v(t) is piecewise polynomial from the WSS, δe(t) is also piecewise
 * polynomial of the same degree:
 *
 *   BANG arc:     δe(τ) = PressureAdvance·α_e·(v0 + a0·τ + ½·η·τ²)
 *   SINGULAR arc: δe(τ) = PressureAdvance·α_e·(v0 + a*·τ)
 *   WALL arc:     δe(τ) = PressureAdvance·α_e·v_wall
 *
 * The cumulative (integrated) offset is:
 *
 *   Δe(t) = ∫₀^t δe(t') dt'
 *
 * which is piecewise polynomial of degree one higher.
 *
 * With smoothing, the offset uses the continuous-time smoothed velocity:
 *
 *   δe(t) = PressureAdvance · v_e_smooth(t)
 *
 * where v_e_smooth(t) = (1/T_s) ∫_{t-T_s/2}^{t+T_s/2} v_e(τ) dτ
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §1
 * @see AnalyticalExtrusionTypes.hpp for the ExtrusionTrajectory.
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for analytical linear pressure advance.
 */
struct AnalyticalLinearPressureAdvanceParams {
    /// Classic linear PressureAdvance amount [s].
    double pressureAdvance = 0.0;

    /// Smoothing window [s] (0 = no smoothing).
    double smoothTime = 0.0;

    /// Maximum absolute compensation [mm] (safety clamp, 0 = no clamp).
    double maxCompensation = 0.0;
};

/**
 * @brief Analytical linear pressure advance.
 *
 * Computes the extruder position offset δe(t) = PressureAdvance · v_e(t) in closed form
 * from the WSS piecewise-polynomial velocity.
 *
 * Usage:
 *   AnalyticalLinearPressureAdvance<2> pressureAdvance(trajectory, params);
 *   double offset = pressureAdvance.offsetAtTime(t);
 *   double cumulative = pressureAdvance.integratedOffsetAtTime(t);
 */
template<size_t Dim, typename T = double>
class AnalyticalLinearPressureAdvance {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct from an extrusion trajectory and parameters.
     */
    AnalyticalLinearPressureAdvance(const Traj& traj, AnalyticalLinearPressureAdvanceParams params)
        : traj_(&traj), params_(params) {
        precomputeIntegratedOffsets();
    }

    /**
     * @brief Compute the instantaneous position offset δe(t) [mm].
     *
     * This is the pressure-advance offset added to the raw extruder
     * position at time t.
     */
    double offsetAtTime(double t) const {
        if (params_.pressureAdvance <= 0.0) return 0.0;

        double vE;
        if (params_.smoothTime > 0.0) {
            vE = smoothedExtruderVelocity(*traj_, t, params_.smoothTime);
        } else {
            vE = traj_->extruderVelocityAtTime(t);
        }

        double offset = params_.pressureAdvance * vE;
        return clampOffset(offset);
    }

    /**
     * @brief Compute the cumulative (integrated) offset Δe(t) [mm].
     *
     * Δe(t) = ∫₀^t δe(t') dt' = PressureAdvance · ∫₀^t v_e(t') dt'
     *
     * Without smoothing, this is PressureAdvance · (total extruded distance up to time t),
     * which is PressureAdvance · α_e · s(t) — a piecewise polynomial.
     *
     * With smoothing, the integral is computed numerically from the
     * precomputed offset samples.
     */
    double integratedOffsetAtTime(double t) const {
        if (params_.pressureAdvance <= 0.0) return 0.0;

        if (params_.smoothTime <= 0.0) {
            // Without smoothing: Δe(t) = PressureAdvance · α_e · s(t)
            // = PressureAdvance · extruderPositionAtTime(t)
            return params_.pressureAdvance * traj_->extruderPositionAtTime(t);
        }

        // With smoothing: use precomputed integrated offset table
        return integratedOffsetInterp(t);
    }

    /**
     * @brief Compute the offset at multiple time points.
     *
     * @param times Vector of query times [s]
     * @return Vector of offsets [mm]
     */
    std::vector<double> offsetSeries(const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(offsetAtTime(t));
        return result;
    }

    /**
     * @brief Compute the integrated offset at multiple time points.
     */
    std::vector<double> integratedOffsetSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(integratedOffsetAtTime(t));
        return result;
    }

    /**
     * @brief Compute the adjusted extruder position at time t.
     *
     * e_adjusted(t) = e_raw(t) + δe(t)
     */
    double adjustedExtruderPosition(double t) const {
        return traj_->extruderPositionAtTime(t) + offsetAtTime(t);
    }

    /**
     * @brief Compute the adjusted extruder position at multiple times.
     */
    std::vector<double> adjustedExtruderPositionSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(adjustedExtruderPosition(t));
        return result;
    }

    /// Parameters
    const AnalyticalLinearPressureAdvanceParams& params() const { return params_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalLinearPressureAdvanceParams params_;

    // Precomputed table for smoothed integrated offset
    std::vector<double> tableTimes_;
    std::vector<double> tableIntegratedOffsets_;

    double clampOffset(double offset) const {
        if (params_.maxCompensation > 0.0) {
            return std::clamp(offset, -params_.maxCompensation,
                              params_.maxCompensation);
        }
        return offset;
    }

    void precomputeIntegratedOffsets() {
        if (params_.smoothTime <= 0.0 || params_.pressureAdvance <= 0.0)
            return;

        // Sample the offset densely and compute cumulative integral
        double totalT = traj_->totalTime();
        if (totalT <= 0.0) return;

        int numSamples = std::max(100, static_cast<int>(totalT / 0.001));
        double dt = totalT / numSamples;

        tableTimes_.resize(numSamples + 1);
        tableIntegratedOffsets_.resize(numSamples + 1);

        double cumulative = 0.0;
        for (int i = 0; i <= numSamples; ++i) {
            double t = static_cast<double>(i) * dt;
            tableTimes_[i] = t;
            tableIntegratedOffsets_[i] = cumulative;
            // Trapezoidal integration of offsetAtTime
            if (i < numSamples) {
                double off0 = offsetAtTime(t);
                double off1 = offsetAtTime(t + dt);
                cumulative += 0.5 * (off0 + off1) * dt;
            }
        }
    }

    double integratedOffsetInterp(double t) const {
        if (tableTimes_.empty()) return 0.0;
        if (t <= tableTimes_.front()) return tableIntegratedOffsets_.front();
        if (t >= tableTimes_.back()) return tableIntegratedOffsets_.back();

        // Binary search
        auto it = std::lower_bound(tableTimes_.begin(), tableTimes_.end(), t);
        size_t idx = static_cast<size_t>(it - tableTimes_.begin());
        if (idx == 0) return tableIntegratedOffsets_[0];

        double t0 = tableTimes_[idx - 1];
        double t1 = tableTimes_[idx];
        double v0 = tableIntegratedOffsets_[idx - 1];
        double v1 = tableIntegratedOffsets_[idx];
        double frac = (t - t0) / (t1 - t0);
        return v0 + frac * (v1 - v0);
    }
};

} // namespace MotionPlanner::analytical::extrusion
