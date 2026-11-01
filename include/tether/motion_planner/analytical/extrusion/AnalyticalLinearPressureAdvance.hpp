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
    /**Classic linear PressureAdvance amount [s]. */
    double pressureAdvance = 0.0;

    /**Smoothing window [s] (0 = no smoothing). */
    double smoothTime = 0.0;

    /**Maximum absolute compensation [mm] (safety clamp, 0 = no clamp). */
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
     *
     * Stores a pointer to @p traj — the trajectory must outlive this
     * object. When `params.smoothTime > 0` the constructor precomputes a
     * trapezoidal-integration table of the smoothed offset (≥100 samples,
     * ~1 ms resolution) so integratedOffsetAtTime() is O(log n) per query.
     *
     * @param traj    Extrusion trajectory supplying v_e(t) and e(t);
     *                borrowed, not owned.
     * @param params  Pressure-advance amount [s], smoothing window [s],
     *                and optional compensation clamp [mm]. Copied.
     */
    AnalyticalLinearPressureAdvance(const Traj& traj, AnalyticalLinearPressureAdvanceParams params)
        : traj_(&traj), params_(params) {
        precomputeIntegratedOffsets();
    }

    /**
     * @brief Compute the instantaneous position offset δe(t) [mm].
     *
     * This is the pressure-advance offset added to the raw extruder
     * position at time t: `δe = pressureAdvance · v_e(t)`, where v_e is
     * the raw WSS extruder velocity or its centered moving average when
     * `smoothTime > 0`. Returns 0 when `pressureAdvance <= 0`, and the
     * result is symmetrically clamped to `maxCompensation` when that is
     * positive.
     *
     * @param t  Query time [s] within the trajectory.
     * @return Extruder position offset [mm] to add at time t.
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
     * precomputed offset samples — trapezoidal accumulation at ~1 ms
     * resolution, linearly interpolated between table entries and clamped
     * to the table ends outside [0, totalTime].
     *
     * @param t  Query time [s].
     * @return Accumulated offset ∫₀^t δe dt' [mm·s] — note the units:
     *         this integrates a position offset over time, so it is
     *         PressureAdvance-scaled extrusion-distance, not a position.
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
     *
     * Batch form of integratedOffsetAtTime() — same semantics per element.
     *
     * @param times  Query times [s], any order.
     * @return Integrated offsets [mm·s], same size and order as @p times.
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
     * `e_adjusted(t) = e_raw(t) + δe(t)` — the raw extrusion position with
     * the instantaneous pressure-advance offset applied, i.e. the target
     * position stream a stepper generator would consume.
     *
     * @param t  Query time [s].
     * @return Pressure-advance-adjusted extruder position [mm].
     */
    double adjustedExtruderPosition(double t) const {
        return traj_->extruderPositionAtTime(t) + offsetAtTime(t);
    }

    /**
     * @brief Compute the adjusted extruder position at multiple times.
     *
     * Batch form of adjustedExtruderPosition() — same semantics per
     * element; this is the ready-to-sample command stream for the
     * extruder axis.
     *
     * @param times  Query times [s], any order.
     * @return Adjusted extruder positions [mm], same size and order as
     *         @p times.
     */
    std::vector<double> adjustedExtruderPositionSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(adjustedExtruderPosition(t));
        return result;
    }

    /**
     * @brief The parameters this compensator was constructed with.
     *
     * @return Const reference to the copied parameter set (pressureAdvance
     *         [s], smoothTime [s], maxCompensation [mm]); valid for the
     *         lifetime of this object.
     */
    const AnalyticalLinearPressureAdvanceParams& params() const { return params_; }

    /**
     * @brief The extrusion trajectory this compensator reads from.
     *
     * @return Const reference to the trajectory passed to the constructor.
     *         The reference aliases the caller's original — the trajectory
     *         must remain alive while this object exists.
     */
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
