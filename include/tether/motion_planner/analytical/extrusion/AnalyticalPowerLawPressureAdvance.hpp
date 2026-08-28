/**
 * @file AnalyticalPowerLawPressureAdvance.hpp
 * @brief Analytical power-law pressure advance on WSS arcs.
 *
 * @details
 * The power-law pressure-advance model in continuous-time form:
 *
 *   δe(t) = K_base · (v_e(t) · A_f)^n = K_base · (α_e · A_f)^n · v(t)^n
 *
 * where:
 *   K_base = βV_m·C_n/A_f  [filament-mm / (mm³/s)^n]
 *   A_f    = filament cross-sectional area [mm²]
 *   n      = flow index (1 = Newtonian → reduces to linear PressureAdvance)
 *   v_e(t) = α_e · v(t)    [mm/s] (piecewise polynomial from WSS)
 *
 * The offset δe(t) is evaluated in closed form: within each arc, v(τ) is a
 * known polynomial, so v(τ)^n is a known function evaluated directly.
 *
 * The cumulative offset Δe(t) = ∫₀^t δe(t') dt' involves ∫ v(τ)^n dτ, which
 * has a closed form for:
 *   - Constant v (WALL arcs): v^n · τ
 *   - Linear v (SINGULAR arcs): (v0 + a*·τ)^(n+1) / (a*·(n+1))
 *   - Quadratic v (BANG arcs): polynomial expansion (integer n) or
 *     Gauss-Legendre quadrature (non-integer n)
 *
 * With smoothing, the offset uses the continuous-time smoothed velocity
 * v_e_smooth(t) (rectangular-window convolution, see AnalyticalExtrusionTypes).
 *
 * Newtonian limit: n=1, K_base = PressureAdvance/A_f → δe = PressureAdvance · v_e (linear PressureAdvance).
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §2
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for analytical power-law pressure advance.
 */
struct AnalyticalPowerLawPressureAdvanceParams {
    /// K_base = βV_m·C_n/A_f [filament-mm / (mm³/s)^n]
    double baseGain = 0.0;

    /// Flow index n (1 = Newtonian, <1 = shear-thinning, >1 = shear-thickening)
    double flowIndex = 1.0;

    /// Filament diameter [mm]
    double filamentDiameterMm = 1.75;

    /// Smoothing window [s] (0 = no smoothing)
    double smoothTime = 0.0;

    /// Maximum absolute compensation [mm] (safety clamp, 0 = no clamp)
    double maxCompensation = 0.0;
};

/**
 * @brief Analytical power-law pressure advance.
 *
 * Computes the extruder position offset δe(t) = K_base · (v_e(t)·A_f)^n
 * in closed form from the WSS piecewise-polynomial velocity.
 */
template<size_t Dim, typename T = double>
class AnalyticalPowerLawPressureAdvance {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct from an extrusion trajectory and parameters.
     */
    AnalyticalPowerLawPressureAdvance(const Traj& traj, AnalyticalPowerLawPressureAdvanceParams params)
        : traj_(&traj), params_(params) {
        filamentAreaMm2_ = M_PI * params_.filamentDiameterMm
                           * params_.filamentDiameterMm / 4.0;
        precomputeIntegratedOffsets();
    }

    /**
     * @brief Compute the instantaneous position offset δe(t) [mm].
     */
    double offsetAtTime(double t) const {
        if (params_.baseGain <= 0.0) return 0.0;

        double vE;
        if (params_.smoothTime > 0.0) {
            vE = smoothedExtruderVelocity(*traj_, t, params_.smoothTime);
        } else {
            vE = traj_->extruderVelocityAtTime(t);
        }

        if (vE <= 0.0) return 0.0;

        double Q = vE * filamentAreaMm2_;
        double offset = params_.baseGain * std::pow(Q, params_.flowIndex);
        return clampOffset(offset);
    }

    /**
     * @brief Compute the cumulative (integrated) offset Δe(t) [mm].
     *
     * Without smoothing:
     *   Δe(t) = K_base · A_f^n · ∫₀^t v_e(τ)^n dτ
     *         = K_base · (α_e · A_f)^n · ∫₀^t v(τ)^n dτ
     *
     * The integral ∫ v(τ)^n dτ is computed per-arc using the closed-form
     * or quadrature helpers from AnalyticalExtrusionTypes.
     */
    double integratedOffsetAtTime(double t) const {
        if (params_.baseGain <= 0.0) return 0.0;

        if (params_.smoothTime <= 0.0) {
            return integratedOffsetClosedForm(t);
        }

        return integratedOffsetInterp(t);
    }

    /**
     * @brief Compute the offset at multiple time points.
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
    const AnalyticalPowerLawPressureAdvanceParams& params() const { return params_; }

    /// Filament cross-sectional area [mm²]
    double filamentAreaMm2() const { return filamentAreaMm2_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalPowerLawPressureAdvanceParams params_;
    double filamentAreaMm2_ = 0.0;

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

    /**
     * @brief Closed-form integrated offset without smoothing.
     *
     * Δe(t) = K_base · (α_e · A_f)^n · Σ_arcs ∫₀^{τ_end} v(τ)^n dτ
     */
    double integratedOffsetClosedForm(double t) const {
        const auto& arcs = traj_->arcs();
        if (arcs.empty()) return 0.0;

        double n = params_.flowIndex;
        double Keff = params_.baseGain * std::pow(filamentAreaMm2_, n);
        // Keff has units [filament-mm / (mm/s)^n], and we multiply by
        // (α_e · v)^n integrated over time → [filament-mm]

        double total = 0.0;
        for (const auto& a : arcs) {
            if (a.extrusionRatio <= 0.0) continue;
            double tauEnd;
            if (t >= a.t0 + a.duration) {
                tauEnd = a.duration;
            } else if (t > a.t0) {
                tauEnd = t - a.t0;
            } else {
                break;  // Past the query time
            }

            // ∫₀^{tauEnd} (α_e · v(τ))^n dτ = α_e^n · ∫ v(τ)^n dτ
            double alphaN = std::pow(a.extrusionRatio, n);
            double integral = velocityPowerIntegral(a.c0, a.c1, a.c2,
                                                     a.c3, n, tauEnd);
            total += Keff * alphaN * integral;
        }
        return total;
    }

    void precomputeIntegratedOffsets() {
        if (params_.smoothTime <= 0.0 || params_.baseGain <= 0.0)
            return;

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
