/**
 * @file AnalyticalCrossWLFPressureAdvance.hpp
 * @brief Analytical Cross-WLF pressure advance on WSS arcs.
 *
 * @details
 * The Cross-WLF pressure-advance model in continuous-time form:
 *
 *   δe(t) = (βV_m / A_f) · P_LUT(Q(t), T(t))
 *
 * where:
 *   Q(t) = α_e · A_f · v(t)  [mm³/s] — piecewise polynomial from WSS
 *   T(t) = melt temperature from AnalyticalMeltZoneThermalObserver [°C]
 *   P_LUT(Q, T) = bilinear interpolation on a 2-D {Q, T} → P grid
 *
 * The LUT is piecewise bilinear: within each grid cell
 * (Q_j, Q_{j+1}) × (T_k, T_{k+1}):
 *
 *   P = c₀₀ + c₁₀·ΔQ + c₀₁·ΔT + c₁₁·ΔQ·ΔT
 *
 * where ΔQ = Q - Q_j, ΔT = T - T_k.
 *
 * Since Q(t) is piecewise polynomial and T(t) is piecewise exponential
 * (from the thermal observer), the offset δe(t) is a piecewise
 * bilinear×polynomial×exponential function — evaluated in closed form
 * at any time t by:
 *   1. Evaluating Q(t) and T(t) (both closed-form)
 *   2. Looking up the LUT cell
 *   3. Evaluating the bilinear expression
 *
 * When Q(t) or T(t) crosses a cell boundary within an arc, the arc is
 * subdivided at the crossing time (found by root-finding on the polynomial
 * Q(t) - Q_j = 0).
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §3
 * @see AnalyticalMeltZoneThermalObserver.hpp for T(t)
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"
#include "AnalyticalMeltZoneThermalObserver.hpp"
#include "tether/control/extrusion/PressureFlowLut.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for analytical Cross-WLF pressure advance.
 */
struct AnalyticalCrossWLFPressureAdvanceParams {
    /// βV_m / A_f [mm/Pa] — melt compressibility over filament area
    double compressibilityOverArea = 0.0;

    /// Filament diameter [mm]
    double filamentDiameterMm = 1.75;

    /// Smoothing window [s] (0 = no smoothing)
    double smoothTime = 0.0;

    /// Maximum absolute compensation [mm] (safety clamp, 0 = no clamp)
    double maxCompensation = 0.0;

    /// Default melt temperature [°C] when no thermal observer is provided
    double defaultTempC = 210.0;
};

/**
 * @brief Analytical Cross-WLF pressure advance.
 *
 * Computes the extruder position offset δe(t) = (βV_m/A_f) · P_LUT(Q(t), T(t))
 * in closed form from the WSS piecewise-polynomial velocity and the
 * analytical thermal observer's piecewise-exponential temperature.
 */
template<size_t Dim, typename T = double>
class AnalyticalCrossWLFPressureAdvance {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;
    using ThermalObs = AnalyticalMeltZoneThermalObserver<Dim, T>;

    /**
     * @brief Construct with a thermal observer (for temperature-dependent P).
     *
     * @param traj Extrusion trajectory
     * @param lut Pressure-flow lookup table {Q, T} → P
     * @param params Parameters
     * @param thermalObserver Thermal observer (provides T(t))
     */
    AnalyticalCrossWLFPressureAdvance(const Traj& traj,
                         std::shared_ptr<tether::control::extrusion::PressureFlowLut> lut,
                         AnalyticalCrossWLFPressureAdvanceParams params,
                         const ThermalObs* thermalObserver = nullptr)
        : traj_(&traj)
        , lut_(std::move(lut))
        , params_(params)
        , thermalObserver_(thermalObserver) {
        filamentAreaMm2_ = M_PI * params_.filamentDiameterMm
                           * params_.filamentDiameterMm / 4.0;
    }

    /**
     * @brief Compute the instantaneous position offset δe(t) [mm].
     */
    double offsetAtTime(double t) const {
        if (!lut_ || lut_->empty() || params_.compressibilityOverArea <= 0.0)
            return 0.0;

        double vE;
        if (params_.smoothTime > 0.0) {
            vE = smoothedExtruderVelocity(*traj_, t, params_.smoothTime);
        } else {
            vE = traj_->extruderVelocityAtTime(t);
        }

        if (vE <= 0.0) return 0.0;

        double Q = vE * filamentAreaMm2_;
        double tempC = getTempAt(t);

        double P = lut_->pressure(Q, tempC);
        double offset = params_.compressibilityOverArea * P;
        return clampOffset(offset);
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

    /**
     * @brief Compute the flow rate Q(t) at time t [mm³/s].
     */
    double flowAtTime(double t) const {
        double vE = traj_->extruderVelocityAtTime(t);
        return vE * filamentAreaMm2_;
    }

    /**
     * @brief Compute the pressure P(t) at time t [Pa].
     */
    double pressureAtTime(double t) const {
        if (!lut_ || lut_->empty()) return 0.0;
        double Q = flowAtTime(t);
        if (Q <= 0.0) return 0.0;
        double tempC = getTempAt(t);
        return lut_->pressure(Q, tempC);
    }

    /// Parameters
    const AnalyticalCrossWLFPressureAdvanceParams& params() const { return params_; }

    /// LUT
    const tether::control::extrusion::PressureFlowLut& lut() const {
        return *lut_;
    }

    /// Filament area [mm²]
    double filamentAreaMm2() const { return filamentAreaMm2_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    std::shared_ptr<tether::control::extrusion::PressureFlowLut> lut_;
    AnalyticalCrossWLFPressureAdvanceParams params_;
    const ThermalObs* thermalObserver_;
    double filamentAreaMm2_ = 0.0;

    double clampOffset(double offset) const {
        if (params_.maxCompensation > 0.0) {
            return std::clamp(offset, -params_.maxCompensation,
                              params_.maxCompensation);
        }
        return offset;
    }

    double getTempAt(double t) const {
        if (thermalObserver_) {
            return thermalObserver_->meltTempAt(t);
        }
        return params_.defaultTempC;
    }
};

} // namespace MotionPlanner::analytical::extrusion
