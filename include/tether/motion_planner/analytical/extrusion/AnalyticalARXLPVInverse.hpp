/**
 * @file AnalyticalARXLPVInverse.hpp
 * @brief Analytical ARX LPV inverse filter on WSS arcs.
 *
 * @details
 * The continuous-time equivalent of the discrete ARX model is an ODE.
 * For a first-order model (N_a = 1, N_b = 0):
 *
 *   ẏ(t) + a(p(t))·y(t) = b(p(t))·x(t)
 *
 * Solving for x(t):
 *
 *   x(t) = (ẏ(t) + a(p(t))·y(t)) / b(p(t))
 *
 * Since y(t) is piecewise polynomial from the WSS, ẏ(t) is also piecewise
 * polynomial (one degree lower).  The scheduling parameter p(t) = v(t) is
 * piecewise polynomial.  The ARX coefficients a(p), b(p) are interpolated
 * from the LUT.
 *
 * Within each arc, using the arc-averaged p̄:
 *
 *   x(t) = (ẏ(t) + a(p̄)·y(t)) / b(p̄)
 *
 * This is a ratio of polynomials — fully closed-form.
 *
 * For higher-order models (N_a ≥ 2), the continuous-time equivalent is:
 *
 *   Σ a_i(p)·y^(i)(t) = Σ b_j(p)·x^(j)(t)
 *
 * where y^(i) denotes the i-th time derivative.  Since y(t) is piecewise
 * polynomial of degree K, y^(i) is piecewise polynomial of degree K-i
 * (zero for i > K).  The equation is solved algebraically for x(t) when
 * N_b = 0, or as an ODE when N_b > 0.
 *
 * Transport delay: the discrete delay d becomes a continuous-time delay
 * t_d = d·T_s.  The analytical form uses a delayed target y_tgt(t + t_d),
 * evaluated by looking ahead in the WSS arc list.
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §6
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief ARX model identified at a single operating point (continuous-time).
 */
struct AnalyticalARXModelPoint {
    double parameter = 0.0;            ///< Scheduling parameter p
    std::vector<double> aCoeffs;       ///< A: [a1, a2, ..., aNa] (continuous-time)
    std::vector<double> bCoeffs;       ///< B: [b0, b1, ..., bNb] (continuous-time)
    double delay = 0.0;                ///< Transport delay [s]
};

/**
 * @brief Parameters for analytical ARX LPV inverse filter.
 */
struct AnalyticalARXLPVParams {
    /// Order of A (autoregressive part)
    int na = 1;

    /// Order of B (exogenous part, excluding delay)
    int nb = 0;

    /// If true, use extruder position as y(t); if false, use velocity
    bool usePosition = false;
};

/**
 * @brief Analytical ARX LPV inverse filter.
 *
 * Computes the required input x(t) by algebraically inverting the
 * continuous-time ARX ODE, using piecewise-polynomial y(t) from the WSS.
 */
template<size_t Dim, typename T = double>
class AnalyticalARXLPVInverse {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct from trajectory and parameters.
     */
    AnalyticalARXLPVInverse(const Traj& traj, AnalyticalARXLPVParams params)
        : traj_(&traj), params_(params) {}

    /**
     * @brief Add an identified ARX model at operating point p.
     */
    void addModelPoint(const AnalyticalARXModelPoint& point) {
        modelLut_[point.parameter] = point;
    }

    /**
     * @brief Add an identified ARX model (convenience overload).
     */
    void addModelPoint(double p, std::vector<double> a,
                       std::vector<double> b, double delay = 0.0) {
        AnalyticalARXModelPoint point;
        point.parameter = p;
        point.aCoeffs = std::move(a);
        point.bCoeffs = std::move(b);
        point.delay = delay;
        modelLut_[p] = std::move(point);
    }

    /**
     * @brief Compute the required input x(t) at time t.
     *
     * For the first-order case (na=1, nb=0):
     *   x(t) = (ẏ(t+d) + a(p̄)·y(t+d)) / b(p̄)
     *
     * where d is the transport delay and p̄ is the arc-averaged velocity.
     */
    double inputAtTime(double t) const {
        if (modelLut_.empty()) return 0.0;

        const auto& arcs = traj_->arcs();
        if (arcs.empty()) return 0.0;

        // Get interpolated model at the current arc's average velocity
        size_t idx = traj_->findArc(t);
        const auto& a = arcs[idx];
        double pBar = a.avgPathVelocity();
        auto model = interpolateModel(pBar);

        if (model.bCoeffs.empty() || std::abs(model.bCoeffs[0]) < 1e-15)
            return 0.0;

        // Apply transport delay: look ahead by model.delay
        double tDelayed = t + model.delay;
        double y, ydot;

        if (params_.usePosition) {
            y = traj_->extruderPositionAtTime(tDelayed);
            // ẏ = v_e (derivative of position = velocity)
            ydot = traj_->extruderVelocityAtTime(tDelayed);
        } else {
            y = traj_->extruderVelocityAtTime(tDelayed);
            // ẏ = α_e · a(t) (derivative of velocity = acceleration)
            ydot = a.extrusionRatio * traj_->pathAccelerationAtTime(tDelayed);
        }

        // For na=1, nb=0: x = (ẏ + a1·y) / b0
        if (params_.na == 1 && params_.nb == 0) {
            double a1 = model.aCoeffs.size() >= 1 ? model.aCoeffs[0] : 0.0;
            double b0 = model.bCoeffs[0];
            return (ydot + a1 * y) / b0;
        }

        // For na=2, nb=0: x = (ÿ + a1·ẏ + a2·y) / b0
        if (params_.na == 2 && params_.nb == 0) {
            double a1 = model.aCoeffs.size() >= 1 ? model.aCoeffs[0] : 0.0;
            double a2 = model.aCoeffs.size() >= 2 ? model.aCoeffs[1] : 0.0;
            double b0 = model.bCoeffs[0];
            // ÿ = derivative of ẏ
            // For y = v_e: ÿ = α_e · η (jerk)
            // For y = position: ÿ = α_e · a(t) (acceleration)
            double yddot;
            if (params_.usePosition) {
                yddot = a.extrusionRatio * traj_->pathAccelerationAtTime(tDelayed);
            } else {
                yddot = a.extrusionRatio * traj_->pathJerkAtTime(tDelayed);
            }
            return (yddot + a1 * ydot + a2 * y) / b0;
        }

        // General case: na ≥ 1, nb ≥ 0
        // Σ a_i · y^(i) = Σ b_j · x^(j)
        // For nb = 0: x = (Σ a_i · y^(i)) / b0
        // For nb > 0: need to solve ODE (simplified: assume x^(j) ≈ 0 for j>0)
        double numerator = ydot;
        for (int i = 0; i < static_cast<int>(model.aCoeffs.size()); ++i) {
            // y^(i+1) — for higher derivatives, use 0 (polynomial degree limit)
            double yDeriv = 0.0;
            if (i == 0) yDeriv = ydot;
            else if (i == 1 && !params_.usePosition) {
                yDeriv = a.extrusionRatio * traj_->pathJerkAtTime(tDelayed);
            }
            numerator += model.aCoeffs[i] * (i == 0 ? y : yDeriv);
        }
        return numerator / model.bCoeffs[0];
    }

    /**
     * @brief Compute the required input at multiple time points.
     */
    std::vector<double> inputSeries(const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(inputAtTime(t));
        return result;
    }

    /**
     * @brief Compute the adjusted extruder position at time t.
     */
    double adjustedExtruderPosition(double t) const {
        return traj_->extruderPositionAtTime(t) + inputAtTime(t);
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

    /// Number of model points
    size_t numModelPoints() const { return modelLut_.size(); }

    /// Parameters
    const AnalyticalARXLPVParams& params() const { return params_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalARXLPVParams params_;
    std::map<double, AnalyticalARXModelPoint> modelLut_;

    AnalyticalARXModelPoint interpolateModel(double p) const {
        if (modelLut_.empty()) return {};
        if (modelLut_.size() == 1) return modelLut_.begin()->second;

        auto upper = modelLut_.lower_bound(p);
        if (upper == modelLut_.begin()) return upper->second;
        if (upper == modelLut_.end()) return std::prev(upper)->second;

        auto lower = std::prev(upper);
        double p0 = lower->first, p1 = upper->first;
        double frac = (p - p0) / (p1 - p0);

        AnalyticalARXModelPoint result;
        result.parameter = p;
        result.delay = lower->second.delay
                       + frac * (upper->second.delay - lower->second.delay);
        result.aCoeffs.resize(lower->second.aCoeffs.size());
        for (size_t i = 0; i < result.aCoeffs.size(); ++i)
            result.aCoeffs[i] = lower->second.aCoeffs[i]
                + frac * (upper->second.aCoeffs[i] - lower->second.aCoeffs[i]);
        result.bCoeffs.resize(lower->second.bCoeffs.size());
        for (size_t i = 0; i < result.bCoeffs.size(); ++i)
            result.bCoeffs[i] = lower->second.bCoeffs[i]
                + frac * (upper->second.bCoeffs[i] - lower->second.bCoeffs[i]);
        return result;
    }
};

} // namespace MotionPlanner::analytical::extrusion
