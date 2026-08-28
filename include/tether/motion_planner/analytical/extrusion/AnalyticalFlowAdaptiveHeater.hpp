/**
 * @file AnalyticalFlowAdaptiveHeater.hpp
 * @brief Analytical flow-adaptive heater controller with no explicit sampling.
 *
 * @details
 * Detects the first flow onset and last flow stop by solving the cubic
 * threshold-crossing equation in closed form on each WSS arc.  This avoids
 * the per-arc dense sampling used by `SamplingFlowAdaptiveHeater`.
 *
 * The output is a transient temperature delta [°C] to be added to the
 * extruder temperature target; see `FlowAdaptiveHeaterBase.hpp` for the
 * physical model.
 *
 * @see FlowAdaptiveHeaterBase.hpp
 * @see SamplingFlowAdaptiveHeater.hpp
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §8
 */

#pragma once

#include "FlowAdaptiveHeaterBase.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Analytical flow-adaptive heater controller.
 */
template<size_t Dim, typename T = double>
class AnalyticalFlowAdaptiveHeater
    : public FlowAdaptiveHeaterBase<Dim, T,
                                    AnalyticalFlowAdaptiveHeater<Dim, T>> {
public:
    using Base = FlowAdaptiveHeaterBase<Dim, T,
                                        AnalyticalFlowAdaptiveHeater<Dim, T>>;
    using Traj = ExtrusionTrajectory<Dim, T>;

    AnalyticalFlowAdaptiveHeater(const Traj& traj,
                                 AnalyticalFlowAdaptiveHeaterParams params)
        : Base(traj, params) {}

private:
    friend Base;

    /// Find the first and last local times [s] within an arc where
    /// v(τ) > vThresh, by solving the polynomial roots analytically.
    static std::pair<double, double> flowInterval(
        const ExtrusionArc& a, double vThresh) {

        // p(τ) = c3·τ³ + c2·τ² + c1·τ + (c0 - vThresh)
        double c0 = a.c0 - vThresh;
        double c1 = a.c1;
        double c2 = a.c2;
        double c3 = a.c3;
        double d = a.duration;

        std::vector<double> roots;
        roots.reserve(6);
        roots.push_back(0.0);
        roots.push_back(d);

        if (std::abs(c3) > 1e-15) {
            // Cubic: c3·τ³ + c2·τ² + c1·τ + c0 = 0
            // Normalize: τ³ + p·τ² + q·τ + r = 0
            double p = c2 / c3;
            double q = c1 / c3;
            double r = c0 / c3;
            // Depressed cubic: y³ + py' + q' = 0, τ = y - p/3
            double pp = q - p * p / 3.0;
            double qq = 2.0 * p * p * p / 27.0 - p * q / 3.0 + r;
            double disc = (qq * qq) / 4.0 + (pp * pp * pp) / 27.0;
            double shift = -p / 3.0;

            if (disc > 1e-15) {
                // One real root (Cardano)
                double sq = std::sqrt(disc);
                double u = std::cbrt(-qq / 2.0 + sq);
                double v_root = std::cbrt(-qq / 2.0 - sq);
                double root = u + v_root + shift;
                if (root >= 0.0 && root <= d) roots.push_back(root);
            } else if (disc < -1e-15) {
                // Three distinct real roots (trigonometric)
                double m = 2.0 * std::sqrt(-pp / 3.0);
                double theta = std::acos(3.0 * qq / (pp * m)) / 3.0;
                for (int k = 0; k < 3; ++k) {
                    double root = m * std::cos(theta + 2.0 * k * M_PI / 3.0)
                                  + shift;
                    if (root >= 0.0 && root <= d) roots.push_back(root);
                }
            } else {
                // disc ≈ 0: three real roots, at least two equal
                if (std::abs(pp) < 1e-15) {
                    // Triple root
                    double root = shift;
                    if (root >= 0.0 && root <= d) roots.push_back(root);
                } else {
                    double root1 = 3.0 * qq / pp + shift;
                    double root2 = -3.0 * qq / (2.0 * pp) + shift;
                    if (root1 >= 0.0 && root1 <= d) roots.push_back(root1);
                    if (root2 >= 0.0 && root2 <= d
                        && std::abs(root2 - root1) > 1e-15)
                        roots.push_back(root2);
                }
            }
        } else if (std::abs(c2) > 1e-15) {
            double disc = c1 * c1 - 4.0 * c2 * c0;
            if (disc >= 0.0) {
                double s = std::sqrt(disc);
                double r1 = (-c1 - s) / (2.0 * c2);
                double r2 = (-c1 + s) / (2.0 * c2);
                if (r1 >= 0.0 && r1 <= d) roots.push_back(r1);
                if (r2 >= 0.0 && r2 <= d && std::abs(r2 - r1) > 1e-15)
                    roots.push_back(r2);
            }
        } else if (std::abs(c1) > 1e-15) {
            double r = -c0 / c1;
            if (r >= 0.0 && r <= d) roots.push_back(r);
        }

        std::sort(roots.begin(), roots.end());

        double first = std::numeric_limits<double>::infinity();
        double last = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i + 1 < roots.size(); ++i) {
            double tmid = 0.5 * (roots[i] + roots[i + 1]);
            double v = c0 + c1 * tmid + c2 * tmid * tmid
                       + c3 * tmid * tmid * tmid;
            if (v > 0.0) {
                first = std::min(first, roots[i]);
                last = std::max(last, roots[i + 1]);
            }
        }

        return {first, last};
    }

    void detectFlowTransitions() {
        const auto& arcs = this->traj_->arcs();
        double threshold = this->params_.flowThresholdMm3PerS;

        if (this->filamentAreaMm2_ <= 0.0) return;

        for (const auto& a : arcs) {
            if (a.extrusionRatio <= 0.0) continue;

            double vThresh = threshold / (a.extrusionRatio * this->filamentAreaMm2_);
            auto [first, last] = flowInterval(a, vThresh);

            if (first <= last) {
                double tOnset = a.t0 + first;
                double tStop = a.t0 + last;

                if (!this->hasFlowOnset_ || tOnset < this->flowOnsetTime_) {
                    this->flowOnsetTime_ = tOnset;
                    this->hasFlowOnset_ = true;
                }
                if (!this->hasFlowStop_ || tStop > this->flowStopTime_) {
                    this->flowStopTime_ = tStop;
                    this->hasFlowStop_ = true;
                }
            }
        }
    }
};

} // namespace MotionPlanner::analytical::extrusion
