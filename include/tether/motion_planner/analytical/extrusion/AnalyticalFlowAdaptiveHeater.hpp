/**
 * @file AnalyticalFlowAdaptiveHeater.hpp
 * @brief Analytical flow-adaptive heater controller with no explicit sampling.
 *
 * @details
 * Detects the first flow onset and last flow stop by solving the quadratic
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
    /// v(τ) > vThresh, by solving the quadratic roots analytically.
    static std::pair<double, double> flowInterval(
        const ExtrusionArc& a, double vThresh) {

        // p(τ) = (c2) τ² + (c1) τ + (c0 - vThresh)
        double c0 = a.c0 - vThresh;
        double c1 = a.c1;
        double c2 = a.c2;
        double d = a.duration;

        std::vector<double> roots;
        roots.reserve(4);
        roots.push_back(0.0);
        roots.push_back(d);

        if (std::abs(c2) > 1e-15) {
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
            double v = c0 + c1 * tmid + c2 * tmid * tmid;
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
