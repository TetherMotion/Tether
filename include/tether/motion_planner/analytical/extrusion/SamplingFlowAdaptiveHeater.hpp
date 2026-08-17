/**
 * @file SamplingFlowAdaptiveHeater.hpp
 * @brief Flow-adaptive heater controller that detects flow transitions by
 *        explicit point sampling.
 *
 * @details
 * This is the sampled-space counterpart to `AnalyticalFlowAdaptiveHeater`.
 * It estimates the first flow onset and last flow stop by sampling the
 * piecewise-polynomial flow Q(t) on each arc.  The output is the same
 * transient temperature delta [°C] as the analytical version.
 *
 * @see FlowAdaptiveHeaterBase.hpp
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §8
 */

#pragma once

#include "FlowAdaptiveHeaterBase.hpp"

#include <algorithm>
#include <cmath>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Flow-adaptive heater controller with explicit flow-transition
 *        sampling.
 */
template<size_t Dim, typename T = double>
class SamplingFlowAdaptiveHeater
    : public FlowAdaptiveHeaterBase<Dim, T,
                                    SamplingFlowAdaptiveHeater<Dim, T>> {
public:
    using Base = FlowAdaptiveHeaterBase<Dim, T,
                                        SamplingFlowAdaptiveHeater<Dim, T>>;
    using Traj = ExtrusionTrajectory<Dim, T>;

    SamplingFlowAdaptiveHeater(const Traj& traj,
                               AnalyticalFlowAdaptiveHeaterParams params)
        : Base(traj, params) {}

private:
    friend Base;

    void detectFlowTransitions() {
        const auto& arcs = this->traj_->arcs();
        double threshold = this->params_.flowThresholdMm3PerS;

        for (const auto& a : arcs) {
            if (a.extrusionRatio <= 0.0) continue;

            int nCheck = std::max(10,
                static_cast<int>(a.duration / 0.001));

            // Onset: first time Q crosses threshold.
            for (int i = 0; i <= nCheck; ++i) {
                double tau = a.duration * static_cast<double>(i) / nCheck;
                double Q = a.extruderVelocity(tau) * this->filamentAreaMm2_;
                if (Q > threshold) {
                    double t = a.t0 + tau;
                    if (!this->hasFlowOnset_ || t < this->flowOnsetTime_) {
                        this->flowOnsetTime_ = t;
                        this->hasFlowOnset_ = true;
                    }
                    break;
                }
            }

            // Stop: last time Q is still above threshold.
            double lastFlowTime = 0.0;
            for (int i = 0; i <= nCheck; ++i) {
                double tau = a.duration * static_cast<double>(i) / nCheck;
                double Q = a.extruderVelocity(tau) * this->filamentAreaMm2_;
                if (Q > threshold) {
                    lastFlowTime = a.t0 + tau;
                }
            }
            if (lastFlowTime > 0.0) {
                if (!this->hasFlowStop_ || lastFlowTime > this->flowStopTime_) {
                    this->flowStopTime_ = lastFlowTime;
                    this->hasFlowStop_ = true;
                }
            }
        }
    }
};

} // namespace MotionPlanner::analytical::extrusion
