/**
 * @file ExtrusionFlowTracker.hpp
 * @brief Shared recent-flow tap between the motion dispatcher and the
 *        flow-adaptive heater compensation.
 *
 * @details
 * KlippyInstance never runs heater loops itself (the application calls
 * Heater::control() / FlowAdaptiveHeaterController::compute()), so the
 * heater compensation needs a way to learn the current extrusion flow
 * without coupling to the dispatcher. This small thread-unsafe-by-design
 * shared holder is updated by MotionDispatcher on every move (from the
 * per-move E-velocity) and read by the heater compensation hook.
 *
 * The tracker exposes both the instantaneous flow and a short exponentially
 * weighted moving average so the heater controller can use a smoothed value
 * for its feed-forward.
 */

#pragma once

#include <atomic>
#include <cmath>

namespace tether::klipper::motion {

/// @brief Shared recent-extrusion-flow tap.
class ExtrusionFlowTracker {
public:
    /// @brief Set the filament cross-sectional area [mm²] (used to convert
    /// E-axis velocity to volumetric flow).
    void setFilamentAreaMm2(double area) { filamentAreaMm2_ = area; }
    double filamentAreaMm2() const { return filamentAreaMm2_; }

    /// @brief Set the filament diameter [mm] (convenience; updates the area).
    void setFilamentDiameterMm(double d) {
        filamentAreaMm2_ = M_PI * d * d / 4.0;
    }

    /// @brief Record the E-axis velocity from the latest move [mm/s].
    /// Updates the instantaneous and smoothed flow estimates.
    void setExtruderVelocityMmPerS(double velMmPerS, double dtSec) {
        const double Q = velMmPerS * filamentAreaMm2_;
        instantaneousQ_.store(Q, std::memory_order_relaxed);
        // Exponentially weighted moving average with a ~0.5 s time constant.
        const double tau = 0.5;
        const double alpha = (dtSec > 0.0) ? std::clamp(dtSec / tau, 0.0, 1.0)
                                          : 1.0;
        double prev = smoothedQ_.load(std::memory_order_relaxed);
        double next;
        do {
            next = prev + alpha * (Q - prev);
        } while (!smoothedQ_.compare_exchange_weak(prev, next,
                 std::memory_order_relaxed));
    }

    /// @brief Instantaneous volumetric flow [mm³/s].
    double instantaneousFlowMm3PerS() const {
        return instantaneousQ_.load(std::memory_order_relaxed);
    }

    /// @brief Smoothed (EWMA) volumetric flow [mm³/s].
    double smoothedFlowMm3PerS() const {
        return smoothedQ_.load(std::memory_order_relaxed);
    }

    /// @brief Reset the tracker to zero flow.
    void reset() {
        instantaneousQ_.store(0.0, std::memory_order_relaxed);
        smoothedQ_.store(0.0, std::memory_order_relaxed);
    }

private:
    double filamentAreaMm2_ = M_PI * 1.75 * 1.75 / 4.0; // 1.75 mm filament
    std::atomic<double> instantaneousQ_{0.0};
    std::atomic<double> smoothedQ_{0.0};
};

} // namespace tether::klipper::motion
