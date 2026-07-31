#pragma once

/// @file PressureAdvance.hpp
/// @brief Pressure advance filter for extrusion motion

#include <cmath>
#include <deque>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Pressure advance parameters for an extruder.
struct PressureAdvanceParams {
    double pressureAdvance = 0.0;    ///< PA amount in seconds
    double smoothTime = 0.04;        ///< Smoothing window in seconds (0 = no smoothing)
};

/// @brief Pressure advance filter for extrusion motion.
///
/// Implements the Klipper pressure advance algorithm:
/// When the extruder accelerates, pressure builds in the hotend.
/// PA advances the extruder timing so that pressure is built up
/// before the move starts and released after the move ends.
class PressureAdvance {
public:
    explicit PressureAdvance(PressureAdvanceParams params = {})
        : params_(params) {}

    /// @brief Set the pressure advance parameters.
    void setParams(PressureAdvanceParams params) { params_ = params; }

    /// @brief Get the current parameters.
    const PressureAdvanceParams& params() const { return params_; }

    /// @brief Check if pressure advance is active.
    bool isActive() const { return params_.pressureAdvance > 0.0; }

    /// @brief Compute the pressure-adjusted extrusion for a move segment.
    /// @param startE Extrusion at start of segment (mm)
    /// @param endE Extrusion at end of segment (mm)
    /// @param startVel Velocity at start (mm/s)
    /// @param endVel Velocity at end (mm/s)
    /// @param dt Segment duration (seconds)
    /// @return Adjusted extrusion amount for this segment.
    double computeExtrusion(double startE, double endE,
                            double startVel, double endVel,
                            double dt) const {
        if (!isActive() || dt <= 0.0) return endE - startE;

        // PA adjusts the extrusion based on velocity change
        // The extra extrusion = PA * (endVel - startVel)
        double velChange = endVel - startVel;
        double paExtrusion = params_.pressureAdvance * velChange;
        double baseExtrusion = endE - startE;
        return baseExtrusion + paExtrusion;
    }

    /// @brief Compute the smoothed pressure advance over a window.
    /// @param extrusionRate Current extrusion rate (mm/s)
    /// @param time Current time
    /// @return Smoothed extrusion rate.
    double smoothExtrusionRate(double extrusionRate, double time) {
        if (params_.smoothTime <= 0.0) return extrusionRate;
        // Add to history
        history_.push_back({time, extrusionRate});
        // Remove old entries
        double cutoff = time - params_.smoothTime;
        while (!history_.empty() && history_.front().time < cutoff) {
            history_.pop_front();
        }
        // Average over the window
        double sum = 0.0;
        for (const auto& h : history_) sum += h.rate;
        return history_.empty() ? extrusionRate : sum / static_cast<double>(history_.size());
    }

    /// @brief Reset the smoothing history.
    void reset() { history_.clear(); }

private:
    struct HistoryEntry {
        double time;
        double rate;
    };
    PressureAdvanceParams params_;
    std::deque<HistoryEntry> history_;
};

} // namespace tether::klipper::klippy
