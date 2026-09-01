#pragma once

/// @file LinearMoveSimulator.hpp
/// @brief S-curve-based linear move simulator for the emulated printer.
///
/// Replaces the previous "jump to target" behavior in the Klipper emulation
/// with a full 7-phase S-curve velocity profile.  The toolhead is animated
/// from the current position to the requested target with jerk-limited
/// acceleration and deceleration, which is what Mainsail/Fluidd display
/// when connected to a real printer.

#include "tether/motion_planner/SCurveProfile.hpp"

#include <array>
#include <cmath>
#include <limits>

namespace Simulation {

/// @brief Result of one step of a linear move simulation.
struct LinearMoveStepResult {
    std::array<double, 4> position = {0, 0, 0, 0};
    double velocity = 0.0;
    double elapsedTime = 0.0;
    bool complete = false;
};

/// @brief S-curve planner for a point-to-point 4D move.
///
/// Builds a 1D S-curve along the straight-line path from start to target,
/// then interpolates the X/Y/Z/E components proportionally.  Start and end
/// velocity are zero (each move is treated as a standalone point-to-point
/// segment).
class LinearMoveSimulator {
public:
    /// @brief Configure and start a linear move.
    /// @param start Starting position (X, Y, Z, E) in machine coordinates.
    /// @param end Target position (X, Y, Z, E) in machine coordinates.
    /// @param maxVelocity Requested cruise velocity (mm/s).
    /// @param maxAcceleration Maximum acceleration (mm/s^2).
    /// @param maxJerk Maximum jerk (mm/s^3).
    void start(const std::array<double, 4>& start,
               const std::array<double, 4>& end,
               double maxVelocity,
               double maxAcceleration,
               double maxJerk) {
        start_ = start;
        end_ = end;
        current_ = start;
        elapsedTime_ = 0.0;
        velocity_ = 0.0;
        complete_ = false;

        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double dz = end[2] - start[2];
        double de = end[3] - start[3];
        distance_ = std::sqrt(dx * dx + dy * dy + dz * dz + de * de);

        if (distance_ < 1e-12 || maxVelocity <= 0.0 || maxAcceleration <= 0.0 ||
            maxJerk <= 0.0) {
            // Trivial or invalid move — go straight to the target.
            complete_ = true;
            current_ = end;
            return;
        }

        MotionPlanner::SCurveConstraints<double> constraints;
        constraints.maxVelocity = maxVelocity;
        constraints.maxAcceleration = maxAcceleration;
        constraints.maxJerk = maxJerk;

        bool ok = profile_.compute(distance_, 0.0, 0.0, constraints);
        if (!ok) {
            complete_ = true;
            current_ = end;
            return;
        }
    }

    /// @brief Advance the move by dt seconds.
    LinearMoveStepResult step(double dt) {
        if (complete_ || dt <= 0.0) {
            return {current_, velocity_, elapsedTime_, complete_};
        }

        elapsedTime_ += dt;

        if (elapsedTime_ >= profile_.totalDuration()) {
            complete_ = true;
            current_ = end_;
            velocity_ = 0.0;
        } else {
            auto state = profile_.evaluateAt(elapsedTime_);
            double alpha = (distance_ > 1e-12) ? state.position / distance_ : 1.0;
            for (size_t i = 0; i < 4; ++i) {
                current_[i] = start_[i] + alpha * (end_[i] - start_[i]);
            }
            velocity_ = state.velocity;
        }

        return {current_, velocity_, elapsedTime_, complete_};
    }

    /// @brief Jump to the final position and mark complete.
    void finish() {
        complete_ = true;
        current_ = end_;
        velocity_ = 0.0;
    }

    /// @brief Check if the move has reached the target.
    bool isComplete() const { return complete_; }

    /// @brief Current position.
    const std::array<double, 4>& position() const { return current_; }

    /// @brief Current path velocity.
    double velocity() const { return velocity_; }

    /// @brief Elapsed time since start.
    double elapsedTime() const { return elapsedTime_; }

private:
    std::array<double, 4> start_ = {0, 0, 0, 0};
    std::array<double, 4> end_ = {0, 0, 0, 0};
    std::array<double, 4> current_ = {0, 0, 0, 0};
    double distance_ = 0.0;
    double elapsedTime_ = 0.0;
    double velocity_ = 0.0;
    bool complete_ = false;
    MotionPlanner::SCurveProfile<double> profile_;
};

} // namespace Simulation
