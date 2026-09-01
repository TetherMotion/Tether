#pragma once

/// @file HomingAxisSimulator.hpp
/// @brief Kinematic simulator for single-axis homing motion.
///
/// Simulates the physical motion of a single printer axis during G28 homing.
/// The axis accelerates from rest to homing speed, cruises toward the endstop
/// position, and stops when the endstop is reached. An optional second-pass
/// (bounce) moves back and re-approaches at a slower speed for precision.
///
/// This is a kinematic (velocity-profile) simulation, not a dynamical system
/// in the control-theory sense — it models position vs. time under a
/// trapezoidal velocity profile with constant acceleration, which is how
/// real stepper-driven axes behave during homing.
///
/// Used by the Klipper G28 handler to produce realistic position updates
/// during homing so that Mainsail/Fluidd see the toolhead move to the
/// endstop position over time, just like a real printer.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace Simulation {

/// @brief Configuration for homing a single axis.
struct HomingAxisConfig {
    std::string name;                ///< Axis name: "x", "y", "z"
    double endstopPosition = 0.0;    ///< Position where the endstop triggers (position_endstop)
    double homingSpeed = 50.0;       ///< Search speed in mm/s
    double secondHomingSpeed = 0.0;  ///< Second pass speed (0 = single pass)
    double bounceDistance = 5.0;     ///< Back-off distance after first trigger (mm)
    double acceleration = 2000.0;    ///< Axis acceleration in mm/s^2
    bool positiveDirection = true;   ///< True = home toward max, false = toward min
    double positionMin = -std::numeric_limits<double>::infinity();
    double positionMax = std::numeric_limits<double>::infinity();
};

/// @brief Result of a homing simulation step.
struct HomingStepResult {
    double position = 0.0;           ///< Current axis position (mm)
    double velocity = 0.0;           ///< Current velocity (mm/s)
    double elapsedTime = 0.0;        ///< Total elapsed time (s)
    bool endstopTriggered = false;   ///< True when endstop reached
    bool complete = false;           ///< True when homing sequence is done
};

/// @brief Kinematic simulator for single-axis homing.
///
/// Models the trapezoidal velocity profile motion of a stepper-driven axis
/// during G28 homing. The simulation proceeds in phases:
///
/// 1. **Seek**: Accelerate from current position toward the endstop at
///    `homingSpeed`. Stop when position reaches `endstopPosition`.
/// 2. **Bounce** (if `secondHomingSpeed > 0`): Move back by `bounceDistance`,
///    then re-approach at `secondHomingSpeed`.
/// 3. **Settle**: Set final position to `endstopPosition`.
///
/// The simulator is stepped forward in time with `step(dt)`. Each step
/// returns the current position, velocity, and whether the endstop has
/// been triggered.
class HomingAxisSimulator {
public:
    enum class Phase {
        Idle,       ///< Not started
        Seeking,    ///< Moving toward endstop at homing speed
        Bouncing,   ///< Backing off after first trigger
        ReApproaching, ///< Re-approaching at second homing speed
        Settled,    ///< Position set, homing complete
    };

    explicit HomingAxisSimulator(const HomingAxisConfig& config)
        : config_(config) {}

    /// @brief Start homing from the given initial position.
    void start(double initialPosition) {
        currentPosition_ = initialPosition;
        currentVelocity_ = 0.0;
        elapsedTime_ = 0.0;
        phase_ = Phase::Seeking;
        endstopTriggered_ = false;
        complete_ = false;
        // Compute direction toward endstop
        direction_ = (config_.endstopPosition >= initialPosition) ? 1.0 : -1.0;
        // If positiveDirection is false, we home toward min (endstop at min)
        if (!config_.positiveDirection) {
            direction_ = -1.0;
        }
    }

    /// @brief Advance the simulation by dt seconds.
    /// @param dt Time step in seconds.
    /// @return Step result with current position, velocity, and flags.
    HomingStepResult step(double dt) {
        if (complete_ || phase_ == Phase::Idle) {
            return {currentPosition_, currentVelocity_, elapsedTime_,
                    endstopTriggered_, complete_};
        }

        elapsedTime_ += dt;

        switch (phase_) {
            case Phase::Seeking:
                stepSeeking(dt);
                break;
            case Phase::Bouncing:
                stepBouncing(dt);
                break;
            case Phase::ReApproaching:
                stepReApproaching(dt);
                break;
            default:
                break;
        }

        return {currentPosition_, currentVelocity_, elapsedTime_,
                endstopTriggered_, complete_};
    }

    /// @brief Run the full homing sequence to completion.
    /// @param initialPosition Starting position (mm).
    /// @param dt Time step for simulation (s).
    /// @return Final position after homing.
    double run(double initialPosition, double dt = 0.001) {
        start(initialPosition);
        while (!complete_) {
            step(dt);
            if (elapsedTime_ > 60.0) break; // Safety timeout
        }
        return currentPosition_;
    }

    /// @brief Get the current phase.
    Phase phase() const { return phase_; }

    /// @brief Get the current position.
    double position() const { return currentPosition_; }

    /// @brief Get the current velocity.
    double velocity() const { return currentVelocity_; }

    /// @brief Get elapsed time.
    double elapsedTime() const { return elapsedTime_; }

    /// @brief Check if homing is complete.
    bool isComplete() const { return complete_; }

    /// @brief Check if endstop was triggered.
    bool endstopTriggered() const { return endstopTriggered_; }

    /// @brief Get the phase as a string (for debugging/status reporting).
    const char* phaseString() const {
        switch (phase_) {
            case Phase::Idle:          return "idle";
            case Phase::Seeking:       return "seeking";
            case Phase::Bouncing:      return "bouncing";
            case Phase::ReApproaching: return "re_approaching";
            case Phase::Settled:       return "settled";
        }
        return "unknown";
    }

private:
    HomingAxisConfig config_;
    double currentPosition_ = 0.0;
    double currentVelocity_ = 0.0;
    double elapsedTime_ = 0.0;
    double direction_ = 1.0;
    Phase phase_ = Phase::Idle;
    bool endstopTriggered_ = false;
    bool complete_ = false;

    void stepSeeking(double dt) {
        // Accelerate toward target speed
        double targetSpeed = config_.homingSpeed * direction_;
        double dv = config_.acceleration * dt;
        if (std::abs(currentVelocity_) < std::abs(targetSpeed)) {
            currentVelocity_ += direction_ * dv;
            if (std::abs(currentVelocity_) > std::abs(targetSpeed)) {
                currentVelocity_ = targetSpeed;
            }
        }

        // Move
        currentPosition_ += currentVelocity_ * dt;

        // Check if we've reached the endstop position
        double distToEndstop = (config_.endstopPosition - currentPosition_) * direction_;
        if (distToEndstop <= 0.0) {
            currentPosition_ = config_.endstopPosition;
            currentVelocity_ = 0.0;
            endstopTriggered_ = true;

            if (config_.secondHomingSpeed > 0.0 && config_.bounceDistance > 0.0) {
                // Start bounce phase
                phase_ = Phase::Bouncing;
            } else {
                // Single pass — done
                phase_ = Phase::Settled;
                complete_ = true;
            }
        }
    }

    void stepBouncing(double dt) {
        // Move away from endstop by bounceDistance
        double bounceTarget = config_.endstopPosition - direction_ * config_.bounceDistance;
        double targetSpeed = config_.homingSpeed * (-direction_); // reverse direction

        // Accelerate to reverse speed
        double dv = config_.acceleration * dt;
        if (std::abs(currentVelocity_) < std::abs(targetSpeed)) {
            currentVelocity_ += (-direction_) * dv;
            if (std::abs(currentVelocity_) > std::abs(targetSpeed)) {
                currentVelocity_ = targetSpeed;
            }
        }

        currentPosition_ += currentVelocity_ * dt;

        // Check if we've moved far enough
        double distBounced = (currentPosition_ - config_.endstopPosition) * (-direction_);
        if (distBounced >= config_.bounceDistance) {
            currentPosition_ = bounceTarget;
            currentVelocity_ = 0.0;
            phase_ = Phase::ReApproaching;
        }
    }

    void stepReApproaching(double dt) {
        // Re-approach endstop at second homing speed
        double targetSpeed = config_.secondHomingSpeed * direction_;
        double dv = config_.acceleration * dt;
        if (std::abs(currentVelocity_) < std::abs(targetSpeed)) {
            currentVelocity_ += direction_ * dv;
            if (std::abs(currentVelocity_) > std::abs(targetSpeed)) {
                currentVelocity_ = targetSpeed;
            }
        }

        currentPosition_ += currentVelocity_ * dt;

        // Check if we've reached the endstop
        double distToEndstop = (config_.endstopPosition - currentPosition_) * direction_;
        if (distToEndstop <= 0.0) {
            currentPosition_ = config_.endstopPosition;
            currentVelocity_ = 0.0;
            phase_ = Phase::Settled;
            complete_ = true;
        }
    }
};

/// @brief Multi-axis homing coordinator.
///
/// Homes axes sequentially (X, then Y, then Z) using HomingAxisSimulator
/// instances. This mirrors how Klipper homes axes one at a time by default
/// (unless multi-axis homing is configured).
class HomingCoordinator {
public:
    /// @brief Add an axis to be homed.
    void addAxis(const HomingAxisConfig& config) {
        axes_.push_back(config);
    }

    /// @brief Run homing for all axes sequentially.
    /// @param initialPositions Map of axis name -> initial position (mm).
    /// @param dt Simulation time step (s).
    /// @return Map of axis name -> final homed position.
    std::map<std::string, double> run(
        const std::map<std::string, double>& initialPositions,
        double dt = 0.001) {

        std::map<std::string, double> finalPositions;
        for (const auto& cfg : axes_) {
            double initPos = 0.0;
            auto it = initialPositions.find(cfg.name);
            if (it != initialPositions.end()) initPos = it->second;

            HomingAxisSimulator sim(cfg);
            double finalPos = sim.run(initPos, dt);
            finalPositions[cfg.name] = finalPos;
        }
        return finalPositions;
    }

    /// @brief Get the list of axis configs.
    const std::vector<HomingAxisConfig>& axes() const { return axes_; }

private:
    std::vector<HomingAxisConfig> axes_;
};

} // namespace Simulation
