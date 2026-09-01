#pragma once

/// @file HomingAxisSimulator.hpp
/// @brief Kinematic simulator for single-axis homing motion.
///
/// Simulates the physical motion of a single printer axis during G28 homing.
/// The axis uses a jerk-limited S-curve velocity profile for each phase:
/// acceleration, cruise, and deceleration to the endstop. An optional
/// second-pass (bounce) moves back and re-approaches at a slower speed for
/// precision.
///
/// This is a kinematic (velocity-profile) simulation, not a dynamical system
/// in the control-theory sense — it models position vs. time under a
/// 7-phase S-curve, which is how real stepper-driven axes behave during
/// controlled homing moves.
///
/// Used by the Klipper G28 handler to produce realistic position updates
/// during homing so that Mainsail/Fluidd see the toolhead move to the
/// endstop position over time, just like a real printer.

#include "tether/motion_planner/SCurveProfile.hpp"

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
/// Models the S-curve velocity profile motion of a stepper-driven axis
/// during G28 homing. The simulation proceeds in phases:
///
/// 1. **Seek**: S-curve from current position to the endstop, starting and
///    ending at rest.
/// 2. **Bounce** (if `secondHomingSpeed > 0`): S-curve back by
///    `bounceDistance`, then S-curve re-approach at `secondHomingSpeed`.
/// 3. **Settle**: Final position is `endstopPosition`.
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
        startPhase(config_.endstopPosition, config_.homingSpeed);
    }

    /// @brief Advance the simulation by dt seconds.
    /// @param dt Time step in seconds.
    /// @return Step result with current position, velocity, and flags.
    HomingStepResult step(double dt) {
        if (complete_ || phase_ == Phase::Idle || phase_ == Phase::Settled) {
            return {currentPosition_, currentVelocity_, elapsedTime_,
                    endstopTriggered_, complete_};
        }

        elapsedTime_ += dt;
        phaseElapsed_ += dt;

        if (phaseElapsed_ >= profile_.totalDuration()) {
            currentPosition_ = phaseTarget_;
            currentVelocity_ = 0.0;
            finishPhase();
        } else {
            auto state = profile_.evaluateAt(phaseElapsed_);
            currentPosition_ = phaseStartPos_ + state.position * phaseDirection_;
            currentVelocity_ = state.velocity * phaseDirection_;
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
    double phaseStartPos_ = 0.0;
    double phaseTarget_ = 0.0;
    double phaseDirection_ = 1.0;
    double phaseElapsed_ = 0.0;
    Phase phase_ = Phase::Idle;
    bool endstopTriggered_ = false;
    bool complete_ = false;
    MotionPlanner::SCurveProfile<double> profile_;

    /// @brief Set up an S-curve profile for the current phase.
    /// @param target Position to reach in this phase.
    /// @param maxVelocity Maximum cruise velocity for this phase.
    void startPhase(double target, double maxVelocity) {
        phaseStartPos_ = currentPosition_;
        phaseTarget_ = target;
        phaseDirection_ = (target >= currentPosition_) ? 1.0 : -1.0;
        double distance = std::abs(target - currentPosition_);
        phaseElapsed_ = 0.0;

        if (distance < 1e-12 || maxVelocity <= 0.0) {
            // Trivial move — finish this phase immediately.
            currentPosition_ = target;
            currentVelocity_ = 0.0;
            finishPhase();
            return;
        }

        MotionPlanner::SCurveConstraints<double> constraints;
        constraints.maxVelocity = maxVelocity;
        constraints.maxAcceleration = config_.acceleration;
        constraints.maxJerk = config_.acceleration * 10.0;

        bool ok = profile_.compute(distance, 0.0, 0.0, constraints);
        if (!ok) {
            // Fall back to an instant jump if the profile cannot be computed.
            currentPosition_ = target;
            currentVelocity_ = 0.0;
            finishPhase();
        }
    }

    /// @brief Advance to the next phase or mark homing complete.
    void finishPhase() {
        switch (phase_) {
            case Phase::Seeking:
                endstopTriggered_ = true;
                if (config_.secondHomingSpeed > 0.0 && config_.bounceDistance > 0.0) {
                    phase_ = Phase::Bouncing;
                    startPhase(config_.endstopPosition - phaseDirection_ * config_.bounceDistance,
                               config_.homingSpeed);
                } else {
                    phase_ = Phase::Settled;
                    complete_ = true;
                }
                break;
            case Phase::Bouncing:
                phase_ = Phase::ReApproaching;
                startPhase(config_.endstopPosition, config_.secondHomingSpeed);
                break;
            case Phase::ReApproaching:
                phase_ = Phase::Settled;
                complete_ = true;
                break;
            default:
                break;
        }
    }
};

/// @brief Multi-axis homing coordinator.
///
/// Homes axes sequentially (X, then Y, then Z) using HomingAxisSimulator
/// instances. This mirrors how Klipper homes axes one time by default
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
