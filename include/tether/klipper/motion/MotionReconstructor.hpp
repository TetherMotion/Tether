/**
 * @file MotionReconstructor.hpp
 * @brief Reconstruct Tether motion segments from queue_step sequences.
 *
 * @details
 * In reconstruct+replan mode, the device decodes incoming queue_step
 * sequences and feeds them back into Tether's motion subsystem for analysis
 * and re-planning. The MotionReconstructor converts a sequence of StepCommand
 * entries into a time-position series, which can then be used to build
 * Tether MotionSegments or a MotionPlan for analysis.
 *
 * The reconstruction computes, for each step, the clock time and the
 * cumulative position, producing a sampled trajectory. This trajectory can
 * be compared against the original Tether MotionPlan (on the host side) to
 * verify the translation, or re-planned through Tether's motion subsystem
 * for smoother execution.
 */

#pragma once

#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/motion/MotionBlock.hpp"

#include <cstdint>
#include <vector>
#include <cmath>

namespace tether::klipper::motion {

/// @brief A sampled point in the reconstructed trajectory.
struct TrajectoryPoint {
    uint32_t clock = 0;       ///< MCU clock at this point
    double position = 0;     ///< Cumulative position (steps)
    double velocity = 0;     ///< Step rate (steps per clock tick)
    double acceleration = 0; ///< Acceleration (steps per clock^2)
};

/**
 * @brief Reconstruct a trajectory from a queue_step sequence.
 */
class MotionReconstructor {
public:
    /**
     * @brief Reconstruct a trajectory from step commands.
     * @param steps The step commands to reconstruct.
     * @param startClock The MCU clock at which the sequence begins.
     * @param stepsPerMm Steps per millimeter (to convert to mm).
     * @return Sampled trajectory points (one per step).
     */
    static std::vector<TrajectoryPoint> reconstruct(
        const std::vector<objects::StepCommand>& steps,
        uint32_t startClock,
        double stepsPerMm = 1.0);

    /**
     * @brief Reconstruct and produce a MotionBlock for analysis.
     * @param steps The step commands.
     * @param oid The stepper OID.
     * @param startClock The starting MCU clock.
     * @param stepsPerMm Steps per millimeter.
     * @return A MotionBlock with the decoded steps and trajectory info.
     */
    static MotionBlock toMotionBlock(
        const std::vector<objects::StepCommand>& steps,
        uint8_t oid,
        uint32_t startClock,
        double stepsPerMm = 1.0);

    /**
     * @brief Compute acceleration for each trajectory point.
     * @param traj The trajectory points (must have velocity computed).
     * @return A vector of acceleration values (steps per clock^2).
     */
    static std::vector<double> computeAcceleration(
        const std::vector<TrajectoryPoint>& traj);

    /**
     * @brief Apply a moving-average smoothing to trajectory velocities.
     * @param traj The trajectory points to smooth in-place.
     * @param windowSize The smoothing window size (number of points).
     */
    static void smoothVelocities(std::vector<TrajectoryPoint>& traj,
                                  size_t windowSize = 5);

    /**
     * @brief Compute summary statistics for a trajectory.
     * @param traj The trajectory points.
     * @return A struct with max/min/avg velocity and acceleration.
     */
    struct TrajectoryStats {
        double maxVelocity = 0.0;
        double minVelocity = 0.0;
        double avgVelocity = 0.0;
        double maxAcceleration = 0.0;
        double minAcceleration = 0.0;
        double avgAcceleration = 0.0;
        double totalDistance = 0.0;
        double totalTime = 0.0;
    };

    static TrajectoryStats computeStats(const std::vector<TrajectoryPoint>& traj);
};

} // namespace tether::klipper::motion
