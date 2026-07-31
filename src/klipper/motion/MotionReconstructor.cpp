/**
 * @file MotionReconstructor.cpp
 * @brief MotionReconstructor implementation.
 */

#include "tether/klipper/motion/MotionReconstructor.hpp"

namespace tether::klipper::motion {

std::vector<TrajectoryPoint> MotionReconstructor::reconstruct(
    const std::vector<objects::StepCommand>& steps,
    uint32_t startClock,
    double stepsPerMm) {

    std::vector<TrajectoryPoint> traj;
    uint32_t clock = startClock;
    double position = 0.0;
    uint32_t interval = 0;
    int32_t add = 0;

    for (const auto& cmd : steps) {
        interval = cmd.interval;
        add = cmd.add;
        for (uint16_t i = 0; i < cmd.count; ++i) {
            clock += interval;
            position += 1.0; // each step advances position by 1 (in steps)
            TrajectoryPoint p;
            p.clock = clock;
            p.position = position / stepsPerMm; // convert to mm
            p.velocity = (interval > 0) ? 1.0 / static_cast<double>(interval) : 0.0;
            traj.push_back(p);
            interval = static_cast<uint32_t>(static_cast<int32_t>(interval) + add);
        }
    }
    return traj;
}

MotionBlock MotionReconstructor::toMotionBlock(
    const std::vector<objects::StepCommand>& steps,
    uint8_t oid,
    uint32_t startClock,
    double stepsPerMm) {
    MotionBlock block;
    block.oid = oid;
    block.steps = steps;
    block.startClock = startClock;
    block.sourceLabel = "reconstructed";
    block.recordTime = std::chrono::steady_clock::now();
    (void)stepsPerMm; // used in reconstruct() if needed for analysis
    return block;
}

std::vector<double> MotionReconstructor::computeAcceleration(
    const std::vector<TrajectoryPoint>& traj) {
    std::vector<double> accel;
    if (traj.size() < 2) {
        accel.resize(traj.size(), 0.0);
        return accel;
    }
    accel.resize(traj.size());
    accel[0] = 0.0;
    for (size_t i = 1; i < traj.size(); ++i) {
        uint32_t dt = traj[i].clock - traj[i - 1].clock;
        if (dt == 0) {
            accel[i] = 0.0;
        } else {
            accel[i] = (traj[i].velocity - traj[i - 1].velocity) /
                       static_cast<double>(dt);
        }
    }
    return accel;
}

void MotionReconstructor::smoothVelocities(
    std::vector<TrajectoryPoint>& traj, size_t windowSize) {
    if (traj.size() < 3 || windowSize < 2) return;

    size_t half = windowSize / 2;
    std::vector<double> smoothed;
    smoothed.reserve(traj.size());

    for (size_t i = 0; i < traj.size(); ++i) {
        double sum = 0.0;
        size_t count = 0;
        for (size_t j = (i >= half ? i - half : 0);
             j <= std::min(i + half, traj.size() - 1); ++j) {
            sum += traj[j].velocity;
            ++count;
        }
        smoothed.push_back(sum / static_cast<double>(count));
    }

    for (size_t i = 0; i < traj.size(); ++i) {
        traj[i].velocity = smoothed[i];
    }
}

MotionReconstructor::TrajectoryStats MotionReconstructor::computeStats(
    const std::vector<TrajectoryPoint>& traj) {
    TrajectoryStats stats;
    if (traj.empty()) return stats;

    double velSum = 0.0;
    stats.maxVelocity = traj[0].velocity;
    stats.minVelocity = traj[0].velocity;

    for (const auto& p : traj) {
        if (p.velocity > stats.maxVelocity) stats.maxVelocity = p.velocity;
        if (p.velocity < stats.minVelocity) stats.minVelocity = p.velocity;
        velSum += p.velocity;
    }
    stats.avgVelocity = velSum / static_cast<double>(traj.size());

    // Compute acceleration
    auto accel = computeAcceleration(traj);
    double accelSum = 0.0;
    stats.maxAcceleration = accel.empty() ? 0.0 : accel[0];
    stats.minAcceleration = stats.maxAcceleration;
    for (double a : accel) {
        if (a > stats.maxAcceleration) stats.maxAcceleration = a;
        if (a < stats.minAcceleration) stats.minAcceleration = a;
        accelSum += a;
    }
    if (!accel.empty()) {
        stats.avgAcceleration = accelSum / static_cast<double>(accel.size());
    }

    // Total distance and time
    stats.totalDistance = traj.back().position - traj.front().position;
    stats.totalTime = static_cast<double>(traj.back().clock - traj.front().clock);

    return stats;
}

} // namespace tether::klipper::motion
