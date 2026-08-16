/**
 * @file AccelerationProfileAnalyzer.cpp
 * @brief Analyze acceleration profile and estimate acceleration-limited time.
 */
#include "tether/analysis/AccelerationProfileAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace tether::analysis {

AccelerationProfileResult AccelerationProfileAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    AccelerationProfileResult result;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;
    double prevSpeed = 0;
    double totalAcceleration = 0;
    double feedRate = 0;

    // For acceleration-limited time.
    double unlimitedTime = 0, limitedTime = 0;
    int directionChanges = 0, stops = 0;
    double totalDistance = 0;
    double prevDirX = 0, prevDirY = 0, prevDirZ = 0;
    bool hasPrevDir = false;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);
        if (!isMotion(m)) {
            prev.feedRate = m.feedRate;
            continue;
        }

        double dx = m.x - prev.x, dy = m.y - prev.y, dz = m.z - prev.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > 0.001 && m.feedRate > 0) {
            double targetSpeed = m.feedRate / 60.0;
            double unlimitedSegTime = dist / targetSpeed;
            unlimitedTime += unlimitedSegTime;

            // Direction change detection.
            if (hasPrevDir) {
                double dirLen = dist;
                double dirX = dx / dirLen, dirY = dy / dirLen, dirZ = dz / dirLen;
                double dot = prevDirX * dirX + prevDirY * dirY + prevDirZ * dirZ;
                if (dot < 0.5) {
                    directionChanges++;
                    prevSpeed = 0;
                }
                prevDirX = dirX; prevDirY = dirY; prevDirZ = dirZ;
            } else {
                prevDirX = dx / dist; prevDirY = dy / dist; prevDirZ = dz / dist;
                hasPrevDir = true;
            }

            // Acceleration-limited time (trapezoidal profile).
            double speedDelta = std::abs(targetSpeed - prevSpeed);
            double accelTime = speedDelta / params_.maxAccelMmPerS2;
            double accelDistance = prevSpeed * accelTime +
                0.5 * params_.maxAccelMmPerS2 * accelTime * accelTime;

            double limitedSegTime;
            if (accelDistance >= dist) {
                limitedSegTime = std::sqrt(2.0 * dist / params_.maxAccelMmPerS2);
                stops++;
            } else {
                double cruiseDistance = dist - accelDistance;
                double cruiseTime = cruiseDistance / targetSpeed;
                limitedSegTime = accelTime + cruiseTime + accelTime;
            }
            limitedTime += limitedSegTime;
            totalDistance += dist;
            prevSpeed = targetSpeed;

            // Sampled segments.
            if (i % params_.sampleInterval == 0) {
                result.segments.push_back({
                    i, unlimitedSegTime, limitedSegTime,
                    limitedSegTime > unlimitedSegTime * 1.1
                });
            }

            // Acceleration profile point.
            double acceleration = (unlimitedSegTime > 0)
                ? (targetSpeed - prevSpeed) / unlimitedSegTime : 0;
            // Recompute properly: use the speed delta from the *previous* move.
            double prevSpeedForAccel = (i > 0) ? prevSpeed : 0;
            // Actually we already updated prevSpeed, so we need to track separately.
            // Let's use the move time approach.
            double moveTime = dist / targetSpeed;
            double accel = (moveTime > 0) ? std::abs(targetSpeed - prevSpeed) / moveTime : 0;

            AccelerationPoint pt;
            pt.line = i;
            pt.acceleration = accel;
            pt.type = (std::abs(accel) > params_.maxAccelMmPerS2 * 0.8) ? AccelerationPoint::Type::Jerk
                : (accel > 10) ? AccelerationPoint::Type::Acceleration
                : (accel < -10) ? AccelerationPoint::Type::Deceleration
                : AccelerationPoint::Type::Steady;
            pt.magnitude = (std::abs(accel) < 100) ? AccelerationPoint::Magnitude::Low
                : (std::abs(accel) < 500) ? AccelerationPoint::Magnitude::Medium
                : (std::abs(accel) < params_.maxAccelMmPerS2) ? AccelerationPoint::Magnitude::High
                : AccelerationPoint::Magnitude::Extreme;
            result.points.push_back(pt);
            totalAcceleration += std::abs(accel);
        }

        prev = m;
    }

    // Profile statistics.
    if (!result.points.empty()) {
        result.avgAcceleration = totalAcceleration / result.points.size();
        result.maxAcceleration = 0;
        for (const auto& p : result.points)
            result.maxAcceleration = std::max(result.maxAcceleration, std::abs(p.acceleration));
        result.accelerationCount = static_cast<int>(std::count_if(
            result.points.begin(), result.points.end(),
            [](const AccelerationPoint& p) { return p.type == AccelerationPoint::Type::Acceleration; }));
        result.decelerationCount = static_cast<int>(std::count_if(
            result.points.begin(), result.points.end(),
            [](const AccelerationPoint& p) { return p.type == AccelerationPoint::Type::Deceleration; }));
        result.jerkCount = static_cast<int>(std::count_if(
            result.points.begin(), result.points.end(),
            [](const AccelerationPoint& p) { return p.type == AccelerationPoint::Type::Jerk; }));
        result.smoothnessScore = std::max(0.0,
            100.0 - result.jerkCount * 5.0 - (result.maxAcceleration / params_.maxAccelMmPerS2) * 30.0);
    }

    // Time estimation.
    result.limitedTime = limitedTime;
    result.unlimitedTime = unlimitedTime;
    result.accelerationOverhead = limitedTime - unlimitedTime;
    result.overheadPercentage = (unlimitedTime > 0)
        ? (result.accelerationOverhead / unlimitedTime) * 100.0 : 0;
    result.directionChanges = directionChanges;
    result.stops = stops;
    result.avgLimitedSpeed = (limitedTime > 0) ? (totalDistance / limitedTime) * 60.0 : 0;

    // Recommendations.
    result.recommendations.push_back(
        std::format("{} accel points, avg {:.0f}mm/s², max {:.0f}mm/s²",
                    result.points.size(), result.avgAcceleration, result.maxAcceleration));
    if (result.jerkCount > 10) {
        result.recommendations.push_back(
            std::format("{} jerk events — reduce acceleration", result.jerkCount));
    }
    if (result.overheadPercentage > 20) {
        result.recommendations.push_back(
            std::format("Accel overhead: {:.1f}% — short moves dominate", result.overheadPercentage));
    }
    if (result.smoothnessScore > 80) {
        result.recommendations.push_back("Smooth acceleration profile");
    }

    return result;
}

} // namespace tether::analysis
