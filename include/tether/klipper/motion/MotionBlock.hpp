/**
 * @file MotionBlock.hpp
 * @brief Analyzable motion block emitted by the device for inspection.
 *
 * @details
 * A MotionBlock represents a decoded unit of motion as seen by the device:
 * a sequence of queue_step commands for a single stepper, plus the decoded
 * motion parameters (interval, count, add). The device emits these blocks
 * to a MotionBlockSink for analysis, visualization, and verification.
 *
 * In reconstruct+replan mode, the MotionBlock also carries the reconstructed
 * Tether MotionSegment(s) after re-feeding the steps through Tether's motion
 * subsystem.
 */

#pragma once

#include "tether/klipper/objects/Stepper.hpp"

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <optional>

namespace tether::klipper::motion {

/// @brief A decoded queue_step sequence for analysis.
struct MotionBlock {
    /// OID of the stepper this block targets.
    uint8_t oid = 0;
    /// Ordered step commands in this block.
    std::vector<objects::StepCommand> steps;
    /// Starting MCU clock for the first step.
    uint32_t startClock = 0;
    /// Total number of steps in this block.
    uint32_t totalSteps() const {
        uint32_t n = 0;
        for (const auto& s : steps) n += s.count;
        return n;
    }
    /// Total duration in clock ticks.
    uint32_t totalDuration() const {
        if (steps.empty()) return 0;
        uint32_t dur = 0;
        for (const auto& s : steps) {
            // For constant interval: dur += interval * count.
            // For accelerating (add != 0): approximate with first interval.
            // Exact duration = sum of interval + add*i for i in 0..count-1.
            // = count * interval + add * count*(count-1)/2
            int64_t interval = s.interval;
            int64_t count = s.count;
            int64_t add = s.add;
            int64_t seqDur = count * interval + add * count * (count - 1) / 2;
            dur += static_cast<uint32_t>(seqDur);
        }
        return dur;
    }
    /// Average step rate (steps per clock tick).
    double averageStepRate() const {
        uint32_t dur = totalDuration();
        if (dur == 0) return 0;
        return static_cast<double>(totalSteps()) / static_cast<double>(dur);
    }
    /// Source label (e.g. G-code line) for traceability.
    std::string sourceLabel;
    /// Timestamp when this block was recorded (host time).
    std::chrono::steady_clock::time_point recordTime;
};

} // namespace tether::klipper::motion
