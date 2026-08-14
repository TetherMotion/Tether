/**
 * @file MotionTranslator.hpp
 * @brief Translate a Tether MotionPlan into Klipper queue_step sequences.
 *
 * @details
 * The MotionTranslator samples a Tether MotionPlan at regular intervals and
 * converts the per-axis position into step counts for each stepper. For each
 * axis, it computes the step interval, count, and add (acceleration delta)
 * that best reproduce the planned motion.
 *
 * The translation process:
 *   1. Sample the MotionPlan at the planner's sample interval.
 *   2. For each axis, compute the cumulative step count (position * stepsPerMm).
 *   3. Group consecutive steps into queue_step commands with constant or
 *      linearly-changing interval (acceleration via `add`).
 *   4. Emit StepCommand sequences per axis, tagged with the starting MCU clock.
 *
 * This replaces Klipper's motion kernel entirely: Tether's MotionPlanner does
 * all the trajectory generation; the translator just discretizes it into the
 * Klipper wire format.
 */

#pragma once

#include "tether/klipper/objects/Stepper.hpp"
#include "tether/kinematics/KinematicsTransform.hpp"
#include "tether/motion_planner/MotionPlan.hpp"

#if TETHER_ENABLE_PRESSURE_ADVANCE
#include "tether/klipper/klippy/PressureAdvance.hpp"
#endif

#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include <cmath>
#include <algorithm>

namespace tether::klipper::motion {

/// @brief Per-axis translation config.
struct AxisConfig {
    /// Steps per millimeter for this axis.
    double stepsPerMm = 80.0;
    /// Direction invert (true = reverse direction).
    bool invertDirection = false;
};

#if TETHER_ENABLE_PRESSURE_ADVANCE
/// @brief Pressure advance configuration for the motion translator.
///
/// PA is applied only when `enabled` is true AND `pressureAdvance > 0`.
/// This allows compile-time inclusion (TETHER_ENABLE_PRESSURE_ADVANCE=ON)
/// with runtime opt-in via config or G-code.
struct PressureAdvanceConfig {
    bool enabled = false;            ///< Runtime enable/disable (default: off)
    double pressureAdvance = 0.0;    ///< PA amount in seconds
    double smoothTime = 0.0;         ///< Smoothing window in seconds (0 = no smoothing)
    size_t extruderAxis = 3;         ///< Axis index of the extruder (typically 3 = E)
};
#endif

/// @brief A translated step sequence for one axis.
struct AxisStepSequence {
    uint8_t oid = 0;
    uint32_t startClock = 0;
    std::vector<objects::StepCommand> steps;
};

// KinematicsTransform has been extracted to
// tether/kinematics/KinematicsTransform.hpp (in the tether_kinematics module).
// It is included above. A using-alias keeps existing code that referenced
// `tether::klipper::motion::KinematicsTransform` compiling without changes.
using KinematicsTransform = ::tether::kinematics::KinematicsTransform;

/**
 * @brief Translate a Tether MotionPlan into Klipper queue_step sequences.
 *
 * @tparam Dim Dimensionality of the MotionPlan.
 * @tparam T   Numeric type (usually double).
 */
template<size_t Dim, typename T = double>
class MotionTranslator {
public:
    /// @brief Construct with per-axis configs and OIDs.
    MotionTranslator(std::array<AxisConfig, Dim> axisConfigs,
                      std::array<uint8_t, Dim> axisOids)
        : axisConfigs_(std::move(axisConfigs)), axisOids_(std::move(axisOids)) {}

    /**
     * @brief Translate a MotionPlan into per-axis step sequences.
     *
     * @param plan The Tether MotionPlan to translate.
     * @param clockFreqHz MCU clock frequency (for converting seconds to ticks).
     * @param sampleIntervalSec Sample interval for discretization (seconds).
     * @param startClock The MCU clock at which the motion should begin.
     * @return Per-axis step sequences.
     */
    std::vector<AxisStepSequence> translate(
        const MotionPlanner::MotionPlan<Dim, T>& plan,
        uint32_t clockFreqHz,
        double sampleIntervalSec,
        uint32_t startClock) const;

    /// @brief Set the source label for emitted blocks (traceability).
    void setSourceLabel(std::string label) { sourceLabel_ = std::move(label); }

    /// @brief Get the source label.
    const std::string& sourceLabel() const { return sourceLabel_; }

    /// @brief Set the kinematics transform.
    void setKinematicsTransform(const KinematicsTransform& kt) { kinematics_ = kt; }

    /// @brief Get the kinematics transform.
    const KinematicsTransform& kinematicsTransform() const { return kinematics_; }

#if TETHER_ENABLE_PRESSURE_ADVANCE
    /// @brief Set pressure advance configuration for extruder step generation.
    void setPressureAdvanceConfig(const PressureAdvanceConfig& pa) { paConfig_ = pa; }

    /// @brief Get the current pressure advance configuration.
    const PressureAdvanceConfig& pressureAdvanceConfig() const { return paConfig_; }
#endif

private:
    std::array<AxisConfig, Dim> axisConfigs_;
    std::array<uint8_t, Dim> axisOids_;
    std::string sourceLabel_;
    KinematicsTransform kinematics_;
#if TETHER_ENABLE_PRESSURE_ADVANCE
    PressureAdvanceConfig paConfig_{};
#endif
};

// ---------------------------------------------------------------------------
// Implementation (template, header-only)
// ---------------------------------------------------------------------------

template<size_t Dim, typename T>
std::vector<AxisStepSequence> MotionTranslator<Dim, T>::translate(
    const MotionPlanner::MotionPlan<Dim, T>& plan,
    uint32_t clockFreqHz,
    double sampleIntervalSec,
    uint32_t startClock) const {

    std::vector<AxisStepSequence> result;
    if (plan.totalDuration() <= T(0)) return result;

    // Sample the plan and compute cumulative steps per axis.
    // We sample at regular intervals and compute the step count for each
    // sample. Then we group consecutive steps into queue_step commands.
    // For acceleration/deceleration, we compute the per-step interval
    // change and encode it as the `add` parameter.

    double totalSec = static_cast<double>(plan.totalDuration());
    int numSamples = std::max(1, static_cast<int>(totalSec / sampleIntervalSec));
    uint32_t intervalTicks = static_cast<uint32_t>(sampleIntervalSec * clockFreqHz);

    // Per-axis cumulative step positions and velocities at each sample.
    std::array<std::vector<int64_t>, Dim> axisSteps;
    std::array<std::vector<double>, Dim> axisVel;
    for (auto& v : axisSteps) v.resize(numSamples + 1, 0);
    for (auto& v : axisVel) v.resize(numSamples + 1, 0.0);

    for (int i = 0; i <= numSamples; ++i) {
        double t = std::min(static_cast<double>(i) * sampleIntervalSec, totalSec);
        auto state = plan.evaluateAt(static_cast<T>(t));
        // Extract Cartesian position (first 3 axes: X, Y, Z)
        double cartX = (Dim > 0) ? static_cast<double>(state.position[0]) : 0.0;
        double cartY = (Dim > 1) ? static_cast<double>(state.position[1]) : 0.0;
        double cartZ = (Dim > 2) ? static_cast<double>(state.position[2]) : 0.0;
        // Apply kinematics transform to get stepper-space positions
        auto stepperPos = kinematics_.forwardActuatorKinematics(cartX, cartY, cartZ);
        // Extract velocities for kinematics transform (approximate)
        double velX = (Dim > 0) ? static_cast<double>(state.velocity[0]) : 0.0;
        double velY = (Dim > 1) ? static_cast<double>(state.velocity[1]) : 0.0;
        double velZ = (Dim > 2) ? static_cast<double>(state.velocity[2]) : 0.0;
        // For velocity, use the Jacobian approximation:
        // For CoreXY: dA/dt = dX/dt + dY/dt, dB/dt = dX/dt - dY/dt
        // For Cartesian: identity
        std::array<double, 3> stepperVel;
        switch (kinematics_.kinematics()) {
            case ::tether::kinematics::PrinterKinematics::CoreXY:
                stepperVel = {velX + velY, velX - velY, velZ};
                break;
            case ::tether::kinematics::PrinterKinematics::CoreXZ:
                stepperVel = {velX + velZ, velX - velZ, velY};
                break;
            case ::tether::kinematics::PrinterKinematics::CoreYZ:
                stepperVel = {velY + velZ, velY - velZ, velX};
                break;
            default:
                // For Cartesian, Delta, and others: approximate with Cartesian velocity
                stepperVel = {velX, velY, velZ};
                break;
        }
        for (size_t axis = 0; axis < Dim; ++axis) {
            double pos, vel;
            if (axis < 3) {
                pos = stepperPos[axis];
                vel = stepperVel[axis];
            } else {
                // Extra axes (e.g., extruder) use Cartesian directly
                pos = static_cast<double>(state.position[axis]);
                vel = static_cast<double>(state.velocity[axis]);
            }
            double steps = pos * axisConfigs_[axis].stepsPerMm;
            if (axisConfigs_[axis].invertDirection) {
                steps = -steps;
                vel = -vel;
            }
            axisSteps[axis][i] = static_cast<int64_t>(std::round(steps));
            axisVel[axis][i] = vel * axisConfigs_[axis].stepsPerMm; // steps/sec
        }
    }

#if TETHER_ENABLE_PRESSURE_ADVANCE
    // Apply pressure advance to the extruder axis.
    //
    // The PA-adjusted position is:  adjusted_e = e + PA * smoothed_velocity
    //
    // We smooth the extruder velocity with a centered moving average over the
    // smoothTime window, then recompute the E-axis step positions. The step
    // deltas (computed later) naturally become:  Δe + PA * Δv, which is the
    // correct PA adjustment per segment.
    if (paConfig_.enabled && paConfig_.pressureAdvance > 0.0 && paConfig_.extruderAxis < Dim) {
        const size_t eAxis = paConfig_.extruderAxis;
        const double pa = paConfig_.pressureAdvance;
        const double spm = axisConfigs_[eAxis].stepsPerMm;

        // Recover raw E positions in mm (axisSteps already has invertDirection
        // applied, so the PA offset is computed in the same frame).
        std::vector<double> rawPosMm(numSamples + 1);
        for (int i = 0; i <= numSamples; ++i)
            rawPosMm[i] = static_cast<double>(axisSteps[eAxis][i]) / spm;

        // Smooth velocity (mm/s) with a centered moving average.
        std::vector<double> smoothedVelMm(numSamples + 1, 0.0);
        if (paConfig_.smoothTime > 0.0) {
            int halfWindow = std::max(1,
                static_cast<int>(paConfig_.smoothTime / sampleIntervalSec / 2.0));
            for (int i = 0; i <= numSamples; ++i) {
                double sum = 0.0;
                int count = 0;
                for (int j = std::max(0, i - halfWindow);
                     j <= std::min(numSamples, i + halfWindow); ++j) {
                    sum += axisVel[eAxis][j] / spm; // steps/sec → mm/s
                    ++count;
                }
                smoothedVelMm[i] = (count > 0) ? sum / count : 0.0;
            }
        } else {
            for (int i = 0; i <= numSamples; ++i)
                smoothedVelMm[i] = axisVel[eAxis][i] / spm;
        }

        // Recompute E-axis step positions with PA offset.
        for (int i = 0; i <= numSamples; ++i) {
            double adjustedPos = rawPosMm[i] + pa * smoothedVelMm[i];
            axisSteps[eAxis][i] = static_cast<int64_t>(
                std::round(adjustedPos * spm));
        }
    }
#endif

    // Build step sequences per axis with acceleration grouping.
    // For each axis, we compute per-step intervals from the velocity
    // and group them into queue_step commands. When the velocity is
    // changing linearly (acceleration), we encode the interval change
    // as the `add` parameter.
    //
    // Edge cases handled:
    //   - Zero velocity: falls back to sample interval
    //   - Direction reversal: splits group at reversal boundary
    //   - Overflow protection: clamps interval and add to valid ranges
    //   - Very high acceleration: limits group size to keep add in range
    //   - Minimum interval: clamped to 1 tick

    // Klipper wire format limits:
    //   interval: uint32_t (but practically limited by clock freq)
    //   count: uint16_t (max 65535)
    //   add: int32_t (but Klipper uses int16_t in practice)
    constexpr int32_t ADD_MAX = 32767;
    constexpr int32_t ADD_MIN = -32768;
    constexpr uint32_t INTERVAL_MAX = 0x7FFFFFFF; // Avoid uint32 overflow in add computation

    for (size_t axis = 0; axis < Dim; ++axis) {
        AxisStepSequence seq;
        seq.oid = axisOids_[axis];
        seq.startClock = startClock;

        // Build raw per-sample step deltas and per-step intervals.
        struct RawStep {
            int64_t delta;
            uint32_t interval;  // ticks per step in this sample
        };
        std::vector<RawStep> rawSteps;
        int64_t prevSteps = axisSteps[axis][0]; // start from initial position
        for (int i = 0; i < numSamples; ++i) {
            int64_t curSteps = axisSteps[axis][i + 1];
            int64_t delta = curSteps - prevSteps;
            if (delta != 0) {
                // Compute per-step interval from velocity
                // velocity in steps/sec -> interval = clockFreq / velocity
                double avgVel = (axisVel[axis][i] + axisVel[axis][i + 1]) / 2.0;
                uint32_t stepInterval;
                if (std::abs(avgVel) > 1e-9) {
                    double ticks = static_cast<double>(clockFreqHz) / std::abs(avgVel);
                    // Clamp to prevent overflow
                    if (ticks > static_cast<double>(INTERVAL_MAX))
                        stepInterval = INTERVAL_MAX;
                    else if (ticks < 1.0)
                        stepInterval = 1;
                    else
                        stepInterval = static_cast<uint32_t>(ticks);
                } else {
                    stepInterval = intervalTicks;
                }
                // Ensure minimum interval of 1 tick
                if (stepInterval == 0) stepInterval = 1;
                rawSteps.push_back({delta, stepInterval});
            }
            prevSteps = curSteps;
        }

        // Group consecutive steps with the same delta sign and linearly
        // changing interval into a single StepCommand with `add`.
        // Split groups at direction reversals or when add would overflow.
        //
        // Each raw step carries `delta` (signed step count for one sample
        // interval) and `interval` (per-step ticks within that sample). When
        // merging samples, the queue_step `count` is the TOTAL number of
        // steps (sum of |delta|), not the number of merged samples.
        size_t idx = 0;
        while (idx < rawSteps.size()) {
            int64_t delta = rawSteps[idx].delta;
            int64_t deltaSign = (delta > 0) ? 1 : -1;
            uint32_t baseInterval = rawSteps[idx].interval;
            // mergedSamples: how many raw samples are in this group (for add math)
            // totalSteps: sum of |delta| across merged samples (the queue_step count)
            size_t mergedSamples = 1;
            uint32_t totalSteps = static_cast<uint32_t>(std::abs(delta));

            // Merge consecutive steps with the same delta sign.
            // Also split when the interval change would cause add to overflow,
            // or when totalSteps would overflow uint16_t.
            while (idx + mergedSamples < rawSteps.size() &&
                   (rawSteps[idx + mergedSamples].delta > 0 ? 1 : -1) == deltaSign) {
                uint32_t nextAbs = static_cast<uint32_t>(
                    std::abs(rawSteps[idx + mergedSamples].delta));
                if (static_cast<uint64_t>(totalSteps) + static_cast<uint64_t>(nextAbs) > 65535) break;

                // Check if adding this sample would cause add to overflow.
                if (mergedSamples > 1) {
                    uint32_t nextInterval = rawSteps[idx + mergedSamples].interval;
                    int64_t projectedChange = static_cast<int64_t>(nextInterval) -
                                              static_cast<int64_t>(baseInterval);
                    int64_t projectedAdd = projectedChange /
                                            static_cast<int64_t>(mergedSamples);
                    if (projectedAdd > ADD_MAX || projectedAdd < ADD_MIN) {
                        break; // Split here to avoid overflow
                    }
                }
                totalSteps += nextAbs;
                ++mergedSamples;
            }
            if (totalSteps == 0) { idx += mergedSamples; continue; }
            if (totalSteps > 65535) totalSteps = 65535;

            // Compute `add` from the interval change across the group.
            // Klipper applies `add` to the interval EVERY STEP, so the per-step
            // delta is (lastInterval - firstInterval) / (totalSteps - 1).
            // (Dividing by mergedSamples-1 would be per-sample, far too large
            // when a single sample covers many steps.)
            int32_t add = 0;
            if (totalSteps > 1) {
                uint32_t lastInterval = rawSteps[idx + mergedSamples - 1].interval;
                if (lastInterval != baseInterval) {
                    int64_t totalChange = static_cast<int64_t>(lastInterval) -
                                          static_cast<int64_t>(baseInterval);
                    int64_t computedAdd = totalChange /
                                          static_cast<int64_t>(totalSteps - 1);
                    // Clamp to valid range
                    if (computedAdd > ADD_MAX) add = ADD_MAX;
                    else if (computedAdd < ADD_MIN) add = ADD_MIN;
                    else add = static_cast<int32_t>(computedAdd);
                }
            }

            objects::StepCommand cmd;
            cmd.interval = baseInterval;
            cmd.count = static_cast<uint16_t>(totalSteps);
            cmd.add = add;
            cmd.dir = static_cast<int8_t>(deltaSign);

            seq.steps.push_back(cmd);
            idx += mergedSamples;
        }

        if (!seq.steps.empty()) result.push_back(std::move(seq));
    }
    return result;
}

} // namespace tether::klipper::motion
