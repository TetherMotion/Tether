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
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/motion/MotionBlock.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/klippy/DeltaPrinter.hpp"
#include "tether/klipper/klippy/RotaryDeltaPrinter.hpp"
#include "tether/motion_planner/MotionPlan.hpp"

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

/// @brief A translated step sequence for one axis.
struct AxisStepSequence {
    uint8_t oid = 0;
    uint32_t startClock = 0;
    std::vector<objects::StepCommand> steps;
};

/// @brief Kinematics transform: converts Cartesian (X,Y,Z) to stepper positions.
///
/// For Cartesian: stepper = cartesian (identity).
/// For CoreXY: A = X+Y, B = X-Y.
/// For CoreXZ: A = X+Z, B = X-Z.
/// For Delta: uses DeltaPrinter::cartesianToTower.
class KinematicsTransform {
public:
    /// @brief Set the kinematics type.
    void setKinematics(klippy::Kinematics k) { kinematics_ = k; }

    /// @brief Get the kinematics type.
    klippy::Kinematics kinematics() const { return kinematics_; }

    /// @brief Set the delta printer (for delta kinematics).
    void setDeltaPrinter(const klippy::DeltaPrinter* dp) { deltaPrinter_ = dp; }

    /// @brief Set the rotary delta printer (for rotary delta kinematics).
    void setRotaryDeltaPrinter(const klippy::RotaryDeltaPrinter* rdp) {
        rotaryDeltaPrinter_ = rdp;
    }

    /// @brief Set winch anchor parameters (for winch kinematics).
    void setWinchParams(double anchorRadius, double anchorHeight) {
        winchAnchorRadius_ = anchorRadius;
        winchAnchorHeight_ = anchorHeight;
    }

    /// @brief Transform a Cartesian position to stepper-space positions.
    ///
    /// @param x, y, z Cartesian position in mm.
    /// @return Array of 3 stepper positions (A, B, C) in mm.
    std::array<double, 3> transform(double x, double y, double z) const {
        switch (kinematics_) {
            case klippy::Kinematics::CoreXY:
                // A = X + Y, B = X - Y, C = Z
                return {x + y, x - y, z};
            case klippy::Kinematics::CoreXZ:
                // A = X + Z, B = X - Z, C = Y
                return {x + z, x - z, y};
            case klippy::Kinematics::CoreYZ:
                // A = Y + Z, B = Y - Z, C = X
                return {y + z, y - z, x};
            case klippy::Kinematics::HybridCoreXY:
                // HybridCoreXY: X/Y use CoreXY belts, Z is independent (leadscrew)
                // A = X + Y, B = X - Y, C = Z
                return {x + y, x - y, z};
            case klippy::Kinematics::HybridCoreXZ:
                // HybridCoreXZ: X/Z use CoreXZ belts, Y is independent
                // A = X + Z, B = X - Z, C = Y
                return {x + z, x - z, y};
            case klippy::Kinematics::Delta:
                if (deltaPrinter_) {
                    return deltaPrinter_->cartesianToTower(x, y, z);
                }
                return {x, y, z};
            case klippy::Kinematics::RotaryDelta: {
                // Rotary delta: three upper arms at 120° spacing.
                // Stepper positions are shoulder angles (radians).
                if (rotaryDeltaPrinter_) {
                    return rotaryDeltaPrinter_->cartesianToTower(x, y, z);
                }
                // Fallback: no geometry configured, return identity.
                return {x, y, z};
            }
            case klippy::Kinematics::Polar: {
                // Polar: A = radius, B = angle (degrees), C = Z
                double radius = std::sqrt(x*x + y*y);
                double angle = std::atan2(y, x) * 180.0 / M_PI;
                return {radius, angle, z};
            }
            case klippy::Kinematics::Winch: {
                // Winch/cable: A/B/C are cable lengths from three anchors.
                // Anchors at fixed positions (simplified: equilateral triangle).
                double anchorR = winchAnchorRadius_;
                double az1 = 0.0, az2 = 2.0*M_PI/3.0, az3 = 4.0*M_PI/3.0;
                double a1x = anchorR * std::cos(az1), a1y = anchorR * std::sin(az1);
                double a2x = anchorR * std::cos(az2), a2y = anchorR * std::sin(az2);
                double a3x = anchorR * std::cos(az3), a3y = anchorR * std::sin(az3);
                double h = winchAnchorHeight_;
                double la = std::sqrt((x-a1x)*(x-a1x) + (y-a1y)*(y-a1y) + (z-h)*(z-h));
                double lb = std::sqrt((x-a2x)*(x-a2x) + (y-a2y)*(y-a2y) + (z-h)*(z-h));
                double lc = std::sqrt((x-a3x)*(x-a3x) + (y-a3y)*(y-a3y) + (z-h)*(z-h));
                return {la, lb, lc};
            }
            case klippy::Kinematics::Cartesian:
            case klippy::Kinematics::None:
            default:
                return {x, y, z};
        }
    }

    /// @brief Transform stepper-space positions back to Cartesian.
    ///
    /// @param a, b, c Stepper positions in mm.
    /// @return Array of 3 Cartesian positions (X, Y, Z) in mm.
    std::array<double, 3> inverseTransform(double a, double b, double c) const {
        switch (kinematics_) {
            case klippy::Kinematics::CoreXY:
                // X = (A + B) / 2, Y = (A - B) / 2, Z = C
                return {(a + b) / 2.0, (a - b) / 2.0, c};
            case klippy::Kinematics::CoreXZ:
                // X = (A + B) / 2, Y = C, Z = (A - B) / 2
                return {(a + b) / 2.0, c, (a - b) / 2.0};
            case klippy::Kinematics::CoreYZ:
                // X = C, Y = (A + B) / 2, Z = (A - B) / 2
                return {c, (a + b) / 2.0, (a - b) / 2.0};
            case klippy::Kinematics::Delta:
                if (deltaPrinter_) {
                    return deltaPrinter_->towerToCartesian(a, b, c);
                }
                return {a, b, c};
            case klippy::Kinematics::HybridCoreXY:
                // X = (A + B) / 2, Y = (A - B) / 2, Z = C
                return {(a + b) / 2.0, (a - b) / 2.0, c};
            case klippy::Kinematics::HybridCoreXZ:
                // X = (A + B) / 2, Y = C, Z = (A - B) / 2
                return {(a + b) / 2.0, c, (a - b) / 2.0};
            case klippy::Kinematics::RotaryDelta: {
                // Inverse rotary delta: from shoulder angles to Cartesian
                if (rotaryDeltaPrinter_) {
                    return rotaryDeltaPrinter_->towerToCartesian(a, b, c);
                }
                return {a, b, c};
            }
            case klippy::Kinematics::Polar: {
                // Inverse polar: A = radius, B = angle (degrees), C = Z
                double rad = a * M_PI / 180.0;
                return {b * std::cos(rad), b * std::sin(rad), c};
            }
            case klippy::Kinematics::Winch: {
                // Inverse winch: trilateration from three cable lengths
                // Simplified: anchors at equilateral triangle, height h
                double anchorR = winchAnchorRadius_;
                double h = winchAnchorHeight_;
                double a1x = anchorR, a1y = 0.0;
                double a2x = anchorR * std::cos(2.0*M_PI/3.0);
                double a2y = anchorR * std::sin(2.0*M_PI/3.0);
                // Using first two anchors to estimate XY
                double da = a*a - b*b;
                double dx = da / (2.0 * (a1x - a2x));
                double dy = (a*a - dx*dx - (dx - a1x)*(dx - a1x) +
                            a2x*a2x + a2y*a2y - 2.0*dx*a2x) / (2.0 * a2y);
                double dz = std::sqrt(std::max(0.0, a*a - (dx-a1x)*(dx-a1x) - (dy-a1y)*(dy-a1y)));
                return {dx, dy, dz - h};
            }
            case klippy::Kinematics::Cartesian:
            case klippy::Kinematics::None:
            default:
                return {a, b, c};
        }
    }

private:
    klippy::Kinematics kinematics_ = klippy::Kinematics::Cartesian;
    const klippy::DeltaPrinter* deltaPrinter_ = nullptr;
    const klippy::RotaryDeltaPrinter* rotaryDeltaPrinter_ = nullptr;
    double winchAnchorRadius_ = 500.0;   ///< Winch anchor radius (mm)
    double winchAnchorHeight_ = 300.0;   ///< Winch anchor height (mm)
};

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

    /// @brief Set the kinematics transform.
    void setKinematicsTransform(const KinematicsTransform& kt) { kinematics_ = kt; }

    /// @brief Get the kinematics transform.
    const KinematicsTransform& kinematicsTransform() const { return kinematics_; }

private:
    std::array<AxisConfig, Dim> axisConfigs_;
    std::array<uint8_t, Dim> axisOids_;
    std::string sourceLabel_;
    KinematicsTransform kinematics_;
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
        auto stepperPos = kinematics_.transform(cartX, cartY, cartZ);
        // Extract velocities for kinematics transform (approximate)
        double velX = (Dim > 0) ? static_cast<double>(state.velocity[0]) : 0.0;
        double velY = (Dim > 1) ? static_cast<double>(state.velocity[1]) : 0.0;
        double velZ = (Dim > 2) ? static_cast<double>(state.velocity[2]) : 0.0;
        // For velocity, use the Jacobian approximation:
        // For CoreXY: dA/dt = dX/dt + dY/dt, dB/dt = dX/dt - dY/dt
        // For Cartesian: identity
        std::array<double, 3> stepperVel;
        switch (kinematics_.kinematics()) {
            case klippy::Kinematics::CoreXY:
                stepperVel = {velX + velY, velX - velY, velZ};
                break;
            case klippy::Kinematics::CoreXZ:
                stepperVel = {velX + velZ, velX - velZ, velY};
                break;
            case klippy::Kinematics::CoreYZ:
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
                if (static_cast<uint64_t>(totalSteps) + nextAbs > 65535) break;

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
