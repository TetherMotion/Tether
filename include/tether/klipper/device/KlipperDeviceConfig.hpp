/**
 * @file KlipperDeviceConfig.hpp
 * @brief Configuration struct for KlipperDevice (extracted for layer separation).
 */

#pragma once

#include "tether/klipper/motion/MotionBlockSink.hpp"

#include <cstdint>
#include <memory>

namespace tether::klipper::device {

/// @brief Motion execution mode for the device.
enum class MotionMode {
    Passthrough,       ///< Execute steps directly on a virtual stepper.
    ReconstructReplan, ///< Reconstruct steps into MotionBlocks for analysis.
};

/// @brief Configuration for the Klipper device.
struct KlipperDeviceConfig {
    /// MCU clock frequency in Hz.
    uint32_t clockFreqHz = 180000000;
    /// Motion execution mode.
    MotionMode motionMode = MotionMode::Passthrough;
    /// Sink for motion blocks (required for ReconstructReplan mode).
    std::shared_ptr<motion::MotionBlockSink> motionSink;
    /// If true, enable the real-time StepScheduler. When enabled, queue_step
    /// commands are also forwarded to the StepScheduler, which fires steps
    /// against a real monotonic timer (std::chrono::steady_clock). The
    /// step callback updates the Stepper's position counter, so both the
    /// simulated tick() path and the real-time scheduler path stay in sync.
    /// Call device.tickStepScheduler() periodically to pump the scheduler.
    bool useStepScheduler = false;
};

} // namespace tether::klipper::device
