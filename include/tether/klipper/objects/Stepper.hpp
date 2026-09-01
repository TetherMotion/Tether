/**
 * @file Stepper.hpp
 * @brief Stepper motor object (device-side handler + host-side proxy).
 *
 * @details
 * The stepper object is the core motion peripheral. On the device side, it
 * executes `queue_step` sequences: each step has an interval (clock ticks
 * between steps), a count (number of steps), and an add (signed delta added
 * to the interval each step, for acceleration). The stepper maintains a
 * position counter and a step buffer.
 *
 * On the host side, the StepperProxy holds the OID and builds `queue_step`
 * command content for the motion translator.
 *
 * @see MotionTranslator for how Tether's MotionPlan is converted to queue_step.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <vector>
#include <deque>
#include <string>

namespace tether::klipper::objects {

/// @brief A single step in a queue_step sequence.
struct StepCommand {
    uint32_t interval = 0;  ///< Clock ticks between steps
    uint16_t count = 0;      ///< Number of steps
    int16_t add = 0;         ///< Signed delta added to interval each step
    /// @brief Step direction: +1 (forward) or -1 (reverse).
    /// Set via set_next_step_dir on the device; recorded here so the
    /// host-side dispatcher and tests can reproduce the intended motion.
    int8_t dir = 1;
};

/**
 * @brief Device-side stepper handler.
 *
 * Maintains the current position and a queue of pending step sequences. In
 * passthrough mode, steps are executed at the scheduled clock times on a
 * virtual stepper. In reconstruct mode, the step sequence is fed to the
 * MotionReconstructor for analysis.
 */
class Stepper {
public:
    explicit Stepper(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    /// @return Current step position (signed).
    int32_t position() const { return position_.load(std::memory_order_relaxed); }

    /// @brief Set the current step position (signed).
    void setPosition(int32_t p) { position_.store(p, std::memory_order_relaxed); }

    /// @brief Set the step direction invert mask.
    void setStepInvert(uint8_t invert) { stepInvert_ = invert; }

    /// @brief Set the active step direction (+1 forward, -1 reverse).
    /// Applied to subsequent enqueued step commands as they execute.
    void setDirection(int8_t dir) { dir_ = (dir < 0) ? -1 : 1; }

    /// @return The active step direction.
    int8_t direction() const { return dir_; }

    /// @brief Reset the stepper (clear queue, reset position).
    void reset() {
        position_.store(0, std::memory_order_relaxed);
        queue_.clear();
        nextStepClock_ = 0;
    }

    /// @brief Execute a single step in the given direction (real-time path).
    /// Advances the position counter by +1 or -1. Used by the StepScheduler
    /// to fire individual steps without going through the queue.
    void step(int8_t dir) {
        position_.fetch_add((dir < 0) ? -1 : 1, std::memory_order_relaxed);
    }

    /**
     * @brief Enqueue a step sequence.
     * @param cmd Step command (interval, count, add).
     * @param clock The MCU clock at which this sequence begins.
     */
    void enqueueStep(const StepCommand& cmd, uint32_t clock) {
        queue_.push_back({cmd, clock});
    }

    /**
     * @brief Advance the stepper to the given MCU clock, executing any
     *        steps whose time has arrived.
     * @param clock Current MCU clock.
     * @return Number of steps taken.
     */
    uint32_t tick(uint32_t clock);

    /// @return True if the stepper has no pending steps.
    bool idle() const { return queue_.empty(); }

    /// @return Number of pending step commands.
    size_t pendingCommands() const { return queue_.size(); }

private:
    struct QueuedStep {
        StepCommand cmd;
        uint32_t startClock;
    };
    uint8_t oid_;
    std::atomic<int32_t> position_{0};
    uint8_t stepInvert_ = 0;
    int8_t dir_ = 1;
    std::deque<QueuedStep> queue_;
    uint32_t nextStepClock_ = 0;
    uint32_t currentInterval_ = 0;
    uint16_t remainingSteps_ = 0;
    int16_t currentAdd_ = 0;
};

/**
 * @brief Host-side stepper proxy.
 *
 * Holds the OID and provides helpers to build queue_step command content.
 */
class StepperProxy {
public:
    explicit StepperProxy(uint8_t oid) : oid_(oid) {}
    uint8_t oid() const { return oid_; }

private:
    uint8_t oid_;
};

} // namespace tether::klipper::objects
