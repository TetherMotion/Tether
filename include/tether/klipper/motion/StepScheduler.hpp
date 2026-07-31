/**
 * @file StepScheduler.hpp
 * @brief Real-time step scheduler that executes StepCommands against a
 *        monotonic timer.
 *
 * @details
 * The StepScheduler is the real-time counterpart to the Stepper's simulated
 * `tick()` method. Instead of advancing a virtual MCU clock, it uses
 * `std::chrono::steady_clock` (a real monotonic timer) to determine when
 * each step is due and fires a callback for each step taken.
 *
 * Each scheduled entry is a (StepCommand, startClock) pair, where
 * `startClock` is the MCU clock tick at which the sequence begins. The
 * scheduler converts MCU ticks to wall time using the clock frequency and
 * a reference (anchor) pair of (wallTime, mcuTick) established at
 * `start()` or via `setClockAnchor()`.
 *
 * Usage:
 * @code
 *   StepScheduler sched(180000000); // 180 MHz
 *   sched.setStepCallback([&](uint8_t oid, int8_t dir) {
 *       // drive real GPIO / step pin here
 *   });
 *   sched.start();                   // anchors clock to now
 *   // ... enqueue steps ...
 *   objects::StepCommand cmd{1000, 100, 0, 1};
 *   sched.schedule(oid, cmd, startClock);
 *   // ... call tick() periodically from a timer ISR / RTOS thread ...
 *   sched.tick();                    // fires due steps
 *   sched.wait();                    // blocks until all steps done
 * @endcode
 *
 * The scheduler is single-threaded and not thread-safe by default. For
 * multi-threaded use, wrap calls with a mutex externally.
 */

#pragma once

#include "tether/klipper/objects/Stepper.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <thread>
#include <vector>

namespace tether::klipper::motion {

/// @brief Real-time step scheduler executing StepCommands against a
///        monotonic timer.
class StepScheduler {
public:
    /// @brief Callback fired for each step taken.
    /// @param oid The stepper OID.
    /// @param dir The step direction (+1 or -1).
    using StepCallback = std::function<void(uint8_t oid, int8_t dir)>;

    /// @brief Construct with a clock frequency.
    /// @param clockFreqHz MCU clock frequency in Hz (ticks per second).
    explicit StepScheduler(uint32_t clockFreqHz = 180000000)
        : clockFreqHz_(clockFreqHz) {}

    /// @brief Set the per-step callback.
    void setStepCallback(StepCallback cb) { stepCb_ = std::move(cb); }

    /// @brief Set the clock anchor: (wallTime, mcuTick). Allows aligning
    ///        the scheduler's clock to an externally-synchronised reference.
    void setClockAnchor(std::chrono::steady_clock::time_point wallTime,
                        uint32_t mcuTick) {
        anchorWall_ = wallTime;
        anchorTick_ = mcuTick;
        anchored_ = true;
    }

    /// @brief Anchor the clock to the current wall time and MCU tick 0.
    ///        Subsequent schedule() calls use MCU ticks relative to this anchor.
    void start() {
        anchorWall_ = std::chrono::steady_clock::now();
        anchorTick_ = 0;
        anchored_ = true;
    }

    /// @brief Reset the scheduler: clear all pending steps and re-anchor.
    void reset() {
        entries_.clear();
        start();
    }

    /// @brief Schedule a step sequence for a given stepper OID.
    /// @param oid The stepper OID.
    /// @param cmd The step command (interval, count, add, dir).
    /// @param startClock The MCU clock tick at which this sequence begins.
    void schedule(uint8_t oid, const objects::StepCommand& cmd,
                  uint32_t startClock) {
        if (cmd.count == 0) return;
        Entry e;
        e.oid = oid;
        e.cmd = cmd;
        e.startClock = startClock;
        e.nextStepClock = startClock;
        e.currentInterval = cmd.interval;
        e.stepsExecuted = 0;
        entries_.push_back(std::move(e));
    }

    /// @brief Schedule multiple step sequences (one per OID).
    /// @param oid The stepper OID.
    /// @param cmds The step commands to schedule in order.
    /// @param startClock The MCU clock tick at which the first sequence begins.
    void scheduleSequence(uint8_t oid,
                          const std::vector<objects::StepCommand>& cmds,
                          uint32_t startClock) {
        uint32_t clock = startClock;
        for (const auto& cmd : cmds) {
            if (cmd.count == 0) continue;
            schedule(oid, cmd, clock);
            // Advance the clock by the total duration of this command.
            // Total = sum(interval + add*i for i in 0..count-1)
            //       = count*interval + add*count*(count-1)/2
            uint64_t dur = static_cast<uint64_t>(cmd.count) * cmd.interval
                         + static_cast<int64_t>(cmd.add)
                           * static_cast<uint64_t>(cmd.count)
                           * (cmd.count - 1) / 2;
            clock += static_cast<uint32_t>(dur);
        }
    }

    /// @brief Process due steps. Call this periodically (e.g. from a timer
    ///        ISR or RTOS thread at a high rate).
    /// @return Number of steps fired in this call.
    size_t tick() {
        if (!anchored_ || !stepCb_) return 0;
        auto now = std::chrono::steady_clock::now();
        uint32_t mcuNow = wallToMcu(now);
        size_t fired = 0;

        for (auto& e : entries_) {
            while (e.stepsExecuted < e.cmd.count) {
                if (mcuNow < e.nextStepClock) break;
                stepCb_(e.oid, e.cmd.dir);
                ++e.stepsExecuted;
                // Advance nextStepClock by currentInterval, then update
                // currentInterval for the next step (acceleration).
                e.nextStepClock += e.currentInterval;
                e.currentInterval = static_cast<uint32_t>(
                    static_cast<int32_t>(e.currentInterval) + e.cmd.add);
                ++fired;
            }
        }

        // Remove completed entries.
        entries_.erase(
            std::remove_if(entries_.begin(), entries_.end(),
                           [](const Entry& e) {
                               return e.stepsExecuted >= e.cmd.count;
                           }),
            entries_.end());

        return fired;
    }

    /// @brief Block until all scheduled steps have fired.
    /// @param maxWaitMs Maximum wait time in milliseconds (0 = no limit).
    /// @return True if all steps completed, false on timeout.
    bool wait(uint32_t maxWaitMs = 0) {
        auto deadline = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(maxWaitMs);
        while (!entries_.empty()) {
            tick();
            if (entries_.empty()) return true;
            if (maxWaitMs > 0 && std::chrono::steady_clock::now() > deadline)
                return false;
            // Brief sleep to avoid busy-looping.
            std::this_thread::sleep_for(
                std::chrono::microseconds(50));
        }
        return true;
    }

    /// @return True if there are no pending steps.
    bool idle() const { return entries_.empty(); }

    /// @return Number of pending step commands (not individual steps).
    size_t pendingCommands() const { return entries_.size(); }

    /// @return Number of individual steps still pending across all commands.
    size_t pendingSteps() const {
        size_t total = 0;
        for (const auto& e : entries_) {
            total += (e.cmd.count - e.stepsExecuted);
        }
        return total;
    }

    /// @return The clock frequency in Hz.
    uint32_t clockFrequency() const { return clockFreqHz_; }

private:
    /// @brief Convert a wall time to an MCU clock tick (32-bit).
    uint32_t wallToMcu(std::chrono::steady_clock::time_point wall) const {
        auto delta = std::chrono::duration<double>(wall - anchorWall_).count();
        uint64_t ticks = anchorTick_
            + static_cast<uint64_t>(delta * clockFreqHz_);
        return static_cast<uint32_t>(ticks);
    }

    struct Entry {
        uint8_t oid = 0;
        objects::StepCommand cmd;
        uint32_t startClock = 0;
        uint32_t nextStepClock = 0;
        uint32_t currentInterval = 0;
        uint16_t stepsExecuted = 0;
    };

    uint32_t clockFreqHz_;
    StepCallback stepCb_;
    std::chrono::steady_clock::time_point anchorWall_;
    uint32_t anchorTick_ = 0;
    bool anchored_ = false;
    std::deque<Entry> entries_;
};

} // namespace tether::klipper::motion
