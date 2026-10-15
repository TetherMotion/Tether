#pragma once

/**
 * @file SlaveSyncRecovery.hpp
 * @brief Run a slave transition step with `SlaveSupervisor`-backed
 *        recovery on failure.
 *
 * A phased multi-slave startup (INIT → PRE-OP → SAFE-OP → OP) wants the
 * same failure policy everywhere: try the step, and if it fails, let the
 * supervisor's recovery machinery run the slot's `reinitialize()`.
 * `transitionWithRecovery()` encapsulates that policy, including the
 * subtle case where the supervisor's own AL-status-code poller already
 * triggered recovery mid-step — in that case the caller must *wait* for
 * the in-flight recovery instead of injecting a second critical event.
 *
 * On success (including a completed recovery) the slot's `on_reached`
 * callback has already run — the slave is in its operational state and
 * its `ready` flag set by whoever wired the slot.
 */

#include <chrono>
#include <stop_token>
#include <string_view>
#include <thread>

#include "logging/Logger.hpp"
#include "tether/ethercat/SlaveSupervisor.hpp"

namespace EtherCAT::Util {

/**
 * @brief Execute `step_fn()` for `slave_index`; recover via the
 *        supervisor when it fails.
 *
 * Recovery flow:
 *   1. `step_fn()` succeeded → return true immediately.
 *   2. Supervisor not running → recovery impossible, return false.
 *   3. `supervisor.isSlaveRecovering(slave_index)` already true (the
 *      poller fired on an AL status code while the step ran) → poll
 *      `isSlaveRecovering` until it clears or `stop` is requested — the
 *      in-flight recovery runs the full reinit sequence.
 *   4. Otherwise → `markCritical(slave_index, "<step> failed")`, which
 *      runs `attemptRecovery` synchronously on this thread.
 *   5. Return whether `slaveState()` ended in `SlaveRecoveryState::Normal`.
 *
 * @tparam StepFn  Callable `bool()` — the transition step
 *                 (e.g. `coord.transitionOne(idx, ECState::SafeOp, …)`).
 * @param supervisor  Running `SlaveSupervisor` configured with a
 *                    recovery handler that reinitializes the slave.
 * @param slave_index Slave whose step failed / is being attempted.
 * @param step_fn     The transition step to attempt first.
 * @param step_name   Human-readable step name for log messages.
 * @param stop        Cooperative cancellation for the recovery wait.
 * @param log_tag     Logger tag prefix.
 * @return true on success (direct or post-recovery), false on
 *         permanent failure or cancellation.
 */
template <typename StepFn>
bool transitionWithRecovery(EtherCAT::SlaveSupervisor& supervisor,
                            uint16_t slave_index,
                            StepFn&& step_fn,
                            std::string_view step_name,
                            std::stop_token stop = {},
                            const char* log_tag = "ethercat") {
    if (step_fn()) return true;
    TETHER_LOGE(log_tag, "{} failed — triggering supervisor recovery",
                step_name);
    if (!supervisor.isRunning()) {
        TETHER_LOGE(log_tag,
                    "Supervisor not running — cannot recover {}",
                    step_name);
        return false;
    }

    // The supervisor's ALStatusCodes trigger can fire from the poller
    // thread WHILE this thread's step was still running.  In that case
    // markCritical() is a no-op (state is already Recovering) and this
    // thread must WAIT for the poller-initiated recovery to complete
    // instead of injecting a second critical event.
    if (supervisor.isSlaveRecovering(slave_index)) {
        TETHER_LOGI(log_tag,
                    "{}: supervisor recovery already in progress "
                    "(triggered by AL status code poller) — waiting for "
                    "completion...",
                    step_name);
        while (supervisor.isSlaveRecovering(slave_index)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (stop.stop_requested()) break;
        }
    } else {
        // No recovery in progress — trigger one ourselves.  markCritical
        // runs attemptRecovery synchronously on this thread, so when it
        // returns the state is already Normal or Failed.
        supervisor.markCritical(slave_index,
                                std::string(step_name) + " failed");
    }

    const auto state = supervisor.slaveState(slave_index);
    if (state == EtherCAT::SlaveRecoveryState::Normal) {
        TETHER_LOGI(log_tag, "{} recovered after {} attempt(s)",
                    step_name, supervisor.slaveAttemptCount(slave_index));
        return true;
    }
    TETHER_LOGE(log_tag,
                "{} recovery failed permanently after {} attempt(s)",
                step_name, supervisor.slaveAttemptCount(slave_index));
    return false;
}

} // namespace EtherCAT::Util
