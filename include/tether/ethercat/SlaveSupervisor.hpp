/**
 * @file SlaveSupervisor.hpp
 * @brief Critical-condition detection and automatic slave recovery for EtherCAT
 *
 * @details
 * The SlaveSupervisor monitors EtherCAT slaves for *critical conditions* —
 * errors that prevent further operation of a slave (e.g. the slave dropped
 * out of OP, reported a fatal AL status code, or the application flagged it
 * via markCritical()).  When a critical condition is detected, the supervisor:
 *
 *   1. Notifies all registered event listeners.
 *   2. Pauses PDO data flow for the affected slave (via the per-slave
 *      "suspended" flag that the motion loop checks each cycle).
 *   3. Forces the slave back to INIT.
 *   4. Invokes the registered ISlaveRecoveryHandler to re-initialize the
 *      slave from scratch (mailbox → PRE_OP → PDO config → SAFE_OP → OP,
 *      plus any drive-level re-init such as CiA 402 enable).
 *   5. Resumes PDO data flow once the handler reports success.
 *
 * Recovery is **opt-in**.  The supervisor is disabled by default and must
 * be explicitly enabled via the config.  A user-configurable retry limit
 * caps how many consecutive recovery attempts are made before the supervisor
 * gives up and reports a permanent failure.
 *
 * ## Critical-condition triggers
 *
 * The supervisor can be configured to react to any combination of:
 *   - **AL status codes**: standard codes indicating the slave needs reset
 *     (SlaveNeedsInit, SlaveNeedsColdStart, SlaveNeedsPreOp,
 *     SlaveNeedsSafeOp, FatalSyncError, NoSyncError, SynchronizationError).
 *   - **Transition failures**: OP/SAFE_OP not confirmed after timeout, or
 *     the slave dropping to a lower state unexpectedly.
 *   - **App-injected flag**: the application calls `markCritical()` to
 *     signal a condition the supervisor cannot detect on its own.
 *
 * Triggers are user-configurable via `RecoveryConfig::triggers` (a bitmask
 * of `CriticalTrigger` values).  Additional AL status codes can be added
 * via `RecoveryConfig::critical_al_codes`.
 *
 * ## Motion-loop integration
 *
 * The supervisor exposes `isSlaveSuspended(slave_index)` which the motion
 * loop (or DS402Master) must check each cycle.  While a slave is suspended,
 * its PDO data must not be passed to motion controllers.  The supervisor
 * guarantees that `isSlaveSuspended()` returns true from the moment recovery
 * starts until the handler completes and the slave is back in OP.
 *
 * ## Event listeners
 *
 * Listeners implement `IRecoveryEventListener` and are registered via
 * `addEventListener()`.  They receive notifications for:
 *   - Critical condition detected
 *   - Recovery started
 *   - Recovery succeeded
 *   - Recovery failed (retry or permanent)
 *   - Slave suspended / resumed
 *
 * @see SlaveStatusPoller for the background monitoring that feeds events
 *      into the supervisor.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <atomic>
#include <string>
#include <string_view>

#include "tether/ethercat/FaultDetection.hpp"  // ALStatusCode, IFaultTransport
#include "tether/ethercat/SlaveStatusPoller.hpp"  // SlaveStatusEvent
#include "tether/ethercat/ALResetController.hpp"  // ALResetController
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/SlaveRecovery.hpp"

namespace EtherCAT {

class Master;
class Slave;

// ============================================================================
// SlaveSupervisor
// ============================================================================

/**
 * @brief Monitors EtherCAT slaves for critical conditions and orchestrates
 *        automatic recovery.
 *
 * The supervisor is constructed with a reference to the Master and an
 * optional recovery handler.  When enabled, it:
 *
 *   1. Registers a callback with the Master's SlaveStatusPoller to detect
 *      state drops and error flags.
 *   2. Exposes `handleStatusEvent()` and `handleTransitionFailure()` for
 *      the application / state machine to report critical conditions.
 *   3. Exposes `markCritical()` for the application to inject a critical
 *      condition that the supervisor cannot detect on its own.
 *   4. When a critical condition is detected, suspends PDO data for the
 *      slave, forces it to INIT, calls the recovery handler, and resumes
 *      PDO data on success.
 *
 * ## Thread safety
 *
 * - `isSlaveSuspended()` is lock-free (atomic) and safe to call from the
 *   realtime loop.
 * - All other methods must be called from non-realtime threads.
 * - Recovery is serialized per-slave via an internal mutex.
 *
 * ## Usage
 *
 * @code
 *   // 1. Create a recovery handler (e.g. one that re-configures a DS402 drive)
 *   class MyRecoveryHandler : public ISlaveRecoveryHandler {
 *       bool reinitializeSlave(uint16_t idx) override {
 *           return ds402.configureDrive(configs[idx]) &&
 *                  ds402.enableDrive(idx);
 *       }
 *   };
 *
 *   // 2. Configure and enable the supervisor
 *   RecoveryConfig cfg;
 *   cfg.enabled = true;
 *   cfg.max_attempts = 3;
 *   cfg.stop_loop_during_recovery = false;  // single-slave recovery
 *
 *   auto& supervisor = master.slaveSupervisor();
 *   supervisor.configure(cfg);
 *   supervisor.setRecoveryHandler(std::make_unique<MyRecoveryHandler>());
 *   supervisor.addEventListener(my_listener);
 *   supervisor.start();
 *
 *   // 3. In the motion loop, check suspension state:
 *   bool updateMotion(double dt) {
 *       for (auto& [idx, ctrl] : controllers) {
 *           if (supervisor.isSlaveSuspended(idx)) continue;
 *           ctrl->update(drive, dt);
 *       }
 *       return true;
 *   }
 *
 *   // 4. On a transition failure (e.g. from CiA402Drive::gotoOp()):
 *   supervisor.handleTransitionFailure(idx, "OP not confirmed after 5s",
 *                                       al_code);
 * @endcode
 */
class SlaveSupervisor {
public:
    /**Maximum number of slaves the supervisor can track */
    static constexpr size_t kMaxSlaves = ECAT_STATUS_POLLER_MAX_SLAVES;

    /**
     * @brief Construct a supervisor bound to a Master.
     * @param master The EtherCAT Master whose slaves are supervised.
     */
    explicit SlaveSupervisor(Master& master);
    ~SlaveSupervisor();

    // Non-copyable, non-movable
    SlaveSupervisor(const SlaveSupervisor&) = delete;
    SlaveSupervisor& operator=(const SlaveSupervisor&) = delete;
    SlaveSupervisor(SlaveSupervisor&&) = delete;
    SlaveSupervisor& operator=(SlaveSupervisor&&) = delete;

    // ----- Configuration -----

    /**
     * @brief Set the recovery configuration.
     *
     * Must be called before `start()`.  Calling after start() requires a
     * stop() + start() cycle to take effect.
     *
     * @param config  Recovery policy: enable flag, critical triggers,
     *                retry limits/backoff, and timeouts. Copied into the
     *                supervisor; the caller may reuse or destroy the
     *                argument afterwards.
     */
    void configure(const RecoveryConfig& config);

    /**
     * @brief Read the currently active recovery configuration.
     *
     * @return Const reference to the configuration installed by the last
     *         configure() call (or the defaults if never configured). The
     *         reference remains valid until the next configure() call or
     *         destruction of the supervisor — it is not a snapshot, so do
     *         not hold it across a configure().
     */
    const RecoveryConfig& config() const { return config_; }

    /**
     * @brief Whether supervision is enabled in the current configuration.
     *
     * start() refuses to run when this returns false, so a supervisor can
     * be constructed and wired up while staying inert until the application
     * enables it via configure().
     *
     * @return `config_.enabled`
     */
    bool isEnabled() const { return config_.enabled; }

    // ----- Recovery Handler -----

    /**
     * @brief Set the recovery handler invoked during slave re-initialization.
     *
     * The handler performs the actual recovery work — re-configuring the
     * slave's mailbox/SDOs, re-mapping PDOs, and driving the AL state
     * machine back to OP — while the supervisor only decides when and how
     * often to attempt it. Ownership is transferred to the supervisor;
     * the handler must outlive any in-progress recovery, which stop()
     * guarantees by cancelling first.
     *
     * Must be set before start(); a supervisor without a handler refuses
     * to start, and if the handler is cleared mid-run recovery attempts
     * fail immediately and count against the retry limit.
     *
     * @param handler  Recovery implementation to adopt; may be nullptr to
     *                 uninstall the current handler.
     */
    void setRecoveryHandler(std::unique_ptr<ISlaveRecoveryHandler> handler);

    // ----- Event Listeners -----

    /**
     * @brief Register an event listener.
     *
     * The listener is invoked on every recovery lifecycle event (critical
     * condition detected, attempt started/finished, slave recovered or
     * declared failed). Listeners are called synchronously on the recovery
     * thread, so callbacks must be fast and must not call back into the
     * supervisor. The supervisor does not take ownership — the caller must
     * keep the listener alive until it is removed or the supervisor is
     * stopped.
     *
     * @param listener  Listener to add; borrowed, not owned.
     * @return true on success, false if the listener is null or already registered.
     */
    bool addEventListener(IRecoveryEventListener* listener);

    /**
     * @brief Unregister an event listener.
     *
     * @param listener  Listener previously passed to addEventListener().
     * @return true if the listener was found and removed.
     */
    bool removeEventListener(IRecoveryEventListener* listener);

    /**Remove all event listeners */
    void clearEventListeners();

    // ----- Lifecycle -----

    /**
     * @brief Initialize the supervisor for the given number of slaves.
     *
     * Resets all per-slave state.  Called automatically by start(), but
     * can be called explicitly to pre-size internal arrays.
     *
     * @param slave_count Number of slaves to supervise
     * @return true on success
     */
    bool init(uint16_t slave_count);

    /**
     * @brief Start supervision.
     *
     * Registers a callback with the Master's SlaveStatusPoller (if running)
     * and begins monitoring.  The supervisor must be enabled
     * (`config_.enabled == true`) and initialized.
     *
     * @return true on success, false if already running, disabled, or no
     *         handler is set.
     */
    bool start();

    /**
     * @brief Stop supervision and cancel any in-progress recovery.
     */
    void stop();

    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    // ----- Critical Condition Reporting -----

    /**
     * @brief Handle a SlaveStatusPoller event.
     *
     * Called automatically when registered as a poller callback, but can
     * also be called directly by the application.
     *
     * Evaluates the event against the configured triggers and initiates
     * recovery if a critical condition is detected.
     *
     * @param event  Poller event: slave index, observed AL state /
     *               working-counter failure, and timestamp. Only events for
     *               slaves below slaveCount() are acted on; out-of-range
     *               indices are ignored.
     */
    void handleStatusEvent(const SlaveStatusEvent& event);

    /**
     * @brief Report a transition failure (e.g. OP not confirmed after timeout).
     *
     * Called by the state machine or application when a state transition
     * fails.  If `CriticalTrigger::TransitionFailures` is active, this
     * initiates recovery.
     *
     * @param slave_index    Slave that failed the transition
     * @param detail         Human-readable description of the failure
     * @param al_status_code AL_STATUS_CODE at the time of failure (0 if unknown)
     */
    void handleTransitionFailure(uint16_t slave_index,
                                  std::string_view detail,
                                  uint16_t al_status_code = 0);

    /**
     * @brief Mark a slave as critical (app-injected trigger).
     *
     * Allows the application to signal a critical condition that the
     * supervisor cannot detect on its own (e.g. a drive-specific fault
     * that requires a full re-init).  If `CriticalTrigger::AppInjected`
     * is active, this initiates recovery.
     *
     * @param slave_index Slave to mark critical
     * @param detail      Human-readable reason
     */
    void markCritical(uint16_t slave_index, std::string_view detail);

    // ----- Motion-Loop Integration -----

    /**
     * @brief Check if a slave's PDO data is currently suspended.
     *
     * **Lock-free and realtime-safe.**  The motion loop must call this
     * each cycle and skip any slave for which it returns true.
     *
     * @param slave_index Slave to check
     * @return true if the slave's PDO data must not be used
     */
    bool isSlaveSuspended(uint16_t slave_index) const;

    /**
     * @brief Check if a slave is currently in recovery.
     *
     * @param slave_index Slave to check
     * @return true if recovery is in progress for this slave
     */
    bool isSlaveRecovering(uint16_t slave_index) const;

    // ----- Query -----

    /**
     * @brief Get the current recovery state of a slave.
     *
     * @param slave_index  Slave to query (0-based station index).
     * @return SlaveRecoveryState, or Normal if the slave index is invalid.
     */
    SlaveRecoveryState slaveState(uint16_t slave_index) const;

    /**
     * @brief Get the number of recovery attempts made for a slave.
     *
     * Counts every attempt since init() or the last resetSlaveState(),
     * including the one currently in progress. Compared against the
     * configured retry limit — when it reaches the limit the slave is
     * declared failed and no further automatic attempts are made.
     *
     * @param slave_index  Slave to query (0-based station index).
     * @return Attempt count, or 0 for an invalid index.
     */
    int slaveAttemptCount(uint16_t slave_index) const;

    /**
     * @brief Reset the recovery state for a slave (e.g. after manual
     *        intervention).  Does not affect an in-progress recovery.
     *
     * Clears the attempt counter and moves the slave back to
     * SlaveRecoveryState::Normal so automatic recovery can trigger again.
     * Also clears the suspended flag, so the motion loop resumes using the
     * slave's PDO data — only call this once the slave is actually healthy.
     *
     * @param slave_index  Slave to reset (0-based station index); invalid
     *                     indices are ignored.
     */
    void resetSlaveState(uint16_t slave_index);

    /**
     * @brief Number of slaves under supervision.
     *
     * Set by init() and used to bound every per-slave query and event —
     * indices ≥ slaveCount() are treated as invalid and ignored.
     *
     * @return Slave count passed to the last successful init()/start().
     */
    uint16_t slaveCount() const { return slave_count_.load(std::memory_order_acquire); }

private:
    // ----- Internal per-slave state -----
    struct PerSlaveState {
        std::atomic<SlaveRecoveryState> state{SlaveRecoveryState::Normal};
        std::atomic<bool> suspended{false};
        std::atomic<int> attempt_count{0};
        uint16_t last_al_status_code = 0;
        std::string last_detail;
        // Mutex serializes recovery attempts for this slave
        std::mutex recovery_mutex;
    };

    // ----- Helpers -----
    void dispatchEvent(RecoveryEventType type, uint16_t slave_index,
                       uint16_t al_code, int attempt,
                       std::string_view detail);
    void setSuspended(uint16_t slave_index, bool suspended);
    bool forceSlaveToInit(uint16_t slave_index);
    bool attemptRecovery(uint16_t slave_index);
    void notifyRecoveryResult(uint16_t slave_index, bool success);

    // Check if triggers are active and the condition matches
    bool shouldTriggerRecovery(CriticalTrigger trigger_category,
                                uint16_t al_status_code) const;

    Master& master_;
    RecoveryConfig config_;
    std::unique_ptr<ISlaveRecoveryHandler> recovery_handler_;

    std::atomic<uint16_t> slave_count_{0};
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};

    mutable std::mutex listeners_mutex_;
    std::vector<IRecoveryEventListener*> listeners_;

    // Per-slave state array (sized to kMaxSlaves, only [0,slave_count) used)
    std::array<PerSlaveState, kMaxSlaves> slave_states_{};

    // Handle for the poller callback (for deregistration)
    CallbackHandle poller_handle_{0};
};

} // namespace EtherCAT
