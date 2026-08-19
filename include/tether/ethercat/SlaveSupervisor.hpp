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

namespace EtherCAT {

class Master;
class Slave;

// ============================================================================
// Critical Trigger Flags (bitmask)
// ============================================================================

/**
 * @brief Bitmask of critical-condition trigger categories
 *
 * Combine with `|` and pass to `RecoveryConfig::triggers`.
 */
enum class CriticalTrigger : uint16_t {
    None                = 0x0000,
    /// React to standard AL status codes that indicate the slave needs reset
    /// (SlaveNeedsInit, SlaveNeedsColdStart, SlaveNeedsPreOp,
    ///  SlaveNeedsSafeOp, FatalSyncError, NoSyncError, SynchronizationError)
    ALStatusCodes       = 0x0001,
    /// React to transition failures (OP/SAFE_OP not confirmed, or slave
    /// dropping to a lower state unexpectedly)
    TransitionFailures  = 0x0002,
    /// React to app-injected critical flags via markCritical()
    AppInjected         = 0x0004,
    /// React to all triggers
    All                 = 0xFFFF,
};

inline CriticalTrigger operator|(CriticalTrigger a, CriticalTrigger b) {
    return static_cast<CriticalTrigger>(
        static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

inline CriticalTrigger operator&(CriticalTrigger a, CriticalTrigger b) {
    return static_cast<CriticalTrigger>(
        static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
}

inline bool has_trigger(CriticalTrigger flags, CriticalTrigger test) {
    return (static_cast<uint16_t>(flags) & static_cast<uint16_t>(test)) != 0;
}

// ============================================================================
// Recovery State
// ============================================================================

/// Per-slave recovery state tracked by the supervisor
enum class SlaveRecoveryState : uint8_t {
    /// Slave is operating normally (no critical condition)
    Normal      = 0,
    /// Critical condition detected, recovery pending
    Critical    = 1,
    /// Recovery in progress (slave is being re-initialized)
    Recovering  = 2,
    /// Recovery succeeded, slave is back in operation
    Recovered   = 3,
    /// Recovery failed permanently (retry limit exceeded)
    Failed      = 4,
};

// ============================================================================
// Recovery Event
// ============================================================================

/// Type of recovery event
enum class RecoveryEventType : uint8_t {
    /// Critical condition detected on a slave
    CriticalDetected  = 0,
    /// Recovery attempt started
    RecoveryStarted   = 1,
    /// Recovery attempt succeeded
    RecoverySucceeded = 2,
    /// Recovery attempt failed (will retry if attempts remain)
    RecoveryFailed    = 3,
    /// Recovery permanently failed (retry limit exhausted)
    RecoveryGaveUp    = 4,
    /// Slave PDO data suspended (motion loop must skip this slave)
    SlaveSuspended    = 5,
    /// Slave PDO data resumed (motion loop may use this slave again)
    SlaveResumed      = 6,
};

/// Human-readable name for a recovery event type
const char* recoveryEventTypeName(RecoveryEventType type);

/**
 * @brief Event payload delivered to recovery event listeners
 */
struct RecoveryEvent {
    RecoveryEventType type;           ///< Event type
    uint16_t slave_index;             ///< Affected slave index
    SlaveRecoveryState state;         ///< Current recovery state of the slave
    uint16_t al_status_code;          ///< AL_STATUS_CODE that triggered this (0 if N/A)
    int attempt;                      ///< Recovery attempt number (1-based, 0 for detection)
    int max_attempts;                 ///< Configured max attempts
    std::string_view detail;          ///< Human-readable detail string
};

// ============================================================================
// Recovery Handler Interface
// ============================================================================

/**
 * @brief Interface for re-initializing a slave from scratch after a critical
 *        condition.
 *
 * The supervisor calls this handler during recovery, after forcing the slave
 * to INIT.  The handler is responsible for performing the full
 * re-initialization sequence (mailbox config → PRE_OP → PDO config →
 * SAFE_OP → OP, plus any drive-level re-init such as CiA 402 fault reset
 * and enable).
 *
 * Implementations typically wrap DS402Master::configureDrive() +
 * enableDrive(), or the equivalent for non-DS402 slaves.
 *
 * The handler runs in a **non-realtime** context (the supervisor's recovery
 * thread or the caller's thread).  It must not be invoked from a realtime
 * loop callback.
 */
class ISlaveRecoveryHandler {
public:
    virtual ~ISlaveRecoveryHandler() = default;

    /**
     * @brief Re-initialize the slave from scratch.
     *
     * Called after the supervisor has forced the slave to INIT.
     *
     * @param slave_index  Slave to re-initialize
     * @return true if the slave is back in OP and ready for PDO exchange,
     *         false on failure.
     */
    virtual bool reinitializeSlave(uint16_t slave_index) = 0;
};

// ============================================================================
// Event Listener Interface
// ============================================================================

/**
 * @brief Interface for receiving recovery events.
 *
 * Register implementations via `SlaveSupervisor::addEventListener()`.
 * Callbacks are invoked from the supervisor's recovery context (non-realtime
 * thread).  They must not block or perform heavy work.
 */
class IRecoveryEventListener {
public:
    virtual ~IRecoveryEventListener() = default;

    /**
     * @brief Called when a recovery event occurs.
     */
    virtual void onRecoveryEvent(const RecoveryEvent& event) = 0;
};

// ============================================================================
// Recovery Configuration
// ============================================================================

/**
 * @brief Configuration for the SlaveSupervisor
 */
struct RecoveryConfig {
    /// Enable automatic recovery (default: false — opt-in)
    bool enabled = false;

    /// Which trigger categories are active (default: All)
    CriticalTrigger triggers = CriticalTrigger::All;

    /// Maximum recovery attempts before giving up (default: 3)
    /// Set to 0 for unlimited retries (not recommended).
    int max_attempts = 3;

    /// Delay between recovery attempts in milliseconds (default: 1000)
    uint32_t retry_delay_ms = 1000;

    /// Delay after forcing slave to INIT before calling the handler (default: 500)
    uint32_t post_init_delay_ms = 500;

    /// Poll interval for background monitoring in milliseconds (default: 250)
    uint32_t poll_interval_ms = 250;

    /// Whether to stop the entire motion loop during recovery (default: true)
    /// If true, all slaves are suspended during recovery of any slave.
    /// If false, only the affected slave is suspended.
    bool stop_loop_during_recovery = true;

    /// Additional AL status codes (beyond the standard critical set) that
    /// should trigger recovery.  Values are raw AL_STATUS_CODE register
    /// values (e.g. 0xAC00 for a vendor-specific code).
    std::vector<uint16_t> critical_al_codes;

    /**
     * @brief Check if a given AL status code should trigger recovery.
     */
    bool isCriticalALCode(uint16_t code) const {
        // Standard critical codes
        switch (static_cast<ALStatusCode>(code)) {
            case ALStatusCode::SlaveNeedsColdStart:  // 0x0020
            case ALStatusCode::SlaveNeedsInit:       // 0x0021
            case ALStatusCode::SlaveNeedsPreOp:      // 0x0022
            case ALStatusCode::SlaveNeedsSafeOp:     // 0x0023
            case ALStatusCode::FatalSyncError:       // 0x002C
            case ALStatusCode::NoSyncError:          // 0x002D
            case ALStatusCode::SynchronizationError: // 0x001A
                return true;
            default:
                break;
        }
        // User-configured additional codes
        for (uint16_t extra : critical_al_codes) {
            if (extra == code) return true;
        }
        return false;
    }
};

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
    /// Maximum number of slaves the supervisor can track
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
     */
    void configure(const RecoveryConfig& config);

    /// Get the current configuration
    const RecoveryConfig& config() const { return config_; }

    /// Check if the supervisor is enabled (config_.enabled == true)
    bool isEnabled() const { return config_.enabled; }

    // ----- Recovery Handler -----

    /**
     * @brief Set the recovery handler invoked during slave re-initialization.
     *
     * Must be set before recovery is attempted.  If no handler is set,
     * recovery will fail immediately.
     */
    void setRecoveryHandler(std::unique_ptr<ISlaveRecoveryHandler> handler);

    // ----- Event Listeners -----

    /**
     * @brief Register an event listener.
     * @return true on success, false if the listener is null or already registered.
     */
    bool addEventListener(IRecoveryEventListener* listener);

    /**
     * @brief Unregister an event listener.
     * @return true if the listener was found and removed.
     */
    bool removeEventListener(IRecoveryEventListener* listener);

    /// Remove all event listeners
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
     * @return SlaveRecoveryState, or Normal if the slave index is invalid.
     */
    SlaveRecoveryState slaveState(uint16_t slave_index) const;

    /**
     * @brief Get the number of recovery attempts made for a slave.
     */
    int slaveAttemptCount(uint16_t slave_index) const;

    /**
     * @brief Reset the recovery state for a slave (e.g. after manual
     *        intervention).  Does not affect an in-progress recovery.
     */
    void resetSlaveState(uint16_t slave_index);

    /// Get the number of slaves being supervised
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
