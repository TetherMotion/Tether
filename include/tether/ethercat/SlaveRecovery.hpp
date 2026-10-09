/**
 * @file SlaveRecovery.hpp
 * @brief Slave recovery types: triggers, state, events, handler/listener
 *        interfaces and configuration.
 *
 * Split out of SlaveSupervisor.hpp.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tether/ethercat/FaultDetection.hpp"   // ALStatusCode
#include "tether/ethercat/SlaveStatusPoller.hpp" // SlaveStatusEvent
#include "tether/ethercat/ALResetController.hpp"

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

    /// Quiesce the cyclic/async wire exchange around the recovery
    /// handler's re-initialization (default: true).  The handler mutates
    /// the PDO mapping — suspension serializes that against the loop
    /// thread instead of relying on the epoch guard alone.  Applies to
    /// the cyclic and async loops; legacy loops use
    /// stop_loop_during_recovery.
    bool suspend_cyclic_exchange = true;

    /// Max wait for the exchange to quiesce during suspend (µs).
    uint32_t exchange_suspend_timeout_us = 20'000;

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

} // namespace EtherCAT
