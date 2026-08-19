/**
 * @file SlaveSupervisor.cpp
 * @brief Implementation of SlaveSupervisor — critical-condition detection
 *        and automatic slave recovery.
 */

#include "tether/ethercat/SlaveSupervisor.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/platform/Platform.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace EtherCAT {

static const char* TAG = "slave_supervisor";

// ============================================================================
// Recovery event type names
// ============================================================================

const char* recoveryEventTypeName(RecoveryEventType type) {
    switch (type) {
        case RecoveryEventType::CriticalDetected:  return "CriticalDetected";
        case RecoveryEventType::RecoveryStarted:   return "RecoveryStarted";
        case RecoveryEventType::RecoverySucceeded: return "RecoverySucceeded";
        case RecoveryEventType::RecoveryFailed:    return "RecoveryFailed";
        case RecoveryEventType::RecoveryGaveUp:    return "RecoveryGaveUp";
        case RecoveryEventType::SlaveSuspended:    return "SlaveSuspended";
        case RecoveryEventType::SlaveResumed:      return "SlaveResumed";
    }
    return "Unknown";
}

// ============================================================================
// Construction / Destruction
// ============================================================================

SlaveSupervisor::SlaveSupervisor(Master& master)
    : master_(master)
{
}

SlaveSupervisor::~SlaveSupervisor() {
    stop();
}

// ============================================================================
// Configuration
// ============================================================================

void SlaveSupervisor::configure(const RecoveryConfig& config) {
    config_ = config;
}

void SlaveSupervisor::setRecoveryHandler(std::unique_ptr<ISlaveRecoveryHandler> handler) {
    recovery_handler_ = std::move(handler);
}

// ============================================================================
// Event Listeners
// ============================================================================

bool SlaveSupervisor::addEventListener(IRecoveryEventListener* listener) {
    if (listener == nullptr) return false;
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    if (std::find(listeners_.begin(), listeners_.end(), listener) != listeners_.end()) {
        return false;  // Already registered
    }
    listeners_.push_back(listener);
    return true;
}

bool SlaveSupervisor::removeEventListener(IRecoveryEventListener* listener) {
    if (listener == nullptr) return false;
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    auto it = std::find(listeners_.begin(), listeners_.end(), listener);
    if (it == listeners_.end()) return false;
    listeners_.erase(it);
    return true;
}

void SlaveSupervisor::clearEventListeners() {
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    listeners_.clear();
}

// ============================================================================
// Lifecycle
// ============================================================================

bool SlaveSupervisor::init(uint16_t slave_count) {
    if (slave_count > kMaxSlaves) {
        TETHER_LOGE(TAG, "init: slave_count %u exceeds max %zu", slave_count, kMaxSlaves);
        return false;
    }

    // Reset all per-slave state
    for (size_t i = 0; i < kMaxSlaves; ++i) {
        slave_states_[i].state.store(SlaveRecoveryState::Normal, std::memory_order_relaxed);
        slave_states_[i].suspended.store(false, std::memory_order_relaxed);
        slave_states_[i].attempt_count.store(0, std::memory_order_relaxed);
        slave_states_[i].last_al_status_code = 0;
        slave_states_[i].last_detail.clear();
    }

    slave_count_.store(slave_count, std::memory_order_release);
    initialized_.store(true, std::memory_order_release);
    return true;
}

bool SlaveSupervisor::start() {
    if (running_.load(std::memory_order_acquire)) {
        TETHER_LOGW(TAG, "start: supervisor already running");
        return false;
    }

    if (!config_.enabled) {
        TETHER_LOGW(TAG, "start: supervisor is disabled (config_.enabled == false)");
        return false;
    }

    if (!recovery_handler_) {
        TETHER_LOGE(TAG, "start: no recovery handler set — cannot start");
        return false;
    }

    if (!initialized_.load(std::memory_order_acquire)) {
        // Auto-init from discovered slave count
        const uint16_t count = master_.getDiscoveredSlaveCount();
        if (count == 0) {
            TETHER_LOGE(TAG, "start: no slaves discovered — cannot start");
            return false;
        }
        if (!init(count)) {
            return false;
        }
    }

    // Register with the SlaveStatusPoller for state-drop and error events
    auto& poller = master_.statusPoller();
    poller.setPollIntervalMs(config_.poll_interval_ms);

    StatusFilter filter;
    filter.transition_flags = static_cast<uint8_t>(
        StatusTransitionFlags::ToLowerState |
        StatusTransitionFlags::ErrorSet |
        StatusTransitionFlags::AnyTransition);
    poller_handle_ = poller.registerCallback(filter,
        [this](const SlaveStatusEvent& ev) { handleStatusEvent(ev); });

    if (poller_handle_ == 0) {
        TETHER_LOGE(TAG, "start: failed to register poller callback");
        return false;
    }

    // Ensure the poller is running
    if (!poller.isRunning()) {
        if (!poller.start()) {
            TETHER_LOGE(TAG, "start: failed to start status poller");
            poller.unregisterCallback(poller_handle_);
            poller_handle_ = 0;
            return false;
        }
    }

    running_.store(true, std::memory_order_release);
    TETHER_LOGI(TAG, "Supervisor started: %u slave(s), max_attempts=%d, triggers=0x%04X",
                slaveCount(), config_.max_attempts,
                static_cast<uint16_t>(config_.triggers));
    return true;
}

void SlaveSupervisor::stop() {
    if (!running_.load(std::memory_order_acquire)) return;

    running_.store(false, std::memory_order_release);

    // Unregister from the poller
    if (poller_handle_ != 0) {
        master_.statusPoller().unregisterCallback(poller_handle_);
        poller_handle_ = 0;
    }

    // Clear all suspended flags
    for (size_t i = 0; i < kMaxSlaves; ++i) {
        slave_states_[i].suspended.store(false, std::memory_order_relaxed);
    }

    TETHER_LOGI(TAG, "Supervisor stopped");
}

// ============================================================================
// Critical Condition Reporting
// ============================================================================

void SlaveSupervisor::handleStatusEvent(const SlaveStatusEvent& event) {
    if (!running_.load(std::memory_order_acquire)) return;
    if (event.slave_index >= slaveCount()) return;

    // Already in recovery or failed — don't re-trigger
    const auto state = slave_states_[event.slave_index].state.load(std::memory_order_acquire);
    if (state == SlaveRecoveryState::Recovering || state == SlaveRecoveryState::Failed) {
        return;
    }

    bool should_recover = false;
    std::string detail;

    // Check for error flag set with a critical AL status code
    if (event.new_error_flag && event.al_status_code != 0) {
        if (has_trigger(config_.triggers, CriticalTrigger::ALStatusCodes) &&
            config_.isCriticalALCode(event.al_status_code)) {
            should_recover = true;
            detail = std::format("AL status code 0x{:04X} ({}) with error flag",
                                 event.al_status_code,
                                 getALStatusCodeName(event.al_status_code));
        }
    }

    // Check for state drop to INIT or PRE_OP (transition failure)
    if (!should_recover && has_trigger(config_.triggers, CriticalTrigger::TransitionFailures)) {
        const auto new_state_val = static_cast<uint8_t>(event.new_state);
        const auto old_state_val = static_cast<uint8_t>(event.old_state);

        // State rank: INIT=1, PRE_OP=2, BOOT=3, SAFE_OP=4, OP=8
        // A drop to INIT or PRE_OP from a higher state is critical
        const bool dropped_to_init = (new_state_val == static_cast<uint8_t>(SlaveState::INIT));
        const bool dropped_to_preop = (new_state_val == static_cast<uint8_t>(SlaveState::PRE_OP));
        const bool was_higher = (old_state_val == static_cast<uint8_t>(SlaveState::SAFE_OP) ||
                                  old_state_val == static_cast<uint8_t>(SlaveState::OP));

        if ((dropped_to_init || dropped_to_preop) && was_higher) {
            should_recover = true;
            detail = std::format("Slave dropped from {} to {}",
                                 slaveStateToString(event.old_state),
                                 slaveStateToString(event.new_state));
        }
    }

    if (should_recover) {
        slave_states_[event.slave_index].last_al_status_code = event.al_status_code;
        slave_states_[event.slave_index].last_detail = detail;
        attemptRecovery(event.slave_index);
    }
}

void SlaveSupervisor::handleTransitionFailure(uint16_t slave_index,
                                               std::string_view detail,
                                               uint16_t al_status_code) {
    if (!running_.load(std::memory_order_acquire)) return;
    if (slave_index >= slaveCount()) return;
    if (!has_trigger(config_.triggers, CriticalTrigger::TransitionFailures)) return;

    // Already in recovery or failed — don't re-trigger
    const auto state = slave_states_[slave_index].state.load(std::memory_order_acquire);
    if (state == SlaveRecoveryState::Recovering || state == SlaveRecoveryState::Failed) {
        return;
    }

    slave_states_[slave_index].last_al_status_code = al_status_code;
    slave_states_[slave_index].last_detail = std::string(detail);
    attemptRecovery(slave_index);
}

void SlaveSupervisor::markCritical(uint16_t slave_index, std::string_view detail) {
    if (!running_.load(std::memory_order_acquire)) return;
    if (slave_index >= slaveCount()) return;
    if (!has_trigger(config_.triggers, CriticalTrigger::AppInjected)) return;

    // Already in recovery or failed — don't re-trigger
    const auto state = slave_states_[slave_index].state.load(std::memory_order_acquire);
    if (state == SlaveRecoveryState::Recovering || state == SlaveRecoveryState::Failed) {
        return;
    }

    slave_states_[slave_index].last_al_status_code = 0;
    slave_states_[slave_index].last_detail = std::string(detail);
    attemptRecovery(slave_index);
}

// ============================================================================
// Motion-Loop Integration
// ============================================================================

bool SlaveSupervisor::isSlaveSuspended(uint16_t slave_index) const {
    if (slave_index >= kMaxSlaves) return false;
    return slave_states_[slave_index].suspended.load(std::memory_order_acquire);
}

bool SlaveSupervisor::isSlaveRecovering(uint16_t slave_index) const {
    if (slave_index >= kMaxSlaves) return false;
    return slave_states_[slave_index].state.load(std::memory_order_acquire) ==
           SlaveRecoveryState::Recovering;
}

SlaveRecoveryState SlaveSupervisor::slaveState(uint16_t slave_index) const {
    if (slave_index >= kMaxSlaves) return SlaveRecoveryState::Normal;
    return slave_states_[slave_index].state.load(std::memory_order_acquire);
}

int SlaveSupervisor::slaveAttemptCount(uint16_t slave_index) const {
    if (slave_index >= kMaxSlaves) return 0;
    return slave_states_[slave_index].attempt_count.load(std::memory_order_acquire);
}

void SlaveSupervisor::resetSlaveState(uint16_t slave_index) {
    if (slave_index >= kMaxSlaves) return;
    // Don't reset if currently recovering
    if (slave_states_[slave_index].state.load(std::memory_order_acquire) ==
        SlaveRecoveryState::Recovering) {
        return;
    }
    slave_states_[slave_index].state.store(SlaveRecoveryState::Normal,
                                            std::memory_order_release);
    slave_states_[slave_index].suspended.store(false, std::memory_order_release);
    slave_states_[slave_index].attempt_count.store(0, std::memory_order_release);
    slave_states_[slave_index].last_al_status_code = 0;
    slave_states_[slave_index].last_detail.clear();
}

// ============================================================================
// Internal: Recovery Orchestration
// ============================================================================

bool SlaveSupervisor::shouldTriggerRecovery(CriticalTrigger trigger_category,
                                              uint16_t al_status_code) const {
    if (!has_trigger(config_.triggers, trigger_category)) return false;
    if (trigger_category == CriticalTrigger::ALStatusCodes &&
        al_status_code != 0 && !config_.isCriticalALCode(al_status_code)) {
        return false;
    }
    return true;
}

void SlaveSupervisor::dispatchEvent(RecoveryEventType type, uint16_t slave_index,
                                      uint16_t al_code, int attempt,
                                      std::string_view detail) {
    RecoveryEvent event;
    event.type = type;
    event.slave_index = slave_index;
    event.state = slave_states_[slave_index].state.load(std::memory_order_relaxed);
    event.al_status_code = al_code;
    event.attempt = attempt;
    event.max_attempts = config_.max_attempts;
    event.detail = detail;

    std::lock_guard<std::mutex> lock(listeners_mutex_);
    for (auto* listener : listeners_) {
        if (listener) listener->onRecoveryEvent(event);
    }
}

void SlaveSupervisor::setSuspended(uint16_t slave_index, bool suspended) {
    slave_states_[slave_index].suspended.store(suspended, std::memory_order_release);
    dispatchEvent(suspended ? RecoveryEventType::SlaveSuspended
                             : RecoveryEventType::SlaveResumed,
                  slave_index, 0, 0,
                  suspended ? "PDO data suspended" : "PDO data resumed");
}

bool SlaveSupervisor::forceSlaveToInit(uint16_t slave_index) {
    // Use ALResetController for a robust two-step AL reset to INIT
    ALResetController ctrl(master_);
    auto result = ctrl.resetSlave(slave_index,
                                   static_cast<uint8_t>(SlaveState::INIT),
                                   20,  // max iterations
                                   50); // sleep ms
    if (!result.success) {
        TETHER_LOGE(TAG, "%s: Failed to force to INIT: %s",
                    master_.slaveLogPrefix(slave_index).c_str(), result.message.c_str());
        return false;
    }

    // Also reset the Slave object's internal config flags
    auto& slave = master_.slave(slave_index);
    (void)slave.transitionToInit();

    TETHER_LOGI(TAG, "%s: Forced to INIT (%s)",
                master_.slaveLogPrefix(slave_index).c_str(), result.message.c_str());
    return true;
}

bool SlaveSupervisor::attemptRecovery(uint16_t slave_index) {
    auto& ss = slave_states_[slave_index];

    // Lock per-slave recovery mutex to serialize attempts.
    // This mutex is held for the entire retry loop, preventing concurrent
    // recovery attempts on the same slave (e.g. from poller + app).
    std::lock_guard<std::mutex> recovery_lock(ss.recovery_mutex);

    const int max = config_.max_attempts;

    TETHER_LOGE(TAG, "%s: Critical condition detected: %s (AL code 0x%04X)",
                master_.slaveLogPrefix(slave_index).c_str(), ss.last_detail.c_str(), ss.last_al_status_code);
    dispatchEvent(RecoveryEventType::CriticalDetected, slave_index,
                  ss.last_al_status_code, 0, ss.last_detail);

    // Retry loop: attempt recovery up to max_attempts times.
    for (;;) {
        // Check retry limit
        const int current_attempt = ss.attempt_count.fetch_add(1, std::memory_order_acq_rel) + 1;

        if (max > 0 && current_attempt > max) {
            ss.state.store(SlaveRecoveryState::Failed, std::memory_order_release);
            TETHER_LOGE(TAG, "%s: Recovery permanently failed after %d attempts",
                        master_.slaveLogPrefix(slave_index).c_str(), max);
            dispatchEvent(RecoveryEventType::RecoveryGaveUp, slave_index,
                          ss.last_al_status_code, current_attempt - 1,
                          "Retry limit exhausted");
            ss.suspended.store(true, std::memory_order_release);  // Stay suspended
            return false;
        }

        // Enter recovering state
        ss.state.store(SlaveRecoveryState::Recovering, std::memory_order_release);

        TETHER_LOGI(TAG, "%s: Recovery attempt %d/%d started",
                    master_.slaveLogPrefix(slave_index).c_str(), current_attempt, max > 0 ? max : -1);
        dispatchEvent(RecoveryEventType::RecoveryStarted, slave_index,
                      ss.last_al_status_code, current_attempt, ss.last_detail);

        // Suspend PDO data for this slave
        setSuspended(slave_index, true);

        // If configured, suspend all slaves (stop-loop-during-recovery mode)
        if (config_.stop_loop_during_recovery) {
            const uint16_t count = slaveCount();
            for (uint16_t i = 0; i < count; ++i) {
                if (i != slave_index) {
                    setSuspended(i, true);
                }
            }
        }

        // --- Force slave to INIT ---
        if (!forceSlaveToInit(slave_index)) {
            TETHER_LOGE(TAG, "%s: Failed to force to INIT on attempt %d",
                        master_.slaveLogPrefix(slave_index).c_str(), current_attempt);
            dispatchEvent(RecoveryEventType::RecoveryFailed, slave_index,
                          ss.last_al_status_code, current_attempt,
                          "Failed to force slave to INIT");
            // Fall through to retry logic below
        } else {
            // Delay after INIT
            if (config_.post_init_delay_ms > 0) {
                Tether::Platform::Clock::instance().delayMilliseconds(
                    config_.post_init_delay_ms);
            }

            // --- Call the recovery handler to re-initialize from scratch ---
            if (!recovery_handler_) {
                TETHER_LOGE(TAG, "%s: No recovery handler set", master_.slaveLogPrefix(slave_index).c_str());
                ss.state.store(SlaveRecoveryState::Failed, std::memory_order_release);
                dispatchEvent(RecoveryEventType::RecoveryGaveUp, slave_index,
                              ss.last_al_status_code, current_attempt,
                              "No recovery handler");
                return false;
            }

            bool ok = false;
            try {
                ok = recovery_handler_->reinitializeSlave(slave_index);
            } catch (const std::exception& e) {
                TETHER_LOGE(TAG, "%s: Recovery handler threw: %s",
                            master_.slaveLogPrefix(slave_index).c_str(), e.what());
                ok = false;
            } catch (...) {
                TETHER_LOGE(TAG, "%s: Recovery handler threw unknown exception",
                            master_.slaveLogPrefix(slave_index).c_str());
                ok = false;
            }

            if (ok) {
                // Success — resume PDO data
                setSuspended(slave_index, false);
                if (config_.stop_loop_during_recovery) {
                    const uint16_t count = slaveCount();
                    for (uint16_t i = 0; i < count; ++i) {
                        if (i != slave_index) {
                            setSuspended(i, false);
                        }
                    }
                }
                ss.state.store(SlaveRecoveryState::Normal, std::memory_order_release);
                ss.attempt_count.store(0, std::memory_order_release);

                TETHER_LOGI(TAG, "%s: Recovery succeeded on attempt %d",
                            master_.slaveLogPrefix(slave_index).c_str(), current_attempt);
                dispatchEvent(RecoveryEventType::RecoverySucceeded, slave_index,
                              0, current_attempt, "Slave re-initialized successfully");
                return true;
            }

            TETHER_LOGE(TAG, "%s: Recovery handler failed on attempt %d",
                        master_.slaveLogPrefix(slave_index).c_str(), current_attempt);
            dispatchEvent(RecoveryEventType::RecoveryFailed, slave_index,
                          ss.last_al_status_code, current_attempt,
                          "Recovery handler returned false");
        }

        // Check if we've exhausted retries
        if (max > 0 && current_attempt >= max) {
            ss.state.store(SlaveRecoveryState::Failed, std::memory_order_release);
            TETHER_LOGE(TAG, "%s: Giving up after %d attempts", master_.slaveLogPrefix(slave_index).c_str(), max);
            dispatchEvent(RecoveryEventType::RecoveryGaveUp, slave_index,
                          ss.last_al_status_code, current_attempt,
                          "Retry limit exhausted");
            // Stay suspended
            return false;
        }

        // Delay before retry
        ss.state.store(SlaveRecoveryState::Critical, std::memory_order_release);
        if (config_.retry_delay_ms > 0) {
            Tether::Platform::Clock::instance().delayMilliseconds(
                config_.retry_delay_ms);
        }
        // Loop continues to next attempt
    }
}

} // namespace EtherCAT
