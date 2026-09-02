/**
 * @file SlaveStatusPoller.cpp
 * @brief Implementation of background AL state monitor
 */

#include "tether/ethercat/SlaveStatusPoller.hpp"
#include "tether/platform/Platform.hpp"

#include <algorithm>
#include <cstring>
#include <chrono>

namespace EtherCAT {

static const char* TAG = "SlaveStatusPoller";

// ============================================================================
// State ranking helper
// ============================================================================

static int stateRank(SlaveState s) {
    switch (s) {
        case SlaveState::INIT:    return 1;
        case SlaveState::PRE_OP:  return 2;
        case SlaveState::BOOT:    return 3;
        case SlaveState::SAFE_OP: return 4;
        case SlaveState::OP:      return 5;
        default:                  return 0;
    }
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

SlaveStatusPoller::SlaveStatusPoller(IFaultTransport& transport)
    : transport_(transport) {
    for (auto& c : cache_) {
        c.state = SlaveState::INIT;
        c.al_status = 0;
        c.error_flag = false;
    }
}

SlaveStatusPoller::~SlaveStatusPoller() {
    stop();
}

// ============================================================================
// Lifecycle
// ============================================================================

bool SlaveStatusPoller::init(uint16_t slave_count) {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }

    slave_count_.store(std::min(slave_count, static_cast<uint16_t>(kMaxSlaves)), std::memory_order_release);

    for (auto& c : cache_) {
        c.state = SlaveState::INIT;
        c.al_status = 0;
        c.error_flag = false;
    }

    initialized_.store(true, std::memory_order_release);
    TETHER_LOGI(TAG, "Status poller initialized for {} slaves", slave_count_.load(std::memory_order_relaxed));
    return true;
}

void SlaveStatusPoller::shutdown() {
    stop();
    initialized_.store(false, std::memory_order_release);
    slave_count_.store(0, std::memory_order_release);
    clearCallbacks();
}

// ============================================================================
// Callback Registration
// ============================================================================

CallbackHandle SlaveStatusPoller::registerCallback(const StatusFilter& filter,
                                                     StatusCallback callback) {
    if (!callback) {
        return 0;
    }

    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    CallbackHandle handle = next_handle_++;
    callbacks_.push_back({handle, filter, std::move(callback)});
    return handle;
}

bool SlaveStatusPoller::unregisterCallback(CallbackHandle handle) {
    if (handle == 0) {
        return false;
    }

    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    auto it = std::find_if(callbacks_.begin(), callbacks_.end(),
        [handle](const CallbackEntry& entry) { return entry.handle == handle; });
    if (it == callbacks_.end()) {
        return false;
    }
    callbacks_.erase(it);
    return true;
}

void SlaveStatusPoller::clearCallbacks() {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    callbacks_.clear();
}

// ============================================================================
// Thread Control
// ============================================================================

bool SlaveStatusPoller::start() {
    if (running_.load(std::memory_order_acquire)) {
        TETHER_LOGW(TAG, "Status poller already running");
        return false;
    }
    if (!initialized_.load(std::memory_order_acquire) || slave_count_.load(std::memory_order_acquire) == 0) {
        TETHER_LOGW(TAG, "Cannot start: not initialized or no slaves");
        return false;
    }

    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this]() { pollLoop(); });

    TETHER_LOGI(TAG, "Status poller started (interval={} ms, slaves={})",
                poll_interval_ms_, slave_count_.load(std::memory_order_relaxed));
    return true;
}

void SlaveStatusPoller::stop() {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
}

// ============================================================================
// Query
// ============================================================================

SlaveState SlaveStatusPoller::getSlaveState(uint16_t slave_index) const {
    if (!initialized_.load(std::memory_order_acquire) || slave_index >= slave_count_.load(std::memory_order_acquire)) {
        return SlaveState::INIT;
    }
    return cache_[slave_index].state;
}

// ============================================================================
// Private: Poll Loop
// ============================================================================

void SlaveStatusPoller::pollLoop() {
    const auto period = std::chrono::milliseconds(poll_interval_ms_);
    auto next_tick = std::chrono::steady_clock::now();

    while (running_.load(std::memory_order_acquire)) {
        next_tick += period;

        for (uint16_t i = 0, n = slave_count_.load(std::memory_order_acquire); i < n; i++) {
            pollSlave(i);
        }

        std::this_thread::sleep_until(next_tick);
    }

    TETHER_LOGI(TAG, "Status poller thread exiting");
}

void SlaveStatusPoller::pollSlave(uint16_t slave_index) {
    // Read AL_STATUS (register 0x0130, 2 bytes)
    uint16_t al_status_val = 0;
    if (!transport_.readRegister(slave_index, 0x0130, &al_status_val, 2)) {
        return;
    }

    const uint8_t state_bits = static_cast<uint8_t>(al_status_val & 0x000F);
    const bool error_flag = (al_status_val & 0x0010) != 0;
    const SlaveState new_state = static_cast<SlaveState>(state_bits);

    SlaveCache& cached = cache_[slave_index];

    // Check if anything changed
    const bool state_changed = (new_state != cached.state);
    const bool error_changed = (error_flag != cached.error_flag);

    if (!state_changed && !error_changed) {
        // Update raw value but no event to dispatch
        cached.al_status = al_status_val;
        return;
    }

    // Read AL_STATUS_CODE if error bit is set
    uint16_t al_code = 0;
    if (error_flag) {
        transport_.readRegister(slave_index, 0x0134, &al_code, 2);
    }

    // Build event
    SlaveStatusEvent event;
    event.slave_index = slave_index;
    event.old_state = cached.state;
    event.new_state = new_state;
    event.old_al_status = cached.al_status;
    event.new_al_status = al_status_val;
    event.old_error_flag = cached.error_flag;
    event.new_error_flag = error_flag;
    event.al_status_code = al_code;

    // Update cache before dispatching (so callbacks see current state)
    cached.state = new_state;
    cached.al_status = al_status_val;
    cached.error_flag = error_flag;

    // Dispatch to matching callbacks
    dispatchEvent(event);
}

void SlaveStatusPoller::dispatchEvent(const SlaveStatusEvent& event) {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    for (const auto& entry : callbacks_) {
        if (filterMatches(entry.filter, event)) {
            entry.callback(event);
        }
    }
}

// ============================================================================
// Private: Filter Matching
// ============================================================================

bool SlaveStatusPoller::filterMatches(const StatusFilter& filter,
                                       const SlaveStatusEvent& event) const {
    // 1. Slave index filter
    if (filter.slave_index != kAnySlave && event.slave_index != filter.slave_index) {
        return false;
    }

    // 2. From-state filter
    if (filter.from_state != kAnyState && event.old_state != filter.from_state) {
        return false;
    }

    // 3. To-state filter
    if (filter.to_state != kAnyState && event.new_state != filter.to_state) {
        return false;
    }

    // 4. Transition flags
    const auto flags = static_cast<StatusTransitionFlags>(filter.transition_flags);

    // AnyTransition matches any state or error change
    if (has_flag(flags, StatusTransitionFlags::AnyTransition)) {
        return true;
    }

    // Check specific flags — accept if any set flag matches
    bool any_match = false;

    if (has_flag(flags, StatusTransitionFlags::ToHigherState)) {
        if (stateRank(event.new_state) > stateRank(event.old_state)) {
            any_match = true;
        }
    }

    if (has_flag(flags, StatusTransitionFlags::ToLowerState)) {
        if (stateRank(event.new_state) < stateRank(event.old_state)) {
            any_match = true;
        }
    }

    if (has_flag(flags, StatusTransitionFlags::ErrorSet)) {
        if (!event.old_error_flag && event.new_error_flag) {
            any_match = true;
        }
    }

    if (has_flag(flags, StatusTransitionFlags::ErrorCleared)) {
        if (event.old_error_flag && !event.new_error_flag) {
            any_match = true;
        }
    }

    return any_match;
}

} // namespace EtherCAT
