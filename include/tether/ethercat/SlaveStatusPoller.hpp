#pragma once

/**
 * @file SlaveStatusPoller.hpp
 * @brief Background AL state monitor — polls slave AL_STATUS at configurable intervals
 *
 * The SlaveStatusPoller runs a non-realtime background thread that periodically
 * reads AL_STATUS (register 0x0130) and AL_STATUS_CODE (register 0x0134) from
 * all discovered slaves.  When a state transition or error-flag change is
 * detected, registered callbacks are invoked if their filter matches.
 *
 * The poller is owned by the Master and is opt-in: the user must call start()
 * after registering callbacks.  It is automatically stopped in Master::stop().
 *
 * Usage:
 * @code
 *   auto& poller = master.statusPoller();
 *   poller.setPollIntervalMs(250);
 *
 *   StatusFilter drop_filter;
 *   drop_filter.transition_flags = static_cast<uint8_t>(StatusTransitionFlags::ToLowerState);
 *   poller.registerCallback(drop_filter, [](const SlaveStatusEvent& ev) {
 *       TETHER_LOGE("monitor", "Slave {} dropped: {} -> {}",
 *                   ev.slave_index,
 *                   slaveStateToString(ev.old_state),
 *                   slaveStateToString(ev.new_state));
 *   });
 *
 *   poller.start();
 *   // ...
 *   poller.stop();
 * @endcode
 */

#include <cstdint>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <array>

#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/FaultDetection.hpp"

namespace EtherCAT {

// ============================================================================
// Transition Flags (bitmask)
// ============================================================================

enum class StatusTransitionFlags : uint8_t {
    None          = 0x00,
    ToHigherState = 0x01,  ///< new state rank > old state rank
    ToLowerState  = 0x02,  ///< new state rank < old state rank
    ErrorSet      = 0x04,  ///< error bit transitioned 0 -> 1
    ErrorCleared  = 0x08,  ///< error bit transitioned 1 -> 0
    AnyTransition = 0x10,  ///< any state or error-flag change
};

inline StatusTransitionFlags operator|(StatusTransitionFlags a, StatusTransitionFlags b) {
    return static_cast<StatusTransitionFlags>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline StatusTransitionFlags operator&(StatusTransitionFlags a, StatusTransitionFlags b) {
    return static_cast<StatusTransitionFlags>(
        static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

inline bool has_flag(StatusTransitionFlags flags, StatusTransitionFlags test) {
    return (static_cast<uint8_t>(flags) & static_cast<uint8_t>(test)) != 0;
}

// ============================================================================
// Filter and Event
// ============================================================================

/// Sentinel slave index meaning "any slave"
static constexpr uint16_t kAnySlave = 0xFFFF;

/// Sentinel SlaveState meaning "any state" (used in StatusFilter)
static constexpr SlaveState kAnyState = static_cast<SlaveState>(0x00);

/**
 * @brief Filter for which state transitions trigger a callback
 */
struct StatusFilter {
    uint16_t slave_index = kAnySlave;       ///< kAnySlave = all slaves
    SlaveState from_state = kAnyState;       ///< kAnyState = don't filter on old state
    SlaveState to_state = kAnyState;         ///< kAnyState = don't filter on new state
    uint8_t transition_flags = static_cast<uint8_t>(StatusTransitionFlags::AnyTransition);

    /// Construct from transition flags only (any slave, any state)
    StatusFilter(StatusTransitionFlags flags)
        : transition_flags(static_cast<uint8_t>(flags)) {}

    /// Construct from slave index + transition flags
    StatusFilter(uint16_t slave, StatusTransitionFlags flags)
        : slave_index(slave), transition_flags(static_cast<uint8_t>(flags)) {}

    /// Construct from from/to states + transition flags
    StatusFilter(SlaveState from, SlaveState to, StatusTransitionFlags flags)
        : from_state(from), to_state(to), transition_flags(static_cast<uint8_t>(flags)) {}

    /// Default construct (AnyTransition, any slave, any state)
    StatusFilter() = default;
};

/**
 * @brief Event payload delivered to callbacks
 */
struct SlaveStatusEvent {
    uint16_t slave_index;           ///< 0-based slave index
    SlaveState old_state;           ///< Previous AL state
    SlaveState new_state;           ///< Current AL state
    uint16_t old_al_status;         ///< Raw AL_STATUS register value before
    uint16_t new_al_status;         ///< Raw AL_STATUS register value now
    bool old_error_flag;            ///< Error bit (bit 4) before
    bool new_error_flag;            ///< Error bit (bit 4) now
    uint16_t al_status_code;        ///< AL_STATUS_CODE (0x0134), read when error bit is set
};

/// Callback signature
using StatusCallback = std::function<void(const SlaveStatusEvent&)>;

/// Opaque handle for deregistration
using CallbackHandle = uint32_t;

// ============================================================================
// SlaveStatusPoller
// ============================================================================

/**
 * @brief Background AL state monitor with filterable callbacks
 *
 * Runs a non-realtime thread that polls AL_STATUS on all slaves at a
 * configurable interval.  Fires registered callbacks when transitions
 * match their filters.
 */
class SlaveStatusPoller {
public:
    static constexpr size_t kMaxSlaves = ECAT_STATUS_POLLER_MAX_SLAVES;

    explicit SlaveStatusPoller(IFaultTransport& transport);
    ~SlaveStatusPoller();

    // Non-copyable, non-movable
    SlaveStatusPoller(const SlaveStatusPoller&) = delete;
    SlaveStatusPoller& operator=(const SlaveStatusPoller&) = delete;
    SlaveStatusPoller(SlaveStatusPoller&&) = delete;
    SlaveStatusPoller& operator=(SlaveStatusPoller&&) = delete;

    // ----- Lifecycle -----

    /**
     * @brief Initialize for the given number of slaves.
     * Called automatically by Master during discoverSlaves().
     */
    bool init(uint16_t slave_count);

    /**
     * @brief Shut down — stops the thread and clears all state.
     */
    void shutdown();

    bool isInitialized() const { return initialized_.load(std::memory_order_acquire); }
    uint16_t slaveCount() const { return slave_count_.load(std::memory_order_acquire); }

    // ----- Configuration -----

    /// Set poll interval in milliseconds (default 500). Must be called before start().
    void setPollIntervalMs(uint32_t ms) { poll_interval_ms_ = ms; }
    uint32_t pollIntervalMs() const { return poll_interval_ms_; }

    // ----- Callback Registration -----

    /**
     * @brief Register a callback with a filter.
     * @return Handle for later unregisterCallback(), or 0 on failure.
     */
    CallbackHandle registerCallback(const StatusFilter& filter, StatusCallback callback);

    /**
     * @brief Unregister a previously registered callback.
     * @return true if the handle was found and removed.
     */
    bool unregisterCallback(CallbackHandle handle);

    /// Remove all registered callbacks.
    void clearCallbacks();

    // ----- Thread Control -----

    bool start();
    void stop();
    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    // ----- Query -----

    /// Thread-safe snapshot of a slave's current cached state.
    SlaveState getSlaveState(uint16_t slave_index) const;

private:
    void pollLoop();
    void pollSlave(uint16_t slave_index);
    void dispatchEvent(const SlaveStatusEvent& event);
    bool filterMatches(const StatusFilter& filter, const SlaveStatusEvent& event) const;

    IFaultTransport& transport_;
    std::atomic<uint16_t> slave_count_{0};
    uint32_t poll_interval_ms_ = 500;
    std::atomic<bool> initialized_{false};

    struct SlaveCache {
        SlaveState state;
        uint16_t al_status;
        bool error_flag;
    };
    std::array<SlaveCache, kMaxSlaves> cache_{};

    struct CallbackEntry {
        CallbackHandle handle;
        StatusFilter filter;
        StatusCallback callback;
    };
    mutable std::mutex callbacks_mutex_;
    std::vector<CallbackEntry> callbacks_;
    CallbackHandle next_handle_ = 1;

    std::thread thread_;
    std::atomic<bool> running_{false};
};

} // namespace EtherCAT
