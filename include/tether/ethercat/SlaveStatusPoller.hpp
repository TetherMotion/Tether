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

/**
 * @brief Bitmask classifying an AL state transition.
 *
 * Combine with `|` to express interest in several transition kinds in a
 * StatusFilter — e.g. `ToLowerState | ErrorSet` catches both graceful
 * downgrades and error assertions. Higher/lower refer to the AL state
 * rank order (INIT < PRE_OP < SAFE_OP < OP).
 */
enum class StatusTransitionFlags : uint8_t {
    None          = 0x00,
    ToHigherState = 0x01, /**< new state rank > old state rank */
    ToLowerState  = 0x02, /**< new state rank < old state rank */
    ErrorSet      = 0x04, /**< error bit transitioned 0 -> 1 */
    ErrorCleared  = 0x08, /**< error bit transitioned 1 -> 0 */
    AnyTransition = 0x10, /**< any state or error-flag change */
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

/**Sentinel slave index meaning "any slave" */
static constexpr uint16_t kAnySlave = 0xFFFF;

/**Sentinel SlaveState meaning "any state" (used in StatusFilter) */
static constexpr SlaveState kAnyState = static_cast<SlaveState>(0x00);

/**
 * @brief Filter for which state transitions trigger a callback
 */
struct StatusFilter {
    uint16_t slave_index = kAnySlave; /**< kAnySlave = all slaves */
    SlaveState from_state = kAnyState; /**< kAnyState = don't filter on old state */
    SlaveState to_state = kAnyState; /**< kAnyState = don't filter on new state */
    uint8_t transition_flags = static_cast<uint8_t>(StatusTransitionFlags::AnyTransition);

    /**
     * @brief Construct from transition flags only (any slave, any state).
     *
     * @param flags  Bitmask of StatusTransitionFlags the event must
     *               match — combine values with `|`.
     */
    StatusFilter(StatusTransitionFlags flags)
        : transition_flags(static_cast<uint8_t>(flags)) {}

    /**
     * @brief Construct from slave index + transition flags.
     *
     * @param slave  0-based slave index to watch, or kAnySlave for all.
     * @param flags  Bitmask of StatusTransitionFlags the event must match.
     */
    StatusFilter(uint16_t slave, StatusTransitionFlags flags)
        : slave_index(slave), transition_flags(static_cast<uint8_t>(flags)) {}

    /**
     * @brief Construct from from/to states + transition flags.
     *
     * @param from   Required old state, or kAnyState to ignore it.
     * @param to     Required new state, or kAnyState to ignore it.
     * @param flags  Bitmask of StatusTransitionFlags the event must match.
     */
    StatusFilter(SlaveState from, SlaveState to, StatusTransitionFlags flags)
        : from_state(from), to_state(to), transition_flags(static_cast<uint8_t>(flags)) {}

    /**Default construct (AnyTransition, any slave, any state) */
    StatusFilter() = default;
};

/**
 * @brief Event payload delivered to callbacks
 */
struct SlaveStatusEvent {
    uint16_t slave_index; /**< 0-based slave index */
    SlaveState old_state; /**< Previous AL state */
    SlaveState new_state; /**< Current AL state */
    uint16_t old_al_status; /**< Raw AL_STATUS register value before */
    uint16_t new_al_status; /**< Raw AL_STATUS register value now */
    bool old_error_flag; /**< Error bit (bit 4) before */
    bool new_error_flag; /**< Error bit (bit 4) now */
    uint16_t al_status_code; /**< AL_STATUS_CODE (0x0134), read when error bit is set */
};

/**Callback signature */
using StatusCallback = std::function<void(const SlaveStatusEvent&)>;

/**Opaque handle for deregistration */
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
     * Called automatically by Master during discovery.
     *
     * Sizes the per-slave state cache and readies the poller for start().
     * Slave indices ≥ @p slave_count are out of range for every query
     * and event.
     *
     * @param slave_count  Number of slaves to monitor (≤ kMaxSlaves).
     * @return true on success; false if @p slave_count exceeds kMaxSlaves
     *         or the poller is already initialized.
     */
    bool init(uint16_t slave_count);

    /**
     * @brief Shut down — stops the thread and clears all state.
     */
    void shutdown();

    bool isInitialized() const { return initialized_.load(std::memory_order_acquire); }
    uint16_t slaveCount() const { return slave_count_.load(std::memory_order_acquire); }

    // ----- Configuration -----

    /**
     * @brief Set poll interval in milliseconds (default 500).
     *
     * Must be called before start() — a running poller keeps the interval
     * it was started with. Lower values catch transitions sooner at the
     * cost of more non-realtime AL_STATUS traffic on the bus.
     *
     * @param ms  Poll period in milliseconds; must be > 0.
     */
    void setPollIntervalMs(uint32_t ms) { poll_interval_ms_ = ms; }
    uint32_t pollIntervalMs() const { return poll_interval_ms_; }

    // ----- Callback Registration -----

    /**
     * @brief Register a callback with a filter.
     *
     * The callback fires on the poller thread for each polled event
     * matching @p filter — keep it short and non-blocking, and do not
     * call back into the poller. The SlaveStatusPoller does not own the
     * callback's captured state.
     *
     * @param filter    StatusFilter describing which transitions to
     *                  deliver (slave, from/to state, transition flags).
     * @param callback  Function invoked per matching event.
     * @return Handle for later unregisterCallback(), or 0 on failure.
     */
    CallbackHandle registerCallback(const StatusFilter& filter, StatusCallback callback);

    /**
     * @brief Unregister a previously registered callback.
     *
     * @param handle  CallbackHandle returned by registerCallback().
     * @return true if the handle was found and removed.
     */
    bool unregisterCallback(CallbackHandle handle);

    /**Remove all registered callbacks. */
    void clearCallbacks();

    // ----- Thread Control -----

    bool start();
    void stop();
    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    // ----- Query -----

    /**
     * @brief Thread-safe snapshot of a slave's current cached state.
     *
     * Reflects the last AL_STATUS read by the poll thread — may lag the
     * wire state by up to one poll interval; it is a cache, not a live
     * register read.
     *
     * @param slave_index  0-based slave index (must be < slaveCount()).
     * @return Last polled SlaveState, or the initial/invalid value for
     *         an out-of-range index or before the first poll.
     */
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
        // Atomics: written by the poll thread, read by getSlaveState()
        // from any caller thread.
        std::atomic<SlaveState> state{SlaveState::INIT};
        std::atomic<uint16_t>   al_status{0};
        std::atomic<bool>       error_flag{false};
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
