#pragma once

/**
 * @file StateChangeLogger.hpp
 * @brief Generic state-change-driven logging helper
 *
 * Suppresses repetitive log output when a polled value does not change.
 * The logger fires its callback when:
 *   - the state **changes** (always), or
 *   - the state **stays the same** for the Nth consecutive time
 *     (every `repeat_interval` updates, default 10).
 *
 * If `repeat_interval` is `std::nullopt`, the callback fires **only** on
 * state change — repeated unchanged values never fire.
 *
 * This is useful for polling loops (e.g. CRC verification, AL status
 * monitoring) where the value is usually unchanged on every poll and
 * logging every iteration would flood the output.
 *
 * @tparam State  The state type.  Must be equality-comparable
 *                (`operator!=`) and default-constructible.
 *
 * Usage:
 * @code
 *   EtherCAT::StateChangeLogger<uint32_t> crc_logger(
 *       [](uint32_t device_crc, int attempt, bool changed) {
 *           if (changed) {
 *               TETHER_LOGI(TAG, "CRC changed to 0x{:08X} (poll {})", device_crc, attempt);
 *           } else {
 *               TETHER_LOGI(TAG, "CRC still 0x{:08X} (poll {})", device_crc, attempt);
 *           }
 *       });
 *
 *   for (int attempt = 1; ; ++attempt) {
 *       uint32_t crc = readCrc();
 *       crc_logger.update(crc, attempt);
 *       // ...
 *   }
 * @endcode
 */

#include <functional>
#include <optional>
#include <utility>

namespace EtherCAT {

template <typename State>
class StateChangeLogger {
public:
    /// Callback invoked when a log entry should be emitted.
    ///   - `state`    : the current state value
    ///   - `attempt`  : the caller-supplied attempt / poll counter
    ///   - `changed`  : true if this update differs from the last logged state
    using LogFunc = std::function<void(const State& state, int attempt, bool changed)>;

    /**
     * @brief Construct the logger.
     *
     * @param log_func        Called when a log entry should fire.
     * @param repeat_interval Log every N-th consecutive unchanged update.
     *                        Default 10.  Pass `std::nullopt` to log ONLY
     *                        on state change (never on repeated values).
     */
    explicit StateChangeLogger(LogFunc log_func,
                              std::optional<int> repeat_interval = 10)
        : log_func_(std::move(log_func)),
          repeat_interval_(repeat_interval) {}

    /**
     * @brief Record a new state observation.
     *
     * Fires the callback immediately if there is no previous state (first
     * call) or the state differs from the last logged value.  Otherwise
     * fires every `repeat_interval`-th consecutive unchanged update.
     * If `repeat_interval` is `std::nullopt`, repeated unchanged values
     * never fire.
     *
     * @param state    The current state value.
     * @param attempt  Caller-supplied attempt counter (e.g. poll number).
     */
    void update(const State& state, int attempt) {
        if (!last_state_.has_value() || state != *last_state_) {
            log_func_(state, attempt, true);
            last_state_ = state;
            repeat_count_ = 0;
        } else if (repeat_interval_.has_value()) {
            ++repeat_count_;
            if (repeat_count_ >= *repeat_interval_) {
                log_func_(state, attempt, false);
                repeat_count_ = 0;
            }
        }
    }

    /// Reset the logger to its initial state (no previous state; next
    /// update always fires).
    void reset() {
        last_state_.reset();
        repeat_count_ = 0;
    }

    /// Check whether the logger has seen at least one update.
    bool hasInitialValue() const { return last_state_.has_value(); }

    /// Get the last logged state (undefined if hasInitialValue() is false).
    const State& lastState() const { return *last_state_; }

private:
    LogFunc log_func_;
    std::optional<int> repeat_interval_;
    std::optional<State> last_state_;
    int repeat_count_ = 0;
};

} // namespace EtherCAT
