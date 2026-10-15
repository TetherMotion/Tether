#pragma once

/**
 * @file FSoEChannelMonitor.hpp
 * @brief Per-channel FSoE connection-state tracking for relay/monitor
 *        applications.
 *
 * A black-channel relay observes each channel's FSoE connection only
 * through the wire frames it forwards — the master→slave command byte
 * is the channel's observable state (the master drives the FSoE state
 * machine).  `ChannelMonitor` turns that byte stream into per-channel
 * status:
 *
 *   - the current `FSoE::ConnectionState` (Reset/Session/Connection/
 *     Parameter/Data/FailSafe/Error) via `fsoeStateFromCommand()`,
 *   - how long the channel has been in that state,
 *   - one-shot "stuck" detection (non-Data state beyond a threshold,
 *     re-armed on every state change),
 *   - per-channel reset-reason tallies (the reason code the slave
 *     embeds in Reset frames) so an operator can see WHY connections
 *     restart at a glance.
 *
 * Header-only, allocation-free after construction.  Not thread-safe —
 * feed it from the monitoring thread.
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include "tether/fsoe/FSoEHelpers.hpp"

namespace FSoE {

class ChannelMonitor {
public:
    /**
     * @param channels        Number of FSoE channels to track.
     * @param stuck_after     How long a channel may sit in a non-Data,
     *                        non-FailSafe state before `update()` reports
     *                        it stuck (default 20 s).
     */
    explicit ChannelMonitor(
        size_t channels,
        std::chrono::seconds stuck_after = std::chrono::seconds(20))
        : stuck_after_(stuck_after), ch_(channels) {}

    /**
     * @brief Feed the latest master→slave command byte for channel `k`.
     *
     * Updates the derived connection state.  When the channel has sat
     * in the same non-Data, non-FailSafe state for longer than
     * `stuck_after_`, returns true exactly once (until the next state
     * change re-arms the report) so the caller can log a warning.
     *
     * @return true when the channel just became "stuck-warnable".
     */
    bool update(size_t k, uint8_t master_cmd) {
        auto& c = ch_[k];
        const auto now = std::chrono::steady_clock::now();
        const uint8_t st = fsoeStateFromCommand(master_cmd);
        c.last_cmd = master_cmd;
        if (st != c.state) {
            c.state = st;
            c.state_since = now;
            c.stuck_reported = false;
            return false;
        }
        if (!c.stuck_reported &&
            st != ConnectionState::Data &&
            st != ConnectionState::FailSafe &&
            now - c.state_since > stuck_after_) {
            c.stuck_reported = true;
            return true;
        }
        return false;
    }

    /// Direction a Reset frame travelled — master→slave resets and
    /// slave→master resets are tallied separately so a flapping
    /// connection shows which side keeps restarting it.
    enum class ResetOrigin : size_t { Master = 0, Slave = 1 };

    /// Tally a reset-reason code observed on channel `k` (the reason the
    /// endpoint embedded in its Reset frame, 0–255).
    void noteResetReason(size_t k, ResetOrigin origin, uint8_t reason) {
        ++ch_[k].reset_reasons[static_cast<size_t>(origin)][reason];
    }

    /// Occurrences of `reason` on channel `k` in direction `origin`
    /// since construction/clearResetReasons().
    uint32_t resetReasonCount(size_t k, ResetOrigin origin,
                              uint8_t reason) const {
        return ch_[k].reset_reasons[static_cast<size_t>(origin)][reason];
    }
    /// Full reason-code histogram for channel `k` / direction `origin`.
    const std::array<uint32_t, 256>& resetReasons(
        size_t k, ResetOrigin origin) const {
        return ch_[k].reset_reasons[static_cast<size_t>(origin)];
    }
    /// Clear all per-channel counters (state tracking is kept).
    void clearResetReasons() {
        for (auto& c : ch_)
            for (auto& t : c.reset_reasons) t.fill(0);
    }

    /// Derived FSoE connection state of channel `k`
    /// (`FSoE::ConnectionState` ordinal; `fsoeStateName()` formats it).
    uint8_t state(size_t k) const { return ch_[k].state; }
    /// Last master command byte fed for channel `k`.
    uint8_t lastCmd(size_t k) const { return ch_[k].last_cmd; }
    /// When the channel entered its current state.
    std::chrono::steady_clock::time_point stateSince(size_t k) const {
        return ch_[k].state_since;
    }

    size_t size() const { return ch_.size(); }

private:
    struct Channel {
        uint8_t state = 0;
        uint8_t last_cmd = 0;
        std::chrono::steady_clock::time_point state_since{};
        bool stuck_reported = false;
        // [origin][reason-code 0..255] -> count
        std::array<std::array<uint32_t, 256>, 2> reset_reasons{};
    };

    std::chrono::seconds stuck_after_;
    std::vector<Channel> ch_;
};

} // namespace FSoE
