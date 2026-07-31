/**
 * @file MultiMcuSync.hpp
 * @brief Multi-MCU synchronization for coordinated motion across multiple MCUs.
 *
 * In Klipper, multiple MCUs can be connected to a single host. Each MCU has
 * its own clock domain. This module provides:
 *   - McuInstance: represents a single MCU with its clock and connection
 *   - MultiMcuManager: coordinates clock sync and motion across MCUs
 *   - TrsyncManager: trsync (trigger synchronization) for homing across MCUs
 */

#pragma once

#include "tether/klipper/clock/ClockSync.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace tether::klipper::clock {

/// @brief Represents a single MCU instance in a multi-MCU setup.
struct McuInstance {
    std::string name;
    uint8_t mcuId = 0;
    std::shared_ptr<ClockSync> clockSync;
    uint64_t lastHeartbeat = 0;
    bool connected = false;
};

/// @brief Multi-MCU synchronization manager.
class MultiMcuManager {
public:
    /// @brief Register an MCU.
    void registerMcu(const std::string& name, uint8_t mcuId,
                     std::shared_ptr<ClockSync> clockSync) {
        std::lock_guard<std::mutex> lock(mutex_);
        McuInstance mcu;
        mcu.name = name;
        mcu.mcuId = mcuId;
        mcu.clockSync = std::move(clockSync);
        mcu.connected = true;
        mcus_[mcuId] = std::move(mcu);
    }

    /// @brief Unregister an MCU.
    void unregisterMcu(uint8_t mcuId) {
        std::lock_guard<std::mutex> lock(mutex_);
        mcus_.erase(mcuId);
    }

    /// @brief Get an MCU by ID.
    std::shared_ptr<ClockSync> getMcu(uint8_t mcuId) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mcus_.find(mcuId);
        if (it == mcus_.end()) return nullptr;
        return it->second.clockSync;
    }

    /// @brief Get all registered MCU IDs.
    std::vector<uint8_t> mcuIds() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<uint8_t> ids;
        ids.reserve(mcus_.size());
        for (const auto& [id, _] : mcus_) ids.push_back(id);
        return ids;
    }

    /// @brief Get the number of registered MCUs.
    size_t mcuCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mcus_.size();
    }

    /// @brief Convert a host timestamp to MCU clock for a specific MCU.
    uint32_t hostToMcu(uint8_t mcuId, HostTime hostTime) const {
        auto sync = getMcu(mcuId);
        if (!sync) return 0;
        return sync->hostToMcu(hostTime);
    }

    /// @brief Find the MCU with the most delayed clock (for print time alignment).
    uint8_t primaryMcu() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mcus_.empty()) return 0;
        return mcus_.begin()->second.mcuId;
    }

private:
    mutable std::mutex mutex_;
    std::map<uint8_t, McuInstance> mcus_;
};

/// @brief Trsync (trigger synchronization) manager for multi-MCU homing.
class TrsyncManager {
public:
    /// @brief Start a trsync session.
    /// @param timeoutMcuTicks Timeout in MCU ticks.
    void start(uint32_t timeoutMcuTicks) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = true;
        triggered_ = false;
        triggerMcu_ = 0xFF;
        triggerClock_ = 0;
        timeoutTicks_ = timeoutMcuTicks;
    }

    /// @brief Report a trigger from an MCU.
    void reportTrigger(uint8_t mcuId, uint32_t clock) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || triggered_) return;
        triggered_ = true;
        triggerMcu_ = mcuId;
        triggerClock_ = clock;
    }

    /// @brief Check if trsync has been triggered.
    bool isTriggered() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return triggered_;
    }

    /// @brief Check if trsync is active.
    bool isActive() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_;
    }

    /// @brief End the trsync session.
    void end() {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = false;
    }

    /// @brief Get the triggering MCU.
    uint8_t triggerMcu() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return triggerMcu_;
    }

    /// @brief Get the trigger clock.
    uint32_t triggerClock() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return triggerClock_;
    }

private:
    mutable std::mutex mutex_;
    bool active_ = false;
    bool triggered_ = false;
    uint8_t triggerMcu_ = 0xFF;
    uint32_t triggerClock_ = 0;
    uint32_t timeoutTicks_ = 0;
};

} // namespace tether::klipper::clock
