/**
 * @file SystemStatsProvider.hpp
 * @brief Injectable system stats interface for testability.
 *
 * @details
 * Provides an abstract interface for reading system statistics (CPU load,
 * memory available, MCU stats). The default implementation reads from
 * /proc/loadavg and /proc/meminfo on Linux. Tests can inject a mock
 * implementation to control the reported values.
 */

#pragma once

#include <memory>
#include <string>

namespace tether::klipper::klippy {

/// @brief System statistics snapshot.
struct SystemStatsSnapshot {
    double sysload = 0.0;       ///< 1-minute load average
    double memAvailable = 0.0;  ///< Available memory in MB
    uint64_t mcuMcups = 0;      ///< MCU million-commands-per-second
    uint32_t mcuTaskAvg = 0;    ///< MCU average task duration (ticks)
    uint32_t mcuSbAvg = 0;      ///< MCU serial buffer average
};

/// @brief Abstract interface for reading system statistics.
class ISystemStatsProvider {
public:
    virtual ~ISystemStatsProvider() = default;

    /// @brief Read current system statistics.
    virtual SystemStatsSnapshot readStats() = 0;
};

/// @brief Default Linux implementation that reads /proc/loadavg and /proc/meminfo.
class LinuxSystemStatsProvider : public ISystemStatsProvider {
public:
    SystemStatsSnapshot readStats() override;
};

/// @brief Mock implementation for testing. Returns fixed values.
class MockSystemStatsProvider : public ISystemStatsProvider {
public:
    SystemStatsSnapshot snapshot;

    explicit MockSystemStatsProvider(SystemStatsSnapshot snap = {})
        : snapshot(std::move(snap)) {}

    SystemStatsSnapshot readStats() override { return snapshot; }

    /// @brief Set the values to be returned by readStats().
    void setStats(SystemStatsSnapshot snap) { snapshot = std::move(snap); }
};

} // namespace tether::klipper::klippy
