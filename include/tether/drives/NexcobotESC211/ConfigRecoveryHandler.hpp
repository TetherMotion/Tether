#pragma once

/**
 * @file ConfigRecoveryHandler.hpp
 * @brief Re-writes and re-activates stored ESC211 config data on demand.
 *
 * Stores the original FNI/RSP/SDD config data (sections + CRC
 * expectations) so it can be re-written and re-activated when runtime
 * evidence — e.g. accumulating FSoE CRC mismatches tracked by
 * `CrcMismatchTracker` — suggests the device's stored configuration was
 * corrupted or lost.
 *
 * `reWriteAndActivate()`:
 *   1. Re-writes all stored sections to the device's temp objects via
 *      `ConfigManager::writeSections()`
 *   2. Activates the temp data (device command 3/4) when
 *      `Config::re_activate` is set
 *   3. Verifies CRC registers match the expected values when
 *      `Config::verify_crc` is set
 *   4. Invokes an optional post-recovery callback — typically used to
 *      re-start the safety state machine.
 *
 * SDO serialization is the caller's responsibility: pass a shared
 * `std::mutex` via `setSdoMutex()` when other threads also issue SDO
 * traffic on the same slave.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <stop_token>
#include <vector>

#include "tether/drives/NexcobotESC211/ConfigDataTypes.hpp"
#include "tether/drives/NexcobotESC211/ConfigManager.hpp"
#include "tether/drives/NexcobotESC211/CrcVerifier.hpp"

namespace EtherCAT::Drives::ESC211 {

class ConfigRecoveryHandler {
public:
    /// One data type's sections + CRC expectation.
    struct Entry {
        ConfigDataType type;
        std::vector<std::vector<uint8_t>> sections;  // 256-byte sections
        CrcExpectation crc;                          // expected CRC after activation
    };

    struct Config {
        /// If true, re-activate after re-writing.  If false, only re-write
        /// the temp objects (caller activates separately).
        bool re_activate = true;
        /// If true, verify CRC after activation.
        bool verify_crc = true;
    };

    ConfigRecoveryHandler(ConfigManager& config_mgr,
                          Config config,
                          const char* tag = "ESC211ConfigRecoveryHandler");

    /// Store config data for later re-write.  Call this once during
    /// initialization, after parsing the XML files but before (or after)
    /// the initial write+activate.
    void storeData(std::span<const Entry> entries);

    /// Re-write all stored sections to temp objects and optionally
    /// re-activate.  Returns true on success.
    /// This method is thread-safe (locks sdo_mutex_ if set).
    bool reWriteAndActivate();

    /// Returns true if storeData() has been called with at least one entry.
    bool hasData() const { return !entries_.empty(); }

    /// Number of stored data type entries.
    size_t entryCount() const { return entries_.size(); }

    /// Set a callback invoked after successful re-write + activate.
    /// Useful for re-starting the safety state machine.
    void setPostRecoveryCallback(std::function<void()> callback);

    /// Set a mutex to serialize SDO access (the master's SDO channel is
    /// not internally mutex-protected).  If set, reWriteAndActivate()
    /// locks it before any SDO operation.
    void setSdoMutex(std::mutex* mutex);

    /// Set a stop_token for cooperative cancellation.
    void setStopToken(std::stop_token token);

    /// Number of times reWriteAndActivate() has been called.
    uint32_t recoveryCount() const { return recovery_count_.load(); }

private:
    ConfigManager& config_mgr_;
    Config config_;
    const char* tag_;
    std::vector<Entry> entries_;
    std::function<void()> post_recovery_callback_;
    std::mutex* sdo_mutex_ = nullptr;
    std::stop_token stop_token_;
    std::atomic<uint32_t> recovery_count_{0};
};

} // namespace EtherCAT::Drives::ESC211
