/**
 * @file KlippyExpected.hpp
 * @brief std::expected-based wrappers for KlippyHost fallible operations.
 *
 * @details
 * Provides `tryConnect()`, `tryDownloadDictionary()`, `trySyncClock()`, and
 * `trySendCommand()` that return `std::expected<T, KlipperError>` instead of
 * bool. These wrap the existing bool-returning methods, preserving backward
 * compatibility while enabling structured error handling for new code.
 *
 * Example usage:
 *   auto result = tryConnect(host);
 *   if (!result) {
 *       std::cerr << result.error().format() << "\n";
 *       return;
 *   }
 */

#pragma once

#include "tether/klipper/KlipperError.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"

#include <expected>
#include <functional>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Attempt to connect the host. Returns error on failure.
inline std::expected<void, KlipperError> tryConnect(KlippyHost& host) {
    if (host.connect()) return {};
    return std::unexpected(KlipperError::transport("KlippyHost::connect() failed"));
}

/// @brief Attempt to download the data dictionary. Returns error on failure.
inline std::expected<void, KlipperError>
tryDownloadDictionary(KlippyHost& host, std::function<void()> devicePump = nullptr) {
    if (host.downloadDictionary(std::move(devicePump))) return {};
    return std::unexpected(KlipperError::dictionary(
        "KlippyHost::downloadDictionary() failed - identify handshake did not complete"));
}

/// @brief Attempt to synchronize the clock. Returns error on failure.
inline std::expected<void, KlipperError>
trySyncClock(KlippyHost& host, std::function<void()> devicePump = nullptr) {
    if (host.syncClock(std::move(devicePump))) return {};
    return std::unexpected(KlipperError::clockSync(
        "KlippyHost::syncClock() failed - clock synchronization did not converge"));
}

/// @brief Attempt to send a command. Returns error on failure.
inline std::expected<void, KlipperError>
trySendCommand(KlippyHost& host,
               const std::string& formatStr,
               const std::vector<protocol::ParamValue>& params) {
    if (host.sendCommand(formatStr, params)) return {};
    return std::unexpected(KlipperError::protocol(
        "KlippyHost::sendCommand() failed for: " + formatStr));
}

/// @brief Attempt to send a command with no parameters. Returns error on failure.
inline std::expected<void, KlipperError>
trySendCommand(KlippyHost& host, const std::string& formatStr) {
    return trySendCommand(host, formatStr, {});
}

} // namespace tether::klipper::klippy
