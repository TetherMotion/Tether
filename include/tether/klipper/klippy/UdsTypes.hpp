#pragma once

/// @file UdsTypes.hpp
/// @brief Supporting types for the UDS server (handlers, subscriptions, config).

#include "tether/klipper/klippy/JsonValue.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// Endpoint handler
// ============================================================================

using EndpointHandler = std::function<JsonValue(const JsonValue& params)>;

/// @brief Error thrown by endpoint handlers to produce an error response.
class EndpointError : public std::runtime_error {
public:
    explicit EndpointError(const std::string& msg)
        : std::runtime_error(msg) {}
};

// ============================================================================
// Connection (internal)
// ============================================================================

class UdsConnection;

// ============================================================================
// Subscription
// ============================================================================

struct Subscription {
    /// @brief Objects and fields the subscriber wants (empty = all).
    std::map<std::string, std::vector<std::string>> objects;

    /// @brief Response template for push messages.
    JsonValue responseTemplate;

    /// @brief Last-pushed baseline for diff computation.
    std::map<std::string, std::map<std::string, JsonValue>> baseline;

    /// @brief Connection that owns this subscription.
    UdsConnection* conn = nullptr;
};

// ============================================================================
// Remote method registration
// ============================================================================

struct RemoteMethod {
    std::string name;
    JsonValue responseTemplate;
    UdsConnection* conn = nullptr;
};

// ============================================================================
// KlippyUdsServer
// ============================================================================

/// @brief Configuration for the UDS server.
struct UdsServerConfig {
    /// @brief Socket file path (default: /tmp/klippy_uds).
    std::string socketPath = "/tmp/klippy_uds";

    /// @brief Listen backlog.
    int backlog = 1;

    /// @brief Subscription refresh interval in milliseconds.
    int refreshIntervalMs = 250;

    /// @brief Software version string reported by info endpoint.
    std::string softwareVersion = "tether-klippy-1.0.0";

    /// @brief Klipper source path.
    std::string klipperPath = "/usr/lib/tether";

    /// @brief Python path (for compatibility).
    std::string pythonPath = "/usr/bin/python3";

    /// @brief Config file path.
    std::string configFile = "/etc/tether/printer.cfg";

    /// @brief Log file path.
    std::string logFile = "/var/log/tether-klippy.log";

    /// @brief Spoolman server URL (empty = disabled).
    std::string spoolmanUrl;
};

} // namespace tether::klipper::klippy
