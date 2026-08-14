#pragma once

/// @file HttpServerConfig.hpp
/// @brief Configuration for the KlippyHttpServer.

#include <cstdint>
#include <string>
#include <vector>

namespace tether::klipper::http {

/// @brief Configuration for the native HTTP/WebSocket server.
struct HttpServerConfig {
    /// @brief HTTP listen port (default: 7125, Moonraker default).
    uint16_t port = 7125;

    /// @brief HTTPS listen port (default: 7130, 0 = disabled).
    uint16_t tlsPort = 0;

    /// @brief Bind address (empty = all interfaces).
    std::string bindAddress;

    /// @brief Number of Drogon worker threads (0 = auto = CPU count).
    int threads = 0;

    /// @brief API key for authentication.
    std::string apiKey = "tether_default_api_key";

    /// @brief JWT secret for token signing.
    std::string jwtSecret = "tether_jwt_secret";

    /// @brief Trusted client IP addresses (skip auth for these).
    std::vector<std::string> trustedClients = {"127.0.0.1", "::1"};

    /// @brief CORS domains (empty = allow all).
    std::vector<std::string> corsDomains = {"*"};

    /// @brief Root directory for static file serving (empty = disabled).
    std::string webRoot;

    /// @brief G-code file root directory.
    std::string gcodesRoot = "/tmp/tether_sdcard";

    /// @brief Config file root directory.
    std::string configRoot = "/etc/tether";

    /// @brief Log file root directory.
    std::string logsRoot = "/var/log";

    /// @brief TLS certificate path (empty = no TLS).
    std::string sslCertPath;

    /// @brief TLS key path (empty = no TLS).
    std::string sslKeyPath;

    /// @brief Moonraker-compatible version string reported by server/info.
    std::string moonrakerVersion = "tether-moonraker-1.0.0";

    /// @brief Whether authentication is required.
    bool requireAuth = false;

    /// @brief Maximum upload file size in bytes (0 = unlimited).
    size_t maxUploadSize = 500 * 1024 * 1024; // 500 MB
};

} // namespace tether::klipper::http
