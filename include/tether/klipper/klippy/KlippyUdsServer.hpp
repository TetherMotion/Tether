#pragma once

/// @file KlippyUdsServer.hpp
/// @brief Unix domain socket transport layer for the Klipper/Moonraker API.
///
/// KlippyUdsServer is a thin transport that manages UDS connections, frame
/// parsing, and UDS-specific subscriptions. All business logic (endpoint
/// handlers, state, data stores) lives in KlippyServer, which is shared
/// between this transport and the HTTP/WebSocket transport.

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/UdsTypes.hpp"

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace tether::klipper::klippy {

class UdsConnection;

/// @brief Unix domain socket transport for the Klipper/Moonraker API.
///
/// This class manages:
///   - UDS socket lifecycle (create, bind, listen, accept)
///   - Connection management and frame parsing (ETX-delimited JSON)
///   - UDS-specific subscriptions (objects/subscribe, gcode/subscribe_output)
///   - Remote method registration (register_remote_method)
///   - Subscription refresh (periodic diff push to UDS clients)
///
/// All endpoint dispatch delegates to the shared KlippyServer instance.
class KlippyUdsServer {
public:
    /// @brief Construct a UDS transport with its own KlippyServer.
    explicit KlippyUdsServer(UdsServerConfig cfg = {});

    /// @brief Construct a UDS transport using an existing KlippyServer.
    /// @param server Shared business-logic server instance.
    /// @param cfg UDS transport configuration (socketPath, backlog, etc.).
    KlippyUdsServer(KlippyServer& server, UdsServerConfig cfg = {});

    ~KlippyUdsServer();

    KlippyUdsServer(const KlippyUdsServer&) = delete;
    KlippyUdsServer& operator=(const KlippyUdsServer&) = delete;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }

    // ------------------------------------------------------------------
    // Access to the underlying KlippyServer
    // ------------------------------------------------------------------

    /// @brief Get the shared KlippyServer instance.
    KlippyServer& server() { return ownsServer_ ? *ownedServer_ : *externalServer_; }
    const KlippyServer& server() const { return ownsServer_ ? *ownedServer_ : *externalServer_; }

    // ------------------------------------------------------------------
    // UDS-specific operations
    // ------------------------------------------------------------------

    /// @brief Invoke a remote method (Server -> Client push over UDS).
    void invokeRemoteMethod(const std::string& method, const JsonValue& params);

    /// @brief Get number of active UDS subscriptions.
    size_t subscriptionCount() const;

    /// @brief Manually trigger a subscription refresh (for testing).
    void refreshSubscriptions();

private:
    // ------------------------------------------------------------------
    // Frame processing
    // ------------------------------------------------------------------

    void processFrame(UdsConnection& conn, const JsonValue& frame);
    void handleRequest(UdsConnection& conn, const JsonValue& frame);
    void handleNotification(UdsConnection& conn, const JsonValue& frame);
    void sendResponse(UdsConnection& conn, const JsonValue& id, const JsonValue& result);
    void sendError(UdsConnection& conn, const JsonValue& id, const std::string& message);
    void sendPush(UdsConnection& conn, const JsonValue& msg);

    // ------------------------------------------------------------------
    // Event loop
    // ------------------------------------------------------------------

    void eventLoop();
    void acceptConnection();
    void processConnections();
    void cleanupConnections();
    void subscriptionRefreshTick();

    /// @brief Register UDS-specific callbacks on the KlippyServer.
    void registerUdsCallbacks();

    // ------------------------------------------------------------------
    // Member variables
    // ------------------------------------------------------------------

    UdsServerConfig transportConfig_;

    // Either own a KlippyServer or reference an external one
    std::unique_ptr<KlippyServer> ownedServer_;
    KlippyServer* externalServer_ = nullptr;
    bool ownsServer_ = true;

    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::thread eventThread_;

    mutable std::recursive_mutex transportMutex_;

    // Connections
    std::vector<std::unique_ptr<UdsConnection>> connections_;
    int nextConnId_ = 0;

    // UDS-specific subscriptions
    std::vector<Subscription> subscriptions_;
    std::set<UdsConnection*> gcodeSubscribers_;
    std::map<std::string, std::vector<RemoteMethod>> remoteMethods_;

    // Subscription refresh timing
    std::chrono::steady_clock::time_point lastRefresh_;
};

} // namespace tether::klipper::klippy
