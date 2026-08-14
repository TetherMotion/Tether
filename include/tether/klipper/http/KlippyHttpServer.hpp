#pragma once

/// @file KlippyHttpServer.hpp
/// @brief Native HTTP/WebSocket server for Mainsail/Fluidd frontends.
///
/// This server implements the full Moonraker HTTP + WebSocket API directly
/// in C++ using Drogon as the web framework and Glaze for JSON serialization.
/// It reuses all existing KlippyUdsServer endpoint handlers via callEndpoint(),
/// so there is zero business-logic duplication.
///
/// The server provides:
///   - REST HTTP endpoints (GET/POST/DELETE) for all Moonraker APIs
///   - WebSocket endpoint at /websocket for real-time JSON-RPC 2.0
///   - JSON-RPC 2.0 endpoint at POST /server/jsonrpc
///   - File upload (multipart/form-data) and download
///   - Authentication (API key, JWT, oneshot token, trusted clients)
///   - CORS support
///   - Static asset serving for SPA frontends
///   - All 26+ WebSocket notification types

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/http/HttpServerConfig.hpp"
#include "tether/klipper/http/JsonRpcDispatcher.hpp"
#include "tether/klipper/http/WsSessionManager.hpp"
#include "tether/klipper/http/NotificationSink.hpp"

#include <drogon/drogon.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace tether::klipper::http {

class AuthFilter;

/// @brief Native HTTP/WebSocket server implementing the Moonraker API.
///
/// Wraps Drogon's HttpAppFramework and registers all routes, controllers,
/// and filters needed to serve Mainsail/Fluidd directly.
class KlippyHttpServer : public std::enable_shared_from_this<KlippyHttpServer> {
public:
    /// @brief Construct with a reference to the UDS server and config.
    /// @param udsServer The existing KlippyUdsServer (must outlive this).
    /// @param cfg HTTP server configuration.
    explicit KlippyHttpServer(klippy::KlippyUdsServer& udsServer,
                               HttpServerConfig cfg = {});

    ~KlippyHttpServer();

    KlippyHttpServer(const KlippyHttpServer&) = delete;
    KlippyHttpServer& operator=(const KlippyHttpServer&) = delete;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /// @brief Register all routes and start the HTTP server.
    /// @return True on success.
    bool start();

    /// @brief Stop the HTTP server.
    void stop();

    /// @brief Check if the server is running.
    bool isRunning() const { return running_.load(); }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    const HttpServerConfig& config() const { return config_; }
    WsSessionManager& wsSessions() { return wsSessions_; }

    /// @brief Get the JSON-RPC dispatcher (for testing).
    JsonRpcDispatcher& dispatcher() { return dispatcher_; }

    /// @brief Get the NotificationSink for this server (for registering with UDS server).
    NotificationSink* notificationSink() { return notificationBridge_.get(); }

    // ------------------------------------------------------------------
    // Auth (public for AuthFilter access)
    // ------------------------------------------------------------------

    bool checkAuth(const drogon::HttpRequestPtr& req) const;
    void addCorsHeaders(const drogon::HttpResponsePtr& resp,
                        const drogon::HttpRequestPtr& req) const;

    // ------------------------------------------------------------------
    // WebSocket handling (public for KlippyWsController access)
    // ------------------------------------------------------------------

    void handleWsMessage(const drogon::WebSocketConnectionPtr& conn,
                         std::string&& message);
    void handleWsNewConnection(const drogon::WebSocketConnectionPtr& conn);
    void handleWsDisconnect(const drogon::WebSocketConnectionPtr& conn);

private:
    // ------------------------------------------------------------------
    // Setup
    // ------------------------------------------------------------------

    void registerRoutes();
    void registerRestRoutes();
    void registerFileRoutes();
    void registerStubRoutes();
    void registerStaticAssets();
    void registerWebSocket();
    void registerJsonRpcEndpoint();

    // ------------------------------------------------------------------
    // REST handler helpers
    // ------------------------------------------------------------------

    /// Call a UDS endpoint by slash-method name and return a Drogon response.
    drogon::HttpResponsePtr callEndpointAndBuildResponse(
        const std::string& method, const klippy::JsonValue& params);

    /// Build params from query parameters.
    klippy::JsonValue paramsFromQuery(const drogon::HttpRequestPtr& req);

    /// Build params from JSON body.
    klippy::JsonValue paramsFromBody(const drogon::HttpRequestPtr& req);

    /// Build params from both query and body (body takes precedence).
    klippy::JsonValue paramsFromRequest(const drogon::HttpRequestPtr& req);

    // ------------------------------------------------------------------
    // REST endpoint handler factories
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // File handlers
    // ------------------------------------------------------------------

    void handleFileUpload(const drogon::HttpRequestPtr& req,
                          std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleFileDownload(const drogon::HttpRequestPtr& req,
                            std::function<void(const drogon::HttpResponsePtr&)>&& callback,
                            const std::string& root, const std::string& path);
    void handleDeleteFile(const drogon::HttpRequestPtr& req,
                          std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleDirectoryPost(const drogon::HttpRequestPtr& req,
                             std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleDirectoryDelete(const drogon::HttpRequestPtr& req,
                               std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleFileZip(const drogon::HttpRequestPtr& req,
                       std::function<void(const drogon::HttpResponsePtr&)>&& callback);

    // ------------------------------------------------------------------
    // HTTP-only handlers (no UDS equivalent)
    // ------------------------------------------------------------------

    void handleDatabaseItemGet(const drogon::HttpRequestPtr& req,
                               std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleDatabaseItemPost(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleDatabaseItemDelete(const drogon::HttpRequestPtr& req,
                                  std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleHistoryList(const drogon::HttpRequestPtr& req,
                           std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleHistoryTotals(const drogon::HttpRequestPtr& req,
                             std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleHistoryJobGet(const drogon::HttpRequestPtr& req,
                             std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleHistoryJobDelete(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleJobQueueJobPost(const drogon::HttpRequestPtr& req,
                               std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleJobQueueJobDelete(const drogon::HttpRequestPtr& req,
                                 std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleWebcamItemGet(const drogon::HttpRequestPtr& req,
                             std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleWebcamItemPost(const drogon::HttpRequestPtr& req,
                              std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleWebcamItemDelete(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleAccessUserPost(const drogon::HttpRequestPtr& req,
                              std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleAccessUserDelete(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleAccessUserPassword(const drogon::HttpRequestPtr& req,
                                  std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleAccessUsersList(const drogon::HttpRequestPtr& req,
                               std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleAccessApiKeyPost(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleAccessInfo(const drogon::HttpRequestPtr& req,
                          std::function<void(const drogon::HttpResponsePtr&)>&& callback);

    // ------------------------------------------------------------------
    // Stub handlers (sensors, wled, mqtt, extensions, timelapse, sudo, octoprint)
    // ------------------------------------------------------------------

    void handleStubEmpty(const drogon::HttpRequestPtr& req,
                         std::function<void(const drogon::HttpResponsePtr&)>&& callback,
                         const std::string& componentName);
    void handleOctoprintVersion(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleOctoprintServer(const drogon::HttpRequestPtr& req,
                               std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleOctoprintLogin(const drogon::HttpRequestPtr& req,
                              std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleOctoprintPrinter(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleOctoprintJob(const drogon::HttpRequestPtr& req,
                            std::function<void(const drogon::HttpResponsePtr&)>&& callback);
    void handleOctoprintSettings(const drogon::HttpRequestPtr& req,
                                 std::function<void(const drogon::HttpResponsePtr&)>&& callback);

    // ------------------------------------------------------------------
    // WebSocket connection handling
    // ------------------------------------------------------------------

    void handleWebSocketConnect(const drogon::HttpRequestPtr& req,
                                std::function<void(const drogon::HttpResponsePtr&)>&& callback);

    // ------------------------------------------------------------------
    // JSON-RPC endpoint
    // ------------------------------------------------------------------

    void handleJsonRpc(const drogon::HttpRequestPtr& req,
                       std::function<void(const drogon::HttpResponsePtr&)>&& callback);

    // ------------------------------------------------------------------
    // Subscription refresh
    // ------------------------------------------------------------------

    void subscriptionRefreshTick();

    // ------------------------------------------------------------------
    // Auth helpers
    // ------------------------------------------------------------------

    bool isTrustedClient(const std::string& ip) const;
    bool checkApiKey(const std::string& key) const;
    bool checkJwt(const std::string& token) const;
    bool checkOneshotToken(const std::string& token, const std::string& ip);
    std::string generateApiKey() const;
    std::string generateOneshotToken(const std::string& ip);
    std::string generateJwt(const std::string& username) const;

    // ------------------------------------------------------------------
    // CORS helpers
    // ------------------------------------------------------------------

    bool isCorsAllowed(const std::string& origin) const;

    // ------------------------------------------------------------------
    // Internal state
    // ------------------------------------------------------------------

    klippy::KlippyUdsServer& udsServer_;
    HttpServerConfig config_;
    std::atomic<bool> running_{false};

    JsonRpcDispatcher dispatcher_;
    WsSessionManager wsSessions_;
    std::unique_ptr<NotificationSink> notificationBridge_;
    std::shared_ptr<AuthFilter> authFilter_;

    std::mutex oneshotMutex_;
    struct OneshotToken {
        std::string token;
        std::string ip;
        std::chrono::steady_clock::time_point expiry;
    };
    std::vector<OneshotToken> oneshotTokens_;

    // Subscription refresh thread
    std::thread refreshThread_;
    std::atomic<bool> refreshRunning_{false};

    // File root paths
    std::map<std::string, std::string> fileRoots_;
};

} // namespace tether::klipper::http
