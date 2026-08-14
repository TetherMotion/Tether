#pragma once

/// @file KlippyWsController.hpp
/// @brief WebSocket controller for the Klippy HTTP server.
///
/// Drogon's WebSocket controller system requires a class derived from
/// WebSocketController<T, false> with static path routing. This controller
/// delegates all WebSocket events to the parent KlippyHttpServer.

#include "tether/klipper/http/KlippyHttpServer.hpp"

#include <drogon/WebSocketController.h>
#include <drogon/WebSocketConnection.h>

#include <memory>
#include <string>

namespace tether::klipper::http {

class KlippyHttpServer;

/// @brief WebSocket controller for /websocket endpoint.
///
/// Created with AutoCreation=false so we can inject a pointer to the
/// KlippyHttpServer instance. Registered via drogon::app().registerController().
class KlippyWsController
    : public drogon::WebSocketController<KlippyWsController, false> {
public:
    // Define path routing (required by WebSocketController base, even though
    // we register the path manually in KlippyHttpServer::start()).
    static void initPathRouting() {
        // Path is registered manually via app().registerWebSocketController()
        // in KlippyHttpServer::start(). This empty implementation satisfies
        // the base class requirement.
    }

    KlippyWsController() = default;
    explicit KlippyWsController(KlippyHttpServer* server) : server_(server) {}

    void setServer(KlippyHttpServer* server) { server_ = server; }

    void handleNewMessage(const drogon::WebSocketConnectionPtr& conn,
                          std::string&& message,
                          const drogon::WebSocketMessageType& type) override {
        if (type == drogon::WebSocketMessageType::Text && server_) {
            server_->handleWsMessage(conn, std::move(message));
        }
    }

    void handleNewConnection(const drogon::HttpRequestPtr& req,
                             const drogon::WebSocketConnectionPtr& conn) override {
        if (server_) {
            server_->handleWsNewConnection(conn);
        }
    }

    void handleConnectionClosed(const drogon::WebSocketConnectionPtr& conn) override {
        if (server_) {
            server_->handleWsDisconnect(conn);
        }
    }

private:
    KlippyHttpServer* server_ = nullptr;
};

} // namespace tether::klipper::http
