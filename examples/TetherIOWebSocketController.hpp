#pragma once

#include <drogon/WebSocketController.h>
#include <drogon/WebSocketConnection.h>
#include <tether/io/MachineAuth.hpp>
#include <tether/io/MachineService.hpp>
#include <tether/io/Registry.hpp>
#include <tether/io/SchemaCatalog.hpp>
#include <tether/io/Session.hpp>

#include <memory>
#include <mutex>
#include <unordered_map>

namespace tether::io::example {

class TetherIOWebSocketController
    : public drogon::WebSocketController<TetherIOWebSocketController, false> {
public:
    static void initPathRouting() {}

    /// `machineService` may be null (read-only mode). `roleQueryParam` names
    /// a development-only query parameter (e.g. `?role=operator`) that maps
    /// to a session role; production deployments must assign identity from
    /// the authenticated transport instead and leave `roleQueryParam` empty.
    ///
    /// `authProvider`, when set, authenticates the HTTP `Authorization:
    /// Bearer` header on the upgrade request. Unauthenticated sessions stay
    /// observer/read-only — every mutation path fails closed on the server.
    explicit TetherIOWebSocketController(Registry& registry,
                                         tether::io::LogFn logFn = nullptr,
                                         SchemaCatalog* schemaCatalog = nullptr,
                                         machine::MachineService* machineService = nullptr,
                                         std::string roleQueryParam = {},
                                         const machine::IMachineAuthProvider* authProvider = nullptr);

    void handleNewMessage(const drogon::WebSocketConnectionPtr& connection,
                          std::string&& message,
                          const drogon::WebSocketMessageType& type) override;
    void handleNewConnection(const drogon::HttpRequestPtr& request,
                             const drogon::WebSocketConnectionPtr& connection) override;
    void handleConnectionClosed(const drogon::WebSocketConnectionPtr& connection) override;

private:
    struct Client;
    Registry& registry_;
    tether::io::LogFn logFn_;
    SchemaCatalog* schemaCatalog_;
    machine::MachineService* machineService_;
    std::string roleQueryParam_;
    const machine::IMachineAuthProvider* authProvider_ = nullptr;
    std::mutex mutex_;
    std::unordered_map<const drogon::WebSocketConnection*, std::shared_ptr<Client>> clients_;
};

} // namespace tether::io::example
