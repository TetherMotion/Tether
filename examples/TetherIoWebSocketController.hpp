#pragma once

#include <drogon/WebSocketController.h>
#include <drogon/WebSocketConnection.h>
#include <tether/io/Registry.hpp>
#include <tether/io/Session.hpp>

#include <memory>
#include <mutex>
#include <unordered_map>

namespace tether::io::example {

class TetherIoWebSocketController
    : public drogon::WebSocketController<TetherIoWebSocketController, false> {
public:
    static void initPathRouting() {}

    explicit TetherIoWebSocketController(Registry& registry);

    void handleNewMessage(const drogon::WebSocketConnectionPtr& connection,
                          std::string&& message,
                          const drogon::WebSocketMessageType& type) override;
    void handleNewConnection(const drogon::HttpRequestPtr& request,
                             const drogon::WebSocketConnectionPtr& connection) override;
    void handleConnectionClosed(const drogon::WebSocketConnectionPtr& connection) override;

private:
    struct Client;
    Registry& registry_;
    std::mutex mutex_;
    std::unordered_map<const drogon::WebSocketConnection*, std::shared_ptr<Client>> clients_;
};

} // namespace tether::io::example
