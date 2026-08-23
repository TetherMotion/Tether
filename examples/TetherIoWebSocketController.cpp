#include "TetherIoWebSocketController.hpp"

#include <condition_variable>
#include <algorithm>
#include <deque>
#include <vector>
#include <chrono>

namespace tether::io::example {

namespace {

class WebSocketTransport final : public ITransport {
public:
    explicit WebSocketTransport(drogon::WebSocketConnectionPtr connection)
        : connection_(std::move(connection)) {}

    bool send(const uint8_t* data, size_t len) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) return false;
        connection_->send(reinterpret_cast<const char*>(data), len,
                          drogon::WebSocketMessageType::Binary);
        return true;
    }

    size_t receive(uint8_t* data, size_t maxLen, uint32_t timeoutMs) override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (input_.empty() && connected_) {
            inputReady_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                 [this] { return !input_.empty() || !connected_; });
        }
        if (input_.empty()) return 0;
        const size_t count = std::min(maxLen, input_.size());
        std::copy_n(input_.begin(), count, data);
        input_.erase(input_.begin(), input_.begin() + count);
        return count;
    }

    void push(std::string_view bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) return;
        input_.insert(input_.end(), bytes.begin(), bytes.end());
        inputReady_.notify_one();
    }

    void close() override {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
        inputReady_.notify_all();
    }

    bool isConnected() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

private:
    drogon::WebSocketConnectionPtr connection_;
    mutable std::mutex mutex_;
    std::condition_variable inputReady_;
    std::deque<uint8_t> input_;
    bool connected_ = true;
};

} // namespace

struct TetherIoWebSocketController::Client {
    std::unique_ptr<WebSocketTransport> transport;
    WebSocketTransport* transportPtr = nullptr;
    std::unique_ptr<Session> session;
    std::thread worker;

    Client(drogon::WebSocketConnectionPtr connection, Registry& registry)
        : transport(std::make_unique<WebSocketTransport>(std::move(connection))) {
        transportPtr = transport.get();
        session = std::make_unique<Session>(
            std::move(transport), registry,
            [] {
                return static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
            });
        worker = std::thread([this] { session->run(); });
    }

    ~Client() {
        if (session) session->requestStop();
        if (worker.joinable()) worker.join();
        // The Session destructor closes its transport.
    }
};

TetherIoWebSocketController::TetherIoWebSocketController(Registry& registry)
    : registry_(registry) {}

void TetherIoWebSocketController::handleNewConnection(
    const drogon::HttpRequestPtr&, const drogon::WebSocketConnectionPtr& connection) {
    auto client = std::make_shared<Client>(connection, registry_);
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.emplace(connection.get(), std::move(client));
}

void TetherIoWebSocketController::handleNewMessage(
    const drogon::WebSocketConnectionPtr& connection, std::string&& message,
    const drogon::WebSocketMessageType& type) {
    if (type != drogon::WebSocketMessageType::Binary) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = clients_.find(connection.get());
    if (it != clients_.end()) {
        it->second->transportPtr->push(message);
    }
}

void TetherIoWebSocketController::handleConnectionClosed(
    const drogon::WebSocketConnectionPtr& connection) {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = clients_.find(connection.get());
        if (it == clients_.end()) return;
        client = std::move(it->second);
        clients_.erase(it);
    }
    client->session->requestStop();
}

} // namespace tether::io::example
