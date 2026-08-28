#include "TetherIOWebSocketController.hpp"

#include <condition_variable>
#include <algorithm>
#include <deque>
#include <vector>
#include <chrono>
#include <cstdio>

namespace tether::io::example {

namespace {

/// Human-readable name for a MessageType value (for verbose logging).
const char* msgTypeName(uint8_t type) {
    switch (type) {
        case 0x01: return "ListParamsReq";
        case 0x02: return "ListParamsResp";
        case 0x03: return "ConfigureStream";
        case 0x04: return "ConfigureAck";
        case 0x05: return "StartStream";
        case 0x06: return "StopStream";
        case 0x07: return "StreamData";
        case 0x08: return "Error";
        case 0x09: return "GetMetadataReq";
        case 0x0A: return "GetMetadataResp";
        case 0x0B: return "SetParameterReq";
        case 0x0C: return "SetParameterResp";
        case 0x0D: return "PingReq";
        case 0x0E: return "PongResp";
        case 0x20: return "ListSignalsReq";
        case 0x21: return "ListSignalsResp";
        case 0x22: return "GetParamReq";
        case 0x23: return "GetParamResp";
        case 0x24: return "GetSignalReq";
        case 0x25: return "GetSignalResp";
        case 0x26: return "SnapshotParamsReq";
        case 0x27: return "SnapshotParamsResp";
        case 0x28: return "SnapshotSignalsReq";
        case 0x29: return "SnapshotSignalsResp";
        case 0x35: return "ListFunctionsReq";
        case 0x36: return "ListFunctionsResp";
        case 0x37: return "CallFunctionReq";
        case 0x38: return "CallFunctionResp";
        default: return "Unknown";
    }
}

} // namespace

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
        if (messages_.empty() && connected_) {
            inputReady_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                 [this] { return !messages_.empty() || !connected_; });
        }
        if (messages_.empty()) return 0;
        auto& msg = messages_.front();
        const size_t count = std::min(maxLen, msg.size() - msgOffset_);
        std::copy_n(msg.data() + msgOffset_, count, data);
        msgOffset_ += count;
        if (msgOffset_ >= msg.size()) {
            messages_.pop_front();
            msgOffset_ = 0;
        }
        return count;
    }

    bool receiveMessage(std::vector<uint8_t>& out, uint32_t timeoutMs) override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (messages_.empty() && connected_) {
            inputReady_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                 [this] { return !messages_.empty() || !connected_; });
        }
        if (messages_.empty()) return false;
        out = std::move(messages_.front());
        messages_.pop_front();
        msgOffset_ = 0;
        return true;
    }

    void push(std::string_view bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) return;
        messages_.emplace_back(bytes.begin(), bytes.end());
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
    std::deque<std::vector<uint8_t>> messages_;
    size_t msgOffset_ = 0;
    bool connected_ = true;
};

struct TetherIOWebSocketController::Client {
    std::unique_ptr<WebSocketTransport> transport;
    WebSocketTransport* transportPtr = nullptr;
    std::unique_ptr<Session> session;
    std::thread worker;
    tether::io::LogFn logFn;

    Client(drogon::WebSocketConnectionPtr connection, Registry& registry, tether::io::LogFn logFn)
        : transport(std::make_unique<WebSocketTransport>(std::move(connection)))
        , logFn(logFn) {
        transportPtr = transport.get();
        session = std::make_unique<Session>(
            std::move(transport), registry,
            [] {
                return static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
            },
            logFn, nullptr, nullptr, nullptr, nullptr,
            nullptr, nullptr,
            Framing::None);
        worker = std::thread([this] { session->run(); });
    }

    ~Client() {
        if (session) session->requestStop();
        if (worker.joinable()) worker.join();
        // The Session destructor closes its transport.
    }
};

TetherIOWebSocketController::TetherIOWebSocketController(Registry& registry, tether::io::LogFn logFn)
    : registry_(registry), logFn_(logFn) {}

void TetherIOWebSocketController::handleNewConnection(
    const drogon::HttpRequestPtr&, const drogon::WebSocketConnectionPtr& connection) {
    auto client = std::make_shared<Client>(connection, registry_, logFn_);
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.emplace(connection.get(), std::move(client));
    if (logFn_) logFn_("TetherIO", "WebSocket client connected (total=%zu)",
                        clients_.size());
}

void TetherIOWebSocketController::handleNewMessage(
    const drogon::WebSocketConnectionPtr& connection, std::string&& message,
    const drogon::WebSocketMessageType& type) {
    if (type != drogon::WebSocketMessageType::Binary) {
        if (logFn_) logFn_("TetherIO", "Ignoring non-binary WebSocket message (type=%d)",
                            static_cast<int>(type));
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = clients_.find(connection.get());
    if (it != clients_.end()) {
        const auto* data = reinterpret_cast<const uint8_t*>(message.data());
        const auto len = message.size();
        if (len > 0 && logFn_) {
            logFn_("TetherIO", "→ RX %s (%zu bytes)", msgTypeName(data[0]), len);
        }
        it->second->transportPtr->push(message);
    }
}

void TetherIOWebSocketController::handleConnectionClosed(
    const drogon::WebSocketConnectionPtr& connection) {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = clients_.find(connection.get());
        if (it == clients_.end()) return;
        client = std::move(it->second);
        clients_.erase(it);
        if (logFn_) logFn_("TetherIO", "WebSocket client disconnected (remaining=%zu)",
                            clients_.size());
    }
    client->session->requestStop();
}

} // namespace tether::io::example
