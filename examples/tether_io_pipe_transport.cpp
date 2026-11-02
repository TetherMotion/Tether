/**
 * @file tether_io_pipe_transport.cpp
 * @brief In-process transport: client and server in one process, no
 *        sockets — and a template for custom transports.
 *
 * The io layer only needs two interfaces: ITransportServer (start/
 * accept/stop) and ITransport (send/receive/receiveMessage/close).
 * A message-oriented link also opts out of SLIP via Framing::None.
 * This example implements a condition-variable pipe in ~100 lines and
 * runs a full V6 session over it — the same pattern the unit tests
 * use to skip TCP entirely.
 *
 *   ./tether_io_pipe_transport
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Transport.hpp"

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

using namespace tether::io;

namespace {

// One direction of a message pipe: a blocking byte-message queue.
class PipeQueue {
public:
    void push(std::vector<uint8_t> msg) {
        {
            std::lock_guard lk(mtx_);
            queue_.push_back(std::move(msg));
        }
        cv_.notify_one();
    }
    bool pop(std::vector<uint8_t>& out, uint32_t timeoutMs) {
        std::unique_lock lk(mtx_);
        if (!cv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                          [&] { return closed_ || !queue_.empty(); }))
            return false;
        if (queue_.empty()) return false;  // closed
        out = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }
    void close() {
        {
            std::lock_guard lk(mtx_);
            closed_ = true;
        }
        cv_.notify_all();
    }
    bool closed() const {
        std::lock_guard lk(mtx_);
        return closed_;
    }
private:
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<std::vector<uint8_t>> queue_;
    bool closed_ = false;
};

// One end of a connected pipe pair. Message-oriented: receiveMessage()
// delivers whole frames, so the session uses Framing::None.
class MessagePipe : public ITransport {
public:
    MessagePipe(std::shared_ptr<PipeQueue> rx,
                std::shared_ptr<PipeQueue> tx)
        : rx_(std::move(rx)), tx_(std::move(tx)) {}
    ~MessagePipe() override { close(); }

    bool send(const uint8_t* data, size_t len) override {
        if (closed_) return false;
        tx_->push({data, data + len});
        return true;
    }
    size_t receive(uint8_t* buf, size_t maxLen,
                   uint32_t timeoutMs) override {
        std::vector<uint8_t> msg;
        if (closed_ || !rx_->pop(msg, timeoutMs)) return 0;
        const size_t n = std::min(maxLen, msg.size());
        std::memcpy(buf, msg.data(), n);
        return n;
    }
    bool receiveMessage(std::vector<uint8_t>& out,
                        uint32_t timeoutMs) override {
        return !closed_ && rx_->pop(out, timeoutMs);
    }
    void close() override {
        closed_ = true;
        rx_->close();
        tx_->close();
    }
    bool isConnected() const override {
        return !closed_ && !rx_->closed();
    }
private:
    std::shared_ptr<PipeQueue> rx_, tx_;
    bool closed_ = false;
};

// Server that accepts queued in-process connections.
class MessagePipeServer : public ITransportServer {
public:
    bool start() override { listening_ = true; return true; }
    void stop() override {
        {
            std::lock_guard lk(mtx_);
            listening_ = false;
        }
        cv_.notify_all();
    }
    std::unique_ptr<ITransport> accept() override {
        std::unique_lock lk(mtx_);
        cv_.wait(lk, [&] { return !listening_ || !pending_.empty(); });
        if (pending_.empty()) return nullptr;  // stopped
        auto t = std::move(pending_.front());
        pending_.pop_front();
        return t;
    }
    bool isListening() const override { return listening_; }

    // Client side: fabricate a connected pair and queue the server end
    // for the next accept().
    std::unique_ptr<ITransport> connect() {
        auto c2s = std::make_shared<PipeQueue>();
        auto s2c = std::make_shared<PipeQueue>();
        {
            std::lock_guard lk(mtx_);
            pending_.push_back(
                std::make_unique<MessagePipe>(c2s, s2c));
        }
        cv_.notify_one();
        return std::make_unique<MessagePipe>(s2c, c2s);
    }
private:
    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<std::unique_ptr<ITransport>> pending_;
    std::atomic<bool> listening_{false};
};

} // namespace

int main() {
    Registry registry;
    {
        ParamEntry p;
        p.id = 1;
        p.name = "answer";
        p.group = "demo";
        p.valueType = ValueType::U32;
        p.readFn = [](void* d) { const uint32_t v = 42; std::memcpy(d, &v, 4); };
        if (!registry.addParam(p)) return 1;
    }

    auto pipeServer = std::make_unique<MessagePipeServer>();
    auto* pipe = pipeServer.get();

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::None;  // message boundaries preserved
    Server server(registry, std::move(pipeServer), sc);
    if (!server.start()) return 1;

    // No sockets: the "connect" call fabricates the linked pair.
    TetherIOClient client(pipe->connect(), 2000);

    const auto v = client.getParam(1);
    if (!v || v->size() != 4) { server.stop(); return 1; }
    uint32_t answer;
    std::memcpy(&answer, v->data(), 4);
    std::printf("[client] in-process getParam: answer=%u\n", answer);

    server.stop();
    return answer == 42 ? 0 : 1;
}
