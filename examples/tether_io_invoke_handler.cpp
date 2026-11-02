/**
 * @file tether_io_invoke_handler.cpp
 * @brief Client-side InvokeEx catch-all: answering function ids that
 *        were never registered.
 *
 * registerFunctions() advertises a fixed catalog. The catch-all
 * setInvokeHandler() covers everything else — InvokeEx requests whose
 * functionId has no registered entry land there instead of returning
 * "Function not found". That is the hook for dynamic namespaces: here
 * the client treats id 0x1000|channel as a GPIO read without
 * advertising one entry per pin.
 *
 * The server side calls Session::callPeer(functionId, ...) directly —
 * it can invoke ids that never appeared in a RegisterFunctionsReq
 * catalog.
 *
 *   ./tether_io_invoke_handler
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Transport.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace tether::io;

namespace {

// ---- Minimal in-process message pipe (see tether_io_pipe_transport) ----
class PipeQueue {
public:
    void push(std::vector<uint8_t> msg) {
        { std::lock_guard lk(mtx_); queue_.push_back(std::move(msg)); }
        cv_.notify_one();
    }
    bool pop(std::vector<uint8_t>& out, uint32_t timeoutMs) {
        std::unique_lock lk(mtx_);
        cv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                     [&] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return false;
        out = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }
    void close() {
        { std::lock_guard lk(mtx_); closed_ = true; }
        cv_.notify_all();
    }
private:
    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<std::vector<uint8_t>> queue_;
    bool closed_ = false;
};

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
    size_t receive(uint8_t*, size_t, uint32_t) override { return 0; }
    bool receiveMessage(std::vector<uint8_t>& out,
                        uint32_t timeoutMs) override {
        return !closed_ && rx_->pop(out, timeoutMs);
    }
    void close() override { closed_ = true; rx_->close(); tx_->close(); }
    bool isConnected() const override { return !closed_; }
private:
    std::shared_ptr<PipeQueue> rx_, tx_;
    bool closed_ = false;
};

std::pair<std::unique_ptr<MessagePipe>, std::unique_ptr<MessagePipe>>
pipePair() {
    auto a2b = std::make_shared<PipeQueue>();
    auto b2a = std::make_shared<PipeQueue>();
    return {std::make_unique<MessagePipe>(b2a, a2b),
            std::make_unique<MessagePipe>(a2b, b2a)};
}

} // namespace

int main() {
    Registry registry;
    auto [serverEnd, clientEnd] = pipePair();

    Session session(std::move(serverEnd), registry, steadyClockUs,
                    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                    Framing::None);
    session.markRunning();
    std::thread sessionThread([&] { session.run(); });

    // The client registers no catalog at all — every invoke lands in
    // the catch-all. functionId 0x1000|channel is a GPIO read here.
    TetherIOClient client(std::move(clientEnd), 2000);
    const double gpio[] = {0.0, 3.3, 1.8, 5.0};
    client.setInvokeHandler(
        [&](uint64_t functionId,
            const std::vector<TetherIOClient::FunctionArg>&) {
            ClientCallResult r;
            const uint64_t base = functionId & ~0xFFFull;
            const uint64_t channel = functionId & 0xFFF;
            if (base != 0x1000 || channel >= 4) {
                r.success = false;
                r.errorMessage = "unknown functionId";
                return r;
            }
            r.success = true;
            r.hasReturnValue = true;
            r.returnValue.resize(8);
            std::memcpy(r.returnValue.data(), &gpio[channel], 8);
            return r;
        });
    std::atomic<bool> pumpOn{true};
    std::thread pump([&] {
        while (pumpOn) (void)client.receiveMessage(50);
    });

    auto call = [&](uint64_t id) {
        Session::InvokeResult res;
        std::mutex mtx;
        std::condition_variable cv;
        bool done = false;
        if (!session.callPeer(id, {}, 2'000'000,
                              [&](const Session::InvokeResult& r) {
                                  { std::lock_guard lk(mtx); res = r; done = true; }
                                  cv.notify_one();
                              }))
            return res;
        std::unique_lock lk(mtx);
        cv.wait_for(lk, std::chrono::seconds(3), [&] { return done; });
        return res;
    };

    bool ok = true;
    for (uint64_t ch = 0; ch < 4; ++ch) {
        const auto r = call(0x1000 | ch);
        double volts = -1;
        if (r.success && r.returnValue.size() == 8)
            std::memcpy(&volts, r.returnValue.data(), 8);
        std::printf("[server] gpio[%llu] = %.1f V\n",
                    static_cast<unsigned long long>(ch), volts);
        ok &= r.success && volts == gpio[ch];
    }

    // Outside the 0x1000|ch namespace the handler rejects the call.
    const auto bad = call(0x9000);
    std::printf("[server] id 0x9000 -> success=%d (%s)\n",
                (int)bad.success, bad.errorMessage.c_str());
    ok &= !bad.success;

    pumpOn = false;
    client.close();
    pump.join();
    session.requestStop();
    sessionThread.join();
    return ok ? 0 : 1;
}
