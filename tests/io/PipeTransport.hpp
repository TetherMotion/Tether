/**
 * @file PipeTransport.hpp
 * @brief In-memory bidirectional transport pair for unit testing.
 *
 * PipeTransport::create() returns two ITransport endpoints connected by
 * shared thread-safe buffers.  Data sent by endpoint A is received by
 * endpoint B, and vice versa.  This enables true integration testing of
 * Session and Server without TCP/serial dependencies.
 */
#pragma once

#include "tether/io/Transport.hpp"
#include <mutex>
#include <condition_variable>
#include <vector>
#include <memory>
#include <cstring>
#include <chrono>
#include <atomic>

namespace tether { namespace io { namespace testing {

/// Shared, thread-safe byte buffer with blocking receive support.
class PipeBuffer {
public:
    void push(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        buf_.insert(buf_.end(), data, data + len);
        cv_.notify_one();
    }

    size_t pop(uint8_t* dest, size_t maxLen, uint32_t timeoutMs) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (buf_.empty() && timeoutMs > 0) {
            cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                         [this]() { return !buf_.empty() || closed_; });
        }
        if (buf_.empty()) return 0;
        size_t n = std::min(maxLen, buf_.size());
        std::memcpy(dest, buf_.data(), n);
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(n));
        return n;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        cv_.notify_all();
    }

    bool isClosed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<uint8_t> buf_;
    bool closed_ = false;
};

/// One endpoint of a pipe pair.  Sends go into the *peer's* read buffer.
class PipeTransport : public ITransport {
public:
    PipeTransport(std::shared_ptr<PipeBuffer> rxBuf,
                  std::shared_ptr<PipeBuffer> txBuf)
        : rxBuf_(std::move(rxBuf)), txBuf_(std::move(txBuf)) {}

    ~PipeTransport() override { close(); }

    bool send(const uint8_t* data, size_t len) override {
        if (closed_) return false;
        txBuf_->push(data, len);
        return true;
    }

    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override {
        if (closed_) return 0;
        return rxBuf_->pop(buf, maxLen, timeoutMs);
    }

    void close() override {
        closed_ = true;
        rxBuf_->close();
        txBuf_->close();
    }

    bool isConnected() const override {
        return !closed_ && !rxBuf_->isClosed();
    }

    /// Create a connected pair of PipeTransports {endpointA, endpointB}.
    /// Data sent by A is received by B, and vice versa.
    static std::pair<std::unique_ptr<PipeTransport>, std::unique_ptr<PipeTransport>> create() {
        auto bufAtoB = std::make_shared<PipeBuffer>();
        auto bufBtoA = std::make_shared<PipeBuffer>();
        auto a = std::make_unique<PipeTransport>(bufBtoA, bufAtoB);
        auto b = std::make_unique<PipeTransport>(bufAtoB, bufBtoA);
        return {std::move(a), std::move(b)};
    }

private:
    std::shared_ptr<PipeBuffer> rxBuf_;
    std::shared_ptr<PipeBuffer> txBuf_;
    std::atomic<bool> closed_{false};
};

/// An ITransportServer that yields pre-created PipeTransport endpoints.
class PipeTransportServer : public ITransportServer {
public:
    PipeTransportServer() = default;

    bool start() override { listening_ = true; return true; }
    void stop() override {
        listening_ = false;
        std::lock_guard<std::mutex> lock(mutex_);
        cv_.notify_all();
    }

    std::unique_ptr<ITransport> accept() override {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(500),
                     [this]() { return !pending_.empty() || !listening_; });
        if (pending_.empty()) return nullptr;
        auto t = std::move(pending_.front());
        pending_.erase(pending_.begin());
        return t;
    }

    bool isListening() const override { return listening_; }

    /// Queue a transport endpoint for accept() to return.
    /// Returns the "client" end of the pipe.
    std::unique_ptr<PipeTransport> addPendingConnection() {
        auto [client, server] = PipeTransport::create();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_.push_back(std::move(server));
        }
        cv_.notify_one();
        return std::move(client);
    }

private:
    std::atomic<bool> listening_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<std::unique_ptr<ITransport>> pending_;
};

}}} // namespace tether::io::testing
