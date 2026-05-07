/**
 * @file TcpTransport.hpp
 * @brief TCP transport implementation for the Tether IO protocol.
 *
 * Provides a TCP-based transport using POSIX sockets.  This implementation
 * works on Linux, macOS, and ESP-IDF (lwIP).
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Transport.hpp"
#include <cstdint>
#include <atomic>

namespace tether { namespace io {

/**
 * @class TcpTransport
 * @brief A single TCP connection transport.
 *
 * Constructed with an already-connected socket fd. Takes ownership of the fd.
 */
class TcpTransport : public ITransport {
public:
    /// Construct from an already-connected socket fd (takes ownership).
    explicit TcpTransport(int fd);
    ~TcpTransport() override;

    bool send(const uint8_t* data, size_t len) override;
    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override;
    void close() override;
    bool isConnected() const override;

private:
    int fd_;
    std::atomic<bool> connected_{true};
};

/**
 * @class TcpTransportServer
 * @brief TCP server that listens on a port and accepts connections.
 */
class TcpTransportServer : public ITransportServer {
public:
    /// @param port TCP port to listen on.
    /// @param backlog Listen backlog (default 4).
    explicit TcpTransportServer(uint16_t port, int backlog = 4);
    ~TcpTransportServer() override;

    bool start() override;
    void stop() override;
    std::unique_ptr<ITransport> accept() override;
    bool isListening() const override;

private:
    uint16_t port_;
    int backlog_;
    int listenFd_ = -1;
    std::atomic<bool> listening_{false};
};

}} // namespace tether::io
