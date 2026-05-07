/**
 * @file Transport.hpp
 * @brief Abstract transport layer for the Tether IO protocol.
 *
 * The protocol can run over different transports (TCP, serial, etc.).
 * This header defines the abstract `ITransport` interface that transports
 * must implement, plus the `ITransportServer` interface for servers that
 * accept incoming connections.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <memory>
#include <functional>

namespace tether { namespace io {

/**
 * @class ITransport
 * @brief Abstract bidirectional byte-stream transport.
 *
 * Represents a single connection (one client-server pair).
 * Implementations must provide blocking send/receive operations.
 */
class ITransport {
public:
    virtual ~ITransport() = default;

    /// Send exactly `len` bytes. Returns true on success.
    virtual bool send(const uint8_t* data, size_t len) = 0;

    /// Receive up to `maxLen` bytes into `buf`.
    /// Returns the number of bytes received, or 0 on disconnect/error.
    /// May block for up to `timeoutMs` milliseconds (0 = non-blocking poll).
    virtual size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) = 0;

    /// Close the connection.
    virtual void close() = 0;

    /// Returns true if the transport is connected.
    virtual bool isConnected() const = 0;
};

/**
 * @class ITransportServer
 * @brief Abstract server that accepts incoming transport connections.
 *
 * Implementations create new ITransport instances for each accepted connection.
 */
class ITransportServer {
public:
    virtual ~ITransportServer() = default;

    /// Start listening. Returns true on success.
    virtual bool start() = 0;

    /// Stop listening and reject new connections.
    virtual void stop() = 0;

    /// Accept a new connection. Blocks until a connection arrives or stop() is called.
    /// Returns nullptr if the server was stopped or an error occurred.
    virtual std::unique_ptr<ITransport> accept() = 0;

    /// Returns true if the server is listening.
    virtual bool isListening() const = 0;
};

/// Factory function type for creating a serial transport.
/// Parameters: port path, baud rate.
using SerialTransportFactory = std::function<
    std::unique_ptr<ITransport>(const char* port, uint32_t baudRate)>;

}} // namespace tether::io
