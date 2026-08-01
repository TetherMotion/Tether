/**
 * @file TcpStreamTransport.hpp
 * @brief TCP/IP byte-stream transport.
 *
 * @details
 * Wraps a TCP socket as a bidirectional byte stream. Supports both client
 * (connect to a host:port) and server (listen on a port, accept one
 * connection) modes. This enables Klipper protocol over TCP/IP, useful for
 * networked MCU simulators and remote devices.
 *
 * The connection setup (connect/listen/accept) is delegated to shared
 * helpers in `tether::platform::PosixSocket.hpp`, and the actual data
 * transfer is delegated to `tether::io::TcpTransport`.  This avoids
 * duplicating POSIX socket I/O code with the `tether::io` module.
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <cstdint>
#include <string>
#include <atomic>
#include <memory>

#if !defined(ESP_PLATFORM)
#include "tether/io/TcpTransport.hpp"
#include "tether/platform/PosixSocket.hpp"
#endif

namespace tether::klipper::transport {

/**
 * @brief Configuration for a TCP stream transport.
 */
struct TcpTransportConfig {
    /// "client" to connect to host:port; "server" to listen on port.
    std::string mode{"client"};
    /// Remote host to connect to (client mode).
    std::string host{"127.0.0.1"};
    /// Port (both client and server mode).
    uint16_t port = 0;
    /// Listen backlog (server mode).
    int backlog = 1;
    /// Connect/listen timeout in milliseconds.
    int timeoutMs = 5000;
};

/**
 * @brief TCP/IP byte-stream transport.
 *
 * Adapts `tether::io::TcpTransport` to the `IByteStreamTransport` interface.
 * On non-POSIX platforms (ESP-IDF), the open() method always fails —
 * embedded targets should use callback-based transports (UsbSerialTransport,
 * UartTransport) or CanTransport instead.
 */
class TcpStreamTransport : public IByteStreamTransport {
public:
    TcpStreamTransport() = default;
    explicit TcpStreamTransport(TcpTransportConfig config) : config_(std::move(config)) {}
    ~TcpStreamTransport() override;

    bool open() override;
    bool isOpen() const override;
    void close() override;

    size_t write(std::span<const uint8_t> data) override;
    size_t available() const override;
    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override;

private:
    TcpTransportConfig config_;
#if !defined(ESP_PLATFORM)
    std::unique_ptr<tether::io::TcpTransport> transport_;
#endif
    std::atomic<bool> open_{false};
};

} // namespace tether::klipper::transport
