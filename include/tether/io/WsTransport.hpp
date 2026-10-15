/**
 * @file WsTransport.hpp
 * @brief Minimal RFC 6455 WebSocket client transport for the Tether IO protocol.
 *
 * Implements just enough of the WebSocket client protocol to carry Tether
 * IO messages (one binary frame = one protocol message, `Framing::None`):
 * HTTP/1.1 Upgrade handshake with Sec-WebSocket-Accept verification,
 * client-side payload masking, fragmentation reassembly, ping/pong/close
 * control frames, and a bounded message size.
 *
 * The transport is byte-oriented underneath — it runs over any connected
 * `ITransport` (`tcpConnect` for real sockets, `PipeTransport` in tests),
 * so the WS codec is independently testable without a network.
 *
 * TLS (`wss://`) is not supported — no TLS library is vendored.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Transport.hpp"
#include "tether/io/Protocol.hpp"

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace tether { namespace io {

/**
 * @class WsClientTransport
 * @brief ITransport over a client-side WebSocket connection.
 *
 * Construct with a connected byte-stream transport, then call
 * `handshake()` to perform the HTTP/1.1 Upgrade exchange.  Until the
 * handshake succeeds `isConnected()` is false.
 */
class WsClientTransport final : public ITransport {
public:
    struct Options {
        std::string host;                            ///< Host header value
        std::string target = "/tether-io";           ///< Path incl. ?query
        /// Extra request headers (e.g. {"Authorization", "Bearer <t>"}).
        /// Upgrade/Connection/Sec-WebSocket-* headers are managed
        /// internally and rejected here.
        std::vector<std::pair<std::string, std::string>> headers;
        uint32_t handshakeTimeoutMs = 5000;
        size_t maxMessageSize = MAX_MESSAGE_SIZE;
    };

    /// Wrap a connected byte-stream transport (takes ownership).
    explicit WsClientTransport(std::unique_ptr<ITransport> byteTransport);
    ~WsClientTransport() override;

    /// Convenience: TCP-connect then handshake. Returns the failure reason
    /// on error.
    static std::expected<std::unique_ptr<WsClientTransport>, std::string>
    connect(const char* host, uint16_t port, const Options& opts);
    static std::expected<std::unique_ptr<WsClientTransport>, std::string>
    connect(const char* host, uint16_t port) {
        return connect(host, port, Options{});
    }

    /// Perform the HTTP/1.1 Upgrade handshake (sends GET, verifies the
    /// 101 response and Sec-WebSocket-Accept).
    std::expected<void, std::string> handshake(const Options& opts);

    bool send(const uint8_t* data, size_t len) override;
    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override;
    bool receiveMessage(std::vector<uint8_t>& out, uint32_t timeoutMs) override;
    void close() override;
    bool isConnected() const override;

    /// The wrapped byte-stream transport.
    ITransport* inner() const { return inner_.get(); }

private:
    /// Send one frame: opcode 0x2 (binary) by default, always masked.
    bool sendFrame(uint8_t opcode, const uint8_t* payload, size_t len);
    /// Read exactly `n` bytes within the deadline slice. Returns false on
    /// timeout/disconnect.
    bool readExact(uint8_t* dst, size_t n,
                   std::chrono::steady_clock::time_point deadline);
    /// Protocol-error shutdown: close frame 1002 + disconnect.
    void protocolError();

    std::unique_ptr<ITransport> inner_;
    bool connected_ = false;
    bool sentClose_ = false;
    size_t maxMessageSize_ = MAX_MESSAGE_SIZE;
    std::vector<uint8_t> message_;     ///< fragmented message accumulator
    bool fragmented_ = false;
    std::vector<uint8_t> msgBytes_;    ///< leftover for receive() byte view
    size_t msgOffset_ = 0;
};

}} // namespace tether::io
