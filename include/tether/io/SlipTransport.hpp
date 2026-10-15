/**
 * @file SlipTransport.hpp
 * @brief SLIP-framed client transport adapter for the Tether IO protocol.
 *
 * Wraps a byte-stream `ITransport` (e.g. `TcpTransport`, `SerialTransport`)
 * and applies SLIP framing so the wrapped transport satisfies the
 * message-oriented contract (`receiveMessage()`) that `TetherIOClient`
 * requires.  Mirrors `Session`'s deframing on the client side: one SLIP
 * packet per protocol message, bounded buffers, discard-until-END
 * recovery after malformed or oversized frames.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Transport.hpp"
#include "tether/io/Protocol.hpp"
#include "tether/io/ReceiveBuffer.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace tether { namespace io {

/**
 * @class SlipFramedTransport
 * @brief ITransport adapter adding SLIP framing to a byte-stream transport.
 *
 * `send()` SLIP-encodes the message and appends an END delimiter (one send
 * = one packet).  `receiveMessage()` accumulates bytes until a packet
 * completes and returns the decoded protocol message.
 *
 * `receive()` passes through to the inner transport — mixing receive() and
 * receiveMessage() callers on the same instance is not supported (bytes
 * consumed by receive() are lost to the message layer).
 */
class SlipFramedTransport final : public ITransport {
public:
    /// @param inner byte-stream transport (takes ownership)
    /// @param maxEncodedSize bound on the encoded accumulation buffer
    /// @param maxDecodedSize bound on one decoded message
    explicit SlipFramedTransport(
        std::unique_ptr<ITransport> inner,
        size_t maxEncodedSize = MAX_ENCODED_MESSAGE_SIZE,
        size_t maxDecodedSize = MAX_MESSAGE_SIZE);

    bool send(const uint8_t* data, size_t len) override;
    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override;
    bool receiveMessage(std::vector<uint8_t>& out, uint32_t timeoutMs) override;
    void close() override;
    bool isConnected() const override;

    /// Access the wrapped byte-stream transport.
    ITransport* inner() const { return inner_.get(); }

    /// Statistics — useful for `stream --stats` and diagnostics.
    size_t droppedFrames() const { return droppedFrames_; }
    size_t decodedFrames() const { return decodedFrames_; }

private:
    /// Feed one received byte; returns true when a complete message was
    /// produced into `out`.
    bool feedByte(uint8_t byte, std::vector<uint8_t>& out);

    std::unique_ptr<ITransport> inner_;
    std::vector<uint8_t> encoded_;   ///< accumulated encoded packet bytes
    std::vector<uint8_t> decoded_;   ///< decode scratch
    std::vector<uint8_t> pending_;   ///< undispatched bytes from last receive()
    size_t pendingOff_ = 0;
    size_t maxEncoded_;
    size_t maxDecoded_;
    bool discardUntilEnd_ = false;   ///< resync: drop through next END
    size_t droppedFrames_ = 0;
    size_t decodedFrames_ = 0;
};

}} // namespace tether::io
