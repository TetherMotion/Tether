/**
 * @file IByteStreamTransport.hpp
 * @brief Abstract byte-stream transport for Klipper message blocks.
 *
 * @details
 * The Klipper protocol is transport-agnostic: message blocks are framed with
 * a sync byte and CRC, so any reliable byte stream can carry them. This
 * interface abstracts the underlying transport (loopback, pipe/serial, TCP,
 * CAN) so the protocol stack is independent of the physical link.
 *
 * Implementations:
 *   - LoopbackTransport  — in-process pair for tests/examples.
 *   - PipeTransport      — POSIX pipe/serial byte stream.
 *   - TcpStreamTransport — TCP/IP byte stream.
 *   - CanTransport       — byte stream tunneled over CAN frames (via HAL ICan).
 *
 * The interface is intentionally simple: write bytes, poll for readable bytes,
 * and read bytes. Blocking behaviour is governed by @p canBlock.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>
#include <optional>

namespace tether::klipper::transport {

/**
 * @brief Abstract bidirectional byte-stream transport.
 */
class IByteStreamTransport {
public:
    virtual ~IByteStreamTransport() = default;

    /**
     * @brief Open the transport (connect/bind as needed).
     * @return true on success.
     */
    virtual bool open() = 0;

    /// @return true if the transport is open and usable.
    virtual bool isOpen() const = 0;

    /// @brief Close the transport.
    virtual void close() = 0;

    /**
     * @brief Write bytes to the transport.
     * @param data Bytes to send.
     * @return Number of bytes actually written, or 0 on error/closed.
     */
    virtual size_t write(std::span<const uint8_t> data) = 0;

    /**
     * @brief Number of bytes currently available to read without blocking.
     * @return Byte count, or 0 if none available / not open.
     */
    virtual size_t available() const = 0;

    /**
     * @brief Read up to @p maxLen bytes into @p out.
     *
     * Non-blocking by default: returns whatever is currently available
     * (possibly 0). If @p canBlock is true and no data is available, blocks
     * until at least one byte arrives or the transport closes.
     *
     * @param out      Output buffer.
     * @param maxLen   Maximum bytes to read.
     * @param canBlock Whether to block waiting for data.
     * @return Number of bytes read, or 0 on EOF/error.
     */
    virtual size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) = 0;

    /**
     * @brief Convenience: read all currently-available bytes into a vector.
     */
    std::vector<uint8_t> readAll() {
        size_t n = available();
        if (n == 0) return {};
        std::vector<uint8_t> buf(n);
        size_t got = read(buf.data(), n, false);
        buf.resize(got);
        return buf;
    }
};

} // namespace tether::klipper::transport
