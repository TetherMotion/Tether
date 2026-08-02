/**
 * @file TransportExpected.hpp
 * @brief std::expected-based wrappers for transport I/O operations.
 *
 * @details
 * Provides `tryOpen()`, `tryWrite()`, `tryRead()` that return
 * `std::expected<T, KlipperError>` instead of bool/size_t. These wrap
 * the existing IByteStreamTransport methods, preserving backward
 * compatibility while enabling structured error handling.
 */

#pragma once

#include "tether/klipper/KlipperError.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <expected>
#include <span>
#include <vector>

namespace tether::klipper::transport {

/// @brief Attempt to open a transport. Returns error on failure.
inline std::expected<void, KlipperError> tryOpen(IByteStreamTransport& t) {
    if (t.open()) return {};
    return std::unexpected(KlipperError::transport("Transport::open() failed"));
}

/// @brief Attempt to write bytes. Returns the number of bytes written, or error.
inline std::expected<size_t, KlipperError>
tryWrite(IByteStreamTransport& t, std::span<const uint8_t> data) {
    if (!t.isOpen()) {
        return std::unexpected(KlipperError::transport("Transport not open"));
    }
    size_t n = t.write(data);
    if (n == 0 && !data.empty()) {
        return std::unexpected(KlipperError::transport(
            "Transport::write() wrote 0 bytes of " + std::to_string(data.size())));
    }
    return n;
}

/// @brief Attempt to read bytes. Returns the bytes read, or error.
inline std::expected<std::vector<uint8_t>, KlipperError>
tryRead(IByteStreamTransport& t, size_t maxBytes) {
    if (!t.isOpen()) {
        return std::unexpected(KlipperError::transport("Transport not open"));
    }
    size_t avail = t.available();
    if (avail == 0) return std::vector<uint8_t>{};
    size_t toRead = std::min(avail, maxBytes);
    std::vector<uint8_t> buf(toRead);
    size_t n = t.read(buf.data(), toRead, false);
    buf.resize(n);
    return buf;
}

/// @brief Attempt to read all available bytes. Returns the bytes, or error.
inline std::expected<std::vector<uint8_t>, KlipperError>
tryReadAll(IByteStreamTransport& t) {
    if (!t.isOpen()) {
        return std::unexpected(KlipperError::transport("Transport not open"));
    }
    return t.readAll();
}

} // namespace tether::klipper::transport
