/**
 * @file SerialTransport.hpp
 * @brief Abstract serial transport for the Tether IO protocol.
 *
 * Serial transport uses SLIP framing over a UART/serial link.
 * This header defines both the abstract serial driver interface
 * and a concrete POSIX implementation (for /dev/ttyXxx devices).
 *
 * On embedded targets (ESP-IDF), users can provide their own
 * `ISerialDriver` implementation backed by UART HAL or USB CDC.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Transport.hpp"
#include <cstdint>
#include <cstddef>
#include <memory>
#include <atomic>
#include <string>

namespace tether { namespace io {

/**
 * @class ISerialDriver
 * @brief Abstract byte-level serial port driver.
 *
 * Implementations should provide blocking I/O with optional timeout support.
 * This is the platform-specific layer that serial transports delegate to.
 */
class ISerialDriver {
public:
    virtual ~ISerialDriver() = default;

    /// Open the serial port. Returns true on success.
    virtual bool open(const char* port, uint32_t baudRate) = 0;

    /// Close the serial port.
    virtual void close() = 0;

    /// Write bytes. Returns number of bytes written.
    virtual size_t write(const uint8_t* data, size_t len) = 0;

    /// Read bytes with timeout. Returns number of bytes read.
    virtual size_t read(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) = 0;

    /// Returns true if the port is open.
    virtual bool isOpen() const = 0;
};

/**
 * @class SerialTransport
 * @brief Wraps an ISerialDriver into an ITransport.
 */
class SerialTransport : public ITransport {
public:
    /// Construct with an already-opened serial driver (takes ownership).
    explicit SerialTransport(std::unique_ptr<ISerialDriver> driver);
    ~SerialTransport() override;

    bool send(const uint8_t* data, size_t len) override;
    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override;
    void close() override;
    bool isConnected() const override;

private:
    std::unique_ptr<ISerialDriver> driver_;
};

#if !defined(ESP_PLATFORM)
/**
 * @class PosixSerialDriver
 * @brief POSIX serial driver implementation for Linux/macOS.
 *
 * Uses termios-based configuration for baud rate, 8N1 mode.
 */
class PosixSerialDriver : public ISerialDriver {
public:
    PosixSerialDriver() = default;
    ~PosixSerialDriver() override;

    bool open(const char* port, uint32_t baudRate) override;
    void close() override;
    size_t write(const uint8_t* data, size_t len) override;
    size_t read(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override;
    bool isOpen() const override;

private:
    int fd_ = -1;
};
#endif // !ESP_PLATFORM

}} // namespace tether::io
