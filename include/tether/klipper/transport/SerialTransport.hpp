/**
 * @file SerialTransport.hpp
 * @brief USB serial and UART transport implementations.
 *
 * Provides:
 *   - UsbSerialTransport: USB CDC serial transport
 *   - UartTransport: hardware UART transport
 *   - Both implement the ITransport interface from the protocol layer
 */

#pragma once

#include "tether/klipper/protocol/Constants.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace tether::klipper::transport {

/// @brief Transport interface for Klipper communication.
class ITransport {
public:
    virtual ~ITransport() = default;

    /// @brief Open the transport.
    /// @return True on success.
    virtual bool open() = 0;

    /// @brief Close the transport.
    virtual void close() = 0;

    /// @brief Check if the transport is open.
    virtual bool isOpen() const = 0;

    /// @brief Write data to the transport.
    /// @return Number of bytes written, or -1 on error.
    virtual ssize_t write(const uint8_t* data, size_t len) = 0;

    /// @brief Read data from the transport.
    /// @param timeoutMs Read timeout in milliseconds (-1 = blocking).
    /// @return Number of bytes read, or -1 on error/timeout.
    virtual ssize_t read(uint8_t* data, size_t len, int timeoutMs = -1) = 0;

    /// @brief Get the transport name.
    virtual std::string name() const = 0;
};

/// @brief USB CDC serial transport.
class UsbSerialTransport : public ITransport {
public:
    using UsbWriteFunc = std::function<ssize_t(const uint8_t*, size_t)>;
    using UsbReadFunc = std::function<ssize_t(uint8_t*, size_t, int)>;
    using UsbOpenFunc = std::function<bool()>;
    using UsbCloseFunc = std::function<void()>;

    UsbSerialTransport(std::string deviceName,
                       UsbOpenFunc openFunc,
                       UsbCloseFunc closeFunc,
                       UsbWriteFunc writeFunc,
                       UsbReadFunc readFunc)
        : deviceName_(std::move(deviceName))
        , openFunc_(std::move(openFunc))
        , closeFunc_(std::move(closeFunc))
        , writeFunc_(std::move(writeFunc))
        , readFunc_(std::move(readFunc)) {}

    bool open() override {
        if (open_) return true;
        if (openFunc_) open_ = openFunc_();
        return open_;
    }

    void close() override {
        if (!open_) return;
        if (closeFunc_) closeFunc_();
        open_ = false;
    }

    bool isOpen() const override { return open_; }

    ssize_t write(const uint8_t* data, size_t len) override {
        if (!open_ || !writeFunc_) return -1;
        return writeFunc_(data, len);
    }

    ssize_t read(uint8_t* data, size_t len, int timeoutMs = -1) override {
        if (!open_ || !readFunc_) return -1;
        return readFunc_(data, len, timeoutMs);
    }

    std::string name() const override { return "usb:" + deviceName_; }

private:
    std::string deviceName_;
    UsbOpenFunc openFunc_;
    UsbCloseFunc closeFunc_;
    UsbWriteFunc writeFunc_;
    UsbReadFunc readFunc_;
    std::atomic<bool> open_{false};
};

/// @brief Hardware UART transport.
class UartTransport : public ITransport {
public:
    using UartWriteFunc = std::function<ssize_t(const uint8_t*, size_t)>;
    using UartReadFunc = std::function<ssize_t(uint8_t*, size_t)>;
    using UartOpenFunc = std::function<bool()>;
    using UartCloseFunc = std::function<void()>;

    UartTransport(std::string deviceName,
                  uint32_t baudRate,
                  UartOpenFunc openFunc,
                  UartCloseFunc closeFunc,
                  UartWriteFunc writeFunc,
                  UartReadFunc readFunc)
        : deviceName_(std::move(deviceName))
        , baudRate_(baudRate)
        , openFunc_(std::move(openFunc))
        , closeFunc_(std::move(closeFunc))
        , writeFunc_(std::move(writeFunc))
        , readFunc_(std::move(readFunc)) {}

    bool open() override {
        if (open_) return true;
        if (openFunc_) open_ = openFunc_();
        return open_;
    }

    void close() override {
        if (!open_) return;
        if (closeFunc_) closeFunc_();
        open_ = false;
    }

    bool isOpen() const override { return open_; }

    ssize_t write(const uint8_t* data, size_t len) override {
        if (!open_ || !writeFunc_) return -1;
        return writeFunc_(data, len);
    }

    ssize_t read(uint8_t* data, size_t len, int timeoutMs = -1) override {
        if (!open_ || !readFunc_) return -1;
        return readFunc_(data, len);
    }

    std::string name() const override {
        return "uart:" + deviceName_ + "@" + std::to_string(baudRate_);
    }

    uint32_t baudRate() const { return baudRate_; }

private:
    std::string deviceName_;
    uint32_t baudRate_;
    UartOpenFunc openFunc_;
    UartCloseFunc closeFunc_;
    UartWriteFunc writeFunc_;
    UartReadFunc readFunc_;
    std::atomic<bool> open_{false};
};

} // namespace tether::klipper::transport
