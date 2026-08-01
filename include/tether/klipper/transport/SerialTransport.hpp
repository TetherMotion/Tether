/**
 * @file SerialTransport.hpp
 * @brief USB serial and UART transport implementations.
 *
 * Provides:
 *   - UsbSerialTransport: USB CDC serial transport (callback-based)
 *   - UartTransport: hardware UART transport (callback-based)
 *
 * Both implement the IByteStreamTransport interface so they can be used
 * anywhere the Klipper protocol stack expects a byte stream.  The
 * callback-based design allows embedded targets (ESP-IDF, STM32) to wire
 * these transports to their platform-specific USB CDC / UART HAL drivers
 * without pulling in POSIX dependencies.
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/klipper/protocol/Constants.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

namespace tether::klipper::transport {

/// @brief USB CDC serial transport backed by user-supplied callbacks.
class UsbSerialTransport : public IByteStreamTransport {
public:
    using UsbWriteFunc = std::function<ssize_t(const uint8_t*, size_t)>;
    using UsbReadFunc  = std::function<ssize_t(uint8_t*, size_t, int)>;
    using UsbOpenFunc  = std::function<bool()>;
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

    bool isOpen() const override { return open_; }

    void close() override {
        if (!open_.exchange(false)) return;
        if (closeFunc_) closeFunc_();
    }

    size_t write(std::span<const uint8_t> data) override {
        if (!open_ || !writeFunc_ || data.empty()) return 0;
        ssize_t n = writeFunc_(data.data(), data.size());
        return n < 0 ? 0 : static_cast<size_t>(n);
    }

    size_t available() const override {
        // Callback-based transports don't expose a byte count; return 1 if
        // open so the protocol layer falls through to read().
        return open_ ? 1 : 0;
    }

    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override {
        if (!open_ || !readFunc_ || maxLen == 0) return 0;
        int timeoutMs = canBlock ? 1000 : 1;
        ssize_t n = readFunc_(out, maxLen, timeoutMs);
        return n < 0 ? 0 : static_cast<size_t>(n);
    }

    /// @return Transport name (for diagnostics).
    std::string name() const { return "usb:" + deviceName_; }

private:
    std::string deviceName_;
    UsbOpenFunc openFunc_;
    UsbCloseFunc closeFunc_;
    UsbWriteFunc writeFunc_;
    UsbReadFunc readFunc_;
    std::atomic<bool> open_{false};
};

/// @brief Hardware UART transport backed by user-supplied callbacks.
class UartTransport : public IByteStreamTransport {
public:
    using UartWriteFunc = std::function<ssize_t(const uint8_t*, size_t)>;
    using UartReadFunc  = std::function<ssize_t(uint8_t*, size_t)>;
    using UartOpenFunc  = std::function<bool()>;
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

    bool isOpen() const override { return open_; }

    void close() override {
        if (!open_.exchange(false)) return;
        if (closeFunc_) closeFunc_();
    }

    size_t write(std::span<const uint8_t> data) override {
        if (!open_ || !writeFunc_ || data.empty()) return 0;
        ssize_t n = writeFunc_(data.data(), data.size());
        return n < 0 ? 0 : static_cast<size_t>(n);
    }

    size_t available() const override {
        return open_ ? 1 : 0;
    }

    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override {
        if (!open_ || !readFunc_ || maxLen == 0) return 0;
        ssize_t n = readFunc_(out, maxLen);
        return n < 0 ? 0 : static_cast<size_t>(n);
    }

    /// @return Transport name (for diagnostics).
    std::string name() const {
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
