/**
 * @file PosixSerialByteStream.hpp
 * @brief Adapter wrapping tether::io::ISerialDriver into IByteStreamTransport.
 *
 * @details
 * This bridges the Tether IO layer's serial driver (termios-based
 * PosixSerialDriver) to the Klipper protocol's IByteStreamTransport
 * interface. It allows KlippyHost/KlipperDevice to communicate over
 * real /dev/ttyUSBx or /dev/ttyACMx serial ports without manual callback
 * wiring.
 *
 * Usage:
 * @code
 *   auto driver = std::make_unique<tether::io::PosixSerialDriver>();
 *   PosixSerialByteStream transport(std::move(driver),
 *                                   "/dev/ttyUSB0", 250000);
 *   KlippyHost host(transport);
 *   transport.open();
 *   host.connect();
 *   ...
 * @endcode
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/io/SerialTransport.hpp"

#include <memory>
#include <string>
#include <cstring>

namespace tether::klipper::transport {

/// @brief Adapter wrapping tether::io::ISerialDriver into IByteStreamTransport.
class PosixSerialByteStream : public IByteStreamTransport {
public:
    /// @brief Construct with a serial driver, port name, and baud rate.
    /// @param driver The serial driver (takes ownership).
    /// @param port Device path (e.g. "/dev/ttyUSB0").
    /// @param baudRate Baud rate (e.g. 250000).
    PosixSerialByteStream(std::unique_ptr<tether::io::ISerialDriver> driver,
                          std::string port, uint32_t baudRate)
        : driver_(std::move(driver))
        , port_(std::move(port))
        , baudRate_(baudRate) {}

    /// @brief Construct with default PosixSerialDriver.
    /// @param port Device path (e.g. "/dev/ttyUSB0").
    /// @param baudRate Baud rate (e.g. 250000).
    PosixSerialByteStream(std::string port, uint32_t baudRate)
        : driver_(std::make_unique<tether::io::PosixSerialDriver>())
        , port_(std::move(port))
        , baudRate_(baudRate) {}

    ~PosixSerialByteStream() override { close(); }

    bool open() override {
        if (!driver_) return false;
        if (driver_->isOpen()) return true;
        return driver_->open(port_.c_str(), baudRate_);
    }

    bool isOpen() const override {
        return driver_ && driver_->isOpen();
    }

    void close() override {
        if (driver_) driver_->close();
    }

    size_t write(std::span<const uint8_t> data) override {
        if (!isOpen() || data.empty()) return 0;
        return driver_->write(data.data(), data.size());
    }

    size_t available() const override {
        // ISerialDriver doesn't have a non-blocking available() method.
        // We do a zero-timeout read probe via a mutable buffer, but since
        // available() is const, we use a small internal read buffer instead.
        // For simplicity, return 1 if open (the protocol layer will call
        // read() which handles the actual data retrieval).
        return isOpen() ? 1 : 0;
    }

    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override {
        if (!isOpen() || maxLen == 0) return 0;
        // Use a short timeout for non-blocking, longer for blocking.
        uint32_t timeoutMs = canBlock ? 1000 : 1;
        return driver_->read(out, maxLen, timeoutMs);
    }

    /// @return The port name.
    const std::string& port() const { return port_; }

    /// @return The baud rate.
    uint32_t baudRate() const { return baudRate_; }

private:
    std::unique_ptr<tether::io::ISerialDriver> driver_;
    std::string port_;
    uint32_t baudRate_;
};

} // namespace tether::klipper::transport
