/**
 * @file PipeTransport.hpp
 * @brief POSIX pipe/serial byte-stream transport.
 *
 * @details
 * Wraps a pair of file descriptors (one for reading, one for writing) as a
 * bidirectional byte stream. This is used for UART/serial devices and Unix
 * pipes (e.g. the Linux MCU simulator communicates over a pipe). The
 * transport is non-blocking by default; @p read with canBlock=true uses
 * poll() to wait for data.
 *
 * On Linux, a serial device can be opened by path (e.g. "/dev/ttyUSB0") and
 * configured with a baud rate; internally it opens the device with O_RDWR and
 * sets the termios attributes. A pipe pair can be constructed from existing
 * file descriptors.
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <cstdint>
#include <string>
#include <atomic>

namespace tether::klipper::transport {

/**
 * @brief Configuration for a pipe/serial transport.
 */
struct PipeTransportConfig {
    /// Device path for a serial device (e.g. "/dev/ttyUSB0"). Empty for raw fd mode.
    std::string devicePath;
    /// Baud rate (e.g. 115200). Ignored if 0 or for raw fd mode.
    uint32_t baudRate = 115200;
    /// Read file descriptor (raw fd mode). If <0, derived from devicePath.
    int readFd = -1;
    /// Write file descriptor (raw fd mode). If <0, derived from devicePath.
    int writeFd = -1;
    /// If true, the transport owns the fds and closes them on close().
    bool ownsFds = true;
};

/**
 * @brief POSIX pipe/serial byte-stream transport.
 */
class PipeTransport : public IByteStreamTransport {
public:
    PipeTransport() = default;
    explicit PipeTransport(PipeTransportConfig config) : config_(std::move(config)) {}
    ~PipeTransport() override;

    bool open() override;
    bool isOpen() const override;
    void close() override;

    size_t write(std::span<const uint8_t> data) override;
    size_t available() const override;
    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override;

private:
    PipeTransportConfig config_;
    std::atomic<bool> open_{false};
};

} // namespace tether::klipper::transport
