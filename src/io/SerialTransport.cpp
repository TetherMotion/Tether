/**
 * @file SerialTransport.cpp
 * @brief Serial transport implementations.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/SerialTransport.hpp"

#if !defined(ESP_PLATFORM)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#endif

namespace tether { namespace io {

// ---------------------------------------------------------------------------
// SerialTransport
// ---------------------------------------------------------------------------

SerialTransport::SerialTransport(std::unique_ptr<ISerialDriver> driver)
    : driver_(std::move(driver)) {}

SerialTransport::~SerialTransport() {
    close();
}

bool SerialTransport::send(const uint8_t* data, size_t len) {
    if (!driver_ || !driver_->isOpen()) return false;
    size_t sent = 0;
    while (sent < len) {
        size_t n = driver_->write(data + sent, len - sent);
        if (n == 0) return false;
        sent += n;
    }
    return true;
}

size_t SerialTransport::receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    if (!driver_ || !driver_->isOpen()) return 0;
    return driver_->read(buf, maxLen, timeoutMs);
}

void SerialTransport::close() {
    if (driver_) driver_->close();
}

bool SerialTransport::isConnected() const {
    return driver_ && driver_->isOpen();
}

// ---------------------------------------------------------------------------
// PosixSerialDriver
// ---------------------------------------------------------------------------

#if !defined(ESP_PLATFORM)

PosixSerialDriver::~PosixSerialDriver() {
    close();
}

static speed_t baudToSpeed(uint32_t baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 3000000: return B3000000;
        default:      return B115200;
    }
}

bool PosixSerialDriver::open(const char* port, uint32_t baudRate) {
    fd_ = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    // Clear non-blocking after open
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t spd = baudToSpeed(baudRate);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= (CLOCAL | CREAD);

    // Raw mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void PosixSerialDriver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

size_t PosixSerialDriver::write(const uint8_t* data, size_t len) {
    if (fd_ < 0) return 0;
    ssize_t n = ::write(fd_, data, len);
    return (n > 0) ? static_cast<size_t>(n) : 0;
}

size_t PosixSerialDriver::read(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    if (fd_ < 0) return 0;

    if (timeoutMs > 0) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        struct timeval tv;
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        int sel = ::select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (sel <= 0) return 0;
    }

    ssize_t n = ::read(fd_, buf, maxLen);
    return (n > 0) ? static_cast<size_t>(n) : 0;
}

bool PosixSerialDriver::isOpen() const {
    return fd_ >= 0;
}

#endif // !ESP_PLATFORM

}} // namespace tether::io
