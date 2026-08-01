/**
 * @file PipeTransport.cpp
 * @brief POSIX pipe/serial transport implementation.
 *
 * For serial-device mode (devicePath non-empty), delegates the termios
 * setup to `tether::io::PosixSerialDriver` to avoid duplicating the
 * serial-port configuration code that already lives in the `tether::io`
 * module.  The driver's fd is borrowed via `PosixSerialDriver::fd()` and
 * the driver is released without closing (ownership transfers to
 * PipeTransport).  For raw-fd-pair mode (used by the Linux MCU
 * simulator), the transport manages the file descriptors directly.
 */

#include "tether/klipper/transport/PipeTransport.hpp"

#if !defined(ESP_PLATFORM)

#include "tether/io/SerialTransport.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>

namespace tether::klipper::transport {

PipeTransport::~PipeTransport() {
    close();
}

bool PipeTransport::open() {
    if (open_) return true;
    int rfd = config_.readFd;
    int wfd = config_.writeFd;

    if (!config_.devicePath.empty()) {
        // Serial device mode: use io::PosixSerialDriver for termios setup,
        // then dup its fd so we avoid duplicating serial-port code.
        // The driver is destroyed (closing its fd) after we've dup'd.
        auto driver = std::make_unique<tether::io::PosixSerialDriver>();
        if (!driver->open(config_.devicePath.c_str(), config_.baudRate)) {
            return false;
        }
        int fd = ::dup(driver->fd());
        driver.reset();  // closes the original fd; our dup remains valid
        if (fd < 0) return false;
        // Set non-blocking mode for the protocol layer's poll-based reads.
        int flags = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        rfd = fd;
        wfd = fd;
    }
    if (rfd < 0 || wfd < 0) return false;
    config_.readFd = rfd;
    config_.writeFd = wfd;
    open_ = true;
    return true;
}

bool PipeTransport::isOpen() const { return open_; }

void PipeTransport::close() {
    if (!open_.exchange(false)) return;
    if (config_.ownsFds) {
        if (config_.readFd >= 0) ::close(config_.readFd);
        if (config_.writeFd >= 0 && config_.writeFd != config_.readFd) {
            ::close(config_.writeFd);
        }
    }
    config_.readFd = -1;
    config_.writeFd = -1;
}

size_t PipeTransport::write(std::span<const uint8_t> data) {
    if (!open_ || config_.writeFd < 0) return 0;
    size_t total = 0;
    while (total < data.size()) {
        ssize_t n = ::write(config_.writeFd, data.data() + total, data.size() - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            return total; // partial write or error
        }
        total += static_cast<size_t>(n);
    }
    return total;
}

size_t PipeTransport::available() const {
    if (!open_ || config_.readFd < 0) return 0;
    int n = 0;
    if (::ioctl(config_.readFd, FIONREAD, &n) < 0) return 0;
    return static_cast<size_t>(n);
}

size_t PipeTransport::read(uint8_t* out, size_t maxLen, bool canBlock) {
    if (!open_ || config_.readFd < 0 || maxLen == 0) return 0;
    if (canBlock) {
        struct pollfd pfd{config_.readFd, POLLIN, 0};
        int pr = ::poll(&pfd, 1, -1);
        if (pr <= 0) return 0;
    }
    ssize_t n = ::read(config_.readFd, out, maxLen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return 0;
    }
    return static_cast<size_t>(n);
}

} // namespace tether::klipper::transport

#else // ESP_PLATFORM

namespace tether::klipper::transport {

PipeTransport::~PipeTransport() { close(); }
bool PipeTransport::open() { return false; }
bool PipeTransport::isOpen() const { return false; }
void PipeTransport::close() { open_ = false; }
size_t PipeTransport::write(std::span<const uint8_t>) { return 0; }
size_t PipeTransport::available() const { return 0; }
size_t PipeTransport::read(uint8_t*, size_t, bool) { return 0; }

} // namespace tether::klipper::transport

#endif // ESP_PLATFORM
