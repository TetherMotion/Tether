/**
 * @file PipeTransport.cpp
 * @brief POSIX pipe/serial transport implementation.
 */

#include "tether/klipper/transport/PipeTransport.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>

#ifdef __linux__
#include <termios.h>
#endif

namespace tether::klipper::transport {

PipeTransport::~PipeTransport() {
    close();
}

bool PipeTransport::open() {
    if (open_) return true;
    int rfd = config_.readFd;
    int wfd = config_.writeFd;

    if (!config_.devicePath.empty()) {
#ifdef __linux__
        int fd = ::open(config_.devicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return false;
        // Configure as serial if baud rate is set.
        if (config_.baudRate > 0) {
            struct termios tty{};
            if (tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }
            cfmakeraw(&tty);
            cfsetspeed(&tty, static_cast<speed_t>(config_.baudRate));
            tty.c_cflag |= CLOCAL | CREAD;
            if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }
        }
        rfd = fd;
        wfd = fd;
#else
        return false; // serial device mode only supported on Linux
#endif
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
