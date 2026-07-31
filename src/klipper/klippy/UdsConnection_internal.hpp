#pragma once

/// @file UdsConnection_internal.hpp
/// @brief Internal UDS connection class (not part of public API).
///
/// This header is included by the split KlippyUdsServer*.cpp files that
/// need access to the UdsConnection class. It is an implementation detail
/// and should not be included by external code.

#include <cerrno>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Internal class representing a single UDS client connection.
class UdsConnection {
public:
    explicit UdsConnection(int fd, int id) : fd_(fd), id_(id) {}
    ~UdsConnection() { close(); }

    int fd() const { return fd_; }
    int id() const { return id_; }
    bool closed() const { return closed_; }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        closed_ = true;
    }

    /// @brief Read available data and return complete frames.
    std::vector<std::string> readFrames() {
        std::vector<std::string> frames;
        char buf[4096];
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0) {
            // Non-blocking: EAGAIN/EWOULDBLOCK means no data available yet
            if (errno == EAGAIN || errno == EWOULDBLOCK) return frames;
            // Any other error: connection is closed
            closed_ = true;
            return frames;
        }
        if (n == 0) {
            // EOF: peer closed the connection
            closed_ = true;
            return frames;
        }
        partial_.append(buf, n);
        // Split on ETX (0x03)
        size_t pos = 0;
        while (true) {
            size_t etx = partial_.find('\x03', pos);
            if (etx == std::string::npos) break;
            frames.push_back(partial_.substr(pos, etx - pos));
            pos = etx + 1;
        }
        partial_ = partial_.substr(pos);
        return frames;
    }

    /// @brief Write data to the socket.
    bool write(const std::string& data) {
        if (closed_ || fd_ < 0) return false;
        size_t total = 0;
        while (total < data.size()) {
            ssize_t n = ::write(fd_, data.data() + total, data.size() - total);
            if (n <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                closed_ = true;
                return false;
            }
            total += n;
        }
        return true;
    }

    /// @brief Send a JSON frame terminated by ETX.
    bool sendFrame(const std::string& json) {
        return write(json + "\x03");
    }

    /// @brief Set non-blocking mode.
    void setNonBlocking() {
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    }

private:
    int fd_;
    int id_;
    bool closed_ = false;
    std::string partial_; ///< Accumulator for partial frames.
};

} // namespace tether::klipper::klippy
