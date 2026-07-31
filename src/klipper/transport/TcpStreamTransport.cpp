/**
 * @file TcpStreamTransport.cpp
 * @brief TCP/IP stream transport implementation (POSIX sockets).
 */

#include "tether/klipper/transport/TcpStreamTransport.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <cstring>

namespace tether::klipper::transport {

TcpStreamTransport::~TcpStreamTransport() {
    close();
}

bool TcpStreamTransport::open() {
    if (open_) return true;

    if (config_.mode == "server") {
        listenSock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listenSock_ < 0) return false;
        int yes = 1;
        ::setsockopt(listenSock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(config_.port);
        if (::bind(listenSock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(listenSock_); listenSock_ = -1; return false;
        }
        if (::listen(listenSock_, config_.backlog) < 0) {
            ::close(listenSock_); listenSock_ = -1; return false;
        }
        // Accept one connection (blocking with timeout).
        struct pollfd pfd{listenSock_, POLLIN, 0};
        if (::poll(&pfd, 1, config_.timeoutMs) <= 0) {
            ::close(listenSock_); listenSock_ = -1; return false;
        }
        sock_ = ::accept(listenSock_, nullptr, nullptr);
        if (sock_ < 0) { ::close(listenSock_); listenSock_ = -1; return false; }
        ::close(listenSock_); listenSock_ = -1;
    } else {
        // Client mode.
        sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) return false;
        struct addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        std::string portStr = std::to_string(config_.port);
        if (::getaddrinfo(config_.host.c_str(), portStr.c_str(), &hints, &res) != 0) {
            ::close(sock_); sock_ = -1; return false;
        }
        // Set non-blocking for timeout connect.
        int flags = ::fcntl(sock_, F_GETFL, 0);
        ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        int rc = ::connect(sock_, res->ai_addr, res->ai_addrlen);
        ::freeaddrinfo(res);
        if (rc < 0 && errno != EINPROGRESS) {
            ::close(sock_); sock_ = -1; return false;
        }
        struct pollfd pfd{sock_, POLLOUT, 0};
        if (::poll(&pfd, 1, config_.timeoutMs) <= 0) {
            ::close(sock_); sock_ = -1; return false;
        }
        int err = 0;
        socklen_t errLen = sizeof(err);
        ::getsockopt(sock_, SOL_SOCKET, SO_ERROR, &err, &errLen);
        if (err != 0) {
            ::close(sock_); sock_ = -1; return false;
        }
        // Restore blocking mode.
        ::fcntl(sock_, F_SETFL, flags);
    }
    // Disable Nagle for low-latency command/response.
    int one = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    open_ = true;
    return true;
}

bool TcpStreamTransport::isOpen() const { return open_; }

void TcpStreamTransport::close() {
    if (!open_.exchange(false)) return;
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
    if (listenSock_ >= 0) { ::close(listenSock_); listenSock_ = -1; }
}

size_t TcpStreamTransport::write(std::span<const uint8_t> data) {
    if (!open_ || sock_ < 0) return 0;
    size_t total = 0;
    while (total < data.size()) {
        ssize_t n = ::send(sock_, data.data() + total, data.size() - total, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return total;
        }
        total += static_cast<size_t>(n);
    }
    return total;
}

size_t TcpStreamTransport::available() const {
    if (!open_ || sock_ < 0) return 0;
    int n = 0;
    if (::ioctl(sock_, FIONREAD, &n) < 0) return 0;
    return static_cast<size_t>(n);
}

size_t TcpStreamTransport::read(uint8_t* out, size_t maxLen, bool canBlock) {
    if (!open_ || sock_ < 0 || maxLen == 0) return 0;
    if (canBlock) {
        struct pollfd pfd{sock_, POLLIN, 0};
        if (::poll(&pfd, 1, -1) <= 0) return 0;
    }
    ssize_t n = ::recv(sock_, out, maxLen, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return 0;
    }
    return static_cast<size_t>(n);
}

} // namespace tether::klipper::transport
