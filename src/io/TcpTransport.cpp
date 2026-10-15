/**
 * @file TcpTransport.cpp
 * @brief TCP transport implementation using POSIX sockets.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/TcpTransport.hpp"

#include <sys/socket.h>
#include <sys/select.h>
#include <netdb.h>
#include <poll.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <string>

namespace tether { namespace io {

// ---------------------------------------------------------------------------
// TcpTransport
// ---------------------------------------------------------------------------

TcpTransport::TcpTransport(int fd) : fd_(fd) {}

TcpTransport::~TcpTransport() {
    close();
}

bool TcpTransport::send(const uint8_t* data, size_t len) {
    if (fd_ < 0) return false;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(fd_, data + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            connected_ = false;
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

size_t TcpTransport::receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    if (fd_ < 0) return 0;

    // timeoutMs == 0 is a non-blocking poll per the Transport contract;
    // select() with a zero timeval implements both cases.
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        struct timeval tv;
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        int n = ::select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (n <= 0) return 0;
    }

    ssize_t r = ::recv(fd_, buf, maxLen, 0);
    if (r <= 0) {
        connected_ = false;
        return 0;
    }
    return static_cast<size_t>(r);
}

void TcpTransport::close() {
    if (fd_ >= 0) {
        ::shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
    }
    connected_ = false;
}

bool TcpTransport::isConnected() const {
    return connected_.load(std::memory_order_relaxed) && fd_ >= 0;
}

// ---------------------------------------------------------------------------
// tcpConnect
// ---------------------------------------------------------------------------

std::unique_ptr<TcpTransport> tcpConnect(const char* host, uint16_t port,
                                         uint32_t timeoutMs) {
    if (!host || !*host) return nullptr;

    struct addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* addrs = nullptr;
    const std::string portStr = std::to_string(port);
    if (::getaddrinfo(host, portStr.c_str(), &hints, &addrs) != 0 || !addrs) {
        return nullptr;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    int fd = -1;
    for (struct addrinfo* ai = addrs; ai && fd < 0; ai = ai->ai_next) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        const int remainMs = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now).count());

        fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;

        // Non-blocking connect + poll() so timeoutMs bounds the attempt.
        const int flags = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        int rc = ::connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc < 0 && errno == EINPROGRESS) {
            struct pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLOUT;
            rc = ::poll(&pfd, 1, remainMs);
            if (rc > 0) {
                int err = 0;
                socklen_t errLen = sizeof(err);
                ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &errLen);
                rc = (err == 0) ? 0 : -1;
            }
        }
        if (rc != 0) {
            ::close(fd);
            fd = -1;
            continue;
        }
        ::fcntl(fd, F_SETFL, flags);   // restore blocking mode
        int opt = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    }
    ::freeaddrinfo(addrs);

    if (fd < 0) return nullptr;
    return std::make_unique<TcpTransport>(fd);
}

// ---------------------------------------------------------------------------
// TcpTransportServer
// ---------------------------------------------------------------------------

TcpTransportServer::TcpTransportServer(uint16_t port, std::string bindAddress,
                                       int backlog)
    : port_(port), bindAddress_(std::move(bindAddress)), backlog_(backlog) {}

TcpTransportServer::~TcpTransportServer() {
    stop();
}

bool TcpTransportServer::start() {
    if (listening_) return false;

    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) return false;

    int opt = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port_);
    if (bindAddress_.empty() || bindAddress_ == "0.0.0.0") {
        addr.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, bindAddress_.c_str(), &addr.sin_addr) != 1) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    if (::bind(listenFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    if (::listen(listenFd_, backlog_) < 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    listening_ = true;
    return true;
}

void TcpTransportServer::stop() {
    listening_ = false;
    const int fd = listenFd_.exchange(-1);
    if (fd >= 0) {
        // shutdown() wakes a thread blocked in accept() — close() alone leaves
        // it parked on the still-open file description.
        ::shutdown(fd, SHUT_RDWR);
        ::close(fd);
    }
}

std::unique_ptr<ITransport> TcpTransportServer::accept() {
    const int fd = listenFd_.load(std::memory_order_acquire);
    if (fd < 0) return nullptr;

    struct sockaddr_in clientAddr{};
    socklen_t addrLen = sizeof(clientAddr);
    int clientFd = ::accept(fd,
                            reinterpret_cast<struct sockaddr*>(&clientAddr),
                            &addrLen);
    if (clientFd < 0) return nullptr;

    // Disable Nagle for low latency
    int opt = 1;
    ::setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    return std::make_unique<TcpTransport>(clientFd);
}

bool TcpTransportServer::isListening() const {
    return listening_.load(std::memory_order_relaxed);
}

}} // namespace tether::io
