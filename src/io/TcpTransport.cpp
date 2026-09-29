/**
 * @file TcpTransport.cpp
 * @brief TCP transport implementation using POSIX sockets.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/TcpTransport.hpp"

#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

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
    if (listenFd_ >= 0) {
        // shutdown() wakes a thread blocked in accept() — close() alone leaves
        // it parked on the still-open file description.
        ::shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
        listenFd_ = -1;
    }
}

std::unique_ptr<ITransport> TcpTransportServer::accept() {
    if (listenFd_ < 0) return nullptr;

    struct sockaddr_in clientAddr{};
    socklen_t addrLen = sizeof(clientAddr);
    int clientFd = ::accept(listenFd_,
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
