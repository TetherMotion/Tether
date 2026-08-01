/**
 * @file PosixSocket.hpp
 * @brief Shared POSIX TCP socket helpers for transport implementations.
 *
 * @details
 * Provides `posixTcpConnect()` and `posixTcpListenAccept()` helpers that
 * encapsulate the boilerplate of creating a TCP connection (client or
 * server) with a timeout.  Both return a connected file descriptor or -1
 * on failure.  The actual send/receive over the fd should be delegated to
 * `tether::io::TcpTransport` to avoid duplicating socket I/O code.
 *
 * These helpers are only available on POSIX platforms (Linux, macOS).
 * On ESP-IDF they are not compiled.
 */

#pragma once

#include <cstdint>
#include <string>

#if !defined(ESP_PLATFORM)

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

namespace tether::platform {

/// @brief Connect to a TCP host:port with a timeout (client mode).
/// @param host Hostname or IP address.
/// @param port TCP port.
/// @param timeoutMs Connect timeout in milliseconds.
/// @return Connected file descriptor, or -1 on failure.
inline int posixTcpConnect(const std::string& host, uint16_t port, int timeoutMs) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    std::string portStr = std::to_string(port);
    if (::getaddrinfo(host.c_str(), portStr.c_str(), &hints, &res) != 0) {
        ::close(sock);
        return -1;
    }

    // Set non-blocking for timeout connect.
    int flags = ::fcntl(sock, F_GETFL, 0);
    ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int rc = ::connect(sock, res->ai_addr, res->ai_addrlen);
    ::freeaddrinfo(res);
    if (rc < 0 && errno != EINPROGRESS) {
        ::close(sock);
        return -1;
    }

    struct pollfd pfd{sock, POLLOUT, 0};
    if (::poll(&pfd, 1, timeoutMs) <= 0) {
        ::close(sock);
        return -1;
    }
    int err = 0;
    socklen_t errLen = sizeof(err);
    ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &errLen);
    if (err != 0) {
        ::close(sock);
        return -1;
    }

    // Restore blocking mode.
    ::fcntl(sock, F_SETFL, flags);

    // Disable Nagle for low-latency command/response.
    int one = 1;
    ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    return sock;
}

/// @brief Listen on a port and accept one connection (server mode).
/// @param port TCP port to listen on.
/// @param backlog Listen backlog.
/// @param timeoutMs Accept timeout in milliseconds.
/// @return Connected file descriptor, or -1 on failure.
inline int posixTcpListenAccept(uint16_t port, int backlog, int timeoutMs) {
    int listenSock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenSock < 0) return -1;

    int yes = 1;
    ::setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (::bind(listenSock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listenSock);
        return -1;
    }
    if (::listen(listenSock, backlog) < 0) {
        ::close(listenSock);
        return -1;
    }

    struct pollfd pfd{listenSock, POLLIN, 0};
    if (::poll(&pfd, 1, timeoutMs) <= 0) {
        ::close(listenSock);
        return -1;
    }
    int sock = ::accept(listenSock, nullptr, nullptr);
    ::close(listenSock);
    if (sock < 0) return -1;

    // Disable Nagle for low-latency command/response.
    int one = 1;
    ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    return sock;
}

} // namespace tether::platform

#endif // !ESP_PLATFORM
