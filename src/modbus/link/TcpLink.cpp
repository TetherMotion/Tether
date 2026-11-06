#include "tether/modbus/link/TcpLink.hpp"

#if !defined(ESP_PLATFORM)

#include <cerrno>
#include <cstring>
#include <fcntl.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tether::modbus {
namespace {
ModbusError sysErr(std::string what, int e) {
    return ModbusError::transport(std::move(what) + ": " + std::strerror(e), e);
}
} // namespace

// ---- TcpLink -----------------------------------------------------------------
TcpLink::TcpLink(std::string host, uint16_t port, int connectMs,
                 OpenOpts opts)
    : host_(std::move(host)), port_(port), connectMs_(connectMs),
      opts_(opts) {}

TcpLink::TcpLink(int fd, std::string peer)
    : host_(peer), peer_(std::move(peer)), fd_(fd) {}

TcpLink::~TcpLink() { close(); }

Expected<void> TcpLink::open() {
    if (fd_.load() >= 0) return {};
    const int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return err(sysErr("socket", errno));

    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    const std::string portStr = std::to_string(port_);
    if (::getaddrinfo(host_.c_str(), portStr.c_str(), &hints, &res) != 0) {
        ::close(sock);
        return err(ModbusError::transport("getaddrinfo failed: " + host_));
    }
    int flags = ::fcntl(sock, F_GETFL, 0);
    ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int rc = ::connect(sock, res->ai_addr, res->ai_addrlen);
    ::freeaddrinfo(res);
    if (rc < 0 && errno != EINPROGRESS) {
        const int e = errno;
        ::close(sock);
        return err(sysErr("connect " + host_, e));
    }
    if (rc < 0) {
        pollfd pfd{sock, POLLOUT, 0};
        if (::poll(&pfd, 1, connectMs_) <= 0) {
            ::close(sock);
            return err(ModbusError::timedOut("tcp connect timeout"));
        }
        int soErr = 0;
        socklen_t len = sizeof(soErr);
        ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &soErr, &len);
        if (soErr != 0) {
            ::close(sock);
            return err(sysErr("connect " + host_, soErr));
        }
    }
    ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int one = 1;
    ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    fd_.store(sock);
    peer_ = host_ + ":" + portStr;
    return {};
}

void TcpLink::close() {
    // Exchange first so concurrent readers/writers see "closed" immediately;
    // shutdown() wakes any thread blocked in poll/recv on the old fd.
    const int f = fd_.exchange(-1);
    if (f >= 0) { ::shutdown(f, SHUT_RDWR); ::close(f); }
}
bool TcpLink::isOpen() const { return fd_.load() >= 0; }

Expected<void> TcpLink::write(std::span<const uint8_t> bytes,
                              int timeoutMs) {
    const int fd = fd_.load();
    if (fd < 0) return err(ModbusError::transport("not open"));
    size_t off = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    while (off < bytes.size()) {
        const ssize_t n = ::send(fd, bytes.data() + off,
                                 bytes.size() - off, MSG_NOSIGNAL);
        if (n > 0) { off += size_t(n); continue; }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const auto remain = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remain <= 0)
                return err(ModbusError::timedOut("tcp write timeout"));
            pollfd pfd{fd, POLLOUT, 0};
            ::poll(&pfd, 1, int(remain));
            continue;
        }
        const int e = errno;
        return err(sysErr("tcp send", e));
    }
    if (tap_) tap_(true, bytes);
    return {};
}

Expected<size_t> TcpLink::read(std::span<uint8_t> buf, int timeoutMs) {
    const int fd = fd_.load();
    if (fd < 0) return err<size_t>(ModbusError::transport("not open"));
    pollfd pfd{fd, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeoutMs);
    if (rc < 0) return err<size_t>(sysErr("poll", errno));
    if (rc == 0) return size_t{0};
    if (pfd.revents & (POLLERR | POLLNVAL))
        return err<size_t>(sysErr("tcp socket error", EIO));
    const ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return size_t{0};
        return err<size_t>(sysErr("tcp recv", errno));
    }
    if (n == 0)
        return err<size_t>(ModbusError::transport("peer closed", ECONNRESET));
    if (tap_) tap_(false, buf.subspan(0, size_t(n)));
    return size_t(n);
}

Expected<void> TcpLink::flush(Flush) { return {}; }

LinkInfo TcpLink::info() const {
    return {"tcp:" + peer_.empty() ? host_ + ":" + std::to_string(port_)
                                   : "tcp:" + peer_,
            peer_, LinkKind::Tcp, true, true};
}
int TcpLink::fd() const { return fd_.load(); }

// ---- TcpListener ---------------------------------------------------------------
TcpListener::TcpListener(std::string bindHost, uint16_t port, int backlog,
                         bool reuseAddr)
    : bindHost_(std::move(bindHost)), port_(port), backlog_(backlog),
      reuseAddr_(reuseAddr) {}

TcpListener::~TcpListener() { close(); }

Expected<void> TcpListener::listen() {
    if (fd_.load() >= 0) return {};
    const int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return err(sysErr("socket", errno));
    if (reuseAddr_) {
        int yes = 1;
        ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (bindHost_.empty() || bindHost_ == "0.0.0.0")
        addr.sin_addr.s_addr = INADDR_ANY;
    else if (::inet_pton(AF_INET, bindHost_.c_str(), &addr.sin_addr) != 1) {
        ::close(sock);
        return err(ModbusError::config("bad bind addr: " + bindHost_));
    }
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        const int e = errno;
        ::close(sock);
        return err(sysErr("bind", e));
    }
    if (::listen(sock, backlog_) < 0) {
        const int e = errno;
        ::close(sock);
        return err(sysErr("listen", e));
    }
    int flags = ::fcntl(sock, F_GETFL, 0);
    ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    fd_.store(sock);
    return {};
}

Expected<std::unique_ptr<ILink>> TcpListener::accept(int timeoutMs) {
    const int fd = fd_.load();
    if (fd < 0)
        return err<std::unique_ptr<ILink>>(
            ModbusError::transport("not listening"));
    pollfd pfd{fd, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeoutMs);
    if (rc < 0)
        return err<std::unique_ptr<ILink>>(sysErr("poll", errno));
    if (rc == 0)
        return err<std::unique_ptr<ILink>>(ModbusError::timedOut("accept"));
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    const int sock = ::accept(fd, reinterpret_cast<sockaddr*>(&peer), &len);
    if (sock < 0)
        return err<std::unique_ptr<ILink>>(sysErr("accept", errno));
    int flags = ::fcntl(sock, F_GETFL, 0);
    ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int one = 1;
    ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    char ip[INET_ADDRSTRLEN];
    ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
    const std::string peerStr = std::string(ip) + ":" +
                                std::to_string(ntohs(peer.sin_port));
    return std::unique_ptr<ILink>{std::make_unique<TcpLink>(sock, peerStr)};
}

void TcpListener::close() {
    const int f = fd_.exchange(-1);
    if (f >= 0) { ::shutdown(f, SHUT_RDWR); ::close(f); }
}
bool TcpListener::isListening() const { return fd_.load() >= 0; }
std::string TcpListener::boundEndpoint() const {
    return bindHost_ + ":" + std::to_string(port_);
}

Expected<std::unique_ptr<ILink>>
makeTcpLink(std::string_view host, uint16_t port, int connectMs,
            const OpenOpts& opts) {
    return std::unique_ptr<ILink>{
        std::make_unique<TcpLink>(std::string(host), port, connectMs, opts)};
}

} // namespace tether::modbus

#endif // !ESP_PLATFORM
