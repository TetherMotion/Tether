#include "tether/modbus/link/UdpLink.hpp"

#if defined(TETHER_MODBUS_UDP) && !defined(ESP_PLATFORM)

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tether::modbus {
namespace {

ModbusError sysErr(const char* what) {
    return ModbusError::transport(
        std::string(what) + ": " + std::strerror(errno), errno);
}

int waitFd(int fd, short events, int timeoutMs) {
    pollfd p{fd, events, 0};
    const int r = ::poll(&p, 1, timeoutMs);
    if (r <= 0) return r;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) return -2;
    return (p.revents & events) ? 1 : 0;
}

} // namespace

UdpLink::UdpLink(std::string host, uint16_t port, int connectMs,
                 OpenOpts opts)
    : host_(std::move(host)), port_(port), connectMs_(connectMs),
      opts_(opts) {}

UdpLink::UdpLink(int fd, std::string peer)
    : peer_(std::move(peer)), fd_(fd) {}

UdpLink::~UdpLink() { close(); }

Expected<void> UdpLink::open() {
    if (fd_ >= 0) return {};
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;
    const auto portStr = std::to_string(port_);
    if (::getaddrinfo(host_.c_str(), portStr.c_str(), &hints, &res) != 0 ||
        !res)
        return err(ModbusError::transport(
            "udp resolve failed: " + host_));
    int fd = -1;
    std::string peer;
    for (auto* ai = res; ai; ai = ai->ai_next) {
        fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;
        if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
            char h[NI_MAXHOST], s[NI_MAXSERV];
            if (::getnameinfo(ai->ai_addr, ai->ai_addrlen, h, sizeof h,
                              s, sizeof s,
                              NI_NUMERICHOST | NI_NUMERICSERV) == 0)
                peer = std::string(h) + ":" + s;
            break;
        }
        ::close(fd); fd = -1;
    }
    ::freeaddrinfo(res);
    if (fd < 0)
        return err(ModbusError::transport("udp connect failed"));
    // Nonblocking: all waits go through poll().
    const int fl = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    // Confirm the peer actually answers: connected UDP delivers ICMP
    // port-unreachable as ECONNREFUSED on the next op — surface it at
    // first read instead of here (UDP has no handshake).
    fd_ = fd;
    peer_ = peer.empty() ? host_ + ":" + portStr : peer;
    return {};
}

void UdpLink::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool UdpLink::isOpen() const { return fd_ >= 0; }

Expected<void> UdpLink::write(std::span<const uint8_t> bytes,
                              int timeoutMs) {
    if (fd_ < 0) return err(ModbusError::transport("udp closed"));
    const auto dl = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);
    for (;;) {
        const ssize_t n = ::send(fd_, bytes.data(), bytes.size(), 0);
        if (n == ssize_t(bytes.size())) {
            if (tap_) tap_(true, bytes);
            return {};
        }
        if (n >= 0)   // partial datagram (shouldn't happen) → treated
            return err(ModbusError::transport("udp short datagram"));
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            const auto left =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    dl - std::chrono::steady_clock::now()).count();
            if (left <= 0)
                return err(ModbusError::timedOut("udp send timeout"));
            if (waitFd(fd_, POLLOUT, int(left)) <= 0)
                return err(ModbusError::timedOut("udp send timeout"));
            continue;
        }
        return err(sysErr("udp send"));
    }
}

Expected<size_t> UdpLink::read(std::span<uint8_t> buf, int timeoutMs) {
    if (fd_ < 0)
        return err<size_t>(ModbusError::transport("udp closed"));
    const auto dl = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);
    for (;;) {
        const ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
        if (n >= 0) {
            if (tap_ && n > 0) tap_(false, buf.first(size_t(n)));
            return size_t(n);
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            const auto left =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    dl - std::chrono::steady_clock::now()).count();
            if (left <= 0) return size_t{0};
            const int w = waitFd(fd_, POLLIN, int(left));
            if (w == 0) return size_t{0};
            if (w < 0)
                return err<size_t>(sysErr("udp poll"));
            continue;
        }
        if (errno == EINTR) continue;
        return err<size_t>(sysErr("udp recv"));
    }
}

Expected<void> UdpLink::flush(Flush f) {
    if (f == Flush::Rx || f == Flush::Both) (void)drain();
    return {};
}

Expected<size_t> UdpLink::drain() {
    if (fd_ < 0) return size_t{0};
    uint8_t junk[512];
    size_t total = 0;
    while (::recv(fd_, junk, sizeof junk, MSG_DONTWAIT) > 0)
        ++total;
    return total;
}

LinkInfo UdpLink::info() const {
    return {"udp", peer_.empty() ? host_ : peer_, LinkKind::Udp,
            true, false};
}

Expected<std::unique_ptr<ILink>>
makeUdpLink(std::string_view host, uint16_t port, int connectMs,
            const OpenOpts& opts) {
    auto l = std::make_unique<UdpLink>(std::string(host), port, connectMs,
                                       opts);
    if (auto e = l->open(); !e)
        return err<std::unique_ptr<ILink>>(e.error());
    return std::unique_ptr<ILink>(std::move(l));
}

} // namespace tether::modbus

#endif // TETHER_MODBUS_UDP && !ESP_PLATFORM
