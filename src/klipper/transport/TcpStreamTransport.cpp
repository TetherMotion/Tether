/**
 * @file TcpStreamTransport.cpp
 * @brief TCP/IP stream transport implementation.
 *
 * Delegates connection setup to `tether::platform::posixTcpConnect()` /
 * `posixTcpListenAccept()` and data transfer to `tether::io::TcpTransport`,
 * eliminating the duplicated POSIX socket code that previously existed here.
 */

#include "tether/klipper/transport/TcpStreamTransport.hpp"

#if !defined(ESP_PLATFORM)

#include <sys/ioctl.h>
#include <poll.h>

namespace tether::klipper::transport {

TcpStreamTransport::~TcpStreamTransport() {
    close();
}

bool TcpStreamTransport::open() {
    if (open_) return true;

    int fd = -1;
    if (config_.mode == "server") {
        fd = tether::platform::posixTcpListenAccept(
            config_.port, config_.backlog, config_.timeoutMs);
    } else {
        fd = tether::platform::posixTcpConnect(
            config_.host, config_.port, config_.timeoutMs);
    }
    if (fd < 0) return false;

    transport_ = std::make_unique<tether::io::TcpTransport>(fd);
    open_ = true;
    return true;
}

bool TcpStreamTransport::isOpen() const { return open_; }

void TcpStreamTransport::close() {
    if (!open_.exchange(false)) return;
    if (transport_) {
        transport_->close();
        transport_.reset();
    }
}

size_t TcpStreamTransport::write(std::span<const uint8_t> data) {
    if (!open_ || !transport_ || data.empty()) return 0;
    bool ok = transport_->send(data.data(), data.size());
    return ok ? data.size() : 0;
}

size_t TcpStreamTransport::available() const {
    if (!open_ || !transport_) return 0;
    // io::TcpTransport doesn't expose FIONREAD; use a zero-timeout poll via
    // isConnected() as a proxy.  The protocol layer will call read() which
    // handles the actual data retrieval.
    return transport_->isConnected() ? 1 : 0;
}

size_t TcpStreamTransport::read(uint8_t* out, size_t maxLen, bool canBlock) {
    if (!open_ || !transport_ || maxLen == 0) return 0;
    uint32_t timeoutMs = canBlock ? 0 : 1;  // 0 = block forever in io::TcpTransport
    if (!canBlock) {
        // Non-blocking: use a 1ms timeout probe.
        size_t n = transport_->receive(out, maxLen, 1);
        return n;
    }
    // Blocking: io::TcpTransport::receive with timeoutMs=0 means no wait.
    // Use a long timeout instead for blocking behaviour.
    return transport_->receive(out, maxLen, 60000);
}

} // namespace tether::klipper::transport

#else // ESP_PLATFORM

namespace tether::klipper::transport {

TcpStreamTransport::~TcpStreamTransport() { close(); }
bool TcpStreamTransport::open() { return false; }
bool TcpStreamTransport::isOpen() const { return false; }
void TcpStreamTransport::close() { open_ = false; }
size_t TcpStreamTransport::write(std::span<const uint8_t>) { return 0; }
size_t TcpStreamTransport::available() const { return 0; }
size_t TcpStreamTransport::read(uint8_t*, size_t, bool) { return 0; }

} // namespace tether::klipper::transport

#endif // ESP_PLATFORM
