/**
 * @file tether_io_serial_transport.cpp
 * @brief SerialTransport over a pseudo-terminal: the UART path without
 *        hardware.
 *
 * PosixSerialDriver opens a tty/pty path and applies termios+baud;
 * SerialTransport adapts it to ITransport. Serial links are
 * byte-stream oriented, so the session runs the default SLIP framing.
 * A serial link is point-to-point — there is no accept(); the server
 * side is a bare Session on the other end (here: the pty master fd,
 * wrapped in a minimal fd transport).
 *
 *   ./tether_io_serial_transport
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SerialTransport.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Transport.hpp"

#include <cerrno>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <pty.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace tether::io;

namespace {

// Minimal ITransport over one fd — the "other end" of the pty.
class FdTransport : public ITransport {
public:
    explicit FdTransport(int fd) : fd_(fd) {}
    ~FdTransport() override { close(); }

    bool send(const uint8_t* data, size_t len) override {
        return ::write(fd_, data, len) == static_cast<ssize_t>(len);
    }
    size_t receive(uint8_t* buf, size_t maxLen,
                   uint32_t timeoutMs) override {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        timeval tv{static_cast<time_t>(timeoutMs / 1000),
                   static_cast<suseconds_t>((timeoutMs % 1000) * 1000)};
        if (::select(fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0)
            return 0;
        const ssize_t n = ::read(fd_, buf, maxLen);
        return n > 0 ? static_cast<size_t>(n) : 0;
    }
    void close() override {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }
    bool isConnected() const override { return fd_ >= 0; }
private:
    int fd_;
};

} // namespace

int main() {
    // Virtual serial pair: master fd <-/-> slave path.
    int masterFd = -1, slaveFd = -1;
    char slaveName[64]{};
    if (::openpty(&masterFd, &slaveFd, slaveName, nullptr, nullptr) != 0) {
        std::fprintf(stderr, "openpty failed: %s\n", std::strerror(errno));
        return 1;
    }
    ::close(slaveFd);  // the driver re-opens by path
    std::printf("[setup] pty pair: master fd=%d, slave=%s\n", masterFd,
                slaveName);

    Registry registry;
    {
        ParamEntry p;
        p.id = 1;
        p.name = "answer";
        p.group = "demo";
        p.valueType = ValueType::U32;
        p.readFn = [](void* d) { const uint32_t v = 42; std::memcpy(d, &v, 4); };
        if (!registry.addParam(p)) { ::close(masterFd); return 1; }
    }

    // Server side: a Session directly on the pty master (serial links
    // are point-to-point — no listener).
    Session session(std::make_unique<FdTransport>(masterFd), registry,
                    steadyClockUs);
    std::thread sessionThread([&] { session.run(); });

    // Client side: SerialTransport on the pty slave path.
    auto driver = std::make_unique<PosixSerialDriver>();
    if (!driver->open(slaveName, 115200)) {
        std::fprintf(stderr, "driver open %s failed\n", slaveName);
        session.requestStop();
        sessionThread.join();
        return 1;
    }
    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(
            std::make_unique<SerialTransport>(std::move(driver))),
        2000);

    const auto v = client.getParam(1);
    uint32_t answer = 0;
    if (v && v->size() == 4) std::memcpy(&answer, v->data(), 4);
    std::printf("[client] over serial pty: answer=%u\n", answer);

    client.close();
    session.requestStop();
    sessionThread.join();
    return answer == 42 ? 0 : 1;
}
