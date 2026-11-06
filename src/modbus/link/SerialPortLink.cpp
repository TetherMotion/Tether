#include "tether/modbus/link/SerialPortLink.hpp"

#if !defined(ESP_PLATFORM)

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/serial.h>
#endif

namespace tether::modbus {
namespace {

speed_t baudToSpeed(uint32_t baud) {
    switch (baud) {
        case 50: return B50; case 75: return B75; case 110: return B110;
        case 134: return B134; case 150: return B150; case 200: return B200;
        case 300: return B300; case 600: return B600; case 1200: return B1200;
        case 1800: return B1800; case 2400: return B2400; case 4800: return B4800;
        case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B576000
        case 576000: return B576000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1152000
        case 1152000: return B1152000;
#endif
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
#ifdef B2500000
        case 2500000: return B2500000;
#endif
#ifdef B3000000
        case 3000000: return B3000000;
#endif
#ifdef B3500000
        case 3500000: return B3500000;
#endif
#ifdef B4000000
        case 4000000: return B4000000;
#endif
        default: return B0;
    }
}

ModbusError sysErr(std::string what, int e) {
    return ModbusError::transport(what + ": " + std::strerror(e), e);
}

} // namespace

SerialPortLink::SerialPortLink(SerialConfig cfg, OpenOpts opts)
    : cfg_(std::move(cfg)), opts_(opts) {}

SerialPortLink::~SerialPortLink() { close(); }

Expected<void> SerialPortLink::applyTermios(int fd, const SerialConfig& c,
                                            const OpenOpts& o) {
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0)
        return err(sysErr("tcgetattr", errno));

    ::cfmakeraw(&tio);
    const speed_t spd = baudToSpeed(c.baud);
    if (spd == B0)
        return err(ModbusError::config(
            "unsupported baud: " + std::to_string(c.baud)));
    ::cfsetispeed(&tio, spd);
    ::cfsetospeed(&tio, spd);

    tio.c_cflag &= ~CSIZE;
    switch (c.dataBits) {
        case 5: tio.c_cflag |= CS5; break;
        case 6: tio.c_cflag |= CS6; break;
        case 7: tio.c_cflag |= CS7; break;
        default: tio.c_cflag |= CS8; break;
    }
    switch (c.parity) {
        case Parity::None: tio.c_cflag &= ~(PARENB | PARODD); break;
        case Parity::Even: tio.c_cflag |= PARENB; tio.c_cflag &= ~PARODD; break;
        case Parity::Odd:  tio.c_cflag |= PARENB | PARODD; break;
    }
    if (c.stopBits == StopBits::Two) tio.c_cflag |= CSTOPB;
    else                            tio.c_cflag &= ~CSTOPB;

    // Hardware parity/framing error checking (only meaningful when the
    // UART actually generates a parity bit). Clear first — cfmakeraw
    // leaves INPCK/IGNPAR alone so bits from an earlier configure leak.
    tio.c_iflag &= ~(INPCK | IGNPAR | PARMRK);
    if (c.parity != Parity::None) {
        switch (o.parityCheck) {
            case ParityCheck::Passthrough: break;
            case ParityCheck::Drop:
                tio.c_iflag |= INPCK | IGNPAR;
                break;
            case ParityCheck::Mark:
                tio.c_iflag |= INPCK | PARMRK;
                break;
        }
    }

    tio.c_cflag |= CLOCAL | CREAD;
    switch (c.flow) {
        case FlowControl::RtsCts:
#ifdef CRTSCTS
            tio.c_cflag |= CRTSCTS;
            break;
#else
            return err(ModbusError::config("rtscts unsupported"));
#endif
        case FlowControl::XonXoff:
            tio.c_iflag |= IXON | IXOFF;
            break;
        default: break;
    }
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        // Drivers without hardware parity (ptys, virtual ports) silently
        // drop PARENB/PARODD; glibc detects the read-back mismatch and
        // reports EINVAL. Retry without the parity bits — the link still
        // works (INPCK/PARMRK stay set so injected mark sequences are
        // still un-marked) and parityApplied() reports the fallback.
        if (errno != EINVAL || c.parity == Parity::None)
            return err(sysErr("tcsetattr", errno));
        tio.c_cflag &= ~(PARENB | PARODD);
        if (::tcsetattr(fd, TCSANOW, &tio) != 0)
            return err(sysErr("tcsetattr", errno));
    }
    return {};
}

Expected<void> SerialPortLink::setRts(bool on) {
#ifdef TIOCMBIS
    int status;
    if (::ioctl(fd_, TIOCMGET, &status) != 0)
        return err(sysErr("TIOCMGET", errno));
    if (on) status |= TIOCM_RTS; else status &= ~TIOCM_RTS;
    if (::ioctl(fd_, TIOCMSET, &status) != 0)
        return err(sysErr("TIOCMSET", errno));
    return {};
#else
    (void)on;
    return err(ModbusError::unsupported("RTS control unavailable"));
#endif
}

Expected<void> SerialPortLink::openOnce() {
    int flags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    fd_ = ::open(cfg_.device.c_str(), flags);
    if (fd_ < 0) return err(sysErr("open " + cfg_.device, errno));
    if (opts_.exclusive) {
        if (::ioctl(fd_, TIOCEXCL) != 0) {
            const int e = errno;
            ::close(fd_); fd_ = -1;
            return err(sysErr("TIOCEXCL", e));
        }
    }
    if (auto e = applyTermios(fd_, cfg_, opts_); !e) {
        ::close(fd_); fd_ = -1;
        return err(e.error());
    }
    ::tcflush(fd_, TCIOFLUSH);
    unmarker_ = ParityUnmarker{};

    // Read back what the driver actually accepted. PARENB may be dropped
    // on parity-less ports (ptys, virtual ports) — parityApplied_ reports
    // that for diagnostics. Un-marking keys off INPCK|PARMRK instead:
    // when those are set the tty PARMRK-escapes the input stream
    // (literal 0xFF arrives doubled even without PARENB), so un-marking
    // is required for plain data integrity, not just error marking.
    termios rb{};
    const bool gotRb = ::tcgetattr(fd_, &rb) == 0;
    parityApplied_ = cfg_.parity == Parity::None ||
                     (gotRb && (rb.c_cflag & PARENB));
    unmarkActive_ = gotRb &&
                    (rb.c_iflag & (INPCK | PARMRK)) == (INPCK | PARMRK);

#ifdef __linux__
    if (opts_.rs485) {
        serial_rs485 rs{};
        rs.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
        if (opts_.rs485DelayUs > 0) {
            rs.flags |= SER_RS485_RTS_AFTER_SEND;
            rs.delay_rts_after_send = uint32_t(opts_.rs485DelayUs);
        }
        ::ioctl(fd_, TIOCSRS485, &rs); // best-effort; manual_rts falls back
    }
#endif
    // Return to blocking-with-poll semantics (fd stays O_NONBLOCK; we poll).
    return {};
}

Expected<void> SerialPortLink::open() {
    if (fd_ >= 0) return {};
    return openOnce();
}

void SerialPortLink::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialPortLink::isOpen() const { return fd_ >= 0; }

Expected<void> SerialPortLink::write(std::span<const uint8_t> bytes,
                                     int timeoutMs) {
    if (fd_ < 0) return err(ModbusError::transport("not open"));
    if (opts_.rtsBefore) {
        if (auto e = setRts(true); !e) return e;
        if (opts_.rs485DelayUs > 0)
            ::usleep(uint32_t(opts_.rs485DelayUs));
    }
    size_t off = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    while (off < bytes.size()) {
        const ssize_t n = ::write(fd_, bytes.data() + off, bytes.size() - off);
        if (n > 0) { off += size_t(n); continue; }
        if (n < 0 && errno == EAGAIN) {
            auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remain <= 0) {
                if (opts_.rtsBefore) (void)setRts(false);
                return err(ModbusError::timedOut("serial write timeout"));
            }
            pollfd pfd{fd_, POLLOUT, 0};
            ::poll(&pfd, 1, int(remain));
            continue;
        }
        const int e = errno;
        if (opts_.rtsBefore) (void)setRts(false);
        return err(sysErr("serial write", e));
    }
    ::tcdrain(fd_);
    if (opts_.rtsBefore) {
        if (opts_.rs485DelayUs > 0)
            ::usleep(uint32_t(opts_.rs485DelayUs));
        (void)setRts(false);
    }
    if (tap_) tap_(true, bytes);
    return {};
}

Expected<size_t> SerialPortLink::read(std::span<uint8_t> buf, int timeoutMs) {
    if (fd_ < 0) return err<size_t>(ModbusError::transport("not open"));
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeoutMs);
    if (rc < 0) return err<size_t>(sysErr("poll", errno));
    if (rc == 0) return size_t{0};
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return err<size_t>(sysErr("serial hangup", EIO));
    const ssize_t n = ::read(fd_, buf.data(), buf.size());
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return size_t{0};
        return err<size_t>(sysErr("serial read", errno));
    }
    if (n == 0) return size_t{0};
    if (unmarkActive_) {
        // PARMRK mode: strip FF 00 <err-byte> marks / collapse FF FF.
        const std::vector<uint8_t> marked(buf.begin(), buf.begin() + n);
        std::vector<uint8_t> clean;
        parityMarkedErrors_ += unmarker_.feed(marked, clean);
        if (clean.empty()) return size_t{0};
        const size_t out = std::min(clean.size(), buf.size());
        std::copy_n(clean.begin(), out, buf.begin());
        if (tap_) tap_(false, buf.subspan(0, out));
        return out;
    }
    if (tap_) tap_(false, buf.subspan(0, size_t(n)));
    return size_t(n);
}

Expected<void> SerialPortLink::flush(Flush f) {
    if (fd_ < 0) return err(ModbusError::transport("not open"));
    switch (f) {
        case Flush::Rx:   ::tcflush(fd_, TCIFLUSH); break;
        case Flush::Tx:   ::tcflush(fd_, TCOFLUSH); break;
        case Flush::Both: ::tcflush(fd_, TCIOFLUSH); break;
    }
    return {};
}

Expected<size_t> SerialPortLink::drain() {
    if (fd_ < 0) return err<size_t>(ModbusError::transport("not open"));
    int avail = 0;
    if (::ioctl(fd_, FIONREAD, &avail) != 0)
        return err<size_t>(sysErr("FIONREAD", errno));
    if (avail <= 0) return size_t{0};
    std::vector<uint8_t> tmp; tmp.resize(size_t(avail));
    const ssize_t n = ::read(fd_, tmp.data(), tmp.size());
    return n > 0 ? size_t(n) : size_t{0};
}

LinkInfo SerialPortLink::info() const {
    return {"serial:" + cfg_.describe(), cfg_.describe(),
            LinkKind::Serial, true, true};
}

int SerialPortLink::fd() const { return fd_; }

Expected<SerialPortLink::SerialErrorCounters>
SerialPortLink::serialErrors() const {
#ifdef TIOCGICOUNT
    if (fd_ < 0) return err<SerialErrorCounters>(
        ModbusError::transport("not open"));
    struct serial_icounter_struct ic{};
    if (::ioctl(fd_, TIOCGICOUNT, &ic) != 0)
        return err<SerialErrorCounters>(sysErr("TIOCGICOUNT", errno));
    SerialErrorCounters c;
    c.frame = ic.frame;
    c.parity = ic.parity;
    c.overrun = ic.overrun;
    c.brk = ic.brk;
    return c;
#else
    return err<SerialErrorCounters>(
        ModbusError::unsupported("serial error counters unavailable"));
#endif
}

Expected<std::unique_ptr<ILink>>
makeSerialLink(const SerialConfig& cfg, const OpenOpts& opts) {
    return std::make_unique<SerialPortLink>(cfg, opts);
}

} // namespace tether::modbus

#endif // !ESP_PLATFORM
