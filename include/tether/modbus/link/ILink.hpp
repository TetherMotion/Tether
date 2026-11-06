/**
 * @file ILink.hpp
 * @brief Byte-stream link abstraction (plan AN.1).
 */
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

#include "tether/modbus/ModbusError.hpp"

namespace tether::modbus {

struct LinkError {
    enum class Kind { Open, Io, Timeout, Closed, Busy, Unsupported } kind;
    int sysErrno = 0;
    std::string message;
};

enum class LinkKind { Serial, Tcp, Udp, Loopback, Pty, Script, Sim };
enum class Reopen { Auto, Manual };
enum class Flush  { Rx, Tx, Both };

/**
 * How a serial link treats bytes the UART flagged with parity/framing
 * errors. Only meaningful when the port has parity enabled.
 *  - Passthrough: parity errors unchecked (INPCK off) — bad bytes arrive
 *    as garbage; the Modbus CRC still rejects the frame.
 *  - Drop:        INPCK|IGNPAR — errored bytes are discarded by the tty
 *    layer; frame arrives short → CRC/length reject (byte-loss path).
 *  - Mark:        INPCK|PARMRK — errored bytes arrive prefixed 0xFF 0x00
 *    (literal 0xFF doubled); the link un-marks them, counts errors via
 *    parityMarkedErrors(), and keeps the byte so positions stay intact.
 */
enum class ParityCheck { Passthrough, Drop, Mark };

struct OpenOpts {
    Reopen reopen = Reopen::Auto;
    int    reopenDelayMs = 500;
    int    ioPollMs = 1;
    bool   exclusive = true;
    bool   rs485 = false;
    int    rs485DelayUs = 0;
    bool   rtsBefore = false;
    ParityCheck parityCheck = ParityCheck::Mark;
};

struct LinkInfo {
    std::string name;
    std::string detail;
    LinkKind    kind = LinkKind::Loopback;
    bool        fdBacked = true;
    bool        stream = true;
};

/**
 * @brief Blocking byte-stream transport with millisecond deadlines.
 *
 * A `read` timeout is NOT an error — it returns 0 bytes. Errors are
 * only I/O faults. One calling thread at a time (channels own links).
 */
class ILink {
public:
    virtual ~ILink() = default;

    virtual Expected<void>   open() = 0;
    virtual void             close() = 0;
    virtual bool             isOpen() const = 0;

    /// Write ALL bytes or fail. May block up to timeoutMs.
    virtual Expected<void>   write(std::span<const uint8_t> bytes,
                                   int timeoutMs) = 0;
    /// Read up to buf.size(); 0 = timeout elapsed with no data.
    virtual Expected<size_t> read(std::span<uint8_t> buf,
                                  int timeoutMs) = 0;

    virtual Expected<void>   flush(Flush) = 0;
    virtual LinkInfo         info() const = 0;
    LinkKind                 kind() const { return info().kind; }
    virtual int              fd() const { return -1; }

    /// Best-effort discard of already-arrived bytes (echo removal).
    virtual Expected<size_t> drain() { return size_t{0}; }

    /// Optional raw tap for capture tooling.
    virtual void setTap(std::function<void(bool tx,
                        std::span<const uint8_t>)>) {}
};

/**@brief Listener for server-side TCP. */
class ILinkListener {
public:
    virtual ~ILinkListener() = default;
    virtual Expected<void> listen() = 0;
    virtual Expected<std::unique_ptr<ILink>> accept(int timeoutMs) = 0;
    virtual void             close() = 0;
    virtual bool             isListening() const = 0;
    virtual std::string      boundEndpoint() const = 0;
};

// ---- factories --------------------------------------------------------------
struct SerialConfig;
Expected<std::unique_ptr<ILink>>
    makeSerialLink(const SerialConfig& cfg, const OpenOpts& opts = {});
Expected<std::unique_ptr<ILink>>
    makeTcpLink(std::string_view host, uint16_t port, int connectMs,
                const OpenOpts& opts = {});
Expected<std::unique_ptr<ILink>> makeLoopbackLink();
Expected<std::unique_ptr<ILink>>
    makeTcpListener(std::string_view bindHost, uint16_t port, int backlog,
                    bool reuseAddr);
Expected<std::unique_ptr<ILink>>
    makeUdpLink(std::string_view host, uint16_t port, int connectMs,
                const OpenOpts& opts = {});

} // namespace tether::modbus
