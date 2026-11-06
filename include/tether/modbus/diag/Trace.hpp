/**
 * @file Trace.hpp
 * @brief Wire trace hooks + hex-dump frame logger (plan §18).
 */
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "tether/modbus/link/Framer.hpp"
#include "tether/modbus/link/ILink.hpp"

namespace tether::modbus {

struct WireEvent {
    enum Dir { Tx, Rx } dir;
    enum class Kind { Bytes, Frame, Drop, Open, Close, Error } kind;
    std::vector<uint8_t> bytes;
    std::string note;
    int64_t ns = 0;    ///< steady-clock timestamp
};

using TraceSink = std::function<void(const WireEvent&)>;

/// Attach a sink to an ILink via setTap → every byte block traced.
void attachByteTap(ILink& link, TraceSink sink);

/**
 * @brief Frame-level logger: reassembles ADUs per direction and calls
 * `log(line)` — "→ RTU unit=1 fc=03 len=8 : 01 03 00 6B 00 03 76 87".
 */
class FrameLogger {
public:
    FrameLogger(std::function<void(const std::string&)> log,
                bool rtu = true)
        : log_(std::move(log)), rtu_(rtu) {}

    /// Feed from a byte tap: tap_(dir, bytes).
    void tap(bool tx, std::span<const uint8_t> bytes);
    /// Log one known-complete frame directly.
    void frame(bool tx, uint8_t unit, std::span<const uint8_t> pdu,
               uint16_t txn = 0);

private:
    std::function<void(const std::string&)> log_;
    bool rtu_;
    IFramer* mkFramer();
    std::unique_ptr<IFramer> txF_, rxF_;
};

/// hex dump "01 03 00 6B" for logs.
std::string hexDump(std::span<const uint8_t> b);

/// Capture file: either libpcap (Wireshark-readable) or a compact
/// "MBCAP" record stream (little-endian: dir, len, ns, bytes).
class CaptureFile {
public:
    enum class Format { Mbcap, Pcap };
    ~CaptureFile() { close(); }
    /// MBCAP stream. `rtu` is recorded in the header.
    Expected<void> open(const std::string& path, bool rtu);
    /// Classic libpcap: LINKTYPE_USER0 (147) frames, little-endian.
    Expected<void> openPcap(const std::string& path);
    void record(bool tx, std::span<const uint8_t> bytes);
    void close();
    bool isOpen() const { return f_ != nullptr; }
private:
    void* f_ = nullptr;
    Format fmt_ = Format::Mbcap;
    std::mutex m_;
};

} // namespace tether::modbus
