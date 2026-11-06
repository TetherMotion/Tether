#include "tether/modbus/diag/Trace.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>

#include "tether/modbus/link/Framer.hpp"
#include "tether/modbus/protocol/Crc.hpp"
#include "tether/modbus/protocol/FunctionCode.hpp"

namespace tether::modbus {

std::string hexDump(std::span<const uint8_t> b) {
    std::string s;
    char tmp[4];
    for (size_t i = 0; i < b.size(); ++i) {
        std::snprintf(tmp, sizeof(tmp), "%02X", b[i]);
        s += tmp;
        if (i + 1 < b.size()) s += ' ';
    }
    return s;
}

void attachByteTap(ILink& link, TraceSink sink) {
    link.setTap([sink = std::move(sink)](bool tx,
                                         std::span<const uint8_t> b) {
        WireEvent e;
        e.dir = tx ? WireEvent::Tx : WireEvent::Rx;
        e.kind = WireEvent::Kind::Bytes;
        e.bytes.assign(b.begin(), b.end());
        e.ns = std::chrono::steady_clock::now()
                   .time_since_epoch().count();
        sink(e);
    });
}

IFramer* FrameLogger::mkFramer() {
    if (rtu_) return new RtuFramer(); else return new MbapFramer();
}

void FrameLogger::tap(bool tx, std::span<const uint8_t> bytes) {
    auto& f = tx ? txF_ : rxF_;
    if (!f) f.reset(mkFramer());
    std::vector<Adu> adus;
    f->feed(bytes, adus);
    for (const auto& a : adus)
        frame(tx, a.unit, a.pdu, a.txn);
}

void FrameLogger::frame(bool tx, uint8_t unit, std::span<const uint8_t> pdu,
                        uint16_t txn) {
    if (!log_) return;
    std::string line = tx ? "-> " : "<- ";
    line += rtu_ ? "rtu" : "tcp";
    line += " unit=" + std::to_string(unit);
    if (txn) line += " txn=" + std::to_string(txn);
    if (!pdu.empty()) {
        const auto fc = static_cast<FunctionCode>(pdu[0] & 0x7F);
        line += " fc=" + std::string(fcName(fc));
        if (pdu[0] & 0x80) line += "!(exc)";
    }
    line += " : " + hexDump(pdu);
    log_(line);
}

// ---- CaptureFile -----------------------------------------------------------------
Expected<void> CaptureFile::open(const std::string& path, bool rtu) {
    close();
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_)
        return err(ModbusError::transport("open capture: " + path));
    fmt_ = Format::Mbcap;
    const char magic[8] = {'M','B','C','A','P','0', rtu ? 'R' : 'T', '\n'};
    std::fwrite(magic, 1, 8, static_cast<FILE*>(f_));
    return {};
}

Expected<void> CaptureFile::openPcap(const std::string& path) {
    close();
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_)
        return err(ModbusError::transport("open capture: " + path));
    fmt_ = Format::Pcap;
    // libpcap global header, little-endian, µs timestamps,
    // LINKTYPE_USER0 (147) so each record is a raw ADU.
    const uint8_t gh[24] = {
        0xd4, 0xc3, 0xb2, 0xa1,             // magic LE
        2, 0, 4, 0,                          // ver 2.4
        0, 0, 0, 0, 0, 0, 0, 0,              // tz/sigfigs
        0xff, 0xff, 0x00, 0x00,              // snaplen
        0x93, 0x00, 0x00, 0x00};             // LINKTYPE_USER0
    std::fwrite(gh, 1, sizeof(gh), static_cast<FILE*>(f_));
    return {};
}

void CaptureFile::record(bool tx, std::span<const uint8_t> bytes) {
    if (!f_) return;
    std::lock_guard g(m_);
    FILE* f = static_cast<FILE*>(f_);
    if (fmt_ == Format::Pcap) {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const uint64_t us =
            uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(now)
                         .count());
        const uint32_t n = uint32_t(bytes.size());
        // pcap rec hdr: ts_sec, ts_usec, incl_len, orig_len (+ dir tag in
        // first byte of the payload so direction survives the format).
        const uint8_t hdr[16] = {
            uint8_t(us / 1000000), uint8_t(us / 1000000 >> 8),
            uint8_t(us / 1000000 >> 16), uint8_t(us / 1000000 >> 24),
            uint8_t(us % 1000000), uint8_t(us % 1000000 >> 8),
            uint8_t(us % 1000000 >> 16), uint8_t(us % 1000000 >> 24),
            uint8_t(n + 1), uint8_t((n + 1) >> 8), 0, 0,
            uint8_t(n + 1), uint8_t((n + 1) >> 8), 0, 0};
        std::fwrite(hdr, 1, sizeof(hdr), f);
        const uint8_t d = uint8_t(tx);
        std::fwrite(&d, 1, 1, f);
        std::fwrite(bytes.data(), 1, bytes.size(), f);
    } else {
        const int64_t ns = std::chrono::steady_clock::now()
                               .time_since_epoch().count();
        const uint8_t hdr[16] = {
            uint8_t(tx), 0, 0, 0,
            uint8_t(bytes.size()), uint8_t(bytes.size() >> 8), 0, 0,
            uint8_t(ns), uint8_t(ns >> 8), uint8_t(ns >> 16),
            uint8_t(ns >> 24), uint8_t(ns >> 32), uint8_t(ns >> 40),
            uint8_t(ns >> 48), uint8_t(ns >> 56)};
        std::fwrite(hdr, 1, sizeof(hdr), f);
        std::fwrite(bytes.data(), 1, bytes.size(), f);
    }
    std::fflush(f);
}

void CaptureFile::close() {
    if (f_) { std::fclose(static_cast<FILE*>(f_)); f_ = nullptr; }
}

} // namespace tether::modbus
