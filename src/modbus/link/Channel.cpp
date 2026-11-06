#include "tether/modbus/link/Channel.hpp"

#include <thread>

namespace tether::modbus {
namespace {
inline int64_t nowNs() {
    return std::chrono::steady_clock::now().time_since_epoch().count();
}
constexpr int64_t kNsPerUs = 1000;
constexpr int64_t kNsPerMs = 1000000;
} // namespace

// ---- RtuChannel ------------------------------------------------------------------
RtuChannel::RtuChannel(std::unique_ptr<ILink> link, RtuTiming timing,
                       ChannelOpts opts)
    : link_(std::move(link)), timing_(timing), opts_(opts) {}

Expected<void> RtuChannel::open() { return link_->open(); }
void RtuChannel::close() { link_->close(); }

Expected<std::vector<Adu>> RtuChannel::collect(int firstByteTimeoutMs,
                                               int totalTimeoutMs) {
    framer_.reset();
    std::vector<Adu> out;
    uint8_t buf[256];
    const int64_t start = nowNs();
    const int64_t t15Ns = timing_.t15.count() * kNsPerUs;
    const int64_t t35Ns = timing_.t35.count() * kNsPerUs;
    bool gotAny = false;

    for (;;) {
        const int64_t now = nowNs();
        int pollMs;
        if (!gotAny) {
            const int64_t remain = firstByteTimeoutMs * kNsPerMs - (now - start);
            if (remain <= 0) return out;
            pollMs = int(std::min<int64_t>(remain / kNsPerMs + 1, 10));
        } else {
            pollMs = 1;
        }
        auto r = link_->read(std::span(buf), pollMs);
        if (!r) return err<std::vector<Adu>>(r.error());
        const int64_t after = nowNs();
        if (*r > 0) {
            gotAny = true;
            // Bytes arriving after >t3.5 silence start a NEW frame — the
            // stale pending blob (e.g. a noise burst) must be closed out
            // first, else it fuses with the new frame and kills its CRC.
            if (framer_.pending() > 0 && (after - lastByteNs_) > t35Ns)
                framer_.finishFrame(out);
            framer_.feed(std::span(buf, *r), out);
        }
        if (gotAny) {
            if (*r == 0 && framer_.pending() == 0 && !out.empty())
                return out;          // complete frames + silence → done
            if (*r == 0 && framer_.pending() > 0 &&
                (after - lastByteNs_) > t35Ns) {
                framer_.finishFrame(out);
                return out;
            }
            if (*r > 0) lastByteNs_ = after;
            if ((after - start) > totalTimeoutMs * kNsPerMs) {
                framer_.finishFrame(out);
                return out;
            }
            (void)t15Ns;
        }
    }
}

Expected<Adu> RtuChannel::transact(uint8_t unit,
                                   std::span<const uint8_t> reqPdu,
                                   int timeoutMs) {
    if (!link_->isOpen())
        return err<Adu>(ModbusError::transport("link closed"));
    if (opts_.interFrameDelayUs > 0)
        std::this_thread::sleep_for(
            std::chrono::microseconds(opts_.interFrameDelayUs));

    flush();
    const Frame f = framer_.encode(unit, reqPdu, 0);
    const int writeMs = int(timing_.txTime(f.size()).count() / 1000) + 50;
    if (auto e = link_->write(f, writeMs); !e)
        return err<Adu>(e.error());
    lastByteNs_ = nowNs();
    const uint64_t corruptBefore = framer_.corruptFrames();

    if (opts_.expectEcho) {
        // Wait for + discard our own echo (same byte count).
        size_t got = 0;
        uint8_t tmp[512];
        const int64_t dl = nowNs() + int64_t(timeoutMs) * kNsPerMs;
        while (got < f.size() && nowNs() < dl) {
            auto r = link_->read(std::span(tmp, sizeof(tmp)),
                                 int(std::min<int64_t>(
                                     (dl - nowNs()) / kNsPerMs, 20)));
            if (!r) return err<Adu>(r.error());
            got += *r;
            lastByteNs_ = nowNs();
        }
    }

    // Collect frames until response for our unit (or timeout).
    const int64_t dl = nowNs() + int64_t(timeoutMs) * kNsPerMs;
    for (;;) {
        const int64_t remainMs = (dl - nowNs()) / kNsPerMs;
        if (remainMs <= 0) {
            // Distinguish "data arrived but mangled" (Corrupt — parity/
            // noise worth a re-read) from pure silence (TimedOut).
            if (framer_.corruptFrames() > corruptBefore)
                return err<Adu>(ModbusError::corrupt(
                    "rtu frame crc failed"));
            return err<Adu>(ModbusError::timedOut("rtu response timeout"));
        }
        auto frames = collect(int(remainMs), int(remainMs));
        if (!frames) return err<Adu>(frames.error());
        // Empty = only noise/garbage arrived so far (CRC-dropped at a
        // t3.5 gap). Keep waiting — the response may still be coming;
        // the deadline check above is the only real timeout.
        if (frames->empty()) continue;
        for (auto& a : *frames) {
            if (a.unit == unit || unit == 0) return a;
            // foreign unit → ignore (multi-drop traffic)
        }
    }
}

Expected<void> RtuChannel::sendOnly(uint8_t unit,
                                    std::span<const uint8_t> reqPdu) {
    if (!link_->isOpen())
        return err(ModbusError::transport("link closed"));
    const Frame f = framer_.encode(unit, reqPdu, 0);
    const int writeMs = int(timing_.txTime(f.size()).count() / 1000) + 50;
    return link_->write(f, writeMs);
}

Expected<Adu> RtuChannel::receiveFrame(int timeoutMs) {
    lastByteNs_ = nowNs();
    auto frames = collect(timeoutMs, timeoutMs + 500);
    if (!frames) return err<Adu>(frames.error());
    if (frames->empty())
        return err<Adu>(ModbusError::timedOut("rtu idle"));
    return std::move(frames->front());
}

void RtuChannel::flush() {
    framer_.reset();
    (void)link_->drain();
    (void)link_->flush(Flush::Rx);
}

// ---- MbapChannel ---------------------------------------------------------------------
MbapChannel::MbapChannel(std::unique_ptr<ILink> link, ChannelOpts opts)
    : link_(std::move(link)), opts_(opts) {}

Expected<void> MbapChannel::open() { return link_->open(); }
void MbapChannel::close() { link_->close(); }

Expected<Adu> MbapChannel::receiveAny(int timeoutMs) {
    uint8_t buf[512];
    const int64_t dl = nowNs() + int64_t(timeoutMs) * kNsPerMs;
    while (pending_.empty()) {
        const int64_t remain = (dl - nowNs()) / kNsPerMs;
        if (remain <= 0)
            return err<Adu>(ModbusError::timedOut("mbap response timeout"));
        auto r = link_->read(std::span(buf), int(std::min<int64_t>(remain, 50)));
        if (!r) return err<Adu>(r.error());
        if (*r > 0) {
            std::vector<Adu> out;
            framer_.feed(std::span(buf, *r), out);
            for (auto& a : out) pending_.push_back(std::move(a));
        }
    }
    Adu a = std::move(pending_.front());
    pending_.pop_front();
    return a;
}

Expected<Adu> MbapChannel::transact(uint8_t unit,
                                    std::span<const uint8_t> reqPdu,
                                    int timeoutMs) {
    if (!link_->isOpen())
        return err<Adu>(ModbusError::transport("link closed"));
    const uint16_t txn = nextTxn();
    const Frame f = framer_.encode(unit, reqPdu, txn);
    if (auto e = link_->write(f, timeoutMs); !e)
        return err<Adu>(e.error());
    const int64_t dl = nowNs() + int64_t(timeoutMs) * kNsPerMs;
    for (;;) {
        const int64_t remain = (dl - nowNs()) / kNsPerMs;
        if (remain <= 0)
            return err<Adu>(ModbusError::timedOut("mbap response timeout"));
        auto r = receiveAny(int(remain));
        if (!r) return err<Adu>(r.error());
        if (r->txn == txn) return *r;   // mismatched txn: stray → keep waiting
    }
}

Expected<void> MbapChannel::sendOnly(uint8_t unit,
                                     std::span<const uint8_t> reqPdu) {
    const Frame f = framer_.encode(unit, reqPdu, nextTxn());
    return link_->write(f, 5000);
}

Expected<void> MbapChannel::sendAdu(const Adu& a,
                                    std::span<const uint8_t> pdu) {
    const Frame f = framer_.encode(a.unit, pdu, a.txn);
    return link_->write(f, 5000);
}

void MbapChannel::flush() {
    framer_.reset();
    (void)link_->drain();
}

// ---- ASCII (optional) ------------------------------------------------------------------
#if defined(TETHER_MODBUS_ASCII)
static char hexNib(uint8_t v) { return char(v < 10 ? '0' + v : 'A' + v - 10); }
static int fromHex(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
Frame AsciiFramer::encode(uint8_t unit, std::span<const uint8_t> pdu,
                          uint16_t) {
    Frame f;
    f.push_back(':');
    uint8_t lrc = unit;
    auto put = [&](uint8_t b) {
        f.push_back(uint8_t(hexNib(b >> 4)));
        f.push_back(uint8_t(hexNib(b & 0xF)));
    };
    put(unit);
    for (uint8_t b : pdu) { put(b); lrc = uint8_t(lrc + b); }
    put(uint8_t(-lrc));
    f.push_back('\r'); f.push_back('\n');
    return f;
}
void AsciiFramer::feed(std::span<const uint8_t> bytes,
                       std::vector<Adu>& out) {
    for (uint8_t c : bytes) {
        if (c == ':') { buf_.assign(1, c); continue; }
        if (buf_.empty()) continue;
        if (c == '\r') continue;          // CRLF: decode on '\n' only
        if (c == '\n') {
            if (buf_.size() >= 2 && buf_[0] == ':') {
                // hex-decode payload
                std::vector<uint8_t> raw;
                for (size_t i = 1; i + 1 < buf_.size(); i += 2) {
                    const int hi = fromHex(buf_[i]);
                    const int lo = fromHex(buf_[i + 1]);
                    if (hi < 0 || lo < 0) { raw.clear(); break; }
                    raw.push_back(uint8_t(hi << 4 | lo));
                }
                if (raw.size() >= 3) {
                    uint8_t sum = 0;
                    for (uint8_t b : raw) sum = uint8_t(sum + b);
                    if (sum == 0) {
                        Adu a;
                        a.unit = raw[0];
                        a.pdu.assign(raw.begin() + 1, raw.end() - 1);
                        out.push_back(std::move(a));
                    }
                }
            }
            buf_.clear();
            continue;
        }
        buf_.push_back(c);
        if (buf_.size() > 2 * (kMaxPduSize + 2) + 4) buf_.clear();
    }
}
#endif

} // namespace tether::modbus
